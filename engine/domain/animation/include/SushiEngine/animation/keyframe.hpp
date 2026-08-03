/**************************************************************************/
/* keyframe.hpp                                                          */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

#pragma once

/**
 * @file keyframe.hpp
 * @brief Sparse keyframe curves and pose recording — the animation authoring model.
 *
 * The runtime clip is *dense* (uniform frames, resampled and ACL-compressed for playback); this
 * is its editable counterpart. A @ref ScalarCurve is a sorted list of keys the dope sheet / curve
 * editor manipulates, with constant / linear / cubic-Hermite interpolation (Catmull-Rom
 * auto-tangents), and a @ref QuaternionCurve slerps rotation keys. A @ref ClipAuthoring is the
 * per-joint (and per-morph / per-generic) bundle of curves; @ref ClipAuthoring::bake resamples it
 * to the dense @ref ClipDesc the cook consumes. @ref PoseRecorder captures a live pose over time
 * into keys — the "record" workflow — so a rig posed by hand, IK, or physics can be turned into a
 * clip. Nothing here runs at runtime; it feeds the import/cook path.
 */

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <SushiEngine/animation/clip_blob.hpp> // ClipDesc
#include <SushiEngine/animation/hash.hpp>
#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Animation
    {
        /** @brief How a scalar curve interpolates between its keys. */
        enum class InterpolationMode : std::uint8_t
        {
            Constant = 0, /**< Step: hold the earlier key's value until the next. */
            Linear = 1,   /**< Straight-line between keys. */
            Cubic = 2     /**< Cubic Hermite with per-key tangents (Catmull-Rom if auto). */
        };

        /** @brief One scalar key: its time, value, and cubic in/out tangents (value per second). */
        struct ScalarKey
        {
            float time = 0.0f;
            float value = 0.0f;
            float in_tangent = 0.0f;
            float out_tangent = 0.0f;
        };

        /**
         * @brief A sparse scalar animation curve — the dope-sheet/curve-editor primitive.
         *
         * Keys are kept sorted by time. @ref evaluate clamps to the ends (holds the first/last
         * value outside the range). Cubic mode Hermite-interpolates with each key's tangents;
         * @ref auto_tangents fills them Catmull-Rom for smooth motion through the keys.
         */
        struct ScalarCurve
        {
            std::vector<ScalarKey> keys;
            InterpolationMode mode = InterpolationMode::Linear;

            /** @brief Whether the curve carries any keys. */
            bool empty() const noexcept { return keys.empty(); }

            /** @brief The time of the last key (0 if empty). */
            float duration() const noexcept { return keys.empty() ? 0.0f : keys.back().time; }

            /**
             * @brief Inserts a key at a time, replacing an existing key at (nearly) that time.
             * @param time  The key's time.
             * @param value The key's value.
             * @return The index of the inserted or replaced key.
             */
            std::size_t insert(float time, float value)
            {
                for (std::size_t i = 0; i < keys.size(); ++i)
                {
                    if (std::fabs(keys[i].time - time) < 1e-6f)
                    {
                        keys[i].value = value;
                        return i;
                    }
                    if (keys[i].time > time)
                    {
                        keys.insert(keys.begin() + static_cast<std::ptrdiff_t>(i),
                                    ScalarKey{time, value, 0.0f, 0.0f});
                        return i;
                    }
                }
                keys.push_back(ScalarKey{time, value, 0.0f, 0.0f});
                return keys.size() - 1;
            }

            /** @brief Removes the key at (nearly) a time, if one exists. */
            void remove_at(float time)
            {
                for (std::size_t i = 0; i < keys.size(); ++i)
                    if (std::fabs(keys[i].time - time) < 1e-6f)
                    {
                        keys.erase(keys.begin() + static_cast<std::ptrdiff_t>(i));
                        return;
                    }
            }

            /** @brief Fills every key's tangents Catmull-Rom (finite differences over neighbours). */
            void auto_tangents() noexcept
            {
                const std::size_t n = keys.size();
                for (std::size_t i = 0; i < n; ++i)
                {
                    float tangent = 0.0f;
                    if (n >= 2)
                    {
                        if (i == 0)
                            tangent = (keys[1].value - keys[0].value) /
                                      std::max(1e-6f, keys[1].time - keys[0].time);
                        else if (i == n - 1)
                            tangent = (keys[n - 1].value - keys[n - 2].value) /
                                      std::max(1e-6f, keys[n - 1].time - keys[n - 2].time);
                        else
                            tangent = (keys[i + 1].value - keys[i - 1].value) /
                                      std::max(1e-6f, keys[i + 1].time - keys[i - 1].time);
                    }
                    keys[i].in_tangent = tangent;
                    keys[i].out_tangent = tangent;
                }
            }

            /**
             * @brief Evaluates the curve at a time.
             * @param time    The time to sample.
             * @param fallback The value returned when the curve is empty.
             * @return The interpolated value.
             */
            float evaluate(float time, float fallback = 0.0f) const noexcept
            {
                if (keys.empty())
                    return fallback;
                if (time <= keys.front().time)
                    return keys.front().value;
                if (time >= keys.back().time)
                    return keys.back().value;
                std::size_t i = 0;
                while (i + 1 < keys.size() && keys[i + 1].time <= time)
                    ++i;
                const ScalarKey& a = keys[i];
                const ScalarKey& b = keys[i + 1];
                const float span = b.time - a.time;
                if (span < 1e-8f)
                    return b.value;
                const float u = (time - a.time) / span;
                if (mode == InterpolationMode::Constant)
                    return a.value;
                if (mode == InterpolationMode::Linear)
                    return a.value + (b.value - a.value) * u;
                // Cubic Hermite: tangents are value/second, so scale by the segment span.
                const float m0 = a.out_tangent * span;
                const float m1 = b.in_tangent * span;
                const float u2 = u * u;
                const float u3 = u2 * u;
                const float h00 = 2.0f * u3 - 3.0f * u2 + 1.0f;
                const float h10 = u3 - 2.0f * u2 + u;
                const float h01 = -2.0f * u3 + 3.0f * u2;
                const float h11 = u3 - u2;
                return h00 * a.value + h10 * m0 + h01 * b.value + h11 * m1;
            }
        };

        /** @brief One rotation key: its time and unit quaternion. */
        struct QuaternionKey
        {
            float time = 0.0f;
            Quaternionf value{0.0f, 0.0f, 0.0f, 1.0f};
        };

        /** @brief A sparse rotation curve — slerps between its keys, clamps at the ends. */
        struct QuaternionCurve
        {
            std::vector<QuaternionKey> keys;

            bool empty() const noexcept { return keys.empty(); }
            float duration() const noexcept { return keys.empty() ? 0.0f : keys.back().time; }

            /** @brief Inserts a rotation key, replacing one at (nearly) the same time. */
            void insert(float time, const Quaternionf& value)
            {
                for (std::size_t i = 0; i < keys.size(); ++i)
                {
                    if (std::fabs(keys[i].time - time) < 1e-6f)
                    {
                        keys[i].value = value;
                        return;
                    }
                    if (keys[i].time > time)
                    {
                        keys.insert(keys.begin() + static_cast<std::ptrdiff_t>(i),
                                    QuaternionKey{time, value});
                        return;
                    }
                }
                keys.push_back(QuaternionKey{time, value});
            }

            /** @brief Removes the key at (nearly) a time, if one exists. */
            void remove_at(float time)
            {
                for (std::size_t i = 0; i < keys.size(); ++i)
                    if (std::fabs(keys[i].time - time) < 1e-6f)
                    {
                        keys.erase(keys.begin() + static_cast<std::ptrdiff_t>(i));
                        return;
                    }
            }

            /** @brief Evaluates the rotation at a time (slerp), or @p fallback when empty. */
            Quaternionf evaluate(float time, const Quaternionf& fallback = Quaternionf{0, 0, 0, 1}) const
                noexcept
            {
                if (keys.empty())
                    return fallback;
                if (time <= keys.front().time)
                    return keys.front().value;
                if (time >= keys.back().time)
                    return keys.back().value;
                std::size_t i = 0;
                while (i + 1 < keys.size() && keys[i + 1].time <= time)
                    ++i;
                const QuaternionKey& a = keys[i];
                const QuaternionKey& b = keys[i + 1];
                const float span = b.time - a.time;
                if (span < 1e-8f)
                    return b.value;
                return slerp(a.value, b.value, (time - a.time) / span);
            }
        };

        /** @brief One joint's translation / rotation / scale curves, plus its rest default. */
        struct JointChannels
        {
            NameHash joint_name = 0;
            ScalarCurve translation_x, translation_y, translation_z;
            QuaternionCurve rotation;
            ScalarCurve scale_x, scale_y, scale_z;
            Vector3f default_translation{0.0f, 0.0f, 0.0f};
            Quaternionf default_rotation{0.0f, 0.0f, 0.0f, 1.0f};
            Vector3f default_scale{1.0f, 1.0f, 1.0f};

            /** @brief The translation at a time (per-axis curves, falling back to the default). */
            Vector3f translation_at(float time) const noexcept
            {
                return Vector3f{translation_x.evaluate(time, default_translation.x),
                                translation_y.evaluate(time, default_translation.y),
                                translation_z.evaluate(time, default_translation.z)};
            }
            /** @brief The scale at a time. */
            Vector3f scale_at(float time) const noexcept
            {
                return Vector3f{scale_x.evaluate(time, default_scale.x),
                                scale_y.evaluate(time, default_scale.y),
                                scale_z.evaluate(time, default_scale.z)};
            }
            /** @brief The rotation at a time. */
            Quaternionf rotation_at(float time) const noexcept
            {
                return rotation.evaluate(time, default_rotation);
            }
        };

        /** @brief One named scalar property curve (morph target weight or generic property). */
        struct NamedCurve
        {
            std::string name; /**< The property/target name (hashed by the cook). */
            ScalarCurve curve;
        };

        /**
         * @brief The editable clip: per-joint curves plus morph/generic curves, baked to a clip.
         *
         * The dope sheet and curve editor manipulate this; @ref bake resamples it to the dense
         * @ref ClipDesc the `.sushianim` cook consumes. Joints keep a rest default so a channel
         * with no keys still poses correctly.
         */
        struct ClipAuthoring
        {
            std::vector<JointChannels> joints;
            std::vector<NamedCurve> morphs;
            std::vector<NamedCurve> generics;

            /** @brief The longest key time across every curve — the clip's authored length. */
            float duration() const noexcept
            {
                float longest = 0.0f;
                for (const JointChannels& joint : joints)
                {
                    longest = std::max(longest, joint.translation_x.duration());
                    longest = std::max(longest, joint.translation_y.duration());
                    longest = std::max(longest, joint.translation_z.duration());
                    longest = std::max(longest, joint.rotation.duration());
                    longest = std::max(longest, joint.scale_x.duration());
                    longest = std::max(longest, joint.scale_y.duration());
                    longest = std::max(longest, joint.scale_z.duration());
                }
                for (const NamedCurve& morph : morphs)
                    longest = std::max(longest, morph.curve.duration());
                for (const NamedCurve& generic : generics)
                    longest = std::max(longest, generic.curve.duration());
                return longest;
            }

            /**
             * @brief Resamples the curves to a dense clip at a uniform rate.
             * @param sample_rate The frames-per-second the dense clip is baked at.
             * @param out         Receives the dense clip (frame-major); cleared first.
             * @return True on success; false if there are no joints or the rate is non-positive.
             */
            bool bake(float sample_rate, ClipDesc& out) const
            {
                out = ClipDesc{};
                if (joints.empty() || sample_rate <= 0.0f)
                    return false;

                const float length = duration();
                const std::uint32_t frame_count =
                    length <= 0.0f ? 1u
                                   : static_cast<std::uint32_t>(std::lround(length * sample_rate)) + 1u;
                const std::uint32_t joint_count = static_cast<std::uint32_t>(joints.size());

                out.joint_count = joint_count;
                out.frame_count = frame_count;
                out.sample_rate = sample_rate;
                out.translations.resize(static_cast<std::size_t>(frame_count) * joint_count);
                out.rotations.resize(static_cast<std::size_t>(frame_count) * joint_count);
                out.scales.resize(static_cast<std::size_t>(frame_count) * joint_count);

                for (std::uint32_t f = 0; f < frame_count; ++f)
                {
                    const float time = static_cast<float>(f) / sample_rate;
                    for (std::uint32_t j = 0; j < joint_count; ++j)
                    {
                        const std::uint32_t index = f * joint_count + j;
                        out.translations[index] = joints[j].translation_at(time);
                        out.rotations[index] = normalize(joints[j].rotation_at(time));
                        out.scales[index] = joints[j].scale_at(time);
                    }
                }

                bake_named(morphs, frame_count, sample_rate, out.morph_names, out.morph_weights);
                bake_named(generics, frame_count, sample_rate, out.generic_names, out.generic_values);
                return true;
            }

        private:
            /** @brief Bakes a set of named scalar curves into a frame-major track set + names. */
            static void bake_named(const std::vector<NamedCurve>& curves, std::uint32_t frame_count,
                                   float sample_rate, std::vector<std::string>& out_names,
                                   std::vector<float>& out_values)
            {
                if (curves.empty())
                    return;
                out_names.resize(curves.size());
                for (std::size_t c = 0; c < curves.size(); ++c)
                    out_names[c] = curves[c].name;
                out_values.resize(static_cast<std::size_t>(frame_count) * curves.size());
                for (std::uint32_t f = 0; f < frame_count; ++f)
                {
                    const float time = static_cast<float>(f) / sample_rate;
                    for (std::size_t c = 0; c < curves.size(); ++c)
                        out_values[f * curves.size() + c] = curves[c].curve.evaluate(time);
                }
            }
        };

        /**
         * @brief Captures a live pose over time into keyframes — the "record" workflow.
         *
         * Point it at a @ref ClipAuthoring sized to a skeleton, then call @ref record_pose each
         * time the pose should be sampled (every scrub step, every fixed tick of a physics ragdoll,
         * every gizmo release). Each call inserts one key per channel at that time, so scrubbing a
         * hand-posed or IK-driven rig turns it into an editable clip.
         */
        class PoseRecorder
        {
            public:
                /**
                 * @brief Points the recorder at a clip and sizes it to a skeleton's joints.
                 * @param clip        The authoring clip to record into (its keys are appended to).
                 * @param joint_names One name hash per joint (drives channel identity), or null.
                 * @param joint_count Joints to record.
                 */
                void begin(ClipAuthoring& clip, const NameHash* joint_names, std::uint32_t joint_count)
                {
                    clip_ = &clip;
                    if (clip_->joints.size() != joint_count)
                        clip_->joints.assign(joint_count, JointChannels{});
                    if (joint_names != nullptr)
                        for (std::uint32_t j = 0; j < joint_count; ++j)
                            clip_->joints[j].joint_name = joint_names[j];
                }

                /**
                 * @brief Inserts one key per channel from a local pose at a time.
                 * @param time         The time to key at (seconds).
                 * @param translations Per-joint local translation.
                 * @param rotations    Per-joint local rotation.
                 * @param scales       Per-joint local scale.
                 * @param joint_count  Joints in the arrays.
                 */
                void record_pose(float time, const Vector3f* translations,
                                 const Quaternionf* rotations, const Vector3f* scales,
                                 std::uint32_t joint_count)
                {
                    if (clip_ == nullptr)
                        return;
                    if (clip_->joints.size() < joint_count)
                        clip_->joints.resize(joint_count);
                    for (std::uint32_t j = 0; j < joint_count; ++j)
                    {
                        JointChannels& channels = clip_->joints[j];
                        channels.translation_x.insert(time, translations[j].x);
                        channels.translation_y.insert(time, translations[j].y);
                        channels.translation_z.insert(time, translations[j].z);
                        channels.rotation.insert(time, rotations[j]);
                        channels.scale_x.insert(time, scales[j].x);
                        channels.scale_y.insert(time, scales[j].y);
                        channels.scale_z.insert(time, scales[j].z);
                    }
                }

                /** @brief The clip being recorded into, or null before @ref begin. */
                ClipAuthoring* clip() const noexcept { return clip_; }

            private:
                ClipAuthoring* clip_ = nullptr;
        };
    } // namespace Animation
} // namespace SushiEngine
