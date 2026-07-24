/**************************************************************************/
/* retarget.hpp                                                          */
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
 * @file retarget.hpp
 * @brief Humanoid clip retargeting and mirroring by bind-pose delta (phase A8).
 *
 * Retargeting moves a clip authored for one rig onto another of different proportions (design
 * §4.4): for each canonical bone both rigs share, it transfers the *delta from the source's bind
 * pose* onto the target's bind pose — `target_bind · (source_bind⁻¹ · source_animated)` — so the
 * joint bends the same, while the target's own bone lengths give its proportions. The root's
 * translation transfers scaled by the hip-height ratio so stride follows scale. Mirroring is the
 * same delta transfer with the left/right bones swapped and the delta reflected across the
 * sagittal plane. Both run at import on raw @ref ClipDesc data, so nothing at runtime pays for it.
 */

#include <cmath>
#include <cstdint>
#include <vector>

#include <SushiEngine/animation/clip_blob.hpp> // ClipDesc
#include <SushiEngine/animation/humanoid.hpp>
#include <SushiEngine/animation/skeleton.hpp>
#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Animation
    {
        namespace detail
        {
            /** @brief Fills a joint→bone lookup (-1 where a joint plays no canonical bone). */
            inline void bone_of_joint(const Avatar& avatar, std::uint32_t joint_count,
                                      std::vector<std::int32_t>& out)
            {
                out.assign(joint_count, -1);
                for (std::uint32_t b = 0; b < HUMAN_BONE_COUNT; ++b)
                {
                    const std::int32_t joint = avatar.joints[b];
                    if (joint >= 0 && static_cast<std::uint32_t>(joint) < joint_count)
                        out[static_cast<std::uint32_t>(joint)] = static_cast<std::int32_t>(b);
                }
            }

            /** @brief The animation delta from a bind rotation: `conjugate(bind) · animated`. */
            inline Quaternionf pose_delta(const Quaternionf& bind, const Quaternionf& animated) noexcept
            {
                return normalize(mul(conjugate(bind), animated));
            }

            /** @brief Seeds a clip's joint tracks to a skeleton's bind pose for every frame. */
            inline void seed_bind_tracks(const SkeletonView& skeleton, ClipDesc& clip)
            {
                const std::size_t elements =
                    static_cast<std::size_t>(clip.frame_count) * clip.joint_count;
                clip.translations.resize(elements);
                clip.rotations.resize(elements);
                clip.scales.resize(elements);
                for (std::uint32_t f = 0; f < clip.frame_count; ++f)
                    for (std::uint32_t j = 0; j < clip.joint_count; ++j)
                    {
                        const std::uint32_t index = f * clip.joint_count + j;
                        clip.translations[index] = skeleton.bind_translations[j];
                        clip.rotations[index] = skeleton.bind_rotations[j];
                        clip.scales[index] = skeleton.bind_scales[j];
                    }
            }
        } // namespace detail

        /**
         * @brief Retargets a humanoid clip from a source rig onto a target rig.
         *
         * Both clips are raw (import-time) data. Joints of the target that play a canonical bone
         * the source also has receive the source's bind-pose delta on top of their own bind; every
         * other target joint holds its bind pose. Morph and generic tracks (skeleton-independent)
         * pass through unchanged.
         *
         * @param source          The clip authored for @p source_skeleton (raw tracks).
         * @param source_avatar   The source rig's bone map.
         * @param source_skeleton The source rig.
         * @param target_avatar   The target rig's bone map.
         * @param target_skeleton The target rig.
         * @param out             Receives the retargeted clip for the target rig; cleared first.
         * @return True on success; false if the source clip is malformed for its skeleton.
         */
        inline bool retarget_clip(const ClipDesc& source, const Avatar& source_avatar,
                                  const SkeletonView& source_skeleton, const Avatar& target_avatar,
                                  const SkeletonView& target_skeleton, ClipDesc& out)
        {
            out = ClipDesc{};
            const std::size_t source_elements =
                static_cast<std::size_t>(source.frame_count) * source.joint_count;
            if (source.joint_count != source_skeleton.joint_count ||
                source.rotations.size() != source_elements || source.frame_count == 0)
                return false;

            out.joint_count = target_skeleton.joint_count;
            out.frame_count = source.frame_count;
            out.sample_rate = source.sample_rate;
            detail::seed_bind_tracks(target_skeleton, out);

            std::vector<std::int32_t> target_bone;
            detail::bone_of_joint(target_avatar, target_skeleton.joint_count, target_bone);

            // Hip-height ratio for root translation scaling.
            float hip_scale = 1.0f;
            if (source_avatar.has(HumanBone::Hips) && target_avatar.has(HumanBone::Hips))
            {
                const std::uint32_t hs = static_cast<std::uint32_t>(source_avatar.joint(HumanBone::Hips));
                const std::uint32_t ht = static_cast<std::uint32_t>(target_avatar.joint(HumanBone::Hips));
                const float source_height = length(source_skeleton.bind_translations[hs]);
                if (source_height > 1e-6f)
                    hip_scale = length(target_skeleton.bind_translations[ht]) / source_height;
            }

            for (std::uint32_t j = 0; j < target_skeleton.joint_count; ++j)
            {
                if (target_bone[j] < 0)
                    continue;
                const HumanBone bone = static_cast<HumanBone>(target_bone[j]);
                if (!source_avatar.has(bone))
                    continue;
                const std::uint32_t s = static_cast<std::uint32_t>(source_avatar.joint(bone));
                if (s >= source.joint_count)
                    continue;

                const Quaternionf source_bind = source_skeleton.bind_rotations[s];
                const Quaternionf target_bind = target_skeleton.bind_rotations[j];
                const bool is_hips = bone == HumanBone::Hips;
                const Vector3f source_bind_t = source_skeleton.bind_translations[s];
                const Vector3f target_bind_t = target_skeleton.bind_translations[j];

                for (std::uint32_t f = 0; f < source.frame_count; ++f)
                {
                    const std::uint32_t src = f * source.joint_count + s;
                    const std::uint32_t dst = f * out.joint_count + j;
                    const Quaternionf delta = detail::pose_delta(source_bind, source.rotations[src]);
                    out.rotations[dst] = normalize(mul(target_bind, delta));
                    if (is_hips)
                    {
                        const Vector3f offset = (source.translations[src] - source_bind_t) * hip_scale;
                        out.translations[dst] = target_bind_t + offset;
                    }
                }
            }

            // Morph and generic tracks are skeleton-independent; carry them through.
            out.morph_names = source.morph_names;
            out.morph_weights = source.morph_weights;
            out.generic_names = source.generic_names;
            out.generic_values = source.generic_values;
            return true;
        }

        /**
         * @brief Mirrors a humanoid clip left-to-right on the same rig.
         *
         * Each joint takes the animation delta of its opposite-side bone, reflected across the
         * sagittal plane (rotation `(x, -y, -z, w)`, lateral translation negated), on top of its
         * own bind pose — so a wave of the left hand becomes a wave of the right.
         *
         * @param source   The clip to mirror (raw tracks for @p skeleton).
         * @param avatar   The rig's bone map (gives each joint's canonical bone and its opposite).
         * @param skeleton The rig.
         * @param out      Receives the mirrored clip; cleared first.
         * @return True on success; false if the source clip is malformed for its skeleton.
         */
        inline bool mirror_clip(const ClipDesc& source, const Avatar& avatar,
                                const SkeletonView& skeleton, ClipDesc& out)
        {
            out = ClipDesc{};
            const std::size_t elements =
                static_cast<std::size_t>(source.frame_count) * source.joint_count;
            if (source.joint_count != skeleton.joint_count || source.rotations.size() != elements ||
                source.frame_count == 0)
                return false;

            out.joint_count = skeleton.joint_count;
            out.frame_count = source.frame_count;
            out.sample_rate = source.sample_rate;
            detail::seed_bind_tracks(skeleton, out);

            std::vector<std::int32_t> joint_bone;
            detail::bone_of_joint(avatar, skeleton.joint_count, joint_bone);

            for (std::uint32_t j = 0; j < skeleton.joint_count; ++j)
            {
                if (joint_bone[j] < 0)
                    continue;
                const HumanBone bone = static_cast<HumanBone>(joint_bone[j]);
                const HumanBone mirror_bone = opposite(bone);
                if (!avatar.has(mirror_bone))
                    continue;
                const std::uint32_t s = static_cast<std::uint32_t>(avatar.joint(mirror_bone));
                if (s >= source.joint_count)
                    continue;

                const Quaternionf source_bind = skeleton.bind_rotations[s];
                const Quaternionf target_bind = skeleton.bind_rotations[j];
                const Vector3f source_bind_t = skeleton.bind_translations[s];
                const Vector3f target_bind_t = skeleton.bind_translations[j];

                for (std::uint32_t f = 0; f < source.frame_count; ++f)
                {
                    const std::uint32_t src = f * source.joint_count + s;
                    const std::uint32_t dst = f * out.joint_count + j;
                    Quaternionf delta = detail::pose_delta(source_bind, source.rotations[src]);
                    const Quaternionf mirrored{delta.x, -delta.y, -delta.z, delta.w};
                    out.rotations[dst] = normalize(mul(target_bind, mirrored));
                    const Vector3f offset = source.translations[src] - source_bind_t;
                    out.translations[dst] =
                        target_bind_t + Vector3f{-offset.x, offset.y, offset.z};
                }
            }

            out.morph_names = source.morph_names;
            out.morph_weights = source.morph_weights;
            out.generic_names = source.generic_names;
            out.generic_values = source.generic_values;
            return true;
        }
    } // namespace Animation
} // namespace SushiEngine
