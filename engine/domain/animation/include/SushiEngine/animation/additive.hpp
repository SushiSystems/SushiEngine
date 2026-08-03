/**************************************************************************/
/* additive.hpp                                                          */
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
 * @file additive.hpp
 * @brief Import-time additive baking: a clip minus a reference pose becomes a delta clip (A5).
 *
 * An additive layer adds a *difference* on top of a base pose — a lean, a recoil, a breath —
 * rather than replacing it. Unity's semantics bake that difference at import against a named
 * reference pose (usually the reference clip's first frame), so the runtime blend is a fused
 * multiply-add and never a per-frame subtraction (design §4.2, kills liability 3's runtime
 * cost). @ref bake_additive_clip produces the delta clip: per joint, rotation
 * `delta = conjugate(reference) * source`, translation `delta = source - reference`, scale
 * `delta = source / reference`. The runtime applies it as `result = base ∘ (delta scaled by
 * weight)` in @ref AnimatorEvaluator. Host / asset-domain code — it never runs on the device.
 */

#include <cstdint>
#include <vector>

#include <SushiEngine/animation/clip_blob.hpp> // ClipDesc
#include <SushiEngine/animation/skeleton.hpp>
#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Animation
    {
        /**
         * @brief Bakes a source clip into an additive delta clip against a reference pose.
         *
         * The reference pose is one frame of a reference clip (Unity's "reference pose" — a T-pose
         * or the idle's first frame). Every source frame becomes a per-joint delta from that pose:
         * a rotation applied *after* the base rotation, a translation added, a scale multiplied.
         * Joints the reference clip does not cover default to identity, so the delta is the source
         * pose itself there.
         *
         * @param source          The clip to make additive; its tracks are frame-major (§4.2).
         * @param reference       The clip whose one frame is the reference pose.
         * @param reference_frame The frame of @p reference to read the reference pose from.
         * @param out             Receives the additive clip (same shape as @p source); cleared first.
         * @return True on success; false if @p source is malformed or @p reference_frame is out of range.
         */
        inline bool bake_additive_clip(const ClipDesc& source, const ClipDesc& reference,
                                       std::uint32_t reference_frame, ClipDesc& out)
        {
            out = ClipDesc{};
            const std::size_t source_elements =
                static_cast<std::size_t>(source.frame_count) * source.joint_count;
            if (source.joint_count == 0 || source.frame_count == 0 ||
                source.translations.size() != source_elements ||
                source.rotations.size() != source_elements || source.scales.size() != source_elements)
                return false;
            if (reference_frame >= reference.frame_count)
                return false;

            out.joint_count = source.joint_count;
            out.frame_count = source.frame_count;
            out.sample_rate = source.sample_rate;
            out.translations.resize(source_elements);
            out.rotations.resize(source_elements);
            out.scales.resize(source_elements);

            const std::uint32_t reference_base = reference_frame * reference.joint_count;
            for (std::uint32_t f = 0; f < source.frame_count; ++f)
            {
                for (std::uint32_t j = 0; j < source.joint_count; ++j)
                {
                    const std::uint32_t index = f * source.joint_count + j;

                    Vector3f reference_translation{0.0f, 0.0f, 0.0f};
                    Quaternionf reference_rotation{0.0f, 0.0f, 0.0f, 1.0f};
                    Vector3f reference_scale{1.0f, 1.0f, 1.0f};
                    if (j < reference.joint_count)
                    {
                        const std::uint32_t reference_index = reference_base + j;
                        reference_translation = reference.translations[reference_index];
                        reference_rotation = reference.rotations[reference_index];
                        reference_scale = reference.scales[reference_index];
                    }

                    const Vector3f& source_translation = source.translations[index];
                    const Quaternionf& source_rotation = source.rotations[index];
                    const Vector3f& source_scale = source.scales[index];

                    out.translations[index] = Vector3f{source_translation.x - reference_translation.x,
                                                       source_translation.y - reference_translation.y,
                                                       source_translation.z - reference_translation.z};
                    out.rotations[index] = normalize(mul(conjugate(reference_rotation), source_rotation));
                    out.scales[index] = Vector3f{
                        reference_scale.x != 0.0f ? source_scale.x / reference_scale.x : source_scale.x,
                        reference_scale.y != 0.0f ? source_scale.y / reference_scale.y : source_scale.y,
                        reference_scale.z != 0.0f ? source_scale.z / reference_scale.z : source_scale.z};
                }
            }
            return true;
        }
    } // namespace Animation
} // namespace SushiEngine
