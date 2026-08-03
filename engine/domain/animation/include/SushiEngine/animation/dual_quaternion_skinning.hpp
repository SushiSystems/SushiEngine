/**************************************************************************/
/* dual_quaternion_skinning.hpp                                          */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/* Licensed under the Apache License, Version 2.0 (the "License");        */
/* you may not use this file except in compliance with the License.       */
/* You may obtain a copy of the License at                                */
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                        */
/**************************************************************************/

#pragma once

/**
 * @file dual_quaternion_skinning.hpp
 * @brief Dual-quaternion blending (design §12.4): the candy-wrapper fix for `SkinningPass`.
 *
 * Named in §12.4 as unbuilt: "Linear-blend only; twist joints (shoulders, wrists) will show
 * candy-wrapper artifacts under large rotation. `SkinningPass` is structured so this is a
 * specialization constant, not a redesign — but it's still unbuilt." This header is that
 * blend algorithm, in isolation from the renderer: a per-joint rigid transform (rotation +
 * translation, the shape a skin matrix already is when it carries no scale) represented as a
 * @ref DualQuaternion, blended by the classic Kavan-Collins-Zara-O'Sullivan
 * "Geometric Skinning with Approximate Dual Quaternion Blending" (2007) construction —
 * weighted-sum-then-normalize, with a hemisphere correction against a reference so
 * antipodal quaternion representations of the same rotation don't cancel — and applied to a
 * vertex with the direct (translation-free-extraction) formula from the same paper.
 *
 * @ref skin_position_lbs is the reference this exists to fix: linear-blending two rigid
 * transforms independently (translation lerped, rotation nlerped) is *not* the same rigid
 * motion as blending the transforms themselves — under a large bend, the blended result
 * pulls the surface in toward the joint (the classic "candy wrapper" pinch, provable and
 * measured by @c dual_quaternion_skinning_demo). Dual-quaternion blending does not have this
 * problem for a single bend about a shared pivot: two dual quaternions with the same
 * translation-of-pivot component blend to a result whose *distance from the pivot* is
 * preserved, because the blend genuinely is nlerp of the rotation with the translation
 * carried along correctly, not blended independently of it.
 *
 * What this header does not attempt: this is the CPU-verifiable blend math only. Wiring it
 * into `skinning.comp`/`SkinningPass` needs a second, parallel palette representation
 * (dual quaternions, not `JointMatrix`) threaded through the evaluator → palette → GPU
 * buffer chain — real plumbing, deliberately not attempted blind here, since none of it can
 * be visually verified without a GPU display (the same constraint the original morph-target
 * shader work was built under before the user's own visual confirmation closed it out).
 */

#include <cmath>
#include <cstdint>

#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Animation
    {
        /**
         * @brief A unit dual quaternion: a rigid rotation+translation in eight floats.
         *
         * `real` is the rotation; `dual` encodes the translation as `0.5 * translation_quat *
         * real` (the standard Clifford-algebra dual-quaternion construction). Both parts use
         * evaluation precision (float), matching @ref Quaternionf and the skin palette.
         */
        struct DualQuaternion
        {
            Quaternionf real{0.0f, 0.0f, 0.0f, 1.0f};
            Quaternionf dual{0.0f, 0.0f, 0.0f, 0.0f};
        };

        namespace Detail
        {
            /** @brief Component-wise quaternion scale (no arithmetic operators on QuaternionT). */
            inline Quaternionf quat_scale(const Quaternionf& q, float s) noexcept
            {
                return Quaternionf{q.x * s, q.y * s, q.z * s, q.w * s};
            }

            /** @brief Component-wise quaternion sum. */
            inline Quaternionf quat_add(const Quaternionf& a, const Quaternionf& b) noexcept
            {
                return Quaternionf{a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w};
            }

            /** @brief The quaternion's Euclidean norm, treating it as a plain 4-vector. */
            inline float quat_norm(const Quaternionf& q) noexcept
            {
                return std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
            }

            /** @brief The plain 4-vector dot product (not the rotation-composition product). */
            inline float quat_dot4(const Quaternionf& a, const Quaternionf& b) noexcept
            {
                return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
            }
        } // namespace Detail

        /**
         * @brief Builds a unit dual quaternion from a rigid rotation and translation.
         * @param rotation    A unit rotation.
         * @param translation The rigid transform's translation.
         * @return The equivalent dual quaternion.
         */
        inline DualQuaternion dual_quaternion_from_rigid(const Quaternionf& rotation,
                                                          const Vector3f& translation) noexcept
        {
            DualQuaternion dq;
            dq.real = normalize(rotation);
            const Quaternionf translation_quat{translation.x, translation.y, translation.z, 0.0f};
            dq.dual = Detail::quat_scale(mul(translation_quat, dq.real), 0.5f);
            return dq;
        }

        /**
         * @brief Weighted dual-quaternion blend (Kavan et al. 2007's DLB construction).
         *
         * Weighted-sums both parts against a hemisphere-corrected reference (the first
         * nonzero-weight input; every later input is negated in full if it points into the
         * opposite hemisphere from the reference — the same correction @ref nlerp does for
         * plain rotations, necessary because @c q and @c -q represent the same rotation but
         * would otherwise partially cancel in a naive weighted sum), then re-normalizes onto
         * the unit dual-quaternion manifold.
         *
         * @param dual_quaternions Per-joint rigid transforms, one per skin influence.
         * @param weights          Parallel skin weights (need not sum to 1; renormalized here).
         * @param count            Influences in both arrays (skinning uses up to 4).
         * @return The blended unit dual quaternion, or the identity if every weight is <= 0.
         */
        inline DualQuaternion blend_dual_quaternions(const DualQuaternion* dual_quaternions,
                                                      const float* weights,
                                                      std::uint32_t count) noexcept
        {
            Quaternionf reference{0.0f, 0.0f, 0.0f, 1.0f};
            bool have_reference = false;
            for (std::uint32_t i = 0; i < count; ++i)
            {
                if (weights[i] <= 0.0f)
                    continue;
                if (!have_reference)
                {
                    reference = dual_quaternions[i].real;
                    have_reference = true;
                }
            }
            if (!have_reference)
                return DualQuaternion{};

            Quaternionf real_sum{0.0f, 0.0f, 0.0f, 0.0f};
            Quaternionf dual_sum{0.0f, 0.0f, 0.0f, 0.0f};
            for (std::uint32_t i = 0; i < count; ++i)
            {
                if (weights[i] <= 0.0f)
                    continue;
                const float sign = Detail::quat_dot4(reference, dual_quaternions[i].real) < 0.0f
                                       ? -1.0f
                                       : 1.0f;
                const float w = weights[i] * sign;
                real_sum = Detail::quat_add(real_sum, Detail::quat_scale(dual_quaternions[i].real, w));
                dual_sum = Detail::quat_add(dual_sum, Detail::quat_scale(dual_quaternions[i].dual, w));
            }

            const float norm = Detail::quat_norm(real_sum);
            if (norm <= 1e-8f)
                return DualQuaternion{};
            const float inv_norm = 1.0f / norm;
            const Quaternionf real_n = Detail::quat_scale(real_sum, inv_norm);
            // Project the dual part onto the tangent space of the normalized real part —
            // dividing by the norm alone (as if this were a plain quaternion) would leave the
            // result off the dual-quaternion unit manifold; Kavan et al. eq. 23-24.
            const float projection = Detail::quat_dot4(real_n, dual_sum) * inv_norm;
            const Quaternionf dual_n = Detail::quat_add(
                Detail::quat_scale(dual_sum, inv_norm), Detail::quat_scale(real_n, -projection));

            DualQuaternion result;
            result.real = real_n;
            result.dual = dual_n;
            return result;
        }

        /**
         * @brief Applies a unit dual quaternion to a position (Kavan et al. eq. 15-16).
         *
         * Transforms directly, without extracting a separate rotation+translation pair first
         * (equivalent to that, but the form actually used in a DQS vertex shader).
         *
         * @param dq       A unit (normalized) dual quaternion, e.g. from @ref blend_dual_quaternions.
         * @param position The vertex position in the skin's rest (bind) space.
         * @return The skinned position.
         */
        inline Vector3f skin_position_dqs(const DualQuaternion& dq, const Vector3f& position) noexcept
        {
            const Vector3f rotated = rotate(dq.real, position);
            const Vector3f real_v{dq.real.x, dq.real.y, dq.real.z};
            const Vector3f dual_v{dq.dual.x, dq.dual.y, dq.dual.z};
            const Vector3f translation =
                (dual_v * dq.real.w - real_v * dq.dual.w + cross(real_v, dual_v)) * 2.0f;
            return rotated + translation;
        }

        /**
         * @brief The linear-blend reference this header exists to improve on.
         *
         * Linear blend skinning as `skinning.comp` performs it: the vertex is transformed by
         * **each** influence and the *results* are weight-averaged — algebraically the same as
         * averaging the joint matrices and transforming once, which is what the shader's
         * `mat4`-weighted sum does. This is the candy-wrapper baseline
         * @c dual_quaternion_skinning_demo measures @ref skin_position_dqs against.
         *
         * The averaging happens in *position* space and that is the whole point: the average of
         * two rigid transforms is not a rigid transform, so a vertex under two influences whose
         * rotations differ widely is pulled toward the line between their two images — the
         * collapse @ref skin_position_dqs exists to avoid. An earlier revision blended the
         * rotation with `nlerp` and the translation with `lerp` instead, and documented that as
         * equivalent to the shader's matrix sum. It is not: `nlerp` yields a genuine rotation, so
         * that form preserves the vertex's distance from the twist axis exactly and exhibits no
         * candy-wrapper artifact at all. Measuring dual-quaternion skinning against it therefore
         * compared the new path to a baseline without the defect the new path removes.
         *
         * @param rotations    Per-joint rotations, one per skin influence.
         * @param translations Per-joint translations, one per skin influence.
         * @param weights      Parallel skin weights (need not sum to 1; renormalized here).
         * @param count        Influences in all three arrays.
         * @param position     The vertex position in the skin's rest (bind) space.
         * @return The skinned position, or @p position when no influence carries weight.
         */
        inline Vector3f skin_position_lbs(const Quaternionf* rotations, const Vector3f* translations,
                                          const float* weights, std::uint32_t count,
                                          const Vector3f& position) noexcept
        {
            // No hemisphere correction here, and none is needed: nothing sums quaternions. Each
            // influence's rotation is applied on its own, and `q` and `-q` rotate a vector
            // identically, so the representation's double cover never reaches the result.
            float total_weight = 0.0f;
            Vector3f sum{0.0f, 0.0f, 0.0f};
            for (std::uint32_t i = 0; i < count; ++i)
            {
                if (weights[i] <= 0.0f)
                    continue;
                const Vector3f transformed = rotate(rotations[i], position) + translations[i];
                sum = sum + transformed * weights[i];
                total_weight += weights[i];
            }
            if (total_weight <= 1e-8f)
                return position;
            return sum * (1.0f / total_weight);
        }
    } // namespace Animation
} // namespace SushiEngine
