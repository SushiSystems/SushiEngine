/**************************************************************************/
/* pose_modifier.hpp                                                     */
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
 * @file pose_modifier.hpp
 * @brief The IK / pose-modifier seam: an ordered stack run in model space (phase A6).
 *
 * After the layers blend and the pose composes to model space, an ordered stack of
 * @ref IPoseModifier corrects the *final* animated pose (design §5.3, Unity's pass ordering) —
 * so IK reaches for a target the blend produced, not one it assumed. A modifier reads the
 * model-space matrices, edits the local pose of the joints it owns, and re-composes; the
 * evaluator runs the stack between compose and palette build. This is the OCP seam for new
 * solvers: a solver is a new @ref IPoseModifier, not a change to the evaluator. Ray-driven
 * solvers (foot placement) reach the world through @ref IPoseTaskContext, sampled once at
 * extract so the solver stays a pure function of its inputs.
 */

#include <cstddef>
#include <cstdint>

#include <SushiEngine/animation/skeleton.hpp>
#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Animation
    {
        /**
         * @brief Composes model-space matrices from a local pose by the topological forward scan.
         * @param skeleton The rig (its parent array drives the scan; `parents[i] < i`).
         * @param local_translations Per-joint local translation.
         * @param local_rotations    Per-joint local rotation.
         * @param local_scales       Per-joint local scale.
         * @param model              Receives @c skeleton.joint_count model-space matrices.
         */
        inline void compose_model(const SkeletonView& skeleton, const Vector3f* local_translations,
                                  const Quaternionf* local_rotations, const Vector3f* local_scales,
                                  Mat4* model) noexcept
        {
            for (std::uint32_t i = 0; i < skeleton.joint_count; ++i)
            {
                const Vector3f& t = local_translations[i];
                const Quaternionf& r = local_rotations[i];
                const Vector3f& s = local_scales[i];
                const Mat4 local = compose_transform(Vector3{t.x, t.y, t.z},
                                                     Quaternion{r.x, r.y, r.z, r.w},
                                                     Vector3{s.x, s.y, s.z});
                model[i] = skeleton.parents[i] == NO_PARENT
                               ? local
                               : mul(model[skeleton.parents[i]], local);
            }
        }

        /**
         * @brief The mutable pose one modifier operates on: the local pose plus its model space.
         *
         * A modifier reads @ref model (object space, valid on entry), edits @ref local_rotations
         * (and, rarely, translations) of the joints it owns, and calls @ref recompose so later
         * reads — its own or the next modifier's — see the updated model space. All arrays are
         * @c skeleton.joint_count long and caller-owned (the evaluator's buffers).
         */
        struct PoseModifierContext
        {
            SkeletonView skeleton;                 /**< The rig being posed. */
            Vector3f* local_translations = nullptr; /**< Per-joint local translation (mutable). */
            Quaternionf* local_rotations = nullptr; /**< Per-joint local rotation (mutable). */
            Vector3f* local_scales = nullptr;       /**< Per-joint local scale (mutable). */
            Mat4* model = nullptr;                  /**< Per-joint model-space matrix (object space). */

            /** @brief Re-composes @ref model from the current local pose. */
            void recompose() noexcept
            {
                compose_model(skeleton, local_translations, local_rotations, local_scales, model);
            }

            /** @brief The object-space position of a joint (its model matrix's translation). */
            Vector3 position(std::uint32_t joint) const noexcept
            {
                return Vector3{model[joint].m[12], model[joint].m[13], model[joint].m[14]};
            }

            /** @brief The object-space rotation of a joint. */
            Quaternion rotation(std::uint32_t joint) const noexcept
            {
                return quaternion_from_matrix(model[joint]);
            }

            /**
             * @brief Sets a joint's object-space rotation by editing its local rotation.
             *
             * Converts the desired model-space rotation to local against the parent's current
             * model rotation, so after @ref recompose the joint carries @p world exactly (the
             * parent being unchanged this step).
             *
             * @param joint The joint to orient.
             * @param world The desired object-space rotation.
             */
            void set_rotation(std::uint32_t joint, const Quaternion& world) noexcept
            {
                Quaternion parent{0.0, 0.0, 0.0, 1.0};
                if (skeleton.parents[joint] != NO_PARENT)
                    parent = quaternion_from_matrix(model[skeleton.parents[joint]]);
                const Quaternion local = mul(conjugate(parent), world);
                local_rotations[joint] = Quaternionf{static_cast<float>(local.x),
                                                     static_cast<float>(local.y),
                                                     static_cast<float>(local.z),
                                                     static_cast<float>(local.w)};
            }
        };

        /**
         * @brief One entry in the ordered pose-modifier stack (an IK solver, a look-at, ...).
         *
         * Stateless configuration: a solver holds its joint indices, target, and weight, and
         * @ref solve is const — the mutable state is the pose in the @ref PoseModifierContext.
         */
        class IPoseModifier
        {
            public:
                virtual ~IPoseModifier() = default;

                /**
                 * @brief Corrects the model-space pose in place.
                 * @param context The pose to edit (model space valid on entry).
                 */
                virtual void solve(PoseModifierContext& context) const = 0;
        };

        /**
         * @brief The seam a ray-driven solver (foot placement) reaches the world through.
         *
         * Sampled at extract (one-frame latency, design §5.3) so the solver is a pure function of
         * its inputs and never touches the physics world directly. The default provides no ground.
         */
        class IPoseTaskContext
        {
            public:
                virtual ~IPoseTaskContext() = default;

                /**
                 * @brief Casts a ray and reports the nearest hit, in object space.
                 * @param origin        Ray origin (object space).
                 * @param direction     Ray direction (need not be normalised).
                 * @param out_hit_point Receives the hit position on success.
                 * @param out_normal    Receives the surface normal (unit) on success.
                 * @return True if the ray hit a surface.
                 */
                virtual bool raycast(const Vector3& origin, const Vector3& direction,
                                     Vector3& out_hit_point, Vector3& out_normal) const = 0;
        };

        namespace detail
        {
            /**
             * @brief The shortest-arc rotation taking unit vector @p from onto unit vector @p to.
             * @param from A unit vector.
             * @param to   A unit vector.
             * @return The rotation @c q with `rotate(q, from) == to`.
             */
            inline Quaternion rotation_between(const Vector3& from, const Vector3& to) noexcept
            {
                const Scalar d = dot(from, to);
                if (d >= static_cast<Scalar>(0.999999))
                    return Quaternion{0.0, 0.0, 0.0, 1.0};
                if (d <= static_cast<Scalar>(-0.999999))
                {
                    // Antiparallel: rotate 180 deg about any axis perpendicular to `from`.
                    Vector3 axis = cross(Vector3{1.0, 0.0, 0.0}, from);
                    if (length(axis) < static_cast<Scalar>(1e-6))
                        axis = cross(Vector3{0.0, 1.0, 0.0}, from);
                    axis = normalize(axis);
                    return Quaternion{axis.x, axis.y, axis.z, 0.0};
                }
                const Vector3 axis = cross(from, to);
                Quaternion q{axis.x, axis.y, axis.z, static_cast<Scalar>(1.0) + d};
                return normalize(q);
            }
        } // namespace detail
    } // namespace Animation
} // namespace SushiEngine
