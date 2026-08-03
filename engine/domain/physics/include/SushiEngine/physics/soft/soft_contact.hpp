/**************************************************************************/
/* soft_contact.hpp                                                       */
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
/* permissions and limitations under the License.                         */
/**************************************************************************/

#pragma once

/**
 * @file soft_contact.hpp
 * @brief One contact between two pieces of deformable surface, and its projection.
 *
 * A rigid contact is a claim about two *bodies*, so `ContactManifold` names two
 * of them and the projection splits every correction between two poses. A
 * deformable contact is a claim about two *points that are not bodies*: a
 * vertex against a triangle is one particle against three, and an edge against
 * an edge is two against two. Both are the same statement once written down —
 * two weighted combinations of particles must not approach closer than a
 * distance along a normal — so both are this one type, and neither needs a
 * manifold.
 *
 * The unified form is a single weight per particle, signed by which side the
 * particle is on:
 *
 *     C = dot(sum_i weight_i * position_i, normal) - rest_distance
 *
 * with `weight` negative on the side the normal points away from and positive
 * on the side it points toward, each side's magnitudes summing to one. The
 * constraint's gradient with respect to particle `i` is then simply
 * `weight_i * normal`, which is what makes one projection serve both cases —
 * and what makes a corner-weighted correction fall out rather than being
 * special-cased: a triangle corner that carries a tenth of the contact takes a
 * tenth of the push.
 *
 * **Where the particles live is not this file's business.** A self-collision
 * names four particles of one body; a contact between two bodies names some of
 * each. Rather than write the projection twice, or index into a stitched-
 * together array that would have to be maintained, the projection reaches its
 * particles through a *source* — @ref SoftParticleArray for one body,
 * @ref SoftParticlePair for two — resolved at compile time, so the indirection
 * costs nothing and adding a third arrangement later costs one overload rather
 * than one more copy of the arithmetic.
 *
 * Friction and restitution follow `solver/contact_projection.hpp` exactly —
 * static friction as a cancelled tangential *displacement* clamped to Coulomb's
 * cone, dynamic friction as an impulse bounded by what the normal impulse can
 * pay for, restitution against a closing speed captured before the solve
 * removed it. The arithmetic is re-derived for the four-particle gradient
 * rather than shared as code, because the rigid version reaches through poses
 * and lever arms that a particle does not have.
 */

#include <cmath>
#include <cstdint>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/core/rigid_body.hpp>
#include <SushiEngine/physics/solver/contact_projection.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief A non-penetration constraint between two weighted particle groups.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        struct SoftContactConstraint
        {
            /** @brief The particles involved: a vertex and a triangle, or two edges. */
            std::uint32_t particle[4] = {0, 0, 0, 0};

            /**
             * @brief Each particle's signed share of the contact.
             *
             * Negative on the side the normal points away from, positive on the
             * side it points toward; each side's magnitudes sum to one. An unused
             * slot carries zero, so a constraint with fewer than four distinct
             * particles needs no count alongside it.
             */
            T weight[4] = {0, 0, 0, 0};

            /**
             * @brief Which body each slot's particle belongs to, one bit per slot.
             *
             * Zero — every slot in the first body — for a self-collision or for any
             * source that holds a single array, which is why it needs no attention
             * from those callers at all. @ref SoftParticlePair reads it; nothing
             * else does.
             */
            std::uint8_t body_mask = 0;

            /** @brief Unit normal, pointing from the negative-weight side toward the positive one. */
            Vector3T<T> normal{Vector3T<T>{T(0), T(1), T(0)}};

            /** @brief The separation the pair is resolved to: the two surfaces' combined thickness. */
            T rest_distance = 0;

            /** @brief Accumulated normal impulse; bounds the friction cone. */
            T normal_lambda = 0;

            /** @brief Accumulated friction impulse in the contact's tangent basis. */
            T tangent_lambda[2] = {0, 0};

            /** @brief Closing speed captured before the substep's solve; positive is separating. */
            T normal_velocity = 0;
        };

        /** @brief The particles of one body: every slot indexes the same array. */
        template <typename T>
        struct SoftParticleArray
        {
            RigidBodyT<T>* particles = nullptr;
        };

        /** @brief The particles of two bodies, selected per slot by the constraint's mask. */
        template <typename T>
        struct SoftParticlePair
        {
            RigidBodyT<T>* first = nullptr;
            RigidBodyT<T>* second = nullptr;
        };

        /** @brief Resolves a constraint's slot against a single body's particles. */
        template <typename T>
        inline RigidBodyT<T>& resolve_soft_particle(const SoftParticleArray<T>& source,
                                                    const SoftContactConstraint<T>& constraint,
                                                    int slot) noexcept
        {
            return source.particles[constraint.particle[slot]];
        }

        /** @brief Resolves a constraint's slot against whichever of two bodies its mask names. */
        template <typename T>
        inline RigidBodyT<T>& resolve_soft_particle(const SoftParticlePair<T>& source,
                                                    const SoftContactConstraint<T>& constraint,
                                                    int slot) noexcept
        {
            const bool second = (constraint.body_mask & (1u << slot)) != 0u;
            return (second ? source.second : source.first)[constraint.particle[slot]];
        }

        /**
         * @brief The generalized inverse mass the four particles present along the normal.
         *
         * `sum_i inv_mass_i * weight_i^2` — the same quantity every projection in
         * this engine divides by, with the lever arm's contribution absent because
         * a particle has no rotational freedom to spend a correction on.
         *
         * @tparam T      The scalar element type.
         * @tparam Source Where the slots' particles live.
         * @param source     The particle source.
         * @param constraint The constraint.
         * @return Zero when every particle involved is pinned or unsimulated, which
         *         a caller must read as "nothing here can move".
         */
        template <typename T, typename Source>
        inline T soft_contact_inverse_mass(const Source& source,
                                           const SoftContactConstraint<T>& constraint) noexcept
        {
            T total = 0;
            for (int slot = 0; slot < 4; ++slot)
            {
                const RigidBodyT<T>& particle = resolve_soft_particle(source, constraint, slot);
                if (!is_simulated(particle.flags))
                    continue;
                total += particle.inv_mass * constraint.weight[slot] * constraint.weight[slot];
            }
            return total;
        }

        /**
         * @brief The weighted combination of the four particles' current positions.
         *
         * With the weights signed as @ref SoftContactConstraint describes, this is
         * the vector from the negative side's contact point to the positive side's
         * — so its projection on the normal is the separation, and nothing has to
         * reconstruct either point separately.
         */
        template <typename T, typename Source>
        inline Vector3T<T> soft_contact_offset(const Source& source,
                                               const SoftContactConstraint<T>& constraint) noexcept
        {
            Vector3T<T> total{T(0), T(0), T(0)};
            for (int slot = 0; slot < 4; ++slot)
                total = total + resolve_soft_particle(source, constraint, slot).position *
                                    constraint.weight[slot];
            return total;
        }

        /** @brief The same combination over the poses `predict` stashed at the substep's start. */
        template <typename T, typename Source>
        inline Vector3T<T> soft_contact_previous_offset(
            const Source& source, const SoftContactConstraint<T>& constraint) noexcept
        {
            Vector3T<T> total{T(0), T(0), T(0)};
            for (int slot = 0; slot < 4; ++slot)
                total = total + resolve_soft_particle(source, constraint, slot).previous_position *
                                    constraint.weight[slot];
            return total;
        }

        /** @brief The same combination over velocities: the pair's relative velocity. */
        template <typename T, typename Source>
        inline Vector3T<T> soft_contact_relative_velocity(
            const Source& source, const SoftContactConstraint<T>& constraint) noexcept
        {
            Vector3T<T> total{T(0), T(0), T(0)};
            for (int slot = 0; slot < 4; ++slot)
                total = total + resolve_soft_particle(source, constraint, slot).velocity *
                                    constraint.weight[slot];
            return total;
        }

        /**
         * @brief Applies a positional correction along @p direction, split by weight.
         *
         * @param source     The particle source; positions updated in place.
         * @param constraint The constraint whose weights split the correction.
         * @param direction  Unit direction the correction acts along.
         * @param magnitude  The correction's Lagrange multiplier.
         */
        template <typename T, typename Source>
        inline void apply_soft_contact_position(const Source& source,
                                                const SoftContactConstraint<T>& constraint,
                                                const Vector3T<T>& direction, T magnitude) noexcept
        {
            for (int slot = 0; slot < 4; ++slot)
            {
                RigidBodyT<T>& particle = resolve_soft_particle(source, constraint, slot);
                if (!is_simulated(particle.flags) || particle.inv_mass <= T(0))
                    continue;
                particle.position =
                    particle.position +
                    direction * (magnitude * constraint.weight[slot] * particle.inv_mass);
            }
        }

        /** @brief The velocity counterpart of @ref apply_soft_contact_position. */
        template <typename T, typename Source>
        inline void apply_soft_contact_velocity(const Source& source,
                                                const SoftContactConstraint<T>& constraint,
                                                const Vector3T<T>& direction, T magnitude) noexcept
        {
            for (int slot = 0; slot < 4; ++slot)
            {
                RigidBodyT<T>& particle = resolve_soft_particle(source, constraint, slot);
                if (!is_simulated(particle.flags) || particle.inv_mass <= T(0))
                    continue;
                particle.velocity =
                    particle.velocity +
                    direction * (magnitude * constraint.weight[slot] * particle.inv_mass);
            }
        }

        /**
         * @brief Records the pair's closing speed, at the top of a substep.
         *
         * @param source     The particle source.
         * @param constraint The constraint; its `normal_velocity` is updated.
         */
        template <typename T, typename Source>
        inline void capture_soft_contact_velocity(const Source& source,
                                                  SoftContactConstraint<T>& constraint) noexcept
        {
            constraint.normal_velocity =
                dot(soft_contact_relative_velocity(source, constraint), constraint.normal);
        }

        /**
         * @brief Non-penetration and static friction, positionally, for one contact.
         *
         * The separation is re-derived from the particles' current positions rather
         * than read from where the contact was generated, for the same reason a
         * rigid manifold re-derives it: the contact was found at the top of the
         * tick and the surface has been moving ever since.
         *
         * @param source          The particle source; positions updated in place.
         * @param constraint      The constraint; its accumulators are updated.
         * @param static_friction The combined coefficient bounding the tangent correction.
         */
        template <typename T, typename Source>
        inline void project_soft_contact_position(const Source& source,
                                                  SoftContactConstraint<T>& constraint,
                                                  T static_friction) noexcept
        {
            const T inverse_mass = soft_contact_inverse_mass(source, constraint);
            if (!(inverse_mass > T(0)))
                return;

            const T separation = dot(soft_contact_offset(source, constraint), constraint.normal);
            const T error = separation - constraint.rest_distance;
            if (error < T(0))
            {
                const T delta_lambda = -error / inverse_mass;
                constraint.normal_lambda += delta_lambda;
                apply_soft_contact_position(source, constraint, constraint.normal, delta_lambda);
            }

            if (constraint.normal_lambda <= T(0) || static_friction <= T(0))
                return;

            const Vector3T<T> slide = soft_contact_offset(source, constraint) -
                                      soft_contact_previous_offset(source, constraint);
            const Vector3T<T> tangential =
                slide - constraint.normal * dot(slide, constraint.normal);
            const T magnitude = length(tangential);
            if (magnitude <= T(1e-12))
                return;

            Vector3T<T> tangent_0;
            Vector3T<T> tangent_1;
            contact_tangent_basis(constraint.normal, tangent_0, tangent_1);

            const Vector3T<T> direction = tangential * (T(1) / magnitude);
            const T requested = -magnitude / inverse_mass;
            const T proposed_0 =
                constraint.tangent_lambda[0] + requested * dot(direction, tangent_0);
            const T proposed_1 =
                constraint.tangent_lambda[1] + requested * dot(direction, tangent_1);
            const T limit = static_friction * constraint.normal_lambda;
            // Outside the cone the surfaces are sliding, and sliding is the
            // velocity pass's answer, not this one's — applying the clipped part as
            // well would charge the contact for friction twice.
            if (proposed_0 * proposed_0 + proposed_1 * proposed_1 > limit * limit)
                return;

            const T applied_0 = proposed_0 - constraint.tangent_lambda[0];
            const T applied_1 = proposed_1 - constraint.tangent_lambda[1];
            constraint.tangent_lambda[0] = proposed_0;
            constraint.tangent_lambda[1] = proposed_1;

            apply_soft_contact_position(source, constraint, tangent_0, applied_0);
            apply_soft_contact_position(source, constraint, tangent_1, applied_1);
        }

        /**
         * @brief Dynamic friction and restitution for one contact, after the pose solve.
         *
         * @param source                 The particle source; velocities updated in place.
         * @param constraint             The constraint, with its normal impulse from the pose solve.
         * @param dynamic_friction       The combined sliding coefficient.
         * @param restitution            The combined bounce, in [0, 1].
         * @param restitution_threshold  Below this arrival speed, restitution is suppressed.
         * @param h                      The substep duration, in seconds (> 0).
         */
        template <typename T, typename Source>
        inline void solve_soft_contact_velocity(const Source& source,
                                                const SoftContactConstraint<T>& constraint,
                                                T dynamic_friction, T restitution,
                                                T restitution_threshold, T h) noexcept
        {
            if (h <= T(0) || constraint.normal_lambda <= T(0))
                return;
            const T inverse_mass = soft_contact_inverse_mass(source, constraint);
            if (!(inverse_mass > T(0)))
                return;

            if (dynamic_friction > T(0))
            {
                const Vector3T<T> relative = soft_contact_relative_velocity(source, constraint);
                const Vector3T<T> tangential =
                    relative - constraint.normal * dot(relative, constraint.normal);
                const T speed = length(tangential);
                if (speed > T(1e-12))
                {
                    const Vector3T<T> direction = tangential * (T(-1) / speed);
                    const T halt = speed / inverse_mass;
                    const T budget = dynamic_friction * constraint.normal_lambda / h;
                    apply_soft_contact_velocity(source, constraint, direction,
                                                halt < budget ? halt : budget);
                }
            }

            const Vector3T<T> relative = soft_contact_relative_velocity(source, constraint);
            const T normal_speed = dot(relative, constraint.normal);
            const T bounce_coefficient =
                std::abs(constraint.normal_velocity) > restitution_threshold ? restitution : T(0);
            const T target = -bounce_coefficient * constraint.normal_velocity;
            const T bounce = target > T(0) ? target : T(0);
            const T change = bounce - normal_speed;
            if (std::abs(change) > T(1e-12))
                apply_soft_contact_velocity(source, constraint, constraint.normal,
                                            change / inverse_mass);
        }
    } // namespace Physics
} // namespace SushiEngine
