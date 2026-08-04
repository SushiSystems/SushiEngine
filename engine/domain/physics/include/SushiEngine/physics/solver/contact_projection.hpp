/**************************************************************************/
/* contact_projection.hpp                                                 */
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
 * @file contact_projection.hpp
 * @brief Resolving a contact manifold: non-penetration, friction, and restitution.
 *
 * The recipe is Müller et al. 2020, *Detailed Rigid Body Simulation with XPBD*,
 * §7.4 of `docs/design/physics_system.md`, and it is split across the substep in
 * the way that paper prescribes, for a reason worth stating once:
 *
 * - **Non-penetration and static friction are positional.** A box sitting on a
 *   ramp must not creep, and "must not creep" is a statement about *position*,
 *   not about velocity. Solving it as a velocity constraint leaves a residual
 *   drift every step that no damping removes; solving it positionally, by
 *   cancelling the tangential displacement the anchors accumulated since the
 *   substep began, leaves nothing to drift.
 * - **Dynamic friction and restitution are velocity-level**, because they are
 *   statements about speed that a positional projection cannot express. XPBD's
 *   whole reason for a velocity pass is these two quantities, and the engine
 *   having had no velocity pass at all is exactly why it had neither (§1.2
 *   item 2).
 *
 * The schedule these functions are written for, and each one's place in it:
 *
 * ```
 *   per tick:
 *     generate manifolds                 (collision/manifold.hpp)
 *     warm_start_manifold()              <- inherit last tick's accumulators
 *     for each substep:
 *       capture_contact_velocities()     <- what speed did it arrive at
 *       clear_manifold_impulses()        <- every substep but the first
 *       predict()
 *       ...joints, distance constraints...
 *       solve_manifold_positions()       <- non-penetration + static friction
 *       update_velocity()
 *       solve_manifold_velocities()      <- dynamic friction + restitution
 * ```
 *
 * The accumulators are **per substep**, not per tick, and that is what makes the
 * dynamic-friction budget (`mu * lambda_n / h`) mean anything: a lambda summed
 * over 32 substeps would authorize 32 times the friction a substep can actually
 * pay for, and a sliding box would stop dead. The first substep is the exception
 * — it keeps what warm starting gave it, because the previous tick's final
 * substep is the same quantity measured over the same duration, so it is a
 * genuine estimate rather than a stale total.
 *
 * This layer names `ContactManifold` and `RigidBodyT` and nothing else — no
 * shape, no broadphase, no runtime. It is where `physics/solver` is allowed to
 * be (§3.2): it turns manifolds into corrected positions and velocities without
 * knowing what produced them.
 */

#include <cmath>
#include <cstddef>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/collision/manifold.hpp>
#include <SushiEngine/physics/core/material.hpp>
#include <SushiEngine/physics/core/rigid_body.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief The coefficients one contact is resolved under.
         *
         * Resolved once per manifold from the two bodies' materials rather than
         * looked up per point: every point of a manifold is the same two surfaces
         * meeting, so a per-point lookup would be the same answer four times.
         */
        template <typename T>
        struct ContactSolveParameters
        {
            /**
             * @brief The separation contacts are resolved *to* (§7.6).
             *
             * Zero means the surfaces come to rest exactly touching. A small
             * positive value keeps a visible sliver of air; a small negative value
             * lets a tyre visibly deform into the ground.
             */
            T rest_offset = 0;

            /**
             * @brief The most a contact may push apart in one substep, in metres.
             *
             * §7.6's depenetration budget. A body spawned deeply inside another has a
             * separation error of whole metres, and projecting all of it in one substep
             * turns a placement mistake into an explosion — the pair leaves at a speed
             * nothing in the scene put there, taking whatever it hits with it.
             *
             * A **distance per substep** rather than a velocity, even though §7.6 names a
             * velocity, because that is what the projection can use without being told the
             * substep length: the caller knows `h` and multiplies once, which keeps this
             * function a pure function of its parameters and costs nothing at the call site
             * that was already building them per tick.
             *
             * Zero or less disables the clamp, which is the right default for a solver whose
             * callers have not opted in — an unclamped correction is the previous behaviour
             * and is correct for every scene that does not spawn overlapping.
             */
            T max_depenetration = 0;

            /** @brief Combined static friction; bounds the positional tangent correction. */
            T static_friction = T(0.6);

            /** @brief Combined dynamic friction; bounds the velocity-pass tangent impulse. */
            T dynamic_friction = T(0.5);

            /** @brief Combined restitution: 0 keeps no closing speed, 1 returns all of it. */
            T restitution = 0;

            /**
             * @brief Below this closing speed, restitution is suppressed.
             *
             * The standard anti-jitter threshold. A resting body's contacts carry a
             * closing speed of about `gravity * substep` every substep purely
             * because gravity had a substep to act; bouncing that back is how a
             * settled stack buzzes for ever. Set it to `2 * |gravity| * h` and the
             * buzz has nothing to feed on.
             */
            T restitution_threshold = 0;
        };

        /**
         * @brief The solve parameters for a contact between two materials.
         *
         * @tparam T The scalar element type.
         * @param material_a            The first body's material.
         * @param material_b            The second body's material.
         * @param rest_offset           The separation to resolve to (§7.6).
         * @param restitution_threshold The anti-jitter floor, usually `2 * g * h`.
         */
        template <typename T>
        inline ContactSolveParameters<T> make_contact_parameters(
            const PhysicsMaterialT<T>& material_a, const PhysicsMaterialT<T>& material_b,
            T rest_offset, T restitution_threshold) noexcept
        {
            ContactSolveParameters<T> parameters;
            parameters.rest_offset = rest_offset;
            combine_friction(material_a, material_b, parameters.static_friction,
                             parameters.dynamic_friction);
            parameters.restitution = combine_restitution(material_a, material_b);
            parameters.restitution_threshold = restitution_threshold;
            return parameters;
        }

        /**
         * @brief A body standing in for immovable geometry: infinite mass, at the origin.
         *
         * A static plane has no body, but every projection here is written for two
         * of them. Rather than duplicate each function for the one-body case — which
         * is how a plane contact and a pair contact end up disagreeing, the mistake
         * §1.3 already recorded once — the plane is handed in as a body with zero
         * inverse mass and zero inverse inertia. Every correction it would take is
         * multiplied by zero, so it absorbs nothing and the other body takes all of
         * it, which is what "immovable" means. Its frame is the world frame, which
         * is the frame `generate_obb_plane_manifold` writes its anchors in.
         */
        template <typename T>
        inline RigidBodyT<T> immovable_body() noexcept
        {
            RigidBodyT<T> body;
            body.position = Vector3T<T>{T(0), T(0), T(0)};
            body.orientation = QuaternionT<T>{T(0), T(0), T(0), T(1)};
            body.previous_position = body.position;
            body.previous_orientation = body.orientation;
            body.velocity = Vector3T<T>{T(0), T(0), T(0)};
            body.angular_velocity = Vector3T<T>{T(0), T(0), T(0)};
            body.inv_mass = T(0);
            body.inv_inertia = Vector3T<T>{T(0), T(0), T(0)};
            body.flags = BodyFlags::static_body;
            return body;
        }

        /**
         * @brief Two unit vectors spanning the plane perpendicular to @p normal.
         *
         * Friction is accumulated in this basis rather than along whatever direction
         * the body happened to slide in this substep, which is what makes the
         * accumulator meaningful across substeps and warm-startable across ticks.
         * The basis is built from whichever world axis the normal is least aligned
         * with, so it is continuous under small rotations and identical for the same
         * normal every time — the determinism rule reaching into the friction model.
         */
        template <typename T>
        inline void contact_tangent_basis(const Vector3T<T>& normal, Vector3T<T>& tangent_0,
                                          Vector3T<T>& tangent_1) noexcept
        {
            const T ax = std::abs(normal.x);
            const T ay = std::abs(normal.y);
            const T az = std::abs(normal.z);
            Vector3T<T> reference{T(1), T(0), T(0)};
            if (ay <= ax && ay <= az)
                reference = Vector3T<T>{T(0), T(1), T(0)};
            else if (az <= ax && az <= ay)
                reference = Vector3T<T>{T(0), T(0), T(1)};

            tangent_0 = cross(normal, reference);
            const T tangent_length = length(tangent_0);
            tangent_0 = tangent_length > T(1e-12) ? tangent_0 * (T(1) / tangent_length)
                                                  : Vector3T<T>{T(1), T(0), T(0)};
            tangent_1 = cross(normal, tangent_0);
        }

        /**
         * @brief Records the closing speed at every point, for the velocity pass to read.
         *
         * Called at the top of a substep, before `predict`. By the time restitution
         * is applied the positional solve has removed the very speed restitution is
         * a statement about, so the arrival speed has to be captured while it still
         * exists. Positive is separating.
         */
        template <typename T>
        inline void capture_contact_velocities(ContactManifold<T>& manifold,
                                               const RigidBodyT<T>& body_a,
                                               const RigidBodyT<T>& body_b) noexcept
        {
            for (std::size_t i = 0; i < manifold.point_count; ++i)
            {
                const Vector3T<T> lever_a = rotate(body_a.orientation, manifold.points[i].anchor_a_local);
                const Vector3T<T> lever_b = rotate(body_b.orientation, manifold.points[i].anchor_b_local);
                const Vector3T<T> relative =
                    point_velocity(body_b, lever_b) - point_velocity(body_a, lever_a);
                manifold.points[i].normal_velocity = dot(relative, manifold.normal);
            }
        }

        /**
         * @brief Clears the Lagrange accumulators, at the top of a substep.
         *
         * Called on every substep but the first, per the schedule in this file's
         * header: the accumulators measure one substep's impulse, so carrying them
         * across substeps would inflate the friction budget by the substep count.
         * The first substep is skipped so that what @ref warm_start_manifold
         * inherited from the previous tick survives to be used.
         *
         * What the inherited value buys, precisely: the static-friction cone is
         * bounded by `mu * lambda_n`, so a contact that starts a tick with no normal
         * impulse has no friction until one builds, and a box on a ramp creeps for
         * the substep it takes to build. Inheriting the bound removes that substep.
         * The inherited *impulse* is deliberately not re-applied to the bodies: a
         * positional solver that pushes with a force it has not re-derived from the
         * current poses will push apart a pair that has already separated.
         */
        template <typename T>
        inline void clear_manifold_impulses(ContactManifold<T>& manifold) noexcept
        {
            for (std::size_t i = 0; i < manifold.point_count; ++i)
            {
                manifold.points[i].normal_lambda = T(0);
                manifold.points[i].tangent_lambda[0] = T(0);
                manifold.points[i].tangent_lambda[1] = T(0);
            }
        }

        /**
         * @brief Non-penetration and static friction, positionally, for one manifold.
         *
         * Per point, in order:
         *
         * 1. **Non-penetration.** The separation is re-derived from the two anchors
         *    at their current poses — not read from the manifold, which was
         *    generated a whole tick ago — and the shortfall against `rest_offset` is
         *    projected out, split by generalized inverse mass. A point that is
         *    already at or beyond the rest offset contributes nothing, which is what
         *    makes a contact an inequality rather than a spring.
         * 2. **Static friction.** The anchors' tangential displacement *since the
         *    substep began* is cancelled, clamped so the accumulated tangent impulse
         *    never exceeds `mu_static * lambda_normal` — Coulomb's cone, applied to
         *    positions. Cancelling a displacement rather than a velocity is what
         *    makes a box on a ramp stay exactly where it is rather than settle into
         *    a slow slide.
         *
         * The tangential displacement is measured against `previous_position` /
         * `previous_orientation`, which `predict` stashed at the top of this substep.
         *
         * @param manifold   The manifold; its accumulators are updated in place.
         * @param body_a     The first body (the normal points away from it).
         * @param body_b     The second body; pass @ref immovable_body for static geometry.
         * @param parameters The combined coefficients.
         */
        template <typename T>
        inline void solve_manifold_positions(ContactManifold<T>& manifold, RigidBodyT<T>& body_a,
                                             RigidBodyT<T>& body_b,
                                             const ContactSolveParameters<T>& parameters) noexcept
        {
            if (manifold.point_count == 0)
                return;
            if (body_a.inv_mass + body_b.inv_mass <= T(0))
                return;

            Vector3T<T> tangent_0;
            Vector3T<T> tangent_1;
            contact_tangent_basis(manifold.normal, tangent_0, tangent_1);

            for (std::size_t i = 0; i < manifold.point_count; ++i)
            {
                ContactPoint<T>& point = manifold.points[i];

                const Vector3T<T> lever_a = rotate(body_a.orientation, point.anchor_a_local);
                const Vector3T<T> lever_b = rotate(body_b.orientation, point.anchor_b_local);
                const Vector3T<T> world_a = body_a.position + lever_a;
                const Vector3T<T> world_b = body_b.position + lever_b;

                const T separation = dot(world_b - world_a, manifold.normal);
                point.separation = separation;

                T error = separation - parameters.rest_offset;
                if (error < T(0))
                {
                    // The depenetration budget (§7.6). Clamped rather than skipped: the
                    // contact still resolves, over as many substeps as the budget needs,
                    // which is what makes a deep overlap push apart smoothly instead of
                    // leaving at a speed nothing in the scene put there.
                    //
                    // The budget covers *stale* penetration — overlap that was already
                    // there when the substep began — and on top of it, however much the
                    // pair closed *during* this substep. The distinction is what lets
                    // one clamp serve two different situations. A body spawned inside
                    // another has a violation its own motion did not create; paying it
                    // out at the budget's rate is the whole point of the clamp. A body
                    // that crossed most of a thin plate in one substep has a violation
                    // that *is* its own motion, and clamping that away is how §16.19's
                    // 200 m/s sphere ended the impact substep still buried, had its
                    // speed killed by the velocity pass, and was then walked out of the
                    // far side by the next tick's nearest-face manifold — the recorded
                    // mirror-image rest pose. Cancelling the approach removes exactly
                    // the motion the substep added, so it cannot inject energy.
                    if (parameters.max_depenetration > T(0))
                    {
                        const Vector3T<T> then_world_a =
                            body_a.previous_position +
                            rotate(body_a.previous_orientation, point.anchor_a_local);
                        const Vector3T<T> then_world_b =
                            body_b.previous_position +
                            rotate(body_b.previous_orientation, point.anchor_b_local);
                        const T previous_separation =
                            dot(then_world_b - then_world_a, manifold.normal);
                        const T approach =
                            previous_separation > separation ? previous_separation - separation
                                                         : T(0);
                        const T budget = parameters.max_depenetration + approach;
                        if (error < -budget)
                            error = -budget;
                    }

                    const T w = generalized_inverse_mass(body_a, lever_a, manifold.normal) +
                                generalized_inverse_mass(body_b, lever_b, manifold.normal);
                    if (w > T(0))
                    {
                        const T delta_lambda = -error / w;
                        point.normal_lambda += delta_lambda;
                        const Vector3T<T> impulse = manifold.normal * delta_lambda;
                        apply_positional_impulse(body_a, impulse, lever_a, T(-1));
                        apply_positional_impulse(body_b, impulse, lever_b, T(1));
                    }
                }

                if (point.normal_lambda <= T(0) || parameters.static_friction <= T(0))
                    continue;

                // How far the two anchors slid past each other since the substep
                // began. The poses have moved since the levers above were taken, so
                // both ends are recomputed rather than reused.
                const Vector3T<T> now_a =
                    body_a.position + rotate(body_a.orientation, point.anchor_a_local);
                const Vector3T<T> now_b =
                    body_b.position + rotate(body_b.orientation, point.anchor_b_local);
                const Vector3T<T> then_a =
                    body_a.previous_position +
                    rotate(body_a.previous_orientation, point.anchor_a_local);
                const Vector3T<T> then_b =
                    body_b.previous_position +
                    rotate(body_b.previous_orientation, point.anchor_b_local);

                const Vector3T<T> slide = (now_b - then_b) - (now_a - then_a);
                const Vector3T<T> tangential =
                    slide - manifold.normal * dot(slide, manifold.normal);
                const T magnitude = length(tangential);
                if (magnitude <= T(1e-12))
                    continue;

                const Vector3T<T> direction = tangential * (T(1) / magnitude);
                const Vector3T<T> arm_a = rotate(body_a.orientation, point.anchor_a_local);
                const Vector3T<T> arm_b = rotate(body_b.orientation, point.anchor_b_local);
                const T w = generalized_inverse_mass(body_a, arm_a, direction) +
                            generalized_inverse_mass(body_b, arm_b, direction);
                if (w <= T(0))
                    continue;

                // Cancel the slide — but only if the whole of it fits inside
                // Coulomb's cone. Static friction is the claim "these surfaces have
                // not moved relative to each other", and a claim that has to be
                // clipped to fit is a claim that is false: the contact is sliding,
                // and sliding is the velocity pass's dynamic friction to answer,
                // not this one's. Applying the clipped part *as well* would charge
                // the contact for friction twice and a launched box would
                // decelerate at something like twice mu*g.
                const T requested = -magnitude / w;
                const T proposed_0 = point.tangent_lambda[0] + requested * dot(direction, tangent_0);
                const T proposed_1 = point.tangent_lambda[1] + requested * dot(direction, tangent_1);
                const T limit = parameters.static_friction * point.normal_lambda;
                if (proposed_0 * proposed_0 + proposed_1 * proposed_1 > limit * limit)
                    continue;

                const T applied_0 = proposed_0 - point.tangent_lambda[0];
                const T applied_1 = proposed_1 - point.tangent_lambda[1];
                point.tangent_lambda[0] = proposed_0;
                point.tangent_lambda[1] = proposed_1;

                const Vector3T<T> impulse = tangent_0 * applied_0 + tangent_1 * applied_1;
                if (dot(impulse, impulse) <= T(0))
                    continue;
                apply_positional_impulse(body_a, impulse, arm_a, T(-1));
                apply_positional_impulse(body_b, impulse, arm_b, T(1));
            }
        }

        /**
         * @brief Dynamic friction and restitution, for one manifold, after the pose solve.
         *
         * The two things a positional projection cannot say:
         *
         * - **Dynamic friction** removes tangential speed, up to what the normal
         *   impulse this substep can pay for: `mu_dynamic * |lambda_n| / h`. Beyond
         *   that the surfaces are sliding and friction is a bounded force, not a
         *   weld.
         * - **Restitution** restores the closing speed the positional solve
         *   destroyed, scaled by `e`: the body leaves at `-e` times the speed it
         *   arrived at. Below @ref ContactSolveParameters::restitution_threshold it is
         *   suppressed entirely, because a resting body's contacts carry a closing
         *   speed of about `g * h` every substep purely from gravity, and returning
         *   that is how a settled stack buzzes.
         *
         * @param manifold   The manifold, with `normal_velocity` captured this substep.
         * @param body_a     The first body; velocities updated in place.
         * @param body_b     The second body; pass @ref immovable_body for static geometry.
         * @param parameters The combined coefficients.
         * @param h          The substep duration, in seconds (> 0).
         */
        template <typename T>
        inline void solve_manifold_velocities(ContactManifold<T>& manifold, RigidBodyT<T>& body_a,
                                              RigidBodyT<T>& body_b,
                                              const ContactSolveParameters<T>& parameters,
                                              T h) noexcept
        {
            if (manifold.point_count == 0 || h <= T(0))
                return;
            if (body_a.inv_mass + body_b.inv_mass <= T(0))
                return;

            for (std::size_t i = 0; i < manifold.point_count; ++i)
            {
                ContactPoint<T>& point = manifold.points[i];
                // A point the positional pass never engaged carries no normal
                // impulse, so it has no friction budget and nothing bounced off it.
                if (point.normal_lambda <= T(0))
                    continue;

                const Vector3T<T> lever_a = rotate(body_a.orientation, point.anchor_a_local);
                const Vector3T<T> lever_b = rotate(body_b.orientation, point.anchor_b_local);

                // Dynamic friction. Both quantities the bound compares are
                // *impulses*, which is the step that is easy to get wrong: the
                // impulse that would halt the slide outright is `|v_t| / w`, and
                // what the contact can actually spend is `mu * lambda_n / h`. Take
                // the smaller. Comparing a speed against an impulse instead —
                // `min(mu * lambda_n / h, |v_t|)` reads naturally and is dimensional
                // nonsense — leaves the deceleration off by the generalized mass,
                // which for a corner contact is a factor of four.
                {
                    const Vector3T<T> relative =
                        point_velocity(body_b, lever_b) - point_velocity(body_a, lever_a);
                    const Vector3T<T> tangential =
                        relative - manifold.normal * dot(relative, manifold.normal);
                    const T tangential_speed = length(tangential);
                    if (parameters.dynamic_friction > T(0) && tangential_speed > T(1e-12))
                    {
                        const Vector3T<T> direction = tangential * (T(-1) / tangential_speed);
                        const T w = generalized_inverse_mass(body_a, lever_a, direction) +
                                    generalized_inverse_mass(body_b, lever_b, direction);
                        if (w > T(0))
                        {
                            const T halt = tangential_speed / w;
                            const T budget = parameters.dynamic_friction * point.normal_lambda / h;
                            const Vector3T<T> impulse =
                                direction * (halt < budget ? halt : budget);
                            apply_velocity_impulse(body_a, impulse, lever_a, T(-1));
                            apply_velocity_impulse(body_b, impulse, lever_b, T(1));
                        }
                    }
                }

                // Restitution, applied whenever the point carried an impulse — not
                // only when the material bounces. With `e == 0` it reads "leave this
                // contact at zero closing speed", and that is not a case worth
                // skipping: the positional pass pushes a penetrating body out,
                // `update_velocity` reads that push back as a real velocity, and a
                // body that had sunk in over the tick would launch off the surface
                // at penetration-over-substep — metres per second, out of nothing.
                // Restitution is the mechanism that decides how much of a landing
                // speed comes back, and zero is one of its answers.
                {
                    const Vector3T<T> relative =
                        point_velocity(body_b, lever_b) - point_velocity(body_a, lever_a);
                    const T normal_speed = dot(relative, manifold.normal);
                    const T restitution =
                        std::abs(point.normal_velocity) > parameters.restitution_threshold
                            ? parameters.restitution
                            : T(0);
                    const T target = -restitution * point.normal_velocity;
                    const T bounce = target > T(0) ? target : T(0);
                    const T change = bounce - normal_speed;
                    if (std::abs(change) > T(1e-12))
                    {
                        const T w = generalized_inverse_mass(body_a, lever_a, manifold.normal) +
                                    generalized_inverse_mass(body_b, lever_b, manifold.normal);
                        if (w > T(0))
                        {
                            const Vector3T<T> impulse = manifold.normal * (change / w);
                            apply_velocity_impulse(body_a, impulse, lever_a, T(-1));
                            apply_velocity_impulse(body_b, impulse, lever_b, T(1));
                        }
                    }
                }
            }
        }
    } // namespace Physics
} // namespace SushiEngine
