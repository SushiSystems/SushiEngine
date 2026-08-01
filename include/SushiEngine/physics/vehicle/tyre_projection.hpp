/**************************************************************************/
/* tyre_projection.hpp                                                    */
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
 * @file tyre_projection.hpp
 * @brief Where §11.5's patch is, and who takes the reaction.
 *
 * @ref tyre.hpp is slip and load in, force out, and knows nothing about the world. This
 * file is the other half: it finds the contacts a wheel is standing on, folds them into
 * one patch, asks the model, and spends the answer as an impulse on the wheel and on
 * whatever is under it.
 *
 * ### The load is read back, not invented
 *
 * §11.5: *"Load sensitivity from the contact normal force the solver already recovered."*
 * A wheel is a real rigid body with a real contact (§11.2's fourth row), so the normal
 * force is already a number the solver produced — `ContactPoint::normal_lambda` over the
 * substep it was accumulated in. Nothing here raycasts for the ground or models a spring
 * to it. A raycast wheel is the arcade shortcut and it is the reason arcade cars cannot
 * drive over a kerb: the ray finds the ground and the wheel is not really there.
 *
 * ### One patch per wheel, not one per point
 *
 * A manifold has up to four points and a wheel can touch several bodies at once. Slip is
 * a property of *the patch* — one velocity, one slip angle — so the points are folded
 * into a single load-weighted patch, the model is asked once, and the reaction is shared
 * back out by each contact's share of the load. Evaluating the curve per point would
 * saturate each one separately and give a wheel across a kerb edge more total grip than a
 * wheel flat on the road, which is exactly backwards.
 *
 * ### The wheel must not also have Coulomb friction
 *
 * The solver's own friction runs inside the substep loop on the same contact. If the
 * wheel's material has friction, the tyre model is a *second* tangential source and the
 * two add — a car with twice the grip it was authored with, and no single number wrong
 * anywhere to find. `SuspensionSetupT::material_index` exists so a wheel can point at a
 * frictionless material, and it is not a detail: a tyre model on a gripping wheel is the
 * one mistake this file cannot detect for you.
 */

#include <cmath>
#include <cstddef>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/collision/manifold.hpp>
#include <SushiEngine/physics/core/handle.hpp>
#include <SushiEngine/physics/core/rigid_body.hpp>
#include <SushiEngine/physics/solver/solver_interface.hpp>
#include <SushiEngine/physics/vehicle/tyre.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief What one wheel's tyre did this tick.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        struct TyreReportT
        {
            /** @brief Whether the wheel found any ground at all. */
            bool grounded = false;

            /** @brief Normal force carried by the whole patch, in newtons. */
            T load = T(0);

            /** @brief How far the patch was from rolling. */
            TyreSlipT<T> slip;

            /** @brief What it produced. */
            TyreForceT<T> force;

            /** @brief The patch, in world space; meaningless when not @ref grounded. */
            Vector3T<T> patch{T(0), T(0), T(0)};
        };

        /**
         * @brief The force one contact point is carrying, in newtons.
         *
         * `ContactPoint::normal_lambda` is a *positional* XPBD multiplier, not an impulse:
         * the solver spends it as a displacement, so it carries an extra factor of the
         * substep. Its own dynamic-friction pass is the proof — it compares
         * `normal_lambda / h` against a quantity in kilogram-metres per second, so
         * `normal_lambda / h` is an impulse and the force is that again over `h`.
         *
         * Getting this wrong does not look wrong. A load off by the substep is a load off
         * by a few hundred, and a tyre asked for grip proportional to a load of half a
         * newton simply produces almost nothing — a car that rolls but will not drive,
         * with every formula in the model correct.
         *
         * @tparam T The scalar element type.
         * @param point   The contact point.
         * @param substep The substep the multiplier was accumulated in, in seconds.
         */
        template <typename T>
        inline T contact_point_load(const ContactPoint<T>& point, T substep) noexcept
        {
            return point.normal_lambda / (substep * substep);
        }

        /**
         * @brief Reads the body on the far side of a contact from the wheel.
         *
         * @tparam T The scalar element type.
         * @param solver     The world.
         * @param contact    The contact.
         * @param wheel_is_a Whether the wheel is the contact's first slot.
         * @param other      Filled with the far body when there is one.
         * @return False when the far side is static geometry, which has no body at all.
         */
        template <typename T>
        inline bool tyre_other_body(const IConstraintSolver<T>& solver,
                                    const ContactConstraintT<T>& contact, bool wheel_is_a,
                                    RigidBodyT<T>& other) noexcept
        {
            const std::uint32_t slot = wheel_is_a ? contact.b : contact.a;
            if (slot == null_contact_body || slot >= solver.body_capacity())
            {
                other = immovable_body<T>();
                return false;
            }
            solver.read_bodies(slot, 1, &other);
            return true;
        }

        /**
         * @brief Gives each contacted body its share of the reaction.
         *
         * Shared by load rather than given whole to one body, so a wheel bridging two
         * crates pushes on each in proportion to what it is standing on. The shares sum to
         * one, so the impulse the wheel gained is exactly the impulse the world lost —
         * without that, a car could drive on a wheel touching two things and gain momentum
         * from the arrangement.
         *
         * @tparam T The scalar element type.
         * @param solver     The world.
         * @param wheel_slot The wheel's slot, to recognise which side of a contact it is.
         * @param impulse    What the wheel was given; each body takes its share negated.
         * @param patch      Where, in world space.
         * @param load       The whole patch's load, as the divisor of each share.
         * @param substep    The substep the contact impulses were accumulated in.
         */
        template <typename T>
        inline void tyre_spend_reaction(IConstraintSolver<T>& solver, std::size_t wheel_slot,
                                   const Vector3T<T>& impulse, const Vector3T<T>& patch, T load,
                                   T substep)
        {
            const std::size_t contacts = solver.contact_count();
            for (std::size_t i = 0; i < contacts; ++i)
            {
                ContactConstraintT<T> contact;
                if (!solver.read_contact(i, contact))
                    continue;

                const bool wheel_is_a = std::size_t(contact.a) == wheel_slot;
                if (!wheel_is_a && std::size_t(contact.b) != wheel_slot)
                    continue;

                const std::uint32_t slot = wheel_is_a ? contact.b : contact.a;
                if (slot == null_contact_body || slot >= solver.body_capacity())
                    continue;

                T share = T(0);
                for (std::size_t p = 0; p < contact.manifold.point_count; ++p)
                {
                    const T carried =
                        contact_point_load(contact.manifold.points[p], substep);
                    if (carried > T(0))
                        share += carried;
                }
                if (!(share > T(0)))
                    continue;

                RigidBodyT<T> other;
                solver.read_bodies(slot, 1, &other);
                if (!is_simulated(other.flags))
                    continue;

                apply_velocity_impulse(other, impulse * (share / load),
                                       patch - other.position, T(-1));
                solver.write_body(solver.body_handle(std::size_t(slot)), other);
            }
        }
        /**
         * @brief Finds a wheel's patch, evaluates its tyre, and spends the result.
         *
         * Reads the contacts standing in the solver — which are the ones its last step
         * produced — so the load and the patch are a tick old. That is the same lag the
         * powertrain's wheel speeds carry and the same lag every explicit coupling in this
         * engine accepts: a tick at 60 Hz is 16 mm of travel at 1 m/s, and the alternative
         * is a per-substep callback through the solver seam that the device solver could
         * not honour anyway.
         *
         * The impulse is applied at the patch on the wheel and, in each contact's share of
         * the load, at the patch on whatever that contact was against. Static geometry
         * takes its share and does nothing with it, which is what static geometry is for.
         *
         * @tparam T The scalar element type.
         * @param solver   The world the wheel lives in.
         * @param wheel    The wheel body.
         * @param settings The tyre.
         * @param axle     The wheel's spin axis in world space, as a unit vector.
         * @param tick     The tick, in seconds; the impulse is the force over this.
         * @param substep  The substep the contact impulses were accumulated in, in
         *                 seconds; the load is `normal_lambda` over this.
         * @return What happened, for a gauge or a skid sound to read.
         */
        template <typename T>
        inline TyreReportT<T> apply_tyre_force(IConstraintSolver<T>& solver, BodyHandle wheel,
                                               const TyreSettingsT<T>& settings,
                                               const Vector3T<T>& axle, T tick, T substep)
        {
            TyreReportT<T> report;
            if (!wheel.valid() || !(tick > T(0)) || !(substep > T(0)))
                return report;

            const std::size_t wheel_slot = solver.body_slot(wheel);
            RigidBodyT<T> wheel_body;
            if (!solver.read_body(wheel, wheel_body))
                return report;

            Vector3T<T> patch{T(0), T(0), T(0)};
            Vector3T<T> normal{T(0), T(0), T(0)};
            Vector3T<T> ground_velocity{T(0), T(0), T(0)};
            T load = T(0);

            const std::size_t contacts = solver.contact_count();
            for (std::size_t i = 0; i < contacts; ++i)
            {
                ContactConstraintT<T> contact;
                if (!solver.read_contact(i, contact))
                    continue;

                const bool wheel_is_a = std::size_t(contact.a) == wheel_slot;
                if (!wheel_is_a && std::size_t(contact.b) != wheel_slot)
                    continue;

                RigidBodyT<T> other;
                const bool has_other = tyre_other_body(solver, contact, wheel_is_a, other);
                const RigidBodyT<T>& body_a = wheel_is_a ? wheel_body : other;
                const RigidBodyT<T>& body_b = wheel_is_a ? other : wheel_body;

                // The manifold's normal points *away from* body a, so for a wheel
                // standing on the road the road is the second body and the normal aims
                // down into it. The tyre wants the surface normal holding the wheel up,
                // which is therefore the negation whenever the wheel is the first slot.
                const Vector3T<T> toward_wheel =
                    wheel_is_a ? contact.manifold.normal * T(-1) : contact.manifold.normal;

                for (std::size_t p = 0; p < contact.manifold.point_count; ++p)
                {
                    const ContactPoint<T>& point = contact.manifold.points[p];
                    const T share = contact_point_load(point, substep);
                    if (!(share > T(0)))
                        continue;

                    const Vector3T<T> world =
                        wheel_is_a ? to_world_anchor(body_a.position, body_a.orientation,
                                                     point.anchor_a_local)
                                   : to_world_anchor(body_b.position, body_b.orientation,
                                                     point.anchor_b_local);

                    patch = patch + world * share;
                    normal = normal + toward_wheel * share;
                    if (has_other)
                    {
                        ground_velocity =
                            ground_velocity +
                            point_velocity(other, world - other.position) * share;
                    }
                    load += share;
                }
            }

            if (!(load > T(0)))
                return report;

            patch = patch * (T(1) / load);
            ground_velocity = ground_velocity * (T(1) / load);
            const T normal_length = length(normal);
            if (!(normal_length > T(0)))
                return report;
            normal = normal * (T(1) / normal_length);

            report.grounded = true;
            report.load = load;
            report.patch = patch;

            // The axle laid flat on the surface, and the rolling direction square to it.
            // Taking the tangential part rather than assuming the axle is already in the
            // plane is what lets a cambered wheel, or one on a slope, still be right.
            Vector3T<T> lateral = axle - normal * dot(axle, normal);
            const T lateral_length = length(lateral);
            if (!(lateral_length > T(1e-9)))
                return report;
            lateral = lateral * (T(1) / lateral_length);
            const Vector3T<T> forward = cross(lateral, normal);

            const Vector3T<T> patch_velocity =
                point_velocity(wheel_body, patch - wheel_body.position) - ground_velocity;
            const Vector3T<T> hub_velocity = wheel_body.velocity - ground_velocity;

            report.slip = tyre_slip(settings, dot(patch_velocity, forward),
                                    dot(patch_velocity, lateral), dot(hub_velocity, forward));
            report.force = tyre_force(settings, report.slip, load);

            const Vector3T<T> impulse =
                (forward * report.force.longitudinal + lateral * report.force.lateral) * tick;
            if (!(dot(impulse, impulse) > T(0)))
                return report;

            apply_velocity_impulse(wheel_body, impulse, patch - wheel_body.position, T(1));
            solver.write_body(wheel, wheel_body);
            tyre_spend_reaction(solver, wheel_slot, impulse, patch, load, substep);
            return report;
        }

    } // namespace Physics
} // namespace SushiEngine
