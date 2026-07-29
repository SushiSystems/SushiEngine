/**************************************************************************/
/* contact_constraint.hpp                                                 */
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
 * @file contact_constraint.hpp
 * @brief A contact as a constraint the solve graph can hold: descriptor and projections.
 *
 * §6.3's *kind dimension*, made concrete for the one kind that matters most. A
 * contact is not special in the solver's eyes: it names two body slots, it takes a
 * colour from the same colouring every other constraint takes one from, and it is
 * projected inside the substep loop. What is special is only its **lifetime** — the
 * set is rebuilt every tick from that tick's manifolds rather than living across
 * ticks like a joint — and that is a property of the *store*, not of the descriptor
 * or of the arithmetic.
 *
 * ### Why the descriptor lives beside the projection rather than in `constraints/`
 *
 * `XpbdDistanceConstraintT` sits in `constraints/` with its projection. A contact's
 * projection is `contact_projection.hpp`, here in `solver/`, because it names
 * `ContactManifold` — and a descriptor is the pair *(what is constrained, under what
 * coefficients)*, where the coefficients are `ContactSolveParams`, defined by the
 * projection that consumes them. Splitting the two across modules to satisfy a folder
 * name would mean `constraints/` including `solver/`, which is the dependency running
 * backwards.
 *
 * ### Static geometry has no body
 *
 * A crate resting on a cooked triangle mesh has one body, not two. Rather than a
 * second projection for the one-body case — which is how a plane contact and a pair
 * contact end up disagreeing, the mistake §1.3 already recorded once — the missing
 * side is named @ref null_contact_body and every projection here substitutes
 * @ref immovable_body for it. Its inverse mass is zero, so every correction it would
 * take is multiplied by zero and the other body takes all of it, which is what
 * "immovable" means.
 *
 * ### The substep schedule these three project into
 *
 * ```
 *   per tick:
 *     warm start                    (the caller, from last tick's manifolds)
 *     for each substep:
 *       ContactPreparationT         <- capture arrival speed, clear the accumulators
 *       predict
 *       ...distance constraints, joints...
 *       ContactPositionProjectionT  <- non-penetration + static friction
 *       update_velocity
 *       ContactVelocityProjectionT  <- dynamic friction + restitution
 * ```
 *
 * The three are separate callables rather than one because the solve graph needs
 * them as three nodes: two of them write bodies and must be ordered around the
 * predict and the velocity derivation, and the first must run *before* predict has
 * touched a velocity restitution is a statement about.
 */

#include <cstddef>
#include <cstdint>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/collision/manifold.hpp>
#include <SushiEngine/physics/core/rigid_body.hpp>
#include <SushiEngine/physics/solver/contact_projection.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief The body slot standing for "static geometry", which has no body.
         *
         * Deliberately the same sentinel value for every projection here, and
         * deliberately *not* a flag: a flag would be a second thing to keep in step
         * with the slot, and a contact whose flag and slot disagreed would resolve
         * against whichever body happened to occupy slot `0xFFFFFFFF`'s neighbour.
         */
        inline constexpr std::uint32_t null_contact_body = 0xFFFFFFFFu;

        /**
         * @brief One manifold, the two slots it holds, and the coefficients it solves under.
         *
         * Exposes `a`/`b` in the shape the colouring expects, so a contact colours
         * against joints and distance constraints in one union without the colourer
         * knowing a contact exists (§6.3).
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        struct ContactConstraintT
        {
            /** @brief The scalar element type, so a solver can derive its precision. */
            using Real = T;

            /** @brief First body slot; the normal points away from it. */
            std::uint32_t a = 0;

            /** @brief Second body slot, or @ref null_contact_body for static geometry. */
            std::uint32_t b = null_contact_body;

            /**
             * @brief The caller's identity for this contact, carried through untouched.
             *
             * The solver never reads it. It exists so a caller reading the solved
             * contacts back can match each one to the persistent manifold it came
             * from — which is what warm starting needs and what contact events are
             * reported against.
             */
            std::uint64_t key = 0;

            /** @brief The manifold; its accumulators are updated in place by the solve. */
            ContactManifold<T> manifold;

            /** @brief The combined coefficients, resolved once from the two materials. */
            ContactSolveParams<T> params;
        };

        /**
         * @brief Whether a contact names a slot the solver can address.
         *
         * @param contact  The contact to check.
         * @param capacity The solver's body capacity.
         * @return True when the contact can be projected at all.
         */
        template <typename T>
        inline bool contact_slots_valid(const ContactConstraintT<T>& contact,
                                        std::size_t capacity) noexcept
        {
            if (contact.a >= capacity)
                return false;
            return contact.b == null_contact_body || contact.b < capacity;
        }

        /**
         * @brief Captures arrival speed and clears the per-substep accumulators.
         *
         * Runs at the top of a substep, before `predict`. Two things happen here and
         * they are one node because they are one moment in the schedule:
         *
         * - **The arrival speed is captured.** By the time restitution is applied the
         *   positional solve has removed the very speed restitution is a statement
         *   about, so it has to be recorded while it still exists.
         * - **The accumulators are cleared** — on every substep but the first. They
         *   measure one substep's impulse, so carrying them across substeps would
         *   inflate the friction budget by the substep count and a sliding box would
         *   stop dead. The first substep is skipped so that what warm starting
         *   inherited from the previous tick survives to bound the friction cone from
         *   the very first projection, which is what stops a box on a ramp creeping
         *   for the one substep it would otherwise take to build a normal impulse.
         */
        template <typename T>
        struct ContactPreparationT
        {
            /**
             * @param contact       The contact; its manifold is updated in place.
             * @param bodies        The body array the slots index.
             * @param first_substep Whether this is the tick's first substep.
             */
            void operator()(ContactConstraintT<T>& contact, const RigidBodyT<T>* bodies,
                            bool first_substep) const noexcept
            {
                const RigidBodyT<T> immovable = immovable_body<T>();
                const RigidBodyT<T>& body_a = bodies[contact.a];
                const RigidBodyT<T>& body_b =
                    contact.b == null_contact_body ? immovable : bodies[contact.b];

                capture_contact_velocities(contact.manifold, body_a, body_b);
                if (!first_substep)
                    clear_manifold_impulses(contact.manifold);
            }
        };

        /**
         * @brief Non-penetration and static friction, positionally, for one contact.
         *
         * Runs after `predict` and after the persistent constraint kinds, in the same
         * place `XpbdDistanceProjectionT` runs: it corrects *positions*, and every
         * positional projection in a substep belongs together.
         */
        template <typename T>
        struct ContactPositionProjectionT
        {
            /**
             * @param contact The contact; its accumulators are updated in place.
             * @param bodies  The body array the slots index; corrected in place.
             */
            void operator()(ContactConstraintT<T>& contact,
                            RigidBodyT<T>* bodies) const noexcept
            {
                RigidBodyT<T> immovable = immovable_body<T>();
                RigidBodyT<T>& body_a = bodies[contact.a];
                RigidBodyT<T>& body_b =
                    contact.b == null_contact_body ? immovable : bodies[contact.b];

                solve_manifold_positions(contact.manifold, body_a, body_b, contact.params);
            }
        };

        /**
         * @brief Dynamic friction and restitution, for one contact, after the pose solve.
         *
         * Runs after `update_velocity`, because both quantities it applies are
         * statements about a velocity that does not exist until the pose change has
         * been read back as one.
         */
        template <typename T>
        struct ContactVelocityProjectionT
        {
            /**
             * @param contact The contact; read for its accumulators and coefficients.
             * @param bodies  The body array the slots index; velocities updated in place.
             * @param h       The substep duration, in seconds.
             */
            void operator()(ContactConstraintT<T>& contact, RigidBodyT<T>* bodies,
                            T h) const noexcept
            {
                RigidBodyT<T> immovable = immovable_body<T>();
                RigidBodyT<T>& body_a = bodies[contact.a];
                RigidBodyT<T>& body_b =
                    contact.b == null_contact_body ? immovable : bodies[contact.b];

                solve_manifold_velocities(contact.manifold, body_a, body_b, contact.params, h);
            }
        };

        /**
         * @brief The boundary contact constraint: @ref ContactConstraintT fixed to `Scalar`.
         */
        using ContactConstraint = ContactConstraintT<Scalar>;
    } // namespace Physics
} // namespace SushiEngine
