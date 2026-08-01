/**************************************************************************/
/* soft_body_collision.hpp                                                */
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
 * @file soft_body_collision.hpp
 * @brief §9.6's shared vocabulary: a soft body's contact surface, and the seam
 *        its solve reaches contacts through.
 *
 * §9.6 answers three problems with three different mechanisms — a distance-field
 * query against a rigid body, a hierarchy-against-hierarchy sweep between two
 * soft bodies, and a spatial hash within one — and a soft body may be subject to
 * all three at once. What they share is *when* they run, not how they work: a
 * contact set is built once per tick (§6.1), its accumulators are cleared at the
 * top of every substep but the first, its positions are projected inside the same
 * substep the elements are projected in, and its velocities are solved after the
 * velocities are derived. That schedule is what @ref ISoftBodyCollider names, and
 * it is the only thing `FiniteElementModel` needs to know about collision — the
 * model never names a distance field, a hierarchy or a hash.
 *
 * The constitutive material (`soft_body_material.hpp`) deliberately does not grow
 * these fields. What a body is *made of* belongs to one element's projection;
 * what its surface is *like* belongs to a pair, resolves through the same combine
 * modes every rigid contact already uses, and is authored per instance rather
 * than per material — a rubber ball and a rubber tyre share a constitutive model
 * and want different collision thicknesses.
 */

#include <cstddef>
#include <cstdint>
#include <vector>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/core/material.hpp>
#include <SushiEngine/physics/core/rigid_body.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief One soft body's contact surface: how thick it is, what it is
         *        like to touch, and which of §9.6's tests it opts into.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        struct SoftBodyCollisionSettings
        {
            /**
             * @brief The half-width the surface presents to a contact, in metres.
             *
             * A simulated surface is a zero-thickness sheet of vertices and
             * triangles, and two zero-thickness sheets can only be *exactly*
             * touching or already crossed. The thickness is what turns that into
             * an inequality with room to solve in: it becomes the contact's
             * `rest_offset` (§7.6), so a resting particle settles this far outside
             * whatever it lies on rather than exactly on it.
             */
            T thickness = T(0.01);

            /** @brief The surface's friction and restitution, combined with whatever it touches. */
            PhysicsMaterialT<T> surface{};

            /**
             * @brief Whether this body's own surface is tested against itself (§9.6.3).
             *
             * Off by default because it is the expensive one — every other test
             * here is against a structure something else already built, and this
             * one builds a hash over the body's own surface every tick.
             */
            bool self_collision = false;

            /**
             * @brief Whether soft-vs-soft and self tests sweep the substep (§9.6.2).
             *
             * Continuous (Bridson) tests catch a thin body passing through a
             * surface within one substep, which a discrete test cannot see at all.
             * A thick body cannot cross itself in a substep to begin with, so it
             * pays for the sweep and gets nothing — hence a per-body choice rather
             * than a global one.
             */
            bool continuous = false;
        };

        /** @brief The boundary settings type: @ref SoftBodyCollisionSettings fixed to `Scalar`. */
        using SoftBodyCollisionSettingsDefault = SoftBodyCollisionSettings<Scalar>;

        /**
         * @brief One soft body's collidable surface, as the collision code needs it.
         *
         * Deliberately not `FiniteElementModel`: the surface a contact acts on is
         * particles and triangles, and every model kind §9.7's levels of detail
         * swap between has those. Naming the model here would tie soft-body
         * collision to one of them.
         *
         * It is also what `ISoftBodyModel` (§3.3) hands out, for the same reason
         * in the other direction — a consumer that wants a body's surface wants
         * these five fields whichever model is currently simulating it.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        struct SoftSurfaceView
        {
            /** @brief The body's particles; borrowed, and re-read every tick. */
            RigidBodyT<T>* particles = nullptr;
            std::size_t particle_count = 0;
            /** @brief Three particle indices per surface triangle. */
            const std::uint32_t* surface_indices = nullptr;
            std::size_t index_count = 0;
            /** @brief The surface's thickness and what it is like to touch. */
            SoftBodyCollisionSettings<T> collision{};
        };

        /**
         * @brief The contacts acting on one soft body, as a schedule rather than a mechanism.
         *
         * An implementation owns whatever structure its test needs — a distance
         * field, another body's hierarchy, a spatial hash — and exposes only the
         * four moments a soft-body step has to call it at. The model holds a
         * pointer to this interface and nothing else, which is what keeps
         * `physics/soft/` free of any statement about *what* a soft body is
         * touching (§4.5).
         *
         * Every method takes the particle array rather than storing it, because a
         * model's particles move in memory when fracture (§9.5) duplicates a
         * vertex, and a collider holding a stale pointer across that would be a
         * use-after-free that only a fracturing scene reaches.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        class ISoftBodyCollider
        {
            public:
                virtual ~ISoftBodyCollider() = default;

                /**
                 * @brief Builds this tick's contact set, once, before the substep loop.
                 *
                 * **The tick duration is not decoration.** Contacts are found once per
                 * tick and resolved every substep (§6.1), so the set has to cover
                 * every feature pair that could come within touching distance at any
                 * point during the tick — not merely the pairs touching at the moment
                 * it is built. A body falling at 1 m/s covers 16 mm in a 60 Hz tick,
                 * which is more than a centimetre-thick surface, so a set built
                 * against the opening pose alone contains nothing at all and the body
                 * passes cleanly through whatever it was about to land on. @p dt is
                 * what lets an implementation widen its acceptance by the distance
                 * its particles can actually travel.
                 *
                 * The contacts this produces are **speculative**: a pair admitted
                 * because it might touch is not yet touching, and the projection is an
                 * inequality that does nothing until the separation really does fall
                 * below the surface thickness. So widening costs a slightly longer
                 * contact list and never a spurious push.
                 *
                 * @param particles      The body's particles at the tick's start.
                 * @param particle_count How many; indices at or beyond it are ignored.
                 * @param dt             The tick's duration, in seconds.
                 */
                virtual void generate_contacts(const RigidBodyT<T>* particles,
                                               std::size_t particle_count, T dt) = 0;

                /**
                 * @brief Records each contact's closing speed, at the top of a substep.
                 *
                 * Called before `predict`, because by the time restitution is applied
                 * the positional solve has already removed the speed restitution is a
                 * statement about (§7.4).
                 *
                 * @param particles The body's particles.
                 */
                virtual void capture_velocities(const RigidBodyT<T>* particles) noexcept = 0;

                /**
                 * @brief Projects non-penetration and static friction, inside the substep's position solve.
                 *
                 * @param particles      The body's particles; positions updated in place.
                 * @param substep_index  Which substep this is; the accumulators are
                 *                       cleared on every one but the first, so what
                 *                       the previous tick left is used once and then
                 *                       stops inflating the friction budget.
                 * @param h              The substep duration, in seconds.
                 */
                virtual void project_positions(RigidBodyT<T>* particles, std::size_t substep_index,
                                               T h) = 0;

                /**
                 * @brief Applies dynamic friction and restitution, after the velocities are derived.
                 *
                 * @param particles The body's particles; velocities updated in place.
                 * @param h         The substep duration, in seconds (> 0).
                 */
                virtual void solve_velocities(RigidBodyT<T>* particles, T h) = 0;
        };

        /**
         * @brief The contacts between *two* soft bodies, on the same schedule.
         *
         * A single-body collider cannot express this: both sides move, both take
         * corrections, and neither owns the pair. So a pair collider is driven by
         * whatever holds both bodies — `SoftBodyScene` — rather than by either
         * body's own step, and it reaches its particles itself instead of being
         * handed one array.
         *
         * The moments are the single-body ones with one addition: the tick's
         * duration reaches @ref generate_contacts, because a swept broad phase has
         * to know how far the two surfaces can travel before it can widen its
         * bounds by the right amount.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        class ISoftBodyPairCollider
        {
            public:
                virtual ~ISoftBodyPairCollider() = default;

                /**
                 * @brief Builds this tick's contact set between the two bodies.
                 * @param dt The tick's duration, in seconds.
                 */
                virtual void generate_contacts(T dt) = 0;

                /** @brief Records each contact's closing speed, at the top of a substep. */
                virtual void capture_velocities() noexcept = 0;

                /**
                 * @brief Projects non-penetration and static friction inside a substep.
                 *
                 * @param substep_index Which substep this is.
                 * @param h             The substep duration, in seconds.
                 */
                virtual void project_positions(std::size_t substep_index, T h) = 0;

                /**
                 * @brief Applies dynamic friction and restitution after the velocities are derived.
                 * @param h The substep duration, in seconds (> 0).
                 */
                virtual void solve_velocities(T h) = 0;
        };

        /**
         * @brief Several colliders acting on one body, as one collider.
         *
         * A body in a real scene is touching a floor, another soft body, and
         * possibly itself, all in the same tick — three different mechanisms
         * against three different structures. Rather than teach the model to hold
         * a list, the list *is* a collider: each moment is forwarded to every
         * member in the order they were added, which is a fixed order and
         * therefore a deterministic one (§0.5).
         *
         * The members are borrowed, not owned, in the same shape every other
         * borrowed reference in `physics/` takes — the scene that built them
         * outlives the tick that uses them.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        class SoftBodyColliderSet final : public ISoftBodyCollider<T>
        {
            public:
                /**
                 * @brief Adds a collider to the end of the set.
                 *
                 * @param collider The collider; ignored when null, so a caller
                 *                 assembling a set from optional parts does not
                 *                 have to branch around the ones it lacks.
                 */
                void add(ISoftBodyCollider<T>* collider)
                {
                    if (collider != nullptr)
                        members_.push_back(collider);
                }

                /** @brief Empties the set, without destroying anything in it. */
                void clear() noexcept
                {
                    members_.clear();
                }

                /** @brief How many colliders act on this body. */
                std::size_t size() const noexcept
                {
                    return members_.size();
                }

                void generate_contacts(const RigidBodyT<T>* particles,
                                       std::size_t particle_count, T dt) override
                {
                    for (ISoftBodyCollider<T>* member : members_)
                        member->generate_contacts(particles, particle_count, dt);
                }

                void capture_velocities(const RigidBodyT<T>* particles) noexcept override
                {
                    for (ISoftBodyCollider<T>* member : members_)
                        member->capture_velocities(particles);
                }

                void project_positions(RigidBodyT<T>* particles, std::size_t substep_index,
                                       T h) override
                {
                    for (ISoftBodyCollider<T>* member : members_)
                        member->project_positions(particles, substep_index, h);
                }

                void solve_velocities(RigidBodyT<T>* particles, T h) override
                {
                    for (ISoftBodyCollider<T>* member : members_)
                        member->solve_velocities(particles, h);
                }

            private:
                std::vector<ISoftBodyCollider<T>*> members_;
        };
    } // namespace Physics
} // namespace SushiEngine
