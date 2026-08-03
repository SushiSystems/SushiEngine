/**************************************************************************/
/* soft_rigid_collision.hpp                                               */
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
 * @file soft_rigid_collision.hpp
 * @brief §9.6's first answer: a soft body's surface against a rigid body's
 *        cooked signed-distance field.
 *
 * This is *the* reason §8.4 bakes a distance field for every rigid collider. A
 * surface vertex is a point, and a point against a field is one sample and one
 * gradient: `O(1)`, exact depth however deep it has sunk, and a normal that is
 * the eikonal gradient rather than a guess about which triangle is nearest
 * (`collision/sdf_manifold.hpp` records the same argument for a convex shape).
 * No hierarchy is traversed, no feature is reconstructed, and a vertex a metre
 * inside the solid is answered as cheaply and as correctly as one grazing it.
 *
 * **Nothing here is a second contact solver.** Each contacting vertex becomes an
 * ordinary one-point `ContactManifold` and is handed to `solve_manifold_positions`
 * and `solve_manifold_velocities` — the same projections a crate on a floor uses,
 * so a soft body's friction, restitution, rest offset and depenetration budget
 * are the *same* code with the same behaviour, not a parallel implementation that
 * drifts. A particle is a body with zero inverse inertia, which those projections
 * already support: `generalized_inverse_mass` with a zero lever is just the
 * inverse mass, so the angular path multiplies out rather than needing a case.
 *
 * That is also what makes the coupling two-way for free. The rigid body is the
 * manifold's second body, so it takes its share of every correction weighted by
 * generalized inverse mass — a soft body dropped on a light crate pushes it, and
 * a soft body dropped on the world's static geometry (a partner with
 * `BodyFlags::static_body`) does not, because an unsimulated body presents no
 * inverse mass to begin with.
 */

#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/collision/manifold.hpp>
#include <SushiEngine/physics/core/rigid_body.hpp>
#include <SushiEngine/physics/geometry/shapes.hpp>
#include <SushiEngine/physics/soft/soft_body_collision.hpp>
#include <SushiEngine/physics/solver/contact_projection.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief One surface particle's contact with a distance field.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        struct SoftRigidContact
        {
            /** @brief Which particle is touching, indexing the body's particle array. */
            std::uint32_t particle = 0;

            /** @brief The one-point manifold between that particle and the field. */
            ContactManifold<T> manifold;
        };

        /**
         * @brief The manifold between a point and a signed-distance field.
         *
         * Two field reads: the signed distance says how far the point is from the
         * surface and on which side, and the gradient at the same point is the
         * outward surface normal there. The nearest surface point follows without
         * a search — stepping back along the gradient by exactly the distance
         * lands on it, which is what the eikonal property `|grad d| = 1` buys.
         *
         * The manifold's convention is the engine-wide one: the normal runs from
         * `a` (the particle) toward `b` (the field's solid), so it is the negated
         * gradient, and the separation is the signed distance itself — negative
         * once the particle is inside.
         *
         * @tparam T The scalar element type.
         * @param position       The particle's world-space position.
         * @param field          The field, placed in the world.
         * @param contact_offset Contacts are generated out to this separation, so
         *                       an approaching particle is constrained before it
         *                       overlaps (§7.5) and a resting one is not dropped
         *                       the moment it reaches its rest offset.
         * @return A manifold with `point_count == 0` when the particle is further
         *         away than @p contact_offset or the field is empty.
         */
        template <typename T>
        inline ContactManifold<T> generate_particle_sdf_manifold(const Vector3T<T>& position,
                                                                 const SDFCollider<T>& field,
                                                                 T contact_offset) noexcept
        {
            if (field.distances == nullptr || field.resolution <= 0)
                return ContactManifold<T>{};

            // Interpolated rather than nearest-voxel: a surface is a whole sheet of
            // these queries, so a piecewise-constant field would settle it onto a
            // staircase, and a central difference inside one voxel reads zero and
            // hands back the fixed-axis guard instead of a normal
            // (`sdf_sample_interpolated_local` records the argument in full).
            Vector3T<T> gradient;
            const T distance = sdf_sample_interpolated_world(field, position, gradient);
            if (distance == std::numeric_limits<T>::max() || distance > contact_offset)
                return ContactManifold<T>{};

            const Vector3T<T> point_on_field = position - gradient * distance;
            const QuaternionT<T> identity{T(0), T(0), T(0), T(1)};
            return make_point_manifold(gradient * T(-1), position, point_on_field, distance,
                                       position, identity, field.center, field.orientation,
                                       make_feature_id(0, 0, 0, false));
        }

        /**
         * @brief §9.6.1: one soft body's surface against one rigid body's distance field.
         *
         * Configured by assignment rather than through a constructor, in the same
         * shape every other physics descriptor in this module is: the owner fills
         * the fields it has and steps the model.
         *
         * **The field's placement is read once per tick**, when the contact set is
         * built, and not again. That is not an approximation — the manifold stores
         * its anchors in each body's *local* frame, so every substep re-derives the
         * contact from the two current poses. A field placement refreshed per
         * substep would change nothing except the contact set, which §6.1 fixes for
         * the tick on purpose.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        class SoftRigidCollider final : public ISoftBodyCollider<T>
        {
            public:
                /** @brief The rigid body's cooked field, already placed in the world. */
                SDFCollider<T> field{};

                /**
                 * @brief The body the field belongs to, or null for immovable geometry.
                 *
                 * Null and a body flagged static mean the same thing to the solve —
                 * neither takes a correction — so a caller with no body to hand over
                 * does not have to invent one.
                 */
                RigidBodyT<T>* rigid = nullptr;

                /**
                 * @brief The soft body's surface particle indices, ascending.
                 *
                 * Ascending is a requirement, not a convention: it is what makes the
                 * contact set's order a function of simulation state rather than of
                 * traversal order (§12.1), and it is what lets the previous tick's
                 * accumulators be carried over in one linear pass.
                 */
                const std::uint32_t* surface_vertices = nullptr;

                /** @brief How many entries @ref surface_vertices holds. */
                std::size_t surface_vertex_count = 0;

                /** @brief The combined coefficients; `rest_offset` is the surface's thickness. */
                ContactSolveParameters<T> parameters{};

                /**
                 * @brief How far beyond the rest offset a contact is still generated.
                 *
                 * Speculative distance (§7.5), measured from the rest offset rather
                 * than from the surface, so raising the thickness does not silently
                 * shrink the window a contact is caught in.
                 */
                T contact_offset = 0;

                /**
                 * @brief Builds this tick's contact set, one manifold per touching surface particle.
                 *
                 * Carries the previous tick's normal and tangent accumulators onto a
                 * contact for the same particle — the warm start §7.3 asks for. Only
                 * the *bound* is inherited, never re-applied as an impulse: a
                 * positional solver that pushes with a force it has not re-derived
                 * from the current poses pushes apart a pair that has already
                 * separated. What it buys is that a resting contact starts the tick
                 * with a friction cone rather than spending a substep building one.
                 *
                 * @param particles      The body's particles at the tick's start.
                 * @param particle_count How many; a surface index at or beyond it is skipped.
                 * @param dt             The tick's duration, in seconds.
                 */
                void generate_contacts(const RigidBodyT<T>* particles,
                                       std::size_t particle_count, T dt) override
                {
                    previous_.swap(contacts_);
                    contacts_.clear();
                    if (particles == nullptr || surface_vertices == nullptr)
                        return;

                    // Widened by how far the fastest particle travels this tick, for
                    // the same reason the pair colliders are: a contact set built
                    // only from where the surface is *now* is empty for a body that
                    // arrives and departs between two of these calls, and an empty
                    // set is indistinguishable from nothing to collide with. The
                    // extra manifolds are speculative — `solve_manifold_positions`
                    // is an inequality and does nothing until the separation is
                    // genuinely negative — so this widens the list, not the forces.
                    T fastest = 0;
                    for (std::size_t i = 0; i < surface_vertex_count; ++i)
                    {
                        const std::uint32_t index = surface_vertices[i];
                        if (std::size_t(index) >= particle_count)
                            continue;
                        const T speed = length(particles[index].velocity);
                        if (speed > fastest)
                            fastest = speed;
                    }

                    const T threshold = parameters.rest_offset + contact_offset +
                                        (dt > T(0) ? fastest * dt : T(0));
                    std::size_t inherited = 0;
                    for (std::size_t i = 0; i < surface_vertex_count; ++i)
                    {
                        const std::uint32_t index = surface_vertices[i];
                        if (std::size_t(index) >= particle_count)
                            continue;

                        ContactManifold<T> manifold = generate_particle_sdf_manifold(
                            particles[index].position, field, threshold);
                        if (manifold.point_count == 0)
                            continue;

                        while (inherited < previous_.size() && previous_[inherited].particle < index)
                            ++inherited;
                        if (inherited < previous_.size() && previous_[inherited].particle == index)
                        {
                            const ContactPoint<T>& before = previous_[inherited].manifold.points[0];
                            manifold.points[0].normal_lambda = before.normal_lambda;
                            manifold.points[0].tangent_lambda[0] = before.tangent_lambda[0];
                            manifold.points[0].tangent_lambda[1] = before.tangent_lambda[1];
                        }

                        SoftRigidContact<T> contact;
                        contact.particle = index;
                        contact.manifold = manifold;
                        contacts_.push_back(contact);
                    }
                }

                /**
                 * @brief Records every contact's closing speed before the substep integrates.
                 * @param particles The body's particles.
                 */
                void capture_velocities(const RigidBodyT<T>* particles) noexcept override
                {
                    if (particles == nullptr)
                        return;
                    for (SoftRigidContact<T>& contact : contacts_)
                        capture_contact_velocities(contact.manifold, particles[contact.particle],
                                                   partner());
                }

                /**
                 * @brief Projects every contact's non-penetration and static friction.
                 *
                 * @param particles     The body's particles; positions updated in place.
                 * @param substep_index Which substep this is; the accumulators are cleared
                 *                      on every one but the first.
                 * @param h             Unused here — a positional projection needs no
                 *                      timestep, since it corrects a distance rather than
                 *                      a rate. Present because the seam's other half does.
                 */
                void project_positions(RigidBodyT<T>* particles, std::size_t substep_index,
                                       T h) override
                {
                    (void)h;
                    if (particles == nullptr)
                        return;
                    for (SoftRigidContact<T>& contact : contacts_)
                    {
                        if (substep_index > 0)
                            clear_manifold_impulses(contact.manifold);
                        solve_manifold_positions(contact.manifold, particles[contact.particle],
                                                 partner(), parameters);
                    }
                }

                /**
                 * @brief Applies dynamic friction and restitution at every contact.
                 * @param particles The body's particles; velocities updated in place.
                 * @param h         The substep duration, in seconds (> 0).
                 */
                void solve_velocities(RigidBodyT<T>* particles, T h) override
                {
                    if (particles == nullptr)
                        return;
                    for (SoftRigidContact<T>& contact : contacts_)
                        solve_manifold_velocities(contact.manifold, particles[contact.particle],
                                                  partner(), parameters, h);
                }

                /**
                 * @brief This tick's contacts, ascending by particle index.
                 * @return The contact set; empty until @ref generate_contacts has run.
                 */
                const std::vector<SoftRigidContact<T>>& contacts() const noexcept
                {
                    return contacts_;
                }

            private:
                /** @brief The manifold's second body: the rigid one, or immovable geometry. */
                RigidBodyT<T>& partner() noexcept
                {
                    return rigid != nullptr ? *rigid : immovable_;
                }

                RigidBodyT<T> immovable_{immovable_body<T>()};
                std::vector<SoftRigidContact<T>> contacts_;
                std::vector<SoftRigidContact<T>> previous_;
        };

        /**
         * @brief The contact coefficients for a soft surface meeting a rigid one.
         *
         * The soft body's thickness becomes the contact's rest offset, which is
         * what makes a surface vertex settle a thickness outside the field rather
         * than exactly on its zero level set.
         *
         * @tparam T The scalar element type.
         * @param settings              The soft body's surface.
         * @param rigid_material        The rigid body's material.
         * @param restitution_threshold The anti-jitter floor, usually `2 * g * h`.
         * @return Parameters ready to assign to @ref SoftRigidCollider::parameters.
         */
        template <typename T>
        inline ContactSolveParameters<T> make_soft_rigid_parameters(
            const SoftBodyCollisionSettings<T>& settings, const PhysicsMaterialT<T>& rigid_material,
            T restitution_threshold) noexcept
        {
            return make_contact_parameters(settings.surface, rigid_material, settings.thickness,
                                           restitution_threshold);
        }
    } // namespace Physics
} // namespace SushiEngine
