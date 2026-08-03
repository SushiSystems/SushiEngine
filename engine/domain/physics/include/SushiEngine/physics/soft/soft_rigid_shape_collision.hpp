/**************************************************************************/
/* soft_rigid_shape_collision.hpp                                         */
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
 * @file soft_rigid_shape_collision.hpp
 * @brief §9.6.1's answer for a rigid partner with no baked distance field.
 *
 * `soft_rigid_collision.hpp` reaches a soft body's contacts through a cooked
 * signed-distance field, which is the right representation for a mesh but is
 * not what most authored rigid bodies are: a sphere, a box, a capsule or an
 * infinite plane, none of which have to be baked into a grid to answer "how
 * far, which way" — each has a closed form, and a closed form is exact where
 * a grid is only as fine as its resolution.
 *
 * This file is that closed form, reusing the exact per-shape math the rigid
 * narrowphase already trusts (`collision/manifold.hpp`'s clamp-to-box and
 * nearest-face routines) but written with the particle, not the other rigid
 * body, as side "a" — the convention `generate_particle_sdf_manifold` already
 * established: the normal runs from the particle **toward** the solid.
 *
 * A plane is authored directly in world space (§8.6, `Simulation::Collider`'s
 * own convention: a plane's normal and offset are not relative to the body
 * that carries them), so a plane contact always resolves against
 * `immovable_body()` — passing a real, moving rigid body as its partner would
 * apply that body's pose to an anchor the plane's geometry never used to
 * begin with, which is the inconsistency `immovable_body()`'s own comment
 * warns against. A sphere, box or capsule *does* move with its rigid body, so
 * those three carry a live partner and get two-way coupling for it, on
 * exactly the same terms `soft_rigid_collision.hpp` already documented.
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
         * @brief Manifold between a particle and a solid sphere, particle as side "a".
         *
         * @tparam T The scalar element type.
         * @param position       The particle's world position.
         * @param sphere         The sphere, placed in the world.
         * @param contact_offset Contacts are generated out to this separation (§7.5).
         * @return A manifold with `point_count == 0` when nothing is within range.
         */
        template <typename T>
        inline ContactManifold<T> generate_particle_sphere_manifold(
            const Vector3T<T>& position, const SphereCollider<T>& sphere,
            T contact_offset) noexcept
        {
            const QuaternionT<T> identity{T(0), T(0), T(0), T(1)};
            const Vector3T<T> outward = position - sphere.center;
            const T distance = length(outward);
            const T separation = distance - sphere.radius;
            if (separation > contact_offset)
                return ContactManifold<T>{};

            const Vector3T<T> gradient =
                distance > T(1e-8) ? outward * (T(1) / distance) : Vector3T<T>{T(0), T(1), T(0)};
            const Vector3T<T> point_on_sphere = sphere.center + gradient * sphere.radius;
            return make_point_manifold(gradient * T(-1), position, point_on_sphere, separation,
                                       position, identity, sphere.center, identity,
                                       make_feature_id(0, 0, 0, false));
        }

        /**
         * @brief Manifold between a particle and an infinite half-space plane.
         *
         * Always meant to resolve against `immovable_body()` (see this file's
         * header): the anchor this writes for "b" is a world point, which is the
         * frame `immovable_body()`'s own comment names as the one a plane's anchors
         * belong in.
         *
         * @tparam T The scalar element type.
         * @param position       The particle's world position.
         * @param plane          The plane, in world space.
         * @param contact_offset Contacts are generated out to this separation (§7.5).
         */
        template <typename T>
        inline ContactManifold<T> generate_particle_plane_manifold(
            const Vector3T<T>& position, const PlaneCollider<T>& plane, T contact_offset) noexcept
        {
            const QuaternionT<T> identity{T(0), T(0), T(0), T(1)};
            const T separation = dot(plane.normal, position) - plane.offset;
            if (separation > contact_offset)
                return ContactManifold<T>{};

            const Vector3T<T> point_on_plane = position - plane.normal * separation;
            return make_point_manifold(plane.normal * T(-1), position, point_on_plane, separation,
                                       position, identity, Vector3T<T>{T(0), T(0), T(0)}, identity,
                                       make_feature_id(0, 0, 0, false));
        }

        /**
         * @brief Manifold between a particle and an oriented box.
         *
         * The same clamp-to-box-in-local-space test `generate_obb_sphere_manifold`
         * runs for a sphere of zero radius — restated here with the particle as
         * side "a" so the manifold's anchors track the particle's own frame the
         * way every other particle-vs-rigid manifold in `physics/soft/` does,
         * rather than needing a caller to reinterpret which side is which.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        inline ContactManifold<T> generate_particle_box_manifold(const Vector3T<T>& position,
                                                                  const OrientedBox<T>& box,
                                                                  T contact_offset) noexcept
        {
            const QuaternionT<T> identity{T(0), T(0), T(0), T(1)};
            const auto clamp = [](T value, T low, T high) noexcept
            { return value < low ? low : (value > high ? high : value); };

            const Vector3T<T> local = rotate(conjugate(box.orientation), position - box.center);
            const Vector3T<T> closest{clamp(local.x, -box.half_extents.x, box.half_extents.x),
                                      clamp(local.y, -box.half_extents.y, box.half_extents.y),
                                      clamp(local.z, -box.half_extents.z, box.half_extents.z)};
            const Vector3T<T> delta = local - closest;
            const T distance = length(delta);

            Vector3T<T> gradient_local;
            Vector3T<T> surface_local;
            T separation;
            if (distance <= T(1e-8))
            {
                // Inside the box: clamping returns the point itself and no outward
                // direction exists, so the particle is pushed out through whichever
                // face it is nearest to (`nearest_face_normal`'s own reason for being).
                gradient_local = nearest_face_normal(local, box.half_extents);
                separation = -nearest_face_distance(local, box.half_extents);
                surface_local = local;
            }
            else
            {
                gradient_local = delta * (T(1) / distance);
                separation = distance;
                surface_local = closest;
                if (separation > contact_offset)
                    return ContactManifold<T>{};
            }

            const Vector3T<T> gradient = rotate(box.orientation, gradient_local);
            const Vector3T<T> point_on_box = box.center + rotate(box.orientation, surface_local);
            return make_point_manifold(gradient * T(-1), position, point_on_box, separation,
                                       position, identity, box.center, box.orientation,
                                       make_feature_id(0, 0, 0, false));
        }

        /**
         * @brief Manifold between a particle and a capsule (a swept-sphere segment).
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        inline ContactManifold<T> generate_particle_capsule_manifold(
            const Vector3T<T>& position, const CapsuleCollider<T>& capsule,
            T contact_offset) noexcept
        {
            const QuaternionT<T> identity{T(0), T(0), T(0), T(1)};
            const Vector3T<T> local = rotate(conjugate(capsule.orientation), position - capsule.center);
            const T clamped_y = local.y < -capsule.half_height
                                    ? -capsule.half_height
                                    : (local.y > capsule.half_height ? capsule.half_height : local.y);
            const Vector3T<T> closest_local{T(0), clamped_y, T(0)};
            const Vector3T<T> delta = local - closest_local;
            const T distance = length(delta);
            const T separation = distance - capsule.radius;
            if (separation > contact_offset)
                return ContactManifold<T>{};

            const Vector3T<T> gradient_local =
                distance > T(1e-8) ? delta * (T(1) / distance) : Vector3T<T>{T(1), T(0), T(0)};
            const Vector3T<T> gradient = rotate(capsule.orientation, gradient_local);
            const Vector3T<T> closest_world = capsule.center + rotate(capsule.orientation, closest_local);
            const Vector3T<T> point_on_capsule = closest_world + gradient * capsule.radius;
            return make_point_manifold(gradient * T(-1), position, point_on_capsule, separation,
                                       position, identity, capsule.center, capsule.orientation,
                                       make_feature_id(0, 0, 0, false));
        }

        /** @brief Which of the four closed-form shapes a @ref SoftRigidPrimitiveCollider tests against. */
        enum class SoftRigidPrimitiveKind
        {
            Sphere,
            Box,
            Plane,
            Capsule
        };

        /**
         * @brief One surface particle's contact with a closed-form primitive shape.
         * @tparam T The scalar element type.
         */
        template <typename T>
        struct SoftRigidPrimitiveContact
        {
            std::uint32_t particle = 0;
            ContactManifold<T> manifold;
        };

        /**
         * @brief §9.6.1 for the three primitive shapes and the plane, without a baked field.
         *
         * Configured by assignment, the same shape every other physics descriptor
         * in `physics/soft/` takes: an owner (`sim/`) fills in which shape this is,
         * its world-space parameters, and — for a sphere, box or capsule — the
         * rigid body it moves with, then refreshes the shape fields once per tick
         * before `generate_contacts` runs, exactly as `SoftRigidCollider::field`
         * already documents.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        class SoftRigidPrimitiveCollider final : public ISoftBodyCollider<T>
        {
            public:
                /** @brief Which shape @ref generate_contacts tests against. */
                SoftRigidPrimitiveKind kind = SoftRigidPrimitiveKind::Box;

                /** @brief Valid when @ref kind is @ref SoftRigidPrimitiveKind::Sphere. */
                SphereCollider<T> sphere{};
                /** @brief Valid when @ref kind is @ref SoftRigidPrimitiveKind::Box. */
                OrientedBox<T> box{};
                /** @brief Valid when @ref kind is @ref SoftRigidPrimitiveKind::Plane; world space always. */
                PlaneCollider<T> plane{};
                /** @brief Valid when @ref kind is @ref SoftRigidPrimitiveKind::Capsule. */
                CapsuleCollider<T> capsule{};

                /**
                 * @brief Whether the shape fields above hold a real placement yet.
                 *
                 * Starts false, the same way `SoftRigidCollider::field.distances ==
                 * nullptr` reads as "not configured": the owner refreshes a shape's
                 * placement once per tick from wherever the rigid body actually is,
                 * and a soft body stepped before that first refresh must not collide
                 * against whatever the shape's default-constructed fields happen to
                 * describe (an oriented box defaults to a unit cube sitting at the
                 * world origin, which is a real place a real body could be standing).
                 */
                bool configured = false;

                /**
                 * @brief The body this shape moves with, or null for immovable geometry.
                 *
                 * A plane always resolves against `immovable_body()` regardless of this
                 * field (see this file's header) — set it for a sphere, box or capsule
                 * so the correction couples back into whatever the shape belongs to.
                 */
                RigidBodyT<T>* rigid = nullptr;

                /** @brief The soft body's surface particle indices, ascending. */
                const std::uint32_t* surface_vertices = nullptr;
                /** @brief How many entries @ref surface_vertices holds. */
                std::size_t surface_vertex_count = 0;

                /** @brief The combined coefficients; `rest_offset` is the surface's thickness. */
                ContactSolveParameters<T> parameters{};

                /** @brief How far beyond the rest offset a contact is still generated (§7.5). */
                T contact_offset = 0;

                void generate_contacts(const RigidBodyT<T>* particles,
                                       std::size_t particle_count, T dt) override
                {
                    previous_.swap(contacts_);
                    contacts_.clear();
                    if (!configured || particles == nullptr || surface_vertices == nullptr)
                        return;

                    // Widened by how far the fastest particle travels this tick, for the
                    // same reason `SoftRigidCollider` is (§9.6's speculative contract): a
                    // set built only from where the surface is *now* is empty for a body
                    // that arrives and departs between two calls to this method.
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

                        const ContactManifold<T> manifold =
                            manifold_for(particles[index].position, threshold);
                        if (manifold.point_count == 0)
                            continue;

                        SoftRigidPrimitiveContact<T> contact;
                        contact.particle = index;
                        contact.manifold = manifold;

                        while (inherited < previous_.size() && previous_[inherited].particle < index)
                            ++inherited;
                        if (inherited < previous_.size() && previous_[inherited].particle == index)
                        {
                            const ContactPoint<T>& before = previous_[inherited].manifold.points[0];
                            contact.manifold.points[0].normal_lambda = before.normal_lambda;
                            contact.manifold.points[0].tangent_lambda[0] = before.tangent_lambda[0];
                            contact.manifold.points[0].tangent_lambda[1] = before.tangent_lambda[1];
                        }

                        contacts_.push_back(contact);
                    }
                }

                void capture_velocities(const RigidBodyT<T>* particles) noexcept override
                {
                    if (particles == nullptr)
                        return;
                    for (SoftRigidPrimitiveContact<T>& contact : contacts_)
                        capture_contact_velocities(contact.manifold, particles[contact.particle],
                                                   partner());
                }

                void project_positions(RigidBodyT<T>* particles, std::size_t substep_index,
                                       T h) override
                {
                    (void)h;
                    if (particles == nullptr)
                        return;
                    for (SoftRigidPrimitiveContact<T>& contact : contacts_)
                    {
                        if (substep_index > 0)
                            clear_manifold_impulses(contact.manifold);
                        solve_manifold_positions(contact.manifold, particles[contact.particle],
                                                 partner(), parameters);
                    }
                }

                void solve_velocities(RigidBodyT<T>* particles, T h) override
                {
                    if (particles == nullptr)
                        return;
                    for (SoftRigidPrimitiveContact<T>& contact : contacts_)
                        solve_manifold_velocities(contact.manifold, particles[contact.particle],
                                                  partner(), parameters, h);
                }

                /** @brief This tick's contacts, ascending by particle index. */
                const std::vector<SoftRigidPrimitiveContact<T>>& contacts() const noexcept
                {
                    return contacts_;
                }

            private:
                ContactManifold<T> manifold_for(const Vector3T<T>& position, T threshold) const noexcept
                {
                    switch (kind)
                    {
                        case SoftRigidPrimitiveKind::Sphere:
                            return generate_particle_sphere_manifold(position, sphere, threshold);
                        case SoftRigidPrimitiveKind::Box:
                            return generate_particle_box_manifold(position, box, threshold);
                        case SoftRigidPrimitiveKind::Plane:
                            return generate_particle_plane_manifold(position, plane, threshold);
                        case SoftRigidPrimitiveKind::Capsule:
                            return generate_particle_capsule_manifold(position, capsule, threshold);
                    }
                    return ContactManifold<T>{};
                }

                /** @brief The manifold's second body: the rigid one, or immovable geometry. */
                RigidBodyT<T>& partner() noexcept
                {
                    // A plane's geometry never reads a body pose (this file's header),
                    // so it always resolves against the world-frame placeholder even
                    // when an owner mistakenly left `rigid` set.
                    return (rigid != nullptr && kind != SoftRigidPrimitiveKind::Plane) ? *rigid
                                                                                       : immovable_;
                }

                RigidBodyT<T> immovable_{immovable_body<T>()};
                std::vector<SoftRigidPrimitiveContact<T>> contacts_;
                std::vector<SoftRigidPrimitiveContact<T>> previous_;
        };
    } // namespace Physics
} // namespace SushiEngine
