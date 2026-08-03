/**************************************************************************/
/* contact_solver.hpp                                                     */
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
 * @file contact_solver.hpp
 * @brief Positional (PBD) contact resolution for rigid bodies and cloth particles.
 *
 * Non-penetration is an inequality constraint — a contact only ever pushes bodies
 * apart, never pulls them together — so it is handled as a projection pass over the
 * predicted positions rather than through the compile-once `XPBDSolver` (whose
 * constraint set is fixed): contacts appear and vanish as bodies move, so they are
 * regenerated from the narrowphase each pass. Run it between `predict` and
 * `update_velocity` in a sub-step: the position change it makes is exactly what
 * `update_velocity` then reads back as the post-contact velocity, so a body landing
 * on a surface loses its downward velocity without any explicit restitution term
 * (inelastic contact).
 *
 * A body collides as an oriented box or a sphere, and a correction is split between
 * the pair by generalized inverse mass, so it moves *and* turns them. What this model
 * still does not have: friction, restitution, a clipped contact manifold (each pair
 * yields one point, so a resting box rocks), and cloth self-collision.
 */

#include <cstddef>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/collision/broadphase.hpp>
#include <SushiEngine/physics/collision/narrowphase.hpp>
#include <SushiEngine/physics/core/rigid_body.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief Pushes every penetrating body out of a static half-space plane.
         *
         * A static plane has infinite mass, so a contacting body is moved out along
         * the plane normal by the full penetration depth. A body with `inv_mass == 0`
         * (pinned) is left untouched.
         *
         * @tparam T The scalar element type.
         * @param bodies The bodies to resolve; their positions are updated in place.
         * @param radii  One collision radius per body.
         * @param count  Number of bodies.
         * @param plane  The static ground/half-space plane.
         */
        template <typename T>
        inline void resolve_plane_contacts(RigidBodyT<T>* bodies, const T* radii,
                                           std::size_t count, const PlaneCollider<T>& plane) noexcept
        {
            for (std::size_t i = 0; i < count; ++i)
            {
                if (bodies[i].inv_mass <= T(0))
                    continue;
                const SphereCollider<T> sphere{bodies[i].position, radii[i]};
                const Contact<T> contact = collide_sphere_plane(sphere, plane);
                if (contact.hit)
                    bodies[i].position = bodies[i].position + contact.normal * contact.depth;
            }
        }

        /**
         * @brief Separates every overlapping pair of spherical bodies, by inverse mass.
         *
         * The penetration is split between the two bodies in proportion to their
         * inverse masses (a pinned body takes none of the correction), the standard PBD
         * contact projection. O(count^2) pairwise — a broadphase is a later concern;
         * this is correct for the modest body counts the editor drives today.
         *
         * @tparam T The scalar element type.
         * @param bodies The bodies to resolve; positions updated in place.
         * @param radii  One collision radius per body.
         * @param count  Number of bodies.
         */
        template <typename T>
        inline void resolve_pair_contacts(RigidBodyT<T>* bodies, const T* radii,
                                          std::size_t count) noexcept
        {
            for (std::size_t i = 0; i < count; ++i)
                for (std::size_t j = i + 1; j < count; ++j)
                {
                    const T w = bodies[i].inv_mass + bodies[j].inv_mass;
                    if (w <= T(0))
                        continue;
                    const SphereCollider<T> a{bodies[i].position, radii[i]};
                    const SphereCollider<T> b{bodies[j].position, radii[j]};
                    const Contact<T> contact = collide_sphere_sphere(a, b);
                    if (!contact.hit)
                        continue;
                    const Vector3T<T> correction = contact.normal * (contact.depth / w);
                    bodies[i].position = bodies[i].position - correction * bodies[i].inv_mass;
                    bodies[j].position = bodies[j].position + correction * bodies[j].inv_mass;
                }
        }

        /**
         * @brief One full contact projection pass: ground first, then body pairs.
         *
         * Convenience wrapper running @ref resolve_plane_contacts then
         * @ref resolve_pair_contacts for @p iterations sweeps, so deep or stacked
         * overlaps converge (each sweep is one Gauss-Seidel iteration over the
         * contacts).
         *
         * @tparam T The scalar element type.
         * @param bodies     The bodies to resolve; positions updated in place.
         * @param radii      One collision radius per body.
         * @param count      Number of bodies.
         * @param ground     The static ground plane.
         * @param iterations Contact sweeps this pass (>= 1).
         */
        template <typename T>
        inline void resolve_contacts(RigidBodyT<T>* bodies, const T* radii, std::size_t count,
                                     const PlaneCollider<T>& ground, std::size_t iterations) noexcept
        {
            for (std::size_t iteration = 0; iteration < iterations; ++iteration)
            {
                resolve_plane_contacts(bodies, radii, count, ground);
                resolve_pair_contacts(bodies, radii, count);
            }
        }

        /**
         * @brief Pushes every penetrating body out of each of several static planes.
         *
         * A scene may have more than one static surface (a terrain plus angled ramps,
         * say), so this runs @ref resolve_plane_contacts once per plane. Each plane is a
         * half-space of infinite mass; a body is moved out of whichever ones it crosses.
         *
         * @tparam T The scalar element type.
         * @param bodies      The bodies to resolve; positions updated in place.
         * @param radii       One collision radius per body.
         * @param count       Number of bodies.
         * @param planes      The static planes.
         * @param plane_count Number of planes.
         */
        template <typename T>
        inline void resolve_static_plane_contacts(RigidBodyT<T>* bodies, const T* radii,
                                                  std::size_t count, const PlaneCollider<T>* planes,
                                                  std::size_t plane_count) noexcept
        {
            for (std::size_t p = 0; p < plane_count; ++p)
                resolve_plane_contacts(bodies, radii, count, planes[p]);
        }

        /**
         * @brief Pushes every penetrating body out of a set of static sphere obstacles.
         *
         * One-directional coupling: the obstacles are treated as immovable (infinite
         * mass), so a contacting body takes the whole correction and the obstacle is
         * untouched. This is how cloth particles collide with the rigid bodies of a
         * scene — the rigid bodies are snapshotted as spheres for the cloth's sub-step,
         * so the cloth drapes over them without (yet) pushing back on them.
         *
         * @tparam T The scalar element type.
         * @param bodies         The bodies to resolve; positions updated in place.
         * @param radii          One collision radius per body.
         * @param count          Number of bodies.
         * @param obstacles      The static sphere obstacles.
         * @param obstacle_count Number of obstacles.
         */
        template <typename T>
        inline void resolve_sphere_obstacle_contacts(RigidBodyT<T>* bodies, const T* radii,
                                                     std::size_t count,
                                                     const SphereCollider<T>* obstacles,
                                                     std::size_t obstacle_count) noexcept
        {
            for (std::size_t i = 0; i < count; ++i)
            {
                if (bodies[i].inv_mass <= T(0))
                    continue;
                for (std::size_t k = 0; k < obstacle_count; ++k)
                {
                    const SphereCollider<T> sphere{bodies[i].position, radii[i]};
                    const Contact<T> contact = collide_sphere_sphere(sphere, obstacles[k]);
                    // Normal runs from the body to the obstacle; move the body the other
                    // way (out of the obstacle) by the full depth, since the obstacle is
                    // static and takes none of the correction.
                    if (contact.hit)
                        bodies[i].position = bodies[i].position - contact.normal * contact.depth;
                }
            }
        }

        /**
         * @brief One body in the unified contact pass: a shape plus live pose pointers.
         *
         * The rigid and cloth worlds live in separate buffers, so the two-way contact
         * solver views both through a common handle: `position` and `orientation` point
         * straight into the owning buffer's body (a correction updates the real body),
         * `inv_mass` and `inv_inertia` weight how much of a contact this body absorbs
         * linearly and angularly, and the shape is either an oriented box or a sphere.
         * `filter` says which bodies this one is willing to touch — and it is what
         * cloth's lack of self-collision is expressed with, since a cloth particle is
         * a body on the cloth layer whose mask excludes that layer (§4.4). The
         * `is_cloth` boolean this replaced was §1.2 item 15: a behaviour difference
         * carried as a type tag, which every routine touching the type then had to
         * know about.
         *
         * `orientation` may be null for a body that has no meaningful rotation (a cloth
         * particle), in which case it reads as identity and never rotates. A zero
         * `inv_inertia` likewise pins every rotational degree of freedom, which is what
         * keeps cloth particles and spheres behaving exactly as they did before contacts
         * could produce torque.
         */
        template <typename T>
        struct ContactBody
        {
            Vector3T<T>* position = nullptr;
            QuaternionT<T>* orientation = nullptr;
            T inv_mass = T(0);
            Vector3T<T> inv_inertia; /**< Diagonal body-local inverse inertia; zero pins rotation. */
            bool is_box = false;
            Vector3T<T> half_extents{Vector3T<T>{T(0.5), T(0.5), T(0.5)}}; /**< Box shape. */
            T radius = T(0.5); /**< Sphere shape. */
            CollisionFilter filter{}; /**< Which bodies this one is willing to touch. */
        };

        /** @brief A contact body's orientation, or identity when it carries none. */
        template <typename T>
        inline QuaternionT<T> contact_body_orientation(const ContactBody<T>& body) noexcept
        {
            return body.orientation ? *body.orientation
                                    : QuaternionT<T>{T(0), T(0), T(0), T(1)};
        }

        /**
         * @brief Applies a body's world-space inverse inertia to a world-space vector.
         *
         * The inertia is stored as a body-local diagonal, so the vector is rotated into
         * the body frame, scaled per axis, and rotated back — the similarity transform
         * `R * I_local^-1 * R^T` without forming the matrix.
         */
        template <typename T>
        inline Vector3T<T> apply_inverse_inertia(const ContactBody<T>& body,
                                                 const Vector3T<T>& world_vector) noexcept
        {
            const QuaternionT<T> q = contact_body_orientation(body);
            const Vector3T<T> local = rotate(conjugate(q), world_vector);
            const Vector3T<T> scaled{local.x * body.inv_inertia.x, local.y * body.inv_inertia.y,
                                     local.z * body.inv_inertia.z};
            return rotate(q, scaled);
        }

        /**
         * @brief The generalized inverse mass a body presents along @p normal at @p lever.
         *
         * The linear share plus the angular share the lever arm exposes:
         * `inv_mass + (r x n) . I^-1 (r x n)`. A body with no rotational freedom returns
         * its inverse mass unchanged, which is what makes the angular path a strict
         * extension of the purely positional one.
         */
        template <typename T>
        inline T contact_generalized_mass(const ContactBody<T>& body, const Vector3T<T>& lever,
                                          const Vector3T<T>& normal) noexcept
        {
            const Vector3T<T> torque_axis = cross(lever, normal);
            return body.inv_mass + dot(torque_axis, apply_inverse_inertia(body, torque_axis));
        }

        /** @brief The world-space AABB enclosing a contact body's shape. */
        template <typename T>
        inline AABB<T> contact_body_aabb(const ContactBody<T>& body) noexcept
        {
            T ex = body.radius;
            T ey = body.radius;
            T ez = body.radius;
            if (body.is_box)
            {
                // Enclose the oriented box: each axis's world extent is the sum of the
                // absolute projections of the (rotated) half-extent axes onto that axis.
                Vector3T<T> axes[3];
                const OrientedBox<T> box{*body.position, body.half_extents,
                                         contact_body_orientation(body)};
                obb_axes(box, axes);
                ex = std::abs(axes[0].x) * body.half_extents.x +
                     std::abs(axes[1].x) * body.half_extents.y +
                     std::abs(axes[2].x) * body.half_extents.z;
                ey = std::abs(axes[0].y) * body.half_extents.x +
                     std::abs(axes[1].y) * body.half_extents.y +
                     std::abs(axes[2].y) * body.half_extents.z;
                ez = std::abs(axes[0].z) * body.half_extents.x +
                     std::abs(axes[1].z) * body.half_extents.y +
                     std::abs(axes[2].z) * body.half_extents.z;
            }
            const Vector3T<T>& c = *body.position;
            return AABB<T>{Vector3T<T>{c.x - ex, c.y - ey, c.z - ez},
                           Vector3T<T>{c.x + ex, c.y + ey, c.z + ez}};
        }

        /**
         * @brief Narrowphase between two contact bodies, normal oriented from @p a to @p b.
         *
         * Dispatches on the shape pair: box/box (SAT), box/sphere (oriented closest point,
         * flipped when the box is @p b so the normal still runs a→b), or sphere/sphere.
         */
        template <typename T>
        inline Contact<T> contact_body_narrowphase(const ContactBody<T>& a,
                                                   const ContactBody<T>& b) noexcept
        {
            const OrientedBox<T> box_a{*a.position, a.half_extents, contact_body_orientation(a)};
            const OrientedBox<T> box_b{*b.position, b.half_extents, contact_body_orientation(b)};
            if (a.is_box && b.is_box)
                return collide_obb_obb(box_a, box_b);
            if (a.is_box)
                return collide_obb_sphere(box_a, SphereCollider<T>{*b.position, b.radius});
            if (b.is_box)
            {
                // Box is the second shape: test box→sphere then flip the normal to a→b.
                Contact<T> contact =
                    collide_obb_sphere(box_b, SphereCollider<T>{*a.position, a.radius});
                contact.normal = contact.normal * T(-1);
                return contact;
            }
            return collide_sphere_sphere(SphereCollider<T>{*a.position, a.radius},
                                         SphereCollider<T>{*b.position, b.radius});
        }

        /**
         * @brief Applies one body's share of a contact impulse to its pose.
         *
         * `sign` is +1 for the body the normal points toward and -1 for the other, so the
         * pair pushes apart. The linear part moves the centre of mass; the angular part
         * turns the body about the lever arm from its centre to the contact point, which
         * is what lets a box struck off-centre topple instead of sliding flat.
         */
        template <typename T>
        inline void apply_contact_impulse(ContactBody<T>& body, const Vector3T<T>& impulse,
                                          const Vector3T<T>& lever, T sign) noexcept
        {
            *body.position = *body.position + impulse * (sign * body.inv_mass);
            if (!body.orientation)
                return;
            const Vector3T<T> rotation =
                apply_inverse_inertia(body, cross(lever, impulse * sign));
            if (dot(rotation, rotation) > T(0))
                *body.orientation = apply_angular_correction(*body.orientation, rotation);
        }

        /**
         * @brief Resolves one contacting pair, splitting the correction by generalized mass.
         *
         * The two-way PBD projection, extended to rotation: each body's share is weighted
         * by `inv_mass + (r x n) . I^-1 (r x n)`, so a light body yields to a heavy one,
         * two equal bodies split the push, and a contact away from the centre of mass
         * spends part of its correction as a turn rather than a slide. Bodies with no
         * rotational freedom (cloth particles, spheres) reduce exactly to the older
         * inverse-mass split. Skips cloth-cloth pairs and pairs with no movable mass.
         *
         * The manifold is a single point, so a box resting on a face is held by one
         * corner at a time and rocks slightly rather than settling rigidly flat.
         */
        template <typename T>
        inline void resolve_contact_bodies(ContactBody<T>& a, ContactBody<T>& b) noexcept
        {
            if (!filters_collide(a.filter, b.filter))
                return;
            if (a.inv_mass + b.inv_mass <= T(0))
                return;
            const Contact<T> contact = contact_body_narrowphase(a, b);
            if (!contact.hit)
                return;

            const Vector3T<T> lever_a = contact.point - *a.position;
            const Vector3T<T> lever_b = contact.point - *b.position;
            const T w = contact_generalized_mass(a, lever_a, contact.normal) +
                        contact_generalized_mass(b, lever_b, contact.normal);
            if (w <= T(0))
                return;

            const Vector3T<T> impulse = contact.normal * (contact.depth / w);
            apply_contact_impulse(a, impulse, lever_a, T(-1));
            apply_contact_impulse(b, impulse, lever_b, T(1));
        }

        /**
         * @brief Pushes one contact body out of a static half-space plane.
         *
         * The plane is immovable, so the body absorbs the whole correction — but it
         * absorbs it at the contact point, so a box that lands on a corner rotates
         * toward lying flat instead of being lifted rigidly.
         *
         * This is the *same* projection @ref resolve_contact_bodies applies, with the
         * plane contributing no generalized mass, and the sameness is the point.
         * Scaling the impulse by an extra `inv_mass / w` factor here holds only for a
         * body of unit inverse mass — a heavier body clears a fraction of its
         * penetration per sweep and a lighter one overshoots it — and the angular
         * share then fails to conserve the correction, so a plane contact and a
         * body-pair contact on the same geometry disagree. One projection, used
         * twice, cannot disagree with itself.
         */
        template <typename T>
        inline void resolve_contact_body_plane(ContactBody<T>& body,
                                               const PlaneCollider<T>& plane) noexcept
        {
            if (body.inv_mass <= T(0))
                return;
            const Contact<T> contact =
                body.is_box
                    ? collide_obb_plane(OrientedBox<T>{*body.position, body.half_extents,
                                                       contact_body_orientation(body)},
                                        plane)
                    : collide_sphere_plane(SphereCollider<T>{*body.position, body.radius}, plane);
            if (!contact.hit)
                return;

            const Vector3T<T> lever = contact.point - *body.position;
            const T w = contact_generalized_mass(body, lever, contact.normal);
            if (w <= T(0))
                return;
            const Vector3T<T> impulse = contact.normal * (contact.depth / w);
            apply_contact_impulse(body, impulse, lever, T(1));
        }
    } // namespace Physics
} // namespace SushiEngine
