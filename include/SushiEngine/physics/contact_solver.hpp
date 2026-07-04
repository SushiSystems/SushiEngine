/**************************************************************************/
/* contact_solver.hpp                                                    */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/* Licensed under the Apache License, Version 2.0 (the "License");        */
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
 * @brief Positional (PBD) contact resolution for spherical rigid bodies.
 *
 * Non-penetration is an inequality constraint — a contact only ever pushes bodies
 * apart, never pulls them together — so it is handled as a projection pass over the
 * predicted positions rather than through the compile-once `XpbdSolver` (whose
 * constraint set is fixed): contacts appear and vanish as bodies move, so they are
 * regenerated from the narrowphase each pass. Run it between `predict` and
 * `update_velocity` in a sub-step: the position change it makes is exactly what
 * `update_velocity` then reads back as the post-contact velocity, so a body landing
 * on a surface loses its downward velocity without any explicit restitution term
 * (inelastic contact). Bodies are treated as spheres (each with a radius); this is
 * the smallest contact model that makes the editor's colliders actually collide.
 */

#include <cstddef>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/broadphase.hpp>
#include <SushiEngine/physics/collision.hpp>
#include <SushiEngine/physics/rigid_body.hpp>

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
         * @brief One body in the unified contact pass: a shape plus a live position pointer.
         *
         * The rigid and cloth worlds live in separate buffers, so the two-way contact
         * solver views both through a common handle: `position` points straight into the
         * owning buffer's body (a correction updates the real body), `inv_mass` weights how
         * much of a contact this body absorbs, and the shape is either an oriented box or a
         * sphere. `is_cloth` lets the pass skip cloth-cloth pairs (no self-collision yet).
         */
        template <typename T>
        struct ContactBody
        {
            Vector3T<T>* position = nullptr;
            T inv_mass = T(0);
            bool is_box = false;
            Vector3T<T> half_extents{Vector3T<T>{T(0.5), T(0.5), T(0.5)}}; /**< Box shape. */
            QuaternionT<T> orientation{QuaternionT<T>{T(0), T(0), T(0), T(1)}};
            T radius = T(0.5); /**< Sphere shape. */
            bool is_cloth = false;
        };

        /** @brief The world-space AABB enclosing a contact body's shape. */
        template <typename T>
        inline Aabb<T> contact_body_aabb(const ContactBody<T>& body) noexcept
        {
            T ex = body.radius;
            T ey = body.radius;
            T ez = body.radius;
            if (body.is_box)
            {
                // Enclose the oriented box: each axis's world extent is the sum of the
                // absolute projections of the (rotated) half-extent axes onto that axis.
                Vector3T<T> axes[3];
                const OrientedBox<T> box{*body.position, body.half_extents, body.orientation};
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
            return Aabb<T>{Vector3T<T>{c.x - ex, c.y - ey, c.z - ez},
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
            if (a.is_box && b.is_box)
                return collide_obb_obb(OrientedBox<T>{*a.position, a.half_extents, a.orientation},
                                       OrientedBox<T>{*b.position, b.half_extents, b.orientation});
            if (a.is_box)
                return collide_obb_sphere(OrientedBox<T>{*a.position, a.half_extents, a.orientation},
                                          SphereCollider<T>{*b.position, b.radius});
            if (b.is_box)
            {
                // Box is the second shape: test box→sphere then flip the normal to a→b.
                Contact<T> contact =
                    collide_obb_sphere(OrientedBox<T>{*b.position, b.half_extents, b.orientation},
                                       SphereCollider<T>{*a.position, a.radius});
                contact.normal = contact.normal * T(-1);
                return contact;
            }
            return collide_sphere_sphere(SphereCollider<T>{*a.position, a.radius},
                                         SphereCollider<T>{*b.position, b.radius});
        }

        /**
         * @brief Resolves one contacting pair, splitting the correction by inverse mass.
         *
         * The standard two-way PBD projection: each body moves along the contact normal in
         * proportion to its share of the total inverse mass, so a light body yields to a
         * heavy one and two equal bodies split the push. Skips cloth-cloth pairs and pairs
         * with no movable mass.
         */
        template <typename T>
        inline void resolve_contact_bodies(ContactBody<T>& a, ContactBody<T>& b) noexcept
        {
            if (a.is_cloth && b.is_cloth)
                return;
            const T w = a.inv_mass + b.inv_mass;
            if (w <= T(0))
                return;
            const Contact<T> contact = contact_body_narrowphase(a, b);
            if (!contact.hit)
                return;
            const Vector3T<T> correction = contact.normal * (contact.depth / w);
            *a.position = *a.position - correction * a.inv_mass;
            *b.position = *b.position + correction * b.inv_mass;
        }

        /** @brief Pushes one contact body out of a static half-space plane (full correction). */
        template <typename T>
        inline void resolve_contact_body_plane(ContactBody<T>& body,
                                               const PlaneCollider<T>& plane) noexcept
        {
            if (body.inv_mass <= T(0))
                return;
            const Contact<T> contact =
                body.is_box
                    ? collide_obb_plane(OrientedBox<T>{*body.position, body.half_extents,
                                                       body.orientation},
                                        plane)
                    : collide_sphere_plane(SphereCollider<T>{*body.position, body.radius}, plane);
            if (contact.hit)
                *body.position = *body.position + contact.normal * contact.depth;
        }
    } // namespace Physics
} // namespace SushiEngine
