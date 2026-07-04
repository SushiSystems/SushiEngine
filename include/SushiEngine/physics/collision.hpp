/**************************************************************************/
/* collision.hpp                                                         */
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
 * @file collision.hpp
 * @brief The collision narrowphase: pure contact-generation between primitive shapes.
 *
 * Each function takes two shapes and returns a `Contact` describing how they overlap
 * — the contact normal, the penetration depth, and a representative point — or a
 * miss. They are pure and free of any runtime, ECS, or solver dependency, so the
 * geometry is unit-tested directly; `physics/contact_solver.hpp` consumes the
 * contacts to push bodies apart. Shapes and contacts are element-parametric (`T` is
 * `float` or `double`), matching the templated rigid-body physics.
 *
 * Convention: a `Contact`'s `normal` points from the first shape toward the second
 * and is unit length; `depth` is the (positive) overlap along that normal. Resolving
 * moves the first shape by `-normal` and the second by `+normal`, weighted by inverse
 * mass, until `depth` reaches zero.
 */

#include <cmath>

#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /** @brief A solid sphere: a centre and a radius. */
        template <typename T>
        struct SphereCollider
        {
            Vector3T<T> center;
            T radius = 1;
        };

        /**
         * @brief A half-space plane: points with `dot(normal, x) >= offset` are outside.
         *
         * `normal` is unit length and points away from the solid side; `offset` is the
         * plane's signed distance from the origin along `normal`. A ground plane at
         * height `h` is `{normal = (0, 1, 0), offset = h}`.
         */
        template <typename T>
        struct PlaneCollider
        {
            Vector3T<T> normal{Vector3T<T>{T(0), T(1), T(0)}};
            T offset = 0;
        };

        /** @brief An axis-aligned box: a centre and per-axis half-extents. */
        template <typename T>
        struct BoxCollider
        {
            Vector3T<T> center;
            Vector3T<T> half_extents{Vector3T<T>{T(0.5), T(0.5), T(0.5)}};
        };

        /**
         * @brief The result of a narrowphase test: whether, where, and how deep two shapes overlap.
         *
         * `hit` is false for a miss, in which case the other fields are unspecified.
         */
        template <typename T>
        struct Contact
        {
            bool hit = false;
            Vector3T<T> normal;  /**< Unit contact normal, from the first shape to the second. */
            T depth = 0;         /**< Positive penetration depth along @ref normal. */
            Vector3T<T> point;   /**< A representative contact point in world space. */
        };

        /**
         * @brief Contact between a sphere and a half-space plane.
         * @param sphere The sphere.
         * @param plane  The plane; its `normal` must be unit length.
         * @return The contact if the sphere crosses the plane, else a miss. The normal
         * is the plane normal (pushing the sphere out along it).
         */
        template <typename T>
        inline Contact<T> collide_sphere_plane(const SphereCollider<T>& sphere,
                                               const PlaneCollider<T>& plane) noexcept
        {
            const T signed_distance = dot(plane.normal, sphere.center) - plane.offset;
            const T depth = sphere.radius - signed_distance;
            if (depth <= T(0))
                return Contact<T>{};
            Contact<T> contact;
            contact.hit = true;
            contact.normal = plane.normal;
            contact.depth = depth;
            contact.point = sphere.center - plane.normal * signed_distance;
            return contact;
        }

        /**
         * @brief Contact between two spheres.
         * @param a The first sphere.
         * @param b The second sphere.
         * @return The contact if they overlap, else a miss. The normal points from
         * @p a's centre to @p b's.
         */
        template <typename T>
        inline Contact<T> collide_sphere_sphere(const SphereCollider<T>& a,
                                                const SphereCollider<T>& b) noexcept
        {
            const Vector3T<T> delta = b.center - a.center;
            const T distance = length(delta);
            const T depth = (a.radius + b.radius) - distance;
            if (depth <= T(0))
                return Contact<T>{};
            Contact<T> contact;
            contact.hit = true;
            contact.normal = distance > T(1e-8) ? delta * (T(1) / distance)
                                                : Vector3T<T>{T(0), T(1), T(0)};
            contact.depth = depth;
            contact.point = a.center + contact.normal * a.radius;
            return contact;
        }

        /**
         * @brief Contact between a sphere and an axis-aligned box (sphere as the second shape).
         *
         * Uses the box's closest surface point to the sphere centre; the sphere-inside-box
         * degenerate case pushes out along the +Y axis rather than picking the nearest
         * face, which is enough for the resting-on-a-surface cases this supports today.
         *
         * @param box    The box.
         * @param sphere The sphere.
         * @return The contact if they overlap, else a miss. The normal points from the
         * box toward the sphere.
         */
        template <typename T>
        inline Contact<T> collide_box_sphere(const BoxCollider<T>& box,
                                             const SphereCollider<T>& sphere) noexcept
        {
            const auto clamp = [](T value, T low, T high) noexcept
            { return value < low ? low : (value > high ? high : value); };

            const Vector3T<T> min{box.center.x - box.half_extents.x,
                                  box.center.y - box.half_extents.y,
                                  box.center.z - box.half_extents.z};
            const Vector3T<T> max{box.center.x + box.half_extents.x,
                                  box.center.y + box.half_extents.y,
                                  box.center.z + box.half_extents.z};
            const Vector3T<T> closest{clamp(sphere.center.x, min.x, max.x),
                                      clamp(sphere.center.y, min.y, max.y),
                                      clamp(sphere.center.z, min.z, max.z)};
            const Vector3T<T> delta = sphere.center - closest;
            const T distance = length(delta);
            const T depth = sphere.radius - distance;
            if (depth <= T(0))
                return Contact<T>{};
            Contact<T> contact;
            contact.hit = true;
            contact.normal = distance > T(1e-8) ? delta * (T(1) / distance)
                                                : Vector3T<T>{T(0), T(1), T(0)};
            contact.depth = depth;
            contact.point = closest;
            return contact;
        }

        /**
         * @brief Contact between an axis-aligned box and a half-space plane.
         * @param box   The box.
         * @param plane The plane; its `normal` must be unit length.
         * @return The contact if the box crosses the plane, else a miss. The normal is
         * the plane normal (pushing the box out along it).
         */
        template <typename T>
        inline Contact<T> collide_box_plane(const BoxCollider<T>& box,
                                            const PlaneCollider<T>& plane) noexcept
        {
            const T projected_radius = std::abs(plane.normal.x) * box.half_extents.x +
                                       std::abs(plane.normal.y) * box.half_extents.y +
                                       std::abs(plane.normal.z) * box.half_extents.z;
            const T signed_distance = dot(plane.normal, box.center) - plane.offset;
            const T depth = projected_radius - signed_distance;
            if (depth <= T(0))
                return Contact<T>{};
            Contact<T> contact;
            contact.hit = true;
            contact.normal = plane.normal;
            contact.depth = depth;
            contact.point = box.center - plane.normal * signed_distance;
            return contact;
        }

        /**
         * @brief An oriented box (OBB): a centre, per-axis half-extents, and a rotation.
         *
         * Unlike `BoxCollider` (axis-aligned), this carries the body's orientation, so a
         * tumbling rigid body collides as the box it looks like rather than its bounding
         * sphere. Its three local axes are the rotation applied to the world basis.
         */
        template <typename T>
        struct OrientedBox
        {
            Vector3T<T> center;
            Vector3T<T> half_extents{Vector3T<T>{T(0.5), T(0.5), T(0.5)}};
            QuaternionT<T> orientation{QuaternionT<T>{T(0), T(0), T(0), T(1)}};
        };

        /** @brief The three world-space axes (columns of the rotation) of an oriented box. */
        template <typename T>
        inline void obb_axes(const OrientedBox<T>& box, Vector3T<T> axes[3]) noexcept
        {
            axes[0] = rotate(box.orientation, Vector3T<T>{T(1), T(0), T(0)});
            axes[1] = rotate(box.orientation, Vector3T<T>{T(0), T(1), T(0)});
            axes[2] = rotate(box.orientation, Vector3T<T>{T(0), T(0), T(1)});
        }

        /**
         * @brief Contact between an oriented box and a half-space plane.
         *
         * The box's extent along the plane normal is the sum of each axis's projection,
         * so a rotated box resting on the ground touches by its true lowest corner/face.
         *
         * @return The contact if the box crosses the plane, else a miss (normal = plane normal).
         */
        template <typename T>
        inline Contact<T> collide_obb_plane(const OrientedBox<T>& box,
                                            const PlaneCollider<T>& plane) noexcept
        {
            Vector3T<T> axes[3];
            obb_axes(box, axes);
            const T radius = std::abs(dot(axes[0], plane.normal)) * box.half_extents.x +
                             std::abs(dot(axes[1], plane.normal)) * box.half_extents.y +
                             std::abs(dot(axes[2], plane.normal)) * box.half_extents.z;
            const T signed_distance = dot(plane.normal, box.center) - plane.offset;
            const T depth = radius - signed_distance;
            if (depth <= T(0))
                return Contact<T>{};
            Contact<T> contact;
            contact.hit = true;
            contact.normal = plane.normal;
            contact.depth = depth;
            contact.point = box.center - plane.normal * signed_distance;
            return contact;
        }

        /**
         * @brief Contact between an oriented box and a sphere (sphere as the second shape).
         *
         * Works in the box's local frame: the sphere centre is rotated into box space,
         * clamped to the box, and the closest point gives the normal — the oriented
         * counterpart of @ref collide_box_sphere.
         *
         * @return The contact if they overlap, else a miss (normal points box → sphere).
         */
        template <typename T>
        inline Contact<T> collide_obb_sphere(const OrientedBox<T>& box,
                                             const SphereCollider<T>& sphere) noexcept
        {
            const auto clamp = [](T value, T low, T high) noexcept
            { return value < low ? low : (value > high ? high : value); };

            const Vector3T<T> local = rotate(conjugate(box.orientation), sphere.center - box.center);
            const Vector3T<T> closest{clamp(local.x, -box.half_extents.x, box.half_extents.x),
                                      clamp(local.y, -box.half_extents.y, box.half_extents.y),
                                      clamp(local.z, -box.half_extents.z, box.half_extents.z)};
            const Vector3T<T> delta = local - closest;
            const T distance = length(delta);
            const T depth = sphere.radius - distance;
            if (depth <= T(0))
                return Contact<T>{};
            const Vector3T<T> normal_local =
                distance > T(1e-8) ? delta * (T(1) / distance) : Vector3T<T>{T(0), T(1), T(0)};
            Contact<T> contact;
            contact.hit = true;
            contact.normal = rotate(box.orientation, normal_local);
            contact.depth = depth;
            contact.point = box.center + rotate(box.orientation, closest);
            return contact;
        }

        /**
         * @brief Contact between two oriented boxes via the separating-axis theorem.
         *
         * Tests the 15 candidate separating axes (each box's 3 face normals and the 9
         * edge-edge cross products); if none separates, the axis of least overlap is the
         * contact normal and that overlap is the penetration depth. Near-degenerate cross
         * axes (parallel edges) are skipped, as the SAT requires. The normal is oriented
         * to point from @p a toward @p b.
         *
         * @return The contact if the boxes overlap, else a miss.
         */
        template <typename T>
        inline Contact<T> collide_obb_obb(const OrientedBox<T>& a, const OrientedBox<T>& b) noexcept
        {
            Vector3T<T> axis_a[3];
            Vector3T<T> axis_b[3];
            obb_axes(a, axis_a);
            obb_axes(b, axis_b);
            const T ea[3] = {a.half_extents.x, a.half_extents.y, a.half_extents.z};
            const T eb[3] = {b.half_extents.x, b.half_extents.y, b.half_extents.z};
            const Vector3T<T> center_delta = b.center - a.center;

            bool have_axis = false;
            T best_depth = T(0);
            Vector3T<T> best_axis{T(0), T(1), T(0)};

            const auto test_axis = [&](const Vector3T<T>& raw) -> bool
            {
                const T length_squared = dot(raw, raw);
                if (length_squared < T(1e-12))
                    return true; // degenerate (e.g. parallel edges): not a separating axis
                const Vector3T<T> axis = raw * (T(1) / std::sqrt(length_squared));
                const T ra = std::abs(dot(axis_a[0], axis)) * ea[0] +
                             std::abs(dot(axis_a[1], axis)) * ea[1] +
                             std::abs(dot(axis_a[2], axis)) * ea[2];
                const T rb = std::abs(dot(axis_b[0], axis)) * eb[0] +
                             std::abs(dot(axis_b[1], axis)) * eb[1] +
                             std::abs(dot(axis_b[2], axis)) * eb[2];
                const T distance = std::abs(dot(center_delta, axis));
                const T overlap = ra + rb - distance;
                if (overlap < T(0))
                    return false; // found a separating axis
                if (!have_axis || overlap < best_depth)
                {
                    have_axis = true;
                    best_depth = overlap;
                    best_axis = axis;
                }
                return true;
            };

            for (int i = 0; i < 3; ++i)
                if (!test_axis(axis_a[i]))
                    return Contact<T>{};
            for (int i = 0; i < 3; ++i)
                if (!test_axis(axis_b[i]))
                    return Contact<T>{};
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    if (!test_axis(cross(axis_a[i], axis_b[j])))
                        return Contact<T>{};

            Contact<T> contact;
            contact.hit = true;
            contact.depth = best_depth;
            // Orient the least-overlap axis from a toward b.
            contact.normal =
                dot(best_axis, center_delta) < T(0) ? best_axis * T(-1) : best_axis;
            contact.point = a.center + center_delta * T(0.5);
            return contact;
        }
    } // namespace Physics
} // namespace SushiEngine
