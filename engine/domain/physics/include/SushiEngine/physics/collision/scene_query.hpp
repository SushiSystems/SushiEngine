/**************************************************************************/
/* scene_query.hpp                                                        */
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
 * @file scene_query.hpp
 * @brief Asking the collision world a question that is not "what is touching".
 *
 * §7.7. A raycast is what a weapon, a camera, a footstep probe, a placement tool
 * and a line of sight all are, and every one of them wants the same three things:
 * the geometry answered exactly, the candidates found by a tree descent rather
 * than a scan, and a filter applied before either.
 *
 * **Ray against a shape is an overload set, like everything else here.** Adding a
 * shape adds a `ray_cast` overload and a line in the registration; no existing
 * function is edited (§4.2). Four of the overloads are closed forms because they
 * are worth having exactly — a sphere, a plane, an oriented box, a triangle. The
 * rest are one generic routine: **conservative advancement**, which walks the ray
 * forward by the distance the shape is currently away from it and repeats. It
 * needs nothing from a shape but its support function, so it covers the capsule,
 * the cooked hull, and every convex shape not yet written; and it is the same
 * machinery §7.5's tier two uses for a fast body, arrived at from the other side.
 *
 * **Determinism.** Every query here reports its hits in an order derived from the
 * hits themselves — by distance, ties broken by proxy — never in traversal order.
 * A tree's traversal order is a function of its shape, its shape is a function of
 * insertion history, and a gameplay system that reads `hits[0]` must not be
 * reading a history.
 */

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/collision/broadphase.hpp>
#include <SushiEngine/physics/collision/narrowphase_dispatch.hpp>
#include <SushiEngine/physics/geometry/gjk.hpp>
#include <SushiEngine/physics/geometry/shapes.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /** @brief What a ray query hit, and where. */
        template <typename T>
        struct RayHit
        {
            bool hit = false;
            T distance = 0;                    /**< Along the ray's direction, from its origin. */
            Vector3T<T> point{};               /**< The world point of impact. */
            Vector3T<T> normal{};              /**< The surface normal there, pointing outward. */
            ProxyId proxy = null_proxy;        /**< Which proxy was hit. */
            std::uint32_t payload = 0;         /**< That proxy's payload — usually the body. */
        };

        /**
         * @brief Which proxies a query is willing to see.
         *
         * The mask is the same layer word bodies carry, so "the camera ignores the
         * player" is authored once and honoured by contacts and queries alike. The
         * predicate is the escape hatch for what a mask cannot express — "not the
         * body that fired this shot" — and is deliberately not a mask, because a
         * per-query exclusion encoded as a layer is a layer that leaks into the
         * collision matrix and never leaves it.
         */
        template <typename T>
        struct QueryFilter
        {
            /** @brief A proxy is considered when its layer is in this mask. */
            std::uint32_t layer_mask = 0xFFFFFFFFu;

            /** @brief Proxies carrying any of these flags are skipped. */
            std::uint32_t reject_flags = 0;

            /** @brief Optional last word; return false to skip the proxy. */
            std::function<bool(ProxyId, std::uint32_t)> predicate;

            /** @brief Whether @p proxy passes. */
            bool accepts(ProxyId id, const BroadphaseProxy<T>& record) const
            {
                if ((record.filter.layer & layer_mask) == 0u)
                    return false;
                if (has_any_flag(record.flags, reject_flags))
                    return false;
                return !predicate || predicate(id, record.payload);
            }
        };

        // Ray against one shape

        /**
         * @brief Ray against a sphere: the quadratic, solved for the near root.
         *
         * A ray starting inside reports distance zero and the normal it would
         * leave through, because "I am inside geometry" is an answer a character
         * controller needs and a miss is not one.
         */
        template <typename T>
        inline bool ray_cast(const SphereCollider<T>& sphere, const Vector3T<T>& origin,
                             const Vector3T<T>& direction, T max_distance, RayHit<T>& out) noexcept
        {
            const Vector3T<T> to_origin = origin - sphere.center;
            const T b = dot(to_origin, direction);
            const T c = dot(to_origin, to_origin) - sphere.radius * sphere.radius;
            if (c > T(0) && b > T(0))
                return false; // outside and pointing away
            const T discriminant = b * b - c;
            if (discriminant < T(0))
                return false;
            T distance = -b - std::sqrt(discriminant);
            if (distance < T(0))
                distance = T(0);
            if (distance > max_distance)
                return false;
            out.hit = true;
            out.distance = distance;
            out.point = origin + direction * distance;
            out.normal = safe_normalize(out.point - sphere.center);
            return true;
        }

        /**
         * @brief Ray against a half-space, from the outside only.
         *
         * A ray already inside the solid half is not reported: a plane is a
         * boundary, and a query that starts under the ground wants to know it hit
         * nothing on the way out, not that it hit the ground from beneath.
         */
        template <typename T>
        inline bool ray_cast(const PlaneCollider<T>& plane, const Vector3T<T>& origin,
                             const Vector3T<T>& direction, T max_distance, RayHit<T>& out) noexcept
        {
            const T along = dot(plane.normal, direction);
            const T height = dot(plane.normal, origin) - plane.offset;
            if (height < T(0))
                return false;
            if (along >= T(0))
                return false;
            const T distance = -height / along;
            if (distance > max_distance)
                return false;
            out.hit = true;
            out.distance = distance;
            out.point = origin + direction * distance;
            out.normal = plane.normal;
            return true;
        }

        /** @brief Ray against an oriented box: the slab test, in the box's own frame. */
        template <typename T>
        inline bool ray_cast(const OrientedBox<T>& box, const Vector3T<T>& origin,
                             const Vector3T<T>& direction, T max_distance, RayHit<T>& out) noexcept
        {
            const QuaternionT<T> inverse_rotation = conjugate(box.orientation);
            const Vector3T<T> local_origin = rotate(inverse_rotation, origin - box.center);
            const Vector3T<T> local_direction = rotate(inverse_rotation, direction);

            const T origin_components[3] = {local_origin.x, local_origin.y, local_origin.z};
            const T direction_components[3] = {local_direction.x, local_direction.y,
                                               local_direction.z};
            const T extents[3] = {box.half_extents.x, box.half_extents.y, box.half_extents.z};

            T near_distance = T(0);
            T far_distance = max_distance;
            int entry_axis = 0;
            T entry_sign = T(1);
            const Vector3T<T> inverse = ray_inverse_direction(local_direction);
            const T inverse_components[3] = {inverse.x, inverse.y, inverse.z};

            for (int axis = 0; axis < 3; ++axis)
            {
                T entry = (-extents[axis] - origin_components[axis]) * inverse_components[axis];
                T exit = (extents[axis] - origin_components[axis]) * inverse_components[axis];
                T sign = T(-1);
                if (entry > exit)
                {
                    const T swap = entry;
                    entry = exit;
                    exit = swap;
                    sign = T(1);
                }
                if (entry > near_distance)
                {
                    near_distance = entry;
                    entry_axis = axis;
                    entry_sign = sign;
                }
                if (exit < far_distance)
                    far_distance = exit;
                if (near_distance > far_distance)
                    return false;
                if (direction_components[axis] == T(0) &&
                    (origin_components[axis] < -extents[axis] ||
                     origin_components[axis] > extents[axis]))
                    return false;
            }

            out.hit = true;
            out.distance = near_distance;
            out.point = origin + direction * near_distance;
            Vector3T<T> local_normal{T(0), T(0), T(0)};
            if (entry_axis == 0)
                local_normal.x = entry_sign;
            else if (entry_axis == 1)
                local_normal.y = entry_sign;
            else
                local_normal.z = entry_sign;
            out.normal = rotate(box.orientation, local_normal);
            return true;
        }

        /** @brief Ray against a triangle, by Möller–Trumbore. */
        template <typename T>
        inline bool ray_cast(const TriangleCollider<T>& triangle, const Vector3T<T>& origin,
                             const Vector3T<T>& direction, T max_distance, RayHit<T>& out) noexcept
        {
            const Vector3T<T> edge_1 = triangle.b - triangle.a;
            const Vector3T<T> edge_2 = triangle.c - triangle.a;
            const Vector3T<T> perpendicular = cross(direction, edge_2);
            const T determinant = dot(edge_1, perpendicular);
            if (std::abs(determinant) <= T(1e-18))
                return false; // the ray runs in the triangle's plane
            const T inverse_determinant = T(1) / determinant;
            const Vector3T<T> to_vertex = origin - triangle.a;
            const T u = dot(to_vertex, perpendicular) * inverse_determinant;
            if (u < T(0) || u > T(1))
                return false;
            const Vector3T<T> across = cross(to_vertex, edge_1);
            const T v = dot(direction, across) * inverse_determinant;
            if (v < T(0) || u + v > T(1))
                return false;
            const T distance = dot(edge_2, across) * inverse_determinant;
            if (distance < T(0) || distance > max_distance)
                return false;

            out.hit = true;
            out.distance = distance;
            out.point = origin + direction * distance;
            const Vector3T<T> face = triangle_normal(triangle);
            out.normal = dot(face, direction) > T(0) ? face * T(-1) : face;
            return true;
        }

        /**
         * @brief Ray against any convex shape, by conservative advancement.
         *
         * Walk a point along the ray. At each step ask how far the shape is — the
         * one question a support function answers — and advance by the *most* the
         * point could travel without passing through: the distance divided by how
         * much of the direction points at the shape. That step can never overshoot
         * the surface, which is what makes the iteration safe to stop at any point
         * and call a miss.
         *
         * The cost is a handful of GJK calls, not a closed form, so the shapes with
         * one of those keep their own overload above. This is the general answer,
         * and it is what makes a cooked hull castable the day it is cooked.
         *
         * @param shape        Any shape with a `support()` overload.
         * @param origin       The ray's origin.
         * @param direction    Unit direction.
         * @param max_distance How far to look.
         * @param out          Receives the hit.
         * @return True on a hit.
         */
        template <typename T, typename Shape>
        inline bool ray_cast_convex(const Shape& shape, const Vector3T<T>& origin,
                                    const Vector3T<T>& direction, T max_distance,
                                    RayHit<T>& out) noexcept
        {
            constexpr T tolerance = T(1e-6);
            T travelled = T(0);
            Vector3T<T> point = origin;
            Vector3T<T> normal{T(0), T(1), T(0)};

            for (int iteration = 0; iteration < 48; ++iteration)
            {
                const ConvexContact<T> contact =
                    collide_convex<T>(SphereCollider<T>{point, T(0)}, shape);
                if (!contact.valid)
                    return false;
                normal = contact.normal;
                if (contact.separation <= tolerance)
                {
                    out.hit = true;
                    out.distance = travelled;
                    out.point = point;
                    // `normal` runs from the ray's point toward the shape, so the
                    // outward surface normal is its reverse.
                    out.normal = normal * T(-1);
                    return true;
                }
                const T approach = dot(direction, normal);
                if (approach <= T(1e-12))
                    return false; // never getting any closer
                travelled += contact.separation / approach;
                if (travelled > max_distance)
                    return false;
                point = origin + direction * travelled;
            }
            return false;
        }

        /** @brief Ray against a capsule, through the general convex routine. */
        template <typename T>
        inline bool ray_cast(const CapsuleCollider<T>& capsule, const Vector3T<T>& origin,
                             const Vector3T<T>& direction, T max_distance, RayHit<T>& out) noexcept
        {
            return ray_cast_convex<T>(capsule, origin, direction, max_distance, out);
        }

        /** @brief Ray against a cooked convex hull, through the same routine. */
        template <typename T>
        inline bool ray_cast(const ConvexHullView<T>& hull, const Vector3T<T>& origin,
                             const Vector3T<T>& direction, T max_distance, RayHit<T>& out) noexcept
        {
            return ray_cast_convex<T>(hull, origin, direction, max_distance, out);
        }

        // The registration: shape type to ray routine

        /** @brief The signature every ray-cast entry has. */
        template <typename T>
        using RayFunction = bool (*)(const CollisionShape<T>&, const Vector3T<T>&,
                                     const Vector3T<T>&, T, RayHit<T>&);

        /** @brief Table entry: recover the concrete shape, run its overload. */
        template <typename T, typename Shape>
        inline bool ray_entry(const CollisionShape<T>& shape, const Vector3T<T>& origin,
                              const Vector3T<T>& direction, T max_distance,
                              RayHit<T>& out) noexcept
        {
            return ray_cast(ShapeTraits<T, Shape>::from(shape), origin, direction, max_distance,
                            out);
        }

        /** @brief Table entry for a half-space, which has no `ShapeTraits`. */
        template <typename T>
        inline bool plane_ray_entry(const CollisionShape<T>& shape, const Vector3T<T>& origin,
                                    const Vector3T<T>& direction, T max_distance,
                                    RayHit<T>& out) noexcept
        {
            return ray_cast(PlaneCollider<T>{shape.plane_normal, shape.plane_offset}, origin,
                            direction, max_distance, out);
        }

        /** @brief One routine per shape kind, folded out of the same shape list. */
        template <typename T>
        struct RayCastTable
        {
            static constexpr std::size_t kind_count = static_cast<std::size_t>(ShapeType::count);
            RayFunction<T> entries[kind_count] = {};

            RayFunction<T> get(ShapeType type) const noexcept
            {
                return entries[static_cast<std::size_t>(type)];
            }
        };

        /** @brief Fills the table by folding the convex shape list. */
        template <typename T, typename... Shapes>
        inline void register_ray_shapes(RayCastTable<T>& table, TypeList<Shapes...>) noexcept
        {
            ((table.entries[static_cast<std::size_t>(ShapeTraits<T, Shapes>::type)] =
                  &ray_entry<T, Shapes>),
             ...);
            table.entries[static_cast<std::size_t>(ShapeType::plane)] = &plane_ray_entry<T>;
        }

        /** @brief The one ray table, built on first use. */
        template <typename T>
        inline const RayCastTable<T>& ray_cast_table() noexcept
        {
            static const RayCastTable<T> table = []()
            {
                RayCastTable<T> built;
                register_ray_shapes<T>(built, ConvexShapes<T>{});
                return built;
            }();
            return table;
        }

        /**
         * @brief Casts a ray at a type-erased shape.
         *
         * @return True on a hit; false for a miss *and* for a shape kind nobody has
         *         registered a routine for, which is the same rule the narrowphase
         *         table follows and for the same reason.
         */
        template <typename T>
        inline bool ray_cast_shape(const CollisionShape<T>& shape, const Vector3T<T>& origin,
                                   const Vector3T<T>& direction, T max_distance,
                                   RayHit<T>& out) noexcept
        {
            const RayFunction<T> entry = ray_cast_table<T>().get(shape.type);
            if (entry == nullptr)
                return false;
            return entry(shape, origin, direction, max_distance, out);
        }

        /** @brief The same shape, moved by @p offset. A half-space does not move. */
        template <typename T>
        inline CollisionShape<T> translate_shape(const CollisionShape<T>& shape,
                                                 const Vector3T<T>& offset) noexcept
        {
            CollisionShape<T> moved = shape;
            if (moved.type != ShapeType::plane)
                moved.center = moved.center + offset;
            return moved;
        }

        // Closest points between two type-erased shapes

        /** @brief The signature every closest-point entry has. */
        template <typename T>
        using ClosestFunction = ConvexContact<T> (*)(const CollisionShape<T>&,
                                                     const CollisionShape<T>&);

        /** @brief Table entry for a convex pair: straight to GJK. */
        template <typename T, typename ShapeA, typename ShapeB>
        inline ConvexContact<T> convex_closest_entry(const CollisionShape<T>& a,
                                                     const CollisionShape<T>& b) noexcept
        {
            return collide_convex<T>(ShapeTraits<T, ShapeA>::from(a),
                                     ShapeTraits<T, ShapeB>::from(b));
        }

        /**
         * @brief Table entry for a convex shape against a half-space.
         *
         * Analytic, because a plane has no support point in the direction away from
         * its solid side — its support function runs off to infinity, and GJK on an
         * unbounded shape does not converge. The deepest point of the shape along
         * the inward normal is the whole answer.
         */
        template <typename T, typename ShapeA>
        inline ConvexContact<T> convex_plane_closest_entry(const CollisionShape<T>& a,
                                                           const CollisionShape<T>& b) noexcept
        {
            const auto shape = ShapeTraits<T, ShapeA>::from(a);
            const PlaneCollider<T> plane{b.plane_normal, b.plane_offset};
            const Vector3T<T> deepest = support(shape, plane.normal * T(-1));
            ConvexContact<T> contact;
            contact.valid = true;
            contact.normal = plane.normal * T(-1); // from the shape toward the plane
            contact.separation = dot(plane.normal, deepest) - plane.offset;
            contact.point_a = deepest;
            contact.point_b = deepest - plane.normal * contact.separation;
            return contact;
        }

        /** @brief The flip of the above, so both orders resolve. */
        template <typename T, typename ShapeB>
        inline ConvexContact<T> plane_convex_closest_entry(const CollisionShape<T>& a,
                                                           const CollisionShape<T>& b) noexcept
        {
            ConvexContact<T> contact = convex_plane_closest_entry<T, ShapeB>(b, a);
            contact.normal = contact.normal * T(-1);
            const Vector3T<T> swap = contact.point_a;
            contact.point_a = contact.point_b;
            contact.point_b = swap;
            return contact;
        }

        /** @brief One closest-point routine per ordered shape pair. */
        template <typename T>
        struct ClosestPointTable
        {
            static constexpr std::size_t kind_count = static_cast<std::size_t>(ShapeType::count);
            ClosestFunction<T> entries[kind_count][kind_count] = {};

            ClosestFunction<T> get(ShapeType a, ShapeType b) const noexcept
            {
                return entries[static_cast<std::size_t>(a)][static_cast<std::size_t>(b)];
            }
        };

        /** @brief Fills one row: @p ShapeA against every convex shape, and the plane. */
        template <typename T, typename ShapeA, typename... Shapes>
        inline void register_closest_row(ClosestPointTable<T>& table, TypeList<Shapes...>) noexcept
        {
            const std::size_t row = static_cast<std::size_t>(ShapeTraits<T, ShapeA>::type);
            ((table.entries[row][static_cast<std::size_t>(ShapeTraits<T, Shapes>::type)] =
                  &convex_closest_entry<T, ShapeA, Shapes>),
             ...);
            table.entries[row][static_cast<std::size_t>(ShapeType::plane)] =
                &convex_plane_closest_entry<T, ShapeA>;
            table.entries[static_cast<std::size_t>(ShapeType::plane)][row] =
                &plane_convex_closest_entry<T, ShapeA>;
        }

        /** @brief Fills the whole block. */
        template <typename T, typename... Shapes>
        inline void register_closest_shapes(ClosestPointTable<T>& table,
                                            TypeList<Shapes...> list) noexcept
        {
            (register_closest_row<T, Shapes>(table, list), ...);
        }

        /** @brief The one closest-point table, built on first use. */
        template <typename T>
        inline const ClosestPointTable<T>& closest_point_table() noexcept
        {
            static const ClosestPointTable<T> table = []()
            {
                ClosestPointTable<T> built;
                register_closest_shapes<T>(built, ConvexShapes<T>{});
                return built;
            }();
            return table;
        }

        /**
         * @brief The closest pair of points on two type-erased shapes.
         *
         * @return The contact; `valid` is false when the pair has no registered
         *         routine. `separation` is negative when they overlap, so one call
         *         answers both "how far apart" and "do they touch".
         */
        template <typename T>
        inline ConvexContact<T> closest_shapes(const CollisionShape<T>& a,
                                               const CollisionShape<T>& b) noexcept
        {
            const ClosestFunction<T> entry = closest_point_table<T>().get(a.type, b.type);
            if (entry == nullptr)
                return ConvexContact<T>{};
            return entry(a, b);
        }

        /** @brief Whether two type-erased shapes overlap at all. */
        template <typename T>
        inline bool shapes_overlap(const CollisionShape<T>& a, const CollisionShape<T>& b) noexcept
        {
            const ConvexContact<T> contact = closest_shapes(a, b);
            return contact.valid && contact.separation <= T(0);
        }

        // Queries over a broadphase

        /**
         * @brief The closest thing a ray hits.
         *
         * @param broadphase The tree to descend.
         * @param shape_of   Called as `shape_of(proxy)`; returns its `CollisionShape`.
         * @param origin     The ray's origin.
         * @param direction  Unit direction.
         * @param max_distance How far to look.
         * @param filter     Which proxies to consider.
         * @return The nearest hit, or a hit with `hit == false`.
         */
        template <typename T, typename ShapeOf>
        inline RayHit<T> raycast_closest(const IBroadphase<T>& broadphase, ShapeOf&& shape_of,
                                         const Vector3T<T>& origin, const Vector3T<T>& direction,
                                         T max_distance, const QueryFilter<T>& filter = {})
        {
            RayHit<T> best;
            broadphase.query_ray(
                origin, direction, max_distance,
                [&](ProxyId id)
                {
                    const BroadphaseProxy<T>& record = broadphase.proxy(id);
                    if (!filter.accepts(id, record))
                        return;
                    RayHit<T> hit;
                    if (!ray_cast_shape<T>(shape_of(id), origin, direction, max_distance, hit))
                        return;
                    // Ties break by proxy identifier, never by which branch the tree
                    // happened to visit first: two coincident surfaces must answer
                    // the same way on every run.
                    if (!best.hit || hit.distance < best.distance ||
                        (hit.distance == best.distance && id < best.proxy))
                    {
                        hit.proxy = id;
                        hit.payload = record.payload;
                        best = hit;
                    }
                });
            return best;
        }

        /** @brief Everything a ray hits, nearest first, ties broken by proxy. */
        template <typename T, typename ShapeOf>
        inline std::vector<RayHit<T>> raycast_all(const IBroadphase<T>& broadphase,
                                                  ShapeOf&& shape_of, const Vector3T<T>& origin,
                                                  const Vector3T<T>& direction, T max_distance,
                                                  const QueryFilter<T>& filter = {})
        {
            std::vector<RayHit<T>> hits;
            broadphase.query_ray(origin, direction, max_distance,
                                 [&](ProxyId id)
                                 {
                                     const BroadphaseProxy<T>& record = broadphase.proxy(id);
                                     if (!filter.accepts(id, record))
                                         return;
                                     RayHit<T> hit;
                                     if (!ray_cast_shape<T>(shape_of(id), origin, direction,
                                                            max_distance, hit))
                                         return;
                                     hit.proxy = id;
                                     hit.payload = record.payload;
                                     hits.push_back(hit);
                                 });
            std::sort(hits.begin(), hits.end(),
                      [](const RayHit<T>& l, const RayHit<T>& r) noexcept
                      {
                          return l.distance != r.distance ? l.distance < r.distance
                                                          : l.proxy < r.proxy;
                      });
            return hits;
        }

        /**
         * @brief Every proxy whose shape actually overlaps @p query, by proxy order.
         *
         * "Actually" is the whole difference between this and a broadphase query:
         * the tree answers with boxes, and a box overlap is not a shape overlap.
         * A trigger volume that fired on bounding boxes is a trigger volume that
         * fires in the doorway next to it.
         */
        template <typename T, typename ShapeOf>
        inline std::vector<ProxyId> overlap_shape(const IBroadphase<T>& broadphase,
                                                  ShapeOf&& shape_of,
                                                  const CollisionShape<T>& query,
                                                  const QueryFilter<T>& filter = {})
        {
            std::vector<ProxyId> found;
            broadphase.query_overlap(shape_world_bounds(query),
                                     [&](ProxyId id)
                                     {
                                         const BroadphaseProxy<T>& record = broadphase.proxy(id);
                                         if (!filter.accepts(id, record))
                                             return;
                                         if (shapes_overlap<T>(query, shape_of(id)))
                                             found.push_back(id);
                                     });
            std::sort(found.begin(), found.end());
            return found;
        }

        /** @brief What a closest-point query answers with. */
        template <typename T>
        struct ClosestResult
        {
            bool found = false;
            T distance = 0;             /**< Negative when the shapes overlap. */
            Vector3T<T> point_on_query; /**< On the query shape. */
            Vector3T<T> point_on_shape; /**< On the proxy's shape. */
            Vector3T<T> normal{};       /**< From the query shape toward the proxy's. */
            ProxyId proxy = null_proxy;
            std::uint32_t payload = 0;
        };

        /**
         * @brief The nearest proxy to a shape, within @p max_distance.
         *
         * The search box is the query's bounds grown by the search radius, so the
         * cost is a tree descent over a small region rather than a scan — which is
         * §7.7's whole requirement, and the reason this takes a maximum rather than
         * offering to search the world.
         */
        template <typename T, typename ShapeOf>
        inline ClosestResult<T> closest_point(const IBroadphase<T>& broadphase, ShapeOf&& shape_of,
                                              const CollisionShape<T>& query, T max_distance,
                                              const QueryFilter<T>& filter = {})
        {
            ClosestResult<T> best;
            const AABB<T> search = aabb_expand(shape_world_bounds(query), max_distance);
            broadphase.query_overlap(
                search,
                [&](ProxyId id)
                {
                    const BroadphaseProxy<T>& record = broadphase.proxy(id);
                    if (!filter.accepts(id, record))
                        return;
                    const ConvexContact<T> contact = closest_shapes<T>(query, shape_of(id));
                    if (!contact.valid || contact.separation > max_distance)
                        return;
                    if (best.found && !(contact.separation < best.distance ||
                                        (contact.separation == best.distance && id < best.proxy)))
                        return;
                    best.found = true;
                    best.distance = contact.separation;
                    best.point_on_query = contact.point_a;
                    best.point_on_shape = contact.point_b;
                    best.normal = contact.normal;
                    best.proxy = id;
                    best.payload = record.payload;
                });
            return best;
        }

        /**
         * @brief Moves a shape along a direction until something stops it.
         *
         * Conservative advancement again, this time with a shape instead of a
         * point: the same argument that makes the ray safe makes this safe, since
         * a convex shape's closest distance bounds how far it can travel before
         * touching. A sweep is what a character step, a camera pull-in and a
         * placement check all are, and it is tier two of §7.5 in query form.
         *
         * @param broadphase The tree to descend.
         * @param shape_of   Called as `shape_of(proxy)`.
         * @param query      The shape to move, at its starting pose.
         * @param direction  Unit direction of travel.
         * @param distance   How far to try to move.
         * @param filter     Which proxies to consider.
         * @return The first blocking hit, with `distance` the travel before contact.
         */
        template <typename T, typename ShapeOf>
        inline RayHit<T> sweep_shape(const IBroadphase<T>& broadphase, ShapeOf&& shape_of,
                                     const CollisionShape<T>& query,
                                     const Vector3T<T>& direction, T distance,
                                     const QueryFilter<T>& filter = {})
        {
            constexpr T tolerance = T(1e-6);
            RayHit<T> best;

            // One box covering the whole sweep: the candidates are found once, not
            // once per advancement step.
            const AABB<T> start = shape_world_bounds(query);
            const AABB<T> finish = shape_world_bounds(translate_shape(query, direction * distance));
            broadphase.query_overlap(
                aabb_union(start, finish),
                [&](ProxyId id)
                {
                    const BroadphaseProxy<T>& record = broadphase.proxy(id);
                    if (!filter.accepts(id, record))
                        return;
                    const CollisionShape<T> target = shape_of(id);

                    T travelled = T(0);
                    for (int iteration = 0; iteration < 48; ++iteration)
                    {
                        const ConvexContact<T> contact = closest_shapes<T>(
                            translate_shape(query, direction * travelled), target);
                        if (!contact.valid)
                            return;
                        if (contact.separation <= tolerance)
                        {
                            if (best.hit && !(travelled < best.distance ||
                                              (travelled == best.distance && id < best.proxy)))
                                return;
                            best.hit = true;
                            best.distance = travelled;
                            best.point = contact.point_b;
                            best.normal = contact.normal * T(-1);
                            best.proxy = id;
                            best.payload = record.payload;
                            return;
                        }
                        const T approach = dot(direction, contact.normal);
                        if (approach <= T(1e-12))
                            return;
                        travelled += contact.separation / approach;
                        if (travelled > distance)
                            return;
                    }
                });
            return best;
        }
    } // namespace Physics
} // namespace SushiEngine
