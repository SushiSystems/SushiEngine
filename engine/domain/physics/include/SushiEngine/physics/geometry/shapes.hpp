/**************************************************************************/
/* shapes.hpp                                                             */
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
 * @file shapes.hpp
 * @brief The collision shape value types and the pure queries over one shape.
 *
 * `physics/geometry` owns shapes and the mathematics of a single shape; it knows
 * nothing about bodies, constraints, or the solver. Anything that takes *two*
 * shapes and reports how they overlap belongs one layer up in
 * `physics/collision`. Keeping that line means a new shape is a value type plus a
 * support function here, plus a narrowphase registration there — never an edit to
 * an existing shape.
 *
 * Every type is trivially copyable and element-parametric (`T` is `float` or
 * `double`), so a shape crosses into device code untouched.
 */

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief Which shape a collider is, for narrowphase dispatch.
         *
         * The ordered pair of two of these keys the dispatch table (§4.2): adding a
         * shape appends an enumerator and registers its pair functions, and no
         * existing file grows a branch. `count` is the table's dimension and must
         * stay last.
         */
        enum class ShapeType : std::uint8_t
        {
            sphere = 0,
            plane,
            box,
            oriented_box,
            capsule,
            convex_hull,
            /**
             * @brief A cooked signed-distance field, placed in the world (§7.5).
             *
             * Not a bounded convex set — like `plane`, it takes explicit dispatch
             * entries rather than a `support()` overload, because there is no
             * meaningful "furthest point along a direction" for an implicit volume.
             */
            signed_distance_field,
            count
        };

        /** @brief An axis-aligned bounding box: its minimum and maximum corners. */
        template <typename T>
        struct AABB
        {
            Vector3T<T> min;
            Vector3T<T> max;
        };

        /** @brief Whether two AABBs overlap on all three axes. */
        template <typename T>
        inline bool aabb_overlap(const AABB<T>& a, const AABB<T>& b) noexcept
        {
            return a.min.x <= b.max.x && b.min.x <= a.max.x && a.min.y <= b.max.y &&
                   b.min.y <= a.max.y && a.min.z <= b.max.z && b.min.z <= a.max.z;
        }

        /** @brief The smallest box containing both. */
        template <typename T>
        inline AABB<T> aabb_union(const AABB<T>& a, const AABB<T>& b) noexcept
        {
            return AABB<T>{Vector3T<T>{a.min.x < b.min.x ? a.min.x : b.min.x,
                                       a.min.y < b.min.y ? a.min.y : b.min.y,
                                       a.min.z < b.min.z ? a.min.z : b.min.z},
                           Vector3T<T>{a.max.x > b.max.x ? a.max.x : b.max.x,
                                       a.max.y > b.max.y ? a.max.y : b.max.y,
                                       a.max.z > b.max.z ? a.max.z : b.max.z}};
        }

        /** @brief A box grown by @p margin on every side. */
        template <typename T>
        inline AABB<T> aabb_expand(const AABB<T>& box, T margin) noexcept
        {
            return AABB<T>{
                Vector3T<T>{box.min.x - margin, box.min.y - margin, box.min.z - margin},
                Vector3T<T>{box.max.x + margin, box.max.y + margin, box.max.z + margin}};
        }

        /** @brief Whether @p outer encloses @p inner entirely. */
        template <typename T>
        inline bool aabb_contains(const AABB<T>& outer, const AABB<T>& inner) noexcept
        {
            return outer.min.x <= inner.min.x && outer.min.y <= inner.min.y &&
                   outer.min.z <= inner.min.z && outer.max.x >= inner.max.x &&
                   outer.max.y >= inner.max.y && outer.max.z >= inner.max.z;
        }

        /**
         * @brief A box's surface area — the cost a hierarchy's insertion minimizes.
         *
         * Surface area rather than volume because the quantity that matters is the
         * probability that a random ray or a random neighbouring box meets this one,
         * and for a convex body that probability is proportional to its surface area.
         * A flat box has almost no volume and is not remotely free to traverse.
         */
        template <typename T>
        inline T aabb_surface_area(const AABB<T>& box) noexcept
        {
            const Vector3T<T> size = box.max - box.min;
            const T x = size.x > T(0) ? size.x : T(0);
            const T y = size.y > T(0) ? size.y : T(0);
            const T z = size.z > T(0) ? size.z : T(0);
            return T(2) * (x * y + y * z + z * x);
        }

        /**
         * @brief The component-wise reciprocal a slab test wants, with no infinities.
         *
         * A zero component becomes a large finite number rather than an infinity, so
         * the slab arithmetic downstream can never form `0 * infinity` — which is the
         * one case that turns an axis-parallel ray, the most common kind an editor
         * ever casts, into a not-a-number and a silently missed hit.
         */
        template <typename T>
        inline Vector3T<T> ray_inverse_direction(const Vector3T<T>& direction) noexcept
        {
            constexpr T huge = T(1e30);
            const auto reciprocal = [](T value) noexcept -> T
            {
                if (value > T(0))
                    return value > T(1) / huge ? T(1) / value : huge;
                if (value < T(0))
                    return value < T(-1) / huge ? T(1) / value : -huge;
                return huge;
            };
            return Vector3T<T>{reciprocal(direction.x), reciprocal(direction.y),
                               reciprocal(direction.z)};
        }

        /**
         * @brief Whether a ray meets a box within @p max_distance, by slabs.
         *
         * The direction arrives as its reciprocal because a query tests one ray
         * against hundreds of boxes: the division belongs to the query, not to the
         * node. Use @ref ray_inverse_direction to form it.
         *
         * @param box               The box to test.
         * @param origin            The ray's origin.
         * @param inverse_direction Component-wise `1 / direction`.
         * @param max_distance      How far along the ray to look.
         * @return True when the ray is inside the box somewhere before @p max_distance.
         */
        template <typename T>
        inline bool ray_hits_aabb(const AABB<T>& box, const Vector3T<T>& origin,
                                  const Vector3T<T>& inverse_direction, T max_distance) noexcept
        {
            T near_distance = T(0);
            T far_distance = max_distance;

            const T origin_components[3] = {origin.x, origin.y, origin.z};
            const T inverse_components[3] = {inverse_direction.x, inverse_direction.y,
                                             inverse_direction.z};
            const T min_components[3] = {box.min.x, box.min.y, box.min.z};
            const T max_components[3] = {box.max.x, box.max.y, box.max.z};

            for (int axis = 0; axis < 3; ++axis)
            {
                T entry = (min_components[axis] - origin_components[axis]) *
                          inverse_components[axis];
                T exit = (max_components[axis] - origin_components[axis]) *
                         inverse_components[axis];
                if (entry > exit)
                {
                    const T swap = entry;
                    entry = exit;
                    exit = swap;
                }
                if (entry > near_distance)
                    near_distance = entry;
                if (exit < far_distance)
                    far_distance = exit;
                if (near_distance > far_distance)
                    return false;
            }
            return true;
        }

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

        /**
         * @brief A capsule: a line segment swept by a sphere.
         *
         * The segment runs along the shape's **local Y axis**, from
         * `-half_height` to `+half_height`, and every point within `radius` of it is
         * inside. Stated as a swept sphere rather than as a cylinder with two caps
         * because that is what makes it cheap: every query reduces to the
         * corresponding sphere query against the closest point on the segment, and
         * its support function is the segment's support plus `radius` — no faces, no
         * edges, no special cases at the seam.
         *
         * The shape a character controller and a limb are, and the reason it is the
         * first shape P2 adds.
         */
        template <typename T>
        struct CapsuleCollider
        {
            Vector3T<T> center;
            QuaternionT<T> orientation{QuaternionT<T>{T(0), T(0), T(0), T(1)}};
            T half_height = T(0.5); /**< Half the segment's length, excluding the caps. */
            T radius = T(0.5);
        };

        /**
         * @brief A borrowed view of a convex hull's vertices, posed in the world.
         *
         * A *view*, not an owner: the vertex array is cooked (§5.4) and shared by
         * every instance of the asset, so a shape stays small and trivially
         * copyable and a thousand crates do not hold a thousand vertex arrays. The
         * vertices are in the hull's own frame; @ref center and @ref orientation
         * place it.
         *
         * Only vertices are needed, because everything the narrowphase asks of a
         * convex shape goes through its support function, and a support function
         * needs no faces, no edges, and no adjacency. Faces arrive with the cooker
         * (P4) for the contact-patch path; the general convex routine does not wait
         * for them.
         */
        template <typename T>
        struct ConvexHullView
        {
            const Vector3T<T>* vertices = nullptr;
            std::uint32_t vertex_count = 0;
            Vector3T<T> center;
            QuaternionT<T> orientation{QuaternionT<T>{T(0), T(0), T(0), T(1)}};
            /**
             * @brief A small inflation applied to the hull, in metres.
             *
             * The standard trick (§5.2): closest-point routines are numerically
             * fragile on exactly-touching flat features, and a hull that is a hair
             * fatter than its vertices never has to resolve that case. The cooker
             * shrinks the vertices by the same amount, so the *shape* is unchanged.
             */
            T convex_radius = 0;
        };

        /**
         * @brief A cooked signed-distance field, placed in the world.
         *
         * A non-owning view over a `.sushicollision` blob's baked field (§8.4,
         * `physics/cooking/collision_asset.hpp`), the same reference-not-own
         * pattern `ConvexHullView` uses for a hull's vertices. `distances` holds
         * `resolution^3` values in the asset's own local frame, indexed
         * `x + resolution * (y + resolution * z)`; @ref center and @ref
         * orientation place that frame in the world, exactly as they do for every
         * other shape here.
         *
         * Deliberately not part of `ConvexShapes` (§4.2): an implicit volume has
         * no support function, so it takes explicit narrowphase entries instead —
         * `physics/collision/sdf_manifold.hpp` — the same treatment a half-space
         * plane gets and for the same reason.
         */
        template <typename T>
        struct SDFCollider
        {
            const float* distances = nullptr;
            std::int32_t resolution = 0;
            /** @brief Padded bounds of the baked cube, in the asset's own local frame. */
            Vector3T<T> field_min{Vector3T<T>{T(0), T(0), T(0)}};
            Vector3T<T> field_max{Vector3T<T>{T(0), T(0), T(0)}};
            Vector3T<T> center{Vector3T<T>{T(0), T(0), T(0)}};
            QuaternionT<T> orientation{QuaternionT<T>{T(0), T(0), T(0), T(1)}};
        };

        /**
         * @brief Half a voxel, in world units — the step a central-difference
         *        gradient samples at.
         *
         * Half a voxel rather than a fixed epsilon: a field baked at the fidelity
         * dial's lowest resolution has voxels centimetres wide, and a fixed
         * millimetre step would sample the same voxel twice and report a zero
         * gradient there.
         */
        template <typename T>
        inline T sdf_gradient_epsilon(const SDFCollider<T>& field) noexcept
        {
            if (field.resolution <= 0)
                return T(1e-3);
            const Vector3T<T> span = field.field_max - field.field_min;
            const T smallest = std::min(span.x, std::min(span.y, span.z));
            const T voxel = smallest / T(field.resolution);
            return voxel * T(0.5);
        }

        /**
         * @brief Nearest-voxel sample of the field at a point in its own local frame.
         *
         * The same convention `Cooking::collision_asset_distance` uses — clamped
         * to the brick rather than refused, because a query outside the padded
         * bounds (a shape still approaching from a distance) is the ordinary
         * case, and the nearest boundary voxel is the furthest-outside value
         * available, which is the right answer there.
         *
         * @return The signed distance, or the largest finite @p T when the field
         *         is empty — a caller must treat that as "nothing to collide
         *         with" rather than as a real, very-far-away sample.
         */
        template <typename T>
        inline T sdf_sample_local(const SDFCollider<T>& field,
                                  const Vector3T<T>& local_point) noexcept
        {
            if (field.distances == nullptr || field.resolution <= 0)
                return std::numeric_limits<T>::max();

            const T component[3] = {local_point.x, local_point.y, local_point.z};
            const T low[3] = {field.field_min.x, field.field_min.y, field.field_min.z};
            const T high[3] = {field.field_max.x, field.field_max.y, field.field_max.z};
            std::int32_t voxel[3];
            for (int axis = 0; axis < 3; ++axis)
            {
                const T span = high[axis] - low[axis];
                if (!(span > T(0)))
                    return std::numeric_limits<T>::max();
                const T normalized = (component[axis] - low[axis]) / span;
                std::int32_t index = static_cast<std::int32_t>(normalized * T(field.resolution));
                if (index < 0)
                    index = 0;
                if (index >= field.resolution)
                    index = field.resolution - 1;
                voxel[axis] = index;
            }
            const std::size_t offset =
                std::size_t(voxel[0]) +
                std::size_t(field.resolution) *
                    (std::size_t(voxel[1]) + std::size_t(field.resolution) * std::size_t(voxel[2]));
            return T(field.distances[offset]);
        }

        /** @brief Nearest-voxel sample of the field at a point in world space. */
        template <typename T>
        inline T sdf_sample_world(const SDFCollider<T>& field,
                                  const Vector3T<T>& world_point) noexcept
        {
            const Vector3T<T> local = rotate(conjugate(field.orientation), world_point - field.center);
            return sdf_sample_local(field, local);
        }

        /**
         * @brief The field's gradient at a world point, by central difference.
         *
         * A signed distance field's gradient is unit length and points away from
         * the nearest surface point — outward whether the query is inside or
         * outside the solid (the eikonal property `|grad d| = 1`) — so this
         * doubles as the surface normal at whatever point turns out to be
         * nearest, with no separate normal computation.
         *
         * Returns a fixed axis when the field is empty or the sample is outside
         * the brick on every side (a degenerate case, not a direction to trust),
         * so a caller never receives a zero vector to divide by.
         */
        template <typename T>
        inline Vector3T<T> sdf_gradient_world(const SDFCollider<T>& field,
                                              const Vector3T<T>& world_point) noexcept
        {
            const T epsilon = sdf_gradient_epsilon(field);
            const Vector3T<T> local =
                rotate(conjugate(field.orientation), world_point - field.center);
            const T dx = sdf_sample_local(field, local + Vector3T<T>{epsilon, T(0), T(0)}) -
                        sdf_sample_local(field, local - Vector3T<T>{epsilon, T(0), T(0)});
            const T dy = sdf_sample_local(field, local + Vector3T<T>{T(0), epsilon, T(0)}) -
                        sdf_sample_local(field, local - Vector3T<T>{T(0), epsilon, T(0)});
            const T dz = sdf_sample_local(field, local + Vector3T<T>{T(0), T(0), epsilon}) -
                        sdf_sample_local(field, local - Vector3T<T>{T(0), T(0), epsilon});
            Vector3T<T> local_gradient{dx, dy, dz};
            const T length_squared = dot(local_gradient, local_gradient);
            // A flat sample (every neighbour reads the same value, at the edge of
            // an empty field or a degenerate brick) has no direction to trust; a
            // fixed axis is at least a stable, reproducible answer rather than a
            // division by zero.
            local_gradient = length_squared > T(1e-24)
                                 ? local_gradient * (T(1) / std::sqrt(length_squared))
                                 : Vector3T<T>{T(0), T(1), T(0)};
            return rotate(field.orientation, local_gradient);
        }

        /**
         * @brief Trilinear sample of the field, with the interpolant's own gradient.
         *
         * @ref sdf_sample_local reads the nearest voxel, which is the right answer
         * for a convex pair: one shape produces one sample per tick, and half a
         * voxel of quantization is smaller than the manifold's other errors. A
         * soft body queries the field **once per surface vertex, every tick**
         * (§9.6.1), and there the piecewise-constant field shows in two ways a
         * single sample never does. A surface resting on it settles onto a
         * staircase rather than a plane, and — worse — a central difference over
         * two samples that land in the same voxel reads exactly zero, so
         * @ref sdf_gradient_world falls through to its fixed-axis guard and hands
         * back a normal that has nothing to do with the surface.
         *
         * So this reads the eight surrounding voxel centres and interpolates,
         * which makes both the value and the gradient continuous, and returns the
         * gradient *analytically* from the same eight values rather than by
         * differencing the interpolant again — six more samples for a quantity the
         * first eight already determine.
         *
         * Outside the brick the coordinates clamp, exactly as the nearest-voxel
         * sampler's do: the field is constant there and the gradient is zero,
         * which a caller reads as "nothing near enough to collide with" because
         * the padded boundary values are positive by construction.
         *
         * @tparam T The scalar element type.
         * @param field        The field.
         * @param local_point  The query point, in the field's own local frame.
         * @param out_gradient Receives the interpolant's gradient in the same local
         *                     frame, unnormalized; zero where the field is flat.
         * @return The interpolated signed distance, or the largest finite @p T when
         *         the field is empty — the same "nothing to collide with" answer
         *         @ref sdf_sample_local gives.
         */
        template <typename T>
        inline T sdf_sample_interpolated_local(const SDFCollider<T>& field,
                                               const Vector3T<T>& local_point,
                                               Vector3T<T>& out_gradient) noexcept
        {
            out_gradient = Vector3T<T>{T(0), T(0), T(0)};
            if (field.distances == nullptr || field.resolution <= 0)
                return std::numeric_limits<T>::max();

            const T component[3] = {local_point.x, local_point.y, local_point.z};
            const T low[3] = {field.field_min.x, field.field_min.y, field.field_min.z};
            const T high[3] = {field.field_max.x, field.field_max.y, field.field_max.z};
            std::int32_t low_voxel[3];
            std::int32_t high_voxel[3];
            T fraction[3];
            T voxels_per_unit[3];
            for (int axis = 0; axis < 3; ++axis)
            {
                const T span = high[axis] - low[axis];
                if (!(span > T(0)))
                    return std::numeric_limits<T>::max();

                // Voxel `i` holds the value at the centre of its cell, so the
                // continuous coordinate the eight neighbours are indexed by is the
                // normalized position in voxels, shifted back by half a cell.
                const T coordinate =
                    ((component[axis] - low[axis]) / span) * T(field.resolution) - T(0.5);
                std::int32_t index = static_cast<std::int32_t>(std::floor(coordinate));
                if (index < 0)
                    index = 0;
                if (index > field.resolution - 1)
                    index = field.resolution - 1;
                T offset = coordinate - T(index);
                if (offset < T(0))
                    offset = T(0);
                if (offset > T(1))
                    offset = T(1);

                low_voxel[axis] = index;
                high_voxel[axis] = index + 1 < field.resolution ? index + 1 : index;
                fraction[axis] = offset;
                voxels_per_unit[axis] = T(field.resolution) / span;
            }

            const std::size_t resolution = std::size_t(field.resolution);
            const auto value_at = [&](int x, int y, int z) -> T
            {
                const std::size_t ix = std::size_t(x == 0 ? low_voxel[0] : high_voxel[0]);
                const std::size_t iy = std::size_t(y == 0 ? low_voxel[1] : high_voxel[1]);
                const std::size_t iz = std::size_t(z == 0 ? low_voxel[2] : high_voxel[2]);
                return T(field.distances[ix + resolution * (iy + resolution * iz)]);
            };

            const T v000 = value_at(0, 0, 0);
            const T v100 = value_at(1, 0, 0);
            const T v010 = value_at(0, 1, 0);
            const T v110 = value_at(1, 1, 0);
            const T v001 = value_at(0, 0, 1);
            const T v101 = value_at(1, 0, 1);
            const T v011 = value_at(0, 1, 1);
            const T v111 = value_at(1, 1, 1);

            const T tx = fraction[0];
            const T ty = fraction[1];
            const T tz = fraction[2];

            const T edge_00 = v000 + (v100 - v000) * tx;
            const T edge_10 = v010 + (v110 - v010) * tx;
            const T edge_01 = v001 + (v101 - v001) * tx;
            const T edge_11 = v011 + (v111 - v011) * tx;
            const T face_0 = edge_00 + (edge_10 - edge_00) * ty;
            const T face_1 = edge_01 + (edge_11 - edge_01) * ty;

            const T along_x_00 = v100 - v000;
            const T along_x_10 = v110 - v010;
            const T along_x_01 = v101 - v001;
            const T along_x_11 = v111 - v011;
            const T along_x_0 = along_x_00 + (along_x_10 - along_x_00) * ty;
            const T along_x_1 = along_x_01 + (along_x_11 - along_x_01) * ty;

            out_gradient = Vector3T<T>{
                (along_x_0 + (along_x_1 - along_x_0) * tz) * voxels_per_unit[0],
                ((edge_10 - edge_00) + ((edge_11 - edge_01) - (edge_10 - edge_00)) * tz) *
                    voxels_per_unit[1],
                (face_1 - face_0) * voxels_per_unit[2]};

            return face_0 + (face_1 - face_0) * tz;
        }

        /**
         * @brief Trilinear sample of the field at a world point, with a unit world normal.
         *
         * The world-space face of @ref sdf_sample_interpolated_local: the query
         * rotates into the field's frame, the gradient rotates back out, and it is
         * normalized here because every caller wants a direction rather than a
         * slope. A flat neighbourhood — an empty field, or a query clamped outside
         * the brick — yields a fixed axis rather than a zero vector, matching
         * @ref sdf_gradient_world so a caller never receives something to divide by.
         *
         * @tparam T The scalar element type.
         * @param field      The field, placed in the world.
         * @param world_point The query point, in world space.
         * @param out_normal Receives the unit outward normal at the nearest surface point.
         * @return The interpolated signed distance; negative inside the solid.
         */
        template <typename T>
        inline T sdf_sample_interpolated_world(const SDFCollider<T>& field,
                                               const Vector3T<T>& world_point,
                                               Vector3T<T>& out_normal) noexcept
        {
            const Vector3T<T> local =
                rotate(conjugate(field.orientation), world_point - field.center);
            Vector3T<T> local_gradient;
            const T distance = sdf_sample_interpolated_local(field, local, local_gradient);
            const T length_squared = dot(local_gradient, local_gradient);
            local_gradient = length_squared > T(1e-24)
                                 ? local_gradient * (T(1) / std::sqrt(length_squared))
                                 : Vector3T<T>{T(0), T(1), T(0)};
            out_normal = rotate(field.orientation, local_gradient);
            return distance;
        }

        /** @brief The world-space box enclosing a signed-distance field's padded brick. */
        template <typename T>
        inline AABB<T> world_bounds(const SDFCollider<T>& field) noexcept
        {
            if (field.resolution <= 0)
                return AABB<T>{field.center, field.center};
            const Vector3T<T> corners_local[8] = {
                Vector3T<T>{field.field_min.x, field.field_min.y, field.field_min.z},
                Vector3T<T>{field.field_max.x, field.field_min.y, field.field_min.z},
                Vector3T<T>{field.field_min.x, field.field_max.y, field.field_min.z},
                Vector3T<T>{field.field_max.x, field.field_max.y, field.field_min.z},
                Vector3T<T>{field.field_min.x, field.field_min.y, field.field_max.z},
                Vector3T<T>{field.field_max.x, field.field_min.y, field.field_max.z},
                Vector3T<T>{field.field_min.x, field.field_max.y, field.field_max.z},
                Vector3T<T>{field.field_max.x, field.field_max.y, field.field_max.z}};
            Vector3T<T> low = field.center + rotate(field.orientation, corners_local[0]);
            Vector3T<T> high = low;
            for (int i = 1; i < 8; ++i)
            {
                const Vector3T<T> world = field.center + rotate(field.orientation, corners_local[i]);
                low = Vector3T<T>{std::min(low.x, world.x), std::min(low.y, world.y),
                                  std::min(low.z, world.z)};
                high = Vector3T<T>{std::max(high.x, world.x), std::max(high.y, world.y),
                                   std::max(high.z, world.z)};
            }
            return AABB<T>{low, high};
        }

        /**
         * @brief One triangle of a mesh, in world space.
         *
         * A triangle is a convex set, so it needs no narrowphase of its own: it gets
         * a `support()` overload like every other convex shape and collides against
         * spheres, boxes, capsules and hulls through the one general routine. That
         * is what makes "convex versus triangle mesh" (§7.2) a *traversal* problem
         * rather than a geometry problem — the hierarchy names the candidate
         * triangles and the existing machinery does the rest.
         */
        template <typename T>
        struct TriangleCollider
        {
            Vector3T<T> a;
            Vector3T<T> b;
            Vector3T<T> c;
        };

        /**
         * @brief The triangle's plane normal, unit length, by right-hand winding.
         *
         * Returns a zero vector for a degenerate triangle, which the caller must
         * treat as "no face here" rather than normalizing into a direction that
         * means nothing.
         */
        template <typename T>
        inline Vector3T<T> triangle_normal(const TriangleCollider<T>& triangle) noexcept
        {
            const Vector3T<T> normal = cross(triangle.b - triangle.a, triangle.c - triangle.a);
            const T length_squared = dot(normal, normal);
            if (length_squared <= T(1e-24))
                return Vector3T<T>{T(0), T(0), T(0)};
            return normal * (T(1) / std::sqrt(length_squared));
        }

        /** @brief The two endpoints of a capsule's segment, in world space. */
        template <typename T>
        inline void capsule_segment(const CapsuleCollider<T>& capsule, Vector3T<T>& start,
                                    Vector3T<T>& end) noexcept
        {
            const Vector3T<T> axis =
                rotate(capsule.orientation, Vector3T<T>{T(0), T(1), T(0)}) * capsule.half_height;
            start = capsule.center - axis;
            end = capsule.center + axis;
        }

        /**
         * @brief The three world-space axes (columns of the rotation) of an oriented box.
         *
         * @tparam T The scalar element type.
         * @param box  The box to read the orientation from.
         * @param axes Receives the three unit axes, in local x/y/z order.
         */
        template <typename T>
        inline void obb_axes(const OrientedBox<T>& box, Vector3T<T> axes[3]) noexcept
        {
            axes[0] = rotate(box.orientation, Vector3T<T>{T(1), T(0), T(0)});
            axes[1] = rotate(box.orientation, Vector3T<T>{T(0), T(1), T(0)});
            axes[2] = rotate(box.orientation, Vector3T<T>{T(0), T(0), T(1)});
        }

        /**
         * @brief The box's furthest point along @p direction (its support point).
         *
         * Steps out from the centre along each local axis by that axis's half-extent,
         * signed to follow @p direction. Contact resolution needs this rather than the
         * centre: the lever arm from the centre of mass to where a body is actually
         * touched is what turns a push into a torque.
         *
         * @tparam T The scalar element type.
         * @param box       The box to query.
         * @param direction The (not necessarily unit) direction to support along.
         * @return The world-space corner furthest along @p direction.
         */
        template <typename T>
        inline Vector3T<T> obb_support_point(const OrientedBox<T>& box,
                                             const Vector3T<T>& direction) noexcept
        {
            Vector3T<T> axes[3];
            obb_axes(box, axes);
            const T extents[3] = {box.half_extents.x, box.half_extents.y, box.half_extents.z};
            Vector3T<T> point = box.center;
            for (int i = 0; i < 3; ++i)
            {
                const T sign = dot(axes[i], direction) < T(0) ? T(-1) : T(1);
                point = point + axes[i] * (extents[i] * sign);
            }
            return point;
        }

        /**
         * @brief The axis of a box's local frame the point @p local lies nearest to.
         *
         * Answers "which face is this point about to leave through" for a point that
         * is *inside* the box, where no closest-surface direction exists: the face
         * whose plane the point is nearest to is the one it should be pushed out of.
         * The narrowphase needs this for the deep-penetration case, where clamping to
         * the surface returns the point itself and the usual normal is undefined.
         *
         * @tparam T The scalar element type.
         * @param local        The point, expressed in the box's local frame.
         * @param half_extents The box's per-axis half-extents.
         * @return A unit local-frame axis pointing out of the nearest face.
         */
        template <typename T>
        inline Vector3T<T> nearest_face_normal(const Vector3T<T>& local,
                                               const Vector3T<T>& half_extents) noexcept
        {
            const T gap[3] = {half_extents.x - (local.x < T(0) ? -local.x : local.x),
                              half_extents.y - (local.y < T(0) ? -local.y : local.y),
                              half_extents.z - (local.z < T(0) ? -local.z : local.z)};
            const T coordinate[3] = {local.x, local.y, local.z};

            int axis = 0;
            for (int i = 1; i < 3; ++i)
                if (gap[i] < gap[axis])
                    axis = i;

            // A point exactly on the centre plane of its nearest axis has no side to
            // prefer; +1 is chosen so the result is deterministic rather than
            // dependent on a signed zero.
            const T sign = coordinate[axis] < T(0) ? T(-1) : T(1);
            Vector3T<T> normal{T(0), T(0), T(0)};
            if (axis == 0)
                normal.x = sign;
            else if (axis == 1)
                normal.y = sign;
            else
                normal.z = sign;
            return normal;
        }

        /**
         * @brief How far inside the box @p local is, along @ref nearest_face_normal.
         *
         * The depth companion to @ref nearest_face_normal: the distance from the point
         * to the face it is nearest to. Reported separately because the narrowphase
         * adds the sphere radius to it, and combining the two would hide that.
         *
         * @tparam T The scalar element type.
         * @param local        The point, expressed in the box's local frame.
         * @param half_extents The box's per-axis half-extents.
         * @return The (non-negative) distance from @p local to the nearest face.
         */
        template <typename T>
        inline T nearest_face_distance(const Vector3T<T>& local,
                                       const Vector3T<T>& half_extents) noexcept
        {
            const T gap[3] = {half_extents.x - (local.x < T(0) ? -local.x : local.x),
                              half_extents.y - (local.y < T(0) ? -local.y : local.y),
                              half_extents.z - (local.z < T(0) ? -local.z : local.z)};
            T smallest = gap[0];
            for (int i = 1; i < 3; ++i)
                if (gap[i] < smallest)
                    smallest = gap[i];
            return smallest < T(0) ? T(0) : smallest;
        }

        // ------------------------------------------------------------------------
        // Support functions.
        //
        // A convex shape's support function returns its furthest point along a
        // direction, and it is the *only* thing the general convex narrowphase
        // (GJK/EPA, `geometry/gjk.hpp`) ever asks of a shape. That is the Open/Closed
        // payoff §4.2 promises, made concrete: a new convex shape adds one overload
        // of `support` here and gains hull-hull, hull-box, hull-capsule and every
        // other convex pairing for free, without a line changing anywhere else.
        //
        // Overloads rather than a switch on ShapeType, deliberately. A switch is a
        // file every new shape has to edit, which is the violation; an overload set
        // is resolved at the call site and a new one joins it by existing.
        // ------------------------------------------------------------------------

        /** @brief A unit vector along @p direction, or a fixed axis when it has no length. */
        template <typename T>
        inline Vector3T<T> safe_normalize(const Vector3T<T>& direction) noexcept
        {
            const T length_squared = dot(direction, direction);
            if (length_squared <= T(1e-24))
                return Vector3T<T>{T(1), T(0), T(0)};
            return direction * (T(1) / std::sqrt(length_squared));
        }

        /** @brief The sphere's furthest point along @p direction. */
        template <typename T>
        inline Vector3T<T> support(const SphereCollider<T>& sphere,
                                   const Vector3T<T>& direction) noexcept
        {
            return sphere.center + safe_normalize(direction) * sphere.radius;
        }

        /** @brief The oriented box's furthest point along @p direction. */
        template <typename T>
        inline Vector3T<T> support(const OrientedBox<T>& box,
                                   const Vector3T<T>& direction) noexcept
        {
            return obb_support_point(box, direction);
        }

        /**
         * @brief The capsule's furthest point along @p direction.
         *
         * The segment's furthest endpoint, pushed out by the radius: a swept sphere's
         * support is the sweep path's support plus the sphere's. No cap/side case.
         */
        template <typename T>
        inline Vector3T<T> support(const CapsuleCollider<T>& capsule,
                                   const Vector3T<T>& direction) noexcept
        {
            Vector3T<T> start;
            Vector3T<T> end;
            capsule_segment(capsule, start, end);
            const Vector3T<T>& furthest =
                dot(end, direction) >= dot(start, direction) ? end : start;
            return furthest + safe_normalize(direction) * capsule.radius;
        }

        // ------------------------------------------------------------------------
        // World bounds. Another overload set, for the same reason: the broadphase
        // and the mesh hierarchy both need a shape's box, and neither should have
        // to know what kinds of shape exist.
        // ------------------------------------------------------------------------

        /** @brief The world-space box enclosing a sphere. */
        template <typename T>
        inline AABB<T> world_bounds(const SphereCollider<T>& sphere) noexcept
        {
            const Vector3T<T> extent{sphere.radius, sphere.radius, sphere.radius};
            return AABB<T>{sphere.center - extent, sphere.center + extent};
        }

        /** @brief The world-space box enclosing an oriented box. */
        template <typename T>
        inline AABB<T> world_bounds(const OrientedBox<T>& box) noexcept
        {
            Vector3T<T> axes[3];
            obb_axes(box, axes);
            const T extents[3] = {box.half_extents.x, box.half_extents.y, box.half_extents.z};
            Vector3T<T> extent{T(0), T(0), T(0)};
            for (int i = 0; i < 3; ++i)
            {
                extent.x += std::abs(axes[i].x) * extents[i];
                extent.y += std::abs(axes[i].y) * extents[i];
                extent.z += std::abs(axes[i].z) * extents[i];
            }
            return AABB<T>{box.center - extent, box.center + extent};
        }

        /** @brief The world-space box enclosing a capsule. */
        template <typename T>
        inline AABB<T> world_bounds(const CapsuleCollider<T>& capsule) noexcept
        {
            Vector3T<T> start;
            Vector3T<T> end;
            capsule_segment(capsule, start, end);
            const Vector3T<T> radius{capsule.radius, capsule.radius, capsule.radius};
            const Vector3T<T> low{start.x < end.x ? start.x : end.x,
                                  start.y < end.y ? start.y : end.y,
                                  start.z < end.z ? start.z : end.z};
            const Vector3T<T> high{start.x > end.x ? start.x : end.x,
                                   start.y > end.y ? start.y : end.y,
                                   start.z > end.z ? start.z : end.z};
            return AABB<T>{low - radius, high + radius};
        }

        /** @brief The world-space box enclosing a triangle. */
        template <typename T>
        inline AABB<T> world_bounds(const TriangleCollider<T>& triangle) noexcept
        {
            AABB<T> bounds;
            bounds.min = Vector3T<T>{
                std::min(triangle.a.x, std::min(triangle.b.x, triangle.c.x)),
                std::min(triangle.a.y, std::min(triangle.b.y, triangle.c.y)),
                std::min(triangle.a.z, std::min(triangle.b.z, triangle.c.z))};
            bounds.max = Vector3T<T>{
                std::max(triangle.a.x, std::max(triangle.b.x, triangle.c.x)),
                std::max(triangle.a.y, std::max(triangle.b.y, triangle.c.y)),
                std::max(triangle.a.z, std::max(triangle.b.z, triangle.c.z))};
            return bounds;
        }

        /** @brief The world-space box enclosing a convex hull. */
        template <typename T>
        inline AABB<T> world_bounds(const ConvexHullView<T>& hull) noexcept
        {
            if (hull.vertices == nullptr || hull.vertex_count == 0)
                return AABB<T>{hull.center, hull.center};
            Vector3T<T> low = rotate(hull.orientation, hull.vertices[0]);
            Vector3T<T> high = low;
            for (std::uint32_t i = 1; i < hull.vertex_count; ++i)
            {
                const Vector3T<T> world = rotate(hull.orientation, hull.vertices[i]);
                low = Vector3T<T>{std::min(low.x, world.x), std::min(low.y, world.y),
                                  std::min(low.z, world.z)};
                high = Vector3T<T>{std::max(high.x, world.x), std::max(high.y, world.y),
                                   std::max(high.z, world.z)};
            }
            const Vector3T<T> inflation{hull.convex_radius, hull.convex_radius,
                                        hull.convex_radius};
            return AABB<T>{hull.center + low - inflation, hull.center + high + inflation};
        }

        /** @brief The triangle's furthest vertex along @p direction. */
        template <typename T>
        inline Vector3T<T> support(const TriangleCollider<T>& triangle,
                                   const Vector3T<T>& direction) noexcept
        {
            const T pa = dot(triangle.a, direction);
            const T pb = dot(triangle.b, direction);
            const T pc = dot(triangle.c, direction);
            if (pa >= pb && pa >= pc)
                return triangle.a;
            return pb >= pc ? triangle.b : triangle.c;
        }

        /**
         * @brief The hull's furthest vertex along @p direction, plus its convex radius.
         *
         * Linear in the vertex count. A hill-climb over face adjacency would be
         * logarithmic, and it is deliberately not done here: adjacency is cooked data
         * this shape does not carry yet (P4), and the cooker's piece budget keeps a
         * hull at a few dozen vertices, where the scan wins on cache behaviour
         * anyway. Revisit when a hull is measured to be large enough to care.
         *
         * A hull with no vertices supports its own centre, so a malformed asset
         * behaves as a point rather than reading past the end of nothing.
         */
        template <typename T>
        inline Vector3T<T> support(const ConvexHullView<T>& hull,
                                   const Vector3T<T>& direction) noexcept
        {
            const Vector3T<T> inflation = safe_normalize(direction) * hull.convex_radius;
            if (hull.vertices == nullptr || hull.vertex_count == 0)
                return hull.center + inflation;

            // Rotate the query into the hull's frame rather than every vertex into
            // the world's: one rotation instead of N.
            const Vector3T<T> local_direction =
                rotate(conjugate(hull.orientation), direction);
            std::uint32_t best = 0;
            T best_projection = dot(hull.vertices[0], local_direction);
            for (std::uint32_t i = 1; i < hull.vertex_count; ++i)
            {
                const T projection = dot(hull.vertices[i], local_direction);
                if (projection > best_projection)
                {
                    best_projection = projection;
                    best = i;
                }
            }
            return hull.center + rotate(hull.orientation, hull.vertices[best]) + inflation;
        }
    } // namespace Physics
} // namespace SushiEngine
