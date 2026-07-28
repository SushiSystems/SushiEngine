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
            count
        };

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
    } // namespace Physics
} // namespace SushiEngine
