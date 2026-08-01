/**************************************************************************/
/* closest_point.hpp                                                      */
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
 * @file closest_point.hpp
 * @brief Closest features between the primitives a deformable surface is made of.
 *
 * Two routines, both returning *where* on the feature the answer lies rather
 * than only the point itself. That is the whole reason they are here and not
 * folded into an existing narrowphase file: a rigid pair only needs the world
 * point, because the shape it belongs to is a single body and the anchor is
 * expressed in that body's frame. A deformable pair needs the barycentric
 * weights, because the "body" is three or four independent particles and the
 * correction has to be split between them in proportion to how much each one
 * actually carries the contact — a vertex touching a triangle's corner must not
 * push the two far corners as hard as the near one.
 *
 * `collision/manifold.hpp` has a segment-segment routine of its own, written
 * for the edge-edge case of box-box: it takes centre/direction/half-length,
 * which is the form an oriented box's edges already have, and returns points.
 * Neither its parameterization nor its result is what a pair of mesh edges
 * needs, so this is a second routine rather than a call into that one.
 */

#include <cmath>

#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief The point of a triangle closest to @p point, and its barycentric weights.
         *
         * Ericson's region test: the answer lies in one of seven regions — the
         * three vertices, the three edges, or the face interior — and each is
         * decided by the signs of a handful of dot products rather than by
         * projecting and then checking. The weights always sum to one and are
         * never negative, so a caller can use them directly as the shares of a
         * correction without clamping them first.
         *
         * @tparam T The scalar element type.
         * @param point   The query point.
         * @param a,b,c   The triangle's corners.
         * @param weight  Receives the barycentric weights of the result, in corner order.
         * @return The closest point on the triangle.
         */
        template <typename T>
        inline Vector3T<T> closest_point_on_triangle(const Vector3T<T>& point,
                                                     const Vector3T<T>& a, const Vector3T<T>& b,
                                                     const Vector3T<T>& c, T weight[3]) noexcept
        {
            const Vector3T<T> ab = b - a;
            const Vector3T<T> ac = c - a;
            const Vector3T<T> ap = point - a;
            const T d1 = dot(ab, ap);
            const T d2 = dot(ac, ap);
            if (d1 <= T(0) && d2 <= T(0))
            {
                weight[0] = T(1);
                weight[1] = T(0);
                weight[2] = T(0);
                return a;
            }

            const Vector3T<T> bp = point - b;
            const T d3 = dot(ab, bp);
            const T d4 = dot(ac, bp);
            if (d3 >= T(0) && d4 <= d3)
            {
                weight[0] = T(0);
                weight[1] = T(1);
                weight[2] = T(0);
                return b;
            }

            const T vc = d1 * d4 - d3 * d2;
            if (vc <= T(0) && d1 >= T(0) && d3 <= T(0))
            {
                const T along = d1 / (d1 - d3);
                weight[0] = T(1) - along;
                weight[1] = along;
                weight[2] = T(0);
                return a + ab * along;
            }

            const Vector3T<T> cp = point - c;
            const T d5 = dot(ab, cp);
            const T d6 = dot(ac, cp);
            if (d6 >= T(0) && d5 <= d6)
            {
                weight[0] = T(0);
                weight[1] = T(0);
                weight[2] = T(1);
                return c;
            }

            const T vb = d5 * d2 - d1 * d6;
            if (vb <= T(0) && d2 >= T(0) && d6 <= T(0))
            {
                const T along = d2 / (d2 - d6);
                weight[0] = T(1) - along;
                weight[1] = T(0);
                weight[2] = along;
                return a + ac * along;
            }

            const T va = d3 * d6 - d5 * d4;
            if (va <= T(0) && (d4 - d3) >= T(0) && (d5 - d6) >= T(0))
            {
                const T along = (d4 - d3) / ((d4 - d3) + (d5 - d6));
                weight[0] = T(0);
                weight[1] = T(1) - along;
                weight[2] = along;
                return b + (c - b) * along;
            }

            const T total = va + vb + vc;
            // A degenerate triangle leaves every region test inconclusive and the
            // total at zero; the first corner is a stable answer where there is no
            // meaningful one, rather than a division by zero.
            if (!(total > T(0)) && !(total < T(0)))
            {
                weight[0] = T(1);
                weight[1] = T(0);
                weight[2] = T(0);
                return a;
            }

            const T inverse = T(1) / total;
            const T along_ab = vb * inverse;
            const T along_ac = vc * inverse;
            weight[0] = T(1) - along_ab - along_ac;
            weight[1] = along_ab;
            weight[2] = along_ac;
            return a + ab * along_ab + ac * along_ac;
        }

        /**
         * @brief Where two segments come closest, as a parameter along each.
         *
         * Ericson's clamped solve: the unconstrained closest pair of the two
         * infinite lines, then clamped to each segment in turn — clamping one
         * parameter changes where the other's optimum lies, which is why the
         * second is recomputed rather than clamped in place. Parallel segments
         * have a whole interval of equally close pairs; the branch there picks the
         * start of the first segment, which is stable and reproducible rather than
         * a division by something near zero.
         *
         * @tparam T The scalar element type.
         * @param p0,p1 The first segment's endpoints.
         * @param q0,q1 The second segment's endpoints.
         * @param s     Receives the parameter along the first segment, in [0, 1].
         * @param t     Receives the parameter along the second segment, in [0, 1].
         */
        template <typename T>
        inline void closest_points_on_edges(const Vector3T<T>& p0, const Vector3T<T>& p1,
                                            const Vector3T<T>& q0, const Vector3T<T>& q1, T& s,
                                            T& t) noexcept
        {
            const auto clamp_unit = [](T value) noexcept
            { return value < T(0) ? T(0) : (value > T(1) ? T(1) : value); };

            const Vector3T<T> direction_p = p1 - p0;
            const Vector3T<T> direction_q = q1 - q0;
            const Vector3T<T> offset = p0 - q0;
            const T length_p = dot(direction_p, direction_p);
            const T length_q = dot(direction_q, direction_q);
            const T offset_along_q = dot(direction_q, offset);
            const T degenerate = T(1e-18);

            if (length_p <= degenerate && length_q <= degenerate)
            {
                s = T(0);
                t = T(0);
                return;
            }
            if (length_p <= degenerate)
            {
                s = T(0);
                t = clamp_unit(offset_along_q / length_q);
                return;
            }

            const T offset_along_p = dot(direction_p, offset);
            if (length_q <= degenerate)
            {
                t = T(0);
                s = clamp_unit(-offset_along_p / length_p);
                return;
            }

            const T between = dot(direction_p, direction_q);
            const T denominator = length_p * length_q - between * between;
            s = denominator > degenerate
                    ? clamp_unit((between * offset_along_q - offset_along_p * length_q) /
                                 denominator)
                    : T(0);

            t = (between * s + offset_along_q) / length_q;
            if (t < T(0))
            {
                t = T(0);
                s = clamp_unit(-offset_along_p / length_p);
            }
            else if (t > T(1))
            {
                t = T(1);
                s = clamp_unit((between - offset_along_p) / length_p);
            }
        }
    } // namespace Physics
} // namespace SushiEngine
