/**************************************************************************/
/* continuous_proximity.hpp                                               */
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
 * @file continuous_proximity.hpp
 * @brief When, within one substep, a moving vertex meets a moving triangle or
 *        two moving edges meet.
 *
 * The test §9.6.2 calls continuous, after Bridson et al. A discrete proximity
 * test asks "are these two features close *now*", which is the right question
 * for a body thick enough that it cannot cross itself in one substep. A sheet a
 * millimetre thick moving at ten metres a second travels a hundred times its
 * own thickness in a substep, and every discrete test along the way answers
 * "nothing here": the two surfaces were apart before and are apart after, on
 * the wrong side.
 *
 * What is true throughout such a crossing is that the four points involved
 * become **coplanar** at the moment of contact. Assuming each point moves at a
 * constant velocity across the substep — which is exactly what a substep's
 * predict step makes true — that condition is a cubic in time:
 *
 *     f(t) = ( u(t) x v(t) ) . w(t) = 0
 *
 * where `u`, `v` and `w` are the three difference vectors that define the
 * feature pair, each linear in `t`. Its roots in `[0, 1]` are the candidate
 * moments; a root is only a *contact* if the features are actually near each
 * other there, which is checked by re-running the ordinary closest-feature test
 * at that instant. The cubic is what finds the moment; the proximity test is
 * what decides whether it means anything.
 *
 * The roots are found by bracketing rather than by the closed-form cubic
 * formula: the derivative's own roots split `[0, 1]` into at most three
 * intervals on which the cubic is monotone, and a sign change across a monotone
 * interval brackets exactly one root that forty bisections then pin down to
 * about a part in a trillion of the substep, far finer than the proximity
 * test that follows can distinguish. The closed form is shorter and is famously
 * ill-conditioned near the double roots this problem is full of — a grazing
 * contact *is* a double root — which is the case that matters most here.
 */

#include <cmath>
#include <cstddef>

#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief The cubic `((u0 + t du) x (v0 + t dv)) . (w0 + t dw)`, by coefficient.
         *
         * @tparam T The scalar element type.
         * @param u0,du The first difference vector at `t = 0` and its change across the step.
         * @param v0,dv The second.
         * @param w0,dw The third.
         * @param coefficient Receives `[t^0, t^1, t^2, t^3]`.
         */
        template <typename T>
        inline void coplanarity_coefficients(const Vector3T<T>& u0, const Vector3T<T>& du,
                                             const Vector3T<T>& v0, const Vector3T<T>& dv,
                                             const Vector3T<T>& w0, const Vector3T<T>& dw,
                                             T coefficient[4]) noexcept
        {
            const Vector3T<T> constant_cross = cross(u0, v0);
            const Vector3T<T> linear_cross = cross(u0, dv) + cross(du, v0);
            const Vector3T<T> quadratic_cross = cross(du, dv);

            coefficient[0] = dot(constant_cross, w0);
            coefficient[1] = dot(constant_cross, dw) + dot(linear_cross, w0);
            coefficient[2] = dot(linear_cross, dw) + dot(quadratic_cross, w0);
            coefficient[3] = dot(quadratic_cross, dw);
        }

        /**
         * @brief The roots of a cubic inside `[0, 1]`, ascending.
         *
         * Bracket and bisect over the intervals the derivative's roots cut `[0, 1]`
         * into, on each of which the cubic is monotone. Degenerate cases fall out
         * of the same machinery rather than needing their own branches: a cubic
         * whose leading coefficients vanish is a quadratic or a line, whose
         * derivative simply has fewer roots and therefore fewer intervals.
         *
         * @tparam T The scalar element type.
         * @param coefficient `[t^0, t^1, t^2, t^3]`.
         * @param root        Receives up to three roots, ascending.
         * @return How many roots were found.
         */
        template <typename T>
        inline int coplanarity_roots(const T coefficient[4], T root[3]) noexcept
        {
            const auto evaluate = [&](T t) noexcept
            {
                return ((coefficient[3] * t + coefficient[2]) * t + coefficient[1]) * t +
                       coefficient[0];
            };

            // The derivative's roots, which are where the cubic turns.
            T turn[2];
            int turn_count = 0;
            {
                const T a = T(3) * coefficient[3];
                const T b = T(2) * coefficient[2];
                const T c = coefficient[1];
                if (std::abs(a) <= T(1e-14))
                {
                    if (std::abs(b) > T(1e-14))
                    {
                        const T single = -c / b;
                        if (single > T(0) && single < T(1))
                            turn[turn_count++] = single;
                    }
                }
                else
                {
                    const T discriminant = b * b - T(4) * a * c;
                    if (discriminant >= T(0))
                    {
                        const T square_root = std::sqrt(discriminant);
                        T first = (-b - square_root) / (T(2) * a);
                        T second = (-b + square_root) / (T(2) * a);
                        if (first > second)
                        {
                            const T swap = first;
                            first = second;
                            second = swap;
                        }
                        if (first > T(0) && first < T(1))
                            turn[turn_count++] = first;
                        if (second > T(0) && second < T(1))
                            turn[turn_count++] = second;
                    }
                }
            }

            T boundary[4];
            int boundary_count = 0;
            boundary[boundary_count++] = T(0);
            for (int i = 0; i < turn_count; ++i)
                boundary[boundary_count++] = turn[i];
            boundary[boundary_count++] = T(1);

            int found = 0;
            // Two adjacent intervals share an endpoint, so a root sitting exactly on
            // one can be reported by both; each candidate is admitted only if it is
            // clear of the last one accepted. The intervals are walked in order, so
            // comparing against the last is enough to keep the result ascending and
            // free of repeats.
            const auto admit = [&](T candidate) noexcept
            {
                if (found > 0 && candidate - root[found - 1] <= T(1e-9))
                    return;
                if (found < 3)
                    root[found++] = candidate;
            };

            for (int i = 0; i + 1 < boundary_count && found < 3; ++i)
            {
                T low = boundary[i];
                T high = boundary[i + 1];
                T value_low = evaluate(low);
                const T value_high = evaluate(high);

                if (value_low == T(0))
                {
                    admit(low);
                    continue;
                }
                if (value_high == T(0))
                {
                    admit(high);
                    continue;
                }
                if (!((value_low < T(0)) != (value_high < T(0))))
                    continue;

                for (int step = 0; step < 40; ++step)
                {
                    const T middle = (low + high) * T(0.5);
                    const T value = evaluate(middle);
                    if ((value < T(0)) != (value_low < T(0)))
                    {
                        high = middle;
                    }
                    else
                    {
                        low = middle;
                        value_low = value;
                    }
                }
                admit((low + high) * T(0.5));
            }
            return found;
        }

        /**
         * @brief The candidate times at which a moving vertex is coplanar with a moving triangle.
         *
         * @tparam T The scalar element type.
         * @param start Positions at the substep's start: the vertex, then the
         *              triangle's three corners.
         * @param end   The same four points at the substep's end.
         * @param time  Receives up to three candidate times in `[0, 1]`, ascending.
         * @return How many candidates were found; each still has to be checked for
         *         actual proximity, since coplanar is not the same as touching.
         */
        template <typename T>
        inline int vertex_triangle_coplanarity_times(const Vector3T<T> start[4],
                                                     const Vector3T<T> end[4], T time[3]) noexcept
        {
            const Vector3T<T> u0 = start[2] - start[1];
            const Vector3T<T> v0 = start[3] - start[1];
            const Vector3T<T> w0 = start[0] - start[1];
            const Vector3T<T> du = (end[2] - end[1]) - u0;
            const Vector3T<T> dv = (end[3] - end[1]) - v0;
            const Vector3T<T> dw = (end[0] - end[1]) - w0;

            T coefficient[4];
            coplanarity_coefficients(u0, du, v0, dv, w0, dw, coefficient);
            return coplanarity_roots(coefficient, time);
        }

        /**
         * @brief The candidate times at which two moving edges are coplanar.
         *
         * @tparam T The scalar element type.
         * @param start Positions at the substep's start: the first edge's two
         *              endpoints, then the second edge's two.
         * @param end   The same four points at the substep's end.
         * @param time  Receives up to three candidate times in `[0, 1]`, ascending.
         * @return How many candidates were found.
         */
        template <typename T>
        inline int edge_edge_coplanarity_times(const Vector3T<T> start[4],
                                               const Vector3T<T> end[4], T time[3]) noexcept
        {
            const Vector3T<T> u0 = start[1] - start[0];
            const Vector3T<T> v0 = start[3] - start[2];
            const Vector3T<T> w0 = start[2] - start[0];
            const Vector3T<T> du = (end[1] - end[0]) - u0;
            const Vector3T<T> dv = (end[3] - end[2]) - v0;
            const Vector3T<T> dw = (end[2] - end[0]) - w0;

            T coefficient[4];
            coplanarity_coefficients(u0, du, v0, dv, w0, dw, coefficient);
            return coplanarity_roots(coefficient, time);
        }

        /** @brief The four points interpolated to time @p t across the substep. */
        template <typename T>
        inline void interpolate_positions(const Vector3T<T> start[4], const Vector3T<T> end[4], T t,
                                          Vector3T<T> out[4]) noexcept
        {
            for (int i = 0; i < 4; ++i)
                out[i] = start[i] + (end[i] - start[i]) * t;
        }
    } // namespace Physics
} // namespace SushiEngine
