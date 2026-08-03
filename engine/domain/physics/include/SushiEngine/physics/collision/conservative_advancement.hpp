/**************************************************************************/
/* conservative_advancement.hpp                                          */
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
 * @file conservative_advancement.hpp
 * @brief §7.5 tier 2: an accurate time of impact for the motion tier 1 cannot bound.
 *
 * Tier 1 (`sim/physics_simulation.hpp`'s speculative contacts) widens a pair's
 * manifold-generation offset by how far it can *translate* this tick and lets
 * the ordinary substep loop catch the crossing. That is correct and free for the
 * overwhelming majority of fast motion, but it asks the narrowphase a single
 * question at the tick's start pose — so it has no answer for a body whose
 * *rotation* sweeps a feature past a thin obstacle without much translation at
 * all (a blade, a spinning wheel's rim), and no guaranteed answer when the
 * closest feature at the start pose is not the one actually struck.
 *
 * Conservative advancement is Redon et al.'s method: repeatedly ask the exact
 * GJK/EPA distance between the two shapes at an advanced pose, and bound how
 * fast that distance can possibly shrink before the next query — from the
 * closing speed along the *current* normal, plus a conservative allowance for
 * whatever either body's own rotation could add regardless of which way it
 * happens to be turning. Because the bound can only ever overestimate the true
 * rate, the advance can never step past the real time of impact; it either
 * lands on it (within tolerance) or proves there is none inside the interval.
 *
 * This file is deliberately narrow. It answers one question — "do these two
 * convex shapes touch somewhere in `[0, dt]`, and where" — as a pure function of
 * two shapes and their velocities. `sim/physics_simulation.hpp` decides *which*
 * pairs are worth asking (§7.5's own trigger: per-substep motion exceeding a
 * fraction of the shape's thinnest dimension) and what to do with the answer.
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/collision/narrowphase_dispatch.hpp>
#include <SushiEngine/physics/core/rigid_body.hpp>
#include <SushiEngine/physics/geometry/gjk.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief An upper bound on how far a point on the shape's surface sits from
         *        its centre — the lever a rotation swings a surface point through.
         *
         * Deliberately a bound, not the exact support radius: it feeds the
         * conservative-advancement closing-speed estimate, where an overestimate
         * only makes the advance more cautious and an underestimate would let it
         * step past a real impact.
         */
        template <typename T>
        inline T collision_shape_bounding_radius(const CollisionShape<T>& shape) noexcept
        {
            switch (shape.type)
            {
                case ShapeType::sphere:
                    return shape.radius;
                case ShapeType::box:
                case ShapeType::oriented_box:
                    return length(shape.half_extents);
                case ShapeType::capsule:
                    return shape.half_height + shape.radius;
                case ShapeType::convex_hull:
                {
                    T best = shape.convex_radius;
                    for (std::uint32_t i = 0; i < shape.vertex_count; ++i)
                        best = std::max(best, length(shape.vertices[i]) + shape.convex_radius);
                    return best;
                }
                default:
                    return T(0);
            }
        }

        /**
         * @brief The smallest dimension a fast-enough body could pass through unresolved.
         *
         * The §7.5 trigger reads this: a body whose motion this tick approaches or
         * exceeds a fraction of its own thinnest dimension is the one tier 1's
         * per-tick, start-pose manifold cannot be trusted to have bounded correctly,
         * because the feature it saw at the start may not be the one actually hit.
         */
        template <typename T>
        inline T collision_shape_thinnest_extent(const CollisionShape<T>& shape) noexcept
        {
            switch (shape.type)
            {
                case ShapeType::sphere:
                    return T(2) * shape.radius;
                case ShapeType::box:
                case ShapeType::oriented_box:
                {
                    T smallest = shape.half_extents.x;
                    smallest = std::min(smallest, shape.half_extents.y);
                    smallest = std::min(smallest, shape.half_extents.z);
                    return T(2) * smallest;
                }
                case ShapeType::capsule:
                    return T(2) * shape.radius;
                case ShapeType::convex_hull:
                {
                    if (shape.vertices == nullptr || shape.vertex_count == 0)
                        return T(2) * shape.convex_radius;
                    Vector3T<T> low = shape.vertices[0];
                    Vector3T<T> high = low;
                    for (std::uint32_t i = 1; i < shape.vertex_count; ++i)
                    {
                        const Vector3T<T>& v = shape.vertices[i];
                        low = Vector3T<T>{std::min(low.x, v.x), std::min(low.y, v.y),
                                          std::min(low.z, v.z)};
                        high = Vector3T<T>{std::max(high.x, v.x), std::max(high.y, v.y),
                                           std::max(high.z, v.z)};
                    }
                    const Vector3T<T> extent = high - low;
                    const T smallest = std::min(extent.x, std::min(extent.y, extent.z));
                    return smallest + T(2) * shape.convex_radius;
                }
                default:
                    // A half-space plane has no thinnest dimension to tunnel through —
                    // its own analytic separation formula is exact at any distance, so
                    // it never needs this tier (§7.5's ground-plane case is tier 1's).
                    return std::numeric_limits<T>::max();
            }
        }

        /** @brief The signature a distance-dispatch entry has: exact GJK/EPA, no clipping. */
        template <typename T>
        using DistanceFunction = ConvexContact<T> (*)(const CollisionShape<T>&,
                                                       const CollisionShape<T>&);

        /** @brief Table entry for a convex pair: recover both types, run GJK/EPA directly. */
        template <typename T, typename ShapeA, typename ShapeB>
        inline ConvexContact<T> convex_distance_entry(const CollisionShape<T>& a,
                                                      const CollisionShape<T>& b) noexcept
        {
            return collide_convex<T>(ShapeTraits<T, ShapeA>::from(a), ShapeTraits<T, ShapeB>::from(b));
        }

        /**
         * @brief The distance-query table: one exact GJK/EPA entry per ordered convex pair.
         *
         * The same generation the narrowphase manifold table uses (§4.2), over the
         * same `ConvexShapes` list, so a shape that gains a `support()` overload
         * gains a conservative-advancement entry for free — no plane entries,
         * because a half-space is not a bounded convex set GJK can be asked about,
         * and it does not need this tier (see @ref collision_shape_thinnest_extent).
         */
        template <typename T>
        struct DistanceTable
        {
            static constexpr std::size_t kind_count = static_cast<std::size_t>(ShapeType::count);
            DistanceFunction<T> entries[kind_count][kind_count] = {};

            void set(ShapeType a, ShapeType b, DistanceFunction<T> fn) noexcept
            {
                entries[static_cast<std::size_t>(a)][static_cast<std::size_t>(b)] = fn;
            }

            DistanceFunction<T> get(ShapeType a, ShapeType b) const noexcept
            {
                return entries[static_cast<std::size_t>(a)][static_cast<std::size_t>(b)];
            }
        };

        /** @brief Fills one row of the convex block: @p ShapeA against every convex shape. */
        template <typename T, typename ShapeA, typename... Shapes>
        inline void register_distance_row(DistanceTable<T>& table, TypeList<Shapes...>) noexcept
        {
            (table.set(ShapeTraits<T, ShapeA>::type, ShapeTraits<T, Shapes>::type,
                       &convex_distance_entry<T, ShapeA, Shapes>),
             ...);
        }

        /** @brief Fills the whole convex block. */
        template <typename T, typename... Shapes>
        inline void register_distance_shapes(DistanceTable<T>& table, TypeList<Shapes...> list) noexcept
        {
            (register_distance_row<T, Shapes>(table, list), ...);
        }

        /** @brief The one table, built on first use. */
        template <typename T>
        inline const DistanceTable<T>& distance_table() noexcept
        {
            static const DistanceTable<T> table = []()
            {
                DistanceTable<T> built;
                register_distance_shapes<T>(built, ConvexShapes<T>{});
                return built;
            }();
            return table;
        }

        /**
         * @brief The exact separation between two type-erased convex shapes, whatever they are.
         *
         * An invalid contact (never touched, not registered — a plane on either
         * side) reports `valid == false` rather than a guessed distance.
         */
        template <typename T>
        inline ConvexContact<T> convex_shape_distance(const CollisionShape<T>& a,
                                                       const CollisionShape<T>& b) noexcept
        {
            const DistanceFunction<T> entry = distance_table<T>().get(a.type, b.type);
            if (entry == nullptr)
                return ConvexContact<T>{};
            return entry(a, b);
        }

        /**
         * @brief A type-erased shape, translated and rotated by @p t of its velocity.
         *
         * Rotating even a shape whose `support()` ignores orientation (a sphere) is
         * harmless — the field is simply unread — so one function serves every
         * shape kind uniformly, the same way `CollisionShape` itself does.
         */
        template <typename T>
        inline CollisionShape<T> advance_collision_shape(const CollisionShape<T>& shape,
                                                          const Vector3T<T>& linear_velocity,
                                                          const Vector3T<T>& angular_velocity,
                                                          T t) noexcept
        {
            CollisionShape<T> moved = shape;
            moved.center = shape.center + linear_velocity * t;
            moved.orientation = apply_angular_correction(shape.orientation, angular_velocity * t);
            return moved;
        }

        /** @brief What a conservative-advancement query answers with. */
        template <typename T>
        struct ConservativeAdvancementResult
        {
            bool impact = false;
            /** @brief Meaningful only when @ref impact is true. */
            T time_of_impact = 0;
            /** @brief The witness pair and normal at @ref time_of_impact's advanced pose. */
            ConvexContact<T> contact;
        };

        /**
         * @brief Finds the earliest time in `[0, dt]` two moving convex shapes reach
         *        @p target_separation, or proves there is none.
         *
         * @param shape_a           The first shape, at the tick's start pose.
         * @param linear_velocity_a Its linear velocity.
         * @param angular_velocity_a Its angular velocity (world-frame).
         * @param shape_b           The second shape, at the tick's start pose.
         * @param linear_velocity_b Its linear velocity.
         * @param angular_velocity_b Its angular velocity (world-frame).
         * @param target_separation The separation that counts as an impact — the
         *                          same `contact_offset` tier 1 generates contacts to.
         * @param dt                How far ahead to look, in seconds.
         * @param tolerance         Convergence threshold, in metres, and the floor
         *                          on each step so a near-zero closing speed cannot
         *                          stall the loop.
         * @param max_iterations    The iteration budget; conservative advancement
         *                          converges in a handful for well-conditioned
         *                          geometry, so this is a generous ceiling rather
         *                          than a tuned figure.
         * @return `impact == false` both when the shapes never approach that close
         *         within `dt` and when the budget is spent without converging — the
         *         two are treated alike deliberately, because a caller must never
         *         read a non-convergent search as a verified miss elsewhere and an
         *         unverified one here.
         */
        template <typename T>
        inline ConservativeAdvancementResult<T> conservative_advance(
            const CollisionShape<T>& shape_a, const Vector3T<T>& linear_velocity_a,
            const Vector3T<T>& angular_velocity_a, const CollisionShape<T>& shape_b,
            const Vector3T<T>& linear_velocity_b, const Vector3T<T>& angular_velocity_b,
            T target_separation, T dt, T tolerance = T(1e-4), int max_iterations = 32) noexcept
        {
            ConservativeAdvancementResult<T> result;
            if (dt <= T(0))
                return result;

            // A bound on how much either body's own rotation could add to the
            // closing speed, regardless of which way it happens to be turning —
            // added unconditionally rather than signed, which is what keeps the
            // bound conservative without having to know the true contribution.
            const T angular_bound =
                length(angular_velocity_a) * collision_shape_bounding_radius(shape_a) +
                length(angular_velocity_b) * collision_shape_bounding_radius(shape_b);
            const Vector3T<T> relative_velocity = linear_velocity_b - linear_velocity_a;

            T t = T(0);
            for (int iteration = 0; iteration < max_iterations; ++iteration)
            {
                const CollisionShape<T> moved_a =
                    advance_collision_shape(shape_a, linear_velocity_a, angular_velocity_a, t);
                const CollisionShape<T> moved_b =
                    advance_collision_shape(shape_b, linear_velocity_b, angular_velocity_b, t);
                const ConvexContact<T> contact = convex_shape_distance<T>(moved_a, moved_b);
                if (!contact.valid)
                    return result; // unregistered pair (a plane): this tier does not apply

                if (contact.separation <= target_separation)
                {
                    result.impact = true;
                    result.time_of_impact = t;
                    result.contact = contact;
                    return result;
                }

                // A lower bound on the rate the separation can shrink: the linear
                // closing speed along the current normal, plus the rotation
                // allowance. Never negative by construction elsewhere in the
                // engine's normal convention (normal runs a toward b, so a positive
                // component of `relative_velocity` along it is separating).
                const T closing_speed = -dot(relative_velocity, contact.normal) + angular_bound;
                if (closing_speed <= T(0))
                    return result; // not approaching within the bound: no impact this interval

                const T step = (contact.separation - target_separation) / closing_speed;
                t += step > tolerance ? step : tolerance;
                if (t >= dt)
                    return result; // exhausted the interval without reaching the target
            }
            return result;
        }

        /**
         * @brief Whether a shape's motion this tick is large enough to need tier 2.
         *
         * §7.5's own trigger: translation plus the swept arc of rotation, compared
         * against a fraction of the shape's own thinnest dimension. Both terms are
         * simulation state — velocity, angular velocity, the cooked or authored
         * shape — so the decision is deterministic per §0.5, and a state-derived
         * quantity rather than a wall-clock or frame-rate one.
         *
         * @param motion_fraction How much of the thinnest dimension one tick's
         *                        motion may cover before tier 2 engages. One half is
         *                        the conservative middle ground: smaller catches
         *                        more pairs tier 1 already handles for free, larger
         *                        risks a feature change tier 1 cannot see.
         */
        template <typename T>
        inline bool needs_conservative_advancement(const CollisionShape<T>& shape,
                                                    const Vector3T<T>& linear_velocity,
                                                    const Vector3T<T>& angular_velocity, T dt,
                                                    T motion_fraction = T(0.5)) noexcept
        {
            const T thinnest = collision_shape_thinnest_extent(shape);
            if (!(thinnest > T(0)) || thinnest == std::numeric_limits<T>::max())
                return false;
            const T linear_travel = length(linear_velocity) * dt;
            const T angular_travel =
                length(angular_velocity) * collision_shape_bounding_radius(shape) * dt;
            return (linear_travel + angular_travel) > motion_fraction * thinnest;
        }
    } // namespace Physics
} // namespace SushiEngine
