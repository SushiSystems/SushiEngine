/**************************************************************************/
/* statistics.hpp                                                         */
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
 * @file statistics.hpp
 * @brief What one tick did, measured.
 *
 * The physics contributes nothing to any statistics panel today, and none of the
 * performance targets this system is held to are verifiable without it — which is
 * why this is foundational work rather than something added once the engine is
 * fast (§13.3).
 *
 * Two counters deserve their names read carefully. `compile_count` and
 * `compose_count` come from the runtime and are *cumulative*, not per-tick: after
 * warm-up they must stop climbing. A compose count that advances every tick means
 * the graph is being rebuilt every tick, which makes every other number here
 * meaningless. They are reported for exactly that reason.
 */

#include <cstddef>
#include <cstdint>

#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief Per-stage wall-clock timings for one tick, in milliseconds.
         *
         * Populated from the runtime's per-node timings, which are collected only
         * when the scene was configured with profiling on. With it off these stay
         * zero rather than being estimated, because a made-up timing is worse than
         * an absent one.
         */
        template <typename T>
        struct PhysicsStageTimings
        {
            T broadphase_ms = 0;
            T narrowphase_ms = 0;
            T island_build_ms = 0;
            T solve_ms = 0;      /**< Every substep's projection sweep, summed. */
            T velocity_ms = 0;
            T write_back_ms = 0;
            T total_ms = 0;      /**< The whole tick, including anything not broken out above. */
        };

        /**
         * @brief What one physics tick contained and how long it took.
         *
         * @tparam T The scalar element type used for timings.
         */
        template <typename T>
        struct PhysicsStatisticsT
        {
            /** @brief Bodies that were integrated and projected this tick. */
            std::size_t awake_bodies = 0;

            /** @brief Bodies that exist but were skipped. */
            std::size_t sleeping_bodies = 0;

            /** @brief Islands the awake set partitioned into. */
            std::size_t islands = 0;

            /** @brief Bodies in the largest island — the sequential depth of the tick. */
            std::size_t largest_island = 0;

            /** @brief Candidate pairs the broadphase examined. */
            std::size_t broadphase_pairs_tested = 0;

            /** @brief Candidate pairs the broadphase produced for the narrowphase. */
            std::size_t broadphase_pairs_produced = 0;

            /** @brief Manifolds the narrowphase produced. */
            std::size_t manifolds = 0;

            /** @brief Individual contact points across all manifolds. */
            std::size_t contact_points = 0;

            /** @brief Live constraints of every kind. */
            std::size_t constraints = 0;

            /** @brief Colours the constraint set partitioned into. */
            std::size_t colors = 0;

            /** @brief Constraints in the largest colour — the widest parallel batch. */
            std::size_t largest_color = 0;

            /** @brief Substeps this tick actually ran, within the schedule's bounds. */
            std::size_t substeps = 0;

            /** @brief Bodies escalated to continuous collision this tick. */
            std::size_t continuous_escalations = 0;

            /** @brief Constraints that broke or elements that fractured this tick. */
            std::size_t fracture_events = 0;

            /**
             * @brief Times the solve graph has been compiled since the scene was made.
             *
             * Cumulative. One after warm-up; anything that keeps climbing is a bug.
             */
            std::size_t compile_count = 0;

            /**
             * @brief Times the region graph has been recomposed since the scene was made.
             *
             * Cumulative, and the single most important number here: recomposition
             * is what the whole late-binding design exists to avoid paying per tick.
             */
            std::size_t compose_count = 0;

            /**
             * @brief Times a capacity budget was exceeded since the scene was made.
             *
             * Cumulative, and never zero-and-forgotten: a scene that overflowed once
             * silently dropped work, and the number that overflowed is the number to
             * raise in `PhysicsCapacities`.
             */
            std::size_t capacity_overflows = 0;

            /** @brief Per-stage timings; zero unless the scene was configured with profiling. */
            PhysicsStageTimings<T> timings;
        };

        /** @brief The boundary statistics: `PhysicsStatisticsT` fixed to `Scalar`. */
        using PhysicsStatistics = PhysicsStatisticsT<Scalar>;
    } // namespace Physics
} // namespace SushiEngine
