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
         * Populated only when `PhysicsConfiguration::profiling` is on. With it off
         * these stay zero rather than being estimated, because a made-up timing is
         * worse than an absent one — and a field that is *structurally* always zero
         * is the same failure wearing the opposite mask, which is why there is one
         * field per stage that can actually be measured and no more.
         *
         * The host stages are measured with a host clock, which is honest because
         * they genuinely run on the host (§16.6): broadphase, narrowphase, the
         * island partition, and the transfers back. `solve_ms` is the device
         * composition's own measured cost, from the run report.
         *
         * There is deliberately no per-stage breakdown *inside* `solve_ms`. The
         * runtime reports device time per node and names each one, but its public
         * `add()` surface carries no label, so every physics node arrives as
         * `unnamed_task` and the only way to attribute one is its plan index — a
         * compile-time internal, and exactly the kind of engine-side claim about a
         * runtime detail that §18 records the cost of making. Splitting predict from
         * the projection sweeps from the velocity derivation is a runtime ask, not
         * something to guess at here.
         */
        template <typename T>
        struct PhysicsStageTimings
        {
            T broadphase_ms = 0;
            T narrowphase_ms = 0;
            T island_build_ms = 0;
            /**
             * @brief The whole device composition: one `run()`, every substep.
             *
             * Predict, every constraint kind's projection sweep, the velocity
             * derivation, the velocity pass and the motion reduction — all of it,
             * because that is the granularity the runtime's report gives without
             * node labels. See this struct's note.
             */
            T solve_ms = 0;
            T write_back_ms = 0;

            /**
             * @brief The soft-body world's whole tick: every body, every substep.
             *
             * Its own line rather than folded into @ref solve_ms, because it is not the
             * same solve. Soft bodies run a host XPBD schedule outside the rigid
             * composition — a tick can be entirely this and nothing else — so charging it
             * to the device solve would report device time for work no device did.
             */
            T soft_body_ms = 0;

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

            /**
             * @brief Live joints, of the @ref constraints above.
             *
             * Broken out rather than merely included because a joint costs a great
             * deal more than a distance constraint — several constraint rows, a
             * swing/twist decomposition, and a force readback off the device — so a
             * tick with four hundred constraints of which four hundred are joints is
             * a different tick from one with four hundred distance links.
             */
            std::size_t joints = 0;

            /**
             * @brief Live FEM elements, of the @ref constraints above.
             *
             * Broken out for the same reason @ref joints is, from the other end of the
             * range: an element is four-body and carries two projections, so a tick
             * whose constraint count is mostly elements is a different tick from one of
             * the same count in distance links, and a soft body's cost is invisible in
             * the total.
             */
            std::size_t elements = 0;

            /**
             * @brief Live beams, of the @ref constraints above.
             *
             * Broken out because a vehicle is the one thing that spends this kind, and
             * a reader looking at a tick that got slower wants to know whether a car
             * arrived. A beam costs about what a distance constraint costs, so the
             * number is a population count rather than a warning.
             */
            std::size_t beams = 0;

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
