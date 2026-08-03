/**************************************************************************/
/* configuration.hpp                                                      */
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
 * @file configuration.hpp
 * @brief The scene's budgets and its substepping schedule.
 *
 * Two kinds of number live here and they are not the same kind of decision.
 *
 * The **capacities** are budgets. A scene allocates its device buffers once at these
 * sizes and never grows them, because an `Execution::Buffer` cannot be resized in
 * place and a growth would invalidate the raw pointer every compiled graph node
 * captured (§6.4). Exceeding a capacity is therefore a reported, recomposing event
 * rather than a routine one, and the number that was exceeded is the number to
 * raise.
 *
 * The **substep schedule** is a quality dial. The count is derived from simulation
 * state each tick (§6.2) and is bounded by the two numbers here; the graph is always
 * built for the maximum, and the surplus substeps are switched off, so changing the
 * count within its bounds costs nothing.
 */

#include <cstddef>

#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief Fixed device-buffer budgets for one scene.
         *
         * Every field is a hard ceiling, not a hint. The defaults are sized for a
         * mid-sized gameplay scene and are meant to be authored per scene rather
         * than tuned globally.
         */
        struct PhysicsCapacities
        {
            /** @brief Maximum simultaneously live rigid bodies. */
            std::size_t bodies = 4096;

            /** @brief Maximum simultaneously live constraints of all kinds. */
            std::size_t constraints = 16384;

            /**
             * @brief Maximum simultaneously live joints.
             *
             * Its own budget rather than a share of @ref constraints because the two
             * are different orders of magnitude and different in kind: a soft-body
             * lattice spends distance constraints by the ten thousand, while joints
             * are authored one at a time and a scene full of vehicles and ragdolls
             * still counts them in hundreds. Sizing one buffer for both would mean a
             * joint descriptor — several times larger than a distance constraint —
             * allocated sixteen thousand times over for a scene that will never hold
             * a hundred of them.
             */
            std::size_t joints = 1024;

            /** @brief Maximum contacts retained in one tick. */
            std::size_t contacts = 16384;

            /**
             * @brief Maximum simultaneously live FEM elements (§9.1's tetrahedra).
             *
             * Its own budget for the same reason @ref joints has one, in the other
             * direction: an element is a four-body constraint carrying a rest-state
             * matrix, a plastic one, two multipliers and a Lamé pair, so it is several
             * times a distance constraint's size and spent by the ten thousand — a
             * default share of @ref constraints would make every scene in the engine
             * carry megabytes for a kind most of them never use.
             *
             * Zero by default, therefore, and opt-in: a scene with soft bodies sets it.
             * That is not a silent trap, because `add_element` on a full budget reports
             * a capacity overflow like every other kind rather than dropping the
             * element quietly.
             *
             * Worth setting deliberately rather than to the element count. Like every
             * banded kind (`constraint_store.hpp`), this budget is divided evenly into
             * one fixed band per @ref colors, so what has to fit is the *busiest*
             * band and not the total; a budget of exactly the element count leaves
             * each colour `1 / colors` of it and rejects a lattice partway in. For
             * tetrahedra the bound is easy to state: a colour class is a set of
             * vertex-disjoint elements, so no band can exceed a quarter of the
             * particles, whatever order they arrive in.
             */
            std::size_t elements = 0;

            /**
             * @brief Maximum simultaneously live beams (§11.1's node-beam links).
             *
             * Zero by default and opt-in, on exactly @ref elements' reasoning: a scene
             * with no vehicles should not carry a band per colour for a kind it never
             * uses. A beam is a two-body constraint like a distance constraint, so the
             * busiest band is bounded the same way — by how many beams meet at one
             * node, which for §11.3's lattice is the node's degree.
             */
            std::size_t beams = 0;

            /**
             * @brief Maximum colours the constraint set may partition into.
             *
             * A ceiling on colours is a ceiling on graph *structure*, which is the
             * thing recomposition is expensive in. Greedy colouring uses at most one
             * more colour than the busiest body's constraint count, so this is
             * really a bound on how many constraints may meet at one body — which
             * is why 32 rather than a smaller round number: a three-dimensional
             * soft-body lattice links each interior vertex to eighteen neighbours,
             * and a ceiling below that would reject an ordinary soft body.
             *
             * The solve graph is built with one node *per kind* per colour per
             * substep, so this multiplies the compiled node count directly and by a
             * factor that grows as kinds are added. Raising it is not free even when
             * the extra colours stay empty.
             *
             * Bounded at 64 by the colour mask the incremental colourer keeps per
             * body; a larger value is rejected rather than silently truncated.
             */
            std::size_t colors = 32;
        };

        /**
         * @brief How a tick is divided into substeps, and the bounds on that division.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        struct SubstepSchedule
        {
            /** @brief Fewest substeps a tick may use, however still the scene is. */
            std::size_t minimum = 4;

            /**
             * @brief Most substeps a tick may use.
             *
             * Also the number of substeps the graph is *built* for. Raising it
             * enlarges the compiled graph even when the extra substeps are never
             * enabled, so it is a structural number, not a runtime one.
             */
            std::size_t maximum = 32;

            /**
             * @brief How far a body may move in one substep, in characteristic sizes.
             *
             * The quantity the derived count targets: a body should not travel more
             * than a fraction of its own size per substep, or a contact generated at
             * the start of the substep no longer describes where it is by the end.
             * The default of a quarter is the usual conservative choice.
             */
            T motion_budget = T(0.25);
        };

        /** @brief The boundary substep schedule: `SubstepSchedule` fixed to `Scalar`. */
        using DefaultSubstepSchedule = SubstepSchedule<Scalar>;

        /**
         * @brief Everything a scene is configured with, in one value.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        struct PhysicsConfigurationT
        {
            /** @brief The device-buffer budgets. */
            PhysicsCapacities capacities;

            /** @brief The substepping schedule. */
            SubstepSchedule<T> substeps;

            /**
             * @brief Which device the scene's allocations are pinned to.
             *
             * One device for the whole scene, because unified shared memory is
             * context-bound: the runtime verifies a node's buffers are co-located on
             * its first dispatch and fails the run if they straddle a boundary.
             * Multi-device physics is a domain-decomposition problem, not a
             * configuration one.
             */
            std::size_t device_index = 0;

            /**
             * @brief Whether the runtime collects per-node timings.
             *
             * Off by default: with profiling disabled the dispatch hot path reads no
             * timestamps at all. Turned on when the profiler panel is open, which is
             * the only time the numbers are looked at.
             */
            bool profiling = false;

            /**
             * @brief Whether the runtime's thermal rebalancer may migrate tasks.
             *
             * Off for physics. It is a background thread on a millisecond heartbeat
             * that moves work mid-run; for a fixed-rate tick the cost is jitter, and
             * jitter is the one thing a fixed-rate tick cannot absorb.
             */
            bool rebalancer = false;

            /** @brief Below this smoothed motion, a body starts its sleep timer. */
            T sleep_motion_threshold = T(0.01);

            /** @brief Seconds below the threshold before a body actually sleeps. */
            T sleep_delay = T(0.5);

            /**
             * @brief Conservative-advancement sweeps (§7.5 tier 2) allowed in one tick.
             *
             * State-derived, not authored per body: any pair thin or fast enough to
             * trip `needs_conservative_advancement` asks for the exact-time-of-impact
             * sweep, and a scene of a thousand such bodies arriving at once should
             * degrade to tier 1's speculative margin rather than spend an unbounded
             * amount of the tick on tier 2 — the same reasoning `FemFractureBudget`
             * states for fracture. A pair that loses the budget this tick is not
             * dropped; it keeps tier 1's manifold, which is safe in the
             * over-generation direction (§1.2) and simply less exact.
             */
            std::size_t continuous_advancement_budget = 256;
        };

        /** @brief The boundary configuration: `PhysicsConfigurationT` fixed to `Scalar`. */
        using PhysicsConfiguration = PhysicsConfigurationT<Scalar>;
    } // namespace Physics
} // namespace SushiEngine
