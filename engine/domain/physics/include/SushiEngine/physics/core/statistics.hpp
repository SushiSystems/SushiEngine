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

#include <array>
#include <cstddef>
#include <cstdint>

#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief The named graph-node kinds `RuntimeGraphBuilder::build_graph` emits.
         *
         * One entry per distinct node label the solve graph names (§18 R8) — not per
         * colour or substep instance. The compiled graph holds one node per colour
         * per substep for every persistent constraint kind, and all of those share
         * one of these names; attributing cost to "how much of the tick
         * `element_project` spent" means summing every instance with this label
         * together, which is exactly what @ref PhysicsNodeTiming holds one row of.
         *
         * @ref Count is not a kind. It is the array bound every fixed-capacity
         * breakdown in this file is sized to, following the same never-grow
         * discipline `PhysicsCapacities` uses for its device buffers: the kind set
         * is fixed at compile time, so the breakdown is a `std::array`, not a
         * `std::map` a hot path would have to allocate into.
         */
        enum class PhysicsNodeKind : std::size_t
        {
            Predict = 0,
            DistanceProject,
            BeamProject,
            ElementProject,
            JointProject,
            ContactPrepare,
            ContactPosition,
            UpdateVelocity,
            BeamVelocity,
            JointVelocity,
            ContactVelocity,
            MotionMeasure,
            Count
        };

        /** @brief How many named node kinds exist; the bound every fixed array below uses. */
        constexpr std::size_t PHYSICS_NODE_KIND_COUNT = static_cast<std::size_t>(PhysicsNodeKind::Count);

        /**
         * @brief The exact node label `RuntimeGraphBuilder::build_graph` emits for @p kind.
         *
         * The single definition the conversion in `statistics_from_report.hpp`
         * depends on: `runtime_graph_builder.hpp`'s `emit_node` calls are the source
         * of truth for each string, and this function is the one place that copy of
         * the string lives outside that file. A `switch` over the enum rather than a
         * table, so a kind added to @ref PhysicsNodeKind without a case here is a
         * compiler warning, not a silent gap.
         */
        constexpr const char* physics_node_kind_name(PhysicsNodeKind kind) noexcept
        {
            switch (kind)
            {
                case PhysicsNodeKind::Predict:
                    return "predict";
                case PhysicsNodeKind::DistanceProject:
                    return "distance_project";
                case PhysicsNodeKind::BeamProject:
                    return "beam_project";
                case PhysicsNodeKind::ElementProject:
                    return "element_project";
                case PhysicsNodeKind::JointProject:
                    return "joint_project";
                case PhysicsNodeKind::ContactPrepare:
                    return "contact_prepare";
                case PhysicsNodeKind::ContactPosition:
                    return "contact_position";
                case PhysicsNodeKind::UpdateVelocity:
                    return "update_velocity";
                case PhysicsNodeKind::BeamVelocity:
                    return "beam_velocity";
                case PhysicsNodeKind::JointVelocity:
                    return "joint_velocity";
                case PhysicsNodeKind::ContactVelocity:
                    return "contact_velocity";
                case PhysicsNodeKind::MotionMeasure:
                    return "motion_measure";
                case PhysicsNodeKind::Count:
                    break;
            }
            return "";
        }

        /**
         * @brief One named node kind's summed device/host cost for one tick, in milliseconds.
         *
         * Summed across every colour and substep instance sharing @ref
         * physics_node_kind_name's label for this kind, the same grouping
         * `soft_body_budget.cpp` already does by hand with a `std::map`. Zero unless
         * `PhysicsConfiguration::profiling` is on, for the same reason every timing
         * in this file is: an unmeasured field must read as absent, not as a
         * plausible-looking zero.
         */
        template <typename T>
        struct PhysicsNodeTiming
        {
            /** @brief Summed kernel device time (from SYCL event profiling). */
            T device_ms = 0;

            /** @brief Summed wall-clock for native/host work run on a worker. */
            T host_ms = 0;

            /** @brief Times a node with this label was dispatched this tick. */
            std::size_t invocations = 0;
        };

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
         * `solve_ms` itself stays one number — it is one `run()`, and the one case
         * that is not the same device solve at all, the soft-body host schedule, is
         * already broken out below as @ref soft_body_ms rather than folded in here.
         * What §18 R8 closed is the breakdown *inside* `solve_ms`, once it needed
         * one: the runtime's ordinary `add()` overloads used to drop every node's label, so a node
         * arrived as `unnamed_task` and the only way to attribute one was its plan
         * index — a compile-time internal, and exactly the kind of engine-side claim
         * about a runtime detail §18 records the cost of making. `add_parallel` /
         * `add_host` (`runtime_backend.hpp`) now forward `NodeDescriptor::name`
         * through, so `predict`, every projection kind, the two velocity passes and
         * `motion_measure` each report under their real name, and @ref node_timings
         * below is that attribution, folded by name.
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
             * derivation, the velocity pass and the motion reduction — all of it, at
             * the granularity of one wall-clock number for the whole `run()`. See
             * this struct's note and @ref node_timings for how much of it went to
             * which named kind.
             */
            T solve_ms = 0;

            /**
             * @brief @ref solve_ms, broken down by @ref PhysicsNodeKind.
             *
             * Populated by `physics_node_timings_from_report`
             * (`core/statistics_from_report.hpp`) from the same
             * `SushiRuntime::Core::RunReport` `solve_ms` is read from. Every entry
             * stays zero unless profiling is on, same as every other field here; a
             * scene with no joints or contacts correctly reports zero device time
             * for `joint_project`, `contact_prepare` and the rest, because those
             * nodes dispatch against zero live elements and do nothing — that is a
             * true measurement, not a missing one.
             */
            std::array<PhysicsNodeTiming<T>, PHYSICS_NODE_KIND_COUNT> node_timings{};

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

            /**
             * @brief Pairs that asked for continuous collision but lost the budget.
             *
             * A skipped pair is not dropped — it keeps tier 1's speculative
             * manifold (§7.5) rather than tier 2's exact sweep, which is safe in
             * the over-generation direction and simply less exact. Non-zero is not
             * an error, the same reading `FEMFractureReport::elements_skipped`
             * gets; it is how a caller measures whether
             * `PhysicsConfiguration::continuous_advancement_budget` actually bound
             * this tick.
             */
            std::size_t continuous_advancement_skipped = 0;

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
