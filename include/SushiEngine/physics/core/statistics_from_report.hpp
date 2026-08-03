/**************************************************************************/
/* statistics_from_report.hpp                                            */
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
/* permissions and limitations under the License.                        */
/**************************************************************************/

#pragma once

/**
 * @file statistics_from_report.hpp
 * @brief Turns a SushiRuntime run report into `PhysicsStageTimings::node_timings`.
 *
 * The one file under `physics/core` that names `SushiRuntime::Core::RunReport`.
 * §17.5's runtime-instability risk names a standing rule for exactly this
 * situation — an engine-side dependency on an unstable runtime type should have
 * one adapter, not a copy of the coupling at every call site — so this
 * conversion lives here rather than being inlined wherever a caller happens to
 * hold both a `PhysicsStageTimings<T>` and a `RunReport`. A future change to
 * `RunReport`'s shape costs this file plus its test, not every reader of
 * `PhysicsStatistics`.
 */

#include <array>
#include <cstddef>
#include <optional>
#include <string_view>

#include <SushiEngine/physics/core/statistics.hpp>
#include <SushiRuntime/core/run_report.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief The inverse of @ref physics_node_kind_name: which kind, if any, @p name is.
         *
         * A name that matches none of the known kinds is not an error — the most
         * common case is `unnamed_task`, the label a node reports when it was never
         * given one at all — it simply contributes to no kind's total. Every node
         * `RuntimeGraphBuilder::build_graph` emits now names itself (§18 R8), so a
         * miss here against a report taken from that graph is a symptom worth
         * looking into, not a defect this function should paper over by asserting.
         */
        inline std::optional<PhysicsNodeKind> physics_node_kind_from_name(std::string_view name) noexcept
        {
            for (std::size_t index = 0; index < PHYSICS_NODE_KIND_COUNT; ++index)
            {
                const PhysicsNodeKind kind = static_cast<PhysicsNodeKind>(index);
                if (name == physics_node_kind_name(kind))
                    return kind;
            }
            return std::nullopt;
        }

        /**
         * @brief Groups @p report's node timings by name into the fixed per-kind array.
         *
         * Mirrors `soft_body_budget.cpp`'s own grouping exactly, replacing its
         * hand-rolled `std::map<std::string, NamedCost>` with the fixed-capacity
         * array `PhysicsStageTimings::node_timings` holds: the compiled graph emits
         * one node per colour per substep for every persistent constraint kind, all
         * sharing one of @ref PhysicsNodeKind's labels, so "how much of the tick
         * `element_project` spent" is the sum of `device_ms` / `host_ms` /
         * `invocations` across every row with that name — the number §16.37's
         * measurement needed and §18 R8 unblocked.
         *
         * A row whose name is not one of the known kinds (`unnamed_task`, or a
         * future node this list has not caught up with) is silently skipped: a
         * report the caller understands better than this function is not this
         * function's error to raise. Returned in full and zeroed whether or not
         * @p report carries any rows, so a caller with profiling off gets the same
         * all-zero shape @ref PhysicsStageTimings::node_timings starts life as.
         *
         * @tparam T The scalar element type the caller's `PhysicsStageTimings<T>` uses.
         * @param report The backend-native report — `RuntimeGraphBuilder::native_report()`.
         * @return One entry per @ref PhysicsNodeKind, summed across every matching row.
         */
        template <typename T>
        std::array<PhysicsNodeTiming<T>, PHYSICS_NODE_KIND_COUNT>
        physics_node_timings_from_report(const SushiRuntime::Core::RunReport& report) noexcept
        {
            std::array<PhysicsNodeTiming<T>, PHYSICS_NODE_KIND_COUNT> timings{};
            for (const SushiRuntime::Core::NodeTiming& node : report.node_timings)
            {
                const std::optional<PhysicsNodeKind> kind = physics_node_kind_from_name(node.name);
                if (!kind.has_value())
                    continue;

                PhysicsNodeTiming<T>& sum = timings[static_cast<std::size_t>(*kind)];
                sum.device_ms += T(node.device_ms);
                sum.host_ms += T(node.host_ms);
                sum.invocations += node.invocations;
            }
            return timings;
        }
    } // namespace Physics
} // namespace SushiEngine
