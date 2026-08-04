/**************************************************************************/
/* test_execution_dynamic_graph.cpp                                       */
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

// P8's "one DynamicGraph region per island" (docs/design/physics_system.md §6.6, §18 R3)
// leans on the execution seam's region wrappers, which the physics nodes themselves do
// not use: they build against Execution::Graph. This is the seam-level proof that
// Execution::DynamicGraph/Execution::Region behave the way RuntimeGraphBuilder's
// per-island wiring will depend on: two regions keyed independently, a drop that takes
// effect immediately for has_region() and leaves the other region's own state
// untouched, and a run that recomposes without the caller driving a separate commit
// step.

#include <cstdint>

#include <gtest/gtest.h>

#include <SushiEngine/execution/context.hpp>

using namespace SushiEngine;

namespace
{
    /**
     * @brief Declares and adds a node that writes @p value across @p count elements
     * starting at @p base of @p buffer.
     *
     * A free function rather than a member of either wrapper: `Execution::Region`
     * and `Execution::Graph` share the same `add_parallel` surface by design (§6.6),
     * so one helper exercises both without knowing which it was handed.
     *
     * @tparam GraphLike Either `Execution::Graph` or `Execution::Region`.
     * @param graph  The node-building surface to record against.
     * @param buffer The buffer the node writes.
     * @param base   First element index the node touches.
     * @param count  How many elements, from @p base.
     * @param value  The value every touched element is set to.
     */
    template <typename GraphLike>
    void emit_fill_node(GraphLike&& graph, Execution::Buffer<std::int32_t>& buffer,
                        std::size_t base, std::size_t count, std::int32_t value)
    {
        std::int32_t* data = buffer.data();
        const Execution::ResourceAccess accesses[] = {
            Execution::ResourceAccess{
                buffer.interval(Execution::ElementRange{base, count}),
                Execution::AccessIntent::ComputeWrite}};

        Execution::NodeDescriptor node;
        node.name = "fill";
        node.accesses = accesses;
        node.access_count = 1;
        node.capacity = count;
        graph.add_parallel(node, [data, base, value](std::size_t i)
                            { data[base + i] = value; });
    }
}

TEST(Integration_ExecutionDynamicGraph, TwoRegionsWriteDisjointSlicesIndependently)
{
    Execution::Runtime runtime = Execution::Runtime::create();
    Execution::Context execution = runtime.context();

    const std::size_t total = 8;
    const std::size_t half = total / 2;
    Execution::Buffer<std::int32_t> values =
        execution.allocate<std::int32_t>(total, Execution::MemoryVisibility::HostShared);
    for (std::size_t i = 0; i < total; ++i)
        values[i] = 0;

    Execution::DynamicGraph graph = execution.create_dynamic_graph();

    constexpr Execution::DynamicGraph::RegionKey region_a = 0;
    constexpr Execution::DynamicGraph::RegionKey region_b = 1;

    emit_fill_node(graph.region(region_a), values, 0, half, 1);
    emit_fill_node(graph.region(region_b), values, half, half, 2);

    ASSERT_TRUE(graph.has_region(region_a));
    ASSERT_TRUE(graph.has_region(region_b));
    EXPECT_EQ(graph.region_count(), 2u);

    const Execution::RunReport first = graph.run();
    ASSERT_TRUE(first.is_successful);

    for (std::size_t i = 0; i < half; ++i)
        EXPECT_EQ(values[i], 1) << "region a's slice at index " << i;
    for (std::size_t i = half; i < total; ++i)
        EXPECT_EQ(values[i], 2) << "region b's slice at index " << i;

    // Dropping a region takes effect on the structure immediately (§6.6's own
    // words), not only after the next run — so a caller deciding whether to
    // re-add a key never has to run first to find out the old one is gone.
    graph.drop(region_a);
    EXPECT_FALSE(graph.has_region(region_a));
    EXPECT_TRUE(graph.has_region(region_b));
    EXPECT_EQ(graph.region_count(), 1u);

    // A fresh region at a different key, replacing the dropped one's slice with
    // a different value, so the assertion below cannot pass by coincidence if
    // the drop silently left the old node live.
    constexpr Execution::DynamicGraph::RegionKey region_c = 2;
    emit_fill_node(graph.region(region_c), values, 0, half, 3);

    const Execution::RunReport second = graph.run();
    ASSERT_TRUE(second.is_successful);

    for (std::size_t i = 0; i < half; ++i)
        EXPECT_EQ(values[i], 3) << "region c replaced region a's slice at index " << i;
    for (std::size_t i = half; i < total; ++i)
        EXPECT_EQ(values[i], 2) << "region b's slice must survive an unrelated region's churn";

    EXPECT_EQ(graph.region_count(), 2u);
}

TEST(Integration_ExecutionDynamicGraph, RunningTwiceWithNoMutationDoesNotRecompose)
{
    // The late-binding promise §6.6 makes for the static Graph applies here too:
    // a run that follows no region()/drop() call since the last one must not
    // recompose, or every performance number this design is held to is void.
    Execution::Runtime runtime = Execution::Runtime::create();
    Execution::Context execution = runtime.context();

    Execution::Buffer<std::int32_t> values =
        execution.allocate<std::int32_t>(4, Execution::MemoryVisibility::HostShared);

    Execution::DynamicGraph graph = execution.create_dynamic_graph();
    emit_fill_node(graph.region(Execution::DynamicGraph::RegionKey{0}), values, 0, 4, 7);

    graph.run();
    const std::size_t after_first = graph.compile_count();
    graph.run();
    graph.run();
    EXPECT_EQ(graph.compile_count(), after_first)
        << "no region changed between these runs; the plan must not recompose";
}
