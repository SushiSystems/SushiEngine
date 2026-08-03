/**************************************************************************/
/* test_physics_statistics.cpp                                           */
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

// §18 R8, consumed on the statistics side: `physics_node_timings_from_report`
// (core/statistics_from_report.hpp) groups a SushiRuntime run report's per-node
// rows by name into `PhysicsStageTimings::node_timings`, the same grouping
// `soft_body_budget.cpp` already does by hand with a `std::map`. A live report
// needs a runtime and a device to produce -- this test builds a synthetic one
// instead, so the conversion is checked against known-correct sums without
// either.

#include <cstddef>
#include <optional>

#include <gtest/gtest.h>

#include <SushiEngine/physics/core/statistics_from_report.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    /** @brief One `NodeTiming` row, as the compiled graph would name it. */
    SushiRuntime::Core::NodeTiming row(const char* name, double device_ms, double host_ms,
                                      std::size_t invocations)
    {
        SushiRuntime::Core::NodeTiming timing;
        timing.name = name;
        timing.device_ms = device_ms;
        timing.host_ms = host_ms;
        timing.invocations = invocations;
        return timing;
    }
}

TEST(Unit_PhysicsStatistics, AnEmptyReportProducesAllZeroTimings)
{
    const SushiRuntime::Core::RunReport report;
    const auto timings = physics_node_timings_from_report<double>(report);

    for (std::size_t index = 0; index < PHYSICS_NODE_KIND_COUNT; ++index)
    {
        EXPECT_DOUBLE_EQ(timings[index].device_ms, 0.0);
        EXPECT_DOUBLE_EQ(timings[index].host_ms, 0.0);
        EXPECT_EQ(timings[index].invocations, 0u);
    }
}

TEST(Unit_PhysicsStatistics, InstancesSharingANameAreSummedIntoOneKind)
{
    // Mirrors the shape a real tick produces: the compiled graph holds one node
    // per colour per substep, so `element_project` arrives as several rows all
    // sharing that name, which is exactly what a colour/substep unroll of two
    // colours across two substeps looks like once folded into one kind.
    SushiRuntime::Core::RunReport report;
    report.node_timings.push_back(row("element_project", 1.0, 0.0, 4));
    report.node_timings.push_back(row("element_project", 2.5, 0.0, 4));
    report.node_timings.push_back(row("element_project", 0.5, 0.0, 4));

    const auto timings = physics_node_timings_from_report<double>(report);
    const PhysicsNodeTiming<double>& element =
        timings[static_cast<std::size_t>(PhysicsNodeKind::ElementProject)];

    EXPECT_DOUBLE_EQ(element.device_ms, 4.0);
    EXPECT_DOUBLE_EQ(element.host_ms, 0.0);
    EXPECT_EQ(element.invocations, 12u);
}

TEST(Unit_PhysicsStatistics, DifferentNamesLandInDifferentKindsAndDoNotBleedTogether)
{
    SushiRuntime::Core::RunReport report;
    report.node_timings.push_back(row("predict", 0.6, 0.0, 32));
    report.node_timings.push_back(row("update_velocity", 0.4, 0.0, 32));
    report.node_timings.push_back(row("motion_measure", 0.1, 0.0, 32));

    const auto timings = physics_node_timings_from_report<double>(report);

    const auto& predict = timings[static_cast<std::size_t>(PhysicsNodeKind::Predict)];
    const auto& update_velocity =
        timings[static_cast<std::size_t>(PhysicsNodeKind::UpdateVelocity)];
    const auto& motion_measure =
        timings[static_cast<std::size_t>(PhysicsNodeKind::MotionMeasure)];

    EXPECT_DOUBLE_EQ(predict.device_ms, 0.6);
    EXPECT_EQ(predict.invocations, 32u);
    EXPECT_DOUBLE_EQ(update_velocity.device_ms, 0.4);
    EXPECT_EQ(update_velocity.invocations, 32u);
    EXPECT_DOUBLE_EQ(motion_measure.device_ms, 0.1);
    EXPECT_EQ(motion_measure.invocations, 32u);

    // A kind that never dispatched this tick -- correctly zero, not a gap. This
    // scene's report named no contact or joint rows at all, exactly like §16.37's
    // measured scene, which has neither.
    const auto& contact_prepare =
        timings[static_cast<std::size_t>(PhysicsNodeKind::ContactPrepare)];
    EXPECT_DOUBLE_EQ(contact_prepare.device_ms, 0.0);
    EXPECT_EQ(contact_prepare.invocations, 0u);
}

TEST(Unit_PhysicsStatistics, AnUnrecognisedNameIsSkippedRatherThanMisattributed)
{
    // `unnamed_task` is the label a node reports when it was never given one --
    // the exact failure §18 R8 closed. A row like this should contribute to no
    // kind's total rather than being folded into whichever kind happens to be
    // first, which would silently overstate that kind's cost.
    SushiRuntime::Core::RunReport report;
    report.node_timings.push_back(row("unnamed_task", 99.0, 0.0, 1));
    report.node_timings.push_back(row("predict", 0.2, 0.0, 1));

    const auto timings = physics_node_timings_from_report<double>(report);
    std::size_t total_invocations = 0;
    double total_device_ms = 0.0;
    for (const auto& timing : timings)
    {
        total_invocations += timing.invocations;
        total_device_ms += timing.device_ms;
    }

    EXPECT_EQ(total_invocations, 1u);
    EXPECT_DOUBLE_EQ(total_device_ms, 0.2);
}

TEST(Unit_PhysicsStatistics, EveryNodeKindNamesTheStringRuntimeGraphBuilderEmits)
{
    // Pinned so a rename in `runtime_graph_builder.hpp`'s `emit_node` calls is
    // caught here rather than silently stopping this conversion from ever
    // matching a real report again.
    EXPECT_STREQ(physics_node_kind_name(PhysicsNodeKind::Predict), "predict");
    EXPECT_STREQ(physics_node_kind_name(PhysicsNodeKind::DistanceProject), "distance_project");
    EXPECT_STREQ(physics_node_kind_name(PhysicsNodeKind::BeamProject), "beam_project");
    EXPECT_STREQ(physics_node_kind_name(PhysicsNodeKind::ElementProject), "element_project");
    EXPECT_STREQ(physics_node_kind_name(PhysicsNodeKind::JointProject), "joint_project");
    EXPECT_STREQ(physics_node_kind_name(PhysicsNodeKind::ContactPrepare), "contact_prepare");
    EXPECT_STREQ(physics_node_kind_name(PhysicsNodeKind::ContactPosition), "contact_position");
    EXPECT_STREQ(physics_node_kind_name(PhysicsNodeKind::UpdateVelocity), "update_velocity");
    EXPECT_STREQ(physics_node_kind_name(PhysicsNodeKind::BeamVelocity), "beam_velocity");
    EXPECT_STREQ(physics_node_kind_name(PhysicsNodeKind::JointVelocity), "joint_velocity");
    EXPECT_STREQ(physics_node_kind_name(PhysicsNodeKind::ContactVelocity), "contact_velocity");
    EXPECT_STREQ(physics_node_kind_name(PhysicsNodeKind::MotionMeasure), "motion_measure");
}

TEST(Unit_PhysicsStatistics, AnUnknownNameHasNoKind)
{
    EXPECT_FALSE(physics_node_kind_from_name("unnamed_task").has_value());
    EXPECT_FALSE(physics_node_kind_from_name("").has_value());
}

TEST(Unit_PhysicsStatistics, EveryKnownNameRoundTripsThroughItsKind)
{
    for (std::size_t index = 0; index < PHYSICS_NODE_KIND_COUNT; ++index)
    {
        const PhysicsNodeKind kind = static_cast<PhysicsNodeKind>(index);
        const std::optional<PhysicsNodeKind> found =
            physics_node_kind_from_name(physics_node_kind_name(kind));
        ASSERT_TRUE(found.has_value());
        EXPECT_EQ(*found, kind);
    }
}
