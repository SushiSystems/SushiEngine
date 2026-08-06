/**************************************************************************/
/* test_model_instantiation_plan.cpp                                      */
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

// model_import.md §5's ten rules, which is every decision the import makes. The planner reads
// no file and touches no device, so every one of them is checked against a description built
// here by hand — which is the whole reason the decision was separated from the entity creation
// it drives.

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>

#include <SushiEngine/model/instantiation_plan.hpp>

using SushiEngine::Geometry::GLTFCameraDescription;
using SushiEngine::Geometry::GLTFLightDescription;
using SushiEngine::Geometry::GLTFLightKind;
using SushiEngine::Geometry::GLTFNodeDescription;
using SushiEngine::Geometry::GLTFSceneDescription;
using SushiEngine::Model::ModelImportReport;
using SushiEngine::Model::ModelImportSettings;
using SushiEngine::Model::ModelInstantiationPlan;
using SushiEngine::Model::PlannedComponent;
using SushiEngine::Model::plan_model_instantiation;
using SushiEngine::Model::Quaternionf;
using SushiEngine::Model::Vector3f;

namespace
{
    GLTFNodeDescription node(const char* name, std::int32_t parent, std::uint32_t source)
    {
        GLTFNodeDescription n;
        n.name = name;
        n.parent = parent;
        n.source_index = source;
        return n;
    }

    // The composed world position of a planned entity, walking its parent chain. Rotation is
    // left out deliberately: every fixture that uses this helper has identity rotations, and a
    // test helper that reimplements the engine's transform composition would be testing itself.
    // The one fixture with a rotated pivot states its expected numbers analytically instead.
    Vector3f world_position(const ModelInstantiationPlan& plan, std::size_t index)
    {
        // A parent contributes `translation + scale * (whatever its child composed)`, so the
        // walk builds the position from the entity outwards rather than accumulating a scale.
        Vector3f position{0.0f, 0.0f, 0.0f};
        for (std::int32_t i = static_cast<std::int32_t>(index); i >= 0;
             i = plan.entities[static_cast<std::size_t>(i)].parent)
        {
            const auto& entity = plan.entities[static_cast<std::size_t>(i)];
            position.x = entity.translation.x + entity.scale.x * position.x;
            position.y = entity.translation.y + entity.scale.y * position.y;
            position.z = entity.translation.z + entity.scale.z * position.z;
        }
        return position;
    }
}

TEST(Unit_ModelInstantiationPlan, AnEmptyDescriptionPlansNothing)
{
    GLTFSceneDescription description;
    ModelImportReport report;
    const auto plan = plan_model_instantiation(description, ModelImportSettings{}, "Empty", report);
    EXPECT_TRUE(plan.entities.empty());
    EXPECT_EQ(report.entities, 0u);
}

TEST(Unit_ModelInstantiationPlan, ANodeWithOneMeshPrimitiveCarriesItsOwnShape)
{
    GLTFSceneDescription description;
    GLTFNodeDescription tire = node("Tire", -1, 0);
    tire.mesh = 0;
    tire.primitive_count = 1;
    description.nodes.push_back(tire);

    ModelImportReport report;
    const auto plan = plan_model_instantiation(description, ModelImportSettings{}, "Wheel", report);
    ASSERT_EQ(plan.entities.size(), 1u);
    EXPECT_EQ(plan.entities[0].name, "Tire");
    EXPECT_EQ(plan.entities[0].component, PlannedComponent::Shape);
    EXPECT_EQ(plan.entities[0].source_node, 0u);
    EXPECT_EQ(plan.entities[0].primitive, 0u);
    EXPECT_EQ(report.primitives_imported, 1u);
}

TEST(Unit_ModelInstantiationPlan, ANodeWithTwoPrimitivesStaysATransformAndGainsAChildPerPrimitive)
{
    GLTFSceneDescription description;
    GLTFNodeDescription body = node("Body", -1, 0);
    body.mesh = 0;
    body.primitive_count = 2;
    description.nodes.push_back(body);

    ModelImportReport report;
    const auto plan = plan_model_instantiation(description, ModelImportSettings{}, "Car", report);
    ASSERT_EQ(plan.entities.size(), 3u);
    EXPECT_EQ(plan.entities[0].name, "Body");
    EXPECT_EQ(plan.entities[0].component, PlannedComponent::None);
    EXPECT_EQ(plan.entities[1].name, "Body (0)");
    EXPECT_EQ(plan.entities[1].component, PlannedComponent::Shape);
    EXPECT_EQ(plan.entities[1].parent, 0);
    EXPECT_EQ(plan.entities[1].primitive, 0u);
    EXPECT_EQ(plan.entities[2].name, "Body (1)");
    EXPECT_EQ(plan.entities[2].primitive, 1u);
    // The split children sit exactly on their node: the visible pivot stays the node's own.
    EXPECT_FLOAT_EQ(plan.entities[1].translation.x, 0.0f);
    EXPECT_FLOAT_EQ(plan.entities[1].scale.x, 1.0f);
}

TEST(Unit_ModelInstantiationPlan, EveryParentPrecedesItsChild)
{
    GLTFSceneDescription description;
    description.nodes.push_back(node("Root", -1, 0));
    description.nodes.push_back(node("Middle", 0, 1));
    description.nodes.push_back(node("Leaf", 1, 2));

    ModelImportReport report;
    const auto plan = plan_model_instantiation(description, ModelImportSettings{}, "Chain", report);
    ASSERT_EQ(plan.entities.size(), 3u);
    for (std::size_t i = 0; i < plan.entities.size(); ++i)
        EXPECT_LT(plan.entities[i].parent, static_cast<std::int32_t>(i));
    EXPECT_EQ(plan.entities[2].name, "Leaf");
    EXPECT_EQ(plan.entities[2].parent, 1);
}

TEST(Unit_ModelInstantiationPlan, SeveralRootsGainASyntheticRootNamedAfterTheFile)
{
    GLTFSceneDescription description;
    description.nodes.push_back(node("First", -1, 0));
    description.nodes.push_back(node("Second", -1, 1));

    ModelImportReport report;
    const auto plan = plan_model_instantiation(description, ModelImportSettings{}, "Scene", report);
    ASSERT_EQ(plan.entities.size(), 3u);
    EXPECT_EQ(plan.entities[0].name, "Scene");
    EXPECT_EQ(plan.entities[0].parent, -1);
    EXPECT_EQ(plan.entities[1].parent, 0);
    EXPECT_EQ(plan.entities[2].parent, 0);
}

TEST(Unit_ModelInstantiationPlan, ASingleRootIsTheSubtreeRootWithNoWrapper)
{
    GLTFSceneDescription description;
    description.nodes.push_back(node("Car", -1, 0));
    description.nodes.push_back(node("Wheel", 0, 1));

    ModelImportReport report;
    const auto plan = plan_model_instantiation(description, ModelImportSettings{}, "Scene", report);
    ASSERT_EQ(plan.entities.size(), 2u);
    EXPECT_EQ(plan.entities[0].name, "Car");
}

TEST(Unit_ModelInstantiationPlan, AnUnnamedNodeIsNamedAfterItsFileIndex)
{
    GLTFSceneDescription description;
    description.nodes.push_back(node("", -1, 7));

    ModelImportReport report;
    const auto plan = plan_model_instantiation(description, ModelImportSettings{}, "Scene", report);
    ASSERT_EQ(plan.entities.size(), 1u);
    EXPECT_EQ(plan.entities[0].name, "node 7");
}

TEST(Unit_ModelInstantiationPlan, SiblingNamesAreDisambiguatedAndCousinsKeepTheirSharedName)
{
    GLTFSceneDescription description;
    description.nodes.push_back(node("Root", -1, 0));
    description.nodes.push_back(node("Wheel", 0, 1));
    description.nodes.push_back(node("Wheel", 0, 2));
    description.nodes.push_back(node("Tire", 1, 3));
    description.nodes.push_back(node("Tire", 2, 4));

    ModelImportReport report;
    const auto plan = plan_model_instantiation(description, ModelImportSettings{}, "Scene", report);
    ASSERT_EQ(plan.entities.size(), 5u);
    EXPECT_EQ(plan.entities[1].name, "Wheel");
    EXPECT_EQ(plan.entities[2].name, "Wheel (1)");
    // Cousins under different parents keep the file's own name: renaming one would misdescribe
    // the file.
    EXPECT_EQ(plan.entities[3].name, "Tire");
    EXPECT_EQ(plan.entities[4].name, "Tire");
}

TEST(Unit_ModelInstantiationPlan, ADirectionalLightIsSkippedAndCountedRatherThanCreated)
{
    GLTFSceneDescription description;
    GLTFNodeDescription sun = node("Sun", -1, 0);
    sun.light = 0;
    description.nodes.push_back(sun);
    GLTFLightDescription light;
    light.kind = GLTFLightKind::Directional;
    description.lights.push_back(light);

    ModelImportReport report;
    const auto plan = plan_model_instantiation(description, ModelImportSettings{}, "Scene", report);
    ASSERT_EQ(plan.entities.size(), 1u);
    EXPECT_EQ(plan.entities[0].component, PlannedComponent::None);
    EXPECT_EQ(report.lights_skipped_directional, 1u);
    EXPECT_EQ(report.lights_imported, 0u);
    EXPECT_FALSE(report.warnings.empty());
}

TEST(Unit_ModelInstantiationPlan, APointLightBecomesALightEntityWhenTheSettingIsOn)
{
    GLTFSceneDescription description;
    GLTFNodeDescription lamp = node("Lamp", -1, 0);
    lamp.light = 0;
    description.nodes.push_back(lamp);
    description.lights.push_back(GLTFLightDescription{});

    ModelImportReport report;
    const auto plan = plan_model_instantiation(description, ModelImportSettings{}, "Scene", report);
    ASSERT_EQ(plan.entities.size(), 1u);
    EXPECT_EQ(plan.entities[0].component, PlannedComponent::Light);
    EXPECT_EQ(plan.entities[0].light, 0);
    EXPECT_EQ(report.lights_imported, 1u);
}

TEST(Unit_ModelInstantiationPlan, ImportLightsOffLeavesTheNodeAPlainTransform)
{
    GLTFSceneDescription description;
    GLTFNodeDescription lamp = node("Lamp", -1, 0);
    lamp.light = 0;
    description.nodes.push_back(lamp);
    description.lights.push_back(GLTFLightDescription{});

    ModelImportSettings settings;
    settings.import_lights = false;
    ModelImportReport report;
    const auto plan = plan_model_instantiation(description, settings, "Scene", report);
    ASSERT_EQ(plan.entities.size(), 1u);
    EXPECT_EQ(plan.entities[0].component, PlannedComponent::None);
    EXPECT_EQ(report.lights_imported, 0u);
}

TEST(Unit_ModelInstantiationPlan, ACameraNodeBecomesACameraEntity)
{
    GLTFSceneDescription description;
    GLTFNodeDescription shot = node("Shot", -1, 0);
    shot.camera = 0;
    description.nodes.push_back(shot);
    description.cameras.push_back(GLTFCameraDescription{});

    ModelImportReport report;
    const auto plan = plan_model_instantiation(description, ModelImportSettings{}, "Scene", report);
    ASSERT_EQ(plan.entities.size(), 1u);
    EXPECT_EQ(plan.entities[0].component, PlannedComponent::Camera);
    EXPECT_EQ(plan.entities[0].camera, 0);
    EXPECT_EQ(report.cameras_imported, 1u);
}

TEST(Unit_ModelInstantiationPlan, ImportCamerasOffLeavesTheNodeAPlainTransform)
{
    GLTFSceneDescription description;
    GLTFNodeDescription shot = node("Shot", -1, 0);
    shot.camera = 0;
    description.nodes.push_back(shot);
    description.cameras.push_back(GLTFCameraDescription{});

    ModelImportSettings settings;
    settings.import_cameras = false;
    ModelImportReport report;
    const auto plan = plan_model_instantiation(description, settings, "Scene", report);
    ASSERT_EQ(plan.entities.size(), 1u);
    EXPECT_EQ(plan.entities[0].component, PlannedComponent::None);
    EXPECT_EQ(report.cameras_imported, 0u);
}

TEST(Unit_ModelInstantiationPlan, ANodeCarryingBothAMeshAndACameraPrefersItsGeometry)
{
    // glTF permits it and the engine's entity cannot be both. Geometry wins because it is what
    // the artist sees, and the camera is reported rather than dropped in silence.
    GLTFSceneDescription description;
    GLTFNodeDescription both = node("Both", -1, 0);
    both.mesh = 0;
    both.primitive_count = 1;
    both.camera = 0;
    description.nodes.push_back(both);
    description.cameras.push_back(GLTFCameraDescription{});

    ModelImportReport report;
    const auto plan = plan_model_instantiation(description, ModelImportSettings{}, "Scene", report);
    ASSERT_FALSE(plan.entities.empty());
    EXPECT_EQ(plan.entities[0].component, PlannedComponent::Shape);
    EXPECT_FALSE(report.warnings.empty());
}

TEST(Unit_ModelInstantiationPlan, ASkinnedNodeKeepsItsPlaceInTheTreeButLosesItsMesh)
{
    GLTFSceneDescription description;
    GLTFNodeDescription rigged = node("Body", -1, 0);
    rigged.mesh = 0;
    rigged.primitive_count = 1;
    rigged.skin = 0;
    description.nodes.push_back(rigged);
    description.skin_count = 1;

    ModelImportReport report;
    const auto plan = plan_model_instantiation(description, ModelImportSettings{}, "Scene", report);
    ASSERT_EQ(plan.entities.size(), 1u);
    EXPECT_EQ(plan.entities[0].component, PlannedComponent::None);
    EXPECT_EQ(report.skinned_nodes_skipped, 1u);
    EXPECT_EQ(report.primitives_imported, 0u);
}

TEST(Unit_ModelInstantiationPlan, ScaleFactorReachesGeometryScaleAndEveryTranslationButNoParentChain)
{
    GLTFSceneDescription description;
    GLTFNodeDescription root = node("Root", -1, 0);
    root.translation = SushiEngine::Geometry::Vector3f{1.0f, 0.0f, 0.0f};
    description.nodes.push_back(root);
    GLTFNodeDescription child = node("Child", 0, 1);
    child.translation = SushiEngine::Geometry::Vector3f{2.0f, 0.0f, 0.0f};
    child.mesh = 0;
    child.primitive_count = 1;
    description.nodes.push_back(child);

    ModelImportSettings settings;
    settings.scale_factor = 0.5f;
    ModelImportReport report;
    const auto plan = plan_model_instantiation(description, settings, "Scene", report);
    ASSERT_EQ(plan.entities.size(), 2u);

    // Every translation is scaled, so the composed position scales with it: the file places the
    // child at 3 metres out, and half-scale puts it at 1.5.
    EXPECT_FLOAT_EQ(world_position(plan, 1).x, 1.5f);
    // The root carries no scale, so the factor cannot compound down the hierarchy.
    EXPECT_FLOAT_EQ(plan.entities[0].scale.x, 1.0f);
    // The factor lands on the geometry-carrying entity, which is where resolve_collider reads it.
    EXPECT_FLOAT_EQ(plan.entities[1].scale.x, 0.5f);
}

TEST(Unit_ModelInstantiationPlan, PreservePivotsOffDropsAnEmptyNodeAndKeepsItsChildWhereItWas)
{
    GLTFSceneDescription description;
    GLTFNodeDescription root = node("Root", -1, 0);
    root.translation = SushiEngine::Geometry::Vector3f{1.0f, 0.0f, 0.0f};
    root.mesh = 0;
    root.primitive_count = 1;
    description.nodes.push_back(root);
    GLTFNodeDescription pivot = node("Pivot", 0, 1);
    pivot.translation = SushiEngine::Geometry::Vector3f{2.0f, 0.0f, 0.0f};
    description.nodes.push_back(pivot);
    GLTFNodeDescription leaf = node("Leaf", 1, 2);
    leaf.translation = SushiEngine::Geometry::Vector3f{4.0f, 0.0f, 0.0f};
    leaf.mesh = 0;
    leaf.primitive_count = 1;
    description.nodes.push_back(leaf);

    ModelImportReport kept_report;
    const auto kept =
        plan_model_instantiation(description, ModelImportSettings{}, "Scene", kept_report);
    ASSERT_EQ(kept.entities.size(), 3u);
    const float expected = world_position(kept, 2).x;

    ModelImportSettings settings;
    settings.preserve_pivots = false;
    ModelImportReport report;
    const auto dropped = plan_model_instantiation(description, settings, "Scene", report);
    ASSERT_EQ(dropped.entities.size(), 2u);
    EXPECT_EQ(report.pivots_dropped, 1u);
    EXPECT_EQ(dropped.entities[1].name, "Leaf");
    EXPECT_EQ(dropped.entities[1].parent, 0);
    // Dropping a pivot must not move what hung from it.
    EXPECT_FLOAT_EQ(world_position(dropped, 1).x, expected);
}

TEST(Unit_ModelInstantiationPlan, DroppingARotatedAndScaledPivotComposesRatherThanAddsItsTransform)
{
    // Every other pivot fixture uses identity rotations, which a planner that added vectors
    // instead of composing transforms would pass. This one cannot be passed that way: the
    // pivot turns a quarter-turn about Z and doubles, so its child's folded translation is the
    // rotated, scaled offset rather than the sum.
    constexpr float QUARTER_TURN = 1.5707963267948966f;
    const Quaternionf about_z = SushiEngine::quaternion_axis_angle(
        SushiEngine::Geometry::Vector3f{0.0f, 0.0f, 1.0f}, QUARTER_TURN);

    GLTFSceneDescription description;
    description.nodes.push_back(node("Root", -1, 0));
    GLTFNodeDescription pivot = node("Pivot", 0, 1);
    pivot.translation = SushiEngine::Geometry::Vector3f{3.0f, 0.0f, 0.0f};
    pivot.rotation = about_z;
    pivot.scale = SushiEngine::Geometry::Vector3f{2.0f, 2.0f, 2.0f};
    description.nodes.push_back(pivot);
    GLTFNodeDescription leaf = node("Leaf", 1, 2);
    leaf.translation = SushiEngine::Geometry::Vector3f{0.0f, 1.0f, 0.0f};
    leaf.mesh = 0;
    leaf.primitive_count = 1;
    description.nodes.push_back(leaf);

    ModelImportSettings settings;
    settings.preserve_pivots = false;
    ModelImportReport report;
    const auto plan = plan_model_instantiation(description, settings, "Scene", report);
    ASSERT_EQ(plan.entities.size(), 2u);
    EXPECT_EQ(report.pivots_dropped, 1u);

    // (3,0,0) + Rz(90 deg) * (2,2,2) * (0,1,0) = (3,0,0) + (-2,0,0).
    EXPECT_NEAR(plan.entities[1].translation.x, 1.0f, 1e-5f);
    EXPECT_NEAR(plan.entities[1].translation.y, 0.0f, 1e-5f);
    // The pivot's rotation and scale reach the child too, not just its translation.
    EXPECT_NEAR(plan.entities[1].rotation.z, about_z.z, 1e-5f);
    EXPECT_NEAR(plan.entities[1].rotation.w, about_z.w, 1e-5f);
    EXPECT_NEAR(plan.entities[1].scale.x, 2.0f, 1e-5f);
}

TEST(Unit_ModelInstantiationPlan, PreservePivotsOffKeepsANodeThatCarriesSomething)
{
    GLTFSceneDescription description;
    description.nodes.push_back(node("Root", -1, 0));
    GLTFNodeDescription lamp = node("Lamp", 0, 1);
    lamp.light = 0;
    description.nodes.push_back(lamp);
    description.lights.push_back(GLTFLightDescription{});

    ModelImportSettings settings;
    settings.preserve_pivots = false;
    ModelImportReport report;
    const auto plan = plan_model_instantiation(description, settings, "Scene", report);
    // The root is never dropped, and a node carrying a light is not empty.
    ASSERT_EQ(plan.entities.size(), 2u);
    EXPECT_EQ(report.pivots_dropped, 0u);
}

TEST(Unit_ModelInstantiationPlan, GenerateCollidersMarksOnlyGeometryCarryingEntities)
{
    GLTFSceneDescription description;
    description.nodes.push_back(node("Root", -1, 0));
    GLTFNodeDescription part = node("Part", 0, 1);
    part.mesh = 0;
    part.primitive_count = 1;
    description.nodes.push_back(part);

    ModelImportSettings settings;
    settings.generate_colliders = true;
    ModelImportReport report;
    const auto plan = plan_model_instantiation(description, settings, "Scene", report);
    ASSERT_EQ(plan.entities.size(), 2u);
    EXPECT_FALSE(plan.entities[0].generate_collider);
    EXPECT_TRUE(plan.entities[1].generate_collider);
}

TEST(Unit_ModelInstantiationPlan, AColliderUnderAFileScaledParentIsWarnedAbout)
{
    GLTFSceneDescription description;
    GLTFNodeDescription root = node("Root", -1, 0);
    root.scale = SushiEngine::Geometry::Vector3f{2.0f, 2.0f, 2.0f};
    description.nodes.push_back(root);
    GLTFNodeDescription part = node("Part", 0, 1);
    part.mesh = 0;
    part.primitive_count = 1;
    description.nodes.push_back(part);

    ModelImportSettings settings;
    settings.generate_colliders = true;
    ModelImportReport report;
    const auto plan = plan_model_instantiation(description, settings, "Scene", report);
    // The engine resolves a collider at the entity's own scale, so a parent's scale does not
    // reach it. The importer does not work around that; it says so.
    EXPECT_FALSE(report.warnings.empty());
    EXPECT_EQ(plan.entities.size(), 2u);
}

TEST(Unit_ModelInstantiationPlan, TheRootRotationLandsOnTheRootAndNowhereElse)
{
    GLTFSceneDescription description;
    description.nodes.push_back(node("Root", -1, 0));
    description.nodes.push_back(node("Child", 0, 1));

    ModelImportSettings settings;
    settings.root_rotation_degrees = Vector3f{-90.0f, 0.0f, 0.0f};
    ModelImportReport report;
    const auto plan = plan_model_instantiation(description, settings, "Scene", report);
    ASSERT_EQ(plan.entities.size(), 2u);
    EXPECT_GT(std::fabs(plan.entities[0].rotation.x), 0.001f);
    EXPECT_FLOAT_EQ(plan.entities[1].rotation.x, 0.0f);
    EXPECT_FLOAT_EQ(plan.entities[1].rotation.w, 1.0f);
}

TEST(Unit_ModelInstantiationPlan, TheRootRotationLandsOnASyntheticRootWhenTheFileHasSeveral)
{
    // A multi-root file's subtree root is the synthetic one, so that is what the correction has
    // to reach — otherwise a non-conformant file with two roots would import unrotated.
    GLTFSceneDescription description;
    description.nodes.push_back(node("First", -1, 0));
    description.nodes.push_back(node("Second", -1, 1));

    ModelImportSettings settings;
    settings.root_rotation_degrees = Vector3f{-90.0f, 0.0f, 0.0f};
    ModelImportReport report;
    const auto plan = plan_model_instantiation(description, settings, "Scene", report);
    ASSERT_EQ(plan.entities.size(), 3u);
    EXPECT_GT(std::fabs(plan.entities[0].rotation.x), 0.001f);
    EXPECT_FLOAT_EQ(plan.entities[1].rotation.w, 1.0f);
    EXPECT_FLOAT_EQ(plan.entities[2].rotation.w, 1.0f);
}

TEST(Unit_ModelInstantiationPlan, TheReportCountsWhatTheFileHeldAndWhatThePlanProduced)
{
    GLTFSceneDescription description;
    description.nodes.push_back(node("Root", -1, 0));
    GLTFNodeDescription part = node("Part", 0, 1);
    part.mesh = 0;
    part.primitive_count = 3;
    description.nodes.push_back(part);
    description.material_count = 4;

    ModelImportReport report;
    const auto plan = plan_model_instantiation(description, ModelImportSettings{}, "Scene", report);
    EXPECT_EQ(report.nodes, 2u);
    EXPECT_EQ(report.entities, static_cast<std::uint32_t>(plan.entities.size()));
    EXPECT_EQ(report.entities, 5u);
    EXPECT_EQ(report.primitives_imported, 3u);
    EXPECT_EQ(report.materials, 4u);
}
