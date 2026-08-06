# Model Import Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development
> (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use
> checkbox (`- [ ]`) syntax for tracking.

**Goal:** Import a glTF file's node graph as an entity hierarchy, driven by per-asset settings
in a `.meta` sidecar, reachable by dragging the asset out of the Project panel.

**Architecture:** Three layers that cannot see each other. `engine/asset/gltf` reads the file into a
renderer-free `GLTFSceneDescription`. A new `engine/asset/model` module turns that plus a
`ModelImportSettings` into a `ModelInstantiationPlan` through one pure function with no device and
no editor. The editor executes the plan against `IWorldEditor` and `IAssetLibrary` and decides
nothing. Every hard decision is therefore unit-testable without a GPU.

**Tech Stack:** C++17, cgltf 1.15 (already vendored, owned by `sushiengine_gltf`), nlohmann_json,
GoogleTest, CMake with `sushiengine_add_module`, Dear ImGui for the editor surface.

**Spec:** `docs/design/model_import.md`. Every section reference below (§N) points into it.

## Global Constraints

- **C++17 only.** No C++20/23 facilities. (`docs/CONTRIBUTING.md` §4)
- **Allman braces**, opening brace on its own line, including for namespaces. Nested namespaces
  written out (`namespace A\n{\n namespace B\n{`), never `namespace A::B`.
- **Naming:** types `PascalCase`, functions and variables `snake_case`, members trailing underscore,
  constants `UPPER_SNAKE`, namespaces `PascalCase`.
- **No abbreviations in any identifier or in prose.** `Statistics` not `Stats`, `Parameters` not
  `Params`. Well-known acronyms stay fully upper-case: `GPU`, `API`, `GLTF`, `ECS`.
- **Every CMake target, function and option starts with `sushiengine_`.** Never bare `sushi_`.
- **Every new source file carries the Apache 2.0 header** copied verbatim from a neighbouring file.
- **Every public function carries Doxygen** with `@brief` (why it exists), one line on the mechanism
  when non-obvious, `@param` for each parameter and `@return`. (`docs/CONTRIBUTING.md` §4)
- **No historical references in comments.** No "previously", no "we used to", no issue numbers.
- **No separator comments** (`// =====`, `// -----`).
- **Prose in `docs/` obeys `docs/documentation-style-guide.md`:** 100-column lines, present tense,
  no marketing, honest about gaps, every path real and every link resolving.
- **Never invoke `cmake` or `ninja` directly.** Only the `se` CLI. (`CLAUDE.md`)

## Build and test policy — read before Task 1

**The implementing agent does not run builds or tests.** This machine cannot carry them; the user
runs every `se build` and `se test` invocation. Steps below marked **[USER CHECKPOINT]** are handed
to the user with the exact command, and work stops until they report the result.

An agent must never report a test as passing that it did not see pass. Where a step cannot be
executed, say so plainly and hand it over. Writing the test is the agent's job; observing it fail
and then pass is the user's.

The three checks that guard the tree, all runnable by the agent because they are pure Python and
touch no compiler:

```bash
python tools/documentation/check_documentation_length.py
python tools/documentation/check_module_documentation.py
python tools/layering/check_include_layering.py
```

The build and test commands the user runs:

```bash
se build                          # configure and build
se test --suite functional         # unit + integration + regression
```

---
---

## Test naming — non-negotiable

`tests/CMakeLists.txt:665-676` registers `gtest_discover_tests` three times, filtering on
`Unit_*`, `Integration_*` and `Regression_*`. **A test whose suite name carries none of those
prefixes is discovered by nothing**: CTest runs zero of its cases and `se test --suite functional`
reports green having never executed them. Every suite name in this part is therefore
`Unit_<Name>` for a file under `tests/unit/` and `Integration_<Name>` for one under
`tests/integration/`. Do not drop the prefix, and do not trust a green run of a test you cannot
see named in the CTest output.

## Where this plan sits

This is **part three of four** of the model import plan. It builds the decision that turns a
node graph into a list of entities,

and the render seam that returns imported meshes tagged with the node they came from.

Consumes: `Geometry::GLTFSceneDescription` and `import_gltf_scene` (part one, Task 1);
`Model::ModelImportSettings` and the `sushiengine_model` module (part two, Task 2).

Produces `plan_model_instantiation`, `ModelInstantiationPlan`, `ModelImportReport`,
`PlannedEntity`, `PlannedComponent`, `Render::ImportedPrimitive` and
`IAssetLibrary::load_gltf_scene`. Part four calls every one of them.

Task 4 is the task that carries the design: everything difficult is there, and none of it
touches a device, a window or a world.

The plan is split because a plan file obeys `docs/documentation-style-guide.md`'s 900-line
ceiling. Each part is one subject and each ends with something that builds and tests.

---

## Task 4: The instantiation plan — the pure decision function

This is the task that carries the design. Everything difficult is here, and nothing here touches a
device, a window or a world.

**Files:**
- Create: `engine/asset/model/include/SushiEngine/model/instantiation_plan.hpp`
- Modify: `engine/asset/model/source/instantiation_plan.cpp` (created empty in Task 2)
- Modify: `engine/asset/model/README.md`
- Test: `tests/unit/test_model_instantiation_plan.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `Geometry::GLTFSceneDescription` (Task 1), `Model::ModelImportSettings` (Task 2).
- Produces: `PlannedComponent`, `PlannedEntity`, `ModelInstantiationPlan`, `ModelImportReport`, and
  `ModelInstantiationPlan plan_model_instantiation(const Geometry::GLTFSceneDescription&, const
  ModelImportSettings&, const std::string& file_stem, ModelImportReport& report)`. Tasks 6 and 9
  consume these exact names.

- [ ] **Step 1: Write the header**

```cpp
#pragma once

/**
 * @file instantiation_plan.hpp
 * @brief What entities a glTF file becomes, decided without creating any.
 *
 * The whole of the import's decision-making: which node becomes which entity, when a node
 * splits because it carries more materials than one entity can, how a dropped pivot folds its
 * transform into its children, how a name collision resolves, and where the scale factor goes.
 * Separated from the code that creates entities so all of it is testable with no device, no
 * window and no world — the editor's executor then has no branches worth testing.
 */

#include <cstdint>
#include <string>
#include <vector>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/gltf/scene_import.hpp>
#include <SushiEngine/model/import_settings.hpp>

namespace SushiEngine
{
    namespace Model
    {
        /** @brief Unit quaternion in import precision; `Vector3f` comes from import_settings.hpp. */
        using Quaternionf = QuaternionT<float>;

        /** @brief What a planned entity carries beyond its name and transform. */
        enum class PlannedComponent
        {
            None,   /**< A pure transform: a pivot, or a node whose mesh split into children. */
            Shape,  /**< One imported primitive, drawn with one material. */
            Light,
            Camera
        };

        /** @brief One entity the import creates. */
        struct PlannedEntity
        {
            std::string name;

            /** @brief Index into @ref ModelInstantiationPlan::entities, or -1 for the root. */
            std::int32_t parent = -1;

            Vector3f translation{0.0f, 0.0f, 0.0f};
            Quaternionf rotation{0.0f, 0.0f, 0.0f, 1.0f};
            Vector3f scale{1.0f, 1.0f, 1.0f};

            PlannedComponent component = PlannedComponent::None;

            /** @brief The glTF node this came from; the key a renderer's import is joined on. */
            std::uint32_t source_node = 0;

            /** @brief Which primitive of that node's mesh, when @ref component is Shape. */
            std::uint32_t primitive = 0;

            /** @brief Index into `GLTFSceneDescription::lights`, when component is Light. */
            std::int32_t light = -1;

            /** @brief Index into `GLTFSceneDescription::cameras`, when component is Camera. */
            std::int32_t camera = -1;

            /** @brief Whether this entity's mesh should be queued for cooking (§9's setting). */
            bool generate_collider = false;
        };

        /** @brief The entities to create, parents always before their children. */
        struct ModelInstantiationPlan
        {
            std::vector<PlannedEntity> entities;
        };

        /** @brief What an import found, produced and could not use. */
        struct ModelImportReport
        {
            std::uint32_t nodes = 0;
            std::uint32_t entities = 0;
            std::uint32_t primitives_imported = 0;
            std::uint32_t lights_imported = 0;
            std::uint32_t lights_skipped_directional = 0;
            std::uint32_t cameras_imported = 0;
            std::uint32_t skinned_nodes_skipped = 0;
            std::uint32_t pivots_dropped = 0;
            std::uint32_t materials = 0;

            /** @brief Everything an artist has to be told, in the words they need. */
            std::vector<std::string> warnings;
        };

        /**
         * @brief Decides what entities a described glTF file becomes.
         *
         * Pure: reads no file, touches no device, creates nothing. Applies the rules the design
         * document's §5 states, in that order, and records in @p report everything it could not
         * carry across so a partial import is visibly partial.
         *
         * @param description The file's node graph, from `Geometry::import_gltf_scene`.
         * @param settings    The asset's `.meta` settings.
         * @param file_stem   The asset's file name without extension; names the synthetic root a
         *                    multi-root file needs, and nothing else.
         * @param report      Receives counts and warnings; overwritten.
         * @return The entities to create. Empty when the description holds no node.
         */
        ModelInstantiationPlan plan_model_instantiation(
            const Geometry::GLTFSceneDescription& description, const ModelImportSettings& settings,
            const std::string& file_stem, ModelImportReport& report);
    } // namespace Model
} // namespace SushiEngine
```

- [ ] **Step 2: Write the failing tests**

Create `tests/unit/test_model_instantiation_plan.cpp` with the Apache header. Build descriptions by
hand — the point of this task is that no file is needed.

```cpp
#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include <SushiEngine/model/instantiation_plan.hpp>

using SushiEngine::Geometry::GLTFCameraDescription;
using SushiEngine::Geometry::GLTFLightDescription;
using SushiEngine::Geometry::GLTFLightKind;
using SushiEngine::Geometry::GLTFNodeDescription;
using SushiEngine::Geometry::GLTFSceneDescription;
using SushiEngine::Model::ModelImportReport;
using SushiEngine::Model::ModelImportSettings;
using SushiEngine::Model::PlannedComponent;
using SushiEngine::Model::plan_model_instantiation;

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
    // left out deliberately: every fixture here uses identity rotations, and a test helper that
    // reimplements the engine's transform composition would be testing itself.
    SushiEngine::Vector3f world_position(const SushiEngine::Model::ModelInstantiationPlan& plan,
                                         std::size_t index)
    {
        SushiEngine::Vector3f total{0.0f, 0.0f, 0.0f};
        SushiEngine::Vector3f accumulated_scale{1.0f, 1.0f, 1.0f};
        for (std::int32_t i = static_cast<std::int32_t>(index); i >= 0;
             i = plan.entities[static_cast<std::size_t>(i)].parent)
        {
            const auto& entity = plan.entities[static_cast<std::size_t>(i)];
            total.x += entity.translation.x * accumulated_scale.x;
            total.y += entity.translation.y * accumulated_scale.y;
            total.z += entity.translation.z * accumulated_scale.z;
            accumulated_scale.x *= entity.scale.x;
            accumulated_scale.y *= entity.scale.y;
            accumulated_scale.z *= entity.scale.z;
        }
        return total;
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
    root.translation = SushiEngine::Vector3f{1.0f, 0.0f, 0.0f};
    description.nodes.push_back(root);
    GLTFNodeDescription child = node("Child", 0, 1);
    child.translation = SushiEngine::Vector3f{2.0f, 0.0f, 0.0f};
    child.mesh = 0;
    child.primitive_count = 1;
    description.nodes.push_back(child);

    ModelImportSettings settings;
    settings.scale_factor = 0.5f;
    ModelImportReport report;
    const auto plan = plan_model_instantiation(description, settings, "Scene", report);
    ASSERT_EQ(plan.entities.size(), 2u);

    // Every translation is scaled, so the composed position scales with it.
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
    root.translation = SushiEngine::Vector3f{1.0f, 0.0f, 0.0f};
    root.mesh = 0;
    root.primitive_count = 1;
    description.nodes.push_back(root);
    GLTFNodeDescription pivot = node("Pivot", 0, 1);
    pivot.translation = SushiEngine::Vector3f{2.0f, 0.0f, 0.0f};
    description.nodes.push_back(pivot);
    GLTFNodeDescription leaf = node("Leaf", 1, 2);
    leaf.translation = SushiEngine::Vector3f{4.0f, 0.0f, 0.0f};
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
    root.scale = SushiEngine::Vector3f{2.0f, 2.0f, 2.0f};
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
}

TEST(Unit_ModelInstantiationPlan, TheRootRotationLandsOnTheRootAndNowhereElse)
{
    GLTFSceneDescription description;
    description.nodes.push_back(node("Root", -1, 0));
    description.nodes.push_back(node("Child", 0, 1));

    ModelImportSettings settings;
    settings.root_rotation_degrees = SushiEngine::Vector3f{-90.0f, 0.0f, 0.0f};
    ModelImportReport report;
    const auto plan = plan_model_instantiation(description, settings, "Scene", report);
    ASSERT_EQ(plan.entities.size(), 2u);
    EXPECT_GT(std::fabs(plan.entities[0].rotation.x), 0.001f);
    EXPECT_FLOAT_EQ(plan.entities[1].rotation.x, 0.0f);
    EXPECT_FLOAT_EQ(plan.entities[1].rotation.w, 1.0f);
}
```

- [ ] **Step 3: Register the test**

Add `unit/test_model_instantiation_plan.cpp` to `tests/CMakeLists.txt`'s executable source list.

- [ ] **Step 4: [USER CHECKPOINT] Verify the tests fail**

Hand the user: `se build`
Expected: a link error naming `plan_model_instantiation`.

- [ ] **Step 5: Implement the planner**

Write `engine/asset/model/source/instantiation_plan.cpp`. Structure it as a first pass that decides
which description nodes survive (pivot dropping), building a map from description index to planned
index or to "folded into parent", and a second pass that emits entities in description order so the
parent-before-child invariant is inherited from Task 1's own invariant rather than re-established.

Apply the rules in §5's stated order. Specific points the tests pin down:

- A dropped pivot's translation composes into each child's translation through the child's own
  rotation and scale — get this right by composing the transforms rather than adding vectors, and
  keep the fixtures' identity rotations from hiding a bug by adding one case with a rotated pivot
  once the identity ones pass.
- The synthetic root is emitted first, at index 0, when the description has more than one root.
- Sibling disambiguation appends `" (n)"` starting at 1 for the second occurrence, scoped to one
  parent's children.
- `report.materials` is `description.material_count`; `report.nodes` is `description.nodes.size()`.
- The `generate_colliders` warning fires when a planned entity with `generate_collider` has any
  ancestor whose description scale is not one.
- **A node carrying more than one of mesh, light and camera resolves in that priority order**, and
  reports what it could not also be. glTF permits the combination and an entity carries one of
  these, so the alternative is dropping something silently. Geometry wins because it is what an
  artist sees. This rule is not in the design document's §5 list as originally written; it was found
  by writing the tests, and Task 10 adds it to the document rather than leaving the code ahead of
  the specification.

- [ ] **Step 6: [USER CHECKPOINT] Verify the tests pass**

```bash
se build
se test --suite functional
```

Expected: every `Unit_ModelInstantiationPlan.*` case passes.

- [ ] **Step 7: Update the README and commit**

Add `instantiation_plan.hpp` to `engine/asset/model/README.md`'s public-surface table and name the
test file in its Tests section.

```bash
python tools/documentation/check_documentation_length.py
python tools/documentation/check_module_documentation.py
git add engine/asset/model tests/unit/test_model_instantiation_plan.cpp tests/CMakeLists.txt
git commit -m "feat(model): decide what entities a glTF node graph becomes"
```

---

## Task 5: The render seam — meshes with provenance, in local space

**Files:**
- Modify: `engine/presentation/render/include/SushiEngine/render/asset_library_interface.hpp`
- Modify: `engine/presentation/render/source/material/gltf_importer.{hpp,cpp}`
- Modify: `engine/presentation/render/source/material/asset_library.{hpp,cpp}`

**Interfaces:**
- Consumes: nothing from earlier tasks.
- Produces: `Render::ImportedPrimitive` and
  `std::size_t IAssetLibrary::load_gltf_scene(const char* path, ImportedPrimitive* out, std::size_t
  capacity)`. Task 6 consumes both.

- [ ] **Step 1: Declare the seam**

In `asset_library_interface.hpp`, beside `load_gltf` (line 118):

```cpp
/** @brief One imported primitive, and the glTF node it belongs to. */
struct ImportedPrimitive
{
    /** @brief The file's own node index — the key an importer's plan is joined on. */
    std::uint32_t source_node = 0;
    /** @brief Which primitive of that node's mesh. */
    std::uint32_t primitive = 0;
    MeshId mesh = INVALID_MESH;
    Render::Material material;
};

/**
 * @brief Imports every primitive of a glTF file, in mesh-local space, with its node recorded.
 *
 * Differs from @ref load_gltf in the two ways a scene-graph import needs. It reports which node
 * each primitive came from, so a caller matches meshes to nodes by the file's own index rather
 * than by trusting two parsers to walk in the same order forever. And it does not bake a node's
 * world transform into the vertices, because the entity's transform carries the placement:
 * baking it here would apply it twice, and leaving it out lets several nodes referencing one
 * glTF mesh share one @ref MeshId instead of each uploading their own.
 *
 * @param out      Receives one entry per imported primitive.
 * @param capacity Capacity of @p out.
 * @param path     Path to a `.gltf` or `.glb` file.
 * @return Number of entries written, or 0 if the file could not be read.
 */
virtual std::size_t load_gltf_scene(const char* path, ImportedPrimitive* out,
                                    std::size_t capacity) = 0;
```

Adding a pure virtual to `IAssetLibrary` breaks every implementer. Grep for `: public IAssetLibrary`
and for `IAssetLibrary` in `tests/` before writing, and add the override to each — a test double
that silently returns 0 is the correct behaviour for one, but it must be written, not inherited.

- [ ] **Step 2: Implement the importer variant**

In `gltf_importer.{hpp,cpp}`, add `import_gltf_scene_meshes` alongside `import_gltf`. Read
`import_gltf`'s attribute handling first: reuse it rather than copying it, factoring the shared
per-primitive vertex assembly into a helper both call. The two differences are that the new entry
point skips the node-world-transform bake and records `cgltf_node_index` and the primitive index
into each output entry.

- [ ] **Step 3: Implement it on `AssetLibrary` and extend the cache**

`asset_library.hpp:227-257` documents a per-path cache whose entry is meshes and materials. The two
entry points produce different vertices for one path, so the cache key must distinguish them.
Change `gltf_cache_`'s key to a pair of path and a small enum, or keep two maps — either is fine,
but the header's Doxygen must state which, because it currently promises a plain by-path cache and
a stale promise there is a defect.

- [ ] **Step 4: [USER CHECKPOINT] Verify the tree still builds**

```bash
se build
se test --suite functional
```

Expected: a clean build and no regression. There is no new test here: this seam's behaviour is
exercised through Task 6, and asserting a `MeshId` without a device is not something this repository
can do (`engine/asset/gltf/README.md` states the same limitation about the renderer's importer).
Say that plainly in the commit rather than implying coverage that does not exist.

- [ ] **Step 5: Commit**

```bash
git add engine/presentation/render
git commit -m "feat(render): import a glTF's primitives in local space with their node index"
```
