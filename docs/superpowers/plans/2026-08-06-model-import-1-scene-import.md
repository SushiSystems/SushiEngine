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

This is **part one of four** of the model import plan. It builds reading a glTF file's node graph.

Task numbering runs across all four parts. Later parts consume `import_gltf_scene` and
`GLTFSceneDescription` from this one, so nothing else can start until it is done.

The File structure table below covers all four parts, so a reader of any one of them can see
where their work sits in the whole.

The plan is split because a plan file obeys `docs/documentation-style-guide.md`'s 900-line
ceiling. Each part is one subject and each ends with something that builds and tests.

## File structure

**Created:**

| Path | Responsibility |
|---|---|
| `engine/asset/gltf/include/SushiEngine/gltf/scene_import.hpp` | The node-graph description type and `import_gltf_scene`. |
| `engine/asset/gltf/source/scene_importer.cpp` | cgltf walk producing that description. |
| `engine/asset/model/CMakeLists.txt` | The new module. |
| `engine/asset/model/README.md` | What the module owns, its tier, dependencies, tests. |
| `engine/asset/model/include/SushiEngine/model/import_settings.hpp` | `ModelImportSettings`. |
| `engine/asset/model/include/SushiEngine/model/import_settings_io.hpp` | `.meta` load and save. |
| `engine/asset/model/include/SushiEngine/model/instantiation_plan.hpp` | Plan, report, and the pure planning function. |
| `engine/asset/model/source/import_settings_io.cpp` | JSON reading and writing. |
| `engine/asset/model/source/instantiation_plan.cpp` | The planning function. |
| `applications/editor/source/project/model_instantiate.hpp` | Editor-side execution entry point. |
| `applications/editor/source/project/model_instantiate.cpp` | Executes a plan; decides nothing. |
| `tests/unit/test_model_instantiation_plan.cpp` | The planning function's behaviour. |
| `tests/unit/test_model_import_settings_io.cpp` | `.meta` round trip and migration. |
| `tests/integration/test_gltf_scene_import.cpp` | `import_gltf_scene` against a real asset. |

**Modified:**

| Path | Change |
|---|---|
| `cmake/EngineLayers.cmake:18-42` | Register `model asset` in `SUSHIENGINE_MODULE_LAYERS`. |
| `engine/asset/CMakeLists.txt` | `add_subdirectory(model)`. |
| `engine/asset/gltf/CMakeLists.txt` | Add `source/scene_importer.cpp`. |
| `engine/asset/gltf/README.md` | New header row; update the tests section. |
| `engine/presentation/render/include/SushiEngine/render/asset_library_interface.hpp` | `ImportedPrimitive`, `load_gltf_scene`. |
| `engine/presentation/render/source/material/asset_library.{hpp,cpp}` | Implement it; extend the cache key. |
| `engine/presentation/render/source/material/gltf_importer.{hpp,cpp}` | `import_gltf_scene_meshes`, local space, node provenance. |
| `engine/world/authoring/include/SushiEngine/authoring/cook_bake_state.hpp` | Declare the migration. |
| `engine/world/authoring/source/cook_bake_state.cpp:252-298` | Migrate overrides to `.meta`, drop the object. |
| `applications/editor/source/project/project_panel.cpp:62-104,474-511` | Naming fix, model routing, `.meta` filtering. |
| `applications/editor/source/scene/hierarchy_panel.cpp` | Asset drop target. |
| `applications/editor/source/ui/viewport_panel.cpp` | Asset drop target. |
| `applications/editor/CMakeLists.txt` | Add `model_instantiate.cpp`; link `sushiengine_model`. |
| `tests/CMakeLists.txt:38-608,619-660` | Register the three new test files; link `sushiengine_model`. |
| `docs/modules/README.md` | Index the new module. |
| `docs/reference/changelog.md` | Entries under `## [Unreleased]`. |
| `docs/design/model_import.md` §13 | Roadmap status as phases land. |

---
---

## Task 1: The glTF node-graph description

**Files:**
- Create: `engine/asset/gltf/include/SushiEngine/gltf/scene_import.hpp`
- Create: `engine/asset/gltf/source/scene_importer.cpp`
- Modify: `engine/asset/gltf/CMakeLists.txt`
- Modify: `engine/asset/gltf/README.md`
- Test: `tests/integration/test_gltf_scene_import.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: nothing from earlier tasks. cgltf is already linked by this module
  (`engine/asset/gltf/CMakeLists.txt`); `CGLTF_INCLUDE_DIR` is already located there.
- Produces: `SushiEngine::Geometry::GLTFSceneDescription`, `GLTFNodeDescription`,
  `GLTFLightDescription`, `GLTFCameraDescription`, `GLTFLightKind`, `GLTFCameraKind`, and
  `bool import_gltf_scene(const char* path, GLTFSceneDescription& out)`. Tasks 4, 5, 6 and 8 all
  depend on these exact names.

- [ ] **Step 1: Write the header**

Copy the Apache 2.0 header block verbatim from
`engine/asset/gltf/include/SushiEngine/gltf/mesh_import.hpp` lines 1-22, then:

```cpp
#pragma once

/**
 * @file scene_import.hpp
 * @brief A glTF file's node graph, exactly as the file states it.
 *
 * The counterpart to `mesh_import.hpp`, which answers "what triangles are in this file" by
 * merging everything into one mesh. This answers "what is this file's structure" and merges
 * nothing: one entry per node, keeping the name, the parent, the local transform and what the
 * node carries. Applying settings, deciding which node becomes what, and talking to a renderer
 * are all somebody else's job — this reports the file and stops.
 *
 * Nodes come back parent-before-child so a consumer builds a tree in one pass and never holds a
 * forward reference.
 */

#include <cstdint>
#include <string>
#include <vector>

#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Geometry
    {
        /**
         * @brief 3-vector and unit quaternion in import precision.
         *
         * Declared here rather than assumed global: `Vector3f` and `Quaternionf` are
         * per-namespace aliases of `Vector3T<float>`/`QuaternionT<float>` in this codebase,
         * following `SushiEngine/animation/skeleton.hpp`.
         */
        using Vector3f = Vector3T<float>;
        using Quaternionf = QuaternionT<float>;

        /** @brief What a `KHR_lights_punctual` light is. */
        enum class GLTFLightKind
        {
            Directional,
            Point,
            Spot
        };

        /** @brief Which projection a glTF camera uses. */
        enum class GLTFCameraKind
        {
            Perspective,
            Orthographic
        };

        /** @brief One `KHR_lights_punctual` light, in the file's own units. */
        struct GLTFLightDescription
        {
            std::string name;
            GLTFLightKind kind = GLTFLightKind::Point;
            float color[3] = {1.0f, 1.0f, 1.0f};
            float intensity = 1.0f;
            /** @brief Metres the light reaches, or zero for the glTF default of unlimited. */
            float range = 0.0f;
            float spot_inner_cone_radians = 0.0f;
            float spot_outer_cone_radians = 0.7853982f;
        };

        /** @brief One glTF camera, in the file's own units. */
        struct GLTFCameraDescription
        {
            std::string name;
            GLTFCameraKind kind = GLTFCameraKind::Perspective;
            /** @brief Vertical field of view in radians; perspective only. */
            float vertical_field_of_view_radians = 0.7853982f;
            /** @brief Horizontal magnification; orthographic only. */
            float orthographic_width = 1.0f;
            /** @brief Vertical magnification; orthographic only. */
            float orthographic_height = 1.0f;
            float near_plane = 0.1f;
            /** @brief Far plane, or zero when the file declares an infinite one. */
            float far_plane = 0.0f;
        };

        /** @brief One glTF node: where it sits, and what it carries. */
        struct GLTFNodeDescription
        {
            /** @brief The node's own name, or empty when the file does not name it. */
            std::string name;

            /** @brief Index into @ref GLTFSceneDescription::nodes, or -1 when this is a root. */
            std::int32_t parent = -1;

            /**
             * @brief The file's own node index.
             *
             * The join key a consumer matches against a renderer's imported primitives. A file
             * defines it, so two independent readers of the same file derive the same value;
             * traversal order does not have that property.
             */
            std::uint32_t source_index = 0;

            Vector3f translation{0.0f, 0.0f, 0.0f};
            Quaternionf rotation{0.0f, 0.0f, 0.0f, 1.0f};
            Vector3f scale{1.0f, 1.0f, 1.0f};

            /** @brief Index into the file's meshes, or -1 when the node carries none. */
            std::int32_t mesh = -1;

            /** @brief Primitives that mesh holds; zero when @ref mesh is -1. */
            std::uint32_t primitive_count = 0;

            /** @brief Index into @ref GLTFSceneDescription::cameras, or -1. */
            std::int32_t camera = -1;

            /** @brief Index into @ref GLTFSceneDescription::lights, or -1. */
            std::int32_t light = -1;

            /** @brief Index into the file's skins, or -1. */
            std::int32_t skin = -1;
        };

        /** @brief A glTF file's structure. */
        struct GLTFSceneDescription
        {
            /** @brief Every node, parents always before their children. */
            std::vector<GLTFNodeDescription> nodes;
            std::vector<GLTFLightDescription> lights;
            std::vector<GLTFCameraDescription> cameras;
            std::uint32_t material_count = 0;
            std::uint32_t skin_count = 0;
        };

        /**
         * @brief Reads a glTF file's node graph without reading its geometry.
         *
         * Parses the file and walks its default scene, decomposing each node's transform into
         * translation, rotation and scale rather than a matrix, because that is the form an
         * entity transform takes and converting to a matrix here only to decompose it again
         * downstream would lose precision for nothing. Buffers are not loaded: nothing here
         * reads vertex data.
         *
         * @param path Path to a `.gltf` or `.glb` file.
         * @param out  Receives the description; cleared first, and left cleared on failure.
         * @return False when the file cannot be parsed, or declares no node.
         */
        bool import_gltf_scene(const char* path, GLTFSceneDescription& out);
    } // namespace Geometry
} // namespace SushiEngine
```

Note on the vector types: `Vector3f` and `Quaternionf` are **not** global in this codebase. They are
per-namespace aliases of `Vector3T<float>` and `QuaternionT<float>`, which come from
`SushiEngine/core/types.hpp`; `SushiEngine/animation/skeleton.hpp:51-55` is the precedent this
header follows by declaring its own. Do not reach into `Animation` for them.

`gltf`'s CMake currently lists `core` under `PRIVATE_DEPENDS`
(`engine/asset/gltf/CMakeLists.txt`), which was correct while no header here named a scalar. This
header does, so promote `core` to `PUBLIC_DEPENDS` in that file and update the `core` line in
`engine/asset/gltf/README.md`'s Dependencies section from private to public, with the reason: a
header now names the type.

- [ ] **Step 2: Write the failing test**

Create `tests/integration/test_gltf_scene_import.cpp` with the Apache header, then:

```cpp
#include <gtest/gtest.h>

#include <string>

#include <SushiEngine/gltf/scene_import.hpp>

namespace
{
    std::string asset(const char* name)
    {
        return std::string(SE_TEST_ASSET_DIR) + "/" + name;
    }
}

TEST(GLTFSceneImport, AMissingFileFailsAndLeavesTheDescriptionEmpty)
{
    SushiEngine::Geometry::GLTFSceneDescription description;
    description.nodes.resize(3);
    EXPECT_FALSE(SushiEngine::Geometry::import_gltf_scene(
        asset("this_file_does_not_exist.gltf").c_str(), description));
    EXPECT_TRUE(description.nodes.empty());
}

TEST(GLTFSceneImport, ANullPathFails)
{
    SushiEngine::Geometry::GLTFSceneDescription description;
    EXPECT_FALSE(SushiEngine::Geometry::import_gltf_scene(nullptr, description));
}
```

`assets/models/` holds three files and every one of them is rigged (`morph_face.gltf`,
`rigged_arm_anim.gltf`, `rigged_chain.gltf`) — there is no static, multi-node asset in the tree to
test against. Rather than add a binary asset, the test authors its own glTF, which it can do
precisely because `import_gltf_scene` never calls `cgltf_load_buffers`: a hierarchy of nodes that
carry no mesh needs no buffer, no accessor and no binary blob, so a valid file is a short JSON
string. That makes the fixture deterministic and lets it set up exactly the shape under test.

```cpp
namespace
{
    // A hierarchy with one root, two children, a grandchild, a spot light and a camera. No
    // buffers: import_gltf_scene reads structure only, so nothing here needs vertex data.
    const char* HIERARCHY_GLTF = R"json({
      "asset": { "version": "2.0" },
      "extensionsUsed": [ "KHR_lights_punctual" ],
      "extensions": {
        "KHR_lights_punctual": {
          "lights": [
            { "name": "Lamp", "type": "spot", "color": [1, 0.5, 0.25], "intensity": 3,
              "range": 12, "spot": { "innerConeAngle": 0.2, "outerConeAngle": 0.6 } }
          ]
        }
      },
      "cameras": [
        { "name": "Shot", "type": "perspective",
          "perspective": { "yfov": 0.8, "znear": 0.1, "zfar": 500 } }
      ],
      "scene": 0,
      "scenes": [ { "nodes": [ 0 ] } ],
      "nodes": [
        { "name": "Root", "children": [ 1, 2 ], "translation": [1, 0, 0] },
        { "name": "Left", "children": [ 3 ], "translation": [0, 2, 0] },
        { "name": "Right", "camera": 0 },
        { "name": "Deep",
          "extensions": { "KHR_lights_punctual": { "light": 0 } } }
      ]
    })json";

    // Writes `contents` to `name` in the working directory and removes it on destruction, so a
    // run leaves nothing behind and two runs cannot see each other's files.
    class ScratchGLTF
    {
        public:
            explicit ScratchGLTF(const char* name, const char* contents) : path_(name)
            {
                std::ofstream stream(path_, std::ios::binary);
                stream << contents;
            }

            ~ScratchGLTF() { std::remove(path_.c_str()); }

            const char* path() const noexcept { return path_.c_str(); }

        private:
            std::string path_;
    };
}

TEST(GLTFSceneImport, EveryParentPrecedesItsChildAndEverySourceIndexIsUnique)
{
    const ScratchGLTF file("test_scene_import_hierarchy.gltf", HIERARCHY_GLTF);
    SushiEngine::Geometry::GLTFSceneDescription description;
    ASSERT_TRUE(SushiEngine::Geometry::import_gltf_scene(file.path(), description));
    ASSERT_EQ(description.nodes.size(), 4u);

    for (std::size_t i = 0; i < description.nodes.size(); ++i)
    {
        EXPECT_LT(description.nodes[i].parent, static_cast<std::int32_t>(i));
        EXPECT_GE(description.nodes[i].parent, -1);
        for (std::size_t j = 0; j < i; ++j)
            EXPECT_NE(description.nodes[j].source_index, description.nodes[i].source_index);
    }
}

TEST(GLTFSceneImport, TheTreeShapeAndTheLocalTransformsSurvive)
{
    const ScratchGLTF file("test_scene_import_shape.gltf", HIERARCHY_GLTF);
    SushiEngine::Geometry::GLTFSceneDescription description;
    ASSERT_TRUE(SushiEngine::Geometry::import_gltf_scene(file.path(), description));
    ASSERT_EQ(description.nodes.size(), 4u);

    EXPECT_EQ(description.nodes[0].name, "Root");
    EXPECT_EQ(description.nodes[0].parent, -1);
    EXPECT_FLOAT_EQ(description.nodes[0].translation.x, 1.0f);

    EXPECT_EQ(description.nodes[1].name, "Left");
    EXPECT_EQ(description.nodes[1].parent, 0);
    EXPECT_FLOAT_EQ(description.nodes[1].translation.y, 2.0f);

    // "Deep" hangs off "Left", so a depth-first walk must place it before "Right" would be if
    // the walk were breadth-first. Pinning the order pins the parent-before-child guarantee.
    EXPECT_EQ(description.nodes[2].name, "Deep");
    EXPECT_EQ(description.nodes[2].parent, 1);

    EXPECT_EQ(description.nodes[3].name, "Right");
    EXPECT_EQ(description.nodes[3].parent, 0);
}

TEST(GLTFSceneImport, APunctualLightAndACameraComeBackWithTheirValues)
{
    const ScratchGLTF file("test_scene_import_light.gltf", HIERARCHY_GLTF);
    SushiEngine::Geometry::GLTFSceneDescription description;
    ASSERT_TRUE(SushiEngine::Geometry::import_gltf_scene(file.path(), description));

    ASSERT_EQ(description.lights.size(), 1u);
    EXPECT_EQ(description.lights[0].kind, SushiEngine::Geometry::GLTFLightKind::Spot);
    EXPECT_FLOAT_EQ(description.lights[0].intensity, 3.0f);
    EXPECT_FLOAT_EQ(description.lights[0].range, 12.0f);
    EXPECT_FLOAT_EQ(description.lights[0].spot_outer_cone_radians, 0.6f);
    EXPECT_EQ(description.nodes[2].light, 0);

    ASSERT_EQ(description.cameras.size(), 1u);
    EXPECT_EQ(description.cameras[0].kind, SushiEngine::Geometry::GLTFCameraKind::Perspective);
    EXPECT_FLOAT_EQ(description.cameras[0].vertical_field_of_view_radians, 0.8f);
    EXPECT_EQ(description.nodes[3].camera, 0);
}

TEST(GLTFSceneImport, ARiggedAssetInTheTreeReportsItsSkin)
{
    SushiEngine::Geometry::GLTFSceneDescription description;
    ASSERT_TRUE(
        SushiEngine::Geometry::import_gltf_scene(asset("rigged_chain.gltf").c_str(), description));
    EXPECT_GT(description.skin_count, 0u);
    EXPECT_FALSE(description.nodes.empty());
}
```

The last case is the one real-file check, and it uses `rigged_chain.gltf`, which does exist. It
proves the importer survives a file produced by a real exporter rather than only the hand-written
fixture. Add `#include <cstdio>` and `#include <fstream>` for `ScratchGLTF`.

The depth-first ordering the second case pins is a decision, not an accident: `append_node` in
Step 5 recurses into each child immediately. If the implementation is changed to a breadth-first
walk, this test fails and that is correct — the parent-before-child guarantee holds either way, but
the plan's consumers index by position and a silent reordering would move them.

- [ ] **Step 3: Register the test and the source**

In `engine/asset/gltf/CMakeLists.txt`, add `source/scene_importer.cpp` to the `SOURCES` list.

In `tests/CMakeLists.txt`, add `integration/test_gltf_scene_import.cpp` to the
`add_executable(sushiengine_functional_tests ...)` source list at line 38 onward, keeping the
surrounding ordering convention. `sushiengine_gltf` is already linked (line 645) and
`SE_TEST_ASSET_DIR` is already defined (line 660), so neither needs adding.

- [ ] **Step 4: [USER CHECKPOINT] Verify the test fails to link**

Hand the user: `se build`
Expected: a link error naming `import_gltf_scene`, because `scene_importer.cpp` is registered but
empty. Wait for their report before continuing.

- [ ] **Step 5: Implement the importer**

Create `engine/asset/gltf/source/scene_importer.cpp` with the Apache header. Model the parse and
teardown on `engine/asset/gltf/source/mesh_importer.cpp:100-120`, which is the established shape in
this module. Do **not** call `cgltf_load_buffers`: no vertex data is read here.

```cpp
#include <SushiEngine/gltf/scene_import.hpp>

#include <cstddef>
#include <cstdint>
#include <vector>

#include <cgltf.h>

namespace SushiEngine
{
    namespace Geometry
    {
        namespace
        {
            GLTFLightKind light_kind(cgltf_light_type type)
            {
                switch (type)
                {
                    case cgltf_light_type_directional:
                        return GLTFLightKind::Directional;
                    case cgltf_light_type_spot:
                        return GLTFLightKind::Spot;
                    default:
                        return GLTFLightKind::Point;
                }
            }

            // Appends `node` and then its children, so a parent is always written before any of
            // its descendants and a consumer can build the tree in one forward pass.
            void append_node(const cgltf_data& data, const cgltf_node& node, std::int32_t parent,
                             GLTFSceneDescription& out)
            {
                GLTFNodeDescription description;
                description.parent = parent;
                description.source_index = std::uint32_t(cgltf_node_index(&data, &node));
                if (node.name != nullptr)
                    description.name = node.name;

                // cgltf leaves whichever form the file used unset, so both are asked for and the
                // decomposed one wins when present.
                cgltf_float matrix[16];
                cgltf_node_transform_local(&node, matrix);
                float translation[3] = {matrix[12], matrix[13], matrix[14]};
                description.translation = Vector3f{translation[0], translation[1], translation[2]};
                if (node.has_rotation)
                {
                    description.rotation = Quaternionf{node.rotation[0], node.rotation[1],
                                                       node.rotation[2], node.rotation[3]};
                }
                if (node.has_scale)
                    description.scale = Vector3f{node.scale[0], node.scale[1], node.scale[2]};
                if (node.has_translation)
                {
                    description.translation =
                        Vector3f{node.translation[0], node.translation[1], node.translation[2]};
                }

                if (node.mesh != nullptr)
                {
                    description.mesh = std::int32_t(cgltf_mesh_index(&data, node.mesh));
                    description.primitive_count = std::uint32_t(node.mesh->primitives_count);
                }
                if (node.camera != nullptr)
                    description.camera = std::int32_t(cgltf_camera_index(&data, node.camera));
                if (node.light != nullptr)
                    description.light = std::int32_t(cgltf_light_index(&data, node.light));
                if (node.skin != nullptr)
                    description.skin = std::int32_t(cgltf_skin_index(&data, node.skin));

                const std::int32_t self = std::int32_t(out.nodes.size());
                out.nodes.push_back(std::move(description));
                for (cgltf_size c = 0; c < node.children_count; ++c)
                    append_node(data, *node.children[c], self, out);
            }
        } // namespace

        bool import_gltf_scene(const char* path, GLTFSceneDescription& out)
        {
            out = GLTFSceneDescription{};
            if (path == nullptr)
                return false;

            cgltf_options options{};
            cgltf_data* data = nullptr;
            if (cgltf_parse_file(&options, path, &data) != cgltf_result_success)
                return false;

            out.material_count = std::uint32_t(data->materials_count);
            out.skin_count = std::uint32_t(data->skins_count);

            out.lights.reserve(data->lights_count);
            for (cgltf_size l = 0; l < data->lights_count; ++l)
            {
                const cgltf_light& source = data->lights[l];
                GLTFLightDescription light;
                if (source.name != nullptr)
                    light.name = source.name;
                light.kind = light_kind(source.type);
                light.color[0] = source.color[0];
                light.color[1] = source.color[1];
                light.color[2] = source.color[2];
                light.intensity = source.intensity;
                light.range = source.range;
                light.spot_inner_cone_radians = source.spot_inner_cone_angle;
                light.spot_outer_cone_radians = source.spot_outer_cone_angle;
                out.lights.push_back(std::move(light));
            }

            out.cameras.reserve(data->cameras_count);
            for (cgltf_size c = 0; c < data->cameras_count; ++c)
            {
                const cgltf_camera& source = data->cameras[c];
                GLTFCameraDescription camera;
                if (source.name != nullptr)
                    camera.name = source.name;
                if (source.type == cgltf_camera_type_orthographic)
                {
                    camera.kind = GLTFCameraKind::Orthographic;
                    camera.orthographic_width = source.data.orthographic.xmag;
                    camera.orthographic_height = source.data.orthographic.ymag;
                    camera.near_plane = source.data.orthographic.znear;
                    camera.far_plane = source.data.orthographic.zfar;
                }
                else
                {
                    camera.kind = GLTFCameraKind::Perspective;
                    camera.vertical_field_of_view_radians = source.data.perspective.yfov;
                    camera.near_plane = source.data.perspective.znear;
                    camera.far_plane = source.data.perspective.has_zfar
                                           ? source.data.perspective.zfar
                                           : 0.0f;
                }
                out.cameras.push_back(std::move(camera));
            }

            // The default scene when the file names one, every root otherwise: a file with no
            // `scene` member is legal and still has nodes worth importing.
            const cgltf_scene* scene = data->scene != nullptr ? data->scene
                                       : (data->scenes_count > 0 ? &data->scenes[0] : nullptr);
            if (scene != nullptr)
            {
                for (cgltf_size n = 0; n < scene->nodes_count; ++n)
                    append_node(*data, *scene->nodes[n], -1, out);
            }
            else
            {
                for (cgltf_size n = 0; n < data->nodes_count; ++n)
                {
                    if (data->nodes[n].parent == nullptr)
                        append_node(*data, data->nodes[n], -1, out);
                }
            }

            cgltf_free(data);
            return !out.nodes.empty();
        }
    } // namespace Geometry
} // namespace SushiEngine
```

- [ ] **Step 6: [USER CHECKPOINT] Verify the tests pass**

Hand the user:

```bash
se build
se test --suite functional
```

Expected: `GLTFSceneImport.*` all pass. Do not claim they pass before the user reports it.

- [ ] **Step 7: Update the module README**

In `engine/asset/gltf/README.md`, add a row to the public-surface table:

```markdown
| `scene_import.hpp` | A glTF file's node graph, exactly as the file states it: one entry per node with its name, parent, local transform, and the mesh, camera, light or skin it carries. |
```

Add `scene_importer.cpp` to the sentence listing the module's sources, and replace the "Tests" line
about `mesh_import.hpp` having no coverage with the honest current state: `scene_import.hpp` is
covered by `tests/integration/test_gltf_scene_import.cpp`, and `mesh_import.hpp` still is not.

- [ ] **Step 8: Run the documentation checks and commit**

```bash
python tools/documentation/check_documentation_length.py
python tools/documentation/check_module_documentation.py
git add engine/asset/gltf tests/integration/test_gltf_scene_import.cpp tests/CMakeLists.txt
git commit -m "feat(gltf): read a glTF file's node graph without its geometry"
```
