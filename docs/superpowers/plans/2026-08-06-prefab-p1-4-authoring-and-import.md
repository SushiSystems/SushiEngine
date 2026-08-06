# Prefab P1 Implementation Plan — part one, the asset and the instance

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development
> (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use
> checkbox (`- [ ]`) syntax for tracking.

**Goal:** Save an entity subtree as a `.sushiprefab` asset, place instances of it, and rebuild stale
instances when a scene is opened.

**Architecture:** A prefab file is one rooted slice of the entity array `capture_scene` already
produces. Task 1 extracts that per-entity record into functions so the prefab serializer calls them
rather than copying them. A scene stores an instance expanded, plus one component on its root naming
the prefab and the revision it was built from. A refresh pass runs in `load_scene` — beside
`resolve_scene_assets`, which already sits there for the same reason — and never in `apply_scene`,
which is undo's path.

**Tech Stack:** C++17, nlohmann_json, GoogleTest, CMake with `sushiengine_add_module`.

**Spec:** `docs/design/prefab_system.md`. Section references (§N) point into it.

## Global Constraints

- **C++17 only.** No C++20/23 facilities.
- **Allman braces**, namespaces included; nested namespaces written out, never `namespace A::B`.
- **Naming:** types `PascalCase`, functions and variables `snake_case`, members trailing underscore,
  constants `UPPER_SNAKE`, namespaces `PascalCase`.
- **No abbreviations in any identifier or in prose.** Acronyms stay fully upper-case.
- **Every CMake target, function and option starts with `sushiengine_`.**
- **Every new source file carries the Apache 2.0 header**, copied verbatim from a neighbour.
- **Every public function carries Doxygen**: `@brief` (why it exists), one line on the mechanism
  when it is not obvious, `@param` for each parameter, `@return`.
- **No historical references in comments**, no separator comments.
- **Prose in `docs/` obeys `docs/documentation-style-guide.md`**: 100-column lines, present tense,
  no marketing, honest about gaps, every path real and every link resolving.
- **Never invoke `cmake` or `ninja` directly.** Only the `se` CLI.

## Build and test policy — read before Task 1

**The implementing agent does not run builds or tests.** Do not start any process that runs `se`,
`cmake`, `ninja` or `ctest`, in the foreground or the background, and do not write or edit any build
configuration file, including `cli/config.local.toml`. This machine cannot carry a build; the user
runs every one. Steps marked **[USER CHECKPOINT]** are handed to the user with the exact command and
the expected output.

**Never report a test as passing that you did not see pass.** Writing the test is the agent's job;
observing it fail and then pass is the user's.

**Test suite names must be prefixed** `Unit_`, `Integration_` or `Regression_`.
`tests/CMakeLists.txt:665-676` registers `gtest_discover_tests` three times filtering on exactly
those, so a suite without a prefix is discovered by nothing and the run reports green having
executed zero of its cases. The suite currently stands at **1469 tests, all passing**; anything
added must raise that count.

Checks the agent runs itself — pure Python, no compiler:

```bash
python tools/documentation/check_documentation_length.py
python tools/documentation/check_module_documentation.py
python tools/layering/check_include_layering.py
```

Pre-existing and not yours: four changelog bullet-length warnings, and three `world -> presentation`
layering notes.

## Where this plan sits

This is **part four of four**: the gestures that make the previous three reachable. It consumes all
of them and must not start until part three is committed — every task here calls a function one of
them produced.

The others, in order: `2026-08-06-prefab-p1-1-entity-record.md`,
`2026-08-06-prefab-p1-2-prefab-document.md`, `2026-08-06-prefab-p1-3-instance-and-refresh.md`.
Task numbers run across all four, so "Task 2" below is part two's task.

Split into four because a plan file obeys `docs/documentation-style-guide.md`'s 900-line ceiling.

## The user interface rule this part is shaped by

**The interface is written unlinked first, reviewed, and only then wired.** Task 5 draws every
control this part adds and wires none of them; it ends at a review gate the user answers before
Task 6 begins. This is a standing instruction, not a preference for this feature, and the reason is
that placement and wording are cheap to change in a build that does nothing and expensive to change
in one that does.

**Nothing unlinked reaches `main`.** Task 5's commit stays on the branch until Task 8 wires the last
of it. `docs/design/editor_ux_overhaul.md`'s wire-or-remove rule is the same rule read from the
other end: a control that advertises a capability the build does not have is exactly what it
forbids, so the window between Task 5 and Task 8 is the only place an unwired control may exist.

---

## File structure

**Modified:**

| Path | Change |
|---|---|
| `applications/editor/source/project/project_panel.cpp` | A drop target for a Hierarchy entity: writes a `.sushiprefab`. |
| `applications/editor/source/ui/viewport_panel.cpp` | A drop target for an asset path: places a model. |
| `applications/editor/source/scene/inspector_panel.cpp` | A Prefab section on an instance root. |
| `applications/editor/source/core/editor_context.hpp` | The fields the three share while unwired. |
| `applications/editor/CMakeLists.txt` | Link `sushiengine_serialization` if it is not linked already. |
| `engine/world/CMakeLists.txt` | Add the `model_import` subdirectory. |
| `tests/CMakeLists.txt` | Register the new test file. |
| `docs/reference/changelog.md` | An `Added` bullet. |

**Created:**

| Path | Responsibility |
|---|---|
| `engine/world/model_import/include/SushiEngine/model_import/prefab_output.hpp` | A glTF file on disk becomes a `.sushiprefab` beside it. |
| `engine/world/model_import/source/prefab_output.cpp` | Import, plan, build, capture, write. |
| `engine/world/model_import/CMakeLists.txt`, `README.md` | The new module. |
| `tests/integration/test_model_prefab_output.cpp` | That output, headless. |

A new module rather than a function added to an existing one, because the import path cannot live
where the planner does: `engine/asset/model` is asset-tier and `prefab_revision` and `IWorldEditor`
are both world-tier. Task 7 states the finding in full.

The three editor panels carry no test. They are ImGui immediate-mode drawing over functions parts
one to three already test headlessly; their gate is the review in Task 5 and the user running
`se editor`.

---

## Task 5: Every control, drawn and wired to nothing

**Files:**
- Modify: `applications/editor/source/core/editor_context.hpp`,
  `applications/editor/source/project/project_panel.cpp`,
  `applications/editor/source/ui/viewport_panel.cpp`,
  `applications/editor/source/scene/inspector_panel.cpp`
- Test: none. The gate is the user's review.

**Interfaces:**
- Consumes: from parts one to three, only Task 3's `has_prefab_instance` and `prefab_instance`,
  and only in Step 4. Steps 2 and 3 deliberately call nothing, so a reviewer cannot mistake a
  drawn control for a working one.
- Produces: `EditorContext::prefab_ui` — the struct below. Tasks 6 and 8 replace its fields'
  readers with real calls and Task 8 deletes it.

- [ ] **Step 1: Add the shared state**

In `applications/editor/source/core/editor_context.hpp`, beside the other panel state:

```cpp
/**
 * @brief What the prefab gestures would do, while they do nothing.
 *
 * Every field here is written by a gesture and read only by the status line reporting it. The
 * struct exists so the panels can be reviewed for placement and wording before any of them
 * writes a file, and it is deleted when the last of them is wired.
 */
struct PrefabInterfacePreview
{
    /** @brief The `.sushiprefab` the last Hierarchy-to-Project drop would have written. */
    std::string authored_path;

    /** @brief The model the last viewport drop would have placed. */
    std::string placed_asset;
};

/** @brief See @ref PrefabInterfacePreview. Removed by Task 8. */
PrefabInterfacePreview prefab_ui;
```

- [ ] **Step 2: Draw the authoring drop target**

In `project_panel.cpp`, after the file grid and before the panel's `ImGui::End()`, make the whole
browser a target the way `hierarchy_panel.cpp:351` makes its window one:

```cpp
// The whole browser rather than one tile: the gesture is "put this entity in this folder", and
// a target the size of an icon makes the user aim at something that is not the destination.
if (ImGui::BeginDragDropTargetCustom(ImGui::GetCurrentWindow()->InnerRect,
                                     ImGui::GetID("##prefab_author_target")))
{
    if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("HIERARCHY_ENTITY"))
    {
        const Simulation::EntityId dragged =
            *static_cast<const Simulation::EntityId*>(payload->Data);
        // unique_child_path (:156) is what the wiring will use, so the preview shows the name
        // the user will actually get, " (1)" suffix and all.
        context.prefab_ui.authored_path =
            unique_child_path(current, world->name(dragged), ".sushiprefab").string();
    }
    ImGui::EndDragDropTarget();
}

if (!context.prefab_ui.authored_path.empty())
{
    ImGui::Separator();
    ImGui::TextDisabled("Would write: %s", context.prefab_ui.authored_path.c_str());
    ImGui::SameLine();
    if (ImGui::SmallButton("Clear##prefab_preview"))
        context.prefab_ui.authored_path.clear();
}
```

`world` is `world_of(context)`; return early when it is null, as every panel does.

- [ ] **Step 3: Draw the viewport drop target**

In `viewport_panel.cpp`, immediately after the `ImGui::Image` at `:728`. A drag-drop target binds to
the last item drawn, so it has to come before anything is drawn over the view:

```cpp
// Right after the Image and before the overlays: BeginDragDropTarget binds to the last item,
// and a toolbar drawn in between would take the drop instead of the view.
std::string dropped_asset;
if (accept_asset_drop(dropped_asset))
{
    const std::string extension = std::filesystem::path(dropped_asset).extension().string();
    // A model and nothing else. A texture dropped on the view has no meaning here, and
    // accepting it would place an empty entity the user then has to find and delete.
    if (extension == ".gltf" || extension == ".glb")
        context.prefab_ui.placed_asset = dropped_asset;
}
if (!context.prefab_ui.placed_asset.empty())
{
    ImGui::SetCursorPos(ImVec2(12.0f, 34.0f));
    ImGui::TextDisabled("Would place: %s",
                        std::filesystem::path(context.prefab_ui.placed_asset)
                            .filename()
                            .string()
                            .c_str());
}
```

`ViewportPanel` may not take an `EditorContext` today. Read its `draw` signature first and thread
one in the way the other panels receive it, rather than reaching for a global.

- [ ] **Step 4: Draw the Prefab section on an instance**

In `inspector_panel.cpp`, beside the other component sections. This is the one control here that is
not a preview — it displays state parts one to three already produce, so **it stays unchanged after
Tasks 6 and 8** rather than being replaced.

```cpp
if (world->has_prefab_instance(selected))
{
    if (ImGui::CollapsingHeader("Prefab", ImGuiTreeNodeFlags_DefaultOpen))
    {
        const Simulation::PrefabInstanceParameters link = world->prefab_instance(selected);
        ImGui::TextDisabled("Source");
        ImGui::SameLine();
        ImGui::TextUnformatted(link.path.c_str());
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("%s", link.path.c_str());

        // The revision is shown because it is the only thing that explains why a subtree
        // changed on open. An artist who cannot see it has no way to tell a refresh from
        // someone else's edit.
        ImGui::TextDisabled("Revision");
        ImGui::SameLine();
        ImGui::TextUnformatted(link.revision.c_str());

        if (!std::filesystem::exists(link.path))
            ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.2f, 1.0f),
                               "Unlinked: this prefab was not found.");
    }
}
```

No "Override", no "Apply", no "Revert" — not even disabled. §8's last bullet forbids a control that
advertises a capability the build does not have, and every one of those belongs to a later phase.

The unlinked warning is the consumer of part three's returned paths: a missing prefab keeps its
component so the link survives a file that is merely not pulled yet, and this line is what tells the
user which of their instances are in that state.

- [ ] **Step 5: [USER CHECKPOINT] Review the interface**

Hand the user:

```bash
se build
se editor
```

Ask them to try three things and say whether the placement and the wording are right:

1. Drag an entity from the Hierarchy onto the Project browser. A line appears under the grid naming
   the file that would be written. **No file is created.**
2. Drag a `.gltf` from the Project browser onto the Scene view. A line appears in the view naming
   the model that would be placed. **No entity is created.** A `.png` dragged the same way does
   nothing, which is deliberate.
3. Select an entity carrying a prefab link and look at the Inspector's Prefab section. (Nothing
   creates one yet, so this is only checkable after Task 6 — say so rather than letting them hunt
   for it.)

**Do not proceed to Task 6 until they answer.** If they want anything moved or reworded, change it
here, where nothing depends on it.

- [ ] **Step 6: Commit**

```bash
git add applications/editor
git commit -m "feat(editor): draw the prefab gestures before wiring them"
```

This commit stays on the branch. It must not reach `main` on its own.

---

## Task 6: Wire the authoring gesture

**Files:**
- Modify: `applications/editor/source/project/project_panel.cpp`,
  `applications/editor/CMakeLists.txt`
- Test: none new. `capture_prefab` is tested headlessly by Task 2.

**Interfaces:**
- Consumes: Task 2's `capture_prefab`; Task 3's `set_prefab_instance`.
- Produces: nothing other tasks call.

- [ ] **Step 1: Replace the preview with the write**

The drop body from Task 5 Step 2 becomes:

```cpp
if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("HIERARCHY_ENTITY"))
{
    const Simulation::EntityId dragged =
        *static_cast<const Simulation::EntityId*>(payload->Data);
    const fs::path target = unique_child_path(current, world->name(dragged), ".sushiprefab");
    const nlohmann::json document = Scene::capture_prefab(*world, dragged);

    std::ofstream file(target.string());
    if (!file || !(file << document.dump(2)))
    {
        // Report through whatever channel this file already uses; read the Cooking Override
        // modal's failure path in this same file and match it rather than adding a second.
    }
    else
    {
        // §6: the dragged entity becomes an instance of what it just wrote, so what stays
        // selected is the thing the user edits next. A gesture that left the original a plain
        // subtree would make "did that work?" unanswerable without opening the folder.
        context.history.record(*world);
        Simulation::PrefabInstanceParameters link;
        link.path = target.string();
        link.revision = document.value("revision", std::string());
        world->set_prefab_instance(dragged, link);
    }
}
```

Delete `PrefabInterfacePreview::authored_path` and the status line that read it.

`unique_child_path` (`:156-163`) rather than an overwrite: §6 requires the `" (n)"` convention, and
silently replacing a prefab that other scenes instantiate would rebuild all of them on their next
open with content their author never chose.

- [ ] **Step 2: Link the module if it is not linked**

`project_panel.cpp` now includes `prefab_serializer.hpp`. Check whether
`applications/editor/CMakeLists.txt` already lists `sushiengine_serialization`; add it if not.

- [ ] **Step 3: [USER CHECKPOINT] Verify by hand**

```bash
se build
se editor
```

Ask the user to confirm, in this order:

1. Dragging an entity onto the browser creates a `.sushiprefab` that appears in the grid.
2. The dragged entity's Inspector now shows the Prefab section, naming that file.
3. Dragging a second entity of the same name produces `Name (1).sushiprefab`, not an overwrite.
4. Undo restores the entity to a plain subtree. **The file stays.** Undo is a scene operation, and
   deleting a file the user can see would be an edit they cannot undo back.

Point 4 is a deliberate asymmetry and reads like a bug if it arrives unannounced, so state it to
them rather than waiting to be asked.

- [ ] **Step 4: Commit**

```bash
git add applications/editor
git commit -m "feat(editor): save a Hierarchy subtree as a prefab by dragging it to a folder"
```

---

## Task 7: The model importer writes a prefab

This is §7, and it closes the gap `model_import.md` §4.3 left open.

**Read this first.** `plan_model_instantiation` has **no production caller** — a grep finds it in
`instantiation_plan.hpp:128`, its own source, and `tests/unit/test_model_instantiation_plan.cpp`,
and nowhere else. The planner exists and nothing imports. So this task *writes* the import path, it
does not modify one.

It cannot live in `engine/asset/model`. `prefab_revision` and `IWorldEditor` are both world-tier,
`model` is asset-tier, and `sushiengine_add_module` raises `FATAL_ERROR` at configure time on an
asset module that depends on world. That constraint is doing real work here: the planner is a pure
function, which is why 27 tests run it without a simulation, and writing a prefab is not. They
belong in different modules because they are different kinds of thing.

**Files:**
- Create: `engine/world/model_import/CMakeLists.txt`, `README.md`,
  `include/SushiEngine/model_import/prefab_output.hpp`, `source/prefab_output.cpp`
- Modify: `engine/world/CMakeLists.txt` (add the subdirectory), `tests/CMakeLists.txt`
- Test: `tests/integration/test_model_prefab_output.cpp`

**Interfaces:**
- Consumes: `Geometry::import_gltf_scene`, `Model::load_model_import_settings`,
  `Model::plan_model_instantiation`, `Simulation::create_simulation`, and Task 2's
  `Scene::capture_prefab`.
- Produces: `ModelImport::instantiate_plan` and `ModelImport::write_model_prefab`. Task 8 relies on
  the file the latter writes, not on the function.

- [ ] **Step 1: Create the module**

`engine/world/model_import/CMakeLists.txt`:

```cmake
# model_import — the path from a glTF file on disk to a `.sushiprefab` beside it. It is the one
# place that knows all four of import, settings, planning and prefab output, which is why it sits
# above every module that knows exactly one of them.

sushiengine_add_module(NAME model_import LAYER world
    SOURCES
        source/prefab_output.cpp
    # A caller names ModelImportReport to read what the import could not carry across.
    PUBLIC_DEPENDS model
    # The pieces the path is assembled from; none appears in this module's header.
    PRIVATE_DEPENDS gltf serialization simulation)

target_include_directories(sushiengine_model_import
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
```

Add `add_subdirectory(model_import)` to `engine/world/CMakeLists.txt` beside its neighbours, and
write `README.md` following the shape `check_module_documentation.py` enforces — copy the structure
from `engine/world/serialization/README.md`.

- [ ] **Step 2: Write the header**

`include/SushiEngine/model_import/prefab_output.hpp`, Apache header copied from a neighbour:

```cpp
#pragma once

/**
 * @file prefab_output.hpp
 * @brief A glTF file on disk becomes a `.sushiprefab` beside it.
 *
 * The import path proper: read the file, read its `.meta`, plan what entities it becomes, build
 * them, and write the result as a prefab. Split from `Model::plan_model_instantiation` because
 * that function is pure and this one is not — the planner is tested without a simulation and
 * lives an entire tier lower for exactly that reason.
 */

#include <string>

#include <SushiEngine/model/instantiation_plan.hpp>
#include <SushiEngine/simulation/simulation.hpp>

namespace SushiEngine
{
    namespace ModelImport
    {
        /**
         * @brief Creates @p plan's entities in @p world and returns the subtree's root.
         *
         * Entities are created in plan order, which the planner guarantees is parent-before-child,
         * so a parent always exists when its child is parented.
         *
         * @param world The world to populate.
         * @param plan  What to create.
         * @param source_path The asset the plan came from; becomes each Shape's `mesh_path`, which
         *     is what `resolve_scene_assets` re-derives the live mesh handle from on load.
         * @return The root entity, or `NULL_ENTITY` when the plan is empty.
         */
        SushiEngine::Simulation::EntityId instantiate_plan(
            SushiEngine::Simulation::IWorldEditor& world,
            const SushiEngine::Model::ModelInstantiationPlan& plan,
            const std::string& source_path);

        /**
         * @brief Imports @p asset_path and writes its hierarchy as `<asset_path>.sushiprefab`.
         *
         * The whole path with the extension appended, matching `.meta`'s convention, so
         * `models/Car.gltf` yields `models/Car.gltf.sushiprefab` and a `.glb` of the same stem
         * cannot collide with it.
         *
         * @param asset_path Path to a `.gltf` or `.glb`.
         * @param report     Receives the import's counts and warnings; overwritten.
         * @return False when the file cannot be read, holds no node, or the prefab cannot be
         *     written. The prefab is left untouched in every failing case rather than truncated.
         */
        bool write_model_prefab(const std::string& asset_path,
                                SushiEngine::Model::ModelImportReport& report);
    } // namespace ModelImport
} // namespace SushiEngine
```

- [ ] **Step 3: Write the failing tests**

Create `tests/integration/test_model_prefab_output.cpp` with the Apache header and a file comment.
Copy the `ScratchGLTF` class and the inline-glTF-string idiom from
`tests/integration/test_gltf_scene_import.cpp:40-106` verbatim — that file writes its fixtures as
string literals and removes them in a destructor, so a run leaves nothing behind and two runs cannot
see each other's files.

```cpp
namespace
{
    // One root, one child, one grandchild. No buffers and no meshes: what this file pins is the
    // shape of the output, and a primitive would need vertex data that says nothing about it.
    const char* NESTED_GLTF = R"json({
      "asset": { "version": "2.0" },
      "scene": 0,
      "scenes": [ { "nodes": [ 0 ] } ],
      "nodes": [
        { "name": "Body", "children": [ 1 ], "translation": [1, 0, 0] },
        { "name": "Wheel", "children": [ 2 ], "translation": [0, 2, 0] },
        { "name": "Tire", "translation": [0, 0, 3] }
      ]
    })json";

    /** @brief The prefab path write_model_prefab produces for @p asset. */
    std::string prefab_path_of(const std::string& asset)
    {
        return asset + ".sushiprefab";
    }

    /** @brief Reads a written prefab, or an empty object when it is not there. */
    nlohmann::json read_document(const std::string& path)
    {
        std::ifstream file(path);
        if (!file)
            return nlohmann::json::object();
        nlohmann::json document;
        try
        {
            file >> document;
        }
        catch (const nlohmann::json::parse_error&)
        {
            return nlohmann::json::object();
        }
        return document;
    }
} // namespace

TEST(Integration_ModelPrefabOutput, ImportingAModelWritesAPrefabBesideIt)
{
    const ScratchGLTF file("sushiengine_prefab_output_beside.gltf", NESTED_GLTF);
    const std::string prefab = prefab_path_of(file.path());
    std::error_code error;
    std::filesystem::remove(prefab, error);

    SushiEngine::Model::ModelImportReport report;
    ASSERT_TRUE(SushiEngine::ModelImport::write_model_prefab(file.path(), report));

    const nlohmann::json document = read_document(prefab);
    ASSERT_TRUE(document.contains("entities"));
    ASSERT_TRUE(document.contains("revision"));
    EXPECT_FALSE(document["revision"].get<std::string>().empty());
    ASSERT_EQ(document["entities"].size(), 3u);
    // Entry 0 is the root and is a root: an instance placed from this file must hang under
    // whatever the scene gives it, not under an index into a document that is not there.
    EXPECT_EQ(document["entities"].front().value("parent", 0), -1);
    EXPECT_EQ(document["entities"].front().value("name", std::string()), "Body");

    std::filesystem::remove(prefab, error);
}

TEST(Integration_ModelPrefabOutput, TheHierarchySurvivesIntoThePrefab)
{
    const ScratchGLTF file("sushiengine_prefab_output_nested.gltf", NESTED_GLTF);
    const std::string prefab = prefab_path_of(file.path());
    std::error_code error;
    std::filesystem::remove(prefab, error);

    SushiEngine::Model::ModelImportReport report;
    ASSERT_TRUE(SushiEngine::ModelImport::write_model_prefab(file.path(), report));

    const nlohmann::json entities = read_document(prefab)["entities"];
    ASSERT_EQ(entities.size(), 3u);

    // Body -> Wheel -> Tire, and not three siblings. This is the user-visible claim of the
    // whole model-import effort — "does the engine support a model's children" — and a writer
    // that flattened the tree would pass every other case in this file.
    const auto index_of = [&entities](const char* name)
    {
        for (std::size_t i = 0; i < entities.size(); ++i)
            if (entities[i].value("name", std::string()) == name)
                return static_cast<int>(i);
        return -1;
    };
    const int body = index_of("Body");
    const int wheel = index_of("Wheel");
    const int tire = index_of("Tire");
    ASSERT_NE(body, -1);
    ASSERT_NE(wheel, -1);
    ASSERT_NE(tire, -1);
    EXPECT_EQ(entities[static_cast<std::size_t>(wheel)].value("parent", -1), body);
    EXPECT_EQ(entities[static_cast<std::size_t>(tire)].value("parent", -1), wheel);

    // The transforms are local, so the grandchild keeps its own (0, 0, 3) rather than an
    // accumulated one — a writer that stored world transforms would put 1, 2, 3 here.
    const nlohmann::json& deep_position =
        entities[static_cast<std::size_t>(tire)]["position"];
    EXPECT_NEAR(deep_position[0].get<double>(), 0.0, 1e-5);
    EXPECT_NEAR(deep_position[1].get<double>(), 0.0, 1e-5);
    EXPECT_NEAR(deep_position[2].get<double>(), 3.0, 1e-5);

    std::filesystem::remove(prefab, error);
}

TEST(Integration_ModelPrefabOutput, ImportingTwiceWithNoChangeKeepsTheRevision)
{
    const ScratchGLTF file("sushiengine_prefab_output_stable.gltf", NESTED_GLTF);
    const std::string prefab = prefab_path_of(file.path());
    std::error_code error;
    std::filesystem::remove(prefab, error);

    SushiEngine::Model::ModelImportReport report;
    ASSERT_TRUE(SushiEngine::ModelImport::write_model_prefab(file.path(), report));
    const std::string first = read_document(prefab).value("revision", std::string("a"));
    ASSERT_TRUE(SushiEngine::ModelImport::write_model_prefab(file.path(), report));
    const std::string second = read_document(prefab).value("revision", std::string("b"));

    // The half that matters more. An unchanged asset must not churn its revision, or part
    // three's refresh rebuilds every imported instance in every scene, every time one opens.
    EXPECT_EQ(first, second);

    std::filesystem::remove(prefab, error);
}

TEST(Integration_ModelPrefabOutput, ChangingTheSettingsChangesTheRevision)
{
    const ScratchGLTF file("sushiengine_prefab_output_settings.gltf", NESTED_GLTF);
    const std::string prefab = prefab_path_of(file.path());
    const std::string meta = std::string(file.path()) + ".meta";
    std::error_code error;
    std::filesystem::remove(prefab, error);
    std::filesystem::remove(meta, error);

    SushiEngine::Model::ModelImportReport report;
    ASSERT_TRUE(SushiEngine::ModelImport::write_model_prefab(file.path(), report));
    const std::string before = read_document(prefab).value("revision", std::string("a"));

    // §7's claim that reimport is not a feature anyone writes: a setting change changes the
    // plan, which changes the prefab, which changes the revision, which is what makes part
    // three rebuild every placed instance.
    SushiEngine::Model::ModelImportSettings settings;
    settings.scale_factor = 2.0f;
    ASSERT_TRUE(SushiEngine::Model::save_model_import_settings(file.path(), settings));

    ASSERT_TRUE(SushiEngine::ModelImport::write_model_prefab(file.path(), report));
    const std::string after = read_document(prefab).value("revision", std::string("b"));
    EXPECT_NE(before, after);

    std::filesystem::remove(prefab, error);
    std::filesystem::remove(meta, error);
}

TEST(Integration_ModelPrefabOutput, AMissingAssetFailsAndWritesNothing)
{
    const std::string absent = "sushiengine_prefab_output_absent.gltf";
    const std::string prefab = prefab_path_of(absent);
    std::error_code error;
    std::filesystem::remove(prefab, error);

    SushiEngine::Model::ModelImportReport report;
    EXPECT_FALSE(SushiEngine::ModelImport::write_model_prefab(absent, report));
    // Nothing written, rather than an empty prefab: a placement made from a truncated file
    // would produce nothing and report success, which is worse than a failure that says so.
    EXPECT_FALSE(std::filesystem::exists(prefab));
}
```

`ModelImportSettings::scale_factor` is used above as the setting that is changed. Check the field's
real name in `engine/asset/model/include/SushiEngine/model/import_settings.hpp` before writing this
case, and use whichever scalar setting is there — the case is about *a* setting changing, not about
that one.

- [ ] **Step 4: Register the test**

Add `integration/test_model_prefab_output.cpp` to `tests/CMakeLists.txt`, and link
`sushiengine_model_import` into the test executable.

- [ ] **Step 5: [USER CHECKPOINT] Verify they fail**

`se build` — expect a link error naming `write_model_prefab`.

- [ ] **Step 6: Implement**

`instantiate_plan` walks `plan.entities` in order. For each:

```cpp
const EntityId id = entity.component == Model::PlannedComponent::Camera
                        ? world.create_camera(entity.name)
                        : world.create(entity.name);
created.push_back(id);
if (entity.parent >= 0)
    world.set_parent(id, created[static_cast<std::size_t>(entity.parent)]);

Simulation::EntityTransform transform;
transform.position = Vector3{entity.translation.x, entity.translation.y, entity.translation.z};
transform.rotation = Quaternion{entity.rotation.x, entity.rotation.y, entity.rotation.z,
                                entity.rotation.w};
transform.scale = Vector3{entity.scale.x, entity.scale.y, entity.scale.z};
world.set_transform(id, transform);
```

`PlannedEntity`'s transform fields are `Vector3f`/`Quaternionf` and `EntityTransform`'s are the
`Scalar` aliases, so each component widens explicitly. Write the conversion out rather than relying
on an implicit one, so the widening is visible where a later reader looks for it.

Then the component, by `entity.component`:
- `None` — `world.set_has_renderer(id, false)`. A pivot is not drawn.
- `Shape` — `set_has_shape(id, true)`, then `shape_parameters` with `mesh_path = source_path` and
  `kind` set to whatever `ShapeParameters` uses for an imported mesh. Leave `mesh` at its invalid
  default: the live handle is re-derived from the path by `resolve_scene_assets`
  (`scene_serializer.cpp:1509-1530`), and a number written here would name nothing in the session
  that opens the file.
- `Light` — `set_has_light(id, true)` and copy the description's `lights[entity.light]` into
  `LightParameters`.
- `Camera` — `set_camera_parameters` from `cameras[entity.camera]`.

`write_model_prefab` is then the four steps in order:

```cpp
Geometry::GLTFSceneDescription description;
if (!Geometry::import_gltf_scene(asset_path.c_str(), description))
    return false;

Model::ModelImportSettings settings;
// A missing .meta is the ordinary case for an asset nobody has configured, not a failure:
// load leaves `settings` at its defaults and the import proceeds with them.
(void)Model::load_model_import_settings(asset_path, settings);

const std::string stem = std::filesystem::path(asset_path).stem().string();
const Model::ModelInstantiationPlan plan =
    Model::plan_model_instantiation(description, settings, stem, report);
if (plan.entities.empty())
    return false;
```

then build the plan in a scratch simulation and capture it:

```cpp
// A scratch world rather than emitting the entity records directly. The record shape has
// exactly one writer (part one's `write_entity_record`) and this keeps it that way: a field
// added to an entity is carried into imported prefabs the day it is added, with no edit here.
// It costs one simulation per import, which is an authoring action, not a frame.
const auto simulation = Simulation::create_simulation();
if (simulation == nullptr)
    return false;
Simulation::IWorldEditor& world = simulation->world();
for (const Simulation::EntityId id : world.entities())
    world.destroy(id);

const Simulation::EntityId root = instantiate_plan(world, plan, asset_path);
if (root == Simulation::NULL_ENTITY)
    return false;
const nlohmann::json document = Scene::capture_prefab(world, root);
```

Write it to `asset_path + ".sushiprefab"`, and return whether the stream succeeded. Write to a
temporary beside the target and rename over it, or check the stream before it is closed — a
half-written prefab is a file every scene referencing it will fail to parse on its next open.

`AMissingAssetFailsAndWritesNothing` is what holds this last point: every early return above happens
before the file is touched.

- [ ] **Step 7: [USER CHECKPOINT] Verify**

```bash
se build
se test --suite functional
```
Expected: the five `Integration_ModelPrefabOutput` cases pass.

- [ ] **Step 8: Commit**

```bash
python tools/documentation/check_module_documentation.py
python tools/layering/check_include_layering.py
git add engine/world/model_import engine/world/CMakeLists.txt tests
git commit -m "feat(model-import): write an imported model's hierarchy as a prefab beside the asset"
```

---

## Task 8: Wire the viewport drop

**Files:**
- Modify: `applications/editor/source/ui/viewport_panel.cpp`,
  `applications/editor/source/core/editor_context.hpp`
- Test: none new.

**Interfaces:**
- Consumes: Task 2's `apply_prefab`; Task 3's `set_prefab_instance`; Task 7's output file.

- [ ] **Step 1: Replace the preview with the placement**

The drop body from Task 5 Step 3 becomes: append `.sushiprefab` to the dropped asset path, read and
parse the document, `apply_prefab(world, document, NULL_ENTITY)`, and set the new root's
`PrefabInstanceParameters` to that path and the document's revision.

Two cases the body must handle, both ordinary rather than exceptional:

- **No prefab beside the asset** — the model has not been imported yet. Report it and place
  nothing. Do not import inline: importing is not something to do inside a mouse-release handler,
  and a gesture that blocked the interface for a large model would read as a hang.
- **The drop lands while no simulation is attached.** `world_of(context)` returns null before the
  first scene exists; return early, as every panel does.

Record undo with `context.history.record(*world)` **before** creating anything, so the placement
undoes as one step rather than leaving a partial subtree behind.

- [ ] **Step 2: Delete `PrefabInterfacePreview`**

Every field now has a real consumer, so the struct and its `EditorContext` member go. Grep for
`prefab_ui` and confirm nothing reads it before deleting.

**This is the step that makes Task 5's commit safe to merge.** Until it lands, the branch carries
controls that draw and do nothing, and the branch must not be merged in that state.

- [ ] **Step 3: [USER CHECKPOINT] Verify by hand**

```bash
se build
se editor
```

Ask the user to confirm:

1. Dragging an imported `.gltf` onto the Scene view places its hierarchy, children included.
2. The placed root's Inspector shows the Prefab section naming the model's `.sushiprefab`.
3. Undo removes the whole placement in one step.
4. Dragging a `.gltf` that has never been imported reports that and places nothing.

- [ ] **Step 4: Changelog and commit**

Add under `## [Unreleased]` / `Added`, at most 240 characters:

```markdown
- Added prefab authoring: drag an entity to a project folder to save it as a `.sushiprefab`, and
  drag an imported model into the scene to place an instance of it.
```

```bash
python tools/documentation/check_documentation_length.py
git add applications/editor docs/reference/changelog.md
git commit -m "feat(editor): place an imported model by dragging it into the scene"
```

---

## What P1 does not build

- **Overrides.** An edit inside an instance does not survive a refresh, and no control suggests it
  might (§2, §8).
- **Nested prefabs.** A prefab authored from a subtree containing an instance flattens it. §8 asks
  that this be reported by name; the report belongs to the phase that can do something about it,
  and Task 6 does not draw one. Say so when handing over, rather than leaving it to be discovered.
- **A rebuild count stated before it runs.** §8 asks for this "on the control that triggers it".
  P1 has no such control — the refresh runs when a scene opens and nowhere else — so the bullet has
  nothing to attach to and no control is invented to give it one. It becomes real in the phase that
  adds a manual refresh, and it is listed here so a reader of §8 can see it was read and not
  missed.
- **Prefab edit mode**, **runtime instantiation**, **variants.** Later phases (§10).
