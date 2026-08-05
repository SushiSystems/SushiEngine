# Static Mesh Authoring Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development
> (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use
> checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let an artist place an imported glTF as a plain, non-deforming visual object in the
scene — today the only doors into a glTF are `Crowd` (rigged characters) and the physics cooking
pipeline (colliders, soft bodies, node-beam shells).

**Architecture:** Extend the existing Renderer component (`ShapeParameters`) with an imported-mesh
field, following the exact discriminator convention `Render::MeshInstance` already uses (mesh
present overrides `kind`). Wire that field through the one extraction site
(`RuntimeSimulation`'s instance loop) and the one copy site (`main.cpp`) that currently drop it —
every render pass downstream already consumes `MeshInstance::mesh` and needs no change. No new ECS
component, no change to `PrimitiveKind` (shared with `ColliderParameters` — see spec §3), no change
to the cooking pipeline.

**Tech Stack:** C++17, the SushiEngine ECS/authoring layer (`engine/world/simulation`,
`engine/world/serialization`), the Vulkan renderer's already-built `MeshInstance::mesh` path
(`engine/presentation/render`), Dear ImGui (`applications/editor`), GoogleTest (`se test`).

## Global Constraints

- **Spec:** `docs/design/static_mesh_authoring.md`, approved and committed at `56c0f1cb`. Every
  task below implements one of its numbered sections; do not deviate from field names, function
  names, or file:line targets it gives without re-reading the spec first.
- **No new `PrimitiveKind` value.** It is shared with `ColliderParameters`; the imported-mesh
  concept lives only on `ShapeParameters`/`RenderInstance` (spec §3).
- **Builds go through the `se` CLI only** (`se build`, `se test`, `se editor --no-run`) — never
  raw `cmake`/`ninja` (`CLAUDE.md`).
- **C++17, Allman braces, `snake_case` functions/variables, `PascalCase` types, no abbreviations**
  — match the file being edited (`docs/CONTRIBUTING.md` §4).
- **Every source file carries the Apache 2.0 license header** already at the top of every file this
  plan touches — do not remove it.
- **`applications/editor/` has no test harness today** (spec §1, confirmed by grep and by
  `tests/CMakeLists.txt` naming no editor source). Tasks 3 and 4 touch editor UI code and are
  verified by build + manual check in `se editor`, not by an automated test — this is a pre-existing
  gap the plan inherits, not one it papers over.
- **Documentation ships with the change** (`docs/CONTRIBUTING.md` §5): Task 5 updates
  `docs/reference/changelog.md`; run
  `python tools/documentation/check_documentation_length.py` before committing any docs edit.

---

### Task 1: Data model and the extraction pipeline

**Files:**
- Modify: `engine/world/simulation/include/SushiEngine/simulation/simulation.hpp:435-439`
  (`ShapeParameters`), `:122-131` (`RenderInstance`)
- Modify: `engine/world/simulation/source/runtime_simulation.cpp:3583-3596` (the instance-population
  loop)
- Modify: `applications/editor/source/main.cpp:489-501` (the `RenderInstance` → `MeshInstance` copy)
- Test: `tests/integration/test_shape_render_extraction.cpp` (new file)

**Interfaces:**
- Consumes: `Render::MeshId`/`Render::INVALID_MESH`
  (`engine/domain/material/include/SushiEngine/material/material.hpp:66-69`,
  `using MeshId = std::uint32_t; constexpr MeshId INVALID_MESH = 0xFFFFFFFFu;`),
  `Render::MeshInstance::mesh`
  (`engine/presentation/render/include/SushiEngine/render/scene_view.hpp:123`, already consumed
  by every render pass — no change needed there).
- Produces: `ShapeParameters::mesh_path` (`std::string`), `ShapeParameters::mesh`
  (`Render::MeshId`), `RenderInstance::mesh` (`Render::MeshId`) — the fields Tasks 2-5 read and
  write.

- [ ] **Step 1: Write the failing test**

Create `tests/integration/test_shape_render_extraction.cpp`:

```cpp
/**************************************************************************/
/* test_shape_render_extraction.cpp                                       */
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

// A Renderer entity whose ShapeParameters names an imported mesh must extract into
// RenderScene::instances carrying that same Render::MeshId, alongside whatever `kind`
// still says — the render passes are the ones that ignore `kind` when `mesh` is set
// (Render::MeshInstance's own documented rule), not this extraction step. This is the
// one place that claim is checked against the entity that produces it.

#include <gtest/gtest.h>

#include <SushiEngine/simulation/simulation.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Simulation;

namespace
{
    void clear_world(IWorldEditor& world)
    {
        for (const EntityId id : world.entities())
            world.destroy(id);
    }

    const RenderInstance* find_instance(const RenderScene& scene, EntityId id)
    {
        for (const RenderInstance& instance : scene.instances)
            if (instance.id == id)
                return &instance;
        return nullptr;
    }
} // namespace

TEST(Integration_ShapeRenderExtraction, AnImportedMeshSurvivesExtraction)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();
    clear_world(world);

    const EntityId id = world.create_box("Prop");
    ASSERT_NE(id, NULL_ENTITY);

    ShapeParameters shape = world.shape_parameters(id);
    shape.mesh_path = "models/car.gltf";
    shape.mesh = Render::MeshId(7);
    world.set_shape_parameters(id, shape);

    simulation->tick(simulation->fixed_dt_seconds());

    const RenderInstance* instance = find_instance(simulation->render_scene(), id);
    ASSERT_NE(instance, nullptr) << "the entity reached no render channel at all";
    EXPECT_EQ(instance->mesh, Render::MeshId(7));
}

TEST(Integration_ShapeRenderExtraction, ABoxWithNoImportedMeshExtractsAsInvalid)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();
    clear_world(world);

    const EntityId id = world.create_box("Box");
    ASSERT_NE(id, NULL_ENTITY);
    simulation->tick(simulation->fixed_dt_seconds());

    const RenderInstance* instance = find_instance(simulation->render_scene(), id);
    ASSERT_NE(instance, nullptr);
    EXPECT_EQ(instance->mesh, Render::INVALID_MESH)
        << "a Box Renderer with no imported mesh must not silently draw one";
}
```

Register it beside `test_shape_render_extraction.cpp`'s siblings in
`tests/CMakeLists.txt` (find the `test_soft_body_service.cpp` entry under
`Integration` sources and add this file the same way, same label).

- [ ] **Step 2: Run the test to verify it fails**

Run: `se test --suite functional -k Integration_ShapeRenderExtraction`
Expected: **build failure** — `ShapeParameters` has no member `mesh_path`/`mesh`, `RenderInstance`
has no member `mesh`. This is the TDD red state; a compile error is the expected failure mode for a
new field, not a runtime assertion failure.

- [ ] **Step 3: Add the fields**

In `simulation.hpp`, extend `ShapeParameters` (`:435-439`):

```cpp
struct ShapeParameters
{
    PrimitiveKind kind = PrimitiveKind::Box;
    Vector3 parameters{Vector3{0.5, 0.5, 0.5}};
    std::string mesh_path;                    // glTF path `mesh` was imported from; empty = none
    Render::MeshId mesh = Render::INVALID_MESH; // set, `kind`/`parameters` are ignored downstream
};
```

Extend `RenderInstance` (`:122-131`):

```cpp
struct RenderInstance
{
    EntityId id = NULL_ENTITY;
    Matrix4 model;
    Vector3 color;
    PrimitiveKind shape_kind = PrimitiveKind::Box;
    Vector3 shape_parameters{Vector3{0.5, 0.5, 0.5}};
    Render::Material material{};
    Render::MeshId mesh = Render::INVALID_MESH; // mirrors ShapeParameters::mesh
};
```

Both files already include `<SushiEngine/material/material.hpp>` transitively through
`Render::Material`; if the compiler cannot resolve `Render::MeshId`/`Render::INVALID_MESH` in
`simulation.hpp`, add `#include <SushiEngine/material/material.hpp>` next to the existing render
includes at the top of the file.

- [ ] **Step 4: Wire the extraction loop**

In `runtime_simulation.cpp`, the instance-population loop (`:3583-3596`) already does:

```cpp
instance.shape_kind = record->shape_parameters.kind;
instance.shape_parameters = record->shape_parameters.parameters;
```

Add immediately after:

```cpp
instance.mesh = record->shape_parameters.mesh;
```

- [ ] **Step 5: Wire the editor-side copy**

In `applications/editor/source/main.cpp`, the `RenderInstance` → `MeshInstance` copy (`:489-501`)
already does:

```cpp
instance.kind = static_cast<SushiEngine::Render::MeshKind>(source.shape_kind);
instance.shape_parameters = source.shape_parameters;
```

Add immediately after:

```cpp
instance.mesh = source.mesh;
```

- [ ] **Step 6: Run the test to verify it passes**

Run: `se test --suite functional -k Integration_ShapeRenderExtraction`
Expected: both `Integration_ShapeRenderExtraction` cases PASS.

- [ ] **Step 7: Run the full functional suite to confirm no regression**

Run: `se test --suite functional`
Expected: same pass count as before this task plus the two new tests, no new failures.

- [ ] **Step 8: Commit**

```bash
git add engine/world/simulation/include/SushiEngine/simulation/simulation.hpp \
        engine/world/simulation/source/runtime_simulation.cpp \
        applications/editor/source/main.cpp \
        tests/integration/test_shape_render_extraction.cpp \
        tests/CMakeLists.txt
git commit -m "feat(simulation,render): carry an imported mesh through render extraction"
```

---

### Task 2: `bind_shape_mesh` and the Inspector's Renderer section

**Files:**
- Modify: `applications/editor/source/scene/inspector_panel.cpp:602-648` (Renderer section),
  add `bind_shape_mesh` near `bind_crowd_mesh` (`:144-165`)

**Interfaces:**
- Consumes: `ShapeParameters::mesh_path`/`::mesh` (Task 1), `Render::IAssetLibrary::load_gltf`
  (`engine/presentation/render/include/SushiEngine/render/asset_library_interface.hpp:118`,
  already implemented, zero prior callers), `EditorContext::assets`
  (already used by `bind_crowd_mesh`, same file).
- Produces: `bind_shape_mesh(EditorContext&, ShapeParameters&)`, called only from this file's
  Renderer section (Task 2) — no other task depends on its name.

- [ ] **Step 1: Add `bind_shape_mesh`**

In `inspector_panel.cpp`, immediately after `bind_crowd_mesh` (`:144-165`):

```cpp
/**
 * @brief Imports a Renderer's mesh from its authored path.
 *
 * Unlike @ref bind_crowd_mesh, the imported material is not adopted onto the entity's
 * own Material: a Shape always has its own authored, serialized Material already, and
 * silently overwriting it on every re-import would make it a value the file cannot
 * actually hold still.
 *
 * @param context Editor state; supplies the asset library and the console.
 * @param values  The Shape being authored; its mesh is written from @c mesh_path.
 */
void bind_shape_mesh(EditorContext& context, SushiEngine::Simulation::ShapeParameters& values)
{
    values.mesh = SushiEngine::Render::INVALID_MESH;
    if (context.assets == nullptr || values.mesh_path.empty())
        return;
    SushiEngine::Render::MeshId meshes[1] = {SushiEngine::Render::INVALID_MESH};
    SushiEngine::Render::Material materials[1]{};
    if (context.assets->load_gltf(values.mesh_path.c_str(), meshes, materials, 1) == 0)
    {
        editor_log(context, "No mesh imported from '" + values.mesh_path + "'.",
                  LogLevel::Warning);
        return;
    }
    values.mesh = meshes[0];
}
```

- [ ] **Step 2: Extend the Renderer section's mesh picker**

In `inspector_panel.cpp:616-618`, the mesh-kind combo currently reads:

```cpp
static const char* const MESH_NAMES[] = {"Box", "Sphere", "Cylinder"};
editor.choice("Mesh", &decltype(editor)::Values::kind, MESH_NAMES, 3,
              "Which primitive this renderer draws.");
```

This combo edits `ShapeParameters::kind` directly and has no fourth option for "no primitive, an
imported mesh instead" — `kind` and `mesh` are separate fields (Task 1), so the UI needs its own
local toggle rather than a fourth `PrimitiveKind` value (spec §3 — `PrimitiveKind` is not
extended). Replace the `if (world->has_shape(id))` block's body (`:606-648`) with:

```cpp
if (world->has_shape(id))
{
    const ComponentAccess<SushiEngine::Simulation::ShapeParameters> access{
        &IWorldEditor::has_shape, &IWorldEditor::shape_parameters,
        &IWorldEditor::set_shape_parameters};
    ComponentEditor<SushiEngine::Simulation::ShapeParameters> editor(context, *world, access, id);

    SushiEngine::Simulation::ShapeParameters& values = editor.mutable_values();
    bool imported = values.mesh != SushiEngine::Render::INVALID_MESH;

    static const char* const MESH_NAMES[] = {"Box", "Sphere", "Cylinder", "Imported"};
    int choice = imported ? 3 : static_cast<int>(values.kind);
    if (ImGui::Combo("Mesh", &choice, MESH_NAMES, 4))
    {
        if (choice == 3)
        {
            imported = true;
        }
        else
        {
            imported = false;
            values.kind = static_cast<SushiEngine::Simulation::PrimitiveKind>(choice);
            values.mesh = SushiEngine::Render::INVALID_MESH;
        }
    }

    if (imported)
    {
        ImGui::SetNextItemWidth(-80.0f);
        ImGui::InputText("Source Mesh", &values.mesh_path);
        ImGui::SameLine();
        if (ImGui::Button("Load"))
            bind_shape_mesh(context, values);
        if (values.mesh == SushiEngine::Render::INVALID_MESH)
            ImGui::TextColored(warning_color(), "No mesh imported -- this renderer draws "
                                                "nothing yet.");
        else
            ImGui::TextDisabled("Mesh imported.");
    }
    else
    {
        switch (values.kind)
        {
            case SushiEngine::Simulation::PrimitiveKind::Sphere:
                editor.vector_component(
                    "Radius##Mesh", &decltype(editor)::Values::parameters, 0, 0.01f,
                    0.01f, 1000.0f, "%.3f m", "Sphere radius before Scale, in metres.");
                break;
            case SushiEngine::Simulation::PrimitiveKind::Cylinder:
                editor.vector("Radius / Half Height##Mesh",
                              &decltype(editor)::Values::parameters, 0.01f, 0.01f,
                              1000.0f, "%.3f m",
                              "X is the radius, Y the half height, in metres; "
                              "Z is unused.");
                break;
            default:
                editor.vector("Half Extents##Mesh",
                              &decltype(editor)::Values::parameters, 0.01f, 0.01f,
                              1000.0f, "%.3f m",
                              "Half the box's size along each local axis, in "
                              "metres, before Scale.");
                break;
        }
    }
}
else if (ImGui::SmallButton("Add Mesh"))
{
    set_presence(&IWorldEditor::set_has_shape, true);
}
```

`warning_color()` is already defined in this file (`:129`) and already used by the Soft Body
section this mirrors (`:1044`).

- [ ] **Step 3: Build**

Run: `se editor --no-run`
Expected: clean build, no warnings (`-Wall -Wextra -Werror -pedantic` per `docs/CONTRIBUTING.md`
§3).

- [ ] **Step 4: Manual verification**

There is no automated test for this file (Global Constraints). Run `se editor`, open or create a
scene, select an entity with a Renderer, switch its Mesh combo to "Imported", type a real project
`.gltf`/`.glb` path into "Source Mesh", press Load, and confirm: (a) a non-existent path shows the
warning text and logs a `LogLevel::Warning` line; (b) a real path clears the warning and the entity
renders the imported mesh in the Scene view; (c) switching back to "Box"/"Sphere"/"Cylinder" clears
`mesh` and restores the primitive dimension editor.

- [ ] **Step 5: Commit**

```bash
git add applications/editor/source/scene/inspector_panel.cpp
git commit -m "feat(editor): add an Imported mesh kind to the Renderer's Inspector section"
```

---

### Task 3: Create ▸ Objects entry

**Files:**
- Modify: `applications/editor/source/scene/scene_commands.cpp:67-124` (the `Objects` submenu)

**Interfaces:**
- Consumes: `IWorldEditor::create_box` (already used by the existing `"Box"` entry, same file,
  `:69-74`) — the new entity starts as a Box exactly like every other primitive entry, so it has a
  valid `ShapeParameters` from creation; the artist switches it to "Imported" in the Inspector
  (Task 2) immediately after, the same two-step flow every other component in this menu already
  uses.
- Produces: nothing new consumed elsewhere — this task only adds a menu item.

- [ ] **Step 1: Add the menu item**

In `scene_commands.cpp`, immediately after the existing `"Box"` entry (`:69-74`):

```cpp
if (ImGui::MenuItem("Imported Mesh"))
{
    context.history.record(*world);
    select_only(context, world->create_box("Imported Mesh"));
    editor_log(context, "Created object 'Imported Mesh'.");
}
```

- [ ] **Step 2: Build**

Run: `se editor --no-run`
Expected: clean build.

- [ ] **Step 3: Manual verification**

Run `se editor`. Entity ▸ Objects (and the Hierarchy's equivalent context menu, which shares this
function per its own comment at `scene_commands.cpp:47-52`) shows "Imported Mesh" beside "Box".
Selecting it creates an entity named "Imported Mesh" with a Renderer already present, ready for
the Inspector's Mesh combo (Task 2).

- [ ] **Step 4: Commit**

```bash
git add applications/editor/source/scene/scene_commands.cpp
git commit -m "feat(editor): add Imported Mesh to the Create Objects menu"
```

---

### Task 4: Serialization

**Files:**
- Modify: `engine/world/serialization/source/scene_serializer.cpp` (wherever `ShapeParameters`'s
  `kind`/`parameters` are already read and written — locate with `grep -n
  "shape_parameters\|ShapeParameters" engine/world/serialization/source/scene_serializer.cpp`
  before editing, since this plan's line numbers are for files Task 1-3 touch, not this one)
- Test: `tests/integration/test_scene_serializer_roundtrip.cpp`

**Interfaces:**
- Consumes: `ShapeParameters::mesh_path` (Task 1), `Render::IAssetLibrary::load_gltf` (Task 2's
  import call, reused here for the load-time rebind).
- Produces: nothing new consumed elsewhere.

- [ ] **Step 1: Read the existing Shape read/write code**

Run: `grep -n "shape_parameters\|has_shape" engine/world/serialization/source/scene_serializer.cpp`.
Read both the write side (inside the entity-capture loop, alongside the `has_soft_body`/
`soft_body_parameters` pattern already read for Task-adjacent context at
`scene_serializer.cpp:654-677`) and the read side (the `apply_scene`/`load_scene` entity-restore
loop) before writing the two changes below — this file was not fully re-read for this plan and the
exact surrounding lines must be confirmed against the current tree, not guessed from the box
kind/parameters fields alone.

- [ ] **Step 2: Write the failing test**

In `tests/integration/test_scene_serializer_roundtrip.cpp`, add near the Crowd tests (`:733-760`
is `ACrowdSurvivesCaptureApply`, the closest precedent — this mirrors its shape, not its assertions,
since an imported mesh has no working `IAssetLibrary` in this test process, exactly like
`ACrowdWithAnUnloadableRigComesBackUnbound`, `:806-829`, has no real skeleton file for its
"unloadable" case):

```cpp
TEST(Integration_SceneSerializer, AnImportedMeshPathSurvivesCaptureApply)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();
    clear_world(world);

    const EntityId id = world.create_box("Prop");
    ASSERT_NE(id, NULL_ENTITY);
    ShapeParameters authored = world.shape_parameters(id);
    authored.mesh_path = "models/car.gltf";
    world.set_shape_parameters(id, authored);

    const nlohmann::json snapshot = Scene::capture_scene(world);
    clear_world(world);
    Scene::apply_scene(world, snapshot);

    const EntityId restored = find_by_name(world, "Prop");
    ASSERT_NE(restored, NULL_ENTITY);
    const ShapeParameters after = world.shape_parameters(restored);
    EXPECT_EQ(after.mesh_path, "models/car.gltf");
    // No real Render::IAssetLibrary runs in this test process, so the path cannot
    // resolve — proving the round trip carries the path rather than a stale handle
    // from a session that no longer exists, the same claim
    // ACrowdWithAnUnloadableRigComesBackUnbound makes for Crowd's mesh_path.
    EXPECT_EQ(after.mesh, Render::INVALID_MESH);
}
```

- [ ] **Step 3: Run the test to verify it fails**

Run: `se test --suite functional -k AnImportedMeshPathSurvivesCaptureApply`
Expected: FAIL — `mesh_path` comes back empty (not yet written/read by the serializer).

- [ ] **Step 4: Extend the write side**

Wherever Step 1 found `shape_parameters`'s `kind`/`parameters` written into the entity's JSON
object, add `mesh_path` as a sibling field, matching the existing key-naming convention in that
same block (snake_case JSON keys, e.g. `"mesh_path"`).

- [ ] **Step 5: Extend the read side**

Wherever Step 1 found `kind`/`parameters` read back, add: read `mesh_path` (default empty), and
when it is non-empty, call the same rebind Task 2's `bind_shape_mesh` performs — inline the same
three lines (`load_gltf` into a one-element `MeshId`/`Material` array, assign `.mesh`) using the
`Render::IAssetLibrary& assets` parameter this function already threads through for Crowd's own
rebind (`scene_serializer.cpp:1478-1495`), rather than calling into `applications/editor/` code
(the serializer is an `engine/` module and must not depend on `applications/editor/`).

- [ ] **Step 6: Run the test to verify it passes**

Run: `se test --suite functional -k AnImportedMeshPathSurvivesCaptureApply`
Expected: PASS.

- [ ] **Step 7: Run the full functional suite**

Run: `se test --suite functional`
Expected: same pass count as before this task plus the one new test, no new failures.

- [ ] **Step 8: Commit**

```bash
git add engine/world/serialization/source/scene_serializer.cpp \
        tests/integration/test_scene_serializer_roundtrip.cpp
git commit -m "feat(serialization): carry an imported mesh's path through save, undo and play"
```

---

### Task 5: Changelog

**Files:**
- Modify: `docs/reference/changelog.md`

- [ ] **Step 1: Add the entry**

Under `## [Unreleased]` ▸ `### Added`, at the top of that list, in this repository's bullet style
(`docs/CONTRIBUTING.md` §6, `docs/documentation-style-guide.md`):

```markdown
- 2026-08-05 — Added an "Imported" Renderer mesh kind: `ShapeParameters::mesh_path`/`::mesh`,
  wired to `Render::IAssetLibrary::load_gltf` and every render pass that already consumed
  `MeshInstance::mesh`. Placing an ordinary static prop in the scene no longer requires `Crowd`
  or the physics cooking pipeline. See `docs/design/static_mesh_authoring.md`.
  - Added the Inspector Renderer section's "Imported" mesh choice and Create ▸ Objects'
    "Imported Mesh" entry.
  - Added `mesh_path` to scene serialization, re-binding on load the same way Crowd's own
    `mesh_path` already does.
```

- [ ] **Step 2: Check the length ceilings**

Run: `python tools/documentation/check_documentation_length.py`
Expected: no new violations (the existing four pre-existing warnings in `changelog.md` are
unrelated to this entry — do not fix them as part of this change).

- [ ] **Step 3: Commit**

```bash
git add docs/reference/changelog.md
git commit -m "docs: log the Imported mesh renderer kind"
```

---

## Plan Self-Review Notes

- **Spec coverage:** §3 (data model) → Task 1. §4 (import/binding) → Task 2 Step 1. §5 (render
  wiring) → Task 1 Steps 4-5. §6 (editor UI) → Tasks 2-3. §7 (serialization) → Task 4. §8
  (physics) → no task; the spec states it is unchanged and no file this plan touches is a physics
  file. §9 (testing) → Tasks 1 and 4's test steps; the spec's third bullet ("a unit test on
  `bind_shape_mesh`") is *not* a separate task — `applications/editor/` has no harness to put it
  in (Global Constraints), so Task 2 substitutes a manual-verification step instead, and says why.
- **Type consistency:** `ShapeParameters::mesh`/`RenderInstance::mesh` are `Render::MeshId`
  everywhere across Tasks 1, 2 and 4; `bind_shape_mesh`'s signature
  (`EditorContext&, ShapeParameters&`) matches its one call site in Task 2 Step 2.
- **Placeholder scan:** no TBD/TODO; Task 4 Steps 1, 4 and 5 name a `grep` to run rather than a
  guessed line number because `scene_serializer.cpp`'s Shape read/write block was not re-read line
  by line while writing this plan (unlike every other file above, which was) — that is a deliberate
  instruction to verify before editing, not an unresolved placeholder.
