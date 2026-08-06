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

This is **part four of four**. Parts one to three build everything this part calls, and all three
must be complete before this one starts. From them this part consumes:

- `Geometry::import_gltf_scene`, `Geometry::GLTFSceneDescription` (part one, Task 1)
- `Model::ModelImportSettings`, `Model::load_model_import_settings` (part two, Task 2)
- `Model::plan_model_instantiation`, `ModelInstantiationPlan`, `ModelImportReport`,
  `PlannedEntity`, `PlannedComponent` (part three, Task 4)
- `Render::IAssetLibrary::load_gltf_scene`, `Render::ImportedPrimitive` (part three, Task 5)

Task numbering runs across all four parts, so Task 6 is the first task here.

## Task 6: Execute a plan in the editor

**Files:**
- Create: `applications/editor/source/project/model_instantiate.{hpp,cpp}`
- Modify: `applications/editor/CMakeLists.txt`

**Interfaces:**
- Consumes: Tasks 1, 2, 4, 5.
- Produces: `Simulation::EntityId instantiate_model(EditorContext& context, const std::string&
  asset_path, Simulation::EntityId parent)`, declared in `namespace SushiEngine { namespace Editor
  {`, returning the created subtree's root or `Simulation::NULL_ENTITY` on failure. `EntityId` is
  `std::uint64_t` (`simulation.hpp:77`). Tasks 7 and 8 call it.

- [ ] **Step 1: Write the executor**

One function, no branches beyond null checks. It reads the settings, imports the description, plans,
imports the meshes, then walks the plan once creating entities. Every step's failure returns
`NULL_ENTITY` after one `editor_log` naming the path and the reason.

Bracket the whole walk in one `context.history` change so twenty entities appear and disappear
together, matching how the Hierarchy panel's own commands record. Read
`applications/editor/source/scene/scene_commands.cpp` for the established creation and undo idiom
before writing, and follow it rather than inventing a second one.

Map `PlannedComponent` to the world's creators: `None` to `create`, `Shape` to `create_box` followed
by `set_shape_parameters` carrying the imported `MeshId`, `Light` to `create_light` followed by
`set_light_parameters`, `Camera` to `create_camera` then `set_camera_parameters`. Confirm each
signature in `simulation.hpp` before calling it: `create` is at `:1043`, `create_light` at `:1625`,
`create_camera` at `:1285`, `set_shape_parameters` at `:1745`, `set_transform` at `:1057`,
`set_parent` at `:1263`, `set_name` at `:1049`, `set_material` at `:1133`.

Convert a `GLTFLightDescription` to `LightParameters` (`simulation.hpp:453-462`) explicitly: point
and spot only, `spot_*_cone_radians` converted to the degrees that struct takes, and `range` mapped
to its `range`. A directional light never reaches here — Task 4 already dropped it.

Adopt the imported material onto each Shape entity through `set_material` when
`settings.import_materials` is set. This is the one place the import deliberately differs from
`bind_shape_mesh`, which does not adopt: a node's material is what makes a multi-material model look
right, and there is no previously authored material on an entity created this instant to protect.

- [ ] **Step 2: Report the result**

Log one line summarising the report, and one warning line per `report.warnings` entry. The summary
reads as counts, not as a verdict:

```
Imported 'models/Car.gltf': 15 nodes, 22 primitives, 6 materials, 2 lights, 1 camera.
```

Append what was skipped only when non-zero, so an ordinary import does not read like a list of
problems.

- [ ] **Step 3: Register the source**

Add `source/project/model_instantiate.cpp` to `applications/editor/CMakeLists.txt`'s source list and
link `sushiengine_model`.

- [ ] **Step 4: [USER CHECKPOINT] Verify**

```bash
se build
```

Then, since this task has no automated coverage, hand the user a manual check: open the editor,
place a model through Task 7's drag once it exists, or temporarily call `instantiate_model` from an
existing menu item. State in the handoff that this task is verified by eye, not by test, and why:
creating entities needs a live world and a device, which is the boundary
`applications/editor/source/physics/cook_bake_panel.hpp:36-39` already draws around editor code.

- [ ] **Step 5: Commit**

```bash
git add applications/editor
git commit -m "feat(editor): create an entity hierarchy from a planned model import"
```

---

## Task 7: Drag an asset into the scene

**Files:**
- Modify: `applications/editor/source/scene/hierarchy_panel.cpp`
- Modify: `applications/editor/source/ui/viewport_panel.cpp`

**Interfaces:**
- Consumes: `instantiate_model` (Task 6), `accept_asset_drop`
  (`applications/editor/source/ui/panel_widgets.cpp:56-67`), `ASSET_PATH_PAYLOAD`
  (`panel_widgets.hpp:59`).
- Produces: nothing later tasks name.

- [ ] **Step 1: Add the Hierarchy row drop target**

In the row-drawing function that already calls `ImGui::BeginDragDropTarget` for `HIERARCHY_ENTITY`
(`hierarchy_panel.cpp:150-179`), accept `ASSET_PATH_PAYLOAD` as well. A row's drop parents the new
subtree under that row's entity; the panel's empty-space target (`:351-359`) parents it at the root.
Only accept paths whose extension is `.gltf` or `.glb`, lower-cased — a dropped `.txt` must do
nothing rather than log a failed import.

- [ ] **Step 2: Add the viewport drop target**

In `viewport_panel.cpp`, after the viewport image is submitted, add a drop target over it that calls
`instantiate_model` with `NULL_ENTITY` as the parent. Placing the model at the drop point in the
world needs a ray cast this task does not build; place it at the scene origin and say so in the
commit rather than implying the drop position is honoured.

- [ ] **Step 3: [USER CHECKPOINT] Verify by hand**

Hand the user: `se editor`, then drag a `.gltf` from the Project panel onto a Hierarchy row, onto
Hierarchy empty space, and onto the viewport. Expected: a named subtree appears in the right place
each time, and one undo removes all of it.

- [ ] **Step 4: Commit**

```bash
git add applications/editor
git commit -m "feat(editor): instantiate a model by dragging it out of the Project panel"
```

---

## Task 8: Not every glTF is a character

**Files:**
- Modify: `applications/editor/source/project/project_panel.cpp:62-104,474-511`
- Modify: `docs/` wherever the same claim appears

**Interfaces:**
- Consumes: `instantiate_model` (Task 6), `import_gltf_scene` (Task 1).
- Produces: nothing later tasks name.

- [ ] **Step 1: Rename and re-route**

`has_character_extension` becomes `is_model_extension`, and its comment stops calling every glTF a
rigged character asset. `open_character_in_preview` becomes `open_model_asset` and branches: import
the description first, and when `description.skin_count > 0` route to the animated preview as
today; otherwise call `instantiate_model`. Update all four call sites (`:484`, `:501`, `:508`, and
the context-menu Open at `:495`).

- [ ] **Step 2: Fix the log lines**

The success line stops asserting a category the file has not claimed and reports what arrived. The
failure line stops saying "as a rigged character" for a file never offered as one — it names what
actually failed, which is either the parse or the instantiation.

- [ ] **Step 3: Hide `.meta` from the browser**

In the same file's directory listing, skip entries whose extension is `.meta`. Task 2 puts one
every configured asset, and showing them doubles the browser's contents with files nothing opens.

- [ ] **Step 4: Sweep the documentation**

```bash
grep -rni "character" docs/ --include=*.md
```

Correct every sentence claiming a glTF import is a character import. Leave alone the ones genuinely
about rigged characters, crowds or skinning — the point is accuracy, not removing the word.

- [ ] **Step 5: [USER CHECKPOINT] Verify by hand**

Hand the user: `se editor`, then double-click a static `.gltf` and a rigged one. Expected: the
one appears in the scene and the log names counts; the rigged one still opens the preview; no line
calls a static prop a character.

- [ ] **Step 6: Run the checks and commit**

```bash
python tools/documentation/check_documentation_length.py
git add applications/editor docs
git commit -m "fix(editor): stop calling every imported glTF a rigged character"
```

---

## Task 9: Generate colliders on import

**Files:**
- Modify: `applications/editor/source/project/model_instantiate.cpp`

**Interfaces:**
- Consumes: `PlannedEntity::generate_collider` (Task 4), `Authoring::CookBakeState::bake`
  (`engine/world/authoring/include/SushiEngine/authoring/cook_bake_state.hpp`).
- Produces: nothing later tasks name.

- [ ] **Step 1: Queue the cook**

In the plan walk, when `entity.generate_collider` is set and `context.cook_bake_state` is non-null,
call `context.cook_bake_state->bake(asset_path)`. Read `project_panel.cpp:70-80` first: it already
queues on open and documents that `bake` is a no-op past the first call for an unchanged asset, so
queuing once per model rather than once per entity is what this should do — confirm which by reading
`CookBakeState::bake`, and queue at whichever granularity the cooker actually keys on.

- [ ] **Step 2: Carry the report's warning through**

Task 4 warns when a collider-carrying entity sits under a file-scaled parent. Surface it in
the log with the others; it is the one case where the collider is knowingly the wrong size,
and §11 commits to saying so at import.

- [ ] **Step 3: [USER CHECKPOINT] Verify by hand**

Hand the user: `se editor`, set `generate_colliders` in a model's `.meta` (by hand for now — the
inspector that edits it is the next sub-project), drag the model in, and check the Bake window lists
the asset. Expected: the asset appears in "Baked this session" with a collider report.

- [ ] **Step 4: Commit**

```bash
git add applications/editor
git commit -m "feat(editor): cook colliders for an imported model when its settings ask for it"
```

---

## Task 10: Close the documentation

**Files:**
- Modify: `docs/reference/changelog.md`, `docs/design/model_import.md` §13,
  `docs/architecture/` (the chapter naming the import path), `README.md` if it describes importing

- [ ] **Step 1: Write the changelog entries**

Under `## [Unreleased]` / `Added`, one bullet each, 240 characters at most:

```markdown
- Added glTF scene import: a file's node graph becomes an entity hierarchy with its pivots, names,
  per-node materials, lights and cameras, replacing the previous single-primitive binding.
- Added per-asset model import settings in a `<asset>.meta` sidecar: scale, root rotation, and
  toggles for materials, lights, cameras, pivots and collider generation.
- Added drag-and-drop model placement from the Project panel onto the Hierarchy and the viewport.
```

Task 3's `Changed` bullet and Task 8's `Fixed` bullet are written in their own tasks; check they are
present rather than writing them twice.

- [ ] **Step 2: Update the architecture chapter**

```bash
grep -rln "load_gltf\|import" docs/architecture/
```

Update whichever chapter describes the asset import path so it names `import_gltf_scene`,
`plan_model_instantiation` and `load_gltf_scene`. Every class and concept an architecture chapter
names must exist, which is what makes this part of the change rather than after it.

- [ ] **Step 3: Update the design document's roadmap**

In `docs/design/model_import.md` §13, change P0's status from "designed, not started" to shipped
with the date, and record in the same paragraph anything that changed during implementation —
following `static_mesh_authoring.md` §11's precedent, which records seven such changes honestly.
Update the status line at the top of the file and the `docs/design/README.md` table row to match.

- [ ] **Step 4: Run every check**

```bash
python tools/documentation/check_documentation_length.py
python tools/documentation/check_module_documentation.py
python tools/layering/check_include_layering.py
```

- [ ] **Step 5: [USER CHECKPOINT] Final verification**

```bash
se build
se test --suite functional
```

Expected: a clean build with `-Wall -Wextra -Werror -pedantic`, and every test passing. Report the
actual output. If anything fails, the task is not complete — say so and fix it.

- [ ] **Step 6: Commit**

```bash
git add docs README.md
git commit -m "docs: document glTF scene import and its per-asset settings"
```

---
---

## What this plan does not build

Stated so it is not mistaken for an oversight:

- **No link from a placed subtree back to its asset, and therefore no reimport** (§4.3). Changing an
  asset's `.meta` does not update subtrees already in a scene. The prefab system owns this.
- **No asset inspector.** The `.meta` is edited by hand in this phase. The Bake window and the
  Cooking Override modal stay where they are.
- **No rigged models inside a hierarchy.** A skinned node keeps its place in the tree and loses its
  mesh, and the import says so.
- **No drop-position placement.** A model dropped on the viewport lands at the scene origin.
