# Project Selection Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development
> (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use
> checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add `File ▸ New Project...`/`File ▸ Load Project...` so an artist can point the editor at
a different project directory at runtime — today `EditorContext::project_root` is resolved once at
startup and never reassigned anywhere in the editor.

**Architecture:** A new directories-only picker modal (no native OS dialog, no new dependency —
`std::filesystem`, matching how the Project panel's own navigation already works cross-platform)
feeds a `request_switch_project`/`switch_project` pair that mirrors the existing
`request_new_scene`/`new_scene` unsaved-changes-guard pattern exactly. Persistence reuses the
already cross-platform `Preferences`/`IPreferencesStore` machinery unchanged — the switch just
raises the same `preferences_dirty` flag every other preference change already does.

**Tech Stack:** C++17, Dear ImGui (`applications/editor`), `std::filesystem`, the SushiEngine
authoring/preferences layer (`engine/world/authoring`).

## Global Constraints

- **Spec:** `docs/design/project_selection.md`, approved and committed at `5d50695`. Every task
  below implements one of its numbered sections; do not deviate from field names, function names,
  or file:line targets it gives without re-reading the spec first.
- **No project manifest file, no recent-projects list, no native OS dialog** (spec §2) — do not
  add any of these.
- **Builds go through the `se` CLI only** (`se build`, `se editor --no-run`) — never raw
  `cmake`/`ninja` (`CLAUDE.md`).
- **C++17, Allman braces, `snake_case` functions/variables, `PascalCase` types, no abbreviations**
  — match the file being edited (`docs/CONTRIBUTING.md` §4).
- **Every source file carries the Apache 2.0 license header** already at the top of every file
  this plan touches — do not remove it.
- **`applications/editor/` has no test harness today** (spec §1/§8) — every task in this plan
  touches only `applications/editor/`, so every task is verified by build + manual check in
  `se editor`, not by an automated test. This is a pre-existing gap the plan inherits.
- **Documentation ships with the change** (`docs/CONTRIBUTING.md` §5): Task 4 updates
  `docs/reference/changelog.md`; run
  `python tools/documentation/check_documentation_length.py` before committing any docs edit.

---

### Task 1: State and the picker modal

**Files:**
- Modify: `applications/editor/source/core/editor_context.hpp:287-333` (new fields, extend
  `PendingSceneAction`)
- Create: `applications/editor/source/project/project_picker.hpp`
- Create: `applications/editor/source/project/project_picker.cpp`
- Modify: `applications/editor/source/project/project_panel.hpp` or the editor's panel-registration
  point (locate with `grep -rn "draw_cook_bake_panel\|draw_project_panel"
  applications/editor/source/` before editing) to call the new `draw_project_picker(context)` once
  per frame, the same way `draw_cook_bake_panel` is already called unconditionally every frame
  regardless of window visibility
  (`applications/editor/source/physics/cook_bake_panel.cpp:487-494`'s own comment
  explains why: a request must survive the modal being closed and reopened).

**Interfaces:**
- Consumes: nothing from another task.
- Produces: `EditorContext::show_project_picker` (`bool`), `::project_picker_mode`
  (`ProjectPickerMode`), `::project_picker_directory` (`std::string`),
  `::project_picker_new_folder_name` (`std::string`) — Task 2 sets these to open the modal;
  `void draw_project_picker(EditorContext&)` — Task 2 does not call this directly, only opens it
  via the state above, since it is drawn once per frame unconditionally like `cook_bake_panel`.

- [ ] **Step 1: Add the picker's state to `EditorContext`**

In `editor_context.hpp`, immediately after `current_directory` (`:288`):

```cpp
enum class ProjectPickerMode { New, Load };

bool show_project_picker = false;
ProjectPickerMode project_picker_mode = ProjectPickerMode::Load;
std::string project_picker_directory;       // where the modal is currently browsing
std::string project_picker_new_folder_name; // New mode only
```

In the same file, extend `PendingSceneAction` (`:326-331`):

```cpp
enum class PendingSceneAction
{
    None,
    New,
    Open,
    SwitchProject
};
PendingSceneAction pending_scene_action = PendingSceneAction::None;
std::string pending_scene_open_path;
std::string pending_project_switch_path; // target for SwitchProject
```

- [ ] **Step 2: Create the picker header**

Create `applications/editor/source/project/project_picker.hpp`:

```cpp
/**************************************************************************/
/* project_picker.hpp                                                     */
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

namespace SushiEngine
{
    namespace Editor
    {
        struct EditorContext;

        /**
         * @brief Draws the New/Load Project directory picker, when open.
         *
         * A no-op when @ref EditorContext::show_project_picker is false. Called
         * unconditionally once per frame, the same way `draw_cook_bake_panel` is, so a
         * request survives the modal being closed and reopened.
         *
         * @param context Editor state; reads and writes the picker fields it owns.
         */
        void draw_project_picker(EditorContext& context);
    } // namespace Editor
} // namespace SushiEngine
```

- [ ] **Step 3: Create the picker implementation**

Create `applications/editor/source/project/project_picker.cpp`:

```cpp
/**************************************************************************/
/* project_picker.cpp                                                     */
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

#include "project_picker.hpp"

#include "../core/editor_context.hpp"
#include "../scene/scene_commands.hpp" // request_switch_project (Task 2)

#include <filesystem>
#include <system_error>
#include <vector>

#include <imgui.h>
#include <misc/cpp/imgui_stdlib.h>

namespace SushiEngine
{
    namespace Editor
    {
        namespace fs = std::filesystem;

        void draw_project_picker(EditorContext& context)
        {
            if (!context.show_project_picker)
                return;

            const char* title = context.project_picker_mode == ProjectPickerMode::New
                                     ? "New Project"
                                     : "Load Project";
            ImGui::OpenPopup(title);
            if (!ImGui::BeginPopupModal(title, &context.show_project_picker,
                                        ImGuiWindowFlags_AlwaysAutoResize))
            {
                ImGui::EndPopup();
                return;
            }

            const fs::path current(context.project_picker_directory);
            ImGui::TextDisabled("%s", current.string().c_str());
            ImGui::SameLine();
            if (ImGui::SmallButton("Up"))
                context.project_picker_directory = current.parent_path().string();
            ImGui::Separator();

            ImGui::BeginChild("project_picker_list", ImVec2(360.0f, 220.0f), true);
            std::error_code ec;
            std::vector<fs::directory_entry> directories;
            for (const auto& entry : fs::directory_iterator(current, ec))
                if (entry.is_directory())
                    directories.push_back(entry);
            for (const fs::directory_entry& entry : directories)
            {
                const std::string name = entry.path().filename().string();
                if (ImGui::Selectable(name.c_str()))
                    context.project_picker_directory = entry.path().string();
            }
            ImGui::EndChild();

            if (context.project_picker_mode == ProjectPickerMode::New)
            {
                ImGui::SetNextItemWidth(-FLT_MIN);
                ImGui::InputTextWithHint("##new_project_name", "Folder name",
                                         &context.project_picker_new_folder_name);
                ImGui::BeginDisabled(context.project_picker_new_folder_name.empty());
                if (ImGui::Button("Create & Select"))
                {
                    const fs::path target =
                        current / context.project_picker_new_folder_name;
                    std::error_code create_ec;
                    fs::create_directories(target, create_ec);
                    if (!create_ec)
                    {
                        request_switch_project(context, target.string());
                        context.show_project_picker = false;
                    }
                }
                ImGui::EndDisabled();
            }
            else
            {
                if (ImGui::Button("Select This Folder"))
                {
                    request_switch_project(context, current.string());
                    context.show_project_picker = false;
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("Cancel"))
                context.show_project_picker = false;

            ImGui::EndPopup();
        }
    } // namespace Editor
} // namespace SushiEngine
```

- [ ] **Step 4: Register the source files and the per-frame call**

Add `project/project_picker.cpp` to the editor's CMake source list (find it with
`grep -n "project_panel.cpp" applications/editor/CMakeLists.txt` and add the new file the same
way, same target). Add `#include "project/project_picker.hpp"` and a `draw_project_picker(context);`
call in `main.cpp` beside the existing unconditional per-frame panel calls (find the exact spot
with `grep -n "draw_cook_bake_panel" applications/editor/source/main.cpp`).

- [ ] **Step 5: Build**

Run: `se editor --no-run`
Expected: clean build. `request_switch_project` does not exist yet (Task 2) — this task is not
independently buildable until Task 2 lands; if executing tasks strictly in order this is expected
and Task 2 follows immediately. If a reviewer needs Task 1 to build alone, forward-declare
`void request_switch_project(EditorContext&, const std::string&);` at the top of
`project_picker.cpp` instead of including `scene_commands.hpp`, and remove the forward declaration
once Task 2 adds the real header include.

- [ ] **Step 6: Commit**

```bash
git add applications/editor/source/core/editor_context.hpp \
        applications/editor/source/project/project_picker.hpp \
        applications/editor/source/project/project_picker.cpp \
        applications/editor/CMakeLists.txt \
        applications/editor/source/main.cpp
git commit -m "feat(editor): add the New/Load Project directory picker modal"
```

---

### Task 2: Switching, guarded by unsaved changes

**Files:**
- Modify: `applications/editor/source/scene/scene_commands.hpp` (declare
  `request_switch_project`)
- Modify: `applications/editor/source/scene/scene_commands.cpp:551-634` (`new_scene`,
  `perform_pending_scene_action`, add `switch_project`/`request_switch_project`)

**Interfaces:**
- Consumes: `scene_is_dirty` (`editor_context.hpp:672`), `new_scene` (`scene_commands.cpp:555`),
  `EditorContext::cook_bake_state` (already declared, `editor_context.hpp:264`,
  `Authoring::CookBakeState*`), `EditorContext::preferences_dirty` (`:562`).
- Produces: `void request_switch_project(EditorContext&, const std::string&)` — Task 1's picker
  calls this; Task 3's menu items call this too.

- [ ] **Step 1: Declare `request_switch_project`**

In `scene_commands.hpp`, beside the existing `request_new_scene`/`request_open_scene`
declarations:

```cpp
/**
 * @brief Requests a project switch, deferring to the unsaved-changes prompt when the
 * current scene is dirty rather than discarding it silently.
 * @param context  Editor state.
 * @param new_root The directory to make the new @ref EditorContext::project_root.
 */
void request_switch_project(EditorContext& context, const std::string& new_root);
```

- [ ] **Step 2: Implement `switch_project` and `request_switch_project`**

In `scene_commands.cpp`, immediately after `new_scene` (`:555-573`), inside the same anonymous
namespace `new_scene`/`open_scene` already live in:

```cpp
// Re-points every project-scoped path a live session holds, and persists the choice
// the same way every other Preferences field change already does (preferences_dirty,
// flushed once per frame by main.cpp). The old project's scene has nothing left to
// belong to once project_root moves, so this always starts from a clean scene rather
// than carrying entities whose asset paths resolve against a directory that is no
// longer current.
void switch_project(EditorContext& context, const std::string& new_root)
{
    new_scene(context);
    context.project_root = new_root;
    context.current_directory = new_root;
    context.preferences.last_project_root = new_root;
    context.preferences_dirty = true;
    if (context.cook_bake_state != nullptr)
    {
        context.cook_bake_state->set_profile_storage_path(
            (std::filesystem::path(new_root) / "cooking_profile.json").string());
        context.cook_bake_state->load_profiles();
    }
    editor_log(context, "Switched project to '" + new_root + "'.");
}
```

After the closing `}` of the anonymous namespace (`:598`), beside `request_new_scene`/
`request_open_scene` (`:619-634`):

```cpp
void request_switch_project(EditorContext& context, const std::string& new_root)
{
    if (scene_is_dirty(context))
    {
        context.pending_scene_action = EditorContext::PendingSceneAction::SwitchProject;
        context.pending_project_switch_path = new_root;
    }
    else
    {
        switch_project(context, new_root);
    }
}
```

- [ ] **Step 3: Resolve the pending action**

In `perform_pending_scene_action` (`:602-617`), add a third `case` and clear the new field
alongside the existing cleanup:

```cpp
void perform_pending_scene_action(EditorContext& context)
{
    switch (context.pending_scene_action)
    {
        case EditorContext::PendingSceneAction::New:
            new_scene(context);
            break;
        case EditorContext::PendingSceneAction::Open:
            open_scene(context, context.pending_scene_open_path);
            break;
        case EditorContext::PendingSceneAction::SwitchProject:
            switch_project(context, context.pending_project_switch_path);
            break;
        case EditorContext::PendingSceneAction::None:
            break;
    }
    context.pending_scene_action = EditorContext::PendingSceneAction::None;
    context.pending_scene_open_path.clear();
    context.pending_project_switch_path.clear();
}
```

`switch_project` is declared above `perform_pending_scene_action` in the same file (Step 2 places
it in the same anonymous namespace `new_scene` lives in, and `perform_pending_scene_action` already
calls `new_scene`/`open_scene` from outside that namespace today — `switch_project` needs the same
visibility `new_scene` already has, so no additional forward declaration is needed if it is placed
in the same namespace block).

- [ ] **Step 4: Build**

Run: `se editor --no-run`
Expected: clean build, `project_picker.cpp`'s forward declaration (Task 1 Step 5) can now be
replaced with the real `#include "scene_commands.hpp"` if that workaround was used.

- [ ] **Step 5: Manual verification**

Run `se editor`. With a clean (just-opened, unmodified) scene: File ▸ New Project..., create a
folder, confirm the window's Project panel now browses the new folder and the log shows "Switched
project to '...'". Then File ▸ Load Project... back to the original project directory. Then, with
unsaved changes (move an entity), repeat File ▸ Load Project... and confirm the existing
unsaved-changes prompt appears before anything switches — do not add a new prompt UI; this must be
the same modal `request_new_scene` already triggers today.

- [ ] **Step 6: Commit**

```bash
git add applications/editor/source/scene/scene_commands.hpp \
        applications/editor/source/scene/scene_commands.cpp
git commit -m "feat(editor): switch projects through the same unsaved-changes guard as New Scene"
```

---

### Task 3: File menu entries

**Files:**
- Modify: `applications/editor/source/ui/editor_panels.cpp:96-98` (top of the `File` menu)

**Interfaces:**
- Consumes: `EditorContext::show_project_picker`/`::project_picker_mode`/
  `::project_picker_directory` (Task 1), `default_projects_root()`
  (`applications/editor/source/main.cpp:101-121` — already a free function in the `main.cpp`
  anonymous namespace; move it to a small shared header if `editor_panels.cpp` cannot see it, e.g.
  `applications/editor/source/project/project_picker.hpp`, since two translation units now need
  it).

- [ ] **Step 1: Make `default_projects_root` shared**

`default_projects_root()` is currently a `main.cpp`-local anonymous-namespace function
(`main.cpp:95-122`). Move its declaration into `project_picker.hpp` (Task 1) and its definition
into `project_picker.cpp`, dropping the anonymous namespace so both `main.cpp` and
`editor_panels.cpp` can call it; update `main.cpp:338-340`'s call site to the now-shared function
(same name, same behavior, no signature change).

- [ ] **Step 2: Add the menu items**

In `editor_panels.cpp`, immediately above the existing `"New Scene"` entry (`:98`):

```cpp
if (ImGui::MenuItem("New Project..."))
{
    context.project_picker_mode = ProjectPickerMode::New;
    context.project_picker_directory =
        std::filesystem::path(default_projects_root()).parent_path().string();
    context.project_picker_new_folder_name.clear();
    context.show_project_picker = true;
}
if (ImGui::MenuItem("Load Project..."))
{
    context.project_picker_mode = ProjectPickerMode::Load;
    context.project_picker_directory =
        std::filesystem::path(context.project_root).parent_path().string();
    context.show_project_picker = true;
}
ImGui::Separator();
```

- [ ] **Step 3: Build**

Run: `se editor --no-run`
Expected: clean build.

- [ ] **Step 4: Manual verification**

Run `se editor`. File menu shows "New Project..." and "Load Project..." above "New Scene", each
opening Task 1's modal in the right mode with the right starting directory.

- [ ] **Step 5: Commit**

```bash
git add applications/editor/source/ui/editor_panels.cpp \
        applications/editor/source/project/project_picker.hpp \
        applications/editor/source/project/project_picker.cpp \
        applications/editor/source/main.cpp
git commit -m "feat(editor): add File > New/Load Project menu entries"
```

---

### Task 4: Changelog

**Files:**
- Modify: `docs/reference/changelog.md`

- [ ] **Step 1: Add the entry**

Under `## [Unreleased]` ▸ `### Added`, at the top of that list:

```markdown
- 2026-08-05 — Added File > New/Load Project: `EditorContext::project_root` can be changed at
  runtime instead of only at startup, through the same unsaved-changes guard `request_new_scene`
  already uses. See `docs/design/project_selection.md`.
  - Added a directories-only picker modal (`project_picker.cpp`), no native OS dialog and no new
    dependency.
  - Changed `default_projects_root` from a `main.cpp`-local function into a shared one so the
    File menu can seed the picker's starting directory.
```

- [ ] **Step 2: Check the length ceilings**

Run: `python tools/documentation/check_documentation_length.py`
Expected: no new violations.

- [ ] **Step 3: Commit**

```bash
git add docs/reference/changelog.md
git commit -m "docs: log File > New/Load Project"
```

---

## Plan Self-Review Notes

- **Spec coverage:** §3 (state) → Task 1 Step 1. §4 (File menu) → Task 3. §5 (picker modal) →
  Task 1 Steps 2-3. §6 (switching) → Task 2. §7 (persistence) → Task 2 Step 2 (`preferences_dirty`),
  no separate task since the spec itself says "no new mechanism". §8 (testing) → the Global
  Constraints note and every task's manual-verification step; the spec's own §8 already says this
  tier has no harness, so no task invents one.
- **Type consistency:** `ProjectPickerMode`/`show_project_picker`/`project_picker_directory` are
  used identically in Task 1's modal, Task 2's `switch_project` (indirectly, via the string it
  receives), and Task 3's menu items. `request_switch_project(EditorContext&, const std::string&)`
  has the same signature at its declaration (Task 2 Step 1), its two call sites (Task 1's modal,
  already written against this exact signature), and its definition (Task 2 Step 2).
- **Placeholder scan:** no TBD/TODO. Task 1 Step 4 and Task 2 Step 3 both call out real ordering
  dependencies between tasks explicitly (what breaks if built out of order, and the fallback) rather
  than papering over them — that is deliberate sequencing guidance, not an unresolved gap.
- **A genuine cross-task dependency, flagged rather than hidden:** Task 1's `project_picker.cpp`
  calls `request_switch_project`, which Task 2 defines. Tasks in this plan are meant to be done in
  order (1 → 2 → 3 → 4); Task 1 Step 5 gives the forward-declaration workaround for a reviewer who
  wants Task 1 buildable and reviewable in isolation anyway.
