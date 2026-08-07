# Project Selection — New/Load Project in the editor (`SushiEngine::Editor`)

**Status:** shipped, 2026-08-05 (§9).

`EditorContext::project_root` is resolved exactly once, at startup, and never reassigned anywhere
else in the editor: an artist gets whatever `default_projects_root()` computes or whatever the
preferences file already held, permanently, for the life of the process. There is no in-app way to
point the editor at a different project directory — the only way today is to hand-edit the
preferences JSON file on disk. This document adds `File ▸ New Project...`/`File ▸ Load Project...`,
reusing machinery that is already cross-platform and already built: the preferences store's config
directory, and the unsaved-changes guard every scene replacement already goes through.

Companion doc: `docs/design/editor_feature_sync_gaps.md` — the audit convention this document's §1
follows.

---

## §1 Audit — what exists today, and where it stops

- **`project_root` is set once and never again.** `applications/editor/source/main.cpp` resolves it
  from `context.preferences.last_project_root` or, when that is empty, `default_projects_root()`
  (`applications/editor/source/main.cpp`) — and nothing else in `applications/editor/` ever assigns
  `context.project_root`.
- **The default resolution is already cross-platform.** `default_projects_root()`
  (`applications/editor/source/main.cpp`) reads `USERPROFILE` on Windows and `HOME` elsewhere,
  landing on `<home>/sushiengine/project`.
- **Persistence is already cross-platform and already built.** `Preferences::last_project_root` and
  `::recent_scenes` (`engine/world/authoring/include/SushiEngine/authoring/preferences.hpp:58-136`)
  round-trip through `IPreferencesStore::save`/`::load`
  (`engine/world/authoring/include/SushiEngine/authoring/preferences.hpp`) to a config directory
  that is already `%APPDATA%/SushiEngine` on Windows and `$XDG_CONFIG_HOME` (or
  `~/.config/SushiEngine`) elsewhere
  (`engine/world/authoring/include/SushiEngine/authoring/preferences.hpp`).
  `context.preferences_dirty` (`applications/editor/source/core/editor_context.hpp`) is the
  established "persist this at the next frame boundary" flag — `applications/editor/source/main.cpp`
  is the one place that reads it and calls `save()`; every panel that changes a `Preferences` field
  sets it rather than saving directly (e.g. `applications/editor/source/scene/scene_commands.cpp`,
  `applications/editor/source/ui/editor_panels.cpp`).
- **No folder or file picker exists anywhere in the editor.** Neither a native OS dialog nor a
  custom one — `grep` for dialog/picker/chooser vocabulary across `applications/editor/` matches
  nothing.
- **The unsaved-changes guard this reuses.** `EditorContext::PendingSceneAction`
  (`applications/editor/source/core/editor_context.hpp:326-333`) parks a scene-replacing action when
  `scene_is_dirty(context)` (`:672`) is true; `request_new_scene`/`request_open_scene`
  (`applications/editor/source/scene/scene_commands.cpp`) raise it, `perform_pending_scene_action`
  (`:599-617`) resolves it once the unsaved-changes prompt clears the way, and
  `new_scene`/`open_scene` (`:551-596`) are the actions themselves.
- **A project-scoped path that would go stale on a silent switch.** `cook_bake_state`'s
  `cooking_profile.json` location is set from `project_root` only at startup
  (`applications/editor/source/main.cpp`); nothing re-points it if `project_root` changes later.
  `EditorContext` already holds the pointer this needs
  (`applications/editor/source/core/editor_context.hpp:264`,
  `Authoring::CookBakeState* cook_bake_state`).
- **Where the menu item belongs.** `File` (`applications/editor/source/ui/editor_panels.cpp`)
  already opens with `New Scene`/`Open Scene...`/`Open Recent`, in that order, each item routed
  through the same guard this document extends.
- **No test coverage exists for `applications/editor/` at all.** Confirmed by grep and by
  `tests/CMakeLists.txt` naming no editor source — not a gap this feature introduces, one it
  inherits; see §8.

## §2 Non-goals

- **No project manifest file.** Any directory is a valid project; `New Project` creates one if it
  does not exist and nothing more.
- **No recent-projects list.** `last_project_root` already remembers one; a list is not needed
  for the problem this document solves.
- **No native OS file dialog, and no new dependency.** §5 is a small directory-only browser built
  from `std::filesystem`, matching what the Project panel's own navigation already proves works
  cross-platform.
- **No project templates or asset scaffolding.**

## §3 State

`EditorContext` gains a picker's transient UI state:

```cpp
enum class ProjectPickerMode { New, Load };

bool show_project_picker = false;
ProjectPickerMode project_picker_mode = ProjectPickerMode::Load;
std::string project_picker_directory;    // where the modal is currently browsing
std::string project_picker_new_folder_name; // New mode only
```

`PendingSceneAction` (§1) gains a third value, `SwitchProject`, and a matching target field beside
the existing `pending_scene_open_path`:

```cpp
enum class PendingSceneAction { None, New, Open, SwitchProject };
std::string pending_project_switch_path;
```

## §4 File menu

Two items open the picker, inserted above `New Scene` in
`applications/editor/source/ui/editor_panels.cpp` as their own separated group:

- **New Project...** — `project_picker_mode = New`; `project_picker_directory` starts at
  `default_projects_root()`'s parent.
- **Load Project...** — `project_picker_mode = Load`; `project_picker_directory` starts at the
  current `project_root`'s parent.

## §5 The picker modal

A new, dedicated, directories-only browser — not a reuse of the Project panel's asset grid, which
draws files and thumbnails this has no use for. It lists only `is_directory` entries under
`project_picker_directory`, double-click enters one, an "Up" affordance goes to
`.parent_path()`, and the current path is shown as text. `New` mode adds a folder-name field and a
"Create & Select" button, which runs `std::filesystem::create_directories(directory / name)` before
confirming; `Load` mode has a single "Select This Folder" button confirming
`project_picker_directory` itself, with no creation step.

Confirming either calls `request_switch_project(context, chosen_path)` (§6) and closes the modal.

## §6 Switching

Mirrors `request_new_scene`/`request_open_scene`
(`applications/editor/source/scene/scene_commands.cpp`) exactly — a project switch discards the
current scene the same way a new/opened one does, so it goes through the identical guard:

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

`perform_pending_scene_action` (`:602-617`) gets a third `case`, calling `switch_project` the same
way its `Open` case calls `open_scene`, and clearing `pending_project_switch_path` afterward the
same way the existing cleanup already clears `pending_scene_open_path` (`:616`). `switch_project`
itself:

```cpp
void switch_project(EditorContext& context, const std::string& new_root)
{
    new_scene(context); // the old project's scene has nothing left to belong to
    context.project_root = new_root;
    context.current_directory = new_root;
    context.preferences.last_project_root = new_root;
    context.preferences_dirty = true; // main.cpp:1249-1254 persists it next frame
    if (context.cook_bake_state != nullptr)
    {
        context.cook_bake_state->set_profile_storage_path(
            (std::filesystem::path(new_root) / "cooking_profile.json").string());
        context.cook_bake_state->load_profiles();
    }
    editor_log(context, "Switched project to '" + new_root + "'.");
}
```

## §7 Persistence

No new mechanism. `context.preferences_dirty = true` (§6) is the same flag every other `Preferences`
field change already raises (§1); the existing once-per-frame flush
(`applications/editor/source/main.cpp`) writes `last_project_root` through `IPreferencesStore::save`
exactly as it already does for every other preference.

## §8 Testing

`applications/editor/` carries no unit tests today (§1) — not specific to Crowd, Soft Body, or any
other feature already in that tier, and not a gap this document takes on. `switch_project`/
`request_switch_project` are written as free functions over an explicit `EditorContext&` rather
than buried inside a draw call specifically so they are not harder to test than the rest of this
tier already is, but adding the first test harness for `applications/editor/` is a separate,
larger, cross-cutting piece of work and out of scope here.

## §9 Roadmap

P0 — this document's entire scope (§3-§7) — **complete**, built via
`docs/superpowers/plans/2026-08-05-project-selection.md`, four tasks plus a final-review fix
wave, all task-scoped and reviewed.

The final whole-branch review found and closed five real gaps §3-§7's illustrative code left open:
`switch_project` did not validate `new_root` as an existing directory before persisting it into
`preferences.last_project_root` (a rejected/deleted target would have reinstated exactly the
"hand-edit the preferences file" failure mode this document exists to remove), did not reset
`CookBakeState`'s cooking-profile settings before loading the new project's own (so a fidelity dial
and per-asset overrides could silently carry across a switch), and the picker's directory listing
used the throwing `directory_iterator` increment and left a `create_directories` failure unreported.
`applications/editor/source/project/project_picker.cpp`'s modal also called `ImGui::EndPopup()` on
the path where `BeginPopupModal` already returns false and closes it internally — reachable by the
modal's own close button, and a real contract violation, not a hypothetical one. All five are fixed
on the shipped branch; `switch_project` validates before mutating any state and resets the profile
set, and the picker uses the non-throwing iterator form throughout. Navigation in the shipped picker
is single-click, not the double-click §5 describes — the one interaction detail that changed during
implementation.
