# Project Panel View Overhaul (Phase 1) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the Project window's fixed-size, flat-colour tile grid with a Unity-style
browser: big vector-icon tiles by asset category, a Ctrl+scroll zoom, and a toggleable compact
list view — persisted across sessions.

**Architecture:** `applications/editor/source/project/project_panel.cpp` gains a category
classifier and a vector glyph painter (mirroring the existing `draw_toolbar_icon`/`icon_button`
pattern in `panel_widgets.cpp`), a shared per-entry interaction handler used by both a grid
view and a new list view, and a toolbar that switches between them. View mode and zoom persist
through two new `Authoring::Preferences` fields, following the exact load/save pattern already
used for `gizmo_mode`/`gizmo_space` in `preferences.cpp`.

**Tech Stack:** C++17, Dear ImGui (immediate mode, `ImDrawList` primitives), nlohmann::json
(preferences persistence). No new dependencies.

## Global Constraints

- Zoom range: tile size clamped to **[48, 160]** pixels; default **76** (today's fixed constant).
- Zoom step: **8px per wheel notch**, only while Ctrl is held and the grid is hovered, only in
  grid view.
- Real image thumbnails are **out of scope** for this plan — every category, images included,
  gets a procedurally-drawn vector glyph (Phase 2 spec, not this one).
- List view row size is fixed (not affected by zoom), matching Unity's List View.
- No icon font, no bitmap atlas, no new third-party dependency — every glyph is drawn with
  `ImDrawList` primitives, the same convention `draw_toolbar_icon` in
  `applications/editor/source/ui/panel_widgets.cpp:180`-ish already establishes for the toolbar.
- Preferences are a plain JSON aggregate read with `json.value(key, default)` — a preferences
  file written before this change must still load cleanly, defaulting the two new fields.
- **No automated test exists for any ImGui panel in this codebase** (confirmed: `sushiengine_editor`
  in `applications/editor/CMakeLists.txt` is a monolithic `add_executable`, not a library the
  `sushiengine_functional_tests` target links against). Every task's verification step is
  therefore "build via `se editor --no-run`, then the user manually checks the exact behaviour
  listed" — this is not a shortcut, it is the pattern the approved spec's own Testing section
  specifies. Do not invent test files that cannot compile into any target.
- Builds and the running editor are the user's to execute (`se editor`, never raw
  cmake/ninja) — an implementer proposes the build/verify command, the user runs it and reports
  the result.

---

### Task 1: Persisted view-mode and zoom preferences

**Files:**
- Modify: `engine/world/authoring/include/SushiEngine/authoring/preferences.hpp:42-136`
- Modify: `engine/world/authoring/source/preferences.cpp:111-139` (enum string helpers),
  `:603-651` (load), `:653-`(save, the `json[...]  = ...` block around line 676)

**Interfaces:**
- Produces: `enum class SushiEngine::Authoring::ProjectBrowserViewMode { Grid, List };`,
  `Preferences::project_view_mode` (`ProjectBrowserViewMode`, default `Grid`),
  `Preferences::project_tile_size` (`float`, default `76.0f`).

- [ ] **Step 1: Add the enum and the two fields to `Preferences`**

In `preferences.hpp`, add the enum right after `EditorTheme` (line 48):

```cpp
        /** @brief Which layout the Project window's browser draws: icon grid or compact rows. */
        enum class ProjectBrowserViewMode
        {
            Grid,
            List
        };
```

Then, inside `struct Preferences`, immediately after `gizmo_space` (preferences.hpp:135), add:

```cpp
            /** @brief The Project window's browser layout, restored on start. */
            ProjectBrowserViewMode project_view_mode = ProjectBrowserViewMode::Grid;

            /** @brief The Project window's icon size in pixels, restored on start (Ctrl+scroll to zoom). */
            float project_tile_size = 76.0f;
```

- [ ] **Step 2: Add the string conversion helpers in `preferences.cpp`**

Immediately after `gizmo_space_from` (preferences.cpp:139), add:

```cpp
            const char* to_string(ProjectBrowserViewMode mode) noexcept
            {
                return mode == ProjectBrowserViewMode::List ? "list" : "grid";
            }

            ProjectBrowserViewMode project_view_mode_from(const std::string& value) noexcept
            {
                return value == "list" ? ProjectBrowserViewMode::List
                                       : ProjectBrowserViewMode::Grid;
            }
```

- [ ] **Step 3: Load the two fields**

In the `load()` body, immediately after the `gizmo_space` line (preferences.cpp:644-645), add:

```cpp
                        preferences.project_view_mode = project_view_mode_from(
                            json.value("project_view_mode", to_string(preferences.project_view_mode)));
                        preferences.project_tile_size = json.value(
                            "project_tile_size", preferences.project_tile_size);
```

- [ ] **Step 4: Save the two fields**

In the `save()` body, immediately after the `json["gizmo_space"] = ...` line (preferences.cpp:677),
add:

```cpp
                        json["project_view_mode"] = to_string(preferences.project_view_mode);
                        json["project_tile_size"] = preferences.project_tile_size;
```

- [ ] **Step 5: Build to verify it compiles**

Ask the user to run: `se editor --no-run`
Expected: builds clean, no new warnings from `preferences.cpp`/`preferences.hpp`.

- [ ] **Step 6: Commit**

```bash
git add engine/world/authoring/include/SushiEngine/authoring/preferences.hpp engine/world/authoring/source/preferences.cpp
git commit -m "feat(editor): persist the Project window's view mode and zoom level"
```

---

### Task 2: Asset category classifier

**Files:**
- Modify: `applications/editor/source/project/project_panel.cpp:251-262` (replaces `tile_color`)

**Interfaces:**
- Consumes: nothing new (uses `has_text_extension` already defined at `project_panel.cpp:140`).
- Produces: `enum class EntryCategory { Folder, Scene, Prefab, Model, Image, Audio, Code, Text, Unknown };`
  and `EntryCategory entry_category(const fs::path& path, bool is_dir);`, both in the file's
  anonymous namespace. Task 3 consumes `EntryCategory` by name.

- [ ] **Step 1: Replace `tile_color` with the classifier**

Replace the `tile_color` function (`project_panel.cpp:251-262`) with:

```cpp
            // The Project browser's asset kinds, one glyph and one meaning per entry. Checked
            // in order of specificity: an extension that is also handled by has_text_extension
            // (e.g. a shader) must not be reclassified as plain Text before Code/Scene/etc. get
            // a look, so Folder/Scene/Prefab/Model/Image/Audio/Code are all decided first.
            enum class EntryCategory
            {
                Folder,
                Scene,
                Prefab,
                Model,
                Image,
                Audio,
                Code,
                Text,
                Unknown
            };

            EntryCategory entry_category(const fs::path& path, bool is_dir)
            {
                if (is_dir)
                    return EntryCategory::Folder;

                const std::string ext = to_lower(path.extension().string());
                if (ext == ".sushiscene")
                    return EntryCategory::Scene;
                if (ext == ".sushiprefab")
                    return EntryCategory::Prefab;
                if (ext == ".gltf" || ext == ".glb" || ext == ".fbx" || ext == ".obj")
                    return EntryCategory::Model;
                if (ext == ".png" || ext == ".jpg" || ext == ".jpeg" || ext == ".tga" ||
                    ext == ".bmp" || ext == ".hdr" || ext == ".exr")
                    return EntryCategory::Image;
                if (ext == ".wav" || ext == ".mp3" || ext == ".ogg" || ext == ".flac")
                    return EntryCategory::Audio;
                if (ext == ".cpp" || ext == ".cc" || ext == ".hpp" || ext == ".h" || ext == ".inl")
                    return EntryCategory::Code;
                if (has_text_extension(path))
                    return EntryCategory::Text;
                return EntryCategory::Unknown;
            }
```

- [ ] **Step 2: Fix the one call site that used `tile_color`**

`project_panel.cpp:487-490` currently reads:

```cpp
                    const ImU32 color = tile_color(entry.path(), is_dir);
                    ImGui::PushStyleColor(ImGuiCol_Button, color);
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, color);
                    ImGui::PushStyleColor(ImGuiCol_ButtonActive, color);
```

Leave this block as-is for now — Task 4 replaces the whole tile-drawing block it belongs to,
including these three lines. This task only needs the file to compile, so temporarily change
just the first line to keep it valid:

```cpp
                    const ImU32 color = IM_COL32(90, 90, 90, 255); // replaced by draw_entry_icon in Task 4
```

- [ ] **Step 3: Build to verify it compiles**

Ask the user to run: `se editor --no-run`
Expected: builds clean. `entry_category` is otherwise unused until Task 3/4 — expect a possible
"unused function" warning, which Task 4 resolves by calling it.

- [ ] **Step 4: Commit**

```bash
git add applications/editor/source/project/project_panel.cpp
git commit -m "refactor(editor): classify Project browser entries by asset category"
```

---

### Task 3: Vector icon glyphs per category

**Files:**
- Modify: `applications/editor/source/project/project_panel.cpp` (add `draw_entry_icon`,
  in the anonymous namespace, after `entry_category` from Task 2)

**Interfaces:**
- Consumes: `EntryCategory` (Task 2).
- Produces: `void draw_entry_icon(ImDrawList* list, ImVec2 origin, float size, EntryCategory category, ImU32 color);`
  — paints into the `size`×`size` box whose top-left is `origin`. Task 4 consumes this by name.

- [ ] **Step 1: Add the glyph painter**

Add, after `entry_category`:

```cpp
            // One flat vector glyph per EntryCategory, in the same drawn-not-fonted spirit as
            // draw_toolbar_icon (panel_widgets.cpp): no bitmap or font asset to ship, and a
            // vector shape stays crisp at every zoom level Task 5 allows. `size` is the tile's
            // full icon box; every glyph is drawn centred inside it with a margin so it never
            // touches the box edge.
            void draw_entry_icon(ImDrawList* list, ImVec2 origin, float size,
                                 EntryCategory category, ImU32 color)
            {
                const float margin = size * 0.15f;
                const ImVec2 c{origin.x + size * 0.5f, origin.y + size * 0.5f};
                const float r = size * 0.5f - margin;
                const float thickness = std::max(1.5f, size * 0.03f);

                switch (category)
                {
                    case EntryCategory::Folder:
                    {
                        // A tabbed folder: a small tab rect atop a wider body rect.
                        const ImVec2 body_min{c.x - r, c.y - r * 0.5f};
                        const ImVec2 body_max{c.x + r, c.y + r};
                        list->AddRectFilled(ImVec2(c.x - r, c.y - r), ImVec2(c.x - r * 0.2f, c.y - r * 0.5f), color, 1.0f);
                        list->AddRectFilled(body_min, body_max, color, 2.0f);
                        break;
                    }
                    case EntryCategory::Scene:
                    {
                        // A page with a folded corner and a small play-triangle mark.
                        list->AddRect(ImVec2(c.x - r * 0.7f, c.y - r), ImVec2(c.x + r * 0.7f, c.y + r), color, 1.0f, ImDrawFlags_None, thickness);
                        list->AddTriangleFilled(ImVec2(c.x - r * 0.25f, c.y - r * 0.35f), ImVec2(c.x - r * 0.25f, c.y + r * 0.35f), ImVec2(c.x + r * 0.35f, c.y), color);
                        break;
                    }
                    case EntryCategory::Prefab:
                    {
                        // A hexagon — Unity's own prefab convention, distinct from every other glyph here.
                        ImVec2 points[6];
                        for (int i = 0; i < 6; ++i)
                        {
                            const float angle = static_cast<float>(i) / 6.0f * 6.28318530718f - 1.5707963f;
                            points[i] = ImVec2(c.x + r * std::cos(angle), c.y + r * std::sin(angle));
                        }
                        list->AddConvexPolyFilled(points, 6, color);
                        break;
                    }
                    case EntryCategory::Model:
                    {
                        // A wireframe cube: a front square, a back square offset up-right, four connectors.
                        const float o = r * 0.4f;
                        const ImVec2 f0{c.x - r * 0.7f, c.y - r * 0.4f}, f1{c.x + r * 0.2f, c.y - r * 0.4f};
                        const ImVec2 f2{c.x + r * 0.2f, c.y + r * 0.85f}, f3{c.x - r * 0.7f, c.y + r * 0.85f};
                        const ImVec2 b0{f0.x + o, f0.y - o}, b1{f1.x + o, f1.y - o};
                        const ImVec2 b2{f2.x + o, f2.y - o}, b3{f3.x + o, f3.y - o};
                        list->AddQuad(f0, f1, f2, f3, color, thickness);
                        list->AddQuad(b0, b1, b2, b3, color, thickness);
                        list->AddLine(f0, b0, color, thickness);
                        list->AddLine(f1, b1, color, thickness);
                        list->AddLine(f2, b2, color, thickness);
                        list->AddLine(f3, b3, color, thickness);
                        break;
                    }
                    case EntryCategory::Image:
                    {
                        // A picture frame with a corner sun and a mountain fold — the classic
                        // "image" glyph, and the placeholder a future real thumbnail replaces.
                        list->AddRect(ImVec2(c.x - r, c.y - r * 0.75f), ImVec2(c.x + r, c.y + r * 0.75f), color, 1.0f, ImDrawFlags_None, thickness);
                        list->AddCircleFilled(ImVec2(c.x - r * 0.5f, c.y - r * 0.3f), r * 0.18f, color);
                        list->AddTriangleFilled(ImVec2(c.x - r * 0.7f, c.y + r * 0.6f), ImVec2(c.x - r * 0.1f, c.y - r * 0.1f), ImVec2(c.x + r * 0.4f, c.y + r * 0.6f), color);
                        list->AddTriangleFilled(ImVec2(c.x + r * 0.1f, c.y + r * 0.6f), ImVec2(c.x + r * 0.5f, c.y + r * 0.1f), ImVec2(c.x + r * 0.85f, c.y + r * 0.6f), color);
                        break;
                    }
                    case EntryCategory::Audio:
                    {
                        // A speaker body plus two arcs of increasing radius: a waveform mark.
                        list->AddTriangleFilled(ImVec2(c.x - r * 0.85f, c.y - r * 0.3f), ImVec2(c.x - r * 0.85f, c.y + r * 0.3f), ImVec2(c.x - r * 0.35f, c.y + r * 0.3f), color);
                        list->AddRectFilled(ImVec2(c.x - r * 0.95f, c.y - r * 0.3f), ImVec2(c.x - r * 0.55f, c.y + r * 0.3f), color, 1.0f);
                        list->PathArcTo(ImVec2(c.x - r * 0.35f, c.y), r * 0.5f, -0.6f, 0.6f, 12);
                        list->PathStroke(color, ImDrawFlags_None, thickness);
                        list->PathArcTo(ImVec2(c.x - r * 0.35f, c.y), r * 0.85f, -0.6f, 0.6f, 12);
                        list->PathStroke(color, ImDrawFlags_None, thickness);
                        break;
                    }
                    case EntryCategory::Code:
                    {
                        // A page with a "</>" mark.
                        list->AddRect(ImVec2(c.x - r * 0.7f, c.y - r), ImVec2(c.x + r * 0.7f, c.y + r), color, 1.0f, ImDrawFlags_None, thickness);
                        list->AddLine(ImVec2(c.x - r * 0.4f, c.y - r * 0.15f), ImVec2(c.x - r * 0.6f, c.y + r * 0.15f), color, thickness);
                        list->AddLine(ImVec2(c.x - r * 0.6f, c.y + r * 0.15f), ImVec2(c.x - r * 0.4f, c.y + r * 0.4f), color, thickness);
                        list->AddLine(ImVec2(c.x + r * 0.4f, c.y - r * 0.15f), ImVec2(c.x + r * 0.6f, c.y + r * 0.15f), color, thickness);
                        list->AddLine(ImVec2(c.x + r * 0.6f, c.y + r * 0.15f), ImVec2(c.x + r * 0.4f, c.y + r * 0.4f), color, thickness);
                        break;
                    }
                    case EntryCategory::Text:
                    case EntryCategory::Unknown:
                        // A plain page; Unknown adds no decoration, Text stops here too by design.
                        list->AddRect(ImVec2(c.x - r * 0.7f, c.y - r), ImVec2(c.x + r * 0.7f, c.y + r), color, 1.0f, ImDrawFlags_None, thickness);
                        break;
                }
            }
```

Add `#include <cmath>` near the top of `project_panel.cpp` if not already present (check the
existing include block at lines 35-43 — it currently has no `<cmath>`; `std::cos`/`std::sin`
above need it).

- [ ] **Step 2: Build to verify it compiles**

Ask the user to run: `se editor --no-run`
Expected: builds clean. `draw_entry_icon` is unused until Task 4 — an "unused function" warning
here is expected and resolved next task.

- [ ] **Step 3: Commit**

```bash
git add applications/editor/source/project/project_panel.cpp
git commit -m "feat(editor): draw a distinct vector glyph per Project browser asset category"
```

---

### Task 4: Grid view uses the classifier and glyphs, at a dynamic tile size

**Files:**
- Modify: `applications/editor/source/project/project_panel.cpp:371-626` (`draw_project_panel`
  body: constants at 447-448, the per-entry loop at 456-572, the popup-context-window block at
  575-586, the delete-target handling at 588-604)

**Interfaces:**
- Consumes: `EntryCategory`/`entry_category` (Task 2), `draw_entry_icon` (Task 3),
  `context.preferences.project_tile_size` (Task 1).
- Produces: `void draw_project_entry_interactions(EditorContext& context, const fs::directory_entry& entry, bool is_dir, const fs::path& current, fs::path& delete_target);`
  — called immediately after the entry's clickable widget is drawn (so `ImGui::IsItemHovered`/
  `IsItemClicked`/`BeginPopupContextItem` see that widget). Handles: click-to-select, the
  folder-relative-path tooltip, drag source, double-click dispatch, and the right-click context
  menu (Open/Cooking Override/Rename/Delete/Show in Explorer/Create). Task 6 (list view) reuses
  this unchanged. `void draw_project_grid_view(EditorContext& context, const std::vector<fs::directory_entry>& entries, const fs::path& current, fs::path& delete_target);`
  — the grid rendering loop, extracted out of `draw_project_panel`.

- [ ] **Step 1: Extract the shared interaction handler**

The current per-entry block mixes drawing with interaction handling. Pull the interaction half
(lines 496-568 today, everything from the `if (clicked)` after the button through the closing
`}` of `BeginPopupContextItem`) into a new function, placed in the anonymous namespace right
before `draw_project_panel`:

```cpp
            // The behaviour behind one Project browser entry, independent of whether it was
            // drawn as a grid tile or a list row: both call this immediately after drawing
            // their own clickable widget, so ImGui's "the last item" state (IsItemHovered,
            // IsItemClicked, BeginPopupContextItem) refers to whichever widget the caller drew.
            void draw_project_entry_interactions(EditorContext& context,
                                                 const fs::directory_entry& entry, bool is_dir,
                                                 const fs::path& current, fs::path& delete_target)
            {
                const std::string path_string = entry.path().string();

                // Dragging a file out of the browser is how an asset reaches a slot that
                // wants one; directories are not draggable because nothing accepts one.
                if (!is_dir)
                    set_asset_drag_source(path_string, entry.path().filename().string());

                // The tile/row shows a truncated or plain filename, which is ambiguous the
                // moment a search returns two files of the same name from different folders —
                // so the full path relative to the browsed folder is always one hover away.
                if (ImGui::IsItemHovered())
                {
                    std::error_code relative_ec;
                    const fs::path shown = fs::relative(entry.path(), current, relative_ec);
                    ImGui::SetTooltip("%s", relative_ec ? path_string.c_str() : shown.string().c_str());
                }

                const auto open_entry = [&]()
                {
                    context.selected_project_path = path_string;
                    if (is_dir)
                        context.current_directory = path_string;
                    else if (entry.path().extension() == ".sushiscene")
                        request_open_scene(context, path_string);
                    else if (has_character_extension(entry.path()))
                        open_character_in_preview(context, entry.path());
                    else if (has_text_extension(entry.path()))
                        open_document(context, entry.path());
                    else
                        open_with_default_app(entry.path());
                };

                // Double-click detection is independent of a Button's own pressed-on-release
                // return, which can miss the second click of a fast double-click; hover +
                // IsMouseDoubleClicked is the reliable pair, and works the same for a Selectable.
                if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
                    open_entry();

                if (ImGui::BeginPopupContextItem())
                {
                    context.selected_project_path = path_string;
                    if (!is_dir && ImGui::MenuItem("Open"))
                        open_entry();
                    if (has_character_extension(entry.path()) && context.cook_bake_state != nullptr &&
                        ImGui::MenuItem("Cooking Override..."))
                        context.cooking_override_target = path_string;
                    if (ImGui::MenuItem("Rename"))
                        context.renaming_project_path = path_string;
                    if (ImGui::MenuItem("Delete"))
                        delete_target = entry.path();
                    if (ImGui::MenuItem("Show in Explorer", nullptr, false, SHELL_INTEGRATION_AVAILABLE))
                        show_in_explorer(entry.path());
                    if (!SHELL_INTEGRATION_AVAILABLE &&
                        ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
                        ImGui::SetTooltip("Windows-only for now.");
                    ImGui::Separator();
                    draw_project_create_menu(context, current);
                    ImGui::EndPopup();
                }
            }
```

Note `open_entry()` folds in the plain-click `context.selected_project_path = path_string`
assignment too (the original code set it both on a plain click and again on double-click/menu
open — this keeps that on double-click/menu-open only; the plain click's own `selected_project_path`
assignment stays at the call site in Step 2, since that happens on every click, not just a double).

- [ ] **Step 2: Rewrite the grid loop to use it, `entry_category`, and `draw_entry_icon`**

Replace the whole per-entry loop (`project_panel.cpp:456-572`) with a call to a new
`draw_project_grid_view`, and define that function in the anonymous namespace right after
`draw_project_entry_interactions`:

```cpp
            // The Unity-style icon grid: one square tile per entry, laid out left-to-right and
            // wrapped at the available width, at whatever size context.preferences.project_tile_size
            // currently holds (Task 5 changes that value; this function only reads it).
            void draw_project_grid_view(EditorContext& context,
                                        const std::vector<fs::directory_entry>& entries,
                                        const fs::path& current, fs::path& delete_target)
            {
                const float tile_size = context.preferences.project_tile_size;
                constexpr float TILE_SPACING = 8.0f;
                const float avail_width = ImGui::GetContentRegionAvail().x;
                float row_x = 0.0f;

                for (std::size_t i = 0; i < entries.size(); ++i)
                {
                    const fs::directory_entry& entry = entries[i];
                    const bool is_dir = entry.is_directory();
                    const std::string path_string = entry.path().string();
                    const std::string name = entry.path().filename().string();

                    if (row_x + tile_size > avail_width && row_x > 0.0f)
                        row_x = 0.0f;
                    else if (i > 0 && row_x > 0.0f)
                        ImGui::SameLine();
                    row_x += tile_size + TILE_SPACING;

                    ImGui::PushID(path_string.c_str());
                    ImGui::BeginGroup();

                    if (context.renaming_project_path == path_string)
                    {
                        ImGui::Dummy(ImVec2(tile_size, tile_size * 0.6f));
                        std::string entered;
                        if (inline_rename_field(context, path_string, name, tile_size, entered))
                        {
                            std::error_code rename_ec;
                            const fs::path renamed = entry.path().parent_path() / entered;
                            if (!entered.empty() && renamed != entry.path())
                                fs::rename(entry.path(), renamed, rename_ec);
                            context.renaming_project_path.clear();
                        }
                    }
                    else
                    {
                        const ImVec2 origin = ImGui::GetCursorScreenPos();
                        const float icon_size = tile_size * 0.75f;
                        const bool clicked =
                            ImGui::InvisibleButton("##tile", ImVec2(tile_size, tile_size));
                        if (clicked)
                            context.selected_project_path = path_string;

                        ImDrawList* draw_list = ImGui::GetWindowDrawList();
                        const bool selected = context.selected_project_path == path_string;
                        if (selected || ImGui::IsItemHovered())
                            draw_list->AddRectFilled(
                                origin, ImVec2(origin.x + tile_size, origin.y + tile_size),
                                ImGui::GetColorU32(selected ? ImGuiCol_ButtonActive : ImGuiCol_ButtonHovered),
                                3.0f);
                        draw_entry_icon(draw_list, ImVec2(origin.x + (tile_size - icon_size) * 0.5f, origin.y),
                                       icon_size, entry_category(entry.path(), is_dir),
                                       ImGui::GetColorU32(ImGuiCol_Text));
                        const std::string label = truncate_label(name);
                        const ImVec2 text_size = ImGui::CalcTextSize(label.c_str());
                        draw_list->AddText(ImVec2(origin.x + (tile_size - text_size.x) * 0.5f, origin.y + icon_size + 2.0f),
                                          ImGui::GetColorU32(ImGuiCol_Text), label.c_str());

                        draw_project_entry_interactions(context, entry, is_dir, current, delete_target);
                    }

                    ImGui::EndGroup();
                    ImGui::PopID();
                }
            }
```

Then, inside `draw_project_panel`, replace the whole block from the `constexpr float TILE_SIZE`
line (`project_panel.cpp:447`) through the end of the entry loop (`project_panel.cpp:572`) with:

```cpp
            fs::path delete_target;
            draw_project_grid_view(context, entries, current, delete_target);
```

(`delete_target` was already declared at `project_panel.cpp:454` as `fs::path delete_target;`
immediately before the old loop — keep exactly one declaration; the snippet above shows where
it now lives relative to the new call.)

Remove the now-dead `tile_color`-replacement stub from Task 2 Step 2 — `draw_entry_icon` fully
replaces it, so delete the `const ImU32 color = IM_COL32(90, 90, 90, 255);` line and the three
`PushStyleColor`/`PopStyleColor(3)` lines and the old `ImGui::Button(label.c_str(), ...)` call;
they no longer exist once this task's rewrite lands (they were part of the block just replaced).

- [ ] **Step 3: Build to verify it compiles**

Ask the user to run: `se editor --no-run`
Expected: builds clean, no leftover references to the deleted inline block or the Task-2 stub.

- [ ] **Step 4: Manual verification**

Ask the user to run `se editor`, open the Project window, and confirm:
- Every folder shows a folder-tab glyph; a `.cpp`/`.hpp` shows the code glyph; a `.sushiscene`
  shows the scene glyph; a `.sushiprefab` shows the hexagon; a `.png`/`.jpg` shows the picture
  glyph; anything else shows a plain page.
- Clicking a tile selects it (highlight shows); double-clicking a folder descends into it;
  double-clicking a `.sushiscene` opens it; right-click still shows the full context menu
  (Open/Rename/Delete/Show in Explorer/Create); rename-in-place still works; dragging a file
  out still offers it as an asset drop elsewhere (e.g. onto a Material texture slot).
- Tiles are still 76px (Task 5 is what makes this configurable) and still wrap correctly at the
  panel's width.

- [ ] **Step 5: Commit**

```bash
git add applications/editor/source/project/project_panel.cpp
git commit -m "refactor(editor): draw Project browser tiles as icon glyphs at a dynamic size"
```

---

### Task 5: Ctrl+scroll zoom

**Files:**
- Modify: `applications/editor/source/project/project_panel.cpp` (inside `draw_project_panel`,
  right after the `project_grid` child window begins — see the `ImGui::BeginChild("project_grid", ...)`
  call, currently `project_panel.cpp:404`, now shifted by Task 4's edits — search for it rather
  than trusting the line number)

**Interfaces:**
- Consumes: `context.preferences.project_tile_size` (Task 1).
- Produces: nothing new exported; the zoom is applied in place before `draw_project_grid_view`
  runs so it reads the already-updated size the same frame.

- [ ] **Step 1: Add the zoom handler**

Immediately after `ImGui::BeginChild("project_grid", ImVec2(0.0f, 0.0f), true);`, add:

```cpp
            // Ctrl+scroll zooms the icon grid, Unity-style; consuming the wheel value here
            // (zeroing it) stops ImGui's own child-scroll logic from also scrolling the list
            // on the same wheel event later this frame.
            if (context.preferences.project_view_mode == Authoring::ProjectBrowserViewMode::Grid &&
                ImGui::IsWindowHovered() && ImGui::GetIO().KeyCtrl && ImGui::GetIO().MouseWheel != 0.0f)
            {
                constexpr float MIN_TILE_SIZE = 48.0f;
                constexpr float MAX_TILE_SIZE = 160.0f;
                constexpr float ZOOM_STEP = 8.0f;
                context.preferences.project_tile_size = std::clamp(
                    context.preferences.project_tile_size + ImGui::GetIO().MouseWheel * ZOOM_STEP,
                    MIN_TILE_SIZE, MAX_TILE_SIZE);
                ImGui::GetIO().MouseWheel = 0.0f;
            }
```

Add `#include <algorithm>` to `project_panel.cpp` if not already present (it is — line 35 of
the original file already includes it, for `std::sort`/`std::transform`).

- [ ] **Step 2: Build to verify it compiles**

Ask the user to run: `se editor --no-run`

- [ ] **Step 3: Manual verification**

Ask the user to run `se editor`, hover the Project grid, and confirm:
- Holding Ctrl and scrolling up grows the tiles, scrolling down shrinks them.
- The size stops changing at very large and very small tiles (clamped to roughly 48–160px —
  eyeball it, no exact pixel check needed).
- Scrolling *without* Ctrl still scrolls the grid vertically as before (unaffected).
- Closing and reopening the editor keeps the zoom level from before close (Task 1's
  persistence).

- [ ] **Step 4: Commit**

```bash
git add applications/editor/source/project/project_panel.cpp
git commit -m "feat(editor): Ctrl+scroll zooms the Project browser's icon grid"
```

---

### Task 6: List view and the Grid/List toolbar toggle

**Files:**
- Modify: `applications/editor/source/project/project_panel.cpp` (add `draw_project_list_view`
  beside `draw_project_grid_view`; add the toolbar toggle in `draw_project_panel`, beside the
  existing search field at `project_panel.cpp:406-407`)

**Interfaces:**
- Consumes: `draw_project_entry_interactions` (Task 4), `entry_category`/`draw_entry_icon`
  (Tasks 2-3), `context.preferences.project_view_mode` (Task 1).
- Produces: `void draw_project_list_view(EditorContext& context, const std::vector<fs::directory_entry>& entries, const fs::path& current, fs::path& delete_target);`

- [ ] **Step 1: Add the list view function**

Add beside `draw_project_grid_view`:

```cpp
            // Unity's List View: one compact row per entry, a small fixed-size glyph before
            // the name, unaffected by the grid's zoom (Unity's own list rows are a fixed size).
            void draw_project_list_view(EditorContext& context,
                                        const std::vector<fs::directory_entry>& entries,
                                        const fs::path& current, fs::path& delete_target)
            {
                constexpr float ROW_ICON_SIZE = 16.0f;
                for (const fs::directory_entry& entry : entries)
                {
                    const bool is_dir = entry.is_directory();
                    const std::string path_string = entry.path().string();
                    const std::string name = entry.path().filename().string();

                    ImGui::PushID(path_string.c_str());

                    if (context.renaming_project_path == path_string)
                    {
                        std::string entered;
                        if (inline_rename_field(context, path_string, name, -FLT_MIN, entered))
                        {
                            std::error_code rename_ec;
                            const fs::path renamed = entry.path().parent_path() / entered;
                            if (!entered.empty() && renamed != entry.path())
                                fs::rename(entry.path(), renamed, rename_ec);
                            context.renaming_project_path.clear();
                        }
                    }
                    else
                    {
                        const bool selected = context.selected_project_path == path_string;
                        const ImVec2 origin = ImGui::GetCursorScreenPos();
                        // Reserve room for the icon; the label is drawn by Selectable itself,
                        // indented past the icon column.
                        ImGui::Dummy(ImVec2(ROW_ICON_SIZE + 4.0f, 0.0f));
                        ImGui::SameLine();
                        if (ImGui::Selectable((name + "##row").c_str(), selected))
                            context.selected_project_path = path_string;
                        ImDrawList* draw_list = ImGui::GetWindowDrawList();
                        draw_entry_icon(draw_list, origin, ROW_ICON_SIZE,
                                       entry_category(entry.path(), is_dir),
                                       ImGui::GetColorU32(ImGuiCol_Text));

                        draw_project_entry_interactions(context, entry, is_dir, current, delete_target);
                    }

                    ImGui::PopID();
                }
            }
```

- [ ] **Step 2: Add the toolbar toggle and the view-mode dispatch**

In `draw_project_panel`, immediately before the search field
(`ImGui::InputTextWithHint("##project_filter", ...)`, currently `project_panel.cpp:407`), add:

```cpp
            using Authoring::ProjectBrowserViewMode;
            bool is_grid = context.preferences.project_view_mode == ProjectBrowserViewMode::Grid;
            if (ImGui::RadioButton("Grid", is_grid))
                context.preferences.project_view_mode = ProjectBrowserViewMode::Grid;
            ImGui::SameLine();
            if (ImGui::RadioButton("List", !is_grid))
                context.preferences.project_view_mode = ProjectBrowserViewMode::List;
```

Then replace the `draw_project_grid_view(context, entries, current, delete_target);` call added
in Task 4 Step 2 with:

```cpp
            fs::path delete_target;
            if (context.preferences.project_view_mode == ProjectBrowserViewMode::Grid)
                draw_project_grid_view(context, entries, current, delete_target);
            else
                draw_project_list_view(context, entries, current, delete_target);
```

- [ ] **Step 3: Build to verify it compiles**

Ask the user to run: `se editor --no-run`

- [ ] **Step 4: Manual verification**

Ask the user to run `se editor`, open the Project window, and confirm:
- Grid/List radio buttons switch the view immediately.
- List view shows a small icon and the name per row, full-row selectable, at a fixed compact
  height regardless of the grid's zoom level.
- Rename, double-click-open, drag-out, delete, and the right-click context menu all behave the
  same in List view as in Grid view.
- Closing and reopening the editor keeps whichever view was active (Task 1's persistence).

- [ ] **Step 5: Commit**

```bash
git add applications/editor/source/project/project_panel.cpp
git commit -m "feat(editor): add a List view to the Project browser, toggled from its toolbar"
```

---

## Explicitly out of scope (do not implement here)

- Real image thumbnails — a follow-up Phase 2 spec, once this lands and is reviewed.
- Sortable Explorer-style detail columns (Type/Size/Modified) in List view.
- Multi-select in either view (the panel already tracks only one `selected_project_path`, and
  this plan does not change that).
