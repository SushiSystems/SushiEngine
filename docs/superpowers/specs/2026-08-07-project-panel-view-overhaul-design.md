# Project panel view overhaul (Phase 1: grid/list, zoom, type icons)

**Status:** designed, 2026-08-07.

## Problem

`applications/editor/source/project/project_panel.cpp` draws the Project window's asset grid
with a fixed 76px `ImGui::Button` per entry, filled with a flat category colour and labelled
with the extension string and a truncated name — no distinguishable icon shape, no way to
resize the tiles, and no alternative to the grid (Unity's Project window offers a zoomable icon
grid and a compact list, and switches between them from the same toolbar).

## Scope

This phase covers the browsing surface only: grid view, list view, a zoom control, and a
distinct icon glyph per asset category (including image files). It does **not** load real
image content into a thumbnail — every category, images included, gets a procedurally drawn
vector glyph in this phase. Real image thumbnails (loading a texture through
`Render::IAssetLibrary::load_texture` and registering it with `ImGuiBackend::register_texture`
for `ImGui::Image`) are deliberately deferred to a Phase 2 spec, once this UI is approved and
in the tree — the project's UI-first convention (ship the ImGui surface, then wire the heavier
data path behind it) applies here because the thumbnail loader is the single largest piece of
new engineering in this feature and should not gate the rest of it.

## Persisted state

Two fields added to `Authoring::Preferences` (`engine/world/authoring/include/SushiEngine/authoring/preferences.hpp`),
alongside the other host settings it already carries (theme, camera speed, snap):

```cpp
enum class ProjectBrowserViewMode { Grid, List };

ProjectBrowserViewMode project_view_mode = ProjectBrowserViewMode::Grid;
float project_tile_size = 76.0f; // matches today's fixed constant as the default
```

`Preferences` is a plain aggregate serialized to JSON by its store; a preferences file written
before this change simply lacks these keys, and the loader's `value(key, default)` pattern
(already used for every other field) leaves them at their defaults — no migration needed.

## Type classification and icon glyphs

`project_panel.cpp`'s existing `tile_color(path, is_dir)` is replaced by an `entry_category()`
classifier returning one of: `Folder`, `Scene` (`.sushiscene`), `Prefab` (`.sushiprefab`),
`Model` (`.gltf`, `.glb`, `.fbx`, `.obj`), `Image` (`.png`, `.jpg`, `.jpeg`, `.tga`, `.bmp`,
`.hdr`, `.exr`), `Audio` (`.wav`, `.mp3`, `.ogg`, `.flac`), `Code` (the existing C++ extension
set), `Text` (the existing `has_text_extension` set, minus what `Code` now claims), `Unknown`.

Each category gets a small, recognizable flat glyph drawn directly with `ImDrawList` primitives
(lines/rects/triangles/circles) — no icon font or bitmap atlas is introduced. Folder: a tabbed
folder outline. Prefab: a hexagon (Unity's own convention for prefabs, and distinct from every
other shape in the set). Scene: a rectangle with a corner fold and a play-style triangle mark.
Model: a wireframe cube. Image: a picture-frame rectangle with a corner "sun" circle and a
mountain fold — also the *placeholder* Phase 2 will show while a real thumbnail is loading, and
the permanent glyph for image files this phase ships without one. Audio: a speaker-and-waveform
mark. Code: a page with a `</>` mark. Text/Unknown: a plain page, undecorated for Unknown.
Drawing vector shapes rather than bitmaps means every glyph stays crisp at any zoom level with
no mipmapping or blur to manage.

## Grid view

The per-entry `ImGui::Button` is replaced with an `ImGui::InvisibleButton` sized to
`preferences.project_tile_size`, with the icon glyph and the name+extension label painted onto
it directly via `ImDrawList`. All existing per-entry behaviour — selection, inline rename,
double-click-to-open, the context menu, the asset drag source, the tooltip showing the
folder-relative path — carries over unchanged; only the paint call changes. The existing
row-wrapping arithmetic (`row_x` accumulation against `avail_width`) is parameterized by the
live tile size instead of the current `TILE_SIZE` constant.

## List view

A new rendering path, used when `project_view_mode == List`: one compact row per entry
(Unity's List View proportions — roughly 20px tall), a small fixed-size icon glyph immediately
before the name, and `ImGui::Selectable` spanning the row's full width so a click anywhere on
the row selects it. Not affected by the zoom control (Unity's list rows are a fixed size).
Selection, rename, double-click, context menu, and drag-source wiring are factored into helpers
shared with the grid view rather than duplicated, so the two views cannot drift in behaviour.

## Toolbar

A pair of small toggle buttons (Grid / List) placed beside the existing search field, writing
`context.preferences.project_view_mode`.

## Zoom (Ctrl+scroll)

Active only while the grid child window is hovered, in grid view, with `io.KeyCtrl` set and
`io.MouseWheel != 0`. Each wheel step adjusts `project_tile_size` by a fixed increment (8px)
and clamps the result to **[48, 160]**. The handler zeroes `io.MouseWheel` for that frame after
applying the zoom so ImGui's own child-scroll logic (which reads the same wheel value later in
the frame) does not also scroll the grid on a zoom gesture.

## Code organization

`draw_project_panel` is currently one ~250-line function with tile rendering inlined in its
loop. This change splits out `entry_category()`, `draw_entry_icon()` (the shared glyph
painter), `draw_project_grid_view()`, and `draw_project_list_view()`, each with one
responsibility, so the panel function becomes orchestration (breadcrumb, tree pane, toolbar,
delete/drag handling) over two view functions rather than one large mixed block.

## Testing

No ImGui panel in this codebase has automated coverage (confirmed: no test exercises
`project_panel.cpp` or any other `draw_*_panel` function). Verification is manual, run by the
user through `se editor` after the change: tile size responds to Ctrl+scroll and stays within
[48, 160]; each category shows a visually distinct glyph; Grid/List toggle round-trips; the
view mode and zoom survive an editor restart; rename, delete, drag-to-prefab, double-click-open,
and the context menu all still behave exactly as before the refactor.

## Explicitly out of scope (this phase)

- Real image thumbnails (Phase 2).
- Sortable columns / Explorer-style detail view for the list (the user asked for Unity's
  icon+name list specifically, not a Details view).
- Multi-select in the grid/list (not requested; today's panel already only tracks one
  `selected_project_path`, and this phase does not change that).
