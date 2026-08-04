# UI

This file covers the retained ECS UI: the component set and anchor solver, the authoring façade
over them, and the render pass that composites the resulting draw list.

## 1. UI (retained ECS canvas)

The UI is retained and lives in the ECS, the same choice the rest of the engine makes: a `Canvas`
is an entity, and every button, panel, and label is an entity under it carrying
`engine/domain/ui/include/SushiEngine/ui/components.hpp` components — `RectTransform` (UGUI
anchor/pivot/offset layout), `UIImage`, `UIText`, `UIButton`, linked by `UIParent`. So UI appears
in the hierarchy, serializes with the scene, and (in a networked game) snapshots like any other
component.

`resolve_rect` (`engine/domain/ui/include/SushiEngine/ui/layout.hpp`) is the entire anchor model
as a pure function of a parent rectangle and a `RectTransform`, so it is unit-tested
(`Unit_UILayout`, `tests/unit/test_ui_layout.cpp`) with no world. The `UI` façade
(`engine/domain/ui/include/SushiEngine/ui/ui.hpp`) is the authoring surface:
`canvas`/`panel`/`image`/`label`/`button` builders spawn the entities into an existing `World` and
keep a light ordered index (the same host-record pattern
[the Rigid Body toggle](domain-physics.md#11-xpbd-the-rigid-body-generalization-sushiloop-m2)
uses), giving a deterministic paint and hit-test order.

Each frame `update(screen_size, pointer)` resolves every `ComputedRect` (parents before
children), runs the button state machine and press-and-release-inside click detection off an
explicit per-frame `PointerInput` (input as a value, not a global — the same determinism
discipline the sim follows), tints each button's graphic, and fires `on_click` callbacks.
`build_draw_list()` emits a renderer-agnostic `UIDrawList` of coloured rects and text runs, in
paint order. `samples/authoring/ui_demo.cpp` and `Integration_UI`
(`tests/integration/test_ui.cpp`) drive a canvas + button headlessly and assert layout, clicks,
and button states.

### 1.1. The overlay pass

`UIDrawList` lives in its own header
(`engine/domain/ui/include/SushiEngine/ui/draw_list.hpp`) rather than in `ui.hpp`, because the
two sides of that contract weigh very differently: building a list needs the world and the layout
solver, drawing one needs three structs. That split is what lets
`engine/presentation/render/include/SushiEngine/render/scene_view.hpp` name a UI draw list
without pulling the ECS in behind it.

`Render::UIView` carries it across the render seam as a non-owning POD, the same shape as
`DeformableMeshView`, plus the screen size the layout was solved against — an editor viewport
solves its UI against the viewport, not the window, so the renderer cannot infer it from the
target.

`Passes::UIPass` (`engine/presentation/render/source/passes/ui_pass.*`) runs **last of the colour
passes**, after tone mapping and FXAA, and composites into `targets.resolve` with
`AttachmentLoad::Load`. The position is the point: UI drawn before the tone map would shift hue
with the scene's exposure, and UI drawn before the AA filter would have its text softened. Drawn
here it is exactly the colour it was authored as.

Rectangles and glyphs share one vertex format, one atlas and one indexed draw.
`Material::FontAtlas` (`engine/presentation/render/source/material/font_atlas.*`) bakes printable
ASCII once at bring-up with `stb_truetype` (from the `Stb` vcpkg package the image loader already
uses, so the font path costs no new dependency) and reserves texel (0,0) as opaque white, which
is what untextured geometry samples — without it a solid panel would need a second pipeline.
`Geometry::UIBuffers` (`engine/presentation/render/source/geometry/ui_buffers.*`) tessellates the
list into per-frame-slot host-visible buffers that only grow, the arrangement
`DeformableBuffers` uses and for the same reason.

It degrades rather than fails. With no font the atlas slot falls back to the texture library's
opaque-white default, so panels still draw and only the labels go missing; with no bindless heap
the pass does not register at all.

In the editor the elements now come from the renderer, so the Game view shows what a shipped
build shows. `paint_ui_overlay` is reduced to authoring chrome the runtime never has: canvas
extents, an edit-mode outline that keeps a transparent element grabbable, and the selected
element's outline and resize handles.
