# UI {#module-ui}

`ui` owns the retained user interface as ECS components — canvas, anchored rectangles, images,
text and buttons — together with the anchor solver, the pointer and click model, and the
immediate draw list they flatten into. A panel is entities the schedule steps, so there is no
second scene graph, and the renderer only ever sees vertices.

## Tier

`domain` — the second tier in `cmake/EngineLayers.cmake`, so a module here may depend on
`foundation` and on other `domain` modules, and on nothing above.

## Dependencies

- `core` (public) — the screen-space vector, rectangle and colour types build on the engine's
  own value types.
- `ecs` (public) — the components are real ECS components and the façade spawns real entities,
  so `World` appears in this module's published headers.

The module is header-only.

## Public surface

`ui.hpp` is the façade a consumer includes. Headers are relative to `include/SushiEngine/ui/`.

| Header | Declares |
|---|---|
| `rect.hpp` | The two-dimensional value types the layer works in: a screen-space vector, rectangle and colour. |
| `components.hpp` | The component set: `RectTransform`, `Canvas`, `UIImage`, `UIText`, `UIButton`. |
| `layout.hpp` | `resolve_rect` — one `RectTransform` resolved against a parent rectangle. |
| `interaction.hpp` | The pointer input model and the click event it emits. |
| `draw_list.hpp` | The contract between whatever builds an interface and whatever draws it. |
| `ui.hpp` | The `UI` façade: `canvas()`, `panel()`, `image()`, `label()`, `button()`, and the layout and drive passes over them. |

## Tests

Covered by the functional suite in `tests/`. `tests/unit/test_ui_layout.cpp` drives the anchor
solver, and `tests/integration/test_ui.cpp` builds a canvas of entities, lays it out and drives
the pointer and click model through the schedule.
`tests/unit/test_input_virtual_controls.cpp` exercises the pointer seam from the input side.

Nothing asserts on the flattened draw list's vertex output, because the consumer that reads it is
the renderer and no test target links one.

## Further reading

No design document covers this module.
[`editor_ux_overhaul.md`](../../../docs/design/editor_ux_overhaul.md) describes the editor
interface, which is a separate immediate-mode stack and not this one.
- [`domain-ui.md`](../../../docs/architecture/domain-ui.md) — the retained canvas and its overlay
  pass.
