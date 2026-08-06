# Authoring {#module-authoring}

`authoring` owns the services an editor is built out of, with no editor in them: the undo stack
that snapshots a whole world, the persisted preferences and panel state, the autosave decision,
the cook and bake model behind the Bake window, and the heat scale a soft-body overlay colours
by. Each is a policy stated in one testable place rather than a behaviour buried in a panel.

## Tier

`world` — the fifth tier in `cmake/EngineLayers.cmake`, so a module here may depend on every
tier below it and on other `world` modules; only `application` sits above. It lives here rather
than under an application directory because the test suite drives all of it with no editor in
sight, and nothing outside an application directory may depend on one.

## Dependencies

Engine modules, all public because each is spelled in a header this target publishes:

- `core` — the value types the persisted settings are held in.
- `environment` — `preferences.hpp` holds the environment a new scene starts from.
- `geometry` — `cook_bake_state.hpp` starts a cook at a mesh.
- `physics` — `soft_body_heat.hpp` scales a stress reading against a physical material.
- `serialization` — the undo stack captures and applies a scene in the serializer's format.
- `simulation` — the undo stack snapshots a world through `IWorldEditor`.

And one private engine module:

- `model` — `cook_bake_state.cpp` reads an asset's `<asset>.meta` sidecar to find what that
  asset says differs from the project's cooking defaults, and folds it in before submitting a
  cook. Private because no header here names a `ModelImportSettings`.

External: `nlohmann_json::nlohmann_json` publicly, because `command_history.hpp` holds a snapshot
as a JSON document; `sushiengine_physics_cooking` publicly, because `cook_bake_state.hpp` is
written against the cooking service and the three cooked asset kinds it produces. The renderer's
include root is added publicly for `RenderSettings`, which `preferences.hpp` persists — the
include root rather than the module, because a configure with `SUSHIENGINE_BUILD_RENDER` off has
no library to link.

## Public surface

Headers are relative to `include/SushiEngine/authoring/`.

| Header | Declares |
|---|---|
| `command_history.hpp` | Undo and redo over whole-world snapshots, in the scene serializer's format, with a discrete-action mode and a bracketed mode for continuous drags. |
| `preferences.hpp` | The persisted host settings: theme, the environment a new scene starts from, the render and simulation quality budgets. |
| `panel_visibility.hpp` | Which editor windows are shown — one flag per window, persisted with the preferences. |
| `game_view_settings.hpp` | The Game view's aspect presets and orientation. |
| `gizmo_state.hpp` | The gizmo's mode and axis-frame vocabulary, free of the controller. |
| `autosave.hpp` | `AutosaveTimer` — the autosave decision as a tickable clock that runs only while a save would be meaningful. |
| `cook_bake_state.hpp` | What the Bake panel knows, with no interface toolkit near it. Its stored document holds the project's cooking default alone; what one asset says differs lives in that asset's `.meta` sidecar, and `CookingOverrideMigration` reports what one read moved there. |
| `soft_body_heat.hpp` | What a soft-body debug view means, with no drawing in it. |

`command_history.cpp`, `preferences.cpp` and `cook_bake_state.cpp` are the module's sources; the
rest are header-only.

## Tests

Covered by the functional suite in `tests/`, which links `sushiengine_authoring` directly rather
than compiling its translation units in by hand. `tests/unit/test_autosave.cpp` drives the timer
policy, `test_preferences_roundtrip.cpp` the persisted settings, `test_cook_bake_state.cpp` the
bake model, and `test_soft_body_heat.cpp` the heat scale. The undo stack is driven through
`tests/integration/test_scene_serializer_roundtrip.cpp`, `test_physics_authoring.cpp`,
`test_physics_joint_component.cpp` and `test_vehicle_component.cpp`.

`panel_visibility.hpp`, `game_view_settings.hpp` and `gizmo_state.hpp` are covered only where
the preferences round-trip carries them; no case drives them on their own.

## Further reading

- [`editor_ux_overhaul.md`](../../../docs/design/editor_ux_overhaul.md) — the editor these
  services back, phase by phase.
- [`editor_feature_sync_gaps.md`](../../../docs/design/editor_feature_sync_gaps.md) — the audit
  of editor surface against engine features.
