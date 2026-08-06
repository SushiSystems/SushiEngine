# Model {#module-model}

`model` owns what a model asset says about how it is imported, and the decision that turns a
glTF node graph into a list of entities to create. The settings live in a `<asset>.meta` sidecar
beside the asset, so moving, renaming or copying the asset carries them along instead of
orphaning them.

## Tier

`asset` — the third tier in `cmake/EngineLayers.cmake`, so a module here may depend on
`foundation`, `domain` and other `asset` modules, and on nothing above.

## Dependencies

- `core` (public) — `import_settings.hpp` spells a rotation as a `Vector3f`.
- `physics` (public) — `import_settings.hpp` embeds a `Physics::Cooking::ImportProfileOverride`,
  so what an asset says differs from the project's cooking defaults is part of its settings
  rather than a second document keyed by the same path.
- `gltf` (public) — the node graph the instantiation planner reads is a `gltf` type. The
  dependency costs nothing at the parser level: cgltf is a private include directory of that
  module and no header there names a cgltf type, so this one takes the description without
  taking the parser.
- `nlohmann_json` (private) — the sidecar's encoding. No header here names a JSON document, so
  nothing that reads an asset's settings has to link the library to compile against them.

It links no device and no editor, deliberately: every hard decision in the import path is here,
so all of it is testable on a machine with no GPU and no window.

## Public surface

Headers are relative to `include/SushiEngine/model/`.

| Header | Declares |
|---|---|
| `import_settings.hpp` | `ModelImportSettings` — the scale, root rotation, per-kind toggles and cooking override one asset carries — and its equality. |
| `import_settings_io.hpp` | `model_import_settings_path`, `load_model_import_settings` and `save_model_import_settings`, plus `migrate_cooking_overrides_to_sidecars` — the one-way move off a project document's path-keyed `overrides` object. |
| `instantiation_plan.hpp` | `plan_model_instantiation` — the pure decision that turns a node graph plus its settings into a `ModelInstantiationPlan` of `PlannedEntity` rows and a `ModelImportReport`. |

The module's sources are `import_settings_io.cpp` and `instantiation_plan.cpp`.

## Tests

`tests/unit/test_model_import_settings_io.cpp` covers both headers: the sidecar path, the
round trip of every field, that an unset cooking override stays unset rather than becoming a
value, that a malformed sidecar is reported rather than read as "no settings", and that the
migration writes a sidecar without disturbing what it already held, drops an override whose
asset is gone, and finds nothing to do on a second run.

`tests/unit/test_cook_bake_state.cpp` covers the consumer end: an asset's sidecar decides which
cookers run for it.

`tests/unit/test_model_instantiation_plan.cpp` covers `instantiation_plan.hpp` rule by rule
against descriptions built in the test: what a node with one primitive becomes and what one
with several splits into, that a parent always precedes its child, the synthetic root a
multi-root file gains, sibling name disambiguation and the cousins it leaves alone, the
directional light and the skinned mesh that are counted rather than created, where the scale
factor and the root rotation land, and that dropping a rotated, scaled pivot composes its
transform into its child instead of adding it.

## Further reading

- [`model_import.md`](../../../docs/design/model_import.md) — §3 states the three-way split this
  module sits in the middle of, and §4.2 the sidecar and its migration.
