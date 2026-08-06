# Model Import {#module-model_import}

`model_import` owns the path from a glTF file on disk to a `.sushiprefab` beside it: read the
file, read its `.meta`, plan what entities it becomes, build them, and write the result. It is the
only place that knows all of those at once, which is why it sits above every module that knows
exactly one.

## Tier

`world` — the fifth tier in `cmake/EngineLayers.cmake`, so a module here may depend on every tier
below it and on other `world` modules; only `application` sits above.

This module exists because of that rule rather than in spite of it.
`Model::plan_model_instantiation` is pure — it reads no file, touches no device and creates
nothing — so it lives in `asset`, where its own tests run it without a simulation. Writing a
prefab needs `Simulation::IWorldEditor` and
the prefab shape, both `world`, and `sushiengine_add_module` refuses an `asset` module that
depends on `world` at configure time. The two halves are different kinds of thing and the tier
rule made that visible.

## Dependencies

Public, because both are spelled in the header this target publishes:

- `model` — `ModelInstantiationPlan` and `ModelImportReport` are what a caller passes and reads.
  `gltf` arrives publicly through it, which is why `GLTFSceneDescription` is nameable here.
- `simulation` — `instantiate_plan` populates an `IWorldEditor`.

`serialization` is private: only the source names `Scene::capture_prefab`.

## Public surface

| Header | Declares |
|---|---|
| `SushiEngine/model_import/prefab_output.hpp` | Instantiating a plan into a world, and writing an asset's prefab beside it. |

`instantiate_plan` creates the entities a plan describes and returns the subtree's root. It sets
each Shape's `mesh_path`, `source_node` and `primitive` and leaves its `mesh` invalid: a mesh
handle belongs to the session that imported it, and this import may run with no renderer at all.
`resolve_scene_assets` derives the handle from those three when a scene is opened.

`write_model_prefab` runs the whole path and writes `<asset>.sushiprefab` — the full path with the
extension appended, matching `.meta`'s convention, so `models/Car.gltf` and `models/Car.glb`
cannot collide. It builds the plan in a scratch simulation and calls `capture_prefab` rather than
emitting entity records itself, so the record shape keeps exactly one writer and a field added to
an entity reaches imported prefabs with no edit here.

Changing an asset's `.meta` changes the plan, which changes the prefab, which changes its
revision — and the refresh pass rebuilds every placed instance the next time a scene is opened.
Reimport is not a feature written here; it is what these steps already do.

## Tests

`tests/integration/test_model_prefab_output.cpp` covers the output headlessly: that a prefab lands
beside the asset, that the file's hierarchy survives into it as parent indices, that an unchanged
asset keeps its revision, that a changed setting changes it, and that a missing asset writes
nothing at all.

## Further reading

- `docs/design/model_import.md` — what a glTF file becomes, and why.
- `docs/design/prefab_system.md` §7 — where this connects to prefabs.
