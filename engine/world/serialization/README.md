# Serialization {#module-serialization}

`serialization` owns the one JSON shape for a `.sushiscene` scene file, a `.sushieffect` particle
asset, and the environment both of them embed — plus the byte encoding and content hash the
scene shape names a cooked blob by. It is interface-free by construction: every function here is
driven through `Simulation::IWorldEditor`'s query surface and `nlohmann::json`, so the module
names no editor type and anything that has to read or write a scene can link it.

## Tier

`world` — the fifth tier in `cmake/EngineLayers.cmake`, so a module here may depend on every tier
below it and on other `world` modules; only `application` sits above.

## Dependencies

Engine modules, public because each is spelled in a header this target publishes:

- `simulation` — the scene shape is written against `IWorldEditor`.
- `astro` — the scene carries the sky's epoch as a Julian date.
- `environment` — the environment shape is written against the environment itself.
- `vfx` — the effect shape is written against a `ParticleEffect`.

`material` is private: only `effect_serializer.cpp` names a material, resolving an effect's
textures against the asset library it is handed. `nlohmann_json::nlohmann_json` is public — it is
the document type every entry point takes. The renderer's include root is added privately for
`IAssetLibrary`, which both serializers resolve a texture or mesh name through; the include root
rather than the module, because a configure with `SUSHIENGINE_BUILD_RENDER` off has no library
to link.

The module also adds `include/SushiEngine/serialization` itself as a public include directory,
because its headers are spelled by bare file name.

## Public surface

Headers are relative to `include/SushiEngine/serialization/`.

| Header | Declares |
|---|---|
| `scene_serializer.hpp` | Reading and writing a `.sushiscene` file through `IWorldEditor`. |
| `scene_blob_table.hpp` | The table that names each cooked blob a scene references by content hash. |
| `environment_serializer.hpp` | The one JSON shape for an authored environment. |
| `effect_serializer.hpp` | Reading and writing `.sushieffect` files. |

`source/byte_encoding.cpp` holds the base64 codec and the content hash. It is module-private —
spelled in no published header — because JSON is the only reason either exists and there is one
consumer.

## Tests

Covered by the functional suite in `tests/`, which links `sushiengine_serialization` directly.
`tests/integration/test_scene_serializer_roundtrip.cpp` round-trips a whole scene, including the
blob table and the embedded environment, and `tests/unit/test_vfx_effect_serializer.cpp`
round-trips an effect. `tests/unit/test_byte_encoding.cpp` covers the private codec by naming it
through its source path, which the test target adds as an include directory.

`environment_serializer.hpp` has no case of its own; it is covered where the scene round-trip
carries an environment.

## Further reading

- [`vfx_particle_system.md`](../../../docs/design/vfx_particle_system.md) — §8 specifies the
  `.sushieffect` shape.
