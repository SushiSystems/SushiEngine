# Material {#module-material}

`material` owns the authored surface: its texture maps and scalars, the identifiers a texture or
mesh is registered with the renderer under, and the alpha, blend, cull and wrap state that
decides how it is drawn. It is the authoring form — held by value on a mesh instance and edited
in the inspector — so nothing in it is sensitive to a graphics layout and fields may be added
without touching a shader's packing.

## Tier

`domain` — the second tier in `cmake/EngineLayers.cmake`, so a module here may depend on
`foundation` and on other `domain` modules, and on nothing above.

## Dependencies

- `core` (public) — tints, factors and tiling are the engine's own scalar and vector types.

The module is header-only and graphics-free. The importer that writes a surface, the serializer
that persists it and the inspector that edits it describe it in one vocabulary without any of
them naming a device.

## Public surface

One header, `include/SushiEngine/material/material.hpp`. It declares the `TextureId` and
`MeshId` handles and their invalid sentinels, the `SurfaceType`, `MaterialCullMode` and
`TextureWrap` enumerations with `is_alpha_blended` over the first, and the `Material` structure
itself: a metallic-roughness material in the glTF sense, extended with a detail set and the
advanced lobes, where every map is optional and an unset identifier means "use the scalar or tint
beside it". The types are declared in `SushiEngine::Render`, the namespace the render seam
shares.

## Tests

No test file of its own, and none includes `material.hpp` directly. The type is covered through
`tests/integration/test_scene_serializer_roundtrip.cpp`, which builds a fully populated
`Render::Material`, persists it and compares it back field by field, and through
`tests/unit/test_vfx_effect_serializer.cpp`, which resolves an effect's textures against it.

`is_alpha_blended` is not asserted on anywhere. Nothing about the module needs a device, so that
is a real gap rather than a consequence of the environment.

## Further reading

No design document covers this module.
[`render_pipeline_refactor.md`](../../../docs/design/render_pipeline_refactor.md) describes the
shading model the renderer packs these values into.
- [`presentation-render.md`](../../../docs/architecture/presentation-render.md) — how a material
  reaches a draw.
