# glTF {#module-gltf}

`gltf` owns turning a glTF file into engine data: its triangles as a `Geometry::TriangleMesh`,
and its skin and animations as the relocatable blobs the animator plays. It also owns cgltf's one
implementation unit, so every consumer takes the parser's symbols from here rather than compiling
them again.

## Tier

`asset` — the third tier in `cmake/EngineLayers.cmake`, so a module here may depend on
`foundation`, `domain` and other `asset` modules, and on nothing above.

## Dependencies

- `geometry` (public) — `mesh_import.hpp` hands back a `TriangleMesh`, so the type is part of
  what this module publishes.
- `core` (private) — only the sources name a scalar.
- `animation` (private) — only the sources name a skeleton, clip or morph blob.
- `cgltf` (private include directory) — located by its header, the documented vcpkg approach for
  a port with no CMake export. No header here names a cgltf type, so nothing that reads an
  imported mesh has to locate the parser to compile against it.

It links no device and no graphics stack, deliberately: the cooking pipeline starts at a file
rather than at a mesh somebody already had, and reaching one through the renderer's importer
would mean bringing up a graphics stack to cook a collider.

## Public surface

Headers are relative to `include/SushiEngine/gltf/`.

| Header | Declares |
|---|---|
| `mesh_import.hpp` | A glTF file's triangles, with no renderer between the file and them. |
| `skeleton_import.hpp` | `import_gltf_skeleton`, which cooks a skin into a `.sushiskel` blob, and `import_gltf_animated`, which imports a skin and all its animations against one joint order, returning the `.sushianim` blobs and the morph target names. |

The module's sources are `cgltf_implementation.cpp`, `mesh_importer.cpp`,
`skeleton_importer.cpp` and `animation_importer.cpp`; the animation importer is reached through
`skeleton_import.hpp` rather than through a header of its own.

## Tests

Thin. `tests/unit/test_animation_morph_import.cpp` is the only case that includes a header from
this module, importing a rigged glTF and checking the morph target names and the cooked blobs.
The test binary links `sushiengine_gltf` directly, and a real asset path is supplied through
`SE_TEST_ASSET_DIR`.

`mesh_import.hpp` has no coverage at all. Its only consumer in the tree is the editor shell, and
no test includes it. Importing a mesh needs nothing but a file, so that is a genuine gap rather
than an environment limitation.

## Further reading

- [`physics_system.md`](../../../docs/design/physics_system.md) — §3.4 explains why the importer
  sits below the renderer instead of inside it.
