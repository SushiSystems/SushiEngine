# Geometry {#module-geometry}

`geometry` owns engine-neutral triangle geometry: the mesh value types, the topology analysis and
repair a cooker runs before it can cook, the closest-point hierarchy every distance query goes
through, the signed-distance bake over it, and the meshlet split the mesh-shader draw path takes.
Both the renderer and the physics read a mesh's distance field, and neither may own it.

## Tier

`domain` — the second tier in `cmake/EngineLayers.cmake`, so a module here may depend on
`foundation` and on other `domain` modules, and on nothing above.

## Dependencies

- None. The module links nothing — no Vulkan, no SYCL, no runtime — deliberately: a cooker that
  needed a device would fail on a build machine, and a mesh that belonged to one of its two
  consumers could not be read by the other.

## Public surface

Headers are relative to `include/SushiEngine/geometry/`.

| Header | Declares |
|---|---|
| `triangle_mesh.hpp` | `TriangleMesh` — positions and indices, and nothing else. |
| `mesh_vertex.hpp` | The engine's drawable vertex format. |
| `mesh_utilities.hpp` | The topology analysis and repair a cooker needs before it can cook a mesh. |
| `mesh_distance_query.hpp` | The host closest-point hierarchy, built for hundreds of millions of queries. |
| `signed_distance_field.hpp` | Baking a triangle mesh into a cube of signed distances. |
| `meshlet.hpp` | Splitting a mesh into meshlets for the mesh-shader draw path. |

Four of these compile: `mesh_utilities.cpp`, `meshlet.cpp`, `mesh_distance_query.cpp` and
`signed_distance_field.cpp` are the module's sources.

## Tests

Covered by the functional suite in `tests/`, which links `sushiengine_geometry` directly.
`tests/unit/test_geometry_mesh_utilities.cpp`, `test_geometry_mesh_distance.cpp` and
`test_geometry_sdf.cpp` drive the three compiled paths, and the cooking, mass-property, import
and penetration tests read a `TriangleMesh` throughout —
`tests/unit/test_collision_cooker.cpp`, `test_cooking_core.cpp`, `test_soft_body_cooker.cpp`,
`test_mesh_mass_properties.cpp`, `test_import_chain.cpp` and
`tests/regression/test_penetration_contract.cpp` among them.

## Further reading

- [`physics_system.md`](../../../docs/design/physics_system.md) — §3.4 states why this module
  links nothing and why the cooking pipeline starts here.
- [`domain-physics.md`](../../../docs/architecture/domain-physics.md) — how the solver consumes
  these shapes.
