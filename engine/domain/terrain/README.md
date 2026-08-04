# Terrain {#module-terrain}

`terrain` owns a planet's surface as data: the cube-to-ellipsoid mapping that says where a tile
is, the quadtree that decides which patches are drawn and at what resolution, the tile addressing
and residency a streamer works in, and the height sources and layer stack a tile is baked from.
The authoritative definition of a body's ground lives here, so the offline baker, the test suite
and the renderer read one definition rather than three.

## Tier

`domain` — the second tier in `cmake/EngineLayers.cmake`, so a module here may depend on
`foundation` and on other `domain` modules, and on nothing above.

## Dependencies

- `core` (public) — tile corners, heights and body-fixed positions are the engine's own value
  types, in the double precision a planet needs.

The module is header-only and graphics-free on purpose.

## Public surface

Headers are relative to `include/SushiEngine/terrain/`.

| Header | Declares |
|---|---|
| `cube_sphere.hpp` | The cube-to-ellipsoid map, and the difference form that keeps precision near the surface. |
| `tile_address.hpp` | Which patch of a body a tile is, and how its samples are laid out. |
| `quadtree.hpp` | Which patches are drawn this frame, and at what resolution. |
| `tile_residency.hpp` | Which cache slot a node reads its heights from, and where inside it. |
| `height_source.hpp` | Where a tile's measured elevation comes from. |
| `height_function.hpp` | The authoritative definition of a body's ground. |
| `layer_stack.hpp` | The ordered stack of authored records that reshapes measured ground. |
| `terrain_authoring.hpp` | One body's ground as something an author can change and watch. |
| `pack_format.hpp` | The baked terrain asset: its byte layout, its reader, and its refusals. |

## Tests

Covered by the functional suite in `tests/`: `tests/unit/test_cube_sphere.cpp` for the mapping,
`test_terrain_quadtree.cpp` for selection, `test_tile_residency.cpp` for the cache addressing,
`test_terrain_layers.cpp` for the authored layer stack, and `test_planet_pack.cpp` for the baked
asset's layout and its refusals. `tests/unit/test_terrain_frame.cpp` covers the frame maths that
lives in the renderer's source tree, reached by include path so the suite does not have to link
the renderer.

## Further reading

- [`solar_system_overhaul.md`](../../../docs/design/solar_system_overhaul.md) — where the
  elevation comes from, how it reaches the device, and how a body is drawn from a metre to
  orbit.
- [`domain-terrain.md`](../../../docs/architecture/domain-terrain.md) — planetary terrain, from a
  metre to orbit.
