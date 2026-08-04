# Environment {#module-environment}

`environment` owns the scene's outdoor state as plain data: the sun, the planet it lights and the
sky layers drawn around it, the punctual lights under that, and the atmospheric fields a frame is
shaded against — the regional nest's parameters and readback, the synoptic placement, and the
spatial weather grid. Four consumers author, draw, edit and persist these values, and this module
is the one definition all four agree on.

## Tier

`domain` — the second tier in `cmake/EngineLayers.cmake`, so a module here may depend on
`foundation` and on other `domain` modules, and on nothing above.

## Dependencies

- `core` (public) — directions, positions and colours are the engine's own value types.
- `material` (public) — the sky and ground surfaces carry the same texture identifiers and
  surface state an authored material does.

The module is header-only and device-free on purpose. The simulation authors these values, the
renderer consumes them, and the editor and the serializer both read them; a shared graphics stack
between the four would put a device in the path of every one of them.

## Public surface

Headers are relative to `include/SushiEngine/environment/`. The types are declared in
`SushiEngine::Render`, the namespace the render seam shares.

| Header | Declares |
|---|---|
| `environment.hpp` | The scene-level lighting and sky environment the renderer shades against. |
| `light.hpp` | The punctual scene lights the clustered light engine shades against. |
| `atmosphere_nest.hpp` | The regional nest's parameters, vertical grid and base state, as data. |
| `weather_field.hpp` | The spatial weather field the renderer reads instead of one global column. |
| `synoptic_field.hpp` | The render seam's view of where the planet's weather is. |

## Tests

Thin, and worth naming plainly. `tests/integration/test_ephemeris.cpp` is the only case that
includes an environment header directly, checking the sky state the ephemeris fills in. The rest
of the module is exercised indirectly — the scene and environment serializers round-trip it in
`tests/integration/test_scene_serializer_roundtrip.cpp`, and the weather bridges in
`tests/unit/test_weather_field.cpp` and `tests/unit/test_weather_world_coupling.cpp` write into
these types — but no case drives `Environment` or `Light` on its own.

## Further reading

- [`atmosphere_system.md`](../../../docs/design/atmosphere_system.md) — the weather and nest
  fields this module publishes to the renderer.
- [`solar_system_overhaul.md`](../../../docs/design/solar_system_overhaul.md) — the planetary
  sky and sun state the environment carries.
- [`domain-atmosphere.md`](../../../docs/architecture/domain-atmosphere.md) — what writes these
  fields and what reads them.
