# Astro {#module-astro}

`astro` owns where the sun, moon, planets and stars actually are: astronomical time, orbital
elements and the ephemeris over them, body orientation and gravity, and the reference frames an
observer is placed in. It answers in real units at real epochs and in double precision on the
host, so the sky a frame is lit by and a headless test asserting an altitude at a date come from
one set of numbers.

## Tier

`domain` — the second tier in `cmake/EngineLayers.cmake`, so a module here may depend on
`foundation` and on other `domain` modules, and on nothing above.

## Dependencies

- `core` (public) — positions, directions and frames are the engine's own vector and quaternion
  types.
- `environment` (public) — the ephemeris fills in the sun, moon and sky state the renderer
  shades against, so the environment's types appear in this module's published headers.

The module is header-only and reaches no device and no graphics stack.

## Public surface

Headers are relative to `include/SushiEngine/astro/`.

| Header | Declares |
|---|---|
| `julian_date.hpp` | Julian Date, the J2000 epoch, and sidereal time. |
| `orbital_elements.hpp` | Keplerian elements and the two-body position they generate. |
| `celestial_bodies.hpp` | The solar-system body catalogue: identity, physical size and orbit. |
| `ephemeris.hpp` | The assembler that turns a date and an observer into a sky full of bodies. |
| `star_catalog.hpp` | The brightest fixed stars as J2000 positions, magnitudes and colours. |
| `gravity.hpp`, `gravity_field.hpp` | The solar system's gravitational field, and the injectable interface a dynamics step takes it through. |
| `astro_dynamics.hpp` | One symplectic step of a free body's authority state. |
| `body_orientation.hpp` | How a body is turned in space over time: spin about its pole and pole direction. |
| `reference_frame.hpp` | Which body an entity's coordinates are relative to. |
| `topocentric.hpp` | The observer's local East-Up-South basis. |
| `surface_frame.hpp` | The body-fixed surface frame: where "down" is at a point on a body. |
| `scene_frame.hpp` | The bijection between a heliocentric-ecliptic position and the scene's local origin. |

## Tests

Covered by the functional suite in `tests/`. Thirteen unit files cover the pieces individually —
`test_julian_date.cpp`, `test_orbital_elements.cpp`, `test_celestial_bodies.cpp`,
`test_star_catalog.cpp`, `test_gravity.cpp`, `test_body_orientation.cpp`,
`test_reference_frame.cpp`, `test_topocentric.cpp`, `test_surface_frame.cpp`,
`test_scene_frame.cpp`, `test_body_frame.cpp`, `test_terrain_frame.cpp` and `test_season.cpp` —
and `tests/integration/test_ephemeris.cpp` and `tests/integration/test_astro_dynamics.cpp` check
the assembled sky and the integrator.

## Further reading

- [`solar_system_overhaul.md`](../../../docs/design/solar_system_overhaul.md) — the planetary
  and solar-scale regime this module supplies the frames and positions for.
- [`domain-astro.md`](../../../docs/architecture/domain-astro.md) — the ephemeris, gravity and the
  three coordinate frames.
