# Atmosphere {#module-atmosphere}

`atmosphere` owns T1, the global dynamical core — two-layer moist quasi-geostrophic flow on a
latitude/longitude grid, from which cyclones, fronts, the jet and storm tracks emerge rather than
being placed — together with the mean climatology it is a departure from and the Fourier
transform its elliptic inversion is built on. The regional nest (T2) is a Vulkan compute service
and lives beside the renderer; the seam between the two tiers is plain data.

## Tier

`domain` — the second tier in `cmake/EngineLayers.cmake`, so a module here may depend on
`foundation` and on other `domain` modules, and on nothing above.

## Dependencies

- `core` (public) — the grid fields and query results are the engine's own scalar and vector
  types.

It reaches no device and no graphics stack, deliberately: a gameplay query, the regional nest's
parent forcing and a headless probe all ask it what the weather is a thousand kilometres away,
and none of the three should have to bring a graphics stack up to find out. The target is built
position-independent because `sushiengine_simulation`, its one library consumer, is.

## Public surface

Headers are relative to `include/SushiEngine/atmosphere/`.

| Header | Declares |
|---|---|
| `geographic_position.hpp` | Where a query is, shared by every tier of the module. |
| `climatology.hpp` | T0 — the baked mean state T1 is a departure from. |
| `quasigeostrophic_core.hpp` | T1 — the global dynamical core and its integration step. |
| `synoptic_field.hpp` | Where the weather is at planet scale, as a closed-form function. |
| `fourier_transform.hpp` | A power-of-two discrete Fourier transform and the pairing that halves it. |

Three of these compile: `climatology.cpp`, `fourier_transform.cpp` and
`quasigeostrophic_core.cpp` are the module's sources.

## Tests

Covered by the functional suite in `tests/`. `tests/unit/test_quasigeostrophic_core.cpp` drives
the dynamical core, `tests/unit/test_climatology.cpp` the mean state,
`tests/unit/test_synoptic_field.cpp` the closed-form placement, and
`tests/unit/test_season.cpp` the epoch-to-year-position mapping the climatology is indexed by.
The test binary links `sushiengine_atmosphere` directly, so the compiled units are exercised as
built rather than recompiled.

## Further reading

- [`atmosphere_system.md`](../../../docs/design/atmosphere_system.md) — the meteorology, the T0
  through T2 tier split, and why T1 stays off the device.
- [`domain-atmosphere.md`](../../../docs/architecture/domain-atmosphere.md) — the global dynamical
  core and the regional nest.
