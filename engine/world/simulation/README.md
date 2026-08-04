# Simulation {#module-simulation}

`simulation` owns the live world behind the `ISimulation` seam: it holds a SushiRuntime, an ECS
`World` and a `Schedule`, defines the engine's built-in component set, and hands all of it out
only as plain C++. It is also where the physics, weather, audio and animation domains are wired
into one stepping world and extracted back out for a host to draw.

## Tier

`world` — the fifth tier in `cmake/EngineLayers.cmake`, so a module here may depend on every tier
below it and on other `world` modules; only `application` sits above.

## Dependencies

Engine modules, public because each is spelled in a header this target publishes: `animation`,
`astro`, `atmosphere`, `audio`, `core`, `ecs`, `environment`, `execution`, `physics`, `ui` and
`vfx`. `components.hpp` carries the ECS and core value types, `physics_services.hpp` the bodies
and joints, `weather_types.hpp` the atmospheric column, `audio_extract.hpp` the acoustic scene,
and `simulation.hpp` the animation, astronomy, environment and particle vocabulary a frame is
described in. `ui` is there because `simulation.hpp` hands out a `Render::SceneView`, whose
header carries an interface draw list.

Private: `gltf` and `loop` — `runtime_simulation.cpp` imports crowd skeletons and clips from
glTF and steps on the loop's clock, and no header here names either.

External: `sushiengine_physics_cooking` publicly, because `physics_simulation.hpp` builds a soft
body or a vehicle straight from a `.sushisoft` or `.sushinodebeam` blob and the loaders for both
are compiled there. `SushiEngine` — the umbrella interface target — is private:
`runtime_simulation.cpp` includes the umbrella header to assemble a whole world, but nothing
published names it. The renderer's include root is added publicly for `Render::SceneView`; the
include root rather than the module, because nothing here calls into the renderer and a
configure with `SUSHIENGINE_BUILD_RENDER` off has no library to link.

`source/runtime_simulation.cpp` is the one device translation unit in the engine outside an
example, which is why the SYCL policy is applied to this target and why `-fsycl` propagates
publicly — a static library's device images are finalized at the consuming executable's link.
The target is built position-independent because the shells that link it are.

## Public surface

`simulation.hpp` is the seam a host includes. Headers are relative to
`include/SushiEngine/simulation/`.

| Group | Headers | Declares |
|---|---|---|
| The seam | `simulation.hpp`, `components.hpp`, `simulation_settings.hpp` | `ISimulation` and `IWorldEditor`, the built-in ECS component set, and the host's simulation quality budgets. |
| Physics | `physics_simulation.hpp`, `physics_services.hpp`, `physics_bridge.hpp`, `physics_extract.hpp`, `physics_assembly.hpp`, `collider.hpp`, `joint_params.hpp`, `ragdoll.hpp` | The live physics scene behind one solver, the entity-to-body bridge, the extract that turns authored records into descriptors, and the assembly asset. |
| Weather | `weather_types.hpp`, `weather_provider.hpp`, `seeded_weather.hpp`, `ingested_weather.hpp`, `metar_parser.hpp`, `weather_wind.hpp`, `weather_flight_hazards.hpp`, `weather_field_buffer.hpp`, `weather_world_coupling.hpp`, `weather_cloudscape_compiler.hpp` | The `IWeatherProvider` seam and its seeded and real-data implementations, the METAR parser, the wind and hazard queries, and the bridges that turn a provider's column state into what the renderer reads. |
| Atmosphere and season | `atmosphere_forcing_buffer.hpp`, `climatology_asset.hpp`, `season.hpp` | The storage behind the regional nest's forcing, the baked climatology reader, and the epoch-to-year-position mapping. |
| Audio | `audio_extract.hpp` | The wall-clock audio snapshot extract — the ECS half of the audio bridge. |

## Tests

Covered by the functional suite in `tests/`, which links `sushiengine_simulation` directly; more
than twenty files include a header from this module.

- Weather: `tests/unit/test_weather_field.cpp`, `test_weather_world_coupling.cpp`,
  `test_weather_flight_hazards.cpp`, `test_metar_parser.cpp`, `test_ingested_weather.cpp`,
  `test_atmosphere_nest.cpp`, `test_atmosphere_quality.cpp`, `test_climatology.cpp`,
  `test_synoptic_field.cpp`, `test_season.cpp`.
- Physics: `tests/unit/test_physics_assembly.cpp`, `test_physics_extract.cpp`, and the
  integration cases `test_physics_simulation.cpp`, `test_physics_bridge.cpp`,
  `test_physics_authoring.cpp`, `test_physics_joint_component.cpp`, `test_joint_assembly.cpp`,
  `test_joint_parking.cpp`, `test_soft_body_service.cpp`, `test_vehicle_component.cpp`.
- The seam itself: `tests/integration/test_scene_serializer_roundtrip.cpp` drives a real
  simulation through `IWorldEditor`, and `tests/integration/test_audio_ecs.cpp` the audio
  extract.
- `tests/regression/test_penetration_contract.cpp` holds the contract between the visible mesh
  and the simulated one across the extract.

## Further reading

- [`physics_system.md`](../../../docs/design/physics_system.md) — the physics assembly, the
  penetration contract, and the seams this module wires.
- [`atmosphere_system.md`](../../../docs/design/atmosphere_system.md) — the weather tiers and the
  provider seam.
- [`SUSHILOOP.md`](../../../docs/design/SUSHILOOP.md) — the deterministic tick the world is
  stepped on.
- [`world.md`](../../../docs/architecture/world.md) — what the loop steps and what it hands the
  renderer.
