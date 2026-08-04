# VFX {#module-vfx}

`vfx` owns a particle effect in both of its forms: as authored — a stack of modules parameterised
by curves and gradients — and as compiled, the plain-data emitter table both simulation backends
read. It also owns the deterministic host backend that steps that table byte-reproducibly; the
cosmetic device backend consumes the same table from the renderer.

## Tier

`domain` — the second tier in `cmake/EngineLayers.cmake`, so a module here may depend on
`foundation` and on other `domain` modules, and on nothing above.

## Dependencies

- `core` (public) — particle positions, velocities and colours are the engine's own value types.

The module is header-only, so the deterministic backend's kernels instantiate inside the
consuming translation unit.

## Public surface

`vfx.hpp` is the umbrella a consumer includes. Headers are relative to
`include/SushiEngine/vfx/`.

| Header | Declares |
|---|---|
| `asset_id.hpp` | The effect handle, factored out so the effect model and the database do not depend on each other. |
| `particle_effect.hpp` | A particle effect asset: one or more emitters authored as a unit. |
| `emitter_descriptor.hpp` | One authored emitter — its module stack, capacity and simulation domain. |
| `modules.hpp` | The module taxonomy: one descriptor structure per authorable behaviour. |
| `curve.hpp`, `gradient.hpp` | A keyframed scalar curve and a colour-and-alpha gradient over normalized age, each bakeable to a lookup table. |
| `emitter_compiler.hpp` | Bakes an authored effect into a `CompiledEffect`. |
| `compiled_emitter.hpp` | The plain-data boundary between authoring and simulation: one emitter, flattened. |
| `deterministic_backend.hpp` | The host backend — a fixed-pool, byte-reproducible integrator. |
| `random.hpp` | The deterministic pseudo-random generator the particle system seeds from. |
| `effect_database.hpp` | The asset registry the ECS component's identifier resolves through. |

## Tests

Covered by the functional suite in `tests/`. `tests/unit/test_vfx_authoring.cpp` drives the
authoring model and the compiler, `tests/integration/test_particle_determinism.cpp` checks that
the host backend reproduces the same particles from the same seed, and
`tests/unit/test_vfx_effect_serializer.cpp` round-trips an effect through the `.sushieffect`
format.

The device-side cosmetic backend has no coverage here: it is compute shaders in the renderer, and
no test target links one.

## Further reading

- [`vfx_particle_system.md`](../../../docs/design/vfx_particle_system.md) — the authoring model,
  the two backends behind one seam, and the render-graph integration.
- [`domain-vfx.md`](../../../docs/architecture/domain-vfx.md) — emitters, the two backends and the
  draw path.
