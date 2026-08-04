# Core {#module-core}

`core` owns the value vocabulary the whole engine is written in: the scalar, vector, quaternion
and matrix aliases every other module spells its geometry in, the floating-origin world position
built on them, and the seeded generator a deterministic system draws its randomness from. Both
are single seams on purpose — the precision the engine simulates at, and the sequence a replay
reproduces, are each decided in one file rather than per consumer.

## Tier

`foundation` — the lowest tier in `cmake/EngineLayers.cmake`, so a module here may depend only on
other `foundation` modules.

## Dependencies

- None. The module links nothing at all, and that is deliberate: every tier above reaches it, so
  anything it depended on would become a dependency of the entire engine.

## Public surface

Headers are relative to `include/SushiEngine/core/`.

| Header | Declares |
|---|---|
| `types.hpp` | The linear-algebra integration seam: `Scalar`, `Vector3`, `Quaternion`, `Matrix4`, the parametric `Vector3T`/`QuaternionT`, `WorldVector3`/`SectorCoord` for floating-origin coordinates, and the free functions over them. |
| `random_number_generator.hpp` | `Loop::RNGState`, `seed_rng`, `next_u64` and `next_unit` — the seeded state a world carries so a replay draws the same numbers. |
| `blas_placeholder.hpp` | The `SushiEngine::placeholder` types `types.hpp` currently aliases, standing in until SushiBLAS owns them. Reached only through `types.hpp`; consumers never name it. |

`Scalar` is always `double`, with no build option to change it: the engine simulates planet- and
solar-scale worlds, where single precision quantises camera and transform math to the metre.

## Tests

Covered by the functional suite in `tests/`. `tests/unit/test_math_primitives.cpp` exercises the
vector, quaternion and transform algebra, `tests/unit/test_rng.cpp` the seeded generator, and
`tests/unit/test_floating_origin.cpp` and `tests/unit/test_floating_origin_stress.cpp` the
sector-plus-offset world position at planetary distances.

## Further reading

No design document covers this module on its own; the precision decision behind `Scalar` is
stated in `types.hpp` and in `blas_placeholder.hpp`.
- [`foundation.md`](../../../docs/architecture/foundation.md) — the value-type seam these types
  define.
