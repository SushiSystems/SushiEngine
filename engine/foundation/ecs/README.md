# ECS {#module-ecs}

`ecs` owns what an entity, a component and a system are: generation-checked entity handles,
archetype-chunk storage where every component is its own structure-of-arrays column, and the
schedule that turns a system's declared reads and writes into execution nodes. It writes no
scheduler of its own — a column's allocation is the key the execution backend's hazard tracker
orders work by, so the ordering falls out of the declarations rather than out of engine code.

## Tier

`foundation` — the lowest tier in `cmake/EngineLayers.cmake`, so a module here may depend only on
other `foundation` modules.

## Dependencies

- `execution` (public) — `World`, `Archetype`, `Chunk` and `Schedule` are typed against
  `Execution::Context`, `Execution::Graph` and `Execution::Buffer`, which is what lets the same
  ECS run on either execution backend without a change here.

The module is header-only, so its kernels instantiate inside the consuming translation unit —
the safe way to ship device code.

## Public surface

Headers are relative to `include/SushiEngine/ecs/`.

| Header | Declares |
|---|---|
| `entity.hpp` | The `Entity{index, generation}` handle. |
| `component.hpp` | Component identity and the `Read<T>` / `Write<T>` access tags a system declares with. |
| `chunk.hpp` | A fixed-capacity block of structure-of-arrays columns, each column its own execution allocation. |
| `archetype.hpp` | One entity shape and the chunks that store it. |
| `world.hpp` | `World` — spawn, destroy, component access, queries, and the structure version a schedule rebuilds on. |
| `command_buffer.hpp` | `CommandBuffer`, which defers structural changes to an explicit barrier. |
| `schedule.hpp` | `Schedule::each<Access...>` — compiles systems into an execution graph once and replays it. |

## Tests

Covered by the functional suite in `tests/`, reached through the umbrella
`<SushiEngine/SushiEngine.hpp>`: `tests/unit/test_world.cpp` for entity lifetime and component
access, `tests/unit/test_command_buffer.cpp` for deferred structural change, and
`tests/integration/test_schedule.cpp` for graph compilation, replay and the read-after-write
ordering between systems. Most of the physics, loop and simulation tests drive the same storage
as a side effect.

## Further reading

No design document covers this module on its own.
[`unified_hazard_model.md`](../../../docs/design/unified_hazard_model.md) states the hazard
vocabulary the schedule's ordering is derived from.
- [`foundation.md`](../../../docs/architecture/foundation.md) — the entity-component-system and the
  system graph.
