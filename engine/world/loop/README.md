# Loop {#module-loop}

`loop` owns the orchestration surface a session is driven through: `Loop::App`, the
fixed-timestep clock under it, the per-tick command capture, the rollback ring, and the
reconciliation a network path replays against. It schedules the model rather than being part of
it, which is exactly why it sits above the domain — a domain module reaching up here for a clock
or a seed becomes an edge the configure refuses.

## Tier

`world` — the fifth tier in `cmake/EngineLayers.cmake`, so a module here may depend on every tier
below it and on other `world` modules; only `application` sits above.

## Dependencies

- `core` (public) — the seeded generator state a deterministic tick draws from.
- `ecs` (public) — `App` owns a `World`, a `Schedule` and a `CommandBuffer`, and hands all three
  out.
- `execution` (public) — the schedule it drives is typed against the execution seam.

The module is header-only, so its systems instantiate inside the consuming translation unit.

## Public surface

`app.hpp` is the authoring surface a game is written against; the other four are the layers
underneath it. Headers are relative to `include/SushiEngine/loop/`.

| Header | Declares |
|---|---|
| `app.hpp` | `Loop::App<Command>` — one fixed tick: capture snapshot, sample command, apply, run the schedule, apply deferred structural changes, reconcile. `App::system<Access...>(name).each(fn)` is sugar over `Schedule::each`. |
| `fixed_timestep.hpp` | The fixed-step accumulator that decides how many ticks a frame owes. |
| `input.hpp` | Per-tick command capture: a numbered command buffer. |
| `rollback.hpp` | Per-chunk snapshot capture and restore. |
| `net.hpp` | The network layer: a transport seam and the find-earliest-mismatch, restore, replay reconciliation over it. |

Two limits are worth stating up front rather than discovering: rollback does not survive a spawn
or destroy inside the rolled-back range, and the network layer is loopback-only scaffolding —
there are no sockets and no wire serialization.

## Tests

Covered by the functional suite in `tests/`, mostly through the umbrella header.
`tests/unit/test_fixed_timestep.cpp` drives the accumulator and `test_network_id.cpp` the
identity scheme; `tests/integration/test_app.cpp` drives the tick end to end,
`test_rollback.cpp` the snapshot ring, `test_deterministic_replay.cpp` and
`test_input_determinism.cpp` the replay contract, and `test_net_client_server.cpp` and
`test_net_reconciliation.cpp` the loopback transport and the reconciliation algorithm.

## Further reading

- [`SUSHILOOP.md`](../../../docs/design/SUSHILOOP.md) — the design of the deterministic,
  network-ready loop and the milestones it was built in.
- [`world.md`](../../../docs/architecture/world.md) — snapshots, rollback and reconciliation.
