# World

This file covers the world tier's SushiLoop layers: the per-tick snapshot buffer rollback is
built on, the loopback network reconciliation built on that, and the host-side loop core a
standalone game is authored against.

## 1. SushiLoop Snapshot: rollback (M3)

`engine/world/loop/include/SushiEngine/loop/rollback.hpp`'s `RollbackBuffer` is SushiLoop's
Snapshot layer: a fixed-capacity ring of per-tick world snapshots, keyed directly by `Chunk*`.
`capture(world, tick)` walks every archetype (`World::query` with the empty signature matches all
of them, since `signature_contains` treats an empty `need` as a subset of everything) and every
chunk, byte-copying each column's *live* rows (`count() * column_size`, not the chunk's full
capacity).

`restore(tick)` writes those bytes straight back into the same chunks and restores each chunk's
live count via `Chunk::restore_count` — a rollback-only accessor that bypasses
`allocate_row`/`remove_row`'s entity-directory bookkeeping entirely, which is safe only because
of this class's central constraint: **no entity may spawn or be destroyed, and no chunk/archetype
may be created, between a capture and its matching restore.** A `Chunk*` is stable identity only
as long as the chunk it names keeps existing and keeps holding the same entities in the same
rows; `RollbackBuffer` does not (yet) defend against a violation, which is why the constraint is
documented as a hard scope boundary rather than handled generically.

This also means capture is deliberately *not* the "only what changed" delta the design note
(`docs/design/SUSHILOOP.md`) ultimately wants — every live chunk is copied in full every tick.
Real per-write dirty tracking needs something upstream (`Schedule`, `CommandBuffer`) to mark a
chunk touched, which nothing does yet; getting the capture/restore/replay invariant right first,
on the whole-chunk case, is this milestone's scope. `RollbackBuffer` also does not decide *when*
to roll back or replay ticks forward afterward — that orchestration (a game loop, or later the
Net layer's reconciliation) is the caller's job, the same way `Chunk` does not know what a system
is.

### 1.1. SushiLoop Net: loopback reconciliation (M4)

`engine/world/loop/include/SushiEngine/loop/net.hpp` (namespace `Loop::Net`) is SushiLoop's
network layer, scoped deliberately narrow: **loopback only**. `LoopbackChannel<Command>` is an
in-process, synchronous stand-in for a client-to-server command link —
`client_send(tick, command)` records the client's own prediction into an `InputHistory<Command>`
and queues it; `server_process(corrector)` drains the queue and returns one `Ack` per tick, where
`corrector` (a caller-supplied callable) stands in for whatever a real server would compute
authoritatively. There are no sockets, no threads, no serialization, and no general P2P/lockstep
protocol here — those are out of scope for this milestone, the same way M3 scoped out per-write
dirty tracking.

Reconciliation (`net::reconcile`) is built entirely on the existing M3 machinery, not a parallel
mechanism: for every ack that disagrees with what the client predicted for that tick, it corrects
the client's `InputHistory` in place (a new `InputHistory::correct`, an overwrite rather than
`record`'s append-only insert), tracks the earliest disagreeing tick, and — if any correction
happened — calls `RollbackBuffer::restore` on that earliest tick and replays every tick from
there through the caller's current tick, re-applying the (now-corrected) history via a
caller-supplied `apply` function. Ticks the client already predicted correctly are simply
re-simulated identically; only the mispredicted ticks change on replay. This is the direct
generalization of [the snapshot invariant](#1-sushiloop-snapshot-rollback-m3) (rollback+replay
reproduces an uninterrupted run) to the case where the *input itself* changes underneath the
replay, not just the tick range.

`net::make_network_id(client_id, tick, spawn_sequence)` is the deterministic-id half M4 needs: an
entity spawned mid-simulation must get the same id on server and client without a matching
round-trip, so the id is derived from facts both sides already agree on from the numbered command
stream itself — which client is spawning, which tick, and that spawn's index among the client's
spawns that tick — packed into one 64-bit value, rather than assigned by whichever side's spawn
call happens to run first.

`Integration_NetReconciliation` (`tests/integration/test_net_reconciliation.cpp`) proves the
milestone's key invariant with a toy `Scalar` command: a client that mispredicts several ticks
and later reconciles against the server's authoritative commands converges to exactly what an
uninterrupted server-only simulation would have produced. `Unit_NetworkId`
(`tests/unit/test_network_id.cpp`) covers `make_network_id`'s collision behaviour directly. Both
are kept as narrow, isolated proofs of `net.hpp`'s mechanics.

**`samples/networking/net_demo.cpp` and `Integration_NetClientServer`**
(`tests/integration/test_net_client_server.cpp`) wire the same machinery into a live
client/server harness driven by a real gameplay command instead of the toy one. `PlayerCommand` —
two movement axes, applied to a player entity's `Position` — is the `Command` SushiLoop's real
command stream now uses; it is deliberately game-side (defined at the point of use, not in
`engine/world/loop/`), matching `input.hpp`'s stance that the command type is left to the game.

"Client" and "server" are modelled as two logical roles in one process, each owning its own
`ecs::World`, which is the honest shape of a loopback-only milestone — there is no second process
or thread to model them as. The harness proves the full chain live: per-tick prediction into
`InputHistory`, batched `LoopbackChannel::server_process` acks, `net::reconcile` rolling back and
replaying on misprediction, and convergence to an uninterrupted authoritative-only baseline world
— then, in a second phase kept strictly outside the ticks captured by `RollbackBuffer`,
`make_network_id` proves its agreement against an actual independent spawn on both the client's
and the server's `World`, with no matching round trip.

That second phase's placement is deliberate, not incidental: `RollbackBuffer` still cannot
survive a spawn or destroy inside a tick range it might roll back and replay (see
[the hard constraint](#1-sushiloop-snapshot-rollback-m3)), and this milestone does not attempt to
lift that — the demo and test sidestep it by never spawning within the reconciled window, rather
than solving rebasing across a structural change. Real transport (sockets), and rebasing
`RollbackBuffer` across a network-driven structural change, remain later work; so does wiring any
of this into the editor's Play mode, which still runs `RuntimeSimulation` directly with no
client/server split.

## 2. SushiLoop core

`docs/design/SUSHILOOP.md` is the design note; this section is the pointer from architecture to
it. `engine/world/loop/` holds the first, purely host-side layer of SushiLoop — plain C++, no
runtime or SYCL involvement — that the fixed-tick sim/net/snapshot work (M1 onward) builds on:

- **`FixedTimestepClock`**
  (`engine/world/loop/include/SushiEngine/loop/fixed_timestep.hpp`) turns real elapsed time into
  a whole number of fixed simulation steps plus a leftover interpolation fraction. It never reads
  the wall clock itself — the host accumulates real delta time into it — which is what keeps the
  *number* of ticks a run performs independent of timing jitter, a determinism precondition.
  `RuntimeSimulation` (see
  [the XPBD section](domain-physics.md#11-xpbd-the-rigid-body-generalization-sushiloop-m2)) now
  owns one of these and is its first consumer: the editor's main loop measures real frame time
  and feeds it into `ISimulation::tick(real_delta_seconds)`, keeping the one wall-clock read
  outside `engine/world/simulation/` entirely.

- **`RNGState`**
  (`engine/foundation/core/include/SushiEngine/core/random_number_generator.hpp`) is a trivially
  copyable xorshift128+ generator, storable as an ECS component so seeded randomness travels with
  the world through snapshots and rollback instead of living in a hidden global.

- **`InputHistory<Command>`** (`engine/world/loop/include/SushiEngine/loop/input.hpp`) is the
  per-tick, numbered command buffer shape that networked input capture and rollback replay
  (M3/M4) will read and write; the command type itself is left to the game.

- **`Loop::App<Command>`** (`engine/world/loop/include/SushiEngine/loop/app.hpp`) is the
  authoring API that ties the above together into the settled surface a game is written against.
  It is the composition root: it owns the `SushiRuntime`, builds the `Execution::Context` over it
  (handed out by `execution()` for a subsystem that needs its own graph or columns), and owns the
  `World` and `Schedule`.

  It drives one fixed-step deterministic loop: each `step_once()` captures the tick's command
  into `InputHistory` (and a `RollbackBuffer` snapshot when enabled), applies it via the game's
  `on_command`, runs the `Schedule`, and applies the `CommandBuffer` barrier. Systems are
  declared ergonomically with `app.system<Read<A>, Write<B>>("name").each(fn)`, a thin wrapper
  over `Schedule::each` (via `SystemBuilder`).

  The loop is **always multiplayer-ready**: the command stream is numbered every tick regardless
  of network state, and the network is reached only through
  `Loop::Net::INetworkTransport<Command>` (see
  [the net layer](#11-sushiloop-net-loopback-reconciliation-m4)), so `connect()`-ing a transport
  turns a single-player game networked with no change to its systems. This is the point at which
  the SushiLoop core layers are wired into `Schedule`/`World` — the standalone-game host,
  distinct from `engine/world/simulation/`'s editor-facing `ISimulation` (see
  [the XPBD section](domain-physics.md#11-xpbd-the-rigid-body-generalization-sushiloop-m2)).

`SUSHIENGINE_DETERMINISTIC_FP` (`cmake/ProjectOptions.cmake`, default `ON`) disables fast-math and
FP contraction on the `SushiEngine` INTERFACE target, closing off two ways a build could make the
same floating-point expression evaluate differently between runs.
