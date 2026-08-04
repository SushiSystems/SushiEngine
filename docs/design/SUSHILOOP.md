# SushiLoop — Design

SushiLoop is the deterministic, network-ready game loop for SushiEngine. It is the
foundation for building actual games on the engine, and eventually for "sushiverse",
a physics-driven virtual universe. This document records the design decisions and the
plan to build it. It is a living design note, not a spec.

## What SushiLoop is (and is not)

SushiLoop is a fixed-step deterministic simulation loop plus an ergonomic way to
register gameplay systems. Users write their game logic in C++.

It is **not** a per-instance object model like Unity's MonoBehaviour, where every
object has its own `Update()` method that runs one at a time on the CPU. That model
fights everything the engine is good at: data-parallel execution on SushiRuntime,
determinism, and scale. Instead, gameplay is written as **ECS systems** (the DOTS
style): a system declares which components it reads and writes, and the engine runs it
in parallel across all matching entities. SushiLoop gives that a familiar, tidy
authoring surface and a well-defined frame lifecycle, but the execution stays
data-oriented underneath.

## The layers

The dependency direction is strictly one way, top to bottom. Nothing lower ever
depends on anything higher.

```
Game code (C++ ECS systems)          — the user writes this; one SYCL translation unit
        |
SushiLoop core
  - Time     : fixed outer tick, deterministic physics sub-stepping, render interpolation
  - Sim      : the ECS world + Schedule (already exists) + XPBD physics on the GPU
  - Snapshot : delta / dirty-chunk recording, and rewind for rollback
  - Net      : server-authoritative, with client-side prediction and reconciliation
  - Space    : WGS84 / ECEF coordinates with a floating origin, in double precision
        |
SushiEngine (ECS, render seam, editor)   — already exists
        |
SushiRuntime (task graph, USM, SYCL)     — fragile API; keep the seam thin
```

## Core decisions

These are locked. Each entry says what we chose and why.

**Authoring model: pure ECS systems.** No per-instance OOP behaviours. SushiLoop is a
fixed-step loop plus a system-registration API. Users write gameplay in C++ as systems
that declare read/write access to components, exactly as the existing `Schedule`
already does. The familiar "MonoBehaviour feel" is delivered as ergonomic naming and
clear lifecycle hooks, never as a return to one-at-a-time virtual `Update()` calls.

**Determinism scope: same-binary deterministic.** The same build, given the same
inputs, produces the same world state every time. We do **not** promise bit-for-bit
identical results across different CPUs, GPUs, or operating systems — that would
require fixed-point math or a heavily constrained software float, and it makes GPU
physics essentially impossible. Agreement between different machines on the network is
handled by a server that is the source of truth (reconciliation and state-sync), not
by lockstep. This is enough for rollback, which only ever re-simulates past frames on
the same machine.

Same-binary determinism has hard rules that the simulation core must always obey:

1. The only source of nondeterminism is player input. Every tick's input is captured,
   numbered, and sent over the network, so rollback can replay it exactly.
2. No wall-clock time (`chrono::now`), no unseeded randomness, and no
   memory-address-dependent iteration order anywhere in the sim. Random number
   generator state lives inside the world like any other component.
3. Fast-math and floating-point reassociation are turned off for the sim. Reduction
   and contact-resolution order is fixed and reproducible — the chunk order and the
   physics graph-colouring already give a stable, deterministic order.

**Physics: GPU XPBD in double precision, with a floating origin.** Physics runs on the
GPU through SushiRuntime kernels. The solver is a single unified XPBD (position-based
dynamics) framework that handles rigid bodies, soft bodies, cloth, and rope in one
place. XPBD is stable under a fixed timestep, parallelises well on the GPU, and behaves
predictably for rollback. It supersedes and extends the current graph-coloured PGS
solver rather than living beside it. Positions are double precision and expressed
relative to a floating origin so that planet-scale coordinates keep their local
precision.

**Rollback state: delta / dirty-chunk snapshots.** To rewind, we record only the
chunks that changed each tick, into a ring buffer, rather than copying the whole world
every tick. This fits the engine's existing model cleanly: the ECS already tracks
structural change through `structure_version` and works chunk by chunk. We add a
per-chunk dirty/version stamp and snapshot just the dirty chunks. When a late input
arrives from the server, we roll the affected chunks back to the earliest touched tick,
replay the intervening inputs, and reconcile.

**Threading: separate sim, render, and net threads, communicating by snapshot.** The
simulation runs on its own thread at a fixed tick rate; that thread is the isolated
island where all the determinism rules apply. After each tick it extracts a snapshot
(the natural growth of the existing `RenderScene` seam). The render thread interpolates
between the two most recent snapshots. The network thread is separate and only hands
the sim a queue of numbered inputs. Keeping determinism confined to one thread is what
makes the whole thing tractable.

**Time model: fixed outer tick, deterministic sub-stepping, interpolated render.** The
loop accumulates real elapsed time and steps the simulation in fixed increments:

```
accumulator += real_delta;
while (accumulator >= FIXED_DT)
{
    sim.tick();
    accumulator -= FIXED_DT;
}
render(interpolation = accumulator / FIXED_DT);
```

Inside a single `tick()`, physics may split into several sub-steps for fast-moving
bodies — but the number of sub-steps is derived from simulation state, never from the
clock, so it stays deterministic. The render thread draws between the last two sim
states using the leftover fraction.

**World scale: a single planet, WGS84 / ECEF, with a floating origin.** The starting
target is one Earth-like planet using ECEF coordinates divided into floating-origin
sectors. Larger scales (solar system, galaxy, hierarchical reference frames) are a
later ambition, not the first build.

**Network topology: server-authoritative with client-side prediction.** A dedicated
server is the authority on world state. Clients predict locally from their own input,
then reconcile against the server's authoritative updates using the rollback
machinery. This is the topology that scales toward an MMO-sized sushiverse.

**Compilation and distribution: one executable.** A game is a single executable that
links the engine, compiled as a single SYCL translation unit — exactly like the
existing sandbox. There is **no** first-class hot-reload, because SYCL device code
(the kernels) is compiled ahead of time and cannot be swapped at runtime. An optional
CPU-only gameplay-script layer that reloads may be added much later, but it is not part
of the core.

**Input and players: a player is an ECS entity, input is a per-tick command buffer.**
Every player is an entity in the world. Input is captured not as raw key presses but as
an abstract list of commands for each tick — this command buffer is numbered, sent over
the network, and is the single thing rollback replays. Modelling players as entities
keeps ownership, authority, and replay uniform: the server owns the authoritative
command stream, and a client's local prediction is just the same commands applied early.

**Editor determinism: play-in-editor runs the same sim core as a standalone build.**
Pressing play in the editor uses the exact same fixed-tick simulation thread as the
shipped executable; the editor is only a host around it. This catches determinism bugs
during development instead of after shipping, and avoids the classic "but it worked in
the editor" trap.

**Randomness and identity: seeded RNG lives in world state, network ids are
deterministic.** The random number generator's state is part of the world, so it is
snapshotted and rewound with everything else during rollback. Entities spawned during
simulation receive the same stable network id on every client, so a spawn on the server
and the matching predicted spawn on a client refer to the same object without a separate
matching step.

**Linux support is first-class**, alongside Windows. The stack already supports it (the
intel/llvm SYCL toolchain, SDL2 + Vulkan, and CMake/Ninja are all cross-platform). The
one discipline required is to keep the simulation core free of Windows-only APIs and
MSVC-specific behaviour.

## Working with the fragile runtime

The SushiRuntime API can change without warning. All the new code that talks to it —
the physics pass and the snapshot recorder both call `graph.add(...)` — must keep those
calls in one narrow place, mirroring how `Schedule` already isolates them. If the
runtime's signatures change, a single file should need updating, not the whole engine.

## Build plan

Each milestone compiles and can be validated on its own.

- **M0 — Type seam.** Firm up `Scalar` for float and double, and add the
  floating-origin / sector vector type for WGS84 to `core/types.hpp`. Small and
  foundational.
- **M1 — SushiLoop skeleton.** The fixed-tick loop, input capture, and the determinism
  guard rails (a build flag that disables fast-math, a seeded RNG component). Prove it
  with a deterministic replay test in the sandbox: the same input stream produces the
  same world hash.
- **M2 — XPBD core.** The unified position-based solver, rigid bodies first. Port the
  existing PGS demo to XPBD and confirm it matches.
- **M3 — Delta snapshots and single-machine rollback.** Chunk-delta recording, rewind
  and replay, and a test for the key invariant: a rollback-and-replay produces exactly
  the same result as an uninterrupted simulation.
- **M4 — Network layer.** Server-authoritative transport with client prediction and
  reconciliation. Loopback first, then real sockets.
- **M5 — Soft bodies and cloth** on top of XPBD, plus stress-testing the floating
  origin with large coordinates.

Linux is kept building on both platforms in CI from M1 onward.
