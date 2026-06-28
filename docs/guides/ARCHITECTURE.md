# SushiEngine Architecture

This document explains how SushiEngine is put together: the relationship to
SushiRuntime, the layers, and the seams a non-trivial change will touch. Read it
before making a change that crosses more than one file.

## 1. The head and the battery

SushiEngine is a **head**; SushiRuntime is a **plugged-in component** — the
battery. The engine is the product a game is built against: it owns the loop, the
world, and — as it grows — the window, the renderer, and the editor. SushiRuntime
is a hardware-agnostic orchestration backbone the engine hands work to.

The dependency points one way only:

```
SushiEngine  ──depends on──▶  SushiRuntime
```

The runtime never knows the engine exists. It has no concept of a game, a frame, an
entity, a component, or a renderer; it schedules an abstract task graph across
whatever hardware is present. This is deliberate, and it is the rule that keeps both
projects changeable: the engine may be rewritten without touching the runtime, and
the runtime may gain backends without the engine noticing.

A practical consequence: a feature that only *composes* the runtime's public API
belongs in the engine. A feature that needs the runtime to do something it cannot
yet do belongs in SushiRuntime, behind its public API — not bolted onto the engine
as a workaround.

## 2. Layers

The engine is header-only at this stage. Each layer depends only on the ones below.

| Layer | Headers | Responsibility |
|-------|---------|----------------|
| Physics     | `physics/pgs_solver.hpp`, `physics/graph_coloring.hpp` | Graph-coloured PGS constraint solver (see §4). |
| Schedule    | `ecs/schedule.hpp` | Compiles systems to a runtime graph and replays it. |
| Commands    | `ecs/command_buffer.hpp` | Records structural changes, applied at a barrier. |
| World       | `ecs/world.hpp` | Entities, archetypes, spawn/destroy, component access. |
| Storage     | `ecs/archetype.hpp`, `ecs/chunk.hpp` | Archetype chunks of structure-of-arrays columns. |
| Identity    | `ecs/entity.hpp`, `ecs/component.hpp` | Entity handles, component ids, access tags. |
| Value types | `core/types.hpp` | The single seam for scalars and vectors (see §6). |

`SushiEngine.hpp` is the umbrella header that pulls the surface together.

## 3. The ECS and the system graph

This is where the engine's "the graph behaves like a game engine" thesis is built —
on the runtime, not in it.

**Storage is archetype chunks.** Entities sharing one component set form an
*archetype*; an archetype stores its entities in fixed-capacity *chunks*. Within a
chunk each component is a separate contiguous column backed by its own runtime
allocation (structure-of-arrays). A column's base pointer is therefore a distinct
resource the runtime's dependency tracker keys on, which is what makes chunks the
unit of parallelism: two systems touching different chunks run in parallel, two
touching the same column are ordered — with no scheduler written in the engine.

**A system is a graph node.** A system declares the components it reads and writes
(`Read<T>` / `Write<T>`); the Schedule emits one node per matching chunk, keyed on
that chunk's columns. The runtime's dependency tracker *is* the system scheduler:
it derives the ordering from component access. A read-after-write on a component
orders the two systems; disjoint access leaves them parallel.

**Counts are late-bound; structure is compiled.** Each node iterates its chunk's
live entity count, re-read every step, so spawning and destroying entities within
existing chunks varies the work with no recompile. The graph is rebuilt only when
the *chunk set* changes (a new archetype or chunk) — reported by the world's
`structure_version`. Pre-reserving chunks keeps a steady spawn/destroy workload at a
single compile.

**Structural changes are deferred.** Systems run as device kernels and must never
see entities appear or vanish mid-frame. Gameplay code records spawns and destroys
into a `CommandBuffer` during the frame, and the loop applies them once, at an
explicit barrier between steps. Destroy is an O(1) swap-remove that keeps a chunk's
live rows packed; entity handles carry a generation so a stale handle to a destroyed
entity is detected, not silently reused.

The worked example in `sandbox/main.cpp` exercises all of this — Position, Velocity,
Mass, and Lifetime components; `apply_forces`, `integrate`, and a parallel
`decay_lifetime` system; per-frame spawn and deferred destroy — and checks every
surviving entity against an independent scalar reference, with the graph compiled
exactly once across the whole run.

## 4. The physics constraint solver

Physics is a domain layer on top of the runtime, the same way the ECS is: it
expresses itself as ordinary read/write sets and lets the dependency tracker do the
ordering. The solver here is **Projected Gauss-Seidel** (PGS), the sequential
constraint method, parallelised by **graph colouring**.

A sequential Gauss-Seidel sweep cannot run all constraints at once: two constraints
that share a body would race. So the constraints are *edge-coloured* over the bodies
(`color_constraints`) — each colour is a batch in which no two constraints share a
body. The `ConstraintSolver` then emits one task per colour: a parallel projection
over that colour's constraints, which is race-free because the bodies are disjoint.
Every colour reads and writes the shared position array, so the runtime orders the
colours into a sequential sweep — colour k+1 after colour k — while parallelising
fully *within* a colour. That ordering is exactly Gauss-Seidel across colours, and
because a colour's constraints are independent, the parallel result equals the
sequential one. The sweep is repeated for the iteration count, and the whole solve is
one graph compiled once and replayed every frame.

The solver owns no engine concept beyond bodies and constraints — it takes a position
array, an inverse-mass array, and a projection functor — so a new constraint type
(contacts, angular joints) is added by providing its POD and its device projection;
the colouring and the graph structure are reused unchanged. `DistanceConstraint` with
`DistanceProjection` is the first concrete type, exercised by `examples/pgs_demo.cpp`
(a hanging chain checked against a scalar reference).

## 5. The render seam (planned)

Rendering does not belong inside the runtime — the runtime knows no graphics, just
as it knows no math. Nor is it bolted onto the side as a second, hand-synchronized
loop. Instead, rendering enters the engine's graph as an **opaque host sink node**:
a node the runtime orders against the simulation by the data it reads (the transform
columns) but whose body — the actual draw calls — is engine code the runtime never
introspects.

The first cut consumes the simulation result after the step's completion latch and
uploads transforms to the renderer (a host round-trip). A later cut promotes
rendering to an in-graph sink node pinned to a render thread, so the scheduler can
overlap the next step's simulation with the current step's draw. Graphics go through
The-Forge; the editor GUI through Dear ImGui.

## 6. The value-type seam

The engine takes its scalar and vector types from `core/types.hpp` and nowhere
else. Those types belong to **SushiBLAS** (tensors, and the floats derived from
them). Until that library exists, `core/types.hpp` aliases a minimal placeholder in
`core/blas_placeholder.hpp`. When SushiBLAS lands, re-point `core/types.hpp` at it
and delete the placeholder — a single-file change, because nothing else in the
engine names the underlying type.

This is the same discipline as §1: one seam, not parallel paths.

## 7. Validation and tooling

The engine ships no device code of its own, so there is nothing to test in
isolation — a meaningful test must instantiate kernels and run them against the
real runtime. The suite in `tests/` does exactly that: it follows SushiRuntime's
layout (`functional/{unit,integration,regression}`, a shared `common/` with the
GoogleTest entry point and a process-wide runtime fixture) and builds one binary,
`se_functional_tests`, as a SYCL translation-unit set. There are no mocks. Tests
carry the `Unit_*` / `Integration_*` / `Regression_*` suite-name prefixes, which
`tests/CMakeLists.txt` turns into CTest labels so a sub-suite is one `ctest -L`
away. The integration tests re-run the sandbox and PGS claims as assertions
(scalar-reference agreement, `compile_count == 1`); the unit tests pin the host
bookkeeping (entity directory, swap-remove, command buffer, graph colouring).

The `se` developer CLI (`cli/`) is the counterpart to the runtime's `sr`: a thin
Typer layer over a service layer that issues the cmake/ctest calls. It owns no
build knowledge the CMake does not — its job is to resolve the toolchain the
engine consumes (SushiRuntime's bundled clang++ and vcpkg) and snapshot the MSVC
environment on Windows, then drive configure/build/test/run. The same one-way
dependency holds: the CLI reads the runtime's `dependencies/` tree but the engine
never reaches back into runtime source.

## 8. Milestones

- **WP-3 — the ECS layer (done).** Archetype-chunk storage, systems scheduled by
  component access, deferred spawn/destroy via a command buffer, compiled once and
  replayed, validated against a scalar reference. This is the substrate plan's first
  end-to-end milestone (the SushiEngine side of WP-3). It uses dense per-chunk
  columns and whole-column resource identity; graduating to region-keyed sub-chunks
  (runtime WP-2) and device residency (WP-6) follows as scale grows.
- **WP-3 — the physics solver (done).** Graph-coloured Projected Gauss-Seidel over
  distance constraints (§4), generic over the constraint type, validated against a
  scalar reference. Next constraint types (contacts, joints) and rigid-body state
  build on the same colouring and graph structure.
- **Rendering.** A window, The-Forge rendering, and Dear ImGui; rendering enters as
  the opaque sink node of §5.
- **Editor host shell.** The editor as a host application that runs the game as a
  scene, with play/pause and inspection panels.
