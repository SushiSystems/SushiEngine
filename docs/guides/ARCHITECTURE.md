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

The runtime never knows the engine exists. It has no concept of a game, a frame, a
component, or a renderer; it schedules an abstract task graph across whatever
hardware is present. This is deliberate, and it is the rule that keeps both
projects changeable: the engine may be rewritten without touching the runtime, and
the runtime may gain backends without the engine noticing.

A practical consequence: a feature that only *composes* the runtime's public API
belongs in the engine. A feature that needs the runtime to do something it cannot
yet do belongs in SushiRuntime, behind its public API — not bolted onto the engine
as a workaround.

## 2. Layers

The engine is small and header-only at this stage. Each layer depends only on the
ones below it.

| Layer | Header | Responsibility |
|-------|--------|----------------|
| Application | `app/application.hpp` | Owns the runtime, world, and loop; drives stepping. |
| Simulation  | `sim/simulation.hpp`  | Builds the per-step task graph from a `World` and replays it. |
| World / ECS | `ecs/world.hpp`, `ecs/components.hpp` | Structure-of-arrays component store over runtime memory. |
| Value types | `core/types.hpp`      | The single seam for scalars and vectors (see §5). |

`SushiEngine.hpp` is the umbrella header that pulls the surface together.

## 3. The simulation graph

A `World` keeps each component as one contiguous field backed by runtime memory:
positions live in a double-buffered `State` (a step reads the current field and
writes the next, and the runtime swaps them), velocities in a plain `Buffer`. The
fields are shared USM, so the host may seed and read them while a device kernel
drives the arrays.

`Simulation` expresses one fixed timestep as a SushiRuntime graph. A *system* is a
graph node; the dependency tracker orders systems by the component fields they read
and write, so the runtime's scheduler *is* the system scheduler. The graph is built
once in the constructor and replayed every step, so the runtime compiles it exactly
once and steady-state stepping pays no analysis cost.

The Milestone A graph is a single node — integrate position by velocity. Real games
add more systems (gravity, collision, culling, AI); they compose the same way, each
naming the fields it touches.

## 4. The render seam (planned)

Rendering does not belong inside the runtime — the runtime knows no graphics, just
as it knows no math. Nor is it bolted onto the side as a second, hand-synchronized
loop. Instead, rendering enters the engine's graph as an **opaque host sink node**:
a node the runtime orders against the simulation by the data it reads (the transform
fields) but whose body — the actual draw calls — is engine code the runtime never
introspects.

The first cut consumes the simulation result after the step's completion latch and
uploads transforms to the renderer (a host round-trip). A later cut promotes
rendering to an in-graph sink node pinned to a render thread, so the scheduler can
overlap the next step's simulation with the current step's draw. Graphics go through
The-Forge; the editor GUI through Dear ImGui.

## 5. The value-type seam

The engine takes its scalar and vector types from `core/types.hpp` and nowhere
else. Those types belong to **SushiBLAS** (tensors, and the floats derived from
them). Until that library exists, `core/types.hpp` aliases a minimal placeholder in
`core/blas_placeholder.hpp`. When SushiBLAS lands, re-point `core/types.hpp` at it
and delete the placeholder — a single-file change, because nothing else in the
engine names the underlying type.

This is the same discipline as §1: one seam, not parallel paths.

## 6. Milestones

- **A — headless core (done).** The loop runs with no rendering: a world is
  integrated each fixed timestep on a runtime graph, and the result is validated
  against its closed-form value. Proves the head drives the battery.
- **B — window and rendering.** A window, The-Forge rendering, and Dear ImGui;
  rendering enters as the opaque sink node of §4.
- **C — editor host shell.** The editor as a host application that runs the game as
  a scene, with play/pause and inspection panels.
