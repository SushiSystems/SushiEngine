# Overview

This file covers the two facts every other file in this directory rests on: the one-way
relationship between SushiEngine and SushiRuntime, and the layer stack the modules are
arranged into.

## 1. The head and the battery

SushiEngine is a **head**; SushiRuntime is a **plugged-in component** — the battery. The engine
is the product a game is built against: it owns the loop, the world, and — as it grows — the
window, the renderer, and the editor. SushiRuntime is a hardware-agnostic orchestration
backbone the engine hands work to.

The dependency points one way only:

```
SushiEngine  ──depends on──▶  SushiRuntime
```

The runtime never knows the engine exists. It has no concept of a game, a frame, an entity, a
component, or a renderer; it schedules an abstract task graph across whatever hardware is
present. This is deliberate, and it is the rule that keeps both projects changeable: the engine
may be rewritten without touching the runtime, and the runtime may gain backends without the
engine noticing.

A practical consequence: a feature that only *composes* the runtime's public API belongs in the
engine. A feature that needs the runtime to do something it cannot yet do belongs in
SushiRuntime, behind its public API — not bolted onto the engine as a workaround.

## 2. Layers

The engine is header-only at this stage. Each layer depends only on the ones below.

| Layer | Headers | Responsibility |
|-------|---------|----------------|
| SushiLoop core | `engine/world/loop/include/SushiEngine/loop/` — `app.hpp`, `fixed_timestep.hpp`, `input.hpp`, `rollback.hpp`, `net.hpp`; plus `engine/foundation/core/include/SushiEngine/core/random_number_generator.hpp` | The `Loop::App` authoring API over a fixed-step deterministic loop; plus seeded RNG, per-tick input capture, rollback snapshots, and loopback network reconciliation (see [rollback](world.md#1-sushiloop-snapshot-rollback-m3), [SushiLoop core](world.md#2-sushiloop-core), and the [milestones](roadmap.md#1-milestones)). |
| UI | `engine/domain/ui/include/SushiEngine/ui/` — `rect.hpp`, `components.hpp`, `layout.hpp`, `interaction.hpp`, `ui.hpp` | Retained ECS UI (Unity UGUI-shaped): `RectTransform`/`Canvas`/`UIImage`/`UIText`/`UIButton` components, the `resolve_rect` anchor solver, the pointer/click model, and the `UI` façade that builds, lays out, and drives a canvas of buttons (see [`domain-ui.md`](domain-ui.md)). |
| Physics | `engine/domain/physics/include/SushiEngine/physics/` — `core/`, `geometry/`, `collision/`, `constraints/`, `solver/`, `soft/`, `scene/` | Body state and handles, shapes and mass properties, broad/narrowphase, constraint descriptors and their projections, the XPBD solvers behind `IConstraintSolver`, soft-body topology, and the world lifecycle (see [`domain-physics.md`](domain-physics.md)). |
| Geometry | `engine/domain/geometry/include/SushiEngine/geometry/` — `triangle_mesh.hpp`, `mesh_utilities.hpp`, `mesh_distance_query.hpp`, `signed_distance_field.hpp` | Engine-neutral triangle geometry: the mesh value types, topology analysis and repair, a host closest-point hierarchy, and the shared signed-distance baker that queries it. Links nothing — no Vulkan, no SYCL, no runtime — because both the renderer and the physics read it and neither may own it. |
| Cooking | `engine/domain/physics/include/SushiEngine/physics/cooking/` — `cooking_parameters.hpp`, `cooking_report.hpp`, `cooker_interface.hpp`, `cooked_asset_store.hpp`, `collision_cooker.hpp`, `soft_body_cooker.hpp`, `mesh_post_processor.hpp`, `cooking_service.hpp` | The offline pipeline that turns an imported mesh into a simulation asset: `docs/design/physics_system.md` §8.2's fidelity dial, the cook report and the thresholds that fail a bad cook loudly, the `ICookingStage`/`IMeshCooker`/`ICookedAssetStore` seams, the content-hash cache, the two cookers that produce `.sushicollision` and `.sushisoft`, and the ordered import chain that runs them on a worker thread. Host-only, links only Geometry (plus Threads), and never linked into a shipping runtime path — an importer that needs a GPU is an importer that fails on a build machine. Getting triangles out of a file is a `MeshLoader` seam the consumer wires, which is why this module needs no cgltf (see §8 of `docs/design/physics_system.md`). |
| Atmosphere | `engine/domain/atmosphere/include/SushiEngine/atmosphere/` — `fourier_transform.hpp`, `quasigeostrophic_core.hpp` | T1, the global dynamical core: two-layer moist quasi-geostrophic flow on a latitude/longitude grid, from which cyclones, fronts, the jet and storm tracks emerge rather than being placed (see §5 of `docs/design/atmosphere_system.md`). Links nothing, for the same reason Geometry does — its consumers are gameplay queries, the regional nest's parent forcing, and a headless probe, and none should need a device to ask what the weather is. The regional nest (T2) is a Vulkan compute service and stays under `engine/presentation/render/source/atmosphere/`. |
| Animation | `engine/domain/animation/include/SushiEngine/animation/` — `skeleton*.hpp`, `clip*.hpp`, `animator_*.hpp`, `blend_tree.hpp`, `avatar_mask.hpp`, `additive.hpp`, `pose_modifier.hpp`, `ik_*.hpp`, `morph.hpp`, `generic_track.hpp`, `humanoid.hpp`, `retarget.hpp`, `edit_preview.hpp`, `animation_database.hpp` | Skeletal-animation stack (phases A0–A9): skeleton/clip/controller/mask assets, the deterministic `animator_step`, the `AnimatorEvaluator` (blend trees, mask-gated layers, additive), the IK / pose-modifier stack, morph + generic tracks, humanoid retargeting, and controller JSON authoring, behind the `IAnimationDatabase` seam (see [`domain-animation.md`](domain-animation.md)). |
| Execution | `engine/foundation/execution/include/SushiEngine/execution/` — `access.hpp`, `interval.hpp`, `node_descriptor.hpp`, `hazard.hpp`, `context.hpp` | The seam every subsystem allocates and schedules through: the access algebra (`AccessIntent`, `BufferInterval`, `DeterminismClass`), the normative hazard semantic, and the `Context`/`Graph`/`Buffer` names a compile-time backend policy resolves. SushiRuntime is one implementation of it (`engine/foundation/execution/include/SushiEngine/execution/backend/runtime_backend.hpp`), not the thing the engine is typed against (see [the ECS and the system graph](foundation.md#1-the-ecs-and-the-system-graph) and `docs/design/unified_hazard_model.md`). |
| Schedule | `engine/foundation/ecs/include/SushiEngine/ecs/schedule.hpp` | Compiles systems to an execution graph and replays it. |
| Commands | `engine/foundation/ecs/include/SushiEngine/ecs/command_buffer.hpp` | Records structural changes, applied at a barrier. |
| World | `engine/foundation/ecs/include/SushiEngine/ecs/world.hpp` | Entities, archetypes, spawn/destroy, component access. |
| Storage | `engine/foundation/ecs/include/SushiEngine/ecs/archetype.hpp`, `.../ecs/chunk.hpp` | Archetype chunks of structure-of-arrays columns. |
| Identity | `engine/foundation/ecs/include/SushiEngine/ecs/entity.hpp`, `.../ecs/component.hpp` | Entity handles, component ids, access tags. |
| Value types | `engine/foundation/core/include/SushiEngine/core/types.hpp` | The single seam for scalars and vectors (see [the value-type seam](foundation.md#2-the-value-type-seam)). |

`engine/include/SushiEngine/SushiEngine.hpp` is the umbrella header that pulls the surface
together.
