# VFX Particle System — authoring model, dual simulation backends & render integration (SushiEngine::Vfx)

This document is the **umbrella** for SushiEngine's AAA VFX particle system: the product vision (a
Niagara/VFX-Graph-class effect authoring pipeline feeding both a GPU-cosmetic simulator and a
CPU-deterministic simulator), the data model an artist authors, the two simulation backends behind a
single seam, the render-graph integration, and the editor authoring surface. The heavy per-particle
math lives in compute shaders (`render/shaders/particle_*.comp`) and in the deterministic CPU
integrator; this doc specifies the **architecture and the seams**, not the shader source.

Companion docs: `animation_system.md` (the structural template — trivially-copyable state
column + snapshot extract + a compute subsystem wired end to end), `audio_system.md` (the
sibling wall-clock snapshot consumer that also lives *outside* the deterministic island), and the
renderer's `../ARCHITECTURE.md` (the render graph the draw passes plug into).

**Status (2026-07-24): VFX1 + the deterministic-ECS connection + all of VFX2's lighting are
implemented** (`se editor --no-run` green each increment; the newest step is build-pending + awaits a
GPU visual check). Done: the whole VFX1 vertical slice (authoring model, both simulation backends, GPU
emit/simulate/billboard render, blend-state prerequisite, quality tiers, editor panel + emitter gizmo,
demo, tests); the **deterministic emitter → ECS** connection (`IWorldEditor` emitter quartet, host-side
pools on `RuntimeSimulation::Record`, `ParticleBillboard` extract, editor
GameObject/Add-Component/Inspector/`.sushiscene`); **VFX2a** (per-blend bucketing → additive + true-alpha
draws, and GPU **bitonic back-to-front depth-sort** of the alpha bucket); **VFX2b sun-lit particles**
(world-space directional sun + ambient on the alpha bucket — the camera-relative hazard sidestepped); and
**VFX2c clustered-punctual-light particles** — the froxel version: the lit (true-alpha) bucket now maps
each sprite to a cluster and shades it with the scene's punctual lights + the environment SH ambient, in
the same camera-relative space the meshes use. The froxel primitives were refactored into a binding-free
`clustered_lighting_common.glsl` shared with `pbr.frag` (§7.7), and the camera-relative hazard was met by
carrying `centre − eye` down from the vertex stage and reading the view-depth from `gl_Position.w`; the
one accepted limitation is that this camera-relative subtraction is in float, so it inherits the cosmetic
pool's float32 position precision (a near-camera envelope, documented in §13). The roadmap in §12 marks
each item's state.

---

## §0 The one decision that shapes everything — a hybrid, two-backend system

Particles in a deterministic networked engine face a fork: gameplay-affecting particles must be
**bit-reproducible** for rollback/replay (SushiLoop's determinism rules), but AAA visual scale
(millions of sparks, smoke, dust) is only reachable on the **GPU**, which is non-deterministic and
must stay *out* of the deterministic sim island. SushiEngine resolves this by shipping **both**, behind
one authoring model and one backend seam:

```
                          ParticleEffect  (one authored asset, artist-facing)
                                   │  EmitterCompiler bakes → CompiledEffect (POD + LUTs)
             ┌─────────────────────┴─────────────────────┐
   domain = Cosmetic                              domain = Deterministic
             ▼                                             ▼
   GpuParticleSystem (render-side)              CpuDeterministicBackend (ECS tick)
   compute emit→simulate→sort→draw              fixed pools, PCG32, Euler integrate,
   millions, non-deterministic,                 byte-snapshottable, rollback-safe,
   OUTSIDE the deterministic island             INSIDE the fixed-step schedule,
   (like skinning / audio)                      thousands, gameplay-authoritative
             └─────────────────────┬─────────────────────┘
                     both render through the same billboard/mesh draw path
```

Every `ParticleEmitter` carries a `SimulationDomain` enum; the engine routes it to the matching backend.
The two backends are **interchangeable** behind `Vfx::IParticleBackend` (LSP), and both consume the
**same** `CompiledEffect`, so an artist authors once and only picks a domain. This is the direct
consequence of the user's "hybrid" decision — the flexibility cost (two simulators) is accepted for the
reach it buys: real gameplay particles that survive netcode *and* GPU-scale cosmetic spectacle.

**Invariant:** the cosmetic path reads emitter transforms from a snapshot and writes no sim state
(a run is byte-identical with cosmetic VFX on or off); the deterministic path's entire per-tick state is
a fixed-size, pointer-free, integer-seeded column, exactly like `Animation::AnimatorInstance`.

---

## §1 Why none exists today — audit

1. **Greenfield.** No `include/SushiEngine/vfx/`, no particle component, no particle pass, no shader.
2. **No blend state in the pipeline factory.** `Render::Resources::GraphicsPipelineDesc`
   (`render/resources/pipeline_cache.hpp`) and `fill_color_blend()` (`pipeline_cache.cpp`) hard-set
   `blendEnable = VK_FALSE`; additive/alpha billboards are impossible until the desc gains per-attachment
   blend fields **and** those fields join the byte-comparable fragment-output pipeline-library cache key.
   This is phase VFX1's first task (§7.5).
3. **No extract channel for emitters.** `Render::ISceneView::render(...)` (`scene_view.hpp:253`) takes
   POD instance arrays (`MeshInstance`, `SkinnedInstance`, `ClothStrandView`, lights, decals) but no
   emitter channel — a trailing `emitters`/`emitter_count` pair is added exactly like `skinned` was.
4. **No effect asset registry.** Animation has `AnimationDatabase`; VFX needs the sibling
   `Vfx::EffectDatabase`.

---

## §2 Survey — what this system adopts

| Source | What SushiEngine takes (model only, no code/dependency) |
|---|---|
| Unreal **Niagara** | Emitter = an ordered **stack of composable modules** (spawn / init / update / render); modules read & write per-particle *attributes*; system = many emitters. This is the authoring vocabulary. |
| Unity **VFX Graph** | GPU-first simulation; property-over-lifetime via **curves & gradients** baked to LUTs sampled in the sim shader; block-based capacity. |
| Unreal **Cascade** | The simpler fixed-module emitter as the CPU-deterministic path's baseline (bounded, cheap, predictable). |
| **Wicked Engine** GPU particles | The GPU **dead-list / alive-list + atomic counter** allocator and the compute→indirect-draw handoff. |
| SushiEngine **skinning** subsystem | The end-to-end wiring template: system-owned VMA pools, a compute `IRenderPass`, a consumer draw pass, the extract POD, the `passes_` slotting (`render/scene/skinning_system.*`, `render/passes/skinning_pass.*`). |
| SushiEngine **animation** `AnimatorInstance` | The deterministic, byte-snapshottable, integer-RNG state-column pattern for the CPU backend. |

**Skip list:** offline baking / simulation caches (real-time only for now); CPU-side sorting for the GPU
path (sorted on the GPU); network-replicated cosmetic particles (output-only, outside the island).

---

## §3 Architecture — one asset, two backends, one draw path

```
 AUTHORING (editor + engine, plain C++)          SIMULATION                         RENDER
 ─────────────────────────────────────   │   ─────────────────────────   │   ────────────────────────
  ParticleEffect                          │                               │
   └─ EmitterDescriptor[]                 │   Cosmetic emitters:          │   ParticleSimPass (compute)
       ├─ SpawnModule[]                   │     GpuParticleSystem.prepare  │     emit + simulate
       ├─ ShapeModule                     │     per-slot VMA pools ────────┼─▶  ParticlePass (graphics)
       ├─ InitModule[]                    │                               │     billboard indirect draw
       ├─ UpdateModule[]  (curves/grads)  │   Deterministic emitters:     │     into frame.targets.scene_final
       └─ RenderModule                    │     CpuDeterministicBackend    │        (samples depth for soft
   EmitterCompiler ──▶ CompiledEffect ────┼──▶  ECS system, fixed pools ───┼─▶      fade + hard occlusion)
     (POD params + baked curve/grad LUTs) │     extract positions          │
                                          │                               │
  Vfx::IParticleBackend seam (LSP): GpuParticleSystem | CpuDeterministicBackend
```

The renderer never sees an `EmitterDescriptor`, a curve, or a module — only a `CompiledEffect` (POD +
LUT bytes) and a per-frame `ParticleEmitterView` (transform + effect handle + domain + time). This is
the same DIP boundary the renderer already holds against animation (palette floats + a mesh, never a
clip). The editor never sees the SYCL runtime — it talks to engine headers and a thin
`IParticleEffectStore`.

---

## §4 Data model

### 4.1 Modules — a data-oriented, Open/Closed taxonomy

A module is **not** a virtual object (per-particle GPU code can't dispatch through a vtable, and
components must stay trivially copyable). A module is a **tagged POD descriptor**; the module *set* is an
enum, and behaviour lives in three places that a new module extends without touching existing ones:
(1) a descriptor struct, (2) an `EmitterCompiler` handler that bakes it into `CompiledEffect`, (3) a
branch in the sim shader / CPU integrator keyed on the compiled flag. Header:
`include/SushiEngine/vfx/modules.hpp`.

| Stage | Modules (VFX1 slice) | Later |
|---|---|---|
| **Spawn** | rate-over-time, burst-list | distance-based, event-triggered |
| **Shape** (initial position + emit direction) | point, sphere, hemisphere, cone, box, circle | mesh-surface, skeleton-socket |
| **Init** (per-particle at birth, min/max ranges) | lifetime, velocity, size, color, rotation, mass | initial mesh index, custom attribute |
| **Update** (per-tick) | velocity integrate, gravity, drag, curl-noise turbulence, size-over-life (curve), color-over-life (gradient), rotation-over-life, velocity-over-life, flipbook/sub-UV | vortex, point/line attractor, drag volume, GPU collision, orbital |
| **Render** | billboard, additive\|alpha\|premultiplied, soft depth-fade, texture, sort mode | stretched/velocity-aligned, mesh, ribbon/trail, lit, six-way lightmap |

Per-particle **attributes** (the working set both backends carry): `position`, `velocity`, `age`,
`lifetime`, `size`, `color` (rgb), `alpha`, `rotation`, `angular_velocity`, `seed`, `flipbook_index`.
GPU layout is float AoS (`GpuParticle`, §5.1); CPU layout mirrors it in `Vector3f`/float.

### 4.2 Curves & gradients

- `include/SushiEngine/vfx/curve.hpp` — `AnimationCurve`: keyframed (time, value, in/out tangent),
  evaluatable on the CPU, and **bakeable** to a fixed-width `float` LUT (default 64 samples) uploaded
  as a 1-D texture / SSBO row the sim shader samples by normalized age.
- `include/SushiEngine/vfx/gradient.hpp` — `ColorGradient`: separate color keys + alpha keys, bakeable
  to an RGBA8 (or RGBA16F) LUT row. Color-over-life samples it by normalized age.

Both are authoring types (heap-backed key vectors); neither crosses into a component or the GPU — only
their **baked LUT bytes** do, inside `CompiledEffect`.

### 4.3 CompiledEffect — the POD boundary

`include/SushiEngine/vfx/compiled_emitter.hpp` — `EmitterCompiler` flattens one `EmitterDescriptor` into
a `CompiledEmitter` POD: packed spawn params, shape enum + params, init ranges, a bitfield of enabled
update modules + their scalar params, render flags (blend mode, sort, lit), texture/atlas ids, capacity,
and **offsets into a shared LUT atlas** for its baked curves/gradients. `CompiledEffect` = a span of
`CompiledEmitter` + the LUT atlas bytes. This is the single artifact both backends and the GPU consume —
the equivalent of `resolve_quality()` turning authored `RenderSettings` into a POD `QualityParams`.

### 4.4 The asset & its registry

- `include/SushiEngine/vfx/particle_effect.hpp` — `ParticleEffect` = `std::vector<EmitterDescriptor>` +
  metadata (name, bounds, default domain). The artist's document; serialized as `.sushieffect` (JSON via
  `nlohmann_json`, the format the scene serializer already uses).
- `include/SushiEngine/vfx/effect_database.hpp` — `EffectDatabase`: `AssetId → ParticleEffect` +
  lazily-built `CompiledEffect`, mirroring `Animation::AnimationDatabase`. Owns compilation caching.

### 4.5 ECS component

`sim/components.hpp` gains `ParticleEmitter` (the central "one home so ids stay stable across TUs" file):

```
struct ParticleEmitter                 // trivially copyable, byte-snapshottable
{
    AssetId          effect;           // into EffectDatabase
    SimulationDomain domain;           // Cosmetic | Deterministic
    std::uint32_t    seed;             // per-emitter RNG seed (deterministic path)
    float            time;             // seconds since play (deterministic path advances this)
    float            spawn_accumulator; // fractional-spawn carry
    std::uint32_t    flags;            // playing / looping / prewarmed
};
```

Position/orientation are **not** duplicated — read from `Simulation::Transform` / `Orientation`, the
same columns the renderer and audio read.

### 4.6 Extract POD

`render/scene_view.hpp` gains, alongside `SkinnedInstance`:

```
struct ParticleEmitterView             // the render-side extract seam
{
    Mat4                  model;       // emitter object-to-world
    const void*           compiled;    // CompiledEmitter* (renderer treats as opaque bytes)
    std::uint32_t         effect_id;   // stable key for pool reconciliation
    std::uint32_t         emitter_index;
    SimulationDomain      domain;
    float                 time;
    float                 dt;
    std::uint32_t         spawn_count; // particles to emit this frame (host-computed)
    std::uint32_t         id;          // picking id (0 = none)
};
```

Added as a trailing `emitters`/`emitter_count` pair on `ISceneView::render(...)` and to
`Frame::SceneDrawList` in `render/frame/frame_context.hpp`, copied in `VulkanSceneView::render` exactly
where `skinned` is copied.

---

## §5 GPU-cosmetic backend

`render/scene/particle_system.cpp` — modeled feature-for-feature on `SkinningSystem`
(`Allocation`/`grow()`/`destroy()`, one buffer set per frame slot). It owns:

### 5.1 Persistent, system-owned pools (per emitter, ping-pong, grown on demand)

- `GpuParticle` state buffer (AoS, std430 float): `vec4 position_life; vec4 velocity_age;
  vec4 color_alpha; vec4 size_rot_seed_flip;` — 64 bytes, one `static_assert`-locked struct shared with
  the shader (the `GpuInstance` precedent).
- **dead-list** (free indices), **alive-list** ping-pong (current/next), and an atomic **counter buffer**
  (alive count + dead count + indirect-dispatch args). This is the standard GPU particle allocator.
- The **compiled-effect table + LUT atlas** uploaded once per change (change-gated like the atmosphere
  LUTs), not per frame.

These carry state frame-to-frame, so they are **system-owned VMA** and hand-barriered — they must never
be graph transients (transients alias and recycle each `begin_frame`).

### 5.2 Per-frame graph transients (the sim→draw handoff)

The **draw-instance buffer** (this frame's compacted alive particles the vertex shader pulls) and the
**indirect draw-args buffer** are declared as graph transients in
`render/rhi/vulkan/view_resources.cpp declare_targets()` beside `draw_commands`, usage
`STORAGE_BUFFER | INDIRECT_BUFFER`. `ParticleSimPass` declares `write(StorageWrite)`; `ParticlePass`
declares `read(instances, StorageRead)` + `read(args, IndirectRead)`. The graph then derives the
compute-write → indirect/vertex-read barrier automatically — cleaner and async-safe versus a hand
barrier, and the exact pattern cull→opaque already uses.

### 5.3 The compute pipeline (per emitter, per frame)

1. **Emit** (`particle_emit.comp`): read `spawn_count`, pop indices from the dead-list, initialize each
   new particle from the shape + init modules (seeded by `seed` + particle index), push to the alive-list.
2. **Simulate** (`particle_simulate.comp`): per alive particle apply the enabled update modules — forces
   (gravity/drag/curl-noise), integrate position/velocity, advance age, sample the size curve / color
   gradient LUTs by normalized age, advance flipbook. Retire (`age >= lifetime`) → push index back to the
   dead-list; survive → append to the compacted draw-instance buffer and `atomicAdd` the indirect
   vertex/instance count.

`ParticleSimPass : IRenderPass` mirrors `SkinningPass`: owns its compute pipeline
(`pipelines_.create_compute(layout, shaders_.module("particle_simulate.comp"))`), per-frame descriptor
sets from `frame.descriptors`, `vkCmdDispatch`, `rebuild_pipelines()` hot-reload hook. Registered in
`passes_` **immediately after `skinning_pass_`** (compute batch, before `depth_prepass_`).

### 5.4 The draw pass

`ParticlePass : IRenderPass`, graphics, `vertex_stride = 0` — the vertex shader expands 6 verts/particle
from `gl_VertexIndex` and pulls the particle from the draw-instance storage buffer (the fullscreen-pass
precedent, `cloud_composite_pass.cpp:105`). It:

- declares `color_attachment(0, frame.targets.scene_final, Load)` and **does not attach depth**;
- declares `read(frame.targets.depth, SampledFragment)` and samples it in the fragment shader for both
  the **soft depth-fade** and a **hard occlusion** test (compare billboard view-Z to scene depth). By
  the time this pass runs, depth is already `SHADER_READ_ONLY_OPTIMAL` (SSR sampled it), so no transition;
- blends additive / alpha / premultiplied via the new blend-state path (§7.5);
- issues `vkCmdDrawIndirect` against the transient args buffer.

Registered in `passes_` **between `ssr_pass_` and `taa_pass_`**, writing `scene_final`. Not before SSR
(would pollute reflections), not after TAA (would lose temporal AA + hit an extent mismatch). Particles
write no velocity in the slice; fast sparks ghost mildly under TAA the same way clouds do — revisited in
VFX2 if a post-TAA composite into `targets.resolved` becomes necessary.

### 5.5 Queue

VFX1 runs the sim on the **graphics queue** (no `set_queue(AsyncCompute)`), mirroring skinning/cull —
simplest-correct. Async compute is a later optimization; if taken, the §5.2 handoff buffers stay graph
transients so the graph both barriers and marks them concurrent, and the persistent pools are touched
only inside the compute pass.

---

## §6 CPU-deterministic backend

`CpuDeterministicBackend` runs as an **ECS system inside the fixed-step schedule** (the sim TU,
`sushi_sim`), never on wall clock. Its entire per-tick state is a **fixed-size, pointer-free column** so a
tick is byte-snapshottable and bit-reproducible for rollback — the `AnimatorInstance` contract:

- particles live in a `constexpr`-capped per-emitter pool (e.g. `MAX_DETERMINISTIC_PARTICLES` per
  emitter); no heap, no `std::vector` growth mid-tick;
- RNG is **PCG32** seeded from `ParticleEmitter::seed` + a per-tick counter — no wall clock, no
  `Math::random`;
- integration is fixed-step Euler over the same `CompiledEmitter` params + baked LUTs the GPU path uses
  (curves evaluated by sampling the same LUT so both backends agree visually);
- spawn/kill are deterministic functions of `(seed, tick, accumulator)`.

Extract reads the live pool's positions/sizes/colors into `ParticleEmitterView`-adjacent instance data
and renders them through the **same** billboard `ParticlePass` (a host-filled instance buffer path rather
than the GPU compute path). Gameplay may query the pool (e.g. "is this cell on fire") because it is
authoritative sim state.

The determinism test (`Integration_ParticleDeterminism`) asserts (a) two independent runs of the same
seed + tick stream produce byte-identical pool state, and (b) a snapshot → advance → restore → replay
reproduces the captured state byte-for-byte (`std::memcmp == 0`) — exactly as `animator_demo.cpp` proves
for `AnimatorInstance`.

---

## §7 Render-graph integration (validated against current code)

| Concern | Decision (with the seam) |
|---|---|
| **Sim pass slot** | after `skinning_pass_`, before `depth_prepass_` (`vulkan_scene_view.cpp:125-156`) |
| **Draw pass slot** | between `ssr_pass_` and `taa_pass_`, writes `frame.targets.scene_final` |
| **Persistent state** | system-owned VMA, `grow()/destroy()/Allocation` (`skinning_system.cpp:52-83`) |
| **Sim→draw handoff** | graph transients in `view_resources.cpp declare_targets()` (`STORAGE\|INDIRECT`, like `draw_commands` `:772-786`) |
| **Depth for soft particles** | sample only (`SampledFragment`), never attach; soft-fade + occlusion in FS |
| **Queue** | graphics (no async) for the slice |
| **Billboards** | `vertex_stride = 0`, expand from `gl_VertexIndex`, pull from storage buffer |
| **Extract wiring** | `ParticleEmitterView` + `emitters/emitter_count` on `render()` + `SceneDrawList` + copy in `VulkanSceneView::render` |
| **Build** | shaders via `sushi_compile_shader` + `${…_HEADER}` in `render/CMakeLists.txt` + catalogue in `shader_catalogue.cpp`; sources into `add_library(sushi_render …)` |

### 7.5 Blend-state prerequisite (do first)

Extend `Render::Resources::GraphicsPipelineDesc` with per-attachment blend (`bool blend_enable`, src/dst
color & alpha factors, color & alpha ops), teach `fill_color_blend()` to honor them, and **fold the new
fields into the byte-comparable fragment-output pipeline-library cache key** so two pipelines that differ
only in blend don't collide. Defaults reproduce today's opaque behaviour, so every existing pass is
untouched. Without this, "transparent" particles draw opaque.

### 7.6 Quality tiers

`render/quality_params.hpp` gains `bool gpu_particles`, `std::uint32_t max_particles`,
`std::uint32_t particle_sim_substeps`, `std::uint32_t particle_lod_bias`; `render/frame/quality.cpp`
scales them per tier (Low disables `gpu_particles`, Ultra raises `max_particles`), mirroring the
`max_skinned_instances` precedent. Passes early-out when `!frame.quality.gpu_particles`.

### 7.7 Clustered lighting for the lit bucket (VFX2c)

The lit (true-alpha) bucket shades with the scene's punctual lights through the same Forward+ froxel
grid the meshes use, plus the environment SH ambient — a diffuse blob, so no BRDF and no per-light
shadow. Three decisions kept it correct **and** device-agnostic:

- **One copy of the froxel math.** The pure primitives — the grid `#define`s, the `PunctualLight`
  struct, `cluster_index(frag, view_z, depth, screen)`, and `punctual_attenuation(...)` — moved to a
  binding-free `render/shaders/clustered_lighting_common.glsl`. `clustered_lighting.glsl` now includes
  it and keeps only its own `set 0` bindings 14–22 and the shading that reads them (its
  `cluster_index_for` became a one-line wrapper, behaviour-identical for `pbr.frag`); `particle.frag`
  includes the same header and declares a **smaller subset** of those buffers on the pass's own set
  (bindings 3–7), so the two consumers share the math but bind on different sets. This is the header's
  Open/Closed seam — a new consumer adds a binding block, not another copy of `cluster_index`.
- **Camera-relative without a precision cliff or a sign trap.** Particles are stored in absolute-world
  float, but the froxels and `light_buffer` are camera-relative. The vertex stage subtracts the eye
  (`centre − eye`, the eye packed into the push's spare `w` lanes exactly as `scene_uniforms` packs it)
  and passes the camera-relative centre down as a varying; the froxel **view-depth is read straight from
  `gl_Position.w`** (the perspective clip `w` equals the positive view distance the z-slice is keyed on),
  which needs no `cam_forward` vector and so has no cross-product sign ambiguity. The one accepted
  limitation: the `centre − eye` subtraction is in float, so it inherits the cosmetic pool's float32
  position precision — a near-camera envelope (see §13).
- **Within the 128-byte push budget.** No new push space was spent: the eye rides `camera_right.w`,
  `camera_up.w`, `sun_direction.w`, and the lit flag doubles as the SH-ambient scale on `sun_radiance.w`
  (`<0` = unlit/emissive, `>=0` = lit with that IBL ambient scale), so the block stays exactly 128 bytes,
  matching `MeshPushConstants` — the house budget that keeps the layout portable to a 128-byte device.

The pass reads `frame.targets.cluster_grid` / `light_index` (so the graph derives the compute→fragment
barrier), and binds `lights_.light_buffer()`, `lights_.config_buffer()` (a truncated, layout-compatible
`ClusterBlock`), and `ibl_.sh_buffer()` on every bucket — the unlit fragment path never samples them.

---

## §8 Editor authoring

Modeled on the animation subsystem's seams, but promoted to real panels (not the inlined-in-main shortcut
the skeleton preview used).

- `editor/vfx/effect_preview.{hpp,cpp}` — an `EffectPreview` state class (owns the previewed
  `ParticleEffect`, its `EffectDatabase` handle, a preview world `Mat4`, playback clock) + a free-function
  viewport overlay `draw_emitter_gizmo(preview, camera_view, image_origin, w, h, draw_list)` that paints
  the emitter shape (sphere/cone/box wireframe) and bounds, mirroring
  `animation/skeleton_debug_draw`'s `draw_skeleton_overlay`.
- `draw_particle_editor_panel(EditorContext&)` in `editor/ui/editor_panels.cpp` — the module-stack
  authoring UI (emitter list, per-stage module add/remove/reorder, module params), a **curve editor** and
  a **gradient editor** widget (ImGui draw-list based), a preview toolbar (play/pause/restart, domain
  toggle). Registered at the three edit points (`PanelVisibility` bool, Window-menu `MenuItem`, the
  `draw_*_panel` call block in `main.cpp`) + a `DockBuilderDockWindow` placement.
- Emitter gizmo threads into the Scene `ViewportPanel::draw` overlay region and the existing gizmo
  drag + `history.begin_change/end_change` undo bracket (an emitter is an entity with a `ParticleEmitter`
  component; its transform moves through `IWorldEditor` like any other).
- `editor/CMakeLists.txt` gains `vfx/effect_preview.cpp`; panel functions need no CMake change.

---

## §9 SOLID

- **SRP** — authoring model (`vfx/`), compilation (`EmitterCompiler`), GPU resources
  (`ParticleSystem`), compute (`ParticleSimPass`), draw (`ParticlePass`), deterministic sim
  (`CpuDeterministicBackend`), and editor UI are each one responsibility in one place.
- **OCP** — a new module = one descriptor + one compiler handler + one shader/integrator branch keyed on
  a compiled flag; no existing module, pass, or backend is edited. A new render style (mesh, ribbon) = a
  new `RenderModule` variant + a draw path, gated by a flag. A new backend = a new `IParticleBackend`.
- **LSP** — `GpuParticleSystem` and `CpuDeterministicBackend` are substitutable behind
  `IParticleBackend`; the caller picks by `SimulationDomain` and treats them identically.
- **ISP** — the editor depends on a thin `IParticleEffectStore` (load/save/list effects), not on the
  runtime; the renderer depends on `ParticleEmitterView` + opaque `CompiledEmitter` bytes, not on
  authoring types; the extract POD is separate from the authoring model.
- **DIP** — high-level policy (what an effect *is*) is authoring-side; low-level mechanism (how it
  simulates/draws) depends on the POD `CompiledEffect` abstraction, never the reverse. The renderer and
  the editor both depend on interfaces/PODs, not concretions.

---

## §10 Fixed capacities & tiers (VFX1 baseline)

| Cap | Value | Where |
|---|---|---|
| GPU particles (Ultra) | 4 M | `QualityParams::max_particles` |
| GPU particles (High) | 1 M | tier scale |
| GPU particles (Low) | disabled | `gpu_particles = false` |
| Deterministic particles / emitter | 1 024 | `MAX_DETERMINISTIC_PARTICLES` |
| Emitters / effect | 16 | `EmitterDescriptor` cap |
| Curve LUT width | 64 | `EmitterCompiler` |
| Gradient LUT width | 64 | `EmitterCompiler` |
| Update modules / emitter | 32 (bitfield) | `CompiledEmitter` |
| Frame slots | inherited from the scene view | `ParticleSystem` |

---

## §11 Testing & determinism

- `Unit_ParticleCurve` / `Unit_ParticleGradient` — keyframe evaluation vs analytic reference; LUT bake
  round-trips within tolerance.
- `Unit_EmitterCompiler` — an authored `EmitterDescriptor` compiles to the expected `CompiledEmitter`
  flags/params/LUT offsets.
- `Unit_ParticleSpawn` — rate + burst produce the expected counts over N ticks; pool never exceeds
  capacity; dead/alive accounting balances.
- `Integration_ParticleDeterminism` — the two byte-exact checks in §6 (independent-run equality + rollback
  replay), following `test_input_determinism.cpp`'s two-session structure.
- `examples/particle_demo.cpp` — a headless self-checking demo (GPU path where a device is present; CPU
  path always) returning 0/1, registered via `add_sushi_sycl_executable`.

No mocks — everything runs against the real runtime, per house style.

---

## §12 Roadmap (✅ done · ◐ partial · ☐ to do — status 2026-07-24)

- ✅ **VFX1 — Vertical slice.** Blend-state prerequisite; the `vfx/` authoring model + curves/gradients +
  compiler; `ParticleEmitter` component; extract seam; `GpuParticleSystem` + emit/simulate/billboard
  passes + shaders; `CpuDeterministicBackend`; quality gating; editor panel + emitter gizmo; demo +
  tests + docs. **Reached: authored emitters render as GPU particles in the viewport; the deterministic
  path passes the rollback test.**
- ✅ **Bağla — deterministic emitters wired into the ECS.** `IWorldEditor` emitter quartet
  (`create/has/set_has/params/set_params`) + effect enumerator; each emitter's fixed pool lives host-side
  on `RuntimeSimulation::Record` (Option A — off the ECS chunk, like cloth); `step_particle_emitters()`
  advances every playing pool on the fixed tick; `extract()` emits `RenderScene::particle_billboards`;
  a new `Render::ParticleBillboard` extract channel draws them (host-uploaded `GpuParticle` buffer);
  built-in Fire/Sparks/Smoke library; editor GameObject ▸ Particle Emitter + Add-Component + Inspector
  (effect/seed/playing) + `.sushiscene` persistence.
- ◐ **VFX2 — Transparency & lit particles.**
  - ✅ Per-blend bucketing (additive/premultiplied vs true-alpha) with two draws + a premultiplied "over"
    alpha pipeline.
  - ✅ GPU **bitonic back-to-front depth-sort** of the alpha bucket (`ParticleSortPass` +
    `particle_sort.comp` + `particle_sorted.vert`; gated on `has_alpha()`; pool capacity dropped to 2^16
    to keep the sort tractable).
  - ✅ **Sun-lit particles** — the alpha bucket receives the world-space directional sun + a flat ambient
    (camera-relative hazard sidestepped); the additive bucket stays emissive.
  - ✅ **Clustered-punctual-light particles** (VFX2c, the froxel version) — the lit bucket maps each
    sprite to a cluster and accumulates `punctual_attenuation(...) * NdotL * radiance` over that
    cluster's lights, plus `gi_sh_irradiance(sh, n)` ambient (the flat ambient became the SH ambient).
    Camera-relative via `centre − eye` + `gl_Position.w` view-depth; the froxel primitives extracted into
    the binding-free `clustered_lighting_common.glsl` shared with `pbr.frag`. See §7.7. **Implemented,
    build-pending + awaits a GPU visual check** (the camera-relative correctness is not visible at
    compile time).
  - ☐ Volumetric-shadow receive.
- ☐ **VFX3 — Ribbons / trails / beams.** Compute trail-geometry generator + ribbon draw.
- ☐ **VFX4 — Mesh particles.** Mesh-instance emission wired onto the existing GPU-driven indirect path
  (`GpuInstance`/`cull.comp`/`mesh_gpu.vert`).
- ☐ **VFX5 — GPU collision & force fields.** Depth/SDF collision; vortex, point/line attractors, drag
  volumes; upgraded curl-noise turbulence.
- ☐ **VFX6 — Editor polish.** Full timeline + node-graph authoring, effect library browser, live scrubbing,
  `.sushieffect` asset files.

---

## §13 Deferred / open

- **▶ RESUME HERE (2026-07-25).** VFX2c is code-complete but **not yet built or GPU-verified**. Next
  session, in order:
  1. **Build** — `se editor --no-run` (user builds; do not run ninja/cmake). Touched files:
     `render/shaders/clustered_lighting_common.glsl` (new include), `clustered_lighting.glsl`,
     `particle.frag` / `particle.vert` / `particle_sorted.vert`, `render/passes/particle_pass.{hpp,cpp}`,
     `render/rhi/vulkan/vulkan_scene_view.cpp`, `render/CMakeLists.txt` (added the new include to
     `SHADER_INCLUDES`). Fix any GLSL/C++ error before proceeding.
  2. **GPU visual check** — place a point/spot light beside a lit (true-alpha, e.g. Smoke) emitter and
     confirm the puff picks up the light and reads correctly; confirm no z-slice mismatch (the froxel
     view-depth now comes from `gl_Position.w`, which must equal the mesh path's
     `dot(cam_forward, v_world_position)`).
  3. **Only after it looks right on the GPU**, flip §12's VFX2c ✅ note from "build-pending" to shipped
     and move VFX2 from ◐ toward the next item (Volumetric-shadow receive, or VFX3 ribbons).
  Details of the implementation and the accepted float-precision limitation are below.
- **Clustered-punctual-light particles — IMPLEMENTED (2026-07-24), awaiting build + GPU visual check.**
  The lit bucket's draw pass took `lights_` + `ibl_` in its constructor and added `set 0` bindings 3–7
  (light buffer, cluster grid, light-index list, `ClusterBlock` UBO, IBL SH) — the "pass's own set"
  option, not the shared `SceneLayout`. The predicted binding collision was sidestepped not by *copying*
  the helpers but by *extracting* them: `clustered_lighting_common.glsl` now holds the binding-free
  `PunctualLight` / `cluster_index` / `punctual_attenuation` / grid `#define`s, included by both
  `clustered_lighting.glsl` (which keeps its 14–22 bindings) and `particle.frag` (which declares 3–7), so
  there is exactly one copy of the froxel math (§7.7). The camera-relative conversion was done in the
  **vertex** stage (`centre − eye`, eye packed into the push's spare `w` lanes) with the froxel view-depth
  taken from **`gl_Position.w`** rather than a `dot(cam_forward, camrel)` — no `cam_forward` in the push
  (keeps the block at the 128-byte house budget) and no cross-product sign trap. **Still to verify on a
  GPU:** that `gl_Position.w` matches the mesh path's `dot(cam_forward, v_world_position)` z-slice and that
  lit smoke reads correctly near punctual lights. **Accepted limitation:** the `centre − eye` subtraction
  is in float, so at planetary distances it inherits the cosmetic pool's float32 position precision — a
  near-camera envelope, the same envelope the pool's absolute-float positions already live in.
- **Known slice limitations to revisit.** Deterministic billboards draw additively (no per-billboard blend
  on `Render::ParticleBillboard`), so a deterministic Smoke emitter glows rather than composites; only
  `emitters[0]` of a multi-emitter effect is CPU-simulated on the deterministic path; the alpha sort runs
  over the whole 2^16 pool when any alpha emitter exists (optimize with a smaller sort budget later);
  particles write no velocity, so fast sparks ghost mildly under TAA (like clouds); the GPU pool's
  cross-frame write→read is gated by frame pacing, not fully decoupled from frames in flight; the Particle
  Editor panel recompiles the effect each frame it is open. None of these block the shipped features.
- **Async-compute sim** (VFX5+): overlap the sim with graphics; requires the §5.2 handoff buffers stay
  graph transients and the pools stay compute-only.
- **Post-TAA particle composite** into `targets.resolved` for ghost-free fast sparks — only if VFX2's
  velocity-less pre-TAA ghosting proves unacceptable.
- **GPU-side deterministic path** — not planned; determinism stays CPU-side by construction. If GPU
  determinism is ever required, it is a separate backend behind the same seam.
- **Interop** with SushiRuntime's SYCL graph for the sim (zero-copy USM) — tracked with the renderer's
  M2 interop milestone, not before.
