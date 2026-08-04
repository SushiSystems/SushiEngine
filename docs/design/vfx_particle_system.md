# VFX Particle System — authoring, two simulation backends, render integration (`SushiEngine::VFX`)

**Status:** in progress, VFX1 to VFX7 done as of 2026-07-25; beams and SDF collision open (§12).

This document is the **umbrella** for SushiEngine's AAA VFX particle system: the product vision (a
Niagara/VFX-Graph-class effect authoring pipeline feeding both a GPU-cosmetic simulator and a
CPU-deterministic simulator), the data model an artist authors, the two simulation backends behind a
single seam, the render-graph integration, and the editor authoring surface. The heavy per-particle
math lives in compute shaders (`render/shaders/particle_*.comp`) and in the deterministic CPU
integrator; this doc specifies the **architecture and the seams**, not the shader source.

Companion docs: `animation_system.md` (the structural template — trivially-copyable state column +
snapshot extract + a compute subsystem wired end to end), `audio_system.md` (the sibling wall-clock
snapshot consumer that also lives *outside* the deterministic island), and the renderer's
`docs/architecture/` (the render graph the draw passes plug into).

**VFX1 + the deterministic-ECS connection + all of VFX2 are implemented** (`se editor --no-run`
green each increment; the two newest steps await a GPU visual check). Done: the whole VFX1 vertical
slice (authoring model, both simulation backends, GPU emit/simulate/billboard render, blend-state
prerequisite, quality tiers, editor panel + emitter gizmo, demo, tests); the **deterministic emitter
→ ECS** connection (`IWorldEditor` emitter quartet, host-side pools on `RuntimeSimulation::Record`,
`ParticleBillboard` extract, editor GameObject/Add-Component/Inspector/`.sushiscene`); **VFX2a**
(per-blend bucketing → additive + true-alpha draws, and GPU **bitonic back-to-front depth-sort** of
the alpha bucket); **VFX2b sun-lit particles** (world-space directional sun + ambient on the alpha
bucket — the camera-relative hazard sidestepped); and **VFX2c clustered-punctual-light particles** —
the froxel version: the lit (true-alpha) bucket now maps each sprite to a cluster and shades it with
the scene's punctual lights + the environment SH ambient, in the same camera-relative space the
meshes use. The froxel primitives were refactored into a binding-free
`clustered_lighting_common.glsl` shared with `pbr.frag` (§7.7), and the camera-relative hazard was
met by carrying `centre − eye` down from the vertex stage and reading the view-depth from
`gl_Position.w`; the one accepted limitation is that this camera-relative subtraction is in float,
so it inherits the cosmetic pool's float32 position precision (a near-camera envelope, documented in
§13). VFX2c also brought **shadow receive**: the same carried centre projects into the sun's cascade
atlas through a cheap particle-specific four-tap sampler, so a puff standing in a mesh's shadow
darkens (§7.8). The roadmap in §12 marks each item's state.

---

## §0 The one decision that shapes everything — a hybrid, two-backend system

Particles in a deterministic networked engine face a fork: gameplay-affecting particles must be
**bit-reproducible** for rollback/replay (SushiLoop's determinism rules), but AAA visual scale
(millions of sparks, smoke, dust) is only reachable on the **GPU**, which is non-deterministic and
must stay *out* of the deterministic sim island. SushiEngine resolves this by shipping **both**,
behind one authoring model and one backend seam:

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

Every `ParticleEmitter` carries a `SimulationDomain` enum; the engine routes it to the matching
backend. The two backends are **interchangeable** behind `VFX::IParticleBackend` (LSP), and both
consume the **same** `CompiledEffect`, so an artist authors once and only picks a domain. This is
the direct consequence of the user's "hybrid" decision — the flexibility cost (two simulators) is
accepted for the reach it buys: real gameplay particles that survive netcode *and* GPU-scale
cosmetic spectacle.

**Invariant:** the cosmetic path reads emitter transforms from a snapshot and writes no sim state (a
run is byte-identical with cosmetic VFX on or off); the deterministic path's entire per-tick state
is a fixed-size, pointer-free, integer-seeded column, exactly like `Animation::AnimatorInstance`.

---

## §1 Why none exists today — audit

1. **Greenfield.** No `include/SushiEngine/vfx/`, no particle component, no particle pass, no
   shader.
2. **No blend state in the pipeline factory.** `Render::Resources::GraphicsPipelineDesc`
   (`render/resources/pipeline_cache.hpp`) and `fill_color_blend()` (`pipeline_cache.cpp`) hard-set
   `blendEnable = VK_FALSE`; additive/alpha billboards are impossible until the desc gains
   per-attachment blend fields **and** those fields join the byte-comparable fragment-output
   pipeline-library cache key. This is phase VFX1's first task (§7.5).
3. **No extract channel for emitters.** `Render::ISceneView::render(...)` (`scene_view.hpp:253`)
   takes POD instance arrays (`MeshInstance`, `SkinnedInstance`, `ClothStrandView`, lights, decals)
   but no emitter channel — a trailing `emitters`/`emitter_count` pair is added exactly like
   `skinned` was.
4. **No effect asset registry.** Animation has `AnimationDatabase`; VFX needs the sibling
   `VFX::EffectDatabase`.

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

**Skip list:** offline baking / simulation caches (real-time only for now); CPU-side sorting for the
GPU path (sorted on the GPU); network-replicated cosmetic particles (output-only, outside the
island).

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
  VFX::IParticleBackend seam (LSP): GpuParticleSystem | CpuDeterministicBackend
```

The renderer never sees an `EmitterDescriptor`, a curve, or a module — only a `CompiledEffect`
(POD + LUT bytes) and a per-frame `ParticleEmitterView` (transform + effect handle + domain + time).
This is the same DIP boundary the renderer already holds against animation (palette floats + a mesh,
never a clip). The editor never sees the SYCL runtime — it talks to engine headers and a thin
`IParticleEffectStore`.

---

## §4 Data model

### 4.1 Modules — a data-oriented, Open/Closed taxonomy

A module is **not** a virtual object (per-particle GPU code can't dispatch through a vtable, and
components must stay trivially copyable). A module is a **tagged POD descriptor**; the module *set*
is an enum, and behaviour lives in three places that a new module extends without touching existing
ones: (1) a descriptor struct, (2) an `EmitterCompiler` handler that bakes it into `CompiledEffect`,
(3) a branch in the sim shader / CPU integrator keyed on the compiled flag. Header:
`include/SushiEngine/vfx/modules.hpp`.

| Stage | Modules (VFX1 slice) | Later |
|---|---|---|
| **Spawn** | rate-over-time, burst-list | distance-based, event-triggered |
| **Shape** (initial position + emit direction) | point, sphere, hemisphere, cone, box, circle | mesh-surface, skeleton-socket |
| **Init** (per-particle at birth, min/max ranges) | lifetime, velocity, size, color, rotation, mass | initial mesh index, custom attribute |
| **Update** (per-tick) | velocity integrate, gravity, drag, curl-noise turbulence, size-over-life (curve), color-over-life (gradient), rotation-over-life, velocity-over-life, flipbook/sub-UV | vortex, point/line attractor, drag volume, GPU collision, orbital |
| **Render** | billboard, additive\|alpha\|premultiplied, soft depth-fade, texture, sort mode | stretched/velocity-aligned, mesh, ribbon/trail, lit, six-way lightmap |

Per-particle **attributes** (the working set both backends carry): `position`, `velocity`, `age`,
`lifetime`, `size`, `color` (rgb), `alpha`, `rotation`, `angular_velocity`, `seed`,
`flipbook_index`. GPU layout is float AoS (`GpuParticle`, §5.1); CPU layout mirrors it in
`Vector3f`/float.

### 4.2 Curves & gradients

- `include/SushiEngine/vfx/curve.hpp` — `AnimationCurve`: keyframed (time, value, in/out tangent),
  evaluatable on the CPU, and **bakeable** to a fixed-width `float` LUT (default 64 samples)
  uploaded as a 1-D texture / SSBO row the sim shader samples by normalized age.
- `include/SushiEngine/vfx/gradient.hpp` — `ColorGradient`: separate color keys + alpha keys,
  bakeable
  to an RGBA8 (or RGBA16F) LUT row. Color-over-life samples it by normalized age.

Both are authoring types (heap-backed key vectors); neither crosses into a component or the GPU —
only their **baked LUT bytes** do, inside `CompiledEffect`.

### 4.3 CompiledEffect — the POD boundary

`include/SushiEngine/vfx/compiled_emitter.hpp` — `EmitterCompiler` flattens one `EmitterDescriptor`
into a `CompiledEmitter` POD: packed spawn params, shape enum + params, init ranges, a bitfield of
enabled update modules + their scalar params, render flags (blend mode, sort, lit), texture/atlas
ids, capacity, and **offsets into a shared LUT atlas** for its baked curves/gradients.
`CompiledEffect` = a span of `CompiledEmitter` + the LUT atlas bytes. This is the single artifact
both backends and the GPU consume — the equivalent of `resolve_quality()` turning authored
`RenderSettings` into a POD `QualityParams`.

### 4.4 The asset & its registry

- `include/SushiEngine/vfx/particle_effect.hpp` — `ParticleEffect` =
  `std::vector<EmitterDescriptor>` + metadata (name, bounds, default domain). The artist's document;
  serialized as `.sushieffect` (JSON via `nlohmann_json`, the format the scene serializer already
  uses).
- `include/SushiEngine/vfx/effect_database.hpp` — `EffectDatabase`: `AssetId → ParticleEffect` +
  lazily-built `CompiledEffect`, mirroring `Animation::AnimationDatabase`. Owns compilation caching.

### 4.5 ECS component

`sim/components.hpp` gains `ParticleEmitter` (the central "one home so ids stay stable across TUs"
file):

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
`Frame::SceneDrawList` in `render/frame/frame_context.hpp`, copied in `VulkanSceneView::render`
exactly
where `skinned` is copied.

---

## §5 GPU-cosmetic backend

`render/scene/particle_system.cpp` — modeled feature-for-feature on `SkinningSystem`
(`Allocation`/`grow()`/`destroy()`, one buffer set per frame slot). It owns:

### 5.1 Persistent, system-owned pools (per emitter, ping-pong, grown on demand)

- `GpuParticle` state buffer (AoS, std430 float): `vec4 position_life; vec4 velocity_age;
  vec4 color_alpha; vec4 size_rot_seed_flip;` — 64 bytes, one `static_assert`-locked struct shared
  with the shader (the `GpuInstance` precedent).
- **dead-list** (free indices), **alive-list** ping-pong (current/next), and an atomic **counter
  buffer** (alive count + dead count + indirect-dispatch args). This is the standard GPU particle
  allocator.
- The **compiled-effect table + LUT atlas** uploaded once per change (change-gated like the
  atmosphere
  LUTs), not per frame.

These carry state frame-to-frame, so they are **system-owned VMA** and hand-barriered — they must
never be graph transients (transients alias and recycle each `begin_frame`).

### 5.2 Per-frame graph transients (the sim→draw handoff)

The **draw-instance buffer** (this frame's compacted alive particles the vertex shader pulls) and
the **indirect draw-args buffer** are declared as graph transients in
`render/rhi/vulkan/view_resources.cpp declare_targets()` beside `draw_commands`, usage
`STORAGE_BUFFER | INDIRECT_BUFFER`. `ParticleSimPass` declares `write(StorageWrite)`; `ParticlePass`
declares `read(instances, StorageRead)` + `read(args, IndirectRead)`. The graph then derives the
compute-write → indirect/vertex-read barrier automatically — cleaner and async-safe versus a hand
barrier, and the exact pattern cull→opaque already uses.

### 5.3 The compute pipeline (per emitter, per frame)

1. **Emit** (`particle_emit.comp`): read `spawn_count`, pop indices from the dead-list, initialize
   each new particle from the shape + init modules (seeded by `seed` + particle index), push to the
   alive-list.
2. **Simulate** (`particle_simulate.comp`): per alive particle apply the enabled update modules —
   forces (gravity/drag/curl-noise), integrate position/velocity, advance age, sample the size curve
   / color gradient LUTs by normalized age, advance flipbook. Retire (`age >= lifetime`) → push
   index back to the dead-list; survive → append to the compacted draw-instance buffer and
   `atomicAdd` the indirect vertex/instance count.

`ParticleSimPass : IRenderPass` mirrors `SkinningPass`: owns its compute pipeline
(`pipelines_.create_compute(layout, shaders_.module("particle_simulate.comp"))`), per-frame
descriptor sets from `frame.descriptors`, `vkCmdDispatch`, `rebuild_pipelines()` hot-reload hook.
Registered in `passes_` **immediately after `skinning_pass_`** (compute batch, before
`depth_prepass_`).

### 5.4 The draw pass

`ParticlePass : IRenderPass`, graphics, `vertex_stride = 0` — the vertex shader expands 6
verts/particle
from `gl_VertexIndex` and pulls the particle from the draw-instance storage buffer (the
fullscreen-pass
precedent, `cloud_composite_pass.cpp:105`). It:

- declares `color_attachment(0, frame.targets.scene_final, Load)` and **does not attach depth**;
- declares `read(frame.targets.depth, SampledFragment)` and samples it in the fragment shader for
  both the **soft depth-fade** and a **hard occlusion** test (compare billboard view-Z to scene
  depth). By the time this pass runs, depth is already `SHADER_READ_ONLY_OPTIMAL` (SSR sampled it),
  so no transition;
- blends additive / alpha / premultiplied via the new blend-state path (§7.5);
- issues `vkCmdDrawIndirect` against the transient args buffer.

Registered in `passes_` **between `ssr_pass_` and `taa_pass_`**, writing `scene_final`. Not before
SSR (would pollute reflections), not after TAA (would lose temporal AA + hit an extent mismatch).
Particles write no velocity in the slice; fast sparks ghost mildly under TAA the same way clouds do
— revisited in VFX2 if a post-TAA composite into `targets.resolved` becomes necessary.

### 5.5 Queue

VFX1 runs the sim on the **graphics queue** (no `set_queue(AsyncCompute)`), mirroring skinning/cull
— simplest-correct. Async compute is a later optimization; if taken, the §5.2 handoff buffers stay
graph transients so the graph both barriers and marks them concurrent, and the persistent pools are
touched only inside the compute pass.

---

## §6 CPU-deterministic backend

`CpuDeterministicBackend` runs as an **ECS system inside the fixed-step schedule** (the sim TU,
`sushi_sim`), never on wall clock. Its entire per-tick state is a **fixed-size, pointer-free
column** so a
tick is byte-snapshottable and bit-reproducible for rollback — the `AnimatorInstance` contract:

- particles live in a `constexpr`-capped per-emitter pool (e.g. `MAX_DETERMINISTIC_PARTICLES` per
  emitter); no heap, no `std::vector` growth mid-tick;
- RNG is **PCG32** seeded from `ParticleEmitter::seed` + a per-tick counter — no wall clock, no
  `Math::random`;
- integration is fixed-step Euler over the same `CompiledEmitter` params + baked LUTs the GPU path
  uses (curves evaluated by sampling the same LUT so both backends agree visually);
- spawn/kill are deterministic functions of `(seed, tick, accumulator)`.

Extract reads the live pool's positions/sizes/colors into `ParticleEmitterView`-adjacent instance
data and renders them through the **same** billboard `ParticlePass` (a host-filled instance buffer
path rather than the GPU compute path). Gameplay may query the pool (e.g. "is this cell on fire")
because it is authoritative sim state.

The determinism test (`Integration_ParticleDeterminism`) asserts (a) two independent runs of the
same seed + tick stream produce byte-identical pool state, and (b) a snapshot → advance → restore →
replay reproduces the captured state byte-for-byte (`std::memcmp == 0`) — exactly as
`animator_demo.cpp` proves for `AnimatorInstance`.

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

Extend `Render::Resources::GraphicsPipelineDesc` with per-attachment blend (`bool blend_enable`,
src/dst color & alpha factors, color & alpha ops), teach `fill_color_blend()` to honor them, and
**fold the new fields into the byte-comparable fragment-output pipeline-library cache key** so two
pipelines that differ only in blend don't collide. Defaults reproduce today's opaque behaviour, so
every existing pass is untouched. Without this, "transparent" particles draw opaque.

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
  binding-free `render/shaders/clustered_lighting_common.glsl`. `clustered_lighting.glsl` now
  includes it and keeps only its own `set 0` bindings 14–22 and the shading that reads them (its
  `cluster_index_for` became a one-line wrapper, behaviour-identical for `pbr.frag`);
  `particle.frag` includes the same header and declares a **smaller subset** of those buffers on the
  pass's own set (bindings 3–7), so the two consumers share the math but bind on different sets.
  This is the header's Open/Closed seam — a new consumer adds a binding block, not another copy of
  `cluster_index`.
- **Camera-relative without a precision cliff or a sign trap.** Particles are stored in
  absolute-world float, but the froxels and `light_buffer` are camera-relative. The vertex stage
  subtracts the eye (`centre − eye`, the eye packed into the push's spare `w` lanes exactly as
  `scene_uniforms` packs it) and passes the camera-relative centre down as a varying; the froxel
  **view-depth is read straight from `gl_Position.w`** (the perspective clip `w` equals the positive
  view distance the z-slice is keyed on), which needs no `cam_forward` vector and so has no
  cross-product sign ambiguity. The one accepted limitation: the `centre − eye` subtraction is in
  float, so it inherits the cosmetic pool's float32 position precision — a near-camera envelope (see
  §13).
- **Within the 128-byte push budget.** No new push space was spent: the eye rides `camera_right.w`,
  `camera_up.w`, `sun_direction.w`, and the lit flag doubles as the SH-ambient scale on
  `sun_radiance.w` (`<0` = unlit/emissive, `>=0` = lit with that IBL ambient scale), so the block
  stays exactly 128 bytes, matching `MeshPushConstants` — the house budget that keeps the layout
  portable to a 128-byte device.

The pass reads `frame.targets.cluster_grid` / `light_index` (so the graph derives the
compute→fragment
barrier), and binds `lights_.light_buffer()`, `lights_.config_buffer()` (a truncated,
layout-compatible
`ClusterBlock`), and `ibl_.sh_buffer()` on every bucket — the unlit fragment path never samples
them.

### 7.8 Shadow receive for the lit bucket (VFX2c)

The sun term is multiplied by the sun's cascade visibility, so a puff standing in the shadow of
geometry darkens. This is the cheap half of "volumetric shadowing": the medium *receives* shadow, it
does not yet *cast* one (self-shadowing a particle field needs its own light-space march — §13).

- **A particle-specific sampler, not the mesh path's.** `render/shaders/particle_shadow.glsl`
  exposes `particle_sun_shadow(atlas, position, view_depth, rotation)`: cascade select, one
  projection, a four-tap fixed-radius Vogel disc through the comparison sampler, and the mesh path's
  last-cascade fade. It deliberately skips the two most expensive parts of `sample_sun_shadow`. The
  blocker search (PCSS) is dropped because a transparency pass pays every tap once per **overdrawn
  sprite layer**, not once per pixel; the normal offset is dropped because a puff has no surface
  normal to offset along, so the whole bias budget goes into the depth bias — over-biasing a
  floating point sample only shifts where its shadow begins and can never print acne across a face.
  The result is soft and chunky, which is what a volumetric medium wants.
- **No new space anywhere.** The cascade block and atlas bind on the pass's own set at the **scene
  set's own numbers** (10 and 11, unused here), so `shadow_common.glsl`'s
  `layout(set = 0, binding = 10)` declaration is reused verbatim instead of copied. No push-constant
  lane was spent: the shadow projection reuses the camera-relative centre §7.7 already carries (the
  cascade matrices are fitted around the eye, the same space).
- **One copy of the cascade math.** The sampler-free helpers — `vogel_disc`,
  `select_shadow_cascade`, `shadow_tile_origin`, `shadow_tile_clamp`, `shadow_atlas_texel` — moved
  from `shadow_sampling.glsl` into `shadow_common.glsl`, which is exactly the split that file's own
  header describes ("the block and the arithmetic every stage can do"). `shadow_sampling.glsl` keeps
  only what needs a sampler, so the particle path does not drag in `temporal_common.glsl` (and its
  binding-9 UBO) to reach a Vogel disc. The tap rotation is therefore the frame-static
  `interleaved_gradient_noise` rather than `temporal_dither` — a still pattern on a soft sprite
  beats an unresolved animated one.

The pass reads `frame.targets.shadow` and `frame.targets.shadow_atlas`, so the graph orders the
atlas
render before the draw, and binds the atlas through `ShadowPass::atlas_sampler`.

### 7.9 Render alignment: velocity-stretched sprites (VFX3a)

`RenderAlignment::VelocityStretched` was in the authoring model from VFX1 with nothing behind it —
an authored intent the renderer silently dropped. It now works, and the way it was added is the
pattern the remaining alignment modes (ribbon, mesh) should follow.

- **One expansion, three draws.**
  `particle_quad_offset(p, corner, camera_right, camera_up, alignment,
  velocity_stretch)` in `particle_common.glsl` is now the only place a billboard corner is placed.
  The additive draw, the depth-sorted alpha draw, and the deterministic-billboard draw all call it,
  where before the first two carried copies of the same four lines. A new mode is a branch in one
  function.
- **The stretch.** The quad's long axis is the particle's velocity projected onto the camera plane
  and its half-length is `size + speed * velocity_stretch` (the authored
  `RenderModule::velocity_stretch`, streak metres per m/s), the short axis stays `size`. The
  particle's roll is dropped in this mode — the velocity already fixes the orientation, and spinning
  a streak would only wobble it. Two degenerate cases fall back to camera-facing, which is exactly
  what a zero-length streak is: a particle barely moving, and one flying straight at the eye (null
  screen-projected velocity, so the stretch direction would be undefined).
- **Where alignment lives, and the binding it forced.** Alignment is per-**emitter**; the draw list
  is per-**particle**. Rather than widen the full 80-byte `GpuParticle`, the two GPU draws bind this
  frame's emitter table to the **vertex** stage (binding 12) and index it by `emitter_index` —
  `GpuEmitter`'s spare `pad0`/`pad_a` lanes became `alignment`/`velocity_stretch`, so the record's
  size is unchanged. Deterministic billboards cannot do this: they belong to no GPU emitter, index 0
  of the table is an unrelated cosmetic emitter, and on a billboard-only frame there is no table at
  all. They got `particle_billboard.vert` and a pipeline of their own instead of a flag on the
  shared shader — the two differ in what they are permitted to *read*, which is a pipeline property,
  not a uniform. Consequence: **velocity stretch is a cosmetic-path feature**; the deterministic
  path stays camera-facing (`Render::ParticleBillboard` carries no velocity to stretch along).

### 7.10 Ribbons and trails (VFX3b)

`RenderAlignment::Ribbon` draws a particle as a strip through its own recent positions.

- **Trail history is persistent state, so it lives with the pool.** `ParticleSystem` owns a second
  device-local buffer of `capacity * TRAIL_POINTS` `vec4`s (xyz sample position, w its size),
  zero-cleared alongside the pool on the same one-time path. `TRAIL_POINTS` is 8, mirrored between
  `ParticleSystem` and `particle_common.glsl`.
- **Shifted, not a ring.** Each simulate step moves every sample one place down and writes the new
  position at index 0. A ring would save seven `vec4` moves but would put a head index in the vertex
  stage's way, and the 128-byte draw push has no room for one — the moves are nothing beside the
  curl-noise evaluation already in that loop. The emit pass collapses the whole history onto the
  birth point, because a recycled slot still holds the previous occupant's trail and would otherwise
  draw a streak from wherever that particle died.
- **A third bucket, keyed on geometry rather than blend.** A ribbon expands into a whole strip while
  a sprite expands into one quad — a different vertex count per instance, which is a property of the
  draw and cannot be a branch inside a shader. So the compute compaction routes ribbons to
  `particle_ribbon` with its own `VkDrawIndirectCommand` (`particle_args` grew to three, ribbons at
  offset 32) and `particle_ribbon.vert` draws `(TRAIL_POINTS - 1) * 6` vertices per instance.
- **Orientation and taper.** Each sample's side offset is perpendicular to both the local trail
  direction (taken from the *neighbouring* samples, so a corner shared by two segments resolves
  identically from both and the band has no crease) and the eye ray, so the strip always presents
  its width to the camera and twists with the path. Width and alpha taper to zero at the tail; the
  width comes from the sample's own recorded size, so a size-over-life curve is already baked into
  the strip's profile. Two degeneracies fall back to the camera's right axis: a trail that has not
  moved yet, and one running straight at the eye.
- **No ribbon fragment shader.** The shared fragment's radial falloff expects a round sprite; a
  ribbon wants a soft edge across the band and none along it. Emitting `v = 0.5` collapses the
  radial term to `1 - u²`, which is exactly that, so the strip reuses `particle.frag` unchanged.
- **Slice limits.** The ribbon bucket composites with the premultiplied "over" and shades
  **emissive**, whatever the emitter authored. Both follow from the bucket being keyed on geometry
  and therefore mixing blends: an unlit trail can only be too bright, while a lit one could come out
  black. Splitting the ribbon bucket by blend is the one increment that fixes both. Ribbons are also
  not depth-sorted (the sort pass keys the alpha sprite list only).

### 7.11 Mesh particles (VFX4)

`RenderAlignment::Mesh` draws each particle as a solid mesh instance — debris, shells, rocks.

- **The one particle path that is not transparent, so it does not draw with the others.**
  `ParticleMeshPass` runs immediately after the opaque pass, loading the scene's HDR colour and
  depth, and depth-**tests and writes** like any other solid surface. Drawing debris in the
  transparency pass would leave overlapping pieces compositing in list order, which no blend mode
  fixes. This is why mesh particles are a pass rather than a fourth bucket in `ParticlePass`.
- **One draw per mesh, so the list is sliced.** A draw binds one mesh, so mesh particles cannot
  share one indirect command the way sprites do. Up to `MAX_MESH_EMITTERS` (4) mesh-aligned emitters
  each claim an equal slice of a shared list and one `VkDrawIndexedIndirectCommand`; the emitter
  carries its slice index in `GpuEmitter::mesh_slot` and the compaction bumps that command's
  instance count. An emitter past the last slice draws **nothing** — sharing a slice would render
  its particles as another emitter's mesh, which is worse than dropping them.
- **Who knows what.** The index count is a host fact (which mesh the emitter authored) and the
  instance count is a GPU fact, so the sim pass seeds the whole command from `ParticleSystem`'s
  mesh-draw table — which is why `prepare` now takes the `MeshRegistry`. The reverse also matters:
  the instance count is a GPU atomic the host cannot clamp, so the **vertex shader** clamps its
  index to the slice; an overflowing instance redraws the slice's last particle instead of reading
  into the next emitter's slice.
- **Placement, not material.** The mesh supplies the geometry; the particle supplies position, a
  uniform scale from its size, a tumble about its direction of travel (Rodrigues about the velocity
  axis, angle from the roll the update step already integrates), and a colour tint over the mesh's
  own vertex colour. Faces are not culled: debris turns, so a back face is as likely to be the
  visible one.
- **Shading.** Not `pbr.frag`: a mesh particle carries no material, and reaching the bindless
  material heap would tie this pass to the whole scene set. It is a diffuse surface lit by the sun —
  shadowed through the same `particle_shadow.glsl` sampler the lit sprites use, its second consumer
  — plus a flat ambient. **A mesh particle that needs a real material is a mesh instance, not a
  particle**, and belongs on the GPU-driven instance path.
- **Slice limits.** The pass writes colour and depth only, not the id, velocity, or gbuffer targets
  the opaque pass fills, so mesh particles are not pickable, contribute no motion vectors (mild TAA
  ghosting on fast debris), and are absent from SSR and GTAO. They are also outside the GPU cull.

### 7.12 Force fields (VFX5a)

Gravity, drag, and turbulence act everywhere alike. A **force field** has a place — a centre, a
radius it reaches, and a falloff over that reach — which is what makes it the tool for *authored*
motion: an updraft over a fire, a swirl in a portal, a pocket of still air behind cover.

- **Three kinds, one evaluation.** `Point` pulls toward (or, with a negative strength, pushes from)
  its centre; `Vortex` swirls about an axis through it; `Drag` damps velocity inside it. Point and
  Vortex contribute acceleration; Drag returns a factor applied to velocity after integration,
  alongside the emitter's own drag.
- **Weight, not inverse-square.** A field's weight is `pow(1 - distance/radius, falloff)`: one at
  the centre, zero at the rim. Nothing outside the radius is touched — so a field is genuinely local
  and costs nothing to particles elsewhere — and nothing spikes to infinity at the centre, which a
  raw inverse-square does and which no amount of clamping makes look intentional.
- **Fixed count, because the boundary is a POD.** `MAX_FORCE_FIELDS` is 4. `CompiledEmitter` is the
  byte-comparable record both backends and the GPU read, so a variable-length list cannot live in
  it; the compiler takes the first four *enabled* entries and drops the rest.
- **Authored local, evaluated world.** Fields are authored in the emitter's frame so they travel
  with it. Each backend places them into world space once per step — the render system bakes the
  emitter matrix into `GpuEmitter` when it flattens the frame, and the deterministic backend applies
  the emitter pose at the top of `integrate` — so neither evaluates a transform per particle.
- **Both backends, one meaning.** The GLSL `particle_force_fields` and the deterministic backend's
  field loop are line-for-line counterparts, as `curl_noise` already is across the two. They read
  the same authored record, so they have to agree on what it means; the two are not required to be
  bit-identical to each other (they are different domains — see §2), only each deterministic in
  itself.

### 7.13 Depth collision (VFX5b)

Cosmetic particles bounce off whatever the camera can see, tested against the depth the renderer has
already produced — no collision geometry, no broadphase.

- **The pass-order problem, and why there was never one.** `ParticleSimPass` runs *before*
  `depth_prepass`, so this frame's depth does not exist when the particles move. But `HizPass` owns
  a **persistent** image rather than a graph transient, so at sim time its level 0 still holds *last
  frame's* linearised depth — `near / depth`, which is exactly the linear view distance the test
  needs. One frame of lag on a spark's bounce is invisible; needing a depth buffer that does not
  exist yet would not have been. This is the same cross-frame read `cull_pass` already makes of the
  occlusion pyramid.
- **Knowing when there is nothing to read.** `HizPass::has_history()` is false before the first
  build and while the pass is switched off (it follows SSR), because an image that has never been
  written holds garbage and has never left `UNDEFINED`. The sim pass then binds a 1×1 stand-in and
  clears the collision bit in its push constant — a combined-image-sampler binding needs a real view
  even on a frame the shader will not read it.
- **The test.** Project the particle, read the pyramid at its pixel, and compare view depths. A
  particle in front of the surface is free. One further behind it than the authored *thickness* is
  taken to be past the object rather than resting on it and is also let through — the depth buffer
  records a surface, not a solid. Between the two is contact.
- **The normal comes from the depth gradient.** Two neighbouring taps are turned back into
  camera-relative positions (a pixel's ray, scaled so its forward component is one, times the linear
  depth) and crossed. At a silhouette the neighbours are on a different surface entirely, so a
  gradient there would point nowhere real; a depth jump larger than four thicknesses falls back to a
  camera-facing normal, which is the safe answer at an edge.
- **The response** splits velocity into normal and tangential parts, keeps `restitution` of the
  normal and sheds `friction` of the tangential, and lifts the particle back out along the normal by
  the penetration so the next step starts in contact rather than deeper in.
- **What it cannot do, by construction.** Particles off screen, behind the viewer, or hidden behind
  something nearer pass straight through, and a thin wall is only as solid as the thickness allows.
  That is the right trade for sparks skittering off a floor in view and the wrong tool for gameplay,
  which is why the **deterministic backend has no counterpart** — collision here is a cosmetic-path
  feature and the authoring UI says so.
- **Cost.** The push block reached exactly its 128-byte budget: a view-projection, the camera basis
  with the two half-fov tangents in the spare `w` lanes, and two vec4s of counts. The emit dispatch
  shares the layout and reads only the emitter index from it.

---

## §8 Editor authoring

Modeled on the animation subsystem's seams, but promoted to real panels (not the inlined-in-main
shortcut the skeleton preview used).

- `editor/vfx/effect_preview.{hpp,cpp}` — an `EffectPreview` state class (owns the previewed
  `ParticleEffect`, its `EffectDatabase` handle, a preview world `Mat4`, playback clock) + a
  free-function viewport overlay
  `draw_emitter_gizmo(preview, camera_view, image_origin, w, h, draw_list)` that paints the emitter
  shape (sphere/cone/box wireframe) and bounds, mirroring `animation/skeleton_debug_draw`'s
  `draw_skeleton_overlay`.
- `draw_particle_editor_panel(EditorContext&)` in `editor/ui/editor_panels.cpp` — the module-stack
  authoring UI (emitter list, per-stage module add/remove/reorder, module params), a **curve
  editor** and a **gradient editor** widget (ImGui draw-list based), a preview toolbar
  (play/pause/restart, domain toggle). Registered at the three edit points (`PanelVisibility` bool,
  Window-menu `MenuItem`, the `draw_*_panel` call block in `main.cpp`) + a `DockBuilderDockWindow`
  placement.
- Emitter gizmo threads into the Scene `ViewportPanel::draw` overlay region and the existing gizmo
  drag + `history.begin_change/end_change` undo bracket (an emitter is an entity with a
  `ParticleEmitter` component; its transform moves through `IWorldEditor` like any other).
- `editor/CMakeLists.txt` gains `vfx/effect_preview.cpp`; panel functions need no CMake change.

### 8.1 Effect assets, library, and timeline (VFX6)

- **`.sushieffect` files (`editor/serialization/effect_serializer.{hpp,cpp}`).** JSON in the same
  shape and spirit as `.sushiscene`, one object per emitter with a sub-object per module.
  Deliberately the **descriptor** tree, never the compiled record: the compiled form is a build
  product, so persisting it would tie a saved asset to a layout that changes with every phase.
  Curves and gradients are written as their authored key lists rather than baked LUTs, for the same
  reason. Every missing key falls back to the module's default, so a file written by an older build
  loads with the newer defaults instead of failing — and an enum out of range falls back too, since
  a number from a newer build must never become an out-of-range enum here.
- **Library browser.** A "Library" section on the particle panel lists
  `assets/effects/*.sushieffect` and loads one on click, with name/Save/Refresh. The listing is
  re-read on demand rather than watched: an author saves far less often than the panel redraws, and
  a filesystem scan per frame would be the panel's dominant cost for information that almost never
  changes. There is no delete — removing an asset is the file manager's job, not a button one click
  from a scrub bar.
- **Timeline and scrubbing.** The emitter's cycle drawn as a bar with its burst times marked and a
  draggable play head, calling `EffectPreview::seek`. What that seek *means* depends on which
  backend is previewing, and the difference is the honest one:
  - **GPU preview** — it moves the **emission schedule** only. The rate and burst evaluation jump to
    the new time and the fractional-particle accumulators are cleared (carrying that debt across a
    discontinuity would spill it as a burst at the new time), but the particles already alive do not
    wind back and cannot: the cosmetic pool lives on the GPU and advances one step per rendered
    frame, so there is no host copy to rewind. The question it answers is "what does this emitter
    *do* at this point in its cycle", which is an authoring question worth answering.
  - **Deterministic preview** ("CPU (scrubbable)") — a **true scrub**. `CpuDeterministicBackend` is
    a pure function of (state, emitter, dt), so seeking resets the pools and replays from zero at
    the fixed step, reproducing exactly the frame that time would have shown. A Step button advances
    one tick. The replay is capped at 4096 steps so dragging to the end of a long cycle cannot stall
    the editor; past that the scrub is approximate rather than unresponsive.
  - The preview steps **every** emitter in this mode, whatever domain it declares. The domain says
    which backend *ships* the effect; the preview's job is to show the author what the same asset
    looks like through the other one. Its particles come out as `Render::ParticleBillboard`s — the
    same channel the sim's own deterministic emitters use — and the viewport concatenates them with
    the sim's rather than adding a channel.
- **Node-graph view.** The module stack drawn left-to-right as nodes with links, clicking one to
  toggle it. Deliberately a **presentation** of the authoring model and not a second one: the stack
  order (spawn → shape → init → update → render) is the pipeline a particle actually goes through,
  so the graph is that pipeline laid out, not a free-form canvas whose edges would have to be
  validated back into the same fixed order. Numbers stay in the sections below, where they are
  editable side by side.

---

## §9 SOLID

- **SRP** — authoring model (`vfx/`), compilation (`EmitterCompiler`), GPU resources
  (`ParticleSystem`), compute (`ParticleSimPass`), draw (`ParticlePass`), deterministic sim
  (`CpuDeterministicBackend`), and editor UI are each one responsibility in one place.
- **OCP** — a new module = one descriptor + one compiler handler + one shader/integrator branch
  keyed on a compiled flag; no existing module, pass, or backend is edited. A new render style
  (mesh, ribbon) = a new `RenderModule` variant + a draw path, gated by a flag. A new backend = a
  new `IParticleBackend`.
- **LSP** — `GpuParticleSystem` and `CpuDeterministicBackend` are substitutable behind
  `IParticleBackend`; the caller picks by `SimulationDomain` and treats them identically.
- **ISP** — the editor depends on a thin `IParticleEffectStore` (load/save/list effects), not on the
  runtime; the renderer depends on `ParticleEmitterView` + opaque `CompiledEmitter` bytes, not on
  authoring types; the extract POD is separate from the authoring model.
- **DIP** — high-level policy (what an effect *is*) is authoring-side; low-level mechanism (how it
  simulates/draws) depends on the POD `CompiledEffect` abstraction, never the reverse. The renderer
  and the editor both depend on interfaces/PODs, not concretions.

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

- `Unit_ParticleCurve` / `Unit_ParticleGradient` — keyframe evaluation vs analytic reference; LUT
  bake round-trips within tolerance.
- `Unit_EmitterCompiler` — an authored `EmitterDescriptor` compiles to the expected
  `CompiledEmitter`
  flags/params/LUT offsets.
- `Unit_ParticleSpawn` — rate + burst produce the expected counts over N ticks; pool never exceeds
  capacity; dead/alive accounting balances.
- `Integration_ParticleDeterminism` — the two byte-exact checks in §6 (independent-run equality +
  rollback replay), following `test_input_determinism.cpp`'s two-session structure.
- `examples/particle_demo.cpp` — a headless self-checking demo (GPU path where a device is present;
  CPU path always) returning 0/1, registered via `add_sushi_sycl_executable`.

No mocks — everything runs against the real runtime, per house style.

---

## §12 Roadmap (✅ done · ◐ partial · ☐ to do — status 2026-07-24)

- ✅ **VFX1 — Vertical slice.** Blend-state prerequisite; the `vfx/` authoring model +
  curves/gradients + compiler; `ParticleEmitter` component; extract seam; `GpuParticleSystem` +
  emit/simulate/billboard passes + shaders; `CpuDeterministicBackend`; quality gating; editor
  panel + emitter gizmo; demo + tests + docs. **Reached: authored emitters render as GPU particles
  in the viewport; the deterministic path passes the rollback test.**
- ✅ **Bağla — deterministic emitters wired into the ECS.** `IWorldEditor` emitter quartet
  (`create/has/set_has/params/set_params`) + effect enumerator; each emitter's fixed pool lives
  host-side on `RuntimeSimulation::Record` (Option A — off the ECS chunk, like cloth);
  `step_particle_emitters()` advances every playing pool on the fixed tick; `extract()` emits
  `RenderScene::particle_billboards`; a new `Render::ParticleBillboard` extract channel draws them
  (host-uploaded `GpuParticle` buffer); built-in Fire/Sparks/Smoke library; editor GameObject ▸
  Particle Emitter + Add-Component + Inspector (effect/seed/playing) + `.sushiscene` persistence.
- ◐ **VFX2 — Transparency & lit particles.**
  - ✅ Per-blend bucketing (additive/premultiplied vs true-alpha) with two draws + a premultiplied
    "over" alpha pipeline.
  - ✅ GPU **bitonic back-to-front depth-sort** of the alpha bucket (`ParticleSortPass` +
    `particle_sort.comp` + `particle_sorted.vert`; gated on `has_alpha()`; pool capacity dropped to
    2^16 to keep the sort tractable).
  - ✅ **Sun-lit particles** — the alpha bucket receives the world-space directional sun + a flat
    ambient (camera-relative hazard sidestepped); the additive bucket stays emissive.
  - ✅ **Clustered-punctual-light particles** (VFX2c, the froxel version) — the lit bucket maps each
    sprite to a cluster and accumulates `punctual_attenuation(...) * NdotL * radiance` over that
    cluster's lights, plus `gi_sh_irradiance(sh, n)` ambient (the flat ambient became the SH
    ambient). Camera-relative via `centre − eye` + `gl_Position.w` view-depth; the froxel primitives
    extracted into the binding-free `clustered_lighting_common.glsl` shared with `pbr.frag`. See
    §7.7. **Built green; awaits a GPU visual check** (the camera-relative correctness is not visible
    at compile time).
  - ✅ **Volumetric-shadow receive** — the lit bucket's sun term is multiplied by the sun's cascade
    visibility (`particle_shadow.glsl`: four-tap Vogel, no blocker search, no normal offset, bound
    at the scene set's own binding numbers on the pass's own set). See §7.8. Built green; **awaits
    the same GPU visual check** as VFX2c above. Casting (a particle field shadowing itself and the
    world) is not in this increment — see §13.
- ✅ **VFX3 — Streaks, ribbons, trails.**
  - ✅ **VFX3a velocity-stretched sprites** — `RenderAlignment::VelocityStretched` (authored since
    VFX1, unimplemented until now) plus a new authored `velocity_stretch` scale; the quad expansion
    was factored into one shared `particle_quad_offset`, and the deterministic billboards moved to a
    pipeline of their own so the GPU draws can read the emitter table. See §7.9. Built green.
  - ✅ **VFX3b ribbons / trails** — `RenderAlignment::Ribbon`: a persistent per-slot trail history
    written by the simulate step, a third indirect draw bucket, and `particle_ribbon.vert` expanding
    each particle into a tapered strip. See §7.10. Built green. Limits: the ribbon bucket is
    emissive, "over"-blended, and unsorted whatever the emitter authored.
  - ☐ **Beams.** A ribbon between two authored endpoints rather than through a particle's own
    history; the strip expansion is already there, only the sample source differs.
- ✅ **VFX4 — Mesh particles.** `RenderAlignment::Mesh` + a `ParticleMeshPass` drawing solid,
  depth-tested instances with the opaque geometry, one indexed indirect draw per mesh-aligned
  emitter. See §7.11. Built green. **Not** wired onto the GPU-driven instance path as originally
  sketched: `InstanceSystem` builds its records host-side each frame, and a GPU-simulated particle's
  transform never reaches the host, so a pass of its own was the only honest route.
- ✅ **VFX5 — GPU collision & force fields.**
  - ✅ **VFX5a force fields** — placed Point / Vortex / Drag fields, up to four per emitter, honoured
    by **both** backends. See §7.12. Built green.
  - ✅ **VFX5b depth collision** — a `CollisionModule` bounces cosmetic particles off last frame's
    Hi-Z pyramid, which is still readable at sim time because the pyramid is pass-owned rather than
    a graph transient. See §7.13. Built green. Cosmetic path only, by construction.
  - ☐ **SDF collision** against the existing `sdf_clipmap`, for particles that must collide off
    screen.
- ✅ **VFX6 — Editor polish.**
  - ✅ **`.sushieffect` asset files** — a descriptor-tree JSON serializer with defaulting reads.
  - ✅ **Effect library browser** — list, load, and save into `assets/effects`.
  - ✅ **Timeline + scrubbing** — the emitter cycle with burst markers and a draggable head.
  - ✅ **Deterministic-backed scrubbing** — a CPU preview mode whose scrub is exact, because the
    backend is a pure function of (state, emitter, dt) and can be replayed from zero.
  - ✅ **Node-graph view** — the module stack laid out as the pipeline it is, with click-to-toggle.
- ✅ **VFX7 — The particle material.** The four render-module fields that had been authored,
  compiled, and ignored since VFX1 now do what they say: the sprite **texture** (through the same
  bindless heap `pbr.frag` samples, at set 1, with a `RENDER_TEXTURED` flag rather than a sentinel
  index), its **flipbook** cell (chosen by the sim since VFX1, read by nobody until now), **soft
  particles** (the hard depth discard now followed by a fade over the authored distance), and
  **lit** — moved from the draw *bucket*, which mixes emitters, onto the emitter itself, which also
  lifts the VFX3b limitation that a ribbon could not be lit. Textures persist by **path**, not by
  the session-scoped handle. Editor: a **Material** section on the Particle System component. Built
  green; the shaders compile to SPIR-V at build time, so the GLSL is validated. Awaits a GPU visual
  check (§13). Deliberately sprite-only — a particle that needs a real surface is a mesh particle.

---

## §13 Deferred / open

- **▶ RESUME HERE (2026-07-25).** VFX2c (clustered lights §7.7 **and** shadow receive §7.8) is
  code-complete and **builds green** (`se editor --no-run`). What remains is the **GPU visual
  check** — neither the camera-relative correctness nor the shadow projection is visible at compile
  time. In a Smoke (true-alpha) emitter's viewport:
  1. Place a point/spot light beside the puff — it should pick the light up, and the falloff should
     track as the light moves. Dolly the camera in and out: no banding or popping as the sprite
     crosses froxel z-slices (the view-depth now comes from `gl_Position.w`, which must equal the
     mesh path's `dot(cam_forward, v_world_position)`).
  2. Put the puff in the shadow of a mesh with the sun on — it should darken, with a soft edge, and
     the shadow should track the sun. Watch for a shadow that is *offset* from the caster (a sign
     the cascade projection is not getting camera-relative input) and for the puff going dark
     everywhere (an over-large bias or an inverted reference).
  3. Also flip an emitter to **Velocity Stretched** in the particle editor (VFX3a, §7.9) and raise
     the Stretch slider — fast particles should draw as streaks aimed along their motion, and slow
     ones should stay round.
  4. **Only after all three read right**, drop the "awaits a GPU visual check" notes from §12 and
     move VFX2 from ◐ to ✅ — VFX3b (ribbons) is then the next increment.
- **▶ Particle material (VFX7) — the GPU visual check.** Code-complete and built green, shaders
  compiled to SPIR-V, but none of it is visible at compile time. On a Fire or Smoke emitter's
  Particle System component, under **Material**:
  1. Point **Texture** at any PNG and press Load. The round dot should be replaced by the sheet, and
     the sheet's own alpha should shape it — a square, hard-edged sprite means the texture's alpha
     is opaque, not that the falloff is still being applied on top.
  2. Set **Flipbook** to the sheet's grid (columns, rows). Cells should advance over each particle's
     life and swap cleanly; a smeared or half-blended cell means the cell index is being
     interpolated.
  3. Turn **Soft Particles** on and push an emitter into the floor. The hard intersection line
     should dissolve over the Fade distance. Watch for particles disappearing entirely (a near-plane
     mismatch between `cluster.depth.x` and the depth buffer's projection) or for no change at all
     (the emitter's `soft_particles` not reaching `RENDER_SOFT`).
  4. Tick **Lit** on an **Additive** emitter and on a **Ribbon** one — both were unlit by
     construction before, so this is the case the old bucket-keyed path could not express. Move a
     point light past them; they should pick it up. Then untick it on the Smoke emitter, which used
     to be lit whether or not its author asked: it should go emissive.
  5. Save the effect, reload it, and reopen the scene. The texture should come back — that is the
     path-not-handle round-trip working. A particle that comes back as a plain white square means
     the path resolved to nothing and `heap_index` fell through to the White default.
- **Particles cast no shadow.** The lit bucket receives the sun's cascade shadow (§7.8) but nothing
  in the scene is darkened *by* a particle: the sprites are never drawn into the cascade atlas, and
  a smoke column self-shadows only through its own alpha blend. Doing it properly is a light-space
  density march (the volumetric-fog froxel volume is the natural place to inject into), which
  belongs with VFX5's GPU collision/force-field work rather than with a transparency pass.
- **Clustered-punctual-light particles — IMPLEMENTED (2026-07-24), builds green, awaiting a GPU
  visual check.** The lit bucket's draw pass took `lights_` + `ibl_` in its constructor and added
  `set 0` bindings 3–7 (light buffer, cluster grid, light-index list, `ClusterBlock` UBO, IBL SH) —
  the "pass's own set" option, not the shared `SceneLayout`. The predicted binding collision was
  sidestepped not by *copying* the helpers but by *extracting* them:
  `clustered_lighting_common.glsl` now holds the binding-free `PunctualLight` / `cluster_index` /
  `punctual_attenuation` / grid `#define`s, included by both `clustered_lighting.glsl` (which keeps
  its 14–22 bindings) and `particle.frag` (which declares 3–7), so there is exactly one copy of the
  froxel math (§7.7). The camera-relative conversion was done in the **vertex** stage
  (`centre − eye`, eye packed into the push's spare `w` lanes) with the froxel view-depth taken from
  **`gl_Position.w`** rather than a `dot(cam_forward, camrel)` — no `cam_forward` in the push (keeps
  the block at the 128-byte house budget) and no cross-product sign trap. **Still to verify on a
  GPU:** that `gl_Position.w` matches the mesh path's `dot(cam_forward, v_world_position)` z-slice
  and that lit smoke reads correctly near punctual lights. **Accepted limitation:** the
  `centre − eye` subtraction is in float, so at planetary distances it inherits the cosmetic pool's
  float32 position precision — a near-camera envelope, the same envelope the pool's absolute-float
  positions already live in.
- **Known slice limitations to revisit.** Deterministic billboards draw additively (no per-billboard
  blend on `Render::ParticleBillboard`), so a deterministic Smoke emitter glows rather than
  composites; only `emitters[0]` of a multi-emitter effect is CPU-simulated on the deterministic
  path; the alpha sort runs over the whole 2^16 pool when any alpha emitter exists (optimize with a
  smaller sort budget later); particles write no velocity, so fast sparks ghost mildly under TAA
  (like clouds); the GPU pool's cross-frame write→read is gated by frame pacing, not fully decoupled
  from frames in flight; the Particle Editor panel recompiles the effect each frame it is open. None
  of these block the shipped features.
- **Async-compute sim** (VFX5+): overlap the sim with graphics; requires the §5.2 handoff buffers
  stay graph transients and the pools stay compute-only.
- **Post-TAA particle composite** into `targets.resolved` for ghost-free fast sparks — only if
  VFX2's
  velocity-less pre-TAA ghosting proves unacceptable.
- **GPU-side deterministic path** — not planned; determinism stays CPU-side by construction. If GPU
  determinism is ever required, it is a separate backend behind the same seam.
- **Interop** with SushiRuntime's SYCL graph for the sim (zero-copy USM) — tracked with the
  renderer's
  M2 interop milestone, not before.
