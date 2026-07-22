# Render Pipeline Refactor — Toward an AAA, Performance-First Renderer

Status: **Living design document.** Phases 0, 1, 2 and 5 are **shipped and verified
against source** (see §3 — Completed). **Phase 3 is shipped:** items 3.1 (the tier
contract / `QualityParams`), 3.2 (`VulkanSceneView` → `ViewResources` extraction), 3.3
(background PSO optimizer), 3.4 (Vulkan 1.4 floor), 3.5 (descriptor-writer seam), and
3.7 (IBL diffuse → SH-9) are **shipped**; 3.6 (the Slang decision) is now **decided —
deferred with recorded rationale** (GLSL stays; revisit at Phase 6). **Phase 4 is
essentially complete:** the clustered Forward+ core (4.1–4.3), **per-light shadows**
(4.4 — spot *and* point-light cube maps, soft Vogel PCF), **clustered decals** (4.5 —
tint *and* bindless albedo/ORM textures), the **Lighting window** (4.6), and the
**emissive→bloom seam** (4.7, verified HDR) are all **shipped**, each tier-wired and
authored as a per-entity component. Remaining in Phase 4 are opt-in refinements, not
gaps: area/IES lights (deferred by design to a later increment), projected normal-map
decals, and shadow caching / screen-coverage quadtree tiles / adaptive per-light PCSS.
Alongside, a **rendering-quality pass** landed: Vogel-disc shadow PCF, a temporally
stable analytic-ground shadow, softened punctual shadows, and a TPDF output dither —
replacing the visible dithering/banding with smooth, temporally-stable results (an early
down-payment on §5.1 blue-noise and §9.6 dither). **Phase 5 is complete for its shippable
scope:** the shared noise source is consolidated into one `blue_noise.glsl` (§5.1), **GTAO**
(§5.2) ships — half-resolution horizon occlusion with a bent normal, joint-bilateral-upsampled
and frame-stable so it reads smooth, feeding the indirect diffuse and a bent-normal specular
occlusion (§5.5) — and **hi-Z + screen-space reflections** (§5.3) ship: a pass-owned
nearest-depth pyramid, a thin roughness/F0 G-buffer, and a smooth mirror trace folded into
the scene. Local reflection probes (§5.4) are deferred with recorded rationale (they need the
editor's visual loop to land a scene-capture path safely); the reflection chain functions
without them via SSR plus the IBL fallback. **Phase 7 is shipped:** the Hillaire 2020 LUT
stack (transmittance, multi-scatter, per-frame sky-view, and the aerial-perspective froxel
volume), **volumetric fog** with authored local box/ellipsoid volumes, and the cloud
coupling seam — the atmosphere is LUT-driven end to end, retiring the full-resolution
per-pixel single-scatter march for the common cases (background sky and near meshes) and
LUT-accelerating the rest. Only opt-in refinements remain (CSM god-ray shafts and
punctual-light fog, per-tier LUT resolutions, temporal amortization). **Phase 6 GI is
shipped (core):** a probe-volume cascade of SH-9 irradiance probes fed by a pluggable
tracer — the default SDF cone tracer (all hardware) rebuilds a scene distance clipmap
from the analytic primitives and per-mesh bricks baked at import and sphere-traces it per
probe for occlusion and one coloured bounce, amortized by sparse round-robin relight, with
rough reflections falling back to the same probe cache. Multiple cascades, emissive
injection, toroidal amortization, and the Tier B ray-query tracer remain as tier-scalable
refinements. This revision re-audits the codebase, folds in
a 2024–2026 state-of-the-art survey (SIGGRAPH Advances 2021/2023/2025, GDC 2024/2025,
GPUOpen, vendor SDK documentation), and replaces the remaining roadmap with detailed,
AAA-complete phases. The guiding constraint is unchanged: the *red line between
maximum realism and performance* — where the two conflict, **performance wins**.
Every technique carries a quality tier so the expensive half is opt-in per platform.

Companion document: **[weather_and_clouds.md](weather_and_clouds.md)** — the
volumetric cloud + planetary weather simulation plan (Phase 8 here defers to it).

---

## 0. North star

- **Physically-based, temporally-stable, HDR end to end.** No banding, no shimmer,
  no LDR intermediate until the final display encode — and the display encode itself
  is HDR-capable (scRGB / HDR10), not hard-wired to sRGB.
- **Data-oriented and GPU-driven.** The CPU describes *what* to draw; the GPU decides
  *which* to draw and submits its own work. Draw count must not scale the CPU cost.
- **A render graph, not a call sequence.** Passes declare reads/writes; barriers,
  aliasing, and scheduling are derived, never hand-written. This is the SOLID spine —
  shipped in Phase 0 and now the law for every new pass.
- **Planet-scale correct.** Camera-relative rendering + reverse-Z are invariants —
  every new pass inherits them. Anything that caches world-space data (GI probes,
  reflection probes, cloud history) must be camera-relative or rebase-aware.
- **Runtime-interop first.** Heavy precompute (noise, LUTs, culling, simulation)
  runs on the GPU / SushiRuntime SYCL graph, not `std::thread` on the CPU.
- **The tier contract is real.** `RenderQuality` (Low / Medium / High / Ultra) gates
  the expensive half of *every* pass. Today it gates exactly one thing (ray-traced
  shadows); Phase 3 makes the contract enforceable and every later phase lands
  tier-wired on day one.
- **Upscaler-ready temporal core.** Every temporal pass consumes the same
  color/depth/motion/exposure/jitter contract a vendor upscaler needs, so FSR/DLSS/
  XeSS slot in behind one interface instead of forking the pipeline.

---

## 1. Where we are — verified audit (2026-07)

The claims below were verified against source, not the previous revision of this
document.

### 1.1 Shipped and working

| Area | State |
|---|---|
| Render graph | `render/graph/` — declared access → derived `VkImageMemoryBarrier2`, dynamic-rendering scopes, dead-pass culling, transient lifetime reuse. No pass writes a barrier (one audited exception: acceleration-structure builds, which have no graph vocabulary). |
| Pass split | `render/passes/` — one file per pass, 15 passes, all behind `register_pass(graph, frame)`. |
| Resources | Disk-backed `VkPipelineCache` + `VK_EXT_graphics_pipeline_library` 4-library fast link, bindless update-after-bind heap, per-slot descriptor pools, sampler/shader caches, shader hot-reload (glslang in-process). |
| Profiling | Per-pass timestamp queries surfaced in the editor viewport HUD — every phase lands against a measured budget. |
| Materials | Full Unity-Standard-parity model: albedo/MR-ORM/normal/height(POM)/occlusion/emission/detail set, per-set tiling-offset, GPU material SoA + per-draw index, glTF 2.0 + `KHR_materials_*`, mip-streamed texture library against a byte budget. |
| BRDF | Height-correlated Smith, Kulla-Conty multi-scatter, roughness-aware Fresnel, specular occlusion (Lagarde), IOR-derived F0 — plus anisotropy, clearcoat, sheen, transmission behind per-material flags (landed early; re-gate under tiers in Phase 3). |
| IBL | Sky-captured cubemap → GGX-prefiltered specular chain + irradiance cube + split-sum BRDF LUT; recapture rate-limited and change-gated. |
| Shadows | 4-cascade CSM in one 2×2 atlas (sphere-bounded, world-anchored texel snap, camera-relative fit), PCSS (blocker search + sun angular radius), screen-space contact shadows, cloud transmittance on the sun term, and tier-gated ray-query sun shadows over a real BLAS/TLAS. |
| Temporal | Motion vectors (camera-relative-safe), Halton jitter, TAA (velocity dilation, YCoCg clip, Catmull-Rom, Karis weighting, sharpen), temporal upscaling to the output grid, GPU-time-driven dynamic resolution, VRS mask on the sky pass, FXAA fallback. |
| Depth | Depth prepass sharing `mesh.vert` bit-exactly; reverse-Z, infinite far, `D32_SFLOAT_S8_UINT`. |

### 1.2 Debt carried out of the completed phases (owned by Phase 3)

1. **The tier contract is unenforced.** `RenderQuality` exists and the editor edits
   it, but only `RayTracedShadowPass` reads it. Nothing else scales with tier.
2. **`VulkanSceneView` is 771 lines**, not the promised <300 — ~350 lines are
   target/readback lifecycle that belongs in a view-resource module.
3. **Transient "aliasing" is whole-resource reuse**, not memory aliasing on shared
   allocations. Fine at today's target count; revisit when froxel volumes, GI
   probes, and history buffers multiply the transient set.
4. **The upscaling hook is homegrown-temporal only.** No vendor upscaler behind it,
   and no abstraction a vendor upscaler could implement yet.
5. **IBL diffuse is a convolved 32³ cubemap, not SH** — costs a sample per pixel
   where a 9-coefficient SH would be uniform reads, and blocks cheap probe blending
   (Phase 6 needs SH anyway).
6. **Cloth is still CPU-triangulated and re-uploaded per frame** (buffers are
   per-slot and grow-only now, but the geometry work is host-side).
7. **No per-instance GPU path**: one `vkCmdDrawIndexed` + push constants per object.
   The material/motion SoA indices are already the right shape; the draw loop is not.

### 1.3 The AAA gap (what the new phases exist to close)

**Lighting** — one directional light, period. No punctual/spot/area lights, no
clustered culling, no per-light shadows, no GI beyond the sky IBL, no AO beyond the
material map, no SSR, no volumetric fog. This is the largest remaining realism gap
and it is *structural* (the light list, cluster grid, and shadow atlas are missing,
not just passes).

**Atmosphere** — still a full-res per-pixel single-scatter march with a hand-tuned
multi-scatter constant. The Hillaire LUT stack (transmittance / multi-scatter /
sky-view / aerial-perspective froxels) is absent — quality *and* performance on the
table simultaneously.

**Clouds** — half-res march exists but repeats visibly, degrades with altitude, and
has no temporal accumulation and no weather dynamics. Full plan in the companion
document.

**Post** — fixed exposure × ACES × gamma. No auto-exposure, bloom, grading, DoF,
motion blur, dither, or HDR output.

**Geometry** — no instancing, no indirect draws, no GPU culling, no meshlets.

**Delivery** — single graphics queue, binary fences, double buffering, no async
compute, no vendor upscalers, no frame pacing.

**Editor** — no Lighting window, no Post-Process Volume stack (the Rendering panel
and material inspector exist).

---

## 2. Target architecture

```
                    ┌─────────────────────────────────────────────┐
                    │              RenderGraph  (shipped)          │
                    │  declares passes, derives barriers/aliasing, │
                    │  schedules graphics + async-compute queues   │
                    └─────────────────────────────────────────────┘
   FrameContext ──▶  passes register into the graph each frame:

   [GPU 2-phase cull/HZB] → [Depth prepass ✓] → [Clustered light+decal cull]
        → [Forward+ opaque (thin GBuffer optional)] → [Shadows: CSM ✓ /atlas/RT ✓]
        → [GTAO] → [Sky-view+aerial LUTs] → [Lighting + IBL ✓ + decals] → [SSR]
        → [GI probe gather] → [Froxel fog] → [Clouds (companion doc)]
        → [Transparents/OIT] → [TAA ✓ / vendor upscaler] → [Auto-exposure] → [Bloom]
        → [DoF/MotionBlur] → [Tonemap (AgX/ACES/Neutral) + grade + dither]
        → [HDR/SDR encode] → [FXAA ✓] → [ImGui / present]

   (VRS ✓ + dynamic resolution ✓ wrap the shading-heavy passes; the graph places
    compute passes on the async queue once Phase 11 opens it.)
```

Module layout is unchanged and shipped: `render/graph/`, `render/frame/`,
`render/passes/`, `render/resources/`, `render/material/`, plus `render/scene/`,
`render/geometry/`, `render/textures/`, `render/raytracing/`. New systems land as
new modules behind the same seams (`render/lighting/`, `render/gi/`,
`render/weather/`, `render/post/`).

### 2.1 Lighting architecture decision (validated 2025)

**Clustered Forward+ stays the default** — the 2024–2026 survey confirms it: froxel
clustering is the settled norm (Doom, Detroit, Unity Forward+), robust to depth
discontinuities, MSAA/TAA-friendly, and the low-bandwidth choice for a flight sim's
mostly-opaque, planet-resolution scenes. The froxel grid is shared infrastructure:
light culling, decals, and volumetric fog all consume it (built once in Phase 4,
reused by Phases 7/8/13). A thin optional GBuffer stays on the table for deferred
decals/SSR fidelity, tier-gated. Visibility-buffer rendering is explicitly **not**
adopted — it pays off when triangle density collapses below pixel size (Nanite,
Northlight), which the planet LOD ladder already prevents.

For *many shadowed lights*, the 2025 pattern to adopt later is **stochastic light
visibility** (UE5 MegaLights-shaped: importance-sample a few lights per pixel with
temporal feedback, visibility by ray query or SDF trace) rather than ReSTIR DI,
which shipped only on HW-RT flagships and scales poorly on consoles. That is
Phase 12, and it drops into the clustered grid rather than replacing it.

### 2.2 Material model — **shipped**, see §3 Phase 1

The §2.2 model of the previous revision shipped essentially verbatim (verified:
maps, detail set, tiling/offset, rendering state, glTF import incl. extensions,
inspector). Two follow-ups live in the roadmap: tier-gating the advanced lobes
(Phase 3) and sparse virtual texturing for terrain-scale sets (Phase 10).

### 2.3 Per-camera view & culling system

Unchanged as a target; realized in Phase 10 (GPU two-phase Hi-Z per view) and
Phase 4 (per-light culling against the shadow atlas). Each `ISceneView` owns its
frustum, culling result, post-volume stack, and tier.

### 2.4 Editor integration — Lighting & Post-Processing windows

Unchanged as a target (the previous revision's full control list stands). The
Lighting window lands with Phase 4 (it edits the light list the phase creates); the
Post-Process Volume stack lands with Phase 9. Both back onto plain data structs that
map 1:1 onto pass parameters — the editor writes data, passes read data, neither
names the other (DIP).

---

## 3. Completed phases — archive

Kept for the record; each item is verified in source. Residual debt is listed in
§1.2 and owned by Phase 3.

### Phase 0 — Foundation ✓ (shipped)

1. ✓ RenderGraph with automatic barriers, dead-pass culling, transient reuse
   (`render/graph/render_graph.*`, `resource_state.*`).
2. ✓ Disk-backed `VkPipelineCache` + `VK_EXT_graphics_pipeline_library` fast link
   with monolithic fallback (`render/resources/pipeline_cache.*`).
3. ✓ Bindless update-after-bind heap, set 1, textures + storage buffers
   (`render/resources/descriptor_heap.*`).
4. ✓ Per-pass GPU timestamps + editor HUD (`render/graph/gpu_profiler.*`).
5. ✓ Shader hot-reload with in-process glslang and `#include`
   (`render/resources/shader_library.*`).
6. ✓ Cloud noise on compute dispatches (`render/textures/cloud_noise.*`) — Vulkan
   compute rather than SYCL; the SYCL interop seam remains Phase 11.

### Phase 1 — Shading core ✓ (shipped)

1. ✓ Vertex format: position, normal, tangent (w = handedness, zero = derive from
   derivatives), UV0/UV1, color; primitives regenerated with analytic tangents.
2. ✓ Material system + textures: the full §2.2 surface incl. POM with
   self-shadowing, detail set, per-set ST, neutral-default fallbacks so unset maps
   cost nothing semantically (`render/material/`, `pbr.frag`).
3. ✓ glTF 2.0 import (cgltf): MR + spec-gloss conversion, `KHR_texture_transform`,
   clearcoat/sheen/transmission/emissive_strength, tangent generation.
4. ✓ Mip-based texture streaming against a byte budget, non-blocking uploads,
   fence-guarded reclamation (`render/material/texture_library.*`).
5. ✓ BRDF upgrade: Kulla-Conty energy compensation, height-correlated Smith,
   roughness-aware Fresnel, specular occlusion, IOR-derived F0; advanced lobes
   present behind material flags.
6. ✓ IBL from the engine's own sky: prefiltered specular chain + irradiance cube +
   split-sum LUT, change-gated recapture (`render/passes/ibl_pass.*`).

### Phase 2 — Shadows ✓ (shipped)

1. ✓ CSM: up to 4 cascades in one 2×2 atlas — one image/pass/barrier/profiler
   entry; practical splits, sphere-bounded cascades, world-anchored texel snap,
   camera-relative fit, PCSS with 16 rotated Poisson taps averaged by the temporal
   resolve (`render/passes/shadow_pass.*`, `scene/shadow_uniforms.*`).
2. ✓ Screen-space contact shadows, metre-bounded march on the prepass depth
   (`render/passes/contact_shadow_pass.*`).
3. ✓ Cloud transmittance on the sun term for meshes and the analytic ground
   (`cloud_shadow_common.glsl`).
4. ✓ *(Ultra)* Ray-traced sun shadows: BLAS-per-mesh + per-frame TLAS, ray-query
   fullscreen visibility mask so the material shader never forks
   (`render/raytracing/`, `render/passes/ray_traced_shadow_pass.*`).

### Phase 5 — Anti-aliasing & temporal core ✓ (shipped)

1. ✓ Motion vectors: R16G16 target, per-frame previous-transform array, both sides
   camera-relative against their own frame's eye; sky reprojected from the view ray.
2. ✓ TAA: velocity dilation, YCoCg neighborhood clip, Catmull-Rom history,
   Karis-weighted blend, sharpen (`render/passes/taa_pass.*`, `taa.frag`).
3. ◐ Upscaling: homegrown temporal upscale (history at output extent) shipped; the
   vendor-upscaler abstraction is Phase 11.
4. ✓ FXAA fallback, fully history-free (`render/passes/fxaa_pass.*`).
5. ✓ Dynamic resolution driven by measured GPU time, quantized steps
   (`render/frame/resolution_controller.*`).
6. ✓ VRS: compute-derived per-tile rate image from luminance contrast + motion,
   bound by the sky pass; silently absent without the extension
   (`render/passes/shading_rate_pass.*`).

---

## 4. Roadmap — the remaining phases, AAA-complete

Ordering favors (a) structural unblockers, then (b) realism-per-cost. Every phase:
ships with its editor surface, its tier wiring, its GPU-profiler budget, and its
CHANGELOG/ARCHITECTURE entry in the same PR; lands as new files behind the
`register_pass(graph, frame)` contract (OCP); and must not regress the
camera-relative / reverse-Z invariants.

### Phase 3 — Foundation hardening & platform floor

Pays §1.2's debt before new systems multiply it, and locks the platform baseline.

1. **Enforce the tier contract.** ✓ **Shipped.** A single
   resolver `resolve_quality(authored) → {effective RenderSettings, QualityParams}`
   in one file (`render/frame/quality.cpp`); `QualityParams` and the resolver live in
   the *public* header `include/SushiEngine/render/quality_params.hpp` rather than
   under `render/frame/`, because the editor's resolved-params readout must see them
   and `render/frame/` is a private include dir (SRP: tier policy still lives in one
   file). The contract adopted: the authored settings **are** the High baseline —
   High resolves to the request verbatim (no default regression), Low/Medium scale the
   expensive half down from it, Ultra pushes it up. Wired through it so far: PCSS
   filter/blocker taps (via two spare `ShadowUniforms` lanes → `shadow_sampling.glsl`),
   contact-march length + shadow atlas size + cascade count (via the effective
   settings, no shader change), VRS coarsest-rate cap (`shading_rate_pass`), and the
   advanced material lobes (a tier lobe-mask AND-ed in `MaterialSystem`), and the cloud
   march budget (near/far/light step counts pushed to `cloud.frag` on the shared
   fragment push range, clamped in-shader so a stale push can never spin the loop out).
   Editor: the Rendering panel's "Tier resolves to" tree. Acceptance met: switching
   Low↔Ultra visibly rescales the shadow, contact, VRS, material, and cloud passes in
   the profiler HUD.
2. **Slim `VulkanSceneView`**: ✓ **Shipped.** The entire per-frame device lifecycle —
   the double-buffered command slots, the resolve image, the picking readback, the
   uniform staging buffers, the transient pools, and the parity-ping-ponged temporal
   history — moved into a new `render/rhi/vulkan/view_resources.{hpp,cpp}`; the view
   drives it through an interface and never touches a `VkImage` directly. Logic was
   copied verbatim (pure refactor, no behavior change). `vulkan_scene_view.cpp` went
   **771 → 372 lines**. The residual is not target/readback lifecycle but irreducible
   orchestration — `render()` (build FrameContext, fill uniforms, register passes,
   compile, submit) and the 14-pass constructor init list — so the module boundary,
   not a sub-300 line count, is the real acceptance here; fragmenting `render()` further
   would trade clarity for a number.
3. **PSO hitch elimination — background half ✓ shipped, precache list open.**
   `GraphicsPipelineFactory` hands out a swappable `PipelineHandle`: a graphics
   pipeline is fast-linked (GPL) for instant availability, then a background thread
   rebuilds it fully optimized (monolithic) and atomically swaps it into the handle
   (release/acquire on the handle's pointer); the superseded pipeline retires after a
   frames-in-flight delay sized past every view sharing the factory's clock. Passes
   resolve the handle at bind time and the factory owns every graphics pipeline. Not
   routed through SushiRuntime: PSO compile is a driver call, not GPU-computable work.
   Still open: pipeline usage harvesting to a precache list warmed at startup — a
   never-before-seen permutation still creates its GPL pipeline synchronously on
   first use today, so "zero first-use hitches in a captured flight" is not yet fully
   met.
4. **Vulkan 1.4 floor**: ✓ **Shipped.** Instance and device now require 1.4;
   `maintenance5`/`maintenance6`/push descriptors are required features (mandatory in
   1.4, no longer probed), `host_image_copy` is probed and stays optional. Texture
   streaming gained a host-image-copy path: a CPU-built mip chain is copied straight
   into the optimal-tiled image with no staging buffer, queue submit, or fence,
   gated on `SHADER_READ_ONLY` actually appearing in the device's `pCopyDstLayouts`;
   staging-plus-blit remains the fallback where it doesn't, and a superseded
   host-copied image retires on the same shared-clock frames-in-flight delay as the
   PSO swap above (sized past every view, not just one). Scene set 0 converted to
   push descriptors (`SceneLayout` + `SceneSetWriter`); the per-frame allocate/
   update/bind is gone across all eleven graphics passes. The fallbacks these
   obsoleted are deleted.
5. **Binding-layer future-proofing**: ✓ **Shipped.** New `DescriptorWriter` +
   `bind_descriptor_set()` centralise every descriptor write and set bind — the
   compute/RT/noise passes, `SceneSetWriter`, and `SceneLayout`'s bindless-heap bind
   all route through it, so the announced `VK_EXT_descriptor_heap` (Roadmap 2026)
   becomes a swap behind these two functions rather than a sweep of every call site.
   `VK_EXT_descriptor_buffer` is deliberately **not** adopted (dead end).
6. **Shading-language decision**: ✓ **Decided — deferred, rationale recorded.** GLSL
   stays. The reasoning: the shader set (~35 files) already has a working module system —
   `#include` composition (`shadow_common`, `temporal_common`, `clustered_lighting`,
   `pbr_common`, `shadow_sampling`), offline SPIR-V-header compilation via
   `render/tools/shader_compiler`, and in-process glslang hot-reload — so the structural
   wins Slang offers (modules, includes) are largely already in hand. Slang's *distinctive*
   wins (generics, interfaces, multi-backend codegen) pay off when a real permutation
   explosion or a second target (HLSL/WGSL) appears; this engine is SPIR-V/Vulkan-only and
   has neither yet. Against that, the migration is horizontal and unbudgeted: rewrite the
   compile tool around `slangc`, re-plumb the hot-reload path off glslang, and port every
   shader while preserving the bit-exact `mesh.vert`↔depth-prepass parity invariant — all
   with no user-visible feature. **Revisit trigger:** adopt Slang when generics would
   remove real duplication (the Phase 6 GI-tracer strategy tiers, or a material-permutation
   blowup) or when a non-SPIR-V backend is required — evaluated again at Phase 6, not before.
7. **IBL diffuse → SH-9** (from the audit): ✓ **Shipped.** A new `sh_project.comp`
   projects the captured environment radiance into 9 L2 SH coefficients in one
   change-gated workgroup dispatch inside the IBL build, with the cosine-lobe band
   factors and 1/π baked into the stored coefficients so a shader-side `evaluate_sh(n)`
   returns exactly the magnitude the irradiance cube did (verified against the uniform-
   environment case). `pbr.frag`'s diffuse term is now nine storage reads + a degree-two
   polynomial in the normal instead of a filtered cube fetch; the coefficients live in a
   144-byte IblPass-owned SSBO at scene-set binding 13, an IblPass-private resource
   barriered to fragment-read like the cubes. Phase 6's probe blending is now a blend of
   coefficients. (The irradiance cube is left generated but unused — a trivial cleanup
   removal later.)

*Performance note:* this phase is net-negative GPU cost (SH, precache) and
removes the worst CPU spikes (PSO links).

### Phase 4 — The light engine: clustered Forward+ (the structural gap)

The scene cannot exceed one light today; this phase makes light count a content
decision. New module `render/lighting/`. **Shipped:** 4.1–4.7 (the clustered core,
spot + point-light shadows, tint + textured decals, the Lighting window, and the
verified emissive→bloom seam). Only opt-in refinements remain (area/IES lights,
normal-map decals, shadow caching / quadtree / adaptive PCSS).

1. **Light list**: ✓ **Shipped (4A).** `PunctualLight` is a *record* in the public
   `include/SushiEngine/render/light.hpp` — point / spot, color, a raw-radiance
   `intensity` on the sun's footing (candela/lumen arrive with Phase 9 auto-exposure),
   `range` with a windowed inverse-square falloff, and spot inner/outer cones. It
   crosses the `render()` seam as a `const PunctualLight*` + count beside
   `MeshInstance`/`ClothStrandView`, is authored as a **per-entity light component**
   (`IWorldEditor::has_light`/`light_params`/`set_*`/`create_light`, Inspector + Create
   menu, the cloth-component precedent), and packs into one grow-only per-slot storage
   buffer (`LightSystem`, mirroring `MotionSystem` — positions made camera-relative in
   double). Area (rect/tube), IES profiles, and cookies are deferred. *SOLID:* no
   `ILightSource` — lights are data; the sun stays in `Environment`.
2. **Froxel cluster grid** (compute): ✓ **Shipped (4A).** A fixed 16×9×24 grid with
   logarithmic Z slicing (the Olsson mapping; same addressing the Phase 7 aerial
   froxels will share). `cluster_build.comp` runs one invocation per cluster,
   reconstructs the cluster's view-space AABB from its screen tile + depth slice, and
   tests each light's bounding sphere against it. Each cluster owns a fixed slot of the
   index list (`MAX_LIGHTS_PER_CLUSTER`), so the build needs **no global atomic and no
   clear** — it writes an authoritative count, race-free. Grid + index list are graph
   transients (`LightCullPass` writes, opaque reads, compute→fragment barrier derived);
   the compacted-atomic build is a later refinement when froxel occupancy demands it.
3. **Forward+ shading path**: ✓ **Shipped (4A).** `pbr.frag` includes
   `clustered_lighting.glsl`, maps the pixel to its cluster from screen position + the
   camera-relative view depth, and loops that cluster's lights with the existing base
   BRDF (windowed inverse-square + spot cone), adding to the direct term. Four new
   scene-set bindings (14–17: light buffer, count grid, index list, config UBO) carry
   it; the sun path is unchanged. Per-light specular occlusion / contact shadows and the
   advanced lobes stay a sun-path feature for now. Tier wiring: `LightEngineSettings`
   (`max_lights`, `cluster_far_distance`) in `RenderSettings`, scaled per tier in
   `resolve_quality` and surfaced in the Rendering panel's "Tier resolves to" tree.
4. **Per-light shadows**: ✓ **Shipped — spot + point.** One shared depth **atlas**
   (a 4×4 tile grid — one image, one `LightShadowPass`, one barrier), tiles claimed by
   the first N shadow-casting lights up to a tier-scaled budget. A **spot** claims one
   tile; a **point light** claims six — one 90° perspective face per cube direction,
   rendered into six atlas tiles (the pass already loops tiles, so six faces cost no new
   code) and selected in-shader by the fragment's dominant axis off the light
   (`cube_shadow_face` + a base record in `cone.z`). Each caster's camera-relative
   perspective light matrix (built in `LightSystem::assign_shadows`, a conventional depth
   projection unlike the reverse-Z camera) lands in a per-frame shadow buffer at scene-set
   binding 19; `light_shadow.vert` renders depth into the tile. `pbr.frag` filters it with
   an **8-tap rotated Vogel PCF** (soft penumbra, not a single hard 2×2 tap) through the
   comparison sampler (binding 18), the rotation frame-advanced so the temporal resolve
   averages the residual. A caster whose tiles do not fit the budget is shaded unshadowed.
   **Remaining:** quadtree tiles sized by screen coverage, dormant-light caching, and
   adaptive PCSS (blocker search) on High+ — the atlas exposes only the comparison
   sampler today, so a variable-penumbra search would need the raw-depth atlas bound too.
5. **Clustered decals**: ✓ **Shipped — tint + textured.** Projected box decals culled
   into the *same* froxel grid — `cluster_build.comp` gained a second sphere-vs-AABB pass
   writing a parallel decal grid + index list (bindings 20–22), and `pbr.frag` projects
   the fragment into each cluster decal's oriented box before shading, so the decal is
   lit, not pasted. A decal carries optional **bindless albedo and ORM maps** (`Decal::
   albedo_map`/`orm_map`, resolved to heap indices in `LightSystem::pack_decals` exactly
   as a material's maps are, packed into a sixth GPU lane): the decal's local right/up
   coordinates are the projection uv, the albedo texture's rgb replaces the tint and its
   alpha cuts the silhouette, and the ORM map overrides occlusion/roughness/metallic where
   it lands — so a decal reads as wet, rusted, or polished, not merely recoloured. With no
   map set it is the flat tint. Authored as a per-entity Decal component (create menu,
   Inspector with a path+Load texture field mirroring the material slots, copy/paste).
   **Remaining:** projected **normal**-map decals (lane 21 is reserved for it) — they need
   an orientation-signed tangent blend into the surface normal, a small follow-up.
6. **Editor Lighting window** (§2.4): ✓ **Shipped.** A dedicated `draw_lighting_panel`
   gathering the sun (elevation/azimuth/colour/intensity), the IBL source toggle +
   intensity, the sun's cascade-shadow settings, the tier-resolved punctual/decal/atlas
   budget, and the full punctual-light list — every light-bearing entity editable in
   place with an Add Light button and select-to-entity. **Remaining:** a full ephemeris
   driver surface and an authored decal list (decals live in the Inspector for now).
7. **Emissive → bloom seam**: ✓ **Verified.** The emissive term is HDR end to end with
   no clamp before where Phase 9's bloom will read it. Audited path: the editor authors an
   HDR emissive colour + an intensity up to ×100; `MaterialSystem` packs `emissive =
   colour × intensity` unclamped into the material SoA; `pbr.frag` adds `emissive` (map ×
   packed colour) straight into `out_color` on the `R16G16B16A16_SFLOAT` HDR target; and
   that float value flows unclamped through the sky/cloud composites and the TAA resolve
   (all `HDR_FORMAT`) into `resolved` — the buffer bloom will threshold. The only clamp in
   the chain is the ACES tonemap, which sits *after* the bloom seam. One noted Phase 9
   decision (not a Phase 4 gap): TAA's firefly clip can dim an isolated bright emissive
   texel; if that proves too aggressive for bloom, Phase 9 reads the pre-resolve HDR — a
   bloom-input choice, made when bloom lands.

*Performance notes:* clustered culling is the flat-cost foundation everything
reuses; the atlas + caching keeps worst-case shadow cost bounded by tile budget,
not light count. Tier: light count ceiling, atlas size, PCSS on/off.
*SOLID:* lights are data; culling, atlas, and shading depend on the grid and the
list, never on each other (ISP). `ILightSource` never appears — lights are not
polymorphic objects, they are records (data-oriented mandate).

### Phase 5 — Screen-space lighting quality: GTAO + SSR

The two biggest per-pixel realism wins after shadows, both bounded-cost. **Phase 5 is
complete for its shippable scope:** the shared-noise consolidation (5.1), **GTAO** (5.2),
**hi-Z + screen-space reflections** (5.3), and the **bent-normal specular occlusion** the
audit calls for (5.5) are shipped and building; local **reflection probes** (5.4) are
deferred with recorded rationale (below) — the reflection chain functions without them.

1. **Blue-noise infrastructure first**: a shared spatiotemporal blue-noise
   texture/sequence (per-frame offset, TAA-aware) as a `render/resources/` asset —
   GTAO, SSR, contact shadows, dither, DoF, and the cloud march all consume the
   same source (SRP; kills the per-pass ad-hoc hashes). ◐ **The SRP consolidation
   shipped:** a single `blue_noise.glsl` is now the one home for the interleaved-gradient
   hash, its frame-advanced form, and the TPDF dither — `temporal_common.glsl` includes it
   (keeping only the temporal-block-dependent phase/dither), and the two duplicate local
   copies in `contact_shadow.frag` and `tonemap.frag` were deleted and routed through it;
   the new GTAO pass draws its slice rotation from the same file. **Remaining:** swap the
   hash for a *baked* spatiotemporal blue-noise texture asset (the source upgrade, not the
   seam). *Earlier down-payment (2026-07):* the sun-shadow filter moved to a **Vogel disc**
   (even coverage at any tap count, so low tiers stop speckling), the analytic ground took a
   **frame-stable screen-space rotation** (it cannot be TAA-reprojected under translation,
   so a frame-varying dither read as raw noise), and punctual shadows gained a soft rotated
   Vogel PCF.
2. **GTAO** (half-res, compute, async-ready): horizon-based with a **bent normal +
   visibility cone** output; spatial denoise + the TAA accumulates the rest.
   Feeds: diffuse AO (multiplies IBL/GI diffuse), **specular occlusion upgraded**
   from the material-AO approximation to bent-normal cone vs reflection cone.
   Budget: ≤0.5 ms at 1080p internal on the mid tier.
   ✓ **Shipped.** New module-shaped pass `GtaoPass` (`render/passes/gtao_pass.*`) registers
   two graph sub-passes: `gtao.comp` marches the depth prepass at half resolution — view
   position and normal reconstructed from depth alone (no normal G-buffer exists), a few
   **frame-stable** rotated horizon slices integrating the GTAO arc into a visibility and a
   view-space **bent normal** (Jimenez 2016) — and `gtao_resolve.frag` **joint-bilateral
   upsamples** that to full resolution (depth-weighted, so the AO does not halo across the
   object boundaries the engine is being cleaned of) and rotates the bent normal into world
   space. The rotation is frame-stable and the resolve denoises, so the result is smooth
   without leaning on a temporal history to hide slice noise. The opaque pass samples it at
   a new scene-set binding (**23**, `AO_BINDING`): `pbr.frag` multiplies its ambient/IBL
   diffuse (and the flat-ambient fallback) by `occlusion × visibility` and occludes indirect
   specular. Tier: `GtaoSettings` (`enabled/radius/intensity/power/slices/steps`) in
   `RenderSettings`, slices+steps scaled per tier in `resolve_quality`. Direct light is
   untouched — AO darkens only the indirect term. **Remaining refinement:** a dedicated
   temporal accumulation (reproject via velocity) for even lower variance, and moving the
   slice rotation onto the baked blue-noise once 5.1's asset lands.
3. **Stochastic SSR**: hi-Z traced (needs the depth pyramid — build it here, it is
   also Phase 10's culling input; shared infrastructure, built once), GGX-jittered
   ray from the blue-noise sequence, half-res trace + neighborhood reuse +
   temporal accumulation (the FFX-SSSR shape), roughness-cutoff fade to IBL.
   Fallback chain: SSR hit → nearest reflection probe → sky IBL.
   Budget: ≤1.2 ms at 1080p internal, High tier. ✓ **Shipped.** Three pieces:
   (a) the **hi-Z pyramid** (`render/passes/hiz_pass.*` + `hiz.comp`) — a pass-owned
   nearest-depth mip chain (the graph exposes no per-mip storage view, so it owns the image
   and drives its own per-level barriers on the IblPass model), level 0 linearising the depth
   prepass and each finer level the 2×2 minimum above it, sampled with `textureLod`, rebuilt
   on resize, and reused by Phase 10 culling later; (b) a **thin reflection G-buffer**
   (`GBUFFER_FORMAT` RG16F = roughness, scalar F0) the opaque pass writes as a fourth
   attachment — `pbr.frag` fills it, the flat grid/outline shaders write fully-rough so they
   never reflect — since there is no normal buffer and the trace reconstructs the geometric
   normal from depth like GTAO; (c) the **trace + composite** (`ssr_pass.*` + `ssr.frag`),
   a fullscreen pass after the cloud composite that mirror-traces each smooth surface's
   reflection ray through the pyramid (linear march + binary refine), samples the lit scene
   at the hit, and folds it back Fresnel-weighted into a `scene_reflected` target the resolve
   reads via the `scene_final` indirection. **Deliberately a sharp mirror trace, no stochastic
   jitter** — gated to smooth surfaces (rough keep IBL), so it stays at the engine's
   shimmer-free bar rather than needing a temporal denoiser. Tier: `SsrSettings`
   (`enabled/max_steps/thickness/roughness_cutoff/intensity`). **Remaining refinements:**
   glossy reflections (roughness-mip prefilter of the scene colour), a proper hi-Z cell
   traversal for the march (it samples level 0 today), metal reflection tint (F0 is scalar),
   and the SSR→probe→sky fallback once probes (5.4) land — SSR currently just keeps the IBL
   term where a ray misses.
4. **Reflection probes**: box/sphere-projected local captures through the existing
   IBL capture path (it already renders the sky; point it at the scene), authored
   in the editor, blended by proximity, camera-relative anchored. ○ **Deferred —
   rationale recorded.** The reflection *chain* already functions without them: SSR
   resolves on-screen reflections and the surface's own IBL specular is the off-screen
   fallback, so probes are a fidelity increment (local geometry in reflections where a
   ray leaves the screen), not a gap. Against that, they are the one item in this phase
   that cannot be landed responsibly without the editor's visual loop: a local probe
   must render the *scene* (not just the sky the IBL pass renders today) from an
   arbitrary position into six cube faces, GGX-prefilter each, and blend by proximity —
   a secondary geometry render path driven from a non-camera viewpoint, with six
   face-orientation conventions and a prefilter/blend whose correctness is only
   confirmable on screen. **Plan when picked up:** a `ReflectionProbe` record (position,
   box/sphere extent, captured cube handle) authored as a per-entity component (the decal
   component is the precedent); a capture pass that reuses `IblPass`'s cube + GGX-prefilter
   machinery but renders the opaque + sky passes into each face from the probe eye
   (camera-relative, so the probe rebases like every other cache); SSR's miss path and
   `pbr.frag`'s specular IBL selecting the nearest probe's prefiltered cube before the sky
   cube; a Lighting-window/inspector authoring surface. **Revisit trigger:** land with the
   next block of visually-verified renderer work, when the editor loop is in use — not blind.
5. **Specular-occlusion chain audit**: one documented path — GTAO cone → bent
   normal → probe/SSR/IBL — replacing today's scalar approximation everywhere.
   ◐ **The GTAO→bent-normal half shipped** with 5.2 (`bent_reflection_visibility` in
   `pbr.frag` occludes indirect specular against the bent-normal cone, above the scalar
   Lagarde term); the probe/SSR ends of the chain arrive with 5.3/5.4.

*Tier:* GTAO res + tap count; SSR res, max steps, roughness cutoff; probes
static-only on Low. *SOLID:* both passes read the graph's depth/normal/velocity
handles; neither knows the other exists; the fallback chain is data (a priority
list), not branching code.

### Phase 6 — Global illumination (the 2025 baseline, no RT required) — **shipped (core)**

Flat sky ambient + AO is not AAA. The industry-settled mid-size pattern (Lumen-SW /
Brixelizer / AC-Shadows-shaped): **probe volumes fed by a pluggable tracer, cached
aggressively, refined per-pixel by Phase 5's screen-space terms.** New module
`render/gi/`.

**Shipped:** the probe-volume foundation (6.1), the SDF cone tracer with imported-mesh
bricks (6.2a/6.2b), sparse relight amortization (6.2c), and the reflection radiance-cache
fallback (part of item 4). Per-item status is marked below. **Deferred as tier-scalable
refinements** (each its own verified slice): multiple cascades (item 1 ships one cascade),
emissive injection (item 5), toroidal probe addressing (to amortize the trace during
continuous camera movement, not only when stationary), and Tier B ray-query tracing
(item 2's Ultra tier — the BLAS/TLAS seam exists; the consumer is future work).

1. **Irradiance probe clipmaps**: 3–4 cascaded volumes around the camera
   (camera-relative by construction — the planet-scale answer), SH-9 or
   octahedral-visibility probes (DDGI-style with depth for leak rejection),
   ~(32³–48³) probes per cascade, sparse relight (N probes per frame, dirty-first).
   ✓ **Shipped (one cascade).** A single 32×8×32 SH-9 cascade snapped to a world
   lattice, `IrradianceVolumePass` at scene set-0 bindings 29–30, sparse round-robin
   relight forced full on a lattice-cell shift or a sun move. Multiple cascades for
   distant coverage are the tier-scalable refinement.
2. **Pluggable tracer behind `render/gi/tracer.hpp`** (DIP — the strategy seam):
   ✓ **Shipped (Tier A).** `IProbeTracer` seam with `SdfProbeTracer` (default) and
   `EnvironmentProbeTracer` (floor tier).
   - **Tier A (all hardware)**: SDF scene tracing — per-mesh signed distance
     fields baked at import into a brick atlas + a coarse global clipmap SDF;
     cone/sphere tracing against it. ✓ **Shipped.** `mesh_sdf_baker` bakes a brick
     per imported mesh at import; `SdfProbeTracer` rebuilds a 64³ clipmap each frame
     from the analytic primitives + the mesh bricks and sphere-traces it per probe.
     (The same SDF assets later accelerate the cloud march and Phase 12's stochastic
     visibility — shared, not bespoke.)
   - **Tier B (Ultra)**: ray-query against the Phase 2 BLAS/TLAS, same probe
     consumers, zero shading-path fork — exactly the RT-shadow pattern. *Deferred*
     (the BLAS/TLAS seam exists; the probe consumer is future work).
3. **Sky/ground handoff**: probes integrate the Hillaire sky (Phase 7 LUTs) and
   the analytic planet ground as their environment miss — GI, sky, and IBL agree
   by construction; the IBL specular cube remains the distant-specular source.
   ✓ **Shipped (via the environment SH).** The trace miss reads the IBL environment
   SH the atmosphere already fed, so GI and IBL agree; a direct sky-view LUT read at
   the miss is a later refinement.
4. **Radiance-cache reuse for reflections**: SSR/probe misses fall back to probe
   radiance (the "one world cache serves GI and reflections" pattern) before sky.
   ✓ **Shipped.** Rough reflections in `pbr.frag` blend the probe radiance in the
   reflection direction, weighted by roughness, gated on GI being on.
5. **Emissive injection**: emissive surfaces contribute via the tracer (SDF albedo
   atlas / ray-query hit shading) — lights-from-materials for free at probe rate.
   *Deferred* — needs an emissive channel in the clipmap alongside the albedo.

*Performance:* probe relight is compute, async-queue-ready, amortized (nothing
per-pixel except the final trilinear+SH gather ≈0.3 ms); cost is flat in scene
complexity. This is deliberately **not** per-pixel ReSTIR GI — probe caches need
no denoiser (the survey's consistent conclusion for mid-size engines).
*Tier:* cascade count/extent, probes-per-frame, tracer tier, bounce count (1 now,
multibounce via cache feedback on High+).

### Phase 7 — Atmosphere: the LUT stack + froxel fog — **shipped**

Replaces the full-res per-pixel march — a large speedup *and* multiple scattering
at once. The performance-first mandate's flagship. **Shipped:** the full LUT stack
(7.1), volumetric fog with local volumes (7.2/7.3), and the cloud coupling seam (7.4).

1. **Hillaire 2020 LUT stack**: ✓ **Shipped.** A new `AtmosphereLutPass`
   (`render/passes/atmosphere_lut_pass.*`) owns four resources built by four compute
   shaders that share `atmosphere_common.glsl` (so builder and sampler can never drift):
   a **transmittance LUT** (256×64, Bruneton parameterization, optical depth to space)
   and a **multiple-scattering LUT** (32×32, Hillaire's 64-ray isotropic gather closed to
   all orders by `L2/(1−f_ms)`) — both view-independent and change-gated on the medium;
   a per-frame **sky-view LUT** (192×108, the background sky's in-scatter in the camera's
   local frame, non-linear horizon split); and a per-frame **aerial-perspective froxel
   volume** (32×32×32, squared depth distribution, one column-march thread per (x,y),
   reading the scene UBO so the camera basis matches `sky.frag` exactly). `sky.frag` now
   selects: background → sky-view LUT + transmittance for the fade; mesh within 32 km →
   aerial volume; analytic ground and far geometry → a march that reads the sun's
   transmittance and the multiple scattering from the LUTs instead of a per-sample light
   ray. Five scene-set bindings (24–28) carry the stack; the sun disk/eclipse/star/body
   work and the WGS84 ground intersection are unchanged. The IBL capture keeps the march
   (its cube viewpoints do not match the camera-aligned volumes) but reads the
   transmittance/MS LUTs so the captured environment tracks the same scattering, gated by
   `ibl_params.w`. Net-negative GPU cost. **Remaining refinements:** sky-view/aerial
   temporal amortization, and per-tier LUT resolutions.
2. **Froxel volumetric fog**: ✓ **Shipped.** A dedicated `VolumetricFogPass`
   (`render/passes/volumetric_fog_pass.*`, `fog_scatter.comp`) marches a height-graded fog
   medium into a second camera-frustum froxel volume (reusing the aerial addressing), each
   froxel gathering the sun — phase-weighted, attenuated by the transmittance LUT — plus a
   constant ambient fill, its in-scatter (sun radiance folded in) and transmittance folded
   over **every** pixel in the composite. Authored through `Environment::fog` and a Fog
   editor section; tier-gated off on Low (`QualityParams::volumetric_fog`); a no-op when
   disabled. **Remaining:** CSM-shadowed god-ray shafts and punctual-light in-scatter —
   the increment that turns the single column-march into a jittered inject/integrate pair
   consuming the Phase 4 cluster grid and the shadow atlas.
3. **Local fog volumes**: ✓ **Shipped.** Up to eight authored box/ellipsoid primitives
   (`FogVolume` records on `Environment`, edited as a list in the Fog panel) blend extra
   density and tint into the same froxel grid for valley or airfield fog — data, not new
   passes. Packed camera-relative into a small ring of uniform buffers the fog pass owns,
   with a soft-edge falloff so a primitive fades in rather than cutting a hard boundary.
4. **Cloud coupling seam**: ✓ **Landed (architectural).** The transmittance and sky-view
   LUTs are scene-set bindings any set-0 shader inherits, and `atmosphere_common.glsl`
   exposes `sample_transmittance` (sun transmittance at arbitrary altitude/angle) and
   `sample_sky_view` (sky radiance) as the ready API. The cloud pass already shares the
   scene layout, so the companion cloud plan's lighting model plugs into these without
   touching the atmosphere again — the consumption itself lands with Phase 8 (§weather doc).

*Tier:* fog on/off at Low (shipped); LUT/froxel resolutions per tier is a refinement.
*SOLID:* LUT generation, fog, and the sky composite are separate passes sharing resource
handles; the ephemeris keeps driving inputs through `Environment` unchanged (OCP over the
existing seam), and fog volumes are records, not objects.

### Phase 8 — Volumetric clouds & planetary weather → companion document

The full engineering plan — Nubis³-class voxel/SDF-accelerated clouds,
anti-repetition modeling, altitude-stable quality, temporal accumulation, and the
dynamic meteorology simulation (wind fields, fronts, humidity/precipitation cycle,
diurnal heating) that drives coverage instead of static authored maps — lives in
**[weather_and_clouds.md](weather_and_clouds.md)**. It consumes: Phase 7's
transmittance/sky-radiance seam, Phase 5's blue-noise, the temporal core, and the
SDF assets of Phase 6. Its acceptance criteria (flight-sim-grade: no visible
repetition, crisp from ground to 20 km, flyable-through) are contractual there.

### Phase 9 — Post-processing stack & display-out

1. **Auto-exposure**: compute luminance histogram → EV with min/max clamps,
   metering mask, adaptation rates, EV compensation; physical-camera units
   (EV100) so sun/sky/emissive intensities finally mean something. Replaces the
   fixed `0.18`.
2. **Physically-based bloom**: 6–7 mip progressive down/upsample, energy-
   conserving, threshold-free (scatter parameter), optional lens-dirt. ≤0.4 ms.
3. **Tonemap upgrade**: **AgX default** (industry consensus for hue preservation),
   ACES and **Khronos PBR Neutral** selectable; the tonemapper parameterized on
   display transform, not hard-coded to gamma-sRGB.
4. **HDR output**: scRGB (FP16) and HDR10 (PQ/Rec.2020) swapchain paths via
   `VK_EXT_swapchain_colorspace` + `VK_EXT_hdr_metadata`; UI composited in linear
   before the single encode; peak-nits / paper-white exposed. Cheap, high
   perceived value.
5. **Color grading**: white balance, lift-gamma-gain, saturation/contrast, 3D LUT
   slot — applied inside the tonemap pass (one fullscreen pass for the whole
   grade+map+encode chain).
6. **Blue-noise dither** before every 8-bit quantize (kills the sky banding) —
   consumes Phase 5's shared blue-noise. *Landed early (2026-07):* `tonemap.frag` now
   applies a **triangular-PDF dither of one LSB** (two decorrelated interleaved-gradient
   draws) before the UNORM write, static in screen space since the temporal resolve has
   already run — the horizon/sky gradient no longer bands. Upgrading the source to the
   shared blue-noise texture is the residual, in step with §5.1.
7. **DoF** (gather-based bokeh, physical aperture) and **motion blur**
   (per-object + camera from the shipped velocity target), both tier-gated, both
   after TAA in the post chain.
8. **Vignette / chromatic aberration / film grain / lens flare**: standard knobs,
   grain applied pre-dither in linear.
9. **Post-Process Volume stack + editor window** (§2.4): global + blendable
   volumes resolving to the per-camera parameter block the passes read.

*SOLID:* every effect is a pass or a parameter of the display-transform pass;
the volume stack is pure data resolution (no pass reads the editor).

### Phase 10 — GPU-driven geometry

Removes the per-object CPU wall before scene density grows into it.

1. **Instancing + multi-draw-indirect**: per-instance records in one storage
   buffer (transform/material/motion indices — the SoA shapes already exist),
   one `vkCmdDrawIndexedIndirectCount` per pass bucket. CPU cost flat in
   instance count.
2. **GPU two-phase occlusion culling**: depth pyramid (**the Phase 5 hi-Z,
   reused**) — phase 1 draws last-frame-visible + tests the rest against
   prev-frame HZB; phase 2 retests false negatives against current HZB. No
   readback, no popping. Per-view (§2.3): scene, shadow cascades, atlas tiles,
   probes.
3. **GPU LOD selection** folded into culling (screen-coverage select, hooks into
   the existing body-LOD ladder so planet-to-space reuses one system);
   HLOD/impostors for far clusters.
4. **Meshlets** *(tier)*: `VK_EXT_mesh_shader` path with per-meshlet cone +
   HZB culling (the Northlight/id pattern — meshlets without software raster);
   classic path remains the fallback. Adopt `VK_EXT_device_generated_commands`
   only if pass-bucket switching shows up in profiles.
5. **Sparse virtual texturing** for terrain/large sets: software SVT with a
   shader-written feedback buffer (Vulkan has no sampler feedback), page cache +
   async transfer-queue uploads (Vulkan 1.4 guarantees the queue).
6. **Cloth on GPU**: triangulation + normals in compute from the persistent
   per-slot buffers; upload only particle positions.

*Tier:* meshlets, SVT residency budget, HLOD distance. *SOLID:* culling writes
draw args; passes consume them — the draw loop stops being renderer code and
becomes GPU data (the GPU-driven mandate realized).

### Phase 11 — Upscaling, async compute & frame delivery

1. **Upscaler abstraction**: one interface (`render/frame/upscaler.hpp`) taking
   color/depth/motion/exposure/jitter + camera deltas — implemented by the
   shipped temporal upscale, **FSR 3.1** (vendor-agnostic floor; its API carries
   forward to FSR4-class ML upgrades), **DLSS via Streamline**, **XeSS** (DP4a
   fallback covers non-Intel). Camera-relative motion vectors are already the
   correct input contract. Frame generation: interface-reserved, not integrated
   until latency plumbing exists.
2. **Async compute queue**: the graph scheduler places flagged passes (GTAO, GI
   relight, cluster build, LUTs, histogram, SVT transcode) on the compute queue
   with cross-queue sync derived like everything else. Target: 10–20% frame-time
   recovery from overlap.
3. **Timeline semaphores + frame pacing**: replace binary fences, 3 frames in
   flight optional, submit-as-late-as-possible latency mode, per-queue
   present timing.
4. **SushiRuntime SYCL interop**: device-UUID-matched zero-copy of simulation
   output (weather grid, particles, soft bodies) into renderer-visible buffers —
   the "interop-first" mandate's landing.

### Phase 12 — The ray-tracing tier & many-light scaling *(Ultra)*

Everything here drops into seams earlier phases built; nothing forks the pipeline.

1. **RT probe feeding**: Phase 6 Tier B matured — ray-query probe relight with
   SHaRC-style world-space radiance cache; multibounce via cache feedback.
2. **RT reflections**: replace SSR *misses* only (the hybrid pattern), NRD-class
   denoiser (ReBLUR) — adopt the library, do not write SVGF from scratch.
3. **Stochastic direct lighting** (MegaLights-shaped): reservoir-lite importance
   sampling over the clustered light list, visibility by ray query (Tier B) or
   SDF trace (Tier A "high"), temporal feedback through the existing TAA chain —
   removes the per-light shadow-map ceiling; hundreds of shadowed lights.
4. **RT sun shadows**: already shipped (Phase 2); fold its denoise into the NRD
   integration.

### Phase 13 — Completeness (opportunistic, tier-gated)

- **OIT** for layered glass/canopy/particles: MBOIT default, adaptive-voxel OIT
  where cockpit-through-cloud layering demands it.
- **Screen-space subsurface scattering** (separable Burley) feeding the existing
  transmission lobe.
- **Hardware tessellation / displacement** where POM's silhouette limit shows
  (terrain patches, hero close-ups).
- **CAS** after upscaling.
- **Water**: screen-space + planar reflections, foam, refraction for the planet
  ocean (couples to the weather doc's wind field for sea state).
- **Debug views**: overdraw, mip level, light complexity, cluster occupancy, VRS
  rate, cull heatmap, probe visualization — surfaced beside the profiler HUD.

---

## 5. Cross-cutting concerns

- **Quality tiers.** Phase 3 makes the tier contract enforceable; from then on a
  phase that lands without tier wiring is incomplete by definition. The default
  target is **High at a locked frame budget**; the red line moves by dropping a
  tier, never by ad-hoc feature disabling.
- **Performance budgets.** Every phase item above carries a budget; the Phase 0
  profiler HUD is the referee. A pass that blows its budget ships tier-reduced,
  not deferred.
- **Color management.** One convention: sRGB/linear tagged at import (shipped),
  all lighting linear, one display transform (Phase 9 makes it HDR-aware).
- **Precision invariants.** Camera-relative + reverse-Z non-negotiable. New
  world-anchored caches (GI probes, reflection probes, SDF clipmaps, weather
  grids) must define their rebase story in their design review — the CSM
  world-anchored texel snap is the precedent.
- **Determinism.** The weather simulation (companion doc) is part of the SushiLoop
  determinism domain; rendering may consume it but never feed back into it.

## 6. How this satisfies SOLID

- **SRP** — graph / passes / pools / material / frame context shipped as separate
  modules; new systems (lighting, GI, post, weather) follow the same split; tier
  policy centralizes in one resolver instead of scattering per pass.
- **OCP** — every roadmap item is a new `register_pass` client or new data
  consumed by an existing pass; the shipped passes are not edited to add effects
  (the cloud-genus catalogue remains the extension pattern).
- **LSP** — all passes honor `register_pass(graph, frame)`; both GI tracer tiers
  honor one tracer interface; every upscaler honors one upscaler interface —
  swapping implementations never changes callers.
- **ISP** — passes see handles and parameter blocks, not the renderer; the editor
  sees data structs, not passes; lights/volumes/probes are records, not objects.
- **DIP** — passes depend on graph/resource abstractions; Vulkan stays in
  `render/rhi/vulkan`; the GI tracer, upscaler backend, and weather provider are
  strategy seams chosen at runtime.

## 7. Suggested sequencing

**3 → 4 → 5 → 7 → 6 → 8(companion) → 9 → 10 → 11 → 12 → 13**, with 5/7
interleavable after 4, and 9.1–9.6 (exposure/bloom/tonemap/dither) landable any
time after 5 (they only need the blue-noise asset). Rationale: 3 unblocks honest
tiering everywhere; 4 is the structural lighting gap; 5+6+7 are the realism
core; 8 rides on 5/6/7's seams; 10/11 are the scale/perf multipliers; 12 is the
Ultra crown.

## 8. Biggest realism-per-cost wins (if you only do five things)

1. **Clustered light engine** (Phase 4) — the scene stops being a one-light demo.
2. **Atmosphere LUTs + froxel fog** (Phase 7) — multiple scattering, god rays,
   *and* a large speedup at once.
3. **GTAO + stochastic SSR** (Phase 5) — contact darkening and glossy response;
   the two cheapest per-pixel realism multipliers left.
4. **Probe-volume GI** (Phase 6) — ambient stops being flat; time-of-day indirect
   for free; no denoiser needed.
5. **Clouds & weather** (Phase 8 / companion doc) — the flight-sim identity
   feature: War-Thunder-class skies that evolve.
