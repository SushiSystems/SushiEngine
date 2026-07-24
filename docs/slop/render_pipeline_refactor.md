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
refinements. **Phase 9 post-processing is shipped (core):** a tier-wired post chain after the
temporal resolve — histogram auto-exposure, energy-conserving bloom, an AgX/ACES/Neutral tone
curve, a white-balance/lift-gamma-gain colour grade, gather-based depth of field and motion
blur, and the vignette/chromatic-aberration/film-grain lens effects — all reading one post
parameter block authored by a dedicated **Post Process** editor window; only HDR10/scRGB
swapchain output is deferred (unverifiable without an HDR display). **Phase 10 GPU-driven
geometry is shipped (core):** the CPU stops issuing one draw per object — `InstanceSystem`
packs instances into a `GpuInstance` buffer grouped by mesh into per-mesh buckets, a compute
`CullPass` frustum-, screen-coverage-, and occlusion-culls each instance (testing against a new
persistent max-Z occlusion pyramid, the conservative twin of the Phase 5 hi-Z, reprojected from
last frame) and writes one indirect draw per bucket, and `mesh_gpu.vert` draws them from a set-2
instance descriptor set — a two-path design (GPU-driven when the tier permits, the heap is
present, and nothing is selected; classic CPU per-instance draw otherwise), authored by a **GPU
Culling** editor panel; cloth triangulation moved to a compute pass (`cloth.comp`/`ClothPass`, the
host uploading only particle positions), and a `VK_EXT_mesh_shader` meshlet path (task + mesh
shaders, per-meshlet frustum cull, device+Ultra gated with classic fallback) added on top, leaving
only sparse virtual texturing deferred with recorded rationale. **Phase 11 delivery is shipped
(core):** the render graph learned queues — a pass may ask for the async compute queue, and the
compiler splits the schedule into per-queue submissions, deriving their cross-queue waits and
their concurrent-sharing needs from the same declarations the barriers come from (the clustered
light cull and the GTAO march ride it today) — binary fences gave way to per-queue timeline
semaphores with 2-or-3 frames in flight and selectable present pacing, reconstruction moved
behind one `IUpscaler` interface that the shipped temporal resolve now implements (vendor
backends deferred; an absent SDK resolves back to it with the reason surfaced), and the engine
half of the SushiRuntime interop seam — an exportable, UUID-stamped device allocation — landed
with the runtime-side import recorded as blocked on an API the runtime does not yet have. This
revision re-audits the codebase, folds in
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
4. ~~**The upscaling hook is homegrown-temporal only.** No vendor upscaler behind it,
   and no abstraction a vendor upscaler could implement yet.~~ *Half paid in Phase 11:*
   the abstraction exists (`IUpscaler`) and the temporal resolve implements it; no vendor
   backend is vendored yet (§Phase 11.1).
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

   (VRS ✓ + dynamic resolution ✓ wrap the shading-heavy passes; the graph places a
    flagged compute pass on the async queue ✓ and derives the cross-queue wait.)
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
   compute rather than SYCL; the SYCL interop seam landed engine-side in Phase 11.4,
   with the runtime-side import still blocked on a SushiRuntime API.

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
   vendor-upscaler abstraction shipped in Phase 11.1 (the resolve now implements it).
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

### Phase 6 — Global illumination (the 2025 baseline, no RT required) — **shipped**

Flat sky ambient + AO is not AAA. The industry-settled mid-size pattern (Lumen-SW /
Brixelizer / AC-Shadows-shaped): **probe volumes fed by a pluggable tracer, cached
aggressively, refined per-pixel by Phase 5's screen-space terms.** New module
`render/gi/`.

**Shipped:** the probe-volume foundation (6.1), the SDF cone tracer with imported-mesh
bricks (6.2a/6.2b), sparse relight amortization (6.2c), the reflection radiance-cache
fallback (part of item 4), the three-cascade clipmap (item 1), and emissive injection
(item 5). Per-item status is marked below. **Deferred as tier-scalable refinements** (each
its own verified slice): toroidal probe addressing (to amortize the trace during continuous
camera movement, not only when stationary — a cascade cell crossing still forces a full
relight), and Tier B ray-query tracing (item 2's Ultra tier — the BLAS/TLAS seam exists;
the consumer is future work).

1. **Irradiance probe clipmaps**: 3–4 cascaded volumes around the camera
   (camera-relative by construction — the planet-scale answer), SH-9 or
   octahedral-visibility probes (DDGI-style with depth for leak rejection),
   ~(32³–48³) probes per cascade, sparse relight (N probes per frame, dirty-first).
   ✓ **Shipped (three cascades).** Three nested 32×8×32 SH-9 cascades at 4/8/16 m
   spacing (~124/248/496 m reach), each snapped to its own world lattice;
   `IrradianceVolumePass` at scene set-0 bindings 29–30; the shading pass reads the
   finest cascade containing the point and falls back outward. Sparse round-robin
   relight spans all cascades and is forced full only for a cascade that shifts a cell
   (the coarse grids shift far less often) or on a sun move. Toroidal addressing to
   amortize the shift itself during continuous movement is the tier-scalable refinement.
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
   ✓ **Shipped.** A second emissive clipmap parallels the distance/albedo one; the
   populate pass writes each surface's `material.emissive * intensity` (primitives and
   imported meshes), and the probe relight adds it at each trace hit — a glowing surface
   warms and tints nearby geometry through the same one-bounce trace, no separate light.

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

### Phase 9 — Post-processing stack & display-out — **shipped (core)**

The tail is no longer a fixed exposure × ACES × gamma. A tier-wired post chain now runs
after the temporal resolve, every pass reading one `PostProcessUniforms` block (scene-set
binding 31) the scene view fills from `RenderSettings::post` — the block the **Post Process**
editor window authors. **Shipped:** auto-exposure (9.1), bloom (9.2), the AgX/ACES/Neutral
tone-curve choice (9.3), the colour grade (9.5), the dither (9.6), DoF + motion blur (9.7),
the lens effects (9.8), and the editor window (9.9). **Deferred with rationale:** HDR10/scRGB
output (9.4). The one structural simplification vs the original plan: the Post-Process *volume
stack* (9.9) resolves today to a single global block per view rather than blendable spatial
volumes — the block-the-passes-read seam is in place, so blendable volumes are a later data
refinement over the same interface, not a new one.

1. **Auto-exposure**: ✓ **Shipped.** `AutoExposurePass` builds a 256-bin log-luminance
   histogram (`luminance_histogram.comp`, tiled shared-memory accumulation) of the resolved
   frame into a per-slot host-visible buffer; the scene view reads it back after the slot's
   fence — the picking-readback path, so no second scene-set binding and no cross-frame graph
   ping-pong — trims the tails, and eases a stored exposure toward the value mapping the
   central luminance mass onto `key`, with min/max EV clamps, up/down adaptation speeds, and
   EV compensation. Manual exposure (the scene's authored value × an EV compensation) stays the
   default and reproduces the fixed `0.18` behaviour exactly, so nothing regresses.
2. **Physically-based bloom**: ✓ **Shipped.** `BloomPass` — a progressive mip pyramid,
   13-tap Karis-averaged downsample (`bloom_down.comp`) and 3×3 tent upsample
   (`bloom_up.comp`), owned dual pyramids barriered by hand on the hi-Z model (the graph
   exposes no per-mip storage view), the finest upsample written into a half-resolution graph
   target the display transform composites in scene-linear space before exposure.
   Threshold-free by default (energy-conserving). Optional lens-dirt is the residual.
3. **Tonemap upgrade**: ✓ **Shipped.** `tonemap.frag` selects **AgX** (the new default),
   **ACES**, or **Khronos PBR Neutral** at runtime from the post block; each returns linear
   display values and the sRGB encode is applied once, so the operators stay interchangeable.
4. **HDR output**: ○ **Deferred — rationale recorded.** scRGB/HDR10 output is swapchain-
   colourspace surgery in `vulkan_window_renderer` (`VK_EXT_swapchain_colorspace` +
   `VK_EXT_hdr_metadata`) whose correctness — the PQ/paper-white/peak-nits encode and the
   linear UI composite — is only confirmable on an HDR display this project cannot verify
   against, the same deferral posture the reflection-probe and RT-tier items take. The
   display transform is already parameterised on the tone curve rather than hard-wired to
   gamma-sRGB, so the HDR encode lands as a new final-encode branch behind the existing pass,
   not a rewrite. **Revisit trigger:** when an HDR presentation target is available to verify.
5. **Color grading**: ✓ **Shipped.** White balance (temperature/tint), lift/gamma/gain,
   contrast, and saturation, all inside the one display-transform pass (grade → map → encode).
   A 3D-LUT slot is reserved on the settings but its texture binding is a later increment.
6. **Blue-noise dither**: ✓ **Shipped (early, 2026-07).** `tonemap.frag` applies a
   triangular-PDF dither of one LSB before the UNORM write, static in screen space since the
   temporal resolve has already run. Upgrading the source to the baked blue-noise texture is
   the residual, in step with §5.1.
7. **DoF + motion blur**: ✓ **Shipped.** `DofPass` (gather bokeh from a thin-lens circle of
   confusion, `dof.frag`) and `MotionBlurPass` (a velocity gather, `motion_blur.frag`), both
   after the temporal resolve, both tier-gated to High/Ultra and off by default. Each hands
   its output to the next stage through the `post_color` chain the view builds, so the tone
   map reads the last effect that ran without the passes naming one another.
8. **Vignette / chromatic aberration / film grain**: ✓ **Shipped**, folded into the display-
   transform pass; grain is applied in display-linear space before the dither. Lens flare is
   the residual (a bloom-derived ghost/halo pass).
9. **Post-Process window** (§2.4): ✓ **Shipped.** A dedicated `draw_post_process_panel`
   authors the whole `RenderSettings::post` block — exposure mode + parameters, tone curve,
   bloom, grade, DoF, motion blur, and the lens effects — with a "Tier resolves to" readout,
   persisted through the preferences JSON like the rest of `RenderSettings`. The blendable
   spatial volume stack over the same block is the residual.

*SOLID:* every effect is a pass or a parameter of the display-transform pass; the block the
passes read is pure data the editor resolves into (no pass reads the editor).

### Phase 10 — GPU-driven geometry — **shipped (core)**

Removes the per-object CPU wall before scene density grows into it. The core landed — the
CPU no longer issues one draw per object. `InstanceSystem` packs every opaque mesh instance
into a `GpuInstance` storage buffer grouped by mesh into per-mesh buckets; `CullPass` runs
before the depth prepass and frustum-, screen-coverage-, and occlusion-culls each instance,
compacting survivors per bucket and writing one `VkDrawIndexedIndirectCommand` each;
`mesh_gpu.vert` reads the instance record (set 2) in place of the classic push constant and
draws indirectly. **Shipped:** instancing + MDI (10.1), single-phase GPU occlusion culling
(10.2), screen-coverage LOD (10.3), a `VK_EXT_mesh_shader` meshlet path (10.4, device+Ultra gated
with classic fallback), and GPU cloth triangulation (10.6) — the "remove the per-object CPU wall"
core plus compute-driven soft-body triangulation and a mesh-shader draw path.
**Deferred with rationale:** sparse virtual texturing (10.5).
The whole path is two-path: GPU-driven when the tier permits
(`gpu_driven` — off on Low, on for Medium/High/Ultra), the author's `GpuCullingSettings.enabled`
is set, the bindless heap is present, and nothing is selected; classic CPU per-instance draw
otherwise. The editor's **GPU Culling** panel authors `RenderSettings::gpu_culling`.

1. **Instancing + multi-draw-indirect**: ✓ **Shipped** as per-mesh-bucket MDI. `InstanceSystem`
   packs per-instance records into one storage buffer (transform, bounding sphere, and the
   material/motion/pick indices — the SoA shapes already exist), grouped by mesh into buckets;
   one `vkCmdDrawIndexedIndirect` per bucket, the instance count GPU-decided. CPU cost flat in
   the distinct-mesh count, not the instance count. The single-call-across-all-meshes variant
   (`vkCmdDrawIndexedIndirectCount` over a pooled mega vertex/index buffer) is deliberately
   **not** done — it would need a `MeshRegistry` rewrite; the per-mesh-bucket form is the
   shipped design and the win (flat CPU cost) is already realized.
2. **GPU occlusion culling**: ✓ **Shipped** as single-phase. `cull.comp` tests each instance's
   bounding sphere against the **previous frame's** max-Z depth pyramid — reprojected with the
   previous view-projection and the eye delta — and drops what is hidden. `OcclusionPass` owns
   that pyramid: a persistent farthest-depth mip chain built after the depth prepass, the
   conservative twin of the Phase 5 hi-Z (nearest-depth is right for reflection marching and
   wrong for culling, so this is a *new* pyramid, not the reused one the original plan named).
   No readback, no popping. The two-phase re-test (phase 1 draws last-frame-visible + tests the
   rest against the prev-frame HZB, phase 2 retests false negatives against the current HZB)
   that removes the one-frame disocclusion latency is ○ **deferred** as a documented refinement.
   Per-view culling of shadow cascades, atlas tiles, and probes (§2.3) rides the same mechanism
   when those views adopt it.
3. **GPU LOD selection**: ✓ **Shipped** as screen-coverage culling, folded into the cull — an
   instance whose projected on-screen diameter is below an authored pixel threshold
   (`min_screen_diameter`) is dropped (LOD = "not drawn"). Discrete mesh-LOD swapping (multiple
   LOD meshes, hooking into the body-LOD ladder so planet-to-space reuses one system) and
   HLOD/impostors for far clusters are ○ **extensions** that reuse the same bucket mechanism
   once LOD meshes are authored.
4. **Meshlets** *(tier)*: ✓ **Shipped** as a `VK_EXT_mesh_shader` (task + mesh) draw path. Every
   mesh is meshletised at import/init — greedy clustering into ≤64-vertex / ≤124-triangle meshlets,
   each with a bounding sphere and a normal cone (`geometry/meshlet.{hpp,cpp}`). When the tier is
   **Ultra** and the device offers mesh shaders (`VulkanDevice::supports_mesh_shader()`, detected at
   runtime), a task shader (`meshlet.task`) frustum-culls each mesh's clusters and a mesh shader
   (`meshlet.mesh`) emits the survivors camera-relative into the shared `pbr.frag` — used in **both**
   the depth prepass and the opaque pass so culling stays consistent. The pipeline factory gained a
   `create_mesh` path (task+mesh+fragment, no vertex input) and `QualityParams` gained an Ultra-only
   `meshlets` flag. It is purely additive and device-gated: where mesh shaders are absent, the tier is
   below Ultra, or an object is selected, the same geometry falls back through the GPU-driven MDI or
   classic path (the layering is meshlets → GPU-driven MDI → classic), so no hardware is left unable
   to render. The task shader currently does per-meshlet **frustum** culling only; per-meshlet
   hierarchical-Z (HZB) occlusion culling plus normal-cone back-face culling (the latter needs
   single-sided geometry), and fully GPU-driven indirect mesh tasks (`vkCmdDrawMeshTasksIndirectEXT`
   in place of the current one-`drawMeshTasks`-per-instance CPU loop), are ○ **deferred** refinements.
   `VK_EXT_device_generated_commands` is adopted only if pass-bucket switching shows up in profiles.
5. **Sparse virtual texturing** for terrain/large sets: ○ **Deferred** — a whole subsystem
   (software SVT with a shader-written feedback buffer since Vulkan has no sampler feedback,
   a page cache, and async transfer-queue uploads), out of this increment's scope.
6. **Cloth on GPU**: ✓ **Shipped** as compute-driven soft-body triangulation. The host uploads
   only particle positions (in a strand-local frame with a per-strand camera-relative origin so
   planet-scale precision survives); `cloth.comp`, driven by a new `ClothPass`, computes the
   area-weighted vertex normals — reproducing the exact CPU triangle winding — and writes the
   drawable `MeshVertex` and index buffers the opaque pass then draws. `ClothBuffers` was
   reworked into a host-visible positions buffer plus device-local compute-written vertex/index
   buffers, with a `prepare()` that packs positions and lays out per-strand ranges; no per-vertex
   float work happens on the CPU anymore.

*Tier:* meshlets, SVT residency budget, HLOD distance. *SOLID:* culling writes
draw args; passes consume them — the draw loop stops being renderer code and
becomes GPU data (the GPU-driven mandate realized).

### Phase 11 — Upscaling, async compute & frame delivery — **shipped (core)**

The frame now reaches the GPU on two queues and the display on the host's terms, and
reconstruction became an interface rather than a hard-wired pass. **Shipped:** the
upscaler seam (11.1), async compute with graph-derived cross-queue sync (11.2), timeline
semaphores + configurable frames in flight + present pacing (11.3), and the engine half
of the runtime interop seam (11.4). **Deferred with rationale:** the vendor upscaler
backends and the runtime-side SYCL import — both blocked on something outside this
repository, recorded below rather than stubbed.

1. **Upscaler abstraction**: ✓ **Shipped (the seam), vendor backends deferred.**
   `render/frame/upscaler.hpp` names the contract — `UpscaleInputs` (colour, depth,
   velocity, history, output, the scene/temporal blocks, both extents, jitter, exposure,
   and a history-reset flag) and `IUpscaler`, whose implementations register their own
   graph passes so a three-dispatch backend and a one-draw backend look identical from
   outside. `TaaPass` was refactored to *be* that first implementation rather than a pass
   the frame loop calls by name: its `register_pass` now builds `UpscaleInputs` from the
   frame and calls its own `register_upscale`, which is exactly what a vendor backend would
   be handed. `UpscaleMode` gained `Fsr`/`Dlss`/`Xess`, and
   `Frame::upscaler_availability` reports per backend whether this build carries its SDK;
   one it does not resolves back to `Temporal` with the reason surfaced in the editor, so a
   project may author a vendor upscaler unconditionally and a build without it degrades
   instead of failing. **Deferred — rationale recorded:** none of FSR 3.1, DLSS/Streamline,
   or XeSS is vendored. Each ships as a separate redistributable under its own licence, and
   pulling one into the dependency set is a project decision rather than a renderer default;
   integrating one is also the kind of work whose correctness is only confirmable on screen
   (ghosting, disocclusion, the reactive mask), which this increment could not verify.
   Camera-relative motion vectors, the jitter sequence, and the exposure input — the three
   things a vendor upscaler is fussy about — are already in the contract, so adoption is a
   new file behind `IUpscaler` and a `SUSHI_WITH_*` definition, not a change to the frame
   loop. Frame generation stays interface-reserved. **Revisit trigger:** when a target
   platform makes one backend the obvious floor, or when latency plumbing for frame
   generation exists.
2. **Async compute queue**: ✓ **Shipped.** `VulkanDevice` now finds a compute queue whose
   *family* is distinct from graphics (the separate-family query, not vk-bootstrap's
   "dedicated" one — dedicated also excludes a family that can transfer, and every real
   async-compute family can, so AMD's compute rings were being rejected). The graph gained
   `PassQueue`: a pass declares `AsyncCompute`, `compile()` splits the schedule wherever the
   queue changes into `Submission`s, and each submission's wait is **derived from the same
   resource declarations the barriers come from** — the latest earlier submission on the
   *other* queue that produced what it consumes or consumed what it overwrites. Same-queue
   dependencies need nothing (submission order), and because a queue's timeline values rise
   with its submissions, one wait covers every earlier one. The same walk marks which
   resources both queues touch (`TextureDesc::cross_queue`), and *only those* are allocated
   concurrent: the graph cannot transfer queue-family ownership, and paying for concurrent
   sharing on every transient would cost attachment compression for nothing. Barrier
   emission became queue-aware — a resource last touched by the other queue has its source
   scope dropped, because those stages are not even nameable on this queue (a compute buffer
   cannot wait on the fragment-test stage a depth prepass wrote in) and the submission
   timeline has already ordered it. **Flagged today:** the clustered **light cull** (its
   inputs are host-written, so it starts immediately and overlaps the depth prepass and the
   cascades) and the **GTAO horizon march** (the heaviest compute pass, overlapping the
   punctual shadow atlas). Tier-, device-, and author-gated; absent any of the three every
   pass records on the graphics queue exactly as before. **Remaining:** the LUT stack, the
   fog volume, the GI relight, and the histogram are *not* flagged, because each
   hand-barriers resources it owns and the cross-queue wait is derived from declarations —
   importing those resources into the graph is the prerequisite, and is the next increment
   here. Graph-derived queue-family **ownership transfers** (which would remove the
   concurrent-sharing cost entirely and let an attachment cross queues) are the other
   documented refinement.
3. **Timeline semaphores + frame pacing**: ✓ **Shipped.** The per-slot binary fence is gone.
   Each queue carries one monotonic timeline semaphore; every submission signals its own
   value and waits on the value it depends on, and a slot is reusable once both timelines
   have reached what that slot submitted (`vkWaitSemaphores` on both, in place of
   `vkWaitForFences`). Command buffers are handed out **per submission** rather than per
   slot — a frame that compiles to several submissions would otherwise record two of them
   into one buffer, which is exactly the bug validation caught. Frames in flight is now a
   setting (2 or 3, capped to 2 on Low) with every slot allocated up front, so changing depth
   costs an idle and no reallocation; present pacing (V-Sync / Mailbox / Immediate) is a
   setting applied through `IWindowRenderer::set_present_mode`, which rebuilds the swapchain
   only on an actual change. A **latent bug this surfaced**: the `timelineSemaphore` feature
   had never been enabled at device creation — core since 1.2 and mandatory at the 1.4 floor,
   but still a feature that must be asked for, and nothing had used one until now.
   **Remaining:** an explicit submit-as-late-as-possible latency mode, and per-queue present
   timing (`VK_GOOGLE_display_timing`).
4. **SushiRuntime SYCL interop**: ◐ **Engine half shipped; the runtime half is blocked.**
   `include/SushiEngine/render/interop.hpp` plus `render/interop/vulkan_interop_buffer.cpp`
   allocate a device-local buffer whose memory another API can import by OS handle: a
   *dedicated*, exportable allocation (the handle names the whole allocation, so a
   suballocated block cannot be exported), created concurrent across both queue families,
   and stamped with the device UUID that `RenderDeviceDesc::required_uuid` already selects
   the graphics device by. The public header carries no Vulkan, SYCL, or platform type — the
   one translation unit that pulls in the Win32 Vulkan header is the implementation, which is
   also why `VulkanDevice` exposes only `supports_external_memory()` and not the handle
   getter. **Blocked, not stubbed:** the importing half has to live where the SYCL device
   does, and SushiRuntime's memory surface (`usm_allocator.hpp`, `memory_pool.hpp`) has no
   external-memory import today. SushiEngine depends on SushiRuntime and never the reverse,
   so the engine cannot add it; `sushi_render` also links no runtime and compiles no SYCL by
   design. **Revisit trigger:** when SushiRuntime grows an import-external-memory entry point,
   at which point the consumer is a sim-side call, not a renderer change.

*Verification note:* this phase is where `vulkan-validationlayers` entered the engine's
`sushistack.deps.toml`. Every claim above was checked against a live run with the layer
loaded, diffed against a same-build baseline with async compute off, so "no new validation
errors" means exactly that and not "none were reported". The pre-existing errors that diff
surfaced (the LUT/fog volumes sampled in `GENERAL`, a `levelCount` of zero, an unwritten
temporal descriptor) are real and predate this phase; they are logged for the increment that
imports those hand-barriered resources into the graph.

### Phase 12 — The ray-tracing tier & many-light scaling *(Ultra)* — **12.3 Tier A shipped**

Everything here drops into seams earlier phases built; nothing forks the pipeline. **Shipped:**
the Tier A half of stochastic direct lighting (12.3) — the one item of this phase that does not
need ray-tracing hardware, and the one that carries the phase's actual scaling win. **Deferred
with recorded rationale:** the three ray-query items (12.1, 12.2, and 12.3's Tier B), because
the development device offers no ray query and shipping three unrunnable features would be the
opposite of the posture this document takes everywhere else.

1. **RT probe feeding**: ○ **Deferred — rationale recorded.** Phase 6 Tier B matured:
   ray-query probe relight with a SHaRC-style world-space radiance cache, multibounce via cache
   feedback. Every seam it needs exists — `IProbeTracer` is the strategy interface, the
   BLAS/TLAS is built per frame, and a new tracer is a new file. What is missing is hardware:
   the development GPU (Pascal) reports no `VK_KHR_ray_query`, so the tracer could be compiled
   but not executed, let alone compared against the SDF tracer it would replace. **Revisit
   trigger:** an RT-capable development device.
2. **RT reflections**: ○ **Deferred — same rationale.** Replace SSR *misses* only (the hybrid
   pattern), denoised by an NRD-class library rather than a hand-written SVGF. Two blockers, not
   one: the hardware above, and NRD itself, which is an external SDK this build does not vendor
   — the same "a redistributable under its own licence is a project decision" line the vendor
   upscalers sit behind (§11.1).
3. **Stochastic direct lighting** (MegaLights-shaped): ◐ **Tier A shipped; Tier B deferred.**
   The shipped half is the one the plan already scoped for non-RT hardware — "visibility by ray
   query (Tier B) or **SDF trace (Tier A high)**". A pixel's cluster lights split in two: those
   holding a punctual shadow-atlas tile are filtered against it exactly as before, and those
   beyond the atlas budget — previously shaded fully unshadowed — are importance-sampled. Each
   pixel draws a tier-scaled number of them proportionally to `luminance × falloff × cosine`
   (stratified, frame-advanced so consecutive frames pick differently), marches the GI distance
   clipmap toward each pick with `sdf_visibility()`, and weights by `1/(K·p)`. The estimator is
   unbiased, so the existing temporal resolve — no new denoiser — averages it toward shadowing
   every light. **This is the phase's actual win:** the shadowed-light ceiling stops being the
   atlas tile count (4–16 by tier) and becomes a per-pixel sample budget that does not grow with
   the light count. Plumbing worth recording: the per-frame push set was already full at its
   guaranteed 32 bindings, so the field reaches every shading pipeline through a new *volume*
   array on the bindless heap (one registration for an image created with the device), and the
   field's placement rides spare lanes of the cluster block rather than a binding of its own.
   Tier: one sample on High, two on Ultra, zero below — where there is no GI field to march
   anyway. **Remaining:** Tier B visibility (a ray query in place of the march, on the same
   estimator — a one-function change once hardware exists), a reservoir carried across frames
   (ReSTIR-lite) instead of independent per-frame draws, and specular light sampling (the
   estimator is diffuse-weighted today).
4. **RT sun shadows**: ✓ already shipped (Phase 2); folding its denoise into an NRD integration
   waits on the same library decision as 12.2.

*Verification note:* the shipped half was checked with the validation layer loaded and diffed
against a same-build baseline — no new validation errors, no device loss, and the estimator
confirmed to execute (field registered in heap slot 0, one sample per pixel, four non-atlas
lights in the cluster). Its *look* — how quickly the noise resolves under motion, how the
penumbra reads — is not verified: that needs an authored many-light scene and a pair of eyes on
the image, and is the first thing to check when this is next opened.

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
