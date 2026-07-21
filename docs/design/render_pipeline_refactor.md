# Render Pipeline Refactor — Toward an AAA, Performance-First Renderer

Status: **Living design document.** Phases 0, 1, 2 and 5 are **shipped and verified
against source** (see §3 — Completed). **Phase 3 is in progress:** items 3.1 (the tier
contract / `QualityParams`), 3.2 (`VulkanSceneView` → `ViewResources` extraction), and
3.7 (IBL diffuse → SH-9) are **shipped**; 3.3–3.6 remain. This revision re-audits the codebase, folds in
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
3. **PSO hitch elimination**: background link-time-optimized pipeline recompile +
   swap on top of the existing GPL fast link; pipeline usage harvesting to a
   precache list warmed at startup (the Khronos/Epic-documented recipe). Target:
   zero first-use hitches in a captured flight.
4. **Vulkan 1.4 floor** (drivers are conformant industry-wide since 2025): promote
   maintenance5/6, `host_image_copy` for texture streaming, push descriptors;
   delete the fallbacks they obsolete.
5. **Binding-layer future-proofing**: isolate every descriptor-set touch behind the
   existing heap/allocator seams so the announced `VK_EXT_descriptor_heap`
   (Roadmap 2026) is a backend swap, not a refactor. Do **not** adopt
   `VK_EXT_descriptor_buffer` (dead end).
6. **Shading-language decision**: evaluate migrating `render/shaders/` to **Slang**
   (Khronos-hosted, Valve-shipped, generics/modules, SPIR-V-first) while the shader
   count is still ~30. If adopted, the hot-reload path and build tool compile Slang;
   GLSL includes are ported module-by-module. If deferred, record why — the cost
   only grows.
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
decision. New module `render/lighting/`.

1. **Light list**: point / spot / directional-secondary / area (rect, tube) lights
   as engine objects — color, intensity in physical units (lumen/candela; the sun
   stays lux), range by inverse-square with a windowed falloff, spot cones, IES
   profiles and cookies (bindless textures). CPU side is a plain SoA; GPU side one
   storage buffer.
2. **Froxel cluster grid** (compute): view-frustum-aligned, ~16×9×24 base with
   logarithmic Z slicing (matches the aerial-perspective froxels of Phase 7 so the
   two share addressing), per-cluster light index list built each frame. Budget:
   ≤0.3 ms at 1080p internal for 1k lights.
3. **Forward+ shading path**: `pbr.frag` gains a cluster fetch + light loop with
   the existing BRDF; specular occlusion and contact shadows apply per light where
   meaningful. Sun path unchanged.
4. **Per-light shadows**: one shared shadow **atlas** (like the CSM atlas —
   one image, one pass, one profiler entry), quadtree-allocated tiles sized by
   screen coverage, dormant-light caching (a static light's tile persists until it
   or its casters move), spot = 1 tile, point = 6 or DPCF-paraboloid under tier.
   PCF now; PCSS on High+.
5. **Clustered decals**: project into the same froxel grid; sample in the opaque
   pass before shading (albedo/normal/roughness overrides).
6. **Editor Lighting window** (§2.4): sun/ephemeris driver, environment/IBL
   source, shadow settings, and the punctual-light list with per-light everything.
7. **Emissive → bloom seam**: emissive intensity already exists; verify HDR range
   survives to Phase 9's bloom threshold.

*Performance notes:* clustered culling is the flat-cost foundation everything
reuses; the atlas + caching keeps worst-case shadow cost bounded by tile budget,
not light count. Tier: light count ceiling, atlas size, PCSS on/off.
*SOLID:* lights are data; culling, atlas, and shading depend on the grid and the
list, never on each other (ISP). `ILightSource` never appears — lights are not
polymorphic objects, they are records (data-oriented mandate).

### Phase 5 — Screen-space lighting quality: GTAO + SSR

The two biggest per-pixel realism wins after shadows, both bounded-cost.

1. **Blue-noise infrastructure first**: a shared spatiotemporal blue-noise
   texture/sequence (per-frame offset, TAA-aware) as a `render/resources/` asset —
   GTAO, SSR, contact shadows, dither, DoF, and the cloud march all consume the
   same source (SRP; kills the per-pass ad-hoc hashes).
2. **GTAO** (half-res, compute, async-ready): horizon-based with a **bent normal +
   visibility cone** output; spatial denoise + the TAA accumulates the rest.
   Feeds: diffuse AO (multiplies IBL/GI diffuse), **specular occlusion upgraded**
   from the material-AO approximation to bent-normal cone vs reflection cone.
   Budget: ≤0.5 ms at 1080p internal on the mid tier.
3. **Stochastic SSR**: hi-Z traced (needs the depth pyramid — build it here, it is
   also Phase 10's culling input; shared infrastructure, built once), GGX-jittered
   ray from the blue-noise sequence, half-res trace + neighborhood reuse +
   temporal accumulation (the FFX-SSSR shape), roughness-cutoff fade to IBL.
   Fallback chain: SSR hit → nearest reflection probe → sky IBL.
   Budget: ≤1.2 ms at 1080p internal, High tier.
4. **Reflection probes**: box/sphere-projected local captures through the existing
   IBL capture path (it already renders the sky; point it at the scene), authored
   in the editor, blended by proximity, camera-relative anchored.
5. **Specular-occlusion chain audit**: one documented path — GTAO cone → bent
   normal → probe/SSR/IBL — replacing today's scalar approximation everywhere.

*Tier:* GTAO res + tap count; SSR res, max steps, roughness cutoff; probes
static-only on Low. *SOLID:* both passes read the graph's depth/normal/velocity
handles; neither knows the other exists; the fallback chain is data (a priority
list), not branching code.

### Phase 6 — Global illumination (the 2025 baseline, no RT required)

Flat sky ambient + AO is not AAA. The industry-settled mid-size pattern (Lumen-SW /
Brixelizer / AC-Shadows-shaped): **probe volumes fed by a pluggable tracer, cached
aggressively, refined per-pixel by Phase 5's screen-space terms.** New module
`render/gi/`.

1. **Irradiance probe clipmaps**: 3–4 cascaded volumes around the camera
   (camera-relative by construction — the planet-scale answer), SH-9 or
   octahedral-visibility probes (DDGI-style with depth for leak rejection),
   ~(32³–48³) probes per cascade, sparse relight (N probes per frame, dirty-first).
2. **Pluggable tracer behind `render/gi/tracer.hpp`** (DIP — the strategy seam):
   - **Tier A (all hardware)**: SDF scene tracing — per-mesh signed distance
     fields baked at import into a brick atlas + a coarse global clipmap SDF;
     cone/sphere tracing against it. (The same SDF assets later accelerate the
     cloud march and Phase 12's stochastic visibility — shared, not bespoke.)
   - **Tier B (Ultra)**: ray-query against the Phase 2 BLAS/TLAS, same probe
     consumers, zero shading-path fork — exactly the RT-shadow pattern.
3. **Sky/ground handoff**: probes integrate the Hillaire sky (Phase 7 LUTs) and
   the analytic planet ground as their environment miss — GI, sky, and IBL agree
   by construction; the IBL specular cube remains the distant-specular source.
4. **Radiance-cache reuse for reflections**: SSR/probe misses fall back to probe
   radiance (the "one world cache serves GI and reflections" pattern) before sky.
5. **Emissive injection**: emissive surfaces contribute via the tracer (SDF albedo
   atlas / ray-query hit shading) — lights-from-materials for free at probe rate.

*Performance:* probe relight is compute, async-queue-ready, amortized (nothing
per-pixel except the final trilinear+SH gather ≈0.3 ms); cost is flat in scene
complexity. This is deliberately **not** per-pixel ReSTIR GI — probe caches need
no denoiser (the survey's consistent conclusion for mid-size engines).
*Tier:* cascade count/extent, probes-per-frame, tracer tier, bounce count (1 now,
multibounce via cache feedback on High+).

### Phase 7 — Atmosphere: the LUT stack + froxel fog

Replaces the full-res per-pixel march — a large speedup *and* multiple scattering
at once. The performance-first mandate's flagship.

1. **Hillaire 2020 LUT stack** (the unchallenged production SOTA): transmittance
   LUT (256×64), multi-scatter LUT (32×32), per-frame **sky-view LUT** (192×108
   lat-long, marched once at low res), **aerial-perspective froxel volume**
   (32×32×32, camera-relative, same Z slicing as the light grid). `sky.frag`
   becomes LUT lookups + the sun disk/eclipse/star work; meshes read aerial
   perspective from the froxel volume. WGS84 ellipsoid handling and the eclipse/
   phase geometry carry over unchanged. Budget: the whole stack ≤0.4 ms/frame vs
   multiple ms today; sky-view LUT amortizable over 2 frames under tier.
2. **Froxel volumetric fog**: reuses the aerial froxel addressing and the Phase 4
   cluster grid — sun (CSM-shadowed, cloud-shadowed) + punctual lights in-scatter
   into a 3D scattering/transmittance volume, temporally jittered and accumulated,
   height-fog term analytic. God rays, lit fog, aerosol haze — one pass, one
   volume. Budget: ≤0.6 ms at 1080p, High tier.
3. **Local fog volumes**: authored density primitives (box/ellipsoid, blend into
   the froxel grid) for valley fog / airfield ground fog — data, not new passes.
4. **Cloud coupling seam**: the LUT stack exposes sun transmittance and sky
   radiance at arbitrary height — the exact inputs the companion cloud plan's
   lighting model consumes (§weather doc). Land the seam here so clouds plug in
   without touching the atmosphere again.

*Tier:* LUT resolutions, froxel depth resolution, fog on/off at Low.
*SOLID:* LUT generation, sky composite, and fog are three passes sharing two
resource handles; the ephemeris keeps driving inputs through `Environment`
unchanged (OCP over the existing seam).

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
   consumes Phase 5's shared blue-noise.
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
