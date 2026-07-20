# Render Pipeline Refactor — Toward an HDRP-Class, Performance-First Renderer

Status: **Proposal / design**. This document is the engineering plan for evolving
`render/` from its current offscreen forward pass into a modular, HDRP-class
pipeline. The guiding constraint is the *red line between maximum realism and
performance*; where the two conflict, **performance wins**. Every technique below
carries a quality tier so the expensive half is opt-in per platform.

---

## 0. North star

- **Physically-based, temporally-stable, HDR end to end.** No banding, no shimmer,
  no LDR intermediate until the final display encode.
- **Data-oriented and GPU-driven.** The CPU describes *what* to draw; the GPU decides
  *which* to draw and submits its own work. Draw count must not scale the CPU cost.
- **A render graph, not a call sequence.** Passes declare reads/writes; barriers,
  aliasing, and scheduling are derived, never hand-written. This is the SOLID spine.
- **Planet-scale correct.** Camera-relative rendering + reverse-Z are already in
  place and are kept as invariants — every new pass inherits them.
- **Runtime-interop first.** Heavy precompute (noise, LUTs, culling, particle sim)
  runs on the GPU / SushiRuntime SYCL graph, not `std::thread` on the CPU.

---

## 1. Current state — honest audit

The current renderer (`render/rhi/vulkan/vulkan_scene_view.cpp`, ~2300 lines) is a
single class that owns device resources, generates noise on the CPU, records four
passes by hand, and resolves picking. It is competent and already does several things
right, but it is a monolith and is missing most of the modern lighting/AA/post stack.

### 1.1 What is already good (keep and build on)

| Area | Current state |
|---|---|
| Precision | Camera-relative rendering; eye subtracted in double before the float cast (`make_push`, UBO fill). **Correct at planet scale.** |
| Depth | Reverse-Z, `D32_SFLOAT_S8_UINT`, `GREATER_OR_EQUAL`. **Correct.** |
| HDR intermediates | Scene + composite are `R16G16B16A16_SFLOAT`. |
| Sky/atmosphere | Ray-marched single-scattering Rayleigh+Mie, quadratic step distribution, analytic ellipsoid ground, eclipse/phase geometry, star field. Physically motivated. |
| Clouds | Nubis-style Perlin–Worley volumes, WMO genus taxonomy, single-medium multi-deck march. Genuinely advanced. |
| API level | Vulkan 1.3 dynamic rendering + synchronization2 + VMA + timeline-ready. Modern baseline. |
| Tonemap | ACES filmic + gamma encode in a dedicated pass. |

### 1.2 What is missing or weak (the refactor targets)

**Lighting & shading**
- Single directional light only. **No shadows of any kind** (no CSM, no contact,
  no ray-traced). This is the largest single realism gap.
- **No punctual/area lights**, no clustered/tiled light culling — the scene cannot
  scale past one light.
- **No IBL.** Ambient is a flat constant `vec3` (`scene.ambient.xyz * albedo`); there
  is no prefiltered environment, no irradiance, no BRDF LUT, no reflection probes.
- **No SSAO/GTAO, no SSR.** Contact darkening and glossy reflections are absent.
- BRDF is single-scatter Cook-Torrance GGX; **no multi-scatter energy compensation**
  (rough metals lose energy), no anisotropy/clearcoat/sheen/subsurface.

**Geometry & materials**
- Vertex format is position+normal only (24 B). **No UVs, no tangents, no vertex
  color** → normal maps, detail maps, and albedo textures are impossible today.
- **No textures at all**, no material asset, no texture streaming.
- **No mesh loading** (glTF); only three procedural primitives.
- **No instancing / indirect draw / GPU culling.** One `vkCmdDrawIndexed` per object,
  push-constant per draw → CPU-bound above a few thousand objects.
- Cloth is CPU-triangulated and re-uploaded **every frame**.

**Anti-aliasing & temporal**
- **No anti-aliasing whatsoever** (`rasterizationSamples = 1`, no TAA/FXAA/SMAA).
  Edges and the volumetric clouds shimmer. **No motion vectors** exist, which blocks
  TAA, motion blur, and temporal cloud reprojection.

**Post-processing**
- Only ACES + gamma. **No bloom, no auto-exposure** (exposure is a fixed `0.18`),
  no DoF, no motion blur, no color grading/LUT, no vignette/CA/grain, no lens flare.
- **No dithering before the 8-bit quantize** → visible banding on the sky gradient.

**Atmosphere & clouds (quality/perf)**
- Atmosphere is marched **per pixel, full-res, every frame** with no precomputed LUTs
  → expensive and single-scatter only (no multiple scattering, sky is too dark).
- Clouds are half-res but the tonemap upsample is a **naive bilinear** `texture()`
  fetch — not depth-aware/bilateral → halos at silhouettes. No temporal reprojection
  or blue-noise jitter accumulation → the 32–96-step march is noisy under motion.

**Architecture**
- One ~2300-line class = SRP violation. Barriers are hand-wired and conservative
  (full `FRAGMENT_SHADER` stage everywhere).
- **No render graph, no frame graph, no pass abstraction.** Adding a pass means
  editing the god-class and threading new descriptor bindings through
  `create_targets`.
- **No pipeline cache**, no descriptor indexing/bindless, no shader hot-reload,
  single graphics queue, no async compute, no GPU timing/instrumentation.
- One giant `SceneUbo` re-uploaded in full each frame; descriptor sets rebuilt on
  every resize.

---

## 2. Target architecture

```
                    ┌─────────────────────────────────────────────┐
                    │              RenderGraph                     │
                    │  (declares passes, derives barriers/aliasing,│
                    │   schedules graphics + async-compute queues) │
                    └─────────────────────────────────────────────┘
   FrameContext ──▶  passes register into the graph each frame:

   [GPU 2-phase cull/HZB] → [Depth prepass] → [Clustered light+decal cull]
        → [GBuffer OR Forward+] → [Shadows: CSM/contact/RT] → [SSAO/GTAO]
        → [Sky-view+aerial LUTs] → [Lighting + IBL + decals] → [SSR]
        → [Volumetric fog/lighting] → [Volumetric clouds (½-res, temporal)]
        → [Transparents (Forward)] → [TAA resolve] → [Auto-exposure] → [Bloom]
        → [DoF/MotionBlur] → [Tonemap + grade + dither] → [FXAA/output]
        → [ImGui / present]

   (VRS mask + dynamic-resolution scale wrap the shading-heavy passes; every pass is
    scheduled across graphics + async-compute queues by the graph.)
```

The pipeline splits into **library modules** behind narrow interfaces, so a pass is a
unit that can be added, reordered, or A/B-tested without touching its neighbours:

- `render/graph/` — `RenderGraph`, `RenderPassNode`, `ResourceHandle`,
  transient-resource allocator with memory aliasing, automatic barrier insertion from
  declared access, queue scheduling. **This lands first; everything else is a client.**
- `render/frame/` — `FrameContext` (per-frame camera, jittered matrices, prev-frame
  matrices for motion vectors, exposure state, quality tier).
- `render/passes/` — one file per pass (`depth_prepass`, `gbuffer`, `shadow_csm`,
  `gtao`, `lighting`, `ssr`, `atmosphere_lut`, `clouds`, `taa`, `bloom`, `tonemap`, …).
- `render/resources/` — `TexturePool`, `BufferPool`, `PipelineCache`, `SamplerCache`,
  bindless descriptor heap.
- `render/material/` — `Material`, `TextureAsset`, `MeshAsset`, glTF import.

`VulkanSceneView` becomes a thin orchestrator: build `FrameContext`, let each pass
register into the graph, compile, submit. Target: **< 300 lines.**

### 2.1 Lighting architecture decision

Adopt a **hybrid**: a **depth prepass + clustered Forward+** as the default (MSAA/TAA
friendly, cheap for a flight-sim's mostly-opaque scenes, no fat GBuffer bandwidth at
planet-scale resolutions), with an **optional thin GBuffer** path enabled by the
quality tier for scenes that want deferred decals/SSR fidelity. Clustered light
culling (froxel grid, compute) is shared by both. This keeps the performance-first
promise: Forward+ has the lower memory/bandwidth floor, which is where we lean.

### 2.2 Material model (Unity-Standard parity, extended)

The material is a **GPU struct** (SoA in a storage buffer, indexed per instance so it
is GPU-driven friendly) plus a set of **bindless texture indices**. All maps are
optional — a null index means "use the scalar/constant". Every map samples through a
**per-material main `tiling (xy)` + `offset (xy)`** (the Unity ST vector), with the
detail set carrying its own tiling/offset. This is the headline authoring surface and
must be first-class.

**Core PBR inputs (metallic-roughness, glTF-native; spec-gloss import-converted):**

| Slot | Map | Scalars / tint | Notes |
|---|---|---|---|
| **Albedo** | base-color (sRGB) + alpha | `base_color` tint (RGBA) | alpha drives cutout/transparent |
| **Metallic-Roughness** | packed MR (or ORM) | `metallic`, `roughness` (or `smoothness`) | glTF packs R=occl?, G=rough, B=metal; support ORM single-map |
| **Normal** | tangent-space normal | `normal_scale` (bump strength) | needs tangents (Phase 1.1); reconstruct Z |
| **Height** | height/displacement | `height_scale`, POM step count | **Parallax Occlusion Mapping** (self-occlusion + optional contact shadow), silhouette-clip toggle |
| **Occlusion** | AO | `occlusion_strength` | multiplies indirect diffuse; feeds specular occlusion |
| **Emission** | emissive (sRGB) | `emissive_color` (HDR), `emissive_intensity`, `emissive` toggle | unlit radiance, tonemapped; drives bloom |

**Detail (secondary) set — Unity-style:** `detail_albedo`, `detail_normal`,
`detail_mask`, with an independent `detail_tiling/offset`, blended over the base for
close-up micro-detail without huge base textures.

**Advanced lobes (quality-tier gated, off by default for perf):** anisotropy
(+ direction), clearcoat (+ roughness + normal), sheen (cloth), subsurface /
transmission (thin/skin/foliage). Each is a separate `#define` shader permutation so
the default opaque material stays cheap.

**Rendering state (per material, Unity-parity):** `surface_type`
(Opaque / Cutout / Transparent / Fade), `alpha_cutoff`, `cull_mode`
(Off / Front / Back — i.e. double-sided), `blend_mode`, `render_queue`, receive/cast
shadow flags, GPU-instancing flag. Sampler carries anisotropic filtering level and
wrap mode.

Import path: **glTF 2.0** maps 1:1 onto the core set (metallic-roughness, normal,
occlusion, emissive) including `KHR_materials_*` extensions (clearcoat, sheen,
transmission, ior, emissive_strength) → the advanced lobes. A material inspector in
the editor exposes every field above, laid out like Unity's Standard shader: base map
+ tint, MR + sliders, normal + strength, height + amount, occlusion + strength,
emission + HDR color + intensity, detail fold-out, tiling/offset at the top.

### 2.3 Per-camera view & culling system

Each camera / `ISceneView` owns an independent **view context**: its frustum, its
**culling result**, its **post-process volume stack**, and its **quality tier**. This
lets the editor's viewport, a game camera, and reflection/shadow views each cull and
grade independently. Culling is layered, cheapest first:

1. **CPU frustum + distance/small-object cull** — coarse reject on a BVH of instance
   bounds; produces the candidate set.
2. **GPU two-phase occlusion culling (Hi-Z)** — build a depth pyramid (Hi-Z) from the
   depth prepass; **phase 1** draws last-frame-visible objects and tests the rest
   against the *previous* frame's Hi-Z; **phase 2** re-tests the false-negatives
   against the *current* Hi-Z and draws the newly-revealed ones. This is the UE5/
   "GPU-driven" pattern — no CPU readback, no popping, one indirect draw. Compute
   writes `VkDrawIndexedIndirectCommand` args directly.
3. **Per-light shadow culling** — each cascade / spot / point face culls its own set,
   reusing the same BVH.

The result feeds indirect draws (Phase 7). Because culling is GPU-side and per-view,
draw-call count stops being a CPU cost — the core performance-first promise.

### 2.4 Editor integration — Lighting & Post-Processing windows

Two dedicated editor panels back the whole feature set with data structs that map
directly onto pass parameters (added to `editor/ui/editor_panels.*`).

**Lighting window**
- **Sun/directional**: direction (or a sun-position/time-of-day driver tied to the
  existing ephemeris), color, intensity (lux/EV), angular size (soft shadows).
- **Environment / IBL**: source (procedural sky capture / HDRI / solid), intensity,
  rotation; ambient mode (flat / SH / IBL).
- **Shadows**: cascade count, max distance, per-cascade resolution, split lambda,
  depth bias/normal bias, softness (PCF/PCSS), contact-shadow toggle, RT-shadow tier.
- **Atmosphere & fog**: existing atmosphere params + volumetric fog density, height
  falloff, scattering albedo, sun/god-ray strength.
- **Punctual/area lights list**: add/remove point/spot/area/tube lights; per-light
  color, intensity, range, spot cone, shadows on/off, **cookie/IES profile**.

**Post-Processing window** (a stackable **Post-Process Volume**, Unity-post-stack /
HDRP-Volume style — settings can be global or blended by camera/trigger):
- **Auto-Exposure**: min/max EV, exposure compensation, adaptation speed, metering.
- **Tonemapping**: mode (ACES / AgX / Neutral), manual EV override.
- **White Balance / Color Grading**: temperature, tint, contrast, saturation,
  lift-gamma-gain, and a **3D LUT** slot.
- **Bloom**: threshold, intensity, scatter, lens-dirt.
- **Depth of Field**: focus distance, aperture (f-stop), focal length, bokeh quality.
- **Motion Blur**: intensity, sample count.
- **Ambient Occlusion (GTAO)**: intensity, radius, thickness.
- **Screen-Space Reflections**: quality, max distance, thickness.
- **Vignette, Chromatic Aberration, Film Grain, Lens Flare**: standard knobs.
- **Dithering**: toggle (on by default; blue-noise, pre-quantize).

Each override is toggleable; unset overrides inherit the volume below. The stack
resolves to the per-camera params the RenderGraph passes read.

---

## 3. Phased roadmap

Each phase is independently shippable and leaves the editor working. Ordering favors
(a) unblocking everything else, then (b) the biggest realism-per-cost wins.

### Phase 0 — Foundation (no visible change; enables all of the above) — **COMPLETED**
1. **RenderGraph** with automatic barriers + transient aliasing. Port the existing
   four passes onto it unchanged as the migration proof.
2. **PipelineCache** (`VkPipelineCache` on disk) + `VK_EXT_graphics_pipeline_library`
   for fast startup.
3. **Bindless** (`VK_EXT_descriptor_indexing`): one global texture/buffer heap;
   materials index into it. Kills per-slot descriptor rebuilds.
4. **GPU timing** (timestamp queries) + a per-pass profiler HUD in the editor.
5. **Shader hot-reload** (watch `render/shaders/`, recompile to SPIR-V, rebuild
   pipelines) — force-multiplier for all subsequent visual work.
6. Move cloud-noise generation off `std::thread` onto a **compute shader / SushiRuntime
   SYCL** job.

### Phase 1 — Shading core — **COMPLETED**
1. **Vertex format upgrade**: position, normal, tangent, UV0 (+ optional UV1/color).
   Regenerate primitive meshes with tangents; add a proper mesh vertex struct.
2. **Material system + textures** — the full §2.2 model: albedo, MR/ORM, normal,
   height (**parallax occlusion mapping**), occlusion, emission, detail set, all with
   per-map **tiling/offset**; sRGB vs linear handling; anisotropic sampling; mip
   generation; bindless texture indices; a GPU material buffer indexed per instance.
3. **glTF 2.0 mesh + material import** (`render/material/gltf.*`, incl. `KHR_materials_*`)
   so real assets and their PBR materials replace primitives.
3a. **Texture streaming / virtual texturing** hook so 4K+ material sets do not blow the
   VRAM budget (mip-based residency; upgrade to sparse/VT under the Ultra tier).
4. **BRDF upgrade**: multi-scatter energy compensation (Kulla-Conty), Fresnel with
   roughness, specular occlusion; optional clearcoat/sheen behind the tier.
5. **IBL**: capture/prefilter an environment (from the analytic sky!) into a
   prefiltered specular cube + irradiance SH + a shared **split-sum BRDF LUT**. Replace
   the flat ambient constant. This alone transforms material readability.

### Phase 2 — Shadows — **COMPLETED**
1. **Cascaded Shadow Maps** for the sun (3–4 cascades, stabilized, PCF/PCSS), tuned
   for planet-scale far planes (log/practical split + reverse-Z depth).
2. **Screen-space contact shadows** to recover small-scale contact the CSM resolution
   misses.
3. **Cloud → ground shadowing** already exists analytically; feed the cloud
   transmittance into the sun term properly here.
4. *(Tier: Ultra)* **Ray-traced sun shadows** via the `blas_placeholder`/TLAS path
   when `VK_KHR_ray_tracing_pipeline` is present.

### Phase 3 — Ambient occlusion, reflections, GI
1. **GTAO** (ground-truth AO) with a bent-normal output feeding both diffuse and
   specular occlusion.
2. **SSR** (hi-Z traced) for glossy screen-space reflections; fall back to IBL probes
   off-screen.
3. **Reflection probes** (box/sphere-projected) placed in the scene; blended with SSR.
4. **Specular occlusion** from GTAO bent normals + horizon-based AO on the IBL specular.
5. **Clustered decals** (projected onto the depth/GBuffer, sharing the froxel grid) for
   surface detail without extra geometry.
6. *(Tier: Ultra)* **RTGI / DDGI** or ray-traced reflections on the RT path.

### Phase 4 — Atmosphere & clouds (perf + fidelity)
1. **Precomputed atmosphere LUTs (Bruneton / Hillaire)**: transmittance LUT,
   multi-scatter LUT, per-frame **sky-view LUT** (low-res, marched once) and **aerial-
   perspective froxel volume**. The full-res per-pixel march is replaced by cheap LUT
   lookups → *multiple scattering for free and a large speedup*. Directly serves the
   performance-first mandate.
2. **Temporal volumetric clouds**: reproject the half-res march with per-frame
   blue-noise ray-offset + a history buffer (needs motion vectors from Phase 5, so
   land the reprojection scaffold here and enable when TAA arrives).
3. **Bilateral (depth-aware) upsample** for the cloud composite to kill silhouette
   halos, replacing the naive bilinear fetch in `tonemap.frag`.
4. **Volumetric fog / lighting (froxel)** — a view-frustum-aligned 3D volume that
   accumulates in-scattered light from the sun and every clustered light, integrated to
   a scattering/transmittance texture. Gives god rays, light shafts, height fog, and
   fog that punctual lights actually illuminate — one of the biggest atmosphere wins,
   and it reuses the aerial-perspective froxel infrastructure.

### Phase 5 — Anti-aliasing & temporal core — **COMPLETED**
1. **Motion vectors**: a velocity target written from current vs previous clip
   position (add prev-frame matrices to `FrameContext`; jittered projection).
2. **TAA** with neighborhood clamping, YCoCg history, and a sharpening pass. This is
   the single biggest perceived-quality jump and is *cheaper than MSAA* at these
   resolutions — the performance-first AA choice.
3. **Upscaling hook** (FSR2/XeSS-style or vendor DLSS/FSR) sharing the TAA motion
   vectors, so the internal render resolution can drop while output stays sharp — the
   ultimate performance lever.
4. Keep a cheap **FXAA** fallback for the low tier / no-history first frame.
5. **Dynamic resolution scaling** — drive the internal render scale from the measured
   GPU frame time (Phase 0 timers) so the frame budget holds under load; the TAA
   upscaler hides the resolution change. The primary automatic performance governor.
6. **Variable Rate Shading (VRS)** — shade at reduced rate where it is invisible
   (post-DoF/motion-blur regions, low-contrast interiors, far froxel fog), driven by a
   VRS mask from luminance/edge/velocity. Big fill-rate saving at planet-scale
   resolutions where `sky.frag`/`cloud.frag`/lighting are the cost.

### Phase 6 — Post-processing stack
1. **Auto-exposure** (compute histogram → adapted luminance) replacing the fixed
   `exposure = 0.18`; physical camera (EV100, aperture/shutter/ISO) optional.
2. **Physically-based bloom** (progressive down/upsample, energy-conserving).
3. **Depth of field** (bokeh) and **motion blur** (per-object + camera, from velocity).
4. **Tonemap upgrade**: AgX or Tony-McMapface alongside ACES (better hue handling),
   selectable; **display transform** aware (sRGB / Rec.2020 HDR output path).
5. **Color grading** (3D LUT), vignette, chromatic aberration, film grain, lens flare.
6. **Dithering** (blue-noise) before the 8-bit encode → removes sky banding.

### Phase 7 — GPU-driven geometry
1. **Instancing + indirect draw**: one draw per mesh type, per-instance data in a
   storage buffer. Removes the per-object CPU cost in `render()`.
2. **GPU two-phase occlusion culling** via a **Hi-Z pyramid** (the §2.3 system), fully
   per-camera and per-shadow-view; compute writes the indirect draw args — no CPU
   readback, no popping.
3. **Mesh LOD** + **HLOD/impostor** selection on the GPU; hooks into the existing
   body-LOD ladder so the planet-to-space transition reuses one LOD system.
4. *(Tier: Ultra)* **Mesh shaders / meshlets** with per-meshlet cone + Hi-Z culling.
5. Persistent cloth buffers (double-buffered, no per-frame realloc) + GPU
   triangulation.

### Phase 8 — Async & interop
1. **Async compute queue** for GTAO, clouds, bloom, culling, LUTs — overlapped with
   graphics. The RenderGraph scheduler places them.
2. **SushiRuntime SYCL interop** for particle/GPU-sim data feeding the renderer
   without a round-trip to host memory (the "interop-first" mandate).
3. **Frame pacing & low latency** — timeline-semaphore frame graph, triple buffering,
   and a Reflex-style latency-reduction (submit-as-late-as-possible) so the measured
   budget translates into steady, low-input-lag frames.

---

### Phase 9 — Additional tier-gated techniques (completeness)

Professional-pipeline features that are not on the critical path but round out parity;
each is Ultra-tier and slots into the graph without disturbing the default path:
- **Order-Independent Transparency** (weighted-blended or per-pixel linked-list) for
  correct layered glass/foliage/particles.
- **Screen-space subsurface scattering** (separable Burley) for skin/wax/foliage,
  feeding the material subsurface lobe.
- **Hardware tessellation / displacement** for true silhouette displacement where POM
  is not enough (terrain patches, close-up hero assets).
- **CAS / contrast-adaptive sharpening** after upscaling.
- **Water rendering** (screen-space + planar reflection, foam, refraction) for the
  planet ocean beyond the current analytic tint.
- **Debug/validation views** (overdraw, mip level, light complexity, VRS rate, cull
  heatmap) surfaced in the editor alongside the GPU profiler.

## 4. Cross-cutting concerns

- **Quality tiers.** A `RenderQuality` enum (Low / Medium / High / Ultra) gates the
  expensive half of every pass (RT, DDGI, mesh shaders, DoF, higher LUT/shadow res,
  internal resolution scale). The default target is **High at a locked frame budget**;
  the red line moves toward performance by dropping a tier, never by disabling a
  feature ad hoc.
- **Color management.** One documented convention: textures tagged sRGB/linear at
  import; all lighting in linear; a single display-transform at the end. Fixes the
  ad-hoc `pow(1/2.2)`.
- **Performance budget.** Per-pass GPU timings surfaced in the editor from Phase 0, so
  every subsequent pass is landed against a measured budget, not a guess.
- **Precision invariants.** Camera-relative + reverse-Z are non-negotiable; the graph
  validates that no pass reintroduces absolute world positions at float precision.

## 5. How this satisfies SOLID

- **SRP** — the god-class splits into graph, passes, resource pools, material, frame
  context; each has one reason to change.
- **OCP** — a new effect is a new `RenderPassNode` registered into the graph; existing
  passes are untouched (mirrors the existing `cloud_genus_profile` extension pattern).
- **LSP** — every pass honors the same `register(graph, frame)` contract; the RHI
  stays behind `IRenderDevice` / `ISceneView`.
- **ISP** — passes depend on narrow handles (`ResourceHandle`, `FrameContext`), not on
  the whole renderer.
- **DIP** — passes depend on graph/resource abstractions; Vulkan specifics stay in the
  `render/rhi/vulkan` implementation layer.

## 6. Suggested sequencing

Phase 0 → 1 → 5 (TAA/motion vectors, since Phase 4.2 and Phase 6 depend on them) →
2 → 3 → 4 → 6 → 7 → 8 → 9.

> **Status.** Phases 0, 1, 5 and 2 are complete and verified in the editor. Phase 3
> (ambient occlusion, reflections, GI) is next in the sequence, and inherits the
> acceleration structure Phase 2 built. Each completed phase shipped its
> CHANGELOG/ARCHITECTURE entry with it. Phases 2/3/6 can interleave once the temporal core exists;
Phase 9 items land opportunistically per demand.
Each phase ships its CHANGELOG/ARCHITECTURE entry in the same PR, per repo policy.

## 7. Biggest realism-per-cost wins (if you only do five things)

1. **IBL** (Phase 1.5) — replaces flat ambient; single largest material-fidelity jump.
2. **CSM sun shadows** (Phase 2.1) — grounds every object in the scene.
3. **TAA + motion vectors** (Phase 5) — kills all shimmer; unlocks upscaling for perf.
4. **Atmosphere LUTs** (Phase 4.1) — multiple scattering *and* a big speedup at once.
5. **GTAO + dithering** (Phase 3.1 / 6.6) — contact darkening + removes sky banding.
