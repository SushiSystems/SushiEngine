# Asset cooking and hitch-free instantiation

**Status:** No phase is built.

## 1. Purpose

Dropping a 400,000-triangle glTF into the editor scene stalls the editor for hundreds of
milliseconds. The stall is a chain, and every link runs on the main thread in the drop frame:
the file is parsed, the geometry is copied whole into GPU-visible memory, every texture is
decoded and its mip chain generated, and the first draw that needs a pipeline the disk cache has
not seen compiles it. Nothing about the asset is remembered between drops — the same file pays
the same price every time it is spawned.

Two things are missing outright, independent of the hitch: meshoptimizer appears nowhere in the
tree, so no imported mesh is vertex-cache-, overdraw- or fetch-optimized; and no level-of-detail
data exists anywhere, so every mesh draws at full triangle count at every distance.

This document designs the fix as two halves that share one artifact: **import-time cooking** to
a GPU-ready blob, and **asynchronous instantiation** that streams the blob in without ever
blocking the frame. The render optimization design (`docs/design/render_optimization.md`, written
in parallel with this one) consumes the blob's LOD chain, error metadata and shadow index buffer;
this document owns producing them.

## 2. What exists today

The import path is split across two tiers, and both halves parse the source file on the main
thread at spawn time:

- The asset tier parses with cgltf and decides what entities the file becomes:
  `engine/asset/gltf/source/scene_importer.cpp` and
  `engine/asset/gltf/source/mesh_importer.cpp` read the file, and
  `engine/asset/model/include/SushiEngine/model/instantiation_plan.hpp` turns the result into a
  device-free plan the editor's executor replays
  (`applications/editor/source/scene/scene_commands.cpp`).
- The render tier then parses the same file a second time to get geometry and materials:
  `engine/presentation/render/source/material/gltf_importer.cpp` re-reads every primitive and
  uploads it.

The uploads themselves, honestly characterized:

- **Meshes** — `engine/presentation/render/source/geometry/mesh_registry.cpp` copies full-size
  float vertices and the file's indices, unoptimized, into a persistently mapped host-visible
  buffer with one `memcpy` on the calling thread. There is no staging copy and no device-local
  placement: the GPU reads mesh geometry from host-visible memory on every frame, forever. The
  same call meshletizes the mesh (an in-house clusterer,
  `engine/domain/geometry/source/meshlet.cpp`) and bakes its signed-distance brick, also on the
  calling thread.
- **Textures** — `engine/presentation/render/source/material/texture_library.cpp` decodes on the
  calling thread, then records a per-texture one-shot staging buffer, the copy, and the whole
  mip chain as successive blits into one command buffer submitted on the graphics queue and
  collected by fence. The GPU side is already asynchronous; the decode and the per-texture
  submit are not, and every mip is regenerated at every spawn.
- **Pipelines** — `engine/presentation/render/source/resources/pipeline_cache.cpp` already
  persists a `VkPipelineCache` to disk and rebuilds fast-linked pipelines optimized on a
  background thread. What remains synchronous is the first-sight build itself: a pipeline the
  disk cache has never seen compiles on the main thread inside the frame that first draws it.
- **Queues** — `engine/presentation/render/source/rhi/vulkan/vulkan_device.hpp` exposes a
  graphics queue only. No dedicated transfer queue exists, so no upload can overlap rendering
  at the queue level today.

Two pieces this design builds on already exist: the `.meta` import-settings sidecar
(`engine/asset/model/include/SushiEngine/model/import_settings.hpp`, read and written by
`engine/asset/model/source/import_settings_io.cpp`), which is where the cooking settings live;
and the Profiler panel (`docs/design/profiling_system.md`), whose CPU channels — including the
main-thread "asset work" channel — GPU pass timings and memory sections are where every
acceptance number below is read.

## 3. Architecture and boundaries

Three components, each behind its own boundary. Maintainability is priority one here: no
consumer of a cooked asset may know the blob's layout, and no part of the cooker may know
Vulkan.

1. **The cooker and its blob — a new asset-tier module, `engine/asset/cooking`** (namespace
   `SushiEngine::Cooking`), sibling to the model and gltf modules and the geometry counterpart
   of the physics cooker. It owns the meshoptimizer pipeline, the blob format, and a reader that
   hands out typed views — vertex ranges, per-LOD index ranges with their error metadata, mip
   payloads — so consumers ask for meaning, never offsets. The module is device-free and fully
   unit-testable; it carries its own `README.md` and tests like every module.
2. **The staging-ring uploader — a render/RHI concern behind an interface.** An uploader
   interface in the render module's public seam, implemented against the Vulkan device,
   consumed by the mesh registry and texture library. Callers hand it bytes and a destination;
   they never see queues, semaphores or ownership transfers.
3. **The editor — a consumer of both through existing seams.** The drop path
   (`applications/editor/source/project/project_panel.cpp` into
   `applications/editor/source/scene/scene_commands.cpp`) keeps its shape: the instantiation
   plan is unchanged, and only the geometry/texture source behind the render import swaps from
   "parse the file" to "map the blob". Import-settings additions follow the project's UI-first
   workflow — the unlinked ImGui section first, approval, then wiring.

meshoptimizer arrives the only way a dependency arrives in this workspace: declared in
`cli/sushistack.deps.toml` and provisioned by `ss install` from vcpkg. No vendored copy, no
manual install.

## 4. The cook pipeline

Run once at import, on a worker, in meshoptimizer's canonical order:

1. `meshopt_generateVertexRemap` — index the mesh and drop duplicate vertices.
2. **Simplify per LOD** — each level from the previous one, roughly 50% of its triangles per
   step, 4–5 levels, target error 1e-2 per step, using `meshopt_simplifyWithAttributes` with
   normal weight 0.5 so shading survives simplification, and border vertices locked
   (`meshopt_SimplifyLockBorder`) so quantized seams between kit pieces stay watertight. Each
   level records its object-space error — the simplifier's returned error times
   `meshopt_simplifyScale`, accumulated across the chain — which is what a screen-space-error
   LOD selector needs at draw time.
3. Per-LOD `meshopt_optimizeVertexCache`, then `meshopt_optimizeOverdraw` at threshold 1.05.
4. **Assemble every LOD into one index buffer, coarsest first**, then run a single
   `meshopt_optimizeVertexFetch` over the combined buffer. Coarsest-first is a deliberate
   instantiation property, not a packing detail: the first bytes uploaded are a complete,
   drawable mesh.
5. **Quantize attributes** — positions as 16-bit SNORM normalized to the per-mesh bounding box
   (uniform sub-0.1 mm resolution on a car-sized mesh, where half floats degrade with distance
   from the origin), normals as 10-10-10-2 SNORM carrying the bitangent sign in the spare bits
   (8-bit octahedral visibly bands on normal-mapped surfaces), tangents as 8-8 octahedral, UVs
   as 16-bit UNORM against the UV bounds. Dequantization is an explicit per-mesh scale and
   offset in the per-draw data — never baked into the node transform — and every pass,
   including motion vectors' previous-frame position, reads the same quantized stream so
   depth-equal tests and TAA reprojection stay bit-exact. The render pipelines gain the
   matching quantized vertex format.
6. `meshopt_generateShadowIndexBuffer` — a positions-only index buffer for depth-only passes
   (prepass and shadow cascades), typically 20–40% fewer vertices to fetch.

LOD generation is automatic at import. The level count and per-level ratios are settings on
`ModelImportSettings`, so the `.meta` sidecar carries them and re-import respects them; the
defaults above apply to a file with no sidecar.

## 5. The cooked blob

One file per source asset, written beside it like the `.meta` sidecar, invalidated by a hash of
the source file and the import settings. The format is flat, sectioned, and mmap-able — a
versioned header, then aligned sections the reader returns as spans:

- The quantized vertex buffer, and the combined index buffer with per-LOD ranges and each
  level's accumulated object-space error.
- The shadow index buffer.
- Meshlet data, cooked once here instead of rebuilt per spawn.
- Texture payloads: block-compressed, with the full mip chain cooked at import. The encoder is
  bc7enc_rdo with its integrated bc7e.ispc BC7 kernel — BC7 for albedo and emissive, BC5 for
  normals, BC4 for single-channel masks. Rate-distortion optimization is a per-texture import
  setting, mild for albedo and off by default for normals, where its correlated block noise
  reads as specular crawl. Upstream vcpkg carries no suitable encoder (its basisu is a
  transcoding tool with a lower quality ceiling; its nvtt predates modern BC7), so bc7enc_rdo
  arrives as a local overlay port through the same `ss` provisioning flow — the one deliberate
  deviation from stock ports in this design, called out here so it is a decision, not a drift.

Scene spawn does zero parsing: map the file, validate the header, memcpy sections into staging.
The blob layout is private to `engine/asset/cooking`; the renderer receives typed views and
uploads them, and a layout change is a version bump plus a re-cook, never a consumer edit.

## 6. Hitch-free instantiation

The spawn path, once the blob exists:

- **Worker-thread IO.** A worker maps and validates the blob; the main thread sees a readiness
  flip. The instantiation plan still executes immediately — entities appear in the frame of the
  drop — but each mesh renders as a placeholder until its geometry is resident.
- **Coarsest LOD first.** Because the combined index buffer is laid out coarsest-first, the
  placeholder is the mesh's own coarsest level, uploaded in the first chunk; finer levels
  stream behind it and refine in place. No bounding-box proxy is needed.
- **Transfer-queue staging ring.** The device gains a dedicated transfer queue (backed by a
  real copy engine on every target GPU, the development Pascal card included), and the uploader
  owns a persistent ring of HOST_VISIBLE staging chunks whose reuse is fenced off one timeline
  semaphore. Resources are EXCLUSIVE and move to the graphics queue by queue-family ownership
  transfer, with the release and acquire barriers emitted as a matched pair from a single
  uploader helper — validation layers largely cannot diagnose a mismatched or missing acquire,
  so correctness comes from construction, not tooling. Nothing in the upload path ever waits
  for a queue or the device to idle. Mesh buffers become device-local as a side effect, retiring
  the render-from-host-memory behavior of the current mesh registry.
- **Per-frame upload budget.** The uploader submits at most 8–16 MB of copies per frame
  (settled by measurement on the profiler), so a large drop spreads over a few frames of
  background latency instead of one long frame.
- **Mips cooked, not generated.** The blit-based mip chain in the texture library survives only
  for uncooked sources; a cooked texture uploads its block-compressed levels directly.
- **Pipeline pre-warm.** The pipeline factory's existing worker and disk cache are extended
  with first-sight compilation at import time: when a cook finishes, the materials it names
  have their pipelines built on the worker, and a draw that arrives before its pipeline is
  ready uses the engine default material for a frame rather than waiting.

## 7. Editor surface

The Import Settings window gains a Cooking section: LOD level count, per-level ratio, target
error, and a re-cook action, stored in the `.meta` sidecar like every existing setting. Per the
UI-first workflow it ships unlinked first — the section drawn against a mock settings object,
approved on appearance — and is wired in the same phase that makes the settings real. The
Profiler panel needs no additions; its asset-work channel, frame history and VRAM sections
already show every number this document is accepted against.

## 8. Phases

Each phase is independently shippable, ordered so the hitch shrinks earliest. Acceptance
numbers are read from the Profiler panel on the reference scene: the 400,000-triangle car
model dropped into an otherwise idle editor scene.

| Phase | Scope | Acceptance |
| --- | --- | --- |
| COOK0 | meshoptimizer declared in `cli/sushistack.deps.toml`; the `engine/asset/cooking` module: cook pipeline (§4), blob format and reader (§5), unit tests over remap/LOD/quantization round-trips; cooking runs on a worker at import and the render import consumes the blob when present. | The drop frame's asset-work channel falls from hundreds of milliseconds to under 50 ms (parse and mesh processing leave the spawn path); a re-drop of a cooked asset shows no cook cost at all. |
| COOK1 | Asynchronous instantiation: worker-thread blob IO, readiness flip, coarsest-LOD placeholder drawn from the first uploaded range. | Dropping the car never puts the frame over 20 ms; the full-detail mesh appears within a few frames, and the frame-history graph shows no spike at the swap. |
| COOK2 | The transfer queue, the staging-ring uploader behind its interface, the per-frame upload budget; mesh buffers become device-local. | During the upload window no frame exceeds the budgeted cost (upload work invisible in the GPU pass timings); steady-state opaque-pass GPU time does not regress after the device-local move. |
| COOK3 | Cooked textures: block compression and full mip chains in the blob; the texture library uploads cooked levels directly. | Texture-heavy drops show no decode or mip-blit cost in the asset-work channel; `resident_texture_bytes` on the asset library interface drops roughly 4x for the same textures (block compression), read from the Memory section. |
| COOK4 | Pipeline pre-warm: first-sight compilation moved to the factory worker at cook completion, default-material fallback for a not-yet-ready pipeline. | First draw of a new material shows no compile spike in the frame history; a cold-cache editor start followed by a drop stays under the COOK1 frame bound. |
| COOK5 | The Import Settings Cooking section, UI-first: unlinked ImGui, approval, then wiring to `ModelImportSettings` and re-cook. | Changing the LOD count re-cooks and the per-viewport triangle counters change accordingly; the settings survive a restart via the `.meta` sidecar. |

COOK3, COOK4 and COOK5 are independent of each other and may land in any order after COOK2.

## 9. Relationship to the render optimization program

This document produces data it does not consume. The LOD chain and its per-level object-space
errors exist for screen-space-error LOD selection in the culling compute; the shadow index
buffer exists for the depth-only passes; the cooked meshlet data feeds the meshlet path. All
three consumers are designed in `docs/design/render_optimization.md`, written in parallel with
this document, and every one of them reads the blob through the `engine/asset/cooking` reader —
never the layout.
