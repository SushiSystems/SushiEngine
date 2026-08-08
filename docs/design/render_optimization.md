# Render optimization program

**Status:** No phase is built. R0 is the entry point; R3 is blocked on the asset-cooking design's
cook phases (`docs/design/asset_cooking.md`, written in parallel with this document).

## 1. Purpose

`docs/design/profiling_system.md` built the measurement layer and deliberately did not optimize.
This document is the program its closing section promises: the ordered set of renderer
optimizations whose acceptance criteria are numbers the editor's Profiler panel
(`applications/editor/source/ui/profiler_panel.cpp`) shows against a recorded baseline. Every
phase names the passes it touches, the interfaces it adds, and the Profiler number that proves it
paid.

The program targets two hardware classes with one code path. The development and acceptance
machine is a GTX 1060 6 GB (Pascal, Vulkan 1.3); every core phase must also be the right design on
current GPUs (RTX 40/50, RDNA3/4). None of the core phases fork on capability — screen-space-error
LOD selection, per-cascade GPU culling, bucketed indirect lists, and LUT-driven sky are the
current best practice on both classes. The techniques that genuinely need newer hardware sit in
the backlog of §11 and are marked as unrunnable here, the way
`docs/design/render_pipeline_refactor.md` marks its ray-query items.

## 2. What exists today

The audit below is against source, not against what earlier plans intended.

- **Culling** — `engine/presentation/render/source/passes/cull_pass.cpp` dispatches
  `engine/presentation/render/shaders/cull.comp`: per-instance frustum test, a screen-coverage
  gate, and Hi-Z occlusion against *last* frame's max-depth pyramid
  (`engine/presentation/render/source/passes/occlusion_pass.cpp` reduces the prepass depth each
  frame; the cull samples it one frame late, reprojected). Survivors are compacted per bucket and
  each bucket gets one indirect draw. A statistics buffer (drawn, tested, triangles) reads back a
  frame late. This is a one-pass scheme: an object disoccluded this frame is drawn next frame.
- **Opaque submission** — `engine/presentation/render/source/passes/opaque_pass.cpp` holds four
  pipelines: classic per-instance, skinned, GPU-indirect (one `vkCmdDrawIndexedIndirect` per
  bucket), and a hardware mesh-shader meshlet path (`VK_EXT_mesh_shader`, task + mesh stages) that
  the device gates — Pascal does not offer it, so on the acceptance machine the meshlet path never
  runs. No path sorts: the classic loop draws in scene order (grouping buffer binds only when
  consecutive instances share a mesh), and buckets are submitted in registration order.
- **Depth prepass** — `engine/presentation/render/source/passes/depth_prepass.cpp` draws the full
  scene depth-only before opaque, in the same three flavors, unsorted. Opaque loads its result
  and tests `GREATER_OR_EQUAL`, so material shading is early-Z-rejected behind it. The prepass
  depth also feeds the occlusion pyramid and the SSR Hi-Z chain.
- **Shadows** — `engine/presentation/render/source/passes/shadow_pass.cpp` draws every
  shadow-casting instance into every cascade of the 2×2 atlas (up to four; lower tiers clamp to
  two or three) with no visibility test of any kind — no frustum, no cascade bounds, no GPU path.
  A caster behind the camera or outside a cascade's light-space extent still costs its full
  vertex work in that cascade. This is the largest confirmed gap in the tree.
- **Sky** — the Hillaire 2020 LUT stack is already shipped
  (`engine/presentation/render/source/passes/atmosphere_lut_pass.cpp`: transmittance 256×64,
  multi-scatter 32², per-frame sky-view 192×108, aerial-perspective froxels 32³). What remains
  costly is the consumer: `engine/presentation/render/source/passes/sky_pass.cpp` draws one
  fullscreen triangle with no depth test, and `engine/presentation/render/shaders/sky.frag`
  (1,031 lines) runs on every pixel — background pixels take the one-fetch sky-view path, but
  geometry pixels still execute the shader's full setup, body and ring intersection loops, and
  the aerial composite, and the analytic-ground and beyond-froxel-range branches keep a 32-step
  march. The pass is also the aerial-perspective composite, which is why it cannot simply be
  depth-tested away as it stands.
- **Level of detail** — none. No LOD chain exists anywhere in the mesh registry; the cull
  shader's screen-coverage gate drops instances below a pixel threshold, which is a binary
  version of the real thing.
- **Pipelines** — `engine/presentation/render/source/resources/pipeline_cache.hpp` already
  persists a disk-backed `VkPipelineCache` with a graphics-pipeline-library builder, so the
  "persist the PSO cache" hygiene item common to programs like this one is already done and is
  not a phase here.
- **Meshlets** — `engine/presentation/render/shaders/meshlet.task` culls per-cluster frustum only;
  its descriptor already carries the `cone` field for backface cone culling and does not read it.
- **Shading rate** — `engine/presentation/render/source/passes/shading_rate_pass.cpp` builds a
  rate image the sky and cloud passes opt into on devices that support it; Pascal does not.
- **Measurement** — `docs/design/profiling_system.md` (built): CPU channels, per-pass GPU timings
  via `engine/presentation/render/source/graph/gpu_profiler.hpp`, draw and triangle counters,
  culling counts, and video-memory budgets, all in the Profiler panel.

## 3. Program rules

- **Baseline first.** No optimization lands before R0 records the reference scene's numbers into
  this document. Every later phase states its acceptance as a Profiler number against that table.
- **Two quality bars.** Culling and sorting phases (R1, R2, R6) must be pixel-identical — same
  image, fewer cycles — verified with the `se render --probe golden` harness on a captured frame.
  Level-of-detail and sky phases (R3, R4) may change pixels imperceptibly at distance:
  conservative screen-space-error thresholds with hysteresis — and a dithered cross-fade under
  TAA only if popping proves visible — never a visible pop or silhouette change at the
  acceptance thresholds.
- **One code path.** A core phase never branches on GPU capability. Anything that would is
  backlog (§11).
- **Boundaries.** Every phase stays inside `engine/presentation/render` unless its section says
  otherwise; the two exceptions are R3's consumption of cooked asset data through the existing
  asset-library seam and the editor debug tooling, which follows the project's UI-first workflow
  (the unlinked ImGui panel first, approval, then wiring).

## 4. R0 — baseline

The reference scene is the two-car scene: the 405k-triangle car plus the 5k-triangle car, editor
Scene view at 1080p, default quality tier, camera at the standard framing (recorded alongside the
numbers so the measurement is repeatable). From the Profiler panel, record: CPU frame total and
per-channel times, every GPU pass timing (prepass, occlusion reduce, cull, shadow cascades,
opaque, sky, and the post chain), draw calls, submitted triangles, instances tested and drawn,
and video memory used. The numbers land in the table below as the program's fixed reference;
later phases cite it, they do not re-measure it.

| Measurement | Baseline (R0 fills this in) |
| --- | --- |
| CPU frame total / render submission channel | — |
| GPU: depth prepass / occlusion / cull | — |
| GPU: shadow cascades | — |
| GPU: opaque | — |
| GPU: sky | — |
| Draw calls / submitted triangles | — |
| Video memory used | — |

Acceptance: the table is filled from the panel on the acceptance machine, with the scene and
camera framing recorded beside it.

## 5. R1 — per-cascade GPU shadow culling

The highest-leverage phase: the shadow pass currently pays instance-count × cascade-count vertex
work with zero visibility testing (§2), and the research ranking's larger item — the sky — has
already banked most of its win through the shipped LUT stack.

Design: extend the existing cull compute rather than adding a CPU path that would reintroduce
readbacks. `engine/presentation/render/shaders/cull.comp` gains a shadow mode dispatched with
one Y-dimension slice per cascade;
each thread tests its instance's bounding sphere (already camera-relative in the instance record)
against that cascade's light-space AABB — tighter than frustum planes for an orthographic
cascade — and compacts survivors into per-cascade indirect draw lists. The cascade AABBs join the
`CullParameters` uniform. `ShadowPass` gains a GPU-driven path symmetrical with the prepass's:
per cascade, bind the atlas viewport, then one indirect draw per bucket from that cascade's list;
the classic loop remains the fallback exactly as it is for opaque. `CullPass` owns the new
buffers and exposes them through the frame-context targets the passes already share; no interface
outside the render module changes.

Casters that cast no shadow into any cascade (behind the light range, `cast_shadows` false) are
dropped in the same dispatch. A follow-up flagged item — off by default, since it is not
pixel-identical — updates the far cascade(s) at half rate for near-static scenes.

Acceptance: shadow-cascade GPU time drops against R0 with the camera framing unchanged; shadow
draw calls drop by the culled fraction; the golden probe shows a pixel-identical image.

## 6. R2 — opaque sort keys and bucket ordering

The classic path sorts its instance list once per frame before recording: color pass by
pipeline → material → mesh (state coherence; with the prepass in place, early-Z equal already
rejects hidden fragments, so depth order buys the color pass little), prepass front-to-back by
8–16 coarse depth buckets (a full sort is overkill at this draw count). The sort key is one
64-bit integer built where the draw list is assembled, so the recording loops stay dumb.

The GPU path is already partitioned into per-mesh buckets; this phase orders bucket submission
by pipeline and coarse nearest-instance depth on the CPU (bucket count is small), and orders
compacted instances within a bucket coarse-front-to-back in the cull shader with a stable per-bucket
depth-bucket counter — no GPU sort.

Acceptance: pixel-identical via the golden probe; opaque and prepass GPU times at or below R0
(the win is scene-dependent and honest zero is a valid finding at two meshes — the phase is
mostly load-bearing for R3, which multiplies the bucket count by the LOD chain length); CPU
render-submission channel does not regress.

## 7. R3 — GPU LOD selection (blocked on asset cooking)

**Blocked:** this phase consumes the LOD chain, the per-LOD object-space error metadata, and the
positions-only shadow index buffer that the asset-cooking design (`docs/design/asset_cooking.md`,
in parallel authorship) cooks at import. It cannot start before those cook phases land, and its
data contract is stated here so the two documents agree: per mesh, an ordered chain of index
ranges into one combined index buffer, coarsest first, each range carrying the simplifier's
accumulated object-space error.

Design: LOD selection happens in `engine/presentation/render/shaders/cull.comp`, where the
projected data already lives. Each
instance's thread computes error-in-pixels = object-space error × screen height / (2 × distance ×
tan(half fov)), and picks the coarsest LOD whose error stays under the threshold — 1–2 px,
conservative by default — with 10–20% hysteresis between the up and down thresholds so a
hovering camera never oscillates. Buckets become per-(mesh, LOD) index ranges; the bucket count
grows by the chain length and R2's ordering keeps submission coherent. Transitions ship in two
stages. Stage one is the hysteresis-guarded pop alone: at a ~1 px error threshold the switch is
sub-pixel and TAA hides most of the remainder — this is what most shipping engines default to,
and it needs no extra pipelines. Stage two, built only if pops prove objectionable on real
content, is a complementary dithered cross-fade: one ramped threshold drives both LODs, the
incoming LOD takes the inverted dither so every pixel is covered exactly once, the noise is
frame-varying interleaved gradient noise seeded by pixel, frame and draw, and the discard is
compiled bit-identically into the depth-prepass and color-pass shaders from one shared include —
any divergence yields holes under the prepass's equal test. Both LODs write correct motion
vectors for the fade's duration, and steady-state draws never take the discard variant. The
shadow and prepass paths consume the cooked positions-only index buffer and the same per-cascade
selected LOD.

Editor tooling, UI-first: a forced-LOD selector and an LOD-colorize view mode in the render
debug UI, built unlinked, approved, then wired to a debug flag in the cull parameters.

Acceptance: with the reference car duplicated into a grid at distance, submitted triangles and
opaque GPU time drop against the R0-derived grid measurement while a still-frame comparison at
the acceptance threshold shows no visible change; toggling forced-LOD 0 restores the undropped
numbers exactly.

## 8. R4 — sky pass split

Reality first: the Hillaire LUTs are shipped and consumed (§2), so what this phase buys is not
the research ranking's raymarch replacement but the remaining structural cost — every pixel runs
the whole of `engine/presentation/render/shaders/sky.frag`, and geometry pixels pay setup and
intersection work they do not need.

Design: split the pass in two. A **sky draw** renders only where no geometry is — the fullscreen
triangle gains a depth test (`GREATER_OR_EQUAL` at the reverse-Z far value, depth writes off), so
covered pixels are rejected before the fragment shader runs — and keeps the sky-view fetch, sun
disc, bodies, rings, and stars. An **aerial composite** applies transmittance and in-scatter to
geometry pixels only: one froxel fetch plus the fog fold, in a fragment shader a fraction of the
size. The analytic-ground and beyond-froxel march branches move with the sky draw, whose
coverage the depth test now bounds. The MRT ground-shadow output stays with the branch that
produces it. Both pipelines live in `SkyPass`; no interface outside the pass changes.

Acceptance: sky-pass GPU time (the two new passes summed) drops against R0 in a framing where
geometry covers a meaningful fraction of the screen, and does not regress in a pure-sky framing;
the image matches to within the LUT paths' existing tolerance (no new visible difference).

## 9. R5 — prepass experiment

At 405k triangles and 1080p the scene may not be overdraw-bound, and the full prepass doubles
vertex and raster work — but its depth feeds the occlusion pyramid and the SSR Hi-Z chain, so it
cannot simply be deleted. The experiment: measure (a) status quo, (b) prepass off with the
occlusion pyramid reduced from the opaque pass's depth instead, and (c) a partial prepass of the
N largest instances by projected area. R2's front-to-back prepass order and, once R3's cooked
data exists, the positions-only index buffer both shift the trade, so the experiment runs after
them and re-runs after R3. The measured decision is recorded in this section; the losing
configurations are removed, not flagged off.

Acceptance: the recorded decision, with the Profiler numbers for all measured configurations at
the reference framing, and the winner enabled.

## 10. R6 — hygiene: meshlet cones and barrier batching

Two small items, one phase. First, `meshlet.task` starts reading the cone data its descriptor
already carries: backface cone rejection beside the existing per-cluster frustum test —
unverifiable on the acceptance machine (no mesh shaders on Pascal) and therefore written, code
reviewed, and acceptance-deferred to hardware that runs it, stated as such. Second, barrier
batching: passes that issue consecutive single-resource `vkCmdPipelineBarrier2` calls — the cull
pass's back-to-back command and statistics buffer barriers in
`engine/presentation/render/source/passes/cull_pass.cpp` are the model case — fold them into one
dependency with multiple barriers, swept across the pass files. The
persistent pipeline cache the research prescribes already exists (§2) and is out of scope.

Acceptance: pixel-identical via the golden probe; no Profiler regression (the barrier win on this
frame count may be under measurement noise — an honest "no measurable change" is a pass, the
point is not paying per-resource submission cost as the pass count grows).

## 11. Backlog — needs hardware this machine lacks

Held in this document so nobody redesigns them, unrunnable on the acceptance machine, in the way
`docs/design/render_pipeline_refactor.md` holds its ray-query items:

- **Meshlet path exercised at all** — the `VK_EXT_mesh_shader` backend already exists (§2);
  everything meshlet-shaped, including R6's cones and per-meshlet occlusion culling in the task
  shader, accepts only on a mesh-shader device.
- **Async compute** — a render-graph queue flag overlapping Hi-Z reduction, light clustering, and
  bloom with raster on Ampere+/RDNA3+; forced off on Pascal, where pre-Ampere scheduling
  serializes the overlap with gaps.
- **Variable-rate shading spread** — the rate image and its two consumers exist (§2); extending
  it to more passes is pure subtraction on supporting devices and a no-op on Pascal.
- **Two-pass occlusion culling** — draw last frame's visible set, reduce, re-test the rejects
  against the fresh pyramid. Hardware-agnostic, unlike the rest of this list, but deferred: the
  one-pass scheme's cost is a frame of disocclusion latency, not GPU time, and the phase only
  pays once scenes are dense enough for aggressive occlusion to matter. Revisit when a Profiler
  measurement shows the cull's drawn/tested ratio leaving wins on the table.

## 12. Rejected techniques

From the research reject list, recorded so future readers do not relitigate:

- **Nanite-style continuous cluster LOD** — mesh-shader-class throughput plus a renderer-scale
  engineering project; discrete LOD chains with meshlets cover this content scale.
- **Full GPU radix sort of draws** — coarse depth buckets suffice at thousands of draws.
- **Bruneton-style high-dimensional atmosphere LUTs** — the shipped Hillaire stack exists
  precisely to avoid their cost and artifacts.
- **Sampler-feedback / sparse tiled texture streaming** — poor API fit on Pascal and unnecessary
  at 6 GB and 1080p; capped resident mip bias first if VRAM ever pressures.
- **Async compute on Pascal** — pre-Ampere overlap executes with gaps; the dedicated transfer
  queue for uploads remains fine.
- **Ray-query hybrid shadows** — waits until a TLAS has a second consumer; CSM stays the
  universal tier regardless, which is why R1 is worth building now.

## 13. Phases

| Phase | Scope | Acceptance |
| --- | --- | --- |
| R0 | Baseline: reference-scene Profiler numbers recorded into §4. | The §4 table is filled on the acceptance machine, scene and framing recorded. |
| R1 | Per-cascade GPU shadow culling in the cull compute; indirect shadow path; flagged half-rate far cascades. | Shadow GPU time and draw calls drop vs R0; golden probe pixel-identical. |
| R2 | Sort keys: classic color pipeline→material→mesh, prepass depth buckets; GPU bucket and intra-bucket ordering. | Pixel-identical; opaque/prepass GPU time and submission channel at or below R0. |
| R3 | GPU LOD selection by screen-space error with hysteresis pop (dithered cross-fade only if pops show); LOD debug UI (UI-first). **Blocked on asset-cooking cook phases.** | Grid-scene triangles and opaque time drop; no visible change at threshold; forced-LOD 0 restores R0-equivalent numbers. |
| R4 | Sky split: depth-tested sky draw plus thin aerial composite. | Sky GPU time drops in geometry-covered framing, no regression in pure-sky framing; no new visible difference. |
| R5 | Prepass experiment: full vs none vs partial, occlusion source moved as needed; decision recorded in §9. | Decision and numbers recorded; winning configuration enabled. |
| R6 | Meshlet cone culling (acceptance-deferred to capable hardware); barrier batching sweep. | Pixel-identical; no regression; cone change code-reviewed. |

R0 lands first and R1 next; R2 before R3 because R3 multiplies bucket count and is blocked on the
parallel cooking work anyway; R4–R6 land in the numbered order unless R3's unblocking reorders
them, which the phase table's criteria are independent of.
