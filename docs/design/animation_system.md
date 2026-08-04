# Skeletal Animation System — Animation, Animator, Blend Trees, and IK

An engineering plan for a Unity-parity, data-oriented character animation stack: clip
assets and compression, an Animator controller with layered state machines and blend
trees, an IK / pose-modifier stack, and GPU skinning — built on the archetype-chunk ECS,
the SushiRuntime task graph, and the render graph, without breaking determinism,
rollback, replay-only graph compilation, or the temporal core.

Status: **A0–A9 shipped and merged to main** (`09dd9be`, 2026-07-24) — skeleton/clip/
controller assets, the deterministic `animator_step`, blend trees, layered masks +
additive, the IK / pose-modifier stack, morph + generic tracks, humanoid retargeting,
controller JSON authoring, GPU skinning (compute pre-skin, previous-pose motion
vectors), and the editor GUI (Animation window, Animator/Mecanim graph window) are all
in the tree and CPU-verified. **Mecanim-parity authoring is done.** As of 2026-07-25's
audit, §12.1's live wiring is also done: a character actually GPU-skins in the editor
(`AnimatedMeshPreview`), GPU morph blending, the Statistics panel's Animation section,
and an Animator Preview window with a live layer list + mask authoring + a viewport IK
gizmo — all real code, **the editor GUI parts are still unverified on hardware** (a
`se editor --no-run` build + visual check is the next step for those specifically).
**§12.3's device-batched evaluator and all of §12.4 except neural compression were,
unlike the editor GUI work, actually built and run this session** — the toolchain
(`se` CLI, the llvm-sycl clang++, the sibling `sushiruntime`) is present and working
here, and this machine's CPU/OpenCL SYCL backend (auto-detected, no discrete GPU
needed) is enough to compile and execute a real SushiRuntime graph. Ten new
`examples/*_demo.cpp` binaries exist for §12.3/§12.4 alone, all passing: motion
matching's core AND its blend-graph crossfade wiring, full-body IK, ragdoll blending,
runtime retargeting, jiggle bones, dual-quaternion skinning (host proof of the
candy-wrapper fix, cross-checked bit-exact against a SushiRuntime SYCL device kernel),
ARKit-52 facial blendshape mapping, and a minimal sequencer timeline core. Dual-
quaternion skinning's blend math is also now wired into `skinning.comp`/`SkinningPass`
(an opt-in `SkinnedInstance::use_dual_quaternion_skinning` flag) — discovered mid-
session that `render/tools/shader_compiler` (glslang, build-time GLSL→SPIR-V) is
*also* headlessly runnable here, so the shader itself compiles and the C++ plumbing
links cleanly into `sushi_render.lib`, without needing a GPU display. Do not assume "no
GPU" means "cannot build/compile-check" for future animation *or* rendering work in
this repo — check `se build`/ninja against `build/` or `build-editor/` before assuming
something is unverifiable. `se_editor.exe` itself links clean as of this session's end
(an unrelated concurrent session's `load_scene` link break resolved on its own), with a
"Dual-Quaternion Skinning" checkbox wired into the Animator Preview window, **and §12.3's
device-batched evaluator is now wired into a real live-scene path** — a new Crowd
entity kind (`Simulation::CrowdParams`, `ISimulation::register_crowd_skeleton`/
`register_crowd_clip`, `RenderScene::skinned_instances`) that `RuntimeSimulation` samples
through `DeviceBatchEvaluator` every tick, merged into the viewport's draw call
alongside the editor's single-instance preview. What remains of §12.4: neural/ML
compression (deliberately never attempted — see its own entry below) and an actual
visual/GPU-display check that a bent joint renders correctly under DQS (SPIR-V validity
and C++ linkage are not the same as "looks right"). What remains of §12.3: an actual
interactive-session launch check — this session's own attempt to run `se_editor.exe`
hit a `vkQueueSubmit`/`VK_ERROR_DEVICE_LOST` very early despite a real GPU being
enumerable (`vulkaninfo` confirms a GTX 1060), unconfirmed whether pre-existing or
introduced by this session's work — see the crowd wiring entry for the exact finding.
**2026-07-30**: §12.2's glTF `WEIGHTS` animation-channel import — the last open in-scope
item in this document — is closed and verified by a real test suite, so morph weights are
clip-driven end to end (`examples/assets/morph_face.gltf`,
`tests/functional/unit/test_animation_morph_import.cpp`, seven cases, all passing). A
latent 64-track buffer overrun in `sample_morph_state` was found and fixed on the way.
That session also found the one genuinely large remaining gap and then closed its core: the
animation stack had **no tests in `tests/functional/` at all**, every claim below resting on
an `examples/*_demo.cpp` binary, which `CONTRIBUTING.md` §3.2 explicitly says does not
substitute for a test. **Ten suites, 125 cases, all passing** now cover the asset layer, the
deterministic tick, the blend trees, the layer/mask fold, the pose-modifier stack, retargeting,
the controller's persistence, the authoring model, and the §12.4 tail. §12.5 item 7 has the
table, the five defects the suites found, and the one item still uncovered
(`DeviceBatchEvaluator`, the only piece needing SushiRuntime).

Writing them paid for itself immediately and kept paying: **rotation root motion turned out
never to have been implemented**, though §0.1 listed it as met and two earlier audits of this
document confirmed it (implemented and tested 2026-07-30, §12.2); and of the four further
defects the §12.4 tail's suites found, **three were a comment or a doc describing behaviour the
code did not have** — including one that made dual-quaternion skinning's whole comparison
against linear blending meaningless. Read §12.5 item 7 before trusting any "confirmed present"
claim here. The failure mode both rounds demonstrate is the same: prose about code, with
nothing that fails when the two disagree.

Audited against the actual source tree on 2026-07-25; do not trust phase checkmarks in
this document without re-grepping, the previous revision drifted from the code for
weeks before this rewrite.

---

## §0 Goals and hard acceptance criteria

The system is done when every one of these holds. They are contractual, not aspirational.

1. **Authoring parity with Unity's Mecanim surface.** An `Animator` on an entity, driven
   by a controller asset containing layered state machines; states hold clips or blend
   trees (1D, 2D simple-directional, 2D freeform-directional, 2D freeform-cartesian,
   direct); transitions carry conditions over typed parameters (float / int / bool /
   trigger), exit time, duration, offset, and interruption sources; layers carry avatar
   masks and override/additive blending; animation events fire callbacks; root motion
   moves the entity (translation **and** rotation); two-bone IK, look-at IK, and chain
   IK modify the final pose. **Met** — the rotation half only as of 2026-07-30; it was
   listed as met for two revisions while nothing sampled it (§12.2). Test-guarded now
   (`Unit_AnimatorStep`, `Unit_AnimationBlendTree`, `Unit_AnimationLayers`,
   `Unit_AnimationIk`).
2. **Deterministic where it must be.** Everything that can affect gameplay — parameter
   values, state-machine state, transition progress, normalized times, event firing,
   root-motion deltas — advances only in the fixed-tick simulation, is bit-reproducible
   under `SE_DETERMINISTIC_FP`, lives in trivially-copyable ECS columns, and survives
   `RollbackBuffer` capture/restore byte-exactly. Pose evaluation and skinning are
   derived, per-render-frame data and are explicitly outside the rollback domain.
   **Met, and test-guarded as of 2026-07-30** (`Unit_AnimatorStep`: a `static_assert` that
   every column is trivially copyable, a same-inputs replay compared with `memcmp`, and a
   capture/restore/replay compared the same way).
3. **Replay-only graphs stay replay-only.** After warm-up, `Schedule::compile_count()`
   remains 1 with any number of animated entities spawning and despawning inside
   reserved chunks. Animation never migrates archetypes mid-run and never allocates
   pose memory inside a tick. **Met by construction (§5.4).**
4. **Performance floor** (High tier, 1440p output, the roadmap's reference GPU):
   100 unique skinned characters × 80 joints × 2 layers with active IK evaluate and
   skin in ≤ 1.0 ms GPU + ≤ 0.6 ms CPU total; 1000 crowd-LOD instances (≤ 32 effective
   joints, 15 Hz update, no IK) fit in the same budget alongside them. **Not met as
   specified** — the evaluator (`batch_evaluator.hpp`, `animator_evaluator.hpp`) is a
   plain host-side loop today; there is no SYCL device kernel for weight resolution,
   pose compose, or closed-form IK (§12.3). The CPU budget line in §9 assumes a device
   path that does not exist yet.
5. **Temporally clean.** Skinned meshes produce correct motion vectors from
   *previous-pose* skinning — zero TAA ghosting on animated characters. **Met**:
   `SkinningPass` writes `skinned_prev_position` every dispatch; `mesh_skinned.vert`
   consumes it. Awaiting a GPU-hardware pass to confirm empirically (no eyes on this
   box — see [[animation-headless-verification]]).
6. **Tier-wired on day one.** Max skinned instances, bone-LOD ladder, update-rate
   throttling, IK iteration caps and distance cutoff, influence count (4/8), and morph
   target caps are `QualityParams` fields resolved by `resolve_quality()`. **Met.**
7. **Memory-honest.** A 60-second, 30 Hz, 80-joint clip compresses to ≤ 250 KB at
   transparent quality (ACL-class ratios); pose pools, palettes, and event queues are
   fixed-capacity, allocated at load. **Compression met** (6.6×–17.6× measured by
   `animation_benchmark`). **Byte-accountable in the editor's statistics panel: not
   met** — no such panel rows exist yet (§12.1).
8. **Docs land with code.** Every phase ships its CHANGELOG entry, ARCHITECTURE
   section, and editor surface in the same PR. **Met** — CHANGELOG and
   `ARCHITECTURE.md` §12 are current, and the editor surfaces that were missing (mask
   editor, IK gizmos) shipped 2026-07-25 (§12.1). The sibling standard is **not** met:
   `CONTRIBUTING.md` §3.2 requires a test with new behavior, and the animation stack has
   one test file (§12.5 item 7).

---

## §1 Why the engine couldn't animate before this work — audit summary (historical)

Numbered so later sections can say which liability they killed. All eight are now
killed; kept for context on why the architecture looks the way it does.

1. **No skinning data path.** Fixed by the `SkinVertex` parallel stream + `SkinningPass`.
2. **The glTF importer discarded the skeleton.** Fixed by `gltf_skeleton_import.hpp`.
3. **The math seam had no interpolation.** Fixed by `core/types.hpp` additions (`lerp`,
   `nlerp`, `slerp`, matrix inverse, TRS decompose, quaternion-from-matrix).
4. **Hierarchy was host-side and non-topological.** Fixed: `SkeletonAsset`'s
   parent-index array is topologically sorted at import (`parent[i] < i`).
5. **No large per-object per-frame channel.** Fixed by the palette pool +
   `SkinnedGeometry` extract channel.
6. **TAA would ghost anything skinned.** Fixed by previous-pose skinning in
   `SkinningPass` (§0.5).
7. **No tier knobs for animation.** Fixed — `QualityParams` carries the full animation
   tier table (§6.6).
8. **No asset identity for animation.** Fixed — `.sushiskel` / `.sushianim` /
   `.sushictrl` are versioned, relocatable, `IAnimationDatabase`-backed.

---

## §2 Survey conclusions — what we adopted and why

| Source | What we took |
|---|---|
| **Unity Mecanim** | The *authoring model*, verbatim: Animator + controller asset, layered state machines, typed parameters, transition semantics, blend tree taxonomy, avatar masks, animation events. We took the surface, not the per-object C# graph evaluation. |
| **Unity DOTS Animation / Kinemation** | The *evaluation model*: skeletons as flat SoA blobs, pooled poses, batched evaluation. Bones-as-entities explicitly rejected. |
| **ACL (Animation Compression Library)** | The compression scheme shape, implemented in-engine rather than binding the library. |
| **Unreal GPU Skin Cache / Decima / Frostbite** | Compute pre-skinning: skin once per character, every consumer binds the result as a static mesh. |
| **Rune Skovbo Johansen — gradient band interpolation** | The 2D freeform blend weight algorithm. |
| **GDC "IK Rig" / Uncharted foot IK** | The IK architecture: an ordered stack of pose modifiers in model space after blending. |
| **Overwatch (GDC ECS talks)** | The determinism split: gameplay-authoritative state at fixed tick, derived pose per render frame. |

**Skip list, as originally scoped** (superseded by §12, which is the current,
audited state of these — some are still correctly deferred, some deserve
re-prioritization now that Mecanim-parity is done):
motion matching, full-body IK, Unity muscle-space retargeting, runtime retargeting,
Playables-style user graph API, bones as entities (permanently rejected), vertex-shader
skinning fallback (permanently rejected — one skinning path, tiers scale inputs not
implementations), draw-time morph target textures (permanently rejected — morphs run
in the compute pre-skin dispatch, see §12.1 for the fact that this integration itself
is still unbuilt).

---

## §3 Architecture — three domains, one-way data flow

```
  ASSET DOMAIN (import / cook time)
  glTF (nodes, skins, animations) ──► animation importer
        │ topological sort, bind-pose delta retarget, additive-delta bake
        ▼
  SkeletonAsset ─ ClipAsset ─ ControllerAsset          (immutable, relocatable blobs)
        │ loaded into shared-USM AnimationDatabase (read-only after load)
        ▼
  SIMULATION DOMAIN (fixed tick, deterministic, rollback-safe)          ~0.10 ms CPU
  ECS columns: AnimatorInstance, AnimatorParameterBlock,
               AnimatorLayerState[L], AnimatorEventQueue, RootMotionDelta
  systems: animator_step (state machines, transitions, time, events, root-motion
           sample) ──► apply_root_motion (writes Transform / Orientation)
        │ layer states + times + alpha (read-only snapshot at extract)
        ▼
  EVALUATION DOMAIN (per rendered frame, derived data, no rollback)  measured, not SYCL
  AnimationEvaluator (host-side today, batched over instances by a plain loop — §12.3):
    blend-tree weights ► clip sampling (decompress) ► layer blend + masks
    ► local pose ► model-space compose ► pose-modifier stack (IK)
    ► skin matrices (model × inverse-bind) ► palette pool (float, object space)
        │ SkinnedInstance records + palette pool via RenderScene
        ▼
  RENDER DOMAIN (render graph)
  JointPaletteSystem (per-frame palette SoA, double-buffered for motion)
  ► SkinningPass (compute; positions/normals/tangents + previous positions;
    linear-blend skinning only — morph deltas NOT yet applied here, §12.1)
  ► existing passes consume skinned streams as static meshes
```

One-way data flow, four seams (DIP). Each layer names an interface and cannot know
what stands behind it:

- **`Animation::IAnimationDatabase`** — hands out immutable `SkeletonView` /
  `ClipView` / `ControllerView` blobs by asset id.
- **`Animation::IPoseModifier`** — the IK / procedural-pose seam. Stateless
  `apply(PoseContext&)` over a model-space pose; solvers are registered per-Animator as
  an ordered stack.
- **`Animation::IPoseTaskContext`** — what a pose modifier may ask of the world
  (ray query for foot placement, target positions). Ground-plane implementation ships
  first; not yet physics-backed.
- **`Render::SkinnedGeometry` channel on `RenderScene`** — the extract seam. The
  renderer sees palette floats, stream handles, and counts; it cannot see clips,
  controllers, or the evaluator.

Domain rules unchanged from the original design: simulation touches only
trivially-copyable ECS columns and the immutable database (rollback-safe by
construction); evaluation owns all heavyweight pose memory outside the ECS, recomputed
every frame; render consumes extracted data only.

---

## §4 Data model and assets — as shipped

### 4.1 SkeletonAsset

Flat, immutable blob: `joint_count` (≤ `MAX_JOINTS` = 256), topologically sorted
parent-index array, bind-pose local TRS (SoA), inverse-bind matrices, joint name hashes
(FNV-1a 64), and a bone-LOD table (every LOD prefix is a valid skeleton).

### 4.2 ClipAsset and compression

Tracks: per-joint rotation/translation/scale, morph-weight, generic float
(property-hash addressed), event track, root-motion metadata. Storage is the
ACL-shaped scheme: uniform-time segments, per-segment range reduction, three-smallest-
component quaternions, error-driven bit-rate selection. Additive clips baked at import
as deltas. **Shipped, CPU-verified**, 6.6×–17.6× compression measured.

### 4.3 ControllerAsset — the compiled Animator

Authored in the editor as JSON, compiled to a flat index-linked blob at load:
parameters, layers, states, transitions, conditions, blend-tree nodes. Evaluation is a
data interpreter over the blob — no virtual dispatch per node, device-visible
structurally (not yet device-*executed*, §12.3).

### 4.4 Import pipeline

`animation/gltf_skeleton_import.hpp` extends the cgltf lane: preserves the joint tree,
reads skins/animations/morph targets, resamples to uniform rate. Humanoid import maps
joints to the canonical avatar and retargets by bind-pose delta **at import time**;
`runtime_retarget.hpp`'s `RuntimeRetargeter` (§12.4, closed 2026-07-25) extends this to
a target rig chosen at runtime, for the "swap a character model mid-game" case import-
time retargeting cannot cover. Cooked artifacts: `.sushiskel`, `.sushianim`, `.sushictrl`.

### 4.5 Fixed capacities (documented, asserted, tier-scaled where marked)

| Cap | Value | Where |
|---|---|---|
| `MAX_JOINTS` per skeleton | 256 | asset compile error above |
| `MAX_LAYERS` per controller | 4 | controller compile |
| `MAX_PARAMETERS` | 32 | `AnimatorParameterBlock` column |
| `MAX_EVENTS_PER_TICK` | 8 per entity | `AnimatorEventQueue` column |
| Influences per vertex | 4 (8 on Ultra, tier knob) | skinning stream |
| `MAX_POSE_MODIFIERS` per Animator | 8 | modifier stack |
| Morph targets per mesh | 64 active (tier knob) | skinning dispatch (cap defined; GPU application not wired, §12.1) |
| Blend-tree children per node | 16 | controller compile |

---

## §5 Runtime evaluation in detail

### 5.1 Simulation tick — the deterministic half

`AnimatorInstance`, `AnimatorParameterBlock`, `AnimatorLayerState × MAX_LAYERS`,
`AnimatorEventQueue`, `RootMotionDelta` — all trivially copyable, registered in one
shared header. `animator_step` advances normalized time, evaluates transitions
(Any-State first, then current-state, honoring exit time and interruption source),
steps/starts crossfades, consumes triggers once, appends events, and samples the root
track — both translation (`blend_root_delta`) and rotation (`blend_root_rotation`, added
2026-07-30; the earlier claim that this existed was wrong, see §12.2) — into
`RootMotionDelta`. `apply_root_motion` writes `Transform`/`Orientation` as a second,
dependency-ordered system. Line references are deliberately omitted here: citing them is
what let two audits mistake the delta's *consumer* for its *producer*.

### 5.2 Frame evaluation — blend trees, sampling, layers, compose

At extract, the evaluator snapshots layer states plus the loop's `interpolation()`
alpha. Evaluation today is a **host-side, single-threaded-per-call chain** (not a
SushiRuntime/SYCL device graph — §12.3):

1. Weight resolution over the blend tree (1D segment lerp, 2D simple-directional
   angular sectors, 2D freeform gradient bands, direct).
2. Sample + blend: decompress and accumulate weighted poses; layers fold in
   mask-gated (override nlerp / additive FMA).
3. Compose: model-space pose by forward scan over the topologically sorted parent
   array.
4. Pose-modifier stack (§5.3), model space, in place.
5. Palette build: `skin[i] = model_pose[i] × inverse_bind[i]`, object space.

Bone LOD and update-rate throttling select per instance before step 1
(`batch_evaluator.hpp`): distance/screen-coverage bucket → joint-count prefix and
update rate; throttled instances reuse their last composed pose.

### 5.3 The IK / pose-modifier stack

Ordered `IPoseModifier` stack per Animator, model space, after layer blending. Shipped
solvers: **TwoBoneIk** (analytic, pole-vector, soft-clamped), **LookAtIk** (weighted
chain aim, cone-clamped), **ChainIk** (FABRIK, iteration-capped), **FootPlacementIk**
(ray query → two-bone → ankle-to-normal). `ik_foot_placement.hpp` explicitly documents
that **pelvis-height (cross-foot) adjustment is left to a higher-level rig pass** — not
implemented, not a bug, a scoping decision that stands until something needs it.

### 5.4 Pose pools and graph integration

All pose-sized memory lives in module-owned shared-USM `Buffer`s sized at load,
sliced per instance by stable offsets, claimed at spawn / released at destroy.
`compile_count == 1` is preserved by construction — nothing here is an ECS column
except the small deterministic state (5.1) and a `PoseSlot` handle.

### 5.5 Determinism and rollback

Rollback snapshots capture animator columns byte-exactly. The evaluator reads sim
state only at extract. Pose pools/palettes/skinned streams are never snapshotted.
No archetype migration for enable/disable. `animator_step` uses no wall clock, no RNG.
**Met, CPU-verified** (`animator_demo`).

---

## §6 Render integration

### 6.1 Vertex streams

`SkinVertex { uint8/uint16 joints[4|8]; unorm8 weights[4|8] }`, a parallel stream —
`MeshVertex` stays frozen. Morph target deltas ship as packed per-target delta
buffers (asset side only — the GPU consumer isn't wired, §12.1).

### 6.2 SkinningPass — compute pre-skin, once per character

`render/passes/skinning_pass.cpp/.hpp`, early in the frame: one dispatch per skinned
instance batch, reads base vertices + skin stream + current/previous palettes, writes
transient `skinned_position/normal/tangent` and `skinned_prev_position`. Depth prepass,
opaque, and shadow passes bind the skinned streams through the existing vertex-input
path. Linear-blend skinning by default; an instance may opt into dual-quaternion
skinning (`SkinnedInstance::use_dual_quaternion_skinning`, §12.4, closed 2026-07-25) via
a push-constant flag rather than a true SPIR-V specialization constant (simpler to wire,
same functional effect — no pipeline-permutation infrastructure needed). The blend math
and the GLSL port both compile/link cleanly (glslang headlessly, via
`render/tools/shader_compiler`); an actual GPU visual check is still open.

> **Correction (as implemented).** The palette does not ride the scene set (set 0 is
> full at 32 bindings). It is a **SkinningPass-local** resource, current + previous,
> bound to that compute pass's own descriptor set. `mesh_skinned.vert` reads a skinned
> stream's previous position (a vertex input, keyed off a push-constant flag), not the
> palette directly. Instances reference their palette slice by a stable
> `palette_offset`.

### 6.3 JointPaletteSystem and motion vectors

`render/scene/joint_palette_system.*`: a per-frame palette SoA, double-buffered so the
previous frame's palettes survive for prev-position skinning.

### 6.4 Bounds and culling

Skinned bounds = bind-pose bounds inflated by a per-clip factor baked at import;
IK-active instances add a fixed margin.

### 6.5 Morph targets — asset side shipped, GPU side not

Weights are clip tracks, blended like any track (`morph.hpp`, CPU reference,
`morph_demo`-verified). **`SkinningPass` does not apply morph deltas** — grepped, no
"morph" anywhere in `render/passes/skinning_pass.*`. This is the single biggest gap
between "the morph system exists" and "a character's face can actually deform on
screen" (§12.1).

### 6.6 Quality tiers (`QualityParams`)

| Knob | Low | Medium | High | Ultra |
|---|---|---|---|---|
| Max full-rate skinned instances | 32 | 64 | 128 | 256 |
| Bone LOD bias | +2 | +1 | 0 | 0 |
| Update-rate ladder (Hz by bucket) | 30/15/10 | 60/30/15 | 60/30/15 | 60/60/30 |
| IK | off > 15 m | on, 2 it. | on, 4 it. | on, 8 it. |
| Influences | 4 | 4 | 4 | 8 |
| Active morphs / mesh | 8 | 16 | 32 | 64 |

---

## §7 Editor integration — shipped vs. not

Shipped:

- **Animation window** (`editor/animation/animation_panel.*`) — Unity's Animation-window
  shape, entity-based: targets the Hierarchy-selected entity, Record keys its
  transform, Record-off scrubbing drives the object live, Bake → `.sushianim`.
- **Animator window** (`editor/animation/animator_graph_panel.*`) — Mecanim-style
  state-machine graph: grid canvas, draggable nodes, right-click ▸ Make Transition To,
  Entry/Exit/Any-State nodes, parameter panel, JSON save/load, **and** a blend-tree
  inspector with a live 1D threshold bar and 2D pad visualization
  (`draw_blend_visualization`, `animator_graph_panel.cpp:251-386`).
- **Skeleton debug draw** (`editor/animation/skeleton_debug_draw.*`) — joint
  octahedrons + names overlay; unused since the glTF Skeleton-Preview window was
  removed, but the primitive stays available.

Not shipped (§12.1 has the full accounting):

- **Avatar mask editor + per-layer weight UI.** No mask-editing UI anywhere in
  `editor/animation/`.
- **IK gizmos** — draggable targets/pole vectors, per-solver weight sliders. Absent;
  `ik_foot_placement.hpp`'s pelvis adjustment isn't implemented either, so there'd be
  nothing to expose yet for that solver specifically.
- **Statistics panel rows** — pose pool bytes, palette bytes, compression ratio,
  per-stage profiler ms. `animation_benchmark` already computes these numbers
  headlessly; they were never wired into `editor/ui/editor_panels.cpp`.

---

## §8 Phased roadmap — history (A0–A9, complete)

Kept for the record; every phase below shipped. New work goes under §12.

- **A0 — Math seam + skeleton foundations.** ✅ Shipped.
- **A1 — Single-clip playback + GPU skinning, temporally clean.** ✅ Shipped — GPU
  skinning pipeline built end to end and confirmed present in the tree
  (`skinning_pass.cpp/.hpp`, `skinning_system.hpp`), not merely "written pending a GPU
  build" as earlier drafts of this document claimed.
- **A2 — Compression + batching + LOD.** ✅ CPU core shipped. Device-batched
  evaluation still open (§12.3); statistics panel still open (§12.1).
- **A3 — Animator core: parameters, state machine, events, root motion.** ✅ Shipped,
  including rotation root motion (confirmed in source, not just translation).
- **A4 — Blend trees.** ✅ Shipped, including the editor 2D visualizer (confirmed
  in source).
- **A5 — Layers + masks + additive.** ✅ CPU core shipped. Editor mask/weight UI
  still open (§12.1).
- **A6 — IK / pose-modifier stack.** ✅ CPU core shipped. Editor gizmos and
  pelvis-height adjustment still open (§12.1, §5.3).
- **A7 — Morph targets + generic tracks.** ✅ CPU core shipped. GPU-side morph
  application still open (§12.1).
- **A8 — Humanoid avatar + import retargeting.** ✅ Shipped.
- **A9 — Authoring suite completion.** ✅ Shipped, including the editor GUI (Animator
  graph window, controller JSON persistence, edit-mode scrub) — confirmed present in
  the tree, not "pending" as earlier drafts of this document claimed.

---

## §9 Performance budget summary — aspirational, not yet met

High tier, 1440p output, the roadmap's reference GPU; 100 hero characters (80 joints,
2 layers, IK) + 1000 crowd (≤ 32 joints, throttled).

| Stage | Budget | Domain | Status |
|---|---|---|---|
| `animator_step` + root motion (1100 instances) | 0.10 ms | sim tick | plausible, cheap ECS work |
| Weight resolution + sampling + layer blend | 0.25 ms | evaluator | **unverified — no device kernel, §12.3** |
| Compose + palette build | 0.10 ms | evaluator | **unverified — same** |
| IK stack (100 instances) | 0.10 ms | evaluator | **unverified — same** |
| Palette upload + extract | 0.05 ms | CPU | plausible |
| SkinningPass (compute) | 0.30 ms | GPU | untested on hardware (no GPU access this session) |
| Marginal cost in prepass/opaque/shadows | ≤ 0.15 ms | GPU | untested |
| **Total** | **≤ 1.05 ms** | | **not measured end to end** |
| Memory: pose pools + double palettes (1100 inst.) | ≤ 24 MB | | plausible from fixed-capacity design |

Do not report this budget as met until `batch_evaluator.hpp`'s successor runs on the
SushiRuntime task graph and the numbers come from the pass profiler, not this table.

---

## §10 SOLID

Unchanged from the original design and still holds structurally:

- **SRP.** Each stage is one module: blob compiler, state-machine interpreter,
  blend-weight resolver, clip sampler, layer blender, composer, each IK solver,
  palette system, skinning pass.
- **OCP.** New blend-node kinds and state kinds extend the compiled node table and
  interpreter case list without touching existing states; new IK solvers implement
  `IPoseModifier` and register.
- **LSP.** Every `IPoseModifier` is substitutable in any stack position; every clip
  source behind `IAnimationDatabase` yields views with identical sampling semantics.
- **ISP.** Narrow seams per consumer: `IAnimationEventSink`, `IPoseTaskContext`,
  `SkinnedGeometry`.
- **DIP.** Simulation and evaluation depend on database views; the renderer depends on
  extracted records; pose modifiers depend on an abstract query context.

---

## §11 Dependencies on the main roadmap

- **Phase 3 (foundation hardening)** — binding 14 claims the next scene-set slot;
  already accounted for (§6.2's correction note).
- **Phase 10 (GPU-driven geometry)** — `palette_offset` designed to fold into the
  instance SoA; coordinate before either lands its buffer layouts.
- **Phase 11 (async compute / SYCL interop)** — evaluator palettes are the first
  candidate for zero-copy SYCL→Vulkan; blocked on §12.3 landing a device evaluator
  first (no point building the interop before there's a device-side producer).
- **Physics / XPBD** — `IPoseTaskContext`'s physics-backed implementation and ragdoll
  blending (§12.4) consume the existing solver; nothing here blocks on it.
- **SushiBLAS** — all math lands behind `core/types.hpp`; the swap remains one file.

---

## §12 Gap analysis — what "AAA" still requires (audited 2026-07-25)

Mecanim-parity authoring (§0.1) is done. This section is the honest remainder,
split by why it's missing: **unfinished A-phase work** (§12.1 — CPU seam shipped,
consumer never wired), **a structural gap against the stated performance goal**
(§12.3), and **features never in scope for A0–A9 at all** (§12.4 — genuinely new
work, not a bug).

### 12.1 Unfinished editor / GPU wiring (small, scoped, close to done)

These have their data model and CPU logic already shipped; only the last consumer is
missing.

- **The animation-evaluator-to-renderer bridge — CLOSED 2026-07-25.** Turned out to be
  the real blocker under the original "GPU morph blending" line: nothing anywhere in
  the codebase ever constructed a `Render::SkinnedInstance` — the entire GPU-skinning
  path (`SkinningSystem`, `SkinningPass`, the opaque pass's skinned-range draw) was
  built and unit-adjacent-verified but had zero live producer, so no character could
  ever actually appear skinned on screen. Closed by, in order: (1)
  `Render::Assets::import_gltf_skinned_mesh` (`render/material/gltf_importer.cpp`) —
  reads `JOINTS_0`/`WEIGHTS_0`, remaps glTF joint indices into the cooked skeleton's
  topological order via the existing (previously unused) `Animation::remap_from_order`
  (`skin_vertex.hpp`), uploads through `MeshRegistry::add_skinned_mesh`; exposed on
  `IAssetLibrary::load_gltf_skinned_mesh`. (2) `Editor::AnimatedMeshPreview`
  (`editor/animation/animated_mesh_preview.*`) — mirrors `EffectPreview`'s shape:
  imports skeleton + first clip + skinned mesh sharing one skin index, runs the A1
  `Animation::ClipEvaluator` each `update(dt)`, ping-pongs a previous-pose buffer so
  motion vectors are correct from frame one, and builds the frame's `SkinnedInstance`.
  (3) `ViewportPanel::draw` gained an `animated_mesh` parameter threaded into
  `ISceneView::render`'s `skinned`/`skinned_count` slots (previously always
  `nullptr, 0`); `editor/main.cpp` owns one `AnimatedMeshPreview`, loads
  `examples/assets/rigged_arm_anim.gltf` at startup (fails silently, does not block
  editor boot, if the path is wrong for a given run), and updates/threads it into both
  the Scene and Game viewport draws. **Unverified on hardware — this machine has no
  GPU access this session.** Needs the user to build (`se editor --no-run`) and confirm
  a skinned character actually renders before this is trusted; the joint-remap and
  previous-palette ping-pong logic in particular have failure modes (wrong bind pose,
  motion-vector popping) that only show up visually, not at compile time.
- **GPU morph target blending — CLOSED 2026-07-25, unverified on hardware.** Extended
  the chain end to end: `MeshRegistry::set_morph_targets`/`morph_buffer`/
  `morph_target_count` (target-major, tightly-packed `float[3]` per (target, vertex) —
  deliberately a flat `float[]` in the shader, not `vec3[]`, because std430 pads a
  `vec3` array element to 16 bytes while the host packs at 12; a `vec3[]` binding would
  misalign every read past the first target, caught in review before this ever ran);
  `gltf_importer.cpp` now reads `primitive.targets[]`'s POSITION deltas (glTF morph
  deltas are already relative, no subtraction needed) alongside the skinned-mesh
  import; `SkinnedInstance`/`SkinnedRange`/`SkinningSystem` carry a per-instance
  morph-weight slice through a new packed per-frame weight buffer; `SkinningPass`
  binds two more descriptor slots (mesh morph deltas, frame morph weights — falling
  back to the already-bound skin/palette buffer when an instance has none, since a
  descriptor set needs a live handle even for a binding the shader will not read) and
  `skinning.comp` blends `Σ weight × delta` into the base position before joint
  skinning. `AnimatedMeshPreview::set_morph_weights` was the manual seam; the weights are
  clip-driven as of 2026-07-30 (§12.2). Still needs the user's GPU build to confirm
  visually: `rigged_arm_anim.gltf`, the rig wired into `editor/main.cpp`, has no blend
  shapes, so point `panel_state.hpp`'s `character_path` at
  `examples/assets/morph_face.gltf` instead — a deliberately minimal fixture (one skinned
  triangle, two targets), enough to see a target move on screen but not a face.
- **Live layered/masked/IK `AnimatorEvaluator` path — CLOSED 2026-07-25, unverified on
  hardware.** Part 1 of the two-part re-scope below: `AnimatedMeshPreview` no longer
  runs the cut-down single-clip `ClipEvaluator` — `load_gltf` now compiles a minimal
  one-layer `ControllerAsset` around the base clip (`Animation::compile_controller_blob`)
  and drives it through the real `Animation::animator_step` (deterministic tick) +
  `Animation::AnimatorEvaluator` (layers, masks, additive, pose-modifier stack) chain,
  the same evaluator `animator_demo`/`layered_animation_demo`/`ik_demo` exercise
  headlessly. New API: `add_layer(clip_name, mask_path, weight, additive)` compiles in
  another mask-gated layer looping a different clip from the same glTF file (recompiles
  the controller blob each time — cheap for a preview, not meant for a hot loop);
  `set_two_bone_ik(upper, mid, tip, target, pole, weight)` attaches the one shipped
  `TwoBoneIk` solver to the pose-modifier stack. **Two bugs caught in review before they
  shipped**: (1) `TwoBoneIk`'s own default weight is 1.0 (meant for a caller that sets
  real joints immediately) — left alone, the preview would have run IK at full weight
  against the degenerate all-joint-0 chain from frame one; `clear()` now explicitly
  zeroes it. (2) none, actually — first-pass draft assumed `restart()`'s old
  `time_ = 0` semantics could stay; corrected to `animator_instance_.initialized = 0`
  (the new state's reset hook) during the same pass, not a shipped bug, but worth
  naming since it is the kind of thing a partial rewrite silently gets wrong.
  Statistics panel gained a `Layers`/`Two-bone IK` row. No mask/IK gizmo UI yet — that
  is part 2, below, unstarted.
- **Mask editor + IK gizmo UI — part 2, CLOSED 2026-07-25, unverified on hardware.**
  `AnimatedMeshPreview` gained the API part 2 needed first: `add_layer` now takes an
  in-memory `Animation::MaskDesc*` instead of a `.sushimask` file path (cooks the blob
  and registers it internally — no save step); `layer_count`/`layer_name`/
  `layer_additive`/`layer_weight`/`set_layer_weight`/`remove_layer` for a live layer
  list; `set_layer_weight` writes a compiled `AnimatorParameterBlock` slot
  (`LayerRecord::weight_parameter` — the same seam Mecanim's own layer-weight sliders
  use) so it never recompiles the controller; `skeleton()`/`available_clips()` for
  joint/clip enumeration; `set_ik_target`/`two_bone_ik()` for the gizmo's per-frame
  read/write. New **Animator Preview** window (`editor/animation/
  animator_preview_panel.*`, Window ▸ Animator Preview): a layer list with live weight
  sliders and remove buttons, an Add Layer form (clip combo from the source file, a
  per-joint checkbox mask with a default-weight slider, initial weight, additive
  toggle), and a two-bone IK section (joint-name combos for upper/mid/tip, a pole-vector
  drag field, a weight slider, Apply). The IK **target** itself is not a panel field —
  it is dragged directly in the Scene viewport: `ViewportPanel` gained a second,
  independent `GizmoController` (`ik_gizmo_`) and an `ik_gizmo` draw parameter (true
  for Scene, false for Game — the same authored-here/played-there split as the
  selection gizmo), reusing the existing translate-gizmo math against a throwaway
  `Simulation::EntityTransform` built from the IK target converted to/from the
  character's world space each frame (`transform_point`/`SushiEngine::affine_inverse`)
  — no bespoke drag/projection code written for this. Mask/IK authoring is now
  genuinely interactive, closing the original design's promise, not a numeric-only
  fallback.
- **Statistics panel — CLOSED 2026-07-25, unverified on hardware.** Added an
  "Animation" section to the Statistics panel: source path, joint count, palette
  bytes/frame, clip format (raw/compressed) + frame count + sample rate, uncompressed
  clip-track byte estimate, and active morph-weight count — sourced from a new
  `AnimatedMeshPreview::statistics()` snapshot, threaded through a new
  `EditorContext::animated_mesh_preview` pointer (mirrors `particle_preview`). Honest
  about scope: this reports the *one live preview instance*, not a whole-scene
  aggregate (there is only one instance to aggregate today — no multi-character ECS
  wiring yet, §12.3/§12.4), and the clip is always reported "raw" because the live
  import path (`import_gltf_animated` → `build_clip_blob`) never runs the ACL-shaped
  compressor — that path only exists in `animation_benchmark` today. No compression
  ratio is shown for that reason; showing one would have been a fabricated number.
- **Pelvis-height (cross-foot) adjustment in `FootPlacementIk`.** Explicitly deferred
  in the header comment; revisit once uneven-terrain foot planting is actually used by
  a gameplay feature (no speculative build).

### 12.2 Root-motion / IK correctness refinements

- **Rotation root motion — was NOT present; implemented 2026-07-30.** This entry previously
  read "rotation root motion is present (§5.1) — no work needed here, corrects an error in the
  prior revision of this document." That correction was itself wrong, and §0.1 listed the
  translation-and-rotation contract as **Met** on the strength of it. What existed was the
  `RootMotionDelta::rotation` field and its consumer (`apply_root_motion` composes it into the
  entity's orientation); what did not exist was anything writing it. `animator_step` set only
  `root_motion.position` — a grep for `root_motion.rotation` found no assignment anywhere — so
  every clip whose root turned moved the character without turning it. Both earlier audits read
  the struct field and the consumer and concluded the sampler was there. Found by writing
  `Unit_AnimatorStep`, which is the whole argument for the suite: a claim that survived two
  document audits did not survive one test.
  Closed by `detail::root_rotation_delta` — the rotational counterpart of `root_at`, continuous
  across a loop for the same reason: the per-cycle turn `rotation(0)⁻¹ · rotation(duration)` is
  compounded once per whole loop crossed, bounded by `MAX_ROOT_ROTATION_CYCLES` so a
  pathological step saturates rather than looping unboundedly — and
  `detail::blend_root_rotation`, which blends a blend tree's per-clip deltas with the same
  hemisphere-corrected weighted component sum the evaluator already uses on a pose.
  `joint_translation`'s frame bracketing was factored into `bracket_frames` and shared with a
  new `joint_rotation` rather than duplicated. Verified by test: a clip turning a quarter circle
  per loop turns the entity a quarter circle per loop across two wraps, about the clip's axis
  only, and a clip whose root does not turn leaves the orientation exactly alone over 300 ticks
  instead of drifting by an accumulated near-identity delta.
- **glTF `WEIGHTS` animation-channel import — CLOSED 2026-07-30, actually verified.**
  Found 2026-07-25 while wiring GPU morph blending (§12.1):
  `import/gltf_animation_importer.cpp` read translation/rotation/scale channels only —
  never `cgltf_animation_path_type_weights` — so a clip's morph-weight tracks
  (`.sushianim` v2, A7) could never be populated from a glTF source, and
  `AnimatedMeshPreview::set_morph_weights` had to stay a hand-set seam. Closed by:
  - **The sampler.** `read_weight`/`sample_weight` alongside the existing
    `sample_vec3`/`sample_quat`. glTF packs *every* target's weight into one sampler
    (`target_count` scalars per key), so one channel becomes one track per target — a
    single element read at `key * target_count + target_index`, with the same
    cubic-spline-reads-the-middle-component and step-vs-linear handling the joint
    samplers use. Tracks are named after the target (`mesh.extras.targetNames`, else the
    positional `morph_<index>`); a name a second channel repeats is skipped, since
    `ClipView::find_morph` is first-match and could never reach the duplicate anyway.
  - **The mesh side.** `GltfAnimationImport::morph_target_names` reports the target order
    the *mesh* fixes — which is what `morph.hpp`'s `sample_morph_state` needs to map a
    clip's name-addressed tracks onto a mesh's index-addressed weights, and which nothing
    in the codebase produced before. Its documented contract is that it matches
    `Render::Assets::import_gltf_skinned_mesh`'s upload order (first triangle primitive of
    the first node bound to this skin carrying a complete skinned vertex set); the
    predicate is written out in both files with a note to keep them in step. The
    positional fallback name is the load-bearing detail: both sides derive `morph_<i>`
    from the same index, so an unnamed target still resolves by hash.
  - **The consumer.** `AnimatedMeshPreview` samples the base layer's clip into the mesh's
    target order every `update()` (`clip_driven_morphs()`, on by default, a no-op when the
    clip has no morph tracks so a hand-set pose survives), exposes
    `morph_target_name(index)` for slider labels, and reports the track count and drive
    mode through `Statistics`. The Animator Preview window gained a "Driven by clip"
    checkbox that disables the sliders while it is on; the Statistics panel's morph row
    now says clip-driven vs. manual. Morph weights are sampled from the *base* clip, not
    blended across layers — layer blending is a pose operation, weight tracks are not
    part of a pose.
  - **A real latent bug found and fixed on the way.** `sample_morph_state` sampled *all*
    of a clip's tracks into a `float[MAX_MORPH_TARGETS]` (64) stack buffer while clamping
    only its own loop bound — a clip with more than 64 morph tracks overran it. It was
    unreachable before this work (nothing could produce morph tracks from an asset), and
    would have become reachable the moment a real facial rig landed. Fixed by adding
    `ClipView::sample_morph_track` (one track, same bracketing) and having
    `sample_morph_state` sample only the tracks a mesh's targets actually name — which is
    also strictly less work in the common case.

  **Verified by running it**, not by inspection: `examples/assets/morph_face.gltf` (a
  skinned triangle with two named targets, `jawOpen`/`eyeBlinkLeft`, one animation driving
  them and one driving only a joint rotation) and
  `tests/functional/unit/test_animation_morph_import.cpp` — seven cases covering the
  import, the cooked clip's named tracks, the resampled keys against the authored ones
  (including that between-key sampling is linear, not stepped), an animation with no
  `weights` channel yielding no morph tracks, `sample_morph_state` resolving a
  *reverse-ordered* mesh with an undriven extra target (the case a positional mapping
  silently gets wrong), and joint tracks still importing alongside a weights channel. All
  seven pass. This is also the **animation stack's first test under `tests/functional/`**
  — see the standards note below.

### 12.3 The performance floor — device-batched evaluator: CLOSED 2026-07-25, actually verified

`batch_evaluator.hpp` and `animator_evaluator.hpp` were host-side, single-threaded
per-call, with zero `sycl::`/`parallel_for`/`queue.submit` anywhere under
`include/SushiEngine/animation/`. §0.4's 100-hero + 1000-crowd budget assumed a
SushiRuntime task-graph device path for sampling, compose, and skinning that did not
exist. New `include/SushiEngine/animation/device_batch_evaluator.hpp` —
`Animation::DeviceBatchEvaluator` — closes it, following `physics/pgs_solver.hpp`'s
proven `Graph::add(Extent, In/Out(buffers), lambda)` shape:

- **One thread per instance**, each doing the full sequential per-joint sample →
  compose → skin loop inside the kernel — exactly the design's own "parallel across
  instances, sequential 256-max inner loop" from §5.2, just moved on-device instead of
  host `for`. `Mat4 model[MAX_JOINTS]` is per-thread kernel-private scratch.
- **Scope, deliberately narrower than `AnimatorEvaluator`**: one shared skeleton per
  batch (the crowd-LOD case `batch_evaluator.hpp` already assumed), one clip per
  instance — no blend trees, layers, masks, or IK on the device path yet. Hero
  characters (the ones using those) keep running through the host
  `AnimatorEvaluator`; this is the crowd floor §6.6's Low/Medium tiers need.
  `ClipFormat::Raw` only — `bind_clip` rejects a compressed clip (`INVALID_CLIP_HANDLE`);
  ACL-shaped device decode is unimplemented, not silently skipped.
- **Data placement, corrected from the original design's assumption.** §3's "loaded
  into shared-USM `AnimationDatabase`" was audited **false** in §12.1's investigation —
  `AnimationDatabase` stores blobs in plain process heap. Rather than changing that
  widely-used class's storage model, `DeviceBatchEvaluator` owns its own USM-backed
  copies of exactly the skeleton/clip data it batches (`SushiRuntime::API::Buffer<T>`
  SoA arrays), the same choice `ConstraintSolver` makes for constraints.
- **`compile_count() == 1`** across repeated `evaluate()` calls at a fixed instance
  count — the graph only rebuilds when the instance count or skeleton changes,
  matching the engine-wide replay-only invariant.

**Actually built and run this session** — the first animation-system claim in this
document with a real, non-"unverified on hardware" result, because this machine's CPU
SYCL/OpenCL backend (auto-detected by CMake, no discrete GPU needed) is enough to
compile and execute a SushiRuntime graph:

```
[device_batch_evaluator_demo] max host/device position error: 0.000000
[device_batch_evaluator_demo] OK — 37-instance batch matches the host evaluator, compile_count == 1
```

`examples/device_batch_evaluator_demo.cpp` cross-checks a 37-instance batch (mixed
looping/clamped playback times spanning more than one full loop) element-for-element —
every joint, all 16 matrix entries — against the host `ClipEvaluator` on the same
skeleton/clip, plus the `compile_count == 1` replay invariant across repeated
`evaluate()` calls. Zero difference. Registered as `add_sushi_sycl_executable
(device_batch_evaluator_demo ...)` in the root `CMakeLists.txt`, same pattern as
`pgs_demo`. Built via `se build` / raw ninja against the existing `build/` tree; run
directly (needs `sycl8.dll` from the llvm-sycl toolchain's `bin/` on `PATH`, same as
every other SYCL demo binary here).

**Live-scene wiring — CLOSED 2026-07-25.** A real "Crowd" entity kind now exists
end-to-end: `Simulation::CrowdParams` (skeleton/clip/mesh/material handles + playback
state), `ISimulation::register_crowd_skeleton`/`register_crowd_clip` (glTF import into
a private `Animation::AnimationDatabase`, cached by path), `IWorldEditor::create_crowd`/
`has_crowd`/`crowd_params`/`set_crowd_params`, and `RenderScene::skinned_instances`.
`RuntimeSimulation` owns a `DeviceBatchEvaluator` member (constructed from the same
`SushiRuntime::API::Runtime` `World`/`Schedule` already share); `step_crowd_playback()`
advances every playing crowd entity's clip time on the fixed tick, and `extract_crowd()`
(called from the existing `extract()`) finds the frame's bound skeleton (first crowd
entity seen wins; a different skeleton is skipped that frame, not drawn wrong — one
shared skeleton per `DeviceBatchEvaluator` batch, its own documented scoping), lazily
binds any newly-seen clips, runs one `set_instances`+`evaluate()` for every crowd entity
sharing that skeleton, and fills one `Render::SkinnedInstance` per entity, pointing
into the evaluator's own retained palette buffer. `editor/ui/viewport_panel.*` merges
`RenderScene::skinned_instances` with the single-instance `AnimatedMeshPreview` channel
(the same "world content plus the authored subject" pattern already used for particle
billboards/emitters), so both the Scene and Game views draw them.

Architecture note worth remembering: the sim seam (`ISimulation`) had never loaded any
mesh/skeleton/clip content before this — every existing entity kind (Box/Sphere/Cylinder,
lights, decals) is either procedural or references a `Render::` handle a host already
resolved (`TextureId` on `Material` is the exact precedent `CrowdParams::mesh` follows).
Skeleton/clip loading needed real file I/O (`Animation::import_gltf_skeleton`/
`import_gltf_animated`, declared in the engine's animation surface but implemented in
`render/material/gltf_skeleton_importer.cpp`, the one place cgltf is linked) — `sushi_sim`
doesn't link `sushi_render` itself, but the final `se_editor.exe` link does, so the
symbol resolves correctly at the executable link even though `sushi_sim.lib` alone
carries it unresolved. This is intentional per the header's own comment, not a
workaround.

**Verified**: `sushi_sim` (the crowd C++) and the full `se_editor.exe` (crowd C++ +
the earlier dual-quaternion-skinning shader/pass work + the viewport merge) both
compile and link cleanly under this project's `-Wall -Wextra -Werror`.

**A real, unrelated pre-existing bug was found and fixed while chasing this**:
launching `se_editor.exe` hit `vkQueueSubmit failed (VkResult -4)`
(`VK_ERROR_DEVICE_LOST`) very early, before or regardless of `--no-run`.
`vulkaninfo` confirmed a real, enumerable GPU (a GTX 1060) — not a "no GPU"
environment issue. `editor/main.cpp`'s `--validation` CLI flag turned out to be dead
code from a copy-paste mistake (`c6618377`, 2026-07-24): it wrote `desc.width = ...`
instead of `desc.enable_validation = true;` inside the flag's `if`, and — the actual
likely root cause — `desc.width` (the `WindowRendererDesc` the swapchain sizes
against) was consequently **never set outside that dead branch at all**, silently
defaulting to 1280×720 while the real window is created at 1600×900. A swapchain sized
against the wrong extent relative to the actual surface is a plausible, real cause of
an early device loss. Both are now fixed: `--validation` actually enables the layers,
and `desc.width` is always set from the window's real drawable size. **Not able to
confirm the fix**: an unrelated, in-progress refactor elsewhere in the tree
(`physics/contact_solver.hpp`'s `ContactBody::orientation` migrating from value to
pointer, mid-edit, not this session's work) currently blocks the whole engine from
compiling, so the device-lost fix could not be re-tested end-to-end before this entry
was written — verify it once that refactor lands. What remains genuinely unbuilt: the
excluded device-path scope (layers/masks/IK/compressed clips), and the actual 100-hero
+ 1000-crowd timing numbers from §9's budget table (this proves *correctness* and real
*wiring*, not profiled *timing* — the pass profiler isn't hooked to this path yet).

### 12.4 Features never scoped for A0–A9 — genuinely new work

Deliberately out of the original plan (§2's skip list) or never mentioned in it at
all. None of these are bugs; each is a real engineering project roughly A-phase sized.

- **Motion matching — CORE AND BLEND-GRAPH WIRING BOTH CLOSED 2026-07-25, actually
  verified.** New `include/SushiEngine/animation/motion_matching.hpp` —
  `Animation::MotionDatabase` — is the searchable-database-and-nearest-neighbor core
  every fuller motion-matching implementation is built on: `add_clip` samples a clip
  into evenly-spaced (clip, time, feature) candidates via the existing `ClipEvaluator`
  (root velocity by finite difference, plus two optional foot-height proxies — not true
  ground-contact phase, see the header's own honesty note about that gap);
  `find_best` is a weighted brute-force nearest-neighbor search over the pool, with
  independent weights for velocity vs. foot-height so either term can be tested/tuned
  in isolation. **Explicitly not built**: trajectory-window prediction (past/future
  root motion, the actual "matching" half of real motion-matching talks), and a
  spatial index (brute-force is fine at the hundreds-to-low-thousands scale a curated
  set has; a k-d tree is the scaling follow-up once real content outgrows it, not built
  speculatively). `examples/motion_matching_demo.cpp` proves: exact and near-exact
  velocity queries resolve to the right clip; a shared-velocity, different-stance-height
  query is steerable by the foot-height weight alone (proves the two weights are
  independent knobs). **Actually run**, not "unverified on hardware" — this is plain
  host C++, no GPU needed. New `include/SushiEngine/animation/motion_match_sampler.hpp`
  — `MotionMatchSampler` — is the blend-graph wiring this header's own comment used to
  call out as the caller's job: a reference driver that re-searches `find_best` on a
  fixed interval (hysteresis against candidates that tie every tick), and crossfades
  from whatever was already playing into a newly-selected candidate over a configurable
  duration, using the same two-contribution shape `AnimatorEvaluator::pose_layer`
  already uses for state transitions (both clips keep advancing their own playback time
  through the fade). A caller can still ignore this and blend the raw `find_best`
  selection their own way — nothing about `MotionDatabase` requires it — but nobody has
  to write the crossfade bookkeeping from scratch to get a working result now.
  `examples/motion_match_sampler_demo.cpp` proves the search hysteresis actually
  triggers a switch to the right (forward) candidate, that the crossfade starts close
  to the outgoing pose rather than snapping (z=0.0028 at the instant it begins), and
  that the output has clearly moved with the new clip once the configured crossfade
  duration elapses (z=0.4333). Actually run, host C++, no GPU needed.
- **Full-body IK — CLOSED 2026-07-25, actually verified.** New
  `include/SushiEngine/animation/ik_full_body.hpp` — `Animation::FullBodyIk`, a fifth
  `IPoseModifier` alongside the four narrow analytic solvers (two-bone, look-at, FABRIK
  chain, foot placement). General-purpose multi-effector Cyclic Coordinate Descent
  (CCD): an arbitrary list of end effectors, each with its own target and weight, all
  solved from a shared configurable root joint — each effector's ancestor chain (walked
  from the joint nearest its tip up to `root_joint`) gets rotated joint-by-joint to
  reduce that effector's own tip-to-target error, repeated for `iterations` full passes.
  Deliberately simpler than a production full-body IK package: no joint limits/
  constraints, and — documented as a real limitation, not silently — effectors sharing
  a rotatable ancestor can undo each other's adjustment across passes (CCD's known
  weakness; a caller with genuinely conflicting effectors must order/weight them
  deliberately, this solver does not arbitrate). `examples/full_body_ik_demo.cpp`
  proves a 3-joint chain converges to an out-of-plane target (final error 0.000002),
  and — the harder, more distinctive case — two independent limbs sharing a common,
  never-rotated anchor joint (each solved by its own `FullBodyIk` instance with
  `root_joint` set to exclude the shared anchor) both reach their own exact-reach
  targets with zero measurable error and zero interference. **Caught one real test bug
  in review, not a solver bug**: the skeleton cook's topological sort reorders joints
  by depth (`stable_sort`), so two joints at the same depth do not keep their authored
  index order — the first draft of the two-limb test assumed they did and got garbage
  results (a ~2.4 unit error) from targeting the wrong joint entirely; fixed by
  resolving every joint index by name (`SkeletonView::find_joint`) after the cook, the
  same gotcha the design has flagged for `SkinVertex` joint indices elsewhere. Plain
  host C++, no GPU needed — actually built and run, not "unverified."
- **Ragdoll blending / active ragdoll / physical hit reaction — CLOSED 2026-07-25,
  actually verified.** New `include/SushiEngine/animation/ragdoll_blend.hpp` —
  `Animation::RagdollBlend`, an `IPoseModifier` (design §5.3/§11's named seam) — blends
  named joints' local pose toward caller-supplied physics-body object-space transforms
  by a per-joint weight (0 = pure animation, 1 = pure physics), converting each
  absolute target to a local delta against its already-composed parent
  (`inverse(model[parent]) * target`) before blending, then calling `recompose()` so
  the change cascades correctly through any untouched descendants — not a naive direct
  overwrite of `context.model[joint]`, which would silently strand children at their
  old pose. **Explicitly not built**: mapping skeleton joints to `Physics::XpbdSolver`
  bodies, inverse dynamics, or velocity-continuous handoff — this header is the
  *blend*, the physics-to-object-space resolution is the caller's job (the same
  contract `IPoseTaskContext`/`FootPlacementIk` already use). `examples/
  ragdoll_blend_demo.cpp` proves weight 0/0.5/1 blending against a hand-built
  `PoseModifierContext`, and — the property a naive implementation gets wrong —
  that targeting a *parent* joint correctly cascades to an untouched *child*
  (root moved to (10,0,0), child not directly targeted lands at (11,0,0), not left at
  its old (1,0,0)). Actually run, host C++, no GPU needed.
- **Dual-quaternion skinning — CLOSED 2026-07-25, algorithm actually verified,
  `skinning.comp`/`SkinningPass` wiring landed and compiles/links but is pending the
  user's GPU visual confirmation (same status as §12.1's editor GUI work).** New
  `include/SushiEngine/animation/dual_quaternion_skinning.hpp` — `DualQuaternion`,
  `dual_quaternion_from_rigid`, `blend_dual_quaternions` (the Kavan-Collins-Zara-
  O'Sullivan 2007 weighted-sum-then-normalize construction, with the hemisphere
  correction `nlerp` already needs), and `skin_position_dqs`, plus `skin_position_lbs`
  as the linear-blend reference `skinning.comp`'s mat4-weighted-sum path is
  algebraically equivalent to for rigid (no-scale) joint transforms.
  `examples/dual_quaternion_skinning_demo.cpp` proves the actual claim, not just that
  the math runs: the classic bent-elbow case (Kavan et al.'s own motivating example — a
  120° bend about an offset pivot, a vertex on the joint's outer surface weighted 50/50
  between the two bones) shows LBS visibly pinching the vertex toward the pivot
  (distance-from-pivot error 0.414 against a bone-length-1 radius) while DQS preserves
  it exactly (error 0.000000) — the candy-wrapper artifact, reproduced and fixed, not
  asserted. The same computation also runs as a SushiRuntime SYCL kernel over 16 bend
  angles and bit-matches the host reference exactly (max error 0.00000000). **Caught one
  real sign-error bug in review**: the first draft of `skin_position_dqs`'s translation-
  extraction formula had `real.xyz*dual.w - dual.xyz*real.w` where the correct
  Hamilton-product derivation (checked against this codebase's own `mul` convention, not
  assumed from memory) gives `real.w*dual.xyz - dual.w*real.xyz` — the sign flip made
  DQS score *worse* than LBS (error 3.06 vs. 0.41) until the host/device comparison
  caught it.

  **The live wiring** (added after the algorithm closed, once `render/tools/
  shader_compiler` turned out to make even the GLSL side headlessly verifiable — see
  below): `SkinnedInstance` (`render/scene_view.hpp`) gained one opt-in bool,
  `use_dual_quaternion_skinning` (default false, zero behavior change for every existing
  caller). `SkinningSystem::prepare` derives a parallel dual-quaternion palette every
  frame from the existing linear-blend palette it already builds (`skinning_system.cpp`'s
  `dual_quaternion_from_matrix16` — the exact Hamilton-product algebra from the verified
  header, reimplemented in plain floats so the renderer stays independent of the
  `Animation` namespace, per `SkinnedInstance`'s own "only palette floats, never
  animation types" contract) — derived, not a second source of truth, and cheap enough
  to always compute rather than gating it behind the flag. `SkinningPass` grew an 8th
  descriptor binding (the DQ palette, same always-bind-a-valid-buffer fallback contract
  the morph bindings use) and a `use_dual_quaternion` push-constant field.
  `skinning.comp` ported the verified blend/skin functions to GLSL as literal
  translations of the header's own math (`blend_dual_quaternions`, `skin_position_dqs`,
  a shared `rotate_by_quaternion` helper) behind an `if (pc.use_dual_quaternion != 0u)`
  branch — the existing linear-blend path is completely unchanged in the `else` branch.
  Previous-frame motion-vector sampling always uses the linear-blend previous palette
  regardless of the flag (documented tradeoff — DQS's rigidity guarantee is not worth a
  second previous-frame palette at motion-vector precision). The `DualQuaternion` GLSL
  struct is two `vec4`s (32 bytes, naturally aligned) — deliberately avoiding the
  std430 vec3-array padding trap the morph-delta buffer already hit once this project.

  **What "compiles/links" actually means here, precisely**: `render/tools/
  shader_compiler` links `glslang` and turns GLSL into SPIR-V at *build* time (so the
  shipped renderer carries no runtime shader-compiler dependency) — which means it is a
  real, headless, no-GPU-needed compiler this environment can run. `skinning.comp`
  compiled to SPIR-V cleanly on the first attempt after the port (624 lines of generated
  header, no glslang parse/link errors). `skinning_pass.cpp`/`skinning_system.cpp`
  (the new binding/push-constant/palette-derivation C++) compiled and linked cleanly
  into `sushi_render.lib` under this project's `-Wall -Wextra -Werror`. The full
  `se_editor.exe` link initially failed for a reason entirely unrelated to this work —
  a missing `SushiEngine::Editor::load_scene` symbol from another, unrelated,
  in-progress scene-serialization change (`editor/serialization/scene_serializer.cpp`,
  `editor/ui/editor_panels.cpp`) — and resolved on its own once that concurrent session
  progressed further; the editor links clean now.

  **A UI toggle was added once the link unblocked**, so the flag is actually reachable
  from the editor rather than only from code: `AnimatedMeshPreview::
  set_dual_quaternion_skinning`/`dual_quaternion_skinning()`, a "Dual-Quaternion
  Skinning" checkbox in the Animator Preview window (`animator_preview_panel.cpp`),
  and a Statistics panel line reporting which blend mode the loaded preview uses
  (`editor_panels.cpp`). `se_editor.exe` links clean with all of it in. **What is still
  genuinely unverified**: whether the shader actually *looks right* — SPIR-V validity
  and C++ linkage prove the plumbing is wired correctly, not that a bent joint renders
  without artifacts on real geometry. That last check needs the user's own eyes:
  launch the editor, load a rigged character with a bendable limb, and flip the new
  checkbox.
- **Runtime retargeting — CLOSED 2026-07-25, actually verified.** New
  `include/SushiEngine/animation/runtime_retarget.hpp` — `RuntimeRetargeter` — plus a
  new public function in `retarget.hpp` itself, `retarget_pose_frame`, the per-frame
  counterpart to `retarget_clip`'s bind-pose-delta transfer (shared, not duplicated,
  via the existing `detail::pose_delta` helper). Where `retarget_clip` bakes a new clip
  for a *known* target rig once at import, `RuntimeRetargeter` samples a clip against
  its own source skeleton every frame (an ordinary `ClipEvaluator`) and retargets that
  frame onto a target skeleton chosen at runtime — the same-session "swap a character
  model mid-game" case §12.4 named as missing, at the honest cost of redoing the delta
  transfer every frame instead of once offline. `examples/runtime_retarget_demo.cpp`
  is also the first built-and-run check of the retargeting *algebra itself* (phase A8
  shipped `retarget_clip`/`mirror_clip` with no demo of their own): retargeting onto a
  bind-identical clone of the source rig reproduces plain `ClipEvaluator` sampling
  exactly (hand-derivation-free — it validates the delta-transfer math by checking it
  against a case where the correct answer is "unchanged"), and retargeting the same
  clip onto a rig with double hip height and double arm length reproduces the *same*
  joint rotation bit-for-bit while the reach scales to the target's *own* bone length —
  a right-isoceles-triangle distance check (`length * sqrt(2)` for a 90° bend) that is
  correct regardless of the rotation's handedness convention, so it does not depend on
  a possibly-wrong assumption about sign. Actually run, host C++, no GPU needed.
- **Jiggle bones / physics-driven secondary motion — CLOSED 2026-07-25, actually
  verified.** New `include/SushiEngine/animation/jiggle_bone.hpp` — `JiggleBone`, a
  sixth `IPoseModifier`: a single-point-mass spring-damper (VRM SpringBone / Unity
  DynamicBone's model) per configured joint, Verlet-integrated toward the joint's
  "rigid" animated position each frame and distance-constrained back onto its bind
  bone length. **Caught one real conceptual bug in review, the most interesting one
  this session**: the first draft rotated the *configured joint's own* local rotation
  to try to move it — but a joint's own rotation only orients its *children* in a
  skeletal hierarchy (`model[joint] = model[parent] * local_translation[joint]`; the
  joint's own position never reads its own rotation), so the first draft's demo showed
  exactly zero lag no matter what the spring computed — an invisible no-op, not a
  crash, caught only because the demo asserted a *quantitative* lag bound instead of
  just "no crash." Fixed by rotating the joint's **parent** instead (the same ancestor-
  rotates-to-move-a-descendant shape `FullBodyIk`'s CCD already uses) — after the fix,
  `examples/jiggle_bone_demo.cpp` shows a real, non-trivial lag of 1.03 units one frame
  after a sudden 2-unit parent lurch, re-settling to within 0.00 units of the new rigid
  rest 300 frames later, with the bind bone length held exactly throughout. A second
  bug surfaced fixing the first: the class only works correctly when the caller
  recomposes from a fresh, un-jiggled animated pose every frame (as `AnimatorEvaluator`
  already does) — a demo that reused the previous frame's jiggle-mutated rotation as
  next frame's "rest" reading fed back on itself and diverged; documented explicitly in
  the header as the contract, not left implicit. Actually run, host C++, no GPU needed.
- **Facial animation blendshape mapping — CLOSED 2026-07-25, actually verified.** New
  `include/SushiEngine/animation/facial_blendshapes.hpp` — the canonical ARKit-52
  blendshape name set (`ARKitBlendshape`, `arkit_blendshape_name`) and
  `FacialBlendshapeMap`, the same shape as `humanoid.hpp`'s `Avatar` but for morph
  targets instead of joints: which mesh morph target (if any) plays each of the 52
  canonical shapes, resolved once by exact name (ARKit names are already a fixed
  standard, no alias heuristic needed unlike bone naming) and addressed by index every
  frame via `set_facial_blendshape`. `list_missing` reports every canonical shape a
  mesh has no target for — deliberately not a silent no-op, since a rig missing
  `jawOpen` will visibly fail to speak and a caller should be able to find out why
  without a debugger. `examples/facial_blendshapes_demo.cpp` proves table integrity (52
  distinct, non-empty names), resolution against a 6-target stub mesh whose target
  order does NOT match canonical order (proving name-based, not positional,
  resolution), and that setting an unmapped shape is a documented no-op that does not
  corrupt any other `MorphState` slot. **Explicitly not built**: lip-sync tooling
  (phoneme/audio-driven weight generation) and a facial-authoring editor panel — this
  is the name-mapping prerequisite both would need, not a replacement for either.
- **Cinematics / sequencer timeline tooling — CORE CLOSED 2026-07-25, actually
  verified; an authoring UI is still open.** New
  `include/SushiEngine/animation/sequence_timeline.hpp` — `SequenceTimeline`, keyed
  float tracks (`SequenceFloatTrack`, written into an `AnimatorParameterBlock` slot —
  the existing parameter seam `AnimatorEvaluator`'s blend trees already read) plus
  one-shot `SequenceEvent`s dispatched by `advance`, which fires every event crossed in
  `(previous_time, current_time]`, in time order — including firing several events at
  once for a large step, not just the last one a naive "which single event is closest"
  implementation would catch. `evaluate` is a pure function of time (safe to scrub to
  any point, forward or backward, with no hidden state), while `advance` is
  deliberately stateful-by-the-caller (forward-only; a backward scrub fires nothing,
  looping is the caller's job) — the same split every timeline/animation system in the
  industry makes between "where is the playhead" and "what fired since last tick".
  `examples/sequence_timeline_demo.cpp` proves both halves, plus the specific case a
  naive implementation gets wrong (a dropped-frame-sized step crossing two events fires
  both, in order) and the specific case a stateful-everywhere implementation gets wrong
  (scrubbing backward over already-fired events does not refire them). **Explicitly not
  built**: an editor timeline widget, nested sub-sequences, or serialization — this is
  the evaluation core such a layer sits on top of, the same relationship
  `MotionDatabase` has to a full motion-matching feature.
- **Neural/ML-based compression or synthesis — deliberately never attempted.**
  Classical ACL-shaped compression only; not a priority — the classical scheme already
  hits AAA-typical ratios, and unlike every other §12.4 item this genuinely needs a
  training pipeline and a dataset, not an afternoon of engineering. Left as a real,
  explicit non-goal rather than a token stub.

### 12.5 Suggested order of attack

Cheapest-and-most-blocking first, matching §12.1/§12.3 grouping:

1. ~~The evaluator-to-renderer bridge~~ — done 2026-07-25, pending the user's GPU
   build/visual confirmation (§12.1).
2. ~~GPU morph blending~~ — done 2026-07-25, pending the user's GPU build/visual
   confirmation (§12.1); the morph-target asset it needed and the glTF `WEIGHTS`-channel
   import gap it surfaced are both ~~closed 2026-07-30 and verified by test~~ (§12.2).
3. ~~Statistics panel~~ — done 2026-07-25, pending the user's GPU build/visual
   confirmation (§12.1).
4. ~~Mask editor + IK gizmos~~ — both parts done 2026-07-25, pending the user's GPU
   build/visual confirmation (§12.1): part 1 the live layered/masked/IK
   `AnimatorEvaluator` path, part 2 the Animator Preview window + viewport IK gizmo.
5. ~~Device-batched evaluator~~ — done and **actually verified on this machine's CPU
   SYCL backend** 2026-07-25 (§12.3) — the first non-"unverified on hardware" claim in
   this document. Scoped to single-clip crowd batching (no layers/masks/IK/compressed
   clips on-device yet); not wired into a live scene.
6. §12.4, all of it except neural compression. ~~Motion matching (core + blend-graph
   wiring), ragdoll blending, full-body IK, dual-quaternion skinning (blend math),
   runtime retargeting, jiggle bones, facial blendshape mapping, and a sequencer
   timeline core~~ — all done and **actually verified** 2026-07-25 (host C++ for
   everything except dual-quaternion skinning, which also cross-checks bit-exact
   against a SushiRuntime SYCL device kernel — no GPU display needed for any of it).
   Dual-quaternion skinning's `skinning.comp`/`SkinningPass` wiring landed too (opt-in
   flag, compiles/links cleanly including the GLSL itself via headless glslang) — what's
   left there is purely a visual check, not more engineering. One genuine follow-up
   remains: neural/ML compression (needs a training pipeline and dataset, not an
   afternoon of engineering — a real non-goal, not a shortcut).
7. **The regression-test debt — closed, 2026-07-30, core and §12.4 tail alike.**
   Not a feature, a standards gap: `CONTRIBUTING.md` §3.2 says new behavior ships with a
   GoogleTest under `tests/functional/` and says in as many words that a demo under
   `examples/` "does not substitute for a test." Every claim in §8 and §12 rested on an
   `examples/*_demo.cpp` binary; the animation stack — forty-odd headers, every A-phase,
   every §12.4 item — had **zero** tests in the suite. Ten suites now exist, **125 cases,
   all passing**:

   | Suite | Covers | Cases |
   |---|---|---|
   | `Unit_AnimationClip` | skeleton/clip cooks + refusals, sampling contract, compression error bound | 11 |
   | `Unit_AnimatorStep` | state machine, events, root motion, determinism, rollback | 11 |
   | `Unit_AnimationBlendTree` | all five node kinds, unit-partition sweep, capacity bound | 9 |
   | `Unit_AnimationLayers` | fold order, masks, additive, the additive bake | 12 |
   | `Unit_AnimationIk` | seven solvers' convergence and limits, stack order, zero-weight no-op | 20 |
   | `Unit_AnimationMorphImport` | the glTF `weights` lane end to end (§12.2) | 7 |
   | `Unit_AnimationRetarget` | §4.4's avatar mapping, delta transfer, stride scaling, mirroring, the runtime path | 12 |
   | `Unit_AnimationControllerJson` | §4.3's persistence, round-tripped through the compiled blob's bytes | 10 |
   | `Unit_AnimationKeyframe` | §4.2's curves, auto-tangents, the bake, the pose recorder, generic tracks | 14 |
   | `Unit_AnimationAuthoringTail` | §12.4's motion matching, DQS algebra, ARKit-52 mapping, sequencer timeline | 19 |

   The suites were written to pin properties rather than to restate the code, and every round
   of them paid for itself. From the core: **`Unit_AnimatorStep` found that rotation root
   motion had never been implemented** despite §0.1 listing it as met and two document audits
   confirming it (see §12.2), and `Unit_AnimationIk` pinned a real fixed point in the
   jiggle-bone model as a documented limitation instead of a latent surprise.

   From the tail, four more, and the pattern in them is worth naming: **three of the four were
   a comment or a doc describing behaviour the code did not have**, which is the failure mode a
   subsystem verified only by demos accumulates — a demo exercises the path the author had in
   mind, and the prose drifts from the code with nothing to arbitrate.

   - **`apply_generic_tracks` overran a 64-entry stack buffer.** It clamped its dispatch loop
     to `MAX_GENERIC_TRACKS` but sampled with `sample_generic`, which writes one value per
     track the clip carries, and nothing caps a cooked clip's generic-track count. This is the
     *sibling* of the `sample_morph_state` overrun fixed earlier the same day: that fix was
     applied to one instance of a pattern rather than to the pattern, and the second instance
     survived. Reproduced as an access violation against the pre-fix header before the fix was
     accepted, so the regression test is known to regress.
   - **`skin_position_lbs` was not linear blend skinning.** It blended rotation with `nlerp`
     and translation with `lerp`, documented as equivalent to `skinning.comp`'s matrix-weighted
     sum. It is not: `nlerp` yields a genuine rotation, so that form preserves a vertex's
     distance from the twist axis *exactly* and exhibits no candy-wrapper collapse. Since this
     function exists only as the baseline dual-quaternion skinning is measured against, the
     comparison was between the new path and a reference without the defect the new path
     removes. Now transforms per influence and averages the results.
   - **`FootPlacementIk`'s comment described a guard that was never there** ("only plant when
     the ground is at or above the animated foot"). The unconditional behaviour is correct — a
     clip authored on flat ground must step *into* a dip, not hover over it — and an
     unreachable floor needs no guard, because `TwoBoneIk` extends toward an out-of-range
     target without stretching. Both directions and the unreachable case are now pinned.
   - **`SequenceFloatTrack` claimed `evaluate` sorted its keys.** It does not, and the same
     paragraph contradicted itself about where the cost lived. Sorting per evaluate would put a
     copy and a sort on the per-frame path; the requirement is now stated and `sort_keys` /
     `sort_tracks` provided for a bulk-loaded timeline.

   **What is still untested**, and it is one item: **`DeviceBatchEvaluator`'s host/device
   agreement** (§12.4's device path). It is the only piece of the animation stack that needs
   SushiRuntime, so unlike everything above it cannot be compiled and run without the project
   build — which is why it is named here rather than quietly counted as covered.
