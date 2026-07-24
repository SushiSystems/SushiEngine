# Skeletal Animation System — Animation, Animator, Blend Trees, and IK

An engineering plan for a Unity-parity, data-oriented character animation stack: clip
assets and compression, an Animator controller with layered state machines and blend
trees, an IK / pose-modifier stack, and GPU skinning — built on the archetype-chunk ECS,
the SushiRuntime task graph, and the render graph, without breaking determinism,
rollback, replay-only graph compilation, or the temporal core.

Status: **A0 shipped; A1 render pipeline complete (pending GPU build); A2–A9 CPU cores
shipped (A9 editor GUI pending).** A4–A6 add blend trees, mask-gated override/additive layers,
and the IK / pose-modifier stack (two-bone, look-at, FABRIK, foot placement). A7 adds morph +
generic float tracks (`.sushianim` v2) with a binding registry; A8 adds the canonical humanoid
avatar, bind-pose-delta retargeting across proportions, and mirroring; A9 adds the controller
JSON persistence (byte-identical round-trip) and edit-mode scrub preview — only the A9 editor GUI
remains, with the deferred GPU skinning pipeline. A3's compiled controller,
the `animator_step` interpreter (state machine, crossfades, triggers, events, root motion), and
its determinism + rollback guarantee are done
and CPU-verified (`animator_demo`). A2's ACL-shaped compression, batched evaluator,
bone-LOD/update-rate throttling, and the headless benchmark are done and CPU-verified. For A1:
clip asset,
evaluator, glTF animation import, skin-vertex format, and `QualityParams` tier fields done
and CPU-verified. The GPU skinning pipeline is written end to end — `SkinningSystem` +
`SkinningPass` (compute LBS into a transient interleaved vertex buffer), `mesh_skinned.vert`
(previous-pose motion vectors), the `SkinnedInstance` extract channel on `ISceneView::render`,
skinned draw in the opaque pass, and mesh-registry skin buffers — awaiting a GPU build. The
one remaining wire is feeding it: a skinned-mesh import (glTF JOINTS_0/WEIGHTS_0 → mesh
registry) and an app/editor loop that evaluates palettes into `SkinnedInstance`. Companion to
`render_pipeline_refactor.md` (the AAA roadmap); Phase 10 of that document (GPU-driven
geometry) and this plan share the instancing/SoA trajectory and must not diverge.

---

## §0 Goals and hard acceptance criteria

The system is done when every one of these holds. They are contractual, not aspirational.

1. **Authoring parity with Unity's Mecanim surface.** An `Animator` on an entity, driven
   by a controller asset containing layered state machines; states hold clips or blend
   trees (1D, 2D simple-directional, 2D freeform-directional, 2D freeform-cartesian,
   direct); transitions carry conditions over typed parameters (float / int / bool /
   trigger), exit time, duration, offset, and interruption sources; layers carry avatar
   masks and override/additive blending; animation events fire callbacks; root motion
   moves the entity; two-bone IK, look-at IK, and chain IK modify the final pose.
2. **Deterministic where it must be.** Everything that can affect gameplay — parameter
   values, state-machine state, transition progress, normalized times, event firing,
   root-motion deltas — advances only in the fixed-tick simulation, is bit-reproducible
   under `SE_DETERMINISTIC_FP`, lives in trivially-copyable ECS columns, and survives
   `RollbackBuffer` capture/restore byte-exactly. Pose evaluation and skinning are
   derived, per-render-frame data and are explicitly outside the rollback domain.
3. **Replay-only graphs stay replay-only.** After warm-up, `Schedule::compile_count()`
   remains 1 with any number of animated entities spawning and despawning inside
   reserved chunks. Animation never migrates archetypes mid-run and never allocates
   pose memory inside a tick.
4. **Performance floor** (High tier, 1440p output, the roadmap's reference GPU):
   100 unique skinned characters × 80 joints × 2 layers with active IK evaluate and
   skin in ≤ 1.0 ms GPU + ≤ 0.6 ms CPU total; 1000 crowd-LOD instances (≤ 32 effective
   joints, 15 Hz update, no IK) fit in the same budget alongside them. Numbers are
   measured by the pass profiler and a headless animation benchmark, not estimated.
5. **Temporally clean.** Skinned meshes produce correct motion vectors from
   *previous-pose* skinning — zero TAA ghosting on animated characters, verified with
   the temporal debug view. Depth prepass and the forward pass consume identical
   skinned positions (bit-exact), preserving the prepass contract.
6. **Tier-wired on day one.** Max skinned instances, bone-LOD ladder, update-rate
   throttling, IK iteration caps and distance cutoff, influence count (4/8), and morph
   target caps are `QualityParams` fields resolved by `resolve_quality()`; no animation
   code ever reads the raw `RenderQuality` enum.
7. **Memory-honest.** A 60-second, 30 Hz, 80-joint clip compresses to ≤ 250 KB at
   transparent quality (ACL-class ratios); pose pools, palettes, and event queues are
   fixed-capacity, allocated at load, and byte-accountable in the editor's statistics
   panel.
8. **Docs land with code.** Every phase ships its CHANGELOG entry, ARCHITECTURE section,
   and editor surface in the same PR (per CLAUDE.md, restated here because animation
   spans four layers and is the easiest place to let it slide).

---

## §1 Why the engine cannot animate today — audit summary

Numbered so later sections can say which liability they kill.

1. **No skinning data path.** `MeshVertex` (60 bytes: position, normal, tangent, uv0,
   uv1, color) carries no joint indices or weights; no shader reads a joint palette; no
   pass skins. The only deforming geometry is cloth, CPU-triangulated and re-uploaded
   whole every frame — a pattern that cannot scale to characters.
2. **The glTF importer discards the skeleton.** `import_gltf` bakes every primitive into
   its node's world transform and drops the node hierarchy, `skins`, inverse-bind
   matrices, and `animations` entirely — the exact data a skeleton needs.
3. **The math seam has no interpolation.** `core/types.hpp` exposes no `lerp`, `nlerp`,
   `slerp`, no matrix inverse, no TRS decompose, no quaternion-from-matrix. Every one of
   these is load-bearing for sampling, blending, and IK.
4. **Hierarchy is host-side and non-topological.** Parent links live in editor-side
   `RuntimeSimulation::Record`, not in ECS columns; chunk row order is swap-remove
   packed, so parents do not precede children. Nothing today can compose a joint chain
   on the device.
5. **No large per-object per-frame channel.** Per-draw data is a 128-byte push constant
   plus SoA indices (`material_index`, `motion_index`). There is no mechanism to hand
   the renderer N×80 matrices per frame except the cloth full-re-upload anti-pattern.
6. **TAA will ghost anything skinned.** Motion vectors come from
   `previous_model[]` (binding 8) — previous *rigid* transform only. A skinned vertex
   needs previous-*pose* skinning or every animated character smears.
7. **No tier knobs for animation.** `QualityParams` scales shadows, clouds, VRS, and
   BRDF lobes; an animation system added without tier fields would be the first
   subsystem outside the tier contract — the exact debt the contract was built to end.
8. **No asset identity for animation.** There is no clip, skeleton, or controller asset
   type, no binary format, and no editor window that could author one.

---

## §2 Survey conclusions — what we adopt and why

| Source | What we take |
|---|---|
| **Unity Mecanim** | The *authoring model*, verbatim: Animator + controller asset, layered state machines, typed parameters, transition semantics (exit time, duration, interruption source), blend tree taxonomy (1D / 2D×3 / direct), avatar masks, animation events. It is the surface the user asked for and the best-documented authoring contract in the industry. We take the surface, **not** the implementation (per-object C# graph evaluation). |
| **Unity DOTS Animation / Kinemation** | The *evaluation model*: skeletons are flat SoA blobs (parent-index array, topologically sorted), poses are pooled buffers not per-bone entities, evaluation is batched across all instances. Bones-as-entities is explicitly rejected — it bloats archetypes, breaks batching, and would wreck our `compile_count == 1` invariant. |
| **ACL (Animation Compression Library, N. Frechette)** | The compression scheme shape: uniform-time segments, per-segment range reduction, per-track variable quantization, three-smallest-component quaternions, error-driven bit-rate selection measured at the *vertex* (virtual-vertex error metric). We implement the scheme in-engine rather than binding the library (C++17, SYCL-visible decompression, no external dep through `ss`). |
| **Unreal GPU Skin Cache / Decima / Frostbite** | Compute pre-skinning: skin once per character into transient position/normal/tangent streams, then every consumer (depth prepass, forward, shadows, future GPU-driven culling) binds the result as a static mesh. Kills the skin-per-pass cost multiplier and keeps `mesh.vert` untouched (liability 6's bit-exact prepass constraint). Previous-frame skinned positions are written by the same dispatch for motion vectors. |
| **Rune Skovbo Johansen — gradient band interpolation** | The 2D freeform blend weight algorithm (both directional and cartesian variants), the same math Unity ships. Closed form, no runtime triangulation. |
| **GDC "IK Rig" (A. Bereznyak) / Uncharted foot IK** | The IK architecture: a small ordered stack of *pose modifiers* running in model space after blending — analytic two-bone with pole vector, weighted look-at chains, foot placement with ray queries and pelvis adjustment — rather than a monolithic full-body solver. |
| **Overwatch (GDC ECS talks)** | The determinism split: gameplay-authoritative animation *state* advances at fixed tick inside the rollback domain; *pose* is derived per rendered frame from that state plus the interpolation alpha, and is never part of the netcode state. |
| **Skip list** | **Motion matching** (needs a locomotion database pipeline; revisit after A9 — the pose-modifier and clip-sampling seams are designed so it can land as a new state type). **Full-body IK** (Unity's FBIK/FinalIK class solvers; the pose-modifier seam accepts one later). **Unity muscle space** for humanoid retargeting (we retarget by bind-pose delta at import instead — cheaper, deterministic, and loses nothing we need). **Runtime retargeting** (import-time only until a proven need). **Playables-style user graph API** (the controller blob interpreter is the one evaluation path until a second client exists). **Bones as entities** (see above). **Vertex-shader skinning fallback** (one skinning path; tiers scale *inputs*, not implementations). **`vkCmdDrawIndexed`-time morph target textures** (morphs run in the same compute pre-skin dispatch). |

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
  SIMULATION DOMAIN (fixed tick, deterministic, rollback-safe)          ~0.10 ms CPU/GPU
  ECS columns: AnimatorInstance, AnimatorParameterBlock,
               AnimatorLayerState[L], AnimatorEventQueue, RootMotionDelta
  systems: animator_step (state machines, transitions, time, events, root-motion
           sample) ──► apply_root_motion (writes Transform / Orientation)
        │ layer states + times + alpha (read-only snapshot at extract)
        ▼
  EVALUATION DOMAIN (per rendered frame, derived data, no rollback)     ~0.45 ms
  AnimationEvaluator (module-owned SushiRuntime graph, batched over instances):
    blend-tree weights ► clip sampling (decompress) ► layer blend + masks
    ► local pose ► model-space compose ► pose-modifier stack (IK)
    ► skin matrices (model × inverse-bind) ► palette pool (float, object space)
        │ SkinnedInstance records + palette pool via RenderScene
        ▼
  RENDER DOMAIN (render graph)                                          ~0.45 ms GPU
  JointPaletteSystem (per-frame palette SoA, double-buffered for motion)
  ► SkinningPass (compute; positions/normals/tangents + previous positions)
  ► existing passes consume skinned streams as static meshes
```

One-way data flow, four seams (DIP). Each layer names an interface and cannot know
what stands behind it:

- **`Animation::IAnimationDatabase`** — hands out immutable `SkeletonView` /
  `ClipView` / `ControllerView` blobs by asset id. The simulation and evaluator depend
  on views, never on the importer or file formats. (Kills liability 8.)
- **`Animation::IPoseModifier`** — the IK / procedural-pose seam. Stateless
  `apply(PoseContext&)` over a model-space pose; solvers are registered per-Animator as
  an ordered stack. Coarse-grained dispatch (per instance, never per bone).
- **`Animation::IPoseTaskContext`** — what a pose modifier may ask of the world
  (ray query for foot placement, target positions). Backed by physics later; a
  ground-plane implementation ships first. Query results are sampled at extract with
  one-frame latency — no mid-evaluation sync with the physics graph.
- **`Render::SkinnedGeometry` channel on `RenderScene`** — the extract seam. The
  renderer sees palette floats, stream handles, and counts; it cannot see clips,
  controllers, or the evaluator, exactly as it sees cloth vertices today. (Kills 5.)

Domain rules:

- The simulation domain touches only trivially-copyable ECS columns and the immutable
  database — it is rollback-safe by construction (§5.5).
- The evaluation domain owns all heavyweight pose memory (pose pools, palettes) outside
  the ECS; it is recomputed every frame from sim state, so rollback never snapshots it.
- The render domain consumes extracted data only. GPU skinning is a render-graph pass
  with declared reads/writes like any other.

---

## §4 Data model and assets

### 4.1 SkeletonAsset

A flat, immutable blob: `joint_count` (≤ `MAX_JOINTS`, see 4.5), a **topologically
sorted** parent-index array (`parent[i] < i`, sorted at import — kills liability 4 by
construction: compose is a forward scan, never a pointer chase), bind-pose local TRS
(SoA: translations, rotations, scales), inverse-bind matrices (float, object space),
joint name hashes (FNV-1a 64) for mask/IK/attachment lookup, and the bone-LOD table:
per-LOD joint counts where the sort order guarantees every LOD prefix is a valid
skeleton (LOD build reorders siblings so leaf chains — fingers, twist bones — sort
last within their subtree).

### 4.2 ClipAsset and compression

A clip is a set of tracks over a named skeleton: per-joint rotation / translation /
scale tracks, morph-weight tracks, generic float tracks (property-hash addressed,
bound at load through a small registry — no reflection), an event track (normalized
time + name hash + int/float payload), and root-motion metadata (which joint is root,
what was extracted).

Storage is the ACL-shaped scheme (§2): uniform-time segments (~16 frames), per-segment
per-track range reduction, three-smallest-component quaternions, per-track bit rates
chosen by an import-time error solver measured with the virtual-vertex metric against
the raw curves. Constant tracks collapse to one value; identity tracks to one bit.
Decompression is branch-light and SYCL-friendly: a sample touches exactly two segments
worst-case, and one instance's whole-pose sample is a coalesced forward scan.
**Additive clips** are baked at import as deltas against a named reference clip/frame
(Unity semantics), so runtime additive blending is a fused multiply-add, not a
subtraction. (Kills 3's runtime cost and meets acceptance criterion 7.)

### 4.3 ControllerAsset — the compiled Animator

Authored in the editor as JSON (like `.sushiscene`), **compiled to a flat index-linked
blob** at load: parameter table (name hash, type, default), layer table (mask ref,
blend mode, weight), state array (clip-or-tree ref, speed, speed-parameter, cycle
offset, mirror flag reserved), transition array (source, destination, condition span,
exit time, duration, offset, interruption source), condition array (parameter index,
comparator, threshold), and blend-tree node array (kind, parameter indices, children
span with thresholds/positions). Sub-state machines flatten at compile; Any-State
transitions compile to per-state transition spans. Evaluation is a data interpreter
over the blob — no virtual per node, no allocation, device-visible. This is the
OCP seam for new state kinds (a motion-matching state later is a new node kind plus
one interpreter case, not a new class hierarchy).

### 4.4 Import pipeline

`animation/import/` extends the existing cgltf lane: preserve `nodes` as the joint
tree (the current importer's world-baking is bypassed for skinned meshes), read
`skins` (joints, inverse-bind matrices), read `animations` (samplers resampled to
uniform rate at import; cubic-spline sampled through), read morph targets. Skinned
mesh primitives emit the parallel skinning vertex stream (4.5). Humanoid import (A8)
maps joints to the canonical avatar by name-hash table + heuristics and retargets by
bind-pose delta at import. Cooked artifacts: `.sushiskel`, `.sushianim`,
`.sushictrl` — versioned, little-endian, relocatable (offsets, no pointers).

### 4.5 Fixed capacities (documented, asserted, tier-scaled where marked)

| Cap | Value | Where |
|---|---|---|
| `MAX_JOINTS` per skeleton | 256 | asset compile error above |
| `MAX_LAYERS` per controller | 4 | controller compile |
| `MAX_PARAMETERS` | 32 | `AnimatorParameterBlock` column |
| `MAX_EVENTS_PER_TICK` | 8 per entity | `AnimatorEventQueue` column |
| Influences per vertex | 4 (8 on Ultra, tier knob) | skinning stream |
| `MAX_POSE_MODIFIERS` per Animator | 8 | modifier stack |
| Morph targets per mesh | 64 active (tier knob) | skinning dispatch |
| Blend-tree children per node | 16 | controller compile |

---

## §5 Runtime evaluation in detail

### 5.1 Simulation tick — the deterministic half

ECS components (all trivially copyable, all in one shared header registered in fixed
order, archetypes reserved at load — invariant 3):

- `AnimatorInstance` — controller asset id, skeleton asset id, global speed, flags
  (root motion on/off, cull mode).
- `AnimatorParameterBlock` — 32 slots of `{ type, value union }`; written by gameplay
  systems and `on_command`, read by `animator_step`.
- `AnimatorLayerState` × `MAX_LAYERS` — current state index, normalized time, next
  state index, transition normalized progress, transition id, layer weight.
- `AnimatorEventQueue` — fixed ring of fired events this tick, drained host-side at
  the frame barrier into `IAnimationEventSink` callbacks.
- `RootMotionDelta` — position delta + rotation delta produced this tick.

`animator_step` is one per-entity kernel (registered via the ordinary
`app.system<Read<AnimatorParameterBlock>, Write<AnimatorLayerState>, …>` path; the
immutable controller blob base pointer is captured in the lambda — read-only USM data
needs no dependency key). Per layer it: advances normalized time by
`fixed_dt × state_speed × speed_parameter`; evaluates transition conditions in blob
order (first match wins, Unity semantics: Any-State first, then current-state
transitions, honoring exit time and interruption source); steps or starts crossfades;
consumes triggers exactly once; appends events whose normalized time was crossed
(loop-aware); samples the root track at old and new times and writes
`RootMotionDelta`. `apply_root_motion` then writes `Transform` / `Orientation` —
a second system, ordered by the column dependency, so gameplay systems can be
scheduled between them (the `control` → `integrate` idiom).

Everything here is integer/enum state plus a handful of scalars — bit-stable under
`SE_DETERMINISTIC_FP`, byte-snapshottable, replayable. (Meets criterion 2.)

### 5.2 Frame evaluation — blend trees, sampling, layers, compose

At extract, the evaluator snapshots layer states plus the loop's `interpolation()`
alpha (state time is advanced by `alpha × fixed_dt × speed` for sampling only — sim
state is never touched). Evaluation is a short chain of batched kernels on the
module-owned SushiRuntime graph, each over *all visible animator instances*:

1. **Weight resolution.** Interpret each active state's blend tree against the
   parameter block: 1D = segment lerp over sorted thresholds; 2D simple-directional =
   angular sector weights; 2D freeform = gradient band interpolation (precomputed
   per-pair data baked into the blob at compile); direct = parameter-per-child.
   Output: per instance, a flat span of (clip id, time, weight) contributions —
   crossfades contribute both states' sets scaled by transition progress.
2. **Sample + blend.** For each contribution, decompress the pose at its time and
   accumulate weighted into the instance's local-pose slot: translations/scales lerp;
   rotations neighborhood-corrected weighted nlerp (slerp reserved for 2-input
   crossfades where it is visibly better — both provided by the A0 math additions).
   Layers then fold in mask-gated: override layers nlerp toward their pose by layer
   weight; additive layers FMA their baked deltas. Masks are per-joint weight arrays
   baked from the avatar mask asset.
3. **Compose.** Model-space pose by forward scan over the topologically sorted parent
   array (parallel across instances, sequential 256-max inner loop — measured, not
   assumed; the batch keeps the device busy).
4. **Pose-modifier stack** (§5.3), model space, in place.
5. **Palette build.** `skin[i] = model_pose[i] × inverse_bind[i]`, written as floats
   into the frame's palette pool slot. Palettes are **object space** — the camera-
   relative subtraction stays entirely in the per-instance model matrix, so double
   precision never enters pose data (kills the world-anchored-cache concern by
   never anchoring poses to the world).

Bone LOD and update-rate throttling select per instance before step 1: distance/
screen-coverage bucket → joint-count prefix (4.1) and update rate (60/30/15 Hz,
round-robin phased); throttled instances reuse their last composed pose, and the
palette interpolates between last two poses to avoid strobing. Buckets and rates are
`QualityParams` fields. (Meets criteria 4 and 6.)

### 5.3 The IK / pose-modifier stack

Ordered `IPoseModifier` stack per Animator, run in model space after layer blending —
so IK corrects the *final* animated pose, Unity's pass ordering. Shipped solvers:

- **TwoBoneIk** — analytic (law of cosines), pole-vector controlled, soft-clamped near
  full extension, per-solver weight for blending in/out. Arms and legs.
- **LookAtIk** — weighted chain aim (spine→head→eyes distribution weights), clamped
  cone, Unity `SetLookAtPosition` semantics.
- **ChainIk** — FABRIK, iteration-capped (tier knob), for tails/tentacles/cables.
- **FootPlacementIk** — composite: ray queries through `IPoseTaskContext` (sampled at
  extract, one-frame latency), per-foot two-bone solve, ankle re-orientation to
  surface normal, pelvis height adjustment; weights driven by curves or foot-phase
  events from the clip.

Closed-form solvers run inside the batched kernel chain (branchless, grouped by solver
type); FABRIK chains run as `add_host` tasks ordered against the pose buffers (the
runtime's designed lane for iterative host work). IK is distance-gated and
iteration-capped by tier (kills liability 7 for IK specifically).

### 5.4 Pose pools and graph integration

All pose-sized memory — local poses, model poses, palettes (current + previous) — lives
in module-owned shared-USM `Buffer`s sized at load from reserved instance counts
(tier knob), sliced per instance by stable offsets. Instances claim slots at spawn and
release at destroy; slots are stable for an instance's lifetime, so the evaluator
graph's dependency keys (the pool base pointers) never change and its graph also
compiles once. Nothing here is an ECS column: the ECS carries only the small
deterministic state (5.1) plus a `PoseSlot { uint32 offset }` handle component.
(Preserves invariant 3; keeps rollback snapshots small.)

### 5.5 Determinism and rollback — the contract, restated precisely

- Rollback snapshots capture animator columns byte-exactly; restoring them restores
  the state machine perfectly. The evaluator reads sim state only at extract, so a
  rolled-back-and-replayed tick range produces identical animator state, identical
  root motion, identical re-fired events (event delivery is tick-keyed and
  deduplicated across reconciliation replays, the same rule as `on_command`).
- Pose pools, palettes, and skinned streams are never snapshotted — derived data.
- No archetype migration: enabling/disabling animation at runtime is a flag in
  `AnimatorInstance`, not component add/remove (invariant 3, rollback `Chunk*` keying).
- `animator_step` uses no wall clock, no `Math.random` — time is `fixed_dt`, variation
  is `RngState` if ever needed.

---

## §6 Render integration

### 6.1 Vertex streams — `MeshVertex` stays frozen

Skinning attributes ship as a **parallel stream**, not a `MeshVertex` change:
`SkinVertex { uint8/uint16 joints[4|8]; unorm8 weights[4|8] }` in its own VkBuffer per
skinned mesh (kills liability 1 without touching the 15 passes bound to the base
format). Morph target deltas ship as packed per-target position/normal delta buffers.

### 6.2 SkinningPass — compute pre-skin, once per character

A new render-graph pass (`render/passes/skinning_pass.*`, `register_pass` like the
other 15) early in the frame: one dispatch per skinned instance batch reads base
vertices + skin stream + morph deltas + current and previous palettes, writes
transient `skinned_position/normal/tangent` streams **and `skinned_prev_position`**.
Graph-declared writes give every consumer its barrier for free. Depth prepass, opaque,
and shadow passes bind the skinned streams through the existing vertex-input path —
`mesh.vert` gains only a "previous position from stream if skinned" branch keyed off a
push-constant flag, keeping prepass/forward bit-exact by construction (kills
liabilities 1 and 6; skin cost is paid once, not per pass). Linear-blend skinning
ships first; the dispatch is structured so dual-quaternion is a later specialization
constant, not a redesign.

### 6.3 JointPaletteSystem and motion vectors

`render/scene/joint_palette_system.*`, modeled on `MotionSystem`: a per-frame palette
SoA storage buffer, double-buffered so the previous frame's palettes survive for 6.2's
prev-position skinning.

> **Correction (A1 implementation).** This section originally placed the palette at
> **scene-set binding 14**. By the time animation landed, the scene set (set 0) was a
> push-descriptor set **full at 32 bindings** (`SceneLayout::BINDING_COUNT == 32`, with
> binding 14 taken by `LIGHT_BINDING`), so the palette cannot ride set 0 — exactly the
> slot pressure §11 warned about. It does not need to: under the compute-pre-skin model
> (§6.2) the *graphics* passes consume the skinned vertex streams as static meshes and
> never read the palette. The palette is therefore a **SkinningPass-local** resource,
> bound to that compute pass's own descriptor set, current + previous. `mesh.vert` reads
> a skinned stream's previous position (a vertex input, keyed off a push-constant flag),
> not a palette. No scene-set binding is added. Instances still reference their palette
> slice by a stable `palette_offset`, which folds into Phase 10's instance SoA as planned.

### 6.4 Bounds and culling

Skinned bounds = bind-pose bounds inflated by a per-clip factor baked at import
(conservative, zero per-frame cost); IK-active instances add a fixed margin. Bounds
feed the existing per-view culling and, later, Phase 10's GPU culling unchanged.
Camera-relative as all bounds are.

### 6.5 Morph targets

Weights are clip tracks (4.2) blended like any track, uploaded per instance; applied
in the SkinningPass dispatch before joint blending. Active-target cap per tier.

### 6.6 Quality tiers (`QualityParams` additions)

| Knob | Low | Medium | High | Ultra |
|---|---|---|---|---|
| Max full-rate skinned instances | 32 | 64 | 128 | 256 |
| Bone LOD bias | +2 | +1 | 0 | 0 |
| Update-rate ladder (Hz by bucket) | 30/15/10 | 60/30/15 | 60/30/15 | 60/60/30 |
| IK | off > 15 m | on, 2 it. | on, 4 it. | on, 8 it. |
| Influences | 4 | 4 | 4 | 8 |
| Active morphs / mesh | 8 | 16 | 32 | 64 |

High is the authored baseline per the tier contract; the table follows the resolver's
existing shape.

---

## §7 Editor integration

Ships incrementally (each phase's surface listed in §8):

- **Skeleton debug draw** — joint octahedrons + names overlay in the scene viewport;
  selected-joint inspector.
- **Animator window** — Mecanim-style state-machine graph editor (states, transitions,
  live state highlight in play mode), layer list, parameter panel with live editing.
- **Animation window** — dope sheet + curve view over a selected clip; event track
  editing; preview scrubbing in edit mode (evaluator runs on demand outside the loop).
- **Blend tree inspector** — thresholds/positions editing with a live 2D weight
  visualization; per-child preview.
- **IK gizmos** — targets and pole vectors as draggable gizmos; per-solver weight
  sliders.
- **Inspector components** — Animator (controller/avatar slots), Skinned Mesh Renderer
  (mesh + skeleton binding), statistics panel rows (pose pool bytes, palette bytes,
  compression ratios, per-stage ms from the profiler).
- `.sushiscene` gains Animator/skinned-renderer serialization; undo/redo covers
  controller edits through the existing whole-snapshot mechanism.

---

## §8 Phased roadmap

Prefix **A**. Each phase is independently shippable, lands tier wiring + editor
surface + profiler budget + CHANGELOG/ARCHITECTURE in the same PR, and none breaks
`compile_count == 1` or determinism tests.

- **A0 — Math seam + skeleton foundations (triage). ✅ SHIPPED.** Add to the seam through
  `core/types.hpp`: `lerp`, `nlerp`, `slerp` (neighborhood-corrected), quaternion-
  from-matrix, affine inverse, TRS decompose — parametric on scalar type (evaluation
  runs float; boundary stays double). `SkeletonAsset` + blob loader +
  `IAnimationDatabase`. Importer preserves node hierarchy + skins for skinned meshes.
  Editor: skeleton debug draw. *Ships: load a rigged glTF, see its rest pose.*
  (Kills liabilities 2, 3, 4.)
- **A1 — Single-clip playback + GPU skinning, temporally clean.** Uncompressed
  `ClipAsset` (raw resampled tracks), simple `AnimationPlayer` component (legacy-
  Animation parity: play/loop/speed), evaluator skeleton chain (sample → compose →
  palette), pose pools, `SkinnedGeometry` extract channel, `JointPaletteSystem`
  (binding 14), `SkinningPass` with prev-position stream, skin vertex stream import,
  first `QualityParams` fields. Acceptance criterion 5 is A1's exit test.
  *Ships: a character looping an animation with zero TAA ghosting.* (Kills 1, 5, 6, 7.)
- **A2 — Compression + batching + LOD. ✅ CPU CORE SHIPPED.** ACL-shaped compression with the
  virtual-vertex error solver (`clip_compress.hpp`, `SUSHACMP` blob decoded transparently by
  `ClipView`); batched multi-instance evaluation, bone LOD + update-rate throttling
  (`batch_evaluator.hpp`); headless `animation_benchmark`. Verified 6.6×–17.6× compression,
  correct round-trip, and the 350/1100 re-pose throttle. Remaining: the editor statistics-panel
  rows (the benchmark already exposes the numbers), and porting the batched pose from the naïve
  header loop onto the SIMD/runtime-graph device path the §9 CPU budget assumes.
- **A3 — Animator core: parameters, state machine, events, root motion. ✅ CPU CORE SHIPPED.**
  `ControllerAsset` blob compiler + interpreter (`animator_controller.hpp`); `animator_step` /
  `apply_root_motion` (`animator_step.hpp`); crossfades; trigger semantics; event queue +
  `IAnimationEventSink`; deterministic trivially-copyable columns (`animator_components.hpp`).
  Determinism + rollback proven byte-exact by `animator_demo` (criterion 2's exit met CPU-side).
  Remaining: the `ControllerAsset` JSON authoring (the `.sushiscene`-style editor serialization
  of `ControllerDesc`), registering `animator_step`/`apply_root_motion` as ECS systems in the
  loop, the editor parameter panel + live state readout, and rotation root motion (translation
  ships; rotation is a refinement).
- **A4 — Blend trees. ✅ CPU CORE SHIPPED.** All five node kinds — 1D, 2D
  simple-directional, 2D freeform-directional, 2D freeform-cartesian, direct — as a flat
  node/child array plus a compile-time gradient-band pair table (`blend_tree.hpp`); a state
  holds a clip *or* a blend tree (`StateRecord::blend_tree`, controller blob v2); nested trees
  flatten; the `AnimatorEvaluator` samples and weight-blends the resolved contributions with
  per-contribution normalized-time sync. `blend_tree_demo` checks all five kinds' weights
  against the analytic answer and poses a 1D locomotion state end to end. Remaining: the editor
  blend-tree inspector + 2D visualizer (GPU/editor-side), and porting weight resolution onto the
  batched device path. *Ships: WASD locomotion blend space.*
- **A5 — Layers + masks + additive. ✅ CPU CORE SHIPPED.** Avatar mask asset
  (`.sushimask`, name-hash keyed, skeleton-resolved — `avatar_mask.hpp`); mask-gated layer fold
  (override nlerp / additive FMA) in `AnimatorEvaluator`; import-time additive baking against a
  reference pose (`additive.hpp`); layer weight animatable from a parameter
  (`LayerRecord::weight_parameter`, applied in `animator_step`). `layered_animation_demo` proves
  the shipped upper-body aim layer over locomotion, the halfway weight blend, and the baked
  additive delta. Remaining: the editor mask editor + per-layer weight UI (editor-side).
  *Ships: upper-body aim layer over locomotion.*
- **A6 — IK / pose-modifier stack. ✅ CPU CORE SHIPPED.** The `IPoseModifier` +
  `IPoseTaskContext` seams (`pose_modifier.hpp`); the stack runs in model space between compose
  and palette in `AnimatorEvaluator`. Solvers: analytic pole-controlled `TwoBoneIk`
  (`ik_two_bone.hpp`, law of cosines, soft-clamp beyond reach, per-solver weight); chain-aim
  `LookAtIk` (`ik_look_at.hpp`, weight-distributed, cone-clamped); FABRIK `ChainIk`
  (`ik_chain.hpp`, iteration-capped); composite `FootPlacementIk` (`ik_foot_placement.hpp`,
  ray → two-bone → ankle-to-normal). `ik_demo` checks reach, length preservation, aim, FABRIK
  convergence, and foot planting numerically, plus the evaluator stack path. Remaining: the
  editor gizmos, pelvis-height (cross-foot) adjustment, and grouping the closed-form solvers
  into the batched device kernel. *Ships: foot-planted character on uneven ground, aiming at a
  target.*
- **A7 — Morph targets + generic tracks. ✅ CPU CORE SHIPPED.** The `.sushianim` blob is v2:
  morph-weight and generic float tracks alongside the joint tracks, sampled through the same
  `ClipView`. `morph.hpp` maps a clip's morph tracks onto a mesh's target order by name, holds
  the per-instance `MorphState`, and gives the CPU reference of the SkinningPass morph blend;
  `generic_track.hpp` routes generic tracks to an `IFloatSink` binding registry (material/UI/script
  hooks). `morph_demo` verifies all of it. Remaining: the skin-pass dispatch that applies the
  vertex deltas (GPU), and morph/generic on the *compressed* clip path.
- **A8 — Humanoid avatar + import retargeting. ✅ CPU CORE SHIPPED.** `humanoid.hpp` — the
  canonical `HumanBone` set and an `Avatar` mapping built by name (explicit table or an alias
  heuristic). `retarget.hpp` — bind-pose-delta `retarget_clip` across differently-proportioned
  rigs (root translation scaled by hip height) and `mirror_clip` (opposite bone + sagittal
  reflection). `retarget_demo` proves mapping, retarget, and mirror. Remaining: wiring these into
  the glTF import lane and the humanoid muscle-space refinement. *Ships: one clip library driving
  differently-proportioned rigs.*
- **A9 — Authoring suite completion. ◐ CPU SEAMS SHIPPED (editor GUI pending).** The
  headless-verifiable seams: `animator_controller_json.hpp` — `ControllerDesc` ⇄ JSON (the
  editor's save/load and undo/redo snapshot), round-tripping to a byte-identical blob; and
  `edit_preview.hpp` — `scrub_to_state` pins a state at an explicit time so the `AnimatorEvaluator`
  poses it off the loop. `authoring_demo` verifies both. Remaining (the editor GUI push): the
  Animator graph window, the dope sheet / curve editor, and live preview scrubbing UI. *Ships: a
  character animated end-to-end without leaving the editor.*
- **Future (post-A9, seams already shaped for them):** motion matching as a state
  kind, ragdoll blending as a pose modifier over the existing XPBD bodies,
  dual-quaternion skinning, SYCL↔Vulkan palette interop (roadmap Phase 11), Phase 10
  instance-SoA merge.

---

## §9 Performance budget summary

High tier, 1440p output, the roadmap's reference GPU; 100 hero characters (80 joints,
2 layers, IK) + 1000 crowd (≤ 32 joints, throttled). Enforced by the A2 benchmark and
the pass profiler.

| Stage | Budget | Domain |
|---|---|---|
| `animator_step` + root motion (1100 instances) | 0.10 ms | sim tick |
| Weight resolution + sampling + layer blend | 0.25 ms | evaluator |
| Compose + palette build | 0.10 ms | evaluator |
| IK stack (100 instances) | 0.10 ms | evaluator |
| Palette upload + extract | 0.05 ms | CPU |
| SkinningPass (compute) | 0.30 ms | GPU |
| Marginal cost in prepass/opaque/shadows | ≤ 0.15 ms | GPU |
| **Total** | **≤ 1.05 ms** | |
| Memory: pose pools + double palettes (1100 inst.) | ≤ 24 MB | |

---

## §10 SOLID

- **SRP.** Each stage is one module with one reason to change: blob compiler,
  state-machine interpreter, blend-weight resolver, clip sampler, layer blender,
  composer, each IK solver, palette system, skinning pass. The importer knows formats;
  the database knows storage; the evaluator knows math; none knows another's job.
- **OCP.** New blend-node kinds and state kinds extend the compiled node table and its
  interpreter case list without touching existing states; new IK solvers implement
  `IPoseModifier` and register — the stack never changes; new tier behavior is new
  `QualityParams` fields through the existing resolver.
- **LSP.** Every `IPoseModifier` is substitutable in any stack position (model-space
  pose in, model-space pose out, weight-blendable, no hidden ordering contract beyond
  the stack's). Every clip source behind `IAnimationDatabase` yields views with
  identical sampling semantics.
- **ISP.** Narrow seams per consumer: event consumers see only `IAnimationEventSink`;
  IK sees only `IPoseTaskContext` (not physics); the renderer sees only the
  `SkinnedGeometry` channel; the editor authors through the JSON/compiler boundary,
  never the runtime blob. No interface forces an implementer to stub what it doesn't
  use.
- **DIP.** Simulation and evaluation depend on database *views*, not importers or
  files; the renderer depends on extracted records, not the evaluator; pose modifiers
  depend on an abstract query context, not the physics engine; and interface dispatch
  stays at instance granularity — SOLID at the seams, data-oriented inside them.

---

## §11 Dependencies on the main roadmap

- **Phase 3 (foundation hardening)** — binding 14 claims the next scene-set slot; the
  binding-future-proofing work (3.5) must count it. Tier fields ride the shipped
  resolver.
- **Phase 10 (GPU-driven geometry)** — `palette_offset` is designed to fold into the
  instance SoA records; SkinningPass output streams are the input mesh-shader/meshlet
  paths will consume. Coordinate before either lands its buffer layouts.
- **Phase 11 (async compute / SYCL interop)** — evaluator palettes are the first
  candidate for zero-copy SYCL→Vulkan; until then they cross via the upload path.
- **Physics / XPBD** — `IPoseTaskContext`'s physics-backed implementation and future
  ragdoll blending consume the existing solver; nothing in A0–A9 blocks on it.
- **SushiBLAS** — all A0 math lands behind the `core/types.hpp` seam, so the swap
  remains one file.
