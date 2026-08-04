# Animation

This file covers the skeletal-animation stack: the asset formats and import lanes, the
deterministic animator tick, the per-frame evaluation and pose-modifier stack, and the editor
windows that author all of it.

## 1. Skeletal animation (assets, animator, blend trees, layers, IK, retargeting — phases A0–A9)

The skeletal-animation stack (full plan in `docs/design/animation_system.md`) is a
Unity-Mecanim-parity character system split across three domains — an asset domain that cooks
skeletons, clips, and controllers; a deterministic simulation domain that advances animator state
at fixed tick; and a per-frame evaluation domain that samples, blends, and skins. Phase A0 is the
foundation the other domains stand on: the math and the skeleton asset, with no evaluator, ECS
columns, or renderer coupling yet.

**The math seam additions**
(`engine/foundation/core/include/SushiEngine/core/types.hpp`, see
[the value-type seam](foundation.md#2-the-value-type-seam)). Sampling, blending, and IK are all
interpolation and matrix algebra the seam did not previously expose. A0 adds, behind the same one
file, `lerp` (vector and scalar), `nlerp`/`slerp` (both neighbourhood-corrected: they flip a
rotation to the near hemisphere so a blend takes the short arc, and `slerp` degrades to `nlerp`
when the inputs are nearly parallel), `quaternion_from_matrix`, `affine_inverse` (a true inverse
through the adjugate, so it is correct under non-uniform scale — this is what builds inverse-bind
matrices, where the rotation-only transpose short cut would be wrong), and `decompose_transform`
(the TRS decompose that inverts `compose_transform`). The interpolators are element-parametric
(`Vector3T<T>`/`QuaternionT<T>`) so the evaluator can run float while the boundary stays double;
the matrix ops stay at boundary precision because they run at import, not per frame.

**The skeleton asset** (`engine/domain/animation/include/SushiEngine/animation/`). A skeleton is
a flat, immutable, relocatable blob, never a pointer-linked tree. `SkeletonView` (`skeleton.hpp`)
is a non-owning structure-of-arrays view: a topologically sorted parent array with the invariant
`parent[i] < i` (so composing model space is a forward scan, never a pointer chase — this is what
retires the engine's old host-side, non-topological hierarchy), bind-pose local TRS, object-space
inverse-bind matrices (`JointMatrix`, 16 floats, GLSL column-major — the palette layout, distinct
from the double `Matrix4` because object-space joint data never needs the range that forces
double), FNV-1a 64 joint name hashes (`hash.hpp`) for mask / IK / attachment lookup, and a
bone-LOD table.

The `.sushiskel` format (`skeleton_blob.hpp`) is versioned, little-endian, and
position-independent (byte offsets, no pointers): the cook `build_skeleton_blob` sorts joints by
depth so the invariant holds, remaps every parent reference, derives inverse-bind matrices from
the bind pose when the source did not supply them (glTF does), and lays the SoA sections out at
aligned offsets; the load `load_skeleton_blob` validates the header and returns a `SkeletonView`
that aliases the bytes with zero copy or parse.

**The glTF import lane**
(`engine/asset/gltf/include/SushiEngine/gltf/skeleton_import.hpp`). The mesh importer bakes each
primitive into its node's world transform and drops the node graph, skins, and inverse-bind
matrices — the exact data a rig is. `import_gltf_skeleton` is the parallel lane that keeps them:
it reads one skin, turns each joint node's local transform into bind-pose TRS, copies the
inverse-bind matrices through (or lets the cook derive them), and produces a `.sushiskel` blob.
The declaration lives on the engine's animation surface; the implementation lives in the cgltf
lane (`engine/asset/gltf/source/skeleton_importer.cpp`) — the one place cgltf is linked — and
adds no new dependency, since animation is header-only over `core/types.hpp`.

**The database seam** (`animation_database.hpp`). `IAnimationDatabase` is the
dependency-inversion boundary the simulation and evaluator will sit behind — they request a
`SkeletonView` by `AssetId` and never see the importer, the file format, or the byte buffers.
`AnimationDatabase` is the in-memory owner: each blob lives in its own heap buffer so the buffers
do not move as the database grows and the views it hands out stay valid for its lifetime. Clip
and controller views extend this same interface in later phases; the id space is shared across
all three asset kinds.

**The editor overlay** (`applications/editor/source/animation/skeleton_debug_draw.*`).
`SkeletonPreview` imports a rigged glTF and caches its bind-pose model-space joint positions;
`draw_skeleton_overlay` projects those through the Scene panel's camera (the gizmo's exact
view-projection) and paints bones, joint octahedra, and names over the rendered image. The
viewport panel takes it as one optional, defaulted parameter, so no other call site changes. A
"Skeleton Preview" window in the editor drives it — the surface behind A0's "load a rigged glTF,
see its rest pose". The readable names come from the blob's debug name table
(`SkeletonView::joint_name`), added for this and the coming joint inspector; the runtime path
still addresses joints by hash.

`samples/animation/skeleton_demo.cpp` validates the whole data foundation headlessly (TRS
round-trips, the affine inverse, slerp/nlerp behaviour, an out-of-order chain that cooks and
loads to satisfy `parent[i] < i`, name-hash lookup, name-string round-trip, and an identity skin
matrix at the bind pose), and `import_gltf_skeleton` is exercised by
`assets/models/rigged_chain.gltf`. With this, **A0 is complete**; A1 adds single-clip playback
and GPU skinning (a skinned character looping an animation with zero TAA ghosting).

**The evaluation + animator stack (phases A1–A5, CPU cores shipped).** Above the A0 assets sits
the two-halves split of the design doc's §5. The *deterministic tick* (`animator_step.hpp`) is
the interpreter over the compiled `.sushictrl` controller (`animator_controller.hpp`): per layer
it advances normalized time, runs the state machine (Any-State then current, crossfades,
triggers, events), and accumulates root motion — all over trivially-copyable ECS columns
(`animator_components.hpp`), so it is byte-snapshottable and rollback-exact (A3).

The *derived frame pose* (`animator_evaluator.hpp`) is recomputed each frame and never
snapshotted: it resolves each layer's active state to weighted clip contributions, samples and
weight-blends them, folds the layers, composes model space, and builds the object-space palette.
A state's motion is a single clip or a **blend tree** (`blend_tree.hpp`, A4) — five node kinds
(1D, 2D simple/freeform-directional, freeform-cartesian, direct) over a flat node/child array
with a gradient-band pair table baked at compile; the controller blob is v2 to carry it.

Layers fold gated by an **avatar mask** (`avatar_mask.hpp`, A5) — a name-hash-keyed `.sushimask`
resolved to the rig — either override (nlerp) or **additive** (FMA of a delta baked at import by
`additive.hpp`); a layer's weight can be driven by a parameter through `animator_step`. Clip
sampling and compression (`clip*.hpp`, A1/A2) and the batched crowd evaluator with bone-LOD and
update-rate throttling (`batch_evaluator.hpp`, A2) sit behind the same views.

After the layers fold, an ordered **pose-modifier stack** (`pose_modifier.hpp`, A6) runs in model
space between compose and palette — the `IPoseModifier` seam, Unity's pass ordering, so IK
corrects the final blended pose. The shipped solvers are analytic pole-controlled two-bone
(`ik_two_bone.hpp`), weight-distributed cone-clamped look-at (`ik_look_at.hpp`), iteration-capped
FABRIK (`ik_chain.hpp`), and composite foot placement (`ik_foot_placement.hpp`) that rays to the
ground through the `IPoseTaskContext` seam.

Beyond posing joints, a clip carries **morph-weight and generic float tracks** (`.sushianim` v2,
A7): `morph.hpp` maps a clip's morph tracks onto a mesh's target order — by target *name*, so one
clip drives any mesh sharing the naming — and gives the CPU reference of the skin-pass morph
blend. The importer fills those tracks from glTF `weights` animation channels, naming each after
the target it drives (`engine/asset/gltf/source/animation_importer.cpp`;
`GLTFAnimationImport::morph_target_names` reports the mesh's own target order, the order the
render mesh's delta buffer is uploaded in), and `generic_track.hpp` routes generic tracks to an
`IFloatSink` binding registry (material/UI/script hooks).

**Retargeting** (A8) lets one clip library drive many rigs: `humanoid.hpp` maps a skeleton's
joints to canonical `HumanBone`s (by an alias heuristic or an explicit table), and
`retarget.hpp` transfers a clip's bind-pose deltas onto another rig (`retarget_clip`) or mirrors
them left-to-right (`mirror_clip`). **Authoring** (A9) persists a controller as JSON
(`animator_controller_json.hpp`, the only animation header that pulls in nlohmann/json — the
editor's save/load and undo/redo, round-tripping to a byte-identical blob) and previews states
off the loop (`edit_preview.hpp`'s `scrub_to_state`).

Under the *dense* runtime clip sits the *sparse* **keyframe authoring model** (`keyframe.hpp`):
`ScalarCurve` / `QuaternionCurve` (the dope-sheet / curve-editor primitives,
constant/linear/cubic), a `ClipAuthoring` bundle that `bake`s to the dense `ClipDescription`, and
a `PoseRecorder` that captures a live pose into keys (the "record" workflow). The editor's
**Animation window** (`applications/editor/source/animation/animation_panel.*`) is a GUI over it,
Unity's Animation-window shape: it targets the **Hierarchy-selected entity**, and with Record
armed keys its transform as it is moved, or with Record off **drives the object live in the Scene
view** from its keys as the timeline is scrubbed — Bake writes a dense `.sushianim`. The
**Animator window** (`applications/editor/source/animation/animator_graph_panel.*`) is the
Mecanim state-machine graph editor over a `ControllerDescription` — a grid canvas of draggable
state nodes, transition arrows made by right-click ▸ Make Transition To (a `"Exit"` target
compiles to the layer's entry), Entry/Exit/Any-State nodes, a parameter panel, and JSON save/load
through `animator_controller_json.hpp`.

Guarded by ten CTest suites, 125 cases — `Unit_AnimationClip` (the asset formats and the
compressed error bound), `Unit_AnimatorStep` (the state machine, root motion, and the byte-exact
determinism/rollback contract), `Unit_AnimationBlendTree`, `Unit_AnimationLayers`,
`Unit_AnimationIk` (seven pose modifiers including foot placement), `Unit_AnimationMorphImport`,
`Unit_AnimationRetarget`, `Unit_AnimationControllerJSON`, `Unit_AnimationKeyframe`, and
`Unit_AnimationAuthoringTail` (the design doc's §12.4 motion matching, dual-quaternion algebra,
ARKit-52 mapping and sequencer timeline). The headless demos (`clip_demo`,
`animation_benchmark`, `animator_demo`, `blend_tree_demo`, `layered_animation_demo`, `ik_demo`,
`morph_demo`, `retarget_demo`, `authoring_demo`, `keyframe_demo`, all under `samples/animation/`)
remain as runnable examples rather than as the verification of record.

One piece is still untested and is named rather than counted: `DeviceBatchEvaluator`'s
host/device agreement, the only part of the stack that needs SushiRuntime. What remains besides
it is a visual pass on the GPU side:
`engine/presentation/render/source/passes/skinning_pass.*`,
`engine/presentation/render/source/scene/skinning_system.*` and
`engine/presentation/render/shaders/skinning.comp` are built, linked and driven by a live
producer (`Editor::AnimatedMeshPreview`,
`applications/editor/source/animation/animated_mesh_preview.*`, for the authored subject,
`RenderScene::skinned_instances` for crowd entities), but the compute pre-skin, the morph blend
and the opt-in dual-quaternion path have never been looked at on a display. The editor windows
the A9 phase named all shipped (Animation, Animator graph, Animator Preview).
