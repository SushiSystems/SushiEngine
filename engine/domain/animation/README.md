# Animation {#module-animation}

`animation` owns what a character's motion is: the skeleton and clip assets it is authored as,
the animator state machine and blend trees that choose between them each tick, the inverse
kinematics and pose modifiers that correct the chosen pose, and the evaluators that turn all of
it into a skinning palette. The tick is deterministic, so a replayed or rolled-back frame
produces the same pose.

## Tier

`domain` — the second tier in `cmake/EngineLayers.cmake`, so a module here may depend on
`foundation` and on other `domain` modules, and on nothing above.

## Dependencies

- `core` (public) — every pose, track and palette is spelled in the engine's scalar, vector and
  quaternion types.
- `execution` (public) — the batched device evaluator schedules its work through the execution
  seam, and the header a consumer instantiates it from names that seam.

The module is header-only, so its kernels instantiate inside the consuming translation unit —
the safe way to ship device code.

## Public surface

`animation.hpp` is the umbrella a consumer includes; `animation_database.hpp` is the seam that
hands skeletons and clips out by identifier, so nothing downstream sees the importer or the file
format. Headers are relative to `include/SushiEngine/animation/`.

| Group | Headers | Declares |
|---|---|---|
| Assets and identity | `asset_id.hpp`, `hash.hpp`, `skeleton*.hpp`, `clip*.hpp`, `keyframe.hpp`, `generic_track.hpp`, `morph.hpp`, `skin_vertex.hpp`, `animation_database.hpp` | The name hash, the immutable skeleton view, the relocatable clip blob and its compressed form, and the database seam over them. |
| Controller | `animator_controller*.hpp`, `animator_components.hpp`, `animator_step.hpp`, `animator_evaluator.hpp`, `blend_tree.hpp`, `avatar_mask.hpp`, `additive.hpp`, `animation_player.hpp` | The state machine and its JSON authoring form, the deterministic tick, layered and mask-gated evaluation, and the single-clip player component. |
| Pose modifiers | `pose_modifier.hpp`, `ik_*.hpp`, `jiggle_bone.hpp`, `ragdoll_blend.hpp` | The modifier stack: two-bone, full-body, look-at and foot-placement inverse kinematics, secondary bone motion, and the ragdoll blend. |
| Retargeting | `humanoid.hpp`, `retarget.hpp`, `runtime_retarget.hpp` | The humanoid rig description and the offline and run-time retargeting paths over it. |
| Evaluation and skinning | `evaluator.hpp`, `batch_evaluator.hpp`, `device_batch_evaluator.hpp`, `dual_quaternion_skinning.hpp`, `facial_blendshapes.hpp` | Single-character and batched pose evaluation, the device batch path, and the two skinning models. |
| Motion matching | `motion_matching.hpp`, `motion_match_sampler.hpp` | The feature database and the sampler that queries it. |
| Authoring | `edit_preview.hpp`, `sequence_timeline.hpp` | The editor-facing preview state and the sequencer timeline. |

## Tests

Covered by the functional suite in `tests/`: ten `tests/unit/test_animation_*.cpp` files plus
`tests/unit/test_animator_step.cpp` cover the asset formats, keyframe sampling, blend trees,
layer and mask folding, the inverse-kinematics stack, retargeting, controller persistence, the
morph import path and the authoring curves.

The device batch evaluator is the one part with no coverage: it is the only piece that needs a
real device, and the suite runs on machines that may not have one.

## Further reading

- [`animation_system.md`](../../../docs/design/animation_system.md) — the full design, phases A0
  through A9.
- [`domain-animation.md`](../../../docs/architecture/domain-animation.md) — the animation stack,
  clip assets through retargeting.
