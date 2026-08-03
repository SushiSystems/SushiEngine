# animation — the animation stack, phase by phase. Each demo checks its own phase against an
# analytic answer or a host reference, which is what lets the whole chain be verified without a
# character artist or a screenshot.

# skeleton_demo exercises the phase-A0 animation foundations: the math seam additions
# (TRS decompose, affine inverse, slerp/nlerp) and the skeleton cook/load path
# (topological sort, inverse-bind derivation, IAnimationDatabase, bind-pose skinning),
# checked headlessly.
sushiengine_add_sample(skeleton_demo skeleton_demo.cpp)

# clip_demo exercises the phase-A1 clip + evaluator spine: the .sushianim cook/load,
# ClipView sampling (lerp/nlerp with looping), and the ClipEvaluator producing an
# object-space skin palette, checked headlessly against an analytic quarter-circle.
sushiengine_add_sample(clip_demo clip_demo.cpp)

# animation_benchmark measures the phase-A2 crowd: ACL-shaped clip compression plus the
# batched evaluator with bone-LOD and update-rate throttling, over 1100 instances,
# reporting compression ratio and CPU pose time per frame.
sushiengine_add_sample(animation_benchmark animation_benchmark.cpp)

# animator_demo exercises the phase-A3 Animator core: the compiled controller, the
# animator_step interpreter (state machine, crossfades, triggers, events, root motion), and
# its determinism + rollback-replay guarantees, checked headlessly.
sushiengine_add_sample(animator_demo animator_demo.cpp)

# device_batch_evaluator_demo exercises the §12.3 device-batched evaluator: a crowd of
# instances sampled/composed/skinned in one compiled SushiRuntime graph run, cross-checked
# element-for-element against the host ClipEvaluator and an analytic answer, plus the
# compile_count == 1 replay-only invariant.
sushiengine_add_sample(device_batch_evaluator_demo device_batch_evaluator_demo.cpp)

# motion_matching_demo exercises the §12.4 motion-matching database/search core: a pool of
# sampled poses across several analytic-velocity clips, searched by nearest-neighbor against
# a desired velocity (and, independently, a foot-height proxy term).
sushiengine_add_sample(motion_matching_demo motion_matching_demo.cpp)

# ragdoll_blend_demo exercises the §12.4/§11 ragdoll-blend pose modifier: weight-blending a
# joint's local pose toward a physics-supplied object-space transform, and the correct
# parent-to-untouched-child cascade through recompose().
sushiengine_add_sample(ragdoll_blend_demo ragdoll_blend_demo.cpp)

# full_body_ik_demo exercises the §12.4 general-purpose multi-effector CCD solver: a
# single-chain convergence proof and two independent limbs sharing an unrotated anchor,
# each reaching its own target without interfering with the other.
sushiengine_add_sample(full_body_ik_demo full_body_ik_demo.cpp)

# motion_match_sampler_demo exercises the §12.4 motion-matching blend-graph wiring left open
# by motion_matching.hpp: periodic re-search hysteresis and a two-clip crossfade actually
# changing the output pose over the configured duration.
sushiengine_add_sample(motion_match_sampler_demo motion_match_sampler_demo.cpp)

# runtime_retarget_demo exercises the §12.4 same-session retargeting gap: retargeting onto a
# bind-identical clone reproduces direct clip sampling exactly, and retargeting onto a
# differently-proportioned rig reproduces the same joint bend while reaching by the target's
# own bone lengths.
sushiengine_add_sample(runtime_retarget_demo runtime_retarget_demo.cpp)

# jiggle_bone_demo exercises the §12.4 jiggle-bone / secondary-motion spring-damper pose
# modifier: settle-to-rest, the distance constraint holding every frame, real lag on a sudden
# parent lurch, and re-settling once the parent holds still again.
sushiengine_add_sample(jiggle_bone_demo jiggle_bone_demo.cpp)

# dual_quaternion_skinning_demo exercises the §12.4 dual-quaternion skinning blend math: the
# classic bent-elbow candy-wrapper case (LBS visibly pinches, DQS does not), plus a SushiRuntime
# SYCL kernel cross-check that the same header functions bit-match on device.
sushiengine_add_sample(dual_quaternion_skinning_demo dual_quaternion_skinning_demo.cpp)

# facial_blendshapes_demo exercises the §12.4 ARKit-52 canonical blendshape mapping: table
# integrity, name-order-independent resolution against a partial mesh, and missing-shape
# reporting instead of silent no-ops.
sushiengine_add_sample(facial_blendshapes_demo facial_blendshapes_demo.cpp)

# sequence_timeline_demo exercises the §12.4 minimal cinematics/sequencer timeline core:
# pure-function float-track evaluation (scrub-safe) and forward-only event dispatch that
# fires every event crossed in a step, in time order.
sushiengine_add_sample(sequence_timeline_demo sequence_timeline_demo.cpp)

# blend_tree_demo exercises the phase-A4 blend trees: all five node kinds (1D, 2D
# simple/freeform-directional, freeform-cartesian, direct) through the compile -> load ->
# resolve path, nested trees, and a blend-tree state posed end to end by the AnimatorEvaluator,
# checked headlessly against the analytic weights.
sushiengine_add_sample(blend_tree_demo blend_tree_demo.cpp)

# layered_animation_demo exercises the phase-A5 layers/masks/additive: an upper-body aim layer
# over a locomotion base gated by an avatar mask, an animatable layer weight, and an
# import-baked additive layer, checked headlessly.
sushiengine_add_sample(layered_animation_demo layered_animation_demo.cpp)

# ik_demo exercises the phase-A6 IK / pose-modifier stack: analytic two-bone (reach, length
# preservation, pole, out-of-reach clamp), look-at chain aim, FABRIK chain convergence, and
# ground-ray foot placement, plus running a solver through the AnimatorEvaluator's modifier
# stack, checked headlessly.
sushiengine_add_sample(ik_demo ik_demo.cpp)

# morph_demo exercises the phase-A7 morph targets + generic tracks: the .sushianim v2 morph and
# generic float tracks, sampling a clip's morph tracks onto a mesh's target order, the CPU morph
# blend reference, and the generic-track binding registry, checked headlessly.
sushiengine_add_sample(morph_demo morph_demo.cpp)

# retarget_demo exercises the phase-A8 humanoid avatar + retargeting: heuristic bone mapping,
# bind-pose-delta retargeting across differently-proportioned rigs, and left/right mirroring,
# checked headlessly.
sushiengine_add_sample(retarget_demo retarget_demo.cpp)

# authoring_demo exercises the phase-A9 authoring seams: a controller round-tripping through JSON
# to a byte-identical blob (save/load + undo/redo), and edit-mode scrub preview posing a
# blend-tree state off the loop, checked headlessly.
sushiengine_add_sample(authoring_demo authoring_demo.cpp)
# Header-only JSON for animator_controller_json.hpp's save/load round trip.
find_package(nlohmann_json CONFIG REQUIRED)
target_link_libraries(authoring_demo PRIVATE nlohmann_json::nlohmann_json)

# keyframe_demo exercises the animation authoring model: sparse scalar/quaternion keyframe curves
# (constant/linear/cubic), the pose recorder that captures a live pose into keys, baking to the
# dense runtime clip, and morph-curve baking, checked headlessly.
sushiengine_add_sample(keyframe_demo keyframe_demo.cpp)
