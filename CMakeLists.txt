cmake_minimum_required(VERSION 3.20)

# SushiEngine — the head; SushiRuntime is the plugged-in component (the battery).
#
# This file is intentionally thin: it orchestrates a sequence of focused modules
# under cmake/, each with a single responsibility, rather than doing the work
# itself. The ordering matters and is the one thing this file owns:
#   * Vcpkg + Toolchain run BEFORE project() — they shape the toolchain file and
#     the compiler that project()'s probe validates. As the top-level project we
#     reach project() first, so the SYCL compiler and vcpkg toolchain must be
#     picked here; the runtime's own selection (in the Runtime module below) would
#     otherwise run too late to take effect.
#   * everything else runs AFTER project().
# See cmake/ for the details of each stage.
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# --- Pre-project: toolchain shaping -------------------------------------------
include(Vcpkg)      # (Windows) locate vcpkg + set the toolchain file
include(Toolchain)  # pick the default SYCL compiler (clang++) before the probe

project(SushiEngine VERSION 0.1.0 LANGUAGES CXX)

# --- Post-project: options, runtime, SYCL policy ------------------------------
include(ProjectOptions)  # C++17 baseline, cross-cutting options
include(Runtime)         # add_subdirectory(sushiruntime) -> sushiruntime target,
                         # add_sycl_to_target(), SR_SYCL_TOOLCHAIN
include(SyclTarget)      # sushi_apply_sycl() / add_sushi_sycl_executable() helpers

# --- The engine + its examples ------------------------------------------------
# The engine is header-only so its SYCL kernels instantiate inside the consuming
# translation unit (each example below), which is the safe way to ship device code.
add_library(SushiEngine INTERFACE)
target_include_directories(SushiEngine INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(SushiEngine INTERFACE sushiruntime)

# Determinism guard rail (see cmake/ProjectOptions.cmake): forbid the compiler from
# reordering floating-point operations or fusing multiply-adds, either of which can
# change a result's bit pattern between builds and would silently break replay.
# clang/gcc only — MSVC's equivalent (/fp:strict) is not wired up here because the
# engine's primary compiler is the intel/llvm clang++ SYCL toolchain.
target_compile_options(SushiEngine INTERFACE
    $<$<AND:$<BOOL:${SE_DETERMINISTIC_FP}>,$<CXX_COMPILER_ID:Clang,AppleClang,GNU>>:-fno-fast-math;-ffp-contract=off>)

# The sandbox is the ECS worked example; pgs_demo/xpbd_demo are the physics
# constraint solvers (plain PGS and its rigid-body XPBD generalization).
add_sushi_sycl_executable(sandbox sandbox/main.cpp)
add_sushi_sycl_executable(pgs_demo examples/pgs_demo.cpp)
add_sushi_sycl_executable(xpbd_demo examples/xpbd_demo.cpp)
add_sushi_sycl_executable(cloth_demo examples/cloth_demo.cpp)
add_sushi_sycl_executable(net_demo examples/net_demo.cpp)

# first_game is the smallest complete game written against the settled SushiLoop
# authoring API (Loop::App): pure-ECS systems, a fixed-step deterministic loop, and
# multiplayer reached by one connect() — headless and self-checking.
add_sushi_sycl_executable(first_game examples/first_game.cpp)

# ui_demo exercises the ECS UI (canvas + button): anchor layout, pointer-driven button
# states, and click callbacks, checked headlessly.
add_sushi_sycl_executable(ui_demo examples/ui_demo.cpp)

# soft_body_demo exercises the volumetric XPBD lattice and positional contact
# resolution (Phase 4 physics), checked headlessly.
add_sushi_sycl_executable(soft_body_demo examples/soft_body_demo.cpp)

# skeleton_demo exercises the phase-A0 animation foundations: the math seam additions
# (TRS decompose, affine inverse, slerp/nlerp) and the skeleton cook/load path
# (topological sort, inverse-bind derivation, IAnimationDatabase, bind-pose skinning),
# checked headlessly.
add_sushi_sycl_executable(skeleton_demo examples/skeleton_demo.cpp)

# clip_demo exercises the phase-A1 clip + evaluator spine: the .sushianim cook/load,
# ClipView sampling (lerp/nlerp with looping), and the ClipEvaluator producing an
# object-space skin palette, checked headlessly against an analytic quarter-circle.
add_sushi_sycl_executable(clip_demo examples/clip_demo.cpp)

# animation_benchmark measures the phase-A2 crowd: ACL-shaped clip compression plus the
# batched evaluator with bone-LOD and update-rate throttling, over 1100 instances,
# reporting compression ratio and CPU pose time per frame.
add_sushi_sycl_executable(animation_benchmark examples/animation_benchmark.cpp)

# animator_demo exercises the phase-A3 Animator core: the compiled controller, the
# animator_step interpreter (state machine, crossfades, triggers, events, root motion), and
# its determinism + rollback-replay guarantees, checked headlessly.
add_sushi_sycl_executable(animator_demo examples/animator_demo.cpp)

# particle_demo exercises the phase-VFX1 particle system's CPU-verifiable half: the authoring
# model, the emitter compiler's POD boundary, and the deterministic CPU backend's spawn/lifetime
# accounting plus its determinism and rollback-replay guarantees, checked headlessly. The GPU
# cosmetic backend is validated visually in the editor.
add_sushi_sycl_executable(particle_demo examples/particle_demo.cpp)

# blend_tree_demo exercises the phase-A4 blend trees: all five node kinds (1D, 2D
# simple/freeform-directional, freeform-cartesian, direct) through the compile -> load ->
# resolve path, nested trees, and a blend-tree state posed end to end by the AnimatorEvaluator,
# checked headlessly against the analytic weights.
add_sushi_sycl_executable(blend_tree_demo examples/blend_tree_demo.cpp)

# layered_animation_demo exercises the phase-A5 layers/masks/additive: an upper-body aim layer
# over a locomotion base gated by an avatar mask, an animatable layer weight, and an
# import-baked additive layer, checked headlessly.
add_sushi_sycl_executable(layered_animation_demo examples/layered_animation_demo.cpp)

# ik_demo exercises the phase-A6 IK / pose-modifier stack: analytic two-bone (reach, length
# preservation, pole, out-of-reach clamp), look-at chain aim, FABRIK chain convergence, and
# ground-ray foot placement, plus running a solver through the AnimatorEvaluator's modifier
# stack, checked headlessly.
add_sushi_sycl_executable(ik_demo examples/ik_demo.cpp)

# morph_demo exercises the phase-A7 morph targets + generic tracks: the .sushianim v2 morph and
# generic float tracks, sampling a clip's morph tracks onto a mesh's target order, the CPU morph
# blend reference, and the generic-track binding registry, checked headlessly.
add_sushi_sycl_executable(morph_demo examples/morph_demo.cpp)

# retarget_demo exercises the phase-A8 humanoid avatar + retargeting: heuristic bone mapping,
# bind-pose-delta retargeting across differently-proportioned rigs, and left/right mirroring,
# checked headlessly.
add_sushi_sycl_executable(retarget_demo examples/retarget_demo.cpp)

# authoring_demo exercises the phase-A9 authoring seams: a controller round-tripping through JSON
# to a byte-identical blob (save/load + undo/redo), and edit-mode scrub preview posing a
# blend-tree state off the loop, checked headlessly.
add_sushi_sycl_executable(authoring_demo examples/authoring_demo.cpp)

# keyframe_demo exercises the animation authoring model: sparse scalar/quaternion keyframe curves
# (constant/linear/cubic), the pose recorder that captures a live pose into keys, baking to the
# dense runtime clip, and morph-curve baking, checked headlessly.
add_sushi_sycl_executable(keyframe_demo examples/keyframe_demo.cpp)

# --- Optional subdirectories --------------------------------------------------
# The editor presents through the Vulkan renderer, so turning the editor on turns
# the renderer on too. Render must be added before the editor so the sushi_render
# target exists when the editor links it.
if(SE_BUILD_EDITOR)
    set(SE_BUILD_RENDER ON)
    set(SE_BUILD_INPUT ON)
endif()

# The compiled input backend (input/): the SDL event translator behind the header-only
# action layer. A plain compiled target (no runtime link, no SYCL); the editor forces
# it on because its window feeds the translator. Added before the editor so the
# sushi_input target exists when the editor links it.
if(SE_BUILD_INPUT)
    add_subdirectory(input)
endif()

# The compiled audio backend (audio/): the SDL device that drives the from-scratch DSP
# mix, behind the header-only DSP core and action layer. A plain STATIC target (no
# runtime link, no SYCL) needing only SDL2. OFF by default; `se audio` (or
# -DSE_BUILD_AUDIO=ON) turns it on. audio_demo, the S0 silent-block vertical slice, is
# declared here (not with the other examples above) because it links sushi_audio, which
# only exists once this subdirectory is added.
if(SE_BUILD_AUDIO)
    add_subdirectory(audio)
    add_sushi_sycl_executable(audio_demo examples/audio_demo.cpp)
    target_link_libraries(audio_demo PRIVATE sushi_audio)

    # audio_dsp_demo, the S1 slice: generate → mix → filter a sine through the
    # header-only DSP core's block graph, self-checked headless then played best-effort
    # through the SDL device.
    add_sushi_sycl_executable(audio_dsp_demo examples/audio_dsp_demo.cpp)
    target_link_libraries(audio_dsp_demo PRIVATE sushi_audio)

    # audio_mixer_demo, the S2 slice: a prioritized multi-source mix through the voice
    # manager (virtual/real cap, distance culling) and the mixer bus DAG (inserts, aux
    # sends, RTPC-ramped gain), self-checked headless then played best-effort.
    add_sushi_sycl_executable(audio_mixer_demo examples/audio_mixer_demo.cpp)
    target_link_libraries(audio_mixer_demo PRIVATE sushi_audio)

    # audio_propagation_demo, the S3 slice: a flyby Doppler through the fractional-delay
    # propagation model (delay/Doppler, air absorption, distance gain), self-checked
    # headless (pitch up approaching, down receding) then played best-effort.
    add_sushi_sycl_executable(audio_propagation_demo examples/audio_propagation_demo.cpp)
    target_link_libraries(audio_propagation_demo PRIVATE sushi_audio)

    # audio_spatial_demo, the S4 slice: a source orbiting the head in binaural 3D through
    # the ambisonic scene bus + analytic HRTF decode, self-checked headless (left/right
    # cues + head-tracking) then played best-effort (headphones recommended).
    add_sushi_sycl_executable(audio_spatial_demo examples/audio_spatial_demo.cpp)
    target_link_libraries(audio_spatial_demo PRIVATE sushi_audio)
endif()

# The Vulkan renderer. OFF by default (see ProjectOptions); a plain compiled target
# (no runtime link, no SYCL) that pulls Vulkan/VMA/vk-bootstrap from vcpkg.
if(SE_BUILD_RENDER)
    add_subdirectory(render)
endif()

# The live world behind the ISimulation seam: a SYCL static library owning the
# runtime, world, and schedule. The editor links it to tick and draw a real world.
if(SE_BUILD_EDITOR)
    add_subdirectory(sim)
endif()

# The Unity-style ImGui editor shell. OFF by default (see ProjectOptions). It links
# sushi_sim (the runtime-backed world), so its final link is SYCL-aware — the plain
# stock-toolchain lane is preserved by the sandbox and render_probe, not the editor.
if(SE_BUILD_EDITOR)
    add_subdirectory(editor)
endif()

# One SYCL test binary against the real runtime (no mocks). OFF by default.
if(SE_BUILD_TESTS)
    enable_testing()
    find_package(GTest CONFIG REQUIRED)
    add_subdirectory(tests)
endif()
