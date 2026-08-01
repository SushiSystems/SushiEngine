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

# Publish the execution-backend choice (see cmake/ProjectOptions.cmake) to every
# consumer, so which implementation Execution::Context/Graph/Buffer denote is settled
# once by the build instead of per translation unit.
if(SE_EXECUTION_BACKEND STREQUAL "runtime")
    target_compile_definitions(SushiEngine INTERFACE SUSHIENGINE_EXECUTION_BACKEND_RUNTIME=1)
elseif(SE_EXECUTION_BACKEND STREQUAL "native")
    target_compile_definitions(SushiEngine INTERFACE SUSHIENGINE_EXECUTION_BACKEND_NATIVE=1)
else()
    message(FATAL_ERROR
        "SE_EXECUTION_BACKEND must be 'runtime' or 'native', got '${SE_EXECUTION_BACKEND}'")
endif()

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

# soft_body_budget measures docs/slop/physics_system.md §13.1's soft-body line — one
# body, 20 000 tetrahedra, 32 substeps, ≤ 3 ms/tick — and reports the number rather
# than asserting it, because the target is stated against one desktop-class GPU and a
# suite that fails on the hardware it happens to run on is a suite people stop reading.
# The scene's *shape* is pinned by the conformance suite; this reports its *cost*.
add_sushi_sycl_executable(soft_body_budget examples/soft_body_budget.cpp)

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

# device_batch_evaluator_demo exercises the §12.3 device-batched evaluator: a crowd of
# instances sampled/composed/skinned in one compiled SushiRuntime graph run, cross-checked
# element-for-element against the host ClipEvaluator and an analytic answer, plus the
# compile_count == 1 replay-only invariant.
add_sushi_sycl_executable(device_batch_evaluator_demo examples/device_batch_evaluator_demo.cpp)

# motion_matching_demo exercises the §12.4 motion-matching database/search core: a pool of
# sampled poses across several analytic-velocity clips, searched by nearest-neighbor against
# a desired velocity (and, independently, a foot-height proxy term).
add_sushi_sycl_executable(motion_matching_demo examples/motion_matching_demo.cpp)

# ragdoll_blend_demo exercises the §12.4/§11 ragdoll-blend pose modifier: weight-blending a
# joint's local pose toward a physics-supplied object-space transform, and the correct
# parent-to-untouched-child cascade through recompose().
add_sushi_sycl_executable(ragdoll_blend_demo examples/ragdoll_blend_demo.cpp)

# full_body_ik_demo exercises the §12.4 general-purpose multi-effector CCD solver: a
# single-chain convergence proof and two independent limbs sharing an unrotated anchor,
# each reaching its own target without interfering with the other.
add_sushi_sycl_executable(full_body_ik_demo examples/full_body_ik_demo.cpp)

# motion_match_sampler_demo exercises the §12.4 motion-matching blend-graph wiring left open
# by motion_matching.hpp: periodic re-search hysteresis and a two-clip crossfade actually
# changing the output pose over the configured duration.
add_sushi_sycl_executable(motion_match_sampler_demo examples/motion_match_sampler_demo.cpp)

# runtime_retarget_demo exercises the §12.4 same-session retargeting gap: retargeting onto a
# bind-identical clone reproduces direct clip sampling exactly, and retargeting onto a
# differently-proportioned rig reproduces the same joint bend while reaching by the target's
# own bone lengths.
add_sushi_sycl_executable(runtime_retarget_demo examples/runtime_retarget_demo.cpp)

# jiggle_bone_demo exercises the §12.4 jiggle-bone / secondary-motion spring-damper pose
# modifier: settle-to-rest, the distance constraint holding every frame, real lag on a sudden
# parent lurch, and re-settling once the parent holds still again.
add_sushi_sycl_executable(jiggle_bone_demo examples/jiggle_bone_demo.cpp)

# dual_quaternion_skinning_demo exercises the §12.4 dual-quaternion skinning blend math: the
# classic bent-elbow candy-wrapper case (LBS visibly pinches, DQS does not), plus a SushiRuntime
# SYCL kernel cross-check that the same header functions bit-match on device.
add_sushi_sycl_executable(dual_quaternion_skinning_demo examples/dual_quaternion_skinning_demo.cpp)

# facial_blendshapes_demo exercises the §12.4 ARKit-52 canonical blendshape mapping: table
# integrity, name-order-independent resolution against a partial mesh, and missing-shape
# reporting instead of silent no-ops.
add_sushi_sycl_executable(facial_blendshapes_demo examples/facial_blendshapes_demo.cpp)

# sequence_timeline_demo exercises the §12.4 minimal cinematics/sequencer timeline core:
# pure-function float-track evaluation (scrub-safe) and forward-only event dispatch that
# fires every event crossed in a step, in time order.
add_sushi_sycl_executable(sequence_timeline_demo examples/sequence_timeline_demo.cpp)

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
# Header-only JSON for animator_controller_json.hpp's save/load round trip.
find_package(nlohmann_json CONFIG REQUIRED)
target_link_libraries(authoring_demo PRIVATE nlohmann_json::nlohmann_json)

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
    set(SE_BUILD_AUDIO ON) # the editor's live audio system links sushi_audio (S9)
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

    # audio_reverb_demo, the S5 slice: the Jot FDN reverb on a per-zone aux bus (I3DL2
    # params, Sabine/Eyring room RT60), self-checked headless (bounded/decaying tail,
    # predelay, longer-decay-rings-longer, shoebox factory) then played best-effort.
    add_sushi_sycl_executable(audio_reverb_demo examples/audio_reverb_demo.cpp)
    target_link_libraries(audio_reverb_demo PRIVATE sushi_audio)

    # audio_scene_demo, the S6 slice: the ECS→audio control-plane bridge (AudioScene)
    # reconciling per-frame world snapshots into live voices — an emitter flyby (Doppler +
    # L/R), a second emitter appearing, and one being dropped — self-checked headless then
    # played best-effort. The real ECS read is covered by Integration_AudioEcs.
    add_sushi_sycl_executable(audio_scene_demo examples/audio_scene_demo.cpp)
    target_link_libraries(audio_scene_demo PRIVATE sushi_audio)

    # audio_occlusion_demo, the S7 slice: occlusion/obstruction over the acoustic BVH
    # (BLAS ray + TLAS refit), three-band material transmission, soft multi-ray occlusion,
    # the room/portal doorway model, and image-source early reflections — self-checked
    # headless (including the integrated voice-manager path) then played best-effort (a
    # tone sliding behind a concrete wall).
    add_sushi_sycl_executable(audio_occlusion_demo examples/audio_occlusion_demo.cpp)
    target_link_libraries(audio_occlusion_demo PRIVATE sushi_audio)

    # audio_bank_demo, the S8 slice: the asset pipeline — a bank of IMA-ADPCM SFX + a PCM
    # music track baked to a binary and loaded back, a Random footstep event resolved
    # through the bank-backed IEmitterSourceFactory, and a streamed music bed fed by a
    # background worker into the lock-free ring. Self-checked headless then played best-effort.
    add_sushi_sycl_executable(audio_bank_demo examples/audio_bank_demo.cpp)
    target_link_libraries(audio_bank_demo PRIVATE sushi_audio)

    # audio_profiler_demo, the S9 slice: the audio→GUI live-profiler telemetry channel —
    # the seqlock snapshot (voice counts, per-bus + master meters, scope) published by the
    # audio thread and read lock-free by the "GUI". Self-checked headless then polled live
    # off a running device.
    add_sushi_sycl_executable(audio_profiler_demo examples/audio_profiler_demo.cpp)
    target_link_libraries(audio_profiler_demo PRIVATE sushi_audio)

    # audio_procedural_demo, the S10 slice: procedural SFX — modal-impact synthesis (a
    # struck-object resonator bank) and wind (speed-driven filtered noise + Aeolian tone) as
    # plain VoiceSources. Self-checked headless then played best-effort.
    add_sushi_sycl_executable(audio_procedural_demo examples/audio_procedural_demo.cpp)
    target_link_libraries(audio_procedural_demo PRIVATE sushi_audio)

    # audio_convolution_demo, the S10 slice: the convolution reverb (radix-2 FFT +
    # partitioned overlap-save) behind the IReverb seam on a per-zone aux bus, driven by a
    # synthesised room impulse response. Self-checked headless then played best-effort.
    add_sushi_sycl_executable(audio_convolution_demo examples/audio_convolution_demo.cpp)
    target_link_libraries(audio_convolution_demo PRIVATE sushi_audio)

    # audio_authoring_demo: the authoring project (DAW backend) — author a container graph,
    # flatten it to the runtime event database, and bake a bank. Pure engine headers.
    add_sushi_sycl_executable(audio_authoring_demo examples/audio_authoring_demo.cpp)
    target_link_libraries(audio_authoring_demo PRIVATE sushi_audio)

    # audio_raytrace_demo: Monte-Carlo ray-traced room acoustics (measured RT60 + baked IR)
    # and Maekawa edge diffraction. Pure engine headers, no third-party dep.
    add_sushi_sycl_executable(audio_raytrace_demo examples/audio_raytrace_demo.cpp)
    target_link_libraries(audio_raytrace_demo PRIVATE sushi_audio)

    # audio_accelerator_demo, the S10 slice: the SushiRuntime SYCL DSP accelerator — batch
    # FIR convolution offloaded to the device with k-block lookahead, checked against a CPU
    # reference. A SYCL translation unit (it includes accelerator_sycl.hpp + the runtime);
    # it needs no sushi_audio, only the runtime the SYCL executable macro already links.
    add_sushi_sycl_executable(audio_accelerator_demo examples/audio_accelerator_demo.cpp)

    # audio_miniaudio_demo: the same engine through the miniaudio (native low-latency)
    # IAudioDevice backend instead of SDL — self-checked headless then played best-effort.
    add_sushi_sycl_executable(audio_miniaudio_demo examples/audio_miniaudio_demo.cpp)
    target_link_libraries(audio_miniaudio_demo PRIVATE sushi_audio)

    # audio_opus_demo: the Opus codec (compressed music/dialogue) behind the IAudioCodec
    # seam — a full encode→decode round-trip self-check. Guarded by the presence of the
    # vcpkg Opus package; it links core libopus (Opus::opus) and defines SE_AUDIO_OPUS.
    find_package(Opus CONFIG QUIET)
    find_package(OpusFile CONFIG QUIET)
    if(Opus_FOUND AND OpusFile_FOUND)
        add_sushi_sycl_executable(audio_opus_demo examples/audio_opus_demo.cpp)
        target_link_libraries(audio_opus_demo PRIVATE sushi_audio Opus::opus OpusFile::opusfile)
        target_compile_definitions(audio_opus_demo PRIVATE SE_AUDIO_OPUS=1)
    else()
        message(STATUS "Opus package not found — audio_opus_demo disabled (declare opusfile via ss)")
    endif()

    # audio_vorbis_demo: the Vorbis codec behind the IAudioCodec seam — encode→decode round-trip.
    # Guarded by the vcpkg Vorbis package; links libvorbis (Vorbis::vorbis + Vorbis::vorbisenc).
    find_package(Vorbis CONFIG QUIET)
    if(Vorbis_FOUND)
        add_sushi_sycl_executable(audio_vorbis_demo examples/audio_vorbis_demo.cpp)
        target_link_libraries(audio_vorbis_demo PRIVATE sushi_audio Vorbis::vorbis Vorbis::vorbisenc)
        target_compile_definitions(audio_vorbis_demo PRIVATE SE_AUDIO_VORBIS=1)
    else()
        message(STATUS "Vorbis package not found — audio_vorbis_demo disabled (declare libvorbis via ss)")
    endif()

    # audio_stream_compressed_demo: streaming Opus + Vorbis from a disk-like IDataSource through
    # the StreamingDecoder pipeline. Needs both compressed codecs.
    if(Opus_FOUND AND OpusFile_FOUND AND Vorbis_FOUND)
        add_sushi_sycl_executable(audio_stream_compressed_demo examples/audio_stream_compressed_demo.cpp)
        target_link_libraries(audio_stream_compressed_demo
            PRIVATE sushi_audio Opus::opus OpusFile::opusfile Vorbis::vorbis Vorbis::vorbisenc)
        target_compile_definitions(audio_stream_compressed_demo PRIVATE SE_AUDIO_OPUS=1 SE_AUDIO_VORBIS=1)
    endif()

    # audio_sofa_demo: the SOFA/HDF5 measured-HRTF path — loads a real HRIR set and drives the
    # binaural spatializer through it. Guarded by the presence of the vcpkg HDF5 package; it
    # links HDF5 (C API) and defines SE_AUDIO_SOFA.
    find_package(hdf5 CONFIG QUIET)
    if(hdf5_FOUND OR TARGET hdf5::hdf5-shared OR TARGET hdf5::hdf5-static)
        add_sushi_sycl_executable(audio_sofa_demo examples/audio_sofa_demo.cpp)
        if(TARGET hdf5::hdf5-shared)
            target_link_libraries(audio_sofa_demo PRIVATE sushi_audio hdf5::hdf5-shared)
        else()
            target_link_libraries(audio_sofa_demo PRIVATE sushi_audio hdf5::hdf5-static)
        endif()
        target_compile_definitions(audio_sofa_demo PRIVATE SE_AUDIO_SOFA=1)
        # audio_magls_demo: MagLS ambisonic→binaural decode + anthropometric HRTF, driven from a
        # real SOFA set. Same HDF5 guard as the SOFA demo.
        add_sushi_sycl_executable(audio_magls_demo examples/audio_magls_demo.cpp)
        if(TARGET hdf5::hdf5-shared)
            target_link_libraries(audio_magls_demo PRIVATE sushi_audio hdf5::hdf5-shared)
        else()
            target_link_libraries(audio_magls_demo PRIVATE sushi_audio hdf5::hdf5-static)
        endif()
        target_compile_definitions(audio_magls_demo PRIVATE SE_AUDIO_SOFA=1)
    else()
        message(STATUS "HDF5 package not found — audio_sofa_demo disabled (declare hdf5 via ss)")
    endif()
endif()

# The engine-neutral geometry module (geometry/): a triangle mesh and the shared
# signed-distance baker, linking nothing at all. Added unconditionally and before
# the renderer, because both the renderer and the physics read it and neither is
# allowed to own it — the physics sits below the renderer, so pointing it at a
# Vulkan-owning class would invert the layering and make an offline cooker require
# a device (docs/slop/physics_system.md §3.4).
add_subdirectory(geometry)

# The offline cooking pipeline (cooking/, headers under physics/cooking): mesh in,
# simulation asset out. Immediately after geometry because it is that module's first
# real consumer, and host-only by design — it links sushi_geometry and nothing else,
# so a build machine with no GPU can still turn a mesh into a collision asset
# (docs/slop/physics_system.md §3.1, §8).
add_subdirectory(cooking)

# The engine-neutral atmosphere module (atmosphere/): T1, the global dynamical
# core, and the Fourier transform its elliptic inversion is built on. Linking
# nothing, for the same reason geometry does: its consumers are the simulation's
# gameplay queries, the regional nest's parent forcing, and a headless probe, and
# none of the three should have to bring up a device to ask what the weather is
# (docs/slop/atmosphere_system.md §5). The nest itself is a Vulkan service and
# stays in render/atmosphere.
add_subdirectory(atmosphere)

# The engine-neutral asset importers (import/): glTF in, engine blob out, linking
# nothing — and the one translation unit that defines cgltf's implementation. Before
# the renderer for the same reason geometry is: the renderer reads it, and a target
# that only wants to load a skeleton must not have to bring up a graphics stack to
# find `import_gltf_skeleton` (docs/slop/physics_system.md §3.4).
add_subdirectory(import)

# The Vulkan renderer. OFF by default (see ProjectOptions); a plain compiled target
# (no runtime link, no SYCL) that pulls Vulkan/VMA/vk-bootstrap from vcpkg.
if(SE_BUILD_RENDER)
    add_subdirectory(render)
endif()

# The live world behind the ISimulation seam: a SYCL static library owning the
# runtime, world, and schedule. The editor links it to tick and draw a real world,
# and so does the test suite — the scene-serializer round-trip drives the real
# simulation through IWorldEditor, which is the whole point of that test. So the
# condition is either consumer, not just the editor: with tests on and the editor
# off the target went missing and the test binary failed to link against a library
# nothing had been asked to build. It needs no renderer (it links SushiEngine and
# sushi_atmosphere only), so building it in the tests lane costs one static library.
if(SE_BUILD_EDITOR OR SE_BUILD_TESTS)
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
