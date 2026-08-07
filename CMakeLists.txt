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

# Read before project(): which SushiEngine::Execution backend this configure is
# for (RUNTIME-PORT1). A -D override on the command line is already visible here
# — CMake populates command-line values before any script runs — and a fresh
# configure with no override falls back to the same "runtime" default
# ProjectOptions.cmake formalizes as the cache entry below (a re-configure with
# an existing cache entry picks that entry up instead, since DEFINED is already
# true by then and this block is skipped). This is not optional plumbing: when
# native is selected, the SushiRuntime subproject and its SYCL toolchain probe
# are skipped entirely (below), which is the actual mechanism behind "no SYCL
# toolchain needed" for a native build — gating sushiengine_apply_sycl() later, after
# project() already required a SYCL-capable compiler, would be too late.
if(NOT DEFINED SUSHIENGINE_EXECUTION_BACKEND)
    set(SUSHIENGINE_EXECUTION_BACKEND "runtime")
endif()

# Pre-project: toolchain shaping.
include(Vcpkg)  # (Windows) locate vcpkg + set the toolchain file — needed by both
                # backends (SDL2/GoogleTest/nlohmann_json all come from vcpkg too).
if(SUSHIENGINE_EXECUTION_BACKEND STREQUAL "runtime")
    include(Toolchain)  # pick the default SYCL compiler (clang++) before the probe
endif()

project(SushiEngine VERSION 0.1.0 LANGUAGES CXX)

# Post-project: options, runtime, SYCL policy.
include(ProjectOptions)  # C++17 baseline, cross-cutting options
if(SUSHIENGINE_EXECUTION_BACKEND STREQUAL "runtime")
    include(Runtime)         # add_subdirectory(sushiruntime) -> sushiruntime target,
                             # add_sycl_to_target(), SR_SYCL_TOOLCHAIN
    include(SyclTarget)      # sushiengine_apply_sycl() / sushiengine_add_sycl_executable() helpers
else()
    include(NativeExecutionTarget)  # sushiengine_add_native_executable() helper
endif()

# What a shell implies, settled here because engine/domain/{audio,input} read these options to
# decide whether to declare their compiled backend, and the module tree is added next. The
# editor presents through the Vulkan renderer, feeds its window into the input translator, and
# runs a live audio system (S9), so turning it on turns all three on. The player presents and
# takes input the same way; audio is not forced on because PlayerApp's S5 skeleton has no live
# audio system yet. Both shells exist on the runtime lane only, so the implication does too.
if(SUSHIENGINE_EXECUTION_BACKEND STREQUAL "runtime")
    if(SUSHIENGINE_BUILD_EDITOR)
        set(SUSHIENGINE_BUILD_RENDER ON)
        set(SUSHIENGINE_BUILD_INPUT ON)
        set(SUSHIENGINE_BUILD_AUDIO ON)
    endif()
    if(SUSHIENGINE_BUILD_PLAYER)
        set(SUSHIENGINE_BUILD_RENDER ON)
        set(SUSHIENGINE_BUILD_INPUT ON)
    endif()
endif()

# The engine's modules. Before the SushiEngine target below, which links what is
# declared here — sushiengine_execution_native among them on the native lane.
# sushiengine_add_module() grants each module its own include/ and checks its dependencies
# against the tier order in cmake/EngineLayers.cmake.
include(Module)
add_subdirectory(engine)

# The engine + its consumers.
# The engine is header-only so its SYCL kernels instantiate inside the consuming
# translation unit (each sample and test binary), which is the safe way to ship device code.
add_library(SushiEngine INTERFACE)
target_include_directories(SushiEngine INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/engine/include")

# The modules that have their own include root ride along here, so a consumer that
# links only SushiEngine still resolves every engine header.
target_link_libraries(SushiEngine INTERFACE
    sushiengine_core
    sushiengine_ecs
    sushiengine_execution
    sushiengine_animation
    sushiengine_astro
    sushiengine_atmosphere
    sushiengine_audio
    sushiengine_environment
    sushiengine_geometry
    sushiengine_imaging
    sushiengine_input
    sushiengine_material
    sushiengine_physics
    sushiengine_terrain
    sushiengine_ui
    sushiengine_vfx
    sushiengine_gltf
    sushiengine_loop)

# The execution backend and the macro naming it come from the execution module, which
# owns that seam (see engine/foundation/execution/CMakeLists.txt).

# Determinism guard rail (see cmake/ProjectOptions.cmake): forbid the compiler from
# reordering floating-point operations or fusing multiply-adds, either of which can
# change a result's bit pattern between builds and would silently break replay.
# clang/gcc only — MSVC's equivalent (/fp:strict) is not wired up here because the
# engine's primary compiler is the intel/llvm clang++ SYCL toolchain.
target_compile_options(SushiEngine INTERFACE
    $<$<AND:$<BOOL:${SUSHIENGINE_DETERMINISTIC_FP}>,$<CXX_COMPILER_ID:Clang,AppleClang,GNU>>:-fno-fast-math;-ffp-contract=off>)

# Runtime-backend lane: the shells, the samples and the test suite.
# All of it is SYCL/SushiRuntime-aware — the native lane below is deliberately
# narrow (RUNTIME-PORT1) rather than an attempt to make all of this build without
# a SYCL toolchain too; that is future, milestone-by-milestone work, not this one.
if(SUSHIENGINE_EXECUTION_BACKEND STREQUAL "runtime")

# The host shader compiler and the headless probes. After the module tree because three of the
# probes link the renderer, and on this lane only, because that is where they have always been
# built and the native lane carries no target it has not proved out.
add_subdirectory(tools)

# The world tier's compiled modules: the live world behind the ISimulation seam, the
# `.sushiscene`/`.sushieffect`/environment file shapes, and the authoring services that
# edit them. Added from here rather than with the rest of engine/world because the targets
# below this condition are their only consumers — the test suite among them, since the scene
# round-trip drives a real simulation through IWorldEditor rather than a mock, and a
# tests-on/editor-off configure would otherwise have nothing to link. None of the three links
# the renderer, so the tests lane pays three static libraries for them and no more.
if(SUSHIENGINE_BUILD_EDITOR OR SUSHIENGINE_BUILD_TESTS OR SUSHIENGINE_BUILD_PLAYER)
    add_subdirectory(engine/world/simulation)
    add_subdirectory(engine/world/serialization)
    add_subdirectory(engine/world/authoring)
    # After serialization, which it links privately to write a prefab.
    add_subdirectory(engine/world/model_import)
endif()

# The worked examples, on both execution lanes — samples/CMakeLists.txt picks the lane's own
# executable helper and declares only sandbox and pgs_demo on the native one. After the world
# tier because the physics sample scene links sushiengine_simulation.
add_subdirectory(samples)

# sushiengine_platform: the window/lifecycle seam (IPlatformWindow) and its SDL2
# implementation. Both the editor and the player need it and neither owns it, so it
# is added once, ahead of both, guarded so turning both on in one configure does not
# add_subdirectory() it twice. It is added from here rather than with the rest of
# engine/foundation because its guard reads options settled further up this file.
if((SUSHIENGINE_BUILD_EDITOR OR SUSHIENGINE_BUILD_PLAYER) AND NOT TARGET sushiengine_platform)
    add_subdirectory(engine/foundation/platform)
endif()

# The two application shells, each behind its own option. After sushiengine_platform, which both
# link and neither owns.
add_subdirectory(applications)

# One SYCL test binary against the real runtime (no mocks). OFF by default.
if(SUSHIENGINE_BUILD_TESTS)
    enable_testing()
    find_package(GTest CONFIG REQUIRED)
    add_subdirectory(tests)
endif()

else() # SUSHIENGINE_EXECUTION_BACKEND STREQUAL "native"

# Native lane (RUNTIME-PORT1): only the executables this milestone proves out —
# no SYCL toolchain, no SushiRuntime subproject. samples/ declares sandbox and
# pgs_demo here and nothing else; widen that list milestone by milestone as the
# native backend gains real coverage (RUNTIME-PORT1.x/PORT2), not speculatively —
# each addition is its own reviewable diff against a real build/run oracle, not a
# guess about what "should" work. Everything else in the runtime-backend lane above
# — every other sample, engine/world/, applications/, tests/ — stays out of the
# native lane's build graph entirely rather than being added as broken targets
# nobody asked `cmake --build .` to actually build.
add_subdirectory(samples)

# sushiengine_native_execution_tests: the Execution-seam conformance cases (hazard
# vocabulary + DynamicGraph), run against NativeBackend instead of
# SushiRuntime. Its own subdirectory, not tests/ itself — that target
# (sushiengine_functional_tests) links sushiengine_simulation/the scene
# serializers/sushiengine_gltf, none of which this lane builds.
if(SUSHIENGINE_BUILD_TESTS)
    enable_testing()
    find_package(GTest CONFIG REQUIRED)
    add_subdirectory(tests/native_execution)
endif()

endif()
