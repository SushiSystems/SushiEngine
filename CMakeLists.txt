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

# The precision choice reaches every consumer through this one INTERFACE definition:
# sandbox, pgs_demo, sushi_sim (and thus the editor), and the tests all link
# SushiEngine, so all agree on Scalar. core/blas_placeholder.hpp is the sole reader.
target_compile_definitions(SushiEngine INTERFACE
    $<$<BOOL:${SE_SCALAR_DOUBLE}>:SE_SCALAR_DOUBLE>)

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

# --- Optional subdirectories --------------------------------------------------
# The editor presents through the Vulkan renderer, so turning the editor on turns
# the renderer on too. Render must be added before the editor so the sushi_render
# target exists when the editor links it.
if(SE_BUILD_EDITOR)
    set(SE_BUILD_RENDER ON)
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
