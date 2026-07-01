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

# The sandbox is the ECS worked example; pgs_demo is the physics constraint solver.
add_sushi_sycl_executable(sandbox sandbox/main.cpp)
add_sushi_sycl_executable(pgs_demo examples/pgs_demo.cpp)

# --- Optional subdirectories --------------------------------------------------
# The Unity-style ImGui editor shell. OFF by default (see ProjectOptions); it is a
# plain C++ target — no runtime link, no SYCL — so it builds on a stock toolchain.
if(SE_BUILD_EDITOR)
    add_subdirectory(editor)
endif()

# The Vulkan renderer. OFF by default (see ProjectOptions); a plain compiled target
# (no runtime link, no SYCL) that pulls Vulkan/VMA/vk-bootstrap from vcpkg.
if(SE_BUILD_RENDER)
    add_subdirectory(render)
endif()

# One SYCL test binary against the real runtime (no mocks). OFF by default.
if(SE_BUILD_TESTS)
    enable_testing()
    find_package(GTest CONFIG REQUIRED)
    add_subdirectory(tests)
endif()
