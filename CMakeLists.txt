cmake_minimum_required(VERSION 3.20)

# SushiEngine is the head; SushiRuntime is a plugged-in component (the battery).
# The runtime configures the SYCL toolchain inside its own CMake, but it does so
# just before its project() call. As the top-level project we reach project()
# first, so we must pick the SYCL compiler and the vcpkg toolchain here, before
# our project(), or the runtime's selection runs too late to take effect.

set(SUSHIRUNTIME_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../sushiruntime"
    CACHE PATH "Path to the SushiRuntime sibling checkout")

# Mirror the runtime's intel-llvm default: use clang++ when the caller named no
# compiler. Pass -DCMAKE_CXX_COMPILER=... to override (e.g. acpp, icx-cl).
if(NOT DEFINED CMAKE_CXX_COMPILER OR CMAKE_CXX_COMPILER STREQUAL "")
    find_program(SE_CLANGXX NAMES clang++)
    if(SE_CLANGXX)
        set(CMAKE_CXX_COMPILER "${SE_CLANGXX}" CACHE FILEPATH "CXX compiler")
    endif()
endif()

# vcpkg toolchain file (the runtime needs hwloc on Windows). Resolve it from
# VCPKG_ROOT and set it before project(), exactly as the runtime does.
if(WIN32)
    if((NOT DEFINED VCPKG_ROOT OR VCPKG_ROOT STREQUAL "") AND DEFINED ENV{VCPKG_ROOT})
        set(VCPKG_ROOT "$ENV{VCPKG_ROOT}" CACHE PATH "Path to vcpkg root")
    endif()
    if(VCPKG_ROOT AND (NOT DEFINED CMAKE_TOOLCHAIN_FILE OR CMAKE_TOOLCHAIN_FILE STREQUAL ""))
        set(CMAKE_TOOLCHAIN_FILE "${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
            CACHE FILEPATH "vcpkg toolchain file")
    endif()
endif()

project(SushiEngine VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Pull the runtime in as an in-tree component. This provides the `sushiruntime`
# target and defines the add_sycl_to_target() command we apply to our SYCL TUs.
add_subdirectory("${SUSHIRUNTIME_DIR}" "${CMAKE_BINARY_DIR}/sushiruntime")

# The engine is header-only so its SYCL kernels instantiate inside the consuming
# translation unit (the sandbox), which is the safe way to ship device code.
add_library(SushiEngine INTERFACE)
target_include_directories(SushiEngine INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(SushiEngine INTERFACE sushiruntime)

# Each example is one SYCL translation unit linking the engine. The runtime applies
# -fsycl directory-scoped inside its own subdirectory, which does not reach a target
# defined here, and its add_sycl_to_target() is a no-op stub on the intel-llvm path;
# so on intel-llvm we add the flag ourselves. This helper keeps every example target
# consistent: SYCL flags and, on Windows, the runtime DLL copy.
function(add_engine_executable name source)
    add_executable(${name} ${source})
    target_link_libraries(${name} PRIVATE SushiEngine)
    set_target_properties(${name} PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON)

    add_sycl_to_target(TARGET ${name} SOURCES ${source})
    if(SR_SYCL_TOOLCHAIN STREQUAL "intel-llvm")
        target_compile_options(${name} PRIVATE -fsycl)
        target_link_options(${name} PRIVATE -fsycl)
    endif()

    if(WIN32)
        add_custom_command(TARGET ${name} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_FILE:sushiruntime>
                $<TARGET_FILE_DIR:${name}>)
    endif()
endfunction()

# The sandbox is the ECS worked example; pgs_demo is the physics constraint solver.
add_engine_executable(sandbox sandbox/main.cpp)
add_engine_executable(pgs_demo examples/pgs_demo.cpp)

# --- Testing ---
# OFF by default so a plain configure stays minimal; the CLI (`se project build`)
# and CI turn it ON. GoogleTest comes from vcpkg, the same toolchain the runtime
# already requires on Windows.
option(SE_BUILD_TESTS "Build the SushiEngine test suite" OFF)

if(SE_BUILD_TESTS)
    enable_testing()
    find_package(GTest CONFIG REQUIRED)
    add_subdirectory(tests)
endif()
