# ProjectOptions — the C++ baseline and the cross-cutting build switches, in one
# place so the whole project's shape is visible at a glance.
if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)  # timestamp extracted archives (FetchContent)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# Every lane writes its own database into its own tree; `.clangd` at the root names
# build/default as the one an editor reads, since the three cannot be merged.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# The editor is a plain C++ SDL2/ImGui shell — no runtime link, no SYCL — so it
# builds on a stock toolchain. OFF so a plain configure needs no SDL2/ImGui;
# `se editor` (or -DSUSHIENGINE_BUILD_EDITOR=ON) turns it on.
option(SUSHIENGINE_BUILD_EDITOR "Build the SushiEngine editor" OFF)

# The ImGui-free runtime shell (applications/player/, PLATFORM0 S5): sushiengine_platform's window,
# sushiengine_render's renderer, and PlayerApp's own start/frame/suspend/resume/shutdown
# loop, with no sushiengine_imgui link at all — the point of the whole player/editor split.
# OFF so a plain configure needs nothing beyond what SUSHIENGINE_BUILD_RENDER already needs;
# `se player` (or -DSUSHIENGINE_BUILD_PLAYER=ON) turns it on.
option(SUSHIENGINE_BUILD_PLAYER "Build the SushiEngine player" OFF)

# OFF so a plain configure stays minimal; the CLI (`se build`) and CI turn it ON.
# GoogleTest comes from vcpkg, the same toolchain the runtime already requires on
# Windows.
option(SUSHIENGINE_BUILD_TESTS "Build the SushiEngine test suite" OFF)

# The worked examples under samples/. OFF because a plain configure has no reason to declare
# 52 more executables, each of which is its own SYCL translation unit and so its own device
# compile; `se run <demo>` needs -DSUSHIENGINE_BUILD_EXAMPLES=ON. The sandbox and pgs_demo sit outside
# this option deliberately — they are the only two targets that prove the native execution
# lane builds at all, so they are declared whatever it says.
option(SUSHIENGINE_BUILD_EXAMPLES "Build the SushiEngine examples" OFF)

# The Vulkan renderer (engine/presentation/render/). A plain compiled target — no runtime link, no SYCL —
# so it builds on a stock toolchain, but it needs the Vulkan/VMA/vk-bootstrap vcpkg
# packages. OFF so a plain configure needs none of them; `se build --render` (or
# -DSUSHIENGINE_BUILD_RENDER=ON) turns it on.
option(SUSHIENGINE_BUILD_RENDER "Build the SushiEngine Vulkan renderer" OFF)

# The compiled input backend. A plain STATIC library — no runtime link, no
# SYCL — that carries the one SDL-aware input component (the event translator) and
# needs only the SDL2 vcpkg package the editor already requires. OFF so a plain
# configure needs nothing; the editor forces it ON (its window feeds the translator),
# and `-DSUSHIENGINE_BUILD_INPUT=ON` turns it on for a standalone windowed game. The header-only
# action layer above it is the input module itself, which needs no build option — it
# rides the SushiEngine INTERFACE target and is exercised headlessly by the test suite.
option(SUSHIENGINE_BUILD_INPUT "Build the SushiEngine compiled input backend (SDL translator)" OFF)

# The compiled audio backend. A plain STATIC library — no runtime link, no
# SYCL — that carries the OS-aware audio components (the SDL and miniaudio devices) and
# needs only the SDL2 vcpkg package the editor and input backend already require. OFF so a
# plain configure needs nothing; `se audio` (or -DSUSHIENGINE_BUILD_AUDIO=ON) turns it on. The
# from-scratch DSP core and action layer above it are the audio module itself, which needs
# no build option — it rides the SushiEngine INTERFACE target.
option(SUSHIENGINE_BUILD_AUDIO "Build the SushiEngine compiled audio backend (SDL device)" OFF)

# Which implementation SushiEngine::Execution's Context/Graph/Buffer denote. The seam
# is a compile-time policy rather than a virtual interface because a device backend has
# to forward each kernel into its own launch as the original callable, and a type-erased
# one cannot be captured into device code; one binary needs one backend regardless,
# since a device translation unit already requires that compiler for the whole unit.
#
#   runtime — SushiRuntime's task graph and shared USM (the only backend today).
#   native  — the thread-pool backend for platforms SushiRuntime cannot reach (RUNTIME-PORT1).
set(SUSHIENGINE_EXECUTION_BACKEND "runtime" CACHE STRING
    "Execution backend for SushiEngine::Execution (runtime|native)")
set_property(CACHE SUSHIENGINE_EXECUTION_BACKEND PROPERTY STRINGS runtime native)

# Determinism guard rail (SushiLoop M0/M1, docs/design/SUSHILOOP.md): reassociation and
# fused contraction let the compiler evaluate the same floating-point expression
# differently between builds or optimisation levels, which breaks the "same input,
# same result" contract rollback and replay depend on. ON by default so a plain
# configure is deterministic out of the box; a specialised, non-sim target may turn
# it off if it has no determinism obligation.
option(SUSHIENGINE_DETERMINISTIC_FP "Disable fast-math/FP-reassociation on the engine target" ON)
