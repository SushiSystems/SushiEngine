# ProjectOptions — the C++ baseline and the cross-cutting build switches, in one
# place so the whole project's shape is visible at a glance.
if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)  # timestamp extracted archives (FetchContent)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# The editor is a plain C++ SDL2/ImGui shell — no runtime link, no SYCL — so it
# builds on a stock toolchain. OFF so a plain configure needs no SDL2/ImGui;
# `se editor` (or -DSE_BUILD_EDITOR=ON) turns it on.
option(SE_BUILD_EDITOR "Build the SushiEngine editor" OFF)

# OFF so a plain configure stays minimal; the CLI (`se build`) and CI turn it ON.
# GoogleTest comes from vcpkg, the same toolchain the runtime already requires on
# Windows.
option(SE_BUILD_TESTS "Build the SushiEngine test suite" OFF)

# The Vulkan renderer (render/). A plain compiled target — no runtime link, no SYCL —
# so it builds on a stock toolchain, but it needs the Vulkan/VMA/vk-bootstrap vcpkg
# packages. OFF so a plain configure needs none of them; `se build --render` (or
# -DSE_BUILD_RENDER=ON) turns it on.
option(SE_BUILD_RENDER "Build the SushiEngine Vulkan renderer" OFF)
