# ============================================================================
# sushi_input — the compiled input backend
#
# A plain STATIC library mirroring the sushi_render discipline: it links SDL2 and
# nothing else — no SYCL, no sushiruntime — so it builds on a stock toolchain and
# never touches the one-way SushiEngine -> SushiRuntime layering. It carries the ONE
# SDL-aware input component, the SdlInputTranslator, which turns already-pumped
# SDL_Event records into engine InputEvents. The entire action layer above it is the
# header-only include/SushiEngine/input/ tree, picked up through the SushiEngine
# INTERFACE target with no build step of its own.
#
# SDL2 comes from the platform package manager (vcpkg on Windows via sdl2[vulkan],
# already declared in cli/sushistack.deps.toml; the system elsewhere) through
# find_package, exactly as the editor resolves it.
# ============================================================================

find_package(SDL2 CONFIG REQUIRED)

add_library(sushi_input STATIC
    sdl/sdl_input_translator.cpp)

# PUBLIC the engine headers (for the action layer this backend implements against) and
# this directory (so consumers reach <sdl/sdl_input_translator.hpp>). SDL2 is PRIVATE:
# the translator header crosses native events as const void*, so SDL never leaks into a
# consumer's translation unit.
target_include_directories(sushi_input
    PUBLIC "${CMAKE_SOURCE_DIR}/include" "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(sushi_input PRIVATE SDL2::SDL2)
target_compile_features(sushi_input PUBLIC cxx_std_17)
set_target_properties(sushi_input PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON)
