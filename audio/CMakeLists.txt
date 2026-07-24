# ============================================================================
# sushi_audio — the compiled audio device backend
#
# A plain STATIC library mirroring the sushi_input discipline: it links SDL2 and
# nothing else — no SYCL, no sushiruntime — so it builds on a stock toolchain and
# never touches the one-way SushiEngine -> SushiRuntime layering. It carries the ONE
# SDL-aware audio component, the SdlAudioDevice, which opens an OS output stream and
# drives the from-scratch DSP mix (an IAudioRenderer) once per block on SDL's audio
# callback thread. The entire DSP core and action layer above it is the header-only
# include/SushiEngine/audio/ tree, picked up through the SushiEngine INTERFACE target
# with no build step of its own.
#
# SDL2 comes from the platform package manager (vcpkg on Windows via sdl2[vulkan],
# already declared in cli/sushistack.deps.toml; the system elsewhere) through
# find_package, exactly as the input backend and editor resolve it.
# ============================================================================

find_package(SDL2 CONFIG REQUIRED)

add_library(sushi_audio STATIC
    sdl/sdl_audio_device.cpp)

# PUBLIC the engine headers (for the seam this backend implements against) and this
# directory (so consumers reach <sdl/sdl_audio_device.hpp>). SDL2 is PRIVATE: the
# device header exposes no SDL type — the OS handle is held as an opaque integer — so
# SDL never leaks into a consumer's translation unit.
target_include_directories(sushi_audio
    PUBLIC "${CMAKE_SOURCE_DIR}/include" "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(sushi_audio PRIVATE SDL2::SDL2)
target_compile_features(sushi_audio PUBLIC cxx_std_17)
set_target_properties(sushi_audio PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON)
