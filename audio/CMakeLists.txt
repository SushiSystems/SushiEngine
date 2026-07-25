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
    sdl/sdl_audio_device.cpp
    # miniaudio backend: the native low-latency IAudioDevice (WASAPI/CoreAudio/ALSA/…),
    # a single vendored public-domain header, no extra linked dependency.
    miniaudio/ma_audio_device.cpp
    miniaudio/miniaudio_impl.cpp)

# PUBLIC the engine headers (for the seam this backend implements against) and this
# directory (so consumers reach <sdl/sdl_audio_device.hpp>). SDL2 is PRIVATE: the
# device header exposes no SDL type — the OS handle is held as an opaque integer — so
# SDL never leaks into a consumer's translation unit.
target_include_directories(sushi_audio
    PUBLIC "${CMAKE_SOURCE_DIR}/include" "${CMAKE_CURRENT_SOURCE_DIR}")
# The vendored miniaudio single header (private: consumers use the opaque MaAudioDevice).
target_include_directories(sushi_audio PRIVATE "${CMAKE_SOURCE_DIR}/third_party/miniaudio")
target_link_libraries(sushi_audio PRIVATE SDL2::SDL2)
# miniaudio drives the OS backends itself (runtime-loaded on Windows); on Unix it needs
# the usual dl/pthread/math libs.
if(NOT WIN32)
    find_package(Threads REQUIRED)
    target_link_libraries(sushi_audio PRIVATE Threads::Threads ${CMAKE_DL_LIBS} m)
endif()
target_compile_features(sushi_audio PUBLIC cxx_std_17)
set_target_properties(sushi_audio PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON)

# The vendored miniaudio implementation is third-party C; compile it and the thin device
# wrapper without the engine's strict warnings-as-errors.
set_source_files_properties(
    miniaudio/miniaudio_impl.cpp miniaudio/ma_audio_device.cpp
    PROPERTIES COMPILE_OPTIONS "-w")
