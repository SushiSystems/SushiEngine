# audio — the from-scratch DSP graph, the voice and mixer model over it, and the
# spatialisation, propagation, reverb and asset seams around that. Header-only: everything
# above the output device is host C++ a consumer compiles into its own translation unit, so a
# headless target can mix and assert on a block without opening a stream.
sushiengine_add_module(NAME audio LAYER domain PUBLIC_DEPENDS core)

# The one part of audio that touches an operating system: SDL2's output stream and miniaudio's
# native low-latency backends, each an IAudioDevice that drives the mix once per block on the
# device's own callback thread. Its own target rather than part of the module, because the
# module has to exist for every configure while this needs SDL2 — a plain configure asks for
# no audio packages at all, and `se audio` or the editor turns the option on.
if(SUSHIENGINE_BUILD_AUDIO)
    find_package(SDL2 CONFIG REQUIRED)

    add_library(sushiengine_audio_backend STATIC
        source/backend/sdl/sdl_audio_device.cpp
        source/backend/miniaudio/ma_audio_device.cpp
        source/backend/miniaudio/miniaudio_impl.cpp)

    # Consumers name the two devices <sdl/sdl_audio_device.hpp> and
    # <miniaudio/ma_audio_device.hpp>, so the directory holding both is public. SDL2 is
    # private: the device header keeps the OS handle as an opaque integer, so SDL never leaks
    # into a consumer's translation unit.
    target_include_directories(sushiengine_audio_backend
        PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/source/backend"
        PRIVATE "${CMAKE_SOURCE_DIR}/third_party/miniaudio")
    target_link_libraries(sushiengine_audio_backend PUBLIC sushiengine_audio PRIVATE SDL2::SDL2)
    # miniaudio drives the OS backends itself (runtime-loaded on Windows); on Unix it needs
    # the usual dl/pthread/math libs.
    if(NOT WIN32)
        find_package(Threads REQUIRED)
        target_link_libraries(sushiengine_audio_backend
            PRIVATE Threads::Threads ${CMAKE_DL_LIBS} m)
    endif()
    set_target_properties(sushiengine_audio_backend PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON)

    # The vendored miniaudio implementation is third-party C; compile it and the thin device
    # wrapper over it without the engine's warnings-as-errors.
    set_source_files_properties(
        source/backend/miniaudio/miniaudio_impl.cpp
        source/backend/miniaudio/ma_audio_device.cpp
        PROPERTIES COMPILE_OPTIONS "-w")
endif()
