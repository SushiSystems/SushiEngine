# input — the device-neutral action layer: raw events, the binding tables over them, per-player
# action maps, gestures, haptics and the replay tick stream. Header-only. Its pointer seam is
# spelled in ui's types, which is a same-tier edge and legal.
sushiengine_add_module(NAME input LAYER domain PUBLIC_DEPENDS ui)

# The one part of input that touches an operating system: the translator that turns
# already-pumped SDL_Event records into engine InputEvents. Its own target rather than part of
# the module, because the module has to exist for every configure while this needs SDL2 — the
# editor and the player force the option on because their windows feed the translator.
if(SE_BUILD_INPUT)
    find_package(SDL2 CONFIG REQUIRED)

    add_library(sushi_input STATIC source/backend/sdl/sdl_input_translator.cpp)

    # Consumers name the translator <sdl/sdl_input_translator.hpp>, so the directory holding it
    # is public. SDL2 is private: the header crosses native events as const void*, so SDL never
    # leaks into a consumer's translation unit.
    target_include_directories(sushi_input
        PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/source/backend")
    target_link_libraries(sushi_input PUBLIC sushiengine_input PRIVATE SDL2::SDL2)
    set_target_properties(sushi_input PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON)
endif()
