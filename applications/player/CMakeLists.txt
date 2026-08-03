# player — PLATFORM0 S5's ImGui-free runtime shell. Presents through the same Vulkan renderer
# the editor uses (sushiengine_render), and drives the same ISimulation seam
# (sushiengine_simulation/sushiengine_serialization) — but never links sushiengine_imgui, which
# is the entire point of PLATFORM0's editor/player split. A future S6 headless mode builds on
# top of this skeleton; it does not exist yet.
#
# If a later change accidentally makes sushiengine_player depend on sushiengine_imgui (a stray
# include from the editor's own source tree pulled in transitively, say), that dependency would
# have to show up as an explicit link here to resolve — there is no test target for this beyond
# the CMake graph itself never naming sushiengine_imgui below.

if(NOT TARGET sushiengine_render)
    message(FATAL_ERROR
        "The player needs the Vulkan renderer. Configure with -DSUSHIENGINE_BUILD_RENDER=ON "
        "(the top-level CMake enables it automatically when SUSHIENGINE_BUILD_PLAYER is on).")
endif()

find_package(SDL2 CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)

add_executable(sushiengine_player
    source/main.cpp
    source/player_app.cpp
    source/boot_manifest.cpp
    source/launch_configuration.cpp)
target_link_libraries(sushiengine_player PRIVATE
    sushiengine_platform
    sushiengine_render
    sushiengine_simulation
    sushiengine_serialization
    sushiengine_input_backend
    nlohmann_json::nlohmann_json)
# SDL2 defines SDL_main on some platforms (Windows); link the entry shim when the
# target exists so WinMain resolves without forcing it on Linux/macOS.
if(TARGET SDL2::SDL2main)
    target_link_libraries(sushiengine_player PRIVATE SDL2::SDL2main)
endif()
# The target carries the repository's prefix so it cannot collide with a SushiRuntime one in the
# single flat CMake namespace the two trees share; the file on disk keeps the short name, which
# is what `se player` looks for and what a shipped build is launched as.
set_target_properties(sushiengine_player PROPERTIES
    OUTPUT_NAME se_player
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON)

# On Windows the SDL2 runtime DLL must sit next to the executable.
if(WIN32 AND TARGET SDL2::SDL2)
    add_custom_command(TARGET sushiengine_player POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:SDL2::SDL2>
            $<TARGET_FILE_DIR:sushiengine_player>)
endif()

# The runtime DLL must sit next to the player too, since sushiengine_simulation links it.
if(WIN32 AND TARGET sushiruntime)
    add_custom_command(TARGET sushiengine_player POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:sushiruntime>
            $<TARGET_FILE_DIR:sushiengine_player>)
endif()
