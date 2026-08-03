# ============================================================================
# se_player
#
# PLATFORM0 S5's ImGui-free runtime shell. Presents through the same Vulkan
# renderer the editor uses (sushi_render), and drives the same ISimulation seam
# (sushi_sim/sushi_scene) — but never links sushi_imgui, which is the entire
# point of PLATFORM0's editor/player split. A future S6 headless mode and
# `se player` CLI subcommand build on top of this skeleton; neither exists yet.
#
# If a later change accidentally makes se_player depend on sushi_imgui (a stray
# editor/ include pulled in transitively, say), that dependency would have to
# show up as an explicit link here to resolve — there is no test target for
# this beyond the CMake graph itself never naming sushi_imgui below.
# ============================================================================

if(NOT TARGET sushi_render)
    message(FATAL_ERROR
        "The player needs the Vulkan renderer. Configure with -DSE_BUILD_RENDER=ON "
        "(the top-level CMake enables it automatically when SE_BUILD_PLAYER is on).")
endif()

find_package(SDL2 CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)

add_executable(se_player
    main.cpp
    player_app.cpp
    boot_manifest.cpp)
target_link_libraries(se_player PRIVATE
    sushi_platform sushi_render sushi_sim sushi_scene sushi_input
    nlohmann_json::nlohmann_json)
# SDL2 defines SDL_main on some platforms (Windows); link the entry shim when the
# target exists so WinMain resolves without forcing it on Linux/macOS.
if(TARGET SDL2::SDL2main)
    target_link_libraries(se_player PRIVATE SDL2::SDL2main)
endif()
set_target_properties(se_player PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON)

# On Windows the SDL2 runtime DLL must sit next to the executable.
if(WIN32 AND TARGET SDL2::SDL2)
    add_custom_command(TARGET se_player POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:SDL2::SDL2>
            $<TARGET_FILE_DIR:se_player>)
endif()

# The runtime DLL must sit next to the player too, since sushi_sim links it.
if(WIN32 AND TARGET sushiruntime)
    add_custom_command(TARGET se_player POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:sushiruntime>
            $<TARGET_FILE_DIR:se_player>)
endif()
