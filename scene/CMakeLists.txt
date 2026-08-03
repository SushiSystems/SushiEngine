# ============================================================================
# sushi_scene
#
# The one JSON shape for a `.sushiscene` file, a `.sushieffect` particle asset,
# and the `Render::Environment` both embed. UI-free by construction: every
# function here is driven purely through `Simulation::IWorldEditor`'s query
# surface and `nlohmann::json`, so this library names no editor or ImGui type
# and can be linked by anything that needs to read or write a scene file —
# the editor, `se_player`, and `physics_sample_scene` alike.
# ============================================================================

find_package(nlohmann_json CONFIG REQUIRED)

add_library(sushi_scene STATIC
    scene_serializer.cpp
    effect_serializer.cpp
    environment_serializer.cpp)
target_include_directories(sushi_scene PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(sushi_scene PUBLIC sushi_sim nlohmann_json::nlohmann_json)
set_target_properties(sushi_scene PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON)
