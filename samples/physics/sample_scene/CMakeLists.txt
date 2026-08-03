# sample_scene — the physics demonstration scene, and the program that writes it to a
# `.sushiscene`. Generated rather than hand-authored, and deliberately: the file comes out of
# the same builder the integration suite steps and asserts on, so the shipped scene cannot
# drift from the code that is tested.
#
# The builder is a library because three consumers need it — the generator here, the
# integration suite, and anything that later wants the scene without a file — and compiling one
# translation unit into each of them by relative path is how a directory ends up owning source
# it does not contain.

add_library(sushiengine_physics_sample_scene STATIC physics_sample_scene.cpp)
target_include_directories(sushiengine_physics_sample_scene
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
# Public: the builder's only argument is a Simulation::IWorldEditor, which its header spells.
target_link_libraries(sushiengine_physics_sample_scene PUBLIC sushiengine_simulation)
set_target_properties(sushiengine_physics_sample_scene PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    # Position-independent for the same reason sushiengine_simulation is: the shells that link
    # it are.
    POSITION_INDEPENDENT_CODE ON)

if(SUSHIENGINE_BUILD_SAMPLE_DEMOS)
    # nlohmann_json is an imported target and therefore directory-scoped, so the directory that
    # links sushiengine_serialization has to find it even though it names no JSON header itself.
    find_package(nlohmann_json CONFIG REQUIRED)
    sushiengine_add_sample(physics_sample_scene main.cpp)
    target_link_libraries(physics_sample_scene
        PRIVATE sushiengine_physics_sample_scene
                sushiengine_serialization
                nlohmann_json::nlohmann_json)
endif()
