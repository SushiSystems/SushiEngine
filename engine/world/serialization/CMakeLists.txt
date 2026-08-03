# serialization — the one JSON shape for a `.sushiscene` file, a `.sushieffect` particle asset,
# and the environment both of them embed. UI-free by construction: every function here is driven
# through Simulation::IWorldEditor's query surface and nlohmann::json, so this module names no
# editor or ImGui type and anything that has to read or write a scene can link it.

find_package(nlohmann_json CONFIG REQUIRED)

sushiengine_add_module(NAME serialization LAYER world
    SOURCES
        source/scene_serializer.cpp
        source/effect_serializer.cpp
        source/environment_serializer.cpp
    # Public because each is spelled in a header this target publishes: the scene shape is written
    # against IWorldEditor and carries the sky's epoch as a Julian date, the effect shape against a
    # ParticleEffect, and the environment shape against the environment itself.
    PUBLIC_DEPENDS astro environment simulation vfx
    # Only effect_serializer.cpp names a material, resolving an effect's textures against the
    # asset library it is handed.
    PRIVATE_DEPENDS material
    EXTERNAL_PUBLIC nlohmann_json::nlohmann_json)

# Its headers are spelled by bare file name, so the directory holding them is public too.
target_include_directories(sushiengine_serialization
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/SushiEngine/serialization")

# IAssetLibrary, which both serializers resolve a texture or mesh name through. The include root
# rather than the module: a configure with SE_BUILD_RENDER off has no library to link.
target_include_directories(sushiengine_serialization
    PRIVATE "${CMAKE_SOURCE_DIR}/engine/presentation/render/include")

# The name the editor, the player, the sample-scene generator and the test suite link.
add_library(sushi_scene ALIAS sushiengine_serialization)
