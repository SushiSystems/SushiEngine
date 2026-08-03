# authoring — the services an editor is built out of, with no editor in them: the undo stack that
# snapshots a world, the persisted preferences, the autosave clock, the cook/bake model behind the
# Bake window, and the heat scale the soft-body overlay colours by. World rather than under an
# application directory, because the test suite drives all five with no editor in sight and
# nothing outside an application directory may depend on one.

find_package(nlohmann_json CONFIG REQUIRED)

sushiengine_add_module(NAME authoring LAYER world
    SOURCES
        source/command_history.cpp
        source/preferences.cpp
        source/cook_bake_state.cpp
    # Public because each is spelled in a header this target publishes: command_history.hpp
    # snapshots a world through IWorldEditor and owns the Scene::SceneBlobTable its snapshots
    # name cooked assets by, preferences.hpp holds the environment a new scene starts from and
    # the simulation's own quality budgets, cook_bake_state.hpp the mesh a cook starts at, and
    # soft_body_heat.hpp the material a stress reading is scaled against.
    PUBLIC_DEPENDS core environment geometry physics serialization simulation
    # nlohmann_json is public because command_history.hpp holds a snapshot as a json document.
    # sushiengine_physics_cooking is public for the same reason: cook_bake_state.hpp is written
    # against the cooking service and the three cooked asset kinds it produces.
    EXTERNAL_PUBLIC nlohmann_json::nlohmann_json sushiengine_physics_cooking)

# RenderSettings, which preferences.hpp persists as a host setting. The include root rather than
# the module: a configure with SUSHIENGINE_BUILD_RENDER off has no library to link.
target_include_directories(sushiengine_authoring
    PUBLIC "${CMAKE_SOURCE_DIR}/engine/presentation/render/include")
