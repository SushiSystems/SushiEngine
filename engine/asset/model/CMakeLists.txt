# model — what a model asset says about how it is imported, and the decision that turns a glTF
# node graph into a list of entities to create. It links no device and no editor, deliberately:
# every hard decision in the import path is here, so all of it is testable on a machine with no
# GPU and no window.

find_package(nlohmann_json CONFIG REQUIRED)

# physics is public because import_settings.hpp embeds an ImportProfileOverride. gltf is public
# because instantiation_plan.hpp names a GLTFSceneDescription. core is public for Vector3f.
sushiengine_add_module(NAME model LAYER asset
    SOURCES
        source/import_settings_io.cpp
        source/instantiation_plan.cpp
    PUBLIC_DEPENDS core physics gltf
    # nlohmann_json is private: the `.meta` sidecar's encoding is this module's own business
    # and no header here names a json document.
    EXTERNAL_PRIVATE nlohmann_json::nlohmann_json)
