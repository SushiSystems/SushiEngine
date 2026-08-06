# model_import — the path from a glTF file on disk to a `.sushiprefab` beside it. It is the one
# place that knows all four of reading the file, reading its `.meta`, planning what entities it
# becomes, and writing the result, which is why it sits above every module that knows exactly one
# of them. `model` itself cannot do this: it is an `asset`-tier module while `IWorldEditor` and the
# prefab shape are both `world`, an edge `sushiengine_add_module` refuses at configure time.

sushiengine_add_module(NAME model_import LAYER world
    SOURCES
        source/prefab_output.cpp
    # A caller names ModelImportReport to read what the import could not carry across, and
    # ModelInstantiationPlan to instantiate one it planned itself. `gltf` arrives publicly through
    # `model`, so it is not repeated here.
    PUBLIC_DEPENDS model simulation
    # The rest of the path; neither appears in this module's header.
    PRIVATE_DEPENDS serialization)

# Its header is spelled `SushiEngine/model_import/prefab_output.hpp`, so the include root is the
# directory holding `SushiEngine`, not the one holding the header.
target_include_directories(sushiengine_model_import
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
