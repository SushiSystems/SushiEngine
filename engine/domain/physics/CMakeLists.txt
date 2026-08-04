# physics — what a simulated body is: colliders, constraints, the solvers that move them,
# soft bodies, vehicles, and the cooking seam that turns a mesh into any of those. Header-only
# so its kernels instantiate inside the consuming translation unit, which is the safe way to
# ship device code.
sushiengine_add_module(NAME physics LAYER domain PUBLIC_DEPENDS core execution geometry)

# The offline half of physics/cooking: mesh in, simulation asset out. Its own target rather
# than its own module, because it is host-only and never linked into a shipping runtime path
# — it is the one part of physics allowed to be slow and to allocate freely
# (docs/design/physics_system.md §3.1, §8). Every consumer it has (the editor, the live
# simulation, the test suite) is a runtime-lane target, and the native lane carries no target
# it has not proved out, so it is declared on that lane only.
if(SUSHIENGINE_EXECUTION_BACKEND STREQUAL "runtime")
    # `CookingService` owns a worker thread, because §8.1 puts cooking off the main thread and
    # three seconds of frozen editor per dropped asset is a pipeline artists route around by
    # not dropping assets in. Threads is the only addition; the glTF importer is deliberately
    # *not* linked — the service takes a `MeshLoader` seam, so this still builds on a machine
    # with no cgltf and no device.
    find_package(Threads REQUIRED)

    add_library(sushiengine_physics_cooking STATIC
        source/cooking/cooking_parameters.cpp
        source/cooking/cooked_asset_store.cpp
        source/cooking/convex_decomposition.cpp
        source/cooking/collision_asset.cpp
        source/cooking/collision_cooker.cpp
        source/cooking/tetrahedral_mesh.cpp
        source/cooking/soft_body_asset.cpp
        source/cooking/soft_body_cooker.cpp
        source/cooking/node_beam_asset.cpp
        source/cooking/node_beam_cooker.cpp
        source/cooking/mesh_post_processor.cpp
        source/cooking/cooking_service.cpp)
    target_link_libraries(sushiengine_physics_cooking
        PUBLIC sushiengine_physics sushiengine_geometry Threads::Threads)
    set_target_properties(sushiengine_physics_cooking PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON)
endif()
