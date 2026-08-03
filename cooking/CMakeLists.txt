# ============================================================================
# sushi_cooking — the offline cooking pipeline (physics/cooking, §3.1)
#
# Mesh in, simulation asset out. Host-only and never linked into a shipping
# runtime path: it is the one module allowed to be slow, allocate freely, and
# depend on geometry/ (docs/slop/physics_system.md §3.1, §8).
#
# It links `sushi_geometry` and nothing else. No Vulkan, no SDL, no SYCL, no
# sushiruntime — an importer that needs a GPU is an importer that fails on a
# build machine, and the whole point of the neutral geometry module was to make
# this target possible at all. §6.6 leaves the door open for the two heaviest
# stages to be dispatched through the runtime when one is present; that is an
# optional acceleration behind the same ICookingStage seam, not a dependency,
# and the rule "cooking must work with no device present" outranks it.
#
# The source directory is `cooking/` while the headers live under
# include/SushiEngine/physics/cooking/ — the same arrangement `geometry/` and
# `sim/` already use, since the include tree follows the module layout and the
# source tree follows the build.
# ============================================================================

add_library(sushi_cooking STATIC
    cooking_parameters.cpp
    cooked_asset_store.cpp
    convex_decomposition.cpp
    collision_asset.cpp
    collision_cooker.cpp
    tetrahedral_mesh.cpp
    soft_body_asset.cpp
    soft_body_cooker.cpp
    node_beam_asset.cpp
    node_beam_cooker.cpp
    mesh_post_processor.cpp
    cooking_service.cpp)

target_include_directories(sushi_cooking PUBLIC "${CMAKE_SOURCE_DIR}/include")

# `CookingService` owns a worker thread, because §8.1 puts cooking off the main thread and
# three seconds of frozen editor per dropped asset is a pipeline artists route around by not
# dropping assets in. Threads is the only addition; the glTF importer is deliberately *not*
# linked — the service takes a `MeshLoader` seam, so this target still depends on nothing but
# geometry and can be built on a machine with no cgltf and no device.
find_package(Threads REQUIRED)
target_link_libraries(sushi_cooking PUBLIC sushiengine_core sushi_geometry Threads::Threads)
target_compile_features(sushi_cooking PUBLIC cxx_std_17)
set_target_properties(sushi_cooking PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON)
