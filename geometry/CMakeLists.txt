# ============================================================================
# sushi_geometry — the engine-neutral geometry module
#
# The layer everything that reasons about a surface reads, and the layer nothing
# reasons about it *above*. It links nothing: no Vulkan, no SDL, no SYCL, no
# sushiruntime. That is the whole point — the renderer sits above it and so does
# the physics, and a mesh's distance field stopped being a global-illumination
# private the moment a collision narrowphase and an offline cooker both wanted one.
#
# It must stay dependency-free. An importer that needs a GPU is an importer that
# fails on a build machine, and the cooking pipeline this module exists to unblock
# is host-only by design (docs/slop/physics_system.md §3.4).
#
# The headers are the include/SushiEngine/geometry/ tree, reached through the
# SushiEngine INTERFACE target; only the baker has a compiled unit.
# ============================================================================

add_library(sushi_geometry STATIC
    signed_distance_field.cpp)

target_include_directories(sushi_geometry PUBLIC "${CMAKE_SOURCE_DIR}/include")
target_compile_features(sushi_geometry PUBLIC cxx_std_17)
set_target_properties(sushi_geometry PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON)
