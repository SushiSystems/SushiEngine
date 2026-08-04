# geometry — the triangle mesh, the distance hierarchy every cooking stage queries, and
# the field bake over it. It depends on nothing, deliberately: the renderer and the physics
# both read a mesh's distance field and neither may own it, and a cooker that needed a
# device would fail on a build machine (docs/design/physics_system.md §3.4).
sushiengine_add_module(NAME geometry LAYER domain
    SOURCES
        source/mesh_utilities.cpp
        source/meshlet.cpp
        source/mesh_distance_query.cpp
        source/signed_distance_field.cpp)
