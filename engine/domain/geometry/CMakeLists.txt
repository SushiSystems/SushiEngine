# geometry — the triangle mesh, the distance hierarchy every cooking stage queries, and
# the field bake over it. Depends only on core, for the Vector3/Matrix4 types the
# thumbnail-camera math (mesh_thumbnail_camera.cpp) and mesh vertex data already need.
sushiengine_add_module(NAME geometry LAYER domain
    SOURCES
        source/mesh_utilities.cpp
        source/meshlet.cpp
        source/mesh_distance_query.cpp
        source/signed_distance_field.cpp
        source/mesh_thumbnail_camera.cpp
    PUBLIC_DEPENDS core)
