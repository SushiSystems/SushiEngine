# gltf — a glTF file's triangles as a Geometry::TriangleMesh, its node graph as a
# Geometry::GLTFSceneDescription, and its skin and animations as the relocatable blobs the
# animator plays. It links no device and no graphics stack,
# deliberately: the cooking pipeline starts at a file rather than at a mesh somebody already
# had, and reaching one through the renderer's importer would mean bringing up a graphics stack
# to cook a collider (docs/design/physics_system.md §3.4). It also owns cgltf's one
# implementation unit, which is what the renderer links it for — the renderer includes the
# header for its types and takes the symbols from here.

# cgltf ships nothing but the header, so it is located by the header itself — the documented
# vcpkg approach for a port with no CMake export.
find_path(CGLTF_INCLUDE_DIR "cgltf.h" REQUIRED)

# geometry is public because mesh_import.hpp hands back a TriangleMesh, and core because
# scene_import.hpp names Vector3T and QuaternionT in the node description it hands back.
# animation is private: only the sources name a skeleton, clip and morph blob.
sushiengine_add_module(NAME gltf LAYER asset
    SOURCES
        source/cgltf_implementation.cpp
        source/mesh_importer.cpp
        source/scene_importer.cpp
        source/skeleton_importer.cpp
        source/animation_importer.cpp
    PUBLIC_DEPENDS geometry core
    PRIVATE_DEPENDS animation)

# Private because no header here names a cgltf type, so nothing that reads an imported mesh or
# blob has to locate the parser to compile against it.
target_include_directories(sushiengine_gltf PRIVATE "${CGLTF_INCLUDE_DIR}")
