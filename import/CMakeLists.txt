# ============================================================================
# sushi_import — the engine-neutral asset importers
#
# glTF in, engine blob out. It links nothing: no Vulkan, no SDL, no SYCL, no
# sushiruntime — the same rule sushi_geometry follows, and for the same reason.
#
# These two translation units used to sit in render/material/, which made a
# skeleton importer part of the Vulkan renderer. Nothing in them ever touched a
# device: they read a glTF file and write an Animation::SkeletonBlob or ClipBlob.
# The cost of the misplacement was concrete rather than theoretical — the test lane
# needed `import_gltf_skeleton`, could not link the renderer to get it, and was
# repaired by compiling render/'s sources into the test target: a stopgap that
# worked by pretending the layering was different than it was.
#
# It also owns cgltf's single implementation unit, because that is what the
# renderer was really being linked for. The renderer includes cgltf.h for its
# types and takes the symbols from here.
# ============================================================================

find_path(CGLTF_INCLUDE_DIR "cgltf.h" REQUIRED)

add_library(sushi_import STATIC
    cgltf_impl.cpp
    gltf_skeleton_importer.cpp
    gltf_animation_importer.cpp
    # §8.1's MeshImporter box: a glTF file's triangles as a Geometry::TriangleMesh, with no
    # renderer between the file and them. The cooking pipeline starts at a file rather than
    # at a mesh somebody already had, and reaching one through the renderer's importer would
    # mean bringing up a graphics stack to cook a collider.
    gltf_mesh_importer.cpp)

target_include_directories(sushi_import PUBLIC
    "${CMAKE_SOURCE_DIR}/include"
    "${CGLTF_INCLUDE_DIR}")
# The imported blobs are spelled in core's scalar and vector types.
target_link_libraries(sushi_import PUBLIC sushiengine_core)
target_compile_features(sushi_import PUBLIC cxx_std_17)
set_target_properties(sushi_import PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON)
