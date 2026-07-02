# ============================================================================
# SushiEngine renderer
#
# A greenfield Vulkan 1.3 renderer behind an RHI abstraction (include/SushiEngine/
# render/rhi/). Consumers program against the abstract IRenderDevice; the Vulkan
# backend (rhi/vulkan/) is the only implementation today. Like the editor this is a
# plain compiled target — it does NOT link the runtime or compile SYCL, so it builds
# on a stock toolchain. Interop with SushiRuntime (sharing USM with Vulkan) is a
# later milestone layered on top of this device.
#
# Vulkan, VMA, vk-bootstrap, and glslang come from vcpkg (declared in
# cli/sushistack.deps.toml, provisioned by `ss install`).
# ============================================================================

find_package(VulkanHeaders CONFIG REQUIRED)
find_package(VulkanLoader CONFIG REQUIRED)
find_package(VulkanMemoryAllocator CONFIG REQUIRED)
find_package(vk-bootstrap CONFIG REQUIRED)
find_package(glslang CONFIG REQUIRED)

# --- Build-time shader compilation ------------------------------------------
# A host tool compiles GLSL to SPIR-V and emits a C++ header of embedded words, so
# the renderer ships SPIR-V and carries no runtime shader-compiler dependency.
add_executable(shader_compiler tools/shader_compiler/main.cpp)
target_link_libraries(shader_compiler PRIVATE
    glslang::glslang
    glslang::glslang-default-resource-limits
    glslang::SPIRV)
set_target_properties(shader_compiler PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON)

set(SHADER_GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
file(MAKE_DIRECTORY "${SHADER_GEN_DIR}")

# sushi_compile_shader(<vert|frag> <input> <symbol> <out_var>)
#   Emits a rule that compiles <input> to <SHADER_GEN_DIR>/<input-name>.h exposing
#   SushiEngine::render::shaders::<symbol>; sets <out_var> to the generated header.
function(sushi_compile_shader stage input symbol out_var)
    get_filename_component(name "${input}" NAME)
    set(output "${SHADER_GEN_DIR}/${name}.h")
    add_custom_command(
        OUTPUT "${output}"
        COMMAND shader_compiler ${stage} "${input}" "${output}" ${symbol}
        DEPENDS shader_compiler "${input}"
        COMMENT "Compiling shader ${name} -> SPIR-V header"
        VERBATIM)
    set(${out_var} "${output}" PARENT_SCOPE)
endfunction()

sushi_compile_shader(vert "${CMAKE_CURRENT_SOURCE_DIR}/shaders/triangle.vert"
    triangle_vert_spv TRIANGLE_VERT_HEADER)
sushi_compile_shader(frag "${CMAKE_CURRENT_SOURCE_DIR}/shaders/triangle.frag"
    triangle_frag_spv TRIANGLE_FRAG_HEADER)
sushi_compile_shader(vert "${CMAKE_CURRENT_SOURCE_DIR}/shaders/mesh.vert"
    mesh_vert_spv MESH_VERT_HEADER)
sushi_compile_shader(frag "${CMAKE_CURRENT_SOURCE_DIR}/shaders/mesh.frag"
    mesh_frag_spv MESH_FRAG_HEADER)
sushi_compile_shader(frag "${CMAKE_CURRENT_SOURCE_DIR}/shaders/line.frag"
    line_frag_spv LINE_FRAG_HEADER)

# --- Renderer library --------------------------------------------------------
add_library(sushi_render STATIC
    rhi/vulkan/vulkan_device.cpp
    rhi/vulkan/vulkan_offscreen.cpp
    rhi/vulkan/vulkan_window_renderer.cpp
    rhi/vulkan/vulkan_scene_view.cpp
    rhi/vulkan/vma_impl.cpp
    "${TRIANGLE_VERT_HEADER}"
    "${TRIANGLE_FRAG_HEADER}"
    "${MESH_VERT_HEADER}"
    "${MESH_FRAG_HEADER}"
    "${LINE_FRAG_HEADER}")
target_include_directories(sushi_render
    PUBLIC "${CMAKE_SOURCE_DIR}/include"
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}" "${SHADER_GEN_DIR}")
target_link_libraries(sushi_render PUBLIC
    Vulkan::Headers
    Vulkan::Loader
    GPUOpen::VulkanMemoryAllocator
    vk-bootstrap::vk-bootstrap)
target_compile_features(sushi_render PUBLIC cxx_std_17)

# Headless smoke test: brings the device up, renders the triangle offscreen, and
# reads the pixels back to prove the pipeline path — the render analogue of the ECS
# sandbox, and it runs in CI without a display.
add_executable(render_probe probe/main.cpp)
target_include_directories(render_probe PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(render_probe PRIVATE sushi_render)
set_target_properties(render_probe PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON)
