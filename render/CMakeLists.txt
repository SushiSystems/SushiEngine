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
# The library is organised as the render-graph stack: graph/ derives barriers and
# resource lifetimes, resources/ owns the caches and pools, passes/ holds one file
# per pass, scene/ and geometry/ carry the shared scene data, and rhi/vulkan/ is the
# only place Vulkan-specific device code lives.
#
# Vulkan, VMA, vk-bootstrap, and glslang come from vcpkg (declared in
# cli/sushistack.deps.toml, provisioned by `ss install`).
# ============================================================================

find_package(VulkanHeaders CONFIG REQUIRED)
find_package(VulkanLoader CONFIG REQUIRED)
find_package(VulkanMemoryAllocator CONFIG REQUIRED)
find_package(vk-bootstrap CONFIG REQUIRED)
find_package(glslang CONFIG REQUIRED)
# The pipeline factory runs a background optimizer thread; Threads links the
# platform's threading primitives (a no-op link on Windows, pthread on Linux).
find_package(Threads REQUIRED)
# stb and cgltf are header-only. stb ships a find module (reached through vcpkg's
# wrapper); cgltf ships nothing but the header, so it is located by the header itself
# — the documented vcpkg approach for a port with no CMake export.
find_package(Stb REQUIRED)
find_path(CGLTF_INCLUDE_DIR "cgltf.h" REQUIRED)

# --- Build-time shader compilation ------------------------------------------
# A host tool compiles GLSL to SPIR-V and emits a C++ header of embedded words, so
# the renderer ships SPIR-V and needs no shader files at runtime. The same glslang
# is also linked into the renderer, where it backs shader hot-reload against the
# source tree during development.
add_executable(shader_compiler tools/shader_compiler/main.cpp)
target_include_directories(shader_compiler PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(shader_compiler PRIVATE
    glslang::glslang
    glslang::glslang-default-resource-limits
    glslang::SPIRV)
set_target_properties(shader_compiler PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON)

set(SHADER_GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
file(MAKE_DIRECTORY "${SHADER_GEN_DIR}")

# Shared GLSL pulled in by #include; every shader depends on it, because the tool
# resolves includes against the shader directory and cannot report what it read.
set(SHADER_INCLUDES
    "${CMAKE_CURRENT_SOURCE_DIR}/shaders/cloud_noise_common.glsl"
    "${CMAKE_CURRENT_SOURCE_DIR}/shaders/pbr_common.glsl"
    "${CMAKE_CURRENT_SOURCE_DIR}/shaders/ibl_common.glsl"
    "${CMAKE_CURRENT_SOURCE_DIR}/shaders/temporal_common.glsl"
    "${CMAKE_CURRENT_SOURCE_DIR}/shaders/shadow_common.glsl"
    "${CMAKE_CURRENT_SOURCE_DIR}/shaders/shadow_sampling.glsl"
    "${CMAKE_CURRENT_SOURCE_DIR}/shaders/cloud_shadow_common.glsl"
    "${CMAKE_CURRENT_SOURCE_DIR}/shaders/clustered_lighting.glsl"
    "${CMAKE_CURRENT_SOURCE_DIR}/shaders/blue_noise.glsl"
    "${CMAKE_CURRENT_SOURCE_DIR}/shaders/atmosphere_common.glsl"
    "${CMAKE_CURRENT_SOURCE_DIR}/shaders/gi_common.glsl"
    "${CMAKE_CURRENT_SOURCE_DIR}/shaders/sdf_common.glsl")

# sushi_compile_shader(<vert|frag|comp> <input> <symbol> <out_var>)
#   Emits a rule that compiles <input> to <SHADER_GEN_DIR>/<input-name>.h exposing
#   SushiEngine::Render::Shaders::<symbol>; sets <out_var> to the generated header.
function(sushi_compile_shader stage input symbol out_var)
    get_filename_component(name "${input}" NAME)
    set(output "${SHADER_GEN_DIR}/${name}.h")
    add_custom_command(
        OUTPUT "${output}"
        COMMAND shader_compiler ${stage} "${input}" "${output}" ${symbol}
        DEPENDS shader_compiler "${input}" ${SHADER_INCLUDES}
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
sushi_compile_shader(frag "${CMAKE_CURRENT_SOURCE_DIR}/shaders/pbr.frag"
    pbr_frag_spv PBR_FRAG_HEADER)
sushi_compile_shader(frag "${CMAKE_CURRENT_SOURCE_DIR}/shaders/line.frag"
    line_frag_spv LINE_FRAG_HEADER)
sushi_compile_shader(frag "${CMAKE_CURRENT_SOURCE_DIR}/shaders/outline.frag"
    outline_frag_spv OUTLINE_FRAG_HEADER)
sushi_compile_shader(vert "${CMAKE_CURRENT_SOURCE_DIR}/shaders/outline.vert"
    outline_vert_spv OUTLINE_VERT_HEADER)
sushi_compile_shader(vert "${CMAKE_CURRENT_SOURCE_DIR}/shaders/fullscreen.vert"
    fullscreen_vert_spv FULLSCREEN_VERT_HEADER)
sushi_compile_shader(frag "${CMAKE_CURRENT_SOURCE_DIR}/shaders/sky.frag"
    sky_frag_spv SKY_FRAG_HEADER)
sushi_compile_shader(frag "${CMAKE_CURRENT_SOURCE_DIR}/shaders/cloud.frag"
    cloud_frag_spv CLOUD_FRAG_HEADER)
sushi_compile_shader(frag "${CMAKE_CURRENT_SOURCE_DIR}/shaders/tonemap.frag"
    tonemap_frag_spv TONEMAP_FRAG_HEADER)
sushi_compile_shader(vert "${CMAKE_CURRENT_SOURCE_DIR}/shaders/shadow.vert"
    shadow_vert_spv SHADOW_VERT_HEADER)
sushi_compile_shader(frag "${CMAKE_CURRENT_SOURCE_DIR}/shaders/contact_shadow.frag"
    contact_shadow_frag_spv CONTACT_SHADOW_FRAG_HEADER)
sushi_compile_shader(frag "${CMAKE_CURRENT_SOURCE_DIR}/shaders/ray_shadow.frag"
    ray_shadow_frag_spv RAY_SHADOW_FRAG_HEADER)
sushi_compile_shader(frag "${CMAKE_CURRENT_SOURCE_DIR}/shaders/cloud_composite.frag"
    cloud_composite_frag_spv CLOUD_COMPOSITE_FRAG_HEADER)
sushi_compile_shader(frag "${CMAKE_CURRENT_SOURCE_DIR}/shaders/taa.frag"
    taa_frag_spv TAA_FRAG_HEADER)
sushi_compile_shader(frag "${CMAKE_CURRENT_SOURCE_DIR}/shaders/fxaa.frag"
    fxaa_frag_spv FXAA_FRAG_HEADER)
sushi_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/shading_rate.comp"
    shading_rate_comp_spv SHADING_RATE_HEADER)
sushi_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/cloud_noise_volume.comp"
    cloud_noise_volume_comp_spv CLOUD_NOISE_VOLUME_HEADER)
sushi_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/cloud_noise_weather.comp"
    cloud_noise_weather_comp_spv CLOUD_NOISE_WEATHER_HEADER)
sushi_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/ibl_prefilter.comp"
    ibl_prefilter_comp_spv IBL_PREFILTER_HEADER)
sushi_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/ibl_irradiance.comp"
    ibl_irradiance_comp_spv IBL_IRRADIANCE_HEADER)
sushi_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/brdf_lut.comp"
    brdf_lut_comp_spv BRDF_LUT_HEADER)
sushi_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/sh_project.comp"
    sh_project_comp_spv SH_PROJECT_HEADER)
sushi_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/cluster_build.comp"
    cluster_build_comp_spv CLUSTER_BUILD_HEADER)
sushi_compile_shader(vert "${CMAKE_CURRENT_SOURCE_DIR}/shaders/light_shadow.vert"
    light_shadow_vert_spv LIGHT_SHADOW_VERT_HEADER)
sushi_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/gtao.comp"
    gtao_comp_spv GTAO_COMP_HEADER)
sushi_compile_shader(frag "${CMAKE_CURRENT_SOURCE_DIR}/shaders/gtao_resolve.frag"
    gtao_resolve_frag_spv GTAO_RESOLVE_FRAG_HEADER)
sushi_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/hiz.comp"
    hiz_comp_spv HIZ_COMP_HEADER)
sushi_compile_shader(frag "${CMAKE_CURRENT_SOURCE_DIR}/shaders/ssr.frag"
    ssr_frag_spv SSR_FRAG_HEADER)
sushi_compile_shader(frag "${CMAKE_CURRENT_SOURCE_DIR}/shaders/ground_shadow_resolve.frag"
    ground_shadow_resolve_frag_spv GROUND_SHADOW_RESOLVE_FRAG_HEADER)
sushi_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/transmittance_lut.comp"
    transmittance_lut_comp_spv TRANSMITTANCE_LUT_HEADER)
sushi_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/multiscatter_lut.comp"
    multiscatter_lut_comp_spv MULTISCATTER_LUT_HEADER)
sushi_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/sky_view_lut.comp"
    sky_view_lut_comp_spv SKY_VIEW_LUT_HEADER)
sushi_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/aerial_perspective.comp"
    aerial_perspective_comp_spv AERIAL_PERSPECTIVE_HEADER)
sushi_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/fog_scatter.comp"
    fog_scatter_comp_spv FOG_SCATTER_HEADER)
sushi_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/gi_probe_relight.comp"
    gi_probe_relight_comp_spv GI_PROBE_RELIGHT_HEADER)
sushi_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/sdf_populate.comp"
    sdf_populate_comp_spv SDF_POPULATE_HEADER)
sushi_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/sdf_probe_relight.comp"
    sdf_probe_relight_comp_spv SDF_PROBE_RELIGHT_HEADER)

# --- Renderer library --------------------------------------------------------
add_library(sushi_render STATIC
    shader_catalogue.cpp
    graph/render_graph.cpp
    graph/resource_state.cpp
    graph/gpu_profiler.cpp
    resources/transient_pool.cpp
    resources/descriptor_allocator.cpp
    resources/descriptor_heap.cpp
    resources/pipeline_cache.cpp
    resources/sampler_cache.cpp
    resources/shader_library.cpp
    scene/scene_layout.cpp
    scene/scene_uniforms.cpp
    scene/temporal_uniforms.cpp
    scene/motion_system.cpp
    scene/shadow_uniforms.cpp
    lighting/light_system.cpp
    raytracing/scene_accelerator.cpp
    frame/temporal_jitter.cpp
    frame/resolution_controller.cpp
    frame/quality.cpp
    geometry/mesh_registry.cpp
    geometry/cloth_buffers.cpp
    material/asset_library.cpp
    material/texture_library.cpp
    material/material_system.cpp
    material/gltf_importer.cpp
    textures/cloud_noise.cpp
    passes/fullscreen.cpp
    passes/ibl_pass.cpp
    passes/atmosphere_lut_pass.cpp
    passes/volumetric_fog_pass.cpp
    passes/irradiance_volume_pass.cpp
    gi/probe_volume.cpp
    gi/environment_probe_tracer.cpp
    gi/sdf_clipmap.cpp
    gi/sdf_probe_tracer.cpp
    gi/mesh_sdf_baker.cpp
    passes/opaque_pass.cpp
    passes/gtao_pass.cpp
    passes/hiz_pass.cpp
    passes/ssr_pass.cpp
    passes/light_cull_pass.cpp
    passes/light_shadow_pass.cpp
    passes/sky_pass.cpp
    passes/ground_shadow_resolve_pass.cpp
    passes/cloud_pass.cpp
    passes/depth_only.cpp
    passes/depth_prepass.cpp
    passes/shadow_pass.cpp
    passes/contact_shadow_pass.cpp
    passes/ray_traced_shadow_pass.cpp
    passes/cloud_composite_pass.cpp
    passes/taa_pass.cpp
    passes/fxaa_pass.cpp
    passes/shading_rate_pass.cpp
    passes/tonemap_pass.cpp
    passes/picking_pass.cpp
    rhi/vulkan/vulkan_device.cpp
    rhi/vulkan/vulkan_offscreen.cpp
    rhi/vulkan/vulkan_window_renderer.cpp
    rhi/vulkan/vulkan_scene_view.cpp
    rhi/vulkan/view_resources.cpp
    rhi/vulkan/vma_impl.cpp
    material/stb_impl.cpp
    "${TRIANGLE_VERT_HEADER}"
    "${TRIANGLE_FRAG_HEADER}"
    "${MESH_VERT_HEADER}"
    "${PBR_FRAG_HEADER}"
    "${LINE_FRAG_HEADER}"
    "${OUTLINE_FRAG_HEADER}"
    "${OUTLINE_VERT_HEADER}"
    "${FULLSCREEN_VERT_HEADER}"
    "${SKY_FRAG_HEADER}"
    "${CLOUD_FRAG_HEADER}"
    "${TONEMAP_FRAG_HEADER}"
    "${CLOUD_COMPOSITE_FRAG_HEADER}"
    "${TAA_FRAG_HEADER}"
    "${FXAA_FRAG_HEADER}"
    "${SHADING_RATE_HEADER}"
    "${SHADOW_VERT_HEADER}"
    "${CONTACT_SHADOW_FRAG_HEADER}"
    "${RAY_SHADOW_FRAG_HEADER}"
    "${CLOUD_NOISE_VOLUME_HEADER}"
    "${CLOUD_NOISE_WEATHER_HEADER}"
    "${IBL_PREFILTER_HEADER}"
    "${IBL_IRRADIANCE_HEADER}"
    "${BRDF_LUT_HEADER}"
    "${SH_PROJECT_HEADER}"
    "${CLUSTER_BUILD_HEADER}"
    "${LIGHT_SHADOW_VERT_HEADER}"
    "${GTAO_COMP_HEADER}"
    "${GTAO_RESOLVE_FRAG_HEADER}"
    "${HIZ_COMP_HEADER}"
    "${SSR_FRAG_HEADER}"
    "${GROUND_SHADOW_RESOLVE_FRAG_HEADER}"
    "${TRANSMITTANCE_LUT_HEADER}"
    "${MULTISCATTER_LUT_HEADER}"
    "${SKY_VIEW_LUT_HEADER}"
    "${AERIAL_PERSPECTIVE_HEADER}"
    "${FOG_SCATTER_HEADER}"
    "${GI_PROBE_RELIGHT_HEADER}"
    "${SDF_POPULATE_HEADER}"
    "${SDF_PROBE_RELIGHT_HEADER}")
target_include_directories(sushi_render
    PUBLIC "${CMAKE_SOURCE_DIR}/include"
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}" "${SHADER_GEN_DIR}")
# Shader hot-reload watches this directory when it exists, which is the case for a
# development build from the source tree and not for a deployed binary.
target_compile_definitions(sushi_render PRIVATE
    SUSHI_SHADER_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/shaders")
target_link_libraries(sushi_render PUBLIC
    Vulkan::Headers
    Vulkan::Loader
    GPUOpen::VulkanMemoryAllocator
    vk-bootstrap::vk-bootstrap)
target_link_libraries(sushi_render PRIVATE
    glslang::glslang
    glslang::glslang-default-resource-limits
    glslang::SPIRV
    Threads::Threads)
# Both header-only libraries need exactly one translation unit to carry their
# implementation; material/stb_impl.cpp is that unit for both.
target_include_directories(sushi_render PRIVATE
    "${Stb_INCLUDE_DIR}"
    "${CGLTF_INCLUDE_DIR}")
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
