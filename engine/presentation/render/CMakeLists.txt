# render — the greenfield Vulkan renderer behind the RHI seam (include/SushiEngine/render/rhi/).
# Consumers program against the abstract IRenderDevice; source/rhi/vulkan/ is the only
# implementation, and the only place Vulkan-specific device code lives. A plain compiled target:
# it links neither SushiRuntime nor SYCL, so it builds on a stock toolchain. The source tree is
# the render-graph stack: graph/ derives barriers and resource lifetimes, resources/ owns the
# caches and pools, passes/ holds one file per pass, and scene/ and geometry/ carry the shared
# scene data. Vulkan, VMA, vk-bootstrap and glslang come from vcpkg (declared in
# cli/sushistack.deps.toml, provisioned by `ss install`).

find_package(VulkanHeaders CONFIG REQUIRED)
find_package(VulkanLoader CONFIG REQUIRED)
find_package(VulkanMemoryAllocator CONFIG REQUIRED)
find_package(vk-bootstrap CONFIG REQUIRED)
# The same glslang tools/shader_compiler uses is linked in here too, where it backs shader
# hot-reload against the source tree during development.
find_package(glslang CONFIG REQUIRED)
# The pipeline factory runs a background optimizer thread; Threads links the platform's threading
# primitives (a no-op link on Windows, pthread on Linux).
find_package(Threads REQUIRED)
# stb and cgltf are header-only. stb ships a find module (reached through vcpkg's wrapper); cgltf
# ships nothing but the header, so it is located by the header itself — the documented vcpkg
# approach for a port with no CMake export.
find_package(Stb REQUIRED)
find_path(CGLTF_INCLUDE_DIR "cgltf.h" REQUIRED)

# GLSL in, a C++ header of embedded SPIR-V words out, so the renderer ships its shaders inside the
# binary and reads no .glsl at runtime. tools/shader_compiler is the host tool that does it.
include(Shaders)
sushiengine_configure_shaders("${CMAKE_CURRENT_SOURCE_DIR}/shaders")

sushiengine_compile_shader(vert "${CMAKE_CURRENT_SOURCE_DIR}/shaders/triangle.vert"
    triangle_vert_spv TRIANGLE_VERT_HEADER)
sushiengine_compile_shader(frag "${CMAKE_CURRENT_SOURCE_DIR}/shaders/triangle.frag"
    triangle_frag_spv TRIANGLE_FRAG_HEADER)
sushiengine_compile_shader(vert "${CMAKE_CURRENT_SOURCE_DIR}/shaders/mesh.vert"
    mesh_vert_spv MESH_VERT_HEADER)
sushiengine_compile_shader(vert "${CMAKE_CURRENT_SOURCE_DIR}/shaders/mesh_gpu.vert"
    mesh_gpu_vert_spv MESH_GPU_VERT_HEADER)
sushiengine_compile_shader(vert "${CMAKE_CURRENT_SOURCE_DIR}/shaders/mesh_skinned.vert"
    mesh_skinned_vert_spv MESH_SKINNED_VERT_HEADER)
sushiengine_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/skinning.comp"
    skinning_comp_spv SKINNING_COMP_HEADER)
sushiengine_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/particle_emit.comp"
    particle_emit_comp_spv PARTICLE_EMIT_COMP_HEADER)
sushiengine_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/particle_simulate.comp"
    particle_simulate_comp_spv PARTICLE_SIMULATE_COMP_HEADER)
sushiengine_compile_shader(vert "${CMAKE_CURRENT_SOURCE_DIR}/shaders/particle.vert"
    particle_vert_spv PARTICLE_VERT_HEADER)
sushiengine_compile_shader(frag "${CMAKE_CURRENT_SOURCE_DIR}/shaders/particle.frag"
    particle_frag_spv PARTICLE_FRAG_HEADER)
sushiengine_compile_shader(vert "${CMAKE_CURRENT_SOURCE_DIR}/shaders/particle_sorted.vert"
    particle_sorted_vert_spv PARTICLE_SORTED_VERT_HEADER)
sushiengine_compile_shader(vert "${CMAKE_CURRENT_SOURCE_DIR}/shaders/particle_billboard.vert"
    particle_billboard_vert_spv PARTICLE_BILLBOARD_VERT_HEADER)
sushiengine_compile_shader(vert "${CMAKE_CURRENT_SOURCE_DIR}/shaders/particle_ribbon.vert"
    particle_ribbon_vert_spv PARTICLE_RIBBON_VERT_HEADER)
sushiengine_compile_shader(vert "${CMAKE_CURRENT_SOURCE_DIR}/shaders/particle_mesh.vert"
    particle_mesh_vert_spv PARTICLE_MESH_VERT_HEADER)
sushiengine_compile_shader(frag "${CMAKE_CURRENT_SOURCE_DIR}/shaders/particle_mesh.frag"
    particle_mesh_frag_spv PARTICLE_MESH_FRAG_HEADER)
sushiengine_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/particle_sort.comp"
    particle_sort_comp_spv PARTICLE_SORT_COMP_HEADER)
sushiengine_compile_shader(frag "${CMAKE_CURRENT_SOURCE_DIR}/shaders/pbr.frag"
    pbr_frag_spv PBR_FRAG_HEADER)
sushiengine_compile_shader(frag "${CMAKE_CURRENT_SOURCE_DIR}/shaders/outline.frag"
    outline_frag_spv OUTLINE_FRAG_HEADER)
sushiengine_compile_shader(frag "${CMAKE_CURRENT_SOURCE_DIR}/shaders/grid.frag"
    grid_frag_spv GRID_FRAG_HEADER)
sushiengine_compile_shader(vert "${CMAKE_CURRENT_SOURCE_DIR}/shaders/outline.vert"
    outline_vert_spv OUTLINE_VERT_HEADER)
sushiengine_compile_shader(vert "${CMAKE_CURRENT_SOURCE_DIR}/shaders/fullscreen.vert"
    fullscreen_vert_spv FULLSCREEN_VERT_HEADER)
# Terrain draws through the shared pbr.frag, so it ships a vertex stage and no fragment
# one: its output signature is mesh.vert's exactly, which is what lets the existing
# shading path light it with no parallel code.
sushiengine_compile_shader(vert "${CMAKE_CURRENT_SOURCE_DIR}/shaders/terrain.vert"
    terrain_vert_spv TERRAIN_VERT_HEADER)
sushiengine_compile_shader(frag "${CMAKE_CURRENT_SOURCE_DIR}/shaders/sky.frag"
    sky_frag_spv SKY_FRAG_HEADER)
sushiengine_compile_shader(frag "${CMAKE_CURRENT_SOURCE_DIR}/shaders/cloud.frag"
    cloud_frag_spv CLOUD_FRAG_HEADER)
sushiengine_compile_shader(frag "${CMAKE_CURRENT_SOURCE_DIR}/shaders/tonemap.frag"
    tonemap_frag_spv TONEMAP_FRAG_HEADER)
sushiengine_compile_shader(vert "${CMAKE_CURRENT_SOURCE_DIR}/shaders/shadow.vert"
    shadow_vert_spv SHADOW_VERT_HEADER)
sushiengine_compile_shader(frag "${CMAKE_CURRENT_SOURCE_DIR}/shaders/contact_shadow.frag"
    contact_shadow_frag_spv CONTACT_SHADOW_FRAG_HEADER)
sushiengine_compile_shader(frag "${CMAKE_CURRENT_SOURCE_DIR}/shaders/ray_shadow.frag"
    ray_shadow_frag_spv RAY_SHADOW_FRAG_HEADER)
sushiengine_compile_shader(frag "${CMAKE_CURRENT_SOURCE_DIR}/shaders/cloud_composite.frag"
    cloud_composite_frag_spv CLOUD_COMPOSITE_FRAG_HEADER)
sushiengine_compile_shader(frag "${CMAKE_CURRENT_SOURCE_DIR}/shaders/taa.frag"
    taa_frag_spv TAA_FRAG_HEADER)
sushiengine_compile_shader(frag "${CMAKE_CURRENT_SOURCE_DIR}/shaders/fxaa.frag"
    fxaa_frag_spv FXAA_FRAG_HEADER)
sushiengine_compile_shader(vert "${CMAKE_CURRENT_SOURCE_DIR}/shaders/ui.vert"
    ui_vert_spv UI_VERT_HEADER)
sushiengine_compile_shader(frag "${CMAKE_CURRENT_SOURCE_DIR}/shaders/ui.frag"
    ui_frag_spv UI_FRAG_HEADER)
sushiengine_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/shading_rate.comp"
    shading_rate_comp_spv SHADING_RATE_HEADER)
sushiengine_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/cloud_noise_volume.comp"
    cloud_noise_volume_comp_spv CLOUD_NOISE_VOLUME_HEADER)
sushiengine_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/cloud_noise_weather.comp"
    cloud_noise_weather_comp_spv CLOUD_NOISE_WEATHER_HEADER)
sushiengine_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/cloud_noise_mip.comp"
    cloud_noise_mip_comp_spv CLOUD_NOISE_MIP_HEADER)
sushiengine_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/cloudscape_field.comp"
    cloudscape_field_comp_spv CLOUDSCAPE_FIELD_HEADER)
sushiengine_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/cloudscape_skip.comp"
    cloudscape_skip_comp_spv CLOUDSCAPE_SKIP_HEADER)
sushiengine_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/cloudscape_far_light.comp"
    cloudscape_far_light_comp_spv CLOUDSCAPE_FAR_LIGHT_HEADER)
sushiengine_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/atmosphere_shift.comp"
    atmosphere_shift_comp_spv ATMOSPHERE_SHIFT_HEADER)
sushiengine_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/atmosphere_advect_velocity.comp"
    atmosphere_advect_velocity_comp_spv ATMOSPHERE_ADVECT_VELOCITY_HEADER)
sushiengine_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/atmosphere_advect_scalars.comp"
    atmosphere_advect_scalars_comp_spv ATMOSPHERE_ADVECT_SCALARS_HEADER)
sushiengine_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/atmosphere_surface.comp"
    atmosphere_surface_comp_spv ATMOSPHERE_SURFACE_HEADER)
sushiengine_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/atmosphere_forces.comp"
    atmosphere_forces_comp_spv ATMOSPHERE_FORCES_HEADER)
sushiengine_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/atmosphere_divergence.comp"
    atmosphere_divergence_comp_spv ATMOSPHERE_DIVERGENCE_HEADER)
sushiengine_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/atmosphere_pressure.comp"
    atmosphere_pressure_comp_spv ATMOSPHERE_PRESSURE_HEADER)
sushiengine_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/atmosphere_project.comp"
    atmosphere_project_comp_spv ATMOSPHERE_PROJECT_HEADER)
sushiengine_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/atmosphere_microphysics.comp"
    atmosphere_microphysics_comp_spv ATMOSPHERE_MICROPHYSICS_HEADER)
sushiengine_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/atmosphere_extinction.comp"
    atmosphere_extinction_comp_spv ATMOSPHERE_EXTINCTION_HEADER)
sushiengine_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/atmosphere_readback.comp"
    atmosphere_readback_comp_spv ATMOSPHERE_READBACK_HEADER)
sushiengine_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/cloud_light_volume.comp"
    cloud_light_volume_comp_spv CLOUD_LIGHT_VOLUME_HEADER)
sushiengine_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/cloud_shadow_map.comp"
    cloud_shadow_map_comp_spv CLOUD_SHADOW_MAP_HEADER)
sushiengine_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/cloud_taa.comp"
    cloud_taa_comp_spv CLOUD_TAA_HEADER)
sushiengine_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/cloud_panorama.comp"
    cloud_panorama_comp_spv CLOUD_PANORAMA_HEADER)
sushiengine_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/ibl_prefilter.comp"
    ibl_prefilter_comp_spv IBL_PREFILTER_HEADER)
sushiengine_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/ibl_irradiance.comp"
    ibl_irradiance_comp_spv IBL_IRRADIANCE_HEADER)
sushiengine_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/brdf_lut.comp"
    brdf_lut_comp_spv BRDF_LUT_HEADER)
sushiengine_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/sh_project.comp"
    sh_project_comp_spv SH_PROJECT_HEADER)
sushiengine_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/cluster_build.comp"
    cluster_build_comp_spv CLUSTER_BUILD_HEADER)
sushiengine_compile_shader(vert "${CMAKE_CURRENT_SOURCE_DIR}/shaders/light_shadow.vert"
    light_shadow_vert_spv LIGHT_SHADOW_VERT_HEADER)
sushiengine_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/gtao.comp"
    gtao_comp_spv GTAO_COMP_HEADER)
sushiengine_compile_shader(frag "${CMAKE_CURRENT_SOURCE_DIR}/shaders/gtao_resolve.frag"
    gtao_resolve_frag_spv GTAO_RESOLVE_FRAG_HEADER)
sushiengine_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/hiz.comp"
    hiz_comp_spv HIZ_COMP_HEADER)
sushiengine_compile_shader(frag "${CMAKE_CURRENT_SOURCE_DIR}/shaders/ssr.frag"
    ssr_frag_spv SSR_FRAG_HEADER)
sushiengine_compile_shader(frag "${CMAKE_CURRENT_SOURCE_DIR}/shaders/ground_shadow_resolve.frag"
    ground_shadow_resolve_frag_spv GROUND_SHADOW_RESOLVE_FRAG_HEADER)
sushiengine_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/transmittance_lut.comp"
    transmittance_lut_comp_spv TRANSMITTANCE_LUT_HEADER)
sushiengine_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/multiscatter_lut.comp"
    multiscatter_lut_comp_spv MULTISCATTER_LUT_HEADER)
sushiengine_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/sky_view_lut.comp"
    sky_view_lut_comp_spv SKY_VIEW_LUT_HEADER)
sushiengine_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/aerial_perspective.comp"
    aerial_perspective_comp_spv AERIAL_PERSPECTIVE_HEADER)
sushiengine_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/fog_scatter.comp"
    fog_scatter_comp_spv FOG_SCATTER_HEADER)
sushiengine_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/gi_probe_relight.comp"
    gi_probe_relight_comp_spv GI_PROBE_RELIGHT_HEADER)
sushiengine_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/sdf_populate.comp"
    sdf_populate_comp_spv SDF_POPULATE_HEADER)
sushiengine_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/sdf_probe_relight.comp"
    sdf_probe_relight_comp_spv SDF_PROBE_RELIGHT_HEADER)
sushiengine_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/bloom_down.comp"
    bloom_down_comp_spv BLOOM_DOWN_HEADER)
sushiengine_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/bloom_up.comp"
    bloom_up_comp_spv BLOOM_UP_HEADER)
sushiengine_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/luminance_histogram.comp"
    luminance_histogram_comp_spv LUMINANCE_HISTOGRAM_HEADER)
sushiengine_compile_shader(frag "${CMAKE_CURRENT_SOURCE_DIR}/shaders/dof.frag"
    dof_frag_spv DOF_HEADER)
sushiengine_compile_shader(frag "${CMAKE_CURRENT_SOURCE_DIR}/shaders/motion_blur.frag"
    motion_blur_frag_spv MOTION_BLUR_HEADER)
sushiengine_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/cull.comp"
    cull_comp_spv CULL_HEADER)
sushiengine_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/occlusion.comp"
    occlusion_comp_spv OCCLUSION_HEADER)
sushiengine_compile_shader(comp "${CMAKE_CURRENT_SOURCE_DIR}/shaders/deformable.comp"
    deformable_comp_spv DEFORMABLE_HEADER)
sushiengine_compile_shader(task "${CMAKE_CURRENT_SOURCE_DIR}/shaders/meshlet.task"
    meshlet_task_spv MESHLET_TASK_HEADER)
sushiengine_compile_shader(mesh "${CMAKE_CURRENT_SOURCE_DIR}/shaders/meshlet.mesh"
    meshlet_mesh_spv MESHLET_MESH_HEADER)

sushiengine_add_module(NAME render LAYER presentation
    SOURCES
        source/shader_catalogue.cpp
        source/graph/render_graph.cpp
        source/graph/resource_state.cpp
        source/graph/gpu_profiler.cpp
        source/graph/pass_capture.cpp
        source/resources/transient_pool.cpp
        source/resources/descriptor_allocator.cpp
        source/resources/descriptor_heap.cpp
        source/resources/pipeline_cache.cpp
        source/resources/sampler_cache.cpp
        source/resources/shader_library.cpp
        source/scene/scene_layout.cpp
        source/scene/scene_uniforms.cpp
        source/scene/post_process_uniforms.cpp
        source/scene/temporal_uniforms.cpp
        source/scene/motion_system.cpp
        source/scene/instance_system.cpp
        source/scene/skinning_system.cpp
        source/scene/particle_system.cpp
        source/scene/shadow_uniforms.cpp
        source/lighting/light_system.cpp
        source/raytracing/scene_accelerator.cpp
        source/frame/temporal_jitter.cpp
        source/frame/resolution_controller.cpp
        source/frame/upscaler.cpp
        source/frame/quality.cpp
        source/geometry/mesh_registry.cpp
        source/geometry/deformable_buffers.cpp
        source/geometry/ui_buffers.cpp
        source/material/asset_library.cpp
        source/material/font_atlas.cpp
        source/material/texture_library.cpp
        source/material/material_system.cpp
        source/material/gltf_importer.cpp
        source/textures/cloud_noise.cpp
        source/passes/fullscreen.cpp
        source/passes/ibl_pass.cpp
        source/passes/atmosphere_lut_pass.cpp
        source/passes/volumetric_fog_pass.cpp
        source/passes/irradiance_volume_pass.cpp
        source/gi/probe_volume.cpp
        source/gi/environment_probe_tracer.cpp
        source/gi/sdf_clipmap.cpp
        source/gi/sdf_probe_tracer.cpp
        source/gi/mesh_sdf_baker.cpp
        source/passes/opaque_pass.cpp
        source/passes/transparent_pass.cpp
        source/passes/gtao_pass.cpp
        source/passes/hiz_pass.cpp
        source/passes/occlusion_pass.cpp
        source/passes/cull_pass.cpp
        source/passes/deformable_pass.cpp
        source/passes/skinning_pass.cpp
        source/passes/particle_sim_pass.cpp
        source/passes/particle_sort_pass.cpp
        source/passes/particle_pass.cpp
        source/passes/particle_mesh_pass.cpp
        source/passes/ssr_pass.cpp
        source/passes/light_cull_pass.cpp
        source/passes/light_shadow_pass.cpp
        source/passes/sky_pass.cpp
        source/passes/ground_shadow_resolve_pass.cpp
        source/passes/weather_field_pass.cpp
        source/passes/cloudscape_compile_pass.cpp
        source/atmosphere/atmosphere_nest.cpp
        source/passes/cloud_light_volume_pass.cpp
        source/passes/cloud_shadow_map_pass.cpp
        source/passes/cloud_panorama_pass.cpp
        source/passes/cloud_pass.cpp
        source/passes/cloud_taa_pass.cpp
        source/passes/depth_only.cpp
        source/passes/depth_prepass.cpp
        source/passes/shadow_pass.cpp
        source/passes/contact_shadow_pass.cpp
        source/passes/ray_traced_shadow_pass.cpp
        source/passes/cloud_composite_pass.cpp
        source/passes/taa_pass.cpp
        source/passes/dof_pass.cpp
        source/passes/motion_blur_pass.cpp
        source/passes/auto_exposure_pass.cpp
        source/passes/bloom_pass.cpp
        source/passes/grid_pass.cpp
        source/passes/fxaa_pass.cpp
        source/passes/ui_pass.cpp
        source/passes/shading_rate_pass.cpp
        source/passes/tonemap_pass.cpp
        source/passes/picking_pass.cpp
        source/interop/vulkan_interop_buffer.cpp
        source/rhi/vulkan/vulkan_device.cpp
        source/rhi/vulkan/vulkan_offscreen.cpp
        source/rhi/vulkan/vulkan_window_renderer.cpp
        source/rhi/vulkan/vulkan_scene_view.cpp
        source/rhi/vulkan/view_resources.cpp
        source/rhi/vulkan/vma_impl.cpp
        source/material/stb_impl.cpp
        # Planetary terrain: set 2 and the height slot pool. The bookkeeping these wrap --
        # selection, residency, the inheritance rectangle -- is neutral header code in the
        # terrain module and is tested without a device.
        source/terrain/terrain_layout.cpp
        source/terrain/tile_cache.cpp
        source/terrain/planet_terrain.cpp
        source/passes/shading_set.cpp
        source/passes/terrain_pass.cpp
    # Public because each is spelled in a header this target publishes: asset_library_interface.hpp
    # hands back the material a loaded primitive comes as and answers with the atmosphere mirror,
    # scene_view.hpp carries the environment, the punctual lights and a UI draw list,
    # planet_terrain.hpp a terrain quadtree and layer stack, and particle_system.hpp the compiled
    # emitter table.
    PUBLIC_DEPENDS core ecs material environment ui terrain vfx geometry gltf
    # The glTF importer fills in skeleton and skin-vertex blobs; no header here exposes them.
    PRIVATE_DEPENDS animation
    EXTERNAL_PUBLIC
        Vulkan::Headers
        Vulkan::Loader
        GPUOpen::VulkanMemoryAllocator
        vk-bootstrap::vk-bootstrap
    EXTERNAL_PRIVATE
        glslang::glslang
        glslang::glslang-default-resource-limits
        glslang::SPIRV
        Threads::Threads)

# The generated SPIR-V headers, listed apart from SOURCES because sushiengine_add_module() reads
# those as paths relative to the module directory and these are absolute paths in the build tree.
# Listing them is what puts each shader's compile rule in this target's build graph.
target_sources(sushiengine_render PRIVATE
    "${TRIANGLE_VERT_HEADER}"
    "${TRIANGLE_FRAG_HEADER}"
    "${TERRAIN_VERT_HEADER}"
    "${MESH_VERT_HEADER}"
    "${MESH_GPU_VERT_HEADER}"
    "${MESH_SKINNED_VERT_HEADER}"
    "${SKINNING_COMP_HEADER}"
    "${PARTICLE_EMIT_COMP_HEADER}"
    "${PARTICLE_SIMULATE_COMP_HEADER}"
    "${PARTICLE_VERT_HEADER}"
    "${PARTICLE_FRAG_HEADER}"
    "${PARTICLE_SORTED_VERT_HEADER}"
    "${PARTICLE_BILLBOARD_VERT_HEADER}"
    "${PARTICLE_RIBBON_VERT_HEADER}"
    "${PARTICLE_MESH_VERT_HEADER}"
    "${PARTICLE_MESH_FRAG_HEADER}"
    "${PARTICLE_SORT_COMP_HEADER}"
    "${PBR_FRAG_HEADER}"
    "${OUTLINE_FRAG_HEADER}"
    "${GRID_FRAG_HEADER}"
    "${OUTLINE_VERT_HEADER}"
    "${FULLSCREEN_VERT_HEADER}"
    "${SKY_FRAG_HEADER}"
    "${CLOUD_FRAG_HEADER}"
    "${TONEMAP_FRAG_HEADER}"
    "${CLOUD_COMPOSITE_FRAG_HEADER}"
    "${TAA_FRAG_HEADER}"
    "${FXAA_FRAG_HEADER}"
    "${UI_VERT_HEADER}"
    "${UI_FRAG_HEADER}"
    "${SHADING_RATE_HEADER}"
    "${SHADOW_VERT_HEADER}"
    "${CONTACT_SHADOW_FRAG_HEADER}"
    "${RAY_SHADOW_FRAG_HEADER}"
    "${CLOUD_NOISE_VOLUME_HEADER}"
    "${CLOUD_NOISE_WEATHER_HEADER}"
    "${CLOUD_NOISE_MIP_HEADER}"
    "${CLOUDSCAPE_FIELD_HEADER}"
    "${CLOUDSCAPE_SKIP_HEADER}"
    "${CLOUDSCAPE_FAR_LIGHT_HEADER}"
    "${ATMOSPHERE_SHIFT_HEADER}"
    "${ATMOSPHERE_ADVECT_VELOCITY_HEADER}"
    "${ATMOSPHERE_ADVECT_SCALARS_HEADER}"
    "${ATMOSPHERE_SURFACE_HEADER}"
    "${ATMOSPHERE_FORCES_HEADER}"
    "${ATMOSPHERE_DIVERGENCE_HEADER}"
    "${ATMOSPHERE_PRESSURE_HEADER}"
    "${ATMOSPHERE_PROJECT_HEADER}"
    "${ATMOSPHERE_MICROPHYSICS_HEADER}"
    "${ATMOSPHERE_EXTINCTION_HEADER}"
    "${ATMOSPHERE_READBACK_HEADER}"
    "${CLOUD_LIGHT_VOLUME_HEADER}"
    "${CLOUD_SHADOW_MAP_HEADER}"
    "${CLOUD_TAA_HEADER}"
    "${CLOUD_PANORAMA_HEADER}"
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
    "${SDF_PROBE_RELIGHT_HEADER}"
    "${BLOOM_DOWN_HEADER}"
    "${BLOOM_UP_HEADER}"
    "${LUMINANCE_HISTOGRAM_HEADER}"
    "${DOF_HEADER}"
    "${MOTION_BLUR_HEADER}"
    "${CULL_HEADER}"
    "${OCCLUSION_HEADER}"
    "${DEFORMABLE_HEADER}"
    "${MESHLET_TASK_HEADER}"
    "${MESHLET_MESH_HEADER}")

target_include_directories(sushiengine_render PRIVATE
    "${SUSHIENGINE_SHADER_GENERATED_DIRECTORY}"
    # Both header-only libraries need exactly one translation unit to carry their implementation;
    # source/material/stb_impl.cpp is that unit for both.
    "${Stb_INCLUDE_DIR}"
    "${CGLTF_INCLUDE_DIR}")

# Shader hot-reload watches this directory when it exists, which is the case for a development
# build from the source tree and not for a deployed binary.
target_compile_definitions(sushiengine_render PRIVATE
    SUSHIENGINE_SHADER_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/shaders"
    SUSHIENGINE_PIPELINE_CACHE_DIR="${CMAKE_BINARY_DIR}")
