/**************************************************************************/
/* opaque_pass.cpp                                                        */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/* Licensed under the Apache License, Version 2.0 (the "License");        */
/* you may not use this file except in compliance with the License.       */
/* You may obtain a copy of the License at                                */
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

#include "passes/opaque_pass.hpp"

#include <cstddef>
#include <utility>
#include <vector>

#include <SushiEngine/render/cloth_mesh.hpp>

#include "frame/frame_context.hpp"
#include "geometry/cloth_buffers.hpp"
#include "geometry/mesh_registry.hpp"
#include "lighting/cluster_config.hpp"
#include "lighting/light_system.hpp"
#include "material/material_system.hpp"
#include "scene/motion_system.hpp"
#include "passes/ibl_pass.hpp"
#include "passes/irradiance_volume_pass.hpp"
#include "graph/render_graph.hpp"
#include "resources/descriptor_allocator.hpp"
#include "resources/pipeline_cache.hpp"
#include "resources/sampler_cache.hpp"
#include "resources/shader_library.hpp"
#include "scene/scene_layout.hpp"
#include "scene/scene_uniforms.hpp"
#include "passes/shadow_pass.hpp"
#include "scene/shadow_uniforms.hpp"
#include "scene/temporal_uniforms.hpp"
#include "textures/cloud_noise.hpp"
#include "rhi/vulkan/vulkan_device.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Passes
        {
            namespace
            {
                using Scene::MeshPushConstants;

                /** @brief The shader stages the mesh push constants are visible to. */
                constexpr VkShaderStageFlags PUSH_STAGES =
                    VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;

                /**
                 * @brief Fills a push constant from a transform, material, and pick ids.
                 *
                 * The world translation column of @p model has the camera @p eye subtracted
                 * from it in double before the float cast, so what reaches the GPU is the
                 * object's offset from the camera — a small number that keeps full float
                 * precision even at planetary range.
                 *
                 * @param model         Object-to-world transform, absolute.
                 * @param eye           Camera world position; pass zeros for geometry that
                 *                      is already expressed relative to the eye.
                 * @param material      The surface the instance shades with.
                 * @param entity_id     Picking id written to the id target.
                 * @param selected_id   The id currently highlighted.
                 * @param material_index Index into the frame's material array.
                 * @param motion_index  Index into the frame's previous-transform array.
                 * @param outline_shift Screen-space outline expansion, 0 for the lit pass.
                 * @return The filled push constant.
                 */
                MeshPushConstants make_push(const Mat4& model, const double eye[3],
                                            const Render::Material& material,
                                            std::uint32_t entity_id, std::uint32_t selected_id,
                                            std::uint32_t material_index,
                                            std::uint32_t motion_index,
                                            float outline_shift = 0.0f)
                {
                    MeshPushConstants push{};
                    for (int i = 0; i < 16; ++i)
                        push.model[i] = static_cast<float>(model.m[i]);
                    push.model[12] = static_cast<float>(model.m[12] - eye[0]);
                    push.model[13] = static_cast<float>(model.m[13] - eye[1]);
                    push.model[14] = static_cast<float>(model.m[14] - eye[2]);
                    push.albedo_metallic[0] = static_cast<float>(material.albedo.x);
                    push.albedo_metallic[1] = static_cast<float>(material.albedo.y);
                    push.albedo_metallic[2] = static_cast<float>(material.albedo.z);
                    push.albedo_metallic[3] = material.metallic;
                    push.emissive_roughness[0] = static_cast<float>(material.emissive.x);
                    push.emissive_roughness[1] = static_cast<float>(material.emissive.y);
                    push.emissive_roughness[2] = static_cast<float>(material.emissive.z);
                    push.emissive_roughness[3] = material.roughness;
                    push.outline_shift[0] = outline_shift;
                    push.outline_shift[1] = outline_shift;
                    push.entity_id = entity_id;
                    push.selected = selected_id;
                    push.material_index = material_index;
                    push.motion_index = motion_index;
                    return push;
                }

                /** @brief A flat, unlit material carrying just a colour. */
                Render::Material flat_material(const Vector3& color)
                {
                    Render::Material material;
                    material.albedo = color;
                    material.metallic = 0.0f;
                    material.roughness = 0.9f;
                    return material;
                }

                /** @brief The description shared by all three of this pass's pipelines. */
                Resources::GraphicsPipelineDesc base_desc(VkPipelineLayout layout)
                {
                    Resources::GraphicsPipelineDesc desc;
                    desc.layout = layout;
                    desc.vertex_stride = sizeof(Geometry::MeshVertex);
                    desc.attribute_count = 6;
                    desc.attributes[0] = {
                        0, VK_FORMAT_R32G32B32_SFLOAT,
                        static_cast<std::uint32_t>(offsetof(Geometry::MeshVertex, position))};
                    desc.attributes[1] = {
                        1, VK_FORMAT_R32G32B32_SFLOAT,
                        static_cast<std::uint32_t>(offsetof(Geometry::MeshVertex, normal))};
                    desc.attributes[2] = {
                        2, VK_FORMAT_R32G32B32A32_SFLOAT,
                        static_cast<std::uint32_t>(offsetof(Geometry::MeshVertex, tangent))};
                    desc.attributes[3] = {
                        3, VK_FORMAT_R32G32_SFLOAT,
                        static_cast<std::uint32_t>(offsetof(Geometry::MeshVertex, uv0))};
                    desc.attributes[4] = {
                        4, VK_FORMAT_R32G32_SFLOAT,
                        static_cast<std::uint32_t>(offsetof(Geometry::MeshVertex, uv1))};
                    // Normalised so the shader reads the packed byte colour as 0..1.
                    desc.attributes[5] = {
                        5, VK_FORMAT_R8G8B8A8_UNORM,
                        static_cast<std::uint32_t>(offsetof(Geometry::MeshVertex, color))};
                    desc.depth_test = VK_TRUE;
                    desc.depth_write = VK_TRUE;
                    desc.depth_compare = VK_COMPARE_OP_GREATER_OR_EQUAL; // reverse-Z
                    desc.stencil_test = VK_TRUE;
                    desc.stencil.compareOp = VK_COMPARE_OP_ALWAYS;
                    desc.stencil.passOp = VK_STENCIL_OP_REPLACE;
                    desc.stencil.failOp = VK_STENCIL_OP_KEEP;
                    desc.stencil.depthFailOp = VK_STENCIL_OP_KEEP;
                    desc.stencil.compareMask = 0xFF;
                    desc.stencil.writeMask = 0xFF;
                    desc.dynamic_stencil_reference = VK_TRUE;
                    desc.color_count = 4;
                    desc.color_formats[0] = Frame::HDR_FORMAT;
                    desc.color_formats[1] = Frame::ID_FORMAT;
                    desc.color_formats[2] = Frame::VELOCITY_FORMAT;
                    desc.color_formats[3] = Frame::GBUFFER_FORMAT;
                    desc.depth_format = Frame::DEPTH_FORMAT;
                    desc.stencil_format = Frame::DEPTH_FORMAT;
                    return desc;
                }
            } // namespace

            OpaquePass::OpaquePass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                                   Resources::GraphicsPipelineFactory& pipelines,
                                   Scene::SceneLayout& layout, Geometry::MeshRegistry& meshes,
                                   Geometry::ClothBuffers& cloth,
                                   Assets::MaterialSystem& materials, Scene::MotionSystem& motion,
                               Textures::CloudNoise& noise, IblPass& ibl,
                               IrradianceVolumePass& gi, Lighting::LightSystem& lights)
                : device_(device), shaders_(shaders), pipelines_(pipelines), layout_(layout),
                  meshes_(meshes), cloth_(cloth), materials_(materials), motion_(motion),
                  noise_(noise), ibl_(ibl), gi_(gi), lights_(lights)
            {
                create_pipelines();
            }

            OpaquePass::~OpaquePass() { destroy_pipelines(); }

            void OpaquePass::create_pipelines()
            {
                const VkShaderModule vertex = shaders_.module("mesh.vert");
                const VkShaderModule outline_vertex = shaders_.module("outline.vert");

                Resources::GraphicsPipelineDesc mesh = base_desc(layout_.pipeline_layout());
                mesh.vertex_shader = vertex;
                mesh.fragment_shader = shaders_.module("pbr.frag");
                mesh_pipeline_ = pipelines_.create(mesh);

                Resources::GraphicsPipelineDesc line = mesh;
                line.topology = VK_PRIMITIVE_TOPOLOGY_LINE_LIST;
                line.fragment_shader = shaders_.module("line.frag");
                line_pipeline_ = pipelines_.create(line);

                // The outline draws the selected shape as a thick wireframe, masked to the
                // texels the object did not already cover. Reverse-Z makes the bias that
                // pulls it in front of its object positive.
                Resources::GraphicsPipelineDesc outline = mesh;
                outline.vertex_shader = outline_vertex;
                outline.fragment_shader = shaders_.module("outline.frag");
                outline.polygon_mode = VK_POLYGON_MODE_LINE;
                outline.line_width = 8.0f;
                outline.depth_bias_enable = VK_TRUE;
                outline.depth_bias_constant = 2.0f;
                outline.depth_bias_slope = 2.0f;
                outline.depth_write = VK_FALSE;
                outline.stencil.compareOp = VK_COMPARE_OP_NOT_EQUAL;
                outline.stencil.passOp = VK_STENCIL_OP_KEEP;
                outline_pipeline_ = pipelines_.create(outline);
            }

            void OpaquePass::destroy_pipelines()
            {
                // The factory owns these pipelines and swaps in their optimized rebuilds;
                // the pass only drops its handles. clear_libraries() frees the pipelines.
                mesh_pipeline_ = Resources::PipelineHandle{};
                line_pipeline_ = Resources::PipelineHandle{};
                outline_pipeline_ = Resources::PipelineHandle{};
            }

            void OpaquePass::rebuild_pipelines()
            {
                destroy_pipelines();
                create_pipelines();
            }

            void OpaquePass::register_pass(Graph::RenderGraph& graph,
                                           const Frame::FrameContext& frame)
            {
                // Soft bodies are triangulated and uploaded now, before the graph runs, so
                // the execute function below only records draws. The upload targets this
                // frame slot's own buffers, which the GPU is guaranteed to be done with.
                const Geometry::Mesh* cloth = nullptr;
                std::vector<std::pair<std::uint32_t, std::uint32_t>> cloth_ranges;
                if (frame.draws.strand_count > 0)
                {
                    std::vector<ClothVertex> triangulated_vertices;
                    std::vector<std::uint32_t> triangulated_indices;
                    std::vector<Geometry::MeshVertex> vertices;
                    std::vector<std::uint32_t> indices;
                    cloth_ranges.reserve(frame.draws.strand_count);

                    for (std::size_t s = 0; s < frame.draws.strand_count; ++s)
                    {
                        const ClothStrandView& strand = frame.draws.strands[s];
                        triangulate_cloth_grid(strand.vertices, strand.rows, strand.cols,
                                               triangulated_vertices, triangulated_indices);
                        if (triangulated_indices.empty())
                            continue;

                        const std::uint32_t base_vertex =
                            static_cast<std::uint32_t>(vertices.size());
                        // Cloth points arrive as absolute world positions, so they are made
                        // camera-relative here, in double before the float cast, exactly as
                        // make_push does for a model translation.
                        for (const ClothVertex& vertex : triangulated_vertices)
                        {
                            Geometry::MeshVertex out{};
                            out.position[0] = static_cast<float>(vertex.position.x - frame.eye[0]);
                            out.position[1] = static_cast<float>(vertex.position.y - frame.eye[1]);
                            out.position[2] = static_cast<float>(vertex.position.z - frame.eye[2]);
                            out.normal[0] = static_cast<float>(vertex.normal.x);
                            out.normal[1] = static_cast<float>(vertex.normal.y);
                            out.normal[2] = static_cast<float>(vertex.normal.z);
                            for (int channel = 0; channel < 4; ++channel)
                                out.color[channel] = 255;
                            vertices.push_back(out);
                        }

                        const std::uint32_t index_offset = static_cast<std::uint32_t>(indices.size());
                        for (std::uint32_t index : triangulated_indices)
                            indices.push_back(base_vertex + index);
                        cloth_ranges.emplace_back(
                            index_offset, static_cast<std::uint32_t>(triangulated_indices.size()));
                    }

                    // Soft bodies carry no authored UVs, so no tangent frame can be derived
                    // from them; their zero tangent tells the shader to fall back to
                    // screen-space derivatives.
                    if (!indices.empty())
                        cloth = &cloth_.upload(frame.slot, vertices.data(), vertices.size(),
                                               indices.data(), indices.size());
                }

                // The quality tier decides which advanced BRDF lobes are evaluated at all;
                // apply that once here, before any material is packed, so a lower tier
                // strips clearcoat/sheen/anisotropy/transmission across the whole frame and
                // the shader's lobe branches are simply never taken.
                std::uint32_t allowed_lobes = 0;
                if (frame.quality.lobe_anisotropy)
                    allowed_lobes |= Assets::MATERIAL_ANISOTROPY;
                if (frame.quality.lobe_clearcoat)
                    allowed_lobes |= Assets::MATERIAL_CLEARCOAT;
                if (frame.quality.lobe_sheen)
                    allowed_lobes |= Assets::MATERIAL_SHEEN;
                if (frame.quality.lobe_transmission)
                    allowed_lobes |= Assets::MATERIAL_TRANSMISSION;
                materials_.set_allowed_lobes(allowed_lobes);

                // Pack every material this frame draws with before the graph runs, so the
                // execute function records draws only and the array is already complete
                // when the descriptor set is written. A draw carries this array's index.
                std::vector<std::uint32_t> instance_materials;
                instance_materials.reserve(frame.draws.instance_count);
                for (std::size_t i = 0; i < frame.draws.instance_count; ++i)
                    instance_materials.push_back(
                        materials_.push(frame.draws.instances[i].material));

                std::vector<std::uint32_t> strand_materials;
                strand_materials.reserve(frame.draws.strand_count);
                for (std::size_t s = 0; s < frame.draws.strand_count; ++s)
                    strand_materials.push_back(
                        materials_.push(flat_material(frame.draws.strands[s].color)));

                // Where each of those objects was last frame, packed the same way and
                // for the same reason. The transform pushed here is the one the draw
                // actually uses, primitive scaling included, so the two clip positions
                // the vertex shader differences describe the same geometry.
                const std::uint32_t grid_motion = motion_.push(NO_PICK, Mat4{});

                std::vector<std::uint32_t> instance_motions;
                instance_motions.reserve(frame.draws.instance_count);
                for (std::size_t i = 0; i < frame.draws.instance_count; ++i)
                {
                    const MeshInstance& instance = frame.draws.instances[i];
                    const Mat4 model =
                        instance.mesh != INVALID_MESH
                            ? instance.model
                            : mul(instance.model, Geometry::shape_scale(instance.kind,
                                                                        instance.shape_params));
                    instance_motions.push_back(motion_.push(instance.id, model));
                }

                std::vector<std::uint32_t> strand_motions;
                strand_motions.reserve(frame.draws.strand_count);
                for (std::size_t s = 0; s < frame.draws.strand_count; ++s)
                    strand_motions.push_back(motion_.push_camera_relative());

                Graph::ClearColor scene_clear;
                Graph::ClearColor id_clear;
                id_clear.integer = true;
                id_clear.uint32[0] = NO_PICK;
                // Zero motion for texels no draw covers; the temporal resolve replaces
                // it with a view-ray reprojection wherever the depth says nothing is
                // there, so this value is only ever read as "no geometry moved here".
                Graph::ClearColor velocity_clear;
                // Where no surface draws, the reflection G-buffer reads fully rough so the
                // SSR trace never reflects off the cleared background.
                Graph::ClearColor gbuffer_clear;
                gbuffer_clear.float32[0] = 1.0f;  // roughness
                gbuffer_clear.float32[1] = 0.04f; // dielectric F0

                graph.add_pass(
                    "opaque",
                    [&](Graph::RenderPassBuilder& builder)
                    {
                        builder.color_attachment(0, frame.targets.hdr, Graph::AttachmentLoad::Clear,
                                                 scene_clear);
                        builder.color_attachment(1, frame.targets.id, Graph::AttachmentLoad::Clear,
                                                 id_clear);
                        builder.color_attachment(2, frame.targets.velocity,
                                                 Graph::AttachmentLoad::Clear, velocity_clear);
                        builder.color_attachment(3, frame.targets.gbuffer,
                                                 Graph::AttachmentLoad::Clear, gbuffer_clear);
                        // The prepass already filled this, so it is loaded rather than
                        // cleared; the depths written here are recomputed by the same
                        // vertex shader and therefore identical, and what the test buys
                        // is rejecting an occluded fragment before the material shader
                        // ever runs on it.
                        builder.depth_stencil_attachment(frame.targets.depth,
                                                         Graph::AttachmentLoad::Load, 0.0f, 0);
                        builder.read(frame.targets.uniforms, Graph::BufferAccess::UniformRead);
                        builder.read(frame.targets.temporal, Graph::BufferAccess::UniformRead);
                        builder.read(frame.targets.shadow, Graph::BufferAccess::UniformRead);
                        builder.read(frame.targets.shadow_atlas,
                                     Graph::TextureAccess::SampledFragment);
                        builder.read(frame.targets.contact_shadow,
                                     Graph::TextureAccess::SampledFragment);
                        builder.read(frame.targets.ray_shadow,
                                     Graph::TextureAccess::SampledFragment);
                        // The froxel grid the cull pass built: read here so the graph
                        // derives the compute→fragment barrier that makes the light lists
                        // visible before shading loops them.
                        builder.read(frame.targets.cluster_grid,
                                     Graph::BufferAccess::StorageRead);
                        builder.read(frame.targets.light_index,
                                     Graph::BufferAccess::StorageRead);
                        builder.read(frame.targets.light_shadow_atlas,
                                     Graph::TextureAccess::SampledFragment);
                        builder.read(frame.targets.decal_grid, Graph::BufferAccess::StorageRead);
                        builder.read(frame.targets.decal_index, Graph::BufferAccess::StorageRead);
                        builder.read(frame.targets.ao, Graph::TextureAccess::SampledFragment);
                    },
                    [this, &frame, cloth, cloth_ranges, instance_materials, strand_materials,
                     grid_motion, instance_motions,
                     strand_motions](VkCommandBuffer cmd, const Graph::PassContext& context)
                    {
                        Scene::SceneSetWriter writer;
                        writer.uniform(Scene::SceneLayout::SCENE_BINDING,
                                       context.buffer(frame.targets.uniforms),
                                       sizeof(Scene::SceneUniforms));
                        writer.image(1, ibl_.irradiance(), ibl_.sampler());
                        writer.image(2, ibl_.specular(), ibl_.sampler());
                        writer.image(3, ibl_.brdf_lut(), ibl_.sampler());
                        writer.image(Scene::SceneLayout::SHADOW_ATLAS_BINDING,
                                     context.sampled_view(frame.targets.shadow_atlas),
                                     ShadowPass::atlas_sampler(*frame.samplers));
                        writer.image(Scene::SceneLayout::SHADOW_DEPTH_BINDING,
                                     context.sampled_view(frame.targets.shadow_atlas),
                                     ShadowPass::atlas_depth_sampler(*frame.samplers));
                        writer.image(4, context.sampled_view(frame.targets.ray_shadow),
                                     frame.samplers->get(Resources::SamplerDesc{}));
                        writer.image(5, context.sampled_view(frame.targets.contact_shadow),
                                     frame.samplers->get(Resources::SamplerDesc{}));
                        writer.image(6, noise_.weather(), noise_.sampler());
                        writer.storage(Scene::SceneLayout::MATERIAL_BINDING, materials_.buffer(),
                                       materials_.buffer_range());
                        writer.storage(Scene::SceneLayout::MOTION_BINDING, motion_.buffer(),
                                       motion_.buffer_range());
                        writer.uniform(Scene::SceneLayout::TEMPORAL_BINDING,
                                       context.buffer(frame.targets.temporal),
                                       sizeof(Scene::TemporalUniforms));
                        writer.uniform(Scene::SceneLayout::SHADOW_BINDING,
                                       context.buffer(frame.targets.shadow),
                                       sizeof(Scene::ShadowUniforms));
                        writer.storage(Scene::SceneLayout::IBL_SH_BINDING, ibl_.sh_buffer(),
                                       IblPass::sh_buffer_bytes());
                        // The clustered light engine's four bindings: the light array and
                        // config block are host-written and bound directly (like the
                        // material array); the count grid and index list are the graph
                        // transients the cull pass wrote this frame.
                        writer.storage(Scene::SceneLayout::LIGHT_BINDING, lights_.light_buffer(),
                                       lights_.light_buffer_range());
                        writer.storage(Scene::SceneLayout::CLUSTER_GRID_BINDING,
                                       context.buffer(frame.targets.cluster_grid),
                                       Lighting::CLUSTER_COUNT * sizeof(std::uint32_t));
                        writer.storage(Scene::SceneLayout::LIGHT_INDEX_BINDING,
                                       context.buffer(frame.targets.light_index),
                                       Lighting::LIGHT_INDEX_COUNT * sizeof(std::uint32_t));
                        writer.uniform(Scene::SceneLayout::CLUSTER_CONFIG_BINDING,
                                       lights_.config_buffer(), lights_.config_buffer_range());
                        // Punctual spot shadows: the atlas through the same comparison
                        // sampler the sun cascades use, and the per-caster matrix buffer.
                        writer.image(Scene::SceneLayout::LIGHT_SHADOW_ATLAS_BINDING,
                                     context.sampled_view(frame.targets.light_shadow_atlas),
                                     ShadowPass::atlas_sampler(*frame.samplers));
                        writer.storage(Scene::SceneLayout::LIGHT_SHADOW_DATA_BINDING,
                                       lights_.shadow_buffer(), lights_.shadow_buffer_range());
                        // Clustered decals: the decal array (host-written, bound directly)
                        // and the count grid + index list the cull pass wrote this frame.
                        writer.storage(Scene::SceneLayout::DECAL_BINDING, lights_.decal_buffer(),
                                       lights_.decal_buffer_range());
                        writer.storage(Scene::SceneLayout::DECAL_GRID_BINDING,
                                       context.buffer(frame.targets.decal_grid),
                                       Lighting::CLUSTER_COUNT * sizeof(std::uint32_t));
                        writer.storage(Scene::SceneLayout::DECAL_INDEX_BINDING,
                                       context.buffer(frame.targets.decal_index),
                                       Lighting::DECAL_INDEX_COUNT * sizeof(std::uint32_t));
                        // The resolved ambient occlusion the shading pass multiplies its
                        // indirect diffuse and specular by.
                        writer.image(Scene::SceneLayout::AO_BINDING,
                                     context.sampled_view(frame.targets.ao),
                                     frame.samplers->get(Resources::SamplerDesc{}));
                        // Probe-volume GI: the SH grid the shading pass gathers and the config
                        // block that locates a surface in it. Both are pass-owned resources
                        // the irradiance-volume pass barriered before this pass runs.
                        writer.storage(Scene::SceneLayout::GI_PROBE_SH_BINDING, gi_.probe_sh_buffer(),
                                       IrradianceVolumePass::probe_sh_bytes());
                        writer.uniform(Scene::SceneLayout::GI_PROBE_CONFIG_BINDING,
                                       gi_.config_buffer(frame.index),
                                       IrradianceVolumePass::config_bytes());
                        writer.commit(cmd, frame.layout->pipeline_layout());
                        frame.layout->bind_heap(cmd);

                        const VkPipelineLayout pipeline_layout = frame.layout->pipeline_layout();
                        const VkDeviceSize zero = 0;

                        // Ground grid: a single flat-coloured line draw.
                        const Geometry::Mesh& grid = meshes_.grid();
                        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, line_pipeline_.get());
                        vkCmdSetStencilReference(cmd, VK_STENCIL_FACE_FRONT_AND_BACK, 0);
                        vkCmdBindVertexBuffers(cmd, 0, 1, &grid.vertices, &zero);
                        const MeshPushConstants grid_push =
                            make_push(Mat4{}, frame.eye,
                                      flat_material(Vector3{0.32f, 0.33f, 0.40f}), NO_PICK,
                                      NO_PICK, 0, grid_motion);
                        vkCmdPushConstants(cmd, pipeline_layout, PUSH_STAGES, 0,
                                           sizeof(MeshPushConstants), &grid_push);
                        vkCmdDraw(cmd, grid.vertex_count, 1, 0, 0);

                        // Instances draw grouped by geometry so each mesh's buffers are
                        // bound once per group rather than once per instance. An instance
                        // with no imported mesh falls back to the primitive its kind names,
                        // which is why both live in one loop rather than two paths.
                        const auto draw_instances = [&](VkPipeline pipeline, bool outline)
                        {
                            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
                            if (outline)
                                vkCmdSetStencilReference(cmd, VK_STENCIL_FACE_FRONT_AND_BACK, 1);

                            VkBuffer bound_vertices = VK_NULL_HANDLE;
                            for (std::size_t i = 0; i < frame.draws.instance_count; ++i)
                            {
                                const MeshInstance& instance = frame.draws.instances[i];
                                if (outline && instance.id != frame.draws.selected_id)
                                    continue;

                                const bool imported = instance.mesh != INVALID_MESH;
                                const Geometry::Mesh& mesh = imported
                                                                 ? meshes_.mesh(instance.mesh)
                                                                 : meshes_.primitive(instance.kind);
                                if (mesh.index_count == 0)
                                    continue;
                                if (mesh.vertices != bound_vertices)
                                {
                                    vkCmdBindVertexBuffers(cmd, 0, 1, &mesh.vertices, &zero);
                                    vkCmdBindIndexBuffer(cmd, mesh.indices, 0,
                                                         VK_INDEX_TYPE_UINT32);
                                    bound_vertices = mesh.vertices;
                                }

                                // An imported mesh carries its own geometry and scale; only
                                // a primitive needs its unit mesh mapped onto shape params.
                                const Mat4 model =
                                    imported ? instance.model
                                             : mul(instance.model,
                                                   Geometry::shape_scale(instance.kind,
                                                                         instance.shape_params));
                                const MeshPushConstants push =
                                    outline ? make_push(model, frame.eye,
                                                        flat_material(instance.color), instance.id,
                                                        frame.draws.selected_id,
                                                        instance_materials[i],
                                                        instance_motions[i], 0.006f)
                                            : make_push(model, frame.eye, instance.material,
                                                        instance.id, frame.draws.selected_id,
                                                        instance_materials[i],
                                                        instance_motions[i]);
                                if (!outline)
                                    vkCmdSetStencilReference(
                                        cmd, VK_STENCIL_FACE_FRONT_AND_BACK,
                                        instance.id == frame.draws.selected_id ? 1 : 0);
                                vkCmdPushConstants(cmd, pipeline_layout, PUSH_STAGES, 0,
                                                   sizeof(MeshPushConstants), &push);
                                vkCmdDrawIndexed(cmd, mesh.index_count, 1, 0, 0, 0);
                            }
                        };

                        draw_instances(mesh_pipeline_.get(), false);
                        if (frame.draws.selected_id != NO_PICK)
                            draw_instances(outline_pipeline_.get(), true);

                        if (cloth == nullptr || cloth->index_count == 0)
                            return;

                        // Soft bodies draw with the same lit pipeline the primitives use
                        // (already double-sided), so they shade and pick like any other
                        // object rather than as a bare wireframe. Their vertices are
                        // already camera-relative, so the push carries no eye of its own.
                        const double no_eye[3] = {0.0, 0.0, 0.0};
                        const auto draw_cloth = [&](VkPipeline pipeline, bool outline)
                        {
                            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
                            vkCmdBindVertexBuffers(cmd, 0, 1, &cloth->vertices, &zero);
                            vkCmdBindIndexBuffer(cmd, cloth->indices, 0, VK_INDEX_TYPE_UINT32);
                            if (outline)
                                vkCmdSetStencilReference(cmd, VK_STENCIL_FACE_FRONT_AND_BACK, 1);
                            std::size_t range = 0;
                            for (std::size_t s = 0; s < frame.draws.strand_count; ++s)
                            {
                                const ClothStrandView& strand = frame.draws.strands[s];
                                if (strand.rows < 2 || strand.cols < 2)
                                    continue;
                                const std::uint32_t index_offset = cloth_ranges[range].first;
                                const std::uint32_t index_count = cloth_ranges[range].second;
                                ++range;
                                if (outline && strand.id != frame.draws.selected_id)
                                    continue;
                                if (!outline)
                                    vkCmdSetStencilReference(
                                        cmd, VK_STENCIL_FACE_FRONT_AND_BACK,
                                        strand.id == frame.draws.selected_id ? 1 : 0);
                                const MeshPushConstants push =
                                    make_push(Mat4{}, no_eye, flat_material(strand.color),
                                              strand.id, frame.draws.selected_id,
                                              strand_materials[s], strand_motions[s],
                                              outline ? 0.006f : 0.0f);
                                vkCmdPushConstants(cmd, pipeline_layout, PUSH_STAGES, 0,
                                                   sizeof(MeshPushConstants), &push);
                                vkCmdDrawIndexed(cmd, index_count, 1, index_offset, 0, 0);
                            }
                        };

                        draw_cloth(mesh_pipeline_.get(), false);
                        if (frame.draws.selected_id != NO_PICK)
                            draw_cloth(outline_pipeline_.get(), true);
                    });
            }
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
