/**************************************************************************/
/* transparent_pass.cpp                                                  */
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

#include "passes/transparent_pass.hpp"

#include "passes/shading_set.hpp"

#include <algorithm>
#include <cstddef>
#include <vector>

#include "frame/frame_context.hpp"
#include "geometry/mesh_registry.hpp"
#include "lighting/cluster_config.hpp"
#include "lighting/light_system.hpp"
#include "material/material_system.hpp"
#include "scene/motion_system.hpp"
#include "scene/skinning_system.hpp"
#include "passes/cloud_shadow_map_pass.hpp"
#include "passes/ibl_pass.hpp"
#include "passes/irradiance_volume_pass.hpp"
#include "graph/render_graph.hpp"
#include "resources/descriptor_allocator.hpp"
#include "resources/descriptor_writer.hpp"
#include "resources/pipeline_cache.hpp"
#include "resources/sampler_cache.hpp"
#include "resources/shader_library.hpp"
#include "scene/scene_layout.hpp"
#include "scene/scene_uniforms.hpp"
#include "passes/shadow_pass.hpp"
#include "scene/shadow_uniforms.hpp"
#include "scene/temporal_uniforms.hpp"
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
                 * from it in double before the float cast, matching OpaquePass's make_push, so
                 * the same pbr.frag reads consistent camera-relative geometry from both passes.
                 *
                 * @param model         Object-to-world transform, absolute.
                 * @param eye           Camera world position.
                 * @param material      The surface the instance shades with.
                 * @param entity_id     Picking id written to the id target.
                 * @param selected_id   The id currently highlighted.
                 * @param material_index Index into the frame's material array.
                 * @param motion_index  Index into the frame's previous-transform array.
                 * @return The filled push constant.
                 */
                MeshPushConstants make_push(const Matrix4& model, const double eye[3],
                                            const Render::Material& material,
                                            std::uint32_t entity_id, std::uint32_t selected_id,
                                            std::uint32_t material_index,
                                            std::uint32_t motion_index)
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
                    push.outline_shift[0] = 0.0f;
                    push.outline_shift[1] = 0.0f;
                    push.entity_id = entity_id;
                    push.selected = selected_id;
                    push.material_index = material_index;
                    push.motion_index = motion_index;
                    return push;
                }

                /** @brief The description shared by this pass's pipeline. */
                Resources::GraphicsPipelineDescription base_desc(VkPipelineLayout layout)
                {
                    Resources::GraphicsPipelineDescription desc;
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
                    desc.attributes[5] = {
                        5, VK_FORMAT_R8G8B8A8_UNORM,
                        static_cast<std::uint32_t>(offsetof(Geometry::MeshVertex, color))};
                    desc.depth_test = VK_TRUE;
                    desc.depth_write = VK_FALSE;
                    desc.depth_compare = VK_COMPARE_OP_GREATER_OR_EQUAL; // reverse-Z
                    desc.color_count = 4;
                    desc.color_formats[0] = Frame::HDR_FORMAT;
                    desc.color_formats[1] = Frame::ID_FORMAT;
                    desc.color_formats[2] = Frame::VELOCITY_FORMAT;
                    desc.color_formats[3] = Frame::GBUFFER_FORMAT;
                    desc.depth_format = Frame::DEPTH_FORMAT;
                    desc.stencil_format = Frame::DEPTH_FORMAT;
                    desc.blend.enable = VK_TRUE;
                    // Only the HDR colour attachment blends; entity-ID, velocity, and G-buffer
                    // carry raw values (an alpha-weighted ID or velocity is meaningless) and
                    // ID_FORMAT in particular is an integer format the blend feature bit does
                    // not even cover.
                    desc.blend_mask = 0x1u;
                    desc.blend.src_color = VK_BLEND_FACTOR_SRC_ALPHA;
                    desc.blend.dst_color = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
                    desc.blend.color_op = VK_BLEND_OP_ADD;
                    desc.blend.src_alpha = VK_BLEND_FACTOR_ONE;
                    desc.blend.dst_alpha = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
                    desc.blend.alpha_op = VK_BLEND_OP_ADD;
                    return desc;
                }

                /** @brief Which kind of geometry a sorted transparent draw item names. */
                enum class ItemKind
                {
                    Rigid,
                    Skinned,
                };

                /** @brief One transparent draw, sorted alongside every other kind. */
                struct DrawItem
                {
                    ItemKind kind;
                    std::size_t index; /**< Index into frame.draws.instances or skinning ranges. */
                    double distance_squared;
                };
            } // namespace

            TransparentPass::TransparentPass(Vulkan::VulkanDevice& device,
                                             Resources::ShaderLibrary& shaders,
                                             Resources::GraphicsPipelineFactory& pipelines,
                                             Scene::SceneLayout& layout,
                                             Geometry::MeshRegistry& meshes,
                                             Assets::MaterialSystem& materials,
                                             Scene::MotionSystem& motion,
                                             Scene::SkinningSystem& skinning,
                                             CloudShadowMapPass& cloud_shadow, IBLPass& ibl,
                                             IrradianceVolumePass& gi, Lighting::LightSystem& lights)
                : device_(device), shaders_(shaders), pipelines_(pipelines), layout_(layout),
                  meshes_(meshes), materials_(materials), motion_(motion), skinning_(skinning),
                  cloud_shadow_(cloud_shadow), ibl_(ibl), gi_(gi), lights_(lights)
            {
                create_pipelines();
            }

            TransparentPass::~TransparentPass() { destroy_pipelines(); }

            void TransparentPass::create_pipelines()
            {
                Resources::GraphicsPipelineDescription mesh = base_desc(layout_.pipeline_layout());
                mesh.vertex_shader = shaders_.module("mesh.vert");
                mesh.fragment_shader = shaders_.module("pbr.frag");
                mesh_pipeline_ = pipelines_.create(mesh);

                // Matches OpaquePass's skinned pipeline: the SkinningPass output is the
                // MeshVertex layout plus a previous-frame position at offset 60, stride 72.
                Resources::GraphicsPipelineDescription skinned =
                    base_desc(layout_.pipeline_layout());
                skinned.vertex_stride = static_cast<std::uint32_t>(Scene::SKINNED_VERTEX_SIZE);
                skinned.attribute_count = 7;
                skinned.attributes[6] = {6, VK_FORMAT_R32G32B32_SFLOAT, 60};
                skinned.vertex_shader = shaders_.module("mesh_skinned.vert");
                skinned.fragment_shader = shaders_.module("pbr.frag");
                skinned_pipeline_ = pipelines_.create(skinned);
            }

            void TransparentPass::destroy_pipelines()
            {
                mesh_pipeline_ = Resources::PipelineHandle{};
                skinned_pipeline_ = Resources::PipelineHandle{};
            }

            void TransparentPass::rebuild_pipelines()
            {
                destroy_pipelines();
                create_pipelines();
            }

            void TransparentPass::register_pass(Graph::RenderGraph& graph,
                                                const Frame::FrameContext& frame)
            {
                // Gather the frame's transparent geometry — rigid instances and skinned
                // character slices alike — and pack their material and previous transform
                // now, before the graph runs, matching OpaquePass's pattern so the execute
                // function below only records draws.
                const std::vector<Scene::SkinnedRange>& skinned_ranges = skinning_.ranges();

                std::vector<DrawItem> order;
                order.reserve(frame.draws.instance_count + skinned_ranges.size());
                for (std::size_t i = 0; i < frame.draws.instance_count; ++i)
                {
                    const MeshInstance& instance = frame.draws.instances[i];
                    if (!is_alpha_blended(instance.material.surface_type))
                        continue;
                    const double dx = instance.model.m[12] - frame.eye[0];
                    const double dy = instance.model.m[13] - frame.eye[1];
                    const double dz = instance.model.m[14] - frame.eye[2];
                    order.push_back({ItemKind::Rigid, i, dx * dx + dy * dy + dz * dz});
                }
                for (std::size_t i = 0; i < skinned_ranges.size(); ++i)
                {
                    const Scene::SkinnedRange& range = skinned_ranges[i];
                    if (!is_alpha_blended(range.material.surface_type))
                        continue;
                    const double dx = range.model.m[12] - frame.eye[0];
                    const double dy = range.model.m[13] - frame.eye[1];
                    const double dz = range.model.m[14] - frame.eye[2];
                    order.push_back({ItemKind::Skinned, i, dx * dx + dy * dy + dz * dz});
                }

                if (order.empty())
                    return;

                std::vector<std::uint32_t> instance_materials(frame.draws.instance_count, 0);
                std::vector<std::uint32_t> instance_motions(frame.draws.instance_count, 0);
                std::vector<std::uint32_t> skinned_materials(skinned_ranges.size(), 0);
                std::vector<std::uint32_t> skinned_motions(skinned_ranges.size(), 0);
                for (const DrawItem& item : order)
                {
                    if (item.kind == ItemKind::Rigid)
                    {
                        const MeshInstance& instance = frame.draws.instances[item.index];
                        instance_materials[item.index] = materials_.push(instance.material);
                        const Matrix4 model =
                            instance.mesh != INVALID_MESH
                                ? instance.model
                                : mul(instance.model, Geometry::shape_scale(
                                                           instance.kind, instance.shape_params));
                        instance_motions[item.index] = motion_.push(instance.id, model);
                    }
                    else
                    {
                        const Scene::SkinnedRange& range = skinned_ranges[item.index];
                        skinned_materials[item.index] = materials_.push(range.material);
                        skinned_motions[item.index] = motion_.push(range.id, range.model);
                    }
                }

                // Back-to-front by squared distance from the eye to each item's world
                // translation, so the classic painter's-algorithm ordering the missing depth
                // write requires holds regardless of authoring order or kind.
                std::sort(order.begin(), order.end(),
                         [](const DrawItem& a, const DrawItem& b)
                         { return a.distance_squared > b.distance_squared; });

                graph.add_pass(
                    "transparent",
                    [&](Graph::RenderPassBuilder& builder)
                    {
                        builder.color_attachment(0, frame.targets.hdr, Graph::AttachmentLoad::Load);
                        builder.color_attachment(1, frame.targets.id, Graph::AttachmentLoad::Load);
                        builder.color_attachment(2, frame.targets.velocity,
                                                 Graph::AttachmentLoad::Load);
                        builder.color_attachment(3, frame.targets.gbuffer,
                                                 Graph::AttachmentLoad::Load);
                        // OpaquePass already wrote the depth this frame; loaded here for the
                        // depth test only, since the pipeline does not write it.
                        builder.depth_stencil_attachment(frame.targets.depth,
                                                         Graph::AttachmentLoad::Load, 0.0f, 0,
                                                         true);
                        // Everything set 0 points at, declared where it is written.
                        declare_shading_set(builder, frame);
                    },
                    [this, &frame, order, instance_materials, instance_motions, skinned_materials,
                     skinned_motions](VkCommandBuffer cmd, const Graph::PassContext& context)
                    {
                        const ShadingSetSources sources{ibl_,       cloud_shadow_, gi_,
                                                        materials_, motion_,       lights_};
                        Scene::SceneSetWriter writer;
                        write_shading_set(writer, sources, frame, context);
                        writer.commit(cmd, frame.layout->pipeline_layout());
                        frame.layout->bind_heap(cmd);

                        const VkPipelineLayout pipeline_layout = frame.layout->pipeline_layout();
                        const VkDeviceSize zero = 0;
                        const VkBuffer skinned_vertices = skinning_.output_buffer(frame.slot);
                        const std::vector<Scene::SkinnedRange>& skinned_ranges =
                            skinning_.ranges();

                        // Rigid and skinned items are interleaved by depth, so the pipeline
                        // (and, for rigid draws, the vertex/index buffers) is rebound whenever
                        // consecutive items differ — the sorted order is small, so the extra
                        // rebinds cost far less than drawing out of order would.
                        ItemKind bound_kind = ItemKind::Rigid;
                        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                          mesh_pipeline_.get());
                        vkCmdSetStencilReference(cmd, VK_STENCIL_FACE_FRONT_AND_BACK, 0);

                        VkBuffer bound_vertices = VK_NULL_HANDLE;
                        for (const DrawItem& item : order)
                        {
                            if (item.kind == ItemKind::Rigid)
                            {
                                if (bound_kind != ItemKind::Rigid)
                                {
                                    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                                      mesh_pipeline_.get());
                                    bound_kind = ItemKind::Rigid;
                                    bound_vertices = VK_NULL_HANDLE;
                                }

                                const MeshInstance& instance = frame.draws.instances[item.index];
                                const bool imported = instance.mesh != INVALID_MESH;
                                const Geometry::Mesh& mesh =
                                    imported ? meshes_.mesh(instance.mesh)
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

                                const Matrix4 model =
                                    imported ? instance.model
                                             : mul(instance.model,
                                                   Geometry::shape_scale(instance.kind,
                                                                         instance.shape_params));
                                const MeshPushConstants push = make_push(
                                    model, frame.eye, instance.material, instance.id,
                                    frame.draws.selected_id, instance_materials[item.index],
                                    instance_motions[item.index]);
                                vkCmdPushConstants(cmd, pipeline_layout, PUSH_STAGES, 0,
                                                   sizeof(MeshPushConstants), &push);
                                vkCmdDrawIndexed(cmd, mesh.index_count, 1, 0, 0, 0);
                            }
                            else
                            {
                                if (skinned_vertices == VK_NULL_HANDLE)
                                    continue;
                                if (bound_kind != ItemKind::Skinned)
                                {
                                    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                                      skinned_pipeline_.get());
                                    bound_kind = ItemKind::Skinned;
                                    bound_vertices = VK_NULL_HANDLE;
                                }

                                const Scene::SkinnedRange& range = skinned_ranges[item.index];
                                const Geometry::Mesh& mesh = meshes_.mesh(range.mesh);
                                if (mesh.index_count == 0)
                                    continue;
                                const VkDeviceSize vertex_offset =
                                    static_cast<VkDeviceSize>(range.base_vertex) *
                                    Scene::SKINNED_VERTEX_SIZE;
                                vkCmdBindVertexBuffers(cmd, 0, 1, &skinned_vertices,
                                                       &vertex_offset);
                                vkCmdBindIndexBuffer(cmd, mesh.indices, 0, VK_INDEX_TYPE_UINT32);

                                const MeshPushConstants push = make_push(
                                    range.model, frame.eye, range.material, range.id,
                                    frame.draws.selected_id, skinned_materials[item.index],
                                    skinned_motions[item.index]);
                                vkCmdPushConstants(cmd, pipeline_layout, PUSH_STAGES, 0,
                                                   sizeof(MeshPushConstants), &push);
                                vkCmdDrawIndexed(cmd, range.index_count, 1, 0, 0, 0);
                            }
                        }
                    });
            }
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
