/**************************************************************************/
/* light_shadow_pass.cpp                                                  */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/* Licensed under the Apache License, Version 2.0 (the "License");        */
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

#include "passes/light_shadow_pass.hpp"

#include "frame/frame_context.hpp"
#include "geometry/mesh_registry.hpp"
#include "graph/render_graph.hpp"
#include "lighting/light_system.hpp"
#include "passes/depth_only.hpp"
#include "resources/pipeline_cache.hpp"
#include "resources/shader_library.hpp"
#include "scene/scene_layout.hpp"
#include "scene/scene_uniforms.hpp"
#include "rhi/vulkan/vulkan_device.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Passes
        {
            LightShadowPass::LightShadowPass(Vulkan::VulkanDevice& device,
                                             Resources::ShaderLibrary& shaders,
                                             Resources::GraphicsPipelineFactory& pipelines,
                                             Scene::SceneLayout& layout,
                                             Geometry::MeshRegistry& meshes,
                                             Lighting::LightSystem& lights)
                : device_(device), shaders_(shaders), pipelines_(pipelines), layout_(layout),
                  meshes_(meshes), lights_(lights)
            {
                create_pipeline();
            }

            LightShadowPass::~LightShadowPass() { destroy_pipeline(); }

            void LightShadowPass::create_pipeline()
            {
                Resources::GraphicsPipelineDesc desc = depth_only_pipeline_desc(
                    layout_.pipeline_layout(), shaders_.module("light_shadow.vert"),
                    Frame::SHADOW_FORMAT);
                // Conventional depth like the sun atlas: the spot projection is not
                // reverse-Z, so a LESS compare occludes and the atlas clears to lit.
                desc.depth_compare = VK_COMPARE_OP_LESS;
                desc.cull_mode = VK_CULL_MODE_NONE;
                pipeline_ = pipelines_.create(desc);
            }

            void LightShadowPass::destroy_pipeline()
            {
                pipeline_ = Resources::PipelineHandle{};
            }

            void LightShadowPass::rebuild_pipelines()
            {
                destroy_pipeline();
                create_pipeline();
            }

            void LightShadowPass::register_pass(Graph::RenderGraph& graph,
                                                const Frame::FrameContext& frame)
            {
                if (lights_.shadow_caster_count() == 0 ||
                    !frame.targets.light_shadow_atlas.valid())
                    return;

                graph.add_pass(
                    "light shadows",
                    [&](Graph::RenderPassBuilder& builder)
                    {
                        // One clear over the whole atlas; each caster draws into its own
                        // tile, and a tile no caster fills stays at the far plane (lit).
                        builder.depth_stencil_attachment(frame.targets.light_shadow_atlas,
                                                         Graph::AttachmentLoad::Clear, 1.0f, 0);
                    },
                    [this, &frame](VkCommandBuffer cmd, const Graph::PassContext& context)
                    {
                        Scene::SceneSetWriter writer;
                        writer.storage(Scene::SceneLayout::LIGHT_SHADOW_DATA_BINDING,
                                       lights_.shadow_buffer(), lights_.shadow_buffer_range());
                        writer.commit(cmd, frame.layout->pipeline_layout());
                        frame.layout->bind_heap(cmd);

                        const VkPipelineLayout pipeline_layout = frame.layout->pipeline_layout();
                        const VkDeviceSize zero = 0;
                        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_.get());

                        const std::uint32_t casters = lights_.shadow_caster_count();
                        for (std::uint32_t c = 0; c < casters; ++c)
                        {
                            const Lighting::LightSystem::ShadowTile& tile = lights_.shadow_tile(c);
                            VkViewport viewport{};
                            viewport.x = static_cast<float>(tile.x);
                            viewport.y = static_cast<float>(tile.y);
                            viewport.width = static_cast<float>(tile.size);
                            viewport.height = static_cast<float>(tile.size);
                            viewport.maxDepth = 1.0f;
                            VkRect2D scissor{};
                            scissor.offset = {static_cast<std::int32_t>(tile.x),
                                              static_cast<std::int32_t>(tile.y)};
                            scissor.extent = {tile.size, tile.size};
                            vkCmdSetViewport(cmd, 0, 1, &viewport);
                            vkCmdSetScissor(cmd, 0, 1, &scissor);

                            VkBuffer bound_vertices = VK_NULL_HANDLE;
                            for (std::size_t i = 0; i < frame.draws.instance_count; ++i)
                            {
                                const MeshInstance& instance = frame.draws.instances[i];
                                if (!instance.material.cast_shadows)
                                    continue;
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
                                const Mat4 model =
                                    imported ? instance.model
                                             : mul(instance.model,
                                                   Geometry::shape_scale(instance.kind,
                                                                         instance.shape_params));
                                const Scene::MeshPushConstants push = depth_only_push(
                                    model, frame.eye, static_cast<float>(tile.index));
                                vkCmdPushConstants(cmd, pipeline_layout, DEPTH_PUSH_STAGES, 0,
                                                   sizeof(Scene::MeshPushConstants), &push);
                                vkCmdDrawIndexed(cmd, mesh.index_count, 1, 0, 0, 0);
                            }
                        }
                    });
            }
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
