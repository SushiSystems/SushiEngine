/**************************************************************************/
/* ui_pass.cpp                                                            */
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

#include "passes/ui_pass.hpp"

#include <cstddef>

#include "frame/frame_context.hpp"
#include "geometry/ui_buffers.hpp"
#include "graph/render_graph.hpp"
#include "material/font_atlas.hpp"
#include "material/texture_library.hpp"
#include "resources/descriptor_heap.hpp"
#include "resources/descriptor_writer.hpp"
#include "resources/shader_library.hpp"
#include "rhi/vulkan/vulkan_check.hpp"
#include "rhi/vulkan/vulkan_device.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Passes
        {
            UIPass::UIPass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                           Resources::GraphicsPipelineFactory& pipelines,
                           Geometry::UIBuffers& geometry, const Assets::FontAtlas& font,
                           const Assets::TextureLibrary& textures, Resources::DescriptorHeap& heap)
                : device_(device), shaders_(shaders), pipelines_(pipelines), geometry_(geometry),
                  font_(font), textures_(textures), heap_(heap)
            {
                VkDescriptorSetLayoutCreateInfo empty_info{};
                empty_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
                empty_info.bindingCount = 0;
                Vulkan::check(vkCreateDescriptorSetLayout(device_.device(), &empty_info, nullptr,
                                                          &empty_layout_),
                              "vkCreateDescriptorSetLayout(ui overlay)");

                VkPushConstantRange range{};
                range.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
                range.size = sizeof(Push);

                VkDescriptorSetLayout sets[2] = {empty_layout_, heap_.layout()};

                VkPipelineLayoutCreateInfo pipeline_info{};
                pipeline_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
                pipeline_info.setLayoutCount = heap_.available() ? 2 : 1;
                pipeline_info.pSetLayouts = sets;
                pipeline_info.pushConstantRangeCount = 1;
                pipeline_info.pPushConstantRanges = &range;
                Vulkan::check(vkCreatePipelineLayout(device_.device(), &pipeline_info, nullptr,
                                                     &pipeline_layout_),
                              "vkCreatePipelineLayout(ui overlay)");

                create_pipeline();
            }

            UIPass::~UIPass()
            {
                destroy_pipeline();
                if (pipeline_layout_ != VK_NULL_HANDLE)
                    vkDestroyPipelineLayout(device_.device(), pipeline_layout_, nullptr);
                if (empty_layout_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorSetLayout(device_.device(), empty_layout_, nullptr);
            }

            void UIPass::create_pipeline()
            {
                Resources::GraphicsPipelineDescription description;
                description.layout = pipeline_layout_;
                description.vertex_shader = shaders_.module("ui.vert");
                description.fragment_shader = shaders_.module("ui.frag");
                description.vertex_stride = sizeof(Geometry::UIVertex);
                description.attribute_count = 3;
                description.attributes[0] = {0, VK_FORMAT_R32G32_SFLOAT,
                                             offsetof(Geometry::UIVertex, x)};
                description.attributes[1] = {1, VK_FORMAT_R32G32_SFLOAT,
                                             offsetof(Geometry::UIVertex, u)};
                description.attributes[2] = {2, VK_FORMAT_R8G8B8A8_UNORM,
                                             offsetof(Geometry::UIVertex, color)};
                description.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
                // A UI quad has no meaningful facing and no depth: it is painted in list order
                // over a finished image.
                description.cull_mode = VK_CULL_MODE_NONE;
                description.depth_test = VK_FALSE;
                description.depth_write = VK_FALSE;
                description.color_count = 1;
                description.color_formats[0] = Frame::RESOLVE_FORMAT;
                description.depth_format = VK_FORMAT_UNDEFINED;
                // Premultiplied "over": the host already scaled rgb by alpha, so a translucent
                // panel darkens what is behind it by exactly its own alpha.
                description.blend.enable = VK_TRUE;
                description.blend.source_color = VK_BLEND_FACTOR_ONE;
                description.blend.destination_color = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
                description.blend.color_op = VK_BLEND_OP_ADD;
                description.blend.source_alpha = VK_BLEND_FACTOR_ONE;
                description.blend.destination_alpha = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
                description.blend.alpha_op = VK_BLEND_OP_ADD;
                pipeline_ = pipelines_.create(description);
            }

            void UIPass::destroy_pipeline()
            {
                // The factory owns the pipeline; the pass drops only its handle.
                pipeline_ = Resources::PipelineHandle{};
            }

            void UIPass::rebuild_pipelines()
            {
                destroy_pipeline();
                create_pipeline();
            }

            void UIPass::register_pass(Graph::RenderGraph& graph,
                                       const Frame::FrameContext& frame)
            {
                // Nothing authored, nothing packed, or no heap to reach the atlas through: the
                // overlay simply does not exist this frame.
                if (frame.draws.ui == nullptr || geometry_.empty() || !heap_.available())
                    return;

                const std::uint32_t slot = frame.slot;
                const VkBuffer vertices = geometry_.vertices(slot);
                const VkBuffer indices = geometry_.indices(slot);
                if (vertices == VK_NULL_HANDLE || indices == VK_NULL_HANDLE)
                    return;

                Push push{};
                push.screen[0] = frame.draws.ui->width;
                push.screen[1] = frame.draws.ui->height;
                // Without a baked font the atlas slot resolves to the library's opaque-white
                // default, which is exactly what the untextured geometry wants — so panels keep
                // drawing and only the labels are missing.
                push.atlas[0] = textures_.heap_index(
                    font_.valid() ? static_cast<TextureId>(font_.texture()) : INVALID_TEXTURE,
                    Assets::DefaultTexture::White);
                if (push.atlas[0] == Resources::INVALID_HEAP_INDEX)
                    return;

                const std::uint32_t index_count = geometry_.index_count();

                graph.add_pass(
                    "ui overlay",
                    [&](Graph::RenderPassBuilder& builder)
                    {
                        // Load, not Discard: this composites over the finished image rather than
                        // replacing it.
                        builder.color_attachment(0, frame.targets.resolve,
                                                 Graph::AttachmentLoad::Load);
                    },
                    [this, push, vertices, indices, index_count](VkCommandBuffer command,
                                                                 const Graph::PassContext&)
                    {
                        vkCmdBindPipeline(command, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                          pipeline_.get());
                        Resources::bind_descriptor_set(command, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                                       pipeline_layout_, HEAP_SET, heap_.set());
                        vkCmdPushConstants(command, pipeline_layout_,
                                           VK_SHADER_STAGE_VERTEX_BIT |
                                               VK_SHADER_STAGE_FRAGMENT_BIT,
                                           0, sizeof(push), &push);

                        const VkDeviceSize offset = 0;
                        vkCmdBindVertexBuffers(command, 0, 1, &vertices, &offset);
                        vkCmdBindIndexBuffer(command, indices, 0, VK_INDEX_TYPE_UINT32);
                        vkCmdDrawIndexed(command, index_count, 1, 0, 0, 0);
                    });
            }
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
