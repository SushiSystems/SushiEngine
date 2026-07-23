/**************************************************************************/
/* cloth_pass.cpp                                                         */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

#include "passes/cloth_pass.hpp"

#include "frame/frame_context.hpp"
#include "geometry/cloth_buffers.hpp"
#include "graph/render_graph.hpp"
#include "resources/descriptor_allocator.hpp"
#include "resources/descriptor_writer.hpp"
#include "resources/pipeline_cache.hpp"
#include "resources/shader_library.hpp"
#include "rhi/vulkan/vulkan_check.hpp"
#include "rhi/vulkan/vulkan_device.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Passes
        {
            namespace
            {
                constexpr std::uint32_t GROUP_SIZE = 64;

                std::uint32_t groups(std::uint32_t value) noexcept
                {
                    return value == 0 ? 1u : (value + GROUP_SIZE - 1) / GROUP_SIZE;
                }

                void buffer_barrier(VkCommandBuffer cmd, VkBuffer buffer,
                                    VkPipelineStageFlags2 dst_stage, VkAccessFlags2 dst_access)
                {
                    VkBufferMemoryBarrier2 barrier{};
                    barrier.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER_2;
                    barrier.srcStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
                    barrier.srcAccessMask = VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT;
                    barrier.dstStageMask = dst_stage;
                    barrier.dstAccessMask = dst_access;
                    barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                    barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                    barrier.buffer = buffer;
                    barrier.offset = 0;
                    barrier.size = VK_WHOLE_SIZE;

                    VkDependencyInfo dependency{};
                    dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
                    dependency.bufferMemoryBarrierCount = 1;
                    dependency.pBufferMemoryBarriers = &barrier;
                    vkCmdPipelineBarrier2(cmd, &dependency);
                }
            } // namespace

            ClothPass::ClothPass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                                 Resources::GraphicsPipelineFactory& pipelines,
                                 Geometry::ClothBuffers& cloth)
                : device_(device), shaders_(shaders), pipelines_(pipelines), cloth_(cloth)
            {
                // Three storage buffers: the packed positions (read), and the vertex and index
                // buffers (written).
                VkDescriptorSetLayoutBinding bindings[3]{};
                for (std::uint32_t i = 0; i < 3; ++i)
                {
                    bindings[i].binding = i;
                    bindings[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                    bindings[i].descriptorCount = 1;
                    bindings[i].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                }

                VkDescriptorSetLayoutCreateInfo layout_info{};
                layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
                layout_info.bindingCount = 3;
                layout_info.pBindings = bindings;
                Vulkan::check(vkCreateDescriptorSetLayout(device_.device(), &layout_info, nullptr,
                                                          &set_layout_),
                              "vkCreateDescriptorSetLayout(cloth)");

                VkPushConstantRange range{};
                range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                range.size = sizeof(Push);

                VkPipelineLayoutCreateInfo pipeline_info{};
                pipeline_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
                pipeline_info.setLayoutCount = 1;
                pipeline_info.pSetLayouts = &set_layout_;
                pipeline_info.pushConstantRangeCount = 1;
                pipeline_info.pPushConstantRanges = &range;
                Vulkan::check(vkCreatePipelineLayout(device_.device(), &pipeline_info, nullptr,
                                                     &pipeline_layout_),
                              "vkCreatePipelineLayout(cloth)");

                create_pipeline();
            }

            ClothPass::~ClothPass()
            {
                destroy_pipeline();
                if (pipeline_layout_ != VK_NULL_HANDLE)
                    vkDestroyPipelineLayout(device_.device(), pipeline_layout_, nullptr);
                if (set_layout_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorSetLayout(device_.device(), set_layout_, nullptr);
            }

            void ClothPass::create_pipeline()
            {
                pipeline_ =
                    pipelines_.create_compute(pipeline_layout_, shaders_.module("cloth.comp"));
            }

            void ClothPass::destroy_pipeline()
            {
                if (pipeline_ != VK_NULL_HANDLE)
                    vkDestroyPipeline(device_.device(), pipeline_, nullptr);
                pipeline_ = VK_NULL_HANDLE;
            }

            void ClothPass::rebuild_pipelines()
            {
                destroy_pipeline();
                create_pipeline();
            }

            void ClothPass::register_pass(Graph::RenderGraph& graph,
                                          const Frame::FrameContext& frame)
            {
                if (cloth_.empty())
                    return;

                const std::uint32_t slot = frame.slot;

                graph.add_pass(
                    "cloth",
                    [](Graph::RenderPassBuilder& builder)
                    {
                        // The buffers it writes are ClothBuffers-owned, not graph resources, so
                        // the graph sees no output and would cull the pass; the writes are made
                        // visible to the opaque draw by the hand barrier at the end of execute.
                        builder.set_side_effect();
                    },
                    [this, &frame, slot](VkCommandBuffer cmd, const Graph::PassContext&)
                    {
                        const Geometry::Mesh& mesh = cloth_.mesh(slot);
                        if (mesh.vertices == VK_NULL_HANDLE || mesh.indices == VK_NULL_HANDLE)
                            return;

                        const VkDescriptorSet set = frame.descriptors->allocate(set_layout_);
                        Resources::DescriptorWriter writer;
                        writer.storage_buffer(0, cloth_.positions(slot), cloth_.positions_range());
                        writer.storage_buffer(1, mesh.vertices,
                                              static_cast<VkDeviceSize>(mesh.vertex_count) *
                                                  sizeof(Geometry::MeshVertex));
                        writer.storage_buffer(2, mesh.indices,
                                              static_cast<VkDeviceSize>(mesh.index_count) *
                                                  sizeof(std::uint32_t));
                        writer.update(device_.device(), set);

                        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_);
                        Resources::bind_descriptor_set(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                                       pipeline_layout_, 0, set);

                        for (const Geometry::ClothStrandRange& range : cloth_.ranges())
                        {
                            Push push{};
                            push.a[1] = range.rows;
                            push.a[2] = range.cols;
                            push.a[3] = range.base_vertex;
                            push.b[0] = range.base_index;
                            push.origin[0] = range.origin[0];
                            push.origin[1] = range.origin[1];
                            push.origin[2] = range.origin[2];

                            // Mode 0: one thread per grid vertex writes the MeshVertex.
                            push.a[0] = 0;
                            vkCmdPushConstants(cmd, pipeline_layout_, VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                               sizeof(Push), &push);
                            vkCmdDispatch(cmd, groups(range.vertex_count), 1, 1);

                            // Mode 1: one thread per quad writes its six indices.
                            push.a[0] = 1;
                            vkCmdPushConstants(cmd, pipeline_layout_, VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                               sizeof(Push), &push);
                            vkCmdDispatch(cmd,
                                          groups((range.rows - 1) * (range.cols - 1)), 1, 1);
                        }

                        // Make the written geometry visible to the opaque pass's vertex fetch
                        // and index read; these buffers are outside the graph's tracking.
                        buffer_barrier(cmd, mesh.vertices,
                                       VK_PIPELINE_STAGE_2_VERTEX_ATTRIBUTE_INPUT_BIT,
                                       VK_ACCESS_2_VERTEX_ATTRIBUTE_READ_BIT);
                        buffer_barrier(cmd, mesh.indices, VK_PIPELINE_STAGE_2_INDEX_INPUT_BIT,
                                       VK_ACCESS_2_INDEX_READ_BIT);
                    });
            }
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
