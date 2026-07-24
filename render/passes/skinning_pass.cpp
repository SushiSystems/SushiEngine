/**************************************************************************/
/* skinning_pass.cpp                                                     */
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

#include "passes/skinning_pass.hpp"

#include "frame/frame_context.hpp"
#include "geometry/mesh_registry.hpp"
#include "graph/render_graph.hpp"
#include "resources/descriptor_allocator.hpp"
#include "resources/descriptor_writer.hpp"
#include "resources/pipeline_cache.hpp"
#include "resources/shader_library.hpp"
#include "rhi/vulkan/vulkan_check.hpp"
#include "rhi/vulkan/vulkan_device.hpp"
#include "scene/skinning_system.hpp"

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

            SkinningPass::SkinningPass(Vulkan::VulkanDevice& device,
                                       Resources::ShaderLibrary& shaders,
                                       Resources::GraphicsPipelineFactory& pipelines,
                                       Scene::SkinningSystem& skinning,
                                       Geometry::MeshRegistry& meshes)
                : device_(device), shaders_(shaders), pipelines_(pipelines), skinning_(skinning),
                  meshes_(meshes)
            {
                // Five storage buffers: base vertices, skin stream, current palette, previous
                // palette (read), and the skinned output (written).
                VkDescriptorSetLayoutBinding bindings[5]{};
                for (std::uint32_t i = 0; i < 5; ++i)
                {
                    bindings[i].binding = i;
                    bindings[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                    bindings[i].descriptorCount = 1;
                    bindings[i].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                }

                VkDescriptorSetLayoutCreateInfo layout_info{};
                layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
                layout_info.bindingCount = 5;
                layout_info.pBindings = bindings;
                Vulkan::check(vkCreateDescriptorSetLayout(device_.device(), &layout_info, nullptr,
                                                          &set_layout_),
                              "vkCreateDescriptorSetLayout(skinning)");

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
                              "vkCreatePipelineLayout(skinning)");

                create_pipeline();
            }

            SkinningPass::~SkinningPass()
            {
                destroy_pipeline();
                if (pipeline_layout_ != VK_NULL_HANDLE)
                    vkDestroyPipelineLayout(device_.device(), pipeline_layout_, nullptr);
                if (set_layout_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorSetLayout(device_.device(), set_layout_, nullptr);
            }

            void SkinningPass::create_pipeline()
            {
                pipeline_ =
                    pipelines_.create_compute(pipeline_layout_, shaders_.module("skinning.comp"));
            }

            void SkinningPass::destroy_pipeline()
            {
                if (pipeline_ != VK_NULL_HANDLE)
                    vkDestroyPipeline(device_.device(), pipeline_, nullptr);
                pipeline_ = VK_NULL_HANDLE;
            }

            void SkinningPass::rebuild_pipelines()
            {
                destroy_pipeline();
                create_pipeline();
            }

            void SkinningPass::register_pass(Graph::RenderGraph& graph,
                                             const Frame::FrameContext& frame)
            {
                if (skinning_.empty())
                    return;

                const std::uint32_t slot = frame.slot;

                graph.add_pass(
                    "skinning",
                    [](Graph::RenderPassBuilder& builder)
                    {
                        // The output buffer is SkinningSystem-owned, not a graph resource, so the
                        // graph would cull the pass; the hand barrier at the end makes the write
                        // visible to the geometry passes' vertex fetch.
                        builder.set_side_effect();
                    },
                    [this, &frame, slot](VkCommandBuffer cmd, const Graph::PassContext&)
                    {
                        const VkBuffer output = skinning_.output_buffer(slot);
                        if (output == VK_NULL_HANDLE)
                            return;

                        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_);

                        for (const Scene::SkinnedRange& range : skinning_.ranges())
                        {
                            const Geometry::Mesh& mesh = meshes_.mesh(range.mesh);
                            const VkBuffer skin = meshes_.skin_buffer(range.mesh);
                            if (mesh.vertices == VK_NULL_HANDLE || skin == VK_NULL_HANDLE)
                                continue;

                            const VkDescriptorSet set = frame.descriptors->allocate(set_layout_);
                            Resources::DescriptorWriter writer;
                            writer.storage_buffer(0, mesh.vertices,
                                                  static_cast<VkDeviceSize>(range.vertex_count) *
                                                      sizeof(Geometry::MeshVertex));
                            writer.storage_buffer(1, skin,
                                                  static_cast<VkDeviceSize>(range.vertex_count) *
                                                      Scene::SKIN_VERTEX_SIZE);
                            writer.storage_buffer(2, skinning_.palette_buffer(slot),
                                                  skinning_.palette_range());
                            writer.storage_buffer(3, skinning_.previous_palette_buffer(slot),
                                                  skinning_.palette_range());
                            writer.storage_buffer(4, output, skinning_.output_range());
                            writer.update(device_.device(), set);
                            Resources::bind_descriptor_set(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                                           pipeline_layout_, 0, set);

                            Push push{};
                            push.vertex_count = range.vertex_count;
                            push.palette_base = range.palette_base;
                            push.out_base = range.base_vertex;
                            push.prev_valid = range.prev_valid;
                            vkCmdPushConstants(cmd, pipeline_layout_, VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                               sizeof(Push), &push);
                            vkCmdDispatch(cmd, groups(range.vertex_count), 1, 1);
                        }

                        // Make the skinned vertices visible to the geometry passes' vertex fetch.
                        buffer_barrier(cmd, output, VK_PIPELINE_STAGE_2_VERTEX_ATTRIBUTE_INPUT_BIT,
                                       VK_ACCESS_2_VERTEX_ATTRIBUTE_READ_BIT);
                    });
            }
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
