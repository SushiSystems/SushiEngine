/**************************************************************************/
/* particle_sort_pass.cpp                                                 */
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

#include "passes/particle_sort_pass.hpp"

#include "frame/frame_context.hpp"
#include "graph/render_graph.hpp"
#include "resources/descriptor_allocator.hpp"
#include "resources/descriptor_writer.hpp"
#include "resources/pipeline_cache.hpp"
#include "resources/shader_library.hpp"
#include "rhi/vulkan/vulkan_check.hpp"
#include "rhi/vulkan/vulkan_device.hpp"
#include "scene/particle_system.hpp"

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

                void compute_barrier(VkCommandBuffer cmd)
                {
                    VkMemoryBarrier2 barrier{};
                    barrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER_2;
                    barrier.srcStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
                    barrier.srcAccessMask = VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT;
                    barrier.dstStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
                    barrier.dstAccessMask =
                        VK_ACCESS_2_SHADER_STORAGE_READ_BIT | VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT;

                    VkDependencyInfo dependency{};
                    dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
                    dependency.memoryBarrierCount = 1;
                    dependency.pMemoryBarriers = &barrier;
                    vkCmdPipelineBarrier2(cmd, &dependency);
                }
            } // namespace

            ParticleSortPass::ParticleSortPass(Vulkan::VulkanDevice& device,
                                               Resources::ShaderLibrary& shaders,
                                               Resources::GraphicsPipelineFactory& pipelines,
                                               Scene::ParticleSystem& particles)
                : device_(device), shaders_(shaders), pipelines_(pipelines), particles_(particles)
            {
                // Three storage buffers: the alpha list, the sort keys, and the draw args.
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
                              "vkCreateDescriptorSetLayout(particle sort)");

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
                              "vkCreatePipelineLayout(particle sort)");

                create_pipeline();
            }

            ParticleSortPass::~ParticleSortPass()
            {
                destroy_pipeline();
                if (pipeline_layout_ != VK_NULL_HANDLE)
                    vkDestroyPipelineLayout(device_.device(), pipeline_layout_, nullptr);
                if (set_layout_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorSetLayout(device_.device(), set_layout_, nullptr);
            }

            void ParticleSortPass::create_pipeline()
            {
                pipeline_ = pipelines_.create_compute(pipeline_layout_,
                                                      shaders_.module("particle_sort.comp"));
            }

            void ParticleSortPass::destroy_pipeline()
            {
                if (pipeline_ != VK_NULL_HANDLE)
                    vkDestroyPipeline(device_.device(), pipeline_, nullptr);
                pipeline_ = VK_NULL_HANDLE;
            }

            void ParticleSortPass::rebuild_pipelines()
            {
                destroy_pipeline();
                create_pipeline();
            }

            void ParticleSortPass::register_pass(Graph::RenderGraph& graph,
                                                 const Frame::FrameContext& frame)
            {
                if (particles_.empty() || !frame.quality.gpu_particles)
                    return;

                const std::uint32_t count = particles_.capacity();
                const bool sort_alpha = particles_.has_alpha();
                float eye[4] = {static_cast<float>(frame.eye[0]), static_cast<float>(frame.eye[1]),
                                static_cast<float>(frame.eye[2]), 0.0f};

                graph.add_pass(
                    "particle sort",
                    [&frame](Graph::RenderPassBuilder& builder)
                    {
                        builder.read(frame.targets.particle_alpha, Graph::BufferAccess::StorageRead);
                        builder.read(frame.targets.particle_args, Graph::BufferAccess::StorageRead);
                        builder.write(frame.targets.particle_sort_keys,
                                      Graph::BufferAccess::StorageReadWrite);
                    },
                    [this, &frame, count, sort_alpha, eye](VkCommandBuffer cmd,
                                                           const Graph::PassContext& context)
                    {
                        const VkDescriptorSet set = frame.descriptors->allocate(set_layout_);
                        Resources::DescriptorWriter writer;
                        writer.storage_buffer(0, context.buffer(frame.targets.particle_alpha),
                                              particles_.pool_range());
                        writer.storage_buffer(
                            1, context.buffer(frame.targets.particle_sort_keys),
                            static_cast<VkDeviceSize>(count) * 2 * sizeof(std::uint32_t));
                        writer.storage_buffer(2, context.buffer(frame.targets.particle_args),
                                              sizeof(std::uint32_t) * 8);
                        writer.update(device_.device(), set);
                        Resources::bind_descriptor_set(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                                       pipeline_layout_, 0, set);
                        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_);

                        // Seed the keys (always, so the buffer has a producer).
                        Push push{};
                        push.mode = 0;
                        push.count = count;
                        push.eye[0] = eye[0];
                        push.eye[1] = eye[1];
                        push.eye[2] = eye[2];
                        vkCmdPushConstants(cmd, pipeline_layout_, VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                           sizeof(Push), &push);
                        vkCmdDispatch(cmd, groups(count), 1, 1);

                        if (!sort_alpha)
                            return;

                        // Bitonic sort: log2(N)*(log2(N)+1)/2 compare-exchange stages.
                        for (std::uint32_t k = 2; k <= count; k <<= 1)
                        {
                            for (std::uint32_t j = k >> 1; j > 0; j >>= 1)
                            {
                                compute_barrier(cmd);
                                push.mode = 1;
                                push.stage_k = k;
                                push.stage_j = j;
                                vkCmdPushConstants(cmd, pipeline_layout_,
                                                   VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(Push),
                                                   &push);
                                vkCmdDispatch(cmd, groups(count), 1, 1);
                            }
                        }
                    });
            }
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
