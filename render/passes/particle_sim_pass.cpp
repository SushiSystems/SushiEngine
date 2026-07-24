/**************************************************************************/
/* particle_sim_pass.cpp                                                  */
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

#include "passes/particle_sim_pass.hpp"

#include <vector>

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

                void memory_barrier(VkCommandBuffer cmd, VkPipelineStageFlags2 src_stage,
                                    VkAccessFlags2 src_access, VkPipelineStageFlags2 dst_stage,
                                    VkAccessFlags2 dst_access)
                {
                    VkMemoryBarrier2 barrier{};
                    barrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER_2;
                    barrier.srcStageMask = src_stage;
                    barrier.srcAccessMask = src_access;
                    barrier.dstStageMask = dst_stage;
                    barrier.dstAccessMask = dst_access;

                    VkDependencyInfo dependency{};
                    dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
                    dependency.memoryBarrierCount = 1;
                    dependency.pMemoryBarriers = &barrier;
                    vkCmdPipelineBarrier2(cmd, &dependency);
                }
            } // namespace

            ParticleSimPass::ParticleSimPass(Vulkan::VulkanDevice& device,
                                             Resources::ShaderLibrary& shaders,
                                             Resources::GraphicsPipelineFactory& pipelines,
                                             Scene::ParticleSystem& particles)
                : device_(device), shaders_(shaders), pipelines_(pipelines), particles_(particles)
            {
                // Seven storage buffers: pool, emitter table, additive draw list, indirect args,
                // curve LUTs, gradient LUTs, alpha draw list. Shared by emit and simulate.
                VkDescriptorSetLayoutBinding bindings[7]{};
                for (std::uint32_t i = 0; i < 7; ++i)
                {
                    bindings[i].binding = i;
                    bindings[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                    bindings[i].descriptorCount = 1;
                    bindings[i].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                }

                VkDescriptorSetLayoutCreateInfo layout_info{};
                layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
                layout_info.bindingCount = 7;
                layout_info.pBindings = bindings;
                Vulkan::check(vkCreateDescriptorSetLayout(device_.device(), &layout_info, nullptr,
                                                          &set_layout_),
                              "vkCreateDescriptorSetLayout(particle sim)");

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
                              "vkCreatePipelineLayout(particle sim)");

                create_pipelines();
            }

            ParticleSimPass::~ParticleSimPass()
            {
                destroy_pipelines();
                if (pipeline_layout_ != VK_NULL_HANDLE)
                    vkDestroyPipelineLayout(device_.device(), pipeline_layout_, nullptr);
                if (set_layout_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorSetLayout(device_.device(), set_layout_, nullptr);
            }

            void ParticleSimPass::create_pipelines()
            {
                emit_pipeline_ =
                    pipelines_.create_compute(pipeline_layout_, shaders_.module("particle_emit.comp"));
                simulate_pipeline_ = pipelines_.create_compute(
                    pipeline_layout_, shaders_.module("particle_simulate.comp"));
            }

            void ParticleSimPass::destroy_pipelines()
            {
                if (emit_pipeline_ != VK_NULL_HANDLE)
                    vkDestroyPipeline(device_.device(), emit_pipeline_, nullptr);
                if (simulate_pipeline_ != VK_NULL_HANDLE)
                    vkDestroyPipeline(device_.device(), simulate_pipeline_, nullptr);
                emit_pipeline_ = VK_NULL_HANDLE;
                simulate_pipeline_ = VK_NULL_HANDLE;
            }

            void ParticleSimPass::rebuild_pipelines()
            {
                destroy_pipelines();
                create_pipelines();
            }

            void ParticleSimPass::register_pass(Graph::RenderGraph& graph,
                                                const Frame::FrameContext& frame)
            {
                if (particles_.empty() || !frame.quality.gpu_particles)
                    return;

                const std::uint32_t slot = frame.slot;
                const std::uint32_t capacity = particles_.capacity();
                const float dt =
                    frame.draws.emitter_count > 0 ? frame.draws.emitters[0].dt : 0.0f;

                graph.add_pass(
                    "particle sim",
                    [&frame](Graph::RenderPassBuilder& builder)
                    {
                        builder.write(frame.targets.particle_draw, Graph::BufferAccess::StorageWrite);
                        builder.write(frame.targets.particle_alpha, Graph::BufferAccess::StorageWrite);
                        builder.write(frame.targets.particle_args, Graph::BufferAccess::StorageWrite);
                    },
                    [this, &frame, slot, capacity, dt](VkCommandBuffer cmd,
                                                       const Graph::PassContext& context)
                    {
                        const VkBuffer draw_buffer = context.buffer(frame.targets.particle_draw);
                        const VkBuffer alpha_buffer = context.buffer(frame.targets.particle_alpha);
                        const VkBuffer args_buffer = context.buffer(frame.targets.particle_args);

                        // Zero the device-local pool exactly once, so every slot starts dead.
                        if (particles_.needs_clear())
                        {
                            vkCmdFillBuffer(cmd, particles_.pool(), 0, VK_WHOLE_SIZE, 0);
                            memory_barrier(cmd, VK_PIPELINE_STAGE_2_ALL_TRANSFER_BIT,
                                           VK_ACCESS_2_TRANSFER_WRITE_BIT,
                                           VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                           VK_ACCESS_2_SHADER_STORAGE_READ_BIT |
                                               VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT);
                            particles_.mark_cleared();
                        }

                        // Reset both indirect draws (additive at [0..3], alpha at [4..7]) to
                        // 6 vertices, zero instances, then make the reset visible to the atomics.
                        const std::uint32_t initial_args[8] = {6u, 0u, 0u, 0u, 6u, 0u, 0u, 0u};
                        vkCmdUpdateBuffer(cmd, args_buffer, 0, sizeof(initial_args), initial_args);
                        memory_barrier(cmd, VK_PIPELINE_STAGE_2_ALL_TRANSFER_BIT,
                                       VK_ACCESS_2_TRANSFER_WRITE_BIT,
                                       VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                       VK_ACCESS_2_SHADER_STORAGE_READ_BIT |
                                           VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT);

                        const VkDescriptorSet set = frame.descriptors->allocate(set_layout_);
                        Resources::DescriptorWriter writer;
                        writer.storage_buffer(0, particles_.pool(), particles_.pool_range());
                        writer.storage_buffer(1, particles_.emitter_buffer(slot),
                                              particles_.emitter_range());
                        writer.storage_buffer(2, draw_buffer, particles_.pool_range());
                        writer.storage_buffer(3, args_buffer, sizeof(initial_args));
                        writer.storage_buffer(4, particles_.curve_lut_buffer(slot),
                                              particles_.curve_lut_range());
                        writer.storage_buffer(5, particles_.gradient_lut_buffer(slot),
                                              particles_.gradient_lut_range());
                        writer.storage_buffer(6, alpha_buffer, particles_.pool_range());
                        writer.update(device_.device(), set);
                        Resources::bind_descriptor_set(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                                       pipeline_layout_, 0, set);

                        // Advance the existing particles first, over the whole pool.
                        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, simulate_pipeline_);
                        Push simulate_push{};
                        simulate_push.emitter_index = 0;
                        simulate_push.capacity = capacity;
                        simulate_push.dt = dt;
                        vkCmdPushConstants(cmd, pipeline_layout_, VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                           sizeof(Push), &simulate_push);
                        vkCmdDispatch(cmd, groups(capacity), 1, 1);

                        // Order the emit pass after the sweep so it wins the recycled ring slots.
                        memory_barrier(cmd, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                       VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT,
                                       VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                       VK_ACCESS_2_SHADER_STORAGE_READ_BIT |
                                           VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT);

                        // Then spawn each emitter's new particles into the ring.
                        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, emit_pipeline_);
                        const std::vector<Scene::GpuEmitter>& emitters = particles_.emitters();
                        for (std::uint32_t e = 0; e < emitters.size(); ++e)
                        {
                            if (emitters[e].spawn_count == 0)
                                continue;
                            Push emit_push{};
                            emit_push.emitter_index = e;
                            emit_push.capacity = capacity;
                            emit_push.dt = dt;
                            vkCmdPushConstants(cmd, pipeline_layout_, VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                               sizeof(Push), &emit_push);
                            vkCmdDispatch(cmd, groups(emitters[e].spawn_count), 1, 1);
                        }
                    });
            }
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
