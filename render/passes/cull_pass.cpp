/**************************************************************************/
/* cull_pass.cpp                                                          */
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

#include "passes/cull_pass.hpp"

#include <cstring>

#include "frame/frame_context.hpp"
#include "graph/render_graph.hpp"
#include "passes/occlusion_pass.hpp"
#include "resources/descriptor_allocator.hpp"
#include "resources/descriptor_writer.hpp"
#include "resources/pipeline_cache.hpp"
#include "resources/sampler_cache.hpp"
#include "resources/shader_library.hpp"
#include "scene/gpu_instance.hpp"
#include "scene/instance_system.hpp"
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
                constexpr VkDeviceSize STATS_BYTES = 4 * sizeof(std::uint32_t);

                std::uint32_t groups(std::uint32_t value) noexcept
                {
                    return value == 0 ? 1u : (value + GROUP_SIZE - 1) / GROUP_SIZE;
                }

                Resources::SamplerDesc point_sampler() noexcept
                {
                    Resources::SamplerDesc desc;
                    desc.filter = VK_FILTER_NEAREST;
                    desc.mipmap_mode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
                    desc.max_lod = 16.0f;
                    return desc;
                }

                void buffer_barrier(VkCommandBuffer cmd, VkBuffer buffer,
                                    VkPipelineStageFlags2 src_stage, VkAccessFlags2 src_access,
                                    VkPipelineStageFlags2 dst_stage, VkAccessFlags2 dst_access)
                {
                    VkBufferMemoryBarrier2 barrier{};
                    barrier.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER_2;
                    barrier.srcStageMask = src_stage;
                    barrier.srcAccessMask = src_access;
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

            CullPass::CullPass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                               Resources::GraphicsPipelineFactory& pipelines,
                               OcclusionPass& occlusion, Scene::InstanceSystem& instances)
                : device_(device), shaders_(shaders), pipelines_(pipelines), occlusion_(occlusion),
                  instances_(instances)
            {
                // Two uniform buffers (the scene block and the cull params), five storage
                // buffers (instances, buckets, draw commands, compacted, stats), and the
                // occlusion pyramid — all read or written by the compute stage.
                VkDescriptorSetLayoutBinding bindings[8]{};
                for (std::uint32_t i = 0; i < 8; ++i)
                {
                    bindings[i].binding = i;
                    bindings[i].descriptorCount = 1;
                    bindings[i].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                    bindings[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                }
                bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
                bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
                bindings[7].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;

                VkDescriptorSetLayoutCreateInfo layout_info{};
                layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
                layout_info.bindingCount = 8;
                layout_info.pBindings = bindings;
                Vulkan::check(vkCreateDescriptorSetLayout(device_.device(), &layout_info, nullptr,
                                                          &set_layout_),
                              "vkCreateDescriptorSetLayout(cull)");

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
                              "vkCreatePipelineLayout(cull)");

                for (SlotBuffers& slot : slots_)
                {
                    VkBufferCreateInfo params_info{};
                    params_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
                    params_info.size = sizeof(Params);
                    params_info.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
                    VmaAllocationCreateInfo params_alloc{};
                    params_alloc.usage = VMA_MEMORY_USAGE_AUTO;
                    params_alloc.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
                                         VMA_ALLOCATION_CREATE_MAPPED_BIT;
                    VmaAllocationInfo params_mapped{};
                    Vulkan::check(vmaCreateBuffer(device_.allocator(), &params_info, &params_alloc,
                                                  &slot.params, &slot.params_allocation,
                                                  &params_mapped),
                                  "vmaCreateBuffer(cull params)");
                    slot.params_mapped = params_mapped.pMappedData;

                    VkBufferCreateInfo stats_info{};
                    stats_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
                    stats_info.size = STATS_BYTES;
                    stats_info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                       VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
                    VmaAllocationCreateInfo stats_alloc{};
                    stats_alloc.usage = VMA_MEMORY_USAGE_AUTO;
                    Vulkan::check(vmaCreateBuffer(device_.allocator(), &stats_info, &stats_alloc,
                                                  &slot.stats, &slot.stats_allocation, nullptr),
                                  "vmaCreateBuffer(cull stats)");

                    VkBufferCreateInfo readback_info{};
                    readback_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
                    readback_info.size = STATS_BYTES;
                    readback_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
                    VmaAllocationCreateInfo readback_alloc{};
                    readback_alloc.usage = VMA_MEMORY_USAGE_AUTO;
                    readback_alloc.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT |
                                           VMA_ALLOCATION_CREATE_MAPPED_BIT;
                    VmaAllocationInfo readback_mapped{};
                    Vulkan::check(vmaCreateBuffer(device_.allocator(), &readback_info,
                                                  &readback_alloc, &slot.readback,
                                                  &slot.readback_allocation, &readback_mapped),
                                  "vmaCreateBuffer(cull readback)");
                    slot.readback_mapped = readback_mapped.pMappedData;
                    if (slot.readback_mapped != nullptr)
                        std::memset(slot.readback_mapped, 0, STATS_BYTES);
                }

                create_pipeline();
            }

            CullPass::~CullPass()
            {
                destroy_pipeline();
                for (SlotBuffers& slot : slots_)
                {
                    if (slot.params != VK_NULL_HANDLE)
                        vmaDestroyBuffer(device_.allocator(), slot.params, slot.params_allocation);
                    if (slot.stats != VK_NULL_HANDLE)
                        vmaDestroyBuffer(device_.allocator(), slot.stats, slot.stats_allocation);
                    if (slot.readback != VK_NULL_HANDLE)
                        vmaDestroyBuffer(device_.allocator(), slot.readback,
                                         slot.readback_allocation);
                }
                if (pipeline_layout_ != VK_NULL_HANDLE)
                    vkDestroyPipelineLayout(device_.device(), pipeline_layout_, nullptr);
                if (set_layout_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorSetLayout(device_.device(), set_layout_, nullptr);
            }

            void CullPass::create_pipeline()
            {
                pipeline_ = pipelines_.create_compute(pipeline_layout_, shaders_.module("cull.comp"));
            }

            void CullPass::destroy_pipeline()
            {
                if (pipeline_ != VK_NULL_HANDLE)
                    vkDestroyPipeline(device_.device(), pipeline_, nullptr);
                pipeline_ = VK_NULL_HANDLE;
            }

            void CullPass::rebuild_pipelines()
            {
                destroy_pipeline();
                create_pipeline();
            }

            CullStatistics CullPass::statistics(std::uint32_t slot) const
            {
                CullStatistics out;
                if (slot >= SLOTS || slots_[slot].readback_mapped == nullptr)
                    return out;
                const std::uint32_t* words =
                    static_cast<const std::uint32_t*>(slots_[slot].readback_mapped);
                out.drawn = words[0];
                out.tested = words[1];
                return out;
            }

            void CullPass::register_pass(Graph::RenderGraph& graph,
                                         const Frame::FrameContext& frame)
            {
                if (!frame.quality.gpu_driven || !frame.settings.gpu_culling.enabled ||
                    frame.gpu_instance_count == 0 || frame.gpu_bucket_count == 0 ||
                    !frame.targets.draw_commands.valid() || !frame.targets.compacted.valid())
                    return;

                const std::uint32_t slot = frame.slot;
                const std::uint32_t bucket_count = frame.gpu_bucket_count;
                const std::uint32_t candidate_count = frame.gpu_instance_count;

                const Graph::BufferHandle commands = frame.targets.draw_commands;
                const Graph::BufferHandle compacted = frame.targets.compacted;
                const Graph::BufferHandle uniforms = frame.targets.uniforms;

                graph.add_pass(
                    "gpu cull",
                    [commands, compacted, uniforms](Graph::RenderPassBuilder& builder)
                    {
                        builder.read(uniforms, Graph::BufferAccess::UniformComputeRead);
                        builder.write(commands, Graph::BufferAccess::StorageWrite);
                        builder.write(compacted, Graph::BufferAccess::StorageWrite);
                    },
                    [this, &frame, slot, bucket_count, candidate_count, commands, compacted,
                     uniforms](VkCommandBuffer cmd, const Graph::PassContext& context)
                    {
                        // Make last frame's occlusion pyramid readable (or clear it far on the
                        // first frame) before the cull samples it.
                        occlusion_.prepare_sampling(cmd);

                        // Fill the cull parameters into this slot's mapped UBO. Host writes made
                        // before this submit are visible to the compute read.
                        Params values{};
                        std::memcpy(values.previous_view_projection, frame.previous_view_projection,
                                    sizeof(values.previous_view_projection));
                        values.delta_eye[0] = frame.eye_delta[0];
                        values.delta_eye[1] = frame.eye_delta[1];
                        values.delta_eye[2] = frame.eye_delta[2];
                        values.delta_eye[3] = 0.0f;
                        values.hiz[0] = static_cast<float>(occlusion_.mip_count());
                        values.hiz[1] = static_cast<float>(occlusion_.width());
                        values.hiz[2] = static_cast<float>(occlusion_.height());
                        values.hiz[3] = frame.occlusion_near;
                        values.flags[0] = frame.settings.gpu_culling.frustum ? 1.0f : 0.0f;
                        values.flags[1] = frame.settings.gpu_culling.occlusion ? 1.0f : 0.0f;
                        values.flags[2] = frame.settings.gpu_culling.min_screen_diameter;
                        values.flags[3] = 0.0f;
                        if (slots_[slot].params_mapped != nullptr)
                            std::memcpy(slots_[slot].params_mapped, &values, sizeof(values));

                        const VkBuffer stats = slots_[slot].stats;
                        const VkBuffer readback = slots_[slot].readback;

                        const VkDescriptorSet set = frame.descriptors->allocate(set_layout_);
                        Resources::DescriptorWriter writer;
                        // Only the leading two matrices of the scene block are read here.
                        writer.uniform_buffer(0, context.buffer(uniforms), 2 * 16 * sizeof(float));
                        writer.uniform_buffer(1, slots_[slot].params, sizeof(Params));
                        writer.storage_buffer(2, instances_.instance_buffer(),
                                              instances_.instance_buffer_range());
                        writer.storage_buffer(3, instances_.bucket_buffer(),
                                              instances_.bucket_buffer_range());
                        writer.storage_buffer(
                            4, context.buffer(commands),
                            bucket_count * sizeof(VkDrawIndexedIndirectCommand));
                        writer.storage_buffer(5, context.buffer(compacted),
                                              candidate_count * sizeof(std::uint32_t));
                        writer.storage_buffer(6, stats, STATS_BYTES);
                        writer.sampled_image(7, occlusion_.pyramid_view(),
                                             frame.samplers->get(point_sampler()),
                                             VK_IMAGE_LAYOUT_GENERAL);
                        writer.update(device_.device(), set);

                        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_);
                        Resources::bind_descriptor_set(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                                       pipeline_layout_, 0, set);

                        // Pass 0: seed each bucket's indirect command and zero the counters.
                        Push seed{};
                        seed.mode = 0;
                        seed.bucket_count = bucket_count;
                        seed.candidate_count = candidate_count;
                        vkCmdPushConstants(cmd, pipeline_layout_, VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                           sizeof(Push), &seed);
                        vkCmdDispatch(cmd, groups(bucket_count), 1, 1);

                        // Make the seeded counters visible to the atomics the cull pass runs.
                        buffer_barrier(cmd, context.buffer(commands),
                                       VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                       VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT,
                                       VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                       VK_ACCESS_2_SHADER_STORAGE_READ_BIT |
                                           VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT);
                        buffer_barrier(cmd, stats, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                       VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT,
                                       VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                       VK_ACCESS_2_SHADER_STORAGE_READ_BIT |
                                           VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT);

                        // Pass 1: cull each instance and compact the survivors per bucket.
                        Push cull{};
                        cull.mode = 1;
                        cull.bucket_count = bucket_count;
                        cull.candidate_count = candidate_count;
                        vkCmdPushConstants(cmd, pipeline_layout_, VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                           sizeof(Push), &cull);
                        vkCmdDispatch(cmd, groups(candidate_count), 1, 1);

                        // Copy the counts into the host-visible readback the editor reads a frame
                        // late; it is only for the cull statistics, never for the draw itself.
                        buffer_barrier(cmd, stats, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                       VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT,
                                       VK_PIPELINE_STAGE_2_COPY_BIT, VK_ACCESS_2_TRANSFER_READ_BIT);
                        VkBufferCopy copy{};
                        copy.size = STATS_BYTES;
                        vkCmdCopyBuffer(cmd, stats, readback, 1, &copy);
                        buffer_barrier(cmd, readback, VK_PIPELINE_STAGE_2_COPY_BIT,
                                       VK_ACCESS_2_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_2_HOST_BIT,
                                       VK_ACCESS_2_HOST_READ_BIT);
                    });
            }
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
