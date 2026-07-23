/**************************************************************************/
/* auto_exposure_pass.cpp                                                 */
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

#include "passes/auto_exposure_pass.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "frame/frame_context.hpp"
#include "graph/render_graph.hpp"
#include "resources/descriptor_allocator.hpp"
#include "resources/descriptor_writer.hpp"
#include "resources/pipeline_cache.hpp"
#include "resources/sampler_cache.hpp"
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
                constexpr std::uint32_t GROUP_SIZE = 16;
                // The histogram's log2-luminance window. The floor has to reach true
                // moonlight, which is physically derived and lands near 3e-6 of sunlight —
                // roughly 2^-18 off a sunlit surface — so a -10 floor would pile every
                // moonlit night into the first bin and meter it as black. The authored EV
                // range clamps the *result*, not the bins.
                constexpr float MIN_LOG = -20.0f;
                constexpr float MAX_LOG = 10.0f;

                std::uint32_t groups(std::uint32_t extent) noexcept
                {
                    return (extent + GROUP_SIZE - 1) / GROUP_SIZE;
                }

                Resources::SamplerDesc linear_sampler() noexcept
                {
                    Resources::SamplerDesc desc;
                    desc.filter = VK_FILTER_LINEAR;
                    desc.mipmap_mode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
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

            AutoExposurePass::AutoExposurePass(Vulkan::VulkanDevice& device,
                                               Resources::ShaderLibrary& shaders,
                                               Resources::GraphicsPipelineFactory& pipelines)
                : device_(device), shaders_(shaders), pipelines_(pipelines)
            {
                VkDescriptorSetLayoutBinding bindings[2]{};
                bindings[0].binding = 0;
                bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                bindings[0].descriptorCount = 1;
                bindings[0].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                bindings[1].binding = 1;
                bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                bindings[1].descriptorCount = 1;
                bindings[1].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

                VkDescriptorSetLayoutCreateInfo layout_info{};
                layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
                layout_info.bindingCount = 2;
                layout_info.pBindings = bindings;
                Vulkan::check(vkCreateDescriptorSetLayout(device_.device(), &layout_info, nullptr,
                                                          &set_layout_),
                              "vkCreateDescriptorSetLayout(auto exposure)");

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
                              "vkCreatePipelineLayout(auto exposure)");

                const VkDeviceSize size = BINS * sizeof(std::uint32_t);
                for (SlotBuffers& slot : slots_)
                {
                    VkBufferCreateInfo histogram_info{};
                    histogram_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
                    histogram_info.size = size;
                    histogram_info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                           VK_BUFFER_USAGE_TRANSFER_SRC_BIT |
                                           VK_BUFFER_USAGE_TRANSFER_DST_BIT;
                    VmaAllocationCreateInfo histogram_alloc{};
                    histogram_alloc.usage = VMA_MEMORY_USAGE_AUTO;
                    Vulkan::check(vmaCreateBuffer(device_.allocator(), &histogram_info,
                                                  &histogram_alloc, &slot.histogram,
                                                  &slot.histogram_allocation, nullptr),
                                  "vmaCreateBuffer(histogram)");

                    VkBufferCreateInfo readback_info{};
                    readback_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
                    readback_info.size = size;
                    readback_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
                    VmaAllocationCreateInfo readback_alloc{};
                    readback_alloc.usage = VMA_MEMORY_USAGE_AUTO;
                    readback_alloc.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT |
                                           VMA_ALLOCATION_CREATE_MAPPED_BIT;
                    VmaAllocationInfo readback_mapped{};
                    Vulkan::check(vmaCreateBuffer(device_.allocator(), &readback_info,
                                                  &readback_alloc, &slot.readback,
                                                  &slot.readback_allocation, &readback_mapped),
                                  "vmaCreateBuffer(histogram readback)");
                    slot.readback_mapped = readback_mapped.pMappedData;
                    std::memset(slot.readback_mapped, 0, size);
                }

                create_pipeline();
            }

            AutoExposurePass::~AutoExposurePass()
            {
                destroy_pipeline();
                for (SlotBuffers& slot : slots_)
                {
                    if (slot.histogram != VK_NULL_HANDLE)
                        vmaDestroyBuffer(device_.allocator(), slot.histogram,
                                         slot.histogram_allocation);
                    if (slot.readback != VK_NULL_HANDLE)
                        vmaDestroyBuffer(device_.allocator(), slot.readback,
                                         slot.readback_allocation);
                }
                if (pipeline_layout_ != VK_NULL_HANDLE)
                    vkDestroyPipelineLayout(device_.device(), pipeline_layout_, nullptr);
                if (set_layout_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorSetLayout(device_.device(), set_layout_, nullptr);
            }

            void AutoExposurePass::create_pipeline()
            {
                pipeline_ = pipelines_.create_compute(pipeline_layout_,
                                                      shaders_.module("luminance_histogram.comp"));
            }

            void AutoExposurePass::destroy_pipeline()
            {
                if (pipeline_ != VK_NULL_HANDLE)
                    vkDestroyPipeline(device_.device(), pipeline_, nullptr);
                pipeline_ = VK_NULL_HANDLE;
            }

            void AutoExposurePass::rebuild_pipelines()
            {
                destroy_pipeline();
                create_pipeline();
            }

            void AutoExposurePass::register_pass(Graph::RenderGraph& graph,
                                                 const Frame::FrameContext& frame)
            {
                // Only meter when the scene actually reads the adapted value; manual exposure
                // needs no histogram.
                if (frame.settings.post.exposure_mode != ExposureMode::Automatic ||
                    !frame.targets.post_color.valid())
                    return;

                const std::uint32_t slot = frame.slot;
                const Graph::TextureHandle source = frame.targets.post_color;
                const std::uint32_t width = frame.post_width();
                const std::uint32_t height = frame.post_height();

                graph.add_pass(
                    "auto exposure",
                    [source](Graph::RenderPassBuilder& builder)
                    {
                        builder.read(source, Graph::TextureAccess::SampledCompute);
                        // The histogram and readback are pass-owned, so the graph sees no output
                        // and would cull the pass without this.
                        builder.set_side_effect();
                    },
                    [this, &frame, source, slot, width, height](VkCommandBuffer cmd,
                                                                const Graph::PassContext& context)
                    {
                        const VkBuffer histogram = slots_[slot].histogram;
                        const VkBuffer readback = slots_[slot].readback;

                        // Clear the histogram, then make the clear visible to the shader writes.
                        vkCmdFillBuffer(cmd, histogram, 0, VK_WHOLE_SIZE, 0);
                        buffer_barrier(cmd, histogram, VK_PIPELINE_STAGE_2_CLEAR_BIT,
                                       VK_ACCESS_2_TRANSFER_WRITE_BIT,
                                       VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                       VK_ACCESS_2_SHADER_STORAGE_READ_BIT |
                                           VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT);

                        const VkDescriptorSet set = frame.descriptors->allocate(set_layout_);
                        Resources::DescriptorWriter writer;
                        writer.sampled_image(0, context.sampled_view(source),
                                             frame.samplers->get(linear_sampler()));
                        writer.storage_buffer(1, histogram, BINS * sizeof(std::uint32_t));
                        writer.update(device_.device(), set);

                        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_);
                        Resources::bind_descriptor_set(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                                       pipeline_layout_, 0, set);
                        Push push{};
                        push.a[0] = static_cast<float>(width);
                        push.a[1] = static_cast<float>(height);
                        push.a[2] = MIN_LOG;
                        push.a[3] = 1.0f / (MAX_LOG - MIN_LOG);
                        vkCmdPushConstants(cmd, pipeline_layout_, VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                           sizeof(Push), &push);
                        vkCmdDispatch(cmd, groups(width), groups(height), 1);

                        // Make the built histogram visible to the copy, then copy it into the
                        // host-visible buffer the scene view reads back after this slot's fence.
                        buffer_barrier(cmd, histogram, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                       VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT,
                                       VK_PIPELINE_STAGE_2_COPY_BIT,
                                       VK_ACCESS_2_TRANSFER_READ_BIT);
                        VkBufferCopy copy{};
                        copy.size = BINS * sizeof(std::uint32_t);
                        vkCmdCopyBuffer(cmd, histogram, readback, 1, &copy);
                        buffer_barrier(cmd, readback, VK_PIPELINE_STAGE_2_COPY_BIT,
                                       VK_ACCESS_2_TRANSFER_WRITE_BIT,
                                       VK_PIPELINE_STAGE_2_HOST_BIT, VK_ACCESS_2_HOST_READ_BIT);
                    });
            }

            float AutoExposurePass::adapt(std::uint32_t slot, const AutoExposureSettings& settings,
                                          float delta_seconds, float current) const
            {
                const std::uint32_t* bins =
                    static_cast<const std::uint32_t*>(slots_[slot].readback_mapped);
                if (bins == nullptr)
                    return current;

                // Bin 0 holds near-black and is excluded so a dark border cannot drag the average.
                double total = 0.0;
                for (std::uint32_t i = 1; i < BINS; ++i)
                    total += static_cast<double>(bins[i]);
                if (total <= 0.0)
                    return current;

                const double low_cut = settings.low_percent * total;
                const double high_cut = settings.high_percent * total;
                double sum_weight = 0.0;
                double sum_log = 0.0;
                double seen = 0.0;
                for (std::uint32_t i = 1; i < BINS; ++i)
                {
                    const double count = static_cast<double>(bins[i]);
                    const double lo = std::max(seen, low_cut);
                    const double hi = std::min(seen + count, high_cut);
                    const double weight = std::max(hi - lo, 0.0);
                    seen += count;
                    if (weight > 0.0)
                    {
                        const float t = (static_cast<float>(i) - 1.0f) / 254.0f;
                        const float log_lum = MIN_LOG + t * (MAX_LOG - MIN_LOG);
                        sum_log += static_cast<double>(log_lum) * weight;
                        sum_weight += weight;
                    }
                }
                if (sum_weight <= 0.0)
                    return current;

                float avg_log = static_cast<float>(sum_log / sum_weight);
                avg_log = std::min(std::max(avg_log, settings.min_ev), settings.max_ev);
                const float avg_luminance = std::exp2(avg_log);
                float target = settings.key / std::max(avg_luminance, 1e-4f);
                target *= std::exp2(settings.compensation);

                // Ease toward the target at the rate for the direction of change: a scene that
                // just brightened wants a smaller exposure, adapted at speed_up.
                const float speed = target < current ? settings.speed_up : settings.speed_down;
                float rate = 1.0f - std::exp2(-std::max(delta_seconds, 0.0f) * speed);
                rate = std::min(std::max(rate, 0.0f), 1.0f);
                return current + (target - current) * rate;
            }
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
