/**************************************************************************/
/* occlusion_pass.cpp                                                     */
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

#include "passes/occlusion_pass.hpp"

#include <algorithm>

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
                constexpr VkFormat OCCLUSION_FORMAT = VK_FORMAT_R32_SFLOAT;
                constexpr std::uint32_t GROUP_SIZE = 8;

                std::uint32_t groups(std::uint32_t extent) noexcept
                {
                    return (extent + GROUP_SIZE - 1) / GROUP_SIZE;
                }

                std::uint32_t mip_levels(std::uint32_t width, std::uint32_t height) noexcept
                {
                    std::uint32_t largest = std::max(width, height);
                    std::uint32_t levels = 1;
                    while (largest > 1)
                    {
                        largest >>= 1;
                        ++levels;
                    }
                    return levels;
                }

                std::uint32_t mip_extent(std::uint32_t base, std::uint32_t level) noexcept
                {
                    return std::max(base >> level, 1u);
                }

                Resources::SamplerDesc point_sampler() noexcept
                {
                    Resources::SamplerDesc desc;
                    desc.filter = VK_FILTER_NEAREST;
                    desc.mipmap_mode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
                    desc.max_lod = 16.0f; // reach every mip the pyramid holds
                    return desc;
                }

                void transition(VkCommandBuffer cmd, VkImage image, std::uint32_t base_mip,
                                std::uint32_t mip_count, VkImageLayout from, VkImageLayout to,
                                VkPipelineStageFlags2 source, VkPipelineStageFlags2 destination,
                                VkAccessFlags2 source_access, VkAccessFlags2 destination_access)
                {
                    VkImageMemoryBarrier2 barrier{};
                    barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
                    barrier.srcStageMask = source;
                    barrier.srcAccessMask = source_access;
                    barrier.dstStageMask = destination;
                    barrier.dstAccessMask = destination_access;
                    barrier.oldLayout = from;
                    barrier.newLayout = to;
                    barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                    barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                    barrier.image = image;
                    barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                    barrier.subresourceRange.baseMipLevel = base_mip;
                    barrier.subresourceRange.levelCount = mip_count;
                    barrier.subresourceRange.layerCount = 1;

                    VkDependencyInfo dependency{};
                    dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
                    dependency.imageMemoryBarrierCount = 1;
                    dependency.pImageMemoryBarriers = &barrier;
                    vkCmdPipelineBarrier2(cmd, &dependency);
                }
            } // namespace

            OcclusionPass::OcclusionPass(Vulkan::VulkanDevice& device,
                                         Resources::ShaderLibrary& shaders,
                                         Resources::GraphicsPipelineFactory& pipelines)
                : device_(device), shaders_(shaders), pipelines_(pipelines)
            {
                VkDescriptorSetLayoutBinding bindings[3]{};
                bindings[0].binding = 0;
                bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                bindings[0].descriptorCount = 1;
                bindings[0].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                bindings[1].binding = 1;
                bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
                bindings[1].descriptorCount = 1;
                bindings[1].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                bindings[2].binding = 2;
                bindings[2].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
                bindings[2].descriptorCount = 1;
                bindings[2].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

                VkDescriptorSetLayoutCreateInfo layout_info{};
                layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
                layout_info.bindingCount = 3;
                layout_info.pBindings = bindings;
                Vulkan::check(vkCreateDescriptorSetLayout(device_.device(), &layout_info, nullptr,
                                                          &set_layout_),
                              "vkCreateDescriptorSetLayout(occlusion)");

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
                              "vkCreatePipelineLayout(occlusion)");

                create_pipeline();
            }

            OcclusionPass::~OcclusionPass()
            {
                destroy_image();
                destroy_pipeline();
                if (pipeline_layout_ != VK_NULL_HANDLE)
                    vkDestroyPipelineLayout(device_.device(), pipeline_layout_, nullptr);
                if (set_layout_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorSetLayout(device_.device(), set_layout_, nullptr);
            }

            void OcclusionPass::create_pipeline()
            {
                pipeline_ =
                    pipelines_.create_compute(pipeline_layout_, shaders_.module("occlusion.comp"));
            }

            void OcclusionPass::destroy_pipeline()
            {
                if (pipeline_ != VK_NULL_HANDLE)
                    vkDestroyPipeline(device_.device(), pipeline_, nullptr);
                pipeline_ = VK_NULL_HANDLE;
            }

            void OcclusionPass::rebuild_pipelines()
            {
                destroy_pipeline();
                create_pipeline();
            }

            void OcclusionPass::create_image(std::uint32_t width, std::uint32_t height)
            {
                width_ = width;
                height_ = height;
                mips_ = mip_levels(width, height);

                VkImageCreateInfo image_info{};
                image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
                image_info.imageType = VK_IMAGE_TYPE_2D;
                image_info.format = OCCLUSION_FORMAT;
                image_info.extent = {width, height, 1};
                image_info.mipLevels = mips_;
                image_info.arrayLayers = 1;
                image_info.samples = VK_SAMPLE_COUNT_1_BIT;
                image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
                image_info.usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT |
                                   VK_IMAGE_USAGE_TRANSFER_DST_BIT;

                VmaAllocationCreateInfo alloc{};
                alloc.usage = VMA_MEMORY_USAGE_AUTO;
                Vulkan::check(vmaCreateImage(device_.allocator(), &image_info, &alloc, &image_,
                                             &allocation_, nullptr),
                              "vmaCreateImage(occlusion)");

                VkImageViewCreateInfo view_info{};
                view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
                view_info.image = image_;
                view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
                view_info.format = OCCLUSION_FORMAT;
                view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                view_info.subresourceRange.levelCount = mips_;
                view_info.subresourceRange.layerCount = 1;
                Vulkan::check(
                    vkCreateImageView(device_.device(), &view_info, nullptr, &sample_view_),
                    "vkCreateImageView(occlusion sample)");

                mip_views_.resize(mips_, VK_NULL_HANDLE);
                for (std::uint32_t level = 0; level < mips_; ++level)
                {
                    VkImageViewCreateInfo mip_info = view_info;
                    mip_info.subresourceRange.baseMipLevel = level;
                    mip_info.subresourceRange.levelCount = 1;
                    Vulkan::check(vkCreateImageView(device_.device(), &mip_info, nullptr,
                                                    &mip_views_[level]),
                                  "vkCreateImageView(occlusion mip)");
                }
            }

            void OcclusionPass::destroy_image()
            {
                for (VkImageView& view : mip_views_)
                    if (view != VK_NULL_HANDLE)
                        vkDestroyImageView(device_.device(), view, nullptr);
                mip_views_.clear();
                if (sample_view_ != VK_NULL_HANDLE)
                    vkDestroyImageView(device_.device(), sample_view_, nullptr);
                if (image_ != VK_NULL_HANDLE)
                    vmaDestroyImage(device_.allocator(), image_, allocation_);
                sample_view_ = VK_NULL_HANDLE;
                image_ = VK_NULL_HANDLE;
                allocation_ = VK_NULL_HANDLE;
                width_ = 0;
                height_ = 0;
                mips_ = 0;
            }

            void OcclusionPass::ensure_extent(std::uint32_t width, std::uint32_t height)
            {
                if (width == 0 || height == 0)
                    return;
                if (image_ != VK_NULL_HANDLE && width_ == width && height_ == height)
                    return;
                if (image_ != VK_NULL_HANDLE)
                    vkDeviceWaitIdle(device_.device());
                destroy_image();
                create_image(width, height);
                // A brand-new image holds no depth; the cull clears it before its first read.
                dirty_ = true;
            }

            void OcclusionPass::prepare_sampling(VkCommandBuffer cmd)
            {
                if (image_ == VK_NULL_HANDLE)
                    return;

                if (dirty_)
                {
                    // No valid depth yet (first frame or post-resize): clear the whole chain to
                    // a far distance so nothing occludes, then make the clear visible to the
                    // sampling read. Self-corrects next frame once real depth is reduced in.
                    transition(cmd, image_, 0, mips_, VK_IMAGE_LAYOUT_UNDEFINED,
                               VK_IMAGE_LAYOUT_GENERAL, VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,
                               VK_PIPELINE_STAGE_2_CLEAR_BIT, VK_ACCESS_2_NONE,
                               VK_ACCESS_2_TRANSFER_WRITE_BIT);
                    VkClearColorValue clear{};
                    clear.float32[0] = 1e30f;
                    VkImageSubresourceRange range{};
                    range.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                    range.levelCount = mips_;
                    range.layerCount = 1;
                    vkCmdClearColorImage(cmd, image_, VK_IMAGE_LAYOUT_GENERAL, &clear, 1, &range);
                    transition(cmd, image_, 0, mips_, VK_IMAGE_LAYOUT_GENERAL,
                               VK_IMAGE_LAYOUT_GENERAL, VK_PIPELINE_STAGE_2_CLEAR_BIT,
                               VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                               VK_ACCESS_2_TRANSFER_WRITE_BIT, VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
                    dirty_ = false;
                    return;
                }

                // Steady state: make last frame's build visible to this frame's sampling read.
                transition(cmd, image_, 0, mips_, VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_GENERAL,
                           VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                           VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                           VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT, VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
            }

            void OcclusionPass::register_pass(Graph::RenderGraph& graph,
                                              const Frame::FrameContext& frame)
            {
                if (!frame.quality.gpu_driven || frame.camera == nullptr ||
                    !frame.targets.depth.valid() || image_ == VK_NULL_HANDLE)
                    return;

                const float near_plane = frame.camera->near_plane;
                const Graph::TextureHandle depth = frame.targets.depth;

                graph.add_pass(
                    "occlusion pyramid",
                    [depth](Graph::RenderPassBuilder& builder)
                    {
                        builder.read(depth, Graph::TextureAccess::SampledCompute);
                        // The pyramid is pass-owned and read next frame by the cull, so the
                        // graph sees no consumer of this pass's output and would cull it.
                        builder.set_side_effect();
                    },
                    [this, &frame, depth, near_plane](VkCommandBuffer cmd,
                                                      const Graph::PassContext& context)
                    {
                        const VkSampler sampler = frame.samplers->get(point_sampler());
                        const VkImageView depth_view = context.sampled_view(depth);

                        // Every level is rewritten from this frame's depth. The transition waits
                        // on the cull's sampling read (COMPUTE) so rebuilding cannot clobber the
                        // pyramid while this frame's cull is still reading last frame's. A frame
                        // whose cull did not run (an object is selected, or the scene is empty)
                        // never moved a freshly created image out of UNDEFINED through
                        // prepare_sampling, so the build initialises it here instead.
                        const VkImageLayout old_layout =
                            dirty_ ? VK_IMAGE_LAYOUT_UNDEFINED : VK_IMAGE_LAYOUT_GENERAL;
                        transition(cmd, image_, 0, mips_, old_layout, VK_IMAGE_LAYOUT_GENERAL,
                                   VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                   VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                   VK_ACCESS_2_SHADER_SAMPLED_READ_BIT,
                                   VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT);
                        dirty_ = false;

                        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_);

                        for (std::uint32_t level = 0; level < mips_; ++level)
                        {
                            const std::uint32_t dw = mip_extent(width_, level);
                            const std::uint32_t dh = mip_extent(height_, level);
                            const std::uint32_t sw =
                                level == 0 ? width_ : mip_extent(width_, level - 1);
                            const std::uint32_t sh =
                                level == 0 ? height_ : mip_extent(height_, level - 1);
                            const std::uint32_t src_level = level == 0 ? 0 : level - 1;

                            const VkDescriptorSet set = frame.descriptors->allocate(set_layout_);
                            Resources::DescriptorWriter writer;
                            writer.sampled_image(0, depth_view, sampler);
                            writer.storage_image(1, mip_views_[src_level]);
                            writer.storage_image(2, mip_views_[level]);
                            writer.update(device_.device(), set);

                            Resources::bind_descriptor_set(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                                           pipeline_layout_, 0, set);
                            Push push{};
                            push.a[0] = level;
                            push.a[1] = dw;
                            push.a[2] = dh;
                            push.a[3] = sw;
                            push.b[0] = sh;
                            push.c[0] = near_plane;
                            vkCmdPushConstants(cmd, pipeline_layout_, VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                               sizeof(Push), &push);
                            vkCmdDispatch(cmd, groups(dw), groups(dh), 1);

                            if (level + 1 < mips_)
                                transition(cmd, image_, level, 1, VK_IMAGE_LAYOUT_GENERAL,
                                           VK_IMAGE_LAYOUT_GENERAL,
                                           VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                           VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                           VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT,
                                           VK_ACCESS_2_SHADER_STORAGE_READ_BIT);
                        }

                        // Leave the whole chain visible to next frame's cull sampling read.
                        transition(cmd, image_, 0, mips_, VK_IMAGE_LAYOUT_GENERAL,
                                   VK_IMAGE_LAYOUT_GENERAL,
                                   VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                   VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                   VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT,
                                   VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
                    });
            }
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
