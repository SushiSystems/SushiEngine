/**************************************************************************/
/* vulkan_window_renderer.cpp                                             */
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

#include "vulkan_window_renderer.hpp"

#include <stdexcept>
#include <string>

#include "material/asset_library.hpp"
#include "vulkan_scene_view.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Vulkan
        {
            namespace
            {
                /** @brief Throws with context when a Vulkan call did not succeed. */
                void check(VkResult result, const char* what)
                {
                    if (result != VK_SUCCESS)
                        throw std::runtime_error(std::string("SushiEngine: ") + what +
                                                 " failed (VkResult " +
                                                 std::to_string(static_cast<int>(result)) + ")");
                }

                /** @brief Records a sync2 color-image layout transition. */
                void transition(VkCommandBuffer cmd, VkImage image, VkImageLayout old_layout,
                                VkImageLayout new_layout, VkPipelineStageFlags2 src_stage,
                                VkPipelineStageFlags2 dst_stage, VkAccessFlags2 src_access,
                                VkAccessFlags2 dst_access)
                {
                    VkImageMemoryBarrier2 barrier{};
                    barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
                    barrier.srcStageMask = src_stage;
                    barrier.srcAccessMask = src_access;
                    barrier.dstStageMask = dst_stage;
                    barrier.dstAccessMask = dst_access;
                    barrier.oldLayout = old_layout;
                    barrier.newLayout = new_layout;
                    barrier.image = image;
                    barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                    barrier.subresourceRange.levelCount = 1;
                    barrier.subresourceRange.layerCount = 1;

                    VkDependencyInfo dependency{};
                    dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
                    dependency.imageMemoryBarrierCount = 1;
                    dependency.pImageMemoryBarriers = &barrier;
                    vkCmdPipelineBarrier2(cmd, &dependency);
                }

                /** @brief Builds a windowed device desc from the renderer desc. */
                RenderDeviceDescription to_device_desc(const WindowRendererDescription& desc)
                {
                    RenderDeviceDescription device_desc;
                    device_desc.enable_validation = desc.enable_validation;
                    device_desc.preference = desc.preference;
                    device_desc.required_instance_extensions = desc.required_instance_extensions;
                    device_desc.surface_factory = desc.surface_factory;
                    return device_desc;
                }
            } // namespace

            VulkanWindowRenderer::VulkanWindowRenderer(const WindowRendererDescription& desc)
                : device_(to_device_desc(desc)),
                  headless_(device_.surface() == VK_NULL_HANDLE)
            {
                if (!headless_)
                {
                    present_mode_ = desc.present_mode;
                    create_swapchain(desc.width, desc.height);
                    create_frames();
                }
                else
                {
                    // No swapchain to size acquire()'s later resize check against, but
                    // extent_ is still read (color_format()/image_count() callers aside,
                    // nothing headless-relevant needs it) — recorded for completeness.
                    extent_.width = desc.width;
                    extent_.height = desc.height;
                }
                assets_.reset(new Assets::AssetLibrary(device_, desc.shader_source_directory,
                                                        desc.pipeline_cache_path));
            }

            VulkanWindowRenderer::~VulkanWindowRenderer()
            {
                vkDeviceWaitIdle(device_.device());
                assets_.reset();
                destroy_frames();
                destroy_swapchain();
            }

            IAssetLibrary& VulkanWindowRenderer::assets() noexcept { return *assets_; }

            void VulkanWindowRenderer::create_swapchain(std::uint32_t width, std::uint32_t height)
            {
                vkb::SwapchainBuilder builder(device_.physical_device(), device_.device(),
                                              device_.surface());
                builder.set_desired_format(VkSurfaceFormatKHR{
                           VK_FORMAT_B8G8R8A8_UNORM, VK_COLOR_SPACE_SRGB_NONLINEAR_KHR})
                    .set_desired_present_mode(present_mode(present_mode_))
                    .set_desired_extent(width, height)
                    .add_image_usage_flags(VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT);
                if (swapchain_.swapchain != VK_NULL_HANDLE)
                    builder.set_old_swapchain(swapchain_);

                auto result = builder.build();
                if (!result)
                    throw std::runtime_error("SushiEngine: swapchain creation failed: " +
                                             result.error().message());

                destroy_swapchain(); // tears down the old one referenced above
                swapchain_ = result.value();
                format_ = swapchain_.image_format;
                extent_ = swapchain_.extent;
                images_ = swapchain_.get_images().value();
                views_ = swapchain_.get_image_views().value();

                render_finished_.resize(images_.size());
                VkSemaphoreCreateInfo semaphore_info{};
                semaphore_info.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
                for (VkSemaphore& semaphore : render_finished_)
                    check(vkCreateSemaphore(device_.device(), &semaphore_info, nullptr, &semaphore),
                          "vkCreateSemaphore(render_finished)");
            }

            VkPresentModeKHR VulkanWindowRenderer::present_mode(PresentMode mode) noexcept
            {
                switch (mode)
                {
                    case PresentMode::Mailbox:
                        return VK_PRESENT_MODE_MAILBOX_KHR;
                    case PresentMode::Immediate:
                        return VK_PRESENT_MODE_IMMEDIATE_KHR;
                    case PresentMode::Vsync:
                    default:
                        return VK_PRESENT_MODE_FIFO_KHR;
                }
            }

            void VulkanWindowRenderer::set_present_mode(PresentMode mode)
            {
                if (mode == present_mode_)
                    return;
                present_mode_ = mode;
                if (headless_)
                    return; // recorded, but there is no swapchain to rebuild with it
                // The pacing is baked into the swapchain, so changing it means building a
                // new one; the builder falls back to FIFO where the surface does not offer
                // what was asked for, which is why no capability check is needed here.
                vkDeviceWaitIdle(device_.device());
                create_swapchain(extent_.width, extent_.height);
            }

            void VulkanWindowRenderer::destroy_swapchain()
            {
                for (VkImageView view : views_)
                    vkDestroyImageView(device_.device(), view, nullptr);
                for (VkSemaphore semaphore : render_finished_)
                    vkDestroySemaphore(device_.device(), semaphore, nullptr);
                views_.clear();
                render_finished_.clear();
                images_.clear();
                if (swapchain_.swapchain != VK_NULL_HANDLE)
                {
                    vkb::destroy_swapchain(swapchain_);
                    swapchain_ = vkb::Swapchain{};
                }
            }

            void VulkanWindowRenderer::create_frames()
            {
                VkCommandPoolCreateInfo pool_info{};
                pool_info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
                pool_info.flags = VK_COMMAND_POOL_CREATE_TRANSIENT_BIT;
                pool_info.queueFamilyIndex = device_.graphics_queue_family();

                VkSemaphoreCreateInfo semaphore_info{};
                semaphore_info.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;

                VkFenceCreateInfo fence_info{};
                fence_info.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
                fence_info.flags = VK_FENCE_CREATE_SIGNALED_BIT;

                for (FrameResources& frame : frames_)
                {
                    check(vkCreateCommandPool(device_.device(), &pool_info, nullptr, &frame.pool),
                          "vkCreateCommandPool");
                    VkCommandBufferAllocateInfo cmd_info{};
                    cmd_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
                    cmd_info.commandPool = frame.pool;
                    cmd_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
                    cmd_info.commandBufferCount = 1;
                    check(vkAllocateCommandBuffers(device_.device(), &cmd_info, &frame.cmd),
                          "vkAllocateCommandBuffers");
                    check(vkCreateSemaphore(device_.device(), &semaphore_info, nullptr,
                                            &frame.image_available),
                          "vkCreateSemaphore(image_available)");
                    check(vkCreateFence(device_.device(), &fence_info, nullptr, &frame.in_flight),
                          "vkCreateFence");
                }
            }

            void VulkanWindowRenderer::destroy_frames()
            {
                for (FrameResources& frame : frames_)
                {
                    if (frame.in_flight != VK_NULL_HANDLE)
                        vkDestroyFence(device_.device(), frame.in_flight, nullptr);
                    if (frame.image_available != VK_NULL_HANDLE)
                        vkDestroySemaphore(device_.device(), frame.image_available, nullptr);
                    if (frame.pool != VK_NULL_HANDLE)
                        vkDestroyCommandPool(device_.device(), frame.pool, nullptr);
                    frame = FrameResources{};
                }
            }

            VkCommandBuffer VulkanWindowRenderer::acquire(std::uint32_t width, std::uint32_t height)
            {
                if (headless_)
                    return VK_NULL_HANDLE; // no swapchain image ever exists to acquire

                if (frame_acquired_)
                    return frames_[frame_index_].cmd;

                if (width == 0 || height == 0)
                    return VK_NULL_HANDLE; // minimized: nothing to present

                if (width != extent_.width || height != extent_.height)
                {
                    vkDeviceWaitIdle(device_.device());
                    create_swapchain(width, height);
                }

                FrameResources& frame = frames_[frame_index_];
                check(vkWaitForFences(device_.device(), 1, &frame.in_flight, VK_TRUE, UINT64_MAX),
                      "vkWaitForFences");

                VkResult acquired = vkAcquireNextImageKHR(
                    device_.device(), swapchain_.swapchain, UINT64_MAX, frame.image_available,
                    VK_NULL_HANDLE, &image_index_);
                if (acquired == VK_ERROR_OUT_OF_DATE_KHR)
                {
                    vkDeviceWaitIdle(device_.device());
                    create_swapchain(width, height);
                    return VK_NULL_HANDLE;
                }
                if (acquired != VK_SUCCESS && acquired != VK_SUBOPTIMAL_KHR)
                    check(acquired, "vkAcquireNextImageKHR");

                check(vkResetFences(device_.device(), 1, &frame.in_flight), "vkResetFences");
                check(vkResetCommandPool(device_.device(), frame.pool, 0), "vkResetCommandPool");

                VkCommandBufferBeginInfo begin_info{};
                begin_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
                begin_info.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
                check(vkBeginCommandBuffer(frame.cmd, &begin_info), "vkBeginCommandBuffer");

                frame_acquired_ = true;
                rendering_open_ = false;
                return frame.cmd;
            }

            void* VulkanWindowRenderer::begin_frame(std::uint32_t width, std::uint32_t height)
            {
                VkCommandBuffer cmd = acquire(width, height);
                if (cmd == VK_NULL_HANDLE)
                    return nullptr;

                transition(cmd, images_[image_index_], VK_IMAGE_LAYOUT_UNDEFINED,
                           VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                           VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,
                           VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, 0,
                           VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT);

                VkRenderingAttachmentInfo color_attachment{};
                color_attachment.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
                color_attachment.imageView = views_[image_index_];
                color_attachment.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
                color_attachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
                color_attachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
                color_attachment.clearValue.color = {{0.10f, 0.11f, 0.12f, 1.0f}};

                VkRenderingInfo rendering{};
                rendering.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
                rendering.renderArea.extent = extent_;
                rendering.layerCount = 1;
                rendering.colorAttachmentCount = 1;
                rendering.pColorAttachments = &color_attachment;
                vkCmdBeginRendering(cmd, &rendering);

                rendering_open_ = true;
                return cmd;
            }

            void VulkanWindowRenderer::present_scene_view(ISceneView& view, std::uint32_t slot,
                                                           std::uint32_t width,
                                                           std::uint32_t height)
            {
                // Read the source before touching any frame state: a slot that has never
                // rendered must leave the renderer exactly as it found it, so the caller's
                // next attempt (once the view has actually rendered something) starts clean.
                VulkanSceneView& vulkan_view = static_cast<VulkanSceneView&>(view);
                const PresentSource source = vulkan_view.present_source(slot);
                if (source.image == VK_NULL_HANDLE)
                    return;

                VkCommandBuffer cmd = acquire(width, height);
                if (cmd == VK_NULL_HANDLE)
                    return; // minimized or the swapchain was just rebuilt; try again next tick

                if (rendering_open_)
                    return; // a begin_frame() on this renderer already opened a UI scope; the
                            // blit below cannot record inside it, and mixing the two frame
                            // styles in one tick is not a supported call pattern.

                VkImage swap_image = images_[image_index_];

                transition(cmd, swap_image, VK_IMAGE_LAYOUT_UNDEFINED,
                           VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                           VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_2_BLIT_BIT, 0,
                           VK_ACCESS_2_TRANSFER_WRITE_BIT);

                const VkPipelineStageFlags2 source_src_stage =
                    source.state.stage == VK_PIPELINE_STAGE_2_NONE
                        ? VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT
                        : source.state.stage;
                transition(cmd, source.image, source.state.layout,
                           VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, source_src_stage,
                           VK_PIPELINE_STAGE_2_BLIT_BIT, source.state.access,
                           VK_ACCESS_2_TRANSFER_READ_BIT);

                VkImageBlit blit{};
                blit.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                blit.srcSubresource.layerCount = 1;
                blit.srcOffsets[1] = {static_cast<std::int32_t>(source.width),
                                      static_cast<std::int32_t>(source.height), 1};
                blit.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                blit.dstSubresource.layerCount = 1;
                blit.dstOffsets[1] = {static_cast<std::int32_t>(extent_.width),
                                      static_cast<std::int32_t>(extent_.height), 1};
                // A blit, not vkCmdCopyImage: the resolve is R8G8B8A8 and the swapchain is
                // B8G8R8A8 (create_swapchain()'s desired format), a channel order a raw copy
                // would carry over verbatim rather than convert, swapping red and blue in
                // whatever is presented.
                vkCmdBlitImage(cmd, source.image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                               swap_image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &blit,
                               VK_FILTER_LINEAR);

                // Restored to the layout the render graph believes it is resting in — not
                // politeness, the same reason ViewResources::read_output() restores it: the
                // graph tracks this image's state across frames, and returning it in a
                // different layout would make the next render()'s first barrier describe a
                // transition that did not happen.
                transition(cmd, source.image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                           source.state.layout, VK_PIPELINE_STAGE_2_BLIT_BIT, source_src_stage,
                           VK_ACCESS_2_TRANSFER_READ_BIT, source.state.access);

                transition(cmd, swap_image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                           VK_IMAGE_LAYOUT_PRESENT_SRC_KHR, VK_PIPELINE_STAGE_2_BLIT_BIT,
                           VK_PIPELINE_STAGE_2_BOTTOM_OF_PIPE_BIT,
                           VK_ACCESS_2_TRANSFER_WRITE_BIT, 0);
            }

            void VulkanWindowRenderer::end_frame()
            {
                if (!frame_acquired_)
                    return;

                FrameResources& frame = frames_[frame_index_];

                if (rendering_open_)
                {
                    vkCmdEndRendering(frame.cmd);

                    transition(frame.cmd, images_[image_index_],
                               VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                               VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
                               VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                               VK_PIPELINE_STAGE_2_BOTTOM_OF_PIPE_BIT,
                               VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT, 0);
                    rendering_open_ = false;
                }
                // Else present_scene_view() already left the swapchain image in
                // PRESENT_SRC_KHR, or neither ran and this frame was acquired but never
                // targeted — the caller's own bug, not this method's to guess at.

                check(vkEndCommandBuffer(frame.cmd), "vkEndCommandBuffer");

                // The one place in a frame that is after every scene view has submitted, which
                // is exactly what the atmosphere's step has to be ordered behind: it overwrites
                // fields those views sample, and it costs ~13 ms, so running it before them put
                // a whole step of weather in the frame's critical path. Stepping here instead
                // leaves this frame reading the previous step — one nest step of staleness, a
                // couple of seconds of game time, against a medium whose own time scale is
                // minutes.
                if (assets_)
                    assets_->flush_atmosphere();

                // ALL_COMMANDS rather than COLOR_ATTACHMENT_OUTPUT: a begin_frame() submit
                // only ever touches the swapchain image at that stage, but a
                // present_scene_view()-only submit touches it at BLIT instead, which the
                // legacy submit-stage order places after COLOR_ATTACHMENT_OUTPUT — gating on
                // the earlier stage alone would not hold back a command that never reaches
                // it, letting the blit race the presentation engine's release of the image.
                const VkPipelineStageFlags wait_stage = VK_PIPELINE_STAGE_ALL_COMMANDS_BIT;
                VkSubmitInfo submit{};
                submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
                submit.waitSemaphoreCount = 1;
                submit.pWaitSemaphores = &frame.image_available;
                submit.pWaitDstStageMask = &wait_stage;
                submit.commandBufferCount = 1;
                submit.pCommandBuffers = &frame.cmd;
                submit.signalSemaphoreCount = 1;
                submit.pSignalSemaphores = &render_finished_[image_index_];
                check(vkQueueSubmit(device_.graphics_queue(), 1, &submit, frame.in_flight),
                      "vkQueueSubmit");

                VkPresentInfoKHR present{};
                present.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
                present.waitSemaphoreCount = 1;
                present.pWaitSemaphores = &render_finished_[image_index_];
                present.swapchainCount = 1;
                present.pSwapchains = &swapchain_.swapchain;
                present.pImageIndices = &image_index_;
                const VkResult presented = vkQueuePresentKHR(device_.graphics_queue(), &present);
                if (presented != VK_SUCCESS && presented != VK_SUBOPTIMAL_KHR &&
                    presented != VK_ERROR_OUT_OF_DATE_KHR)
                    check(presented, "vkQueuePresentKHR");

                frame_index_ = (frame_index_ + 1) % FRAMES_IN_FLIGHT;
                frame_acquired_ = false;
            }

            void VulkanWindowRenderer::wait_idle()
            {
                vkDeviceWaitIdle(device_.device());
            }

            std::unique_ptr<ISceneView> VulkanWindowRenderer::create_scene_view()
            {
                return std::unique_ptr<ISceneView>(new VulkanSceneView(device_, *assets_));
            }
        } // namespace Vulkan

        std::unique_ptr<IWindowRenderer> create_window_renderer(
            const WindowRendererDescription& desc)
        {
            return std::unique_ptr<IWindowRenderer>(new Vulkan::VulkanWindowRenderer(desc));
        }
    } // namespace Render
} // namespace SushiEngine
