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

namespace SushiEngine
{
    namespace render
    {
        namespace vulkan
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
                RenderDeviceDesc to_device_desc(const WindowRendererDesc& desc)
                {
                    RenderDeviceDesc device_desc;
                    device_desc.enable_validation = desc.enable_validation;
                    device_desc.preference = desc.preference;
                    device_desc.required_instance_extensions = desc.required_instance_extensions;
                    device_desc.surface_factory = desc.surface_factory;
                    return device_desc;
                }
            } // namespace

            VulkanWindowRenderer::VulkanWindowRenderer(const WindowRendererDesc& desc)
                : device_(to_device_desc(desc))
            {
                if (device_.surface() == VK_NULL_HANDLE)
                    throw std::runtime_error(
                        "SushiEngine: a windowed renderer needs a surface factory");
                create_swapchain(desc.width, desc.height);
                create_frames();
            }

            VulkanWindowRenderer::~VulkanWindowRenderer()
            {
                vkDeviceWaitIdle(device_.device());
                destroy_frames();
                destroy_swapchain();
            }

            void VulkanWindowRenderer::create_swapchain(std::uint32_t width, std::uint32_t height)
            {
                vkb::SwapchainBuilder builder(device_.physical_device(), device_.device(),
                                              device_.surface());
                builder.set_desired_format(VkSurfaceFormatKHR{
                           VK_FORMAT_B8G8R8A8_UNORM, VK_COLOR_SPACE_SRGB_NONLINEAR_KHR})
                    .set_desired_present_mode(VK_PRESENT_MODE_FIFO_KHR)
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

            void* VulkanWindowRenderer::begin_frame(std::uint32_t width, std::uint32_t height)
            {
                if (width == 0 || height == 0)
                    return nullptr; // minimized: nothing to present

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
                    return nullptr;
                }
                if (acquired != VK_SUCCESS && acquired != VK_SUBOPTIMAL_KHR)
                    check(acquired, "vkAcquireNextImageKHR");

                check(vkResetFences(device_.device(), 1, &frame.in_flight), "vkResetFences");
                check(vkResetCommandPool(device_.device(), frame.pool, 0), "vkResetCommandPool");

                VkCommandBufferBeginInfo begin_info{};
                begin_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
                begin_info.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
                check(vkBeginCommandBuffer(frame.cmd, &begin_info), "vkBeginCommandBuffer");

                transition(frame.cmd, images_[image_index_], VK_IMAGE_LAYOUT_UNDEFINED,
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
                vkCmdBeginRendering(frame.cmd, &rendering);

                frame_open_ = true;
                return frame.cmd;
            }

            void VulkanWindowRenderer::end_frame()
            {
                if (!frame_open_)
                    return;

                FrameResources& frame = frames_[frame_index_];
                vkCmdEndRendering(frame.cmd);

                transition(frame.cmd, images_[image_index_],
                           VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                           VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
                           VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                           VK_PIPELINE_STAGE_2_BOTTOM_OF_PIPE_BIT,
                           VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT, 0);

                check(vkEndCommandBuffer(frame.cmd), "vkEndCommandBuffer");

                const VkPipelineStageFlags wait_stage =
                    VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
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
                frame_open_ = false;
            }

            void VulkanWindowRenderer::wait_idle()
            {
                vkDeviceWaitIdle(device_.device());
            }
        } // namespace vulkan

        std::unique_ptr<IWindowRenderer> create_window_renderer(const WindowRendererDesc& desc)
        {
            return std::unique_ptr<IWindowRenderer>(new vulkan::VulkanWindowRenderer(desc));
        }
    } // namespace render
} // namespace SushiEngine
