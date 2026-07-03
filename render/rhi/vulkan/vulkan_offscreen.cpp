/**************************************************************************/
/* vulkan_offscreen.cpp                                                   */
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

#include "vulkan_offscreen.hpp"

#include <cstring>
#include <stdexcept>
#include <string>

#include "triangle.vert.h"
#include "triangle.frag.h"

namespace SushiEngine
{
    namespace Render
    {
        namespace Vulkan
        {
            namespace
            {
                constexpr VkFormat COLOR_FORMAT = VK_FORMAT_R8G8B8A8_UNORM;

                /**
                 * @brief Throws with a message when a Vulkan call did not succeed.
                 * @param result The VkResult to check.
                 * @param what   What was being attempted, for the message.
                 */
                void check(VkResult result, const char* what)
                {
                    if (result != VK_SUCCESS)
                        throw std::runtime_error(std::string("SushiEngine: ") + what +
                                                 " failed (VkResult " +
                                                 std::to_string(static_cast<int>(result)) + ")");
                }

                /**
                 * @brief Creates a shader module from embedded SPIR-V words.
                 * @param device Logical device.
                 * @param words  Pointer to the SPIR-V word array.
                 * @param count  Number of words.
                 * @return The created shader module.
                 */
                VkShaderModule make_shader(VkDevice device, const std::uint32_t* words,
                                           std::size_t count)
                {
                    VkShaderModuleCreateInfo info{};
                    info.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
                    info.codeSize = count * sizeof(std::uint32_t);
                    info.pCode = words;
                    VkShaderModule module = VK_NULL_HANDLE;
                    check(vkCreateShaderModule(device, &info, nullptr, &module),
                          "vkCreateShaderModule");
                    return module;
                }

                /**
                 * @brief Records a Vulkan 1.3 image layout transition on the given command buffer.
                 * @param cmd            Command buffer being recorded.
                 * @param image          Image to transition.
                 * @param old_layout     Current layout.
                 * @param new_layout     Target layout.
                 * @param src_stage      Source pipeline stage mask.
                 * @param dst_stage      Destination pipeline stage mask.
                 * @param src_access     Source access mask.
                 * @param dst_access     Destination access mask.
                 */
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
            } // namespace

            TriangleRenderResult render_triangle_offscreen(VulkanDevice& device,
                                                           std::uint32_t width,
                                                           std::uint32_t height)
            {
                const VkDevice vk_device = device.device();
                const VmaAllocator allocator = device.allocator();

                // Color target (GPU-only) and a host-visible buffer to read it back into.
                VkImageCreateInfo image_info{};
                image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
                image_info.imageType = VK_IMAGE_TYPE_2D;
                image_info.format = COLOR_FORMAT;
                image_info.extent = {width, height, 1};
                image_info.mipLevels = 1;
                image_info.arrayLayers = 1;
                image_info.samples = VK_SAMPLE_COUNT_1_BIT;
                image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
                image_info.usage =
                    VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;

                VmaAllocationCreateInfo image_alloc{};
                image_alloc.usage = VMA_MEMORY_USAGE_AUTO;

                VkImage image = VK_NULL_HANDLE;
                VmaAllocation image_allocation = VK_NULL_HANDLE;
                check(vmaCreateImage(allocator, &image_info, &image_alloc, &image,
                                     &image_allocation, nullptr),
                      "vmaCreateImage");

                VkImageViewCreateInfo view_info{};
                view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
                view_info.image = image;
                view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
                view_info.format = COLOR_FORMAT;
                view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                view_info.subresourceRange.levelCount = 1;
                view_info.subresourceRange.layerCount = 1;
                VkImageView view = VK_NULL_HANDLE;
                check(vkCreateImageView(vk_device, &view_info, nullptr, &view),
                      "vkCreateImageView");

                const VkDeviceSize readback_size =
                    VkDeviceSize(width) * VkDeviceSize(height) * 4;
                VkBufferCreateInfo buffer_info{};
                buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
                buffer_info.size = readback_size;
                buffer_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;

                VmaAllocationCreateInfo buffer_alloc{};
                buffer_alloc.usage = VMA_MEMORY_USAGE_AUTO;
                buffer_alloc.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT |
                                     VMA_ALLOCATION_CREATE_MAPPED_BIT;

                VkBuffer readback = VK_NULL_HANDLE;
                VmaAllocation readback_allocation = VK_NULL_HANDLE;
                VmaAllocationInfo readback_info{};
                check(vmaCreateBuffer(allocator, &buffer_info, &buffer_alloc, &readback,
                                      &readback_allocation, &readback_info),
                      "vmaCreateBuffer");

                // Pipeline: no vertex input, dynamic viewport/scissor, dynamic rendering.
                VkShaderModule vertex_module =
                    make_shader(vk_device, Shaders::triangle_vert_spv,
                                Shaders::triangle_vert_spv_word_count);
                VkShaderModule fragment_module =
                    make_shader(vk_device, Shaders::triangle_frag_spv,
                                Shaders::triangle_frag_spv_word_count);

                VkPipelineShaderStageCreateInfo stages[2]{};
                stages[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
                stages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
                stages[0].module = vertex_module;
                stages[0].pName = "main";
                stages[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
                stages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
                stages[1].module = fragment_module;
                stages[1].pName = "main";

                VkPipelineVertexInputStateCreateInfo vertex_input{};
                vertex_input.sType =
                    VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;

                VkPipelineInputAssemblyStateCreateInfo input_assembly{};
                input_assembly.sType =
                    VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
                input_assembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

                VkPipelineViewportStateCreateInfo viewport_state{};
                viewport_state.sType =
                    VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
                viewport_state.viewportCount = 1;
                viewport_state.scissorCount = 1;

                VkPipelineRasterizationStateCreateInfo raster{};
                raster.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
                raster.polygonMode = VK_POLYGON_MODE_FILL;
                raster.cullMode = VK_CULL_MODE_NONE;
                raster.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
                raster.lineWidth = 1.0f;

                VkPipelineMultisampleStateCreateInfo multisample{};
                multisample.sType =
                    VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
                multisample.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

                VkPipelineColorBlendAttachmentState blend_attachment{};
                blend_attachment.colorWriteMask =
                    VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
                    VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;

                VkPipelineColorBlendStateCreateInfo blend{};
                blend.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
                blend.attachmentCount = 1;
                blend.pAttachments = &blend_attachment;

                const VkDynamicState dynamic_states[2] = {VK_DYNAMIC_STATE_VIEWPORT,
                                                          VK_DYNAMIC_STATE_SCISSOR};
                VkPipelineDynamicStateCreateInfo dynamic{};
                dynamic.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
                dynamic.dynamicStateCount = 2;
                dynamic.pDynamicStates = dynamic_states;

                VkPipelineLayoutCreateInfo layout_info{};
                layout_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
                VkPipelineLayout pipeline_layout = VK_NULL_HANDLE;
                check(vkCreatePipelineLayout(vk_device, &layout_info, nullptr,
                                             &pipeline_layout),
                      "vkCreatePipelineLayout");

                VkFormat color_format = COLOR_FORMAT;
                VkPipelineRenderingCreateInfo rendering_info{};
                rendering_info.sType = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO;
                rendering_info.colorAttachmentCount = 1;
                rendering_info.pColorAttachmentFormats = &color_format;

                VkGraphicsPipelineCreateInfo pipeline_info{};
                pipeline_info.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
                pipeline_info.pNext = &rendering_info;
                pipeline_info.stageCount = 2;
                pipeline_info.pStages = stages;
                pipeline_info.pVertexInputState = &vertex_input;
                pipeline_info.pInputAssemblyState = &input_assembly;
                pipeline_info.pViewportState = &viewport_state;
                pipeline_info.pRasterizationState = &raster;
                pipeline_info.pMultisampleState = &multisample;
                pipeline_info.pColorBlendState = &blend;
                pipeline_info.pDynamicState = &dynamic;
                pipeline_info.layout = pipeline_layout;

                VkPipeline pipeline = VK_NULL_HANDLE;
                check(vkCreateGraphicsPipelines(vk_device, VK_NULL_HANDLE, 1, &pipeline_info,
                                                nullptr, &pipeline),
                      "vkCreateGraphicsPipelines");

                // Command pool + buffer + fence for a single synchronous submit.
                VkCommandPoolCreateInfo pool_info{};
                pool_info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
                pool_info.queueFamilyIndex = device.graphics_queue_family();
                VkCommandPool pool = VK_NULL_HANDLE;
                check(vkCreateCommandPool(vk_device, &pool_info, nullptr, &pool),
                      "vkCreateCommandPool");

                VkCommandBufferAllocateInfo cmd_info{};
                cmd_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
                cmd_info.commandPool = pool;
                cmd_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
                cmd_info.commandBufferCount = 1;
                VkCommandBuffer cmd = VK_NULL_HANDLE;
                check(vkAllocateCommandBuffers(vk_device, &cmd_info, &cmd),
                      "vkAllocateCommandBuffers");

                VkCommandBufferBeginInfo begin_info{};
                begin_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
                begin_info.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
                check(vkBeginCommandBuffer(cmd, &begin_info), "vkBeginCommandBuffer");

                transition(cmd, image, VK_IMAGE_LAYOUT_UNDEFINED,
                           VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                           VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,
                           VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, 0,
                           VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT);

                VkRenderingAttachmentInfo color_attachment{};
                color_attachment.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
                color_attachment.imageView = view;
                color_attachment.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
                color_attachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
                color_attachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
                color_attachment.clearValue.color = {{0.05f, 0.05f, 0.08f, 1.0f}};

                VkRenderingInfo rendering{};
                rendering.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
                rendering.renderArea.extent = {width, height};
                rendering.layerCount = 1;
                rendering.colorAttachmentCount = 1;
                rendering.pColorAttachments = &color_attachment;

                vkCmdBeginRendering(cmd, &rendering);
                vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);

                VkViewport viewport{};
                viewport.width = float(width);
                viewport.height = float(height);
                viewport.maxDepth = 1.0f;
                vkCmdSetViewport(cmd, 0, 1, &viewport);

                VkRect2D scissor{};
                scissor.extent = {width, height};
                vkCmdSetScissor(cmd, 0, 1, &scissor);

                vkCmdDraw(cmd, 3, 1, 0, 0);
                vkCmdEndRendering(cmd);

                transition(cmd, image, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                           VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                           VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                           VK_PIPELINE_STAGE_2_COPY_BIT,
                           VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,
                           VK_ACCESS_2_TRANSFER_READ_BIT);

                VkBufferImageCopy copy{};
                copy.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                copy.imageSubresource.layerCount = 1;
                copy.imageExtent = {width, height, 1};
                vkCmdCopyImageToBuffer(cmd, image,
                                       VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, readback, 1,
                                       &copy);

                check(vkEndCommandBuffer(cmd), "vkEndCommandBuffer");

                VkFenceCreateInfo fence_info{};
                fence_info.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
                VkFence fence = VK_NULL_HANDLE;
                check(vkCreateFence(vk_device, &fence_info, nullptr, &fence),
                      "vkCreateFence");

                VkSubmitInfo submit{};
                submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
                submit.commandBufferCount = 1;
                submit.pCommandBuffers = &cmd;
                check(vkQueueSubmit(device.graphics_queue(), 1, &submit, fence),
                      "vkQueueSubmit");
                check(vkWaitForFences(vk_device, 1, &fence, VK_TRUE, UINT64_MAX),
                      "vkWaitForFences");

                vmaInvalidateAllocation(allocator, readback_allocation, 0, readback_size);

                TriangleRenderResult result;
                result.width = width;
                result.height = height;
                const auto* pixels = static_cast<const std::uint8_t*>(readback_info.pMappedData);
                auto read_pixel = [&](std::uint32_t x, std::uint32_t y)
                {
                    const std::size_t offset = (std::size_t(y) * width + x) * 4;
                    Pixel pixel;
                    pixel.r = pixels[offset + 0];
                    pixel.g = pixels[offset + 1];
                    pixel.b = pixels[offset + 2];
                    pixel.a = pixels[offset + 3];
                    return pixel;
                };
                result.center = read_pixel(width / 2, height / 2);
                result.corner = read_pixel(0, 0);

                vkDestroyFence(vk_device, fence, nullptr);
                vkDestroyCommandPool(vk_device, pool, nullptr);
                vkDestroyPipeline(vk_device, pipeline, nullptr);
                vkDestroyPipelineLayout(vk_device, pipeline_layout, nullptr);
                vkDestroyShaderModule(vk_device, fragment_module, nullptr);
                vkDestroyShaderModule(vk_device, vertex_module, nullptr);
                vmaDestroyBuffer(allocator, readback, readback_allocation);
                vkDestroyImageView(vk_device, view, nullptr);
                vmaDestroyImage(allocator, image, image_allocation);

                return result;
            }
        } // namespace Vulkan
    } // namespace Render
} // namespace SushiEngine
