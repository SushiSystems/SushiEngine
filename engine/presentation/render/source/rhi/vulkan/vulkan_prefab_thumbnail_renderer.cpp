/**************************************************************************/
/* vulkan_prefab_thumbnail_renderer.cpp                                   */
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

#include "vulkan_prefab_thumbnail_renderer.hpp"

#include <cstring>
#include <stdexcept>

#include <SushiEngine/geometry/mesh_thumbnail_camera.hpp>
#include <SushiEngine/material/material.hpp>

#include "prefab_thumbnail.frag.h"
#include "prefab_thumbnail.vert.h"
#include "vulkan_check.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Vulkan
        {
            namespace
            {
                void write_matrix(const Matrix4& source, float destination[16])
                {
                    for (int i = 0; i < 16; ++i)
                        destination[i] = static_cast<float>(source.m[i]);
                }
            } // namespace

            VulkanPrefabThumbnailRenderer::VulkanPrefabThumbnailRenderer(VulkanDevice& device)
                : device_(device)
                , assets_(device)
            {
                VkCommandPoolCreateInfo pool_info{};
                pool_info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
                pool_info.flags = VK_COMMAND_POOL_CREATE_TRANSIENT_BIT;
                pool_info.queueFamilyIndex = device_.graphics_queue_family();
                Vulkan::check(
                    vkCreateCommandPool(device_.device(), &pool_info, nullptr, &command_pool_),
                    "vkCreateCommandPool(prefab thumbnail)");

                create_pipeline();
            }

            VulkanPrefabThumbnailRenderer::~VulkanPrefabThumbnailRenderer()
            {
                destroy_targets();
                destroy_pipeline();
                vkDestroyCommandPool(device_.device(), command_pool_, nullptr);
            }

            void VulkanPrefabThumbnailRenderer::create_pipeline()
            {
                VkShaderModuleCreateInfo vert_info{};
                vert_info.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
                vert_info.codeSize = sizeof(Shaders::prefab_thumbnail_vert_spv);
                vert_info.pCode = Shaders::prefab_thumbnail_vert_spv;
                VkShaderModule vert_module = VK_NULL_HANDLE;
                Vulkan::check(
                    vkCreateShaderModule(device_.device(), &vert_info, nullptr, &vert_module),
                    "vkCreateShaderModule(prefab_thumbnail.vert)");

                VkShaderModuleCreateInfo frag_info{};
                frag_info.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
                frag_info.codeSize = sizeof(Shaders::prefab_thumbnail_frag_spv);
                frag_info.pCode = Shaders::prefab_thumbnail_frag_spv;
                VkShaderModule frag_module = VK_NULL_HANDLE;
                if (vkCreateShaderModule(device_.device(), &frag_info, nullptr, &frag_module) !=
                    VK_SUCCESS)
                {
                    vkDestroyShaderModule(device_.device(), vert_module, nullptr);
                    throw std::runtime_error(
                        "SushiEngine: vkCreateShaderModule(prefab_thumbnail.frag) failed");
                }

                // Set 1 is this renderer's own bindless heap (matching prefab_thumbnail.frag's
                // `layout(set = 1, binding = 0)`); set 0 is reserved but carries no bindings
                // today, since every per-draw value this pipeline needs travels as a push
                // constant -- kept as an empty set 0 rather than renumbering the heap to set 0,
                // so the shader-side set numbers match pbr.frag's own set-1-for-the-heap
                // convention exactly (same layout Phase 3a's VulkanMeshThumbnailRenderer uses).
                VkDescriptorSetLayoutCreateInfo empty_set_info{};
                empty_set_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
                VkDescriptorSetLayout empty_set_layout = VK_NULL_HANDLE;
                if (vkCreateDescriptorSetLayout(device_.device(), &empty_set_info, nullptr,
                                                &empty_set_layout) != VK_SUCCESS)
                {
                    vkDestroyShaderModule(device_.device(), frag_module, nullptr);
                    vkDestroyShaderModule(device_.device(), vert_module, nullptr);
                    throw std::runtime_error(
                        "SushiEngine: vkCreateDescriptorSetLayout(prefab thumbnail empty set 0) "
                        "failed");
                }

                VkDescriptorSetLayout set_layouts[2] = {empty_set_layout, assets_.heap().layout()};
                VkPushConstantRange push_range{};
                push_range.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
                push_range.size = sizeof(Push);

                VkPipelineLayoutCreateInfo layout_info{};
                layout_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
                // Matches scene_layout.cpp's heap_.available() ? 2 : 1 convention: Vulkan only
                // reads the first setLayoutCount entries of pSetLayouts, so this is sufficient
                // even though the array itself still has 2 elements. PrefabThumbnailAssetLibrary's
                // own constructor already throws when its heap is unavailable, so this is defense
                // in depth rather than a path this renderer expects to take.
                layout_info.setLayoutCount = assets_.heap().available() ? 2 : 1;
                layout_info.pSetLayouts = set_layouts;
                layout_info.pushConstantRangeCount = 1;
                layout_info.pPushConstantRanges = &push_range;
                const VkResult layout_result = vkCreatePipelineLayout(
                    device_.device(), &layout_info, nullptr, &pipeline_layout_);
                vkDestroyDescriptorSetLayout(device_.device(), empty_set_layout, nullptr);
                if (layout_result != VK_SUCCESS)
                {
                    vkDestroyShaderModule(device_.device(), frag_module, nullptr);
                    vkDestroyShaderModule(device_.device(), vert_module, nullptr);
                    throw std::runtime_error(
                        "SushiEngine: vkCreatePipelineLayout(prefab thumbnail) failed");
                }

                VkPipelineShaderStageCreateInfo stages[2]{};
                stages[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
                stages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
                stages[0].module = vert_module;
                stages[0].pName = "main";
                stages[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
                stages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
                stages[1].module = frag_module;
                stages[1].pName = "main";

                // Matches MeshVertex's confirmed 60-byte layout: position@0 (offset 0),
                // normal@1 (offset 12), uv0@3 (offset 40) -- locations 2/4/5 (tangent/uv1/color)
                // are declared in MeshVertex but unused by prefab_thumbnail.vert, so they are
                // simply omitted here; Vulkan does not require contiguous attribute locations.
                VkVertexInputBindingDescription binding{};
                binding.binding = 0;
                binding.stride = 60; // sizeof(SushiEngine::Geometry::MeshVertex)
                binding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

                VkVertexInputAttributeDescription attributes[3]{};
                attributes[0] = {0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0};
                attributes[1] = {1, 0, VK_FORMAT_R32G32B32_SFLOAT, 12};
                attributes[2] = {3, 0, VK_FORMAT_R32G32_SFLOAT, 40};

                VkPipelineVertexInputStateCreateInfo vertex_input{};
                vertex_input.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
                vertex_input.vertexBindingDescriptionCount = 1;
                vertex_input.pVertexBindingDescriptions = &binding;
                vertex_input.vertexAttributeDescriptionCount = 3;
                vertex_input.pVertexAttributeDescriptions = attributes;

                VkPipelineInputAssemblyStateCreateInfo input_assembly{};
                input_assembly.sType =
                    VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
                input_assembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

                VkPipelineViewportStateCreateInfo viewport_state{};
                viewport_state.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
                viewport_state.viewportCount = 1;
                viewport_state.scissorCount = 1;

                VkPipelineRasterizationStateCreateInfo rasterization{};
                rasterization.sType =
                    VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
                rasterization.polygonMode = VK_POLYGON_MODE_FILL;
                // VK_CULL_MODE_NONE, matching every other pipeline in this module (see Phase
                // 3a's own VulkanMeshThumbnailRenderer::create_pipeline for the full winding/
                // Y-flip rationale, which applies identically here): this engine's
                // perspective() flips Y for Vulkan's clip space, reversing the apparent winding
                // of every triangle relative to the vertex data's actual glTF CCW-front
                // winding. Culling BACK here would cull what should be front-facing. Not
                // culling is also robust to a mirrored (negative-determinant) node transform.
                rasterization.cullMode = VK_CULL_MODE_NONE;
                rasterization.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
                rasterization.lineWidth = 1.0f;

                VkPipelineMultisampleStateCreateInfo multisample{};
                multisample.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
                multisample.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

                // Reverse-Z (per Matrix4::perspective's documented convention): depth clears to
                // 0.0 and passes when the new fragment's depth is >= what's already there.
                VkPipelineDepthStencilStateCreateInfo depth_stencil{};
                depth_stencil.sType =
                    VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
                depth_stencil.depthTestEnable = VK_TRUE;
                depth_stencil.depthWriteEnable = VK_TRUE;
                depth_stencil.depthCompareOp = VK_COMPARE_OP_GREATER_OR_EQUAL;

                VkPipelineColorBlendAttachmentState color_blend_attachment{};
                color_blend_attachment.colorWriteMask =
                    VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
                    VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;

                VkPipelineColorBlendStateCreateInfo color_blend{};
                color_blend.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
                color_blend.attachmentCount = 1;
                color_blend.pAttachments = &color_blend_attachment;

                const VkDynamicState dynamic_states[2] = {VK_DYNAMIC_STATE_VIEWPORT,
                                                          VK_DYNAMIC_STATE_SCISSOR};
                VkPipelineDynamicStateCreateInfo dynamic_state{};
                dynamic_state.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
                dynamic_state.dynamicStateCount = 2;
                dynamic_state.pDynamicStates = dynamic_states;

                VkFormat color_format = COLOR_FORMAT;
                VkPipelineRenderingCreateInfo rendering_info{};
                rendering_info.sType = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO;
                rendering_info.colorAttachmentCount = 1;
                rendering_info.pColorAttachmentFormats = &color_format;
                rendering_info.depthAttachmentFormat = DEPTH_FORMAT;

                VkGraphicsPipelineCreateInfo pipeline_info{};
                pipeline_info.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
                pipeline_info.pNext = &rendering_info;
                pipeline_info.stageCount = 2;
                pipeline_info.pStages = stages;
                pipeline_info.pVertexInputState = &vertex_input;
                pipeline_info.pInputAssemblyState = &input_assembly;
                pipeline_info.pViewportState = &viewport_state;
                pipeline_info.pRasterizationState = &rasterization;
                pipeline_info.pMultisampleState = &multisample;
                pipeline_info.pDepthStencilState = &depth_stencil;
                pipeline_info.pColorBlendState = &color_blend;
                pipeline_info.pDynamicState = &dynamic_state;
                pipeline_info.layout = pipeline_layout_;

                const VkResult pipeline_result = vkCreateGraphicsPipelines(
                    device_.device(), VK_NULL_HANDLE, 1, &pipeline_info, nullptr, &pipeline_);
                vkDestroyShaderModule(device_.device(), frag_module, nullptr);
                vkDestroyShaderModule(device_.device(), vert_module, nullptr);
                if (pipeline_result != VK_SUCCESS)
                {
                    vkDestroyPipelineLayout(device_.device(), pipeline_layout_, nullptr);
                    pipeline_layout_ = VK_NULL_HANDLE;
                    throw std::runtime_error(
                        "SushiEngine: vkCreateGraphicsPipelines(prefab thumbnail) failed");
                }
            }

            void VulkanPrefabThumbnailRenderer::destroy_pipeline()
            {
                if (pipeline_ != VK_NULL_HANDLE)
                    vkDestroyPipeline(device_.device(), pipeline_, nullptr);
                if (pipeline_layout_ != VK_NULL_HANDLE)
                    vkDestroyPipelineLayout(device_.device(), pipeline_layout_, nullptr);
            }

            void VulkanPrefabThumbnailRenderer::ensure_targets(std::uint32_t width,
                                                                std::uint32_t height)
            {
                if (width <= target_width_ && height <= target_height_ &&
                    color_image_ != VK_NULL_HANDLE)
                    return;

                destroy_targets();

                VkImageCreateInfo color_info{};
                color_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
                color_info.imageType = VK_IMAGE_TYPE_2D;
                color_info.format = COLOR_FORMAT;
                color_info.extent = {width, height, 1};
                color_info.mipLevels = 1;
                color_info.arrayLayers = 1;
                color_info.samples = VK_SAMPLE_COUNT_1_BIT;
                color_info.tiling = VK_IMAGE_TILING_OPTIMAL;
                color_info.usage =
                    VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
                VmaAllocationCreateInfo alloc_info{};
                alloc_info.usage = VMA_MEMORY_USAGE_AUTO;
                Vulkan::check(vmaCreateImage(device_.allocator(), &color_info, &alloc_info,
                                            &color_image_, &color_allocation_, nullptr),
                              "vmaCreateImage(prefab thumbnail color)");

                VkImageViewCreateInfo color_view_info{};
                color_view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
                color_view_info.image = color_image_;
                color_view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
                color_view_info.format = COLOR_FORMAT;
                color_view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                color_view_info.subresourceRange.levelCount = 1;
                color_view_info.subresourceRange.layerCount = 1;
                Vulkan::check(
                    vkCreateImageView(device_.device(), &color_view_info, nullptr, &color_view_),
                    "vkCreateImageView(prefab thumbnail color)");

                VkImageCreateInfo depth_info = color_info;
                depth_info.format = DEPTH_FORMAT;
                depth_info.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
                Vulkan::check(vmaCreateImage(device_.allocator(), &depth_info, &alloc_info,
                                            &depth_image_, &depth_allocation_, nullptr),
                              "vmaCreateImage(prefab thumbnail depth)");

                VkImageViewCreateInfo depth_view_info{};
                depth_view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
                depth_view_info.image = depth_image_;
                depth_view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
                depth_view_info.format = DEPTH_FORMAT;
                depth_view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
                depth_view_info.subresourceRange.levelCount = 1;
                depth_view_info.subresourceRange.layerCount = 1;
                Vulkan::check(
                    vkCreateImageView(device_.device(), &depth_view_info, nullptr, &depth_view_),
                    "vkCreateImageView(prefab thumbnail depth)");

                target_width_ = width;
                target_height_ = height;
            }

            void VulkanPrefabThumbnailRenderer::destroy_targets()
            {
                if (depth_view_ != VK_NULL_HANDLE)
                    vkDestroyImageView(device_.device(), depth_view_, nullptr);
                if (depth_image_ != VK_NULL_HANDLE)
                    vmaDestroyImage(device_.allocator(), depth_image_, depth_allocation_);
                if (color_view_ != VK_NULL_HANDLE)
                    vkDestroyImageView(device_.device(), color_view_, nullptr);
                if (color_image_ != VK_NULL_HANDLE)
                    vmaDestroyImage(device_.allocator(), color_image_, color_allocation_);
                depth_view_ = VK_NULL_HANDLE;
                depth_image_ = VK_NULL_HANDLE;
                depth_allocation_ = VK_NULL_HANDLE;
                color_view_ = VK_NULL_HANDLE;
                color_image_ = VK_NULL_HANDLE;
                color_allocation_ = VK_NULL_HANDLE;
                target_width_ = 0;
                target_height_ = 0;
            }

            bool VulkanPrefabThumbnailRenderer::render_thumbnail(
                const PrefabThumbnailDraw* draws, std::size_t count,
                const SushiEngine::Geometry::AABB3& bounds, std::uint32_t width,
                std::uint32_t height, FrameImage& out_image)
            {
                // This class trusts its input: the caller (a world-tier-aware orchestrator --
                // see this class's own header for why that orchestration cannot live here) has
                // already resolved every entity's mesh/material and composed its model matrix,
                // and already unioned every entity's bounds into @p bounds. There is no
                // "unresolved" or "skip this one" case left to handle here beyond the fixed
                // capacity check below.
                if (count == 0 || count > MAX_PRIMITIVES)
                    return false;

                // render_thumbnail's documented contract (prefab_thumbnail_renderer.hpp) is to
                // return false on any render failure, including a Vulkan error -- never to
                // throw. Every Vulkan::check() below this point (including inside
                // ensure_targets()) can throw, so the whole render/readback sequence is wrapped
                // in a try/catch that converts any such throw into a false return, cleaning up
                // whatever of these was already created (mirroring Phase 3a's
                // VulkanMeshThumbnailRenderer::render_thumbnail exactly). No Vulkan resource
                // exists yet at the point this try block opens, and ensure_targets() itself only
                // ever leaves this renderer's persistent, member-owned targets in a state
                // destroy_targets() (called from the destructor) already knows how to unwind
                // correctly on a partial create.
                VkCommandBuffer command = VK_NULL_HANDLE;
                VkBuffer readback_buffer = VK_NULL_HANDLE;
                VmaAllocation readback_allocation = VK_NULL_HANDLE;
                VkFence fence = VK_NULL_HANDLE;
                try
                {
                ensure_targets(width, height);

                const SushiEngine::Geometry::ThumbnailCamera camera =
                    SushiEngine::Geometry::three_quarter_camera_for_bounds(
                        bounds, static_cast<float>(width) / static_cast<float>(height));
                Matrix4 view_projection = mul(camera.projection, camera.view);

                VkCommandBufferAllocateInfo command_alloc{};
                command_alloc.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
                command_alloc.commandPool = command_pool_;
                command_alloc.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
                command_alloc.commandBufferCount = 1;
                Vulkan::check(
                    vkAllocateCommandBuffers(device_.device(), &command_alloc, &command),
                    "vkAllocateCommandBuffers(prefab thumbnail)");

                VkCommandBufferBeginInfo begin{};
                begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
                begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
                Vulkan::check(vkBeginCommandBuffer(command, &begin),
                              "vkBeginCommandBuffer(prefab thumbnail)");

                VkImageMemoryBarrier2 to_attachment[2]{};
                to_attachment[0].sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
                to_attachment[0].srcStageMask = VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT;
                to_attachment[0].dstStageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;
                to_attachment[0].dstAccessMask = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT;
                to_attachment[0].oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
                to_attachment[0].newLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
                to_attachment[0].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                to_attachment[0].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                to_attachment[0].image = color_image_;
                to_attachment[0].subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
                to_attachment[1].sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
                to_attachment[1].srcStageMask = VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT;
                to_attachment[1].dstStageMask = VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT;
                to_attachment[1].dstAccessMask =
                    VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
                to_attachment[1].oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
                to_attachment[1].newLayout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
                to_attachment[1].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                to_attachment[1].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                to_attachment[1].image = depth_image_;
                to_attachment[1].subresourceRange = {VK_IMAGE_ASPECT_DEPTH_BIT, 0, 1, 0, 1};
                VkDependencyInfo to_attachment_dependency{};
                to_attachment_dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
                to_attachment_dependency.imageMemoryBarrierCount = 2;
                to_attachment_dependency.pImageMemoryBarriers = to_attachment;
                vkCmdPipelineBarrier2(command, &to_attachment_dependency);

                VkRenderingAttachmentInfo color_attachment{};
                color_attachment.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
                color_attachment.imageView = color_view_;
                color_attachment.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
                color_attachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
                color_attachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
                color_attachment.clearValue.color = {{0.0f, 0.0f, 0.0f, 0.0f}};

                VkRenderingAttachmentInfo depth_attachment{};
                depth_attachment.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
                depth_attachment.imageView = depth_view_;
                depth_attachment.imageLayout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
                depth_attachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
                depth_attachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
                // Reverse-Z: clears to 0.0, not 1.0.
                depth_attachment.clearValue.depthStencil = {0.0f, 0};

                VkRenderingInfo rendering_info{};
                rendering_info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
                rendering_info.renderArea = {{0, 0}, {width, height}};
                rendering_info.layerCount = 1;
                rendering_info.colorAttachmentCount = 1;
                rendering_info.pColorAttachments = &color_attachment;
                rendering_info.pDepthAttachment = &depth_attachment;
                vkCmdBeginRendering(command, &rendering_info);

                VkViewport viewport{0.0f, 0.0f, static_cast<float>(width),
                                    static_cast<float>(height), 0.0f, 1.0f};
                VkRect2D scissor{{0, 0}, {width, height}};
                vkCmdSetViewport(command, 0, 1, &viewport);
                vkCmdSetScissor(command, 0, 1, &scissor);

                vkCmdBindPipeline(command, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_);
                VkDescriptorSet heap_set = assets_.heap().set();
                vkCmdBindDescriptorSets(command, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                        pipeline_layout_, 1, 1, &heap_set, 0, nullptr);

                for (std::size_t i = 0; i < count; ++i)
                {
                    const PrefabThumbnailDraw& draw = draws[i];
                    if (draw.mesh == INVALID_MESH)
                        continue;

                    const Geometry::Mesh& mesh = assets_.meshes().mesh(draw.mesh);

                    Push push{};
                    write_matrix(draw.model, push.model);
                    write_matrix(view_projection, push.view_projection);
                    push.albedo[0] = static_cast<float>(draw.material.albedo.x);
                    push.albedo[1] = static_cast<float>(draw.material.albedo.y);
                    push.albedo[2] = static_cast<float>(draw.material.albedo.z);
                    push.albedo[3] = draw.material.base_alpha;
                    push.albedo_texture_index = draw.material.albedo_map != INVALID_TEXTURE
                                                    ? static_cast<std::int32_t>(draw.material.albedo_map)
                                                    : -1;
                    vkCmdPushConstants(command, pipeline_layout_,
                                       VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
                                       0, sizeof(Push), &push);

                    VkDeviceSize vertex_offset = 0;
                    vkCmdBindVertexBuffers(command, 0, 1, &mesh.vertices, &vertex_offset);
                    vkCmdBindIndexBuffer(command, mesh.indices, 0, VK_INDEX_TYPE_UINT32);
                    vkCmdDrawIndexed(command, mesh.index_count, 1, 0, 0, 0);
                }

                vkCmdEndRendering(command);

                VkImageMemoryBarrier2 to_transfer_src{};
                to_transfer_src.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
                to_transfer_src.srcStageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;
                to_transfer_src.srcAccessMask = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT;
                to_transfer_src.dstStageMask = VK_PIPELINE_STAGE_2_COPY_BIT;
                to_transfer_src.dstAccessMask = VK_ACCESS_2_TRANSFER_READ_BIT;
                to_transfer_src.oldLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
                to_transfer_src.newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
                to_transfer_src.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                to_transfer_src.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                to_transfer_src.image = color_image_;
                to_transfer_src.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
                VkDependencyInfo to_transfer_src_dependency{};
                to_transfer_src_dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
                to_transfer_src_dependency.imageMemoryBarrierCount = 1;
                to_transfer_src_dependency.pImageMemoryBarriers = &to_transfer_src;
                vkCmdPipelineBarrier2(command, &to_transfer_src_dependency);

                const VkDeviceSize readback_size = VkDeviceSize(width) * height * 4;
                VkBufferCreateInfo readback_info{};
                readback_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
                readback_info.size = readback_size;
                readback_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
                VmaAllocationCreateInfo readback_alloc_info{};
                readback_alloc_info.usage = VMA_MEMORY_USAGE_AUTO;
                readback_alloc_info.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT |
                                            VMA_ALLOCATION_CREATE_MAPPED_BIT;
                VmaAllocationInfo readback_mapped{};
                Vulkan::check(vmaCreateBuffer(device_.allocator(), &readback_info,
                                              &readback_alloc_info, &readback_buffer,
                                              &readback_allocation, &readback_mapped),
                              "vmaCreateBuffer(prefab thumbnail readback)");

                VkBufferImageCopy copy{};
                copy.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                copy.imageSubresource.layerCount = 1;
                copy.imageExtent = {width, height, 1};
                vkCmdCopyImageToBuffer(command, color_image_, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                                       readback_buffer, 1, &copy);

                Vulkan::check(vkEndCommandBuffer(command),
                              "vkEndCommandBuffer(prefab thumbnail)");

                VkFenceCreateInfo fence_info{};
                fence_info.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
                Vulkan::check(vkCreateFence(device_.device(), &fence_info, nullptr, &fence),
                              "vkCreateFence(prefab thumbnail)");

                VkCommandBufferSubmitInfo command_submit{};
                command_submit.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO;
                command_submit.commandBuffer = command;
                VkSubmitInfo2 submit{};
                submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO_2;
                submit.commandBufferInfoCount = 1;
                submit.pCommandBufferInfos = &command_submit;
                Vulkan::check(
                    vkQueueSubmit2(device_.graphics_queue(), 1, &submit, fence),
                    "vkQueueSubmit2(prefab thumbnail)");
                vkWaitForFences(device_.device(), 1, &fence, VK_TRUE, UINT64_MAX);

                vmaInvalidateAllocation(device_.allocator(), readback_allocation, 0,
                                        readback_size);
                out_image.width = width;
                out_image.height = height;
                out_image.rgba.resize(static_cast<std::size_t>(readback_size));
                std::memcpy(out_image.rgba.data(), readback_mapped.pMappedData,
                           static_cast<std::size_t>(readback_size));

                vkDestroyFence(device_.device(), fence, nullptr);
                vkFreeCommandBuffers(device_.device(), command_pool_, 1, &command);
                vmaDestroyBuffer(device_.allocator(), readback_buffer, readback_allocation);
                return true;
                }
                catch (const std::exception&)
                {
                    // Clean up whatever of the command buffer / readback buffer / fence was
                    // successfully created before the throw point, mirroring the defensive
                    // cleanup pattern create_pipeline() above already uses for its own throw
                    // paths, and Phase 3a's VulkanMeshThumbnailRenderer::render_thumbnail's own
                    // catch block exactly.
                    if (fence != VK_NULL_HANDLE)
                        vkDestroyFence(device_.device(), fence, nullptr);
                    if (readback_buffer != VK_NULL_HANDLE)
                        vmaDestroyBuffer(device_.allocator(), readback_buffer, readback_allocation);
                    if (command != VK_NULL_HANDLE)
                        vkFreeCommandBuffers(device_.device(), command_pool_, 1, &command);
                    return false;
                }
            }
        } // namespace Vulkan
    } // namespace Render
} // namespace SushiEngine
