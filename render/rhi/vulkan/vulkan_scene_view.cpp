/**************************************************************************/
/* vulkan_scene_view.cpp                                                  */
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

#include "vulkan_scene_view.hpp"

#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include "mesh.vert.h"
#include "mesh.frag.h"
#include "line.frag.h"

namespace SushiEngine
{
    namespace render
    {
        namespace vulkan
        {
            namespace
            {
                /** @brief Interleaved position + normal, the scene view's only vertex format. */
                struct Vertex
                {
                    float position[3];
                    float normal[3];
                };

                /** @brief Per-draw push constant: MVP plus the normal basis with colour in w. */
                struct MeshPush
                {
                    Mat4 mvp;
                    float n0[4];
                    float n1[4];
                    float n2[4];
                };

                void check(VkResult result, const char* what)
                {
                    if (result != VK_SUCCESS)
                        throw std::runtime_error(std::string("SushiEngine: ") + what +
                                                 " failed (VkResult " +
                                                 std::to_string(static_cast<int>(result)) + ")");
                }

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

                void transition(VkCommandBuffer cmd, VkImage image, VkImageAspectFlags aspect,
                                VkImageLayout old_layout, VkImageLayout new_layout,
                                VkPipelineStageFlags2 src_stage, VkPipelineStageFlags2 dst_stage,
                                VkAccessFlags2 src_access, VkAccessFlags2 dst_access)
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
                    barrier.subresourceRange.aspectMask = aspect;
                    barrier.subresourceRange.levelCount = 1;
                    barrier.subresourceRange.layerCount = 1;

                    VkDependencyInfo dependency{};
                    dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
                    dependency.imageMemoryBarrierCount = 1;
                    dependency.pImageMemoryBarriers = &barrier;
                    vkCmdPipelineBarrier2(cmd, &dependency);
                }

                /** @brief Fills a push constant from a model matrix and colour. */
                MeshPush make_push(const Mat4& view_projection, const Mat4& model, const Vec3& color)
                {
                    MeshPush push;
                    push.mvp = mul(view_projection, model);
                    push.n0[0] = model.m[0]; push.n0[1] = model.m[1]; push.n0[2] = model.m[2];
                    push.n1[0] = model.m[4]; push.n1[1] = model.m[5]; push.n1[2] = model.m[6];
                    push.n2[0] = model.m[8]; push.n2[1] = model.m[9]; push.n2[2] = model.m[10];
                    push.n0[3] = color.x;
                    push.n1[3] = color.y;
                    push.n2[3] = color.z;
                    return push;
                }
            } // namespace

            VulkanSceneView::VulkanSceneView(VulkanDevice& device) : device_(device)
            {
                create_sampler();
                create_pipelines();
                create_geometry();
                create_targets();
            }

            VulkanSceneView::~VulkanSceneView()
            {
                vkDeviceWaitIdle(device_.device());
                destroy_targets();
                destroy_geometry();
                destroy_pipelines();
                if (sampler_ != VK_NULL_HANDLE)
                    vkDestroySampler(device_.device(), sampler_, nullptr);
            }

            void VulkanSceneView::create_sampler()
            {
                VkSamplerCreateInfo info{};
                info.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
                info.magFilter = VK_FILTER_LINEAR;
                info.minFilter = VK_FILTER_LINEAR;
                info.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
                info.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
                info.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
                check(vkCreateSampler(device_.device(), &info, nullptr, &sampler_),
                      "vkCreateSampler");
            }

            void VulkanSceneView::create_pipelines()
            {
                VkPushConstantRange range{};
                range.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
                range.offset = 0;
                range.size = sizeof(MeshPush);

                VkPipelineLayoutCreateInfo layout_info{};
                layout_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
                layout_info.pushConstantRangeCount = 1;
                layout_info.pPushConstantRanges = &range;
                check(vkCreatePipelineLayout(device_.device(), &layout_info, nullptr, &layout_),
                      "vkCreatePipelineLayout");

                VkShaderModule vertex_module = make_shader(device_.device(), shaders::mesh_vert_spv,
                                                           shaders::mesh_vert_spv_word_count);
                VkShaderModule mesh_fragment = make_shader(device_.device(), shaders::mesh_frag_spv,
                                                           shaders::mesh_frag_spv_word_count);
                VkShaderModule line_fragment = make_shader(device_.device(), shaders::line_frag_spv,
                                                           shaders::line_frag_spv_word_count);

                VkVertexInputBindingDescription binding{};
                binding.binding = 0;
                binding.stride = sizeof(Vertex);
                binding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

                VkVertexInputAttributeDescription attributes[2]{};
                attributes[0].location = 0;
                attributes[0].format = VK_FORMAT_R32G32B32_SFLOAT;
                attributes[0].offset = offsetof(Vertex, position);
                attributes[1].location = 1;
                attributes[1].format = VK_FORMAT_R32G32B32_SFLOAT;
                attributes[1].offset = offsetof(Vertex, normal);

                VkPipelineVertexInputStateCreateInfo vertex_input{};
                vertex_input.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
                vertex_input.vertexBindingDescriptionCount = 1;
                vertex_input.pVertexBindingDescriptions = &binding;
                vertex_input.vertexAttributeDescriptionCount = 2;
                vertex_input.pVertexAttributeDescriptions = attributes;

                VkPipelineViewportStateCreateInfo viewport_state{};
                viewport_state.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
                viewport_state.viewportCount = 1;
                viewport_state.scissorCount = 1;

                VkPipelineMultisampleStateCreateInfo multisample{};
                multisample.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
                multisample.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

                VkPipelineColorBlendAttachmentState blend_attachment{};
                blend_attachment.colorWriteMask =
                    VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
                    VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;

                VkPipelineColorBlendStateCreateInfo blend{};
                blend.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
                blend.attachmentCount = 1;
                blend.pAttachments = &blend_attachment;

                VkPipelineDepthStencilStateCreateInfo depth{};
                depth.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
                depth.depthTestEnable = VK_TRUE;
                depth.depthWriteEnable = VK_TRUE;
                depth.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;

                const VkDynamicState dynamic_states[2] = {VK_DYNAMIC_STATE_VIEWPORT,
                                                          VK_DYNAMIC_STATE_SCISSOR};
                VkPipelineDynamicStateCreateInfo dynamic{};
                dynamic.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
                dynamic.dynamicStateCount = 2;
                dynamic.pDynamicStates = dynamic_states;

                VkFormat color_format = COLOR_FORMAT;
                VkPipelineRenderingCreateInfo rendering_info{};
                rendering_info.sType = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO;
                rendering_info.colorAttachmentCount = 1;
                rendering_info.pColorAttachmentFormats = &color_format;
                rendering_info.depthAttachmentFormat = DEPTH_FORMAT;

                // The two pipelines differ only in topology, cull mode, and fragment
                // shader; everything else is shared, so build them from one template.
                auto build = [&](VkPrimitiveTopology topology, VkCullModeFlags cull,
                                 VkShaderModule fragment) -> VkPipeline
                {
                    VkPipelineShaderStageCreateInfo stages[2]{};
                    stages[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
                    stages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
                    stages[0].module = vertex_module;
                    stages[0].pName = "main";
                    stages[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
                    stages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
                    stages[1].module = fragment;
                    stages[1].pName = "main";

                    VkPipelineInputAssemblyStateCreateInfo input_assembly{};
                    input_assembly.sType =
                        VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
                    input_assembly.topology = topology;

                    VkPipelineRasterizationStateCreateInfo raster{};
                    raster.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
                    raster.polygonMode = VK_POLYGON_MODE_FILL;
                    raster.cullMode = cull;
                    raster.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
                    raster.lineWidth = 1.0f;

                    VkGraphicsPipelineCreateInfo info{};
                    info.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
                    info.pNext = &rendering_info;
                    info.stageCount = 2;
                    info.pStages = stages;
                    info.pVertexInputState = &vertex_input;
                    info.pInputAssemblyState = &input_assembly;
                    info.pViewportState = &viewport_state;
                    info.pRasterizationState = &raster;
                    info.pMultisampleState = &multisample;
                    info.pDepthStencilState = &depth;
                    info.pColorBlendState = &blend;
                    info.pDynamicState = &dynamic;
                    info.layout = layout_;

                    VkPipeline pipeline = VK_NULL_HANDLE;
                    check(vkCreateGraphicsPipelines(device_.device(), VK_NULL_HANDLE, 1, &info,
                                                    nullptr, &pipeline),
                          "vkCreateGraphicsPipelines");
                    return pipeline;
                };

                mesh_pipeline_ = build(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST,
                                       VK_CULL_MODE_NONE, mesh_fragment);
                line_pipeline_ = build(VK_PRIMITIVE_TOPOLOGY_LINE_LIST,
                                       VK_CULL_MODE_NONE, line_fragment);

                vkDestroyShaderModule(device_.device(), line_fragment, nullptr);
                vkDestroyShaderModule(device_.device(), mesh_fragment, nullptr);
                vkDestroyShaderModule(device_.device(), vertex_module, nullptr);
            }

            void VulkanSceneView::destroy_pipelines()
            {
                if (line_pipeline_ != VK_NULL_HANDLE)
                    vkDestroyPipeline(device_.device(), line_pipeline_, nullptr);
                if (mesh_pipeline_ != VK_NULL_HANDLE)
                    vkDestroyPipeline(device_.device(), mesh_pipeline_, nullptr);
                if (layout_ != VK_NULL_HANDLE)
                    vkDestroyPipelineLayout(device_.device(), layout_, nullptr);
            }

            void VulkanSceneView::create_geometry()
            {
                // A host-visible buffer is enough for this small, static geometry and
                // avoids a staging copy.
                auto upload = [&](const void* data, VkDeviceSize bytes, VkBufferUsageFlags usage,
                                  std::uint32_t count) -> Buffer
                {
                    VkBufferCreateInfo buffer_info{};
                    buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
                    buffer_info.size = bytes;
                    buffer_info.usage = usage;

                    VmaAllocationCreateInfo alloc{};
                    alloc.usage = VMA_MEMORY_USAGE_AUTO;
                    alloc.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
                                  VMA_ALLOCATION_CREATE_MAPPED_BIT;

                    Buffer result;
                    result.count = count;
                    VmaAllocationInfo info{};
                    check(vmaCreateBuffer(device_.allocator(), &buffer_info, &alloc,
                                          &result.buffer, &result.allocation, &info),
                          "vmaCreateBuffer");
                    std::memcpy(info.pMappedData, data, static_cast<std::size_t>(bytes));
                    return result;
                };

                // Unit cube centred on the origin, per-face normals for flat shading.
                const float h = 0.5f;
                const Vertex cube[] = {
                    {{-h, -h, h}, {0, 0, 1}},  {{h, -h, h}, {0, 0, 1}},
                    {{h, h, h}, {0, 0, 1}},    {{-h, h, h}, {0, 0, 1}},
                    {{h, -h, -h}, {0, 0, -1}}, {{-h, -h, -h}, {0, 0, -1}},
                    {{-h, h, -h}, {0, 0, -1}}, {{h, h, -h}, {0, 0, -1}},
                    {{h, -h, h}, {1, 0, 0}},   {{h, -h, -h}, {1, 0, 0}},
                    {{h, h, -h}, {1, 0, 0}},   {{h, h, h}, {1, 0, 0}},
                    {{-h, -h, -h}, {-1, 0, 0}},{{-h, -h, h}, {-1, 0, 0}},
                    {{-h, h, h}, {-1, 0, 0}},  {{-h, h, -h}, {-1, 0, 0}},
                    {{-h, h, h}, {0, 1, 0}},   {{h, h, h}, {0, 1, 0}},
                    {{h, h, -h}, {0, 1, 0}},   {{-h, h, -h}, {0, 1, 0}},
                    {{-h, -h, -h}, {0, -1, 0}},{{h, -h, -h}, {0, -1, 0}},
                    {{h, -h, h}, {0, -1, 0}},  {{-h, -h, h}, {0, -1, 0}},
                };
                std::uint16_t indices[36];
                for (std::uint16_t face = 0; face < 6; ++face)
                {
                    const std::uint16_t base = static_cast<std::uint16_t>(face * 4);
                    const std::uint16_t offsets[6] = {0, 1, 2, 2, 3, 0};
                    for (int i = 0; i < 6; ++i)
                        indices[face * 6 + i] = static_cast<std::uint16_t>(base + offsets[i]);
                }

                // Ground grid on the XZ plane; the normal is unused by the line shader.
                std::vector<Vertex> grid;
                const int extent = 10;
                const float span = static_cast<float>(extent);
                for (int i = -extent; i <= extent; ++i)
                {
                    const float t = static_cast<float>(i);
                    grid.push_back({{-span, 0, t}, {0, 1, 0}});
                    grid.push_back({{span, 0, t}, {0, 1, 0}});
                    grid.push_back({{t, 0, -span}, {0, 1, 0}});
                    grid.push_back({{t, 0, span}, {0, 1, 0}});
                }

                cube_vertices_ = upload(cube, sizeof(cube), VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, 24);
                cube_indices_ = upload(indices, sizeof(indices), VK_BUFFER_USAGE_INDEX_BUFFER_BIT, 36);
                grid_vertices_ = upload(grid.data(), grid.size() * sizeof(Vertex),
                                        VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
                                        static_cast<std::uint32_t>(grid.size()));
            }

            void VulkanSceneView::destroy_geometry()
            {
                for (Buffer* buffer : {&cube_vertices_, &cube_indices_, &grid_vertices_})
                    if (buffer->buffer != VK_NULL_HANDLE)
                        vmaDestroyBuffer(device_.allocator(), buffer->buffer, buffer->allocation);
            }

            void VulkanSceneView::create_targets()
            {
                for (Slot& slot : slots_)
                {
                    VkImageCreateInfo color_info{};
                    color_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
                    color_info.imageType = VK_IMAGE_TYPE_2D;
                    color_info.format = COLOR_FORMAT;
                    color_info.extent = {width_, height_, 1};
                    color_info.mipLevels = 1;
                    color_info.arrayLayers = 1;
                    color_info.samples = VK_SAMPLE_COUNT_1_BIT;
                    color_info.tiling = VK_IMAGE_TILING_OPTIMAL;
                    color_info.usage =
                        VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;

                    VmaAllocationCreateInfo image_alloc{};
                    image_alloc.usage = VMA_MEMORY_USAGE_AUTO;
                    check(vmaCreateImage(device_.allocator(), &color_info, &image_alloc,
                                         &slot.color, &slot.color_allocation, nullptr),
                          "vmaCreateImage(color)");

                    VkImageCreateInfo depth_info = color_info;
                    depth_info.format = DEPTH_FORMAT;
                    depth_info.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
                    check(vmaCreateImage(device_.allocator(), &depth_info, &image_alloc,
                                         &slot.depth, &slot.depth_allocation, nullptr),
                          "vmaCreateImage(depth)");

                    VkImageViewCreateInfo color_view{};
                    color_view.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
                    color_view.image = slot.color;
                    color_view.viewType = VK_IMAGE_VIEW_TYPE_2D;
                    color_view.format = COLOR_FORMAT;
                    color_view.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                    color_view.subresourceRange.levelCount = 1;
                    color_view.subresourceRange.layerCount = 1;
                    check(vkCreateImageView(device_.device(), &color_view, nullptr,
                                            &slot.color_view),
                          "vkCreateImageView(color)");

                    VkImageViewCreateInfo depth_view = color_view;
                    depth_view.image = slot.depth;
                    depth_view.format = DEPTH_FORMAT;
                    depth_view.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
                    check(vkCreateImageView(device_.device(), &depth_view, nullptr,
                                            &slot.depth_view),
                          "vkCreateImageView(depth)");

                    VkCommandPoolCreateInfo pool_info{};
                    pool_info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
                    pool_info.flags = VK_COMMAND_POOL_CREATE_TRANSIENT_BIT;
                    pool_info.queueFamilyIndex = device_.graphics_queue_family();
                    check(vkCreateCommandPool(device_.device(), &pool_info, nullptr, &slot.pool),
                          "vkCreateCommandPool");

                    VkCommandBufferAllocateInfo cmd_info{};
                    cmd_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
                    cmd_info.commandPool = slot.pool;
                    cmd_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
                    cmd_info.commandBufferCount = 1;
                    check(vkAllocateCommandBuffers(device_.device(), &cmd_info, &slot.cmd),
                          "vkAllocateCommandBuffers");

                    VkFenceCreateInfo fence_info{};
                    fence_info.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
                    check(vkCreateFence(device_.device(), &fence_info, nullptr, &slot.fence),
                          "vkCreateFence");
                    slot.ever_rendered = false;
                }
            }

            void VulkanSceneView::destroy_targets()
            {
                for (Slot& slot : slots_)
                {
                    if (slot.fence != VK_NULL_HANDLE)
                        vkDestroyFence(device_.device(), slot.fence, nullptr);
                    if (slot.pool != VK_NULL_HANDLE)
                        vkDestroyCommandPool(device_.device(), slot.pool, nullptr);
                    if (slot.depth_view != VK_NULL_HANDLE)
                        vkDestroyImageView(device_.device(), slot.depth_view, nullptr);
                    if (slot.depth != VK_NULL_HANDLE)
                        vmaDestroyImage(device_.allocator(), slot.depth, slot.depth_allocation);
                    if (slot.color_view != VK_NULL_HANDLE)
                        vkDestroyImageView(device_.device(), slot.color_view, nullptr);
                    if (slot.color != VK_NULL_HANDLE)
                        vmaDestroyImage(device_.allocator(), slot.color, slot.color_allocation);
                    slot = Slot{};
                }
            }

            void VulkanSceneView::resize(std::uint32_t width, std::uint32_t height)
            {
                const std::uint32_t new_width = width < 1 ? 1 : width;
                const std::uint32_t new_height = height < 1 ? 1 : height;
                if (new_width == width_ && new_height == height_)
                    return;
                vkDeviceWaitIdle(device_.device());
                destroy_targets();
                width_ = new_width;
                height_ = new_height;
                create_targets();
            }

            SceneViewTexture VulkanSceneView::texture(std::uint32_t slot) const noexcept
            {
                SceneViewTexture texture;
                texture.sampler = sampler_;
                texture.image_view = slots_[slot].color_view;
                return texture;
            }

            void VulkanSceneView::render(const CameraView& camera, const MeshInstance* instances,
                                         std::size_t count)
            {
                const std::uint32_t index = frame_counter_ % SLOTS;
                ++frame_counter_;
                Slot& slot = slots_[index];

                if (slot.ever_rendered)
                    check(vkWaitForFences(device_.device(), 1, &slot.fence, VK_TRUE, UINT64_MAX),
                          "vkWaitForFences");
                check(vkResetFences(device_.device(), 1, &slot.fence), "vkResetFences");
                check(vkResetCommandPool(device_.device(), slot.pool, 0), "vkResetCommandPool");

                VkCommandBufferBeginInfo begin{};
                begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
                begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
                check(vkBeginCommandBuffer(slot.cmd, &begin), "vkBeginCommandBuffer");

                const VkImageLayout color_from = slot.ever_rendered
                                                     ? VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL
                                                     : VK_IMAGE_LAYOUT_UNDEFINED;
                const VkPipelineStageFlags2 color_src_stage =
                    slot.ever_rendered ? VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT
                                       : VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT;
                const VkAccessFlags2 color_src_access =
                    slot.ever_rendered ? VK_ACCESS_2_SHADER_SAMPLED_READ_BIT : 0;
                transition(slot.cmd, slot.color, VK_IMAGE_ASPECT_COLOR_BIT, color_from,
                           VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, color_src_stage,
                           VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, color_src_access,
                           VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT);
                transition(slot.cmd, slot.depth, VK_IMAGE_ASPECT_DEPTH_BIT,
                           VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL,
                           VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,
                           VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT, 0,
                           VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT);

                VkRenderingAttachmentInfo color_attachment{};
                color_attachment.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
                color_attachment.imageView = slot.color_view;
                color_attachment.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
                color_attachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
                color_attachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
                color_attachment.clearValue.color = {{0.05f, 0.06f, 0.08f, 1.0f}};

                VkRenderingAttachmentInfo depth_attachment{};
                depth_attachment.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
                depth_attachment.imageView = slot.depth_view;
                depth_attachment.imageLayout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
                depth_attachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
                depth_attachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
                depth_attachment.clearValue.depthStencil = {1.0f, 0};

                VkRenderingInfo rendering{};
                rendering.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
                rendering.renderArea.extent = {width_, height_};
                rendering.layerCount = 1;
                rendering.colorAttachmentCount = 1;
                rendering.pColorAttachments = &color_attachment;
                rendering.pDepthAttachment = &depth_attachment;
                vkCmdBeginRendering(slot.cmd, &rendering);

                VkViewport viewport{};
                viewport.width = static_cast<float>(width_);
                viewport.height = static_cast<float>(height_);
                viewport.maxDepth = 1.0f;
                vkCmdSetViewport(slot.cmd, 0, 1, &viewport);
                VkRect2D scissor{};
                scissor.extent = {width_, height_};
                vkCmdSetScissor(slot.cmd, 0, 1, &scissor);

                const Mat4 view_projection = mul(camera.projection, camera.view);
                const VkDeviceSize zero_offset = 0;

                // Ground grid: a single flat-coloured line draw.
                vkCmdBindPipeline(slot.cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, line_pipeline_);
                vkCmdBindVertexBuffers(slot.cmd, 0, 1, &grid_vertices_.buffer, &zero_offset);
                const MeshPush grid_push = make_push(view_projection, Mat4{}, Vec3{0.32f, 0.33f, 0.40f});
                vkCmdPushConstants(slot.cmd, layout_, VK_SHADER_STAGE_VERTEX_BIT, 0,
                                   sizeof(MeshPush), &grid_push);
                vkCmdDraw(slot.cmd, grid_vertices_.count, 1, 0, 0);

                // Instanced lit cubes.
                vkCmdBindPipeline(slot.cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, mesh_pipeline_);
                vkCmdBindVertexBuffers(slot.cmd, 0, 1, &cube_vertices_.buffer, &zero_offset);
                vkCmdBindIndexBuffer(slot.cmd, cube_indices_.buffer, 0, VK_INDEX_TYPE_UINT16);
                for (std::size_t i = 0; i < count; ++i)
                {
                    const MeshPush push =
                        make_push(view_projection, instances[i].model, instances[i].color);
                    vkCmdPushConstants(slot.cmd, layout_, VK_SHADER_STAGE_VERTEX_BIT, 0,
                                       sizeof(MeshPush), &push);
                    vkCmdDrawIndexed(slot.cmd, cube_indices_.count, 1, 0, 0, 0);
                }

                vkCmdEndRendering(slot.cmd);

                transition(slot.cmd, slot.color, VK_IMAGE_ASPECT_COLOR_BIT,
                           VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                           VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                           VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                           VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                           VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT, VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);

                check(vkEndCommandBuffer(slot.cmd), "vkEndCommandBuffer");

                VkSubmitInfo submit{};
                submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
                submit.commandBufferCount = 1;
                submit.pCommandBuffers = &slot.cmd;
                check(vkQueueSubmit(device_.graphics_queue(), 1, &submit, slot.fence),
                      "vkQueueSubmit");

                slot.ever_rendered = true;
                current_slot_ = index;
            }
        } // namespace vulkan
    } // namespace render
} // namespace SushiEngine
