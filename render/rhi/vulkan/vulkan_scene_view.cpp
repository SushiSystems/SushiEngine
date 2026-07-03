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

#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include "mesh.vert.h"
#include "mesh.frag.h"
#include "line.frag.h"
#include "outline.frag.h"
#include "outline.vert.h"

namespace SushiEngine
{
    namespace Render
    {
        namespace Vulkan
        {
            namespace
            {
                /** @brief Interleaved position + normal, the scene view's only vertex format. */
                struct Vertex
                {
                    float position[3];
                    float normal[3];
                };

                /**
                 * @brief Per-draw push constant: MVP, the normal basis with colour in w,
                 *        the draw's picking id, and the currently selected id for highlight.
                 *
                 * 120 bytes, within the 128-byte push-constant floor every Vulkan device
                 * guarantees. The MVP is an explicit @c float[16] rather than an engine
                 * @c Mat4: GPU data is 32-bit regardless of the engine's Scalar precision,
                 * so a double-precision build narrows to float exactly here at the upload
                 * boundary and the shader's @c mat4 layout is unchanged.
                 */
                struct MeshPush
                {
                    float mvp[16];
                    float n0[4];
                    float n1[4];
                    float n2[4];
                    std::uint32_t entity_id;
                    std::uint32_t selected;
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

                /** @brief Fills a push constant from a model matrix, colour, and pick ids. */
                MeshPush make_push(const Mat4& view_projection, const Mat4& model, const Vector3& color,
                                   std::uint32_t entity_id, std::uint32_t selected_id)
                {
                    MeshPush push;
                    const Mat4 mvp = mul(view_projection, model);
                    for (int i = 0; i < 16; ++i)
                        push.mvp[i] = static_cast<float>(mvp.m[i]);
                    push.n0[0] = static_cast<float>(model.m[0]);
                    push.n0[1] = static_cast<float>(model.m[1]);
                    push.n0[2] = static_cast<float>(model.m[2]);
                    push.n1[0] = static_cast<float>(model.m[4]);
                    push.n1[1] = static_cast<float>(model.m[5]);
                    push.n1[2] = static_cast<float>(model.m[6]);
                    push.n2[0] = static_cast<float>(model.m[8]);
                    push.n2[1] = static_cast<float>(model.m[9]);
                    push.n2[2] = static_cast<float>(model.m[10]);
                    push.n0[3] = static_cast<float>(color.x);
                    push.n1[3] = static_cast<float>(color.y);
                    push.n2[3] = static_cast<float>(color.z);
                    push.entity_id = entity_id;
                    push.selected = selected_id;
                    return push;
                }

                /**
                 * @brief The local scale that maps the renderer's unit mesh onto an
                 * instance's authored shape params.
                 *
                 * Every unit mesh (see create_geometry) is built at the {0.5} scale a
                 * default `ShapeParams` describes, so this is exactly `2 * params` for
                 * Box (half-extents) and Cylinder (radius, half-height, radius), and a
                 * uniform `2 * params.x` for Sphere (radius).
                 */
                Mat4 shape_scale(MeshKind kind, const Vector3& params) noexcept
                {
                    switch (kind)
                    {
                        case MeshKind::Sphere:
                            return scaling(Vector3{params.x * 2, params.x * 2, params.x * 2});
                        case MeshKind::Cylinder:
                            return scaling(Vector3{params.x * 2, params.y * 2, params.x * 2});
                        case MeshKind::Box:
                        default:
                            return scaling(Vector3{params.x * 2, params.y * 2, params.z * 2});
                    }
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
                range.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
                range.offset = 0;
                range.size = sizeof(MeshPush);

                VkPipelineLayoutCreateInfo layout_info{};
                layout_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
                layout_info.pushConstantRangeCount = 1;
                layout_info.pPushConstantRanges = &range;
                check(vkCreatePipelineLayout(device_.device(), &layout_info, nullptr, &layout_),
                      "vkCreatePipelineLayout");

                VkShaderModule vertex_module = make_shader(device_.device(), Shaders::mesh_vert_spv,
                                                           Shaders::mesh_vert_spv_word_count);
                VkShaderModule mesh_fragment = make_shader(device_.device(), Shaders::mesh_frag_spv,
                                                           Shaders::mesh_frag_spv_word_count);
                VkShaderModule line_fragment = make_shader(device_.device(), Shaders::line_frag_spv,
                                                           Shaders::line_frag_spv_word_count);

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

                // Two colour attachments: the shaded image and the R32_UINT id target.
                // Neither blends; the id attachment must not, as it is an integer format.
                VkPipelineColorBlendAttachmentState blend_attachments[2]{};
                for (VkPipelineColorBlendAttachmentState& attachment : blend_attachments)
                    attachment.colorWriteMask =
                        VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
                        VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;

                VkPipelineColorBlendStateCreateInfo blend{};
                blend.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
                blend.attachmentCount = 2;
                blend.pAttachments = blend_attachments;

                VkPipelineDepthStencilStateCreateInfo depth{};
                depth.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
                depth.depthTestEnable = VK_TRUE;
                depth.depthWriteEnable = VK_TRUE;
                depth.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
                depth.stencilTestEnable = VK_TRUE;
                depth.front.compareOp = VK_COMPARE_OP_ALWAYS;
                depth.front.passOp = VK_STENCIL_OP_REPLACE;
                depth.front.failOp = VK_STENCIL_OP_KEEP;
                depth.front.depthFailOp = VK_STENCIL_OP_KEEP;
                depth.front.compareMask = 0xFF;
                depth.front.writeMask = 0xFF;
                depth.front.reference = 0;
                depth.back = depth.front;

                const VkDynamicState dynamic_states[3] = {VK_DYNAMIC_STATE_VIEWPORT,
                                                          VK_DYNAMIC_STATE_SCISSOR,
                                                          VK_DYNAMIC_STATE_STENCIL_REFERENCE};
                VkPipelineDynamicStateCreateInfo dynamic{};
                dynamic.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
                dynamic.dynamicStateCount = 3;
                dynamic.pDynamicStates = dynamic_states;

                const VkFormat color_formats[2] = {COLOR_FORMAT, ID_FORMAT};
                VkPipelineRenderingCreateInfo rendering_info{};
                rendering_info.sType = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO;
                rendering_info.colorAttachmentCount = 2;
                rendering_info.pColorAttachmentFormats = color_formats;
                rendering_info.depthAttachmentFormat = DEPTH_FORMAT;
                rendering_info.stencilAttachmentFormat = DEPTH_FORMAT;

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

                VkShaderModule outline_vertex = make_shader(device_.device(),
                                                            Shaders::outline_vert_spv,
                                                            Shaders::outline_vert_spv_word_count);
                VkShaderModule outline_fragment = make_shader(device_.device(),
                                                             Shaders::outline_frag_spv,
                                                             Shaders::outline_frag_spv_word_count);

                auto build_outline = [&](VkShaderModule fragment) -> VkPipeline
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
                    input_assembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

                    VkPipelineRasterizationStateCreateInfo raster{};
                    raster.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
                    raster.polygonMode = VK_POLYGON_MODE_LINE;
                    raster.cullMode = VK_CULL_MODE_NONE;
                    raster.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
                    raster.lineWidth = 8.0f;
                    raster.depthBiasEnable = VK_TRUE;
                    raster.depthBiasConstantFactor = -2.0f;
                    raster.depthBiasClamp = 0.0f;
                    raster.depthBiasSlopeFactor = -2.0f;

                    VkPipelineDepthStencilStateCreateInfo depth_outline = depth;
                    depth_outline.depthWriteEnable = VK_FALSE;
                    depth_outline.depthTestEnable = VK_TRUE;
                    depth_outline.stencilTestEnable = VK_TRUE;
                    depth_outline.front.compareOp = VK_COMPARE_OP_NOT_EQUAL;
                    depth_outline.front.passOp = VK_STENCIL_OP_KEEP;
                    depth_outline.front.failOp = VK_STENCIL_OP_KEEP;
                    depth_outline.front.depthFailOp = VK_STENCIL_OP_KEEP;
                    depth_outline.front.reference = 1;
                    depth_outline.back = depth_outline.front;

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
                    info.pDepthStencilState = &depth_outline;
                    info.pColorBlendState = &blend;
                    info.pDynamicState = &dynamic;
                    info.layout = layout_;

                    VkPipeline pipeline = VK_NULL_HANDLE;
                    check(vkCreateGraphicsPipelines(device_.device(), VK_NULL_HANDLE, 1, &info,
                                                    nullptr, &pipeline),
                          "vkCreateGraphicsPipelines(outline)");
                    return pipeline;
                };

                outline_pipeline_ = build_outline(outline_fragment);

                vkDestroyShaderModule(device_.device(), outline_vertex, nullptr);
                vkDestroyShaderModule(device_.device(), outline_fragment, nullptr);
                vkDestroyShaderModule(device_.device(), line_fragment, nullptr);
                vkDestroyShaderModule(device_.device(), mesh_fragment, nullptr);
                vkDestroyShaderModule(device_.device(), vertex_module, nullptr);
            }

            void VulkanSceneView::destroy_pipelines()
            {
                if (outline_pipeline_ != VK_NULL_HANDLE)
                    vkDestroyPipeline(device_.device(), outline_pipeline_, nullptr);
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

                // Unit UV-sphere, radius 0.5, centred on the origin: one ring of
                // vertices per latitude step plus the two poles, smooth-shaded (each
                // vertex's normal is its own direction from the centre).
                constexpr int SPHERE_RINGS = 16;
                constexpr int SPHERE_SEGMENTS = 24;
                std::vector<Vertex> sphere;
                std::vector<std::uint16_t> sphere_index;
                for (int ring = 0; ring <= SPHERE_RINGS; ++ring)
                {
                    const float v = static_cast<float>(ring) / SPHERE_RINGS;
                    const float phi = v * 3.14159265f;
                    for (int segment = 0; segment <= SPHERE_SEGMENTS; ++segment)
                    {
                        const float u = static_cast<float>(segment) / SPHERE_SEGMENTS;
                        const float theta = u * 2.0f * 3.14159265f;
                        const float nx = std::sin(phi) * std::cos(theta);
                        const float ny = std::cos(phi);
                        const float nz = std::sin(phi) * std::sin(theta);
                        sphere.push_back({{nx * 0.5f, ny * 0.5f, nz * 0.5f}, {nx, ny, nz}});
                    }
                }
                for (int ring = 0; ring < SPHERE_RINGS; ++ring)
                {
                    for (int segment = 0; segment < SPHERE_SEGMENTS; ++segment)
                    {
                        const std::uint16_t a =
                            static_cast<std::uint16_t>(ring * (SPHERE_SEGMENTS + 1) + segment);
                        const std::uint16_t b = static_cast<std::uint16_t>(a + SPHERE_SEGMENTS + 1);
                        sphere_index.insert(sphere_index.end(),
                                            {a, b, static_cast<std::uint16_t>(a + 1), b,
                                             static_cast<std::uint16_t>(b + 1),
                                             static_cast<std::uint16_t>(a + 1)});
                    }
                }

                // Unit cylinder: radius 0.5, half-height 0.5, capped, centred on the
                // origin with its axis along Y. Side vertices carry the radial normal;
                // cap vertices are duplicated with the flat +-Y normal.
                constexpr int CYLINDER_SEGMENTS = 24;
                std::vector<Vertex> cylinder;
                std::vector<std::uint16_t> cylinder_index;
                for (int segment = 0; segment <= CYLINDER_SEGMENTS; ++segment)
                {
                    const float theta =
                        static_cast<float>(segment) / CYLINDER_SEGMENTS * 2.0f * 3.14159265f;
                    const float x = std::cos(theta) * 0.5f;
                    const float z = std::sin(theta) * 0.5f;
                    cylinder.push_back({{x, -0.5f, z}, {std::cos(theta), 0, std::sin(theta)}});
                    cylinder.push_back({{x, 0.5f, z}, {std::cos(theta), 0, std::sin(theta)}});
                }
                for (int segment = 0; segment < CYLINDER_SEGMENTS; ++segment)
                {
                    const std::uint16_t a = static_cast<std::uint16_t>(segment * 2);
                    const std::uint16_t b = static_cast<std::uint16_t>(a + 1);
                    const std::uint16_t c = static_cast<std::uint16_t>(a + 2);
                    const std::uint16_t d = static_cast<std::uint16_t>(a + 3);
                    cylinder_index.insert(cylinder_index.end(), {a, c, b, b, c, d});
                }
                const std::uint16_t bottom_center =
                    static_cast<std::uint16_t>(cylinder.size());
                cylinder.push_back({{0, -0.5f, 0}, {0, -1, 0}});
                const std::uint16_t top_center = static_cast<std::uint16_t>(cylinder.size());
                cylinder.push_back({{0, 0.5f, 0}, {0, 1, 0}});
                for (int segment = 0; segment < CYLINDER_SEGMENTS; ++segment)
                {
                    const float theta0 =
                        static_cast<float>(segment) / CYLINDER_SEGMENTS * 2.0f * 3.14159265f;
                    const float theta1 =
                        static_cast<float>(segment + 1) / CYLINDER_SEGMENTS * 2.0f * 3.14159265f;
                    const std::uint16_t bottom0 = static_cast<std::uint16_t>(cylinder.size());
                    cylinder.push_back(
                        {{std::cos(theta0) * 0.5f, -0.5f, std::sin(theta0) * 0.5f}, {0, -1, 0}});
                    const std::uint16_t bottom1 = static_cast<std::uint16_t>(cylinder.size());
                    cylinder.push_back(
                        {{std::cos(theta1) * 0.5f, -0.5f, std::sin(theta1) * 0.5f}, {0, -1, 0}});
                    cylinder_index.insert(cylinder_index.end(), {bottom_center, bottom1, bottom0});

                    const std::uint16_t top0 = static_cast<std::uint16_t>(cylinder.size());
                    cylinder.push_back(
                        {{std::cos(theta0) * 0.5f, 0.5f, std::sin(theta0) * 0.5f}, {0, 1, 0}});
                    const std::uint16_t top1 = static_cast<std::uint16_t>(cylinder.size());
                    cylinder.push_back(
                        {{std::cos(theta1) * 0.5f, 0.5f, std::sin(theta1) * 0.5f}, {0, 1, 0}});
                    cylinder_index.insert(cylinder_index.end(), {top_center, top0, top1});
                }

                sphere_vertices_ = upload(sphere.data(), sphere.size() * sizeof(Vertex),
                                          VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
                                          static_cast<std::uint32_t>(sphere.size()));
                sphere_indices_ = upload(sphere_index.data(),
                                         sphere_index.size() * sizeof(std::uint16_t),
                                         VK_BUFFER_USAGE_INDEX_BUFFER_BIT,
                                         static_cast<std::uint32_t>(sphere_index.size()));
                cylinder_vertices_ = upload(cylinder.data(), cylinder.size() * sizeof(Vertex),
                                           VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
                                           static_cast<std::uint32_t>(cylinder.size()));
                cylinder_indices_ = upload(cylinder_index.data(),
                                          cylinder_index.size() * sizeof(std::uint16_t),
                                          VK_BUFFER_USAGE_INDEX_BUFFER_BIT,
                                          static_cast<std::uint32_t>(cylinder_index.size()));
            }

            void VulkanSceneView::destroy_geometry()
            {
                for (Buffer* buffer : {&cube_vertices_, &cube_indices_, &sphere_vertices_,
                                       &sphere_indices_, &cylinder_vertices_, &cylinder_indices_,
                                       &grid_vertices_, &cloth_vertices_})
                    if (buffer->buffer != VK_NULL_HANDLE)
                        vmaDestroyBuffer(device_.allocator(), buffer->buffer, buffer->allocation);
                cloth_vertices_capacity_ = 0;
                cloth_vertices_mapped_ = nullptr;
            }

            void VulkanSceneView::ensure_cloth_capacity(VkDeviceSize bytes)
            {
                if (bytes <= cloth_vertices_capacity_)
                    return;
                if (cloth_vertices_.buffer != VK_NULL_HANDLE)
                    vmaDestroyBuffer(device_.allocator(), cloth_vertices_.buffer,
                                     cloth_vertices_.allocation);

                VkBufferCreateInfo buffer_info{};
                buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
                buffer_info.size = bytes;
                buffer_info.usage = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;

                VmaAllocationCreateInfo alloc{};
                alloc.usage = VMA_MEMORY_USAGE_AUTO;
                alloc.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
                              VMA_ALLOCATION_CREATE_MAPPED_BIT;

                cloth_vertices_ = Buffer{};
                VmaAllocationInfo info{};
                check(vmaCreateBuffer(device_.allocator(), &buffer_info, &alloc,
                                      &cloth_vertices_.buffer, &cloth_vertices_.allocation, &info),
                      "vmaCreateBuffer(cloth)");
                cloth_vertices_mapped_ = info.pMappedData;
                cloth_vertices_capacity_ = bytes;
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

                    // The id target the mesh pass writes; copied to a host buffer each
                    // frame so a click can read the entity under the cursor.
                    VkImageCreateInfo id_info = color_info;
                    id_info.format = ID_FORMAT;
                    id_info.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT |
                                    VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
                    check(vmaCreateImage(device_.allocator(), &id_info, &image_alloc,
                                         &slot.id, &slot.id_allocation, nullptr),
                          "vmaCreateImage(id)");

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
                    depth_view.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT | VK_IMAGE_ASPECT_STENCIL_BIT;
                    check(vkCreateImageView(device_.device(), &depth_view, nullptr,
                                            &slot.depth_view),
                          "vkCreateImageView(depth)");

                    VkImageViewCreateInfo id_view = color_view;
                    id_view.image = slot.id;
                    id_view.format = ID_FORMAT;
                    check(vkCreateImageView(device_.device(), &id_view, nullptr, &slot.id_view),
                          "vkCreateImageView(id)");

                    // Host-visible readback buffer sized to the id image, filled by a
                    // copy at the end of each render so pick() reads it without a stall.
                    VkBufferCreateInfo readback_info{};
                    readback_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
                    readback_info.size = VkDeviceSize(width_) * height_ * sizeof(std::uint32_t);
                    readback_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;

                    VmaAllocationCreateInfo readback_alloc{};
                    readback_alloc.usage = VMA_MEMORY_USAGE_AUTO_PREFER_HOST;
                    readback_alloc.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT |
                                           VMA_ALLOCATION_CREATE_MAPPED_BIT;
                    VmaAllocationInfo readback_mapped{};
                    check(vmaCreateBuffer(device_.allocator(), &readback_info, &readback_alloc,
                                          &slot.readback, &slot.readback_allocation, &readback_mapped),
                          "vmaCreateBuffer(readback)");
                    slot.readback_mapped = readback_mapped.pMappedData;

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
                    if (slot.readback != VK_NULL_HANDLE)
                        vmaDestroyBuffer(device_.allocator(), slot.readback, slot.readback_allocation);
                    if (slot.id_view != VK_NULL_HANDLE)
                        vkDestroyImageView(device_.device(), slot.id_view, nullptr);
                    if (slot.id != VK_NULL_HANDLE)
                        vmaDestroyImage(device_.allocator(), slot.id, slot.id_allocation);
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
                                         std::size_t count, std::uint32_t selected_id,
                                         const ClothStrandView* strands, std::size_t strand_count)
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
                transition(slot.cmd, slot.depth, VK_IMAGE_ASPECT_DEPTH_BIT | VK_IMAGE_ASPECT_STENCIL_BIT,
                           VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL,
                           VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,
                           VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT, 0,
                           VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT);
                // The id target is rebuilt every frame, so its prior contents are
                // discarded (UNDEFINED) before it is cleared and drawn into.
                transition(slot.cmd, slot.id, VK_IMAGE_ASPECT_COLOR_BIT,
                           VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                           VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,
                           VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, 0,
                           VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT);

                VkRenderingAttachmentInfo color_attachment{};
                color_attachment.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
                color_attachment.imageView = slot.color_view;
                color_attachment.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
                color_attachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
                color_attachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
                color_attachment.clearValue.color = {{0.05f, 0.06f, 0.08f, 1.0f}};

                VkRenderingAttachmentInfo id_attachment{};
                id_attachment.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
                id_attachment.imageView = slot.id_view;
                id_attachment.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
                id_attachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
                id_attachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
                id_attachment.clearValue.color.uint32[0] = NO_PICK;

                const VkRenderingAttachmentInfo color_attachments[2] = {color_attachment,
                                                                        id_attachment};

                VkRenderingAttachmentInfo depth_attachment{};
                depth_attachment.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
                depth_attachment.imageView = slot.depth_view;
                depth_attachment.imageLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
                depth_attachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
                depth_attachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
                depth_attachment.clearValue.depthStencil = {1.0f, 0};

                VkRenderingInfo rendering{};
                rendering.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
                rendering.renderArea.extent = {width_, height_};
                rendering.layerCount = 1;
                rendering.colorAttachmentCount = 2;
                rendering.pColorAttachments = color_attachments;
                rendering.pDepthAttachment = &depth_attachment;
                rendering.pStencilAttachment = &depth_attachment;
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
                const std::uint32_t stages = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
                vkCmdBindPipeline(slot.cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, line_pipeline_);
                vkCmdSetStencilReference(slot.cmd, VK_STENCIL_FACE_FRONT_AND_BACK, 0);
                vkCmdBindVertexBuffers(slot.cmd, 0, 1, &grid_vertices_.buffer, &zero_offset);
                const MeshPush grid_push =
                    make_push(view_projection, Mat4{}, Vector3{0.32f, 0.33f, 0.40f}, NO_PICK, NO_PICK);
                vkCmdPushConstants(slot.cmd, layout_, stages, 0, sizeof(MeshPush), &grid_push);
                vkCmdDraw(slot.cmd, grid_vertices_.count, 1, 0, 0);

                // Lit mesh instances, grouped by which unit mesh they draw with (three
                // small passes over `instances` rather than three buckets built per
                // frame) so each mesh's vertex/index buffers are bound once per group.
                vkCmdBindPipeline(slot.cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, mesh_pipeline_);
                const struct { MeshKind kind; const Buffer* vertices; const Buffer* indices; }
                mesh_groups[] = {
                    {MeshKind::Box, &cube_vertices_, &cube_indices_},
                    {MeshKind::Sphere, &sphere_vertices_, &sphere_indices_},
                    {MeshKind::Cylinder, &cylinder_vertices_, &cylinder_indices_},
                };
                for (const auto& group : mesh_groups)
                {
                    bool bound = false;
                    for (std::size_t i = 0; i < count; ++i)
                    {
                        if (instances[i].kind != group.kind)
                            continue;
                        if (!bound)
                        {
                            vkCmdBindVertexBuffers(slot.cmd, 0, 1, &group.vertices->buffer,
                                                   &zero_offset);
                            vkCmdBindIndexBuffer(slot.cmd, group.indices->buffer, 0,
                                                 VK_INDEX_TYPE_UINT16);
                            bound = true;
                        }
                        const Mat4 scaled_model =
                            mul(instances[i].model, shape_scale(instances[i].kind,
                                                                instances[i].shape_params));
                        const MeshPush push = make_push(view_projection, scaled_model,
                                                        instances[i].color, instances[i].id,
                                                        selected_id);
                        vkCmdSetStencilReference(slot.cmd, VK_STENCIL_FACE_FRONT_AND_BACK,
                                                 (instances[i].id == selected_id) ? 1 : 0);
                        vkCmdPushConstants(slot.cmd, layout_, stages, 0, sizeof(MeshPush), &push);
                        vkCmdDrawIndexed(slot.cmd, group.indices->count, 1, 0, 0, 0);
                    }
                }

                // Outline rendering: render solid scaled shape of the selected object, masked by stencil.
                if (selected_id != NO_PICK)
                {
                    vkCmdBindPipeline(slot.cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                      outline_pipeline_);
                    vkCmdSetStencilReference(slot.cmd, VK_STENCIL_FACE_FRONT_AND_BACK, 1);
                    for (const auto& group : mesh_groups)
                    {
                        bool bound = false;
                        for (std::size_t i = 0; i < count; ++i)
                        {
                            if (instances[i].kind != group.kind || instances[i].id != selected_id)
                                continue;
                            if (!bound)
                            {
                                vkCmdBindVertexBuffers(slot.cmd, 0, 1, &group.vertices->buffer,
                                                       &zero_offset);
                                vkCmdBindIndexBuffer(slot.cmd, group.indices->buffer, 0,
                                                     VK_INDEX_TYPE_UINT16);
                                bound = true;
                            }
                            const Mat4 scaled_model =
                                mul(instances[i].model, shape_scale(instances[i].kind,
                                                                    instances[i].shape_params));
                            const MeshPush push = make_push(view_projection, scaled_model,
                                                            instances[i].color, instances[i].id,
                                                            selected_id);
                            vkCmdPushConstants(slot.cmd, layout_, stages, 0, sizeof(MeshPush), &push);
                            vkCmdDrawIndexed(slot.cmd, group.indices->count, 1, 0, 0, 0);
                        }
                    }
                }

                // Soft-body wireframes: every strand's grid edges (horizontal, then
                // vertical) concatenated into one line-list draw per strand, uploaded
                // to the single host-visible cloth buffer just ahead of recording it.
                if (strand_count > 0)
                {
                    std::size_t total_lines = 0;
                    for (std::size_t s = 0; s < strand_count; ++s)
                        if (strands[s].rows > 0 && strands[s].cols > 0)
                            total_lines += strands[s].rows * (strands[s].cols - 1) +
                                          (strands[s].rows - 1) * strands[s].cols;
                    if (total_lines > 0)
                    {
                        ensure_cloth_capacity(total_lines * 2 * sizeof(Vertex));
                        auto* dst = static_cast<Vertex*>(cloth_vertices_mapped_);
                        std::uint32_t written = 0;
                        for (std::size_t s = 0; s < strand_count; ++s)
                        {
                            const ClothStrandView& strand = strands[s];
                            if (strand.rows == 0 || strand.cols == 0)
                                continue;
                            const auto at = [&](std::uint32_t row, std::uint32_t col)
                            {
                                const Vector3& p = strand.vertices[row * strand.cols + col];
                                return Vertex{{static_cast<float>(p.x), static_cast<float>(p.y),
                                              static_cast<float>(p.z)}, {0, 1, 0}};
                            };
                            for (std::uint32_t row = 0; row < strand.rows; ++row)
                                for (std::uint32_t col = 0; col + 1 < strand.cols; ++col)
                                {
                                    dst[written++] = at(row, col);
                                    dst[written++] = at(row, col + 1);
                                }
                            for (std::uint32_t col = 0; col < strand.cols; ++col)
                                for (std::uint32_t row = 0; row + 1 < strand.rows; ++row)
                                {
                                    dst[written++] = at(row, col);
                                    dst[written++] = at(row + 1, col);
                                }
                        }

                        vkCmdBindPipeline(slot.cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, line_pipeline_);
                        vkCmdBindVertexBuffers(slot.cmd, 0, 1, &cloth_vertices_.buffer, &zero_offset);
                        const MeshPush cloth_push = make_push(view_projection, Mat4{},
                                                              Vector3{0.85f, 0.85f, 0.9f}, NO_PICK,
                                                              NO_PICK);
                        vkCmdPushConstants(slot.cmd, layout_, stages, 0, sizeof(MeshPush),
                                          &cloth_push);
                        vkCmdDraw(slot.cmd, written, 1, 0, 0);
                    }
                }

                vkCmdEndRendering(slot.cmd);

                transition(slot.cmd, slot.color, VK_IMAGE_ASPECT_COLOR_BIT,
                           VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                           VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                           VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                           VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                           VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT, VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);

                // Copy the id target into the host-visible readback buffer so pick()
                // can resolve a click without its own submit.
                transition(slot.cmd, slot.id, VK_IMAGE_ASPECT_COLOR_BIT,
                           VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                           VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                           VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                           VK_PIPELINE_STAGE_2_COPY_BIT,
                           VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT, VK_ACCESS_2_TRANSFER_READ_BIT);

                VkBufferImageCopy copy{};
                copy.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                copy.imageSubresource.layerCount = 1;
                copy.imageExtent = {width_, height_, 1};
                vkCmdCopyImageToBuffer(slot.cmd, slot.id,
                                       VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, slot.readback, 1, &copy);

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

            std::uint32_t VulkanSceneView::pick(std::uint32_t x, std::uint32_t y)
            {
                Slot& slot = slots_[current_slot_];
                if (!slot.ever_rendered || slot.readback_mapped == nullptr ||
                    x >= width_ || y >= height_)
                    return NO_PICK;

                // Ensure the copy that filled the readback buffer has completed before
                // reading it; a click is rare, so the wait is not a hot path.
                check(vkWaitForFences(device_.device(), 1, &slot.fence, VK_TRUE, UINT64_MAX),
                      "vkWaitForFences(pick)");

                const std::uint32_t* pixels = static_cast<const std::uint32_t*>(slot.readback_mapped);
                return pixels[static_cast<std::size_t>(y) * width_ + x];
            }
        } // namespace Vulkan
    } // namespace Render
} // namespace SushiEngine
