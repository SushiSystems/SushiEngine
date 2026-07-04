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
#include <utility>
#include <vector>

#include "mesh.vert.h"
#include "pbr.frag.h"
#include "line.frag.h"
#include "outline.frag.h"
#include "outline.vert.h"
#include "fullscreen.vert.h"
#include "sky.frag.h"
#include "tonemap.frag.h"

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
                 * @brief Per-draw push constant: the model matrix, the PBR material, an
                 *        optional screen-space outline shift, and the pick/selection ids.
                 *
                 * 120 bytes, within the 128-byte push-constant floor every Vulkan device
                 * guarantees. The clip transform is finished in the shader from the model
                 * matrix and the scene block's view/projection, so no MVP is packed here.
                 * Matrices and colours are explicit @c float arrays: GPU data is 32-bit
                 * regardless of the engine's Scalar precision, so a double-precision build
                 * narrows to float exactly here at the upload boundary.
                 */
                struct MeshPush
                {
                    float model[16];
                    float albedo_metallic[4];    // xyz = albedo, w = metallic
                    float emissive_roughness[4]; // xyz = emissive, w = roughness
                    float outline_shift[4];      // xy = screen-space shift, zw spare
                    std::uint32_t entity_id;
                    std::uint32_t selected;
                };

                /**
                 * @brief Per-frame scene uniform block, shared by every pass (std140).
                 *
                 * Mirrors the `SceneBlock` in the shaders exactly: two matrices followed by
                 * fifteen vec4s. Kept as flat @c float arrays so the C++ side never disagrees
                 * with the GLSL std140 packing (every member is 16-byte aligned).
                 */
                struct SceneUbo
                {
                    float view[16];
                    float proj[16];
                    float cam_forward[4];   // xyz = unit forward, w = camera pos x
                    float cam_right[4];     // xyz = right * tan(fovx/2), w = camera pos y
                    float cam_up[4];        // xyz = up * tan(fovy/2), w = camera pos z
                    float planet_center[4]; // xyz = centre relative to camera, w = surface radius
                    float planet_radii[4];  // xyz = ellipsoid semi-axes, w = atmosphere height
                    float sun_dir[4];       // xyz = direction to sun, w = intensity
                    float sun_color[4];     // xyz = colour, w = exposure
                    float ambient[4];       // xyz = ambient radiance
                    float rayleigh[4];      // xyz = per-metre Rayleigh, w = Mie coefficient
                    float scatter[4];       // x = Mie g, y = Rayleigh H, z = Mie H, w = altitude
                    float ground_albedo[4]; // xyz
                    float ocean_color[4];   // xyz
                    float cloud_params[4];  // base_alt, top_alt, coverage, density
                    float star_params[4];   // brightness, density, atmo_enabled, stars_enabled
                    float misc[4];          // near, far, time, clouds_enabled
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

                /** @brief Fills a push constant from a model matrix, material, and pick ids. */
                MeshPush make_push(const Mat4& model, const Material& material,
                                   std::uint32_t entity_id, std::uint32_t selected_id,
                                   float outline_shift = 0.0f)
                {
                    MeshPush push{};
                    for (int i = 0; i < 16; ++i)
                        push.model[i] = static_cast<float>(model.m[i]);
                    push.albedo_metallic[0] = static_cast<float>(material.albedo.x);
                    push.albedo_metallic[1] = static_cast<float>(material.albedo.y);
                    push.albedo_metallic[2] = static_cast<float>(material.albedo.z);
                    push.albedo_metallic[3] = material.metallic;
                    push.emissive_roughness[0] = static_cast<float>(material.emissive.x);
                    push.emissive_roughness[1] = static_cast<float>(material.emissive.y);
                    push.emissive_roughness[2] = static_cast<float>(material.emissive.z);
                    push.emissive_roughness[3] = material.roughness;
                    push.outline_shift[0] = outline_shift;
                    push.outline_shift[1] = outline_shift;
                    push.entity_id = entity_id;
                    push.selected = selected_id;
                    return push;
                }

                /** @brief A flat, unlit material carrying just a colour (grid, cloth defaults). */
                Material flat_material(const Vector3& color)
                {
                    Material material;
                    material.albedo = color;
                    material.metallic = 0.0f;
                    material.roughness = 0.9f;
                    return material;
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
                create_descriptors();
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
                destroy_descriptors();
                if (sampler_ != VK_NULL_HANDLE)
                    vkDestroySampler(device_.device(), sampler_, nullptr);
            }

            void VulkanSceneView::create_descriptors()
            {
                // One set layout for every pass: binding 0 is the per-frame scene UBO
                // (read by vertex and fragment stages), bindings 1 and 2 are combined
                // image samplers the sky/tonemap passes read (depth, hdr / composite).
                VkDescriptorSetLayoutBinding bindings[3]{};
                bindings[0].binding = 0;
                bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
                bindings[0].descriptorCount = 1;
                bindings[0].stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
                bindings[1].binding = 1;
                bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                bindings[1].descriptorCount = 1;
                bindings[1].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
                bindings[2].binding = 2;
                bindings[2].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                bindings[2].descriptorCount = 1;
                bindings[2].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

                VkDescriptorSetLayoutCreateInfo layout_info{};
                layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
                layout_info.bindingCount = 3;
                layout_info.pBindings = bindings;
                check(vkCreateDescriptorSetLayout(device_.device(), &layout_info, nullptr,
                                                  &set_layout_),
                      "vkCreateDescriptorSetLayout");

                // Three sets per slot (mesh, sky, tonemap); pool reset and reallocated
                // whenever the targets are rebuilt (see create_targets).
                VkDescriptorPoolSize sizes[2]{};
                sizes[0].type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
                sizes[0].descriptorCount = SLOTS * 3;
                sizes[1].type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                sizes[1].descriptorCount = SLOTS * 3 * 2;

                VkDescriptorPoolCreateInfo pool_info{};
                pool_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
                pool_info.maxSets = SLOTS * 3;
                pool_info.poolSizeCount = 2;
                pool_info.pPoolSizes = sizes;
                check(vkCreateDescriptorPool(device_.device(), &pool_info, nullptr,
                                             &descriptor_pool_),
                      "vkCreateDescriptorPool");
            }

            void VulkanSceneView::destroy_descriptors()
            {
                if (descriptor_pool_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorPool(device_.device(), descriptor_pool_, nullptr);
                if (set_layout_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorSetLayout(device_.device(), set_layout_, nullptr);
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
                layout_info.setLayoutCount = 1;
                layout_info.pSetLayouts = &set_layout_;
                layout_info.pushConstantRangeCount = 1;
                layout_info.pPushConstantRanges = &range;
                check(vkCreatePipelineLayout(device_.device(), &layout_info, nullptr, &layout_),
                      "vkCreatePipelineLayout");

                VkShaderModule vertex_module = make_shader(device_.device(), Shaders::mesh_vert_spv,
                                                           Shaders::mesh_vert_spv_word_count);
                VkShaderModule mesh_fragment = make_shader(device_.device(), Shaders::pbr_frag_spv,
                                                           Shaders::pbr_frag_spv_word_count);
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

                const VkFormat color_formats[2] = {HDR_FORMAT, ID_FORMAT};
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

                // Fullscreen post passes (sky, tonemap): a vertex-only triangle, no depth,
                // no vertex input, one colour attachment. The sky pass targets the HDR
                // composite; the tonemap pass targets the LDR resolve image.
                VkShaderModule fullscreen_vertex =
                    make_shader(device_.device(), Shaders::fullscreen_vert_spv,
                                Shaders::fullscreen_vert_spv_word_count);
                VkShaderModule sky_fragment = make_shader(device_.device(), Shaders::sky_frag_spv,
                                                          Shaders::sky_frag_spv_word_count);
                VkShaderModule tonemap_fragment =
                    make_shader(device_.device(), Shaders::tonemap_frag_spv,
                                Shaders::tonemap_frag_spv_word_count);

                auto build_fullscreen = [&](VkShaderModule fragment, VkFormat color_format) -> VkPipeline
                {
                    VkPipelineShaderStageCreateInfo stages[2]{};
                    stages[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
                    stages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
                    stages[0].module = fullscreen_vertex;
                    stages[0].pName = "main";
                    stages[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
                    stages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
                    stages[1].module = fragment;
                    stages[1].pName = "main";

                    VkPipelineVertexInputStateCreateInfo empty_input{};
                    empty_input.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;

                    VkPipelineInputAssemblyStateCreateInfo input_assembly{};
                    input_assembly.sType =
                        VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
                    input_assembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

                    VkPipelineRasterizationStateCreateInfo raster{};
                    raster.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
                    raster.polygonMode = VK_POLYGON_MODE_FILL;
                    raster.cullMode = VK_CULL_MODE_NONE;
                    raster.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
                    raster.lineWidth = 1.0f;

                    VkPipelineDepthStencilStateCreateInfo no_depth{};
                    no_depth.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;

                    VkPipelineColorBlendAttachmentState blend_attachment{};
                    blend_attachment.colorWriteMask =
                        VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
                        VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
                    VkPipelineColorBlendStateCreateInfo fullscreen_blend{};
                    fullscreen_blend.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
                    fullscreen_blend.attachmentCount = 1;
                    fullscreen_blend.pAttachments = &blend_attachment;

                    const VkDynamicState fullscreen_dynamic_states[2] = {VK_DYNAMIC_STATE_VIEWPORT,
                                                                         VK_DYNAMIC_STATE_SCISSOR};
                    VkPipelineDynamicStateCreateInfo fullscreen_dynamic{};
                    fullscreen_dynamic.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
                    fullscreen_dynamic.dynamicStateCount = 2;
                    fullscreen_dynamic.pDynamicStates = fullscreen_dynamic_states;

                    VkPipelineRenderingCreateInfo fullscreen_rendering{};
                    fullscreen_rendering.sType = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO;
                    fullscreen_rendering.colorAttachmentCount = 1;
                    fullscreen_rendering.pColorAttachmentFormats = &color_format;

                    VkGraphicsPipelineCreateInfo info{};
                    info.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
                    info.pNext = &fullscreen_rendering;
                    info.stageCount = 2;
                    info.pStages = stages;
                    info.pVertexInputState = &empty_input;
                    info.pInputAssemblyState = &input_assembly;
                    info.pViewportState = &viewport_state;
                    info.pRasterizationState = &raster;
                    info.pMultisampleState = &multisample;
                    info.pDepthStencilState = &no_depth;
                    info.pColorBlendState = &fullscreen_blend;
                    info.pDynamicState = &fullscreen_dynamic;
                    info.layout = layout_;

                    VkPipeline pipeline = VK_NULL_HANDLE;
                    check(vkCreateGraphicsPipelines(device_.device(), VK_NULL_HANDLE, 1, &info,
                                                    nullptr, &pipeline),
                          "vkCreateGraphicsPipelines(fullscreen)");
                    return pipeline;
                };

                sky_pipeline_ = build_fullscreen(sky_fragment, HDR_FORMAT);
                tonemap_pipeline_ = build_fullscreen(tonemap_fragment, RESOLVE_FORMAT);

                vkDestroyShaderModule(device_.device(), tonemap_fragment, nullptr);
                vkDestroyShaderModule(device_.device(), sky_fragment, nullptr);
                vkDestroyShaderModule(device_.device(), fullscreen_vertex, nullptr);
                vkDestroyShaderModule(device_.device(), outline_vertex, nullptr);
                vkDestroyShaderModule(device_.device(), outline_fragment, nullptr);
                vkDestroyShaderModule(device_.device(), line_fragment, nullptr);
                vkDestroyShaderModule(device_.device(), mesh_fragment, nullptr);
                vkDestroyShaderModule(device_.device(), vertex_module, nullptr);
            }

            void VulkanSceneView::destroy_pipelines()
            {
                if (tonemap_pipeline_ != VK_NULL_HANDLE)
                    vkDestroyPipeline(device_.device(), tonemap_pipeline_, nullptr);
                if (sky_pipeline_ != VK_NULL_HANDLE)
                    vkDestroyPipeline(device_.device(), sky_pipeline_, nullptr);
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
                                       &grid_vertices_, &cloth_vertices_, &cloth_indices_})
                    if (buffer->buffer != VK_NULL_HANDLE)
                        vmaDestroyBuffer(device_.allocator(), buffer->buffer, buffer->allocation);
                cloth_vertices_capacity_ = 0;
                cloth_vertices_mapped_ = nullptr;
                cloth_indices_capacity_ = 0;
                cloth_indices_mapped_ = nullptr;
            }

            void VulkanSceneView::ensure_cloth_capacity(VkDeviceSize vertex_bytes,
                                                        VkDeviceSize index_bytes)
            {
                const auto grow = [&](Buffer& buffer, VkDeviceSize& capacity, void*& mapped,
                                      VkDeviceSize bytes, VkBufferUsageFlags usage,
                                      const char* what)
                {
                    if (bytes <= capacity)
                        return;
                    if (buffer.buffer != VK_NULL_HANDLE)
                        vmaDestroyBuffer(device_.allocator(), buffer.buffer, buffer.allocation);

                    VkBufferCreateInfo buffer_info{};
                    buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
                    buffer_info.size = bytes;
                    buffer_info.usage = usage;

                    VmaAllocationCreateInfo alloc{};
                    alloc.usage = VMA_MEMORY_USAGE_AUTO;
                    alloc.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
                                  VMA_ALLOCATION_CREATE_MAPPED_BIT;

                    buffer = Buffer{};
                    VmaAllocationInfo info{};
                    check(vmaCreateBuffer(device_.allocator(), &buffer_info, &alloc, &buffer.buffer,
                                          &buffer.allocation, &info),
                          what);
                    mapped = info.pMappedData;
                    capacity = bytes;
                };

                grow(cloth_vertices_, cloth_vertices_capacity_, cloth_vertices_mapped_,
                    vertex_bytes, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, "vmaCreateBuffer(cloth vertices)");
                grow(cloth_indices_, cloth_indices_capacity_, cloth_indices_mapped_, index_bytes,
                    VK_BUFFER_USAGE_INDEX_BUFFER_BIT, "vmaCreateBuffer(cloth indices)");
            }

            void VulkanSceneView::create_targets()
            {
                // The descriptor sets are tied to this size's images, so the pool is reset
                // and the sets reallocated whenever the targets are (re)built.
                check(vkResetDescriptorPool(device_.device(), descriptor_pool_, 0),
                      "vkResetDescriptorPool");

                for (Slot& slot : slots_)
                {
                    VmaAllocationCreateInfo image_alloc{};
                    image_alloc.usage = VMA_MEMORY_USAGE_AUTO;

                    auto make_image = [&](VkFormat format, VkImageUsageFlags usage, VkImage& image,
                                          VmaAllocation& allocation, const char* what)
                    {
                        VkImageCreateInfo info{};
                        info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
                        info.imageType = VK_IMAGE_TYPE_2D;
                        info.format = format;
                        info.extent = {width_, height_, 1};
                        info.mipLevels = 1;
                        info.arrayLayers = 1;
                        info.samples = VK_SAMPLE_COUNT_1_BIT;
                        info.tiling = VK_IMAGE_TILING_OPTIMAL;
                        info.usage = usage;
                        check(vmaCreateImage(device_.allocator(), &info, &image_alloc, &image,
                                             &allocation, nullptr), what);
                    };

                    auto make_view = [&](VkImage image, VkFormat format, VkImageAspectFlags aspect,
                                         VkImageView& view, const char* what)
                    {
                        VkImageViewCreateInfo info{};
                        info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
                        info.image = image;
                        info.viewType = VK_IMAGE_VIEW_TYPE_2D;
                        info.format = format;
                        info.subresourceRange.aspectMask = aspect;
                        info.subresourceRange.levelCount = 1;
                        info.subresourceRange.layerCount = 1;
                        check(vkCreateImageView(device_.device(), &info, nullptr, &view), what);
                    };

                    // HDR scene target (opaque pass writes, sky pass samples); HDR composite
                    // (sky pass writes, tonemap samples); LDR resolve (tonemap writes, editor
                    // samples). Depth carries SAMPLED so the sky pass can read it.
                    make_image(HDR_FORMAT,
                               VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
                               slot.hdr, slot.hdr_allocation, "vmaCreateImage(hdr)");
                    make_image(HDR_FORMAT,
                               VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
                               slot.composite, slot.composite_allocation, "vmaCreateImage(composite)");
                    make_image(RESOLVE_FORMAT,
                               VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
                               slot.resolve, slot.resolve_allocation, "vmaCreateImage(resolve)");
                    make_image(DEPTH_FORMAT,
                               VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
                               slot.depth, slot.depth_allocation, "vmaCreateImage(depth)");
                    make_image(ID_FORMAT,
                               VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT,
                               slot.id, slot.id_allocation, "vmaCreateImage(id)");

                    make_view(slot.hdr, HDR_FORMAT, VK_IMAGE_ASPECT_COLOR_BIT, slot.hdr_view,
                              "vkCreateImageView(hdr)");
                    make_view(slot.composite, HDR_FORMAT, VK_IMAGE_ASPECT_COLOR_BIT,
                              slot.composite_view, "vkCreateImageView(composite)");
                    make_view(slot.resolve, RESOLVE_FORMAT, VK_IMAGE_ASPECT_COLOR_BIT,
                              slot.resolve_view, "vkCreateImageView(resolve)");
                    make_view(slot.depth, DEPTH_FORMAT,
                              VK_IMAGE_ASPECT_DEPTH_BIT | VK_IMAGE_ASPECT_STENCIL_BIT, slot.depth_view,
                              "vkCreateImageView(depth)");
                    make_view(slot.depth, DEPTH_FORMAT, VK_IMAGE_ASPECT_DEPTH_BIT,
                              slot.depth_sample_view, "vkCreateImageView(depth sample)");
                    make_view(slot.id, ID_FORMAT, VK_IMAGE_ASPECT_COLOR_BIT, slot.id_view,
                              "vkCreateImageView(id)");

                    // Per-frame scene uniform buffer, host-visible and mapped.
                    VkBufferCreateInfo ubo_info{};
                    ubo_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
                    ubo_info.size = sizeof(SceneUbo);
                    ubo_info.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
                    VmaAllocationCreateInfo ubo_alloc{};
                    ubo_alloc.usage = VMA_MEMORY_USAGE_AUTO;
                    ubo_alloc.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
                                      VMA_ALLOCATION_CREATE_MAPPED_BIT;
                    VmaAllocationInfo ubo_mapped{};
                    check(vmaCreateBuffer(device_.allocator(), &ubo_info, &ubo_alloc, &slot.ubo,
                                          &slot.ubo_allocation, &ubo_mapped),
                          "vmaCreateBuffer(ubo)");
                    slot.ubo_mapped = ubo_mapped.pMappedData;

                    // Allocate the three descriptor sets and point them at this slot's UBO
                    // and sampled images (the layout each will be sampled in is fixed).
                    VkDescriptorSetLayout layouts[3] = {set_layout_, set_layout_, set_layout_};
                    VkDescriptorSetAllocateInfo set_info{};
                    set_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
                    set_info.descriptorPool = descriptor_pool_;
                    set_info.descriptorSetCount = 3;
                    set_info.pSetLayouts = layouts;
                    VkDescriptorSet sets[3]{};
                    check(vkAllocateDescriptorSets(device_.device(), &set_info, sets),
                          "vkAllocateDescriptorSets");
                    slot.mesh_set = sets[0];
                    slot.sky_set = sets[1];
                    slot.tonemap_set = sets[2];

                    VkDescriptorBufferInfo ubo_desc{};
                    ubo_desc.buffer = slot.ubo;
                    ubo_desc.range = sizeof(SceneUbo);

                    auto image_desc = [&](VkImageView view)
                    {
                        VkDescriptorImageInfo info{};
                        info.sampler = sampler_;
                        info.imageView = view;
                        info.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
                        return info;
                    };
                    const VkDescriptorImageInfo depth_desc = image_desc(slot.depth_sample_view);
                    const VkDescriptorImageInfo hdr_desc = image_desc(slot.hdr_view);
                    const VkDescriptorImageInfo composite_desc = image_desc(slot.composite_view);

                    VkWriteDescriptorSet writes[7]{};
                    auto write_ubo = [&](VkWriteDescriptorSet& w, VkDescriptorSet set)
                    {
                        w.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
                        w.dstSet = set;
                        w.dstBinding = 0;
                        w.descriptorCount = 1;
                        w.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
                        w.pBufferInfo = &ubo_desc;
                    };
                    auto write_image = [&](VkWriteDescriptorSet& w, VkDescriptorSet set,
                                           std::uint32_t binding, const VkDescriptorImageInfo& info)
                    {
                        w.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
                        w.dstSet = set;
                        w.dstBinding = binding;
                        w.descriptorCount = 1;
                        w.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                        w.pImageInfo = &info;
                    };
                    write_ubo(writes[0], slot.mesh_set);
                    write_ubo(writes[1], slot.sky_set);
                    write_image(writes[2], slot.sky_set, 1, depth_desc);
                    write_image(writes[3], slot.sky_set, 2, hdr_desc);
                    write_ubo(writes[4], slot.tonemap_set);
                    write_image(writes[5], slot.tonemap_set, 1, composite_desc);
                    // The mesh set never samples bindings 1/2, but give them a valid view so
                    // the set is fully initialised.
                    write_image(writes[6], slot.mesh_set, 1, depth_desc);
                    vkUpdateDescriptorSets(device_.device(), 7, writes, 0, nullptr);

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
                    if (slot.ubo != VK_NULL_HANDLE)
                        vmaDestroyBuffer(device_.allocator(), slot.ubo, slot.ubo_allocation);
                    if (slot.readback != VK_NULL_HANDLE)
                        vmaDestroyBuffer(device_.allocator(), slot.readback, slot.readback_allocation);
                    if (slot.id_view != VK_NULL_HANDLE)
                        vkDestroyImageView(device_.device(), slot.id_view, nullptr);
                    if (slot.id != VK_NULL_HANDLE)
                        vmaDestroyImage(device_.allocator(), slot.id, slot.id_allocation);
                    if (slot.depth_sample_view != VK_NULL_HANDLE)
                        vkDestroyImageView(device_.device(), slot.depth_sample_view, nullptr);
                    if (slot.depth_view != VK_NULL_HANDLE)
                        vkDestroyImageView(device_.device(), slot.depth_view, nullptr);
                    if (slot.depth != VK_NULL_HANDLE)
                        vmaDestroyImage(device_.allocator(), slot.depth, slot.depth_allocation);
                    if (slot.resolve_view != VK_NULL_HANDLE)
                        vkDestroyImageView(device_.device(), slot.resolve_view, nullptr);
                    if (slot.resolve != VK_NULL_HANDLE)
                        vmaDestroyImage(device_.allocator(), slot.resolve, slot.resolve_allocation);
                    if (slot.composite_view != VK_NULL_HANDLE)
                        vkDestroyImageView(device_.device(), slot.composite_view, nullptr);
                    if (slot.composite != VK_NULL_HANDLE)
                        vmaDestroyImage(device_.allocator(), slot.composite, slot.composite_allocation);
                    if (slot.hdr_view != VK_NULL_HANDLE)
                        vkDestroyImageView(device_.device(), slot.hdr_view, nullptr);
                    if (slot.hdr != VK_NULL_HANDLE)
                        vmaDestroyImage(device_.allocator(), slot.hdr, slot.hdr_allocation);
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
                texture.image_view = slots_[slot].resolve_view;
                return texture;
            }

            void VulkanSceneView::render(const CameraView& camera, const Environment& environment,
                                         const MeshInstance* instances,
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

                // Fill the per-frame scene uniform block from the camera and environment.
                // The sky pass works in camera-relative space, so the planet centre and the
                // ray basis are all expressed relative to the camera eye recovered here.
                {
                    const Mat4& V = camera.view;
                    const Mat4& P = camera.projection;
                    const double right[3] = {V.m[0], V.m[4], V.m[8]};
                    const double up[3] = {V.m[1], V.m[5], V.m[9]};
                    const double fwd[3] = {-V.m[2], -V.m[6], -V.m[10]};
                    const double eye[3] = {
                        -(V.m[0] * V.m[12] + V.m[1] * V.m[13] + V.m[2] * V.m[14]),
                        -(V.m[4] * V.m[12] + V.m[5] * V.m[13] + V.m[6] * V.m[14]),
                        -(V.m[8] * V.m[12] + V.m[9] * V.m[13] + V.m[10] * V.m[14])};
                    const double tan_half_x = P.m[0] != 0.0 ? 1.0 / P.m[0] : 1.0;
                    const double tan_half_y = P.m[5] != 0.0 ? 1.0 / (-P.m[5]) : 1.0;

                    const double a = environment.planet.semi_major;
                    const double b = environment.planet.semi_minor();
                    // Planet anchored so its equatorial surface passes through the local
                    // origin (the ground grid), centre straight down by a.
                    const double center_local[3] = {0.0, -a, 0.0};
                    const double to_center[3] = {eye[0] - center_local[0], eye[1] - center_local[1],
                                                 eye[2] - center_local[2]};
                    const double altitude =
                        std::sqrt(to_center[0] * to_center[0] + to_center[1] * to_center[1] +
                                  to_center[2] * to_center[2]) - a;

                    const Vector3 sun_dir = normalize(environment.sun.direction);

                    SceneUbo ubo{};
                    for (int i = 0; i < 16; ++i)
                    {
                        ubo.view[i] = static_cast<float>(V.m[i]);
                        ubo.proj[i] = static_cast<float>(P.m[i]);
                    }
                    ubo.cam_forward[0] = static_cast<float>(fwd[0]);
                    ubo.cam_forward[1] = static_cast<float>(fwd[1]);
                    ubo.cam_forward[2] = static_cast<float>(fwd[2]);
                    ubo.cam_forward[3] = static_cast<float>(eye[0]);
                    ubo.cam_right[0] = static_cast<float>(right[0] * tan_half_x);
                    ubo.cam_right[1] = static_cast<float>(right[1] * tan_half_x);
                    ubo.cam_right[2] = static_cast<float>(right[2] * tan_half_x);
                    ubo.cam_right[3] = static_cast<float>(eye[1]);
                    ubo.cam_up[0] = static_cast<float>(-up[0] * tan_half_y);
                    ubo.cam_up[1] = static_cast<float>(-up[1] * tan_half_y);
                    ubo.cam_up[2] = static_cast<float>(-up[2] * tan_half_y);
                    ubo.cam_up[3] = static_cast<float>(eye[2]);
                    ubo.planet_center[0] = static_cast<float>(center_local[0] - eye[0]);
                    ubo.planet_center[1] = static_cast<float>(center_local[1] - eye[1]);
                    ubo.planet_center[2] = static_cast<float>(center_local[2] - eye[2]);
                    ubo.planet_center[3] = static_cast<float>(a);
                    ubo.planet_radii[0] = static_cast<float>(a);
                    ubo.planet_radii[1] = static_cast<float>(a);
                    ubo.planet_radii[2] = static_cast<float>(b);
                    ubo.planet_radii[3] = environment.atmosphere.height;
                    ubo.sun_dir[0] = static_cast<float>(sun_dir.x);
                    ubo.sun_dir[1] = static_cast<float>(sun_dir.y);
                    ubo.sun_dir[2] = static_cast<float>(sun_dir.z);
                    ubo.sun_dir[3] = environment.sun.intensity;
                    ubo.sun_color[0] = static_cast<float>(environment.sun.color.x);
                    ubo.sun_color[1] = static_cast<float>(environment.sun.color.y);
                    ubo.sun_color[2] = static_cast<float>(environment.sun.color.z);
                    ubo.sun_color[3] = environment.exposure;
                    ubo.ambient[0] = static_cast<float>(environment.ambient.x);
                    ubo.ambient[1] = static_cast<float>(environment.ambient.y);
                    ubo.ambient[2] = static_cast<float>(environment.ambient.z);
                    ubo.rayleigh[0] = static_cast<float>(environment.atmosphere.rayleigh_coefficient.x);
                    ubo.rayleigh[1] = static_cast<float>(environment.atmosphere.rayleigh_coefficient.y);
                    ubo.rayleigh[2] = static_cast<float>(environment.atmosphere.rayleigh_coefficient.z);
                    ubo.rayleigh[3] = environment.atmosphere.mie_coefficient;
                    ubo.scatter[0] = environment.atmosphere.mie_anisotropy;
                    ubo.scatter[1] = environment.atmosphere.rayleigh_scale_height;
                    ubo.scatter[2] = environment.atmosphere.mie_scale_height;
                    ubo.scatter[3] = static_cast<float>(altitude);
                    ubo.ground_albedo[0] = static_cast<float>(environment.surface.ground_albedo.x);
                    ubo.ground_albedo[1] = static_cast<float>(environment.surface.ground_albedo.y);
                    ubo.ground_albedo[2] = static_cast<float>(environment.surface.ground_albedo.z);
                    ubo.ocean_color[0] = static_cast<float>(environment.surface.ocean_color.x);
                    ubo.ocean_color[1] = static_cast<float>(environment.surface.ocean_color.y);
                    ubo.ocean_color[2] = static_cast<float>(environment.surface.ocean_color.z);
                    ubo.cloud_params[0] = environment.clouds.base_altitude;
                    ubo.cloud_params[1] = environment.clouds.top_altitude;
                    ubo.cloud_params[2] = environment.clouds.coverage;
                    ubo.cloud_params[3] = environment.clouds.density;
                    ubo.star_params[0] = environment.stars.brightness;
                    ubo.star_params[1] = environment.stars.density;
                    ubo.star_params[2] = environment.atmosphere.enabled ? 1.0f : 0.0f;
                    ubo.star_params[3] = environment.stars.enabled ? 1.0f : 0.0f;
                    ubo.misc[0] = camera.near_plane;
                    ubo.misc[1] = camera.far_plane;
                    ubo.misc[2] = static_cast<float>(frame_counter_) * 0.016f;
                    ubo.misc[3] = environment.clouds.enabled ? 1.0f : 0.0f;
                    std::memcpy(slot.ubo_mapped, &ubo, sizeof(SceneUbo));
                }

                VkCommandBufferBeginInfo begin{};
                begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
                begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
                check(vkBeginCommandBuffer(slot.cmd, &begin), "vkBeginCommandBuffer");

                VkViewport viewport{};
                viewport.width = static_cast<float>(width_);
                viewport.height = static_cast<float>(height_);
                viewport.maxDepth = 1.0f;
                VkRect2D scissor{};
                scissor.extent = {width_, height_};

                const VkDeviceSize zero_offset = 0;
                const std::uint32_t stages = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;

                // ---- Opaque pass: grid + lit meshes + cloth into the HDR + id targets. ----
                transition(slot.cmd, slot.hdr, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_UNDEFINED,
                           VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,
                           VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, 0,
                           VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT);
                transition(slot.cmd, slot.depth, VK_IMAGE_ASPECT_DEPTH_BIT | VK_IMAGE_ASPECT_STENCIL_BIT,
                           VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL,
                           VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,
                           VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT, 0,
                           VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT);
                transition(slot.cmd, slot.id, VK_IMAGE_ASPECT_COLOR_BIT,
                           VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                           VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,
                           VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, 0,
                           VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT);

                VkRenderingAttachmentInfo color_attachment{};
                color_attachment.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
                color_attachment.imageView = slot.hdr_view;
                color_attachment.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
                color_attachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
                color_attachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
                color_attachment.clearValue.color = {{0.0f, 0.0f, 0.0f, 1.0f}};

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
                depth_attachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
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

                vkCmdSetViewport(slot.cmd, 0, 1, &viewport);
                vkCmdSetScissor(slot.cmd, 0, 1, &scissor);
                vkCmdBindDescriptorSets(slot.cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, layout_, 0, 1,
                                        &slot.mesh_set, 0, nullptr);

                // Ground grid: a single flat-coloured line draw.
                vkCmdBindPipeline(slot.cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, line_pipeline_);
                vkCmdSetStencilReference(slot.cmd, VK_STENCIL_FACE_FRONT_AND_BACK, 0);
                vkCmdBindVertexBuffers(slot.cmd, 0, 1, &grid_vertices_.buffer, &zero_offset);
                const MeshPush grid_push =
                    make_push(Mat4{}, flat_material(Vector3{0.32f, 0.33f, 0.40f}), NO_PICK, NO_PICK);
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
                        const MeshPush push = make_push(scaled_model, instances[i].material,
                                                        instances[i].id, selected_id);
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
                            const MeshPush push = make_push(scaled_model,
                                                            flat_material(instances[i].color),
                                                            instances[i].id, selected_id, 0.006f);
                            vkCmdPushConstants(slot.cmd, layout_, stages, 0, sizeof(MeshPush), &push);
                            vkCmdDrawIndexed(slot.cmd, group.indices->count, 1, 0, 0, 0);
                        }
                    }
                }

                // Soft-body meshes: every strand's grid triangulated (see
                // triangulate_cloth_grid) and concatenated into one indexed draw per
                // strand, uploaded to the host-visible cloth buffers just ahead of
                // recording it. Drawn with the same lit mesh pipeline Box/Sphere/
                // Cylinder use (already double-sided) so cloth shades and picks like
                // any other object instead of drawing as a bare wireframe.
                if (strand_count > 0)
                {
                    std::vector<ClothVertex> triangulated_vertices;
                    std::vector<std::uint32_t> triangulated_indices;
                    std::vector<Vertex> all_vertices;
                    std::vector<std::uint32_t> all_indices;
                    std::vector<std::pair<std::uint32_t, std::uint32_t>> draw_ranges;
                    draw_ranges.reserve(strand_count);

                    for (std::size_t s = 0; s < strand_count; ++s)
                    {
                        const ClothStrandView& strand = strands[s];
                        triangulate_cloth_grid(strand.vertices, strand.rows, strand.cols,
                                               triangulated_vertices, triangulated_indices);
                        if (triangulated_indices.empty())
                            continue;

                        const std::uint32_t base_vertex =
                            static_cast<std::uint32_t>(all_vertices.size());
                        for (const ClothVertex& vertex : triangulated_vertices)
                            all_vertices.push_back(
                                Vertex{{static_cast<float>(vertex.position.x),
                                       static_cast<float>(vertex.position.y),
                                       static_cast<float>(vertex.position.z)},
                                      {static_cast<float>(vertex.normal.x),
                                       static_cast<float>(vertex.normal.y),
                                       static_cast<float>(vertex.normal.z)}});

                        const std::uint32_t index_offset =
                            static_cast<std::uint32_t>(all_indices.size());
                        for (std::uint32_t index : triangulated_indices)
                            all_indices.push_back(base_vertex + index);
                        draw_ranges.emplace_back(
                            index_offset, static_cast<std::uint32_t>(triangulated_indices.size()));
                    }

                    if (!all_indices.empty())
                    {
                        ensure_cloth_capacity(all_vertices.size() * sizeof(Vertex),
                                             all_indices.size() * sizeof(std::uint32_t));
                        std::memcpy(cloth_vertices_mapped_, all_vertices.data(),
                                   all_vertices.size() * sizeof(Vertex));
                        std::memcpy(cloth_indices_mapped_, all_indices.data(),
                                   all_indices.size() * sizeof(std::uint32_t));

                        vkCmdBindPipeline(slot.cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, mesh_pipeline_);
                        vkCmdBindVertexBuffers(slot.cmd, 0, 1, &cloth_vertices_.buffer, &zero_offset);
                        vkCmdBindIndexBuffer(slot.cmd, cloth_indices_.buffer, 0,
                                             VK_INDEX_TYPE_UINT32);
                        std::size_t range = 0;
                        for (std::size_t s = 0; s < strand_count; ++s)
                        {
                            const ClothStrandView& strand = strands[s];
                            if (strand.rows < 2 || strand.cols < 2)
                                continue;
                            const auto [index_offset, index_count] = draw_ranges[range++];
                            vkCmdSetStencilReference(slot.cmd, VK_STENCIL_FACE_FRONT_AND_BACK,
                                                     (strand.id == selected_id) ? 1 : 0);
                            const MeshPush cloth_push = make_push(Mat4{}, flat_material(strand.color),
                                                                  strand.id, selected_id);
                            vkCmdPushConstants(slot.cmd, layout_, stages, 0, sizeof(MeshPush),
                                              &cloth_push);
                            vkCmdDrawIndexed(slot.cmd, index_count, 1, index_offset, 0, 0);
                        }

                        // Outline pass for a selected cloth mesh, masked the same way as
                        // the Box/Sphere/Cylinder outline above.
                        if (selected_id != NO_PICK)
                        {
                            vkCmdBindPipeline(slot.cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                              outline_pipeline_);
                            vkCmdSetStencilReference(slot.cmd, VK_STENCIL_FACE_FRONT_AND_BACK, 1);
                            range = 0;
                            for (std::size_t s = 0; s < strand_count; ++s)
                            {
                                const ClothStrandView& strand = strands[s];
                                if (strand.rows < 2 || strand.cols < 2)
                                    continue;
                                const auto [index_offset, index_count] = draw_ranges[range++];
                                if (strand.id != selected_id)
                                    continue;
                                const MeshPush cloth_push = make_push(Mat4{},
                                                                      flat_material(strand.color),
                                                                      strand.id, selected_id, 0.006f);
                                vkCmdPushConstants(slot.cmd, layout_, stages, 0, sizeof(MeshPush),
                                                  &cloth_push);
                                vkCmdDrawIndexed(slot.cmd, index_count, 1, index_offset, 0, 0);
                            }
                        }
                    }
                }

                vkCmdEndRendering(slot.cmd);

                // Make the HDR scene and the depth buffer readable by the sky pass.
                transition(slot.cmd, slot.hdr, VK_IMAGE_ASPECT_COLOR_BIT,
                           VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                           VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                           VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                           VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                           VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT, VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
                transition(slot.cmd, slot.depth, VK_IMAGE_ASPECT_DEPTH_BIT | VK_IMAGE_ASPECT_STENCIL_BIT,
                           VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL,
                           VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                           VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT,
                           VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                           VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT,
                           VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);

                // ---- Sky pass: planet + atmosphere + clouds + stars into the composite. ----
                transition(slot.cmd, slot.composite, VK_IMAGE_ASPECT_COLOR_BIT,
                           VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                           VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,
                           VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, 0,
                           VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT);

                VkRenderingAttachmentInfo sky_attachment{};
                sky_attachment.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
                sky_attachment.imageView = slot.composite_view;
                sky_attachment.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
                sky_attachment.loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
                sky_attachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;

                VkRenderingInfo sky_rendering{};
                sky_rendering.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
                sky_rendering.renderArea.extent = {width_, height_};
                sky_rendering.layerCount = 1;
                sky_rendering.colorAttachmentCount = 1;
                sky_rendering.pColorAttachments = &sky_attachment;
                vkCmdBeginRendering(slot.cmd, &sky_rendering);
                vkCmdSetViewport(slot.cmd, 0, 1, &viewport);
                vkCmdSetScissor(slot.cmd, 0, 1, &scissor);
                vkCmdBindDescriptorSets(slot.cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, layout_, 0, 1,
                                        &slot.sky_set, 0, nullptr);
                vkCmdBindPipeline(slot.cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, sky_pipeline_);
                vkCmdDraw(slot.cmd, 3, 1, 0, 0);
                vkCmdEndRendering(slot.cmd);

                transition(slot.cmd, slot.composite, VK_IMAGE_ASPECT_COLOR_BIT,
                           VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                           VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                           VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                           VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                           VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT, VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);

                // ---- Tonemap pass: resolve the HDR composite into the LDR image ImGui reads. ----
                const VkImageLayout resolve_from = slot.ever_rendered
                                                       ? VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL
                                                       : VK_IMAGE_LAYOUT_UNDEFINED;
                const VkPipelineStageFlags2 resolve_src_stage =
                    slot.ever_rendered ? VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT
                                       : VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT;
                const VkAccessFlags2 resolve_src_access =
                    slot.ever_rendered ? VK_ACCESS_2_SHADER_SAMPLED_READ_BIT : 0;
                transition(slot.cmd, slot.resolve, VK_IMAGE_ASPECT_COLOR_BIT, resolve_from,
                           VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, resolve_src_stage,
                           VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, resolve_src_access,
                           VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT);

                VkRenderingAttachmentInfo tonemap_attachment{};
                tonemap_attachment.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
                tonemap_attachment.imageView = slot.resolve_view;
                tonemap_attachment.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
                tonemap_attachment.loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
                tonemap_attachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;

                VkRenderingInfo tonemap_rendering{};
                tonemap_rendering.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
                tonemap_rendering.renderArea.extent = {width_, height_};
                tonemap_rendering.layerCount = 1;
                tonemap_rendering.colorAttachmentCount = 1;
                tonemap_rendering.pColorAttachments = &tonemap_attachment;
                vkCmdBeginRendering(slot.cmd, &tonemap_rendering);
                vkCmdSetViewport(slot.cmd, 0, 1, &viewport);
                vkCmdSetScissor(slot.cmd, 0, 1, &scissor);
                vkCmdBindDescriptorSets(slot.cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, layout_, 0, 1,
                                        &slot.tonemap_set, 0, nullptr);
                vkCmdBindPipeline(slot.cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, tonemap_pipeline_);
                vkCmdDraw(slot.cmd, 3, 1, 0, 0);
                vkCmdEndRendering(slot.cmd);

                transition(slot.cmd, slot.resolve, VK_IMAGE_ASPECT_COLOR_BIT,
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
