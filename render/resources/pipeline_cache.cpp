/**************************************************************************/
/* pipeline_cache.cpp                                                     */
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

#include "resources/pipeline_cache.hpp"

#include <cstring>
#include <fstream>
#include <utility>

#include "rhi/vulkan/vulkan_check.hpp"
#include "rhi/vulkan/vulkan_device.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Resources
        {
            namespace
            {
                /** @brief The vertex-input state structs, kept alive across a create call. */
                struct VertexInputState
                {
                    VkVertexInputBindingDescription binding{};
                    VkVertexInputAttributeDescription attributes[MAX_VERTEX_ATTRIBUTES]{};
                    VkPipelineVertexInputStateCreateInfo info{};
                    VkPipelineInputAssemblyStateCreateInfo assembly{};
                };

                /** @brief The colour-blend state structs, kept alive across a create call. */
                struct ColorBlendState
                {
                    VkPipelineColorBlendAttachmentState attachments[MAX_PIPELINE_COLOR_ATTACHMENTS]{};
                    VkPipelineColorBlendStateCreateInfo info{};
                };

                /**
                 * @brief Builds the vertex input and input assembly state from a description.
                 * @param desc  The pipeline description.
                 * @param state Receives the filled structs.
                 */
                void fill_vertex_input(const GraphicsPipelineDesc& desc, VertexInputState& state)
                {
                    state.binding.binding = 0;
                    state.binding.stride = desc.vertex_stride;
                    state.binding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
                    for (std::uint32_t i = 0; i < desc.attribute_count; ++i)
                    {
                        state.attributes[i].binding = 0;
                        state.attributes[i].location = desc.attributes[i].location;
                        state.attributes[i].format = desc.attributes[i].format;
                        state.attributes[i].offset = desc.attributes[i].offset;
                    }

                    state.info.sType =
                        VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
                    // A stride of zero is how a fullscreen pass says it fetches nothing:
                    // no binding, no attributes, the vertex shader synthesises positions.
                    state.info.vertexBindingDescriptionCount = desc.vertex_stride > 0 ? 1 : 0;
                    state.info.pVertexBindingDescriptions =
                        desc.vertex_stride > 0 ? &state.binding : nullptr;
                    state.info.vertexAttributeDescriptionCount = desc.attribute_count;
                    state.info.pVertexAttributeDescriptions =
                        desc.attribute_count > 0 ? state.attributes : nullptr;

                    state.assembly.sType =
                        VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
                    state.assembly.topology = desc.topology;
                }

                /**
                 * @brief Builds the rasterisation state from a description.
                 * @param desc The pipeline description.
                 * @return The filled struct.
                 */
                VkPipelineRasterizationStateCreateInfo fill_rasterization(
                    const GraphicsPipelineDesc& desc)
                {
                    VkPipelineRasterizationStateCreateInfo raster{};
                    raster.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
                    raster.polygonMode = desc.polygon_mode;
                    raster.cullMode = desc.cull_mode;
                    raster.frontFace = desc.front_face;
                    raster.lineWidth = desc.line_width;
                    raster.depthBiasEnable = desc.depth_bias_enable;
                    raster.depthBiasConstantFactor = desc.depth_bias_constant;
                    raster.depthBiasSlopeFactor = desc.depth_bias_slope;
                    return raster;
                }

                /**
                 * @brief Builds the depth/stencil state from a description.
                 * @param desc The pipeline description.
                 * @return The filled struct.
                 */
                VkPipelineDepthStencilStateCreateInfo fill_depth_stencil(
                    const GraphicsPipelineDesc& desc)
                {
                    VkPipelineDepthStencilStateCreateInfo depth{};
                    depth.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
                    depth.depthTestEnable = desc.depth_test;
                    depth.depthWriteEnable = desc.depth_write;
                    depth.depthCompareOp = desc.depth_compare;
                    depth.stencilTestEnable = desc.stencil_test;
                    depth.front = desc.stencil;
                    depth.back = desc.stencil;
                    return depth;
                }

                /**
                 * @brief Builds the colour blend state from a description.
                 * @param desc  The pipeline description.
                 * @param state Receives the filled structs.
                 */
                void fill_color_blend(const GraphicsPipelineDesc& desc, ColorBlendState& state)
                {
                    for (std::uint32_t i = 0; i < desc.color_count; ++i)
                        state.attachments[i].colorWriteMask =
                            VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
                            VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
                    state.info.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
                    state.info.attachmentCount = desc.color_count;
                    state.info.pAttachments = desc.color_count > 0 ? state.attachments : nullptr;
                }

                /**
                 * @brief Builds the dynamic-rendering format declaration from a description.
                 * @param desc The pipeline description.
                 * @return The filled struct, pointing into @p desc.
                 */
                VkPipelineRenderingCreateInfo fill_rendering(const GraphicsPipelineDesc& desc)
                {
                    VkPipelineRenderingCreateInfo rendering{};
                    rendering.sType = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO;
                    rendering.colorAttachmentCount = desc.color_count;
                    rendering.pColorAttachmentFormats =
                        desc.color_count > 0 ? desc.color_formats : nullptr;
                    rendering.depthAttachmentFormat = desc.depth_format;
                    rendering.stencilAttachmentFormat = desc.stencil_format;
                    return rendering;
                }

                /**
                 * @brief Builds a shader stage struct.
                 * @param stage  Which stage the module belongs to.
                 * @param module The compiled shader module.
                 * @return The filled struct.
                 */
                VkPipelineShaderStageCreateInfo shader_stage(VkShaderStageFlagBits stage,
                                                            VkShaderModule module)
                {
                    VkPipelineShaderStageCreateInfo info{};
                    info.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
                    info.stage = stage;
                    info.module = module;
                    info.pName = "main";
                    return info;
                }

                /** @brief The description subset that identifies a vertex-input library. */
                GraphicsPipelineDesc vertex_input_key(const GraphicsPipelineDesc& desc)
                {
                    GraphicsPipelineDesc key{};
                    // Zeroed whole, padding included, so a byte comparison of two keys
                    // built the same way is a correct equality test.
                    std::memset(&key, 0, sizeof(key));
                    key.vertex_stride = desc.vertex_stride;
                    key.attribute_count = desc.attribute_count;
                    for (std::uint32_t i = 0; i < desc.attribute_count; ++i)
                        key.attributes[i] = desc.attributes[i];
                    key.topology = desc.topology;
                    return key;
                }

                /** @brief The description subset that identifies a pre-rasterisation library. */
                GraphicsPipelineDesc pre_rasterization_key(const GraphicsPipelineDesc& desc)
                {
                    GraphicsPipelineDesc key{};
                    // Zeroed whole, padding included, so a byte comparison of two keys
                    // built the same way is a correct equality test.
                    std::memset(&key, 0, sizeof(key));
                    key.layout = desc.layout;
                    key.vertex_shader = desc.vertex_shader;
                    key.polygon_mode = desc.polygon_mode;
                    key.cull_mode = desc.cull_mode;
                    key.front_face = desc.front_face;
                    key.line_width = desc.line_width;
                    key.depth_bias_enable = desc.depth_bias_enable;
                    key.depth_bias_constant = desc.depth_bias_constant;
                    key.depth_bias_slope = desc.depth_bias_slope;
                    return key;
                }

                /** @brief The description subset that identifies a fragment-shader library. */
                GraphicsPipelineDesc fragment_shader_key(const GraphicsPipelineDesc& desc)
                {
                    GraphicsPipelineDesc key{};
                    // Zeroed whole, padding included, so a byte comparison of two keys
                    // built the same way is a correct equality test.
                    std::memset(&key, 0, sizeof(key));
                    key.layout = desc.layout;
                    key.fragment_shader = desc.fragment_shader;
                    key.depth_test = desc.depth_test;
                    key.depth_write = desc.depth_write;
                    key.depth_compare = desc.depth_compare;
                    key.stencil_test = desc.stencil_test;
                    key.stencil = desc.stencil;
                    key.dynamic_stencil_reference = desc.dynamic_stencil_reference;
                    key.depth_format = desc.depth_format;
                    key.stencil_format = desc.stencil_format;
                    return key;
                }

                /** @brief The description subset that identifies a fragment-output library. */
                GraphicsPipelineDesc fragment_output_key(const GraphicsPipelineDesc& desc)
                {
                    GraphicsPipelineDesc key{};
                    // Zeroed whole, padding included, so a byte comparison of two keys
                    // built the same way is a correct equality test.
                    std::memset(&key, 0, sizeof(key));
                    key.color_count = desc.color_count;
                    for (std::uint32_t i = 0; i < desc.color_count; ++i)
                        key.color_formats[i] = desc.color_formats[i];
                    key.depth_format = desc.depth_format;
                    key.stencil_format = desc.stencil_format;
                    return key;
                }
            } // namespace

            PipelineCache::PipelineCache(Vulkan::VulkanDevice& device, std::string path)
                : device_(device), path_(std::move(path))
            {
                // A blob from another driver or another device is rejected by the driver's
                // own header check, so a stale file costs a cold compile and nothing more.
                std::vector<char> blob;
                std::ifstream stream(path_, std::ios::binary | std::ios::ate);
                if (stream)
                {
                    const std::streamsize size = stream.tellg();
                    if (size > 0)
                    {
                        blob.resize(static_cast<std::size_t>(size));
                        stream.seekg(0);
                        stream.read(blob.data(), size);
                        if (!stream)
                            blob.clear();
                    }
                }

                VkPipelineCacheCreateInfo info{};
                info.sType = VK_STRUCTURE_TYPE_PIPELINE_CACHE_CREATE_INFO;
                info.initialDataSize = blob.size();
                info.pInitialData = blob.empty() ? nullptr : blob.data();
                Vulkan::check(vkCreatePipelineCache(device_.device(), &info, nullptr, &cache_),
                              "vkCreatePipelineCache");
            }

            PipelineCache::~PipelineCache()
            {
                save();
                if (cache_ != VK_NULL_HANDLE)
                    vkDestroyPipelineCache(device_.device(), cache_, nullptr);
            }

            void PipelineCache::save() const
            {
                if (cache_ == VK_NULL_HANDLE || path_.empty())
                    return;
                std::size_t size = 0;
                if (vkGetPipelineCacheData(device_.device(), cache_, &size, nullptr) != VK_SUCCESS ||
                    size == 0)
                    return;
                std::vector<char> blob(size);
                if (vkGetPipelineCacheData(device_.device(), cache_, &size, blob.data()) !=
                    VK_SUCCESS)
                    return;
                std::ofstream stream(path_, std::ios::binary | std::ios::trunc);
                if (stream)
                    stream.write(blob.data(), static_cast<std::streamsize>(size));
            }

            GraphicsPipelineFactory::GraphicsPipelineFactory(Vulkan::VulkanDevice& device,
                                                             PipelineCache& cache)
                : device_(device), cache_(cache)
            {
            }

            GraphicsPipelineFactory::~GraphicsPipelineFactory() { clear_libraries(); }

            void GraphicsPipelineFactory::clear_libraries()
            {
                std::vector<Library>* groups[] = {&vertex_input_, &pre_rasterization_,
                                                  &fragment_shader_, &fragment_output_};
                for (std::vector<Library>* group : groups)
                {
                    for (Library& library : *group)
                        if (library.pipeline != VK_NULL_HANDLE)
                            vkDestroyPipeline(device_.device(), library.pipeline, nullptr);
                    group->clear();
                }
            }

            VkPipeline GraphicsPipelineFactory::create(const GraphicsPipelineDesc& desc)
            {
                // A depth-only pipeline has no fragment stage, so three of the four
                // libraries would be empty or absent; building it whole is both simpler
                // and, for the one or two such pipelines a frame has, no slower.
                if (device_.supports_pipeline_library() && !desc.shading_rate_attachment &&
                    desc.fragment_shader != VK_NULL_HANDLE)
                {
                    const VkPipeline linked = create_linked(desc);
                    if (linked != VK_NULL_HANDLE)
                        return linked;
                }
                return create_monolithic(desc);
            }

            VkPipeline GraphicsPipelineFactory::create_compute(VkPipelineLayout layout,
                                                               VkShaderModule shader)
            {
                VkComputePipelineCreateInfo info{};
                info.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
                info.stage = shader_stage(VK_SHADER_STAGE_COMPUTE_BIT, shader);
                info.layout = layout;

                VkPipeline pipeline = VK_NULL_HANDLE;
                Vulkan::check(vkCreateComputePipelines(device_.device(), cache_.handle(), 1, &info,
                                                       nullptr, &pipeline),
                              "vkCreateComputePipelines");
                return pipeline;
            }

            VkPipeline GraphicsPipelineFactory::create_monolithic(const GraphicsPipelineDesc& desc)
            {
                VertexInputState vertex_input;
                fill_vertex_input(desc, vertex_input);
                ColorBlendState blend;
                fill_color_blend(desc, blend);
                const VkPipelineRasterizationStateCreateInfo raster = fill_rasterization(desc);
                const VkPipelineDepthStencilStateCreateInfo depth = fill_depth_stencil(desc);
                VkPipelineRenderingCreateInfo rendering = fill_rendering(desc);

                VkPipelineShaderStageCreateInfo stages[2] = {
                    shader_stage(VK_SHADER_STAGE_VERTEX_BIT, desc.vertex_shader),
                    shader_stage(VK_SHADER_STAGE_FRAGMENT_BIT, desc.fragment_shader)};

                VkPipelineViewportStateCreateInfo viewport{};
                viewport.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
                viewport.viewportCount = 1;
                viewport.scissorCount = 1;

                VkPipelineMultisampleStateCreateInfo multisample{};
                multisample.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
                multisample.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

                VkDynamicState dynamic_states[3] = {VK_DYNAMIC_STATE_VIEWPORT,
                                                    VK_DYNAMIC_STATE_SCISSOR,
                                                    VK_DYNAMIC_STATE_STENCIL_REFERENCE};
                VkPipelineDynamicStateCreateInfo dynamic{};
                dynamic.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
                dynamic.dynamicStateCount = desc.dynamic_stencil_reference ? 3u : 2u;
                dynamic.pDynamicStates = dynamic_states;

                VkGraphicsPipelineCreateInfo info{};
                info.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
                info.pNext = &rendering;
                if (desc.shading_rate_attachment)
                    info.flags |=
                        VK_PIPELINE_CREATE_RENDERING_FRAGMENT_SHADING_RATE_ATTACHMENT_BIT_KHR;
                info.stageCount = desc.fragment_shader != VK_NULL_HANDLE ? 2u : 1u;
                info.pStages = stages;
                info.pVertexInputState = &vertex_input.info;
                info.pInputAssemblyState = &vertex_input.assembly;
                info.pViewportState = &viewport;
                info.pRasterizationState = &raster;
                info.pMultisampleState = &multisample;
                info.pDepthStencilState = &depth;
                info.pColorBlendState = &blend.info;
                info.pDynamicState = &dynamic;
                info.layout = desc.layout;

                VkPipeline pipeline = VK_NULL_HANDLE;
                Vulkan::check(vkCreateGraphicsPipelines(device_.device(), cache_.handle(), 1, &info,
                                                        nullptr, &pipeline),
                              "vkCreateGraphicsPipelines");
                return pipeline;
            }

            VkPipeline GraphicsPipelineFactory::vertex_input_library(
                const GraphicsPipelineDesc& desc)
            {
                const GraphicsPipelineDesc key = vertex_input_key(desc);
                for (const Library& library : vertex_input_)
                    if (std::memcmp(&library.key, &key, sizeof(key)) == 0)
                        return library.pipeline;

                VertexInputState vertex_input;
                fill_vertex_input(desc, vertex_input);

                VkGraphicsPipelineLibraryCreateInfoEXT library_info{};
                library_info.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_LIBRARY_CREATE_INFO_EXT;
                library_info.flags =
                    VK_GRAPHICS_PIPELINE_LIBRARY_VERTEX_INPUT_INTERFACE_BIT_EXT;

                VkGraphicsPipelineCreateInfo info{};
                info.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
                info.pNext = &library_info;
                info.flags = VK_PIPELINE_CREATE_LIBRARY_BIT_KHR;
                info.pVertexInputState = &vertex_input.info;
                info.pInputAssemblyState = &vertex_input.assembly;

                VkPipeline pipeline = VK_NULL_HANDLE;
                if (vkCreateGraphicsPipelines(device_.device(), cache_.handle(), 1, &info, nullptr,
                                              &pipeline) != VK_SUCCESS)
                    return VK_NULL_HANDLE;
                vertex_input_.push_back(Library{key, pipeline});
                return pipeline;
            }

            VkPipeline GraphicsPipelineFactory::pre_rasterization_library(
                const GraphicsPipelineDesc& desc)
            {
                const GraphicsPipelineDesc key = pre_rasterization_key(desc);
                for (const Library& library : pre_rasterization_)
                    if (std::memcmp(&library.key, &key, sizeof(key)) == 0)
                        return library.pipeline;

                const VkPipelineRasterizationStateCreateInfo raster = fill_rasterization(desc);
                const VkPipelineShaderStageCreateInfo stage =
                    shader_stage(VK_SHADER_STAGE_VERTEX_BIT, desc.vertex_shader);

                VkPipelineViewportStateCreateInfo viewport{};
                viewport.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
                viewport.viewportCount = 1;
                viewport.scissorCount = 1;

                VkDynamicState dynamic_states[2] = {VK_DYNAMIC_STATE_VIEWPORT,
                                                    VK_DYNAMIC_STATE_SCISSOR};
                VkPipelineDynamicStateCreateInfo dynamic{};
                dynamic.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
                dynamic.dynamicStateCount = 2;
                dynamic.pDynamicStates = dynamic_states;

                VkGraphicsPipelineLibraryCreateInfoEXT library_info{};
                library_info.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_LIBRARY_CREATE_INFO_EXT;
                library_info.flags =
                    VK_GRAPHICS_PIPELINE_LIBRARY_PRE_RASTERIZATION_SHADERS_BIT_EXT;

                VkGraphicsPipelineCreateInfo info{};
                info.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
                info.pNext = &library_info;
                info.flags = VK_PIPELINE_CREATE_LIBRARY_BIT_KHR;
                info.stageCount = 1;
                info.pStages = &stage;
                info.pViewportState = &viewport;
                info.pRasterizationState = &raster;
                info.pDynamicState = &dynamic;
                info.layout = desc.layout;

                VkPipeline pipeline = VK_NULL_HANDLE;
                if (vkCreateGraphicsPipelines(device_.device(), cache_.handle(), 1, &info, nullptr,
                                              &pipeline) != VK_SUCCESS)
                    return VK_NULL_HANDLE;
                pre_rasterization_.push_back(Library{key, pipeline});
                return pipeline;
            }

            VkPipeline GraphicsPipelineFactory::fragment_shader_library(
                const GraphicsPipelineDesc& desc)
            {
                const GraphicsPipelineDesc key = fragment_shader_key(desc);
                for (const Library& library : fragment_shader_)
                    if (std::memcmp(&library.key, &key, sizeof(key)) == 0)
                        return library.pipeline;

                const VkPipelineDepthStencilStateCreateInfo depth = fill_depth_stencil(desc);
                const VkPipelineShaderStageCreateInfo stage =
                    shader_stage(VK_SHADER_STAGE_FRAGMENT_BIT, desc.fragment_shader);

                VkPipelineMultisampleStateCreateInfo multisample{};
                multisample.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
                multisample.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

                VkDynamicState stencil_reference = VK_DYNAMIC_STATE_STENCIL_REFERENCE;
                VkPipelineDynamicStateCreateInfo dynamic{};
                dynamic.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
                dynamic.dynamicStateCount = desc.dynamic_stencil_reference ? 1u : 0u;
                dynamic.pDynamicStates = &stencil_reference;

                VkPipelineRenderingCreateInfo rendering = fill_rendering(desc);
                VkGraphicsPipelineLibraryCreateInfoEXT library_info{};
                library_info.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_LIBRARY_CREATE_INFO_EXT;
                library_info.pNext = &rendering;
                library_info.flags = VK_GRAPHICS_PIPELINE_LIBRARY_FRAGMENT_SHADER_BIT_EXT;

                VkGraphicsPipelineCreateInfo info{};
                info.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
                info.pNext = &library_info;
                info.flags = VK_PIPELINE_CREATE_LIBRARY_BIT_KHR;
                info.stageCount = 1;
                info.pStages = &stage;
                info.pMultisampleState = &multisample;
                info.pDepthStencilState = &depth;
                info.pDynamicState = &dynamic;
                info.layout = desc.layout;

                VkPipeline pipeline = VK_NULL_HANDLE;
                if (vkCreateGraphicsPipelines(device_.device(), cache_.handle(), 1, &info, nullptr,
                                              &pipeline) != VK_SUCCESS)
                    return VK_NULL_HANDLE;
                fragment_shader_.push_back(Library{key, pipeline});
                return pipeline;
            }

            VkPipeline GraphicsPipelineFactory::fragment_output_library(
                const GraphicsPipelineDesc& desc)
            {
                const GraphicsPipelineDesc key = fragment_output_key(desc);
                for (const Library& library : fragment_output_)
                    if (std::memcmp(&library.key, &key, sizeof(key)) == 0)
                        return library.pipeline;

                ColorBlendState blend;
                fill_color_blend(desc, blend);

                VkPipelineMultisampleStateCreateInfo multisample{};
                multisample.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
                multisample.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

                VkPipelineRenderingCreateInfo rendering = fill_rendering(desc);
                VkGraphicsPipelineLibraryCreateInfoEXT library_info{};
                library_info.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_LIBRARY_CREATE_INFO_EXT;
                library_info.pNext = &rendering;
                library_info.flags = VK_GRAPHICS_PIPELINE_LIBRARY_FRAGMENT_OUTPUT_INTERFACE_BIT_EXT;

                VkGraphicsPipelineCreateInfo info{};
                info.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
                info.pNext = &library_info;
                info.flags = VK_PIPELINE_CREATE_LIBRARY_BIT_KHR;
                info.pMultisampleState = &multisample;
                info.pColorBlendState = &blend.info;

                VkPipeline pipeline = VK_NULL_HANDLE;
                if (vkCreateGraphicsPipelines(device_.device(), cache_.handle(), 1, &info, nullptr,
                                              &pipeline) != VK_SUCCESS)
                    return VK_NULL_HANDLE;
                fragment_output_.push_back(Library{key, pipeline});
                return pipeline;
            }

            VkPipeline GraphicsPipelineFactory::create_linked(const GraphicsPipelineDesc& desc)
            {
                // Four independently cached halves: two pipelines that differ only in their
                // fragment shader reuse the vertex input, pre-rasterisation, and output
                // libraries, which is what makes a permutation cheap to add.
                const VkPipeline libraries[4] = {
                    vertex_input_library(desc), pre_rasterization_library(desc),
                    fragment_shader_library(desc), fragment_output_library(desc)};
                for (VkPipeline library : libraries)
                    if (library == VK_NULL_HANDLE)
                        return VK_NULL_HANDLE;

                VkPipelineLibraryCreateInfoKHR link{};
                link.sType = VK_STRUCTURE_TYPE_PIPELINE_LIBRARY_CREATE_INFO_KHR;
                link.libraryCount = 4;
                link.pLibraries = libraries;

                VkGraphicsPipelineCreateInfo info{};
                info.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
                info.pNext = &link;
                info.layout = desc.layout;

                VkPipeline pipeline = VK_NULL_HANDLE;
                if (vkCreateGraphicsPipelines(device_.device(), cache_.handle(), 1, &info, nullptr,
                                              &pipeline) != VK_SUCCESS)
                    return VK_NULL_HANDLE;
                return pipeline;
            }
        } // namespace Resources
    } // namespace Render
} // namespace SushiEngine
