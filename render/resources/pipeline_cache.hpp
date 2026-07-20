/**************************************************************************/
/* pipeline_cache.hpp                                                     */
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

#pragma once

/**
 * @file pipeline_cache.hpp
 * @brief Pipeline construction: a disk-backed VkPipelineCache and a GPL builder.
 *
 * Two mechanisms, one purpose — never pay for a pipeline twice. The disk cache
 * carries the driver's compiled forms across process runs, so a second launch skips
 * shader compilation entirely. The factory describes a pipeline as data and, where
 * VK_EXT_graphics_pipeline_library is available, builds it from four independently
 * cached libraries (vertex input, pre-rasterisation, fragment shader, fragment
 * output) so pipelines that differ in only one of those four reuse the other three.
 * Devices without the extension fall back to monolithic creation, which is why no
 * caller ever branches on support.
 */

#include <cstdint>
#include <string>
#include <vector>

#include <vulkan/vulkan.h>

namespace SushiEngine
{
    namespace Render
    {
        namespace Vulkan
        {
            class VulkanDevice;
        }

        namespace Resources
        {
            /**
             * @brief A VkPipelineCache persisted to disk across runs.
             *
             * The blob is loaded in the constructor and written back by save(); the
             * driver validates its own header, so a stale or foreign blob is rejected
             * harmlessly rather than corrupting anything. Non-copyable.
             */
            class PipelineCache
            {
                public:
                    /**
                     * @brief Creates the cache, seeding it from @p path if readable.
                     * @param device The live Vulkan device.
                     * @param path   File the blob is read from and written back to.
                     */
                    PipelineCache(Vulkan::VulkanDevice& device, std::string path);
                    ~PipelineCache();

                    PipelineCache(const PipelineCache&) = delete;
                    PipelineCache& operator=(const PipelineCache&) = delete;

                    /** @brief The handle to pass to every pipeline creation call. */
                    VkPipelineCache handle() const noexcept { return cache_; }

                    /** @brief Writes the driver's current cache blob back to disk. */
                    void save() const;

                private:
                    Vulkan::VulkanDevice& device_;
                    std::string path_;
                    VkPipelineCache cache_ = VK_NULL_HANDLE;
            };

            /** @brief One vertex attribute in a pipeline description. */
            struct VertexAttribute
            {
                std::uint32_t location = 0;
                VkFormat format = VK_FORMAT_UNDEFINED;
                std::uint32_t offset = 0;
            };

            /** @brief The largest vertex attribute count a pipeline description carries. */
            constexpr std::uint32_t MAX_VERTEX_ATTRIBUTES = 8;

            /** @brief The largest colour attachment count a pipeline description carries. */
            constexpr std::uint32_t MAX_PIPELINE_COLOR_ATTACHMENTS = 8;

            /**
             * @brief A graphics pipeline expressed as plain data.
             *
             * Deliberately a flat POD: it is both the creation parameter and — split into
             * four subsets — the cache key for the pipeline libraries, so it must be
             * comparable byte for byte.
             */
            struct GraphicsPipelineDesc
            {
                VkPipelineLayout layout = VK_NULL_HANDLE;
                VkShaderModule vertex_shader = VK_NULL_HANDLE;
                VkShaderModule fragment_shader = VK_NULL_HANDLE;

                std::uint32_t vertex_stride = 0; /**< Zero means the pipeline fetches no vertices. */
                VertexAttribute attributes[MAX_VERTEX_ATTRIBUTES]{};
                std::uint32_t attribute_count = 0;
                VkPrimitiveTopology topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

                VkPolygonMode polygon_mode = VK_POLYGON_MODE_FILL;
                VkCullModeFlags cull_mode = VK_CULL_MODE_NONE;
                VkFrontFace front_face = VK_FRONT_FACE_COUNTER_CLOCKWISE;
                float line_width = 1.0f;
                VkBool32 depth_bias_enable = VK_FALSE;
                float depth_bias_constant = 0.0f;
                float depth_bias_slope = 0.0f;

                VkBool32 depth_test = VK_FALSE;
                VkBool32 depth_write = VK_FALSE;
                VkCompareOp depth_compare = VK_COMPARE_OP_GREATER_OR_EQUAL;
                VkBool32 stencil_test = VK_FALSE;
                VkStencilOpState stencil{};

                VkFormat color_formats[MAX_PIPELINE_COLOR_ATTACHMENTS]{};
                std::uint32_t color_count = 0;
                VkFormat depth_format = VK_FORMAT_UNDEFINED;
                VkFormat stencil_format = VK_FORMAT_UNDEFINED;

                VkBool32 dynamic_stencil_reference = VK_FALSE;

                /**
                 * @brief The pass this pipeline runs in may bind a shading rate image.
                 *
                 * Vulkan requires the pipeline to have been created knowing this, so it
                 * cannot be decided at record time. Setting it also forces monolithic
                 * creation: the flag has to agree across all four pipeline libraries and
                 * their link, and a pipeline that opts into variable-rate shading is
                 * created once at start-up, so the library reuse buys nothing here.
                 */
                VkBool32 shading_rate_attachment = VK_FALSE;
            };

            /**
             * @brief Builds graphics and compute pipelines, reusing everything it can.
             *
             * Owns the pipeline libraries it creates, so its lifetime must cover every
             * pipeline built through it. Non-copyable.
             */
            class GraphicsPipelineFactory
            {
                public:
                    /**
                     * @brief Binds the factory to a device and the cache it compiles into.
                     * @param device The live Vulkan device.
                     * @param cache  Disk-backed cache every creation call passes.
                     */
                    GraphicsPipelineFactory(Vulkan::VulkanDevice& device, PipelineCache& cache);
                    ~GraphicsPipelineFactory();

                    GraphicsPipelineFactory(const GraphicsPipelineFactory&) = delete;
                    GraphicsPipelineFactory& operator=(const GraphicsPipelineFactory&) = delete;

                    /**
                     * @brief Creates a graphics pipeline from its description.
                     *
                     * Uses pipeline libraries when the device supports them, monolithic
                     * creation otherwise; the returned pipeline is identical either way and
                     * is owned by the caller.
                     *
                     * @param desc What the pipeline must be.
                     * @return The created pipeline.
                     */
                    VkPipeline create(const GraphicsPipelineDesc& desc);

                    /**
                     * @brief Creates a compute pipeline.
                     * @param layout  The pipeline layout.
                     * @param shader  The compute shader module.
                     * @return The created pipeline, owned by the caller.
                     */
                    VkPipeline create_compute(VkPipelineLayout layout, VkShaderModule shader);

                    /** @brief Destroys every cached library; the caller must have idled the device. */
                    void clear_libraries();

                private:
                    /** @brief One cached pipeline library and the description subset that keyed it. */
                    struct Library
                    {
                        GraphicsPipelineDesc key{};
                        VkPipeline pipeline = VK_NULL_HANDLE;
                    };

                    VkPipeline create_monolithic(const GraphicsPipelineDesc& desc);
                    VkPipeline create_linked(const GraphicsPipelineDesc& desc);
                    VkPipeline vertex_input_library(const GraphicsPipelineDesc& desc);
                    VkPipeline pre_rasterization_library(const GraphicsPipelineDesc& desc);
                    VkPipeline fragment_shader_library(const GraphicsPipelineDesc& desc);
                    VkPipeline fragment_output_library(const GraphicsPipelineDesc& desc);

                    Vulkan::VulkanDevice& device_;
                    PipelineCache& cache_;
                    std::vector<Library> vertex_input_;
                    std::vector<Library> pre_rasterization_;
                    std::vector<Library> fragment_shader_;
                    std::vector<Library> fragment_output_;
            };
        } // namespace Resources
    } // namespace Render
} // namespace SushiEngine
