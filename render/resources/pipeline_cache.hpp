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

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include <vulkan/vulkan.h>

#include "resources/pipeline_handle.hpp"

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
             * @brief Colour-blend state applied to every colour attachment of a pipeline.
             *
             * A flat POD so it byte-compares as part of the fragment-output library key. The
             * default is no blending (@c enable false with one/zero factors), which reproduces
             * the renderer's original opaque behaviour, so a pipeline that never sets it is
             * created exactly as before. Transparent draws — additive or alpha billboards — set
             * @c enable and the factors. The same state applies to all attachments, which is all
             * the engine needs today: multi-attachment G-buffer passes leave it disabled.
             */
            struct ColorBlend
            {
                VkBool32 enable = VK_FALSE;                          /**< Whether blending is on. */
                VkBlendFactor src_color = VK_BLEND_FACTOR_ONE;      /**< Source colour factor. */
                VkBlendFactor dst_color = VK_BLEND_FACTOR_ZERO;     /**< Destination colour factor. */
                VkBlendOp color_op = VK_BLEND_OP_ADD;               /**< Colour blend op. */
                VkBlendFactor src_alpha = VK_BLEND_FACTOR_ONE;      /**< Source alpha factor. */
                VkBlendFactor dst_alpha = VK_BLEND_FACTOR_ZERO;     /**< Destination alpha factor. */
                VkBlendOp alpha_op = VK_BLEND_OP_ADD;               /**< Alpha blend op. */
            };

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
                /** @brief Task/mesh stages, for a mesh-shader pipeline (no vertex input). */
                VkShaderModule task_shader = VK_NULL_HANDLE;
                VkShaderModule mesh_shader = VK_NULL_HANDLE;

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

                /**
                 * @brief Colour-blend state; part of the fragment-output library key.
                 *
                 * Default-constructed (no blending) reproduces the original opaque behaviour, so
                 * every existing pass is unaffected. A transparent pass — an additive or alpha
                 * particle billboard — sets it. Because it keys the fragment-output library, two
                 * pipelines identical but for their blend do not alias to one cached library.
                 */
                ColorBlend blend{};

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
             * pipeline built through it. A graphics pipeline is first made available as a
             * GPL fast link — instant, but not cross-stage optimized — and a background
             * thread then rebuilds it fully optimized and atomically swaps it into the
             * PipelineHandle, so the first frames never stall and the steady state runs the
             * optimized form. Non-copyable.
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
                     * creation otherwise. A fast-linked pipeline is also queued for a
                     * background optimized rebuild; a monolithic one is already optimal and
                     * is not. The returned handle stays valid for the factory's lifetime and
                     * always resolves to the best pipeline built so far. The factory owns the
                     * underlying pipelines.
                     *
                     * @param desc What the pipeline must be.
                     * @return A handle that resolves to the pipeline at bind time.
                     */
                    PipelineHandle create(const GraphicsPipelineDesc& desc);

                    /**
                     * @brief Creates a mesh-shader pipeline (task + mesh + optional fragment).
                     *
                     * Always monolithic — the pipeline-library path is vertex-input-centric and
                     * a mesh pipeline has no vertex input at all. Used for the meshlet draw path;
                     * @c GraphicsPipelineDesc::task_shader and @c mesh_shader name the stages,
                     * and a null @c fragment_shader makes a depth-only mesh pipeline.
                     *
                     * @param desc What the pipeline must be (task/mesh/fragment + state).
                     * @return A handle that resolves to the pipeline at bind time.
                     */
                    PipelineHandle create_mesh(const GraphicsPipelineDesc& desc);

                    /**
                     * @brief Advances the optimizer's retirement clock by one frame.
                     *
                     * Called once per frame. Superseded fast-linked pipelines are destroyed
                     * here, a fixed number of frames after they were swapped out, so no
                     * in-flight command buffer is ever left binding a freed pipeline.
                     */
                    void tick();

                    /**
                     * @brief Creates a compute pipeline.
                     * @param layout  The pipeline layout.
                     * @param shader  The compute shader module.
                     * @return The created pipeline, owned by the caller.
                     */
                    VkPipeline create_compute(VkPipelineLayout layout, VkShaderModule shader);

                    /**
                     * @brief Tears down every pipeline and cached library.
                     *
                     * The caller must have idled the device. The background optimizer is
                     * quiesced first, so no half-built pipeline is left dangling, then every
                     * created pipeline, its optimized rebuild, and the four library caches
                     * are destroyed. Handed-out PipelineHandles are invalid afterward until
                     * their passes rebuild.
                     */
                    void clear_libraries();

                private:
                    /** @brief One cached pipeline library and the description subset that keyed it. */
                    struct Library
                    {
                        GraphicsPipelineDesc key{};
                        VkPipeline pipeline = VK_NULL_HANDLE;
                    };

                    /**
                     * @brief One created pipeline and its optional optimized replacement.
                     *
                     * @c active is what a PipelineHandle resolves to; it starts as @c initial
                     * (the fast link, or a monolithic build when libraries are unavailable)
                     * and the background thread flips it to @c optimized once that is built.
                     * @c initial is nulled the moment it is handed to the retirement list, so
                     * teardown never double-frees it.
                     */
                    struct Slot
                    {
                        GraphicsPipelineDesc desc{};
                        std::atomic<VkPipeline> active{VK_NULL_HANDLE};
                        VkPipeline initial = VK_NULL_HANDLE;
                        VkPipeline optimized = VK_NULL_HANDLE;
                    };

                    /** @brief A swapped-out pipeline awaiting a frames-in-flight delay. */
                    struct RetiredPipeline
                    {
                        VkPipeline pipeline = VK_NULL_HANDLE;
                        std::uint64_t retire_frame = 0;
                    };

                    VkPipeline create_monolithic(const GraphicsPipelineDesc& desc);
                    VkPipeline create_mesh_monolithic(const GraphicsPipelineDesc& desc);
                    VkPipeline create_linked(const GraphicsPipelineDesc& desc);
                    VkPipeline vertex_input_library(const GraphicsPipelineDesc& desc);
                    VkPipeline pre_rasterization_library(const GraphicsPipelineDesc& desc);
                    VkPipeline fragment_shader_library(const GraphicsPipelineDesc& desc);
                    VkPipeline fragment_output_library(const GraphicsPipelineDesc& desc);

                    void worker_main();
                    void enqueue_optimize(Slot* slot);
                    void quiesce();
                    void shutdown_worker();
                    void destroy_all();

                    Vulkan::VulkanDevice& device_;
                    PipelineCache& cache_;
                    std::vector<Library> vertex_input_;
                    std::vector<Library> pre_rasterization_;
                    std::vector<Library> fragment_shader_;
                    std::vector<Library> fragment_output_;

                    // Every VkPipeline this factory owns lives in a stable slot, so the
                    // atomic a PipelineHandle points at never moves as more are created.
                    std::vector<std::unique_ptr<Slot>> slots_;

                    // The background optimizer: worker_ drains jobs_ (fast-linked slots to
                    // rebuild optimized) under jobs_mutex_; cache_mutex_ serialises the
                    // pipeline cache across it and the main thread; retired_ holds the
                    // swapped-out pipelines the per-frame tick() destroys once safe.
                    std::thread worker_;
                    std::mutex jobs_mutex_;
                    std::condition_variable jobs_cv_;
                    std::condition_variable idle_cv_;
                    std::queue<Slot*> jobs_;
                    bool busy_ = false;
                    bool stop_ = false;
                    std::mutex cache_mutex_;
                    std::mutex retire_mutex_;
                    std::vector<RetiredPipeline> retired_;
                    std::atomic<std::uint64_t> frame_counter_{0};
            };
        } // namespace Resources
    } // namespace Render
} // namespace SushiEngine
