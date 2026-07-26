/**************************************************************************/
/* cloudscape_compile_pass.hpp                                           */
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
 * @file cloudscape_compile_pass.hpp
 * @brief The cloudscape T3 bake: the deck stack compiled into a sampled density field.
 *
 * Bakes the authored cloud deck stack's six-deck genus loop into a 3D density field once
 * per change instead of once per march sample, so the view march (CloudPass) spends a
 * couple of fetches where it used to spend up to ~48 (8 texture reads x up to 6 decks).
 * Also bakes a coarser max-pooled copy the march's cheap/coarse probe reads. Change-gated
 * like AtmosphereLutPass's static LUTs: a POD snapshot of the deck stack is memcmp'd
 * against the last bake, and the (amortized, but not free) rebake dispatch only runs when
 * something an author actually touched changed. The images are private to this pass and
 * barriered by hand, exactly as the atmosphere LUTs are; the render graph only schedules
 * the pass, and only when there is something to rebake.
 */

#include "passes/render_pass.hpp"

#include <cstdint>

#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>

namespace SushiEngine
{
    namespace Render
    {
        namespace Resources
        {
            class GraphicsPipelineFactory;
            class SamplerCache;
            class ShaderLibrary;
        }

        namespace Textures
        {
            class CloudNoise;
        }

        namespace Vulkan
        {
            class VulkanDevice;
        }

        namespace Passes
        {
            /**
             * @brief Builds and owns the baked cloudscape density field and its skip field.
             *
             * Non-copyable: it owns images, views, and compute pipelines.
             */
            class CloudscapeCompilePass final : public IRenderPass
            {
                public:
                    /**
                     * @brief Builds the field images and the two bake pipelines.
                     * @param device    The live Vulkan device.
                     * @param shaders   Library the compute modules come from.
                     * @param pipelines Factory the compute pipelines are built through.
                     * @param samplers  Cache providing the field's tiling sampler.
                     * @param noise     The cloud noise volumes the bake samples.
                     */
                    CloudscapeCompilePass(Vulkan::VulkanDevice& device,
                                          Resources::ShaderLibrary& shaders,
                                          Resources::GraphicsPipelineFactory& pipelines,
                                          Resources::SamplerCache& samplers,
                                          Textures::CloudNoise& noise);
                    ~CloudscapeCompilePass() override;

                    CloudscapeCompilePass(const CloudscapeCompilePass&) = delete;
                    CloudscapeCompilePass& operator=(const CloudscapeCompilePass&) = delete;

                    void register_pass(Graph::RenderGraph& graph,
                                       const Frame::FrameContext& frame) override;
                    void rebuild_pipelines() override;

                    /** @brief The fine density field the view march samples: rg = layer0/1. */
                    VkImageView field_view() const noexcept { return field_.view; }

                    /** @brief The max-pooled downsample the march's coarse probe samples. */
                    VkImageView skip_view() const noexcept { return skip_.view; }

                    /** @brief The linear, REPEAT sampler both fields tile under. */
                    VkSampler sampler() const noexcept { return sampler_; }

                    /** @brief The flat XZ tile size the field wraps, metres — for cloud.frag's UV. */
                    static float tile_meters() noexcept { return TILE_METERS; }

                    /**
                     * @brief World size of one skip-field texel, metres, XZ axes.
                     *
                     * The Nubis3 step rule's `skip_distance` term: when the coarse probe
                     * reads empty space, the march can safely advance by a whole skip cell
                     * without missing a wisp the fine field could see, since the skip field
                     * is a max-pool over exactly this many fine texels.
                     */
                    static float skip_cell_meters() noexcept
                    {
                        return TILE_METERS / static_cast<float>(FIELD_RESOLUTION_XZ / SKIP_DOWNSAMPLE_XZ);
                    }

                private:
                    /** @brief One baked volume: its image, view, and allocation. */
                    struct Volume
                    {
                        VkImage image = VK_NULL_HANDLE;
                        VmaAllocation allocation = VK_NULL_HANDLE;
                        VkImageView view = VK_NULL_HANDLE;
                        std::uint32_t width = 0;
                        std::uint32_t height = 0;
                        std::uint32_t depth = 0;
                    };

                    /** @brief One deck's memcmp-relevant authored state. */
                    struct DeckSnapshot
                    {
                        std::uint32_t enabled = 0;
                        std::uint32_t genus = 0;
                        float coverage_bias = 0.0f;
                        float density_scale = 0.0f;
                    };

                    /** @brief What the bake depends on, compared frame to frame to gate it. */
                    struct Snapshot
                    {
                        DeckSnapshot decks[6]; /**< Mirrors Render::CLOUD_MAX_DECKS. */
                        float weather_scale = 0.0f;
                    };

                    /** @brief The bake shader's push block. */
                    struct Push
                    {
                        float tile_meters;
                    };

                    void create_volume(Volume& volume, std::uint32_t width, std::uint32_t height,
                                       std::uint32_t depth);
                    void destroy_volume(Volume& volume);
                    void create_pipelines();
                    void destroy_pipelines();
                    bool cloudscape_changed(const Snapshot& snapshot);

                    /** @brief The wrapped, wind-neutral tile the field covers, metres per axis. */
                    static constexpr float TILE_METERS = 32768.0f;

                    // Fixed at construction, like the atmosphere LUTs and the cloud noise
                    // volumes: this is baked infrastructure sized once at renderer setup, not
                    // a per-frame cost the quality tier scales (that's the march step counts).
                    static constexpr std::uint32_t FIELD_RESOLUTION_XZ = 256;
                    static constexpr std::uint32_t FIELD_RESOLUTION_Y = 32;
                    static constexpr std::uint32_t SKIP_DOWNSAMPLE_XZ = 4;
                    static constexpr std::uint32_t SKIP_DOWNSAMPLE_Y = 2;

                    Vulkan::VulkanDevice& device_;
                    Resources::ShaderLibrary& shaders_;
                    Resources::GraphicsPipelineFactory& pipelines_;
                    Textures::CloudNoise& noise_;

                    Volume field_;
                    Volume skip_;
                    VkSampler sampler_ = VK_NULL_HANDLE;

                    VkDescriptorSetLayout field_layout_ = VK_NULL_HANDLE;
                    VkPipelineLayout field_pipeline_layout_ = VK_NULL_HANDLE;
                    VkPipeline field_pipeline_ = VK_NULL_HANDLE;
                    VkDescriptorSetLayout skip_layout_ = VK_NULL_HANDLE;
                    VkPipelineLayout skip_pipeline_layout_ = VK_NULL_HANDLE;
                    VkPipeline skip_pipeline_ = VK_NULL_HANDLE;

                    Snapshot last_snapshot_{};
                    bool built_ = false;
            };
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
