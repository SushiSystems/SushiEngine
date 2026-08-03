/**************************************************************************/
/* cloud_panorama_pass.hpp                                               */
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
 * @file cloud_panorama_pass.hpp
 * @brief The W3 far-field impostor: the cloudscape marched into a compressed panorama.
 *
 * Design doc §4.6: beyond the primary view's own march range and for reflection-probe
 * capture, a march per screen pixel (or per probe face) is not affordable. This bakes
 * the same T3 field + amortized light volume into a 512x256 equirectangular panorama,
 * refreshed a row group at a time across `AMORTIZE_FRAMES` frames — the same cadence
 * `CloudLightVolumePass`/`CloudShadowMapPass` already refresh at, since all three are
 * driven by the sun/camera drifting continuously rather than a discrete author edit.
 * The image is pass-owned and barriered by hand, exactly as those two are.
 *
 * Landed as a verified, standalone bake this phase; wiring a consumer (the reflection
 * probe capture in `IblPass`) is scoped out — see the W3 CHANGELOG entry for why.
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

        namespace Vulkan
        {
            class VulkanDevice;
        }

        namespace Passes
        {
            class CloudscapeCompilePass;
            class CloudLightVolumePass;

            /**
             * @brief Builds and owns the baked cloud panorama.
             *
             * Non-copyable: it owns an image, a view, and a compute pipeline.
             */
            class CloudPanoramaPass final : public IRenderPass
            {
                public:
                    /**
                     * @brief Builds the panorama image and its bake pipeline.
                     * @param device       The live Vulkan device.
                     * @param shaders      Library the bake compute module comes from.
                     * @param pipelines    Factory the compute pipeline is built through.
                     * @param samplers     Cache providing the panorama's sampler.
                     * @param cloudscape   The pass that owns the two cloudscape windows this bake reads.
                     * @param light_volume The pass that owns the baked near-window light volume.
                     */
                    CloudPanoramaPass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                                      Resources::GraphicsPipelineFactory& pipelines,
                                      Resources::SamplerCache& samplers,
                                      CloudscapeCompilePass& cloudscape,
                                      CloudLightVolumePass& light_volume);
                    ~CloudPanoramaPass() override;

                    CloudPanoramaPass(const CloudPanoramaPass&) = delete;
                    CloudPanoramaPass& operator=(const CloudPanoramaPass&) = delete;

                    void register_pass(Graph::RenderGraph& graph,
                                       const Frame::FrameContext& frame) override;
                    void rebuild_pipelines() override;

                    /** @brief The baked panorama a future consumer samples by direction. */
                    VkImageView view() const noexcept { return panorama_.view; }

                    /** @brief The linear, wrap-in-U/clamp-in-V sampler the panorama reads through. */
                    VkSampler sampler() const noexcept { return sampler_; }

                private:
                    /** @brief The baked panorama: its image, view, and allocation. */
                    struct Panorama
                    {
                        VkImage image = VK_NULL_HANDLE;
                        VmaAllocation allocation = VK_NULL_HANDLE;
                        VkImageView view = VK_NULL_HANDLE;
                    };

                    /** @brief The bake shader's push block. */
                    struct Push
                    {
                        std::uint32_t row_start;
                        std::uint32_t row_count;
                    };

                    void create_panorama();
                    void destroy_panorama();
                    void create_pipeline();
                    void destroy_pipeline();

                    /** @brief Fixed, tier-independent bake resolution (2:1 equirectangular). */
                    static constexpr std::uint32_t WIDTH = 512;
                    static constexpr std::uint32_t HEIGHT = 256;
                    /** @brief How many frames a full refresh cycle spreads across. */
                    static constexpr std::uint32_t AMORTIZE_FRAMES = 8;
                    static constexpr std::uint32_t ROW_COUNT = HEIGHT / AMORTIZE_FRAMES;

                    Vulkan::VulkanDevice& device_;
                    Resources::ShaderLibrary& shaders_;
                    Resources::GraphicsPipelineFactory& pipelines_;
                    CloudscapeCompilePass& cloudscape_;
                    CloudLightVolumePass& light_volume_;

                    Panorama panorama_;
                    VkSampler sampler_ = VK_NULL_HANDLE;

                    VkDescriptorSetLayout set_layout_ = VK_NULL_HANDLE;
                    VkPipelineLayout pipeline_layout_ = VK_NULL_HANDLE;
                    VkPipeline pipeline_ = VK_NULL_HANDLE;

                    std::uint32_t frame_counter_ = 0;
                    bool built_ = false;
            };
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
