/**************************************************************************/
/* atmosphere_lut_pass.hpp                                                */
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
 * @file atmosphere_lut_pass.hpp
 * @brief The view-independent half of the Hillaire 2020 atmosphere LUT stack.
 *
 * Builds and owns two small textures the sky march reads instead of integrating them
 * per pixel: the transmittance LUT (optical depth from any altitude and sun angle to
 * the top of the atmosphere) and the multiple-scattering LUT (the infinite-order
 * isotropic scattering the single march omits). Neither depends on the view or the sun
 * direction — only on the medium coefficients and the planet radii — so the pass is
 * change-gated: it rebuilds only when those inputs move, and otherwise leaves last
 * frame's LUTs in place for the sky and IBL passes to sample. The images are private
 * to this pass and barriered by hand, exactly as the hi-Z pyramid and the IBL cubes
 * are; the render graph only schedules the pass.
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
            class ShaderLibrary;
        }

        namespace Vulkan
        {
            class VulkanDevice;
        }

        namespace Passes
        {
            /**
             * @brief Builds and owns the transmittance and multiple-scattering LUTs.
             *
             * Non-copyable: it owns images, views, and compute pipelines.
             */
            class AtmosphereLutPass final : public IRenderPass
            {
                public:
                    /**
                     * @brief Creates the LUT images and the two build pipelines.
                     * @param device    The live Vulkan device.
                     * @param shaders   Library the compute modules come from.
                     * @param pipelines Factory the compute pipelines are built through.
                     */
                    AtmosphereLutPass(Vulkan::VulkanDevice& device,
                                      Resources::ShaderLibrary& shaders,
                                      Resources::GraphicsPipelineFactory& pipelines);
                    ~AtmosphereLutPass() override;

                    AtmosphereLutPass(const AtmosphereLutPass&) = delete;
                    AtmosphereLutPass& operator=(const AtmosphereLutPass&) = delete;

                    void register_pass(Graph::RenderGraph& graph,
                                       const Frame::FrameContext& frame) override;
                    void rebuild_pipelines() override;

                    /** @brief The transmittance LUT view sky.frag and the IBL capture sample. */
                    VkImageView transmittance_view() const noexcept { return transmittance_.view; }

                    /** @brief The multiple-scattering LUT view the atmosphere march samples. */
                    VkImageView multiscatter_view() const noexcept { return multiscatter_.view; }

                    /** @brief The per-frame sky-view LUT the background sky reads directly. */
                    VkImageView sky_view_view() const noexcept { return sky_view_.view; }

                    /** @brief The per-frame aerial-perspective froxel volume mesh pixels read. */
                    VkImageView aerial_view() const noexcept { return aerial_.view; }

                private:
                    /** @brief Push block mirroring the static LUT shaders' AtmosBlock. */
                    struct Push
                    {
                        float rayleigh[4]; /**< xyz Rayleigh scattering, w Mie scattering. */
                        float heights[4];  /**< x Rayleigh h, y Mie h, z bottom, w top. */
                        float extra[4];    /**< x Mie extinction. */
                    };

                    /** @brief Push block mirroring sky_view_lut.comp's SkyViewBlock. */
                    struct SkyViewPush
                    {
                        float rayleigh[4]; /**< xyz Rayleigh scattering, w Mie scattering. */
                        float heights[4];  /**< x Rayleigh h, y Mie h, z bottom, w top. */
                        float extra[4];    /**< x Mie extinction, y Mie anisotropy g. */
                        float center[4];   /**< xyz planet centre, camera-relative. */
                        float sun[4];      /**< xyz unit direction to the sun, camera-relative. */
                    };

                    /** @brief One LUT or volume: its image, sampled view, and allocation. */
                    struct Lut
                    {
                        VkImage image = VK_NULL_HANDLE;
                        VmaAllocation allocation = VK_NULL_HANDLE;
                        VkImageView view = VK_NULL_HANDLE;
                        std::uint32_t width = 0;
                        std::uint32_t height = 0;
                        std::uint32_t depth = 1; /**< >1 makes it a 3D volume. */
                    };

                    void create_image(Lut& lut, std::uint32_t width, std::uint32_t height);
                    void create_volume(Lut& lut, std::uint32_t width, std::uint32_t height,
                                       std::uint32_t depth);
                    void destroy_image(Lut& lut);
                    void create_pipelines();
                    void destroy_pipelines();
                    bool medium_changed(const Push& push);

                    Vulkan::VulkanDevice& device_;
                    Resources::ShaderLibrary& shaders_;
                    Resources::GraphicsPipelineFactory& pipelines_;

                    Lut transmittance_;
                    Lut multiscatter_;
                    Lut sky_view_;
                    Lut aerial_;

                    VkDescriptorSetLayout transmittance_layout_ = VK_NULL_HANDLE;
                    VkPipelineLayout transmittance_pipeline_layout_ = VK_NULL_HANDLE;
                    VkPipeline transmittance_pipeline_ = VK_NULL_HANDLE;
                    VkDescriptorSetLayout multiscatter_layout_ = VK_NULL_HANDLE;
                    VkPipelineLayout multiscatter_pipeline_layout_ = VK_NULL_HANDLE;
                    VkPipeline multiscatter_pipeline_ = VK_NULL_HANDLE;
                    VkDescriptorSetLayout sky_view_layout_ = VK_NULL_HANDLE;
                    VkPipelineLayout sky_view_pipeline_layout_ = VK_NULL_HANDLE;
                    VkPipeline sky_view_pipeline_ = VK_NULL_HANDLE;
                    VkDescriptorSetLayout aerial_layout_ = VK_NULL_HANDLE;
                    VkPipelineLayout aerial_pipeline_layout_ = VK_NULL_HANDLE;
                    VkPipeline aerial_pipeline_ = VK_NULL_HANDLE;

                    Push last_push_{};
                    bool built_ = false;
            };
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
