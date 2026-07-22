/**************************************************************************/
/* volumetric_fog_pass.hpp                                                */
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
 * @file volumetric_fog_pass.hpp
 * @brief Ground-hugging volumetric fog integrated into a froxel volume.
 *
 * A compute pass that marches a height-graded fog medium into a camera-frustum-aligned
 * 3D volume, reusing the aerial-perspective addressing: each froxel holds the sun and
 * ambient light scattered toward the camera through the fog and the fog's transmittance
 * out to that depth. sky.frag then folds the fog over every pixel as one fetch. The
 * volume is owned by this pass and barriered by hand; the sun's attenuation through the
 * air comes from the atmosphere transmittance LUT the pass is handed. Sun-shadowed god
 * rays and punctual-light fog are later increments that turn the single march into an
 * inject/integrate pair.
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
            class AtmosphereLutPass;

            /**
             * @brief Builds and owns the volumetric-fog froxel volume.
             *
             * Non-copyable: it owns an image, a view, and a compute pipeline.
             */
            class VolumetricFogPass final : public IRenderPass
            {
                public:
                    /**
                     * @brief Creates the fog volume and its build pipeline.
                     * @param device     The live Vulkan device.
                     * @param shaders    Library the compute module comes from.
                     * @param pipelines  Factory the compute pipeline is built through.
                     * @param atmosphere The pass owning the transmittance LUT the fog reads.
                     */
                    VolumetricFogPass(Vulkan::VulkanDevice& device,
                                      Resources::ShaderLibrary& shaders,
                                      Resources::GraphicsPipelineFactory& pipelines,
                                      AtmosphereLutPass& atmosphere);
                    ~VolumetricFogPass() override;

                    VolumetricFogPass(const VolumetricFogPass&) = delete;
                    VolumetricFogPass& operator=(const VolumetricFogPass&) = delete;

                    void register_pass(Graph::RenderGraph& graph,
                                       const Frame::FrameContext& frame) override;
                    void rebuild_pipelines() override;

                    /** @brief The fog froxel volume view the composite reads. */
                    VkImageView fog_view() const noexcept { return volume_.view; }

                private:
                    /** @brief Max local fog volumes; mirrors MAX_FOG_VOLUMES. */
                    static constexpr std::uint32_t MAX_VOLUMES = 8;

                    /** @brief Frames the local-volume uniform ring covers. */
                    static constexpr std::uint32_t RING = 3;

                    /** @brief Push block mirroring fog_scatter.comp's FogBlock. */
                    struct Push
                    {
                        float color_density[4]; /**< xyz scattering tint, w extinction. */
                        float params[4];        /**< x falloff, y ambient, z phase g, w enabled. */
                    };

                    /** @brief UBO mirroring fog_scatter.comp's FogVolumes block. */
                    struct VolumesBlock
                    {
                        float volume[MAX_VOLUMES * 3][4]; /**< centre+shape, extent+falloff, colour+density. */
                        float count[4];                   /**< x = active count. */
                    };

                    struct Volume
                    {
                        VkImage image = VK_NULL_HANDLE;
                        VmaAllocation allocation = VK_NULL_HANDLE;
                        VkImageView view = VK_NULL_HANDLE;
                        std::uint32_t size = 0;
                    };

                    void create_volume();
                    void destroy_volume();
                    void create_volume_buffers();
                    void destroy_volume_buffers();
                    void create_pipeline();
                    void destroy_pipeline();

                    Vulkan::VulkanDevice& device_;
                    Resources::ShaderLibrary& shaders_;
                    Resources::GraphicsPipelineFactory& pipelines_;
                    AtmosphereLutPass& atmosphere_;

                    Volume volume_;
                    VkBuffer volume_buffers_[RING] = {};
                    VmaAllocation volume_allocations_[RING] = {};
                    void* volume_mapped_[RING] = {};
                    VkDescriptorSetLayout set_layout_ = VK_NULL_HANDLE;
                    VkPipelineLayout pipeline_layout_ = VK_NULL_HANDLE;
                    VkPipeline pipeline_ = VK_NULL_HANDLE;
            };
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
