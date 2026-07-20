/**************************************************************************/
/* cloud_noise.hpp                                                        */
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
 * @file cloud_noise.hpp
 * @brief The volumetric cloud noise set, generated once on the GPU.
 *
 * Three tileable 3D volumes (cumuliform shape, erosion detail, anisotropic cirrus)
 * and one 2D weather map, built by compute dispatches at construction and sampled
 * thereafter by the sky and cloud passes. Generating them on the GPU rather than on
 * a CPU thread pool is what keeps bring-up off the host's critical path.
 */

#include <cstdint>

#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>

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
            class DescriptorHeap;
            class GraphicsPipelineFactory;
            class SamplerCache;
            class ShaderLibrary;
        }

        namespace Textures
        {
            /**
             * @brief Owns the four cloud noise textures and the sampler they read through.
             *
             * Non-copyable: it owns images, views, and their allocations.
             */
            class CloudNoise
            {
                public:
                    /**
                     * @brief Allocates the volumes and generates them with one fenced submit.
                     * @param device    The live Vulkan device.
                     * @param shaders   Library holding the noise compute shaders.
                     * @param pipelines Factory the compute pipelines are built through.
                     * @param samplers  Cache providing the tiling sampler.
                     * @param heap      Bindless heap the volumes are registered into.
                     */
                    CloudNoise(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                               Resources::GraphicsPipelineFactory& pipelines,
                               Resources::SamplerCache& samplers, Resources::DescriptorHeap& heap);
                    ~CloudNoise();

                    CloudNoise(const CloudNoise&) = delete;
                    CloudNoise& operator=(const CloudNoise&) = delete;

                    /** @brief The cumuliform Perlin-Worley base shape volume. */
                    VkImageView shape() const noexcept { return volumes_[SHAPE].view; }

                    /** @brief The high-frequency Worley erosion volume. */
                    VkImageView detail() const noexcept { return volumes_[DETAIL].view; }

                    /** @brief The 2D coverage/type weather map. */
                    VkImageView weather() const noexcept { return volumes_[WEATHER].view; }

                    /** @brief The wind-stretched anisotropic cirrus volume. */
                    VkImageView cirrus() const noexcept { return volumes_[CIRRUS].view; }

                    /** @brief The linear, REPEAT sampler the volumes tile under. */
                    VkSampler sampler() const noexcept { return sampler_; }

                private:
                    /** @brief Which volume a slot in the texture array holds. */
                    enum Slot : std::uint32_t
                    {
                        SHAPE = 0,
                        DETAIL = 1,
                        CIRRUS = 2,
                        WEATHER = 3,
                        SLOT_COUNT = 4,
                    };

                    /** @brief One generated texture and the heap slot it was registered in. */
                    struct Volume
                    {
                        VkImage image = VK_NULL_HANDLE;
                        VmaAllocation allocation = VK_NULL_HANDLE;
                        VkImageView view = VK_NULL_HANDLE;
                        std::uint32_t resolution = 0;
                        std::uint32_t heap_index = 0;
                        bool three_dimensional = true;
                    };

                    void create_volume(Slot slot, std::uint32_t resolution, bool three_dimensional);
                    void generate(Resources::ShaderLibrary& shaders,
                                  Resources::GraphicsPipelineFactory& pipelines);

                    Vulkan::VulkanDevice& device_;
                    Resources::DescriptorHeap& heap_;
                    Volume volumes_[SLOT_COUNT];
                    VkSampler sampler_ = VK_NULL_HANDLE;
                    VkDescriptorSetLayout set_layout_ = VK_NULL_HANDLE;
                    VkPipelineLayout pipeline_layout_ = VK_NULL_HANDLE;
                    VkDescriptorPool pool_ = VK_NULL_HANDLE;
            };
        } // namespace Textures
    } // namespace Render
} // namespace SushiEngine
