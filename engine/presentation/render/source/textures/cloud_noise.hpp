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
 * Four tileable 3D volumes (cumuliform shape, erosion detail, anisotropic cirrus,
 * and the view march's combined carve volume) and one 2D weather map, built by
 * compute dispatches at construction and sampled thereafter by the sky and cloud
 * passes. Generating them on the GPU rather than on a CPU thread pool is what keeps
 * bring-up off the host's critical path.
 *
 * The carve volume is the one exception to "generate and forget": it alone carries a mip
 * chain, because it alone is sampled at a world scale fixed in metres by a march whose
 * step grows with distance. The same bring-up submit that builds the chain reads its
 * finest level back, so the host can measure what each level's filter removed — see
 * @ref CloudNoise::march_carve_spread.
 */

#include <cstdint>
#include <vector>

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
             * @brief Owns the five cloud noise textures and the samplers they read through.
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

                    /**
                     * @brief The view march's combined carve volume (CloudsV2).
                     *
                     * One volume carrying everything the per-sample analytic carve needs —
                     * r = CDF-uniformised base shape, g = erosion fbm, b = incommensurate
                     * fine-erosion fbm, a = curl-warp potential — because the march has
                     * exactly one free noise binding to read it through.
                     */
                    VkImageView march() const noexcept { return volumes_[MARCH].view; }

                    /** @brief The linear, REPEAT sampler the volumes tile under. */
                    VkSampler sampler() const noexcept { return sampler_; }

                    /**
                     * @brief The march volume's own sampler, reaching its whole mip chain.
                     *
                     * Separate from @ref sampler only because a sampler's `maxLod` is part of
                     * its identity: the shared one stops at level 0, which is correct for the
                     * four single-level volumes and would silently pin the carve to its finest
                     * level. Trilinear between levels, so a march crossing a LOD boundary
                     * along one ray sees the band limit change continuously rather than
                     * stepping.
                     */
                    VkSampler march_sampler() const noexcept { return march_sampler_; }

                    /** @brief How many mip levels the march volume carries. */
                    std::uint32_t march_mip_count() const noexcept
                    {
                        return static_cast<std::uint32_t>(volumes_[MARCH].mip_views.size());
                    }

                    /**
                     * @brief Per-mip standard deviation of the shape the filter removed.
                     *
                     * Index @c l holds `sqrt(Var(level 0) - Var(level l))` for the march
                     * volume's r channel — the spread of the sub-texel detail a fetch at level
                     * @c l no longer carries. Measured from the generated volume rather than
                     * assumed, because the carve's coverage threshold has to be restated
                     * against it: a filtered field is narrower than the uniform one the
                     * threshold was calibrated on, and thresholding the mean alone would turn
                     * every distant column all-cloud or all-clear (see cloud.frag's
                     * `carve_shape`). Entry 0 is exactly zero by construction.
                     *
                     * @return The spreads, one per level, in level order.
                     */
                    const std::vector<float>& march_carve_spread() const noexcept
                    {
                        return march_carve_spread_;
                    }

                private:
                    /** @brief Which volume a slot in the texture array holds. */
                    enum Slot : std::uint32_t
                    {
                        SHAPE = 0,
                        DETAIL = 1,
                        CIRRUS = 2,
                        WEATHER = 3,
                        // Appended after WEATHER so the slot indices the generator pushes as
                        // `kind` stay stable; cloud_noise_volume.comp names 4 explicitly.
                        MARCH = 4,
                        SLOT_COUNT = 5,
                    };

                    /** @brief One generated texture and the heap slot it was registered in. */
                    struct Volume
                    {
                        VkImage image = VK_NULL_HANDLE;
                        VmaAllocation allocation = VK_NULL_HANDLE;
                        /** @brief The sampled view, covering every level the volume has. */
                        VkImageView view = VK_NULL_HANDLE;
                        /**
                         * @brief One single-level storage view per mip, in level order.
                         *
                         * A storage image binding names exactly one level, so the generator
                         * writes through @c mip_views[0] and the downsample chain reads and
                         * writes consecutive pairs. Every volume has at least one entry, which
                         * is what lets the generation path stay free of special cases.
                         */
                        std::vector<VkImageView> mip_views;
                        std::uint32_t resolution = 0;
                        std::uint32_t heap_index = 0;
                        bool three_dimensional = true;
                    };

                    void create_volume(Slot slot, std::uint32_t resolution, bool three_dimensional,
                                       std::uint32_t mip_levels);
                    void generate(Resources::ShaderLibrary& shaders,
                                  Resources::GraphicsPipelineFactory& pipelines);
                    void measure_carve_spread(const void* level_zero, std::uint32_t resolution);

                    Vulkan::VulkanDevice& device_;
                    Resources::DescriptorHeap& heap_;
                    Volume volumes_[SLOT_COUNT];
                    VkSampler sampler_ = VK_NULL_HANDLE;
                    VkSampler march_sampler_ = VK_NULL_HANDLE;
                    std::vector<float> march_carve_spread_;
                    VkDescriptorSetLayout set_layout_ = VK_NULL_HANDLE;
                    VkPipelineLayout pipeline_layout_ = VK_NULL_HANDLE;
                    VkDescriptorSetLayout mip_set_layout_ = VK_NULL_HANDLE;
                    VkPipelineLayout mip_pipeline_layout_ = VK_NULL_HANDLE;
                    VkDescriptorPool pool_ = VK_NULL_HANDLE;
            };
        } // namespace Textures
    } // namespace Render
} // namespace SushiEngine
