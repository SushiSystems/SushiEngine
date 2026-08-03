/**************************************************************************/
/* sampler_cache.hpp                                                      */
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
 * @file sampler_cache.hpp
 * @brief One VkSampler per distinct sampler description, shared by every user.
 *
 * Samplers are immutable, cheap, and few; the only thing worth avoiding is
 * creating the same one twice and having to track two lifetimes for it. Every
 * sampler in the renderer is requested through this cache, which owns them all.
 */

#include <cstdint>
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
             * @brief What a sampler must be, as a comparable POD.
             *
             * Value-initialised so padding is zero and equality is a byte comparison.
             */
            struct SamplerDesc
            {
                VkFilter filter = VK_FILTER_LINEAR;
                VkSamplerMipmapMode mipmap_mode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
                VkSamplerAddressMode address_mode = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
                float max_anisotropy = 1.0f; /**< Values above 1 enable anisotropic filtering. */
                float max_lod = 0.0f;        /**< Highest mip the sampler may reach. */

                /**
                 * @brief Compare the sample against a reference instead of returning it.
                 *
                 * What makes a shadow map filterable: the hardware compares each texel
                 * to the reference depth and returns the filtered *fraction* that passed,
                 * so one bilinear fetch is already a 2x2 percentage-closer filter.
                 */
                VkBool32 compare_enable = VK_FALSE;
                VkCompareOp compare_op = VK_COMPARE_OP_LESS_OR_EQUAL;

                /** @brief Colour returned outside a CLAMP_TO_BORDER address range. */
                VkBorderColor border_color = VK_BORDER_COLOR_FLOAT_OPAQUE_BLACK;
            };

            /**
             * @brief Creates samplers on demand and hands the same one back thereafter.
             *
             * Non-copyable: it owns the samplers it creates, which stay alive until it
             * is destroyed.
             */
            class SamplerCache
            {
                public:
                    /**
                     * @brief Binds the cache to the device it creates samplers on.
                     * @param device The live Vulkan device.
                     */
                    explicit SamplerCache(Vulkan::VulkanDevice& device);
                    ~SamplerCache();

                    SamplerCache(const SamplerCache&) = delete;
                    SamplerCache& operator=(const SamplerCache&) = delete;

                    /**
                     * @brief The sampler matching a description, created on first request.
                     *
                     * The requested anisotropy is clamped to the device limit, so a caller
                     * may ask for more than the hardware offers without checking.
                     *
                     * @param desc What the sampler must be.
                     * @return The shared sampler; never destroyed by the caller.
                     */
                    VkSampler get(const SamplerDesc& desc);

                private:
                    /** @brief One cached sampler and the description that keyed it. */
                    struct Entry
                    {
                        SamplerDesc desc{};
                        VkSampler sampler = VK_NULL_HANDLE;
                    };

                    Vulkan::VulkanDevice& device_;
                    std::vector<Entry> entries_;
                    float max_anisotropy_ = 1.0f;
            };
        } // namespace Resources
    } // namespace Render
} // namespace SushiEngine
