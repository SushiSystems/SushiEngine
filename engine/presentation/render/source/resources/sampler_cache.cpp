/**************************************************************************/
/* sampler_cache.cpp                                                      */
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

#include "resources/sampler_cache.hpp"

#include <algorithm>
#include <cstring>

#include "rhi/vulkan/vulkan_check.hpp"
#include "rhi/vulkan/vulkan_device.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Resources
        {
            SamplerCache::SamplerCache(Vulkan::VulkanDevice& device) : device_(device)
            {
                VkPhysicalDeviceProperties properties{};
                vkGetPhysicalDeviceProperties(device_.physical_device(), &properties);
                max_anisotropy_ = properties.limits.maxSamplerAnisotropy;
            }

            SamplerCache::~SamplerCache()
            {
                for (Entry& entry : entries_)
                    if (entry.sampler != VK_NULL_HANDLE)
                        vkDestroySampler(device_.device(), entry.sampler, nullptr);
                entries_.clear();
            }

            VkSampler SamplerCache::get(const SamplerDescription& description)
            {
                for (const Entry& entry : entries_)
                    if (std::memcmp(&entry.description, &description, sizeof(SamplerDescription)) ==
                        0)
                        return entry.sampler;

                const float anisotropy = std::min(description.max_anisotropy, max_anisotropy_);

                VkSamplerCreateInfo info{};
                info.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
                info.magFilter = description.filter;
                info.minFilter = description.filter;
                info.mipmapMode = description.mipmap_mode;
                info.addressModeU = description.address_mode;
                info.addressModeV = description.address_mode;
                info.addressModeW = description.address_mode;
                info.anisotropyEnable = anisotropy > 1.0f ? VK_TRUE : VK_FALSE;
                info.maxAnisotropy = anisotropy;
                info.maxLod = description.max_lod;
                info.compareEnable = description.compare_enable;
                info.compareOp = description.compare_op;
                info.borderColor = description.border_color;

                Entry entry;
                entry.description = description;
                Vulkan::check(vkCreateSampler(device_.device(), &info, nullptr, &entry.sampler),
                              "vkCreateSampler");
                entries_.push_back(entry);
                return entry.sampler;
            }
        } // namespace Resources
    } // namespace Render
} // namespace SushiEngine
