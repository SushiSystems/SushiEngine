/**************************************************************************/
/* weather_field_pass.hpp                                                 */
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
 * @file weather_field_pass.hpp
 * @brief Uploads the simulation's spatial weather field for the cloud passes to read.
 *
 * The render-side half of `docs/slop/atmosphere_system.md` §7: the simulation publishes a
 * horizontal grid of coverage/density/type/precipitation (`Render::WeatherField`), and the
 * cloud march reads it per sample to decide where cloud actually is. This pass is only the
 * transport — it owns the image, uploads when the field's revision changes, and hands the
 * view to whoever binds it. It computes nothing.
 *
 * A **3D image**, not a 2D array, deliberately: the three vertical bands need to blend as a
 * march sample climbs through them, and hardware trilinear filtering does that in the same
 * single fetch the horizontal interpolation already costs. A 2D array cannot filter across
 * layers, so it would double the fetch count for exactly this.
 *
 * The image is always the full `WEATHER_FIELD_MAX_CELLS` square regardless of what the
 * producer publishes — a provider with a coarser field (a station blend, or a uniform
 * authored sky) is resampled up on the host at upload time. That keeps the image, its view,
 * and its descriptor fixed for the renderer's lifetime, the same way the cloud noise volumes
 * and the atmosphere LUTs are sized once at construction.
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
            class SamplerCache;
        }

        namespace Vulkan
        {
            class VulkanDevice;
        }

        namespace Passes
        {
            /**
             * @brief Owns the weather field image and keeps it in step with the simulation.
             *
             * Non-copyable: it owns an image, a view, and its staging buffers.
             */
            class WeatherFieldPass final : public IRenderPass
            {
                public:
                    /**
                     * @brief Creates the field image, its staging ring, and its sampler.
                     * @param device   The live Vulkan device.
                     * @param samplers Cache providing the clamped, linear sampler the field reads under.
                     */
                    WeatherFieldPass(Vulkan::VulkanDevice& device, Resources::SamplerCache& samplers);
                    ~WeatherFieldPass() override;

                    WeatherFieldPass(const WeatherFieldPass&) = delete;
                    WeatherFieldPass& operator=(const WeatherFieldPass&) = delete;

                    void register_pass(Graph::RenderGraph& graph,
                                       const Frame::FrameContext& frame) override;

                    /** @brief The field image: rgba = coverage / density*0.5 / convective / precipitation. */
                    VkImageView view() const noexcept { return view_; }

                    /** @brief The linear, edge-clamped sampler the field is read under. */
                    VkSampler sampler() const noexcept { return sampler_; }

                private:
                    /** @brief One host-visible staging buffer, permanently mapped. */
                    struct Staging
                    {
                        VkBuffer buffer = VK_NULL_HANDLE;
                        VmaAllocation allocation = VK_NULL_HANDLE;
                        void* mapped = nullptr;
                    };

                    // Matches ViewResources::SLOTS. Indexing the staging ring by the frame's own
                    // slot is what makes reuse safe without a fence of our own: a slot is only
                    // recorded into again once the frame that last used it has retired, which is
                    // the invariant the slot count exists to provide.
                    static constexpr std::uint32_t SLOTS = 3;

                    void create_image();
                    void create_staging();
                    void destroy_image();
                    void destroy_staging();

                    Vulkan::VulkanDevice& device_;

                    VkImage image_ = VK_NULL_HANDLE;
                    VmaAllocation allocation_ = VK_NULL_HANDLE;
                    VkImageView view_ = VK_NULL_HANDLE;
                    VkSampler sampler_ = VK_NULL_HANDLE;
                    Staging staging_[SLOTS]{};

                    // 0 means "nothing uploaded yet", which is also the revision a field that has
                    // never been published reports — so the first real publish always uploads.
                    std::uint32_t uploaded_revision_ = 0;
                    bool cleared_ = false;
            };
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
