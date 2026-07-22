/**************************************************************************/
/* environment_probe_tracer.hpp                                           */
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
 * @file environment_probe_tracer.hpp
 * @brief The degenerate probe tracer: every probe is the distant environment.
 *
 * The lowest tier of the @ref IProbeTracer seam. It performs no local trace at all —
 * it broadcasts the IBL build's nine environment SH coefficients into every probe, so
 * the probe volume reproduces the flat environment ambient exactly. That makes it the
 * safe identity the whole probe pipeline (storage, config, trilinear gather, scheduling)
 * is validated against before an occluding tracer replaces it, and a legitimate floor
 * tier where local tracing is too costly to afford.
 */

#include "gi/tracer.hpp"

#include <vulkan/vulkan.h>

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

        namespace Gi
        {
            /**
             * @brief Fills every probe with the environment SH; no local occlusion.
             *
             * Non-copyable: owns a compute pipeline and descriptor layout.
             */
            class EnvironmentProbeTracer final : public IProbeTracer
            {
                public:
                    /**
                     * @brief Builds the broadcast compute pipeline.
                     * @param device    The live Vulkan device.
                     * @param shaders   Library the compute module comes from.
                     * @param pipelines Factory the compute pipeline is built through.
                     */
                    EnvironmentProbeTracer(Vulkan::VulkanDevice& device,
                                           Resources::ShaderLibrary& shaders,
                                           Resources::GraphicsPipelineFactory& pipelines);
                    ~EnvironmentProbeTracer() override;

                    EnvironmentProbeTracer(const EnvironmentProbeTracer&) = delete;
                    EnvironmentProbeTracer& operator=(const EnvironmentProbeTracer&) = delete;

                    void relight(VkCommandBuffer cmd, const ProbeRelightInputs& inputs) override;
                    const char* name() const noexcept override { return "environment"; }
                    void rebuild_pipelines() override;

                private:
                    /** @brief Push block mirroring gi_probe_relight.comp's Push. */
                    struct Push
                    {
                        std::int32_t probe_count;
                    };

                    void create_pipeline();
                    void destroy_pipeline();

                    Vulkan::VulkanDevice& device_;
                    Resources::ShaderLibrary& shaders_;
                    Resources::GraphicsPipelineFactory& pipelines_;
                    VkDescriptorSetLayout layout_ = VK_NULL_HANDLE;
                    VkPipelineLayout pipeline_layout_ = VK_NULL_HANDLE;
                    VkPipeline pipeline_ = VK_NULL_HANDLE;
            };
        } // namespace Gi
    } // namespace Render
} // namespace SushiEngine
