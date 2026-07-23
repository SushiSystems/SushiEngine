/**************************************************************************/
/* irradiance_volume_pass.hpp                                             */
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
 * @file irradiance_volume_pass.hpp
 * @brief The probe-volume GI pass: owns the probe SH grid and relights it each frame.
 *
 * Holds one cascade of diffuse irradiance probes (nine SH coefficients each) in a
 * device-local storage buffer, and a small ring of host-visible config blocks the
 * shading pass reads to locate the lattice. Each frame the pass snaps the lattice to the
 * camera and, when GI is on, hands the buffer to its @ref Gi::IProbeTracer to refill —
 * the tracer is the strategy that decides how a probe's incident radiance is gathered.
 * The probe buffer is pass-owned and hand-barriered to the shading pass's fragment reads,
 * exactly as the IBL SH buffer and the atmosphere LUTs are; the render graph only
 * schedules the pass in registration order, which places it after the IBL build (whose
 * SH it reads as the distant-environment fallback) and before the geometry pass.
 */

#include "passes/render_pass.hpp"

#include <cstdint>
#include <memory>

#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>

#include "gi/probe_volume.hpp"
#include "gi/tracer.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Resources
        {
            class GraphicsPipelineFactory;
            class ShaderLibrary;
        }

        namespace Geometry
        {
            class MeshRegistry;
        }

        namespace Vulkan
        {
            class VulkanDevice;
        }


        namespace Passes
        {
            class IblPass;

            /**
             * @brief Owns the probe SH grid and its per-frame relight.
             *
             * Non-copyable: it owns a storage buffer, a ring of uniform buffers, and a
             * tracer that owns a compute pipeline.
             */
            class IrradianceVolumePass final : public IRenderPass
            {
                public:
                    /**
                     * @brief Allocates the probe buffers and builds the default tracer.
                     * @param device    The live Vulkan device.
                     * @param shaders   Library the tracer's compute module comes from.
                     * @param pipelines Factory the tracer's pipeline is built through.
                     * @param ibl       The IBL build whose environment SH seeds the probes.
                     * @param meshes    The mesh store the SDF tracer bakes occluder bricks from.
                     */
                    IrradianceVolumePass(Vulkan::VulkanDevice& device,
                                         Resources::ShaderLibrary& shaders,
                                         Resources::GraphicsPipelineFactory& pipelines,
                                         IblPass& ibl, Geometry::MeshRegistry& meshes);
                    ~IrradianceVolumePass() override;

                    IrradianceVolumePass(const IrradianceVolumePass&) = delete;
                    IrradianceVolumePass& operator=(const IrradianceVolumePass&) = delete;

                    void register_pass(Graph::RenderGraph& graph,
                                       const Frame::FrameContext& frame) override;
                    void rebuild_pipelines() override;

                    /** @brief The probe SH grid the shading pass gathers (binding 29). */
                    VkBuffer probe_sh_buffer() const noexcept { return probe_sh_; }

                    /** @brief Bytes of the probe SH grid. */
                    static VkDeviceSize probe_sh_bytes() noexcept
                    {
                        return Gi::probe_sh_buffer_bytes();
                    }

                    /**
                     * @brief This frame's probe config block (binding 30).
                     * @param frame_index The frame counter, selecting the ring slot.
                     * @return The config buffer the shading pass binds this frame.
                     */
                    VkBuffer config_buffer(std::uint32_t frame_index) const noexcept
                    {
                        return config_buffers_[frame_index % RING];
                    }

                    /** @brief Bytes of the probe config block. */
                    static VkDeviceSize config_bytes() noexcept
                    {
                        return sizeof(Gi::ProbeVolumeConfig);
                    }

                    /**
                     * @brief The scene field the shading pass may trace shadow rays against.
                     *
                     * Forwarded from whichever tracer this pass holds, so the shading pass
                     * depends on the field existing and never on which tier produced it.
                     *
                     * @param frame_index The frame counter, selecting the tracer's ring slot.
                     * @return The field, or an empty record when the tracer keeps none.
                     */
                    Gi::VisibilityField visibility_field(std::uint32_t frame_index) const noexcept
                    {
                        return tracer_ ? tracer_->visibility_field(frame_index)
                                       : Gi::VisibilityField{};
                    }

                    /**
                     * @brief Whether this frame's field holds what this frame's scene looks like.
                     *
                     * The field is repopulated inside this pass, so it is only meaningful on a
                     * frame the pass actually ran; on any other, the contents belong to
                     * whenever GI was last on and must not be traced.
                     */
                    bool field_live() const noexcept { return field_live_; }

                private:
                    static constexpr std::uint32_t RING = 3;

                    /** @brief Whether the last registered frame repopulated the scene field. */
                    bool field_live_ = false;

                    void create_buffers();
                    void destroy_buffers();

                    Vulkan::VulkanDevice& device_;
                    IblPass& ibl_;

                    VkBuffer probe_sh_ = VK_NULL_HANDLE;
                    VmaAllocation probe_sh_allocation_ = VK_NULL_HANDLE;

                    VkBuffer config_buffers_[RING] = {VK_NULL_HANDLE, VK_NULL_HANDLE,
                                                      VK_NULL_HANDLE};
                    VmaAllocation config_allocations_[RING] = {VK_NULL_HANDLE, VK_NULL_HANDLE,
                                                               VK_NULL_HANDLE};
                    void* config_mapped_[RING] = {nullptr, nullptr, nullptr};

                    std::unique_ptr<Gi::IProbeTracer> tracer_;
                    bool probe_buffer_initialized_ = false;
            };
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
