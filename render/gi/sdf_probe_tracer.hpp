/**************************************************************************/
/* sdf_probe_tracer.hpp                                                    */
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
 * @file sdf_probe_tracer.hpp
 * @brief The SDF cone tracer (Tier A): occlusion and one colour bounce, all hardware.
 *
 * The @ref IProbeTracer that gives probes real spatial variation without ray tracing. It
 * owns a coarse scene distance clipmap, rebuilds it each frame from the frame's analytic
 * primitives, and sphere-traces it per probe to gather occlusion and a single coloured
 * bounce before projecting to SH. An open probe reduces to the distant environment; an
 * occluded one darkens and picks up nearby albedo. Imported meshes fold into the same
 * clipmap as baked bricks in a later slice, so the tracer never grows a second code path.
 */

#include "gi/probe_volume.hpp"
#include "gi/tracer.hpp"

#include <cstdint>
#include <vector>

#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>

namespace SushiEngine
{
    namespace Render
    {
        namespace Geometry
        {
            class MeshRegistry;
        }

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
             * @brief Sphere-traces a scene distance clipmap to relight the probes.
             *
             * Non-copyable: owns a 3D image, two compute pipelines, and ring buffers.
             */
            class SdfProbeTracer final : public IProbeTracer
            {
                public:
                    /**
                     * @brief Builds the clipmap, the populate and trace pipelines, and buffers.
                     * @param device    The live Vulkan device.
                     * @param shaders   Library the compute modules come from.
                     * @param pipelines Factory the compute pipelines are built through.
                     */
                    SdfProbeTracer(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                                   Resources::GraphicsPipelineFactory& pipelines,
                                   Geometry::MeshRegistry& meshes);
                    ~SdfProbeTracer() override;

                    SdfProbeTracer(const SdfProbeTracer&) = delete;
                    SdfProbeTracer& operator=(const SdfProbeTracer&) = delete;

                    void relight(VkCommandBuffer cmd, const ProbeRelightInputs& inputs) override;
                    const char* name() const noexcept override { return "sdf"; }
                    void rebuild_pipelines() override;

                private:
                    static constexpr std::uint32_t RING = 3;

                    /** @brief Push block mirroring sdf_probe_relight.comp's Push. */
                    struct RelightPush
                    {
                        std::int32_t first_probe;
                        std::int32_t relight_count;
                        std::int32_t ray_count;
                        float max_trace_distance;
                    };

                    void create_clipmap();
                    void destroy_clipmap();
                    void create_buffers();
                    void destroy_buffers();
                    void create_pipelines();
                    void destroy_pipelines();

                    /**
                     * @brief Uploads any not-yet-resident mesh bricks and builds the instances.
                     * @param inputs This frame's relight inputs (the draw list and eye).
                     * @param ring   The ring slot to write the instance buffer into.
                     * @return The number of mesh instances written this frame.
                     */
                    std::int32_t build_mesh_instances(const ProbeRelightInputs& inputs,
                                                      std::uint32_t ring);

                    Vulkan::VulkanDevice& device_;
                    Resources::ShaderLibrary& shaders_;
                    Resources::GraphicsPipelineFactory& pipelines_;
                    Geometry::MeshRegistry& meshes_;

                    VkImage clipmap_ = VK_NULL_HANDLE;
                    VmaAllocation clipmap_allocation_ = VK_NULL_HANDLE;
                    VkImageView clipmap_view_ = VK_NULL_HANDLE;

                    // A second clipmap holding each voxel's emitted radiance (emissive
                    // materials), sampled at a trace hit so light-emitting surfaces inject
                    // into the probes. Same lattice as the distance clipmap.
                    VkImage emissive_ = VK_NULL_HANDLE;
                    VmaAllocation emissive_allocation_ = VK_NULL_HANDLE;
                    VkImageView emissive_view_ = VK_NULL_HANDLE;

                    // The shared brick atlas: MAX_SDF_BRICKS slots, host-written once per mesh.
                    VkBuffer brick_atlas_ = VK_NULL_HANDLE;
                    VmaAllocation brick_atlas_allocation_ = VK_NULL_HANDLE;
                    void* brick_atlas_mapped_ = nullptr;
                    std::vector<bool> brick_uploaded_;

                    VkBuffer mesh_buffers_[RING] = {VK_NULL_HANDLE, VK_NULL_HANDLE, VK_NULL_HANDLE};
                    VmaAllocation mesh_allocations_[RING] = {VK_NULL_HANDLE, VK_NULL_HANDLE,
                                                             VK_NULL_HANDLE};
                    void* mesh_mapped_[RING] = {nullptr, nullptr, nullptr};

                    VkBuffer primitive_buffers_[RING] = {VK_NULL_HANDLE, VK_NULL_HANDLE,
                                                         VK_NULL_HANDLE};
                    VmaAllocation primitive_allocations_[RING] = {VK_NULL_HANDLE, VK_NULL_HANDLE,
                                                                  VK_NULL_HANDLE};
                    void* primitive_mapped_[RING] = {nullptr, nullptr, nullptr};

                    VkBuffer clip_config_buffers_[RING] = {VK_NULL_HANDLE, VK_NULL_HANDLE,
                                                           VK_NULL_HANDLE};
                    VmaAllocation clip_config_allocations_[RING] = {VK_NULL_HANDLE, VK_NULL_HANDLE,
                                                                    VK_NULL_HANDLE};
                    void* clip_config_mapped_[RING] = {nullptr, nullptr, nullptr};

                    VkBuffer probe_config_buffers_[RING] = {VK_NULL_HANDLE, VK_NULL_HANDLE,
                                                            VK_NULL_HANDLE};
                    VmaAllocation probe_config_allocations_[RING] = {VK_NULL_HANDLE, VK_NULL_HANDLE,
                                                                     VK_NULL_HANDLE};
                    void* probe_config_mapped_[RING] = {nullptr, nullptr, nullptr};

                    VkDescriptorSetLayout populate_layout_ = VK_NULL_HANDLE;
                    VkPipelineLayout populate_pipeline_layout_ = VK_NULL_HANDLE;
                    VkPipeline populate_pipeline_ = VK_NULL_HANDLE;
                    VkDescriptorSetLayout relight_layout_ = VK_NULL_HANDLE;
                    VkPipelineLayout relight_pipeline_layout_ = VK_NULL_HANDLE;
                    VkPipeline relight_pipeline_ = VK_NULL_HANDLE;

                    // Sparse-relight state: a round-robin window each frame across all
                    // cascades, forced to a full relight when any cascade's lattice shifts a
                    // cell or the sun moves, so no probe is ever read holding a value baked for
                    // a different world position. One centre cell per cascade, since each snaps
                    // to its own spacing and the coarse ones shift far less often.
                    std::uint32_t relight_offset_ = 0;
                    bool has_relit_ = false;
                    std::int32_t last_center_cell_[GI_NUM_CASCADES][3] = {};
                    float last_sun_[3] = {0.0f, 0.0f, 0.0f};
                    float last_sun_intensity_ = -1.0f;
            };
        } // namespace Gi
    } // namespace Render
} // namespace SushiEngine
