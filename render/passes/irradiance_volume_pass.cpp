/**************************************************************************/
/* irradiance_volume_pass.cpp                                             */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/* Licensed under the Apache License, Version 2.0 (the "License");        */
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

#include "passes/irradiance_volume_pass.hpp"

#include <cstring>

#include <SushiEngine/render/environment.hpp>

#include "frame/frame_context.hpp"
#include "gi/sdf_probe_tracer.hpp"
#include "graph/render_graph.hpp"
#include "passes/ibl_pass.hpp"
#include "rhi/vulkan/vulkan_check.hpp"
#include "rhi/vulkan/vulkan_device.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Passes
        {
            namespace
            {
                void buffer_barrier(VkCommandBuffer cmd, VkBuffer buffer,
                                    VkPipelineStageFlags2 source, VkAccessFlags2 source_access,
                                    VkPipelineStageFlags2 destination,
                                    VkAccessFlags2 destination_access)
                {
                    VkBufferMemoryBarrier2 barrier{};
                    barrier.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER_2;
                    barrier.srcStageMask = source;
                    barrier.srcAccessMask = source_access;
                    barrier.dstStageMask = destination;
                    barrier.dstAccessMask = destination_access;
                    barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                    barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                    barrier.buffer = buffer;
                    barrier.offset = 0;
                    barrier.size = VK_WHOLE_SIZE;

                    VkDependencyInfo dependency{};
                    dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
                    dependency.bufferMemoryBarrierCount = 1;
                    dependency.pBufferMemoryBarriers = &barrier;
                    vkCmdPipelineBarrier2(cmd, &dependency);
                }
            } // namespace

            IrradianceVolumePass::IrradianceVolumePass(
                Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                Resources::GraphicsPipelineFactory& pipelines, IblPass& ibl)
                : device_(device), ibl_(ibl),
                  tracer_(std::make_unique<Gi::SdfProbeTracer>(device, shaders, pipelines))
            {
                create_buffers();
            }

            IrradianceVolumePass::~IrradianceVolumePass()
            {
                destroy_buffers();
            }

            void IrradianceVolumePass::create_buffers()
            {
                // The probe SH grid: one device-local storage buffer, nine SH vec4 per probe.
                // One buffer, not one per slot — a relight is change-gated on the environment
                // and never overwrites while a prior submit still reads, exactly like the IBL
                // SH buffer this seeds from.
                VkBufferCreateInfo sh_info{};
                sh_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
                sh_info.size = probe_sh_bytes();
                sh_info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
                VmaAllocationCreateInfo sh_alloc{};
                sh_alloc.usage = VMA_MEMORY_USAGE_AUTO;
                Vulkan::check(vmaCreateBuffer(device_.allocator(), &sh_info, &sh_alloc, &probe_sh_,
                                              &probe_sh_allocation_, nullptr),
                              "vmaCreateBuffer(gi probe sh)");

                // A small ring of host-visible config blocks: the lattice snaps to the camera
                // every frame, so the block is rewritten every frame and must not clobber a
                // slot a prior submit is still reading.
                for (std::uint32_t i = 0; i < RING; ++i)
                {
                    VkBufferCreateInfo info{};
                    info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
                    info.size = sizeof(Gi::ProbeVolumeConfig);
                    info.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
                    VmaAllocationCreateInfo alloc{};
                    alloc.usage = VMA_MEMORY_USAGE_AUTO;
                    alloc.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
                                  VMA_ALLOCATION_CREATE_MAPPED_BIT;
                    VmaAllocationInfo mapped{};
                    Vulkan::check(vmaCreateBuffer(device_.allocator(), &info, &alloc,
                                                  &config_buffers_[i], &config_allocations_[i],
                                                  &mapped),
                                  "vmaCreateBuffer(gi probe config)");
                    config_mapped_[i] = mapped.pMappedData;
                }
            }

            void IrradianceVolumePass::destroy_buffers()
            {
                if (probe_sh_ != VK_NULL_HANDLE)
                    vmaDestroyBuffer(device_.allocator(), probe_sh_, probe_sh_allocation_);
                probe_sh_ = VK_NULL_HANDLE;
                for (std::uint32_t i = 0; i < RING; ++i)
                    if (config_buffers_[i] != VK_NULL_HANDLE)
                        vmaDestroyBuffer(device_.allocator(), config_buffers_[i],
                                         config_allocations_[i]);
            }

            void IrradianceVolumePass::rebuild_pipelines()
            {
                tracer_->rebuild_pipelines();
            }

            void IrradianceVolumePass::register_pass(Graph::RenderGraph& graph,
                                                     const Frame::FrameContext& frame)
            {
                if (frame.environment == nullptr)
                    return;
                const Environment& environment = *frame.environment;

                const bool enabled = environment.gi.enabled && frame.quality.probe_gi;

                // The config block is written every frame — enabled or not — so the shading
                // pass always has a valid binding to read and fall back on.
                const std::uint32_t ring = frame.index % RING;
                Gi::ProbeVolumeConfig config{};
                Gi::configure_probe_volume(frame.eye, enabled, environment.gi.intensity,
                                           environment.gi.normal_bias, config);
                std::memcpy(config_mapped_[ring], &config, sizeof(config));

                if (!enabled)
                    return;

                graph.add_pass(
                    "gi probe relight",
                    [](Graph::RenderPassBuilder& builder)
                    {
                        // The probe buffer and the IBL SH it reads are pass-owned and
                        // barriered by hand below; a side effect keeps the pass alive.
                        builder.set_side_effect();
                    },
                    [this, &frame, config](VkCommandBuffer cmd, const Graph::PassContext&)
                    {
                        // Order the relight's compute read after the IBL build's SH write.
                        // The graph runs passes in registration order, so the IBL pass has
                        // already recorded; on frames it did not rebuild this is a harmless
                        // execution barrier over a buffer written in an earlier frame.
                        buffer_barrier(cmd, ibl_.sh_buffer(),
                                       VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                       VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT,
                                       VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                       VK_ACCESS_2_SHADER_STORAGE_READ_BIT);

                        Gi::ProbeRelightInputs inputs{};
                        inputs.probe_sh = probe_sh_;
                        inputs.probe_sh_bytes = probe_sh_bytes();
                        inputs.probe_count = static_cast<std::uint32_t>(Gi::PROBE_COUNT_TOTAL);
                        inputs.environment_sh = ibl_.sh_buffer();
                        inputs.environment_sh_bytes = IblPass::sh_buffer_bytes();
                        inputs.config = &config;
                        inputs.frame = &frame;
                        tracer_->relight(cmd, inputs);

                        // Make the refilled probes visible to the shading pass's fragment
                        // reads — the buffer is pass-owned, so this is recorded, not derived.
                        buffer_barrier(cmd, probe_sh_,
                                       VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                       VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT,
                                       VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                                       VK_ACCESS_2_SHADER_STORAGE_READ_BIT);
                    });
            }
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
