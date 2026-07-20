/**************************************************************************/
/* gpu_profiler.cpp                                                       */
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

#include "graph/gpu_profiler.hpp"

#include "rhi/vulkan/vulkan_check.hpp"
#include "rhi/vulkan/vulkan_device.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Graph
        {
            GpuProfiler::GpuProfiler(Vulkan::VulkanDevice& device, std::uint32_t frame_slots,
                                     std::uint32_t max_passes)
                : device_(device), max_passes_(max_passes)
            {
                VkPhysicalDeviceProperties properties{};
                vkGetPhysicalDeviceProperties(device_.physical_device(), &properties);
                timestamp_period_ = properties.limits.timestampPeriod;
                // A period of zero means the device does not support timestamp queries at
                // all; the profiler then records nothing and reports no timings rather
                // than emitting commands the driver would reject.
                enabled_ = timestamp_period_ > 0.0f;
                if (!enabled_)
                    return;

                slots_.resize(frame_slots);
                for (Slot& slot : slots_)
                {
                    VkQueryPoolCreateInfo info{};
                    info.sType = VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO;
                    info.queryType = VK_QUERY_TYPE_TIMESTAMP;
                    info.queryCount = max_passes_ * 2;
                    Vulkan::check(vkCreateQueryPool(device_.device(), &info, nullptr, &slot.pool),
                                  "vkCreateQueryPool(profiler)");
                    slot.names.reserve(max_passes_);
                }
            }

            GpuProfiler::~GpuProfiler()
            {
                for (Slot& slot : slots_)
                    if (slot.pool != VK_NULL_HANDLE)
                        vkDestroyQueryPool(device_.device(), slot.pool, nullptr);
            }

            void GpuProfiler::resolve(std::uint32_t slot_index)
            {
                if (!enabled_ || slot_index >= slots_.size())
                    return;
                Slot& slot = slots_[slot_index];
                if (slot.recorded == 0)
                    return;

                std::vector<std::uint64_t> stamps(slot.recorded * 2);
                const VkResult result = vkGetQueryPoolResults(
                    device_.device(), slot.pool, 0, slot.recorded * 2,
                    stamps.size() * sizeof(std::uint64_t), stamps.data(), sizeof(std::uint64_t),
                    VK_QUERY_RESULT_64_BIT);
                if (result != VK_SUCCESS)
                    return;

                timings_.clear();
                timings_.reserve(slot.recorded);
                for (std::uint32_t i = 0; i < slot.recorded; ++i)
                {
                    PassTiming timing;
                    timing.name = slot.names[i];
                    const std::uint64_t begin = stamps[i * 2];
                    const std::uint64_t end = stamps[i * 2 + 1];
                    // Timestamps are in device ticks; timestampPeriod converts a tick to
                    // nanoseconds, and a wrapped counter (end < begin) reports as zero
                    // rather than as an enormous duration.
                    const double ticks = end > begin ? static_cast<double>(end - begin) : 0.0;
                    timing.milliseconds =
                        static_cast<float>(ticks * static_cast<double>(timestamp_period_) * 1e-6);
                    timings_.push_back(std::move(timing));
                }
            }

            void GpuProfiler::begin_frame(std::uint32_t slot_index, VkCommandBuffer cmd)
            {
                if (!enabled_ || slot_index >= slots_.size())
                    return;
                recording_slot_ = slot_index;
                Slot& slot = slots_[slot_index];
                vkCmdResetQueryPool(cmd, slot.pool, 0, max_passes_ * 2);
                slot.names.clear();
                slot.recorded = 0;
            }

            std::uint32_t GpuProfiler::begin_pass(VkCommandBuffer cmd, const char* name)
            {
                if (!enabled_ || recording_slot_ >= slots_.size())
                    return INVALID_TIMER;
                Slot& slot = slots_[recording_slot_];
                if (slot.recorded >= max_passes_)
                    return INVALID_TIMER;

                const std::uint32_t timer = slot.recorded++;
                slot.names.emplace_back(name != nullptr ? name : "pass");
                vkCmdWriteTimestamp2(cmd, VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, slot.pool, timer * 2);
                return timer;
            }

            void GpuProfiler::end_pass(VkCommandBuffer cmd, std::uint32_t timer)
            {
                if (!enabled_ || timer == INVALID_TIMER || recording_slot_ >= slots_.size())
                    return;
                vkCmdWriteTimestamp2(cmd, VK_PIPELINE_STAGE_2_BOTTOM_OF_PIPE_BIT,
                                     slots_[recording_slot_].pool, timer * 2 + 1);
            }
        } // namespace Graph
    } // namespace Render
} // namespace SushiEngine
