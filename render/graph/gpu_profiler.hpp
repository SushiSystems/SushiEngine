/**************************************************************************/
/* gpu_profiler.hpp                                                       */
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
 * @file gpu_profiler.hpp
 * @brief Per-pass GPU timing from timestamp queries.
 *
 * The graph brackets every pass it executes with a timestamp write. Because the
 * results are only readable once the submit has completed, the profiler keeps one
 * query pool per frame slot and resolves a slot's timings at the point the caller
 * has already waited on that slot's fence — so reading costs no extra stall. The
 * numbers surface in the editor's profiler HUD, which is what makes every later
 * pass landable against a measured budget rather than a guess.
 */

#include <cstdint>
#include <string>
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

        namespace Graph
        {
            /** @brief One pass's measured GPU duration for the last completed frame. */
            struct PassTiming
            {
                std::string name;          /**< The pass name as registered in the graph. */
                float milliseconds = 0.0f; /**< GPU time between the pass's two timestamps. */
            };

            /**
             * @brief Timestamp-query GPU timing, one query pool per frame slot.
             *
             * Non-copyable: it owns Vulkan query pools.
             */
            class GpuProfiler
            {
                public:
                    /**
                     * @brief Creates one query pool per frame slot.
                     * @param device      The live Vulkan device.
                     * @param frame_slots Number of frames the caller cycles through.
                     * @param max_passes  Upper bound on timed passes in a single frame.
                     */
                    GpuProfiler(Vulkan::VulkanDevice& device, std::uint32_t frame_slots,
                                std::uint32_t max_passes);
                    ~GpuProfiler();

                    GpuProfiler(const GpuProfiler&) = delete;
                    GpuProfiler& operator=(const GpuProfiler&) = delete;

                    /**
                     * @brief Reads back a completed slot's timestamps into timings().
                     *
                     * Only call once the submit that wrote @p slot has signalled its fence;
                     * the results are otherwise undefined.
                     *
                     * @param slot The frame slot whose submit has completed.
                     */
                    void resolve(std::uint32_t slot);

                    /**
                     * @brief Resets a slot's queries and starts recording into it.
                     * @param slot The frame slot being recorded.
                     * @param cmd  The command buffer the frame records into.
                     */
                    void begin_frame(std::uint32_t slot, VkCommandBuffer cmd);

                    /**
                     * @brief Writes the opening timestamp of a pass.
                     * @param cmd  The recording command buffer.
                     * @param name The pass name reported alongside the measured time.
                     * @return A handle to pass to end_pass(), or INVALID_TIMER if full.
                     */
                    std::uint32_t begin_pass(VkCommandBuffer cmd, const char* name);

                    /**
                     * @brief Writes the closing timestamp of a pass.
                     * @param cmd   The recording command buffer.
                     * @param timer The handle returned by begin_pass().
                     */
                    void end_pass(VkCommandBuffer cmd, std::uint32_t timer);

                    /** @brief Timings from the most recently resolved frame. */
                    const std::vector<PassTiming>& timings() const noexcept { return timings_; }

                    /** @brief Whether the device reported usable timestamp queries. */
                    bool enabled() const noexcept { return enabled_; }

                    /** @brief The value begin_pass() returns when the frame's queries are exhausted. */
                    static constexpr std::uint32_t INVALID_TIMER = 0xFFFFFFFFu;

                private:
                    /** @brief One frame slot's query pool and the names recorded into it. */
                    struct Slot
                    {
                        VkQueryPool pool = VK_NULL_HANDLE;
                        std::vector<std::string> names;
                        std::uint32_t recorded = 0;
                    };

                    Vulkan::VulkanDevice& device_;
                    std::vector<Slot> slots_;
                    std::vector<PassTiming> timings_;
                    std::uint32_t max_passes_ = 0;
                    std::uint32_t recording_slot_ = 0;
                    float timestamp_period_ = 1.0f;
                    bool enabled_ = false;
            };
        } // namespace Graph
    } // namespace Render
} // namespace SushiEngine
