/**************************************************************************/
/* pass_capture.hpp                                                       */
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
 * @file pass_capture.hpp
 * @brief Per-pass output hashing: what changed, not merely that something did.
 *
 * A whole-frame golden answers "did the image change". This answers "which pass
 * changed it", which is the question a 39-pass port actually has to keep asking.
 * The graph copies every texture a pass wrote into a staging buffer straight after
 * that pass records, and the host hashes the bytes once the submit has completed.
 *
 * This is a debug instrument and is off unless something turns it on. When it is on
 * it is not free and not invisible: every transient gains @c TRANSFER_SRC usage,
 * which is part of the pool's reuse key, so a captured frame aliases its transients
 * differently from an uncaptured one. Contents are unaffected — a pass writes what
 * it writes — with one honest exception, which is a pass reading a transient it
 * never initialised. That is a bug, and capture makes it show up as a hash that
 * differs between captured and uncaptured runs rather than hiding it.
 *
 * The instrument follows @ref GpuProfiler exactly: one store per frame slot,
 * begin_frame() on the slot being recorded, resolve() on a slot whose submit has
 * completed. They are the same lifecycle because they are the same kind of thing.
 */

#include <cstdint>
#include <string>
#include <vector>

#include <vulkan/vulkan.h>

#include <vk_mem_alloc.h>

#include "graph/resource_handle.hpp"

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
            /**
             * @brief One texture one pass wrote, and the hash of what it held afterwards.
             *
             * @c pass and @c resource together are the identity a golden is keyed on. A
             * pass that writes two targets contributes two of these, and one texture
             * written by two passes contributes one per pass — the point being to say
             * which write changed, not merely that the texture ended up different.
             */
            struct CapturedPass
            {
                std::string pass;     /**< The pass name the graph registered. */
                std::string resource; /**< TextureDesc::name of the written texture. */
                std::uint32_t width = 0;
                std::uint32_t height = 0;
                std::uint32_t depth = 0;
                std::uint32_t layers = 0;
                VkFormat format = VK_FORMAT_UNDEFINED;
                std::uint64_t hash = 0; /**< FNV-1a 64 over the copied bytes. */
            };

            /**
             * @brief Copies pass outputs into host memory and hashes them.
             *
             * Owned by whoever owns the graph, handed to the graph by pointer, and
             * driven by the frame loop that knows about slots. Non-copyable.
             */
            class PassCapture
            {
                public:
                    /**
                     * @brief Staging bytes one frame slot may spend before it starts dropping.
                     *
                     * Sized against the frame's largest single output rather than its total:
                     * a shadow atlas is four cascades of a square depth map and dwarfs every
                     * screen-sized target beneath it. 96 MiB — the first value tried — was
                     * spent by that one texture and truncated the frame at sixteen outputs,
                     * which is exactly why the drop counters below are reported rather than
                     * merely kept.
                     */
                    static constexpr VkDeviceSize DEFAULT_BUDGET = VkDeviceSize(256) << 20;

                    /**
                     * @brief Prepares one staging buffer per frame slot, allocated on use.
                     *
                     * A slot's buffer is allocated the first time it is recorded into, not
                     * here: a caller cycling two frames should not pay for a third, and the
                     * budget is large enough that guessing wrong is expensive. Once
                     * allocated a buffer is never grown — growing one mid-frame would mean
                     * moving storage a recorded-but-unsubmitted copy already points at.
                     *
                     * @param device      The live Vulkan device.
                     * @param frame_slots Number of frames the caller cycles through.
                     * @param budget      Staging bytes per slot; outputs past it are dropped
                     *                    and counted, never silently truncated.
                     */
                    PassCapture(Vulkan::VulkanDevice& device, std::uint32_t frame_slots,
                                VkDeviceSize budget = DEFAULT_BUDGET);
                    ~PassCapture();

                    PassCapture(const PassCapture&) = delete;
                    PassCapture& operator=(const PassCapture&) = delete;

                    /**
                     * @brief Discards a slot's previous contents and records into it.
                     *
                     * Must name the slot the frame is being recorded into, for the same
                     * reason the transient pools do: a copy recorded into a slot still in
                     * flight would overwrite bytes the host has not read yet.
                     *
                     * @param slot The frame slot being recorded.
                     */
                    void begin_frame(std::uint32_t slot);

                    /**
                     * @brief Whether this texture would be captured, were it offered now.
                     *
                     * Asked by the graph *before* it transitions the image, so a texture
                     * this turns down costs no barrier at all. Turning one down is counted:
                     * see dropped_by_budget() and dropped_by_format().
                     *
                     * @param desc The written texture's description.
                     * @return true if a following record() would copy it.
                     */
                    bool wants(const TextureDesc& desc);

                    /**
                     * @brief Records the copy of one pass output into the active slot.
                     *
                     * The caller must have transitioned @p image into
                     * @c VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL and must have had wants()
                     * return true for @p desc since the last record().
                     *
                     * Mip 0 only, every array layer, every depth slice, and for a
                     * depth/stencil format the depth aspect alone. A regression confined to
                     * mip 3 or to the stencil aspect is therefore invisible here — stated
                     * plainly because a hash that silently covers less than it appears to is
                     * worse than no hash.
                     *
                     * @param cmd   The recording command buffer.
                     * @param pass  The pass name, copied.
                     * @param desc  The written texture's description.
                     * @param image The physical image to copy from.
                     */
                    void record(VkCommandBuffer cmd, const char* pass, const TextureDesc& desc,
                                VkImage image);

                    /**
                     * @brief Hashes a completed slot's captured bytes.
                     *
                     * Only call once the submit that wrote @p slot is known to have
                     * completed; the results are otherwise undefined, exactly as for
                     * @ref GpuProfiler::resolve.
                     *
                     * @param slot The frame slot whose submit has completed.
                     * @param out  Receives one entry per captured output, in record order.
                     * @return Whether @p slot named a real slot that had been recorded into.
                     */
                    bool resolve(std::uint32_t slot, std::vector<CapturedPass>& out) const;

                    /**
                     * @brief Outputs a completed slot turned down for want of staging bytes.
                     *
                     * A non-zero answer means the frame was captured *incompletely*, and a
                     * caller recording a reference from it would be recording a reference
                     * that cannot notice the passes it never saw.
                     *
                     * @param slot The frame slot to report on.
                     */
                    std::uint32_t dropped_by_budget(std::uint32_t slot) const noexcept;

                    /**
                     * @brief Outputs a completed slot turned down as un-copyable.
                     * @param slot The frame slot to report on.
                     */
                    std::uint32_t dropped_by_format(std::uint32_t slot) const noexcept;

                    /**
                     * @brief Staging bytes a completed slot actually spent.
                     * @param slot The frame slot to report on.
                     */
                    VkDeviceSize bytes_used(std::uint32_t slot) const noexcept;

                    /** @brief Staging bytes one slot may spend. */
                    VkDeviceSize budget() const noexcept { return budget_; }

                private:
                    /** @brief One recorded copy: where it landed and what it was. */
                    struct Entry
                    {
                        std::string pass;
                        std::string resource;
                        TextureDesc desc{};
                        VkDeviceSize offset = 0;
                        VkDeviceSize size = 0;
                    };

                    /**
                     * @brief One frame slot's staging buffer and what it holds.
                     *
                     * The drop counters live here rather than on the capture because they
                     * describe a *frame*, and the frame a caller resolves is not always the
                     * one being recorded.
                     */
                    struct Store
                    {
                        VkBuffer buffer = VK_NULL_HANDLE;
                        VmaAllocation allocation = VK_NULL_HANDLE;
                        void* mapped = nullptr;
                        VkDeviceSize cursor = 0;
                        bool recorded = false;
                        std::uint32_t dropped_budget = 0;
                        std::uint32_t dropped_format = 0;
                        std::vector<Entry> entries;
                    };

                    /** @brief Allocates @p store's buffer if it has none; false if it cannot. */
                    bool ensure_allocated(Store& store);

                    Vulkan::VulkanDevice& device_;
                    VkDeviceSize budget_ = 0;
                    std::vector<Store> stores_;
                    std::uint32_t active_ = 0xFFFFFFFFu;
                    /** @brief Size the last wants() computed, consumed by the next record(). */
                    VkDeviceSize pending_size_ = 0;
            };
        } // namespace Graph
    } // namespace Render
} // namespace SushiEngine
