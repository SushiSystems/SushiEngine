/**************************************************************************/
/* view_resources.hpp                                                     */
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
 * @file view_resources.hpp
 * @brief The per-frame GPU resources a scene view owns, split out of the orchestrator.
 *
 * A scene view's job each frame is to build a FrameContext, let passes register into the
 * graph, and submit. Everything it *owns* to make that possible — the double-buffered
 * command slots, the resolve image the editor samples, the picking readback, the three
 * per-frame uniform staging buffers, the transient pools, and the ping-ponged temporal
 * history — is target/readback lifecycle, not orchestration. It lives here so the view is
 * left with the frame loop alone.
 *
 * Two invariants carry over unchanged and are commented at their sites: the temporal
 * history is ping-ponged by frame **parity**, not by slot (the resolve needs the frame
 * immediately before this one, and with two slots in flight the other slot's image is two
 * frames old); and the resolve/history live at the *output* extent so rendering below it
 * is an upscale, not a blur.
 */

#include <cstdint>
#include <memory>
#include <vector>

#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>

#include <SushiEngine/render/scene_view.hpp>

#include "frame/frame_context.hpp"
#include "graph/render_graph.hpp"
#include "resources/transient_pool.hpp"
#include "scene/post_process_uniforms.hpp"
#include "scene/scene_uniforms.hpp"
#include "scene/shadow_uniforms.hpp"
#include "scene/temporal_uniforms.hpp"

#include "vulkan_device.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Vulkan
        {
            /**
             * @brief The double-buffered device resources one scene view renders with.
             *
             * Owns everything per-slot (command recording state, persistent targets, the
             * transient pools) and the two accumulated history frames. The view drives them
             * through this interface; it never touches a `VkImage` or a mapped pointer
             * directly. Non-copyable: it owns Vulkan and VMA handles.
             */
            class ViewResources
            {
                public:
                    /**
                     * @brief How many frame slots exist.
                     *
                     * The ceiling, not the depth in use: how many frames the CPU actually
                     * runs ahead is @c FrameDeliverySettings::frames_in_flight, which the
                     * view changes between frames, so every slot is allocated up front and
                     * the deepest setting costs no reallocation to reach.
                     */
                    static constexpr std::uint32_t SLOTS = 3;

                    /**
                     * @brief Brings up the command slots, targets, and history.
                     * @param device     The live Vulkan device.
                     * @param ui_sampler  The sampler the editor reads the resolve through.
                     * @param width       Initial output width in pixels.
                     * @param height      Initial output height in pixels.
                     */
                    ViewResources(VulkanDevice& device, VkSampler ui_sampler, std::uint32_t width,
                                  std::uint32_t height);
                    ~ViewResources();

                    ViewResources(const ViewResources&) = delete;
                    ViewResources& operator=(const ViewResources&) = delete;

                    /**
                     * @brief Reallocates the targets and history at a new output extent.
                     *
                     * Idles the device first. The command slots and transient pools persist;
                     * only the extent-sized resources are rebuilt, so history is invalidated.
                     */
                    void resize(std::uint32_t width, std::uint32_t height);

                    std::uint32_t width() const noexcept { return width_; }
                    std::uint32_t height() const noexcept { return height_; }

                    // --- Command slot access, driven by the view's frame loop -------------
                    /**
                     * @brief Returns a slot's command buffers to the initial state for a frame.
                     *
                     * Resets both queues' pools and rewinds the hand-out cursors, so the
                     * frame's first acquire_command_buffer() starts from the same buffer every
                     * time and the ones a deeper frame needed are reused rather than regrown.
                     *
                     * @param slot The frame slot about to be recorded.
                     */
                    void reset_commands(std::uint32_t slot);

                    /**
                     * @brief Hands out the next unused command buffer for a queue.
                     *
                     * One per submission rather than one per slot: how many submissions a
                     * frame compiles to depends on how its passes were scheduled, and two of
                     * them recorded into one buffer would submit the same buffer twice. Grows
                     * the slot's pool of buffers on demand and never shrinks it.
                     *
                     * @param slot  The frame slot being recorded.
                     * @param queue Which queue the submission goes to.
                     * @return A command buffer in the initial state, from that queue's family.
                     */
                    VkCommandBuffer acquire_command_buffer(std::uint32_t slot,
                                                           Graph::PassQueue queue);

                    /**
                     * @brief The timeline a queue's submissions signal.
                     * @param queue Which queue's timeline.
                     * @return The semaphore submissions on that queue signal and wait on.
                     */
                    VkSemaphore timeline(Graph::PassQueue queue) const noexcept;

                    /**
                     * @brief Reserves the next value a queue's submission will signal.
                     *
                     * Timeline values rise monotonically per queue, so the value handed back
                     * is also what a later submission waits on to order itself after this
                     * one. Recorded into @p slot so the slot's next occupant knows how far
                     * the GPU must have got before it may reuse the slot's resources.
                     *
                     * @param slot  The frame slot the submission belongs to.
                     * @param queue The queue the submission goes to.
                     * @return The value that submission must signal.
                     */
                    std::uint64_t signal_next(std::uint32_t slot, Graph::PassQueue queue) noexcept;

                    /**
                     * @brief Blocks until every submission recorded for a slot has completed.
                     *
                     * Replaces the per-slot fence: with work on two queues there are two
                     * values to reach, and one wait on both is what makes the slot's
                     * transients, readbacks, and timestamps safe to touch again.
                     *
                     * @param slot The frame slot to wait on.
                     */
                    void wait_for_slot(std::uint32_t slot) const;

                    bool ever_rendered(std::uint32_t slot) const noexcept;
                    void mark_rendered(std::uint32_t slot) noexcept;

                    /** @brief Resets both transient pools for a new frame in this slot. */
                    void begin_slot(std::uint32_t slot);
                    Resources::TexturePool& textures(std::uint32_t slot);
                    Resources::BufferPool& buffers(std::uint32_t slot);

                    // --- Per-frame uniform staging (the three mapped uniform buffers) -----
                    void upload_scene(std::uint32_t slot, const Scene::SceneUniforms& uniforms);
                    void upload_temporal(std::uint32_t slot,
                                         const Scene::TemporalUniforms& uniforms);
                    void upload_shadow(std::uint32_t slot, const Scene::ShadowUniforms& uniforms);
                    void upload_post(std::uint32_t slot,
                                     const Scene::PostProcessUniforms& uniforms);

                    /** @brief Records the internal extent the id target was written at. */
                    void note_render_extent(std::uint32_t slot, std::uint32_t render_width,
                                            std::uint32_t render_height) noexcept;

                    /**
                     * @brief Builds this frame's FrameTargets: transient graph textures plus
                     *        the imports of the owned resolve, history, readback, and uniforms.
                     * @param graph                The frame graph being built.
                     * @param frame                The frame whose extents size the transients.
                     * @param shading_rate_enabled Whether the shading-rate mask runs this frame.
                     * @param rate_texel_width      Shading-rate tile width in pixels.
                     * @param rate_texel_height     Shading-rate tile height in pixels.
                     * @return The handles every pass reads for the frame.
                     */
                    Frame::FrameTargets declare_targets(Graph::RenderGraph& graph,
                                                        const Frame::FrameContext& frame,
                                                        bool shading_rate_enabled,
                                                        std::uint32_t rate_texel_width,
                                                        std::uint32_t rate_texel_height);

                    /** @brief The sampler + resolve view the editor samples for a slot. */
                    SceneViewTexture texture(std::uint32_t slot) const noexcept;

                    bool history_valid() const noexcept { return history_valid_; }
                    void set_history_valid(bool valid) noexcept { history_valid_ = valid; }

                    /**
                     * @brief Reads the picking id under a click, rescaled from the output to
                     *        the internal render extent, after the slot's submit completes.
                     * @param slot         The slot whose id target to read.
                     * @param x            Click x in output pixels.
                     * @param y            Click y in output pixels.
                     * @param output_width  The view's output width the click is relative to.
                     * @param output_height The view's output height the click is relative to.
                     * @return The id under the click, or @c NO_PICK when unavailable.
                     */
                    std::uint32_t pick(std::uint32_t slot, std::uint32_t x, std::uint32_t y,
                                       std::uint32_t output_width,
                                       std::uint32_t output_height) const;

                private:
                    /**
                     * @brief One frame slot: its persistent targets and recording state.
                     *
                     * The transient pools are per slot rather than shared, because a pool
                     * hands a resource back out as soon as its frame ends — sharing one
                     * between slots would let this frame overwrite resources the previous
                     * frame's submit is still reading.
                     */
                    struct Slot
                    {
                        VkImage resolve = VK_NULL_HANDLE;
                        VmaAllocation resolve_allocation = VK_NULL_HANDLE;
                        VkImageView resolve_view = VK_NULL_HANDLE;
                        Graph::TextureState resolve_state{};
                        VkBuffer readback = VK_NULL_HANDLE;
                        VmaAllocation readback_allocation = VK_NULL_HANDLE;
                        void* readback_mapped = nullptr;
                        Graph::BufferState readback_state{};
                        VkBuffer uniforms = VK_NULL_HANDLE;
                        VmaAllocation uniforms_allocation = VK_NULL_HANDLE;
                        void* uniforms_mapped = nullptr;
                        Graph::BufferState uniforms_state{};
                        VkBuffer temporal = VK_NULL_HANDLE;
                        VmaAllocation temporal_allocation = VK_NULL_HANDLE;
                        void* temporal_mapped = nullptr;
                        Graph::BufferState temporal_state{};
                        VkBuffer shadow = VK_NULL_HANDLE;
                        VmaAllocation shadow_allocation = VK_NULL_HANDLE;
                        void* shadow_mapped = nullptr;
                        Graph::BufferState shadow_state{};
                        VkBuffer post = VK_NULL_HANDLE;
                        VmaAllocation post_allocation = VK_NULL_HANDLE;
                        void* post_mapped = nullptr;
                        Graph::BufferState post_state{};
                        VkCommandPool pool = VK_NULL_HANDLE;
                        VkCommandPool compute_pool = VK_NULL_HANDLE;
                        /** @brief Buffers handed out this frame, per queue, and how many. */
                        std::vector<VkCommandBuffer> graphics_buffers;
                        std::vector<VkCommandBuffer> compute_buffers;
                        std::uint32_t graphics_used = 0;
                        std::uint32_t compute_used = 0;
                        /** @brief Timeline values this slot's last submissions signal. */
                        std::uint64_t graphics_target = 0;
                        std::uint64_t compute_target = 0;
                        bool ever_rendered = false;
                        /** @brief Render extent the id target in this slot was written at. */
                        std::uint32_t readback_width = 0;
                        std::uint32_t readback_height = 0;
                        std::unique_ptr<Resources::TexturePool> textures;
                        std::unique_ptr<Resources::BufferPool> buffers;
                    };

                    /**
                     * @brief One accumulated frame the temporal resolve reads and writes.
                     *
                     * Two of them, ping-ponged by frame parity rather than by frame slot:
                     * the resolve needs the frame immediately before this one, and with
                     * two slots in flight the other slot's image is two frames old.
                     */
                    struct History
                    {
                        VkImage image = VK_NULL_HANDLE;
                        VmaAllocation allocation = VK_NULL_HANDLE;
                        VkImageView view = VK_NULL_HANDLE;
                        Graph::TextureState state{};
                    };

                    void create_command();
                    void create_targets();
                    void destroy_targets();

                    VulkanDevice& device_;
                    VkSampler ui_sampler_ = VK_NULL_HANDLE;
                    /**
                     * @brief One monotonic timeline per queue, not one per frame slot.
                     *
                     * A single shared counter would have to be signalled in increasing order,
                     * which is exactly what two overlapping queues cannot promise; a timeline
                     * per queue lets each rise with its own submissions, and a wait on one
                     * queue's value still covers everything earlier on that queue.
                     */
                    VkSemaphore graphics_timeline_ = VK_NULL_HANDLE;
                    VkSemaphore compute_timeline_ = VK_NULL_HANDLE;
                    std::uint64_t graphics_value_ = 0;
                    std::uint64_t compute_value_ = 0;
                    Slot slots_[SLOTS];
                    History history_[2];
                    bool history_valid_ = false;
                    std::uint32_t width_ = 16;
                    std::uint32_t height_ = 16;
            };
        } // namespace Vulkan
    } // namespace Render
} // namespace SushiEngine
