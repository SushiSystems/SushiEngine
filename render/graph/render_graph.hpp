/**************************************************************************/
/* render_graph.hpp                                                       */
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
 * @file render_graph.hpp
 * @brief The frame graph: passes declare resources, the graph derives the rest.
 *
 * A pass registers a setup function that names the textures and buffers it touches
 * and how, plus an execute function that records draws. From those declarations the
 * graph derives, per frame: which passes actually contribute (culling), which
 * transient resources may share one allocation (lifetime aliasing), every image and
 * buffer barrier, and the dynamic-rendering scope around an attachment-using pass.
 * No pass writes a VkImageMemoryBarrier2 or a VkRenderingInfo, which is what lets a
 * new effect be added without editing its neighbours.
 */

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include <vulkan/vulkan.h>

#include "graph/resource_handle.hpp"
#include "graph/resource_state.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Vulkan
        {
            class VulkanDevice;
        }

        namespace Resources
        {
            class TexturePool;
            class BufferPool;
        }

        namespace Graph
        {
            class GpuProfiler;
            class RenderGraph;

            /** @brief The largest number of colour attachments one pass may declare. */
            constexpr std::uint32_t MAX_COLOR_ATTACHMENTS = 8;

            /** @brief The value a submission's @c wait takes when it waits on nothing. */
            constexpr std::uint32_t NO_SUBMISSION = 0xFFFFFFFFu;

            /**
             * @brief One contiguous run of passes recorded into one command buffer.
             *
             * The compiler splits the schedule wherever the queue changes, so a frame with
             * no async pass is one submission and the caller's loop degenerates to what it
             * always did. @c wait is the index of the latest earlier submission on the
             * *other* queue that this one depends on, or @c NO_SUBMISSION: a dependency on
             * the same queue is already ordered by submission order and needs no wait, and
             * because a queue's timeline values rise with its submissions, waiting on the
             * latest cross-queue dependency covers every earlier one.
             */
            struct Submission
            {
                PassQueue queue = PassQueue::Graphics;
                std::uint32_t first = 0; /**< First position in the compiled order. */
                std::uint32_t count = 0; /**< Passes in this run. */
                std::uint32_t wait = NO_SUBMISSION;
            };

            /**
             * @brief An externally owned texture the graph may schedule against.
             *
             * @c state points at storage the caller keeps alive across frames: the graph
             * reads the layout the image is currently in and writes back the layout it
             * left it in, so an image the editor samples between frames is tracked
             * correctly without the graph owning it.
             */
            struct ImportedTexture
            {
                VkImage image = VK_NULL_HANDLE;
                VkImageView view = VK_NULL_HANDLE;
                VkImageView sample_view = VK_NULL_HANDLE; /**< Falls back to @c view when null. */
                TextureDesc desc{};
                TextureState* state = nullptr;
            };

            /** @brief An externally owned buffer the graph may schedule against. */
            struct ImportedBuffer
            {
                VkBuffer buffer = VK_NULL_HANDLE;
                void* mapped = nullptr;
                BufferDesc desc{};
                BufferState* state = nullptr;
            };

            /**
             * @brief What a pass's execute function is handed to reach its resources.
             *
             * Resolves handles to the physical objects the graph picked this frame, so a
             * pass never caches a VkImageView across frames and never learns whether its
             * target was aliased onto another pass's.
             */
            class PassContext
            {
                public:
                    /**
                     * @brief Binds the context to the graph and the pass being executed.
                     * @param graph       The graph resolving the handles.
                     * @param render_area The area the graph opened rendering over.
                     */
                    PassContext(const RenderGraph& graph, VkExtent2D render_area) noexcept
                        : graph_(graph), render_area_(render_area) {}

                    /**
                     * @brief The image behind a texture handle this frame.
                     * @param handle The declared texture.
                     * @return The physical VkImage.
                     */
                    VkImage image(TextureHandle handle) const;

                    /**
                     * @brief The full-subresource view of a texture handle this frame.
                     * @param handle The declared texture.
                     * @return The physical VkImageView.
                     */
                    VkImageView view(TextureHandle handle) const;

                    /**
                     * @brief The view of a texture that is legal to sample.
                     *
                     * Identical to view() except for depth/stencil textures, where the
                     * attachment view spans both aspects and only this depth-aspect view
                     * may be bound to a sampler.
                     *
                     * @param handle The declared texture.
                     * @return The physical VkImageView to sample through.
                     */
                    VkImageView sampled_view(TextureHandle handle) const;

                    /**
                     * @brief The description a texture was created or imported with.
                     * @param handle The declared texture.
                     * @return The description, including its extent and format.
                     */
                    const TextureDesc& texture_desc(TextureHandle handle) const;

                    /**
                     * @brief The buffer behind a buffer handle this frame.
                     * @param handle The declared buffer.
                     * @return The physical VkBuffer.
                     */
                    VkBuffer buffer(BufferHandle handle) const;

                    /**
                     * @brief The host mapping of a host-visible buffer, or nullptr.
                     * @param handle The declared buffer.
                     * @return The mapped pointer, or nullptr if the buffer is device-local.
                     */
                    void* mapped(BufferHandle handle) const;

                    /** @brief The extent the graph opened dynamic rendering over. */
                    VkExtent2D render_area() const noexcept { return render_area_; }

                private:
                    const RenderGraph& graph_;
                    VkExtent2D render_area_{};
            };

            /**
             * @brief The setup-time interface a pass declares its resource use through.
             *
             * Every call here feeds barrier derivation, culling, and lifetime analysis.
             * A pass that omits a declaration will be missing the barrier for it, so
             * declaration is the contract, not an optimisation hint.
             */
            class RenderPassBuilder
            {
                public:
                    /**
                     * @brief Binds the builder to the pass being set up.
                     * @param graph The graph the pass belongs to.
                     * @param pass  Index of the pass node being built.
                     */
                    RenderPassBuilder(RenderGraph& graph, std::uint32_t pass) noexcept
                        : graph_(graph), pass_(pass) {}

                    /**
                     * @brief Declares that the pass reads a texture.
                     * @param handle The texture read.
                     * @param access How it is read.
                     */
                    void read(TextureHandle handle, TextureAccess access);

                    /**
                     * @brief Declares that the pass writes a texture.
                     * @param handle The texture written.
                     * @param access How it is written.
                     */
                    void write(TextureHandle handle, TextureAccess access);

                    /**
                     * @brief Declares that the pass reads a buffer.
                     * @param handle The buffer read.
                     * @param access How it is read.
                     */
                    void read(BufferHandle handle, BufferAccess access);

                    /**
                     * @brief Declares that the pass writes a buffer.
                     * @param handle The buffer written.
                     * @param access How it is written.
                     */
                    void write(BufferHandle handle, BufferAccess access);

                    /**
                     * @brief Binds a texture as a colour attachment for this pass.
                     *
                     * The graph opens dynamic rendering over every attachment a pass
                     * declares and closes it afterwards, so the execute function records
                     * draws only.
                     *
                     * @param index  Attachment slot, matching the fragment shader's location.
                     * @param handle The texture written.
                     * @param load   What to do with the existing contents.
                     * @param clear  Clear value, used when @p load is Clear.
                     */
                    void color_attachment(std::uint32_t index, TextureHandle handle,
                                          AttachmentLoad load, const ClearColor& clear = {});

                    /**
                     * @brief Binds a texture as this pass's depth/stencil attachment.
                     * @param handle    The depth (or depth/stencil) texture.
                     * @param load      What to do with the existing contents.
                     * @param depth     Clear depth, used when @p load is Clear.
                     * @param stencil   Clear stencil, used when @p load is Clear.
                     * @param read_only Bind for testing only, never writing.
                     */
                    void depth_stencil_attachment(TextureHandle handle, AttachmentLoad load,
                                                  float depth = 0.0f, std::uint32_t stencil = 0,
                                                  bool read_only = false);

                    /**
                     * @brief Binds a per-tile image controlling this pass's shading rate.
                     *
                     * One texel covers one @p texel_width × @p texel_height block of the
                     * render area and names the rate its fragments shade at, so the pass
                     * pays full rate only where the mask asks for it. Ignored — with no
                     * effect on the pass's correctness — on a device without
                     * attachment-based fragment shading rate, which is why a pass may
                     * declare it unconditionally.
                     *
                     * @param handle       The rate image, one byte per tile.
                     * @param texel_width  Pixels one texel covers horizontally.
                     * @param texel_height Pixels one texel covers vertically.
                     */
                    void shading_rate_attachment(TextureHandle handle, std::uint32_t texel_width,
                                                 std::uint32_t texel_height);

                    /**
                     * @brief Overrides the rendering area, which otherwise follows attachment 0.
                     * @param width  Render-area width in pixels.
                     * @param height Render-area height in pixels.
                     */
                    void set_render_area(std::uint32_t width, std::uint32_t height);

                    /**
                     * @brief Asks for the pass to be recorded on the async compute queue.
                     *
                     * Honoured only when the frame enabled async compute and the device has a
                     * compute family of its own; otherwise the declaration is ignored and the
                     * pass records on the graphics queue as usual, so declaring it never
                     * changes what the pass must be able to do.
                     *
                     * Two conditions the flagging pass owes the graph. Everything it produces
                     * must be *declared* — the cross-queue wait is derived from declarations,
                     * so a pass that hand-barriers a resource it owns (the LUT and fog
                     * volumes do) would have its consumers unsynchronised. And the resources
                     * it shares must be graph transients: the graph marks those concurrent
                     * for itself (@c TextureDesc::cross_queue), but it cannot change the
                     * sharing mode of an *imported* allocation someone else created, so an
                     * import that crosses queues has to have been created shared by its owner.
                     *
                     * @param queue The queue the pass would prefer to run on.
                     */
                    void set_queue(PassQueue queue);

                    /**
                     * @brief Marks the pass as never cullable.
                     *
                     * Needed by passes whose result leaves the graph in a way the graph
                     * cannot see — a readback copy the host reads later, for instance.
                     */
                    void set_side_effect();

                private:
                    RenderGraph& graph_;
                    std::uint32_t pass_ = 0;
            };

            /**
             * @brief Builds, compiles, and executes one frame's pass graph.
             *
             * The lifecycle is begin_frame() → create/import resources → add_pass()… →
             * compile() → execute(). Handles and pass registrations do not survive
             * begin_frame(), so the graph is rebuilt each frame; only the physical
             * resources behind it (owned by the pools) persist. Non-copyable.
             */
            class RenderGraph
            {
                public:
                    /** @brief What an execute function is: record commands for one pass. */
                    using ExecuteFunction = std::function<void(VkCommandBuffer, const PassContext&)>;

                    /**
                     * @brief Binds the graph to the device and profiler it uses.
                     * @param device   The live Vulkan device.
                     * @param profiler Per-pass timing, or nullptr to record none.
                     */
                    RenderGraph(Vulkan::VulkanDevice& device, GpuProfiler* profiler);

                    RenderGraph(const RenderGraph&) = delete;
                    RenderGraph& operator=(const RenderGraph&) = delete;

                    /**
                     * @brief Discards the previous frame's passes and binds this frame's pools.
                     *
                     * The pools must belong to the frame slot being recorded: a transient
                     * handed out here may be reused as soon as the pool's next begin_frame(),
                     * so sharing one pool between slots would let a frame overwrite the
                     * resources of one still in flight.
                     *
                     * @param textures Pool backing this slot's transient textures.
                     * @param buffers  Pool backing this slot's transient buffers.
                     */
                    void begin_frame(Resources::TexturePool& textures,
                                     Resources::BufferPool& buffers);

                    /**
                     * @brief Whether this frame may split passes onto the compute queue.
                     *
                     * Set once per frame before the passes register. False collapses every
                     * @c PassQueue::AsyncCompute declaration back to the graphics queue, so a
                     * frame always compiles to something submittable regardless of device or
                     * settings.
                     *
                     * @param enabled Whether the second queue is available and wanted.
                     */
                    void set_async_compute_enabled(bool enabled) noexcept;

                    /**
                     * @brief Declares a transient texture the graph allocates and may alias.
                     * @param desc What the texture must be; usage is unioned with the
                     *             declared accesses before allocation.
                     * @return A handle valid until the next begin_frame().
                     */
                    TextureHandle create_texture(const TextureDesc& desc);

                    /**
                     * @brief Declares a transient buffer the graph allocates and may alias.
                     * @param desc What the buffer must be; usage is unioned with the
                     *             declared accesses before allocation.
                     * @return A handle valid until the next begin_frame().
                     */
                    BufferHandle create_buffer(const BufferDesc& desc);

                    /**
                     * @brief Brings an externally owned texture under the graph's scheduling.
                     * @param imported The image, view, description, and tracked state.
                     * @return A handle valid until the next begin_frame().
                     */
                    TextureHandle import_texture(const ImportedTexture& imported);

                    /**
                     * @brief Brings an externally owned buffer under the graph's scheduling.
                     * @param imported The buffer, mapping, description, and tracked state.
                     * @return A handle valid until the next begin_frame().
                     */
                    BufferHandle import_buffer(const ImportedBuffer& imported);

                    /**
                     * @brief Registers a pass, running its setup immediately.
                     * @param name    Pass name, used by the profiler and debug labels.
                     * @param setup   Declares the pass's resource use through the builder.
                     * @param execute Records the pass's commands at execute() time.
                     */
                    void add_pass(const char* name,
                                  const std::function<void(RenderPassBuilder&)>& setup,
                                  ExecuteFunction execute);

                    /**
                     * @brief Culls unused passes, assigns physical resources, plans lifetimes.
                     *
                     * Must be called after the last add_pass() and before execute().
                     */
                    void compile();

                    /** @brief How many command buffers this frame compiled into. */
                    std::uint32_t submission_count() const noexcept
                    {
                        return static_cast<std::uint32_t>(submissions_.size());
                    }

                    /**
                     * @brief One compiled submission: its queue and what it waits on.
                     * @param index Submission index, below submission_count().
                     * @return The queue, pass range, and wait dependency.
                     */
                    const Submission& submission(std::uint32_t index) const
                    {
                        return submissions_[index];
                    }

                    /**
                     * @brief Records one submission's passes, with barriers and rendering scopes.
                     *
                     * Must be called for the submissions in index order: the barrier state
                     * machine walks the schedule once, so recording them out of order would
                     * derive transitions from the wrong preceding access.
                     *
                     * @param cmd   The command buffer to record into, from that queue's family.
                     * @param index Which submission to record.
                     */
                    void execute(VkCommandBuffer cmd, std::uint32_t index);

                private:
                    friend class RenderPassBuilder;
                    friend class PassContext;

                    /** @brief One declared texture use inside a pass. */
                    struct TextureUse
                    {
                        TextureHandle handle;
                        TextureAccess access;
                    };

                    /** @brief One declared buffer use inside a pass. */
                    struct BufferUse
                    {
                        BufferHandle handle;
                        BufferAccess access;
                    };

                    /** @brief One colour attachment binding inside a pass. */
                    struct ColorAttachment
                    {
                        TextureHandle handle;
                        AttachmentLoad load = AttachmentLoad::Discard;
                        ClearColor clear{};
                        bool bound = false;
                    };

                    /** @brief The per-tile shading rate binding inside a pass. */
                    struct ShadingRateAttachment
                    {
                        TextureHandle handle;
                        std::uint32_t texel_width = 1;
                        std::uint32_t texel_height = 1;
                        bool bound = false;
                    };

                    /** @brief The depth/stencil attachment binding inside a pass. */
                    struct DepthAttachment
                    {
                        TextureHandle handle;
                        AttachmentLoad load = AttachmentLoad::Clear;
                        float depth = 0.0f;
                        std::uint32_t stencil = 0;
                        bool read_only = false;
                        bool bound = false;
                    };

                    /** @brief A registered pass and everything it declared. */
                    struct PassNode
                    {
                        std::string name;
                        ExecuteFunction execute;
                        std::vector<TextureUse> texture_reads;
                        std::vector<TextureUse> texture_writes;
                        std::vector<BufferUse> buffer_reads;
                        std::vector<BufferUse> buffer_writes;
                        ColorAttachment color[MAX_COLOR_ATTACHMENTS];
                        DepthAttachment depth;
                        ShadingRateAttachment shading_rate;
                        std::uint32_t color_count = 0;
                        PassQueue queue = PassQueue::Graphics;
                        VkExtent2D render_area{0, 0};
                        bool has_render_area = false;
                        bool side_effect = false;
                        bool culled = false;
                        std::uint32_t live_writes = 0;
                    };

                    /** @brief A virtual texture: its description and this frame's backing. */
                    struct TextureResource
                    {
                        TextureDesc desc{};
                        bool imported = false;
                        VkImage image = VK_NULL_HANDLE;
                        VkImageView view = VK_NULL_HANDLE;
                        VkImageView sample_view = VK_NULL_HANDLE;
                        TextureState* external_state = nullptr;
                        std::uint32_t entry = 0xFFFFFFFFu;
                        std::uint32_t readers = 0;
                        std::uint32_t first_pass = 0xFFFFFFFFu;
                        std::uint32_t last_pass = 0;
                        bool touched_by_graphics = false;
                        bool touched_by_compute = false;
                        std::vector<std::uint32_t> producers;
                    };

                    /** @brief A virtual buffer: its description and this frame's backing. */
                    struct BufferResource
                    {
                        BufferDesc desc{};
                        bool imported = false;
                        VkBuffer buffer = VK_NULL_HANDLE;
                        void* mapped = nullptr;
                        BufferState* external_state = nullptr;
                        std::uint32_t entry = 0xFFFFFFFFu;
                        std::uint32_t readers = 0;
                        std::uint32_t first_pass = 0xFFFFFFFFu;
                        std::uint32_t last_pass = 0;
                        bool touched_by_graphics = false;
                        bool touched_by_compute = false;
                        std::vector<std::uint32_t> producers;
                    };

                    void declare_texture(std::uint32_t pass, TextureHandle handle,
                                         TextureAccess access, bool is_write);
                    void declare_buffer(std::uint32_t pass, BufferHandle handle,
                                        BufferAccess access, bool is_write);
                    void cull_passes();
                    void build_submissions();
                    void mark_cross_queue_resources();
                    void assign_resources();
                    void touch(std::uint32_t pass, TextureResource& resource);
                    void touch(std::uint32_t pass, BufferResource& resource);
                    TextureState& texture_state(const TextureResource& resource);
                    BufferState& buffer_state(const BufferResource& resource);
                    void emit_barriers(VkCommandBuffer cmd, const PassNode& pass);
                    void begin_rendering(VkCommandBuffer cmd, const PassNode& pass,
                                         VkExtent2D area);
                    VkExtent2D resolve_render_area(const PassNode& pass) const;

                    Vulkan::VulkanDevice& device_;
                    Resources::TexturePool* textures_ = nullptr;
                    Resources::BufferPool* buffers_ = nullptr;
                    GpuProfiler* profiler_ = nullptr;
                    std::vector<PassNode> passes_;
                    std::vector<TextureResource> texture_resources_;
                    std::vector<BufferResource> buffer_resources_;
                    std::vector<std::uint32_t> order_;
                    std::vector<Submission> submissions_;
                    bool async_compute_enabled_ = false;
            };
        } // namespace Graph
    } // namespace Render
} // namespace SushiEngine
