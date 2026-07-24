/**************************************************************************/
/* particle_system.hpp                                                    */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
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
 * @file particle_system.hpp
 * @brief The GPU cosmetic particle backend's persistent pools and per-frame emitter table.
 *
 * Modeled on `SkinningSystem`: a non-copyable owner of VMA allocations that the host packs
 * each frame and the compute/draw passes consume. It holds one **shared** particle pool
 * (double-buffered so a frame reads last frame's state and writes this frame's, which keeps
 * the ping-pong safe across frames in flight), a per-slot table of the frame's active
 * emitters flattened to `GpuEmitter`, and the two baked LUT atlases the sim samples. The host
 * advances each emitter's ring cursor and decides its spawn count; the emit shader stays a
 * pure allocator writing into the ring.
 *
 * State pools are system-owned (they carry particles frame to frame and must never be graph
 * transients, which alias and recycle); the per-frame compacted draw list and the indirect
 * draw arguments the billboard pass consumes are graph transients declared in
 * `view_resources.cpp`, so the graph derives the compute→draw barriers.
 *
 * The pool is a single in-place buffer touched only by the compute passes (the draw pass reads
 * the compacted transient, never the pool), so within a frame a hand barrier orders the sim
 * write against the next read. Cross-frame the write→read is gated by the frame's own pacing;
 * fully decoupling it from frames in flight is a VFX2 hardening, noted where it matters.
 */

#include <cstddef>
#include <cstdint>
#include <vector>

#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>

#include <SushiEngine/render/scene_view.hpp>

namespace SushiEngine
{
    namespace Render
    {
        namespace Vulkan
        {
            class VulkanDevice;
        }

        namespace Scene
        {
            /**
             * @brief One active emitter flattened for the GPU, packed to std430 vec4 groups.
             *
             * The compute-visible subset of a `Vfx::CompiledEmitter` plus this frame's world
             * transform, ring cursor, and spawn count. Laid out as a 4x4 matrix followed by
             * ten vec4s so the GLSL mirror in `particle_common.glsl` needs no padding guesswork.
             */
            struct GpuEmitter
            {
                float model[16];                 /**< Emitter object-to-world (column-major). */

                std::uint32_t shape;             /**< Vfx::EmitterShape. */
                std::uint32_t shape_flags;       /**< Vfx::ShapeFlags. */
                std::uint32_t update_flags;      /**< Vfx::UpdateFlags. */
                std::uint32_t capacity;          /**< Shared pool capacity (all emitters). */

                float shape_radius;
                float shape_cone_angle;
                float shape_arc;
                float drag_coefficient;

                float box_half_extents[3];
                float turbulence_frequency;

                float gravity[3];
                float turbulence_amplitude;

                float color[3];
                float pad_color;

                float lifetime_min, lifetime_max, speed_min, speed_max;
                float size_min, size_max, rotation_min, rotation_max;
                float angular_min, angular_max, pad_a, pad_b;

                std::int32_t size_curve_lut;     /**< Curve-atlas row, or -1. */
                std::int32_t color_gradient_lut; /**< Gradient-atlas row, or -1. */
                std::uint32_t spawn_base;        /**< Ring cursor this frame. */
                std::uint32_t spawn_count;       /**< Particles to emit this frame. */

                std::uint32_t seed;
                std::uint32_t frame;
                std::uint32_t flipbook_rows;
                std::uint32_t flipbook_columns;

                std::uint32_t blend;   /**< Vfx::BlendMode: buckets the particle (additive vs alpha). */
                std::uint32_t sort;    /**< Vfx::SortMode: whether the alpha segment is depth-sorted. */
                std::uint32_t pad0;
                std::uint32_t pad1;
            };

            /**
             * @brief Persistent particle pools plus the per-frame emitter table and LUT atlases.
             *
             * Non-copyable: it owns VMA allocations.
             */
            class ParticleSystem
            {
                public:
                    /**
                     * @brief Allocates the shared pool and the per-slot upload buffers.
                     * @param device      The live Vulkan device.
                     * @param frame_slots Number of frames in flight.
                     * @param capacity    The shared pool's particle budget.
                     */
                    ParticleSystem(Vulkan::VulkanDevice& device, std::uint32_t frame_slots,
                                   std::uint32_t capacity);
                    ~ParticleSystem();

                    ParticleSystem(const ParticleSystem&) = delete;
                    ParticleSystem& operator=(const ParticleSystem&) = delete;

                    /**
                     * @brief Flattens the frame's emitters and uploads the table plus LUT atlases.
                     *
                     * Builds one @ref GpuEmitter per view (baking in its world transform, spawn
                     * cursor, and spawn count), copies the effect's curve/gradient atlases into
                     * this slot's host buffers, and advances each emitter's ring cursor by its
                     * spawn count. Records nothing on the GPU.
                     *
                     * @param slot        The frame slot being recorded.
                     * @param frame_index Monotonic frame counter (selects the ping-pong copy).
                     * @param emitters    The frame's cosmetic emitters.
                     * @param count       Number of entries in @p emitters.
                     */
                    void prepare(std::uint32_t slot, std::uint32_t frame_index,
                                 const ParticleEmitterView* emitters, std::size_t count);

                    /**
                     * @brief Uploads the frame's already-simulated deterministic billboards.
                     *
                     * Packs each into a host-visible @ref GpuParticle record (position, size,
                     * colour, alpha, rotation) the billboard pass draws directly — no GPU
                     * simulation. Independent of the emitter pool.
                     *
                     * @param slot       The frame slot being recorded.
                     * @param billboards The frame's deterministic particles.
                     * @param count      Number of entries in @p billboards.
                     */
                    void prepare_billboards(std::uint32_t slot, const ParticleBillboard* billboards,
                                            std::size_t count);

                    /** @brief Whether the frame has any cosmetic emitters to simulate. */
                    bool empty() const noexcept { return emitters_.empty(); }

                    /** @brief Whether any active emitter is true-alpha (so the sort is worth running). */
                    bool has_alpha() const noexcept { return has_alpha_; }

                    /** @brief Whether the frame has any deterministic billboards to draw. */
                    bool billboards_empty() const noexcept { return billboard_count_ == 0; }

                    /** @brief Number of deterministic billboards this frame. */
                    std::uint32_t billboard_count() const noexcept { return billboard_count_; }

                    /** @brief This slot's host-visible deterministic-billboard buffer. */
                    VkBuffer billboard_buffer(std::uint32_t slot) const noexcept;

                    /** @brief Bytes of this frame's billboard buffer. */
                    VkDeviceSize billboard_range() const noexcept;

                    /** @brief The active emitters flattened for the GPU this frame. */
                    const std::vector<GpuEmitter>& emitters() const noexcept { return emitters_; }

                    /** @brief The shared pool's particle capacity. */
                    std::uint32_t capacity() const noexcept { return capacity_; }

                    /** @brief The shared, persistent particle pool. */
                    VkBuffer pool() const noexcept;

                    /** @brief Bytes of the pool. */
                    VkDeviceSize pool_range() const noexcept;

                    /** @brief Whether the device-local pool still needs its one-time zero clear. */
                    bool needs_clear() const noexcept { return needs_clear_; }

                    /** @brief Records that the pool has been cleared, so the sim pass clears once. */
                    void mark_cleared() noexcept { needs_clear_ = false; }

                    /** @brief This slot's host-visible emitter table. */
                    VkBuffer emitter_buffer(std::uint32_t slot) const noexcept;

                    /** @brief Bytes of this frame's emitter table. */
                    VkDeviceSize emitter_range() const noexcept;

                    /** @brief This slot's host-visible baked curve-LUT atlas. */
                    VkBuffer curve_lut_buffer(std::uint32_t slot) const noexcept;

                    /** @brief This slot's host-visible baked gradient-LUT atlas. */
                    VkBuffer gradient_lut_buffer(std::uint32_t slot) const noexcept;

                    /** @brief Bytes of this frame's curve-LUT atlas (at least one float). */
                    VkDeviceSize curve_lut_range() const noexcept;

                    /** @brief Bytes of this frame's gradient-LUT atlas (at least one float). */
                    VkDeviceSize gradient_lut_range() const noexcept;

                private:
                    struct Allocation
                    {
                        VkBuffer buffer = VK_NULL_HANDLE;
                        VmaAllocation allocation = VK_NULL_HANDLE;
                        void* mapped = nullptr;
                        VkDeviceSize capacity = 0;
                    };

                    void grow(Allocation& target, VkDeviceSize bytes, VkBufferUsageFlags usage,
                              bool host_visible, bool zero_initialize);
                    void destroy(Allocation& target);

                    Vulkan::VulkanDevice& device_;
                    std::uint32_t capacity_ = 0;
                    Allocation pool_;                         /**< Shared persistent particle pool. */
                    std::vector<Allocation> emitter_tables_;  /**< Host-visible GpuEmitter[], per slot. */
                    std::vector<Allocation> curve_luts_;      /**< Host-visible curve atlas, per slot. */
                    std::vector<Allocation> gradient_luts_;   /**< Host-visible gradient atlas, per slot. */
                    std::vector<Allocation> billboards_;      /**< Host-visible GpuParticle[], per slot. */
                    std::uint32_t billboard_count_ = 0;       /**< This frame's deterministic billboards. */
                    std::vector<GpuEmitter> emitters_;        /**< This frame's flattened emitters. */
                    std::uint32_t ring_cursor_ = 0;           /**< Shared pool ring write cursor. */
                    bool needs_clear_ = true;                 /**< Pool awaits its one-time zero clear. */
                    bool has_alpha_ = false;                  /**< Any active emitter uses true-alpha blending. */
                    VkDeviceSize curve_bytes_ = sizeof(float);
                    VkDeviceSize gradient_bytes_ = sizeof(float);
            };
        } // namespace Scene
    } // namespace Render
} // namespace SushiEngine
