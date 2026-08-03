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
 * emitters flattened to `GPUEmitter`, and the two baked LUT atlases the sim samples. The host
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
#include <SushiEngine/vfx/compiled_emitter.hpp>

namespace SushiEngine
{
    namespace Render
    {
        namespace Vulkan
        {
            class VulkanDevice;
        }

        namespace Geometry
        {
            class MeshRegistry;
        }

        namespace Assets
        {
            class TextureLibrary;
        }

        namespace Scene
        {
            /**
             * @brief One active emitter flattened for the GPU, packed to std430 vec4 groups.
             *
             * The compute-visible subset of a `VFX::CompiledEmitter` plus this frame's world
             * transform, ring cursor, and spawn count. Laid out as a 4x4 matrix followed by
             * ten vec4s so the GLSL mirror in `particle_common.glsl` needs no padding guesswork.
             */
            struct GPUEmitter
            {
                float model[16];                 /**< Emitter object-to-world (column-major). */

                std::uint32_t shape;             /**< VFX::EmitterShape. */
                std::uint32_t shape_flags;       /**< VFX::ShapeFlags. */
                std::uint32_t update_flags;      /**< VFX::UpdateFlags. */
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
                float angular_min, angular_max, velocity_stretch, pad_b;

                std::int32_t size_curve_lut;     /**< Curve-atlas row, or -1. */
                std::int32_t color_gradient_lut; /**< Gradient-atlas row, or -1. */
                std::uint32_t spawn_base;        /**< Ring cursor this frame. */
                std::uint32_t spawn_count;       /**< Particles to emit this frame. */

                std::uint32_t seed;
                std::uint32_t frame;
                std::uint32_t flipbook_rows;
                std::uint32_t flipbook_columns;

                std::uint32_t blend;     /**< VFX::BlendMode: buckets the particle (additive vs alpha). */
                std::uint32_t sort;      /**< VFX::SortMode: whether the alpha segment is depth-sorted. */
                std::uint32_t alignment; /**< VFX::RenderAlignment: how the vertex stage orients the quad. */
                std::uint32_t mesh_slot; /**< Mesh-draw slice this emitter fills, or NO_MESH_SLOT. */

                std::uint32_t force_field_count; /**< Active entries in @ref force_fields. */
                float collision_restitution;     /**< Normal velocity kept on a depth bounce. */
                float collision_friction;        /**< Tangential velocity shed on a depth bounce. */
                float collision_thickness;       /**< Depth behind a surface still counted as contact. */

                /**
                 * @brief The particle material: @ref VFX::RenderFlags bits.
                 *
                 * Carried per emitter rather than per draw because a bucket mixes emitters — the
                 * additive list holds every non-alpha sprite whatever its author asked for — so
                 * the draw is the wrong granularity for "is this lit", "is this soft", "is this
                 * textured".
                 */
                std::uint32_t render_flags;
                std::uint32_t texture;      /**< Bindless heap slot; meaningful with RENDER_TEXTURED. */
                float soft_fade_distance;   /**< Metres the soft-particle fade ramps over. */
                float pad_material;

                /**
                 * @brief The beam span, in world space (Beam alignment only).
                 *
                 * The compiled record keeps the endpoints emitter-local; like the force fields
                 * below they are transformed here, once per frame, so the vertex stage reads
                 * the space its particles already live in.
                 */
                float beam_start[3];
                float beam_width;         /**< Strip width in metres. */
                float beam_end[3];
                float beam_sag;           /**< Metres the midpoint droops below the chord. */
                float beam_noise_amplitude; /**< Lateral jitter, metres. */
                float beam_noise_frequency; /**< Jitter cycles along the span. */
                float beam_pad[2];

                /**
                 * @brief The emitter's placed force fields, baked to world space.
                 *
                 * Three `vec4`s each: (centre.xyz, strength), (axis.xyz, radius),
                 * (kind, falloff, pad, pad). The compiled record keeps them emitter-local; the
                 * transform is applied here, once per frame, so the shader evaluates them in the
                 * absolute space its particles live in.
                 */
                float force_fields[VFX::MAX_FORCE_FIELDS][12];
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
                     * @brief Trail samples kept per particle for the ribbon path.
                     *
                     * Mirrored by `TRAIL_POINTS` in particle_common.glsl. The strip a ribbon draws
                     * is one quad shorter than this.
                     */
                    static constexpr std::uint32_t TRAIL_POINTS = 8;

                    /** @brief Vertices one ribbon instance draws: six per segment. */
                    static constexpr std::uint32_t RIBBON_VERTICES = (TRAIL_POINTS - 1) * 6;

                    /**
                     * @brief Distinct meshes that can be drawn as mesh particles in one frame.
                     *
                     * Each claims an equal slice of the mesh draw list and one indexed indirect
                     * command, because one draw can bind only one mesh. Mirrored by
                     * `MAX_MESH_EMITTERS` in particle_common.glsl.
                     */
                    static constexpr std::uint32_t MAX_MESH_EMITTERS = 4;

                    /** @brief Marks an emitter that owns no mesh-draw slice. */
                    static constexpr std::uint32_t NO_MESH_SLOT = 0xFFFFFFFFu;

                    /** @brief One frame's mesh-particle draw: which mesh, and which slice it fills. */
                    struct MeshDraw
                    {
                        MeshId mesh;                 /**< Mesh the slice's particles are drawn as. */
                        std::uint32_t slot;          /**< Slice index, and so the indirect command's. */
                        std::uint32_t index_count;   /**< Indices the mesh draws. */
                    };

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
                     * Builds one @ref GPUEmitter per view (baking in its world transform, spawn
                     * cursor, and spawn count), copies the effect's curve/gradient atlases into
                     * this slot's host buffers, and advances each emitter's ring cursor by its
                     * spawn count. Records nothing on the GPU.
                     *
                     * Mesh-aligned emitters additionally claim one of the @ref MAX_MESH_EMITTERS
                     * mesh-draw slices here, which is why the mesh registry is needed: the slice's
                     * indirect command has to carry its mesh's index count, and only the host knows
                     * it.
                     *
                     * Sprite textures resolve here too, for the same reason: the authored record
                     * names a texture-library id, and only the library knows which bindless heap
                     * slot that id currently occupies. An emitter whose texture cannot be resolved
                     * loses its `RENDER_TEXTURED` bit and draws as the built-in dot, so the
                     * fragment stage never indexes the heap with an unallocated slot.
                     *
                     * @param slot        The frame slot being recorded.
                     * @param frame_index Monotonic frame counter (selects the ping-pong copy).
                     * @param emitters    The frame's cosmetic emitters.
                     * @param count       Number of entries in @p emitters.
                     * @param meshes      Registry the mesh-aligned emitters resolve their mesh in.
                     * @param textures    Library the sprite textures resolve their heap slot in.
                     */
                    void prepare(std::uint32_t slot, std::uint32_t frame_index,
                                 const ParticleEmitterView* emitters, std::size_t count,
                                 const Geometry::MeshRegistry& meshes,
                                 const Assets::TextureLibrary& textures);

                    /**
                     * @brief Uploads the frame's already-simulated deterministic billboards.
                     *
                     * Packs each into a host-visible @ref GPUParticle record (position, size,
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

                    /**
                     * @brief This frame's mesh-particle draws, at most @ref MAX_MESH_EMITTERS.
                     *
                     * One per distinct mesh-aligned emitter; the mesh pass issues one indexed
                     * indirect draw from each, and the sim pass seeds each command's index count.
                     */
                    const std::vector<MeshDraw>& mesh_draws() const noexcept { return mesh_draws_; }

                    /** @brief Particles one mesh-draw slice holds. */
                    std::uint32_t mesh_slice() const noexcept
                    {
                        return capacity_ / MAX_MESH_EMITTERS;
                    }

                    /** @brief The active emitters flattened for the GPU this frame. */
                    const std::vector<GPUEmitter>& emitters() const noexcept { return emitters_; }

                    /** @brief The shared pool's particle capacity. */
                    std::uint32_t capacity() const noexcept { return capacity_; }

                    /** @brief The shared, persistent particle pool. */
                    VkBuffer pool() const noexcept;

                    /** @brief Bytes of the pool. */
                    VkDeviceSize pool_range() const noexcept;

                    /**
                     * @brief Recent positions kept per pool slot for the ribbon path.
                     *
                     * Persistent and device-local like the pool, and for the same reason: a trail
                     * is state that has to survive the frame that recorded it. Each slot holds
                     * @ref TRAIL_POINTS `vec4`s — xyz the sample's world position, w its size —
                     * newest first, shifted one place by every simulate step.
                     */
                    VkBuffer trail_buffer() const noexcept;

                    /** @brief Bytes of the trail history. */
                    VkDeviceSize trail_range() const noexcept;

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
                    Allocation trails_;                       /**< Persistent per-slot trail history. */
                    std::vector<Allocation> emitter_tables_;  /**< Host-visible GPUEmitter[], per slot. */
                    std::vector<Allocation> curve_luts_;      /**< Host-visible curve atlas, per slot. */
                    std::vector<Allocation> gradient_luts_;   /**< Host-visible gradient atlas, per slot. */
                    std::vector<Allocation> billboards_;      /**< Host-visible GPUParticle[], per slot. */
                    std::uint32_t billboard_count_ = 0;       /**< This frame's deterministic billboards. */
                    std::vector<GPUEmitter> emitters_;        /**< This frame's flattened emitters. */
                    std::vector<MeshDraw> mesh_draws_;        /**< This frame's mesh-particle draws. */
                    std::uint32_t ring_cursor_ = 0;           /**< Shared pool ring write cursor. */
                    bool needs_clear_ = true;                 /**< Pool awaits its one-time zero clear. */
                    bool has_alpha_ = false;                  /**< Any active emitter uses true-alpha blending. */
                    VkDeviceSize curve_bytes_ = sizeof(float);
                    VkDeviceSize gradient_bytes_ = sizeof(float);
            };
        } // namespace Scene
    } // namespace Render
} // namespace SushiEngine
