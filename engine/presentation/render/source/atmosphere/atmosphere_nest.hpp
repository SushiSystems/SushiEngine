/**************************************************************************/
/* atmosphere_nest.hpp                                                    */
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
 * @file atmosphere_nest.hpp
 * @brief T2: the regional nest — anelastic non-hydrostatic convection, on the GPU.
 *
 * `docs/design/atmosphere_system.md` §6. The replacement for `RegionalWeatherGrid`, whose audit
 * (§1.3–§1.5) found three independent 2-D layers with no vertical advection at all, saturation
 * expressed as `if (humidity > 0.85)`, no latent heating, and a semi-Lagrangian scheme running
 * at Courant ≈ 0.02 — its maximally diffusive regime — in a single-threaded scalar CPU loop.
 * What runs here instead is a real cloud model: prognostic potential temperature, three
 * moisture species and a staggered velocity field, transported at Courant ≈ 1 by a limited
 * MacCormack scheme, closed by an anelastic pressure projection, and rained out by Kessler
 * warm-rain microphysics with latent heating. A cumulus grows because condensation warms the
 * parcel that lifted the vapour, not because a coverage number went up.
 *
 * **Why this is a device-level service and not a render pass.** The editor runs three
 * `ISceneView`s (Scene, Game, and the VFX preview). A per-view nest would simulate three
 * divergent atmospheres at three times the cost and several hundred megabytes, and the
 * simulation would have to pick one of them to answer "what is the weather". There is one
 * atmosphere, so it lives beside the other device-level services in `AssetLibrary`, is stepped
 * once per frame from `IWindowRenderer::begin_frame`, and is centred on the *simulation's
 * observer* rather than on any camera — which decouples it from views entirely.
 *
 * **How its writes are ordered against three readers.** The step records into its own command
 * buffer and submits on the graphics queue, signalling a timeline semaphore. Each scene view's
 * first submission waits on that value, which is what makes the cross-submit read of a shared
 * mutable resource ordered rather than merely usually-fine. The engine already requires Vulkan
 * 1.4 and `ViewResources` already runs its own per-queue timelines, so this is the same
 * mechanism the render graph's queue hand-off uses, not a new one.
 *
 * **Nothing reads the state synchronously.** Gameplay reads @ref atmosphere_mirror, an
 * asynchronous readback two or three frames old (§3.2). That is the whole concurrency story.
 */

#include <cstdint>
#include <memory>
#include <vector>

#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>

#include <SushiEngine/environment/atmosphere_nest.hpp>

#include "graph/gpu_profiler.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Resources
        {
            class GraphicsPipelineFactory;
            class SamplerCache;
            class ShaderLibrary;
        }

        namespace Vulkan
        {
            class VulkanDevice;
        }

        namespace Atmosphere
        {
            /**
             * @brief The regional nest: its GPU state, its step, and its readback mirror.
             *
             * Non-copyable; it owns images, buffers, pipelines, a command pool and a timeline.
             */
            class AtmosphereNest final : public IAtmosphereMirror
            {
                public:
                    /**
                     * @brief Brings up the nest at @p size and builds its pipelines.
                     * @param device    The live Vulkan device.
                     * @param shaders   Library the compute modules come from.
                     * @param pipelines Factory the compute pipelines are built through.
                     * @param samplers  Cache providing the clamped, linear sampler the fields read under.
                     * @param size      The discretization; fixed for the nest's lifetime.
                     */
                    AtmosphereNest(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                                   Resources::GraphicsPipelineFactory& pipelines,
                                   Resources::SamplerCache& samplers, const AtmosphereNestSize& size);
                    ~AtmosphereNest() override;

                    AtmosphereNest(const AtmosphereNest&) = delete;
                    AtmosphereNest& operator=(const AtmosphereNest&) = delete;

                    /**
                     * @brief Advances the nest by however much game time the forcing reports.
                     *
                     * Called once per frame, before any view renders. Decides the step count
                     * from the forcing's elapsed game time and the CFL-chosen step length,
                     * re-centres the lattice if the observer has drifted a whole cell, records
                     * every due step into one command buffer, and submits it. A frame with no
                     * step due records nothing and submits nothing.
                     *
                     * @param parameters The authored physics; uploaded fresh each step.
                     * @param forcing    The parent solution, the observer, and the elapsed time.
                     * @param readers    Timeline waits the submission must not overtake — the
                     *                   views that sampled the *previous* step's fields this
                     *                   frame. Optional; a caller with no concurrent readers
                     *                   (the headless probe) passes none.
                     * @param reader_count Entries in @p readers.
                     */
                    void step(const AtmosphereNestParameters& parameters,
                              const AtmosphereForcing& forcing,
                              const VkSemaphoreSubmitInfo* readers = nullptr,
                              std::uint32_t reader_count = 0);

                    AtmosphereMirror atmosphere_mirror() const noexcept override;

                    /** @brief Rebuilds the compute pipelines after a shader edit. */
                    void rebuild_pipelines();

                    /**
                     * @brief The timeline a scene view's first submission must wait on.
                     *
                     * Zero when the nest has never stepped, which a caller reads as "nothing to
                     * wait for" — a view that renders before the first step has nothing of the
                     * nest's to read anyway.
                     */
                    VkSemaphore timeline() const noexcept { return timeline_; }

                    /** @brief The timeline value the most recent step signals. */
                    std::uint64_t timeline_value() const noexcept { return timeline_value_; }

                    /**
                     * @brief The optical extinction field, for the cloudscape bake to read.
                     *
                     * §7.1's `sigma_ext`, in 1/m, over the nest's own lattice. Consumed directly
                     * by the bake in the increment that follows this one; until then it is what
                     * the readback derives coverage and cloud base from.
                     */
                    VkImageView extinction_view() const noexcept { return extinction_.view; }

                    /** @brief The sampler every nest field is read under (linear, edge-clamped). */
                    VkSampler sampler() const noexcept { return sampler_; }

                    /** @brief The nest's discretization. */
                    const AtmosphereNestSize& size() const noexcept { return size_; }

                    /** @brief Scene-absolute XZ metres of the low corner of cell (0, 0). */
                    void origin(double& x, double& z) const noexcept;

                    /** @brief Total game seconds the nest has simulated. */
                    double simulated_seconds() const noexcept { return simulated_seconds_; }

                    /**
                     * @brief Seconds of game time the most recent step advanced by.
                     *
                     * Not a constant: the vertical CFL is taken against the updraft the nest is
                     * measured to be producing, so this lengthens in a quiet airmass and
                     * tightens when convection gets going. Zero before the first step.
                     */
                    float step_seconds() const noexcept { return step_seconds_; }

                    /** @brief Steps taken since construction — the editor's "is it running" readout. */
                    std::uint64_t step_count() const noexcept { return step_index_; }

                    /**
                     * @brief What the most recently completed *stepping* frame cost, per stage.
                     *
                     * Unmeasured until one has been read back, which takes as many frames as the
                     * readback ring is deep. See @ref AtmosphereStepCost for why it is a
                     * breakdown rather than a single number.
                     */
                    const AtmosphereStepCost& step_cost() const noexcept { return step_cost_; }

                private:
                    /** @brief One 3-D field: image, view, allocation. */
                    struct Volume
                    {
                        VkImage image = VK_NULL_HANDLE;
                        VmaAllocation allocation = VK_NULL_HANDLE;
                        VkImageView view = VK_NULL_HANDLE;
                    };

                    /** @brief A pair of volumes advection needs both states of. */
                    struct Buffered
                    {
                        Volume front;
                        Volume back;
                        void swap() noexcept
                        {
                            Volume temporary = front;
                            front = back;
                            back = temporary;
                        }
                    };

                    /**
                     * @brief The parameter block, laid out to match `atmosphere_nest_common.glsl`.
                     *
                     * A separate type from `AtmosphereNestParameters` on purpose: that one
                     * is what an author edits and the scene serializes, this one is what
                     * std140 wants, and conflating them would put a `bool` and a step
                     * counter into a file format.
                     */
                    struct NestParameters
                    {
                        float gas_constant_dry;
                        float gas_constant_vapour;
                        float specific_heat_pressure;
                        float latent_heat_vaporization;
                        float gravity;
                        float reference_pressure;
                        float water_density;

                        float surface_temperature;
                        float surface_pressure;
                        float lapse_rate;
                        float tropopause_altitude;
                        float surface_humidity;
                        float humidity_scale_height;
                        float free_troposphere_drying;
                        float free_troposphere_exponent;

                        float eddy_viscosity;
                        float boundary_layer_depth;
                        float boundary_layer_velocity_scale;
                        float sponge_depth;
                        float sponge_rate;
                        float boundary_relaxation;
                        float thermal_seed_amplitude;
                        float thermal_seed_length;
                        float thermal_seed_period;
                        float coriolis;
                        float convective_velocity_scale;

                        float cloud_top_longwave_flux;
                        float cloud_top_equilibrium_depression;
                        float cloud_top_entrainment_efficiency;
                        float cloud_water_absorption;
                        float cloud_critical_humidity;
                        float autoconversion_rate;
                        float autoconversion_threshold;
                        float accretion_rate;
                        float accretion_exponent;
                        float rain_evaporation_rate;
                        float fall_speed_coefficient;
                        float fall_speed_exponent;
                        float droplet_effective_radius;
                        float latent_heat_fusion;
                        float freezing_temperature;
                        float glaciation_temperature;
                        float snow_fall_speed_coefficient;
                        float snow_fall_speed_exponent;
                        float glaciated_autoconversion_factor;
                        float ice_effective_radius;

                        float solar_constant;
                        float clear_sky_transmittance;
                        float surface_albedo;
                        float surface_emissivity;
                        float surface_heat_capacity;
                        float surface_moisture_availability;
                        float surface_exchange_coefficient;
                        float surface_minimum_wind;
                        float solar_elevation_sine;

                        float spacing;
                        float domain_top;
                        float dt;
                        std::int32_t cells_x;
                        std::int32_t cells_z;
                        std::int32_t levels;
                        std::int32_t boundary_zone;
                        // 58 scalars is 232 bytes, which std140 rounds a block up to 240 — so the
                        // two floats below are that rounding, written down rather than left to
                        // the compiler to imply. Keep the total a multiple of four scalars: the
                        // host binds `sizeof(NestParameters)` as the range, and a struct short
                        // of the block the shader declares binds less than it reads. Neither
                        // side is edited without the other.
                        float padding[2];
                    };

                    // The push blocks these two stages take are *not* declared here. They were,
                    // in a second copy that nothing referenced and that therefore could not be
                    // kept honest: the real ones live in the anonymous namespace of the
                    // translation unit that fills them, beside the shaders' `layout(push_constant)`
                    // they have to match field for field.

                    /** @brief One frame's readback slot: a host-visible copy and what it holds. */
                    struct MirrorSlot
                    {
                        VkBuffer buffer = VK_NULL_HANDLE;
                        VmaAllocation allocation = VK_NULL_HANDLE;
                        void* mapped = nullptr;
                        VkBuffer profile_buffer = VK_NULL_HANDLE;
                        VmaAllocation profile_allocation = VK_NULL_HANDLE;
                        void* profile_mapped = nullptr;
                        std::uint64_t timeline_value = 0; /**< 0 = never filled. */
                        double simulated_seconds = 0.0;
                        double origin_x = 0.0;
                        double origin_z = 0.0;
                        /**
                         * @brief Whether this slot's submission recorded a step of the physics.
                         *
                         * A frame that only re-centres the lattice still records a shift, an
                         * extinction and a readback, and publishing that as "the step's cost"
                         * would report a fraction of one whenever the observer moved.
                         */
                        bool stepped = false;
                        /** @brief The nest's step index at the time, for the published cost. */
                        std::uint64_t step_index = 0;
                        /** @brief Steps this submission recorded; the cost's divisor. */
                        std::uint32_t steps = 0;
                    };

                    /** @brief How many readback slots the ring carries; matches the frame depth. */
                    static constexpr std::uint32_t MIRROR_SLOTS = 3;

                    /**
                     * @brief Descriptor sets one recording slot's pool can serve in a frame.
                     *
                     * A frame records a shift, an extinction and a readback, plus every stage of
                     * every step it takes and two sets per pressure sweep. This is therefore
                     * what caps @ref AtmosphereNestParameters::max_steps_per_frame in practice, and
                     * `step()` reads it to clamp the step count rather than trusting the pool to
                     * be big enough — 512 sets is a few tens of kilobytes and covers four steps
                     * at twice the default sweep count.
                     */
                    static constexpr std::uint32_t DESCRIPTOR_SETS_PER_SLOT = 512;

                    void create_volumes();
                    void destroy_volumes();
                    void create_volume(Volume& volume, VkFormat format, std::uint32_t width,
                                       std::uint32_t height, std::uint32_t depth);
                    void create_buffers();
                    void destroy_buffers();
                    void create_pipelines();
                    void destroy_pipelines();
                    void create_layouts();
                    void destroy_layouts();
                    void create_commands();
                    void destroy_commands();

                    float choose_step(const AtmosphereNestParameters& parameters,
                                      const AtmosphereForcing& forcing) const;
                    void upload_parameters(const AtmosphereNestParameters& parameters, float dt);
                    void upload_forcing(VkCommandBuffer command, const AtmosphereForcing& forcing);
                    void record_shift(VkCommandBuffer command, std::int32_t shift_x,
                                      std::int32_t shift_z, const AtmosphereForcing& forcing);
                    void record_clear(VkCommandBuffer command);
                    /**
                     * @brief Records one step of the nest into @p command.
                     * @param step_seconds Game seconds this step *begins* at. The thermal seed is
                     *                     correlated in time, so it needs the step's own clock —
                     *                     and the parameter block, uploaded once for a frame that
                     *                     may record several steps, cannot carry it.
                     * @param timed        Whether to break this step into profiled sections. One
                     *                     step of a frame at most; the query pool is finite.
                     */
                    void record_step(VkCommandBuffer command, const AtmosphereForcing& forcing,
                                     double step_seconds, bool timed);
                    void record_extinction(VkCommandBuffer command);
                    void record_readback(VkCommandBuffer command, std::uint32_t slot);
                    void collect_readback();
                    void publish_cost(std::uint32_t slot);
                    VkDescriptorSet allocate(std::uint32_t stage, std::uint32_t slot);
                    void prepare_layouts(VkCommandBuffer command);

                    Vulkan::VulkanDevice& device_;
                    Resources::ShaderLibrary& shaders_;
                    Resources::GraphicsPipelineFactory& pipelines_;
                    AtmosphereNestSize size_;

                    Buffered wind_x_;
                    Buffered wind_y_;
                    Buffered wind_z_;
                    Buffered theta_;
                    Buffered moisture_;
                    Volume pressure_;
                    Volume divergence_;
                    Volume extinction_;
                    Volume surface_rain_; /**< 2-D, one texel per horizontal cell. */
                    /**
                     * @brief 2-D surface state: skin temperature (K), H, LE, net radiation (W/m²).
                     *
                     * Double-buffered for the *shift* and nothing else. It is the only field
                     * outside the prognostic five that survives a re-centre — the skin
                     * temperature is a state variable, not a step output, so a nest that walks
                     * two kilometres must bring the ground's warmth with it rather than
                     * re-deriving it. The surface stage itself writes it in place.
                     */
                    Buffered surface_;
                    /**
                     * @brief 2-D per-column cloud: cell-mean optical depth, and the cover it
                     *        spans.
                     *
                     * Written by the extinction stage, which runs once a *frame*, and read by
                     * the surface stage, which runs once a *step* — so the ground is always
                     * shaded by the cloud the previous frame ended with. Single-buffered: the
                     * two never touch it in the same dispatch.
                     */
                    Volume cloud_shade_;
                    Volume forcing_;      /**< 2-D, the parent solution. */
                    /**
                     * @brief 2-D, the parent's large-scale vertical motion. Positive is ascent.
                     *
                     * Separate from @ref forcing_ because that image's four channels are full,
                     * and separate *in kind* as well: the parent solution above is what the
                     * Davies zone relaxes toward, while this is applied across the whole domain
                     * — a motion the nest cannot generate rather than a boundary condition.
                     */
                    Volume forcing_vertical_;
                    VkSampler sampler_ = VK_NULL_HANDLE;

                    VkBuffer parameters_ = VK_NULL_HANDLE;
                    VmaAllocation parameters_allocation_ = VK_NULL_HANDLE;
                    void* parameters_mapped_ = nullptr;
                    VkBuffer mirror_ = VK_NULL_HANDLE;
                    VmaAllocation mirror_allocation_ = VK_NULL_HANDLE;
                    VkBuffer profile_ = VK_NULL_HANDLE;
                    VmaAllocation profile_allocation_ = VK_NULL_HANDLE;
                    VkBuffer forcing_staging_ = VK_NULL_HANDLE;
                    VmaAllocation forcing_staging_allocation_ = VK_NULL_HANDLE;
                    void* forcing_staging_mapped_ = nullptr;
                    MirrorSlot mirror_slots_[MIRROR_SLOTS]{};

                    // One entry per stage of the step, in STAGES' own order. Built from a
                    // table rather than ten hand-written blocks: every layout here is "the
                    // parameter block, then a run of sampled images, storage images and one
                    // storage buffer", and a table says that once instead of ten times.
                    static constexpr std::uint32_t STAGE_COUNT = 11;
                    VkDescriptorSetLayout layouts_[STAGE_COUNT]{};
                    VkPipelineLayout pipeline_layouts_[STAGE_COUNT]{};
                    VkPipeline stage_pipelines_[STAGE_COUNT]{};
                    // One pool per in-flight slot: the sets a step allocates stay live until
                    // that step completes, so a single pool could never be reset without
                    // invalidating the two steps still in flight beside it.
                    VkDescriptorPool descriptor_pools_[MIRROR_SLOTS]{};
                    bool layouts_ready_ = false;

                    VkCommandPool command_pool_ = VK_NULL_HANDLE;
                    VkCommandBuffer commands_[MIRROR_SLOTS]{};
                    VkSemaphore timeline_ = VK_NULL_HANDLE;
                    std::uint64_t timeline_value_ = 0;
                    std::uint32_t slot_ = 0;

                    // The absolute lattice the nest is snapped to, in cells. Integer so a
                    // re-centre is an exact translation and a surviving cell is copied rather
                    // than resampled — the same discipline the cloudscape window uses.
                    long long origin_cell_x_ = 0;
                    long long origin_cell_z_ = 0;
                    bool seeded_ = false;

                    double last_total_seconds_ = 0.0;
                    double pending_seconds_ = 0.0;
                    double simulated_seconds_ = 0.0;
                    std::uint64_t step_index_ = 0;

                    // The host-side copy the mirror view points at, refilled from whichever slot
                    // most recently completed. Kept here rather than handing out the mapped
                    // pointer so a consumer can never read a slot the GPU is refilling.
                    std::vector<AtmosphereMirrorColumn> mirror_columns_;
                    std::vector<AtmosphereProfileLevel> profile_levels_;
                    AtmosphereMirror mirror_view_{};
                    // The timeline value of the snapshot currently held above, so a completed
                    // slot is copied out exactly once however many frames it stays completed.
                    std::uint64_t mirror_taken_ = 0;

                    // Timestamp queries around each recorded section, one pool per in-flight
                    // slot. The render graph's profiler rather than a second implementation of
                    // the same thing: it depends on nothing but the device, and the alternative
                    // is a hundred lines of query-pool plumbing that would drift from it.
                    // Resolved where the slot's timeline value is already known to have passed,
                    // so the measurement costs no stall of its own.
                    std::unique_ptr<Graph::GPUProfiler> profiler_;
                    AtmosphereStepCost step_cost_{};

                    // Resolved from the authored parameters each step; the shader loop needs a
                    // count and the parameter block is uploaded once, not per sweep.
                    std::uint32_t pressure_sweeps_ = 12;
                    float solar_elevation_sine_ = 0.0f;
                    float coriolis_ = 0.0f;
                    // The thermal seed's lattice spacing, metres. Held here for the same reason as
                    // the two above: `record_step` needs it to wrap the world origin the seed is
                    // addressed in, and it is handed the step rather than the parameters.
                    float seed_length_m_ = 6000.0f;

                    /**
                     * @brief Domain peak |w| from the last readback, m/s; the vertical CFL's bound.
                     *
                     * Zero until the first readback, which `choose_step` reads as "nothing
                     * measured yet, keep the conservative constant". Tracked with a decay so it
                     * *shortens* the step the instant convection strengthens and only lengthens
                     * it again slowly — the asymmetry a stability bound wants, since being late
                     * to tighten is a wrong answer and being late to relax is only a cost.
                     */
                    float measured_updraft_ = 0.0f;

                    /** @brief The step length most recently chosen, seconds of game time. */
                    float step_seconds_ = 0.0f;
            };
        } // namespace Atmosphere
    } // namespace Render
} // namespace SushiEngine
