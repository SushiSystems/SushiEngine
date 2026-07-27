/**************************************************************************/
/* cloudscape_compile_pass.hpp                                           */
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
 * @file cloudscape_compile_pass.hpp
 * @brief The cloudscape T3 bake: the simulated atmosphere compiled into sampled density.
 *
 * Bakes the cloud density field once per rebake instead of once per march sample, so the view
 * march (CloudPass) spends a couple of fetches where it used to spend up to ~48 (8 texture
 * reads x up to 6 decks). Also bakes a coarser max-pooled copy the march's cheap/coarse probe
 * reads. The images are private to this pass and barriered by hand, exactly as the atmosphere
 * LUTs are; the render graph only schedules the pass, and only when there is something to
 * rebake.
 *
 * **Phase B (`docs/slop/atmosphere_system.md` §7.2/§7.4) changed what the field *is*.** It was
 * a periodic 32 km tile addressed by `fract(p.xz / tile)`, camera-locked and wind-scrolled at
 * sample time, carrying one globally compiled deck stack. It is now:
 *
 * - **Two camera-centred, non-wrapping windows.** A near one at the old span and texel size,
 *   and a far one eight times wider for the part of the march that leaves it. Non-wrapping is
 *   what gives every texel a recoverable world position — the property CloudLightVolumePass
 *   and CloudShadowMapPass needed before they could carry spatial weather at all, and the
 *   reason phase A had to defer them.
 * - **Resolved from the simulation, per column.** When the published field classifies
 *   (`WeatherField::derives_genus`) the bake picks a genus and a coverage per baked column
 *   rather than instantiating the compiled deck stack everywhere, so a stratus sheet, a
 *   cumulus field and a cirrus deck coexist where the simulation actually put them. A manually
 *   authored sky still bakes its own deck stack, unchanged.
 * - **Rebaked on a cadence rather than on an author edit.** The old gate was a memcmp of the
 *   deck stack, which is right for a sky that only changes when someone types. This one also
 *   watches the camera drifting out of its window, the weather advancing, the wind blowing,
 *   and — for the far window's own sun-depth channel — the sun moving. Each window carries its
 *   own budget, because the far one resolves eight times coarser and needs proportionally less
 *   of everything.
 *
 * The window is decided in @ref update_window, called by the scene view *before* the scene
 * uniform block is uploaded, because the bake and every consumer of the bake have to read the
 * same mapping out of that block. The pass owns the state; the view only owns the ordering.
 */

#include "passes/render_pass.hpp"

#include <cstdint>

#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>

namespace SushiEngine
{
    namespace Render
    {
        struct Environment;

        namespace Scene
        {
            struct SceneUniforms;
        }

        namespace Resources
        {
            class GraphicsPipelineFactory;
            class SamplerCache;
            class ShaderLibrary;
        }

        namespace Textures
        {
            class CloudNoise;
        }

        namespace Vulkan
        {
            class VulkanDevice;
        }

        namespace Passes
        {
            class WeatherFieldPass;

            /**
             * @brief Builds and owns the baked cloudscape windows and the near skip field.
             *
             * Non-copyable: it owns images, views, a uniform buffer, and compute pipelines.
             */
            class CloudscapeCompilePass final : public IRenderPass
            {
                public:
                    /**
                     * @brief Builds the window images, the genus catalogue, and the bake pipelines.
                     * @param device    The live Vulkan device.
                     * @param shaders   Library the compute modules come from.
                     * @param pipelines Factory the compute pipelines are built through.
                     * @param samplers  Cache providing the windows' edge-clamped sampler.
                     * @param noise     The cloud noise volumes the bake samples.
                     * @param weather   The uploaded simulation field the bake resolves columns from.
                     */
                    CloudscapeCompilePass(Vulkan::VulkanDevice& device,
                                          Resources::ShaderLibrary& shaders,
                                          Resources::GraphicsPipelineFactory& pipelines,
                                          Resources::SamplerCache& samplers,
                                          Textures::CloudNoise& noise,
                                          WeatherFieldPass& weather);
                    ~CloudscapeCompilePass() override;

                    CloudscapeCompilePass(const CloudscapeCompilePass&) = delete;
                    CloudscapeCompilePass& operator=(const CloudscapeCompilePass&) = delete;

                    /**
                     * @brief Places this frame's windows and stamps their mapping into @p uniforms.
                     *
                     * Must run before the scene block is uploaded and before @ref register_pass:
                     * it is what decides whether anything is rebaked this frame, and every
                     * consumer of the bake reads the resulting `cloud_field_*` members. Snaps each
                     * window's origin to its own texel lattice, so re-centring the window on a
                     * moved camera reproduces the identical pattern at the identical world point
                     * and a rebake is invisible.
                     *
                     * @param frame       The frame being recorded; supplies the eye and the slot.
                     * @param environment The scene's environment; supplies the cloudscape and the field.
                     * @param uniforms    The block being filled, read for the deck stack and the
                     *                    march shell, written for the window mapping.
                     */
                    void update_window(const Frame::FrameContext& frame,
                                       const Environment& environment,
                                       Scene::SceneUniforms& uniforms);

                    void register_pass(Graph::RenderGraph& graph,
                                       const Frame::FrameContext& frame) override;
                    void rebuild_pipelines() override;

                    /** @brief The near window's fine field: r = density, g = vertical profile. */
                    VkImageView field_view() const noexcept { return near_.view; }

                    /** @brief The max-pooled downsample of the near window the coarse probe reads. */
                    VkImageView skip_view() const noexcept { return skip_.view; }

                    /** @brief The far window: r = density, g = profile, b = optical depth to the sun. */
                    VkImageView far_view() const noexcept { return far_.view; }

                    /** @brief The linear, edge-clamped sampler every window is read under. */
                    VkSampler sampler() const noexcept { return sampler_; }

                    /**
                     * @brief The near window's world span, metres.
                     *
                     * What the light volume and the cloud shadow map convert a world step into a
                     * UV step with; both are baked over exactly this window.
                     */
                    static float near_span_meters() noexcept { return NEAR_SPAN_METERS; }

                    /**
                     * @brief Whether the near window is being re-placed this frame.
                     *
                     * The light volume and the cloud shadow map are amortized across eight
                     * frames, which is right while they refresh in place under a drifting sun —
                     * but the moment the window itself moves, the slices already baked describe a
                     * different piece of the world than the ones still to come, and their
                     * consumers address all of them through the *new* mapping. So both do a full
                     * refresh on the frame this returns true, instead of banding for an eighth of
                     * a second after every re-centring. Valid only after @ref update_window.
                     */
                    bool near_window_moved() const noexcept { return near_dirty_; }

                private:
                    /** @brief One baked volume: its image, view, and allocation. */
                    struct Volume
                    {
                        VkImage image = VK_NULL_HANDLE;
                        VmaAllocation allocation = VK_NULL_HANDLE;
                        VkImageView view = VK_NULL_HANDLE;
                        std::uint32_t width = 0;
                        std::uint32_t height = 0;
                        std::uint32_t depth = 0;
                    };

                    /** @brief One deck's memcmp-relevant authored state. */
                    struct DeckSnapshot
                    {
                        std::uint32_t enabled = 0;
                        std::uint32_t genus = 0;
                        float coverage_bias = 0.0f;
                        float density_scale = 0.0f;
                    };

                    /**
                     * @brief What a rebake depends on beyond the window's own placement.
                     *
                     * Compared frame to frame. The march shell rides here because the bake maps
                     * its Y axis across exactly that altitude span, so a shell that moved is a
                     * field addressed against the wrong altitudes.
                     */
                    struct Snapshot
                    {
                        DeckSnapshot decks[6]; /**< Mirrors Render::CLOUD_MAX_DECKS. */
                        float weather_scale = 0.0f;
                        float shell_base = 0.0f;
                        float shell_top = 0.0f;
                        std::uint32_t derive_genus = 0;
                    };

                    /** @brief The density bake's push block; see cloudscape_field.comp. */
                    struct Push
                    {
                        float pattern_origin[2];
                        float world_origin[2];
                        float span_meters;
                        float weather_scale;
                        std::uint32_t derive_genus;
                    };

                    /** @brief The far window's sun-depth pass's push block. */
                    struct FarLightPush
                    {
                        float span_meters;
                    };

                    /**
                     * @brief The WMO genus catalogue and its classifier thresholds, as GPU data.
                     *
                     * Uploaded once at construction from `Render::cloud_genus_profile` and
                     * `Render::cloud_genus_thresholds`, because it is a compile-time table and
                     * re-uploading it per frame would say otherwise. Laid out to match the deck
                     * arrays in the scene block, so the bake's deck evaluation is the same code
                     * whether the deck came from an author or from the classifier.
                     */
                    struct GenusCatalogue
                    {
                        float a[10][4]; /**< base altitude, top altitude, coverage, density. */
                        float b[10][4]; /**< stratiform, detail strength, shape scale, detail scale. */
                        float c[10][4]; /**< wind.xyz, noise kind. */
                        float d[10][4]; /**< anvil, spare, spare, spare. */
                        float thresholds[4];      /**< convective, low_broken, middle_overcast, high_sheet. */
                        float thresholds_tail[4]; /**< cb convective, cb coverage, enable coverage, spare. */
                    };

                    /**
                     * @brief Where one window is anchored, and what it absorbed when it was baked.
                     *
                     * `pattern_*` is the scene-absolute XZ of the window's corner *plus* everything
                     * the wind has displaced the sky by, which is the frame the noise is evaluated
                     * in; `wind_*` and `eye_*` are what that included, so the camera-relative
                     * corner the weather lookup needs can be recovered. Doubles because the scene
                     * coordinate is planet-scale and the snap has to land on the same lattice
                     * every time or a rebake would shift the sky by a fraction of a texel.
                     */
                    struct Window
                    {
                        double pattern_origin_x = 0.0;
                        double pattern_origin_z = 0.0;
                        double wind_x = 0.0;
                        double wind_z = 0.0;
                        double eye_x = 0.0;
                        double eye_z = 0.0;
                        float time_seconds = 0.0f;
                        float sun[3] = {0.0f, 1.0f, 0.0f};
                        bool baked = false;
                    };

                    void create_volume(Volume& volume, std::uint32_t width, std::uint32_t height,
                                       std::uint32_t depth);
                    void destroy_volume(Volume& volume);
                    void create_catalogue();
                    void destroy_catalogue();
                    void create_pipelines();
                    void destroy_pipelines();
                    bool cloudscape_changed(const Snapshot& snapshot);
                    void place_window(Window& window, float span, std::uint32_t resolution,
                                      const double eye[3], double wind_x, double wind_z,
                                      float time_seconds, const float sun[3]);
                    static void window_map(const Window& window, float span, const double eye[3],
                                           double wind_x, double wind_z, float map[4]);
                    static void window_push(const Window& window, float span, float weather_scale,
                                            bool derive_genus, Push& push);

                    void record_density(VkCommandBuffer cmd, const Frame::FrameContext& frame,
                                        VkBuffer uniform_buffer, Volume& target, const Push& push);
                    void record_skip(VkCommandBuffer cmd, const Frame::FrameContext& frame);
                    void record_far_light(VkCommandBuffer cmd, const Frame::FrameContext& frame,
                                          VkBuffer uniform_buffer);

                    /**
                     * @brief The near window's world span, metres.
                     *
                     * Deliberately the old tile period: it keeps the near texel at 128 m, so
                     * nothing about how finely the sky is resolved where the camera actually is
                     * changed when the tile became a window.
                     */
                    static constexpr float NEAR_SPAN_METERS = 32768.0f;

                    /**
                     * @brief The far window's world span, metres.
                     *
                     * Eight times the near window, which covers the march's own reach (it is
                     * bounded at fourteen shell thicknesses, ~150 km) with room to spare, at
                     * ~1 km per texel. Past this the sampler clamps, which is honest: that is
                     * beyond anything the march reaches.
                     */
                    static constexpr float FAR_SPAN_METERS = NEAR_SPAN_METERS * 8.0f;

                    // Fixed at construction, like the atmosphere LUTs and the cloud noise
                    // volumes: this is baked infrastructure sized once at renderer setup, not
                    // a per-frame cost the quality tier scales (that's the march step counts).
                    static constexpr std::uint32_t FIELD_RESOLUTION_XZ = 256;
                    static constexpr std::uint32_t FIELD_RESOLUTION_Y = 32;
                    static constexpr std::uint32_t SKIP_DOWNSAMPLE_XZ = 4;
                    static constexpr std::uint32_t SKIP_DOWNSAMPLE_Y = 2;

                    /**
                     * @brief How far the camera may drift before a window is re-centred, as a
                     *        fraction of its own span.
                     *
                     * Not one texel: re-centring costs a full rebake, and because the bake is a
                     * pure function of world position, letting the camera sit a little off-centre
                     * changes nothing about what it sees — only how much window is left ahead of
                     * it. At 8 % the near window still keeps 12 km of reach in the direction of
                     * travel while rebaking roughly once every 2.6 km of movement.
                     *
                     * The drift is measured in the *pattern* frame, which carries the wind as well
                     * as the camera. That is deliberate and it is why there is no separate wind
                     * trigger: the wind is expressed purely as a shift of the window's own origin,
                     * so a stationary camera under a blowing sky drifts out of its window at
                     * exactly the wind's rate, and the sky advects with no staleness whatsoever in
                     * between — the lookup slides, the content does not go out of date.
                     */
                    static constexpr double REBAKE_DRIFT_FRACTION = 0.08;

                    /**
                     * @brief Minimum seconds between weather-driven rebakes, per window.
                     *
                     * The published field changes every frame — it is interpolated between the
                     * simulation's own 15 s ticks — so "the weather changed" is true continuously
                     * and cannot gate anything on its own. These are what turn it into a cadence.
                     * One second gives fifteen samples per simulated tick, which is far finer than
                     * a medium whose own time scale is minutes can show; the far window, resolving
                     * eight times coarser, takes a quarter of that rate.
                     */
                    static constexpr float NEAR_WEATHER_INTERVAL_SECONDS = 1.0f;
                    static constexpr float FAR_WEATHER_INTERVAL_SECONDS = 4.0f;

                    /**
                     * @brief How far the sun may move before the far window's light channel is stale.
                     *
                     * Only the far window carries a baked sun direction (the near window's light
                     * lives in CloudLightVolumePass, which refreshes on its own eight-frame
                     * cadence). One degree is well inside what a ~1 km texel can show, and at the
                     * sun's own rate it is about four minutes — so this only ever fires for a
                     * camera that has been standing still under an unchanging sky.
                     */
                    static constexpr float FAR_SUN_COS_TOLERANCE = 0.99985f; // cos(1 degree)

                    Vulkan::VulkanDevice& device_;
                    Resources::ShaderLibrary& shaders_;
                    Resources::GraphicsPipelineFactory& pipelines_;
                    Textures::CloudNoise& noise_;
                    WeatherFieldPass& weather_;

                    Volume near_;
                    Volume skip_;
                    Volume far_source_; /**< The far density bake, read by the sun-depth pass. */
                    Volume far_;        /**< That same density plus its sun depth; what consumers read. */
                    VkSampler sampler_ = VK_NULL_HANDLE;

                    VkBuffer catalogue_ = VK_NULL_HANDLE;
                    VmaAllocation catalogue_allocation_ = VK_NULL_HANDLE;

                    VkDescriptorSetLayout field_layout_ = VK_NULL_HANDLE;
                    VkPipelineLayout field_pipeline_layout_ = VK_NULL_HANDLE;
                    VkPipeline field_pipeline_ = VK_NULL_HANDLE;
                    VkDescriptorSetLayout skip_layout_ = VK_NULL_HANDLE;
                    VkPipelineLayout skip_pipeline_layout_ = VK_NULL_HANDLE;
                    VkPipeline skip_pipeline_ = VK_NULL_HANDLE;
                    VkDescriptorSetLayout far_light_layout_ = VK_NULL_HANDLE;
                    VkPipelineLayout far_light_pipeline_layout_ = VK_NULL_HANDLE;
                    VkPipeline far_light_pipeline_ = VK_NULL_HANDLE;

                    Snapshot last_snapshot_{};
                    Window near_window_{};
                    Window far_window_{};
                    Push near_push_{};
                    Push far_push_{};
                    bool near_dirty_ = false;
                    bool far_dirty_ = false;
                    bool built_ = false;
            };
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
