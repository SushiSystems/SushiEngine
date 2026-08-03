/**************************************************************************/
/* vulkan_scene_view.cpp                                                  */
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

#include "vulkan_scene_view.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "frame/temporal_jitter.hpp"
#include "geometry/mesh_registry.hpp"
#include "material/asset_library.hpp"
#include "gi/sdf_clipmap.hpp"
#include "material/material_system.hpp"
#include "resources/descriptor_heap.hpp"
#include "resources/sampler_cache.hpp"
#include "scene/post_process_uniforms.hpp"
#include "scene/scene_uniforms.hpp"
#include "scene/shadow_uniforms.hpp"
#include "scene/temporal_uniforms.hpp"
#include "terrain/terrain_frame.hpp"
#include "vulkan_check.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Vulkan
        {
            namespace
            {
                /**
                 * @brief Upper bound on timed passes in one frame.
                 *
                 * Sized to cover the whole graph, not a sample of it. At 16 the profiler ran
                 * out of timers part-way down the pass list and silently stopped measuring —
                 * `GPUProfiler::begin_pass` returns INVALID_TIMER once the budget is spent, so
                 * the overflow read as *absence*: at Ultra, where more tier-gated passes are
                 * enabled ahead of them, `sky` and `clouds` simply vanished from the panel and
                 * looked like passes that were not running.
                 *
                 * That is not only a reporting problem. `measured_frame_milliseconds()` is the
                 * sum of these timings, and it drives the auto-exposure adaptation rate and the
                 * dynamic-resolution governor — both were being steered by a number that
                 * omitted most of the frame. The list registers ~44 pass objects and the
                 * multi-node passes (gtao, ray shadows) contribute two or three each, so 64
                 * leaves headroom above the true worst case rather than sitting on it.
                 */
                constexpr std::uint32_t MAX_TIMED_PASSES = 64;
            } // namespace

            VulkanSceneView::VulkanSceneView(VulkanDevice& device,
                                             Assets::AssetLibrary& assets)
                : device_(device), assets_(assets), descriptors_(device, SLOTS),
                  deformable_(device, SLOTS), materials_(device, assets.textures(), SLOTS),
                  motion_(device, SLOTS), ui_geometry_(device, SLOTS),
                  instance_system_(device, SLOTS),
                  skinning_(device, SLOTS), particles_(device, SLOTS, 1u << 16),
                  lights_(device, SLOTS),
                  accelerator_(device, assets.meshes(), SLOTS),
                  terrain_layout_(device, assets.layout(), assets.heap()),
                  terrain_(device, Terrain::PlanetTerrainDescription{}),
                  profiler_(device, SLOTS, MAX_TIMED_PASSES),
                  graph_(device, &profiler_),
                  atmosphere_lut_pass_(device, assets.shaders(), assets.pipelines()),
                  volumetric_fog_pass_(device, assets.shaders(), assets.pipelines(),
                                       atmosphere_lut_pass_, lights_),
                  weather_field_pass_(device, assets.samplers()),
                  cloudscape_compile_pass_(device, assets.shaders(), assets.pipelines(),
                                           assets.samplers(), assets.cloud_noise(),
                                           weather_field_pass_),
                  cloud_light_volume_pass_(device, assets.shaders(), assets.pipelines(),
                                           assets.samplers(), cloudscape_compile_pass_),
                  cloud_shadow_map_pass_(device, assets.shaders(), assets.pipelines(),
                                        assets.samplers(), cloudscape_compile_pass_),
                  cloud_panorama_pass_(device, assets.shaders(), assets.pipelines(),
                                      assets.samplers(), cloudscape_compile_pass_,
                                      cloud_light_volume_pass_),
                  ibl_pass_(device, assets.shaders(), assets.pipelines(), assets.samplers(),
                            assets.layout(), assets.cloud_noise(), atmosphere_lut_pass_,
                            volumetric_fog_pass_),
                  irradiance_volume_pass_(device, assets.shaders(), assets.pipelines(), ibl_pass_,
                                          assets.meshes()),
                  depth_prepass_(device, assets.shaders(), assets.pipelines(), assets.layout(),
                                 assets.meshes(), motion_, instance_system_),
                  shadow_pass_(device, assets.shaders(), assets.pipelines(), assets.layout(),
                               assets.meshes()),
                  contact_shadow_pass_(device, assets.shaders(), assets.pipelines(),
                                       assets.layout()),
                  ray_shadow_pass_(device, assets.shaders(), assets.pipelines(), accelerator_),
                  gtao_pass_(device, assets.shaders(), assets.pipelines(), assets.layout()),
                  hiz_pass_(device, assets.shaders(), assets.pipelines()),
                  occlusion_pass_(device, assets.shaders(), assets.pipelines()),
                  cull_pass_(device, assets.shaders(), assets.pipelines(), occlusion_pass_,
                             instance_system_),
                  deformable_pass_(device, assets.shaders(), assets.pipelines(), deformable_),
                  skinning_pass_(device, assets.shaders(), assets.pipelines(), skinning_,
                                 assets.meshes()),
                  particle_sim_pass_(device, assets.shaders(), assets.pipelines(), particles_,
                                     hiz_pass_, irradiance_volume_pass_),
                  particle_sort_pass_(device, assets.shaders(), assets.pipelines(), particles_),
                  opaque_pass_(device, assets.shaders(), assets.pipelines(), assets.layout(),
                               assets.meshes(), deformable_, materials_, motion_,
                               cloud_shadow_map_pass_, ibl_pass_, irradiance_volume_pass_, lights_,
                               instance_system_, skinning_),
                  terrain_pass_(device, assets.shaders(), assets.pipelines(), terrain_layout_,
                                terrain_, ibl_pass_, cloud_shadow_map_pass_,
                                irradiance_volume_pass_, materials_, motion_, lights_),
                  transparent_pass_(device, assets.shaders(), assets.pipelines(), assets.layout(),
                                    assets.meshes(), materials_, motion_, skinning_,
                                    cloud_shadow_map_pass_, ibl_pass_, irradiance_volume_pass_,
                                    lights_),
                  light_cull_pass_(device, assets.shaders(), assets.pipelines(), lights_),
                  light_shadow_pass_(device, assets.shaders(), assets.pipelines(), assets.layout(),
                                     assets.meshes(), lights_),
                  shading_rate_pass_(device, assets.shaders(), assets.pipelines()),
                  sky_pass_(device, assets.shaders(), assets.pipelines(), assets.layout(),
                            cloud_shadow_map_pass_, atmosphere_lut_pass_, volumetric_fog_pass_,
                            lights_),
                  ground_shadow_resolve_pass_(device, assets.shaders(), assets.pipelines(),
                                              assets.layout()),
                  cloud_pass_(device, assets.shaders(), assets.pipelines(), assets.layout(),
                              cloudscape_compile_pass_, cloud_light_volume_pass_,
                              assets.cloud_noise(), atmosphere_lut_pass_),
                  // 16u, 16u: the same construction-time placeholder resources_ below
                  // uses — width_/height_ are declared later in the class and are not
                  // yet initialized this early in the member-init list, so the real
                  // output extent reaches this pass through its resize(), called
                  // alongside resources_.resize() from VulkanSceneView::resize().
                  cloud_taa_pass_(device, assets.shaders(), assets.pipelines(), assets.samplers(),
                                 16u, 16u),
                  cloud_composite_pass_(device, assets.shaders(), assets.pipelines(),
                                        assets.layout(), cloud_taa_pass_, atmosphere_lut_pass_),
                  ssr_pass_(device, assets.shaders(), assets.pipelines(), assets.layout(),
                            hiz_pass_),
                  particle_pass_(device, assets.shaders(), assets.pipelines(), particles_, lights_,
                                 ibl_pass_, assets.heap()),
                  particle_mesh_pass_(device, assets.shaders(), assets.pipelines(), particles_,
                                      assets.meshes()),
                  taa_pass_(device, assets.shaders(), assets.pipelines(), assets.layout()),
                  dof_pass_(device, assets.shaders(), assets.pipelines(), assets.layout()),
                  motion_blur_pass_(device, assets.shaders(), assets.pipelines(), assets.layout()),
                  auto_exposure_pass_(device, assets.shaders(), assets.pipelines()),
                  bloom_pass_(device, assets.shaders(), assets.pipelines()),
                  grid_pass_(device, assets.shaders(), assets.pipelines(), assets.layout()),
                  tonemap_pass_(device, assets.shaders(), assets.pipelines(), assets.layout()),
                  fxaa_pass_(device, assets.shaders(), assets.pipelines(), assets.layout()),
                  ui_pass_(device, assets.shaders(), assets.pipelines(), ui_geometry_, font_,
                           assets.textures(), assets.heap()),
                  resources_(device, assets.samplers().get(Resources::SamplerDescription{}), 16u,
                             16u)
            {
                // Bake the overlay font once, here, because it is a device upload and the
                // overlay pass must never rasterize a glyph mid-frame. A failure is not fatal:
                // the atlas stays invalid, the pass falls back to the opaque-white default, and
                // the UI draws its panels without its labels.
                font_.build_from_system_font(assets.textures(), UI_FONT_PIXEL_HEIGHT);

                // Registration order is execution order. The IBL chain is captured before
                // anything samples it; the depth prepass runs first among the geometry so
                // the contact-shadow march has a complete depth buffer to walk, and the
                // cascades are drawn before the surfaces that sample them; the shading
                // rate mask sits after the geometry pass
                // because it reads that pass's motion vectors and before the sky because
                // the sky is what it steers; the temporal resolve needs a complete scene,
                // so it follows the cloud composite; and the display transform is last
                // except for the spatial filter and the picking readback.
                passes_ = {&atmosphere_lut_pass_,
                           // The cloud bake trio runs this early so every consumer this
                           // frame — opaque/transparent mesh shading, the sky pass, and
                           // CloudPass's own march later on — reads this frame's bake
                           // rather than lagging a frame behind it. Each is itself
                           // change-gated or amortized, so running early costs nothing on
                           // the (common) frame where there is nothing new to bake.
                           // Before the bakes that read it: the field upload is the one
                           // thing this frame's cloud tier depends on that comes from
                           // outside the renderer.
                           &weather_field_pass_,
                           &cloudscape_compile_pass_,
                           &cloud_light_volume_pass_,
                           &cloud_shadow_map_pass_,
                           &cloud_panorama_pass_,
                           &ibl_pass_,
                           &irradiance_volume_pass_,
                           &cull_pass_,
                           &skinning_pass_,
                           &particle_sim_pass_,
                           &particle_sort_pass_,
                           &depth_prepass_,
                           &occlusion_pass_,
                           &shadow_pass_,
                           &contact_shadow_pass_,
                           &ray_shadow_pass_,
                           &light_cull_pass_,
                           // The punctual in-scatter march reads the cluster grid/index
                           // list the cull pass just wrote this frame, so it must follow
                           // it; runs before the shadow pass since it doesn't sample the
                           // punctual atlas (unshadowed in-scatter, see the pass header).
                           &volumetric_fog_pass_,
                           &light_shadow_pass_,
                           &gtao_pass_,
                           &hiz_pass_,
                           &deformable_pass_,
                           &opaque_pass_,
                           // The body's ground, after the meshes standing on it: it covers
                           // the whole screen, so letting their finished depth reject it
                           // costs one pass of ordering and saves the shading.
                           &terrain_pass_,
                           // Mesh particles are solid geometry, so they belong with the opaque
                           // surfaces and before anything that reads a finished depth buffer.
                           &particle_mesh_pass_,
                           // Transparent surfaces composite over the finished opaque depth;
                           // they test against it but never write it, so they must follow
                           // every pass that still contributes to that depth buffer.
                           &transparent_pass_,
                           &shading_rate_pass_,
                           &sky_pass_,
                           &ground_shadow_resolve_pass_,
                           &cloud_pass_,
                           &cloud_taa_pass_,
                           &cloud_composite_pass_,
                           &ssr_pass_,
                           &particle_pass_,
                           &taa_pass_,
                           &dof_pass_,
                           &motion_blur_pass_,
                           &auto_exposure_pass_,
                           &bloom_pass_,
                           &grid_pass_,
                           &tonemap_pass_,
                           &fxaa_pass_,
                           // The game's own UI, last of the colour passes: composited onto the
                           // finished image so it is neither tone mapped nor anti-aliased.
                           &ui_pass_,
                           &picking_pass_};
            }

            VulkanSceneView::~VulkanSceneView()
            {
                // Idle before anything is torn down; ViewResources then releases its
                // handles as a member without a further wait.
                vkDeviceWaitIdle(device_.device());
            }

            void VulkanSceneView::resize(std::uint32_t width, std::uint32_t height)
            {
                const std::uint32_t new_width = width < 1 ? 1 : width;
                const std::uint32_t new_height = height < 1 ? 1 : height;
                if (new_width == width_ && new_height == height_)
                    return;
                width_ = new_width;
                height_ = new_height;
                resources_.resize(new_width, new_height);
                // The cloud buffer's own dedicated history is sized off the output
                // extent too (see CloudTAAPass's header comment for why it stays fixed
                // across dynamic resolution and tier changes otherwise); resize() is a
                // no-op when the derived half-extent hasn't actually changed.
                cloud_taa_pass_.resize(new_width, new_height);
                // Nothing accumulated into the new images yet, and the dynamic-resolution
                // governor hasn't rescaled for the new extent — report the full output size
                // until update_render_extent() runs at the top of the next render().
                render_width_ = width_;
                render_height_ = height_;
            }

            void VulkanSceneView::set_settings(const RenderSettings& settings)
            {
                // Switching the anti-aliasing mode invalidates what has been accumulated:
                // the history was produced by a different resolve, and blending the two
                // would show as a lingering ghost of the old mode.
                if (settings.anti_aliasing != settings_.anti_aliasing)
                    resources_.set_history_valid(false);
                if (!settings.dynamic_resolution.enabled &&
                    settings_.dynamic_resolution.enabled)
                    resolution_.reset();
                settings_ = settings;
            }

            void VulkanSceneView::render_resolution(std::uint32_t& width,
                                                    std::uint32_t& height) const noexcept
            {
                width = render_width_;
                height = render_height_;
            }

            float VulkanSceneView::measured_frame_milliseconds() const noexcept
            {
                float total = 0.0f;
                for (const Graph::PassTiming& timing : profiler_.timings())
                    total += timing.milliseconds;
                return total;
            }

            void VulkanSceneView::update_render_extent()
            {
                const float governed =
                    resolution_.update(settings_.dynamic_resolution,
                                       measured_frame_milliseconds());
                // The manual scale is the ceiling the governor works under, so a host that
                // has already chosen to render small is never scaled back up past it.
                const float scale = governed * (settings_.render_scale < 1.0f
                                                    ? settings_.render_scale
                                                    : 1.0f);
                const std::uint32_t width = Frame::scaled_extent(width_, scale);
                const std::uint32_t height = Frame::scaled_extent(height_, scale);
                if (width == render_width_ && height == render_height_)
                    return;
                render_width_ = width;
                render_height_ = height;
            }

            void VulkanSceneView::update_frame_slots(std::uint32_t requested)
            {
                const std::uint32_t slots =
                    requested < 2u ? 2u : (requested > SLOTS ? SLOTS : requested);
                if (slots == frame_slots_)
                    return;
                // Changing the depth changes which slot a frame index maps to, so the frames
                // already in flight have to land first: their resources would otherwise be
                // reused by a frame the old mapping never expected to collide with.
                vkDeviceWaitIdle(device_.device());
                frame_slots_ = slots;
            }

            void VulkanSceneView::update_terrain_body(int body)
            {
                if (!terrain_layout_.available() || body == terrain_.body())
                    return;
                // Same reasoning as above: the frames in flight hold slot bindings, and
                // re-pointing the pool at another world invalidates every one of them.
                // A body change is a scene-level event, so the stall is paid once.
                vkDeviceWaitIdle(device_.device());
                terrain_.set_body(body, Terrain::default_pack_path(body));
            }

            SceneViewTexture VulkanSceneView::texture(std::uint32_t slot) const noexcept
            {
                return resources_.texture(slot);
            }

            std::size_t VulkanSceneView::pass_timing_count() const noexcept
            {
                return profiler_.timings().size();
            }

            ScenePassTiming VulkanSceneView::pass_timing(std::size_t index) const noexcept
            {
                const std::vector<Graph::PassTiming>& timings = profiler_.timings();
                if (index >= timings.size())
                    return ScenePassTiming{};
                ScenePassTiming timing;
                timing.name = timings[index].name.c_str();
                timing.milliseconds = timings[index].milliseconds;
                return timing;
            }

            void VulkanSceneView::cull_statistics(std::uint32_t& drawn,
                                                  std::uint32_t& tested) const noexcept
            {
                // The current slot's readback holds the counts its submit wrote and this
                // frame's fence wait made safe to read.
                const Passes::CullStatistics stats = cull_pass_.statistics(current_slot_);
                drawn = stats.drawn;
                tested = stats.tested;
            }

            void VulkanSceneView::render(const CameraView& camera, const Environment& environment,
                                         const MeshInstance* instances, std::size_t count,
                                         std::uint32_t selected_id,
                                         const DeformableMeshView* deformable,
                                         std::size_t deformable_count,
                                         const PunctualLight* lights, std::size_t light_count,
                                         const Decal* decals, std::size_t decal_count,
                                         bool show_grid, const SkinnedInstance* skinned,
                                         std::size_t skinned_count,
                                         const ParticleEmitterView* emitters,
                                         std::size_t emitter_count,
                                         const ParticleBillboard* billboards,
                                         std::size_t billboard_count, const UIView* ui)
            {
                // Resolve the quality tier once, here, into the effective settings every
                // pass reads and the extra per-pass knobs that have no home in
                // RenderSettings. The authored settings_ stays the untouched baseline the
                // tier scales from, so the next frame resolves from the same request. It is
                // resolved before the slot is picked because how deep the frame chain runs
                // is one of the things it decides.
                const ResolvedQuality resolved = resolve_quality(settings_);
                const RenderSettings& effective = resolved.settings;

                update_frame_slots(effective.delivery.frames_in_flight);

                const std::uint32_t index = frame_counter_ % frame_slots_;

                resources_.wait_for_slot(index);
                // The slot's submissions have completed on both queues, so its timestamps
                // are readable and its transient resources are free.
                profiler_.resolve(index);
                // Texture streaming and shader reload are device-level, so the asset
                // library owns them; it reports back when the pipelines this view built
                // have been invalidated.
                if (assets_.update())
                    for (Passes::IRenderPass* pass : passes_)
                        pass->rebuild_pipelines();

                // The governor reads the timings that were just resolved, so the extent it
                // picks is a response to a completed frame rather than a guess.
                update_render_extent();

                resources_.reset_commands(index);
                descriptors_.begin_frame(index);
                resources_.begin_slot(index);

                Frame::FrameContext frame;
                frame.index = frame_counter_;
                frame.slot = index;
                frame.width = render_width_;
                frame.height = render_height_;
                frame.output_width = width_;
                frame.output_height = height_;
                frame.settings = effective;
                frame.quality = resolved.parameters;
                frame.camera = &camera;
                frame.environment = &environment;
                Scene::camera_eye(camera.view, frame.eye);
                frame.draws.instances = instances;
                frame.draws.instance_count = count;
                frame.draws.deformable = deformable;
                frame.draws.deformable_count = deformable_count;
                frame.draws.skinned = skinned;
                frame.draws.skinned_count = skinned_count;
                frame.draws.emitters = emitters;
                frame.draws.emitter_count = emitter_count;
                frame.draws.billboards = billboards;
                frame.draws.billboard_count = billboard_count;
                // Only a non-empty overlay reaches the pass: a null or degenerate one is the
                // normal case for a viewport that draws no game UI at all.
                frame.draws.ui = (ui != nullptr && !ui->empty()) ? ui : nullptr;
                frame.particle_capacity = emitter_count > 0 ? particles_.capacity() : 0;
                frame.draws.lights = lights;
                frame.draws.light_count = light_count;
                frame.draws.decals = decals;
                frame.draws.decal_count = decal_count;
                frame.draws.selected_id = selected_id;
                frame.grid.enabled = show_grid;
                frame.descriptors = &descriptors_;
                frame.samplers = &assets_.samplers();
                frame.layout = &assets_.layout();
                frame.history_valid = resources_.history_valid() && frame.temporal_enabled();

                // The jitter cycle is longer when resolving into a larger grid, because
                // more distinct sub-pixel positions are needed to fill it.
                const bool upscaling = render_width_ < width_ || render_height_ < height_;
                const std::uint32_t phases =
                    Frame::JITTER_PHASE_COUNT * (upscaling ? 2u : 1u);
                if (frame.temporal_enabled())
                    Frame::frame_jitter(frame_counter_, render_width_, render_height_,
                                        settings_.temporal.jitter_scale, phases, frame.jitter);

                // Secondary directional casters need their atlas tile before the scene
                // block is filled, so its shadow-index lanes can be patched in below;
                // begin_frame() also resets the light system's other per-frame buffers,
                // which the packing calls further down fill in.
                lights_.begin_frame(index);
                lights_.assign_directional_shadows(environment.lights, environment.light_count,
                                                   effective.lights.shadow_atlas_size,
                                                   effective.shadows.max_directional_shadow_casters,
                                                   effective.shadows.distance);

                // The one regional atmosphere. Only *latched* here, and stepped at the window
                // frame's end (see AssetLibrary::stage_atmosphere): the step is a ~13 ms
                // dispatch chain, so running it in front of the passes that read it put a whole
                // step of weather inside the frame's critical path. This frame's passes
                // therefore read the step submitted at the end of the previous one.
                // The grid arrives on the environment, not the resolved render quality:
                // the nest is device-global and its size is a simulation budget the host
                // resolves from its own atmosphere tier (sim/simulation_settings.hpp),
                // so the render tier cannot rebuild the weather and two views cannot
                // disagree about the one nest's grid.
                assets_.stage_atmosphere(environment.atmosphere_nest,
                                         environment.atmosphere_forcing,
                                         environment.atmosphere_nest_size);

                // The near-field body's real ground. Selected here, ahead of the scene
                // block, because whether it drew is what decides whether the analytic
                // ellipsoid below should draw at all — and the answer is only known once
                // the selection has run.
                update_terrain_body(environment.planet_body);
                if (terrain_.loaded())
                {
                    SushiEngine::Terrain::FrustumPlanes terrain_frustum;
                    Terrain::TerrainFrameView terrain_view;
                    Terrain::build_terrain_frame(camera, environment, frame.eye, render_height_,
                                                 frame_counter_, index, terrain_frustum,
                                                 terrain_view);
                    terrain_.prepare(terrain_view);
                }

                Scene::SceneUniforms uniforms;
                Scene::fill_scene_uniforms(camera, environment, frame.eye,
                                           static_cast<float>(frame_counter_) * 0.016f, uniforms);
                if (terrain_.drawing())
                    Scene::suppress_analytic_ground(uniforms);
                // The cloudscape windows are placed here, between the fill and the upload,
                // because the bake and every consumer of the bake — the view march, the light
                // volume, the cloud shadow map, the panorama, and the ground/mesh shadow lookup —
                // read the same mapping out of this block, and the pass that owns the window
                // state cannot reach it once it has been uploaded. The pass keeps the state; the
                // view only owns the ordering.
                cloudscape_compile_pass_.update_window(frame, environment, assets_.atmosphere(),
                                                       uniforms);
                for (int i = 0; i < 4; ++i)
                    uniforms.light_shadow_a[i] =
                        static_cast<float>(lights_.directional_shadow_index(static_cast<std::uint32_t>(i)));
                uniforms.light_shadow_b[0] = static_cast<float>(lights_.directional_shadow_index(4));
                // Weather-driven ground wetness (design doc §5.3, W5); see scene_uniforms.hpp's
                // field doc for why this lane and not a new UBO member.
                uniforms.light_shadow_b[1] = environment.weather.ground_wetness;
                uniforms.light_shadow_b[2] = 0.0f;
                uniforms.light_shadow_b[3] = 0.0f;
                // The image-based lighting chain is this view's, so its parameters are
                // stamped in here rather than by the environment fill, which knows
                // nothing about the renderer's internals.
                uniforms.ibl_parameters[0] = environment.ibl_intensity;
                uniforms.ibl_parameters[1] = static_cast<float>(ibl_pass_.specular_mip_count());
                uniforms.ibl_parameters[2] = environment.image_based_lighting ? 1.0f : 0.0f;
                // The main sky pass reads the background from the sky-view LUT; the IBL
                // cube capture clears this so it keeps the per-pixel march (see IBLPass).
                uniforms.ibl_parameters[3] = 1.0f;
                // The jitter enters the projection here and nowhere else, so every pass
                // that transforms by it inherits the offset and no world-space value
                // moves. It is subtracted, not added: the third column is scaled by view
                // z and the perspective divide is by -z, so a positive entry shifts the
                // result negative. The sign matters — it is what the sky's ray offset and
                // the motion vector's jitter removal are both matched against.
                uniforms.proj[8] -= frame.jitter[0];
                uniforms.proj[9] -= frame.jitter[1];
                resources_.upload_scene(index, uniforms);

                // The cascades are fitted before the targets are declared, because the
                // atlas the graph allocates has to be sized to the fit.
                //
                // They follow light 0, not the Sun. The ephemeris orders the celestial
                // lights by what each actually delivers at the camera, so light 0 is the
                // Sun by day and whichever body dominates after it sets — which is what
                // gives a full moon real cast shadows instead of a flat ambient lift. The
                // single atlas goes to that light because every other one is, by the same
                // ordering, too faint to resolve a shadow edge from.
                const Vector3 key_light = environment.light_count > 0
                                              ? environment.lights[0].direction
                                              : environment.sun.direction;
                Scene::ShadowUniforms shadow_uniforms;
                frame.cascade_count = Scene::fit_shadow_cascades(
                    camera, frame.eye, key_light, effective.shadows, shadow_uniforms);
                frame.shadow_resolution =
                    frame.cascade_count > 0 ? (effective.shadows.resolution < 64u
                                                   ? 64u
                                                   : effective.shadows.resolution)
                                            : 1u;
                // Stamped after the fit rather than inside it: whether the frame traces
                // depends on the device and on this view's passes, which the fit knows
                // nothing about, and the flag the material shader reads has to describe
                // what actually ran.
                shadow_uniforms.flags[2] = ray_shadow_pass_.enabled(frame) ? 1.0f : 0.0f;
                // The soft-shadow tap counts have no field of their own in the block; they
                // ride the two spare lanes the fit leaves untouched (parameters.z, filter.w),
                // so the tier can cheapen the penumbra filter with no struct change and no
                // extra plumbing — the sampler reads them straight back.
                shadow_uniforms.parameters[2] =
                    static_cast<float>(resolved.parameters.shadow_filter_taps);
                shadow_uniforms.filter[3] =
                    static_cast<float>(resolved.parameters.shadow_blocker_taps);
                resources_.upload_shadow(index, shadow_uniforms);

                // The eye's own frame-to-frame travel, formed in double before the narrowing
                // cast: the block's matrix is translation-free (camera-relative rendering), so
                // this is the only way a ray-marched reprojection can see the camera move.
                double previous_temporal_eye[3];
                Scene::camera_eye(previous_camera_.view, previous_temporal_eye);
                const float temporal_eye_delta[3] = {
                    static_cast<float>(frame.eye[0] - previous_temporal_eye[0]),
                    static_cast<float>(frame.eye[1] - previous_temporal_eye[1]),
                    static_cast<float>(frame.eye[2] - previous_temporal_eye[2])};
                Scene::TemporalUniforms temporal;
                Scene::fill_temporal_uniforms(effective, previous_camera_.view,
                                              previous_camera_.projection, frame.jitter,
                                              previous_jitter_, render_width_, render_height_,
                                              width_, height_, frame.history_valid,
                                              temporal_eye_delta, temporal);
                resources_.upload_temporal(index, temporal);

                // The exposure the display transform applies. Manual exposure multiplies the
                // scene-authored value by the user's EV compensation — reproducing the prior
                // fixed behaviour when the compensation is zero; automatic exposure uses the
                // value the auto-exposure pass adapted from a past frame's luminance, read back
                // and eased in update_auto_exposure() at the top of the next frame.
                float linear_exposure =
                    environment.exposure * std::exp2(effective.post.exposure_compensation);
                if (effective.post.exposure_mode == ExposureMode::Automatic)
                {
                    // Adapt the stored exposure from the histogram this slot built two frames
                    // ago (its readback is safe past the fence waited on above). A wild frame
                    // time is treated as a nominal step so a hitch cannot jerk the exposure.
                    float delta = measured_frame_milliseconds() * 0.001f;
                    if (delta <= 0.0f || delta > 0.5f)
                        delta = 0.016f;
                    adapted_exposure_ = auto_exposure_pass_.adapt(
                        index, effective.post.auto_exposure, delta, adapted_exposure_);
                    linear_exposure = adapted_exposure_;
                }
                const bool bloom_active = effective.post.bloom.enabled && resolved.parameters.bloom;
                // The display transform and depth of field run at the post-resolve extent — the
                // output grid when the temporal resolve accumulated into it, the render extent
                // otherwise — so their pixel-space maths (chromatic aberration, the DoF texel
                // step) take that resolution, not the internal render one.
                const bool resolves_temporally =
                    effective.anti_aliasing == AntiAliasingMode::Temporal;
                const std::uint32_t post_w = resolves_temporally ? width_ : render_width_;
                const std::uint32_t post_h = resolves_temporally ? height_ : render_height_;
                Scene::PostProcessUniforms post_uniforms;
                Scene::fill_post_process_uniforms(effective.post, linear_exposure, bloom_active,
                                                  frame_counter_, post_w, post_h, post_uniforms);
                // The camera near plane the depth-of-field pass linearises reverse-Z depth with;
                // stamped here rather than by the pack, which knows nothing about the camera.
                post_uniforms.misc[2] = camera.near_plane;
                resources_.upload_post(index, post_uniforms);

                materials_.begin_frame(index);
                motion_.begin_frame(index, frame.eye);
                // Pack and configure the light engine before the passes register: the cull
                // pass reads the packed count and the grid's near/far when it builds its
                // push constant, and the far distance is the tier-scaled cluster reach.
                // (begin_frame() and assign_directional_shadows() already ran above, before
                // the scene block was filled, so its shadow-index lanes could be patched in.)
                lights_.pack(lights, light_count, frame.eye, effective.lights.max_lights);
                lights_.pack_decals(decals, decal_count, frame.eye, effective.lights.max_decals,
                                    assets_.textures());
                lights_.set_config(render_width_, render_height_, camera.near_plane,
                                   effective.lights.cluster_far_distance);
                // Assign spot casters their atlas tiles and build their maps; this patches
                // the packed lights' shadow index, so it must run before the upload below.
                lights_.assign_shadows(lights, light_count, frame.eye,
                                       effective.lights.shadow_atlas_size,
                                       effective.lights.max_shadow_casters, camera.near_plane);

                // Stochastic shadows for the lights the atlas had no tile for.
                // The GI tracer's distance field is the thing a shadow ray marches, so this
                // path exists only where that field is live. It is registered in the bindless
                // heap once — the field's image is created with the device and never
                // reallocated — because the per-frame push set is full and a volume needs a
                // home every pipeline can reach.
                std::uint32_t samples = 0;
                GI::SDFClipmapConfiguration field_config{};
                if (resolved.parameters.stochastic_light_samples > 0 &&
                    effective.lights.stochastic_shadows && irradiance_volume_pass_.field_live())
                {
                    if (visibility_field_slot_ == Resources::INVALID_HEAP_INDEX)
                    {
                        const GI::VisibilityField field =
                            irradiance_volume_pass_.visibility_field(frame_counter_);
                        if (field.valid())
                            visibility_field_slot_ = assets_.heap().allocate_volume(
                                field.distance_field,
                                assets_.samplers().get(Resources::SamplerDescription{}));
                    }
                    if (visibility_field_slot_ != Resources::INVALID_HEAP_INDEX)
                    {
                        samples = resolved.parameters.stochastic_light_samples;
                        // Derived from the eye by the same pure function the tracer's own
                        // populate calls, so the block the shading pass marches with and the
                        // field it marches can never describe different cubes.
                        GI::configure_sdf_clipmap(frame.eye, 0, field_config);
                    }
                }
                lights_.set_stochastic_shadows(samples, effective.lights.stochastic_distance,
                                               effective.lights.stochastic_softness,
                                               visibility_field_slot_ ==
                                                       Resources::INVALID_HEAP_INDEX
                                                   ? 0u
                                                   : visibility_field_slot_,
                                               field_config.origin_voxel,
                                               field_config.resolution);

                // GPU-driven geometry: pack this frame's instances for the cull pass.
                // The path is permitted by the tier and the author, and needs the bindless
                // heap (its instance set rides set 2). A selection falls back to the classic
                // per-instance draw so the outline's stencil mask still works, but the cull
                // machinery is still primed so the pyramid stays fresh for when it resumes.
                const bool gpu_capable =
                    resolved.parameters.gpu_driven && settings_.gpu_culling.enabled &&
                    assets_.layout().gpu_pipeline_layout() != VK_NULL_HANDLE;
                // The meshlet path (Ultra + mesh shaders) draws instances itself, so when it is
                // active the GPU-driven cull is not built — the two are alternative geometry
                // paths, and meshlets take priority. A selection falls back to neither.
                const bool meshlet_active =
                    resolved.parameters.meshlets && device_.supports_mesh_shader() &&
                    assets_.layout().meshlet_pipeline_layout() != VK_NULL_HANDLE &&
                    selected_id == NO_PICK;
                const bool use_gpu = gpu_capable && !meshlet_active && selected_id == NO_PICK;
                if (gpu_capable)
                    occlusion_pass_.ensure_extent(render_width_, render_height_);

                // The previous frame's camera-relative view-projection and the eye shift since,
                // for the cull's occlusion reprojection onto last frame's depth pyramid. The
                // view's translation is zeroed to make it camera-relative, matching how the
                // geometry that built the pyramid was uploaded.
                Matrix4 previous_view = previous_camera_.view;
                previous_view.m[12] = 0.0;
                previous_view.m[13] = 0.0;
                previous_view.m[14] = 0.0;
                const Matrix4 previous_view_projection =
                    mul(previous_camera_.projection, previous_view);
                for (int i = 0; i < 16; ++i)
                    frame.previous_view_projection[i] =
                        static_cast<float>(previous_view_projection.m[i]);
                double previous_eye[3];
                Scene::camera_eye(previous_camera_.view, previous_eye);
                frame.eye_delta[0] = static_cast<float>(frame.eye[0] - previous_eye[0]);
                frame.eye_delta[1] = static_cast<float>(frame.eye[1] - previous_eye[1]);
                frame.eye_delta[2] = static_cast<float>(frame.eye[2] - previous_eye[2]);
                frame.occlusion_near = previous_camera_.near_plane;

                if (use_gpu)
                {
                    // The advanced lobes the tier permits, applied before any material is
                    // packed — the same gate the opaque pass applies for the classic path.
                    std::uint32_t allowed_lobes = 0;
                    if (resolved.parameters.lobe_anisotropy)
                        allowed_lobes |= Assets::MATERIAL_ANISOTROPY;
                    if (resolved.parameters.lobe_clearcoat)
                        allowed_lobes |= Assets::MATERIAL_CLEARCOAT;
                    if (resolved.parameters.lobe_sheen)
                        allowed_lobes |= Assets::MATERIAL_SHEEN;
                    if (resolved.parameters.lobe_transmission)
                        allowed_lobes |= Assets::MATERIAL_TRANSMISSION;
                    materials_.set_allowed_lobes(allowed_lobes);

                    std::vector<std::uint32_t> instance_materials;
                    std::vector<std::uint32_t> instance_motions;
                    instance_materials.reserve(count);
                    instance_motions.reserve(count);
                    for (std::size_t i = 0; i < count; ++i)
                    {
                        const MeshInstance& instance = instances[i];
                        instance_materials.push_back(materials_.push(instance.material));
                        const Matrix4 model =
                            instance.mesh != INVALID_MESH
                                ? instance.model
                                : mul(instance.model,
                                      Geometry::shape_scale(instance.kind,
                                                            instance.shape_parameters));
                        instance_motions.push_back(motion_.push(instance.id, model));
                    }
                    instance_system_.build(index, instances, count, frame.eye,
                                           instance_materials.data(), instance_motions.data(),
                                           assets_.meshes());
                    frame.gpu_bucket_count = instance_system_.bucket_count();
                    frame.gpu_instance_count = instance_system_.instance_count();
                }

                // Pack this frame's simulated surfaces and lay out their geometry; the
                // deformable pass shades them on the GPU and the opaque pass draws the result.
                deformable_.prepare(index, deformable, deformable_count, frame.eye);

                // Pack this frame's skinned characters' palettes and lay out their output; the
                // skinning pass deforms them into the output buffer and the opaque pass draws it.
                skinning_.prepare(index, skinned, skinned_count, assets_.meshes());

                // Tessellate this frame's UI overlay into one vertex/index buffer. Done here,
                // beside the other host packs and before the graph is built, because the pass
                // has to know whether there is anything to draw when it registers.
                if (frame.draws.ui != nullptr)
                    ui_geometry_.prepare(index, *frame.draws.ui, font_);
                else
                    ui_geometry_.prepare(index, UIView{}, font_);

                // Flatten this frame's cosmetic emitters and upload their table and LUT atlases;
                // the particle sim pass emits and integrates into the shared pool and the
                // particle pass billboards the result.
                particles_.prepare(index, frame_counter_, emitters, emitter_count,
                                   assets_.meshes(), assets_.textures());
                particles_.prepare_billboards(index, billboards, billboard_count);

                graph_.begin_frame(resources_.textures(index), resources_.buffers(index));
                // Before the passes register, like the graph's own frame reset: the slot a
                // copy is recorded into has to be the slot the caller will later resolve.
                if (capture_)
                    capture_->begin_frame(index);
                // Decided before the passes register, because a pass's queue declaration is
                // collapsed at declaration time: the second queue has to exist on the device,
                // the tier has to permit it, and the author has to have asked for it.
                graph_.set_async_compute_enabled(device_.supports_async_compute() &&
                                                 resolved.parameters.async_compute &&
                                                 effective.delivery.async_compute);
                frame.targets = resources_.declare_targets(
                    graph_, frame, shading_rate_pass_.enabled(frame),
                    shading_rate_pass_.texel_width(), shading_rate_pass_.texel_height());
                for (Passes::IRenderPass* pass : passes_)
                    pass->register_pass(graph_, frame);

                // The editor samples the resolve image between frames, so the frame must
                // end with it in a shader-readable layout. Declaring that as a read is all
                // it takes — the graph derives the transition like any other.
                const Graph::TextureHandle resolve = frame.targets.resolve;
                graph_.add_pass(
                    "present",
                    [resolve](Graph::RenderPassBuilder& builder)
                    {
                        builder.read(resolve, Graph::TextureAccess::SampledFragment);
                        builder.set_side_effect();
                    },
                    Graph::RenderGraph::ExecuteFunction{});
                graph_.compile();
                // Every pass has registered, so the frame's per-object arrays are complete
                // and can be uploaded before any of them records a descriptor write.
                materials_.upload();
                motion_.upload();
                lights_.upload();
                // The GPU-driven instance and bucket buffers the cull pass reads; a no-op when
                // the classic path packed nothing this frame.
                instance_system_.upload();

                // One command buffer per compiled submission, recorded and submitted in
                // index order. With no async pass the graph compiles to a single graphics
                // submission and this is exactly the one buffer it always recorded; with
                // one, the runs alternate between the two queues and the graph's derived
                // wait is what orders them.
                const std::uint32_t submissions = graph_.submission_count();
                std::vector<std::uint64_t> signalled(submissions, 0);
                bool profiler_started = false;
                // Resolved once, before the submission loop: the nest may not exist at all (a
                // scene with weather off never builds one), and only the frame's *first*
                // submission has to wait on it — every later one is already ordered behind that
                // one by the graph's own timeline.
                Atmosphere::AtmosphereNest* nest = assets_.atmosphere();
                const VkSemaphore atmosphere_timeline =
                    nest != nullptr && nest->timeline_value() > 0 ? nest->timeline() : VK_NULL_HANDLE;
                const std::uint64_t atmosphere_value = nest != nullptr ? nest->timeline_value() : 0;
                bool atmosphere_waited = false;
                for (std::uint32_t i = 0; i < submissions; ++i)
                {
                    const Graph::Submission& submission = graph_.submission(i);
                    const bool compute = submission.queue == Graph::PassQueue::AsyncCompute;
                    const VkCommandBuffer buffer =
                        resources_.acquire_command_buffer(index, submission.queue);

                    VkCommandBufferBeginInfo begin{};
                    begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
                    begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
                    check(vkBeginCommandBuffer(buffer, &begin), "vkBeginCommandBuffer");
                    if (!profiler_started)
                    {
                        // The query-pool reset has to precede every timestamp the frame
                        // writes, and the frame's first submission is the only place that is
                        // guaranteed — whichever queue it turned out to be on.
                        profiler_.begin_frame(index, buffer);
                        profiler_started = true;
                    }
                    graph_.execute(buffer, i);
                    check(vkEndCommandBuffer(buffer), "vkEndCommandBuffer");

                    VkCommandBufferSubmitInfo command_submit{};
                    command_submit.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO;
                    command_submit.commandBuffer = buffer;

                    VkSemaphoreSubmitInfo waits[2]{};
                    std::uint32_t wait_count = 0;
                    if (submission.wait != Graph::NO_SUBMISSION)
                    {
                        const Graph::Submission& producer = graph_.submission(submission.wait);
                        VkSemaphoreSubmitInfo& wait = waits[wait_count++];
                        wait.sType = VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO;
                        wait.semaphore = resources_.timeline(producer.queue);
                        wait.value = signalled[submission.wait];
                        // The producer may have ended in any stage; the consuming queue only
                        // has to be held before it begins, and its own barriers narrow the
                        // scope from there.
                        wait.stageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
                    }
                    if (!atmosphere_waited && atmosphere_timeline != VK_NULL_HANDLE)
                    {
                        // The regional atmosphere is a device-level service shared by every
                        // scene view, and its step submits on its own command buffer (see
                        // atmosphere/atmosphere_nest.hpp). This is what makes reading its fields
                        // from a later submission *ordered* rather than merely usually-fine:
                        // without it, views sampling a resource an earlier submission is writing
                        // is a race the validation layers cannot see and a driver is free to
                        // lose.
                        //
                        // The value waited on is the step submitted at the *end of the previous
                        // frame*, because that is where the step now runs. So this is a wait
                        // that has almost always already been satisfied by the time it is
                        // reached — which is the whole point of having moved it: the frame is
                        // ordered behind a step that finished a frame ago instead of stalling on
                        // one that started a moment ago.
                        VkSemaphoreSubmitInfo& wait = waits[wait_count++];
                        wait.sType = VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO;
                        wait.semaphore = atmosphere_timeline;
                        wait.value = atmosphere_value;
                        wait.stageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
                        atmosphere_waited = true;
                    }

                    signalled[i] = resources_.signal_next(index, submission.queue);
                    VkSemaphoreSubmitInfo signal{};
                    signal.sType = VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO;
                    signal.semaphore = resources_.timeline(submission.queue);
                    signal.value = signalled[i];
                    signal.stageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;

                    VkSubmitInfo2 submit{};
                    submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO_2;
                    submit.waitSemaphoreInfoCount = wait_count;
                    submit.pWaitSemaphoreInfos = waits;
                    submit.commandBufferInfoCount = 1;
                    submit.pCommandBufferInfos = &command_submit;
                    submit.signalSemaphoreInfoCount = 1;
                    submit.pSignalSemaphoreInfos = &signal;
                    check(vkQueueSubmit2(compute ? device_.async_compute_queue()
                                                 : device_.graphics_queue(),
                                         1, &submit, VK_NULL_HANDLE),
                          "vkQueueSubmit2");

                    // This view has now read the atmosphere's fields, so the step recorded at
                    // the end of this frame must not overtake it. Registered per submission
                    // rather than once at the end because a graph with an async pass alternates
                    // queues, and both of them may sample the extinction field.
                    if (nest != nullptr)
                        assets_.note_atmosphere_reader(resources_.timeline(submission.queue),
                                                       signalled[i]);
                }

                motion_.end_frame();
                previous_camera_ = camera;
                previous_jitter_[0] = frame.jitter[0];
                previous_jitter_[1] = frame.jitter[1];
                // Only a temporal frame leaves something behind worth blending with; when
                // the resolve did not run, the image at this parity is stale.
                resources_.set_history_valid(frame.temporal_enabled());
                resources_.note_render_extent(index, render_width_, render_height_);
                resources_.mark_rendered(index);
                current_slot_ = index;
                ++frame_counter_;
            }

            std::uint32_t VulkanSceneView::pick(std::uint32_t x, std::uint32_t y)
            {
                return resources_.pick(current_slot_, x, y, width_, height_);
            }

            bool VulkanSceneView::read_output(std::uint32_t slot, FrameImage& image)
            {
                return resources_.read_output(slot, image);
            }

            bool VulkanSceneView::enable_pass_capture(bool enabled)
            {
                if (!enabled)
                {
                    // Detached from the graph before it is destroyed, in that order: the
                    // graph holds a raw pointer and a frame recorded against a freed
                    // capture is the one failure this seam could actually cause.
                    graph_.set_capture(nullptr);
                    if (capture_)
                    {
                        // Its staging buffers may still be the destination of copies a
                        // frame in flight recorded.
                        vkDeviceWaitIdle(device_.device());
                        capture_.reset();
                    }
                    resources_.set_targets_copyable(false);
                    return true;
                }

                // Before the capture exists, because this rebuilds the owned targets and
                // there is no reason to have staging buffers alive across that.
                resources_.set_targets_copyable(true);
                if (!capture_)
                    capture_ = std::unique_ptr<Graph::PassCapture>(
                        new Graph::PassCapture(device_, SLOTS));
                graph_.set_capture(capture_.get());
                return true;
            }

            bool VulkanSceneView::read_pass_hashes(std::uint32_t slot,
                                                   PassCaptureReport& report)
            {
                if (!capture_ || slot >= SLOTS || !resources_.ever_rendered(slot))
                    return false;

                resources_.wait_for_slot(slot);
                std::vector<Graph::CapturedPass> captured;
                if (!capture_->resolve(slot, captured))
                    return false;

                report.passes.clear();
                report.passes.reserve(captured.size());
                for (const Graph::CapturedPass& entry : captured)
                {
                    PassOutputHash out;
                    out.pass = entry.pass;
                    out.resource = entry.resource;
                    out.hash = entry.hash;
                    report.passes.push_back(std::move(out));
                }
                report.dropped_by_budget = capture_->dropped_by_budget(slot);
                report.dropped_by_format = capture_->dropped_by_format(slot);
                report.bytes_used = capture_->bytes_used(slot);
                report.bytes_budget = capture_->budget();
                return true;
            }
        } // namespace Vulkan
    } // namespace Render
} // namespace SushiEngine
