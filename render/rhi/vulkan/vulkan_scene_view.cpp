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

#include "frame/temporal_jitter.hpp"
#include "material/asset_library.hpp"
#include "resources/sampler_cache.hpp"
#include "scene/scene_uniforms.hpp"
#include "scene/shadow_uniforms.hpp"
#include "scene/temporal_uniforms.hpp"
#include "vulkan_check.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Vulkan
        {
            namespace
            {
                /** @brief Upper bound on timed passes in one frame. */
                constexpr std::uint32_t MAX_TIMED_PASSES = 16;
            } // namespace

            VulkanSceneView::VulkanSceneView(VulkanDevice& device,
                                             Assets::AssetLibrary& assets)
                : device_(device), assets_(assets), descriptors_(device, SLOTS),
                  cloth_(device, SLOTS), materials_(device, assets.textures(), SLOTS),
                  motion_(device, SLOTS), lights_(device, SLOTS),
                  accelerator_(device, assets.meshes(), SLOTS),
                  profiler_(device, SLOTS, MAX_TIMED_PASSES),
                  graph_(device, &profiler_),
                  atmosphere_lut_pass_(device, assets.shaders(), assets.pipelines()),
                  volumetric_fog_pass_(device, assets.shaders(), assets.pipelines(),
                                       atmosphere_lut_pass_),
                  ibl_pass_(device, assets.shaders(), assets.pipelines(), assets.samplers(),
                            assets.layout(), assets.cloud_noise(), atmosphere_lut_pass_,
                            volumetric_fog_pass_),
                  depth_prepass_(device, assets.shaders(), assets.pipelines(), assets.layout(),
                                 assets.meshes(), motion_),
                  shadow_pass_(device, assets.shaders(), assets.pipelines(), assets.layout(),
                               assets.meshes()),
                  contact_shadow_pass_(device, assets.shaders(), assets.pipelines(),
                                       assets.layout()),
                  ray_shadow_pass_(device, assets.shaders(), assets.pipelines(), accelerator_),
                  gtao_pass_(device, assets.shaders(), assets.pipelines(), assets.layout()),
                  hiz_pass_(device, assets.shaders(), assets.pipelines()),
                  opaque_pass_(device, assets.shaders(), assets.pipelines(), assets.layout(),
                               assets.meshes(), cloth_, materials_, motion_,
                               assets.cloud_noise(), ibl_pass_, lights_),
                  light_cull_pass_(device, assets.shaders(), assets.pipelines(), lights_),
                  light_shadow_pass_(device, assets.shaders(), assets.pipelines(), assets.layout(),
                                     assets.meshes(), lights_),
                  shading_rate_pass_(device, assets.shaders(), assets.pipelines()),
                  sky_pass_(device, assets.shaders(), assets.pipelines(), assets.layout(),
                            assets.cloud_noise(), atmosphere_lut_pass_, volumetric_fog_pass_),
                  ground_shadow_resolve_pass_(device, assets.shaders(), assets.pipelines(),
                                              assets.layout()),
                  cloud_pass_(device, assets.shaders(), assets.pipelines(), assets.layout(),
                              assets.cloud_noise()),
                  cloud_composite_pass_(device, assets.shaders(), assets.pipelines(),
                                        assets.layout()),
                  ssr_pass_(device, assets.shaders(), assets.pipelines(), assets.layout(),
                            hiz_pass_),
                  taa_pass_(device, assets.shaders(), assets.pipelines(), assets.layout()),
                  tonemap_pass_(device, assets.shaders(), assets.pipelines(), assets.layout()),
                  fxaa_pass_(device, assets.shaders(), assets.pipelines(), assets.layout()),
                  resources_(device, assets.samplers().get(Resources::SamplerDesc{}), 16u, 16u)
            {
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
                           &volumetric_fog_pass_,
                           &ibl_pass_,
                           &depth_prepass_,
                           &shadow_pass_,
                           &contact_shadow_pass_,
                           &ray_shadow_pass_,
                           &light_cull_pass_,
                           &light_shadow_pass_,
                           &gtao_pass_,
                           &hiz_pass_,
                           &opaque_pass_,
                           &shading_rate_pass_,
                           &sky_pass_,
                           &ground_shadow_resolve_pass_,
                           &cloud_pass_,
                           &cloud_composite_pass_,
                           &ssr_pass_,
                           &taa_pass_,
                           &tonemap_pass_,
                           &fxaa_pass_,
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

            void VulkanSceneView::render(const CameraView& camera, const Environment& environment,
                                         const MeshInstance* instances, std::size_t count,
                                         std::uint32_t selected_id,
                                         const ClothStrandView* strands, std::size_t strand_count,
                                         const PunctualLight* lights, std::size_t light_count,
                                         const Decal* decals, std::size_t decal_count)
            {
                const std::uint32_t index = frame_counter_ % SLOTS;
                const VkFence fence = resources_.fence(index);
                const VkCommandBuffer cmd = resources_.command_buffer(index);

                if (resources_.ever_rendered(index))
                    check(vkWaitForFences(device_.device(), 1, &fence, VK_TRUE, UINT64_MAX),
                          "vkWaitForFences");
                // The fence has been waited on, so this slot's previous submit is complete:
                // its timestamps are readable and its transient resources are free.
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

                check(vkResetFences(device_.device(), 1, &fence), "vkResetFences");
                check(vkResetCommandPool(device_.device(), resources_.command_pool(index), 0),
                      "vkResetCommandPool");
                descriptors_.begin_frame(index);
                resources_.begin_slot(index);

                Frame::FrameContext frame;
                frame.index = frame_counter_;
                frame.slot = index;
                frame.width = render_width_;
                frame.height = render_height_;
                frame.output_width = width_;
                frame.output_height = height_;
                // Resolve the quality tier once, here, into the effective settings every
                // pass reads and the extra per-pass knobs that have no home in
                // RenderSettings. The authored settings_ stays the untouched baseline the
                // tier scales from, so the next frame resolves from the same request.
                const ResolvedQuality resolved = resolve_quality(settings_);
                const RenderSettings& effective = resolved.settings;
                frame.settings = effective;
                frame.quality = resolved.params;
                frame.camera = &camera;
                frame.environment = &environment;
                Scene::camera_eye(camera.view, frame.eye);
                frame.draws.instances = instances;
                frame.draws.instance_count = count;
                frame.draws.strands = strands;
                frame.draws.strand_count = strand_count;
                frame.draws.lights = lights;
                frame.draws.light_count = light_count;
                frame.draws.decals = decals;
                frame.draws.decal_count = decal_count;
                frame.draws.selected_id = selected_id;
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

                Scene::SceneUniforms uniforms;
                Scene::fill_scene_uniforms(camera, environment, frame.eye,
                                           static_cast<float>(frame_counter_) * 0.016f, uniforms);
                // The image-based lighting chain is this view's, so its parameters are
                // stamped in here rather than by the environment fill, which knows
                // nothing about the renderer's internals.
                uniforms.ibl_params[0] = environment.ibl_intensity;
                uniforms.ibl_params[1] = static_cast<float>(ibl_pass_.specular_mip_count());
                uniforms.ibl_params[2] = environment.image_based_lighting ? 1.0f : 0.0f;
                // The main sky pass reads the background from the sky-view LUT; the IBL
                // cube capture clears this so it keeps the per-pixel march (see IblPass).
                uniforms.ibl_params[3] = 1.0f;
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
                Scene::ShadowUniforms shadow_uniforms;
                frame.cascade_count = Scene::fit_shadow_cascades(
                    camera, frame.eye, environment.sun.direction, effective.shadows,
                    shadow_uniforms);
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
                // ride the two spare lanes the fit leaves untouched (params.z, filter.w),
                // so the tier can cheapen the penumbra filter with no struct change and no
                // extra plumbing — the sampler reads them straight back.
                shadow_uniforms.params[2] =
                    static_cast<float>(resolved.params.shadow_filter_taps);
                shadow_uniforms.filter[3] =
                    static_cast<float>(resolved.params.shadow_blocker_taps);
                resources_.upload_shadow(index, shadow_uniforms);

                Scene::TemporalUniforms temporal;
                Scene::fill_temporal_uniforms(effective, previous_camera_.view,
                                              previous_camera_.projection, frame.jitter,
                                              previous_jitter_, render_width_, render_height_,
                                              width_, height_, frame.history_valid, temporal);
                resources_.upload_temporal(index, temporal);

                materials_.begin_frame(index);
                motion_.begin_frame(index, frame.eye);
                // Pack and configure the light engine before the passes register: the cull
                // pass reads the packed count and the grid's near/far when it builds its
                // push constant, and the far distance is the tier-scaled cluster reach.
                lights_.begin_frame(index);
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
                graph_.begin_frame(resources_.textures(index), resources_.buffers(index));
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

                VkCommandBufferBeginInfo begin{};
                begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
                begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
                check(vkBeginCommandBuffer(cmd, &begin), "vkBeginCommandBuffer");
                profiler_.begin_frame(index, cmd);
                graph_.execute(cmd);
                check(vkEndCommandBuffer(cmd), "vkEndCommandBuffer");

                VkCommandBufferSubmitInfo cmd_submit{};
                cmd_submit.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO;
                cmd_submit.commandBuffer = cmd;
                VkSubmitInfo2 submit{};
                submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO_2;
                submit.commandBufferInfoCount = 1;
                submit.pCommandBufferInfos = &cmd_submit;
                check(vkQueueSubmit2(device_.graphics_queue(), 1, &submit, fence),
                      "vkQueueSubmit2");

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
        } // namespace Vulkan
    } // namespace Render
} // namespace SushiEngine
