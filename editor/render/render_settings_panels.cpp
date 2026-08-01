/**************************************************************************/
/* render_settings_panels.cpp                                            */
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

#include "render_settings_panels.hpp"

#include "../ui/panel_widgets.hpp"

#include <cstdint>

#include <imgui.h>

#include <SushiEngine/render/environment.hpp>
#include <SushiEngine/render/quality_params.hpp>

namespace SushiEngine
{
    namespace Editor
    {
        using SushiEngine::Simulation::IWorldEditor;

        namespace
        {
            // The one frame for every settings panel's "Tier resolves to" readout:
            // resolve once, through the same resolver the renderer runs, and let the
            // panel print only the lines its domain owns. The frame lived as three
            // hand-rolled copies before, which is exactly how readouts drift.
            void draw_tier_readout(const SushiEngine::Render::RenderSettings& settings,
                                   void (*draw_lines)(const SushiEngine::Render::ResolvedQuality&))
            {
                if (!ImGui::TreeNode("Tier resolves to"))
                    return;
                draw_lines(SushiEngine::Render::resolve_quality(settings));
                ImGui::TreePop();
            }

            /**
             * @brief Attaches an explanation to the widget just drawn.
             *
             * These panels are a wall of sliders whose names are the renderer's vocabulary,
             * not the author's — "split blend", "roughness cutoff", "knee" — and a slider
             * whose effect a user has to discover by dragging it is a slider they will drag
             * once and then leave alone. One call per row keeps the explanation next to the
             * range it explains, which is what stops the two from drifting.
             *
             * @param text What the setting does and what moving it costs.
             */
            void hint(const char* text)
            {
                if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
                    ImGui::SetTooltip("%s", text);
            }
        } // namespace

        void draw_rendering_panel(EditorContext& context)
        {
            if (!context.panels.rendering)
                return;
            if (!ImGui::Begin("Rendering", &context.panels.rendering))
            {
                ImGui::End();
                return;
            }

            SushiEngine::Render::RenderSettings& settings = context.render_settings;
            using SushiEngine::Render::AntiAliasingMode;
            using SushiEngine::Render::RenderQuality;

            // RenderSettings is plain trivially-copyable data (render_settings.hpp), so a
            // memcmp against a snapshot taken before the widgets run is a cheap, exhaustive
            // way to detect any edit below and persist it via Preferences — without hooking
            // a dirty flag into every slider individually.
            const SushiEngine::Render::RenderSettings settings_before = settings;

            const char* const QUALITY[] = {"Low", "Medium", "High", "Ultra"};
            int quality = static_cast<int>(settings.quality);
            if (ImGui::Combo("Quality", &quality, QUALITY, 4))
                settings.quality = static_cast<RenderQuality>(quality);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Scales rendering only: shadows, clouds' march, material\n"
                                  "lobes, post effects, GPU-driven geometry. The weather\n"
                                  "simulation has its own tier in the Meteorology panel, so\n"
                                  "changing this never touches the running weather.");

            const char* const ANTI_ALIASING[] = {"None", "FXAA", "Temporal"};
            int anti_aliasing = static_cast<int>(settings.anti_aliasing);
            if (ImGui::Combo("Anti-aliasing", &anti_aliasing, ANTI_ALIASING, 3))
                settings.anti_aliasing = static_cast<AntiAliasingMode>(anti_aliasing);

            ImGui::SliderFloat("Render Scale", &settings.render_scale, 0.5f, 1.0f, "%.2f");

            if (settings.anti_aliasing == AntiAliasingMode::Temporal)
            {
                ImGui::PushID("Temporal");
                ImGui::SliderFloat("Still Feedback", &settings.temporal.feedback_still,
                                   0.5f, 0.99f, "%.3f");
                hint("How much of last frame a still pixel keeps. Higher is steadier and "
                     "smears more through disocclusions.");
                ImGui::SliderFloat("Moving Feedback", &settings.temporal.feedback_moving,
                                   0.5f, 0.99f, "%.3f");
                hint("The same weight for a pixel in motion, where a long history is what "
                     "produces ghosting.");
                ImGui::SliderFloat("Sharpness", &settings.temporal.sharpness, 0.0f, 1.0f,
                                   "%.2f");
                hint("Post-resolve sharpening that buys back the softness accumulation costs. "
                     "Too high rings around edges.");
                ImGui::SliderFloat("Jitter", &settings.temporal.jitter_scale, 0.0f, 1.0f,
                                   "%.2f");
                hint("Size of the sub-pixel camera offset each frame. Below one, accumulation "
                     "resolves less real detail.");
                ImGui::Checkbox("Clamp History", &settings.temporal.clamp_history);
                hint("Rejects history samples outside the neighbourhood colour range. The main "
                     "defence against ghosting; off is faster and dirtier.");
                ImGui::PopID();
            }
            else
            {
                ImGui::TextDisabled("Temporal options need the temporal resolve.");
            }

            ImGui::Separator();
            ImGui::Checkbox("Shadows", &settings.shadows.enabled);
            if (settings.shadows.enabled)
            {
                ImGui::PushID("Shadows");
                int cascades = static_cast<int>(settings.shadows.cascade_count);
                if (ImGui::SliderInt("Cascades", &cascades, 1, 4))
                    settings.shadows.cascade_count = static_cast<std::uint32_t>(cascades);

                const char* const RESOLUTIONS[] = {"512", "1024", "2048", "4096"};
                const std::uint32_t VALUES[] = {512u, 1024u, 2048u, 4096u};
                int resolution = 2;
                for (int i = 0; i < 4; ++i)
                    if (VALUES[i] == settings.shadows.resolution)
                        resolution = i;
                if (ImGui::Combo("Resolution", &resolution, RESOLUTIONS, 4))
                    settings.shadows.resolution = VALUES[resolution];

                ImGui::SliderFloat("Shadow Distance", &settings.shadows.distance, 20.0f, 4000.0f,
                                   "%.0f m");
                hint("How far from the camera the cascades cover, in metres. The single "
                     "biggest lever on shadow resolution: less distance is sharper.");
                ImGui::SliderFloat("Split Blend", &settings.shadows.split_blend, 0.0f, 1.0f,
                                   "%.2f");
                hint("Fraction of each cascade blended into the next, hiding the seam between "
                     "resolutions.");
                ImGui::SliderFloat("Normal Bias", &settings.shadows.normal_bias, 0.0f, 6.0f,
                                   "%.2f");
                hint("Pushes the shadow lookup along the surface normal, in texels. Cures "
                     "self-shadowing acne; too much detaches contact shadows.");
                ImGui::SliderFloat("Depth Bias", &settings.shadows.depth_bias, 0.0f, 0.01f,
                                   "%.4f");
                hint("Constant depth offset applied to the comparison. Same trade as Normal "
                     "Bias, applied in depth rather than along the surface.");
                ImGui::SliderFloat("Softness", &settings.shadows.softness, 0.0f, 10.0f, "%.2f");
                hint("Width of the penumbra the filter fakes. Wider reads as a larger light "
                     "source and costs more taps.");
                ImGui::SliderFloat("Min Filter", &settings.shadows.filter_radius, 0.5f, 8.0f,
                                   "%.2f");
                hint("Smallest filter radius, in texels, used where the shadow is sharp.");
                ImGui::SliderFloat("Max Filter", &settings.shadows.max_filter_radius, 2.0f,
                                   48.0f, "%.1f");
                hint("Largest filter radius, in texels; the ceiling on penumbra cost.");
                ImGui::SliderFloat("Cascade Blend", &settings.shadows.cascade_blend, 0.0f, 0.5f,
                                   "%.2f");
                hint("Extra cross-fade at the far edge of the last cascade, so shadows fade "
                     "out rather than stopping at a line.");

                ImGui::Checkbox("Contact Shadows", &settings.shadows.contact_shadows);
                if (settings.shadows.contact_shadows)
                {
                    ImGui::SliderFloat("Contact Reach", &settings.shadows.contact_distance,
                                       0.05f, 2.0f, "%.2f m");
                    hint("How far the screen-space trace searches for a near occluder, in "
                         "metres. This is what puts an object back in contact with the floor.");
                    int steps = static_cast<int>(settings.shadows.contact_steps);
                    if (ImGui::SliderInt("Contact Steps", &steps, 4, 32))
                        settings.shadows.contact_steps = static_cast<std::uint32_t>(steps);
                }

                ImGui::Checkbox("Ray Traced (Ultra)", &settings.shadows.ray_traced);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Traces the sun ray instead of sampling a cascade.\n"
                                      "Ignored unless the device offers ray queries and an\n"
                                      "acceleration structure was built; cascades remain the\n"
                                      "fallback for whatever the structure does not contain.");

                int secondary_casters =
                    static_cast<int>(settings.shadows.max_directional_shadow_casters);
                if (ImGui::SliderInt("Secondary Shadow Casters", &secondary_casters, 0, 4))
                    settings.shadows.max_directional_shadow_casters =
                        static_cast<std::uint32_t>(secondary_casters);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Secondary celestial lights (beyond light 0) that may\n"
                                      "claim a shadow map. They share a tile budget with\n"
                                      "punctual shadow casters below. Zero disables secondary\n"
                                      "directional shadows entirely.");
                ImGui::PopID();
            }

            ImGui::Separator();
            ImGui::Checkbox("Ambient Occlusion (GTAO)", &settings.gtao.enabled);
            if (settings.gtao.enabled)
            {
                ImGui::PushID("Gtao");
                ImGui::SliderFloat("Radius", &settings.gtao.radius, 0.1f, 4.0f, "%.2f m");
                hint("World-space radius the occlusion is gathered over, in metres. Large "
                     "values darken broadly; small ones only crease corners.");
                ImGui::SliderFloat("Intensity", &settings.gtao.intensity, 0.0f, 2.0f, "%.2f");
                hint("Strength of the darkening. Above one it is no longer physical.");
                ImGui::SliderFloat("Power", &settings.gtao.power, 0.5f, 4.0f, "%.2f");
                hint("Exponent on the occlusion term, shaping its falloff from linear to "
                     "sharply contrasted.");

                int slices = static_cast<int>(settings.gtao.slices);
                if (ImGui::SliderInt("Slices", &slices, 1, 6))
                    settings.gtao.slices = static_cast<std::uint32_t>(slices);

                int gtao_steps = static_cast<int>(settings.gtao.steps);
                if (ImGui::SliderInt("Steps", &gtao_steps, 2, 12))
                    settings.gtao.steps = static_cast<std::uint32_t>(gtao_steps);
                ImGui::TextDisabled("Darkens creases and contacts the ambient/IBL term can't see.");
                ImGui::PopID();
            }

            ImGui::Separator();
            ImGui::Checkbox("Screen-Space Reflections", &settings.ssr.enabled);
            if (settings.ssr.enabled)
            {
                ImGui::PushID("Ssr");
                int ssr_steps = static_cast<int>(settings.ssr.max_steps);
                if (ImGui::SliderInt("Max Steps", &ssr_steps, 8, 128))
                    settings.ssr.max_steps = static_cast<std::uint32_t>(ssr_steps);
                ImGui::SliderFloat("Thickness", &settings.ssr.thickness, 0.05f, 4.0f, "%.2f m");
                hint("Assumed depth of what the depth buffer shows, in metres, since a depth "
                     "buffer records surfaces and not solids.");
                ImGui::SliderFloat("Roughness Cutoff", &settings.ssr.roughness_cutoff, 0.0f, 1.0f,
                                   "%.2f");
                hint("Above this roughness a surface falls back to the probes: a rough "
                     "reflection needs more rays than it is worth.");
                ImGui::SliderFloat("Intensity", &settings.ssr.intensity, 0.0f, 1.0f, "%.2f");
                hint("How much the traced reflection replaces the probe one.");
                ImGui::TextDisabled("Traces the scene's own colour into glossy surfaces through\n"
                                    "the hi-Z pyramid, falling back to IBL where a ray misses.");
                ImGui::PopID();
            }

            ImGui::Separator();
            ImGui::Checkbox("Dynamic Resolution", &settings.dynamic_resolution.enabled);
            if (settings.dynamic_resolution.enabled)
            {
                ImGui::PushID("DynamicResolution");
                ImGui::SliderFloat("GPU Budget",
                                   &settings.dynamic_resolution.target_milliseconds, 2.0f,
                                   33.0f, "%.1f ms");
                hint("Frame time the governor aims for. It lowers the render scale to stay "
                     "under this and raises it again when there is room.");
                ImGui::SliderFloat("Minimum Scale",
                                   &settings.dynamic_resolution.minimum_scale, 0.25f, 1.0f,
                                   "%.2f");
                hint("The floor the governor may drop the render scale to.");
                ImGui::SliderFloat("Maximum Scale",
                                   &settings.dynamic_resolution.maximum_scale, 0.25f, 1.0f,
                                   "%.2f");
                hint("The ceiling the governor may raise the render scale to.");
                ImGui::PopID();
            }

            ImGui::Separator();
            ImGui::Checkbox("Variable Rate Shading",
                            &settings.variable_rate_shading.enabled);
            if (settings.variable_rate_shading.enabled)
            {
                ImGui::PushID("VariableRateShading");
                ImGui::SliderFloat("Contrast Threshold",
                                   &settings.variable_rate_shading.luminance_threshold, 0.0f,
                                   0.5f, "%.3f");
                hint("Local contrast below which a tile may be shaded at reduced rate: flat "
                     "regions cost less without a visible difference.");
                ImGui::SliderFloat("Motion Threshold",
                                   &settings.variable_rate_shading.velocity_threshold, 0.0f,
                                   0.2f, "%.3f");
                hint("Screen-space motion above which a tile may be shaded at reduced rate, "
                     "since motion hides the loss.");
                ImGui::TextDisabled("Ignored on a device without shading rate images.");
                ImGui::PopID();
            }

            // What the resolution governor actually settled on, which is the only way to
            // see it work: the slider is a request, this is the answer.
            ImGui::Separator();
            if (context.scene_render_width > 0)
                ImGui::Text("Scene view rendering at %u x %u", context.scene_render_width,
                            context.scene_render_height);

            // What the tier resolves to. The Quality combo above is a request; this is what
            // each pass is actually handed once the resolver has run — the same values the
            // renderer uses, so switching tiers shows the expensive half rescale here and,
            // in milliseconds, in the profiler HUD over the viewport.
            draw_tier_readout(settings, [](const SushiEngine::Render::ResolvedQuality& resolved)
            {
                const SushiEngine::Render::RenderSettings& effective = resolved.settings;
                const SushiEngine::Render::QualityParams& knobs = resolved.params;

                if (effective.shadows.enabled)
                {
                    ImGui::Text("Shadow atlas   %u px, %u cascade(s)", effective.shadows.resolution,
                                effective.shadows.cascade_count);
                    ImGui::Text("PCSS taps      %u filter / %u blocker", knobs.shadow_filter_taps,
                                knobs.shadow_blocker_taps);
                    if (effective.shadows.contact_shadows)
                        ImGui::Text("Contact march  %u steps @ %.2f m",
                                    effective.shadows.contact_steps,
                                    effective.shadows.contact_distance);
                }
                else
                {
                    ImGui::TextDisabled("Shadows off");
                }
                ImGui::Text("Cloud march    %u near / %u far / %u light",
                            knobs.cloud_primary_steps_near, knobs.cloud_primary_steps_far,
                            knobs.cloud_light_steps);
                ImGui::Text("VRS coarse cap %ux (1 = full rate)", knobs.vrs_max_coarse_axis);
                ImGui::Text("Punctual lights %u max, %.0f m cluster reach",
                            effective.lights.max_lights, effective.lights.cluster_far_distance);
                ImGui::Text("Decals         %u max; shadow atlas %u px / %u caster(s)",
                            effective.lights.max_decals, effective.lights.shadow_atlas_size,
                            effective.lights.max_shadow_casters);
                ImGui::Text("Lobes          %s%s%s%s",
                            knobs.lobe_anisotropy ? "aniso " : "",
                            knobs.lobe_clearcoat ? "clearcoat " : "",
                            knobs.lobe_sheen ? "sheen " : "",
                            knobs.lobe_transmission ? "transmission" : "");
                if (!knobs.lobe_anisotropy && !knobs.lobe_clearcoat && !knobs.lobe_sheen &&
                    !knobs.lobe_transmission)
                {
                    ImGui::SameLine();
                    ImGui::TextDisabled("base PBR only");
                }
                ImGui::Text("Async compute  %s", knobs.async_compute ? "permitted" : "off (tier)");
                ImGui::Text("Frames in flight %u", effective.delivery.frames_in_flight);
            });

            ImGui::Separator();
            // Delivery, not fidelity: nothing here changes a pixel, only how much of the
            // device is kept busy and how long a finished frame waits to be seen.
            if (ImGui::TreeNode("Frame Delivery"))
            {
                using SushiEngine::Render::PresentMode;
                SushiEngine::Render::FrameDeliverySettings& delivery = settings.delivery;

                ImGui::Checkbox("Async Compute", &delivery.async_compute);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Runs the flagged compute passes (cluster build, GTAO)\n"
                                      "on a second queue so they overlap the graphics work\n"
                                      "they do not depend on. Ignored where the device has\n"
                                      "no compute queue family of its own.");

                int in_flight = static_cast<int>(delivery.frames_in_flight);
                if (ImGui::SliderInt("Frames in Flight", &in_flight, 2, 3))
                    delivery.frames_in_flight = static_cast<std::uint32_t>(in_flight);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("How far the CPU may run ahead of the GPU.\n"
                                      "Three smooths over a hitch; two cuts latency.");

                const char* const PRESENT[] = {"V-Sync", "Mailbox", "Immediate"};
                int present = static_cast<int>(delivery.present_mode);
                if (ImGui::Combo("Present Mode", &present, PRESENT, 3))
                    delivery.present_mode = static_cast<PresentMode>(present);

                // No Upscaler combo: the frame never consumed an authored backend choice
                // (the temporal reconstruction is governed by Anti-Aliasing and Render
                // Scale), so the combo promised five backends and delivered none of the
                // choice. It returns together with a second real backend — the IUpscaler
                // seam it would select over is still there (render/frame/upscaler.hpp).
                ImGui::TreePop();
            }

            push_if_changed(settings_before, settings, context.preferences_dirty);

            ImGui::End();
        }


        void draw_post_process_panel(EditorContext& context)
        {
            if (!context.panels.post_process)
                return;
            if (!ImGui::Begin("Post Process", &context.panels.post_process))
            {
                ImGui::End();
                return;
            }

            using SushiEngine::Render::ExposureMode;
            using SushiEngine::Render::TonemapOperator;
            SushiEngine::Render::RenderSettings& settings = context.render_settings;
            SushiEngine::Render::PostProcessSettings& post = settings.post;

            // Same exhaustive-memcmp persistence as the Rendering panel: RenderSettings is
            // trivially-copyable, so a snapshot before the widgets catches any edit below.
            const SushiEngine::Render::RenderSettings settings_before = settings;

            if (ImGui::CollapsingHeader("Exposure", ImGuiTreeNodeFlags_DefaultOpen))
            {
                ImGui::PushID("Exposure");
                // The whole exposure chain is authored here, in one panel. The scene
                // multiplier below is *scene content* (it lives on the environment and
                // rides the scene file and the undo history); the mode/EV fields around
                // it are per-user render settings. One home, two owners of the data —
                // the panel says which is which so neither fights the other blind.
                if (IWorldEditor* world = world_of(context))
                {
                    SushiEngine::Render::Environment scene_environment = world->environment();
                    if (ImGui::SliderFloat("Scene Exposure", &scene_environment.exposure,
                                           0.02f, 1.0f))
                        commit_environment_edit(context, *world, scene_environment);
                    finish_environment_edit(context);
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("The environment's pre-tonemap multiplier — saved\n"
                                          "with the scene, not with the editor preferences.");
                }
                const char* const MODES[] = {"Manual", "Automatic"};
                int mode = static_cast<int>(post.exposure_mode);
                if (ImGui::Combo("Mode", &mode, MODES, 2))
                    post.exposure_mode = static_cast<ExposureMode>(mode);

                if (post.exposure_mode == ExposureMode::Manual)
                {
                    ImGui::SliderFloat("Compensation", &post.exposure_compensation,
                                       -6.0f, 6.0f, "%.2f EV");
                    ImGui::TextDisabled("Multiplies the scene's authored exposure.");
                }
                else
                {
                    // Labelled by what they do, not by the field names. These clamp the
                    // *metered* luminance, so the lower one bounds the exposure from above —
                    // the control that decides whether a night scene can be resolved at all is
                    // the one called "Minimum", which reads exactly backwards. The range
                    // reaches -16 because a physically lit night sits around 1e-5 in scene
                    // units and needs ~1000x of gain; the old -10 floor could not express it,
                    // so a correct night was unreachable rather than merely mistuned.
                    ImGui::SliderFloat("Darkest Metered", &post.auto_exposure.min_ev, -16.0f, 8.0f,
                                       "%.1f log2");
                    hint("The darkest scene the eye will adapt to, as log2 of luminance in "
                         "scene units (not photographic EV). Lowering this RAISES the maximum "
                         "exposure, so it is what to reach for when a night renders black. "
                         "Below about -13 the pass's own 1800x ceiling takes over.");
                    ImGui::SliderFloat("Brightest Metered", &post.auto_exposure.max_ev, -2.0f,
                                       20.0f, "%.1f log2");
                    hint("The brightest scene the eye will adapt to. Raising this lowers the "
                         "minimum exposure, which is what stops a bright sky blowing out.");
                    ImGui::SliderFloat("Compensation", &post.auto_exposure.compensation,
                                       -6.0f, 6.0f, "%.2f EV");
                    ImGui::SliderFloat("Adapt Up", &post.auto_exposure.speed_up, 0.1f, 8.0f, "%.2f");
                    hint("How fast the eye adapts when the scene brightens, in stops per "
                         "second.");
                    ImGui::SliderFloat("Adapt Down", &post.auto_exposure.speed_down, 0.1f, 8.0f,
                                       "%.2f");
                    hint("How fast the eye adapts when the scene darkens.");
                    ImGui::SliderFloat("Key", &post.auto_exposure.key, 0.02f, 0.5f, "%.3f");
                    hint("Target average luminance the metering aims to place mid-grey at.");
                }
                ImGui::PopID();
            }

            if (ImGui::CollapsingHeader("Tone Mapping", ImGuiTreeNodeFlags_DefaultOpen))
            {
                const char* const OPERATORS[] = {"AgX", "ACES", "Khronos Neutral"};
                int op = static_cast<int>(post.tonemap);
                if (ImGui::Combo("Curve", &op, OPERATORS, 3))
                    post.tonemap = static_cast<TonemapOperator>(op);
            }

            if (ImGui::CollapsingHeader("Bloom", ImGuiTreeNodeFlags_DefaultOpen))
            {
                ImGui::PushID("Bloom");
                ImGui::Checkbox("Enabled", &post.bloom.enabled);
                ImGui::SliderFloat("Intensity", &post.bloom.intensity, 0.0f, 0.5f, "%.3f");
                hint("How much of the blurred bright pass is added back.");
                ImGui::SliderFloat("Threshold", &post.bloom.threshold, 0.0f, 4.0f, "%.2f");
                hint("Luminance above which a pixel contributes to bloom.");
                ImGui::SliderFloat("Knee", &post.bloom.threshold_knee, 0.0f, 1.0f, "%.2f");
                hint("Softness of that threshold, so a pixel does not start blooming the "
                     "instant it crosses the line.");
                ImGui::PopID();
            }

            if (ImGui::CollapsingHeader("Color Grade"))
            {
                ImGui::PushID("Grade");
                ImGui::SliderFloat("Temperature", &post.grade.temperature, -1.0f, 1.0f, "%.2f");
                ImGui::SliderFloat("Tint", &post.grade.tint, -1.0f, 1.0f, "%.2f");
                ImGui::SliderFloat("Contrast", &post.grade.contrast, 0.5f, 2.0f, "%.2f");
                ImGui::SliderFloat("Saturation", &post.grade.saturation, 0.0f, 2.0f, "%.2f");
                ImGui::SliderFloat3("Lift", post.grade.lift, -0.5f, 0.5f, "%.3f");
                ImGui::SliderFloat3("Gamma", post.grade.gamma, 0.1f, 3.0f, "%.2f");
                ImGui::SliderFloat3("Gain", post.grade.gain, 0.0f, 3.0f, "%.2f");
                ImGui::PopID();
            }

            if (ImGui::CollapsingHeader("Depth of Field"))
            {
                ImGui::PushID("DoF");
                ImGui::Checkbox("Enabled", &post.depth_of_field.enabled);
                ImGui::SliderFloat("Focus Distance", &post.depth_of_field.focus_distance,
                                   0.1f, 1000.0f, "%.2f m", ImGuiSliderFlags_Logarithmic);
                hint("Distance from the camera that is in focus, in metres.");
                ImGui::SliderFloat("Focus Range", &post.depth_of_field.focus_range,
                                   0.05f, 100.0f, "%.2f m", ImGuiSliderFlags_Logarithmic);
                hint("Depth around the focus distance that stays acceptably sharp.");
                ImGui::SliderFloat("Aperture", &post.depth_of_field.aperture, 0.7f, 22.0f,
                                   "f/%.1f");
                hint("Lens f-number. Lower means a wider opening and a shallower depth of "
                     "field, exactly as on a camera.");
                ImGui::SliderFloat("Max Radius", &post.depth_of_field.max_radius, 1.0f, 16.0f,
                                   "%.1f px");
                hint("Largest blur circle, in pixels; the ceiling on the effect cost.");
                ImGui::PopID();
            }

            if (ImGui::CollapsingHeader("Motion Blur"))
            {
                ImGui::PushID("MotionBlur");
                ImGui::Checkbox("Enabled", &post.motion_blur.enabled);
                ImGui::SliderFloat("Intensity", &post.motion_blur.intensity, 0.0f, 2.0f, "%.2f");
                int samples = static_cast<int>(post.motion_blur.samples);
                if (ImGui::SliderInt("Samples", &samples, 2, 32))
                    post.motion_blur.samples = static_cast<std::uint32_t>(samples);
                ImGui::PopID();
            }

            if (ImGui::CollapsingHeader("Lens"))
            {
                ImGui::PushID("Lens");
                ImGui::SliderFloat("Vignette", &post.vignette, 0.0f, 1.0f, "%.2f");
                ImGui::SliderFloat("Chromatic Aberration", &post.chromatic_aberration, 0.0f, 8.0f,
                                   "%.2f");
                ImGui::SliderFloat("Film Grain", &post.film_grain, 0.0f, 0.2f, "%.3f");
                ImGui::PopID();
            }

            draw_tier_readout(settings, [](const SushiEngine::Render::ResolvedQuality& resolved)
            {
                const SushiEngine::Render::QualityParams& knobs = resolved.params;
                ImGui::Text("Bloom: %s", knobs.bloom ? "on" : "off (tier)");
                ImGui::Text("Depth of field: %s", knobs.depth_of_field ? "permitted" : "off (tier)");
                ImGui::Text("Motion blur: %s", knobs.motion_blur ? "permitted" : "off (tier)");
                ImGui::TextDisabled("The tier permits an effect; the toggle above enables it.");
            });

            push_if_changed(settings_before, settings, context.preferences_dirty);

            ImGui::End();
        }

        void draw_gpu_culling_panel(EditorContext& context)
        {
            if (!context.panels.gpu_culling)
                return;
            if (!ImGui::Begin("GPU Culling", &context.panels.gpu_culling))
            {
                ImGui::End();
                return;
            }

            SushiEngine::Render::RenderSettings& settings = context.render_settings;
            SushiEngine::Render::GpuCullingSettings& cull = settings.gpu_culling;

            // Same exhaustive-memcmp persistence as the Post-Process panel: RenderSettings is
            // trivially-copyable, so a snapshot before the widgets catches any edit below.
            const SushiEngine::Render::RenderSettings settings_before = settings;

            ImGui::Checkbox("Enabled", &cull.enabled);
            ImGui::TextDisabled("Take the GPU-driven path when the tier permits it.");

            if (ImGui::CollapsingHeader("Cull Tests", ImGuiTreeNodeFlags_DefaultOpen))
            {
                ImGui::PushID("CullTests");
                ImGui::Indent();
                ImGui::Checkbox("Frustum", &cull.frustum);
                ImGui::Checkbox("Occlusion", &cull.occlusion);
                ImGui::SliderFloat("Min Screen Diameter", &cull.min_screen_diameter,
                                   0.0f, 16.0f, "%.1f px");
                hint("Instances whose projected diameter falls below this many pixels are "
                     "dropped before shading.");
                ImGui::TextDisabled("Drop instances whose projected diameter is below this.");
                ImGui::Unindent();
                ImGui::PopID();
            }

            if (ImGui::CollapsingHeader("Debug"))
            {
                ImGui::PushID("Debug");
                ImGui::Checkbox("Freeze frustum", &cull.freeze);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Latches the cull frustum at this camera pose so you can\n"
                                      "fly out and watch what it keeps and drops. The LOD gate\n"
                                      "and occlusion keep following the live camera.");
                ImGui::PopID();
            }

            // The Scene view's real cull counts, read back a frame late by the main loop
            // — the numbers the old "Show statistics" checkbox only promised.
            ImGui::SeparatorText("Last frame");
            if (context.scene_cull_tested == 0)
                ImGui::TextDisabled("No GPU-driven frame yet (classic path, or the tier "
                                    "keeps it off).");
            else
                ImGui::Text("Drawn %u of %u tested (%.0f%% culled)", context.scene_cull_drawn,
                            context.scene_cull_tested,
                            100.0f * (1.0f - static_cast<float>(context.scene_cull_drawn) /
                                                 static_cast<float>(context.scene_cull_tested)));

            draw_tier_readout(settings, [](const SushiEngine::Render::ResolvedQuality& resolved)
            {
                ImGui::Text("GPU-driven path: %s",
                            resolved.params.gpu_driven ? "permitted" : "off (tier)");
                ImGui::TextDisabled("The Low tier keeps the classic one-draw-per-instance path.");
            });

            push_if_changed(settings_before, settings, context.preferences_dirty);

            ImGui::End();
        }
    } // namespace Editor
} // namespace SushiEngine
