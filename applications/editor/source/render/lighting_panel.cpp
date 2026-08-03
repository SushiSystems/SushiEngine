/**************************************************************************/
/* lighting_panel.cpp                                                    */
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

#include "lighting_panel.hpp"

#include "../ui/panel_widgets.hpp"

#include <string>

#include <imgui.h>
#include <misc/cpp/imgui_stdlib.h>

#include <SushiEngine/environment/environment.hpp>
#include <SushiEngine/render/quality_params.hpp>

namespace SushiEngine
{
    namespace Editor
    {
        using SushiEngine::Simulation::EntityId;
        using SushiEngine::Simulation::IWorldEditor;

        void draw_light_fields(ComponentEditor<SushiEngine::Simulation::LightParameters>& editor)
        {
            using Values = SushiEngine::Simulation::LightParameters;
            editor.toggle("Spot", &Values::is_spot,
                          "A cone aimed along the entity's local -Z, rather than an "
                          "omnidirectional point.");
            editor.color("Color", &Values::color, "Linear light colour, before intensity.");
            editor.number("Intensity", &Values::intensity, 0.1f, 0.0f, 1000.0f, "%.1f",
                          "Radiance scale on the same footing as the sun's; the tonemapper "
                          "turns it into pixels.");
            editor.number("Range", &Values::range, 0.1f, 0.1f, 10000.0f, "%.2f m",
                          "Distance at which the falloff reaches zero, in metres.");

            // With a mixed Spot the cone rows have no honest owner: showing one entity's cone
            // while others are points would invite editing a field most of them lack.
            if (editor.mixed(&Values::is_spot))
            {
                ImGui::TextDisabled("Cone angles hidden: the selection mixes spot and point "
                                    "lights.");
            }
            else if (editor.values().is_spot)
            {
                editor.number("Inner Angle", &Values::inner_degrees, 0.2f, 0.0f, 89.0f,
                              "%.1f deg", "Half-angle of the fully lit core of the cone.");
                editor.number("Outer Angle", &Values::outer_degrees, 0.2f, 0.0f, 89.0f,
                              "%.1f deg", "Half-angle at which the cone reaches darkness.");
            }
            editor.toggle("Casts Shadows", &Values::casts_shadows,
                          "Render a shadow map for this light. Spot lights only for now.");
        }

        void draw_lighting_panel(EditorContext& context)
        {
            if (!context.panels.lighting)
                return;
            if (!ImGui::Begin("Lighting", &context.panels.lighting))
            {
                ImGui::End();
                return;
            }

            IWorldEditor* world = world_of(context);
            if (world == nullptr)
            {
                ImGui::TextUnformatted("No scene open.");
                ImGui::End();
                return;
            }

            // The image-based-lighting source lives on the Environment object; this panel
            // edits it through the same set_environment path the Environment panel uses.
            SushiEngine::Render::Environment environment = world->environment();
            bool env_changed = false;

            // The sun is authored in exactly one place — Environment — instead of the
            // divergent second widget block this panel used to carry (its copy drifted:
            // different ranges, no astronomical-sun awareness in one direction). A link
            // beats a stale duplicate.
            ImGui::TextDisabled("Sun & sky are authored in the Environment panel.");
            ImGui::SameLine();
            if (ImGui::SmallButton("Open##sun_owner"))
            {
                context.panels.environment = true;
                ImGui::SetWindowFocus("Environment");
            }

            if (ImGui::CollapsingHeader("Image-Based Lighting", ImGuiTreeNodeFlags_DefaultOpen))
            {
                if (ImGui::Checkbox("Enabled", &environment.image_based_lighting))
                    env_changed = true;
                if (ImGui::SliderFloat("IBL Intensity", &environment.ibl_intensity, 0.0f, 4.0f))
                    env_changed = true;
            }
            if (env_changed)
                // The environment is scene content: the write lands in the world (and so
                // in the scene file, the undo history, and the play snapshot), bracketed
                // as one undo step per gesture. Preferences keep only the *default* for
                // new scenes, which editing a live scene deliberately does not touch.
                commit_environment_edit(context, *world, environment);
            finish_environment_edit(context);

            // The sun's cascade shadows are render machinery with one editor — the
            // Rendering panel's full shadow block. The four-field subset this panel used
            // to carry was the divergent-duplicate problem again (a second, partial
            // truth about the same settings), so it is a link now.
            ImGui::TextDisabled("Sun shadow settings live in the Rendering panel.");
            ImGui::SameLine();
            if (ImGui::SmallButton("Open##shadow_owner"))
            {
                context.panels.rendering = true;
                ImGui::SetWindowFocus("Rendering");
            }

            // The punctual-light budget the tier resolves this frame, so an author sees how
            // many of the lights below actually shade and cast.
            if (ImGui::CollapsingHeader("Punctual Budget"))
            {
                const SushiEngine::Render::ResolvedQuality resolved =
                    SushiEngine::Render::resolve_quality(context.render_settings);
                ImGui::Text("Max lights   %u", resolved.settings.lights.max_lights);
                ImGui::Text("Shadow atlas %u px, %u caster(s)",
                            resolved.settings.lights.shadow_atlas_size,
                            resolved.settings.lights.max_shadow_casters);
                if (resolved.params.stochastic_light_samples > 0)
                    ImGui::Text("Beyond the atlas: %u traced sample(s)/pixel",
                                resolved.params.stochastic_light_samples);
                else
                    ImGui::TextDisabled("Beyond the atlas: unshadowed (tier)");
            }

            // Shadows for the lights the atlas had no tile for. The atlas is a memory
            // budget; this is a sample budget, which is what lets the caster count stop
            // being a ceiling.
            if (ImGui::CollapsingHeader("Stochastic Shadows"))
            {
                SushiEngine::Render::LightEngineSettings& engine =
                    context.render_settings.lights;
                const SushiEngine::Render::LightEngineSettings engine_before = engine;
                ImGui::Checkbox("Trace Beyond The Atlas", &engine.stochastic_shadows);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Each pixel samples a few of the lights that hold no\n"
                                      "atlas tile and marches the GI distance field toward\n"
                                      "them for visibility; the temporal resolve averages\n"
                                      "the rest. Needs Probe GI on — it builds the field.");
                ImGui::SliderFloat("Ray Reach", &engine.stochastic_distance, 5.0f, 200.0f,
                                   "%.0f m");
                ImGui::SliderFloat("Penumbra", &engine.stochastic_softness, 1.0f, 32.0f,
                                   "%.1f");
                if (!environment.gi.enabled)
                    ImGui::TextDisabled("Probe GI is off, so there is no field to trace.");
                push_if_changed(engine_before, engine, context.preferences_dirty);
            }

            ImGui::Separator();
            if (ImGui::Button("Add Light"))
            {
                context.history.record(*world);
                select_only(context, world->create_light("Light"));
            }

            // The punctual-light list: every light-bearing entity, edited in place. This is
            // the same data the Inspector's Light component edits, gathered in one place.
            for (const EntityId id : world->entities())
            {
                if (!world->has_light(id))
                    continue;
                ImGui::PushID(static_cast<int>(id));
                const std::string label = world->name(id);
                const bool selected = !context.selected_entities.empty() &&
                                      context.selected_entities.front() == id;
                if (ImGui::CollapsingHeader(label.c_str(),
                                            selected ? ImGuiTreeNodeFlags_DefaultOpen : 0))
                {
                    if (ImGui::SmallButton("Select"))
                        select_only(context, id);

                    // Scoped to this row, not to the selection: a list is a list of
                    // individual things, and editing the row you clicked must not reach the
                    // three lights the Hierarchy happens to have selected.
                    const ComponentAccess<SushiEngine::Simulation::LightParameters> access{
                        &IWorldEditor::has_light, &IWorldEditor::light_params,
                        &IWorldEditor::set_light_params};
                    ComponentEditor<SushiEngine::Simulation::LightParameters> editor(
                        context, *world, access, id, OneEntity{});
                    draw_light_fields(editor);
                }
                ImGui::PopID();
            }

            ImGui::End();
        }
    } // namespace Editor
} // namespace SushiEngine
