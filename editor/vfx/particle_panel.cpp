/**************************************************************************/
/* particle_panel.cpp                                                    */
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

#include "particle_panel.hpp"

#include "effect_preview.hpp"
#include "../serialization/effect_serializer.hpp"
#include "../ui/panel_widgets.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

#include <imgui.h>
#include <misc/cpp/imgui_stdlib.h>

namespace SushiEngine
{
    namespace Editor
    {
        using SushiEngine::Simulation::EntityId;
        using SushiEngine::Simulation::IWorldEditor;

        namespace fs = std::filesystem;

        namespace
        {
            /** @brief Where `.sushieffect` assets live, relative to the project the editor ran in. */
            const char* const EFFECT_LIBRARY_DIRECTORY = "assets/effects";

            /**
             * @brief The selected emitter's own effect, created on first use.
             *
             * A new emitter starts from the same default the preview does, so an author who adds
             * the component sees particles immediately and edits from there rather than from an
             * empty effect. The name is derived from the entity, which is what keeps the world
             * registration one-per-emitter.
             */
            Vfx::ParticleEffect& particle_effect_for(EditorContext& context,
                                                     SushiEngine::Simulation::EntityId entity,
                                                     IWorldEditor& world)
            {
                // A scratch copy of the component's own effect: the widgets need a mutable
                // reference to bind to, and the world is written back once per changed frame
                // rather than through a setter per widget. Re-read whenever the selection moves,
                // so switching entities never carries the previous one's effect across.
                if (context.particle_effect_entity != entity)
                {
                    context.particle_effect_entity = entity;
                    context.particle_effect_scratch = world.particle_effect_source(entity);
                }
                return context.particle_effect_scratch;
            }

            /**
             * @brief The effect library: what is on disk, and the load/save that moves between it
             *        and the effect being edited.
             *
             * The listing is re-read on demand rather than watched — an author saves far less often
             * than the panel redraws, and a filesystem scan every frame would be the panel's
             * dominant cost for information that almost never changes.
             */
            /**
             * @brief The sprite-texture slot: an editable path, a load, and a clear.
             *
             * Unlike the material inspector's equivalent this needs no side table for the text
             * the author typed — a particle material stores its own path, because an effect
             * asset has to persist one, and the handle is derived from it rather than the other
             * way round.
             *
             * @param render The render module edited in place.
             * @param assets Library the path is loaded through; null disables loading.
             * @return true when the slot changed.
             */
            bool draw_particle_texture(Vfx::RenderModule& render,
                                       SushiEngine::Render::IAssetLibrary* assets)
            {
                bool changed = false;
                char buffer[512];
                std::snprintf(buffer, sizeof(buffer), "%s", render.texture_path.c_str());

                ImGui::SetNextItemWidth(-150.0f);
                const bool entered = ImGui::InputText("Texture", buffer, sizeof(buffer),
                                                      ImGuiInputTextFlags_EnterReturnsTrue);
                render.texture_path = buffer;
                ImGui::SameLine();
                const bool load = ImGui::SmallButton("Load");
                ImGui::SameLine();
                const bool clear = ImGui::SmallButton("Clear");
                ImGui::SameLine();
                ImGui::TextDisabled("%s", render.texture != Vfx::NO_PARTICLE_TEXTURE ? "set"
                                                                                     : "dot");

                if (clear)
                {
                    if (render.texture != Vfx::NO_PARTICLE_TEXTURE && assets != nullptr)
                        assets->release_texture(
                            static_cast<SushiEngine::Render::TextureId>(render.texture));
                    render.texture = Vfx::NO_PARTICLE_TEXTURE;
                    render.texture_path.clear();
                    return true;
                }
                if ((entered || load) && assets != nullptr)
                {
                    // Sprite sheets are authored in sRGB like any other colour map, so they decode
                    // the same way an albedo map does.
                    const SushiEngine::Render::TextureId loaded =
                        render.texture_path.empty()
                            ? SushiEngine::Render::INVALID_TEXTURE
                            : assets->load_texture(render.texture_path.c_str(),
                                                   SushiEngine::Render::TextureColorSpace::Srgb);
                    if (render.texture != Vfx::NO_PARTICLE_TEXTURE)
                        assets->release_texture(
                            static_cast<SushiEngine::Render::TextureId>(render.texture));
                    render.texture = loaded == SushiEngine::Render::INVALID_TEXTURE
                                         ? Vfx::NO_PARTICLE_TEXTURE
                                         : static_cast<std::uint32_t>(loaded);
                    changed = true;
                }
                return changed;
            }

            bool draw_effect_library(Vfx::ParticleEffect& target,
                                     SushiEngine::Render::IAssetLibrary* assets,
                                     EffectLibraryState& library)
            {
                bool loaded_one = false;
                std::vector<std::string>& files = library.files;
                bool& scanned = library.scanned;
                std::string& status = library.status;
                char (&name_buffer)[64] = library.name_buffer;

                if (!scanned)
                {
                    files = list_effect_files(EFFECT_LIBRARY_DIRECTORY);
                    scanned = true;
                }

                if (!ImGui::CollapsingHeader("Library", ImGuiTreeNodeFlags_DefaultOpen))
                    return false;

                ImGui::SetNextItemWidth(160.0f);
                ImGui::InputText("Name", name_buffer, IM_ARRAYSIZE(name_buffer));
                ImGui::SameLine();
                if (ImGui::Button("Save"))
                {
                    const std::string path = std::string(EFFECT_LIBRARY_DIRECTORY) + "/" +
                                             name_buffer + EFFECT_FILE_EXTENSION;
                    // The file keeps the asset's name; the emitter's own effect keeps the name the
                    // world knows it by, so saving a template never renames the live emitter.
                    Vfx::ParticleEffect asset = target;
                    asset.name = name_buffer;
                    status = save_effect(asset, path) ? "Saved " + path
                                                      : "Could not write " + path;
                    files = list_effect_files(EFFECT_LIBRARY_DIRECTORY);
                }
                ImGui::SameLine();
                if (ImGui::Button("Refresh"))
                {
                    files = list_effect_files(EFFECT_LIBRARY_DIRECTORY);
                    status.clear();
                }

                // Built-in starting points, so the library is useful before anything is saved.
                // Like a file, a template is copied in rather than referenced.
                std::size_t template_count = 0;
                const EffectTemplate* templates = built_in_effect_templates(template_count);
                for (std::size_t t = 0; t < template_count; ++t)
                {
                    if (t > 0)
                        ImGui::SameLine();
                    ImGui::PushID(static_cast<int>(t));
                    if (ImGui::SmallButton(templates[t].name))
                    {
                        const std::string keep = target.name;
                        target = templates[t].build();
                        target.name = keep;
                        std::snprintf(name_buffer, sizeof(name_buffer), "%s", templates[t].name);
                        loaded_one = true;
                        status = std::string("Started from ") + templates[t].name;
                    }
                    ImGui::PopID();
                }

                if (files.empty())
                {
                    ImGui::TextDisabled("No saved effects in %s yet.", EFFECT_LIBRARY_DIRECTORY);
                }
                else if (ImGui::BeginListBox("##effects", ImVec2(-FLT_MIN, 100.0f)))
                {
                    for (std::size_t i = 0; i < files.size(); ++i)
                    {
                        const std::string& file = files[i];
                        const std::string label =
                            std::filesystem::path(file).stem().string();
                        ImGui::PushID(static_cast<int>(i));
                        if (ImGui::Selectable(label.c_str()))
                        {
                            // A library entry is a **template**: it is copied into this emitter,
                            // not bound to it, so editing one emitter never changes another that
                            // started from the same file.
                            Vfx::ParticleEffect loaded;
                            if (load_effect(file, loaded))
                            {
                                // The file names its sprite textures by path; the handles it was
                                // written with belong to whichever session wrote it.
                                if (assets != nullptr)
                                    resolve_effect_textures(loaded, *assets);
                                const std::string keep = target.name;
                                std::snprintf(name_buffer, sizeof(name_buffer), "%s",
                                              loaded.name.c_str());
                                target = std::move(loaded);
                                target.name = keep;
                                loaded_one = true;
                                status = "Loaded " + file;
                            }
                            else
                            {
                                status = "Could not read " + file;
                            }
                        }
                        ImGui::PopID();
                    }
                    ImGui::EndListBox();
                }

                if (!status.empty())
                    ImGui::TextDisabled("%s", status.c_str());
                return loaded_one;
            }

            /**
             * @brief The emitter cycle as a bar: its duration, its bursts, and a draggable head.
             *
             * Dragging seeks the emission schedule — see @ref EffectPreview::seek for why the live
             * particles do not wind back with it.
             */
            void draw_effect_timeline(EffectPreview& preview)
            {
                // Read through the const overload: the mutable one marks the compiled effect stale,
                // and drawing a timeline is not an edit.
                const Vfx::ParticleEffect& effect =
                    static_cast<const EffectPreview&>(preview).effect();
                if (effect.emitters.empty())
                    return;
                const Vfx::EmitterDescriptor& emitter = effect.emitters[0];
                const float duration = emitter.duration > 0.0f ? emitter.duration : 1.0f;
                const float head = emitter.looping ? std::fmod(preview.time(), duration)
                                                   : std::min(preview.time(), duration);

                const ImVec2 origin = ImGui::GetCursorScreenPos();
                const float width = ImGui::GetContentRegionAvail().x;
                const float height = 24.0f;
                ImGui::InvisibleButton("##timeline", ImVec2(width, height));
                const bool active = ImGui::IsItemActive();

                ImDrawList* draw_list = ImGui::GetWindowDrawList();
                draw_list->AddRectFilled(origin, ImVec2(origin.x + width, origin.y + height),
                                         IM_COL32(38, 38, 44, 255), 3.0f);

                // Burst markers, so the schedule is visible rather than only listed.
                for (const Vfx::ParticleBurst& burst : emitter.spawn.bursts)
                {
                    const float t = duration > 0.0f ? burst.time / duration : 0.0f;
                    if (t < 0.0f || t > 1.0f)
                        continue;
                    const float x = origin.x + t * width;
                    draw_list->AddLine(ImVec2(x, origin.y + 2.0f),
                                       ImVec2(x, origin.y + height - 2.0f),
                                       IM_COL32(230, 180, 90, 255), 2.0f);
                }

                const float head_x = origin.x + (head / duration) * width;
                draw_list->AddLine(ImVec2(head_x, origin.y), ImVec2(head_x, origin.y + height),
                                   IM_COL32(240, 240, 240, 255), 2.0f);

                if (active && width > 0.0f)
                {
                    const float fraction =
                        (ImGui::GetIO().MousePos.x - origin.x) / width;
                    preview.seek(std::min(std::max(fraction, 0.0f), 1.0f) * duration);
                }

                ImGui::Text("t = %.2f s / %.2f s%s", static_cast<double>(head),
                            static_cast<double>(duration), emitter.looping ? " (loop)" : "");
            }
            /**
             * @brief The emitter's module stack drawn as a left-to-right node graph.
             *
             * A **presentation** of the authoring model, not a second one: the stack order
             * (spawn → shape → init → update → render) is the pipeline a particle actually goes
             * through, so the graph is that pipeline laid out rather than a free-form canvas whose
             * edges would have to be validated back into the same fixed order. Clicking a node
             * toggles the module it stands for; the parameters stay in the sections below, which is
             * where an author edits numbers.
             */
            void draw_effect_graph(Vfx::EmitterDescriptor& emitter)
            {
                if (!ImGui::CollapsingHeader("Graph"))
                    return;

                struct Node
                {
                    const char* label;
                    bool* enabled; // null for the stages that are always present
                };
                const Node nodes[] = {
                    {"Spawn", &emitter.spawn.enabled},
                    {"Shape", nullptr},
                    {"Init", nullptr},
                    {"Gravity", &emitter.gravity.enabled},
                    {"Drag", &emitter.drag.enabled},
                    {"Turbulence", &emitter.turbulence.enabled},
                    {"Collision", &emitter.collision.enabled},
                    {"Size/Life", &emitter.size_over_life.enabled},
                    {"Colour/Life", &emitter.color_over_life.enabled},
                    {"Render", nullptr},
                };
                constexpr int NODE_COUNT = IM_ARRAYSIZE(nodes);

                const float node_width = 92.0f;
                const float node_height = 30.0f;
                const float gap = 18.0f;
                const int per_row = 4;
                const int rows = (NODE_COUNT + per_row - 1) / per_row;

                const ImVec2 origin = ImGui::GetCursorScreenPos();
                const float height = rows * node_height + (rows - 1) * gap;
                ImGui::InvisibleButton("##graph", ImVec2(per_row * (node_width + gap), height));
                ImDrawList* draw_list = ImGui::GetWindowDrawList();
                const ImVec2 mouse = ImGui::GetIO().MousePos;
                const bool clicked = ImGui::IsItemHovered() && ImGui::IsMouseClicked(0);

                ImVec2 centres[NODE_COUNT];
                for (int i = 0; i < NODE_COUNT; ++i)
                {
                    const int row = i / per_row;
                    const int column = i % per_row;
                    const ImVec2 top_left(origin.x + column * (node_width + gap),
                                          origin.y + row * (node_height + gap));
                    const ImVec2 bottom_right(top_left.x + node_width, top_left.y + node_height);
                    centres[i] = ImVec2((top_left.x + bottom_right.x) * 0.5f,
                                        (top_left.y + bottom_right.y) * 0.5f);

                    const bool on = nodes[i].enabled == nullptr || *nodes[i].enabled;
                    const ImU32 fill = on ? IM_COL32(52, 74, 96, 255) : IM_COL32(40, 40, 46, 255);
                    const ImU32 border = on ? IM_COL32(120, 180, 230, 255) : IM_COL32(78, 78, 86, 255);
                    draw_list->AddRectFilled(top_left, bottom_right, fill, 4.0f);
                    draw_list->AddRect(top_left, bottom_right, border, 4.0f);
                    const ImVec2 text = ImGui::CalcTextSize(nodes[i].label);
                    draw_list->AddText(ImVec2(centres[i].x - text.x * 0.5f,
                                              centres[i].y - text.y * 0.5f),
                                       on ? IM_COL32(235, 240, 245, 255) : IM_COL32(140, 140, 148, 255),
                                       nodes[i].label);

                    if (clicked && nodes[i].enabled != nullptr && mouse.x >= top_left.x &&
                        mouse.x <= bottom_right.x && mouse.y >= top_left.y &&
                        mouse.y <= bottom_right.y)
                    {
                        *nodes[i].enabled = !*nodes[i].enabled;
                    }
                }

                // The links: consecutive stages, wrapping at the end of a row.
                for (int i = 0; i + 1 < NODE_COUNT; ++i)
                {
                    const bool wraps = (i % per_row) == per_row - 1;
                    const ImU32 colour = IM_COL32(96, 104, 118, 255);
                    if (!wraps)
                    {
                        draw_list->AddLine(ImVec2(centres[i].x + node_width * 0.5f, centres[i].y),
                                           ImVec2(centres[i + 1].x - node_width * 0.5f,
                                                  centres[i + 1].y),
                                           colour, 1.5f);
                    }
                    else
                    {
                        // Round the corner out past the row's right edge and back, so a wrapped
                        // link reads as continuing rather than as an unrelated stub.
                        const float turn = centres[i].x + node_width * 0.5f + gap * 0.5f;
                        draw_list->AddLine(ImVec2(centres[i].x + node_width * 0.5f, centres[i].y),
                                           ImVec2(turn, centres[i].y), colour, 1.5f);
                        draw_list->AddLine(ImVec2(turn, centres[i].y),
                                           ImVec2(turn, centres[i + 1].y), colour, 1.5f);
                        draw_list->AddLine(ImVec2(turn, centres[i + 1].y),
                                           ImVec2(centres[i + 1].x - node_width * 0.5f,
                                                  centres[i + 1].y),
                                           colour, 1.5f);
                    }
                }

                ImGui::TextDisabled("Click a stage to toggle it; edit its values below.");
            }
        } // namespace

        void draw_particle_system_component(EditorContext& context, IWorldEditor& world_ref,
                                            SushiEngine::Simulation::EntityId entity)
        {
            EffectPreview* preview = context.particle_preview;
            IWorldEditor* world = &world_ref;
            if (preview == nullptr)
            {
                ImGui::TextDisabled("No particle preview is available.");
                return;
            }

            Vfx::ParticleEffect& target = particle_effect_for(context, entity, *world);
            // The pre-frame shape of the effect, for the undo bracketing at the bottom:
            // these widgets edit the scratch directly and report no per-widget change,
            // so "did this frame edit the effect" is answered by comparing captures.
            const nlohmann::json effect_before = capture_effect(target);
            if (draw_effect_library(target, context.assets, context.panel_state.effect_library))
                context.particle_effect_dirty = true;

            // The preview shows the emitter where it actually is, so the isolated surface and the
            // Scene view do not disagree about position.
            preview->set_position(world->world_transform(entity).position);

            // No play/pause here: the transport belongs to the Preview surface. What stays is what
            // is specific to authoring — the scrubbable backend, the step, and the timeline.
            bool deterministic = preview->deterministic();
            if (ImGui::Checkbox("CPU (scrubbable)", &deterministic))
                preview->set_deterministic(deterministic);
            if (deterministic)
            {
                ImGui::SameLine();
                if (ImGui::Button("Step"))
                    preview->seek(preview->time() + EffectPreview::DETERMINISTIC_STEP);
            }
            bool scene_preview = preview->scene_preview();
            if (ImGui::Checkbox("Preview in Scene", &scene_preview))
                preview->set_scene_preview(scene_preview);
            ImGui::SameLine();
            ImGui::TextDisabled("(the emitter entity itself is always live in the Scene)");

            draw_effect_timeline(*preview);

            Vector3 position = preview->position();
            float position_values[3] = {static_cast<float>(position.x),
                                        static_cast<float>(position.y),
                                        static_cast<float>(position.z)};
            if (ImGui::DragFloat3("Emitter Position", position_values, 0.05f))
                preview->set_position(Vector3{position_values[0], position_values[1],
                                              position_values[2]});

            if (target.emitters.empty())
                return;
            Vfx::EmitterDescriptor& emitter = target.emitters[0];

            draw_effect_graph(emitter);

            ImGui::SeparatorText("Emission");
            ImGui::DragFloat("Rate /s", &emitter.spawn.rate_per_second, 1.0f, 0.0f, 5000.0f);
            int capacity = static_cast<int>(emitter.capacity);
            if (ImGui::DragInt("Capacity", &capacity, 64.0f, 1, 1 << 20))
                emitter.capacity = static_cast<std::uint32_t>(capacity < 1 ? 1 : capacity);

            ImGui::TextDisabled("Bursts (additive to the rate above)");
            int burst_to_remove = -1;
            for (std::size_t i = 0; i < emitter.spawn.bursts.size(); ++i)
            {
                Vfx::ParticleBurst& burst = emitter.spawn.bursts[i];
                ImGui::PushID(static_cast<int>(i));
                ImGui::SetNextItemWidth(90.0f);
                ImGui::DragFloat("##time", &burst.time, 0.05f, 0.0f, emitter.duration);
                ImGui::SameLine();
                ImGui::SetNextItemWidth(90.0f);
                int count = static_cast<int>(burst.count);
                if (ImGui::DragInt("##count", &count, 1.0f, 0, 100000))
                    burst.count = static_cast<std::uint32_t>(count < 0 ? 0 : count);
                ImGui::SameLine();
                if (ImGui::Button("Remove"))
                    burst_to_remove = static_cast<int>(i);
                ImGui::PopID();
            }
            if (burst_to_remove >= 0)
            {
                emitter.spawn.bursts.erase(emitter.spawn.bursts.begin() + burst_to_remove);
                context.particle_effect_dirty = true;
            }
            if (ImGui::Button("Add Burst"))
            {
                emitter.spawn.bursts.push_back(Vfx::ParticleBurst{0.0f, 10u});
                context.particle_effect_dirty = true;
            }

            ImGui::SeparatorText("Shape");
            const char* shape_names[] = {"Point", "Sphere", "Hemisphere", "Cone", "Box", "Circle"};
            int shape = static_cast<int>(emitter.shape.shape);
            if (ImGui::Combo("Shape", &shape, shape_names, IM_ARRAYSIZE(shape_names)))
                emitter.shape.shape = static_cast<Vfx::EmitterShape>(shape);
            ImGui::DragFloat("Radius", &emitter.shape.radius, 0.01f, 0.0f, 20.0f);
            ImGui::SliderAngle("Cone Angle", &emitter.shape.cone_angle_radians, 0.0f, 90.0f);

            ImGui::SeparatorText("Birth");
            ImGui::DragFloatRange2("Lifetime", &emitter.init.lifetime_min, &emitter.init.lifetime_max,
                                   0.05f, 0.0f, 30.0f);
            ImGui::DragFloatRange2("Speed", &emitter.init.speed_min, &emitter.init.speed_max, 0.05f,
                                   0.0f, 50.0f);
            ImGui::DragFloatRange2("Size", &emitter.init.size_min, &emitter.init.size_max, 0.005f,
                                   0.0f, 5.0f);
            float birth_color[3] = {static_cast<float>(emitter.init.color.x),
                                    static_cast<float>(emitter.init.color.y),
                                    static_cast<float>(emitter.init.color.z)};
            if (ImGui::ColorEdit3("Base Colour", birth_color))
                emitter.init.color = Vector3{birth_color[0], birth_color[1], birth_color[2]};

            ImGui::SeparatorText("Forces");
            ImGui::Checkbox("Gravity", &emitter.gravity.enabled);
            if (emitter.gravity.enabled)
            {
                float gravity[3] = {static_cast<float>(emitter.gravity.acceleration.x),
                                    static_cast<float>(emitter.gravity.acceleration.y),
                                    static_cast<float>(emitter.gravity.acceleration.z)};
                if (ImGui::DragFloat3("Acceleration", gravity, 0.1f))
                    emitter.gravity.acceleration = Vector3{gravity[0], gravity[1], gravity[2]};
            }
            ImGui::Checkbox("Drag", &emitter.drag.enabled);
            if (emitter.drag.enabled)
                ImGui::DragFloat("Drag Coefficient", &emitter.drag.coefficient, 0.01f, 0.0f, 10.0f);
            ImGui::Checkbox("Turbulence", &emitter.turbulence.enabled);
            if (emitter.turbulence.enabled)
            {
                ImGui::DragFloat("Frequency", &emitter.turbulence.frequency, 0.01f, 0.0f, 10.0f);
                ImGui::DragFloat("Amplitude", &emitter.turbulence.amplitude, 0.05f, 0.0f, 50.0f);
            }

            ImGui::Checkbox("Depth collision", &emitter.collision.enabled);
            if (emitter.collision.enabled)
            {
                ImGui::DragFloat("Restitution", &emitter.collision.restitution, 0.01f, 0.0f, 1.0f);
                ImGui::DragFloat("Friction", &emitter.collision.friction, 0.01f, 0.0f, 1.0f);
                ImGui::DragFloat("Thickness", &emitter.collision.thickness, 0.02f, 0.001f, 10.0f);
                ImGui::TextDisabled("Screen-space: only surfaces the camera can see.");
            }

            ImGui::SeparatorText("Force Fields");
            if (ImGui::Button("Add Field") &&
                emitter.force_fields.size() < Vfx::MAX_FORCE_FIELDS)
            {
                Vfx::ForceFieldModule field;
                field.enabled = true;
                emitter.force_fields.push_back(field);
            }
            for (std::size_t f = 0; f < emitter.force_fields.size(); ++f)
            {
                Vfx::ForceFieldModule& field = emitter.force_fields[f];
                ImGui::PushID(static_cast<int>(f));
                ImGui::Checkbox("##enabled", &field.enabled);
                ImGui::SameLine();
                const char* kind_names[] = {"Point", "Vortex", "Drag"};
                int kind = static_cast<int>(field.kind);
                ImGui::SetNextItemWidth(120.0f);
                if (ImGui::Combo("##kind", &kind, kind_names, IM_ARRAYSIZE(kind_names)))
                    field.kind = static_cast<Vfx::ForceFieldKind>(kind);
                ImGui::SameLine();
                if (ImGui::SmallButton("Remove"))
                {
                    emitter.force_fields.erase(emitter.force_fields.begin() +
                                               static_cast<std::ptrdiff_t>(f));
                    ImGui::PopID();
                    break;
                }
                if (field.enabled)
                {
                    float position[3] = {static_cast<float>(field.position.x),
                                         static_cast<float>(field.position.y),
                                         static_cast<float>(field.position.z)};
                    if (ImGui::DragFloat3("Position", position, 0.05f))
                        field.position = Vector3{position[0], position[1], position[2]};
                    if (field.kind == Vfx::ForceFieldKind::Vortex)
                    {
                        float axis[3] = {static_cast<float>(field.axis.x),
                                         static_cast<float>(field.axis.y),
                                         static_cast<float>(field.axis.z)};
                        if (ImGui::DragFloat3("Axis", axis, 0.02f, -1.0f, 1.0f))
                            field.axis = Vector3{axis[0], axis[1], axis[2]};
                    }
                    ImGui::DragFloat("Strength", &field.strength, 0.1f, -100.0f, 100.0f);
                    ImGui::DragFloat("Radius", &field.radius, 0.05f, 0.001f, 100.0f);
                    ImGui::DragFloat("Falloff", &field.falloff, 0.05f, 0.0f, 8.0f);
                }
                ImGui::PopID();
            }

            ImGui::SeparatorText("Over Life");
            ImGui::Checkbox("Size over life", &emitter.size_over_life.enabled);
            ImGui::Checkbox("Colour over life", &emitter.color_over_life.enabled);
            if (emitter.color_over_life.enabled &&
                emitter.color_over_life.gradient.color_keys().size() >= 2)
            {
                std::vector<Vfx::ColorKey>& keys = emitter.color_over_life.gradient.color_keys();
                float start[3] = {static_cast<float>(keys.front().color.x),
                                  static_cast<float>(keys.front().color.y),
                                  static_cast<float>(keys.front().color.z)};
                if (ImGui::ColorEdit3("Start", start))
                    keys.front().color = Vector3{start[0], start[1], start[2]};
                float end[3] = {static_cast<float>(keys.back().color.x),
                                static_cast<float>(keys.back().color.y),
                                static_cast<float>(keys.back().color.z)};
                if (ImGui::ColorEdit3("End", end))
                    keys.back().color = Vector3{end[0], end[1], end[2]};
            }

            ImGui::SeparatorText("Render");
            const char* blend_names[] = {"Additive", "Alpha", "Premultiplied"};
            int blend = static_cast<int>(emitter.render.blend);
            if (ImGui::Combo("Blend", &blend, blend_names, IM_ARRAYSIZE(blend_names)))
                emitter.render.blend = static_cast<Vfx::BlendMode>(blend);
            const char* alignment_names[] = {"Face Camera", "Velocity Stretched", "Ribbon", "Mesh"};
            int alignment = static_cast<int>(emitter.render.alignment);
            if (ImGui::Combo("Alignment", &alignment, alignment_names,
                             IM_ARRAYSIZE(alignment_names)))
                emitter.render.alignment = static_cast<Vfx::RenderAlignment>(alignment);
            if (emitter.render.alignment == Vfx::RenderAlignment::VelocityStretched)
                ImGui::DragFloat("Stretch", &emitter.render.velocity_stretch, 0.005f, 0.0f, 1.0f,
                                 "%.3f m per m/s");
            if (emitter.render.alignment == Vfx::RenderAlignment::Ribbon)
                ImGui::TextDisabled("Ribbons trail the last 8 simulated positions.");
            if (emitter.render.alignment == Vfx::RenderAlignment::Mesh)
            {
                int mesh = static_cast<int>(emitter.render.mesh);
                if (ImGui::InputInt("Mesh", &mesh))
                    emitter.render.mesh = static_cast<std::uint32_t>(mesh < 0 ? 0 : mesh);
                ImGui::TextDisabled("Mesh particles draw solid, with the opaque geometry.");
            }

            // The particle material. Sprite-only by design: a puff has no roughness or normal
            // map, and a particle that needs a real surface is a Mesh-aligned one.
            ImGui::SeparatorText("Material");
            if (context.assets == nullptr)
                ImGui::TextDisabled("No asset library; textures cannot be loaded.");
            if (draw_particle_texture(emitter.render, context.assets))
                context.particle_effect_dirty = true;
            if (emitter.render.alignment == Vfx::RenderAlignment::Mesh)
                ImGui::TextDisabled("Mesh particles take their look from the mesh, not this.");

            int flipbook[2] = {static_cast<int>(emitter.render.flipbook_columns),
                               static_cast<int>(emitter.render.flipbook_rows)};
            if (ImGui::DragInt2("Flipbook (cols, rows)", flipbook, 0.1f, 1, 32))
            {
                emitter.render.flipbook_columns =
                    static_cast<std::uint32_t>(flipbook[0] < 1 ? 1 : flipbook[0]);
                emitter.render.flipbook_rows =
                    static_cast<std::uint32_t>(flipbook[1] < 1 ? 1 : flipbook[1]);
            }
            if (emitter.render.flipbook_rows * emitter.render.flipbook_columns > 1)
                ImGui::TextDisabled("The cell advances with the particle's normalised age.");

            ImGui::Checkbox("Soft Particles", &emitter.render.soft_particles);
            if (emitter.render.soft_particles)
            {
                ImGui::SameLine();
                ImGui::SetNextItemWidth(120.0f);
                ImGui::DragFloat("Fade (m)", &emitter.render.soft_fade_distance, 0.01f, 0.0f, 10.0f,
                                 "%.2f");
            }
            ImGui::Checkbox("Lit", &emitter.render.lit);
            ImGui::SameLine();
            ImGui::TextDisabled("(sun + shadow, clustered lights, SH ambient)");

            // Push the edit into the world only when something actually changed, and in place, so
            // dragging a slider does not register a new effect once per frame. The emitter's index
            // is set every time because it is cheap and covers the first edit after a reload.
            // While a widget is active its value has already changed, so registering on those
            // frames covers a drag; the dirty flag covers the changes no widget reports (a library
            // load, an emitter's first appearance).
            //
            // Undo bracketing: a drag is one begin_change/end_change step (armed the frame the
            // scratch first diverges, committed when the widget releases); a discrete change with
            // no active widget (library load, burst add/remove) is one record. Both snapshot the
            // world before set_particle_effect_source writes the edit into it.
            const bool particle_item_active = ImGui::IsAnyItemActive();
            const bool effect_edited = context.particle_effect_dirty ||
                                       capture_effect(target) != effect_before;
            if (effect_edited && !context.particle_effect_change_active)
            {
                if (particle_item_active)
                {
                    context.history.begin_change(*world);
                    context.particle_effect_change_active = true;
                }
                else if (context.particle_effect_dirty)
                    context.history.record(*world);
            }
            if (particle_item_active || context.particle_effect_dirty)
            {
                context.particle_effect_dirty = false;
                world->set_particle_effect_source(entity, target);
            }
            if (!particle_item_active && context.particle_effect_change_active)
            {
                context.history.end_change();
                context.particle_effect_change_active = false;
            }

            // The Preview surface mirrors whatever is being edited, so the isolated view and the
            // scene show the same thing without this section owning a second effect.
            preview->set_effect(target);
        }
    } // namespace Editor
} // namespace SushiEngine
