/**************************************************************************/
/* animation_panel.cpp                                                    */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/* Licensed under the Apache License, Version 2.0 (the "License");        */
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

#include "animation_panel.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include <imgui.h>

#include <SushiEngine/animation/clip_blob.hpp>
#include <SushiEngine/animation/keyframe.hpp>

namespace SushiEngine
{
    namespace Editor
    {
        namespace
        {
            using SushiEngine::Animation::ClipAuthoring;
            using SushiEngine::Animation::ClipDesc;
            using SushiEngine::Animation::JointChannels;
            using SushiEngine::Animation::Quaternionf;
            using SushiEngine::Animation::Vector3f;
            using SushiEngine::Simulation::EntityId;
            using SushiEngine::Simulation::EntityTransform;
            using SushiEngine::Simulation::IWorldEditor;
            using SushiEngine::Simulation::NULL_ENTITY;

            struct AnimationState
            {
                float time = 0.0f;
                float sample_rate = 30.0f;
                float length = 3.0f;
                bool playing = false;
                bool recording = false;
                char save_path[256] = "clip.sushianim";
                std::string status;

                EntityId target = NULL_ENTITY; /**< The Hierarchy entity being animated. */
                std::string target_name;
                JointChannels channels; /**< Position/rotation/scale curves of the target transform. */
                bool have_last = false;
                EntityTransform last_seen{}; /**< The transform read last frame (change detection). */

                int selected_row = 0;
                int selected_key = -1;
            };

            AnimationState& state()
            {
                static AnimationState instance;
                return instance;
            }

            IWorldEditor* world_of(EditorContext& context)
            {
                return context.simulation != nullptr ? &context.simulation->world() : nullptr;
            }

            bool has_keys(const AnimationState& s)
            {
                return !s.channels.translation_x.empty() || !s.channels.rotation.empty() ||
                       !s.channels.scale_x.empty();
            }

            // Whether two transforms differ enough to count as the user having moved the object.
            bool transform_changed(const EntityTransform& a, const EntityTransform& b)
            {
                const auto d = [](double x, double y) { return std::fabs(x - y) > 1e-4; };
                if (d(a.position.x, b.position.x) || d(a.position.y, b.position.y) ||
                    d(a.position.z, b.position.z))
                    return true;
                if (d(a.scale.x, b.scale.x) || d(a.scale.y, b.scale.y) || d(a.scale.z, b.scale.z))
                    return true;
                const double dot = a.rotation.x * b.rotation.x + a.rotation.y * b.rotation.y +
                                   a.rotation.z * b.rotation.z + a.rotation.w * b.rotation.w;
                return std::fabs(dot) < 0.99999;
            }

            // Binds the panel to an entity, clearing the curves and defaulting to its current pose.
            void set_target(AnimationState& s, IWorldEditor& world, EntityId id)
            {
                s.target = id;
                s.target_name = world.name(id);
                s.channels = JointChannels{};
                const EntityTransform t = world.transform(id);
                s.channels.default_translation =
                    Vector3f{static_cast<float>(t.position.x), static_cast<float>(t.position.y),
                             static_cast<float>(t.position.z)};
                s.channels.default_rotation =
                    Quaternionf{static_cast<float>(t.rotation.x), static_cast<float>(t.rotation.y),
                                static_cast<float>(t.rotation.z), static_cast<float>(t.rotation.w)};
                s.channels.default_scale = Vector3f{static_cast<float>(t.scale.x),
                                                    static_cast<float>(t.scale.y),
                                                    static_cast<float>(t.scale.z)};
                s.have_last = false;
                s.selected_key = -1;
            }

            void key_transform(AnimationState& s, const EntityTransform& t, float time)
            {
                s.channels.translation_x.insert(time, static_cast<float>(t.position.x));
                s.channels.translation_y.insert(time, static_cast<float>(t.position.y));
                s.channels.translation_z.insert(time, static_cast<float>(t.position.z));
                s.channels.rotation.insert(
                    time, Quaternionf{static_cast<float>(t.rotation.x), static_cast<float>(t.rotation.y),
                                      static_cast<float>(t.rotation.z),
                                      static_cast<float>(t.rotation.w)});
                s.channels.scale_x.insert(time, static_cast<float>(t.scale.x));
                s.channels.scale_y.insert(time, static_cast<float>(t.scale.y));
                s.channels.scale_z.insert(time, static_cast<float>(t.scale.z));
            }

            EntityTransform evaluate(const AnimationState& s, float time)
            {
                const Vector3f translation = s.channels.translation_at(time);
                const Quaternionf rotation = SushiEngine::normalize(s.channels.rotation_at(time));
                const Vector3f scale = s.channels.scale_at(time);
                EntityTransform out;
                out.position = SushiEngine::Vector3{translation.x, translation.y, translation.z};
                out.rotation =
                    SushiEngine::Quaternion{rotation.x, rotation.y, rotation.z, rotation.w};
                out.scale = SushiEngine::Vector3{scale.x, scale.y, scale.z};
                return out;
            }

            std::string bake_to_disk(const AnimationState& s)
            {
                ClipAuthoring authoring;
                authoring.joints.assign(1, s.channels);
                ClipDesc dense;
                if (!authoring.bake(s.sample_rate, dense))
                    return "Bake failed.";
                std::vector<std::byte> blob;
                if (!SushiEngine::Animation::build_clip_blob(dense, blob))
                    return "Cook failed.";
                std::ofstream file(s.save_path, std::ios::binary);
                if (!file)
                    return std::string("Cannot open ") + s.save_path;
                file.write(reinterpret_cast<const char*>(blob.data()),
                           static_cast<std::streamsize>(blob.size()));
                char message[256];
                std::snprintf(message, sizeof(message), "Baked %u frames (%zu bytes) to %s",
                              dense.frame_count, blob.size(), s.save_path);
                return message;
            }

            // ---- The dope-sheet timeline ------------------------------------------------

            struct DopeRow
            {
                const char* name;
                std::vector<float> key_times;
            };

            void draw_timeline(AnimationState& s, const std::vector<DopeRow>& rows,
                               const EntityTransform& current, IWorldEditor* world)
            {
                ImGui::BeginChild("timeline", ImVec2(0.0f, 0.0f), true);
                const ImVec2 origin = ImGui::GetCursorScreenPos();
                const float width = std::max(ImGui::GetContentRegionAvail().x, 80.0f);
                const float ruler = 18.0f;
                const float row_height = 24.0f;
                const int count = static_cast<int>(rows.size());
                const float height = ruler + row_height * static_cast<float>(std::max(count, 1));
                ImDrawList* draw = ImGui::GetWindowDrawList();
                const auto time_to_x = [&](float t) { return origin.x + (t / s.length) * width; };
                const auto x_to_time = [&](float x)
                { return std::max(0.0f, std::min(s.length, (x - origin.x) / width * s.length)); };

                draw->AddRectFilled(origin, ImVec2(origin.x + width, origin.y + height),
                                    IM_COL32(30, 30, 34, 255));
                for (int i = 0; i < count; ++i)
                {
                    const float y = origin.y + ruler + row_height * static_cast<float>(i);
                    if (i == s.selected_row)
                        draw->AddRectFilled(ImVec2(origin.x, y),
                                            ImVec2(origin.x + width, y + row_height),
                                            IM_COL32(52, 58, 72, 255));
                    draw->AddLine(ImVec2(origin.x, y + row_height),
                                  ImVec2(origin.x + width, y + row_height), IM_COL32(48, 48, 52, 255));
                    draw->AddText(ImVec2(origin.x + 4.0f, y + 4.0f), IM_COL32(210, 210, 210, 255),
                                  rows[i].name);
                }
                for (int t = 0; t <= static_cast<int>(std::ceil(s.length)); ++t)
                {
                    const float x = time_to_x(static_cast<float>(t));
                    draw->AddLine(ImVec2(x, origin.y), ImVec2(x, origin.y + height),
                                  IM_COL32(58, 58, 64, 255));
                    char label[16];
                    std::snprintf(label, sizeof(label), "%ds", t);
                    draw->AddText(ImVec2(x + 2.0f, origin.y + 2.0f), IM_COL32(150, 150, 150, 255),
                                  label);
                }

                ImGui::InvisibleButton("canvas", ImVec2(width, height));
                const bool hovered = ImGui::IsItemHovered();
                const ImVec2 mouse = ImGui::GetIO().MousePos;
                const float pick = 6.0f;

                if (hovered && mouse.y < origin.y + ruler && ImGui::IsMouseDown(ImGuiMouseButton_Left))
                    s.time = x_to_time(mouse.x);

                if (hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left) &&
                    mouse.y >= origin.y + ruler)
                {
                    const int row = static_cast<int>((mouse.y - origin.y - ruler) / row_height);
                    if (row >= 0 && row < count)
                    {
                        s.selected_row = row;
                        int hit = -1;
                        for (int k = 0; k < static_cast<int>(rows[row].key_times.size()); ++k)
                            if (std::fabs(time_to_x(rows[row].key_times[k]) - mouse.x) < pick)
                            {
                                hit = k;
                                break;
                            }
                        if (hit >= 0)
                            s.selected_key = hit;
                        else if (world != nullptr)
                        {
                            // Add a key at the click time with the object's current transform.
                            s.time = x_to_time(mouse.x);
                            key_transform(s, current, s.time);
                        }
                    }
                }
                if (hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Right) &&
                    mouse.y >= origin.y + ruler)
                {
                    const int row = static_cast<int>((mouse.y - origin.y - ruler) / row_height);
                    if (row >= 0 && row < count)
                        for (float key_time : rows[row].key_times)
                            if (std::fabs(time_to_x(key_time) - mouse.x) < pick)
                            {
                                s.channels.translation_x.remove_at(key_time);
                                s.channels.translation_y.remove_at(key_time);
                                s.channels.translation_z.remove_at(key_time);
                                s.channels.rotation.remove_at(key_time);
                                s.channels.scale_x.remove_at(key_time);
                                s.channels.scale_y.remove_at(key_time);
                                s.channels.scale_z.remove_at(key_time);
                                break;
                            }
                }

                for (int i = 0; i < count; ++i)
                {
                    const float y = origin.y + ruler + row_height * static_cast<float>(i) +
                                    row_height * 0.5f;
                    for (float key_time : rows[i].key_times)
                    {
                        const float x = time_to_x(key_time);
                        const float r = 4.0f;
                        draw->AddQuadFilled(ImVec2(x, y - r), ImVec2(x + r, y), ImVec2(x, y + r),
                                            ImVec2(x - r, y), IM_COL32(120, 190, 255, 255));
                    }
                }
                const float px = time_to_x(s.time);
                draw->AddLine(ImVec2(px, origin.y), ImVec2(px, origin.y + height),
                              IM_COL32(255, 80, 80, 255), 1.5f);
                ImGui::EndChild();
            }

            void draw_transport(AnimationState& s, bool has_target)
            {
                if (ImGui::Button(s.playing ? "Pause" : "Play"))
                    s.playing = !s.playing;
                ImGui::SameLine();
                if (ImGui::Button("Stop"))
                {
                    s.playing = false;
                    s.time = 0.0f;
                }
                ImGui::SameLine();
                ImGui::SetNextItemWidth(240.0f);
                ImGui::SliderFloat("Time", &s.time, 0.0f, s.length, "%.3f s");
                ImGui::SameLine();
                ImGui::SetNextItemWidth(80.0f);
                ImGui::DragFloat("Rate", &s.sample_rate, 1.0f, 1.0f, 240.0f, "%.0f Hz");
                ImGui::SameLine();
                ImGui::SetNextItemWidth(80.0f);
                ImGui::DragFloat("Len", &s.length, 0.1f, 0.1f, 120.0f, "%.1f s");

                if (!has_target)
                    ImGui::BeginDisabled();
                // The record button glows red while armed.
                if (s.recording)
                    ImGui::PushStyleColor(ImGuiCol_Button, IM_COL32(180, 40, 40, 255));
                if (ImGui::Button(s.recording ? "● Recording" : "● Record"))
                    s.recording = !s.recording;
                if (s.recording)
                    ImGui::PopStyleColor();
                if (!has_target)
                    ImGui::EndDisabled();

                ImGui::SameLine();
                ImGui::SetNextItemWidth(200.0f);
                ImGui::InputText("##save", s.save_path, sizeof(s.save_path));
                ImGui::SameLine();
                if (ImGui::Button("Bake .sushianim"))
                    s.status = bake_to_disk(s);
                if (!s.status.empty())
                    ImGui::TextUnformatted(s.status.c_str());
            }
        } // namespace

        void draw_animation_panel(EditorContext& context)
        {
            if (!context.panels.animation)
                return;
            if (!ImGui::Begin("Animation", &context.panels.animation))
            {
                ImGui::End();
                return;
            }

            AnimationState& s = state();
            IWorldEditor* world = world_of(context);
            const EntityId id = context.selected_entity;
            const bool has_target =
                world != nullptr && id != NULL_ENTITY && world->exists(id);

            if (has_target && id != s.target)
                set_target(s, *world, id);
            if (!has_target)
            {
                s.target = NULL_ENTITY;
                s.recording = false;
            }

            if (s.playing)
            {
                s.time += ImGui::GetIO().DeltaTime;
                if (s.time > s.length)
                    s.time = std::fmod(s.time, s.length);
            }

            if (has_target)
            {
                ImGui::Text("Target: %s", s.target_name.c_str());
                if (s.recording)
                    ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f),
                                       "Recording — move the object to key it at the playhead.");
                else
                    ImGui::TextDisabled("Not recording — scrubbing drives the object from its keys.");
            }
            else
            {
                ImGui::TextDisabled("Select an object in the Hierarchy to animate it.");
            }
            ImGui::Separator();

            draw_transport(s, has_target);
            ImGui::Separator();

            EntityTransform current{};
            if (has_target)
            {
                current = world->transform(id);

                // The record / preview loop.
                if (s.recording)
                {
                    if (s.have_last && !transform_changed(current, s.last_seen) && has_keys(s))
                    {
                        // The user is not moving it: show the recorded animation at the playhead.
                        const EntityTransform posed = evaluate(s, s.time);
                        world->set_transform(id, posed);
                        s.last_seen = posed;
                    }
                    else
                    {
                        // First frame, or the user moved it: capture the pose at the playhead.
                        key_transform(s, current, s.time);
                        s.last_seen = current;
                    }
                    s.have_last = true;
                }
                else if (has_keys(s))
                {
                    // Preview: drive the object from its keys as the timeline is scrubbed / played.
                    world->set_transform(id, evaluate(s, s.time));
                }
            }

            std::vector<DopeRow> rows = {
                {"Position", {}}, {"Rotation", {}}, {"Scale", {}}};
            for (const auto& key : s.channels.translation_x.keys)
                rows[0].key_times.push_back(key.time);
            for (const auto& key : s.channels.rotation.keys)
                rows[1].key_times.push_back(key.time);
            for (const auto& key : s.channels.scale_x.keys)
                rows[2].key_times.push_back(key.time);
            draw_timeline(s, rows, current, has_target ? world : nullptr);

            ImGui::End();
        }
    } // namespace Editor
} // namespace SushiEngine
