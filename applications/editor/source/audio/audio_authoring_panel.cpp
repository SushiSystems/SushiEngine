/**************************************************************************/
/* audio_authoring_panel.cpp                                              */
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

#include "audio_authoring_panel.hpp"

#include <cstdint>
#include <string>
#include <vector>

#include <imgui.h>

#include <SushiEngine/audio/audio.hpp>

#include "audio_editor_system.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        namespace
        {
            const char* kind_name(Audio::ContainerKind kind)
            {
                switch (kind)
                {
                    case Audio::ContainerKind::Sound: return "Sound";
                    case Audio::ContainerKind::Random: return "Random";
                    case Audio::ContainerKind::Sequence: return "Sequence";
                    case Audio::ContainerKind::Blend: return "Blend";
                    case Audio::ContainerKind::Switch: return "Switch";
                    case Audio::ContainerKind::Layer: return "Layer";
                }
                return "?";
            }

            const char* media_name(const Audio::AudioAuthoringProject& project, std::uint32_t id)
            {
                for (const Audio::AuthoredMedia& m : project.media())
                    if (m.id == id)
                        return m.name.c_str();
                return "<unassigned>";
            }

            // Draws one node and its subtree. Returns nothing; edits the project in place.
            void draw_node(Audio::AudioAuthoringProject& project, AudioEditorSystem& audio, int node)
            {
                if (node < 0 || static_cast<std::size_t>(node) >= project.nodes().size())
                    return;
                Audio::AuthoredNode& n = project.nodes()[static_cast<std::size_t>(node)];
                ImGui::PushID(node);

                if (n.kind == Audio::ContainerKind::Sound)
                {
                    ImGui::BulletText("Sound: %s", media_name(project, n.media_id));
                    ImGui::SameLine();
                    if (ImGui::SmallButton("Play"))
                        audio.preview(n.media_id, 1.0f);
                }
                else
                {
                    const bool open = ImGui::TreeNodeEx(kind_name(n.kind),
                                                        ImGuiTreeNodeFlags_DefaultOpen);
                    if (n.kind == Audio::ContainerKind::Random ||
                        n.kind == Audio::ContainerKind::Layer)
                    {
                        ImGui::SameLine();
                        ImGui::SetNextItemWidth(120.0f);
                        ImGui::DragFloat("weight", &n.weight, 0.01f, 0.0f, 8.0f);
                    }
                    if (open)
                    {
                        // A snapshot of child handles: adding a child below must not disturb the walk.
                        const std::vector<int> children = n.children;
                        for (int c : children)
                            draw_node(project, audio, c);

                        if (ImGui::SmallButton("+ Sound") && !project.media().empty())
                        {
                            const int s = project.create_sound(project.media().front().id);
                            project.add_child(node, s);
                        }
                        ImGui::SameLine();
                        if (ImGui::SmallButton("+ Random"))
                            project.add_child(node,
                                              project.create_container(Audio::ContainerKind::Random));
                        ImGui::SameLine();
                        if (ImGui::SmallButton("+ Layer"))
                            project.add_child(node,
                                              project.create_container(Audio::ContainerKind::Layer));
                        ImGui::TreePop();
                    }
                }
                ImGui::PopID();
            }
        } // namespace

        void draw_audio_authoring_panel(Audio::AudioAuthoringProject& project,
                                        AudioEditorSystem& audio, bool* open)
        {
            if (open != nullptr && !*open)
                return;
            if (!ImGui::Begin("Audio Authoring", open))
            {
                ImGui::End();
                return;
            }

            if (ImGui::CollapsingHeader("Media", ImGuiTreeNodeFlags_DefaultOpen))
            {
                for (const Audio::AuthoredMedia& m : project.media())
                    ImGui::Text("#%u  %s  (%u ch, %u Hz)", m.id, m.name.c_str(), m.channels,
                                m.sample_rate);
            }

            if (ImGui::CollapsingHeader("Events", ImGuiTreeNodeFlags_DefaultOpen))
            {
                for (Audio::AuthoredEvent& ev : project.events())
                {
                    ImGui::PushID(static_cast<int>(ev.id));
                    if (ImGui::TreeNodeEx(ev.name.c_str(), ImGuiTreeNodeFlags_DefaultOpen))
                    {
                        draw_node(project, audio, ev.root);
                        ImGui::TreePop();
                    }
                    ImGui::PopID();
                }
            }

            ImGui::Separator();
            ImGui::TextDisabled("%zu nodes  ·  %zu events  ·  %zu media", project.nodes().size(),
                                project.events().size(), project.media().size());

            ImGui::End();
        }
    } // namespace Editor
} // namespace SushiEngine
