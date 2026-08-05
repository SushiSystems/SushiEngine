/**************************************************************************/
/* project_picker.cpp                                                     */
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

#include "project_picker.hpp"

#include "../core/editor_context.hpp"

#include <filesystem>
#include <system_error>
#include <vector>

#include <imgui.h>
#include <misc/cpp/imgui_stdlib.h>

namespace SushiEngine
{
    namespace Editor
    {
        namespace fs = std::filesystem;

        // Forward declaration for request_switch_project (defined in scene_commands.hpp, Task 2)
        void request_switch_project(EditorContext& context, const std::string& path);

        void draw_project_picker(EditorContext& context)
        {
            if (!context.show_project_picker)
                return;

            const char* title = context.project_picker_mode == EditorContext::ProjectPickerMode::New
                                     ? "New Project"
                                     : "Load Project";
            ImGui::OpenPopup(title);
            if (!ImGui::BeginPopupModal(title, &context.show_project_picker,
                                        ImGuiWindowFlags_AlwaysAutoResize))
            {
                ImGui::EndPopup();
                return;
            }

            const fs::path current(context.project_picker_directory);
            ImGui::TextDisabled("%s", current.string().c_str());
            ImGui::SameLine();
            if (ImGui::SmallButton("Up"))
                context.project_picker_directory = current.parent_path().string();
            ImGui::Separator();

            ImGui::BeginChild("project_picker_list", ImVec2(360.0f, 220.0f), true);
            std::error_code ec;
            std::vector<fs::directory_entry> directories;
            for (const auto& entry : fs::directory_iterator(current, ec))
            {
                std::error_code entry_ec;
                if (entry.is_directory(entry_ec) && !entry_ec)
                    directories.push_back(entry);
            }
            for (const fs::directory_entry& entry : directories)
            {
                const std::string name = entry.path().filename().string();
                if (ImGui::Selectable(name.c_str()))
                    context.project_picker_directory = entry.path().string();
            }
            ImGui::EndChild();

            if (context.project_picker_mode == EditorContext::ProjectPickerMode::New)
            {
                ImGui::SetNextItemWidth(-FLT_MIN);
                ImGui::InputTextWithHint("##new_project_name", "Folder name",
                                         &context.project_picker_new_folder_name);
                ImGui::BeginDisabled(context.project_picker_new_folder_name.empty());
                if (ImGui::Button("Create & Select"))
                {
                    const fs::path target =
                        current / context.project_picker_new_folder_name;
                    std::error_code create_ec;
                    fs::create_directories(target, create_ec);
                    if (!create_ec)
                    {
                        request_switch_project(context, target.string());
                        ImGui::CloseCurrentPopup();
                        context.show_project_picker = false;
                    }
                }
                ImGui::EndDisabled();
            }
            else
            {
                if (ImGui::Button("Select This Folder"))
                {
                    request_switch_project(context, current.string());
                    ImGui::CloseCurrentPopup();
                    context.show_project_picker = false;
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("Cancel"))
            {
                ImGui::CloseCurrentPopup();
                context.show_project_picker = false;
            }

            ImGui::EndPopup();
        }
    } // namespace Editor
} // namespace SushiEngine
