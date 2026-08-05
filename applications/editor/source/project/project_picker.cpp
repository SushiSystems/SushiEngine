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
#include "../scene/scene_commands.hpp"

#include <cstdlib>
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

        std::string default_projects_root()
        {
            fs::path home;
#ifdef _WIN32
            char* value = nullptr;
            std::size_t length = 0;
            if (_dupenv_s(&value, &length, "USERPROFILE") == 0 && value != nullptr)
            {
                home = value;
                std::free(value);
            }
#else
            if (const char* value = std::getenv("HOME"))
                home = value;
#endif
            fs::path root = !home.empty() ? home / "sushiengine" / "project" : fs::current_path();
            std::error_code ec;
            fs::create_directories(root, ec);
            return root.string();
        }

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
                return;

            const fs::path current(context.project_picker_directory);
            ImGui::TextDisabled("%s", current.string().c_str());
            ImGui::SameLine();
            if (ImGui::SmallButton("Up"))
                context.project_picker_directory = current.parent_path().string();
            ImGui::Separator();

            ImGui::BeginChild("project_picker_list", ImVec2(360.0f, 220.0f), true);
            std::error_code ec;
            std::vector<fs::directory_entry> directories;
            // Explicit iterator form: the range-for's implicit operator++ is the throwing
            // overload, so a directory that becomes unreadable mid-browse would otherwise
            // throw std::filesystem::filesystem_error out of the middle of a frame.
            for (auto it = fs::directory_iterator(current, ec);
                 !ec && it != fs::directory_iterator(); it.increment(ec))
            {
                const fs::directory_entry& entry = *it;
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
                    else
                    {
                        editor_log(context, "Could not create project folder '" +
                                                 target.string() + "': " + create_ec.message(),
                                   LogLevel::Error);
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
