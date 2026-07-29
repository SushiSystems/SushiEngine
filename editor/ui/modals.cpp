/**************************************************************************/
/* modals.cpp                                                            */
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

#include "modals.hpp"

#include "../scene/scene_commands.hpp"
#include "../serialization/scene_serializer.hpp"
#include "panel_widgets.hpp"

#include <filesystem>
#include <string>

#include <imgui.h>
#include <misc/cpp/imgui_stdlib.h>

namespace SushiEngine
{
    namespace Editor
    {
        using SushiEngine::Simulation::IWorldEditor;

        namespace fs = std::filesystem;

        void draw_save_scene_as_modal(EditorContext& context, bool& running)
        {
            if (!context.show_save_scene_as)
                return;

            const char* popup_id = "Save Scene As";
            ImGui::OpenPopup(popup_id);
            if (ImGui::BeginPopupModal(popup_id, &context.show_save_scene_as,
                                       ImGuiWindowFlags_AlwaysAutoResize))
            {
                ImGui::TextDisabled("%s", context.project_root.c_str());
                ImGui::SetNextItemWidth(320.0f);
                const bool commit = ImGui::InputText("##save_scene_name", &context.save_scene_as_name,
                                                      ImGuiInputTextFlags_EnterReturnsTrue);

                const bool confirmed = ImGui::Button("Save") || commit;
                ImGui::SameLine();
                const bool cancelled = ImGui::Button("Cancel");

                if (confirmed && !context.save_scene_as_name.empty())
                {
                    fs::path path = fs::path(context.project_root) / context.save_scene_as_name;
                    if (path.extension() != ".sushiscene")
                        path += ".sushiscene";
                    IWorldEditor* world = world_of(context);
                    const SceneSkyState sky = capture_sky_state(context);
                    if (world != nullptr && save_scene(*world, path.string(), &sky))
                    {
                        context.scene_path = path.string();
                        context.saved_scene_revision = context.history.revision();
                        note_recent_scene(context, context.scene_path);
                        editor_log(context, "Saved scene '" + context.scene_path + "'.");
                        // This save-as was raised to unblock a pending window close (Ctrl+S
                        // or "Save" from the unsaved-changes prompt with no scene path yet)
                        // or a pending New/Open scene request; finish whichever is waiting.
                        if (context.exit_after_save)
                            running = false;
                        else if (context.pending_scene_action != EditorContext::PendingSceneAction::None)
                            perform_pending_scene_action(context);
                    }
                    else
                    {
                        editor_log(context, "Failed to save scene '" + path.string() + "'.",
                                   LogLevel::Error);
                    }
                    context.show_save_scene_as = false;
                    context.exit_after_save = false;
                    ImGui::CloseCurrentPopup();
                }
                else if (cancelled)
                {
                    // A cancelled save-as also aborts any pending close or pending
                    // New/Open scene request it was raised for.
                    if (context.exit_after_save)
                        context.close_requested = false;
                    context.pending_scene_action = EditorContext::PendingSceneAction::None;
                    context.pending_scene_open_path.clear();
                    context.show_save_scene_as = false;
                    context.exit_after_save = false;
                    ImGui::CloseCurrentPopup();
                }

                ImGui::EndPopup();
            }
        }

        void draw_exit_confirm_modal(EditorContext& context, bool& running)
        {
            if (!context.close_requested)
                return;
            // A Save-As triggered by this same close is still pending; wait for it
            // (it resolves close_requested/running itself on save or cancel).
            if (context.show_save_scene_as)
                return;
            if (!scene_is_dirty(context))
            {
                running = false;
                return;
            }

            const char* popup_id = "Unsaved Changes";
            ImGui::OpenPopup(popup_id);
            if (ImGui::BeginPopupModal(popup_id, nullptr, ImGuiWindowFlags_AlwaysAutoResize))
            {
                ImGui::Text("The scene has unsaved changes. Save before closing?");
                ImGui::Spacing();

                if (ImGui::Button("Save"))
                {
                    if (context.scene_path.empty())
                        context.exit_after_save = true; // save_current_scene opens Save As
                    if (save_current_scene(context))
                        running = false; // saved straight to an existing path
                    ImGui::CloseCurrentPopup();
                }
                ImGui::SameLine();
                if (ImGui::Button("Don't Save"))
                {
                    running = false;
                    ImGui::CloseCurrentPopup();
                }
                ImGui::SameLine();
                if (ImGui::Button("Cancel"))
                {
                    context.close_requested = false;
                    ImGui::CloseCurrentPopup();
                }

                ImGui::EndPopup();
            }
        }

        void draw_scene_action_confirm_modal(EditorContext& context)
        {
            if (context.pending_scene_action == EditorContext::PendingSceneAction::None)
                return;
            // A Save-As triggered by this same request is still pending; wait for it
            // (draw_save_scene_as_modal resolves pending_scene_action itself on save or
            // cancel).
            if (context.show_save_scene_as)
                return;
            if (!scene_is_dirty(context))
            {
                perform_pending_scene_action(context);
                return;
            }

            const char* popup_id = "Unsaved Changes##scene_action";
            ImGui::OpenPopup(popup_id);
            if (ImGui::BeginPopupModal(popup_id, nullptr, ImGuiWindowFlags_AlwaysAutoResize))
            {
                ImGui::Text("The scene has unsaved changes. Save before continuing?");
                ImGui::Spacing();

                if (ImGui::Button("Save"))
                {
                    if (save_current_scene(context))
                        perform_pending_scene_action(context); // saved straight to an existing path
                    // else: save_current_scene opened the Save-As modal, which finishes the
                    // pending action once the save completes.
                    ImGui::CloseCurrentPopup();
                }
                ImGui::SameLine();
                if (ImGui::Button("Don't Save"))
                {
                    perform_pending_scene_action(context);
                    ImGui::CloseCurrentPopup();
                }
                ImGui::SameLine();
                if (ImGui::Button("Cancel"))
                {
                    context.pending_scene_action = EditorContext::PendingSceneAction::None;
                    context.pending_scene_open_path.clear();
                    ImGui::CloseCurrentPopup();
                }

                ImGui::EndPopup();
            }
        }
    } // namespace Editor
} // namespace SushiEngine
