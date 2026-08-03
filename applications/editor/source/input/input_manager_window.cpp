/**************************************************************************/
/* input_manager_window.cpp                                               */
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

#include "input_manager_window.hpp"

#include "editor_contexts.hpp"

#include <cstdint>
#include <string>

#include <imgui.h>

#include <SushiEngine/input/bindings_json.hpp>

namespace SushiEngine
{
    namespace Editor
    {
        namespace
        {
            // A readable label for a control, for the Input Manager window. The editor contexts
            // are keyboard-only, so keys get their glyph and modifiers a "Ctrl"/"Shift"/"Alt"
            // prefix; other families fall back to a family name (they are not editor-bound yet).
            std::string input_key_name(std::uint16_t ordinal)
            {
                if (ordinal >= 4 && ordinal <= 29) // A..Z
                    return std::string(1, static_cast<char>('A' + (ordinal - 4)));
                if (ordinal >= 30 && ordinal <= 38) // 1..9
                    return std::string(1, static_cast<char>('1' + (ordinal - 30)));
                switch (ordinal)
                {
                    case 39: return "0";
                    case 40: return "Enter";
                    case 41: return "Esc";
                    case 42: return "Backspace";
                    case 43: return "Tab";
                    case 44: return "Space";
                    case 70: return "PrintScreen";
                    case 72: return "Pause";
                    case 73: return "Insert";
                    case 74: return "Home";
                    case 75: return "PageUp";
                    case 76: return "Delete";
                    case 77: return "End";
                    case 78: return "PageDown";
                    case 79: return "Right";
                    case 80: return "Left";
                    case 81: return "Down";
                    case 82: return "Up";
                    case 118: return "Menu";
                    case 224: case 228: return "Ctrl";
                    case 225: case 229: return "Shift";
                    case 226: case 230: return "Alt";
                    default: return "Key#" + std::to_string(ordinal);
                }
            }

            std::string input_control_label(const SushiEngine::Input::ControlPath& path)
            {
                using SushiEngine::Input::DeviceFamily;
                switch (path.family)
                {
                    case DeviceFamily::Keyboard: return input_key_name(path.control);
                    case DeviceFamily::Mouse:    return "Mouse";
                    case DeviceFamily::Gamepad:  return "Pad";
                    case DeviceFamily::Virtual:  return "Virtual";
                    case DeviceFamily::Touch:    return "Touch";
                }
                return "?";
            }

        } // namespace

        std::string input_binding_label(const SushiEngine::Input::Binding& binding)
        {
            std::string label;
            for (std::uint8_t i = 0; i < binding.chord.count; ++i)
                label += input_control_label(binding.chord.modifiers[i]) + "+";
            label += input_control_label(binding.control);
            return label;
        }

        void draw_input_manager_window(EditorContext& context)
        {
            if (!context.panels.input_manager)
                return;

            // Needs the live contexts and manager main() wired in; without them there is nothing
            // to configure, so close quietly rather than show an empty shell.
            if (context.input_manager == nullptr || context.editor_global_context == nullptr ||
                context.editor_viewport_context == nullptr)
            {
                context.panels.input_manager = false;
                return;
            }

            ImGui::SetNextWindowSize(ImVec2(540.0f, 480.0f), ImGuiCond_FirstUseEver);
            if (!ImGui::Begin("Input Manager", &context.panels.input_manager))
            {
                ImGui::End();
                return;
            }

            ImGui::TextWrapped("Bind editor shortcuts and viewport tools. Click Rebind, then press a "
                               "key (Esc cancels). Changes are saved to your preferences and restored "
                               "on the next launch.");
            ImGui::Separator();

            RebindState& rebind = context.panel_state.rebind;
            SushiEngine::Input::RebindingListener& rebind_listener = rebind.listener;
            std::string& rebinding_action = rebind.action;
            SushiEngine::Input::InputContext*& rebinding_context = rebind.context;

            const auto serialize_bindings = [&]()
            {
                nlohmann::json bindings =
                    SushiEngine::Input::bindings_to_json(*context.editor_global_context);
                bindings = SushiEngine::Input::bindings_to_json(*context.editor_viewport_context, bindings);
                context.preferences.input_bindings = bindings.dump();
                context.preferences_dirty = true;
            };

            const auto draw_context_section =
                [&](const char* label, SushiEngine::Input::InputContext& bindings_context)
            {
                if (!ImGui::CollapsingHeader(label, ImGuiTreeNodeFlags_DefaultOpen))
                    return;
                for (const auto& action : bindings_context.actions())
                {
                    if (action->type != SushiEngine::Input::ActionType::Button)
                        continue;
                    ImGui::PushID(action->name.c_str());
                    ImGui::TextUnformatted(action->name.c_str());

                    ImGui::SameLine(170.0f);
                    const std::string binding_text = action->button_bindings.empty()
                                                         ? std::string("(unbound)")
                                                         : input_binding_label(action->button_bindings[0]);
                    ImGui::TextDisabled("%s", binding_text.c_str());

                    ImGui::SameLine(320.0f);
                    const bool capturing =
                        rebind_listener.listening() && rebinding_action == action->name;
                    if (capturing)
                    {
                        ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "Press a key... (Esc)");
                    }
                    else if (ImGui::Button("Rebind"))
                    {
                        rebind_listener.begin(SushiEngine::Input::RebindShape::Button, 0.5f, 5.0f);
                        rebinding_action = action->name;
                        rebinding_context = &bindings_context;
                    }

                    if (!action->button_bindings.empty())
                    {
                        const SushiEngine::Input::Action* conflict = SushiEngine::Input::binding_conflict(
                            bindings_context, action->button_bindings[0].control, action->name);
                        if (conflict != nullptr)
                        {
                            ImGui::SameLine();
                            ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "conflicts with %s",
                                               conflict->name.c_str());
                        }
                    }
                    ImGui::PopID();
                }
            };

            draw_context_section("Editor Global", *context.editor_global_context);
            draw_context_section("Editor Viewport", *context.editor_viewport_context);

            ImGui::Separator();
            if (ImGui::Button("Reset to Defaults"))
            {
                // Rebuild in place: the objects keep their addresses (main() pushed pointers to
                // them onto the mapper stack), so a move-assign of a fresh context is safe.
                *context.editor_global_context = SushiEngine::Input::InputContext{"EditorGlobal"};
                build_editor_global_context(*context.editor_global_context);
                *context.editor_viewport_context = SushiEngine::Input::InputContext{"EditorViewport"};
                build_editor_viewport_context(*context.editor_viewport_context);
                context.preferences.input_bindings.clear();
                context.preferences_dirty = true;
            }

            // Advance an active capture with this frame's raw events.
            if (rebind_listener.listening())
            {
                const SushiEngine::Input::RebindStatus status =
                    rebind_listener.update(context.input_manager->frame_events());
                if (status == SushiEngine::Input::RebindStatus::Captured && rebinding_context != nullptr)
                {
                    if (SushiEngine::Input::Action* action =
                            rebinding_context->find_action(rebinding_action))
                        SushiEngine::Input::set_button_binding(*action, rebind_listener.captured());
                    serialize_bindings();
                    rebinding_context = nullptr;
                }
            }

            ImGui::End();
        }
    } // namespace Editor
} // namespace SushiEngine
