/**************************************************************************/
/* script_panel.cpp                                                      */
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

#include "script_panel.hpp"

#include "../project/project_panel.hpp"
#include "../ui/panel_widgets.hpp"

#include <cctype>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include <imgui.h>
#include <misc/cpp/imgui_stdlib.h>

namespace SushiEngine
{
    namespace Editor
    {
        using SushiEngine::Simulation::IWorldEditor;

        namespace fs = std::filesystem;

        namespace
        {
            /** @brief Whether @p name is a legal C++/script identifier. */
            bool is_valid_identifier(const std::string& name)
            {
                if (name.empty())
                    return false;
                if (!(std::isalpha(static_cast<unsigned char>(name[0])) || name[0] == '_'))
                    return false;
                for (const char c : name)
                    if (!(std::isalnum(static_cast<unsigned char>(c)) || c == '_'))
                        return false;
                return true;
            }

        } // namespace

        void register_script_definition(EditorContext& context,
                                        const SushiEngine::Simulation::ScriptComponent& script)
        {
            for (const SushiEngine::Simulation::ScriptComponent& definition :
                 context.script_catalog)
                if (definition.type_name == script.type_name)
                    return;
            context.script_catalog.push_back(script);
        }

        /**
         * @brief Draws editable widgets for every field of a script component.
         *
         * @param context Editor state, for the undo history around each edit.
         * @param world   The world being edited (snapshotted for undo).
         * @param script  The instance whose fields are drawn and mutated in place.
         * @return Whether any field changed this frame.
         */
        bool draw_script_fields(EditorContext& context, IWorldEditor& world,
                                SushiEngine::Simulation::ScriptComponent& script)
        {
            using SushiEngine::Simulation::ScriptFieldKind;
            bool changed = false;
            if (script.fields.empty())
                ImGui::TextDisabled("No fields.");
            for (std::size_t i = 0; i < script.fields.size(); ++i)
            {
                SushiEngine::Simulation::ScriptField& field = script.fields[i];
                ImGui::PushID(static_cast<int>(i));
                const char* label = field.name.c_str();
                switch (field.kind)
                {
                    case ScriptFieldKind::Float:
                    {
                        float value = static_cast<float>(field.number);
                        if (ImGui::DragFloat(label, &value, 0.01f))
                        {
                            field.number = static_cast<SushiEngine::Scalar>(value);
                            changed = true;
                        }
                        break;
                    }
                    case ScriptFieldKind::Int:
                    {
                        int value = static_cast<int>(field.number);
                        if (ImGui::DragInt(label, &value))
                        {
                            field.number = static_cast<SushiEngine::Scalar>(value);
                            changed = true;
                        }
                        break;
                    }
                    case ScriptFieldKind::Bool:
                    {
                        if (ImGui::Checkbox(label, &field.flag))
                        {
                            context.history.record(world);
                            changed = true;
                        }
                        break;
                    }
                    case ScriptFieldKind::Vector3:
                    {
                        float value[3] = {static_cast<float>(field.vector.x),
                                          static_cast<float>(field.vector.y),
                                          static_cast<float>(field.vector.z)};
                        if (ImGui::DragFloat3(label, value, 0.01f))
                        {
                            field.vector =
                                SushiEngine::Vector3{value[0], value[1], value[2]};
                            changed = true;
                        }
                        break;
                    }
                    case ScriptFieldKind::Color:
                    {
                        float value[3] = {static_cast<float>(field.vector.x),
                                          static_cast<float>(field.vector.y),
                                          static_cast<float>(field.vector.z)};
                        if (ImGui::ColorEdit3(label, value))
                        {
                            field.vector =
                                SushiEngine::Vector3{value[0], value[1], value[2]};
                            changed = true;
                        }
                        break;
                    }
                    case ScriptFieldKind::Text:
                    {
                        if (ImGui::InputText(label, &field.text))
                            changed = true;
                        break;
                    }
                }
                track_item_undo(context, world);
                ImGui::PopID();
            }
            return changed;
        }

        /** @brief The C++ system stub scaffolded for a newly created script @p type_name. */
        std::string script_stub_source(const std::string& type_name)
        {
            std::ostringstream out;
            out << "// " << type_name << " — a SushiEngine custom component.\n"
                << "//\n"
                << "// Authored in the editor as a data-driven component (its fields are\n"
                << "// edited in the Inspector and saved with the scene). Fill in the ECS\n"
                << "// system below to give it behaviour, then register it on your\n"
                << "// Loop::App the same way the built-in systems are.\n"
                << "#pragma once\n\n"
                << "#include <SushiEngine/core/types.hpp>\n\n"
                << "struct " << type_name << "\n"
                << "{\n"
                << "    SushiEngine::Scalar speed = SushiEngine::Scalar(1);\n"
                << "};\n\n"
                << "// Example system (register with app.system<...>(\"" << type_name
                << "\").each(...)):\n"
                << "//\n"
                << "//   app.system<SushiEngine::Write<Transform>, SushiEngine::Read<"
                << type_name << ">>(\"" << type_name << "\")\n"
                << "//      .each([](std::size_t i, Transform* transform, const " << type_name
                << "* self)\n"
                << "//      {\n"
                << "//          transform[i].position.x += self[i].speed;\n"
                << "//      });\n";
            return out.str();
        }

        /**
         * @brief The New Script modal: names a custom component, scaffolds its C++
         * stub in the project, registers it in the catalog, and attaches it.
         *
         * Driven by `context.show_new_script` (raised by the Add Component menu). On
         * Create it seeds a one-field definition (a `speed` float, mirroring the
         * generated stub), writes `<Name>.hpp` under the project root, opens it in
         * the Text Editor, and attaches the component to the entity that requested it.
         */
        void draw_new_script_modal(EditorContext& context)
        {
            if (context.show_new_script)
            {
                ImGui::OpenPopup("New Script");
                context.show_new_script = false;
            }

            const ImVec2 center = ImGui::GetMainViewport()->GetCenter();
            ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
            if (!ImGui::BeginPopupModal("New Script", nullptr,
                                        ImGuiWindowFlags_AlwaysAutoResize))
                return;

            ImGui::InputText("Class Name", &context.new_script_name);
            ImGui::TextDisabled("Creates <Name>.hpp in the project and attaches it.");

            const bool valid = is_valid_identifier(context.new_script_name);
            ImGui::BeginDisabled(!valid);
            if (ImGui::Button("Create"))
            {
                SushiEngine::Simulation::ScriptComponent definition;
                definition.type_name = context.new_script_name;
                SushiEngine::Simulation::ScriptField field;
                field.name = "speed";
                field.kind = SushiEngine::Simulation::ScriptFieldKind::Float;
                field.number = SushiEngine::Scalar(1);
                definition.fields.push_back(field);
                register_script_definition(context, definition);

                const fs::path path =
                    fs::path(context.project_root) / (context.new_script_name + ".hpp");
                std::ofstream stream(path, std::ios::binary);
                if (stream)
                {
                    stream << script_stub_source(context.new_script_name);
                    stream.close();
                    open_document(context, path);
                    editor_log(context, "Created script '" + path.filename().string() + "'.");
                }
                else
                {
                    editor_log(context, "Failed to write script '" + path.string() + "'.",
                               LogLevel::Error);
                }

                IWorldEditor* world = world_of(context);
                if (world != nullptr && world->exists(context.new_script_target))
                {
                    context.history.record(*world);
                    world->add_script_component(context.new_script_target, definition);
                }
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndDisabled();
            ImGui::SameLine();
            if (ImGui::Button("Cancel"))
                ImGui::CloseCurrentPopup();
            ImGui::EndPopup();
        }
} // namespace Editor
} // namespace SushiEngine
