/**************************************************************************/
/* panel_widgets.cpp                                                      */
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

#include "panel_widgets.hpp"

#include "../input/input_manager_window.hpp"

#include <algorithm>
#include <cctype>
#include <cfloat>
#include <cmath>

#include <imgui.h>
#include <imgui_internal.h>
#include <misc/cpp/imgui_stdlib.h>

namespace SushiEngine
{
    namespace Editor
    {
        Simulation::IWorldEditor* world_of(EditorContext& context)
        {
            return context.simulation != nullptr ? &context.simulation->world() : nullptr;
        }

        void set_asset_drag_source(const std::string& path, const std::string& label)
        {
            if (!ImGui::BeginDragDropSource(ImGuiDragDropFlags_SourceAllowNullID))
                return;
            ImGui::SetDragDropPayload(ASSET_PATH_PAYLOAD, path.c_str(), path.size() + 1);
            ImGui::TextUnformatted(label.c_str());
            ImGui::EndDragDropSource();
        }

        bool accept_asset_drop(std::string& out_path)
        {
            if (!ImGui::BeginDragDropTarget())
                return false;
            bool accepted = false;
            if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(ASSET_PATH_PAYLOAD))
            {
                out_path.assign(static_cast<const char*>(payload->Data));
                accepted = true;
            }
            ImGui::EndDragDropTarget();
            return accepted;
        }

        void track_item_undo(EditorContext& context, Simulation::IWorldEditor& world)
        {
            if (ImGui::IsItemActivated())
                context.history.begin_change(world);
            if (ImGui::IsItemDeactivatedAfterEdit())
                context.history.end_change();
        }

        ComponentSection component_header(EditorContext& context, const char* label,
                                          bool value_actions)
        {
            ComponentSection section;
            bool keep = true;
            section.open =
                ImGui::CollapsingHeader(label, &keep, ImGuiTreeNodeFlags_DefaultOpen);
            section.remove = !keep;

            // The menu hangs off the header itself rather than a separate button, which is
            // where Unity puts it and, more usefully, the only place that stays reachable
            // when the section is collapsed. A null id means "the item just drawn", so each
            // header owns its own popup; a shared literal here would give all eleven sections
            // in the window one popup between them.
            if (!ImGui::BeginPopupContextItem())
                return section;
            if (value_actions)
            {
                if (ImGui::MenuItem("Reset"))
                    section.reset = true;
                ImGui::Separator();
                if (ImGui::MenuItem("Copy Values"))
                    section.copy = true;
                const bool can_paste = context.component_clipboard.values != nullptr &&
                                       context.component_clipboard.component == label;
                if (ImGui::MenuItem("Paste Values", nullptr, false, can_paste))
                    section.paste = true;
                if (!can_paste && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
                    ImGui::SetTooltip("Copy Values from another %s first.", label);
                ImGui::Separator();
            }
            if (ImGui::MenuItem("Remove Component"))
                section.remove = true;
            ImGui::EndPopup();
            return section;
        }

        namespace
        {
            /**
             * @brief Opens a labelled field row, in a table when there is one and inline when not.
             *
             * `ImGui::TableNextRow` dereferences the current table, so opening a row
             * unconditionally would turn a call from a panel that draws no table into an
             * access violation rather than a layout glitch.
             *
             * The widget is therefore *total* rather than asking every caller to remember a
             * precondition. A widget that works in one context and crashes in another is a
             * trap whose only defence is documentation nobody re-reads; a widget that lays
             * itself out according to where it finds itself has no precondition to forget.
             *
             * @param label The row's label, drawn in the label column or before the field.
             * @return Whether the caller is inside a table, which decides the field's width.
             */
            bool begin_field_row(const char* label)
            {
                if (ImGui::GetCurrentTable() == nullptr)
                    return false;
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::AlignTextToFramePadding();
                ImGui::TextUnformatted(label);
                ImGui::TableSetColumnIndex(1);
                ImGui::SetNextItemWidth(-FLT_MIN);
                return true;
            }
        } // namespace

        bool vector3_field(EditorContext& context, Simulation::IWorldEditor& world,
                           const char* label, float values[3], float speed, bool mixed,
                           const char* format, const char* tooltip)
        {
            ImGui::PushID(label);
            const bool tabled = begin_field_row(label);
            const bool changed = ImGui::DragFloat3(tabled ? "##v" : label, values, speed, 0.0f,
                                                   0.0f, mixed ? "-" : format);
            track_item_undo(context, world);
            if (tooltip != nullptr && ImGui::IsItemHovered())
                ImGui::SetTooltip("%s", tooltip);
            ImGui::PopID();
            return changed;
        }

        bool scalar_field(EditorContext& context, Simulation::IWorldEditor& world,
                          const char* label, float* value, float speed, float min_value,
                          float max_value, const char* format, bool mixed, const char* tooltip)
        {
            ImGui::PushID(label);
            const bool tabled = begin_field_row(label);
            const bool changed = ImGui::DragFloat(tabled ? "##v" : label, value, speed, min_value,
                                                  max_value, mixed ? "-" : format);
            track_item_undo(context, world);
            if (tooltip != nullptr && ImGui::IsItemHovered())
                ImGui::SetTooltip("%s", tooltip);
            ImGui::PopID();
            return changed;
        }

        bool inline_rename_field(EditorContext& context, const std::string& target_key,
                                 const std::string& seed, float width, std::string& out_text)
        {
            if (context.rename_target != target_key)
            {
                context.rename_buffer = seed;
                context.rename_target = target_key;
                ImGui::SetKeyboardFocusHere();
            }
            ImGui::SetNextItemWidth(width);
            const bool entered = ImGui::InputText(
                "##rename", &context.rename_buffer,
                ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_AutoSelectAll);
            if (!entered && !ImGui::IsItemDeactivated())
                return false;

            out_text = context.rename_buffer;
            context.rename_target.clear();
            context.rename_buffer.clear();
            return true;
        }

        std::string shortcut_for_action(EditorContext& context, const char* action_name)
        {
            // Global first, then the viewport context: the transform-tool chords live in the
            // viewport's own context because they must stand down while the camera has the
            // keyboard, and a caller asking "what is Rotate bound to" should not have to know
            // which context that put it in.
            Input::InputContext* const contexts[] = {context.editor_global_context,
                                                     context.editor_viewport_context};
            for (Input::InputContext* candidate : contexts)
            {
                if (candidate == nullptr)
                    continue;
                Input::Action* action = candidate->find_action(action_name);
                if (action != nullptr && !action->button_bindings.empty())
                    return input_binding_label(action->button_bindings.front());
            }
            return std::string();
        }

        bool menu_item_for_action(EditorContext& context, const char* label,
                                  const char* action_name, bool enabled)
        {
            const std::string shortcut = shortcut_for_action(context, action_name);
            return ImGui::MenuItem(label, shortcut.empty() ? nullptr : shortcut.c_str(), false,
                                   enabled);
        }

        namespace
        {
            /**
             * @brief Paints one @ref ToolbarIcon inside a square of side @p side.
             *
             * Laid out from the square's centre against a single radius, so every icon reads
             * as the same visual weight beside the others — the failure mode of drawn icons is
             * one shape optically twice the size of its neighbour.
             *
             * @param list The draw list to paint into.
             * @param origin Top-left corner of the button, in screen space.
             * @param side The button's side length.
             * @param icon Which shape to paint.
             * @param color Packed colour, already carrying the current disabled dimming.
             */
            void draw_toolbar_icon(ImDrawList* list, const ImVec2& origin, float side,
                                   ToolbarIcon icon, ImU32 color)
            {
                const ImVec2 c(origin.x + side * 0.5f, origin.y + side * 0.5f);
                const float r = side * 0.22f;
                const float thickness = side * 0.09f < 1.0f ? 1.0f : side * 0.09f;
                switch (icon)
                {
                    case ToolbarIcon::Play:
                        list->AddTriangleFilled(ImVec2(c.x - r * 0.7f, c.y - r),
                                                ImVec2(c.x - r * 0.7f, c.y + r),
                                                ImVec2(c.x + r, c.y), color);
                        break;
                    case ToolbarIcon::Stop:
                        list->AddRectFilled(ImVec2(c.x - r * 0.85f, c.y - r * 0.85f),
                                            ImVec2(c.x + r * 0.85f, c.y + r * 0.85f), color,
                                            1.0f);
                        break;
                    case ToolbarIcon::Pause:
                        list->AddRectFilled(ImVec2(c.x - r * 0.85f, c.y - r),
                                            ImVec2(c.x - r * 0.25f, c.y + r), color, 1.0f);
                        list->AddRectFilled(ImVec2(c.x + r * 0.25f, c.y - r),
                                            ImVec2(c.x + r * 0.85f, c.y + r), color, 1.0f);
                        break;
                    case ToolbarIcon::Step:
                        // A play head that stops at a wall: one frame forward, then hold.
                        list->AddTriangleFilled(ImVec2(c.x - r, c.y - r), ImVec2(c.x - r, c.y + r),
                                                ImVec2(c.x + r * 0.3f, c.y), color);
                        list->AddRectFilled(ImVec2(c.x + r * 0.55f, c.y - r),
                                            ImVec2(c.x + r, c.y + r), color, 1.0f);
                        break;
                    case ToolbarIcon::Move:
                    {
                        // A four-way arrow: the axes plus a head on each end.
                        list->AddLine(ImVec2(c.x - r, c.y), ImVec2(c.x + r, c.y), color,
                                      thickness);
                        list->AddLine(ImVec2(c.x, c.y - r), ImVec2(c.x, c.y + r), color,
                                      thickness);
                        const float head = r * 0.45f;
                        list->AddTriangleFilled(ImVec2(c.x + r, c.y),
                                                ImVec2(c.x + r - head, c.y - head * 0.8f),
                                                ImVec2(c.x + r - head, c.y + head * 0.8f), color);
                        list->AddTriangleFilled(ImVec2(c.x - r, c.y),
                                                ImVec2(c.x - r + head, c.y - head * 0.8f),
                                                ImVec2(c.x - r + head, c.y + head * 0.8f), color);
                        list->AddTriangleFilled(ImVec2(c.x, c.y - r),
                                                ImVec2(c.x - head * 0.8f, c.y - r + head),
                                                ImVec2(c.x + head * 0.8f, c.y - r + head), color);
                        list->AddTriangleFilled(ImVec2(c.x, c.y + r),
                                                ImVec2(c.x - head * 0.8f, c.y + r - head),
                                                ImVec2(c.x + head * 0.8f, c.y + r - head), color);
                        break;
                    }
                    case ToolbarIcon::Rotate:
                    {
                        // An open arc with a head on the leading end: a turn, mid-turn.
                        list->PathArcTo(c, r, 0.35f * 3.14159265f, 1.85f * 3.14159265f, 16);
                        list->PathStroke(color, ImDrawFlags_None, thickness);
                        const float head = r * 0.55f;
                        const ImVec2 tip(c.x + r * 0.94f, c.y - r * 0.35f);
                        list->AddTriangleFilled(tip, ImVec2(tip.x - head, tip.y - head * 0.35f),
                                                ImVec2(tip.x - head * 0.2f, tip.y + head * 0.85f),
                                                color);
                        break;
                    }
                    case ToolbarIcon::Scale:
                        // A small handle pulled away from a large one along the diagonal.
                        list->AddLine(ImVec2(c.x - r * 0.7f, c.y + r * 0.7f),
                                      ImVec2(c.x + r * 0.7f, c.y - r * 0.7f), color, thickness);
                        list->AddRectFilled(ImVec2(c.x - r, c.y + r * 0.4f),
                                            ImVec2(c.x - r * 0.4f, c.y + r), color, 1.0f);
                        list->AddRect(ImVec2(c.x + r * 0.15f, c.y - r),
                                      ImVec2(c.x + r, c.y - r * 0.15f), color, 1.0f,
                                      ImDrawFlags_None, thickness);
                        break;
                }
            }
        } // namespace

        bool icon_button(const char* id, ToolbarIcon icon, bool active, const char* tooltip)
        {
            const float side = ImGui::GetFrameHeight();
            ImGui::PushID(id);
            if (active)
                ImGui::PushStyleColor(ImGuiCol_Button,
                                      ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
            const ImVec2 origin = ImGui::GetCursorScreenPos();
            const bool pressed = ImGui::Button("", ImVec2(side, side));
            if (active)
                ImGui::PopStyleColor();
            if (tooltip != nullptr && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
                ImGui::SetTooltip("%s", tooltip);
            // Painted after the button so the shape sits on top of its frame, and through
            // GetColorU32 so BeginDisabled's alpha reaches the glyph as well as the frame —
            // an icon that stays crisp on a greyed-out button reads as still clickable.
            draw_toolbar_icon(ImGui::GetWindowDrawList(), origin, side, icon,
                              ImGui::GetColorU32(ImGuiCol_Text));
            ImGui::PopID();
            return pressed;
        }

        std::string to_lower(const std::string& text)
        {
            std::string out = text;
            std::transform(out.begin(), out.end(), out.begin(),
                           [](unsigned char c) { return static_cast<char>(::tolower(c)); });
            return out;
        }

        void quaternion_to_euler_degrees(const Quaternion& q, float out[3])
        {
            constexpr float RAD_TO_DEG = 57.2957795f;
            const float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
            const float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
            const float roll = std::atan2(sinr_cosp, cosr_cosp);

            const float sinp = 2.0f * (q.w * q.y - q.z * q.x);
            const float pitch =
                std::fabs(sinp) >= 1.0f ? std::copysign(1.5707963f, sinp) : std::asin(sinp);

            const float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
            const float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
            const float yaw = std::atan2(siny_cosp, cosy_cosp);

            out[0] = roll * RAD_TO_DEG;
            out[1] = pitch * RAD_TO_DEG;
            out[2] = yaw * RAD_TO_DEG;
        }

        Quaternion euler_degrees_to_quat(const float in[3])
        {
            constexpr float DEG_TO_RAD = 0.01745329f;
            const float roll = in[0] * DEG_TO_RAD;
            const float pitch = in[1] * DEG_TO_RAD;
            const float yaw = in[2] * DEG_TO_RAD;

            const float cr = std::cos(roll * 0.5f), sr = std::sin(roll * 0.5f);
            const float cp = std::cos(pitch * 0.5f), sp = std::sin(pitch * 0.5f);
            const float cy = std::cos(yaw * 0.5f), sy = std::sin(yaw * 0.5f);

            Quaternion q;
            q.w = cr * cp * cy + sr * sp * sy;
            q.x = sr * cp * cy - cr * sp * sy;
            q.y = cr * sp * cy + sr * cp * sy;
            q.z = cr * cp * sy - sr * sp * cy;
            return q;
        }

        void commit_environment_edit(EditorContext& context, Simulation::IWorldEditor& world,
                                     const Render::Environment& environment)
        {
            if (!context.environment_change_active)
            {
                if (ImGui::IsAnyItemActive())
                {
                    context.history.begin_change(world);
                    context.environment_change_active = true;
                }
                else
                    context.history.record(world);
            }
            world.set_environment(environment);
        }

        void finish_environment_edit(EditorContext& context)
        {
            if (context.environment_change_active && !ImGui::IsAnyItemActive())
            {
                context.history.end_change();
                context.environment_change_active = false;
            }
        }
    } // namespace Editor
} // namespace SushiEngine
