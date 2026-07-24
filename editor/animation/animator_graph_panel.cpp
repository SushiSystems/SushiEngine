/**************************************************************************/
/* animator_graph_panel.cpp                                              */
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

#include "animator_graph_panel.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <imgui.h>

#include <SushiEngine/animation/animator_controller.hpp>
#include <SushiEngine/animation/animator_controller_json.hpp>

namespace SushiEngine
{
    namespace Editor
    {
        namespace
        {
            using SushiEngine::Animation::AssetId;
            using SushiEngine::Animation::BlendChildDesc;
            using SushiEngine::Animation::BlendTreeNodeDesc;
            using SushiEngine::Animation::BlendTreeType;
            using SushiEngine::Animation::ControllerDesc;
            using SushiEngine::Animation::INVALID_ASSET;
            using SushiEngine::Animation::LayerDesc;
            using SushiEngine::Animation::ParameterDesc;
            using SushiEngine::Animation::ParameterType;
            using SushiEngine::Animation::StateDesc;
            using SushiEngine::Animation::TransitionDesc;

            constexpr float NODE_W = 128.0f;
            constexpr float NODE_H = 44.0f;

            ImVec2 operator+(const ImVec2& a, const ImVec2& b) { return ImVec2(a.x + b.x, a.y + b.y); }
            ImVec2 operator-(const ImVec2& a, const ImVec2& b) { return ImVec2(a.x - b.x, a.y - b.y); }

            struct GraphState
            {
                ControllerDesc controller;
                int layer = 0;
                std::vector<ImVec2> positions;       /**< One per state in the current layer. */
                std::vector<std::string> clip_paths; /**< Per-state `.sushianim` motion path. */
                int positions_layer = -1;
                ImVec2 pan{40.0f, 40.0f};
                float zoom = 1.0f;               /**< Canvas zoom (scroll wheel), clamped. */
                ImVec2 entry_pos{20.0f, 20.0f};  /**< Entry node graph position. */
                ImVec2 exit_pos{560.0f, 20.0f};  /**< Exit node graph position. */
                int selected_state = -1;
                int selected_transition_state = -1; /**< Source state, or -2 for Any-State. */
                int selected_transition = -1;
                bool linking = false;
                bool link_from_drag = false; /**< The wire is being dragged from an output nub. */
                int link_source = -1; /**< Source state index, or -2 for Any-State. */
                ImVec2 context_menu_pos{0.0f, 0.0f}; /**< Graph-space point of the last canvas right-click. */
                char new_state[64] = "New State";
                char new_param[64] = "param";
                char io_path[256] = "controller.json";
                std::string status;
                bool seeded = false;
            };

            GraphState& state()
            {
                static GraphState instance;
                return instance;
            }

            void seed(GraphState& g)
            {
                if (g.seeded)
                    return;
                g.seeded = true;
                LayerDesc layer;
                layer.name = "Base Layer";
                layer.default_state = "Idle";
                StateDesc idle;
                idle.name = "Idle";
                StateDesc move;
                move.name = "Move";
                TransitionDesc to_move;
                to_move.destination = "Move";
                to_move.duration = 0.15f;
                idle.transitions.push_back(to_move);
                TransitionDesc to_idle;
                to_idle.destination = "Idle";
                move.transitions.push_back(to_idle);
                layer.states = {idle, move};
                g.controller.layers.push_back(layer);
                g.controller.parameters.push_back(ParameterDesc{"speed", ParameterType::Float, 0.0f});
            }

            LayerDesc& current_layer(GraphState& g)
            {
                if (g.controller.layers.empty())
                    g.controller.layers.push_back(LayerDesc{});
                g.layer = std::min(g.layer, static_cast<int>(g.controller.layers.size()) - 1);
                if (g.layer < 0)
                    g.layer = 0;
                return g.controller.layers[g.layer];
            }

            // Grid-lays out positions when the layer or its state count changed; keeps the
            // per-state clip-path list the same length as the states.
            void ensure_positions(GraphState& g)
            {
                LayerDesc& layer = current_layer(g);
                if (g.clip_paths.size() != layer.states.size())
                    g.clip_paths.resize(layer.states.size());
                if (g.positions_layer == g.layer && g.positions.size() == layer.states.size())
                    return;
                g.positions_layer = g.layer;
                const std::size_t previous = g.positions.size();
                g.positions.resize(layer.states.size());
                for (std::size_t i = previous; i < layer.states.size(); ++i)
                    g.positions[i] = ImVec2(60.0f + static_cast<float>(i % 4) * 180.0f,
                                            60.0f + static_cast<float>(i / 4) * 90.0f);
            }

            int state_index(const LayerDesc& layer, const std::string& name)
            {
                for (std::size_t i = 0; i < layer.states.size(); ++i)
                    if (layer.states[i].name == name)
                        return static_cast<int>(i);
                return -1;
            }

            float segment_distance(const ImVec2& p, const ImVec2& a, const ImVec2& b)
            {
                const ImVec2 ab = b - a;
                const ImVec2 ap = p - a;
                const float len_sq = ab.x * ab.x + ab.y * ab.y;
                float t = len_sq > 1e-4f ? (ap.x * ab.x + ap.y * ab.y) / len_sq : 0.0f;
                t = std::max(0.0f, std::min(1.0f, t));
                const ImVec2 closest(a.x + ab.x * t, a.y + ab.y * t);
                const ImVec2 d = p - closest;
                return std::sqrt(d.x * d.x + d.y * d.y);
            }

            void draw_arrow(ImDrawList* draw, const ImVec2& from, const ImVec2& to, ImU32 color)
            {
                draw->AddLine(from, to, color, 2.0f);
                const ImVec2 dir = to - from;
                const float len = std::sqrt(dir.x * dir.x + dir.y * dir.y);
                if (len < 1e-3f)
                    return;
                const ImVec2 unit(dir.x / len, dir.y / len);
                const ImVec2 normal(-unit.y, unit.x);
                const ImVec2 tip = to - ImVec2(unit.x * 4.0f, unit.y * 4.0f);
                const ImVec2 base = tip - ImVec2(unit.x * 10.0f, unit.y * 10.0f);
                draw->AddTriangleFilled(tip, base + ImVec2(normal.x * 5.0f, normal.y * 5.0f),
                                        base - ImVec2(normal.x * 5.0f, normal.y * 5.0f), color);
            }

            // ---- The parameter + layer side panel --------------------------------------

            void save_controller(GraphState& g)
            {
                nlohmann::json json = SushiEngine::Animation::controller_to_json(g.controller);
                // Editor-only side data (ignored by the runtime): per-state clip file paths.
                nlohmann::json paths = nlohmann::json::object();
                const LayerDesc& layer = current_layer(g);
                for (std::size_t i = 0; i < layer.states.size() && i < g.clip_paths.size(); ++i)
                    if (!g.clip_paths[i].empty())
                        paths[layer.states[i].name] = g.clip_paths[i];
                json["editor"] = nlohmann::json{{"clip_paths", paths}};
                std::ofstream file(g.io_path);
                if (!file)
                {
                    g.status = std::string("Cannot write ") + g.io_path;
                    return;
                }
                file << json.dump(2);
                g.status = std::string("Saved ") + g.io_path;
            }

            void load_controller(GraphState& g)
            {
                std::ifstream file(g.io_path);
                if (!file)
                {
                    g.status = std::string("Cannot read ") + g.io_path;
                    return;
                }
                std::stringstream buffer;
                buffer << file.rdbuf();
                try
                {
                    const nlohmann::json json = nlohmann::json::parse(buffer.str());
                    g.controller = SushiEngine::Animation::controller_from_json(json);
                    g.positions_layer = -1;
                    g.selected_state = -1;
                    // Restore the editor-only per-state clip paths, keyed by state name.
                    g.layer = 0;
                    g.clip_paths.clear();
                    const LayerDesc& layer = current_layer(g);
                    g.clip_paths.resize(layer.states.size());
                    if (json.contains("editor") && json["editor"].contains("clip_paths"))
                    {
                        const nlohmann::json& paths = json["editor"]["clip_paths"];
                        for (std::size_t i = 0; i < layer.states.size(); ++i)
                            if (paths.contains(layer.states[i].name))
                                g.clip_paths[i] = paths[layer.states[i].name].get<std::string>();
                    }
                    g.status = std::string("Loaded ") + g.io_path;
                }
                catch (const std::exception& error)
                {
                    g.status = std::string("Parse error: ") + error.what();
                }
            }

            // A dropdown of the controller's parameters that writes the chosen name.
            void param_combo(const char* label, std::string& value, const ControllerDesc& controller)
            {
                if (ImGui::BeginCombo(label, value.empty() ? "(none)" : value.c_str()))
                {
                    if (ImGui::Selectable("(none)", value.empty()))
                        value.clear();
                    for (const ParameterDesc& p : controller.parameters)
                        if (ImGui::Selectable(p.name.c_str(), value == p.name))
                            value = p.name;
                    ImGui::EndCombo();
                }
            }

            // A small blend-space picture: 1D thresholds on a bar, or 2D positions on a pad.
            void draw_blend_visualization(const BlendTreeNodeDesc& node)
            {
                const bool one_d = node.type == BlendTreeType::Simple1D;
                const ImVec2 origin = ImGui::GetCursorScreenPos();
                const float width = std::max(ImGui::GetContentRegionAvail().x, 60.0f);
                const float height = one_d ? 40.0f : 120.0f;
                ImDrawList* draw = ImGui::GetWindowDrawList();
                draw->AddRectFilled(origin, origin + ImVec2(width, height), IM_COL32(20, 20, 24, 255));
                draw->AddRect(origin, origin + ImVec2(width, height), IM_COL32(60, 60, 70, 255));

                if (one_d)
                {
                    float lo = 0.0f, hi = 1.0f;
                    for (const BlendChildDesc& c : node.children)
                    {
                        lo = std::min(lo, c.threshold);
                        hi = std::max(hi, c.threshold);
                    }
                    if (hi - lo < 1e-3f)
                        hi = lo + 1.0f;
                    const float y = origin.y + height * 0.5f;
                    draw->AddLine(ImVec2(origin.x + 6.0f, y), ImVec2(origin.x + width - 6.0f, y),
                                  IM_COL32(90, 90, 100, 255));
                    for (const BlendChildDesc& c : node.children)
                    {
                        const float x = origin.x + 6.0f +
                                        (c.threshold - lo) / (hi - lo) * (width - 12.0f);
                        draw->AddCircleFilled(ImVec2(x, y), 4.0f, IM_COL32(120, 190, 255, 255));
                        char label[16];
                        std::snprintf(label, sizeof(label), "%d",
                                      c.clip == INVALID_ASSET ? -1 : static_cast<int>(c.clip));
                        draw->AddText(ImVec2(x - 4.0f, y + 6.0f), IM_COL32(180, 180, 180, 255), label);
                    }
                }
                else
                {
                    float range = 1.0f;
                    for (const BlendChildDesc& c : node.children)
                    {
                        range = std::max(range, std::fabs(c.position_x));
                        range = std::max(range, std::fabs(c.position_y));
                    }
                    const ImVec2 center = origin + ImVec2(width * 0.5f, height * 0.5f);
                    draw->AddLine(ImVec2(origin.x, center.y), ImVec2(origin.x + width, center.y),
                                  IM_COL32(60, 60, 70, 255));
                    draw->AddLine(ImVec2(center.x, origin.y), ImVec2(center.x, origin.y + height),
                                  IM_COL32(60, 60, 70, 255));
                    for (const BlendChildDesc& c : node.children)
                    {
                        const float x = center.x + c.position_x / range * (width * 0.45f);
                        const float y = center.y - c.position_y / range * (height * 0.45f);
                        draw->AddCircleFilled(ImVec2(x, y), 4.0f, IM_COL32(120, 190, 255, 255));
                    }
                }
                ImGui::Dummy(ImVec2(width, height));
            }

            // The Motion editor for a state: a single clip, or a blend tree with children.
            void draw_motion(StateDesc& s, const ControllerDesc& controller, std::string& clip_path)
            {
                ImGui::SeparatorText("Motion");
                if (!s.blend_tree)
                {
                    // The animation this state plays: a `.sushianim` file (baked in the Animation
                    // window). The numeric id is what the runtime AnimationDatabase keys the clip by.
                    char buffer[256];
                    std::snprintf(buffer, sizeof(buffer), "%s", clip_path.c_str());
                    ImGui::SetNextItemWidth(-1.0f);
                    if (ImGui::InputTextWithHint("##clip", "clip.sushianim", buffer, sizeof(buffer)))
                        clip_path = buffer;
                    int clip = s.clip == INVALID_ASSET ? -1 : static_cast<int>(s.clip);
                    if (ImGui::InputInt("Clip Id", &clip))
                        s.clip = clip < 0 ? INVALID_ASSET : static_cast<AssetId>(clip);
                    if (ImGui::Button("Convert to Blend Tree"))
                    {
                        s.blend_tree = std::make_shared<BlendTreeNodeDesc>();
                        s.blend_tree->type = BlendTreeType::Simple1D;
                        if (!controller.parameters.empty())
                            s.blend_tree->parameter_x = controller.parameters[0].name;
                    }
                    return;
                }

                BlendTreeNodeDesc& node = *s.blend_tree;
                int type = static_cast<int>(node.type);
                if (ImGui::Combo("Type", &type,
                                 "1D\0" "2D Simple Dir\0" "2D Freeform Dir\0" "2D Freeform Cart\0"
                                 "Direct\0"))
                    node.type = static_cast<BlendTreeType>(type);
                param_combo("Param X", node.parameter_x, controller);
                if (type >= 1 && type <= 3)
                    param_combo("Param Y", node.parameter_y, controller);
                if (type == 4)
                    ImGui::Checkbox("Normalize", &node.normalize);

                int remove = -1;
                for (int i = 0; i < static_cast<int>(node.children.size()); ++i)
                {
                    BlendChildDesc& child = node.children[i];
                    ImGui::PushID(i);
                    int clip = child.clip == INVALID_ASSET ? -1 : static_cast<int>(child.clip);
                    ImGui::SetNextItemWidth(70.0f);
                    if (ImGui::InputInt("Clip", &clip, 0))
                        child.clip = clip < 0 ? INVALID_ASSET : static_cast<AssetId>(clip);
                    ImGui::SameLine();
                    if (type == 0)
                    {
                        ImGui::SetNextItemWidth(80.0f);
                        ImGui::DragFloat("Thr", &child.threshold, 0.01f);
                    }
                    else if (type >= 1 && type <= 3)
                    {
                        ImGui::SetNextItemWidth(60.0f);
                        ImGui::DragFloat("X", &child.position_x, 0.01f);
                        ImGui::SameLine();
                        ImGui::SetNextItemWidth(60.0f);
                        ImGui::DragFloat("Y", &child.position_y, 0.01f);
                    }
                    else
                    {
                        param_combo("P", child.parameter, controller);
                    }
                    ImGui::SameLine();
                    if (ImGui::SmallButton("X##rm")) // distinct id from the 2D "X" drag field
                        remove = i;
                    ImGui::PopID();
                }
                if (remove >= 0)
                    node.children.erase(node.children.begin() + remove);
                if (ImGui::Button("Add Child"))
                    node.children.push_back(BlendChildDesc{});
                ImGui::SameLine();
                if (ImGui::Button("Convert to Clip"))
                {
                    s.blend_tree.reset();
                    return;
                }
                draw_blend_visualization(node);
            }

            void draw_side(GraphState& g)
            {
                ImGui::BeginChild("side", ImVec2(240.0f, 0.0f), true);
                ensure_positions(g); // keep clip_paths sized to the states before indexing

                // Layer selector.
                LayerDesc& layer = current_layer(g);
                if (ImGui::BeginCombo("Layer", layer.name.c_str()))
                {
                    for (int i = 0; i < static_cast<int>(g.controller.layers.size()); ++i)
                        if (ImGui::Selectable(g.controller.layers[i].name.c_str(), g.layer == i))
                            g.layer = i;
                    ImGui::EndCombo();
                }

                ImGui::SeparatorText("States");
                ImGui::SetNextItemWidth(-60.0f);
                ImGui::InputText("##ns", g.new_state, sizeof(g.new_state));
                ImGui::SameLine();
                if (ImGui::Button("Add") && g.new_state[0] != '\0')
                {
                    StateDesc s;
                    s.name = g.new_state;
                    layer.states.push_back(s);
                    if (layer.default_state.empty())
                        layer.default_state = s.name;
                    ensure_positions(g);
                }
                if (g.selected_state >= 0 && g.selected_state < static_cast<int>(layer.states.size()))
                {
                    StateDesc& s = layer.states[g.selected_state];
                    char buffer[64];
                    std::snprintf(buffer, sizeof(buffer), "%s", s.name.c_str());
                    if (ImGui::InputText("Name", buffer, sizeof(buffer)))
                        s.name = buffer;
                    if (ImGui::Button("Set Default"))
                        layer.default_state = s.name;
                    ImGui::SameLine();
                    if (ImGui::Button("Link From"))
                    {
                        g.linking = true;
                        g.link_from_drag = false;
                        g.link_source = g.selected_state;
                    }
                    ImGui::DragFloat("Speed", &s.speed, 0.01f, 0.0f, 10.0f);
                    static std::string empty_path;
                    std::string& clip_path =
                        g.selected_state < static_cast<int>(g.clip_paths.size())
                            ? g.clip_paths[g.selected_state]
                            : empty_path;
                    draw_motion(s, g.controller, clip_path);
                }

                ImGui::SeparatorText("Parameters");
                ImGui::SetNextItemWidth(-60.0f);
                ImGui::InputText("##np", g.new_param, sizeof(g.new_param));
                ImGui::SameLine();
                if (ImGui::Button("Add##p") && g.new_param[0] != '\0')
                    g.controller.parameters.push_back(
                        ParameterDesc{g.new_param, ParameterType::Float, 0.0f});
                int remove_param = -1;
                for (int i = 0; i < static_cast<int>(g.controller.parameters.size()); ++i)
                {
                    ParameterDesc& p = g.controller.parameters[i];
                    ImGui::PushID(i);
                    char buffer[64];
                    std::snprintf(buffer, sizeof(buffer), "%s", p.name.c_str());
                    ImGui::SetNextItemWidth(90.0f);
                    if (ImGui::InputText("##pn", buffer, sizeof(buffer)))
                        p.name = buffer;
                    ImGui::SameLine();
                    int type = static_cast<int>(p.type);
                    ImGui::SetNextItemWidth(80.0f);
                    if (ImGui::Combo("##pt", &type, "Float\0Int\0Bool\0Trigger\0"))
                        p.type = static_cast<ParameterType>(type);
                    ImGui::SameLine();
                    if (ImGui::SmallButton("X"))
                        remove_param = i;
                    ImGui::PopID();
                }
                if (remove_param >= 0)
                    g.controller.parameters.erase(g.controller.parameters.begin() + remove_param);

                ImGui::SeparatorText("File");
                ImGui::SetNextItemWidth(-1.0f);
                ImGui::InputText("##io", g.io_path, sizeof(g.io_path));
                if (ImGui::Button("Save JSON"))
                    save_controller(g);
                ImGui::SameLine();
                if (ImGui::Button("Load JSON"))
                    load_controller(g);
                if (!g.status.empty())
                    ImGui::TextWrapped("%s", g.status.c_str());

                ImGui::EndChild();
            }

            // Removes a state and every transition (in any state or Any-State) that targets it.
            void delete_state(GraphState& g, int index)
            {
                LayerDesc& layer = current_layer(g);
                if (index < 0 || index >= static_cast<int>(layer.states.size()))
                    return;
                const std::string name = layer.states[index].name;
                layer.states.erase(layer.states.begin() + index);
                // Keep the parallel per-state editor arrays aligned with the states.
                if (index < static_cast<int>(g.positions.size()))
                    g.positions.erase(g.positions.begin() + index);
                if (index < static_cast<int>(g.clip_paths.size()))
                    g.clip_paths.erase(g.clip_paths.begin() + index);
                const auto strip = [&](std::vector<TransitionDesc>& list)
                {
                    list.erase(std::remove_if(list.begin(), list.end(),
                                              [&](const TransitionDesc& t)
                                              { return t.destination == name; }),
                               list.end());
                };
                for (StateDesc& s : layer.states)
                    strip(s.transitions);
                strip(layer.any_state_transitions);
                if (layer.default_state == name)
                    layer.default_state = layer.states.empty() ? std::string{} : layer.states[0].name;
                g.selected_state = -1;
                g.positions_layer = -1;
            }

            // ---- The graph canvas ------------------------------------------------------

            void draw_grid(ImDrawList* draw, const ImVec2& origin, const ImVec2& size,
                           const ImVec2& pan, float step)
            {
                const ImU32 line = IM_COL32(38, 38, 44, 255);
                for (float x = std::fmod(pan.x, step); x < size.x; x += step)
                    draw->AddLine(ImVec2(origin.x + x, origin.y),
                                  ImVec2(origin.x + x, origin.y + size.y), line);
                for (float y = std::fmod(pan.y, step); y < size.y; y += step)
                    draw->AddLine(ImVec2(origin.x, origin.y + y),
                                  ImVec2(origin.x + size.x, origin.y + y), line);
            }

            void draw_canvas(GraphState& g)
            {
                ImGui::BeginChild("graph", ImVec2(0.0f, 0.0f), true,
                                  ImGuiWindowFlags_NoScrollWithMouse | ImGuiWindowFlags_NoMove);
                LayerDesc& layer = current_layer(g);
                ensure_positions(g);

                const ImVec2 origin = ImGui::GetCursorScreenPos();
                const ImVec2 size = ImGui::GetContentRegionAvail();
                ImDrawList* draw = ImGui::GetWindowDrawList();
                draw->AddRectFilled(origin, origin + size, IM_COL32(24, 24, 28, 255));

                // Zoom-scaled metrics: positions are graph-space, everything drawn is × zoom.
                const float zoom = g.zoom;
                const float nw = NODE_W * zoom;
                const float nh = NODE_H * zoom;
                const float head = 28.0f * zoom;
                const float rounding = 6.0f * zoom;
                ImFont* font = ImGui::GetFont();
                const float font_size = ImGui::GetFontSize() * zoom;
                draw_grid(draw, origin, size, g.pan, 24.0f * zoom);

                const auto g2s = [&](const ImVec2& p)
                { return ImVec2(origin.x + g.pan.x + p.x * zoom, origin.y + g.pan.y + p.y * zoom); };
                const auto text = [&](const ImVec2& pos, ImU32 col, const char* s)
                { draw->AddText(font, font_size, pos, col, s); };
                const auto node_pos = [&](int i) { return g2s(g.positions[i]); };
                const auto node_center = [&](int i)
                { return node_pos(i) + ImVec2(nw * 0.5f, nh * 0.5f); };
                const ImVec2 entry_screen = g2s(g.entry_pos);
                const ImVec2 exit_screen = g2s(g.exit_pos);
                const ImVec2 entry_center = entry_screen + ImVec2(nw * 0.5f, head * 0.5f);
                const ImVec2 exit_center = exit_screen + ImVec2(nw * 0.5f, head * 0.5f);
                const int count = static_cast<int>(layer.states.size());
                const int default_index = state_index(layer, layer.default_state);

                // Hit-tests the transition whose line is nearest the mouse (state index + slot).
                const auto hit_transition = [&](const ImVec2& mouse, int& out_state, int& out_t) -> bool
                {
                    for (int i = 0; i < count; ++i)
                        for (int t = 0; t < static_cast<int>(layer.states[i].transitions.size()); ++t)
                        {
                            const std::string& dest = layer.states[i].transitions[t].destination;
                            const int d = state_index(layer, dest);
                            if (d < 0 && dest != "Exit")
                                continue;
                            const ImVec2 to = d >= 0 ? node_center(d) : exit_center;
                            if (d == i)
                                continue;
                            if (segment_distance(mouse, node_center(i), to) < 6.0f)
                            {
                                out_state = i;
                                out_t = t;
                                return true;
                            }
                        }
                    return false;
                };

                // Entry arrow to the default state.
                if (default_index >= 0)
                    draw_arrow(draw, entry_center, node_center(default_index),
                               IM_COL32(120, 210, 140, 220));

                // Transitions (drawn under the nodes). Exit transitions point at the Exit node.
                for (int i = 0; i < count; ++i)
                    for (int t = 0; t < static_cast<int>(layer.states[i].transitions.size()); ++t)
                    {
                        const std::string& dest = layer.states[i].transitions[t].destination;
                        const int d = state_index(layer, dest);
                        ImVec2 to;
                        if (d >= 0)
                        {
                            if (d == i)
                                continue;
                            to = node_center(d);
                        }
                        else if (dest == "Exit")
                        {
                            to = exit_center;
                        }
                        else
                        {
                            continue;
                        }
                        const bool sel = g.selected_transition_state == i && g.selected_transition == t;
                        draw_arrow(draw, node_center(i), to,
                                   sel ? IM_COL32(255, 200, 80, 255) : IM_COL32(150, 170, 200, 220));
                    }

                // Nodes.
                int pending_delete = -1;
                for (int i = 0; i < count; ++i)
                {
                    const ImVec2 p = node_pos(i);
                    ImGui::PushID(i);
                    ImGui::SetCursorScreenPos(p);
                    ImGui::InvisibleButton("node", ImVec2(nw, nh));
                    if (ImGui::IsItemActive() && ImGui::IsMouseDragging(ImGuiMouseButton_Left))
                    {
                        const ImVec2 delta = ImGui::GetIO().MouseDelta;
                        g.positions[i] = g.positions[i] + ImVec2(delta.x / zoom, delta.y / zoom);
                    }
                    if (ImGui::IsItemActivated())
                    {
                        if (g.linking && g.link_source != i)
                        {
                            TransitionDesc transition;
                            transition.destination = layer.states[i].name;
                            transition.duration = 0.1f;
                            if (g.link_source == -2)
                                layer.any_state_transitions.push_back(transition);
                            else if (g.link_source >= 0 && g.link_source < count)
                                layer.states[g.link_source].transitions.push_back(transition);
                            g.linking = false;
                        }
                        else
                        {
                            g.selected_state = i;
                        }
                    }
                    // Right-click context menu: transitions, default, delete.
                    if (ImGui::BeginPopupContextItem("node_ctx"))
                    {
                        g.selected_state = i;
                        if (ImGui::BeginMenu("Make Transition To"))
                        {
                            for (int j = 0; j < count; ++j)
                                if (j != i && ImGui::MenuItem(layer.states[j].name.c_str()))
                                {
                                    TransitionDesc t;
                                    t.destination = layer.states[j].name;
                                    t.duration = 0.1f;
                                    layer.states[i].transitions.push_back(t);
                                }
                            ImGui::Separator();
                            if (ImGui::MenuItem("Exit"))
                            {
                                TransitionDesc t;
                                t.destination = "Exit";
                                layer.states[i].transitions.push_back(t);
                            }
                            ImGui::EndMenu();
                        }
                        if (ImGui::MenuItem("Set as Default (Entry)"))
                            layer.default_state = layer.states[i].name;
                        if (ImGui::MenuItem("Delete State"))
                            pending_delete = i;
                        ImGui::EndPopup();
                    }
                    const bool is_default = layer.states[i].name == layer.default_state;
                    const bool is_selected = g.selected_state == i;
                    const ImU32 fill = is_default ? IM_COL32(46, 74, 52, 255) : IM_COL32(48, 52, 62, 255);
                    const ImU32 border = is_selected ? IM_COL32(255, 200, 80, 255)
                                                     : IM_COL32(90, 100, 120, 255);
                    draw->AddRectFilled(p, p + ImVec2(nw, nh), fill, rounding);
                    draw->AddRect(p, p + ImVec2(nw, nh), border, rounding, 0, 2.0f);
                    text(p + ImVec2(8.0f * zoom, 6.0f * zoom), IM_COL32(235, 235, 235, 255),
                         layer.states[i].name.c_str());
                    if (layer.states[i].blend_tree)
                        text(p + ImVec2(nw - 22.0f * zoom, 6.0f * zoom),
                             IM_COL32(150, 200, 255, 255), "BT");
                    if (is_default)
                        text(p + ImVec2(8.0f * zoom, nh - 18.0f * zoom), IM_COL32(150, 220, 160, 255),
                             "default");

                    // Output nub on the right edge (a fixed-size screen handle, easy to grab at
                    // any zoom): drag from it to a target node to connect.
                    const ImVec2 nub = p + ImVec2(nw, nh * 0.5f);
                    ImGui::SetCursorScreenPos(nub - ImVec2(7.0f, 7.0f));
                    ImGui::InvisibleButton("nub", ImVec2(14.0f, 14.0f));
                    if (ImGui::IsItemActivated())
                    {
                        g.linking = true;
                        g.link_from_drag = true;
                        g.link_source = i;
                    }
                    draw->AddCircleFilled(nub, 5.0f, IM_COL32(210, 200, 120, 255));
                    ImGui::PopID();
                }

                // Complete a dragged wire on release over a target node (or the Exit node).
                if (g.linking && g.link_from_drag && ImGui::IsMouseReleased(ImGuiMouseButton_Left))
                {
                    const ImVec2 m = ImGui::GetIO().MousePos;
                    const auto inside = [&](const ImVec2& a, const ImVec2& b)
                    { return m.x >= a.x && m.x <= b.x && m.y >= a.y && m.y <= b.y; };
                    int target = -1;
                    for (int i = 0; i < count; ++i)
                    {
                        const ImVec2 a = node_pos(i);
                        if (inside(a, a + ImVec2(nw, nh)))
                        {
                            target = i;
                            break;
                        }
                    }
                    const bool over_exit = inside(exit_screen, exit_screen + ImVec2(nw, head));
                    if (g.link_source >= 0 && g.link_source < count &&
                        (target >= 0 || over_exit) && target != g.link_source)
                    {
                        TransitionDesc transition;
                        transition.destination = over_exit ? std::string("Exit")
                                                           : layer.states[target].name;
                        transition.duration = 0.1f;
                        layer.states[g.link_source].transitions.push_back(transition);
                    }
                    g.linking = false;
                    g.link_from_drag = false;
                }

                // Entry node (green), draggable.
                ImGui::SetCursorScreenPos(entry_screen);
                ImGui::InvisibleButton("entry", ImVec2(nw, head));
                if (ImGui::IsItemActive() && ImGui::IsMouseDragging(ImGuiMouseButton_Left))
                {
                    const ImVec2 delta = ImGui::GetIO().MouseDelta;
                    g.entry_pos = g.entry_pos + ImVec2(delta.x / zoom, delta.y / zoom);
                }
                draw->AddRectFilled(entry_screen, entry_screen + ImVec2(nw, head),
                                    IM_COL32(40, 80, 48, 255), rounding);
                text(entry_screen + ImVec2(8.0f * zoom, 6.0f * zoom), IM_COL32(160, 230, 170, 255),
                     "Entry");

                // Exit node (red), draggable.
                ImGui::SetCursorScreenPos(exit_screen);
                ImGui::InvisibleButton("exit", ImVec2(nw, head));
                if (ImGui::IsItemActive() && ImGui::IsMouseDragging(ImGuiMouseButton_Left))
                {
                    const ImVec2 delta = ImGui::GetIO().MouseDelta;
                    g.exit_pos = g.exit_pos + ImVec2(delta.x / zoom, delta.y / zoom);
                }
                draw->AddRectFilled(exit_screen, exit_screen + ImVec2(nw, head),
                                    IM_COL32(90, 40, 44, 255), rounding);
                text(exit_screen + ImVec2(8.0f * zoom, 6.0f * zoom), IM_COL32(240, 160, 160, 255),
                     "Exit");

                // Any-State node (an amber source you can link transitions from).
                const ImVec2 any_p = g2s(ImVec2(200.0f, -6.0f));
                ImGui::SetCursorScreenPos(any_p);
                ImGui::InvisibleButton("anystate", ImVec2(nw, head));
                if (ImGui::IsItemActivated())
                {
                    g.linking = true;
                    g.link_source = -2;
                }
                draw->AddRectFilled(any_p, any_p + ImVec2(nw, head), IM_COL32(70, 60, 40, 255),
                                    rounding);
                text(any_p + ImVec2(8.0f * zoom, 6.0f * zoom), IM_COL32(230, 210, 160, 255),
                     "Any State");

                // Canvas-level interaction — no full-canvas button, so dragging a node (an item on
                // top) is never stolen. Pan, transition selection, and the canvas menu key off
                // "over the window but not over any node".
                const bool window_hovered = ImGui::IsWindowHovered();
                const bool any_active = ImGui::IsAnyItemActive();
                const bool any_hovered = ImGui::IsAnyItemHovered();
                const ImVec2 mouse = ImGui::GetIO().MousePos;

                if (window_hovered && !any_active && ImGui::IsMouseDragging(ImGuiMouseButton_Middle))
                    g.pan = g.pan + ImGui::GetIO().MouseDelta;

                // Scroll wheel zooms, keeping the graph point under the cursor fixed.
                const float wheel = ImGui::GetIO().MouseWheel;
                if (window_hovered && wheel != 0.0f)
                {
                    const float old_zoom = g.zoom;
                    float next = old_zoom * std::pow(1.15f, wheel);
                    next = std::max(0.35f, std::min(2.5f, next));
                    const float gx = (mouse.x - origin.x - g.pan.x) / old_zoom;
                    const float gy = (mouse.y - origin.y - g.pan.y) / old_zoom;
                    g.pan.x = mouse.x - origin.x - gx * next;
                    g.pan.y = mouse.y - origin.y - gy * next;
                    g.zoom = next;
                }

                // Left-click empty space: select the transition line under the cursor, else clear.
                if (window_hovered && !any_hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
                {
                    int hs = -1;
                    int ht = -1;
                    if (hit_transition(mouse, hs, ht))
                    {
                        g.selected_transition_state = hs;
                        g.selected_transition = ht;
                        g.selected_state = -1;
                    }
                    else
                    {
                        g.selected_transition_state = -1;
                        g.selected_transition = -1;
                        g.selected_state = -1;
                        g.linking = false;
                    }
                }
                // Right-click empty space: delete the transition line under the cursor, else the
                // "Add State Here" menu.
                if (window_hovered && !any_hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Right))
                {
                    int hs = -1;
                    int ht = -1;
                    if (hit_transition(mouse, hs, ht))
                    {
                        std::vector<TransitionDesc>& list = layer.states[hs].transitions;
                        if (ht < static_cast<int>(list.size()))
                            list.erase(list.begin() + ht);
                        g.selected_transition_state = -1;
                        g.selected_transition = -1;
                    }
                    else
                    {
                        g.context_menu_pos = ImVec2((mouse.x - origin.x - g.pan.x) / zoom,
                                                    (mouse.y - origin.y - g.pan.y) / zoom);
                        ImGui::OpenPopup("bg_ctx");
                    }
                }
                if (ImGui::BeginPopup("bg_ctx"))
                {
                    if (ImGui::MenuItem("Add State Here"))
                    {
                        StateDesc s;
                        s.name = "State " + std::to_string(layer.states.size());
                        layer.states.push_back(s);
                        if (layer.default_state.empty())
                            layer.default_state = s.name;
                        ensure_positions(g);
                        g.positions.back() = g.context_menu_pos;
                    }
                    ImGui::EndPopup();
                }

                if (g.linking)
                {
                    const ImVec2 from =
                        g.link_source == -2
                            ? any_p + ImVec2(nw * 0.5f, head * 0.5f)
                            : (g.link_source >= 0 && g.link_source < count
                                   ? node_pos(g.link_source) + ImVec2(nw, nh * 0.5f)
                                   : mouse);
                    draw->AddLine(from, mouse, IM_COL32(255, 200, 80, 200), 2.0f);
                }

                if (pending_delete >= 0)
                    delete_state(g, pending_delete);

                ImGui::EndChild();
            }
        } // namespace

        void draw_animator_graph_panel(EditorContext& context)
        {
            if (!context.panels.animator_graph)
                return;
            if (!ImGui::Begin("Animator", &context.panels.animator_graph))
            {
                ImGui::End();
                return;
            }

            GraphState& g = state();
            seed(g);

            if (ImGui::Button("Add State"))
            {
                LayerDesc& layer = current_layer(g);
                StateDesc s;
                s.name = "State " + std::to_string(layer.states.size());
                layer.states.push_back(s);
                if (layer.default_state.empty())
                    layer.default_state = s.name;
                ensure_positions(g);
            }
            ImGui::SameLine();
            const bool can_delete_state = g.selected_state >= 0;
            if (!can_delete_state)
                ImGui::BeginDisabled();
            if (ImGui::Button("Delete State"))
                delete_state(g, g.selected_state);
            if (!can_delete_state)
                ImGui::EndDisabled();
            ImGui::SameLine();
            const bool can_delete_transition = g.selected_transition >= 0;
            if (!can_delete_transition)
                ImGui::BeginDisabled();
            if (ImGui::Button("Delete Transition"))
            {
                LayerDesc& layer = current_layer(g);
                if (g.selected_transition_state >= 0 &&
                    g.selected_transition_state < static_cast<int>(layer.states.size()))
                {
                    std::vector<TransitionDesc>& list =
                        layer.states[g.selected_transition_state].transitions;
                    if (g.selected_transition < static_cast<int>(list.size()))
                        list.erase(list.begin() + g.selected_transition);
                }
                g.selected_transition = -1;
                g.selected_transition_state = -1;
            }
            if (!can_delete_transition)
                ImGui::EndDisabled();
            ImGui::SameLine();
            ImGui::TextDisabled("(Drag a node to move. Drag from its right nub to a node to link. "
                                "Click a line to select, right-click or Delete to remove. "
                                "Middle-drag to pan, scroll to zoom.)");

            // Delete key removes the selected transition, else the selected state.
            if (ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows) &&
                ImGui::IsKeyPressed(ImGuiKey_Delete, false))
            {
                LayerDesc& layer = current_layer(g);
                if (g.selected_transition >= 0 && g.selected_transition_state >= 0 &&
                    g.selected_transition_state < static_cast<int>(layer.states.size()))
                {
                    std::vector<TransitionDesc>& list =
                        layer.states[g.selected_transition_state].transitions;
                    if (g.selected_transition < static_cast<int>(list.size()))
                        list.erase(list.begin() + g.selected_transition);
                    g.selected_transition = -1;
                    g.selected_transition_state = -1;
                }
                else if (g.selected_state >= 0)
                {
                    delete_state(g, g.selected_state);
                }
            }

            ImGui::Separator();
            draw_side(g);
            ImGui::SameLine();
            draw_canvas(g);

            ImGui::End();
        }
    } // namespace Editor
} // namespace SushiEngine
