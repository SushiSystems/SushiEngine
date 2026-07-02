/**************************************************************************/
/* editor_panels.cpp                                                      */
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

#include "editor_panels.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <system_error>

#include <filesystem>

#include <imgui.h>
#include <imgui_internal.h>
#include <misc/cpp/imgui_stdlib.h>

namespace fs = std::filesystem;

namespace sushi::editor
{
    using SushiEngine::sim::EntityId;
    using SushiEngine::sim::EntityTransform;
    using SushiEngine::sim::IWorldEditor;
    using SushiEngine::sim::NULL_ENTITY;

    namespace
    {
        // A file is opened into the text editor only when it looks textual; the
        // browser still lists everything, but double-clicking a binary is a no-op.
        bool has_text_extension(const fs::path& path)
        {
            static const char* kTextExtensions[] = {
                ".txt", ".md", ".cpp", ".hpp", ".h", ".c", ".cc", ".inl",
                ".cmake", ".toml", ".ini", ".json", ".yaml", ".yml", ".glsl",
                ".frag", ".vert", ".comp", ".py", ".sh", ".bat", ".xml", ".cfg"};

            std::string ext = path.extension().string();
            std::transform(ext.begin(), ext.end(), ext.begin(),
                           [](unsigned char c) { return static_cast<char>(::tolower(c)); });
            for (const char* candidate : kTextExtensions)
            {
                if (ext == candidate)
                    return true;
            }
            return path.has_filename() && !path.has_extension();
        }

        // Load a file into an open Document, or focus it if already open. Files that
        // fail to read are silently skipped rather than opening an empty buffer that
        // a later save would clobber over the real (unreadable) contents.
        void open_document(EditorContext& context, const fs::path& path)
        {
            const std::string path_string = path.string();
            for (std::size_t i = 0; i < context.documents.size(); ++i)
            {
                if (context.documents[i].path == path_string)
                {
                    context.active_document = static_cast<int>(i);
                    return;
                }
            }

            std::ifstream stream(path, std::ios::binary);
            if (!stream)
                return;

            std::ostringstream buffer;
            buffer << stream.rdbuf();

            Document document;
            document.path = path_string;
            document.display_name = path.filename().string();
            document.text = buffer.str();
            context.documents.push_back(std::move(document));
            context.active_document = static_cast<int>(context.documents.size() - 1);
        }

        // Persist a document's buffer to its path and clear the dirty flag on success.
        void save_document(Document& document)
        {
            std::ofstream stream(document.path, std::ios::binary | std::ios::trunc);
            if (!stream)
                return;
            stream << document.text;
            document.dirty = false;
        }

        std::string to_lower(const std::string& text)
        {
            std::string out = text;
            std::transform(out.begin(), out.end(), out.begin(),
                           [](unsigned char c) { return static_cast<char>(::tolower(c)); });
            return out;
        }

        // A labelled drag-float row for a 3-component vector, matching Unity's
        // X/Y/Z inspector rows. Returns true when any component changed this frame.
        bool vector3_field(const char* label, float values[3], float speed)
        {
            ImGui::PushID(label);
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted(label);
            ImGui::TableSetColumnIndex(1);
            ImGui::SetNextItemWidth(-FLT_MIN);
            const bool changed = ImGui::DragFloat3("##v", values, speed);
            ImGui::PopID();
            return changed;
        }

        // Editor-facing rotation presentation: the world stores a quaternion, but the
        // inspector edits Euler degrees like Unity. These conversions are a display
        // concern local to the panel, not part of the engine's math seam.
        void quat_to_euler_degrees(const SushiEngine::Quat& q, float out[3])
        {
            constexpr float RAD_TO_DEG = 57.2957795f;
            const float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
            const float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
            const float roll = std::atan2(sinr_cosp, cosr_cosp);

            const float sinp = 2.0f * (q.w * q.y - q.z * q.x);
            const float pitch = std::fabs(sinp) >= 1.0f
                                    ? std::copysign(1.5707963f, sinp)
                                    : std::asin(sinp);

            const float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
            const float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
            const float yaw = std::atan2(siny_cosp, cosy_cosp);

            out[0] = roll * RAD_TO_DEG;
            out[1] = pitch * RAD_TO_DEG;
            out[2] = yaw * RAD_TO_DEG;
        }

        SushiEngine::Quat euler_degrees_to_quat(const float in[3])
        {
            constexpr float DEG_TO_RAD = 0.01745329f;
            const float roll = in[0] * DEG_TO_RAD;
            const float pitch = in[1] * DEG_TO_RAD;
            const float yaw = in[2] * DEG_TO_RAD;

            const float cr = std::cos(roll * 0.5f), sr = std::sin(roll * 0.5f);
            const float cp = std::cos(pitch * 0.5f), sp = std::sin(pitch * 0.5f);
            const float cy = std::cos(yaw * 0.5f), sy = std::sin(yaw * 0.5f);

            SushiEngine::Quat q;
            q.w = cr * cp * cy + sr * sp * sy;
            q.x = sr * cp * cy - cr * sp * sy;
            q.y = cr * sp * cy + sr * cp * sy;
            q.z = cr * cp * sy - sr * sp * cy;
            return q;
        }

        // The world's editor surface, or nullptr before the simulation is injected.
        IWorldEditor* world_of(EditorContext& context)
        {
            return context.simulation != nullptr ? &context.simulation->world() : nullptr;
        }
    }

    void draw_menu_bar(EditorContext& context, bool& running)
    {
        if (!ImGui::BeginMainMenuBar())
            return;

        IWorldEditor* world = world_of(context);

        if (ImGui::BeginMenu("File"))
        {
            if (ImGui::MenuItem("New Entity", "Ctrl+N", false, world != nullptr))
            {
                context.selected_entity = world->create("Entity");
                editor_log(context, "Created entity 'Entity'.");
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Save", "Ctrl+S", false,
                                context.active_document >= 0))
            {
                Document& document = context.documents[static_cast<std::size_t>(
                    context.active_document)];
                save_document(document);
                editor_log(context, "Saved '" + document.display_name + "'.");
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Exit", "Alt+F4"))
                running = false;
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Edit"))
        {
            if (ImGui::MenuItem("Preferences...", nullptr))
                context.show_preferences = true;
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("GameObject"))
        {
            if (ImGui::MenuItem("Create Empty", nullptr, false, world != nullptr))
            {
                context.selected_entity = world->create("Entity");
                editor_log(context, "Created entity 'Entity'.");
            }
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Window"))
        {
            ImGui::MenuItem("Scene", nullptr, &context.panels.scene_view);
            ImGui::MenuItem("Game", nullptr, &context.panels.game_view);
            ImGui::MenuItem("Hierarchy", nullptr, &context.panels.hierarchy);
            ImGui::MenuItem("Inspector", nullptr, &context.panels.inspector);
            ImGui::MenuItem("Project", nullptr, &context.panels.project);
            ImGui::MenuItem("Text Editor", nullptr, &context.panels.text_editor);
            ImGui::MenuItem("Console", nullptr, &context.panels.console);
            ImGui::MenuItem("Statistics", nullptr, &context.panels.statistics);
            ImGui::MenuItem("Toolbar", nullptr, &context.panels.toolbar);
            ImGui::Separator();
            ImGui::MenuItem("ImGui Demo", nullptr, &context.show_imgui_demo);
            ImGui::EndMenu();
        }

        ImGui::EndMainMenuBar();
    }

    void draw_hierarchy_panel(EditorContext& context)
    {
        if (!context.panels.hierarchy)
            return;
        if (!ImGui::Begin("Hierarchy", &context.panels.hierarchy))
        {
            ImGui::End();
            return;
        }

        IWorldEditor* world = world_of(context);
        if (world == nullptr)
        {
            ImGui::TextDisabled("No world.");
            ImGui::End();
            return;
        }

        if (ImGui::Button("Add Entity"))
        {
            context.selected_entity = world->create("Entity");
            editor_log(context, "Created entity 'Entity'.");
        }
        ImGui::SameLine();
        ImGui::BeginDisabled(context.selected_entity == NULL_ENTITY);
        if (ImGui::Button("Delete"))
        {
            world->destroy(context.selected_entity);
            context.selected_entity = NULL_ENTITY;
        }
        ImGui::EndDisabled();

        ImGui::SetNextItemWidth(-FLT_MIN);
        ImGui::InputTextWithHint("##filter", "Search...", &context.hierarchy_filter);
        ImGui::Separator();

        const std::string lower_filter = to_lower(context.hierarchy_filter);

        // The entity being deleted this frame is deferred out of the loop so the
        // entity list is never mutated while it is being walked.
        EntityId delete_target = NULL_ENTITY;

        if (ImGui::BeginChild("entities"))
        {
            for (const EntityId id : world->entities())
            {
                const std::string entity_name = world->name(id);
                if (!lower_filter.empty() &&
                    to_lower(entity_name).find(lower_filter) == std::string::npos)
                    continue;

                ImGui::PushID(static_cast<int>(id));

                if (context.renaming_entity == id)
                {
                    // Inline rename: an autofocused field seeded from the name, seeded
                    // once as the target changes, committed on Enter or focus loss.
                    static std::string buffer;
                    static EntityId active = NULL_ENTITY;
                    if (active != id)
                    {
                        buffer = entity_name;
                        active = id;
                        ImGui::SetKeyboardFocusHere();
                    }
                    ImGui::SetNextItemWidth(-FLT_MIN);
                    const bool commit = ImGui::InputText(
                        "##rename", &buffer,
                        ImGuiInputTextFlags_EnterReturnsTrue |
                            ImGuiInputTextFlags_AutoSelectAll);
                    if (commit || ImGui::IsItemDeactivated())
                    {
                        world->set_name(id, buffer);
                        context.renaming_entity = NULL_ENTITY;
                        active = NULL_ENTITY;
                    }
                }
                else
                {
                    const bool selected = context.selected_entity == id;
                    if (ImGui::Selectable(entity_name.c_str(), selected,
                                          ImGuiSelectableFlags_AllowDoubleClick))
                    {
                        context.selected_entity = id;
                        if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
                            context.renaming_entity = id;
                    }
                    if (ImGui::BeginPopupContextItem())
                    {
                        context.selected_entity = id;
                        if (ImGui::MenuItem("Rename"))
                            context.renaming_entity = id;
                        if (ImGui::MenuItem("Delete"))
                            delete_target = id;
                        ImGui::EndPopup();
                    }
                }

                ImGui::PopID();
            }
        }
        ImGui::EndChild();

        if (delete_target != NULL_ENTITY)
        {
            if (context.selected_entity == delete_target)
                context.selected_entity = NULL_ENTITY;
            world->destroy(delete_target);
        }

        ImGui::End();
    }

    void draw_inspector_panel(EditorContext& context)
    {
        if (!context.panels.inspector)
            return;
        if (!ImGui::Begin("Inspector", &context.panels.inspector))
        {
            ImGui::End();
            return;
        }

        IWorldEditor* world = world_of(context);
        if (world == nullptr || !world->exists(context.selected_entity))
        {
            ImGui::TextDisabled("Nothing selected.");
            ImGui::End();
            return;
        }

        const EntityId id = context.selected_entity;

        bool visible = world->visible(id);
        if (ImGui::Checkbox("##visible", &visible))
            world->set_visible(id, visible);
        ImGui::SameLine();
        std::string name = world->name(id);
        ImGui::SetNextItemWidth(-FLT_MIN);
        if (ImGui::InputText("##name", &name))
            world->set_name(id, name);
        ImGui::Text("Id: %llu", static_cast<unsigned long long>(id));
        ImGui::Separator();

        if (ImGui::CollapsingHeader("Transform", ImGuiTreeNodeFlags_DefaultOpen))
        {
            EntityTransform transform = world->transform(id);
            // ImGui edits at float precision; the components are engine Scalar (float
            // or double per build), so narrow explicitly into the widget buffers and
            // widen back on write.
            float position[3] = {static_cast<float>(transform.position.x),
                                 static_cast<float>(transform.position.y),
                                 static_cast<float>(transform.position.z)};
            float rotation[3];
            quat_to_euler_degrees(transform.rotation, rotation);
            float scale[3] = {static_cast<float>(transform.scale.x),
                              static_cast<float>(transform.scale.y),
                              static_cast<float>(transform.scale.z)};

            bool changed = false;
            if (ImGui::BeginTable("transform", 2, ImGuiTableFlags_SizingStretchProp))
            {
                ImGui::TableSetupColumn("label", ImGuiTableColumnFlags_WidthFixed, 80.0f);
                ImGui::TableSetupColumn("value");
                changed |= vector3_field("Position", position, 0.05f);
                changed |= vector3_field("Rotation", rotation, 0.5f);
                changed |= vector3_field("Scale", scale, 0.05f);
                ImGui::EndTable();
            }

            if (changed)
            {
                transform.position = SushiEngine::Vec3{position[0], position[1], position[2]};
                transform.rotation = euler_degrees_to_quat(rotation);
                transform.scale = SushiEngine::Vec3{scale[0], scale[1], scale[2]};
                world->set_transform(id, transform);
            }
        }

        if (ImGui::CollapsingHeader("Renderer", ImGuiTreeNodeFlags_DefaultOpen))
        {
            const SushiEngine::Vec3 current = world->color(id);
            float color[3] = {static_cast<float>(current.x), static_cast<float>(current.y),
                              static_cast<float>(current.z)};
            if (ImGui::ColorEdit3("Color", color))
                world->set_color(id, SushiEngine::Vec3{color[0], color[1], color[2]});
        }

        ImGui::End();
    }

    void draw_project_panel(EditorContext& context)
    {
        if (!context.panels.project)
            return;
        if (!ImGui::Begin("Project", &context.panels.project))
        {
            ImGui::End();
            return;
        }

        const fs::path root(context.project_root);
        fs::path current(context.current_directory);

        ImGui::TextDisabled("%s", current.string().c_str());
        ImGui::Separator();

        const bool at_root =
            fs::weakly_canonical(current) == fs::weakly_canonical(root);
        ImGui::BeginDisabled(at_root);
        if (ImGui::Selectable("..", false, ImGuiSelectableFlags_DontClosePopups))
            context.current_directory = current.parent_path().string();
        ImGui::EndDisabled();

        std::error_code ec;
        std::vector<fs::directory_entry> entries;
        for (const auto& entry : fs::directory_iterator(current, ec))
            entries.push_back(entry);

        // Directories first, then files; each group alphabetical — the ordering a
        // filesystem browser is expected to present regardless of iteration order.
        std::sort(entries.begin(), entries.end(),
                  [](const fs::directory_entry& a, const fs::directory_entry& b)
                  {
                      const bool a_dir = a.is_directory();
                      const bool b_dir = b.is_directory();
                      if (a_dir != b_dir)
                          return a_dir;
                      return a.path().filename().string() <
                             b.path().filename().string();
                  });

        for (const auto& entry : entries)
        {
            const bool is_dir = entry.is_directory();
            const std::string name =
                (is_dir ? "[D] " : "     ") + entry.path().filename().string();

            if (ImGui::Selectable(name.c_str()) &&
                ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
            {
                if (is_dir)
                    context.current_directory = entry.path().string();
                else if (has_text_extension(entry.path()))
                    open_document(context, entry.path());
            }
        }

        ImGui::End();
    }

    void draw_text_editor_panel(EditorContext& context)
    {
        if (!context.panels.text_editor)
            return;
        if (!ImGui::Begin("Text Editor", &context.panels.text_editor))
        {
            ImGui::End();
            return;
        }

        if (context.documents.empty())
        {
            ImGui::TextDisabled("Open a file from the Project panel.");
            ImGui::End();
            return;
        }

        if (ImGui::BeginTabBar("documents", ImGuiTabBarFlags_Reorderable |
                                                ImGuiTabBarFlags_AutoSelectNewTabs))
        {
            for (std::size_t i = 0; i < context.documents.size();)
            {
                Document& document = context.documents[i];
                bool open = true;

                ImGui::PushID(static_cast<int>(i));
                std::string title = document.display_name;
                if (document.dirty)
                    title += " *";
                title += "###doc";

                ImGuiTabItemFlags tab_flags =
                    document.dirty ? ImGuiTabItemFlags_UnsavedDocument : 0;
                if (ImGui::BeginTabItem(title.c_str(), &open, tab_flags))
                {
                    context.active_document = static_cast<int>(i);

                    if (ImGui::Button("Save"))
                    {
                        save_document(document);
                        editor_log(context, "Saved '" + document.display_name + "'.");
                    }
                    ImGui::SameLine();
                    ImGui::TextDisabled("%s", document.path.c_str());

                    const ImVec2 size(-FLT_MIN, -FLT_MIN);
                    if (ImGui::InputTextMultiline("##text", &document.text, size,
                                                  ImGuiInputTextFlags_AllowTabInput))
                        document.dirty = true;

                    ImGui::EndTabItem();
                }
                ImGui::PopID();

                if (!open)
                {
                    context.documents.erase(context.documents.begin() +
                                            static_cast<std::ptrdiff_t>(i));
                    if (context.active_document >= static_cast<int>(i))
                        --context.active_document;
                }
                else
                {
                    ++i;
                }
            }
            ImGui::EndTabBar();
        }

        ImGui::End();
    }

    void draw_toolbar_panel(EditorContext& context)
    {
        if (!context.panels.toolbar)
            return;
        if (!ImGui::Begin("Toolbar", &context.panels.toolbar))
        {
            ImGui::End();
            return;
        }

        const bool playing = context.play_state == PlayState::Playing;
        if (ImGui::Button(playing ? "Stop" : "Play"))
        {
            if (playing || context.play_state == PlayState::Paused)
            {
                context.play_state = PlayState::Stopped;
                editor_log(context, "Playback stopped.");
            }
            else
            {
                context.play_state = PlayState::Playing;
                editor_log(context, "Playback started.");
            }
        }
        ImGui::SameLine();

        ImGui::BeginDisabled(context.play_state == PlayState::Stopped);
        if (ImGui::Button(context.play_state == PlayState::Paused ? "Resume"
                                                                  : "Pause"))
        {
            context.play_state = context.play_state == PlayState::Paused
                                     ? PlayState::Playing
                                     : PlayState::Paused;
            editor_log(context, context.play_state == PlayState::Paused
                                    ? "Playback paused."
                                    : "Playback resumed.");
        }
        ImGui::SameLine();
        if (ImGui::Button("Step"))
            editor_log(context, "Stepped one frame.");
        ImGui::EndDisabled();

        ImGui::SameLine();
        const char* state_text = context.play_state == PlayState::Playing ? "Playing"
                                 : context.play_state == PlayState::Paused ? "Paused"
                                                                           : "Stopped";
        ImGui::TextDisabled("| %s", state_text);

        ImGui::End();
    }

    void draw_console_panel(EditorContext& context)
    {
        if (!context.panels.console)
            return;
        if (!ImGui::Begin("Console", &context.panels.console))
        {
            ImGui::End();
            return;
        }

        if (ImGui::Button("Clear"))
            context.console_lines.clear();
        ImGui::SameLine();
        ImGui::TextDisabled("%zu message(s)", context.console_lines.size());
        ImGui::Separator();

        if (ImGui::BeginChild("scroll", ImVec2(0.0f, 0.0f), false,
                              ImGuiWindowFlags_HorizontalScrollbar))
        {
            for (const std::string& line : context.console_lines)
                ImGui::TextUnformatted(line.c_str());
            if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY())
                ImGui::SetScrollHereY(1.0f);
        }
        ImGui::EndChild();

        ImGui::End();
    }

    void draw_statistics_panel(EditorContext& context)
    {
        if (!context.panels.statistics)
            return;
        if (!ImGui::Begin("Statistics", &context.panels.statistics))
        {
            ImGui::End();
            return;
        }

        const ImGuiIO& io = ImGui::GetIO();
        ImGui::Text("Frame: %.2f ms", 1000.0f / io.Framerate);
        ImGui::Text("FPS:   %.0f", io.Framerate);
        ImGui::Separator();
        ImGui::Text("World entities: %zu", context.world_entity_count);
        ImGui::Text("Open files:     %zu", context.documents.size());

        ImGui::End();
    }

    void draw_status_bar(EditorContext& context)
    {
        ImGuiViewport* viewport = ImGui::GetMainViewport();
        const float height = ImGui::GetFrameHeight();

        if (ImGui::BeginViewportSideBar("##StatusBar", viewport, ImGuiDir_Down, height,
                                        ImGuiWindowFlags_MenuBar))
        {
            if (ImGui::BeginMenuBar())
            {
                IWorldEditor* world = world_of(context);
                const bool has_selection =
                    world != nullptr && world->exists(context.selected_entity);
                ImGui::Text("Selected: %s",
                            has_selection ? world->name(context.selected_entity).c_str()
                                          : "none");
                ImGui::Separator();
                const char* state_text =
                    context.play_state == PlayState::Playing   ? "Playing"
                    : context.play_state == PlayState::Paused  ? "Paused"
                                                               : "Stopped";
                ImGui::Text("State: %s", state_text);
                ImGui::Separator();
                ImGui::Text("Entities: %zu", context.world_entity_count);
                ImGui::EndMenuBar();
            }
        }
        ImGui::End();
    }

    void apply_theme(EditorTheme theme)
    {
        switch (theme)
        {
            case EditorTheme::Light:   ImGui::StyleColorsLight(); break;
            case EditorTheme::Classic: ImGui::StyleColorsClassic(); break;
            case EditorTheme::Dark:    ImGui::StyleColorsDark(); break;
        }
    }

    void draw_preferences_window(EditorContext& context)
    {
        if (!context.show_preferences)
            return;

        ImGui::SetNextWindowSize(ImVec2(460.0f, 420.0f), ImGuiCond_FirstUseEver);
        if (!ImGui::Begin("Preferences", &context.show_preferences))
        {
            ImGui::End();
            return;
        }

        Preferences& preferences = context.preferences;
        bool changed = false;

        if (ImGui::CollapsingHeader("General", ImGuiTreeNodeFlags_DefaultOpen))
        {
            // Precision is a compile-time choice, so this records intent for the next
            // build rather than changing the running engine. Flag the mismatch.
            const char* precision_items[] = {"Single (float)", "Double"};
            int precision_index = preferences.precision == ScalarPrecision::Double ? 1 : 0;
            if (ImGui::Combo("Scalar precision", &precision_index, precision_items, 2))
            {
                preferences.precision =
                    precision_index == 1 ? ScalarPrecision::Double : ScalarPrecision::Single;
                changed = true;
            }
            if (preferences.precision != current_precision())
            {
                ImGui::TextColored(ImVec4(0.95f, 0.75f, 0.25f, 1.0f),
                                   "Rebuild required to apply precision.");
                ImGui::TextDisabled("Run: se editor %s",
                                    preferences.precision == ScalarPrecision::Double
                                        ? "--double"
                                        : "(without --double)");
            }
            else
            {
                ImGui::TextDisabled("Matches this build.");
            }

            const char* theme_items[] = {"Dark", "Light", "Classic"};
            int theme_index = static_cast<int>(preferences.theme);
            if (ImGui::Combo("Theme", &theme_index, theme_items, 3))
            {
                preferences.theme = static_cast<EditorTheme>(theme_index);
                apply_theme(preferences.theme);
                changed = true;
            }
        }

        if (ImGui::CollapsingHeader("Editor", ImGuiTreeNodeFlags_DefaultOpen))
        {
            changed |= ImGui::Checkbox("Autosave", &preferences.autosave);
            changed |= ImGui::DragFloat("Camera move speed", &preferences.camera_move_speed,
                                        0.1f, 0.1f, 100.0f, "%.1f");
        }

        if (ImGui::CollapsingHeader("Scene", ImGuiTreeNodeFlags_DefaultOpen))
        {
            changed |= ImGui::Checkbox("Show grid", &preferences.grid_visible);
            changed |= ImGui::Checkbox("Snap to grid", &preferences.snap_enabled);
            if (preferences.snap_enabled)
            {
                changed |= ImGui::DragFloat("Move snap", &preferences.snap_translate,
                                            0.01f, 0.001f, 10.0f, "%.3f");
                changed |= ImGui::DragFloat("Rotate snap (deg)", &preferences.snap_rotate_degrees,
                                            0.5f, 1.0f, 90.0f, "%.1f");
                changed |= ImGui::DragFloat("Scale snap", &preferences.snap_scale,
                                            0.01f, 0.001f, 10.0f, "%.3f");
            }
        }

        ImGui::Separator();
        if (context.preferences_store != nullptr)
            ImGui::TextDisabled("%s", context.preferences_store->path().c_str());

        if (changed)
            context.preferences_dirty = true;

        ImGui::End();
    }

    void build_default_layout(std::uint32_t dockspace_id)
    {
        ImGui::DockBuilderRemoveNode(dockspace_id);
        ImGui::DockBuilderAddNode(dockspace_id, ImGuiDockNodeFlags_DockSpace);
        ImGui::DockBuilderSetNodeSize(dockspace_id,
                                      ImGui::GetMainViewport()->WorkSize);

        ImGuiID center = dockspace_id;
        ImGuiID top = ImGui::DockBuilderSplitNode(center, ImGuiDir_Up, 0.06f,
                                                  nullptr, &center);
        ImGuiID left = ImGui::DockBuilderSplitNode(center, ImGuiDir_Left, 0.20f,
                                                   nullptr, &center);
        ImGuiID right = ImGui::DockBuilderSplitNode(center, ImGuiDir_Right, 0.25f,
                                                    nullptr, &center);
        ImGuiID bottom = ImGui::DockBuilderSplitNode(center, ImGuiDir_Down, 0.35f,
                                                     nullptr, &center);

        ImGui::DockBuilderDockWindow("Toolbar", top);
        // Scene and Game share the centre node, so they open tabbed like Unity.
        ImGui::DockBuilderDockWindow("Scene", center);
        ImGui::DockBuilderDockWindow("Game", center);
        ImGui::DockBuilderDockWindow("Hierarchy", left);
        ImGui::DockBuilderDockWindow("Inspector", right);
        ImGui::DockBuilderDockWindow("Statistics", right);
        ImGui::DockBuilderDockWindow("Project", bottom);
        ImGui::DockBuilderDockWindow("Text Editor", bottom);
        ImGui::DockBuilderDockWindow("Console", bottom);
        ImGui::DockBuilderFinish(dockspace_id);
    }
}
