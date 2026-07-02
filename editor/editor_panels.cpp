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
    namespace
    {
        // The payload id used to carry a dragged node's id between the hierarchy's
        // drag source and its drop targets. Kept local so nothing else can collide.
        constexpr const char* HIERARCHY_DRAG_PAYLOAD = "SUSHI_NODE";

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

        // A node survives the hierarchy filter if it, or any descendant, contains the
        // (already lower-cased) query — so filtering keeps ancestors visible as the
        // path to a match rather than hiding a parent whose child matched.
        bool subtree_matches(const SceneNode* node, const std::string& lower_query)
        {
            if (lower_query.empty())
                return true;
            if (to_lower(node->name).find(lower_query) != std::string::npos)
                return true;
            for (const auto& child : node->children)
            {
                if (subtree_matches(child.get(), lower_query))
                    return true;
            }
            return false;
        }

        int count_nodes(const std::vector<std::unique_ptr<SceneNode>>& nodes)
        {
            int total = 0;
            for (const auto& node : nodes)
                total += 1 + count_nodes(node->children);
            return total;
        }

        // Recursively draw one hierarchy node and its subtree, wiring selection,
        // the right-click context menu, and drag-and-drop reparenting. A deferred
        // (node, new_parent) request is returned via the out-params so the tree is
        // never mutated mid-walk, which would invalidate the iterators above us.
        void draw_hierarchy_node(EditorContext& context, SceneNode* node,
                                 const std::string& lower_filter,
                                 SceneNode*& reparent_child, SceneNode*& reparent_to,
                                 bool& reparent_requested, SceneNode*& delete_target,
                                 SceneNode*& add_child_target)
        {
            if (!subtree_matches(node, lower_filter))
                return;

            ImGui::PushID(static_cast<int>(node->id));

            // Inline rename: while this node is the rename target, an autofocused
            // text field stands in for the tree label. Committed on Enter or focus
            // loss; children stay visible under a manual indent so the tree shape is
            // preserved without a matching TreePop.
            if (context.renaming_node == node->id)
            {
                ImGui::SetNextItemWidth(-FLT_MIN);
                ImGui::SetKeyboardFocusHere();
                if (ImGui::InputText("##rename", &node->name,
                                     ImGuiInputTextFlags_EnterReturnsTrue |
                                         ImGuiInputTextFlags_AutoSelectAll) ||
                    ImGui::IsItemDeactivated())
                {
                    context.renaming_node = 0;
                }

                ImGui::Indent();
                for (auto& child : node->children)
                    draw_hierarchy_node(context, child.get(), lower_filter,
                                        reparent_child, reparent_to,
                                        reparent_requested, delete_target,
                                        add_child_target);
                ImGui::Unindent();
                ImGui::PopID();
                return;
            }

            ImGuiTreeNodeFlags flags =
                ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_SpanAvailWidth |
                ImGuiTreeNodeFlags_DefaultOpen;
            if (node->id == context.selected_node)
                flags |= ImGuiTreeNodeFlags_Selected;
            if (node->children.empty())
                flags |= ImGuiTreeNodeFlags_Leaf;

            const bool open = ImGui::TreeNodeEx(node->name.c_str(), flags);

            if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen())
                context.selected_node = node->id;
            if (ImGui::IsItemHovered() &&
                ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
            {
                context.renaming_node = node->id;
                context.selected_node = node->id;
            }

            if (ImGui::BeginDragDropSource())
            {
                ImGui::SetDragDropPayload(HIERARCHY_DRAG_PAYLOAD, &node, sizeof(node));
                ImGui::TextUnformatted(node->name.c_str());
                ImGui::EndDragDropSource();
            }
            if (ImGui::BeginDragDropTarget())
            {
                if (const ImGuiPayload* payload =
                        ImGui::AcceptDragDropPayload(HIERARCHY_DRAG_PAYLOAD))
                {
                    reparent_child = *static_cast<SceneNode* const*>(payload->Data);
                    reparent_to = node;
                    reparent_requested = true;
                }
                ImGui::EndDragDropTarget();
            }

            if (ImGui::BeginPopupContextItem())
            {
                context.selected_node = node->id;
                if (ImGui::MenuItem("Rename"))
                    context.renaming_node = node->id;
                if (ImGui::MenuItem("Add Child"))
                    add_child_target = node;
                if (node->parent != nullptr && ImGui::MenuItem("Unparent"))
                {
                    reparent_child = node;
                    reparent_to = nullptr;
                    reparent_requested = true;
                }
                if (ImGui::MenuItem("Delete"))
                    delete_target = node;
                ImGui::EndPopup();
            }

            if (open)
            {
                for (auto& child : node->children)
                    draw_hierarchy_node(context, child.get(), lower_filter,
                                        reparent_child, reparent_to, reparent_requested,
                                        delete_target, add_child_target);
                ImGui::TreePop();
            }

            ImGui::PopID();
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
    }

    void draw_menu_bar(EditorContext& context, bool& running)
    {
        if (!ImGui::BeginMainMenuBar())
            return;

        if (ImGui::BeginMenu("File"))
        {
            if (ImGui::MenuItem("New Entity", "Ctrl+N"))
            {
                SceneNode* node = context.scene.create_node("Entity");
                context.selected_node = node->id;
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

        if (ImGui::BeginMenu("GameObject"))
        {
            if (ImGui::MenuItem("Create Empty"))
            {
                SceneNode* node = context.scene.create_node("Entity");
                context.selected_node = node->id;
            }
            if (ImGui::MenuItem("Create Child", nullptr, false,
                                context.selected_node != 0))
            {
                SceneNode* parent = context.scene.find(context.selected_node);
                SceneNode* node = context.scene.create_node("Entity", parent);
                context.selected_node = node->id;
            }
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Window"))
        {
            ImGui::MenuItem("Scene", nullptr, &context.panels.scene_view);
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

        if (ImGui::Button("Add Entity"))
        {
            SceneNode* node = context.scene.create_node("Entity");
            context.selected_node = node->id;
            editor_log(context, "Created entity 'Entity'.");
        }
        ImGui::SameLine();
        ImGui::BeginDisabled(context.selected_node == 0);
        if (ImGui::Button("Delete"))
        {
            context.scene.remove_node(context.scene.find(context.selected_node));
            context.selected_node = 0;
        }
        ImGui::EndDisabled();

        ImGui::SetNextItemWidth(-FLT_MIN);
        ImGui::InputTextWithHint("##filter", "Search...", &context.hierarchy_filter);
        ImGui::Separator();

        const std::string lower_filter = to_lower(context.hierarchy_filter);

        SceneNode* reparent_child = nullptr;
        SceneNode* reparent_to = nullptr;
        bool reparent_requested = false;
        SceneNode* delete_target = nullptr;
        SceneNode* add_child_target = nullptr;

        for (auto& root : context.scene.roots())
            draw_hierarchy_node(context, root.get(), lower_filter, reparent_child,
                                reparent_to, reparent_requested, delete_target,
                                add_child_target);

        // Dropping onto the empty area below the tree promotes a node to a root.
        ImGui::Dummy(ImVec2(0.0f, ImGui::GetContentRegionAvail().y > 0.0f
                                       ? ImGui::GetContentRegionAvail().y
                                       : 1.0f));
        if (ImGui::BeginDragDropTarget())
        {
            if (const ImGuiPayload* payload =
                    ImGui::AcceptDragDropPayload(HIERARCHY_DRAG_PAYLOAD))
            {
                reparent_child = *static_cast<SceneNode* const*>(payload->Data);
                reparent_to = nullptr;
                reparent_requested = true;
            }
            ImGui::EndDragDropTarget();
        }

        if (add_child_target != nullptr)
        {
            SceneNode* node = context.scene.create_node("Entity", add_child_target);
            context.selected_node = node->id;
        }
        if (delete_target != nullptr)
        {
            if (context.selected_node == delete_target->id)
                context.selected_node = 0;
            context.scene.remove_node(delete_target);
        }
        if (reparent_requested)
            context.scene.reparent(reparent_child, reparent_to);

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

        SceneNode* node = context.scene.find(context.selected_node);
        if (node == nullptr)
        {
            ImGui::TextDisabled("Nothing selected.");
            ImGui::End();
            return;
        }

        ImGui::Checkbox("##visible", &node->visible);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(-FLT_MIN);
        ImGui::InputText("##name", &node->name);
        ImGui::Text("Id: %llu", static_cast<unsigned long long>(node->id));
        ImGui::Separator();

        if (ImGui::CollapsingHeader("Transform", ImGuiTreeNodeFlags_DefaultOpen))
        {
            if (ImGui::BeginTable("transform", 2,
                                  ImGuiTableFlags_SizingStretchProp))
            {
                ImGui::TableSetupColumn("label", ImGuiTableColumnFlags_WidthFixed,
                                        80.0f);
                ImGui::TableSetupColumn("value");
                vector3_field("Position", node->transform.position, 0.05f);
                vector3_field("Rotation", node->transform.rotation, 0.5f);
                vector3_field("Scale", node->transform.scale, 0.05f);
                ImGui::EndTable();
            }
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
        ImGui::Text("Entities:   %d", count_nodes(context.scene.roots()));
        ImGui::Text("Open files: %zu", context.documents.size());
        ImGui::Separator();
        ImGui::Text("World entities: %zu", context.world_entity_count);

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
                SceneNode* selected = context.scene.find(context.selected_node);
                ImGui::Text("Selected: %s",
                            selected != nullptr ? selected->name.c_str() : "none");
                ImGui::Separator();
                const char* state_text =
                    context.play_state == PlayState::Playing   ? "Playing"
                    : context.play_state == PlayState::Paused  ? "Paused"
                                                               : "Stopped";
                ImGui::Text("State: %s", state_text);
                ImGui::Separator();
                ImGui::Text("Entities: %d", count_nodes(context.scene.roots()));
                ImGui::EndMenuBar();
            }
        }
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
        ImGui::DockBuilderDockWindow("Scene", center);
        ImGui::DockBuilderDockWindow("Hierarchy", left);
        ImGui::DockBuilderDockWindow("Inspector", right);
        ImGui::DockBuilderDockWindow("Statistics", right);
        ImGui::DockBuilderDockWindow("Project", bottom);
        ImGui::DockBuilderDockWindow("Text Editor", bottom);
        ImGui::DockBuilderDockWindow("Console", bottom);
        ImGui::DockBuilderFinish(dockspace_id);
    }
}
