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

#include "../serialization/scene_serializer.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <system_error>
#include <unordered_map>

#include <filesystem>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <shellapi.h>
#endif

#include <imgui.h>
#include <imgui_internal.h>
#include <misc/cpp/imgui_stdlib.h>

namespace fs = std::filesystem;

namespace SushiEngine
{
    namespace Editor
    {
        using SushiEngine::Simulation::EntityId;
        using SushiEngine::Simulation::EntityTransform;
        using SushiEngine::Simulation::IWorldEditor;
        using SushiEngine::Simulation::NULL_ENTITY;

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

            // A child path named `base` under `parent`, disambiguated with " (n)" if it
            // already exists — matches Unity's "New Folder", "New Folder (1)", ... naming.
            fs::path unique_child_path(const fs::path& parent, const std::string& base,
                                       const std::string& extension)
            {
                fs::path candidate = parent / (base + extension);
                for (int n = 1; fs::exists(candidate); ++n)
                    candidate = parent / (base + " (" + std::to_string(n) + ")" + extension);
                return candidate;
            }

            // Truncates a display name to a tile-friendly length rather than wrapping —
            // simpler and robust across filenames of any length.
            std::string truncate_label(const std::string& name, std::size_t max_chars = 16)
            {
                if (name.size() <= max_chars)
                    return name;
                return name.substr(0, max_chars - 1) + "…";
            }

            // Opens the platform file browser at `path`, selecting it if it's a file.
            // Windows-only for now; the project targets Windows first (see CLAUDE.md).
            void show_in_explorer(const fs::path& path)
            {
    #ifdef _WIN32
                const std::string command = (fs::is_directory(path) ? "explorer \"" : "explorer /select,\"") +
                                            path.string() + "\"";
                std::system(command.c_str());
    #else
                (void)path;
    #endif
            }

            // Launches `path` with whatever the OS has associated with its extension —
            // Explorer's own double-click "open" verb (ShellExecute), not a shell command
            // line, so a path never round-trips through shell quoting/injection.
            void open_with_default_app(const fs::path& path)
            {
    #ifdef _WIN32
                ShellExecuteW(nullptr, L"open", path.wstring().c_str(), nullptr, nullptr, SW_SHOWNORMAL);
    #else
                (void)path;
    #endif
            }

            // A tile's icon fill colour by kind: folders, code, and everything else.
            ImU32 tile_color(const fs::path& path, bool is_dir)
            {
                if (is_dir)
                    return IM_COL32(210, 180, 90, 255);
                std::string ext = to_lower(path.extension().string());
                if (ext == ".cpp" || ext == ".cc" || ext == ".hpp" || ext == ".h" || ext == ".inl")
                    return IM_COL32(90, 150, 220, 255);
                if (has_text_extension(path))
                    return IM_COL32(150, 150, 150, 255);
                return IM_COL32(90, 90, 90, 255);
            }

            // Recursively draws one folder node of the Project panel's tree pane; clicking
            // a node (anywhere in its row) navigates the grid pane to that folder.
            void draw_project_tree_node(EditorContext& context, const fs::path& dir)
            {
                std::error_code ec;
                std::vector<fs::path> subdirs;
                for (const auto& entry : fs::directory_iterator(dir, ec))
                    if (entry.is_directory())
                        subdirs.push_back(entry.path());
                std::sort(subdirs.begin(), subdirs.end());

                const std::string label = dir.filename().empty() ? dir.string() : dir.filename().string();
                ImGuiTreeNodeFlags flags =
                    ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_SpanFullWidth;
                if (subdirs.empty())
                    flags |= ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen;
                std::error_code cmp_ec;
                if (fs::weakly_canonical(dir, cmp_ec) ==
                    fs::weakly_canonical(fs::path(context.current_directory), cmp_ec))
                    flags |= ImGuiTreeNodeFlags_Selected;

                ImGui::PushID(dir.string().c_str());
                const bool open = ImGui::TreeNodeEx(label.c_str(), flags);
                if (ImGui::IsItemClicked())
                    context.current_directory = dir.string();
                if (open && !(flags & ImGuiTreeNodeFlags_Leaf))
                {
                    for (const fs::path& sub : subdirs)
                        draw_project_tree_node(context, sub);
                    ImGui::TreePop();
                }
                ImGui::PopID();
            }

            // The "Create ▸" submenu shared by the grid's background and item context
            // menus: new folder or new source/text file, seeded into inline rename.
            void draw_project_create_menu(EditorContext& context, const fs::path& parent)
            {
                if (ImGui::BeginMenu("Create"))
                {
                    struct Entry { const char* label; const char* base; const char* ext; };
                    static const Entry entries[] = {
                        {"Folder", "New Folder", ""},
                        {"C++ Header", "NewHeader", ".hpp"},
                        {"C++ Source", "NewSource", ".cpp"},
                        {"Text File", "New Text File", ".txt"},
                    };
                    for (const Entry& entry : entries)
                    {
                        if (ImGui::MenuItem(entry.label))
                        {
                            const fs::path path = unique_child_path(parent, entry.base, entry.ext);
                            std::error_code ec;
                            if (entry.ext[0] == '\0')
                                fs::create_directory(path, ec);
                            else
                                std::ofstream(path, std::ios::binary).close();
                            context.renaming_project_path = path.string();
                            context.selected_project_path = path.string();
                        }
                    }
                    ImGui::EndMenu();
                }
            }

            // A labelled drag-float row for a 3-component vector, matching Unity's
            // X/Y/Z inspector rows. Returns true when any component changed this frame.
            bool vector3_field(EditorContext& context, IWorldEditor& world, const char* label,
                              float values[3], float speed)
            {
                ImGui::PushID(label);
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted(label);
                ImGui::TableSetColumnIndex(1);
                ImGui::SetNextItemWidth(-FLT_MIN);
                const bool changed = ImGui::DragFloat3("##v", values, speed);
                if (ImGui::IsItemActivated())
                    context.history.begin_change(world);
                if (ImGui::IsItemDeactivatedAfterEdit())
                    context.history.end_change();
                ImGui::PopID();
                return changed;
            }

            // Editor-facing rotation presentation: the world stores a quaternion, but the
            // inspector edits Euler degrees like Unity. These conversions are a display
            // concern local to the panel, not part of the engine's math seam.
            void quaternion_to_euler_degrees(const SushiEngine::Quaternion& q, float out[3])
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

            SushiEngine::Quaternion euler_degrees_to_quat(const float in[3])
            {
                constexpr float DEG_TO_RAD = 0.01745329f;
                const float roll = in[0] * DEG_TO_RAD;
                const float pitch = in[1] * DEG_TO_RAD;
                const float yaw = in[2] * DEG_TO_RAD;

                const float cr = std::cos(roll * 0.5f), sr = std::sin(roll * 0.5f);
                const float cp = std::cos(pitch * 0.5f), sp = std::sin(pitch * 0.5f);
                const float cy = std::cos(yaw * 0.5f), sy = std::sin(yaw * 0.5f);

                SushiEngine::Quaternion q;
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

            // Shared "Create Empty Entity" / "Camera" / "Objects > Box/Sphere/Cylinder/
            // Terrain" menu items, reused by the Entity menu, the hierarchy's row context
            // menu, its empty-space context menu, and the filtered search view, so all
            // entry points create identically — the Entity menu can never drift out of
            // sync with the Hierarchy's right-click menu. The new entity is selected the
            // same way every other creation path in this file already does.
            void draw_create_object_menu_items(EditorContext& context, IWorldEditor* world)
            {
                if (ImGui::MenuItem("Create Empty Entity", nullptr, false, world != nullptr))
                {
                    context.history.record(*world);
                    select_only(context, world->create("Entity"));
                    editor_log(context, "Created entity 'Entity'.");
                }
                if (ImGui::MenuItem("Camera", nullptr, false, world != nullptr))
                {
                    context.history.record(*world);
                    select_only(context, world->create_camera("Camera"));
                    editor_log(context, "Created camera 'Camera'.");
                }
                if (ImGui::BeginMenu("Objects", world != nullptr))
                {
                    if (ImGui::MenuItem("Box"))
                    {
                        context.history.record(*world);
                        select_only(context, world->create_box("Box"));
                        editor_log(context, "Created object 'Box'.");
                    }
                    if (ImGui::MenuItem("Sphere"))
                    {
                        context.history.record(*world);
                        select_only(context, world->create_sphere("Sphere"));
                        editor_log(context, "Created object 'Sphere'.");
                    }
                    if (ImGui::MenuItem("Cylinder"))
                    {
                        context.history.record(*world);
                        select_only(context, world->create_cylinder("Cylinder"));
                        editor_log(context, "Created object 'Cylinder'.");
                    }
                    if (ImGui::MenuItem("Terrain"))
                    {
                        context.history.record(*world);
                        select_only(context, world->create_terrain("Terrain"));
                        editor_log(context, "Created object 'Terrain'.");
                    }
                    if (ImGui::MenuItem("Cloth"))
                    {
                        context.history.record(*world);
                        select_only(context, world->create_cloth("Cloth"));
                        editor_log(context, "Created object 'Cloth'.");
                    }
                    ImGui::EndMenu();
                }
                if (ImGui::BeginMenu("UI", world != nullptr))
                {
                    // Image/Text/Button parent to the selected UI entity (typically a
                    // Canvas) so they lay out inside it; with no UI selected they anchor
                    // straight to the viewport, still visible and re-parentable later.
                    const EntityId ui_parent =
                        world != nullptr && world->has_ui(context.selected_entity)
                            ? context.selected_entity
                            : NULL_ENTITY;
                    if (ImGui::MenuItem("Canvas"))
                    {
                        context.history.record(*world);
                        select_only(context, world->create_canvas("Canvas"));
                        editor_log(context, "Created UI 'Canvas'.");
                    }
                    if (ImGui::MenuItem("Panel"))
                    {
                        context.history.record(*world);
                        select_only(context, world->create_ui_element(
                                                 "Panel", SushiEngine::Simulation::UIElementKind::Panel,
                                                 ui_parent));
                        editor_log(context, "Created UI 'Panel'.");
                    }
                    if (ImGui::MenuItem("Image"))
                    {
                        context.history.record(*world);
                        select_only(context, world->create_ui_element(
                                                 "Image", SushiEngine::Simulation::UIElementKind::Image,
                                                 ui_parent));
                        editor_log(context, "Created UI 'Image'.");
                    }
                    if (ImGui::MenuItem("Text"))
                    {
                        context.history.record(*world);
                        select_only(context, world->create_ui_element(
                                                 "Text", SushiEngine::Simulation::UIElementKind::Text,
                                                 ui_parent));
                        editor_log(context, "Created UI 'Text'.");
                    }
                    if (ImGui::MenuItem("Button"))
                    {
                        context.history.record(*world);
                        select_only(context, world->create_ui_element(
                                                 "Button", SushiEngine::Simulation::UIElementKind::Button,
                                                 ui_parent));
                        editor_log(context, "Created UI 'Button'.");
                    }
                    ImGui::EndMenu();
                }
            }

            // Shared Copy/Cut/Paste menu items, reused everywhere
            // `draw_create_object_menu_items` is (see its comment) so clipboard actions
            // never drift out of sync between the Edit menu and the Hierarchy either.
            void draw_clipboard_menu_items(EditorContext& context, IWorldEditor* world)
            {
                const bool has_selection = world != nullptr && !context.selected_entities.empty();
                if (ImGui::MenuItem("Copy", "Ctrl+C", false, has_selection))
                    copy_selection(context);
                if (ImGui::MenuItem("Cut", "Ctrl+X", false, has_selection))
                    cut_selection(context);
                if (ImGui::MenuItem("Paste", "Ctrl+V", false,
                                    world != nullptr && !context.clipboard.empty()))
                    paste_clipboard(context);
            }
        }

        void copy_selection(EditorContext& context)
        {
            IWorldEditor* world = world_of(context);
            if (world == nullptr || context.selected_entities.empty())
                return;

            context.clipboard.clear();
            context.clipboard.reserve(context.selected_entities.size());
            for (const EntityId id : context.selected_entities)
            {
                ClipboardEntity entry;
                entry.original = id;
                entry.original_parent = world->parent(id);
                entry.name = world->name(id);
                entry.transform = world->transform(id);
                entry.color = world->color(id);
                entry.visible = world->visible(id);
                entry.has_renderer = world->has_renderer(id);
                entry.is_camera = world->is_camera(id);
                entry.camera_params = world->camera_params(id);
                entry.has_physics_body = world->has_physics_body(id);
                entry.physics_body_params = world->physics_body_params(id);
                entry.has_cloth = world->has_cloth(id);
                entry.cloth_params = world->cloth_params(id);
                entry.has_shape = world->has_shape(id);
                entry.shape_params = world->shape_params(id);
                entry.has_collider = world->has_collider(id);
                entry.collider_params = world->collider_params(id);
                entry.has_ui = world->has_ui(id);
                entry.ui_params = world->ui_params(id);
                for (const std::string& type_name : world->script_components(id))
                    entry.scripts.push_back(world->script_component(id, type_name));
                context.clipboard.push_back(entry);
            }
            editor_log(context, "Copied " + std::to_string(context.clipboard.size()) + " entit" +
                                     (context.clipboard.size() == 1 ? "y" : "ies") + ".");
        }

        void cut_selection(EditorContext& context)
        {
            IWorldEditor* world = world_of(context);
            if (world == nullptr || context.selected_entities.empty())
                return;

            copy_selection(context);
            context.history.record(*world);
            for (const EntityId id : context.selected_entities)
                world->destroy(id);
            select_only(context, NULL_ENTITY);
            editor_log(context, "Cut " + std::to_string(context.clipboard.size()) + " entit" +
                                     (context.clipboard.size() == 1 ? "y" : "ies") + ".");
        }

        void paste_clipboard(EditorContext& context)
        {
            IWorldEditor* world = world_of(context);
            if (world == nullptr || context.clipboard.empty())
                return;

            context.history.record(*world);

            std::unordered_map<EntityId, EntityId> original_to_new;
            std::vector<EntityId> pasted;
            pasted.reserve(context.clipboard.size());

            for (const ClipboardEntity& entry : context.clipboard)
            {
                const EntityId id = world->create(entry.name);
                world->set_transform(id, entry.transform);
                world->set_color(id, entry.color);
                world->set_visible(id, entry.visible);
                world->set_has_renderer(id, entry.has_renderer);
                world->set_is_camera(id, entry.is_camera);
                if (entry.is_camera)
                    world->set_camera_params(id, entry.camera_params);
                world->set_has_physics_body(id, entry.has_physics_body);
                if (entry.has_physics_body)
                    world->set_physics_body_params(id, entry.physics_body_params);
                world->set_has_cloth(id, entry.has_cloth);
                if (entry.has_cloth)
                    world->set_cloth_params(id, entry.cloth_params);
                world->set_has_shape(id, entry.has_shape);
                if (entry.has_shape)
                    world->set_shape_params(id, entry.shape_params);
                world->set_has_collider(id, entry.has_collider);
                if (entry.has_collider)
                    world->set_collider_params(id, entry.collider_params);
                world->set_has_ui(id, entry.has_ui);
                if (entry.has_ui)
                    world->set_ui_params(id, entry.ui_params);
                for (const SushiEngine::Simulation::ScriptComponent& script : entry.scripts)
                    world->add_script_component(id, script);

                original_to_new[entry.original] = id;
                pasted.push_back(id);
            }

            // Second pass: internal parent/child links between copied entities take
            // priority; anything else falls back to the original's external parent
            // (still alive) so a paste-in-place keeps its old spot in the hierarchy.
            for (std::size_t i = 0; i < context.clipboard.size(); ++i)
            {
                const ClipboardEntity& entry = context.clipboard[i];
                const EntityId new_id = pasted[i];
                const auto mapped = original_to_new.find(entry.original_parent);
                if (mapped != original_to_new.end())
                    world->set_parent(new_id, mapped->second);
                else if (world->exists(entry.original_parent))
                    world->set_parent(new_id, entry.original_parent);
            }

            context.selected_entity = pasted.empty() ? NULL_ENTITY : pasted.back();
            context.selection_anchor = context.selected_entity;
            context.selected_entities = pasted;
            editor_log(context, "Pasted " + std::to_string(pasted.size()) + " entit" +
                                     (pasted.size() == 1 ? "y" : "ies") + ".");
        }

        bool save_current_scene(EditorContext& context)
        {
            IWorldEditor* world = world_of(context);
            if (world == nullptr)
                return false;
            if (context.scene_path.empty())
            {
                context.save_scene_as_name = "Scene.sushiscene";
                context.show_save_scene_as = true;
                return false;
            }
            if (save_scene(*world, context.scene_path))
            {
                context.saved_scene_revision = context.history.revision();
                editor_log(context, "Saved scene '" + context.scene_path + "'.");
                return true;
            }
            editor_log(context, "Failed to save scene '" + context.scene_path + "'.");
            return false;
        }

        namespace
        {
            // Wipes the live world back to empty. Shared by the immediate path (scene
            // already clean) and `perform_pending_scene_action` (scene was dirty and the
            // unsaved-changes prompt just resolved it).
            void new_scene(EditorContext& context)
            {
                IWorldEditor* world = world_of(context);
                if (world == nullptr)
                    return;
                context.history.record(*world);
                for (const EntityId id : world->entities())
                    world->destroy(id);
                context.scene_path.clear();
                context.saved_scene_revision = context.history.revision();
                select_only(context, NULL_ENTITY);
                editor_log(context, "New scene.");
            }

            // Loads @p path over the live world. Shared by the immediate path and
            // `perform_pending_scene_action`.
            void open_scene(EditorContext& context, const std::string& path)
            {
                IWorldEditor* world = world_of(context);
                if (world == nullptr)
                    return;
                if (load_scene(*world, path))
                {
                    context.scene_path = path;
                    context.saved_scene_revision = context.history.revision();
                    select_only(context, NULL_ENTITY);
                    editor_log(context, "Loaded scene '" + path + "'.");
                }
                else
                {
                    editor_log(context, "Failed to load scene '" + path + "'.");
                }
            }

            // Runs whichever scene replacement was parked in `pending_scene_action`, then
            // clears it. Called once the unsaved-changes prompt (or its absence, when the
            // scene was already clean) has cleared the way.
            void perform_pending_scene_action(EditorContext& context)
            {
                switch (context.pending_scene_action)
                {
                    case EditorContext::PendingSceneAction::New:
                        new_scene(context);
                        break;
                    case EditorContext::PendingSceneAction::Open:
                        open_scene(context, context.pending_scene_open_path);
                        break;
                    case EditorContext::PendingSceneAction::None:
                        break;
                }
                context.pending_scene_action = EditorContext::PendingSceneAction::None;
                context.pending_scene_open_path.clear();
            }
        }

        // Requests a scene replacement (New or Open), deferring to the unsaved-changes
        // prompt when the current scene is dirty rather than discarding it silently.
        void request_new_scene(EditorContext& context)
        {
            if (scene_is_dirty(context))
                context.pending_scene_action = EditorContext::PendingSceneAction::New;
            else
                new_scene(context);
        }

        void request_open_scene(EditorContext& context, const std::string& path)
        {
            if (scene_is_dirty(context))
            {
                context.pending_scene_action = EditorContext::PendingSceneAction::Open;
                context.pending_scene_open_path = path;
            }
            else
            {
                open_scene(context, path);
            }
        }

        void draw_menu_bar(EditorContext& context)
        {
            if (!ImGui::BeginMainMenuBar())
                return;

            IWorldEditor* world = world_of(context);

            if (ImGui::BeginMenu("File"))
            {
                if (ImGui::MenuItem("New Entity", "Ctrl+N", false, world != nullptr))
                {
                    select_only(context, world->create("Entity"));
                    editor_log(context, "Created entity 'Entity'.");
                }
                ImGui::Separator();
                if (ImGui::MenuItem("New Scene", nullptr, false, world != nullptr))
                    request_new_scene(context);
                if (ImGui::MenuItem("Save Scene", "Ctrl+S", false, world != nullptr))
                    (void)save_current_scene(context);
                if (ImGui::MenuItem("Save Scene As...", nullptr, false, world != nullptr))
                {
                    context.save_scene_as_name =
                        context.scene_path.empty() ? "Scene.sushiscene" : fs::path(context.scene_path).filename().string();
                    context.show_save_scene_as = true;
                }
                ImGui::Separator();
                if (ImGui::MenuItem("Save", nullptr, false,
                                    context.active_document >= 0))
                {
                    Document& document = context.documents[static_cast<std::size_t>(
                        context.active_document)];
                    save_document(document);
                    editor_log(context, "Saved '" + document.display_name + "'.");
                }
                ImGui::Separator();
                if (ImGui::MenuItem("Exit", "Alt+F4"))
                    context.close_requested = true;
                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("Edit"))
            {
                // Undo/redo replaces the world wholesale (see CommandHistory), so entity
                // ids from before the swap are no longer valid; drop the selection rather
                // than risk it aliasing an unrelated new entity.
                if (ImGui::MenuItem("Undo", "Ctrl+Z", false, context.history.can_undo()) &&
                    world != nullptr && context.history.undo(*world))
                    select_only(context, NULL_ENTITY);
                if (ImGui::MenuItem("Redo", "Ctrl+Y", false, context.history.can_redo()) &&
                    world != nullptr && context.history.redo(*world))
                    select_only(context, NULL_ENTITY);
                ImGui::Separator();
                draw_clipboard_menu_items(context, world);
                ImGui::Separator();
                if (ImGui::MenuItem("Preferences...", nullptr))
                    context.show_preferences = true;
                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("Entity"))
            {
                draw_create_object_menu_items(context, world);
                ImGui::Separator();
                const bool has_selection =
                    world != nullptr && world->exists(context.selected_entity);
                if (ImGui::MenuItem("Align With View", nullptr, false, has_selection))
                    context.align_with_view_requested = true;
                if (ImGui::MenuItem("Move to View", nullptr, false, has_selection))
                    context.move_to_view_requested = true;
                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("Window"))
            {
                ImGui::MenuItem("Scene", nullptr, &context.panels.scene_view);
                ImGui::MenuItem("Game", nullptr, &context.panels.game_view);
                ImGui::MenuItem("Hierarchy", nullptr, &context.panels.hierarchy);
                ImGui::MenuItem("Inspector", nullptr, &context.panels.inspector);
                ImGui::MenuItem("Environment", nullptr, &context.panels.environment);
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

        namespace
        {
            // Depth-first pre-order (roots in world order, each followed by its children)
            // — the same order draw_entity_node recurses in, so a Shift-range over it
            // matches what is visually listed whenever every node is expanded.
            void collect_display_order(IWorldEditor* world, EntityId parent,
                                       std::vector<EntityId>& out)
            {
                for (const EntityId id : world->entities())
                    if (world->parent(id) == parent)
                    {
                        out.push_back(id);
                        collect_display_order(world, id, out);
                    }
            }

            // Shift+click: select every entity between the anchor (the last plain or
            // Ctrl click) and @p id in @p order, inclusive. The anchor itself does not
            // move, so repeated Shift-clicks re-extend the same range rather than
            // chaining from the previous Shift target.
            void select_range(EditorContext& context, const std::vector<EntityId>& order,
                              EntityId id)
            {
                const auto anchor_it =
                    std::find(order.begin(), order.end(), context.selection_anchor);
                const auto to_it = std::find(order.begin(), order.end(), id);
                if (anchor_it == order.end() || to_it == order.end())
                {
                    select_only(context, id);
                    return;
                }
                auto a = static_cast<std::size_t>(std::distance(order.begin(), anchor_it));
                auto b = static_cast<std::size_t>(std::distance(order.begin(), to_it));
                if (a > b)
                    std::swap(a, b);
                context.selected_entities.clear();
                for (std::size_t i = a; i <= b; ++i)
                    context.selected_entities.push_back(order[i]);
                context.selected_entity = id;
            }

            // One Hierarchy row: rename field or selectable label, drag-reparent source
            // and target, context menu, and (when not renaming) a recursive draw of its
            // children so parenting nests visually the way Unity's hierarchy does.
            // @p order is the full display-order flattening, used to resolve Shift-range
            // selection; @p delete_requested is set when this row's Delete is chosen.
            void draw_entity_node(EditorContext& context, IWorldEditor* world, EntityId id,
                                   const std::vector<EntityId>& order, bool& delete_requested)
            {
                const std::string entity_name = world->name(id);
                ImGui::PushID(static_cast<int>(id));

                if (context.renaming_entity == id)
                {
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
                        ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_AutoSelectAll);
                    if (commit || ImGui::IsItemDeactivated())
                    {
                        context.history.record(*world);
                        world->set_name(id, buffer);
                        context.renaming_entity = NULL_ENTITY;
                        active = NULL_ENTITY;
                    }
                    ImGui::PopID();
                    return;
                }

                std::vector<EntityId> children;
                for (const EntityId candidate : world->entities())
                    if (world->parent(candidate) == id)
                        children.push_back(candidate);

                ImGuiTreeNodeFlags flags =
                    ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_SpanAvailWidth;
                if (children.empty())
                    flags |= ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen;
                if (is_selected(context, id))
                    flags |= ImGuiTreeNodeFlags_Selected;

                const bool open = ImGui::TreeNodeEx(entity_name.c_str(), flags);
                if (ImGui::IsItemClicked(ImGuiMouseButton_Left) && !ImGui::IsItemToggledOpen())
                {
                    const ImGuiIO& io = ImGui::GetIO();
                    if (io.KeyShift)
                        select_range(context, order, id);
                    else if (io.KeyCtrl)
                        toggle_selected(context, id);
                    else
                        select_only(context, id);
                    if (!io.KeyShift && !io.KeyCtrl &&
                        ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
                        context.frame_selected_requested = true;
                }

                if (ImGui::BeginDragDropSource())
                {
                    ImGui::SetDragDropPayload("HIERARCHY_ENTITY", &id, sizeof(EntityId));
                    ImGui::TextUnformatted(entity_name.c_str());
                    ImGui::EndDragDropSource();
                }
                if (ImGui::BeginDragDropTarget())
                {
                    if (const ImGuiPayload* payload =
                            ImGui::AcceptDragDropPayload("HIERARCHY_ENTITY", ImGuiDragDropFlags_AcceptBeforeDelivery | ImGuiDragDropFlags_AcceptNoDrawDefaultRect))
                    {
                        const EntityId dragged = *static_cast<const EntityId*>(payload->Data);
                        
                        ImVec2 mouse_pos = ImGui::GetMousePos();
                        ImVec2 item_pos = ImGui::GetItemRectMin();
                        ImVec2 item_size = ImGui::GetItemRectSize();
                        
                        bool drop_before = mouse_pos.y < item_pos.y + item_size.y * 0.25f;
                        bool drop_after = mouse_pos.y > item_pos.y + item_size.y * 0.75f;
                        
                        if (payload->IsDelivery())
                        {
                            context.history.record(*world);
                            if (drop_before)
                            {
                                world->set_parent(dragged, world->parent(id));
                                world->move_entity(dragged, id, false);
                            }
                            else if (drop_after)
                            {
                                world->set_parent(dragged, world->parent(id));
                                world->move_entity(dragged, id, true);
                            }
                            else
                            {
                                world->set_parent(dragged, id);
                            }
                        }
                        else
                        {
                            if (drop_before)
                            {
                                ImGui::GetWindowDrawList()->AddLine(
                                    item_pos, ImVec2(item_pos.x + item_size.x, item_pos.y),
                                    IM_COL32(255, 255, 0, 255), 2.0f);
                            }
                            else if (drop_after)
                            {
                                ImGui::GetWindowDrawList()->AddLine(
                                    ImVec2(item_pos.x, item_pos.y + item_size.y),
                                    ImVec2(item_pos.x + item_size.x, item_pos.y + item_size.y),
                                    IM_COL32(255, 255, 0, 255), 2.0f);
                            }
                            else
                            {
                                ImGui::GetWindowDrawList()->AddRect(
                                    item_pos, ImVec2(item_pos.x + item_size.x, item_pos.y + item_size.y),
                                    IM_COL32(255, 255, 0, 255), 0.0f, 0, 2.0f);
                            }
                        }
                    }
                    ImGui::EndDragDropTarget();
                }

                if (ImGui::BeginPopupContextItem())
                {
                    // Right-clicking an entity outside the current selection replaces it;
                    // right-clicking one already selected preserves the multi-selection so
                    // Delete can act on the whole group.
                    if (!is_selected(context, id))
                        select_only(context, id);
                    if (ImGui::MenuItem("Rename"))
                        context.renaming_entity = id;
                    if (ImGui::MenuItem("Unparent", nullptr, false,
                                        world->parent(id) != NULL_ENTITY))
                    {
                        context.history.record(*world);
                        world->set_parent(id, NULL_ENTITY);
                    }
                    if (ImGui::MenuItem("Delete"))
                        delete_requested = true;
                    ImGui::Separator();
                    draw_clipboard_menu_items(context, world);
                    ImGui::Separator();
                    draw_create_object_menu_items(context, world);
                    ImGui::EndPopup();
                }

                if (open && !children.empty())
                {
                    for (const EntityId child : children)
                        draw_entity_node(context, world, child, order, delete_requested);
                    ImGui::TreePop();
                }

                ImGui::PopID();
            }
        } // namespace

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
                context.history.record(*world);
                select_only(context, world->create("Entity"));
                editor_log(context, "Created entity 'Entity'.");
            }
            ImGui::SameLine();
            ImGui::BeginDisabled(context.selected_entities.empty());
            if (ImGui::Button("Delete"))
            {
                context.history.record(*world);
                for (const EntityId target : context.selected_entities)
                    world->destroy(target);
                select_only(context, NULL_ENTITY);
            }
            ImGui::EndDisabled();

            ImGui::SetNextItemWidth(-FLT_MIN);
            ImGui::InputTextWithHint("##filter", "Search...", &context.hierarchy_filter);
            ImGui::Separator();

            const std::string lower_filter = to_lower(context.hierarchy_filter);

            // Delete is deferred out of the loop below so the entity list is never
            // mutated while it is being walked; a row sets this when its own Delete
            // (context menu or, implicitly, being part of the selection) fires.
            bool delete_requested = false;

            if (ImGui::BeginChild("entities"))
            {
                if (!lower_filter.empty())
                {
                    // A search filter flattens the tree: nesting is meaningless once most
                    // of the hierarchy is hidden, so matches are listed directly. Ctrl/Shift
                    // still work, ranging over the filtered order shown here.
                    std::vector<EntityId> filtered_order;
                    for (const EntityId id : world->entities())
                        if (to_lower(world->name(id)).find(lower_filter) != std::string::npos)
                            filtered_order.push_back(id);

                    for (const EntityId id : filtered_order)
                    {
                        const std::string entity_name = world->name(id);
                        ImGui::PushID(static_cast<int>(id));

                        if (context.renaming_entity == id)
                        {
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
                                context.history.record(*world);
                                world->set_name(id, buffer);
                                context.renaming_entity = NULL_ENTITY;
                                active = NULL_ENTITY;
                            }
                        }
                        else
                        {
                            const bool selected = is_selected(context, id);
                            if (ImGui::Selectable(entity_name.c_str(), selected,
                                                  ImGuiSelectableFlags_AllowDoubleClick))
                            {
                                const ImGuiIO& io = ImGui::GetIO();
                                if (io.KeyShift)
                                    select_range(context, filtered_order, id);
                                else if (io.KeyCtrl)
                                    toggle_selected(context, id);
                                else
                                    select_only(context, id);
                                if (!io.KeyShift && !io.KeyCtrl &&
                                    ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
                                    context.frame_selected_requested = true;
                            }
                            if (ImGui::BeginPopupContextItem())
                            {
                                if (!is_selected(context, id))
                                    select_only(context, id);
                                if (ImGui::MenuItem("Rename"))
                                    context.renaming_entity = id;
                                if (ImGui::MenuItem("Delete"))
                                    delete_requested = true;
                                ImGui::Separator();
                                draw_clipboard_menu_items(context, world);
                                ImGui::Separator();
                                draw_create_object_menu_items(context, world);
                                ImGui::EndPopup();
                            }
                        }
                        ImGui::PopID();
                    }
                }
                else
                {
                    std::vector<EntityId> order;
                    collect_display_order(world, NULL_ENTITY, order);

                    // The root canvas itself accepts drops too, so dragging an entity onto
                    // empty space unparents it back to the top level.
                    if (ImGui::BeginDragDropTargetCustom(ImGui::GetCurrentWindow()->InnerRect,
                                                         ImGui::GetID("hierarchy_root")))
                    {
                        if (const ImGuiPayload* payload =
                                ImGui::AcceptDragDropPayload("HIERARCHY_ENTITY"))
                        {
                            const EntityId dragged = *static_cast<const EntityId*>(payload->Data);
                            context.history.record(*world);
                            world->set_parent(dragged, NULL_ENTITY);
                        }
                        ImGui::EndDragDropTarget();
                    }

                    for (const EntityId id : world->entities())
                        if (world->parent(id) == NULL_ENTITY)
                            draw_entity_node(context, world, id, order, delete_requested);

                    // Right-clicking blank space below the tree, not any row, so this
                    // never fires on top of a row's own BeginPopupContextItem above.
                    if (ImGui::BeginPopupContextWindow("hierarchy_empty_space",
                                                        ImGuiPopupFlags_MouseButtonRight |
                                                            ImGuiPopupFlags_NoOpenOverItems))
                    {
                        draw_clipboard_menu_items(context, world);
                        ImGui::Separator();
                        draw_create_object_menu_items(context, world);
                        ImGui::EndPopup();
                    }
                }

                if (ImGui::IsWindowHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Left) && !ImGui::IsAnyItemHovered())
                {
                    select_only(context, NULL_ENTITY);
                }
            }
            ImGui::EndChild();

            if (delete_requested && !context.selected_entities.empty())
            {
                context.history.record(*world);
                for (const EntityId target : context.selected_entities)
                    world->destroy(target);
                select_only(context, NULL_ENTITY);
            }

            ImGui::End();
        }

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

            /** @brief Adds @p script's type to the Add Component catalog if it is not already there. */
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
                    if (ImGui::IsItemActivated())
                        context.history.begin_change(world);
                    if (ImGui::IsItemDeactivatedAfterEdit())
                        context.history.end_change();
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
                        editor_log(context, "Failed to write script '" + path.string() + "'.");
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
        } // namespace

        void draw_inspector_panel(EditorContext& context)
        {
            draw_new_script_modal(context);
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
            {
                context.history.record(*world);
                world->set_visible(id, visible);
            }
            ImGui::SameLine();
            std::string name = world->name(id);
            ImGui::SetNextItemWidth(-FLT_MIN);
            if (ImGui::InputText("##name", &name))
                world->set_name(id, name);
            if (ImGui::IsItemActivated())
                context.history.begin_change(*world);
            if (ImGui::IsItemDeactivatedAfterEdit())
                context.history.end_change();
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
                quaternion_to_euler_degrees(transform.rotation, rotation);
                float scale[3] = {static_cast<float>(transform.scale.x),
                                  static_cast<float>(transform.scale.y),
                                  static_cast<float>(transform.scale.z)};

                bool changed = false;
                if (ImGui::BeginTable("transform", 2, ImGuiTableFlags_SizingStretchProp))
                {
                    ImGui::TableSetupColumn("label", ImGuiTableColumnFlags_WidthFixed, 80.0f);
                    ImGui::TableSetupColumn("value");
                    changed |= vector3_field(context, *world, "Position", position, 0.05f);
                    changed |= vector3_field(context, *world, "Rotation", rotation, 0.5f);
                    changed |= vector3_field(context, *world, "Scale", scale, 0.05f);
                    ImGui::EndTable();
                }

                if (changed)
                {
                    transform.position = SushiEngine::Vector3{position[0], position[1], position[2]};
                    transform.rotation = euler_degrees_to_quat(rotation);
                    transform.scale = SushiEngine::Vector3{scale[0], scale[1], scale[2]};
                    world->set_transform(id, transform);
                }
            }

            // Unity-style modular components: Transform above is mandatory on every
            // entity; Renderer, Camera, and Rigid Body are independently attached/
            // detached below, each with its own "x" to remove it, plus an Add Component
            // menu for whichever are currently missing.
            if (world->is_camera(id))
            {
                bool keep_camera = true;
                const bool camera_open = ImGui::CollapsingHeader(
                    "Camera", &keep_camera, ImGuiTreeNodeFlags_DefaultOpen);
                if (!keep_camera)
                {
                    context.history.record(*world);
                    world->set_is_camera(id, false);
                }
                else if (camera_open)
                {
                    SushiEngine::Simulation::CameraParams params = world->camera_params(id);
                    bool changed = false;

                    float fov_degrees =
                        static_cast<float>(params.vertical_fov_radians) * 57.29578f;
                    if (ImGui::DragFloat("FOV (deg)", &fov_degrees, 0.5f, 10.0f, 170.0f, "%.1f"))
                    {
                        params.vertical_fov_radians =
                            static_cast<SushiEngine::Scalar>(fov_degrees / 57.29578f);
                        changed = true;
                    }
                    if (ImGui::IsItemActivated())
                        context.history.begin_change(*world);
                    if (ImGui::IsItemDeactivatedAfterEdit())
                        context.history.end_change();

                    float near_plane = static_cast<float>(params.near_plane);
                    float far_plane = static_cast<float>(params.far_plane);
                    if (ImGui::DragFloat("Near", &near_plane, 0.01f, 0.001f, 10.0f, "%.3f"))
                    {
                        params.near_plane = static_cast<SushiEngine::Scalar>(near_plane);
                        changed = true;
                    }
                    if (ImGui::IsItemActivated())
                        context.history.begin_change(*world);
                    if (ImGui::IsItemDeactivatedAfterEdit())
                        context.history.end_change();

                    if (ImGui::DragFloat("Far", &far_plane, 1.0f, 1.0f, 10000.0f, "%.1f"))
                    {
                        params.far_plane = static_cast<SushiEngine::Scalar>(far_plane);
                        changed = true;
                    }
                    if (ImGui::IsItemActivated())
                        context.history.begin_change(*world);
                    if (ImGui::IsItemDeactivatedAfterEdit())
                        context.history.end_change();

                    int display_index = static_cast<int>(params.display_index);
                    if (ImGui::DragInt("Display", &display_index, 0.1f, 0, 15))
                    {
                        params.display_index = static_cast<std::uint32_t>(display_index < 0 ? 0 : display_index);
                        changed = true;
                    }
                    if (ImGui::IsItemActivated())
                        context.history.begin_change(*world);
                    if (ImGui::IsItemDeactivatedAfterEdit())
                        context.history.end_change();

                    int priority = static_cast<int>(params.priority);
                    if (ImGui::DragInt("Priority", &priority, 0.1f))
                    {
                        params.priority = priority;
                        changed = true;
                    }
                    if (ImGui::IsItemActivated())
                        context.history.begin_change(*world);
                    if (ImGui::IsItemDeactivatedAfterEdit())
                        context.history.end_change();

                    if (ImGui::Checkbox("Active", &params.active))
                    {
                        context.history.record(*world);
                        changed = true;
                    }

                    if (changed)
                        world->set_camera_params(id, params);
                }
            }

            if (world->has_renderer(id))
            {
                bool keep_renderer = true;
                const bool renderer_open = ImGui::CollapsingHeader(
                    "Renderer", &keep_renderer, ImGuiTreeNodeFlags_DefaultOpen);
                if (!keep_renderer)
                {
                    // The mesh (Shape) is a feature of the Renderer, so removing the
                    // Renderer takes its mesh with it — a Renderer never lingers as an
                    // invisible component and a mesh never survives without one to draw it.
                    context.history.record(*world);
                    world->set_has_shape(id, false);
                    world->set_has_renderer(id, false);
                }
                else if (renderer_open)
                {
                    const SushiEngine::Vector3 current = world->color(id);
                    float color[3] = {static_cast<float>(current.x),
                                      static_cast<float>(current.y),
                                      static_cast<float>(current.z)};
                    if (ImGui::ColorEdit3("Color", color))
                        world->set_color(id, SushiEngine::Vector3{color[0], color[1], color[2]});
                    if (ImGui::IsItemActivated())
                        context.history.begin_change(*world);
                    if (ImGui::IsItemDeactivatedAfterEdit())
                        context.history.end_change();

                    // PBR material: metallic / roughness / emissive. Albedo is the Color
                    // above, so only the surface-response fields are authored here.
                    SushiEngine::Render::Material material = world->material(id);
                    bool material_changed = false;
                    if (ImGui::SliderFloat("Metallic", &material.metallic, 0.0f, 1.0f))
                        material_changed = true;
                    if (ImGui::IsItemActivated())
                        context.history.begin_change(*world);
                    if (ImGui::IsItemDeactivatedAfterEdit())
                        context.history.end_change();
                    if (ImGui::SliderFloat("Roughness", &material.roughness, 0.045f, 1.0f))
                        material_changed = true;
                    if (ImGui::IsItemActivated())
                        context.history.begin_change(*world);
                    if (ImGui::IsItemDeactivatedAfterEdit())
                        context.history.end_change();
                    float emissive[3] = {static_cast<float>(material.emissive.x),
                                         static_cast<float>(material.emissive.y),
                                         static_cast<float>(material.emissive.z)};
                    if (ImGui::ColorEdit3("Emissive", emissive,
                                          ImGuiColorEditFlags_HDR | ImGuiColorEditFlags_Float))
                    {
                        material.emissive =
                            SushiEngine::Vector3{emissive[0], emissive[1], emissive[2]};
                        material_changed = true;
                    }
                    if (ImGui::IsItemActivated())
                        context.history.begin_change(*world);
                    if (ImGui::IsItemDeactivatedAfterEdit())
                        context.history.end_change();
                    if (material_changed)
                        world->set_material(id, material);

                    // The mesh is the Renderer's own property (Unity's MeshFilter folded
                    // into the MeshRenderer here): its primitive kind and per-kind
                    // dimensions are edited inline, and a Renderer with no mesh yet can be
                    // given one so it starts drawing.
                    if (world->has_shape(id))
                    {
                        SushiEngine::Simulation::ShapeParams params = world->shape_params(id);
                        bool changed = false;

                        // Plane is not a drawable mesh (Terrain uses a thin Box), so only
                        // the three solid primitives are offered as the Renderer's mesh.
                        static const char* const MESH_NAMES[] = {"Box", "Sphere", "Cylinder"};
                        int kind_index = static_cast<int>(params.kind);
                        if (kind_index > 2)
                            kind_index = 0;
                        if (ImGui::Combo("Mesh", &kind_index, MESH_NAMES, 3))
                        {
                            context.history.record(*world);
                            params.kind =
                                static_cast<SushiEngine::Simulation::PrimitiveKind>(kind_index);
                            changed = true;
                        }

                        switch (params.kind)
                        {
                            case SushiEngine::Simulation::PrimitiveKind::Sphere:
                            {
                                float radius = static_cast<float>(params.params.x);
                                if (ImGui::DragFloat("Radius##Mesh", &radius, 0.01f, 0.01f,
                                                     1000.0f, "%.3f"))
                                {
                                    params.params.x = radius;
                                    changed = true;
                                }
                                break;
                            }
                            case SushiEngine::Simulation::PrimitiveKind::Cylinder:
                            {
                                float radius = static_cast<float>(params.params.x);
                                float half_height = static_cast<float>(params.params.y);
                                if (ImGui::DragFloat("Radius##Mesh", &radius, 0.01f, 0.01f,
                                                     1000.0f, "%.3f"))
                                {
                                    params.params.x = radius;
                                    changed = true;
                                }
                                if (ImGui::DragFloat("Half Height##Mesh", &half_height, 0.01f,
                                                     0.01f, 1000.0f, "%.3f"))
                                {
                                    params.params.y = half_height;
                                    changed = true;
                                }
                                break;
                            }
                            default:
                            {
                                float half_extents[3] = {static_cast<float>(params.params.x),
                                                         static_cast<float>(params.params.y),
                                                         static_cast<float>(params.params.z)};
                                if (ImGui::DragFloat3("Half Extents##Mesh", half_extents, 0.01f,
                                                      0.01f, 1000.0f, "%.3f"))
                                {
                                    params.params = SushiEngine::Vector3{
                                        half_extents[0], half_extents[1], half_extents[2]};
                                    changed = true;
                                }
                                break;
                            }
                        }
                        if (ImGui::IsItemActivated())
                            context.history.begin_change(*world);
                        if (ImGui::IsItemDeactivatedAfterEdit())
                            context.history.end_change();

                        if (changed)
                            world->set_shape_params(id, params);
                    }
                    else if (ImGui::SmallButton("Add Mesh"))
                    {
                        context.history.record(*world);
                        world->set_has_shape(id, true);
                    }
                }
            }

            if (world->has_physics_body(id))
            {
                bool keep_physics = true;
                const bool physics_open = ImGui::CollapsingHeader(
                    "Rigid Body", &keep_physics, ImGuiTreeNodeFlags_DefaultOpen);
                if (!keep_physics)
                {
                    context.history.record(*world);
                    world->set_has_physics_body(id, false);
                }
                else if (physics_open)
                {
                    SushiEngine::Simulation::PhysicsBodyParams params = world->physics_body_params(id);
                    bool changed = false;

                    float inv_mass = static_cast<float>(params.inv_mass);
                    if (ImGui::DragFloat("Inverse Mass", &inv_mass, 0.01f, 0.0f, 100.0f, "%.3f"))
                    {
                        params.inv_mass = static_cast<SushiEngine::Scalar>(inv_mass < 0.0f ? 0.0f : inv_mass);
                        changed = true;
                    }
                    if (ImGui::IsItemActivated())
                        context.history.begin_change(*world);
                    if (ImGui::IsItemDeactivatedAfterEdit())
                        context.history.end_change();

                    float inv_inertia[3] = {static_cast<float>(params.inv_inertia.x),
                                            static_cast<float>(params.inv_inertia.y),
                                            static_cast<float>(params.inv_inertia.z)};
                    if (ImGui::DragFloat3("Inverse Inertia", inv_inertia, 0.01f, 0.0f, 100.0f, "%.3f"))
                    {
                        params.inv_inertia = SushiEngine::Vector3{
                            inv_inertia[0] < 0.0f ? 0.0f : inv_inertia[0],
                            inv_inertia[1] < 0.0f ? 0.0f : inv_inertia[1],
                            inv_inertia[2] < 0.0f ? 0.0f : inv_inertia[2]};
                        changed = true;
                    }
                    if (ImGui::IsItemActivated())
                        context.history.begin_change(*world);
                    if (ImGui::IsItemDeactivatedAfterEdit())
                        context.history.end_change();

                    if (changed)
                        world->set_physics_body_params(id, params);
                }
            }

            if (world->has_collider(id))
            {
                bool keep_collider = true;
                const bool collider_open = ImGui::CollapsingHeader(
                    "Collider", &keep_collider, ImGuiTreeNodeFlags_DefaultOpen);
                if (!keep_collider)
                {
                    context.history.record(*world);
                    world->set_has_collider(id, false);
                }
                else if (collider_open)
                {
                    SushiEngine::Simulation::ColliderParams params = world->collider_params(id);
                    bool changed = false;

                    static const char* const KIND_NAMES[] = {"Box", "Sphere", "Cylinder", "Plane"};
                    int kind_index = static_cast<int>(params.kind);
                    if (ImGui::Combo("Kind", &kind_index, KIND_NAMES, 4))
                    {
                        context.history.record(*world);
                        params.kind = static_cast<SushiEngine::Simulation::PrimitiveKind>(kind_index);
                        changed = true;
                    }

                    if (params.kind == SushiEngine::Simulation::PrimitiveKind::Plane)
                    {
                        float normal[3] = {static_cast<float>(params.params.x),
                                          static_cast<float>(params.params.y),
                                          static_cast<float>(params.params.z)};
                        if (ImGui::DragFloat3("Normal", normal, 0.01f, -1.0f, 1.0f, "%.3f"))
                        {
                            params.params =
                                SushiEngine::Vector3{normal[0], normal[1], normal[2]};
                            changed = true;
                        }
                    }
                    else
                    {
                        float values[3] = {static_cast<float>(params.params.x),
                                          static_cast<float>(params.params.y),
                                          static_cast<float>(params.params.z)};
                        const char* label = params.kind == SushiEngine::Simulation::PrimitiveKind::Box
                                               ? "Half Extents##Collider"
                                               : "Radius / Half Height##Collider";
                        if (ImGui::DragFloat3(label, values, 0.01f, 0.01f, 1000.0f, "%.3f"))
                        {
                            params.params = SushiEngine::Vector3{values[0], values[1], values[2]};
                            changed = true;
                        }
                    }
                    if (ImGui::IsItemActivated())
                        context.history.begin_change(*world);
                    if (ImGui::IsItemDeactivatedAfterEdit())
                        context.history.end_change();

                    if (changed)
                        world->set_collider_params(id, params);
                }
            }

            if (world->has_cloth(id))
            {
                bool keep_cloth = true;
                const bool cloth_open = ImGui::CollapsingHeader(
                    "Cloth", &keep_cloth, ImGuiTreeNodeFlags_DefaultOpen);
                if (!keep_cloth)
                {
                    context.history.record(*world);
                    world->set_has_cloth(id, false);
                }
                else if (cloth_open)
                {
                    SushiEngine::Simulation::ClothParams params = world->cloth_params(id);
                    bool changed = false;

                    int rows = static_cast<int>(params.rows);
                    if (ImGui::DragInt("Rows", &rows, 0.1f, 1, 64))
                    {
                        params.rows = static_cast<std::size_t>(rows < 1 ? 1 : rows);
                        changed = true;
                    }
                    if (ImGui::IsItemActivated())
                        context.history.begin_change(*world);
                    if (ImGui::IsItemDeactivatedAfterEdit())
                        context.history.end_change();

                    int cols = static_cast<int>(params.cols);
                    if (ImGui::DragInt("Columns", &cols, 0.1f, 1, 64))
                    {
                        params.cols = static_cast<std::size_t>(cols < 1 ? 1 : cols);
                        changed = true;
                    }
                    if (ImGui::IsItemActivated())
                        context.history.begin_change(*world);
                    if (ImGui::IsItemDeactivatedAfterEdit())
                        context.history.end_change();

                    float spacing = static_cast<float>(params.spacing);
                    if (ImGui::DragFloat("Spacing", &spacing, 0.01f, 0.01f, 10.0f, "%.3f"))
                    {
                        params.spacing = static_cast<SushiEngine::Scalar>(spacing < 0.01f ? 0.01f : spacing);
                        changed = true;
                    }
                    if (ImGui::IsItemActivated())
                        context.history.begin_change(*world);
                    if (ImGui::IsItemDeactivatedAfterEdit())
                        context.history.end_change();

                    float compliance = static_cast<float>(params.compliance);
                    if (ImGui::DragFloat("Compliance", &compliance, 0.0001f, 0.0f, 1.0f, "%.5f"))
                    {
                        params.compliance = static_cast<SushiEngine::Scalar>(compliance < 0.0f ? 0.0f : compliance);
                        changed = true;
                    }
                    if (ImGui::IsItemActivated())
                        context.history.begin_change(*world);
                    if (ImGui::IsItemDeactivatedAfterEdit())
                        context.history.end_change();

                    if (changed)
                        world->set_cloth_params(id, params);
                }
            }

            if (world->has_ui(id))
            {
                bool keep_ui = true;
                const bool ui_open = ImGui::CollapsingHeader(
                    "UI Element", &keep_ui, ImGuiTreeNodeFlags_DefaultOpen);
                if (!keep_ui)
                {
                    context.history.record(*world);
                    world->set_has_ui(id, false);
                }
                else if (ui_open)
                {
                    SushiEngine::Simulation::UIElementParams params = world->ui_params(id);
                    bool changed = false;

                    static const char* const UI_KINDS[] = {"Canvas", "Panel", "Image", "Text",
                                                           "Button"};
                    int kind_index = static_cast<int>(params.kind);
                    if (ImGui::Combo("Kind##UI", &kind_index, UI_KINDS, 5))
                    {
                        context.history.record(*world);
                        params.kind =
                            static_cast<SushiEngine::Simulation::UIElementKind>(kind_index);
                        changed = true;
                    }

                    const bool is_canvas =
                        params.kind == SushiEngine::Simulation::UIElementKind::Canvas;
                    const bool has_text =
                        params.kind == SushiEngine::Simulation::UIElementKind::Text ||
                        params.kind == SushiEngine::Simulation::UIElementKind::Button;

                    if (!is_canvas)
                    {
                        float anchor_min[2] = {static_cast<float>(params.anchor_min_x),
                                               static_cast<float>(params.anchor_min_y)};
                        if (ImGui::DragFloat2("Anchor Min", anchor_min, 0.01f, 0.0f, 1.0f, "%.2f"))
                        {
                            params.anchor_min_x = anchor_min[0];
                            params.anchor_min_y = anchor_min[1];
                            changed = true;
                        }
                        float anchor_max[2] = {static_cast<float>(params.anchor_max_x),
                                               static_cast<float>(params.anchor_max_y)};
                        if (ImGui::DragFloat2("Anchor Max", anchor_max, 0.01f, 0.0f, 1.0f, "%.2f"))
                        {
                            params.anchor_max_x = anchor_max[0];
                            params.anchor_max_y = anchor_max[1];
                            changed = true;
                        }
                        float pivot[2] = {static_cast<float>(params.pivot_x),
                                          static_cast<float>(params.pivot_y)};
                        if (ImGui::DragFloat2("Pivot", pivot, 0.01f, 0.0f, 1.0f, "%.2f"))
                        {
                            params.pivot_x = pivot[0];
                            params.pivot_y = pivot[1];
                            changed = true;
                        }
                        float position[2] = {static_cast<float>(params.position_x),
                                             static_cast<float>(params.position_y)};
                        if (ImGui::DragFloat2("Position", position, 1.0f, -8192.0f, 8192.0f, "%.0f"))
                        {
                            params.position_x = position[0];
                            params.position_y = position[1];
                            changed = true;
                        }
                    }

                    float size[2] = {static_cast<float>(params.size_x),
                                     static_cast<float>(params.size_y)};
                    if (ImGui::DragFloat2(is_canvas ? "Reference Size" : "Size", size, 1.0f, 0.0f,
                                          8192.0f, "%.0f"))
                    {
                        params.size_x = size[0];
                        params.size_y = size[1];
                        changed = true;
                    }
                    if (ImGui::IsItemActivated())
                        context.history.begin_change(*world);
                    if (ImGui::IsItemDeactivatedAfterEdit())
                        context.history.end_change();

                    if (!is_canvas)
                    {
                        float color[3] = {static_cast<float>(params.color.x),
                                          static_cast<float>(params.color.y),
                                          static_cast<float>(params.color.z)};
                        if (ImGui::ColorEdit3("Color##UI", color))
                        {
                            params.color = SushiEngine::Vector3{color[0], color[1], color[2]};
                            changed = true;
                        }
                        if (ImGui::IsItemActivated())
                            context.history.begin_change(*world);
                        if (ImGui::IsItemDeactivatedAfterEdit())
                            context.history.end_change();

                        float alpha = static_cast<float>(params.alpha);
                        if (ImGui::DragFloat("Opacity", &alpha, 0.01f, 0.0f, 1.0f, "%.2f"))
                        {
                            params.alpha = static_cast<SushiEngine::Scalar>(alpha);
                            changed = true;
                        }
                    }

                    if (has_text)
                    {
                        float font_size = static_cast<float>(params.font_size);
                        if (ImGui::DragFloat("Font Size", &font_size, 0.5f, 4.0f, 128.0f, "%.0f"))
                        {
                            params.font_size = static_cast<SushiEngine::Scalar>(font_size);
                            changed = true;
                        }
                        std::string text = params.text;
                        if (ImGui::InputText("Text", &text))
                        {
                            std::snprintf(params.text, sizeof(params.text), "%s", text.c_str());
                            changed = true;
                        }
                        if (ImGui::IsItemActivated())
                            context.history.begin_change(*world);
                        if (ImGui::IsItemDeactivatedAfterEdit())
                            context.history.end_change();
                    }

                    if (changed)
                        world->set_ui_params(id, params);
                }
            }

            // Script (custom) components. Any type discovered here that is not yet in
            // the Add Component catalog is registered, so a scene loaded from disk
            // repopulates the menu without a separate load hook.
            for (const std::string& type_name : world->script_components(id))
            {
                register_script_definition(context, world->script_component(id, type_name));

                bool keep_script = true;
                ImGui::PushID(type_name.c_str());
                const bool script_open = ImGui::CollapsingHeader(
                    type_name.c_str(), &keep_script, ImGuiTreeNodeFlags_DefaultOpen);
                if (!keep_script)
                {
                    context.history.record(*world);
                    world->remove_script_component(id, type_name);
                }
                else if (script_open)
                {
                    SushiEngine::Simulation::ScriptComponent script =
                        world->script_component(id, type_name);
                    if (draw_script_fields(context, *world, script))
                        world->set_script_component(id, script);
                }
                ImGui::PopID();
            }

            ImGui::Separator();
            if (ImGui::Button("Add Component"))
                ImGui::OpenPopup("AddComponentPopup");
            if (ImGui::BeginPopup("AddComponentPopup"))
            {
                if (!world->has_renderer(id) && ImGui::MenuItem("Renderer"))
                {
                    // A Renderer comes with a default Box mesh, since the mesh is the
                    // Renderer's own property here (see the Renderer header above).
                    context.history.record(*world);
                    world->set_has_renderer(id, true);
                    world->set_has_shape(id, true);
                }
                if (!world->is_camera(id) && ImGui::MenuItem("Camera"))
                {
                    context.history.record(*world);
                    world->set_is_camera(id, true);
                }
                if (!world->has_physics_body(id) && ImGui::MenuItem("Rigid Body"))
                {
                    context.history.record(*world);
                    world->set_has_physics_body(id, true);
                }
                if (!world->has_cloth(id) && ImGui::MenuItem("Cloth"))
                {
                    context.history.record(*world);
                    world->set_has_cloth(id, true);
                }
                if (!world->has_collider(id) && ImGui::MenuItem("Collider"))
                {
                    context.history.record(*world);
                    world->set_has_collider(id, true);
                }
                if (!world->has_ui(id) && ImGui::MenuItem("UI Element"))
                {
                    context.history.record(*world);
                    world->set_has_ui(id, true);
                }

                // User-defined script components: every catalog entry not already on
                // the entity, plus the New Script scaffold. This is the editor's
                // MonoBehaviour-style "attach a custom component" surface.
                if (ImGui::BeginMenu("Scripts"))
                {
                    for (const SushiEngine::Simulation::ScriptComponent& definition :
                         context.script_catalog)
                    {
                        if (world->has_script_component(id, definition.type_name))
                            continue;
                        if (ImGui::MenuItem(definition.type_name.c_str()))
                        {
                            context.history.record(*world);
                            world->add_script_component(id, definition);
                        }
                    }
                    ImGui::Separator();
                    if (ImGui::MenuItem("New Script..."))
                    {
                        context.show_new_script = true;
                        context.new_script_name.clear();
                        context.new_script_target = id;
                    }
                    ImGui::EndMenu();
                }
                ImGui::EndPopup();
            }

            ImGui::End();
        }

        void draw_environment_panel(EditorContext& context)
        {
            if (!context.panels.environment)
                return;
            if (!ImGui::Begin("Environment", &context.panels.environment))
            {
                ImGui::End();
                return;
            }

            IWorldEditor* world = world_of(context);
            if (world == nullptr)
            {
                ImGui::TextUnformatted("No scene open.");
                ImGui::End();
                return;
            }

            SushiEngine::Render::Environment environment = world->environment();
            bool changed = false;

            if (ImGui::CollapsingHeader("Sun", ImGuiTreeNodeFlags_DefaultOpen))
            {
                // Author the sun as azimuth/elevation, derived from and written back into
                // its unit direction so the panel holds no state of its own.
                const SushiEngine::Vector3 dir =
                    SushiEngine::normalize(environment.sun.direction);
                float elevation = std::asin(static_cast<float>(
                    dir.y < -1.0 ? -1.0 : (dir.y > 1.0 ? 1.0 : dir.y))) * 57.29578f;
                float azimuth = std::atan2(static_cast<float>(dir.z),
                                           static_cast<float>(dir.x)) * 57.29578f;
                bool sun_moved = false;
                // The astronomical sun (Solar System section) overrides this manual
                // direction downstream, so disable the sliders while it is on.
                ImGui::BeginDisabled(context.sky_astronomical_sun);
                if (ImGui::SliderFloat("Elevation", &elevation, -10.0f, 90.0f, "%.1f deg"))
                    sun_moved = true;
                if (ImGui::SliderFloat("Azimuth", &azimuth, -180.0f, 180.0f, "%.1f deg"))
                    sun_moved = true;
                ImGui::EndDisabled();
                if (sun_moved)
                {
                    const float e = elevation / 57.29578f;
                    const float a = azimuth / 57.29578f;
                    environment.sun.direction = SushiEngine::Vector3{
                        std::cos(e) * std::cos(a), std::sin(e), std::cos(e) * std::sin(a)};
                    changed = true;
                }

                float sun_color[3] = {static_cast<float>(environment.sun.color.x),
                                      static_cast<float>(environment.sun.color.y),
                                      static_cast<float>(environment.sun.color.z)};
                if (ImGui::ColorEdit3("Sun Color", sun_color))
                {
                    environment.sun.color =
                        SushiEngine::Vector3{sun_color[0], sun_color[1], sun_color[2]};
                    changed = true;
                }
                if (ImGui::SliderFloat("Sun Intensity", &environment.sun.intensity, 0.0f, 40.0f))
                    changed = true;
            }

            if (ImGui::CollapsingHeader("Atmosphere", ImGuiTreeNodeFlags_DefaultOpen))
            {
                if (ImGui::Checkbox("Atmosphere Enabled", &environment.atmosphere.enabled))
                    changed = true;
                if (ImGui::SliderFloat("Exposure", &environment.exposure, 0.02f, 1.0f))
                    changed = true;
            }

            if (ImGui::CollapsingHeader("Clouds"))
            {
                if (ImGui::Checkbox("Clouds Enabled", &environment.clouds.enabled))
                    changed = true;
                if (ImGui::SliderFloat("Coverage", &environment.clouds.coverage, 0.0f, 1.0f))
                    changed = true;
                if (ImGui::SliderFloat("Density", &environment.clouds.density, 0.0f, 1.0f))
                    changed = true;
            }

            if (ImGui::CollapsingHeader("Stars"))
            {
                if (ImGui::Checkbox("Stars Enabled", &environment.stars.enabled))
                    changed = true;
                if (ImGui::SliderFloat("Star Brightness", &environment.stars.brightness, 0.0f, 4.0f))
                    changed = true;
                if (ImGui::SliderFloat("Star Density", &environment.stars.density, 0.0f, 1.0f))
                    changed = true;
            }

            // The solar-system sky edits the editor context, not the world's environment:
            // the ephemeris repopulates the bodies and stars from these each frame in the
            // main loop, so scrubbing the date never re-extracts the world.
            if (ImGui::CollapsingHeader("Solar System", ImGuiTreeNodeFlags_DefaultOpen))
            {
                ImGui::Checkbox("Sky Enabled", &context.sky_enabled);
                ImGui::SameLine();
                ImGui::Checkbox("Astronomical Sun", &context.sky_astronomical_sun);

                // Interplanetary flight: detaches from Earth's surface and flies the whole
                // solar system in a heliocentric frame. The camera speed scales with the
                // distance to the nearest body, so approaching a planet slows to a crawl.
                ImGui::Checkbox("Space Mode (interplanetary)", &context.sky_space_mode);
                if (context.sky_space_mode)
                    ImGui::TextDisabled("Flying heliocentric. Fly toward a body to approach it.");

                ImGui::InputInt("Year", &context.sky_date.year);
                ImGui::SliderInt("Month", &context.sky_date.month, 1, 12);
                ImGui::SliderInt("Day", &context.sky_date.day, 1, 31);
                ImGui::SliderInt("Hour", &context.sky_date.hour, 0, 23);
                ImGui::SliderInt("Minute", &context.sky_date.minute, 0, 59);

                float latitude = static_cast<float>(context.sky_latitude_degrees);
                if (ImGui::SliderFloat("Latitude", &latitude, -90.0f, 90.0f, "%.2f deg"))
                    context.sky_latitude_degrees = latitude;
                float longitude = static_cast<float>(context.sky_longitude_degrees);
                if (ImGui::SliderFloat("Longitude", &longitude, -180.0f, 180.0f, "%.2f deg"))
                    context.sky_longitude_degrees = longitude;

                ImGui::Checkbox("Animate Time", &context.sky_animate);
                float days_per_second = static_cast<float>(context.sky_days_per_second);
                if (ImGui::SliderFloat("Days / Second", &days_per_second, 0.0f, 60.0f, "%.3f"))
                    context.sky_days_per_second = days_per_second;
                if (ImGui::Button("Reset Time Offset"))
                    context.sky_accumulated_days = 0.0;
                ImGui::SameLine();
                ImGui::Text("Offset: %.2f d", context.sky_accumulated_days);
            }

            if (changed)
                world->set_environment(environment);

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
            const fs::path current(context.current_directory);

            // Breadcrumb path and up-navigation, above the tree/grid split.
            ImGui::TextDisabled("%s", current.string().c_str());
            std::error_code root_ec;
            const bool at_root =
                fs::weakly_canonical(current, root_ec) == fs::weakly_canonical(root, root_ec);
            ImGui::SameLine();
            ImGui::BeginDisabled(at_root);
            if (ImGui::SmallButton("Up"))
                context.current_directory = current.parent_path().string();
            ImGui::EndDisabled();
            ImGui::Separator();

            // Left: a folder tree rooted at the project. Right: a Unity-style icon grid
            // of the current folder's contents. The tree only ever shows directories; the
            // grid shows both, so files are reachable without cluttering the tree.
            ImGui::BeginChild("project_tree", ImVec2(180.0f, 0.0f), true);
            draw_project_tree_node(context, root);
            ImGui::EndChild();

            ImGui::SameLine();
            ImGui::BeginChild("project_grid", ImVec2(0.0f, 0.0f), true);

            ImGui::SetNextItemWidth(-FLT_MIN);
            ImGui::InputTextWithHint("##project_filter", "Search...", &context.project_filter);
            const std::string lower_filter = to_lower(context.project_filter);

            std::error_code ec;
            std::vector<fs::directory_entry> entries;
            for (const auto& entry : fs::directory_iterator(current, ec))
            {
                if (!lower_filter.empty() &&
                    to_lower(entry.path().filename().string()).find(lower_filter) == std::string::npos)
                    continue;
                entries.push_back(entry);
            }
            std::sort(entries.begin(), entries.end(),
                      [](const fs::directory_entry& a, const fs::directory_entry& b)
                      {
                          const bool a_dir = a.is_directory();
                          const bool b_dir = b.is_directory();
                          if (a_dir != b_dir)
                              return a_dir;
                          return a.path().filename().string() < b.path().filename().string();
                      });

            constexpr float TILE_SIZE = 76.0f;
            constexpr float TILE_SPACING = 8.0f;
            const float avail_width = ImGui::GetContentRegionAvail().x;
            float row_x = 0.0f;

            // The delete target is deferred out of the loop so the directory listing is
            // never mutated (and filesystem-iterated again next frame) mid-walk.
            fs::path delete_target;

            for (std::size_t i = 0; i < entries.size(); ++i)
            {
                const fs::directory_entry& entry = entries[i];
                const bool is_dir = entry.is_directory();
                const std::string path_string = entry.path().string();
                const std::string name = entry.path().filename().string();

                if (row_x + TILE_SIZE > avail_width && row_x > 0.0f)
                    row_x = 0.0f;
                else if (i > 0 && row_x > 0.0f)
                    ImGui::SameLine();
                row_x += TILE_SIZE + TILE_SPACING;

                ImGui::PushID(path_string.c_str());
                ImGui::BeginGroup();

                if (context.renaming_project_path == path_string)
                {
                    // Inline rename: mirrors the Hierarchy's pattern — an autofocused field
                    // seeded once as the target changes, committed on Enter or focus loss.
                    static std::string buffer;
                    static std::string active;
                    if (active != path_string)
                    {
                        buffer = name;
                        active = path_string;
                        ImGui::SetKeyboardFocusHere();
                    }
                    ImGui::Dummy(ImVec2(TILE_SIZE, TILE_SIZE * 0.6f));
                    ImGui::SetNextItemWidth(TILE_SIZE);
                    const bool commit =
                        ImGui::InputText("##rename", &buffer,
                                         ImGuiInputTextFlags_EnterReturnsTrue |
                                             ImGuiInputTextFlags_AutoSelectAll);
                    if (commit || ImGui::IsItemDeactivated())
                    {
                        std::error_code rename_ec;
                        const fs::path renamed = entry.path().parent_path() / buffer;
                        if (!buffer.empty() && renamed != entry.path())
                            fs::rename(entry.path(), renamed, rename_ec);
                        context.renaming_project_path.clear();
                        active.clear();
                    }
                }
                else
                {
                    const ImU32 color = tile_color(entry.path(), is_dir);
                    ImGui::PushStyleColor(ImGuiCol_Button, color);
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, color);
                    ImGui::PushStyleColor(ImGuiCol_ButtonActive, color);
                    const std::string label =
                        (is_dir ? std::string("[D]") : entry.path().extension().string()) + "\n" +
                        truncate_label(name) + "##tile";
                    const bool clicked = ImGui::Button(label.c_str(), ImVec2(TILE_SIZE, TILE_SIZE));
                    ImGui::PopStyleColor(3);
                    if (clicked)
                        context.selected_project_path = path_string;
                    // Double-click detection is independent of the Button's own
                    // pressed-on-release return, which can miss the second click of a
                    // fast double-click; hover + IsMouseDoubleClicked is the reliable pair.
                    if (ImGui::IsItemHovered() &&
                        ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
                    {
                        context.selected_project_path = path_string;
                        if (is_dir)
                            context.current_directory = path_string;
                        else if (entry.path().extension() == ".sushiscene")
                        {
                            request_open_scene(context, path_string);
                        }
                        else if (has_text_extension(entry.path()))
                            open_document(context, entry.path());
                        else
                            open_with_default_app(entry.path());
                    }

                    if (ImGui::BeginPopupContextItem())
                    {
                        context.selected_project_path = path_string;
                        if (!is_dir && ImGui::MenuItem("Open"))
                        {
                            if (entry.path().extension() == ".sushiscene")
                            {
                                request_open_scene(context, path_string);
                            }
                            else if (has_text_extension(entry.path()))
                                open_document(context, entry.path());
                            else
                                open_with_default_app(entry.path());
                        }
                        if (ImGui::MenuItem("Rename"))
                            context.renaming_project_path = path_string;
                        if (ImGui::MenuItem("Delete"))
                            delete_target = entry.path();
                        if (ImGui::MenuItem("Show in Explorer"))
                            show_in_explorer(entry.path());
                        ImGui::Separator();
                        draw_project_create_menu(context, current);
                        ImGui::EndPopup();
                    }
                }

                ImGui::EndGroup();
                ImGui::PopID();
            }

            // Right-click on empty grid space: create new items in the current folder.
            if (ImGui::BeginPopupContextWindow("project_grid_context", ImGuiPopupFlags_MouseButtonRight |
                                                                           ImGuiPopupFlags_NoOpenOverItems))
            {
                draw_project_create_menu(context, current);
                if (ImGui::MenuItem("Show in Explorer"))
                    show_in_explorer(current);
                ImGui::EndPopup();
            }

            if (!delete_target.empty())
            {
                std::error_code delete_ec;
                const std::string deleted_string = delete_target.string();
                fs::remove_all(delete_target, delete_ec);
                if (context.selected_project_path == deleted_string)
                    context.selected_project_path.clear();
                for (std::size_t i = 0; i < context.documents.size(); ++i)
                    if (context.documents[i].path == deleted_string)
                    {
                        context.documents.erase(context.documents.begin() +
                                                static_cast<std::ptrdiff_t>(i));
                        if (context.active_document >= static_cast<int>(i))
                            --context.active_document;
                        break;
                    }
            }

            ImGui::EndChild();
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
                    if (context.simulation != nullptr && context.play_mode_snapshot.has_value())
                    {
                        apply_scene(context.simulation->world(), *context.play_mode_snapshot);
                        context.play_mode_snapshot.reset();
                        select_only(context, SushiEngine::Simulation::NULL_ENTITY);
                    }
                    editor_log(context, "Playback stopped; scene restored.");
                }
                else
                {
                    context.play_state = PlayState::Playing;
                    if (context.simulation != nullptr)
                        context.play_mode_snapshot = capture_scene(context.simulation->world());
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
            {
                context.step_requested = true;
                editor_log(context, "Stepped one frame.");
            }
            ImGui::EndDisabled();

            ImGui::SameLine();
            const char* state_text = context.play_state == PlayState::Playing ? "Playing"
                                     : context.play_state == PlayState::Paused ? "Paused"
                                                                               : "Stopped";
            ImGui::TextDisabled("| %s", state_text);

            // Transform tool selector, mirroring Unity's W/E/R. The hotkeys apply only
            // when no text field is capturing keys, so typing a name never switches tools.
            ImGui::SameLine();
            ImGui::TextDisabled("|");
            const GizmoMode modes[3] = {GizmoMode::Translate, GizmoMode::Rotate, GizmoMode::Scale};
            const char* mode_labels[3] = {"Move", "Rotate", "Scale"};
            for (int i = 0; i < 3; ++i)
            {
                ImGui::SameLine();
                const bool active = context.gizmo_mode == modes[i];
                if (active)
                    ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
                if (ImGui::Button(mode_labels[i]))
                    context.gizmo_mode = modes[i];
                if (active)
                    ImGui::PopStyleColor();
            }

            // Local/World axis-frame toggle, mirroring Unity's gizmo-space button. Disabled
            // for Scale, which always drags in local axes (a world-aligned scale on a
            // rotated object would shear it).
            ImGui::SameLine();
            ImGui::TextDisabled("|");
            ImGui::SameLine();
            ImGui::BeginDisabled(context.gizmo_mode == GizmoMode::Scale);
            const char* space_label = context.gizmo_space == GizmoSpace::Local ? "Local" : "World";
            if (ImGui::Button(space_label))
                context.gizmo_space = context.gizmo_space == GizmoSpace::Local ? GizmoSpace::World
                                                                                : GizmoSpace::Local;
            ImGui::EndDisabled();

            // While right mouse is held the Scene camera owns WASD for flight, so the tool
            // hotkeys stand down to avoid switching tools as the user moves.
            if (!ImGui::GetIO().WantTextInput && !ImGui::IsMouseDown(ImGuiMouseButton_Right))
            {
                if (ImGui::IsKeyPressed(ImGuiKey_W, false))
                    context.gizmo_mode = GizmoMode::Translate;
                else if (ImGui::IsKeyPressed(ImGuiKey_E, false))
                    context.gizmo_mode = GizmoMode::Rotate;
                else if (ImGui::IsKeyPressed(ImGuiKey_R, false))
                    context.gizmo_mode = GizmoMode::Scale;
            }

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
                    const std::string scene_name =
                        context.scene_path.empty()
                            ? std::string("Untitled")
                            : fs::path(context.scene_path).filename().string();
                    ImGui::Text("Scene: %s%s", scene_name.c_str(),
                                scene_is_dirty(context) ? "*" : "");
                    ImGui::Separator();
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
                    if (world != nullptr && save_scene(*world, path.string()))
                    {
                        context.scene_path = path.string();
                        context.saved_scene_revision = context.history.revision();
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
                        editor_log(context, "Failed to save scene '" + path.string() + "'.");
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
    } // namespace Editor
} // namespace SushiEngine
