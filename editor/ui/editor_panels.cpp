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
#include "material_inspector.hpp"

#include "../serialization/scene_serializer.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
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

#include <SushiEngine/astro/celestial_bodies.hpp>
#include <SushiEngine/render/quality_params.hpp>
#include <SushiEngine/render/upscaler_info.hpp>

namespace fs = std::filesystem;

namespace SushiEngine
{
    namespace Editor
    {
        using SushiEngine::Simulation::EntityFrame;
        using SushiEngine::Simulation::EntityId;
        using SushiEngine::Simulation::EntityTransform;
        using SushiEngine::Simulation::FrameMode;
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

            // A single labelled scalar row, for fields that are not a homogeneous vector3
            // (the geodetic latitude/longitude/altitude of a Surface-frame position, whose
            // three components have different units and ranges).
            bool scalar_field(EditorContext& context, IWorldEditor& world, const char* label,
                              float* value, float speed, float min_value, float max_value,
                              const char* format)
            {
                ImGui::PushID(label);
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted(label);
                ImGui::TableSetColumnIndex(1);
                ImGui::SetNextItemWidth(-FLT_MIN);
                const bool changed =
                    ImGui::DragFloat("##v", value, speed, min_value, max_value, format);
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
                    if (ImGui::MenuItem("Light"))
                    {
                        context.history.record(*world);
                        select_only(context, world->create_light("Light"));
                        editor_log(context, "Created object 'Light'.");
                    }
                    if (ImGui::MenuItem("Decal"))
                    {
                        context.history.record(*world);
                        select_only(context, world->create_decal("Decal"));
                        editor_log(context, "Created object 'Decal'.");
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
                entry.has_light = world->has_light(id);
                entry.light_params = world->light_params(id);
                entry.has_decal = world->has_decal(id);
                entry.decal_params = world->decal_params(id);
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
                world->set_has_light(id, entry.has_light);
                if (entry.has_light)
                    world->set_light_params(id, entry.light_params);
                world->set_has_decal(id, entry.has_decal);
                if (entry.has_decal)
                    world->set_decal_params(id, entry.decal_params);
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

        namespace
        {
            // Mirrors the Environment panel's Solar System fields into the persisted shape,
            // so a scene save/load round-trips the date/time/location it was authored with.
            SceneSkyState capture_sky_state(const EditorContext& context)
            {
                SceneSkyState sky;
                sky.enabled = context.sky_enabled;
                sky.date = context.sky_date;
                sky.latitude_degrees = context.sky_latitude_degrees;
                sky.longitude_degrees = context.sky_longitude_degrees;
                sky.astronomical_sun = context.sky_astronomical_sun;
                sky.animate = context.sky_animate;
                sky.days_per_second = context.sky_days_per_second;
                sky.accumulated_days = context.sky_accumulated_days;
                return sky;
            }

            void apply_sky_state(EditorContext& context, const SceneSkyState& sky)
            {
                context.sky_enabled = sky.enabled;
                context.sky_date = sky.date;
                context.sky_latitude_degrees = sky.latitude_degrees;
                context.sky_longitude_degrees = sky.longitude_degrees;
                context.sky_astronomical_sun = sky.astronomical_sun;
                context.sky_animate = sky.animate;
                context.sky_days_per_second = sky.days_per_second;
                context.sky_accumulated_days = sky.accumulated_days;
                // Forces the main loop's date-change detection to re-seek the epoch from
                // the freshly-loaded date rather than keeping whatever scene came before.
                context.sky_authored_start_cache = -1.0;
            }
        } // namespace

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
            const SceneSkyState sky = capture_sky_state(context);
            if (save_scene(*world, context.scene_path, &sky))
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
                SceneSkyState sky = capture_sky_state(context);
                if (load_scene(*world, path, &sky))
                {
                    apply_sky_state(context, sky);
                    // Environment/Lighting are an editor (host) setting, not scene data, so
                    // the editor's own value wins over whatever the scene file's (legacy)
                    // environment block may still contain.
                    world->set_environment(context.preferences.environment);
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
                ImGui::MenuItem("Rendering", nullptr, &context.panels.rendering);
                ImGui::MenuItem("Lighting", nullptr, &context.panels.lighting);
                ImGui::MenuItem("Post Process", nullptr, &context.panels.post_process);
                ImGui::MenuItem("GPU Culling", nullptr, &context.panels.gpu_culling);
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
                // Reference row: a celestial body + interpretation mode. When a body is
                // picked the transform below is authored *frame-local* (small metres from the
                // body, ground-local rotation in Surface), the Unity-parent analogue with the
                // body as the parent; "Scene" (-1) is the plain scene transform, unchanged.
                EntityFrame frame = world->entity_frame(id);
                if (ImGui::BeginTable("reference", 2, ImGuiTableFlags_SizingStretchProp))
                {
                    ImGui::TableSetupColumn("label", ImGuiTableColumnFlags_WidthFixed, 80.0f);
                    ImGui::TableSetupColumn("value");
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::AlignTextToFramePadding();
                    ImGui::TextUnformatted("Reference");
                    ImGui::TableSetColumnIndex(1);

                    const char* current_body =
                        frame.reference_body < 0
                            ? "Scene"
                            : SushiEngine::Astro::body_properties(
                                  static_cast<SushiEngine::Astro::BodyId>(frame.reference_body))
                                  .name;
                    ImGui::SetNextItemWidth(-FLT_MIN);
                    if (ImGui::BeginCombo("##reference_body", current_body))
                    {
                        if (ImGui::Selectable("Scene", frame.reference_body < 0))
                        {
                            context.history.record(*world);
                            frame.reference_body = -1;
                            world->set_entity_frame(id, frame);
                        }
                        for (int b = 0; b < SushiEngine::Astro::BODY_COUNT; ++b)
                        {
                            const char* body_name =
                                SushiEngine::Astro::body_properties(
                                    static_cast<SushiEngine::Astro::BodyId>(b)).name;
                            if (ImGui::Selectable(body_name, frame.reference_body == b))
                            {
                                context.history.record(*world);
                                frame.reference_body = b;
                                world->set_entity_frame(id, frame);
                            }
                        }
                        ImGui::EndCombo();
                    }

                    // Mode is only meaningful once a body is the reference.
                    if (frame.reference_body >= 0)
                    {
                        ImGui::TableNextRow();
                        ImGui::TableSetColumnIndex(0);
                        ImGui::AlignTextToFramePadding();
                        ImGui::TextUnformatted("Mode");
                        ImGui::TableSetColumnIndex(1);
                        const char* modes[] = {"Auto", "Free", "Surface"};
                        int mode_index = static_cast<int>(frame.mode);
                        ImGui::SetNextItemWidth(-FLT_MIN);
                        if (ImGui::Combo("##frame_mode", &mode_index, modes, 3))
                        {
                            context.history.record(*world);
                            frame.mode = static_cast<FrameMode>(mode_index);
                            world->set_entity_frame(id, frame);
                        }
                    }
                    ImGui::EndTable();
                }

                // With a body picked, edit the frame-local transform; otherwise the plain
                // scene transform. The widget/write path is identical either way.
                const bool frame_local = frame.reference_body >= 0;
                EntityTransform transform =
                    frame_local ? world->frame_local_transform(id) : world->transform(id);
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

                // In Surface mode the position is geodetic (lat°, lon°, altitude m) — the
                // place-on-a-planet coordinate — so it is shown as three labelled fields
                // rather than an opaque X/Y/Z. Free/Scene keep the Cartesian Position vector.
                const bool surface = frame_local && world->is_surface_frame(id);

                bool changed = false;
                if (ImGui::BeginTable("transform", 2, ImGuiTableFlags_SizingStretchProp))
                {
                    ImGui::TableSetupColumn("label", ImGuiTableColumnFlags_WidthFixed, 80.0f);
                    ImGui::TableSetupColumn("value");
                    if (surface)
                    {
                        changed |= scalar_field(context, *world, "Latitude", &position[0], 0.01f,
                                                -90.0f, 90.0f, "%.5f deg");
                        changed |= scalar_field(context, *world, "Longitude", &position[1], 0.01f,
                                                -180.0f, 180.0f, "%.5f deg");
                        changed |= scalar_field(context, *world, "Altitude", &position[2], 0.1f,
                                                -1.0e7f, 1.0e9f, "%.2f m");
                    }
                    else
                    {
                        changed |= vector3_field(context, *world, "Position", position, 0.05f);
                    }
                    changed |= vector3_field(context, *world, "Rotation", rotation, 0.5f);
                    changed |= vector3_field(context, *world, "Scale", scale, 0.05f);
                    ImGui::EndTable();
                }

                if (changed)
                {
                    transform.position = SushiEngine::Vector3{position[0], position[1], position[2]};
                    transform.rotation = euler_degrees_to_quat(rotation);
                    transform.scale = SushiEngine::Vector3{scale[0], scale[1], scale[2]};
                    if (frame_local)
                        world->set_frame_local_transform(id, transform);
                    else
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

                    // The full PBR surface: maps, scalars, and rendering state, laid out
                    // like Unity's Standard shader. Albedo's tint is the Color row above,
                    // which the material editor leaves alone.
                    SushiEngine::Render::Material material = world->material(id);
                    if (context.assets != nullptr &&
                        draw_material_editor(material, *context.assets))
                    {
                        context.history.begin_change(*world);
                        world->set_material(id, material);
                        context.history.end_change();
                    }

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

                    float drag = static_cast<float>(params.drag_coefficient);
                    if (ImGui::DragFloat("Drag Coefficient", &drag, 0.001f, 0.0f, 100.0f, "%.4f"))
                    {
                        params.drag_coefficient =
                            static_cast<SushiEngine::Scalar>(drag < 0.0f ? 0.0f : drag);
                        changed = true;
                    }
                    if (ImGui::IsItemActivated())
                        context.history.begin_change(*world);
                    if (ImGui::IsItemDeactivatedAfterEdit())
                        context.history.end_change();
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("Quadratic drag k: acceleration -k|v|v (per metre). "
                                          "0 disables; higher = lower terminal speed.");

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

            if (world->surface_anchored(id))
            {
                bool keep_anchor = true;
                const bool anchor_open = ImGui::CollapsingHeader(
                    "Surface Anchor", &keep_anchor, ImGuiTreeNodeFlags_DefaultOpen);
                if (!keep_anchor)
                {
                    context.history.record(*world);
                    world->set_surface_anchored(id, false);
                }
                else if (anchor_open)
                {
                    ImGui::TextWrapped(
                        "Orientation is ground-local: the East-North-Up frame on the "
                        "dominant body is composed onto it each step, so the entity stays "
                        "upright anywhere on the planet. Rotate it to face along the "
                        "local horizon.");
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

            if (world->has_light(id))
            {
                bool keep_light = true;
                const bool light_open = ImGui::CollapsingHeader(
                    "Light", &keep_light, ImGuiTreeNodeFlags_DefaultOpen);
                if (!keep_light)
                {
                    context.history.record(*world);
                    world->set_has_light(id, false);
                }
                else if (light_open)
                {
                    SushiEngine::Simulation::LightParams params = world->light_params(id);
                    bool changed = false;

                    bool is_spot = params.is_spot;
                    if (ImGui::Checkbox("Spot", &is_spot))
                    {
                        context.history.record(*world);
                        params.is_spot = is_spot;
                        changed = true;
                    }

                    float color[3] = {static_cast<float>(params.color.x),
                                      static_cast<float>(params.color.y),
                                      static_cast<float>(params.color.z)};
                    if (ImGui::ColorEdit3("Color", color))
                    {
                        params.color = SushiEngine::Vector3{color[0], color[1], color[2]};
                        changed = true;
                    }
                    if (ImGui::IsItemActivated())
                        context.history.begin_change(*world);
                    if (ImGui::IsItemDeactivatedAfterEdit())
                        context.history.end_change();

                    if (ImGui::DragFloat("Intensity", &params.intensity, 0.1f, 0.0f, 1000.0f,
                                         "%.1f"))
                        changed = true;
                    if (ImGui::IsItemActivated())
                        context.history.begin_change(*world);
                    if (ImGui::IsItemDeactivatedAfterEdit())
                        context.history.end_change();

                    if (ImGui::DragFloat("Range", &params.range, 0.1f, 0.1f, 10000.0f, "%.2f"))
                        changed = true;
                    if (ImGui::IsItemActivated())
                        context.history.begin_change(*world);
                    if (ImGui::IsItemDeactivatedAfterEdit())
                        context.history.end_change();

                    if (is_spot)
                    {
                        if (ImGui::DragFloat("Inner Angle", &params.inner_degrees, 0.2f, 0.0f,
                                             89.0f, "%.1f deg"))
                            changed = true;
                        if (ImGui::IsItemActivated())
                            context.history.begin_change(*world);
                        if (ImGui::IsItemDeactivatedAfterEdit())
                            context.history.end_change();

                        if (ImGui::DragFloat("Outer Angle", &params.outer_degrees, 0.2f, 0.0f,
                                             89.0f, "%.1f deg"))
                            changed = true;
                        if (ImGui::IsItemActivated())
                            context.history.begin_change(*world);
                        if (ImGui::IsItemDeactivatedAfterEdit())
                            context.history.end_change();

                        bool casts = params.casts_shadows;
                        if (ImGui::Checkbox("Casts Shadows", &casts))
                        {
                            context.history.record(*world);
                            params.casts_shadows = casts;
                            changed = true;
                        }
                    }

                    if (changed)
                        world->set_light_params(id, params);
                }
            }

            if (world->has_decal(id))
            {
                bool keep_decal = true;
                const bool decal_open = ImGui::CollapsingHeader(
                    "Decal", &keep_decal, ImGuiTreeNodeFlags_DefaultOpen);
                if (!keep_decal)
                {
                    context.history.record(*world);
                    world->set_has_decal(id, false);
                }
                else if (decal_open)
                {
                    SushiEngine::Simulation::DecalParams params = world->decal_params(id);
                    bool changed = false;

                    float color[3] = {static_cast<float>(params.color.x),
                                      static_cast<float>(params.color.y),
                                      static_cast<float>(params.color.z)};
                    if (ImGui::ColorEdit3("Tint", color))
                    {
                        params.color = SushiEngine::Vector3{color[0], color[1], color[2]};
                        changed = true;
                    }
                    if (ImGui::IsItemActivated())
                        context.history.begin_change(*world);
                    if (ImGui::IsItemDeactivatedAfterEdit())
                        context.history.end_change();

                    float size[3] = {static_cast<float>(params.half_extents.x),
                                     static_cast<float>(params.half_extents.y),
                                     static_cast<float>(params.half_extents.z)};
                    if (ImGui::DragFloat3("Half Extents", size, 0.05f, 0.05f, 100.0f, "%.2f"))
                    {
                        params.half_extents = SushiEngine::Vector3{size[0], size[1], size[2]};
                        changed = true;
                    }
                    if (ImGui::IsItemActivated())
                        context.history.begin_change(*world);
                    if (ImGui::IsItemDeactivatedAfterEdit())
                        context.history.end_change();

                    if (ImGui::SliderFloat("Opacity", &params.opacity, 0.0f, 1.0f))
                        changed = true;
                    if (ImGui::IsItemActivated())
                        context.history.begin_change(*world);
                    if (ImGui::IsItemDeactivatedAfterEdit())
                        context.history.end_change();

                    // Optional projected maps, loaded through the asset library exactly as a
                    // material's are (path field + Load/Clear). The record stores only the
                    // opaque texture id, so the typed path lives in a UI-side map keyed by
                    // the widget id.
                    if (context.assets != nullptr)
                    {
                        static std::unordered_map<ImGuiID, std::string> decal_paths;
                        const auto map_field =
                            [&](const char* label, SushiEngine::Render::TextureId& tex,
                                SushiEngine::Render::TextureColorSpace cs)
                        {
                            ImGui::PushID(label);
                            const ImGuiID wid = ImGui::GetID(label);
                            std::string& stored = decal_paths[wid];
                            char buffer[512] = {};
                            stored.copy(buffer, sizeof(buffer) - 1);
                            ImGui::SetNextItemWidth(-140.0f);
                            const bool enter =
                                ImGui::InputText(label, buffer, sizeof(buffer),
                                                 ImGuiInputTextFlags_EnterReturnsTrue);
                            stored = buffer;
                            ImGui::SameLine();
                            const bool load = ImGui::SmallButton("Load");
                            if (enter || load)
                            {
                                context.history.record(*world);
                                const SushiEngine::Render::TextureId loaded =
                                    stored.empty()
                                        ? SushiEngine::Render::INVALID_TEXTURE
                                        : context.assets->load_texture(stored.c_str(), cs);
                                if (tex != SushiEngine::Render::INVALID_TEXTURE)
                                    context.assets->release_texture(tex);
                                tex = loaded;
                                changed = true;
                            }
                            ImGui::SameLine();
                            if (ImGui::SmallButton("Clear") &&
                                tex != SushiEngine::Render::INVALID_TEXTURE)
                            {
                                context.history.record(*world);
                                context.assets->release_texture(tex);
                                tex = SushiEngine::Render::INVALID_TEXTURE;
                                stored.clear();
                                changed = true;
                            }
                            ImGui::SameLine();
                            ImGui::TextDisabled(
                                "%s",
                                tex != SushiEngine::Render::INVALID_TEXTURE ? "set" : "none");
                            ImGui::PopID();
                        };
                        map_field("Albedo Map", params.albedo_map,
                                  SushiEngine::Render::TextureColorSpace::Srgb);
                        map_field("ORM Map", params.orm_map,
                                  SushiEngine::Render::TextureColorSpace::Linear);
                    }

                    if (changed)
                        world->set_decal_params(id, params);
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
                if (!world->has_light(id) && ImGui::MenuItem("Light"))
                {
                    context.history.record(*world);
                    world->set_has_light(id, true);
                }
                if (!world->has_decal(id) && ImGui::MenuItem("Decal"))
                {
                    context.history.record(*world);
                    world->set_has_decal(id, true);
                }
                if (!world->has_collider(id) && ImGui::MenuItem("Collider"))
                {
                    context.history.record(*world);
                    world->set_has_collider(id, true);
                }
                if (!world->surface_anchored(id) && ImGui::MenuItem("Surface Anchor"))
                {
                    context.history.record(*world);
                    world->set_surface_anchored(id, true);
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

        void draw_rendering_panel(EditorContext& context)
        {
            if (!context.panels.rendering)
                return;
            if (!ImGui::Begin("Rendering", &context.panels.rendering))
            {
                ImGui::End();
                return;
            }

            SushiEngine::Render::RenderSettings& settings = context.render_settings;
            using SushiEngine::Render::AntiAliasingMode;
            using SushiEngine::Render::RenderQuality;
            using SushiEngine::Render::UpscaleMode;

            // RenderSettings is plain trivially-copyable data (render_settings.hpp), so a
            // memcmp against a snapshot taken before the widgets run is a cheap, exhaustive
            // way to detect any edit below and persist it via Preferences — without hooking
            // a dirty flag into every slider individually.
            const SushiEngine::Render::RenderSettings settings_before = settings;

            const char* const QUALITY[] = {"Low", "Medium", "High", "Ultra"};
            int quality = static_cast<int>(settings.quality);
            if (ImGui::Combo("Quality", &quality, QUALITY, 4))
                settings.quality = static_cast<RenderQuality>(quality);

            const char* const ANTI_ALIASING[] = {"None", "FXAA", "Temporal"};
            int anti_aliasing = static_cast<int>(settings.anti_aliasing);
            if (ImGui::Combo("Anti-aliasing", &anti_aliasing, ANTI_ALIASING, 3))
                settings.anti_aliasing = static_cast<AntiAliasingMode>(anti_aliasing);

            ImGui::SliderFloat("Render Scale", &settings.render_scale, 0.5f, 1.0f, "%.2f");

            if (settings.anti_aliasing == AntiAliasingMode::Temporal)
            {
                ImGui::PushID("Temporal");
                ImGui::SliderFloat("Still Feedback", &settings.temporal.feedback_still,
                                   0.5f, 0.99f, "%.3f");
                ImGui::SliderFloat("Moving Feedback", &settings.temporal.feedback_moving,
                                   0.5f, 0.99f, "%.3f");
                ImGui::SliderFloat("Sharpness", &settings.temporal.sharpness, 0.0f, 1.0f,
                                   "%.2f");
                ImGui::SliderFloat("Jitter", &settings.temporal.jitter_scale, 0.0f, 1.0f,
                                   "%.2f");
                ImGui::Checkbox("Clamp History", &settings.temporal.clamp_history);
                ImGui::PopID();
            }
            else
            {
                ImGui::TextDisabled("Temporal options need the temporal resolve.");
            }

            ImGui::Separator();
            ImGui::Checkbox("Shadows", &settings.shadows.enabled);
            if (settings.shadows.enabled)
            {
                ImGui::PushID("Shadows");
                int cascades = static_cast<int>(settings.shadows.cascade_count);
                if (ImGui::SliderInt("Cascades", &cascades, 1, 4))
                    settings.shadows.cascade_count = static_cast<std::uint32_t>(cascades);

                const char* const RESOLUTIONS[] = {"512", "1024", "2048", "4096"};
                const std::uint32_t VALUES[] = {512u, 1024u, 2048u, 4096u};
                int resolution = 2;
                for (int i = 0; i < 4; ++i)
                    if (VALUES[i] == settings.shadows.resolution)
                        resolution = i;
                if (ImGui::Combo("Resolution", &resolution, RESOLUTIONS, 4))
                    settings.shadows.resolution = VALUES[resolution];

                ImGui::SliderFloat("Distance (m)", &settings.shadows.distance, 20.0f, 4000.0f,
                                   "%.0f");
                ImGui::SliderFloat("Split Blend", &settings.shadows.split_blend, 0.0f, 1.0f,
                                   "%.2f");
                ImGui::SliderFloat("Normal Bias", &settings.shadows.normal_bias, 0.0f, 6.0f,
                                   "%.2f");
                ImGui::SliderFloat("Depth Bias", &settings.shadows.depth_bias, 0.0f, 0.01f,
                                   "%.4f");
                ImGui::SliderFloat("Softness", &settings.shadows.softness, 0.0f, 10.0f, "%.2f");
                ImGui::SliderFloat("Min Filter", &settings.shadows.filter_radius, 0.5f, 8.0f,
                                   "%.2f");
                ImGui::SliderFloat("Max Filter", &settings.shadows.max_filter_radius, 2.0f,
                                   48.0f, "%.1f");
                ImGui::SliderFloat("Cascade Blend", &settings.shadows.cascade_blend, 0.0f, 0.5f,
                                   "%.2f");

                ImGui::Checkbox("Contact Shadows", &settings.shadows.contact_shadows);
                if (settings.shadows.contact_shadows)
                {
                    ImGui::SliderFloat("Contact Reach (m)", &settings.shadows.contact_distance,
                                       0.05f, 2.0f, "%.2f");
                    int steps = static_cast<int>(settings.shadows.contact_steps);
                    if (ImGui::SliderInt("Contact Steps", &steps, 4, 32))
                        settings.shadows.contact_steps = static_cast<std::uint32_t>(steps);
                }
                ImGui::PopID();
            }

            ImGui::Separator();
            ImGui::Checkbox("Dynamic Resolution", &settings.dynamic_resolution.enabled);
            if (settings.dynamic_resolution.enabled)
            {
                ImGui::PushID("DynamicResolution");
                ImGui::SliderFloat("GPU Budget (ms)",
                                   &settings.dynamic_resolution.target_milliseconds, 2.0f,
                                   33.0f, "%.1f");
                ImGui::SliderFloat("Minimum Scale",
                                   &settings.dynamic_resolution.minimum_scale, 0.25f, 1.0f,
                                   "%.2f");
                ImGui::SliderFloat("Maximum Scale",
                                   &settings.dynamic_resolution.maximum_scale, 0.25f, 1.0f,
                                   "%.2f");
                ImGui::PopID();
            }

            ImGui::Separator();
            ImGui::Checkbox("Variable Rate Shading",
                            &settings.variable_rate_shading.enabled);
            if (settings.variable_rate_shading.enabled)
            {
                ImGui::PushID("VariableRateShading");
                ImGui::SliderFloat("Contrast Threshold",
                                   &settings.variable_rate_shading.luminance_threshold, 0.0f,
                                   0.5f, "%.3f");
                ImGui::SliderFloat("Motion Threshold",
                                   &settings.variable_rate_shading.velocity_threshold, 0.0f,
                                   0.2f, "%.3f");
                ImGui::TextDisabled("Ignored on a device without shading rate images.");
                ImGui::PopID();
            }

            // What the resolution governor actually settled on, which is the only way to
            // see it work: the slider is a request, this is the answer.
            ImGui::Separator();
            if (context.scene_render_width > 0)
                ImGui::Text("Scene view rendering at %u x %u", context.scene_render_width,
                            context.scene_render_height);

            // What the tier resolves to. The Quality combo above is a request; this is what
            // each pass is actually handed once the resolver has run — the same values the
            // renderer uses, so switching tiers shows the expensive half rescale here and,
            // in milliseconds, in the profiler HUD over the viewport.
            if (ImGui::TreeNode("Tier resolves to"))
            {
                const SushiEngine::Render::ResolvedQuality resolved =
                    SushiEngine::Render::resolve_quality(settings);
                const SushiEngine::Render::RenderSettings& effective = resolved.settings;
                const SushiEngine::Render::QualityParams& knobs = resolved.params;

                if (effective.shadows.enabled)
                {
                    ImGui::Text("Shadow atlas   %u px, %u cascade(s)", effective.shadows.resolution,
                                effective.shadows.cascade_count);
                    ImGui::Text("PCSS taps      %u filter / %u blocker", knobs.shadow_filter_taps,
                                knobs.shadow_blocker_taps);
                    if (effective.shadows.contact_shadows)
                        ImGui::Text("Contact march  %u steps @ %.2f m",
                                    effective.shadows.contact_steps,
                                    effective.shadows.contact_distance);
                }
                else
                {
                    ImGui::TextDisabled("Shadows off");
                }
                ImGui::Text("Cloud march    %u near / %u far / %u light",
                            knobs.cloud_primary_steps_near, knobs.cloud_primary_steps_far,
                            knobs.cloud_light_steps);
                ImGui::Text("VRS coarse cap %ux (1 = full rate)", knobs.vrs_max_coarse_axis);
                ImGui::Text("Punctual lights %u max, %.0f m cluster reach",
                            effective.lights.max_lights, effective.lights.cluster_far_distance);
                ImGui::Text("Decals         %u max; shadow atlas %u px / %u caster(s)",
                            effective.lights.max_decals, effective.lights.shadow_atlas_size,
                            effective.lights.max_shadow_casters);
                ImGui::Text("Lobes          %s%s%s%s",
                            knobs.lobe_anisotropy ? "aniso " : "",
                            knobs.lobe_clearcoat ? "clearcoat " : "",
                            knobs.lobe_sheen ? "sheen " : "",
                            knobs.lobe_transmission ? "transmission" : "");
                if (!knobs.lobe_anisotropy && !knobs.lobe_clearcoat && !knobs.lobe_sheen &&
                    !knobs.lobe_transmission)
                {
                    ImGui::SameLine();
                    ImGui::TextDisabled("base PBR only");
                }
                ImGui::Text("Async compute  %s", knobs.async_compute ? "permitted" : "off (tier)");
                ImGui::Text("Frames in flight %u", effective.delivery.frames_in_flight);
                ImGui::TreePop();
            }

            ImGui::Separator();
            // Delivery, not fidelity: nothing here changes a pixel, only how much of the
            // device is kept busy and how long a finished frame waits to be seen.
            if (ImGui::TreeNode("Frame Delivery"))
            {
                using SushiEngine::Render::PresentMode;
                SushiEngine::Render::FrameDeliverySettings& delivery = settings.delivery;

                ImGui::Checkbox("Async Compute", &delivery.async_compute);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Runs the flagged compute passes (cluster build, GTAO)\n"
                                      "on a second queue so they overlap the graphics work\n"
                                      "they do not depend on. Ignored where the device has\n"
                                      "no compute queue family of its own.");

                int in_flight = static_cast<int>(delivery.frames_in_flight);
                if (ImGui::SliderInt("Frames in Flight", &in_flight, 2, 3))
                    delivery.frames_in_flight = static_cast<std::uint32_t>(in_flight);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("How far the CPU may run ahead of the GPU.\n"
                                      "Three smooths over a hitch; two cuts latency.");

                const char* const PRESENT[] = {"V-Sync", "Mailbox", "Immediate"};
                int present = static_cast<int>(delivery.present_mode);
                if (ImGui::Combo("Present Mode", &present, PRESENT, 3))
                    delivery.present_mode = static_cast<PresentMode>(present);

                ImGui::Separator();
                // The upscaler is chosen as a backend, not as a checkbox: the built-in
                // temporal reconstruction and a vendor library implement one interface, and
                // this is where the choice between them is made and what it resolved to is
                // reported.
                const char* const UPSCALERS[] = {"None", "Temporal (built-in)", "FSR 3.1",
                                                 "DLSS", "XeSS"};
                int upscaler = static_cast<int>(settings.upscale);
                if (ImGui::Combo("Upscaler", &upscaler, UPSCALERS, 5))
                    settings.upscale = static_cast<UpscaleMode>(upscaler);

                const SushiEngine::Render::Frame::UpscalerAvailability availability =
                    SushiEngine::Render::Frame::upscaler_availability(settings.upscale);
                if (availability.available)
                    ImGui::TextDisabled(
                        "Runs: %s", SushiEngine::Render::Frame::upscale_mode_name(settings.upscale));
                else
                    ImGui::TextDisabled(
                        "Runs: %s — %s",
                        SushiEngine::Render::Frame::upscale_mode_name(
                            SushiEngine::Render::Frame::resolve_upscale_mode(settings.upscale)),
                        availability.reason);
                ImGui::TreePop();
            }

            if (std::memcmp(&settings_before, &settings, sizeof(settings)) != 0)
                context.preferences_dirty = true;

            ImGui::End();
        }

        void draw_post_process_panel(EditorContext& context)
        {
            if (!context.panels.post_process)
                return;
            if (!ImGui::Begin("Post Process", &context.panels.post_process))
            {
                ImGui::End();
                return;
            }

            using SushiEngine::Render::ExposureMode;
            using SushiEngine::Render::TonemapOperator;
            SushiEngine::Render::RenderSettings& settings = context.render_settings;
            SushiEngine::Render::PostProcessSettings& post = settings.post;

            // Same exhaustive-memcmp persistence as the Rendering panel: RenderSettings is
            // trivially-copyable, so a snapshot before the widgets catches any edit below.
            const SushiEngine::Render::RenderSettings settings_before = settings;

            if (ImGui::CollapsingHeader("Exposure", ImGuiTreeNodeFlags_DefaultOpen))
            {
                ImGui::PushID("Exposure");
                const char* const MODES[] = {"Manual", "Automatic"};
                int mode = static_cast<int>(post.exposure_mode);
                if (ImGui::Combo("Mode", &mode, MODES, 2))
                    post.exposure_mode = static_cast<ExposureMode>(mode);

                if (post.exposure_mode == ExposureMode::Manual)
                {
                    ImGui::SliderFloat("Compensation (EV)", &post.exposure_compensation,
                                       -6.0f, 6.0f, "%.2f");
                    ImGui::TextDisabled("Multiplies the scene's authored exposure.");
                }
                else
                {
                    ImGui::SliderFloat("Min EV", &post.auto_exposure.min_ev, -10.0f, 8.0f, "%.1f");
                    ImGui::SliderFloat("Max EV", &post.auto_exposure.max_ev, -2.0f, 20.0f, "%.1f");
                    ImGui::SliderFloat("Compensation (EV)", &post.auto_exposure.compensation,
                                       -6.0f, 6.0f, "%.2f");
                    ImGui::SliderFloat("Adapt Up", &post.auto_exposure.speed_up, 0.1f, 8.0f, "%.2f");
                    ImGui::SliderFloat("Adapt Down", &post.auto_exposure.speed_down, 0.1f, 8.0f,
                                       "%.2f");
                    ImGui::SliderFloat("Key", &post.auto_exposure.key, 0.02f, 0.5f, "%.3f");
                }
                ImGui::PopID();
            }

            if (ImGui::CollapsingHeader("Tone Mapping", ImGuiTreeNodeFlags_DefaultOpen))
            {
                const char* const OPERATORS[] = {"AgX", "ACES", "Khronos Neutral"};
                int op = static_cast<int>(post.tonemap);
                if (ImGui::Combo("Curve", &op, OPERATORS, 3))
                    post.tonemap = static_cast<TonemapOperator>(op);
            }

            if (ImGui::CollapsingHeader("Bloom", ImGuiTreeNodeFlags_DefaultOpen))
            {
                ImGui::PushID("Bloom");
                ImGui::Checkbox("Enabled", &post.bloom.enabled);
                ImGui::SliderFloat("Intensity", &post.bloom.intensity, 0.0f, 0.5f, "%.3f");
                ImGui::SliderFloat("Threshold", &post.bloom.threshold, 0.0f, 4.0f, "%.2f");
                ImGui::SliderFloat("Knee", &post.bloom.threshold_knee, 0.0f, 1.0f, "%.2f");
                ImGui::PopID();
            }

            if (ImGui::CollapsingHeader("Color Grade"))
            {
                ImGui::PushID("Grade");
                ImGui::SliderFloat("Temperature", &post.grade.temperature, -1.0f, 1.0f, "%.2f");
                ImGui::SliderFloat("Tint", &post.grade.tint, -1.0f, 1.0f, "%.2f");
                ImGui::SliderFloat("Contrast", &post.grade.contrast, 0.5f, 2.0f, "%.2f");
                ImGui::SliderFloat("Saturation", &post.grade.saturation, 0.0f, 2.0f, "%.2f");
                ImGui::SliderFloat3("Lift", post.grade.lift, -0.5f, 0.5f, "%.3f");
                ImGui::SliderFloat3("Gamma", post.grade.gamma, 0.1f, 3.0f, "%.2f");
                ImGui::SliderFloat3("Gain", post.grade.gain, 0.0f, 3.0f, "%.2f");
                ImGui::PopID();
            }

            if (ImGui::CollapsingHeader("Depth of Field"))
            {
                ImGui::PushID("DoF");
                ImGui::Checkbox("Enabled", &post.depth_of_field.enabled);
                ImGui::SliderFloat("Focus Distance (m)", &post.depth_of_field.focus_distance,
                                   0.1f, 1000.0f, "%.2f", ImGuiSliderFlags_Logarithmic);
                ImGui::SliderFloat("Focus Range (m)", &post.depth_of_field.focus_range,
                                   0.05f, 100.0f, "%.2f", ImGuiSliderFlags_Logarithmic);
                ImGui::SliderFloat("Aperture (f)", &post.depth_of_field.aperture, 0.7f, 22.0f,
                                   "%.1f");
                ImGui::SliderFloat("Max Radius (px)", &post.depth_of_field.max_radius, 1.0f, 16.0f,
                                   "%.1f");
                ImGui::PopID();
            }

            if (ImGui::CollapsingHeader("Motion Blur"))
            {
                ImGui::PushID("MotionBlur");
                ImGui::Checkbox("Enabled", &post.motion_blur.enabled);
                ImGui::SliderFloat("Intensity", &post.motion_blur.intensity, 0.0f, 2.0f, "%.2f");
                int samples = static_cast<int>(post.motion_blur.samples);
                if (ImGui::SliderInt("Samples", &samples, 2, 32))
                    post.motion_blur.samples = static_cast<std::uint32_t>(samples);
                ImGui::PopID();
            }

            if (ImGui::CollapsingHeader("Lens"))
            {
                ImGui::PushID("Lens");
                ImGui::SliderFloat("Vignette", &post.vignette, 0.0f, 1.0f, "%.2f");
                ImGui::SliderFloat("Chromatic Aberration", &post.chromatic_aberration, 0.0f, 8.0f,
                                   "%.2f");
                ImGui::SliderFloat("Film Grain", &post.film_grain, 0.0f, 0.2f, "%.3f");
                ImGui::PopID();
            }

            if (ImGui::TreeNode("Tier resolves to"))
            {
                const SushiEngine::Render::ResolvedQuality resolved =
                    SushiEngine::Render::resolve_quality(settings);
                const SushiEngine::Render::QualityParams& knobs = resolved.params;
                ImGui::Text("Bloom: %s", knobs.bloom ? "on" : "off (tier)");
                ImGui::Text("Depth of field: %s", knobs.depth_of_field ? "permitted" : "off (tier)");
                ImGui::Text("Motion blur: %s", knobs.motion_blur ? "permitted" : "off (tier)");
                ImGui::TextDisabled("The tier permits an effect; the toggle above enables it.");
                ImGui::TreePop();
            }

            if (std::memcmp(&settings_before, &settings, sizeof(settings)) != 0)
                context.preferences_dirty = true;

            ImGui::End();
        }

        void draw_gpu_culling_panel(EditorContext& context)
        {
            if (!context.panels.gpu_culling)
                return;
            if (!ImGui::Begin("GPU Culling", &context.panels.gpu_culling))
            {
                ImGui::End();
                return;
            }

            SushiEngine::Render::RenderSettings& settings = context.render_settings;
            SushiEngine::Render::GpuCullingSettings& cull = settings.gpu_culling;

            // Same exhaustive-memcmp persistence as the Post-Process panel: RenderSettings is
            // trivially-copyable, so a snapshot before the widgets catches any edit below.
            const SushiEngine::Render::RenderSettings settings_before = settings;

            ImGui::Checkbox("Enabled", &cull.enabled);
            ImGui::TextDisabled("Take the GPU-driven path when the tier permits it.");

            if (ImGui::CollapsingHeader("Cull Tests", ImGuiTreeNodeFlags_DefaultOpen))
            {
                ImGui::PushID("CullTests");
                ImGui::Indent();
                ImGui::Checkbox("Frustum", &cull.frustum);
                ImGui::Checkbox("Occlusion", &cull.occlusion);
                ImGui::SliderFloat("Min Screen Diameter (px)", &cull.min_screen_diameter,
                                   0.0f, 16.0f, "%.1f");
                ImGui::TextDisabled("Drop instances whose projected diameter is below this.");
                ImGui::Unindent();
                ImGui::PopID();
            }

            if (ImGui::CollapsingHeader("Debug"))
            {
                ImGui::PushID("Debug");
                ImGui::Checkbox("Freeze frustum (debug)", &cull.freeze);
                ImGui::Checkbox("Show statistics", &cull.show_statistics);
                if (cull.show_statistics)
                {
                    // Settings-only panel: it has no ISceneView to read the readback counts
                    // from, so it points at where the live numbers surface instead.
                    ImGui::TextDisabled("Live counts appear in the Profiler HUD as the "
                                        "\"gpu cull\" pass.");
                }
                ImGui::PopID();
            }

            if (ImGui::TreeNode("Tier resolves to"))
            {
                const SushiEngine::Render::ResolvedQuality resolved =
                    SushiEngine::Render::resolve_quality(settings);
                const SushiEngine::Render::QualityParams& knobs = resolved.params;
                ImGui::Text("GPU-driven path: %s", knobs.gpu_driven ? "permitted" : "off (tier)");
                ImGui::TextDisabled("The Low tier keeps the classic one-draw-per-instance path.");
                ImGui::TreePop();
            }

            if (std::memcmp(&settings_before, &settings, sizeof(settings)) != 0)
                context.preferences_dirty = true;

            ImGui::End();
        }

        void draw_lighting_panel(EditorContext& context)
        {
            if (!context.panels.lighting)
                return;
            if (!ImGui::Begin("Lighting", &context.panels.lighting))
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

            // The sun and the image-based-lighting source live on the Environment; the
            // Lighting window edits the same object the Environment panel does, so a change
            // in either lands in the world through set_environment.
            SushiEngine::Render::Environment environment = world->environment();
            bool env_changed = false;
            if (ImGui::CollapsingHeader("Sun", ImGuiTreeNodeFlags_DefaultOpen))
            {
                const SushiEngine::Vector3 dir =
                    SushiEngine::normalize(environment.sun.direction);
                float elevation = std::asin(static_cast<float>(
                    dir.y < -1.0 ? -1.0 : (dir.y > 1.0 ? 1.0 : dir.y))) * 57.29578f;
                float azimuth = std::atan2(static_cast<float>(dir.z),
                                           static_cast<float>(dir.x)) * 57.29578f;
                bool sun_moved = false;
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
                    env_changed = true;
                }
                float sun_color[3] = {static_cast<float>(environment.sun.color.x),
                                      static_cast<float>(environment.sun.color.y),
                                      static_cast<float>(environment.sun.color.z)};
                if (ImGui::ColorEdit3("Color", sun_color))
                {
                    environment.sun.color =
                        SushiEngine::Vector3{sun_color[0], sun_color[1], sun_color[2]};
                    env_changed = true;
                }
                if (ImGui::SliderFloat("Intensity", &environment.sun.intensity, 0.0f, 40.0f))
                    env_changed = true;
            }

            if (ImGui::CollapsingHeader("Image-Based Lighting", ImGuiTreeNodeFlags_DefaultOpen))
            {
                if (ImGui::Checkbox("Enabled", &environment.image_based_lighting))
                    env_changed = true;
                if (ImGui::SliderFloat("IBL Intensity", &environment.ibl_intensity, 0.0f, 4.0f))
                    env_changed = true;
            }
            if (env_changed)
            {
                world->set_environment(environment);
                // Environment/Lighting are an editor setting (see Preferences::environment),
                // not scene data, so they persist through the preferences store like theme
                // or render settings rather than through Save Scene.
                context.preferences.environment = environment;
                context.preferences_dirty = true;
            }

            // The sun's cascade shadows are a render-machinery setting, so they live on the
            // render settings the Rendering panel also edits.
            if (ImGui::CollapsingHeader("Sun Shadows", ImGuiTreeNodeFlags_DefaultOpen))
            {
                SushiEngine::Render::ShadowSettings& shadows = context.render_settings.shadows;
                const SushiEngine::Render::ShadowSettings shadows_before = shadows;
                ImGui::Checkbox("Cast Sun Shadows", &shadows.enabled);
                int cascades = static_cast<int>(shadows.cascade_count);
                if (ImGui::SliderInt("Cascades", &cascades, 1, 4))
                    shadows.cascade_count = static_cast<std::uint32_t>(cascades);
                ImGui::SliderFloat("Distance", &shadows.distance, 50.0f, 2000.0f, "%.0f m");
                ImGui::Checkbox("Contact Shadows", &shadows.contact_shadows);
                if (std::memcmp(&shadows_before, &shadows, sizeof(shadows)) != 0)
                    context.preferences_dirty = true;
            }

            // The punctual-light budget the tier resolves this frame, so an author sees how
            // many of the lights below actually shade and cast.
            if (ImGui::CollapsingHeader("Punctual Budget"))
            {
                const SushiEngine::Render::ResolvedQuality resolved =
                    SushiEngine::Render::resolve_quality(context.render_settings);
                ImGui::Text("Max lights   %u", resolved.settings.lights.max_lights);
                ImGui::Text("Shadow atlas %u px, %u caster(s)",
                            resolved.settings.lights.shadow_atlas_size,
                            resolved.settings.lights.max_shadow_casters);
                if (resolved.params.stochastic_light_samples > 0)
                    ImGui::Text("Beyond the atlas: %u traced sample(s)/pixel",
                                resolved.params.stochastic_light_samples);
                else
                    ImGui::TextDisabled("Beyond the atlas: unshadowed (tier)");
            }

            // Shadows for the lights the atlas had no tile for. The atlas is a memory
            // budget; this is a sample budget, which is what lets the caster count stop
            // being a ceiling.
            if (ImGui::CollapsingHeader("Stochastic Shadows"))
            {
                SushiEngine::Render::LightEngineSettings& engine =
                    context.render_settings.lights;
                const SushiEngine::Render::LightEngineSettings engine_before = engine;
                ImGui::Checkbox("Trace Beyond The Atlas", &engine.stochastic_shadows);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Each pixel samples a few of the lights that hold no\n"
                                      "atlas tile and marches the GI distance field toward\n"
                                      "them for visibility; the temporal resolve averages\n"
                                      "the rest. Needs Probe GI on — it builds the field.");
                ImGui::SliderFloat("Ray Reach", &engine.stochastic_distance, 5.0f, 200.0f,
                                   "%.0f m");
                ImGui::SliderFloat("Penumbra", &engine.stochastic_softness, 1.0f, 32.0f,
                                   "%.1f");
                if (!environment.gi.enabled)
                    ImGui::TextDisabled("Probe GI is off, so there is no field to trace.");
                if (std::memcmp(&engine_before, &engine, sizeof(engine)) != 0)
                    context.preferences_dirty = true;
            }

            ImGui::Separator();
            if (ImGui::Button("Add Light"))
            {
                context.history.record(*world);
                select_only(context, world->create_light("Light"));
            }

            // The punctual-light list: every light-bearing entity, edited in place. This is
            // the same data the Inspector's Light component edits, gathered in one place.
            for (const EntityId id : world->entities())
            {
                if (!world->has_light(id))
                    continue;
                ImGui::PushID(static_cast<int>(id));
                const std::string label = world->name(id);
                const bool selected = !context.selected_entities.empty() &&
                                      context.selected_entities.front() == id;
                if (ImGui::CollapsingHeader(label.c_str(),
                                            selected ? ImGuiTreeNodeFlags_DefaultOpen : 0))
                {
                    if (ImGui::SmallButton("Select"))
                        select_only(context, id);

                    SushiEngine::Simulation::LightParams params = world->light_params(id);
                    bool changed = false;
                    if (ImGui::Checkbox("Spot", &params.is_spot))
                        changed = true;
                    float color[3] = {static_cast<float>(params.color.x),
                                      static_cast<float>(params.color.y),
                                      static_cast<float>(params.color.z)};
                    if (ImGui::ColorEdit3("Color", color))
                    {
                        params.color = SushiEngine::Vector3{color[0], color[1], color[2]};
                        changed = true;
                    }
                    if (ImGui::DragFloat("Intensity", &params.intensity, 0.1f, 0.0f, 1000.0f,
                                         "%.1f"))
                        changed = true;
                    if (ImGui::DragFloat("Range", &params.range, 0.1f, 0.1f, 10000.0f, "%.2f"))
                        changed = true;
                    if (params.is_spot)
                    {
                        if (ImGui::DragFloat("Inner Angle", &params.inner_degrees, 0.2f, 0.0f,
                                             89.0f, "%.1f deg"))
                            changed = true;
                        if (ImGui::DragFloat("Outer Angle", &params.outer_degrees, 0.2f, 0.0f,
                                             89.0f, "%.1f deg"))
                            changed = true;
                        if (ImGui::Checkbox("Casts Shadows", &params.casts_shadows))
                            changed = true;
                    }
                    if (changed)
                    {
                        context.history.record(*world);
                        world->set_light_params(id, params);
                    }
                }
                ImGui::PopID();
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
                float height_km = environment.atmosphere.height * 0.001f;
                if (ImGui::SliderFloat("Height", &height_km, 10.0f, 300.0f, "%.0f km"))
                {
                    environment.atmosphere.height = height_km * 1000.0f;
                    changed = true;
                }
                if (ImGui::SliderFloat("Mie Anisotropy", &environment.atmosphere.mie_anisotropy,
                                       0.0f, 0.99f))
                    changed = true;
            }

            if (ImGui::CollapsingHeader("Fog"))
            {
                if (ImGui::Checkbox("Fog Enabled", &environment.fog.enabled))
                    changed = true;
                if (ImGui::SliderFloat("Density", &environment.fog.density, 0.0f, 0.1f,
                                       "%.4f /m"))
                    changed = true;
                float fog_falloff_km = environment.fog.height_falloff * 1000.0f;
                if (ImGui::SliderFloat("Height Falloff", &fog_falloff_km, 0.0f, 5.0f,
                                       "%.3f /km"))
                {
                    environment.fog.height_falloff = fog_falloff_km * 0.001f;
                    changed = true;
                }
                float fog_color[3] = {
                    static_cast<float>(environment.fog.scattering_color.x),
                    static_cast<float>(environment.fog.scattering_color.y),
                    static_cast<float>(environment.fog.scattering_color.z)};
                if (ImGui::ColorEdit3("Fog Color", fog_color))
                {
                    environment.fog.scattering_color =
                        SushiEngine::Vector3{fog_color[0], fog_color[1], fog_color[2]};
                    changed = true;
                }
                if (ImGui::SliderFloat("Ambient Fill", &environment.fog.ambient, 0.0f, 1.0f))
                    changed = true;
                if (ImGui::SliderFloat("Sun Anisotropy", &environment.fog.phase_anisotropy,
                                       0.0f, 0.95f))
                    changed = true;

                ImGui::SeparatorText("Local Fog Volumes");
                if (environment.fog_volume_count < SushiEngine::Render::MAX_FOG_VOLUMES &&
                    ImGui::Button("Add Volume"))
                {
                    environment.fog_volumes[environment.fog_volume_count] =
                        SushiEngine::Render::FogVolume{};
                    ++environment.fog_volume_count;
                    changed = true;
                }
                for (int i = 0; i < environment.fog_volume_count; ++i)
                {
                    ImGui::PushID(i);
                    SushiEngine::Render::FogVolume& v = environment.fog_volumes[i];
                    bool open = ImGui::TreeNode("volume", "Volume %d", i);
                    ImGui::SameLine();
                    if (ImGui::SmallButton("X"))
                    {
                        for (int j = i; j + 1 < environment.fog_volume_count; ++j)
                            environment.fog_volumes[j] = environment.fog_volumes[j + 1];
                        --environment.fog_volume_count;
                        changed = true;
                        if (open)
                            ImGui::TreePop();
                        ImGui::PopID();
                        break;
                    }
                    if (open)
                    {
                        int shape = static_cast<int>(v.shape);
                        if (ImGui::Combo("Shape", &shape, "Box\0Ellipsoid\0"))
                        {
                            v.shape = static_cast<SushiEngine::Render::FogVolumeShape>(shape);
                            changed = true;
                        }
                        double center[3] = {v.center.x, v.center.y, v.center.z};
                        if (ImGui::InputScalarN("Center", ImGuiDataType_Double, center, 3))
                        {
                            v.center = SushiEngine::WorldVector3{center[0], center[1], center[2]};
                            changed = true;
                        }
                        float extent[3] = {static_cast<float>(v.extent.x),
                                           static_cast<float>(v.extent.y),
                                           static_cast<float>(v.extent.z)};
                        if (ImGui::DragFloat3("Extent", extent, 5.0f, 1.0f, 100000.0f, "%.0f m"))
                        {
                            v.extent = SushiEngine::Vector3{extent[0], extent[1], extent[2]};
                            changed = true;
                        }
                        float col[3] = {static_cast<float>(v.color.x),
                                        static_cast<float>(v.color.y),
                                        static_cast<float>(v.color.z)};
                        if (ImGui::ColorEdit3("Color", col))
                        {
                            v.color = SushiEngine::Vector3{col[0], col[1], col[2]};
                            changed = true;
                        }
                        if (ImGui::SliderFloat("Density", &v.density, 0.0f, 0.2f, "%.4f /m"))
                            changed = true;
                        if (ImGui::SliderFloat("Edge Falloff", &v.edge_falloff, 0.0f, 0.99f))
                            changed = true;
                        ImGui::TreePop();
                    }
                    ImGui::PopID();
                }
            }

            if (ImGui::CollapsingHeader("Global Illumination"))
            {
                if (ImGui::Checkbox("Probe GI Enabled", &environment.gi.enabled))
                    changed = true;
                ImGui::TextDisabled("Requires High or Ultra tier.");
                if (ImGui::SliderFloat("GI Intensity", &environment.gi.intensity, 0.0f, 4.0f))
                    changed = true;
                if (ImGui::SliderFloat("Normal Bias", &environment.gi.normal_bias, 0.0f, 2.0f,
                                       "%.2f m"))
                    changed = true;
            }

            if (ImGui::CollapsingHeader("Surface"))
            {
                float ground[3] = {static_cast<float>(environment.surface.ground_albedo.x),
                                   static_cast<float>(environment.surface.ground_albedo.y),
                                   static_cast<float>(environment.surface.ground_albedo.z)};
                if (ImGui::ColorEdit3("Ground Albedo", ground))
                {
                    environment.surface.ground_albedo =
                        SushiEngine::Vector3{ground[0], ground[1], ground[2]};
                    changed = true;
                }
                float ocean[3] = {static_cast<float>(environment.surface.ocean_color.x),
                                  static_cast<float>(environment.surface.ocean_color.y),
                                  static_cast<float>(environment.surface.ocean_color.z)};
                if (ImGui::ColorEdit3("Ocean Color", ocean))
                {
                    environment.surface.ocean_color =
                        SushiEngine::Vector3{ocean[0], ocean[1], ocean[2]};
                    changed = true;
                }
            }

            if (ImGui::CollapsingHeader("Clouds"))
            {
                if (ImGui::Checkbox("Clouds Enabled", &environment.clouds.enabled))
                    changed = true;
                ImGui::BeginDisabled(!environment.clouds.enabled);

                // Presets: one click sets the whole sky (which decks, which genera, medium
                // tuning). Everything below is optional fine-tuning, tucked into Advanced.
                ImGui::SeparatorText("Preset");
                for (int p = 0; p < SushiEngine::Render::WEATHER_PRESET_COUNT; ++p)
                {
                    SushiEngine::Render::WeatherPreset preset =
                        static_cast<SushiEngine::Render::WeatherPreset>(p);
                    if (p > 0)
                        ImGui::SameLine();
                    if (ImGui::Button(SushiEngine::Render::weather_preset_name(preset)))
                    {
                        const bool was_enabled = environment.clouds.enabled;
                        environment.clouds = SushiEngine::Render::cloud_weather_preset(preset);
                        environment.clouds.enabled = was_enabled;
                        changed = true;
                    }
                }

                if (ImGui::TreeNode("Advanced"))
                {

                // Shared medium: every deck is one physical volume, so the scattering
                // knobs, ground shadow, and weather evolution apply to the whole stack.
                ImGui::SeparatorText("Medium (all decks)");
                if (ImGui::SliderFloat("Light Absorption", &environment.clouds.light_absorption,
                                       0.0f, 2.0f))
                    changed = true;
                if (ImGui::SliderFloat("Forward Scatter", &environment.clouds.forward_scattering,
                                       0.0f, 0.99f))
                    changed = true;
                if (ImGui::SliderFloat("Powder", &environment.clouds.powder_strength, 0.0f, 1.0f))
                    changed = true;
                if (ImGui::SliderFloat("Ambient Fill", &environment.clouds.ambient_strength,
                                       0.0f, 2.0f))
                    changed = true;
                if (ImGui::SliderFloat("Ground Shadow", &environment.clouds.ground_shadow_strength,
                                       0.0f, 1.0f))
                    changed = true;
                if (ImGui::SliderFloat("Weather Scale", &environment.clouds.weather_scale,
                                       10000.0f, 200000.0f, "%.0f m"))
                    changed = true;
                if (ImGui::SliderFloat("Evolution Rate", &environment.clouds.evolution_rate,
                                       0.0f, 1.0f))
                    changed = true;

                // One deck per row: pick any of the ten WMO genera and nudge its coverage
                // and density. Each deck inherits its genus's physical altitude band and
                // morphology from the catalogue, so the sky is a few coexisting genera.
                const char* genus_items[SushiEngine::Render::CLOUD_GENUS_COUNT];
                for (int g = 0; g < SushiEngine::Render::CLOUD_GENUS_COUNT; ++g)
                    genus_items[g] = SushiEngine::Render::cloud_genus_name(
                        static_cast<SushiEngine::Render::CloudGenus>(g));

                for (int i = 0; i < SushiEngine::Render::CLOUD_MAX_DECKS; ++i)
                {
                    SushiEngine::Render::CloudDeck& deck = environment.clouds.decks[i];
                    ImGui::PushID(i);
                    char header[32];
                    std::snprintf(header, sizeof(header), "Deck %d", i + 1);
                    ImGui::SeparatorText(header);
                    if (ImGui::Checkbox("Enabled", &deck.enabled))
                        changed = true;
                    ImGui::BeginDisabled(!deck.enabled);

                    int genus = static_cast<int>(deck.genus);
                    if (ImGui::Combo("Genus", &genus, genus_items,
                                     SushiEngine::Render::CLOUD_GENUS_COUNT))
                    {
                        deck.genus = static_cast<SushiEngine::Render::CloudGenus>(genus);
                        changed = true;
                    }
                    if (ImGui::SliderFloat("Coverage Bias", &deck.coverage_bias, -1.0f, 1.0f))
                        changed = true;
                    if (ImGui::SliderFloat("Density Scale", &deck.density_scale, 0.0f, 2.0f))
                        changed = true;
                    ImGui::EndDisabled();
                    ImGui::PopID();
                }
                    ImGui::TreePop();
                }
                ImGui::EndDisabled();
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

            if (ImGui::CollapsingHeader("Night Lighting", ImGuiTreeNodeFlags_DefaultOpen))
            {
                if (ImGui::Checkbox("Dynamic Ambient", &environment.night.enabled))
                    changed = true;
                ImGui::BeginDisabled(!environment.night.enabled);
                if (ImGui::SliderFloat("Reflected Light",
                                       &environment.night.reflected_intensity, 0.0f, 4.0f))
                    changed = true;
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip(
                        "Scales every light the sky's reflecting bodies cast. 1 is "
                        "physical: a full Moon is ~3e-6 of sunlight.");
                if (ImGui::SliderFloat("Star Ambient", &environment.night.star_intensity,
                                       0.0f, 4.0f))
                    changed = true;
                ImGui::EndDisabled();
                if (!environment.night.enabled || !context.sky_astronomical_sun)
                {
                    float ambient[3] = {static_cast<float>(environment.ambient.x),
                                        static_cast<float>(environment.ambient.y),
                                        static_cast<float>(environment.ambient.z)};
                    if (ImGui::ColorEdit3("Ambient", ambient))
                    {
                        environment.ambient =
                            SushiEngine::Vector3{ambient[0], ambient[1], ambient[2]};
                        changed = true;
                    }
                }
            }

            // The solar-system sky edits the editor context, not the world's environment:
            // the ephemeris repopulates the bodies and stars from these each frame in the
            // main loop, so scrubbing the date never re-extracts the world.
            if (ImGui::CollapsingHeader("Solar System", ImGuiTreeNodeFlags_DefaultOpen))
            {
                ImGui::Checkbox("Sky Enabled", &context.sky_enabled);
                ImGui::SameLine();
                ImGui::Checkbox("Astronomical Sun", &context.sky_astronomical_sun);

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

                // Quick travel: one button per body, consumed by the main loop, which
                // teleports the camera to the body's sunlit side (Earth brings you home).
                ImGui::Separator();
                ImGui::TextDisabled("Travel");
                ImGui::PushID("Travel");
                const int per_row = 4;
                for (int body = 0; body < SushiEngine::Astro::BODY_COUNT; ++body)
                {
                    if (body % per_row != 0)
                        ImGui::SameLine();
                    const SushiEngine::Astro::BodyProperties properties =
                        SushiEngine::Astro::body_properties(
                            static_cast<SushiEngine::Astro::BodyId>(body));
                    if (ImGui::Button(properties.name, ImVec2(72.0f, 0.0f)))
                        context.sky_travel_target = body;
                }
                ImGui::PopID();
            }

            if (changed)
            {
                world->set_environment(environment);
                // Environment/Lighting are an editor setting (see Preferences::environment),
                // not scene data, so they persist through the preferences store like theme
                // or render settings rather than through Save Scene.
                context.preferences.environment = environment;
                context.preferences_dirty = true;
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

            // Per-pass GPU times from the render graph's timestamp queries. They lag
            // the displayed frame by one submit slot — the most recent measurement that
            // has actually been read back.
            ImGui::Separator();
            if (context.gpu_statistics.empty())
            {
                ImGui::TextDisabled("GPU timings unavailable");
            }
            for (const ViewportGpuStatistics& statistics : context.gpu_statistics)
            {
                float total = 0.0f;
                for (const GpuPassStatistic& pass : statistics.passes)
                    total += pass.milliseconds;
                ImGui::Text("%s GPU: %.3f ms", statistics.viewport.c_str(), total);
                for (const GpuPassStatistic& pass : statistics.passes)
                    ImGui::TextDisabled("  %-18s %6.3f", pass.pass.c_str(),
                                        pass.milliseconds);
            }

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
                    const SceneSkyState sky = capture_sky_state(context);
                    if (world != nullptr && save_scene(*world, path.string(), &sky))
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
