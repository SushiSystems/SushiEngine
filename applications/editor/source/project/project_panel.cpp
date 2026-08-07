/**************************************************************************/
/* project_panel.cpp                                                      */
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

#include "project_panel.hpp"

#include <SushiEngine/authoring/cook_bake_state.hpp>
#include <SushiEngine/model/import_settings_io.hpp>

#include "prefab_serializer.hpp"

#include "../animation/animated_mesh_preview.hpp"
#include "../scene/scene_commands.hpp"
#include "../ui/panel_widgets.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <shellapi.h>
#endif

#include <imgui.h>
#include <imgui_internal.h>
#include <misc/cpp/imgui_stdlib.h>

namespace SushiEngine
{
    namespace Editor
    {
        namespace fs = std::filesystem;

        namespace
        {
            // A rigged character asset: double-clicking one loads it into the animated
            // preview instead of handing a binary to the text editor or the shell.
            bool has_character_extension(const fs::path& path)
            {
                std::string ext = path.extension().string();
                std::transform(ext.begin(), ext.end(), ext.begin(),
                               [](unsigned char c) { return static_cast<char>(::tolower(c)); });
                return ext == ".gltf" || ext == ".glb";
            }

            // Queues a mesh for the physics cooking pipeline the moment the project panel actually
            // opens it, rather than only when an artist finds the Bake panel and presses its
            // button. `Authoring::CookBakeState::bake` is a no-op past the first call for an
            // unchanged asset (§8.1's cache), so opening the same file twice costs one cache
            // lookup, not a second cook.
            void queue_mesh_for_cooking(EditorContext& context, const fs::path& path)
            {
                if (context.cook_bake_state == nullptr)
                    return;
                context.cook_bake_state->bake(path.string());
            }

            // Routes a rigged glTF into the animated preview and opens the surfaces that
            // show the result — the Project panel's share of the character-loading flow
            // (the Animator panel's Load Character field is the other entry).
            void open_character_in_preview(EditorContext& context, const fs::path& path)
            {
                queue_mesh_for_cooking(context, path);
                if (context.animated_mesh_preview == nullptr || context.assets == nullptr)
                    return;
                if (context.animated_mesh_preview->load_gltf(path.string().c_str(),
                                                             *context.assets))
                {
                    context.panels.preview = true;
                    context.panels.animator_preview = true;
                    editor_log(context, "Loaded character '" + path.string() +
                                            "' into the preview.");
                }
                else
                {
                    editor_log(context, "Could not load '" + path.string() +
                                            "' as a rigged character.",
                               LogLevel::Warning);
                }
            }

            // §8.1's per-asset override, one field: a checkbox that decides whether this
            // asset has an opinion at all, and the value it holds when it does. Unchecked
            // shows the project default, dimmed, rather than a blank — the same "what would
            // this use" convention `pinned_int_row` (Bake panel) draws for pinned fields.
            bool optional_bool_row(const char* label, std::optional<bool>& value,
                                   bool project_default)
            {
                bool changed = false;
                bool overridden = value.has_value();
                ImGui::PushID(label);
                if (ImGui::Checkbox("##override", &overridden))
                {
                    value = overridden ? std::optional<bool>(project_default) : std::nullopt;
                    changed = true;
                }
                ImGui::SameLine();
                bool shown = value.value_or(project_default);
                ImGui::BeginDisabled(!overridden);
                if (ImGui::Checkbox(label, &shown) && overridden)
                {
                    value = shown;
                    changed = true;
                }
                ImGui::EndDisabled();
                ImGui::PopID();
                return changed;
            }

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

            // Saves `entity` and its descendants into `folder` as a prefab, and turns the
            // entity into an instance of what was just written. Converting it is what makes
            // the gesture legible: what stays selected is the thing the user edits next, and
            // a gesture that left the original a plain subtree would make "did that work?"
            // unanswerable without opening the folder.
            void write_entity_as_prefab(EditorContext& context, Simulation::IWorldEditor& world,
                                        Simulation::EntityId entity, const fs::path& folder)
            {
                const fs::path target =
                    unique_child_path(folder, world.name(entity), ".sushiprefab");
                const nlohmann::json document = Scene::capture_prefab(world, entity);
                if (document["entities"].empty())
                {
                    editor_log(context, "Nothing to save: that entity is no longer in the scene.");
                    return;
                }

                {
                    std::ofstream file(target.string());
                    if (!file || !(file << document.dump(2)))
                    {
                        editor_log(context, "Could not write '" + target.string() + "'.");
                        return;
                    }
                }

                // Recorded before the world changes, so one Ctrl+Z restores the subtree to
                // what it was. The file is deliberately not removed by that undo: it is
                // visible in the browser, and deleting a file the user can see would be an
                // edit they cannot undo back.
                context.history.record(world);
                Simulation::PrefabInstanceParameters link;
                link.path = target.string();
                link.revision = document.value("revision", std::string());
                world.set_prefab_instance(entity, link);
                editor_log(context, "Saved prefab '" + target.filename().string() + "'.");
            }

            // Truncates a display name to a tile-friendly length rather than wrapping —
            // simpler and robust across filenames of any length.
            std::string truncate_label(const std::string& name, std::size_t max_chars = 16)
            {
                if (name.size() <= max_chars)
                    return name;
                return name.substr(0, max_chars - 1) + "…";
            }

            // Shared by the zoom handler and the grid view's read-site so a corrupted or
            // hand-edited preferences file can never hand ImGui::InvisibleButton a zero or
            // negative dimension (it asserts both are nonzero).
            constexpr float MIN_TILE_SIZE = 48.0f;
            constexpr float MAX_TILE_SIZE = 160.0f;

            // Whether the platform shell verbs (Show in Explorer, open with the default
            // app) exist on this build. Off Windows the menu items offering them are
            // disabled with a reason rather than silently doing nothing — a control that
            // does nothing is a bug, not a placeholder (editor_ux_overhaul.md §2.4).
#ifdef _WIN32
            constexpr bool SHELL_INTEGRATION_AVAILABLE = true;
#else
            constexpr bool SHELL_INTEGRATION_AVAILABLE = false;
#endif

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

            // The Project browser's asset kinds, one glyph and one meaning per entry. Checked
            // in order of specificity: an extension that is also handled by has_text_extension
            // (e.g. a shader) must not be reclassified as plain Text before Code/Scene/etc. get
            // a look, so Folder/Scene/Prefab/Model/Image/Audio/Code are all decided first.
            enum class EntryCategory
            {
                Folder,
                Scene,
                Prefab,
                Model,
                Image,
                Audio,
                Code,
                Text,
                Unknown
            };

            EntryCategory entry_category(const fs::path& path, bool is_dir)
            {
                if (is_dir)
                    return EntryCategory::Folder;

                const std::string ext = to_lower(path.extension().string());
                if (ext == ".sushiscene")
                    return EntryCategory::Scene;
                if (ext == ".sushiprefab")
                    return EntryCategory::Prefab;
                if (ext == ".gltf" || ext == ".glb" || ext == ".fbx" || ext == ".obj")
                    return EntryCategory::Model;
                if (ext == ".png" || ext == ".jpg" || ext == ".jpeg" || ext == ".tga" ||
                    ext == ".bmp" || ext == ".hdr" || ext == ".exr")
                    return EntryCategory::Image;
                if (ext == ".wav" || ext == ".mp3" || ext == ".ogg" || ext == ".flac")
                    return EntryCategory::Audio;
                if (ext == ".cpp" || ext == ".cc" || ext == ".hpp" || ext == ".h" || ext == ".inl")
                    return EntryCategory::Code;
                if (has_text_extension(path))
                    return EntryCategory::Text;
                return EntryCategory::Unknown;
            }

            // One flat vector glyph per EntryCategory, in the same drawn-not-fonted spirit as
            // draw_toolbar_icon (panel_widgets.cpp): no bitmap or font asset to ship, and a
            // vector shape stays crisp at every zoom level Task 5 allows. `size` is the tile's
            // full icon box; every glyph is drawn centred inside it with a margin so it never
            // touches the box edge.
            void draw_entry_icon(ImDrawList* list, ImVec2 origin, float size,
                                 EntryCategory category, ImU32 color)
            {
                const float margin = size * 0.15f;
                const ImVec2 c{origin.x + size * 0.5f, origin.y + size * 0.5f};
                const float r = size * 0.5f - margin;
                const float thickness = std::max(1.5f, size * 0.03f);

                switch (category)
                {
                    case EntryCategory::Folder:
                    {
                        // A tabbed folder: a small tab rect atop a wider body rect.
                        const ImVec2 body_min{c.x - r, c.y - r * 0.5f};
                        const ImVec2 body_max{c.x + r, c.y + r};
                        list->AddRectFilled(ImVec2(c.x - r, c.y - r), ImVec2(c.x - r * 0.2f, c.y - r * 0.5f), color, 1.0f);
                        list->AddRectFilled(body_min, body_max, color, 2.0f);
                        break;
                    }
                    case EntryCategory::Scene:
                    {
                        // A page with a folded corner and a small play-triangle mark.
                        list->AddRect(ImVec2(c.x - r * 0.7f, c.y - r), ImVec2(c.x + r * 0.7f, c.y + r), color, 1.0f, ImDrawFlags_None, thickness);
                        list->AddTriangleFilled(ImVec2(c.x - r * 0.25f, c.y - r * 0.35f), ImVec2(c.x - r * 0.25f, c.y + r * 0.35f), ImVec2(c.x + r * 0.35f, c.y), color);
                        break;
                    }
                    case EntryCategory::Prefab:
                    {
                        // A hexagon — Unity's own prefab convention, distinct from every other glyph here.
                        ImVec2 points[6];
                        for (int i = 0; i < 6; ++i)
                        {
                            const float angle = static_cast<float>(i) / 6.0f * 6.28318530718f - 1.5707963f;
                            points[i] = ImVec2(c.x + r * std::cos(angle), c.y + r * std::sin(angle));
                        }
                        list->AddConvexPolyFilled(points, 6, color);
                        break;
                    }
                    case EntryCategory::Model:
                    {
                        // A wireframe cube: a front square, a back square offset up-right, four connectors.
                        const float o = r * 0.4f;
                        const ImVec2 f0{c.x - r * 0.7f, c.y - r * 0.4f}, f1{c.x + r * 0.2f, c.y - r * 0.4f};
                        const ImVec2 f2{c.x + r * 0.2f, c.y + r * 0.85f}, f3{c.x - r * 0.7f, c.y + r * 0.85f};
                        const ImVec2 b0{f0.x + o, f0.y - o}, b1{f1.x + o, f1.y - o};
                        const ImVec2 b2{f2.x + o, f2.y - o}, b3{f3.x + o, f3.y - o};
                        list->AddQuad(f0, f1, f2, f3, color, thickness);
                        list->AddQuad(b0, b1, b2, b3, color, thickness);
                        list->AddLine(f0, b0, color, thickness);
                        list->AddLine(f1, b1, color, thickness);
                        list->AddLine(f2, b2, color, thickness);
                        list->AddLine(f3, b3, color, thickness);
                        break;
                    }
                    case EntryCategory::Image:
                    {
                        // A picture frame with a corner sun and a mountain fold — the classic
                        // "image" glyph, and the placeholder a future real thumbnail replaces.
                        list->AddRect(ImVec2(c.x - r, c.y - r * 0.75f), ImVec2(c.x + r, c.y + r * 0.75f), color, 1.0f, ImDrawFlags_None, thickness);
                        list->AddCircleFilled(ImVec2(c.x - r * 0.5f, c.y - r * 0.3f), r * 0.18f, color);
                        list->AddTriangleFilled(ImVec2(c.x - r * 0.7f, c.y + r * 0.6f), ImVec2(c.x - r * 0.1f, c.y - r * 0.1f), ImVec2(c.x + r * 0.4f, c.y + r * 0.6f), color);
                        list->AddTriangleFilled(ImVec2(c.x + r * 0.1f, c.y + r * 0.6f), ImVec2(c.x + r * 0.5f, c.y + r * 0.1f), ImVec2(c.x + r * 0.85f, c.y + r * 0.6f), color);
                        break;
                    }
                    case EntryCategory::Audio:
                    {
                        // A speaker body plus two arcs of increasing radius: a waveform mark.
                        list->AddTriangleFilled(ImVec2(c.x - r * 0.85f, c.y - r * 0.3f), ImVec2(c.x - r * 0.85f, c.y + r * 0.3f), ImVec2(c.x - r * 0.35f, c.y + r * 0.3f), color);
                        list->AddRectFilled(ImVec2(c.x - r * 0.95f, c.y - r * 0.3f), ImVec2(c.x - r * 0.55f, c.y + r * 0.3f), color, 1.0f);
                        list->PathArcTo(ImVec2(c.x - r * 0.35f, c.y), r * 0.5f, -0.6f, 0.6f, 12);
                        list->PathStroke(color, ImDrawFlags_None, thickness);
                        list->PathArcTo(ImVec2(c.x - r * 0.35f, c.y), r * 0.85f, -0.6f, 0.6f, 12);
                        list->PathStroke(color, ImDrawFlags_None, thickness);
                        break;
                    }
                    case EntryCategory::Code:
                    {
                        // A page with a "</>" mark.
                        list->AddRect(ImVec2(c.x - r * 0.7f, c.y - r), ImVec2(c.x + r * 0.7f, c.y + r), color, 1.0f, ImDrawFlags_None, thickness);
                        list->AddLine(ImVec2(c.x - r * 0.4f, c.y - r * 0.15f), ImVec2(c.x - r * 0.6f, c.y + r * 0.15f), color, thickness);
                        list->AddLine(ImVec2(c.x - r * 0.6f, c.y + r * 0.15f), ImVec2(c.x - r * 0.4f, c.y + r * 0.4f), color, thickness);
                        list->AddLine(ImVec2(c.x + r * 0.4f, c.y - r * 0.15f), ImVec2(c.x + r * 0.6f, c.y + r * 0.15f), color, thickness);
                        list->AddLine(ImVec2(c.x + r * 0.6f, c.y + r * 0.15f), ImVec2(c.x + r * 0.4f, c.y + r * 0.4f), color, thickness);
                        break;
                    }
                    case EntryCategory::Text:
                    case EntryCategory::Unknown:
                        // A plain page; Unknown adds no decoration, Text stops here too by design.
                        list->AddRect(ImVec2(c.x - r * 0.7f, c.y - r), ImVec2(c.x + r * 0.7f, c.y + r), color, 1.0f, ImDrawFlags_None, thickness);
                        break;
                }
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

            // The behaviour behind one Project browser entry, independent of whether it was
            // drawn as a grid tile or a list row: both call this immediately after drawing
            // their own clickable widget, so ImGui's "the last item" state (IsItemHovered,
            // IsItemClicked, BeginPopupContextItem) refers to whichever widget the caller drew.
            void draw_project_entry_interactions(EditorContext& context,
                                                 const fs::directory_entry& entry, bool is_dir,
                                                 const fs::path& current, fs::path& delete_target)
            {
                const std::string path_string = entry.path().string();

                // Dragging a file out of the browser is how an asset reaches a slot that
                // wants one; directories are not draggable because nothing accepts one.
                if (!is_dir)
                    set_asset_drag_source(path_string, entry.path().filename().string());

                // The tile/row shows a truncated or plain filename, which is ambiguous the
                // moment a search returns two files of the same name from different folders —
                // so the full path relative to the browsed folder is always one hover away.
                if (ImGui::IsItemHovered())
                {
                    std::error_code relative_ec;
                    const fs::path shown = fs::relative(entry.path(), current, relative_ec);
                    ImGui::SetTooltip("%s", relative_ec ? path_string.c_str() : shown.string().c_str());
                }

                const auto open_entry = [&]()
                {
                    context.selected_project_path = path_string;
                    if (is_dir)
                        context.current_directory = path_string;
                    else if (entry.path().extension() == ".sushiscene")
                        request_open_scene(context, path_string);
                    else if (has_character_extension(entry.path()))
                        open_character_in_preview(context, entry.path());
                    else if (has_text_extension(entry.path()))
                        open_document(context, entry.path());
                    else
                        open_with_default_app(entry.path());
                };

                // Double-click detection is independent of a Button's own pressed-on-release
                // return, which can miss the second click of a fast double-click; hover +
                // IsMouseDoubleClicked is the reliable pair, and works the same for a Selectable.
                if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
                    open_entry();

                if (ImGui::BeginPopupContextItem())
                {
                    context.selected_project_path = path_string;
                    if (!is_dir && ImGui::MenuItem("Open"))
                        open_entry();
                    if (has_character_extension(entry.path()) && context.cook_bake_state != nullptr &&
                        ImGui::MenuItem("Cooking Override..."))
                        context.cooking_override_target = path_string;
                    if (ImGui::MenuItem("Rename"))
                        context.renaming_project_path = path_string;
                    if (ImGui::MenuItem("Delete"))
                        delete_target = entry.path();
                    if (ImGui::MenuItem("Show in Explorer", nullptr, false, SHELL_INTEGRATION_AVAILABLE))
                        show_in_explorer(entry.path());
                    if (!SHELL_INTEGRATION_AVAILABLE &&
                        ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
                        ImGui::SetTooltip("Windows-only for now.");
                    ImGui::Separator();
                    draw_project_create_menu(context, current);
                    ImGui::EndPopup();
                }
            }

            // The rename-in-place commit shared by both views: seed/edit/commit-on-Enter through
            // inline_rename_field, then apply the rename to disk. Layout (whether a placeholder box
            // is reserved first, and at what width the field draws) stays with each caller.
            void commit_rename_if_entered(EditorContext& context, const fs::directory_entry& entry,
                                          const std::string& path_string, const std::string& name,
                                          float width)
            {
                std::string entered;
                if (inline_rename_field(context, path_string, name, width, entered))
                {
                    std::error_code rename_ec;
                    const fs::path renamed = entry.path().parent_path() / entered;
                    if (!entered.empty() && renamed != entry.path())
                        fs::rename(entry.path(), renamed, rename_ec);
                    context.renaming_project_path.clear();
                }
            }

            // The Unity-style icon grid: one square tile per entry, laid out left-to-right and
            // wrapped at the available width, at whatever size context.preferences.project_tile_size
            // currently holds (Task 5 changes that value; this function only reads it).
            void draw_project_grid_view(EditorContext& context,
                                        const std::vector<fs::directory_entry>& entries,
                                        const fs::path& current, fs::path& delete_target)
            {
                const float tile_size =
                    std::clamp(context.preferences.project_tile_size, MIN_TILE_SIZE, MAX_TILE_SIZE);
                constexpr float TILE_SPACING = 8.0f;
                const float avail_width = ImGui::GetContentRegionAvail().x;
                float row_x = 0.0f;

                for (std::size_t i = 0; i < entries.size(); ++i)
                {
                    const fs::directory_entry& entry = entries[i];
                    const bool is_dir = entry.is_directory();
                    const std::string path_string = entry.path().string();
                    const std::string name = entry.path().filename().string();

                    if (row_x + tile_size > avail_width && row_x > 0.0f)
                        row_x = 0.0f;
                    else if (i > 0 && row_x > 0.0f)
                        ImGui::SameLine();
                    row_x += tile_size + TILE_SPACING;

                    ImGui::PushID(path_string.c_str());
                    ImGui::BeginGroup();

                    if (context.renaming_project_path == path_string)
                    {
                        ImGui::Dummy(ImVec2(tile_size, tile_size * 0.6f));
                        commit_rename_if_entered(context, entry, path_string, name, tile_size);
                    }
                    else
                    {
                        const ImVec2 origin = ImGui::GetCursorScreenPos();
                        const float line_height = ImGui::GetTextLineHeight();
                        constexpr float LABEL_SPACING = 4.0f;
                        const float icon_size =
                            std::max(tile_size * 0.5f, tile_size - line_height - LABEL_SPACING);
                        const bool clicked =
                            ImGui::InvisibleButton("##tile", ImVec2(tile_size, tile_size));
                        if (clicked)
                            context.selected_project_path = path_string;

                        ImDrawList* draw_list = ImGui::GetWindowDrawList();
                        const bool selected = context.selected_project_path == path_string;
                        if (selected || ImGui::IsItemHovered())
                            draw_list->AddRectFilled(
                                origin, ImVec2(origin.x + tile_size, origin.y + tile_size),
                                ImGui::GetColorU32(selected ? ImGuiCol_ButtonActive : ImGuiCol_ButtonHovered),
                                3.0f);
                        draw_entry_icon(draw_list, ImVec2(origin.x + (tile_size - icon_size) * 0.5f, origin.y),
                                       icon_size, entry_category(entry.path(), is_dir),
                                       ImGui::GetColorU32(ImGuiCol_Text));
                        const float glyph_width = ImGui::CalcTextSize("M").x;
                        const std::size_t max_chars = glyph_width > 0.0f
                            ? std::max<std::size_t>(3, static_cast<std::size_t>(tile_size / glyph_width))
                            : 16;
                        const std::string label = truncate_label(name, max_chars);
                        const ImVec2 text_size = ImGui::CalcTextSize(label.c_str());
                        const ImVec2 tile_max(origin.x + tile_size, origin.y + tile_size);
                        draw_list->PushClipRect(origin, tile_max, true);
                        draw_list->AddText(ImVec2(origin.x + (tile_size - text_size.x) * 0.5f, origin.y + icon_size + 2.0f),
                                          ImGui::GetColorU32(ImGuiCol_Text), label.c_str());
                        draw_list->PopClipRect();

                        draw_project_entry_interactions(context, entry, is_dir, current, delete_target);
                    }

                    ImGui::EndGroup();
                    ImGui::PopID();
                }
            }

            // Unity's List View: one compact row per entry, a small fixed-size glyph before
            // the name, unaffected by the grid's zoom (Unity's own list rows are a fixed size).
            void draw_project_list_view(EditorContext& context,
                                        const std::vector<fs::directory_entry>& entries,
                                        const fs::path& current, fs::path& delete_target)
            {
                constexpr float ROW_ICON_SIZE = 16.0f;
                for (const fs::directory_entry& entry : entries)
                {
                    const bool is_dir = entry.is_directory();
                    const std::string path_string = entry.path().string();
                    const std::string name = entry.path().filename().string();

                    ImGui::PushID(path_string.c_str());

                    if (context.renaming_project_path == path_string)
                    {
                        commit_rename_if_entered(context, entry, path_string, name, -FLT_MIN);
                    }
                    else
                    {
                        const bool selected = context.selected_project_path == path_string;
                        const ImVec2 origin = ImGui::GetCursorScreenPos();
                        // Reserve room for the icon; the label is drawn by Selectable itself,
                        // indented past the icon column.
                        ImGui::Dummy(ImVec2(ROW_ICON_SIZE + 4.0f, 0.0f));
                        ImGui::SameLine();
                        if (ImGui::Selectable(name.c_str(), selected))
                            context.selected_project_path = path_string;
                        ImDrawList* draw_list = ImGui::GetWindowDrawList();
                        const float line_height = ImGui::GetTextLineHeight();
                        const ImVec2 icon_origin = ROW_ICON_SIZE < line_height
                            ? ImVec2(origin.x, origin.y + (line_height - ROW_ICON_SIZE) * 0.5f)
                            : origin;
                        draw_entry_icon(draw_list, icon_origin, ROW_ICON_SIZE,
                                       entry_category(entry.path(), is_dir),
                                       ImGui::GetColorU32(ImGuiCol_Text));

                        draw_project_entry_interactions(context, entry, is_dir, current, delete_target);
                    }

                    ImGui::PopID();
                }
            }

        } // namespace

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
            using Authoring::ProjectBrowserViewMode;
            // Read Ctrl before BeginChild: NoScrollWithMouse must be a BeginChild argument, and
            // it is only wanted while Ctrl is held in Grid view — otherwise plain scrolling (and,
            // in List view, plain wheel scrolling under Ctrl) must still work. Folding the view-mode
            // check in here means the BeginChild flag and the zoom condition below share one gate,
            // so the two can never disagree about whether a zoom gesture is in progress.
            const bool zoom_gesture_active =
                context.preferences.project_view_mode == ProjectBrowserViewMode::Grid &&
                ImGui::GetIO().KeyCtrl;
            ImGui::BeginChild("project_grid", ImVec2(0.0f, 0.0f), true,
                              zoom_gesture_active ? ImGuiWindowFlags_NoScrollWithMouse : 0);

            // Ctrl+scroll zooms the icon grid, Unity-style. ImGuiWindowFlags_NoScrollWithMouse
            // above (applied only while the gesture is active) is what actually stops the window's
            // own wheel-scroll: ImGui applies mouse wheel to the hovered window during NewFrame(),
            // before this code runs, so zeroing io.MouseWheel here could never have prevented it.
            if (ImGui::IsWindowHovered() && zoom_gesture_active && ImGui::GetIO().MouseWheel != 0.0f)
            {
                constexpr float ZOOM_STEP = 8.0f;
                context.preferences.project_tile_size = std::clamp(
                    context.preferences.project_tile_size + ImGui::GetIO().MouseWheel * ZOOM_STEP,
                    MIN_TILE_SIZE, MAX_TILE_SIZE);
                context.preferences_dirty = true;
            }

            const bool is_grid = context.preferences.project_view_mode == ProjectBrowserViewMode::Grid;
            if (ImGui::RadioButton("Grid", is_grid))
            {
                context.preferences.project_view_mode = ProjectBrowserViewMode::Grid;
                context.preferences_dirty = true;
            }
            ImGui::SameLine();
            if (ImGui::RadioButton("List", !is_grid))
            {
                context.preferences.project_view_mode = ProjectBrowserViewMode::List;
                context.preferences_dirty = true;
            }

            ImGui::SetNextItemWidth(-FLT_MIN);
            ImGui::InputTextWithHint("##project_filter", "Search...", &context.project_filter);
            const std::string lower_filter = to_lower(context.project_filter);

            std::error_code ec;
            std::vector<fs::directory_entry> entries;
            if (lower_filter.empty())
            {
                for (const auto& entry : fs::directory_iterator(current, ec))
                    entries.push_back(entry);
            }
            else
            {
                // A search descends: an asset the user is looking for is almost never in the
                // folder they happen to be standing in, and a filter that only hid the
                // current folder's contents would be a search that could not find anything.
                // Capped, because a deep project tree walked every frame is a stutter — and
                // a result list too long to read is not a result list.
                constexpr std::size_t MAX_SEARCH_RESULTS = 400;
                fs::recursive_directory_iterator walk(
                    current, fs::directory_options::skip_permission_denied, ec);
                const fs::recursive_directory_iterator done;
                for (; walk != done && entries.size() < MAX_SEARCH_RESULTS; walk.increment(ec))
                {
                    if (ec)
                        break;
                    if (to_lower(walk->path().filename().string()).find(lower_filter) !=
                        std::string::npos)
                        entries.push_back(*walk);
                }
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

            // The delete target is deferred out of the loop so the directory listing is
            // never mutated (and filesystem-iterated again next frame) mid-walk.
            fs::path delete_target;
            if (context.preferences.project_view_mode == ProjectBrowserViewMode::Grid)
                draw_project_grid_view(context, entries, current, delete_target);
            else
                draw_project_list_view(context, entries, current, delete_target);

            // Right-click on empty grid space: create new items in the current folder.
            if (ImGui::BeginPopupContextWindow("project_grid_context", ImGuiPopupFlags_MouseButtonRight |
                                                                           ImGuiPopupFlags_NoOpenOverItems))
            {
                draw_project_create_menu(context, current);
                if (ImGui::MenuItem("Show in Explorer", nullptr, false,
                                    SHELL_INTEGRATION_AVAILABLE))
                    show_in_explorer(current);
                if (!SHELL_INTEGRATION_AVAILABLE &&
                    ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
                    ImGui::SetTooltip("Windows-only for now.");
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

            // Dragging an entity out of the Hierarchy and onto this browser saves it as a
            // prefab in the folder being browsed. The whole browser is the target rather than
            // one tile, because the gesture is "put this entity in this folder" and a target
            // the size of an icon makes the user aim at something that is not the destination.
            if (ImGui::BeginDragDropTargetCustom(ImGui::GetCurrentWindow()->InnerRect,
                                                 ImGui::GetID("project_prefab_author")))
            {
                if (const ImGuiPayload* payload =
                        ImGui::AcceptDragDropPayload("HIERARCHY_ENTITY"))
                {
                    const Simulation::EntityId dragged =
                        *static_cast<const Simulation::EntityId*>(payload->Data);
                    if (Simulation::IWorldEditor* world = world_of(context))
                        write_entity_as_prefab(context, *world, dragged, current);
                }
                ImGui::EndDragDropTarget();
            }

            ImGui::EndChild();
            ImGui::End();
        }

        void draw_cooking_override_modal(EditorContext& context)
        {
            if (context.cooking_override_target.empty())
                return;
            if (context.cook_bake_state == nullptr)
            {
                context.cooking_override_target.clear();
                return;
            }

            using Physics::Cooking::ImportProfileLibrary;
            using Physics::Cooking::ImportProfileOverride;

            // Reloaded whenever the target changes, not every frame — reading the working
            // copy back out of the library each frame would discard whatever the artist is
            // mid-edit on the moment the library's own default happens to redraw the modal.
            static std::string loaded_for;
            static ImportProfileOverride working;
            ImportProfileLibrary& profiles = context.cook_bake_state->profiles();
            if (loaded_for != context.cooking_override_target)
            {
                // Read from the asset's sidecar, which is where an override lives now; the
                // library's copy is only this session's.
                Model::ModelImportSettings settings;
                (void)Model::load_model_import_settings(context.cooking_override_target, settings);
                working = settings.cooking;
                loaded_for = context.cooking_override_target;
            }

            const char* popup_id = "Cooking Override";
            ImGui::OpenPopup(popup_id);
            if (ImGui::BeginPopupModal(popup_id, nullptr, ImGuiWindowFlags_AlwaysAutoResize))
            {
                const auto& project = profiles.project_default().parameters;
                ImGui::TextDisabled("%s", context.cooking_override_target.c_str());
                ImGui::TextDisabled("What this asset says differs from the project default.");
                ImGui::Separator();

                bool fidelity_overridden = working.fidelity.has_value();
                ImGui::PushID("fidelity");
                if (ImGui::Checkbox("##override", &fidelity_overridden))
                    working.fidelity = fidelity_overridden
                                            ? std::optional<float>(project.fidelity)
                                            : std::nullopt;
                ImGui::SameLine();
                float shown_fidelity = working.fidelity.value_or(project.fidelity);
                ImGui::BeginDisabled(!fidelity_overridden);
                ImGui::SetNextItemWidth(160.0f);
                if (ImGui::SliderFloat("Fidelity", &shown_fidelity, 0.0f, 1.0f, "%.2f") &&
                    fidelity_overridden)
                    working.fidelity = shown_fidelity;
                ImGui::EndDisabled();
                ImGui::PopID();

                optional_bool_row("Cook collision", working.cook_collision,
                                  project.cook_collision);
                optional_bool_row("Cook soft body", working.cook_soft_body,
                                  project.cook_soft_body);
                optional_bool_row("Cook node beam", working.cook_node_beam,
                                  project.cook_node_beam);
                optional_bool_row("Authored static", working.static_geometry,
                                  project.static_geometry);

                ImGui::Separator();
                const bool close = [&]
                {
                    // An override is persisted to the asset's own `.meta` sidecar, and set on
                    // the in-memory library as well so this session's cooks see it without a
                    // re-read. The project document no longer carries overrides at all, so
                    // `save_profiles` alone would leave Apply looking like it saved and losing
                    // the value on restart.
                    const auto commit = [&context, &profiles](const ImportProfileOverride& value)
                    {
                        profiles.set_override(context.cooking_override_target, value);
                        Model::ModelImportSettings settings;
                        if (!Model::load_model_import_settings(context.cooking_override_target,
                                                               settings))
                        {
                            editor_log(context,
                                       "Could not read the import settings beside '" +
                                           context.cooking_override_target +
                                           "'; the previous ones were replaced.",
                                       LogLevel::Warning);
                        }
                        settings.cooking = value;
                        if (!Model::save_model_import_settings(context.cooking_override_target,
                                                               settings))
                        {
                            editor_log(context,
                                       "Could not write the import settings beside '" +
                                           context.cooking_override_target +
                                           "'; this change is lost when the editor closes.",
                                       LogLevel::Warning);
                        }
                        context.cook_bake_state->rebake(context.cooking_override_target);
                    };

                    if (ImGui::Button("Apply"))
                    {
                        commit(working);
                        return true;
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("Clear Override"))
                    {
                        commit(ImportProfileOverride{});
                        return true;
                    }
                    ImGui::SameLine();
                    return ImGui::Button("Cancel");
                }();
                if (close)
                {
                    context.cooking_override_target.clear();
                    loaded_for.clear();
                    ImGui::CloseCurrentPopup();
                }
                ImGui::EndPopup();
            }
        }

        namespace
        {
            // The Save / Don't Save / Cancel prompt raised by closing a dirty tab. A modal
            // rather than an inline row because the choice must be made before the tab bar
            // is walked again: leaving the tab open and unanswered would let a second close
            // click overwrite which document is being asked about.
            void draw_document_close_prompt(EditorContext& context)
            {
                if (context.closing_document < 0)
                    return;
                if (context.closing_document >= static_cast<int>(context.documents.size()))
                {
                    context.closing_document = -1;
                    return;
                }

                const std::size_t index = static_cast<std::size_t>(context.closing_document);
                Document& document = context.documents[index];

                ImGui::OpenPopup("Unsaved Changes##document");
                if (!ImGui::BeginPopupModal("Unsaved Changes##document", nullptr,
                                            ImGuiWindowFlags_AlwaysAutoResize))
                    return;

                ImGui::Text("'%s' has unsaved changes.", document.display_name.c_str());
                ImGui::Separator();

                const auto close_document = [&context, index]()
                {
                    context.documents.erase(context.documents.begin() +
                                            static_cast<std::ptrdiff_t>(index));
                    if (context.active_document >= static_cast<int>(index))
                        --context.active_document;
                    context.closing_document = -1;
                };

                if (ImGui::Button("Save"))
                {
                    save_document(document);
                    editor_log(context, "Saved '" + document.display_name + "'.");
                    close_document();
                    ImGui::CloseCurrentPopup();
                }
                ImGui::SameLine();
                if (ImGui::Button("Don't Save"))
                {
                    close_document();
                    ImGui::CloseCurrentPopup();
                }
                ImGui::SameLine();
                if (ImGui::Button("Cancel"))
                {
                    context.closing_document = -1;
                    ImGui::CloseCurrentPopup();
                }
                ImGui::EndPopup();
            }
        } // namespace

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

                    if (!open && document.dirty)
                    {
                        // A dirty tab asks before it goes. ImGui's UnsavedDocument flag only
                        // draws the dot, so without this prompt closing would discard the
                        // buffer and typed work would vanish without a word.
                        context.closing_document = static_cast<int>(i);
                        ++i;
                    }
                    else if (!open)
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

            draw_document_close_prompt(context);

            ImGui::End();
        }
    } // namespace Editor
} // namespace SushiEngine
