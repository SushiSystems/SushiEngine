/**************************************************************************/
/* project_panel.cpp                                                     */
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

#include "../animation/animated_mesh_preview.hpp"
#include "../scene/scene_commands.hpp"
#include "../ui/panel_widgets.hpp"

#include <algorithm>
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

            // Truncates a display name to a tile-friendly length rather than wrapping —
            // simpler and robust across filenames of any length.
            std::string truncate_label(const std::string& name, std::size_t max_chars = 16)
            {
                if (name.size() <= max_chars)
                    return name;
                return name.substr(0, max_chars - 1) + "…";
            }

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
            ImGui::BeginChild("project_grid", ImVec2(0.0f, 0.0f), true);

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
                // current folder's contents was a search that could not find anything.
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
                    ImGui::Dummy(ImVec2(TILE_SIZE, TILE_SIZE * 0.6f));
                    std::string entered;
                    if (inline_rename_field(context, path_string, name, TILE_SIZE, entered))
                    {
                        std::error_code rename_ec;
                        const fs::path renamed = entry.path().parent_path() / entered;
                        if (!entered.empty() && renamed != entry.path())
                            fs::rename(entry.path(), renamed, rename_ec);
                        context.renaming_project_path.clear();
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
                    // Dragging a file out of the browser is how an asset reaches a slot that
                    // wants one; directories are not draggable because nothing accepts one.
                    if (!is_dir)
                        set_asset_drag_source(path_string, name);
                    // The tile shows a truncated filename, which is ambiguous the moment a
                    // search returns two files of the same name from different folders — so
                    // the full path relative to the browsed folder is always one hover away.
                    if (ImGui::IsItemHovered())
                    {
                        std::error_code relative_ec;
                        const fs::path shown =
                            fs::relative(entry.path(), current, relative_ec);
                        ImGui::SetTooltip("%s", relative_ec ? path_string.c_str()
                                                            : shown.string().c_str());
                    }
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
                        else if (has_character_extension(entry.path()))
                            open_character_in_preview(context, entry.path());
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
                            else if (has_character_extension(entry.path()))
                                open_character_in_preview(context, entry.path());
                            else if (has_text_extension(entry.path()))
                                open_document(context, entry.path());
                            else
                                open_with_default_app(entry.path());
                        }
                        if (has_character_extension(entry.path()) &&
                            context.cook_bake_state != nullptr &&
                            ImGui::MenuItem("Cooking Override..."))
                            context.cooking_override_target = path_string;
                        if (ImGui::MenuItem("Rename"))
                            context.renaming_project_path = path_string;
                        if (ImGui::MenuItem("Delete"))
                            delete_target = entry.path();
                        if (ImGui::MenuItem("Show in Explorer", nullptr, false,
                                            SHELL_INTEGRATION_AVAILABLE))
                            show_in_explorer(entry.path());
                        if (!SHELL_INTEGRATION_AVAILABLE &&
                            ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
                            ImGui::SetTooltip("Windows-only for now.");
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
                working = profiles.get_override(context.cooking_override_target);
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
                    if (ImGui::Button("Apply"))
                    {
                        profiles.set_override(context.cooking_override_target, working);
                        context.cook_bake_state->save_profiles();
                        context.cook_bake_state->rebake(context.cooking_override_target);
                        return true;
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("Clear Override"))
                    {
                        profiles.set_override(context.cooking_override_target,
                                              ImportProfileOverride{});
                        context.cook_bake_state->save_profiles();
                        context.cook_bake_state->rebake(context.cooking_override_target);
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
                        // draws the dot; closing still discarded the buffer, which is the
                        // one place in the editor where typed work vanished without a word.
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
