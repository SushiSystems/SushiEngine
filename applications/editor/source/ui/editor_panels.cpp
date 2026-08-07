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
#include "modals.hpp"
#include "panel_widgets.hpp"

#include "../project/project_panel.hpp"
#include "../project/project_picker.hpp"
#include "../core/preferences_window.hpp"
#include "../input/input_manager_window.hpp"
#include "../scene/hierarchy_panel.hpp"
#include "../scene/inspector_panel.hpp"
#include "../scene/scene_commands.hpp"
#include "../vfx/particle_panel.hpp"

#include "../audio/audio_panels.hpp"
#include "effect_serializer.hpp"
#include "scene_serializer.hpp"
#include "../animation/animated_mesh_preview.hpp"
#include "../vfx/effect_preview.hpp"

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
#include <SushiEngine/input/bindings_json.hpp>
#include <SushiEngine/input/input_manager.hpp>
#include <SushiEngine/render/quality_params.hpp>

#include "../input/editor_contexts.hpp"

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



        void draw_menu_bar(EditorContext& context)
        {
            if (!ImGui::BeginMainMenuBar())
                return;

            IWorldEditor* world = world_of(context);

            if (ImGui::BeginMenu("File"))
            {
                if (ImGui::MenuItem("New Project..."))
                {
                    context.project_picker_mode = EditorContext::ProjectPickerMode::New;
                    context.project_picker_directory =
                        fs::path(default_projects_root()).parent_path().string();
                    context.project_picker_new_folder_name.clear();
                    context.show_project_picker = true;
                }
                if (ImGui::MenuItem("Load Project..."))
                {
                    context.project_picker_mode = EditorContext::ProjectPickerMode::Load;
                    context.project_picker_directory =
                        fs::path(context.project_root).parent_path().string();
                    context.show_project_picker = true;
                }
                ImGui::Separator();
                if (menu_item_for_action(context, "New Scene", "NewScene", world != nullptr))
                    request_new_scene(context);
                if (ImGui::MenuItem("Open Scene...", nullptr, false, world != nullptr))
                {
                    context.open_scene_path.clear();
                    context.show_open_scene = true;
                }
                if (ImGui::BeginMenu("Open Recent", world != nullptr &&
                                                        !context.preferences.recent_scenes.empty()))
                {
                    // A copy: opening a clean scene runs immediately and reorders the
                    // recent list mid-iteration otherwise.
                    const std::vector<std::string> recent = context.preferences.recent_scenes;
                    for (const std::string& path : recent)
                        if (ImGui::MenuItem(path.c_str()))
                            request_open_scene(context, path);
                    ImGui::Separator();
                    if (ImGui::MenuItem("Clear Recent"))
                    {
                        context.preferences.recent_scenes.clear();
                        context.preferences_dirty = true;
                    }
                    ImGui::EndMenu();
                }
                if (menu_item_for_action(context, "Save Scene", "Save", world != nullptr))
                    (void)save_current_scene(context);
                if (ImGui::MenuItem("Save Scene As...", nullptr, false, world != nullptr))
                {
                    context.save_scene_as_name =
                        context.scene_path.empty() ? "Scene.sushiscene" : fs::path(context.scene_path).filename().string();
                    context.show_save_scene_as = true;
                }
                ImGui::Separator();
                if (ImGui::MenuItem("New Entity", nullptr, false, world != nullptr))
                {
                    select_only(context, world->create("Entity"));
                    editor_log(context, "Created entity 'Entity'.");
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
                // Undo/redo replaces the world wholesale (see Authoring::CommandHistory), so entity
                // ids from before the swap are no longer valid; drop the selection rather
                // than risk it aliasing an unrelated new entity.
                if (menu_item_for_action(context, "Undo", "Undo", context.history.can_undo()) &&
                    world != nullptr && context.history.undo(*world))
                    select_only(context, NULL_ENTITY);
                if (menu_item_for_action(context, "Redo", "Redo", context.history.can_redo()) &&
                    world != nullptr && context.history.redo(*world))
                    select_only(context, NULL_ENTITY);
                ImGui::Separator();
                draw_clipboard_menu_items(context, world);
                ImGui::Separator();
                if (ImGui::MenuItem("Input Manager...", nullptr))
                    context.panels.input_manager = true;
                if (ImGui::MenuItem("Preferences...", nullptr))
                    context.panels.preferences = true;
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
                // Domain submenus, alphabetized within each group, so 20+ windows stay
                // scannable and a new panel has one obvious home instead of a flat list
                // that grows past the screen. Every item toggles its Authoring::PanelVisibility
                // flag; a reopened window lands in its dock home (build_default_layout
                // docks all windows, open or not).
                if (ImGui::BeginMenu("General"))
                {
                    ImGui::MenuItem("Console", nullptr, &context.panels.console);
                    ImGui::MenuItem("Game", nullptr, &context.panels.game_view);
                    ImGui::MenuItem("Hierarchy", nullptr, &context.panels.hierarchy);
                    ImGui::MenuItem("Inspector", nullptr, &context.panels.inspector);
                    ImGui::MenuItem("Project", nullptr, &context.panels.project);
                    ImGui::MenuItem("Scene", nullptr, &context.panels.scene_view);
                    ImGui::Separator();
                    // Unity's maximize: the Scene view covers the whole editor viewport
                    // and returns to its dock slot when toggled back.
                    ImGui::MenuItem("Scene Fullscreen", "Shift+Space",
                                    &context.scene_view_fullscreen);
                    ImGui::EndMenu();
                }
                if (ImGui::BeginMenu("Rendering"))
                {
                    ImGui::MenuItem("GPU Culling", nullptr, &context.panels.gpu_culling);
                    ImGui::MenuItem("Lighting", nullptr, &context.panels.lighting);
                    ImGui::MenuItem("Post Process", nullptr, &context.panels.post_process);
                    ImGui::MenuItem("Rendering", nullptr, &context.panels.rendering);
                    ImGui::EndMenu();
                }
                if (ImGui::BeginMenu("World"))
                {
                    ImGui::MenuItem("Environment", nullptr, &context.panels.environment);
                    ImGui::MenuItem("Meteorology", nullptr, &context.panels.meteorology);
                    ImGui::MenuItem("Terrain", nullptr, &context.panels.terrain);
                    ImGui::EndMenu();
                }
                if (ImGui::BeginMenu("Animation"))
                {
                    ImGui::MenuItem("Animation", nullptr, &context.panels.animation);
                    ImGui::MenuItem("Animator", nullptr, &context.panels.animator_preview);
                    ImGui::MenuItem("Animator Graph", nullptr, &context.panels.animator_graph);
                    ImGui::MenuItem("Preview", nullptr, &context.panels.preview);
                    ImGui::EndMenu();
                }
                if (ImGui::BeginMenu("Audio"))
                {
                    ImGui::MenuItem("Audio Authoring", nullptr, &context.panels.audio_authoring);
                    ImGui::MenuItem("Audio Mixer", nullptr, &context.panels.audio_mixer);
                    ImGui::MenuItem("Audio Profiler", nullptr, &context.panels.audio_profiler);
                    ImGui::EndMenu();
                }
                if (ImGui::BeginMenu("Analysis"))
                {
                    ImGui::MenuItem("Physics", nullptr, &context.panels.physics);
                    ImGui::MenuItem("Vehicle", nullptr, &context.panels.vehicle);
                    ImGui::MenuItem("Assembly", nullptr, &context.panels.assembly);
                    ImGui::MenuItem("Bake", nullptr, &context.panels.bake);
                    ImGui::MenuItem("Statistics", nullptr, &context.panels.statistics);
                    ImGui::MenuItem("Profiler", nullptr, &context.panels.profiler);
                    ImGui::MenuItem("Text Editor", nullptr, &context.panels.text_editor);
                    ImGui::Separator();
                    // §9.3/§9.4's debug views for the selected soft body. Radio rather than
                    // checkboxes: the three draw over the same edges, so any two at once
                    // would be one hiding the other.
                    if (ImGui::BeginMenu("Soft Body View"))
                    {
                        const auto item = [&context](const char* label,
                                                     Authoring::SoftBodyDebugView view)
                        {
                            if (ImGui::MenuItem(label, nullptr,
                                                context.soft_body_debug_view == view))
                                context.soft_body_debug_view = view;
                        };
                        item("Off", Authoring::SoftBodyDebugView::Off);
                        item("Tetrahedra", Authoring::SoftBodyDebugView::Wireframe);
                        item("Stress", Authoring::SoftBodyDebugView::Stress);
                        item("Plastic Strain", Authoring::SoftBodyDebugView::PlasticStrain);
                        ImGui::EndMenu();
                    }
                    ImGui::EndMenu();
                }
                ImGui::Separator();
                if (ImGui::MenuItem("Reset Layout"))
                    context.layout_reset_requested = true;
                ImGui::MenuItem("ImGui Demo", nullptr, &context.show_imgui_demo);
                ImGui::EndMenu();
            }

            ImGui::EndMainMenuBar();

            // File ▸ Open Scene…: a path prompt rooted at the project, routed through the
            // same unsaved-changes gate every other scene replacement takes.
            if (context.show_open_scene)
            {
                ImGui::OpenPopup("Open Scene");
                if (ImGui::BeginPopupModal("Open Scene", &context.show_open_scene,
                                           ImGuiWindowFlags_AlwaysAutoResize))
                {
                    ImGui::TextDisabled("%s", context.project_root.c_str());
                    ImGui::SetNextItemWidth(420.0f);
                    const bool commit =
                        ImGui::InputText("##open_scene_path", &context.open_scene_path,
                                         ImGuiInputTextFlags_EnterReturnsTrue);
                    const bool confirmed = ImGui::Button("Open") || commit;
                    ImGui::SameLine();
                    if (ImGui::Button("Cancel"))
                        context.show_open_scene = false;
                    if (confirmed && !context.open_scene_path.empty())
                    {
                        fs::path path = context.open_scene_path;
                        if (path.is_relative())
                            path = fs::path(context.project_root) / path;
                        if (path.extension().empty())
                            path += ".sushiscene";
                        if (fs::exists(path))
                        {
                            request_open_scene(context, path.string());
                            context.show_open_scene = false;
                        }
                        else
                        {
                            editor_log(context, "No scene at '" + path.string() + "'.",
                                       LogLevel::Warning);
                        }
                    }
                    ImGui::EndPopup();
                }
            }
        }




        namespace
        {
            // Play/Stop and Pause/Resume as functions rather than button handlers, so the
            // toolbar buttons and the Ctrl+P / Ctrl+Shift+P shortcuts drive the exact same
            // transitions — the scene snapshot taken on Play and restored verbatim on Stop
            // (Unity's edit-mode-is-never-mutated-by-play-mode guarantee) included.
            void toggle_play_stop(EditorContext& context)
            {
                if (context.play_state == PlayState::Stopped)
                {
                    context.play_state = PlayState::Playing;
                    if (context.simulation != nullptr)
                        context.play_mode_snapshot = Scene::capture_scene(
                            context.simulation->world(), &context.play_mode_blobs);
                    editor_log(context, "Playback started.");
                }
                else
                {
                    context.play_state = PlayState::Stopped;
                    if (context.simulation != nullptr && context.play_mode_snapshot.has_value())
                    {
                        Scene::apply_scene(context.simulation->world(),
                                           *context.play_mode_snapshot,
                                           &context.play_mode_blobs);
                        context.play_mode_snapshot.reset();
                        select_only(context, SushiEngine::Simulation::NULL_ENTITY);
                    }
                    editor_log(context, "Playback stopped; scene restored.");
                }
            }

            void toggle_pause(EditorContext& context)
            {
                if (context.play_state == PlayState::Stopped)
                    return;
                context.play_state = context.play_state == PlayState::Paused
                                         ? PlayState::Playing
                                         : PlayState::Paused;
                editor_log(context, context.play_state == PlayState::Paused
                                        ? "Playback paused."
                                        : "Playback resumed.");
            }
        } // namespace

        void draw_toolbar(EditorContext& context)
        {
            // The playback shortcuts (Unity's Ctrl+P / Ctrl+Shift+P) are resolved here so
            // one place owns every playback entry point. Ctrl+Shift+P also satisfies the
            // plain Ctrl+P chord on the same press, so the more specific pause action is
            // tested first and the else-if keeps the pair mutually exclusive.
            if (context.input_snapshot != nullptr)
            {
                if (context.input_snapshot->pressed("PauseToggle"))
                    toggle_pause(context);
                else if (context.input_snapshot->pressed("PlayToggle"))
                    toggle_play_stop(context);
            }

            // A fixed strip under the menu bar (like the status bar), not a dockable
            // window: the Play button and tool selector are chrome, and a closable
            // Toolbar would mean closing it loses the Play button. The side-bar height
            // reserves its space before the dockspace claims the rest.
            ImGuiViewport* viewport = ImGui::GetMainViewport();
            const float height =
                ImGui::GetFrameHeight() + ImGui::GetStyle().WindowPadding.y * 2.0f;
            if (ImGui::BeginViewportSideBar("##Toolbar", viewport, ImGuiDir_Up, height,
                                            ImGuiWindowFlags_NoScrollbar |
                                                ImGuiWindowFlags_NoSavedSettings))
            {
                // The tooltips are where the icons' names live, and they quote the live
                // binding rather than a hard-coded chord, so a rebind is reflected here too.
                const auto with_shortcut = [&context](const char* text, const char* action)
                {
                    const std::string shortcut = shortcut_for_action(context, action);
                    return shortcut.empty() ? std::string(text)
                                            : std::string(text) + " (" + shortcut + ")";
                };

                const bool in_play_session = context.play_state != PlayState::Stopped;
                if (icon_button("play", in_play_session ? ToolbarIcon::Stop : ToolbarIcon::Play,
                                false,
                                with_shortcut(in_play_session ? "Stop: restore the scene as it "
                                                               "was when Play began"
                                                             : "Play",
                                              "PlayToggle")
                                    .c_str()))
                    toggle_play_stop(context);
                ImGui::SameLine();

                ImGui::BeginDisabled(!in_play_session);
                if (icon_button("pause", ToolbarIcon::Pause,
                                context.play_state == PlayState::Paused,
                                with_shortcut(context.play_state == PlayState::Paused ? "Resume"
                                                                                     : "Pause",
                                              "PauseToggle")
                                    .c_str()))
                    toggle_pause(context);
                ImGui::SameLine();
                if (icon_button("step", ToolbarIcon::Step, false,
                                "Advance the simulation exactly one tick"))
                {
                    context.step_requested = true;
                    editor_log(context, "Stepped one frame.");
                }
                ImGui::EndDisabled();

                ImGui::SameLine();
                const char* state_text = context.play_state == PlayState::Playing ? "Playing"
                                         : context.play_state == PlayState::Paused ? "Paused"
                                                                                   : "Stopped";
                ImGui::AlignTextToFramePadding();
                ImGui::TextDisabled("| %s", state_text);

                // Transform tool selector, mirroring Unity's W/E/R. The hotkeys apply only
                // when no text field is capturing keys, so typing a name never switches tools.
                ImGui::SameLine();
                ImGui::AlignTextToFramePadding();
                ImGui::TextDisabled("|");
                const Authoring::GizmoMode modes[3] = {Authoring::GizmoMode::Translate,
                                                       Authoring::GizmoMode::Rotate,
                                                       Authoring::GizmoMode::Scale};
                const ToolbarIcon mode_icons[3] = {ToolbarIcon::Move, ToolbarIcon::Rotate,
                                                   ToolbarIcon::Scale};
                const char* mode_ids[3] = {"tool_move", "tool_rotate", "tool_scale"};
                const char* mode_names[3] = {"Move", "Rotate", "Scale"};
                const char* mode_actions[3] = {"GizmoTranslate", "GizmoRotate", "GizmoScale"};
                for (int i = 0; i < 3; ++i)
                {
                    ImGui::SameLine();
                    if (icon_button(mode_ids[i], mode_icons[i], context.gizmo_mode == modes[i],
                                    with_shortcut(mode_names[i], mode_actions[i]).c_str()))
                        context.gizmo_mode = modes[i];
                }

                // Local/World axis-frame toggle, mirroring Unity's gizmo-space button. Disabled
                // for Scale, which always drags in local axes (a world-aligned scale on a
                // rotated object would shear it).
                ImGui::SameLine();
                ImGui::AlignTextToFramePadding();
                ImGui::TextDisabled("|");
                ImGui::SameLine();
                ImGui::BeginDisabled(context.gizmo_mode == Authoring::GizmoMode::Scale);
                const char* space_label =
                    context.gizmo_space == Authoring::GizmoSpace::Local ? "Local" : "World";
                if (ImGui::Button(space_label))
                    context.gizmo_space = context.gizmo_space == Authoring::GizmoSpace::Local
                                              ? Authoring::GizmoSpace::World
                                              : Authoring::GizmoSpace::Local;
                ImGui::EndDisabled();

                // Overall Quality: a *derived* preset over the per-domain tiers, never
                // stored anywhere. It reads the common tier when the domains agree and
                // "Custom" when they diverge; picking a tier writes every domain's tier
                // in one gesture. The domains stay the authorities — Render Quality in
                // Rendering, Atmosphere Quality in Meteorology — so this can never
                // reintroduce the cross-domain coupling it replaces.
                ImGui::SameLine();
                ImGui::AlignTextToFramePadding();
                ImGui::TextDisabled("|");
                ImGui::SameLine();
                {
                    using SushiEngine::Render::RenderQuality;
                    using SushiEngine::Simulation::AtmosphereQuality;
                    const char* const TIERS[] = {"Low", "Medium", "High", "Ultra"};
                    const int render_tier = static_cast<int>(context.render_settings.quality);
                    const int atmosphere_tier =
                        static_cast<int>(context.simulation_settings.atmosphere.quality);
                    const bool agree = render_tier == atmosphere_tier;
                    const char* label = agree ? TIERS[render_tier] : "Custom";
                    ImGui::SetNextItemWidth(110.0f);
                    if (ImGui::BeginCombo("Quality", label))
                    {
                        for (int i = 0; i < 4; ++i)
                        {
                            const bool selected = agree && i == render_tier;
                            if (ImGui::Selectable(TIERS[i], selected))
                            {
                                context.render_settings.quality =
                                    static_cast<RenderQuality>(i);
                                context.simulation_settings.atmosphere.quality =
                                    static_cast<AtmosphereQuality>(i);
                                context.preferences_dirty = true;
                            }
                            if (selected)
                                ImGui::SetItemDefaultFocus();
                        }
                        ImGui::EndCombo();
                    }
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip(
                            "Sets every domain's quality tier at once (rendering and the\n"
                            "atmosphere simulation). Shows Custom when the domains are set\n"
                            "differently in their own panels. Note the atmosphere tier\n"
                            "rebuilds the nest: the running weather restarts.");
                }

                // Tool hotkeys now resolve through the EditorViewport input context (rebindable),
                // gated centrally by the mapper's capture gate so a text field never switches tools.
                // While right mouse is held the Scene camera owns WASD for flight, so the tool
                // hotkeys still stand down to avoid switching tools as the user moves.
                if (context.input_snapshot != nullptr &&
                    !ImGui::IsMouseDown(ImGuiMouseButton_Right))
                {
                    if (context.input_snapshot->pressed("GizmoTranslate"))
                        context.gizmo_mode = Authoring::GizmoMode::Translate;
                    else if (context.input_snapshot->pressed("GizmoRotate"))
                        context.gizmo_mode = Authoring::GizmoMode::Rotate;
                    else if (context.input_snapshot->pressed("GizmoScale"))
                        context.gizmo_mode = Authoring::GizmoMode::Scale;
                }
            }
            ImGui::End();
        }


        namespace
        {
            /** @brief The colour a severity is drawn in, and the prefix that names it. */
            struct LevelStyle
            {
                ImVec4 color;
                const char* tag;
            };

            LevelStyle style_of(LogLevel level)
            {
                switch (level)
                {
                    case LogLevel::Warning:
                        return {ImVec4(1.0f, 0.78f, 0.35f, 1.0f), "[warn] "};
                    case LogLevel::Error:
                        return {ImVec4(1.0f, 0.45f, 0.42f, 1.0f), "[error] "};
                    case LogLevel::Info:
                        break;
                }
                return {ImGui::GetStyleColorVec4(ImGuiCol_Text), ""};
            }

            bool level_shown(const Console& console, LogLevel level)
            {
                switch (level)
                {
                    case LogLevel::Warning:
                        return console.show_warnings;
                    case LogLevel::Error:
                        return console.show_errors;
                    case LogLevel::Info:
                        break;
                }
                return console.show_info;
            }
        } // namespace

        void draw_console_panel(EditorContext& context)
        {
            if (!context.panels.console)
                return;
            if (!ImGui::Begin("Console", &context.panels.console))
            {
                ImGui::End();
                return;
            }

            Console& console = context.console;

            if (ImGui::Button("Clear"))
                console.lines.clear();
            ImGui::SameLine();
            // The per-level toggles double as the counts, the way Unity's console does it:
            // one glance says both how many errors there are and whether they are showing.
            ImGui::Checkbox("Info", &console.show_info);
            ImGui::SameLine();
            ImGui::TextDisabled("%zu", console.count_of(LogLevel::Info));
            ImGui::SameLine();
            ImGui::Checkbox("Warnings", &console.show_warnings);
            ImGui::SameLine();
            ImGui::TextColored(style_of(LogLevel::Warning).color, "%zu",
                               console.count_of(LogLevel::Warning));
            ImGui::SameLine();
            ImGui::Checkbox("Errors", &console.show_errors);
            ImGui::SameLine();
            ImGui::TextColored(style_of(LogLevel::Error).color, "%zu",
                               console.count_of(LogLevel::Error));
            ImGui::SameLine();
            ImGui::Checkbox("Collapse", &console.collapse);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Folds a run of identical consecutive lines into one with\n"
                                  "a count. Every line is still recorded — this only changes\n"
                                  "what is shown, so turning it off restores the full run.");
            ImGui::SameLine();
            ImGui::Checkbox("Time", &console.show_timestamps);
            ImGui::SameLine();
            ImGui::SetNextItemWidth(-FLT_MIN);
            ImGui::InputTextWithHint("##console_filter", "Filter...", &console.filter);
            ImGui::Separator();

            const std::string lower_filter = to_lower(console.filter);

            if (ImGui::BeginChild("scroll", ImVec2(0.0f, 0.0f), false,
                                  ImGuiWindowFlags_HorizontalScrollbar))
            {
                for (std::size_t i = 0; i < console.lines.size();)
                {
                    const ConsoleLine& line = console.lines[i];

                    // Runs are folded before filtering so the count reports how many times the
                    // message actually occurred, not how many survived the filter.
                    std::size_t run = 1;
                    if (console.collapse)
                    {
                        while (i + run < console.lines.size() &&
                               console.lines[i + run].level == line.level &&
                               console.lines[i + run].text == line.text)
                            ++run;
                    }
                    i += run;

                    if (!level_shown(console, line.level))
                        continue;
                    if (!lower_filter.empty() &&
                        to_lower(line.text).find(lower_filter) == std::string::npos)
                        continue;

                    const LevelStyle style = style_of(line.level);
                    std::string text;
                    if (console.show_timestamps)
                    {
                        char stamp[32];
                        std::snprintf(stamp, sizeof(stamp), "[%7.2f] ", line.time_seconds);
                        text += stamp;
                    }
                    text += style.tag;
                    text += line.text;
                    if (run > 1)
                    {
                        char repeats[24];
                        std::snprintf(repeats, sizeof(repeats), "  (x%zu)", run);
                        text += repeats;
                    }
                    ImGui::TextColored(style.color, "%s", text.c_str());
                }
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

            if (context.frame_profile.frame_milliseconds > 0.0f)
            {
                ImGui::Text("Frame: %.2f ms", context.frame_profile.frame_milliseconds);
                ImGui::Text("FPS:   %.0f", 1000.0f / context.frame_profile.frame_milliseconds);
            }
            else
            {
                ImGui::TextDisabled("Frame: n/a — first frame pending");
            }
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
            for (const ViewportGPUStatistics& statistics : context.gpu_statistics)
            {
                float total = 0.0f;
                for (const GPUPassStatistic& pass : statistics.passes)
                    total += pass.milliseconds;
                ImGui::Text("%s GPU: %.3f ms", statistics.viewport.c_str(), total);
                for (const GPUPassStatistic& pass : statistics.passes)
                    ImGui::TextDisabled("  %-18s %6.3f", pass.pass.c_str(),
                                        pass.milliseconds);
            }

            // The animation preview's pose-pool/palette/clip footprint (design
            // `slop/animation_system.md` §12.1/§0.7) — the numbers `animation_benchmark`
            // already computes headlessly for a crowd, shown here for the one live instance.
            ImGui::Separator();
            if (ImGui::TreeNodeEx("Animation", ImGuiTreeNodeFlags_DefaultOpen))
            {
                if (context.animated_mesh_preview == nullptr)
                {
                    ImGui::TextDisabled("No preview attached");
                }
                else
                {
                    const SushiEngine::Editor::AnimatedMeshPreview::Statistics stats =
                        context.animated_mesh_preview->statistics();
                    if (!stats.loaded)
                    {
                        ImGui::TextDisabled("No character loaded");
                    }
                    else
                    {
                        ImGui::Text("Source:  %s", stats.source_path.c_str());
                        ImGui::Text("Joints:  %u", stats.joint_count);
                        ImGui::Text("Layers:  %u", stats.layer_count);
                        ImGui::Text("Palette: %zu bytes/frame (x2 with previous-pose)",
                                   stats.palette_bytes);
                        ImGui::Text("Clip:    %s, %u frames @ %.1f fps",
                                   stats.clip_compressed ? "compressed" : "raw",
                                   stats.clip_frame_count, stats.clip_sample_rate);
                        ImGui::Text("Clip tracks: %zu bytes (uncompressed)",
                                   stats.clip_raw_track_bytes);
                        if (stats.active_morph_weights > 0)
                            ImGui::Text("Morph weights: %u active, %u clip track(s), %s",
                                       stats.active_morph_weights, stats.clip_morph_track_count,
                                       stats.clip_driven_morphs && stats.clip_morph_track_count > 0
                                           ? "clip-driven"
                                           : "manual");
                        ImGui::Text("Two-bone IK: %s", stats.ik_active ? "active" : "off");
                        ImGui::Text("Skinning: %s", stats.dual_quaternion_skinning
                                                        ? "dual-quaternion"
                                                        : "linear-blend");
                    }
                }
                ImGui::TreePop();
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
                    // With a multi-selection the primary's name alone understates what an
                    // edit will touch, which is precisely the number a user wants confirmed
                    // before dragging a field.
                    if (has_selection && context.selected_entities.size() > 1)
                        ImGui::Text("Selected: %s +%zu",
                                    world->name(context.selected_entity).c_str(),
                                    context.selected_entities.size() - 1);
                    else
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
                    ImGui::Separator();

                    // Frame cost belongs on the status bar rather than only in Statistics:
                    // it is the one number that says whether what you just authored is
                    // affordable, and it should not need a panel opened to see.
                    const float frame_milliseconds = context.frame_profile.frame_milliseconds;
                    ImGui::Text("%.1f fps / %.2f ms",
                                frame_milliseconds > 0.0f ? 1000.0f / frame_milliseconds : 0.0f,
                                frame_milliseconds);

                    // The problem tally, right-aligned and clickable: an error logged while
                    // the Console is behind another tab is otherwise invisible until
                    // something else goes wrong.
                    const std::size_t errors = context.console.count_of(LogLevel::Error);
                    const std::size_t warnings = context.console.count_of(LogLevel::Warning);
                    if (errors > 0 || warnings > 0)
                    {
                        ImGui::Separator();
                        ImGui::PushStyleColor(ImGuiCol_Text,
                                              errors > 0 ? ImVec4(1.0f, 0.45f, 0.42f, 1.0f)
                                                         : ImVec4(1.0f, 0.78f, 0.35f, 1.0f));
                        char tally[64];
                        std::snprintf(tally, sizeof(tally), "%zu errors, %zu warnings", errors,
                                      warnings);
                        if (ImGui::MenuItem(tally))
                        {
                            context.panels.console = true;
                            ImGui::SetWindowFocus("Console");
                        }
                        ImGui::PopStyleColor();
                        if (ImGui::IsItemHovered())
                            ImGui::SetTooltip("Open the Console.");
                    }
                    ImGui::EndMenuBar();
                }
            }
            ImGui::End();
        }

        namespace
        {
            /**
             * @brief The editor's accent hue: "this one, and it is yours".
             *
             * One colour carries every affirmative state — the checked box, the slider's
             * grab, the selected tab, the focused nav rectangle, the drop target. A shell
             * that spends three hues on those reads as three unrelated languages, and the
             * console already owns amber and red for warnings and errors, so the accent has
             * to stay clear of both: a green with enough blue in it to sit beside them
             * without competing.
             */
            constexpr ImVec4 ACCENT{0.29f, 0.64f, 0.49f, 1.00f};

            /** @brief @p color scaled towards black (@p factor < 1) or white (> 1). */
            ImVec4 shade(const ImVec4& color, float factor)
            {
                const auto mix = [factor](float channel)
                {
                    const float value =
                        factor <= 1.0f ? channel * factor
                                       : channel + (1.0f - channel) * (factor - 1.0f);
                    return value < 0.0f ? 0.0f : (value > 1.0f ? 1.0f : value);
                };
                return ImVec4(mix(color.x), mix(color.y), mix(color.z), color.w);
            }

            /** @brief @p color at a different opacity. */
            ImVec4 fade(const ImVec4& color, float alpha)
            {
                return ImVec4(color.x, color.y, color.z, alpha);
            }

            /**
             * @brief The one geometry scale the whole shell is measured in.
             *
             * Every padding and gap is a multiple of four pixels and every rounded corner is
             * one of two radii, so panels written by different hands still line up. The
             * numbers themselves matter less than there being only a few of them.
             *
             * @param style The style to write, already carrying a base palette.
             */
            void apply_metrics(ImGuiStyle& style)
            {
                style.WindowPadding = ImVec2(8.0f, 8.0f);
                style.FramePadding = ImVec2(8.0f, 4.0f);
                style.ItemSpacing = ImVec2(8.0f, 6.0f);
                style.ItemInnerSpacing = ImVec2(6.0f, 4.0f);
                style.CellPadding = ImVec2(6.0f, 3.0f);
                style.IndentSpacing = 18.0f;
                style.ScrollbarSize = 12.0f;
                style.GrabMinSize = 10.0f;

                // Docked panels are rectangular: a rounded corner inside a dock node draws a
                // notch of the node's background against its neighbour. Everything that
                // floats above the layout — popups, tooltips, the frames of widgets — rounds.
                style.WindowRounding = 0.0f;
                style.ChildRounding = 3.0f;
                style.FrameRounding = 3.0f;
                style.PopupRounding = 4.0f;
                style.ScrollbarRounding = 3.0f;
                style.GrabRounding = 3.0f;
                style.TabRounding = 3.0f;

                style.WindowBorderSize = 1.0f;
                style.ChildBorderSize = 1.0f;
                style.PopupBorderSize = 1.0f;
                style.FrameBorderSize = 0.0f;
                style.TabBarBorderSize = 2.0f;
                style.SeparatorTextBorderSize = 2.0f;

                style.WindowTitleAlign = ImVec2(0.0f, 0.5f);
                // The title-bar collapse arrow is dead weight on a docked editor — a panel is
                // hidden by closing its tab, not by collapsing it in place — and it steals the
                // first characters of every tab's name.
                style.WindowMenuButtonPosition = ImGuiDir_None;
                style.ColorButtonPosition = ImGuiDir_Right;

                // ImGui's 0.6 leaves a disabled control close enough to an enabled one that
                // the difference reads as a rendering artifact rather than a state.
                style.DisabledAlpha = 0.45f;
            }

            /**
             * @brief Puts the accent on every affirmative state and separates the interaction steps.
             *
             * Applied over one of ImGui's base palettes, which is what keeps this to the
             * entries that matter: the base fills in the eighty-odd colours nobody looks at,
             * and this decides the ones a user reads several times a second — is this control
             * under my cursor, is it being pressed, is it off.
             *
             * @param style The style to write.
             * @param light Whether the base palette is the light one, whose hover and active
             *              steps must go *darker* rather than lighter.
             */
            void apply_accent(ImGuiStyle& style, bool light)
            {
                ImVec4* colors = style.Colors;
                const float hover = light ? 0.88f : 1.28f;
                const float press = light ? 0.76f : 1.46f;

                // Three distinct steps per interactive surface. Deriving them from the base
                // rather than spelling nine colours means a theme swap keeps the separation.
                colors[ImGuiCol_FrameBgHovered] = shade(colors[ImGuiCol_FrameBg], hover);
                colors[ImGuiCol_FrameBgActive] = shade(colors[ImGuiCol_FrameBg], press);
                colors[ImGuiCol_ButtonHovered] = shade(colors[ImGuiCol_Button], hover);
                colors[ImGuiCol_ButtonActive] = shade(colors[ImGuiCol_Button], press);
                colors[ImGuiCol_HeaderHovered] = shade(colors[ImGuiCol_Header], hover);
                colors[ImGuiCol_HeaderActive] = shade(colors[ImGuiCol_Header], press);

                // Affirmative states, all one hue.
                colors[ImGuiCol_CheckMark] = ACCENT;
                colors[ImGuiCol_SliderGrab] = ACCENT;
                colors[ImGuiCol_SliderGrabActive] = shade(ACCENT, 1.22f);
                colors[ImGuiCol_SeparatorHovered] = fade(ACCENT, 0.78f);
                colors[ImGuiCol_SeparatorActive] = ACCENT;
                colors[ImGuiCol_ResizeGripHovered] = fade(ACCENT, 0.60f);
                colors[ImGuiCol_ResizeGripActive] = fade(ACCENT, 0.90f);
                colors[ImGuiCol_TextSelectedBg] = fade(ACCENT, 0.35f);
                colors[ImGuiCol_NavCursor] = ACCENT;
                colors[ImGuiCol_DragDropTarget] = ACCENT;
                colors[ImGuiCol_TextLink] = shade(ACCENT, 1.2f);
                colors[ImGuiCol_DockingPreview] = fade(ACCENT, 0.45f);

                // Which tab is the front one is the single most-read piece of state in a
                // docked editor, so it gets the accent as an overline rather than a tinted
                // background — a tint competes with the panel content behind it.
                colors[ImGuiCol_TabSelectedOverline] = ACCENT;
                colors[ImGuiCol_TabDimmedSelectedOverline] = fade(ACCENT, 0.45f);
                colors[ImGuiCol_TabHovered] = shade(colors[ImGuiCol_Header], hover);

                // Alternating table rows carry the atmosphere diagnostics and the input
                // bindings; the default is invisible against the frame background.
                colors[ImGuiCol_TableRowBgAlt] = fade(light ? ImVec4(0.0f, 0.0f, 0.0f, 1.0f)
                                                           : ImVec4(1.0f, 1.0f, 1.0f, 1.0f),
                                                      0.035f);
            }
        } // namespace

        void apply_theme(Authoring::EditorTheme theme)
        {
            // The base palette first, then the shell's own geometry and accent over it: the
            // theme choice stays a choice about lightness, and the parts of the look that are
            // the editor's identity rather than the user's preference do not fork three ways.
            switch (theme)
            {
                case Authoring::EditorTheme::Light:   ImGui::StyleColorsLight(); break;
                case Authoring::EditorTheme::Classic: ImGui::StyleColorsClassic(); break;
                case Authoring::EditorTheme::Dark:    ImGui::StyleColorsDark(); break;
            }
            ImGuiStyle& style = ImGui::GetStyle();
            apply_metrics(style);
            apply_accent(style, theme == Authoring::EditorTheme::Light);
        }




        void build_default_layout(std::uint32_t dockspace_id)
        {
            ImGui::DockBuilderRemoveNode(dockspace_id);
            ImGui::DockBuilderAddNode(dockspace_id, ImGuiDockNodeFlags_DockSpace);
            ImGui::DockBuilderSetNodeSize(dockspace_id,
                                          ImGui::GetMainViewport()->WorkSize);

            // No top split for the Toolbar: it is a fixed viewport side bar (see
            // draw_toolbar), so the dockspace starts below it and nothing here has to
            // guess at its height.
            ImGuiID center = dockspace_id;
            ImGuiID left = ImGui::DockBuilderSplitNode(center, ImGuiDir_Left, 0.20f,
                                                       nullptr, &center);
            ImGuiID right = ImGui::DockBuilderSplitNode(center, ImGuiDir_Right, 0.25f,
                                                        nullptr, &center);
            ImGuiID bottom = ImGui::DockBuilderSplitNode(center, ImGuiDir_Down, 0.35f,
                                                         nullptr, &center);

            // Every window is docked, open or not — DockBuilder records a home for
            // closed windows too, so opening one later lands it in its node as a tab
            // instead of floating loose over the viewport.
            //
            // Centre: the three rendering surfaces, tabbed like Unity's Scene/Game.
            ImGui::DockBuilderDockWindow("Scene", center);
            ImGui::DockBuilderDockWindow("Game", center);
            ImGui::DockBuilderDockWindow("Preview", center);

            ImGui::DockBuilderDockWindow("Hierarchy", left);

            // Right: the Inspector, with every settings panel stacked behind it —
            // Unity's own pattern (Inspector/Lighting/Occlusion share the right stack).
            // Settings are open-on-demand and land in a predictable place instead of
            // floating over the scene. Inspector is docked first so it is the front tab.
            ImGui::DockBuilderDockWindow("Inspector", right);
            ImGui::DockBuilderDockWindow("Environment", right);
            ImGui::DockBuilderDockWindow("Lighting", right);
            ImGui::DockBuilderDockWindow("Rendering", right);
            ImGui::DockBuilderDockWindow("Post Process", right);
            ImGui::DockBuilderDockWindow("GPU Culling", right);
            ImGui::DockBuilderDockWindow("Meteorology", right);
            ImGui::DockBuilderDockWindow("Terrain", right);
            ImGui::DockBuilderDockWindow("Physics", right);
            ImGui::DockBuilderDockWindow("Vehicle", right);
            ImGui::DockBuilderDockWindow("Assembly", right);

            // Bottom: the browser, the console, and the timeline-shaped tools.
            ImGui::DockBuilderDockWindow("Project", bottom);
            ImGui::DockBuilderDockWindow("Console", bottom);
            ImGui::DockBuilderDockWindow("Text Editor", bottom);
            ImGui::DockBuilderDockWindow("Animation", bottom);
            ImGui::DockBuilderDockWindow("Animator Graph", bottom);
            ImGui::DockBuilderDockWindow("Animator", bottom);
            ImGui::DockBuilderDockWindow("Audio Mixer", bottom);
            ImGui::DockBuilderDockWindow("Audio Profiler", bottom);
            // Not in the build yet (its wiring is planned work), but docking the name
            // costs nothing and means the window arrives already homed when it lands.
            ImGui::DockBuilderDockWindow("Audio Authoring", bottom);
            ImGui::DockBuilderDockWindow("Statistics", bottom);
            ImGui::DockBuilderDockWindow("Profiler", bottom);
            ImGui::DockBuilderFinish(dockspace_id);
        }
    } // namespace Editor
} // namespace SushiEngine
