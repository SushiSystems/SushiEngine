/**************************************************************************/
/* main.cpp                                                               */
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

// Editor shell: a Vulkan-backed window hosting Dear ImGui with a full-window
// dockspace and a Unity-style panel set — Hierarchy, Inspector, Project, and a
// text editor. It edits an editor-side scene and the on-disk project; a live
// viewport and world follow in later increments. The window (SdlWindow), the
// graphics device and swapchain (render::IWindowRenderer), and the ImGui/Vulkan
// glue (ImGuiBackend) sit behind narrow seams, so this loop names no windowing or
// graphics API directly and a different backend could replace either.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>

#include <imgui.h>
#include <imgui_internal.h>

#include <vector>

#include <SushiEngine/render/window_renderer.hpp>
#include <SushiEngine/sim/simulation.hpp>

#include "editor_context.hpp"
#include "editor_panels.hpp"
#include "imgui_backend.hpp"
#include "preferences.hpp"
#include "sdl_window.hpp"
#include "viewport_panel.hpp"

namespace
{
    // Where user-authored projects live by default: never inside the engine's own
    // source tree (writing project code next to the engine's is exactly the mixing
    // the Project panel exists to avoid). Falls back to the current directory only if
    // the per-user profile directory cannot be resolved.
    std::string default_projects_root()
    {
        std::filesystem::path home;
#ifdef _WIN32
        char* value = nullptr;
        std::size_t length = 0;
        if (_dupenv_s(&value, &length, "USERPROFILE") == 0 && value != nullptr)
        {
            home = value;
            std::free(value);
        }
#else
        if (const char* value = std::getenv("HOME"))
            home = value;
#endif
        std::filesystem::path root =
            !home.empty() ? home / "sushiengine" / "project" : std::filesystem::current_path();
        std::error_code ec;
        std::filesystem::create_directories(root, ec);
        return root.string();
    }
} // namespace

namespace
{
    // A single dockspace covering the main viewport, so the panels can be dragged,
    // tabbed, and split Unity-style. Rebuilt each frame; ImGui persists the layout
    // to imgui.ini between runs. On the first run (no persisted node yet) the
    // caller-provided default layout is applied once via build_default_layout.
    //
    // @return true on the frame the default layout still needs to be built.
    bool draw_dockspace()
    {
        ImGuiViewport* viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(viewport->WorkPos);
        ImGui::SetNextWindowSize(viewport->WorkSize);
        ImGui::SetNextWindowViewport(viewport->ID);

        ImGuiWindowFlags flags =
            ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse |
            ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus |
            ImGuiWindowFlags_NoDocking;

        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
        ImGui::Begin("SushiEngineDockHost", nullptr, flags);
        ImGui::PopStyleVar(3);

        const ImGuiID dockspace_id = ImGui::GetID("SushiEngineDockSpace");
        const bool needs_layout = ImGui::DockBuilderGetNode(dockspace_id) == nullptr;
        ImGui::DockSpace(dockspace_id, ImVec2(0.0f, 0.0f),
                         ImGuiDockNodeFlags_PassthruCentralNode);
        if (needs_layout)
            sushi::editor::build_default_layout(dockspace_id);
        ImGui::End();
        return needs_layout;
    }
}

int main(int, char**)
{
    try
    {
        sushi::editor::SdlWindow window("SushiEngine Editor", 1600, 900);

        std::uint32_t width = 0;
        std::uint32_t height = 0;
        window.drawable_size(width, height);

        SushiEngine::render::WindowRendererDesc desc;
        desc.required_instance_extensions = window.vulkan_instance_extensions();
        desc.surface_factory = [&window](std::uint64_t instance)
        {
            return window.create_vulkan_surface(instance);
        };
        desc.width = width != 0 ? width : 1600;
        desc.height = height != 0 ? height : 900;
        std::unique_ptr<SushiEngine::render::IWindowRenderer> renderer =
            SushiEngine::render::create_window_renderer(desc);

        sushi::editor::ImGuiBackend imgui(window, *renderer);

        // Two Unity viewports, each a ViewportPanel over the same world but a
        // different injected camera: the Scene view flies freely, the Game view
        // follows the world's camera. The cameras are declared before the panels so
        // they outlive the references the panels hold.
        sushi::editor::FlyCameraSource scene_camera;
        sushi::editor::WorldCameraSource game_camera;
        sushi::editor::ViewportPanel scene_view(*renderer, imgui, "Scene", scene_camera);
        sushi::editor::ViewportPanel game_view(*renderer, imgui, "Game", game_camera);

        // The live world, ticked on SushiRuntime behind the plain-C++ ISimulation
        // seam. The editor sees only the abstraction and the extracted RenderScene;
        // the runtime, SYCL, and ECS stay inside sushi_sim.
        std::unique_ptr<SushiEngine::sim::ISimulation> simulation =
            SushiEngine::sim::create_simulation();
        std::vector<SushiEngine::render::MeshInstance> instances;

        // The world is the single source of truth for entities; the panels read and
        // edit it through the injected simulation. There is no editor-side scene model.
        sushi::editor::EditorContext context;
        context.simulation = simulation.get();
        context.world_entity_count = simulation->entity_count();

        // Load persisted preferences and apply the live-effective ones up front, so the
        // editor opens in the user's theme and camera speed. The store is injected into
        // the context for the Preferences window to display its path.
        std::unique_ptr<sushi::editor::IPreferencesStore> preferences_store =
            sushi::editor::create_preferences_store();
        context.preferences_store = preferences_store.get();
        context.preferences = preferences_store->load();
        sushi::editor::apply_theme(context.preferences.theme);

        // The Project panel's root: the last one the user browsed to, or a
        // %USERPROFILE%/SushiProjects default — never the engine's own source tree,
        // so authored project code never mixes with the engine's.
        context.project_root = context.preferences.last_project_root.empty()
                                    ? default_projects_root()
                                    : context.preferences.last_project_root;
        context.current_directory = context.project_root;
        if (context.preferences.last_project_root != context.project_root)
        {
            context.preferences.last_project_root = context.project_root;
            preferences_store->save(context.preferences);
        }
        scene_camera.set_move_speed(context.preferences.camera_move_speed);

        sushi::editor::editor_log(context, "Editor ready (Vulkan).");
        sushi::editor::editor_log(context, "Live world seeded; press Play to tick it.");

        bool running = true;
        bool gizmo_was_dragging = false;
        while (running)
        {
            running = window.pump_events();

            // Tick the world on the runtime only while playing, so the toolbar's
            // Play/Pause gates motion; then take the fresh snapshot to draw.
            if (context.play_state == sushi::editor::PlayState::Playing)
                simulation->tick();

            const SushiEngine::sim::RenderScene& scene = simulation->render_scene();
            instances.clear();
            instances.reserve(scene.instances.size());
            for (const SushiEngine::sim::RenderInstance& source : scene.instances)
            {
                SushiEngine::render::MeshInstance instance;
                instance.model = source.model;
                instance.color = source.color;
                instance.id = static_cast<std::uint32_t>(source.id);
                instances.push_back(instance);
            }
            context.world_entity_count = simulation->entity_count();

            // Resolve which display the Game view shows: the selected display's camera
            // if present, else the default. Also gather the display options for the
            // Game panel's selector so two cameras on different displays never conflict.
            std::vector<std::uint32_t> displays;
            displays.reserve(scene.display_cameras.size());
            const SushiEngine::sim::CameraState* game = &scene.camera;
            bool selected_display_present = false;
            for (const SushiEngine::sim::DisplayCamera& display_camera : scene.display_cameras)
            {
                displays.push_back(display_camera.display);
                if (display_camera.display == context.game_display)
                {
                    game = &display_camera.state;
                    selected_display_present = true;
                }
            }
            // If the chosen display vanished (its camera was deleted), fall back to the
            // first available so the Game view keeps rendering.
            if (!selected_display_present && !scene.display_cameras.empty())
            {
                context.game_display = scene.display_cameras.front().display;
                game = &scene.display_cameras.front().state;
            }
            game_camera.set_pose(game->position, game->target, game->up,
                                 game->vertical_fov_radians, game->near_plane, game->far_plane);

            imgui.new_frame();

            draw_dockspace();

            // Global undo/redo shortcuts, gated off text-entry widgets so Ctrl+Z in a
            // document or a rename field is not hijacked by the scene history.
            if (!ImGui::GetIO().WantTextInput && simulation != nullptr)
            {
                SushiEngine::sim::IWorldEditor& editor_world = simulation->world();
                const bool ctrl = ImGui::GetIO().KeyCtrl;
                if (ctrl && ImGui::IsKeyPressed(ImGuiKey_Z, false) &&
                    context.history.undo(editor_world))
                    sushi::editor::select_only(context, SushiEngine::sim::NULL_ENTITY);
                else if (ctrl && ImGui::IsKeyPressed(ImGuiKey_Y, false) &&
                         context.history.redo(editor_world))
                    sushi::editor::select_only(context, SushiEngine::sim::NULL_ENTITY);
            }

            sushi::editor::draw_menu_bar(context, running);
            sushi::editor::draw_status_bar(context);
            sushi::editor::draw_toolbar_panel(context);
            // Selection is shared between the viewports and the panels. The scene
            // renderer speaks 32-bit ids; entity ids stay small, so the round-trip is
            // lossless. A left-click in either viewport picks the entity under it.
            std::uint32_t selected = static_cast<std::uint32_t>(context.selected_entity);

            // The Scene view gets the transform gizmo at the selection; a drag edits a
            // local copy of the transform written back to the world afterwards.
            SushiEngine::sim::IWorldEditor& world = simulation->world();
            const bool has_selection = world.exists(context.selected_entity);
            SushiEngine::sim::EntityTransform selected_transform;
            SushiEngine::sim::EntityTransform* gizmo_target = nullptr;
            if (has_selection)
            {
                selected_transform = world.transform(context.selected_entity);
                gizmo_target = &selected_transform;
            }

            // Gizmo snapping comes from the Scene preferences; off unless enabled there.
            sushi::editor::GizmoSnap snap;
            snap.enabled = context.preferences.snap_enabled;
            snap.translate = context.preferences.snap_translate;
            snap.rotate_degrees = context.preferences.snap_rotate_degrees;
            snap.scale = context.preferences.snap_scale;

            bool gizmo_edited = false;
            if (context.panels.scene_view)
                gizmo_edited = scene_view.draw(context.panels.scene_view, instances.data(),
                                               instances.size(), selected, true, gizmo_target,
                                               context.gizmo_mode, context.gizmo_space, &snap);
            if (context.panels.game_view)
            {
                // The Game view is played, not authored: no picking, no gizmo. It offers
                // a display selector so multiple cameras can target different displays.
                sushi::editor::DisplaySelector selector;
                selector.displays = displays.data();
                selector.count = displays.size();
                selector.selected = &context.game_display;
                game_view.draw(context.panels.game_view, instances.data(), instances.size(),
                               selected, false, nullptr, sushi::editor::GizmoMode::Translate,
                               sushi::editor::GizmoSpace::World, nullptr, &selector);
            }

            // One undo step per whole drag, not one per frame: snapshot on the frame
            // the handle is grabbed, commit on the frame it is released.
            const bool gizmo_is_dragging = scene_view.gizmo_dragging();
            if (gizmo_is_dragging && !gizmo_was_dragging)
                context.history.begin_change(world);
            else if (!gizmo_is_dragging && gizmo_was_dragging)
                context.history.end_change();
            gizmo_was_dragging = gizmo_is_dragging;

            // Write a gizmo edit back only when the selection did not change this frame
            // (a pick and a drag are mutually exclusive).
            if (has_selection && gizmo_edited &&
                selected == static_cast<std::uint32_t>(context.selected_entity))
                world.set_transform(context.selected_entity, selected_transform);
            // A viewport click always replaces the whole selection (no multi-select
            // there yet), but only when it actually changed the pick this frame — this
            // runs every frame regardless, and re-collapsing to one entity every frame
            // would fight the Hierarchy's Ctrl/Shift multi-select.
            if (selected != static_cast<std::uint32_t>(context.selected_entity))
                sushi::editor::select_only(
                    context, static_cast<SushiEngine::sim::EntityId>(selected));
            else
                context.selected_entity = selected;

            // Service the Scene-camera / framing requests raised by the Hierarchy
            // (double-click) and the GameObject menu, now that the selection is settled.
            if (world.exists(context.selected_entity) &&
                (context.frame_selected_requested || context.align_with_view_requested ||
                 context.move_to_view_requested))
            {
                const SushiEngine::sim::EntityTransform target =
                    world.transform(context.selected_entity);
                sushi::editor::FlyCamera& fly = scene_camera.camera();
                if (context.frame_selected_requested)
                {
                    // Teleport the camera beside the entity, keeping its facing.
                    fly.position =
                        target.position - fly.forward() * static_cast<SushiEngine::Scalar>(6);
                }
                if (context.align_with_view_requested)
                {
                    SushiEngine::sim::EntityTransform aligned = target;
                    aligned.position = fly.position;
                    aligned.rotation = fly.orientation();
                    world.set_transform(context.selected_entity, aligned);
                }
                if (context.move_to_view_requested)
                {
                    SushiEngine::sim::EntityTransform moved = target;
                    moved.position =
                        fly.position + fly.forward() * static_cast<SushiEngine::Scalar>(6);
                    world.set_transform(context.selected_entity, moved);
                }
            }
            context.frame_selected_requested = false;
            context.align_with_view_requested = false;
            context.move_to_view_requested = false;
            sushi::editor::draw_hierarchy_panel(context);
            sushi::editor::draw_inspector_panel(context);
            sushi::editor::draw_project_panel(context);
            sushi::editor::draw_text_editor_panel(context);
            sushi::editor::draw_console_panel(context);
            sushi::editor::draw_statistics_panel(context);
            sushi::editor::draw_preferences_window(context);
            sushi::editor::draw_save_scene_as_modal(context);

            // Persist preferences once per frame after any edit, and apply the fields
            // that take effect live (the camera speed; theme is applied on change).
            if (context.preferences_dirty)
            {
                scene_camera.set_move_speed(context.preferences.camera_move_speed);
                preferences_store->save(context.preferences);
                context.preferences_dirty = false;
            }
            if (context.show_imgui_demo)
                ImGui::ShowDemoWindow(&context.show_imgui_demo);

            window.drawable_size(width, height);
            if (void* command_buffer = renderer->begin_frame(width, height))
            {
                imgui.render(command_buffer);
                renderer->end_frame();
            }
            else
            {
                ImGui::EndFrame(); // no frame presented (minimized/resize); close the UI frame
            }
        }

        preferences_store->save(context.preferences);
        renderer->wait_idle();
        return 0;
    }
    catch (const std::exception& error)
    {
        std::fprintf(stderr, "SushiEngine editor: %s\n", error.what());
        return 1;
    }
}
