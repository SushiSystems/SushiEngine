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
// graphics device and swapchain (Render::IWindowRenderer), and the ImGui/Vulkan
// glue (ImGuiBackend) sit behind narrow seams, so this loop names no windowing or
// graphics API directly and a different backend could replace either.

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>

#include <imgui.h>
#include <imgui_internal.h>

#include <vector>

#include <SushiEngine/astro/ephemeris.hpp>
#include <SushiEngine/render/window_renderer.hpp>
#include <SushiEngine/sim/simulation.hpp>

#include <memory>

#include "core/editor_context.hpp"
#include "ui/editor_panels.hpp"
#include "ui/imgui_backend.hpp"
#include "core/preferences.hpp"
#include "serialization/scene_serializer.hpp"
#include "window/sdl_window.hpp"
#include "ui/viewport_panel.hpp"

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
            SushiEngine::Editor::build_default_layout(dockspace_id);
        ImGui::End();
        return needs_layout;
    }
}

int main(int, char**)
{
    try
    {
        SushiEngine::Editor::SdlWindow window("SushiEngine Editor", 1600, 900);

        std::uint32_t width = 0;
        std::uint32_t height = 0;
        window.drawable_size(width, height);

        SushiEngine::Render::WindowRendererDesc desc;
        desc.required_instance_extensions = window.vulkan_instance_extensions();
        desc.surface_factory = [&window](std::uint64_t instance)
        {
            return window.create_vulkan_surface(instance);
        };
        desc.width = width != 0 ? width : 1600;
        desc.height = height != 0 ? height : 900;
        std::unique_ptr<SushiEngine::Render::IWindowRenderer> renderer =
            SushiEngine::Render::create_window_renderer(desc);

        SushiEngine::Editor::ImGuiBackend imgui(window, *renderer);

        // Two Unity viewports, each a ViewportPanel over the same world but a
        // different injected camera: the Scene view flies freely, the Game view
        // follows the world's camera. The cameras are declared before the panels so
        // they outlive the references the panels hold.
        SushiEngine::Editor::FlyCameraSource scene_camera;
        SushiEngine::Editor::WorldCameraSource game_camera;
        SushiEngine::Editor::ViewportPanel scene_view(*renderer, imgui, "Scene", scene_camera);
        SushiEngine::Editor::ViewportPanel game_view(*renderer, imgui, "Game", game_camera);

        // The world is the single source of truth for entities; the panels read and
        // edit it through the injected simulation. There is no editor-side scene model.
        SushiEngine::Editor::EditorContext context;

        // Load persisted preferences first, so the editor opens in the user's theme
        // and camera speed. The store is injected into the context for the
        // Preferences window to display its path.
        std::unique_ptr<SushiEngine::Editor::IPreferencesStore> preferences_store =
            SushiEngine::Editor::create_preferences_store();
        context.preferences_store = preferences_store.get();
        context.preferences = preferences_store->load();
        SushiEngine::Editor::apply_theme(context.preferences.theme);

        // The live world, ticked on SushiRuntime behind the plain-C++ ISimulation
        // seam. The editor sees only the abstraction and the extracted RenderScene;
        // the runtime, SYCL, and ECS stay inside sushi_sim.
        std::unique_ptr<SushiEngine::Simulation::ISimulation> simulation =
            SushiEngine::Simulation::create_simulation();
        std::vector<SushiEngine::Render::MeshInstance> instances;
        context.simulation = simulation.get();
        context.world_entity_count = simulation->entity_count();

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

        SushiEngine::Editor::editor_log(context, "Editor ready (Vulkan).");
        SushiEngine::Editor::editor_log(context, "No scene open. Use File > New Scene or open a "
                                            ".sushiscene from the Project panel.");

        bool running = true;
        bool gizmo_was_dragging = false;
        bool ui_was_dragging = false;
        // The one wall-clock read in the editor loop: real elapsed time since the
        // last frame, fed into ISimulation::tick() so its FixedTimestepClock can
        // turn it into whole fixed steps. The sim itself never reads the clock.
        std::chrono::steady_clock::time_point last_frame_time =
            std::chrono::steady_clock::now();
        while (running)
        {
            const std::chrono::steady_clock::time_point frame_time =
                std::chrono::steady_clock::now();
            const SushiEngine::Scalar real_delta_seconds =
                std::chrono::duration<SushiEngine::Scalar>(frame_time - last_frame_time).count();
            last_frame_time = frame_time;

            // A close request (the window's X, or File > Exit) is not obeyed directly;
            // it only sets close_requested, so draw_exit_confirm_modal below gets a
            // chance to hold the window open while unsaved changes are pending.
            if (!window.pump_events())
                context.close_requested = true;

            // Tick the world on the runtime only while playing, so the toolbar's
            // Play/Pause gates motion; then take the fresh snapshot to draw. Step
            // advances exactly one fixed step (via a zero-length real delta plus a
            // full accumulated one) regardless of play_state (normally pressed while
            // Paused) and is a one-shot request the toolbar sets.
            if (context.play_state == SushiEngine::Editor::PlayState::Playing)
                simulation->tick(real_delta_seconds);
            else if (context.step_requested)
                simulation->tick(simulation->fixed_dt_seconds());
            context.step_requested = false;

            const SushiEngine::Simulation::RenderScene& scene = simulation->render_scene();
            instances.clear();
            instances.reserve(scene.instances.size());
            for (const SushiEngine::Simulation::RenderInstance& source : scene.instances)
            {
                SushiEngine::Render::MeshInstance instance;
                instance.model = source.model;
                instance.color = source.color;
                instance.id = static_cast<std::uint32_t>(source.id);
                // Simulation::PrimitiveKind and Render::MeshKind share Box/Sphere/Cylinder
                // ordinal order by construction; Plane never reaches a RenderInstance
                // (Terrain's visual Shape is always a Box).
                instance.kind =
                    static_cast<SushiEngine::Render::MeshKind>(source.shape_kind);
                instance.shape_params = source.shape_params;
                instance.material = source.material;
                instances.push_back(instance);
            }
            context.world_entity_count = simulation->entity_count();

            // Soft-body meshes: one strand view per cloth grid, pointing directly
            // into the snapshot's concatenated vertex buffer for this frame's lifetime.
            std::vector<SushiEngine::Render::ClothStrandView> strands;
            strands.reserve(scene.cloth_instances.size());
            for (const SushiEngine::Simulation::ClothInstance& cloth : scene.cloth_instances)
            {
                SushiEngine::Render::ClothStrandView strand;
                strand.rows = cloth.rows;
                strand.cols = cloth.cols;
                strand.vertices = scene.cloth_vertices.data() + cloth.first_vertex;
                strand.color = cloth.color;
                strand.id = static_cast<std::uint32_t>(cloth.id);
                strands.push_back(strand);
            }

            // Resolve which display the Game view shows: the selected display's camera
            // if present, else the default. Also gather the display options for the
            // Game panel's selector so two cameras on different displays never conflict.
            std::vector<std::uint32_t> displays;
            displays.reserve(scene.display_cameras.size());
            const SushiEngine::Simulation::CameraState* game = &scene.camera;
            bool selected_display_present = false;
            for (const SushiEngine::Simulation::DisplayCamera& display_camera : scene.display_cameras)
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

            // Global undo/redo/save shortcuts, gated off text-entry widgets so Ctrl+Z in
            // a document or a rename field is not hijacked by the scene history.
            if (!ImGui::GetIO().WantTextInput && simulation != nullptr)
            {
                SushiEngine::Simulation::IWorldEditor& editor_world = simulation->world();
                const bool ctrl = ImGui::GetIO().KeyCtrl;
                if (ctrl && ImGui::IsKeyPressed(ImGuiKey_Z, false) &&
                    context.history.undo(editor_world))
                    SushiEngine::Editor::select_only(context, SushiEngine::Simulation::NULL_ENTITY);
                else if (ctrl && ImGui::IsKeyPressed(ImGuiKey_Y, false) &&
                         context.history.redo(editor_world))
                    SushiEngine::Editor::select_only(context, SushiEngine::Simulation::NULL_ENTITY);
                else if (ctrl && ImGui::IsKeyPressed(ImGuiKey_S, false))
                    SushiEngine::Editor::save_current_scene(context);
                else if (ctrl && ImGui::IsKeyPressed(ImGuiKey_C, false))
                    SushiEngine::Editor::copy_selection(context);
                else if (ctrl && ImGui::IsKeyPressed(ImGuiKey_X, false))
                    SushiEngine::Editor::cut_selection(context);
                else if (ctrl && ImGui::IsKeyPressed(ImGuiKey_V, false))
                    SushiEngine::Editor::paste_clipboard(context);
            }

            SushiEngine::Editor::draw_menu_bar(context);
            SushiEngine::Editor::draw_status_bar(context);
            SushiEngine::Editor::draw_toolbar_panel(context);
            // Selection is shared between the viewports and the panels. The scene
            // renderer speaks 32-bit ids; entity ids stay small, so the round-trip is
            // lossless. A left-click in either viewport picks the entity under it.
            std::uint32_t selected = static_cast<std::uint32_t>(context.selected_entity);

            // The Scene view gets the transform gizmo at the selection; a drag edits a
            // local copy of the transform written back to the world afterwards.
            SushiEngine::Simulation::IWorldEditor& world = simulation->world();
            const bool has_selection = world.exists(context.selected_entity);
            SushiEngine::Simulation::EntityTransform selected_transform;
            SushiEngine::Simulation::EntityTransform* gizmo_target = nullptr;
            if (has_selection)
            {
                selected_transform = world.world_transform(context.selected_entity);
                gizmo_target = &selected_transform;
            }

            // Gizmo snapping comes from the Scene preferences; off unless enabled there.
            SushiEngine::Editor::GizmoSnap snap;
            snap.enabled = context.preferences.snap_enabled;
            snap.translate = context.preferences.snap_translate;
            snap.rotate_degrees = context.preferences.snap_rotate_degrees;
            snap.scale = context.preferences.snap_scale;

            // UI overlay: every entity carrying a UI element, flattened with its UI
            // parent resolved to an index, so the viewport can lay it out against the
            // panel rect and paint canvases/panels/images/text/buttons over the 3D view.
            std::vector<SushiEngine::Editor::UIOverlayElement> ui_overlay;
            std::vector<SushiEngine::Simulation::EntityId> ui_ids;
            for (const SushiEngine::Simulation::EntityId id : world.entities())
            {
                if (!world.has_ui(id) || !world.visible(id))
                    continue;
                SushiEngine::Editor::UIOverlayElement element;
                element.id = static_cast<std::uint32_t>(id);
                element.params = world.ui_params(id);
                element.selected = id == context.selected_entity;
                ui_overlay.push_back(element);
                ui_ids.push_back(id);
            }
            for (std::size_t i = 0; i < ui_overlay.size(); ++i)
            {
                const SushiEngine::Simulation::EntityId parent_id = world.parent(ui_ids[i]);
                for (std::size_t j = 0; j < ui_ids.size(); ++j)
                    if (ui_ids[j] == parent_id)
                    {
                        ui_overlay[i].parent = static_cast<int>(j);
                        break;
                    }
            }

            // The Scene view edits the UI (translucent, interactive); the Game view shows
            // it solid. Both draw the same (possibly just-dragged) elements.
            SushiEngine::Editor::UIOverlay scene_ui;
            scene_ui.elements = ui_overlay.data();
            scene_ui.count = ui_overlay.size();
            scene_ui.edit_mode = true;
            scene_ui.selected_id = static_cast<std::uint32_t>(context.selected_entity);
            SushiEngine::Editor::UIOverlay game_ui;
            game_ui.elements = ui_overlay.data();
            game_ui.count = ui_overlay.size();
            game_ui.edit_mode = false;

            // Solar-system sky: repopulate the far-field bodies and stars from the authored
            // epoch and observer each frame. A local copy of the world's environment is
            // mutated and handed to the viewports, so driving the ephemeris never
            // re-extracts the world (unlike set_environment, which does).
            SushiEngine::Render::Environment environment = scene.environment;
            if (context.sky_enabled)
            {
                if (context.sky_animate)
                    context.sky_accumulated_days +=
                        real_delta_seconds * context.sky_days_per_second;
                environment.observer.julian_date =
                    SushiEngine::Astro::julian_date_from_calendar(context.sky_date) +
                    context.sky_accumulated_days;
                environment.observer.latitude_radians =
                    context.sky_latitude_degrees * SushiEngine::Astro::DEGREES_TO_RADIANS;
                environment.observer.longitude_radians =
                    context.sky_longitude_degrees * SushiEngine::Astro::DEGREES_TO_RADIANS;
                environment.observer.astronomical_sun = context.sky_astronomical_sun;

                if (context.sky_space_mode)
                {
                    SushiEngine::Editor::FlyCamera& fly = scene_camera.camera();
                    const double au_to_gm = SushiEngine::Astro::METRES_PER_ASTRONOMICAL_UNIT /
                                            SushiEngine::Astro::METRES_PER_GIGAMETRE;
                    if (!context.sky_space_active)
                    {
                        // Enter space: save the surface pose and drop the camera beside Earth.
                        context.sky_saved_surface_position = fly.position;
                        const SushiEngine::Vector3 earth_au =
                            SushiEngine::Astro::planet_heliocentric_au(
                                SushiEngine::Astro::BodyId::Earth,
                                environment.observer.julian_date);
                        const SushiEngine::Vector3 earth_world =
                            SushiEngine::Astro::ecliptic_to_world(earth_au * au_to_gm);
                        fly.position =
                            earth_world + SushiEngine::Vector3{0.0, 0.03, 0.12};
                        scene_camera.controller().altitude_adaptive = false;
                        context.sky_space_active = true;
                    }
                    const SushiEngine::WorldVector3 camera_gm{fly.position.x, fly.position.y,
                                                              fly.position.z};
                    SushiEngine::Astro::fill_environment_space(environment, camera_gm);

                    // Proximity-scaled flight speed: base speed tracks the nearest body's
                    // surface, so approaching a planet slows the camera to a controllable
                    // crawl while interplanetary hops stay fast. Units are gigametres.
                    double nearest = 1e30;
                    for (int i = 0; i < environment.body_count; ++i)
                    {
                        const double surface_distance =
                            static_cast<double>(environment.bodies[i].distance_metres) -
                            static_cast<double>(environment.bodies[i].mean_radius_metres);
                        if (surface_distance < nearest)
                            nearest = surface_distance;
                    }
                    double base_speed = nearest * 0.5;
                    if (base_speed < 1.0e-4)
                        base_speed = 1.0e-4;
                    if (base_speed > 50.0)
                        base_speed = 50.0;
                    scene_camera.controller().move_speed =
                        static_cast<SushiEngine::Scalar>(base_speed);
                }
                else
                {
                    if (context.sky_space_active)
                    {
                        // Exit space: restore the surface pose and ground-scaled navigation.
                        scene_camera.camera().position = context.sky_saved_surface_position;
                        scene_camera.controller().altitude_adaptive = true;
                        scene_camera.controller().move_speed =
                            context.preferences.camera_move_speed;
                        context.sky_space_active = false;
                    }
                    SushiEngine::Astro::fill_environment_sky(environment);
                }
            }

            bool gizmo_edited = false;
            if (context.panels.scene_view)
            {
                // In the interplanetary regime the world's metre-scale meshes are not part
                // of the gigametre-scale solar system, so the Scene view draws none of them.
                const std::size_t scene_instance_count =
                    context.sky_space_mode ? 0 : instances.size();
                gizmo_edited = scene_view.draw(context.panels.scene_view, instances.data(),
                                               scene_instance_count, environment, selected, true,
                                               gizmo_target, context.gizmo_mode, context.gizmo_space,
                                               &snap, nullptr, strands.data(), strands.size(),
                                               &scene_ui);
                // The Scene view is the surface the UI is authored against, so its size
                // drives every Canvas's layout — the per-frame equivalent of a window
                // resize event for a full-viewport UI root.
                world.set_ui_target_size(scene_view.target_width(), scene_view.target_height());
            }
            if (context.panels.game_view)
            {
                // The Game view is played, not authored: no picking, no gizmo. It offers
                // a display selector so multiple cameras can target different displays.
                SushiEngine::Editor::DisplaySelector selector;
                selector.displays = displays.data();
                selector.count = displays.size();
                selector.selected = &context.game_display;
                // Pass no selection so the Scene view's pick never highlights in the
                // Game view; it is not pickable here so nothing writes this back.
                std::uint32_t no_selection = 0;
                // With no active camera there is nothing to play the scene through, so
                // the Game view draws zero instances (clears to black) rather than
                // falling back to a synthetic default camera and rendering anyway.
                const std::size_t game_instance_count =
                    scene.has_camera ? instances.size() : 0;
                game_view.draw(context.panels.game_view, instances.data(), game_instance_count,
                               environment, no_selection, false, nullptr,
                               SushiEngine::Editor::GizmoMode::Translate,
                               SushiEngine::Editor::GizmoSpace::World, nullptr, &selector,
                               strands.data(), strands.size(), &game_ui);
            }

            // Fold the UI overlay's interaction into the shared selection/edit flow: a UI
            // pick in the Scene view replaces the 3D pick this frame, and a UI drag writes
            // the element's new rect back to the world.
            if (scene_ui.picked_id != 0)
                selected = scene_ui.picked_id;
            const bool ui_is_dragging = scene_view.ui_dragging();
            if (ui_is_dragging && !ui_was_dragging)
                context.history.begin_change(world);
            else if (!ui_is_dragging && ui_was_dragging)
                context.history.end_change();
            ui_was_dragging = ui_is_dragging;
            if (scene_ui.edited_index >= 0 &&
                static_cast<std::size_t>(scene_ui.edited_index) < ui_ids.size())
                world.set_ui_params(
                    ui_ids[static_cast<std::size_t>(scene_ui.edited_index)],
                    ui_overlay[static_cast<std::size_t>(scene_ui.edited_index)].params);

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
                world.set_world_transform(context.selected_entity, selected_transform);
            // A viewport click always replaces the whole selection (no multi-select
            // there yet), but only when it actually changed the pick this frame — this
            // runs every frame regardless, and re-collapsing to one entity every frame
            // would fight the Hierarchy's Ctrl/Shift multi-select.
            if (selected != static_cast<std::uint32_t>(context.selected_entity))
                SushiEngine::Editor::select_only(
                    context, static_cast<SushiEngine::Simulation::EntityId>(selected));
            else
                context.selected_entity = selected;

            // Service the Scene-camera / framing requests raised by the Hierarchy
            // (double-click) and the Entity menu, now that the selection is settled.
            if (world.exists(context.selected_entity) &&
                (context.frame_selected_requested || context.align_with_view_requested ||
                 context.move_to_view_requested))
            {
                const SushiEngine::Simulation::EntityTransform target =
                    world.world_transform(context.selected_entity);
                SushiEngine::Editor::FlyCamera& fly = scene_camera.camera();
                if (context.frame_selected_requested)
                {
                    // Teleport the camera beside the entity, keeping its facing.
                    fly.position =
                        target.position - fly.forward() * static_cast<SushiEngine::Scalar>(6);
                }
                if (context.align_with_view_requested)
                {
                    SushiEngine::Simulation::EntityTransform aligned = target;
                    aligned.position = fly.position;
                    aligned.rotation = fly.orientation();
                    world.set_world_transform(context.selected_entity, aligned);
                }
                if (context.move_to_view_requested)
                {
                    SushiEngine::Simulation::EntityTransform moved = target;
                    moved.position =
                        fly.position + fly.forward() * static_cast<SushiEngine::Scalar>(6);
                    world.set_world_transform(context.selected_entity, moved);
                }
            }
            context.frame_selected_requested = false;
            context.align_with_view_requested = false;
            context.move_to_view_requested = false;
            SushiEngine::Editor::draw_hierarchy_panel(context);
            SushiEngine::Editor::draw_inspector_panel(context);
            SushiEngine::Editor::draw_environment_panel(context);
            SushiEngine::Editor::draw_project_panel(context);
            SushiEngine::Editor::draw_text_editor_panel(context);
            SushiEngine::Editor::draw_console_panel(context);
            SushiEngine::Editor::draw_statistics_panel(context);
            SushiEngine::Editor::draw_preferences_window(context);
            SushiEngine::Editor::draw_save_scene_as_modal(context, running);
            SushiEngine::Editor::draw_exit_confirm_modal(context, running);
            SushiEngine::Editor::draw_scene_action_confirm_modal(context);

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
