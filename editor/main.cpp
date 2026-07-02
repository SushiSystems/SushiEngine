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
#include "sdl_window.hpp"
#include "viewport_panel.hpp"

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
        context.project_root = std::filesystem::current_path().string();
        context.current_directory = context.project_root;
        context.world_entity_count = simulation->entity_count();
        sushi::editor::editor_log(context, "Editor ready (Vulkan).");
        sushi::editor::editor_log(context, "Live world seeded; press Play to tick it.");

        bool running = true;
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

            // Pose the Game camera from the world's camera this frame.
            const SushiEngine::sim::CameraState& game = scene.camera;
            game_camera.set_pose(game.position, game.target, game.up,
                                 game.vertical_fov_radians, game.near_plane, game.far_plane);

            imgui.new_frame();

            draw_dockspace();
            sushi::editor::draw_menu_bar(context, running);
            sushi::editor::draw_status_bar(context);
            sushi::editor::draw_toolbar_panel(context);
            // Selection is shared between the viewports and the panels. The scene
            // renderer speaks 32-bit ids; entity ids stay small, so the round-trip is
            // lossless. A left-click in either viewport picks the entity under it.
            std::uint32_t selected = static_cast<std::uint32_t>(context.selected_entity);
            if (context.panels.scene_view)
                scene_view.draw(context.panels.scene_view, instances.data(), instances.size(),
                                selected);
            if (context.panels.game_view)
                game_view.draw(context.panels.game_view, instances.data(), instances.size(),
                               selected);
            context.selected_entity = selected;
            sushi::editor::draw_hierarchy_panel(context);
            sushi::editor::draw_inspector_panel(context);
            sushi::editor::draw_project_panel(context);
            sushi::editor::draw_text_editor_panel(context);
            sushi::editor::draw_console_panel(context);
            sushi::editor::draw_statistics_panel(context);
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

        renderer->wait_idle();
        return 0;
    }
    catch (const std::exception& error)
    {
        std::fprintf(stderr, "SushiEngine editor: %s\n", error.what());
        return 1;
    }
}
