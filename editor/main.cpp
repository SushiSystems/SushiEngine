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

#include <SushiEngine/render/window_renderer.hpp>

#include "editor_context.hpp"
#include "editor_panels.hpp"
#include "imgui_backend.hpp"
#include "sdl_window.hpp"

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

        sushi::editor::EditorContext context;
        context.project_root = std::filesystem::current_path().string();
        context.current_directory = context.project_root;

        // Seed a small scene so the hierarchy and inspector are populated on first run.
        sushi::editor::SceneNode* camera = context.scene.create_node("Main Camera");
        context.scene.create_node("Directional Light");
        sushi::editor::SceneNode* root = context.scene.create_node("Scene Root");
        context.scene.create_node("Child A", root);
        context.scene.create_node("Child B", root);
        context.selected_node = camera->id;
        sushi::editor::editor_log(context, "Editor ready (Vulkan).");

        bool running = true;
        while (running)
        {
            running = window.pump_events();

            imgui.new_frame();

            draw_dockspace();
            sushi::editor::draw_menu_bar(context, running);
            sushi::editor::draw_status_bar(context);
            sushi::editor::draw_toolbar_panel(context);
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
