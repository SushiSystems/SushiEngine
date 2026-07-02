/**************************************************************************/
/* viewport_panel.hpp                                                     */
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

#ifndef SUSHIENGINE_EDITOR_VIEWPORT_PANEL_HPP
#define SUSHIENGINE_EDITOR_VIEWPORT_PANEL_HPP

#include <cstddef>
#include <memory>
#include <vector>

#include <imgui.h>

#include <SushiEngine/render/scene_view.hpp>
#include <SushiEngine/render/window_renderer.hpp>

#include "camera_controller.hpp"
#include "fly_camera.hpp"
#include "imgui_backend.hpp"

namespace sushi::editor
{
    /**
     * @brief A Unity-style Scene panel: a Vulkan viewport with a fly camera.
     *
     * Owns an offscreen scene view, a fly camera, and its controller. Each frame it
     * sizes the target to the panel, gathers navigation input from ImGui (right mouse
     * to look and fly, WASD/QE, Shift to boost) while the panel is interacted with,
     * renders the given mesh instances, and displays the result with ImGui::Image.
     * The offscreen colour target is registered with the ImGui backend as a texture,
     * re-registered on resize.
     */
    class ViewportPanel
    {
        public:
            /**
             * @brief Creates the scene view and registers its textures with ImGui.
             * @param renderer The window renderer that owns the device.
             * @param imgui    The ImGui backend used to register sampled textures.
             * @param title    The panel window title (e.g. "Scene").
             */
            ViewportPanel(SushiEngine::render::IWindowRenderer& renderer, ImGuiBackend& imgui,
                          const char* title);
            ~ViewportPanel();

            ViewportPanel(const ViewportPanel&) = delete;
            ViewportPanel& operator=(const ViewportPanel&) = delete;

            /**
             * @brief Draws the panel and renders the scene into it.
             *
             * The camera is driven only while the panel is interacted with, so input
             * over other panels never moves the view.
             *
             * @param open      Visibility flag, bound to the panel's close button.
             * @param instances The mesh instances to draw this frame.
             * @param count     Number of instances.
             */
            void draw(bool& open, const SushiEngine::render::MeshInstance* instances,
                      std::size_t count);

            /** @brief The panel's fly camera, for the Game view or inspection. */
            FlyCamera& camera() noexcept { return camera_; }

        private:
            void resize_to(std::uint32_t width, std::uint32_t height);
            void register_textures();
            void unregister_textures();

            ImGuiBackend& imgui_;
            const char* title_;
            std::unique_ptr<SushiEngine::render::ISceneView> view_;
            std::vector<ImTextureID> slot_textures_;
            FlyCamera camera_;
            CameraController controller_;
            bool looking_ = false;
    };
} // namespace sushi::editor

#endif
