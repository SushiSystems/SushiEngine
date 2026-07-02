/**************************************************************************/
/* imgui_backend.hpp                                                      */
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

#ifndef SUSHIENGINE_EDITOR_IMGUI_BACKEND_HPP
#define SUSHIENGINE_EDITOR_IMGUI_BACKEND_HPP

#include <SushiEngine/render/window_renderer.hpp>

#include "platform_window.hpp"

namespace sushi::editor
{
    /**
     * @brief Owns the Dear ImGui context bound to SDL2 + Vulkan for its whole life.
     *
     * The one editor component that legitimately speaks Vulkan: it stands up the
     * ImGui context, the SDL2 platform backend, and the Vulkan renderer backend
     * (dynamic rendering) against the window renderer's device, and routes the
     * window's raw events into ImGui. Construction initializes everything;
     * destruction tears it down in order. new_frame()/render() bracket the UI each
     * tick — render() takes the opaque command buffer from IWindowRenderer::begin_frame.
     */
    class ImGuiBackend
    {
        public:
            /**
             * @brief Initializes ImGui, the SDL2 backend, and the Vulkan backend.
             * @param window   The platform window hosting the UI (an SDL window).
             * @param renderer The windowed renderer providing the device and swapchain.
             * @throws std::runtime_error on descriptor-pool or backend init failure.
             */
            ImGuiBackend(IPlatformWindow& window,
                         SushiEngine::render::IWindowRenderer& renderer);
            ~ImGuiBackend();

            ImGuiBackend(const ImGuiBackend&) = delete;
            ImGuiBackend& operator=(const ImGuiBackend&) = delete;

            /** @brief Begins an ImGui frame (Vulkan + SDL2 + ImGui new-frame). */
            void new_frame();

            /**
             * @brief Renders the accumulated ImGui draw data into @p command_buffer.
             * @param command_buffer The opaque handle from IWindowRenderer::begin_frame.
             */
            void render(void* command_buffer);

        private:
            SushiEngine::render::IWindowRenderer& renderer_;
            void* descriptor_pool_ = nullptr; // VkDescriptorPool
    };
} // namespace sushi::editor

#endif
