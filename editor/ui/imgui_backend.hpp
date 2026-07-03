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

#include <imgui.h>

#include <SushiEngine/render/window_renderer.hpp>

#include "../window/platform_window.hpp"

namespace SushiEngine
{
    namespace Editor
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
                             SushiEngine::Render::IWindowRenderer& renderer);
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

                /**
                 * @brief Registers a sampled image as an ImGui texture for ImGui::Image.
                 *
                 * Wraps ImGui_ImplVulkan_AddTexture so the scene-view panels can display
                 * their offscreen colour targets; the returned id stays valid until
                 * unregister_texture() or a target resize.
                 *
                 * @param sampler    A VkSampler (as void*) for the image.
                 * @param image_view A VkImageView (as void*) in shader-read layout.
                 * @return An ImGui texture id usable with ImGui::Image.
                 */
                ImTextureID register_texture(void* sampler, void* image_view);

                /**
                 * @brief Releases a texture id previously returned by register_texture().
                 * @param texture The id to release; ignored if zero.
                 */
                void unregister_texture(ImTextureID texture);

            private:
                SushiEngine::Render::IWindowRenderer& renderer_;
                void* descriptor_pool_ = nullptr; // VkDescriptorPool
        };
    } // namespace Editor
} // namespace SushiEngine

#endif
