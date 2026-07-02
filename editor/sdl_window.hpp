/**************************************************************************/
/* sdl_window.hpp                                                         */
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

#ifndef SUSHIENGINE_EDITOR_SDL_WINDOW_HPP
#define SUSHIENGINE_EDITOR_SDL_WINDOW_HPP

#include "platform_window.hpp"

struct SDL_Window;

namespace sushi::editor
{
    /**
     * @brief The SDL2 implementation of IPlatformWindow, created for Vulkan.
     *
     * Opens an SDL_WINDOW_VULKAN window, translates SDL surface/extension queries
     * into the neutral IPlatformWindow surface, and pumps SDL events. Owns the SDL
     * subsystem lifetime: constructing it initializes SDL video, destroying it quits.
     * The only SDL-aware editor component besides the ImGui backend.
     */
    class SdlWindow final : public IPlatformWindow
    {
        public:
            /**
             * @brief Opens the window and initializes SDL's video subsystem.
             * @param title  Window title bar text.
             * @param width  Initial window width in pixels.
             * @param height Initial window height in pixels.
             * @throws std::runtime_error if SDL or the window fails to initialize.
             */
            SdlWindow(const char* title, int width, int height);
            ~SdlWindow() override;

            SdlWindow(const SdlWindow&) = delete;
            SdlWindow& operator=(const SdlWindow&) = delete;

            bool pump_events() override;
            void set_event_handler(EventHandler handler) override { handler_ = std::move(handler); }
            void drawable_size(std::uint32_t& width, std::uint32_t& height) const override;
            std::vector<std::string> vulkan_instance_extensions() const override;
            std::uint64_t create_vulkan_surface(std::uint64_t instance) const override;
            void* native_handle() const override { return window_; }

        private:
            SDL_Window* window_ = nullptr;
            EventHandler handler_;
    };
} // namespace sushi::editor

#endif
