/**************************************************************************/
/* platform_window.hpp                                                    */
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

#ifndef SUSHIENGINE_EDITOR_PLATFORM_WINDOW_HPP
#define SUSHIENGINE_EDITOR_PLATFORM_WINDOW_HPP

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace sushi::editor
{
    /**
     * @brief The window the editor runs in, abstracted from any windowing library.
     *
     * The editor's app logic programs against this, never against SDL, so a
     * different backend (GLFW, Win32, Wayland) can replace @ref SdlWindow without
     * touching the loop. It surfaces exactly what a Vulkan host needs: the instance
     * extensions to enable, a way to create a surface once the instance exists, the
     * drawable size, and event pumping. Native OS/library types cross the seam as
     * opaque handles (@c void* / @c std::uint64_t) so this header stays neutral.
     */
    class IPlatformWindow
    {
        public:
            /** @brief Callback receiving each raw platform event (an opaque SDL_Event*). */
            using EventHandler = std::function<void(const void*)>;

            virtual ~IPlatformWindow() = default;

            /**
             * @brief Drains pending OS events, forwarding each to the event handler.
             * @return false once the user has asked to close the window.
             */
            virtual bool pump_events() = 0;

            /**
             * @brief Registers a sink for raw platform events (used by the ImGui backend).
             * @param handler Called once per event with an opaque native event pointer.
             */
            virtual void set_event_handler(EventHandler handler) = 0;

            /**
             * @brief The current drawable size in pixels.
             * @param width  Receives the width.
             * @param height Receives the height.
             */
            virtual void drawable_size(std::uint32_t& width, std::uint32_t& height) const = 0;

            /** @brief Instance extensions the window system needs for a Vulkan surface. */
            virtual std::vector<std::string> vulkan_instance_extensions() const = 0;

            /**
             * @brief Creates a Vulkan surface on this window for @p instance.
             * @param instance The VkInstance (as an integer) to create the surface on.
             * @return The created VkSurfaceKHR (as an integer), or 0 on failure.
             */
            virtual std::uint64_t create_vulkan_surface(std::uint64_t instance) const = 0;

            /** @brief The native window handle (an SDL_Window*), for backend init. */
            virtual void* native_handle() const = 0;
    };
} // namespace sushi::editor

#endif
