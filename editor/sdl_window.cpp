/**************************************************************************/
/* sdl_window.cpp                                                         */
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

#include "sdl_window.hpp"

#include <stdexcept>
#include <string>

#include <vulkan/vulkan.h>

#include <SDL.h>
#include <SDL_vulkan.h>

namespace sushi::editor
{
    SdlWindow::SdlWindow(const char* title, int width, int height)
    {
        if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0)
            throw std::runtime_error(std::string("SDL_Init failed: ") + SDL_GetError());

        const auto flags = static_cast<SDL_WindowFlags>(
            SDL_WINDOW_VULKAN | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
        window_ = SDL_CreateWindow(title, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                                   width, height, flags);
        if (window_ == nullptr)
        {
            const std::string error = SDL_GetError();
            SDL_Quit();
            throw std::runtime_error("SDL_CreateWindow failed: " + error);
        }
    }

    SdlWindow::~SdlWindow()
    {
        if (window_ != nullptr)
            SDL_DestroyWindow(window_);
        SDL_Quit();
    }

    bool SdlWindow::pump_events()
    {
        bool keep_running = true;
        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            if (handler_)
                handler_(&event);
            if (event.type == SDL_QUIT)
                keep_running = false;
            if (event.type == SDL_WINDOWEVENT &&
                event.window.event == SDL_WINDOWEVENT_CLOSE &&
                event.window.windowID == SDL_GetWindowID(window_))
                keep_running = false;
        }
        return keep_running;
    }

    void SdlWindow::drawable_size(std::uint32_t& width, std::uint32_t& height) const
    {
        int w = 0;
        int h = 0;
        SDL_Vulkan_GetDrawableSize(window_, &w, &h);
        width = static_cast<std::uint32_t>(w < 0 ? 0 : w);
        height = static_cast<std::uint32_t>(h < 0 ? 0 : h);
    }

    std::vector<std::string> SdlWindow::vulkan_instance_extensions() const
    {
        unsigned int count = 0;
        if (SDL_Vulkan_GetInstanceExtensions(window_, &count, nullptr) == SDL_FALSE)
            throw std::runtime_error(std::string("SDL_Vulkan_GetInstanceExtensions failed: ") +
                                     SDL_GetError());
        std::vector<const char*> names(count);
        if (SDL_Vulkan_GetInstanceExtensions(window_, &count, names.data()) == SDL_FALSE)
            throw std::runtime_error(std::string("SDL_Vulkan_GetInstanceExtensions failed: ") +
                                     SDL_GetError());
        return std::vector<std::string>(names.begin(), names.end());
    }

    std::uint64_t SdlWindow::create_vulkan_surface(std::uint64_t instance) const
    {
        VkSurfaceKHR surface = VK_NULL_HANDLE;
        if (SDL_Vulkan_CreateSurface(window_, reinterpret_cast<VkInstance>(instance),
                                     &surface) == SDL_FALSE)
            throw std::runtime_error(std::string("SDL_Vulkan_CreateSurface failed: ") +
                                     SDL_GetError());
        return reinterpret_cast<std::uint64_t>(surface);
    }
} // namespace sushi::editor
