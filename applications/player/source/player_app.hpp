/**************************************************************************/
/* player_app.hpp                                                        */
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

#ifndef SUSHIENGINE_PLAYER_PLAYER_APP_HPP
#define SUSHIENGINE_PLAYER_PLAYER_APP_HPP

/**
 * @file player_app.hpp
 * @brief The ImGui-free runtime shell: a window, a live world, and a presented frame.
 *
 * PLATFORM0 S5's skeleton — the first real caller of
 * `IWindowRenderer::present_scene_view()` (S4). Where the editor's `main.cpp` copies a
 * `RenderScene` into panel-drawn viewports wrapped in a dockspace, `PlayerApp` copies
 * the same snapshot straight onto the window: no picking, no gizmos, no undo, no
 * authoring panels — those are editor concerns and stay in `editor/`, which this
 * deliberately does not link (`applications/player/CMakeLists.txt` links no `sushiengine_imgui`).
 *
 * `start()`/`frame()`/`suspend()`/`resume()`/`shutdown()` are separate calls rather
 * than a constructor-owns-everything object because the shape assumes no particular
 * loop: `applications/player/source/main.cpp`'s desktop `while(!app.should_quit())` is one driver, and a
 * future mobile host calling `frame()` from its own OS-owned callback is another.
 * `suspend()`/`resume()` are real behavior, not stubs — they release and rebuild the
 * swapchain-owning renderer and scene view, which is what a host must do around a
 * lost/reacquired drawing surface (this desktop build wires it to SDL's minimize/
 * restore window events; a future mobile host wires the same two calls to its own
 * lifecycle callbacks).
 *
 * Deliberately out of scope for this skeleton (tracked, not silently missing):
 * headless operation (PLATFORM0 S6 — `IWindowRenderer`'s current construction path
 * still requires a real surface), the UI overlay channel (`Render::UIView`; a scene's
 * Canvas/Button entities do not draw yet), and any audio playback.
 */

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <SushiEngine/input/input_manager.hpp>
#include <SushiEngine/render/deformable_mesh.hpp>
#include <SushiEngine/render/scene_view.hpp>
#include <SushiEngine/render/window_renderer.hpp>
#include <SushiEngine/simulation/simulation.hpp>

#include "sdl/sdl_input_translator.hpp"
#include "sdl_window.hpp"

namespace SushiEngine
{
    namespace Player
    {
        /**
         * @brief Owns the window, the renderer, the live world, and drives one frame
         *        of each at a time.
         *
         * Non-copyable: it owns Vulkan-backed resources through `IWindowRenderer`/
         * `ISceneView` and an SDL window.
         */
        class PlayerApp
        {
            public:
                /** @brief What `start()` needs to bring the player up. */
                struct Desc
                {
                    /** @brief A `.sushiscene` to load, or empty to start with an empty world. */
                    std::string scene_path;
                    std::string window_title = "SushiEngine Player"; /**< Window title bar text. */
                    std::uint32_t width = 1280;  /**< Initial window width in pixels. */
                    std::uint32_t height = 720;  /**< Initial window height in pixels. */
                    /** @brief Enables Vulkan validation layers (costs real frame time). */
                    bool enable_validation = false;
                    /**
                     * @brief No window, no input, no swapchain, no present (PLATFORM0 S6).
                     *
                     * For a display-less host — a CI runner with no window/compositor. `start()`
                     * never constructs a window or an input manager; `frame()` never pumps
                     * events and never calls `present_scene_view()`/`end_frame()` (there is no
                     * swapchain image to write into). The world still ticks and the scene view
                     * still renders every call, so a caller that wants the pixels reads them
                     * back through `ISceneView::read_output()` on the view `scene_view()`
                     * exposes — the same seam the RHI0 golden harness reads through.
                     */
                    bool headless = false;
                    /** @brief Forms the outer path component of the per-user data directory. */
                    std::string organization = "SushiSystems";
                    /** @brief Forms the inner path component of the per-user data directory. */
                    std::string application = "SushiEnginePlayer";
                };

                PlayerApp();
                ~PlayerApp();

                PlayerApp(const PlayerApp&) = delete;
                PlayerApp& operator=(const PlayerApp&) = delete;

                /**
                 * @brief Opens the window, brings up the renderer, and loads @p desc's scene.
                 *
                 * Called once before the first `frame()`. Throws `std::runtime_error` if the
                 * window, device, or a named scene file cannot be brought up — a player told
                 * to load a specific scene that then shows an empty world is a worse failure
                 * than one that refuses to start.
                 *
                 * @param desc The launch configuration.
                 */
                void start(const Desc& desc);

                /**
                 * @brief Pumps window events, advances the world, and presents one frame.
                 *
                 * A no-op beyond event pumping while suspended (see @ref suspend) — there is
                 * no swapchain to draw into, and a backgrounded host should not be spending
                 * simulation time on a world nobody can see.
                 *
                 * @param real_delta_seconds Wall-clock time since the last call, in seconds.
                 */
                void frame(double real_delta_seconds);

                /**
                 * @brief Releases the swapchain-owning renderer and scene view.
                 *
                 * The world (the `ISimulation`) is untouched — only the drawing surface is
                 * given up. Idempotent: a second call while already suspended does nothing.
                 */
                void suspend();

                /**
                 * @brief Rebuilds the renderer and scene view `suspend()` released.
                 *
                 * Idempotent: a call while not suspended does nothing.
                 */
                void resume();

                /** @brief Idles the device and releases every owned resource, in order. */
                void shutdown();

                /** @brief Whether the host's frame loop should stop calling @ref frame. */
                bool should_quit() const noexcept { return quit_requested_; }

                /**
                 * @brief The offscreen view `frame()` renders into, or null before `start()`.
                 *
                 * A headless caller's read-back seam: `frame()` always renders (headless or
                 * not), and `Render::ISceneView::read_output(current_slot(), image)` after a
                 * call is how a CI harness gets the pixels with no window to display them in.
                 */
                Render::ISceneView* scene_view() noexcept { return scene_view_.get(); }

            private:
                /** @brief Builds `renderer_`/`scene_view_` from `desc_`; used by start() and resume(). */
                void create_render_resources();

                /** @brief The window's minimize/restore events, routed to suspend()/resume(). */
                void handle_window_event(const void* native_event);

                Desc desc_;
                std::unique_ptr<Platform::SDLWindow> window_;
                std::unique_ptr<Input::InputManager> input_;
                std::unique_ptr<Input::SDLInputTranslator> input_translator_;
                std::unique_ptr<Simulation::ISimulation> simulation_;
                std::unique_ptr<Render::IWindowRenderer> renderer_;
                std::unique_ptr<Render::ISceneView> scene_view_;

                // Per-frame extract scratch: RenderScene's channels that need a shape
                // conversion before ISceneView::render() can take them (see player_app.cpp's
                // comment at the call site for exactly which channels do and do not).
                std::vector<Render::MeshInstance> instances_;
                std::vector<Render::DeformableMeshView> deformable_;
                std::vector<Render::ParticleBillboard> particle_billboards_;

                bool suspended_ = false;
                bool quit_requested_ = false;
        };
    } // namespace Player
} // namespace SushiEngine

#endif
