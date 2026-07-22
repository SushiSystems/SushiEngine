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
#include <SushiEngine/sim/simulation.hpp>

#include "../gizmo/gizmo_controller.hpp"
#include "imgui_backend.hpp"
#include "../camera/scene_camera.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief One UI element to draw as a 2D overlay on top of a viewport.
         *
         * A flattened UI tree node: its authored `params` plus the index of its UI
         * parent in the same array (or -1 when it anchors directly to the viewport).
         * The panel resolves each element's pixel rect against its parent's — a
         * top-left, y-down variant of Unity's uGUI RectTransform math — and paints it
         * with ImGui's draw list over the rendered image, so canvases and buttons are
         * visible while authoring without a dedicated Vulkan 2D pass.
         */
        struct UIOverlayElement
        {
            int parent = -1;                            /**< Index of the UI parent, or -1 for viewport-anchored. */
            std::uint32_t id = 0;                       /**< The owning entity id (for picking; 0 = none). */
            SushiEngine::Simulation::UIElementParams params; /**< The authored rect and paint. */
            bool selected = false;                      /**< Whether to draw a selection outline and handles. */
        };

        /**
         * @brief The UI layer drawn over a viewport, plus its interaction in/out.
         *
         * In edit mode the overlay is drawn translucent (so the 3D scene shows through
         * a canvas rather than being covered) and is interactive: clicking an element
         * picks it, dragging its body moves it, and dragging a corner handle resizes it,
         * writing the change back into `elements[edited_index].params`. In play mode
         * (the Game view) it is drawn solid and non-interactive — the runtime look.
         */
        struct UIOverlay
        {
            UIOverlayElement* elements = nullptr; /**< The flattened UI tree (mutated by drags). */
            std::size_t count = 0;                /**< Number of elements. */
            bool edit_mode = false;               /**< Translucent + interactive (Scene) vs solid (Game). */
            std::uint32_t selected_id = 0;        /**< In: the selected UI entity (drives handles). */
            std::uint32_t picked_id = 0;          /**< Out: UI entity clicked this frame (0 = none). */
            bool consumed_click = false;          /**< Out: a UI pick/drag consumed the left click. */
            int edited_index = -1;                /**< Out: element whose params changed this frame. */
        };

        /**
         * @brief A Unity-style viewport panel: a Vulkan 3D view from an injected camera.
         *
         * Owns an offscreen scene view and displays it; the camera it renders from is
         * supplied by reference (dependency injection), so the same panel serves the
         * Scene view (a navigable fly camera) and the Game view (the world's camera).
         * Each frame it sizes the target to the panel, feeds navigation input to the
         * camera while the panel is interacted with (only if the camera is navigable),
         * renders the given mesh instances, and displays the result with ImGui::Image.
         * The offscreen colour target is registered with the ImGui backend as a texture,
         * re-registered on resize.
         */
        /**
         * @brief A viewport's display-selection control: the choices and the current pick.
         *
         * The Game view can host two or more cameras on different displays; this lets the
         * panel offer a combo to pick which display's resolved camera it shows, so the
         * cameras do not conflict. The host owns the storage; the panel only reads the
         * options and writes the chosen display back.
         */
        struct DisplaySelector
        {
            const std::uint32_t* displays = nullptr; /**< Available display indices. */
            std::size_t count = 0;                   /**< Number of options. */
            std::uint32_t* selected = nullptr;       /**< The chosen display, written on change. */
        };

        class ViewportPanel
        {
            public:
                /**
                 * @brief Creates the scene view and registers its textures with ImGui.
                 * @param renderer The window renderer that owns the device.
                 * @param imgui    The ImGui backend used to register sampled textures.
                 * @param title    The panel window title (e.g. "Scene" or "Game").
                 * @param camera   The camera this panel renders from; must outlive the panel.
                 */
                ViewportPanel(SushiEngine::Render::IWindowRenderer& renderer, ImGuiBackend& imgui,
                              const char* title, ISceneCamera& camera);
                ~ViewportPanel();

                ViewportPanel(const ViewportPanel&) = delete;
                ViewportPanel& operator=(const ViewportPanel&) = delete;

                /**
                 * @brief Draws the panel and renders the scene into it.
                 *
                 * The camera is driven only while the panel is interacted with, so input
                 * over other panels never moves the view.
                 *
                 * @param open        Visibility flag, bound to the panel's close button.
                 * @param instances   The mesh instances to draw this frame.
                 * @param count       Number of instances.
                 * @param selected_id The highlighted instance id; updated when the user
                 *                    left-clicks the viewport to pick (0 clears it).
                 * @param pickable    Whether a left-click picks an entity. The Scene view
                 *                    picks; the Game view passes false so clicking it never
                 *                    selects (the game is played, not authored).
                 * @param gizmo_target When non-null, the transform gizmo is drawn at this
                 *                    transform and a drag edits it in place. Null draws no
                 *                    gizmo. The return value reports whether it changed.
                 * @param gizmo_mode  Which handle set to draw (translate/rotate/scale).
                 * @param gizmo_space Local or World axis frame for the gizmo drag.
                 * @param gizmo_snap  Optional snapping applied to a gizmo drag.
                 * @param display     When non-null, a display-selection combo is drawn over
                 *                    the viewport (used by the Game view to choose which
                 *                    display's camera it shows). Null draws no combo.
                 * @param strands       Soft-body wireframes to draw this frame, or nullptr.
                 * @param strand_count  Number of entries in @p strands.
                 * @param lights        Punctual lights to shade with this frame, or nullptr.
                 * @param light_count   Number of entries in @p lights.
                 * @return Whether the gizmo edited @p gizmo_target this frame.
                 */
                bool draw(bool& open, const SushiEngine::Render::MeshInstance* instances,
                          std::size_t count, const SushiEngine::Render::Environment& environment,
                          std::uint32_t& selected_id, bool pickable = true,
                          SushiEngine::Simulation::EntityTransform* gizmo_target = nullptr,
                          GizmoMode gizmo_mode = GizmoMode::Translate,
                          GizmoSpace gizmo_space = GizmoSpace::World,
                          const GizmoSnap* gizmo_snap = nullptr,
                          const DisplaySelector* display = nullptr,
                          const SushiEngine::Render::ClothStrandView* strands = nullptr,
                          std::size_t strand_count = 0,
                          const SushiEngine::Render::PunctualLight* lights = nullptr,
                          std::size_t light_count = 0,
                          const SushiEngine::Render::Decal* decals = nullptr,
                          std::size_t decal_count = 0, UIOverlay* ui = nullptr);

                /**
                 * @brief Applies the host's fidelity/performance settings to this view.
                 *
                 * Per panel rather than global because the two viewports converge their
                 * temporal history independently and may be sized very differently; the
                 * host still passes both the same values today.
                 *
                 * @param settings The requested quality, anti-aliasing, and scaling.
                 */
                void set_render_settings(const SushiEngine::Render::RenderSettings& settings);

                /**
                 * @brief The internal resolution the last frame was rendered at.
                 * @param width  Receives the internal render width in pixels.
                 * @param height Receives the internal render height in pixels.
                 */
                void render_resolution(std::uint32_t& width,
                                       std::uint32_t& height) const noexcept;

                /** @brief Whether this panel's gizmo currently has a handle grabbed. */
                bool gizmo_dragging() const noexcept { return gizmo_.dragging(); }

                /** @brief Whether a UI element is currently being dragged (moved or resized). */
                bool ui_dragging() const noexcept { return ui_drag_index_ >= 0; }

                /** @brief The panel's current pixel width, as last sized by `draw`. */
                std::uint32_t target_width() const noexcept { return view_->width(); }

                /** @brief The panel's current pixel height, as last sized by `draw`. */
                std::uint32_t target_height() const noexcept { return view_->height(); }

                /** @brief The panel window title this panel was created with. */
                const char* title() const noexcept { return title_; }

                /**
                 * @brief Number of per-pass GPU timings from the last resolved frame.
                 *
                 * Zero until a timed submit has completed, and zero for the whole run on
                 * a device without timestamp queries.
                 *
                 * @return The number of timings `pass_timing` can be asked for.
                 */
                std::size_t pass_timing_count() const noexcept
                {
                    return view_->pass_timing_count();
                }

                /**
                 * @brief One pass's GPU time from the most recently resolved frame.
                 * @param index Timing index in [0, pass_timing_count()).
                 * @return The pass's name and measured milliseconds; the name points at
                 *         storage the scene view owns and is valid until its next render.
                 */
                SushiEngine::Render::ScenePassTiming pass_timing(std::size_t index) const noexcept
                {
                    return view_->pass_timing(index);
                }

            private:
                void resize_to(std::uint32_t width, std::uint32_t height);
                void register_textures();
                void unregister_textures();

                ImGuiBackend& imgui_;
                const char* title_;
                ISceneCamera& camera_;
                std::unique_ptr<SushiEngine::Render::ISceneView> view_;
                std::vector<ImTextureID> slot_textures_;
                bool looking_ = false;
                bool panning_ = false;
                // Active UI drag: the element index being dragged and which handle grabbed
                // (0 = body/move, 1..4 = corner resize), or -1 when no drag is in progress.
                int ui_drag_index_ = -1;
                int ui_drag_handle_ = -1;
                GizmoController gizmo_;
        };
    } // namespace Editor
} // namespace SushiEngine

#endif
