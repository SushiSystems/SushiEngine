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

#include "../animation/animated_mesh_preview.hpp"
#include "../animation/skeleton_debug_draw.hpp"
#include "../vfx/effect_preview.hpp"
#include "../gizmo/gizmo_controller.hpp"
#include "imgui_backend.hpp"
#include "../camera/scene_camera.hpp"
#include "../core/game_view_settings.hpp"

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
                 * @param game_view   When non-null, draws the Game view's aspect/orientation/
                 *                    fullscreen toolbar and constrains the rendered image to
                 *                    the chosen aspect (letterboxed/pillarboxed, centered).
                 *                    Setting fullscreen undocks the panel and expands it to
                 *                    cover the whole editor viewport (Unity's "Maximize on
                 *                    Play"). Null draws no toolbar and fills the panel
                 *                    exactly, as before (the Scene/Preview views).
                 * @param strands       Soft-body wireframes to draw this frame, or nullptr.
                 * @param strand_count  Number of entries in @p strands.
                 * @param lights        Punctual lights to shade with this frame, or nullptr.
                 * @param light_count   Number of entries in @p lights.
                 * @param animated_mesh A previewed skinned character to GPU-skin and draw this
                 *                    frame, or nullptr for none.
                 * @param preview_controls Whether to draw the playback row for whatever this
                 *     surface is previewing. Set on the Preview panel, which is the one surface
                 *     anything being authored is shown on, so its transport belongs with it
                 *     rather than in a window per kind of subject.
                 * @param ik_gizmo    Whether to draw and let the user drag @p animated_mesh's
                 *                    two-bone IK target in this viewport (design §12.1's IK
                 *                    gizmo). The Scene view passes true; the Game view passes
                 *                    false — the same "authored here, played there" split as
                 *                    @p gizmo_target/@p pickable.
                 * @param scene_skinned Crowd characters extracted from the live world this
                 *                    frame (design §12.3/§12.4's `RenderScene::skinned_instances`),
                 *                    concatenated with @p animated_mesh's own preview instance —
                 *                    the same "world content plus whatever's being authored"
                 *                    merge @p billboards/@p emitters already do for their kinds.
                 * @param scene_skinned_count Number of entries in @p scene_skinned.
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
                          std::size_t decal_count = 0, UIOverlay* ui = nullptr,
                          bool show_grid = false, const SkeletonPreview* skeleton = nullptr,
                          bool skeleton_names = true,
                          EffectPreview* particle_preview = nullptr,
                          const SushiEngine::Render::ParticleBillboard* billboards = nullptr,
                          std::size_t billboard_count = 0,
                          AnimatedMeshPreview* animated_mesh = nullptr,
                          const SushiEngine::Render::ParticleEmitterView* emitters = nullptr,
                          std::size_t emitter_count = 0, bool ik_gizmo = false,
                          bool preview_controls = false, GameViewSettings* game_view = nullptr,
                          const SushiEngine::Render::SkinnedInstance* scene_skinned = nullptr,
                          std::size_t scene_skinned_count = 0);

                /**
                 * @brief Draws this panel as the Game view's "no cameras rendering" placeholder.
                 *
                 * Same window, same toolbar, same fullscreen behaviour as @ref draw — just a
                 * black fill with a centred message instead of a rendered scene, the same
                 * affordance Unity's Game view gives when nothing renders through it. Sharing
                 * the panel object (and its fullscreen state machine) with the rendering path
                 * is the point: the old free-function fallback tracked fullscreen in its own
                 * statics, and the two copies could disagree about the dock slot to restore.
                 *
                 * @param open     Visibility flag, bound to the panel's close button.
                 * @param settings The Game view toolbar state this draws and edits in place.
                 */
                void draw_no_camera(bool& open, GameViewSettings& settings);

                /**
                 * @brief Requests (or releases) fullscreen for this panel.
                 *
                 * The same undock-and-cover-the-editor maximize the Game view's toolbar
                 * checkbox drives, exposed as a plain request so any viewport can be
                 * maximized — the Scene view binds it to Shift+Space (Unity's maximize).
                 * Applied by the next @ref draw through the one fullscreen state
                 * machine, so the dock slot is remembered and restored the same way.
                 *
                 * @param enabled Whether this panel should cover the editor viewport.
                 */
                void set_fullscreen(bool enabled) noexcept { fullscreen_requested_ = enabled; }

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
                 * @brief The GPU cull's counts from this view's last resolved frame.
                 * @param drawn  Receives the instances that survived and were drawn.
                 * @param tested Receives the instances the cull considered (both zero on
                 *     the classic path or before the first GPU-driven frame).
                 */
                void cull_statistics(std::uint32_t& drawn, std::uint32_t& tested) const noexcept
                {
                    view_->cull_statistics(drawn, tested);
                }

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

                /**
                 * @brief Debounces a live resize: rebuilds the target only once the
                 * requested size has held still for @ref RESIZE_SETTLE_FRAMES frames.
                 *
                 * A rebuild is a device idle plus a temporal-history reset; per drag
                 * frame that is a black, hitching viewport. Until it settles, the old
                 * image stretches into the new rect.
                 */
                void request_resize(std::uint32_t width, std::uint32_t height);

                void register_textures();
                void unregister_textures();

                /**
                 * @brief Advances the fullscreen state machine for this frame's window.
                 *
                 * On entering fullscreen, remembers the dock slot and forces the next window
                 * undocked and viewport-sized; on leaving, redocks it into the remembered
                 * slot. Shared by @ref draw and @ref draw_no_camera so both paths agree on
                 * one @c was_fullscreen_ / @c saved_dock_id_ pair. Must be called before the
                 * window's `ImGui::Begin`; the returned flags belong to that `Begin`.
                 *
                 * @param want_fullscreen Whether the Game view settings request fullscreen.
                 * @return The extra window flags fullscreen imposes (none when off).
                 */
                ImGuiWindowFlags apply_fullscreen_transition(bool want_fullscreen);

                ImGuiBackend& imgui_;
                const char* title_;
                ISceneCamera& camera_;
                std::unique_ptr<SushiEngine::Render::ISceneView> view_;
                std::vector<ImTextureID> slot_textures_;
                /**
                 * @brief The frame's UI geometry, kept alive across the render call.
                 *
                 * `Render::UiView` is non-owning, so the list it points at has to outlive the
                 * call. Holding it here rather than on the stack also keeps its capacity between
                 * frames, which matters because it is rebuilt from scratch every one.
                 */
                SushiEngine::UI::UIDrawList ui_draw_list_;
                /**
                 * @brief Where the viewport image sat last frame, in screen pixels.
                 *
                 * The UI draw list is built before this frame's image position is known, and the
                 * only thing that needs it is converting the pointer into viewport-local space
                 * for a button's hover shade.
                 */
                ImVec2 last_image_origin_{0.0f, 0.0f};
                bool looking_ = false;
                bool panning_ = false;
                // Active UI drag: the element index being dragged and which handle grabbed
                // (0 = body/move, 1..4 = corner resize), or -1 when no drag is in progress.
                int ui_drag_index_ = -1;
                int ui_drag_handle_ = -1;
                // Fullscreen (Game view "Maximize on Play"): whether the panel is currently
                // forced undocked and full-viewport, and the dock id to restore on exit.
                bool was_fullscreen_ = false;
                unsigned int saved_dock_id_ = 0;
                // A host-side maximize request (set_fullscreen), OR'd with the Game view
                // toolbar's own checkbox by draw().
                bool fullscreen_requested_ = false;
                // The debounced-resize state: the size the panel currently wants and how
                // many consecutive frames it has wanted exactly that.
                std::uint32_t pending_width_ = 0;
                std::uint32_t pending_height_ = 0;
                std::uint32_t pending_stable_frames_ = 0;
                // ~100 ms at 60 fps: long enough to skip every intermediate drag size,
                // short enough that the sharp rebuild lands as the mouse settles.
                static constexpr std::uint32_t RESIZE_SETTLE_FRAMES = 6;
                GizmoController gizmo_;
                // A second, independent GizmoController for the animated-mesh IK target (design
                // §12.1) — its own drag state, separate from the selection gizmo above, since
                // the two can be manipulated in the same frame by different UI elements.
                GizmoController ik_gizmo_;
        };
    } // namespace Editor
} // namespace SushiEngine

#endif
