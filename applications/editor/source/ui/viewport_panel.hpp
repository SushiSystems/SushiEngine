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

#include <SushiEngine/authoring/game_view_settings.hpp>
#include <SushiEngine/authoring/soft_body_heat.hpp>
#include <SushiEngine/render/scene_view.hpp>
#include <SushiEngine/render/window_renderer.hpp>
#include <SushiEngine/simulation/simulation.hpp>

#include "../animation/animated_mesh_preview.hpp"
#include "../animation/skeleton_debug_draw.hpp"
#include "../physics/physics_overlay.hpp"
#include "../vfx/effect_preview.hpp"
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
         * A flattened UI tree node: its authored `parameters` plus the index of its UI
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
            /** @brief The authored rect and paint. */
            SushiEngine::Simulation::UIElementParameters parameters;
            bool selected = false;                      /**< Whether to draw a selection outline and handles. */
        };

        /**
         * @brief The UI layer drawn over a viewport, plus its interaction in/out.
         *
         * In edit mode the overlay is drawn translucent (so the 3D scene shows through
         * a canvas rather than being covered) and is interactive: clicking an element
         * picks it, dragging its body moves it, and dragging a corner handle resizes it,
         * writing the change back into `elements[edited_index].parameters`. In play mode
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
            int edited_index = -1;                /**< Out: element whose values changed here. */
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

        /**
         * @brief Everything a viewport draws in one frame, as one object.
         *
         * The three surfaces — Scene, Game, Preview — differ from each other in about a
         * dozen of these fields and agree on the rest, and every one of them is optional:
         * a viewport with no world content, no gizmo and no overlay is a legitimate frame.
         * Passed as a struct because as a parameter list this was thirty-one positional
         * arguments, where the compiler could not tell a misplaced `nullptr` from an
         * intended one and a reader could not tell which `false` was which.
         *
         * Defaults describe the emptiest honest frame: nothing to draw, nothing to edit.
         * A caller sets only the fields its surface actually has.
         */
        struct ViewportFrameInputs
        {
            /** The mesh instances to draw, and how many. */
            const SushiEngine::Render::MeshInstance* instances = nullptr;
            std::size_t instance_count = 0;

            /**
             * Whether a left-click picks an entity. The Scene view picks; the Game view
             * does not — the game is played, not authored.
             */
            bool pickable = true;

            /**
             * When non-null, the transform gizmo is drawn at this transform and a drag
             * edits it in place; `draw` reports whether it changed.
             */
            SushiEngine::Simulation::EntityTransform* gizmo_target = nullptr;
            GizmoMode gizmo_mode = GizmoMode::Translate;  /**< Which handle set to draw. */
            GizmoSpace gizmo_space = GizmoSpace::World;   /**< Local or world axis frame. */
            const GizmoSnap* gizmo_snap = nullptr;        /**< Optional drag snapping. */

            /**
             * When non-null, a display-selection combo is drawn over the viewport, so the
             * Game view can choose which display's resolved camera it shows.
             */
            const DisplaySelector* display = nullptr;

            /** Soft-body wireframes to draw, and how many. */
            const SushiEngine::Render::DeformableMeshView* deformable = nullptr;
            std::size_t deformable_count = 0;

            /** Punctual lights to shade with, and how many. */
            const SushiEngine::Render::PunctualLight* lights = nullptr;
            std::size_t light_count = 0;

            /** Decals to project, and how many. */
            const SushiEngine::Render::Decal* decals = nullptr;
            std::size_t decal_count = 0;

            /** The 2D UI layer painted over the rendered image, or null for none. */
            UIOverlay* ui_overlay = nullptr;

            bool show_grid = false; /**< Whether to draw the ground grid. */

            const SkeletonPreview* skeleton = nullptr; /**< Skeleton debug draw, or null. */
            bool skeleton_names = true;                /**< Label the skeleton's joints. */

            /**
             * @brief The selected cooked collider as line segments, or null (§14).
             *
             * Six floats per segment, in the asset's frame. Segments rather than an asset,
             * because building them means rebuilding hull faces from the stored point set and
             * that is not a per-frame cost; `CookBakeState` rebuilds them on selection.
             */
            const std::vector<float>* collision_wireframe = nullptr;

            /**
             * @brief The selected soft body's interior, for §9.3/§9.4's debug views (P6-G5).
             *
             * Read live off the simulated body each frame rather than cached, unlike the
             * collision wireframe above: this changes every tick because it *is* the
             * simulation, so caching it would be caching the thing the view exists to
             * watch. Null or empty draws nothing, as does a @ref soft_body_view of
             * @c SoftBodyDebugView::Off.
             */
            const std::vector<SushiEngine::Vector3>* soft_body_positions = nullptr;
            const std::vector<SushiEngine::Simulation::SoftBodyElementSample>* soft_body_elements =
                nullptr;
            /** The selected body's material, which the heat scales normalize against. */
            SushiEngine::Physics::SoftBodyMaterialT<SushiEngine::Scalar> soft_body_material{};
            SoftBodyDebugView soft_body_view = SoftBodyDebugView::Off;

            /**
             * @brief Which §14 physics debug categories to draw, and the world to read them from.
             *
             * The world rather than a snapshot, because every category here is live: a
             * contact list, an island partition and a sleep flag are all *this tick's*, and
             * a copy taken for the overlay would be a second thing to keep in step with the
             * simulation that already has them. Null draws nothing, which is the Game view.
             */
            Simulation::IWorldEditor* physics_world = nullptr;
            PhysicsOverlaySettings physics_overlay{};

            /**
             * @brief The entity whose joint the gizmo is drawn for.
             *
             * Passed in rather than read from `selected_id` beside it: that one is the
             * *picking* channel, a 32-bit id the renderer writes back, and an `EntityId` is
             * 64 bits. Narrowing one into the other to save a field is how a scene past four
             * billion entities draws the wrong gizmo.
             */
            Simulation::EntityId selected_entity = Simulation::NULL_ENTITY;

            /** The isolated effect preview drawn in this viewport, or null. */
            EffectPreview* particle_preview = nullptr;

            /** Particle billboards to draw, and how many. */
            const SushiEngine::Render::ParticleBillboard* billboards = nullptr;
            std::size_t billboard_count = 0;

            /** A previewed skinned character to GPU-skin and draw, or null. */
            AnimatedMeshPreview* animated_mesh = nullptr;

            /** GPU particle emitters to simulate and draw, and how many. */
            const SushiEngine::Render::ParticleEmitterView* emitters = nullptr;
            std::size_t emitter_count = 0;

            /**
             * Whether to draw and let the user drag @c animated_mesh's two-bone IK target.
             * The same authored-here/played-there split as @c gizmo_target and @c pickable.
             */
            bool ik_gizmo = false;

            /**
             * Whether to draw the playback transport for whatever this surface previews.
             * Set on the Preview panel — the one surface anything being authored shows on,
             * so its transport belongs with it rather than in a window per kind of subject.
             */
            bool preview_controls = false;

            /**
             * When non-null, draws the Game view's aspect/orientation/fullscreen toolbar and
             * constrains the image to the chosen aspect, letterboxed and centred. Setting
             * fullscreen undocks the panel to cover the whole editor viewport (Unity's
             * "Maximize on Play"). Null draws no toolbar and fills the panel exactly.
             */
            GameViewSettings* game_view = nullptr;

            /**
             * Crowd characters extracted from the live world this frame, concatenated with
             * @c animated_mesh's own preview instance — the same "world content plus
             * whatever is being authored" merge the billboards and emitters already do.
             */
            const SushiEngine::Render::SkinnedInstance* scene_skinned = nullptr;
            std::size_t scene_skinned_count = 0;
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
                 * @param environment The sky, sun and atmosphere this frame is lit by.
                 * @param selected_id The highlighted instance id; updated when the user
                 *                    left-clicks the viewport to pick (0 clears it).
                 * @param inputs      Everything else this frame draws — see
                 *                    @ref ViewportFrameInputs.
                 * @return Whether the gizmo edited @c inputs.gizmo_target this frame.
                 */
                bool draw(bool& open, const SushiEngine::Render::Environment& environment,
                          std::uint32_t& selected_id, const ViewportFrameInputs& inputs);

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
                 * `Render::UIView` is non-owning, so the list it points at has to outlive the
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
