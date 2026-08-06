/**************************************************************************/
/* viewport_panel.cpp                                                     */
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

#include "viewport_panel.hpp"

#include <algorithm>
#include <cstdio>
#include <vector>

#include <imgui_internal.h>

#include <SushiEngine/ui/layout.hpp>

#include <filesystem>
#include <string>

#include "../physics/collision_overlay.hpp"
#include "../physics/soft_body_overlay.hpp"
#include "game_view_toolbar.hpp"
#include "panel_widgets.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        namespace
        {
            /** @brief Transforms a point by an affine matrix (w = 1). */
            SushiEngine::Vector3 transform_point(const SushiEngine::Matrix4& matrix,
                                                 const SushiEngine::Vector3& p)
            {
                const SushiEngine::Scalar* m = matrix.m;
                return SushiEngine::Vector3{m[0] * p.x + m[4] * p.y + m[8] * p.z + m[12],
                                            m[1] * p.x + m[5] * p.y + m[9] * p.z + m[13],
                                            m[2] * p.x + m[6] * p.y + m[10] * p.z + m[14]};
            }
        } // namespace

        namespace
        {
            /** @brief Builds the `UI::RectTransform` an authored UI element's parameters mean. */
            SushiEngine::UI::RectTransform ui_rect_transform(
                const SushiEngine::Simulation::UIElementParameters& p) noexcept
            {
                using SushiEngine::UI::Vector2;
                SushiEngine::UI::RectTransform transform;
                transform.anchor_min = Vector2{p.anchor_min_x, p.anchor_min_y};
                transform.anchor_max = Vector2{p.anchor_max_x, p.anchor_max_y};
                transform.pivot = Vector2{p.pivot_x, p.pivot_y};
                transform.anchored_position = Vector2{p.position_x, p.position_y};
                transform.size_delta = Vector2{p.size_x, p.size_y};
                return transform;
            }

            /** @brief Converts a `SushiEngine::UI::Rect` to the panel's `ImVec4` (min_x, min_y, w, h). */
            ImVec4 to_im_vec4(const SushiEngine::UI::Rect& rect) noexcept
            {
                return ImVec4(static_cast<float>(rect.min.x), static_cast<float>(rect.min.y),
                              static_cast<float>(rect.size.x), static_cast<float>(rect.size.y));
            }

            /** @brief Converts an `ImVec4` (min_x, min_y, w, h) to a `SushiEngine::UI::Rect`. */
            SushiEngine::UI::Rect to_ui_rect(const ImVec4& r) noexcept
            {
                return SushiEngine::UI::Rect{
                    SushiEngine::UI::Vector2{SushiEngine::Scalar(r.x), SushiEngine::Scalar(r.y)},
                    SushiEngine::UI::Vector2{SushiEngine::Scalar(r.z), SushiEngine::Scalar(r.w)}};
            }

            /**
             * @brief Resolves element @p i's pixel rect against its parent's, memoized.
             *
             * A `Canvas` fills its parent (the viewport); every other element resolves
             * through `SushiEngine::UI::resolve_rect` — the same anchor formula the ECS
             * `UI::` layer and the simulation's mirrored UI entities use, so the overlay
             * is a caller of the one canonical resolver rather than a second
             * implementation of its math. Recurses into the parent first so a child
             * never resolves before the rect it lays out inside; the parent chain is
             * acyclic (the simulation forbids reparent cycles), so the recursion always
             * terminates.
             *
             * @return The element's rect as (min_x, min_y, width, height).
             */
            ImVec4 resolve_ui_rect(const UIOverlayElement* ui, std::size_t count,
                                   std::vector<ImVec4>& rects, std::vector<char>& done,
                                   const ImVec4& root, int i)
            {
                if (done[static_cast<std::size_t>(i)])
                    return rects[static_cast<std::size_t>(i)];
                done[static_cast<std::size_t>(i)] = 1;

                const ImVec4 parent =
                    (ui[i].parent >= 0 && ui[i].parent < static_cast<int>(count))
                        ? resolve_ui_rect(ui, count, rects, done, root, ui[i].parent)
                        : root;

                const SushiEngine::Simulation::UIElementParameters& p = ui[i].parameters;
                const ImVec4 rect =
                    p.kind == SushiEngine::Simulation::UIElementKind::Canvas
                        ? parent
                        : to_im_vec4(SushiEngine::UI::resolve_rect(to_ui_rect(parent),
                                                                   ui_rect_transform(p)));
                rects[static_cast<std::size_t>(i)] = rect;
                return rect;
            }

            /** @brief Resolves every element's pixel rect against @p root, in one pass. */
            void compute_ui_rects(const UIOverlay& ui, const ImVec4& root,
                                  std::vector<ImVec4>& rects)
            {
                rects.assign(ui.count, ImVec4());
                std::vector<char> done(ui.count, 0);
                for (std::size_t i = 0; i < ui.count; ++i)
                    resolve_ui_rect(ui.elements, ui.count, rects, done, root, static_cast<int>(i));
            }

            /** @brief Half-size of a corner resize handle, in pixels. */
            constexpr float UI_HANDLE = 5.0f;

            /** @brief Copies a UI element's colour and opacity into a draw-list colour. */
            SushiEngine::UI::Color to_ui_color(const SushiEngine::Vector3& c, SushiEngine::Scalar a)
            {
                SushiEngine::UI::Color color;
                color.r = c.x;
                color.g = c.y;
                color.b = c.z;
                color.a = a;
                return color;
            }

            /**
             * @brief Turns the flattened overlay into the renderer's UI draw list.
             *
             * This is the seam that moves the game's interface off the editor's ImGui draw
             * list and into the renderer proper: the same authored elements, resolved into the
             * same pixel rectangles, but expressed in the renderer-agnostic form the Vulkan
             * overlay pass consumes. Coordinates are viewport-local (origin at the image's top
             * left), because that is the space the offscreen target is drawn in — the panel's
             * position on screen is an ImGui concern the renderer never sees.
             *
             * @param ui           The flattened overlay.
             * @param rects        Each element's resolved viewport-local rect.
             * @param local_mouse  Pointer position in viewport-local pixels, for button hover.
             * @param mouse_down   Whether the left button is held, for the pressed shade.
             * @param out          Receives the rects and text runs, in paint order.
             */
            void build_ui_draw_list(const UIOverlay& ui, const std::vector<ImVec4>& rects,
                                    const ImVec2& local_mouse, bool mouse_down,
                                    SushiEngine::UI::UIDrawList& out)
            {
                using Kind = SushiEngine::Simulation::UIElementKind;
                out.clear();
                for (std::size_t i = 0; i < ui.count; ++i)
                {
                    const SushiEngine::Simulation::UIElementParameters& p =
                        ui.elements[i].parameters;
                    const ImVec4 r = rects[i];
                    SushiEngine::UI::Rect rect;
                    rect.min.x = r.x;
                    rect.min.y = r.y;
                    rect.size.x = r.z;
                    rect.size.y = r.w;

                    // A canvas is a coordinate space, not a surface: it contributes no pixels of
                    // its own, which is why only the editor draws a bound for it.
                    if (p.kind == Kind::Canvas)
                        continue;

                    if (p.kind == Kind::Button)
                    {
                        const bool hovered = !ui.edit_mode && local_mouse.x >= r.x &&
                                             local_mouse.x <= r.x + r.z && local_mouse.y >= r.y &&
                                             local_mouse.y <= r.y + r.w;
                        const SushiEngine::Scalar tint =
                            hovered ? (mouse_down ? SushiEngine::Scalar(0.8)
                                                  : SushiEngine::Scalar(1.15))
                                    : SushiEngine::Scalar(1);
                        out.rects.push_back(SushiEngine::UI::UIDrawRect{
                            rect, to_ui_color(SushiEngine::Vector3{p.color.x * tint,
                                                                  p.color.y * tint,
                                                                  p.color.z * tint},
                                              p.alpha)});
                    }
                    else if (p.kind != Kind::Text)
                    {
                        out.rects.push_back(
                            SushiEngine::UI::UIDrawRect{rect, to_ui_color(p.color, p.alpha)});
                    }

                    if (p.kind == Kind::Text || p.kind == Kind::Button)
                    {
                        SushiEngine::UI::UITextRun run;
                        run.rect = rect;
                        run.font_size = p.font_size;
                        // A button's label is white against its own fill; a text element paints
                        // in its authored colour.
                        run.color = p.kind == Kind::Button
                                        ? to_ui_color(SushiEngine::Vector3{1, 1, 1}, p.alpha)
                                        : to_ui_color(p.color, p.alpha);
                        run.align = p.kind == Kind::Button ? SushiEngine::UI::TextAlign::Center
                                                           : SushiEngine::UI::TextAlign::Left;
                        std::uint32_t length = 0;
                        while (length + 1 < SushiEngine::UI::UI_TEXT_CAPACITY &&
                               p.text[length] != '\0')
                        {
                            run.text[length] = p.text[length];
                            ++length;
                        }
                        run.length = length;
                        if (length > 0)
                            out.texts.push_back(run);
                    }
                }
            }

            /**
             * @brief Paints the editor's authoring chrome over the rendered viewport image.
             *
             * The elements themselves — fills and labels — are drawn by the renderer's own UI
             * overlay pass, inside the image this paints over, so what is left here is only
             * what the *editor* adds and a shipped game never shows: a canvas's extent, an
             * outline making an otherwise-invisible element grabbable while authoring, and the
             * selected element's outline and four corner resize handles.
             *
             * @param draw_list Panel draw list (clips to the panel, above the image).
             * @param ui        The overlay (elements + edit-mode flag).
             * @param rects     Each element's resolved pixel rect (see compute_ui_rects).
             */
            void paint_ui_overlay(ImDrawList* draw_list, const UIOverlay& ui,
                                  const std::vector<ImVec4>& rects)
            {
                using Kind = SushiEngine::Simulation::UIElementKind;
                for (std::size_t i = 0; i < ui.count; ++i)
                {
                    const SushiEngine::Simulation::UIElementParameters& p =
                        ui.elements[i].parameters;
                    const ImVec4 r = rects[i];
                    const ImVec2 mn(r.x, r.y);
                    const ImVec2 mx(r.x + r.z, r.y + r.w);

                    if (p.kind == Kind::Canvas)
                    {
                        // The canvas paints nothing of its own, so its bound is the only way to
                        // see the space its children are anchored in.
                        draw_list->AddRect(mn, mx, IM_COL32(120, 120, 130, 110));
                    }
                    else if (ui.edit_mode)
                    {
                        // A transparent panel or an empty label still has to be clickable while
                        // authoring, which it cannot be if nothing marks where it is.
                        draw_list->AddRect(mn, mx, IM_COL32(150, 150, 160, 110), 2.0f);
                    }

                    if (ui.elements[i].selected && ui.edit_mode)
                    {
                        draw_list->AddRect(mn, mx, IM_COL32(255, 170, 40, 255), 2.0f, 0, 2.0f);
                        if (p.kind != Kind::Canvas)
                        {
                            const ImVec2 corners[4] = {mn, ImVec2(mx.x, mn.y), ImVec2(mn.x, mx.y),
                                                       mx};
                            for (const ImVec2& c : corners)
                                draw_list->AddRectFilled(ImVec2(c.x - UI_HANDLE, c.y - UI_HANDLE),
                                                         ImVec2(c.x + UI_HANDLE, c.y + UI_HANDLE),
                                                         IM_COL32(255, 170, 40, 255));
                        }
                    }
                }
            }

            /** @brief The four corner points of a rect, in handle order (tl, tr, bl, br). */
            void ui_corners(const ImVec4& r, ImVec2 out[4]) noexcept
            {
                out[0] = ImVec2(r.x, r.y);
                out[1] = ImVec2(r.x + r.z, r.y);
                out[2] = ImVec2(r.x, r.y + r.w);
                out[3] = ImVec2(r.x + r.z, r.y + r.w);
            }

            /**
             * @brief Rewrites @p p's position + size so its screen rect becomes @p target.
             *
             * Delegates to `SushiEngine::UI::apply_screen_rect`, the inverse of the same
             * canonical `resolve_rect` formula `resolve_ui_rect` now calls — so a drag
             * edit and the read-back path agree on one formula rather than each
             * reimplementing it. Lets a drag edit the rect directly in screen space.
             */
            void ui_apply_screen_rect(SushiEngine::Simulation::UIElementParameters& p,
                                      const ImVec4& parent, const ImVec4& target)
            {
                SushiEngine::UI::RectTransform transform = ui_rect_transform(p);
                SushiEngine::UI::apply_screen_rect(to_ui_rect(parent), to_ui_rect(target), transform);
                p.position_x = transform.anchored_position.x;
                p.position_y = transform.anchored_position.y;
                p.size_x = transform.size_delta.x;
                p.size_y = transform.size_delta.y;
            }
        } // namespace
        ViewportPanel::ViewportPanel(SushiEngine::Render::IWindowRenderer& renderer,
                                     ImGuiBackend& imgui, const char* title, ISceneCamera& camera)
            : imgui_(imgui), title_(title), camera_(camera), view_(renderer.create_scene_view())
        {
            register_textures();
        }

        ViewportPanel::~ViewportPanel()
        {
            unregister_textures();
        }

        void ViewportPanel::register_textures()
        {
            slot_textures_.resize(view_->slot_count());
            for (std::uint32_t slot = 0; slot < view_->slot_count(); ++slot)
            {
                const SushiEngine::Render::SceneViewTexture texture = view_->texture(slot);
                slot_textures_[slot] = imgui_.register_texture(texture.sampler, texture.image_view);
            }
        }

        void ViewportPanel::unregister_textures()
        {
            for (ImTextureID texture : slot_textures_)
                imgui_.unregister_texture(texture);
            slot_textures_.clear();
        }

        void ViewportPanel::set_render_settings(
            const SushiEngine::Render::RenderSettings& settings)
        {
            view_->set_settings(settings);
        }

        void ViewportPanel::render_resolution(std::uint32_t& width,
                                              std::uint32_t& height) const noexcept
        {
            view_->render_resolution(width, height);
        }

        void ViewportPanel::resize_to(std::uint32_t width, std::uint32_t height)
        {
            if (width == view_->width() && height == view_->height())
                return;
            // The view's resize waits for the device to go idle before destroying its
            // targets, so the previous ImGui descriptor sets are released only *after*
            // every frame that could still be sampling them has landed. Releasing them
            // first would free sets the two-to-three frames still in flight are reading —
            // a use-after-free that shows as the viewport image vanishing on a resize.
            view_->resize(width, height);
            unregister_textures();
            register_textures();
        }

        void ViewportPanel::request_resize(std::uint32_t width, std::uint32_t height)
        {
            // A live drag changes the available size every frame, and honouring each
            // one costs a device idle, a full target rebuild, and a temporal-history
            // reset — per frame, that is a black, hitching viewport for the whole drag.
            // So the target keeps its extent while the size is still moving and rebuilds
            // once the size has been stable for a few frames; in between the old image
            // is stretched into the new rect, which is what every shipping editor shows
            // during a resize.
            if (width == view_->width() && height == view_->height())
            {
                pending_stable_frames_ = 0;
                return;
            }
            if (width == pending_width_ && height == pending_height_)
            {
                if (++pending_stable_frames_ >= RESIZE_SETTLE_FRAMES)
                {
                    resize_to(width, height);
                    pending_stable_frames_ = 0;
                }
                return;
            }
            pending_width_ = width;
            pending_height_ = height;
            pending_stable_frames_ = 1;
        }

        ImGuiWindowFlags ViewportPanel::apply_fullscreen_transition(bool want_fullscreen)
        {
            // Fullscreen (Unity's "Maximize on Play"): undock the panel and cover the
            // whole editor viewport instead of just stretching the rendered image inside
            // its current dock slot. The dock id it came from is remembered so turning
            // fullscreen back off restores it to the same tab/split rather than leaving
            // it floating.
            ImGuiWindowFlags window_flags = ImGuiWindowFlags_None;
            if (want_fullscreen)
            {
                if (!was_fullscreen_)
                {
                    const ImGuiWindow* self = ImGui::FindWindowByName(title_);
                    saved_dock_id_ = self != nullptr ? self->DockId : 0;
                    was_fullscreen_ = true;
                }
                const ImGuiViewport* main_viewport = ImGui::GetMainViewport();
                ImGui::SetNextWindowDockID(0, ImGuiCond_Always);
                ImGui::SetNextWindowPos(main_viewport->Pos, ImGuiCond_Always);
                ImGui::SetNextWindowSize(main_viewport->Size, ImGuiCond_Always);
                ImGui::SetNextWindowFocus();
                window_flags |= ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
                                ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoDocking;
            }
            else if (was_fullscreen_)
            {
                ImGui::SetNextWindowDockID(saved_dock_id_, ImGuiCond_Always);
                was_fullscreen_ = false;
            }
            return window_flags;
        }

        void ViewportPanel::draw_no_camera(bool& open, Authoring::GameViewSettings& settings)
        {
            const ImGuiWindowFlags window_flags =
                apply_fullscreen_transition(fullscreen_requested_ || settings.fullscreen);
            if (!ImGui::Begin(title_, &open, window_flags))
            {
                ImGui::End();
                return;
            }

            draw_game_view_toolbar(settings);

            // A black fill with a centred message where the image would be — the same
            // "Display 1 No cameras rendering" affordance Unity's Game view gives.
            const ImVec2 origin = ImGui::GetCursorScreenPos();
            const ImVec2 available = ImGui::GetContentRegionAvail();
            const float w = available.x > 1.0f ? available.x : 1.0f;
            const float h = available.y > 1.0f ? available.y : 1.0f;
            ImDrawList* draw_list = ImGui::GetWindowDrawList();
            draw_list->AddRectFilled(origin, ImVec2(origin.x + w, origin.y + h),
                                     IM_COL32(0, 0, 0, 255));

            static const char* MESSAGE = "No cameras rendering";
            const ImVec2 text_size = ImGui::CalcTextSize(MESSAGE);
            const ImVec2 text_pos(origin.x + (w - text_size.x) * 0.5f,
                                  origin.y + (h - text_size.y) * 0.5f);
            draw_list->AddText(text_pos, IM_COL32(200, 200, 200, 255), MESSAGE);

            ImGui::Dummy(ImVec2(w, h));

            ImGui::End();
        }

        bool ViewportPanel::draw(bool& open,
                                 const SushiEngine::Render::Environment& environment,
                                 std::uint32_t& selected_id,
                                 const ViewportFrameInputs& inputs)
        {
            const ImGuiWindowFlags window_flags = apply_fullscreen_transition(
                fullscreen_requested_ || (inputs.game_view != nullptr && inputs.game_view->fullscreen));

            if (!ImGui::Begin(title_, &open, window_flags))
            {
                ImGui::End();
                return false;
            }

            // Game view toolbar: aspect/orientation/fullscreen, drawn before the display
            // selector so the row reads left-to-right the way Unity's Game view does.
            if (inputs.game_view != nullptr)
                draw_game_view_toolbar(*inputs.game_view);

            // Display selector (Game view): a combo over the resolved displays. Drawn before
            // the image so it takes its own strip and the image keeps the correct aspect.
            if (inputs.display != nullptr && inputs.display->displays != nullptr && inputs.display->selected != nullptr &&
                inputs.display->count > 0)
            {
                if (inputs.game_view != nullptr)
                    ImGui::SameLine();
                int current = 0;
                for (std::size_t i = 0; i < inputs.display->count; ++i)
                    if (inputs.display->displays[i] == *inputs.display->selected)
                        current = static_cast<int>(i);
                char label[32];
                std::snprintf(label, sizeof(label), "Display %u", inputs.display->displays[current]);
                ImGui::SetNextItemWidth(160.0f);
                if (ImGui::BeginCombo("##display", label))
                {
                    for (std::size_t i = 0; i < inputs.display->count; ++i)
                    {
                        char item[32];
                        std::snprintf(item, sizeof(item), "Display %u", inputs.display->displays[i]);
                        const bool selected = static_cast<int>(i) == current;
                        if (ImGui::Selectable(item, selected))
                            *inputs.display->selected = inputs.display->displays[i];
                        if (selected)
                            ImGui::SetItemDefaultFocus();
                    }
                    ImGui::EndCombo();
                }
            }

            const ImVec2 content_origin = ImGui::GetCursorScreenPos();
            const ImVec2 available = ImGui::GetContentRegionAvail();
            const float available_w = available.x > 1.0f ? available.x : 1.0f;
            const float available_h = available.y > 1.0f ? available.y : 1.0f;

            // The Game view's aspect preset constrains the rendered image to a letterboxed
            // (or pillarboxed) rect centered in the panel, matching a played build's fixed
            // target aspect instead of stretching to whatever shape the panel happens to be.
            // This applies whether or not fullscreen (panel-maximize) is on — the two are
            // independent: fullscreen decides how big the panel itself is, aspect decides
            // the shape of the image inside it.
            float image_w = available_w;
            float image_h = available_h;
            float aspect_ratio = 0.0f;
            const bool constrained = inputs.game_view != nullptr &&
                                     Authoring::resolve_game_view_aspect_ratio(
                                         inputs.game_view->aspect,
                                         inputs.game_view->orientation,
                                                                    aspect_ratio);
            if (constrained)
            {
                if (available_w / available_h > aspect_ratio)
                    image_w = available_h * aspect_ratio;
                else
                    image_h = available_w / aspect_ratio;
            }
            const ImVec2 image_offset((available_w - image_w) * 0.5f, (available_h - image_h) * 0.5f);

            const std::uint32_t width = static_cast<std::uint32_t>(image_w > 1.0f ? image_w : 1.0f);
            const std::uint32_t height = static_cast<std::uint32_t>(image_h > 1.0f ? image_h : 1.0f);
            // Debounced: during a drag the target keeps its old extent (the image below
            // stretches into the new rect) and rebuilds once the size settles.
            request_resize(width, height);

            if (constrained && (image_offset.x > 0.5f || image_offset.y > 0.5f))
            {
                // Letterbox/pillarbox bars: fill the panel's whole content region first so
                // the empty margins around the constrained image read as intentional bars
                // rather than an unfilled window background.
                ImGui::GetWindowDrawList()->AddRectFilled(
                    content_origin,
                    ImVec2(content_origin.x + available_w, content_origin.y + available_h),
                    IM_COL32(0, 0, 0, 255));
            }
            ImGui::SetCursorScreenPos(
                ImVec2(content_origin.x + image_offset.x, content_origin.y + image_offset.y));

            // Unity fly navigation: right mouse over the panel starts a look session that
            // lasts until the button is released, even if the cursor leaves the panel.
            // Only navigable cameras (the Scene fly camera) consume it; the Game camera
            // is driven by the world and ignores the panel's input.
            ImGuiIO& io = ImGui::GetIO();
            const bool window_hovered = ImGui::IsWindowHovered();
            if (camera_.navigable())
            {
                if (window_hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Right))
                    looking_ = true;
                if (!ImGui::IsMouseDown(ImGuiMouseButton_Right))
                    looking_ = false;

                // Middle-mouse pan and wheel zoom are Unity Scene navigation: they work
                // without holding right mouse, gated to when the panel is hovered so
                // scrolling over other panels never moves the view.
                if (window_hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Middle))
                    panning_ = true;
                if (!ImGui::IsMouseDown(ImGuiMouseButton_Middle))
                    panning_ = false;

                InputState input;
                input.dt = io.DeltaTime;
                input.look_active = looking_;
                if (looking_)
                {
                    input.mouse_dx = io.MouseDelta.x;
                    input.mouse_dy = io.MouseDelta.y;
                    input.forward = ImGui::IsKeyDown(ImGuiKey_W);
                    input.back = ImGui::IsKeyDown(ImGuiKey_S);
                    input.left = ImGui::IsKeyDown(ImGuiKey_A);
                    input.right = ImGui::IsKeyDown(ImGuiKey_D);
                    input.up = ImGui::IsKeyDown(ImGuiKey_E);
                    input.down = ImGui::IsKeyDown(ImGuiKey_Q);
                    input.fast = io.KeyShift;
                }
                if (window_hovered)
                    input.wheel = io.MouseWheel;
                if (panning_)
                {
                    input.pan_active = true;
                    input.pan_dx = io.MouseDelta.x;
                    input.pan_dy = io.MouseDelta.y;
                }
                camera_.process(input);
            }

            const SushiEngine::Render::CameraView camera_view =
                camera_.view(static_cast<float>(width) / static_cast<float>(height));
            // The world's cosmetic emitters, plus the previewed effect's when this surface is
            // showing one. A preview belongs to no entity, so it is concatenated rather than
            // given a channel — the renderer cannot tell the two apart and has no reason to.
            const SushiEngine::Render::ParticleEmitterView* emitters = inputs.emitters;
            std::size_t emitter_count = inputs.emitter_count;
            std::vector<SushiEngine::Render::ParticleEmitterView> merged_emitters;
            const std::size_t preview_emitters =
                inputs.particle_preview != nullptr ? inputs.particle_preview->view_count() : 0;
            if (preview_emitters > 0)
            {
                merged_emitters.reserve(inputs.emitter_count + preview_emitters);
                merged_emitters.insert(merged_emitters.end(), inputs.emitters,
                                       inputs.emitters + inputs.emitter_count);
                merged_emitters.insert(merged_emitters.end(), inputs.particle_preview->views(),
                                       inputs.particle_preview->views() + preview_emitters);
                emitters = merged_emitters.data();
                emitter_count = merged_emitters.size();
            }
            // World crowd characters (design §12.3/§12.4) plus whatever single character is
            // being previewed — the same "world content plus the authored subject" merge
            // the billboards and emitters already do for their own kinds, just below.
            const SushiEngine::Render::SkinnedInstance* preview_skinned =
                inputs.animated_mesh != nullptr ? inputs.animated_mesh->skinned() : nullptr;
            const std::size_t preview_skinned_count =
                inputs.animated_mesh != nullptr ? inputs.animated_mesh->skinned_count() : 0;
            const SushiEngine::Render::SkinnedInstance* skinned = preview_skinned;
            std::size_t skinned_count = preview_skinned_count;
            std::vector<SushiEngine::Render::SkinnedInstance> merged_skinned;
            if (inputs.scene_skinned_count > 0)
            {
                merged_skinned.reserve(inputs.scene_skinned_count + preview_skinned_count);
                merged_skinned.insert(merged_skinned.end(), inputs.scene_skinned,
                                      inputs.scene_skinned + inputs.scene_skinned_count);
                if (preview_skinned_count > 0)
                    merged_skinned.insert(merged_skinned.end(), preview_skinned,
                                          preview_skinned + preview_skinned_count);
                skinned = merged_skinned.data();
                skinned_count = merged_skinned.size();
            }

            // The deterministic preview simulates on the CPU and hands over finished particles, the
            // same channel the sim's own deterministic emitters use, so the two are concatenated
            // rather than given a channel of their own.
            const SushiEngine::Render::ParticleBillboard* all_billboards = inputs.billboards;
            std::size_t all_billboard_count = inputs.billboard_count;
            std::vector<SushiEngine::Render::ParticleBillboard> merged_billboards;
            const std::size_t preview_billboards =
                inputs.particle_preview != nullptr ? inputs.particle_preview->billboard_count() : 0;
            if (preview_billboards > 0)
            {
                merged_billboards.reserve(inputs.billboard_count + preview_billboards);
                merged_billboards.insert(merged_billboards.end(), inputs.billboards,
                                         inputs.billboards + inputs.billboard_count);
                merged_billboards.insert(merged_billboards.end(), inputs.particle_preview->billboards(),
                                         inputs.particle_preview->billboards() + preview_billboards);
                all_billboards = merged_billboards.data();
                all_billboard_count = merged_billboards.size();
            }

            // The game's own UI, drawn by the renderer into the image rather than painted over
            // it afterwards — so what the Game view shows is what a shipped build shows. The
            // rects are resolved viewport-local (origin at the image's top left), which is the
            // space the offscreen target lives in; where the panel sits on screen is an ImGui
            // concern the renderer never learns. The pointer is converted with the previous
            // frame's image origin, which is not yet known for this one; a frame of latency on
            // a hover tint is not perceptible, and it is the only thing that depends on it.
            SushiEngine::Render::UIView ui_view;
            if (inputs.ui_overlay != nullptr && inputs.ui_overlay->count > 0)
            {
                std::vector<ImVec4> local_rects;
                compute_ui_rects(*inputs.ui_overlay, ImVec4(0.0f, 0.0f, static_cast<float>(width),
                                             static_cast<float>(height)),
                                 local_rects);
                const ImVec2 mouse = ImGui::GetIO().MousePos;
                const ImVec2 local_mouse(mouse.x - last_image_origin_.x,
                                         mouse.y - last_image_origin_.y);
                build_ui_draw_list(*inputs.ui_overlay, local_rects, local_mouse,
                                   ImGui::IsMouseDown(ImGuiMouseButton_Left), ui_draw_list_);
                ui_view.rects = ui_draw_list_.rects.data();
                ui_view.rect_count = ui_draw_list_.rects.size();
                ui_view.texts = ui_draw_list_.texts.data();
                ui_view.text_count = ui_draw_list_.texts.size();
                ui_view.width = static_cast<float>(width);
                ui_view.height = static_cast<float>(height);
            }

            view_->render(camera_view, environment, inputs.instances, inputs.instance_count, selected_id, inputs.deformable,
                          inputs.deformable_count, inputs.lights, inputs.light_count, inputs.decals, inputs.decal_count, inputs.show_grid,
                          skinned, skinned_count, emitters, emitter_count, all_billboards,
                          all_billboard_count, ui_view.empty() ? nullptr : &ui_view);

            // The transport for whatever this surface previews. One row for every kind of subject,
            // because "the thing being authored, playing or paused" is a property of the surface;
            // the deeper authoring (an effect's modules, an animator's layers and IK) stays in the
            // component or panel that owns it.
            if (inputs.preview_controls)
            {
                if (inputs.particle_preview != nullptr)
                {
                    const bool playing = inputs.particle_preview->playing();
                    if (ImGui::Button(playing ? "Pause Effect" : "Play Effect"))
                        inputs.particle_preview->set_playing(!playing);
                    ImGui::SameLine();
                    if (ImGui::Button("Restart Effect"))
                        inputs.particle_preview->restart();
                }
                if (inputs.animated_mesh != nullptr && inputs.animated_mesh->loaded())
                {
                    if (inputs.particle_preview != nullptr)
                        ImGui::SameLine();
                    const bool playing = inputs.animated_mesh->playing();
                    if (ImGui::Button(playing ? "Pause Animation" : "Play Animation"))
                        inputs.animated_mesh->set_playing(!playing);
                    ImGui::SameLine();
                    if (ImGui::Button("Restart Animation"))
                        inputs.animated_mesh->restart();
                }
            }

            const ImVec2 image_origin = ImGui::GetCursorScreenPos();
            last_image_origin_ = image_origin;
            ImGui::Image(slot_textures_[view_->current_slot()],
                         ImVec2(static_cast<float>(width), static_cast<float>(height)));
            const bool image_hovered = ImGui::IsItemHovered();

            // Immediately after the Image and before any overlay: a drag-drop target binds to
            // the last item drawn, so a toolbar or a gizmo drawn in between would take the
            // drop instead of the view.
            if (inputs.dropped_model_path != nullptr && ImGui::BeginDragDropTarget())
            {
                if (const ImGuiPayload* payload =
                        ImGui::AcceptDragDropPayload(ASSET_PATH_PAYLOAD))
                {
                    const std::string dropped(static_cast<const char*>(payload->Data));
                    const std::string extension =
                        std::filesystem::path(dropped).extension().string();
                    if (extension == ".gltf" || extension == ".glb")
                        *inputs.dropped_model_path = dropped;
                }
                ImGui::EndDragDropTarget();
            }
            if (inputs.dropped_model_path != nullptr && !inputs.dropped_model_path->empty())
            {
                // Painted straight onto the draw list, the way the UI overlay below is: this
                // is a layer over the rendered image, not a part of the panel's layout.
                // Moving the cursor to place it and moving it back leaves ImGui with a cursor
                // past the content bounds and no item to grow them, which it reports.
                const std::string label =
                    "Would place: " +
                    std::filesystem::path(*inputs.dropped_model_path).filename().string();
                ImGui::GetWindowDrawList()->AddText(
                    ImVec2(image_origin.x + 12.0f, image_origin.y + 12.0f),
                    ImGui::GetColorU32(ImGuiCol_TextDisabled), label.c_str());
            }

            // UI overlay: canvases, panels, images, text, and buttons painted on top of
            // the rendered image with ImGui's draw list — a 2D layer over the 3D view,
            // laid out against the panel rect so it tracks the viewport size. In edit mode
            // it is translucent and interactive: click to pick, drag to move, drag a
            // corner handle to resize (writing back into the element's parameters).
            bool ui_consumed = false;
            std::vector<ImVec4> ui_rects;
            if (inputs.ui_overlay != nullptr && inputs.ui_overlay->count > 0)
            {
                const ImVec4 root(image_origin.x, image_origin.y, static_cast<float>(width),
                                  static_cast<float>(height));
                compute_ui_rects(*inputs.ui_overlay, root, ui_rects);
                paint_ui_overlay(ImGui::GetWindowDrawList(), *inputs.ui_overlay, ui_rects);

                if (inputs.ui_overlay->edit_mode)
                {
                    using Kind = SushiEngine::Simulation::UIElementKind;
                    const float dx = io.MouseDelta.x;
                    const float dy = io.MouseDelta.y;

                    // Continue an in-progress drag: move the body or resize by a corner.
                    if (ui_drag_index_ >= 0 && ui_drag_index_ < static_cast<int>(inputs.ui_overlay->count) &&
                        ImGui::IsMouseDown(ImGuiMouseButton_Left))
                    {
                        ui_consumed = true;
                        const int parent = inputs.ui_overlay->elements[ui_drag_index_].parent;
                        const ImVec4 parent_rect =
                            (parent >= 0 && parent < static_cast<int>(inputs.ui_overlay->count)) ? ui_rects[parent]
                                                                                  : root;
                        ImVec4 rect = ui_rects[ui_drag_index_];
                        if (ui_drag_handle_ == 0)
                        {
                            rect.x += dx;
                            rect.y += dy;
                        }
                        else
                        {
                            float x0 = rect.x, y0 = rect.y, x1 = rect.x + rect.z, y1 = rect.y + rect.w;
                            switch (ui_drag_handle_)
                            {
                                case 1: x0 += dx; y0 += dy; break;
                                case 2: x1 += dx; y0 += dy; break;
                                case 3: x0 += dx; y1 += dy; break;
                                default: x1 += dx; y1 += dy; break;
                            }
                            if (x1 - x0 < 4.0f) x1 = x0 + 4.0f;
                            if (y1 - y0 < 4.0f) y1 = y0 + 4.0f;
                            rect = ImVec4(x0, y0, x1 - x0, y1 - y0);
                        }
                        ui_apply_screen_rect(inputs.ui_overlay->elements[ui_drag_index_].parameters,
                                             parent_rect, rect);
                        inputs.ui_overlay->edited_index = ui_drag_index_;
                    }
                    else
                    {
                        ui_drag_index_ = -1;
                        ui_drag_handle_ = -1;
                    }

                    // Start a drag/pick on a fresh left-click over the panel.
                    if (image_hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left) &&
                        ui_drag_index_ < 0)
                    {
                        const ImVec2 mouse = io.MousePos;
                        int hit = -1;
                        int handle = -1;
                        // A grabbed corner of the already-selected element resizes it.
                        for (std::size_t i = 0; i < inputs.ui_overlay->count; ++i)
                        {
                            if (inputs.ui_overlay->elements[i].id != inputs.ui_overlay->selected_id ||
                                inputs.ui_overlay->elements[i].parameters.kind == Kind::Canvas)
                                continue;
                            ImVec2 corners[4];
                            ui_corners(ui_rects[i], corners);
                            for (int k = 0; k < 4; ++k)
                                if (mouse.x >= corners[k].x - UI_HANDLE &&
                                    mouse.x <= corners[k].x + UI_HANDLE &&
                                    mouse.y >= corners[k].y - UI_HANDLE &&
                                    mouse.y <= corners[k].y + UI_HANDLE)
                                {
                                    hit = static_cast<int>(i);
                                    handle = k + 1;
                                }
                        }
                        // Otherwise the topmost element body under the cursor is picked
                        // and moved; a canvas is never picked by its body so it does not
                        // swallow clicks meant for the scene behind it.
                        if (hit < 0)
                            for (int i = static_cast<int>(inputs.ui_overlay->count) - 1; i >= 0; --i)
                            {
                                if (inputs.ui_overlay->elements[i].parameters.kind == Kind::Canvas)
                                    continue;
                                const ImVec4& r = ui_rects[i];
                                if (mouse.x >= r.x && mouse.x <= r.x + r.z && mouse.y >= r.y &&
                                    mouse.y <= r.y + r.w)
                                {
                                    hit = i;
                                    handle = 0;
                                    break;
                                }
                            }
                        if (hit >= 0)
                        {
                            inputs.ui_overlay->picked_id = inputs.ui_overlay->elements[hit].id;
                            ui_drag_index_ = hit;
                            ui_drag_handle_ = handle;
                            ui_consumed = true;
                        }
                    }
                }
            }

            // Debug overlay: the selection's oriented bounds and a small icon at each
            // punctual light, painted over the 3D image with the same camera projection the
            // gizmo uses. A read-only developer aid — it never consumes input, and it is
            // drawn before the gizmo so the handles sit on top. Only in an authoring
            // (pickable) viewport; the played Game view shows none of it.
            if (inputs.pickable)
            {
                ImDrawList* dl = ImGui::GetWindowDrawList();
                const SushiEngine::Matrix4 vp =
                    SushiEngine::mul(camera_view.projection, camera_view.view);
                const float w = static_cast<float>(width);
                const float h = static_cast<float>(height);

                auto project = [&](const SushiEngine::Vector3& p, ImVec2& out) -> bool
                {
                    const SushiEngine::Scalar* m = vp.m;
                    const SushiEngine::Scalar cx = m[0] * p.x + m[4] * p.y + m[8] * p.z + m[12];
                    const SushiEngine::Scalar cy = m[1] * p.x + m[5] * p.y + m[9] * p.z + m[13];
                    const SushiEngine::Scalar cw = m[3] * p.x + m[7] * p.y + m[11] * p.z + m[15];
                    if (cw <= SushiEngine::Scalar(0.0001))
                        return false;
                    out.x = image_origin.x +
                            static_cast<float>(cx / cw * SushiEngine::Scalar(0.5) +
                                               SushiEngine::Scalar(0.5)) * w;
                    out.y = image_origin.y +
                            static_cast<float>(cy / cw * SushiEngine::Scalar(0.5) +
                                               SushiEngine::Scalar(0.5)) * h;
                    return true;
                };

                if (selected_id != SushiEngine::Render::NO_PICK)
                {
                    for (std::size_t i = 0; i < inputs.instance_count; ++i)
                    {
                        const SushiEngine::Render::MeshInstance& inst = inputs.instances[i];
                        if (inst.id != selected_id)
                            continue;
                        using Kind = SushiEngine::Render::MeshKind;
                        SushiEngine::Vector3 he = inst.shape_parameters;
                        if (inst.kind == Kind::Sphere)
                            he = SushiEngine::Vector3{inst.shape_parameters.x,
                                                      inst.shape_parameters.x,
                                                      inst.shape_parameters.x};
                        else if (inst.kind == Kind::Cylinder)
                            he = SushiEngine::Vector3{inst.shape_parameters.x,
                                                      inst.shape_parameters.y,
                                                      inst.shape_parameters.x};
                        const SushiEngine::Scalar* mm = inst.model.m;
                        ImVec2 corner[8];
                        bool ok = true;
                        for (int c = 0; c < 8 && ok; ++c)
                        {
                            const SushiEngine::Vector3 local{(c & 1) ? he.x : -he.x,
                                                             (c & 2) ? he.y : -he.y,
                                                             (c & 4) ? he.z : -he.z};
                            const SushiEngine::Vector3 wp{
                                mm[0] * local.x + mm[4] * local.y + mm[8] * local.z + mm[12],
                                mm[1] * local.x + mm[5] * local.y + mm[9] * local.z + mm[13],
                                mm[2] * local.x + mm[6] * local.y + mm[10] * local.z + mm[14]};
                            ok = project(wp, corner[c]);
                        }
                        if (ok)
                        {
                            static const int edges[12][2] = {
                                {0, 1}, {1, 3}, {3, 2}, {2, 0}, {4, 5}, {5, 7},
                                {7, 6}, {6, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
                            const ImU32 col = IM_COL32(255, 165, 50, 180);
                            for (const auto& e : edges)
                                dl->AddLine(corner[e[0]], corner[e[1]], col, 1.5f);
                        }
                        break;
                    }
                }

                auto to255 = [](SushiEngine::Scalar v) -> int
                {
                    v = v < SushiEngine::Scalar(0) ? SushiEngine::Scalar(0)
                                                   : (v > SushiEngine::Scalar(1) ? SushiEngine::Scalar(1) : v);
                    return static_cast<int>(v * SushiEngine::Scalar(255));
                };
                for (std::size_t i = 0; i < inputs.light_count; ++i)
                {
                    const SushiEngine::Vector3 lp{
                        static_cast<SushiEngine::Scalar>(inputs.lights[i].position.x),
                        static_cast<SushiEngine::Scalar>(inputs.lights[i].position.y),
                        static_cast<SushiEngine::Scalar>(inputs.lights[i].position.z)};
                    ImVec2 s;
                    if (!project(lp, s))
                        continue;
                    const ImU32 col = IM_COL32(to255(inputs.lights[i].color.x), to255(inputs.lights[i].color.y),
                                               to255(inputs.lights[i].color.z), 230);
                    const float r = 6.0f;
                    dl->AddQuad(ImVec2(s.x, s.y - r), ImVec2(s.x + r, s.y), ImVec2(s.x, s.y + r),
                                ImVec2(s.x - r, s.y), col, 1.5f);
                }
            }

            // Skeleton preview: draw the rest pose of a loaded rigged glTF over the scene
            // (A0's "see its rest pose"), before the gizmo so handles stay on top.
            if (inputs.skeleton != nullptr && inputs.skeleton->loaded())
                draw_skeleton_overlay(*inputs.skeleton, camera_view, image_origin,
                                      static_cast<float>(width), static_cast<float>(height),
                                      ImGui::GetWindowDrawList(), inputs.skeleton_names);

            // The cooked collider over the mesh it was cooked from, so "the collider is not the
            // mesh" is visible rather than a number in a panel (§14). At the origin: a cooked
            // asset is authored in its own space and placing it belongs to whatever instances
            // it, which the bake surface does not.
            if (inputs.collision_wireframe != nullptr && !inputs.collision_wireframe->empty())
                draw_collision_overlay(*inputs.collision_wireframe, SushiEngine::Matrix4{},
                                       camera_view, image_origin, static_cast<float>(width),
                                       static_cast<float>(height), ImGui::GetWindowDrawList());

            // The selected soft body's interior (P6-G5). Already in world space — these
            // are the live particles, not an asset in its own frame — so unlike the
            // collider above there is no placement to apply.
            if (inputs.soft_body_view != Authoring::SoftBodyDebugView::Off &&
                inputs.soft_body_positions != nullptr && inputs.soft_body_elements != nullptr)
                draw_soft_body_overlay(*inputs.soft_body_positions, *inputs.soft_body_elements,
                                       inputs.soft_body_material, inputs.soft_body_view,
                                       camera_view, image_origin, static_cast<float>(width),
                                       static_cast<float>(height), ImGui::GetWindowDrawList());

            // §14's physics debug draw and the joint gizmo, before the transform gizmo so
            // its handles stay on top. Every category is off unless the Physics window asked
            // for it, and the whole pass is skipped when none is.
            if (inputs.physics_world != nullptr && inputs.physics_overlay.any())
                draw_physics_overlay(*inputs.physics_world, inputs.selected_entity,
                                     inputs.physics_overlay, camera_view, image_origin,
                                     static_cast<float>(width), static_cast<float>(height),
                                     ImGui::GetWindowDrawList());

            // Particle emitter gizmo: mark where the previewed effect spawns, before the
            // transform gizmo so its handles stay on top.
            if (inputs.particle_preview != nullptr)
                draw_emitter_gizmo(*inputs.particle_preview, camera_view, image_origin,
                                   static_cast<float>(width), static_cast<float>(height),
                                   ImGui::GetWindowDrawList());

            // IK target gizmo (design §12.1): a second, independent GizmoController drags
            // AnimatedMeshPreview's two-bone IK target, converting to/from the character's
            // object space each frame (the solver's `target` field is object-space; the gizmo
            // manipulates a world-space EntityTransform, the same type the selection gizmo
            // edits, so this reuses that math rather than a bespoke drag implementation).
            // Scene-view only — `ik_gizmo` is false for the Game view, mirroring the
            // gizmo-target/pickable split above.
            if (inputs.ik_gizmo && inputs.animated_mesh != nullptr && inputs.animated_mesh->loaded())
            {
                const SushiEngine::Matrix4& character_world = inputs.animated_mesh->world();
                SushiEngine::Simulation::EntityTransform ik_transform;
                ik_transform.position =
                    transform_point(character_world, inputs.animated_mesh->two_bone_ik().target);
                static const GizmoSnap no_snap;
                const GizmoController::Result ik_result = ik_gizmo_.manipulate(
                    Authoring::GizmoMode::Translate, Authoring::GizmoSpace::World,
                    ik_transform, camera_view,
                    image_origin, static_cast<float>(width), static_cast<float>(height),
                    image_hovered, no_snap);
                if (ik_result.modified)
                    inputs.animated_mesh->set_ik_target(transform_point(
                        SushiEngine::affine_inverse(character_world), ik_transform.position));
            }

            // Transform gizmo: the GizmoController owns the handle drawing and drag mapping
            // for the active mode. Handled before picking so grabbing a handle never
            // reselects the entity under the cursor.
            GizmoController::Result gizmo{};
            if (inputs.gizmo_target != nullptr)
            {
                static const GizmoSnap no_snap;
                gizmo = gizmo_.manipulate(inputs.gizmo_mode, inputs.gizmo_space, *inputs.gizmo_target, camera_view,
                                          image_origin, static_cast<float>(width),
                                          static_cast<float>(height), image_hovered,
                                          inputs.gizmo_snap != nullptr ? *inputs.gizmo_snap : no_snap);
            }

            // Left-click in the viewport picks the entity under the cursor (right mouse is
            // reserved for navigation), unless the click grabbed a gizmo handle. The image
            // is drawn 1:1 with the target, so the local pixel is the cursor offset.
            if (inputs.ui_overlay != nullptr)
                inputs.ui_overlay->consumed_click = ui_consumed;

            if (inputs.pickable && !gizmo.consumed_click && !ui_consumed && image_hovered &&
                ImGui::IsMouseClicked(ImGuiMouseButton_Left))
            {
                const ImVec2 mouse = ImGui::GetIO().MousePos;
                const float local_x = mouse.x - image_origin.x;
                const float local_y = mouse.y - image_origin.y;
                if (local_x >= 0.0f && local_y >= 0.0f)
                    selected_id = view_->pick(static_cast<std::uint32_t>(local_x),
                                              static_cast<std::uint32_t>(local_y));
            }

            ImGui::End();
            return gizmo.modified;
        }
    } // namespace Editor
} // namespace SushiEngine
