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

#include <cstdio>
#include <vector>

#include <SushiEngine/ui/layout.hpp>

namespace SushiEngine
{
    namespace Editor
    {
        namespace
        {
            /** @brief Builds the `UI::RectTransform` equivalent of an authored UI element's params. */
            SushiEngine::UI::RectTransform ui_rect_transform(
                const SushiEngine::Simulation::UIElementParams& p) noexcept
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

                const SushiEngine::Simulation::UIElementParams& p = ui[i].params;
                const ImVec4 rect =
                    p.kind == SushiEngine::Simulation::UIElementKind::Canvas
                        ? parent
                        : to_im_vec4(SushiEngine::UI::resolve_rect(to_ui_rect(parent),
                                                                   ui_rect_transform(p)));
                rects[static_cast<std::size_t>(i)] = rect;
                return rect;
            }

            /** @brief Packs a colour (0..1) and opacity into an ImGui ABGR value. */
            ImU32 ui_color(const SushiEngine::Vector3& c, float alpha)
            {
                const auto channel = [](SushiEngine::Scalar v)
                {
                    const float f = static_cast<float>(v);
                    const float clamped = f < 0.0f ? 0.0f : (f > 1.0f ? 1.0f : f);
                    return static_cast<int>(clamped * 255.0f + 0.5f);
                };
                const float a = alpha < 0.0f ? 0.0f : (alpha > 1.0f ? 1.0f : alpha);
                return IM_COL32(channel(c.x), channel(c.y), channel(c.z),
                                static_cast<int>(a * 255.0f + 0.5f));
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

            /**
             * @brief Paints the UI overlay over the rendered viewport image.
             *
             * In edit mode fills are drawn translucent (so the 3D view shows through a
             * canvas) with an outline, and the selected element gets an outline plus four
             * corner resize handles; in play mode fills are solid — the runtime look.
             *
             * @param draw_list Panel draw list (clips to the panel, above the image).
             * @param ui        The overlay (elements + edit-mode flag).
             * @param rects     Each element's resolved pixel rect (see compute_ui_rects).
             */
            void paint_ui_overlay(ImDrawList* draw_list, const UIOverlay& ui,
                                  const std::vector<ImVec4>& rects)
            {
                using Kind = SushiEngine::Simulation::UIElementKind;
                ImFont* font = ImGui::GetFont();
                const ImVec2 mouse = ImGui::GetIO().MousePos;
                const bool mouse_down = ImGui::IsMouseDown(ImGuiMouseButton_Left);
                // Translucent in edit mode so a full-screen canvas never hides the scene.
                const float fill_scale = ui.edit_mode ? 0.35f : 1.0f;
                for (std::size_t i = 0; i < ui.count; ++i)
                {
                    const SushiEngine::Simulation::UIElementParams& p = ui.elements[i].params;
                    const ImVec4 r = rects[i];
                    const ImVec2 mn(r.x, r.y);
                    const ImVec2 mx(r.x + r.z, r.y + r.w);
                    const float alpha = static_cast<float>(p.alpha);

                    if (p.kind == Kind::Canvas)
                    {
                        // The canvas draws only a faint bound so its extent is visible.
                        draw_list->AddRect(mn, mx, IM_COL32(120, 120, 130, 110));
                    }
                    else if (p.kind == Kind::Button)
                    {
                        const bool hovered = !ui.edit_mode && mouse.x >= mn.x && mouse.x <= mx.x &&
                                             mouse.y >= mn.y && mouse.y <= mx.y;
                        float tint = 1.0f;
                        if (hovered)
                            tint = mouse_down ? 0.8f : 1.15f;
                        const SushiEngine::Vector3 shaded{p.color.x * tint, p.color.y * tint,
                                                          p.color.z * tint};
                        draw_list->AddRectFilled(mn, mx, ui_color(shaded, alpha * fill_scale), 4.0f);
                        draw_list->AddRect(mn, mx, IM_COL32(20, 20, 25, 200), 4.0f);
                        const float font_size = static_cast<float>(p.font_size);
                        const ImVec2 text_size = font->CalcTextSizeA(font_size, FLT_MAX, 0.0f, p.text);
                        const ImVec2 text_pos(mn.x + (r.z - text_size.x) * 0.5f,
                                              mn.y + (r.w - text_size.y) * 0.5f);
                        draw_list->AddText(font, font_size, text_pos, IM_COL32(255, 255, 255, 255),
                                           p.text);
                    }
                    else if (p.kind == Kind::Text)
                    {
                        const float font_size = static_cast<float>(p.font_size);
                        draw_list->AddText(font, font_size, mn, ui_color(p.color, alpha), p.text);
                        if (ui.edit_mode)
                            draw_list->AddRect(mn, mx, IM_COL32(150, 150, 160, 90));
                    }
                    else // Panel / Image
                    {
                        draw_list->AddRectFilled(mn, mx, ui_color(p.color, alpha * fill_scale), 2.0f);
                        if (ui.edit_mode)
                            draw_list->AddRect(mn, mx, IM_COL32(150, 150, 160, 130), 2.0f);
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
            void ui_apply_screen_rect(SushiEngine::Simulation::UIElementParams& p,
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
            // The resize invalidates every slot's image view, so the ImGui texture ids
            // must be released and re-registered against the new views.
            unregister_textures();
            view_->resize(width, height);
            register_textures();
        }

        bool ViewportPanel::draw(bool& open, const SushiEngine::Render::MeshInstance* instances,
                                 std::size_t count,
                                 const SushiEngine::Render::Environment& environment,
                                 std::uint32_t& selected_id, bool pickable,
                                 SushiEngine::Simulation::EntityTransform* gizmo_target,
                                 GizmoMode gizmo_mode, GizmoSpace gizmo_space,
                                 const GizmoSnap* gizmo_snap, const DisplaySelector* display,
                                 const SushiEngine::Render::ClothStrandView* strands,
                                 std::size_t strand_count, UIOverlay* ui)
        {
            if (!ImGui::Begin(title_, &open))
            {
                ImGui::End();
                return false;
            }

            // Display selector (Game view): a combo over the resolved displays. Drawn before
            // the image so it takes its own strip and the image keeps the correct aspect.
            if (display != nullptr && display->displays != nullptr && display->selected != nullptr &&
                display->count > 0)
            {
                int current = 0;
                for (std::size_t i = 0; i < display->count; ++i)
                    if (display->displays[i] == *display->selected)
                        current = static_cast<int>(i);
                char label[32];
                std::snprintf(label, sizeof(label), "Display %u", display->displays[current]);
                ImGui::SetNextItemWidth(160.0f);
                if (ImGui::BeginCombo("##display", label))
                {
                    for (std::size_t i = 0; i < display->count; ++i)
                    {
                        char item[32];
                        std::snprintf(item, sizeof(item), "Display %u", display->displays[i]);
                        const bool selected = static_cast<int>(i) == current;
                        if (ImGui::Selectable(item, selected))
                            *display->selected = display->displays[i];
                        if (selected)
                            ImGui::SetItemDefaultFocus();
                    }
                    ImGui::EndCombo();
                }
            }

            const ImVec2 available = ImGui::GetContentRegionAvail();
            const std::uint32_t width =
                static_cast<std::uint32_t>(available.x > 1.0f ? available.x : 1.0f);
            const std::uint32_t height =
                static_cast<std::uint32_t>(available.y > 1.0f ? available.y : 1.0f);
            resize_to(width, height);

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
            view_->render(camera_view, environment, instances, count, selected_id, strands,
                          strand_count);

            const ImVec2 image_origin = ImGui::GetCursorScreenPos();
            ImGui::Image(slot_textures_[view_->current_slot()],
                         ImVec2(static_cast<float>(width), static_cast<float>(height)));
            const bool image_hovered = ImGui::IsItemHovered();

            // UI overlay: canvases, panels, images, text, and buttons painted on top of
            // the rendered image with ImGui's draw list — a 2D layer over the 3D view,
            // laid out against the panel rect so it tracks the viewport size. In edit mode
            // it is translucent and interactive: click to pick, drag to move, drag a
            // corner handle to resize (writing back into the element's params).
            bool ui_consumed = false;
            std::vector<ImVec4> ui_rects;
            if (ui != nullptr && ui->count > 0)
            {
                const ImVec4 root(image_origin.x, image_origin.y, static_cast<float>(width),
                                  static_cast<float>(height));
                compute_ui_rects(*ui, root, ui_rects);
                paint_ui_overlay(ImGui::GetWindowDrawList(), *ui, ui_rects);

                if (ui->edit_mode)
                {
                    using Kind = SushiEngine::Simulation::UIElementKind;
                    const float dx = io.MouseDelta.x;
                    const float dy = io.MouseDelta.y;

                    // Continue an in-progress drag: move the body or resize by a corner.
                    if (ui_drag_index_ >= 0 && ui_drag_index_ < static_cast<int>(ui->count) &&
                        ImGui::IsMouseDown(ImGuiMouseButton_Left))
                    {
                        ui_consumed = true;
                        const int parent = ui->elements[ui_drag_index_].parent;
                        const ImVec4 parent_rect =
                            (parent >= 0 && parent < static_cast<int>(ui->count)) ? ui_rects[parent]
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
                        ui_apply_screen_rect(ui->elements[ui_drag_index_].params, parent_rect, rect);
                        ui->edited_index = ui_drag_index_;
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
                        for (std::size_t i = 0; i < ui->count; ++i)
                        {
                            if (ui->elements[i].id != ui->selected_id ||
                                ui->elements[i].params.kind == Kind::Canvas)
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
                            for (int i = static_cast<int>(ui->count) - 1; i >= 0; --i)
                            {
                                if (ui->elements[i].params.kind == Kind::Canvas)
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
                            ui->picked_id = ui->elements[hit].id;
                            ui_drag_index_ = hit;
                            ui_drag_handle_ = handle;
                            ui_consumed = true;
                        }
                    }
                }
            }

            // Transform gizmo: the GizmoController owns the handle drawing and drag mapping
            // for the active mode. Handled before picking so grabbing a handle never
            // reselects the entity under the cursor.
            GizmoController::Result gizmo{};
            if (gizmo_target != nullptr)
            {
                static const GizmoSnap no_snap;
                gizmo = gizmo_.manipulate(gizmo_mode, gizmo_space, *gizmo_target, camera_view,
                                          image_origin, static_cast<float>(width),
                                          static_cast<float>(height), image_hovered,
                                          gizmo_snap != nullptr ? *gizmo_snap : no_snap);
            }

            // Left-click in the viewport picks the entity under the cursor (right mouse is
            // reserved for navigation), unless the click grabbed a gizmo handle. The image
            // is drawn 1:1 with the target, so the local pixel is the cursor offset.
            if (ui != nullptr)
                ui->consumed_click = ui_consumed;

            if (pickable && !gizmo.consumed_click && !ui_consumed && image_hovered &&
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
