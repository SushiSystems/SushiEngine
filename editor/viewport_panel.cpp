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

#include <cmath>

namespace sushi::editor
{
    namespace
    {
        // Projects a world point to panel-local screen pixels through the camera's
        // view-projection. Returns false when the point is behind the camera. The
        // projection already carries Vulkan's Y flip, so the NDC maps straight to the
        // top-down image the panel draws.
        bool project_to_screen(const SushiEngine::Mat4& view_projection,
                               const SushiEngine::Vec3& point, const ImVec2& origin,
                               float width, float height, ImVec2& out)
        {
            const float* m = view_projection.m;
            const float x = m[0] * point.x + m[4] * point.y + m[8] * point.z + m[12];
            const float y = m[1] * point.x + m[5] * point.y + m[9] * point.z + m[13];
            const float w = m[3] * point.x + m[7] * point.y + m[11] * point.z + m[15];
            if (w <= 0.0001f)
                return false;
            out.x = origin.x + (x / w * 0.5f + 0.5f) * width;
            out.y = origin.y + (y / w * 0.5f + 0.5f) * height;
            return true;
        }

        // Shortest distance from point m to segment a-b, in pixels — the hit test for
        // an axis handle.
        float distance_to_segment(const ImVec2& a, const ImVec2& b, const ImVec2& m)
        {
            const float abx = b.x - a.x, aby = b.y - a.y;
            const float length_squared = abx * abx + aby * aby;
            float t = length_squared > 0.0f
                          ? ((m.x - a.x) * abx + (m.y - a.y) * aby) / length_squared
                          : 0.0f;
            t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
            const float dx = m.x - (a.x + t * abx), dy = m.y - (a.y + t * aby);
            return std::sqrt(dx * dx + dy * dy);
        }
    }

    ViewportPanel::ViewportPanel(SushiEngine::render::IWindowRenderer& renderer,
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
            const SushiEngine::render::SceneViewTexture texture = view_->texture(slot);
            slot_textures_[slot] = imgui_.register_texture(texture.sampler, texture.image_view);
        }
    }

    void ViewportPanel::unregister_textures()
    {
        for (ImTextureID texture : slot_textures_)
            imgui_.unregister_texture(texture);
        slot_textures_.clear();
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

    void ViewportPanel::draw(bool& open, const SushiEngine::render::MeshInstance* instances,
                             std::size_t count, std::uint32_t& selected_id,
                             SushiEngine::Vec3* gizmo_position)
    {
        if (!ImGui::Begin(title_, &open))
        {
            ImGui::End();
            return;
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
        if (camera_.navigable())
        {
            if (ImGui::IsWindowHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Right))
                looking_ = true;
            if (!ImGui::IsMouseDown(ImGuiMouseButton_Right))
                looking_ = false;

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
            camera_.process(input);
        }

        const SushiEngine::render::CameraView camera_view =
            camera_.view(static_cast<float>(width) / static_cast<float>(height));
        view_->render(camera_view, instances, count, selected_id);

        const ImVec2 image_origin = ImGui::GetCursorScreenPos();
        ImGui::Image(slot_textures_[view_->current_slot()],
                     ImVec2(static_cast<float>(width), static_cast<float>(height)));
        const bool image_hovered = ImGui::IsItemHovered();

        // Translate gizmo: three world-axis handles at the selection, dragged with the
        // left mouse. Handled before picking so grabbing a handle never reselects.
        bool gizmo_consumed_click = false;
        if (gizmo_position != nullptr)
        {
            const float handle_length = 1.2f;
            const SushiEngine::Vec3 axes[3] = {
                {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
            const ImU32 axis_colors[3] = {IM_COL32(230, 80, 80, 255),
                                          IM_COL32(90, 220, 90, 255),
                                          IM_COL32(90, 150, 240, 255)};

            const SushiEngine::Mat4 view_projection =
                SushiEngine::mul(camera_view.projection, camera_view.view);
            const float fw = static_cast<float>(width);
            const float fh = static_cast<float>(height);

            ImVec2 origin_screen;
            ImVec2 tip_screen[3];
            bool tip_ok[3] = {false, false, false};
            const bool origin_ok = project_to_screen(view_projection, *gizmo_position,
                                                     image_origin, fw, fh, origin_screen);

            ImDrawList* draw_list = ImGui::GetWindowDrawList();
            if (origin_ok)
            {
                for (int a = 0; a < 3; ++a)
                {
                    const SushiEngine::Vec3 tip = *gizmo_position + axes[a] * handle_length;
                    tip_ok[a] = project_to_screen(view_projection, tip, image_origin, fw, fh,
                                                  tip_screen[a]);
                    if (tip_ok[a])
                        draw_list->AddLine(origin_screen, tip_screen[a], axis_colors[a], 3.0f);
                }
            }

            const ImVec2 mouse = ImGui::GetIO().MousePos;
            if (gizmo_axis_ >= 0)
            {
                // A drag is in progress: translate along the axis captured at grab time.
                if (ImGui::IsMouseDown(ImGuiMouseButton_Left))
                {
                    const float along = (mouse.x - gizmo_start_mouse_.x) * gizmo_axis_screen_.x +
                                        (mouse.y - gizmo_start_mouse_.y) * gizmo_axis_screen_.y;
                    *gizmo_position = gizmo_start_position_ +
                                      axes[gizmo_axis_] * (along * gizmo_world_per_pixel_);
                    gizmo_consumed_click = true;
                }
                else
                {
                    gizmo_axis_ = -1;
                }
            }
            else if (origin_ok && image_hovered &&
                     ImGui::IsMouseClicked(ImGuiMouseButton_Left))
            {
                // Grab the nearest handle within a pixel threshold.
                int best = -1;
                float best_distance = 8.0f;
                for (int a = 0; a < 3; ++a)
                {
                    if (!tip_ok[a])
                        continue;
                    const float distance =
                        distance_to_segment(origin_screen, tip_screen[a], mouse);
                    if (distance < best_distance)
                    {
                        best_distance = distance;
                        best = a;
                    }
                }
                if (best >= 0)
                {
                    const float dx = tip_screen[best].x - origin_screen.x;
                    const float dy = tip_screen[best].y - origin_screen.y;
                    const float screen_length = std::sqrt(dx * dx + dy * dy);
                    if (screen_length > 0.001f)
                    {
                        gizmo_axis_ = best;
                        gizmo_start_mouse_ = mouse;
                        gizmo_start_position_ = *gizmo_position;
                        gizmo_axis_screen_ = ImVec2(dx / screen_length, dy / screen_length);
                        gizmo_world_per_pixel_ = handle_length / screen_length;
                        gizmo_consumed_click = true;
                    }
                }
            }
        }

        // Left-click in the viewport picks the entity under the cursor (right mouse is
        // reserved for navigation), unless the click grabbed a gizmo handle. The image
        // is drawn 1:1 with the target, so the local pixel is the cursor offset.
        if (!gizmo_consumed_click && gizmo_axis_ < 0 && image_hovered &&
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
    }
} // namespace sushi::editor
