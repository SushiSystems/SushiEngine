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

namespace SushiEngine
{
    namespace Editor
    {
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
                                 std::size_t count, std::uint32_t& selected_id, bool pickable,
                                 SushiEngine::Simulation::EntityTransform* gizmo_target,
                                 GizmoMode gizmo_mode, GizmoSpace gizmo_space,
                                 const GizmoSnap* gizmo_snap, const DisplaySelector* display,
                                 const SushiEngine::Render::ClothStrandView* strands,
                                 std::size_t strand_count)
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
            view_->render(camera_view, instances, count, selected_id, strands, strand_count);

            const ImVec2 image_origin = ImGui::GetCursorScreenPos();
            ImGui::Image(slot_textures_[view_->current_slot()],
                         ImVec2(static_cast<float>(width), static_cast<float>(height)));
            const bool image_hovered = ImGui::IsItemHovered();

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
            if (pickable && !gizmo.consumed_click && image_hovered &&
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
