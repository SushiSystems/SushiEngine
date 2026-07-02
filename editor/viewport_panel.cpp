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

namespace sushi::editor
{
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
                             std::size_t count, std::uint32_t& selected_id)
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

        // Left-click in the viewport picks the entity under the cursor (right mouse is
        // reserved for navigation). The image is drawn 1:1 with the target, so the
        // local pixel is just the cursor offset from the image's top-left.
        if (ImGui::IsItemHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
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
