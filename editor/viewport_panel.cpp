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
                                 ImGuiBackend& imgui, const char* title)
        : imgui_(imgui), title_(title), view_(renderer.create_scene_view())
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
                             std::size_t count)
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
        ImGuiIO& io = ImGui::GetIO();
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
        controller_.update(camera_, input);

        SushiEngine::render::CameraView camera_view;
        camera_view.view = camera_.view_matrix();
        camera_view.projection =
            camera_.projection(static_cast<float>(width) / static_cast<float>(height));
        view_->render(camera_view, instances, count);

        ImGui::Image(slot_textures_[view_->current_slot()],
                     ImVec2(static_cast<float>(width), static_cast<float>(height)));
        ImGui::End();
    }
} // namespace sushi::editor
