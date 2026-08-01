/**************************************************************************/
/* soft_body_overlay.cpp                                                  */
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

#include "soft_body_overlay.hpp"

#include "../core/viewport_projection.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        namespace
        {
            /** @brief The six edges of a tetrahedron, as index pairs into its four vertices. */
            constexpr int TETRAHEDRON_EDGES[6][2] = {{0, 1}, {0, 2}, {0, 3},
                                                     {1, 2}, {1, 3}, {2, 3}};

            /** @brief Packs a `[0, 1]` channel into ImGui's 8-bit representation. */
            int channel(float value) noexcept
            {
                const int scaled = static_cast<int>(clamp_unit(value) * 255.0f + 0.5f);
                return scaled > 255 ? 255 : scaled;
            }

            ImU32 pack(const HeatColour& colour, int alpha) noexcept
            {
                return IM_COL32(channel(colour.r), channel(colour.g), channel(colour.b), alpha);
            }
        } // namespace

        std::size_t draw_soft_body_overlay(
            const std::vector<Vector3>& positions,
            const std::vector<Simulation::SoftBodyElementSample>& elements,
            const Physics::SoftBodyMaterialT<Scalar>& material, SoftBodyDebugView view,
            const Render::CameraView& camera_view, const ImVec2& image_origin, float width,
            float height, ImDrawList* draw_list)
        {
            if (draw_list == nullptr || view == SoftBodyDebugView::Off || elements.empty() ||
                positions.empty())
                return 0;

            const Mat4 view_projection = mul(camera_view.projection, camera_view.view);
            // Pale blue-grey for the plain wireframe: it is a structural view drawn over
            // a shaded body, so it has to read as a scaffold rather than compete with the
            // surface. The heat views set their own colour per element.
            const ImU32 wireframe_colour = IM_COL32(150, 190, 230, 160);

            std::size_t drawn = 0;
            for (const Simulation::SoftBodyElementSample& element : elements)
            {
                bool addressable = true;
                for (int i = 0; i < 4 && addressable; ++i)
                    addressable = element.vertex[i] < positions.size();
                if (!addressable)
                    continue;

                const ImU32 colour =
                    view == SoftBodyDebugView::Wireframe
                        ? wireframe_colour
                        : pack(heat_colour(
                                   soft_body_element_intensity(view, element, material)),
                               200);

                for (const auto& edge : TETRAHEDRON_EDGES)
                {
                    ImVec2 a;
                    ImVec2 b;
                    // Both ends must project. An edge with one end behind the camera is
                    // skipped rather than clipped: clipping to the near plane is the right
                    // answer and a different job, while drawing it unclipped paints a
                    // stripe across the whole viewport.
                    if (!project_to_screen(view_projection, positions[element.vertex[edge[0]]],
                                           image_origin, width, height, a))
                        continue;
                    if (!project_to_screen(view_projection, positions[element.vertex[edge[1]]],
                                           image_origin, width, height, b))
                        continue;
                    draw_list->AddLine(a, b, colour, 1.0f);
                    ++drawn;
                }
            }
            return drawn;
        }
    } // namespace Editor
} // namespace SushiEngine
