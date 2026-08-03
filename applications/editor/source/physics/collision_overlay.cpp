/**************************************************************************/
/* collision_overlay.cpp                                                  */
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

#include "collision_overlay.hpp"

#include "../core/viewport_projection.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        std::size_t draw_collision_overlay(const std::vector<float>& segments, const Mat4& world,
                                           const SushiEngine::Render::CameraView& camera_view,
                                           const ImVec2& image_origin, float width, float height,
                                           ImDrawList* draw_list)
        {
            if (draw_list == nullptr || segments.size() < 6)
                return 0;

            const Mat4 view_projection = mul(camera_view.projection, camera_view.view);
            // Green, and thin. The collider is a second opinion about a shape the artist is
            // already looking at, so it has to be legible over the mesh without becoming the
            // thing they see — an overlay that dominates gets switched off and then shows
            // nothing at all.
            const ImU32 colour = IM_COL32(90, 230, 120, 200);

            std::size_t drawn = 0;
            const std::size_t count = segments.size() / 6;
            for (std::size_t i = 0; i < count; ++i)
            {
                const float* segment = segments.data() + i * 6;
                const Vector3 local_a{Scalar(segment[0]), Scalar(segment[1]), Scalar(segment[2])};
                const Vector3 local_b{Scalar(segment[3]), Scalar(segment[4]), Scalar(segment[5])};

                ImVec2 a;
                ImVec2 b;
                // Both ends have to project, and a segment with one end behind the camera is
                // skipped rather than clipped. Clipping to the near plane is the right answer
                // and it is a different job; drawing the unclipped line is the wrong one, and
                // it draws a stripe across the whole viewport.
                if (!project_to_screen(view_projection, transform_point(world, local_a),
                                       image_origin, width, height, a))
                    continue;
                if (!project_to_screen(view_projection, transform_point(world, local_b),
                                       image_origin, width, height, b))
                    continue;
                draw_list->AddLine(a, b, colour, 1.0f);
                ++drawn;
            }
            return drawn;
        }
    } // namespace Editor
} // namespace SushiEngine
