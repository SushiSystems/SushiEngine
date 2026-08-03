/**************************************************************************/
/* viewport_projection.hpp                                                */
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

#pragma once

/**
 * @file viewport_projection.hpp
 * @brief World point to viewport pixel, once, for everything that draws over the scene.
 *
 * Every overlay the editor draws needs this: the skeleton debug draw, the transform gizmo,
 * and the collider overlay. Three identical definitions of a projection would be three
 * places for a near-plane rule to drift, and the symptom of drift is one overlay drawing a
 * line behind the camera that another correctly clips.
 */

#include <imgui.h>

#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief Projects a world point into viewport pixels.
         *
         * @param view_projection The camera's projection times its view.
         * @param point           The world position.
         * @param origin          Top-left of the viewport image, in screen pixels.
         * @param width           Image width in pixels.
         * @param height          Image height in pixels.
         * @param out             Receives the pixel position when the point is in front.
         * @return False when the point is at or behind the near plane, in which case @p out is
         *         untouched — a caller that draws anyway would draw a line to nowhere.
         */
        inline bool project_to_screen(const Matrix4& view_projection, const Vector3& point,
                                      const ImVec2& origin, float width, float height, ImVec2& out)
        {
            const Scalar* m = view_projection.m;
            const Scalar x = m[0] * point.x + m[4] * point.y + m[8] * point.z + m[12];
            const Scalar y = m[1] * point.x + m[5] * point.y + m[9] * point.z + m[13];
            const Scalar w = m[3] * point.x + m[7] * point.y + m[11] * point.z + m[15];
            if (w <= Scalar(0.0001))
                return false;
            out.x = origin.x + static_cast<float>(x / w * Scalar(0.5) + Scalar(0.5)) * width;
            out.y = origin.y + static_cast<float>(y / w * Scalar(0.5) + Scalar(0.5)) * height;
            return true;
        }

        /**
         * @brief Transforms a point by an affine matrix, treating it as a position (w = 1).
         *
         * @param matrix The affine transform, column-major.
         * @param point  The position to transform.
         * @return The transformed position.
         */
        inline Vector3 transform_point(const Matrix4& matrix, const Vector3& point)
        {
            const Scalar* m = matrix.m;
            return Vector3{m[0] * point.x + m[4] * point.y + m[8] * point.z + m[12],
                           m[1] * point.x + m[5] * point.y + m[9] * point.z + m[13],
                           m[2] * point.x + m[6] * point.y + m[10] * point.z + m[14]};
        }
    } // namespace Editor
} // namespace SushiEngine
