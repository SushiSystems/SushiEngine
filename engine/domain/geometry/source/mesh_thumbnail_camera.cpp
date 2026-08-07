/**************************************************************************/
/* mesh_thumbnail_camera.cpp                                              */
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

#include "SushiEngine/geometry/mesh_thumbnail_camera.hpp"

#include <algorithm>
#include <cmath>

namespace SushiEngine
{
    namespace Geometry
    {
        namespace
        {
            // 35 degrees: tight enough that a model fills most of the frame without its
            // silhouette clipping the corners at the fixed three-quarter angle below.
            constexpr float THUMBNAIL_FOV_Y_RADIANS = 0.6109f;
            // Extra distance past the tightest fit, so the model's silhouette never touches
            // the frame edge.
            constexpr double THUMBNAIL_MARGIN_FACTOR = 1.35;
            // How far above the horizontal the fixed viewing direction sits; larger values
            // look down on the model more steeply.
            constexpr double THUMBNAIL_ELEVATION = 0.8;

            double dot3(const Vector3& a, const Vector3& b)
            {
                return a.x * b.x + a.y * b.y + a.z * b.z;
            }

            double length3(const Vector3& v)
            {
                return std::sqrt(dot3(v, v));
            }

            Vector3 normalized3(const Vector3& v)
            {
                const double len = length3(v);
                if (len <= 0.0)
                    return Vector3{0.0, 0.0, 1.0};
                return Vector3{v.x / len, v.y / len, v.z / len};
            }
        } // namespace

        void expand_aabb(AABB3& bounds, const Vector3& point)
        {
            if (!bounds.initialized)
            {
                bounds.min = point;
                bounds.max = point;
                bounds.initialized = true;
                return;
            }
            bounds.min.x = std::min(bounds.min.x, point.x);
            bounds.min.y = std::min(bounds.min.y, point.y);
            bounds.min.z = std::min(bounds.min.z, point.z);
            bounds.max.x = std::max(bounds.max.x, point.x);
            bounds.max.y = std::max(bounds.max.y, point.y);
            bounds.max.z = std::max(bounds.max.z, point.z);
        }

        ThumbnailCamera three_quarter_camera_for_bounds(const AABB3& bounds, float aspect_ratio)
        {
            const Vector3 center{
                (bounds.min.x + bounds.max.x) * 0.5,
                (bounds.min.y + bounds.max.y) * 0.5,
                (bounds.min.z + bounds.max.z) * 0.5};
            const Vector3 half_extents{
                (bounds.max.x - bounds.min.x) * 0.5,
                (bounds.max.y - bounds.min.y) * 0.5,
                (bounds.max.z - bounds.min.z) * 0.5};
            // A conservative bounding-sphere radius from the box's half-diagonal; never zero,
            // so a degenerate (single-point) box still gets a sane, non-zero camera distance.
            const double radius = std::max(length3(half_extents), 0.001);

            const Vector3 direction = normalized3(Vector3{1.0, THUMBNAIL_ELEVATION, 1.0});
            const double half_fov = static_cast<double>(THUMBNAIL_FOV_Y_RADIANS) * 0.5;
            const double distance = (radius / std::sin(half_fov)) * THUMBNAIL_MARGIN_FACTOR;

            const Vector3 eye{
                center.x + direction.x * distance,
                center.y + direction.y * distance,
                center.z + direction.z * distance};
            const Vector3 up{0.0, 1.0, 0.0};

            ThumbnailCamera camera;
            camera.view = look_at(eye, center, up);
            const float near_plane = static_cast<float>(std::max(distance - radius * 1.5, 0.01));
            const float far_plane = static_cast<float>(distance + radius * 4.0);
            camera.projection =
                perspective(THUMBNAIL_FOV_Y_RADIANS, aspect_ratio, near_plane, far_plane);
            return camera;
        }
    } // namespace Geometry
} // namespace SushiEngine
