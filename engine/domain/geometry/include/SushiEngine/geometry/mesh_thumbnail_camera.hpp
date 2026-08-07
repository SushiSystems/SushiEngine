/**************************************************************************/
/* mesh_thumbnail_camera.hpp                                              */
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
 * @file mesh_thumbnail_camera.hpp
 * @brief A world-space bounding box, and the fixed camera that frames one for a thumbnail.
 *
 * Neither the render module's `Geometry::MeshRegistry` mesh record nor its glTF importer
 * carries a min/max bounding box today (only a bounding radius) — this is the one new piece
 * of real algorithmic content the Project panel's model-thumbnail renderer needs, and it has
 * no device dependency, so it lives here rather than behind Vulkan, the same reason the rest
 * of this module exists.
 */

#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Geometry
    {
        /**
         * @brief An axis-aligned world-space bounding box, built incrementally via @ref expand_aabb.
         *
         * @c initialized distinguishes "never expanded" from "expanded to include the origin" —
         * a box that starts at @c {0,0,0}..{0,0,0} by construction would otherwise silently claim
         * a valid zero-size box before a single point had ever been added to it.
         */
        struct AABB3
        {
            Vector3 min{0.0, 0.0, 0.0};
            Vector3 max{0.0, 0.0, 0.0};
            bool initialized = false;
        };

        /**
         * @brief Grows @p bounds to include @p point, initializing it on the first call.
         * @param bounds The box to grow; read and written in place.
         * @param point  The point @p bounds must include afterward.
         */
        void expand_aabb(AABB3& bounds, const Vector3& point);

        /** @brief A camera's view and projection matrices, ready for @c CameraView. */
        struct ThumbnailCamera
        {
            Matrix4 view;
            Matrix4 projection;
        };

        /**
         * @brief A fixed three-quarter isometric camera that frames @p bounds with margin.
         *
         * Positions the eye along a fixed elevated diagonal direction from the box's center, at
         * a distance computed from the box's bounding-sphere radius and a fixed vertical field
         * of view, so the whole box fits the frame with room to spare — no per-model tuning.
         * @param bounds       The world-space box to frame. An uninitialized (never-expanded) box
         *   is treated as a single point at the origin.
         * @param aspect_ratio The target image's width divided by its height.
         * @return The camera's view and projection matrices.
         */
        ThumbnailCamera three_quarter_camera_for_bounds(const AABB3& bounds, float aspect_ratio);
    } // namespace Geometry
} // namespace SushiEngine
