/**************************************************************************/
/* signed_distance_field.cpp                                              */
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

#include <SushiEngine/geometry/signed_distance_field.hpp>

#include <algorithm>

#include <SushiEngine/geometry/mesh_distance_query.hpp>

namespace SushiEngine
{
    namespace Geometry
    {
        SignedDistanceFieldBrick bake_signed_distance_field(const TriangleMeshView& mesh,
                                                            std::int32_t resolution)
        {
            SignedDistanceFieldBrick brick;

            if (!mesh.has_triangles() || resolution <= 0)
            {
                return brick;
            }

            MeshDistanceQuery surface;
            if (!surface.build(mesh))
            {
                return brick;
            }

            float aabb_min[3];
            float aabb_max[3];
            if (!compute_bounds(mesh, aabb_min, aabb_max))
            {
                return brick;
            }

            const float max_extent = std::max(aabb_max[0] - aabb_min[0],
                                              std::max(aabb_max[1] - aabb_min[1],
                                                       aabb_max[2] - aabb_min[2]));
            const float voxel_size = max_extent / static_cast<float>(resolution);
            const float padding = 2.0f * voxel_size;

            float cell[3];
            for (int axis = 0; axis < 3; ++axis)
            {
                aabb_min[axis] -= padding;
                aabb_max[axis] += padding;
                cell[axis] = (aabb_max[axis] - aabb_min[axis]) / static_cast<float>(resolution);
                brick.aabb_min[axis] = aabb_min[axis];
                brick.aabb_max[axis] = aabb_max[axis];
            }

            const std::size_t voxel_count = static_cast<std::size_t>(resolution) *
                                            static_cast<std::size_t>(resolution) *
                                            static_cast<std::size_t>(resolution);
            brick.resolution = resolution;
            brick.distances.resize(voxel_count);

            for (std::int32_t z = 0; z < resolution; ++z)
            {
                for (std::int32_t y = 0; y < resolution; ++y)
                {
                    for (std::int32_t x = 0; x < resolution; ++x)
                    {
                        const float centre[3] = {
                            aabb_min[0] + (static_cast<float>(x) + 0.5f) * cell[0],
                            aabb_min[1] + (static_cast<float>(y) + 0.5f) * cell[1],
                            aabb_min[2] + (static_cast<float>(z) + 0.5f) * cell[2]};

                        const std::size_t index =
                            static_cast<std::size_t>(x) +
                            static_cast<std::size_t>(resolution) *
                                (static_cast<std::size_t>(y) +
                                 static_cast<std::size_t>(resolution) *
                                     static_cast<std::size_t>(z));
                        brick.distances[index] = surface.signed_distance(centre);
                    }
                }
            }

            return brick;
        }
    } // namespace Geometry
} // namespace SushiEngine
