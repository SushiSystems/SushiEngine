/**************************************************************************/
/* signed_distance_field.hpp                                              */
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
 * @file signed_distance_field.hpp
 * @brief Baking a triangle mesh into a cube of signed distances.
 *
 * This used to be `render/gi/mesh_sdf_baker.hpp`, where it was a global-illumination
 * private. It is not: a mesh's distance field is what a renderer cones-traces
 * through, what a collision narrowphase queries, and what a soft-body cooker uses to
 * decide what is inside a shape. Keeping it behind the renderer meant the physics
 * could only reach it by depending upward, and an offline cooker would have needed a
 * device to import an asset.
 *
 * The bake itself is unchanged — the same closest-point routine, the same sign from
 * the nearest triangle's geometric normal, the same two-voxel padding. What changed
 * is what it reads: a @ref Geometry::TriangleMeshView instead of the renderer's
 * vertex struct, so it no longer knows or cares who owns the geometry.
 */

#include <cstddef>
#include <cstdint>
#include <vector>

#include <SushiEngine/geometry/triangle_mesh.hpp>

namespace SushiEngine
{
    namespace Geometry
    {
        /**
         * @brief A cube of signed distances baked from a triangle mesh.
         *
         * @c distances holds @c resolution^3 signed distances in local units,
         * indexed as @c index = x + resolution * (y + resolution * z) with
         * @c x the fastest-varying axis. Voxel @c (x,y,z)'s centre in local space
         * is, per axis, @c aabb_min + (voxel + 0.5) * (aabb_max - aabb_min) / resolution.
         * Negative distances are inside the surface, positive outside. @c distances
         * is empty for a degenerate mesh, in which case @c resolution is zero.
         */
        struct SignedDistanceFieldBrick
        {
            float aabb_min[3];              // padded local-space AABB minimum
            float aabb_max[3];              // padded local-space AABB maximum
            std::int32_t resolution = 0;    // voxels per axis (cube)
            std::vector<float> distances;   // resolution^3 signed distances, local units
        };

        /**
         * @brief Bakes a signed distance field brick for a triangle mesh.
         *
         * For each voxel centre, the minimum unsigned distance to any triangle; the
         * sign comes from the geometric normal of the nearest triangle (negative when
         * the point is behind it, i.e. inside). The bounds are padded by two voxels so
         * the zero isosurface has clearance and rays approaching from outside read
         * positive distances first.
         *
         * Host-only, and one query per voxel against a @ref MeshDistanceQuery rather
         * than a sweep over every triangle: at the resolutions the cooking pipeline's
         * fidelity dial reaches, the product of the two counts is the difference
         * between a three-second cook and a coffee break.
         *
         * @param mesh       The geometry to bake; only positions and indices are read.
         * @param resolution Voxels per axis of the cube brick (e.g. 32).
         * @return The baked brick; distances empty if the mesh was degenerate.
         */
        SignedDistanceFieldBrick bake_signed_distance_field(const TriangleMeshView& mesh,
                                                            std::int32_t resolution);
    } // namespace Geometry
} // namespace SushiEngine
