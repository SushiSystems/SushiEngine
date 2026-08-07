/**************************************************************************/
/* test_mesh_thumbnail_camera.cpp                                        */
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

// Unit_MeshThumbnailCamera: the model-thumbnail pipeline's bounding-box accumulation and its
// fixed three-quarter camera framing, checked without any device — expand_aabb against known
// point sequences, and the camera against properties that must hold for ANY valid bounds
// (looks away from a degenerate box, moves farther back for a larger box, and actually uses
// the aspect ratio it's given) rather than a single hand-computed matrix, since the exact
// matrix layout is an implementation detail of Matrix4::look_at/perspective this test does not
// own.

#include <cmath>

#include <gtest/gtest.h>

#include "SushiEngine/geometry/mesh_thumbnail_camera.hpp"

using namespace SushiEngine::Geometry;

TEST(Unit_MeshThumbnailCamera, ExpandAabbFromEmptyTakesFirstPoint)
{
    AABB3 bounds;
    expand_aabb(bounds, SushiEngine::Vector3{1.0, 2.0, 3.0});
    EXPECT_TRUE(bounds.initialized);
    EXPECT_DOUBLE_EQ(bounds.min.x, 1.0);
    EXPECT_DOUBLE_EQ(bounds.min.y, 2.0);
    EXPECT_DOUBLE_EQ(bounds.min.z, 3.0);
    EXPECT_DOUBLE_EQ(bounds.max.x, 1.0);
    EXPECT_DOUBLE_EQ(bounds.max.y, 2.0);
    EXPECT_DOUBLE_EQ(bounds.max.z, 3.0);
}

TEST(Unit_MeshThumbnailCamera, ExpandAabbGrowsToEncloseNewPoints)
{
    AABB3 bounds;
    expand_aabb(bounds, SushiEngine::Vector3{0.0, 0.0, 0.0});
    expand_aabb(bounds, SushiEngine::Vector3{-2.0, 5.0, 1.0});
    expand_aabb(bounds, SushiEngine::Vector3{3.0, -1.0, 0.5});
    EXPECT_DOUBLE_EQ(bounds.min.x, -2.0);
    EXPECT_DOUBLE_EQ(bounds.min.y, -1.0);
    EXPECT_DOUBLE_EQ(bounds.min.z, 0.0);
    EXPECT_DOUBLE_EQ(bounds.max.x, 3.0);
    EXPECT_DOUBLE_EQ(bounds.max.y, 5.0);
    EXPECT_DOUBLE_EQ(bounds.max.z, 1.0);
}

TEST(Unit_MeshThumbnailCamera, LargerBoundsProduceAFartherEyeThanSmallerBounds)
{
    AABB3 small_bounds;
    expand_aabb(small_bounds, SushiEngine::Vector3{-1.0, -1.0, -1.0});
    expand_aabb(small_bounds, SushiEngine::Vector3{1.0, 1.0, 1.0});

    AABB3 large_bounds;
    expand_aabb(large_bounds, SushiEngine::Vector3{-10.0, -10.0, -10.0});
    expand_aabb(large_bounds, SushiEngine::Vector3{10.0, 10.0, 10.0});

    const ThumbnailCamera small_camera = three_quarter_camera_for_bounds(small_bounds, 1.0f);
    const ThumbnailCamera large_camera = three_quarter_camera_for_bounds(large_bounds, 1.0f);

    // Both boxes are centered on the origin, so a look_at view matrix's translation column
    // (m[12..14] in this column-major layout: -dot(right,eye), -dot(up,eye), -dot(forward,eye))
    // grows in magnitude exactly as the eye moves farther from that shared center — this holds
    // regardless of the exact fixed viewing direction/margin constants chosen above.
    auto translation_magnitude = [](const SushiEngine::Matrix4& m)
    {
        return std::sqrt(m.m[12] * m.m[12] + m.m[13] * m.m[13] + m.m[14] * m.m[14]);
    };
    EXPECT_GT(translation_magnitude(large_camera.view), translation_magnitude(small_camera.view));
}

TEST(Unit_MeshThumbnailCamera, AspectRatioActuallyReachesTheProjectionMatrix)
{
    AABB3 bounds;
    expand_aabb(bounds, SushiEngine::Vector3{-1.0, -1.0, -1.0});
    expand_aabb(bounds, SushiEngine::Vector3{1.0, 1.0, 1.0});

    const ThumbnailCamera square = three_quarter_camera_for_bounds(bounds, 1.0f);
    const ThumbnailCamera wide = three_quarter_camera_for_bounds(bounds, 2.0f);

    // A deliberately weak assertion (not-equal rather than a specific element/direction): the
    // exact convention Matrix4::perspective uses for folding aspect ratio into its matrix is
    // that function's own implementation detail, not something this test should have to know.
    // What must hold regardless is that the aspect ratio parameter actually changes the result.
    EXPECT_NE(square.projection.m[0], wide.projection.m[0]);
}

TEST(Unit_MeshThumbnailCamera, DegenerateSinglePointBoundsProduceAFiniteCamera)
{
    AABB3 bounds;
    expand_aabb(bounds, SushiEngine::Vector3{5.0, 5.0, 5.0});

    const ThumbnailCamera camera = three_quarter_camera_for_bounds(bounds, 1.0f);
    for (int i = 0; i < 16; ++i)
    {
        EXPECT_TRUE(std::isfinite(camera.view.m[i])) << "view.m[" << i << "]";
        EXPECT_TRUE(std::isfinite(camera.projection.m[i])) << "projection.m[" << i << "]";
    }
}
