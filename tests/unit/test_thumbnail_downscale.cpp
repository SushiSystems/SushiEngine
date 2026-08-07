/**************************************************************************/
/* test_thumbnail_downscale.cpp                                          */
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

// Unit_ThumbnailDownscale: the Project panel thumbnail pipeline's box filter, checked against
// hand-computed cases rather than against itself — a uniform field, a checkerboard whose
// average is known exactly, and a non-square source, so a transposed width/height would fail
// the third case even though the first two would still pass by symmetry.

#include <gtest/gtest.h>

#include "SushiEngine/imaging/box_downscale.hpp"

using SushiEngine::Imaging::box_downscale_rgba8;

TEST(Unit_ThumbnailDownscale, UniformColorStaysUniform)
{
    // 4x4 solid (200, 100, 50, 255), downscaled to 2x2: every output texel averages four
    // identical source texels, so every output texel must equal the input color exactly.
    std::vector<std::uint8_t> source(4 * 4 * 4);
    for (std::size_t i = 0; i < source.size(); i += 4)
    {
        source[i + 0] = 200;
        source[i + 1] = 100;
        source[i + 2] = 50;
        source[i + 3] = 255;
    }

    const std::vector<std::uint8_t> result = box_downscale_rgba8(source.data(), 4, 4, 2, 2);
    ASSERT_EQ(result.size(), std::size_t(2 * 2 * 4));
    for (std::size_t i = 0; i < result.size(); i += 4)
    {
        EXPECT_EQ(result[i + 0], 200);
        EXPECT_EQ(result[i + 1], 100);
        EXPECT_EQ(result[i + 2], 50);
        EXPECT_EQ(result[i + 3], 255);
    }
}

TEST(Unit_ThumbnailDownscale, CheckerboardAveragesToMidGray)
{
    // 2x2 source: black, white, white, black (row-major). Downscaled to 1x1, the single
    // output texel is the box average of all four: (0 + 255 + 255 + 0) / 4 = 127 (integer
    // division truncates 127.5 down).
    const std::uint8_t source[2 * 2 * 4] = {
        0,   0,   0,   255, // top-left: black
        255, 255, 255, 255, // top-right: white
        0,   0,   0,   255, // bottom-left: black
        255, 255, 255, 255  // bottom-right: white
    };

    const std::vector<std::uint8_t> result = box_downscale_rgba8(source, 2, 2, 1, 1);
    ASSERT_EQ(result.size(), std::size_t(1 * 1 * 4));
    EXPECT_EQ(result[0], 127);
    EXPECT_EQ(result[1], 127);
    EXPECT_EQ(result[2], 127);
    EXPECT_EQ(result[3], 255);
}

TEST(Unit_ThumbnailDownscale, NonSquareSourceMapsWidthAndHeightIndependently)
{
    // 4-wide, 2-tall source, downscaled to 2x1: a width/height swap bug would either crash
    // (out-of-bounds read past a 4x2 buffer treated as 2x4) or silently average the wrong
    // texels together. Left half is red, right half is blue.
    std::uint8_t source[4 * 2 * 4];
    for (std::uint32_t y = 0; y < 2; ++y)
    {
        for (std::uint32_t x = 0; x < 4; ++x)
        {
            std::uint8_t* texel = source + (y * 4 + x) * 4;
            const bool left_half = x < 2;
            texel[0] = left_half ? 255 : 0;
            texel[1] = 0;
            texel[2] = left_half ? 0 : 255;
            texel[3] = 255;
        }
    }

    const std::vector<std::uint8_t> result = box_downscale_rgba8(source, 4, 2, 2, 1);
    ASSERT_EQ(result.size(), std::size_t(2 * 1 * 4));
    // Left output texel: pure red.
    EXPECT_EQ(result[0], 255);
    EXPECT_EQ(result[2], 0);
    // Right output texel: pure blue.
    EXPECT_EQ(result[4], 0);
    EXPECT_EQ(result[6], 255);
}
