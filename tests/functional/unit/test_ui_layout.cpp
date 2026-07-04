/**************************************************************************/
/* test_ui_layout.cpp                                                    */
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
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

// Unit_UILayout: the UI anchor model (ui/layout.hpp resolve_rect) in isolation — a
// pure function of a parent rect and a RectTransform, so no runtime or world is
// needed. Covers the three cases that define UGUI layout: a point anchor (size is
// size_delta, positioned by pivot + anchored_position), a full stretch, and a
// stretch inset by a negative size_delta.

#include <gtest/gtest.h>

#include <SushiEngine/ui/layout.hpp>

using namespace SushiEngine;

namespace
{
    constexpr Scalar TOLERANCE = Scalar(1e-3);

    UI::Rect canvas_rect()
    {
        return UI::Rect{UI::Vector2{0, 0}, UI::Vector2{Scalar(1280), Scalar(720)}};
    }
}

TEST(Unit_UILayout, PointAnchorCentresBySizeDelta)
{
    UI::RectTransform transform;
    transform.anchor_min = UI::Vector2{Scalar(0.5), Scalar(0.5)};
    transform.anchor_max = UI::Vector2{Scalar(0.5), Scalar(0.5)};
    transform.pivot = UI::Vector2{Scalar(0.5), Scalar(0.5)};
    transform.size_delta = UI::Vector2{Scalar(200), Scalar(80)};

    const UI::Rect rect = UI::resolve_rect(canvas_rect(), transform);
    EXPECT_NEAR(double(rect.min.x), 540.0, double(TOLERANCE));
    EXPECT_NEAR(double(rect.min.y), 320.0, double(TOLERANCE));
    EXPECT_NEAR(double(rect.size.x), 200.0, double(TOLERANCE));
    EXPECT_NEAR(double(rect.size.y), 80.0, double(TOLERANCE));
}

TEST(Unit_UILayout, FullStretchFillsParent)
{
    UI::RectTransform transform;
    transform.anchor_min = UI::Vector2{0, 0};
    transform.anchor_max = UI::Vector2{1, 1};
    transform.size_delta = UI::Vector2{0, 0};

    const UI::Rect rect = UI::resolve_rect(canvas_rect(), transform);
    EXPECT_NEAR(double(rect.min.x), 0.0, double(TOLERANCE));
    EXPECT_NEAR(double(rect.min.y), 0.0, double(TOLERANCE));
    EXPECT_NEAR(double(rect.size.x), 1280.0, double(TOLERANCE));
    EXPECT_NEAR(double(rect.size.y), 720.0, double(TOLERANCE));
}

TEST(Unit_UILayout, StretchWithNegativeSizeDeltaInsets)
{
    UI::RectTransform transform;
    transform.anchor_min = UI::Vector2{0, 0};
    transform.anchor_max = UI::Vector2{1, 1};
    transform.pivot = UI::Vector2{Scalar(0.5), Scalar(0.5)};
    transform.size_delta = UI::Vector2{Scalar(-20), Scalar(-20)};

    const UI::Rect rect = UI::resolve_rect(canvas_rect(), transform);
    EXPECT_NEAR(double(rect.min.x), 10.0, double(TOLERANCE));
    EXPECT_NEAR(double(rect.min.y), 10.0, double(TOLERANCE));
    EXPECT_NEAR(double(rect.size.x), 1260.0, double(TOLERANCE));
    EXPECT_NEAR(double(rect.size.y), 700.0, double(TOLERANCE));
}
