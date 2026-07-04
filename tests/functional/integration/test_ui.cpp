/**************************************************************************/
/* test_ui.cpp                                                           */
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

// Integration_UI: the UI façade over a real ECS world — canvas + button, driven by a
// scripted pointer, exercising layout, the button state machine, and click detection
// end to end. Uses the shared runtime because the UI entities live in a real World.

#include <gtest/gtest.h>

#include <SushiEngine/SushiEngine.hpp>

#include "test_helpers.hpp"

using namespace SushiEngine;

namespace
{
    UI::PointerInput pointer(Scalar x, Scalar y, bool down)
    {
        return UI::PointerInput{UI::Vector2{x, y}, down};
    }

    UI::RectTransform centred_button()
    {
        UI::RectTransform transform;
        transform.anchor_min = UI::Vector2{Scalar(0.5), Scalar(0.5)};
        transform.anchor_max = UI::Vector2{Scalar(0.5), Scalar(0.5)};
        transform.pivot = UI::Vector2{Scalar(0.5), Scalar(0.5)};
        transform.size_delta = UI::Vector2{Scalar(200), Scalar(80)};
        return transform;
    }
}

TEST(Integration_UI, PressAndReleaseInsideFiresClickOnce)
{
    World world(Harness::shared_runtime(), 64);
    UI::UI ui(world);
    const Entity canvas = ui.canvas();

    int clicks = 0;
    ui.button(canvas, centred_button(), "Play", [&clicks] { ++clicks; });

    const UI::Vector2 screen{Scalar(1280), Scalar(720)};

    ui.update(screen, pointer(Scalar(640), Scalar(360), false));
    ui.update(screen, pointer(Scalar(640), Scalar(360), true));
    ui.update(screen, pointer(Scalar(640), Scalar(360), false));

    EXPECT_EQ(clicks, 1);
    EXPECT_EQ(ui.clicks().size(), std::size_t(1));
}

TEST(Integration_UI, ReleaseOutsideDoesNotClick)
{
    World world(Harness::shared_runtime(), 64);
    UI::UI ui(world);
    const Entity canvas = ui.canvas();

    int clicks = 0;
    ui.button(canvas, centred_button(), "Play", [&clicks] { ++clicks; });

    const UI::Vector2 screen{Scalar(1280), Scalar(720)};

    ui.update(screen, pointer(Scalar(640), Scalar(360), true));  // press inside
    ui.update(screen, pointer(Scalar(10), Scalar(10), false));   // release outside

    EXPECT_EQ(clicks, 0);
    EXPECT_TRUE(ui.clicks().empty());
}

TEST(Integration_UI, ButtonStateFollowsPointer)
{
    World world(Harness::shared_runtime(), 64);
    UI::UI ui(world);
    const Entity canvas = ui.canvas();
    const Entity button = ui.button(canvas, centred_button(), "Play", nullptr);

    const UI::Vector2 screen{Scalar(1280), Scalar(720)};

    ui.update(screen, pointer(Scalar(10), Scalar(10), false)); // pointer away
    EXPECT_EQ(world.get<UI::UIButton>(button).state, UI::ButtonState::Normal);

    ui.update(screen, pointer(Scalar(640), Scalar(360), false)); // hovering
    EXPECT_EQ(world.get<UI::UIButton>(button).state, UI::ButtonState::Highlighted);

    ui.update(screen, pointer(Scalar(640), Scalar(360), true)); // pressed
    EXPECT_EQ(world.get<UI::UIButton>(button).state, UI::ButtonState::Pressed);
}
