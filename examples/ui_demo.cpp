/**************************************************************************/
/* ui_demo.cpp                                                           */
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

// A worked example of the ECS UI (canvas + button) authoring surface, headless and
// self-checking. It builds a canvas with a centred button, drives it with a scripted
// pointer stream, and asserts the two things a UI must get right: the anchor layout
// resolves the button to the expected screen rectangle, and a press-then-release
// inside the button fires its callback exactly once while a press-then-release-outside
// does not. No window is opened; the draw list the renderer would consume is built and
// its size checked, but nothing is rasterised.

#include <cmath>
#include <cstddef>
#include <cstdio>

#include <SushiEngine/SushiEngine.hpp>

using namespace SushiEngine;

namespace
{
    bool nearly_equal(Scalar a, Scalar b) noexcept
    {
        const Scalar diff = a - b;
        return diff > Scalar(-0.01) && diff < Scalar(0.01);
    }

    UI::PointerInput pointer(Scalar x, Scalar y, bool down) noexcept
    {
        return UI::PointerInput{UI::Vector2{x, y}, down};
    }
}

int main()
{
    auto runtime = SushiRuntime::API::Runtime::create();
    World world(runtime, 64);

    UI::UI ui(world);
    const Entity canvas = ui.canvas(UI::Vector2{Scalar(1280), Scalar(720)});

    // A 200x80 button pinned to the centre of the canvas.
    UI::RectTransform transform;
    transform.anchor_min = UI::Vector2{Scalar(0.5), Scalar(0.5)};
    transform.anchor_max = UI::Vector2{Scalar(0.5), Scalar(0.5)};
    transform.pivot = UI::Vector2{Scalar(0.5), Scalar(0.5)};
    transform.anchored_position = UI::Vector2{0, 0};
    transform.size_delta = UI::Vector2{Scalar(200), Scalar(80)};

    int click_count = 0;
    const Entity button = ui.button(canvas, transform, "Play", [&click_count] { ++click_count; });

    const UI::Vector2 screen{Scalar(1280), Scalar(720)};
    const Scalar center_x = Scalar(640);
    const Scalar center_y = Scalar(360);

    // Frame 1: hover with the button up. Frame 2: press inside. Frame 3: release
    // inside — a completed click.
    ui.update(screen, pointer(center_x, center_y, false));
    ui.update(screen, pointer(center_x, center_y, true));
    ui.update(screen, pointer(center_x, center_y, false));

    const UI::Rect rect = world.get<UI::ComputedRect>(button).rect;
    const bool layout_ok = nearly_equal(rect.min.x, center_x - Scalar(100)) &&
                           nearly_equal(rect.min.y, center_y - Scalar(40)) &&
                           nearly_equal(rect.size.x, Scalar(200)) &&
                           nearly_equal(rect.size.y, Scalar(80));
    const bool click_ok = click_count == 1;

    // Press inside, then release outside: not a click. The count must stay at 1.
    ui.update(screen, pointer(center_x, center_y, true));
    ui.update(screen, pointer(Scalar(10), Scalar(10), false));
    const bool miss_ok = click_count == 1;

    // The draw list carries the canvas has no graphic, the button's image, and its
    // label — so at least the button rect plus one text run.
    const UI::UIDrawList draw_list = ui.build_draw_list();
    const bool draw_ok = !draw_list.rects.empty() && !draw_list.texts.empty();

    std::printf("ui_demo: button rect=(%.1f,%.1f,%.1f,%.1f) clicks=%d\n", double(rect.min.x),
                double(rect.min.y), double(rect.size.x), double(rect.size.y), click_count);
    std::printf("layout_ok=%s click_ok=%s miss_ok=%s draw_ok=%s\n", layout_ok ? "true" : "false",
                click_ok ? "true" : "false", miss_ok ? "true" : "false", draw_ok ? "true" : "false");

    const bool ok = layout_ok && click_ok && miss_ok && draw_ok;
    std::printf("RESULT: %s\n", ok ? "OK" : "FAIL");
    return ok ? 0 : 1;
}
