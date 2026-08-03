/**************************************************************************/
/* draw_list.hpp                                                          */
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
 * @file draw_list.hpp
 * @brief The contract between whatever builds a UI and whatever draws it.
 *
 * Separate from `ui.hpp` because the two sides have very different weights: building
 * a draw list needs the ECS world and the layout solver, while drawing one needs
 * nothing but these three structs. Keeping them apart is what lets the renderer's
 * scene-view seam name a UI draw list without pulling the whole ECS in behind it.
 *
 * Everything here is already resolved: rectangles are in screen pixels with a
 * top-left origin (see `rect.hpp`), so a consumer neither knows nor needs the anchor
 * and pivot rules that produced them.
 */

#include <cstdint>
#include <vector>

#include <SushiEngine/ui/components.hpp>
#include <SushiEngine/ui/rect.hpp>

namespace SushiEngine
{
    namespace UI
    {
        /** @brief One coloured rectangle to draw, in paint order. */
        struct UIDrawRect
        {
            Rect rect;
            Color color;
        };

        /**
         * @brief Where a run sits horizontally inside its rectangle.
         *
         * Part of the draw list rather than of the renderer because it is a property of the
         * element — a button's label is centred because that is what a button looks like —
         * while the renderer only knows how wide the glyphs happen to be. Vertical placement
         * has no such choice: a run is always centred in its box.
         */
        enum class TextAlign : std::uint32_t
        {
            Left = 0,
            Center = 1,
            Right = 2,
        };

        /** @brief One text run to draw: its rectangle, string, size, colour, and alignment. */
        struct UITextRun
        {
            Rect rect;
            char text[UI_TEXT_CAPACITY] = {0};
            std::uint32_t length = 0;
            Scalar font_size = 18;
            Color color;
            TextAlign align = TextAlign::Left;
        };

        /**
         * @brief A frame's UI geometry, back to front, for a 2D overlay pass to draw.
         *
         * Renderer-agnostic on purpose: it names no graphics API, so a Vulkan overlay,
         * a test, or a headless tool consume the same list. Rects and texts are each in
         * creation (paint) order, so a later element draws over an earlier one.
         */
        struct UIDrawList
        {
            std::vector<UIDrawRect> rects;
            std::vector<UITextRun> texts;

            /** @brief Whether there is nothing at all to draw. */
            bool empty() const noexcept { return rects.empty() && texts.empty(); }

            /** @brief Drops both lists, keeping their capacity for the next frame. */
            void clear() noexcept
            {
                rects.clear();
                texts.clear();
            }
        };
    } // namespace UI
} // namespace SushiEngine
