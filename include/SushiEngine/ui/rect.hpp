/**************************************************************************/
/* rect.hpp                                                              */
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
 * @file rect.hpp
 * @brief The 2D value types the UI layer works in: a screen-space vector, rect, and colour.
 *
 * The UI uses a top-left origin with x to the right and y downward — the natural
 * frame for screen pixels and pointer coordinates. All three types are trivially
 * copyable so they can be stored directly in ECS components.
 */

#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace UI
    {
        /** @brief A 2D vector in UI (screen) space: x right, y down. */
        struct Vector2
        {
            Scalar x = 0;
            Scalar y = 0;

            /** @brief Componentwise sum. */
            constexpr Vector2 operator+(const Vector2& o) const noexcept
            {
                return Vector2{x + o.x, y + o.y};
            }

            /** @brief Componentwise difference. */
            constexpr Vector2 operator-(const Vector2& o) const noexcept
            {
                return Vector2{x - o.x, y - o.y};
            }

            /** @brief Scaling by a scalar. */
            constexpr Vector2 operator*(Scalar s) const noexcept
            {
                return Vector2{x * s, y * s};
            }
        };

        /**
         * @brief An axis-aligned rectangle: its top-left corner and its size.
         *
         * `min` is the top-left corner (smallest x, smallest y under a y-down frame);
         * `size` is width and height. The bottom-right corner is `min + size`.
         */
        struct Rect
        {
            Vector2 min;
            Vector2 size;

            /** @brief The bottom-right corner (`min + size`). */
            constexpr Vector2 max() const noexcept { return Vector2{min.x + size.x, min.y + size.y}; }

            /** @brief The centre point. */
            constexpr Vector2 center() const noexcept
            {
                return Vector2{min.x + size.x * Scalar(0.5), min.y + size.y * Scalar(0.5)};
            }

            /**
             * @brief Whether @p point lies within the rectangle (inclusive of the edges).
             * @param point The point to test, in the same UI space.
             * @return True when the point is inside or on the border.
             */
            constexpr bool contains(const Vector2& point) const noexcept
            {
                return point.x >= min.x && point.x <= min.x + size.x && point.y >= min.y &&
                       point.y <= min.y + size.y;
            }
        };

        /** @brief A straight RGBA colour in [0, 1], stored in an ECS component. */
        struct Color
        {
            Scalar r = 1;
            Scalar g = 1;
            Scalar b = 1;
            Scalar a = 1;
        };
    } // namespace UI
} // namespace SushiEngine
