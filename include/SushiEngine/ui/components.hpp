/**************************************************************************/
/* components.hpp                                                        */
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
 * @file components.hpp
 * @brief The ECS component set for SushiEngine's retained UI (Unity UGUI-shaped).
 *
 * A UI is built from ordinary entities: a `Canvas` root and, under it, a tree of
 * elements each carrying a `RectTransform` (its anchor-based layout) plus one or more
 * graphics — a `UIImage`, a `UIText`, and/or a `UIButton`. Parent links are a
 * `UIParent` component, and layout writes each element's resolved screen rectangle
 * into a `ComputedRect`. Every type here is trivially copyable, so UI is stored in
 * the same archetype chunks as the rest of the world and travels through scene
 * serialization and (in a networked game) snapshots like any other component.
 */

#include <cstdint>

#include <SushiEngine/ecs/entity.hpp>
#include <SushiEngine/ui/rect.hpp>

namespace SushiEngine
{
    namespace UI
    {
        /**
         * @brief A UGUI-style anchor-based layout rectangle.
         *
         * `anchor_min`/`anchor_max` are fractions of the parent rect (0 = its top-left
         * corner, 1 = its bottom-right), so an element can pin to a corner, stretch
         * across an edge, or fill the parent. `pivot` is the element's own reference
         * point (also fractional). `anchored_position` offsets the pivot from the
         * anchor, in pixels. `size_delta` is the size *added to* the anchor span: with
         * a point anchor (min == max) it is the element's size outright; with a
         * stretched anchor it insets or grows the stretched rect. This is exactly Unity
         * `RectTransform`'s model (see @ref resolve_rect for the resolution formula).
         */
        struct RectTransform
        {
            Vector2 anchor_min{Vector2{Scalar(0.5), Scalar(0.5)}};
            Vector2 anchor_max{Vector2{Scalar(0.5), Scalar(0.5)}};
            Vector2 pivot{Vector2{Scalar(0.5), Scalar(0.5)}};
            Vector2 anchored_position{Vector2{0, 0}};
            Vector2 size_delta{Vector2{Scalar(100), Scalar(100)}};
        };

        /**
         * @brief How a Canvas reconciles its actual screen size with its authored resolution.
         *
         * Mirrors Unity `CanvasScaler`'s "Scale With Screen Size" mode: `ConstantPixelSize`
         * lays child elements out in real screen pixels (today's behaviour, and the
         * default); `ScaleWithScreenSize` instead solves the canvas as if it were
         * `reference_size` and blends the width- and height-based scale factors per
         * `match_width_or_height` (see @ref Canvas).
         */
        enum class CanvasScaleMode : std::uint32_t
        {
            ConstantPixelSize,
            ScaleWithScreenSize,
        };

        /**
         * @brief Marks an entity as a UI root and names its design resolution.
         *
         * A Canvas's own `ComputedRect` is the screen rectangle the layout is solved
         * against; `reference_size` is the resolution the UI was authored at. In
         * `ScaleWithScreenSize` mode the canvas's effective size for child layout is
         * `reference_size` scaled by a factor blended between the width ratio and the
         * height ratio of actual-to-reference size, weighted by `match_width_or_height`
         * (0 = match width only, 1 = match height only) — exactly Unity's formula.
         */
        struct Canvas
        {
            Vector2 reference_size{Vector2{Scalar(1280), Scalar(720)}};
            CanvasScaleMode scale_mode = CanvasScaleMode::ConstantPixelSize;
            Scalar match_width_or_height = Scalar(0);
        };

        /**
         * @brief The parent link that gives the UI its tree, as an ordinary component.
         *
         * A root element (directly under a Canvas) names the Canvas here; the Canvas
         * itself carries no `UIParent`. The layout resolves an element against its
         * parent's `ComputedRect`, so parents must be laid out before their children —
         * the builder creates them in that order.
         */
        struct UIParent
        {
            Entity parent;
        };

        /** @brief The element's resolved screen rectangle, written by the layout pass. */
        struct ComputedRect
        {
            Rect rect;
        };

        /** @brief A solid-colour graphic filling the element's rect (a panel/background). */
        struct UIImage
        {
            Color color{Color{Scalar(1), Scalar(1), Scalar(1), Scalar(1)}};
        };

        /** @brief The maximum number of characters a `UIText` stores inline. */
        constexpr std::size_t UI_TEXT_CAPACITY = 64;

        /**
         * @brief A short text label, stored inline so it stays a trivially copyable component.
         *
         * The string lives in a fixed buffer (`UI_TEXT_CAPACITY` chars) rather than a
         * `std::string`, so the component remains device-storable and snapshot-friendly
         * like every other. Longer text is truncated; rich text is out of scope. Set it
         * through @ref set_text so the length and null terminator stay consistent.
         */
        struct UIText
        {
            char text[UI_TEXT_CAPACITY] = {0};
            std::uint32_t length = 0;
            Scalar font_size = 18;
            Color color{Color{0, 0, 0, 1}};
        };

        /**
         * @brief Copies @p source into @p label, truncating to the inline capacity.
         * @param label  The label to write.
         * @param source A null-terminated C string.
         */
        inline void set_text(UIText& label, const char* source) noexcept
        {
            std::uint32_t i = 0;
            for (; source != nullptr && source[i] != '\0' && i + 1 < UI_TEXT_CAPACITY; ++i)
                label.text[i] = source[i];
            label.text[i] = '\0';
            label.length = i;
        }

        /** @brief The visual/interaction state a button is in for the current frame. */
        enum class ButtonState : std::uint32_t
        {
            Normal,      /**< Idle, pointer elsewhere. */
            Highlighted, /**< Pointer hovering over it. */
            Pressed,     /**< Pointer pressed down on it. */
            Disabled,    /**< Not interactable. */
        };

        /**
         * @brief A clickable button: its state and the per-state tint of its graphic.
         *
         * Mirrors UGUI's `Button` + `ColorBlock`: the interaction layer sets `state`
         * from the pointer each frame and tints `target_graphic`'s `UIImage` with the
         * matching colour. `target_graphic` is usually the button's own entity. A
         * click (press and release both inside the button) is reported through the UI
         * event queue, not stored here.
         */
        struct UIButton
        {
            ButtonState state = ButtonState::Normal;
            bool interactable = true;
            Entity target_graphic;
            Color normal_color{Color{Scalar(0.85), Scalar(0.85), Scalar(0.85), 1}};
            Color highlighted_color{Color{Scalar(0.95), Scalar(0.95), Scalar(0.95), 1}};
            Color pressed_color{Color{Scalar(0.7), Scalar(0.7), Scalar(0.7), 1}};
            Color disabled_color{Color{Scalar(0.5), Scalar(0.5), Scalar(0.5), Scalar(0.6)}};
        };

        /** @brief The tint for @p button's current @p state, per its colour block. */
        inline Color button_state_color(const UIButton& button) noexcept
        {
            switch (button.state)
            {
                case ButtonState::Highlighted: return button.highlighted_color;
                case ButtonState::Pressed: return button.pressed_color;
                case ButtonState::Disabled: return button.disabled_color;
                case ButtonState::Normal:
                default: return button.normal_color;
            }
        }
    } // namespace UI
} // namespace SushiEngine
