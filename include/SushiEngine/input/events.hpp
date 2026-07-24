/**************************************************************************/
/* events.hpp                                                            */
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
 * @file events.hpp
 * @brief The one event record every input source emits, and the ids that key it.
 *
 * There is exactly one event type. A source (SDL translator, scripted replay, or a
 * synthesized virtual control) drains its native input into a flat list of
 * @ref InputEvent records; the @ref DeviceRegistry folds that list into state. Because
 * state is derived from events and never queried back from the source, all sources are
 * interchangeable — the property the whole design leans on for testing without hardware
 * (Liskov holds by construction). The record is a trivially-copyable struct with no
 * unions and no inheritance: events are data.
 */

#include <cstdint>

#include <SushiEngine/input/controls.hpp>

namespace SushiEngine
{
    namespace Input
    {
        /**
         * @brief A stable per-device slot id.
         *
         * Keyboard and mouse occupy fixed slots (@ref KEYBOARD_DEVICE / @ref MOUSE_DEVICE);
         * a gamepad claims the lowest free slot on connect and keeps it until unplugged, so
         * a binding or player assignment survives a reconnect of the same controller
         * ordering. A small integer rather than a pointer or GUID so it copies freely and
         * indexes an array.
         */
        using DeviceId = std::uint16_t;

        /** @brief The fixed slot the keyboard always occupies. */
        constexpr DeviceId KEYBOARD_DEVICE = 0;

        /** @brief The fixed slot the mouse always occupies. */
        constexpr DeviceId MOUSE_DEVICE = 1;

        /** @brief The first slot a hot-plugged gamepad may claim. */
        constexpr DeviceId FIRST_GAMEPAD_DEVICE = 2;

        /** @brief A device id that names no device (the null slot). */
        constexpr DeviceId INVALID_DEVICE = 0xFFFFu;

        /**
         * @brief What an @ref InputEvent reports.
         *
         * The `control` field of the event is interpreted against the type: a bare enum
         * ordinal for the button/key kinds, a bare @ref MouseAxis / @ref GamepadAxis
         * ordinal for the axis kinds, and a pointer id for the touch kinds. Ordinals are
         * internal to a running process (events are never serialized), so unlike the
         * control enums this may be reordered freely.
         */
        enum class EventType : std::uint16_t
        {
            KeyDown,
            KeyUp,
            MouseButtonDown,
            MouseButtonUp,
            MouseMove,   /**< `x`,`y` = new position; `value` unused. Relative deltas are folded from successive positions. */
            MouseWheel,  /**< `x` = horizontal notches, `y` = vertical notches. */
            GamepadButtonDown,
            GamepadButtonUp,
            GamepadAxisMotion, /**< `control` = @ref GamepadAxis ordinal, `value` = normalized [-1,1] (triggers [0,1]). */
            TouchDown,
            TouchMove,
            TouchUp,
            DeviceConnected,
            DeviceDisconnected,
        };

        /**
         * @brief One input occurrence — a flat record, the only shape crossing a source.
         *
         * All fields are always present; which ones carry meaning depends on @ref type
         * (documented per @ref EventType). Buttons and keys carry `value` 0 or 1; analog
         * axes carry the normalized value; pointer and touch events carry a position in
         * window pixels through `x`/`y`. `frame` is the host frame the source stamped the
         * event on, used by relative-axis accumulation and by replay.
         */
        struct InputEvent
        {
            DeviceId device = INVALID_DEVICE; /**< Which device slot produced this. */
            EventType type = EventType::KeyDown;
            std::uint16_t control = 0;        /**< Enum ordinal / pointer id, per @ref type. */
            float value = 0.0f;               /**< Button level (0/1) or normalized axis value. */
            float x = 0.0f;                   /**< Position or wheel-x, per @ref type, in window pixels. */
            float y = 0.0f;                   /**< Position or wheel-y, per @ref type, in window pixels. */
            std::uint64_t frame = 0;          /**< Host frame index the source stamped this on. */
        };

        /**
         * @brief A 2-component float value: the shape of an `Axis2D` action's result.
         *
         * The action layer computes in plain `float` on the host (§2.2); this is that
         * float pair, distinct from `UI::Vector2` (which is `Scalar`/double, for screen
         * geometry). It is the natural aggregate of an @ref InputEvent's `x`/`y`.
         */
        struct Vector2
        {
            float x = 0.0f;
            float y = 0.0f;

            /** @brief Componentwise sum. */
            constexpr Vector2 operator+(const Vector2& other) const noexcept
            {
                return Vector2{x + other.x, y + other.y};
            }

            /** @brief Componentwise difference. */
            constexpr Vector2 operator-(const Vector2& other) const noexcept
            {
                return Vector2{x - other.x, y - other.y};
            }

            /** @brief Scaling by a scalar. */
            constexpr Vector2 operator*(float scalar) const noexcept
            {
                return Vector2{x * scalar, y * scalar};
            }
        };
    } // namespace Input
} // namespace SushiEngine
