/**************************************************************************/
/* controls.hpp                                                          */
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
 * @file controls.hpp
 * @brief Engine-owned control enums — the vocabulary a binding names, in data.
 *
 * Every control the input layer knows is enumerated here, never as an SDL code.
 * Two properties make these enums the anchor of the whole design:
 *
 *  - They are what serialized bindings store, so their ordinals are a wire format:
 *    each enumerator carries an explicit value and must never be renumbered. New
 *    controls append; they do not reorder.
 *  - `Key` is numbered by USB HID keyboard usage IDs (the same table SDL scancodes
 *    are built on), so the numbering is physical-position based and layout- and
 *    library-independent. A translator that maps an SDL scancode to a @ref Key is a
 *    reinterpretation, not a lookup table (`sushi_input` relies on this).
 *
 * The action layer above never branches on device family; these enums exist so a
 * @ref ControlPath can name a control once, in a binding, and be forgotten.
 */

#include <cstdint>

namespace SushiEngine
{
    namespace Input
    {
        /**
         * @brief The device family a control belongs to.
         *
         * Ordinals are serialized inside a @ref ControlPath; do not renumber. The
         * @c Virtual family is a synthesized source (on-screen touch controls, §2.4)
         * that emits gamepad-shaped controls — it is a device family so that gameplay
         * bindings cannot tell it from hardware.
         */
        enum class DeviceFamily : std::uint16_t
        {
            Keyboard = 0,
            Mouse    = 1,
            Gamepad  = 2,
            Touch    = 3,
            Virtual  = 4,
        };

        /**
         * @brief A physical key, numbered by its USB HID keyboard usage ID.
         *
         * The numbering is the HID usage-page-0x07 table, which is exactly the set of
         * values SDL exposes as `SDL_Scancode`. This is deliberate: the SDL translator
         * casts a scancode straight to this enum. Values are stable across keyboard
         * layouts (they name a position, not a glyph) and must never be renumbered.
         * Only the subset the engine currently binds is enumerated; more may be
         * appended at their HID ordinals.
         */
        enum class Key : std::uint16_t
        {
            Unknown = 0,

            A = 4,  B = 5,  C = 6,  D = 7,  E = 8,  F = 9,  G = 10, H = 11, I = 12,
            J = 13, K = 14, L = 15, M = 16, N = 17, O = 18, P = 19, Q = 20, R = 21,
            S = 22, T = 23, U = 24, V = 25, W = 26, X = 27, Y = 28, Z = 29,

            Num1 = 30, Num2 = 31, Num3 = 32, Num4 = 33, Num5 = 34,
            Num6 = 35, Num7 = 36, Num8 = 37, Num9 = 38, Num0 = 39,

            Return    = 40,
            Escape    = 41,
            Backspace = 42,
            Tab       = 43,
            Space     = 44,

            Minus        = 45,
            Equals       = 46,
            LeftBracket  = 47,
            RightBracket = 48,
            Backslash    = 49,
            Semicolon    = 51,
            Apostrophe   = 52,
            Grave        = 53,
            Comma        = 54,
            Period       = 55,
            Slash        = 56,
            CapsLock     = 57,

            F1 = 58, F2 = 59, F3  = 60, F4  = 61, F5  = 62, F6  = 63,
            F7 = 64, F8 = 65, F9  = 66, F10 = 67, F11 = 68, F12 = 69,

            PrintScreen = 70,
            ScrollLock  = 71,
            Pause       = 72,
            Insert      = 73,
            Home        = 74,
            PageUp      = 75,
            Delete      = 76,
            End         = 77,
            PageDown    = 78,

            Right = 79, Left = 80, Down = 81, Up = 82,

            NumLock        = 83, /**< NumLock / Clear. */
            KeypadDivide   = 84,
            KeypadMultiply = 85,
            KeypadMinus    = 86,
            KeypadPlus     = 87,
            KeypadEnter    = 88,
            Keypad1 = 89, Keypad2 = 90, Keypad3 = 91, Keypad4 = 92, Keypad5 = 93,
            Keypad6 = 94, Keypad7 = 95, Keypad8 = 96, Keypad9 = 97, Keypad0 = 98,
            KeypadPeriod   = 99,
            NonUsBackslash = 100, /**< The extra key next to Left-Shift on ISO layouts. */
            Application    = 101, /**< The context-menu ("apps") key. */
            KeypadEquals   = 103,

            F13 = 104, F14 = 105, F15 = 106, F16 = 107, F17 = 108, F18 = 109,
            F19 = 110, F20 = 111, F21 = 112, F22 = 113, F23 = 114, F24 = 115,

            Menu = 118,

            LeftControl  = 224,
            LeftShift    = 225,
            LeftAlt      = 226,
            LeftGui      = 227,
            RightControl = 228,
            RightShift   = 229,
            RightAlt     = 230,
            RightGui     = 231,
        };

        /**
         * @brief A mouse button. Ordinals follow the conventional 1-based physical order.
         *
         * `X1`/`X2` are the two extra side buttons. Serialized; do not renumber.
         */
        enum class MouseButton : std::uint16_t
        {
            Left   = 1,
            Middle = 2,
            Right  = 3,
            X1     = 4,
            X2     = 5,
        };

        /**
         * @brief A relative mouse axis — a per-frame delta, not a level.
         *
         * These bind as *relative* controls: their value is the movement since the last
         * fold and accumulates rather than sampling a position (§2.2). Ordinals are
         * serialized; do not renumber.
         */
        enum class MouseAxis : std::uint16_t
        {
            DeltaX = 0,
            DeltaY = 1,
            Wheel  = 2,
            WheelX = 3,
        };

        /**
         * @brief A gamepad button, in SDL's Xbox-style logical layout.
         *
         * Face buttons are named by position (south/east/west/north) rather than by any
         * vendor's letters, so a binding is layout-neutral. Ordinals match
         * `SDL_GameControllerButton` and are serialized; do not renumber.
         */
        enum class GamepadButton : std::uint16_t
        {
            South         = 0,  /**< Bottom face button (A on Xbox, Cross on PlayStation). */
            East          = 1,  /**< Right face button (B / Circle). */
            West          = 2,  /**< Left face button (X / Square). */
            North         = 3,  /**< Top face button (Y / Triangle). */
            Back          = 4,
            Guide         = 5,
            Start         = 6,
            LeftStick     = 7,  /**< Left stick pressed in (L3). */
            RightStick    = 8,  /**< Right stick pressed in (R3). */
            LeftShoulder  = 9,  /**< Left bumper (LB / L1). */
            RightShoulder = 10, /**< Right bumper (RB / R1). */
            DpadUp        = 11,
            DpadDown      = 12,
            DpadLeft      = 13,
            DpadRight     = 14,
        };

        /**
         * @brief A gamepad analog axis.
         *
         * The four scalar stick axes and the two triggers match
         * `SDL_GameControllerAxis` ordinals 0..5. @ref LeftStick / @ref RightStick are
         * engine-added 2D aliases with no SDL counterpart: they denote the (X, Y) pair
         * and are valid only on an `Axis2D` binding, where a single bind covers both
         * scalar axes and a radial deadzone renormalizes their magnitude. Serialized;
         * do not renumber.
         */
        enum class GamepadAxis : std::uint16_t
        {
            LeftStickX  = 0,
            LeftStickY  = 1,
            RightStickX = 2,
            RightStickY = 3,
            LeftTrigger = 4,
            RightTrigger = 5,

            LeftStick  = 6, /**< 2D alias for (LeftStickX, LeftStickY); Axis2D only. */
            RightStick = 7, /**< 2D alias for (RightStickX, RightStickY); Axis2D only. */
        };

        /** @brief The lifecycle phase of a touch pointer. Serialized; do not renumber. */
        enum class TouchPhase : std::uint16_t
        {
            Began     = 0,
            Moved     = 1,
            Ended     = 2,
            Cancelled = 3,
        };

        /**
         * @brief A control named by its family and ordinal — the atom a binding stores.
         *
         * `control` holds the ordinal of the family's enum (@ref Key, @ref MouseButton,
         * @ref GamepadButton, …). Two paths are equal when both fields match, which is
         * what the mapper's control-consumption set is keyed on. Trivially copyable so a
         * path can live inside a serialized binding without ceremony.
         */
        struct ControlPath
        {
            DeviceFamily family = DeviceFamily::Keyboard;
            std::uint16_t control = 0;

            /** @brief Value equality on both fields, for consumption bookkeeping. */
            constexpr bool operator==(const ControlPath& other) const noexcept
            {
                return family == other.family && control == other.control;
            }

            /** @brief Negation of @ref operator==. */
            constexpr bool operator!=(const ControlPath& other) const noexcept
            {
                return !(*this == other);
            }
        };

        /** @brief Builds a @ref ControlPath naming a keyboard @p key. */
        constexpr ControlPath control_of(Key key) noexcept
        {
            return ControlPath{DeviceFamily::Keyboard, static_cast<std::uint16_t>(key)};
        }

        /** @brief Builds a @ref ControlPath naming a mouse @p button. */
        constexpr ControlPath control_of(MouseButton button) noexcept
        {
            return ControlPath{DeviceFamily::Mouse, static_cast<std::uint16_t>(button)};
        }

        /**
         * @brief Builds a @ref ControlPath naming a relative mouse @p axis.
         *
         * Mouse axes and mouse buttons share the @c Mouse family; the ordinal ranges do
         * not overlap in practice because a consumer knows from an action's shape whether
         * it resolved a button or an axis. The distinct overloads exist for call-site
         * clarity, not for a separate family.
         */
        constexpr ControlPath control_of(MouseAxis axis) noexcept
        {
            return ControlPath{DeviceFamily::Mouse,
                               static_cast<std::uint16_t>(0x0100u | static_cast<std::uint16_t>(axis))};
        }

        /** @brief Builds a @ref ControlPath naming a gamepad @p button. */
        constexpr ControlPath control_of(GamepadButton button) noexcept
        {
            return ControlPath{DeviceFamily::Gamepad, static_cast<std::uint16_t>(button)};
        }

        /**
         * @brief Builds a @ref ControlPath naming a gamepad @p axis.
         *
         * Gamepad axes and buttons share the @c Gamepad family; axis ordinals are offset
         * by 0x0100 so a path unambiguously names one or the other.
         */
        constexpr ControlPath control_of(GamepadAxis axis) noexcept
        {
            return ControlPath{DeviceFamily::Gamepad,
                               static_cast<std::uint16_t>(0x0100u | static_cast<std::uint16_t>(axis))};
        }

        /** @brief The ordinal offset that distinguishes an axis path from a button path. */
        constexpr std::uint16_t AXIS_CONTROL_FLAG = 0x0100u;

        /** @brief Whether @p path names an analog axis rather than a button. */
        constexpr bool is_axis_control(const ControlPath& path) noexcept
        {
            return (path.control & AXIS_CONTROL_FLAG) != 0;
        }

        /** @brief The bare axis ordinal encoded in @p path (its enum value). */
        constexpr std::uint16_t axis_ordinal(const ControlPath& path) noexcept
        {
            return static_cast<std::uint16_t>(path.control & ~AXIS_CONTROL_FLAG);
        }
    } // namespace Input
} // namespace SushiEngine
