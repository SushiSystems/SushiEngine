/**************************************************************************/
/* device_registry.hpp                                                   */
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
 * @file device_registry.hpp
 * @brief Per-device state, folded from events — never queried back from a source.
 *
 * The registry is the single owner of "what is held right now". Each host frame the
 * manager calls @ref DeviceRegistry::begin_frame (which zeroes the relative
 * accumulators — a delta is a per-frame quantity) and then @ref DeviceRegistry::ingest
 * for every drained event. The action layer reads state exclusively through
 * @ref DeviceRegistry::control_value / @ref DeviceRegistry::control_down, so it makes no
 * SDL call and cannot tell a scripted event from a hardware one.
 *
 * Level state (keys, buttons, absolute stick/trigger values, pointer positions)
 * persists across frames; relative state (mouse delta, wheel) accumulates within a
 * frame and is cleared at its start. This is exactly the split the tick-boundary
 * sampler (§2.3) needs: levels are latest-wins, deltas are sums-since-last.
 */

#include <array>
#include <cstdint>

#include <SushiEngine/input/controls.hpp>
#include <SushiEngine/input/events.hpp>

namespace SushiEngine
{
    namespace Input
    {
        /** @brief Largest number of simultaneously tracked gamepads. */
        constexpr DeviceId MAX_GAMEPADS = 8;

        /** @brief Largest number of concurrent touch pointers (§2.4). */
        constexpr int MAX_TOUCH_POINTS = 8;

        /** @brief Number of scalar gamepad axes (four stick axes + two triggers). */
        constexpr int GAMEPAD_AXIS_COUNT = 6;

        /**
         * @brief Derived state for every input device, rebuilt from the event stream.
         *
         * A plain state bag with two mutators (@ref begin_frame, @ref ingest) and pure
         * queries. It holds no history and no edges — edge detection is the mapper's job,
         * computed from this level state frame to frame — which keeps the registry a
         * single-responsibility store.
         */
        class DeviceRegistry
        {
            public:
                /**
                 * @brief Starts a new host frame: clears relative accumulators.
                 *
                 * Mouse delta and wheel are per-frame quantities; zeroing them here means
                 * a frame with no motion reports zero movement, and a frame with several
                 * motion events reports their sum. Level state is untouched.
                 */
                void begin_frame() noexcept
                {
                    mouse_delta_ = Vector2{};
                    wheel_ = Vector2{};
                    key_pressed_edges_ = std::array<std::uint64_t, 4>{};
                    key_released_edges_ = std::array<std::uint64_t, 4>{};
                    mouse_pressed_edges_ = 0;
                    mouse_released_edges_ = 0;
                    for (GamepadState& state : gamepads_)
                    {
                        state.pressed_edges = 0;
                        state.released_edges = 0;
                    }
                }

                /**
                 * @brief Folds one event into device state.
                 * @param event The event to apply.
                 */
                void ingest(const InputEvent& event) noexcept
                {
                    switch (event.type)
                    {
                        case EventType::KeyDown:
                            set_key(event.control, true);
                            break;
                        case EventType::KeyUp:
                            set_key(event.control, false);
                            break;
                        case EventType::MouseButtonDown:
                            set_mouse_button(event.control, true);
                            fold_mouse_as_pointer_button(event.control, true, event.x, event.y);
                            break;
                        case EventType::MouseButtonUp:
                            set_mouse_button(event.control, false);
                            fold_mouse_as_pointer_button(event.control, false, event.x, event.y);
                            break;
                        case EventType::MouseMove:
                            fold_mouse_move(event.x, event.y);
                            if (mouse_as_pointer_)
                                pointers_[0].position = mouse_position_;
                            break;
                        case EventType::MouseWheel:
                            wheel_.x += event.x;
                            wheel_.y += event.y;
                            break;
                        case EventType::GamepadButtonDown:
                            set_gamepad_button(event.device, event.control, true);
                            break;
                        case EventType::GamepadButtonUp:
                            set_gamepad_button(event.device, event.control, false);
                            break;
                        case EventType::GamepadAxisMotion:
                            set_gamepad_axis(event.device, event.control, event.value);
                            break;
                        case EventType::TouchDown:
                            set_pointer(event.control, event.x, event.y, true);
                            break;
                        case EventType::TouchMove:
                            set_pointer(event.control, event.x, event.y, true);
                            break;
                        case EventType::TouchUp:
                            set_pointer(event.control, event.x, event.y, false);
                            break;
                        case EventType::DeviceConnected:
                            set_connected(event.device, true);
                            break;
                        case EventType::DeviceDisconnected:
                            set_connected(event.device, false);
                            break;
                    }
                }

                /**
                 * @brief Resolves the level/analog value a binding's control currently holds.
                 *
                 * Buttons and keys return 0 or 1; analog axes return their processed-free raw
                 * value (deadzone/scale live in the binding, not here). The @p device selects
                 * which physical device answers for gamepad/touch families; keyboard and mouse
                 * ignore it (they are singletons).
                 *
                 * @param path   The control to read.
                 * @param device The device slot to read a gamepad/touch control from.
                 * @return The control's current value.
                 */
                float control_value(const ControlPath& path, DeviceId device) const noexcept
                {
                    switch (path.family)
                    {
                        case DeviceFamily::Keyboard:
                            return key_down(path.control) ? 1.0f : 0.0f;
                        case DeviceFamily::Mouse:
                            if (is_axis_control(path))
                                return mouse_axis_value(axis_ordinal(path));
                            return mouse_button_down(path.control) ? 1.0f : 0.0f;
                        case DeviceFamily::Gamepad:
                        case DeviceFamily::Virtual:
                            if (is_axis_control(path))
                                return gamepad_axis(device, axis_ordinal(path));
                            return gamepad_button(device, path.control) ? 1.0f : 0.0f;
                        case DeviceFamily::Touch:
                            return 0.0f;
                    }
                    return 0.0f;
                }

                /**
                 * @brief Whether a binding's control is currently held (button/key level).
                 * @param path   The control to read.
                 * @param device The device slot for gamepad/touch families.
                 * @return True if the control is a pressed button or a key that is down.
                 */
                bool control_down(const ControlPath& path, DeviceId device) const noexcept
                {
                    if (is_axis_control(path))
                        return control_value(path, device) != 0.0f;
                    return control_value(path, device) != 0.0f;
                }

                /**
                 * @brief Whether @p path's button saw a press this host frame, even if released again.
                 *
                 * Folded from the frame's events independently of the final level, so a press and
                 * release within one host frame still reports a press. This is what lets the
                 * tick-boundary sampler surface a sub-frame tap that the frame-to-frame level diff
                 * would miss. Axis controls have no edge and always report false.
                 *
                 * @param path   The button control to test.
                 * @param device The device slot for gamepad-family controls.
                 * @return True if a down transition occurred this frame.
                 */
                bool control_pressed_edge(const ControlPath& path, DeviceId device) const noexcept
                {
                    if (is_axis_control(path))
                        return false;
                    switch (path.family)
                    {
                        case DeviceFamily::Keyboard:
                            return edge_bit(key_pressed_edges_, path.control);
                        case DeviceFamily::Mouse:
                            return button_edge_bit(mouse_pressed_edges_, path.control);
                        case DeviceFamily::Gamepad:
                        case DeviceFamily::Virtual:
                        {
                            const GamepadState* state = gamepad_slot(device);
                            return state != nullptr && button_edge_bit(state->pressed_edges, path.control);
                        }
                        case DeviceFamily::Touch:
                            return false;
                    }
                    return false;
                }

                /** @brief Whether @p path's button saw a release this host frame (see @ref control_pressed_edge). */
                bool control_released_edge(const ControlPath& path, DeviceId device) const noexcept
                {
                    if (is_axis_control(path))
                        return false;
                    switch (path.family)
                    {
                        case DeviceFamily::Keyboard:
                            return edge_bit(key_released_edges_, path.control);
                        case DeviceFamily::Mouse:
                            return button_edge_bit(mouse_released_edges_, path.control);
                        case DeviceFamily::Gamepad:
                        case DeviceFamily::Virtual:
                        {
                            const GamepadState* state = gamepad_slot(device);
                            return state != nullptr && button_edge_bit(state->released_edges, path.control);
                        }
                        case DeviceFamily::Touch:
                            return false;
                    }
                    return false;
                }

                /**
                 * @brief The default device that answers for a control family.
                 *
                 * Keyboard and mouse are their fixed slots; a gamepad control resolves against
                 * the lowest-numbered connected gamepad (Phase 1 has no per-player routing —
                 * that arrives in Phase 7 as an explicit device id passed to @ref control_value).
                 *
                 * @param family The family to resolve.
                 * @return The device slot, or @ref INVALID_DEVICE if none is connected.
                 */
                DeviceId default_device(DeviceFamily family) const noexcept
                {
                    switch (family)
                    {
                        case DeviceFamily::Keyboard:
                            return KEYBOARD_DEVICE;
                        case DeviceFamily::Mouse:
                            return MOUSE_DEVICE;
                        case DeviceFamily::Gamepad:
                        case DeviceFamily::Virtual:
                            for (DeviceId slot = 0; slot < MAX_GAMEPADS; ++slot)
                                if (gamepads_[slot].connected)
                                    return static_cast<DeviceId>(FIRST_GAMEPAD_DEVICE + slot);
                            return INVALID_DEVICE;
                        case DeviceFamily::Touch:
                            return MOUSE_DEVICE;
                    }
                    return INVALID_DEVICE;
                }

                /** @brief Whether @p key is currently held. */
                bool key_down(Key key) const noexcept { return key_down(static_cast<std::uint16_t>(key)); }

                /** @brief Whether mouse @p button is currently held. */
                bool mouse_button_down(MouseButton button) const noexcept
                {
                    return mouse_button_down(static_cast<std::uint16_t>(button));
                }

                /** @brief The relative value of a mouse @p axis this frame (delta or wheel). */
                float mouse_axis(MouseAxis axis) const noexcept
                {
                    return mouse_axis_value(static_cast<std::uint16_t>(axis));
                }

                /** @brief The mouse cursor position in window pixels. */
                Vector2 mouse_position() const noexcept { return mouse_position_; }

                /** @brief Whether gamepad @p button on @p device is held. */
                bool gamepad_button(DeviceId device, GamepadButton button) const noexcept
                {
                    return gamepad_button(device, static_cast<std::uint16_t>(button));
                }

                /** @brief The value of gamepad @p axis on @p device. */
                float gamepad_axis(DeviceId device, GamepadAxis axis) const noexcept
                {
                    return gamepad_axis(device, static_cast<std::uint16_t>(axis));
                }

                /** @brief Whether the device in @p device slot is connected. */
                bool connected(DeviceId device) const noexcept
                {
                    if (device == KEYBOARD_DEVICE || device == MOUSE_DEVICE)
                        return true;
                    const int slot = device - FIRST_GAMEPAD_DEVICE;
                    if (slot < 0 || slot >= MAX_GAMEPADS)
                        return false;
                    return gamepads_[slot].connected;
                }

                /** @brief A tracked touch pointer's state (for §2.4 / UI pointer feed). */
                struct Pointer
                {
                    Vector2 position; /**< Position in window pixels. */
                    bool active = false; /**< Whether the pointer is currently down. */
                };

                /** @brief The pointer in slot @p index (0..@ref MAX_TOUCH_POINTS-1). */
                const Pointer& pointer(int index) const noexcept { return pointers_[static_cast<std::size_t>(index)]; }

                /**
                 * @brief Makes the mouse masquerade as pointer 0, so touch UIs work on desktop.
                 *
                 * When enabled, mouse motion updates pointer 0's position and the left button drives
                 * its active state, so a virtual control or the UI pointer reads one unified pointer
                 * stream whether the input came from a finger or a mouse.
                 *
                 * @param enabled Whether to fold the mouse into pointer 0.
                 */
                void set_mouse_as_pointer(bool enabled) noexcept { mouse_as_pointer_ = enabled; }

                /** @brief Whether the mouse is folded into pointer 0. */
                bool mouse_as_pointer() const noexcept { return mouse_as_pointer_; }

                /** @brief Pointer 0 — the primary pointer that feeds the engine UI and hit-testing. */
                const Pointer& primary_pointer() const noexcept { return pointers_[0]; }

            private:
                struct GamepadState
                {
                    bool connected = false;
                    std::uint32_t buttons = 0;        /**< Bit per @ref GamepadButton ordinal. */
                    std::uint32_t pressed_edges = 0;  /**< Down transitions this frame. */
                    std::uint32_t released_edges = 0; /**< Up transitions this frame. */
                    std::array<float, GAMEPAD_AXIS_COUNT> axes{};
                };

                static bool edge_bit(const std::array<std::uint64_t, 4>& edges, std::uint16_t ordinal) noexcept
                {
                    if (ordinal >= 256)
                        return false;
                    return (edges[ordinal >> 6] & (std::uint64_t(1) << (ordinal & 63))) != 0;
                }

                static bool button_edge_bit(std::uint32_t edges, std::uint16_t ordinal) noexcept
                {
                    if (ordinal >= 32)
                        return false;
                    return (edges & (std::uint32_t(1) << ordinal)) != 0;
                }

                void set_key(std::uint16_t ordinal, bool down) noexcept
                {
                    if (ordinal >= 256)
                        return;
                    const std::uint64_t bit = std::uint64_t(1) << (ordinal & 63);
                    if (down)
                    {
                        keys_[ordinal >> 6] |= bit;
                        key_pressed_edges_[ordinal >> 6] |= bit;
                    }
                    else
                    {
                        keys_[ordinal >> 6] &= ~bit;
                        key_released_edges_[ordinal >> 6] |= bit;
                    }
                }

                bool key_down(std::uint16_t ordinal) const noexcept
                {
                    if (ordinal >= 256)
                        return false;
                    return (keys_[ordinal >> 6] & (std::uint64_t(1) << (ordinal & 63))) != 0;
                }

                void set_mouse_button(std::uint16_t ordinal, bool down) noexcept
                {
                    if (ordinal >= 32)
                        return;
                    const std::uint32_t bit = std::uint32_t(1) << ordinal;
                    if (down)
                    {
                        mouse_buttons_ |= bit;
                        mouse_pressed_edges_ |= bit;
                    }
                    else
                    {
                        mouse_buttons_ &= ~bit;
                        mouse_released_edges_ |= bit;
                    }
                }

                bool mouse_button_down(std::uint16_t ordinal) const noexcept
                {
                    if (ordinal >= 32)
                        return false;
                    return (mouse_buttons_ & (std::uint32_t(1) << ordinal)) != 0;
                }

                void fold_mouse_move(float x, float y) noexcept
                {
                    if (have_mouse_position_)
                    {
                        mouse_delta_.x += x - mouse_position_.x;
                        mouse_delta_.y += y - mouse_position_.y;
                    }
                    mouse_position_ = Vector2{x, y};
                    have_mouse_position_ = true;
                }

                float mouse_axis_value(std::uint16_t ordinal) const noexcept
                {
                    switch (static_cast<MouseAxis>(ordinal))
                    {
                        case MouseAxis::DeltaX: return mouse_delta_.x;
                        case MouseAxis::DeltaY: return mouse_delta_.y;
                        case MouseAxis::Wheel:  return wheel_.y;
                        case MouseAxis::WheelX: return wheel_.x;
                    }
                    return 0.0f;
                }

                GamepadState* gamepad_slot(DeviceId device) noexcept
                {
                    const int slot = device - FIRST_GAMEPAD_DEVICE;
                    if (slot < 0 || slot >= MAX_GAMEPADS)
                        return nullptr;
                    return &gamepads_[slot];
                }

                const GamepadState* gamepad_slot(DeviceId device) const noexcept
                {
                    const int slot = device - FIRST_GAMEPAD_DEVICE;
                    if (slot < 0 || slot >= MAX_GAMEPADS)
                        return nullptr;
                    return &gamepads_[slot];
                }

                void set_gamepad_button(DeviceId device, std::uint16_t ordinal, bool down) noexcept
                {
                    GamepadState* state = gamepad_slot(device);
                    if (state == nullptr || ordinal >= 32)
                        return;
                    const std::uint32_t bit = std::uint32_t(1) << ordinal;
                    if (down)
                    {
                        state->buttons |= bit;
                        state->pressed_edges |= bit;
                    }
                    else
                    {
                        state->buttons &= ~bit;
                        state->released_edges |= bit;
                    }
                }

                bool gamepad_button(DeviceId device, std::uint16_t ordinal) const noexcept
                {
                    const GamepadState* state = gamepad_slot(device);
                    if (state == nullptr || ordinal >= 32)
                        return false;
                    return (state->buttons & (std::uint32_t(1) << ordinal)) != 0;
                }

                void set_gamepad_axis(DeviceId device, std::uint16_t ordinal, float value) noexcept
                {
                    GamepadState* state = gamepad_slot(device);
                    if (state == nullptr || ordinal >= GAMEPAD_AXIS_COUNT)
                        return;
                    state->axes[ordinal] = value;
                }

                float gamepad_axis(DeviceId device, std::uint16_t ordinal) const noexcept
                {
                    const GamepadState* state = gamepad_slot(device);
                    if (state == nullptr || ordinal >= GAMEPAD_AXIS_COUNT)
                        return 0.0f;
                    return state->axes[ordinal];
                }

                void set_connected(DeviceId device, bool connected) noexcept
                {
                    GamepadState* state = gamepad_slot(device);
                    if (state == nullptr)
                        return;
                    state->connected = connected;
                    if (!connected)
                    {
                        state->buttons = 0;
                        state->axes = std::array<float, GAMEPAD_AXIS_COUNT>{};
                    }
                }

                void set_pointer(std::uint16_t id, float x, float y, bool active) noexcept
                {
                    if (id >= MAX_TOUCH_POINTS)
                        return;
                    pointers_[id].position = Vector2{x, y};
                    pointers_[id].active = active;
                }

                void fold_mouse_as_pointer_button(std::uint16_t control, bool down, float x, float y) noexcept
                {
                    if (!mouse_as_pointer_ || control != static_cast<std::uint16_t>(MouseButton::Left))
                        return;
                    pointers_[0].position = Vector2{x, y};
                    pointers_[0].active = down;
                }

                std::array<std::uint64_t, 4> keys_{};
                std::array<std::uint64_t, 4> key_pressed_edges_{};
                std::array<std::uint64_t, 4> key_released_edges_{};
                std::uint32_t mouse_buttons_ = 0;
                std::uint32_t mouse_pressed_edges_ = 0;
                std::uint32_t mouse_released_edges_ = 0;
                Vector2 mouse_position_;
                Vector2 mouse_delta_;
                Vector2 wheel_;
                bool have_mouse_position_ = false;

                std::array<GamepadState, MAX_GAMEPADS> gamepads_{};
                std::array<Pointer, MAX_TOUCH_POINTS> pointers_{};
                bool mouse_as_pointer_ = false;
        };

        /**
         * @brief Which physical device answers each control family for one consumer.
         *
         * Binding resolution never hard-codes a device; it asks an assignment. The default
         * assigns the keyboard and mouse singletons and leaves the gamepad unset, which resolves
         * to the first connected pad — the single-player behaviour. A local-multiplayer game gives
         * each player its own assignment (player 1 = pad on slot 3, no keyboard), so the same
         * bindings resolve against that player's devices and no others (§2.6). Costs nothing to
         * carry in single-player, and is the whole routing mechanism in multiplayer.
         */
        struct DeviceAssignment
        {
            DeviceId keyboard = KEYBOARD_DEVICE;
            DeviceId mouse = MOUSE_DEVICE;
            DeviceId gamepad = INVALID_DEVICE; /**< INVALID resolves to the first connected pad. */

            /**
             * @brief The device that answers @p family for this assignment.
             * @param family   The control family being resolved.
             * @param registry The device state (for the first-connected fallback).
             * @return The device slot to read the control from.
             */
            DeviceId device_for(DeviceFamily family, const DeviceRegistry& registry) const noexcept
            {
                switch (family)
                {
                    case DeviceFamily::Keyboard:
                        return keyboard;
                    case DeviceFamily::Mouse:
                    case DeviceFamily::Touch:
                        return mouse;
                    case DeviceFamily::Gamepad:
                    case DeviceFamily::Virtual:
                        return gamepad != INVALID_DEVICE ? gamepad
                                                         : registry.default_device(DeviceFamily::Gamepad);
                }
                return INVALID_DEVICE;
            }
        };
    } // namespace Input
} // namespace SushiEngine
