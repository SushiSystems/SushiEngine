/**************************************************************************/
/* virtual_controls.hpp                                                  */
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
 * @file virtual_controls.hpp
 * @brief On-screen touch controls that emit gamepad-shaped events — OCP made literal.
 *
 * A @ref VirtualControlSource owns a screen-space description of sticks and buttons and,
 * each host frame, claims the pointers that land in its regions and emits ordinary
 * gamepad-shaped @ref InputEvent records on a dedicated device slot: a virtual stick is
 * `GamepadAxis::LeftStickX/Y`, a virtual button is a `GamepadButton`. Nothing downstream
 * can tell it from hardware — the @ref DeviceRegistry folds its events like any other, and a
 * `"Move"` binding to `GamepadAxis::LeftStick` resolves the virtual stick through the exact
 * same path. Adding touch to a shipped gamepad game is therefore *placing controls*, not
 * writing input code: a new source emitting existing event shapes, open for extension and
 * closed for modification.
 *
 * It reads pointer state from a @ref DeviceRegistry (fed by touch, or by the mouse when
 * @ref DeviceRegistry::set_mouse_as_pointer is on), so it is exercised headlessly with
 * scripted pointer state. Rendering the controls is the engine UI's job; this source owns
 * only hit-testing and emission.
 */

#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include <SushiEngine/input/controls.hpp>
#include <SushiEngine/input/device_registry.hpp>
#include <SushiEngine/input/events.hpp>
#include <SushiEngine/input/source.hpp>

namespace SushiEngine
{
    namespace Input
    {
        /** @brief An on-screen analog stick: a circle whose deflection maps to a gamepad axis pair. */
        struct VirtualStick
        {
            Vector2 center;                              /**< Region centre, in window pixels. */
            float radius = 1.0f;                         /**< Pixels from centre to full deflection. */
            GamepadAxis x_axis = GamepadAxis::LeftStickX; /**< Axis the horizontal offset drives. */
            GamepadAxis y_axis = GamepadAxis::LeftStickY; /**< Axis the vertical offset drives (screen-down positive). */
        };

        /** @brief An on-screen button: a circular region that presses a gamepad button. */
        struct VirtualButton
        {
            Vector2 center;                             /**< Region centre, in window pixels. */
            float radius = 1.0f;                        /**< Hit radius, in pixels. */
            GamepadButton button = GamepadButton::South; /**< The gamepad button it presses. */
        };

        /**
         * @brief Synthesizes gamepad events from on-screen controls hit by touch pointers.
         *
         * Register it as a *virtual* source on the @ref InputManager (polled after the primary
         * event fold, so the pointer state it reads is up to date). Controls are laid out by the
         * game/UI in pixels; each poll claims one pointer per control (first match, so a pointer
         * cannot drive two controls at once) and emits the corresponding gamepad events on
         * @ref device. A stick emits its position every frame while engaged and re-centres to
         * zero on release; a button emits only its press and release edges.
         */
        class VirtualControlSource final : public IInputSource
        {
            public:
                /**
                 * @brief Creates a source reading @p registry and emitting on slot @p device.
                 * @param registry The device state whose pointers this source hit-tests.
                 * @param device   The gamepad slot the synthesized events carry (default the first).
                 */
                explicit VirtualControlSource(const DeviceRegistry& registry,
                                              DeviceId device = FIRST_GAMEPAD_DEVICE) noexcept
                    : registry_(&registry), device_(device)
                {
                }

                /** @brief Adds an on-screen @p stick; returns its index. */
                std::size_t add_stick(const VirtualStick& stick)
                {
                    sticks_.push_back(stick);
                    stick_engaged_.push_back(false);
                    return sticks_.size() - 1;
                }

                /** @brief Adds an on-screen @p button; returns its index. */
                std::size_t add_button(const VirtualButton& button)
                {
                    buttons_.push_back(button);
                    button_held_.push_back(false);
                    return buttons_.size() - 1;
                }

                /** @brief The device slot this source's synthesized events carry. */
                DeviceId device() const noexcept { return device_; }

                /**
                 * @brief Hit-tests the current pointers and appends the synthesized events.
                 * @param out Buffer the gamepad-shaped events are appended to.
                 */
                void poll(std::vector<InputEvent>& out) override
                {
                    if (!connected_)
                    {
                        InputEvent connected;
                        connected.device = device_;
                        connected.type = EventType::DeviceConnected;
                        out.push_back(connected);
                        connected_ = true;
                    }

                    std::array<bool, MAX_TOUCH_POINTS> claimed{};

                    for (std::size_t index = 0; index < sticks_.size(); ++index)
                    {
                        const VirtualStick& stick = sticks_[index];
                        const int pointer = claim_pointer(stick.center, stick.radius, claimed);
                        if (pointer >= 0)
                        {
                            const Vector2 position = registry_->pointer(pointer).position;
                            float x = (position.x - stick.center.x) / stick.radius;
                            float y = (position.y - stick.center.y) / stick.radius;
                            const float magnitude = std::sqrt(x * x + y * y);
                            if (magnitude > 1.0f)
                            {
                                x /= magnitude;
                                y /= magnitude;
                            }
                            emit_axis(out, stick.x_axis, x);
                            emit_axis(out, stick.y_axis, y);
                            stick_engaged_[index] = true;
                        }
                        else if (stick_engaged_[index])
                        {
                            emit_axis(out, stick.x_axis, 0.0f);
                            emit_axis(out, stick.y_axis, 0.0f);
                            stick_engaged_[index] = false;
                        }
                    }

                    for (std::size_t index = 0; index < buttons_.size(); ++index)
                    {
                        const VirtualButton& button = buttons_[index];
                        const int pointer = claim_pointer(button.center, button.radius, claimed);
                        const bool held = pointer >= 0;
                        if (held && !button_held_[index])
                            emit_button(out, button.button, true);
                        else if (!held && button_held_[index])
                            emit_button(out, button.button, false);
                        button_held_[index] = held;
                    }
                }

            private:
                int claim_pointer(const Vector2& center, float radius,
                                  std::array<bool, MAX_TOUCH_POINTS>& claimed) const noexcept
                {
                    const float radius_squared = radius * radius;
                    for (int index = 0; index < MAX_TOUCH_POINTS; ++index)
                    {
                        if (claimed[static_cast<std::size_t>(index)])
                            continue;
                        const DeviceRegistry::Pointer& pointer = registry_->pointer(index);
                        if (!pointer.active)
                            continue;
                        const float dx = pointer.position.x - center.x;
                        const float dy = pointer.position.y - center.y;
                        if (dx * dx + dy * dy <= radius_squared)
                        {
                            claimed[static_cast<std::size_t>(index)] = true;
                            return index;
                        }
                    }
                    return -1;
                }

                void emit_axis(std::vector<InputEvent>& out, GamepadAxis axis, float value) const
                {
                    InputEvent event;
                    event.device = device_;
                    event.type = EventType::GamepadAxisMotion;
                    event.control = static_cast<std::uint16_t>(axis);
                    event.value = value;
                    out.push_back(event);
                }

                void emit_button(std::vector<InputEvent>& out, GamepadButton button, bool down) const
                {
                    InputEvent event;
                    event.device = device_;
                    event.type = down ? EventType::GamepadButtonDown : EventType::GamepadButtonUp;
                    event.control = static_cast<std::uint16_t>(button);
                    event.value = down ? 1.0f : 0.0f;
                    out.push_back(event);
                }

                const DeviceRegistry* registry_;
                DeviceId device_;
                std::vector<VirtualStick> sticks_;
                std::vector<VirtualButton> buttons_;
                std::vector<bool> stick_engaged_;
                std::vector<bool> button_held_;
                bool connected_ = false;
        };
    } // namespace Input
} // namespace SushiEngine
