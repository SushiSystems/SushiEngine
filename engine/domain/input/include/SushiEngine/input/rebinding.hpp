/**************************************************************************/
/* rebinding.hpp                                                         */
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
 * @file rebinding.hpp
 * @brief Runtime rebinding: capture a control, detect conflicts, apply — all as data.
 *
 * Because a binding is data (@ref bindings.hpp), changing one changes no consumer code —
 * that is the point this file makes operational. @ref RebindingListener enters a capture
 * mode filtered by the expected shape: a button rebind ignores axis noise, and an axis
 * rebind requires deflection past a threshold so a drifting stick cannot bind itself.
 * @ref binding_conflict finds an action already using a captured control in the same
 * context, so a UI can warn before committing. The apply helpers write the captured
 * control back into an action's bindings. None of this touches SDL, SYCL, or the runtime;
 * it is driven by the same raw @ref InputEvent stream the mapper folds.
 */

#include <cstdint>
#include <string>
#include <vector>

#include <SushiEngine/input/action_map.hpp>
#include <SushiEngine/input/bindings.hpp>
#include <SushiEngine/input/controls.hpp>
#include <SushiEngine/input/events.hpp>

namespace SushiEngine
{
    namespace Input
    {
        /** @brief The control shape a rebind expects, so it ignores the other kinds. */
        enum class RebindShape : std::uint8_t
        {
            Button, /**< A key, mouse button, or gamepad button — captured on its press edge. */
            Axis,   /**< A gamepad axis — captured when it deflects past the threshold. */
        };

        /** @brief The lifecycle of a rebind capture. */
        enum class RebindStatus : std::uint8_t
        {
            Idle,      /**< Not capturing. */
            Listening, /**< Capturing; waiting for a matching control. */
            Captured,  /**< A control was captured (read it with @ref RebindingListener::captured). */
            Cancelled, /**< Cancelled by Escape or timeout. */
        };

        /**
         * @brief Captures the next control matching an expected shape, for click-to-rebind.
         *
         * The consumer calls @ref begin, then feeds each host frame's raw events (from
         * @ref InputManager::frame_events) to @ref update until the status leaves
         * @c Listening. The shape filter is what makes capture usable: an axis capture is
         * deaf to button chatter, and a button capture is deaf to a resting stick's jitter.
         * Escape always cancels; an optional timeout cancels a capture the player abandons.
         */
        class RebindingListener
        {
            public:
                /**
                 * @brief Enters capture mode for a control of @p shape.
                 * @param shape          The control shape to accept.
                 * @param axis_threshold Minimum |value| an axis must reach to be captured (drift guard).
                 * @param timeout_seconds Seconds before an unmatched capture self-cancels; 0 disables.
                 */
                void begin(RebindShape shape, float axis_threshold = 0.5f, float timeout_seconds = 0.0f) noexcept
                {
                    shape_ = shape;
                    axis_threshold_ = axis_threshold;
                    timeout_seconds_ = timeout_seconds;
                    elapsed_seconds_ = 0.0f;
                    captured_ = ControlPath{};
                    status_ = RebindStatus::Listening;
                }

                /** @brief Cancels an in-progress capture. */
                void cancel() noexcept
                {
                    if (status_ == RebindStatus::Listening)
                        status_ = RebindStatus::Cancelled;
                }

                /** @brief Whether a capture is currently waiting for a control. */
                bool listening() const noexcept { return status_ == RebindStatus::Listening; }

                /** @brief The capture status. */
                RebindStatus status() const noexcept { return status_; }

                /** @brief The captured control (valid once @ref status is @c Captured). */
                ControlPath captured() const noexcept { return captured_; }

                /**
                 * @brief Feeds one host frame's events into the capture.
                 *
                 * @param events The frame's raw events (@ref InputManager::frame_events).
                 * @param dt     Seconds since the last call, advancing the timeout; may be 0.
                 * @return The status after processing the frame.
                 */
                RebindStatus update(const std::vector<InputEvent>& events, float dt = 0.0f) noexcept
                {
                    if (status_ != RebindStatus::Listening)
                        return status_;

                    elapsed_seconds_ += dt;
                    if (timeout_seconds_ > 0.0f && elapsed_seconds_ >= timeout_seconds_)
                    {
                        status_ = RebindStatus::Cancelled;
                        return status_;
                    }

                    for (const InputEvent& event : events)
                    {
                        // Escape always aborts, whatever the shape.
                        if (event.type == EventType::KeyDown &&
                            event.control == static_cast<std::uint16_t>(Key::Escape))
                        {
                            status_ = RebindStatus::Cancelled;
                            return status_;
                        }

                        if (try_capture(event))
                        {
                            status_ = RebindStatus::Captured;
                            return status_;
                        }
                    }
                    return status_;
                }

            private:
                bool try_capture(const InputEvent& event) noexcept
                {
                    if (shape_ == RebindShape::Button)
                    {
                        switch (event.type)
                        {
                            case EventType::KeyDown:
                                captured_ = ControlPath{DeviceFamily::Keyboard, event.control};
                                return true;
                            case EventType::MouseButtonDown:
                                captured_ = ControlPath{DeviceFamily::Mouse, event.control};
                                return true;
                            case EventType::GamepadButtonDown:
                                captured_ = ControlPath{DeviceFamily::Gamepad, event.control};
                                return true;
                            default:
                                return false;
                        }
                    }

                    // Axis shape: require deflection past the threshold so drift cannot bind.
                    if (event.type == EventType::GamepadAxisMotion)
                    {
                        const float magnitude = event.value < 0.0f ? -event.value : event.value;
                        if (magnitude >= axis_threshold_)
                        {
                            captured_ = ControlPath{DeviceFamily::Gamepad,
                                                    static_cast<std::uint16_t>(AXIS_CONTROL_FLAG | event.control)};
                            return true;
                        }
                    }
                    return false;
                }

                RebindShape shape_ = RebindShape::Button;
                RebindStatus status_ = RebindStatus::Idle;
                ControlPath captured_;
                float axis_threshold_ = 0.5f;
                float timeout_seconds_ = 0.0f;
                float elapsed_seconds_ = 0.0f;
        };

        /**
         * @brief Whether @p action binds @p control anywhere (button, axis, or composite).
         * @param action  The action to inspect.
         * @param control The control to look for.
         * @return True if any of the action's bindings names @p control.
         */
        inline bool action_references(const Action& action, const ControlPath& control) noexcept
        {
            for (const Binding& binding : action.button_bindings)
                if (binding.control == control)
                    return true;
            for (const Binding& binding : action.axis1d_bindings)
                if (binding.control == control)
                    return true;
            for (const CompositeAxis1D& composite : action.axis1d_composites)
                if (composite.negative == control || composite.positive == control)
                    return true;
            for (const Vector2Binding& binding : action.axis2d_bindings)
                if (binding.x_axis == control || binding.y_axis == control)
                    return true;
            for (const CompositeAxis2D& composite : action.axis2d_composites)
                if (composite.up == control || composite.down == control ||
                    composite.left == control || composite.right == control)
                    return true;
            return false;
        }

        /**
         * @brief Finds an action in @p context (other than @p exclude_action) that binds @p control.
         *
         * The conflict a rebind UI surfaces before committing: two actions in one context sharing
         * a control would fight over it (the context's consumption resolves it arbitrarily).
         *
         * @param context        The context to search.
         * @param control        The control being rebound.
         * @param exclude_action The action being rebound (its own use is not a conflict).
         * @return The conflicting action, or nullptr if the control is free.
         */
        inline const Action* binding_conflict(const InputContext& context, const ControlPath& control,
                                              const std::string& exclude_action) noexcept
        {
            for (const std::unique_ptr<Action>& action : context.actions())
            {
                if (action->name == exclude_action)
                    continue;
                if (action_references(*action, control))
                    return action.get();
            }
            return nullptr;
        }

        /**
         * @brief Writes a captured button @p control into @p action's binding at @p index.
         *
         * Replaces the control of the existing button binding at @p index, preserving its chord
         * and processor parameters, or appends a new binding if @p index is out of range. Other
         * bindings on the action (a gamepad alternative, say) are untouched, so rebinding the
         * keyboard key does not drop the pad.
         *
         * @param action  The button action to rebind.
         * @param control The captured control to install.
         * @param index   Which button binding to replace (default the first).
         */
        inline void set_button_binding(Action& action, const ControlPath& control, std::size_t index = 0)
        {
            if (index < action.button_bindings.size())
                action.button_bindings[index].control = control;
            else
                action.button_bindings.push_back(Binding{control, {}, {}, 1.0f, false});
        }
    } // namespace Input
} // namespace SushiEngine
