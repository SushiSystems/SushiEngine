/**************************************************************************/
/* action_map.hpp                                                        */
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
 * @file action_map.hpp
 * @brief The action layer — named actions, contexts, and the mapper that resolves them.
 *
 * A consumer binds to a named action (`"Move"`, `"Jump"`) whose complete vocabulary is
 * three shapes: a `Button` (edges + level), an `Axis1D` (`float`), and an `Axis2D`
 * (@ref Vector2). @ref InputContext groups actions and offers a fluent builder that
 * reads like the binding data it produces. @ref ActionMapper holds a priority-ordered
 * context stack and resolves every action once per host frame into an @ref ActionSnapshot,
 * masking lower contexts by consuming the controls higher ones reference — so pushing a
 * menu context silently suppresses gameplay movement without any consumer testing a flag.
 *
 * All of this is header-only, SDL-free, SYCL-free host code; it reads only a
 * @ref DeviceRegistry, which is why the whole layer is driven by @ref ScriptedInputSource
 * in tests with no window and no hardware.
 */

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <SushiEngine/input/bindings.hpp>
#include <SushiEngine/input/controls.hpp>
#include <SushiEngine/input/device_registry.hpp>
#include <SushiEngine/input/events.hpp>

namespace SushiEngine
{
    namespace Input
    {
        /** @brief The three shapes an action can take — the whole consumer vocabulary. */
        enum class ActionType : std::uint8_t
        {
            Button,
            Axis1D,
            Axis2D,
        };

        /**
         * @brief The per-frame gate mirroring an immediate-mode UI's capture flags.
         *
         * The editor feeds ImGui's `WantCaptureKeyboard`/`WantCaptureMouse`/`WantTextInput`
         * here so that key- or mouse-sourced actions are suppressed exactly when the UI owns
         * that device — the gating `main.cpp` does by hand today, centralized in one place.
         * Gamepad and virtual controls are never gated (the UI does not own a pad).
         */
        struct InputGate
        {
            bool want_capture_keyboard = false;
            bool want_capture_mouse = false;
            bool want_text_input = false;
        };

        /**
         * @brief One action and every binding that can drive it.
         *
         * The binding vectors are populated by @ref InputContext's builders and read by
         * @ref ActionMapper; only the vectors relevant to @ref type are ever filled. A plain
         * data aggregate — resolution lives in the mapper, keeping the action a description.
         */
        struct Action
        {
            std::string name;
            ActionType type = ActionType::Button;
            bool relative = false; /**< An axis whose value is a per-frame delta (mouse), summed across ticks. */

            std::vector<Binding> button_bindings;            /**< @c Button sources. */
            std::vector<Binding> axis1d_bindings;            /**< @c Axis1D single-control sources. */
            std::vector<CompositeAxis1D> axis1d_composites;  /**< @c Axis1D two-button sources. */
            std::vector<Vector2Binding> axis2d_bindings;     /**< @c Axis2D analog-pair sources. */
            std::vector<CompositeAxis2D> axis2d_composites;  /**< @c Axis2D four-button sources. */
        };

        /** @brief The resolved value of one action for one frame. */
        struct ActionValue
        {
            ActionType type = ActionType::Button;
            bool held = false;     /**< Level: the action is active this frame. */
            bool pressed = false;  /**< Edge: became active this frame. */
            bool released = false; /**< Edge: became inactive this frame. */
            bool relative = false; /**< The axis is a per-frame delta; the tick sampler sums it. */
            float axis1d = 0.0f;   /**< `Axis1D` value. */
            Vector2 axis2d;        /**< `Axis2D` value. */
        };

        /**
         * @brief The whole frame's resolved actions, queried by name.
         *
         * The consumer-facing surface: `held`/`pressed`/`released` for buttons, `axis1d`
         * and `axis2d` for axes. A name that resolved to nothing this frame reads as an
         * inactive/zero value, so consumers never branch on presence.
         */
        class ActionSnapshot
        {
            public:
                /** @brief Whether @p action is held this frame. */
                bool held(const std::string& action) const noexcept
                {
                    const ActionValue* value = find(action);
                    return value != nullptr && value->held;
                }

                /** @brief Whether @p action became held this frame (rising edge). */
                bool pressed(const std::string& action) const noexcept
                {
                    const ActionValue* value = find(action);
                    return value != nullptr && value->pressed;
                }

                /** @brief Whether @p action became unheld this frame (falling edge). */
                bool released(const std::string& action) const noexcept
                {
                    const ActionValue* value = find(action);
                    return value != nullptr && value->released;
                }

                /** @brief The `Axis1D` value of @p action, or 0 if absent. */
                float axis1d(const std::string& action) const noexcept
                {
                    const ActionValue* value = find(action);
                    return value != nullptr ? value->axis1d : 0.0f;
                }

                /** @brief The `Axis2D` value of @p action, or (0,0) if absent. */
                Vector2 axis2d(const std::string& action) const noexcept
                {
                    const ActionValue* value = find(action);
                    return value != nullptr ? value->axis2d : Vector2{};
                }

                /** @brief The full resolved value for @p action, or nullptr if absent. */
                const ActionValue* find(const std::string& action) const noexcept
                {
                    const auto iterator = values_.find(action);
                    return iterator != values_.end() ? &iterator->second : nullptr;
                }

                /** @brief Records @p value for @p action (called by the mapper). */
                void set(const std::string& action, const ActionValue& value)
                {
                    values_[action] = value;
                }

                /** @brief Whether @p action already has a resolved value this frame. */
                bool has(const std::string& action) const noexcept
                {
                    return values_.find(action) != values_.end();
                }

                /** @brief Every resolved (name, value) pair this frame, for the tick sampler to fold. */
                const std::unordered_map<std::string, ActionValue>& values() const noexcept
                {
                    return values_;
                }

                /** @brief Drops all resolved values (called at the start of each frame). */
                void clear() noexcept { values_.clear(); }

            private:
                std::unordered_map<std::string, ActionValue> values_;
        };

        /**
         * @brief A fluent builder for a `Button` action; each `bind` returns *this.
         *
         * Holds a pointer to the @ref Action owned by the @ref InputContext (kept stable by
         * `unique_ptr` storage), so a chain of binds mutates one action in place.
         */
        class ButtonBuilder
        {
            public:
                explicit ButtonBuilder(Action* action) noexcept : action_(action) {}

                /** @brief Binds @p key to this action. */
                ButtonBuilder& bind(Key key)
                {
                    action_->button_bindings.push_back(Binding{control_of(key), {}, {}, 1.0f, false});
                    return *this;
                }

                /** @brief Binds @p button (mouse) to this action. */
                ButtonBuilder& bind(MouseButton button)
                {
                    action_->button_bindings.push_back(Binding{control_of(button), {}, {}, 1.0f, false});
                    return *this;
                }

                /** @brief Binds @p button (gamepad) to this action. */
                ButtonBuilder& bind(GamepadButton button)
                {
                    action_->button_bindings.push_back(Binding{control_of(button), {}, {}, 1.0f, false});
                    return *this;
                }

                /** @brief Binds @p key held together with @p modifier (a chord, e.g. Ctrl+Z). */
                ButtonBuilder& bind(Key key, Key modifier)
                {
                    Binding binding{control_of(key), {}, {}, 1.0f, false};
                    binding.chord.require(control_of(modifier));
                    action_->button_bindings.push_back(binding);
                    return *this;
                }

            private:
                Action* action_;
        };

        /** @brief A fluent builder for an `Axis1D` action. */
        class Axis1DBuilder
        {
            public:
                explicit Axis1DBuilder(Action* action) noexcept : action_(action) {}

                /** @brief Binds a gamepad @p axis (trigger or stick component) with a dead-band. */
                Axis1DBuilder& bind(GamepadAxis axis, Deadzone deadzone = {}, bool invert = false)
                {
                    action_->axis1d_bindings.push_back(
                        Binding{control_of(axis), {}, deadzone, 1.0f, invert});
                    return *this;
                }

                /** @brief Binds a relative mouse @p axis (wheel), scaled by @p scale. */
                Axis1DBuilder& bind(MouseAxis axis, float scale = 1.0f, bool invert = false)
                {
                    action_->relative = true; // a mouse axis is a per-frame delta, summed across ticks.
                    action_->axis1d_bindings.push_back(
                        Binding{control_of(axis), {}, {}, scale, invert});
                    return *this;
                }

                /** @brief Binds two keys as a synthesized axis: @p positive minus @p negative. */
                Axis1DBuilder& bind_composite(Key negative, Key positive, float scale = 1.0f)
                {
                    action_->axis1d_composites.push_back(
                        CompositeAxis1D{control_of(negative), control_of(positive), {}, scale});
                    return *this;
                }

            private:
                Action* action_;
        };

        /** @brief A fluent builder for an `Axis2D` action. */
        class Axis2DBuilder
        {
            public:
                explicit Axis2DBuilder(Action* action) noexcept : action_(action) {}

                /** @brief Binds four keys as a move vector (up, down, left, right), diagonally normalized. */
                Axis2DBuilder& bind_composite(Key up, Key down, Key left, Key right, float scale = 1.0f)
                {
                    CompositeAxis2D composite;
                    composite.up = control_of(up);
                    composite.down = control_of(down);
                    composite.left = control_of(left);
                    composite.right = control_of(right);
                    composite.scale = scale;
                    action_->axis2d_composites.push_back(composite);
                    return *this;
                }

                /**
                 * @brief Binds a gamepad @p stick (@c LeftStick or @c RightStick) with a radial dead-band.
                 * @param stick    The 2D stick alias to bind; its X/Y scalar axes are expanded here.
                 * @param deadzone The radial dead-band applied to the stick's magnitude.
                 * @param invert_y Whether to negate Y (screen-space vs. gameplay-space up).
                 */
                Axis2DBuilder& bind(GamepadAxis stick, Deadzone deadzone = {}, bool invert_y = false)
                {
                    Vector2Binding binding;
                    if (stick == GamepadAxis::RightStick)
                    {
                        binding.x_axis = control_of(GamepadAxis::RightStickX);
                        binding.y_axis = control_of(GamepadAxis::RightStickY);
                    }
                    else
                    {
                        binding.x_axis = control_of(GamepadAxis::LeftStickX);
                        binding.y_axis = control_of(GamepadAxis::LeftStickY);
                    }
                    binding.deadzone = deadzone;
                    binding.invert_y = invert_y;
                    action_->axis2d_bindings.push_back(binding);
                    return *this;
                }

                /** @brief Binds relative mouse motion (DeltaX, DeltaY) as a 2D axis for look controls. */
                Axis2DBuilder& bind_mouse(float scale = 1.0f, bool invert_y = false)
                {
                    action_->relative = true; // mouse motion is a per-frame delta, summed across ticks.
                    Vector2Binding binding;
                    binding.x_axis = control_of(MouseAxis::DeltaX);
                    binding.y_axis = control_of(MouseAxis::DeltaY);
                    binding.scale = scale;
                    binding.invert_y = invert_y;
                    action_->axis2d_bindings.push_back(binding);
                    return *this;
                }

            private:
                Action* action_;
        };

        /**
         * @brief A named set of actions with bindings — one gameplay/editor mode's inputs.
         *
         * Actions are added through the typed `add_*` builders; the context owns them with
         * stable addresses (`unique_ptr`) so a builder chain stays valid. A context carries no
         * runtime state — it is a description the @ref ActionMapper resolves against a device
         * state each frame.
         */
        class InputContext
        {
            public:
                /** @brief Creates an empty context named @p name (for logging and rebinding UIs). */
                explicit InputContext(std::string name) : name_(std::move(name)) {}

                /** @brief Adds a `Button` action named @p name and returns its builder. */
                ButtonBuilder add_button(std::string name)
                {
                    return ButtonBuilder{&add_action(std::move(name), ActionType::Button)};
                }

                /** @brief Adds an `Axis1D` action named @p name and returns its builder. */
                Axis1DBuilder add_axis1d(std::string name)
                {
                    return Axis1DBuilder{&add_action(std::move(name), ActionType::Axis1D)};
                }

                /** @brief Adds an `Axis2D` action named @p name and returns its builder. */
                Axis2DBuilder add_axis2d(std::string name)
                {
                    return Axis2DBuilder{&add_action(std::move(name), ActionType::Axis2D)};
                }

                /** @brief The context's name. */
                const std::string& name() const noexcept { return name_; }

                /** @brief The actions in the context, in definition order. */
                const std::vector<std::unique_ptr<Action>>& actions() const noexcept { return actions_; }

                /**
                 * @brief The action named @p name, for mutation (rebinding, deserialization).
                 * @param name The action to find.
                 * @return A pointer to the action, or nullptr if the context has no such action.
                 */
                Action* find_action(const std::string& name) noexcept
                {
                    for (const std::unique_ptr<Action>& action : actions_)
                        if (action->name == name)
                            return action.get();
                    return nullptr;
                }

            private:
                Action& add_action(std::string name, ActionType type)
                {
                    actions_.push_back(std::make_unique<Action>());
                    Action& action = *actions_.back();
                    action.name = std::move(name);
                    action.type = type;
                    return action;
                }

                std::string name_;
                std::vector<std::unique_ptr<Action>> actions_;
        };

        /**
         * @brief Resolves a priority-ordered context stack into a per-frame @ref ActionSnapshot.
         *
         * The stack is last-pushed-highest-priority. Each frame, @ref update walks it from the
         * top: a higher context resolves its actions first, then *consumes* every control those
         * actions reference, so a lower context binding the same control sees it as unavailable.
         * That single mechanism is the entire mode-switching API — push a `"Menu"` context and
         * gameplay movement is masked with no consumer aware of it. Edges (`pressed`/`released`)
         * are computed by comparing each button action's level against the previous frame's.
         */
        class ActionMapper
        {
            public:
                /** @brief Pushes @p context onto the stack as the new highest priority. */
                void push_context(InputContext& context) { stack_.push_back(&context); }

                /** @brief Pops the highest-priority context, if any. */
                void pop_context()
                {
                    if (!stack_.empty())
                        stack_.pop_back();
                }

                /** @brief Removes @p context from the stack wherever it sits. */
                void remove_context(const InputContext& context)
                {
                    for (std::size_t index = stack_.size(); index-- > 0;)
                        if (stack_[index] == &context)
                            stack_.erase(stack_.begin() + static_cast<std::ptrdiff_t>(index));
                }

                /** @brief Empties the context stack. */
                void clear_contexts() noexcept { stack_.clear(); }

                /** @brief The number of contexts currently on the stack. */
                std::size_t context_count() const noexcept { return stack_.size(); }

                /**
                 * @brief Routes this mapper's bindings to a specific set of devices.
                 *
                 * The single-player default reads the keyboard, the mouse, and the first connected
                 * gamepad. A local-multiplayer game gives each player's mapper its own assignment so
                 * the same bindings resolve against that player's devices only (§2.6).
                 *
                 * @param assignment The device each control family resolves against.
                 */
                void set_device_assignment(const DeviceAssignment& assignment) noexcept
                {
                    assignment_ = assignment;
                }

                /** @brief The device assignment this mapper resolves against. */
                const DeviceAssignment& device_assignment() const noexcept { return assignment_; }

                /**
                 * @brief Resolves every action across the stack against @p registry under @p gate.
                 * @param registry The device state to read.
                 * @param gate     The immediate-mode UI capture gate suppressing keyboard/mouse.
                 */
                void update(const DeviceRegistry& registry, const InputGate& gate)
                {
                    snapshot_.clear();
                    consumed_.clear();

                    for (std::size_t index = stack_.size(); index-- > 0;)
                    {
                        const InputContext& context = *stack_[index];
                        for (const std::unique_ptr<Action>& action : context.actions())
                        {
                            if (!snapshot_.has(action->name))
                                snapshot_.set(action->name, resolve(*action, registry, gate));
                        }
                        for (const std::unique_ptr<Action>& action : context.actions())
                            consume_controls(*action);
                    }

                    finalize_edges();
                }

                /** @brief The most recently resolved frame. */
                const ActionSnapshot& snapshot() const noexcept { return snapshot_; }

            private:
                bool family_suppressed(DeviceFamily family, const InputGate& gate) const noexcept
                {
                    if (family == DeviceFamily::Keyboard)
                        return gate.want_capture_keyboard || gate.want_text_input;
                    if (family == DeviceFamily::Mouse)
                        return gate.want_capture_mouse;
                    return false;
                }

                bool consumed(const ControlPath& control) const noexcept
                {
                    for (const ControlPath& taken : consumed_)
                        if (taken == control)
                            return true;
                    return false;
                }

                bool usable(const ControlPath& control, const InputGate& gate) const noexcept
                {
                    return !family_suppressed(control.family, gate) && !consumed(control);
                }

                ActionValue resolve(const Action& action, const DeviceRegistry& registry,
                                    const InputGate& gate) const
                {
                    ActionValue value;
                    value.type = action.type;
                    value.relative = action.relative;

                    switch (action.type)
                    {
                        case ActionType::Button:
                        {
                            for (const Binding& binding : action.button_bindings)
                            {
                                if (!usable(binding.control, gate))
                                    continue;
                                if (binding.evaluate_button(registry, assignment_) > 0.0f)
                                    value.held = true;
                                // The event-level edge component; finalize_edges ORs in the
                                // frame-to-frame level change so a chord that completes without a
                                // fresh control press (Z held, then Ctrl) still reports pressed.
                                if (binding.pressed_edge(registry, assignment_))
                                    value.pressed = true;
                                if (binding.released_edge(registry, assignment_))
                                    value.released = true;
                            }
                            break;
                        }
                        case ActionType::Axis1D:
                        {
                            float best = 0.0f;
                            for (const Binding& binding : action.axis1d_bindings)
                            {
                                if (!usable(binding.control, gate))
                                    continue;
                                const float candidate = binding.evaluate_axis1d(registry, assignment_);
                                if (std::fabs(candidate) > std::fabs(best))
                                    best = candidate;
                            }
                            for (const CompositeAxis1D& composite : action.axis1d_composites)
                            {
                                if (!usable(composite.negative, gate) || !usable(composite.positive, gate))
                                    continue;
                                const float candidate = composite.evaluate(registry, assignment_);
                                if (std::fabs(candidate) > std::fabs(best))
                                    best = candidate;
                            }
                            value.axis1d = best;
                            value.held = best != 0.0f;
                            break;
                        }
                        case ActionType::Axis2D:
                        {
                            Vector2 best;
                            float best_magnitude = 0.0f;
                            for (const Vector2Binding& binding : action.axis2d_bindings)
                            {
                                if (!usable(binding.x_axis, gate) || !usable(binding.y_axis, gate))
                                    continue;
                                const Vector2 candidate = binding.evaluate(registry, assignment_);
                                const float magnitude = std::sqrt(candidate.x * candidate.x +
                                                                  candidate.y * candidate.y);
                                if (magnitude > best_magnitude)
                                {
                                    best_magnitude = magnitude;
                                    best = candidate;
                                }
                            }
                            for (const CompositeAxis2D& composite : action.axis2d_composites)
                            {
                                if (!usable(composite.up, gate) || !usable(composite.down, gate) ||
                                    !usable(composite.left, gate) || !usable(composite.right, gate))
                                    continue;
                                const Vector2 candidate = composite.evaluate(registry, assignment_);
                                const float magnitude = std::sqrt(candidate.x * candidate.x +
                                                                  candidate.y * candidate.y);
                                if (magnitude > best_magnitude)
                                {
                                    best_magnitude = magnitude;
                                    best = candidate;
                                }
                            }
                            value.axis2d = best;
                            value.held = best_magnitude > 0.0f;
                            break;
                        }
                    }

                    return value;
                }

                void consume_controls(const Action& action)
                {
                    for (const Binding& binding : action.button_bindings)
                        consumed_.push_back(binding.control);
                    for (const Binding& binding : action.axis1d_bindings)
                        consumed_.push_back(binding.control);
                    for (const CompositeAxis1D& composite : action.axis1d_composites)
                    {
                        consumed_.push_back(composite.negative);
                        consumed_.push_back(composite.positive);
                    }
                    for (const Vector2Binding& binding : action.axis2d_bindings)
                    {
                        consumed_.push_back(binding.x_axis);
                        consumed_.push_back(binding.y_axis);
                    }
                    for (const CompositeAxis2D& composite : action.axis2d_composites)
                    {
                        consumed_.push_back(composite.up);
                        consumed_.push_back(composite.down);
                        consumed_.push_back(composite.left);
                        consumed_.push_back(composite.right);
                    }
                }

                void finalize_edges()
                {
                    for (const auto& stacked : stack_)
                    {
                        for (const std::unique_ptr<Action>& action : stacked->actions())
                        {
                            const ActionValue* value = snapshot_.find(action->name);
                            if (value == nullptr)
                                continue;

                            ActionValue updated = *value;
                            const bool was_held = previous_held_[action->name];
                            // OR the frame-to-frame level change onto the event-level edges that
                            // resolve() already recorded, so both a fresh press and a chord that
                            // just completed count, and a sub-frame tap survives.
                            updated.pressed = updated.pressed || (updated.held && !was_held);
                            updated.released = updated.released || (!updated.held && was_held);
                            snapshot_.set(action->name, updated);
                            next_held_[action->name] = updated.held;
                        }
                    }
                    previous_held_.swap(next_held_);
                    next_held_.clear();
                }

                std::vector<InputContext*> stack_;
                std::vector<ControlPath> consumed_;
                DeviceAssignment assignment_;
                ActionSnapshot snapshot_;
                std::unordered_map<std::string, bool> previous_held_;
                std::unordered_map<std::string, bool> next_held_;
        };
    } // namespace Input
} // namespace SushiEngine
