/**************************************************************************/
/* bindings.hpp                                                          */
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
 * @file bindings.hpp
 * @brief Bindings are data — the control-to-action mapping and its processor chain.
 *
 * A binding names a control (or a small set of controls, for a composite) and the
 * pure transform from that control's live value to an action's value. Nothing here
 * holds state: every evaluator is a function of a @ref DeviceRegistry, so the same
 * binding evaluated against the same device state always yields the same result. The
 * processor order is fixed — deadzone → invert → scale, with composite assembly and
 * diagonal normalization layered on top — because a stable order is what lets a
 * keyboard composite and an analog stick feed one action through identical downstream
 * math (§2.2).
 *
 * A new binding shape (a chorded axis, a swipe) is a new struct and a new evaluator;
 * no consumer of an action changes. That is the open-for-extension seam SOLID's OCP
 * asks for, expressed as data rather than as branches in the mapper.
 */

#include <array>
#include <cmath>
#include <cstdint>

#include <SushiEngine/input/controls.hpp>
#include <SushiEngine/input/device_registry.hpp>
#include <SushiEngine/input/events.hpp>

namespace SushiEngine
{
    namespace Input
    {
        /**
         * @brief A dead-band that suppresses small control values and renormalizes the rest.
         *
         * @c Axial treats a scalar independently (a trigger, one stick axis): values with
         * magnitude below @ref inner become exactly 0, and the surviving range is remapped
         * so @ref inner maps to 0 and @ref outer maps to 1 (preserving sign). @c Radial
         * treats an (x, y) pair by its magnitude (a stick as a whole), which is what stops a
         * diagonal from crossing the dead-band sooner than a cardinal push. @c None passes
         * the value through unchanged.
         */
        struct Deadzone
        {
            enum class Shape : std::uint8_t
            {
                None,
                Axial,
                Radial,
            };

            Shape shape = Shape::None;
            float inner = 0.0f; /**< Magnitude at or below which the value is zeroed. */
            float outer = 1.0f; /**< Magnitude that remaps to 1 (values beyond clamp to 1). */

            /** @brief A per-scalar dead-band from @p inner to @p outer (triggers, single axes). */
            static Deadzone axial(float inner, float outer = 1.0f) noexcept
            {
                return Deadzone{Shape::Axial, inner, outer};
            }

            /** @brief A magnitude dead-band from @p inner to @p outer (whole sticks). */
            static Deadzone radial(float inner, float outer = 1.0f) noexcept
            {
                return Deadzone{Shape::Radial, inner, outer};
            }

            /** @brief Applies the axial mapping to scalar @p value (a no-op unless @c Axial). */
            float apply_scalar(float value) const noexcept
            {
                if (shape != Shape::Axial)
                    return value;
                return remap_magnitude(value);
            }

            /** @brief Applies the radial mapping to @p value (a no-op unless @c Radial). */
            Vector2 apply_vector(Vector2 value) const noexcept
            {
                if (shape != Shape::Radial)
                    return value;
                const float magnitude = std::sqrt(value.x * value.x + value.y * value.y);
                if (magnitude <= 0.0f)
                    return Vector2{};
                const float scaled = remap_magnitude(magnitude);
                const float factor = scaled / magnitude;
                return Vector2{value.x * factor, value.y * factor};
            }

            private:
                /** @brief Remaps a signed @p value's magnitude through [inner, outer] → [0, 1]. */
                float remap_magnitude(float value) const noexcept
                {
                    const float magnitude = std::fabs(value);
                    if (magnitude <= inner)
                        return 0.0f;
                    const float span = outer - inner;
                    const float unit = span > 0.0f ? (magnitude - inner) / span : 1.0f;
                    const float clamped = unit > 1.0f ? 1.0f : unit;
                    return value < 0.0f ? -clamped : clamped;
                }
        };

        /**
         * @brief An optional held-control requirement gating a binding (Ctrl+S, L2+face).
         *
         * Up to two modifiers, each a full @ref ControlPath so a chord can span families
         * (a keyboard Ctrl, a gamepad L2). An empty gate is always satisfied. Chords are why
         * `Ctrl+Z` and `Z` can bind different actions in the same context without either
         * consumer knowing a modifier exists.
         */
        struct ChordGate
        {
            std::array<ControlPath, 2> modifiers{};
            std::uint8_t count = 0;

            /** @brief Adds @p modifier to the gate (up to two); returns *this for chaining. */
            ChordGate& require(ControlPath modifier) noexcept
            {
                if (count < modifiers.size())
                    modifiers[count++] = modifier;
                return *this;
            }

            /** @brief Whether every required modifier is currently held on @p assignment's devices. */
            bool satisfied(const DeviceRegistry& registry, const DeviceAssignment& assignment) const noexcept
            {
                for (std::uint8_t index = 0; index < count; ++index)
                {
                    const ControlPath& modifier = modifiers[index];
                    if (!registry.control_down(modifier, assignment.device_for(modifier.family, registry)))
                        return false;
                }
                return true;
            }
        };

        /**
         * @brief A single control bound to an action, with its processor parameters.
         *
         * Serves a `Button` action (the control is a key/button) and a single-control
         * `Axis1D` action (a trigger, the mouse wheel). @ref scale multiplies after the
         * dead-band and inversion; @ref invert negates. A 2D action never uses this — it
         * uses @ref Vector2Binding or @ref CompositeAxis2D.
         */
        struct Binding
        {
            ControlPath control;
            ChordGate chord;
            Deadzone deadzone;
            float scale = 1.0f;
            bool invert = false;

            /** @brief The button level (0/1) this binding contributes, gated by its chord. */
            float evaluate_button(const DeviceRegistry& registry, const DeviceAssignment& assignment) const noexcept
            {
                if (!chord.satisfied(registry, assignment))
                    return 0.0f;
                return registry.control_down(control, assignment.device_for(control.family, registry)) ? 1.0f : 0.0f;
            }

            /**
             * @brief Whether this binding's control saw a press this frame with its chord held.
             *
             * Event-level, so it catches a press-and-release inside a single host frame that the
             * frame-to-frame level diff would miss — the sub-frame tap the tick sampler must keep.
             */
            bool pressed_edge(const DeviceRegistry& registry, const DeviceAssignment& assignment) const noexcept
            {
                if (!chord.satisfied(registry, assignment))
                    return false;
                return registry.control_pressed_edge(control, assignment.device_for(control.family, registry));
            }

            /** @brief Whether this binding's control saw a release this frame with its chord held. */
            bool released_edge(const DeviceRegistry& registry, const DeviceAssignment& assignment) const noexcept
            {
                if (!chord.satisfied(registry, assignment))
                    return false;
                return registry.control_released_edge(control, assignment.device_for(control.family, registry));
            }

            /** @brief The scalar value this binding contributes to an `Axis1D` action. */
            float evaluate_axis1d(const DeviceRegistry& registry, const DeviceAssignment& assignment) const noexcept
            {
                if (!chord.satisfied(registry, assignment))
                    return 0.0f;
                float value = registry.control_value(control, assignment.device_for(control.family, registry));
                value = deadzone.apply_scalar(value);
                if (invert)
                    value = -value;
                return value * scale;
            }
        };

        /**
         * @brief Two buttons synthesizing an `Axis1D` (Q/E for down/up, keys for a d-pad axis).
         *
         * Value is `(positive held ? 1 : 0) - (negative held ? 1 : 0)`, then scaled. This is
         * how a keyboard reaches the same `Axis1D` action an analog trigger or stick axis does.
         */
        struct CompositeAxis1D
        {
            ControlPath negative;
            ControlPath positive;
            ChordGate chord;
            float scale = 1.0f;

            /** @brief The scalar value in [-scale, scale] this composite contributes. */
            float evaluate(const DeviceRegistry& registry, const DeviceAssignment& assignment) const noexcept
            {
                if (!chord.satisfied(registry, assignment))
                    return 0.0f;
                const float positive_value =
                    registry.control_down(positive, assignment.device_for(positive.family, registry)) ? 1.0f : 0.0f;
                const float negative_value =
                    registry.control_down(negative, assignment.device_for(negative.family, registry)) ? 1.0f : 0.0f;
                return (positive_value - negative_value) * scale;
            }
        };

        /**
         * @brief Two analog axes forming an `Axis2D` (a gamepad stick, mouse deltas).
         *
         * Reads @ref x_axis and @ref y_axis as scalars, applies a radial dead-band to the
         * pair (so the stick's dead-band is circular), inverts either component, and scales.
         * A mouse-look binding sets @ref x_axis / @ref y_axis to the relative mouse deltas and
         * leaves the dead-band @c None.
         */
        struct Vector2Binding
        {
            ControlPath x_axis;
            ControlPath y_axis;
            ChordGate chord;
            Deadzone deadzone;
            bool invert_x = false;
            bool invert_y = false;
            float scale = 1.0f;

            /** @brief The 2D value this binding contributes to an `Axis2D` action. */
            Vector2 evaluate(const DeviceRegistry& registry, const DeviceAssignment& assignment) const noexcept
            {
                if (!chord.satisfied(registry, assignment))
                    return Vector2{};
                Vector2 value{
                    registry.control_value(x_axis, assignment.device_for(x_axis.family, registry)),
                    registry.control_value(y_axis, assignment.device_for(y_axis.family, registry))};
                value = deadzone.apply_vector(value);
                if (invert_x)
                    value.x = -value.x;
                if (invert_y)
                    value.y = -value.y;
                return value * scale;
            }
        };

        /**
         * @brief Four buttons synthesizing an `Axis2D` move vector (WASD), normalized diagonally.
         *
         * `x = right - left`, `y = up - down`; when @ref normalize is set and the result would
         * exceed unit length (a diagonal), it is renormalized so a diagonal is not faster than a
         * cardinal push. This is the keyboard's path into the very same move action a stick drives.
         */
        struct CompositeAxis2D
        {
            ControlPath up;
            ControlPath down;
            ControlPath left;
            ControlPath right;
            ChordGate chord;
            float scale = 1.0f;
            bool normalize = true;

            /** @brief The 2D value this composite contributes. */
            Vector2 evaluate(const DeviceRegistry& registry, const DeviceAssignment& assignment) const noexcept
            {
                if (!chord.satisfied(registry, assignment))
                    return Vector2{};
                const float up_value = held(registry, assignment, up);
                const float down_value = held(registry, assignment, down);
                const float left_value = held(registry, assignment, left);
                const float right_value = held(registry, assignment, right);
                Vector2 value{right_value - left_value, up_value - down_value};
                if (normalize)
                {
                    const float magnitude = std::sqrt(value.x * value.x + value.y * value.y);
                    if (magnitude > 1.0f)
                        value = value * (1.0f / magnitude);
                }
                return value * scale;
            }

            private:
                static float held(const DeviceRegistry& registry, const DeviceAssignment& assignment,
                                  const ControlPath& control) noexcept
                {
                    return registry.control_down(control, assignment.device_for(control.family, registry)) ? 1.0f : 0.0f;
                }
        };
    } // namespace Input
} // namespace SushiEngine
