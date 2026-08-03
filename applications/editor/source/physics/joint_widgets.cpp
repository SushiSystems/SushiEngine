/**************************************************************************/
/* joint_widgets.cpp                                                      */
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

#include "joint_widgets.hpp"

#include <imgui.h>

#include "../ui/panel_widgets.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        namespace
        {
            /** @brief Degrees per radian, for the rows an author thinks in degrees. */
            constexpr float DEGREES_PER_RADIAN = 57.2957795f;

            /**
             * @brief Closes a field row: its undo bracket and its tooltip.
             *
             * The two things every row here shares, spelled once. `track_item_undo` has to
             * run immediately after the widget while it is still the *current item*, which
             * is why this is a call at the end of each row rather than a wrapper around it.
             */
            void finish_row(EditorContext& context, Simulation::IWorldEditor& world,
                            const char* tooltip)
            {
                track_item_undo(context, world);
                if (tooltip != nullptr && ImGui::IsItemHovered())
                    ImGui::SetTooltip("%s", tooltip);
            }

            /**
             * @brief A `Vector3` row, bridging the engine's scalar to the widget's float.
             *
             * A plain drag rather than @ref vector3_field, and the distinction is load-bearing
             * rather than stylistic: that function is a *table row emitter* — it opens with
             * `ImGui::TableNextRow` — so it may only be called inside the two-column table the
             * Inspector's Transform block sets up. Calling it outside one dereferences a null
             * current table, which is a crash and not a layout glitch. Component sections are
             * drawn outside a table (see `ComponentEditor::vector`, which is a plain drag for
             * the same reason), so these are too.
             */
            bool vector_row(EditorContext& context, Simulation::IWorldEditor& world,
                            const char* label, Vector3& value, const char* tooltip)
            {
                float components[3] = {to_float(value.x), to_float(value.y), to_float(value.z)};
                const bool changed = ImGui::DragFloat3(label, components, 0.01f, 0.0f, 0.0f,
                                                       "%.3f");
                finish_row(context, world, tooltip);
                if (!changed)
                    return false;
                value = Vector3{Scalar(components[0]), Scalar(components[1]),
                                Scalar(components[2])};
                return true;
            }

            /** @brief A `Scalar` row; see @ref vector_row for why it is not @ref scalar_field. */
            bool scalar_row(EditorContext& context, Simulation::IWorldEditor& world,
                            const char* label, Scalar& value, float speed, float low, float high,
                            const char* format, const char* tooltip)
            {
                float shown = to_float(value);
                const bool changed = ImGui::DragFloat(label, &shown, speed, low, high, format);
                finish_row(context, world, tooltip);
                if (!changed)
                    return false;
                value = Scalar(shown);
                return true;
            }
        } // namespace

        bool draw_joint_limit(EditorContext& context, Simulation::IWorldEditor& world,
                              const char* label, Simulation::JointLimitDescription& limit,
                              bool degrees, const char* tooltip, bool upper_only)
        {
            ImGui::PushID(label);
            bool changed = ImGui::Checkbox(label, &limit.enabled);
            track_item_undo(context, world);
            if (tooltip != nullptr && ImGui::IsItemHovered())
                ImGui::SetTooltip("%s", tooltip);

            // The stored unit is radians and metres; the shown one is degrees and metres, so
            // the angular rows convert on both sides of the widget rather than storing what
            // was typed.
            const float scale = degrees ? DEGREES_PER_RADIAN : 1.0f;
            const char* const format = degrees ? "%.1f deg" : "%.3f m";

            ImGui::BeginDisabled(!limit.enabled);
            if (!upper_only)
            {
                float lower = to_float(limit.lower) * scale;
                const bool moved =
                    ImGui::DragFloat("Lower", &lower, 0.5f, -1000.0f, 1000.0f, format);
                finish_row(context, world, nullptr);
                if (moved)
                {
                    limit.lower = Scalar(lower / scale);
                    changed = true;
                }
            }
            float upper = to_float(limit.upper) * scale;
            const bool upper_moved =
                ImGui::DragFloat(upper_only ? "Half Angle" : "Upper", &upper, 0.5f,
                                 upper_only ? 0.0f : -1000.0f, 1000.0f, format);
            finish_row(context, world, nullptr);
            if (upper_moved)
            {
                limit.upper = Scalar(upper / scale);
                changed = true;
            }
            changed |= scalar_row(context, world, "Give", limit.compliance, 1e-7f, 0.0f, 1.0f,
                                  "%.7f",
                                  "Compliance of the stop itself. Zero is a hard stop; a "
                                  "positive value is a bumper, which is what a bump stop and a "
                                  "door seal actually are.");
            ImGui::EndDisabled();
            ImGui::PopID();
            return changed;
        }

        bool draw_joint_params(EditorContext& context, Simulation::IWorldEditor& world,
                               Simulation::JointParameters& params)
        {
            using Simulation::JointMotorType;
            using Simulation::JointType;

            static const char* const TYPE_NAMES[] = {"Fixed",    "Ball",       "Hinge",
                                                     "Slider",   "Distance",   "Cone Twist",
                                                     "Six Degree of Freedom"};
            static const char* const MOTOR_NAMES[] = {"Disabled", "Position", "Velocity"};

            bool changed = false;
            int type_index = static_cast<int>(params.type);
            if (type_index < 0 || type_index >= int(Simulation::JOINT_TYPE_COUNT))
                type_index = 0;
            if (ImGui::BeginCombo("Type", TYPE_NAMES[type_index]))
            {
                for (int option = 0; option < int(Simulation::JOINT_TYPE_COUNT); ++option)
                {
                    if (!ImGui::Selectable(TYPE_NAMES[option], option == type_index))
                        continue;
                    params.type = static_cast<JointType>(option);
                    context.history.record(world);
                    changed = true;
                }
                ImGui::EndCombo();
            }

            // Which rows appear follows the kind. A ball joint has no surviving axis, so a
            // twist limit on it would be a control that changes nothing — and a control that
            // changes nothing is worse than a missing one, because it is tried first.
            const JointType type = params.type;
            const bool has_axis = type != JointType::Fixed && type != JointType::Ball;
            const bool has_twist = type == JointType::Hinge || type == JointType::ConeTwist ||
                                   type == JointType::SixDegreeOfFreedom;
            const bool has_linear = type == JointType::Slider || type == JointType::Distance ||
                                    type == JointType::SixDegreeOfFreedom;
            const bool has_swing =
                type == JointType::ConeTwist || type == JointType::SixDegreeOfFreedom;

            changed |= vector_row(context, world, "Anchor A", params.anchor_a,
                                  "Where the joint attaches on the first body, in its local "
                                  "space, in metres.");
            changed |= vector_row(context, world, "Anchor B", params.anchor_b,
                                  "Where it attaches on the second body, in that body's local "
                                  "space, in metres.");
            if (has_axis)
            {
                changed |= vector_row(context, world, "Axis A", params.axis_a,
                                      "The joint's primary axis on the first body: a hinge's "
                                      "rotation axis, a slider's travel axis, a cone twist's "
                                      "twist axis.");
                changed |= vector_row(context, world, "Axis B", params.axis_b,
                                      "The same axis on the second body. With the two bodies in "
                                      "their authored pose these read zero twist — 'the door is "
                                      "shut'.");
            }

            changed |= scalar_row(context, world, "Give", params.compliance, 1e-7f, 0.0f, 1.0f,
                                  "%.7f",
                                  "Compliance of the joint itself — the attachment and the "
                                  "rotations it removes. The limits and the drive carry their "
                                  "own, because 'the hinge has give' and 'the stop has give' "
                                  "are different statements.");

            if (has_linear)
                changed |= draw_joint_limit(
                    context, world, "Linear Limit", params.linear_limit, false,
                    type == JointType::Distance
                        ? "The range the anchors may be apart. A rope goes slack below the "
                          "lower bound and a strut resists both ways."
                        : "Travel along the primary axis.",
                    false);
            if (has_twist)
                changed |= draw_joint_limit(context, world, "Twist Limit", params.twist_limit, true,
                                            "Rotation about the primary axis: how far the door "
                                            "opens, and how far past shut it may swing.",
                                            false);
            if (has_swing)
                changed |= draw_joint_limit(
                    context, world, "Swing Limit", params.swing_limit, true,
                    "The cone the primary axis may stray inside. Only the half angle is read — "
                    "a swing is an unsigned angle off an axis, so a lower bound would say the "
                    "joint must stay bent.",
                    true);

            if (has_axis)
            {
                int motor_index = static_cast<int>(params.motor.type);
                if (motor_index < 0 || motor_index > 2)
                    motor_index = 0;
                if (ImGui::BeginCombo("Drive", MOTOR_NAMES[motor_index]))
                {
                    for (int option = 0; option < 3; ++option)
                    {
                        if (!ImGui::Selectable(MOTOR_NAMES[option], option == motor_index))
                            continue;
                        params.motor.type = static_cast<JointMotorType>(option);
                        context.history.record(world);
                        changed = true;
                    }
                    ImGui::EndCombo();
                }
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("A servo holds a coordinate; a velocity drive holds a "
                                      "rate. A velocity drive at a target of zero with a small "
                                      "force limit is joint friction.");
                changed |= scalar_row(context, world, "Drive Target", params.motor.target, 0.05f,
                                      -1000.0f, 1000.0f, "%.3f",
                                      "An angle or a distance for a servo; an angular or linear "
                                      "rate for a velocity drive.");
                changed |= scalar_row(context, world, "Drive Force Limit", params.motor.max_force,
                                      1.0f, 0.0f, 1000000.0f, "%.1f",
                                      "The most the drive may spend, in N or N.m. Zero is an "
                                      "ideal drive that holds its target whatever it costs.");
                changed |= scalar_row(context, world, "Drive Damping", params.motor.damping, 0.1f,
                                      0.0f, 1000.0f, "%.2f 1/s",
                                      "Viscous resistance on the driven coordinate. A position "
                                      "drive at a compliance is a spring, and a spring alone "
                                      "rings forever — this is the other half of a suspension "
                                      "strut. Works with the drive disabled too, which is what "
                                      "a steering damper is.");
            }

            changed |= scalar_row(context, world, "Break Force", params.break_force, 10.0f, 0.0f,
                                  1e9f, "%.0f N",
                                  "Force above which the joint tears out and is gone; zero is "
                                  "unbreakable. Measured against the worst single substep, not "
                                  "the mean — an impact's mean is nearly zero because the "
                                  "correction reverses the substep after the hit.");
            changed |= scalar_row(context, world, "Break Torque", params.break_torque, 10.0f, 0.0f,
                                  1e9f, "%.0f N.m",
                                  "Torque above which it shears off; zero is unbreakable.");
            return changed;
        }
    } // namespace Editor
} // namespace SushiEngine
