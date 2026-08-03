/**************************************************************************/
/* vehicle_drive.cpp                                                      */
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

#include "vehicle_drive.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        namespace
        {
            /** @brief Full throttle from rest, in seconds. A cable, so quick. */
            constexpr float THROTTLE_RISE = 0.25f;

            /** @brief Closed from full, in seconds. Faster than it opens: lifting off is instant. */
            constexpr float THROTTLE_FALL = 0.12f;

            /** @brief Lock to lock from centre, in seconds. A rack, so slower than a pedal. */
            constexpr float STEER_RISE = 0.5f;

            /** @brief Back to centre, in seconds. A spring, so faster than it turns. */
            constexpr float STEER_FALL = 0.25f;

            /** @brief The most the front wheels turn, in radians — about 32 degrees. */
            constexpr float STEER_LOCK = 0.56f;

            /** @brief What the brakes can make at each wheel, in N·m. */
            constexpr float BRAKE_TORQUE = 3200.0f;

            /**
             * @brief Moves @p value toward @p target at @p rate, without overshooting it.
             *
             * The overshoot guard is the whole of it: a per-frame step of `dt / seconds` is
             * larger than the remaining distance whenever the frame is long, and a ramp that
             * sails past its target oscillates around it — which on a throttle is a car that
             * surges at low frame rates and is smooth at high ones, the worst kind of bug to
             * reproduce.
             *
             * @param value  The control's current position, moved in place.
             * @param target Where the key says it should be.
             * @param rate   Units per second.
             * @param dt     Seconds elapsed.
             */
            void ramp(float& value, float target, float rate, float dt)
            {
                const float step = rate * dt;
                if (target > value)
                    value = value + step > target ? target : value + step;
                else
                    value = value - step < target ? target : value - step;
            }
        } // namespace

        bool drive_selected_vehicle(EditorContext& context, Simulation::IWorldEditor& world,
                                    const Input::ActionSnapshot& actions, float delta_seconds)
        {
            const Simulation::EntityId id = context.selected_entity;
            if (id == Simulation::NULL_ENTITY || !world.exists(id) || !world.has_vehicle(id))
            {
                // The ramps are reset rather than frozen, so selecting a car does not inherit
                // whatever the last one was doing — a throttle that carried over between two
                // vehicles would be a car that drives off the moment it is clicked.
                context.drive_throttle = 0.0f;
                context.drive_steer = 0.0f;
                return false;
            }

            const bool up = actions.held("DriveThrottle");
            const bool down = actions.held("DriveBrake");
            const bool left = actions.held("DriveLeft");
            const bool right = actions.held("DriveRight");

            ramp(context.drive_throttle, up ? 1.0f : 0.0f,
                 up ? 1.0f / THROTTLE_RISE : 1.0f / THROTTLE_FALL, delta_seconds);

            // Both keys or neither is a centred wheel, not a fight between two ramps: a
            // steering input is one axis and pressing both ends of it means nothing.
            const float steer_target = (left == right) ? 0.0f : (left ? -1.0f : 1.0f);
            ramp(context.drive_steer, steer_target,
                 steer_target == 0.0f ? 1.0f / STEER_FALL : 1.0f / STEER_RISE, delta_seconds);

            Simulation::VehicleInput input = world.vehicle_input(id);
            input.throttle = Scalar(context.drive_throttle);
            input.steer = Scalar(context.drive_steer * STEER_LOCK);
            input.brake = down ? Scalar(BRAKE_TORQUE) : Scalar(0);
            input.clutch = actions.held("DriveClutch") ? Scalar(0) : Scalar(1);

            // A shift is an edge, not a state — the one control here that is genuinely a
            // button. Out of range is refused by the powertrain rather than clamped here,
            // so the gearbox stays the only authority on how many gears it has.
            if (actions.pressed("DriveGearUp"))
                ++input.gear;
            else if (actions.pressed("DriveGearDown") && input.gear > 0)
                --input.gear;

            world.set_vehicle_input(id, input);
            return true;
        }
    } // namespace Editor
} // namespace SushiEngine
