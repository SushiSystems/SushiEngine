/**************************************************************************/
/* vehicle_drive.hpp                                                      */
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
 * @file vehicle_drive.hpp
 * @brief Turning held keys into a vehicle's controls, once per frame.
 *
 * P7 ended with "a drivable vehicle" proved by a test that set a throttle field. This is
 * the difference between that and *drivable*: a person holds a key and the car goes.
 *
 * ### Why the controls are ramped rather than switched
 *
 * A key is a bit and a throttle is not. Setting the pedal to 1 the instant Up goes down and
 * to 0 the instant it comes up is what makes keyboard driving in a physically simulated car
 * undriveable — every input is a step, every step is a shock through a drivetrain that
 * models its own inertia, and the tyres spend their life at the limit. So each control
 * moves toward its target at a rate, and the rates differ because the mechanisms do: a
 * throttle cable is quick, a steering rack is slower, and both return to centre faster than
 * they leave it, because that is what a spring does and what a driver expects.
 *
 * The ramp lives here rather than in the physics for the reason §4.5 gives about every
 * other seam: this is a property of the *input device*, not of the car. A wheel and pedal
 * set would deliver the same `VehicleInput` with no ramp at all, and the vehicle must not
 * be able to tell which it is talking to.
 */

#include <SushiEngine/input/action_map.hpp>
#include <SushiEngine/sim/simulation.hpp>

#include "../core/editor_context.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief Drives the selected entity's vehicle from this frame's held actions.
         *
         * Does nothing at all unless the selection carries a Vehicle, which is what makes
         * this safe to call every frame: the arrow keys are ordinary keys until an author
         * selects a car, and go back to being ordinary keys when they select something else.
         *
         * @param context      Editor state; supplies the selection and the held ramp.
         * @param world        The world; the controls are recorded on the vehicle.
         * @param actions      This frame's resolved input.
         * @param delta_seconds Wall-clock seconds since the last call, for the ramps.
         * @return Whether a vehicle was driven.
         */
        bool drive_selected_vehicle(EditorContext& context, Simulation::IWorldEditor& world,
                                    const Input::ActionSnapshot& actions, float delta_seconds);
    } // namespace Editor
} // namespace SushiEngine
