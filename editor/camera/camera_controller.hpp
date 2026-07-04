/**************************************************************************/
/* camera_controller.hpp                                                  */
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

#ifndef SUSHIENGINE_EDITOR_CAMERA_CONTROLLER_HPP
#define SUSHIENGINE_EDITOR_CAMERA_CONTROLLER_HPP

#include "fly_camera.hpp"
#include "../input/input_state.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief Drives a FlyCamera from an InputState with Unity Scene-view controls.
         *
         * Right mouse gates look and WASD/QE flight (Shift boosts); the mouse wheel
         * dollies along the view direction and the middle mouse pans in the view plane,
         * both without holding right mouse — Unity's Scene navigation. All motion is
         * scaled by frame time (wheel and pan by their fixed steps) so it is frame-rate
         * independent. Stateless and pure — reads input, writes the camera — so it is
         * trivially testable.
         */
        struct CameraController
        {
            SushiEngine::Scalar look_sensitivity = SushiEngine::Scalar(0.0032); /**< Radians of turn per pixel of mouse move. */
            SushiEngine::Scalar move_speed = SushiEngine::Scalar(6);           /**< Base fly speed in units per second. */
            SushiEngine::Scalar boost_multiplier = SushiEngine::Scalar(4);     /**< Speed multiplier while Shift is held. */
            SushiEngine::Scalar pitch_limit = SushiEngine::Scalar(1.5533);     /**< Clamp pitch to just under +/- 90 degrees. */
            SushiEngine::Scalar zoom_step = SushiEngine::Scalar(0.8);          /**< Units dollied per wheel notch. */
            SushiEngine::Scalar pan_speed = SushiEngine::Scalar(0.01);         /**< World units panned per pixel of mouse move. */

            /**
             * @brief Applies one frame of input to @p camera.
             * @param camera The camera to move and turn.
             * @param input  This frame's navigation snapshot.
             */
            void update(FlyCamera& camera, const InputState& input) const noexcept
            {
                // Wheel dolly and middle-mouse pan work whether or not look is active, so
                // the view can be framed without holding right mouse (Unity Scene nav).
                if (input.wheel != 0.0f)
                    camera.position = camera.position + camera.forward() * (static_cast<SushiEngine::Scalar>(input.wheel) * zoom_step);
                if (input.pan_active && (input.pan_dx != 0.0f || input.pan_dy != 0.0f))
                {
                    const SushiEngine::Vector3 right = camera.right();
                    const SushiEngine::Vector3 up = SushiEngine::cross(right, camera.forward());
                    camera.position = camera.position + right * (-static_cast<SushiEngine::Scalar>(input.pan_dx) * pan_speed) +
                                      up * (static_cast<SushiEngine::Scalar>(input.pan_dy) * pan_speed);
                }

                if (!input.look_active)
                    return;

                camera.yaw_radians += static_cast<SushiEngine::Scalar>(input.mouse_dx) * look_sensitivity;
                camera.pitch_radians -= static_cast<SushiEngine::Scalar>(input.mouse_dy) * look_sensitivity;
                if (camera.pitch_radians > pitch_limit)
                    camera.pitch_radians = pitch_limit;
                if (camera.pitch_radians < -pitch_limit)
                    camera.pitch_radians = -pitch_limit;

                const SushiEngine::Scalar speed =
                    move_speed * (input.fast ? boost_multiplier : SushiEngine::Scalar(1)) * static_cast<SushiEngine::Scalar>(input.dt);
                const SushiEngine::Vector3 forward = camera.forward();
                const SushiEngine::Vector3 right = camera.right();
                const SushiEngine::Vector3 world_up{SushiEngine::Scalar(0), SushiEngine::Scalar(1), SushiEngine::Scalar(0)};

                SushiEngine::Vector3 delta{SushiEngine::Scalar(0), SushiEngine::Scalar(0), SushiEngine::Scalar(0)};
                if (input.forward) delta = delta + forward;
                if (input.back)    delta = delta - forward;
                if (input.right)   delta = delta + right;
                if (input.left)    delta = delta - right;
                if (input.up)      delta = delta + world_up;
                if (input.down)    delta = delta - world_up;

                camera.position = camera.position + delta * speed;
            }
        };
    } // namespace Editor
} // namespace SushiEngine

#endif
