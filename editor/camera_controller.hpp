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
#include "input_state.hpp"

namespace sushi::editor
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
        float look_sensitivity = 0.0032f; /**< Radians of turn per pixel of mouse move. */
        float move_speed = 6.0f;          /**< Base fly speed in units per second. */
        float boost_multiplier = 4.0f;    /**< Speed multiplier while Shift is held. */
        float pitch_limit = 1.5533f;      /**< Clamp pitch to just under +/- 90 degrees. */
        float zoom_step = 0.8f;           /**< Units dollied per wheel notch. */
        float pan_speed = 0.01f;          /**< World units panned per pixel of mouse move. */

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
                camera.position = camera.position + camera.forward() * (input.wheel * zoom_step);
            if (input.pan_active && (input.pan_dx != 0.0f || input.pan_dy != 0.0f))
            {
                const SushiEngine::Vec3 right = camera.right();
                const SushiEngine::Vec3 up = SushiEngine::cross(right, camera.forward());
                camera.position = camera.position + right * (-input.pan_dx * pan_speed) +
                                  up * (input.pan_dy * pan_speed);
            }

            if (!input.look_active)
                return;

            camera.yaw_radians += input.mouse_dx * look_sensitivity;
            camera.pitch_radians -= input.mouse_dy * look_sensitivity;
            if (camera.pitch_radians > pitch_limit)
                camera.pitch_radians = pitch_limit;
            if (camera.pitch_radians < -pitch_limit)
                camera.pitch_radians = -pitch_limit;

            const float speed =
                move_speed * (input.fast ? boost_multiplier : 1.0f) * input.dt;
            const SushiEngine::Vec3 forward = camera.forward();
            const SushiEngine::Vec3 right = camera.right();
            const SushiEngine::Vec3 world_up{0.0f, 1.0f, 0.0f};

            SushiEngine::Vec3 delta{0.0f, 0.0f, 0.0f};
            if (input.forward) delta = delta + forward;
            if (input.back)    delta = delta - forward;
            if (input.right)   delta = delta + right;
            if (input.left)    delta = delta - right;
            if (input.up)      delta = delta + world_up;
            if (input.down)    delta = delta - world_up;

            camera.position = camera.position + delta * speed;
        }
    };
} // namespace sushi::editor

#endif
