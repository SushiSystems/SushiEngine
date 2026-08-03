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
            bool altitude_adaptive = true;                                     /**< Scale all motion with proximity so ground-to-planet flight is smooth. */
            SushiEngine::Scalar reference_height = SushiEngine::Scalar(2);     /**< Distance at which the scale is 1x; beyond it motion grows linearly. */
            SushiEngine::Scalar proximity_distance = SushiEngine::Scalar(-1);  /**< Distance to the nearest body surface, metres; negative falls back to camera height. */
            SushiEngine::Vector3 up_reference{SushiEngine::Scalar(0), SushiEngine::Scalar(1), SushiEngine::Scalar(0)}; /**< The local vertical: yaw axis, pitch-clamp pole, and vertical fly axis. */
            SushiEngine::Scalar up_realign_rate = SushiEngine::Scalar(1.2);    /**< Radians/second the up reference may swing toward its target. */

            /**
             * @brief The motion scale for the camera's distance to the nearest surface.
             *
             * When the caller supplies @ref proximity_distance (the distance to the
             * nearest solar-system body's surface), motion scales linearly with it; with
             * no supplied distance the camera's height above the local ground plane is
             * the proxy. Clamped to at least 1x near a surface, so a fixed base speed
             * carries the camera smoothly from a few metres above the ground out past the
             * Moon and the planets without the controls feeling glacial up high or
             * uncontrollable down low — the standard space-sim camera behaviour.
             *
             * @param camera The camera whose position sets the fallback height.
             * @return A multiplier applied to every translation this frame.
             */
            SushiEngine::Scalar motion_scale(const FlyCamera& camera) const noexcept
            {
                if (!altitude_adaptive)
                    return SushiEngine::Scalar(1);
                const SushiEngine::Scalar measure = proximity_distance >= SushiEngine::Scalar(0)
                                                        ? proximity_distance
                                                        : camera.position.y;
                const SushiEngine::Scalar distance =
                    measure > reference_height ? measure : reference_height;
                return distance / reference_height;
            }

            /**
             * @brief Applies one frame of input to @p camera.
             * @param camera The camera to move and turn.
             * @param input  This frame's navigation snapshot.
             */
            void update(FlyCamera& camera, const InputState& input) const noexcept
            {
                const SushiEngine::Scalar scale = motion_scale(camera);

                // Wheel dolly and middle-mouse pan work whether or not look is active, so
                // the view can be framed without holding right mouse (Unity Scene nav).
                if (input.wheel != 0.0f)
                    camera.position = camera.position + camera.forward() * (static_cast<SushiEngine::Scalar>(input.wheel) * zoom_step * scale);
                if (input.pan_active && (input.pan_dx != 0.0f || input.pan_dy != 0.0f))
                {
                    const SushiEngine::Vector3 right = camera.right();
                    const SushiEngine::Vector3 up = SushiEngine::cross(right, camera.forward());
                    camera.position = camera.position + right * (-static_cast<SushiEngine::Scalar>(input.pan_dx) * pan_speed * scale) +
                                      up * (static_cast<SushiEngine::Scalar>(input.pan_dy) * pan_speed * scale);
                }

                if (!input.look_active)
                    return;

                // Free look in the local-vertical frame: yaw spins about the up reference
                // (gravity's vertical, not a global +Y) and pitch tilts about the camera's
                // own right axis, clamped against that same vertical — so the horizon
                // stays level anywhere on any planet and the poles never flip the view.
                const SushiEngine::Scalar yaw_delta =
                    -static_cast<SushiEngine::Scalar>(input.mouse_dx) * look_sensitivity;
                SushiEngine::Scalar pitch_delta =
                    -static_cast<SushiEngine::Scalar>(input.mouse_dy) * look_sensitivity;

                const SushiEngine::Scalar current_sine =
                    SushiEngine::dot(camera.forward(), up_reference);
                const SushiEngine::Scalar clamped_sine =
                    current_sine > SushiEngine::Scalar(1)
                        ? SushiEngine::Scalar(1)
                        : (current_sine < SushiEngine::Scalar(-1) ? SushiEngine::Scalar(-1)
                                                                  : current_sine);
                const SushiEngine::Scalar current_pitch = std::asin(clamped_sine);
                if (current_pitch + pitch_delta > pitch_limit)
                    pitch_delta = pitch_limit - current_pitch;
                if (current_pitch + pitch_delta < -pitch_limit)
                    pitch_delta = -pitch_limit - current_pitch;

                const SushiEngine::Quaternion yaw_rotation =
                    SushiEngine::quaternion_axis_angle(up_reference, yaw_delta);
                const SushiEngine::Quaternion pitch_rotation =
                    SushiEngine::quaternion_axis_angle(camera.right(), pitch_delta);
                camera.rotation = SushiEngine::normalize(
                    SushiEngine::mul(yaw_rotation, SushiEngine::mul(pitch_rotation, camera.rotation)));

                const SushiEngine::Scalar speed =
                    move_speed * (input.fast ? boost_multiplier : SushiEngine::Scalar(1)) *
                    scale * static_cast<SushiEngine::Scalar>(input.dt);
                const SushiEngine::Vector3 forward = camera.forward();
                const SushiEngine::Vector3 right = camera.right();

                SushiEngine::Vector3 delta{SushiEngine::Scalar(0), SushiEngine::Scalar(0), SushiEngine::Scalar(0)};
                if (input.forward) delta = delta + forward;
                if (input.back)    delta = delta - forward;
                if (input.right)   delta = delta + right;
                if (input.left)    delta = delta - right;
                if (input.up)      delta = delta + up_reference;
                if (input.down)    delta = delta - up_reference;

                camera.position = camera.position + delta * speed;
            }

            /**
             * @brief Swings the up reference toward the local vertical, carrying the camera.
             *
             * Parallel transport: the minimal rotation from the current up reference to
             * @p target_up, rate-limited to @ref up_realign_rate, is applied to both the
             * reference and the camera's orientation. Flying around a planet therefore
             * keeps the horizon level with no input, and when the dominant body changes
             * (or the camera crosses to the far hemisphere) the view rolls smoothly to
             * the new vertical over a few frames instead of snapping.
             *
             * @param camera        The camera whose orientation rides the reference.
             * @param target_up     The desired local vertical (radially away from the
             *                      nearest body); need not be unit length.
             * @param delta_seconds Frame time bounding this step's rotation angle.
             */
            void retarget_up(FlyCamera& camera, const SushiEngine::Vector3& target_up,
                             SushiEngine::Scalar delta_seconds) noexcept
            {
                const SushiEngine::Vector3 target = SushiEngine::normalize(target_up);
                const SushiEngine::Scalar cosine = SushiEngine::dot(up_reference, target);
                SushiEngine::Vector3 axis = SushiEngine::cross(up_reference, target);
                const SushiEngine::Scalar axis_length = SushiEngine::length(axis);

                SushiEngine::Scalar angle = std::atan2(axis_length, cosine);
                if (angle < SushiEngine::Scalar(1e-9))
                    return;
                if (axis_length > SushiEngine::Scalar(1e-12))
                    axis = axis * (SushiEngine::Scalar(1) / axis_length);
                else
                    axis = camera.right(); // antipodal flip: pitch over the camera's right

                const SushiEngine::Scalar limit = up_realign_rate * delta_seconds;
                if (angle > limit)
                    angle = limit;

                const SushiEngine::Quaternion transport =
                    SushiEngine::quaternion_axis_angle(axis, angle);
                up_reference = SushiEngine::normalize(SushiEngine::rotate(transport, up_reference));
                camera.rotation =
                    SushiEngine::normalize(SushiEngine::mul(transport, camera.rotation));
            }
        };
    } // namespace Editor
} // namespace SushiEngine

#endif
