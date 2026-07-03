/**************************************************************************/
/* fly_camera.hpp                                                         */
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

#ifndef SUSHIENGINE_EDITOR_FLY_CAMERA_HPP
#define SUSHIENGINE_EDITOR_FLY_CAMERA_HPP

#include <cmath>

#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief A yaw/pitch free-look camera, the editor Scene view's viewpoint.
         *
         * Stores position and orientation as yaw and pitch (no roll, like Unity's Scene
         * camera) and derives the basis vectors and matrices on demand. It holds only
         * state and pure queries; the CameraController mutates it from input.
         */
        struct FlyCamera
        {
            SushiEngine::Vector3 position{0.0f, 2.5f, 7.0f};
            float yaw_radians = -1.5707963f;  /**< 0 faces +X; this faces -Z. */
            float pitch_radians = -0.30f;     /**< Negative looks slightly down. */
            float fov_radians = 1.0471976f;   /**< 60 degrees vertical. */
            float near_plane = 0.1f;
            float far_plane = 500.0f;

            /** @brief Unit forward direction from the current yaw and pitch. */
            SushiEngine::Vector3 forward() const noexcept
            {
                const float cos_pitch = std::cos(pitch_radians);
                return SushiEngine::normalize(SushiEngine::Vector3{
                    cos_pitch * std::cos(yaw_radians), std::sin(pitch_radians),
                    cos_pitch * std::sin(yaw_radians)});
            }

            /** @brief Unit right direction (forward x world-up). */
            SushiEngine::Vector3 right() const noexcept
            {
                return SushiEngine::normalize(
                    SushiEngine::cross(forward(), SushiEngine::Vector3{0.0f, 1.0f, 0.0f}));
            }

            /**
             * @brief The camera's orientation as a quaternion, for aligning objects to it.
             *
             * Built so that at the default yaw/pitch an identity-rotated object (which
             * faces -Z, the engine's forward convention) matches the camera's facing.
             * Used by "Align With View": a yaw about world up, then pitch about the local
             * right axis.
             *
             * @return The unit quaternion whose forward (-Z) equals the camera's forward.
             */
            SushiEngine::Quaternion orientation() const noexcept
            {
                const float yaw_theta = std::atan2(-std::cos(yaw_radians), -std::sin(yaw_radians));
                const SushiEngine::Quaternion yaw_q =
                    SushiEngine::quaternion_axis_angle(SushiEngine::Vector3{0.0f, 1.0f, 0.0f}, yaw_theta);
                const SushiEngine::Quaternion pitch_q =
                    SushiEngine::quaternion_axis_angle(SushiEngine::Vector3{1.0f, 0.0f, 0.0f}, pitch_radians);
                return SushiEngine::normalize(SushiEngine::mul(yaw_q, pitch_q));
            }

            /** @brief The world-to-camera view matrix. */
            SushiEngine::Mat4 view_matrix() const noexcept
            {
                return SushiEngine::look_at(position, position + forward(),
                                            SushiEngine::Vector3{0.0f, 1.0f, 0.0f});
            }

            /**
             * @brief The projection matrix for a given viewport aspect ratio.
             * @param aspect Width / height of the target; treated as 1 if not positive.
             * @return The Vulkan-space perspective projection.
             */
            SushiEngine::Mat4 projection(float aspect) const noexcept
            {
                return SushiEngine::perspective(fov_radians, aspect > 0.0f ? aspect : 1.0f,
                                                near_plane, far_plane);
            }
        };
    } // namespace Editor
} // namespace SushiEngine

#endif
