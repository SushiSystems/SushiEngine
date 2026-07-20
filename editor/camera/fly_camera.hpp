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
         * @brief A quaternion free-look camera, the editor Scene view's viewpoint.
         *
         * Orientation is a single unit quaternion with the engine's facing convention:
         * identity looks down -Z with +Y up. There is no global "up" baked in — the
         * CameraController yaws about whatever up reference gravity supplies (the local
         * vertical of the nearest planet), so the horizon stays level anywhere on any
         * body, northern or southern hemisphere alike. It holds only state and pure
         * queries; the CameraController mutates it from input.
         */
        struct FlyCamera
        {
            SushiEngine::Vector3 position{SushiEngine::Scalar(0), SushiEngine::Scalar(2.5), SushiEngine::Scalar(7)};
            /** Unit orientation; identity faces -Z. Default pitches 0.30 rad down. */
            SushiEngine::Quaternion rotation = SushiEngine::quaternion_axis_angle(
                SushiEngine::Vector3{SushiEngine::Scalar(1), SushiEngine::Scalar(0), SushiEngine::Scalar(0)},
                SushiEngine::Scalar(-0.30));
            SushiEngine::Scalar fov_radians = SushiEngine::Scalar(1.0471976);   /**< 60 degrees vertical. */
            SushiEngine::Scalar near_plane = SushiEngine::Scalar(0.1);
            SushiEngine::Scalar far_plane = SushiEngine::Scalar(500);

            /** @brief Unit forward direction (the rotated -Z axis). */
            SushiEngine::Vector3 forward() const noexcept
            {
                return SushiEngine::rotate(rotation, SushiEngine::Vector3{
                    SushiEngine::Scalar(0), SushiEngine::Scalar(0), SushiEngine::Scalar(-1)});
            }

            /** @brief Unit right direction (the rotated +X axis). */
            SushiEngine::Vector3 right() const noexcept
            {
                return SushiEngine::rotate(rotation, SushiEngine::Vector3{
                    SushiEngine::Scalar(1), SushiEngine::Scalar(0), SushiEngine::Scalar(0)});
            }

            /** @brief Unit up direction (the rotated +Y axis). */
            SushiEngine::Vector3 up() const noexcept
            {
                return SushiEngine::rotate(rotation, SushiEngine::Vector3{
                    SushiEngine::Scalar(0), SushiEngine::Scalar(1), SushiEngine::Scalar(0)});
            }

            /**
             * @brief The camera's orientation as a quaternion, for aligning objects to it.
             *
             * The stored rotation directly: an identity-rotated object faces -Z, the
             * engine's forward convention, so "Align With View" can assign it verbatim.
             *
             * @return The unit quaternion whose forward (-Z) equals the camera's forward.
             */
            SushiEngine::Quaternion orientation() const noexcept
            {
                return rotation;
            }

            /** @brief The world-to-camera view matrix. */
            SushiEngine::Mat4 view_matrix() const noexcept
            {
                return SushiEngine::look_at(position, position + forward(), up());
            }

            /**
             * @brief The projection matrix for a given viewport aspect ratio.
             * @param aspect Width / height of the target; treated as 1 if not positive.
             * @return The Vulkan-space perspective projection.
             */
            SushiEngine::Mat4 projection(float aspect) const noexcept
            {
                const SushiEngine::Scalar a = static_cast<SushiEngine::Scalar>(aspect);
                return SushiEngine::perspective(fov_radians, a > SushiEngine::Scalar(0) ? a : SushiEngine::Scalar(1),
                                                near_plane, far_plane);
            }
        };
    } // namespace Editor
} // namespace SushiEngine

#endif
