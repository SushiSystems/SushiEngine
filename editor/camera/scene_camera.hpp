/**************************************************************************/
/* scene_camera.hpp                                                       */
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

#ifndef SUSHIENGINE_EDITOR_SCENE_CAMERA_HPP
#define SUSHIENGINE_EDITOR_SCENE_CAMERA_HPP

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/render/scene_view.hpp>

#include "camera_controller.hpp"
#include "fly_camera.hpp"
#include "../input/input_state.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief The camera a viewport renders from, decoupled from the panel.
         *
         * A viewport panel owns an offscreen scene view and displays it; where the view
         * is taken from is this abstraction's job. One panel type then serves both the
         * Scene view (a navigable fly camera) and the Game view (a fixed camera driven by
         * the world), and a new camera kind is a new implementation, not a new panel.
         */
        class ISceneCamera
        {
            public:
                virtual ~ISceneCamera() = default;

                /**
                 * @brief Whether this camera consumes navigation input.
                 *
                 * The Scene fly camera does; the Game camera is driven by the world and
                 * ignores the panel's mouse and keyboard.
                 */
                virtual bool navigable() const noexcept = 0;

                /**
                 * @brief Advances the camera from one frame of navigation input.
                 *
                 * A no-op for non-navigable cameras. Only called while the panel is being
                 * interacted with, so input over other panels never moves the view.
                 *
                 * @param input The frame's navigation snapshot.
                 */
                virtual void process(const InputState& input) = 0;

                /**
                 * @brief Builds the view and projection for a target of the given aspect.
                 * @param aspect_ratio Target width / height.
                 * @return The world-to-clip transform split into its two matrices.
                 */
                virtual SushiEngine::Render::CameraView view(float aspect_ratio) const = 0;
        };

        /**
         * @brief A navigable Unity-style fly camera: the Scene view's camera.
         *
         * Wraps a FlyCamera and its stateless controller; `process` drives look and
         * flight while the panel is navigated.
         */
        class FlyCameraSource final : public ISceneCamera
        {
            public:
                bool navigable() const noexcept override { return true; }

                void process(const InputState& input) override
                {
                    controller_.update(camera_, input);
                }

                SushiEngine::Render::CameraView view(float aspect_ratio) const override
                {
                    SushiEngine::Render::CameraView result;
                    result.view = camera_.view_matrix();
                    result.projection = camera_.projection(aspect_ratio);
                    result.world_position = SushiEngine::WorldVector3{
                        camera_.position.x, camera_.position.y, camera_.position.z};
                    result.near_plane = camera_.near_plane;
                    result.far_plane = camera_.far_plane;
                    return result;
                }

                /** @brief The underlying fly camera, for inspection. */
                FlyCamera& camera() noexcept { return camera_; }

                /**
                 * @brief Sets the base fly speed, from the editor's camera-speed preference.
                 * @param units_per_second Base movement speed before the Shift boost.
                 */
                void set_move_speed(SushiEngine::Scalar units_per_second) noexcept
                {
                    controller_.move_speed = units_per_second;
                }

            private:
                FlyCamera camera_;
                CameraController controller_;
        };

        /**
         * @brief A fixed camera posed by the world each frame: the Game view's camera.
         *
         * Holds an eye/target/up frame and lens parameters, set from the simulation's
         * camera state; ignores navigation input. Kept free of the simulation's own
         * types so the seam stays one-directional — the host maps the world camera onto
         * `set_pose`.
         */
        class WorldCameraSource final : public ISceneCamera
        {
            public:
                bool navigable() const noexcept override { return false; }

                void process(const InputState&) override {}

                /**
                 * @brief Sets the camera pose and lens for this frame.
                 * @param position     Eye position in world space.
                 * @param target       Point the camera looks at.
                 * @param up           World up direction.
                 * @param vertical_fov Vertical field of view in radians.
                 * @param near_plane   Near clip distance (> 0).
                 * @param far_plane    Far clip distance (> near).
                 */
                void set_pose(const SushiEngine::Vector3& position, const SushiEngine::Vector3& target,
                              const SushiEngine::Vector3& up, SushiEngine::Scalar vertical_fov,
                              SushiEngine::Scalar near_plane, SushiEngine::Scalar far_plane) noexcept
                {
                    position_ = position;
                    target_ = target;
                    up_ = up;
                    vertical_fov_ = vertical_fov;
                    near_plane_ = near_plane;
                    far_plane_ = far_plane;
                }

                SushiEngine::Render::CameraView view(float aspect_ratio) const override
                {
                    SushiEngine::Render::CameraView result;
                    result.view = SushiEngine::look_at(position_, target_, up_);
                    result.projection =
                        SushiEngine::perspective(vertical_fov_, aspect_ratio, near_plane_, far_plane_);
                    result.world_position =
                        SushiEngine::WorldVector3{position_.x, position_.y, position_.z};
                    result.near_plane = static_cast<float>(near_plane_);
                    result.far_plane = static_cast<float>(far_plane_);
                    return result;
                }

            private:
                SushiEngine::Vector3 position_{SushiEngine::Vector3{0, 7, 12}};
                SushiEngine::Vector3 target_{SushiEngine::Vector3{0, 0, 0}};
                SushiEngine::Vector3 up_{SushiEngine::Vector3{0, 1, 0}};
                SushiEngine::Scalar vertical_fov_ = SushiEngine::Scalar(1.0471976);
                SushiEngine::Scalar near_plane_ = SushiEngine::Scalar(0.1);
                SushiEngine::Scalar far_plane_ = SushiEngine::Scalar(500);
        };
    } // namespace Editor
} // namespace SushiEngine

#endif
