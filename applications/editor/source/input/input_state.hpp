/**************************************************************************/
/* input_state.hpp                                                        */
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

#ifndef SUSHIENGINE_EDITOR_INPUT_STATE_HPP
#define SUSHIENGINE_EDITOR_INPUT_STATE_HPP

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief One frame's navigation input, decoupled from any input library.
         *
         * A neutral snapshot the viewport panel fills (from ImGui, which owns the SDL
         * events) and the camera controller consumes, so the controller carries no
         * dependency on the input source and stays unit-testable. Movement flags follow
         * Unity's fly controls (WASD + Q/E for down/up, Shift to boost); look is active
         * only while the right mouse button is held.
         */
        struct InputState
        {
            bool forward = false; /**< W: move along the camera's forward axis. */
            bool back = false;    /**< S: move backward. */
            bool left = false;    /**< A: strafe left. */
            bool right = false;   /**< D: strafe right. */
            bool up = false;      /**< E: rise along world up. */
            bool down = false;    /**< Q: descend along world up. */
            bool fast = false;    /**< Shift: apply the speed boost. */

            bool look_active = false; /**< Right mouse held: enable look and movement. */
            float mouse_dx = 0.0f;    /**< Horizontal mouse delta in pixels this frame. */
            float mouse_dy = 0.0f;    /**< Vertical mouse delta in pixels this frame. */

            float wheel = 0.0f;       /**< Mouse-wheel notches this frame: dolly along forward. */

            bool pan_active = false;  /**< Middle mouse held: pan in the view plane. */
            float pan_dx = 0.0f;      /**< Horizontal mouse delta while panning, in pixels. */
            float pan_dy = 0.0f;      /**< Vertical mouse delta while panning, in pixels. */

            float dt = 0.0f; /**< Frame time in seconds, for rate-independent motion. */
        };
    } // namespace Editor
} // namespace SushiEngine

#endif
