/**************************************************************************/
/* game_view_render_policy.hpp                                            */
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

#ifndef SUSHIENGINE_EDITOR_GAME_VIEW_RENDER_POLICY_HPP
#define SUSHIENGINE_EDITOR_GAME_VIEW_RENDER_POLICY_HPP

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief The inputs a render-gate decision is made from: what the world can play through.
         *
         * A plain value type so callers can assemble it from whatever they already track
         * (`scene.has_camera`, the display list) without the gate reaching into any of
         * those systems itself.
         */
        struct GameViewRenderInputs
        {
            bool has_active_camera = false; /**< Whether the world has a camera to render from. */
            bool has_display = false;       /**< Whether a display exists for the camera to target. */
        };

        /**
         * @brief Decides whether the Game view should submit a render pass this frame.
         *
         * Single responsibility: this is the one place that answers "is there anything
         * to play the scene through and onto." Callers (the frame loop) ask it before
         * invoking @ref ViewportPanel::draw, instead of embedding the condition in the
         * loop or teaching the panel/scene-view about camera and display existence.
         */
        class GameViewRenderPolicy
        {
            public:
                /**
                 * @brief Whether the Game view should render given the current inputs.
                 * @param inputs The camera/display availability for this frame.
                 * @return True if the Game view should submit a render pass.
                 */
                bool should_render(const GameViewRenderInputs& inputs) const
                {
                    return inputs.has_active_camera && inputs.has_display;
                }
        };
    }
}

#endif
