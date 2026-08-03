/**************************************************************************/
/* animation_player.hpp                                                   */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/* Licensed under the Apache License, Version 2.0 (the "License");        */
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
 * @file animation_player.hpp
 * @brief The A1 single-clip playback component (legacy-Animation parity).
 *
 * A trivially-copyable ECS component: it names a clip and a skeleton and carries the
 * playback cursor. It is the simplest driver of skinning — play, loop, speed — and the
 * seam the Animator controller of A3 will later replace with a state machine. Advancing
 * the cursor is a fixed-tick, deterministic operation (no wall clock), so this lives in
 * the simulation domain like any other component.
 */

#include <cstdint>

#include <SushiEngine/animation/animation_database.hpp>

namespace SushiEngine
{
    namespace Animation
    {
        /**
         * @brief Plays one clip on a skinned entity: which clip, on which skeleton, and where.
         *
         * Trivially copyable so it lives in an ECS column and snapshots byte-exactly. The
         * cursor @ref time advances by @ref speed × dt each tick while @ref playing.
         */
        struct AnimationPlayer
        {
            AssetId clip = INVALID_ASSET;     /**< The clip asset to play. */
            AssetId skeleton = INVALID_ASSET; /**< The skeleton the clip poses. */
            float time = 0.0f;                /**< Playback cursor in seconds. */
            float speed = 1.0f;               /**< Playback rate multiplier. */
            bool loop = true;                 /**< Whether the clip loops. */
            bool playing = true;              /**< Whether the cursor advances. */

            /**
             * @brief Advances the cursor by one step.
             * @param dt_seconds Elapsed time this step (the fixed tick).
             */
            void advance(float dt_seconds) noexcept
            {
                if (playing)
                    time += dt_seconds * speed;
            }
        };
    } // namespace Animation
} // namespace SushiEngine
