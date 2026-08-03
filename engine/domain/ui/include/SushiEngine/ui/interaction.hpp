/**************************************************************************/
/* interaction.hpp                                                       */
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
 * @file interaction.hpp
 * @brief The pointer input model the UI reacts to, and the click event it emits.
 *
 * The host feeds one `PointerInput` per frame (the cursor position and whether the
 * primary button is held); the `UI` façade turns the frame-to-frame change into
 * button states and click events. A click is press-and-release both inside the same
 * button — the standard UGUI semantics — so a press that drags off the button before
 * release does not fire. Keeping input as an explicit per-frame value (rather than a
 * hidden global) matches SushiLoop's determinism model: the same pointer stream
 * produces the same clicks.
 */

#include <SushiEngine/ecs/entity.hpp>
#include <SushiEngine/ui/rect.hpp>

namespace SushiEngine
{
    namespace UI
    {
        /** @brief The pointer state for one frame: where it is and whether it is held. */
        struct PointerInput
        {
            Vector2 position;   /**< Cursor position in UI (screen) space. */
            bool down = false;  /**< Whether the primary button is held this frame. */
        };

        /** @brief One click, emitted when a button is pressed and released inside itself. */
        struct UIClickEvent
        {
            Entity button; /**< The button entity that was clicked. */
        };
    } // namespace UI
} // namespace SushiEngine
