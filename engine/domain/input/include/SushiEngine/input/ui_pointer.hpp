/**************************************************************************/
/* ui_pointer.hpp                                                        */
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
 * @file ui_pointer.hpp
 * @brief Adapts the registry's primary pointer to the engine UI's `UI::PointerInput`.
 *
 * The engine UI already reacts to one host-fed `UI::PointerInput` per frame. This adapter
 * lets the input layer be that one pointer source: the primary pointer (a finger, or the
 * mouse when @ref DeviceRegistry::set_mouse_as_pointer is on) becomes the UI's pointer, so
 * the host no longer hand-feeds it. Kept in its own opt-in header so the core action layer
 * stays free of the `UI` types (which are `Scalar`/double, for screen geometry); only a
 * consumer that drives the UI includes this.
 */

#include <SushiEngine/input/device_registry.hpp>
#include <SushiEngine/ui/interaction.hpp>

namespace SushiEngine
{
    namespace Input
    {
        /**
         * @brief Builds a `UI::PointerInput` from @p registry's primary pointer.
         *
         * The position is converted from the input layer's `float` pixels to the UI's
         * `Scalar` (double) screen space; `down` is the primary pointer's active state.
         *
         * @param registry The device state to read pointer 0 from.
         * @return The per-frame pointer the UI façade consumes.
         */
        inline UI::PointerInput ui_pointer(const DeviceRegistry& registry)
        {
            const DeviceRegistry::Pointer& pointer = registry.primary_pointer();
            UI::PointerInput input;
            input.position = UI::Vector2{static_cast<Scalar>(pointer.position.x),
                                         static_cast<Scalar>(pointer.position.y)};
            input.down = pointer.active;
            return input;
        }
    } // namespace Input
} // namespace SushiEngine
