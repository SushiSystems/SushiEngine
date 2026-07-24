/**************************************************************************/
/* haptics.hpp                                                           */
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
 * @file haptics.hpp
 * @brief The rumble output seam — shake a pad without seeing SDL.
 *
 * The write-side counterpart to @ref IInputSource: gameplay expresses a rumble as two
 * normalized motor intensities and a duration, and the backend (the SDL translator, which
 * implements this) drives the hardware. Consumers depend on this abstraction, never on
 * `SDL_GameControllerRumble` — the ISP/DIP boundary the rest of the input layer already
 * keeps. The device is explicit so a local-multiplayer game can shake the right player's
 * pad; a single-pad game passes @ref FIRST_GAMEPAD_DEVICE.
 */

#include <SushiEngine/input/events.hpp>

namespace SushiEngine
{
    namespace Input
    {
        /** @brief A sink that plays rumble on a gamepad's motors. */
        class IHapticsSink
        {
            public:
                virtual ~IHapticsSink() = default;

                /**
                 * @brief Plays a rumble on @p device for @p duration_seconds.
                 *
                 * Intensities are normalized to [0, 1] and clamped by the implementation. A
                 * device that is disconnected, has no rumble motors, or names no slot is a
                 * silent no-op — a caller never has to check first.
                 *
                 * @param device           The gamepad slot to shake (@ref FIRST_GAMEPAD_DEVICE for the first).
                 * @param low_frequency    The low-frequency (large) motor intensity, in [0, 1].
                 * @param high_frequency   The high-frequency (small) motor intensity, in [0, 1].
                 * @param duration_seconds How long to play, in seconds.
                 */
                virtual void rumble(DeviceId device, float low_frequency, float high_frequency,
                                    float duration_seconds) = 0;
        };
    } // namespace Input
} // namespace SushiEngine
