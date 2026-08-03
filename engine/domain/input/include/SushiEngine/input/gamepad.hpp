/**************************************************************************/
/* gamepad.hpp                                                           */
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
 * @file gamepad.hpp
 * @brief Stable device-slot allocation for hot-plugged gamepads — SDL-free and testable.
 *
 * A gamepad's identity to the rest of the engine is its @ref DeviceId slot, not the
 * backend's connection handle. This table maps a backend instance id (SDL's
 * `SDL_JoystickID`, but any opaque integer works) to the lowest free slot on connect and
 * keeps it until that instance disconnects, so a binding or player assignment made against
 * "gamepad 0" survives an unplug and replug of the same controller ordering. Separated from
 * the SDL translator (SRP) precisely so this policy is unit-testable with no window and no
 * hardware — the translator only adds the SDL open/close and the reverse handle it needs to
 * rumble.
 */

#include <array>
#include <cstdint>

#include <SushiEngine/input/device_registry.hpp>
#include <SushiEngine/input/events.hpp>

namespace SushiEngine
{
    namespace Input
    {
        /** @brief A backend gamepad instance id (SDL_JoystickID); negative means none. */
        using GamepadInstanceId = std::int32_t;

        /** @brief The instance-id value standing for "no instance". */
        constexpr GamepadInstanceId INVALID_INSTANCE = -1;

        /**
         * @brief Allocates stable @ref DeviceId slots to gamepad instance ids, lowest-free.
         *
         * Holds at most @ref MAX_GAMEPADS instances. The mapping is a bijection between
         * occupied slots and live instance ids: @ref connect claims the lowest free slot,
         * @ref disconnect releases it, and the two lookups read it in either direction.
         */
        class GamepadSlotTable
        {
            public:
                /**
                 * @brief Assigns @p instance the lowest free slot, or returns its existing one.
                 * @param instance The backend instance id to place.
                 * @return The device slot id, or @ref INVALID_DEVICE if the table is full.
                 */
                DeviceId connect(GamepadInstanceId instance) noexcept
                {
                    const DeviceId existing = device_for(instance);
                    if (existing != INVALID_DEVICE)
                        return existing;

                    for (int slot = 0; slot < MAX_GAMEPADS; ++slot)
                    {
                        if (slots_[static_cast<std::size_t>(slot)] == INVALID_INSTANCE)
                        {
                            slots_[static_cast<std::size_t>(slot)] = instance;
                            return static_cast<DeviceId>(FIRST_GAMEPAD_DEVICE + slot);
                        }
                    }
                    return INVALID_DEVICE;
                }

                /**
                 * @brief Releases the slot holding @p instance.
                 * @param instance The backend instance id to remove.
                 * @return The device slot it held, or @ref INVALID_DEVICE if it was not present.
                 */
                DeviceId disconnect(GamepadInstanceId instance) noexcept
                {
                    for (int slot = 0; slot < MAX_GAMEPADS; ++slot)
                    {
                        if (slots_[static_cast<std::size_t>(slot)] == instance)
                        {
                            slots_[static_cast<std::size_t>(slot)] = INVALID_INSTANCE;
                            return static_cast<DeviceId>(FIRST_GAMEPAD_DEVICE + slot);
                        }
                    }
                    return INVALID_DEVICE;
                }

                /**
                 * @brief The device slot currently holding @p instance.
                 * @param instance The backend instance id to look up.
                 * @return The device slot id, or @ref INVALID_DEVICE if not connected.
                 */
                DeviceId device_for(GamepadInstanceId instance) const noexcept
                {
                    for (int slot = 0; slot < MAX_GAMEPADS; ++slot)
                        if (slots_[static_cast<std::size_t>(slot)] == instance)
                            return static_cast<DeviceId>(FIRST_GAMEPAD_DEVICE + slot);
                    return INVALID_DEVICE;
                }

                /**
                 * @brief The backend instance id occupying @p device.
                 * @param device The device slot id.
                 * @return The instance id, or @ref INVALID_INSTANCE if the slot is free/out of range.
                 */
                GamepadInstanceId instance_for(DeviceId device) const noexcept
                {
                    const int slot = device - FIRST_GAMEPAD_DEVICE;
                    if (slot < 0 || slot >= MAX_GAMEPADS)
                        return INVALID_INSTANCE;
                    return slots_[static_cast<std::size_t>(slot)];
                }

                /** @brief The number of occupied slots. */
                int connected_count() const noexcept
                {
                    int count = 0;
                    for (const GamepadInstanceId instance : slots_)
                        if (instance != INVALID_INSTANCE)
                            ++count;
                    return count;
                }

            private:
                std::array<GamepadInstanceId, MAX_GAMEPADS> slots_ = filled_free();

                static std::array<GamepadInstanceId, MAX_GAMEPADS> filled_free() noexcept
                {
                    std::array<GamepadInstanceId, MAX_GAMEPADS> slots{};
                    for (GamepadInstanceId& instance : slots)
                        instance = INVALID_INSTANCE;
                    return slots;
                }
        };
    } // namespace Input
} // namespace SushiEngine
