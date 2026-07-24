/**************************************************************************/
/* player.hpp                                                            */
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
 * @file player.hpp
 * @brief Local-multiplayer routing — one mapper per player over a shared device state (§2.6).
 *
 * N local players is N reductions of the same input: each player owns a @ref PlayerHandle
 * (its own @ref ActionMapper and @ref TickSampleAccumulator, resolving the *shared* contexts
 * against its own @ref DeviceAssignment), and the game's `Command` carries one reduction per
 * player. Device state is folded once, globally, into the shared @ref DeviceRegistry; routing
 * is purely which devices each player's assignment reads. A claim ("press A to join") is just
 * pointing a player's assignment at a gamepad. This composes untouched with SushiLoop — a
 * player is an ECS entity, its input a per-tick command — and the per-mapper routing costs
 * nothing to carry in single-player.
 */

#include <cstdint>
#include <memory>
#include <vector>

#include <SushiEngine/input/action_map.hpp>
#include <SushiEngine/input/controls.hpp>
#include <SushiEngine/input/device_registry.hpp>
#include <SushiEngine/input/tick_sample.hpp>

namespace SushiEngine
{
    namespace Input
    {
        /**
         * @brief One local player's input: its own mapper, accumulator, and device assignment.
         *
         * Non-copyable (its mapper holds pointers to the shared contexts). Contexts are pushed
         * per player, but the @ref InputContext objects themselves are shared across players —
         * every player binds the same `"Move"`/`"Jump"`, resolved against different devices.
         */
        class PlayerHandle
        {
            public:
                /**
                 * @brief Creates player @p index with device assignment @p devices.
                 * @param index   The player's stable index (its slot in the roster).
                 * @param devices The devices this player's bindings resolve against.
                 */
                PlayerHandle(int index, const DeviceAssignment& devices) : index_(index), devices_(devices)
                {
                    mapper_.set_device_assignment(devices_);
                }

                PlayerHandle(const PlayerHandle&) = delete;
                PlayerHandle& operator=(const PlayerHandle&) = delete;

                /** @brief The player's stable index. */
                int index() const noexcept { return index_; }

                /** @brief Routes this player's bindings to @p devices. */
                void set_devices(const DeviceAssignment& devices) noexcept
                {
                    devices_ = devices;
                    mapper_.set_device_assignment(devices_);
                }

                /** @brief The devices this player reads. */
                const DeviceAssignment& devices() const noexcept { return devices_; }

                /** @brief Pushes a shared @p context onto this player's stack. */
                void push_context(InputContext& context) { mapper_.push_context(context); }

                /** @brief Pops this player's highest-priority context. */
                void pop_context() { mapper_.pop_context(); }

                /**
                 * @brief Resolves this player's actions against @p registry and folds the tick sample.
                 * @param registry The shared device state.
                 * @param gate     The immediate-mode UI capture gate.
                 */
                void update(const DeviceRegistry& registry, const InputGate& gate)
                {
                    mapper_.update(registry, gate);
                    accumulator_.accumulate(mapper_.snapshot());
                }

                /** @brief This player's resolved actions this frame. */
                const ActionSnapshot& snapshot() const noexcept { return mapper_.snapshot(); }

                /** @brief Reduces this player's frames since its last tick into one sample. */
                TickSample consume_tick_sample() { return accumulator_.consume(); }

                /** @brief This player's mapper, for advanced context management. */
                ActionMapper& mapper() noexcept { return mapper_; }

            private:
                int index_;
                DeviceAssignment devices_;
                ActionMapper mapper_;
                TickSampleAccumulator accumulator_;
        };

        /**
         * @brief A set of local players over one shared device state, with device claiming.
         *
         * The game folds device events once (through an @ref InputManager or by hand), then calls
         * @ref update to resolve every player. Default policy is single-player until a device is
         * claimed: player 0 reads keyboard+mouse and the first pad; a second player claims a pad
         * with @ref claim_gamepad (typically after @ref join_candidates reports it pressed a join
         * button). Players are stored with stable addresses so their mappers' context pointers
         * stay valid as the roster grows.
         */
        class PlayerRoster
        {
            public:
                /**
                 * @brief Adds a player reading @p devices and returns it.
                 * @param devices The new player's device assignment (default: keyboard+mouse+first pad).
                 * @return A reference to the created player, stable for the roster's lifetime.
                 */
                PlayerHandle& add_player(const DeviceAssignment& devices = DeviceAssignment{})
                {
                    const int index = static_cast<int>(players_.size());
                    players_.push_back(std::make_unique<PlayerHandle>(index, devices));
                    return *players_.back();
                }

                /** @brief The number of players. */
                std::size_t size() const noexcept { return players_.size(); }

                /** @brief Player @p index. */
                PlayerHandle& player(std::size_t index) { return *players_[index]; }

                /** @brief Player @p index (const). */
                const PlayerHandle& player(std::size_t index) const { return *players_[index]; }

                /** @brief Resolves every player against @p registry under @p gate. */
                void update(const DeviceRegistry& registry, const InputGate& gate)
                {
                    for (const std::unique_ptr<PlayerHandle>& handle : players_)
                        handle->update(registry, gate);
                }

                /**
                 * @brief The player index that owns gamepad @p device, or -1 if none.
                 * @param device The gamepad slot to look up.
                 */
                int gamepad_owner(DeviceId device) const noexcept
                {
                    for (const std::unique_ptr<PlayerHandle>& handle : players_)
                        if (handle->devices().gamepad == device)
                            return handle->index();
                    return -1;
                }

                /** @brief Routes gamepad @p device to @p player (a claim). */
                void claim_gamepad(std::size_t player_index, DeviceId device)
                {
                    DeviceAssignment devices = players_[player_index]->devices();
                    devices.gamepad = device;
                    players_[player_index]->set_devices(devices);
                }

                /**
                 * @brief Connected gamepads no player owns that are pressing @p join_button.
                 *
                 * The "press A to join" query: a game scans these each frame and, for each, adds a
                 * player (or claims into an empty slot) and calls @ref claim_gamepad. Kept a query so
                 * the join policy (how many players, which slot) stays the game's decision.
                 *
                 * @param registry    The shared device state.
                 * @param join_button The button an unassigned pad presses to join.
                 * @return The device slots of unowned, connected pads holding @p join_button.
                 */
                std::vector<DeviceId> join_candidates(const DeviceRegistry& registry,
                                                      GamepadButton join_button) const
                {
                    std::vector<DeviceId> candidates;
                    for (DeviceId slot = 0; slot < MAX_GAMEPADS; ++slot)
                    {
                        const DeviceId device = static_cast<DeviceId>(FIRST_GAMEPAD_DEVICE + slot);
                        if (!registry.connected(device))
                            continue;
                        if (gamepad_owner(device) != -1)
                            continue;
                        if (registry.gamepad_button(device, join_button))
                            candidates.push_back(device);
                    }
                    return candidates;
                }

            private:
                std::vector<std::unique_ptr<PlayerHandle>> players_;
        };
    } // namespace Input
} // namespace SushiEngine
