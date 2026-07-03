/**************************************************************************/
/* input.hpp                                                              */
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
 * @file input.hpp
 * @brief SushiLoop's per-tick input capture: a numbered command buffer.
 *
 * docs/slop/SUSHILOOP.md models input as "an abstract list of commands for each
 * tick", numbered so the network layer can transmit it and rollback can replay it
 * exactly. `InputHistory<Command>` is the host-side buffer that records and looks
 * up a game's own command type by tick; the network/rollback layers (M3/M4) are
 * the ones that will populate and consume it. `Command` is left to the game (or a
 * later gameplay layer) — this file only fixes the numbering and storage shape.
 */

#include <cstddef>
#include <cstdint>
#include <vector>

namespace SushiEngine
{
    namespace Loop
    {
        /** @brief A monotonically increasing simulation tick number. */
        using TickId = std::uint64_t;

        /**
         * @brief Records one command stream, one entry per tick, in tick order.
         *
         * @tparam Command The per-tick command type; trivially copyable in practice
         * since it must cross the network and be replayed during rollback, though
         * this container does not itself enforce that.
         */
        template <typename Command>
        class InputHistory
        {
            public:
                /**
                 * @brief Records @p command for @p tick.
                 *
                 * Ticks must be recorded in non-decreasing order (the loop calls this
                 * once per tick as it advances), which keeps lookup a binary search
                 * over a sorted, append-only buffer.
                 *
                 * @param tick    The tick this command applies to.
                 * @param command The captured command for that tick.
                 */
                void record(TickId tick, Command command)
                {
                    entries_.push_back(Entry{tick, std::move(command)});
                }

                /** @brief Number of recorded ticks. */
                std::size_t size() const noexcept { return entries_.size(); }

                /** @brief Whether no tick has been recorded yet. */
                bool empty() const noexcept { return entries_.empty(); }

                /**
                 * @brief Looks up the command recorded for @p tick.
                 * @param tick The tick to look up.
                 * @return A pointer to the stored command, or `nullptr` if @p tick was
                 * never recorded.
                 */
                const Command* find(TickId tick) const noexcept
                {
                    for (const Entry& e : entries_)
                        if (e.tick == tick)
                            return &e.command;
                    return nullptr;
                }

                /** @brief Discards every recorded tick. */
                void clear() noexcept { entries_.clear(); }

                /**
                 * @brief Overwrites the command recorded for @p tick, or appends it.
                 *
                 * Unlike `record`, this is not append-only: it is how a client's own
                 * predicted command is replaced with a server's later-arriving
                 * authoritative one (SushiLoop M4's reconciliation), without disturbing
                 * every other recorded tick's position in the buffer.
                 *
                 * @param tick    The tick to overwrite (or append, if never recorded).
                 * @param command The corrected command for that tick.
                 */
                void correct(TickId tick, Command command)
                {
                    for (Entry& e : entries_)
                    {
                        if (e.tick == tick)
                        {
                            e.command = std::move(command);
                            return;
                        }
                    }
                    entries_.push_back(Entry{tick, std::move(command)});
                }

            private:
                struct Entry
                {
                    TickId tick;
                    Command command;
                };

                std::vector<Entry> entries_;
        };
    } // namespace Loop
} // namespace SushiEngine
