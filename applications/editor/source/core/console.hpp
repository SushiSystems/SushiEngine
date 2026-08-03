/**************************************************************************/
/* console.hpp                                                            */
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
 * @file console.hpp
 * @brief The editor's log: structured lines rather than a list of strings.
 *
 * A console of undifferentiated strings can only be read in full, which in practice
 * means it is not read: the one warning that mattered sits between two hundred
 * "Created entity" lines with nothing to tell them apart. Each line therefore carries
 * its severity and the moment it happened, so the panel can filter, colour, and
 * collapse — and so a future file sink or an automated check has something to key on
 * besides substring matching.
 *
 * Collapsing is deliberately *not* done here. Consecutive identical lines are all
 * stored and the panel folds them when asked, because a store that collapsed on write
 * could never be un-collapsed and would silently lose how often something happened.
 */

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace SushiEngine
{
    namespace Editor
    {
        /** @brief How much a console line wants the reader's attention. */
        enum class LogLevel
        {
            Info,    /**< Something happened, as asked. */
            Warning, /**< It worked, but not the way the user probably intended. */
            Error    /**< It did not work. */
        };

        /** @brief One recorded console line. */
        struct ConsoleLine
        {
            LogLevel level = LogLevel::Info;
            std::string text;
            /** Editor uptime when the line was recorded, in seconds. */
            double time_seconds = 0.0;
        };

        /**
         * @brief The console's bounded backlog and the panel's view settings.
         *
         * The view settings live beside the lines because they are what the panel keeps
         * between frames, and there is exactly one console — separating them would put half
         * of one thing in two places.
         */
        struct Console
        {
            /** How many lines to keep; the oldest are dropped past this. */
            static constexpr std::size_t CAPACITY = 1000;

            std::vector<ConsoleLine> lines;

            /** Editor uptime, advanced by the main loop and stamped onto each line. */
            double uptime_seconds = 0.0;

            bool show_info = true;
            bool show_warnings = true;
            bool show_errors = true;
            bool collapse = true;     /**< Fold runs of identical consecutive lines. */
            bool show_timestamps = false;
            std::string filter;       /**< Case-insensitive substring the text must contain. */

            /**
             * @brief Records a line, trimming the oldest once past @ref CAPACITY.
             * @param level The line's severity.
             * @param text The message.
             */
            void append(LogLevel level, const std::string& text)
            {
                ConsoleLine line;
                line.level = level;
                line.text = text;
                line.time_seconds = uptime_seconds;
                lines.push_back(std::move(line));
                if (lines.size() > CAPACITY)
                {
                    lines.erase(lines.begin(),
                                lines.begin() +
                                    static_cast<std::ptrdiff_t>(lines.size() - CAPACITY));
                }
            }

            /** @brief How many recorded lines carry @p level. */
            std::size_t count_of(LogLevel level) const
            {
                std::size_t total = 0;
                for (const ConsoleLine& line : lines)
                    if (line.level == level)
                        ++total;
                return total;
            }
        };
    } // namespace Editor
} // namespace SushiEngine
