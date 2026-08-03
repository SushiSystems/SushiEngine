/**************************************************************************/
/* replay_json.hpp                                                       */
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
 * @file replay_json.hpp
 * @brief A JSON file format for recorded input, for games that want replay files on disk.
 *
 * The opt-in serialization for @ref InputRecorder — the only replay header that includes
 * nlohmann/json, so the in-memory recorder in @ref replay.hpp stays dependency-free. A record
 * is a flat array of events; reads are tolerant field-by-field so a truncated or older file loads
 * what it can rather than throwing.
 */

#include <vector>

#include <nlohmann/json.hpp>

#include <SushiEngine/input/events.hpp>
#include <SushiEngine/input/replay.hpp>

namespace SushiEngine
{
    namespace Input
    {
        /** @brief Serializes a list of @p events to a JSON array. */
        inline nlohmann::json events_to_json(const std::vector<InputEvent>& events)
        {
            nlohmann::json array = nlohmann::json::array();
            for (const InputEvent& event : events)
            {
                array.push_back(nlohmann::json{{"device", event.device},
                                               {"type", static_cast<std::uint16_t>(event.type)},
                                               {"control", event.control},
                                               {"value", event.value},
                                               {"x", event.x},
                                               {"y", event.y},
                                               {"frame", event.frame}});
            }
            return array;
        }

        /** @brief Serializes @p recorder's captured events to a JSON array. */
        inline nlohmann::json recording_to_json(const InputRecorder& recorder)
        {
            return events_to_json(recorder.events());
        }

        /**
         * @brief Reads a JSON array of events, tolerating missing fields and non-arrays.
         * @param document The JSON to read.
         * @return The parsed events (empty if @p document is not an array).
         */
        inline std::vector<InputEvent> events_from_json(const nlohmann::json& document)
        {
            std::vector<InputEvent> events;
            if (!document.is_array())
                return events;
            for (const nlohmann::json& node : document)
            {
                InputEvent event;
                event.device = node.value("device", DeviceId{INVALID_DEVICE});
                event.type = static_cast<EventType>(node.value("type", std::uint16_t{0}));
                event.control = node.value("control", std::uint16_t{0});
                event.value = node.value("value", 0.0f);
                event.x = node.value("x", 0.0f);
                event.y = node.value("y", 0.0f);
                event.frame = node.value("frame", std::uint64_t{0});
                events.push_back(event);
            }
            return events;
        }

        /** @brief Loads a JSON @p document of events into @p recorder. */
        inline void recording_from_json(InputRecorder& recorder, const nlohmann::json& document)
        {
            recorder.set_events(events_from_json(document));
        }
    } // namespace Input
} // namespace SushiEngine
