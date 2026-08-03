/**************************************************************************/
/* boot_manifest.cpp                                                      */
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

#include "boot_manifest.hpp"

#include <fstream>

#include <nlohmann/json.hpp>

namespace
{
    /**
     * @brief Reads one manifest field, leaving @p out alone when it is absent or mistyped.
     *
     * `nlohmann::json::value` covers the absent case but throws on a type mismatch, which
     * would abort the read half way through and hand back a partly-overwritten manifest —
     * the one outcome this file's tolerance rule forbids.
     *
     * @param json The manifest object being read.
     * @param name The field's key.
     * @param out Receives the value; untouched when the key is missing or holds another type.
     */
    template <typename Value>
    void read_field(const nlohmann::json& json, const char* name, Value& out)
    {
        const nlohmann::json::const_iterator found = json.find(name);
        if (found == json.end())
            return;
        try
        {
            out = found->get<Value>();
        }
        catch (const nlohmann::json::exception&)
        {
        }
    }
} // namespace

namespace SushiEngine
{
    namespace Player
    {
        bool load_boot_manifest(const std::string& path, BootManifest& out)
        {
            std::ifstream input(path);
            if (!input.is_open())
                return false;

            nlohmann::json json;
            input >> std::noskipws;
            try
            {
                json = nlohmann::json::parse(input, nullptr, false);
            }
            catch (const nlohmann::json::exception&)
            {
                return false;
            }
            if (json.is_discarded() || !json.is_object())
                return false;

            read_field(json, "scene_path", out.scene_path);
            read_field(json, "window_title", out.window_title);
            read_field(json, "width", out.width);
            read_field(json, "height", out.height);
            read_field(json, "organization", out.organization);
            read_field(json, "application", out.application);
            read_field(json, "enable_validation", out.enable_validation);
            return true;
        }
    } // namespace Player
} // namespace SushiEngine
