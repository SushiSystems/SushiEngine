/**************************************************************************/
/* boot_manifest.cpp                                                     */
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

            out.scene_path = json.value("scene_path", out.scene_path);
            out.window_title = json.value("window_title", out.window_title);
            out.width = json.value("width", out.width);
            out.height = json.value("height", out.height);
            out.organization = json.value("organization", out.organization);
            out.application = json.value("application", out.application);
            out.enable_validation = json.value("enable_validation", out.enable_validation);
            return true;
        }
    } // namespace Player
} // namespace SushiEngine
