/**************************************************************************/
/* launch_configuration.cpp                                               */
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

#include "launch_configuration.hpp"

#include <cstdlib>
#include <filesystem>
#include <string>

namespace SushiEngine
{
    namespace Player
    {
        LaunchConfiguration resolve_launch_configuration(int argument_count,
                                                         const char* const* arguments,
                                                         const std::string& default_manifest_path)
        {
            LaunchConfiguration configuration;

            // First pass: an explicit --manifest has to be found before anything else is
            // applied, since everything that follows is allowed to override what it loads.
            std::string manifest_path;
            for (int i = 1; i < argument_count; ++i)
                if (std::string(arguments[i]) == "--manifest" && i + 1 < argument_count)
                    manifest_path = arguments[++i];
            if (manifest_path.empty() && !default_manifest_path.empty() &&
                std::filesystem::exists(default_manifest_path))
                manifest_path = default_manifest_path;
            if (!manifest_path.empty())
                load_boot_manifest(manifest_path, configuration.manifest);

            // Second pass: CLI arguments layered on top of the manifest (or its defaults,
            // if none was found) — the developer's terminal always wins over the shipped
            // config, which is what makes local testing possible without editing the file.
            for (int i = 1; i < argument_count; ++i)
            {
                const std::string argument = arguments[i];
                if (argument == "--manifest")
                    ++i; // path already consumed above
                else if (argument == "--validation")
                    configuration.manifest.enable_validation = true;
                else if (argument == "--scene" && i + 1 < argument_count)
                    configuration.manifest.scene_path = arguments[++i];
                else if (argument == "--headless")
                    configuration.headless = true;
                else if (argument == "--frames" && i + 1 < argument_count)
                    configuration.frame_count = std::atoi(arguments[++i]);
                else if (argument.rfind("--", 0) != 0)
                    configuration.manifest.scene_path = argument; // a bare positional path
            }
            return configuration;
        }
    } // namespace Player
} // namespace SushiEngine
