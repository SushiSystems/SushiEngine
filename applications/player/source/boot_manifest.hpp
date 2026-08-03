/**************************************************************************/
/* boot_manifest.hpp                                                      */
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

#ifndef SUSHIENGINE_PLAYER_BOOT_MANIFEST_HPP
#define SUSHIENGINE_PLAYER_BOOT_MANIFEST_HPP

/**
 * @file boot_manifest.hpp
 * @brief The shipped-player launch config: a `boot.json` beside the executable.
 *
 * A CLI flag is fine for a developer running `se player -- --scene x.sushiscene` from
 * a terminal; it is not how a shipped double-click-to-play build tells the player
 * which scene to open — there is no terminal to pass one to. This is the same problem
 * `engine/world/authoring`'s `JSONPreferencesStore` solves for editor session
 * state, read the same tolerant way: a missing or corrupt file degrades field-by-field
 * to defaults rather than refusing to start (`applications/player/source/main.cpp` applies CLI
 * arguments on top of whatever this loads, so a developer can always override it
 * locally without editing the file).
 */

#include <cstdint>
#include <string>

namespace SushiEngine
{
    namespace Player
    {
        /** @brief What a `boot.json` may specify; the fields mean what
         *         `PlayerApp::Description`'s do. */
        struct BootManifest
        {
            std::string scene_path;
            std::string window_title = "SushiEngine Player";
            std::uint32_t width = 1280;
            std::uint32_t height = 720;
            std::string organization = "SushiSystems";
            std::string application = "SushiEnginePlayer";
            bool enable_validation = false;
        };

        /**
         * @brief Reads a `boot.json`-shaped file, leaving unset/unreadable fields at default.
         *
         * @param path Filesystem path to the manifest.
         * @param out  Receives the parsed fields; a field the document omits, or spells with
         *     a value of another type, keeps its prior value — so a caller may pre-seed it
         *     with non-default values before calling, and one bad field costs only itself.
         * @return Whether the file was opened and parsed as a JSON object. False (file
         *     missing, unreadable, not valid JSON, or not an object) leaves @p out exactly
         *     as it was passed in — never partially applied.
         */
        bool load_boot_manifest(const std::string& path, BootManifest& out);
    } // namespace Player
} // namespace SushiEngine

#endif
