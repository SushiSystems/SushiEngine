/**************************************************************************/
/* launch_configuration.hpp                                               */
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

#ifndef SUSHIENGINE_PLAYER_LAUNCH_CONFIGURATION_HPP
#define SUSHIENGINE_PLAYER_LAUNCH_CONFIGURATION_HPP

/**
 * @file launch_configuration.hpp
 * @brief What a launch resolves to once the boot manifest and the command line are
 * folded together.
 *
 * The two sources answer different questions and are deliberately not merged into one
 * reader: the manifest (@ref boot_manifest.hpp) is what a shipped, double-clicked build
 * is configured by, and the command line is how a developer overrides that build
 * without editing its config. The precedence between them — the terminal always wins —
 * is the rule this header owns, kept out of `main()` so it is a function with a return
 * value rather than a shape only a launched process can be asked about.
 */

#include <string>

#include "boot_manifest.hpp"

namespace SushiEngine
{
    namespace Player
    {
        /**
         * @brief A resolved launch: the manifest's fields, plus the two run-mode knobs
         *        that have no manifest spelling.
         *
         * `headless` and `frame_count` are command-line only on purpose. They describe how
         * this particular run is driven rather than what the shipped build is, and a
         * `boot.json` that could turn a product's window off would be a way for a broken
         * install to look like a broken machine.
         */
        struct LaunchConfiguration
        {
            /** @brief The manifest's fields, after the command line has overridden them. */
            BootManifest manifest;
            /** @brief No window, no input, no present — the CI-shaped run (PLATFORM0 S6). */
            bool headless = false;
            /** @brief How many frames a headless run advances before it exits. */
            int frame_count = 60;
        };

        /**
         * @brief Resolves a launch from a boot manifest overlaid with command-line arguments.
         *
         * Two passes, because `--manifest` decides what the rest of the arguments override:
         * the first pass finds it (falling back to @p default_manifest_path when the file is
         * there), the second applies `--scene`, `--validation`, `--headless`, `--frames` and a
         * bare positional path on top of whatever it loaded. An unreadable, malformed or
         * non-object manifest leaves every field at its default rather than failing the
         * launch, matching @ref load_boot_manifest.
         *
         * @param argument_count Number of entries in @p arguments, including the program name.
         * @param arguments The `argv` array; entry 0 is skipped as the program name.
         * @param default_manifest_path Manifest to read when no `--manifest` was given and the
         *     file exists; pass an empty string to read no manifest by default.
         * @return The resolved configuration. Every field is populated, either from a manifest,
         *     from an argument, or from its compiled-in default.
         */
        LaunchConfiguration resolve_launch_configuration(int argument_count,
                                                         const char* const* arguments,
                                                         const std::string& default_manifest_path);
    } // namespace Player
} // namespace SushiEngine

#endif
