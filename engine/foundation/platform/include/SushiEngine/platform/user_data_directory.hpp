/**************************************************************************/
/* user_data_directory.hpp                                                */
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

#ifndef SUSHIENGINE_PLATFORM_USER_DATA_DIRECTORY_HPP
#define SUSHIENGINE_PLATFORM_USER_DATA_DIRECTORY_HPP

#include <string>

namespace SushiEngine
{
    namespace Platform
    {
        /**
         * @brief The per-user, per-application directory a shipped host should write
         * its own state into (pipeline cache, save data, preferences).
         *
         * Wraps `SDL_GetPrefPath`, the one cross-platform primitive for this — the
         * OS-correct location (e.g. `%APPDATA%\<organization>\<application>\` on
         * Windows) rather than a path relative to the executable, which is wrong on
         * every platform where the install directory is not writable by the running
         * process (notarized macOS bundles, most of Linux packaging, and every
         * mobile/console sandbox this engine is meant to eventually reach).
         *
         * @param organization Publisher name; forms the outer path component.
         * @param application  Application name; forms the inner path component.
         * @return The directory path, created if it did not exist, with a trailing
         *     separator; empty if SDL could not determine or create it.
         */
        std::string user_data_directory(const char* organization, const char* application);
    } // namespace Platform
} // namespace SushiEngine

#endif
