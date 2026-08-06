/**************************************************************************/
/* import_settings_io.hpp                                                 */
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
 * @file import_settings_io.hpp
 * @brief Reading and writing an asset's `.meta` sidecar.
 *
 * JSON, matching the encoding `CookBakeState` already uses for the project's cooking defaults,
 * so the one-way migration off that document is a field-for-field move rather than a format
 * change.
 */

#include <string>

#include <SushiEngine/model/import_settings.hpp>

namespace SushiEngine
{
    namespace Model
    {
        /**
         * @brief The sidecar path for an asset.
         *
         * Appends `.meta` to the whole path, extension included, so `Car.gltf` and `Car.glb`
         * in one directory keep separate settings.
         *
         * @param asset_path Path to the asset itself.
         * @return The sidecar's path.
         */
        std::string model_import_settings_path(const std::string& asset_path);

        /**
         * @brief Reads an asset's settings, or the defaults when it has none.
         *
         * @param asset_path Path to the asset, not to the sidecar.
         * @param out        Receives the settings; set to the defaults on any failure.
         * @return False when a sidecar exists but could not be read or parsed. A missing
         *         sidecar is not a failure: an asset that has never been configured uses the
         *         defaults, and that is the normal case rather than an error.
         */
        bool load_model_import_settings(const std::string& asset_path, ModelImportSettings& out);

        /**
         * @brief Writes an asset's settings to its sidecar, replacing what was there.
         *
         * @param asset_path Path to the asset, not to the sidecar.
         * @param settings   What to write.
         * @return False when the sidecar could not be opened or written.
         */
        bool save_model_import_settings(const std::string& asset_path,
                                        const ModelImportSettings& settings);
    } // namespace Model
} // namespace SushiEngine
