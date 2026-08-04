/**************************************************************************/
/* climatology_asset.hpp                                                  */
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
 * @file climatology_asset.hpp
 * @brief Reads T0's baked climatology off disk (`docs/design/atmosphere_system.md` §4).
 *
 * **This file exists because `sushiengine_atmosphere` links nothing.**
 * `Atmosphere::Climatology` is a value that adopts bytes; it deliberately cannot open a file, so
 * that the same object serves the editor, a headless probe and a unit test without two of those
 * three growing a filesystem dependency. Somebody still has to do the reading, and this is the
 * smallest possible somebody: one function, on the simulation side, where a path is already an
 * ordinary thing to have.
 *
 * It is the only place in the engine that names the asset.
 */

#include <cstdint>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include <SushiEngine/atmosphere/climatology.hpp>

namespace SushiEngine
{
    namespace Simulation
    {
        /** @brief Where `se climatology bake` writes, relative to the working directory. */
        constexpr const char* CLIMATOLOGY_ASSET_PATH = "assets/atmosphere/climatology.set0";

        /**
         * @brief Loads the baked climatology, falling back to the analytic latitude bands.
         *
         * **A missing or unreadable asset is not an error, and this returns no error to check.**
         * §4 says T0 degrades to analytic bands for a body that is not Earth, and a scene on such
         * a body is not a scene with broken weather — so the fallback is a working mean state that
         * the core has always run on, not a failure mode. Ask the returned object's `baked()`
         * whether real data was found; that is a question about which climatology is in use, which
         * is worth asking, rather than a status code that invites a caller to abort a load over it.
         *
         * A blob that is present but *malformed* is refused by `adopt` rather than half-read, so
         * that case lands here too: analytic bands, and `baked()` false.
         *
         * @param path Asset to read; defaults to @ref CLIMATOLOGY_ASSET_PATH.
         * @return The baked climatology, or the analytic one.
         */
        inline Atmosphere::Climatology load_climatology(
            const std::string& path = CLIMATOLOGY_ASSET_PATH)
        {
            Atmosphere::Climatology climatology;

            std::ifstream file(path, std::ios::binary);
            if (!file)
                return climatology;

            const std::vector<std::uint8_t> blob((std::istreambuf_iterator<char>(file)),
                                                 std::istreambuf_iterator<char>());
            // The return value is dropped on purpose: `adopt` leaves the object analytic when it
            // refuses, which is exactly what this function would have returned anyway.
            (void)climatology.adopt(blob);
            return climatology;
        }
    } // namespace Simulation
} // namespace SushiEngine
