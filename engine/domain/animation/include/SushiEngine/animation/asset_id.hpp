/**************************************************************************/
/* asset_id.hpp                                                           */
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
 * @file asset_id.hpp
 * @brief The animation-asset handle type, factored out so the asset kinds and the database
 *        can name it without a circular include.
 */

#include <cstdint>

namespace SushiEngine
{
    namespace Animation
    {
        /** @brief A handle naming one loaded animation asset within a database. */
        using AssetId = std::uint32_t;

        /** @brief The id returned when an asset fails to load or is not found. */
        constexpr AssetId INVALID_ASSET = 0xFFFFFFFFu;
    } // namespace Animation
} // namespace SushiEngine
