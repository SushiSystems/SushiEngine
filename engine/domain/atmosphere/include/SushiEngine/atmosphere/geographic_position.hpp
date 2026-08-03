/**************************************************************************/
/* geographic_position.hpp                                                */
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
 * @file geographic_position.hpp
 * @brief Where a query is, for every tier of the atmosphere module.
 *
 * Its own header because T0 (`climatology.hpp`) and T1 (`quasigeostrophic_core.hpp`) both need
 * it and the core includes T0 — so leaving it where it was born, inside the core's header,
 * would have made the data tier depend on the model tier to say where a point is.
 */

namespace SushiEngine
{
    namespace Atmosphere
    {
        /**
         * @brief A point on the body, in radians.
         *
         * Spelled here rather than reused from `Simulation::GeodeticPosition` so that this
         * module keeps depending on nothing. The two are the same two numbers and the
         * simulation-side adapter converts in a line.
         */
        struct GeographicPosition
        {
            double latitude_radians = 0.0;  /**< [-pi/2, pi/2]. */
            double longitude_radians = 0.0; /**< East longitude; any range, wrapped on use. */
        };
    } // namespace Atmosphere
} // namespace SushiEngine
