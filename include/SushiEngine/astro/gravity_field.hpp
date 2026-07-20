/**************************************************************************/
/* gravity_field.hpp                                                     */
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
 * @file gravity_field.hpp
 * @brief The gravitational field as an injectable interface, so the dynamics that flies
 *        a free body does not name a concrete field model.
 *
 * @ref gravity.hpp provides the summed on-rails field as a free function; this header
 * lifts it behind @ref IGravityField so a subject's integrator depends on the abstraction
 * (dependency inversion) rather than the concrete summation. The default
 * @ref SummedRailsGravityField is that summation; a patched-conic field (only the dominant
 * attractor), a full N-body field, or a test double can replace it without the integrator
 * changing (open for extension, closed for modification).
 */

#include <SushiEngine/astro/gravity.hpp>
#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Astro
    {
        /**
         * @brief A gravitational field sampled at a heliocentric point and instant.
         *
         * The seam the orbital integrator pulls acceleration from. Heliocentric-ecliptic
         * metres in, acceleration in metres/second^2 out, double precision — the boundary
         * rule the field is always evaluated at (see @ref gravity.hpp).
         */
        class IGravityField
        {
            public:
                virtual ~IGravityField() = default;

                /**
                 * @brief Gravitational acceleration at a heliocentric point.
                 * @param heliocentric_position Field point, heliocentric-ecliptic metres.
                 * @param julian_date           The date the source bodies are placed at.
                 * @return Acceleration, metres/second^2, heliocentric-ecliptic.
                 */
                virtual WorldVector3 acceleration(const WorldVector3& heliocentric_position,
                                                  double julian_date) const noexcept = 0;
        };

        /**
         * @brief The default field: every catalogue body on its analytic rail, summed.
         *
         * A thin adapter over @ref gravity_field — the "on-rails sources, integrated
         * subject" model. Stateless, so one instance serves every subject.
         */
        class SummedRailsGravityField final : public IGravityField
        {
            public:
                WorldVector3 acceleration(const WorldVector3& heliocentric_position,
                                          double julian_date) const noexcept override
                {
                    return gravity_field(heliocentric_position, julian_date);
                }
        };
    } // namespace Astro
} // namespace SushiEngine
