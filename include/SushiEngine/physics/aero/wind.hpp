/**************************************************************************/
/* wind.hpp                                                               */
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
 * @file wind.hpp
 * @brief §11.6's arithmetic: what moving air does to a body that is not moving with it.
 *
 * The seam itself is `sim/`'s — §4.5: *"`sim/` continues to hand the physics a
 * `GravitySampler` and gains a `WindSampler` alongside it, backed by the atmosphere
 * system's wind field."* The physics never names the meteorology. What lives here is the
 * one force law both ends of that seam agree on, so a flag on a pole in a storm and a
 * car's high-speed lift come from the same three lines rather than from two.
 *
 * ### Why this is a *difference* and not a drag force
 *
 * `RigidBodyT::drag_coefficient` already exists and `predict` already spends it, once per
 * substep, as `-k|v|v` — still-air drag on the body's own velocity. What a wind field
 * changes is not *that* there is drag but *what the drag is measured against*: the real
 * force is `-k|v - w|(v - w)`. Adding a second drag term on top would double-count, and
 * replacing the first is not available, because `predict` runs on the device inside one
 * composition (§6.6) and cannot call a host sampler at all.
 *
 * So @ref wind_drag_acceleration returns exactly the difference between the drag the body
 * is really in and the still-air drag `predict` is going to apply anyway. Folded into
 * `RigidBodyT::external_acceleration`, the two together are the right force. With no wind
 * it is exactly zero, which is the property that makes it safe to apply unconditionally.
 *
 * It is evaluated once per tick, on the body's velocity at the tick boundary, for
 * `GravitySampler`'s stated reason: there is no point inside the substep loop at which a
 * host sampler could be called. A body travels at most the substep schedule's motion
 * budget in a tick, over which a field smooth enough to be worth sampling has not
 * meaningfully changed.
 */

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/core/rigid_body.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /** @brief Air density at sea level, in kilograms per cubic metre. */
        template <typename T>
        inline constexpr T sea_level_air_density = T(1.225);

        /**
         * @brief The quadratic drag constant a body of this mass and frontal area carries.
         *
         * `k` such that the drag acceleration is `-k|v|v`, which is what
         * `RigidBodyT::drag_coefficient` means. Derived rather than authored because the
         * numbers an author actually has are the shape's drag coefficient and its area —
         * a car is quoted as "0.30 Cd over 2.2 m²" and never as an acceleration constant.
         *
         * @tparam T The scalar element type.
         * @param drag_coefficient The shape's dimensionless drag coefficient.
         * @param area             Reference area, in square metres.
         * @param mass             The body's mass, in kilograms.
         * @param air_density      Air density, in kilograms per cubic metre.
         * @return The constant, or zero when any input makes it meaningless.
         */
        template <typename T>
        inline T quadratic_drag_constant(T drag_coefficient, T area, T mass,
                                         T air_density = sea_level_air_density<T>) noexcept
        {
            if (!(drag_coefficient > T(0)) || !(area > T(0)) || !(mass > T(0)) ||
                !(air_density > T(0)))
            {
                return T(0);
            }
            return T(0.5) * air_density * drag_coefficient * area / mass;
        }

        /**
         * @brief What a wind field adds to a body, over and above still-air drag.
         *
         * @tparam T The scalar element type.
         * @param body The body, read for its velocity and its drag constant.
         * @param wind The air's own velocity at the body, in metres per second.
         * @return An acceleration to fold into `RigidBodyT::external_acceleration`; zero
         *         for a body with no drag, and zero in still air whatever its drag.
         */
        template <typename T>
        inline Vector3T<T> wind_drag_acceleration(const RigidBodyT<T>& body,
                                                  const Vector3T<T>& wind) noexcept
        {
            if (!(body.drag_coefficient > T(0)))
                return Vector3T<T>{T(0), T(0), T(0)};

            const Vector3T<T> relative = body.velocity - wind;
            const T relative_speed = length(relative);
            const T still_speed = length(body.velocity);
            const Vector3T<T> real = relative * (-body.drag_coefficient * relative_speed);
            const Vector3T<T> assumed = body.velocity * (-body.drag_coefficient * still_speed);
            return real - assumed;
        }

        /**
         * @brief The aerodynamic force on a lifting surface, along its own two axes.
         *
         * A car's downforce, a wing's lift, and the drag that comes with them: both scale
         * with the square of airspeed and with the same reference area, so they are one
         * evaluation and not two. Returned as a force rather than an acceleration because
         * it is applied at a *centre of pressure* — an aerodynamic force has a lever and a
         * body's own centre of mass is not it, which is why downforce changes a car's
         * balance and not merely its grip.
         *
         * @tparam T The scalar element type.
         * @param airspeed          Speed through the air, in metres per second.
         * @param area              Reference area, in square metres.
         * @param force_coefficient The dimensionless coefficient for the axis wanted.
         * @param air_density       Air density, in kilograms per cubic metre.
         * @return The force magnitude, in newtons; never negative.
         */
        template <typename T>
        inline T aerodynamic_force(T airspeed, T area, T force_coefficient,
                                   T air_density = sea_level_air_density<T>) noexcept
        {
            if (!(area > T(0)) || !(force_coefficient > T(0)) || !(air_density > T(0)))
                return T(0);
            return T(0.5) * air_density * force_coefficient * area * airspeed * airspeed;
        }
    } // namespace Physics
} // namespace SushiEngine
