/**************************************************************************/
/* body_orientation.hpp                                                  */
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
 * @file body_orientation.hpp
 * @brief How a body is turned in space over time: its spin about its pole, and the
 *        equatorial frame that spin happens in — for every body, not just Earth.
 *
 * The topocentric sky in @ref ephemeris.hpp was written around Earth: positions were
 * rotated into Earth's equatorial frame and the observer's meridian was Earth's sidereal
 * time. This header generalises both so an observer standing on any body sees its own sky
 * turn with its own day. @ref body_rotation_angle is the IAU prime-meridian angle W(t)
 * (the body's spin); @ref ecliptic_to_body_equatorial rotates an ecliptic direction into
 * the body's own equatorial frame (the plane its spin axis is normal to).
 *
 * Earth is kept bit-identical to the original path (its equatorial conversion is the
 * fixed-obliquity one, and its meridian is driven by sidereal time in the ephemeris), so
 * the home sky does not shift; every other body picks up its true pole and spin.
 */

#include <cmath>

#include <SushiEngine/astro/celestial_bodies.hpp>
#include <SushiEngine/astro/julian_date.hpp>
#include <SushiEngine/astro/orbital_elements.hpp>
#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Astro
    {
        /**
         * @brief The IAU prime-meridian angle W of a body at a Julian Date, radians.
         *
         * W = W0 + Wdot * d with d the days since J2000: the rotation of the body's prime
         * meridian about its pole, the body's own analogue of sidereal time. Retrograde
         * rotators (Venus, Uranus, Pluto) carry a negative rate. Earth's value is defined
         * for completeness but the ephemeris drives Earth's meridian from sidereal time
         * instead, so the home sky is unchanged.
         *
         * @param body        Which body's spin to evaluate.
         * @param julian_date The Julian Date.
         * @return The prime-meridian angle, radians, wrapped to [0, 2pi).
         */
        inline double body_rotation_angle(BodyId body, double julian_date) noexcept
        {
            const double d = julian_date - J2000_JULIAN_DATE;
            double w0_degrees;
            double wdot_degrees_per_day;
            switch (body)
            {
                case BodyId::Sun:     w0_degrees = 84.176;  wdot_degrees_per_day = 14.1844000;   break;
                case BodyId::Mercury: w0_degrees = 329.5988; wdot_degrees_per_day = 6.1385108;   break;
                case BodyId::Venus:   w0_degrees = 160.20;  wdot_degrees_per_day = -1.4813688;   break;
                case BodyId::Earth:   w0_degrees = 190.147; wdot_degrees_per_day = 360.9856235;  break;
                case BodyId::Moon:    w0_degrees = 38.3213; wdot_degrees_per_day = 13.17635815;  break;
                case BodyId::Mars:    w0_degrees = 176.630; wdot_degrees_per_day = 350.89198226; break;
                case BodyId::Jupiter: w0_degrees = 284.95;  wdot_degrees_per_day = 870.5360000;  break;
                case BodyId::Saturn:  w0_degrees = 38.90;   wdot_degrees_per_day = 810.7939024;  break;
                case BodyId::Uranus:  w0_degrees = 203.81;  wdot_degrees_per_day = -501.1600928; break;
                case BodyId::Neptune: w0_degrees = 253.18;  wdot_degrees_per_day = 536.3128492;  break;
                case BodyId::Pluto:   w0_degrees = 302.695; wdot_degrees_per_day = 56.3625225;   break;
                default:              w0_degrees = 0.0;     wdot_degrees_per_day = 0.0;           break;
            }
            return wrap_degrees(w0_degrees + wdot_degrees_per_day * d) * DEGREES_TO_RADIANS;
        }

        /**
         * @brief The body's north pole direction expressed in the J2000 ecliptic frame.
         * @param body Which body's pole to fetch.
         * @return Unit pole direction, ecliptic coordinates.
         */
        inline Vector3 body_pole_ecliptic(BodyId body) noexcept
        {
            return normalize(equatorial_to_ecliptic(body_north_pole_equatorial(body)));
        }

        /**
         * @brief Rotates an ecliptic direction into a body's own equatorial frame.
         *
         * The body's equatorial frame has +Z along its north pole and +X along the
         * ascending node of its equator on the ecliptic (the pole tilted into the ecliptic
         * gives the node by construction). Projecting onto that frame is the generalisation
         * of @ref ecliptic_to_equatorial from Earth to any body. Earth is routed through
         * the exact fixed-obliquity conversion so the home sky is bit-identical.
         *
         * @param body     The body whose equatorial frame to project into.
         * @param ecliptic A direction in the ecliptic frame (need not be unit).
         * @return The same direction in the body's equatorial coordinates.
         */
        inline Vector3 ecliptic_to_body_equatorial(BodyId body, const Vector3& ecliptic) noexcept
        {
            if (body == BodyId::Earth)
                return ecliptic_to_equatorial(ecliptic);

            const Vector3 z = body_pole_ecliptic(body);
            const Vector3 ecliptic_pole{0.0, 0.0, 1.0};
            Vector3 x = cross(ecliptic_pole, z);
            const double x_length = length(x);
            x = x_length > 1e-9 ? x * (1.0 / x_length) : Vector3{1.0, 0.0, 0.0};
            const Vector3 y = cross(z, x);
            return Vector3{dot(ecliptic, x), dot(ecliptic, y), dot(ecliptic, z)};
        }

        /**
         * @brief Re-expresses a body-equatorial direction back in the ecliptic frame.
         *
         * The exact inverse of @ref ecliptic_to_body_equatorial: it rebuilds the same
         * body-equatorial axes (Earth through the fixed-obliquity conversion, every other
         * body from its pole) and expands the components back onto them, so a round trip
         * through the two is the identity. Needed by any consumer that stores state in a
         * body's frame yet must return it to the common ecliptic frame — the scene-frame
         * bijection and the reference-frame rebasing among them.
         *
         * @param body        The body whose equatorial frame @p body_equatorial is in.
         * @param body_equatorial A direction in @p body's equatorial coordinates.
         * @return The same direction in the ecliptic frame.
         */
        inline Vector3 body_equatorial_to_ecliptic(BodyId body,
                                                   const Vector3& body_equatorial) noexcept
        {
            if (body == BodyId::Earth)
                return equatorial_to_ecliptic(body_equatorial);

            const Vector3 z = body_pole_ecliptic(body);
            const Vector3 ecliptic_pole{0.0, 0.0, 1.0};
            Vector3 x = cross(ecliptic_pole, z);
            const double x_length = length(x);
            x = x_length > 1e-9 ? x * (1.0 / x_length) : Vector3{1.0, 0.0, 0.0};
            const Vector3 y = cross(z, x);
            return Vector3{x.x * body_equatorial.x + y.x * body_equatorial.y + z.x * body_equatorial.z,
                           x.y * body_equatorial.x + y.y * body_equatorial.y + z.y * body_equatorial.z,
                           x.z * body_equatorial.x + y.z * body_equatorial.y + z.z * body_equatorial.z};
        }

        /**
         * @brief Re-expresses a J2000 equatorial direction in a body's equatorial frame.
         *
         * The bridge for quantities catalogued in Earth's equatorial frame — the fixed
         * stars, each body's pole — when the observer stands on another body: lift to the
         * ecliptic, then drop into the observer body's equatorial frame. Identity when the
         * observer is on Earth.
         *
         * @param observer_body The body the observer stands on.
         * @param equatorial    A direction in the J2000 (Earth) equatorial frame.
         * @return The same direction in @p observer_body's equatorial coordinates.
         */
        inline Vector3 equatorial_to_body_equatorial(BodyId observer_body,
                                                     const Vector3& equatorial) noexcept
        {
            if (observer_body == BodyId::Earth)
                return equatorial;
            return ecliptic_to_body_equatorial(observer_body, equatorial_to_ecliptic(equatorial));
        }
    } // namespace Astro
} // namespace SushiEngine
