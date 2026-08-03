/**************************************************************************/
/* gravity.hpp                                                           */
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
 * @file gravity.hpp
 * @brief The gravitational field of the solar system, and the integrator that flies
 *        a free body through it.
 *
 * The design is "on-rails sources, integrated subjects": the catalogue bodies stay on
 * their analytic Keplerian rails (they are the gravity *sources*, never the subjects),
 * and a free entity is a @ref StateVector propagated through the summed field they
 * produce. Keeping the planets analytic is what makes the field a pure, deterministic
 * function of position and time — the property @ref SushiLoop needs for same-binary
 * lockstep — and what keeps long orbits from drifting the way a fully dynamic N-body
 * solar system would.
 *
 * Everything here is heliocentric-ecliptic, metres and metres/second, double precision.
 * That is the boundary rule: the inverse-square field must never be evaluated in the
 * simulation's optional single-precision solve — @c |r|^3 over ~1e11 m collapses in
 * float. The field is sampled in double at the seam and handed down as a small local
 * acceleration.
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
        /** @brief Seconds in one day, the bridge between dynamics time and Julian Date. */
        constexpr double SECONDS_PER_DAY = 86400.0;

        /**
         * @brief Standard gravitational parameter mu = G*M of a body, m^3/s^2.
         *
         * Tabulated GM values (IAU/DE440 nominal) rather than a mass times a shared G:
         * mu is what the field needs and is known to far more digits than either factor
         * alone. The Sun dominates by six orders of magnitude; the small bodies still
         * matter close in, where they own the local field inside their sphere of
         * influence.
         *
         * @param body Which body's gravitational parameter to fetch.
         * @return mu in m^3/s^2; zero for a body the model does not attract with.
         */
        inline double standard_gravitational_parameter(BodyId body) noexcept
        {
            switch (body)
            {
                case BodyId::Sun:     return 1.32712440018e20;
                case BodyId::Mercury: return 2.2032e13;
                case BodyId::Venus:   return 3.24859e14;
                case BodyId::Earth:   return 3.986004418e14;
                case BodyId::Moon:    return 4.9048695e12;
                case BodyId::Mars:    return 4.282837e13;
                case BodyId::Jupiter: return 1.26686534e17;
                case BodyId::Saturn:  return 3.7931187e16;
                case BodyId::Uranus:  return 5.793939e15;
                case BodyId::Neptune: return 6.836529e15;
                case BodyId::Pluto:   return 8.71e11;
                default:              return 0.0;
            }
        }

        /**
         * @brief The body a given one orbits — its gravitational parent.
         * @param body Which body to place in the hierarchy.
         * @return The Moon's parent is Earth; every other body orbits the Sun.
         */
        inline BodyId gravitational_parent(BodyId body) noexcept
        {
            return body == BodyId::Moon ? BodyId::Earth : BodyId::Sun;
        }

        /**
         * @brief Heliocentric-ecliptic position of any catalogue body, in metres.
         *
         * The field needs every source in one frame; this is the metre-scale,
         * heliocentric counterpart to @ref planet_heliocentric_au that also resolves the
         * Sun (origin) and the Moon (Earth's position plus its geocentric offset).
         *
         * @param body        Which body to place.
         * @param julian_date The Julian Date.
         * @return Heliocentric-ecliptic position, metres.
         */
        inline WorldVector3 body_heliocentric_metres(BodyId body, double julian_date) noexcept
        {
            Vector3 helio_au;
            if (body == BodyId::Sun)
            {
                helio_au = Vector3{0.0, 0.0, 0.0};
            }
            else if (body == BodyId::Moon)
            {
                helio_au = planet_heliocentric_au(BodyId::Earth, julian_date) +
                           moon_geocentric_ecliptic_au(julian_date);
            }
            else
            {
                helio_au = planet_heliocentric_au(body, julian_date);
            }
            return WorldVector3{helio_au.x * METRES_PER_ASTRONOMICAL_UNIT,
                                helio_au.y * METRES_PER_ASTRONOMICAL_UNIT,
                                helio_au.z * METRES_PER_ASTRONOMICAL_UNIT};
        }

        /**
         * @brief Radius of a body's sphere of influence, metres.
         *
         * The Laplace SOI, @c a * (m_body / m_parent)^(2/5), inside which the body's pull
         * dominates its parent's — the boundary the active-frame selector switches on.
         * Mass ratios are read straight off the gravitational parameters (G cancels); the
         * orbital radius @c a is the body's own semi-major axis (the Moon's fixed mean
         * distance, the planets' Keplerian @c a).
         *
         * @param body Which body's sphere of influence to size.
         * @return SOI radius in metres; zero for the Sun (its influence is the whole system).
         */
        inline double sphere_of_influence_radius(BodyId body) noexcept
        {
            if (body == BodyId::Sun)
                return 0.0;
            const BodyId parent = gravitational_parent(body);
            const double mu_body = standard_gravitational_parameter(body);
            const double mu_parent = standard_gravitational_parameter(parent);
            if (mu_parent <= 0.0)
                return 0.0;
            const double semi_major_metres =
                body == BodyId::Moon
                    ? 3.844e8
                    : keplerian_elements_for(body).semi_major_axis_au *
                          METRES_PER_ASTRONOMICAL_UNIT;
            return semi_major_metres * std::pow(mu_body / mu_parent, 0.4);
        }

        /**
         * @brief The phase-space state of a free body: where it is and how fast it moves.
         *
         * Heliocentric-ecliptic, metres and metres/second, double precision — the
         * absolute truth an entity's floating-origin @c Transform is derived from, never
         * the reverse.
         */
        struct StateVector
        {
            WorldVector3 position; /**< Heliocentric-ecliptic position, metres. */
            WorldVector3 velocity; /**< Heliocentric-ecliptic velocity, metres/second. */
        };

        /**
         * @brief Newtonian pull of a single body on a subject offset from its centre.
         *
         * The one-body term g = -mu * d / |d|^3, with @p offset_from_centre the vector
         * from the body's centre to the subject. Frame-agnostic: because it depends only
         * on the relative vector, it returns the acceleration in whatever inertial frame
         * @p offset_from_centre is expressed in — which is what lets the near-surface
         * regime evaluate the dominant body's pull directly in the scene frame, without a
         * heliocentric round-trip. The self-term at the body's own centre is guarded.
         *
         * @param body             The attracting body.
         * @param offset_from_centre Vector from the body's centre to the subject, metres.
         * @return Acceleration toward the body, metres/second^2, in @p offset's frame.
         */
        inline WorldVector3 body_point_gravity(BodyId body,
                                               const WorldVector3& offset_from_centre) noexcept
        {
            const double mu = standard_gravitational_parameter(body);
            const double distance_squared = offset_from_centre.x * offset_from_centre.x +
                                            offset_from_centre.y * offset_from_centre.y +
                                            offset_from_centre.z * offset_from_centre.z;
            if (mu <= 0.0 || distance_squared <= 0.0)
                return WorldVector3{0.0, 0.0, 0.0};
            const double distance = std::sqrt(distance_squared);
            const double scale = -mu / (distance_squared * distance);
            return WorldVector3{scale * offset_from_centre.x, scale * offset_from_centre.y,
                                scale * offset_from_centre.z};
        }

        /**
         * @brief Gravitational acceleration at a point, summed over every source body.
         *
         * The Newtonian field g(r) = sum over bodies of the per-body @ref body_point_gravity,
         * with each source at its analytic ephemeris position for the given date. The
         * self-term at a source's own centre is guarded against; in practice a subject is
         * never exactly at a centre.
         *
         * @param position    Field point, heliocentric-ecliptic metres.
         * @param julian_date The Julian Date the source bodies are placed at.
         * @return Acceleration, metres/second^2, heliocentric-ecliptic.
         */
        inline WorldVector3 gravity_field(const WorldVector3& position,
                                          double julian_date) noexcept
        {
            WorldVector3 acceleration{0.0, 0.0, 0.0};
            for (int index = 0; index < BODY_COUNT; ++index)
            {
                const BodyId body = static_cast<BodyId>(index);
                const WorldVector3 source = body_heliocentric_metres(body, julian_date);
                const WorldVector3 pull = body_point_gravity(
                    body, WorldVector3{position.x - source.x, position.y - source.y,
                                       position.z - source.z});
                acceleration.x += pull.x;
                acceleration.y += pull.y;
                acceleration.z += pull.z;
            }
            return acceleration;
        }

        /**
         * @brief Advances a state by one fixed step under the summed field.
         *
         * Velocity Verlet (leapfrog in kick-drift-kick form): a symplectic second-order
         * integrator that conserves orbital energy over long runs where explicit Euler
         * would spiral. The field is re-sampled at the step's endpoint with the source
         * bodies advanced to @c julian_date + dt, so the moving sources are accounted
         * for. The step is deterministic: same inputs, same bits.
         *
         * @param state          The state to advance.
         * @param julian_date    The Julian Date at the start of the step.
         * @param delta_seconds  Step length in seconds (dynamics time).
         * @return The state one step later.
         */
        inline StateVector integrate_step(const StateVector& state, double julian_date,
                                          double delta_seconds) noexcept
        {
            const double half = 0.5 * delta_seconds;
            const WorldVector3 a0 = gravity_field(state.position, julian_date);

            StateVector next;
            next.position = WorldVector3{
                state.position.x + state.velocity.x * delta_seconds + a0.x * half * delta_seconds,
                state.position.y + state.velocity.y * delta_seconds + a0.y * half * delta_seconds,
                state.position.z + state.velocity.z * delta_seconds + a0.z * half * delta_seconds};

            const double julian_date_next = julian_date + delta_seconds / SECONDS_PER_DAY;
            const WorldVector3 a1 = gravity_field(next.position, julian_date_next);

            next.velocity = WorldVector3{state.velocity.x + (a0.x + a1.x) * half,
                                         state.velocity.y + (a0.y + a1.y) * half,
                                         state.velocity.z + (a0.z + a1.z) * half};
            return next;
        }
    } // namespace Astro
} // namespace SushiEngine
