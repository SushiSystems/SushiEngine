/**************************************************************************/
/* reference_frame.hpp                                                   */
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
 * @file reference_frame.hpp
 * @brief The active reference frame: which body an entity's coordinates are relative to,
 *        and how to move a state between one body's frame and another's.
 *
 * A free body's absolute truth is heliocentric (see @ref gravity.hpp), but nobody wants
 * to author or render a spacecraft at 1.5e11 m from the Sun. So each entity is expressed
 * relative to the body it is near — its @ref ReferenceFrame — and only re-expressed in a
 * new one when it crosses a sphere of influence (@ref active_frame_body). That
 * re-expression is a @ref rebase: a single double-precision coordinate change, the
 * orbital analogue of the floating-origin sector rebase, that leaves the physics
 * untouched because the field in @ref gravity.hpp is inertial and frame-independent.
 *
 * These frames are body-centred but keep inertial (ecliptic) axes: they translate, they
 * do not rotate. That is all the orbital regime needs. The body-*fixed* rotating frame
 * that surface play needs (pole plus spin, the generalisation of the Earth ENU/ECEF
 * construction in @ref ephemeris.hpp) is a separate, later concern layered on top of this.
 */

#include <SushiEngine/astro/celestial_bodies.hpp>
#include <SushiEngine/astro/gravity.hpp>
#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Astro
    {
        /**
         * @brief A body-centred, inertially-oriented frame at an instant.
         *
         * The origin rides the body (its ephemeris position and velocity at the epoch);
         * the axes stay parallel to the heliocentric ecliptic. A state expressed in this
         * frame is the heliocentric state minus the body's own state — a Galilean shift,
         * so accelerations are identical in both frames and no fictitious terms appear.
         */
        struct ReferenceFrame
        {
            BodyId body;                   /**< The body the frame is centred on. */
            WorldVector3 center;           /**< Body position, heliocentric-ecliptic metres. */
            WorldVector3 center_velocity;  /**< Body velocity, heliocentric-ecliptic m/s. */
        };

        /**
         * @brief Builds the frame centred on a body at a date.
         *
         * The body's position comes straight from the ephemeris; its velocity from a
         * symmetric finite difference of that position, which is ample for a coordinate
         * origin (the physics never differentiates it). The Sun's frame is the
         * heliocentric frame itself — a zero shift.
         *
         * @param body        The body to centre on.
         * @param julian_date The epoch of the frame.
         * @return The body-centred inertial frame.
         */
        inline ReferenceFrame frame_for(BodyId body, double julian_date) noexcept
        {
            ReferenceFrame frame;
            frame.body = body;
            frame.center = body_heliocentric_metres(body, julian_date);

            // Central difference over a small span for the origin's velocity, in seconds.
            constexpr double half_span_seconds = 60.0;
            const double half_span_days = half_span_seconds / SECONDS_PER_DAY;
            const WorldVector3 ahead =
                body_heliocentric_metres(body, julian_date + half_span_days);
            const WorldVector3 behind =
                body_heliocentric_metres(body, julian_date - half_span_days);
            const double inverse_span = 1.0 / (2.0 * half_span_seconds);
            frame.center_velocity =
                WorldVector3{(ahead.x - behind.x) * inverse_span,
                             (ahead.y - behind.y) * inverse_span,
                             (ahead.z - behind.z) * inverse_span};
            return frame;
        }

        /**
         * @brief Re-expresses a heliocentric state in a body-centred frame.
         * @param heliocentric The state in the heliocentric-ecliptic frame.
         * @param frame        The target body-centred frame.
         * @return The same physical state, coordinates relative to @p frame.
         */
        inline StateVector to_frame(const StateVector& heliocentric,
                                    const ReferenceFrame& frame) noexcept
        {
            return StateVector{
                WorldVector3{heliocentric.position.x - frame.center.x,
                             heliocentric.position.y - frame.center.y,
                             heliocentric.position.z - frame.center.z},
                WorldVector3{heliocentric.velocity.x - frame.center_velocity.x,
                             heliocentric.velocity.y - frame.center_velocity.y,
                             heliocentric.velocity.z - frame.center_velocity.z}};
        }

        /**
         * @brief Re-expresses a body-centred state back in the heliocentric frame.
         * @param local The state in @p frame's coordinates.
         * @param frame The body-centred frame @p local is expressed in.
         * @return The same physical state, heliocentric-ecliptic.
         */
        inline StateVector to_heliocentric(const StateVector& local,
                                           const ReferenceFrame& frame) noexcept
        {
            return StateVector{
                WorldVector3{local.position.x + frame.center.x,
                             local.position.y + frame.center.y,
                             local.position.z + frame.center.z},
                WorldVector3{local.velocity.x + frame.center_velocity.x,
                             local.velocity.y + frame.center_velocity.y,
                             local.velocity.z + frame.center_velocity.z}};
        }

        /**
         * @brief Moves a state from one body-centred frame to another.
         *
         * The coordinate change a sphere-of-influence crossing triggers: lift to
         * heliocentric, then drop into the new frame. Done once at the boundary in double
         * precision, it is exact to the ephemeris — the entity's absolute motion is
         * unchanged, only the numbers it is stored as.
         *
         * @param local     The state in @p from's coordinates.
         * @param from      The frame @p local is currently expressed in.
         * @param to        The frame to re-express it in.
         * @return The state in @p to's coordinates.
         */
        inline StateVector rebase(const StateVector& local, const ReferenceFrame& from,
                                  const ReferenceFrame& to) noexcept
        {
            return to_frame(to_heliocentric(local, from), to);
        }

        /**
         * @brief The body whose sphere of influence most tightly contains a point.
         *
         * Walks the catalogue for every SOI the point falls inside and keeps the smallest
         * — the most local dominant attractor, the frame an entity there should be
         * expressed in. The Sun is the fallback: its influence is the whole system, so a
         * point outside every planet's SOI belongs to the heliocentric frame.
         *
         * @param position    The point, heliocentric-ecliptic metres.
         * @param julian_date The date the spheres are placed at.
         * @return The body whose frame is active at @p position.
         */
        inline BodyId active_frame_body(const WorldVector3& position,
                                        double julian_date) noexcept
        {
            BodyId active = BodyId::Sun;
            double smallest_radius = 0.0;
            for (int index = 0; index < BODY_COUNT; ++index)
            {
                const BodyId body = static_cast<BodyId>(index);
                if (body == BodyId::Sun)
                    continue;
                const double soi = sphere_of_influence_radius(body);
                if (soi <= 0.0)
                    continue;
                const WorldVector3 center = body_heliocentric_metres(body, julian_date);
                const double dx = position.x - center.x;
                const double dy = position.y - center.y;
                const double dz = position.z - center.z;
                const double distance_squared = dx * dx + dy * dy + dz * dz;
                if (distance_squared > soi * soi)
                    continue;
                if (active == BodyId::Sun || soi < smallest_radius)
                {
                    active = body;
                    smallest_radius = soi;
                }
            }
            return active;
        }
    } // namespace Astro
} // namespace SushiEngine
