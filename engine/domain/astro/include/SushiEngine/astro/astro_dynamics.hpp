/**************************************************************************/
/* astro_dynamics.hpp                                                    */
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
 * @file astro_dynamics.hpp
 * @brief Advancing a free body's authority state: one symplectic step through an
 *        injected field, then a sphere-of-influence rebase — the two joined.
 *
 * A free entity's physical truth is a @ref StateVector expressed in the active
 * body-centred frame (@ref reference_frame.hpp), plus which body that frame is. This
 * header advances that pair: it lifts the state to the common heliocentric frame,
 * integrates it one step through an injected @ref IGravityField (so the field model is a
 * choice, not a hard-coded summation), then re-expresses the result in whichever body's
 * sphere of influence now contains it — rebasing exactly once at a crossing. The two were
 * previously separate primitives (@ref integrate_step and @ref rebase); this is the one
 * place that composes them into the per-step update the simulation drives.
 */

#include <SushiEngine/astro/gravity_field.hpp>
#include <SushiEngine/astro/reference_frame.hpp>
#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Astro
    {
        /**
         * @brief One velocity-Verlet step of a heliocentric state through an injected field.
         *
         * The field-parameterised counterpart to @ref integrate_step: identical
         * kick-drift-kick symplectic scheme, but the acceleration comes from @p field
         * rather than the hard-coded summation, so the caller chooses the model. The field
         * is re-sampled at the step's endpoint with the sources advanced to
         * @c julian_date + dt. Deterministic: same inputs, same bits.
         *
         * @param state         The heliocentric state to advance.
         * @param julian_date   The Julian Date at the start of the step.
         * @param delta_seconds Step length in seconds.
         * @param field         The gravitational field to integrate through.
         * @return The heliocentric state one step later.
         */
        inline StateVector integrate_step(const StateVector& state, double julian_date,
                                          double delta_seconds, const IGravityField& field) noexcept
        {
            const double half = 0.5 * delta_seconds;
            const WorldVector3 a0 = field.acceleration(state.position, julian_date);

            StateVector next;
            next.position = WorldVector3{
                state.position.x + state.velocity.x * delta_seconds + a0.x * half * delta_seconds,
                state.position.y + state.velocity.y * delta_seconds + a0.y * half * delta_seconds,
                state.position.z + state.velocity.z * delta_seconds + a0.z * half * delta_seconds};

            const double julian_date_next = julian_date + delta_seconds / SECONDS_PER_DAY;
            const WorldVector3 a1 = field.acceleration(next.position, julian_date_next);

            next.velocity = WorldVector3{state.velocity.x + (a0.x + a1.x) * half,
                                         state.velocity.y + (a0.y + a1.y) * half,
                                         state.velocity.z + (a0.z + a1.z) * half};
            return next;
        }

        /**
         * @brief Advances a body's authority state one step, rebasing on a SOI crossing.
         *
         * Lifts @p frame_local (the state in @p frame_body's body-centred frame) to
         * heliocentric, integrates it one step through @p field, then re-selects the active
         * frame from where the subject ended up (@ref active_frame_body) and expresses the
         * result there. When the subject stays inside the same sphere of influence the
         * frame is unchanged and the round trip is a pure Galilean shift; when it crosses
         * one, this is the single double-precision @ref rebase that boundary triggers, with
         * the absolute motion preserved exactly.
         *
         * @param frame_local   In/out: the state in @p frame_body's frame; overwritten with
         *                      the advanced state in the (possibly new) active frame.
         * @param frame_body    In/out: the body whose frame @p frame_local is in; updated
         *                      to the active frame after the step.
         * @param julian_date   The Julian Date at the start of the step.
         * @param delta_seconds Step length in seconds.
         * @param field         The gravitational field to integrate through.
         */
        inline void advance_astro_state(StateVector& frame_local, BodyId& frame_body,
                                        double julian_date, double delta_seconds,
                                        const IGravityField& field) noexcept
        {
            const ReferenceFrame frame = frame_for(frame_body, julian_date);
            const StateVector heliocentric = to_heliocentric(frame_local, frame);
            const StateVector next = integrate_step(heliocentric, julian_date, delta_seconds, field);

            const double julian_date_next = julian_date + delta_seconds / SECONDS_PER_DAY;
            const BodyId active = active_frame_body(next.position, julian_date_next);
            const ReferenceFrame next_frame = frame_for(active, julian_date_next);

            frame_local = to_frame(next, next_frame);
            frame_body = active;
        }
    } // namespace Astro
} // namespace SushiEngine
