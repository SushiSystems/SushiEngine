/**************************************************************************/
/* simulation_settings.hpp                                                */
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
 * @file simulation_settings.hpp
 * @brief The host's simulation quality budgets — the sim-side sibling of RenderSettings.
 *
 * The render quality tier used to carry the atmosphere nest's grid as a stowaway:
 * selecting "Ultra" rendering silently rebuilt the weather simulation at a different
 * resolution, destroying the running weather — a rendering knob deciding a simulation
 * outcome. This file is the separation: each simulated domain owns its own quality
 * tier here, resolved by its own resolver, persisted per user (a grid resolution is a
 * machine budget, not scene content), and surfaced by the panel that owns the domain.
 *
 * Header-only and link-free on purpose, like `atmosphere_nest.hpp`: the editor
 * persists these, the renderer's probe maps `--tier` through the same table, and the
 * tests pin the table — none of them should need `sushi_sim` linked to do so.
 */

#include <SushiEngine/render/atmosphere_nest.hpp>

namespace SushiEngine
{
    namespace Simulation
    {
        /**
         * @brief The atmosphere simulation's quality tier, named like the render tiers.
         *
         * A separate enum rather than a reuse of @c Render::RenderQuality because the
         * two must be able to diverge: a machine can afford Ultra pixels and only a
         * Medium nest, or the reverse. The "Overall Quality" preset in the editor sets
         * both in one gesture; it never stores a combined value.
         */
        enum class AtmosphereQuality
        {
            Low,
            Medium,
            High,
            Ultra
        };

        /**
         * @brief The atmosphere simulation's authored budgets.
         *
         * Today only the tier; the QG core's step budget and similar knobs land here
         * as they become authored rather than compiled-in.
         */
        struct AtmosphereSimulationSettings
        {
            /**
             * @brief Resolves the regional nest's grid via @ref resolve_atmosphere_quality.
             *
             * Changing it rebuilds the nest, and a rebuilt nest starts from its base
             * state — the running weather is lost. That is why it is a tier the
             * Meteorology panel owns (with that warning at the control) and not a
             * side effect of the render tier.
             */
            AtmosphereQuality quality = AtmosphereQuality::High;
        };

        /**
         * @brief Every simulation-side quality budget, persisted per user.
         *
         * The sibling of @c Render::RenderSettings in the settings architecture: one
         * aggregate per domain authority, so no domain's tier can ride another's.
         */
        struct SimulationSettings
        {
            AtmosphereSimulationSettings atmosphere;
        };

        /**
         * @brief Resolves an atmosphere tier to the regional nest's discretization.
         *
         * The sim-side sibling of the renderer's `resolve_quality()`: pure, total, and
         * the only place the tier turns into a grid. The horizontal domain is 384 km at
         * every tier and only its resolution changes, so raising the tier resolves the
         * same weather more finely rather than simulating a different amount of world.
         * High is the shipped 192×192×48 at 2 km — the spacing at which convection
         * stops being parameterized and starts being resolved
         * (`docs/slop/atmosphere_system.md` §2.2), and the baseline every Phase B
         * measurement was taken against.
         *
         * @param quality The authored atmosphere tier.
         * @return The nest grid that tier resolves to.
         */
        inline Render::AtmosphereNestSize
        resolve_atmosphere_quality(AtmosphereQuality quality) noexcept
        {
            switch (quality)
            {
                case AtmosphereQuality::Low:
                    // The same 384 km of atmosphere at a sixth the cells. 4 km is well above
                    // the spacing at which convection resolves, so the floor's sky is smoother
                    // and its cumulus a parameterized haze rather than individual cells —
                    // which is the honest trade for a step this device can afford at all.
                    return {96u, 96u, 32u, 4000.0f, 18000.0f};
                case AtmosphereQuality::Medium:
                    // The same 384 km at 3 km and 40 levels — a third of High's cells. Above
                    // the 2 km at which convection resolves, so a cumulus field here is a
                    // smoother, more parameterized version of the same weather rather than a
                    // different one: the front is in the same place, drawn with less structure.
                    return {128u, 128u, 40u, 3000.0f, 18000.0f};
                case AtmosphereQuality::Ultra:
                    // The same 384 km at 1.5 km and 64 levels — two and a half times High's
                    // cells. Below 2 km the nest is resolving the individual thermal rather
                    // than the field of them, which is where the extra cost actually shows.
                    return {256u, 256u, 64u, 1500.0f, 18000.0f};
                case AtmosphereQuality::High:
                default:
                    // The baseline discretization, and the one every measurement in
                    // `docs/slop/atmosphere_system.md` §11's Phase B2c was taken against: 2 km
                    // is where convection stops being parameterized and starts being resolved
                    // (§2.2), which is what this tier's acceptance bar rests on.
                    return {192u, 192u, 48u, 2000.0f, 18000.0f};
            }
        }
    } // namespace Simulation
} // namespace SushiEngine
