/**************************************************************************/
/* weather_cloudscape_compiler.hpp                                       */
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
 * @file weather_cloudscape_compiler.hpp
 * @brief The sim-side bridge: any `IWeatherProvider`'s column state -> `Render::Cloudscape`.
 *
 * The class named in the task brief as "its own small, focused class (SRP)": a pure
 * function of a `WeatherColumn`, owning no simulation policy of its own — the sim-side
 * analogue of `Render::CloudscapeCompiler`'s "compiles column state into the GPU field
 * set, pure function of its input" description (design doc §3), one step earlier in
 * the pipeline. Its output is written to `Render::Environment::clouds` exactly where
 * manual authoring already writes it (see `RuntimeSimulation`), so `CloudscapeCompilePass`
 * (T3) and everything after it needs zero changes to consume procedurally-driven
 * weather — the entire point of introducing this seam.
 */

#include <algorithm>
#include <cmath>

#include <SushiEngine/render/environment.hpp>
#include <SushiEngine/sim/weather_types.hpp>

namespace SushiEngine
{
    namespace Simulation
    {
        /**
         * @brief Compiles a `WeatherColumn` into a ready-to-render `Render::Cloudscape`.
         *
         * Picks one genus per `CloudLevel` from the level's `convective_fraction`/
         * `coverage` (stratiform vs. cumuliform, thin vs. filled — mirrors the choices a
         * human author already makes in the Advanced deck editor) and fills
         * `CloudDeck::coverage_bias`/`density_scale` so the deck reproduces the column's
         * authored coverage/density on top of `cloud_genus_profile`'s baseline. A fourth
         * deck slot is reserved for towering convection (Cumulonimbus), enabled only when
         * the low level is both filled and strongly convective — the acceptance bar's
         * "cumulus line" at a cold front.
         */
        class WeatherCloudscapeCompiler
        {
            public:
                /**
                 * @brief Compiles @p column into a `Cloudscape`.
                 * @param column The layered-column state to render, from any `IWeatherProvider`.
                 * @return A `Cloudscape` ready to assign to `Render::Environment::clouds`.
                 */
                Render::Cloudscape compile(const WeatherColumn& column) const
                {
                    Render::Cloudscape clouds;
                    clouds.enabled = true;

                    const WeatherLevelState& low = column.levels[static_cast<int>(CloudLevel::Low)];
                    const WeatherLevelState& mid = column.levels[static_cast<int>(CloudLevel::Mid)];
                    const WeatherLevelState& high = column.levels[static_cast<int>(CloudLevel::High)];

                    assign_level(clouds.decks[0], low, pick_low_genus(low));
                    assign_level(clouds.decks[1], mid, pick_mid_genus(mid));
                    assign_level(clouds.decks[2], high, pick_high_genus(high));

                    // W5 world coupling (design doc §5.3): "cloud-base darkening from the rain
                    // channel", the old system's literal `density += density * weather.a`. Applied
                    // here -- before CloudscapeCompilePass's bake ever sees the deck -- rather than
                    // as a render-tier change, so the bake's own contract stays untouched; boosting
                    // the input density scale achieves the identical visual result. Only the low
                    // deck darkens: that is the band T2's moisture closure actually rains from (see
                    // regional_weather_grid.hpp's precipitation derivation), so a mid/high deck
                    // never darkens for a surface shower it takes no part in.
                    clouds.decks[0].density_scale =
                        std::clamp(clouds.decks[0].density_scale * (1.0f + column.precipitation),
                                  0.0f, 2.0f);

                    constexpr float CB_CONVECTIVE_THRESHOLD = 0.75f;
                    constexpr float CB_COVERAGE_THRESHOLD = 0.30f;
                    if (low.convective_fraction > CB_CONVECTIVE_THRESHOLD && low.coverage > CB_COVERAGE_THRESHOLD)
                    {
                        clouds.decks[3].enabled = true;
                        clouds.decks[3].genus = Render::CloudGenus::Cumulonimbus;
                        const Render::CloudGenusProfile profile =
                            Render::cloud_genus_profile(Render::CloudGenus::Cumulonimbus);
                        clouds.decks[3].coverage_bias = std::clamp(low.coverage - profile.coverage, -1.0f, 1.0f);
                        clouds.decks[3].density_scale =
                            std::clamp(low.density_scale * low.convective_fraction, 0.0f, 2.0f);
                    }
                    else
                    {
                        clouds.decks[3].enabled = false;
                    }
                    clouds.decks[4].enabled = false;
                    clouds.decks[5].enabled = false;

                    // The design doc's ask: "the dead evolution_rate uniform becomes the
                    // synoptic advance multiplier" (§5.1). W0 already wired the shader side to
                    // scroll the erosion detail sample by this value every frame
                    // (docs/slop/weather_and_clouds.md's W0 CHANGELOG entry); that consumption is
                    // unchanged here — only the *source* of the value changes, from an
                    // author-set constant to T1/T2's own activity, so weather visibly churns
                    // faster under a strong, windy, convective sky than a calm one.
                    const float wind_speed = std::sqrt(column.wind_u_mps * column.wind_u_mps +
                                                       column.wind_v_mps * column.wind_v_mps);
                    const float front_activity = std::max(low.convective_fraction, mid.convective_fraction);
                    clouds.evolution_rate = std::clamp(0.02f + wind_speed / 50.0f + front_activity * 0.3f, 0.0f, 1.0f);
                    return clouds;
                }

            private:
                static Render::CloudGenus pick_low_genus(const WeatherLevelState& state) noexcept
                {
                    if (state.convective_fraction > 0.5f)
                        return Render::CloudGenus::Cumulus;
                    if (state.convective_fraction > 0.2f)
                        return Render::CloudGenus::Stratocumulus;
                    return Render::CloudGenus::Stratus;
                }

                static Render::CloudGenus pick_mid_genus(const WeatherLevelState& state) noexcept
                {
                    if (state.convective_fraction > 0.5f)
                        return Render::CloudGenus::Altocumulus;
                    if (state.coverage > 0.7f)
                        return Render::CloudGenus::Nimbostratus;
                    return Render::CloudGenus::Altostratus;
                }

                static Render::CloudGenus pick_high_genus(const WeatherLevelState& state) noexcept
                {
                    return state.coverage > 0.6f ? Render::CloudGenus::Cirrostratus : Render::CloudGenus::Cirrus;
                }

                static void assign_level(Render::CloudDeck& deck, const WeatherLevelState& state,
                                         Render::CloudGenus genus) noexcept
                {
                    constexpr float ENABLE_COVERAGE_THRESHOLD = 0.05f;
                    deck.enabled = state.coverage > ENABLE_COVERAGE_THRESHOLD;
                    deck.genus = genus;
                    const Render::CloudGenusProfile profile = Render::cloud_genus_profile(genus);
                    deck.coverage_bias = std::clamp(state.coverage - profile.coverage, -1.0f, 1.0f);
                    deck.density_scale = std::clamp(state.density_scale, 0.0f, 2.0f);
                }
        };
    } // namespace Simulation
} // namespace SushiEngine
