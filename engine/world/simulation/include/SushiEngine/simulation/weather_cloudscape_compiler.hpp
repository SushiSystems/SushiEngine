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
 * A pure function of a `WeatherColumn`, owning no simulation policy of its own. Its output
 * is written to `Render::Environment::clouds` exactly where manual authoring already writes
 * it (see `RuntimeSimulation`), so nothing downstream branches on where the sky came from.
 *
 * **What it stopped being.** Until `docs/slop/atmosphere_system.md` §7.4 it was the sole
 * answer to "what is in the sky": one column, compiled into one deck stack, instantiated
 * everywhere. It is not that any more — when the published field classifies
 * (`Render::WeatherField::derives_genus`) the cloudscape bake resolves a genus and a
 * coverage per baked column and the deck stack no longer decides what a march sample finds.
 * What survives is everything that is genuinely a property of the whole sky rather than of
 * one point: the medium's scattering knobs, the erosion/weather scales, `evolution_rate`,
 * and the genus *label* the editor readout and a METAR-style report quote. That is why the
 * class is kept rather than deleted as §16's disposition table proposed — the label and the
 * medium still need a producer, and this is a truthful one.
 */

#include <algorithm>
#include <cmath>

#include <SushiEngine/environment/environment.hpp>
#include <SushiEngine/simulation/weather_types.hpp>

namespace SushiEngine
{
    namespace Simulation
    {
        /**
         * @brief Compiles a `WeatherColumn` into a ready-to-render `Render::Cloudscape`.
         *
         * Names one genus per `CloudLevel` through `Render::classify_cloud_genus` — the same
         * classifier the GPU bake resolves its per-column genus with — and fills
         * `CloudDeck::coverage_bias`/`density_scale` so the deck reproduces the column's
         * coverage/density on top of `cloud_genus_profile`'s baseline. A fourth deck slot is
         * reserved for towering convection (Cumulonimbus), enabled only when the low level is
         * both filled and strongly convective — the acceptance bar's "cumulus line" at a cold
         * front.
         */
        class WeatherCloudscapeCompiler
        {
            public:
                /**
                 * @brief Compiles @p column into a `Cloudscape`, over the authored medium.
                 *
                 * @p medium is carried through untouched apart from the decks and
                 * `evolution_rate`. It used to be discarded — the compiler returned a
                 * default-constructed `Cloudscape` — which was tolerable while the deck stack was
                 * the whole description of the sky and the editor disabled those sliders under
                 * procedural weather anyway. It is not tolerable now: since
                 * `docs/slop/atmosphere_system.md` §7.4 the decks no longer decide what a march
                 * sample finds, so the scattering knobs, the ground-shadow strength and the
                 * erosion scale are all the authored control over the look that is left, and
                 * resetting them every tick would make them uneditable in the one mode that
                 * matters.
                 *
                 * **`enabled` is carried through like everything else, and used not to be.** It was
                 * forced on, with the reasoning that a provider which has weather to report is
                 * reporting it, and that inheriting a scene authored with the cloudscape switched
                 * off would leave procedural weather silently invisible. That argument is about
                 * *loading* a scene, but the override it justified was level-triggered and ran
                 * every tick — and a per-tick override cannot tell "this scene was authored with
                 * clouds off" from "the author just switched clouds off half a second ago". It
                 * answered both the same way.
                 *
                 * While only `Procedural` installed a provider that stayed invisible: `Manual` ran
                 * no compiler, so the checkbox worked and nobody found it. WM-SEED made *both*
                 * modes install a provider, and the override immediately became a dead toggle in
                 * every mode — the user reported it as "clouds enabled kapatılamıyor". The
                 * silent-invisibility worry it was defending against does not survive contact with
                 * the panel either: `Clouds Enabled` sits directly above the weather-mode radios,
                 * so an author who cleared it is looking straight at the reason.
                 *
                 * The general rule this is an instance of: a compiler derives *what the sky
                 * contains*; whether the sky is drawn at all is the author's, and a derivation
                 * step must not overwrite a decision it has no way to read.
                 *
                 * @param column The layered-column state to render, from any `IWeatherProvider`.
                 * @param medium The current cloudscape, for everything that describes the whole
                 *               sky rather than one column.
                 * @return A `Cloudscape` ready to assign to `Render::Environment::clouds`.
                 */
                Render::Cloudscape compile(const WeatherColumn& column,
                                           const Render::Cloudscape& medium) const
                {
                    Render::Cloudscape clouds = medium;

                    const WeatherLevelState& low = column.levels[static_cast<int>(CloudLevel::Low)];
                    const WeatherLevelState& mid = column.levels[static_cast<int>(CloudLevel::Mid)];
                    const WeatherLevelState& high = column.levels[static_cast<int>(CloudLevel::High)];

                    assign_level(clouds.decks[0], low, Render::CloudBand::Low);
                    assign_level(clouds.decks[1], mid, Render::CloudBand::Middle);
                    assign_level(clouds.decks[2], high, Render::CloudBand::High);

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

                    if (Render::cloud_band_towers(low.coverage, low.convective_fraction))
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
                        clouds.decks[3] = Render::CloudDeck{};
                    }
                    // Cleared outright rather than merely disabled: a deck that keeps last
                    // frame's genus and coverage behind an `enabled` flag is stale state every
                    // reader has to know to distrust, and the memcmp that gates the cloudscape
                    // bake would see it change for no visible reason.
                    clouds.decks[4] = Render::CloudDeck{};
                    clouds.decks[5] = Render::CloudDeck{};

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
                // The genus choice itself lives in `Render::classify_cloud_genus`, not here.
                // It used to live here, as three hard-coded `if` chains -- exactly the OCP
                // finding docs/slop/atmosphere_system.md §1.6 records against this class. It
                // has to move because the cloudscape bake now resolves a genus per baked
                // column on the GPU (§7.4), and two copies of the same thresholds, one in C++
                // and one in GLSL, would eventually label a column as one genus and render it
                // as another.
                static void assign_level(Render::CloudDeck& deck, const WeatherLevelState& state,
                                         Render::CloudBand band) noexcept
                {
                    deck.enabled = state.coverage > Render::cloud_genus_thresholds().enable_coverage;
                    const Render::CloudGenus genus =
                        Render::classify_cloud_genus(band, state.coverage, state.convective_fraction);
                    deck.genus = genus;
                    const Render::CloudGenusProfile profile = Render::cloud_genus_profile(genus);
                    deck.coverage_bias = std::clamp(state.coverage - profile.coverage, -1.0f, 1.0f);
                    deck.density_scale = std::clamp(state.density_scale, 0.0f, 2.0f);
                }
        };
    } // namespace Simulation
} // namespace SushiEngine
