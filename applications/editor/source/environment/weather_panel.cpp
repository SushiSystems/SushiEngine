/**************************************************************************/
/* weather_panel.cpp                                                     */
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

#include "weather_panel.hpp"

#include "../ui/panel_widgets.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <vector>

#include <imgui.h>

#include <SushiEngine/astro/celestial_bodies.hpp>
#include <SushiEngine/environment/environment.hpp>

namespace SushiEngine
{
    namespace Editor
    {
        using SushiEngine::Simulation::IWorldEditor;

        void draw_environment_panel(EditorContext& context)
        {
            if (!context.panels.environment)
                return;
            if (!ImGui::Begin("Environment", &context.panels.environment))
            {
                ImGui::End();
                return;
            }

            IWorldEditor* world = world_of(context);
            if (world == nullptr)
            {
                ImGui::TextUnformatted("No scene open.");
                ImGui::End();
                return;
            }

            SushiEngine::Render::Environment environment = world->environment();
            bool changed = false;

            if (ImGui::CollapsingHeader("Sun", ImGuiTreeNodeFlags_DefaultOpen))
            {
                // Author the sun as azimuth/elevation, derived from and written back into
                // its unit direction so the panel holds no state of its own.
                const SushiEngine::Vector3 dir =
                    SushiEngine::normalize(environment.sun.direction);
                float elevation = std::asin(static_cast<float>(
                    dir.y < -1.0 ? -1.0 : (dir.y > 1.0 ? 1.0 : dir.y))) * 57.29578f;
                float azimuth = std::atan2(static_cast<float>(dir.z),
                                           static_cast<float>(dir.x)) * 57.29578f;
                bool sun_moved = false;
                // The astronomical sun (Solar System section) overrides this manual
                // direction downstream, so disable the sliders while it is on.
                ImGui::BeginDisabled(context.sky_astronomical_sun);
                if (ImGui::SliderFloat("Elevation", &elevation, -10.0f, 90.0f, "%.1f deg"))
                    sun_moved = true;
                if (ImGui::SliderFloat("Azimuth", &azimuth, -180.0f, 180.0f, "%.1f deg"))
                    sun_moved = true;
                ImGui::EndDisabled();
                if (sun_moved)
                {
                    const float e = elevation / 57.29578f;
                    const float a = azimuth / 57.29578f;
                    environment.sun.direction = SushiEngine::Vector3{
                        std::cos(e) * std::cos(a), std::sin(e), std::cos(e) * std::sin(a)};
                    changed = true;
                }

                float sun_color[3] = {static_cast<float>(environment.sun.color.x),
                                      static_cast<float>(environment.sun.color.y),
                                      static_cast<float>(environment.sun.color.z)};
                if (ImGui::ColorEdit3("Sun Color", sun_color))
                {
                    environment.sun.color =
                        SushiEngine::Vector3{sun_color[0], sun_color[1], sun_color[2]};
                    changed = true;
                }
                if (ImGui::SliderFloat("Sun Intensity", &environment.sun.intensity, 0.0f, 40.0f))
                    changed = true;
            }

            if (ImGui::CollapsingHeader("Atmosphere", ImGuiTreeNodeFlags_DefaultOpen))
            {
                if (ImGui::Checkbox("Atmosphere Enabled", &environment.atmosphere.enabled))
                    changed = true;
                // Exposure has one owner: the Post Process panel, where the whole exposure
                // chain (mode, compensation, and this scene multiplier) is authored
                // together. A second slider here is how the two used to fight.
                ImGui::TextDisabled("Exposure is authored in the Post Process panel.");
                ImGui::SameLine();
                if (ImGui::SmallButton("Open##exposure_owner"))
                {
                    context.panels.post_process = true;
                    ImGui::SetWindowFocus("Post Process");
                }
                float height_km = environment.atmosphere.height * 0.001f;
                if (ImGui::SliderFloat("Height", &height_km, 10.0f, 300.0f, "%.0f km"))
                {
                    environment.atmosphere.height = height_km * 1000.0f;
                    changed = true;
                }
                if (ImGui::SliderFloat("Mie Anisotropy", &environment.atmosphere.mie_anisotropy,
                                       0.0f, 0.99f))
                    changed = true;
            }

            if (ImGui::CollapsingHeader("Fog"))
            {
                if (ImGui::Checkbox("Fog Enabled", &environment.fog.enabled))
                    changed = true;
                if (ImGui::SliderFloat("Density", &environment.fog.density, 0.0f, 0.1f,
                                       "%.4f /m"))
                    changed = true;
                float fog_falloff_km = environment.fog.height_falloff * 1000.0f;
                if (ImGui::SliderFloat("Height Falloff", &fog_falloff_km, 0.0f, 5.0f,
                                       "%.3f /km"))
                {
                    environment.fog.height_falloff = fog_falloff_km * 0.001f;
                    changed = true;
                }
                float fog_color[3] = {
                    static_cast<float>(environment.fog.scattering_color.x),
                    static_cast<float>(environment.fog.scattering_color.y),
                    static_cast<float>(environment.fog.scattering_color.z)};
                if (ImGui::ColorEdit3("Fog Color", fog_color))
                {
                    environment.fog.scattering_color =
                        SushiEngine::Vector3{fog_color[0], fog_color[1], fog_color[2]};
                    changed = true;
                }
                if (ImGui::SliderFloat("Ambient Fill", &environment.fog.ambient, 0.0f, 1.0f))
                    changed = true;
                if (ImGui::SliderFloat("Sun Anisotropy", &environment.fog.phase_anisotropy,
                                       0.0f, 0.95f))
                    changed = true;

                ImGui::SeparatorText("Local Fog Volumes");
                if (environment.fog_volume_count < SushiEngine::Render::MAX_FOG_VOLUMES &&
                    ImGui::Button("Add Volume"))
                {
                    environment.fog_volumes[environment.fog_volume_count] =
                        SushiEngine::Render::FogVolume{};
                    ++environment.fog_volume_count;
                    changed = true;
                }
                for (int i = 0; i < environment.fog_volume_count; ++i)
                {
                    ImGui::PushID(i);
                    SushiEngine::Render::FogVolume& v = environment.fog_volumes[i];
                    bool open = ImGui::TreeNode("volume", "Volume %d", i);
                    ImGui::SameLine();
                    if (ImGui::SmallButton("X"))
                    {
                        for (int j = i; j + 1 < environment.fog_volume_count; ++j)
                            environment.fog_volumes[j] = environment.fog_volumes[j + 1];
                        --environment.fog_volume_count;
                        changed = true;
                        if (open)
                            ImGui::TreePop();
                        ImGui::PopID();
                        break;
                    }
                    if (open)
                    {
                        int shape = static_cast<int>(v.shape);
                        if (ImGui::Combo("Shape", &shape, "Box\0Ellipsoid\0"))
                        {
                            v.shape = static_cast<SushiEngine::Render::FogVolumeShape>(shape);
                            changed = true;
                        }
                        double center[3] = {v.center.x, v.center.y, v.center.z};
                        if (ImGui::InputScalarN("Center", ImGuiDataType_Double, center, 3))
                        {
                            v.center = SushiEngine::WorldVector3{center[0], center[1], center[2]};
                            changed = true;
                        }
                        float extent[3] = {static_cast<float>(v.extent.x),
                                           static_cast<float>(v.extent.y),
                                           static_cast<float>(v.extent.z)};
                        if (ImGui::DragFloat3("Extent", extent, 5.0f, 1.0f, 100000.0f, "%.0f m"))
                        {
                            v.extent = SushiEngine::Vector3{extent[0], extent[1], extent[2]};
                            changed = true;
                        }
                        float col[3] = {static_cast<float>(v.color.x),
                                        static_cast<float>(v.color.y),
                                        static_cast<float>(v.color.z)};
                        if (ImGui::ColorEdit3("Color", col))
                        {
                            v.color = SushiEngine::Vector3{col[0], col[1], col[2]};
                            changed = true;
                        }
                        if (ImGui::SliderFloat("Density", &v.density, 0.0f, 0.2f, "%.4f /m"))
                            changed = true;
                        if (ImGui::SliderFloat("Edge Falloff", &v.edge_falloff, 0.0f, 0.99f))
                            changed = true;
                        ImGui::TreePop();
                    }
                    ImGui::PopID();
                }
            }

            if (ImGui::CollapsingHeader("Global Illumination"))
            {
                if (ImGui::Checkbox("Probe GI Enabled", &environment.gi.enabled))
                    changed = true;
                ImGui::TextDisabled("Requires High or Ultra tier.");
                if (ImGui::SliderFloat("GI Intensity", &environment.gi.intensity, 0.0f, 4.0f))
                    changed = true;
                if (ImGui::SliderFloat("Normal Bias", &environment.gi.normal_bias, 0.0f, 2.0f,
                                       "%.2f m"))
                    changed = true;
            }

            if (ImGui::CollapsingHeader("Surface"))
            {
                float ground[3] = {static_cast<float>(environment.surface.ground_albedo.x),
                                   static_cast<float>(environment.surface.ground_albedo.y),
                                   static_cast<float>(environment.surface.ground_albedo.z)};
                if (ImGui::ColorEdit3("Ground Albedo", ground))
                {
                    environment.surface.ground_albedo =
                        SushiEngine::Vector3{ground[0], ground[1], ground[2]};
                    changed = true;
                }
                float ocean[3] = {static_cast<float>(environment.surface.ocean_color.x),
                                  static_cast<float>(environment.surface.ocean_color.y),
                                  static_cast<float>(environment.surface.ocean_color.z)};
                if (ImGui::ColorEdit3("Ocean Color", ocean))
                {
                    environment.surface.ocean_color =
                        SushiEngine::Vector3{ocean[0], ocean[1], ocean[2]};
                    changed = true;
                }
            }

            if (ImGui::CollapsingHeader("Clouds"))
            {
                if (ImGui::Checkbox("Clouds Enabled", &environment.clouds.enabled))
                    changed = true;
                ImGui::BeginDisabled(!environment.clouds.enabled);

                // Weather panel v2 (docs/slop/weather_and_clouds.md §6/§7 W4, and WM-SEED in
                // docs/slop/atmosphere_system.md). Both modes install a provider now and the
                // choice is about *where the sky comes from*: Manual places it from a seed,
                // Procedural grows it in a dynamical core. Manual used to mean no provider at
                // all, which is why an authored deck stack was applied to a whole planet.
                using SushiEngine::Simulation::WeatherMode;
                WeatherMode mode = world->weather_mode();
                bool procedural = mode == WeatherMode::Procedural;
                ImGui::SeparatorText("Weather Mode");
                if (ImGui::RadioButton("Manual (seed)", !procedural))
                {
                    world->set_weather_mode(WeatherMode::Manual);
                    procedural = false;
                }
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip(
                        "Manual: a seed places a dozen pressure systems on a latitude "
                        "climatology, over the whole planet. Somewhere is stormy, somewhere is "
                        "completely clear, and the same seed always gives the same sky.\n\n"
                        "Nothing evolves — that is what a chosen sky means.");
                ImGui::SameLine();
                if (ImGui::RadioButton("Procedural (T1+T2)", procedural))
                {
                    world->set_weather_mode(WeatherMode::Procedural);
                    procedural = true;
                }
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip(
                        "Procedural: a global dynamical core and a regional grid drive the sky; "
                        "presets seed a starting scenario rather than setting deck parameters "
                        "directly, and the sky then evolves on its own.");

                const SushiEngine::Simulation::GeodeticPosition weather_observer{
                    environment.observer.latitude_radians, environment.observer.longitude_radians};

                // The seed replaces the preset row in Manual mode, which is the whole of
                // WM-SEED's user-facing change: a preset set one global sky, and one global sky
                // is precisely what a planet does not have.
                if (!procedural)
                {
                    ImGui::SeparatorText("Seed");
                    // Shown and edited as a signed int because ImGui has no unsigned 64-bit
                    // scalar. Lossy above 2^31, and acceptable: this control exists for a
                    // number an author types or steps, and a scene carrying a wider seed from
                    // elsewhere keeps it until somebody edits *this field*, which is the point
                    // at which they have chosen a new one anyway.
                    int seed = static_cast<int>(world->weather_seed());
                    if (ImGui::InputInt("Weather Seed", &seed))
                        world->set_weather_seed(static_cast<std::uint64_t>(seed));
                    ImGui::SameLine();
                    if (ImGui::Button("Reroll"))
                        world->set_weather_seed(world->weather_seed() + 1);
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip(
                            "Re-places every pressure system. The weather where you are standing "
                            "may not change much — most of the planet is not here.");
                }
                else
                {
                    // In Procedural mode the same buttons inject an anomaly upstream in the
                    // global flow (design doc §6): the sky is not snapped to a fixed look, it
                    // evolves from that disturbance over the following minutes, and what arrives
                    // is whatever the disturbance grows into.
                    ImGui::SeparatorText("Preset");
                    for (int p = 0; p < SushiEngine::Render::WEATHER_PRESET_COUNT; ++p)
                    {
                        SushiEngine::Render::WeatherPreset preset =
                            static_cast<SushiEngine::Render::WeatherPreset>(p);
                        if (p > 0)
                            ImGui::SameLine();
                        if (ImGui::Button(SushiEngine::Render::weather_preset_name(preset)) &&
                            world->weather_authoring() != nullptr)
                            world->weather_authoring()->apply_preset(preset, weather_observer);
                    }
                }

                if (procedural && world->weather_authoring() != nullptr &&
                    world->weather_provider() != nullptr)
                {
                    SushiEngine::Simulation::IWeatherAuthoring& authoring =
                        *world->weather_authoring();
                    const SushiEngine::Simulation::IWeatherProvider& weather =
                        *world->weather_provider();

                    // Time-scrub coupled to the ephemeris clock (design doc §6): only the
                    // time-of-day fraction of the master epoch, not the calendar date -- the
                    // scoped-down half of "time-scrub", named rather than silent (a full date
                    // picker is out of this phase's reach).
                    ImGui::SeparatorText("Time of Day");
                    double day_integer = 0.0;
                    const double day_fraction = std::modf(environment.observer.julian_date + 0.5, &day_integer);
                    float hours = static_cast<float>(day_fraction * 24.0);
                    if (ImGui::SliderFloat("Time of Day", &hours, 0.0f, 24.0f, "%.1f h"))
                    {
                        environment.observer.julian_date = day_integer - 0.5 + double(hours) / 24.0;
                        changed = true;
                    }

                    // ---- The mean state the weather is a departure from (T0, §4) ----------
                    //
                    // **Which climatology is running is otherwise invisible.** A scene on the
                    // analytic latitude bands when somebody meant it to be on the baked asset
                    // looks completely normal -- there is still a jet, there are still cyclones
                    // -- right up until the jet turns out to be in the wrong place and the
                    // seasons do not exist. That is a failure that survives inspection, so the
                    // panel states the source rather than leaving it to be inferred.
                    //
                    // Read-only, deliberately. Nothing here is authorable: T0 is *data*, baked
                    // offline by `se climatology bake` from sources whose attribution rides
                    // inside the asset. A slider over a climatology would be a slider over a
                    // measurement.
                    if (const SushiEngine::Atmosphere::Climatology* climatology =
                            weather.climatology())
                    {
                        ImGui::SeparatorText("Mean State (T0)");
                        if (climatology->baked())
                            ImGui::TextColored(ImVec4(0.55f, 0.85f, 0.55f, 1.0f),
                                               "Baked climatology");
                        else
                            ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.3f, 1.0f),
                                               "Analytic latitude bands (no asset loaded)");
                        if (ImGui::IsItemHovered())
                            ImGui::SetTooltip(
                                "The analytic bands are a working mean state, not an error:\n"
                                "they are what a body that is not Earth uses, and what every\n"
                                "measurement before T0 existed was taken against. But they\n"
                                "have no continents and no season.\n\n"
                                "Expected the real one? Run: se climatology bake");

                        // Where the season currently is, read from the scene's own epoch by the
                        // same conversion the provider feeds the core. The core additionally
                        // quantizes to whole days, so the profile it is actually built for can
                        // sit up to a day behind this -- far below the resolution of a monthly
                        // climatology, and not worth another accessor on the interface to say.
                        const double year_fraction =
                            SushiEngine::Simulation::year_fraction_from_julian_date(
                                environment.observer.julian_date);
                        const int month_index =
                            std::min(11, std::max(0, int(year_fraction * 12.0)));
                        static const char* const MONTHS[] = {
                            "January", "February", "March", "April", "May", "June", "July",
                            "August", "September", "October", "November", "December"};
                        ImGui::Text("Season: %s (year fraction %.3f)", MONTHS[month_index],
                                    year_fraction);

                        // The two numbers that decide whether this mean state makes weather at
                        // all: where the jet is, and how much vertical shear it carries. Phillips'
                        // criterion puts the critical shear near beta * L_d^2, about 8 m/s here,
                        // so a shear below that is a mean state that will never produce a storm
                        // however long it is left running -- worth seeing before waiting.
                        double peak_shear = 0.0;
                        double shear_latitude = 0.0;
                        double peak_jet = 0.0;
                        double jet_latitude = 0.0;
                        for (int degree = -89; degree <= 89; ++degree)
                        {
                            const double latitude = double(degree) * 3.14159265358979323846 / 180.0;
                            const double upper =
                                climatology->upper_zonal_wind_mps(latitude, year_fraction);
                            const double shear =
                                upper - climatology->lower_zonal_wind_mps(latitude, year_fraction);
                            if (std::fabs(shear) > std::fabs(peak_shear))
                            {
                                peak_shear = shear;
                                shear_latitude = double(degree);
                            }
                            if (upper > peak_jet)
                            {
                                peak_jet = upper;
                                jet_latitude = double(degree);
                            }
                        }
                        ImGui::Text("Jet: %.1f m/s at %+.0f deg", peak_jet, jet_latitude);
                        ImGui::Text("Shear: %.1f m/s at %+.0f deg", peak_shear, shear_latitude);
                        if (std::fabs(peak_shear) < 8.0)
                            ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.3f, 1.0f),
                                               "Below the Phillips threshold: this mean state "
                                               "will not grow storms.");

                        // Attribution travels *inside* the asset rather than in a sidecar that
                        // could be separated from the data it describes, so this is the licence
                        // notice as well as the debugging aid.
                        if (climatology->baked() && !climatology->provenance().empty() &&
                            ImGui::TreeNode("Provenance"))
                        {
                            ImGui::PushTextWrapPos(0.0f);
                            ImGui::TextUnformatted(climatology->provenance().c_str());
                            ImGui::PopTextWrapPos();
                            ImGui::TreePop();
                        }
                    }

                    // Synoptic map overlay (design doc §6), rebuilt on the global core.
                    //
                    // **This used to draw a list and now it samples a field**, which is the whole
                    // Phase C swap made visible in one panel. There is no longer a `systems[]` to
                    // iterate: a low is a closed contour in a pressure field, not an ellipse with
                    // a heading, so the map asks `pressure_anomaly_hpa` (or
                    // `frontal_strength_at`) at every cell of a lattice and shades what comes
                    // back. Everything the old panel offered as a slider — depth, radius, speed,
                    // heading — was authored data the core now *derives*, and a slider that
                    // pretends to set a derived quantity is worse than no slider.
                    //
                    // What replaces the sliders is a click: the author disturbs the flow and the
                    // dynamics take it from there. Strictly less direct control, and strictly
                    // more honest, because the previous control was over a picture rather than
                    // over the weather.
                    ImGui::SeparatorText("Synoptic Map");

                    // Two fields worth looking at, sharing one lattice. Pressure is what an
                    // author navigates by; the thermal gradient is what actually decides whether
                    // there is weather at a point, and having it on the same map is what lets
                    // "the front arrives" be watched rather than assumed.
                    WeatherMapState& map = context.panel_state.weather_map;
                    int& map_field = map.field;
                    ImGui::RadioButton("Pressure", &map_field, 0);
                    ImGui::SameLine();
                    ImGui::RadioButton("Fronts", &map_field, 1);
                    ImGui::SameLine();
                    ImGui::TextDisabled(map_field == 0 ? "(hPa anomaly)" : "(K / 100 km)");

                    float& map_span_degrees = map.span_degrees;
                    ImGui::SliderFloat("Map Span", &map_span_degrees, 10.0f, 180.0f, "%.0f deg");

                    constexpr float MAP_SIZE_PX = 220.0f;
                    constexpr int MAP_CELLS = WeatherMapState::CELLS;
                    constexpr double DEG = 3.14159265358979323846 / 180.0;
                    constexpr double HALF_PI = 3.14159265358979323846 * 0.5;
                    const double map_half_span_rad = double(map_span_degrees) * 0.5 * DEG;

                    ImGui::InvisibleButton("##synoptic_map", ImVec2(MAP_SIZE_PX, MAP_SIZE_PX));
                    const bool map_clicked = ImGui::IsItemClicked(ImGuiMouseButton_Left);
                    ImDrawList* map_draw_list = ImGui::GetWindowDrawList();
                    const ImVec2 map_origin = ImGui::GetItemRectMin();
                    const ImVec2 map_center(map_origin.x + MAP_SIZE_PX * 0.5f,
                                            map_origin.y + MAP_SIZE_PX * 0.5f);

                    // Map pixel -> geodetic. A plate-carree window centred on the observer:
                    // longitude wraps freely (the core wraps it again itself), latitude clamps at
                    // the poles because there is nothing past them to sample.
                    const double cell_span_rad = 2.0 * map_half_span_rad / double(MAP_CELLS);
                    auto map_position_at = [&](double u, double v) {
                        const double lat = weather_observer.latitude_radians +
                                           (0.5 - v) * 2.0 * map_half_span_rad;
                        const double lon = weather_observer.longitude_radians +
                                           (u - 0.5) * 2.0 * map_half_span_rad;
                        return SushiEngine::Simulation::GeodeticPosition{
                            lat > HALF_PI ? HALF_PI : (lat < -HALF_PI ? -HALF_PI : lat), lon};
                    };

                    // One pass to sample, one to draw, because the colour scale is normalized
                    // against the extremum actually present rather than a fixed range: a quiet
                    // flow and a deep cyclone both read clearly, and the legend below reports
                    // the number the scale was normalized to so the picture stays quantitative.
                    map.samples.resize(std::size_t(MAP_CELLS) * std::size_t(MAP_CELLS));
                    std::vector<float>& map_samples = map.samples;
                    float map_extremum = 0.0f;
                    for (int cy = 0; cy < MAP_CELLS; ++cy)
                    {
                        for (int cx = 0; cx < MAP_CELLS; ++cx)
                        {
                            const SushiEngine::Simulation::GeodeticPosition p = map_position_at(
                                (double(cx) + 0.5) / double(MAP_CELLS),
                                (double(cy) + 0.5) / double(MAP_CELLS));
                            const double value = map_field == 0 ? weather.pressure_anomaly_hpa(p)
                                                                : weather.frontal_strength_at(p);
                            const float sample = static_cast<float>(value);
                            map_samples[cy * MAP_CELLS + cx] = sample;
                            const float magnitude = sample < 0.0f ? -sample : sample;
                            if (magnitude > map_extremum)
                                map_extremum = magnitude;
                        }
                    }
                    const float map_scale = map_extremum > 1e-4f ? 1.0f / map_extremum : 0.0f;

                    const float cell_px = MAP_SIZE_PX / float(MAP_CELLS);
                    for (int cy = 0; cy < MAP_CELLS; ++cy)
                    {
                        for (int cx = 0; cx < MAP_CELLS; ++cx)
                        {
                            const float t = map_samples[cy * MAP_CELLS + cx] * map_scale;
                            // Pressure is signed and gets a diverging scale (blue trough / orange
                            // ridge, the convention every surface chart uses). The frontal
                            // gradient is a magnitude and gets a sequential one -- shading its
                            // sign would be inventing a distinction it does not have.
                            int r = 20, g = 26, b = 36;
                            if (map_field == 0)
                            {
                                const float m = t < 0.0f ? -t : t;
                                if (t < 0.0f)
                                {
                                    r = 20 + int(30.0f * m);
                                    g = 26 + int(120.0f * m);
                                    b = 36 + int(200.0f * m);
                                }
                                else
                                {
                                    r = 20 + int(225.0f * m);
                                    g = 26 + int(120.0f * m);
                                    b = 36 + int(20.0f * m);
                                }
                            }
                            else
                            {
                                const float m = t < 0.0f ? -t : t;
                                r = 20 + int(200.0f * m);
                                g = 26 + int(210.0f * m);
                                b = 36 + int(90.0f * m);
                            }
                            const ImVec2 lo(map_origin.x + float(cx) * cell_px,
                                            map_origin.y + float(cy) * cell_px);
                            map_draw_list->AddRectFilled(
                                lo, ImVec2(lo.x + cell_px + 1.0f, lo.y + cell_px + 1.0f),
                                IM_COL32(r, g, b, 255));
                        }
                    }

                    // Wind arrows on a much coarser lattice, low in the column (0.25) because
                    // that is the wind an author is placing weather relative to. They are what
                    // turns a shaded blob into a circulation: the rotation around a low, and the
                    // direction the whole pattern is steered, are both only legible as flow.
                    constexpr int WIND_CELLS = 9;
                    for (int wy = 0; wy < WIND_CELLS; ++wy)
                    {
                        for (int wx = 0; wx < WIND_CELLS; ++wx)
                        {
                            const double u = (double(wx) + 0.5) / double(WIND_CELLS);
                            const double v = (double(wy) + 0.5) / double(WIND_CELLS);
                            const SushiEngine::Simulation::WindSample wind =
                                weather.wind_at(map_position_at(u, v), 0.25);
                            const double speed = std::sqrt(wind.eastward_mps * wind.eastward_mps +
                                                           wind.northward_mps * wind.northward_mps);
                            if (speed < 0.5)
                                continue;
                            // Fixed arrow length, so the arrows read as direction and the shading
                            // reads as magnitude -- two signals, one each, rather than both
                            // competing for the same channel.
                            constexpr double ARROW_PX = 9.0;
                            const float ax = map_origin.x + float(u) * MAP_SIZE_PX;
                            const float ay = map_origin.y + float(v) * MAP_SIZE_PX;
                            const float dx = static_cast<float>(wind.eastward_mps / speed * ARROW_PX);
                            const float dy = static_cast<float>(-wind.northward_mps / speed * ARROW_PX);
                            const ImU32 arrow = IM_COL32(230, 235, 245, 170);
                            map_draw_list->AddLine(ImVec2(ax - dx, ay - dy), ImVec2(ax + dx, ay + dy),
                                                   arrow, 1.0f);
                            map_draw_list->AddCircleFilled(ImVec2(ax + dx, ay + dy), 1.6f, arrow);
                        }
                    }

                    map_draw_list->AddRect(
                        map_origin, ImVec2(map_origin.x + MAP_SIZE_PX, map_origin.y + MAP_SIZE_PX),
                        IM_COL32(90, 100, 120, 255));
                    map_draw_list->AddCircleFilled(map_center, 3.5f, IM_COL32(255, 255, 255, 255));
                    map_draw_list->AddCircle(map_center, 6.0f, IM_COL32(255, 255, 255, 160), 12, 1.5f);

                    ImGui::TextDisabled(map_field == 0
                                            ? "Blue = trough, orange = ridge; arrows are the "
                                              "low-level wind."
                                            : "Brighter = sharper thermal gradient; arrows are "
                                              "the low-level wind.");
                    ImGui::Text(map_field == 0 ? "Peak anomaly: %.1f hPa"
                                               : "Peak gradient: %.2f K/100km",
                                double(map_extremum));

                    // Placing weather: click the map. The parameters below are the two a
                    // dynamical core can actually honour -- how big the disturbance is and how
                    // hard it spins. Everything else about the resulting low (how deep it gets,
                    // where it goes, whether it grows a front at all) is the core's answer, not
                    // the author's setting, which is exactly the trade this phase makes.
                    ImGui::SeparatorText("Inject Anomaly");
                    float& inject_radius_km = map.inject_radius_km;
                    // 12 m/s of peak rotational wind, which lands near -26 hPa at the default
                    // radius -- an ordinary deep low rather than a record one. It used to be 20,
                    // which was chosen before the injection carried compensating circulation and
                    // reached -70.
                    float& inject_amplitude_mps = map.inject_amplitude_mps;
                    int& inject_sign = map.inject_sign;
                    ImGui::RadioButton("Low", &inject_sign, 0);
                    ImGui::SameLine();
                    ImGui::RadioButton("High", &inject_sign, 1);
                    ImGui::SliderFloat("Radius", &inject_radius_km, 200.0f, 2000.0f, "%.0f km");
                    // Ceiling lowered from 45: past about 30 m/s at these radii the anomaly is
                    // deeper than any recorded mid-latitude cyclone, so the top of the range was
                    // offering an author a system that cannot exist.
                    ImGui::SliderFloat("Strength", &inject_amplitude_mps, 2.0f, 30.0f, "%.0f m/s");
                    ImGui::TextDisabled("Click the map to place one; it then evolves on its own.");

                    if (map_clicked)
                    {
                        const ImVec2 mouse = ImGui::GetIO().MousePos;
                        const double u = double(mouse.x - map_origin.x) / double(MAP_SIZE_PX);
                        const double v = double(mouse.y - map_origin.y) / double(MAP_SIZE_PX);
                        authoring.inject_vorticity(
                            map_position_at(u, v), double(inject_radius_km) * 1000.0,
                            inject_sign == 0 ? double(inject_amplitude_mps)
                                             : -double(inject_amplitude_mps));
                    }

                    // The resolution the map is actually reading the core at, so a span slid out
                    // to hemispheric is visibly a coarser look at the same field rather than a
                    // different field. Aliasing here is a property of the view, not the core.
                    ImGui::TextDisabled("Sampling ~%.0f km per map cell.",
                                        cell_span_rad * world->environment().planet.mean_radius() /
                                            1000.0);
                }

                if (ImGui::TreeNode("Advanced"))
                {
                    // True in *both* modes now. Manual used to be the mode with no provider, so
                    // its decks survived being edited; it places weather from a seed today and
                    // recompiles the stack from the local column every tick exactly as
                    // Procedural does. Saying "switch to Manual to hand-author decks" would send
                    // an author somewhere the same thing happens.
                    ImGui::TextDisabled(
                        "Deck editing is overwritten every tick: both weather modes compile this "
                        "stack from the column under the camera. The medium below still applies "
                        "-- it describes the whole sky, not one column.");

                    // The medium stays live under procedural weather. It used to be disabled
                    // along with the decks, which was right while WeatherCloudscapeCompiler's
                    // output was the entire description of the sky -- but since
                    // docs/slop/atmosphere_system.md §7.4 the decks no longer decide what a march
                    // sample finds (the cloudscape bake resolves genus and coverage per column
                    // from the simulated field), and these knobs are what is left of the author's
                    // control over how the sky *looks*. Disabling them would leave a procedurally
                    // driven sky with no authored handles at all.
                    ImGui::SeparatorText("Medium (all decks)");
                    if (ImGui::SliderFloat("Light Absorption", &environment.clouds.light_absorption,
                                           0.0f, 2.0f))
                        changed = true;
                    if (ImGui::SliderFloat("Forward Scatter", &environment.clouds.forward_scattering,
                                           0.0f, 0.99f))
                        changed = true;
                    if (ImGui::SliderFloat("Powder", &environment.clouds.powder_strength, 0.0f, 1.0f))
                        changed = true;
                    if (ImGui::SliderFloat("Ambient Fill", &environment.clouds.ambient_strength,
                                           0.0f, 2.0f))
                        changed = true;
                    if (ImGui::SliderFloat("Ground Shadow", &environment.clouds.ground_shadow_strength,
                                           0.0f, 1.0f))
                        changed = true;
                    if (ImGui::SliderFloat("Weather Scale", &environment.clouds.weather_scale,
                                           10000.0f, 200000.0f, "%.0f m"))
                        changed = true;
                    // Evolution rate is the one member of this block procedural weather does
                    // own: it is driven by the simulated wind and front activity, so the sky
                    // churns faster under a strong convective flow than a calm one.
                    ImGui::BeginDisabled(procedural);
                    if (ImGui::SliderFloat("Evolution Rate", &environment.clouds.evolution_rate,
                                           0.0f, 1.0f))
                        changed = true;
                    ImGui::EndDisabled();

                    // One deck per row: pick any of the ten WMO genera and nudge its coverage
                    // and density. Each deck inherits its genus's physical altitude band and
                    // morphology from the catalogue, so the sky is a few coexisting genera.
                    // Disabled under procedural weather, where the decks are recompiled every
                    // tick and an edit here would silently do nothing -- which is what the
                    // warning at the top of this node says.
                    ImGui::BeginDisabled(procedural);
                    const char* genus_items[SushiEngine::Render::CLOUD_GENUS_COUNT];
                    for (int g = 0; g < SushiEngine::Render::CLOUD_GENUS_COUNT; ++g)
                        genus_items[g] = SushiEngine::Render::cloud_genus_name(
                            static_cast<SushiEngine::Render::CloudGenus>(g));

                    for (int i = 0; i < SushiEngine::Render::CLOUD_MAX_DECKS; ++i)
                    {
                        SushiEngine::Render::CloudDeck& deck = environment.clouds.decks[i];
                        ImGui::PushID(i);
                        char header[32];
                        std::snprintf(header, sizeof(header), "Deck %d", i + 1);
                        ImGui::SeparatorText(header);
                        if (ImGui::Checkbox("Enabled", &deck.enabled))
                            changed = true;
                        ImGui::BeginDisabled(!deck.enabled);

                        int genus = static_cast<int>(deck.genus);
                        if (ImGui::Combo("Genus", &genus, genus_items,
                                         SushiEngine::Render::CLOUD_GENUS_COUNT))
                        {
                            deck.genus = static_cast<SushiEngine::Render::CloudGenus>(genus);
                            changed = true;
                        }
                        if (ImGui::SliderFloat("Coverage Bias", &deck.coverage_bias, -1.0f, 1.0f))
                            changed = true;
                        if (ImGui::SliderFloat("Density Scale", &deck.density_scale, 0.0f, 2.0f))
                            changed = true;
                        ImGui::EndDisabled();
                        ImGui::PopID();
                    }

                    ImGui::EndDisabled();
                    ImGui::TreePop();
                }

                ImGui::EndDisabled();
            }

            if (ImGui::CollapsingHeader("Stars"))
            {
                if (ImGui::Checkbox("Stars Enabled", &environment.stars.enabled))
                    changed = true;
                if (ImGui::SliderFloat("Star Brightness", &environment.stars.brightness, 0.0f, 4.0f))
                    changed = true;
                if (ImGui::SliderFloat("Star Density", &environment.stars.density, 0.0f, 1.0f))
                    changed = true;
            }

            if (ImGui::CollapsingHeader("Night Lighting", ImGuiTreeNodeFlags_DefaultOpen))
            {
                if (ImGui::Checkbox("Dynamic Ambient", &environment.night.enabled))
                    changed = true;
                ImGui::BeginDisabled(!environment.night.enabled);
                if (ImGui::SliderFloat("Reflected Light",
                                       &environment.night.reflected_intensity, 0.0f, 4.0f))
                    changed = true;
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip(
                        "Scales every light the sky's reflecting bodies cast. 1 is "
                        "physical: a full Moon is ~3e-6 of sunlight.");
                if (ImGui::SliderFloat("Star Ambient", &environment.night.star_intensity,
                                       0.0f, 4.0f))
                    changed = true;
                ImGui::EndDisabled();
                if (!environment.night.enabled || !context.sky_astronomical_sun)
                {
                    float ambient[3] = {static_cast<float>(environment.ambient.x),
                                        static_cast<float>(environment.ambient.y),
                                        static_cast<float>(environment.ambient.z)};
                    if (ImGui::ColorEdit3("Ambient", ambient))
                    {
                        environment.ambient =
                            SushiEngine::Vector3{ambient[0], ambient[1], ambient[2]};
                        changed = true;
                    }
                }
            }

            // The solar-system sky edits the editor context, not the world's environment:
            // the ephemeris repopulates the bodies and stars from these each frame in the
            // main loop, so scrubbing the date never re-extracts the world.
            if (ImGui::CollapsingHeader("Solar System", ImGuiTreeNodeFlags_DefaultOpen))
            {
                ImGui::Checkbox("Sky Enabled", &context.sky_enabled);
                ImGui::SameLine();
                ImGui::Checkbox("Astronomical Sun", &context.sky_astronomical_sun);

                ImGui::InputInt("Year", &context.sky_date.year);
                ImGui::SliderInt("Month", &context.sky_date.month, 1, 12);
                ImGui::SliderInt("Day", &context.sky_date.day, 1, 31);
                ImGui::SliderInt("Hour", &context.sky_date.hour, 0, 23);
                ImGui::SliderInt("Minute", &context.sky_date.minute, 0, 59);

                float latitude = static_cast<float>(context.sky_latitude_degrees);
                if (ImGui::SliderFloat("Latitude", &latitude, -90.0f, 90.0f, "%.2f deg"))
                    context.sky_latitude_degrees = latitude;
                float longitude = static_cast<float>(context.sky_longitude_degrees);
                if (ImGui::SliderFloat("Longitude", &longitude, -180.0f, 180.0f, "%.2f deg"))
                    context.sky_longitude_degrees = longitude;

                ImGui::Checkbox("Animate Time", &context.sky_animate);
                float days_per_second = static_cast<float>(context.sky_days_per_second);
                if (ImGui::SliderFloat("Days / Second", &days_per_second, 0.0f, 60.0f, "%.3f"))
                    context.sky_days_per_second = days_per_second;
                if (ImGui::Button("Reset Time Offset"))
                    context.sky_accumulated_days = 0.0;
                ImGui::SameLine();
                ImGui::Text("Offset: %.2f d", context.sky_accumulated_days);

                // Quick travel: one button per body, consumed by the main loop, which
                // teleports the camera to the body's sunlit side (Earth brings you home).
                ImGui::Separator();
                ImGui::TextDisabled("Travel");
                ImGui::PushID("Travel");
                const int per_row = 4;
                for (int body = 0; body < SushiEngine::Astro::BODY_COUNT; ++body)
                {
                    if (body % per_row != 0)
                        ImGui::SameLine();
                    const SushiEngine::Astro::BodyProperties properties =
                        SushiEngine::Astro::body_properties(
                            static_cast<SushiEngine::Astro::BodyId>(body));
                    if (ImGui::Button(properties.name, ImVec2(72.0f, 0.0f)))
                        context.sky_travel_target = body;
                }
                ImGui::PopID();
            }

            if (changed)
                // Scene content, like the Lighting panel's sun block: the write lands in
                // the world and the undo history, not in the preferences (which keep only
                // the default_environment a new scene starts from).
                commit_environment_edit(context, *world, environment);
            finish_environment_edit(context);

            ImGui::End();
        }
    } // namespace Editor
} // namespace SushiEngine
