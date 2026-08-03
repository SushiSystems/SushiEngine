/**************************************************************************/
/* meteorology_panel.cpp                                                 */
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

#include "meteorology_panel.hpp"

#include "../ui/panel_widgets.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <ios>

#include <imgui.h>

#include <SushiEngine/environment/environment.hpp>
#include <SushiEngine/sim/simulation_settings.hpp>

namespace SushiEngine
{
    namespace Editor
    {
        using SushiEngine::Simulation::IWorldEditor;

        namespace
        {

            /** @brief The mirror column under the observer — the nest is centred on it. */
            const SushiEngine::Render::AtmosphereMirrorColumn* observer_column(
                const SushiEngine::Render::AtmosphereMirror& mirror)
            {
                if (!mirror.valid())
                    return nullptr;
                const int centre = mirror.cells / 2;
                return &mirror.columns[std::size_t(centre) * std::size_t(mirror.cells) +
                                       std::size_t(centre)];
            }
        } // namespace

        void draw_meteorology_panel(EditorContext& context)
        {
            if (!context.panels.meteorology)
                return;
            if (!ImGui::Begin("Meteorology", &context.panels.meteorology))
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
            const SushiEngine::Render::Environment environment = world->environment();

            const SushiEngine::Render::AtmosphereMirror mirror =
                context.assets != nullptr ? context.assets->atmosphere_mirror()
                                          : SushiEngine::Render::AtmosphereMirror{};

            // ---- The clock, and whether the atmosphere can keep up with it -----------------
            //
            // The most load-bearing readout here. The nest advances in *game* time by at most
            // `max_steps_per_frame` steps of its CFL-chosen length per frame; a sky animating
            // faster than that runs the sun ahead of the atmosphere it is meant to be heating,
            // and the surface forcing then oscillates faster than a boundary layer can respond.
            // Invisible without a number to look at, which is why there is one.
            //
            // The atmosphere's own quality tier lives here — in the panel that owns the
            // domain — not in Rendering. It is the control whose side effect is worth a
            // warning *at the control*: resolving a different grid rebuilds the nest,
            // and a rebuilt nest starts from its base state.
            ImGui::SeparatorText("Quality");
            {
                using SushiEngine::Simulation::AtmosphereQuality;
                const char* const TIERS[] = {"Low", "Medium", "High", "Ultra"};
                int tier = static_cast<int>(context.simulation_settings.atmosphere.quality);
                if (ImGui::Combo("Atmosphere Quality", &tier, TIERS, 4))
                {
                    context.simulation_settings.atmosphere.quality =
                        static_cast<AtmosphereQuality>(tier);
                    context.preferences_dirty = true;
                }
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("The weather simulation's grid resolution — independent\n"
                                      "of the render quality tier. Persisted per user, like a\n"
                                      "graphics setting: a grid is a machine budget, not scene\n"
                                      "content.");
                ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.3f, 1.0f),
                                   "Changing this rebuilds the nest: the running weather "
                                   "restarts from the base state.");
            }

            // The GPU regional nest's authored physics (docs/slop/atmosphere_system.md §6).
            // Everything here is authored data serialized with the scene -- the design doc
            // states twice that no physical constant may live inside a loop body, and this
            // section is what makes that claim mean something to an author rather than only
            // to a reader. It lives in Meteorology, not Environment: this panel owns the
            // atmosphere simulation, and Environment keeps what a level artist touches (the
            // sun, the sky's look, fog, GI). Edits land in the world as scene content, one
            // undo step per gesture, exactly like the Environment panel's own writes.
            {
                SushiEngine::Render::Environment edited = environment;
                bool changed = false;
                if (ImGui::TreeNode("Atmosphere physics (regional nest)"))
                {
                    SushiEngine::Render::AtmosphereParameters& nest = edited.atmosphere_nest;

                    if (ImGui::Checkbox("Nest Enabled", &nest.enabled))
                        changed = true;
                    ImGui::BeginDisabled(!nest.enabled);

                    // The two numbers that decide whether an afternoon convects at all: how hard
                    // the ground heats the air above it, and how much moisture it gives up. A real
                    // surface energy balance replaces both in the next phase.
                    ImGui::SeparatorText("Surface forcing (prescribed this phase)");

                    // The land cover, as the *properties* it implies.
                    //
                    // These presets used to set a pair of fluxes directly. Since Phase B3 the
                    // fluxes are solved, so what a preset sets is what a place actually is —
                    // how bright it is, how much water it can give up, how much heat it stores,
                    // and how moist the airmass over it is. The Bowen ratio it produces then
                    // falls out of the balance and moves through the day as the ground dries,
                    // instead of being pinned.
                    //
                    // The airmass humidity belongs here rather than only on the slider below,
                    // because a semi-desert is not merely a dry *surface*: with the base-state
                    // vapour profile corrected, a dry surface under a 70 %-humidity airmass still
                    // makes an afternoon deck at 2 km, which is the model being right and the
                    // preset being incomplete. A place is a surface and the air over it.
                    //
                    // Presets rather than a default change, deliberately: which of these a scene
                    // is standing on is an authoring decision, and the engine has no way to guess
                    // it. The measured skies are in each tooltip; the panel above reports the
                    // ratio the model is actually running at.
                    struct SurfacePreset
                    {
                        const char* name;
                        float albedo;
                        float availability;
                        float capacity;
                        float humidity;
                        const char* note;
                    };
                    static const SurfacePreset PRESETS[] = {
                        {"Semi-desert / bare soil", 0.30f, 0.10f, 6.0e4f, 0.35f,
                         "Bright, dry and thin, under an airmass to match: it heats\n"
                         "fast, gives up almost no water, and its Bowen ratio runs\n"
                         "well above 1. Measured: clear all day, condensation level\n"
                         "climbing 2.1 -> 3.0 km. The sky stays clear however long it\n"
                         "is left running, and that is the model being right rather\n"
                         "than idle."},
                        {"Mixed cropland", 0.22f, 0.30f, 1.0e5f, 0.60f,
                         "A dry summer, or land partly harvested. Measured: nothing\n"
                         "until mid-afternoon, then a third of the columns carrying\n"
                         "3 % cover at 1.8 km -- cloud, but late and thin."},
                        {"Vegetated summer land", 0.18f, 0.55f, 1.5e5f, 0.65f,
                         "Temperate grassland or forest in leaf. Dark, damp, and\n"
                         "with enough thermal mass to keep the afternoon going --\n"
                         "the fair-weather cumulus case. Measured: a 23 % deck at\n"
                         "1 341 m by mid-afternoon, out of nothing but the ground."},
                        {"Open water / lake", 0.06f, 1.00f, 1.3e7f, 0.75f,
                         "Dark, saturated, and with the heat capacity of a three\n"
                         "metre mixed layer: the skin barely moves over a whole\n"
                         "day. Almost all the sun's energy goes into evaporation,\n"
                         "so the layer is moist but barely buoyant -- measured, a\n"
                         "22 % deck with its base at 214-517 m and staying there,\n"
                         "which is stratocumulus rather than more cumulus. It is\n"
                         "this contrast against the land beside it that a sea\n"
                         "breeze is made of."},
                    };
                    ImGui::TextUnformatted("Land cover");
                    ImGui::SameLine();
                    ImGui::TextDisabled("(sets the three properties below, and the airmass humidity)");
                    for (const SurfacePreset& preset : PRESETS)
                    {
                        if (ImGui::Button(preset.name))
                        {
                            nest.surface_albedo = preset.albedo;
                            nest.surface_moisture_availability = preset.availability;
                            nest.surface_heat_capacity = preset.capacity;
                            nest.surface_humidity = preset.humidity;
                            changed = true;
                        }
                        if (ImGui::IsItemHovered())
                            ImGui::SetTooltip(
                                "albedo %.2f, moisture %.2f, slab %.2g J/m2/K, airmass RH %.2f.\n%s",
                                double(preset.albedo), double(preset.availability),
                                double(preset.capacity), double(preset.humidity), preset.note);
                    }

                    if (ImGui::SliderFloat("Albedo", &nest.surface_albedo, 0.0f, 0.9f, "%.2f"))
                        changed = true;
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("How much sunlight the ground throws away. The widest\n"
                                          "range of any parameter here: 0.06 for water at noon,\n"
                                          "0.20 for vegetation, 0.8 for fresh snow. First thing\n"
                                          "to reach for when a scene heats too fast or too slow.");
                    if (ImGui::SliderFloat("Moisture Availability", &nest.surface_moisture_availability,
                                           0.0f, 1.0f, "%.2f"))
                        changed = true;
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("The fraction of the saturation deficit the ground can\n"
                                          "actually supply -- 1 is open water or saturated soil,\n"
                                          "0 is dry rock. **This is the Bowen ratio's author.**\n"
                                          "It decides whether a heated afternoon reaches its\n"
                                          "condensation level or merely gets hotter, which is\n"
                                          "the difference between a clear sky and a cumulus deck\n"
                                          "at every value of every other parameter here.");
                    if (ImGui::SliderFloat("Heat Capacity", &nest.surface_heat_capacity, 1.0e4f,
                                           1.5e7f, "%.2g J/m2/K", ImGuiSliderFlags_Logarithmic))
                        changed = true;
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("How much energy warms the ground by a degree, and so\n"
                                          "how far the afternoon peak lags solar noon. 1e5 is a\n"
                                          "few centimetres of soil (an hour or two of lag); 1.3e7\n"
                                          "is three metres of water, which barely moves all day.\n"
                                          "Safe at any value: the slab is solved semi-implicitly,\n"
                                          "so a thin one cannot destabilise the step.");
                    if (ImGui::SliderFloat("Surface Patchiness", &nest.thermal_seed_amplitude, 0.0f,
                                           1.0f, "%.2f"))
                        changed = true;
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("How much the surface heating varies across the\n"
                                          "ground, as a fraction. Convection needs it: a\n"
                                          "uniformly heated surface rises as one slab and\n"
                                          "never forms cells. 0.4 is mixed terrain. It scales\n"
                                          "the flux, so it cannot cool the ground and it\n"
                                          "stops at dusk.");
                    // The two scales below are what make the slider above do anything. Measured:
                    // patchiness with no scales -- redrawn per cell per step, as it was -- gives
                    // 1.22x the domain structure of switching the seed off entirely, against
                    // 3.72x for these defaults. They are in metres and seconds rather than cells
                    // and steps so that the quality tier and the frame rate are not physical
                    // parameters of the weather.
                    if (ImGui::SliderFloat("Patch Size", &nest.thermal_seed_length_m, 1000.0f,
                                           24000.0f, "%.0f m"))
                        changed = true;
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("How wide a patch of warmer ground is. Convection\n"
                                          "organises on this scale, so it sets how big the\n"
                                          "cumulus cells come out. 6 km is the scale land\n"
                                          "cover and soil moisture actually vary over; much\n"
                                          "wider is broader than the plumes that form.");
                    if (ImGui::SliderFloat("Patch Lifetime", &nest.thermal_seed_period_s, 60.0f,
                                           3600.0f, "%.0f s"))
                        changed = true;
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("How long a patch stays warmer before the pattern\n"
                                          "renews. This is the axis that matters: a patch\n"
                                          "gone in one step cannot lift anything, because a\n"
                                          "thermal needs a boundary-layer turnover -- about\n"
                                          "15 min -- to organise around it. Longer than an\n"
                                          "hour and the pattern stops renewing at all.");

                    // Without this the two above go nowhere. The turbulence that lifts surface
                    // heat and moisture out of a 54 m surface layer is far below a 2 km grid, so
                    // it is parameterized; with it switched off the fluxes pile up in the lowest
                    // level, and a horizontally uniform hot slab cannot rise at all.
                    ImGui::SeparatorText("Boundary layer");
                    if (ImGui::SliderFloat("Mixing Depth Cap", &nest.boundary_layer_depth_m, 0.0f,
                                           4000.0f, "%.0f m"))
                        changed = true;
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("A ceiling, not the depth. The depth itself is\n"
                                          "diagnosed per column by the parcel method and grows\n"
                                          "through the morning; this stands for the capping\n"
                                          "inversion that stops a fair-weather layer.");
                    if (ImGui::SliderFloat("Mixing Strength", &nest.boundary_layer_velocity_scale,
                                           0.0f, 6.0f, "%.2f m/s"))
                        changed = true;
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("The mixed layer's turbulent velocity scale, which\n"
                                          "sets the whole diffusivity profile (Troen-Mahrt:\n"
                                          "K = 0.4 w z (1 - z/h)^2). 1-2 m/s is a convective\n"
                                          "afternoon, peaking at 150-300 m2/s a third of the\n"
                                          "way up. Zero removes the boundary layer entirely.");

                    ImGui::SeparatorText("Base state");
                    if (ImGui::SliderFloat("Surface Temperature", &nest.surface_temperature, 233.0f,
                                           323.0f, "%.1f K"))
                        changed = true;
                    if (ImGui::SliderFloat("Surface Humidity", &nest.surface_humidity, 0.0f, 1.0f))
                        changed = true;
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("Relative humidity of the airmass at the ground, and\n"
                                          "the single strongest lever on whether a scene has a\n"
                                          "sky. The land-cover presets above set it with the\n"
                                          "surface, because a place is a surface and the air\n"
                                          "over it.");
                    if (ImGui::SliderFloat("Humidity Scale Height", &nest.humidity_scale_height,
                                           500.0f, 6000.0f, "%.0f m"))
                        changed = true;
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("e-folding height of the base-state *mixing ratio*.\n"
                                          "Relative humidity is then whatever q_v / q_s comes\n"
                                          "to -- 70 %% at the ground reads 62 %% at 1.3 km, not\n"
                                          "41 %%. Raising it moistens the whole free troposphere\n"
                                          "and drops the condensation level with it.");
                    if (ImGui::SliderFloat("Free-troposphere Drying", &nest.free_troposphere_drying,
                                           0.0f, 0.95f, "%.2f"))
                        changed = true;
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("Fraction of the surface humidity the base state has\n"
                                          "lost by the tropopause (Weisman-Klemp 1982). It caps\n"
                                          "the profile aloft and nothing near the ground. At 0\n"
                                          "the uncapped mixing ratio reaches saturation near\n"
                                          "9.5 km and every run starts under a cirrus deck.");

                    // Kessler's own rate constants. The autoconversion threshold is the one an
                    // author feels most directly: below it a cloud never rains, however thick it
                    // gets, which is the difference between fair-weather cumulus and a shower.
                    ImGui::SeparatorText("Microphysics (Kessler warm rain)");
                    // The closure that decides whether this nest can draw a cumulus at all. A
                    // 2 km cell's humidity is a *mean*, and a fair-weather cumulus is saturated
                    // air filling a fraction of it, so condensing only when the mean crosses
                    // saturation produces fog and nothing else -- which is exactly what every
                    // run did before this existed.
                    if (ImGui::SliderFloat("Cloud From RH", &nest.cloud_critical_humidity, 0.5f,
                                           1.0f, "%.2f"))
                        changed = true;
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("The cell-mean relative humidity subgrid cloud starts\n"
                                          "at. 0.80 is standard at this spacing. Lower gives\n"
                                          "more, thinner, earlier cloud; 1.00 switches the\n"
                                          "subgrid closure off entirely and condenses on the\n"
                                          "cell mean alone, which is fog or a clear sky.");
                    if (ImGui::SliderFloat("Cloud-top Longwave", &nest.cloud_top_longwave_flux,
                                           0.0f, 150.0f, "%.0f W/m2"))
                        changed = true;
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("The longwave a cloud top loses to space -- the sink a\n"
                                          "cloud has and the clear air beside it does not. It is\n"
                                          "absorbed within the top few tens of grams of water, so\n"
                                          "it cools the top and not the deck, which is what makes\n"
                                          "a stratocumulus overturn instead of sitting still. At\n"
                                          "0 a nocturnal deck's only sink is the parent's\n"
                                          "subsidence and it will outlive the night.");
                    if (ImGui::SliderFloat("Cloud-top Floor", &nest.cloud_top_equilibrium_depression,
                                           1.0f, 60.0f, "%.0f K"))
                        changed = true;
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("How far below its environment a radiating cloud top\n"
                                          "settles. The sky above returns a growing share of what\n"
                                          "the top emits as it cools, and here the two balance and\n"
                                          "the loss above stops. This is what makes that loss a\n"
                                          "flux rather than a sink: raise it far enough and a deck\n"
                                          "that persists cools without bound, which is what a\n"
                                          "quiescent 72 h run measured at -42.7 K.");
                    if (ImGui::SliderFloat("Autoconversion Threshold", &nest.autoconversion_threshold,
                                           0.0f, 5.0e-3f, "%.4f kg/kg"))
                        changed = true;
                    if (ImGui::SliderFloat("Autoconversion Rate", &nest.autoconversion_rate, 0.0f,
                                           5.0e-3f, "%.4f /s"))
                        changed = true;
                    if (ImGui::SliderFloat("Accretion Rate", &nest.accretion_rate, 0.0f, 6.0f,
                                           "%.2f /s"))
                        changed = true;
                    if (ImGui::SliderFloat("Rain Evaporation", &nest.rain_evaporation_rate, 0.0f,
                                           0.1f, "%.3f /s"))
                        changed = true;
                    // The single number deciding whether a given amount of condensate reads as a
                    // crisp white cumulus or a thin haze -- the same water, differently divided.
                    if (ImGui::SliderFloat("Droplet Radius", &nest.droplet_effective_radius, 3.0e-6f,
                                           2.0e-5f, "%.7f m"))
                        changed = true;
                    // Raise this if the sky reads as a solid white ceiling: it is the in-cloud
                    // water content that renders as fully opaque cloud (0.4 g/m³ ≈ a solid
                    // stratocumulus), and it changes nothing about the physics that produced
                    // the water.
                    if (ImGui::SliderFloat("Overcast At", &nest.coverage_reference_lwc, 1.0e-4f,
                                           2.0e-3f, "%.5f kg/m3"))
                        changed = true;

                    // Ice is a *diagnosed phase*, not a second species: everything below is a
                    // function of the cell's own temperature. Which is why there is a band and
                    // not a switch -- a cloud at -5 C is mostly supercooled water and one at
                    // -20 C is mostly crystals, and the band between them is where the two
                    // coexist and where a cloud turns itself into snow.
                    ImGui::SeparatorText("Ice");
                    if (ImGui::SliderFloat("All Liquid Above", &nest.freezing_temperature, 263.15f,
                                           278.15f, "%.2f K"))
                        changed = true;
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("Warm edge of the mixed-phase band. Above this a cloud\n"
                                          "is entirely liquid and every ice term below is\n"
                                          "exactly inert, so a summer scene is untouched by any\n"
                                          "of this.");
                    if (ImGui::SliderFloat("All Ice Below", &nest.glaciation_temperature, 233.15f,
                                           268.15f, "%.2f K"))
                        changed = true;
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("Cold edge. Saturation over ice is lower than over\n"
                                          "water, so a cell crossing into the band starts\n"
                                          "condensing at a humidity it would have been clear at\n"
                                          "-- which is why cold cloud forms where warm cloud\n"
                                          "would not, on the same water.");
                    if (ImGui::SliderFloat("Ice Radius", &nest.ice_effective_radius, 1.0e-5f,
                                           1.0e-4f, "%.7f m"))
                        changed = true;
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("The optical half, and the half you can see. A crystal\n"
                                          "aggregate is nearly four times a droplet's effective\n"
                                          "radius, so glaciated cloud is that much thinner for\n"
                                          "the same water: a translucent cirrus veil and a soft\n"
                                          "anvil top instead of a wall of white.");
                    if (ImGui::SliderFloat("Snow Fall Speed", &nest.snow_fall_speed_coefficient,
                                           1.0f, 20.0f, "%.1f"))
                        changed = true;
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("An order of magnitude under rain's, and that single\n"
                                          "fact is most of what makes snow look like snow: a\n"
                                          "metre a second where a raindrop of the same water\n"
                                          "falls at seven. It is also why a snow shaft leans so\n"
                                          "far downwind.");
                    if (ImGui::SliderFloat("Snow Precipitates At", &nest.glaciated_autoconversion_factor,
                                           0.05f, 1.0f, "%.2f x"))
                        changed = true;
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("Fraction of the warm threshold a fully glaciated cloud\n"
                                          "needs before it precipitates. Crystals stick where\n"
                                          "droplets have to coalesce, so a winter deck snows out\n"
                                          "of cloud that in summer would just sit there. At 1.00\n"
                                          "ice precipitates no more readily than water.");

                    ImGui::SeparatorText("Dynamics");
                    if (ImGui::SliderFloat("Courant Target", &nest.courant_target, 0.1f, 1.5f))
                        changed = true;
                    if (ImGui::SliderFloat("Eddy Viscosity", &nest.eddy_viscosity, 0.0f, 400.0f,
                                           "%.0f m2/s"))
                        changed = true;
                    if (ImGui::SliderFloat("Boundary Relaxation", &nest.boundary_relaxation, 0.0f,
                                           0.2f, "%.3f /s"))
                        changed = true;
                    int pressure_iterations = int(nest.pressure_iterations);
                    if (ImGui::SliderInt("Pressure Sweeps", &pressure_iterations, 1, 48))
                    {
                        nest.pressure_iterations = std::uint32_t(pressure_iterations);
                        changed = true;
                    }
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("Red-black sweeps of the pressure solve per step. The\n"
                                          "vertical is solved exactly, so these only iterate the\n"
                                          "horizontal coupling -- which this grid's anisotropy\n"
                                          "(2 km against 54-560 m) makes a small, strongly\n"
                                          "diagonally dominant correction.\n\n"
                                          "Measured: the solve converges at 2. From 2 to 20 the\n"
                                          "weather is identical to every printed figure and the\n"
                                          "peak divergence agrees to seven. 4 is that with a\n"
                                          "margin. Raising it costs about 0.7 ms a step each and\n"
                                          "buys nothing measurable.");

                    // How much weather one frame may buy. Read out by the Meteorology panel's
                    // clock since that panel existed, but never authorable -- and it is the
                    // cheapest lever on the whole tier's throughput, because the nest advances at
                    // exactly `this x frame rate x step` game seconds per second of wall clock.
                    // Raising it trades frame time for weather time directly.
                    int steps_per_frame = int(nest.max_steps_per_frame);
                    if (ImGui::SliderInt("Steps Per Frame", &steps_per_frame, 1, 32))
                    {
                        nest.max_steps_per_frame = std::uint32_t(steps_per_frame);
                        changed = true;
                    }
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("The nest's throughput, directly: it advances\n"
                                          "steps x frame rate x step-length game seconds per\n"
                                          "second. Past this the surplus is discarded rather\n"
                                          "than caught up, so raising it is how a scene gets its\n"
                                          "weather sooner -- paid for in frame time, since the\n"
                                          "nest submits on the graphics queue.");

                    ImGui::EndDisabled();
                    ImGui::TreePop();
                }
                if (changed)
                    commit_environment_edit(context, *world, edited);
                finish_environment_edit(context);
            }

            ImGui::SeparatorText("Clock");
            const SushiEngine::Render::AtmosphereParameters& nest = environment.atmosphere_nest;

            const double sky_rate = context.sky_animate ? context.sky_days_per_second * 86400.0 : 0.0;
            // The grid the *atmosphere tier* resolves to, not a hard-coded default: the nest's
            // discretization is a tier parameter, so a panel that assumed 192x192x48 would
            // report the wrong step and the wrong sustainable rate on every tier but High.
            // The atmosphere's own tier, not the render tier — the separation that stops
            // "Ultra rendering" from rebuilding the weather.
            const SushiEngine::Render::AtmosphereNestSize size =
                SushiEngine::Simulation::resolve_atmosphere_quality(
                    context.simulation_settings.atmosphere.quality);
            // The step the nest will pick *before its first readback*, which is the only one
            // predictable from here: after that the vertical CFL binds against the updraft the
            // nest measures itself, and the step lengthens in quiet air.
            const float thinnest =
                SushiEngine::Render::atmosphere_level_thickness(0, size.levels, size.top_m);
            const float step = std::clamp(nest.courant_target * thinnest /
                                              std::max(10.0f * nest.convective_velocity_scale, 1.0f),
                                          nest.min_step_seconds, nest.max_step_seconds);
            ImGui::Text("Grid:       %u x %u x %u at %.0f m  (%.0f km domain, %.1f M cells)",
                        size.cells_x, size.cells_z, size.levels, double(size.spacing_m),
                        double(size.spacing_m) * double(size.cells_x) / 1000.0,
                        double(size.cells_x) * double(size.cells_z) * double(size.levels) / 1e6);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Set by the Atmosphere Quality tier in this panel. The domain\n"
                                  "is 384 km at every tier and only the resolution changes, so\n"
                                  "raising the tier resolves the same weather more finely rather\n"
                                  "than simulating a different amount of world.\n\n"
                                  "2 km (High) is where convection stops being parameterized and\n"
                                  "starts being resolved.");
            // What the nest could sustain *if the editor rendered at 60 fps*. It does not: this
            // panel is drawn beside a 192x192x48 nest, three scene views and a deferred
            // renderer, and the frame rate that results is the whole term. Kept only as the
            // estimate to fall back on before the first readback, because until then there is
            // nothing measured to prefer to it — and labelled as an assumption rather than
            // stated as a fact, which is what it used to be.
            const double nominal = double(step) * double(nest.max_steps_per_frame) * 60.0;

            // The measured lag, which is the ground truth the estimate above only guesses at.
            //
            // The simulation accumulates game seconds at whatever rate the sky is animating; the
            // nest takes at most `max_steps_per_frame` steps from the difference and **discards
            // the surplus** (§3.4 — weather that is briefly behind beats a frame that stalls to
            // simulate an hour of it). So a sky running faster than the nest can step does not
            // slow the sky down, it throws weather away, and nothing said so.
            //
            // This is the number to trust, and it is the one that showed the 60 fps assumption
            // was wrong: a session was told "the atmosphere is keeping up with the sky" while
            // four of every five seconds of weather were being dropped, because the editor was
            // drawing at about twelve frames a second and the estimate did not know.
            const double asked = environment.atmosphere_forcing.total_seconds;
            const bool measured = mirror.valid() && asked > 60.0 &&
                                  mirror.simulated_seconds > 1.0;
            const double lag =
                measured ? context.panel_state.meteorology.clock_lag.update(asked, mirror.simulated_seconds) : 1.0;
            // The rate the nest actually achieved, which is the sky's rate divided by how much
            // of it landed. Only meaningful once a readback has happened.
            const double achievable = measured ? sky_rate / lag : nominal;

            ImGui::Text("Sky:        %.0f x real  (%.4f days/s)", sky_rate,
                        context.sky_animate ? context.sky_days_per_second : 0.0);
            if (measured)
                ImGui::Text("Atmosphere: %.0f x real  (%.2f s step, measured over this session)",
                            achievable, step);
            else
                ImGui::Text("Atmosphere: %.0f x real est.  (%.2f s step, %u/frame, assuming 60 fps)",
                            nominal, step, nest.max_steps_per_frame);

            if (!context.sky_animate)
                ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f),
                                   "Time is frozen -- enable Sky > Animate or nothing evolves.");
            else if (measured && lag > 1.2)
            {
                ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.3f, 1.0f),
                                   "%.0f of every %.0f seconds of weather are being thrown away.\n"
                                   "The nest steps in game time and drops whatever it cannot\n"
                                   "reach, so animating the sky faster does not make the weather\n"
                                   "evolve faster -- it makes less of it happen.",
                                   lag - 1.0, lag);
            }
            else if (!measured && sky_rate > nominal * 1.05)
                ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f),
                                   "Sky may be %.1fx faster than the atmosphere can step. No\n"
                                   "readback yet, so this is the 60 fps estimate rather than\n"
                                   "a measurement.",
                                   sky_rate / std::max(nominal, 1.0));
            else
                ImGui::TextColored(ImVec4(0.5f, 1.0f, 0.5f, 1.0f),
                                   "The atmosphere is keeping up with the sky.");

            if (measured)
                ImGui::Text("Session total: asked for %.1f h of weather, simulated %.1f h.",
                            asked / 3600.0, mirror.simulated_seconds / 3600.0);

            // Matched against what the nest *did*, not against what a formula says it could.
            // The measured rate already contains the frame rate, the scene's cost and this
            // machine, none of which the estimate can see; the margin is for the ordinary
            // variation in all three rather than a fudge. The label names what it changes
            // (the Environment panel's time-of-day animation rate) — a button that quietly
            // set another panel's slider read as magic, and magic reads as broken.
            if (ImGui::Button("Slow the sky's time rate to match"))
            {
                context.sky_days_per_second = achievable * 0.9 / 86400.0;
                context.sky_animate = true;
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Sets the Environment panel's Solar System animation rate\n"
                                  "(days per second) to what the atmosphere measurably kept\n"
                                  "up with, so no simulated weather is discarded.");
            ImGui::SameLine();
            if (measured)
                ImGui::TextDisabled("(the rate this machine actually achieved, less 10%%)");
            else
                ImGui::TextDisabled("(estimated; press again once weather has been simulated)");

            // ---- What a step costs, measured ------------------------------------------------
            //
            // The rate above says how much weather is happening; this says what it is costing to
            // make it happen, which is the other half of the same question and was for a long
            // time not asked at all: §12 budgets the step at 2 ms and nothing had ever measured
            // it. Timestamps around each stage of the step, resolved where the nest already
            // waits on the submission's timeline value, so reading them stalls nothing.
            //
            // Both numbers are stated because they answer different questions. The per-step cost
            // is what §12's budget is written against; the per-frame cost is what a dropped frame
            // is caused by, and at a couple of seconds of game time per step the two differ by
            // more than two orders of magnitude. A step that is "seven times over budget" and a
            // tier that costs a tenth of a millisecond a frame are the same measurement.
            const SushiEngine::Render::AtmosphereStepCost cost =
                context.assets != nullptr ? context.assets->atmosphere_step_cost()
                                          : SushiEngine::Render::AtmosphereStepCost{};
            if (cost.measured && cost.steps > 0)
            {
                const float per_step = cost.total_ms / float(cost.steps);
                // The step lands every `step` seconds of game time, and game time runs at
                // `sky_rate` times real time — so this is how much of each wall-clock second of
                // rendering the atmosphere is taking.
                const double steps_per_second =
                    double(step) > 0.0 ? std::max(sky_rate, 1.0) / double(step) : 0.0;
                const double per_frame_ms = double(per_step) * steps_per_second / 60.0;

                if (ImGui::TreeNode("Step cost"))
                {
                    ImGui::Text("%.2f ms per step, %u step(s) in the measured frame",
                                double(per_step), cost.steps);
                    ImGui::Text("~%.3f ms per frame at 60 fps and the current sky rate",
                                per_frame_ms);
                    if (per_step > 2.0f)
                        ImGui::TextDisabled("(doc " "\xc2\xa7" "12 budgets 2.00 ms per step)");
                    ImGui::Separator();
                    // Against *one* step, not against the submission. The breakdown is the first
                    // step of the frame and the total covers all of them, so dividing one by the
                    // other reports a stage as a quarter of its real share on a four-step frame
                    // — which is exactly when someone is looking at this panel.
                    for (int i = 0; i < cost.count; ++i)
                    {
                        const float ms = cost.stages[i].milliseconds;
                        ImGui::Text("%-14s %7.3f ms  %5.1f %%", cost.stages[i].name, double(ms),
                                    per_step > 0.0f ? 100.0 * double(ms / per_step) : 0.0);
                    }
                    ImGui::TextDisabled(
                        "The breakdown is the first step of the frame; the total covers all of\n"
                        "them. Sections can overlap, so they need not sum to the total.");
                    ImGui::TreePop();
                }
            }
            else if (context.assets != nullptr && mirror.valid())
            {
                ImGui::TextDisabled("Step cost: not measured (no timestamp queries on this device)");
            }

            // A frozen sun is a different failure from a frozen clock and reads identically in
            // every other readout: the nest steps, the log fills, and the surface forcing never
            // changes because the sun never moves. Without the astronomical sun the direction is
            // whatever the Environment panel authored, so there is no diurnal cycle at all --
            // no morning growth, no evening decay, just one elevation held forever.
            if (!context.sky_astronomical_sun)
                ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f),
                                   "The sun is authored, not astronomical, so it never moves and\n"
                                   "there is no diurnal cycle -- the ground is held at this one\n"
                                   "elevation. Switch on Sky > Astronomical Sun for morning\n"
                                   "growth and evening decay.");

            // Why there is nothing to look at, when there is nothing to look at. The nest is
            // built lazily and only when several things line up, and "no readback yet" on its own
            // sends you looking in the wrong place -- as it did. Each rung of this chain is a
            // different fix, so the panel names the rung rather than the symptom.
            const bool procedural =
                world->weather_mode() == SushiEngine::Simulation::WeatherMode::Procedural;
            const bool forcing_published = environment.atmosphere_forcing.valid();
            if (mirror.valid())
            {
                ImGui::Text("Simulated %.0f s of game time (%.2f h).", mirror.simulated_seconds,
                            mirror.simulated_seconds / 3600.0);
            }
            else if (!procedural)
            {
                // No longer "the sky is uniform everywhere" — Manual mode places weather over
                // the whole planet from a seed now, so what it does not have is a *simulation*,
                // not a *field*. Naming the wrong absence would send an author looking for a bug
                // that was fixed.
                ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.3f, 1.0f),
                                   "Weather mode is Manual, so the sky is placed from a seed\n"
                                   "rather than simulated: nothing publishes the forcing the\n"
                                   "nest is driven by and no nest is ever built. There is real\n"
                                   "horizontal structure, but nothing below evolves.");
                // The fix, next to the diagnosis. The alternative is a panel that names a
                // control in a different panel and expects you to go find it, which is how the
                // toggle stayed off long enough to be mistaken for a physics bug.
                if (ImGui::Button("Switch to Procedural weather"))
                    world->set_weather_mode(SushiEngine::Simulation::WeatherMode::Procedural);
            }
            else if (!nest.enabled)
            {
                ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.3f, 1.0f),
                                   "The nest is switched off below; the sky is coming from the\n"
                                   "classified deck stack instead.");
                if (ImGui::Button("Switch the nest on"))
                {
                    SushiEngine::Render::Environment updated = environment;
                    updated.atmosphere_nest.enabled = true;
                    commit_environment_edit(context, *world, updated);
                }
            }
            else if (!forcing_published)
            {
                ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.3f, 1.0f),
                                   "Weather is procedural but no forcing reached the renderer\n"
                                   "this frame -- the simulation has not extracted yet.");
            }
            else
            {
                ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f),
                                   "Forcing is published and the nest is building, but no\n"
                                   "readback has completed. If this never clears, the nest is\n"
                                   "seeded but never stepping -- check the clock above.");
            }

            // ---- The sun: one number that carries both the hour and the season -------------
            ImGui::SeparatorText("Solar forcing");
            const SushiEngine::Vector3& to_sun = environment.sun.direction;
            const SushiEngine::Vector3& up = environment.planet_pole;
            const double up_length = std::sqrt(up.x * up.x + up.y * up.y + up.z * up.z);
            const double sun_length =
                std::sqrt(to_sun.x * to_sun.x + to_sun.y * to_sun.y + to_sun.z * to_sun.z);
            const double sine = (up_length > 0.0 && sun_length > 0.0)
                                    ? (to_sun.x * up.x + to_sun.y * up.y + to_sun.z * up.z) /
                                          (up_length * sun_length)
                                    : 0.0;
            const double elevation = std::asin(std::clamp(sine, -1.0, 1.0)) * 180.0 / 3.14159265358979;
            ImGui::Text("Sun elevation: %+.1f deg  (sin = %+.3f)", elevation, sine);
            ImGui::TextDisabled("Seasons are this number: the declination that lifts the summer\n"
                                "sun is already in the ephemeris, so a July noon delivers more\n"
                                "heat than a January one with nothing modelling \"summer\".");

            // **Measured, not authored.** These were the two numbers a user typed in; since
            // Phase B3 they are what the surface energy balance solved, read back off the
            // observer's column. The difference shows the moment the sun moves: the fluxes lag
            // it, because the ground has a heat capacity now.
            const SushiEngine::Render::AtmosphereMirrorColumn* surface_column =
                observer_column(mirror);
            if (surface_column == nullptr)
            {
                ImGui::TextDisabled("Surface flux:  no readback yet");
            }
            else
            {
                ImGui::Text("Skin:          %.1f C   (net radiation %+.0f W/m2)",
                            double(surface_column->skin[0]) - 273.15,
                            double(surface_column->skin[3]));
                ImGui::Text("Surface flux:  %.0f W/m2 sensible, %.0f W/m2 latent",
                            double(surface_column->skin[1]), double(surface_column->skin[2]));

                // The Bowen ratio, named. It is the single number that decides whether a heated
                // boundary layer reaches its condensation level or simply gets hotter, and it is
                // a *derived* quantity so nothing in the panel showed it. It is now derived from
                // the solved fluxes rather than from two authored ones, which means it moves
                // through the day as the surface dries — the fixed ratio was itself part of what
                // made a scene heat all afternoon and stay clear.
                const float latent = surface_column->skin[2];
                const float bowen = latent > 1.0f ? surface_column->skin[1] / latent : 0.0f;
                if (latent <= 1.0f)
                    ImGui::TextDisabled("Bowen ratio:   -- (no latent flux to divide by)");
                else
                    ImGui::Text("Bowen ratio:   %.2f  (sensible / latent)", double(bowen));
                if (bowen > 1.0f)
                    ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f),
                                       "Above 1 is semi-desert: the ground heats the air far "
                                       "faster\nthan it moistens it, so relative humidity falls "
                                       "all day and\nthe condensation level runs away upward. "
                                       "Vegetated land in\nsummer is 0.3-0.6, and that is the "
                                       "difference between a\nclear sky and a cumulus deck -- not "
                                       "the time of day.\nRaise Moisture Availability below to "
                                       "move it.");
            }

            // ---- The column under the observer --------------------------------------------
            ImGui::SeparatorText("Column under the observer");
            const SushiEngine::Render::AtmosphereMirrorColumn* column = observer_column(mirror);
            if (column == nullptr)
            {
                ImGui::TextDisabled("No readback yet.");
            }
            else
            {
                ImGui::Text("Cloud base %.0f m, top %.0f m", column->surface[3], column->extent[0]);
                ImGui::Text("Rain %.2f mm/h   Wind %.1f E, %.1f N m/s", column->surface[0],
                            column->surface[1], column->surface[2]);

                // Why a clear column is clear. The three above say only *that* it is.
                const float humidity = column->extent[1];
                const float updraft = column->extent[2];
                const float lcl = column->extent[3];
                ImGui::Text("Surface RH %.0f%%   Peak updraft %.2f m/s", humidity * 100.0f,
                            updraft);
                if (lcl > 0.0f)
                    ImGui::Text("Condensation level %.0f m", lcl);
                else
                    ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.3f, 1.0f),
                                       "No condensation level: a surface parcel never reaches\n"
                                       "saturation anywhere in the 18 km domain, so this column\n"
                                       "cannot make cloud however long it is heated. Heating in\n"
                                       "fact lowers relative humidity and pushes it further out;\n"
                                       "what this needs is moisture, not time.");
                if (lcl > 0.0f && column->surface[3] <= 0.0f)
                    ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f),
                                       "There is a condensation level but nothing has reached it:\n"
                                       "the convection is not lifting parcels that far, and no\n"
                                       "level is humid enough for the subgrid closure to make\n"
                                       "cloud below it either. Look at RH against Cloud From RH\n"
                                       "in the profile below before reaching for more heating --\n"
                                       "heating alone lowers relative humidity.");
                else if (column->surface[3] > 0.0f && column->surface[3] < 100.0f)
                    ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f),
                                       "The cloud base is on the ground: this is fog, not a\n"
                                       "cumulus deck. The mixed layer is saturating from below\n"
                                       "rather than at its top -- it is too shallow, too wet, or\n"
                                       "not being mixed.");
                if (ImGui::BeginTable("bands", 5, ImGuiTableFlags_Borders |
                                                      ImGuiTableFlags_SizingStretchProp))
                {
                    ImGui::TableSetupColumn("Band");
                    ImGui::TableSetupColumn("Coverage");
                    ImGui::TableSetupColumn("Density");
                    ImGui::TableSetupColumn("Convective");
                    ImGui::TableSetupColumn("dT (C)");
                    ImGui::TableHeadersRow();
                    static const char* names[3] = {"Low", "Middle", "High"};
                    for (int band = 0; band < 3; ++band)
                    {
                        ImGui::TableNextRow();
                        ImGui::TableNextColumn();
                        ImGui::TextUnformatted(names[band]);
                        for (int field = 0; field < 4; ++field)
                        {
                            ImGui::TableNextColumn();
                            ImGui::Text("%.3f", column->bands[band][field]);
                        }
                    }
                    ImGui::EndTable();
                }
            }

            // ---- The observer column, unreduced -------------------------------------------
            //
            // Everything above this line is a vertical *reduction*, which is what gameplay asks
            // for and is exactly what a sky that refuses to make cloud has already destroyed the
            // evidence in: a column of zeros says only that there is no cloud. The nest reads
            // its centre column back level by level for this reason, and until now nothing in
            // the editor showed it — the diagnosis lived only in the headless probe, so seeing
            // it meant leaving the editor and rebuilding.
            //
            // The two columns to read together are RH and Cloud. Cloud is the subgrid fraction,
            // so it rises off zero at the authored critical humidity and not at 100 %: a level
            // reading 88 % RH and 20 % cloud is a scattered cumulus deck, and that difference is
            // the whole of what the closure added.
            ImGui::SeparatorText("Vertical profile under the observer");
            if (!mirror.valid() || mirror.profile == nullptr || mirror.profile_levels <= 0)
            {
                ImGui::TextDisabled("No readback yet.");
            }
            else
            {
                ImGui::Checkbox("Whole domain", &context.panel_state.meteorology.whole_domain);
                ImGui::SameLine();
                ImGui::TextDisabled("(off: the lowest 6 km, where the weather is)");

                // The cloudiest level and its height, which is the one comparison that separates
                // a cumulus field from fog and which no single row shows.
                const SushiEngine::Render::AtmosphereProfileLevel* cloudiest = nullptr;
                for (std::int32_t k = 0; k < mirror.profile_levels; ++k)
                    if (cloudiest == nullptr ||
                        mirror.profile[k].cloud_fraction > cloudiest->cloud_fraction)
                        cloudiest = &mirror.profile[k];
                if (cloudiest != nullptr && cloudiest->cloud_fraction > 0.0f)
                    ImGui::Text("Cloudiest level: %.0f %% at %.0f m",
                                double(cloudiest->cloud_fraction) * 100.0,
                                double(cloudiest->altitude_m));
                else
                    ImGui::TextDisabled("No level holds any cloud.");

                const ImGuiTableFlags flags = ImGuiTableFlags_Borders |
                                              ImGuiTableFlags_RowBg |
                                              ImGuiTableFlags_ScrollY |
                                              ImGuiTableFlags_SizingStretchProp;
                if (ImGui::BeginTable("profile", 9, flags, ImVec2(0.0f, 320.0f)))
                {
                    ImGui::TableSetupScrollFreeze(0, 1);
                    ImGui::TableSetupColumn("z (m)");
                    ImGui::TableSetupColumn("T (C)");
                    ImGui::TableSetupColumn("dTheta (K)");
                    ImGui::TableSetupColumn("RH (%)");
                    ImGui::TableSetupColumn("q_v (g/kg)");
                    ImGui::TableSetupColumn("q_c (g/kg)");
                    ImGui::TableSetupColumn("Cloud (%)");
                    ImGui::TableSetupColumn("w (m/s)");
                    ImGui::TableSetupColumn("Ext (1/m)");
                    ImGui::TableHeadersRow();

                    // Top down, because that is how a sounding is read and how the sky is seen.
                    for (std::int32_t k = mirror.profile_levels - 1; k >= 0; --k)
                    {
                        const SushiEngine::Render::AtmosphereProfileLevel& level =
                            mirror.profile[k];
                        if (!context.panel_state.meteorology.whole_domain && level.altitude_m > 6000.0f)
                            continue;
                        const float humidity =
                            level.saturation > 0.0f ? level.vapour / level.saturation : 0.0f;

                        ImGui::TableNextRow();
                        // Cloudy levels are tinted rather than merely reported, so a deck's
                        // extent is one glance instead of nine columns of arithmetic.
                        if (level.cloud_fraction > 0.0f)
                            ImGui::TableSetBgColor(
                                ImGuiTableBgTarget_RowBg0,
                                ImGui::GetColorU32(ImVec4(0.30f, 0.45f, 0.65f,
                                                          0.20f + 0.45f * level.cloud_fraction)));
                        ImGui::TableNextColumn();
                        ImGui::Text("%.0f", double(level.altitude_m));
                        ImGui::TableNextColumn();
                        ImGui::Text("%.1f", double(level.temperature_k) - 273.15);
                        ImGui::TableNextColumn();
                        ImGui::Text("%+.2f", double(level.theta_perturbation_k));
                        ImGui::TableNextColumn();
                        ImGui::Text("%.0f", double(humidity) * 100.0);
                        ImGui::TableNextColumn();
                        ImGui::Text("%.3f", double(level.vapour) * 1000.0);
                        ImGui::TableNextColumn();
                        ImGui::Text("%.4f", double(level.cloud_water) * 1000.0);
                        ImGui::TableNextColumn();
                        ImGui::Text("%.0f", double(level.cloud_fraction) * 100.0);
                        ImGui::TableNextColumn();
                        ImGui::Text("%+.3f", double(level.wind_up_mps));
                        ImGui::TableNextColumn();
                        ImGui::Text("%.4f", double(level.extinction));
                    }
                    ImGui::EndTable();
                }
            }

            // ---- From condensate to pixels -------------------------------------------------
            //
            // Everything above answers "is there cloud in the model". This answers the next
            // question, which is a different one and has its own ways of being no: the nest can
            // hold a solid deck and the sky still render empty, because between the two sit a
            // switch, a published field, a march shell and a bake. Each is a different fix, and
            // without this the only symptom any of them produces is a blue sky — which is the
            // same symptom as no condensate at all, and that ambiguity has cost this phase more
            // time than any physics in it.
            ImGui::SeparatorText("Render path");
            const SushiEngine::Render::WeatherField& field = environment.weather_field;
            const bool has_condensate = column != nullptr && column->extent[0] > 0.0f;

            if (!environment.clouds.enabled)
                ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.3f, 1.0f),
                                   "Clouds are switched off in Environment > Clouds, so no\n"
                                   "cloudscape is baked and nothing is marched at all.");
            else if (!field.valid())
                ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.3f, 1.0f),
                                   "No weather field reached the renderer: the mirror is not\n"
                                   "being transcribed, so the bake has nothing to read.");
            else if (!field.derives_genus)
                ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f),
                                   "The field is published but not marked as meteorology, so the\n"
                                   "march shell falls back to the authored deck stack instead of\n"
                                   "the span the nest measured.");
            else if (field.union_top_m <= field.union_base_m)
                ImGui::TextColored(has_condensate ? ImVec4(1.0f, 0.5f, 0.3f, 1.0f)
                                                  : ImVec4(0.6f, 0.6f, 0.6f, 1.0f),
                                   has_condensate
                                       ? "The nest holds condensate but the march shell has no\n"
                                         "height, so every baked texel samples the ground and the\n"
                                         "cloud is never crossed."
                                       : "No march shell yet -- no column holds condensate, so\n"
                                         "there is nothing for the shell to span.");
            else
                ImGui::TextColored(ImVec4(0.5f, 1.0f, 0.5f, 1.0f),
                                   "March shell spans %.0f m to %.0f m; the bake reads the nest's\n"
                                   "extinction across exactly that span.",
                                   double(field.union_base_m), double(field.union_top_m));

            ImGui::Text("Field %d x %d cells, shell %.0f-%.0f m", field.cells_x, field.cells_z,
                        double(field.union_base_m), double(field.union_top_m));
            if (has_condensate)
                ImGui::TextDisabled("Observer column holds cloud from %.0f m to %.0f m.",
                                    double(column->surface[3]), double(column->extent[0]));

            // ---- What the bake will publish for that cloud ---------------------------------
            //
            // The green line above proves the plumbing; these prove the *numbers*. Three
            // scalars decide whether a healthy deck survives to the screen, each with its own
            // way of being silently zero: the envelope the carve thresholds against (the
            // nest's own cloud fraction), the water amplitude (in-cloud extinction stated
            // against the authored "Overcast At" reference — a stale serialized reference
            // crushes it), and the medium scale (Clouds > Light absorption — a stale scene
            // value multiplies every sigma in the march). Shown here because a blue sky is
            // the shared symptom of all three, and telling them apart by eye is impossible.
            if (has_condensate && mirror.valid() && mirror.profile != nullptr &&
                mirror.profile_levels > 0)
            {
                const SushiEngine::Render::AtmosphereProfileLevel* densest = nullptr;
                for (std::int32_t k = 0; k < mirror.profile_levels; ++k)
                    if (densest == nullptr ||
                        mirror.profile[k].cloud_fraction > densest->cloud_fraction)
                        densest = &mirror.profile[k];
                if (densest != nullptr && densest->cloud_fraction > 0.01f)
                {
                    const SushiEngine::Render::AtmosphereParameters& physics =
                        environment.atmosphere_nest;
                    // The same formula CloudscapeCompilePass uploads as
                    // atmosphere_nest_params.w, so this preview and the GPU bake cannot
                    // disagree: sigma_ref = 3 * reference_lwc / (2 * rho_w * r_eff).
                    const float radius = physics.droplet_effective_radius > 1.0e-7f
                                             ? physics.droplet_effective_radius
                                             : 1.0e-7f;
                    const float reference_lwc = physics.coverage_reference_lwc > 1.0e-6f
                                                    ? physics.coverage_reference_lwc
                                                    : 1.0e-6f;
                    const float reference_extinction =
                        3.0f * reference_lwc / (2.0f * physics.water_density * radius);
                    const float fraction_floor =
                        densest->cloud_fraction > 0.05f ? densest->cloud_fraction : 0.05f;
                    const float in_cloud_extinction = densest->extinction / fraction_floor;
                    float water = in_cloud_extinction / reference_extinction;
                    water = water < 0.0f ? 0.0f : (water > 1.0f ? 1.0f : water);

                    ImGui::Text("Bake at %.0f m: envelope %.2f, water %.2f  "
                                "(in-cloud %.4f 1/m vs reference %.4f 1/m)",
                                double(densest->altitude_m), double(densest->cloud_fraction),
                                double(water), double(in_cloud_extinction),
                                double(reference_extinction));
                    // A rough full-deck optical depth through the march: density ~ water
                    // through the deck's own thickness at the march's extinction scale
                    // (cloud.frag: cloud_light.x * 0.006). Below ~0.3 the deck is a veil,
                    // below ~0.05 it is invisible.
                    const float thickness = column->extent[0] - column->surface[3];
                    const float optical_depth = water *
                                                environment.clouds.light_absorption * 0.006f *
                                                (thickness > 0.0f ? thickness : 0.0f);
                    ImGui::Text("Light absorption %.2f -> full-deck optical depth ~%.1f",
                                double(environment.clouds.light_absorption),
                                double(optical_depth));

                    if (water < 0.05f)
                        ImGui::TextColored(
                            ImVec4(1.0f, 0.5f, 0.3f, 1.0f),
                            "Water amplitude is ~zero: \"Overcast At\" (%.5f kg/m^3, likely a\n"
                            "stale serialized value) sits far above the in-cloud water, so the\n"
                            "deck bakes as nothing. Set it to 0.00040 and resave the scene.",
                            double(physics.coverage_reference_lwc));
                    if (environment.clouds.light_absorption < 0.05f)
                        ImGui::TextColored(
                            ImVec4(1.0f, 0.5f, 0.3f, 1.0f),
                            "Clouds > Light absorption is ~zero, which scales every march\n"
                            "sigma to nothing — no cloud can render at any water content.");
                }
            }

            // ---- Logging -------------------------------------------------------------------
            //
            // Sampled on the *nest's* clock rather than the wall clock, so a log line is a fixed
            // interval of simulated weather however fast or slow the sky is being animated --
            // which is what makes two runs at different time scales comparable.
            ImGui::SeparatorText("Log");
            MeteorologyLog& log = context.meteorology_log;
            ImGui::InputText("File", log.path, sizeof(log.path));
            ImGui::SliderFloat("Every", &log.interval_seconds, 1.0f, 3600.0f, "%.0f s simulated",
                               ImGuiSliderFlags_Logarithmic);
            if (ImGui::Checkbox("Logging", &log.enabled) && !log.enabled)
            {
                log.stream.close();
                log.header_written = false;
            }

            const bool force = ImGui::Button("Write one line now");
            if (column == nullptr)
            {
                ImGui::SameLine();
                ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.3f, 1.0f),
                                   "nothing to write -- see above");
            }
            if ((log.enabled || force) && column != nullptr)
            {
                const bool due = force || mirror.simulated_seconds >= log.next_at_simulated;
                if (due)
                {
                    if (!log.stream.is_open())
                        log.stream.open(log.path, std::ios::out | std::ios::app);
                    if (log.stream.is_open())
                    {
                        if (!log.header_written)
                        {
                            log.stream << "simulated_s,julian_date,sun_sine,cloud_base_m,"
                                          "cloud_top_m,surface_rh,peak_w,lcl_m,"
                                          "peak_cloud_fraction,peak_fraction_altitude_m,"
                                          "rain_mm_h,wind_e,wind_n,"
                                          "low_cov,low_den,low_conv,low_dT,"
                                          "mid_cov,mid_den,mid_conv,mid_dT,"
                                          "high_cov,high_den,high_conv,high_dT\n";
                            log.header_written = true;
                        }
                        log.stream << mirror.simulated_seconds << ','
                                   << (context.simulation != nullptr
                                           ? context.simulation->julian_date()
                                           : 0.0)
                                   << ',' << sine << ',' << column->surface[3] << ','
                                   << column->extent[0] << ',' << column->extent[1] << ','
                                   << column->extent[2] << ',' << column->extent[3];
                        // The cloudiest level and its height. Logged beside the reductions
                        // because they are the pair that distinguishes fog from a cumulus deck,
                        // and a log of the reductions alone could not: both report a cloud base.
                        float logged_fraction = 0.0f;
                        float logged_altitude = 0.0f;
                        for (std::int32_t k = 0;
                             mirror.profile != nullptr && k < mirror.profile_levels; ++k)
                            if (mirror.profile[k].cloud_fraction > logged_fraction)
                            {
                                logged_fraction = mirror.profile[k].cloud_fraction;
                                logged_altitude = mirror.profile[k].altitude_m;
                            }
                        log.stream << ',' << logged_fraction << ',' << logged_altitude << ','
                                   << column->surface[0] << ',' << column->surface[1] << ','
                                   << column->surface[2];
                        for (int band = 0; band < 3; ++band)
                            for (int field = 0; field < 4; ++field)
                                log.stream << ',' << column->bands[band][field];
                        log.stream << '\n';
                        log.stream.flush();
                    }
                    log.next_at_simulated =
                        mirror.simulated_seconds + double(log.interval_seconds);
                }
            }

            ImGui::End();
        }
    } // namespace Editor
} // namespace SushiEngine
