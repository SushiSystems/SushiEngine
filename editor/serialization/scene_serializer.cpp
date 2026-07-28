/**************************************************************************/
/* scene_serializer.cpp                                                   */
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

#include "scene_serializer.hpp"

#include <cstdio>
#include <fstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "effect_serializer.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        namespace
        {
            using nlohmann::json;
            using SushiEngine::Simulation::EntityId;
            using SushiEngine::Simulation::IWorldEditor;
            using SushiEngine::Simulation::NULL_ENTITY;

            json vec3_to_json(const SushiEngine::Vector3& v)
            {
                return json{{"x", v.x}, {"y", v.y}, {"z", v.z}};
            }

            SushiEngine::Vector3 vec3_from_json(const json& j)
            {
                SushiEngine::Vector3 v;
                v.x = j.value("x", SushiEngine::Scalar(0));
                v.y = j.value("y", SushiEngine::Scalar(0));
                v.z = j.value("z", SushiEngine::Scalar(0));
                return v;
            }

            json quaternion_to_json(const SushiEngine::Quaternion& q)
            {
                return json{{"x", q.x}, {"y", q.y}, {"z", q.z}, {"w", q.w}};
            }

            // The Environment fields an author edits in the Lighting/Environment panels
            // (sun, atmosphere, ground, clouds, stars, night lighting, ambient, exposure,
            // IBL). `bodies`/`sky_stars`/`dominant_*` are excluded: the ephemeris repopulates
            // those every frame from SceneSkyState and are not author state.
            json environment_to_json(const SushiEngine::Render::Environment& e)
            {
                json decks = json::array();
                for (int i = 0; i < SushiEngine::Render::CLOUD_MAX_DECKS; ++i)
                {
                    const SushiEngine::Render::CloudDeck& d = e.clouds.decks[i];
                    decks.push_back(json{{"enabled", d.enabled},
                                          {"genus", static_cast<std::uint32_t>(d.genus)},
                                          {"coverage_bias", d.coverage_bias},
                                          {"density_scale", d.density_scale}});
                }

                return json{
                    {"sun",
                     json{{"direction", vec3_to_json(e.sun.direction)},
                          {"color", vec3_to_json(e.sun.color)},
                          {"intensity", e.sun.intensity}}},
                    {"planet_surface_style", static_cast<std::uint32_t>(e.planet_surface_style)},
                    {"atmosphere",
                     json{{"enabled", e.atmosphere.enabled},
                          {"height", e.atmosphere.height},
                          {"rayleigh_coefficient", vec3_to_json(e.atmosphere.rayleigh_coefficient)},
                          {"mie_coefficient", e.atmosphere.mie_coefficient},
                          {"mie_anisotropy", e.atmosphere.mie_anisotropy},
                          {"rayleigh_scale_height", e.atmosphere.rayleigh_scale_height},
                          {"mie_scale_height", e.atmosphere.mie_scale_height}}},
                    {"surface",
                     json{{"ground_albedo", vec3_to_json(e.surface.ground_albedo)},
                          {"ocean_color", vec3_to_json(e.surface.ocean_color)},
                          {"roughness", e.surface.roughness}}},
                    {"clouds",
                     json{{"enabled", e.clouds.enabled},
                          {"light_absorption", e.clouds.light_absorption},
                          {"forward_scattering", e.clouds.forward_scattering},
                          {"powder_strength", e.clouds.powder_strength},
                          {"ambient_strength", e.clouds.ambient_strength},
                          {"ground_shadow_strength", e.clouds.ground_shadow_strength},
                          {"weather_scale", e.clouds.weather_scale},
                          {"evolution_rate", e.clouds.evolution_rate},
                          {"decks", decks}}},
                    // The regional nest's physics. Every field of AtmosphereParameters is here
                    // rather than a chosen few, because docs/slop/atmosphere_system.md §13's
                    // claim -- "every physical constant is serialized with the scene, and
                    // editable" -- is only true if a scene can actually carry all of them.
                    {"atmosphere_nest",
                     json{{"enabled", e.atmosphere_nest.enabled},
                          {"gas_constant_dry", e.atmosphere_nest.gas_constant_dry},
                          {"gas_constant_vapour", e.atmosphere_nest.gas_constant_vapour},
                          {"specific_heat_pressure", e.atmosphere_nest.specific_heat_pressure},
                          {"latent_heat_vaporization", e.atmosphere_nest.latent_heat_vaporization},
                          {"gravity", e.atmosphere_nest.gravity},
                          {"reference_pressure", e.atmosphere_nest.reference_pressure},
                          {"water_density", e.atmosphere_nest.water_density},
                          {"surface_temperature", e.atmosphere_nest.surface_temperature},
                          {"surface_pressure", e.atmosphere_nest.surface_pressure},
                          {"lapse_rate", e.atmosphere_nest.lapse_rate},
                          {"tropopause_altitude", e.atmosphere_nest.tropopause_altitude},
                          {"surface_humidity", e.atmosphere_nest.surface_humidity},
                          {"humidity_scale_height", e.atmosphere_nest.humidity_scale_height},
                          {"courant_target", e.atmosphere_nest.courant_target},
                          {"max_step_seconds", e.atmosphere_nest.max_step_seconds},
                          {"min_step_seconds", e.atmosphere_nest.min_step_seconds},
                          {"max_steps_per_frame", e.atmosphere_nest.max_steps_per_frame},
                          {"eddy_viscosity", e.atmosphere_nest.eddy_viscosity},
                          {"sponge_depth", e.atmosphere_nest.sponge_depth},
                          {"sponge_rate", e.atmosphere_nest.sponge_rate},
                          {"boundary_zone_cells", e.atmosphere_nest.boundary_zone_cells},
                          {"boundary_relaxation", e.atmosphere_nest.boundary_relaxation},
                          {"pressure_iterations", e.atmosphere_nest.pressure_iterations},
                          {"thermal_seed_amplitude", e.atmosphere_nest.thermal_seed_amplitude},
                          {"thermal_seed_length_m", e.atmosphere_nest.thermal_seed_length_m},
                          {"thermal_seed_period_s", e.atmosphere_nest.thermal_seed_period_s},
                          {"convective_velocity_scale", e.atmosphere_nest.convective_velocity_scale},
                          {"boundary_layer_depth_m", e.atmosphere_nest.boundary_layer_depth_m},
                          {"boundary_layer_velocity_scale",
                           e.atmosphere_nest.boundary_layer_velocity_scale},
                          {"cloud_critical_humidity", e.atmosphere_nest.cloud_critical_humidity},
                          {"autoconversion_rate", e.atmosphere_nest.autoconversion_rate},
                          {"autoconversion_threshold", e.atmosphere_nest.autoconversion_threshold},
                          {"accretion_rate", e.atmosphere_nest.accretion_rate},
                          {"accretion_exponent", e.atmosphere_nest.accretion_exponent},
                          {"rain_evaporation_rate", e.atmosphere_nest.rain_evaporation_rate},
                          {"fall_speed_coefficient", e.atmosphere_nest.fall_speed_coefficient},
                          {"fall_speed_exponent", e.atmosphere_nest.fall_speed_exponent},
                          {"droplet_effective_radius", e.atmosphere_nest.droplet_effective_radius},
                          {"latent_heat_fusion", e.atmosphere_nest.latent_heat_fusion},
                          {"freezing_temperature", e.atmosphere_nest.freezing_temperature},
                          {"glaciation_temperature", e.atmosphere_nest.glaciation_temperature},
                          {"snow_fall_speed_coefficient",
                           e.atmosphere_nest.snow_fall_speed_coefficient},
                          {"snow_fall_speed_exponent", e.atmosphere_nest.snow_fall_speed_exponent},
                          {"glaciated_autoconversion_factor",
                           e.atmosphere_nest.glaciated_autoconversion_factor},
                          {"ice_effective_radius", e.atmosphere_nest.ice_effective_radius},
                          {"coverage_reference_lwc", e.atmosphere_nest.coverage_reference_lwc},
                          {"solar_constant", e.atmosphere_nest.solar_constant},
                          {"clear_sky_transmittance", e.atmosphere_nest.clear_sky_transmittance},
                          {"surface_albedo", e.atmosphere_nest.surface_albedo},
                          {"surface_emissivity", e.atmosphere_nest.surface_emissivity},
                          {"surface_heat_capacity", e.atmosphere_nest.surface_heat_capacity},
                          {"surface_moisture_availability",
                           e.atmosphere_nest.surface_moisture_availability},
                          {"surface_exchange_coefficient",
                           e.atmosphere_nest.surface_exchange_coefficient},
                          {"surface_minimum_wind", e.atmosphere_nest.surface_minimum_wind}}},
                    {"stars",
                     json{{"enabled", e.stars.enabled},
                          {"brightness", e.stars.brightness},
                          {"density", e.stars.density}}},
                    {"night",
                     json{{"enabled", e.night.enabled},
                          {"reflected_intensity", e.night.reflected_intensity},
                          {"star_intensity", e.night.star_intensity}}},
                    {"ambient", vec3_to_json(e.ambient)},
                    {"exposure", e.exposure},
                    {"image_based_lighting", e.image_based_lighting},
                    {"ibl_intensity", e.ibl_intensity}};
            }

            // Applies the persisted fields onto `environment`, leaving every field the JSON
            // omits (including the ephemeris-owned fields never written above) at whatever
            // value the caller passed in.
            SushiEngine::Render::Environment environment_from_json(
                const json& j, SushiEngine::Render::Environment environment)
            {
                if (!j.is_object())
                    return environment;

                if (j.contains("sun") && j["sun"].is_object())
                {
                    const json& s = j["sun"];
                    if (s.contains("direction"))
                        environment.sun.direction = vec3_from_json(s["direction"]);
                    if (s.contains("color"))
                        environment.sun.color = vec3_from_json(s["color"]);
                    environment.sun.intensity = s.value("intensity", environment.sun.intensity);
                }
                environment.planet_surface_style = static_cast<SushiEngine::Render::SurfaceStyle>(
                    j.value("planet_surface_style",
                            static_cast<std::uint32_t>(environment.planet_surface_style)));
                if (j.contains("atmosphere") && j["atmosphere"].is_object())
                {
                    const json& a = j["atmosphere"];
                    environment.atmosphere.enabled = a.value("enabled", environment.atmosphere.enabled);
                    environment.atmosphere.height = a.value("height", environment.atmosphere.height);
                    if (a.contains("rayleigh_coefficient"))
                        environment.atmosphere.rayleigh_coefficient =
                            vec3_from_json(a["rayleigh_coefficient"]);
                    environment.atmosphere.mie_coefficient =
                        a.value("mie_coefficient", environment.atmosphere.mie_coefficient);
                    environment.atmosphere.mie_anisotropy =
                        a.value("mie_anisotropy", environment.atmosphere.mie_anisotropy);
                    environment.atmosphere.rayleigh_scale_height =
                        a.value("rayleigh_scale_height", environment.atmosphere.rayleigh_scale_height);
                    environment.atmosphere.mie_scale_height =
                        a.value("mie_scale_height", environment.atmosphere.mie_scale_height);
                }
                if (j.contains("surface") && j["surface"].is_object())
                {
                    const json& s = j["surface"];
                    if (s.contains("ground_albedo"))
                        environment.surface.ground_albedo = vec3_from_json(s["ground_albedo"]);
                    if (s.contains("ocean_color"))
                        environment.surface.ocean_color = vec3_from_json(s["ocean_color"]);
                    environment.surface.roughness = s.value("roughness", environment.surface.roughness);
                }
                if (j.contains("atmosphere_nest") && j["atmosphere_nest"].is_object())
                {
                    const json& a = j["atmosphere_nest"];
                    SushiEngine::Render::AtmosphereParameters& n = environment.atmosphere_nest;
                    // Every read defaults to the value already in the struct, so a scene written
                    // before a parameter existed loads with that parameter's own default rather
                    // than with a zero the physics would divide by.
                    n.enabled = a.value("enabled", n.enabled);
                    n.gas_constant_dry = a.value("gas_constant_dry", n.gas_constant_dry);
                    n.gas_constant_vapour = a.value("gas_constant_vapour", n.gas_constant_vapour);
                    n.specific_heat_pressure =
                        a.value("specific_heat_pressure", n.specific_heat_pressure);
                    n.latent_heat_vaporization =
                        a.value("latent_heat_vaporization", n.latent_heat_vaporization);
                    n.gravity = a.value("gravity", n.gravity);
                    n.reference_pressure = a.value("reference_pressure", n.reference_pressure);
                    n.water_density = a.value("water_density", n.water_density);
                    n.surface_temperature = a.value("surface_temperature", n.surface_temperature);
                    n.surface_pressure = a.value("surface_pressure", n.surface_pressure);
                    n.lapse_rate = a.value("lapse_rate", n.lapse_rate);
                    n.tropopause_altitude = a.value("tropopause_altitude", n.tropopause_altitude);
                    n.surface_humidity = a.value("surface_humidity", n.surface_humidity);
                    n.humidity_scale_height =
                        a.value("humidity_scale_height", n.humidity_scale_height);
                    n.courant_target = a.value("courant_target", n.courant_target);
                    n.max_step_seconds = a.value("max_step_seconds", n.max_step_seconds);
                    n.min_step_seconds = a.value("min_step_seconds", n.min_step_seconds);
                    n.max_steps_per_frame = a.value("max_steps_per_frame", n.max_steps_per_frame);
                    n.eddy_viscosity = a.value("eddy_viscosity", n.eddy_viscosity);
                    n.sponge_depth = a.value("sponge_depth", n.sponge_depth);
                    n.sponge_rate = a.value("sponge_rate", n.sponge_rate);
                    n.boundary_zone_cells = a.value("boundary_zone_cells", n.boundary_zone_cells);
                    n.boundary_relaxation = a.value("boundary_relaxation", n.boundary_relaxation);
                    n.pressure_iterations = a.value("pressure_iterations", n.pressure_iterations);
                    n.thermal_seed_amplitude =
                        a.value("thermal_seed_amplitude", n.thermal_seed_amplitude);
                    // Absent from every scene written before 2026-07-28, and `value` leaving the
                    // default in place is exactly right for those: the seed used to be white, and
                    // the default is the correlation it should have had.
                    n.thermal_seed_length_m =
                        a.value("thermal_seed_length_m", n.thermal_seed_length_m);
                    n.thermal_seed_period_s =
                        a.value("thermal_seed_period_s", n.thermal_seed_period_s);
                    n.convective_velocity_scale =
                        a.value("convective_velocity_scale", n.convective_velocity_scale);
                    n.boundary_layer_depth_m =
                        a.value("boundary_layer_depth_m", n.boundary_layer_depth_m);
                    n.boundary_layer_velocity_scale =
                        a.value("boundary_layer_velocity_scale",
                                n.boundary_layer_velocity_scale);
                    n.cloud_critical_humidity =
                        a.value("cloud_critical_humidity", n.cloud_critical_humidity);
                    n.autoconversion_rate = a.value("autoconversion_rate", n.autoconversion_rate);
                    n.autoconversion_threshold =
                        a.value("autoconversion_threshold", n.autoconversion_threshold);
                    n.accretion_rate = a.value("accretion_rate", n.accretion_rate);
                    n.accretion_exponent = a.value("accretion_exponent", n.accretion_exponent);
                    n.rain_evaporation_rate =
                        a.value("rain_evaporation_rate", n.rain_evaporation_rate);
                    n.fall_speed_coefficient =
                        a.value("fall_speed_coefficient", n.fall_speed_coefficient);
                    n.fall_speed_exponent = a.value("fall_speed_exponent", n.fall_speed_exponent);
                    n.droplet_effective_radius =
                        a.value("droplet_effective_radius", n.droplet_effective_radius);
                    n.latent_heat_fusion = a.value("latent_heat_fusion", n.latent_heat_fusion);
                    n.freezing_temperature =
                        a.value("freezing_temperature", n.freezing_temperature);
                    n.glaciation_temperature =
                        a.value("glaciation_temperature", n.glaciation_temperature);
                    n.snow_fall_speed_coefficient =
                        a.value("snow_fall_speed_coefficient", n.snow_fall_speed_coefficient);
                    n.snow_fall_speed_exponent =
                        a.value("snow_fall_speed_exponent", n.snow_fall_speed_exponent);
                    n.glaciated_autoconversion_factor =
                        a.value("glaciated_autoconversion_factor",
                                n.glaciated_autoconversion_factor);
                    n.ice_effective_radius =
                        a.value("ice_effective_radius", n.ice_effective_radius);
                    n.coverage_reference_lwc =
                        a.value("coverage_reference_lwc", n.coverage_reference_lwc);
                    // `surface_sensible_flux` / `surface_latent_flux` / `surface_night_flux`
                    // were the prescribed fluxes Phase B3 replaced with a solved balance. A
                    // scene written before it carries them and they are simply not read: what
                    // they encoded — how hot and how wet the ground is — is now the albedo,
                    // moisture availability and heat capacity below, and there is no honest
                    // mapping from a pair of peak fluxes back onto them.
                    n.solar_constant = a.value("solar_constant", n.solar_constant);
                    n.clear_sky_transmittance =
                        a.value("clear_sky_transmittance", n.clear_sky_transmittance);
                    n.surface_albedo = a.value("surface_albedo", n.surface_albedo);
                    n.surface_emissivity = a.value("surface_emissivity", n.surface_emissivity);
                    n.surface_heat_capacity =
                        a.value("surface_heat_capacity", n.surface_heat_capacity);
                    n.surface_moisture_availability =
                        a.value("surface_moisture_availability", n.surface_moisture_availability);
                    n.surface_exchange_coefficient =
                        a.value("surface_exchange_coefficient", n.surface_exchange_coefficient);
                    n.surface_minimum_wind =
                        a.value("surface_minimum_wind", n.surface_minimum_wind);
                }
                if (j.contains("clouds") && j["clouds"].is_object())
                {
                    const json& c = j["clouds"];
                    environment.clouds.enabled = c.value("enabled", environment.clouds.enabled);
                    environment.clouds.light_absorption =
                        c.value("light_absorption", environment.clouds.light_absorption);
                    environment.clouds.forward_scattering =
                        c.value("forward_scattering", environment.clouds.forward_scattering);
                    environment.clouds.powder_strength =
                        c.value("powder_strength", environment.clouds.powder_strength);
                    environment.clouds.ambient_strength =
                        c.value("ambient_strength", environment.clouds.ambient_strength);
                    environment.clouds.ground_shadow_strength =
                        c.value("ground_shadow_strength", environment.clouds.ground_shadow_strength);
                    environment.clouds.weather_scale =
                        c.value("weather_scale", environment.clouds.weather_scale);
                    environment.clouds.evolution_rate =
                        c.value("evolution_rate", environment.clouds.evolution_rate);
                    if (c.contains("decks") && c["decks"].is_array())
                    {
                        const json& decks = c["decks"];
                        for (int i = 0; i < SushiEngine::Render::CLOUD_MAX_DECKS &&
                                        static_cast<std::size_t>(i) < decks.size(); ++i)
                        {
                            const json& d = decks[static_cast<std::size_t>(i)];
                            SushiEngine::Render::CloudDeck& deck = environment.clouds.decks[i];
                            deck.enabled = d.value("enabled", deck.enabled);
                            deck.genus = static_cast<SushiEngine::Render::CloudGenus>(
                                d.value("genus", static_cast<std::uint32_t>(deck.genus)));
                            deck.coverage_bias = d.value("coverage_bias", deck.coverage_bias);
                            deck.density_scale = d.value("density_scale", deck.density_scale);
                        }
                    }
                }
                if (j.contains("stars") && j["stars"].is_object())
                {
                    const json& s = j["stars"];
                    environment.stars.enabled = s.value("enabled", environment.stars.enabled);
                    environment.stars.brightness = s.value("brightness", environment.stars.brightness);
                    environment.stars.density = s.value("density", environment.stars.density);
                }
                if (j.contains("night") && j["night"].is_object())
                {
                    const json& n = j["night"];
                    environment.night.enabled = n.value("enabled", environment.night.enabled);
                    environment.night.reflected_intensity =
                        n.value("reflected_intensity", environment.night.reflected_intensity);
                    environment.night.star_intensity =
                        n.value("star_intensity", environment.night.star_intensity);
                }
                if (j.contains("ambient"))
                    environment.ambient = vec3_from_json(j["ambient"]);
                environment.exposure = j.value("exposure", environment.exposure);
                environment.image_based_lighting =
                    j.value("image_based_lighting", environment.image_based_lighting);
                environment.ibl_intensity = j.value("ibl_intensity", environment.ibl_intensity);
                return environment;
            }

            json sky_to_json(const SceneSkyState& sky)
            {
                return json{{"enabled", sky.enabled},
                            {"year", sky.date.year},
                            {"month", sky.date.month},
                            {"day", sky.date.day},
                            {"hour", sky.date.hour},
                            {"minute", sky.date.minute},
                            {"second", sky.date.second},
                            {"latitude_degrees", sky.latitude_degrees},
                            {"longitude_degrees", sky.longitude_degrees},
                            {"astronomical_sun", sky.astronomical_sun},
                            {"animate", sky.animate},
                            {"days_per_second", sky.days_per_second},
                            {"accumulated_days", sky.accumulated_days}};
            }

            SceneSkyState sky_from_json(const json& j, SceneSkyState sky)
            {
                if (!j.is_object())
                    return sky;
                sky.enabled = j.value("enabled", sky.enabled);
                sky.date.year = j.value("year", sky.date.year);
                sky.date.month = j.value("month", sky.date.month);
                sky.date.day = j.value("day", sky.date.day);
                sky.date.hour = j.value("hour", sky.date.hour);
                sky.date.minute = j.value("minute", sky.date.minute);
                sky.date.second = j.value("second", sky.date.second);
                sky.latitude_degrees = j.value("latitude_degrees", sky.latitude_degrees);
                sky.longitude_degrees = j.value("longitude_degrees", sky.longitude_degrees);
                sky.astronomical_sun = j.value("astronomical_sun", sky.astronomical_sun);
                sky.animate = j.value("animate", sky.animate);
                sky.days_per_second = j.value("days_per_second", sky.days_per_second);
                sky.accumulated_days = j.value("accumulated_days", sky.accumulated_days);
                return sky;
            }

            // W4's procedural weather state (docs/slop/weather_and_clouds.md §5): only T1
            // (the synoptic system list, its RNG, and its clock) is captured. T2's regional
            // grid is deliberately not serialized cell-by-cell -- it reseeds deterministically
            // from the restored synoptic state and the current observer on the next tick, which
            // resumes a visually consistent sky rather than the exact bit-pattern of transient
            // grid cells. The acceptance bar this phase must meet is tick-to-tick replay
            // determinism (see test_weather_determinism.cpp), not save/load byte-fidelity of
            // internal simulation state, so this is a scoped, named simplification rather than
            // a silent gap.
            json weather_to_json(IWorldEditor& world)
            {
                json j;
                j["procedural_enabled"] = world.procedural_weather_enabled();
                SushiEngine::Simulation::IWeatherAuthoring* weather = world.weather_authoring();
                if (weather == nullptr)
                    return j;

                const SushiEngine::Simulation::SynopticState& state = weather->synoptic().state();
                j["rng_s0"] = state.rng.s0;
                j["rng_s1"] = state.rng.s1;
                j["next_system_id"] = state.next_system_id;
                j["elapsed_seconds"] = state.elapsed_seconds;
                j["seconds_to_next_genesis"] = state.seconds_to_next_genesis;

                json systems = json::array();
                for (int i = 0; i < state.system_count; ++i)
                {
                    const SushiEngine::Simulation::PressureSystem& s = state.systems[i];
                    systems.push_back(json{
                        {"id", s.id},
                        {"is_low", s.is_low},
                        {"phase", static_cast<std::uint32_t>(s.phase)},
                        {"age_seconds", s.age_seconds},
                        {"deepen_seconds", s.deepen_seconds},
                        {"mature_seconds", s.mature_seconds},
                        {"fill_seconds", s.fill_seconds},
                        {"center_latitude_radians", s.center_latitude_radians},
                        {"center_longitude_radians", s.center_longitude_radians},
                        {"heading_radians", s.heading_radians},
                        {"curvature_radians_per_second", s.curvature_radians_per_second},
                        {"speed_mps", s.speed_mps},
                        {"central_anomaly_hpa", s.central_anomaly_hpa},
                        {"radius_major_m", s.radius_major_m},
                        {"radius_minor_m", s.radius_minor_m},
                        {"orientation_radians", s.orientation_radians}});
                }
                j["systems"] = systems;
                return j;
            }

            void weather_from_json(const json& j, IWorldEditor& world)
            {
                if (!j.is_object())
                    return;
                const bool enabled = j.value("procedural_enabled", false);
                world.set_procedural_weather_enabled(enabled);
                if (!enabled)
                    return;
                SushiEngine::Simulation::IWeatherAuthoring* weather = world.weather_authoring();
                if (weather == nullptr)
                    return;

                SushiEngine::Simulation::SynopticState state;
                state.rng.s0 = j.value("rng_s0", state.rng.s0);
                state.rng.s1 = j.value("rng_s1", state.rng.s1);
                state.next_system_id = j.value("next_system_id", state.next_system_id);
                state.elapsed_seconds = j.value("elapsed_seconds", state.elapsed_seconds);
                state.seconds_to_next_genesis =
                    j.value("seconds_to_next_genesis", state.seconds_to_next_genesis);

                if (j.contains("systems") && j["systems"].is_array())
                {
                    for (const json& sj : j["systems"])
                    {
                        if (state.system_count >= SushiEngine::Simulation::MAX_SYNOPTIC_SYSTEMS)
                            break;
                        SushiEngine::Simulation::PressureSystem system;
                        system.id = sj.value("id", system.id);
                        system.is_low = sj.value("is_low", system.is_low);
                        system.phase = static_cast<SushiEngine::Simulation::PressureSystemPhase>(
                            sj.value("phase", static_cast<std::uint32_t>(system.phase)));
                        system.age_seconds = sj.value("age_seconds", system.age_seconds);
                        system.deepen_seconds = sj.value("deepen_seconds", system.deepen_seconds);
                        system.mature_seconds = sj.value("mature_seconds", system.mature_seconds);
                        system.fill_seconds = sj.value("fill_seconds", system.fill_seconds);
                        system.center_latitude_radians =
                            sj.value("center_latitude_radians", system.center_latitude_radians);
                        system.center_longitude_radians =
                            sj.value("center_longitude_radians", system.center_longitude_radians);
                        system.heading_radians = sj.value("heading_radians", system.heading_radians);
                        system.curvature_radians_per_second = sj.value(
                            "curvature_radians_per_second", system.curvature_radians_per_second);
                        system.speed_mps = sj.value("speed_mps", system.speed_mps);
                        system.central_anomaly_hpa =
                            sj.value("central_anomaly_hpa", system.central_anomaly_hpa);
                        system.radius_major_m = sj.value("radius_major_m", system.radius_major_m);
                        system.radius_minor_m = sj.value("radius_minor_m", system.radius_minor_m);
                        system.orientation_radians =
                            sj.value("orientation_radians", system.orientation_radians);
                        state.systems[state.system_count++] = system;
                    }
                }
                weather->synoptic().set_state(state, world.environment().planet.mean_radius());
            }

            SushiEngine::Quaternion quaternion_from_json(const json& j)
            {
                SushiEngine::Quaternion q;
                q.x = j.value("x", SushiEngine::Scalar(0));
                q.y = j.value("y", SushiEngine::Scalar(0));
                q.z = j.value("z", SushiEngine::Scalar(0));
                q.w = j.value("w", SushiEngine::Scalar(1));
                return q;
            }

            json ui_to_json(const SushiEngine::Simulation::UIElementParams& p)
            {
                return json{{"kind", static_cast<std::uint32_t>(p.kind)},
                            {"anchor_min", json{{"x", p.anchor_min_x}, {"y", p.anchor_min_y}}},
                            {"anchor_max", json{{"x", p.anchor_max_x}, {"y", p.anchor_max_y}}},
                            {"pivot", json{{"x", p.pivot_x}, {"y", p.pivot_y}}},
                            {"position", json{{"x", p.position_x}, {"y", p.position_y}}},
                            {"size", json{{"x", p.size_x}, {"y", p.size_y}}},
                            {"color", vec3_to_json(p.color)},
                            {"alpha", p.alpha},
                            {"font_size", p.font_size},
                            {"text", std::string(p.text)}};
            }

            SushiEngine::Simulation::UIElementParams ui_from_json(const json& j)
            {
                SushiEngine::Simulation::UIElementParams p;
                p.kind = static_cast<SushiEngine::Simulation::UIElementKind>(
                    j.value("kind", static_cast<std::uint32_t>(p.kind)));
                if (j.contains("anchor_min"))
                {
                    p.anchor_min_x = j["anchor_min"].value("x", p.anchor_min_x);
                    p.anchor_min_y = j["anchor_min"].value("y", p.anchor_min_y);
                }
                if (j.contains("anchor_max"))
                {
                    p.anchor_max_x = j["anchor_max"].value("x", p.anchor_max_x);
                    p.anchor_max_y = j["anchor_max"].value("y", p.anchor_max_y);
                }
                if (j.contains("pivot"))
                {
                    p.pivot_x = j["pivot"].value("x", p.pivot_x);
                    p.pivot_y = j["pivot"].value("y", p.pivot_y);
                }
                if (j.contains("position"))
                {
                    p.position_x = j["position"].value("x", p.position_x);
                    p.position_y = j["position"].value("y", p.position_y);
                }
                if (j.contains("size"))
                {
                    p.size_x = j["size"].value("x", p.size_x);
                    p.size_y = j["size"].value("y", p.size_y);
                }
                if (j.contains("color"))
                    p.color = vec3_from_json(j["color"]);
                p.alpha = j.value("alpha", p.alpha);
                p.font_size = j.value("font_size", p.font_size);
                const std::string text = j.value("text", std::string{});
                std::snprintf(p.text, sizeof(p.text), "%s", text.c_str());
                return p;
            }

            json script_to_json(const SushiEngine::Simulation::ScriptComponent& script)
            {
                json fields = json::array();
                for (const SushiEngine::Simulation::ScriptField& field : script.fields)
                    fields.push_back(json{{"name", field.name},
                                          {"kind", static_cast<std::uint32_t>(field.kind)},
                                          {"number", field.number},
                                          {"flag", field.flag},
                                          {"vector", vec3_to_json(field.vector)},
                                          {"text", field.text}});
                return json{{"type_name", script.type_name}, {"fields", std::move(fields)}};
            }

            SushiEngine::Simulation::ScriptComponent script_from_json(const json& j)
            {
                SushiEngine::Simulation::ScriptComponent script;
                script.type_name = j.value("type_name", std::string{});
                if (j.contains("fields") && j["fields"].is_array())
                    for (const json& f : j["fields"])
                    {
                        SushiEngine::Simulation::ScriptField field;
                        field.name = f.value("name", std::string{});
                        field.kind = static_cast<SushiEngine::Simulation::ScriptFieldKind>(
                            f.value("kind", static_cast<std::uint32_t>(field.kind)));
                        field.number = f.value("number", field.number);
                        field.flag = f.value("flag", field.flag);
                        if (f.contains("vector"))
                            field.vector = vec3_from_json(f["vector"]);
                        field.text = f.value("text", std::string{});
                        script.fields.push_back(field);
                    }
                return script;
            }
        } // namespace

        json capture_scene(IWorldEditor& world)
        {
            const std::vector<EntityId> ids = world.entities();
            std::unordered_map<EntityId, int> index_of;
            for (std::size_t i = 0; i < ids.size(); ++i)
                index_of.emplace(ids[i], static_cast<int>(i));

            json root = json::array();
            for (const EntityId id : ids)
            {
                json entry;
                entry["name"] = world.name(id);
                entry["visible"] = world.visible(id);
                const EntityId parent_id = world.parent(id);
                entry["parent"] = parent_id == NULL_ENTITY ? -1 : index_of.at(parent_id);

                const auto transform = world.transform(id);
                entry["position"] = vec3_to_json(transform.position);
                entry["rotation"] = quaternion_to_json(transform.rotation);
                entry["scale"] = vec3_to_json(transform.scale);

                const bool is_camera = world.is_camera(id);
                entry["is_camera"] = is_camera;
                if (is_camera)
                {
                    const auto params = world.camera_params(id);
                    entry["camera"] = json{{"vertical_fov_radians", params.vertical_fov_radians},
                                           {"near_plane", params.near_plane},
                                           {"far_plane", params.far_plane},
                                           {"display_index", params.display_index},
                                           {"priority", params.priority},
                                           {"active", params.active}};
                }

                // Not mutually exclusive with camera, so it is captured independently
                // rather than in the if/else above (a camera can also carry a Renderer).
                const bool has_renderer = world.has_renderer(id);
                entry["has_renderer"] = has_renderer;
                if (has_renderer)
                    entry["color"] = vec3_to_json(world.color(id));

                // Not mutually exclusive with camera/renderer, so it is its own field
                // rather than sharing the if/else above.
                const bool has_physics_body = world.has_physics_body(id);
                entry["has_physics_body"] = has_physics_body;
                if (has_physics_body)
                {
                    const auto params = world.physics_body_params(id);
                    entry["physics_body"] = json{{"inv_mass", params.inv_mass},
                                                 {"inv_inertia", vec3_to_json(params.inv_inertia)},
                                                 {"drag_coefficient", params.drag_coefficient}};
                }

                // Not mutually exclusive with any of the above, so it is its own field
                // pair too.
                const bool has_cloth = world.has_cloth(id);
                entry["has_cloth"] = has_cloth;
                if (has_cloth)
                {
                    const auto params = world.cloth_params(id);
                    entry["cloth"] = json{{"rows", params.rows},
                                          {"cols", params.cols},
                                          {"spacing", params.spacing},
                                          {"compliance", params.compliance}};
                }

                const bool has_particle_emitter = world.has_particle_emitter(id);
                entry["has_particle_emitter"] = has_particle_emitter;
                if (has_particle_emitter)
                {
                    const auto params = world.particle_emitter_params(id);
                    // The effect goes with the entity, not as an index into a library: it is the
                    // component's own data, so a scene that reloads without it would come back
                    // with emitters that emit nothing.
                    entry["particle_emitter"] =
                        json{{"seed", params.seed},
                             {"playing", params.playing},
                             {"source", capture_effect(world.particle_effect_source(id))}};
                }

                const bool has_audio_emitter = world.has_audio_emitter(id);
                entry["has_audio_emitter"] = has_audio_emitter;
                if (has_audio_emitter)
                {
                    const auto p = world.audio_emitter_params(id);
                    entry["audio_emitter"] = json{{"sound", p.sound},
                                                  {"gain", p.gain},
                                                  {"priority", p.priority},
                                                  {"bus", p.bus},
                                                  {"min_distance", p.min_distance},
                                                  {"max_distance", p.max_distance},
                                                  {"distance_model", p.distance_model},
                                                  {"rolloff", p.rolloff},
                                                  {"doppler_scale", p.doppler_scale},
                                                  {"reverb_send", p.reverb_send},
                                                  {"spatial", p.spatial},
                                                  {"playing", p.playing},
                                                  {"looping", p.looping}};
                }

                const bool has_reverb_zone = world.has_reverb_zone(id);
                entry["has_reverb_zone"] = has_reverb_zone;
                if (has_reverb_zone)
                {
                    const auto p = world.reverb_zone_params(id);
                    entry["reverb_zone"] = json{{"half_extents", vec3_to_json(p.half_extents)},
                                                {"room", p.room},
                                                {"room_hf", p.room_hf},
                                                {"decay_time", p.decay_time},
                                                {"decay_hf_ratio", p.decay_hf_ratio},
                                                {"reflections", p.reflections},
                                                {"reverb", p.reverb},
                                                {"diffusion", p.diffusion},
                                                {"density", p.density},
                                                {"wet_dry_mix", p.wet_dry_mix},
                                                {"send", p.send},
                                                {"priority", p.priority}};
                }

                const bool has_audio_listener = world.has_audio_listener(id);
                entry["has_audio_listener"] = has_audio_listener;
                if (has_audio_listener)
                {
                    const auto p = world.audio_listener_params(id);
                    entry["audio_listener"] = json{{"gain", p.gain}, {"active", p.active}};
                }

                // Not mutually exclusive with any of the above, so it is its own field
                // pair too.
                const bool has_shape = world.has_shape(id);
                entry["has_shape"] = has_shape;
                if (has_shape)
                {
                    const auto params = world.shape_params(id);
                    entry["shape"] = json{{"kind", static_cast<std::uint32_t>(params.kind)},
                                          {"params", vec3_to_json(params.params)}};
                }

                const bool has_collider = world.has_collider(id);
                entry["has_collider"] = has_collider;
                if (has_collider)
                {
                    const auto params = world.collider_params(id);
                    entry["collider"] = json{{"kind", static_cast<std::uint32_t>(params.kind)},
                                             {"params", vec3_to_json(params.params)}};
                }

                const bool surface_anchored = world.surface_anchored(id);
                entry["surface_anchored"] = surface_anchored;
                if (surface_anchored)
                    entry["surface_local_orientation"] =
                        quaternion_to_json(world.surface_local_orientation(id));

                // The reference frame the transform is authored in (body + mode). Only the
                // descriptor is persisted; the frame-local pose is derived from the scene
                // Transform (already stored above) and the descriptor on load. A body of -1
                // (the scene root, default) is omitted to keep plain scenes clean.
                const SushiEngine::Simulation::EntityFrame frame = world.entity_frame(id);
                if (frame.reference_body >= 0)
                {
                    entry["reference_body"] = frame.reference_body;
                    entry["frame_mode"] = static_cast<int>(frame.mode);
                }

                const bool has_ui = world.has_ui(id);
                entry["has_ui"] = has_ui;
                if (has_ui)
                    entry["ui"] = ui_to_json(world.ui_params(id));

                const std::vector<std::string> scripts = world.script_components(id);
                if (!scripts.empty())
                {
                    json script_array = json::array();
                    for (const std::string& type_name : scripts)
                        script_array.push_back(script_to_json(world.script_component(id, type_name)));
                    entry["scripts"] = std::move(script_array);
                }

                root.push_back(std::move(entry));
            }

            return root;
        }

        void apply_scene(IWorldEditor& world, const json& root)
        {
            // Replace the world wholesale: clear every existing entity before
            // recreating the file's, so a load is never a merge with the prior scene.
            for (const EntityId id : world.entities())
                world.destroy(id);

            std::vector<EntityId> created;
            created.reserve(root.size());
            for (const auto& entry : root)
            {
                const std::string name = entry.value("name", std::string("Entity"));
                const bool is_camera = entry.value("is_camera", false);
                const EntityId id = is_camera ? world.create_camera(name) : world.create(name);
                created.push_back(id);

                SushiEngine::Simulation::EntityTransform transform;
                if (entry.contains("position"))
                    transform.position = vec3_from_json(entry["position"]);
                if (entry.contains("rotation"))
                    transform.rotation = quaternion_from_json(entry["rotation"]);
                if (entry.contains("scale"))
                    transform.scale = vec3_from_json(entry["scale"]);
                world.set_transform(id, transform);
                world.set_visible(id, entry.value("visible", true));

                if (is_camera && entry.contains("camera"))
                {
                    const json& c = entry["camera"];
                    SushiEngine::Simulation::CameraParams params;
                    params.vertical_fov_radians =
                        c.value("vertical_fov_radians", params.vertical_fov_radians);
                    params.near_plane = c.value("near_plane", params.near_plane);
                    params.far_plane = c.value("far_plane", params.far_plane);
                    params.display_index = c.value("display_index", params.display_index);
                    params.priority = c.value("priority", params.priority);
                    params.active = c.value("active", params.active);
                    world.set_camera_params(id, params);
                }

                // `create`/`create_camera` both attach a Renderer by default, so an
                // explicit false must detach it; true is a no-op re-attach.
                const bool has_renderer = entry.value("has_renderer", entry.contains("color"));
                world.set_has_renderer(id, has_renderer);
                if (has_renderer && entry.contains("color"))
                    world.set_color(id, vec3_from_json(entry["color"]));

                if (entry.value("has_physics_body", false))
                {
                    world.set_has_physics_body(id, true);
                    if (entry.contains("physics_body"))
                    {
                        const json& p = entry["physics_body"];
                        SushiEngine::Simulation::PhysicsBodyParams params;
                        params.inv_mass = p.value("inv_mass", params.inv_mass);
                        if (p.contains("inv_inertia"))
                            params.inv_inertia = vec3_from_json(p["inv_inertia"]);
                        params.drag_coefficient =
                            p.value("drag_coefficient", params.drag_coefficient);
                        world.set_physics_body_params(id, params);
                    }
                }

                if (entry.value("has_cloth", false))
                {
                    world.set_has_cloth(id, true);
                    if (entry.contains("cloth"))
                    {
                        const json& c = entry["cloth"];
                        SushiEngine::Simulation::ClothParams params;
                        params.rows = c.value("rows", params.rows);
                        params.cols = c.value("cols", params.cols);
                        params.spacing = c.value("spacing", params.spacing);
                        params.compliance = c.value("compliance", params.compliance);
                        world.set_cloth_params(id, params);
                    }
                }

                if (entry.value("has_particle_emitter", false))
                {
                    world.set_has_particle_emitter(id, true);
                    if (entry.contains("particle_emitter"))
                    {
                        const json& p = entry["particle_emitter"];
                        SushiEngine::Simulation::ParticleEmitterParams params;
                        // An older file's "effect" index is ignored: the effect it named was a
                        // library entry that no longer exists, and "source" carries the real thing.
                        params.seed = p.value("seed", params.seed);
                        params.playing = p.value("playing", params.playing);
                        world.set_particle_emitter_params(id, params);
                        // Absent in files written before the effect moved onto the component; the
                        // emitter then keeps the default it was seeded with rather than failing.
                        if (p.contains("source"))
                        {
                            SushiEngine::Vfx::ParticleEffect source;
                            if (apply_effect(p["source"], source))
                                world.set_particle_effect_source(id, source);
                        }
                    }
                }

                if (entry.value("has_audio_emitter", false))
                {
                    world.set_has_audio_emitter(id, true);
                    if (entry.contains("audio_emitter"))
                    {
                        const json& a = entry["audio_emitter"];
                        SushiEngine::Simulation::AudioEmitterParams p;
                        p.sound = a.value("sound", p.sound);
                        p.gain = a.value("gain", p.gain);
                        p.priority = a.value("priority", p.priority);
                        p.bus = a.value("bus", p.bus);
                        p.min_distance = a.value("min_distance", p.min_distance);
                        p.max_distance = a.value("max_distance", p.max_distance);
                        p.distance_model = a.value("distance_model", p.distance_model);
                        p.rolloff = a.value("rolloff", p.rolloff);
                        p.doppler_scale = a.value("doppler_scale", p.doppler_scale);
                        p.reverb_send = a.value("reverb_send", p.reverb_send);
                        p.spatial = a.value("spatial", p.spatial);
                        p.playing = a.value("playing", p.playing);
                        p.looping = a.value("looping", p.looping);
                        world.set_audio_emitter_params(id, p);
                    }
                }

                if (entry.value("has_reverb_zone", false))
                {
                    world.set_has_reverb_zone(id, true);
                    if (entry.contains("reverb_zone"))
                    {
                        const json& z = entry["reverb_zone"];
                        SushiEngine::Simulation::ReverbZoneParams p;
                        if (z.contains("half_extents"))
                            p.half_extents = vec3_from_json(z["half_extents"]);
                        p.room = z.value("room", p.room);
                        p.room_hf = z.value("room_hf", p.room_hf);
                        p.decay_time = z.value("decay_time", p.decay_time);
                        p.decay_hf_ratio = z.value("decay_hf_ratio", p.decay_hf_ratio);
                        p.reflections = z.value("reflections", p.reflections);
                        p.reverb = z.value("reverb", p.reverb);
                        p.diffusion = z.value("diffusion", p.diffusion);
                        p.density = z.value("density", p.density);
                        p.wet_dry_mix = z.value("wet_dry_mix", p.wet_dry_mix);
                        p.send = z.value("send", p.send);
                        p.priority = z.value("priority", p.priority);
                        world.set_reverb_zone_params(id, p);
                    }
                }

                if (entry.value("has_audio_listener", false))
                {
                    world.set_has_audio_listener(id, true);
                    if (entry.contains("audio_listener"))
                    {
                        const json& l = entry["audio_listener"];
                        SushiEngine::Simulation::AudioListenerParams p;
                        p.gain = l.value("gain", p.gain);
                        p.active = l.value("active", p.active);
                        world.set_audio_listener_params(id, p);
                    }
                }

                if (entry.value("has_shape", false))
                {
                    world.set_has_shape(id, true);
                    if (entry.contains("shape"))
                    {
                        const json& s = entry["shape"];
                        SushiEngine::Simulation::ShapeParams params;
                        params.kind = static_cast<SushiEngine::Simulation::PrimitiveKind>(
                            s.value("kind", static_cast<std::uint32_t>(params.kind)));
                        if (s.contains("params"))
                            params.params = vec3_from_json(s["params"]);
                        world.set_shape_params(id, params);
                    }
                }

                if (entry.value("has_collider", false))
                {
                    world.set_has_collider(id, true);
                    if (entry.contains("collider"))
                    {
                        const json& c = entry["collider"];
                        SushiEngine::Simulation::ColliderParams params;
                        params.kind = static_cast<SushiEngine::Simulation::PrimitiveKind>(
                            c.value("kind", static_cast<std::uint32_t>(params.kind)));
                        if (c.contains("params"))
                            params.params = vec3_from_json(c["params"]);
                        world.set_collider_params(id, params);
                    }
                }

                if (entry.value("surface_anchored", false))
                {
                    world.set_surface_anchored(id, true);
                    if (entry.contains("surface_local_orientation"))
                        world.set_surface_local_orientation(
                            id, quaternion_from_json(entry["surface_local_orientation"]));
                }

                // The reference frame (body + mode). Migration: a scene from before the
                // unified dynamic body stored a `has_astro_body` flag instead — map it to a
                // Free reference on the scene's dominant body, the closest representation (its
                // orbital motion returns once the body is also given a velocity; the flag
                // alone only re-expresses the transform). The scene Transform, already loaded,
                // is untouched either way.
                if (entry.contains("reference_body"))
                {
                    SushiEngine::Simulation::EntityFrame frame;
                    frame.reference_body = entry.value("reference_body", -1);
                    frame.mode = static_cast<SushiEngine::Simulation::FrameMode>(
                        entry.value("frame_mode", 0));
                    world.set_entity_frame(id, frame);
                }
                else if (entry.value("has_astro_body", false))
                {
                    SushiEngine::Simulation::EntityFrame frame;
                    frame.reference_body = world.environment().dominant_body_id;
                    frame.mode = SushiEngine::Simulation::FrameMode::Free;
                    world.set_entity_frame(id, frame);
                }

                if (entry.value("has_ui", false))
                {
                    world.set_has_ui(id, true);
                    if (entry.contains("ui"))
                        world.set_ui_params(id, ui_from_json(entry["ui"]));
                }

                if (entry.contains("scripts") && entry["scripts"].is_array())
                    for (const json& s : entry["scripts"])
                        world.add_script_component(id, script_from_json(s));
            }

            // Parent links are resolved only after every entity exists, since a child
            // can be written before its parent in the array.
            for (std::size_t i = 0; i < root.size(); ++i)
            {
                const int parent_index = root[i].value("parent", -1);
                if (parent_index >= 0 && static_cast<std::size_t>(parent_index) < created.size())
                    world.set_parent(created[i], created[static_cast<std::size_t>(parent_index)]);
            }
        }

        bool save_scene(IWorldEditor& world, const std::string& path, const SceneSkyState* sky)
        {
            std::ofstream file(path);
            if (!file)
                return false;

            json root;
            root["entities"] = capture_scene(world);
            root["environment"] = environment_to_json(world.environment());
            root["weather"] = weather_to_json(world);
            if (sky != nullptr)
                root["sky"] = sky_to_json(*sky);

            file << root.dump(2);
            return static_cast<bool>(file);
        }

        namespace
        {
            /**
             * @brief Re-resolves every emitter's sprite textures after a load from disk.
             *
             * The capture carries both a path and the handle it had when written; only the path
             * survives a session, so the handles a file was written with are re-derived here. A
             * post-pass rather than a hook inside `apply_scene` because the in-memory snapshots
             * that share that function are same-session, where the captured handles are still the
             * right ones.
             */
            void resolve_scene_effect_textures(IWorldEditor& world,
                                               SushiEngine::Render::IAssetLibrary& assets)
            {
                for (const EntityId id : world.entities())
                {
                    if (!world.has_particle_emitter(id))
                        continue;
                    SushiEngine::Vfx::ParticleEffect effect = world.particle_effect_source(id);
                    resolve_effect_textures(effect, assets);
                    world.set_particle_effect_source(id, effect);
                }
            }
        } // namespace

        bool load_scene(IWorldEditor& world, const std::string& path, SceneSkyState* sky,
                        SushiEngine::Render::IAssetLibrary* assets)
        {
            std::ifstream file(path);
            if (!file)
                return false;

            json root;
            try
            {
                file >> root;
            }
            catch (const json::parse_error&)
            {
                return false;
            }

            // Pre-environment-persistence scenes are a bare entity array; the environment
            // and sky simply keep whatever the caller already had (their existing defaults).
            if (root.is_array())
            {
                apply_scene(world, root);
                if (assets != nullptr)
                    resolve_scene_effect_textures(world, *assets);
                return true;
            }

            if (!root.is_object() || !root.contains("entities") || !root["entities"].is_array())
                return false;

            apply_scene(world, root["entities"]);
            if (assets != nullptr)
                resolve_scene_effect_textures(world, *assets);
            if (root.contains("environment"))
                world.set_environment(environment_from_json(root["environment"], world.environment()));
            if (root.contains("weather"))
                weather_from_json(root["weather"], world);
            if (sky != nullptr && root.contains("sky"))
                *sky = sky_from_json(root["sky"], *sky);
            return true;
        }
    } // namespace Editor
} // namespace SushiEngine
