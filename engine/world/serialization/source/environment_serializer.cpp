/**************************************************************************/
/* environment_serializer.cpp                                             */
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

#include "environment_serializer.hpp"

namespace SushiEngine
{
    namespace Scene
    {
        namespace
        {
            using nlohmann::json;

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

            json world_vec3_to_json(const SushiEngine::WorldVector3& v)
            {
                return json{{"x", v.x}, {"y", v.y}, {"z", v.z}};
            }

            SushiEngine::WorldVector3 world_vec3_from_json(const json& j,
                                                           SushiEngine::WorldVector3 fallback)
            {
                fallback.x = j.value("x", fallback.x);
                fallback.y = j.value("y", fallback.y);
                fallback.z = j.value("z", fallback.z);
                return fallback;
            }
        } // namespace

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

            json fog_volumes = json::array();
            for (int i = 0; i < e.fog_volume_count && i < SushiEngine::Render::MAX_FOG_VOLUMES;
                 ++i)
            {
                const SushiEngine::Render::FogVolume& v = e.fog_volumes[i];
                fog_volumes.push_back(json{{"center", world_vec3_to_json(v.center)},
                                            {"extent", vec3_to_json(v.extent)},
                                            {"color", vec3_to_json(v.color)},
                                            {"density", v.density},
                                            {"edge_falloff", v.edge_falloff},
                                            {"shape", static_cast<std::uint32_t>(v.shape)}});
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
                {"fog",
                 json{{"enabled", e.fog.enabled},
                      {"density", e.fog.density},
                      {"height_falloff", e.fog.height_falloff},
                      {"scattering_color", vec3_to_json(e.fog.scattering_color)},
                      {"ambient", e.fog.ambient},
                      {"phase_anisotropy", e.fog.phase_anisotropy}}},
                {"fog_volumes", fog_volumes},
                {"gi",
                 json{{"enabled", e.gi.enabled},
                      {"intensity", e.gi.intensity},
                      {"normal_bias", e.gi.normal_bias}}},
                {"surface",
                 json{{"ground_albedo", vec3_to_json(e.surface.ground_albedo)},
                      {"ocean_color", vec3_to_json(e.surface.ocean_color)},
                      {"roughness", e.surface.roughness},
                      {"ocean_roughness", e.surface.ocean_roughness}}},
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
                // The regional nest's physics. Every field of AtmosphereNestParameters is here
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
                      {"free_troposphere_drying", e.atmosphere_nest.free_troposphere_drying},
                      {"free_troposphere_exponent", e.atmosphere_nest.free_troposphere_exponent},
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
                      {"cloud_top_longwave_flux", e.atmosphere_nest.cloud_top_longwave_flux},
                      {"cloud_top_equilibrium_depression",
                       e.atmosphere_nest.cloud_top_equilibrium_depression},
                      {"cloud_water_absorption", e.atmosphere_nest.cloud_water_absorption},
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
                {"ibl_intensity", e.ibl_intensity},
                // The observer is mostly ephemeris-driven while the solar-system sky is
                // enabled (the sky state re-derives it every frame), but it is authored
                // state when that sky is off, and the default environment wants it whole.
                {"observer",
                 json{{"julian_date", e.observer.julian_date},
                      {"latitude_radians", e.observer.latitude_radians},
                      {"longitude_radians", e.observer.longitude_radians},
                      {"astronomical_sun", e.observer.astronomical_sun},
                      {"observer_body", e.observer.observer_body}}}};
        }

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
            if (j.contains("fog") && j["fog"].is_object())
            {
                const json& f = j["fog"];
                environment.fog.enabled = f.value("enabled", environment.fog.enabled);
                environment.fog.density = f.value("density", environment.fog.density);
                environment.fog.height_falloff =
                    f.value("height_falloff", environment.fog.height_falloff);
                if (f.contains("scattering_color"))
                    environment.fog.scattering_color = vec3_from_json(f["scattering_color"]);
                environment.fog.ambient = f.value("ambient", environment.fog.ambient);
                environment.fog.phase_anisotropy =
                    f.value("phase_anisotropy", environment.fog.phase_anisotropy);
            }
            if (j.contains("fog_volumes") && j["fog_volumes"].is_array())
            {
                const json& volumes = j["fog_volumes"];
                environment.fog_volume_count = 0;
                for (const json& entry : volumes)
                {
                    if (environment.fog_volume_count >= SushiEngine::Render::MAX_FOG_VOLUMES)
                        break;
                    SushiEngine::Render::FogVolume& v =
                        environment.fog_volumes[environment.fog_volume_count++];
                    v = SushiEngine::Render::FogVolume{};
                    if (entry.contains("center"))
                        v.center = world_vec3_from_json(entry["center"], v.center);
                    if (entry.contains("extent"))
                        v.extent = vec3_from_json(entry["extent"]);
                    if (entry.contains("color"))
                        v.color = vec3_from_json(entry["color"]);
                    v.density = entry.value("density", v.density);
                    v.edge_falloff = entry.value("edge_falloff", v.edge_falloff);
                    v.shape = static_cast<SushiEngine::Render::FogVolumeShape>(
                        entry.value("shape", static_cast<std::uint32_t>(v.shape)));
                }
                // Slots past the loaded count go back to defaults. The count gates every
                // consumer, but an apply must yield a deterministic state — not "correct
                // count, stale ghosts in the tail" inherited from whatever the base held
                // (the serializer round-trip test is what caught exactly that).
                for (int i = environment.fog_volume_count;
                     i < SushiEngine::Render::MAX_FOG_VOLUMES; ++i)
                    environment.fog_volumes[i] = SushiEngine::Render::FogVolume{};
            }
            if (j.contains("gi") && j["gi"].is_object())
            {
                const json& g = j["gi"];
                environment.gi.enabled = g.value("enabled", environment.gi.enabled);
                environment.gi.intensity = g.value("intensity", environment.gi.intensity);
                environment.gi.normal_bias = g.value("normal_bias", environment.gi.normal_bias);
            }
            if (j.contains("surface") && j["surface"].is_object())
            {
                const json& s = j["surface"];
                if (s.contains("ground_albedo"))
                    environment.surface.ground_albedo = vec3_from_json(s["ground_albedo"]);
                if (s.contains("ocean_color"))
                    environment.surface.ocean_color = vec3_from_json(s["ocean_color"]);
                environment.surface.roughness = s.value("roughness", environment.surface.roughness);
                environment.surface.ocean_roughness =
                    s.value("ocean_roughness", environment.surface.ocean_roughness);
            }
            if (j.contains("atmosphere_nest") && j["atmosphere_nest"].is_object())
            {
                const json& a = j["atmosphere_nest"];
                SushiEngine::Render::AtmosphereNestParameters& n = environment.atmosphere_nest;
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
                n.free_troposphere_drying =
                    a.value("free_troposphere_drying", n.free_troposphere_drying);
                n.free_troposphere_exponent =
                    a.value("free_troposphere_exponent", n.free_troposphere_exponent);
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
                n.cloud_top_longwave_flux =
                    a.value("cloud_top_longwave_flux", n.cloud_top_longwave_flux);
                n.cloud_top_equilibrium_depression =
                    a.value("cloud_top_equilibrium_depression",
                            n.cloud_top_equilibrium_depression);
                n.cloud_water_absorption =
                    a.value("cloud_water_absorption", n.cloud_water_absorption);
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
            if (j.contains("observer") && j["observer"].is_object())
            {
                const json& o = j["observer"];
                environment.observer.julian_date =
                    o.value("julian_date", environment.observer.julian_date);
                environment.observer.latitude_radians =
                    o.value("latitude_radians", environment.observer.latitude_radians);
                environment.observer.longitude_radians =
                    o.value("longitude_radians", environment.observer.longitude_radians);
                environment.observer.astronomical_sun =
                    o.value("astronomical_sun", environment.observer.astronomical_sun);
                environment.observer.observer_body =
                    o.value("observer_body", environment.observer.observer_body);
            }
            return environment;
        }
    } // namespace Scene
} // namespace SushiEngine
