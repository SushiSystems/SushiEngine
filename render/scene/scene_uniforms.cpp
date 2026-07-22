/**************************************************************************/
/* scene_uniforms.cpp                                                     */
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

#include "scene/scene_uniforms.hpp"

#include <algorithm>
#include <cmath>

namespace SushiEngine
{
    namespace Render
    {
        namespace Scene
        {
            void camera_eye(const Mat4& view, double eye[3]) noexcept
            {
                eye[0] = -(view.m[0] * view.m[12] + view.m[1] * view.m[13] +
                           view.m[2] * view.m[14]);
                eye[1] = -(view.m[4] * view.m[12] + view.m[5] * view.m[13] +
                           view.m[6] * view.m[14]);
                eye[2] = -(view.m[8] * view.m[12] + view.m[9] * view.m[13] +
                           view.m[10] * view.m[14]);
            }

            void fill_scene_uniforms(const CameraView& camera, const Environment& environment,
                                     const double eye[3], float time_seconds,
                                     SceneUniforms& uniforms)
            {
                uniforms = SceneUniforms{};

                const Mat4& V = camera.view;
                const Mat4& P = camera.projection;
                const double right[3] = {V.m[0], V.m[4], V.m[8]};
                const double up[3] = {V.m[1], V.m[5], V.m[9]};
                const double forward[3] = {-V.m[2], -V.m[6], -V.m[10]};
                const double tan_half_x = P.m[0] != 0.0 ? 1.0 / P.m[0] : 1.0;
                const double tan_half_y = P.m[5] != 0.0 ? 1.0 / (-P.m[5]) : 1.0;

                const double a = environment.planet.semi_major;
                const double b = environment.planet.semi_minor();
                // The dominant body's centre in the scene frame — Earth straight down by a
                // when at home, another planet's centre after the hand-off.
                const double center_local[3] = {environment.planet_center.x,
                                                environment.planet_center.y,
                                                environment.planet_center.z};
                const double to_center[3] = {eye[0] - center_local[0], eye[1] - center_local[1],
                                             eye[2] - center_local[2]};
                const double altitude =
                    std::sqrt(to_center[0] * to_center[0] + to_center[1] * to_center[1] +
                              to_center[2] * to_center[2]) -
                    environment.planet_surface_reference_metres;

                const Vector3 sun_dir = normalize(environment.sun.direction);

                for (int i = 0; i < 16; ++i)
                {
                    uniforms.view[i] = static_cast<float>(V.m[i]);
                    uniforms.proj[i] = static_cast<float>(P.m[i]);
                }
                // Drop the eye translation: the mesh pass already subtracts the eye from
                // each model's translation, so the view rotates that camera-relative
                // geometry without re-applying the eye.
                uniforms.view[12] = 0.0f;
                uniforms.view[13] = 0.0f;
                uniforms.view[14] = 0.0f;

                uniforms.cam_forward[0] = static_cast<float>(forward[0]);
                uniforms.cam_forward[1] = static_cast<float>(forward[1]);
                uniforms.cam_forward[2] = static_cast<float>(forward[2]);
                uniforms.cam_forward[3] = static_cast<float>(eye[0]);
                uniforms.cam_right[0] = static_cast<float>(right[0] * tan_half_x);
                uniforms.cam_right[1] = static_cast<float>(right[1] * tan_half_x);
                uniforms.cam_right[2] = static_cast<float>(right[2] * tan_half_x);
                uniforms.cam_right[3] = static_cast<float>(eye[1]);
                uniforms.cam_up[0] = static_cast<float>(-up[0] * tan_half_y);
                uniforms.cam_up[1] = static_cast<float>(-up[1] * tan_half_y);
                uniforms.cam_up[2] = static_cast<float>(-up[2] * tan_half_y);
                uniforms.cam_up[3] = static_cast<float>(eye[2]);

                uniforms.planet_center[0] = static_cast<float>(center_local[0] - eye[0]);
                uniforms.planet_center[1] = static_cast<float>(center_local[1] - eye[1]);
                uniforms.planet_center[2] = static_cast<float>(center_local[2] - eye[2]);
                uniforms.planet_center[3] =
                    static_cast<float>(environment.planet_surface_reference_metres);
                uniforms.planet_radii[0] = static_cast<float>(a);
                uniforms.planet_radii[1] = static_cast<float>(a);
                uniforms.planet_radii[2] = static_cast<float>(b);
                uniforms.planet_radii[3] = environment.atmosphere.height;

                uniforms.sun_dir[0] = static_cast<float>(sun_dir.x);
                uniforms.sun_dir[1] = static_cast<float>(sun_dir.y);
                uniforms.sun_dir[2] = static_cast<float>(sun_dir.z);
                uniforms.sun_dir[3] = environment.sun.intensity;
                uniforms.sun_color[0] = static_cast<float>(environment.sun.color.x);
                uniforms.sun_color[1] = static_cast<float>(environment.sun.color.y);
                uniforms.sun_color[2] = static_cast<float>(environment.sun.color.z);
                uniforms.sun_color[3] = environment.exposure;
                uniforms.ambient[0] = static_cast<float>(environment.ambient.x);
                uniforms.ambient[1] = static_cast<float>(environment.ambient.y);
                uniforms.ambient[2] = static_cast<float>(environment.ambient.z);

                uniforms.rayleigh[0] =
                    static_cast<float>(environment.atmosphere.rayleigh_coefficient.x);
                uniforms.rayleigh[1] =
                    static_cast<float>(environment.atmosphere.rayleigh_coefficient.y);
                uniforms.rayleigh[2] =
                    static_cast<float>(environment.atmosphere.rayleigh_coefficient.z);
                uniforms.rayleigh[3] = environment.atmosphere.mie_coefficient;
                uniforms.scatter[0] = environment.atmosphere.mie_anisotropy;
                uniforms.scatter[1] = environment.atmosphere.rayleigh_scale_height;
                uniforms.scatter[2] = environment.atmosphere.mie_scale_height;
                uniforms.scatter[3] = static_cast<float>(altitude);

                uniforms.ground_albedo[0] =
                    static_cast<float>(environment.surface.ground_albedo.x);
                uniforms.ground_albedo[1] =
                    static_cast<float>(environment.surface.ground_albedo.y);
                uniforms.ground_albedo[2] =
                    static_cast<float>(environment.surface.ground_albedo.z);
                uniforms.ocean_color[0] = static_cast<float>(environment.surface.ocean_color.x);
                uniforms.ocean_color[1] = static_cast<float>(environment.surface.ocean_color.y);
                uniforms.ocean_color[2] = static_cast<float>(environment.surface.ocean_color.z);

                // Shared medium: how the whole cloud stack scatters and shadows.
                uniforms.cloud_light[0] = environment.clouds.light_absorption;
                uniforms.cloud_light[1] = environment.clouds.forward_scattering;
                uniforms.cloud_light[2] = environment.clouds.powder_strength;
                uniforms.cloud_light[3] = environment.clouds.ambient_strength;

                // Each deck resolves its genus to a profile (biased by the deck's author
                // tweaks). The union shell [base_min, top_max] over enabled decks is the
                // single span the cloud pass ray marches across.
                float base_min = 1e30f;
                float top_max = 0.0f;
                for (int i = 0; i < CLOUD_MAX_DECKS; ++i)
                {
                    const CloudDeck& deck = environment.clouds.decks[i];
                    const CloudGenusProfile profile = cloud_genus_profile(deck.genus);
                    const bool active = deck.enabled;
                    const float coverage =
                        std::clamp(profile.coverage + deck.coverage_bias, 0.0f, 1.0f);
                    // A disabled deck is encoded as zero density so the shader skips it
                    // without a separate flag, and it never widens the march shell.
                    uniforms.cloud_deck_a[i][0] = profile.base_altitude;
                    uniforms.cloud_deck_a[i][1] = profile.top_altitude;
                    uniforms.cloud_deck_a[i][2] = coverage;
                    uniforms.cloud_deck_a[i][3] =
                        active ? std::max(profile.density * deck.density_scale, 0.0f) : 0.0f;
                    uniforms.cloud_deck_b[i][0] = profile.stratiform;
                    uniforms.cloud_deck_b[i][1] = profile.detail_strength;
                    uniforms.cloud_deck_b[i][2] = profile.shape_scale;
                    uniforms.cloud_deck_b[i][3] = profile.detail_scale;
                    uniforms.cloud_deck_c[i][0] = static_cast<float>(profile.wind.x);
                    uniforms.cloud_deck_c[i][1] = static_cast<float>(profile.wind.y);
                    uniforms.cloud_deck_c[i][2] = static_cast<float>(profile.wind.z);
                    uniforms.cloud_deck_c[i][3] =
                        static_cast<float>(static_cast<std::uint32_t>(profile.noise));
                    uniforms.cloud_deck_d[i][0] = profile.anvil;
                    uniforms.cloud_deck_d[i][1] = environment.clouds.weather_scale;
                    uniforms.cloud_deck_d[i][2] = environment.clouds.evolution_rate;
                    uniforms.cloud_deck_d[i][3] = 0.0f;
                    if (active)
                    {
                        base_min = std::min(base_min, profile.base_altitude);
                        top_max = std::max(top_max, profile.top_altitude);
                    }
                }
                if (top_max <= base_min)
                {
                    base_min = 0.0f;
                    top_max = 0.0f;
                }
                uniforms.cloud_global[0] = environment.clouds.ground_shadow_strength;
                uniforms.cloud_global[1] = base_min;
                uniforms.cloud_global[2] = top_max;
                uniforms.cloud_global[3] = static_cast<float>(CLOUD_MAX_DECKS);

                uniforms.star_params[0] = environment.stars.brightness;
                uniforms.star_params[1] = environment.stars.density;
                uniforms.star_params[2] = environment.atmosphere.enabled ? 1.0f : 0.0f;
                uniforms.star_params[3] = environment.stars.enabled ? 1.0f : 0.0f;
                uniforms.misc[0] = camera.near_plane;
                uniforms.misc[1] = camera.far_plane;
                uniforms.misc[2] = time_seconds;
                uniforms.misc[3] = environment.clouds.enabled ? 1.0f : 0.0f;

                // Far-field solar-system bodies and fixed stars. Their directions are
                // already in the scene's local frame — the same frame the sky pass builds
                // its view rays in — so they copy straight across, no rebasing.
                const int body_count = environment.body_count < MAX_CELESTIAL_BODIES
                                           ? environment.body_count
                                           : MAX_CELESTIAL_BODIES;
                const int star_count = environment.sky_star_count < MAX_SKY_STARS
                                           ? environment.sky_star_count
                                           : MAX_SKY_STARS;
                uniforms.sky_counts[0] = static_cast<float>(body_count);
                uniforms.sky_counts[1] = static_cast<float>(star_count);
                // z: the near-field surface planet (ground/air); the ephemeris clears it
                // past the hand-off altitude, where Earth is a far-field body.
                uniforms.sky_counts[2] = environment.planet_surface_visible ? 1.0f : 0.0f;
                // w: solar-eclipse coverage, computed once by the ephemeris; the sky pass
                // and pbr.frag both dim the sun by it so ground and sky dusk together.
                uniforms.sky_counts[3] = environment.solar_eclipse;
                uniforms.planet_frame[0] = static_cast<float>(environment.planet_pole.x);
                uniforms.planet_frame[1] = static_cast<float>(environment.planet_pole.y);
                uniforms.planet_frame[2] = static_cast<float>(environment.planet_pole.z);
                uniforms.planet_frame[3] = static_cast<float>(environment.planet_surface_style);

                for (int i = 0; i < body_count; ++i)
                {
                    const CelestialBody& body = environment.bodies[i];
                    uniforms.bodies[i * 5 + 0][0] = static_cast<float>(body.direction.x);
                    uniforms.bodies[i * 5 + 0][1] = static_cast<float>(body.direction.y);
                    uniforms.bodies[i * 5 + 0][2] = static_cast<float>(body.direction.z);
                    uniforms.bodies[i * 5 + 0][3] = body.angular_radius;
                    uniforms.bodies[i * 5 + 1][0] = static_cast<float>(body.color.x);
                    uniforms.bodies[i * 5 + 1][1] = static_cast<float>(body.color.y);
                    uniforms.bodies[i * 5 + 1][2] = static_cast<float>(body.color.z);
                    uniforms.bodies[i * 5 + 1][3] = body.brightness;
                    uniforms.bodies[i * 5 + 2][0] = static_cast<float>(body.sun_direction.x);
                    uniforms.bodies[i * 5 + 2][1] = static_cast<float>(body.sun_direction.y);
                    uniforms.bodies[i * 5 + 2][2] = static_cast<float>(body.sun_direction.z);
                    uniforms.bodies[i * 5 + 2][3] = static_cast<float>(body.is_star);
                    uniforms.bodies[i * 5 + 3][0] = body.distance_metres;
                    uniforms.bodies[i * 5 + 3][1] = body.mean_radius_metres;
                    uniforms.bodies[i * 5 + 3][2] = 0.0f;
                    uniforms.bodies[i * 5 + 3][3] = 0.0f;
                    uniforms.bodies[i * 5 + 4][0] = static_cast<float>(body.pole.x);
                    uniforms.bodies[i * 5 + 4][1] = static_cast<float>(body.pole.y);
                    uniforms.bodies[i * 5 + 4][2] = static_cast<float>(body.pole.z);
                    uniforms.bodies[i * 5 + 4][3] = static_cast<float>(body.surface_style);
                }
                for (int i = 0; i < star_count; ++i)
                {
                    const SkyStar& star = environment.sky_stars[i];
                    uniforms.sky_stars[i * 2 + 0][0] = static_cast<float>(star.direction.x);
                    uniforms.sky_stars[i * 2 + 0][1] = static_cast<float>(star.direction.y);
                    uniforms.sky_stars[i * 2 + 0][2] = static_cast<float>(star.direction.z);
                    uniforms.sky_stars[i * 2 + 0][3] = star.brightness;
                    uniforms.sky_stars[i * 2 + 1][0] = static_cast<float>(star.color.x);
                    uniforms.sky_stars[i * 2 + 1][1] = static_cast<float>(star.color.y);
                    uniforms.sky_stars[i * 2 + 1][2] = static_cast<float>(star.color.z);
                    uniforms.sky_stars[i * 2 + 1][3] = 0.0f;
                }
            }
        } // namespace Scene
    } // namespace Render
} // namespace SushiEngine
