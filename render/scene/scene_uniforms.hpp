/**************************************************************************/
/* scene_uniforms.hpp                                                     */
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
 * @file scene_uniforms.hpp
 * @brief The per-frame scene uniform block and the camera/environment fill.
 *
 * One std140 block shared by every pass. Kept as flat float arrays so the C++ side
 * can never disagree with the GLSL packing (every member is 16-byte aligned), and
 * every value is camera-relative: the eye is subtracted in double before the float
 * cast, which is what keeps the planet in place at planetary distances.
 */

#include <cstdint>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/render/environment.hpp>
#include <SushiEngine/render/scene_view.hpp>

namespace SushiEngine
{
    namespace Render
    {
        namespace Scene
        {
            /**
             * @brief The per-frame scene block, mirroring `SceneBlock` in the shaders.
             *
             * Members past @c sky_counts are read only by the sky and cloud passes, so
             * the other shaders' truncated block declarations stay layout-compatible.
             */
            struct SceneUniforms
            {
                float view[16];
                float proj[16];
                float cam_forward[4];   /**< xyz = unit forward, w = camera pos x. */
                float cam_right[4];     /**< xyz = right * tan(fovx/2), w = camera pos y. */
                float cam_up[4];        /**< xyz = up * tan(fovy/2), w = camera pos z. */
                float planet_center[4]; /**< xyz = centre relative to camera, w = surface radius. */
                float planet_radii[4];  /**< xyz = ellipsoid semi-axes, w = atmosphere height. */
                float sun_dir[4];       /**< xyz = direction to sun, w = intensity. */
                float sun_color[4];     /**< xyz = colour, w = exposure. */
                float ambient[4];       /**< xyz = ambient radiance. */
                float rayleigh[4];      /**< xyz = per-metre Rayleigh, w = Mie coefficient. */
                float scatter[4];       /**< x = Mie g, y = Rayleigh H, z = Mie H, w = altitude. */
                float ground_albedo[4];
                float ocean_color[4];
                float cloud_global[4];  /**< ground_shadow_strength, base_min, top_max, deck_count. */
                float star_params[4];   /**< brightness, density, atmosphere on, stars on. */
                float misc[4];          /**< near, far, time, clouds on. */
                float sky_counts[4];    /**< body count, star count, surface visible, solar-eclipse coverage. */
                float planet_frame[4];  /**< xyz = dominant body's north pole, w = surface style. */
                float cloud_light[4];   /**< absorption, forward scattering, powder, ambient. */
                float ibl_params[4];    /**< intensity, specular mip count, ambient mode, spare. */
                float cloud_deck_a[CLOUD_MAX_DECKS][4]; /**< base_alt, top_alt, coverage, density. */
                float cloud_deck_b[CLOUD_MAX_DECKS][4]; /**< stratiform, detail, shape, detail scale. */
                float cloud_deck_c[CLOUD_MAX_DECKS][4]; /**< wind.xyz, noise kind. */
                float cloud_deck_d[CLOUD_MAX_DECKS][4]; /**< anvil, weather scale, evolution, spare. */
                float bodies[MAX_CELESTIAL_BODIES * 5][4];
                float sky_stars[MAX_SKY_STARS * 2][4];
                float planet_ring[4];   /**< x = near-field ring inner radius (m), y = outer radius (m); 0 = none. Appended so shaders reading only earlier fields keep their offsets. */
                float planet_precision[4]; /**< Ellipsoid terms formed in double so the analytic ground never squares planet-scale float32 coordinates: xyz = scaled centre gradient c_rad/a^2 + pole*c_ax/b^2 (subtracted to get the geodetic normal without large-minus-large snap), w = the ray-ellipsoid quadratic constant |M c|^2 - 1 for a camera-origin ray (keeps the "- 1" cancellation's bits at planet scale). Appended after planet_ring for the same offset reason. */
                float lights[MAX_CELESTIAL_LIGHTS * 2][4]; /**< Every body lighting the scene, brightest first. Per light: lane 0 xyz = direction to the body, w = irradiance; lane 1 xyz = colour, w = 1 when the body emits its own light. Appended after planet_precision so shaders reading only earlier fields keep their offsets. */
                float light_counts[4]; /**< x = populated @ref lights entries. The cascades are always fitted to light 0, since the list is ordered by what each light delivers here. */
                /**
                 * @brief Secondary directional casters' shadow record index, one float per light.
                 *
                 * Light 0 never appears here — it samples the cascades, not the punctual
                 * atlas. Lanes 1..4 of light_shadow_a/light_shadow_b (packed 4 then 1,
                 * matching @ref lights' brightest-first order) hold the atlas record
                 * @ref LightSystem::assign_directional_shadows placed that light in, or
                 * -1 when it is unshadowed. Appended after light_counts so shaders
                 * reading only earlier fields keep their offsets.
                 */
                float light_shadow_a[4]; /**< Lights 0-3's shadow record index (lane 0 always -1). */
                /**
                 * @brief Light 4's shadow record index in lane 0, plus weather/spare lanes.
                 *
                 * Lane 1 carries `Environment::weather.ground_wetness` (design doc §5.3, W5) —
                 * `pbr.frag` reads it directly rather than growing this block, since it was
                 * already the nearest otherwise-unused float and every shader that declares
                 * this member already declares the whole block. Lanes 2-3 remain spare.
                 */
                float light_shadow_b[4];
                /**
                 * @brief Scene-XZ -> weather-field UV, with the camera position folded in.
                 *
                 * `xy` = scale, `zw` = offset, so a shader recovers the field UV of a
                 * camera-relative march sample as `p.xz * map.xy + map.zw`. The producer
                 * publishes the transform against **scene-absolute** metres
                 * (`Render::WeatherField`); the eye is added here in double before the float
                 * cast, the same discipline every other planet-scale term in this block
                 * follows. Appended after light_shadow_b so shaders reading only earlier
                 * fields keep their offsets.
                 *
                 * Read by the cloudscape *bake* alone. Since `docs/slop/atmosphere_system.md`
                 * §7.4 the field is what the bake resolves a genus and a coverage from, per
                 * baked column — not a per-sample correction the march applies on top of a
                 * globally compiled deck stack, which is what it was in phase A.
                 */
                float weather_field_map[4];
                /**
                 * @brief The field's vertical band centres, and whether to consult it at all.
                 *
                 * `xyz` = the altitudes (metres above the surface) the three bands are centred
                 * on, ascending, which is what lets a march sample climbing between two bands
                 * read a blend rather than snap at a bucket edge. `w` is 1 when a valid field
                 * was published this frame and 0 otherwise — at 0 every consumer skips the
                 * fetch entirely and the cloudscape renders from its authored deck stack alone,
                 * exactly as it did before the field existed.
                 */
                float weather_field_levels[4];
                /**
                 * @brief Camera-relative XZ metres -> the near cloudscape window's UV.
                 *
                 * `xy` = scale (1/span on both axes), `zw` = offset. The window is
                 * camera-centred and snapped to an absolute texel lattice by
                 * `CloudscapeCompilePass`; the eye and the wind that has blown since the last
                 * bake are folded into the offset here in double, so the pattern keeps
                 * drifting smoothly between bakes without the lookup ever leaving the window.
                 *
                 * Zero scale means "never baked", which
                 * `render/shaders/cloud_field_window.glsl` reads as clear sky.
                 */
                float cloud_field_near[4];
                /**
                 * @brief The same mapping for the coarse far window (§7.2's far field).
                 *
                 * What a march sample past the near window reads instead of a clamped edge:
                 * the same simulated structure baked over an eight-times-wider span, so the
                 * horizon carries the front the near field only shows the beginning of.
                 */
                float cloud_field_far[4];
                /**
                 * @brief The two window spans and the two skip distances, metres.
                 *
                 * `x` = near span, `y` = far span, `z` = the near skip field's cell (the
                 * Nubis3 step rule's `skip_distance` inside the near window), `w` = one far
                 * field texel (the same role past it). Appended last for the same offset
                 * reason as everything above it.
                 */
                float cloud_field_params[4];
                /**
                 * @brief Camera-relative XZ metres -> the regional nest's horizontal UV.
                 *
                 * `xy` = scale, `zw` = offset. Stamped by the scene view from the nest's own
                 * snapped origin, with the eye folded in — the nest is centred on the
                 * simulation's observer, not on any camera, so this is the one place the two
                 * frames are reconciled.
                 */
                float atmosphere_nest_map[4];
                /**
                 * @brief The nest's vertical mapping and whether the bake should read it.
                 *
                 * `x` = domain top (m), `y` = the inverse vertical stretch exponent (the nest's
                 * levels are stretched, so altitude -> texture W is one `pow`), `z` = 1 when the
                 * nest is running, `w` = the extinction of 1 g/m³ of liquid water, which is the
                 * scale the baked density states σ against. Appended last for the same offset
                 * reason as everything above it.
                 */
                float atmosphere_nest_params[4];
                /**
                 * @brief The pattern frame the cloudscape windows were baked in (CloudsV2).
                 *
                 * `xy` = the near window's texel-(0,0) corner, `zw` = the far window's, both
                 * in scene-absolute-plus-wind metres (the frame the bake evaluates its
                 * weather in). The view march's analytic carve reconstructs
                 * `pattern = origin + window_uv * span` from these so its noise stands in
                 * exactly the frame the envelope was baked in. Stamped by
                 * CloudscapeCompilePass::update_window; float, matching the precision the
                 * bake's own push constants already accept. Appended last for the same
                 * offset reason as everything above it.
                 */
                float cloud_field_pattern[4];
                /**
                 * @brief The planetary weather placement's shape terms.
                 *
                 * `x` = how many of @ref synoptic_centre_a are populated, `y` = the ITCZ's
                 * current latitude in radians (the seasonal term of the zonal climatology),
                 * `z` = 1 when the body has a latitudinal cloud structure at all, `w` spare.
                 *
                 * `z` and `x` say different things on purpose: a planet with an atmosphere
                 * always has an ITCZ and a subtropical clear belt even with nothing placed on
                 * it, so a provider that publishes no centres still wants the climatology.
                 * Appended last for the same offset reason as everything above it.
                 */
                float synoptic_params[4];
                /**
                 * @brief Each placed pressure system's direction and extent.
                 *
                 * `xyz` = the unit vector from the planet centre toward the system, **already
                 * rotated into scene space by the simulation** so the march needs no body frame
                 * of its own; `w` = the chord falloff its Gaussian weight uses,
                 * `exp(-w * (1 - dot(radial, xyz)))`. See `Render::SynopticFieldView`.
                 */
                float synoptic_centre_a[Render::SYNOPTIC_FIELD_MAX_CENTRES][4];
                /**
                 * @brief Each placed system's character.
                 *
                 * `x` = signed coverage anomaly at the centre (positive is a low, negative a
                 * high), `y` = convective fraction, `z` = surface precipitation, `w` spare.
                 */
                float synoptic_centre_b[Render::SYNOPTIC_FIELD_MAX_CENTRES][4];
            };

            /**
             * @brief Recovers the camera's world position from its view matrix.
             *
             * Inverts the view transform's rotation applied to its translation column, in
             * double, so the result is the eye every pass renders relative to.
             *
             * @param view The world-to-camera matrix.
             * @param eye  Receives the world eye position in metres.
             */
            void camera_eye(const Mat4& view, double eye[3]) noexcept;

            /**
             * @brief Fills the scene block from this frame's camera and environment.
             *
             * The uploaded view matrix carries no translation, because the mesh pass
             * subtracts the eye from every model translation; the two must never both
             * apply it.
             *
             * @param camera        The view, projection, and clip planes to render from.
             * @param environment   The sun, planet, atmosphere, clouds, and stars.
             * @param eye           The camera world position from camera_eye().
             * @param time_seconds  Animation time driving cloud evolution.
             * @param uniforms      Receives the filled block.
             */
            void fill_scene_uniforms(const CameraView& camera, const Environment& environment,
                                     const double eye[3], float time_seconds,
                                     SceneUniforms& uniforms);

            /**
             * @brief Switches off `sky.frag`'s analytic ellipsoid ground for this frame.
             *
             * The analytic ground is the fallback for a body with no baked terrain
             * (`docs/slop/solar_system_overhaul.md` §10). When real terrain is drawing,
             * the two are not merely redundant — the reference ellipsoid would win every
             * pixel the real elevations dig *below* it, which on the Moon is two
             * kilometres of every mare.
             *
             * Separate from @ref fill_scene_uniforms because the answer is not known when
             * that runs: it depends on what this frame's node selection produced.
             *
             * @param uniforms The block to amend, already filled.
             */
            inline void suppress_analytic_ground(SceneUniforms& uniforms) noexcept
            {
                uniforms.sky_counts[2] = 0.0f;
            }
        } // namespace Scene
    } // namespace Render
} // namespace SushiEngine
