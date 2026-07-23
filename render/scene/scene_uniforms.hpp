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
        } // namespace Scene
    } // namespace Render
} // namespace SushiEngine
