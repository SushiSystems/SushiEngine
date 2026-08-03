/**************************************************************************/
/* shadow_uniforms.hpp                                                    */
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
 * @file shadow_uniforms.hpp
 * @brief Where the sun's cascades sit this frame, and how they are sampled.
 *
 * A block of its own for the same reason the temporal one is: the scene block
 * describes the world and is declared as a truncated prefix by most shaders, so
 * appending to it is fragile, while this has different readers and changes for a
 * different reason.
 *
 * Every matrix here is camera-relative — the cascades are fitted around the eye, which
 * is the origin — so a shadow map for a scene sitting six million metres from the
 * world origin is built entirely out of small numbers.
 */

#include <cstdint>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/render/render_settings.hpp>
#include <SushiEngine/render/scene_view.hpp>

namespace SushiEngine
{
    namespace Render
    {
        namespace Scene
        {
            /**
             * @brief The most cascades the atlas can hold.
             *
             * Four, because the atlas is a two-by-two grid of tiles in one image: one
             * image means one descriptor and one pass, and four cascades already put
             * enough texels near the eye that a fifth buys very little.
             */
            constexpr std::uint32_t MAX_SHADOW_CASCADES = 4;

            /**
             * @brief The shadow block, mirroring @c ShadowBlock in the shaders.
             *
             * Flat float arrays so the C++ side cannot disagree with the std140 packing.
             */
            struct ShadowUniforms
            {
                float cascade_view_projection[MAX_SHADOW_CASCADES][16];
                float splits[4];      /**< View distance each cascade reaches, metres. */
                float texel_size[4];  /**< World metres one shadow texel covers, per cascade. */
                float depth_range[4]; /**< World metres the [0,1] depth spans, per cascade. */
                float params[4];      /**< cascade count, tile uv scale, spare, cascade blend. */
                float filter[4];      /**< min radius, max radius, penumbra slope, spare. */
                float bias[4];        /**< depth bias, normal bias, contact metres, contact steps. */
                float flags[4];       /**< shadows on, contact on, ray traced, atlas resolution. */
            };

            /**
             * @brief Fits the cascades to this frame's camera and sun, and fills the block.
             *
             * The split positions are the practical scheme — a blend of the logarithmic
             * split perspective actually wants and the uniform one that keeps the far
             * cascades from starving. Each cascade is then bounded by a *sphere* rather
             * than by its frustum corners, and the light-space origin is snapped to whole
             * shadow texels: together those are what stop the shadow edges from crawling
             * as the camera turns, because a sphere's extent does not change with
             * rotation and a texel-aligned origin does not shift sub-texel.
             *
             * The texel snap is anchored to the *world*, not to the camera. The fit
             * itself is camera-relative — the eye is the origin — so a grid snapped in
             * that space would travel with the camera and stabilise nothing at all; the
             * eye is added back before the snap and removed after it, in double, so the
             * grid stands still in the world while the numbers stay small.
             *
             * @param camera    This frame's camera; only its rotation and field of view
             *                  are used, since the fit is camera-relative.
             * @param eye       The camera's world position in metres, which the texel
             *                  grid is anchored against.
             * @param sun       Unit direction toward the sun.
             * @param settings  Cascade count, coverage, resolution, and biases.
             * @param uniforms  Receives the filled block.
             * @return The number of cascades actually fitted, zero when shadows are off.
             */
            std::uint32_t fit_shadow_cascades(const CameraView& camera, const double eye[3],
                                              const Vector3& sun,
                                              const ShadowSettings& settings,
                                              ShadowUniforms& uniforms) noexcept;
        } // namespace Scene
    } // namespace Render
} // namespace SushiEngine
