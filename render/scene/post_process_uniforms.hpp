/**************************************************************************/
/* post_process_uniforms.hpp                                              */
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
 * @file post_process_uniforms.hpp
 * @brief The per-frame parameter block the post-processing passes read.
 *
 * The single home for everything the display-transform stack needs — the resolved
 * exposure multiplier, the tone-curve selector, and every grade and lens parameter —
 * kept out of SceneUniforms for the same reason the temporal block is: it changes for
 * a different reason (the author's look, not the world) and is read by a different,
 * small set of passes (tonemap, DoF, motion blur), each of which declares all of it.
 * This is the "per-camera parameter block the passes read" the Post-Process window
 * resolves into: the editor writes the data, the passes consume it, neither names the
 * other.
 */

#include <cstdint>

#include <SushiEngine/render/render_settings.hpp>

namespace SushiEngine
{
    namespace Render
    {
        namespace Scene
        {
            /**
             * @brief The post-processing block, mirroring `PostBlock` in the shaders.
             *
             * Flat vec4 rows so the C++ side cannot disagree with the std140 packing; every
             * member is a 16-byte row. The lane assignments are documented per field and
             * must match the shader declarations exactly.
             */
            struct PostProcessUniforms
            {
                float exposure[4]; /**< x = linear exposure multiplier, y = tone-curve id, z = bloom intensity, w = frame index. */
                float grade0[4];   /**< x = white-balance temperature, y = tint, z = contrast, w = saturation. */
                float lift[4];     /**< xyz = shadow lift, w spare. */
                float gamma[4];    /**< xyz = midtone gamma, w spare. */
                float gain[4];     /**< xyz = highlight gain, w spare. */
                float effects[4];  /**< x = vignette, y = chromatic aberration (px), z = film grain, w = bloom enabled. */
                float dof[4];      /**< x = focus distance (m), y = focus range (m), z = circle-of-confusion scale, w = max radius (px). */
                float motion[4];   /**< x = intensity, y = sample count, z = enabled, w spare. */
                float misc[4];     /**< x = render width, y = render height, z spare, w spare. */
            };

            /**
             * @brief Fills the post-processing block from the resolved settings and exposure.
             *
             * Packs @p settings verbatim into the std140 lanes, folds the tone-curve enum to
             * its integer id, and stamps the already-resolved linear exposure and the frame
             * index the temporal grain/dither advance against. The exposure is computed by the
             * caller (manual from the environment, or the adapted auto-exposure value) so this
             * function stays a pure pack with no policy.
             *
             * @param settings       This frame's resolved post-processing parameters.
             * @param linear_exposure The final linear exposure multiplier the tone map applies.
             * @param bloom_active    Whether the bloom pyramid actually ran this frame (the
             *                        author's enable AND the tier permitting it); the tone map
             *                        composites the bloom target only when this is set.
             * @param frame_index     Monotonic frame counter, for temporal grain and dither.
             * @param render_width    Internal render width in pixels.
             * @param render_height   Internal render height in pixels.
             * @param uniforms        Receives the filled block.
             */
            void fill_post_process_uniforms(const PostProcessSettings& settings,
                                            float linear_exposure, bool bloom_active,
                                            std::uint32_t frame_index, std::uint32_t render_width,
                                            std::uint32_t render_height,
                                            PostProcessUniforms& uniforms) noexcept;

            /**
             * @brief The circle-of-confusion pixel scale a given aperture and focus imply.
             *
             * A convenience the DoF pass and the editor share so both agree on how an f-number
             * turns into a blur radius; kept here beside the block it feeds.
             *
             * @param aperture   The lens f-number (smaller blurs more).
             * @param max_radius The circle-of-confusion ceiling in pixels.
             * @return The per-metre-of-defocus radius scale, in pixels, clamped to @p max_radius.
             */
            float circle_of_confusion_scale(float aperture, float max_radius) noexcept;
        } // namespace Scene
    } // namespace Render
} // namespace SushiEngine
