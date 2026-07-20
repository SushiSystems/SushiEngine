/**************************************************************************/
/* temporal_uniforms.hpp                                                  */
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
 * @file temporal_uniforms.hpp
 * @brief The per-frame block every temporally-aware pass reads.
 *
 * Kept out of SceneUniforms deliberately. That block is the world being drawn and is
 * consumed by shaders which declare a truncated prefix of it, so appending to it is
 * fragile; this one is the frame's relationship to the frame before it, which is a
 * different reason to change and a different set of readers. It is small, so every
 * shader that needs any of it declares all of it.
 *
 * Every matrix here is camera-relative and translation-free, exactly like the scene
 * block's view: the previous frame's eye is folded into the previous transforms the
 * motion system packs, never into this matrix, so the two never both apply it.
 */

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/render/render_settings.hpp>

namespace SushiEngine
{
    namespace Render
    {
        namespace Scene
        {
            /**
             * @brief The temporal block, mirroring @c TemporalBlock in the shaders.
             *
             * Flat float arrays so the C++ side cannot disagree with the std140 packing;
             * every member is a 16-byte row.
             */
            struct TemporalUniforms
            {
                float previous_view_projection[16];
                float jitter[4];      /**< xy = this frame's NDC jitter, zw = last frame's. */
                float resolution[4];  /**< xy = internal render extent, zw = output extent. */
                float blend[4];       /**< still feedback, moving feedback, sharpness, history valid. */
                float thresholds[4];  /**< luminance, velocity, clamp history, spare. */
            };

            /**
             * @brief Composes the previous frame's world-to-clip from its camera.
             *
             * Strips the translation from the view for the same reason the scene block
             * does — the object transforms arrive camera-relative — and uses the
             * unjittered projection, so the jitter never leaks into a motion vector.
             *
             * @param previous_view       Last frame's world-to-camera matrix.
             * @param previous_projection Last frame's unjittered camera-to-clip matrix.
             * @param result              Receives the 16 floats, column-major.
             */
            void previous_view_projection(const Mat4& previous_view,
                                          const Mat4& previous_projection,
                                          float result[16]) noexcept;

            /**
             * @brief Fills the temporal block for one frame.
             *
             * @param settings         This frame's fidelity/performance request.
             * @param previous_view    Last frame's world-to-camera matrix.
             * @param previous_proj    Last frame's unjittered camera-to-clip matrix.
             * @param jitter           This frame's NDC jitter as {x, y}.
             * @param previous_jitter  Last frame's NDC jitter as {x, y}.
             * @param render_width     Internal render width in pixels.
             * @param render_height    Internal render height in pixels.
             * @param output_width     Output width in pixels.
             * @param output_height    Output height in pixels.
             * @param history_valid    Whether a previous resolved frame exists to blend with.
             * @param uniforms         Receives the filled block.
             */
            void fill_temporal_uniforms(const RenderSettings& settings, const Mat4& previous_view,
                                        const Mat4& previous_proj, const float jitter[2],
                                        const float previous_jitter[2],
                                        std::uint32_t render_width, std::uint32_t render_height,
                                        std::uint32_t output_width, std::uint32_t output_height,
                                        bool history_valid, TemporalUniforms& uniforms) noexcept;
        } // namespace Scene
    } // namespace Render
} // namespace SushiEngine
