/**************************************************************************/
/* temporal_uniforms.cpp                                                  */
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

#include "scene/temporal_uniforms.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Scene
        {
            void previous_view_projection(const Mat4& previous_view,
                                          const Mat4& previous_projection,
                                          float result[16]) noexcept
            {
                Mat4 view = previous_view;
                view.m[12] = 0.0;
                view.m[13] = 0.0;
                view.m[14] = 0.0;
                const Mat4 view_projection = mul(previous_projection, view);
                for (int i = 0; i < 16; ++i)
                    result[i] = static_cast<float>(view_projection.m[i]);
            }

            void fill_temporal_uniforms(const RenderSettings& settings, const Mat4& previous_view,
                                        const Mat4& previous_proj, const float jitter[2],
                                        const float previous_jitter[2],
                                        std::uint32_t render_width, std::uint32_t render_height,
                                        std::uint32_t output_width, std::uint32_t output_height,
                                        bool history_valid, TemporalUniforms& uniforms) noexcept
            {
                uniforms = TemporalUniforms{};
                previous_view_projection(previous_view, previous_proj,
                                         uniforms.previous_view_projection);

                uniforms.jitter[0] = jitter[0];
                uniforms.jitter[1] = jitter[1];
                uniforms.jitter[2] = previous_jitter[0];
                uniforms.jitter[3] = previous_jitter[1];

                uniforms.resolution[0] = static_cast<float>(render_width);
                uniforms.resolution[1] = static_cast<float>(render_height);
                uniforms.resolution[2] = static_cast<float>(output_width);
                uniforms.resolution[3] = static_cast<float>(output_height);

                uniforms.blend[0] = settings.temporal.feedback_still;
                uniforms.blend[1] = settings.temporal.feedback_moving;
                uniforms.blend[2] = settings.temporal.sharpness;
                uniforms.blend[3] = history_valid ? 1.0f : 0.0f;

                uniforms.thresholds[0] = settings.variable_rate_shading.luminance_threshold;
                uniforms.thresholds[1] = settings.variable_rate_shading.velocity_threshold;
                uniforms.thresholds[2] = settings.temporal.clamp_history ? 1.0f : 0.0f;
            }
        } // namespace Scene
    } // namespace Render
} // namespace SushiEngine
