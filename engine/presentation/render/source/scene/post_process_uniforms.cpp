/**************************************************************************/
/* post_process_uniforms.cpp                                              */
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

#include "scene/post_process_uniforms.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Scene
        {
            float circle_of_confusion_scale(float aperture, float max_radius) noexcept
            {
                // A thin-lens circle of confusion grows with the aperture diameter, which is
                // inversely proportional to the f-number. This is a perceptual mapping, not a
                // sensor-exact one: it gives an intuitive "smaller f-number blurs more" without
                // exposing focal length, and the result is bounded by the gather's ceiling.
                const float safe_aperture = aperture < 0.5f ? 0.5f : aperture;
                const float scale = max_radius / safe_aperture;
                return scale > max_radius ? max_radius : scale;
            }

            void fill_post_process_uniforms(const PostProcessSettings& settings,
                                            float linear_exposure, bool bloom_active,
                                            std::uint32_t frame_index, std::uint32_t render_width,
                                            std::uint32_t render_height,
                                            PostProcessUniforms& uniforms) noexcept
            {
                uniforms.exposure[0] = linear_exposure;
                uniforms.exposure[1] = static_cast<float>(static_cast<std::uint32_t>(settings.tonemap));
                uniforms.exposure[2] = settings.bloom.intensity;
                uniforms.exposure[3] = static_cast<float>(frame_index & 0xffffu);

                uniforms.grade0[0] = settings.grade.temperature;
                uniforms.grade0[1] = settings.grade.tint;
                uniforms.grade0[2] = settings.grade.contrast;
                uniforms.grade0[3] = settings.grade.saturation;

                for (int i = 0; i < 3; ++i)
                {
                    uniforms.lift[i] = settings.grade.lift[i];
                    uniforms.gamma[i] = settings.grade.gamma[i];
                    uniforms.gain[i] = settings.grade.gain[i];
                }
                uniforms.lift[3] = 0.0f;
                uniforms.gamma[3] = 0.0f;
                uniforms.gain[3] = 0.0f;

                uniforms.effects[0] = settings.vignette;
                uniforms.effects[1] = settings.chromatic_aberration;
                uniforms.effects[2] = settings.film_grain;
                uniforms.effects[3] = (bloom_active ? 1.0f : 0.0f);

                uniforms.dof[0] = settings.depth_of_field.focus_distance;
                uniforms.dof[1] = settings.depth_of_field.focus_range;
                uniforms.dof[2] = circle_of_confusion_scale(settings.depth_of_field.aperture,
                                                            settings.depth_of_field.max_radius);
                uniforms.dof[3] = settings.depth_of_field.max_radius;

                uniforms.motion[0] = settings.motion_blur.intensity;
                uniforms.motion[1] = static_cast<float>(settings.motion_blur.samples);
                uniforms.motion[2] = (settings.motion_blur.enabled ? 1.0f : 0.0f);
                uniforms.motion[3] = 0.0f;

                uniforms.misc[0] = static_cast<float>(render_width);
                uniforms.misc[1] = static_cast<float>(render_height);
                uniforms.misc[2] = 0.0f;
                uniforms.misc[3] = 0.0f;
            }
        } // namespace Scene
    } // namespace Render
} // namespace SushiEngine
