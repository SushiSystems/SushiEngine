/**************************************************************************/
/* shader_catalogue.cpp                                                   */
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

#include "shader_catalogue.hpp"

#include "brdf_lut.comp.h"
#include "cloud.frag.h"
#include "cloud_composite.frag.h"
#include "contact_shadow.frag.h"
#include "cloud_noise_volume.comp.h"
#include "cloud_noise_weather.comp.h"
#include "fullscreen.vert.h"
#include "fxaa.frag.h"
#include "ibl_irradiance.comp.h"
#include "ibl_prefilter.comp.h"
#include "sh_project.comp.h"
#include "line.frag.h"
#include "mesh.vert.h"
#include "outline.frag.h"
#include "outline.vert.h"
#include "pbr.frag.h"
#include "ray_shadow.frag.h"
#include "shading_rate.comp.h"
#include "shadow.vert.h"
#include "sky.frag.h"
#include "taa.frag.h"
#include "tonemap.frag.h"

namespace SushiEngine
{
    namespace Render
    {
        namespace
        {
            using Resources::ShaderSource;
            using Resources::ShaderStage;

            const ShaderSource CATALOGUE[] = {
                {"mesh.vert", ShaderStage::Vertex, Shaders::mesh_vert_spv,
                 Shaders::mesh_vert_spv_word_count, "mesh.vert"},
                {"outline.vert", ShaderStage::Vertex, Shaders::outline_vert_spv,
                 Shaders::outline_vert_spv_word_count, "outline.vert"},
                {"fullscreen.vert", ShaderStage::Vertex, Shaders::fullscreen_vert_spv,
                 Shaders::fullscreen_vert_spv_word_count, "fullscreen.vert"},
                {"pbr.frag", ShaderStage::Fragment, Shaders::pbr_frag_spv,
                 Shaders::pbr_frag_spv_word_count, "pbr.frag"},
                {"line.frag", ShaderStage::Fragment, Shaders::line_frag_spv,
                 Shaders::line_frag_spv_word_count, "line.frag"},
                {"outline.frag", ShaderStage::Fragment, Shaders::outline_frag_spv,
                 Shaders::outline_frag_spv_word_count, "outline.frag"},
                {"sky.frag", ShaderStage::Fragment, Shaders::sky_frag_spv,
                 Shaders::sky_frag_spv_word_count, "sky.frag"},
                {"cloud.frag", ShaderStage::Fragment, Shaders::cloud_frag_spv,
                 Shaders::cloud_frag_spv_word_count, "cloud.frag"},
                {"shadow.vert", ShaderStage::Vertex, Shaders::shadow_vert_spv,
                 Shaders::shadow_vert_spv_word_count, "shadow.vert"},
                {"ray_shadow.frag", ShaderStage::Fragment, Shaders::ray_shadow_frag_spv,
                 Shaders::ray_shadow_frag_spv_word_count, "ray_shadow.frag"},
                {"contact_shadow.frag", ShaderStage::Fragment,
                 Shaders::contact_shadow_frag_spv,
                 Shaders::contact_shadow_frag_spv_word_count, "contact_shadow.frag"},
                {"cloud_composite.frag", ShaderStage::Fragment,
                 Shaders::cloud_composite_frag_spv,
                 Shaders::cloud_composite_frag_spv_word_count, "cloud_composite.frag"},
                {"taa.frag", ShaderStage::Fragment, Shaders::taa_frag_spv,
                 Shaders::taa_frag_spv_word_count, "taa.frag"},
                {"tonemap.frag", ShaderStage::Fragment, Shaders::tonemap_frag_spv,
                 Shaders::tonemap_frag_spv_word_count, "tonemap.frag"},
                {"fxaa.frag", ShaderStage::Fragment, Shaders::fxaa_frag_spv,
                 Shaders::fxaa_frag_spv_word_count, "fxaa.frag"},
                {"shading_rate.comp", ShaderStage::Compute, Shaders::shading_rate_comp_spv,
                 Shaders::shading_rate_comp_spv_word_count, "shading_rate.comp"},
                {"cloud_noise_volume.comp", ShaderStage::Compute,
                 Shaders::cloud_noise_volume_comp_spv,
                 Shaders::cloud_noise_volume_comp_spv_word_count, "cloud_noise_volume.comp"},
                {"cloud_noise_weather.comp", ShaderStage::Compute,
                 Shaders::cloud_noise_weather_comp_spv,
                 Shaders::cloud_noise_weather_comp_spv_word_count, "cloud_noise_weather.comp"},
                {"ibl_prefilter.comp", ShaderStage::Compute, Shaders::ibl_prefilter_comp_spv,
                 Shaders::ibl_prefilter_comp_spv_word_count, "ibl_prefilter.comp"},
                {"ibl_irradiance.comp", ShaderStage::Compute, Shaders::ibl_irradiance_comp_spv,
                 Shaders::ibl_irradiance_comp_spv_word_count, "ibl_irradiance.comp"},
                {"brdf_lut.comp", ShaderStage::Compute, Shaders::brdf_lut_comp_spv,
                 Shaders::brdf_lut_comp_spv_word_count, "brdf_lut.comp"},
                {"sh_project.comp", ShaderStage::Compute, Shaders::sh_project_comp_spv,
                 Shaders::sh_project_comp_spv_word_count, "sh_project.comp"},
            };
        } // namespace

        const Resources::ShaderSource* shader_catalogue() noexcept { return CATALOGUE; }

        std::size_t shader_catalogue_count() noexcept
        {
            return sizeof(CATALOGUE) / sizeof(CATALOGUE[0]);
        }
    } // namespace Render
} // namespace SushiEngine
