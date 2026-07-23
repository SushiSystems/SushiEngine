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

#include "aerial_perspective.comp.h"
#include "bloom_down.comp.h"
#include "bloom_up.comp.h"
#include "brdf_lut.comp.h"
#include "cluster_build.comp.h"
#include "fog_scatter.comp.h"
#include "gi_probe_relight.comp.h"
#include "luminance_histogram.comp.h"
#include "multiscatter_lut.comp.h"
#include "sdf_populate.comp.h"
#include "sdf_probe_relight.comp.h"
#include "sky_view_lut.comp.h"
#include "transmittance_lut.comp.h"
#include "light_shadow.vert.h"
#include "cloud.frag.h"
#include "cloud_composite.frag.h"
#include "contact_shadow.frag.h"
#include "dof.frag.h"
#include "motion_blur.frag.h"
#include "cloud_noise_volume.comp.h"
#include "cloud_noise_weather.comp.h"
#include "fullscreen.vert.h"
#include "fxaa.frag.h"
#include "gtao.comp.h"
#include "gtao_resolve.frag.h"
#include "ground_shadow_resolve.frag.h"
#include "hiz.comp.h"
#include "ssr.frag.h"
#include "ibl_irradiance.comp.h"
#include "ibl_prefilter.comp.h"
#include "sh_project.comp.h"
#include "cloth.comp.h"
#include "cull.comp.h"
#include "mesh.vert.h"
#include "mesh_gpu.vert.h"
#include "meshlet.mesh.h"
#include "meshlet.task.h"
#include "occlusion.comp.h"
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
                {"mesh_gpu.vert", ShaderStage::Vertex, Shaders::mesh_gpu_vert_spv,
                 Shaders::mesh_gpu_vert_spv_word_count, "mesh_gpu.vert"},
                {"outline.vert", ShaderStage::Vertex, Shaders::outline_vert_spv,
                 Shaders::outline_vert_spv_word_count, "outline.vert"},
                {"fullscreen.vert", ShaderStage::Vertex, Shaders::fullscreen_vert_spv,
                 Shaders::fullscreen_vert_spv_word_count, "fullscreen.vert"},
                {"pbr.frag", ShaderStage::Fragment, Shaders::pbr_frag_spv,
                 Shaders::pbr_frag_spv_word_count, "pbr.frag"},
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
                {"cluster_build.comp", ShaderStage::Compute, Shaders::cluster_build_comp_spv,
                 Shaders::cluster_build_comp_spv_word_count, "cluster_build.comp"},
                {"transmittance_lut.comp", ShaderStage::Compute,
                 Shaders::transmittance_lut_comp_spv,
                 Shaders::transmittance_lut_comp_spv_word_count, "transmittance_lut.comp"},
                {"multiscatter_lut.comp", ShaderStage::Compute, Shaders::multiscatter_lut_comp_spv,
                 Shaders::multiscatter_lut_comp_spv_word_count, "multiscatter_lut.comp"},
                {"sky_view_lut.comp", ShaderStage::Compute, Shaders::sky_view_lut_comp_spv,
                 Shaders::sky_view_lut_comp_spv_word_count, "sky_view_lut.comp"},
                {"aerial_perspective.comp", ShaderStage::Compute,
                 Shaders::aerial_perspective_comp_spv,
                 Shaders::aerial_perspective_comp_spv_word_count, "aerial_perspective.comp"},
                {"fog_scatter.comp", ShaderStage::Compute, Shaders::fog_scatter_comp_spv,
                 Shaders::fog_scatter_comp_spv_word_count, "fog_scatter.comp"},
                {"gi_probe_relight.comp", ShaderStage::Compute, Shaders::gi_probe_relight_comp_spv,
                 Shaders::gi_probe_relight_comp_spv_word_count, "gi_probe_relight.comp"},
                {"sdf_populate.comp", ShaderStage::Compute, Shaders::sdf_populate_comp_spv,
                 Shaders::sdf_populate_comp_spv_word_count, "sdf_populate.comp"},
                {"sdf_probe_relight.comp", ShaderStage::Compute, Shaders::sdf_probe_relight_comp_spv,
                 Shaders::sdf_probe_relight_comp_spv_word_count, "sdf_probe_relight.comp"},
                {"light_shadow.vert", ShaderStage::Vertex, Shaders::light_shadow_vert_spv,
                 Shaders::light_shadow_vert_spv_word_count, "light_shadow.vert"},
                {"gtao.comp", ShaderStage::Compute, Shaders::gtao_comp_spv,
                 Shaders::gtao_comp_spv_word_count, "gtao.comp"},
                {"gtao_resolve.frag", ShaderStage::Fragment, Shaders::gtao_resolve_frag_spv,
                 Shaders::gtao_resolve_frag_spv_word_count, "gtao_resolve.frag"},
                {"hiz.comp", ShaderStage::Compute, Shaders::hiz_comp_spv,
                 Shaders::hiz_comp_spv_word_count, "hiz.comp"},
                {"occlusion.comp", ShaderStage::Compute, Shaders::occlusion_comp_spv,
                 Shaders::occlusion_comp_spv_word_count, "occlusion.comp"},
                {"cull.comp", ShaderStage::Compute, Shaders::cull_comp_spv,
                 Shaders::cull_comp_spv_word_count, "cull.comp"},
                {"cloth.comp", ShaderStage::Compute, Shaders::cloth_comp_spv,
                 Shaders::cloth_comp_spv_word_count, "cloth.comp"},
                {"meshlet.task", ShaderStage::Task, Shaders::meshlet_task_spv,
                 Shaders::meshlet_task_spv_word_count, "meshlet.task"},
                {"meshlet.mesh", ShaderStage::Mesh, Shaders::meshlet_mesh_spv,
                 Shaders::meshlet_mesh_spv_word_count, "meshlet.mesh"},
                {"ssr.frag", ShaderStage::Fragment, Shaders::ssr_frag_spv,
                 Shaders::ssr_frag_spv_word_count, "ssr.frag"},
                {"ground_shadow_resolve.frag", ShaderStage::Fragment,
                 Shaders::ground_shadow_resolve_frag_spv,
                 Shaders::ground_shadow_resolve_frag_spv_word_count,
                 "ground_shadow_resolve.frag"},
                {"bloom_down.comp", ShaderStage::Compute, Shaders::bloom_down_comp_spv,
                 Shaders::bloom_down_comp_spv_word_count, "bloom_down.comp"},
                {"bloom_up.comp", ShaderStage::Compute, Shaders::bloom_up_comp_spv,
                 Shaders::bloom_up_comp_spv_word_count, "bloom_up.comp"},
                {"luminance_histogram.comp", ShaderStage::Compute,
                 Shaders::luminance_histogram_comp_spv,
                 Shaders::luminance_histogram_comp_spv_word_count, "luminance_histogram.comp"},
                {"dof.frag", ShaderStage::Fragment, Shaders::dof_frag_spv,
                 Shaders::dof_frag_spv_word_count, "dof.frag"},
                {"motion_blur.frag", ShaderStage::Fragment, Shaders::motion_blur_frag_spv,
                 Shaders::motion_blur_frag_spv_word_count, "motion_blur.frag"},
            };
        } // namespace

        const Resources::ShaderSource* shader_catalogue() noexcept { return CATALOGUE; }

        std::size_t shader_catalogue_count() noexcept
        {
            return sizeof(CATALOGUE) / sizeof(CATALOGUE[0]);
        }
    } // namespace Render
} // namespace SushiEngine
