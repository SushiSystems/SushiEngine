/**************************************************************************/
/* shading_set.cpp                                                        */
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

#include "passes/shading_set.hpp"

#include "lighting/cluster_config.hpp"
#include "lighting/light_system.hpp"
#include "material/material_system.hpp"
#include "passes/cloud_shadow_map_pass.hpp"
#include "passes/ibl_pass.hpp"
#include "passes/irradiance_volume_pass.hpp"
#include "passes/shadow_pass.hpp"
#include "resources/sampler_cache.hpp"
#include "scene/motion_system.hpp"
#include "scene/scene_uniforms.hpp"
#include "scene/shadow_uniforms.hpp"
#include "scene/temporal_uniforms.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Passes
        {
            void declare_shading_set(Graph::RenderPassBuilder& builder,
                                     const Frame::FrameContext& frame)
            {
                builder.read(frame.targets.uniforms, Graph::BufferAccess::UniformRead);
                builder.read(frame.targets.temporal, Graph::BufferAccess::UniformRead);
                builder.read(frame.targets.shadow, Graph::BufferAccess::UniformRead);
                builder.read(frame.targets.shadow_atlas, Graph::TextureAccess::SampledFragment);
                builder.read(frame.targets.contact_shadow, Graph::TextureAccess::SampledFragment);
                builder.read(frame.targets.ray_shadow, Graph::TextureAccess::SampledFragment);
                // The froxel grid the cull pass built: read here so the graph derives the
                // compute→fragment barrier that makes the light lists visible before
                // shading loops them.
                builder.read(frame.targets.cluster_grid, Graph::BufferAccess::StorageRead);
                builder.read(frame.targets.light_index, Graph::BufferAccess::StorageRead);
                builder.read(frame.targets.light_shadow_atlas,
                             Graph::TextureAccess::SampledFragment);
                builder.read(frame.targets.decal_grid, Graph::BufferAccess::StorageRead);
                builder.read(frame.targets.decal_index, Graph::BufferAccess::StorageRead);
                builder.read(frame.targets.ao, Graph::TextureAccess::SampledFragment);
            }

            void write_shading_set(Scene::SceneSetWriter& writer,
                                   const ShadingSetSources& sources,
                                   const Frame::FrameContext& frame,
                                   const Graph::PassContext& context)
            {
                writer.uniform(Scene::SceneLayout::SCENE_BINDING,
                               context.buffer(frame.targets.uniforms),
                               sizeof(Scene::SceneUniforms));
                writer.image(1, sources.ibl.irradiance(), sources.ibl.sampler());
                writer.image(2, sources.ibl.specular(), sources.ibl.sampler());
                writer.image(3, sources.ibl.brdf_lut(), sources.ibl.sampler());
                writer.image(Scene::SceneLayout::SHADOW_ATLAS_BINDING,
                             context.sampled_view(frame.targets.shadow_atlas),
                             ShadowPass::atlas_sampler(*frame.samplers));
                writer.image(Scene::SceneLayout::SHADOW_DEPTH_BINDING,
                             context.sampled_view(frame.targets.shadow_atlas),
                             ShadowPass::atlas_depth_sampler(*frame.samplers));
                writer.image(4, context.sampled_view(frame.targets.ray_shadow),
                             frame.samplers->get(Resources::SamplerDescription{}));
                writer.image(5, context.sampled_view(frame.targets.contact_shadow),
                             frame.samplers->get(Resources::SamplerDescription{}));
                // Kept in GENERAL across CloudShadowMapPass's own compute build.
                writer.image(6, sources.cloud_shadow.view(), sources.cloud_shadow.sampler(),
                             VK_IMAGE_LAYOUT_GENERAL);
                writer.storage(Scene::SceneLayout::MATERIAL_BINDING, sources.materials.buffer(),
                               sources.materials.buffer_range());
                writer.storage(Scene::SceneLayout::MOTION_BINDING, sources.motion.buffer(),
                               sources.motion.buffer_range());
                writer.uniform(Scene::SceneLayout::TEMPORAL_BINDING,
                               context.buffer(frame.targets.temporal),
                               sizeof(Scene::TemporalUniforms));
                writer.uniform(Scene::SceneLayout::SHADOW_BINDING,
                               context.buffer(frame.targets.shadow),
                               sizeof(Scene::ShadowUniforms));
                writer.storage(Scene::SceneLayout::IBL_SH_BINDING, sources.ibl.sh_buffer(),
                               IBLPass::sh_buffer_bytes());
                // The clustered light engine's four bindings: the light array and config
                // block are host-written and bound directly (like the material array); the
                // count grid and index list are the graph transients the cull pass wrote
                // this frame.
                writer.storage(Scene::SceneLayout::LIGHT_BINDING, sources.lights.light_buffer(),
                               sources.lights.light_buffer_range());
                writer.storage(Scene::SceneLayout::CLUSTER_GRID_BINDING,
                               context.buffer(frame.targets.cluster_grid),
                               Lighting::CLUSTER_COUNT * sizeof(std::uint32_t));
                writer.storage(Scene::SceneLayout::LIGHT_INDEX_BINDING,
                               context.buffer(frame.targets.light_index),
                               Lighting::LIGHT_INDEX_COUNT * sizeof(std::uint32_t));
                writer.uniform(Scene::SceneLayout::CLUSTER_CONFIG_BINDING,
                               sources.lights.config_buffer(),
                               sources.lights.config_buffer_range());
                // Punctual spot shadows: the atlas through the same comparison sampler the
                // sun cascades use, and the per-caster matrix buffer.
                writer.image(Scene::SceneLayout::LIGHT_SHADOW_ATLAS_BINDING,
                             context.sampled_view(frame.targets.light_shadow_atlas),
                             ShadowPass::atlas_sampler(*frame.samplers));
                writer.storage(Scene::SceneLayout::LIGHT_SHADOW_DATA_BINDING,
                               sources.lights.shadow_buffer(),
                               sources.lights.shadow_buffer_range());
                // Clustered decals: the decal array (host-written, bound directly) and the
                // count grid + index list the cull pass wrote this frame.
                writer.storage(Scene::SceneLayout::DECAL_BINDING, sources.lights.decal_buffer(),
                               sources.lights.decal_buffer_range());
                writer.storage(Scene::SceneLayout::DECAL_GRID_BINDING,
                               context.buffer(frame.targets.decal_grid),
                               Lighting::CLUSTER_COUNT * sizeof(std::uint32_t));
                writer.storage(Scene::SceneLayout::DECAL_INDEX_BINDING,
                               context.buffer(frame.targets.decal_index),
                               Lighting::DECAL_INDEX_COUNT * sizeof(std::uint32_t));
                // The resolved ambient occlusion the shading pass multiplies its indirect
                // diffuse and specular by.
                writer.image(Scene::SceneLayout::AO_BINDING,
                             context.sampled_view(frame.targets.ao),
                             frame.samplers->get(Resources::SamplerDescription{}));
                // Probe-volume GI: the SH grid the shading pass gathers and the config block
                // that locates a surface in it. Both are pass-owned resources the
                // irradiance-volume pass barriered before this pass runs.
                writer.storage(Scene::SceneLayout::GI_PROBE_SH_BINDING,
                               sources.gi.probe_sh_buffer(),
                               IrradianceVolumePass::probe_sh_bytes());
                writer.uniform(Scene::SceneLayout::GI_PROBE_CONFIG_BINDING,
                               sources.gi.config_buffer(frame.index),
                               IrradianceVolumePass::config_bytes());
            }
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
