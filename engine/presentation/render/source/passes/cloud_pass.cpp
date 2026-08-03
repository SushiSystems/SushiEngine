/**************************************************************************/
/* cloud_pass.cpp                                                         */
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

#include "passes/cloud_pass.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "frame/frame_context.hpp"
#include "graph/render_graph.hpp"
#include "passes/atmosphere_lut_pass.hpp"
#include "passes/cloud_light_volume_pass.hpp"
#include "passes/cloudscape_compile_pass.hpp"
#include "passes/fullscreen.hpp"
#include "resources/descriptor_allocator.hpp"
#include "resources/pipeline_cache.hpp"
#include "resources/sampler_cache.hpp"
#include "resources/shader_library.hpp"
#include "scene/scene_layout.hpp"
#include "scene/scene_uniforms.hpp"
#include "scene/temporal_uniforms.hpp"
#include "textures/cloud_noise.hpp"
#include "rhi/vulkan/vulkan_device.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Passes
        {
            namespace
            {
                // How many mip levels of carve spread the push block carries. The march volume
                // is 128^3, so eight levels reach 1^3 and nothing is truncated; the shader
                // mirrors this as a vec4[2] and clamps its own indexing to the count actually
                // pushed, so a volume with fewer levels is a legal configuration rather than a
                // hazard.
                constexpr std::size_t CARVE_SPREAD_CAPACITY = 8;
            } // namespace

            CloudPass::CloudPass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                                 Resources::GraphicsPipelineFactory& pipelines,
                                 Scene::SceneLayout& layout, CloudscapeCompilePass& cloudscape,
                                 CloudLightVolumePass& light_volume, Textures::CloudNoise& noise,
                                 AtmosphereLUTPass& atmosphere)
                : device_(device), shaders_(shaders), pipelines_(pipelines), layout_(layout),
                  cloudscape_(cloudscape), light_volume_(light_volume), noise_(noise),
                  atmosphere_(atmosphere)
            {
                create_pipeline();
            }

            CloudPass::~CloudPass() { destroy_pipeline(); }

            void CloudPass::create_pipeline()
            {
                Resources::GraphicsPipelineDescription desc = fullscreen_pipeline_desc(
                    layout_.pipeline_layout(), shaders_.module("fullscreen.vert"),
                    shaders_.module("cloud.frag"), Frame::HDR_FORMAT);
                // Second MRT slot: the W3 transmittance-weighted mean march depth
                // (frame.targets.cloud_depth), the aerial-perspective coupling's input.
                desc.color_formats[1] = VK_FORMAT_R32_SFLOAT;
                desc.color_count = 2;
                // Whether a rate image is actually bound is decided per frame (see
                // register_pass), but the pipeline has to be created knowing one may be, so
                // on a device that supports it the cloud pipeline always opts in — the same
                // opt-in sky_pass makes for the frame's other heavy per-pixel march.
                desc.shading_rate_attachment =
                    device_.supports_shading_rate_image() ? VK_TRUE : VK_FALSE;
                pipeline_ = pipelines_.create(desc);
            }

            void CloudPass::destroy_pipeline()
            {
                // The factory owns the pipeline and swaps in its optimized rebuild, so
                // the pass drops only its handle; clear_libraries() frees the pipeline.
                pipeline_ = Resources::PipelineHandle{};
            }

            void CloudPass::rebuild_pipelines()
            {
                destroy_pipeline();
                create_pipeline();
            }

            void CloudPass::register_pass(Graph::RenderGraph& graph,
                                          const Frame::FrameContext& frame)
            {
                // The author's toggle. Disabled, the march would immediately write the
                // shader's own neutral (0,0,0,1) "clear sky" and return — so a hardware
                // attachment clear reproduces that exact output without paying for the
                // descriptor writes, push constants, or the draw itself, and without the
                // graph ever reading frame.targets.cloud while it holds undefined content.
                const bool clouds_on =
                    frame.environment != nullptr && frame.environment->clouds.enabled;

                graph.add_pass(
                    "clouds",
                    [&](Graph::RenderPassBuilder& builder)
                    {
                        builder.color_attachment(0, frame.targets.cloud,
                                                 clouds_on ? Graph::AttachmentLoad::Discard
                                                           : Graph::AttachmentLoad::Clear);
                        builder.color_attachment(1, frame.targets.cloud_depth,
                                                 clouds_on ? Graph::AttachmentLoad::Discard
                                                           : Graph::AttachmentLoad::Clear);
                        if (!clouds_on)
                            return;
                        builder.read(frame.targets.depth, Graph::TextureAccess::SampledFragment);
                        builder.read(frame.targets.uniforms, Graph::BufferAccess::UniformRead);
                        builder.read(frame.targets.temporal, Graph::BufferAccess::UniformRead);
                        // The march is the frame's other heavy per-pixel fill (with the sky),
                        // so it is worth shading below one sample per pixel wherever the
                        // luminance/velocity mask says the difference cannot be seen. The
                        // mask was built for the full-resolution frame; halving its texel
                        // size keeps a tile's physical footprint the same at this pass's half
                        // resolution instead of covering four times the screen area per tile.
                        builder.shading_rate_attachment(
                            frame.targets.shading_rate,
                            std::max<std::uint32_t>(1u, device_.shading_rate_texel_width() / 2),
                            std::max<std::uint32_t>(1u, device_.shading_rate_texel_height() / 2));
                    },
                    [this, &frame, clouds_on](VkCommandBuffer cmd,
                                              const Graph::PassContext& context)
                    {
                        if (!clouds_on)
                            return;

                        // Point, not linear: this feeds the march's occlusion bound, and a
                        // bilinear tap across a silhouette blends near and far scene depth
                        // into a value that matches neither, letting the march either poke
                        // through the edge of foreground geometry or clip short in front of
                        // it. A point sample always reads one real depth.
                        Resources::SamplerDescription point{};
                        point.filter = VK_FILTER_NEAREST;
                        point.mipmap_mode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
                        const VkSampler depth_sampler = frame.samplers->get(point);

                        Scene::SceneSetWriter writer;
                        writer.uniform(Scene::SceneLayout::SCENE_BINDING,
                                       context.buffer(frame.targets.uniforms),
                                       sizeof(Scene::SceneUniforms));
                        writer.image(1, context.sampled_view(frame.targets.depth), depth_sampler);
                        // Bindings 2-5: the shared SceneLayout reserves 7/8/9/10 by name
                        // (MATERIAL_BINDING, MOTION_BINDING, TEMPORAL_BINDING,
                        // SHADOW_BINDING) as a different descriptor type each; only 1-6
                        // are genuinely free per-pass image slots. Kept in GENERAL across
                        // CloudscapeCompilePass's/CloudLightVolumePass's own compute
                        // builds, like the atmosphere LUTs and the fog volume this same
                        // descriptor set already samples elsewhere in the frame.
                        writer.image(2, cloudscape_.field_view(), cloudscape_.sampler(),
                                     VK_IMAGE_LAYOUT_GENERAL);
                        writer.image(3, cloudscape_.skip_view(), cloudscape_.sampler(),
                                     VK_IMAGE_LAYOUT_GENERAL);
                        // The precombined march noise volume the CloudsV2 analytic carve
                        // reads at every distance; see cloud.frag's cloud_density_carved.
                        // Through its own sampler, which is the one allowed to reach the
                        // volume's coarser levels — the shared one stops at level 0 and would
                        // pin the carve to its finest detail at every distance, which is the
                        // aliasing CV3 is about.
                        writer.image(4, noise_.march(), noise_.march_sampler());
                        writer.image(5, light_volume_.view(), light_volume_.sampler(),
                                     VK_IMAGE_LAYOUT_GENERAL);
                        // The far window, in the last free per-pass image slot. It is where the
                        // simulation's weather now reaches the march: the bake resolved coverage
                        // per column, so the march reads the answer instead of the meteorology,
                        // which is exactly what freed this binding up.
                        writer.image(6, cloudscape_.far_view(), cloudscape_.sampler(),
                                     VK_IMAGE_LAYOUT_GENERAL);
                        // The sun's transmittance through the atmosphere, on the shared
                        // SceneLayout slot SkyPass already writes. Without it the march
                        // lights every cloud with the top-of-atmosphere beam: identical at
                        // midnight and at noon, which is why the deck stayed white after
                        // sunset while the sky — which does read this LUT — went black.
                        // Stays in GENERAL across AtmosphereLUTPass's own compute build,
                        // like the other LUTs this descriptor set samples elsewhere.
                        const VkSampler lut_sampler =
                            frame.samplers->get(Resources::SamplerDescription{});
                        writer.image(Scene::SceneLayout::TRANSMITTANCE_LUT_BINDING,
                                     atmosphere_.transmittance_view(), lut_sampler,
                                     VK_IMAGE_LAYOUT_GENERAL);
                        // The sky's own radiance per direction. Once the sun is down this is
                        // what still lights a deck — a cloudy twilight reads grey because the
                        // *sky* is bright, not because the sun is — and a single authored
                        // ambient constant can carry neither that magnitude nor its colour.
                        writer.image(Scene::SceneLayout::SKY_VIEW_LUT_BINDING,
                                     atmosphere_.sky_view_view(), lut_sampler,
                                     VK_IMAGE_LAYOUT_GENERAL);
                        writer.uniform(Scene::SceneLayout::TEMPORAL_BINDING,
                                       context.buffer(frame.targets.temporal),
                                       sizeof(Scene::TemporalUniforms));
                        writer.commit(cmd, frame.layout->pipeline_layout());

                        frame.layout->bind_heap(cmd);
                        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_.get());
                        // The tier's march budget rides the shared push range's fragment
                        // bytes, which this fullscreen pass otherwise leaves unused. Mirrors
                        // cloud.frag's CloudBudget block, byte for byte — including the two
                        // padding words, which exist because the spread array is a vec4[2] and
                        // std430 aligns it to sixteen.
                        struct
                        {
                            std::uint32_t steps_near;
                            std::uint32_t steps_far;
                            std::uint32_t light_steps;
                            std::uint32_t light_taps;
                            std::uint32_t near_far_split;
                            std::uint32_t carve_mip_count;
                            // NDC per buffer row of this pass's own target, which is what the
                            // carve turns into a sample's lateral world footprint. Taken from
                            // the graph's own description of the target rather than
                            // recomputing the tier's resolution scale, so the two can never
                            // disagree about how big this buffer is.
                            float pixel_ndc;
                            std::uint32_t padding;
                            // Per-mip standard deviation of the carve shape each level filtered
                            // away, measured from the generated volume (CloudNoise::
                            // measure_carve_spread). The march integrates its coverage
                            // threshold over this instead of thresholding a filtered mean,
                            // which is what keeps distant coverage correct rather than
                            // all-or-nothing.
                            float carve_spread[CARVE_SPREAD_CAPACITY];
                        } budget{frame.quality.cloud_primary_steps_near,
                                 frame.quality.cloud_primary_steps_far,
                                 frame.quality.cloud_light_steps,
                                 frame.quality.cloud_light_taps,
                                 frame.quality.cloud_near_far_split ? 1u : 0u,
                                 0u,
                                 0.0f,
                                 0u,
                                 {}};

                        const std::uint32_t cloud_height = std::max<std::uint32_t>(
                            1u, context.texture_desc(frame.targets.cloud).height);
                        budget.pixel_ndc = 2.0f / static_cast<float>(cloud_height);

                        const std::vector<float>& spread = noise_.march_carve_spread();
                        budget.carve_mip_count = static_cast<std::uint32_t>(
                            std::min<std::size_t>(spread.size(), CARVE_SPREAD_CAPACITY));
                        for (std::uint32_t level = 0; level < budget.carve_mip_count; ++level)
                            budget.carve_spread[level] = spread[level];
                        // The shared push-constant range is declared VERTEX|FRAGMENT (see
                        // SceneLayout::MeshPushConstants), so a push touching any of its bytes
                        // must cover both stages even though only the fragment shader reads these.
                        vkCmdPushConstants(cmd, layout_.pipeline_layout(),
                                           VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
                                           0, sizeof(budget), &budget);
                        vkCmdDraw(cmd, 3, 1, 0, 0);
                    });
            }
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
