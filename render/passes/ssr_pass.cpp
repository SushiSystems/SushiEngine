/**************************************************************************/
/* ssr_pass.cpp                                                           */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

#include "passes/ssr_pass.hpp"

#include "frame/frame_context.hpp"
#include "graph/render_graph.hpp"
#include "passes/fullscreen.hpp"
#include "passes/hiz_pass.hpp"
#include "resources/pipeline_cache.hpp"
#include "resources/sampler_cache.hpp"
#include "resources/shader_library.hpp"
#include "rhi/vulkan/vulkan_device.hpp"
#include "scene/scene_layout.hpp"
#include "scene/scene_uniforms.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Passes
        {
            SsrPass::SsrPass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                             Resources::GraphicsPipelineFactory& pipelines,
                             Scene::SceneLayout& layout, HizPass& hiz)
                : device_(device), shaders_(shaders), pipelines_(pipelines), layout_(layout),
                  hiz_(hiz)
            {
                create_pipeline();
            }

            SsrPass::~SsrPass() { destroy_pipeline(); }

            void SsrPass::create_pipeline()
            {
                pipeline_ = pipelines_.create(fullscreen_pipeline_desc(
                    layout_.pipeline_layout(), shaders_.module("fullscreen.vert"),
                    shaders_.module("ssr.frag"), Frame::HDR_FORMAT));
            }

            void SsrPass::destroy_pipeline() { pipeline_ = Resources::PipelineHandle{}; }

            void SsrPass::rebuild_pipelines()
            {
                destroy_pipeline();
                create_pipeline();
            }

            void SsrPass::register_pass(Graph::RenderGraph& graph,
                                        const Frame::FrameContext& frame)
            {
                if (!frame.settings.ssr.enabled || !frame.targets.scene_reflected.valid())
                    return;

                const bool trace = hiz_.valid();

                graph.add_pass(
                    "ssr",
                    [&](Graph::RenderPassBuilder& builder)
                    {
                        builder.color_attachment(0, frame.targets.scene_reflected,
                                                 Graph::AttachmentLoad::Discard);
                        builder.read(frame.targets.scene, Graph::TextureAccess::SampledFragment);
                        builder.read(frame.targets.depth, Graph::TextureAccess::SampledFragment);
                        builder.read(frame.targets.gbuffer, Graph::TextureAccess::SampledFragment);
                        builder.read(frame.targets.uniforms, Graph::BufferAccess::UniformRead);
                    },
                    [this, &frame, trace](VkCommandBuffer cmd, const Graph::PassContext& context)
                    {
                        const VkSampler linear = frame.samplers->get(Resources::SamplerDesc{});
                        Resources::SamplerDesc hiz_desc;
                        hiz_desc.filter = VK_FILTER_NEAREST;
                        hiz_desc.mipmap_mode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
                        hiz_desc.max_lod = 16.0f;
                        const VkSampler hiz_sampler = frame.samplers->get(hiz_desc);
                        // The pyramid is pass-owned, so the trace binds its raw view directly;
                        // when it has not been built yet the trace is disabled and the pass just
                        // copies the scene through (the view still needs a valid image to bind).
                        const VkImageView hiz_view =
                            trace ? hiz_.pyramid_view()
                                  : context.sampled_view(frame.targets.scene);

                        Scene::SceneSetWriter writer;
                        writer.uniform(Scene::SceneLayout::SCENE_BINDING,
                                       context.buffer(frame.targets.uniforms),
                                       sizeof(Scene::SceneUniforms));
                        writer.image(1, context.sampled_view(frame.targets.scene), linear);
                        writer.image(2, context.sampled_view(frame.targets.depth), linear);
                        writer.image(3, context.sampled_view(frame.targets.gbuffer), linear);
                        writer.image(4, hiz_view, hiz_sampler);
                        writer.commit(cmd, frame.layout->pipeline_layout());

                        frame.layout->bind_heap(cmd);

                        Push push{};
                        push.p0[0] = static_cast<float>(frame.settings.ssr.max_steps);
                        push.p0[1] = frame.settings.ssr.thickness;
                        push.p0[2] = frame.settings.ssr.roughness_cutoff;
                        push.p0[3] = frame.settings.ssr.intensity;
                        push.p1[0] = trace ? 1.0f : 0.0f;
                        vkCmdPushConstants(cmd, frame.layout->pipeline_layout(),
                                           VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
                                           0, sizeof(Push), &push);

                        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_.get());
                        vkCmdDraw(cmd, 3, 1, 0, 0);
                    });
            }
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
