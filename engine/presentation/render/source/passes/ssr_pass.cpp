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
            SSRPass::SSRPass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                             Resources::GraphicsPipelineFactory& pipelines,
                             Scene::SceneLayout& layout, HiZPass& hiz)
                : device_(device), shaders_(shaders), pipelines_(pipelines), layout_(layout),
                  hiz_(hiz)
            {
                create_pipeline();
            }

            SSRPass::~SSRPass() { destroy_pipeline(); }

            void SSRPass::create_pipeline()
            {
                pipeline_ = pipelines_.create(fullscreen_pipeline_description(
                    layout_.pipeline_layout(), shaders_.module("fullscreen.vert"),
                    shaders_.module("ssr.frag"), Frame::HDR_FORMAT));
            }

            void SSRPass::destroy_pipeline() { pipeline_ = Resources::PipelineHandle{}; }

            void SSRPass::rebuild_pipelines()
            {
                destroy_pipeline();
                create_pipeline();
            }

            void SSRPass::register_pass(Graph::RenderGraph& graph,
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
                    [this, &frame, trace](VkCommandBuffer command,
                                          const Graph::PassContext& context)
                    {
                        const VkSampler linear =
                            frame.samplers->get(Resources::SamplerDescription{});
                        Resources::SamplerDescription hiz_description;
                        hiz_description.filter = VK_FILTER_NEAREST;
                        hiz_description.mipmap_mode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
                        hiz_description.max_lod = 16.0f;
                        const VkSampler hiz_sampler = frame.samplers->get(hiz_description);
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
                        // The pyramid stays in GENERAL across its own build (see hiz_pass.cpp);
                        // the scene fallback is the normal SHADER_READ_ONLY_OPTIMAL color target.
                        writer.image(4, hiz_view, hiz_sampler,
                                    trace ? VK_IMAGE_LAYOUT_GENERAL
                                          : VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
                        writer.commit(command, frame.layout->pipeline_layout());

                        frame.layout->bind_heap(command);

                        Push push{};
                        push.p0[0] = static_cast<float>(frame.settings.ssr.max_steps);
                        push.p0[1] = frame.settings.ssr.thickness;
                        push.p0[2] = frame.settings.ssr.roughness_cutoff;
                        push.p0[3] = frame.settings.ssr.intensity;
                        push.p1[0] = trace ? 1.0f : 0.0f;
                        vkCmdPushConstants(command, frame.layout->pipeline_layout(),
                                           VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
                                           0, sizeof(Push), &push);

                        vkCmdBindPipeline(command, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                          pipeline_.get());
                        vkCmdDraw(command, 3, 1, 0, 0);
                    });
            }
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
