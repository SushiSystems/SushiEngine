/**************************************************************************/
/* dof_pass.cpp                                                           */
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

#include "passes/dof_pass.hpp"

#include "frame/frame_context.hpp"
#include "graph/render_graph.hpp"
#include "passes/fullscreen.hpp"
#include "resources/sampler_cache.hpp"
#include "resources/shader_library.hpp"
#include "scene/post_process_uniforms.hpp"
#include "scene/scene_layout.hpp"
#include "rhi/vulkan/vulkan_device.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Passes
        {
            DofPass::DofPass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                             Resources::GraphicsPipelineFactory& pipelines,
                             Scene::SceneLayout& layout)
                : device_(device), shaders_(shaders), pipelines_(pipelines), layout_(layout)
            {
                create_pipeline();
            }

            DofPass::~DofPass() { destroy_pipeline(); }

            void DofPass::create_pipeline()
            {
                pipeline_ = pipelines_.create(fullscreen_pipeline_desc(
                    layout_.pipeline_layout(), shaders_.module("fullscreen.vert"),
                    shaders_.module("dof.frag"), Frame::HDR_FORMAT));
            }

            void DofPass::destroy_pipeline() { pipeline_ = Resources::PipelineHandle{}; }

            void DofPass::rebuild_pipelines()
            {
                destroy_pipeline();
                create_pipeline();
            }

            void DofPass::register_pass(Graph::RenderGraph& graph,
                                        const Frame::FrameContext& frame)
            {
                if (!frame.targets.dof.valid())
                    return;

                const Graph::TextureHandle source =
                    frame.temporal_enabled() ? frame.targets.resolved : frame.targets.scene_final;
                const Graph::TextureHandle depth = frame.targets.depth;
                const Graph::TextureHandle output = frame.targets.dof;

                graph.add_pass(
                    "depth of field",
                    [source, depth, output, &frame](Graph::RenderPassBuilder& builder)
                    {
                        builder.color_attachment(0, output, Graph::AttachmentLoad::Discard);
                        builder.read(source, Graph::TextureAccess::SampledFragment);
                        builder.read(depth, Graph::TextureAccess::SampledFragment);
                        builder.read(frame.targets.post, Graph::BufferAccess::UniformRead);
                    },
                    [this, &frame, source, depth](VkCommandBuffer cmd,
                                                  const Graph::PassContext& context)
                    {
                        const VkSampler sampler = frame.samplers->get(Resources::SamplerDesc{});
                        Resources::SamplerDesc point{};
                        point.filter = VK_FILTER_NEAREST;
                        point.mipmap_mode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
                        const VkSampler depth_sampler = frame.samplers->get(point);
                        Scene::SceneSetWriter writer;
                        writer.image(1, context.sampled_view(source), sampler);
                        writer.image(2, context.sampled_view(depth), depth_sampler);
                        writer.uniform(Scene::SceneLayout::POST_BINDING,
                                       context.buffer(frame.targets.post),
                                       sizeof(Scene::PostProcessUniforms));
                        writer.commit(cmd, frame.layout->pipeline_layout());

                        frame.layout->bind_heap(cmd);
                        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_.get());
                        vkCmdDraw(cmd, 3, 1, 0, 0);
                    });
            }
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
