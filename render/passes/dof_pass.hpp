/**************************************************************************/
/* dof_pass.hpp                                                           */
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

#pragma once

/**
 * @file dof_pass.hpp
 * @brief Gather-based depth of field, after the temporal resolve.
 *
 * A fullscreen pass on the shared scene layout: it reads the resolved colour and the prepass
 * depth, computes each pixel's circle of confusion against the focus plane authored in the
 * post block, and gathers a bounded Vogel disc weighted so a sharp foreground does not bleed
 * onto a blurred background. It runs only when the tier permits it and the author enables it.
 */

#include "passes/render_pass.hpp"
#include "resources/pipeline_cache.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Resources
        {
            class ShaderLibrary;
            class GraphicsPipelineFactory;
        }

        namespace Scene
        {
            class SceneLayout;
        }

        namespace Vulkan
        {
            class VulkanDevice;
        }

        namespace Passes
        {
            /** @brief Depth-of-field gather written into the frame's DoF target. */
            class DofPass final : public IRenderPass
            {
                public:
                    /**
                     * @brief Builds the DoF pipeline against the shared scene layout.
                     * @param device    The live Vulkan device.
                     * @param shaders   The library the fragment module comes from.
                     * @param pipelines The factory the pipeline is built by.
                     * @param layout    The shared scene pipeline layout.
                     */
                    DofPass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                            Resources::GraphicsPipelineFactory& pipelines, Scene::SceneLayout& layout);
                    ~DofPass() override;

                    DofPass(const DofPass&) = delete;
                    DofPass& operator=(const DofPass&) = delete;

                    void register_pass(Graph::RenderGraph& graph,
                                       const Frame::FrameContext& frame) override;
                    void rebuild_pipelines() override;

                private:
                    void create_pipeline();
                    void destroy_pipeline();

                    Vulkan::VulkanDevice& device_;
                    Resources::ShaderLibrary& shaders_;
                    Resources::GraphicsPipelineFactory& pipelines_;
                    Scene::SceneLayout& layout_;
                    Resources::PipelineHandle pipeline_;
            };
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
