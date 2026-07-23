/**************************************************************************/
/* motion_blur_pass.hpp                                                   */
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
 * @file motion_blur_pass.hpp
 * @brief Per-pixel velocity motion blur, after depth of field in the post chain.
 *
 * A fullscreen pass on the shared scene layout: it reads the previous stage's colour and the
 * shipped velocity target and gathers the colour along each pixel's screen-space motion so a
 * moving surface smears the way a finite shutter would. Runs only when the tier permits it and
 * the author enables it.
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
            /** @brief Velocity motion blur written into the frame's motion-blur target. */
            class MotionBlurPass final : public IRenderPass
            {
                public:
                    /**
                     * @brief Builds the motion-blur pipeline against the shared scene layout.
                     * @param device    The live Vulkan device.
                     * @param shaders   The library the fragment module comes from.
                     * @param pipelines The factory the pipeline is built by.
                     * @param layout    The shared scene pipeline layout.
                     */
                    MotionBlurPass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                                   Resources::GraphicsPipelineFactory& pipelines,
                                   Scene::SceneLayout& layout);
                    ~MotionBlurPass() override;

                    MotionBlurPass(const MotionBlurPass&) = delete;
                    MotionBlurPass& operator=(const MotionBlurPass&) = delete;

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
