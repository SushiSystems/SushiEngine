/**************************************************************************/
/* skinning_pass.hpp                                                     */
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
 * @file skinning_pass.hpp
 * @brief Compute pre-skinning: skins every skinned character once per frame (design §6.2).
 *
 * Runs before the depth prepass. For each skinned instance the host laid out (see
 * `SkinningSystem`), it dispatches one thread per vertex to linear-blend-skin the base
 * vertex by its instance's joint palette (current and previous frame) and write an
 * interleaved SkinnedVertex into the frame's output buffer. The opaque pass then draws each
 * instance's slice as a static mesh. Like `ClothPass`, the output buffer is
 * SkinningSystem-owned rather than a graph resource, so the write is hand-barriered from the
 * compute stage to the vertex-input read.
 */

#include <cstdint>

#include <vulkan/vulkan.h>

#include "passes/render_pass.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Vulkan
        {
            class VulkanDevice;
        }

        namespace Resources
        {
            class ShaderLibrary;
            class GraphicsPipelineFactory;
        }

        namespace Geometry
        {
            class MeshRegistry;
        }

        namespace Scene
        {
            class SkinningSystem;
        }

        namespace Passes
        {
            /** @brief Fills the frame's skinned-vertex buffer from base meshes and palettes. */
            class SkinningPass : public IRenderPass
            {
                public:
                    /**
                     * @brief Builds the skinning compute pipeline.
                     * @param device    The live Vulkan device.
                     * @param shaders   The catalogue skinning.comp comes from.
                     * @param pipelines The factory owning the compute pipeline.
                     * @param skinning  The per-frame palette and output buffers to fill.
                     * @param meshes    The registry the base vertices and skin streams come from.
                     */
                    SkinningPass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                                 Resources::GraphicsPipelineFactory& pipelines,
                                 Scene::SkinningSystem& skinning, Geometry::MeshRegistry& meshes);
                    ~SkinningPass() override;

                    SkinningPass(const SkinningPass&) = delete;
                    SkinningPass& operator=(const SkinningPass&) = delete;

                    void register_pass(Graph::RenderGraph& graph,
                                       const Frame::FrameContext& frame) override;
                    void rebuild_pipelines() override;

                private:
                    struct Push
                    {
                        std::uint32_t vertex_count;
                        std::uint32_t palette_base;
                        std::uint32_t out_base;
                        std::uint32_t prev_valid;
                    };

                    void create_pipeline();
                    void destroy_pipeline();

                    Vulkan::VulkanDevice& device_;
                    Resources::ShaderLibrary& shaders_;
                    Resources::GraphicsPipelineFactory& pipelines_;
                    Scene::SkinningSystem& skinning_;
                    Geometry::MeshRegistry& meshes_;

                    VkDescriptorSetLayout set_layout_ = VK_NULL_HANDLE;
                    VkPipelineLayout pipeline_layout_ = VK_NULL_HANDLE;
                    VkPipeline pipeline_ = VK_NULL_HANDLE;
            };
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
