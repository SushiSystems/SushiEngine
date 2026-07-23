/**************************************************************************/
/* cloth_pass.hpp                                                         */
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
 * @file cloth_pass.hpp
 * @brief Triangulates the frame's soft bodies on the GPU (Phase 10 item 6).
 *
 * Runs just before the opaque pass. For each soft-body grid the host packed positions for,
 * it dispatches one thread per vertex to compute the area-weighted normal and write the
 * MeshVertex, and one thread per quad to write the indices — the work the CPU used to do
 * every frame. The vertex and index buffers it fills are the ones the opaque pass then
 * draws, so it hand-barriers them from the compute write to the vertex-input read (they are
 * ClothBuffers-owned, not graph resources, so the graph cannot derive that dependency).
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
            class ClothBuffers;
        }

        namespace Passes
        {
            /** @brief Fills the soft-body vertex and index buffers from packed positions. */
            class ClothPass : public IRenderPass
            {
                public:
                    /**
                     * @brief Builds the triangulation pipeline.
                     * @param device    The live Vulkan device.
                     * @param shaders   The catalogue the cloth module comes from.
                     * @param pipelines The factory owning the compute pipeline.
                     * @param cloth     The per-frame soft-body buffers to fill.
                     */
                    ClothPass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                              Resources::GraphicsPipelineFactory& pipelines,
                              Geometry::ClothBuffers& cloth);
                    ~ClothPass() override;

                    ClothPass(const ClothPass&) = delete;
                    ClothPass& operator=(const ClothPass&) = delete;

                    void register_pass(Graph::RenderGraph& graph,
                                       const Frame::FrameContext& frame) override;
                    void rebuild_pipelines() override;

                private:
                    struct Push
                    {
                        std::uint32_t a[4]; /**< mode, rows, cols, base vertex. */
                        std::uint32_t b[4]; /**< base index. */
                        float origin[4];    /**< camera-relative strand origin. */
                    };

                    void create_pipeline();
                    void destroy_pipeline();

                    Vulkan::VulkanDevice& device_;
                    Resources::ShaderLibrary& shaders_;
                    Resources::GraphicsPipelineFactory& pipelines_;
                    Geometry::ClothBuffers& cloth_;

                    VkDescriptorSetLayout set_layout_ = VK_NULL_HANDLE;
                    VkPipelineLayout pipeline_layout_ = VK_NULL_HANDLE;
                    VkPipeline pipeline_ = VK_NULL_HANDLE;
            };
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
