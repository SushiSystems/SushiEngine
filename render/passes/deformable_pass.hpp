/**************************************************************************/
/* deformable_pass.hpp                                                    */
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
 * @file deformable_pass.hpp
 * @brief Shades the frame's host-simulated surfaces on the GPU.
 *
 * Runs just before the opaque pass. For each deformable mesh the host packed positions for,
 * it dispatches one thread per vertex to gather the area-weighted normal and write the
 * MeshVertex — the work the CPU used to do every frame. The indices are not touched: they
 * are uploaded mesh-local and the draw supplies the vertex offset, so what used to be a
 * second dispatch mode is now a memcpy the buffers do.
 *
 * The vertex buffer it fills is the one the opaque pass draws, so it hand-barriers the
 * compute write to the vertex-input read — the buffer is DeformableBuffers-owned, not a
 * graph resource, so the graph cannot derive that dependency.
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
            class DeformableBuffers;
        }

        namespace Passes
        {
            /** @brief Fills the deformable vertex buffer from packed positions and topology. */
            class DeformablePass : public IRenderPass
            {
                public:
                    /**
                     * @brief Builds the shading pipeline.
                     * @param device     The live Vulkan device.
                     * @param shaders    The catalogue the deformable module comes from.
                     * @param pipelines  The factory owning the compute pipeline.
                     * @param deformable The per-frame deformable buffers to fill.
                     */
                    DeformablePass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                                   Resources::GraphicsPipelineFactory& pipelines,
                                   Geometry::DeformableBuffers& deformable);
                    ~DeformablePass() override;

                    DeformablePass(const DeformablePass&) = delete;
                    DeformablePass& operator=(const DeformablePass&) = delete;

                    void register_pass(Graph::RenderGraph& graph,
                                       const Frame::FrameContext& frame) override;
                    void rebuild_pipelines() override;

                private:
                    struct Push
                    {
                        std::uint32_t a[4]; /**< vertex count, base vertex, base index, adjacency range base. */
                        std::uint32_t b[4]; /**< adjacency triangle base. */
                        float origin[4];    /**< camera-relative mesh origin. */
                    };

                    void create_pipeline();
                    void destroy_pipeline();

                    Vulkan::VulkanDevice& device_;
                    Resources::ShaderLibrary& shaders_;
                    Resources::GraphicsPipelineFactory& pipelines_;
                    Geometry::DeformableBuffers& deformable_;

                    VkDescriptorSetLayout set_layout_ = VK_NULL_HANDLE;
                    VkPipelineLayout pipeline_layout_ = VK_NULL_HANDLE;
                    VkPipeline pipeline_ = VK_NULL_HANDLE;
            };
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
