/**************************************************************************/
/* cull_pass.hpp                                                          */
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
 * @file cull_pass.hpp
 * @brief The GPU instance cull: frustum, screen-coverage LOD, and Hi-Z occlusion.
 *
 * The compute half of the GPU-driven geometry path. It runs before the depth prepass and,
 * from the frame's packed instance records, tests every instance against the view frustum,
 * its own on-screen size, and last frame's occlusion pyramid, then compacts the survivors
 * per bucket and writes each bucket's `VkDrawIndexedIndirectCommand` — the instance count
 * the draw passes then submit indirectly, with no CPU readback in the loop. A small counter
 * of what survived is read back one frame late, only for the editor's cull statistics.
 */

#include <array>
#include <cstdint>

#include <vk_mem_alloc.h>
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

        namespace Scene
        {
            class InstanceSystem;
        }

        namespace Passes
        {
            class OcclusionPass;

            /** @brief The per-frame cull counts the editor surfaces. */
            struct CullStatistics
            {
                std::uint32_t drawn = 0;  /**< Instances that survived the cull. */
                std::uint32_t tested = 0; /**< Instances the cull considered. */
            };

            /** @brief Frustum/coverage/occlusion-culls instances into indirect draw commands. */
            class CullPass : public IRenderPass
            {
                public:
                    /**
                     * @brief Builds the cull pipeline and the per-slot statistics readbacks.
                     * @param device    The live Vulkan device.
                     * @param shaders   The catalogue the cull module comes from.
                     * @param pipelines The factory owning the compute pipeline.
                     * @param occlusion The occlusion pyramid this pass tests against.
                     * @param instances The system that packed this frame's instance records.
                     */
                    CullPass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                             Resources::GraphicsPipelineFactory& pipelines, OcclusionPass& occlusion,
                             Scene::InstanceSystem& instances);
                    ~CullPass() override;

                    CullPass(const CullPass&) = delete;
                    CullPass& operator=(const CullPass&) = delete;

                    void register_pass(Graph::RenderGraph& graph,
                                       const Frame::FrameContext& frame) override;
                    void rebuild_pipelines() override;

                    /**
                     * @brief The cull counts a slot produced, read back after its fence.
                     * @param slot The frame slot to read.
                     * @return The drawn and tested counts, zero until the slot has rendered.
                     */
                    CullStatistics statistics(std::uint32_t slot) const;

                private:
                    /** @brief How many frames may be in flight; matches the view's slot count. */
                    static constexpr std::uint32_t SLOTS = 3;

                    /** @brief Words in the stats buffer: drawn, tested, and two spare. */
                    static constexpr std::uint32_t STATS_WORDS = 4;

                    struct Push
                    {
                        std::uint32_t mode;
                        std::uint32_t bucket_count;
                        std::uint32_t candidate_count;
                        std::uint32_t pad;
                    };

                    /** @brief The std140 cull parameters, mirrored by cull.comp's CullParams. */
                    struct Params
                    {
                        float previous_view_projection[16];
                        float delta_eye[4];
                        float hiz[4];   /**< mip count, width, height, near. */
                        float flags[4]; /**< frustum on, occlusion on, min diameter, spare. */
                    };

                    /** @brief One slot's cull-params UBO, device stats buffer, and readback. */
                    struct SlotBuffers
                    {
                        VkBuffer params = VK_NULL_HANDLE;
                        VmaAllocation params_allocation = VK_NULL_HANDLE;
                        void* params_mapped = nullptr;
                        VkBuffer stats = VK_NULL_HANDLE;
                        VmaAllocation stats_allocation = VK_NULL_HANDLE;
                        VkBuffer readback = VK_NULL_HANDLE;
                        VmaAllocation readback_allocation = VK_NULL_HANDLE;
                        void* readback_mapped = nullptr;
                    };

                    void create_pipeline();
                    void destroy_pipeline();

                    Vulkan::VulkanDevice& device_;
                    Resources::ShaderLibrary& shaders_;
                    Resources::GraphicsPipelineFactory& pipelines_;
                    OcclusionPass& occlusion_;
                    Scene::InstanceSystem& instances_;

                    VkDescriptorSetLayout set_layout_ = VK_NULL_HANDLE;
                    VkPipelineLayout pipeline_layout_ = VK_NULL_HANDLE;
                    VkPipeline pipeline_ = VK_NULL_HANDLE;
                    std::array<SlotBuffers, SLOTS> slots_{};
            };
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
