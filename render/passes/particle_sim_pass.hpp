/**************************************************************************/
/* particle_sim_pass.hpp                                                  */
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
 * @file particle_sim_pass.hpp
 * @brief Compute particle simulation (design §5.3): emit then integrate, once per frame.
 *
 * Runs beside the other compute producers, after skinning and before the depth prepass. Over
 * the shared pool it dispatches a simulate pass (advance, age, retire the dead, append survivors
 * to the frame's compacted draw list) then a per-emitter emit pass (allocate ring slots,
 * initialise new particles, append them too), building the indirect draw the billboard pass
 * consumes. The compacted list and the indirect arguments are graph transients, so the graph
 * derives the compute→draw barriers; the shared pool is ParticleSystem-owned.
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

        namespace Scene
        {
            class ParticleSystem;
        }

        namespace Passes
        {
            /** @brief Emits and integrates the frame's cosmetic particles on the GPU. */
            class ParticleSimPass : public IRenderPass
            {
                public:
                    /**
                     * @brief Builds the emit and simulate compute pipelines.
                     * @param device    The live Vulkan device.
                     * @param shaders   The catalogue the particle compute shaders come from.
                     * @param pipelines The factory owning the compute pipelines.
                     * @param particles The shared pool, emitter table, and LUT atlases to drive.
                     */
                    ParticleSimPass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                                    Resources::GraphicsPipelineFactory& pipelines,
                                    Scene::ParticleSystem& particles);
                    ~ParticleSimPass() override;

                    ParticleSimPass(const ParticleSimPass&) = delete;
                    ParticleSimPass& operator=(const ParticleSimPass&) = delete;

                    void register_pass(Graph::RenderGraph& graph,
                                       const Frame::FrameContext& frame) override;
                    void rebuild_pipelines() override;

                private:
                    struct Push
                    {
                        std::uint32_t emitter_index;
                        std::uint32_t capacity;
                        float dt;
                    };

                    void create_pipelines();
                    void destroy_pipelines();

                    Vulkan::VulkanDevice& device_;
                    Resources::ShaderLibrary& shaders_;
                    Resources::GraphicsPipelineFactory& pipelines_;
                    Scene::ParticleSystem& particles_;

                    VkDescriptorSetLayout set_layout_ = VK_NULL_HANDLE;
                    VkPipelineLayout pipeline_layout_ = VK_NULL_HANDLE;
                    VkPipeline emit_pipeline_ = VK_NULL_HANDLE;
                    VkPipeline simulate_pipeline_ = VK_NULL_HANDLE;
            };
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
