/**************************************************************************/
/* particle_sort_pass.hpp                                                 */
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
 * @file particle_sort_pass.hpp
 * @brief Bitonic depth-sort of the true-alpha particle bucket (VFX2a).
 *
 * Runs between the particle simulate and draw passes. It seeds one {distance, index} key per pool
 * slot from the alpha list's camera distance, then bitonic-sorts them back-to-front with a fixed
 * `log2(N)*(log2(N)+1)/2` sequence of compute dispatches over the power-of-two pool capacity — the
 * engine's "dispatch a host-known max, read the alive count on the GPU" idiom, no indirect dispatch.
 * The seeding stage always runs (so the key buffer has a producer); the expensive bitonic stages
 * run only when an active emitter is true-alpha. The alpha draw indexes the alpha list through the
 * sorted keys.
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
            /** @brief Depth-sorts the alpha particle bucket back-to-front on the GPU. */
            class ParticleSortPass : public IRenderPass
            {
                public:
                    /**
                     * @brief Builds the bitonic-sort compute pipeline.
                     * @param device    The live Vulkan device.
                     * @param shaders   The catalogue particle_sort.comp comes from.
                     * @param pipelines The factory owning the compute pipeline.
                     * @param particles The system whose alpha bucket and keys are sorted.
                     */
                    ParticleSortPass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                                     Resources::GraphicsPipelineFactory& pipelines,
                                     Scene::ParticleSystem& particles);
                    ~ParticleSortPass() override;

                    ParticleSortPass(const ParticleSortPass&) = delete;
                    ParticleSortPass& operator=(const ParticleSortPass&) = delete;

                    void register_pass(Graph::RenderGraph& graph,
                                       const Frame::FrameContext& frame) override;
                    void rebuild_pipelines() override;

                private:
                    struct Push
                    {
                        std::uint32_t mode;
                        std::uint32_t stage_k;
                        std::uint32_t stage_j;
                        std::uint32_t count;
                        float eye[4];
                    };

                    void create_pipeline();
                    void destroy_pipeline();

                    Vulkan::VulkanDevice& device_;
                    Resources::ShaderLibrary& shaders_;
                    Resources::GraphicsPipelineFactory& pipelines_;
                    Scene::ParticleSystem& particles_;

                    VkDescriptorSetLayout set_layout_ = VK_NULL_HANDLE;
                    VkPipelineLayout pipeline_layout_ = VK_NULL_HANDLE;
                    VkPipeline pipeline_ = VK_NULL_HANDLE;
            };
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
