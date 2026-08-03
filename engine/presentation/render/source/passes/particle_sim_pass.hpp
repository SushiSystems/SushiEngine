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
            class ParticleSystem;
        }

        namespace Passes
        {
            class HiZPass;
            class IrradianceVolumePass;

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
                     * @param hiz       The depth pyramid the collision test reads (last frame's,
                     *                  since this pass runs before the depth prepass).
                     * @param volumes   The pass owning the GI distance field, the collision
                     *                  surface for emitters that must collide off screen.
                     */
                    ParticleSimPass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                                    Resources::GraphicsPipelineFactory& pipelines,
                                    Scene::ParticleSystem& particles, HiZPass& hiz,
                                    IrradianceVolumePass& volumes);
                    ~ParticleSimPass() override;

                    ParticleSimPass(const ParticleSimPass&) = delete;
                    ParticleSimPass& operator=(const ParticleSimPass&) = delete;

                    void register_pass(Graph::RenderGraph& graph,
                                       const Frame::FrameContext& frame) override;
                    void rebuild_pipelines() override;

                private:
                    /** @brief Binding of last frame's depth pyramid, read by the collision test. */
                    static constexpr std::uint32_t DEPTH_PYRAMID_BINDING = 11;
                    /** @brief Binding of the GI distance field, the off-screen collision surface. */
                    static constexpr std::uint32_t SDF_CLIPMAP_BINDING = 12;
                    /** @brief Binding of the distance field's camera-relative parameterization. */
                    static constexpr std::uint32_t SDF_CONFIG_BINDING = 13;
                    /** @brief Bindings in the shared emit/simulate set. */
                    static constexpr std::uint32_t BINDING_COUNT = 14;

                    /**
                     * @brief 128-byte compute constants, shared by the emit and simulate dispatches.
                     *
                     * The camera block is here for the collision tests alone — the depth path
                     * projects each particle into the pyramid, the distance-field path rebases it
                     * against the eye — while the emit dispatch reads only the emitter index.
                     */
                    struct Push
                    {
                        float view_projection[16];
                        float camera_right[4];       /**< xyz right; w = tan(half horizontal fov). */
                        float camera_up[4];          /**< xyz up; w = tan(half vertical fov). */
                        /** @brief x emitter index, y capacity, z pyramid usable, w field usable. */
                        std::uint32_t counts[4];
                        float misc[4];               /**< x = simulation step; yzw = eye position. */
                    };

                    void create_pipelines();
                    void destroy_pipelines();
                    void create_fallback_depth();
                    void destroy_fallback_depth();
                    void create_fallback_field();
                    void destroy_fallback_field();

                    Vulkan::VulkanDevice& device_;
                    Resources::ShaderLibrary& shaders_;
                    Resources::GraphicsPipelineFactory& pipelines_;
                    Scene::ParticleSystem& particles_;
                    HiZPass& hiz_;
                    IrradianceVolumePass& volumes_;

                    VkDescriptorSetLayout set_layout_ = VK_NULL_HANDLE;
                    VkPipelineLayout pipeline_layout_ = VK_NULL_HANDLE;
                    VkPipeline emit_pipeline_ = VK_NULL_HANDLE;
                    VkPipeline simulate_pipeline_ = VK_NULL_HANDLE;

                    /**
                     * @brief A 1x1 stand-in bound when the depth pyramid holds no usable frame.
                     *
                     * A combined-image-sampler binding needs a real view even on a frame where the
                     * shader will not read it (the collision is switched off through the push
                     * constant), and the pyramid does not exist before its first build or while
                     * screen-space reflections are off.
                     */
                    VkImage fallback_image_ = VK_NULL_HANDLE;
                    VmaAllocation fallback_allocation_ = VK_NULL_HANDLE;
                    VkImageView fallback_view_ = VK_NULL_HANDLE;
                    bool fallback_ready_ = false;

                    /**
                     * @brief A 1x1x1 volume and an empty config block, for the same reason.
                     *
                     * The distance field belongs to the GI tier and is absent whenever that tier
                     * is off; the bindings still have to resolve, so they resolve to these and the
                     * shader is told through the push constant not to read them.
                     */
                    VkImage fallback_field_image_ = VK_NULL_HANDLE;
                    VmaAllocation fallback_field_allocation_ = VK_NULL_HANDLE;
                    VkImageView fallback_field_view_ = VK_NULL_HANDLE;
                    VkBuffer fallback_config_ = VK_NULL_HANDLE;
                    VmaAllocation fallback_config_allocation_ = VK_NULL_HANDLE;
                    bool fallback_field_ready_ = false;
            };
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
