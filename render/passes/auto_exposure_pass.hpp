/**************************************************************************/
/* auto_exposure_pass.hpp                                                 */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/* Licensed under the Apache License, Version 2.0 (the "License");        */
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
 * @file auto_exposure_pass.hpp
 * @brief Eye adaptation: a luminance histogram of the resolved scene, reduced on the host.
 *
 * A compute pass builds a 256-bin log-luminance histogram of the frame the display transform
 * will read, copies it into a per-slot host-visible buffer, and does nothing else on the GPU.
 * The scene view reads that buffer back after the slot's fence — the same after-the-fence path
 * the picking readback uses — and eases a stored exposure toward the value that maps the
 * frame's central luminance mass onto middle grey. Keeping the reduction on the CPU is what
 * lets the exposure ride in the CPU-filled post block instead of costing a second scene-set
 * binding, of which the push set has none to spare.
 */

#include <cstdint>

#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>

#include <SushiEngine/render/render_settings.hpp>

#include "passes/render_pass.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Resources
        {
            class ShaderLibrary;
            class GraphicsPipelineFactory;
        }

        namespace Vulkan
        {
            class VulkanDevice;
        }

        namespace Passes
        {
            /** @brief Builds the luminance histogram and adapts the scene's exposure. */
            class AutoExposurePass final : public IRenderPass
            {
                public:
                    /** @brief How many frames may be in flight (matches ViewResources::SLOTS). */
                    static constexpr std::uint32_t SLOTS = 2;

                    /** @brief Bins in the log-luminance histogram. */
                    static constexpr std::uint32_t BINS = 256;

                    /**
                     * @brief Creates the histogram pipeline and the per-slot buffers.
                     * @param device    The live Vulkan device.
                     * @param shaders   The library the compute module comes from.
                     * @param pipelines The factory the compute pipeline is built by.
                     */
                    AutoExposurePass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                                     Resources::GraphicsPipelineFactory& pipelines);
                    ~AutoExposurePass() override;

                    AutoExposurePass(const AutoExposurePass&) = delete;
                    AutoExposurePass& operator=(const AutoExposurePass&) = delete;

                    void register_pass(Graph::RenderGraph& graph,
                                       const Frame::FrameContext& frame) override;
                    void rebuild_pipelines() override;

                    /**
                     * @brief Eases the exposure toward the value the last histogram in this slot
                     *        implies, and returns it.
                     *
                     * Reads the slot's host-visible histogram (written two frames ago and now
                     * safe past the slot's fence), takes the average log-luminance of its central
                     * mass with the tails trimmed, clamps it to the authored EV range, and moves
                     * @p current toward the exposure that maps that luminance onto the key. When
                     * the histogram is empty (auto exposure was just enabled) @p current is
                     * returned unchanged.
                     *
                     * @param slot     The frame slot whose histogram to read.
                     * @param settings The authored auto-exposure parameters.
                     * @param delta_seconds Time since the last adaptation, for the ease rate.
                     * @param current  The exposure carried from the previous frame.
                     * @return The adapted exposure to apply this frame.
                     */
                    float adapt(std::uint32_t slot, const AutoExposureSettings& settings,
                                float delta_seconds, float current) const;

                private:
                    struct SlotBuffers
                    {
                        VkBuffer histogram = VK_NULL_HANDLE;      /**< Device-local histogram. */
                        VmaAllocation histogram_allocation = VK_NULL_HANDLE;
                        VkBuffer readback = VK_NULL_HANDLE;       /**< Host-visible copy. */
                        VmaAllocation readback_allocation = VK_NULL_HANDLE;
                        void* readback_mapped = nullptr;
                    };

                    struct Push
                    {
                        float a[4];
                    };

                    void create_pipeline();
                    void destroy_pipeline();

                    Vulkan::VulkanDevice& device_;
                    Resources::ShaderLibrary& shaders_;
                    Resources::GraphicsPipelineFactory& pipelines_;

                    VkDescriptorSetLayout set_layout_ = VK_NULL_HANDLE;
                    VkPipelineLayout pipeline_layout_ = VK_NULL_HANDLE;
                    VkPipeline pipeline_ = VK_NULL_HANDLE;
                    SlotBuffers slots_[SLOTS];
            };
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
