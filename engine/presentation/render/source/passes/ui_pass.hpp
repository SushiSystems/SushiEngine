/**************************************************************************/
/* ui_pass.hpp                                                            */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/* Licensed under the Apache License, Version 2.0 (the "License");        */
/* you may not use this file except in compliance with the License.       */
/* You may obtain a copy of the License at                                */
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
 * @file ui_pass.hpp
 * @brief The 2D UI overlay: the game's own interface, composited onto the finished image.
 *
 * Runs last of the colour passes, after tone mapping and anti-aliasing, and writes the
 * resolve target directly. That position is the whole point: a UI drawn earlier would be
 * tone mapped (so its colours would drift with the scene's exposure) and anti-aliased (so
 * its text would soften). Drawn here it is exactly the colour it was authored as.
 *
 * One pipeline, one indexed draw, whatever the overlay contains — panels and glyphs share
 * a vertex format and an atlas, so the cost is a single draw call rather than one per
 * widget.
 */

#include <cstdint>

#include <vulkan/vulkan.h>

#include "passes/render_pass.hpp"
#include "resources/pipeline_cache.hpp"

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
            class DescriptorHeap;
        }

        namespace Assets
        {
            class TextureLibrary;
            class FontAtlas;
        }

        namespace Geometry
        {
            class UIBuffers;
        }

        namespace Passes
        {
            /** @brief Draws the frame's 2D UI overlay over the tone-mapped image. */
            class UIPass : public IRenderPass
            {
                public:
                    /**
                     * @brief Builds the overlay pipeline and its layout.
                     * @param device    The live Vulkan device.
                     * @param shaders   The catalogue the overlay shaders come from.
                     * @param pipelines The factory owning the pipeline.
                     * @param geometry  The per-frame vertex and index buffers to draw.
                     * @param font      The glyph atlas whose heap slot the shader samples.
                     * @param textures  The store resolving that atlas to a heap slot.
                     * @param heap      The bindless heap the atlas is reached through.
                     */
                    UIPass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                           Resources::GraphicsPipelineFactory& pipelines,
                           Geometry::UIBuffers& geometry, const Assets::FontAtlas& font,
                           const Assets::TextureLibrary& textures, Resources::DescriptorHeap& heap);
                    ~UIPass() override;

                    UIPass(const UIPass&) = delete;
                    UIPass& operator=(const UIPass&) = delete;

                    void register_pass(Graph::RenderGraph& graph,
                                       const Frame::FrameContext& frame) override;
                    void rebuild_pipelines() override;

                private:
                    /** @brief Set index of the bindless heap, as everywhere else in this renderer. */
                    static constexpr std::uint32_t HEAP_SET = 1;

                    /** @brief The overlay's 32-byte constants. */
                    struct Push
                    {
                        float screen[4];       /**< xy = viewport size in pixels. */
                        std::uint32_t atlas[4]; /**< x = the glyph atlas's bindless slot. */
                    };

                    void create_pipeline();
                    void destroy_pipeline();

                    Vulkan::VulkanDevice& device_;
                    Resources::ShaderLibrary& shaders_;
                    Resources::GraphicsPipelineFactory& pipelines_;
                    Geometry::UIBuffers& geometry_;
                    const Assets::FontAtlas& font_;
                    const Assets::TextureLibrary& textures_;
                    Resources::DescriptorHeap& heap_;

                    /**
                     * @brief An empty set-0 layout, so the heap can stay at set 1.
                     *
                     * The overlay binds no per-frame descriptors at all — its geometry is a
                     * vertex buffer and its one texture is a heap index in a push constant — so
                     * set 0 has nothing in it. It exists anyway because every other pipeline in
                     * this renderer reaches the heap at set 1, and renumbering it here would
                     * make the overlay the one shader that addresses textures differently.
                     */
                    VkDescriptorSetLayout empty_layout_ = VK_NULL_HANDLE;
                    VkPipelineLayout pipeline_layout_ = VK_NULL_HANDLE;
                    Resources::PipelineHandle pipeline_;
            };
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
