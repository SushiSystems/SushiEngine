/**************************************************************************/
/* terrain_layout.hpp                                                     */
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
 * @file terrain_layout.hpp
 * @brief Terrain's own descriptor set, and the pipeline layout it rides in.
 *
 * The per-frame scene set is full: `SceneLayout` declares 32 bindings and its own comment
 * records that as the guaranteed push-descriptor floor, so terrain cannot be the thing
 * that overflows it. It takes set 2 instead — the arrangement the GPU-driven and meshlet
 * paths already use — and leaves sets 0 and 1 byte-for-byte what every other scene
 * pipeline binds. That is what lets terrain be shaded by the existing `pbr.frag` with no
 * parallel lighting code (`docs/slop/solar_system_overhaul.md` §8.3).
 *
 * The push constant is vertex-only, because `pbr.frag` declares none: it takes the
 * material index and the picking id as flat vertex outputs, which is exactly what let
 * terrain reuse it.
 */

#include <cstdint>

#include <vulkan/vulkan.h>

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
            class DescriptorHeap;
        }

        namespace Scene
        {
            class SceneLayout;
        }

        namespace Terrain
        {
            /**
             * @brief What a terrain draw hands its vertex shader.
             *
             * Sixteen bytes, vertex stage only. The node record carries everything else,
             * because a draw covers every visible node of a body at once and a push
             * constant cannot vary within one.
             */
            struct TerrainPushConstants
            {
                std::uint32_t material_index = 0;
                std::uint32_t entity_id = 0;
                std::uint32_t spare0 = 0;
                std::uint32_t spare1 = 0;
            };

            /**
             * @brief Owns terrain's set-2 layout and the pipeline layout built around it.
             *
             * Non-copyable: it owns Vulkan layout objects the terrain pipelines are built
             * against.
             */
            class TerrainLayout
            {
                public:
                    /** @brief Set index terrain's own resources are bound at. */
                    static constexpr std::uint32_t TERRAIN_SET = 2;

                    /** @brief Binding of the per-frame selected-node array. */
                    static constexpr std::uint32_t NODE_BINDING = 0;

                    /** @brief Binding of the height slot pool, sampled by the vertex stage. */
                    static constexpr std::uint32_t HEIGHT_BINDING = 1;

                    /** @brief Binding of the body block: the reference ellipsoid's semi-axes. */
                    static constexpr std::uint32_t BODY_BINDING = 2;

                    /**
                     * @brief Creates the set and pipeline layouts.
                     * @param device The live Vulkan device.
                     * @param scene  The shared scene layout, whose set 0 is reused verbatim.
                     * @param heap   The bindless heap that occupies set 1.
                     */
                    TerrainLayout(Vulkan::VulkanDevice& device, Scene::SceneLayout& scene,
                                  Resources::DescriptorHeap& heap);
                    ~TerrainLayout();

                    TerrainLayout(const TerrainLayout&) = delete;
                    TerrainLayout& operator=(const TerrainLayout&) = delete;

                    /**
                     * @brief Whether terrain can be drawn at all on this device.
                     *
                     * False without the bindless heap. Terrain shades through `pbr.frag`,
                     * which samples material maps out of set 1, and without that set the
                     * set indices shift underneath every shader — so terrain declines
                     * rather than binding a layout its shaders were not compiled against.
                     */
                    bool available() const noexcept { return available_; }

                    /** @brief The set-2 layout a frame writes its node buffer and slots into. */
                    VkDescriptorSetLayout set_layout() const noexcept { return set_layout_; }

                    /** @brief The pipeline layout a terrain draw is built with. */
                    VkPipelineLayout pipeline_layout() const noexcept
                    {
                        return pipeline_layout_;
                    }

                    /**
                     * @brief Binds the bindless heap at set 1 of *this* pipeline layout.
                     *
                     * Not `SceneLayout::bind_heap`, even though sets 0 and 1 are the same
                     * layouts: Vulkan's set-compatibility rule also requires identical push
                     * constant ranges, and terrain's range is vertex-only and 16 bytes where
                     * the scene's is 128 across two stages. Binding through the scene layout
                     * would therefore disturb terrain's sets rather than share them.
                     *
                     * @param command The recording command buffer.
                     */
                    void bind_heap(VkCommandBuffer command) const;

                private:
                    Vulkan::VulkanDevice& device_;
                    Resources::DescriptorHeap& heap_;
                    VkDescriptorSetLayout set_layout_ = VK_NULL_HANDLE;
                    VkPipelineLayout pipeline_layout_ = VK_NULL_HANDLE;
                    bool available_ = false;
            };
        } // namespace Terrain
    } // namespace Render
} // namespace SushiEngine
