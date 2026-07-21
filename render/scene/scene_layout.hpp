/**************************************************************************/
/* scene_layout.hpp                                                       */
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
 * @file scene_layout.hpp
 * @brief The descriptor and pipeline layout every scene pass shares.
 *
 * Set 0 is the per-frame set: binding 0 is the scene uniform block, bindings 1–6 are
 * whatever sampled images the pass needs (depth, the previous target, the cloud
 * noise volumes, the image-based lighting chain), binding 7 is the frame's packed
 * material array, binding 8 is the frame's packed previous-frame transforms, and
 * binding 9 is the temporal block relating this frame to the last one. Set 1 is the
 * global bindless heap, which every material map is sampled out of. Sharing one
 * layout across passes keeps a pipeline bind from invalidating the sets a neighbouring
 * pass just wrote, and it is what lets the existing shaders port onto the graph
 * unchanged.
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
            /**
             * @brief Per-draw constants: transform, material index, outline shift, pick ids.
             *
             * 128 bytes, exactly the push-constant floor every Vulkan device
             * guarantees. The surface parameters live in the frame's material array and
             * the previous frame's transform in the frame's motion array; both travel as
             * one index, which is what keeps the push constant fixed-size as the
             * material and temporal models grow. The legacy colour rows remain for the passes
             * that shade flat (grid lines, the selection outline) and never look a
             * material up. Matrices and colours are explicit float arrays: GPU data is
             * 32-bit whatever the engine's Scalar precision, so a double build narrows
             * to float exactly here, at the upload boundary.
             */
            struct MeshPushConstants
            {
                float model[16];
                float albedo_metallic[4];    /**< xyz = flat albedo, w = metallic. */
                float emissive_roughness[4]; /**< xyz = flat emissive, w = roughness. */
                float outline_shift[4];      /**< xy = screen-space shift, zw spare. */
                std::uint32_t entity_id;
                std::uint32_t selected;
                std::uint32_t material_index; /**< Index into the frame's material array. */
                std::uint32_t motion_index;   /**< Index into the frame's motion array. */
            };

            /**
             * @brief Owns the shared set layout and pipeline layout, and binds them.
             *
             * Non-copyable: it owns Vulkan layout objects that every pass's pipelines
             * are built against.
             */
            class SceneLayout
            {
                public:
                    /** @brief Binding of the per-frame scene uniform block. */
                    static constexpr std::uint32_t SCENE_BINDING = 0;

                    /** @brief First binding of the pass-local sampled image range. */
                    static constexpr std::uint32_t FIRST_IMAGE_BINDING = 1;

                    /** @brief Number of sampled image bindings in the per-frame set. */
                    static constexpr std::uint32_t IMAGE_BINDING_COUNT = 6;

                    /** @brief Binding of the frame's packed material array. */
                    static constexpr std::uint32_t MATERIAL_BINDING = 7;

                    /**
                     * @brief Binding of the frame's packed previous-frame transforms.
                     *
                     * Visible to the vertex stage, unlike the material array: a motion
                     * vector is a difference between two clip positions, and the second
                     * of them can only be computed where the first one is.
                     */
                    static constexpr std::uint32_t MOTION_BINDING = 8;

                    /** @brief Binding of the block relating this frame to the previous one. */
                    static constexpr std::uint32_t TEMPORAL_BINDING = 9;

                    /** @brief Binding of the block placing the sun's shadow cascades. */
                    static constexpr std::uint32_t SHADOW_BINDING = 10;

                    /**
                     * @brief Binding of the sun's shadow cascade atlas.
                     *
                     * Its own binding rather than one of the pass-local image slots,
                     * because it is frame-global: the geometry pass and the sky pass both
                     * sample the same atlas, and the sky pass has all six of its local
                     * slots spoken for by the depth, the scene colour, and the four cloud
                     * volumes. Read through a comparison sampler.
                     */
                    static constexpr std::uint32_t SHADOW_ATLAS_BINDING = 11;

                    /**
                     * @brief Binding of the same atlas through a plain sampler.
                     *
                     * The soft-shadow filter has to know how far above a receiver its
                     * blockers stand, and a comparison sampler can only report whether a
                     * tap passed, never what it stored. Binding the one image twice is
                     * far cheaper than recovering a depth from a sweep of comparisons.
                     */
                    static constexpr std::uint32_t SHADOW_DEPTH_BINDING = 12;

                    /**
                     * @brief Binding of the environment's 9 diffuse SH coefficients.
                     *
                     * A frame-global storage buffer written by the IBL build and read by the
                     * shading pass in place of a diffuse irradiance cubemap sample: nine
                     * uniform reads and a polynomial evaluate replace a filtered cube fetch,
                     * and probe blending later becomes a blend of coefficients.
                     */
                    static constexpr std::uint32_t IBL_SH_BINDING = 13;

                    /** @brief Number of bindings in the per-frame set. */
                    static constexpr std::uint32_t BINDING_COUNT = IBL_SH_BINDING + 1;

                    /**
                     * @brief Creates the set and pipeline layouts.
                     * @param device The live Vulkan device.
                     * @param heap   The bindless heap; included as set 1 when available.
                     */
                    SceneLayout(Vulkan::VulkanDevice& device, Resources::DescriptorHeap& heap);
                    ~SceneLayout();

                    SceneLayout(const SceneLayout&) = delete;
                    SceneLayout& operator=(const SceneLayout&) = delete;

                    /** @brief The per-frame set layout every pass allocates against. */
                    VkDescriptorSetLayout set_layout() const noexcept { return set_layout_; }

                    /** @brief The pipeline layout every scene pipeline is built with. */
                    VkPipelineLayout pipeline_layout() const noexcept { return pipeline_layout_; }

                    /**
                     * @brief Binds the per-frame set, and the bindless heap when present.
                     * @param cmd       The recording command buffer.
                     * @param frame_set The set written for this pass this frame.
                     */
                    void bind(VkCommandBuffer cmd, VkDescriptorSet frame_set) const;

                private:
                    Vulkan::VulkanDevice& device_;
                    Resources::DescriptorHeap& heap_;
                    VkDescriptorSetLayout set_layout_ = VK_NULL_HANDLE;
                    VkPipelineLayout pipeline_layout_ = VK_NULL_HANDLE;
            };

            /**
             * @brief Accumulates descriptor writes for one set, then commits them at once.
             *
             * Holds the VkDescriptorBufferInfo/VkDescriptorImageInfo structs the writes
             * point at, which is the part callers most often get wrong when they build
             * the arrays on the stack and let them die before vkUpdateDescriptorSets.
             */
            class SceneSetWriter
            {
                public:
                    /**
                     * @brief Starts a write batch against one set.
                     * @param set The destination descriptor set.
                     */
                    explicit SceneSetWriter(VkDescriptorSet set) noexcept : set_(set) {}

                    /**
                     * @brief Queues a uniform buffer write.
                     * @param binding Binding number in the set.
                     * @param buffer  The buffer to bind.
                     * @param range   Bytes visible from offset zero.
                     */
                    void uniform(std::uint32_t binding, VkBuffer buffer, VkDeviceSize range);

                    /**
                     * @brief Queues a combined image sampler write.
                     * @param binding Binding number in the set.
                     * @param view    The image view to sample.
                     * @param sampler The sampler to pair with it.
                     */
                    void image(std::uint32_t binding, VkImageView view, VkSampler sampler);

                    /**
                     * @brief Queues a storage buffer write.
                     * @param binding Binding number in the set.
                     * @param buffer  The buffer to bind.
                     * @param range   Bytes visible from offset zero.
                     */
                    void storage(std::uint32_t binding, VkBuffer buffer, VkDeviceSize range);

                    /**
                     * @brief Applies every queued write.
                     * @param device The device owning the set.
                     */
                    void commit(VkDevice device);

                private:
                    static constexpr std::uint32_t CAPACITY = 16;

                    VkDescriptorSet set_ = VK_NULL_HANDLE;
                    VkWriteDescriptorSet writes_[CAPACITY]{};
                    VkDescriptorBufferInfo buffers_[CAPACITY]{};
                    VkDescriptorImageInfo images_[CAPACITY]{};
                    std::uint32_t count_ = 0;
            };
        } // namespace Scene
    } // namespace Render
} // namespace SushiEngine
