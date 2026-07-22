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
 * binding 9 is the temporal block relating this frame to the last one. It is a
 * push-descriptor set: each pass pushes the handful of bindings it uses inline in the
 * command buffer instead of allocating and writing a throw-away set every frame. Set 1
 * is the global bindless heap, which every material map is sampled out of, and is bound
 * the ordinary way. Sharing one layout across passes is what lets the existing shaders
 * port onto the graph unchanged.
 */

#include <cstdint>

#include <vulkan/vulkan.h>

#include "resources/descriptor_writer.hpp"

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

                    /**
                     * @brief Binding of the frame's packed punctual-light array.
                     *
                     * The clustered light engine's three storage buffers and one config
                     * block ride the shared scene set so the shading pass reads them the
                     * same way it reads the material array — the froxel grid the light
                     * count and index list address is built by the cull pass each frame.
                     */
                    static constexpr std::uint32_t LIGHT_BINDING = 14;

                    /** @brief Binding of the per-cluster punctual-light count grid. */
                    static constexpr std::uint32_t CLUSTER_GRID_BINDING = 15;

                    /** @brief Binding of the per-cluster punctual-light index list. */
                    static constexpr std::uint32_t LIGHT_INDEX_BINDING = 16;

                    /** @brief Binding of the cluster-grid config block (dims, depth, tiles). */
                    static constexpr std::uint32_t CLUSTER_CONFIG_BINDING = 17;

                    /**
                     * @brief Binding of the shared punctual-light shadow atlas.
                     *
                     * One depth image, a 4×4 grid of tiles, read through a comparison
                     * sampler like the sun's cascade atlas; a shadow-casting spot renders
                     * into its claimed tile and the shading pass compares against it.
                     */
                    static constexpr std::uint32_t LIGHT_SHADOW_ATLAS_BINDING = 18;

                    /**
                     * @brief Binding of the per-caster shadow matrices + tile rects.
                     *
                     * Visible to the vertex stage too: the punctual shadow pass's vertex
                     * shader reads a caster's light matrix here to project geometry into its
                     * tile, and the fragment stage reads the same record to sample it back.
                     */
                    static constexpr std::uint32_t LIGHT_SHADOW_DATA_BINDING = 19;

                    /** @brief Binding of the frame's packed projected-decal array. */
                    static constexpr std::uint32_t DECAL_BINDING = 20;

                    /** @brief Binding of the per-cluster decal count grid. */
                    static constexpr std::uint32_t DECAL_GRID_BINDING = 21;

                    /** @brief Binding of the per-cluster decal index list. */
                    static constexpr std::uint32_t DECAL_INDEX_BINDING = 22;

                    /**
                     * @brief Binding of the frame's resolved ambient-occlusion target.
                     *
                     * Full-resolution GTAO: rgb is the world-space bent normal, a the
                     * visibility. The shading pass multiplies its ambient/IBL diffuse by the
                     * visibility and occludes indirect specular against the bent-normal cone;
                     * when AO is off the target is a cleared unoccluded image, so the binding
                     * is read unconditionally like the shadow atlases.
                     */
                    static constexpr std::uint32_t AO_BINDING = 23;

                    /**
                     * @brief Binding of the atmosphere transmittance LUT.
                     *
                     * The view-independent optical depth from any altitude and sun angle
                     * to the top of the atmosphere (Hillaire 2020). The sky march reads the
                     * sun's transmittance from it instead of integrating a per-sample light
                     * ray; the IBL sky capture reads it for the same reason.
                     */
                    static constexpr std::uint32_t TRANSMITTANCE_LUT_BINDING = 24;

                    /**
                     * @brief Binding of the atmosphere multiple-scattering LUT.
                     *
                     * The infinite-order isotropic scattering the single march omits, so
                     * the horizon and shadowed ground read hazy rather than black at dusk.
                     */
                    static constexpr std::uint32_t MULTISCATTER_LUT_BINDING = 25;

                    /**
                     * @brief Binding of the per-frame sky-view LUT.
                     *
                     * A latitude-longitude image of the background sky's in-scattered
                     * radiance in the camera's local frame, so a pixel with no geometry is a
                     * single fetch instead of a full atmosphere march.
                     */
                    static constexpr std::uint32_t SKY_VIEW_LUT_BINDING = 26;

                    /**
                     * @brief Binding of the aerial-perspective froxel volume.
                     *
                     * A camera-frustum-aligned 3D texture of the in-scatter and
                     * transmittance from the camera out to each froxel's depth, so a mesh
                     * reads the air in front of it as one fetch instead of a march.
                     */
                    static constexpr std::uint32_t AERIAL_LUT_BINDING = 27;

                    /**
                     * @brief Binding of the volumetric-fog froxel volume.
                     *
                     * The ground-hugging fog's in-scatter and transmittance, folded over
                     * every pixel in the sky composite. Shares the aerial volume's addressing.
                     */
                    static constexpr std::uint32_t FOG_LUT_BINDING = 28;

                    /** @brief Number of bindings in the per-frame set. */
                    static constexpr std::uint32_t BINDING_COUNT = FOG_LUT_BINDING + 1;

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
                     * @brief Binds the bindless heap at set 1, when present.
                     *
                     * The per-frame set 0 is a push-descriptor set, so it is pushed by
                     * SceneSetWriter::commit() rather than bound here; this call only
                     * plants the heap that set 0's push cannot supply.
                     *
                     * @param cmd The recording command buffer.
                     */
                    void bind_heap(VkCommandBuffer cmd) const;

                private:
                    Vulkan::VulkanDevice& device_;
                    Resources::DescriptorHeap& heap_;
                    VkDescriptorSetLayout set_layout_ = VK_NULL_HANDLE;
                    VkPipelineLayout pipeline_layout_ = VK_NULL_HANDLE;
            };

            /**
             * @brief The scene set's writer: the shared DescriptorWriter, in scene terms.
             *
             * A thin adapter over Resources::DescriptorWriter so the scene passes name the
             * bindings they know (uniform, image, storage) while every write still runs
             * through the one seam VK_EXT_descriptor_heap would swap. commit() pushes set 0
             * straight into the command buffer, so no set is allocated or written up front.
             */
            class SceneSetWriter
            {
                public:
                    /** @brief Starts an empty write batch for the per-frame set. */
                    SceneSetWriter() noexcept = default;

                    /**
                     * @brief Queues a uniform buffer write.
                     * @param binding Binding number in the set.
                     * @param buffer  The buffer to bind.
                     * @param range   Bytes visible from offset zero.
                     */
                    void uniform(std::uint32_t binding, VkBuffer buffer, VkDeviceSize range)
                    {
                        writer_.uniform_buffer(binding, buffer, range);
                    }

                    /**
                     * @brief Queues a combined image sampler write.
                     * @param binding Binding number in the set.
                     * @param view    The image view to sample.
                     * @param sampler The sampler to pair with it.
                     */
                    void image(std::uint32_t binding, VkImageView view, VkSampler sampler)
                    {
                        writer_.sampled_image(binding, view, sampler);
                    }

                    /**
                     * @brief Queues a storage buffer write.
                     * @param binding Binding number in the set.
                     * @param buffer  The buffer to bind.
                     * @param range   Bytes visible from offset zero.
                     */
                    void storage(std::uint32_t binding, VkBuffer buffer, VkDeviceSize range)
                    {
                        writer_.storage_buffer(binding, buffer, range);
                    }

                    /**
                     * @brief Pushes every queued write into the command buffer as set 0.
                     * @param cmd    The recording command buffer.
                     * @param layout The pipeline layout whose set 0 is being pushed.
                     */
                    void commit(VkCommandBuffer cmd, VkPipelineLayout layout)
                    {
                        writer_.push(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, layout, 0);
                    }

                private:
                    Resources::DescriptorWriter writer_;
            };
        } // namespace Scene
    } // namespace Render
} // namespace SushiEngine
