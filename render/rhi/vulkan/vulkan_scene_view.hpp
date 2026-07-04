/**************************************************************************/
/* vulkan_scene_view.hpp                                                  */
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
 * @file vulkan_scene_view.hpp
 * @brief Vulkan implementation of ISceneView: an offscreen lit-mesh + grid pass.
 *
 * Internal to the render library. Draws a ground grid and a set of mesh instances
 * into a double-buffered offscreen colour target (with a depth buffer) using Vulkan
 * 1.3 dynamic rendering, then leaves the colour image in a shader-readable layout so
 * the editor can sample it with ImGui. Geometry (a unit cube and the grid) lives in
 * host-visible VMA buffers built once; the two draw pipelines share one layout and
 * vertex shader.
 */

#include <cstdint>

#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>

#include <SushiEngine/render/cloth_mesh.hpp>
#include <SushiEngine/render/scene_view.hpp>

#include "vulkan_device.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Vulkan
        {
            /**
             * @brief An offscreen mesh/grid renderer sampled into the editor viewport.
             *
             * Owns two target slots so the frame being sampled by the UI is never the
             * frame being drawn. Non-copyable: it owns Vulkan and VMA handles.
             */
            class VulkanSceneView final : public ISceneView
            {
                public:
                    /**
                     * @brief Builds the pipelines, geometry, and initial targets.
                     * @param device The live Vulkan device to render on.
                     */
                    explicit VulkanSceneView(VulkanDevice& device);
                    ~VulkanSceneView() override;

                    VulkanSceneView(const VulkanSceneView&) = delete;
                    VulkanSceneView& operator=(const VulkanSceneView&) = delete;

                    void resize(std::uint32_t width, std::uint32_t height) override;
                    std::uint32_t width() const noexcept override { return width_; }
                    std::uint32_t height() const noexcept override { return height_; }
                    void render(const CameraView& camera, const Environment& environment,
                                const MeshInstance* instances,
                                std::size_t count, std::uint32_t selected_id,
                                const ClothStrandView* strands = nullptr,
                                std::size_t strand_count = 0) override;
                    std::uint32_t pick(std::uint32_t x, std::uint32_t y) override;
                    std::uint32_t slot_count() const noexcept override { return SLOTS; }
                    SceneViewTexture texture(std::uint32_t slot) const noexcept override;
                    std::uint32_t current_slot() const noexcept override { return current_slot_; }

                private:
                    static constexpr std::uint32_t SLOTS = 2;
                    // The scene renders to a linear HDR target (atmospheric scattering and
                    // the sun produce values well above 1), then a tonemap pass resolves it
                    // to the LDR image the editor samples with ImGui.
                    static constexpr VkFormat HDR_FORMAT = VK_FORMAT_R16G16B16A16_SFLOAT;
                    static constexpr VkFormat RESOLVE_FORMAT = VK_FORMAT_R8G8B8A8_UNORM;
                    static constexpr VkFormat DEPTH_FORMAT = VK_FORMAT_D24_UNORM_S8_UINT;
                    static constexpr VkFormat ID_FORMAT = VK_FORMAT_R32_UINT;

                    /** @brief A VMA-backed buffer with its element count. */
                    struct Buffer
                    {
                        VkBuffer buffer = VK_NULL_HANDLE;
                        VmaAllocation allocation = VK_NULL_HANDLE;
                        std::uint32_t count = 0;
                    };

                    /** @brief One double-buffered offscreen target and its recording state. */
                    struct Slot
                    {
                        // Opaque pass writes hdr + id + depth; the sky pass samples hdr and
                        // depth and writes composite; the tonemap pass samples composite and
                        // writes resolve, which is the image the editor samples.
                        VkImage hdr = VK_NULL_HANDLE;
                        VmaAllocation hdr_allocation = VK_NULL_HANDLE;
                        VkImageView hdr_view = VK_NULL_HANDLE;
                        VkImage composite = VK_NULL_HANDLE;
                        VmaAllocation composite_allocation = VK_NULL_HANDLE;
                        VkImageView composite_view = VK_NULL_HANDLE;
                        VkImage resolve = VK_NULL_HANDLE;
                        VmaAllocation resolve_allocation = VK_NULL_HANDLE;
                        VkImageView resolve_view = VK_NULL_HANDLE;
                        VkImage depth = VK_NULL_HANDLE;
                        VmaAllocation depth_allocation = VK_NULL_HANDLE;
                        VkImageView depth_view = VK_NULL_HANDLE;        /**< Depth+stencil, for the attachment. */
                        VkImageView depth_sample_view = VK_NULL_HANDLE; /**< Depth aspect only, for sampling. */
                        VkImage id = VK_NULL_HANDLE;
                        VmaAllocation id_allocation = VK_NULL_HANDLE;
                        VkImageView id_view = VK_NULL_HANDLE;
                        VkBuffer readback = VK_NULL_HANDLE;
                        VmaAllocation readback_allocation = VK_NULL_HANDLE;
                        void* readback_mapped = nullptr;
                        VkBuffer ubo = VK_NULL_HANDLE;
                        VmaAllocation ubo_allocation = VK_NULL_HANDLE;
                        void* ubo_mapped = nullptr;
                        VkDescriptorSet mesh_set = VK_NULL_HANDLE;    /**< Opaque pass: UBO only. */
                        VkDescriptorSet sky_set = VK_NULL_HANDLE;     /**< Sky pass: UBO + depth + hdr. */
                        VkDescriptorSet tonemap_set = VK_NULL_HANDLE; /**< Tonemap pass: UBO + composite. */
                        VkCommandPool pool = VK_NULL_HANDLE;
                        VkCommandBuffer cmd = VK_NULL_HANDLE;
                        VkFence fence = VK_NULL_HANDLE;
                        bool ever_rendered = false;
                    };

                    void create_descriptors();
                    void destroy_descriptors();
                    void create_pipelines();
                    void destroy_pipelines();
                    void create_geometry();
                    void destroy_geometry();
                    void create_sampler();
                    void create_targets();
                    void destroy_targets();
                    void ensure_cloth_capacity(VkDeviceSize vertex_bytes, VkDeviceSize index_bytes);

                    VulkanDevice& device_;
                    VkDescriptorSetLayout set_layout_ = VK_NULL_HANDLE;
                    VkDescriptorPool descriptor_pool_ = VK_NULL_HANDLE;
                    VkPipelineLayout layout_ = VK_NULL_HANDLE;
                    VkPipeline mesh_pipeline_ = VK_NULL_HANDLE;
                    VkPipeline line_pipeline_ = VK_NULL_HANDLE;
                    VkPipeline outline_pipeline_ = VK_NULL_HANDLE;
                    VkPipeline sky_pipeline_ = VK_NULL_HANDLE;
                    VkPipeline tonemap_pipeline_ = VK_NULL_HANDLE;
                    VkSampler sampler_ = VK_NULL_HANDLE;
                    Buffer cube_vertices_;
                    Buffer cube_indices_;
                    Buffer sphere_vertices_;
                    Buffer sphere_indices_;
                    Buffer cylinder_vertices_;
                    Buffer cylinder_indices_;
                    Buffer grid_vertices_;
                    // Host-visible and re-uploaded every frame a cloth grid is drawn, so
                    // each buffer's declared capacity can grow but never shrinks between
                    // frames (see ensure_cloth_capacity in the .cpp).
                    Buffer cloth_vertices_;
                    VkDeviceSize cloth_vertices_capacity_ = 0;
                    void* cloth_vertices_mapped_ = nullptr;
                    Buffer cloth_indices_;
                    VkDeviceSize cloth_indices_capacity_ = 0;
                    void* cloth_indices_mapped_ = nullptr;
                    Slot slots_[SLOTS];
                    std::uint32_t width_ = 16;
                    std::uint32_t height_ = 16;
                    std::uint32_t current_slot_ = 0;
                    std::uint32_t frame_counter_ = 0;
            };
        } // namespace Vulkan
    } // namespace Render
} // namespace SushiEngine
