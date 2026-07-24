/**************************************************************************/
/* skinning_system.hpp                                                   */
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
 * @file skinning_system.hpp
 * @brief The per-view, per-frame joint palettes and skinned-vertex output for characters.
 *
 * The animation evaluator hands the renderer object-space joint palettes per skinned
 * instance (current and previous frame) through the `SkinnedInstance` extract channel.
 * This system, modeled on `ClothBuffers`, packs those palettes into host-visible storage
 * buffers and lays out one transient device-local output vertex buffer sliced per
 * instance; the `SkinningPass` compute dispatch fills it, and the opaque pass draws each
 * slice as a static mesh (design §6.2/§6.3). No history is kept here — the evaluator is
 * the source of both palettes, so the "double buffering" is simply the two palettes it
 * provides.
 *
 * The output vertex is 72 bytes: the 60-byte MeshVertex layout plus a 12-byte
 * previous-frame skinned position, so a single vertex binding feeds both the shaded draw
 * and the motion vector's previous position.
 */

#include <cstddef>
#include <cstdint>
#include <vector>

#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/render/material.hpp>
#include <SushiEngine/render/scene_view.hpp>

namespace SushiEngine
{
    namespace Render
    {
        namespace Vulkan
        {
            class VulkanDevice;
        }

        namespace Geometry
        {
            class MeshRegistry;
        }

        namespace Scene
        {
            /** @brief Bytes of one joint palette matrix (16 column-major floats). */
            constexpr VkDeviceSize JOINT_MATRIX_SIZE = 64;

            /** @brief Bytes of one skin vertex (four uint16 joints + four unorm8 weights). */
            constexpr VkDeviceSize SKIN_VERTEX_SIZE = 12;

            /** @brief Bytes of one skinned output vertex (MeshVertex + previous position). */
            constexpr VkDeviceSize SKINNED_VERTEX_SIZE = 72;

            /**
             * @brief One skinned instance's slice of the frame's palette and output buffers.
             *
             * Everything the compute dispatch needs to skin the instance and the opaque pass
             * needs to draw it: the base mesh (its rest vertices, skin stream, and indices),
             * how many vertices and joints, where its palette and output slices begin, its
             * absolute transform, and its material and pick id.
             */
            struct SkinnedRange
            {
                MeshId mesh = INVALID_MESH;      /**< Base mesh: rest verts, skin stream, indices. */
                std::uint32_t vertex_count = 0;  /**< Vertices to skin and draw. */
                std::uint32_t index_count = 0;   /**< Indices to draw with. */
                std::uint32_t base_vertex = 0;   /**< First output vertex of this instance. */
                std::uint32_t palette_base = 0;  /**< First joint slot of this instance. */
                std::uint32_t joint_count = 0;   /**< Joints in this instance's palette. */
                std::uint32_t prev_valid = 0;    /**< 1 if a previous palette was provided. */
                std::uint32_t id = 0;            /**< Picking id. */
                Mat4 model{};                    /**< Absolute object-to-world transform. */
                Material material{};             /**< Surface to shade with. */
            };

            /**
             * @brief Growable palette and skinned-vertex buffers, per frame slot.
             *
             * Non-copyable: it owns VMA allocations.
             */
            class SkinningSystem
            {
                public:
                    /**
                     * @brief Allocates the per-slot buffer sets.
                     * @param device      The live Vulkan device.
                     * @param frame_slots Number of frames in flight.
                     */
                    SkinningSystem(Vulkan::VulkanDevice& device, std::uint32_t frame_slots);
                    ~SkinningSystem();

                    SkinningSystem(const SkinningSystem&) = delete;
                    SkinningSystem& operator=(const SkinningSystem&) = delete;

                    /**
                     * @brief Packs the frame's palettes and lays out the output slices.
                     *
                     * Copies each instance's current and previous palettes into this slot's
                     * host-visible buffers, records a @ref SkinnedRange per instance, and grows
                     * the device-local output vertex buffer to hold every instance's skinned
                     * vertices. Instances whose mesh carries no skin stream are skipped.
                     *
                     * @param slot     The frame slot being recorded.
                     * @param skinned  The frame's skinned instances.
                     * @param count    Number of entries in @p skinned.
                     * @param meshes   The mesh registry (to size each instance by its mesh).
                     */
                    void prepare(std::uint32_t slot, const SkinnedInstance* skinned,
                                 std::size_t count, const Geometry::MeshRegistry& meshes);

                    /** @brief Whether the frame packed any skinned geometry. */
                    bool empty() const noexcept { return ranges_.empty(); }

                    /** @brief The instance slices this frame. */
                    const std::vector<SkinnedRange>& ranges() const noexcept { return ranges_; }

                    /** @brief This slot's current-frame palette buffer. */
                    VkBuffer palette_buffer(std::uint32_t slot) const noexcept;

                    /** @brief This slot's previous-frame palette buffer. */
                    VkBuffer previous_palette_buffer(std::uint32_t slot) const noexcept;

                    /** @brief This slot's device-local skinned-vertex output buffer. */
                    VkBuffer output_buffer(std::uint32_t slot) const noexcept;

                    /** @brief Bytes of the palette buffers this frame (both are the same size). */
                    VkDeviceSize palette_range() const noexcept;

                    /** @brief Bytes of the output buffer this frame. */
                    VkDeviceSize output_range() const noexcept;

                private:
                    /** @brief A VMA-backed buffer and the capacity it was allocated at. */
                    struct Allocation
                    {
                        VkBuffer buffer = VK_NULL_HANDLE;
                        VmaAllocation allocation = VK_NULL_HANDLE;
                        void* mapped = nullptr;
                        VkDeviceSize capacity = 0;
                    };

                    void grow(Allocation& target, VkDeviceSize bytes, VkBufferUsageFlags usage,
                              bool host_visible);
                    void destroy(Allocation& target);

                    Vulkan::VulkanDevice& device_;
                    std::vector<Allocation> palettes_;      /**< Host-visible current palettes, per slot. */
                    std::vector<Allocation> prev_palettes_; /**< Host-visible previous palettes, per slot. */
                    std::vector<Allocation> outputs_;       /**< Device-local skinned vertices, per slot. */
                    std::vector<SkinnedRange> ranges_;
                    std::vector<std::byte> palette_scratch_;
                    std::vector<std::byte> prev_scratch_;
                    std::uint32_t total_joints_ = 0;
                    std::uint32_t total_vertices_ = 0;
            };
        } // namespace Scene
    } // namespace Render
} // namespace SushiEngine
