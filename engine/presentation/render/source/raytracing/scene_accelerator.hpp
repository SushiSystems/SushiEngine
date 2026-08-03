/**************************************************************************/
/* scene_accelerator.hpp                                                  */
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
 * @file scene_accelerator.hpp
 * @brief The scene as a structure a shader can trace a ray against.
 *
 * Two levels, which is what the hardware wants: a bottom-level structure per distinct
 * mesh, built once and kept, and one top-level structure per frame holding an instance
 * per drawn object. Splitting it that way is why a thousand copies of one mesh cost a
 * thousand 64-byte instance records rather than a thousand rebuilds.
 *
 * Everything here is camera-relative, exactly like the rasterised path: an instance's
 * transform has the eye subtracted in double before the float cast. A ray traced against
 * absolute planetary coordinates in single precision would miss by metres.
 *
 * The whole object is inert on a device without ray queries — available() answers false
 * and top_level() hands back nothing, so the passes that would use it never register.
 */

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>

#include <SushiEngine/render/scene_view.hpp>

namespace SushiEngine
{
    namespace Render
    {
        namespace Geometry
        {
            class MeshRegistry;
            struct Mesh;
        }

        namespace Vulkan
        {
            class VulkanDevice;
        }

        namespace RayTracing
        {
            /**
             * @brief Builds and owns the scene's acceleration structures.
             *
             * Device-level for the bottom halves (two viewports tracing the same mesh
             * share one structure) but per frame slot for the top half, which is rebuilt
             * every frame from host-written instance records. Non-copyable: it owns
             * Vulkan and VMA handles.
             */
            class SceneAccelerator
            {
                public:
                    /**
                     * @brief Prepares the structure store; builds nothing yet.
                     * @param device      The live Vulkan device.
                     * @param meshes      Registry the geometry is read out of.
                     * @param frame_slots How many frames may be in flight.
                     */
                    SceneAccelerator(Vulkan::VulkanDevice& device, Geometry::MeshRegistry& meshes,
                                     std::uint32_t frame_slots);
                    ~SceneAccelerator();

                    SceneAccelerator(const SceneAccelerator&) = delete;
                    SceneAccelerator& operator=(const SceneAccelerator&) = delete;

                    /** @brief Whether the device can build or trace anything at all. */
                    bool available() const noexcept { return available_; }

                    /**
                     * @brief Records this frame's structure builds into a command buffer.
                     *
                     * Every structure the frame needs is created and sized before a single
                     * build is recorded, because they share one scratch buffer: growing it
                     * between builds whose scratch address has already been baked in would
                     * be a use-after-free the GPU only discovers much later.
                     *
                     * @param cmd       The command buffer to record into.
                     * @param slot      Which frame slot is being recorded.
                     * @param instances The objects drawn this frame.
                     * @param count     Number of instances.
                     * @param eye       Camera world position, subtracted in double.
                     */
                    void build(VkCommandBuffer cmd, std::uint32_t slot,
                               const MeshInstance* instances, std::size_t count,
                               const double eye[3]);

                    /**
                     * @brief The structure a shader traces against for a frame slot.
                     * @param slot The frame slot last recorded through build().
                     * @return The top-level structure, or VK_NULL_HANDLE if it holds nothing.
                     */
                    VkAccelerationStructureKHR top_level(std::uint32_t slot) const noexcept;

                private:
                    /** @brief A buffer allocated for structure storage, instances, or scratch. */
                    struct Buffer
                    {
                        VkBuffer buffer = VK_NULL_HANDLE;
                        VmaAllocation allocation = VK_NULL_HANDLE;
                        void* mapped = nullptr;
                        VkDeviceSize capacity = 0;
                    };

                    /** @brief One mesh's bottom-level structure, built once and kept. */
                    struct BottomLevel
                    {
                        VkAccelerationStructureKHR structure = VK_NULL_HANDLE;
                        Buffer storage;
                        VkDeviceAddress address = 0;
                    };

                    /** @brief One frame slot's top-level structure and its instance records. */
                    struct TopLevel
                    {
                        VkAccelerationStructureKHR structure = VK_NULL_HANDLE;
                        Buffer storage;
                        Buffer instances;
                        std::uint32_t instance_count = 0;
                    };

                    /**
                     * @brief A bottom-level structure created this frame, awaiting its build.
                     *
                     * It carries its own slice of the shared scratch buffer, which is what
                     * lets several builds be recorded back to back without racing each
                     * other over one region.
                     */
                    struct Pending
                    {
                        VkBuffer key = VK_NULL_HANDLE;
                        VkAccelerationStructureKHR structure = VK_NULL_HANDLE;
                        VkAccelerationStructureGeometryKHR geometry{};
                        std::uint32_t triangle_count = 0;
                        VkDeviceSize scratch_offset = 0;
                    };

                    void create_buffer(Buffer& target, VkDeviceSize bytes,
                                       VkBufferUsageFlags usage, bool host_visible);
                    void destroy_buffer(Buffer& target);
                    VkDeviceAddress address_of(VkBuffer buffer) const;
                    void describe_geometry(const Geometry::Mesh& mesh,
                                           VkAccelerationStructureGeometryKHR& geometry) const;
                    void stage_bottom_level(const Geometry::Mesh& mesh);
                    void record_pending(VkCommandBuffer cmd);
                    void build_top_level(VkCommandBuffer cmd, TopLevel& top,
                                         VkDeviceSize scratch_offset);

                    Vulkan::VulkanDevice& device_;
                    Geometry::MeshRegistry& meshes_;
                    std::unordered_map<VkBuffer, BottomLevel> bottom_;
                    std::vector<TopLevel> top_;
                    std::vector<Pending> pending_;
                    Buffer scratch_;
                    std::vector<VkAccelerationStructureInstanceKHR> records_;
                    VkDeviceSize scratch_needed_ = 0;
                    bool available_ = false;
            };
        } // namespace RayTracing
    } // namespace Render
} // namespace SushiEngine
