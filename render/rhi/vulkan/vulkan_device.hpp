/**************************************************************************/
/* vulkan_device.hpp                                                     */
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
 * @file vulkan_device.hpp
 * @brief The Vulkan implementation of IRenderDevice.
 *
 * Internal to the render library — consumers use the abstract device.hpp. This
 * header pulls in the Vulkan, VMA, and vk-bootstrap types, so only backend
 * translation units include it. It brings up a Vulkan 1.4 instance and device
 * (dynamic rendering + synchronization2 from 1.3; maintenance5/6 + push descriptors
 * from 1.4) and a VMA allocator, and exposes the raw handles later render stages
 * (swapchain, command lists) are built from.
 */

#include <cstdint>

#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>
#include <VkBootstrap.h>

#include <SushiEngine/render/rhi/device.hpp>

namespace SushiEngine
{
    namespace Render
    {
        namespace Vulkan
        {
            /**
             * @brief The acceleration-structure entry points, resolved once per device.
             *
             * VK_KHR_acceleration_structure is an extension, so none of these are loader
             * symbols; every one of them has to come back from vkGetDeviceProcAddr. They
             * are gathered here so no caller repeats that, and so a device without the
             * extension simply leaves them null.
             */
            struct RayTracingFunctions
            {
                PFN_vkCreateAccelerationStructureKHR create_structure = nullptr;
                PFN_vkDestroyAccelerationStructureKHR destroy_structure = nullptr;
                PFN_vkGetAccelerationStructureBuildSizesKHR build_sizes = nullptr;
                PFN_vkCmdBuildAccelerationStructuresKHR build_structures = nullptr;
                PFN_vkGetAccelerationStructureDeviceAddressKHR structure_address = nullptr;

                /** @brief Whether every entry point resolved. */
                bool available() const noexcept
                {
                    return create_structure != nullptr && destroy_structure != nullptr &&
                           build_sizes != nullptr && build_structures != nullptr &&
                           structure_address != nullptr;
                }
            };

            /**
             * @brief The mesh-shader draw entry points, or null members.
             *
             * `VK_EXT_mesh_shader` is not a loader-exposed symbol, so the draw commands are
             * resolved once here. Null when the device does not offer the extension, in which
             * case the renderer takes the classic vertex-fetch path instead — meshlets are a
             * tier addition, never a requirement.
             */
            struct MeshShaderFunctions
            {
                PFN_vkCmdDrawMeshTasksEXT draw_mesh_tasks = nullptr;
                PFN_vkCmdDrawMeshTasksIndirectEXT draw_mesh_tasks_indirect = nullptr;

                /** @brief Whether the mesh-task draw resolved. */
                bool available() const noexcept { return draw_mesh_tasks != nullptr; }
            };

            /**
             * @brief A Vulkan 1.4 device with a VMA allocator and a graphics queue.
             *
             * Selects a physical device (by interop UUID when requested, else by
             * preference), creates the logical device with dynamic-rendering and
             * synchronization2 (1.3) plus maintenance5/6 and push descriptors (1.4)
             * enabled, and stands up a VMA allocator. Destruction
             * releases the allocator, device, and instance in order. Non-copyable:
             * it owns Vulkan handles.
             */
            class VulkanDevice final : public IRenderDevice
            {
                public:
                    /**
                     * @brief Brings up the Vulkan instance, device, and allocator.
                     * @param desc Selection request forwarded from create_render_device().
                     */
                    explicit VulkanDevice(const RenderDeviceDesc& desc);
                    ~VulkanDevice() override;

                    VulkanDevice(const VulkanDevice&) = delete;
                    VulkanDevice& operator=(const VulkanDevice&) = delete;

                    const DeviceInfo& info() const noexcept override { return info_; }
                    NativeDeviceHandles native_handles() const noexcept override;

                    /** @brief The Vulkan instance handle. */
                    VkInstance instance() const noexcept { return instance_.instance; }

                    /**
                     * @brief The presentation surface, or VK_NULL_HANDLE if headless.
                     * @return The VkSurfaceKHR created by the host's surface factory.
                     */
                    VkSurfaceKHR surface() const noexcept { return surface_; }

                    /** @brief The instance's API version, for ImGui/loader setup. */
                    std::uint32_t api_version() const noexcept { return instance_.api_version; }

                    /** @brief The selected physical device handle. */
                    VkPhysicalDevice physical_device() const noexcept { return device_.physical_device; }

                    /** @brief The logical device handle. */
                    VkDevice device() const noexcept { return device_.device; }

                    /** @brief The VMA allocator bound to this device. */
                    VmaAllocator allocator() const noexcept { return allocator_; }

                    /** @brief The graphics-capable queue. */
                    VkQueue graphics_queue() const noexcept { return graphics_queue_; }

                    /** @brief The graphics queue family index. */
                    std::uint32_t graphics_queue_family() const noexcept { return graphics_queue_family_; }

                    /**
                     * @brief Whether a compute queue outside the graphics family exists.
                     *
                     * Only a queue from a *different* family can genuinely overlap graphics
                     * work; a second queue from the graphics family would be scheduled on the
                     * same hardware and buy nothing but submission overhead. False leaves the
                     * render graph putting every pass on the graphics queue, which is always
                     * correct — the async path is an optimisation, never a requirement.
                     */
                    bool supports_async_compute() const noexcept
                    {
                        return async_compute_queue_ != VK_NULL_HANDLE;
                    }

                    /** @brief The dedicated compute queue, or VK_NULL_HANDLE. */
                    VkQueue async_compute_queue() const noexcept { return async_compute_queue_; }

                    /** @brief The dedicated compute queue's family, or the graphics family. */
                    std::uint32_t async_compute_queue_family() const noexcept
                    {
                        return async_compute_queue_family_;
                    }

                    /**
                     * @brief Marks an image as shared between the graphics and compute queues.
                     *
                     * A resource one queue family writes and another reads must either be
                     * created concurrent or have its ownership transferred; the render graph
                     * derives the execution dependency but not the transfer, so a resource an
                     * async-compute pass shares with the graphics queue is created concurrent
                     * here. A no-op when there is no second family, which is why the caller
                     * may apply it unconditionally.
                     *
                     * @param info The create info to mark, filled in place.
                     */
                    void share_across_queues(VkImageCreateInfo& info) const noexcept;

                    /**
                     * @brief Marks a buffer as shared between the graphics and compute queues.
                     * @param info The create info to mark, filled in place.
                     */
                    void share_across_queues(VkBufferCreateInfo& info) const noexcept;

                    /**
                     * @brief Whether device memory can be exported to another API.
                     *
                     * True when the platform's external-memory extension was enabled, which is
                     * what lets an allocation this device owns be imported by SushiRuntime's
                     * SYCL device (matched by @c DeviceInfo::uuid) instead of copied through
                     * the host. The handle getter itself is resolved by the interop module
                     * rather than here, because reaching it means pulling the platform's
                     * Vulkan header — and its Win32 baggage — into every translation unit that
                     * includes this one. False leaves interop buffers unavailable; no render
                     * path depends on them.
                     */
                    bool supports_external_memory() const noexcept
                    {
                        return supports_external_memory_;
                    }

                    /**
                     * @brief Whether the bindless descriptor heap can be created.
                     *
                     * True when the device offered the descriptor-indexing features the
                     * heap needs (runtime arrays, partially bound, update-after-bind).
                     */
                    bool supports_descriptor_indexing() const noexcept
                    {
                        return supports_descriptor_indexing_;
                    }

                    /**
                     * @brief Whether textures may be uploaded straight from host memory.
                     *
                     * True when the 1.4 hostImageCopy feature was offered (the one 1.4
                     * feature that stays optional). When true the texture streamer copies
                     * host pixels into an optimal-tiled image with no staging buffer or
                     * queue submit; when false it keeps the staging + blit upload path.
                     */
                    bool supports_host_image_copy() const noexcept
                    {
                        return supports_host_image_copy_;
                    }

                    /**
                     * @brief Whether pipelines may be built from linked libraries.
                     *
                     * True when VK_EXT_graphics_pipeline_library and its feature were both
                     * available; the pipeline factory falls back to monolithic creation
                     * otherwise.
                     */
                    bool supports_pipeline_library() const noexcept
                    {
                        return supports_pipeline_library_;
                    }

                    /**
                     * @brief Whether a pass may steer its shading rate from an image.
                     *
                     * True when VK_KHR_fragment_shading_rate and its attachment feature
                     * were both available; passes declare the rate image unconditionally
                     * and the graph simply ignores it when this is false.
                     */
                    bool supports_shading_rate_image() const noexcept
                    {
                        return supports_shading_rate_image_;
                    }

                    /** @brief Pixels one texel of a shading rate image must cover, horizontally. */
                    std::uint32_t shading_rate_texel_width() const noexcept
                    {
                        return shading_rate_texel_width_;
                    }

                    /** @brief Pixels one texel of a shading rate image must cover, vertically. */
                    std::uint32_t shading_rate_texel_height() const noexcept
                    {
                        return shading_rate_texel_height_;
                    }

                    /** @brief Widest fragment the device will shade, in pixels. */
                    std::uint32_t max_fragment_width() const noexcept
                    {
                        return max_fragment_width_;
                    }

                    /** @brief Tallest fragment the device will shade, in pixels. */
                    std::uint32_t max_fragment_height() const noexcept
                    {
                        return max_fragment_height_;
                    }

                    /**
                     * @brief Whether a shader may trace a ray against a built structure.
                     *
                     * True when VK_KHR_acceleration_structure and VK_KHR_ray_query were
                     * both available with their features. Ray *query* rather than a ray
                     * tracing pipeline: a shadow ray is a single opaque any-hit test with
                     * no shader table behind it, and a query traces it from inside the
                     * fragment shader that needs the answer.
                     */
                    bool supports_ray_query() const noexcept { return supports_ray_query_; }

                    /**
                     * @brief Whether the device can draw with task and mesh shaders.
                     *
                     * True when VK_EXT_mesh_shader and its task/mesh features were available
                     * and the draw entry points resolved. Meshlet rendering is a tier feature
                     * gated on this; when false the renderer draws the same geometry through
                     * the classic vertex-fetch (or GPU-driven indirect) path, so no hardware
                     * is left unable to render — the mesh-shader path is purely additive.
                     */
                    bool supports_mesh_shader() const noexcept { return supports_mesh_shader_; }

                    /** @brief The mesh-shader draw entry points, or null members. */
                    const MeshShaderFunctions& mesh_shader() const noexcept
                    {
                        return mesh_shader_;
                    }

                    /** @brief Most vertices one mesh-shader workgroup may output. */
                    std::uint32_t max_mesh_output_vertices() const noexcept
                    {
                        return max_mesh_output_vertices_;
                    }

                    /** @brief Most primitives one mesh-shader workgroup may output. */
                    std::uint32_t max_mesh_output_primitives() const noexcept
                    {
                        return max_mesh_output_primitives_;
                    }

                    /**
                     * @brief The acceleration-structure entry points, or null members.
                     *
                     * These are extension functions the loader does not expose as symbols,
                     * so they are resolved once here rather than in every caller.
                     */
                    const RayTracingFunctions& ray_tracing() const noexcept
                    {
                        return ray_tracing_;
                    }

                    /** @brief Alignment every acceleration-structure scratch offset must meet. */
                    std::uint32_t scratch_alignment() const noexcept
                    {
                        return scratch_alignment_;
                    }

                private:
                    vkb::Instance instance_{};
                    VkSurfaceKHR surface_ = VK_NULL_HANDLE;
                    vkb::Device device_{};
                    VmaAllocator allocator_ = VK_NULL_HANDLE;
                    VkQueue graphics_queue_ = VK_NULL_HANDLE;
                    std::uint32_t graphics_queue_family_ = 0;
                    VkQueue async_compute_queue_ = VK_NULL_HANDLE;
                    std::uint32_t async_compute_queue_family_ = 0;
                    /** @brief {graphics, compute} — the pair a concurrent resource names. */
                    std::uint32_t shared_families_[2] = {0, 0};
                    bool supports_external_memory_ = false;
                    bool supports_descriptor_indexing_ = false;
                    bool supports_host_image_copy_ = false;
                    bool supports_pipeline_library_ = false;
                    bool supports_shading_rate_image_ = false;
                    std::uint32_t shading_rate_texel_width_ = 16;
                    std::uint32_t shading_rate_texel_height_ = 16;
                    std::uint32_t max_fragment_width_ = 1;
                    std::uint32_t max_fragment_height_ = 1;
                    bool supports_ray_query_ = false;
                    std::uint32_t scratch_alignment_ = 256;
                    RayTracingFunctions ray_tracing_{};
                    bool supports_mesh_shader_ = false;
                    std::uint32_t max_mesh_output_vertices_ = 0;
                    std::uint32_t max_mesh_output_primitives_ = 0;
                    MeshShaderFunctions mesh_shader_{};
                    DeviceInfo info_{};
            };
        } // namespace Vulkan
    } // namespace Render
} // namespace SushiEngine
