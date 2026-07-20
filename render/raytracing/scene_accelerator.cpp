/**************************************************************************/
/* scene_accelerator.cpp                                                  */
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

#include "raytracing/scene_accelerator.hpp"

#include <cstring>

#include "geometry/mesh_registry.hpp"
#include "rhi/vulkan/vulkan_check.hpp"
#include "rhi/vulkan/vulkan_device.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace RayTracing
        {
            namespace
            {
                /** @brief Instance records a slot's buffer starts out able to hold. */
                constexpr std::size_t INITIAL_INSTANCES = 256;

                /**
                 * @brief Rounds a size up to an alignment.
                 * @param value     The size to round.
                 * @param alignment The alignment to meet; must be a power of two.
                 * @return The rounded size.
                 */
                VkDeviceSize align_up(VkDeviceSize value, VkDeviceSize alignment) noexcept
                {
                    if (alignment == 0)
                        return value;
                    return (value + alignment - 1) & ~(alignment - 1);
                }

                /**
                 * @brief Writes a camera-relative transform into a 3x4 row-major matrix.
                 *
                 * Vulkan wants the transform transposed against the engine's column-major
                 * convention and without its bottom row. The eye leaves the translation in
                 * double, before the float cast, exactly as the rasterised path does — a
                 * ray traced against absolute planetary coordinates in single precision
                 * would miss its target by metres.
                 *
                 * @param model  Object-to-world transform, absolute.
                 * @param eye    Camera world position, metres.
                 * @param result Receives the 3x4 matrix.
                 */
                void fill_transform(const Mat4& model, const double eye[3],
                                    VkTransformMatrixKHR& result) noexcept
                {
                    for (int row = 0; row < 3; ++row)
                        for (int column = 0; column < 3; ++column)
                            result.matrix[row][column] =
                                static_cast<float>(model.m[column * 4 + row]);
                    result.matrix[0][3] = static_cast<float>(model.m[12] - eye[0]);
                    result.matrix[1][3] = static_cast<float>(model.m[13] - eye[1]);
                    result.matrix[2][3] = static_cast<float>(model.m[14] - eye[2]);
                }
            } // namespace

            SceneAccelerator::SceneAccelerator(Vulkan::VulkanDevice& device,
                                               Geometry::MeshRegistry& meshes,
                                               std::uint32_t frame_slots)
                : device_(device), meshes_(meshes)
            {
                available_ = device_.supports_ray_query();
                if (!available_)
                    return;
                top_.resize(frame_slots);
                for (TopLevel& top : top_)
                    create_buffer(
                        top.instances,
                        INITIAL_INSTANCES * sizeof(VkAccelerationStructureInstanceKHR),
                        VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR |
                            VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
                        true);
            }

            SceneAccelerator::~SceneAccelerator()
            {
                if (!available_)
                    return;
                const Vulkan::RayTracingFunctions& api = device_.ray_tracing();
                for (auto& entry : bottom_)
                {
                    if (entry.second.structure != VK_NULL_HANDLE)
                        api.destroy_structure(device_.device(), entry.second.structure, nullptr);
                    destroy_buffer(entry.second.storage);
                }
                bottom_.clear();
                for (TopLevel& top : top_)
                {
                    if (top.structure != VK_NULL_HANDLE)
                        api.destroy_structure(device_.device(), top.structure, nullptr);
                    destroy_buffer(top.storage);
                    destroy_buffer(top.instances);
                }
                top_.clear();
                destroy_buffer(scratch_);
            }

            void SceneAccelerator::create_buffer(Buffer& target, VkDeviceSize bytes,
                                                 VkBufferUsageFlags usage, bool host_visible)
            {
                if (bytes == 0 || bytes <= target.capacity)
                    return;
                destroy_buffer(target);

                VkBufferCreateInfo buffer_info{};
                buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
                buffer_info.size = bytes;
                buffer_info.usage = usage;

                VmaAllocationCreateInfo alloc{};
                alloc.usage = host_visible ? VMA_MEMORY_USAGE_AUTO_PREFER_HOST
                                           : VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
                if (host_visible)
                    alloc.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
                                  VMA_ALLOCATION_CREATE_MAPPED_BIT;

                VmaAllocationInfo info{};
                Vulkan::check(vmaCreateBuffer(device_.allocator(), &buffer_info, &alloc,
                                              &target.buffer, &target.allocation, &info),
                              "vmaCreateBuffer(acceleration structure)");
                target.mapped = info.pMappedData;
                target.capacity = bytes;
            }

            void SceneAccelerator::destroy_buffer(Buffer& target)
            {
                if (target.buffer != VK_NULL_HANDLE)
                    vmaDestroyBuffer(device_.allocator(), target.buffer, target.allocation);
                target.buffer = VK_NULL_HANDLE;
                target.allocation = VK_NULL_HANDLE;
                target.mapped = nullptr;
                target.capacity = 0;
            }

            VkDeviceAddress SceneAccelerator::address_of(VkBuffer buffer) const
            {
                VkBufferDeviceAddressInfo info{};
                info.sType = VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO;
                info.buffer = buffer;
                return vkGetBufferDeviceAddress(device_.device(), &info);
            }

            void SceneAccelerator::describe_geometry(
                const Geometry::Mesh& mesh,
                VkAccelerationStructureGeometryKHR& geometry) const
            {
                geometry = VkAccelerationStructureGeometryKHR{};
                geometry.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;
                geometry.geometryType = VK_GEOMETRY_TYPE_TRIANGLES_KHR;
                // Opaque, so a traced ray never leaves the hardware to run an any-hit
                // shader. A shadow ray only has to know that something was hit.
                geometry.flags = VK_GEOMETRY_OPAQUE_BIT_KHR;

                VkAccelerationStructureGeometryTrianglesDataKHR& triangles =
                    geometry.geometry.triangles;
                triangles.sType =
                    VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_TRIANGLES_DATA_KHR;
                triangles.vertexFormat = VK_FORMAT_R32G32B32_SFLOAT;
                triangles.vertexData.deviceAddress = address_of(mesh.vertices);
                triangles.vertexStride = sizeof(Geometry::MeshVertex);
                triangles.maxVertex = mesh.vertex_count > 0 ? mesh.vertex_count - 1 : 0;
                triangles.indexType = VK_INDEX_TYPE_UINT32;
                triangles.indexData.deviceAddress = address_of(mesh.indices);
            }

            void SceneAccelerator::stage_bottom_level(const Geometry::Mesh& mesh)
            {
                if (mesh.vertices == VK_NULL_HANDLE || mesh.indices == VK_NULL_HANDLE ||
                    mesh.index_count < 3)
                    return;
                // Keyed on the vertex buffer, which is the mesh's stable identity: the
                // registry hands the same handle back for the same geometry, and two
                // instances of one mesh have to share one structure.
                if (bottom_.find(mesh.vertices) != bottom_.end())
                    return;
                for (const Pending& staged : pending_)
                    if (staged.key == mesh.vertices)
                        return;

                const Vulkan::RayTracingFunctions& api = device_.ray_tracing();

                Pending pending;
                pending.key = mesh.vertices;
                pending.triangle_count = mesh.index_count / 3;
                describe_geometry(mesh, pending.geometry);

                VkAccelerationStructureBuildGeometryInfoKHR sizing{};
                sizing.sType =
                    VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
                sizing.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
                sizing.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
                sizing.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
                sizing.geometryCount = 1;
                sizing.pGeometries = &pending.geometry;

                VkAccelerationStructureBuildSizesInfoKHR sizes{};
                sizes.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR;
                api.build_sizes(device_.device(),
                                VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR, &sizing,
                                &pending.triangle_count, &sizes);
                if (sizes.accelerationStructureSize == 0)
                    return;

                BottomLevel level;
                create_buffer(level.storage, sizes.accelerationStructureSize,
                              VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR |
                                  VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
                              false);

                VkAccelerationStructureCreateInfoKHR create{};
                create.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR;
                create.buffer = level.storage.buffer;
                create.size = sizes.accelerationStructureSize;
                create.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
                Vulkan::check(
                    api.create_structure(device_.device(), &create, nullptr, &level.structure),
                    "vkCreateAccelerationStructureKHR(bottom)");

                VkAccelerationStructureDeviceAddressInfoKHR address{};
                address.sType =
                    VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR;
                address.accelerationStructure = level.structure;
                level.address = api.structure_address(device_.device(), &address);

                pending.structure = level.structure;
                // Each staged build takes its own slice of the one scratch buffer. Sharing
                // a region would make the builds race; a buffer each would mean
                // reallocating between builds whose scratch address is already recorded.
                pending.scratch_offset = scratch_needed_;
                scratch_needed_ = align_up(scratch_needed_ + sizes.buildScratchSize,
                                           device_.scratch_alignment());
                pending_.push_back(pending);
                bottom_.emplace(mesh.vertices, level);
            }

            void SceneAccelerator::record_pending(VkCommandBuffer cmd)
            {
                if (pending_.empty())
                    return;
                const Vulkan::RayTracingFunctions& api = device_.ray_tracing();
                const VkDeviceAddress scratch = address_of(scratch_.buffer);
                for (const Pending& pending : pending_)
                {
                    VkAccelerationStructureBuildGeometryInfoKHR build{};
                    build.sType =
                        VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
                    build.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
                    build.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
                    build.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
                    build.geometryCount = 1;
                    build.pGeometries = &pending.geometry;
                    build.dstAccelerationStructure = pending.structure;
                    build.scratchData.deviceAddress = scratch + pending.scratch_offset;

                    VkAccelerationStructureBuildRangeInfoKHR range{};
                    range.primitiveCount = pending.triangle_count;
                    const VkAccelerationStructureBuildRangeInfoKHR* ranges = &range;
                    api.build_structures(cmd, 1, &build, &ranges);
                }
                pending_.clear();
            }

            void SceneAccelerator::build_top_level(VkCommandBuffer cmd, TopLevel& top,
                                                   VkDeviceSize scratch_offset)
            {
                const Vulkan::RayTracingFunctions& api = device_.ray_tracing();

                VkAccelerationStructureGeometryKHR geometry{};
                geometry.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;
                geometry.geometryType = VK_GEOMETRY_TYPE_INSTANCES_KHR;
                geometry.geometry.instances.sType =
                    VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_INSTANCES_DATA_KHR;
                geometry.geometry.instances.data.deviceAddress =
                    address_of(top.instances.buffer);

                VkAccelerationStructureBuildGeometryInfoKHR build{};
                build.sType =
                    VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
                build.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
                // Fast build rather than fast trace: this one is rebuilt every frame, so
                // what it costs to build is paid as often as what it costs to trace.
                build.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_BUILD_BIT_KHR;
                build.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
                build.geometryCount = 1;
                build.pGeometries = &geometry;
                build.dstAccelerationStructure = top.structure;
                build.scratchData.deviceAddress =
                    address_of(scratch_.buffer) + scratch_offset;

                VkAccelerationStructureBuildRangeInfoKHR range{};
                range.primitiveCount = top.instance_count;
                const VkAccelerationStructureBuildRangeInfoKHR* ranges = &range;
                api.build_structures(cmd, 1, &build, &ranges);
            }

            void SceneAccelerator::build(VkCommandBuffer cmd, std::uint32_t slot,
                                         const MeshInstance* instances, std::size_t count,
                                         const double eye[3])
            {
                if (!available_ || slot >= top_.size())
                    return;
                TopLevel& top = top_[slot];
                top.instance_count = 0;
                records_.clear();
                records_.reserve(count);
                pending_.clear();
                scratch_needed_ = 0;

                // Everything is created and sized first, and only then recorded. Doing it
                // in one pass would mean growing the shared scratch buffer between builds
                // that have already had its address baked into them.
                for (std::size_t i = 0; i < count; ++i)
                {
                    const MeshInstance& instance = instances[i];
                    if (!instance.material.cast_shadows)
                        continue;
                    stage_bottom_level(instance.mesh != INVALID_MESH
                                           ? meshes_.mesh(instance.mesh)
                                           : meshes_.primitive(instance.kind));
                }

                for (std::size_t i = 0; i < count; ++i)
                {
                    const MeshInstance& instance = instances[i];
                    if (!instance.material.cast_shadows)
                        continue;
                    const bool imported = instance.mesh != INVALID_MESH;
                    const Geometry::Mesh& mesh = imported ? meshes_.mesh(instance.mesh)
                                                          : meshes_.primitive(instance.kind);
                    const auto found = bottom_.find(mesh.vertices);
                    if (found == bottom_.end())
                        continue;

                    // An imported mesh carries its own scale; a primitive's unit mesh has
                    // to be mapped onto its shape parameters first, exactly as the raster
                    // path does, or the traced silhouette and the drawn one disagree.
                    const Mat4 model =
                        imported ? instance.model
                                 : mul(instance.model,
                                       Geometry::shape_scale(instance.kind,
                                                             instance.shape_params));

                    VkAccelerationStructureInstanceKHR record{};
                    fill_transform(model, eye, record.transform);
                    record.instanceCustomIndex = instance.id & 0xFFFFFFu;
                    record.mask = 0xFF;
                    record.flags = VK_GEOMETRY_INSTANCE_TRIANGLE_FACING_CULL_DISABLE_BIT_KHR;
                    record.accelerationStructureReference = found->second.address;
                    records_.push_back(record);
                }

                if (records_.empty())
                {
                    // Nothing to trace against — but a structure may still have been
                    // created above, and leaving it unbuilt would leave it unusable for
                    // every later frame that does have something to trace.
                    if (!pending_.empty())
                    {
                        create_buffer(scratch_, scratch_needed_,
                                      VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                          VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
                                      false);
                        record_pending(cmd);
                    }
                    return;
                }

                const VkDeviceSize instance_bytes =
                    records_.size() * sizeof(VkAccelerationStructureInstanceKHR);
                create_buffer(
                    top.instances, instance_bytes,
                    VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR |
                        VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
                    true);
                std::memcpy(top.instances.mapped, records_.data(),
                            static_cast<std::size_t>(instance_bytes));
                top.instance_count = static_cast<std::uint32_t>(records_.size());

                const Vulkan::RayTracingFunctions& api = device_.ray_tracing();

                VkAccelerationStructureGeometryKHR top_geometry{};
                top_geometry.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;
                top_geometry.geometryType = VK_GEOMETRY_TYPE_INSTANCES_KHR;
                top_geometry.geometry.instances.sType =
                    VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_INSTANCES_DATA_KHR;

                VkAccelerationStructureBuildGeometryInfoKHR sizing{};
                sizing.sType =
                    VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
                sizing.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
                sizing.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_BUILD_BIT_KHR;
                sizing.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
                sizing.geometryCount = 1;
                sizing.pGeometries = &top_geometry;

                VkAccelerationStructureBuildSizesInfoKHR sizes{};
                sizes.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR;
                api.build_sizes(device_.device(),
                                VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR, &sizing,
                                &top.instance_count, &sizes);
                if (sizes.accelerationStructureSize == 0)
                {
                    top.instance_count = 0;
                    return;
                }

                // Recreated only when it has to grow; rebuilding into a structure that is
                // already large enough is the common case and costs no allocation.
                if (top.structure == VK_NULL_HANDLE ||
                    sizes.accelerationStructureSize > top.storage.capacity)
                {
                    if (top.structure != VK_NULL_HANDLE)
                        api.destroy_structure(device_.device(), top.structure, nullptr);
                    top.structure = VK_NULL_HANDLE;
                    create_buffer(top.storage, sizes.accelerationStructureSize,
                                  VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR |
                                      VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
                                  false);

                    VkAccelerationStructureCreateInfoKHR create{};
                    create.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR;
                    create.buffer = top.storage.buffer;
                    create.size = sizes.accelerationStructureSize;
                    create.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
                    Vulkan::check(api.create_structure(device_.device(), &create, nullptr,
                                                       &top.structure),
                                  "vkCreateAccelerationStructureKHR(top)");
                }

                const VkDeviceSize top_scratch_offset = scratch_needed_;
                create_buffer(scratch_,
                              align_up(scratch_needed_ + sizes.buildScratchSize,
                                       device_.scratch_alignment()),
                              VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                  VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
                              false);

                record_pending(cmd);

                // The top level reads what the bottom-level builds just wrote, and both
                // are recorded into the same command buffer. The render graph derives
                // image and buffer barriers; an acceleration structure build is neither,
                // so this dependency is emitted by hand — the only place in the renderer
                // where that happens, and only because there is no declaration the graph
                // could have derived it from.
                VkMemoryBarrier2 barrier{};
                barrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER_2;
                barrier.srcStageMask =
                    VK_PIPELINE_STAGE_2_ACCELERATION_STRUCTURE_BUILD_BIT_KHR;
                barrier.srcAccessMask = VK_ACCESS_2_ACCELERATION_STRUCTURE_WRITE_BIT_KHR;
                barrier.dstStageMask =
                    VK_PIPELINE_STAGE_2_ACCELERATION_STRUCTURE_BUILD_BIT_KHR;
                barrier.dstAccessMask = VK_ACCESS_2_ACCELERATION_STRUCTURE_READ_BIT_KHR;
                VkDependencyInfo dependency{};
                dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
                dependency.memoryBarrierCount = 1;
                dependency.pMemoryBarriers = &barrier;
                vkCmdPipelineBarrier2(cmd, &dependency);

                build_top_level(cmd, top, top_scratch_offset);

                // And once more, so the fragment shader that traces against it waits for
                // the build rather than racing it.
                barrier.dstStageMask = VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT;
                vkCmdPipelineBarrier2(cmd, &dependency);
            }

            VkAccelerationStructureKHR SceneAccelerator::top_level(
                std::uint32_t slot) const noexcept
            {
                if (!available_ || slot >= top_.size() || top_[slot].instance_count == 0)
                    return VK_NULL_HANDLE;
                return top_[slot].structure;
            }
        } // namespace RayTracing
    } // namespace Render
} // namespace SushiEngine
