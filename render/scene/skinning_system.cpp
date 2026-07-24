/**************************************************************************/
/* skinning_system.cpp                                                   */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

#include "scene/skinning_system.hpp"

#include <cstring>

#include "geometry/mesh_registry.hpp"
#include "rhi/vulkan/vulkan_check.hpp"
#include "rhi/vulkan/vulkan_device.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Scene
        {
            SkinningSystem::SkinningSystem(Vulkan::VulkanDevice& device, std::uint32_t frame_slots)
                : device_(device)
            {
                palettes_.resize(frame_slots);
                prev_palettes_.resize(frame_slots);
                outputs_.resize(frame_slots);
            }

            SkinningSystem::~SkinningSystem()
            {
                for (Allocation& allocation : palettes_)
                    destroy(allocation);
                for (Allocation& allocation : prev_palettes_)
                    destroy(allocation);
                for (Allocation& allocation : outputs_)
                    destroy(allocation);
            }

            void SkinningSystem::grow(Allocation& target, VkDeviceSize bytes,
                                      VkBufferUsageFlags usage, bool host_visible)
            {
                if (bytes == 0 || bytes <= target.capacity)
                    return;
                destroy(target);

                VkBufferCreateInfo buffer_info{};
                buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
                buffer_info.size = bytes;
                buffer_info.usage = usage;

                VmaAllocationCreateInfo alloc{};
                alloc.usage = VMA_MEMORY_USAGE_AUTO;
                if (host_visible)
                    alloc.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
                                  VMA_ALLOCATION_CREATE_MAPPED_BIT;

                VmaAllocationInfo info{};
                Vulkan::check(vmaCreateBuffer(device_.allocator(), &buffer_info, &alloc,
                                              &target.buffer, &target.allocation, &info),
                              "vmaCreateBuffer(skinning)");
                target.mapped = host_visible ? info.pMappedData : nullptr;
                target.capacity = bytes;
            }

            void SkinningSystem::destroy(Allocation& target)
            {
                if (target.buffer != VK_NULL_HANDLE)
                    vmaDestroyBuffer(device_.allocator(), target.buffer, target.allocation);
                target = Allocation{};
            }

            void SkinningSystem::prepare(std::uint32_t slot, const SkinnedInstance* skinned,
                                         std::size_t count, const Geometry::MeshRegistry& meshes)
            {
                ranges_.clear();
                palette_scratch_.clear();
                prev_scratch_.clear();
                total_joints_ = 0;
                total_vertices_ = 0;
                if (skinned == nullptr || count == 0)
                    return;

                std::uint32_t joint_offset = 0;
                std::uint32_t vertex_offset = 0;
                for (std::size_t i = 0; i < count; ++i)
                {
                    const SkinnedInstance& instance = skinned[i];
                    if (instance.mesh == INVALID_MESH || instance.joint_count == 0 ||
                        instance.palette == nullptr)
                        continue;
                    const Geometry::Mesh& mesh = meshes.mesh(instance.mesh);
                    if (mesh.vertex_count == 0 || mesh.index_count == 0 ||
                        meshes.skin_buffer(instance.mesh) == VK_NULL_HANDLE)
                        continue;

                    const std::size_t palette_bytes =
                        static_cast<std::size_t>(instance.joint_count) * JOINT_MATRIX_SIZE;
                    const std::byte* current = static_cast<const std::byte*>(instance.palette);
                    palette_scratch_.insert(palette_scratch_.end(), current, current + palette_bytes);
                    const std::byte* previous =
                        instance.previous_palette != nullptr
                            ? static_cast<const std::byte*>(instance.previous_palette)
                            : current;
                    prev_scratch_.insert(prev_scratch_.end(), previous, previous + palette_bytes);

                    SkinnedRange range;
                    range.mesh = instance.mesh;
                    range.vertex_count = mesh.vertex_count;
                    range.index_count = mesh.index_count;
                    range.base_vertex = vertex_offset;
                    range.palette_base = joint_offset;
                    range.joint_count = instance.joint_count;
                    range.prev_valid = instance.previous_palette != nullptr ? 1u : 0u;
                    range.id = instance.id;
                    range.model = instance.model;
                    range.material = instance.material;
                    ranges_.push_back(range);

                    joint_offset += instance.joint_count;
                    vertex_offset += mesh.vertex_count;
                }

                total_joints_ = joint_offset;
                total_vertices_ = vertex_offset;
                if (total_vertices_ == 0)
                    return;

                grow(palettes_[slot], palette_scratch_.size(), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                     true);
                std::memcpy(palettes_[slot].mapped, palette_scratch_.data(),
                            palette_scratch_.size());
                grow(prev_palettes_[slot], prev_scratch_.size(), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                     true);
                std::memcpy(prev_palettes_[slot].mapped, prev_scratch_.data(), prev_scratch_.size());

                grow(outputs_[slot], static_cast<VkDeviceSize>(total_vertices_) * SKINNED_VERTEX_SIZE,
                     VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, false);
            }

            VkBuffer SkinningSystem::palette_buffer(std::uint32_t slot) const noexcept
            {
                return palettes_[slot].buffer;
            }

            VkBuffer SkinningSystem::previous_palette_buffer(std::uint32_t slot) const noexcept
            {
                return prev_palettes_[slot].buffer;
            }

            VkBuffer SkinningSystem::output_buffer(std::uint32_t slot) const noexcept
            {
                return outputs_[slot].buffer;
            }

            VkDeviceSize SkinningSystem::palette_range() const noexcept
            {
                return static_cast<VkDeviceSize>(total_joints_) * JOINT_MATRIX_SIZE;
            }

            VkDeviceSize SkinningSystem::output_range() const noexcept
            {
                return static_cast<VkDeviceSize>(total_vertices_) * SKINNED_VERTEX_SIZE;
            }
        } // namespace Scene
    } // namespace Render
} // namespace SushiEngine
