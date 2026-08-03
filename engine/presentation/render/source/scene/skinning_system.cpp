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

#include <algorithm>
#include <cmath>
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
            namespace
            {
                /**
                 * @brief A unit rotation quaternion (xyzw) from a column-major rigid matrix's
                 * upper-left 3x3, ignoring translation. Shepperd's method (four-case branch on
                 * the largest of trace/diagonal to keep the divisor away from zero) — the exact
                 * algebra `SushiEngine::placeholder::quaternion_from_matrix` uses, reimplemented
                 * in plain float here so this file stays independent of the `Animation`
                 * namespace (the renderer only ever sees palette floats, never animation types
                 * — see `scene_view.hpp`'s `SkinnedInstance` comment).
                 */
                void quaternion_from_matrix16(const float* m, float* out_xyzw) noexcept
                {
                    const float m00 = m[0], m10 = m[1], m20 = m[2];
                    const float m01 = m[4], m11 = m[5], m21 = m[6];
                    const float m02 = m[8], m12 = m[9], m22 = m[10];
                    const float trace = m00 + m11 + m22;
                    float x, y, z, w;
                    if (trace > 0.0f)
                    {
                        const float s = std::sqrt(trace + 1.0f) * 2.0f; // s = 4w
                        w = 0.25f * s;
                        x = (m21 - m12) / s;
                        y = (m02 - m20) / s;
                        z = (m10 - m01) / s;
                    }
                    else if (m00 > m11 && m00 > m22)
                    {
                        const float s = std::sqrt(1.0f + m00 - m11 - m22) * 2.0f; // s = 4x
                        w = (m21 - m12) / s;
                        x = 0.25f * s;
                        y = (m01 + m10) / s;
                        z = (m02 + m20) / s;
                    }
                    else if (m11 > m22)
                    {
                        const float s = std::sqrt(1.0f + m11 - m00 - m22) * 2.0f; // s = 4y
                        w = (m02 - m20) / s;
                        x = (m01 + m10) / s;
                        y = 0.25f * s;
                        z = (m12 + m21) / s;
                    }
                    else
                    {
                        const float s = std::sqrt(1.0f + m22 - m00 - m11) * 2.0f; // s = 4z
                        w = (m10 - m01) / s;
                        x = (m02 + m20) / s;
                        y = (m12 + m21) / s;
                        z = 0.25f * s;
                    }
                    out_xyzw[0] = x;
                    out_xyzw[1] = y;
                    out_xyzw[2] = z;
                    out_xyzw[3] = w;
                }

                /**
                 * @brief Converts one 16-float column-major joint (skin) matrix to a unit dual
                 * quaternion (design §12.4, `animation/dual_quaternion_skinning.hpp`).
                 *
                 * Assumes the matrix is rigid (no non-uniform scale/shear) — true of a skin
                 * matrix (`model * inverse_bind`) for any rig without non-uniform-scaled
                 * joints, the same assumption the verified DQS blend math already documents.
                 * `out8` receives 8 floats: real quaternion (xyzw) then dual quaternion (xyzw)
                 * — `dual = 0.5 * (translation, 0) * real` (Hamilton product), matching
                 * `Animation::dual_quaternion_from_rigid` exactly (checked against this
                 * codebase's own `mul` convention, not reimplemented independently — a second,
                 * un-cross-checked derivation here would defeat the point of having verified it
                 * once already).
                 */
                void dual_quaternion_from_matrix16(const float* m, float* out8) noexcept
                {
                    float real[4];
                    quaternion_from_matrix16(m, real);
                    const float norm = std::sqrt(real[0] * real[0] + real[1] * real[1] +
                                                 real[2] * real[2] + real[3] * real[3]);
                    if (norm > 1e-8f)
                    {
                        real[0] /= norm;
                        real[1] /= norm;
                        real[2] /= norm;
                        real[3] /= norm;
                    }
                    else
                    {
                        real[0] = 0.0f;
                        real[1] = 0.0f;
                        real[2] = 0.0f;
                        real[3] = 1.0f;
                    }

                    const float tx = m[12], ty = m[13], tz = m[14];
                    const float rx = real[0], ry = real[1], rz = real[2], rw = real[3];
                    // dual = 0.5 * mul((tx,ty,tz,0), real); mul()'s vector part is
                    // a.w*b_v + b.w*a_v + cross(a_v,b_v), and a.w == 0 here.
                    const float cx = ty * rz - tz * ry;
                    const float cy = tz * rx - tx * rz;
                    const float cz = tx * ry - ty * rx;

                    out8[0] = rx;
                    out8[1] = ry;
                    out8[2] = rz;
                    out8[3] = rw;
                    out8[4] = 0.5f * (rw * tx + cx);
                    out8[5] = 0.5f * (rw * ty + cy);
                    out8[6] = 0.5f * (rw * tz + cz);
                    out8[7] = 0.5f * (-(tx * rx + ty * ry + tz * rz));
                }
            } // namespace

            SkinningSystem::SkinningSystem(Vulkan::VulkanDevice& device, std::uint32_t frame_slots)
                : device_(device)
            {
                palettes_.resize(frame_slots);
                prev_palettes_.resize(frame_slots);
                outputs_.resize(frame_slots);
                morph_weights_.resize(frame_slots);
                dual_quaternion_palettes_.resize(frame_slots);
            }

            SkinningSystem::~SkinningSystem()
            {
                for (Allocation& allocation : palettes_)
                    destroy(allocation);
                for (Allocation& allocation : prev_palettes_)
                    destroy(allocation);
                for (Allocation& allocation : outputs_)
                    destroy(allocation);
                for (Allocation& allocation : morph_weights_)
                    destroy(allocation);
                for (Allocation& allocation : dual_quaternion_palettes_)
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
                morph_weight_scratch_.clear();
                dual_quaternion_scratch_.clear();
                total_joints_ = 0;
                total_vertices_ = 0;
                total_morph_weights_ = 0;
                if (skinned == nullptr || count == 0)
                    return;

                std::uint32_t joint_offset = 0;
                std::uint32_t vertex_offset = 0;
                std::uint32_t morph_weight_offset = 0;
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

                    // Derived, not a second source of truth: one dual quaternion per joint,
                    // converted from the same current-frame matrices just copied above. Copies
                    // through a local float array (memcpy, not a reinterpret-and-dereference)
                    // to stay clear of strict-aliasing UB on the caller-owned palette bytes.
                    for (std::uint32_t j = 0; j < instance.joint_count; ++j)
                    {
                        float matrix16[16];
                        std::memcpy(matrix16, current + static_cast<std::size_t>(j) * JOINT_MATRIX_SIZE,
                                   sizeof(matrix16));
                        float dq[8];
                        dual_quaternion_from_matrix16(matrix16, dq);
                        const std::byte* dq_bytes = reinterpret_cast<const std::byte*>(dq);
                        dual_quaternion_scratch_.insert(dual_quaternion_scratch_.end(), dq_bytes,
                                                        dq_bytes + sizeof(dq));
                    }

                    // Clamped to the mesh's own target count: an instance cannot drive more
                    // targets than its mesh carries, and a mesh with none makes this 0 even if
                    // the instance's evaluator produced weights (e.g. reused across meshes).
                    const std::uint32_t mesh_targets = meshes.morph_target_count(instance.mesh);
                    const std::uint32_t morph_target_count =
                        instance.morph_weights != nullptr
                            ? std::min(instance.morph_target_count, mesh_targets)
                            : 0u;
                    if (morph_target_count > 0)
                    {
                        const std::byte* weights =
                            reinterpret_cast<const std::byte*>(instance.morph_weights);
                        morph_weight_scratch_.insert(morph_weight_scratch_.end(), weights,
                                                     weights + morph_target_count * sizeof(float));
                    }

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
                    range.morph_weight_base = morph_weight_offset;
                    range.morph_target_count = morph_target_count;
                    range.use_dual_quaternion = instance.use_dual_quaternion_skinning ? 1u : 0u;
                    ranges_.push_back(range);

                    joint_offset += instance.joint_count;
                    vertex_offset += mesh.vertex_count;
                    morph_weight_offset += morph_target_count;
                }

                total_joints_ = joint_offset;
                total_vertices_ = vertex_offset;
                total_morph_weights_ = morph_weight_offset;
                if (total_vertices_ == 0)
                    return;

                grow(palettes_[slot], palette_scratch_.size(), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                     true);
                std::memcpy(palettes_[slot].mapped, palette_scratch_.data(),
                            palette_scratch_.size());
                grow(prev_palettes_[slot], prev_scratch_.size(), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                     true);
                std::memcpy(prev_palettes_[slot].mapped, prev_scratch_.data(), prev_scratch_.size());

                grow(dual_quaternion_palettes_[slot], dual_quaternion_scratch_.size(),
                     VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, true);
                std::memcpy(dual_quaternion_palettes_[slot].mapped, dual_quaternion_scratch_.data(),
                            dual_quaternion_scratch_.size());

                if (!morph_weight_scratch_.empty())
                {
                    grow(morph_weights_[slot], morph_weight_scratch_.size(),
                         VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, true);
                    std::memcpy(morph_weights_[slot].mapped, morph_weight_scratch_.data(),
                                morph_weight_scratch_.size());
                }

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

            VkBuffer SkinningSystem::morph_weight_buffer(std::uint32_t slot) const noexcept
            {
                return morph_weights_[slot].buffer;
            }

            VkDeviceSize SkinningSystem::morph_weight_range() const noexcept
            {
                return static_cast<VkDeviceSize>(total_morph_weights_) * sizeof(float);
            }

            VkBuffer SkinningSystem::dual_quaternion_palette_buffer(std::uint32_t slot) const noexcept
            {
                return dual_quaternion_palettes_[slot].buffer;
            }

            VkDeviceSize SkinningSystem::dual_quaternion_palette_range() const noexcept
            {
                return static_cast<VkDeviceSize>(total_joints_) * DUAL_QUATERNION_SIZE;
            }
        } // namespace Scene
    } // namespace Render
} // namespace SushiEngine
