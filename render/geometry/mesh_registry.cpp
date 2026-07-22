/**************************************************************************/
/* mesh_registry.cpp                                                      */
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

#include "geometry/mesh_registry.hpp"

#include <cmath>
#include <cstring>
#include <utility>

#include "gi/sdf_clipmap.hpp"
#include "rhi/vulkan/vulkan_check.hpp"
#include "rhi/vulkan/vulkan_device.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Geometry
        {
            namespace
            {
                constexpr float PI = 3.14159265f;
                constexpr int SPHERE_RINGS = 24;
                constexpr int SPHERE_SEGMENTS = 32;
                constexpr int CYLINDER_SEGMENTS = 32;
                constexpr int GRID_EXTENT = 10;

                /**
                 * @brief Builds a vertex with an explicit tangent, UV, and white colour.
                 * @param position Object-space position.
                 * @param normal   Object-space normal, unit length.
                 * @param tangent  Object-space tangent, unit length.
                 * @param u        First UV coordinate.
                 * @param v        Second UV coordinate.
                 * @return The filled vertex.
                 */
                MeshVertex make_vertex(const float position[3], const float normal[3],
                                       const float tangent[3], float u, float v)
                {
                    MeshVertex vertex{};
                    for (int i = 0; i < 3; ++i)
                    {
                        vertex.position[i] = position[i];
                        vertex.normal[i] = normal[i];
                        vertex.tangent[i] = tangent[i];
                    }
                    vertex.tangent[3] = 1.0f;
                    vertex.uv0[0] = u;
                    vertex.uv0[1] = v;
                    vertex.uv1[0] = u;
                    vertex.uv1[1] = v;
                    for (int i = 0; i < 4; ++i)
                        vertex.color[i] = 255;
                    return vertex;
                }
            } // namespace

            Mat4 shape_scale(MeshKind kind, const Vector3& params) noexcept
            {
                switch (kind)
                {
                    case MeshKind::Sphere:
                        return scaling(Vector3{params.x * 2, params.x * 2, params.x * 2});
                    case MeshKind::Cylinder:
                        return scaling(Vector3{params.x * 2, params.y * 2, params.x * 2});
                    case MeshKind::Box:
                    default:
                        return scaling(Vector3{params.x * 2, params.y * 2, params.z * 2});
                }
            }

            void generate_tangents(MeshVertex* vertices, std::size_t vertex_count,
                                   const std::uint32_t* indices, std::size_t index_count)
            {
                if (vertices == nullptr || indices == nullptr || vertex_count == 0)
                    return;

                std::vector<float> accumulated(vertex_count * 6, 0.0f);
                for (std::size_t i = 0; i + 2 < index_count; i += 3)
                {
                    const std::uint32_t i0 = indices[i];
                    const std::uint32_t i1 = indices[i + 1];
                    const std::uint32_t i2 = indices[i + 2];
                    if (i0 >= vertex_count || i1 >= vertex_count || i2 >= vertex_count)
                        continue;

                    const MeshVertex& v0 = vertices[i0];
                    const MeshVertex& v1 = vertices[i1];
                    const MeshVertex& v2 = vertices[i2];
                    const float edge1[3] = {v1.position[0] - v0.position[0],
                                            v1.position[1] - v0.position[1],
                                            v1.position[2] - v0.position[2]};
                    const float edge2[3] = {v2.position[0] - v0.position[0],
                                            v2.position[1] - v0.position[1],
                                            v2.position[2] - v0.position[2]};
                    const float du1 = v1.uv0[0] - v0.uv0[0];
                    const float dv1 = v1.uv0[1] - v0.uv0[1];
                    const float du2 = v2.uv0[0] - v0.uv0[0];
                    const float dv2 = v2.uv0[1] - v0.uv0[1];
                    const float determinant = du1 * dv2 - du2 * dv1;
                    if (std::fabs(determinant) < 1e-12f)
                        continue;
                    const float inverse = 1.0f / determinant;

                    const float tangent[3] = {(edge1[0] * dv2 - edge2[0] * dv1) * inverse,
                                              (edge1[1] * dv2 - edge2[1] * dv1) * inverse,
                                              (edge1[2] * dv2 - edge2[2] * dv1) * inverse};
                    const float bitangent[3] = {(edge2[0] * du1 - edge1[0] * du2) * inverse,
                                                (edge2[1] * du1 - edge1[1] * du2) * inverse,
                                                (edge2[2] * du1 - edge1[2] * du2) * inverse};
                    for (std::uint32_t index : {i0, i1, i2})
                        for (int axis = 0; axis < 3; ++axis)
                        {
                            accumulated[index * 6 + axis] += tangent[axis];
                            accumulated[index * 6 + 3 + axis] += bitangent[axis];
                        }
                }

                for (std::size_t i = 0; i < vertex_count; ++i)
                {
                    MeshVertex& vertex = vertices[i];
                    const float* tangent = &accumulated[i * 6];
                    const float* bitangent = &accumulated[i * 6 + 3];
                    const float n[3] = {vertex.normal[0], vertex.normal[1], vertex.normal[2]};

                    // Gram-Schmidt: remove the component of the accumulated tangent that
                    // lies along the normal, so the frame stays orthonormal after the
                    // per-triangle contributions have been averaged.
                    const float projection = n[0] * tangent[0] + n[1] * tangent[1] +
                                             n[2] * tangent[2];
                    float orthogonal[3] = {tangent[0] - n[0] * projection,
                                           tangent[1] - n[1] * projection,
                                           tangent[2] - n[2] * projection};
                    const float length = std::sqrt(orthogonal[0] * orthogonal[0] +
                                                   orthogonal[1] * orthogonal[1] +
                                                   orthogonal[2] * orthogonal[2]);
                    if (length < 1e-8f)
                    {
                        vertex.tangent[0] = 0.0f;
                        vertex.tangent[1] = 0.0f;
                        vertex.tangent[2] = 0.0f;
                        vertex.tangent[3] = 0.0f;
                        continue;
                    }
                    for (int axis = 0; axis < 3; ++axis)
                        vertex.tangent[axis] = orthogonal[axis] / length;

                    const float cross[3] = {n[1] * orthogonal[2] - n[2] * orthogonal[1],
                                            n[2] * orthogonal[0] - n[0] * orthogonal[2],
                                            n[0] * orthogonal[1] - n[1] * orthogonal[0]};
                    const float handedness = cross[0] * bitangent[0] + cross[1] * bitangent[1] +
                                             cross[2] * bitangent[2];
                    vertex.tangent[3] = handedness < 0.0f ? -1.0f : 1.0f;
                }
            }

            MeshRegistry::MeshRegistry(Vulkan::VulkanDevice& device) : device_(device)
            {
                // Unit cube centred on the origin. Each face is its own quad with a flat
                // normal, its own 0..1 UV patch, and the tangent along that patch's +U.
                const float h = 0.5f;
                struct Face
                {
                    float normal[3];
                    float tangent[3];
                    float bitangent[3];
                };
                const Face faces[6] = {
                    {{0, 0, 1}, {1, 0, 0}, {0, 1, 0}},   {{0, 0, -1}, {-1, 0, 0}, {0, 1, 0}},
                    {{1, 0, 0}, {0, 0, -1}, {0, 1, 0}},  {{-1, 0, 0}, {0, 0, 1}, {0, 1, 0}},
                    {{0, 1, 0}, {1, 0, 0}, {0, 0, -1}},  {{0, -1, 0}, {1, 0, 0}, {0, 0, 1}},
                };
                std::vector<MeshVertex> cube;
                std::vector<std::uint32_t> cube_index;
                for (const Face& face : faces)
                {
                    const std::uint32_t base = static_cast<std::uint32_t>(cube.size());
                    const float corners[4][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
                    for (int corner = 0; corner < 4; ++corner)
                    {
                        const float s = corners[corner][0];
                        const float t = corners[corner][1];
                        const float position[3] = {
                            (face.normal[0] + s * face.tangent[0] + t * face.bitangent[0]) * h,
                            (face.normal[1] + s * face.tangent[1] + t * face.bitangent[1]) * h,
                            (face.normal[2] + s * face.tangent[2] + t * face.bitangent[2]) * h};
                        cube.push_back(make_vertex(position, face.normal, face.tangent,
                                                   s * 0.5f + 0.5f, t * 0.5f + 0.5f));
                    }
                    const std::uint32_t offsets[6] = {0, 1, 2, 2, 3, 0};
                    for (std::uint32_t offset : offsets)
                        cube_index.push_back(base + offset);
                }

                // Unit UV-sphere, radius 0.5: smooth-shaded, UV from the spherical
                // parameterisation, tangent along increasing longitude.
                std::vector<MeshVertex> sphere;
                std::vector<std::uint32_t> sphere_index;
                for (int ring = 0; ring <= SPHERE_RINGS; ++ring)
                {
                    const float v = static_cast<float>(ring) / SPHERE_RINGS;
                    const float phi = v * PI;
                    for (int segment = 0; segment <= SPHERE_SEGMENTS; ++segment)
                    {
                        const float u = static_cast<float>(segment) / SPHERE_SEGMENTS;
                        const float theta = u * 2.0f * PI;
                        const float normal[3] = {std::sin(phi) * std::cos(theta), std::cos(phi),
                                                 std::sin(phi) * std::sin(theta)};
                        const float position[3] = {normal[0] * 0.5f, normal[1] * 0.5f,
                                                   normal[2] * 0.5f};
                        const float tangent[3] = {-std::sin(theta), 0.0f, std::cos(theta)};
                        sphere.push_back(make_vertex(position, normal, tangent, u, 1.0f - v));
                    }
                }
                for (int ring = 0; ring < SPHERE_RINGS; ++ring)
                    for (int segment = 0; segment < SPHERE_SEGMENTS; ++segment)
                    {
                        const std::uint32_t a =
                            static_cast<std::uint32_t>(ring * (SPHERE_SEGMENTS + 1) + segment);
                        const std::uint32_t b = a + SPHERE_SEGMENTS + 1;
                        sphere_index.insert(sphere_index.end(), {a, b, a + 1, b, b + 1, a + 1});
                    }

                // Unit cylinder: radius 0.5, half-height 0.5, capped, axis along Y. The
                // side unwraps to a 0..1 UV strip; the caps take a planar projection.
                std::vector<MeshVertex> cylinder;
                std::vector<std::uint32_t> cylinder_index;
                for (int segment = 0; segment <= CYLINDER_SEGMENTS; ++segment)
                {
                    const float u = static_cast<float>(segment) / CYLINDER_SEGMENTS;
                    const float theta = u * 2.0f * PI;
                    const float normal[3] = {std::cos(theta), 0.0f, std::sin(theta)};
                    const float tangent[3] = {-std::sin(theta), 0.0f, std::cos(theta)};
                    const float bottom[3] = {normal[0] * 0.5f, -0.5f, normal[2] * 0.5f};
                    const float top[3] = {normal[0] * 0.5f, 0.5f, normal[2] * 0.5f};
                    cylinder.push_back(make_vertex(bottom, normal, tangent, u, 0.0f));
                    cylinder.push_back(make_vertex(top, normal, tangent, u, 1.0f));
                }
                for (int segment = 0; segment < CYLINDER_SEGMENTS; ++segment)
                {
                    const std::uint32_t a = static_cast<std::uint32_t>(segment * 2);
                    cylinder_index.insert(cylinder_index.end(),
                                          {a, a + 2, a + 1, a + 1, a + 2, a + 3});
                }
                const float up[3] = {0.0f, 1.0f, 0.0f};
                const float down[3] = {0.0f, -1.0f, 0.0f};
                const float cap_tangent[3] = {1.0f, 0.0f, 0.0f};
                const float bottom_origin[3] = {0.0f, -0.5f, 0.0f};
                const float top_origin[3] = {0.0f, 0.5f, 0.0f};
                const std::uint32_t bottom_center = static_cast<std::uint32_t>(cylinder.size());
                cylinder.push_back(make_vertex(bottom_origin, down, cap_tangent, 0.5f, 0.5f));
                const std::uint32_t top_center = static_cast<std::uint32_t>(cylinder.size());
                cylinder.push_back(make_vertex(top_origin, up, cap_tangent, 0.5f, 0.5f));
                for (int segment = 0; segment < CYLINDER_SEGMENTS; ++segment)
                {
                    const float theta0 = static_cast<float>(segment) / CYLINDER_SEGMENTS * 2.0f * PI;
                    const float theta1 =
                        static_cast<float>(segment + 1) / CYLINDER_SEGMENTS * 2.0f * PI;
                    const float x0 = std::cos(theta0) * 0.5f;
                    const float z0 = std::sin(theta0) * 0.5f;
                    const float x1 = std::cos(theta1) * 0.5f;
                    const float z1 = std::sin(theta1) * 0.5f;

                    const float bottom0_position[3] = {x0, -0.5f, z0};
                    const float bottom1_position[3] = {x1, -0.5f, z1};
                    const std::uint32_t bottom0 = static_cast<std::uint32_t>(cylinder.size());
                    cylinder.push_back(make_vertex(bottom0_position, down, cap_tangent, x0 + 0.5f,
                                                   z0 + 0.5f));
                    const std::uint32_t bottom1 = static_cast<std::uint32_t>(cylinder.size());
                    cylinder.push_back(make_vertex(bottom1_position, down, cap_tangent, x1 + 0.5f,
                                                   z1 + 0.5f));
                    cylinder_index.insert(cylinder_index.end(), {bottom_center, bottom1, bottom0});

                    const float top0_position[3] = {x0, 0.5f, z0};
                    const float top1_position[3] = {x1, 0.5f, z1};
                    const std::uint32_t top0 = static_cast<std::uint32_t>(cylinder.size());
                    cylinder.push_back(
                        make_vertex(top0_position, up, cap_tangent, x0 + 0.5f, z0 + 0.5f));
                    const std::uint32_t top1 = static_cast<std::uint32_t>(cylinder.size());
                    cylinder.push_back(
                        make_vertex(top1_position, up, cap_tangent, x1 + 0.5f, z1 + 0.5f));
                    cylinder_index.insert(cylinder_index.end(), {top_center, top0, top1});
                }

                // Ground grid on the XZ plane; the normal and tangent are unused by the
                // line shader, and the UV carries the world position so a future overlay
                // can key off it.
                std::vector<MeshVertex> grid;
                const float span = static_cast<float>(GRID_EXTENT);
                const float grid_tangent[3] = {1.0f, 0.0f, 0.0f};
                for (int i = -GRID_EXTENT; i <= GRID_EXTENT; ++i)
                {
                    const float t = static_cast<float>(i);
                    const float a[3] = {-span, 0.0f, t};
                    const float b[3] = {span, 0.0f, t};
                    const float c[3] = {t, 0.0f, -span};
                    const float d[3] = {t, 0.0f, span};
                    grid.push_back(make_vertex(a, up, grid_tangent, 0.0f, 0.0f));
                    grid.push_back(make_vertex(b, up, grid_tangent, 1.0f, 0.0f));
                    grid.push_back(make_vertex(c, up, grid_tangent, 0.0f, 0.0f));
                    grid.push_back(make_vertex(d, up, grid_tangent, 0.0f, 1.0f));
                }

                box_vertices_ = upload(cube.data(), cube.size() * sizeof(MeshVertex),
                                       VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
                box_indices_ = upload(cube_index.data(), cube_index.size() * sizeof(std::uint32_t),
                                      VK_BUFFER_USAGE_INDEX_BUFFER_BIT);
                sphere_vertices_ = upload(sphere.data(), sphere.size() * sizeof(MeshVertex),
                                          VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
                sphere_indices_ =
                    upload(sphere_index.data(), sphere_index.size() * sizeof(std::uint32_t),
                           VK_BUFFER_USAGE_INDEX_BUFFER_BIT);
                cylinder_vertices_ = upload(cylinder.data(), cylinder.size() * sizeof(MeshVertex),
                                            VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
                cylinder_indices_ =
                    upload(cylinder_index.data(), cylinder_index.size() * sizeof(std::uint32_t),
                           VK_BUFFER_USAGE_INDEX_BUFFER_BIT);
                grid_vertices_ = upload(grid.data(), grid.size() * sizeof(MeshVertex),
                                        VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);

                box_ = Mesh{box_vertices_.buffer, box_indices_.buffer,
                            static_cast<std::uint32_t>(cube.size()),
                            static_cast<std::uint32_t>(cube_index.size())};
                sphere_ = Mesh{sphere_vertices_.buffer, sphere_indices_.buffer,
                               static_cast<std::uint32_t>(sphere.size()),
                               static_cast<std::uint32_t>(sphere_index.size())};
                cylinder_ = Mesh{cylinder_vertices_.buffer, cylinder_indices_.buffer,
                                 static_cast<std::uint32_t>(cylinder.size()),
                                 static_cast<std::uint32_t>(cylinder_index.size())};
                grid_ = Mesh{grid_vertices_.buffer, VK_NULL_HANDLE,
                             static_cast<std::uint32_t>(grid.size()), 0};
            }

            MeshRegistry::~MeshRegistry()
            {
                Allocation* allocations[] = {&box_vertices_,      &box_indices_,
                                             &sphere_vertices_,   &sphere_indices_,
                                             &cylinder_vertices_, &cylinder_indices_,
                                             &grid_vertices_};
                for (Allocation* allocation : allocations)
                    destroy(*allocation);
                for (Imported& entry : imported_)
                {
                    destroy(entry.vertices);
                    destroy(entry.indices);
                }
                imported_.clear();
            }

            const Mesh& MeshRegistry::primitive(MeshKind kind) const noexcept
            {
                switch (kind)
                {
                    case MeshKind::Sphere:
                        return sphere_;
                    case MeshKind::Cylinder:
                        return cylinder_;
                    case MeshKind::Box:
                    default:
                        return box_;
                }
            }

            MeshId MeshRegistry::add_mesh(const MeshVertex* vertices, std::size_t vertex_count,
                                          const std::uint32_t* indices, std::size_t index_count)
            {
                if (vertices == nullptr || indices == nullptr || vertex_count == 0 ||
                    index_count == 0)
                    return INVALID_MESH;

                Imported entry;
                entry.vertices = upload(vertices, vertex_count * sizeof(MeshVertex),
                                        VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
                entry.indices = upload(indices, index_count * sizeof(std::uint32_t),
                                       VK_BUFFER_USAGE_INDEX_BUFFER_BIT);
                entry.mesh = Mesh{entry.vertices.buffer, entry.indices.buffer,
                                  static_cast<std::uint32_t>(vertex_count),
                                  static_cast<std::uint32_t>(index_count)};
                // Bake the mesh's signed-distance brick now, from the caller's fast CPU data,
                // rather than reading it back from the write-combined upload buffer later. The
                // brick feeds probe GI's scene distance clipmap; it costs nothing when unused.
                entry.brick = Gi::bake_mesh_sdf(vertices, vertex_count, indices, index_count,
                                                Gi::SDF_BRICK_RESOLUTION);
                imported_.push_back(std::move(entry));
                return static_cast<MeshId>(imported_.size() - 1);
            }

            const Gi::MeshSdfBrick* MeshRegistry::mesh_brick(MeshId mesh_id) const noexcept
            {
                if (mesh_id == INVALID_MESH || mesh_id >= imported_.size())
                    return nullptr;
                const Gi::MeshSdfBrick& brick = imported_[mesh_id].brick;
                return brick.resolution > 0 ? &brick : nullptr;
            }

            const Mesh& MeshRegistry::mesh(MeshId mesh_id) const noexcept
            {
                if (mesh_id == INVALID_MESH || mesh_id >= imported_.size())
                    return empty_;
                return imported_[mesh_id].mesh;
            }

            MeshRegistry::Allocation MeshRegistry::upload(const void* data, VkDeviceSize bytes,
                                                          VkBufferUsageFlags usage)
            {
                Allocation allocation;
                grow(allocation, bytes, usage);
                if (allocation.mapped != nullptr && data != nullptr)
                    std::memcpy(allocation.mapped, data, static_cast<std::size_t>(bytes));
                return allocation;
            }

            void MeshRegistry::grow(Allocation& target, VkDeviceSize bytes,
                                    VkBufferUsageFlags usage)
            {
                if (bytes == 0 || bytes <= target.capacity)
                    return;
                destroy(target);

                VkBufferCreateInfo buffer_info{};
                buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
                buffer_info.size = bytes;
                buffer_info.usage = usage;
                // An acceleration structure reads geometry by device address rather than
                // through a descriptor, so a mesh that may ever be traced has to be
                // allocated able to have its address taken. Added only where the device
                // can trace at all, because the build-input flag is meaningless — and
                // rejected — without the extension that defines it.
                if (device_.supports_ray_query())
                    buffer_info.usage |=
                        VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
                        VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR;

                VmaAllocationCreateInfo alloc{};
                alloc.usage = VMA_MEMORY_USAGE_AUTO;
                alloc.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
                              VMA_ALLOCATION_CREATE_MAPPED_BIT;

                VmaAllocationInfo info{};
                Vulkan::check(vmaCreateBuffer(device_.allocator(), &buffer_info, &alloc,
                                              &target.buffer, &target.allocation, &info),
                              "vmaCreateBuffer(mesh)");
                target.mapped = info.pMappedData;
                target.capacity = bytes;
            }

            void MeshRegistry::destroy(Allocation& target)
            {
                if (target.buffer != VK_NULL_HANDLE)
                    vmaDestroyBuffer(device_.allocator(), target.buffer, target.allocation);
                target = Allocation{};
            }

        } // namespace Geometry
    } // namespace Render
} // namespace SushiEngine
