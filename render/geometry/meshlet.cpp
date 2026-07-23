/**************************************************************************/
/* meshlet.cpp                                                            */
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

#include "geometry/meshlet.hpp"

#include <array>
#include <cmath>
#include <cstring>

#include "geometry/mesh_registry.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Geometry
        {
            namespace
            {
                struct Vec3
                {
                    float x = 0.0f;
                    float y = 0.0f;
                    float z = 0.0f;
                };

                Vec3 sub(const Vec3& a, const Vec3& b) noexcept
                {
                    return Vec3{a.x - b.x, a.y - b.y, a.z - b.z};
                }

                Vec3 cross(const Vec3& a, const Vec3& b) noexcept
                {
                    return Vec3{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
                                a.x * b.y - a.y * b.x};
                }

                float dot(const Vec3& a, const Vec3& b) noexcept
                {
                    return a.x * b.x + a.y * b.y + a.z * b.z;
                }

                float length(const Vec3& a) noexcept { return std::sqrt(dot(a, a)); }

                Vec3 position_of(const MeshVertex& vertex) noexcept
                {
                    return Vec3{vertex.position[0], vertex.position[1], vertex.position[2]};
                }

                Vec3 normal_of(const MeshVertex& vertex) noexcept
                {
                    return Vec3{vertex.normal[0], vertex.normal[1], vertex.normal[2]};
                }
            } // namespace

            MeshletData build_meshlets(const MeshVertex* vertices, std::size_t vertex_count,
                                       const std::uint32_t* indices, std::size_t index_count)
            {
                MeshletData data;
                if (vertices == nullptr || indices == nullptr || vertex_count == 0 ||
                    index_count < 3)
                    return data;

                // The current meshlet under construction: which global vertices it holds, and
                // the reverse map so a shared vertex is not added twice.
                std::vector<std::uint32_t> local_to_global;
                std::array<int, 3> triangle_local{};
                // A per-vertex "is it already in this meshlet, and at which local slot" map,
                // reset lazily by stamping the meshlet index rather than clearing the vector.
                std::vector<std::uint32_t> vertex_stamp(vertex_count, 0);
                std::vector<std::uint32_t> vertex_slot(vertex_count, 0);
                std::uint32_t stamp = 0;

                MeshletDescriptor current{};
                current.vertex_offset = 0;
                current.triangle_offset = 0;
                current.vertex_count = 0;
                current.triangle_count = 0;
                local_to_global.clear();
                ++stamp;

                const auto flush = [&]()
                {
                    if (current.triangle_count == 0)
                        return;

                    // Bounding sphere: the AABB centre of the meshlet's vertices, radius the
                    // farthest vertex from it — conservative, never clips a visible cluster.
                    Vec3 lo{1e30f, 1e30f, 1e30f};
                    Vec3 hi{-1e30f, -1e30f, -1e30f};
                    for (std::uint32_t i = 0; i < current.vertex_count; ++i)
                    {
                        const Vec3 p = position_of(vertices[local_to_global[current.vertex_offset + i]]);
                        lo.x = std::fmin(lo.x, p.x);
                        lo.y = std::fmin(lo.y, p.y);
                        lo.z = std::fmin(lo.z, p.z);
                        hi.x = std::fmax(hi.x, p.x);
                        hi.y = std::fmax(hi.y, p.y);
                        hi.z = std::fmax(hi.z, p.z);
                    }
                    const Vec3 centre{(lo.x + hi.x) * 0.5f, (lo.y + hi.y) * 0.5f,
                                      (lo.z + hi.z) * 0.5f};
                    float radius = 0.0f;
                    for (std::uint32_t i = 0; i < current.vertex_count; ++i)
                    {
                        const Vec3 p = position_of(vertices[local_to_global[current.vertex_offset + i]]);
                        radius = std::fmax(radius, length(sub(p, centre)));
                    }
                    current.bounding_sphere[0] = centre.x;
                    current.bounding_sphere[1] = centre.y;
                    current.bounding_sphere[2] = centre.z;
                    current.bounding_sphere[3] = radius;

                    // Normal cone: the normalised average of the triangle normals is the axis,
                    // and the tightest angle any triangle normal makes with it sets the cutoff.
                    // A spread of 90 degrees or more cannot be culled, so the cutoff is pushed
                    // past one to mark the cone as always-visible.
                    Vec3 axis{0.0f, 0.0f, 0.0f};
                    for (std::uint32_t t = 0; t < current.triangle_count; ++t)
                    {
                        const std::uint32_t packed = data.triangles[current.triangle_offset + t];
                        const std::uint32_t a = local_to_global[current.vertex_offset + (packed & 0xFFu)];
                        const std::uint32_t b =
                            local_to_global[current.vertex_offset + ((packed >> 8) & 0xFFu)];
                        const std::uint32_t c =
                            local_to_global[current.vertex_offset + ((packed >> 16) & 0xFFu)];
                        Vec3 n = cross(sub(position_of(vertices[b]), position_of(vertices[a])),
                                       sub(position_of(vertices[c]), position_of(vertices[a])));
                        const float len = length(n);
                        if (len > 1e-12f)
                        {
                            n.x /= len;
                            n.y /= len;
                            n.z /= len;
                            axis.x += n.x;
                            axis.y += n.y;
                            axis.z += n.z;
                        }
                    }
                    const float axis_len = length(axis);
                    if (axis_len > 1e-6f)
                    {
                        axis.x /= axis_len;
                        axis.y /= axis_len;
                        axis.z /= axis_len;
                        float min_dot = 1.0f;
                        for (std::uint32_t t = 0; t < current.triangle_count; ++t)
                        {
                            const std::uint32_t packed = data.triangles[current.triangle_offset + t];
                            const std::uint32_t a =
                                local_to_global[current.vertex_offset + (packed & 0xFFu)];
                            const std::uint32_t b =
                                local_to_global[current.vertex_offset + ((packed >> 8) & 0xFFu)];
                            const std::uint32_t c =
                                local_to_global[current.vertex_offset + ((packed >> 16) & 0xFFu)];
                            Vec3 n = cross(sub(position_of(vertices[b]), position_of(vertices[a])),
                                           sub(position_of(vertices[c]), position_of(vertices[a])));
                            const float len = length(n);
                            if (len > 1e-12f)
                                min_dot = std::fmin(min_dot, dot(axis, Vec3{n.x / len, n.y / len,
                                                                            n.z / len}));
                        }
                        current.cone[0] = axis.x;
                        current.cone[1] = axis.y;
                        current.cone[2] = axis.z;
                        // Cull when dot(view-to-centre, axis) > sin(spread). A non-positive
                        // min_dot means a spread past 90 degrees: never cull.
                        current.cone[3] = min_dot > 0.0f
                                              ? std::sqrt(std::fmax(0.0f, 1.0f - min_dot * min_dot))
                                              : 2.0f;
                    }
                    else
                    {
                        current.cone[0] = 0.0f;
                        current.cone[1] = 0.0f;
                        current.cone[2] = 1.0f;
                        current.cone[3] = 2.0f; // never cull
                    }

                    data.descriptors.push_back(current);
                };

                for (std::size_t i = 0; i + 2 < index_count; i += 3)
                {
                    const std::uint32_t tri[3] = {indices[i], indices[i + 1], indices[i + 2]};
                    if (tri[0] >= vertex_count || tri[1] >= vertex_count || tri[2] >= vertex_count)
                        continue;

                    // Count how many of this triangle's vertices are new to the current meshlet.
                    std::uint32_t added = 0;
                    for (std::uint32_t v = 0; v < 3; ++v)
                        if (vertex_stamp[tri[v]] != stamp)
                        {
                            // Distinct new vertices only (a triangle may repeat one).
                            bool seen_here = false;
                            for (std::uint32_t w = 0; w < v; ++w)
                                if (tri[w] == tri[v])
                                    seen_here = true;
                            if (!seen_here)
                                ++added;
                        }

                    // Start a new meshlet if this triangle would overflow either cap.
                    if (current.triangle_count >= MESHLET_MAX_TRIANGLES ||
                        current.vertex_count + added > MESHLET_MAX_VERTICES)
                    {
                        flush();
                        current = MeshletDescriptor{};
                        current.vertex_offset = static_cast<std::uint32_t>(data.vertices.size());
                        current.triangle_offset =
                            static_cast<std::uint32_t>(data.triangles.size());
                        ++stamp;
                    }

                    // Resolve (or add) each vertex's meshlet-local slot.
                    for (std::uint32_t v = 0; v < 3; ++v)
                    {
                        const std::uint32_t global = tri[v];
                        if (vertex_stamp[global] != stamp)
                        {
                            vertex_stamp[global] = stamp;
                            vertex_slot[global] = current.vertex_count;
                            data.vertices.push_back(global);
                            local_to_global.push_back(global);
                            ++current.vertex_count;
                        }
                        triangle_local[v] = static_cast<int>(vertex_slot[global]);
                    }

                    const std::uint32_t packed =
                        static_cast<std::uint32_t>(triangle_local[0]) |
                        (static_cast<std::uint32_t>(triangle_local[1]) << 8) |
                        (static_cast<std::uint32_t>(triangle_local[2]) << 16);
                    data.triangles.push_back(packed);
                    ++current.triangle_count;
                }

                flush();
                return data;
            }
        } // namespace Geometry
    } // namespace Render
} // namespace SushiEngine
