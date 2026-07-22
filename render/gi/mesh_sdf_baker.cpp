/**************************************************************************/
/* mesh_sdf_baker.cpp                                                     */
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
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

#include "gi/mesh_sdf_baker.hpp"

#include "geometry/mesh_registry.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace SushiEngine
{
    namespace Render
    {
        namespace Gi
        {
            namespace
            {
                struct Vec3
                {
                    float x;
                    float y;
                    float z;
                };

                Vec3 subtract(const Vec3& a, const Vec3& b) noexcept
                {
                    return Vec3{a.x - b.x, a.y - b.y, a.z - b.z};
                }

                Vec3 add(const Vec3& a, const Vec3& b) noexcept
                {
                    return Vec3{a.x + b.x, a.y + b.y, a.z + b.z};
                }

                Vec3 scale(const Vec3& a, float s) noexcept
                {
                    return Vec3{a.x * s, a.y * s, a.z * s};
                }

                float dot(const Vec3& a, const Vec3& b) noexcept
                {
                    return a.x * b.x + a.y * b.y + a.z * b.z;
                }

                Vec3 cross(const Vec3& a, const Vec3& b) noexcept
                {
                    return Vec3{a.y * b.z - a.z * b.y,
                                a.z * b.x - a.x * b.z,
                                a.x * b.y - a.y * b.x};
                }

                /** @brief One triangle prepared for the voxel loop. */
                struct Triangle
                {
                    Vec3 v0;
                    Vec3 v1;
                    Vec3 v2;
                    Vec3 normal;    // geometric normal, normalized (zero if degenerate)
                };

                /**
                 * @brief The closest point on a triangle to a query point.
                 *
                 * The region-based routine from Ericson, "Real-Time Collision Detection"
                 * (section 5.1.5): it tests the point against the seven Voronoi regions of
                 * the triangle — three vertices, three edges, and the interior face — and
                 * returns the closest point on whichever region owns the query, so edges and
                 * vertices are handled exactly rather than approximated by the plane.
                 *
                 * @param point The query point.
                 * @param a     First triangle vertex.
                 * @param b     Second triangle vertex.
                 * @param c     Third triangle vertex.
                 * @return The point on the triangle nearest @p point.
                 */
                Vec3 closest_point_on_triangle(const Vec3& point, const Vec3& a,
                                               const Vec3& b, const Vec3& c) noexcept
                {
                    const Vec3 ab = subtract(b, a);
                    const Vec3 ac = subtract(c, a);
                    const Vec3 ap = subtract(point, a);

                    const float d1 = dot(ab, ap);
                    const float d2 = dot(ac, ap);
                    if (d1 <= 0.0f && d2 <= 0.0f)
                    {
                        return a;   // vertex region A
                    }

                    const Vec3 bp = subtract(point, b);
                    const float d3 = dot(ab, bp);
                    const float d4 = dot(ac, bp);
                    if (d3 >= 0.0f && d4 <= d3)
                    {
                        return b;   // vertex region B
                    }

                    const float vc = d1 * d4 - d3 * d2;
                    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
                    {
                        const float v = d1 / (d1 - d3);
                        return add(a, scale(ab, v));    // edge region AB
                    }

                    const Vec3 cp = subtract(point, c);
                    const float d5 = dot(ab, cp);
                    const float d6 = dot(ac, cp);
                    if (d6 >= 0.0f && d5 <= d6)
                    {
                        return c;   // vertex region C
                    }

                    const float vb = d5 * d2 - d1 * d6;
                    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
                    {
                        const float w = d2 / (d2 - d6);
                        return add(a, scale(ac, w));    // edge region AC
                    }

                    const float va = d3 * d6 - d5 * d4;
                    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
                    {
                        const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
                        return add(b, scale(subtract(c, b), w));    // edge region BC
                    }

                    const float denom = 1.0f / (va + vb + vc);
                    const float v = vb * denom;
                    const float w = vc * denom;
                    return add(a, add(scale(ab, v), scale(ac, w)));     // interior face region
                }
            } // namespace

            MeshSdfBrick bake_mesh_sdf(const Geometry::MeshVertex* vertices, std::size_t vertex_count,
                                       const std::uint32_t* indices, std::size_t index_count,
                                       std::int32_t resolution)
            {
                MeshSdfBrick brick;

                if (vertices == nullptr || indices == nullptr || vertex_count == 0 ||
                    index_count < 3 || resolution <= 0)
                {
                    return brick;
                }

                Vec3 aabb_min{std::numeric_limits<float>::max(),
                              std::numeric_limits<float>::max(),
                              std::numeric_limits<float>::max()};
                Vec3 aabb_max{-std::numeric_limits<float>::max(),
                              -std::numeric_limits<float>::max(),
                              -std::numeric_limits<float>::max()};

                for (std::size_t i = 0; i < vertex_count; ++i)
                {
                    const float* p = vertices[i].position;
                    aabb_min.x = std::min(aabb_min.x, p[0]);
                    aabb_min.y = std::min(aabb_min.y, p[1]);
                    aabb_min.z = std::min(aabb_min.z, p[2]);
                    aabb_max.x = std::max(aabb_max.x, p[0]);
                    aabb_max.y = std::max(aabb_max.y, p[1]);
                    aabb_max.z = std::max(aabb_max.z, p[2]);
                }

                const float extent_x = aabb_max.x - aabb_min.x;
                const float extent_y = aabb_max.y - aabb_min.y;
                const float extent_z = aabb_max.z - aabb_min.z;
                const float max_extent = std::max(extent_x, std::max(extent_y, extent_z));

                const float voxel_size = max_extent / static_cast<float>(resolution);
                const float padding = 2.0f * voxel_size;

                aabb_min.x -= padding;
                aabb_min.y -= padding;
                aabb_min.z -= padding;
                aabb_max.x += padding;
                aabb_max.y += padding;
                aabb_max.z += padding;

                std::vector<Triangle> triangles;
                const std::size_t triangle_count = index_count / 3;
                triangles.reserve(triangle_count);

                for (std::size_t t = 0; t < triangle_count; ++t)
                {
                    const std::uint32_t i0 = indices[t * 3 + 0];
                    const std::uint32_t i1 = indices[t * 3 + 1];
                    const std::uint32_t i2 = indices[t * 3 + 2];
                    if (i0 >= vertex_count || i1 >= vertex_count || i2 >= vertex_count)
                    {
                        continue;
                    }

                    const float* p0 = vertices[i0].position;
                    const float* p1 = vertices[i1].position;
                    const float* p2 = vertices[i2].position;

                    Triangle triangle;
                    triangle.v0 = Vec3{p0[0], p0[1], p0[2]};
                    triangle.v1 = Vec3{p1[0], p1[1], p1[2]};
                    triangle.v2 = Vec3{p2[0], p2[1], p2[2]};

                    const Vec3 face = cross(subtract(triangle.v1, triangle.v0),
                                            subtract(triangle.v2, triangle.v0));
                    const float length = std::sqrt(dot(face, face));
                    if (length > 0.0f)
                    {
                        triangle.normal = scale(face, 1.0f / length);
                    }
                    else
                    {
                        triangle.normal = Vec3{0.0f, 0.0f, 0.0f};
                    }
                    triangles.push_back(triangle);
                }

                if (triangles.empty())
                {
                    return brick;
                }

                const Vec3 cell{(aabb_max.x - aabb_min.x) / static_cast<float>(resolution),
                                (aabb_max.y - aabb_min.y) / static_cast<float>(resolution),
                                (aabb_max.z - aabb_min.z) / static_cast<float>(resolution)};

                const std::size_t voxel_count =
                    static_cast<std::size_t>(resolution) *
                    static_cast<std::size_t>(resolution) *
                    static_cast<std::size_t>(resolution);

                brick.aabb_min[0] = aabb_min.x;
                brick.aabb_min[1] = aabb_min.y;
                brick.aabb_min[2] = aabb_min.z;
                brick.aabb_max[0] = aabb_max.x;
                brick.aabb_max[1] = aabb_max.y;
                brick.aabb_max[2] = aabb_max.z;
                brick.resolution = resolution;
                brick.distances.resize(voxel_count);

                for (std::int32_t z = 0; z < resolution; ++z)
                {
                    for (std::int32_t y = 0; y < resolution; ++y)
                    {
                        for (std::int32_t x = 0; x < resolution; ++x)
                        {
                            const Vec3 centre{
                                aabb_min.x + (static_cast<float>(x) + 0.5f) * cell.x,
                                aabb_min.y + (static_cast<float>(y) + 0.5f) * cell.y,
                                aabb_min.z + (static_cast<float>(z) + 0.5f) * cell.z};

                            float best_squared = std::numeric_limits<float>::max();
                            Vec3 best_closest{0.0f, 0.0f, 0.0f};
                            Vec3 best_normal{0.0f, 0.0f, 0.0f};

                            for (const Triangle& triangle : triangles)
                            {
                                const Vec3 closest = closest_point_on_triangle(
                                    centre, triangle.v0, triangle.v1, triangle.v2);
                                const Vec3 offset = subtract(centre, closest);
                                const float squared = dot(offset, offset);
                                if (squared < best_squared)
                                {
                                    best_squared = squared;
                                    best_closest = closest;
                                    best_normal = triangle.normal;
                                }
                            }

                            float distance = std::sqrt(best_squared);
                            const Vec3 outward = subtract(centre, best_closest);
                            if (dot(outward, best_normal) < 0.0f)
                            {
                                distance = -distance;
                            }

                            const std::size_t index =
                                static_cast<std::size_t>(x) +
                                static_cast<std::size_t>(resolution) *
                                    (static_cast<std::size_t>(y) +
                                     static_cast<std::size_t>(resolution) *
                                         static_cast<std::size_t>(z));
                            brick.distances[index] = distance;
                        }
                    }
                }

                return brick;
            }
        } // namespace Gi
    } // namespace Render
} // namespace SushiEngine
