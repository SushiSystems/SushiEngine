/**************************************************************************/
/* mesh_distance_query.cpp                                                */
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

#include <SushiEngine/geometry/mesh_distance_query.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

#include <SushiEngine/geometry/mesh_utilities.hpp>

namespace SushiEngine
{
    namespace Geometry
    {
        namespace
        {
            /** @brief Triangles per leaf. Small enough to prune, large enough not to thrash. */
            constexpr std::size_t LEAF_TRIANGLE_BUDGET = 4;

            /** @brief Depth of the query's explicit stack; a median split cannot exceed it. */
            constexpr std::size_t QUERY_STACK_DEPTH = 64;

            /** @brief Squared distance from a point to the nearest point of a box. */
            float distance_squared_to_box(const float point[3], const float minimum[3],
                                          const float maximum[3]) noexcept
            {
                float total = 0.0f;
                for (int axis = 0; axis < 3; ++axis)
                {
                    const float below = minimum[axis] - point[axis];
                    const float above = point[axis] - maximum[axis];
                    const float outside = below > above ? below : above;
                    if (outside > 0.0f)
                        total += outside * outside;
                }
                return total;
            }
        } // namespace

        bool MeshDistanceQuery::build(const TriangleMeshView& mesh)
        {
            triangles_.clear();
            order_.clear();
            nodes_.clear();
            if (!mesh.has_triangles())
                return false;

            const std::size_t source_triangles = mesh.triangle_count();
            triangles_.reserve(source_triangles);
            for (std::size_t t = 0; t < source_triangles; ++t)
            {
                const std::uint32_t i0 = mesh.indices[t * 3 + 0];
                const std::uint32_t i1 = mesh.indices[t * 3 + 1];
                const std::uint32_t i2 = mesh.indices[t * 3 + 2];
                if (i0 >= mesh.vertex_count || i1 >= mesh.vertex_count ||
                    i2 >= mesh.vertex_count)
                    continue;

                Triangle triangle{};
                mesh.read_position(i0, triangle.vertex[0]);
                mesh.read_position(i1, triangle.vertex[1]);
                mesh.read_position(i2, triangle.vertex[2]);

                float edge_one[3];
                float edge_two[3];
                for (int axis = 0; axis < 3; ++axis)
                {
                    edge_one[axis] = triangle.vertex[1][axis] - triangle.vertex[0][axis];
                    edge_two[axis] = triangle.vertex[2][axis] - triangle.vertex[0][axis];
                }
                triangle.normal[0] = edge_one[1] * edge_two[2] - edge_one[2] * edge_two[1];
                triangle.normal[1] = edge_one[2] * edge_two[0] - edge_one[0] * edge_two[2];
                triangle.normal[2] = edge_one[0] * edge_two[1] - edge_one[1] * edge_two[0];
                const float length = std::sqrt(triangle.normal[0] * triangle.normal[0] +
                                               triangle.normal[1] * triangle.normal[1] +
                                               triangle.normal[2] * triangle.normal[2]);
                if (!(length > 0.0f))
                    continue;   // no area: no closest point a neighbour does not already own
                for (int axis = 0; axis < 3; ++axis)
                {
                    triangle.normal[axis] /= length;
                    triangle.centroid[axis] = (triangle.vertex[0][axis] + triangle.vertex[1][axis] +
                                               triangle.vertex[2][axis]) /
                                              3.0f;
                }
                triangles_.push_back(triangle);
            }

            if (triangles_.empty())
                return false;

            order_.resize(triangles_.size());
            for (std::size_t i = 0; i < order_.size(); ++i)
                order_[i] = std::uint32_t(i);

            nodes_.reserve(triangles_.size() * 2);
            build_node(0, order_.size());
            return true;
        }

        std::uint32_t MeshDistanceQuery::build_node(std::size_t first, std::size_t count)
        {
            const std::uint32_t self = std::uint32_t(nodes_.size());
            nodes_.emplace_back();

            float minimum[3] = {0.0f, 0.0f, 0.0f};
            float maximum[3] = {0.0f, 0.0f, 0.0f};
            for (std::size_t i = 0; i < count; ++i)
            {
                const Triangle& triangle = triangles_[order_[first + i]];
                for (int corner = 0; corner < 3; ++corner)
                {
                    for (int axis = 0; axis < 3; ++axis)
                    {
                        const float value = triangle.vertex[corner][axis];
                        if (i == 0 && corner == 0)
                        {
                            minimum[axis] = value;
                            maximum[axis] = value;
                        }
                        else
                        {
                            minimum[axis] = std::min(minimum[axis], value);
                            maximum[axis] = std::max(maximum[axis], value);
                        }
                    }
                }
            }
            for (int axis = 0; axis < 3; ++axis)
            {
                nodes_[self].minimum[axis] = minimum[axis];
                nodes_[self].maximum[axis] = maximum[axis];
            }

            if (count <= LEAF_TRIANGLE_BUDGET)
            {
                nodes_[self].first = std::uint32_t(first);
                nodes_[self].count = std::uint32_t(count);
                return self;
            }

            // Widest axis, median split. The comparison falls back to the triangle index
            // so a set of coincident centroids still partitions the same way every run.
            int axis = 0;
            float widest = maximum[0] - minimum[0];
            for (int candidate = 1; candidate < 3; ++candidate)
            {
                const float extent = maximum[candidate] - minimum[candidate];
                if (extent > widest)
                {
                    widest = extent;
                    axis = candidate;
                }
            }
            const std::size_t middle = count / 2;
            std::nth_element(order_.begin() + std::ptrdiff_t(first),
                             order_.begin() + std::ptrdiff_t(first + middle),
                             order_.begin() + std::ptrdiff_t(first + count),
                             [this, axis](std::uint32_t a, std::uint32_t b) noexcept
                             {
                                 const float ca = triangles_[a].centroid[axis];
                                 const float cb = triangles_[b].centroid[axis];
                                 if (ca != cb)
                                     return ca < cb;
                                 return a < b;
                             });

            nodes_[self].count = 0;
            const std::uint32_t left = build_node(first, middle);
            const std::uint32_t right = build_node(first + middle, count - middle);
            nodes_[self].left = left;
            nodes_[self].right = right;
            return self;
        }

        MeshClosestPoint MeshDistanceQuery::closest_point(const float point[3]) const noexcept
        {
            MeshClosestPoint result;
            if (triangles_.empty())
                return result;

            float best_squared = std::numeric_limits<float>::max();
            std::uint32_t stack[QUERY_STACK_DEPTH];
            std::size_t depth = 0;
            stack[depth++] = 0;

            while (depth > 0)
            {
                const Node& node = nodes_[stack[--depth]];
                if (distance_squared_to_box(point, node.minimum, node.maximum) >= best_squared)
                    continue;

                if (node.count == 0)
                {
                    // Both children are pushed and each is re-tested against the best
                    // distance as it is popped, so a child the first descent made
                    // irrelevant costs one box test rather than a subtree walk.
                    const std::uint32_t left = node.left;
                    const std::uint32_t right = node.right;
                    const float left_distance =
                        distance_squared_to_box(point, nodes_[left].minimum, nodes_[left].maximum);
                    const float right_distance = distance_squared_to_box(
                        point, nodes_[right].minimum, nodes_[right].maximum);
                    // The stack holds at most the tree's height, and a median split over a
                    // 32-bit triangle count cannot exceed thirty levels, so this is
                    // unreachable rather than a subtree quietly abandoned.
                    if (depth + 2 > QUERY_STACK_DEPTH)
                        continue;
                    // Nearer child on top of the stack, so it narrows the bound first.
                    if (left_distance <= right_distance)
                    {
                        stack[depth++] = right;
                        stack[depth++] = left;
                    }
                    else
                    {
                        stack[depth++] = left;
                        stack[depth++] = right;
                    }
                    continue;
                }

                for (std::uint32_t i = 0; i < node.count; ++i)
                {
                    const std::uint32_t index = order_[node.first + i];
                    const Triangle& triangle = triangles_[index];
                    float candidate[3];
                    closest_point_on_triangle(point, triangle.vertex[0], triangle.vertex[1],
                                              triangle.vertex[2], candidate);
                    const float dx = point[0] - candidate[0];
                    const float dy = point[1] - candidate[1];
                    const float dz = point[2] - candidate[2];
                    const float squared = dx * dx + dy * dy + dz * dz;
                    // Strictly less, and ties broken by the lower triangle index, so an
                    // equidistant pair resolves the same way on every run.
                    if (squared < best_squared || (squared == best_squared && index < result.triangle))
                    {
                        best_squared = squared;
                        result.point[0] = candidate[0];
                        result.point[1] = candidate[1];
                        result.point[2] = candidate[2];
                        result.triangle = index;
                        result.valid = true;
                    }
                }
            }

            result.distance = std::sqrt(best_squared);
            return result;
        }

        float MeshDistanceQuery::signed_distance(const float point[3]) const noexcept
        {
            const MeshClosestPoint nearest = closest_point(point);
            if (!nearest.valid)
                return 0.0f;

            const Triangle& triangle = triangles_[nearest.triangle];
            const float toward[3] = {point[0] - nearest.point[0], point[1] - nearest.point[1],
                                     point[2] - nearest.point[2]};
            const float side = toward[0] * triangle.normal[0] + toward[1] * triangle.normal[1] +
                               toward[2] * triangle.normal[2];
            return side < 0.0f ? -nearest.distance : nearest.distance;
        }

        void MeshDistanceQuery::bounds(float minimum[3], float maximum[3]) const noexcept
        {
            for (int axis = 0; axis < 3; ++axis)
            {
                minimum[axis] = nodes_.empty() ? 0.0f : nodes_[0].minimum[axis];
                maximum[axis] = nodes_.empty() ? 0.0f : nodes_[0].maximum[axis];
            }
        }

        std::size_t sample_surface_points(const TriangleMeshView& mesh, std::uint32_t order,
                                          std::vector<float>& out)
        {
            out.clear();
            if (!mesh.has_triangles())
                return 0;
            if (order < 1)
                order = 1;

            const std::size_t triangle_count = mesh.triangle_count();
            const std::size_t per_triangle =
                (std::size_t(order) + 1) * (std::size_t(order) + 2) / 2;
            out.reserve(triangle_count * per_triangle * 3);

            const float inverse_order = 1.0f / float(order);
            for (std::size_t t = 0; t < triangle_count; ++t)
            {
                const std::uint32_t i0 = mesh.indices[t * 3 + 0];
                const std::uint32_t i1 = mesh.indices[t * 3 + 1];
                const std::uint32_t i2 = mesh.indices[t * 3 + 2];
                if (i0 >= mesh.vertex_count || i1 >= mesh.vertex_count ||
                    i2 >= mesh.vertex_count)
                    continue;

                float a[3];
                float b[3];
                float c[3];
                mesh.read_position(i0, a);
                mesh.read_position(i1, b);
                mesh.read_position(i2, c);

                for (std::uint32_t i = 0; i <= order; ++i)
                {
                    for (std::uint32_t j = 0; i + j <= order; ++j)
                    {
                        const float weight_b = float(i) * inverse_order;
                        const float weight_c = float(j) * inverse_order;
                        const float weight_a = 1.0f - weight_b - weight_c;
                        for (int axis = 0; axis < 3; ++axis)
                        {
                            out.push_back(a[axis] * weight_a + b[axis] * weight_b +
                                          c[axis] * weight_c);
                        }
                    }
                }
            }
            return out.size() / 3;
        }

        namespace
        {
            /**
             * @brief The largest sampled distance from @p source to @p target.
             *
             * @param signed_measure When set, the *signed* distance is read and only
             *                       outward departures count; otherwise the unsigned one.
             */
            float sampled_extreme(const TriangleMeshView& source, const MeshDistanceQuery& target,
                                  std::uint32_t order, bool signed_measure)
            {
                if (!target.ready())
                    return 0.0f;

                std::vector<float> samples;
                const std::size_t count = sample_surface_points(source, order, samples);
                float worst = 0.0f;
                for (std::size_t i = 0; i < count; ++i)
                {
                    const float* point = samples.data() + i * 3;
                    const float distance = signed_measure ? target.signed_distance(point)
                                                          : target.closest_point(point).distance;
                    if (distance > worst)
                        worst = distance;
                }
                return worst;
            }
        } // namespace

        float one_sided_hausdorff_distance(const TriangleMeshView& source,
                                           const MeshDistanceQuery& target, std::uint32_t order)
        {
            return sampled_extreme(source, target, order, false);
        }

        float max_protrusion_distance(const TriangleMeshView& source,
                                      const MeshDistanceQuery& target, std::uint32_t order)
        {
            return sampled_extreme(source, target, order, true);
        }
    } // namespace Geometry
} // namespace SushiEngine
