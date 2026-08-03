/**************************************************************************/
/* collision_asset.cpp                                                    */
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

#include <SushiEngine/physics/cooking/collision_asset.hpp>

#include <algorithm>

#include <SushiEngine/physics/cooking/convex_decomposition.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        namespace Cooking
        {
            bool build_collision_blob(const CollisionAsset& asset, std::vector<std::byte>& out)
            {
                out.clear();

                const std::size_t pieces = asset.pieces.size();
                const std::size_t hull_vertices = asset.hull_vertices.size();
                const std::size_t mesh_vertices = asset.mesh_vertices.size();
                const std::size_t mesh_triangles = asset.mesh_indices.size() / 3;
                const std::size_t mesh_nodes = asset.mesh_nodes.size();

                // An asset that collides as nothing is not an asset. Both shapes are
                // checked, since which one is populated depends on the flag.
                if (asset.static_mesh)
                {
                    if (mesh_triangles == 0 || mesh_vertices == 0)
                        return false;
                    if (asset.mesh_indices.size() % 3 != 0)
                        return false;
                    if (asset.mesh_order.size() != mesh_triangles)
                        return false;
                    if (asset.mesh_adjacency.size() != mesh_triangles * 3)
                        return false;
                    for (const std::uint32_t index : asset.mesh_indices)
                    {
                        if (index >= mesh_vertices)
                            return false;
                    }
                    for (const std::uint32_t index : asset.mesh_order)
                    {
                        if (index >= mesh_triangles)
                            return false;
                    }
                }
                else
                {
                    if (pieces == 0 || hull_vertices == 0)
                        return false;
                    // A piece naming a range outside the hull array is an asset whose
                    // symptom, unchecked, is a support function reading past the end of a
                    // file.
                    for (const CollisionPieceRecord& piece : asset.pieces)
                    {
                        if (piece.vertex_count == 0)
                            return false;
                        if (std::size_t(piece.first_vertex) + std::size_t(piece.vertex_count) >
                            hull_vertices)
                            return false;
                    }
                }

                const std::int32_t resolution = asset.field.resolution;
                std::size_t field_values = 0;
                if (resolution > 0)
                {
                    field_values = std::size_t(resolution) * std::size_t(resolution) *
                                   std::size_t(resolution);
                    if (asset.field.distances.size() != field_values)
                        return false;
                }

                std::size_t cursor = detail::collision_align_up(sizeof(CollisionBlobHeader), 16);
                const std::size_t summary_offset = cursor;
                cursor = detail::collision_align_up(cursor + sizeof(CollisionAssetSummary), 16);
                const std::size_t pieces_offset = cursor;
                cursor = detail::collision_align_up(cursor + pieces * sizeof(CollisionPieceRecord),
                                                    16);
                const std::size_t hull_vertices_offset = cursor;
                cursor = detail::collision_align_up(cursor + hull_vertices * sizeof(Vector3), 16);
                const std::size_t mesh_vertices_offset = cursor;
                cursor = detail::collision_align_up(cursor + mesh_vertices * sizeof(Vector3), 16);
                const std::size_t mesh_indices_offset = cursor;
                cursor = detail::collision_align_up(
                    cursor + asset.mesh_indices.size() * sizeof(std::uint32_t), 16);
                const std::size_t mesh_nodes_offset = cursor;
                cursor = detail::collision_align_up(
                    cursor + mesh_nodes * sizeof(MeshBVHNode<Scalar>), 16);
                const std::size_t mesh_order_offset = cursor;
                cursor = detail::collision_align_up(
                    cursor + asset.mesh_order.size() * sizeof(std::uint32_t), 16);
                const std::size_t mesh_adjacency_offset = cursor;
                cursor = detail::collision_align_up(
                    cursor + asset.mesh_adjacency.size() * sizeof(std::uint32_t), 16);
                const std::size_t field_offset = cursor;
                const std::size_t total = cursor + field_values * sizeof(float);

                out.assign(total, std::byte{0});
                std::byte* base = out.data();

                // Value-initialized, so the padding a memcpy would otherwise leave
                // indeterminate is zero. The blob has to be byte-reproducible for the same
                // input or the content-hash cache is keyed on an input with two outputs.
                CollisionBlobHeader header{};
                std::memcpy(header.magic, COLLISION_BLOB_MAGIC, sizeof(header.magic));
                header.version = COLLISION_BLOB_VERSION;
                header.flags = asset.static_mesh ? COLLISION_ASSET_STATIC_MESH : 0u;
                header.total_size = std::uint32_t(total);
                header.piece_count = std::uint32_t(pieces);
                header.hull_vertex_count = std::uint32_t(hull_vertices);
                header.mesh_vertex_count = std::uint32_t(mesh_vertices);
                header.mesh_triangle_count = std::uint32_t(mesh_triangles);
                header.mesh_node_count = std::uint32_t(mesh_nodes);
                header.field_resolution = resolution > 0 ? std::uint32_t(resolution) : 0u;
                header.summary_offset = std::uint32_t(summary_offset);
                header.pieces_offset = std::uint32_t(pieces_offset);
                header.hull_vertices_offset = std::uint32_t(hull_vertices_offset);
                header.mesh_vertices_offset = std::uint32_t(mesh_vertices_offset);
                header.mesh_indices_offset = std::uint32_t(mesh_indices_offset);
                header.mesh_nodes_offset = std::uint32_t(mesh_nodes_offset);
                header.mesh_order_offset = std::uint32_t(mesh_order_offset);
                header.mesh_adjacency_offset = std::uint32_t(mesh_adjacency_offset);
                header.field_offset = std::uint32_t(field_offset);
                for (int axis = 0; axis < 3; ++axis)
                {
                    header.field_min[axis] = resolution > 0 ? asset.field.aabb_min[axis] : 0.0f;
                    header.field_max[axis] = resolution > 0 ? asset.field.aabb_max[axis] : 0.0f;
                }
                std::memcpy(base, &header, sizeof(header));

                CollisionAssetSummary summary = asset.summary;
                summary.element_count = asset.static_mesh ? std::uint32_t(mesh_triangles)
                                                          : std::uint32_t(pieces);
                std::memcpy(base + summary_offset, &summary, sizeof(summary));

                const auto copy_section = [base](std::size_t offset, const void* source,
                                                 std::size_t bytes)
                {
                    if (bytes > 0)
                        std::memcpy(base + offset, source, bytes);
                };
                copy_section(pieces_offset, asset.pieces.data(),
                             pieces * sizeof(CollisionPieceRecord));
                copy_section(hull_vertices_offset, asset.hull_vertices.data(),
                             hull_vertices * sizeof(Vector3));
                copy_section(mesh_vertices_offset, asset.mesh_vertices.data(),
                             mesh_vertices * sizeof(Vector3));
                copy_section(mesh_indices_offset, asset.mesh_indices.data(),
                             asset.mesh_indices.size() * sizeof(std::uint32_t));
                copy_section(mesh_nodes_offset, asset.mesh_nodes.data(),
                             mesh_nodes * sizeof(MeshBVHNode<Scalar>));
                copy_section(mesh_order_offset, asset.mesh_order.data(),
                             asset.mesh_order.size() * sizeof(std::uint32_t));
                copy_section(mesh_adjacency_offset, asset.mesh_adjacency.data(),
                             asset.mesh_adjacency.size() * sizeof(std::uint32_t));
                copy_section(field_offset, asset.field.distances.data(),
                             field_values * sizeof(float));
                return true;
            }

            bool validate_collision_blob(const std::byte* data, std::size_t size) noexcept
            {
                if (data == nullptr || size < sizeof(CollisionBlobHeader))
                    return false;

                CollisionBlobHeader header{};
                std::memcpy(&header, data, sizeof(header));
                if (std::memcmp(header.magic, COLLISION_BLOB_MAGIC, sizeof(header.magic)) != 0)
                    return false;
                if (header.version != COLLISION_BLOB_VERSION)
                    return false;
                if (header.total_size > size)
                    return false;

                const auto section_fits = [&header](std::uint32_t offset, std::size_t bytes) noexcept
                {
                    if (bytes == 0)
                        return true;
                    if (offset > header.total_size)
                        return false;
                    return std::size_t(header.total_size) - offset >= bytes;
                };

                if (!section_fits(header.summary_offset, sizeof(CollisionAssetSummary)))
                    return false;
                if (!section_fits(header.pieces_offset,
                                  std::size_t(header.piece_count) * sizeof(CollisionPieceRecord)))
                    return false;
                if (!section_fits(header.hull_vertices_offset,
                                  std::size_t(header.hull_vertex_count) * sizeof(Vector3)))
                    return false;
                if (!section_fits(header.mesh_vertices_offset,
                                  std::size_t(header.mesh_vertex_count) * sizeof(Vector3)))
                    return false;
                if (!section_fits(header.mesh_indices_offset,
                                  std::size_t(header.mesh_triangle_count) * 3 *
                                      sizeof(std::uint32_t)))
                    return false;
                if (!section_fits(header.mesh_nodes_offset,
                                  std::size_t(header.mesh_node_count) * sizeof(MeshBVHNode<Scalar>)))
                    return false;
                if (!section_fits(header.mesh_order_offset,
                                  std::size_t(header.mesh_triangle_count) * sizeof(std::uint32_t)))
                    return false;
                if (!section_fits(header.mesh_adjacency_offset,
                                  std::size_t(header.mesh_triangle_count) * 3 *
                                      sizeof(std::uint32_t)))
                    return false;

                std::size_t field_values = 0;
                if (header.field_resolution > 0)
                {
                    field_values = std::size_t(header.field_resolution) *
                                   std::size_t(header.field_resolution) *
                                   std::size_t(header.field_resolution);
                    if (!section_fits(header.field_offset, field_values * sizeof(float)))
                        return false;
                }

                const bool is_static = (header.flags & COLLISION_ASSET_STATIC_MESH) != 0;
                if (is_static)
                {
                    if (header.mesh_triangle_count == 0 || header.mesh_vertex_count == 0)
                        return false;
                    // Checked here and not only at write time, because a blob may have been
                    // produced by an older writer or edited by hand, and an index past the
                    // vertex array is a traversal that reads someone else's memory.
                    const std::uint32_t* indices =
                        reinterpret_cast<const std::uint32_t*>(data + header.mesh_indices_offset);
                    for (std::uint32_t i = 0; i < header.mesh_triangle_count * 3; ++i)
                    {
                        if (indices[i] >= header.mesh_vertex_count)
                            return false;
                    }
                    const std::uint32_t* order =
                        reinterpret_cast<const std::uint32_t*>(data + header.mesh_order_offset);
                    for (std::uint32_t i = 0; i < header.mesh_triangle_count; ++i)
                    {
                        if (order[i] >= header.mesh_triangle_count)
                            return false;
                    }
                }
                else
                {
                    if (header.piece_count == 0 || header.hull_vertex_count == 0)
                        return false;
                    const CollisionPieceRecord* pieces =
                        reinterpret_cast<const CollisionPieceRecord*>(data + header.pieces_offset);
                    for (std::uint32_t i = 0; i < header.piece_count; ++i)
                    {
                        if (pieces[i].vertex_count == 0)
                            return false;
                        if (std::size_t(pieces[i].first_vertex) +
                                std::size_t(pieces[i].vertex_count) >
                            std::size_t(header.hull_vertex_count))
                            return false;
                    }
                }
                return true;
            }

            std::size_t collision_asset_wireframe(const CollisionAssetView& view,
                                                  std::vector<float>& out)
            {
                out.clear();
                if (!view.valid)
                    return 0;

                // Each triangle's three edges, deduplicated by vertex pair: a shared edge drawn
                // twice is twice the overlay's cost and, at a glance, a brighter line that reads
                // as a crease the geometry does not have.
                std::vector<std::uint64_t> drawn;

                const auto emit = [&out](const float* a, const float* b)
                {
                    for (int axis = 0; axis < 3; ++axis)
                        out.push_back(a[axis]);
                    for (int axis = 0; axis < 3; ++axis)
                        out.push_back(b[axis]);
                };

                // Per mesh, and `drawn` is reset for each: a piece's hull has its own local
                // vertex numbering, so two pieces cannot share an edge and carrying one dedup
                // set across all of them would only make its linear scan quadratic in the whole
                // asset instead of in one hull.
                const auto emit_mesh = [&](const Geometry::TriangleMesh& mesh)
                {
                    drawn.clear();
                    for (std::size_t t = 0; t < mesh.triangle_count(); ++t)
                    {
                        for (int edge = 0; edge < 3; ++edge)
                        {
                            const std::uint32_t i0 = mesh.indices[t * 3 + std::size_t(edge)];
                            const std::uint32_t i1 =
                                mesh.indices[t * 3 + std::size_t((edge + 1) % 3)];
                            const std::uint32_t low = i0 < i1 ? i0 : i1;
                            const std::uint32_t high = i0 < i1 ? i1 : i0;
                            const std::uint64_t key =
                                (std::uint64_t(low) << 32) | std::uint64_t(high);
                            if (std::find(drawn.begin(), drawn.end(), key) != drawn.end())
                                continue;
                            drawn.push_back(key);
                            emit(mesh.positions.data() + std::size_t(i0) * 3,
                                 mesh.positions.data() + std::size_t(i1) * 3);
                        }
                    }
                };

                if (view.static_mesh)
                {
                    // The cooked triangles are the collider, so they are drawn as they are.
                    Geometry::TriangleMesh mesh;
                    mesh.positions.reserve(std::size_t(view.mesh_vertex_count) * 3);
                    for (std::uint32_t v = 0; v < view.mesh_vertex_count; ++v)
                    {
                        mesh.positions.push_back(float(view.mesh_vertices[v].x));
                        mesh.positions.push_back(float(view.mesh_vertices[v].y));
                        mesh.positions.push_back(float(view.mesh_vertices[v].z));
                    }
                    mesh.indices.assign(view.mesh_indices,
                                        view.mesh_indices +
                                            std::size_t(view.mesh_triangle_count) * 3);
                    emit_mesh(mesh);
                    return out.size() / 6;
                }

                for (std::uint32_t piece = 0; piece < view.piece_count; ++piece)
                {
                    const CollisionPieceRecord& record = view.pieces[piece];
                    std::vector<Vector3> points;
                    points.reserve(record.vertex_count);
                    for (std::uint32_t v = 0; v < record.vertex_count; ++v)
                    {
                        // Stored centre-relative, which is the frame `ConvexHullView` reads them
                        // in; the overlay wants them where the shape actually is.
                        points.push_back(record.center +
                                         view.hull_vertices[record.first_vertex + v]);
                    }

                    Geometry::TriangleMesh hull;
                    if (build_convex_hull_mesh(points, hull))
                    {
                        emit_mesh(hull);
                    }
                    else
                    {
                        // A flat or collinear piece has no faces to draw. Its points still say
                        // where it is, so they are drawn as a star from the centre rather than
                        // left invisible — a piece that shows nothing reads as a piece that is
                        // not there.
                        const float centre[3] = {float(record.center.x), float(record.center.y),
                                                 float(record.center.z)};
                        for (const Vector3& point : points)
                        {
                            const float end[3] = {float(point.x), float(point.y),
                                                  float(point.z)};
                            emit(centre, end);
                        }
                    }
                }
                return out.size() / 6;
            }

            CollisionAssetView load_collision_blob(const std::byte* data, std::size_t size) noexcept
            {
                CollisionAssetView view;
                if (!validate_collision_blob(data, size))
                    return view;

                CollisionBlobHeader header{};
                std::memcpy(&header, data, sizeof(header));
                std::memcpy(&view.summary, data + header.summary_offset, sizeof(view.summary));

                view.static_mesh = (header.flags & COLLISION_ASSET_STATIC_MESH) != 0;
                view.piece_count = header.piece_count;
                view.hull_vertex_count = header.hull_vertex_count;
                view.pieces =
                    header.piece_count == 0
                        ? nullptr
                        : reinterpret_cast<const CollisionPieceRecord*>(data + header.pieces_offset);
                view.hull_vertices =
                    header.hull_vertex_count == 0
                        ? nullptr
                        : reinterpret_cast<const Vector3*>(data + header.hull_vertices_offset);

                view.mesh_vertex_count = header.mesh_vertex_count;
                view.mesh_triangle_count = header.mesh_triangle_count;
                view.mesh_node_count = header.mesh_node_count;
                view.mesh_vertices =
                    header.mesh_vertex_count == 0
                        ? nullptr
                        : reinterpret_cast<const Vector3*>(data + header.mesh_vertices_offset);
                view.mesh_indices =
                    header.mesh_triangle_count == 0
                        ? nullptr
                        : reinterpret_cast<const std::uint32_t*>(data + header.mesh_indices_offset);
                view.mesh_nodes = header.mesh_node_count == 0
                                      ? nullptr
                                      : reinterpret_cast<const MeshBVHNode<Scalar>*>(
                                            data + header.mesh_nodes_offset);
                view.mesh_order =
                    header.mesh_triangle_count == 0
                        ? nullptr
                        : reinterpret_cast<const std::uint32_t*>(data + header.mesh_order_offset);
                view.mesh_adjacency = header.mesh_triangle_count == 0
                                          ? nullptr
                                          : reinterpret_cast<const std::uint32_t*>(
                                                data + header.mesh_adjacency_offset);

                view.field_resolution = header.field_resolution;
                view.distances =
                    header.field_resolution == 0
                        ? nullptr
                        : reinterpret_cast<const float*>(data + header.field_offset);
                for (int axis = 0; axis < 3; ++axis)
                {
                    view.field_min[axis] = header.field_min[axis];
                    view.field_max[axis] = header.field_max[axis];
                }
                view.valid = true;
                return view;
            }
        } // namespace Cooking
    } // namespace Physics
} // namespace SushiEngine
