/**************************************************************************/
/* soft_body_asset.cpp                                                    */
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

#include <SushiEngine/physics/cooking/soft_body_asset.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        namespace Cooking
        {
            namespace
            {
                /**
                 * @brief Hands out aligned section offsets, one call per section.
                 *
                 * A loop instead of twenty hand-written cursor expressions. That is the whole
                 * reason it exists: the arithmetic is identical every time, and a format this
                 * wide written out by hand acquires exactly one transposed line, whose symptom
                 * is a section reading its neighbour's bytes and producing geometry rather
                 * than a crash.
                 */
                class BlobLayout
                {
                public:
                    explicit BlobLayout(std::size_t header_bytes)
                        : cursor_(align_up(header_bytes, ALIGNMENT))
                    {
                    }

                    /** @brief Reserves @p bytes and returns the offset they start at. */
                    std::uint32_t reserve(std::size_t bytes) noexcept
                    {
                        const std::uint32_t offset = std::uint32_t(cursor_);
                        cursor_ = align_up(cursor_ + bytes, ALIGNMENT);
                        return offset;
                    }

                    /** @brief The blob's total size, with the last section's padding included. */
                    std::size_t total() const noexcept { return cursor_; }

                private:
                    static constexpr std::size_t ALIGNMENT = 16;

                    static std::size_t align_up(std::size_t value, std::size_t alignment) noexcept
                    {
                        return (value + alignment - 1) & ~(alignment - 1);
                    }

                    std::size_t cursor_;
                };

                /** @brief Copies a vector's bytes into the blob at @p offset. */
                template <typename T>
                void write_section(std::byte* base, std::uint32_t offset,
                                   const std::vector<T>& values) noexcept
                {
                    if (!values.empty())
                        std::memcpy(base + offset, values.data(), values.size() * sizeof(T));
                }

                /** @brief Whether a section of @p bytes fits inside a blob of @p total. */
                bool section_fits(std::uint32_t total, std::uint32_t offset,
                                  std::size_t bytes) noexcept
                {
                    if (bytes == 0)
                        return true;
                    if (offset > total)
                        return false;
                    return std::size_t(total) - offset >= bytes;
                }
            } // namespace

            bool build_soft_body_blob(const SoftBodyAsset& asset, std::vector<std::byte>& out)
            {
                out.clear();
                if (asset.levels.empty() || asset.vertices.empty() || asset.tetrahedra.empty())
                    return false;
                if (asset.tetrahedra.size() % 4 != 0)
                    return false;

                const std::size_t vertices = asset.vertices.size();
                const std::size_t tetrahedra = asset.tetrahedra.size() / 4;
                if (asset.vertex_mass.size() != vertices)
                    return false;
                if (asset.rest_volume.size() != tetrahedra)
                    return false;
                if (asset.rest_inverse.size() != tetrahedra * 3)
                    return false;
                if (asset.surface_indices.size() % 3 != 0)
                    return false;

                // Cross-references, which is what a format this wide gets wrong. Each of these
                // unchecked is a read into a neighbouring section — which produces plausible
                // geometry rather than a crash, and is therefore worse than one.
                for (const std::uint32_t index : asset.tetrahedra)
                {
                    if (std::size_t(index) >= vertices)
                        return false;
                }
                for (const std::uint32_t index : asset.surface_indices)
                {
                    if (std::size_t(index) >= vertices)
                        return false;
                }
                for (const SoftBodyBinding& binding : asset.bindings)
                {
                    if (std::size_t(binding.tetrahedron) >= tetrahedra)
                        return false;
                }
                for (const SoftBodyBinding& binding : asset.mappings)
                {
                    if (std::size_t(binding.tetrahedron) >= tetrahedra)
                        return false;
                }
                for (const SoftBodyLevelRecord& level : asset.levels)
                {
                    if (std::size_t(level.first_vertex) + std::size_t(level.vertex_count) >
                        vertices)
                        return false;
                    if (std::size_t(level.first_tetrahedron) +
                            std::size_t(level.tetrahedron_count) >
                        tetrahedra)
                        return false;
                    if (std::size_t(level.first_surface_index) +
                            std::size_t(level.surface_index_count) >
                        asset.surface_indices.size())
                        return false;
                    if (std::size_t(level.first_mapping) + std::size_t(level.mapping_count) >
                        asset.mappings.size())
                        return false;
                }

                const std::size_t surface_triangles =
                    asset.surface_order.size();
                if (!asset.surface_order.empty() &&
                    asset.surface_adjacency.size() != surface_triangles * 3)
                    return false;

                std::size_t field_values = 0;
                if (asset.field.resolution > 0)
                {
                    field_values = std::size_t(asset.field.resolution) *
                                   std::size_t(asset.field.resolution) *
                                   std::size_t(asset.field.resolution);
                    if (asset.field.distances.size() != field_values)
                        return false;
                }

                BlobLayout layout(sizeof(SoftBlobHeader));
                SoftBlobHeader header{};
                header.parameters_offset = layout.reserve(sizeof(CookingParameters));
                header.summary_offset = layout.reserve(sizeof(SoftBodySummary));
                header.levels_offset =
                    layout.reserve(asset.levels.size() * sizeof(SoftBodyLevelRecord));
                header.vertices_offset = layout.reserve(vertices * sizeof(Vector3));
                header.vertex_mass_offset = layout.reserve(vertices * sizeof(Scalar));
                header.tetrahedra_offset =
                    layout.reserve(asset.tetrahedra.size() * sizeof(std::uint32_t));
                header.rest_inverse_offset =
                    layout.reserve(asset.rest_inverse.size() * sizeof(Vector3));
                header.rest_volume_offset = layout.reserve(tetrahedra * sizeof(Scalar));
                header.surface_indices_offset =
                    layout.reserve(asset.surface_indices.size() * sizeof(std::uint32_t));
                header.bindings_offset =
                    layout.reserve(asset.bindings.size() * sizeof(SoftBodyBinding));
                header.mappings_offset =
                    layout.reserve(asset.mappings.size() * sizeof(SoftBodyBinding));
                header.surface_nodes_offset =
                    layout.reserve(asset.surface_nodes.size() * sizeof(MeshBvhNode<Scalar>));
                header.surface_order_offset =
                    layout.reserve(asset.surface_order.size() * sizeof(std::uint32_t));
                header.surface_adjacency_offset =
                    layout.reserve(asset.surface_adjacency.size() * sizeof(std::uint32_t));
                header.field_offset = layout.reserve(field_values * sizeof(float));

                const std::size_t total = layout.total();
                out.assign(total, std::byte{0});
                std::byte* base = out.data();

                std::memcpy(header.magic, SOFT_BLOB_MAGIC, sizeof(header.magic));
                header.version = SOFT_BLOB_VERSION;
                header.total_size = std::uint32_t(total);
                header.level_count = std::uint32_t(asset.levels.size());
                header.vertex_count = std::uint32_t(vertices);
                header.tetrahedron_count = std::uint32_t(tetrahedra);
                header.surface_index_count = std::uint32_t(asset.surface_indices.size());
                header.binding_count = std::uint32_t(asset.bindings.size());
                header.mapping_count = std::uint32_t(asset.mappings.size());
                header.surface_node_count = std::uint32_t(asset.surface_nodes.size());
                header.surface_triangle_count = std::uint32_t(surface_triangles);
                header.field_resolution =
                    asset.field.resolution > 0 ? std::uint32_t(asset.field.resolution) : 0u;
                for (int axis = 0; axis < 3; ++axis)
                {
                    header.field_min[axis] =
                        asset.field.resolution > 0 ? asset.field.aabb_min[axis] : 0.0f;
                    header.field_max[axis] =
                        asset.field.resolution > 0 ? asset.field.aabb_max[axis] : 0.0f;
                }
                std::memcpy(base, &header, sizeof(header));

                // The parameters travel inside the asset (§8.3 stage 10), so a re-cook is
                // reproducible and a mismatch is detectable without the project file.
                std::memcpy(base + header.parameters_offset, &asset.parameters,
                            sizeof(CookingParameters));
                std::memcpy(base + header.summary_offset, &asset.summary, sizeof(SoftBodySummary));

                write_section(base, header.levels_offset, asset.levels);
                write_section(base, header.vertices_offset, asset.vertices);
                write_section(base, header.vertex_mass_offset, asset.vertex_mass);
                write_section(base, header.tetrahedra_offset, asset.tetrahedra);
                write_section(base, header.rest_inverse_offset, asset.rest_inverse);
                write_section(base, header.rest_volume_offset, asset.rest_volume);
                write_section(base, header.surface_indices_offset, asset.surface_indices);
                write_section(base, header.bindings_offset, asset.bindings);
                write_section(base, header.mappings_offset, asset.mappings);
                write_section(base, header.surface_nodes_offset, asset.surface_nodes);
                write_section(base, header.surface_order_offset, asset.surface_order);
                write_section(base, header.surface_adjacency_offset, asset.surface_adjacency);
                write_section(base, header.field_offset, asset.field.distances);
                return true;
            }

            bool validate_soft_body_blob(const std::byte* data, std::size_t size) noexcept
            {
                if (data == nullptr || size < sizeof(SoftBlobHeader))
                    return false;

                SoftBlobHeader header{};
                std::memcpy(&header, data, sizeof(header));
                if (std::memcmp(header.magic, SOFT_BLOB_MAGIC, sizeof(header.magic)) != 0)
                    return false;
                if (header.version != SOFT_BLOB_VERSION)
                    return false;
                if (header.total_size > size)
                    return false;
                if (header.level_count == 0 || header.vertex_count == 0 ||
                    header.tetrahedron_count == 0)
                    return false;

                const std::uint32_t total = header.total_size;
                if (!section_fits(total, header.parameters_offset, sizeof(CookingParameters)))
                    return false;
                if (!section_fits(total, header.summary_offset, sizeof(SoftBodySummary)))
                    return false;
                if (!section_fits(total, header.levels_offset,
                                  std::size_t(header.level_count) * sizeof(SoftBodyLevelRecord)))
                    return false;
                if (!section_fits(total, header.vertices_offset,
                                  std::size_t(header.vertex_count) * sizeof(Vector3)))
                    return false;
                if (!section_fits(total, header.vertex_mass_offset,
                                  std::size_t(header.vertex_count) * sizeof(Scalar)))
                    return false;
                if (!section_fits(total, header.tetrahedra_offset,
                                  std::size_t(header.tetrahedron_count) * 4 *
                                      sizeof(std::uint32_t)))
                    return false;
                if (!section_fits(total, header.rest_inverse_offset,
                                  std::size_t(header.tetrahedron_count) * 3 * sizeof(Vector3)))
                    return false;
                if (!section_fits(total, header.rest_volume_offset,
                                  std::size_t(header.tetrahedron_count) * sizeof(Scalar)))
                    return false;
                if (!section_fits(total, header.surface_indices_offset,
                                  std::size_t(header.surface_index_count) * sizeof(std::uint32_t)))
                    return false;
                if (!section_fits(total, header.bindings_offset,
                                  std::size_t(header.binding_count) * sizeof(SoftBodyBinding)))
                    return false;
                if (!section_fits(total, header.mappings_offset,
                                  std::size_t(header.mapping_count) * sizeof(SoftBodyBinding)))
                    return false;
                if (!section_fits(total, header.surface_nodes_offset,
                                  std::size_t(header.surface_node_count) *
                                      sizeof(MeshBvhNode<Scalar>)))
                    return false;
                if (!section_fits(total, header.surface_order_offset,
                                  std::size_t(header.surface_triangle_count) *
                                      sizeof(std::uint32_t)))
                    return false;
                if (!section_fits(total, header.surface_adjacency_offset,
                                  std::size_t(header.surface_triangle_count) * 3 *
                                      sizeof(std::uint32_t)))
                    return false;
                if (header.field_resolution > 0)
                {
                    const std::size_t values = std::size_t(header.field_resolution) *
                                               std::size_t(header.field_resolution) *
                                               std::size_t(header.field_resolution);
                    if (!section_fits(total, header.field_offset, values * sizeof(float)))
                        return false;
                }

                // Checked here and not only at write time, because a blob may have been
                // produced by an older writer or edited by hand.
                const std::uint32_t* tetrahedra =
                    reinterpret_cast<const std::uint32_t*>(data + header.tetrahedra_offset);
                for (std::uint32_t i = 0; i < header.tetrahedron_count * 4; ++i)
                {
                    if (tetrahedra[i] >= header.vertex_count)
                        return false;
                }
                const std::uint32_t* surface =
                    reinterpret_cast<const std::uint32_t*>(data + header.surface_indices_offset);
                for (std::uint32_t i = 0; i < header.surface_index_count; ++i)
                {
                    if (surface[i] >= header.vertex_count)
                        return false;
                }
                const SoftBodyBinding* bindings =
                    reinterpret_cast<const SoftBodyBinding*>(data + header.bindings_offset);
                for (std::uint32_t i = 0; i < header.binding_count; ++i)
                {
                    if (bindings[i].tetrahedron >= header.tetrahedron_count)
                        return false;
                }
                const SoftBodyBinding* mappings =
                    reinterpret_cast<const SoftBodyBinding*>(data + header.mappings_offset);
                for (std::uint32_t i = 0; i < header.mapping_count; ++i)
                {
                    if (mappings[i].tetrahedron >= header.tetrahedron_count)
                        return false;
                }
                const SoftBodyLevelRecord* levels =
                    reinterpret_cast<const SoftBodyLevelRecord*>(data + header.levels_offset);
                for (std::uint32_t i = 0; i < header.level_count; ++i)
                {
                    if (std::size_t(levels[i].first_vertex) + std::size_t(levels[i].vertex_count) >
                        std::size_t(header.vertex_count))
                        return false;
                    if (std::size_t(levels[i].first_tetrahedron) +
                            std::size_t(levels[i].tetrahedron_count) >
                        std::size_t(header.tetrahedron_count))
                        return false;
                    if (std::size_t(levels[i].first_surface_index) +
                            std::size_t(levels[i].surface_index_count) >
                        std::size_t(header.surface_index_count))
                        return false;
                    if (std::size_t(levels[i].first_mapping) +
                            std::size_t(levels[i].mapping_count) >
                        std::size_t(header.mapping_count))
                        return false;
                }
                if (header.surface_triangle_count > 0)
                {
                    const std::uint32_t* order =
                        reinterpret_cast<const std::uint32_t*>(data + header.surface_order_offset);
                    for (std::uint32_t i = 0; i < header.surface_triangle_count; ++i)
                    {
                        if (order[i] >= header.surface_triangle_count)
                            return false;
                    }
                }
                return true;
            }

            SoftBodyAssetView load_soft_body_blob(const std::byte* data, std::size_t size) noexcept
            {
                SoftBodyAssetView view;
                if (!validate_soft_body_blob(data, size))
                    return view;

                SoftBlobHeader header{};
                std::memcpy(&header, data, sizeof(header));
                std::memcpy(&view.parameters, data + header.parameters_offset,
                            sizeof(CookingParameters));
                std::memcpy(&view.summary, data + header.summary_offset, sizeof(SoftBodySummary));

                const auto pointer = [data](std::uint32_t offset, std::uint32_t count) -> const void*
                {
                    return count == 0 ? nullptr : static_cast<const void*>(data + offset);
                };

                view.level_count = header.level_count;
                view.levels = static_cast<const SoftBodyLevelRecord*>(
                    pointer(header.levels_offset, header.level_count));
                view.vertex_count = header.vertex_count;
                view.vertices =
                    static_cast<const Vector3*>(pointer(header.vertices_offset, header.vertex_count));
                view.vertex_mass = static_cast<const Scalar*>(
                    pointer(header.vertex_mass_offset, header.vertex_count));
                view.tetrahedron_count = header.tetrahedron_count;
                view.tetrahedra = static_cast<const std::uint32_t*>(
                    pointer(header.tetrahedra_offset, header.tetrahedron_count));
                view.rest_inverse = static_cast<const Vector3*>(
                    pointer(header.rest_inverse_offset, header.tetrahedron_count));
                view.rest_volume = static_cast<const Scalar*>(
                    pointer(header.rest_volume_offset, header.tetrahedron_count));
                view.surface_index_count = header.surface_index_count;
                view.surface_indices = static_cast<const std::uint32_t*>(
                    pointer(header.surface_indices_offset, header.surface_index_count));
                view.binding_count = header.binding_count;
                view.bindings = static_cast<const SoftBodyBinding*>(
                    pointer(header.bindings_offset, header.binding_count));
                view.mapping_count = header.mapping_count;
                view.mappings = static_cast<const SoftBodyBinding*>(
                    pointer(header.mappings_offset, header.mapping_count));
                view.surface_node_count = header.surface_node_count;
                view.surface_triangle_count = header.surface_triangle_count;
                view.surface_nodes = static_cast<const MeshBvhNode<Scalar>*>(
                    pointer(header.surface_nodes_offset, header.surface_node_count));
                view.surface_order = static_cast<const std::uint32_t*>(
                    pointer(header.surface_order_offset, header.surface_triangle_count));
                view.surface_adjacency = static_cast<const std::uint32_t*>(
                    pointer(header.surface_adjacency_offset, header.surface_triangle_count));
                view.field_resolution = header.field_resolution;
                view.distances = static_cast<const float*>(
                    pointer(header.field_offset, header.field_resolution));
                for (int axis = 0; axis < 3; ++axis)
                {
                    view.field_min[axis] = header.field_min[axis];
                    view.field_max[axis] = header.field_max[axis];
                }
                view.valid = true;
                return view;
            }

            Vector3 evaluate_soft_binding(const SoftBodyAssetView& view, std::uint32_t level,
                                          const SoftBodyBinding& binding) noexcept
            {
                if (!view.valid || level >= view.level_count)
                    return Vector3{0, 0, 0};
                const SoftBodyLevelRecord& record = view.levels[level];
                if (binding.tetrahedron < record.first_tetrahedron ||
                    binding.tetrahedron >= record.first_tetrahedron + record.tetrahedron_count)
                    return Vector3{0, 0, 0};

                const std::uint32_t* element =
                    view.tetrahedra + std::size_t(binding.tetrahedron) * 4;
                Vector3 position{0, 0, 0};
                for (int corner = 0; corner < 4; ++corner)
                {
                    position = position +
                               view.vertices[element[corner]] * Scalar(binding.weights[corner]);
                }
                return position;
            }
        } // namespace Cooking
    } // namespace Physics
} // namespace SushiEngine
