/**************************************************************************/
/* gltf_mesh_importer.cpp                                                 */
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

#include <SushiEngine/geometry/gltf_mesh_import.hpp>

#include <cstddef>
#include <cstdint>
#include <vector>

#include <cgltf.h>

namespace SushiEngine
{
    namespace Geometry
    {
        namespace
        {
            /** @brief Applies a column-major 4x4 to a point. */
            void transform_point(const cgltf_float matrix[16], const float in[3], float out[3])
            {
                for (int row = 0; row < 3; ++row)
                {
                    out[row] = float(matrix[row] * cgltf_float(in[0]) +
                                     matrix[4 + row] * cgltf_float(in[1]) +
                                     matrix[8 + row] * cgltf_float(in[2]) + matrix[12 + row]);
                }
            }

            /**
             * @brief Appends one primitive's triangles to @p out, transformed by @p matrix.
             *
             * @return False when the primitive is not something a cooker can read.
             */
            bool append_primitive(const cgltf_primitive& primitive, const cgltf_float matrix[16],
                                 TriangleMesh& out)
            {
                if (primitive.type != cgltf_primitive_type_triangles ||
                    primitive.indices == nullptr)
                    return false;

                const cgltf_accessor* positions = nullptr;
                for (cgltf_size a = 0; a < primitive.attributes_count; ++a)
                {
                    if (primitive.attributes[a].type == cgltf_attribute_type_position)
                    {
                        positions = primitive.attributes[a].data;
                        break;
                    }
                }
                if (positions == nullptr || positions->count == 0)
                    return false;

                const std::uint32_t base = std::uint32_t(out.vertex_count());
                for (cgltf_size v = 0; v < positions->count; ++v)
                {
                    float local[3] = {0.0f, 0.0f, 0.0f};
                    if (!cgltf_accessor_read_float(positions, v, local, 3))
                        return false;
                    float world[3];
                    transform_point(matrix, local, world);
                    out.positions.push_back(world[0]);
                    out.positions.push_back(world[1]);
                    out.positions.push_back(world[2]);
                }

                const cgltf_size index_count = primitive.indices->count;
                for (cgltf_size i = 0; i + 2 < index_count; i += 3)
                {
                    for (int corner = 0; corner < 3; ++corner)
                    {
                        const cgltf_size index =
                            cgltf_accessor_read_index(primitive.indices, i + cgltf_size(corner));
                        out.indices.push_back(base + std::uint32_t(index));
                    }
                }
                return true;
            }
        } // namespace

        bool import_gltf_mesh(const char* path, TriangleMesh& out, GltfMeshImportReport* report)
        {
            out.positions.clear();
            out.indices.clear();
            out.normals.clear();
            if (report != nullptr)
                *report = GltfMeshImportReport{};
            if (path == nullptr)
                return false;

            cgltf_options options{};
            cgltf_data* data = nullptr;
            if (cgltf_parse_file(&options, path, &data) != cgltf_result_success)
                return false;
            if (cgltf_load_buffers(&options, data, path) != cgltf_result_success)
            {
                cgltf_free(data);
                return false;
            }

            GltfMeshImportReport local;
            local.meshes_in_file = std::uint32_t(data->meshes_count);
            if (data->meshes_count > 0 && data->meshes[0].name != nullptr)
                local.first_mesh_name = data->meshes[0].name;

            // Walked over *nodes* and not over meshes, because a node is what carries a
            // transform. A model assembled from instanced nodes imported mesh-by-mesh arrives
            // as a pile of overlapping parts at the origin, which is the classic way a cooked
            // collider comes out the wrong shape.
            for (cgltf_size n = 0; n < data->nodes_count; ++n)
            {
                const cgltf_node& node = data->nodes[n];
                if (node.mesh == nullptr)
                    continue;

                cgltf_float matrix[16];
                cgltf_node_transform_world(&node, matrix);
                for (cgltf_size p = 0; p < node.mesh->primitives_count; ++p)
                {
                    if (append_primitive(node.mesh->primitives[p], matrix, out))
                        ++local.primitives_imported;
                    else
                        ++local.primitives_skipped;
                }
            }

            // A mesh no node references is geometry the file defines and never places. It is
            // still the model's geometry as far as a cooker is concerned, so it is imported at
            // the identity rather than dropped — an exporter that writes an unreferenced mesh
            // is common enough that dropping it would look like a broken importer.
            if (local.primitives_imported == 0)
            {
                static const cgltf_float identity[16] = {1, 0, 0, 0, 0, 1, 0, 0,
                                                         0, 0, 1, 0, 0, 0, 0, 1};
                for (cgltf_size m = 0; m < data->meshes_count; ++m)
                {
                    for (cgltf_size p = 0; p < data->meshes[m].primitives_count; ++p)
                    {
                        if (append_primitive(data->meshes[m].primitives[p], identity, out))
                            ++local.primitives_imported;
                    }
                }
            }

            cgltf_free(data);

            if (report != nullptr)
                *report = local;
            if (out.indices.empty() || out.positions.empty())
            {
                out.positions.clear();
                out.indices.clear();
                return false;
            }

            // An index past the end is a malformed file, and letting one through would put the
            // range check in every cooking stage instead of here once.
            const std::size_t vertices = out.vertex_count();
            for (const std::uint32_t index : out.indices)
            {
                if (std::size_t(index) >= vertices)
                {
                    out.positions.clear();
                    out.indices.clear();
                    return false;
                }
            }
            return true;
        }
    } // namespace Geometry
} // namespace SushiEngine
