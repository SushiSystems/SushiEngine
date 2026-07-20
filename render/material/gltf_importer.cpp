/**************************************************************************/
/* gltf_importer.cpp                                                      */
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

#include "material/gltf_importer.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

#include <cgltf.h>

#include "geometry/mesh_registry.hpp"
#include "material/texture_library.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Assets
        {
            namespace
            {
                /**
                 * @brief Loads a glTF texture into the library, or returns an unset id.
                 *
                 * A texture backed by a URI resolves against the file's directory; one
                 * embedded in a buffer view is decoded from memory and cached under a
                 * synthetic name so two references to it share one upload.
                 *
                 * @param view        The glTF texture view, or nullptr for an absent map.
                 * @param textures    Library the image is loaded into.
                 * @param directory   Directory the glTF file was read from.
                 * @param color_space How the image's values are to be interpreted.
                 * @return The texture id, or INVALID_TEXTURE.
                 */
                TextureId load_texture(const cgltf_texture_view* view, TextureLibrary& textures,
                                       const std::filesystem::path& directory,
                                       TextureColorSpace color_space)
                {
                    if (view == nullptr || view->texture == nullptr ||
                        view->texture->image == nullptr)
                        return INVALID_TEXTURE;
                    const cgltf_image* image = view->texture->image;
                    if (image->uri != nullptr && std::strncmp(image->uri, "data:", 5) != 0)
                    {
                        cgltf_decode_uri(const_cast<char*>(image->uri));
                        return textures.load((directory / image->uri).string().c_str(),
                                             color_space);
                    }
                    // Embedded images are already decoded into the buffer by cgltf's own
                    // loader, so the importer only has to hand the bytes over — but stb
                    // still has to interpret the container, which TextureLibrary::load
                    // cannot do from memory, so this path is limited to URI images today.
                    return INVALID_TEXTURE;
                }

                /**
                 * @brief Reads a float attribute into a destination stride.
                 * @param accessor  The glTF accessor to read.
                 * @param components Floats per element the destination expects.
                 * @param destination First destination element.
                 * @param stride_floats Floats between consecutive destination elements.
                 * @param count     Number of elements to read.
                 */
                void read_attribute(const cgltf_accessor* accessor, cgltf_size components,
                                    float* destination, std::size_t stride_floats,
                                    std::size_t count)
                {
                    if (accessor == nullptr)
                        return;
                    std::vector<float> scratch(components);
                    for (std::size_t i = 0; i < count; ++i)
                    {
                        if (!cgltf_accessor_read_float(accessor, i, scratch.data(), components))
                            continue;
                        for (cgltf_size c = 0; c < components; ++c)
                            destination[i * stride_floats + c] = scratch[c];
                    }
                }

                /**
                 * @brief Converts a glTF material into the engine's authoring form.
                 * @param source    The glTF material, or nullptr for the default material.
                 * @param textures  Library the referenced images are loaded into.
                 * @param directory Directory the glTF file was read from.
                 * @return The converted material.
                 */
                Render::Material convert_material(const cgltf_material* source,
                                                  TextureLibrary& textures,
                                                  const std::filesystem::path& directory)
                {
                    Render::Material material;
                    if (source == nullptr)
                        return material;

                    if (source->has_pbr_metallic_roughness)
                    {
                        const cgltf_pbr_metallic_roughness& pbr = source->pbr_metallic_roughness;
                        material.albedo = Vector3{pbr.base_color_factor[0],
                                                  pbr.base_color_factor[1],
                                                  pbr.base_color_factor[2]};
                        material.base_alpha = pbr.base_color_factor[3];
                        material.metallic = pbr.metallic_factor;
                        material.roughness = pbr.roughness_factor;
                        material.albedo_map = load_texture(&pbr.base_color_texture, textures,
                                                           directory, TextureColorSpace::Srgb);
                        material.metallic_roughness_map =
                            load_texture(&pbr.metallic_roughness_texture, textures, directory,
                                         TextureColorSpace::Linear);
                    }
                    else if (source->has_pbr_specular_glossiness)
                    {
                        // Spec-gloss is converted rather than supported: the diffuse becomes
                        // the base colour, the glossiness inverts to roughness, and the
                        // specular strength stands in for metalness. It is lossy for
                        // coloured-specular dielectrics, which the metallic-roughness model
                        // cannot express at all.
                        const cgltf_pbr_specular_glossiness& sg = source->pbr_specular_glossiness;
                        material.albedo = Vector3{sg.diffuse_factor[0], sg.diffuse_factor[1],
                                                  sg.diffuse_factor[2]};
                        material.base_alpha = sg.diffuse_factor[3];
                        material.roughness = 1.0f - sg.glossiness_factor;
                        const float specular = (sg.specular_factor[0] + sg.specular_factor[1] +
                                                sg.specular_factor[2]) / 3.0f;
                        material.metallic = specular > 0.5f ? 1.0f : 0.0f;
                        material.albedo_map = load_texture(&sg.diffuse_texture, textures,
                                                           directory, TextureColorSpace::Srgb);
                    }

                    material.normal_map = load_texture(&source->normal_texture, textures,
                                                       directory, TextureColorSpace::Linear);
                    if (material.normal_map != INVALID_TEXTURE)
                        material.normal_scale = source->normal_texture.scale;

                    material.occlusion_map = load_texture(&source->occlusion_texture, textures,
                                                          directory, TextureColorSpace::Linear);
                    if (material.occlusion_map != INVALID_TEXTURE)
                        material.occlusion_strength = source->occlusion_texture.scale;

                    material.emissive_map = load_texture(&source->emissive_texture, textures,
                                                         directory, TextureColorSpace::Srgb);
                    material.emissive = Vector3{source->emissive_factor[0],
                                                source->emissive_factor[1],
                                                source->emissive_factor[2]};
                    if (source->has_emissive_strength)
                        material.emissive_intensity =
                            source->emissive_strength.emissive_strength;
                    material.emissive_enabled = material.emissive_map != INVALID_TEXTURE ||
                                                material.emissive.x > 0.0 ||
                                                material.emissive.y > 0.0 ||
                                                material.emissive.z > 0.0;

                    // The main tiling/offset is the KHR_texture_transform on the base map;
                    // the engine applies one transform to the whole main set, so the base
                    // map's is the one that governs.
                    if (source->has_pbr_metallic_roughness &&
                        source->pbr_metallic_roughness.base_color_texture.has_transform)
                    {
                        const cgltf_texture_transform& transform =
                            source->pbr_metallic_roughness.base_color_texture.transform;
                        material.main_transform.tiling_x = transform.scale[0];
                        material.main_transform.tiling_y = transform.scale[1];
                        material.main_transform.offset_x = transform.offset[0];
                        material.main_transform.offset_y = transform.offset[1];
                    }

                    if (source->has_clearcoat)
                    {
                        material.clearcoat = source->clearcoat.clearcoat_factor;
                        material.clearcoat_roughness =
                            source->clearcoat.clearcoat_roughness_factor;
                    }
                    if (source->has_sheen)
                        material.sheen_color = Vector3{source->sheen.sheen_color_factor[0],
                                                       source->sheen.sheen_color_factor[1],
                                                       source->sheen.sheen_color_factor[2]};
                    if (source->has_transmission)
                        material.transmission = source->transmission.transmission_factor;
                    if (source->has_volume)
                    {
                        material.thickness = source->volume.thickness_factor;
                        material.subsurface_color =
                            Vector3{source->volume.attenuation_color[0],
                                    source->volume.attenuation_color[1],
                                    source->volume.attenuation_color[2]};
                    }
                    if (source->has_ior)
                        material.ior = source->ior.ior;
                    if (source->has_anisotropy)
                    {
                        material.anisotropy = source->anisotropy.anisotropy_strength;
                        material.anisotropy_rotation =
                            source->anisotropy.anisotropy_rotation / 6.28318530718f;
                    }

                    switch (source->alpha_mode)
                    {
                        case cgltf_alpha_mode_mask:
                            material.surface_type = SurfaceType::Cutout;
                            material.alpha_cutoff = source->alpha_cutoff;
                            break;
                        case cgltf_alpha_mode_blend:
                            material.surface_type = SurfaceType::Transparent;
                            break;
                        case cgltf_alpha_mode_opaque:
                        default:
                            material.surface_type = SurfaceType::Opaque;
                            break;
                    }
                    material.cull_mode = source->double_sided ? MaterialCullMode::Off
                                                              : MaterialCullMode::Back;
                    return material;
                }
            } // namespace

            std::size_t import_gltf(const char* path, Geometry::MeshRegistry& meshes,
                                    TextureLibrary& textures, MeshId* out_meshes,
                                    Render::Material* out_materials, std::size_t capacity)
            {
                if (path == nullptr || out_meshes == nullptr || out_materials == nullptr ||
                    capacity == 0)
                    return 0;

                cgltf_options options{};
                cgltf_data* data = nullptr;
                if (cgltf_parse_file(&options, path, &data) != cgltf_result_success)
                    return 0;
                if (cgltf_load_buffers(&options, data, path) != cgltf_result_success)
                {
                    cgltf_free(data);
                    return 0;
                }

                const std::filesystem::path directory =
                    std::filesystem::path(path).parent_path();
                std::size_t written = 0;

                // Walking the node graph rather than the mesh list is what bakes each
                // primitive into its node's world transform, so a multi-part asset
                // assembles without the render side carrying a scene graph.
                for (cgltf_size node_index = 0; node_index < data->nodes_count && written < capacity;
                     ++node_index)
                {
                    const cgltf_node& node = data->nodes[node_index];
                    if (node.mesh == nullptr)
                        continue;

                    float world[16];
                    cgltf_node_transform_world(&node, world);
                    // The normal transform is the inverse transpose of the upper 3x3; for
                    // the uniform-scale case that dominates in practice the rotation part
                    // is already correct, and a non-uniform scale is renormalised in the
                    // shader, so the plain upper 3x3 is what travels.
                    const float* m = world;

                    for (cgltf_size primitive_index = 0;
                         primitive_index < node.mesh->primitives_count && written < capacity;
                         ++primitive_index)
                    {
                        const cgltf_primitive& primitive = node.mesh->primitives[primitive_index];
                        if (primitive.type != cgltf_primitive_type_triangles ||
                            primitive.indices == nullptr)
                            continue;

                        const cgltf_accessor* position_accessor = nullptr;
                        const cgltf_accessor* normal_accessor = nullptr;
                        const cgltf_accessor* tangent_accessor = nullptr;
                        const cgltf_accessor* uv0_accessor = nullptr;
                        const cgltf_accessor* uv1_accessor = nullptr;
                        const cgltf_accessor* color_accessor = nullptr;
                        for (cgltf_size a = 0; a < primitive.attributes_count; ++a)
                        {
                            const cgltf_attribute& attribute = primitive.attributes[a];
                            switch (attribute.type)
                            {
                                case cgltf_attribute_type_position:
                                    position_accessor = attribute.data;
                                    break;
                                case cgltf_attribute_type_normal:
                                    normal_accessor = attribute.data;
                                    break;
                                case cgltf_attribute_type_tangent:
                                    tangent_accessor = attribute.data;
                                    break;
                                case cgltf_attribute_type_texcoord:
                                    if (attribute.index == 0)
                                        uv0_accessor = attribute.data;
                                    else if (attribute.index == 1)
                                        uv1_accessor = attribute.data;
                                    break;
                                case cgltf_attribute_type_color:
                                    if (attribute.index == 0)
                                        color_accessor = attribute.data;
                                    break;
                                default:
                                    break;
                            }
                        }
                        if (position_accessor == nullptr)
                            continue;

                        const std::size_t vertex_count = position_accessor->count;
                        std::vector<Geometry::MeshVertex> vertices(vertex_count);
                        for (Geometry::MeshVertex& vertex : vertices)
                            for (int i = 0; i < 4; ++i)
                                vertex.color[i] = 255;

                        constexpr std::size_t STRIDE =
                            sizeof(Geometry::MeshVertex) / sizeof(float);
                        float* base = reinterpret_cast<float*>(vertices.data());
                        read_attribute(position_accessor, 3, base, STRIDE, vertex_count);
                        read_attribute(normal_accessor, 3, base + 3, STRIDE, vertex_count);
                        read_attribute(tangent_accessor, 4, base + 6, STRIDE, vertex_count);
                        read_attribute(uv0_accessor, 2, base + 10, STRIDE, vertex_count);
                        read_attribute(uv1_accessor, 2, base + 12, STRIDE, vertex_count);

                        if (color_accessor != nullptr)
                            for (std::size_t i = 0; i < vertex_count; ++i)
                            {
                                float rgba[4] = {1.0f, 1.0f, 1.0f, 1.0f};
                                cgltf_accessor_read_float(color_accessor, i, rgba, 4);
                                for (int c = 0; c < 4; ++c)
                                    vertices[i].color[c] = static_cast<std::uint8_t>(
                                        std::min(std::max(rgba[c], 0.0f), 1.0f) * 255.0f + 0.5f);
                            }

                        // Bake the node's world transform: the render side has no scene
                        // graph, so a primitive's placement has to live in its vertices.
                        for (Geometry::MeshVertex& vertex : vertices)
                        {
                            const float x = vertex.position[0];
                            const float y = vertex.position[1];
                            const float z = vertex.position[2];
                            vertex.position[0] = m[0] * x + m[4] * y + m[8] * z + m[12];
                            vertex.position[1] = m[1] * x + m[5] * y + m[9] * z + m[13];
                            vertex.position[2] = m[2] * x + m[6] * y + m[10] * z + m[14];

                            const float nx = vertex.normal[0];
                            const float ny = vertex.normal[1];
                            const float nz = vertex.normal[2];
                            vertex.normal[0] = m[0] * nx + m[4] * ny + m[8] * nz;
                            vertex.normal[1] = m[1] * nx + m[5] * ny + m[9] * nz;
                            vertex.normal[2] = m[2] * nx + m[6] * ny + m[10] * nz;

                            const float tx = vertex.tangent[0];
                            const float ty = vertex.tangent[1];
                            const float tz = vertex.tangent[2];
                            vertex.tangent[0] = m[0] * tx + m[4] * ty + m[8] * tz;
                            vertex.tangent[1] = m[1] * tx + m[5] * ty + m[9] * tz;
                            vertex.tangent[2] = m[2] * tx + m[6] * ty + m[10] * tz;
                        }

                        const std::size_t index_count = primitive.indices->count;
                        std::vector<std::uint32_t> indices(index_count);
                        for (std::size_t i = 0; i < index_count; ++i)
                            indices[i] = static_cast<std::uint32_t>(
                                cgltf_accessor_read_index(primitive.indices, i));

                        if (tangent_accessor == nullptr)
                            Geometry::generate_tangents(vertices.data(), vertices.size(),
                                                        indices.data(), indices.size());

                        const MeshId mesh = meshes.add_mesh(vertices.data(), vertices.size(),
                                                            indices.data(), indices.size());
                        if (mesh == INVALID_MESH)
                            continue;

                        out_meshes[written] =
                            mesh;
                        out_materials[written] =
                            convert_material(primitive.material, textures, directory);
                        ++written;
                    }
                }

                cgltf_free(data);
                return written;
            }
        } // namespace Assets
    } // namespace Render
} // namespace SushiEngine
