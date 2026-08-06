/**************************************************************************/
/* scene_importer.cpp                                                     */
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

#include <SushiEngine/gltf/scene_import.hpp>

#include <cstdint>
#include <utility>

#include <SushiEngine/core/types.hpp>

#include <cgltf.h>

namespace SushiEngine
{
    namespace Geometry
    {
        namespace
        {
            GLTFLightKind light_kind(cgltf_light_type type)
            {
                switch (type)
                {
                    case cgltf_light_type_directional:
                        return GLTFLightKind::Directional;
                    case cgltf_light_type_spot:
                        return GLTFLightKind::Spot;
                    default:
                        return GLTFLightKind::Point;
                }
            }

            // glTF states a node's placement either as translation, rotation and scale or as a
            // single matrix, and cgltf reports back whichever form the file used. The matrix
            // form is decomposed here so a consumer only ever sees one representation; a matrix
            // node left undecomposed arrives with an identity rotation and a unit scale, which
            // is a silently wrong pose rather than a visible failure.
            void read_transform(const cgltf_node& node, GLTFNodeDescription& out)
            {
                if (node.has_matrix)
                {
                    cgltf_float local[16];
                    cgltf_node_transform_local(&node, local);

                    Matrix4 matrix;
                    for (int element = 0; element < 16; ++element)
                        matrix.m[element] = Scalar(local[element]);

                    Vector3 translation;
                    Quaternion rotation;
                    Vector3 scale;
                    decompose_transform(matrix, translation, rotation, scale);

                    out.translation = Vector3f{float(translation.x), float(translation.y),
                                               float(translation.z)};
                    out.rotation = Quaternionf{float(rotation.x), float(rotation.y),
                                               float(rotation.z), float(rotation.w)};
                    out.scale = Vector3f{float(scale.x), float(scale.y), float(scale.z)};
                    return;
                }

                // cgltf initialises these three to an identity transform whether or not the
                // file states them, so no `has_` flag has to be consulted.
                out.translation =
                    Vector3f{node.translation[0], node.translation[1], node.translation[2]};
                out.rotation = Quaternionf{node.rotation[0], node.rotation[1], node.rotation[2],
                                           node.rotation[3]};
                out.scale = Vector3f{node.scale[0], node.scale[1], node.scale[2]};
            }

            // Appends `node` and then its children, so a parent is always written before any of
            // its descendants and a consumer can build the tree in one forward pass.
            void append_node(const cgltf_data& data, const cgltf_node& node, std::int32_t parent,
                             GLTFSceneDescription& out)
            {
                GLTFNodeDescription description;
                description.parent = parent;
                description.source_index = std::uint32_t(cgltf_node_index(&data, &node));
                if (node.name != nullptr)
                    description.name = node.name;

                read_transform(node, description);

                if (node.mesh != nullptr)
                {
                    description.mesh = std::int32_t(cgltf_mesh_index(&data, node.mesh));
                    description.primitive_count = std::uint32_t(node.mesh->primitives_count);
                }
                if (node.camera != nullptr)
                    description.camera = std::int32_t(cgltf_camera_index(&data, node.camera));
                if (node.light != nullptr)
                    description.light = std::int32_t(cgltf_light_index(&data, node.light));
                if (node.skin != nullptr)
                    description.skin = std::int32_t(cgltf_skin_index(&data, node.skin));

                const std::int32_t self = std::int32_t(out.nodes.size());
                out.nodes.push_back(std::move(description));
                for (cgltf_size c = 0; c < node.children_count; ++c)
                    append_node(data, *node.children[c], self, out);
            }
        } // namespace

        bool import_gltf_scene(const char* path, GLTFSceneDescription& out)
        {
            out = GLTFSceneDescription{};
            if (path == nullptr)
                return false;

            cgltf_options options{};
            cgltf_data* data = nullptr;
            if (cgltf_parse_file(&options, path, &data) != cgltf_result_success)
                return false;

            out.material_count = std::uint32_t(data->materials_count);
            out.skin_count = std::uint32_t(data->skins_count);

            out.lights.reserve(data->lights_count);
            for (cgltf_size l = 0; l < data->lights_count; ++l)
            {
                const cgltf_light& source = data->lights[l];
                GLTFLightDescription light;
                if (source.name != nullptr)
                    light.name = source.name;
                light.kind = light_kind(source.type);
                light.color[0] = source.color[0];
                light.color[1] = source.color[1];
                light.color[2] = source.color[2];
                light.intensity = source.intensity;
                light.range = source.range;
                light.spot_inner_cone_radians = source.spot_inner_cone_angle;
                light.spot_outer_cone_radians = source.spot_outer_cone_angle;
                out.lights.push_back(std::move(light));
            }

            out.cameras.reserve(data->cameras_count);
            for (cgltf_size c = 0; c < data->cameras_count; ++c)
            {
                const cgltf_camera& source = data->cameras[c];
                GLTFCameraDescription camera;
                if (source.name != nullptr)
                    camera.name = source.name;
                if (source.type == cgltf_camera_type_orthographic)
                {
                    camera.kind = GLTFCameraKind::Orthographic;
                    camera.orthographic_width = source.data.orthographic.xmag;
                    camera.orthographic_height = source.data.orthographic.ymag;
                    camera.near_plane = source.data.orthographic.znear;
                    camera.far_plane = source.data.orthographic.zfar;
                }
                else
                {
                    camera.kind = GLTFCameraKind::Perspective;
                    camera.vertical_field_of_view_radians = source.data.perspective.yfov;
                    camera.near_plane = source.data.perspective.znear;
                    camera.far_plane =
                        source.data.perspective.has_zfar ? source.data.perspective.zfar : 0.0f;
                }
                out.cameras.push_back(std::move(camera));
            }

            // The default scene when the file names one, every root otherwise: a file with no
            // `scene` member is legal and still has nodes worth importing.
            const cgltf_scene* scene =
                data->scene != nullptr ? data->scene
                                       : (data->scenes_count > 0 ? &data->scenes[0] : nullptr);
            if (scene != nullptr)
            {
                for (cgltf_size n = 0; n < scene->nodes_count; ++n)
                    append_node(*data, *scene->nodes[n], -1, out);
            }
            else
            {
                for (cgltf_size n = 0; n < data->nodes_count; ++n)
                {
                    if (data->nodes[n].parent == nullptr)
                        append_node(*data, data->nodes[n], -1, out);
                }
            }

            cgltf_free(data);
            return !out.nodes.empty();
        }
    } // namespace Geometry
} // namespace SushiEngine
