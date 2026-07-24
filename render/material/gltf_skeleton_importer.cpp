/**************************************************************************/
/* gltf_skeleton_importer.cpp                                            */
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

#include <SushiEngine/animation/gltf_skeleton_import.hpp>

#include <cstddef>
#include <string>
#include <vector>

#include <cgltf.h>

#include <SushiEngine/animation/skeleton_blob.hpp>
#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Animation
    {
        namespace
        {
            /**
             * @brief The index of a node within a skin's joint array, or -1 if absent.
             * @param skin The skin whose joints are searched.
             * @param node The node to locate (may be nullptr).
             * @return The joint index, or -1 if @p node is null or not a joint of @p skin.
             */
            int joint_index_of(const cgltf_skin& skin, const cgltf_node* node) noexcept
            {
                if (node == nullptr)
                    return -1;
                for (cgltf_size i = 0; i < skin.joints_count; ++i)
                    if (skin.joints[i] == node)
                        return static_cast<int>(i);
                return -1;
            }
        } // namespace

        bool import_gltf_skeleton(const char* path, std::vector<std::byte>& out_blob,
                                  std::size_t skin_index)
        {
            out_blob.clear();
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
            if (skin_index >= data->skins_count)
            {
                cgltf_free(data);
                return false;
            }

            const cgltf_skin& skin = data->skins[skin_index];
            if (skin.joints_count == 0)
            {
                cgltf_free(data);
                return false;
            }

            SkeletonDesc desc;
            desc.joints.resize(skin.joints_count);
            desc.has_inverse_bind = skin.inverse_bind_matrices != nullptr;

            for (cgltf_size j = 0; j < skin.joints_count; ++j)
            {
                const cgltf_node* node = skin.joints[j];
                JointDesc& joint = desc.joints[j];
                joint.name = node->name != nullptr ? std::string(node->name)
                                                   : "joint_" + std::to_string(j);
                joint.parent = joint_index_of(skin, node->parent);

                // The node's local transform is the bind-pose local TRS. Decomposing the
                // local matrix handles matrix-form and TRS-form nodes the same way.
                float local[16];
                cgltf_node_transform_local(node, local);
                Mat4 local_matrix{};
                for (int i = 0; i < 16; ++i)
                    local_matrix.m[i] = static_cast<Scalar>(local[i]);
                Vector3 translation{};
                Quaternion rotation{};
                Vector3 scale{};
                decompose_transform(local_matrix, translation, rotation, scale);
                joint.bind_translation =
                    Vector3f{static_cast<float>(translation.x), static_cast<float>(translation.y),
                             static_cast<float>(translation.z)};
                joint.bind_rotation =
                    Quaternionf{static_cast<float>(rotation.x), static_cast<float>(rotation.y),
                                static_cast<float>(rotation.z), static_cast<float>(rotation.w)};
                joint.bind_scale = Vector3f{static_cast<float>(scale.x), static_cast<float>(scale.y),
                                            static_cast<float>(scale.z)};

                // glTF stores inverse-bind matrices column-major, the same layout as
                // JointMatrix, so a well-formed read is a straight copy.
                if (desc.has_inverse_bind)
                {
                    float ibm[16];
                    if (cgltf_accessor_read_float(skin.inverse_bind_matrices, j, ibm, 16))
                        for (int i = 0; i < 16; ++i)
                            joint.inverse_bind.m[i] = ibm[i];
                }
            }

            cgltf_free(data);
            return build_skeleton_blob(desc, out_blob);
        }
    } // namespace Animation
} // namespace SushiEngine
