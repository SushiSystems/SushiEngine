/**************************************************************************/
/* gltf_animation_importer.cpp                                           */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/* Licensed under the Apache License, Version 2.0 (the "License");        */
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

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include <cgltf.h>

#include <SushiEngine/animation/clip_blob.hpp>
#include <SushiEngine/animation/skeleton_blob.hpp>
#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Animation
    {
        namespace
        {
            int joint_index_of(const cgltf_skin& skin, const cgltf_node* node) noexcept
            {
                if (node == nullptr)
                    return -1;
                for (cgltf_size i = 0; i < skin.joints_count; ++i)
                    if (skin.joints[i] == node)
                        return static_cast<int>(i);
                return -1;
            }

            // The bind-pose local TRS of a joint node (decomposed from its local matrix, the
            // same source the skeleton cook uses, so the two agree).
            void node_local_trs(const cgltf_node* node, Vector3f& t, Quaternionf& r, Vector3f& s)
            {
                float local[16];
                cgltf_node_transform_local(const_cast<cgltf_node*>(node), local);
                Mat4 matrix{};
                for (int i = 0; i < 16; ++i)
                    matrix.m[i] = static_cast<Scalar>(local[i]);
                Vector3 dt{};
                Quaternion dr{};
                Vector3 ds{};
                decompose_transform(matrix, dt, dr, ds);
                t = Vector3f{static_cast<float>(dt.x), static_cast<float>(dt.y),
                             static_cast<float>(dt.z)};
                r = Quaternionf{static_cast<float>(dr.x), static_cast<float>(dr.y),
                                static_cast<float>(dr.z), static_cast<float>(dr.w)};
                s = Vector3f{static_cast<float>(ds.x), static_cast<float>(ds.y),
                             static_cast<float>(ds.z)};
            }

            float key_time(const cgltf_accessor* input, cgltf_size key)
            {
                float t = 0.0f;
                cgltf_accessor_read_float(input, key, &t, 1);
                return t;
            }

            // Reads a sampler's value at a key (the middle component for cubic-spline output).
            void read_value(const cgltf_animation_sampler* sampler, cgltf_size key, float* out,
                            cgltf_size components)
            {
                const cgltf_size index =
                    sampler->interpolation == cgltf_interpolation_type_cubic_spline ? key * 3 + 1 : key;
                cgltf_accessor_read_float(sampler->output, index, out, components);
            }

            // Locates the key segment [i, i+1] bracketing time, and the in-segment fraction.
            // Returns false (clamped to an end) when time is outside the key range.
            bool bracket(const cgltf_animation_sampler* sampler, float time, cgltf_size& i0,
                         cgltf_size& i1, float& fraction)
            {
                const cgltf_accessor* input = sampler->input;
                const cgltf_size count = input->count;
                if (count == 0)
                {
                    i0 = i1 = 0;
                    fraction = 0.0f;
                    return false;
                }
                if (time <= key_time(input, 0))
                {
                    i0 = i1 = 0;
                    fraction = 0.0f;
                    return false;
                }
                if (time >= key_time(input, count - 1))
                {
                    i0 = i1 = count - 1;
                    fraction = 0.0f;
                    return false;
                }
                for (cgltf_size i = 0; i + 1 < count; ++i)
                {
                    const float ta = key_time(input, i);
                    const float tb = key_time(input, i + 1);
                    if (time >= ta && time < tb)
                    {
                        i0 = i;
                        i1 = i + 1;
                        fraction = tb > ta ? (time - ta) / (tb - ta) : 0.0f;
                        return true;
                    }
                }
                i0 = i1 = count - 1;
                fraction = 0.0f;
                return false;
            }

            Vector3f sample_vec3(const cgltf_animation_sampler* sampler, float time,
                                 const Vector3f& fallback)
            {
                if (sampler == nullptr)
                    return fallback;
                cgltf_size i0, i1;
                float u;
                const bool interior = bracket(sampler, time, i0, i1, u);
                float a[3];
                read_value(sampler, i0, a, 3);
                if (!interior || sampler->interpolation == cgltf_interpolation_type_step)
                    return Vector3f{a[0], a[1], a[2]};
                float b[3];
                read_value(sampler, i1, b, 3);
                return Vector3f{a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u,
                                a[2] + (b[2] - a[2]) * u};
            }

            Quaternionf sample_quat(const cgltf_animation_sampler* sampler, float time,
                                    const Quaternionf& fallback)
            {
                if (sampler == nullptr)
                    return fallback;
                cgltf_size i0, i1;
                float u;
                const bool interior = bracket(sampler, time, i0, i1, u);
                float a[4];
                read_value(sampler, i0, a, 4);
                const Quaternionf qa{a[0], a[1], a[2], a[3]};
                if (!interior || sampler->interpolation == cgltf_interpolation_type_step)
                    return qa;
                float b[4];
                read_value(sampler, i1, b, 4);
                return nlerp(qa, Quaternionf{b[0], b[1], b[2], b[3]}, u);
            }
        } // namespace

        bool import_gltf_animated(const char* path, GltfAnimationImport& out, float sample_rate,
                                  std::size_t skin_index)
        {
            out.skeleton_blob.clear();
            out.clips.clear();
            if (path == nullptr || sample_rate <= 0.0f)
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
            if (skin_index >= data->skins_count || data->skins[skin_index].joints_count == 0)
            {
                cgltf_free(data);
                return false;
            }

            const cgltf_skin& skin = data->skins[skin_index];
            const cgltf_size joint_count = skin.joints_count;

            // Cook the skeleton and keep the sort order so clips resample in the blob's order.
            SkeletonDesc skeleton_desc;
            skeleton_desc.joints.resize(joint_count);
            skeleton_desc.has_inverse_bind = skin.inverse_bind_matrices != nullptr;
            std::vector<Vector3f> bind_t(joint_count), bind_s(joint_count);
            std::vector<Quaternionf> bind_r(joint_count);
            for (cgltf_size j = 0; j < joint_count; ++j)
            {
                const cgltf_node* node = skin.joints[j];
                JointDesc& joint = skeleton_desc.joints[j];
                joint.name = node->name != nullptr ? std::string(node->name)
                                                   : "joint_" + std::to_string(j);
                joint.parent = joint_index_of(skin, node->parent);
                node_local_trs(node, bind_t[j], bind_r[j], bind_s[j]);
                joint.bind_translation = bind_t[j];
                joint.bind_rotation = bind_r[j];
                joint.bind_scale = bind_s[j];
                if (skeleton_desc.has_inverse_bind)
                {
                    float ibm[16];
                    if (cgltf_accessor_read_float(skin.inverse_bind_matrices, j, ibm, 16))
                        for (int i = 0; i < 16; ++i)
                            joint.inverse_bind.m[i] = ibm[i];
                }
            }

            std::vector<int> order;
            if (!build_skeleton_blob(skeleton_desc, out.skeleton_blob, &order))
            {
                cgltf_free(data);
                return false;
            }

            for (cgltf_size a = 0; a < data->animations_count; ++a)
            {
                const cgltf_animation& animation = data->animations[a];

                // Map each joint (by its glTF joint index) to its translation/rotation/scale
                // sampler, and find the animation's duration from the sampler key ranges.
                std::vector<const cgltf_animation_sampler*> track_t(joint_count, nullptr);
                std::vector<const cgltf_animation_sampler*> track_r(joint_count, nullptr);
                std::vector<const cgltf_animation_sampler*> track_s(joint_count, nullptr);
                float duration = 0.0f;
                for (cgltf_size c = 0; c < animation.channels_count; ++c)
                {
                    const cgltf_animation_channel& channel = animation.channels[c];
                    const int j = joint_index_of(skin, channel.target_node);
                    if (j < 0 || channel.sampler == nullptr)
                        continue;
                    switch (channel.target_path)
                    {
                        case cgltf_animation_path_type_translation:
                            track_t[static_cast<cgltf_size>(j)] = channel.sampler;
                            break;
                        case cgltf_animation_path_type_rotation:
                            track_r[static_cast<cgltf_size>(j)] = channel.sampler;
                            break;
                        case cgltf_animation_path_type_scale:
                            track_s[static_cast<cgltf_size>(j)] = channel.sampler;
                            break;
                        default:
                            break;
                    }
                    const cgltf_accessor* input = channel.sampler->input;
                    if (input != nullptr && input->count > 0)
                    {
                        const float last = key_time(input, input->count - 1);
                        if (last > duration)
                            duration = last;
                    }
                }

                ClipDesc clip;
                clip.joint_count = static_cast<std::uint32_t>(joint_count);
                clip.sample_rate = sample_rate;
                clip.frame_count = duration > 0.0f
                                       ? static_cast<std::uint32_t>(
                                             std::floor(duration * sample_rate + 0.5f)) + 1
                                       : 1;
                const std::size_t element_count =
                    static_cast<std::size_t>(clip.frame_count) * joint_count;
                clip.translations.resize(element_count);
                clip.rotations.resize(element_count);
                clip.scales.resize(element_count);

                for (std::uint32_t f = 0; f < clip.frame_count; ++f)
                {
                    const float time = static_cast<float>(f) / sample_rate;
                    for (cgltf_size nj = 0; nj < joint_count; ++nj)
                    {
                        const cgltf_size original = static_cast<cgltf_size>(order[nj]);
                        const std::size_t out_index = static_cast<std::size_t>(f) * joint_count + nj;
                        clip.translations[out_index] =
                            sample_vec3(track_t[original], time, bind_t[original]);
                        clip.rotations[out_index] =
                            sample_quat(track_r[original], time, bind_r[original]);
                        clip.scales[out_index] =
                            sample_vec3(track_s[original], time, bind_s[original]);
                    }
                }

                GltfClip cooked;
                cooked.name = animation.name != nullptr ? std::string(animation.name)
                                                        : "clip_" + std::to_string(a);
                if (build_clip_blob(clip, cooked.blob))
                    out.clips.push_back(std::move(cooked));
            }

            cgltf_free(data);
            return true;
        }
    } // namespace Animation
} // namespace SushiEngine
