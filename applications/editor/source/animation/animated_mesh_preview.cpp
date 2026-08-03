/**************************************************************************/
/* animated_mesh_preview.cpp                                             */
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

#include "animated_mesh_preview.hpp"

#include <algorithm>
#include <utility>

#include <SushiEngine/animation/animator_step.hpp>
#include <SushiEngine/animation/morph.hpp>

namespace SushiEngine
{
    namespace Editor
    {
        bool AnimatedMeshPreview::load_gltf(const char* path, Render::IAssetLibrary& assets,
                                            std::size_t skin_index)
        {
            clear();
            if (path == nullptr)
                return false;

            Animation::GLTFAnimationImport import;
            if (!Animation::import_gltf_animated(path, import, 30.0f, skin_index) ||
                import.clips.empty())
                return false;

            skeleton_id_ = database_.add_skeleton(import.skeleton_blob);
            if (skeleton_id_ == Animation::INVALID_ASSET)
                return false;
            clip_id_ = database_.add_clip(import.clips.front().blob);
            if (clip_id_ == Animation::INVALID_ASSET)
            {
                clear();
                return false;
            }

            skeleton_ = database_.skeleton(skeleton_id_);
            clip_ = database_.clip(clip_id_);
            available_clips_ = std::move(import.clips);

            controller_desc_ = Animation::ControllerDesc{};
            Animation::LayerDesc base_layer;
            base_layer.name = "Base";
            base_layer.weight = 1.0f;
            Animation::StateDesc base_state;
            base_state.name = "State0";
            base_state.clip = clip_id_;
            base_layer.states.push_back(base_state);
            controller_desc_.layers.push_back(base_layer);
            if (!compile_controller())
            {
                clear();
                return false;
            }

            Render::MeshId meshes[1] = {Render::INVALID_MESH};
            Render::Material materials[1]{};
            const std::size_t written =
                assets.load_gltf_skinned_mesh(path, skin_index, meshes, materials, 1);
            if (written == 0 || meshes[0] == Render::INVALID_MESH)
            {
                clear();
                return false;
            }
            mesh_ = meshes[0];
            material_ = materials[0];
            source_path_ = path;

            // The mesh fixes how many targets there are; the import names them, in the same
            // order. A name the file omitted falls back to morph_<index> — the identical
            // fallback the importer's own track names use, so an unnamed target still resolves.
            morph_weights_.assign(assets.morph_target_count(mesh_), 0.0f);
            morph_target_names_.resize(morph_weights_.size());
            morph_target_hashes_.resize(morph_weights_.size());
            for (std::size_t i = 0; i < morph_target_names_.size(); ++i)
            {
                morph_target_names_[i] = i < import.morph_target_names.size()
                                             ? import.morph_target_names[i]
                                             : "morph_" + std::to_string(i);
                morph_target_hashes_[i] = Animation::hash_name(morph_target_names_[i].c_str());
            }
            return true;
        }

        const std::string& AnimatedMeshPreview::morph_target_name(std::uint32_t index) const noexcept
        {
            static const std::string EMPTY;
            return index < morph_target_names_.size() ? morph_target_names_[index] : EMPTY;
        }

        bool AnimatedMeshPreview::add_layer(const char* clip_name,
                                            const Animation::MaskDesc* mask, float weight,
                                            bool additive)
        {
            if (clip_name == nullptr || !loaded())
                return false;

            const Animation::GLTFClip* found = nullptr;
            for (const Animation::GLTFClip& candidate : available_clips_)
                if (candidate.name == clip_name)
                {
                    found = &candidate;
                    break;
                }
            if (found == nullptr)
                return false;

            const Animation::AssetId layer_clip_id = database_.add_clip(found->blob);
            if (layer_clip_id == Animation::INVALID_ASSET)
                return false;

            Animation::AssetId mask_id = Animation::INVALID_ASSET;
            if (mask != nullptr)
            {
                std::vector<std::byte> mask_blob;
                if (!Animation::build_mask_blob(*mask, mask_blob))
                    return false;
                mask_id = database_.add_mask(std::move(mask_blob));
                if (mask_id == Animation::INVALID_ASSET)
                    return false;
            }

            const std::size_t new_layer_index = controller_desc_.layers.size();
            const std::string weight_param_name =
                "layer_weight_" + std::to_string(new_layer_index);

            // Wire the layer's weight through a compiled parameter (design §5.1) — the same
            // seam Mecanim's own layer-weight sliders use — so set_layer_weight is a single
            // float write afterward, never a recompile.
            Animation::ParameterDesc param;
            param.name = weight_param_name;
            param.type = Animation::ParameterType::Float;
            param.default_value = weight;
            controller_desc_.parameters.push_back(param);

            Animation::LayerDesc layer;
            layer.name = "Layer" + std::to_string(new_layer_index);
            layer.weight = weight;
            layer.mask = mask_id;
            layer.blend_mode = additive ? Animation::LayerBlendMode::Additive
                                        : Animation::LayerBlendMode::Override;
            layer.weight_parameter = weight_param_name;
            Animation::StateDesc state;
            state.name = "State0";
            state.clip = layer_clip_id;
            layer.states.push_back(state);
            controller_desc_.layers.push_back(layer);

            if (!compile_controller())
            {
                controller_desc_.layers.pop_back();
                controller_desc_.parameters.pop_back();
                compile_controller(); // best-effort restore of the previous, known-good controller
                return false;
            }
            animator_instance_.parameters.set_float(layer_weight_param_index_[new_layer_index],
                                                     weight);
            return true;
        }

        bool AnimatedMeshPreview::remove_layer(std::size_t index)
        {
            if (index == 0 || index >= controller_desc_.layers.size())
                return false;

            // Copy before erase: the layer's own weight_parameter name is still needed after
            // it leaves controller_desc_.layers, to drop the matching ParameterDesc too.
            const Animation::LayerDesc removed = controller_desc_.layers[index];
            controller_desc_.layers.erase(controller_desc_.layers.begin() +
                                          static_cast<std::ptrdiff_t>(index));
            if (!removed.weight_parameter.empty())
                for (std::size_t p = 0; p < controller_desc_.parameters.size(); ++p)
                    if (controller_desc_.parameters[p].name == removed.weight_parameter)
                    {
                        controller_desc_.parameters.erase(controller_desc_.parameters.begin() +
                                                          static_cast<std::ptrdiff_t>(p));
                        break;
                    }

            return compile_controller();
        }

        float AnimatedMeshPreview::layer_weight(std::size_t index) const noexcept
        {
            if (index >= controller_desc_.layers.size())
                return 0.0f;
            if (index == 0)
                return 1.0f;
            if (index >= layer_weight_param_index_.size())
                return controller_desc_.layers[index].weight;
            return animator_instance_.parameters.values[layer_weight_param_index_[index]].as_float;
        }

        void AnimatedMeshPreview::set_layer_weight(std::size_t index, float weight) noexcept
        {
            if (index == 0 || index >= layer_weight_param_index_.size())
                return;
            animator_instance_.parameters.set_float(layer_weight_param_index_[index], weight);
        }

        void AnimatedMeshPreview::rebuild_weight_param_index_cache()
        {
            layer_weight_param_index_.assign(controller_desc_.layers.size(), 0u);
            for (std::size_t i = 1; i < controller_desc_.layers.size(); ++i)
            {
                const std::string& name = controller_desc_.layers[i].weight_parameter;
                if (name.empty())
                    continue;
                for (std::size_t p = 0; p < controller_desc_.parameters.size(); ++p)
                    if (controller_desc_.parameters[p].name == name)
                    {
                        layer_weight_param_index_[i] = static_cast<std::uint32_t>(p);
                        break;
                    }
            }
        }

        bool AnimatedMeshPreview::apply_controller(const Animation::ControllerDesc& desc)
        {
            if (!loaded())
                return false;
            const Animation::ControllerDesc previous = controller_desc_;
            controller_desc_ = desc;
            if (!compile_controller())
            {
                // Restore the known-good controller so a bad graph reports as a message,
                // never as a character that stops animating.
                controller_desc_ = previous;
                compile_controller();
                return false;
            }
            return true;
        }

        bool AnimatedMeshPreview::compile_controller()
        {
            std::vector<std::byte> blob;
            if (!Animation::compile_controller_blob(controller_desc_, blob))
                return false;
            controller_id_ = database_.add_controller(std::move(blob));
            if (controller_id_ == Animation::INVALID_ASSET)
                return false;
            controller_ = database_.controller(controller_id_);
            animator_instance_.controller = controller_id_;
            animator_instance_.skeleton = skeleton_id_;
            // Re-seed every layer's state next update() — safe even mid-playback: animator_step
            // reinitializes current_state/normalized_time from the compiled defaults, and a
            // freshly added layer has no prior state to preserve anyway.
            animator_instance_.initialized = 0;
            rebuild_weight_param_index_cache();
            return true;
        }

        void AnimatedMeshPreview::clear()
        {
            database_ = Animation::AnimationDatabase();
            skeleton_id_ = Animation::INVALID_ASSET;
            clip_id_ = Animation::INVALID_ASSET;
            skeleton_ = Animation::SkeletonView{};
            clip_ = Animation::ClipView{};
            controller_desc_ = Animation::ControllerDesc{};
            controller_id_ = Animation::INVALID_ASSET;
            controller_ = Animation::ControllerView{};
            animator_instance_ = Animation::AnimatorInstance{};
            // TwoBoneIk's own default weight is 1.0 (meant for a caller who immediately sets
            // real joint indices/target); this preview must start with IK off until
            // set_two_bone_ik is called explicitly, or upper==mid==tip==0 (all joint 0) would
            // solve a degenerate zero-length chain from the very first frame.
            two_bone_ik_ = Animation::TwoBoneIk{};
            two_bone_ik_.weight = 0.0f;
            available_clips_.clear();
            layer_weight_param_index_.clear();
            mesh_ = Render::INVALID_MESH;
            material_ = Render::Material{};
            previous_palette_.clear();
            pose_scratch_.clear();
            instances_.clear();
            source_path_.clear();
            morph_weights_.clear();
            morph_target_names_.clear();
            morph_target_hashes_.clear();
        }

        void AnimatedMeshPreview::update(float dt)
        {
            instances_.clear();
            if (!loaded() || mesh_ == Render::INVALID_MESH)
                return;

            Animation::animator_step(controller_, database_, animator_instance_,
                                     playing_ ? dt : 0.0f);

            const bool have_previous = previous_palette_.size() == skeleton_.joint_count;

            const Animation::IPoseModifier* modifiers[1] = {&two_bone_ik_};
            const std::size_t modifier_count = two_bone_ik_.weight > 0.0f ? 1u : 0u;
            evaluator_.evaluate(controller_, database_, animator_instance_, skeleton_, modifiers,
                                modifier_count);

            // Morph weights come from the base layer's clip, which is what the mesh's blend
            // shapes belong to; layer blending applies to the pose, not to weight tracks. The
            // base state loops, so its normalized time maps straight onto the clip's duration.
            if (clip_driven_morphs_ && clip_.morph_track_count > 0 && !morph_target_hashes_.empty())
            {
                Animation::MorphState morph;
                Animation::sample_morph_state(
                    clip_, animator_instance_.layers[0].normalized_time * clip_.duration, true,
                    morph_target_hashes_.data(),
                    static_cast<std::uint32_t>(morph_target_hashes_.size()), morph);
                const std::size_t count =
                    std::min(morph_weights_.size(), static_cast<std::size_t>(morph.count));
                for (std::size_t i = 0; i < count; ++i)
                    morph_weights_[i] = morph.weights[i];
            }

            Render::SkinnedInstance instance;
            instance.model = world_;
            instance.palette = evaluator_.palette().data();
            // previous_palette_ still holds last frame's pose here — untouched this frame.
            instance.previous_palette = have_previous ? previous_palette_.data() : nullptr;
            instance.joint_count = skeleton_.joint_count;
            instance.id = 0;
            instance.mesh = mesh_;
            instance.material = material_;
            if (!morph_weights_.empty())
            {
                instance.morph_weights = morph_weights_.data();
                instance.morph_target_count = static_cast<std::uint32_t>(morph_weights_.size());
            }
            instance.use_dual_quaternion_skinning = use_dual_quaternion_skinning_;
            instances_.push_back(instance);

            // Ping-pong into the *other* buffer so the one `instance.previous_palette` just
            // captured is not mutated before the caller (the viewport's synchronous extract
            // this same frame) reads it. previous_palette_ becomes this frame's pose for next
            // update()'s "previous"; the old previous_palette_ buffer (now pose_scratch_) is
            // free to overwrite starting next frame, once this frame's instance is long consumed.
            pose_scratch_ = evaluator_.palette();
            std::swap(previous_palette_, pose_scratch_);
        }

        AnimatedMeshPreview::Statistics AnimatedMeshPreview::statistics() const noexcept
        {
            Statistics stats;
            stats.loaded = loaded();
            if (!stats.loaded)
                return stats;

            stats.source_path = source_path_;
            stats.joint_count = skeleton_.joint_count;
            stats.layer_count = controller_.layer_count;
            stats.palette_bytes =
                static_cast<std::size_t>(skeleton_.joint_count) * sizeof(Animation::JointMatrix);
            stats.active_morph_weights = static_cast<std::uint32_t>(morph_weights_.size());
            stats.clip_morph_track_count = clip_.morph_track_count;
            stats.clip_driven_morphs = clip_driven_morphs_;
            stats.dual_quaternion_skinning = use_dual_quaternion_skinning_;
            stats.clip_compressed = clip_.format == Animation::ClipFormat::Compressed;
            stats.clip_frame_count = clip_.frame_count;
            stats.clip_sample_rate = clip_.sample_rate;
            // Raw per-frame, per-joint track cost were this clip stored uncompressed: a
            // Vector3f translation + Quaternionf rotation + Vector3f scale (12+16+12 bytes).
            // The base layer's clip always imports Raw (import_gltf_animated ->
            // build_clip_blob, no ACL-shaped compression — that's a separate, benchmark-only
            // path today, design §12.1), so this is simply the clip's actual size.
            constexpr std::size_t TRACK_BYTES_PER_JOINT_PER_FRAME =
                sizeof(Animation::Vector3f) * 2 + sizeof(Animation::Quaternionf);
            stats.clip_raw_track_bytes = static_cast<std::size_t>(clip_.frame_count) *
                                         clip_.joint_count * TRACK_BYTES_PER_JOINT_PER_FRAME;
            stats.ik_active = two_bone_ik_.weight > 0.0f;
            return stats;
        }
    } // namespace Editor
} // namespace SushiEngine
