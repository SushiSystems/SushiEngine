/**************************************************************************/
/* animated_mesh_preview.hpp                                             */
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

#ifndef SUSHIENGINE_EDITOR_ANIMATED_MESH_PREVIEW_HPP
#define SUSHIENGINE_EDITOR_ANIMATED_MESH_PREVIEW_HPP

#include <cstddef>
#include <string>
#include <vector>

#include <SushiEngine/animation/animation_database.hpp>
#include <SushiEngine/animation/animator_components.hpp>
#include <SushiEngine/animation/animator_controller.hpp>
#include <SushiEngine/animation/animator_evaluator.hpp>
#include <SushiEngine/animation/asset_id.hpp>
#include <SushiEngine/animation/avatar_mask.hpp>
#include <SushiEngine/animation/gltf_skeleton_import.hpp>
#include <SushiEngine/animation/ik_two_bone.hpp>
#include <SushiEngine/core/types.hpp>
#include <SushiEngine/material/material.hpp>
#include <SushiEngine/render/asset_library_interface.hpp>
#include <SushiEngine/render/scene_view.hpp>

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief A live, controller-driven, layered/masked/IK-capable skinned character
         * previewed in the Scene viewport.
         *
         * This is the animation-evaluator-to-renderer bridge (design `slop/animation_system.md`
         * §12.1) — the one piece of the skeletal-animation stack that was never wired end to
         * end: every earlier phase shipped as an isolated, unit-tested component, but nothing
         * constructed a `Render::SkinnedInstance` for a live scene, and nothing ran the richer
         * `AnimatorEvaluator` (layers, masks, IK) outside a headless example. @ref load_gltf
         * imports the skin (mesh + skeleton + first clip, sharing one joint order) and compiles
         * a minimal one-layer, one-state `ControllerAsset` around that clip so the *real*
         * Mecanim-parity evaluator drives the pose from the first frame, not a cut-down
         * single-clip shortcut. @ref add_layer grows it (mask-gated override/additive layers,
         * design §5.2); @ref set_two_bone_ik attaches the one shipped IK solver in the
         * pose-modifier stack (§5.3), off by default (zero weight) until given real joint
         * indices and a target. @ref update advances the deterministic `animator_step` tick,
         * runs `AnimatorEvaluator` (host-side; the batched SushiRuntime device path is still
         * open, §12.3), and rebuilds this frame's single-entry `SkinnedInstance`. Mirrors
         * `Vfx::EffectPreview`'s shape: a state-owning class the viewport reads from every frame.
         *
         * Single-instance by design — this closes the missing wire for one live character, it
         * does not add multi-character ECS wiring (§12.4).
         */
        class AnimatedMeshPreview
        {
            public:
                /**
                 * @brief Imports a rigged, animated glTF's skinned mesh and skeleton, and
                 * compiles a one-layer controller around its first clip.
                 *
                 * All three (mesh, skeleton, clip) read the same skin index, so `SkinVertex`
                 * joint indices, the `SkeletonView` they address, and the sampled clip agree on
                 * joint order (the cooked skeleton's topological sort, not the glTF's authored
                 * order). The compiled controller's base layer loops the clip forever — no
                 * transitions, matching the old single-clip preview's behavior, but now through
                 * the real `ControllerAsset`/`AnimatorEvaluator` path so @ref add_layer and
                 * @ref set_two_bone_ik have something to layer onto / correct.
                 *
                 * @param path       Path to a `.gltf` or `.glb` file with a skin and at least
                 *                   one animation.
                 * @param assets     The asset library the skinned mesh is uploaded through.
                 * @param skin_index Which skin to import (0 by default; most rigs have one).
                 * @return True on success; false if the file, its skin, its geometry, or the
                 *         controller compile failed.
                 */
                bool load_gltf(const char* path, Render::IAssetLibrary& assets,
                               std::size_t skin_index = 0);

                /**
                 * @brief Replaces the preview's controller with an externally authored one.
                 *
                 * The Animator Graph panel's bridge: the authored @ref
                 * Animation::ControllerDesc compiles against the loaded character's clip
                 * set and drives this preview instance. On a failed compile (a state
                 * naming a clip the loaded glTF does not carry, an empty graph) the
                 * previous, known-good controller is restored — the preview never goes
                 * dark to show an error the panel can report instead.
                 *
                 * @param desc The authored controller to compile and bind.
                 * @return True if the controller compiled and now drives the preview.
                 */
                bool apply_controller(const Animation::ControllerDesc& desc);

                /** @brief Drops the loaded character, if any, and every added layer. */
                void clear();

                /** @brief Whether a character is loaded and ready to preview. */
                bool loaded() const noexcept { return skeleton_.valid() && controller_.valid(); }

                /** @brief Whether the preview clock is advancing. */
                bool playing() const noexcept { return playing_; }

                /** @brief Starts or stops playback. */
                void set_playing(bool playing) noexcept { playing_ = playing; }

                /** @brief Resets every layer to its default state at time zero. */
                void restart() noexcept { animator_instance_.initialized = 0; }

                /** @brief The character's world placement. */
                const Mat4& world() const noexcept { return world_; }

                /** @brief Sets where in the world the character is drawn. */
                void set_world(const Mat4& world) noexcept { world_ = world; }

                /**
                 * @brief Adds a mask-gated layer over the base, looping another clip from the
                 * same glTF file (design §5.2).
                 *
                 * Recompiles the controller in place; existing layers keep their live state
                 * (@ref layer_weight stays readable/settable across the recompile — see
                 * @ref compile_controller's implementation note), but the new layer's own state
                 * initializes on the next @ref update, same as a freshly loaded character. The
                 * mask is authored in memory (@p mask, built from the loaded skeleton's joint
                 * names — see @ref skeleton) and cooked to a `.sushimask` blob internally; there
                 * is no on-disk file involved, so an editor mask panel can build one from
                 * checkboxes with no save step.
                 *
                 * @param clip_name The glTF animation's name to loop on this layer (matches
                 *                  `GltfClip::name` from the same file @ref load_gltf used).
                 * @param mask      The mask gating which joints this layer writes, or nullptr
                 *                  for an unmasked (all-joints) layer.
                 * @param weight    The layer's initial blend weight in [0, 1] (live-adjustable
                 *                  afterward via @ref set_layer_weight, no recompile).
                 * @param additive  False = override blend, true = additive (design §5.2).
                 * @return True if the clip was found in the source file and the layer compiled.
                 */
                bool add_layer(const char* clip_name, const Animation::MaskDesc* mask,
                               float weight, bool additive);

                /**
                 * @brief Removes a previously added layer (not the base layer, index 0).
                 * @param index A layer index in [1, @ref layer_count).
                 * @return True if removed and the controller recompiled; false if @p index was
                 *         out of range (including 0 — the base layer cannot be removed).
                 */
                bool remove_layer(std::size_t index);

                /** @brief Number of layers, including the base layer (always >= 1 once loaded). */
                std::size_t layer_count() const noexcept { return controller_desc_.layers.size(); }

                /**
                 * @brief A layer's authored name, for UI display.
                 * @param index A layer index in [0, @ref layer_count).
                 */
                const std::string& layer_name(std::size_t index) const noexcept
                {
                    return controller_desc_.layers[index].name;
                }

                /**
                 * @brief Whether a layer blends additively (vs. override).
                 * @param index A layer index in [0, @ref layer_count).
                 */
                bool layer_additive(std::size_t index) const noexcept
                {
                    return controller_desc_.layers[index].blend_mode ==
                           Animation::LayerBlendMode::Additive;
                }

                /**
                 * @brief A layer's current live weight.
                 *
                 * Layer 0 (the base) is always effectively full weight; layers added via
                 * @ref add_layer read back the live parameter @ref set_layer_weight writes, not
                 * the compiled default (they may have drifted since the last recompile).
                 *
                 * @param index A layer index in [0, @ref layer_count).
                 */
                float layer_weight(std::size_t index) const noexcept;

                /**
                 * @brief Sets a layer's blend weight live, with no controller recompile.
                 *
                 * `add_layer` wires every non-base layer's weight through a compiled
                 * `AnimatorParameterBlock` slot (`LayerRecord::weight_parameter`, design §5.1) —
                 * exactly the seam Mecanim's own layer-weight sliders use — so this is a single
                 * float write, safe to call every frame from a slider without touching the
                 * controller blob. A no-op on layer 0 (the base layer has no weight parameter;
                 * it is always full weight, matching `load_gltf`'s single-clip behavior).
                 *
                 * @param index  A layer index in [1, @ref layer_count).
                 * @param weight The new weight, in [0, 1].
                 */
                void set_layer_weight(std::size_t index, float weight) noexcept;

                /** @brief The bound skeleton, for joint enumeration (mask/IK authoring UI). */
                const Animation::SkeletonView& skeleton() const noexcept { return skeleton_; }

                /** @brief The source glTF's clips, for the layer-add clip picker. */
                const std::vector<Animation::GltfClip>& available_clips() const noexcept
                {
                    return available_clips_;
                }

                /**
                 * @brief Configures (or disables) the one shipped IK solver, two-bone IK.
                 *
                 * Runs in the pose-modifier stack after every layer blends (design §5.3), so it
                 * corrects the final animated pose. Off by default (@p weight 0) until a caller
                 * (e.g. the Animator Preview panel's IK section, or a script) sets real joints.
                 * Joint indices are into the loaded skeleton's cooked (topologically sorted)
                 * order — resolve by name via @ref skeleton's `SkeletonView::joint_name`/
                 * `find_joint`.
                 *
                 * @param upper  Shoulder/hip joint index.
                 * @param mid    Elbow/knee joint index.
                 * @param tip    Wrist/ankle (end effector) joint index.
                 * @param target Object-space goal for the tip.
                 * @param pole   Object-space hint the mid joint bends toward.
                 * @param weight Blend of the correction, in [0, 1]; 0 disables the solver.
                 */
                void set_two_bone_ik(std::uint32_t upper, std::uint32_t mid, std::uint32_t tip,
                                     const Vector3& target, const Vector3& pole, float weight)
                {
                    two_bone_ik_.upper = upper;
                    two_bone_ik_.mid = mid;
                    two_bone_ik_.tip = tip;
                    two_bone_ik_.target = target;
                    two_bone_ik_.pole = pole;
                    two_bone_ik_.weight = weight;
                }

                /**
                 * @brief Moves only the IK target, keeping every other field (joints, pole,
                 * weight) as last configured.
                 *
                 * The seam the viewport IK gizmo drags through every frame — cheaper than
                 * round-tripping the full @ref set_two_bone_ik call when only the target moves.
                 * @param target Object-space goal for the tip.
                 */
                void set_ik_target(const Vector3& target) noexcept { two_bone_ik_.target = target; }

                /** @brief The two-bone IK solver's current configuration, for UI display. */
                const Animation::TwoBoneIk& two_bone_ik() const noexcept { return two_bone_ik_; }

                /**
                 * @brief Sets per-target morph (blend-shape) weights by hand.
                 *
                 * The manual seam, for posing a face/prop the loaded clip does not animate. It
                 * is overwritten every frame while @ref clip_driven_morphs is on and the base
                 * clip carries morph tracks — turn that off first to hold a hand-set pose.
                 * Weights beyond the loaded mesh's own target count are ignored by the
                 * SkinningPass.
                 *
                 * @param weights Per-target weights, in the mesh's target order.
                 * @param count   Entries in @p weights.
                 */
                void set_morph_weights(const float* weights, std::uint32_t count)
                {
                    morph_weights_.assign(weights, weights + count);
                }

                /**
                 * @brief Whether the base clip's morph-weight tracks drive the mesh's targets.
                 *
                 * On by default, and a no-op when the loaded clip has no morph tracks (a rig
                 * without blend shapes, or a glTF whose animations carry no `weights` channel),
                 * so a hand-set pose survives in that case.
                 */
                bool clip_driven_morphs() const noexcept { return clip_driven_morphs_; }

                /** @brief Morph-weight tracks the base clip carries; 0 means nothing to drive. */
                std::uint32_t clip_morph_track_count() const noexcept
                {
                    return clip_.morph_track_count;
                }

                /**
                 * @brief Stops or resumes clip-driven morph weights, per @ref clip_driven_morphs.
                 * @param enabled False to hold whatever @ref set_morph_weights /
                 *                @ref set_morph_weight last wrote.
                 */
                void set_clip_driven_morphs(bool enabled) noexcept { clip_driven_morphs_ = enabled; }

                /**
                 * @brief A morph target's name, for a blend-shape slider's label.
                 *
                 * The name the clip's tracks are matched against (design §6.5) — a target the
                 * source file left unnamed reads back as `morph_<index>`, the same positional
                 * fallback the importer names its tracks with.
                 *
                 * @param index A target index in [0, @ref morph_target_count).
                 * @return The name, or an empty string if @p index is out of range.
                 */
                const std::string& morph_target_name(std::uint32_t index) const noexcept;

                /**
                 * @brief Morph targets the loaded mesh carries, per @ref set_morph_weights.
                 *
                 * Sized from @ref load_gltf's `IAssetLibrary::morph_target_count`, so a
                 * blend-shape authoring UI can build one slider per target without a separate
                 * asset query. 0 when nothing is loaded or the mesh has no morph targets.
                 */
                std::uint32_t morph_target_count() const noexcept
                {
                    return static_cast<std::uint32_t>(morph_weights_.size());
                }

                /**
                 * @brief A single target's current weight, for a blend-shape slider to read.
                 * @param index A target index in [0, @ref morph_target_count).
                 * @return The weight, or 0 if @p index is out of range.
                 */
                float morph_weight(std::uint32_t index) const noexcept
                {
                    return index < morph_weights_.size() ? morph_weights_[index] : 0.0f;
                }

                /**
                 * @brief Sets a single target's weight in place, for a blend-shape slider to write.
                 *
                 * Cheaper than round-tripping @ref set_morph_weights' full-array assign when only
                 * one target moved, and safe to call every frame from a slider.
                 * @param index  A target index in [0, @ref morph_target_count).
                 * @param weight The new weight; out-of-range @p index is ignored.
                 */
                void set_morph_weight(std::uint32_t index, float weight) noexcept
                {
                    if (index < morph_weights_.size())
                        morph_weights_[index] = weight;
                }

                /**
                 * @brief Toggles dual-quaternion skinning for this preview's instance (design §12.4).
                 *
                 * Off by default (linear-blend, unchanged behavior). The candy-wrapper fix is most
                 * visible under a large single-axis bend near a joint with few children — a rig
                 * with a sharply bent limb is the case worth toggling this on for.
                 * @param enabled True to blend via dual quaternions instead of linear-blend.
                 */
                void set_dual_quaternion_skinning(bool enabled) noexcept
                {
                    use_dual_quaternion_skinning_ = enabled;
                }

                /** @brief Whether this preview's instance currently uses dual-quaternion skinning. */
                bool dual_quaternion_skinning() const noexcept { return use_dual_quaternion_skinning_; }

                /**
                 * @brief Advances the animator tick and re-evaluates this frame's skinned
                 * instance (layers, masks, IK — the full `AnimatorEvaluator` chain).
                 * @param dt Seconds since the last frame.
                 */
                void update(float dt);

                /** @brief This frame's skinned instance, or nullptr when nothing is loaded. */
                const Render::SkinnedInstance* skinned() const noexcept
                {
                    return instances_.empty() ? nullptr : instances_.data();
                }

                /** @brief Number of entries in @ref skinned (0 or 1). */
                std::size_t skinned_count() const noexcept { return instances_.size(); }

                /**
                 * @brief This preview's live memory/asset footprint, for the Statistics panel
                 * (design §12.1/§0.7 — the accounting `animation_benchmark` already computes
                 * headlessly, exposed here for the one live instance instead).
                 */
                struct Statistics
                {
                    bool loaded = false;
                    std::string source_path;
                    std::uint32_t joint_count = 0;
                    std::uint32_t layer_count = 0;
                    std::size_t palette_bytes = 0;      /**< One frame's palette (current only). */
                    std::uint32_t active_morph_weights = 0;
                    /** @brief Morph-weight tracks the base clip carries (0 = nothing to drive). */
                    std::uint32_t clip_morph_track_count = 0;
                    bool clip_driven_morphs = false; /**< See set_clip_driven_morphs. */
                    bool clip_compressed = false;
                    std::uint32_t clip_frame_count = 0;
                    float clip_sample_rate = 0.0f;
                    /** @brief Estimated uncompressed track bytes (translation+rotation+scale). */
                    std::size_t clip_raw_track_bytes = 0;
                    bool ik_active = false; /**< Whether the two-bone IK solver has nonzero weight. */
                    bool dual_quaternion_skinning = false; /**< See set_dual_quaternion_skinning. */
                };

                /** @brief Snapshots the current load/pose-pool/clip footprint. */
                Statistics statistics() const noexcept;

            private:
                /**
                 * @brief Recompiles @ref controller_desc_, rebinds @ref controller_, and
                 * rebuilds @ref layer_weight_param_index_ from the compiled layers'
                 * `weight_parameter` names.
                 */
                bool compile_controller();

                /** @brief Resolves each layer's weight-parameter name to an index, freshly. */
                void rebuild_weight_param_index_cache();

                Animation::AnimationDatabase database_;
                Animation::AssetId skeleton_id_ = Animation::INVALID_ASSET;
                Animation::AssetId clip_id_ = Animation::INVALID_ASSET; /**< The base layer's clip. */
                Animation::SkeletonView skeleton_{};
                Animation::ClipView clip_{}; /**< The base layer's clip view, for Statistics only. */
                Animation::ControllerDesc controller_desc_; /**< Authored form; recompiled on add_layer. */
                Animation::AssetId controller_id_ = Animation::INVALID_ASSET;
                Animation::ControllerView controller_{};
                Animation::AnimatorInstance animator_instance_;
                Animation::AnimatorEvaluator evaluator_;
                Animation::TwoBoneIk two_bone_ik_; /**< weight defaults to 0 (off); see set_two_bone_ik. */
                std::string source_path_; /**< The loaded glTF's path, kept for Statistics. */
                std::vector<Animation::GltfClip> available_clips_; /**< From load_gltf, for add_layer. */
                // Parallel to controller_desc_.layers: layer_weight_param_index_[i] is that
                // layer's AnimatorParameterBlock slot (index 0 unused — the base layer has no
                // weight parameter). Lets set_layer_weight/layer_weight write/read the live
                // parameter directly instead of walking the compiled ControllerView by name.
                std::vector<std::uint32_t> layer_weight_param_index_;
                // Ping-ponged so the buffer a just-built SkinnedInstance::previous_palette points
                // at is never the one the *next* update() overwrites — see update()'s comment.
                std::vector<Animation::JointMatrix> previous_palette_;
                std::vector<Animation::JointMatrix> pose_scratch_;
                Render::MeshId mesh_ = Render::INVALID_MESH;
                Render::Material material_{};
                Mat4 world_{};
                bool playing_ = true;
                bool use_dual_quaternion_skinning_ = false; /**< See set_dual_quaternion_skinning. */
                // Sized to the loaded mesh's morph target count by load_gltf (zero-filled), then
                // overwritten in place by set_morph_weights or, while clip_driven_morphs_ is on,
                // by sampling the base clip's morph tracks each update(); see morph_target_count().
                std::vector<float> morph_weights_;
                // Parallel to morph_weights_: the mesh's target names and their hashes, the
                // identity sample_morph_state matches the clip's tracks against.
                std::vector<std::string> morph_target_names_;
                std::vector<Animation::NameHash> morph_target_hashes_;
                bool clip_driven_morphs_ = true;
                std::vector<Render::SkinnedInstance> instances_;
        };
    } // namespace Editor
} // namespace SushiEngine

#endif
