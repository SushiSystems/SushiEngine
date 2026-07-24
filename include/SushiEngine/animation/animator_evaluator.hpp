/**************************************************************************/
/* animator_evaluator.hpp                                                */
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

#pragma once

/**
 * @file animator_evaluator.hpp
 * @brief The A4/A5 frame evaluator: blend trees, crossfades, mask-gated layers, additive.
 *
 * Where @ref animator_step is the deterministic sim tick, this is the derived per-frame pose
 * (design §5.2). For one animator it: resolves each layer's active state (and any in-progress
 * crossfade) into weighted clip contributions — a single clip, or a blend tree read against the
 * parameters (A4); samples and weight-blends them into that layer's local pose; folds the
 * layers, each gated by its avatar mask and animatable weight, override toward or additive onto
 * the base (A5); composes model space by the topological forward scan; and builds the
 * object-space skin palette. Nothing here is snapshotted — it is recomputed each frame from the
 * sim state, so it never enters rollback. Host / single-instance in this phase, the same shape
 * the batched device path takes later.
 */

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <SushiEngine/animation/animation_database.hpp>
#include <SushiEngine/animation/animator_components.hpp>
#include <SushiEngine/animation/animator_controller.hpp>
#include <SushiEngine/animation/avatar_mask.hpp>
#include <SushiEngine/animation/blend_tree.hpp>
#include <SushiEngine/animation/clip.hpp>
#include <SushiEngine/animation/pose_modifier.hpp>
#include <SushiEngine/animation/skeleton.hpp>
#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Animation
    {
        /**
         * @brief Evaluates one animator (layers, blend trees, masks, additive) into a skin palette.
         *
         * Reused across frames: @ref resize once when the skeleton is bound (or automatically on
         * the first @ref evaluate for a new joint count), then @ref evaluate each frame. The
         * palette is read back after evaluation, object space, ready to upload.
         */
        class AnimatorEvaluator
        {
            public:
                /**
                 * @brief Sizes the scratch and output buffers to a skeleton's joint count.
                 * @param joint_count Joints the bound skeleton has.
                 */
                void resize(std::uint32_t joint_count)
                {
                    base_translations_.assign(joint_count, Vector3f{});
                    base_rotations_.assign(joint_count, Quaternionf{});
                    base_scales_.assign(joint_count, Vector3f{1.0f, 1.0f, 1.0f});
                    layer_translations_.assign(joint_count, Vector3f{});
                    layer_rotations_.assign(joint_count, Quaternionf{});
                    layer_scales_.assign(joint_count, Vector3f{1.0f, 1.0f, 1.0f});
                    sample_translations_.assign(joint_count, Vector3f{});
                    sample_rotations_.assign(joint_count, Quaternionf{});
                    sample_scales_.assign(joint_count, Vector3f{1.0f, 1.0f, 1.0f});
                    accumulator_translations_.assign(joint_count, Vector3f{});
                    accumulator_scales_.assign(joint_count, Vector3f{});
                    accumulator_x_.assign(joint_count, 0.0f);
                    accumulator_y_.assign(joint_count, 0.0f);
                    accumulator_z_.assign(joint_count, 0.0f);
                    accumulator_w_.assign(joint_count, 0.0f);
                    reference_rotations_.assign(joint_count, Quaternionf{});
                    have_reference_.assign(joint_count, 0);
                    mask_.assign(joint_count, 1.0f);
                    model_.assign(joint_count, Mat4{});
                    palette_.assign(joint_count, JointMatrix{});
                }

                /**
                 * @brief Evaluates an animator instance into the skin palette.
                 *
                 * Samples each layer at its own @c normalized_time (the render path advances a copy
                 * of the instance by `interpolation × dt × speed` before calling for smooth motion —
                 * this evaluator reads the times as given, keeping sim and render cleanly separated).
                 *
                 * @param controller The compiled controller driving the animator.
                 * @param database   The asset database (clips, masks).
                 * @param instance   The animator's sim state (layer states, parameters).
                 * @param skeleton   The bound skeleton.
                 */
                void evaluate(const ControllerView& controller, const IAnimationDatabase& database,
                              const AnimatorInstance& instance, const SkeletonView& skeleton)
                {
                    evaluate(controller, database, instance, skeleton, nullptr, 0);
                }

                /**
                 * @brief Evaluates an animator instance, then runs an ordered pose-modifier stack.
                 *
                 * The modifiers (IK, look-at, ...) run in model space after the layers blend and
                 * before the palette is built (design §5.3), so they correct the final pose. Each
                 * edits the local pose it owns and re-composes; they run in array order.
                 *
                 * @param controller     The compiled controller driving the animator.
                 * @param database       The asset database (clips, masks).
                 * @param instance       The animator's sim state.
                 * @param skeleton       The bound skeleton.
                 * @param modifiers      The ordered modifier stack (may be null).
                 * @param modifier_count Entries in @p modifiers.
                 */
                void evaluate(const ControllerView& controller, const IAnimationDatabase& database,
                              const AnimatorInstance& instance, const SkeletonView& skeleton,
                              const IPoseModifier* const* modifiers, std::size_t modifier_count)
                {
                    const std::uint32_t count = skeleton.joint_count;
                    if (model_.size() != count)
                        resize(count);

                    seed_bind(skeleton, base_translations_, base_rotations_, base_scales_);

                    const std::uint32_t layers =
                        controller.layer_count < ANIMATOR_MAX_LAYERS ? controller.layer_count
                                                                     : ANIMATOR_MAX_LAYERS;
                    for (std::uint32_t l = 0; l < layers; ++l)
                    {
                        const LayerRecord& layer = controller.layers[l];
                        const AnimatorLayerState& s = instance.layers[l];
                        if (s.current_state < 0)
                            continue;

                        pose_layer(controller, database, instance.parameters, layer, s, skeleton);

                        if (l == 0)
                        {
                            base_translations_ = layer_translations_;
                            base_rotations_ = layer_rotations_;
                            base_scales_ = layer_scales_;
                            continue;
                        }

                        resolve_mask(database, layer, skeleton);
                        const bool additive =
                            static_cast<LayerBlendMode>(layer.blend_mode) == LayerBlendMode::Additive;
                        for (std::uint32_t j = 0; j < count; ++j)
                        {
                            const float gate = s.weight * mask_[j];
                            if (gate <= 0.0f)
                                continue;
                            if (additive)
                                fold_additive(j, gate);
                            else
                                fold_override(j, gate);
                        }
                    }

                    // Compose model space, run the pose-modifier stack in it (§5.3), then skin.
                    compose_model(skeleton, base_translations_.data(), base_rotations_.data(),
                                  base_scales_.data(), model_.data());
                    for (std::size_t m = 0; m < modifier_count; ++m)
                    {
                        if (modifiers[m] == nullptr)
                            continue;
                        PoseModifierContext context;
                        context.skeleton = skeleton;
                        context.local_translations = base_translations_.data();
                        context.local_rotations = base_rotations_.data();
                        context.local_scales = base_scales_.data();
                        context.model = model_.data();
                        modifiers[m]->solve(context);
                    }
                    build_palette(skeleton);
                }

                /** @brief The object-space skin palette (`model * inverse_bind`) per joint. */
                const std::vector<JointMatrix>& palette() const noexcept { return palette_; }

                /** @brief The model-space matrix per joint (bind space, post-compose). */
                const std::vector<Mat4>& model() const noexcept { return model_; }

            private:
                /** @brief One weighted clip sampled at a specific normalized time. */
                struct SampleContribution
                {
                    AssetId clip = INVALID_ASSET;
                    float weight = 0.0f;
                    float normalized_time = 0.0f;
                };

                /** @brief Seeds three local-pose arrays from a skeleton's bind pose. */
                static void seed_bind(const SkeletonView& skeleton, std::vector<Vector3f>& t,
                                      std::vector<Quaternionf>& r, std::vector<Vector3f>& s)
                {
                    for (std::uint32_t j = 0; j < skeleton.joint_count; ++j)
                    {
                        t[j] = skeleton.bind_translations[j];
                        r[j] = skeleton.bind_rotations[j];
                        s[j] = skeleton.bind_scales[j];
                    }
                }

                /**
                 * @brief Appends a state's clip contributions (single clip or blend tree) to a list.
                 * @param weight_scale Scales every contribution (crossfade progress).
                 * @param out_count    Running count into @p out, advanced in place.
                 */
                static void gather_state(const ControllerView& controller,
                                         const AnimatorParameterBlock& parameters,
                                         const StateRecord& state, float normalized_time,
                                         float weight_scale, SampleContribution* out,
                                         std::uint32_t& out_count, std::uint32_t max_out)
                {
                    if (state.blend_tree < 0)
                    {
                        if (state.clip != INVALID_ASSET && out_count < max_out)
                        {
                            out[out_count].clip = state.clip;
                            out[out_count].weight = weight_scale;
                            out[out_count].normalized_time = normalized_time;
                            ++out_count;
                        }
                        return;
                    }
                    BlendContribution resolved[MAX_BLEND_CONTRIBUTIONS];
                    const std::uint32_t n = resolve_blend_tree(
                        controller.nodes, controller.children, controller.pairs,
                        static_cast<std::uint32_t>(state.blend_tree), parameters, resolved,
                        MAX_BLEND_CONTRIBUTIONS);
                    for (std::uint32_t i = 0; i < n && out_count < max_out; ++i)
                    {
                        out[out_count].clip = resolved[i].clip;
                        out[out_count].weight = resolved[i].weight * weight_scale;
                        out[out_count].normalized_time = normalized_time;
                        ++out_count;
                    }
                }

                /** @brief Resolves one layer's active state (and crossfade) into its local pose. */
                void pose_layer(const ControllerView& controller, const IAnimationDatabase& database,
                                const AnimatorParameterBlock& parameters, const LayerRecord& layer,
                                const AnimatorLayerState& s, const SkeletonView& skeleton)
                {
                    SampleContribution contributions[2 * MAX_BLEND_CONTRIBUTIONS];
                    std::uint32_t contribution_count = 0;
                    const std::uint32_t max_out = 2 * MAX_BLEND_CONTRIBUTIONS;

                    const float progress = s.transition_state >= 0 ? s.transition_progress : 0.0f;
                    const StateRecord& current =
                        controller.states[layer.state_base + static_cast<std::uint32_t>(s.current_state)];
                    gather_state(controller, parameters, current, s.normalized_time, 1.0f - progress,
                                 contributions, contribution_count, max_out);
                    if (s.transition_state >= 0 && s.next_state >= 0)
                    {
                        const StateRecord& next =
                            controller.states[layer.state_base + static_cast<std::uint32_t>(s.next_state)];
                        gather_state(controller, parameters, next, s.next_normalized_time, progress,
                                     contributions, contribution_count, max_out);
                    }

                    blend_contributions(database, skeleton, contributions, contribution_count);
                }

                /** @brief Weight-blends sampled contributions into the layer local-pose arrays. */
                void blend_contributions(const IAnimationDatabase& database,
                                         const SkeletonView& skeleton,
                                         const SampleContribution* contributions, std::uint32_t count)
                {
                    const std::uint32_t joints = skeleton.joint_count;
                    for (std::uint32_t j = 0; j < joints; ++j)
                    {
                        accumulator_translations_[j] = Vector3f{0.0f, 0.0f, 0.0f};
                        accumulator_scales_[j] = Vector3f{0.0f, 0.0f, 0.0f};
                        accumulator_x_[j] = 0.0f;
                        accumulator_y_[j] = 0.0f;
                        accumulator_z_[j] = 0.0f;
                        accumulator_w_[j] = 0.0f;
                        have_reference_[j] = 0;
                    }

                    float total_weight = 0.0f;
                    for (std::uint32_t c = 0; c < count; ++c)
                    {
                        const float w = contributions[c].weight;
                        if (w <= 0.0f)
                            continue;
                        total_weight += w;

                        const ClipView clip = database.clip(contributions[c].clip);
                        const bool valid = clip.valid();
                        if (valid)
                        {
                            const float time = contributions[c].normalized_time * clip.duration;
                            clip.sample(time, true, sample_translations_.data(),
                                        sample_rotations_.data(), sample_scales_.data());
                        }

                        for (std::uint32_t j = 0; j < joints; ++j)
                        {
                            const bool driven = valid && j < clip.joint_count;
                            const Vector3f t = driven ? sample_translations_[j] : skeleton.bind_translations[j];
                            const Vector3f sc = driven ? sample_scales_[j] : skeleton.bind_scales[j];
                            Quaternionf r = driven ? sample_rotations_[j] : skeleton.bind_rotations[j];

                            accumulator_translations_[j] = accumulator_translations_[j] + t * w;
                            accumulator_scales_[j] = accumulator_scales_[j] + sc * w;

                            if (have_reference_[j] == 0)
                            {
                                reference_rotations_[j] = r;
                                have_reference_[j] = 1;
                            }
                            if (dot(reference_rotations_[j], r) < 0.0f)
                            {
                                r.x = -r.x;
                                r.y = -r.y;
                                r.z = -r.z;
                                r.w = -r.w;
                            }
                            accumulator_x_[j] += r.x * w;
                            accumulator_y_[j] += r.y * w;
                            accumulator_z_[j] += r.z * w;
                            accumulator_w_[j] += r.w * w;
                        }
                    }

                    for (std::uint32_t j = 0; j < joints; ++j)
                    {
                        if (total_weight > 1e-6f)
                        {
                            const float inv = 1.0f / total_weight;
                            layer_translations_[j] = accumulator_translations_[j] * inv;
                            layer_scales_[j] = accumulator_scales_[j] * inv;
                            layer_rotations_[j] = normalize(Quaternionf{accumulator_x_[j], accumulator_y_[j],
                                                                        accumulator_z_[j], accumulator_w_[j]});
                        }
                        else
                        {
                            layer_translations_[j] = skeleton.bind_translations[j];
                            layer_rotations_[j] = skeleton.bind_rotations[j];
                            layer_scales_[j] = skeleton.bind_scales[j];
                        }
                    }
                }

                /** @brief Fills @ref mask_ with one weight per joint for a layer's avatar mask. */
                void resolve_mask(const IAnimationDatabase& database, const LayerRecord& layer,
                                  const SkeletonView& skeleton)
                {
                    if (layer.mask != INVALID_ASSET && database.has_mask(layer.mask))
                    {
                        database.mask(layer.mask).resolve(skeleton, mask_.data());
                    }
                    else
                    {
                        for (std::uint32_t j = 0; j < skeleton.joint_count; ++j)
                            mask_[j] = 1.0f;
                    }
                }

                /** @brief Blends one joint of the base toward the layer pose (override). */
                void fold_override(std::uint32_t j, float gate)
                {
                    base_translations_[j] = lerp(base_translations_[j], layer_translations_[j], gate);
                    base_scales_[j] = lerp(base_scales_[j], layer_scales_[j], gate);
                    base_rotations_[j] = nlerp(base_rotations_[j], layer_rotations_[j], gate);
                }

                /** @brief Applies one joint of the layer's baked delta onto the base (additive). */
                void fold_additive(std::uint32_t j, float gate)
                {
                    base_translations_[j] = base_translations_[j] + layer_translations_[j] * gate;
                    const Quaternionf identity{0.0f, 0.0f, 0.0f, 1.0f};
                    base_rotations_[j] =
                        normalize(mul(base_rotations_[j], slerp(identity, layer_rotations_[j], gate)));
                    base_scales_[j] = Vector3f{base_scales_[j].x * (1.0f + (layer_scales_[j].x - 1.0f) * gate),
                                               base_scales_[j].y * (1.0f + (layer_scales_[j].y - 1.0f) * gate),
                                               base_scales_[j].z * (1.0f + (layer_scales_[j].z - 1.0f) * gate)};
                }

                /** @brief Builds the object-space skin palette from the composed model space. */
                void build_palette(const SkeletonView& skeleton)
                {
                    for (std::uint32_t i = 0; i < skeleton.joint_count; ++i)
                        palette_[i] = to_joint_matrix(mul(model_[i], to_mat4(skeleton.inverse_bind[i])));
                }

                std::vector<Vector3f> base_translations_;
                std::vector<Quaternionf> base_rotations_;
                std::vector<Vector3f> base_scales_;
                std::vector<Vector3f> layer_translations_;
                std::vector<Quaternionf> layer_rotations_;
                std::vector<Vector3f> layer_scales_;
                std::vector<Vector3f> sample_translations_;
                std::vector<Quaternionf> sample_rotations_;
                std::vector<Vector3f> sample_scales_;
                std::vector<Vector3f> accumulator_translations_;
                std::vector<Vector3f> accumulator_scales_;
                std::vector<float> accumulator_x_;
                std::vector<float> accumulator_y_;
                std::vector<float> accumulator_z_;
                std::vector<float> accumulator_w_;
                std::vector<Quaternionf> reference_rotations_;
                std::vector<std::uint8_t> have_reference_;
                std::vector<float> mask_;
                std::vector<Mat4> model_;
                std::vector<JointMatrix> palette_;
        };
    } // namespace Animation
} // namespace SushiEngine
