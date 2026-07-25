/**************************************************************************/
/* device_batch_evaluator.hpp                                            */
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

#pragma once

/**
 * @file device_batch_evaluator.hpp
 * @brief The A2/§12.3 device-batched evaluator: single-clip crowd sampling on the
 * SushiRuntime task graph.
 *
 * Everything else in the animation module (`evaluator.hpp`'s `ClipEvaluator`,
 * `animator_evaluator.hpp`'s `AnimatorEvaluator`, `batch_evaluator.hpp`'s
 * `BatchEvaluator`) is a host-side C++ loop — correct and CPU-verified, but not the
 * SushiRuntime device kernel the design's §9 performance budget assumes. This is that
 * kernel, following the same shape `physics/pgs_solver.hpp`'s `ConstraintSolver` already
 * proved out: a compiled-once, replayed-every-frame `SushiRuntime::API::Graph` node, one
 * thread per instance, sequential per-instance work inside the thread (the design's own
 * "parallel across instances, sequential 256-max inner loop" — matches
 * `AnimatorEvaluator`'s compose step exactly, just moved on-device).
 *
 * Deliberately scoped down from `AnimatorEvaluator`'s full generality: one shared
 * skeleton per batch (the crowd case `batch_evaluator.hpp` already assumes), one clip
 * per instance (no blend trees, no layers, no masks, no IK — those stay host-side via
 * `AnimatorEvaluator` until they, too, get a device path), and `ClipFormat::Raw` only
 * (`bind_clip` rejects a compressed clip — ACL-shaped device decode is unimplemented,
 * a real follow-up, not a shortcut taken silently). This is the floor the crowd-LOD
 * instances (§6.6's Low/Medium tiers, the ones without IK) can actually use; hero
 * characters keep running through `AnimatorEvaluator` on the host.
 *
 * Data placement note: `Animation::AnimationDatabase` stores every asset blob in plain
 * process heap (`std::vector<std::byte>`), not shared-USM as design §3 originally
 * claimed (audited false 2026-07-25, corrected in `slop/animation_system.md`). Rather
 * than changing that widely-used class's storage model, this evaluator owns its own
 * USM-backed copies of exactly the skeleton/clip data it batches — the same choice
 * `ConstraintSolver` makes for constraints (it does not reach into some host-owned
 * constraint database either).
 */

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include <sycl/sycl.hpp>

#include <SushiRuntime/SushiRuntime.h>

#include <SushiEngine/animation/clip.hpp>
#include <SushiEngine/animation/skeleton.hpp>
#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Animation
    {
        /** @brief Sentinel returned by @ref DeviceBatchEvaluator::bind_clip on failure. */
        constexpr std::uint32_t INVALID_CLIP_HANDLE = 0xFFFFFFFFu;

        /** @brief One instance's playback state for a device-batched evaluate. */
        struct DeviceInstanceDesc
        {
            std::uint32_t clip_handle = INVALID_CLIP_HANDLE; /**< From @ref DeviceBatchEvaluator::bind_clip. */
            float time_seconds = 0.0f;                       /**< Playback time. */
            std::uint32_t loop = 1;                           /**< Non-zero loops cyclically. */
        };

        /**
         * @brief Batches single-clip skeletal sampling onto a compiled SushiRuntime graph.
         *
         * Usage: construct once, @ref bind_skeleton once, @ref bind_clip every clip the
         * batch's instances might reference, then per frame @ref set_instances (only
         * rebuilds the graph when the instance count changes — replay-only otherwise,
         * mirroring the engine-wide `compile_count == 1` invariant) and @ref evaluate.
         * @ref palettes reads back the result, `instance_count() * joint_count()`
         * `JointMatrix`, instance-major.
         */
        class DeviceBatchEvaluator
        {
            public:
                /**
                 * @brief Binds the runtime this evaluator's buffers and graph live on.
                 * @param runtime The live SushiRuntime, kept by reference for the object's life.
                 */
                explicit DeviceBatchEvaluator(SushiRuntime::API::Runtime& runtime) noexcept
                    : runtime_(runtime)
                {
                }

                DeviceBatchEvaluator(const DeviceBatchEvaluator&) = delete;
                DeviceBatchEvaluator& operator=(const DeviceBatchEvaluator&) = delete;

                /**
                 * @brief Binds the skeleton every instance in this batch shares.
                 *
                 * Uploads the parent array, bind pose, and inverse-bind matrices to
                 * device-visible buffers. Invalidates every previously bound clip (their
                 * joint order must match the new skeleton) and the compiled graph — call
                 * this before any @ref bind_clip / @ref set_instances.
                 *
                 * @param skeleton The rig every batched instance is posed against.
                 * @return True if bound; false if @p skeleton is invalid or exceeds
                 *         `MAX_JOINTS` (§4.5's 256 device-kernel scratch cap).
                 */
                bool bind_skeleton(const SkeletonView& skeleton)
                {
                    if (!skeleton.valid() || skeleton.joint_count > MAX_JOINTS)
                        return false;

                    joint_count_ = skeleton.joint_count;
                    parents_.emplace(runtime_.buffer<std::uint16_t>(joint_count_));
                    bind_t_.emplace(runtime_.buffer<Vector3f>(joint_count_));
                    bind_r_.emplace(runtime_.buffer<Quaternionf>(joint_count_));
                    bind_s_.emplace(runtime_.buffer<Vector3f>(joint_count_));
                    inverse_bind_.emplace(runtime_.buffer<JointMatrix>(joint_count_));
                    for (std::uint32_t j = 0; j < joint_count_; ++j)
                    {
                        (*parents_)[j] = skeleton.parents[j];
                        (*bind_t_)[j] = skeleton.bind_translations[j];
                        (*bind_r_)[j] = skeleton.bind_rotations[j];
                        (*bind_s_)[j] = skeleton.bind_scales[j];
                        (*inverse_bind_)[j] = skeleton.inverse_bind[j];
                    }

                    // A new skeleton invalidates every previously bound clip's joint-order
                    // agreement and the compiled graph — clear both so stale device state is
                    // never read.
                    host_translations_.clear();
                    host_rotations_.clear();
                    host_scales_.clear();
                    host_clip_meta_.clear();
                    translations_.reset();
                    rotations_.reset();
                    scales_.reset();
                    clip_meta_.reset();
                    graph_.reset();
                    host_instances_.clear();
                    instance_count_ = 0;
                    return true;
                }

                /**
                 * @brief Registers an uncompressed clip's tracks for device sampling.
                 *
                 * Rebuilds the shared device-side clip-track buffers from the host mirror
                 * every call (appends this clip, then re-uploads the whole set) — cheap
                 * relative to asset load time, and avoids depending on `Buffer<T>` resize
                 * support this codebase's SushiRuntime binding does not expose. Call at
                 * load time, not per frame.
                 *
                 * @param clip The clip to register; must be `ClipFormat::Raw` and share this
                 *             evaluator's bound skeleton's joint count.
                 * @return The handle to pass in a `DeviceInstanceDesc::clip_handle`, or
                 *         @ref INVALID_CLIP_HANDLE if the clip is invalid, compressed, or its
                 *         joint count does not match the bound skeleton.
                 */
                std::uint32_t bind_clip(const ClipView& clip)
                {
                    if (!clip.valid() || clip.format != ClipFormat::Raw ||
                        clip.joint_count != joint_count_ || clip.translations == nullptr)
                        return INVALID_CLIP_HANDLE;

                    ClipMeta meta;
                    meta.base_offset = static_cast<std::uint32_t>(host_translations_.size());
                    meta.frame_count = clip.frame_count;
                    meta.sample_rate = clip.sample_rate;

                    const std::size_t track_elements =
                        static_cast<std::size_t>(clip.frame_count) * clip.joint_count;
                    host_translations_.insert(host_translations_.end(), clip.translations,
                                              clip.translations + track_elements);
                    host_rotations_.insert(host_rotations_.end(), clip.rotations,
                                           clip.rotations + track_elements);
                    host_scales_.insert(host_scales_.end(), clip.scales,
                                        clip.scales + track_elements);

                    const std::uint32_t handle = static_cast<std::uint32_t>(host_clip_meta_.size());
                    host_clip_meta_.push_back(meta);
                    rebuild_clip_buffers();
                    return handle;
                }

                /**
                 * @brief Uploads this frame's instance list, recompiling the graph only when
                 * the instance count changed since the last call.
                 * @param instances This frame's per-instance clip/time/loop state.
                 */
                void set_instances(const std::vector<DeviceInstanceDesc>& instances)
                {
                    host_instances_ = instances;
                    const bool needs_recompile =
                        !graph_.has_value() || instances.size() != instance_count_;
                    instance_count_ = instances.size();

                    if (instance_count_ == 0)
                    {
                        graph_.reset();
                        return;
                    }

                    if (!instances_buffer_.has_value() ||
                        instances_buffer_->size() != instance_count_)
                        instances_buffer_.emplace(
                            runtime_.buffer<DeviceInstanceDesc>(instance_count_));
                    for (std::size_t i = 0; i < instance_count_; ++i)
                        (*instances_buffer_)[i] = host_instances_[i];

                    if (!palette_buffer_.has_value() ||
                        palette_buffer_->size() != instance_count_ * joint_count_)
                        palette_buffer_.emplace(
                            runtime_.buffer<JointMatrix>(instance_count_ * joint_count_));

                    if (needs_recompile)
                        rebuild_graph();
                }

                /**
                 * @brief Runs the compiled sample→compose→palette kernel once over the
                 * current instance list.
                 * @return The run report (device time, etc.) from the SushiRuntime graph.
                 */
                SushiRuntime::RunReport evaluate()
                {
                    if (!graph_.has_value())
                        return SushiRuntime::RunReport{};
                    const SushiRuntime::RunReport report = graph_->run();
                    host_palettes_.resize(instance_count_ * joint_count_);
                    for (std::size_t i = 0; i < host_palettes_.size(); ++i)
                        host_palettes_[i] = (*palette_buffer_)[i];
                    return report;
                }

                /** @brief The last @ref evaluate's palettes, `instance_count() * joint_count()` long, instance-major. */
                const std::vector<JointMatrix>& palettes() const noexcept { return host_palettes_; }

                /** @brief Joints per instance (the bound skeleton's). */
                std::uint32_t joint_count() const noexcept { return joint_count_; }

                /** @brief Instances in the last @ref set_instances call. */
                std::size_t instance_count() const noexcept { return instance_count_; }

                /** @brief Times the evaluate graph has been compiled (1 across ordinary frames). */
                std::size_t compile_count() const noexcept
                {
                    return graph_ ? graph_->compile_count() : 0;
                }

            private:
                /** @brief One bound clip's slice of the shared device track buffers. */
                struct ClipMeta
                {
                    std::uint32_t base_offset = 0; /**< First flat index (frame 0, joint 0). */
                    std::uint32_t frame_count = 0;
                    float sample_rate = 30.0f;
                };

                /** @brief Re-uploads the concatenated host track mirrors to fresh device buffers. */
                void rebuild_clip_buffers()
                {
                    translations_.emplace(runtime_.buffer<Vector3f>(host_translations_.size()));
                    rotations_.emplace(runtime_.buffer<Quaternionf>(host_rotations_.size()));
                    scales_.emplace(runtime_.buffer<Vector3f>(host_scales_.size()));
                    clip_meta_.emplace(runtime_.buffer<ClipMeta>(host_clip_meta_.size()));
                    for (std::size_t i = 0; i < host_translations_.size(); ++i)
                        (*translations_)[i] = host_translations_[i];
                    for (std::size_t i = 0; i < host_rotations_.size(); ++i)
                        (*rotations_)[i] = host_rotations_[i];
                    for (std::size_t i = 0; i < host_scales_.size(); ++i)
                        (*scales_)[i] = host_scales_[i];
                    for (std::size_t i = 0; i < host_clip_meta_.size(); ++i)
                        (*clip_meta_)[i] = host_clip_meta_[i];
                    // The clip buffers moved (fresh allocations); any compiled graph's node
                    // captured the old buffer handles by reference and must be rebuilt.
                    graph_.reset();
                }

                /**
                 * @brief Emits the one-node-per-frame sample/compose/palette kernel.
                 *
                 * One thread per instance; each thread samples every joint's track at its
                 * instance's time (bracketing frame lerp/nlerp, `ClipView::sample`'s exact
                 * loop-wrap algorithm — design §5.2/§9), forward-scans model space
                 * (`parents[j] < j`, the topological-sort invariant every skeleton cook
                 * guarantees), and writes the object-space skin palette. `Mat4 model[MAX_JOINTS]`
                 * is per-thread private scratch, the same "sequential 256-max inner loop" the
                 * host `AnimatorEvaluator` already assumes in its own compose step.
                 */
                void rebuild_graph()
                {
                    graph_.emplace(runtime_.graph());
                    const std::uint32_t joint_count = joint_count_;
                    graph_->add(
                        SushiRuntime::Extent{instance_count_},
                        SushiRuntime::In(*instances_buffer_), SushiRuntime::In(*parents_),
                        SushiRuntime::In(*bind_t_), SushiRuntime::In(*bind_r_),
                        SushiRuntime::In(*bind_s_), SushiRuntime::In(*inverse_bind_),
                        SushiRuntime::In(*translations_), SushiRuntime::In(*rotations_),
                        SushiRuntime::In(*scales_), SushiRuntime::In(*clip_meta_),
                        SushiRuntime::Out(*palette_buffer_),
                        [joint_count](sycl::id<1> id, const DeviceInstanceDesc* instances,
                                     const std::uint16_t* parents, const Vector3f* bind_t,
                                     const Quaternionf* bind_r, const Vector3f* bind_s,
                                     const JointMatrix* inverse_bind, const Vector3f* translations,
                                     const Quaternionf* rotations, const Vector3f* scales,
                                     const ClipMeta* clip_meta, JointMatrix* palette)
                        {
                            const std::size_t instance = id[0];
                            const DeviceInstanceDesc& desc = instances[instance];
                            JointMatrix* out = palette + instance * joint_count;

                            if (desc.clip_handle == INVALID_CLIP_HANDLE)
                            {
                                // No clip bound: hold the bind pose (mirrors ClipEvaluator's
                                // fallback for a joint a clip does not animate, applied here to
                                // every joint).
                                Mat4 model[MAX_JOINTS];
                                for (std::uint32_t j = 0; j < joint_count; ++j)
                                {
                                    const Mat4 local = compose_transform(
                                        Vector3{bind_t[j].x, bind_t[j].y, bind_t[j].z},
                                        Quaternion{bind_r[j].x, bind_r[j].y, bind_r[j].z,
                                                  bind_r[j].w},
                                        Vector3{bind_s[j].x, bind_s[j].y, bind_s[j].z});
                                    model[j] = parents[j] == NO_PARENT ? local
                                                                       : mul(model[parents[j]], local);
                                    out[j] = to_joint_matrix(mul(model[j], to_mat4(inverse_bind[j])));
                                }
                                return;
                            }

                            const ClipMeta& meta = clip_meta[desc.clip_handle];

                            // ClipView::sample's exact bracketing-frame algorithm (clip.hpp),
                            // reproduced here because the device path samples flat SoA buffers,
                            // not a ClipView.
                            float frame_position;
                            std::uint32_t frame0;
                            std::uint32_t frame1;
                            if (meta.frame_count <= 1)
                            {
                                frame_position = 0.0f;
                                frame0 = 0;
                                frame1 = 0;
                            }
                            else if (desc.loop != 0)
                            {
                                const float span = static_cast<float>(meta.frame_count);
                                float local = sycl::fmod(desc.time_seconds * meta.sample_rate, span);
                                if (local < 0.0f)
                                    local += span;
                                frame_position = local;
                                frame0 = static_cast<std::uint32_t>(local) % meta.frame_count;
                                frame1 = (frame0 + 1) % meta.frame_count;
                            }
                            else
                            {
                                const float last = static_cast<float>(meta.frame_count - 1);
                                float local = desc.time_seconds * meta.sample_rate;
                                if (local < 0.0f)
                                    local = 0.0f;
                                if (local > last)
                                    local = last;
                                frame_position = local;
                                frame0 = static_cast<std::uint32_t>(local);
                                frame1 = frame0 + 1 < meta.frame_count ? frame0 + 1 : frame0;
                            }
                            const float alpha = frame_position - sycl::floor(frame_position);

                            const std::size_t base0 =
                                static_cast<std::size_t>(meta.base_offset) +
                                static_cast<std::size_t>(frame0) * joint_count;
                            const std::size_t base1 =
                                static_cast<std::size_t>(meta.base_offset) +
                                static_cast<std::size_t>(frame1) * joint_count;

                            Mat4 model[MAX_JOINTS];
                            for (std::uint32_t j = 0; j < joint_count; ++j)
                            {
                                const Vector3f lt = lerp(translations[base0 + j],
                                                         translations[base1 + j], alpha);
                                const Quaternionf lr =
                                    nlerp(rotations[base0 + j], rotations[base1 + j], alpha);
                                const Vector3f ls =
                                    lerp(scales[base0 + j], scales[base1 + j], alpha);

                                const Mat4 local = compose_transform(
                                    Vector3{lt.x, lt.y, lt.z},
                                    Quaternion{lr.x, lr.y, lr.z, lr.w}, Vector3{ls.x, ls.y, ls.z});
                                model[j] = parents[j] == NO_PARENT ? local
                                                                   : mul(model[parents[j]], local);
                                out[j] = to_joint_matrix(mul(model[j], to_mat4(inverse_bind[j])));
                            }
                        });
                }

                SushiRuntime::API::Runtime& runtime_;
                std::uint32_t joint_count_ = 0;

                std::optional<SushiRuntime::API::Buffer<std::uint16_t>> parents_;
                std::optional<SushiRuntime::API::Buffer<Vector3f>> bind_t_;
                std::optional<SushiRuntime::API::Buffer<Quaternionf>> bind_r_;
                std::optional<SushiRuntime::API::Buffer<Vector3f>> bind_s_;
                std::optional<SushiRuntime::API::Buffer<JointMatrix>> inverse_bind_;

                std::vector<Vector3f> host_translations_;
                std::vector<Quaternionf> host_rotations_;
                std::vector<Vector3f> host_scales_;
                std::vector<ClipMeta> host_clip_meta_;
                std::optional<SushiRuntime::API::Buffer<Vector3f>> translations_;
                std::optional<SushiRuntime::API::Buffer<Quaternionf>> rotations_;
                std::optional<SushiRuntime::API::Buffer<Vector3f>> scales_;
                std::optional<SushiRuntime::API::Buffer<ClipMeta>> clip_meta_;

                std::vector<DeviceInstanceDesc> host_instances_;
                std::optional<SushiRuntime::API::Buffer<DeviceInstanceDesc>> instances_buffer_;
                std::optional<SushiRuntime::API::Buffer<JointMatrix>> palette_buffer_;
                std::vector<JointMatrix> host_palettes_;
                std::size_t instance_count_ = 0;

                std::optional<SushiRuntime::API::Graph> graph_;
        };
    } // namespace Animation
} // namespace SushiEngine
