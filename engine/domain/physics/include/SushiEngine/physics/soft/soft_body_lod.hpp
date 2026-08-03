/**************************************************************************/
/* soft_body_lod.hpp                                                      */
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
 * @file soft_body_lod.hpp
 * @brief §9.7: which model simulates a body, and how state crosses between them.
 *
 * Three separable problems, in three pieces that can each be tested alone:
 *
 * 1. **Which tier.** @ref select_soft_body_tier, a pure function of the coverage
 *    and the tier the body is already in. Hysteresis lives here and nowhere else.
 * 2. **Carrying the state across.** @ref coarsen_soft_body_state and
 *    @ref refine_soft_body_state, which move a deformed pose between two of the
 *    asset's lattices through the barycentric mappings the cooker stored.
 * 3. **Owning the tiers and doing both.** @ref SoftBodyLODChain.
 *
 * **The transfer is written in displacements, not positions,** and that is the
 * whole reason a body does not pop. Reconstructing a coarse vertex as
 * `sum(weight * fine_vertex)` is exact only if the fine lattice is exactly where
 * the embedding says it should be, which it never is — a coarse lattice cannot
 * represent every pose a finer one can, so the reconstruction of an *undeformed*
 * body already lands a fraction of a millimetre off its own rest position. Doing
 * it in displacements makes the rest pose transfer exactly by construction, and
 * leaves only the genuinely unrepresentable part of the deformation to be lost —
 * which is the fidelity the tier was traded for, and is visible as softening
 * rather than as a jump.
 *
 * **Coarsening is the transpose of refining.** Refining evaluates the mapping:
 * each fine vertex is a weighted sum of four coarse ones. Coarsening has no
 * stored inverse and inverting the operator properly is a least-squares solve per
 * transition, so instead each fine vertex's displacement is *scattered* back to
 * the four coarse vertices with the same weights and normalized by the weight
 * that arrived. That is the standard mass-lumped restriction, it costs one pass,
 * and — the property that matters — it reproduces a rigid translation exactly,
 * so a body that is merely falling crosses a tier boundary with no motion at all.
 */

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/cooking/soft_body_asset.hpp>
#include <SushiEngine/physics/core/rigid_body.hpp>
#include <SushiEngine/physics/soft/soft_body_model.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief How much of the screen a body of @p radius at @p distance covers.
         *
         * The projected radius in normalized device coordinates: one means the body
         * fills half the viewport height, and it is independent of the resolution
         * the frame happens to be rendered at — a body must not change how it is
         * simulated because a window was resized, which is what a pixel-count
         * measure would do.
         *
         * @param radius        The body's bounding radius, in metres.
         * @param distance      Distance from the camera to its centre, in metres.
         * @param tan_half_fov  Tangent of half the vertical field of view.
         * @return The coverage; large (not infinite) for a body at the camera.
         */
        template <typename T>
        inline T soft_body_screen_coverage(T radius, T distance, T tan_half_fov) noexcept
        {
            if (!(distance > T(0)) || !(tan_half_fov > T(0)))
                return T(1e9);
            return radius / (distance * tan_half_fov);
        }

        /**
         * @brief How a body chooses its tier, and how reluctant it is to change.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        struct SoftBodyLODSettings
        {
            /**
             * @brief Coverage below which the next coarser tier takes over.
             *
             * One entry per transition, so a chain of `n` tiers has `n - 1` of them,
             * strictly descending. `thresholds[k]` is the boundary between tier `k`
             * and tier `k + 1`.
             */
            std::vector<T> thresholds;

            /**
             * @brief How far past a threshold a body must go before it crosses back.
             *
             * A fraction of the threshold. Zero would make a body sitting exactly on
             * a boundary swap tiers every frame — and a swap is not free, so the
             * worst case of no hysteresis is not a cosmetic flicker but the most
             * expensive thing the system can do, repeated. A fifth is wide enough
             * that ordinary camera motion does not reach across it.
             */
            T hysteresis = T(0.2);
        };

        /**
         * @brief Which tier a body at @p coverage belongs in, given where it is now.
         *
         * A pure function: same inputs, same answer, no state of its own. The
         * current tier is an *input* rather than something the selector remembers,
         * which is what makes hysteresis testable — a band that only exists inside a
         * stateful object can only be tested by driving the object through a
         * sequence and hoping the sequence covers the case.
         *
         * @param settings    The thresholds and the band.
         * @param tier_count  How many tiers exist; the answer is below this.
         * @param current     The tier the body is in now.
         * @param coverage    Its screen coverage this frame.
         * @return The tier it should be in.
         */
        template <typename T>
        inline std::size_t select_soft_body_tier(const SoftBodyLODSettings<T>& settings,
                                                 std::size_t tier_count, std::size_t current,
                                                 T coverage) noexcept
        {
            if (tier_count == 0)
                return 0;
            if (current >= tier_count)
                current = tier_count - 1;

            const T band = settings.hysteresis > T(0) ? settings.hysteresis : T(0);
            // Bounded by the tier count rather than by `while (true)`: the bands
            // cannot oscillate, but a caller that handed in ascending thresholds by
            // mistake should get a wrong answer rather than a hung frame.
            for (std::size_t moved = 0; moved < tier_count; ++moved)
            {
                if (current + 1 < tier_count && current < settings.thresholds.size() &&
                    coverage < settings.thresholds[current] * (T(1) - band))
                {
                    ++current;
                    continue;
                }
                if (current > 0 && current - 1 < settings.thresholds.size() &&
                    coverage > settings.thresholds[current - 1] * (T(1) + band))
                {
                    --current;
                    continue;
                }
                break;
            }
            return current;
        }

        /**
         * @brief Reconstructs a finer lattice's pose from a coarser one's.
         *
         * Applies the mappings stored on @p coarse_level, which bind level
         * `coarse_level - 1`'s vertices into @p coarse_level's elements.
         *
         * @param view         A validated soft-body asset.
         * @param coarse_level The level being left; must be at least one.
         * @param coarse       Its particles.
         * @param coarse_count How many.
         * @param fine         The finer level's particles, written.
         * @param fine_count   How many.
         * @return False when the asset, the level or the array sizes do not line up.
         */
        template <typename T>
        inline bool refine_soft_body_state(const Cooking::SoftBodyAssetView& view,
                                           std::uint32_t coarse_level,
                                           const RigidBodyT<T>* coarse, std::size_t coarse_count,
                                           RigidBodyT<T>* fine, std::size_t fine_count)
        {
            if (!view.valid || coarse_level == 0 || coarse_level >= view.level_count)
                return false;
            const Cooking::SoftBodyLevelRecord& record = view.levels[coarse_level];
            const Cooking::SoftBodyLevelRecord& finer = view.levels[coarse_level - 1];
            if (coarse_count < record.vertex_count || fine_count < record.mapping_count)
                return false;

            for (std::uint32_t k = 0; k < record.mapping_count; ++k)
            {
                const Cooking::SoftBodyBinding& mapping = view.mappings[record.first_mapping + k];
                if (mapping.tetrahedron < record.first_tetrahedron ||
                    mapping.tetrahedron >= record.first_tetrahedron + record.tetrahedron_count)
                    continue;

                const std::uint32_t* element =
                    view.tetrahedra + std::size_t(mapping.tetrahedron) * 4;
                T mapping_weight[4];
                Cooking::read_binding_weights(mapping, mapping_weight);
                Vector3T<T> displacement{T(0), T(0), T(0)};
                Vector3T<T> velocity{T(0), T(0), T(0)};
                bool usable = true;
                for (int corner = 0; corner < 4; ++corner)
                {
                    const std::uint32_t global = element[corner];
                    if (global < record.first_vertex ||
                        global - record.first_vertex >= record.vertex_count)
                    {
                        usable = false;
                        break;
                    }
                    const std::uint32_t local = global - record.first_vertex;
                    const T weight = mapping_weight[corner];
                    const Vector3T<T> rest{T(view.vertices[global].x), T(view.vertices[global].y),
                                           T(view.vertices[global].z)};
                    displacement = displacement + (coarse[local].position - rest) * weight;
                    velocity = velocity + coarse[local].velocity * weight;
                }
                if (!usable)
                    continue;

                const Vector3& source = view.vertices[finer.first_vertex + k];
                RigidBodyT<T>& particle = fine[k];
                particle.position = Vector3T<T>{T(source.x), T(source.y), T(source.z)} + displacement;
                particle.prev_position = particle.position;
                particle.velocity = velocity;
            }
            return true;
        }

        /**
         * @brief Carries a finer lattice's pose onto a coarser one's.
         *
         * The transpose of @ref refine_soft_body_state: every fine vertex scatters
         * its displacement to the four coarse vertices that would have reconstructed
         * it, and each coarse vertex divides by the weight that reached it.
         *
         * A coarse vertex that no mapping reaches — possible where a fine lattice
         * has a detached component — takes the average displacement of the ones that
         * were reached, rather than staying at its rest position. Staying put would
         * tear the coarse lattice apart the moment the body moved, which is a far
         * worse answer than being approximately right.
         *
         * @param view         A validated soft-body asset.
         * @param coarse_level The level being entered; must be at least one.
         * @param fine         The finer level's particles.
         * @param fine_count   How many.
         * @param coarse       This level's particles, written.
         * @param coarse_count How many.
         * @return False when the asset, the level or the array sizes do not line up.
         */
        template <typename T>
        inline bool coarsen_soft_body_state(const Cooking::SoftBodyAssetView& view,
                                            std::uint32_t coarse_level, const RigidBodyT<T>* fine,
                                            std::size_t fine_count, RigidBodyT<T>* coarse,
                                            std::size_t coarse_count)
        {
            if (!view.valid || coarse_level == 0 || coarse_level >= view.level_count)
                return false;
            const Cooking::SoftBodyLevelRecord& record = view.levels[coarse_level];
            const Cooking::SoftBodyLevelRecord& finer = view.levels[coarse_level - 1];
            if (coarse_count < record.vertex_count || fine_count < record.mapping_count)
                return false;

            std::vector<Vector3T<T>> displacement(record.vertex_count,
                                                  Vector3T<T>{T(0), T(0), T(0)});
            std::vector<Vector3T<T>> velocity(record.vertex_count, Vector3T<T>{T(0), T(0), T(0)});
            std::vector<T> weight_sum(record.vertex_count, T(0));

            for (std::uint32_t k = 0; k < record.mapping_count; ++k)
            {
                const Cooking::SoftBodyBinding& mapping = view.mappings[record.first_mapping + k];
                if (mapping.tetrahedron < record.first_tetrahedron ||
                    mapping.tetrahedron >= record.first_tetrahedron + record.tetrahedron_count)
                    continue;

                const Vector3& source = view.vertices[finer.first_vertex + k];
                const Vector3T<T> fine_rest{T(source.x), T(source.y), T(source.z)};
                const Vector3T<T> fine_displacement = fine[k].position - fine_rest;

                const std::uint32_t* element =
                    view.tetrahedra + std::size_t(mapping.tetrahedron) * 4;
                T mapping_weight[4];
                Cooking::read_binding_weights(mapping, mapping_weight);
                for (int corner = 0; corner < 4; ++corner)
                {
                    const std::uint32_t global = element[corner];
                    if (global < record.first_vertex ||
                        global - record.first_vertex >= record.vertex_count)
                        continue;
                    const std::uint32_t local = global - record.first_vertex;
                    const T weight = mapping_weight[corner];
                    displacement[local] = displacement[local] + fine_displacement * weight;
                    velocity[local] = velocity[local] + fine[k].velocity * weight;
                    weight_sum[local] += weight;
                }
            }

            Vector3T<T> average_displacement{T(0), T(0), T(0)};
            Vector3T<T> average_velocity{T(0), T(0), T(0)};
            std::size_t reached = 0;
            for (std::uint32_t i = 0; i < record.vertex_count; ++i)
            {
                if (!(weight_sum[i] > T(1e-6)))
                    continue;
                displacement[i] = displacement[i] * (T(1) / weight_sum[i]);
                velocity[i] = velocity[i] * (T(1) / weight_sum[i]);
                average_displacement = average_displacement + displacement[i];
                average_velocity = average_velocity + velocity[i];
                ++reached;
            }
            if (reached > 0)
            {
                average_displacement = average_displacement * (T(1) / T(reached));
                average_velocity = average_velocity * (T(1) / T(reached));
            }

            for (std::uint32_t i = 0; i < record.vertex_count; ++i)
            {
                const Vector3& source = view.vertices[record.first_vertex + i];
                const bool touched = weight_sum[i] > T(1e-6);
                RigidBodyT<T>& particle = coarse[i];
                particle.position = Vector3T<T>{T(source.x), T(source.y), T(source.z)} +
                                    (touched ? displacement[i] : average_displacement);
                particle.prev_position = particle.position;
                particle.velocity = touched ? velocity[i] : average_velocity;
            }
            return true;
        }

        /**
         * @brief One tier of a chain: a model, and which of the asset's lattices it runs on.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        struct SoftBodyTier
        {
            /** @brief The model; owned, because a tier's lifetime is the chain's. */
            std::unique_ptr<ISoftBodyModel<T>> model;

            /**
             * @brief Which cooked level this tier's particles belong to.
             *
             * Two tiers may name the *same* level — the shape-matching tier and the
             * rigid tier both sit on the coarsest lattice and differ only in what
             * they do with it — and when they do, crossing between them is a copy
             * rather than a barycentric remap, which is both cheaper and exact.
             */
            std::uint32_t level = 0;
        };

        /**
         * @brief A body's tiers, the one that is currently simulating it, and the swap.
         *
         * The chain is what §9.7 calls a substitution: everything outside it holds
         * @ref active as an `ISoftBodyModel*` and never learns that the pointer
         * changed. Tiers are added finest first.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        class SoftBodyLODChain
        {
            public:
                /** @brief Which tier is which coverage; see @ref SoftBodyLODSettings. */
                SoftBodyLODSettings<T> settings;

                /**
                 * @brief Adds a tier, coarser than every tier already added.
                 *
                 * @param model The tier's model; ownership is taken.
                 * @param level Which cooked level its particles belong to.
                 */
                void add_tier(std::unique_ptr<ISoftBodyModel<T>> model, std::uint32_t level)
                {
                    if (model == nullptr)
                        return;
                    SoftBodyTier<T> tier;
                    tier.model = std::move(model);
                    tier.level = level;
                    tiers_.push_back(std::move(tier));
                }

                /** @brief The model currently simulating the body, or null for an empty chain. */
                ISoftBodyModel<T>* active() noexcept
                {
                    return tiers_.empty() ? nullptr : tiers_[current_].model.get();
                }

                /** @brief Which tier is active; zero for an empty chain. */
                std::size_t current_tier() const noexcept { return current_; }

                /** @brief How many tiers the chain holds. */
                std::size_t tier_count() const noexcept { return tiers_.size(); }

                /**
                 * @brief Chooses a tier for this frame's coverage and moves the state if it changed.
                 *
                 * @param view     The cooked asset the tiers were built from.
                 * @param coverage The body's screen coverage this frame.
                 * @return True when the active tier changed.
                 */
                bool update(const Cooking::SoftBodyAssetView& view, T coverage)
                {
                    if (tiers_.empty())
                        return false;
                    const std::size_t target =
                        select_soft_body_tier(settings, tiers_.size(), current_, coverage);
                    if (target == current_)
                        return false;

                    // One step at a time, in both directions. A body that jumps three
                    // tiers — a camera cut, a teleport — has its state carried through
                    // every lattice in between rather than remapped directly, because
                    // the cooker only ever stored mappings between *adjacent* levels.
                    while (current_ < target)
                    {
                        transfer(view, current_, current_ + 1);
                        ++current_;
                    }
                    while (current_ > target)
                    {
                        transfer(view, current_, current_ - 1);
                        --current_;
                    }
                    return true;
                }

            private:
                /**
                 * @brief Moves the pose from tier @p from onto tier @p to.
                 *
                 * @param view The cooked asset.
                 * @param from The tier holding the current pose.
                 * @param to   The tier about to hold it.
                 */
                void transfer(const Cooking::SoftBodyAssetView& view, std::size_t from,
                              std::size_t to)
                {
                    ISoftBodyModel<T>* source = tiers_[from].model.get();
                    ISoftBodyModel<T>* target = tiers_[to].model.get();
                    if (source == nullptr || target == nullptr)
                        return;

                    const SoftSurfaceView<T> source_view = source->surface();
                    const SoftSurfaceView<T> target_view = target->surface();

                    if (tiers_[from].level == tiers_[to].level)
                    {
                        // Same lattice, different physics: the pose is already in the
                        // right coordinates and copying it is exact.
                        const std::size_t count =
                            source_view.particle_count < target_view.particle_count
                                ? source_view.particle_count
                                : target_view.particle_count;
                        for (std::size_t i = 0; i < count; ++i)
                        {
                            target_view.particles[i].position = source_view.particles[i].position;
                            target_view.particles[i].prev_position =
                                source_view.particles[i].position;
                            target_view.particles[i].velocity = source_view.particles[i].velocity;
                        }
                    }
                    else if (tiers_[to].level > tiers_[from].level)
                    {
                        coarsen_soft_body_state(view, tiers_[to].level, source_view.particles,
                                                source_view.particle_count, target_view.particles,
                                                target_view.particle_count);
                    }
                    else
                    {
                        refine_soft_body_state(view, tiers_[from].level, source_view.particles,
                                               source_view.particle_count, target_view.particles,
                                               target_view.particle_count);
                    }

                    target->on_state_replaced();
                }

                std::vector<SoftBodyTier<T>> tiers_;
                std::size_t current_ = 0;
        };
    } // namespace Physics
} // namespace SushiEngine
