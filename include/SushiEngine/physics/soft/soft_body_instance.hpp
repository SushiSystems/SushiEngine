/**************************************************************************/
/* soft_body_instance.hpp                                                 */
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
 * @file soft_body_instance.hpp
 * @brief §6.5's cosmetic `float` column, resolved once at instantiation.
 *
 * Everything in `physics/` is already a template over its scalar type, so a
 * narrower column needs no new arithmetic — `FiniteElementModel<float>` is a
 * complete, correct soft body today. What is missing, and what this file is, are
 * the two things §6.5 actually asks for:
 *
 * 1. **A rule** for which column a body gets, read from its cooked asset and its
 *    component flags rather than chosen at the call site.
 * 2. **A holder** that owns one column or the other and hands out results in the
 *    engine's boundary `Scalar` either way, so nothing downstream has to be
 *    written twice or templated on a decision it does not make.
 *
 * **A body cannot change precision while it is simulating**, and the way that is
 * enforced here is that there is no way to say so: @ref SoftBodyInstance::create
 * takes the precision, and nothing else ever writes it. Changing a body's
 * precision means destroying it and building another, which is honest — the
 * state is stored at a different width and "converting" it is a re-instantiation
 * wearing a smaller word.
 *
 * **Why the flag alone does not decide.** A body inside the deterministic
 * island's authority is replayed on rollback, and a replay is only a replay if
 * it reproduces bit for bit (§0.5, §12.1). Two machines agreeing in `double` and
 * disagreeing in `float` is the entire failure mode the determinism rules exist
 * to prevent, so participation in rollback *overrides* the cosmetic flag rather
 * than being weighed against it. The flag can only ever give up precision the
 * simulation was not relying on.
 *
 * **Half-precision storage is not here.** §6.5's second half — `sycl::half` to
 * store and `float` to compute — is measured and kept or dropped in P8, and
 * writing the storage path before the measurement that justifies it would be
 * building the thing the measurement exists to decide on.
 */

#include <cstddef>
#include <cstdint>
#include <memory>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/cooking/soft_body_asset.hpp>
#include <SushiEngine/physics/soft/fem_fracture.hpp>
#include <SushiEngine/physics/soft/finite_element_model.hpp>
#include <SushiEngine/physics/soft/soft_body_model.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /** @brief Which scalar column a soft body is simulated in (§6.5). */
        enum class SoftBodyPrecision
        {
            /** @brief `double` — the column anything gameplay-authoritative uses. */
            Gameplay,

            /**
             * @brief `float` — half the bandwidth, for bodies nothing replays.
             *
             * Bandwidth is what a soft-body solve is actually limited by, so this
             * is close to twice the throughput rather than a marginal saving.
             */
            Cosmetic
        };

        /** @brief What a component says about a body, for @ref resolve_soft_body_precision. */
        struct SoftBodyPrecisionRequest
        {
            /**
             * @brief The component's own "this is decoration" flag.
             *
             * A request, not a decision: it can only give up precision, and only
             * when nothing else needs it.
             */
            bool cosmetic = false;

            /**
             * @brief Whether the deterministic island replays this body.
             *
             * Overrides @ref cosmetic outright. A curtain that a rollback has to
             * reproduce is not a curtain as far as precision is concerned.
             */
            bool participates_in_rollback = false;
        };

        /**
         * @brief Which column a body gets, from its asset and its component flags.
         *
         * The asset's contribution is a veto rather than a vote: a body with no
         * usable cook has nothing to simulate at either width, and answering
         * `Cosmetic` for it would hand a caller a narrower column for a body that
         * is about to be empty.
         *
         * @param view    The cooked asset.
         * @param request What the component asked for.
         * @return The column to instantiate.
         */
        inline SoftBodyPrecision resolve_soft_body_precision(
            const Cooking::SoftBodyAssetView& view,
            const SoftBodyPrecisionRequest& request) noexcept
        {
            if (!view.valid || view.level_count == 0)
                return SoftBodyPrecision::Gameplay;
            if (request.participates_in_rollback)
                return SoftBodyPrecision::Gameplay;
            return request.cosmetic ? SoftBodyPrecision::Cosmetic : SoftBodyPrecision::Gameplay;
        }

        /**
         * @brief One soft body, in whichever column it was instantiated at.
         *
         * The seam that keeps §6.5 from spreading. `ISoftBodyModel<double>` and
         * `ISoftBodyModel<float>` are unrelated types — that is what a template
         * parameter means — so something has to hold one or the other and answer in
         * one language. This does, and it answers in `Scalar`, the engine's
         * boundary type: a renderer, an editor panel or a gameplay query asks the
         * same question of a cosmetic curtain and a gameplay crate and gets the
         * same kind of answer.
         *
         * Non-copyable, because it owns a model.
         */
        class SoftBodyInstance
        {
            public:
                SoftBodyInstance() = default;
                SoftBodyInstance(const SoftBodyInstance&) = delete;
                SoftBodyInstance& operator=(const SoftBodyInstance&) = delete;
                SoftBodyInstance(SoftBodyInstance&&) = default;
                SoftBodyInstance& operator=(SoftBodyInstance&&) = default;

                /**
                 * @brief Instantiates a tetrahedral body from a cooked asset.
                 *
                 * Replaces whatever was here before, which is the only supported way
                 * to change a body's precision.
                 *
                 * @param view      A validated soft-body asset.
                 * @param level     Which simulation level to build (0 is finest).
                 * @param material  The constitutive parameters, in boundary precision.
                 * @param origin    World position of the asset's local origin.
                 * @param precision Which column, from @ref resolve_soft_body_precision.
                 * @return False when the asset or level is unusable; the instance is
                 *         then empty rather than half-built.
                 */
                bool create(const Cooking::SoftBodyAssetView& view, std::uint32_t level,
                            const SoftBodyMaterialT<Scalar>& material, const Vector3& origin,
                            SoftBodyPrecision precision)
                {
                    gameplay_.reset();
                    cosmetic_.reset();
                    precision_ = precision;
                    if (!view.valid || level >= view.level_count)
                        return false;

                    if (precision == SoftBodyPrecision::Cosmetic)
                    {
                        SoftBodyMaterialT<float> narrow;
                        narrow.young_modulus = float(material.young_modulus);
                        narrow.poisson_ratio = float(material.poisson_ratio);
                        narrow.density = float(material.density);
                        narrow.damping = float(material.damping);
                        narrow.yield_stress = float(material.yield_stress);
                        narrow.plastic_creep = float(material.plastic_creep);
                        narrow.maximum_plastic_strain = float(material.maximum_plastic_strain);
                        narrow.fracture_stress = float(material.fracture_stress);
                        cosmetic_.reset(new FiniteElementModel<float>(build_finite_element_model<float>(
                            view, level, narrow,
                            Vector3T<float>{float(origin.x), float(origin.y), float(origin.z)})));
                        return !cosmetic_->particles.empty();
                    }

                    gameplay_.reset(new FiniteElementModel<Scalar>(
                        build_finite_element_model<Scalar>(view, level, material, origin)));
                    return !gameplay_->particles.empty();
                }

                /** @brief Which column this body is simulated in; fixed at @ref create. */
                SoftBodyPrecision precision() const noexcept { return precision_; }

                /** @brief Whether anything was instantiated. */
                bool valid() const noexcept
                {
                    return gameplay_ != nullptr || cosmetic_ != nullptr;
                }

                /**
                 * @brief Advances the body by one tick.
                 * @param dt       The tick's duration, in seconds.
                 * @param substeps How many sub-steps to divide it into.
                 */
                void step(Scalar dt, std::size_t substeps)
                {
                    if (gameplay_ != nullptr)
                        gameplay_->step(dt, substeps);
                    else if (cosmetic_ != nullptr)
                        cosmetic_->step(float(dt), substeps);
                }

                /**
                 * @brief Removes whatever this tick's stress put over `fracture_stress` (§9.5).
                 *
                 * Called once per tick, after the body has stepped — a gameplay-column
                 * body steps through the interleaved `SoftBodyScene`, not through
                 * @ref step, so this is deliberately a separate call rather than the
                 * tail of @ref step, and the owner (`sim/`) is the one that knows
                 * both happened this tick.
                 *
                 * `fracture_budget` and the running total are shared across whichever
                 * column is active; `FemFractureBudget` carries no scalar type of its
                 * own to disagree between them.
                 *
                 * @return Whether any element was actually removed. A caller holding
                 *         raw pointers into `surface_indices`/`surface_vertices` (§9.6's
                 *         colliders) must treat `true` as those pointers now being
                 *         stale — `rebuild_soft_body_surface` replaces both vectors,
                 *         and a replaced `std::vector`'s old `.data()` is not a
                 *         promise, it is a coincidence.
                 */
                bool step_fracture()
                {
                    if (gameplay_ != nullptr)
                        return Physics::apply_fem_fracture(*gameplay_, fracture_budget,
                                                           total_fractured_)
                                   .elements_removed > 0;
                    if (cosmetic_ != nullptr)
                        return Physics::apply_fem_fracture(*cosmetic_, fracture_budget,
                                                           total_fractured_)
                                   .elements_removed > 0;
                    return false;
                }

                /** @brief The deterministic limits this body's fracture pass is held to (§9.5). */
                FemFractureBudget fracture_budget{};

                /** @brief Sets the uniform acceleration every unpinned particle feels. */
                void set_external_acceleration(const Vector3& acceleration) noexcept
                {
                    if (gameplay_ != nullptr)
                        gameplay_->set_external_acceleration(acceleration);
                    else if (cosmetic_ != nullptr)
                        cosmetic_->set_external_acceleration(Vector3T<float>{
                            float(acceleration.x), float(acceleration.y), float(acceleration.z)});
                }

                /** @brief How many particles the body has. */
                std::size_t particle_count() const noexcept
                {
                    if (gameplay_ != nullptr)
                        return gameplay_->particles.size();
                    if (cosmetic_ != nullptr)
                        return cosmetic_->particles.size();
                    return 0;
                }

                /**
                 * @brief Where a particle is, widened to the boundary type.
                 *
                 * Widening at the boundary rather than storing wide is the whole
                 * point: the saving is in what the solve reads and writes every
                 * substep, not in what a caller asks for once a frame.
                 *
                 * @param index The particle; out of range returns the origin.
                 */
                Vector3 particle_position(std::size_t index) const noexcept
                {
                    if (gameplay_ != nullptr && index < gameplay_->particles.size())
                        return gameplay_->particles[index].position;
                    if (cosmetic_ != nullptr && index < cosmetic_->particles.size())
                    {
                        const Vector3T<float>& p = cosmetic_->particles[index].position;
                        return Vector3{Scalar(p.x), Scalar(p.y), Scalar(p.z)};
                    }
                    return Vector3{0, 0, 0};
                }

                /**
                 * @brief The body's surface triangle list, indexing its particles.
                 *
                 * Precision-independent, which is why it is one accessor rather than two:
                 * a topology is the same list of integers at either width, and the whole
                 * reason §6.5 narrows anything is the positions the solve reads and
                 * writes — never the indices, which are not read by the solve at all.
                 *
                 * @return Null when the instance is empty.
                 */
                const std::uint32_t* surface_indices() const noexcept
                {
                    if (gameplay_ != nullptr)
                        return gameplay_->surface_indices.data();
                    if (cosmetic_ != nullptr)
                        return cosmetic_->surface_indices.data();
                    return nullptr;
                }

                /** @brief How many entries @ref surface_indices has; a multiple of three. */
                std::size_t surface_index_count() const noexcept
                {
                    if (gameplay_ != nullptr)
                        return gameplay_->surface_indices.size();
                    if (cosmetic_ != nullptr)
                        return cosmetic_->surface_indices.size();
                    return 0;
                }

                /** @brief How many tetrahedra the body is made of. */
                std::size_t element_count() const noexcept
                {
                    if (gameplay_ != nullptr)
                        return gameplay_->elements.size();
                    if (cosmetic_ != nullptr)
                        return cosmetic_->elements.size();
                    return 0;
                }

                /**
                 * @brief One element's four particles and its two per-tick readouts.
                 *
                 * The pair §9.3 and §9.4 ask for, read together because they are measured
                 * together — `end_tick` computes the stress and then feeds it to the
                 * plasticity, so anything reading one and not the other is reading half of
                 * one measurement.
                 *
                 * @param index    The element; out of range writes nothing and returns false.
                 * @param vertex   Receives the four particle indices.
                 * @param stress   Receives the von Mises stress, in pascals.
                 * @param plastic  Receives the accumulated plastic strain, dimensionless.
                 * @return Whether @p index named an element.
                 */
                bool element_sample(std::size_t index, std::uint32_t vertex[4], Scalar& stress,
                                    Scalar& plastic) const noexcept
                {
                    if (gameplay_ != nullptr && index < gameplay_->elements.size())
                    {
                        const FemTetrahedronT<Scalar>& element = gameplay_->elements[index];
                        for (int i = 0; i < 4; ++i)
                            vertex[i] = element.vertex[i];
                        stress = element.von_mises_stress;
                        plastic = element.accumulated_plastic_strain;
                        return true;
                    }
                    if (cosmetic_ != nullptr && index < cosmetic_->elements.size())
                    {
                        const FemTetrahedronT<float>& element = cosmetic_->elements[index];
                        for (int i = 0; i < 4; ++i)
                            vertex[i] = element.vertex[i];
                        stress = Scalar(element.von_mises_stress);
                        plastic = Scalar(element.accumulated_plastic_strain);
                        return true;
                    }
                    return false;
                }

                /** @brief Sets the contact surface's settings (§9.6), narrowing them if need be. */
                void set_collision(const SoftBodyCollisionSettings<Scalar>& settings) noexcept
                {
                    if (gameplay_ != nullptr)
                        gameplay_->collision = settings;
                    else if (cosmetic_ != nullptr)
                    {
                        SoftBodyCollisionSettings<float> narrow;
                        narrow.thickness = float(settings.thickness);
                        narrow.surface.static_friction = float(settings.surface.static_friction);
                        narrow.surface.dynamic_friction = float(settings.surface.dynamic_friction);
                        narrow.surface.restitution = float(settings.surface.restitution);
                        narrow.surface.density = float(settings.surface.density);
                        narrow.surface.rolling_friction = float(settings.surface.rolling_friction);
                        narrow.surface.spinning_friction =
                            float(settings.surface.spinning_friction);
                        narrow.surface.friction_combine = settings.surface.friction_combine;
                        narrow.surface.restitution_combine = settings.surface.restitution_combine;
                        narrow.self_collision = settings.self_collision;
                        narrow.continuous = settings.continuous;
                        cosmetic_->collision = narrow;
                    }
                }

                /** @brief The largest von Mises stress in the body, from its last tick (§9.3). */
                Scalar maximum_stress() const noexcept
                {
                    if (gameplay_ != nullptr)
                        return gameplay_->maximum_stress();
                    if (cosmetic_ != nullptr)
                        return Scalar(cosmetic_->maximum_stress());
                    return Scalar(0);
                }

                /** @brief The gameplay-column model, or null when this body is cosmetic. */
                FiniteElementModel<Scalar>* gameplay_model() noexcept { return gameplay_.get(); }

                /** @brief The cosmetic-column model, or null when this body is gameplay. */
                FiniteElementModel<float>* cosmetic_model() noexcept { return cosmetic_.get(); }

            private:
                std::unique_ptr<FiniteElementModel<Scalar>> gameplay_;
                std::unique_ptr<FiniteElementModel<float>> cosmetic_;
                SoftBodyPrecision precision_ = SoftBodyPrecision::Gameplay;
                /** @brief §9.5's scene-level cap: how many elements this body has ever lost. */
                std::uint32_t total_fractured_ = 0;
        };
    } // namespace Physics
} // namespace SushiEngine
