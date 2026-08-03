/**************************************************************************/
/* soft_body_half_storage.hpp                                            */
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
 * @file soft_body_half_storage.hpp
 * @brief §6.5's second half: `sycl::half` for a cosmetic body's *stored* pose.
 *
 * `soft_body_instance.hpp` builds the first half — a cosmetic body's whole
 * projection runs in `float`, half the bandwidth of the gameplay column. This
 * file is the second and narrower half the same section describes: **the
 * particles a cosmetic body carries between ticks may be held at
 * `sycl::half`, provided every arithmetic step still happens in `float`.** A
 * neo-Hookean projection evaluated in eleven significant bits is not a
 * stability trade, it is a broken solve — so nothing here does arithmetic in
 * `half` at all. It only narrows a settled `float` pose for storage and
 * widens it back out before the next tick's compute touches it.
 *
 * That is the whole rule, and it is why there are exactly two functions
 * beneath the storage type rather than a templated arithmetic layer:
 *
 * - @ref widen_half_vector3 — the one place storage becomes something a
 *   projection may read.
 * - @ref narrow_to_half_vector3 — the one place a projection's result becomes
 *   something storage may hold.
 *
 * @ref SoftBodyHalfStorage calls exactly these two functions, at exactly the
 * two points storage meets compute: @ref SoftBodyHalfStorage::widen_into
 * before a tick's `FiniteElementModel<float>::step` runs, and
 * @ref SoftBodyHalfStorage::narrow_from after it finishes. Nothing else in
 * this file, or in the projection it wraps, ever sees a `sycl::half`.
 *
 * **This is the measurement path, not a verdict.** §6.5 states plainly that
 * whether this is worth the narrowing is measured and kept or dropped in P8;
 * `samples/physics/soft_body_half_storage_budget.cpp` is that measurement. Until it
 * is read, this type is additive — nothing in `SoftBodyInstance` or
 * `SoftBodyPrecision` constructs one, so a body's precision selection is
 * unchanged by this file's existence.
 */

#include <cstddef>
#include <vector>

#include <sycl/sycl.hpp>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/soft/finite_element_model.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief A `Vector3T<float>`'s storage footprint, halved.
         *
         * Three `sycl::half` lanes, nothing else — the type exists so a caller
         * can hold an array of these rather than an array of `Vector3T<float>`
         * and get half the bytes moved for it. It carries no arithmetic of its
         * own on purpose: a type that could be added or scaled would invite
         * doing so at eleven significant bits, which is the exact mistake §6.5
         * rules out.
         */
        struct HalfVector3
        {
            sycl::half x{0.0f};
            sycl::half y{0.0f};
            sycl::half z{0.0f};
        };

        /**
         * @brief Widens a stored half-precision vector to the `float` compute column.
         *
         * Exact: every `sycl::half` value is exactly representable in `float`,
         * so this step loses nothing beyond what @ref narrow_to_half_vector3
         * already rounded away when the value was stored.
         *
         * @param stored The half-precision vector read from storage.
         * @return The same value, widened to `float`.
         */
        inline Vector3T<float> widen_half_vector3(const HalfVector3& stored) noexcept
        {
            return Vector3T<float>{float(stored.x), float(stored.y), float(stored.z)};
        }

        /**
         * @brief Narrows a `float` compute-column vector to half-precision storage.
         *
         * Rounds each component to the nearest representable `sycl::half` —
         * a relative error of at most one part in 2^11, the width of a half's
         * mantissa plus its implicit leading bit. Called once a tick, after the
         * projection that produced @p computed has already finished, never
         * from inside it.
         *
         * @param computed The `float` value a tick's compute settled on.
         * @return The value, rounded to half precision for storage.
         */
        inline HalfVector3 narrow_to_half_vector3(const Vector3T<float>& computed) noexcept
        {
            return HalfVector3{sycl::half(computed.x), sycl::half(computed.y),
                               sycl::half(computed.z)};
        }

        /**
         * @brief A cosmetic soft body's position and velocity column, held at half width.
         *
         * Mirrors a `FiniteElementModel<float>`'s particle array at half the
         * storage cost, and only ever touches it at the two seams §6.5 draws:
         * @ref widen_into before the model's own substep loop runs, and
         * @ref narrow_from once it has finished. Everything the substep loop
         * itself does — prediction, the deviatoric and hydrostatic projections,
         * contact resolution, velocity derivation — reads and writes the
         * model's own `float` particles exactly as it does without this class
         * in the picture; this class never appears inside a substep.
         *
         * Orientation, inverse mass, and every other `RigidBodyT` field stay in
         * the model at `float` — this type is deliberately narrow to position
         * and velocity, the two quantities §6.5 names, not a general half-width
         * particle.
         */
        class SoftBodyHalfStorage
        {
            public:
                /**
                 * @brief Captures a model's current pose into half-precision storage.
                 *
                 * Resizes to @p model's particle count, so a body whose particle
                 * array has not yet been captured — or one that changed size,
                 * as fracture (§9.5) does — is handled by the same call rather
                 * than needing a separate first-use path.
                 *
                 * @param model The compute-column model whose pose this storage mirrors.
                 */
                void narrow_from(const FiniteElementModel<float>& model)
                {
                    const std::size_t count = model.particles.size();
                    positions_.resize(count);
                    velocities_.resize(count);
                    for (std::size_t i = 0; i < count; ++i)
                    {
                        positions_[i] = narrow_to_half_vector3(model.particles[i].position);
                        velocities_[i] = narrow_to_half_vector3(model.particles[i].velocity);
                    }
                }

                /**
                 * @brief Widens this storage's pose back into a model, before it steps.
                 *
                 * Writes only `position` and `velocity` on each particle; every
                 * other field — orientation, inverse mass, flags, the material
                 * — is left as the model already has it, since none of those
                 * are what this storage narrowed.
                 *
                 * @param model The compute-column model to overwrite. If its particle
                 *              count differs from @ref particle_count, the shorter of
                 *              the two bounds the write rather than the call failing —
                 *              a mismatch means a caller resized the model without
                 *              re-narrowing, which is a caller bug this makes visible
                 *              (the model comes back partially widened) rather than
                 *              one that reads out of bounds.
                 */
                void widen_into(FiniteElementModel<float>& model) const noexcept
                {
                    const std::size_t count = positions_.size() < model.particles.size()
                                                   ? positions_.size()
                                                   : model.particles.size();
                    for (std::size_t i = 0; i < count; ++i)
                    {
                        model.particles[i].position = widen_half_vector3(positions_[i]);
                        model.particles[i].velocity = widen_half_vector3(velocities_[i]);
                    }
                }

                /** @brief How many particles this storage holds a pose for. */
                std::size_t particle_count() const noexcept { return positions_.size(); }

                /** @brief The stored, half-precision positions; read-only. */
                const std::vector<HalfVector3>& positions() const noexcept { return positions_; }

                /** @brief The stored, half-precision velocities; read-only. */
                const std::vector<HalfVector3>& velocities() const noexcept { return velocities_; }

            private:
                std::vector<HalfVector3> positions_;
                std::vector<HalfVector3> velocities_;
        };
    } // namespace Physics
} // namespace SushiEngine
