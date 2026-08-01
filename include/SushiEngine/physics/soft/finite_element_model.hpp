/**************************************************************************/
/* finite_element_model.hpp                                              */
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

#pragma once

/**
 * @file finite_element_model.hpp
 * @brief A tetrahedral soft body: the first thing to ever consume `.sushisoft` (§16.15).
 *
 * `FiniteElementModel` owns a flat array of particles (`RigidBodyT`, exactly
 * the point-mass-with-no-rotation particles `soft/cloth.hpp` and the mass-spring
 * `soft/soft_body.hpp` already use) and a flat array of `FemTetrahedronT`
 * elements, and steps them with §9.1's two constraints per element, one
 * Gauss-Seidel sweep per sub-step — the same "many small steps, one iteration
 * each" schedule (§0.2) every other constraint kind in this engine already
 * follows.
 *
 * **This is a host-only reference implementation, not yet a constraint kind
 * inside the shared device solver.** `fem_element.hpp`'s file comment records
 * why: `ConstraintStore`/`IncrementalColoring`/`color_constraints` are written
 * for exactly two bodies per constraint, and a tetrahedron touches four. That
 * generalization is real, separable work, deliberately not taken on inside
 * P6-A — this class is the correct, tested physics first, in the same
 * relationship `HostXpbdSolver` has to `RuntimeGraphBuilder`, except for now
 * it is the only implementation rather than a conformance mirror of one.
 *
 * The element sweep runs in a fixed order (element index, ascending) every
 * substep — deterministic per §0.5 by construction, since nothing here reads
 * from a hash container or a wall clock.
 */

#include <cstddef>
#include <cstdint>
#include <vector>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/core/rigid_body.hpp>
#include <SushiEngine/physics/cooking/soft_body_asset.hpp>
#include <SushiEngine/physics/soft/fem_element.hpp>
#include <SushiEngine/physics/soft/fem_plasticity.hpp>
#include <SushiEngine/physics/soft/fem_projection.hpp>
#include <SushiEngine/physics/soft/fem_stress.hpp>
#include <SushiEngine/physics/soft/soft_body_material.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief A tetrahedral soft body: particles, elements, and the material
         *        every element's constraints read.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        class FiniteElementModel
        {
            public:
                /** @brief The body's particles; `.position`/`.velocity` are read after @ref step. */
                std::vector<RigidBodyT<T>> particles;

                /** @brief The body's tetrahedra. */
                std::vector<FemTetrahedronT<T>> elements;

                /** @brief The constitutive parameters every element's constraints read. */
                SoftBodyMaterialT<T> material;

                /** @brief Uniform acceleration applied to every unpinned particle (e.g. gravity). */
                Vector3T<T> external_acceleration{Vector3T<T>{T(0), T(0), T(0)}};

                /**
                 * @brief Advances the body by one tick.
                 *
                 * @param dt        The tick's duration, in seconds.
                 * @param substeps  How many sub-steps to divide it into; at least one.
                 */
                void step(T dt, std::size_t substeps) noexcept
                {
                    if (substeps == 0)
                        substeps = 1;
                    const T h = dt / T(substeps);
                    const LameParameters<T> lame = lame_parameters(material);
                    const T damping_factor =
                        material.damping > T(0)
                            ? (T(1) - material.damping * h > T(0) ? T(1) - material.damping * h
                                                                  : T(0))
                            : T(1);

                    for (std::size_t s = 0; s < substeps; ++s)
                    {
                        for (RigidBodyT<T>& particle : particles)
                            predict(particle, external_acceleration, h);

                        for (FemTetrahedronT<T>& element : elements)
                        {
                            element.deviatoric_lambda = 0;
                            element.hydrostatic_lambda = 0;
                        }

                        // One Gauss-Seidel sweep, fixed element order: §0.2's schedule
                        // (many sub-steps, one iteration each) and §0.5's determinism
                        // rule, both by construction rather than by discipline.
                        for (FemTetrahedronT<T>& element : elements)
                        {
                            project_fem_deviatoric(particles.data(), element, lame.mu, h);
                            project_fem_hydrostatic(particles.data(), element, lame.mu,
                                                    lame.lambda, h);
                        }

                        for (RigidBodyT<T>& particle : particles)
                        {
                            update_velocity(particle, h);
                            if (damping_factor < T(1))
                                particle.velocity = particle.velocity * damping_factor;
                        }
                    }

                    // §9.3's readout, once per tick rather than once per sub-step:
                    // the tick's final pose is the one measurement of it means
                    // anything against, and `F` costs nothing new to recompute
                    // here since every element's projection already built it
                    // this same tick.
                    for (FemTetrahedronT<T>& element : elements)
                        element.von_mises_stress =
                            tetrahedron_von_mises_stress(particles.data(), element, lame.mu,
                                                         lame.lambda);

                    // §9.4's plasticity, gated on the stress just measured above —
                    // see fem_plasticity.hpp's own header comment for why this runs
                    // once per tick rather than once per sub-step.
                    for (FemTetrahedronT<T>& element : elements)
                        apply_fem_plasticity(particles.data(), element, material);
                }

                /**
                 * @brief The largest von Mises stress across every element.
                 *
                 * The gameplay query §9.3 asks for — `ISoftBodyService::maximum_stress`
                 * arrives once P6-G wires a soft body into `sim/` as an entity; until
                 * then this is the model-level answer that call will forward to.
                 *
                 * @return Zero for a body with no elements.
                 */
                T maximum_stress() const noexcept
                {
                    T worst = 0;
                    for (const FemTetrahedronT<T>& element : elements)
                        if (element.von_mises_stress > worst)
                            worst = element.von_mises_stress;
                    return worst;
                }
        };

        /**
         * @brief Builds a `FiniteElementModel` from one level of a cooked `.sushisoft` asset.
         *
         * The asset's tetrahedra name vertices by *global* index, already rebased
         * to fall inside `[level.first_vertex, level.first_vertex + level.vertex_count)`
         * (`Cooking::SoftBodyAssetView`'s own contract, proven by
         * `Unit_SoftBodyCooker.RebasesEveryLevelsIndicesIntoTheSharedArrays`); this
         * rebases them again, to zero, since the model's own particle array holds
         * only this one level.
         *
         * **Placement is translation only.** An initial world orientation is
         * deliberately not accepted: the cooked `rest_inverse` is expressed in the
         * asset's own local frame, and rotating the particles' *positions* without
         * rotating the rest state they are measured against would read as
         * instantaneous strain at spawn — a body placed on its side would appear
         * to already be under load before a single tick ran. Consistent, correct
         * rotation support is future work behind this same function, not a gap
         * papered over here.
         *
         * @param view      A validated soft-body asset.
         * @param level     Which simulation level to build (0 is finest).
         * @param material  The constitutive parameters every element reads.
         * @param origin    World position of the asset's local origin.
         * @return The model; empty (`particles`/`elements` both empty) if @p view
         *         is invalid or @p level is out of range.
         */
        template <typename T>
        inline FiniteElementModel<T> build_finite_element_model(
            const Cooking::SoftBodyAssetView& view, std::uint32_t level,
            const SoftBodyMaterialT<T>& material, const Vector3T<T>& origin) noexcept
        {
            FiniteElementModel<T> model;
            model.material = material;
            if (!view.valid || level >= view.level_count)
                return model;

            const Cooking::SoftBodyLevelRecord& record = view.levels[level];
            model.particles.resize(record.vertex_count);
            for (std::uint32_t i = 0; i < record.vertex_count; ++i)
            {
                const Vector3& source = view.vertices[record.first_vertex + i];
                RigidBodyT<T>& particle = model.particles[i];
                particle.position =
                    origin + Vector3T<T>{T(source.x), T(source.y), T(source.z)};
                particle.prev_position = particle.position;
                particle.orientation = QuaternionT<T>{T(0), T(0), T(0), T(1)};
                particle.prev_orientation = particle.orientation;
                particle.inv_inertia = Vector3T<T>{T(0), T(0), T(0)};
                const Scalar mass = view.vertex_mass[record.first_vertex + i];
                particle.inv_mass = mass > Scalar(0) ? T(1) / T(mass) : T(0);
            }

            model.elements.resize(record.tetrahedron_count);
            for (std::uint32_t t = 0; t < record.tetrahedron_count; ++t)
            {
                const std::uint32_t* source_vertex =
                    view.tetrahedra + std::size_t(record.first_tetrahedron + t) * 4;
                const Vector3* source_inverse =
                    view.rest_inverse + std::size_t(record.first_tetrahedron + t) * 3;

                FemTetrahedronT<T>& element = model.elements[t];
                for (int corner = 0; corner < 4; ++corner)
                    element.vertex[corner] = source_vertex[corner] - record.first_vertex;

                element.rest_inverse_column_0 =
                    Vector3T<T>{T(source_inverse[0].x), T(source_inverse[0].y),
                               T(source_inverse[0].z)};
                element.rest_inverse_column_1 =
                    Vector3T<T>{T(source_inverse[1].x), T(source_inverse[1].y),
                               T(source_inverse[1].z)};
                element.rest_inverse_column_2 =
                    Vector3T<T>{T(source_inverse[2].x), T(source_inverse[2].y),
                               T(source_inverse[2].z)};
                // No plastic deformation yet: the plastic rest state starts equal
                // to the elastic one, exactly as fem_element.hpp's own comment
                // says it must (§9.4 only ever writes these, never the originals).
                element.plastic_inverse_column_0 = element.rest_inverse_column_0;
                element.plastic_inverse_column_1 = element.rest_inverse_column_1;
                element.plastic_inverse_column_2 = element.rest_inverse_column_2;

                element.rest_volume = T(view.rest_volume[record.first_tetrahedron + t]);
            }

            return model;
        }
    } // namespace Physics
} // namespace SushiEngine
