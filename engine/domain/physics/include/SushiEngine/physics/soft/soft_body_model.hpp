/**************************************************************************/
/* soft_body_model.hpp                                                    */
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
 * @file soft_body_model.hpp
 * @brief §3.3's `ISoftBodyModel` seam, and the tick schedule its implementations share.
 *
 * Three things simulate a deformable body in this engine, and §9.7 swaps between
 * them at runtime as a body recedes: the tetrahedral `FiniteElementModel`, the
 * `MassSpringModel` lattice, and the `ShapeMatchingModel` that only keeps a
 * body's silhouette. They disagree about exactly one thing — what a substep's
 * internal projection *is* — and agree about everything else: particles are
 * predicted, something is projected, contacts are projected after it, velocities
 * are derived, contact velocities are solved last.
 *
 * So this file carries two types with two different jobs:
 *
 * - @ref ISoftBodyModel is the **seam**. It is what a level-of-detail tier, the
 *   render binding (§8.6) and `ISoftBodyService` (§9.3) hold, and it says only
 *   what a consumer needs: advance me, here is my surface, here is what you may
 *   push me with. Nothing about tetrahedra, springs or covariance matrices
 *   reaches it, which is what makes the swap a substitution (§4.4).
 *
 * - @ref SoftBodyBase is the **shared schedule**, a template method. It owns the
 *   state all three kinds have and the substep loop all three run, and leaves
 *   exactly one hole — @ref SoftBodyBase::project_constraints — for the part
 *   they actually differ in. It is an implementation convenience, not the seam:
 *   a fourth model kind is free to implement `ISoftBodyModel` directly if its
 *   schedule genuinely differs, and the rigid tier of §9.7 does exactly that.
 *
 * The split matters because the alternative — one abstract class holding both —
 * would make every consumer of the seam depend on a substep loop it never calls,
 * and would make the schedule impossible to opt out of.
 */

#include <cstddef>
#include <cstdint>
#include <vector>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/core/rigid_body.hpp>
#include <SushiEngine/physics/soft/soft_body_collision.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief What every deformable body can be asked to do, whatever simulates it.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        class ISoftBodyModel
        {
            public:
                virtual ~ISoftBodyModel() = default;

                /**
                 * @brief Advances the body by one tick.
                 *
                 * Not `noexcept`, because a collider builds this tick's contact set
                 * and a contact set is a container that can fail to grow.
                 *
                 * @param dt       The tick's duration, in seconds.
                 * @param substeps How many sub-steps to divide it into; at least one.
                 */
                virtual void step(T dt, std::size_t substeps) = 0;

                /**
                 * @brief The body's particles and boundary triangles, as they are now.
                 *
                 * Mutable, because both consumers of it write: §9.7's tier change
                 * transfers state into the new model's particles, and §8.6's
                 * embedding reads them in the same tick they were written.
                 *
                 * The view is rebuilt on each call rather than cached, since fracture
                 * (§9.5) moves the particle array in memory and a cached pointer
                 * across that is a use-after-free only a fracturing scene reaches.
                 */
                virtual SoftSurfaceView<T> surface() noexcept = 0;

                /**
                 * @brief Sets the uniform acceleration every unpinned particle feels.
                 * @param acceleration Gravity, usually; in metres per second squared.
                 */
                virtual void set_external_acceleration(const Vector3T<T>& acceleration) noexcept = 0;

                /**
                 * @brief Points the body at what it is touching, or at nothing.
                 * @param collider Borrowed and must outlive the model; null for free flight.
                 */
                virtual void attach_collider(ISoftBodyCollider<T>* collider) noexcept = 0;

                /**
                 * @brief Told that something outside the model rewrote its particles.
                 *
                 * A model may cache things derived from where its particles are — a
                 * rigid tier's frozen shape, a hierarchy's node bounds — and those
                 * caches are correct exactly as long as the model is the only writer.
                 * Three things break that and all three are legitimate: §9.7's
                 * transfer between levels of detail, §9.5's fracture rebuilding the
                 * particle array, and a networked snapshot being restored.
                 *
                 * A hook rather than a rebuild-every-tick, because for every model
                 * that has such a cache the rebuild costs more than a tick's physics,
                 * and for most of them there is nothing to do at all — which is why
                 * it is not pure.
                 *
                 * Not `noexcept`: rebuilding a cache is rebuilding a container.
                 */
                virtual void on_state_replaced() {}

                /**
                 * @brief The largest von Mises stress in the body, from its last tick (§9.3).
                 *
                 * Not pure: a model with no constitutive law has no stress to report,
                 * and forcing it to invent one would be worse than saying zero. A
                 * shape-matching tier reports zero and means it — which is also why
                 * §9.7 keeps a stress heat map on the finest tier only.
                 *
                 * @return Zero for a model that does not compute stress.
                 */
                virtual T maximum_stress() const noexcept { return T(0); }
        };

        /**
         * @brief The state and the substep schedule every particle-based soft body shares.
         *
         * A template method: @ref step fixes the order of the tick and calls
         * @ref project_constraints for the one part a model kind decides. The
         * ordering is not arbitrary and is the reason this is written once —
         *
         * - contacts are found **once per tick** (§6.1), because the pairs a surface
         *   touches barely change over a tick and re-deriving them thirty times is
         *   the single reason a large substep count would be unaffordable;
         * - closing speeds are captured at the **top** of a substep, before the
         *   prediction, because that is the speed restitution is a statement about
         *   and the positional solve is about to remove it (§7.4);
         * - contacts are projected **after** the internal constraints, so a
         *   constraint that pushed a vertex through a floor has it pushed back out
         *   in the same substep rather than the next one;
         * - contact velocities are solved **after** the material's own damping, or a
         *   bounce would be damped away before it happened.
         *
         * Every one of those was a bug the FEM body hit before the order was fixed,
         * and a second model kind reproducing the loop by hand would get its own
         * chance to hit them again.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        class SoftBodyBase : public ISoftBodyModel<T>
        {
            public:
                /** @brief The body's particles; `.position`/`.velocity` are read after @ref step. */
                std::vector<RigidBodyT<T>> particles;

                /**
                 * @brief The body's boundary triangles, three particle indices each.
                 *
                 * The surface is what collides and what the render mesh is embedded
                 * in, so it is carried on the model rather than left in the asset:
                 * once fracture (§9.5) removes an element the asset's copy is no
                 * longer the body's boundary, and the model's is.
                 */
                std::vector<std::uint32_t> surface_indices;

                /**
                 * @brief The particles that lie on @ref surface_indices, ascending and unique.
                 *
                 * Ascending because every collision test in §9.6 walks it, and the
                 * order it is walked in is the order contacts are generated in —
                 * which §12.1 requires to be a function of simulation state rather
                 * than of how the surface happened to be built.
                 */
                std::vector<std::uint32_t> surface_vertices;

                /** @brief How thick this body's surface is and what it is like to touch (§9.6). */
                SoftBodyCollisionSettings<T> collision;

                /**
                 * @brief What this body is touching, or null for a body in free flight.
                 *
                 * The model names the seam and never an implementation, so which of
                 * §9.6's three tests are in play — and against what — is the owner's
                 * statement, not this class's. Borrowed, and must outlive the model.
                 */
                ISoftBodyCollider<T>* collider = nullptr;

                /** @brief Uniform acceleration applied to every unpinned particle (e.g. gravity). */
                Vector3T<T> external_acceleration{Vector3T<T>{T(0), T(0), T(0)}};

                SoftSurfaceView<T> surface() noexcept override
                {
                    SoftSurfaceView<T> view;
                    view.particles = particles.data();
                    view.particle_count = particles.size();
                    view.surface_indices = surface_indices.data();
                    view.index_count = surface_indices.size();
                    view.collision = collision;
                    return view;
                }

                void set_external_acceleration(const Vector3T<T>& acceleration) noexcept override
                {
                    external_acceleration = acceleration;
                }

                void attach_collider(ISoftBodyCollider<T>* value) noexcept override
                {
                    collider = value;
                }

                /**
                 * @brief Advances the body by one tick, in the order described above.
                 *
                 * @param dt       The tick's duration, in seconds.
                 * @param substeps How many sub-steps to divide it into; at least one.
                 */
                void step(T dt, std::size_t substeps) override
                {
                    if (substeps == 0)
                        substeps = 1;
                    const T h = dt / T(substeps);

                    begin_tick(dt);
                    for (std::size_t s = 0; s < substeps; ++s)
                    {
                        capture_contact_velocities();
                        predict_particles(h);
                        project_constraints(h);
                        project_contacts(s, h);
                        derive_velocities(h);
                        solve_contact_velocities(h);
                    }
                    end_tick();
                }

                /**
                 * @brief Builds this tick's contact set, before any substep runs.
                 *
                 * Public rather than protected because `SoftBodyScene` drives several
                 * bodies through the phases in lockstep, which is the only way two
                 * soft bodies in contact can be correct — each would otherwise finish
                 * its whole tick against poses the other has not reached.
                 *
                 * @param dt The whole tick's duration — not a substep's. The collider
                 *           sizes its speculative margin from it, and a substep's
                 *           worth of travel would under-cover the tick by exactly the
                 *           substep count.
                 */
                void begin_tick(T dt)
                {
                    if (collider != nullptr)
                        collider->generate_contacts(particles.data(), particles.size(), dt);
                }

                /** @brief Records every contact's closing speed, at the top of a substep. */
                void capture_contact_velocities() noexcept
                {
                    if (collider != nullptr)
                        collider->capture_velocities(particles.data());
                }

                /**
                 * @brief Integrates every particle forward and stashes the pose it came from.
                 * @param h The substep duration, in seconds.
                 */
                void predict_particles(T h) noexcept
                {
                    for (RigidBodyT<T>& particle : particles)
                        predict(particle, external_acceleration, h);
                }

                /**
                 * @brief One Gauss-Seidel sweep of whatever holds this body together.
                 *
                 * The one hole in the schedule. Implementations must sweep in a fixed
                 * order that depends only on simulation state (§0.5), and must run one
                 * iteration rather than looping to convergence — §0.2's schedule is
                 * many small substeps with one iteration each, and a model that
                 * iterated here would be stiffer at the same authored compliance than
                 * the others, which is exactly the disagreement §4.4's conformance
                 * suite exists to catch.
                 *
                 * @param h The substep duration, in seconds.
                 */
                virtual void project_constraints(T h) = 0;

                /**
                 * @brief Projects the body's contacts, after its internal constraints.
                 * @param substep_index Which substep this is.
                 * @param h             The substep duration, in seconds.
                 */
                void project_contacts(std::size_t substep_index, T h)
                {
                    if (collider != nullptr)
                        collider->project_positions(particles.data(), substep_index, h);
                }

                /**
                 * @brief Derives velocities from the substep's motion and applies damping.
                 * @param h The substep duration, in seconds.
                 */
                void derive_velocities(T h) noexcept
                {
                    const T rate = damping_rate();
                    const T damping_factor =
                        rate > T(0) ? (T(1) - rate * h > T(0) ? T(1) - rate * h : T(0)) : T(1);
                    for (RigidBodyT<T>& particle : particles)
                    {
                        update_velocity(particle, h);
                        if (damping_factor < T(1))
                            particle.velocity = particle.velocity * damping_factor;
                    }
                }

                /**
                 * @brief Dynamic friction and restitution, after the velocities are derived.
                 * @param h The substep duration, in seconds (> 0).
                 */
                void solve_contact_velocities(T h)
                {
                    if (collider != nullptr)
                        collider->solve_velocities(particles.data(), h);
                }

                /**
                 * @brief The once-per-tick readouts a model kind may have; nothing by default.
                 *
                 * Called after the last substep, because the tick's final pose is the
                 * one measurement a stress or a plastic update means anything against.
                 */
                virtual void end_tick() noexcept {}

                /**
                 * @brief How fast this body bleeds off velocity, per second.
                 *
                 * A rate rather than a factor, so it is step-size independent like
                 * everything else here. Asked for rather than stored, because the FEM
                 * body already carries it inside its constitutive material and a
                 * second copy on this class would give the two a chance to disagree.
                 *
                 * @return Zero — no damping — unless an implementation says otherwise.
                 */
                virtual T damping_rate() const noexcept { return T(0); }
        };
    } // namespace Physics
} // namespace SushiEngine
