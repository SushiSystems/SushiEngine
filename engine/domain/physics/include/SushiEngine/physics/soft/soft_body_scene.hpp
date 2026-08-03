/**************************************************************************/
/* soft_body_scene.hpp                                                    */
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
 * @file soft_body_scene.hpp
 * @brief Several soft bodies advanced together, which is what a contact between
 *        two of them requires.
 *
 * `SoftBodyBase::step` advances one body through every substep of a tick
 * before returning. Two bodies stepped that way cannot touch each other
 * correctly no matter how good the contact code is: the first would finish its
 * whole tick against poses the second has not reached yet, and the second would
 * then answer contacts against a partner that has already arrived. The
 * substeps have to interleave, and something has to own the interleaving.
 *
 * That is all this class is. It holds no physics of its own — every phase it
 * calls is a method the model already exposes and already uses for the
 * single-body case — and it adds exactly one thing: the ordering across bodies.
 * Within a substep every body predicts, then every body projects its own
 * constraints, then every body projects its own contacts, then the pair contacts
 * project, and only then does anything derive a velocity. A pair contact
 * projecting before both its bodies' constraints have run would be answering a
 * pose that is about to change.
 *
 * The bodies are held as `SoftBodyBase`, not as any one model kind: the schedule
 * is the same for a tetrahedral body, a spring lattice and a shape-matched
 * silhouette, so a scene that named one of them would be refusing to step the
 * other two for no reason (§9.7).
 *
 * The models and colliders are **borrowed**. A scene is a schedule, not an
 * owner: the same models are reached by the render extract, the editor's
 * inspector and the cooker's cache, and giving one of those five the
 * lifetime is what makes the other four hold pointers they cannot validate.
 */

#include <cstddef>
#include <vector>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/soft/soft_body_collision.hpp>
#include <SushiEngine/physics/soft/soft_body_model.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief A set of soft bodies and the contacts between them, stepped as one.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        class SoftBodyScene
        {
            public:
                /**
                 * @brief Admits a body to the scene.
                 *
                 * @param model The body; borrowed, must outlive the scene, ignored when null.
                 */
                void add_body(SoftBodyBase<T>* model)
                {
                    if (model != nullptr)
                        bodies_.push_back(model);
                }

                /**
                 * @brief Admits a collider acting between two of the scene's bodies.
                 *
                 * @param collider The pair collider; borrowed, already built, ignored when null.
                 */
                void add_pair_collider(ISoftBodyPairCollider<T>* collider)
                {
                    if (collider != nullptr)
                        pairs_.push_back(collider);
                }

                /** @brief Forgets every body and pair collider, destroying nothing. */
                void clear() noexcept
                {
                    bodies_.clear();
                    pairs_.clear();
                }

                /** @brief How many bodies the scene advances. */
                std::size_t body_count() const noexcept
                {
                    return bodies_.size();
                }

                /** @brief How many pair colliders act between them. */
                std::size_t pair_count() const noexcept
                {
                    return pairs_.size();
                }

                /**
                 * @brief Advances every body by one tick, in lockstep.
                 *
                 * @param dt       The tick's duration, in seconds.
                 * @param substeps How many substeps to divide it into; at least one.
                 */
                void step(T dt, std::size_t substeps)
                {
                    if (substeps == 0)
                        substeps = 1;
                    const T h = dt / T(substeps);

                    for (SoftBodyBase<T>* body : bodies_)
                        body->begin_tick(dt);
                    for (ISoftBodyPairCollider<T>* pair : pairs_)
                        pair->generate_contacts(dt);

                    for (std::size_t s = 0; s < substeps; ++s)
                    {
                        for (SoftBodyBase<T>* body : bodies_)
                            body->capture_contact_velocities();
                        for (ISoftBodyPairCollider<T>* pair : pairs_)
                            pair->capture_velocities();

                        for (SoftBodyBase<T>* body : bodies_)
                            body->predict_particles(h);
                        for (SoftBodyBase<T>* body : bodies_)
                            body->project_constraints(h);

                        for (SoftBodyBase<T>* body : bodies_)
                            body->project_contacts(s, h);
                        for (ISoftBodyPairCollider<T>* pair : pairs_)
                            pair->project_positions(s, h);

                        for (SoftBodyBase<T>* body : bodies_)
                            body->derive_velocities(h);

                        for (SoftBodyBase<T>* body : bodies_)
                            body->solve_contact_velocities(h);
                        for (ISoftBodyPairCollider<T>* pair : pairs_)
                            pair->solve_velocities(h);
                    }

                    for (SoftBodyBase<T>* body : bodies_)
                        body->end_tick();
                }

            private:
                std::vector<SoftBodyBase<T>*> bodies_;
                std::vector<ISoftBodyPairCollider<T>*> pairs_;
        };
    } // namespace Physics
} // namespace SushiEngine
