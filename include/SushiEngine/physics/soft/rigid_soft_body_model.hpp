/**************************************************************************/
/* rigid_soft_body_model.hpp                                              */
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
 * @file rigid_soft_body_model.hpp
 * @brief §9.7's floor: a soft body that has stopped being soft.
 *
 * The coarsest tier of all. One rigid body carrying the asset's cooked mass and
 * inertia tensor, and a rest shape it rebuilds its particles from after every
 * step. Nothing deforms; the particles exist only so the tier can still answer
 * `ISoftBodyModel::surface()` and keep the render binding (§8.6) and the
 * collision surface (§9.6) working unchanged.
 *
 * **This one does not derive from `SoftBodyBase`,** and that is the point of the
 * base being optional. Its tick is not "predict every particle, project, derive
 * every particle" — it is "integrate one body" — and forcing it through the
 * particle schedule would mean integrating hundreds of particles individually,
 * then fitting a rigid transform back out of them, to arrive at the answer one
 * `predict` already gives. The seam is `ISoftBodyModel`; the schedule is a
 * convenience three of the four tiers happen to share.
 *
 * A body this far away is worth roughly nothing per tick, which is the whole
 * argument for the tier: §9.7's parked-car case (§13.1's table row) is a body
 * that costs what a rigid body costs because it *is* one.
 */

#include <cstddef>
#include <cstdint>
#include <vector>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/core/rigid_body.hpp>
#include <SushiEngine/physics/soft/soft_body_model.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief A soft body simulated as one rigid body with a frozen shape.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        class RigidSoftBodyModel : public ISoftBodyModel<T>
        {
            public:
                /** @brief The single body actually integrated. */
                RigidBodyT<T> body{};

                /** @brief Where each particle sits in @ref body's local frame. */
                std::vector<Vector3T<T>> local_positions;

                /** @brief The particles, rebuilt from @ref body after every step. */
                std::vector<RigidBodyT<T>> particles;

                /** @brief The body's boundary triangles, three particle indices each. */
                std::vector<std::uint32_t> surface_indices;

                /** @brief How thick this body's surface is and what it is like to touch. */
                SoftBodyCollisionSettings<T> collision;

                /**
                 * @brief Freezes the current particle positions into a rigid shape.
                 *
                 * Places @ref body at the particles' centroid with the identity
                 * orientation and records every particle relative to it, so the tier
                 * begins exactly where the tier it replaced left off — which is what
                 * makes the swap invisible rather than a snap to the asset's pose.
                 *
                 * The centroid is unweighted, matching `ShapeMatchingModel`'s fit: the
                 * two are adjacent tiers and a body crossing between them must not
                 * shift because they disagreed about where its middle is.
                 */
                void freeze_from_particles()
                {
                    const std::size_t count = particles.size();
                    local_positions.resize(count);
                    if (count == 0)
                        return;

                    Vector3T<T> centre{T(0), T(0), T(0)};
                    for (const RigidBodyT<T>& particle : particles)
                        centre = centre + particle.position;
                    centre = centre * (T(1) / T(count));

                    Vector3T<T> velocity{T(0), T(0), T(0)};
                    for (const RigidBodyT<T>& particle : particles)
                        velocity = velocity + particle.velocity;
                    velocity = velocity * (T(1) / T(count));

                    body.position = centre;
                    body.prev_position = centre;
                    body.orientation = QuaternionT<T>{T(0), T(0), T(0), T(1)};
                    body.prev_orientation = body.orientation;
                    body.velocity = velocity;

                    for (std::size_t i = 0; i < count; ++i)
                        local_positions[i] = particles[i].position - centre;
                }

                void step(T dt, std::size_t substeps) override
                {
                    if (substeps == 0)
                        substeps = 1;
                    const T h = dt / T(substeps);
                    for (std::size_t s = 0; s < substeps; ++s)
                    {
                        predict(body, external_acceleration_, h);
                        update_velocity(body, h);
                    }
                    rebuild_particles();
                }

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
                    external_acceleration_ = acceleration;
                }

                /**
                 * @brief Accepted and ignored: a rigid tier has no per-particle contacts.
                 *
                 * Not a silent no-op by accident. A body at this tier collides as the
                 * rigid body it now is, through `physics/scene`'s ordinary pipeline
                 * and its cooked convex pieces — which is cheaper and better than a
                 * per-particle test against a shape that cannot deform in response.
                 * Accepting the call rather than refusing it keeps the caller from
                 * having to ask which tier it is holding, which is the whole point of
                 * the seam.
                 */
                void attach_collider(ISoftBodyCollider<T>*) noexcept override {}

                /**
                 * @brief Re-freezes, because the pose this tier holds *is* the cache.
                 *
                 * Nothing else in the model survives a wholesale particle rewrite:
                 * the rigid body's position, orientation and the local offsets are all
                 * derived from the particles, so a tier handed a new pose without this
                 * would spend the next tick pulling every particle back to where the
                 * old one was.
                 */
                void on_state_replaced() override { freeze_from_particles(); }

            private:
                /** @brief Writes the rigid pose back onto every particle. */
                void rebuild_particles() noexcept
                {
                    const std::size_t count =
                        particles.size() < local_positions.size() ? particles.size()
                                                                  : local_positions.size();
                    for (std::size_t i = 0; i < count; ++i)
                    {
                        RigidBodyT<T>& particle = particles[i];
                        particle.prev_position = particle.position;
                        particle.position =
                            body.position + rotate(body.orientation, local_positions[i]);
                        // The rigid velocity field, so a consumer reading a particle's
                        // velocity — the render extract's motion vectors, an event sink
                        // — sees the body spinning rather than translating uniformly.
                        particle.velocity =
                            body.velocity +
                            cross(body.angular_velocity,
                                  rotate(body.orientation, local_positions[i]));
                    }
                }

                Vector3T<T> external_acceleration_{Vector3T<T>{T(0), T(0), T(0)}};
        };
    } // namespace Physics
} // namespace SushiEngine
