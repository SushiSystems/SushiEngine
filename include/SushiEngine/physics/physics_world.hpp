/**************************************************************************/
/* physics_world.hpp                                                     */
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
 * @file physics_world.hpp
 * @brief A self-contained XPBD scene: bodies, constraints, and the sub-stepped loop.
 *
 * `PhysicsWorld` is the layer above `XpbdSolver` that turns a one-shot constraint
 * solve into a physics loop: it owns the body buffer, runs predict / solve / derive-
 * velocity for each sub-step, and hands out `RigidBody` state by index. It knows
 * nothing about the ECS on purpose — `physics/` sits below `ecs/` in the engine's
 * layering, so this stays usable standalone (as `examples/xpbd_demo.cpp` does today)
 * and is the seam a later `sim/` component (mapping ECS entities to body indices,
 * syncing Transform/Orientation each frame) will be built on top of, not folded into.
 */

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include <SushiRuntime/SushiRuntime.h>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/rigid_body.hpp>
#include <SushiEngine/physics/xpbd_solver.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /** @brief A stable index into a `PhysicsWorld`'s body array. */
        using BodyId = std::uint32_t;

        /**
         * @brief Owns a set of rigid bodies and constraints and steps them with XPBD.
         *
         * Usage has two phases: register bodies (`add_body`) and constraints
         * (`add_constraint`), call `finalize()` once to upload them and compile the
         * solve graph, then call `step()` every frame. This mirrors how
         * `ConstraintSolver`/`XpbdSolver` themselves are built once and replayed —
         * `finalize()` is that one-time build, `step()` is the replay. Not copyable or
         * movable: `XpbdSolver` keeps a reference to this object's body buffer, so
         * relocating either would leave it dangling.
         *
         * @tparam Constraint A constraint type exposing body indices `a`/`b`, as
         * `XpbdDistanceConstraint` does.
         */
        template <typename Constraint>
        class PhysicsWorld
        {
            public:
                /**
                 * @brief Creates an empty world backed by @p runtime.
                 * @param runtime The runtime that will back the body buffer and solve graph.
                 */
                explicit PhysicsWorld(SushiRuntime::API::Runtime& runtime) noexcept
                    : runtime_(runtime) {}

                PhysicsWorld(const PhysicsWorld&) = delete;
                PhysicsWorld& operator=(const PhysicsWorld&) = delete;
                PhysicsWorld(PhysicsWorld&&) = delete;
                PhysicsWorld& operator=(PhysicsWorld&&) = delete;

                /**
                 * @brief Registers a body before `finalize()`.
                 * @param body The body's initial state.
                 * @return The body's stable index, valid for the world's lifetime.
                 */
                BodyId add_body(const RigidBody& body)
                {
                    pending_bodies_.push_back(body);
                    return BodyId(pending_bodies_.size() - 1);
                }

                /**
                 * @brief Registers a constraint before `finalize()`.
                 * @param constraint The constraint to add.
                 */
                void add_constraint(Constraint constraint)
                {
                    constraints_.push_back(std::move(constraint));
                }

                /**
                 * @brief Uploads the registered bodies and compiles the solve graph.
                 *
                 * Call once, after every `add_body`/`add_constraint` call and before
                 * the first `step()`. Bodies and constraints cannot be added afterward
                 * — this mirrors `XpbdSolver`'s own compile-once-replay-every-frame
                 * structure, which recompiles only if its inputs change.
                 *
                 * @tparam Projection A device-callable projection for @p Constraint.
                 * @param iterations Gauss-Seidel sweeps per sub-step solve.
                 * @param h          The fixed sub-step duration `step()` will use, in seconds.
                 * @param projection The per-constraint projection to apply.
                 */
                template <typename Projection>
                void finalize(std::size_t iterations, Scalar h, Projection projection)
                {
                    h_ = h;
                    bodies_.emplace(runtime_.buffer<RigidBody>(pending_bodies_.size()));
                    for (std::size_t i = 0; i < pending_bodies_.size(); ++i)
                        (*bodies_)[i] = pending_bodies_[i];

                    solver_.emplace(runtime_, *bodies_, constraints_, pending_bodies_.size(),
                                    iterations, h_, projection);
                }

                /**
                 * @brief Advances the world by one fixed outer step, in `step_substeps` sub-steps.
                 *
                 * Each sub-step is XPBD's three phases: predict every body's pose under
                 * @p linear_acceleration (gravity, typically), solve every constraint
                 * once (the compiled Gauss-Seidel sweep), then derive velocity and
                 * angular velocity from how far the solve moved the predicted pose. The
                 * sub-step duration is fixed at `finalize()`'s `h`, so `step_substeps`
                 * must equal the outer step divided by `h` for the two to agree.
                 *
                 * @param linear_acceleration External acceleration applied every sub-step.
                 * @param step_substeps       Number of sub-steps to run this call.
                 */
                void step(Vector3 linear_acceleration, std::size_t step_substeps)
                {
                    for (std::size_t s = 0; s < step_substeps; ++s)
                    {
                        for (std::size_t i = 0; i < body_count(); ++i)
                            predict((*bodies_)[i], linear_acceleration, h_);

                        solver_->solve();

                        for (std::size_t i = 0; i < body_count(); ++i)
                            update_velocity((*bodies_)[i], h_);
                    }
                }

                /** @brief Number of registered bodies. */
                std::size_t body_count() const noexcept { return pending_bodies_.size(); }

                /** @brief Mutable access to a body's current state, by index. */
                RigidBody& body(BodyId id) noexcept { return (*bodies_)[id]; }

                /** @brief Read-only access to a body's current state, by index. */
                const RigidBody& body(BodyId id) const noexcept { return (*bodies_)[id]; }

                /** @brief Number of colours the constraints partitioned into. */
                std::size_t color_count() const noexcept
                {
                    return solver_ ? solver_->color_count() : 0;
                }

                /** @brief The constraint indices grouped by colour, for reference checks. */
                const ColorBatches& colors() const noexcept { return solver_->colors(); }

                /** @brief Times the solve graph has been compiled (1 after warm-up). */
                std::size_t compile_count() const noexcept
                {
                    return solver_ ? solver_->compile_count() : 0;
                }

            private:
                SushiRuntime::API::Runtime& runtime_;
                std::vector<RigidBody> pending_bodies_;
                std::vector<Constraint> constraints_;
                Scalar h_ = 0;
                std::optional<SushiRuntime::API::Buffer<RigidBody>> bodies_;
                std::optional<XpbdSolver<Constraint>> solver_;
        };
    } // namespace Physics
} // namespace SushiEngine
