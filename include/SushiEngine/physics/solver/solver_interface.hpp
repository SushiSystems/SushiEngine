/**************************************************************************/
/* solver_interface.hpp                                                   */
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
 * @file solver_interface.hpp
 * @brief The seam a constraint solver is reached through, and nothing else.
 *
 * This header is the reason `physics/` can name a solver without naming
 * SushiRuntime. It declares what a solver *does* — admit bodies and constraints,
 * advance them, and report what happened — with no mention of a buffer, a graph, or
 * a device. The runtime-backed implementation lives in `runtime_graph_builder.hpp`,
 * which is the one file in the physics layer allowed to include a runtime header;
 * a host reference implementation lives beside it and is what the conformance suite
 * measures the device one against (§4.4).
 *
 * Two things about the shape of this interface are deliberate.
 *
 * State crosses it **by value, one body at a time**, through @ref read_body and
 * @ref write_body rather than through a span. A device-resident implementation has
 * no host-addressable body array to hand out — `operator[]` on a device-resident
 * handle throws — so an interface that returned a pointer would be an interface only
 * a host implementation could satisfy, which is not a seam at all.
 *
 * The substep count is *derived*, not passed. It is a function of simulation state
 * (§6.2), and letting a caller pass it would make the simulation depend on
 * something outside its own state — the one thing determinism does not allow.
 * @ref StepParameters carries what is genuinely external: how much time passed, and
 * what the world is pulling on.
 */

#include <cstddef>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/core/handle.hpp>
#include <SushiEngine/physics/core/rigid_body.hpp>
#include <SushiEngine/physics/core/statistics.hpp>
#include <SushiEngine/physics/constraints/xpbd_constraint.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief What one tick is told from outside the simulation.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        struct StepParameters
        {
            /** @brief The tick's duration in seconds; divided into the derived substeps. */
            T delta_time = T(1) / T(60);

            /**
             * @brief Uniform external acceleration applied to every body.
             *
             * Uniform because that is what a solver can apply without knowing where
             * a body is. A non-uniform field (planetary gravity, wind) is sampled per
             * body by the scene above and folded in there, which is where the
             * sampler already lives.
             */
            Vector3T<T> gravity{T(0), T(-9.81), T(0)};
        };

        /**
         * @brief A constraint solver: what exists, and how it advances.
         *
         * Implementations are substitutable in the Liskov sense and are held to it by
         * a shared conformance suite: given the same bodies, constraints, and step
         * parameters, every implementation must agree on the resulting state within
         * a stated tolerance.
         *
         * @tparam T The scalar element type the solver runs in.
         */
        template <typename T>
        class IConstraintSolver
        {
            public:
                /** @brief The constraint kind this solver admits. */
                using Constraint = XpbdDistanceConstraintT<T>;

                virtual ~IConstraintSolver() = default;

                /**
                 * @brief Admits a body, valid immediately with no finalize step.
                 *
                 * @param body The body's initial state.
                 * @return A handle to it, or an invalid handle when the body capacity
                 *         is exhausted — which is a budget being exceeded, not an
                 *         error, and is counted in the statistics.
                 */
                virtual BodyHandle add_body(const RigidBodyT<T>& body) = 0;

                /**
                 * @brief Removes a body and every constraint attached to it.
                 *
                 * Removing the constraints too is not a convenience: a constraint
                 * naming a slot that has since been reused would silently act on
                 * whatever body took the slot.
                 *
                 * @param handle The body to remove.
                 * @return True when a live body was removed by this call.
                 */
                virtual bool remove_body(BodyHandle handle) = 0;

                /**
                 * @brief Admits a constraint between two live bodies.
                 *
                 * @param constraint The constraint; its `a`/`b` are body slot indices.
                 * @return A handle to it, or an invalid handle when the constraint
                 *         capacity or the colour ceiling is exhausted.
                 */
                virtual ConstraintHandle add_constraint(const Constraint& constraint) = 0;

                /**
                 * @brief Removes a constraint.
                 * @param handle The constraint to remove.
                 * @return True when a live constraint was removed by this call.
                 */
                virtual bool remove_constraint(ConstraintHandle handle) = 0;

                /**
                 * @brief Reads a body's current state out of the solver.
                 * @param handle The body to read.
                 * @param body   Receives the state; untouched when the handle is stale.
                 * @return True when @p handle named a live body.
                 */
                virtual bool read_body(BodyHandle handle, RigidBodyT<T>& body) const = 0;

                /**
                 * @brief Reads a run of body slots in one transfer.
                 *
                 * The bulk form exists because a device-resident implementation pays
                 * a queue round trip per @ref read_body call, and the per-frame job
                 * of copying every body's pose out to the ECS would otherwise be a
                 * thousand round trips. Slots outside the live set are still
                 * written, carrying whatever the solver holds for them.
                 *
                 * @param first The first slot to read.
                 * @param count How many consecutive slots to read.
                 * @param bodies Receives @p count bodies; must have room for them.
                 */
                virtual void read_bodies(std::size_t first, std::size_t count,
                                         RigidBodyT<T>* bodies) const = 0;

                /**
                 * @brief Overwrites a body's state — a teleport, or an authored change.
                 * @param handle The body to write.
                 * @param body   The state to store.
                 * @return True when @p handle named a live body.
                 */
                virtual bool write_body(BodyHandle handle, const RigidBodyT<T>& body) = 0;

                /**
                 * @brief The slot index @p handle addresses, for naming it in a constraint.
                 * @param handle The body to resolve.
                 * @return The slot index, or a value at or past @ref body_capacity when
                 *         the handle is stale.
                 */
                virtual std::size_t body_slot(BodyHandle handle) const = 0;

                /**
                 * @brief Advances every live body by one tick.
                 * @param parameters What the tick is told from outside the simulation.
                 */
                virtual void step(const StepParameters<T>& parameters) = 0;

                /** @brief What the last @ref step did, and the cumulative counters. */
                virtual const PhysicsStatisticsT<T>& statistics() const noexcept = 0;

                /** @brief The fixed number of body slots this solver was built with. */
                virtual std::size_t body_capacity() const noexcept = 0;

                /** @brief The fixed number of constraint slots this solver was built with. */
                virtual std::size_t constraint_capacity() const noexcept = 0;
        };
    } // namespace Physics
} // namespace SushiEngine
