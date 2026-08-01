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
#include <SushiEngine/physics/constraints/joint.hpp>
#include <SushiEngine/physics/constraints/xpbd_constraint.hpp>
#include <SushiEngine/physics/soft/fem_element.hpp>
#include <SushiEngine/physics/solver/contact_constraint.hpp>

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

            /**
             * @brief A floor under this tick's derived substep count; zero imposes none.
             *
             * Not the substep count — that stays derived, because a caller passing it
             * would make the simulation depend on something outside its own state.
             * What a caller legitimately knows is the *quality* it is willing to pay
             * for: a scene of tall stacks wants a floor under the schedule whatever
             * the motion measure says, because a stack's difficulty is a property of
             * its arrangement and the motion measure only sees speed. So the state
             * may raise the count and the caller may raise the floor, and neither can
             * lower what the other asked for.
             */
            std::size_t substep_floor = 0;
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
                /** @brief The persistent constraint kind this solver admits. */
                using Constraint = XpbdDistanceConstraintT<T>;

                /** @brief The per-tick constraint kind this solver admits (§6.3). */
                using Contact = ContactConstraintT<T>;

                /** @brief The articulated persistent kind this solver admits (§10.1). */
                using Joint = JointConstraintT<T>;

                /**
                 * @brief The deformable persistent kind this solver admits (§9.1).
                 *
                 * The four-body kind, and the reason P6-J1 generalized the colouring
                 * and the store past two endpoints. It is projected twice per substep —
                 * deviatoric then hydrostatic — from one node per colour, and its
                 * particles are ordinary bodies in the same buffer: a soft-body particle
                 * is a `RigidBodyT` with no inertia, which is why an element can push a
                 * crate and a crate can push an element with no coupling code at all.
                 */
                using Element = FemTetrahedronT<T>;

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
                 * @brief Admits a FEM element over four live bodies.
                 *
                 * @param element The element; its `vertex` entries are body slot indices
                 *                and its rest state, Lamé pair and rest volume are read
                 *                as given. Its two multipliers are reset each tick.
                 * @return A handle to it, or an invalid handle when the element capacity
                 *         (`PhysicsCapacities::elements`, zero by default) or the colour
                 *         ceiling is exhausted.
                 */
                virtual ConstraintHandle add_element(const Element& element) = 0;

                /**
                 * @brief Removes an element.
                 * @param handle The element to remove.
                 * @return True when a live element was removed by this call.
                 */
                virtual bool remove_element(ConstraintHandle handle) = 0;

                /**
                 * @brief Reads an element back, including what the solve wrote to it.
                 *
                 * The multipliers and — once §9.3's readout runs on the device — the von
                 * Mises stress are settled where the solve is, so this is the only way a
                 * caller learns what an element carried.
                 *
                 * @param handle  The element to read.
                 * @param element Receives its state; untouched when the handle is dead.
                 * @return True when @p element was written.
                 */
                virtual bool read_element(ConstraintHandle handle, Element& element) const = 0;

                /** @brief How many elements this solver can hold at once. */
                virtual std::size_t element_capacity() const noexcept = 0;

                /**
                 * @brief Admits a joint between two live bodies.
                 *
                 * Its own entry point rather than an overload of @ref add_constraint,
                 * because a joint is a different *kind* in the solver's §6.3 sense: it
                 * takes its colour from the same union but lives in its own band, is
                 * projected by its own node, and is read back every tick for the load
                 * it carried. An overload would hide that a joint costs more than a
                 * distance constraint, which is the one thing a caller sizing a scene
                 * needs to know.
                 *
                 * @param joint The joint; its `a`/`b` are body slot indices, both of
                 *              which must name live bodies. An immovable endpoint is a
                 *              body with zero inverse mass, not a missing one — which
                 *              keeps every joint two-sided and stops a one-sided
                 *              projection existing to disagree with the two-sided one.
                 * @return A handle to it, or an invalid handle when the joint capacity
                 *         or the colour ceiling is exhausted.
                 */
                virtual JointHandle add_joint(const Joint& joint) = 0;

                /**
                 * @brief Removes a joint. What a joint breaking actually does.
                 * @param handle The joint to remove.
                 * @return True when a live joint was removed by this call.
                 */
                virtual bool remove_joint(JointHandle handle) = 0;

                /**
                 * @brief Reads a joint back, including the load the last tick left on it.
                 *
                 * The reason a joint is transferred off the device at all: §10.4's
                 * force recovery happens inside the projection, so the multipliers it
                 * settled on are on the device and a break threshold, a load readout,
                 * and a motor-effort gauge are all the same readback.
                 *
                 * @param handle The joint to read.
                 * @param joint  Receives the joint; untouched when the handle is stale.
                 * @return True when @p handle named a live joint.
                 */
                virtual bool read_joint(JointHandle handle, Joint& joint) const = 0;

                /**
                 * @brief Overwrites a joint — a motor target, a limit, a broken flag.
                 *
                 * The whole descriptor rather than a field at a time, because the
                 * alternative is one virtual per authored parameter and a seam that
                 * grows every time a joint kind gains one. A caller reads, edits, and
                 * writes back.
                 *
                 * @param handle The joint to write.
                 * @param joint  The descriptor to store.
                 * @return True when @p handle named a live joint.
                 */
                virtual bool write_joint(JointHandle handle, const Joint& joint) = 0;

                /** @brief The fixed number of joint slots this solver was built with. */
                virtual std::size_t joint_capacity() const noexcept = 0;

                /**
                 * @brief Discards last tick's contacts and opens a fresh submission.
                 *
                 * Contacts are the one constraint kind with no lifetime past a tick
                 * (§6.3), so they are submitted rather than added: no handle is
                 * returned, nothing is removed, and the set that is not resubmitted
                 * is simply gone. What survives a tick is the manifold, which the
                 * caller keeps — it is keyed by the broadphase pair cache, and that
                 * is what makes warm starting possible.
                 */
                virtual void begin_contacts() = 0;

                /**
                 * @brief Submits one contact for this tick's solve.
                 *
                 * @param contact The contact; its `a`/`b` are body slot indices, and
                 *                `b` may be @ref null_contact_body for static geometry.
                 * @return True when it was placed. False is the contact budget or the
                 *         colour ceiling being exceeded — counted in the statistics,
                 *         and a little penetration rather than a failed tick.
                 */
                virtual bool add_contact(const Contact& contact) = 0;

                /** @brief How many contacts the current submission holds. */
                virtual std::size_t contact_count() const noexcept = 0;

                /**
                 * @brief Reads a solved contact back, by submission order.
                 *
                 * The accumulators the solve settled on are what warm starting
                 * inherits next tick and what a contact event reports as its impulse,
                 * so they have to come back out. Submission order rather than storage
                 * order, because storage order is a colouring — an implementation
                 * detail the caller has no way to predict and no reason to learn.
                 *
                 * @param index   The submission index, `[0, contact_count())`.
                 * @param contact Receives the solved contact; untouched when out of range.
                 * @return True when @p index named a submitted contact.
                 */
                virtual bool read_contact(std::size_t index, Contact& contact) const = 0;

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

                /** @brief The fixed number of contact slots one tick may fill. */
                virtual std::size_t contact_capacity() const noexcept = 0;
        };
    } // namespace Physics
} // namespace SushiEngine
