/**************************************************************************/
/* host_solver.hpp                                                        */
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
 * @file host_solver.hpp
 * @brief The reference `IConstraintSolver`: the same solve, in plain host loops.
 *
 * Not a fallback, and not a simplification. This exists so the runtime-backed solver
 * has something to be *compared against* — §4.4's rule that every seam ships with a
 * shared conformance suite, and the reason a device implementation is allowed to
 * replace a host one without silently changing behaviour.
 *
 * For that comparison to mean anything, this must differ from `RuntimeGraphBuilder`
 * in exactly one respect: how the work is executed. Everything else is deliberately
 * identical, and shared rather than re-derived — the same `ConstraintStore` decides
 * which colour a constraint takes and where in that colour's band it sits, and the
 * same `predict` / `XpbdDistanceProjectionT` / `update_velocity` do the arithmetic.
 * If the layout were re-derived here the suite would be measuring the layout, which
 * is the one thing already known to agree.
 *
 * What that leaves it proving is the claim graph colouring actually makes: that
 * projecting a colour's constraints **in parallel** gives the same answer as
 * projecting them **one after another**. They share no body — that is what a colour
 * is — so it should, and this is where "should" is checked.
 *
 * It names no runtime type, which also makes it the solver a test can build without
 * standing up a device.
 */

#include <cstddef>
#include <cstdint>
#include <vector>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/constraints/distance_projection.hpp>
#include <SushiEngine/physics/constraints/xpbd_constraint.hpp>
#include <SushiEngine/physics/core/body_flags.hpp>
#include <SushiEngine/physics/core/configuration.hpp>
#include <SushiEngine/physics/core/handle.hpp>
#include <SushiEngine/physics/core/rigid_body.hpp>
#include <SushiEngine/physics/core/statistics.hpp>
#include <SushiEngine/physics/solver/constraint_store.hpp>
#include <SushiEngine/physics/solver/solver_interface.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief The reference XPBD solver: same layout, same arithmetic, host loops.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        class HostXpbdSolver final : public IConstraintSolver<T>
        {
            public:
                /** @brief The constraint kind this solver admits. */
                using Constraint = XpbdDistanceConstraintT<T>;

                /**
                 * @brief Creates a solver sized by @p configuration.
                 *
                 * The capacities are honoured even though nothing here would break
                 * without them: a reference implementation that accepted a body the
                 * real one refused would report a difference that is not a
                 * difference in the solve.
                 *
                 * @param configuration The scene's budgets and substep schedule.
                 */
                explicit HostXpbdSolver(const PhysicsConfigurationT<T>& configuration)
                    : configuration_(configuration),
                      body_slots_(configuration.capacities.bodies),
                      constraints_store_(configuration.capacities.bodies,
                                         configuration.capacities.constraints,
                                         configuration.capacities.colors),
                      constraints_(constraints_store_.band_capacity() *
                                   constraints_store_.color_count()),
                      bodies_(configuration.capacities.bodies)
                {
                    // Every slot starts retired, matching the runtime-backed solver:
                    // a slot that is never allocated must contribute to nothing.
                    RigidBodyT<T> retired;
                    retired.flags = BodyFlags::static_body;
                    for (RigidBodyT<T>& body : bodies_)
                        body = retired;
                }

                /** @copydoc IConstraintSolver::add_body */
                BodyHandle add_body(const RigidBodyT<T>& body) override
                {
                    const BodyHandle handle = body_slots_.allocate();
                    if (!handle.valid())
                    {
                        ++statistics_.capacity_overflows;
                        return handle;
                    }
                    bodies_[handle.index] = body;
                    bodies_[handle.index].flags &= ~BodyFlags::sleeping;
                    if (handle.index >= body_high_water_)
                        body_high_water_ = handle.index + 1;
                    return handle;
                }

                /** @copydoc IConstraintSolver::remove_body */
                bool remove_body(BodyHandle handle) override
                {
                    if (!body_slots_.alive(handle))
                        return false;
                    remove_constraints_touching(handle.index);

                    RigidBodyT<T> retired;
                    retired.flags = BodyFlags::static_body;
                    bodies_[handle.index] = retired;
                    return body_slots_.release(handle);
                }

                /** @copydoc IConstraintSolver::add_constraint */
                ConstraintHandle add_constraint(const Constraint& constraint) override
                {
                    const ConstraintPlacement placement =
                        constraints_store_.place(constraint.a, constraint.b);
                    if (!placement.handle.valid())
                    {
                        ++statistics_.capacity_overflows;
                        return placement.handle;
                    }
                    constraints_[placement.slot] = constraint;
                    return placement.handle;
                }

                /** @copydoc IConstraintSolver::remove_constraint */
                bool remove_constraint(ConstraintHandle handle) override
                {
                    const std::size_t slot = constraints_store_.slot_of(handle);
                    if (slot >= constraints_.size())
                        return false;
                    const Constraint removed = constraints_[slot];
                    const ConstraintRemoval removal =
                        constraints_store_.remove(handle, removed.a, removed.b);
                    if (!removal.removed)
                        return false;
                    if (removal.slot != removal.moved_from)
                        constraints_[removal.slot] = constraints_[removal.moved_from];
                    return true;
                }

                /** @copydoc IConstraintSolver::read_body */
                bool read_body(BodyHandle handle, RigidBodyT<T>& body) const override
                {
                    if (!body_slots_.alive(handle))
                        return false;
                    body = bodies_[handle.index];
                    return true;
                }

                /** @copydoc IConstraintSolver::read_bodies */
                void read_bodies(std::size_t first, std::size_t count,
                                 RigidBodyT<T>* bodies) const override
                {
                    if (bodies == nullptr)
                        return;
                    for (std::size_t i = 0; i < count && first + i < bodies_.size(); ++i)
                        bodies[i] = bodies_[first + i];
                }

                /** @copydoc IConstraintSolver::write_body */
                bool write_body(BodyHandle handle, const RigidBodyT<T>& body) override
                {
                    if (!body_slots_.alive(handle))
                        return false;
                    bodies_[handle.index] = body;
                    return true;
                }

                /** @copydoc IConstraintSolver::body_slot */
                std::size_t body_slot(BodyHandle handle) const override
                {
                    return body_slots_.alive(handle) ? std::size_t(handle.index)
                                                     : body_slots_.capacity();
                }

                /**
                 * @brief Advances every live body by one tick, sequentially.
                 *
                 * The order is the runtime-backed solver's order, written out: for
                 * each substep, predict every body, then walk the colours in
                 * ascending order projecting each one's live band in slot order, then
                 * derive every body's velocity. Nothing about that sequence is a host
                 * convenience — it is the schedule the graph encodes.
                 *
                 * @param parameters What the tick is told from outside the simulation.
                 */
                void step(const StepParameters<T>& parameters) override
                {
                    live_substeps_ = derive_substep_count(parameters.delta_time);
                    const T h =
                        parameters.delta_time / T(live_substeps_ > 0 ? live_substeps_ : 1);

                    for (std::size_t substep = 0; substep < live_substeps_; ++substep)
                    {
                        for (std::size_t i = 0; i < body_high_water_; ++i)
                            predict(bodies_[i], parameters.gravity, h);

                        for (std::size_t color = 0;
                             color < constraints_store_.color_count(); ++color)
                        {
                            const std::size_t base = constraints_store_.band_base(color);
                            const std::size_t live = constraints_store_.band_size(color);
                            for (std::size_t offset = 0; offset < live; ++offset)
                            {
                                // A local multiplier starting at zero, exactly as the
                                // graph's kernel does: one iteration per substep, so
                                // nothing carries across.
                                T lambda = T(0);
                                XpbdDistanceProjectionT<T> projection;
                                projection(constraints_[base + offset], bodies_.data(),
                                           lambda, h);
                            }
                        }

                        for (std::size_t i = 0; i < body_high_water_; ++i)
                            update_velocity(bodies_[i], h);
                    }

                    measure_motion();
                    refresh_statistics();
                }

                /** @copydoc IConstraintSolver::statistics */
                const PhysicsStatisticsT<T>& statistics() const noexcept override
                {
                    return statistics_;
                }

                /** @copydoc IConstraintSolver::body_capacity */
                std::size_t body_capacity() const noexcept override
                {
                    return body_slots_.capacity();
                }

                /** @copydoc IConstraintSolver::constraint_capacity */
                std::size_t constraint_capacity() const noexcept override
                {
                    return constraints_store_.capacity();
                }

                /** @brief How many colours the layout was built with. */
                std::size_t color_count() const noexcept
                {
                    return constraints_store_.color_count();
                }

                /** @brief Live constraints in colour @p color. */
                std::size_t color_size(std::size_t color) const noexcept
                {
                    return constraints_store_.band_size(color);
                }

            private:
                /**
                 * @brief The fastest live body's speed, for the next tick's schedule.
                 *
                 * A maximum, so the order it is taken in cannot change the answer —
                 * which is why this plain loop agrees with the runtime's fixed-order
                 * reduction rather than merely approximating it.
                 */
                void measure_motion()
                {
                    motion_maximum_ = T(0);
                    for (std::size_t i = 0; i < body_high_water_; ++i)
                    {
                        if (!is_simulated(bodies_[i].flags))
                            continue;
                        const T speed = length(bodies_[i].velocity);
                        if (speed > motion_maximum_)
                            motion_maximum_ = speed;
                    }
                }

                /**
                 * @brief The substep count for this tick, by the same rule as the graph.
                 *
                 * Duplicated arithmetic rather than a shared helper only because the
                 * two read their motion maximum from different places; the rule
                 * itself is the same, and the conformance suite is what keeps it so.
                 *
                 * @param delta_time The tick's duration, in seconds.
                 * @return A substep count within the schedule's bounds.
                 */
                std::size_t derive_substep_count(T delta_time) const
                {
                    const SubstepSchedule<T>& schedule = configuration_.substeps;
                    const std::size_t minimum = schedule.minimum > 0 ? schedule.minimum : 1;
                    const std::size_t maximum =
                        schedule.maximum > minimum ? schedule.maximum : minimum;

                    if (!(motion_maximum_ > T(0)) || !(schedule.motion_budget > T(0)))
                        return minimum;

                    const T wanted = (motion_maximum_ * delta_time) / schedule.motion_budget;
                    if (!(wanted > T(minimum)))
                        return minimum;

                    const std::size_t derived = std::size_t(wanted) + 1;
                    return derived < maximum ? derived : maximum;
                }

                /** @brief Removes every live constraint naming body slot @p body. */
                void remove_constraints_touching(std::uint32_t body)
                {
                    for (std::size_t color = 0; color < constraints_store_.color_count();
                         ++color)
                    {
                        const std::size_t base = constraints_store_.band_base(color);
                        std::size_t offset = constraints_store_.band_size(color);
                        while (offset > 0)
                        {
                            --offset;
                            const std::size_t slot = base + offset;
                            const Constraint& constraint = constraints_[slot];
                            if (constraint.a != body && constraint.b != body)
                                continue;
                            const ConstraintRemoval removal = constraints_store_.remove(
                                constraints_store_.handle_at(slot), constraint.a,
                                constraint.b);
                            if (removal.removed && removal.slot != removal.moved_from)
                                constraints_[removal.slot] =
                                    constraints_[removal.moved_from];
                        }
                    }
                }

                /** @brief Refreshes the per-tick counters after a step. */
                void refresh_statistics()
                {
                    statistics_.awake_bodies = body_slots_.live_count();
                    statistics_.constraints = constraints_store_.live_count();
                    statistics_.colors = constraints_store_.colors_used();
                    statistics_.substeps = live_substeps_;

                    std::size_t largest = 0;
                    for (std::size_t color = 0; color < constraints_store_.color_count();
                         ++color)
                    {
                        const std::size_t live = constraints_store_.band_size(color);
                        if (live > largest)
                            largest = live;
                    }
                    statistics_.largest_color = largest;

                    // No graph, so nothing is ever compiled or composed. Reported as
                    // zero rather than omitted, because a conformance suite comparing
                    // statistics needs to know which fields are meaningfully
                    // comparable and which are properties of the execution strategy.
                    statistics_.compile_count = 0;
                    statistics_.compose_count = 0;
                }

                PhysicsConfigurationT<T> configuration_;
                HandleTable<BodyTag> body_slots_;
                ConstraintStore constraints_store_;
                std::vector<Constraint> constraints_;
                std::vector<RigidBodyT<T>> bodies_;

                std::size_t body_high_water_ = 0;
                std::size_t live_substeps_ = 1;
                T motion_maximum_ = T(0);
                PhysicsStatisticsT<T> statistics_{};
        };
    } // namespace Physics
} // namespace SushiEngine
