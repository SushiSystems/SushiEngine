/**************************************************************************/
/* runtime_graph_builder.hpp                                              */
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
 * @file runtime_graph_builder.hpp
 * @brief The one file in the physics layer that names SushiRuntime.
 *
 * Every `Buffer`, `Graph`, `Dynamic`, `Reads`/`Writes` and `Extent` the physics uses
 * lives inside this class. Nothing under `physics/` includes a runtime header
 * except this one, so when the runtime's API moves — and it is explicitly unstable —
 * one file moves with it and the conformance suite proves the behaviour did not.
 * That is not a preference invented here; it is the codebase restoring a decision it
 * had drifted from, since `physics_world.hpp` and `xpbd_solver.hpp` both include
 * `SushiRuntime.h` today.
 *
 * ### The shape of the graph, and why
 *
 * The whole tick is **one** composition and **one** `run()`. A substep loop driven
 * from the host would be one blocking round trip per substep, and at 32 substeps and
 * 60 Hz that latency is larger than everything else in the system put together. So
 * the graph is built once, for the *maximum* substep count, as:
 *
 *     for each substep s:
 *         predict          (per body)
 *         project colour 0 (per constraint in colour 0)
 *         project colour 1
 *         ...
 *         derive velocity  (per body)
 *     measure motion       (per body, once)
 *     reduce the maximum   (fixed order)
 *
 * Every one of those nodes is late-bound. `sized()` supplies its live element count
 * each step and `when()` switches it off entirely, so the *counts* change every tick
 * without the *structure* changing — and structure is the only thing recomposition
 * costs. A `compose_count()` that climbs every tick is the bug this design exists to
 * make impossible, and a test asserts it does not.
 *
 * ### Why the colours serialize, deliberately
 *
 * Every projection node writes the body buffer, so the dependency tracker orders the
 * colours one after another. That is exactly right: colours exist *because*
 * consecutive ones share bodies. The parallelism is inside a node, across the
 * constraints of one colour, which is where graph colouring puts it.
 *
 * ### Where the data lives
 *
 * The hot columns are device-resident. That is not an optimization applied to a
 * working host loop — it is what removes the host loop. A device-resident handle
 * throws on `operator[]`, so there is no way to accidentally keep a per-body host
 * pass alive; the host reaches state through `read_range`/`write_range`, in bulk,
 * at the tick boundary and nowhere else.
 */

#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include <SushiRuntime/SushiRuntime.h>

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
         * @brief The per-step values a kernel reads instead of capturing.
         *
         * A kernel captures by value when the graph is *built*, so anything that
         * varies per tick cannot be a capture. One shared-residency element holds
         * them and every node reads it; writing it is a host store, not a transfer.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        struct StepUniforms
        {
            /** @brief Uniform external acceleration for this tick. */
            Vector3T<T> gravity{T(0), T(-9.81), T(0)};

            /** @brief The substep duration: the tick's delta time over its substep count. */
            T substep_duration = T(1) / T(240);
        };

        /**
         * @brief A runtime-backed XPBD solver over a mutable set of bodies and constraints.
         *
         * @tparam T The scalar element type; `double` for anything a rollback replays.
         */
        template <typename T>
        class RuntimeGraphBuilder final : public IConstraintSolver<T>
        {
            public:
                /** @brief The constraint kind this solver admits. */
                using Constraint = XpbdDistanceConstraintT<T>;

                /**
                 * @brief Allocates every buffer at capacity and compiles the solve graph.
                 *
                 * Nothing here is deferred to a `finalize()`. The buffers are sized
                 * from @p configuration and never resized — a `Buffer` cannot grow in
                 * place, and a reallocation would invalidate the raw pointer every
                 * compiled node captured — so the graph can be built immediately and
                 * stays valid for every body and constraint that will ever be added.
                 *
                 * @param runtime       The runtime backing the buffers and the graph.
                 * @param configuration The scene's budgets and substep schedule.
                 */
                RuntimeGraphBuilder(SushiRuntime::API::Runtime& runtime,
                                    const PhysicsConfigurationT<T>& configuration)
                    : runtime_(runtime),
                      configuration_(configuration),
                      body_slots_(configuration.capacities.bodies),
                      constraints_store_(configuration.capacities.bodies,
                                         configuration.capacities.constraints,
                                         configuration.capacities.colors),
                      constraint_mirror_(constraints_store_.band_capacity() *
                                         constraints_store_.color_count()),
                      body_mirror_(configuration.capacities.bodies)
                {
                    // The rebalancer migrates tasks mid-run on a millisecond
                    // heartbeat. For a fixed-rate tick the cost is jitter, and jitter
                    // is the one thing a fixed-rate tick cannot absorb.
                    runtime_.rebalancer(configuration_.rebalancer);

                    allocate_buffers();
                    build_graph();
                }

                RuntimeGraphBuilder(const RuntimeGraphBuilder&) = delete;
                RuntimeGraphBuilder& operator=(const RuntimeGraphBuilder&) = delete;
                RuntimeGraphBuilder(RuntimeGraphBuilder&&) = delete;
                RuntimeGraphBuilder& operator=(RuntimeGraphBuilder&&) = delete;

                /** @copydoc IConstraintSolver::add_body */
                BodyHandle add_body(const RigidBodyT<T>& body) override
                {
                    const BodyHandle handle = body_slots_.allocate();
                    if (!handle.valid())
                    {
                        ++statistics_.capacity_overflows;
                        return handle;
                    }

                    RigidBodyT<T> stored = body;
                    // A body's flag word says whether it is simulated; a fresh body
                    // must not inherit the "retired" marking left in a reused slot.
                    stored.flags &= ~BodyFlags::sleeping;
                    stage_body(handle.index, stored);

                    if (handle.index >= body_high_water_)
                        body_high_water_ = handle.index + 1;
                    return handle;
                }

                /** @copydoc IConstraintSolver::remove_body */
                bool remove_body(BodyHandle handle) override
                {
                    if (!body_slots_.alive(handle))
                        return false;

                    // Every constraint touching this body goes with it. A constraint
                    // left naming a freed slot would act on whichever body claims
                    // that slot next, which is a corruption with no symptom.
                    remove_constraints_touching(handle.index);

                    // Retiring the slot rather than compacting: constraints address
                    // bodies by slot index, so moving a body would rewrite every
                    // constraint that names it. A retired slot is marked static, and
                    // the predict and derive kernels skip it by that flag alone.
                    RigidBodyT<T> retired;
                    retired.flags = BodyFlags::static_body;
                    stage_body(handle.index, retired);

                    // The motion measure is reduced over the whole capacity, so a
                    // retired slot must stop contributing or a body removed while
                    // moving fast would pin the substep count at its maximum for
                    // ever. Written straight through: removal is rare, and deferring
                    // it would mean tracking which slots are stale.
                    const T still = T(0);
                    motion_->write_range(
                        SushiRuntime::API::ElementRange{handle.index, 1}, &still);

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
                    stage_constraint(placement.slot, constraint);
                    return placement.handle;
                }

                /** @copydoc IConstraintSolver::remove_constraint */
                bool remove_constraint(ConstraintHandle handle) override
                {
                    if (!constraints_store_.alive(handle))
                        return false;
                    remove_constraint_unchecked(handle);
                    return true;
                }

                /** @copydoc IConstraintSolver::read_body */
                bool read_body(BodyHandle handle, RigidBodyT<T>& body) const override
                {
                    if (!body_slots_.alive(handle))
                        return false;
                    read_bodies(handle.index, 1, &body);
                    return true;
                }

                /** @copydoc IConstraintSolver::read_bodies */
                void read_bodies(std::size_t first, std::size_t count,
                                 RigidBodyT<T>* bodies) const override
                {
                    if (count == 0 || bodies == nullptr)
                        return;
                    const std::vector<RigidBodyT<T>> range = bodies_->read_range(
                        SushiRuntime::API::ElementRange{first, count});
                    for (std::size_t i = 0; i < count && i < range.size(); ++i)
                        bodies[i] = range[i];
                }

                /** @copydoc IConstraintSolver::write_body */
                bool write_body(BodyHandle handle, const RigidBodyT<T>& body) override
                {
                    if (!body_slots_.alive(handle))
                        return false;
                    stage_body(handle.index, body);
                    return true;
                }

                /** @copydoc IConstraintSolver::body_slot */
                std::size_t body_slot(BodyHandle handle) const override
                {
                    return body_slots_.alive(handle) ? std::size_t(handle.index)
                                                     : body_slots_.capacity();
                }

                /** @copydoc IConstraintSolver::step */
                void step(const StepParameters<T>& parameters) override
                {
                    live_substeps_ = derive_substep_count(parameters.delta_time);

                    StepUniforms<T> uniforms;
                    uniforms.gravity = parameters.gravity;
                    uniforms.substep_duration =
                        parameters.delta_time / T(live_substeps_ > 0 ? live_substeps_ : 1);
                    (*uniforms_)[0] = uniforms;

                    flush_staged_writes();

                    if (graph_ && graph_->size() > 0)
                        last_report_ = graph_->run();

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

                /**
                 * @brief How many colours the graph was built with.
                 *
                 * Structural, not live: the graph holds a node per colour per substep
                 * whether or not that colour currently has constraints.
                 */
                std::size_t color_count() const noexcept
                {
                    return constraints_store_.color_count();
                }

                /** @brief How many constraints one colour may hold. */
                std::size_t band_capacity() const noexcept
                {
                    return constraints_store_.band_capacity();
                }

                /** @brief Live constraints in colour @p color. */
                std::size_t color_size(std::size_t color) const noexcept
                {
                    return constraints_store_.band_size(color);
                }

                /** @brief The report from the most recent @ref step. */
                const SushiRuntime::RunReport& last_report() const noexcept
                {
                    return last_report_;
                }

            private:
                /**
                 * @brief Allocates every buffer once, at capacity.
                 *
                 * The hot columns are device-resident; the two host-touched slots —
                 * the per-step uniforms and the reduced motion maximum — are shared,
                 * because they are read or written by the host every tick and a
                 * transfer for four bytes would cost more than the value is worth.
                 */
                void allocate_buffers()
                {
                    using SushiRuntime::API::Residency;

                    // One device for every allocation in the scene. Unified shared
                    // memory is context-bound, and the scheduler fails a run whose
                    // node straddles two contexts — so this is the constraint being
                    // honoured, not a default being accepted.
                    const SushiRuntime::API::DeviceIndex device{
                        std::uint32_t(configuration_.device_index)};

                    const std::size_t bodies = body_slots_.capacity();
                    const std::size_t constraints = constraint_mirror_.size();

                    bodies_.emplace(runtime_.buffer<RigidBodyT<T>>(
                        bodies > 0 ? bodies : 1, device, Residency::Device));
                    constraints_.emplace(runtime_.buffer<Constraint>(
                        constraints > 0 ? constraints : 1, device, Residency::Device));
                    lambdas_.emplace(runtime_.buffer<T>(
                        constraints > 0 ? constraints : 1, device, Residency::Device));
                    motion_.emplace(runtime_.buffer<T>(bodies > 0 ? bodies : 1, device,
                                                       Residency::Device));
                    uniforms_.emplace(runtime_.buffer<StepUniforms<T>>(
                        1, device, Residency::Shared));
                    motion_maximum_.emplace(
                        runtime_.buffer<T>(1, device, Residency::Shared));

                    (*uniforms_)[0] = StepUniforms<T>{};
                    (*motion_maximum_)[0] = T(0);

                    // Every slot starts retired, so a slot that is never allocated
                    // contributes nothing to a predict, a derive, or the motion
                    // maximum. Without this the buffers would start as whatever the
                    // allocator last held.
                    RigidBodyT<T> retired;
                    retired.flags = BodyFlags::static_body;
                    for (std::size_t i = 0; i < bodies; ++i)
                        body_mirror_[i] = retired;
                    bodies_->write_range(SushiRuntime::API::ElementRange{0, bodies},
                                         body_mirror_.data());

                    const std::vector<T> zeros(bodies, T(0));
                    motion_->write_range(SushiRuntime::API::ElementRange{0, bodies},
                                         zeros.data());
                }

                /**
                 * @brief Emits the whole tick as one composition, built for the maximum.
                 *
                 * Every node is late-bound, so this runs exactly once for the life of
                 * the solver however much the body and constraint counts move.
                 */
                void build_graph()
                {
                    using SushiRuntime::API::Reads;
                    using SushiRuntime::API::Writes;

                    graph_.emplace(runtime_.graph());

                    RigidBodyT<T>* bodies = bodies_->data();
                    Constraint* constraints = constraints_->data();
                    T* lambdas = lambdas_->data();
                    T* motion = motion_->data();
                    const StepUniforms<T>* uniforms = uniforms_->data();

                    const std::size_t maximum = configuration_.substeps.maximum > 0
                                                    ? configuration_.substeps.maximum
                                                    : 1;

                    for (std::size_t substep = 0; substep < maximum; ++substep)
                    {
                        graph_->add(
                            SushiRuntime::API::when(substep_predicate(substep))
                                .and_sized(body_count_provider()),
                            Reads(*uniforms_), Writes(*bodies_), body_slots_.capacity(),
                            [bodies, uniforms](std::size_t i)
                            {
                                predict(bodies[i], uniforms->gravity,
                                        uniforms->substep_duration);
                            });

                        for (std::size_t color = 0; color < constraints_store_.color_count();
                             ++color)
                        {
                            const std::size_t base = constraints_store_.band_base(color);
                            const SushiRuntime::API::ElementRange band{
                                base, constraints_store_.band_capacity()};

                            graph_->add(
                                SushiRuntime::API::when(
                                    color_predicate(substep, color))
                                    .and_sized(color_count_provider(color)),
                                Reads(*uniforms_, constraints_->region(band)),
                                Writes(*bodies_, lambdas_->region(band)),
                                constraints_store_.band_capacity(),
                                [bodies, constraints, lambdas, uniforms,
                                 base](std::size_t i)
                                {
                                    const std::size_t k = base + i;
                                    // One iteration per substep is XPBD's small-step
                                    // arrangement, so the multiplier starts at zero
                                    // every time and never carries across. It is
                                    // still stored: warm starting and the impulse
                                    // readout both need the value the solve settled
                                    // on, and recomputing it would cost a second
                                    // projection.
                                    T lambda = T(0);
                                    XpbdDistanceProjectionT<T> projection;
                                    projection(constraints[k], bodies, lambda,
                                               uniforms->substep_duration);
                                    lambdas[k] = lambda;
                                });
                        }

                        graph_->add(
                            SushiRuntime::API::when(substep_predicate(substep))
                                .and_sized(body_count_provider()),
                            Reads(*uniforms_), Writes(*bodies_), body_slots_.capacity(),
                            [bodies, uniforms](std::size_t i)
                            {
                                update_velocity(bodies[i], uniforms->substep_duration);
                            });
                    }

                    // The substep count for the *next* tick is derived from this
                    // tick's outcome, which is what keeps it a function of simulation
                    // state alone. Measuring it here costs one pass; reading it back
                    // on the host would cost a transfer of every body.
                    // Reads(*bodies_) is load-bearing, not decoration. The runtime
                    // cannot infer what a kernel touches — the callable is a
                    // void(size_t) capturing raw pointers — so a node that reads the
                    // bodies without naming them has no edge to the solve at all, and
                    // is free to run beside it. This one did, and the conformance
                    // suite caught it: the two solvers derived different substep
                    // counts because one of them was measuring velocities that were
                    // still being written.
                    graph_->add(SushiRuntime::API::sized(body_count_provider()),
                                Reads(*bodies_), Writes(*motion_),
                                body_slots_.capacity(),
                                [bodies, motion](std::size_t i)
                                {
                                    const RigidBodyT<T>& body = bodies[i];
                                    motion[i] = is_simulated(body.flags)
                                                    ? length(body.velocity)
                                                    : T(0);
                                });

                    // Fixed-order, so the maximum is a function of the element layout
                    // alone and not of the worker count or the steal pattern. The
                    // whole capacity is folded: retired slots hold zero, which is the
                    // identity, so a varying live count needs no varying fold.
                    graph_->add_reduce(*motion_, *motion_maximum_,
                                       body_slots_.capacity(),
                                       SushiRuntime::API::Maximum<T>{}, T(0));
                }

                // The four providers below are returned as lambdas rather than as
                // std::function deliberately: a Dynamic's provider slot is a fixed
                // 48 bytes of inline storage with no heap fallback, and a
                // std::function does not fit it. Each captures `this` and at most
                // two indices, which does.

                /** @brief A provider reporting how many body slots have ever been used. */
                auto body_count_provider() const
                {
                    return [this]() -> std::size_t { return body_high_water_; };
                }

                /** @brief A provider reporting colour @p color's live constraint count. */
                auto color_count_provider(std::size_t color) const
                {
                    return [this, color]() -> std::size_t
                    {
                        return constraints_store_.band_size(color);
                    };
                }

                /** @brief A predicate enabling a colour's node only when it has work. */
                auto color_predicate(std::size_t substep, std::size_t color) const
                {
                    return [this, substep, color]() -> bool
                    {
                        return substep < live_substeps_ &&
                               constraints_store_.band_size(color) > 0;
                    };
                }

                /** @brief A predicate enabling a per-body node only when its substep runs. */
                auto substep_predicate(std::size_t substep) const
                {
                    return [this, substep]() -> bool { return substep < live_substeps_; };
                }

                /**
                 * @brief Stages a body write, to be flushed as one transfer before the step.
                 *
                 * Writing straight through would cost a queue round trip per body,
                 * and a scene that spawns a thousand bodies in one tick would pay a
                 * thousand of them. The mirror is a staging area only — it is never
                 * read back as state, because the device owns the state between
                 * ticks.
                 *
                 * @param slot The slot to write.
                 * @param body The state to stage.
                 */
                void stage_body(std::size_t slot, const RigidBodyT<T>& body)
                {
                    body_mirror_[slot] = body;
                    if (!body_dirty_)
                    {
                        body_dirty_ = true;
                        body_dirty_low_ = slot;
                        body_dirty_high_ = slot + 1;
                        return;
                    }
                    if (slot < body_dirty_low_)
                        body_dirty_low_ = slot;
                    if (slot + 1 > body_dirty_high_)
                        body_dirty_high_ = slot + 1;
                }

                /** @brief Stages a constraint write; see @ref stage_body for why. */
                void stage_constraint(std::size_t slot, const Constraint& constraint)
                {
                    constraint_mirror_[slot] = constraint;
                    if (!constraint_dirty_)
                    {
                        constraint_dirty_ = true;
                        constraint_dirty_low_ = slot;
                        constraint_dirty_high_ = slot + 1;
                        return;
                    }
                    if (slot < constraint_dirty_low_)
                        constraint_dirty_low_ = slot;
                    if (slot + 1 > constraint_dirty_high_)
                        constraint_dirty_high_ = slot + 1;
                }

                /** @brief Sends every staged write to the device as one range each. */
                void flush_staged_writes()
                {
                    if (body_dirty_)
                    {
                        bodies_->write_range(
                            SushiRuntime::API::ElementRange{
                                body_dirty_low_, body_dirty_high_ - body_dirty_low_},
                            body_mirror_.data() + body_dirty_low_);
                        body_dirty_ = false;
                    }
                    if (constraint_dirty_)
                    {
                        constraints_->write_range(
                            SushiRuntime::API::ElementRange{
                                constraint_dirty_low_,
                                constraint_dirty_high_ - constraint_dirty_low_},
                            constraint_mirror_.data() + constraint_dirty_low_);
                        constraint_dirty_ = false;
                    }
                }

                /**
                 * @brief Removes a constraint whose handle is known live.
                 *
                 * The band stays dense: the last constraint in the band moves into
                 * the vacated slot and the count drops by one. Density is what lets a
                 * colour's node iterate `[base, base + count)` with no per-element
                 * liveness test — the alternative is a wasted lane for every hole,
                 * and holes accumulate.
                 *
                 * @param handle The constraint to remove.
                 */
                void remove_constraint_unchecked(ConstraintHandle handle)
                {
                    const std::size_t slot = constraints_store_.slot_of(handle);
                    if (slot >= constraint_mirror_.size())
                        return;
                    const Constraint removed = constraint_mirror_[slot];
                    const ConstraintRemoval removal =
                        constraints_store_.remove(handle, removed.a, removed.b);
                    if (!removal.removed)
                        return;
                    // The store moved the bookkeeping; the descriptor has to follow,
                    // because the store deliberately does not own it — it may live in
                    // a device buffer the host cannot address.
                    if (removal.slot != removal.moved_from)
                        stage_constraint(removal.slot,
                                         constraint_mirror_[removal.moved_from]);
                }

                /**
                 * @brief Removes every live constraint naming body slot @p body.
                 *
                 * Scans the live constraints rather than keeping a per-body list. A
                 * per-body adjacency list would make this O(degree), but it would
                 * also have to be maintained on every add and every swap-remove, and
                 * body removal is the rare operation of the three. If profiling ever
                 * says otherwise, the adjacency list is the change to make.
                 *
                 * @param body The body slot whose constraints go.
                 */
                void remove_constraints_touching(std::uint32_t body)
                {
                    for (std::size_t color = 0; color < constraints_store_.color_count();
                         ++color)
                    {
                        const std::size_t base = constraints_store_.band_base(color);
                        // Downward, because a swap-remove fills the vacated slot from
                        // the top of the band — an entry this walk has already
                        // examined and rejected. Walking upward would move an
                        // unexamined entry into a slot already passed.
                        std::size_t offset = constraints_store_.band_size(color);
                        while (offset > 0)
                        {
                            --offset;
                            const std::size_t slot = base + offset;
                            const Constraint& constraint = constraint_mirror_[slot];
                            if (constraint.a != body && constraint.b != body)
                                continue;
                            const ConstraintRemoval removal = constraints_store_.remove(
                                constraints_store_.handle_at(slot), constraint.a,
                                constraint.b);
                            if (removal.removed && removal.slot != removal.moved_from)
                                stage_constraint(removal.slot,
                                                 constraint_mirror_[removal.moved_from]);
                        }
                    }
                }

                /**
                 * @brief The substep count for this tick, derived from state alone.
                 *
                 * The measure is the fastest body's travel over one tick divided by a
                 * characteristic size, taken from the *previous* tick's reduction.
                 * Using the previous tick is not a compromise: the value is still a
                 * function of simulation state, which is what determinism requires,
                 * and reading the current tick's would mean a transfer in the middle
                 * of the one composition this design exists to keep whole.
                 *
                 * @param delta_time The tick's duration, in seconds.
                 * @return A substep count within the schedule's bounds.
                 */
                std::size_t derive_substep_count(T delta_time)
                {
                    const SubstepSchedule<T>& schedule = configuration_.substeps;
                    const std::size_t minimum = schedule.minimum > 0 ? schedule.minimum : 1;
                    const std::size_t maximum =
                        schedule.maximum > minimum ? schedule.maximum : minimum;

                    const T fastest = (*motion_maximum_)[0];
                    if (!(fastest > T(0)) || !(schedule.motion_budget > T(0)))
                        return minimum;

                    const T travel = fastest * delta_time;
                    const T wanted = travel / schedule.motion_budget;
                    if (!(wanted > T(minimum)))
                        return minimum;

                    const std::size_t derived = std::size_t(wanted) + 1;
                    return derived < maximum ? derived : maximum;
                }

                /** @brief Refreshes the per-tick counters after a run. */
                void refresh_statistics()
                {
                    statistics_.awake_bodies = body_slots_.live_count();
                    statistics_.sleeping_bodies = 0;
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

                    // compile_count is the number this solver is held to: one
                    // after warm-up, for ever. compose_count belongs to the region
                    // graph and stays zero here, because a static composition is
                    // never recomposed — reporting the compile count under both
                    // names would hide exactly the distinction the two exist to draw.
                    statistics_.compile_count = graph_ ? graph_->compile_count() : 0;
                    statistics_.compose_count = 0;
                }

                SushiRuntime::API::Runtime& runtime_;
                PhysicsConfigurationT<T> configuration_;

                HandleTable<BodyTag> body_slots_;
                ConstraintStore constraints_store_;

                std::vector<Constraint> constraint_mirror_;
                std::vector<RigidBodyT<T>> body_mirror_;

                bool body_dirty_ = false;
                std::size_t body_dirty_low_ = 0;
                std::size_t body_dirty_high_ = 0;
                bool constraint_dirty_ = false;
                std::size_t constraint_dirty_low_ = 0;
                std::size_t constraint_dirty_high_ = 0;

                std::size_t body_high_water_ = 0;
                std::size_t live_substeps_ = 1;

                std::optional<SushiRuntime::API::Buffer<RigidBodyT<T>>> bodies_;
                std::optional<SushiRuntime::API::Buffer<Constraint>> constraints_;
                std::optional<SushiRuntime::API::Buffer<T>> lambdas_;
                std::optional<SushiRuntime::API::Buffer<T>> motion_;
                std::optional<SushiRuntime::API::Buffer<StepUniforms<T>>> uniforms_;
                std::optional<SushiRuntime::API::Buffer<T>> motion_maximum_;
                std::optional<SushiRuntime::API::Graph> graph_;

                SushiRuntime::RunReport last_report_{};
                PhysicsStatisticsT<T> statistics_{};
        };
    } // namespace Physics
} // namespace SushiEngine
