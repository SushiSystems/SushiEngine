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
 * @brief The one file in the physics layer that builds an execution graph.
 *
 * Every `Buffer`, `Graph`, and node declaration the physics uses lives inside this
 * class. It is written against `SushiEngine::Execution` — the engine's own seam — so
 * it names no runtime type at all: which backend executes the solve is a build-time
 * choice, and the API instability that made this file's isolation worth having in the
 * first place now stops one layer further out, at the seam's adapter.
 *
 * The isolation itself is unchanged and still the point: nothing else under
 * `physics/` builds a graph, so when the execution surface moves, one file moves with
 * it and the conformance suite proves the behaviour did not.
 *
 * ### The shape of the graph, and why
 *
 * The whole tick is **one** composition and **one** `run()`. A substep loop driven
 * from the host would be one blocking round trip per substep, and at 32 substeps and
 * 60 Hz that latency is larger than everything else in the system put together. So
 * the graph is built once, for the *maximum* substep count, as:
 *
 *     for each substep s:
 *         prepare contacts, per colour  (capture arrival speed, clear accumulators)
 *         predict                       (per body)
 *         project colour 0              (per distance constraint in colour 0)
 *         project colour 1
 *         ...
 *         project elements, per colour  (deviatoric then hydrostatic, per tetrahedron)
 *         project joints, per colour    (attachment, limits, position drives)
 *         project contacts, per colour  (non-penetration + static friction)
 *         derive velocity               (per body)
 *         joint velocity, per colour    (rate drives, friction)
 *         contact velocity, per colour  (dynamic friction + restitution)
 *     measure motion       (per body, once)
 *     reduce the maximum   (Graph::add_reduce, fixed order)
 *
 * Four kinds share that schedule and each is a constraint kind like any other
 * (§6.3) — the same colouring, the same fixed bands, the same late binding. They
 * differ in lifetime: a distance constraint, a joint and a FEM element are written
 * when added, a contact's buffer is refilled every tick. They differ also in how many
 * bodies they name, which they did not until P6-J1: an element is a four-body
 * hyperedge, and `constraint_bodies.hpp` is where the colouring and the store learn
 * that without either of them growing a per-kind branch. That is exactly what `Dynamic` was for:
 * the counts change every tick, the *structure* never does, and `compile_count()`
 * stays one. Where each kind's stages sit in the substep is not a choice made here —
 * it is `contact_projection.hpp`'s and `joint_projection.hpp`'s schedule, and the host
 * solver walks the same one so the conformance suite can hold them to it.
 *
 * The kind dimension is what multiplies the node count: one node per (kind, colour)
 * per substep, plus the per-body stages. That is the cost `PhysicsCapacities::colors`
 * warns about, and it is why a joint kind that needed its own band per named joint
 * type would have been the wrong shape (`joint.hpp`).
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
#include <initializer_list>
#include <optional>
#include <utility>
#include <vector>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/execution/context.hpp>
#include <SushiEngine/physics/constraints/distance_projection.hpp>
#include <SushiEngine/physics/constraints/joint_projection.hpp>
#include <SushiEngine/physics/constraints/xpbd_constraint.hpp>
#include <SushiEngine/physics/core/body_flags.hpp>
#include <SushiEngine/physics/core/configuration.hpp>
#include <SushiEngine/physics/core/handle.hpp>
#include <SushiEngine/physics/core/rigid_body.hpp>
#include <SushiEngine/physics/core/statistics.hpp>
#include <SushiEngine/physics/soft/fem_projection.hpp>
#include <SushiEngine/physics/solver/constraint_store.hpp>
#include <SushiEngine/physics/solver/contact_constraint.hpp>
#include <SushiEngine/physics/solver/contact_store.hpp>
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
                /** @brief The persistent constraint kind this solver admits. */
                using Constraint = XpbdDistanceConstraintT<T>;

                /** @brief The per-tick constraint kind this solver admits. */
                using Contact = ContactConstraintT<T>;

                /** @brief The articulated persistent kind this solver admits. */
                using Joint = JointConstraintT<T>;

                /** @brief The deformable persistent kind this solver admits (§9.1). */
                using Element = FemTetrahedronT<T>;

                /**
                 * @brief Allocates every buffer at capacity and compiles the solve graph.
                 *
                 * Nothing here is deferred to a `finalize()`. The buffers are sized
                 * from @p configuration and never resized — a `Buffer` cannot grow in
                 * place, and a reallocation would invalidate the raw pointer every
                 * compiled node captured — so the graph can be built immediately and
                 * stays valid for every body and constraint that will ever be added.
                 *
                 * @param context       The execution context backing the buffers and the graph.
                 * @param configuration The scene's budgets and substep schedule.
                 */
                RuntimeGraphBuilder(Execution::Context& context,
                                    const PhysicsConfigurationT<T>& configuration)
                    : context_(context),
                      configuration_(configuration),
                      body_slots_(configuration.capacities.bodies),
                      constraints_store_(configuration.capacities.bodies,
                                         configuration.capacities.constraints,
                                         configuration.capacities.colors),
                      joints_store_(constraints_store_.shared_coloring(),
                                    configuration.capacities.joints,
                                    configuration.capacities.colors),
                      elements_store_(constraints_store_.shared_coloring(),
                                      configuration.capacities.elements,
                                      configuration.capacities.colors),
                      contacts_store_(configuration.capacities.bodies,
                                      configuration.capacities.contacts,
                                      configuration.capacities.colors),
                      constraint_mirror_(constraints_store_.band_capacity() *
                                         constraints_store_.color_count()),
                      joint_mirror_(joints_store_.band_capacity() *
                                    joints_store_.color_count()),
                      element_mirror_(elements_store_.band_capacity() *
                                      elements_store_.color_count()),
                      contact_mirror_(contacts_store_.capacity()),
                      body_mirror_(configuration.capacities.bodies)
                {
                    // Migrating work mid-run costs jitter, and jitter is the one thing
                    // a fixed-rate tick cannot absorb.
                    context_.set_work_migration(configuration_.rebalancer);

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
                    remove_joints_touching(handle.index);
                    remove_elements_touching(handle.index);

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
                        Execution::ElementRange{handle.index, 1}, &still);

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

                /** @copydoc IConstraintSolver::add_element */
                ConstraintHandle add_element(const Element& element) override
                {
                    const ConstraintPlacement placement =
                        elements_store_.place_bodies(element.vertex, 4);
                    if (!placement.handle.valid())
                    {
                        ++statistics_.capacity_overflows;
                        return placement.handle;
                    }
                    stage_element(placement.slot, element);
                    return placement.handle;
                }

                /** @copydoc IConstraintSolver::remove_element */
                bool remove_element(ConstraintHandle handle) override
                {
                    if (!elements_store_.alive(handle))
                        return false;
                    remove_element_unchecked(handle);
                    return true;
                }

                /**
                 * @copydoc IConstraintSolver::read_element
                 *
                 * From the mirror, which `step` refreshed from the device — the same
                 * arrangement @ref read_joint has and for the same reason: an element is
                 * written from both sides. The host owns its rest state and its Lamé
                 * pair, the device owns its multipliers, and the mirror holds whichever
                 * is newer so a staged edit is not lost to a readback.
                 */
                bool read_element(ConstraintHandle handle, Element& element) const override
                {
                    if (!elements_store_.alive(handle))
                        return false;
                    const std::size_t slot = elements_store_.slot_of(handle);
                    if (slot >= element_mirror_.size())
                        return false;
                    element = element_mirror_[slot];
                    return true;
                }

                /** @copydoc IConstraintSolver::element_capacity */
                std::size_t element_capacity() const noexcept override
                {
                    return elements_store_.capacity();
                }

                /** @brief Live elements in colour @p color. */
                std::size_t element_color_size(std::size_t color) const noexcept
                {
                    return elements_store_.band_size(color);
                }

                /** @copydoc IConstraintSolver::add_joint */
                JointHandle add_joint(const Joint& joint) override
                {
                    const ConstraintPlacement placement = joints_store_.place(joint.a, joint.b);
                    if (!placement.handle.valid())
                    {
                        ++statistics_.capacity_overflows;
                        return JointHandle{};
                    }
                    stage_joint(placement.slot, joint);
                    return JointHandle{placement.handle.index, placement.handle.generation};
                }

                /** @copydoc IConstraintSolver::remove_joint */
                bool remove_joint(JointHandle handle) override
                {
                    const ConstraintHandle stored{handle.index, handle.generation};
                    if (!joints_store_.alive(stored))
                        return false;
                    remove_joint_unchecked(stored);
                    return true;
                }

                /**
                 * @copydoc IConstraintSolver::read_joint
                 *
                 * From the mirror, which `step` refreshed from the device after the
                 * run. The mirror is authoritative here in a way it deliberately is
                 * not for bodies: a joint is both written by the host (a motor target,
                 * a limit) and written by the device (the load it carried), so the
                 * mirror holds whichever is newer and reading the device instead would
                 * lose a staged edit — the mirror image of the defect §16.5 records
                 * for `read_body`.
                 */
                bool read_joint(JointHandle handle, Joint& joint) const override
                {
                    const ConstraintHandle stored{handle.index, handle.generation};
                    const std::size_t slot = joints_store_.slot_of(stored);
                    if (slot >= joint_mirror_.size())
                        return false;
                    joint = joint_mirror_[slot];
                    return true;
                }

                /** @copydoc IConstraintSolver::write_joint */
                bool write_joint(JointHandle handle, const Joint& joint) override
                {
                    const ConstraintHandle stored{handle.index, handle.generation};
                    const std::size_t slot = joints_store_.slot_of(stored);
                    if (slot >= joint_mirror_.size())
                        return false;
                    stage_joint(slot, joint);
                    return true;
                }

                /** @copydoc IConstraintSolver::joint_capacity */
                std::size_t joint_capacity() const noexcept override
                {
                    return joints_store_.capacity();
                }

                /** @brief Live joints in colour @p color. */
                std::size_t joint_color_size(std::size_t color) const noexcept
                {
                    return joints_store_.band_size(color);
                }

                /** @copydoc IConstraintSolver::begin_contacts */
                void begin_contacts() override
                {
                    contacts_store_.begin();
                    submission_slots_.clear();
                }

                /** @copydoc IConstraintSolver::add_contact */
                bool add_contact(const Contact& contact) override
                {
                    if (!contact_slots_valid(contact, body_slots_.capacity()))
                        return false;
                    const ContactPlacement placement = contacts_store_.place(
                        contact.a, contact.b, constraints_store_.coloring());
                    if (!placement.placed)
                    {
                        ++statistics_.capacity_overflows;
                        return false;
                    }
                    // Written into the mirror only; the bands go to the device as one
                    // transfer each in `step`, because a scene resolving a thousand
                    // contacts should pay for the bands it filled and not for a queue
                    // round trip per manifold.
                    contact_mirror_[placement.slot] = contact;
                    submission_slots_.push_back(placement.slot);
                    return true;
                }

                /** @copydoc IConstraintSolver::contact_count */
                std::size_t contact_count() const noexcept override
                {
                    return submission_slots_.size();
                }

                /** @copydoc IConstraintSolver::read_contact */
                bool read_contact(std::size_t index, Contact& contact) const override
                {
                    if (index >= submission_slots_.size())
                        return false;
                    // From the mirror, which `step` refreshed from the device after
                    // the run. Reading the device here instead would be a round trip
                    // per contact for a caller that is, without exception, about to
                    // read all of them.
                    contact = contact_mirror_[submission_slots_[index]];
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
                    // The device owns the state, so a read has to come from it — but
                    // a write staged since the last step has not reached it yet, and
                    // a body added and then read before the first tick would come
                    // back as the retired slot it used to be. Flushing here keeps one
                    // rule instead of two: the device is the truth, and everything
                    // staged is on the device before anyone looks. The host solver
                    // has no staging and so no way to disagree, which is exactly why
                    // the conformance suite is where this surfaced.
                    flush_staged_writes();
                    const std::vector<RigidBodyT<T>> range = bodies_->read_range(
                        Execution::ElementRange{first, count});
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
                    live_substeps_ = derive_substep_count(parameters.delta_time,
                                                         parameters.substep_floor);

                    StepUniforms<T> uniforms;
                    uniforms.gravity = parameters.gravity;
                    uniforms.substep_duration =
                        parameters.delta_time / T(live_substeps_ > 0 ? live_substeps_ : 1);
                    (*uniforms_)[0] = uniforms;

                    flush_staged_writes();
                    upload_contacts();

                    if (graph_ && graph_->size() > 0)
                        last_report_ = graph_->run();

                    download_contacts();
                    download_joints();
                    download_elements();
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

                /** @copydoc IConstraintSolver::contact_capacity */
                std::size_t contact_capacity() const noexcept override
                {
                    return contacts_store_.capacity();
                }

                /** @brief Live constraints in colour @p color. */
                std::size_t color_size(std::size_t color) const noexcept
                {
                    return constraints_store_.band_size(color);
                }

                /** @brief Contacts submitted into colour @p color this tick. */
                std::size_t contact_color_size(std::size_t color) const noexcept
                {
                    return contacts_store_.band_size(color);
                }

                /** @brief The report from the most recent @ref step. */
                const Execution::RunReport& last_report() const noexcept
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
                    using Execution::MemoryVisibility;

                    // One device for every allocation in the scene. Unified shared
                    // memory is context-bound, and the scheduler fails a run whose
                    // node straddles two contexts — so this is the constraint being
                    // honoured, not a default being accepted.
                    const Execution::DeviceIndex device{
                        std::uint32_t(configuration_.device_index)};

                    const std::size_t bodies = body_slots_.capacity();
                    const std::size_t constraints = constraint_mirror_.size();

                    bodies_.emplace(context_.allocate<RigidBodyT<T>>(
                        bodies > 0 ? bodies : 1, MemoryVisibility::DeviceResident, device));
                    constraints_.emplace(context_.allocate<Constraint>(
                        constraints > 0 ? constraints : 1, MemoryVisibility::DeviceResident,
                        device));
                    lambdas_.emplace(context_.allocate<T>(
                        constraints > 0 ? constraints : 1, MemoryVisibility::DeviceResident,
                        device));
                    const std::size_t joints = joint_mirror_.size();
                    joints_.emplace(context_.allocate<Joint>(
                        joints > 0 ? joints : 1, MemoryVisibility::DeviceResident, device));
                    const std::size_t elements = element_mirror_.size();
                    elements_.emplace(context_.allocate<Element>(
                        elements > 0 ? elements : 1, MemoryVisibility::DeviceResident, device));
                    const std::size_t contacts = contact_mirror_.size();
                    contacts_.emplace(context_.allocate<Contact>(
                        contacts > 0 ? contacts : 1, MemoryVisibility::DeviceResident, device));
                    motion_.emplace(context_.allocate<T>(
                        bodies > 0 ? bodies : 1, MemoryVisibility::DeviceResident, device));
                    // No partial column: `Graph::add_reduce` allocates and owns the
                    // intermediate levels itself, on the same device and with the
                    // same residency as the input.
                    uniforms_.emplace(context_.allocate<StepUniforms<T>>(
                        1, MemoryVisibility::HostShared, device));
                    motion_maximum_.emplace(
                        context_.allocate<T>(1, MemoryVisibility::HostShared, device));

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
                    bodies_->write_range(Execution::ElementRange{0, bodies},
                                         body_mirror_.data());

                    const std::vector<T> zeros(bodies, T(0));
                    motion_->write_range(Execution::ElementRange{0, bodies},
                                         zeros.data());
                }

                /**
                 * @brief Declares a read of one buffer window by a solver kernel.
                 * @param interval The window the kernel reads.
                 * @return The access, as a device-compute read.
                 */
                static Execution::ResourceAccess read_of(Execution::BufferInterval interval) noexcept
                {
                    return Execution::ResourceAccess{interval,
                                                     Execution::AccessIntent::ComputeRead};
                }

                /**
                 * @brief Declares a write of one buffer window by a solver kernel.
                 * @param interval The window the kernel writes.
                 * @return The access, as a device-compute write.
                 */
                static Execution::ResourceAccess write_of(Execution::BufferInterval interval) noexcept
                {
                    return Execution::ResourceAccess{interval,
                                                     Execution::AccessIntent::ComputeWrite};
                }

                /**
                 * @brief Emits one late-bound solver node.
                 *
                 * The declared accesses are load-bearing, not decoration: the tracker
                 * cannot infer what a kernel touches — the callable is a `void(size_t)`
                 * capturing raw pointers — so a node that reads the bodies without
                 * naming them has no edge to the solve at all and is free to run beside
                 * it. The conformance suite has caught exactly that.
                 *
                 * The access list is not copied. An initializer list's backing array
                 * lives to the end of the full expression that created it, which
                 * outlasts the submission the backend copies from.
                 *
                 * @tparam Kernel  The per-element callable's type.
                 * @param name     Diagnostic name for the node.
                 * @param accesses What the kernel reads and writes.
                 * @param capacity Pre-sized upper bound the live count must fit.
                 * @param count    Live element count for this run.
                 * @param enabled  Whether the node runs at all this run; unbound means always.
                 * @param kernel   The per-element callable.
                 */
                template <typename Kernel>
                void emit_node(const char* name,
                               std::initializer_list<Execution::ResourceAccess> accesses,
                               std::size_t capacity, Execution::CountProvider count,
                               Execution::EnabledProvider enabled, Kernel&& kernel)
                {
                    Execution::NodeDescriptor node;
                    node.name = name;
                    node.accesses = accesses.begin();
                    node.access_count = accesses.size();
                    node.capacity = capacity;
                    node.count = count;
                    node.enabled = enabled;
                    graph_->add_parallel(node, std::forward<Kernel>(kernel));
                }

                /**
                 * @brief Emits the whole tick as one composition, built for the maximum.
                 *
                 * Every node is late-bound, so this runs exactly once for the life of
                 * the solver however much the body and constraint counts move.
                 */
                void build_graph()
                {
                    graph_.emplace(context_.create_graph());

                    RigidBodyT<T>* bodies = bodies_->data();
                    Constraint* constraints = constraints_->data();
                    T* lambdas = lambdas_->data();
                    Joint* joints = joints_->data();
                    Element* elements = elements_->data();
                    Contact* contacts = contacts_->data();
                    T* motion = motion_->data();
                    const StepUniforms<T>* uniforms = uniforms_->data();

                    const std::size_t maximum = configuration_.substeps.maximum > 0
                                                    ? configuration_.substeps.maximum
                                                    : 1;

                    for (std::size_t substep = 0; substep < maximum; ++substep)
                    {
                        // The contact preparation reads bodies and the predict below
                        // writes them, so the dependency tracker orders predict after
                        // every preparation node by the write-after-read edge — which
                        // is exactly the ordering the schedule needs and the reason
                        // this does not have to be forced with a false write.
                        for (std::size_t color = 0; color < contacts_store_.color_count();
                             ++color)
                        {
                            const Execution::ElementRange band{
                                contacts_store_.band_base(color),
                                contacts_store_.band_capacity()};
                            const std::size_t base = contacts_store_.band_base(color);
                            const bool first = substep == 0;

                            emit_node(
                                "contact_prepare",
                                {read_of(bodies_->interval()),
                                 write_of(contacts_->interval(band))},
                                contacts_store_.band_capacity(),
                                contact_count_provider(color),
                                contact_predicate(substep, color),
                                [bodies, contacts, base, first](std::size_t i)
                                {
                                    ContactPreparationT<T> prepare;
                                    prepare(contacts[base + i], bodies, first);
                                });
                        }

                        emit_node("predict",
                                  {read_of(uniforms_->interval()),
                                   write_of(bodies_->interval())},
                                  body_slots_.capacity(), body_count_provider(),
                                  substep_predicate(substep),
                                  [bodies, uniforms](std::size_t i)
                                  {
                                      predict(bodies[i], uniforms->gravity,
                                              uniforms->substep_duration);
                                  });

                        for (std::size_t color = 0; color < constraints_store_.color_count();
                             ++color)
                        {
                            const std::size_t base = constraints_store_.band_base(color);
                            const Execution::ElementRange band{
                                base, constraints_store_.band_capacity()};

                            emit_node(
                                "distance_project",
                                {read_of(uniforms_->interval()),
                                 read_of(constraints_->interval(band)),
                                 write_of(bodies_->interval()),
                                 write_of(lambdas_->interval(band))},
                                constraints_store_.band_capacity(),
                                color_count_provider(color),
                                color_predicate(substep, color),
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

                        // The elements, after the distance lattice and before the
                        // joints. Both are constitutive - they say what the material
                        // *is* - so they belong together and ahead of the articulation
                        // hanging off it.
                        //
                        // One node per colour rather than two, with both projections in
                        // it. Splitting them would double the node count for no
                        // parallelism: within a colour no two elements share a particle,
                        // so a deviatoric pass and a hydrostatic pass over the same
                        // colour would be two barriers where the material wants one
                        // Gauss-Seidel sweep - and the ordering *within* an element,
                        // deviatoric then hydrostatic, is part of the answer.
                        //
                        // Skipped structurally when the element budget is zero, which
                        // is the default. A node over an empty band would never run —
                        // its predicate could not become true — but it would still be a
                        // zero-extent region in the composition of every scene in the
                        // engine that has no soft bodies, which is most of them.
                        for (std::size_t color = 0;
                             elements_store_.band_capacity() > 0 &&
                             color < elements_store_.color_count();
                             ++color)
                        {
                            const std::size_t base = elements_store_.band_base(color);
                            const Execution::ElementRange band{
                                base, elements_store_.band_capacity()};

                            emit_node(
                                "element_project",
                                {read_of(uniforms_->interval()),
                                 write_of(bodies_->interval()),
                                 write_of(elements_->interval(band))},
                                elements_store_.band_capacity(),
                                element_count_provider(color),
                                element_predicate(substep, color),
                                [bodies, elements, uniforms, base](std::size_t i)
                                {
                                    Element& element = elements[base + i];
                                    // Reset per substep, not per tick: one iteration per
                                    // substep is XPBD's small-step arrangement, so
                                    // nothing may carry across - the same rule the
                                    // distance kind states by starting its multiplier at
                                    // zero every time.
                                    element.deviatoric_lambda = 0;
                                    element.hydrostatic_lambda = 0;
                                    project_fem_deviatoric(bodies, element,
                                                           uniforms->substep_duration);
                                    project_fem_hydrostatic(bodies, element,
                                                            uniforms->substep_duration);
                                });
                        }

                        // After the distance constraints and before the contacts: a
                        // joint is a structural constraint and a contact is a reactive
                        // one, so the assembly is assembled before it is pushed on.
                        // The host solver walks the same order and the conformance
                        // suite is what stops the two drifting apart.
                        for (std::size_t color = 0; color < joints_store_.color_count(); ++color)
                        {
                            const std::size_t base = joints_store_.band_base(color);
                            const Execution::ElementRange band{
                                base, joints_store_.band_capacity()};
                            const bool first = substep == 0;

                            emit_node(
                                "joint_project",
                                {read_of(uniforms_->interval()),
                                 write_of(bodies_->interval()),
                                 write_of(joints_->interval(band))},
                                joints_store_.band_capacity(),
                                joint_count_provider(color),
                                joint_predicate(substep, color),
                                [bodies, joints, uniforms, base, first](std::size_t i)
                                {
                                    JointProjectionT<T> projection;
                                    projection(joints[base + i], bodies,
                                               uniforms->substep_duration, first);
                                });
                        }

                        // After the persistent kinds and before the velocity
                        // derivation, because non-penetration and static friction are
                        // corrections to *position* and every positional projection in
                        // a substep belongs in one band of the schedule.
                        for (std::size_t color = 0; color < contacts_store_.color_count();
                             ++color)
                        {
                            const Execution::ElementRange band{
                                contacts_store_.band_base(color),
                                contacts_store_.band_capacity()};
                            const std::size_t base = contacts_store_.band_base(color);

                            emit_node(
                                "contact_position",
                                {read_of(uniforms_->interval()),
                                 write_of(bodies_->interval()),
                                 write_of(contacts_->interval(band))},
                                contacts_store_.band_capacity(),
                                contact_count_provider(color),
                                contact_predicate(substep, color),
                                [bodies, contacts, base](std::size_t i)
                                {
                                    ContactPositionProjectionT<T> projection;
                                    projection(contacts[base + i], bodies);
                                });
                        }

                        emit_node("update_velocity",
                                  {read_of(uniforms_->interval()),
                                   write_of(bodies_->interval())},
                                  body_slots_.capacity(), body_count_provider(),
                                  substep_predicate(substep),
                                  [bodies, uniforms](std::size_t i)
                                  {
                                      update_velocity(bodies[i], uniforms->substep_duration);
                                  });

                        // The velocity pass, in the same order as the positional one.
                        // A joint's rate drive and its friction are statements about a
                        // velocity, so they wait for `update_velocity` exactly as the
                        // contact velocity pass below does.
                        for (std::size_t color = 0; color < joints_store_.color_count(); ++color)
                        {
                            const std::size_t base = joints_store_.band_base(color);
                            const Execution::ElementRange band{
                                base, joints_store_.band_capacity()};

                            emit_node(
                                "joint_velocity",
                                {read_of(uniforms_->interval()),
                                 write_of(bodies_->interval()),
                                 write_of(joints_->interval(band))},
                                joints_store_.band_capacity(),
                                joint_count_provider(color),
                                joint_predicate(substep, color),
                                [bodies, joints, uniforms, base](std::size_t i)
                                {
                                    JointVelocityProjectionT<T> projection;
                                    projection(joints[base + i], bodies,
                                               uniforms->substep_duration);
                                });
                        }

                        // Last in the substep. Dynamic friction and restitution are
                        // statements about a velocity, and until `update_velocity` has
                        // read the substep's pose change back as one there is no
                        // velocity for them to be statements about.
                        for (std::size_t color = 0; color < contacts_store_.color_count();
                             ++color)
                        {
                            const Execution::ElementRange band{
                                contacts_store_.band_base(color),
                                contacts_store_.band_capacity()};
                            const std::size_t base = contacts_store_.band_base(color);

                            emit_node(
                                "contact_velocity",
                                {read_of(uniforms_->interval()),
                                 write_of(bodies_->interval()),
                                 write_of(contacts_->interval(band))},
                                contacts_store_.band_capacity(),
                                contact_count_provider(color),
                                contact_predicate(substep, color),
                                [bodies, contacts, uniforms, base](std::size_t i)
                                {
                                    ContactVelocityProjectionT<T> projection;
                                    projection(contacts[base + i], bodies,
                                               uniforms->substep_duration);
                                });
                        }
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
                    emit_node("motion_measure",
                              {read_of(bodies_->interval()), write_of(motion_->interval())},
                              body_slots_.capacity(), body_count_provider(),
                              Execution::EnabledProvider{},
                              [bodies, motion](std::size_t i)
                                {
                                    const RigidBodyT<T>& body = bodies[i];
                                    motion[i] = is_simulated(body.flags)
                                                    ? length(body.velocity)
                                                    : T(0);
                                });

                    // The reduction, which is the runtime's.
                    //
                    // Fixed order is the requirement, not an implementation detail:
                    // the maximum must be a function of the element layout alone and
                    // not of the worker count or the steal pattern, or the derived
                    // substep count — and therefore the whole simulation — depends on
                    // how the scheduler felt that frame (§12.1). `Graph::add_reduce`
                    // is exactly that promise: contiguous tiles of at most 256 values
                    // folded left to right, the partials folded the same way, and
                    // which values meet which a function of `n` and the tile size
                    // alone. This file used to hand-build it out of two ordinary
                    // nodes, because §18's record of the runtime seams was read on a
                    // checkout that did not carry them; it does.
                    //
                    // The whole capacity is folded rather than the live count: a
                    // retired slot holds zero, which is the identity for a maximum
                    // over non-negative speeds, so a varying live count needs no
                    // varying fold — and a fold whose *extent* varied would be a fold
                    // whose order varied.
                    const std::size_t capacity = body_slots_.capacity();
                    graph_->add_reduce(*motion_, *motion_maximum_, capacity, Maximum<T>{},
                                       T(0));
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

                /** @brief A provider reporting colour @p color's live element count. */
                auto element_count_provider(std::size_t color) const
                {
                    return [this, color]() -> std::size_t
                    {
                        return elements_store_.band_size(color);
                    };
                }

                /** @brief A predicate enabling an element colour's node only when it has work. */
                auto element_predicate(std::size_t substep, std::size_t color) const
                {
                    return [this, substep, color]() -> bool
                    {
                        return substep < live_substeps_ &&
                               elements_store_.band_size(color) > 0;
                    };
                }

                /** @brief A provider reporting colour @p color's live joint count. */
                auto joint_count_provider(std::size_t color) const
                {
                    return [this, color]() -> std::size_t
                    {
                        return joints_store_.band_size(color);
                    };
                }

                /** @brief A predicate enabling a joint colour's node only when it has work. */
                auto joint_predicate(std::size_t substep, std::size_t color) const
                {
                    return [this, substep, color]() -> bool
                    {
                        return substep < live_substeps_ && joints_store_.band_size(color) > 0;
                    };
                }

                /** @brief A provider reporting colour @p color's submitted contact count. */
                auto contact_count_provider(std::size_t color) const
                {
                    return [this, color]() -> std::size_t
                    {
                        return contacts_store_.band_size(color);
                    };
                }

                /**
                 * @brief A predicate enabling a contact colour's node when it has work.
                 *
                 * The whole reason contacts can live in the graph at all. A tick with
                 * no contacts switches every one of these nodes off without touching
                 * the composition, so `compile_count()` stays at one through a scene
                 * that goes from an empty room to a collapsing stack and back.
                 */
                auto contact_predicate(std::size_t substep, std::size_t color) const
                {
                    return [this, substep, color]() -> bool
                    {
                        return substep < live_substeps_ &&
                               contacts_store_.band_size(color) > 0;
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

                /** @brief Stages an element write; see @ref stage_body for why. */
                void stage_element(std::size_t slot, const Element& element)
                {
                    element_mirror_[slot] = element;
                    if (!element_dirty_)
                    {
                        element_dirty_ = true;
                        element_dirty_low_ = slot;
                        element_dirty_high_ = slot + 1;
                        return;
                    }
                    if (slot < element_dirty_low_)
                        element_dirty_low_ = slot;
                    if (slot + 1 > element_dirty_high_)
                        element_dirty_high_ = slot + 1;
                }

                /** @brief Stages a joint write; see @ref stage_body for why. */
                void stage_joint(std::size_t slot, const Joint& joint)
                {
                    joint_mirror_[slot] = joint;
                    if (!joint_dirty_)
                    {
                        joint_dirty_ = true;
                        joint_dirty_low_ = slot;
                        joint_dirty_high_ = slot + 1;
                        return;
                    }
                    if (slot < joint_dirty_low_)
                        joint_dirty_low_ = slot;
                    if (slot + 1 > joint_dirty_high_)
                        joint_dirty_high_ = slot + 1;
                }

                /**
                 * @brief Sends every staged write to the device as one range each.
                 *
                 * Const because a read has to be able to force it (see @ref
                 * read_bodies) and a read is const. Nothing about the *simulation*
                 * state changes here — what changes is only where the already-decided
                 * state is, which is what `mutable` is for.
                 */
                void flush_staged_writes() const
                {
                    if (body_dirty_)
                    {
                        bodies_->write_range(
                            Execution::ElementRange{
                                body_dirty_low_, body_dirty_high_ - body_dirty_low_},
                            body_mirror_.data() + body_dirty_low_);
                        body_dirty_ = false;
                    }
                    if (constraint_dirty_)
                    {
                        constraints_->write_range(
                            Execution::ElementRange{
                                constraint_dirty_low_,
                                constraint_dirty_high_ - constraint_dirty_low_},
                            constraint_mirror_.data() + constraint_dirty_low_);
                        constraint_dirty_ = false;
                    }
                    if (element_dirty_)
                    {
                        elements_->write_range(
                            Execution::ElementRange{
                                element_dirty_low_,
                                element_dirty_high_ - element_dirty_low_},
                            element_mirror_.data() + element_dirty_low_);
                        element_dirty_ = false;
                    }
                    if (joint_dirty_)
                    {
                        joints_->write_range(
                            Execution::ElementRange{
                                joint_dirty_low_, joint_dirty_high_ - joint_dirty_low_},
                            joint_mirror_.data() + joint_dirty_low_);
                        joint_dirty_ = false;
                    }
                }

                /**
                 * @brief Brings the solved joints back, so their load readout survives.
                 *
                 * Not optional, for the same reason the contact readback is not: the
                 * multipliers §10.4's force recovery is computed from are settled on
                 * the device, and a break threshold, a load gauge and a motor-effort
                 * readout are all that one quantity. One transfer per non-empty band
                 * rather than one per joint.
                 */
                void download_elements()
                {
                    for (std::size_t color = 0; color < elements_store_.color_count(); ++color)
                    {
                        const std::size_t live = elements_store_.band_size(color);
                        if (live == 0)
                            continue;
                        const std::size_t base = elements_store_.band_base(color);
                        const std::vector<Element> range = elements_->read_range(
                            Execution::ElementRange{base, live});
                        for (std::size_t i = 0; i < live && i < range.size(); ++i)
                            element_mirror_[base + i] = range[i];
                    }
                }

                /**
                 * @brief Removes an element whose handle is known live, keeping the band dense.
                 *
                 * @param handle The element to remove, in the store's own handle space.
                 */
                void remove_element_unchecked(ConstraintHandle handle)
                {
                    const std::size_t slot = elements_store_.slot_of(handle);
                    if (slot >= element_mirror_.size())
                        return;
                    const Element removed = element_mirror_[slot];
                    const ConstraintRemoval removal =
                        elements_store_.remove_bodies(handle, removed.vertex, 4);
                    if (!removal.removed)
                        return;
                    if (removal.slot != removal.moved_from)
                        stage_element(removal.slot, element_mirror_[removal.moved_from]);
                }

                /**
                 * @brief Removes every live element naming body slot @p body.
                 *
                 * All four vertices are tested, not two. That is P6-J1 restated at the
                 * removal end: an element left naming a freed slot would act on whichever
                 * particle claims it next, and testing only the first two would leave
                 * three quarters of them behind.
                 */
                void remove_elements_touching(std::uint32_t body)
                {
                    for (std::size_t color = 0; color < elements_store_.color_count(); ++color)
                    {
                        const std::size_t base = elements_store_.band_base(color);
                        // Downward, for the reason @ref remove_constraints_touching gives.
                        std::size_t offset = elements_store_.band_size(color);
                        while (offset > 0)
                        {
                            --offset;
                            const std::size_t slot = base + offset;
                            const Element& element = element_mirror_[slot];
                            if (element.vertex[0] != body && element.vertex[1] != body &&
                                element.vertex[2] != body && element.vertex[3] != body)
                                continue;
                            const ConstraintRemoval removal = elements_store_.remove_bodies(
                                elements_store_.handle_at(slot), element.vertex, 4);
                            if (removal.removed && removal.slot != removal.moved_from)
                                stage_element(removal.slot, element_mirror_[removal.moved_from]);
                        }
                    }
                }

                void download_joints()
                {
                    for (std::size_t color = 0; color < joints_store_.color_count(); ++color)
                    {
                        const std::size_t live = joints_store_.band_size(color);
                        if (live == 0)
                            continue;
                        const std::size_t base = joints_store_.band_base(color);
                        const std::vector<Joint> range = joints_->read_range(
                            Execution::ElementRange{base, live});
                        for (std::size_t i = 0; i < live && i < range.size(); ++i)
                            joint_mirror_[base + i] = range[i];
                    }
                }

                /**
                 * @brief Removes a joint whose handle is known live, keeping the band dense.
                 *
                 * @param handle The joint to remove, in the store's own handle space.
                 */
                void remove_joint_unchecked(ConstraintHandle handle)
                {
                    const std::size_t slot = joints_store_.slot_of(handle);
                    if (slot >= joint_mirror_.size())
                        return;
                    const Joint removed = joint_mirror_[slot];
                    const ConstraintRemoval removal =
                        joints_store_.remove(handle, removed.a, removed.b);
                    if (!removal.removed)
                        return;
                    if (removal.slot != removal.moved_from)
                        stage_joint(removal.slot, joint_mirror_[removal.moved_from]);
                }

                /** @brief Removes every live joint naming body slot @p body. */
                void remove_joints_touching(std::uint32_t body)
                {
                    for (std::size_t color = 0; color < joints_store_.color_count(); ++color)
                    {
                        const std::size_t base = joints_store_.band_base(color);
                        // Downward, for the reason @ref remove_constraints_touching
                        // gives: a swap-remove fills the vacated slot from the top of
                        // the band, which this walk has already examined.
                        std::size_t offset = joints_store_.band_size(color);
                        while (offset > 0)
                        {
                            --offset;
                            const std::size_t slot = base + offset;
                            const Joint& joint = joint_mirror_[slot];
                            if (joint.a != body && joint.b != body)
                                continue;
                            const ConstraintRemoval removal = joints_store_.remove(
                                joints_store_.handle_at(slot), joint.a, joint.b);
                            if (removal.removed && removal.slot != removal.moved_from)
                                stage_joint(removal.slot, joint_mirror_[removal.moved_from]);
                        }
                    }
                }

                /**
                 * @brief Sends this tick's contacts to the device, one band at a time.
                 *
                 * A band, not the whole buffer: the bands sit at fixed bases with
                 * their live entries dense from each base, so a scene resolving forty
                 * contacts across three colours pays three transfers of forty
                 * manifolds and not one transfer of sixteen thousand. And not one
                 * transfer per contact either, which is what writing through
                 * `add_contact` would have cost.
                 */
                void upload_contacts()
                {
                    for (std::size_t color = 0; color < contacts_store_.color_count();
                         ++color)
                    {
                        const std::size_t live = contacts_store_.band_size(color);
                        if (live == 0)
                            continue;
                        const std::size_t base = contacts_store_.band_base(color);
                        contacts_->write_range(
                            Execution::ElementRange{base, live},
                            contact_mirror_.data() + base);
                    }
                }

                /**
                 * @brief Brings the solved contacts back, so their accumulators survive.
                 *
                 * The impulses the solve settled on are what warm starting inherits
                 * next tick and what a contact event reports, and they were computed
                 * on the device — so this is not an optional readback. It is the price
                 * of contacts being a device-resident kind, and it is one transfer per
                 * non-empty band rather than one per contact.
                 */
                void download_contacts()
                {
                    for (std::size_t color = 0; color < contacts_store_.color_count();
                         ++color)
                    {
                        const std::size_t live = contacts_store_.band_size(color);
                        if (live == 0)
                            continue;
                        const std::size_t base = contacts_store_.band_base(color);
                        const std::vector<Contact> range = contacts_->read_range(
                            Execution::ElementRange{base, live});
                        for (std::size_t i = 0; i < live && i < range.size(); ++i)
                            contact_mirror_[base + i] = range[i];
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
                 * @param floor      A caller-imposed lower bound; zero imposes none.
                 * @return A substep count within the schedule's bounds.
                 */
                std::size_t derive_substep_count(T delta_time, std::size_t floor)
                {
                    const SubstepSchedule<T>& schedule = configuration_.substeps;
                    std::size_t minimum = schedule.minimum > 0 ? schedule.minimum : 1;
                    if (floor > minimum)
                        minimum = floor;
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
                    statistics_.joints = joints_store_.live_count();
                    statistics_.elements = elements_store_.live_count();
                    statistics_.constraints = constraints_store_.live_count() +
                                              statistics_.joints + statistics_.elements;
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

                    statistics_.manifolds = contacts_store_.live_count();
                    std::size_t points = 0;
                    for (const std::size_t slot : submission_slots_)
                        points += contact_mirror_[slot].manifold.point_count;
                    statistics_.contact_points = points;

                    // compile_count is the number this solver is held to: one
                    // after warm-up, for ever. compose_count belongs to the region
                    // graph and stays zero here, because a static composition is
                    // never recomposed — reporting the compile count under both
                    // names would hide exactly the distinction the two exist to draw.
                    statistics_.compile_count = graph_ ? graph_->compile_count() : 0;
                    statistics_.compose_count = 0;

                    // The device half of the tick, measured by the runtime rather than
                    // estimated: the composition holds nothing but physics stages, so
                    // the run's own wall clock *is* the solve's cost. Gated on the
                    // configured flag so a scene nobody is profiling pays not even this
                    // one copy, and so the panel's "no timings" branch means what it
                    // says. Not split per stage — see PhysicsStageTimings.
                    if (configuration_.profiling)
                        statistics_.timings.solve_ms = T(last_report_.total_duration_ms);
                }

                Execution::Context& context_;
                PhysicsConfigurationT<T> configuration_;

                HandleTable<BodyTag> body_slots_;
                ConstraintStore constraints_store_;
                // Colours into the constraint store's colourer, so the two persistent
                // kinds colour over their union (§6.3). Declared after it because the
                // sharing is a reference taken at construction.
                ConstraintStore joints_store_;
                // The four-body kind, colouring into the same union for the same
                // reason: an element and a distance constraint that share a particle
                // must not share a colour.
                ConstraintStore elements_store_;
                ContactStore contacts_store_;

                std::vector<Constraint> constraint_mirror_;
                // The joint mirror is not staging alone, unlike the others: a joint is
                // written by the host *and* by the device, so this holds whichever is
                // newer and is what `read_joint` answers from.
                std::vector<Joint> joint_mirror_;
                // Nor is the element mirror staging alone, for the same reason: the host
                // owns an element's rest state and its Lame pair, the device owns its
                // multipliers, and `read_element` answers from whichever is newer.
                std::vector<Element> element_mirror_;
                std::vector<Contact> contact_mirror_;
                std::vector<std::size_t> submission_slots_;
                std::vector<RigidBodyT<T>> body_mirror_;

                // Mutable because a const read flushes them; see @ref flush_staged_writes.
                mutable bool body_dirty_ = false;
                mutable std::size_t body_dirty_low_ = 0;
                mutable std::size_t body_dirty_high_ = 0;
                mutable bool constraint_dirty_ = false;
                mutable std::size_t constraint_dirty_low_ = 0;
                mutable std::size_t constraint_dirty_high_ = 0;
                mutable bool joint_dirty_ = false;
                mutable std::size_t joint_dirty_low_ = 0;
                mutable std::size_t joint_dirty_high_ = 0;
                mutable bool element_dirty_ = false;
                mutable std::size_t element_dirty_low_ = 0;
                mutable std::size_t element_dirty_high_ = 0;

                std::size_t body_high_water_ = 0;
                std::size_t live_substeps_ = 1;

                // The two staged-into buffers are mutable for the same reason the
                // dirty marks are: a const read flushes them, and moving already-
                // decided state to where it is readable is not a change of state.
                mutable std::optional<Execution::Buffer<RigidBodyT<T>>> bodies_;
                mutable std::optional<Execution::Buffer<Constraint>> constraints_;
                mutable std::optional<Execution::Buffer<Joint>> joints_;
                mutable std::optional<Execution::Buffer<Element>> elements_;
                std::optional<Execution::Buffer<T>> lambdas_;
                std::optional<Execution::Buffer<Contact>> contacts_;
                std::optional<Execution::Buffer<T>> motion_;
                std::optional<Execution::Buffer<StepUniforms<T>>> uniforms_;
                std::optional<Execution::Buffer<T>> motion_maximum_;
                std::optional<Execution::Graph> graph_;

                Execution::RunReport last_report_{};
                PhysicsStatisticsT<T> statistics_{};
        };
    } // namespace Physics
} // namespace SushiEngine
