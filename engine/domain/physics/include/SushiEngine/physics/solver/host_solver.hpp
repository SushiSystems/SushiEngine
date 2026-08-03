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

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/constraints/distance_projection.hpp>
#include <SushiEngine/physics/constraints/joint_projection.hpp>
#include <SushiEngine/physics/constraints/beam_projection.hpp>
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
         * @brief The reference XPBD solver: same layout, same arithmetic, host loops.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        class HostXpbdSolver final : public IConstraintSolver<T>
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

                /** @brief The structural persistent kind this solver admits (§11.1). */
                using Beam = BeamConstraintT<T>;

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
                      joints_store_(constraints_store_.shared_coloring(),
                                    configuration.capacities.joints,
                                    configuration.capacities.colors),
                      elements_store_(constraints_store_.shared_coloring(),
                                      configuration.capacities.elements,
                                      configuration.capacities.colors),
                      beams_store_(constraints_store_.shared_coloring(),
                                   configuration.capacities.beams,
                                   configuration.capacities.colors),
                      contacts_store_(configuration.capacities.bodies,
                                      configuration.capacities.contacts,
                                      configuration.capacities.colors),
                      constraints_(constraints_store_.band_capacity() *
                                   constraints_store_.color_count()),
                      joints_(joints_store_.band_capacity() * joints_store_.color_count()),
                      elements_(elements_store_.band_capacity() *
                                elements_store_.color_count()),
                      beams_(beams_store_.band_capacity() * beams_store_.color_count()),
                      contacts_(contacts_store_.capacity()),
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
                    remove_joints_touching(handle.index);
                    remove_elements_touching(handle.index);
                    remove_beams_touching(handle.index);

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
                    elements_[placement.slot] = element;
                    return placement.handle;
                }

                /** @copydoc IConstraintSolver::remove_element */
                bool remove_element(ConstraintHandle handle) override
                {
                    const std::size_t slot = elements_store_.slot_of(handle);
                    if (slot >= elements_.size())
                        return false;
                    const Element removed = elements_[slot];
                    const ConstraintRemoval removal =
                        elements_store_.remove_bodies(handle, removed.vertex, 4);
                    if (!removal.removed)
                        return false;
                    if (removal.slot != removal.moved_from)
                        elements_[removal.slot] = elements_[removal.moved_from];
                    return true;
                }

                /** @copydoc IConstraintSolver::read_element */
                bool read_element(ConstraintHandle handle, Element& element) const override
                {
                    if (!elements_store_.alive(handle))
                        return false;
                    const std::size_t slot = elements_store_.slot_of(handle);
                    if (slot >= elements_.size())
                        return false;
                    element = elements_[slot];
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

                /** @copydoc IConstraintSolver::add_beam */
                ConstraintHandle add_beam(const Beam& beam) override
                {
                    const ConstraintPlacement placement = beams_store_.place(beam.a, beam.b);
                    if (!placement.handle.valid())
                    {
                        ++statistics_.capacity_overflows;
                        return placement.handle;
                    }
                    beams_[placement.slot] = beam;
                    return placement.handle;
                }

                /** @copydoc IConstraintSolver::remove_beam */
                bool remove_beam(ConstraintHandle handle) override
                {
                    const std::size_t slot = beams_store_.slot_of(handle);
                    if (slot >= beams_.size())
                        return false;
                    const Beam removed = beams_[slot];
                    const ConstraintRemoval removal =
                        beams_store_.remove(handle, removed.a, removed.b);
                    if (!removal.removed)
                        return false;
                    if (removal.slot != removal.moved_from)
                        beams_[removal.slot] = beams_[removal.moved_from];
                    return true;
                }

                /** @copydoc IConstraintSolver::read_beam */
                bool read_beam(ConstraintHandle handle, Beam& beam) const override
                {
                    if (!beams_store_.alive(handle))
                        return false;
                    const std::size_t slot = beams_store_.slot_of(handle);
                    if (slot >= beams_.size())
                        return false;
                    beam = beams_[slot];
                    return true;
                }

                /** @copydoc IConstraintSolver::write_beam */
                bool write_beam(ConstraintHandle handle, const Beam& beam) override
                {
                    if (!beams_store_.alive(handle))
                        return false;
                    const std::size_t slot = beams_store_.slot_of(handle);
                    if (slot >= beams_.size())
                        return false;
                    beams_[slot] = beam;
                    return true;
                }

                /** @copydoc IConstraintSolver::beam_capacity */
                std::size_t beam_capacity() const noexcept override
                {
                    return beams_store_.capacity();
                }

                /** @brief Live beams in colour @p color. */
                std::size_t beam_color_size(std::size_t color) const noexcept
                {
                    return beams_store_.band_size(color);
                }

                /** @copydoc IConstraintSolver::add_joint */
                JointHandle add_joint(const Joint& joint) override
                {
                    const ConstraintPlacement placement =
                        joints_store_.place(joint.a, joint.b);
                    if (!placement.handle.valid())
                    {
                        ++statistics_.capacity_overflows;
                        return JointHandle{};
                    }
                    joints_[placement.slot] = joint;
                    return JointHandle{placement.handle.index, placement.handle.generation};
                }

                /** @copydoc IConstraintSolver::remove_joint */
                bool remove_joint(JointHandle handle) override
                {
                    const ConstraintHandle stored{handle.index, handle.generation};
                    const std::size_t slot = joints_store_.slot_of(stored);
                    if (slot >= joints_.size())
                        return false;
                    const Joint removed = joints_[slot];
                    const ConstraintRemoval removal =
                        joints_store_.remove(stored, removed.a, removed.b);
                    if (!removal.removed)
                        return false;
                    if (removal.slot != removal.moved_from)
                        joints_[removal.slot] = joints_[removal.moved_from];
                    return true;
                }

                /** @copydoc IConstraintSolver::read_joint */
                bool read_joint(JointHandle handle, Joint& joint) const override
                {
                    const ConstraintHandle stored{handle.index, handle.generation};
                    const std::size_t slot = joints_store_.slot_of(stored);
                    if (slot >= joints_.size())
                        return false;
                    joint = joints_[slot];
                    return true;
                }

                /** @copydoc IConstraintSolver::write_joint */
                bool write_joint(JointHandle handle, const Joint& joint) override
                {
                    const ConstraintHandle stored{handle.index, handle.generation};
                    const std::size_t slot = joints_store_.slot_of(stored);
                    if (slot >= joints_.size())
                        return false;
                    joints_[slot] = joint;
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
                    if (!contact_slots_valid(contact, bodies_.size()))
                        return false;
                    const ContactPlacement placement = contacts_store_.place(
                        contact.a, contact.b, constraints_store_.coloring());
                    if (!placement.placed)
                    {
                        ++statistics_.capacity_overflows;
                        return false;
                    }
                    contacts_[placement.slot] = contact;
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
                    contact = contacts_[submission_slots_[index]];
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

                /** @copydoc IConstraintSolver::body_handle */
                BodyHandle body_handle(std::size_t slot) const override
                {
                    if (slot >= body_slots_.capacity())
                        return BodyHandle{};
                    return body_slots_.handle_of(std::uint32_t(slot));
                }

                /**
                 * @brief Advances every live body by one tick, sequentially.
                 *
                 * The order is the runtime-backed solver's order, written out: for
                 * each substep, prepare every contact, predict every body, walk the
                 * colours in ascending order projecting each one's live constraint
                 * band and then its live contact band in slot order, derive every
                 * body's velocity, and finally run the contact velocity pass over the
                 * same colours. Nothing about that sequence is a host convenience — it
                 * is the schedule the graph encodes, and the conformance suite exists
                 * to catch the two drifting apart.
                 *
                 * @param parameters What the tick is told from outside the simulation.
                 */
                void step(const StepParameters<T>& parameters) override
                {
                    // The reference implementation measures its own solve for the same
                    // reason it exists at all: a number from one solver that the other
                    // cannot produce is a number nobody can compare.
                    const std::chrono::steady_clock::time_point began =
                        configuration_.profiling ? std::chrono::steady_clock::now()
                                                 : std::chrono::steady_clock::time_point{};

                    live_substeps_ = derive_substep_count(parameters.delta_time,
                                                         parameters.substep_floor);
                    const T h =
                        parameters.delta_time / T(live_substeps_ > 0 ? live_substeps_ : 1);

                    for (std::size_t substep = 0; substep < live_substeps_; ++substep)
                    {
                        // Before predict, because the arrival speed restitution is a
                        // statement about is the speed the body had when the substep
                        // began, and predict is the first thing that changes it.
                        for_each_contact([&](Contact& contact)
                                         {
                                             ContactPreparationT<T> prepare;
                                             prepare(contact, bodies_.data(), substep == 0);
                                         });

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

                        // The beams, immediately after the distance lattice, because a
                        // beam *is* an axial row: grouping the two kinds that say "these
                        // two points are this far apart" keeps the schedule readable and
                        // costs nothing, since within a colour no two of either share a
                        // body.
                        for_each_beam([&](Beam& beam)
                                      {
                                          BeamProjectionT<T> projection;
                                          projection(beam, bodies_.data(), h, substep == 0);
                                      });

                        // The elements, after the distance lattice and before the
                        // joints. Both are constitutive — they say what the material
                        // *is* — so they belong together and ahead of the articulation
                        // that hangs off it. Deviatoric then hydrostatic, per element,
                        // in slot order: the same sweep `FiniteElementModel` runs, and
                        // the reason a soft body solved here and solved there converge
                        // to the same shape.
                        for_each_element([&](Element& element)
                                         {
                                             // Reset per substep, not per tick: one
                                             // iteration per substep is XPBD's
                                             // small-step arrangement, so nothing may
                                             // carry across — the same rule the
                                             // distance kind expresses by starting its
                                             // multiplier at zero every time.
                                             element.deviatoric_lambda = 0;
                                             element.hydrostatic_lambda = 0;
                                             project_fem_deviatoric(bodies_.data(), element, h);
                                             project_fem_hydrostatic(bodies_.data(), element, h);
                                         });

                        // After the distance constraints and before the contacts: a
                        // joint is a structural constraint and a contact is a reactive
                        // one, so the assembly is assembled before it is pushed on.
                        for_each_joint([&](Joint& joint)
                                       {
                                           JointProjectionT<T> projection;
                                           projection(joint, bodies_.data(), h, substep == 0);
                                       });

                        // After the persistent kinds, because both correct positions
                        // and a substep's positional projections belong together;
                        // separately from them, because the two kinds have their own
                        // bands and the graph gives each its own node (§6.3).
                        for_each_contact([&](Contact& contact)
                                         {
                                             ContactPositionProjectionT<T> projection;
                                             projection(contact, bodies_.data());
                                         });

                        for (std::size_t i = 0; i < body_high_water_; ++i)
                            update_velocity(bodies_[i], h);

                        // The velocity pass, in the same order as the positional one:
                        // beam damping, joint rate drives and friction, then contact
                        // dynamic friction and restitution. All are statements about a
                        // velocity that does not exist until the pose change has been
                        // read back as one.
                        for_each_beam([&](Beam& beam)
                                      {
                                          BeamVelocityProjectionT<T> projection;
                                          projection(beam, bodies_.data(), h);
                                      });

                        for_each_joint([&](Joint& joint)
                                       {
                                           JointVelocityProjectionT<T> projection;
                                           projection(joint, bodies_.data(), h);
                                       });

                        for_each_contact([&](Contact& contact)
                                         {
                                             ContactVelocityProjectionT<T> projection;
                                             projection(contact, bodies_.data(), h);
                                         });
                    }

                    measure_motion();
                    refresh_statistics();

                    if (configuration_.profiling)
                    {
                        const std::chrono::duration<double, std::milli> elapsed =
                            std::chrono::steady_clock::now() - began;
                        statistics_.timings.solve_ms = T(elapsed.count());
                    }
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

                /** @copydoc IConstraintSolver::contact_capacity */
                std::size_t contact_capacity() const noexcept override
                {
                    return contacts_store_.capacity();
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

                /** @brief Contacts submitted into colour @p color this tick. */
                std::size_t contact_color_size(std::size_t color) const noexcept
                {
                    return contacts_store_.band_size(color);
                }

            private:
                /**
                 * @brief Applies @p visit to every submitted contact, in solve order.
                 *
                 * Solve order — colour ascending, then slot within the band — and not
                 * submission order, because Gauss-Seidel makes order an input to the
                 * answer, and the graph's order is the one the colours impose.
                 *
                 * @param visit A callable taking `Contact&`.
                 */
                template <typename Visit>
                void for_each_contact(Visit visit)
                {
                    for (std::size_t color = 0; color < contacts_store_.color_count();
                         ++color)
                    {
                        const std::size_t base = contacts_store_.band_base(color);
                        const std::size_t live = contacts_store_.band_size(color);
                        for (std::size_t offset = 0; offset < live; ++offset)
                            visit(contacts_[base + offset]);
                    }
                }

                /**
                 * @brief Applies @p visit to every live joint, in solve order.
                 *
                 * Colour ascending, then slot within the band — the graph's order, for
                 * the same reason @ref for_each_contact walks that order rather than
                 * insertion order.
                 *
                 * @param visit A callable taking `Joint&`.
                 */
                template <typename Visit>
                void for_each_joint(Visit visit)
                {
                    for (std::size_t color = 0; color < joints_store_.color_count(); ++color)
                    {
                        const std::size_t base = joints_store_.band_base(color);
                        const std::size_t live = joints_store_.band_size(color);
                        for (std::size_t offset = 0; offset < live; ++offset)
                            visit(joints_[base + offset]);
                    }
                }

                /**
                 * @brief Applies @p visit to every live element, in solve order.
                 *
                 * @param visit A callable taking `Element&`.
                 */
                template <typename Visit>
                void for_each_element(Visit visit)
                {
                    for (std::size_t color = 0; color < elements_store_.color_count(); ++color)
                    {
                        const std::size_t base = elements_store_.band_base(color);
                        const std::size_t live = elements_store_.band_size(color);
                        for (std::size_t offset = 0; offset < live; ++offset)
                            visit(elements_[base + offset]);
                    }
                }

                /**
                 * @brief Removes every live element naming body slot @p body.
                 *
                 * All four vertices are tested, not two. That is the whole of P6-J1
                 * restated at the removal end: an element left naming a freed slot would
                 * act on whichever particle claims it next, and testing only `vertex[0]`
                 * and `vertex[1]` would leave three quarters of them behind.
                 */
                void remove_elements_touching(std::uint32_t body)
                {
                    for (std::size_t color = 0; color < elements_store_.color_count(); ++color)
                    {
                        const std::size_t base = elements_store_.band_base(color);
                        std::size_t offset = elements_store_.band_size(color);
                        while (offset > 0)
                        {
                            --offset;
                            const std::size_t slot = base + offset;
                            const Element& element = elements_[slot];
                            if (element.vertex[0] != body && element.vertex[1] != body &&
                                element.vertex[2] != body && element.vertex[3] != body)
                                continue;
                            const ConstraintRemoval removal = elements_store_.remove_bodies(
                                elements_store_.handle_at(slot), element.vertex, 4);
                            if (removal.removed && removal.slot != removal.moved_from)
                                elements_[removal.slot] = elements_[removal.moved_from];
                        }
                    }
                }

                /**
                 * @brief Applies @p visit to every live beam, in solve order.
                 *
                 * @param visit A callable taking `Beam&`.
                 */
                template <typename Visit>
                void for_each_beam(Visit visit)
                {
                    for (std::size_t color = 0; color < beams_store_.color_count(); ++color)
                    {
                        const std::size_t base = beams_store_.band_base(color);
                        const std::size_t live = beams_store_.band_size(color);
                        for (std::size_t offset = 0; offset < live; ++offset)
                            visit(beams_[base + offset]);
                    }
                }

                /** @brief Removes every live beam naming body slot @p body. */
                void remove_beams_touching(std::uint32_t body)
                {
                    for (std::size_t color = 0; color < beams_store_.color_count(); ++color)
                    {
                        const std::size_t base = beams_store_.band_base(color);
                        std::size_t offset = beams_store_.band_size(color);
                        while (offset > 0)
                        {
                            --offset;
                            const std::size_t slot = base + offset;
                            const Beam& beam = beams_[slot];
                            if (beam.a != body && beam.b != body)
                                continue;
                            const ConstraintRemoval removal = beams_store_.remove(
                                beams_store_.handle_at(slot), beam.a, beam.b);
                            if (removal.removed && removal.slot != removal.moved_from)
                                beams_[removal.slot] = beams_[removal.moved_from];
                        }
                    }
                }

                /** @brief Removes every live joint naming body slot @p body. */
                void remove_joints_touching(std::uint32_t body)
                {
                    for (std::size_t color = 0; color < joints_store_.color_count(); ++color)
                    {
                        const std::size_t base = joints_store_.band_base(color);
                        std::size_t offset = joints_store_.band_size(color);
                        while (offset > 0)
                        {
                            --offset;
                            const std::size_t slot = base + offset;
                            const Joint& joint = joints_[slot];
                            if (joint.a != body && joint.b != body)
                                continue;
                            const ConstraintRemoval removal = joints_store_.remove(
                                joints_store_.handle_at(slot), joint.a, joint.b);
                            if (removal.removed && removal.slot != removal.moved_from)
                                joints_[removal.slot] = joints_[removal.moved_from];
                        }
                    }
                }

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
                 * @param floor      A caller-imposed lower bound; zero imposes none.
                 * @return A substep count within the schedule's bounds.
                 */
                std::size_t derive_substep_count(T delta_time, std::size_t floor) const
                {
                    const SubstepSchedule<T>& schedule = configuration_.substeps;
                    std::size_t minimum = schedule.minimum > 0 ? schedule.minimum : 1;
                    if (floor > minimum)
                        minimum = floor;
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
                    statistics_.joints = joints_store_.live_count();
                    statistics_.elements = elements_store_.live_count();
                    statistics_.beams = beams_store_.live_count();
                    statistics_.constraints = constraints_store_.live_count() +
                                              statistics_.joints + statistics_.elements +
                                              statistics_.beams;
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
                        points += contacts_[slot].manifold.point_count;
                    statistics_.contact_points = points;

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
                // Colours into the constraint store's colourer, so the two persistent
                // kinds colour over their union (§6.3). Declared after it because the
                // sharing is a reference taken at construction.
                ConstraintStore joints_store_;
                // The four-body kind, colouring into the same union for the same
                // reason: an element and a distance constraint that share a particle
                // must not share a colour.
                ConstraintStore elements_store_;
                // The structural kind, colouring into the same union: a beam and a
                // contact that share a node must not share a colour any more than a
                // joint and a distance constraint may.
                ConstraintStore beams_store_;
                ContactStore contacts_store_;
                std::vector<Constraint> constraints_;
                std::vector<Joint> joints_;
                std::vector<Element> elements_;
                std::vector<Beam> beams_;
                std::vector<Contact> contacts_;
                std::vector<std::size_t> submission_slots_;
                std::vector<RigidBodyT<T>> bodies_;

                std::size_t body_high_water_ = 0;
                std::size_t live_substeps_ = 1;
                T motion_maximum_ = T(0);
                PhysicsStatisticsT<T> statistics_{};
        };
    } // namespace Physics
} // namespace SushiEngine
