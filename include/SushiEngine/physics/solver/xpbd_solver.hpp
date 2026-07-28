/**************************************************************************/
/* xpbd_solver.hpp                                                       */
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

#include <cstddef>
#include <optional>
#include <vector>

#include <sycl/sycl.hpp>

#include <SushiRuntime/SushiRuntime.h>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/solver/graph_coloring.hpp>
#include <SushiEngine/physics/core/rigid_body.hpp>
#include <SushiEngine/physics/constraints/distance_projection.hpp>
#include <SushiEngine/physics/constraints/xpbd_constraint.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief A compliant (XPBD) constraint solver over rigid bodies, compiled to a graph.
         *
         * The rigid-body counterpart of `ConstraintSolver`: same graph-colouring,
         * same replay-once-compiled structure, but the shared resource is a single
         * `RigidBody` array (position, orientation, and their generalized inverse
         * mass together) instead of separate position/inverse-mass arrays, and each
         * constraint carries a per-step Lagrange multiplier accumulator that must be
         * reset to zero before each `solve()` — XPBD's compliance term depends on
         * that running total within the step, not across steps.
         *
         * @tparam Constraint A constraint type exposing body indices `a`/`b`, as
         * `XpbdDistanceConstraint` does.
         */
        template <typename Constraint>
        class XpbdSolver
        {
            public:
                /** @brief The scalar precision, derived from the constraint type. */
                using Real = typename Constraint::Real;

                /**
                 * @brief Colours the constraints and builds the replayable solve graph.
                 * @tparam Projection A device-callable projection for @p Constraint.
                 * @param runtime     The runtime that backs the graph and buffers.
                 * @param bodies      The rigid bodies the constraints act on, updated in place.
                 * @param constraints The constraints to satisfy.
                 * @param body_count  Number of bodies.
                 * @param iterations  Gauss-Seidel sweeps per solve.
                 * @param h           Sub-step duration passed to the projection, in seconds.
                 * @param projection  The per-constraint projection to apply.
                 */
                template <typename Projection>
                XpbdSolver(SushiRuntime::API::Runtime& runtime,
                          SushiRuntime::API::Buffer<RigidBodyT<Real>>& bodies,
                          const std::vector<Constraint>& constraints,
                          std::size_t body_count, std::size_t iterations, Real h,
                          Projection projection)
                    : runtime_(runtime),
                      bodies_(bodies),
                      iterations_(iterations),
                      h_(h),
                      colors_(color_constraints(constraints, body_count))
                {
                    for (const std::vector<std::uint32_t>& batch : colors_)
                    {
                        SushiRuntime::API::Buffer<Constraint> constraint_buffer =
                            runtime.buffer<Constraint>(batch.size());
                        SushiRuntime::API::Buffer<Real> lambda_buffer =
                            runtime.buffer<Real>(batch.size());
                        for (std::size_t k = 0; k < batch.size(); ++k)
                        {
                            constraint_buffer[k] = constraints[batch[k]];
                            lambda_buffer[k] = Real(0);
                        }
                        constraint_buffers_.push_back(std::move(constraint_buffer));
                        lambda_buffers_.push_back(std::move(lambda_buffer));
                    }
                    build_graph(projection);
                }

                /**
                 * @brief Runs the iteration sweeps once, as one XPBD step.
                 *
                 * Resets every constraint's Lagrange multiplier to zero first — the
                 * compliance term is only meaningful accumulated within a single step —
                 * then replays the compiled colour sweep.
                 *
                 * @return The run report for the solve.
                 */
                SushiRuntime::RunReport solve()
                {
                    for (SushiRuntime::API::Buffer<Real>& lambda_buffer : lambda_buffers_)
                        for (std::size_t k = 0; k < lambda_buffer.size(); ++k)
                            lambda_buffer[k] = Real(0);

                    if (!graph_ || graph_->size() == 0)
                        return SushiRuntime::RunReport{};
                    return graph_->run();
                }

                /** @brief Number of colours the constraints partitioned into. */
                std::size_t color_count() const noexcept { return colors_.size(); }

                /** @brief The constraint indices grouped by colour, for reference checks. */
                const ColorBatches& colors() const noexcept { return colors_; }

                /** @brief Times the solve graph has been compiled (1 after warm-up). */
                std::size_t compile_count() const noexcept
                {
                    return graph_ ? graph_->compile_count() : 0;
                }

            private:
                /**
                 * @brief Emits the iteration-by-colour node grid into the graph.
                 * @tparam Projection The projection functor, baked into each node.
                 * @param projection The projection applied per constraint.
                 */
                template <typename Projection>
                void build_graph(Projection projection)
                {
                    graph_.emplace(runtime_.graph());
                    for (std::size_t iteration = 0; iteration < iterations_; ++iteration)
                        for (std::size_t color = 0; color < constraint_buffers_.size(); ++color)
                        {
                            const std::size_t n = colors_[color].size();
                            if (n == 0)
                                continue;

                            SushiRuntime::API::Buffer<Constraint>& cbuf = constraint_buffers_[color];
                            SushiRuntime::API::Buffer<Real>& lbuf = lambda_buffers_[color];
                            const Real h = h_;
                            graph_->add(
                                SushiRuntime::Extent{n},
                                SushiRuntime::InOut(bodies_),
                                SushiRuntime::In(cbuf),
                                SushiRuntime::InOut(lbuf),
                                [projection, h](sycl::id<1> id, RigidBodyT<Real>* bodies,
                                                const Constraint* cons, Real* lambda)
                                {
                                    projection(cons[id[0]], bodies, lambda[id[0]], h);
                                });
                        }
                }

                SushiRuntime::API::Runtime& runtime_;
                SushiRuntime::API::Buffer<RigidBodyT<Real>>& bodies_;
                std::size_t iterations_;
                Real h_;
                ColorBatches colors_;
                std::vector<SushiRuntime::API::Buffer<Constraint>> constraint_buffers_;
                std::vector<SushiRuntime::API::Buffer<Real>> lambda_buffers_;
                std::optional<SushiRuntime::API::Graph> graph_;
        };
    } // namespace Physics
} // namespace SushiEngine
