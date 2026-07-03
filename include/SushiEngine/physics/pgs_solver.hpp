/**************************************************************************/
/* pgs_solver.hpp                                                        */
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
#include <SushiEngine/physics/constraint.hpp>
#include <SushiEngine/physics/graph_coloring.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief Projects a distance constraint, moving its bodies toward rest length.
         *
         * The position-based projection used by the solver: it splits the length
         * error between the two bodies in proportion to their inverse masses, so a
         * pinned body (inverse mass zero) stays put and the other takes the whole
         * correction. A captureless functor, so it is device-copyable and runs inside
         * a kernel. New constraint types provide their own projection of this shape.
         */
        struct DistanceProjection
        {
            /**
             * @brief Applies one projection of constraint @p c to the body positions.
             * @param c        The constraint being satisfied.
             * @param position The body position array, updated in place.
             * @param inv_mass The per-body inverse masses (zero pins a body).
             */
            void operator()(const DistanceConstraint& c, Vector3* position,
                            const Scalar* inv_mass) const
            {
                const Vector3 pa = position[c.a];
                const Vector3 pb = position[c.b];
                const Vector3 delta = pa - pb;
                const Scalar dist =
                    sycl::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
                if (dist <= Scalar(1e-8))
                    return;

                const Scalar wa = inv_mass[c.a];
                const Scalar wb = inv_mass[c.b];
                const Scalar w = wa + wb;
                if (w <= Scalar(0))
                    return;

                // scale * delta is the full length error along the body axis; each
                // body takes its inverse-mass share of it.
                const Scalar scale = (dist - c.rest_length) / (dist * w);
                position[c.a] = pa - delta * (wa * scale);
                position[c.b] = pb + delta * (wb * scale);
            }
        };

        /**
         * @brief A Projected Gauss-Seidel constraint solver compiled to a task graph.
         *
         * The solver colours the constraints so that each colour is a conflict-free
         * batch, then builds a graph in which every colour is one parallel task that
         * projects all of its constraints at once. Because every colour reads and
         * writes the shared position array, the runtime's dependency tracker orders
         * the colours into a sequential sweep — colour k+1 after colour k — which is
         * exactly Gauss-Seidel across colours while staying fully parallel within a
         * colour. The sweep is repeated for the requested iteration count, and the
         * whole graph is compiled once and replayed every frame.
         *
         * The solver owns no engine concept beyond bodies and constraints: it takes a
         * position array, an inverse-mass array, and a projection functor, so a new
         * constraint type is supported by passing its own POD and projection — the
         * colouring and the graph structure are reused unchanged.
         *
         * @tparam Constraint A constraint type exposing body indices `a` and `b`.
         */
        template <typename Constraint>
        class ConstraintSolver
        {
            public:
                /**
                 * @brief Colours the constraints and builds the replayable solve graph.
                 * @tparam Projection A device-callable projection for @p Constraint.
                 * @param runtime     The runtime that backs the graph and buffers.
                 * @param position    Body positions, updated in place each solve.
                 * @param inv_mass    Per-body inverse masses (zero pins a body).
                 * @param constraints The constraints to satisfy.
                 * @param body_count  Number of bodies.
                 * @param iterations  Gauss-Seidel sweeps per solve.
                 * @param projection  The per-constraint projection to apply.
                 */
                template <typename Projection>
                ConstraintSolver(SushiRuntime::API::Runtime& runtime,
                                 SushiRuntime::API::Buffer<Vector3>& position,
                                 SushiRuntime::API::Buffer<Scalar>& inv_mass,
                                 const std::vector<Constraint>& constraints,
                                 std::size_t body_count, std::size_t iterations,
                                 Projection projection)
                    : runtime_(runtime),
                      position_(position),
                      inv_mass_(inv_mass),
                      iterations_(iterations),
                      colors_(color_constraints(constraints, body_count))
                {
                    for (const std::vector<std::uint32_t>& batch : colors_)
                    {
                        SushiRuntime::API::Buffer<Constraint> buffer =
                            runtime.buffer<Constraint>(batch.size());
                        for (std::size_t k = 0; k < batch.size(); ++k)
                            buffer[k] = constraints[batch[k]];
                        color_buffers_.push_back(std::move(buffer));
                    }
                    build_graph(projection);
                }

                /**
                 * @brief Runs the iteration sweeps once over the current positions.
                 * @return The run report for the solve.
                 */
                SushiRuntime::RunReport solve()
                {
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
                        for (std::size_t color = 0; color < color_buffers_.size(); ++color)
                        {
                            const std::size_t n = colors_[color].size();
                            if (n == 0)
                                continue;

                            SushiRuntime::API::Buffer<Constraint>& batch = color_buffers_[color];
                            graph_->add(
                                SushiRuntime::Extent{n},
                                SushiRuntime::InOut(position_),
                                SushiRuntime::In(inv_mass_),
                                SushiRuntime::In(batch),
                                [projection](sycl::id<1> id, Vector3* position,
                                             const Scalar* inv_mass, const Constraint* cons)
                                {
                                    projection(cons[id[0]], position, inv_mass);
                                });
                        }
                }

                SushiRuntime::API::Runtime& runtime_;
                SushiRuntime::API::Buffer<Vector3>& position_;
                SushiRuntime::API::Buffer<Scalar>& inv_mass_;
                std::size_t iterations_;
                ColorBatches colors_;
                std::vector<SushiRuntime::API::Buffer<Constraint>> color_buffers_;
                std::optional<SushiRuntime::API::Graph> graph_;
        };
    } // namespace Physics
} // namespace SushiEngine
