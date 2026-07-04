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
#include <SushiEngine/physics/graph_coloring.hpp>
#include <SushiEngine/physics/rigid_body.hpp>
#include <SushiEngine/physics/xpbd_constraint.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief Projects one compliant rigid-body distance constraint (XPBD).
         *
         * Implements the generalized positional constraint of Müller et al.,
         * "Detailed Rigid Body Simulation with XPBD": the attachment points are
         * pulled toward `rest_length` apart, with the correction split between the
         * two bodies' linear and angular degrees of freedom by their generalized
         * inverse mass, and scaled by the constraint's compliance so stiffness is
         * independent of iteration count and step size. `compliance == 0` recovers
         * an infinitely stiff (hard) constraint — the rigid-body generalization of
         * `DistanceProjection`. A captureless functor, so it is device-copyable.
         */
        template <typename T>
        struct XpbdDistanceProjectionT
        {
            /**
             * @brief Applies one XPBD iteration of constraint @p c.
             * @param c       The constraint being satisfied.
             * @param bodies  The rigid-body array, updated in place.
             * @param lambda  This constraint's accumulated Lagrange multiplier for the
             *                current step; the caller resets it to zero once per step.
             * @param h       The sub-step duration used for this step, in seconds (> 0).
             */
            void operator()(const XpbdDistanceConstraintT<T>& c, RigidBodyT<T>* bodies,
                            T& lambda, T h) const
            {
                RigidBodyT<T>& body_a = bodies[c.a];
                RigidBodyT<T>& body_b = bodies[c.b];

                const Vector3T<T> anchor_a = rotate(body_a.orientation, c.local_anchor_a);
                const Vector3T<T> anchor_b = rotate(body_b.orientation, c.local_anchor_b);
                const Vector3T<T> p1 = body_a.position + anchor_a;
                const Vector3T<T> p2 = body_b.position + anchor_b;
                const Vector3T<T> d = p2 - p1;
                const T len = length(d);
                if (len <= T(1e-8))
                    return;
                const Vector3T<T> n = d * (T(1) / len);
                const T error = len - c.rest_length;

                // r x n, expressed in each body's own local frame (see rotate()'s
                // doc comment: R(a x b) = (Ra) x (Rb), so rotating the world cross
                // product back by the body's own orientation gives the same result
                // as crossing the local anchor with the locally-expressed normal).
                const Vector3T<T> rxn_a =
                    rotate(conjugate(body_a.orientation), cross(anchor_a, n));
                const Vector3T<T> rxn_b =
                    rotate(conjugate(body_b.orientation), cross(anchor_b, n));

                const Vector3T<T> iixn_a{body_a.inv_inertia.x * rxn_a.x,
                                  body_a.inv_inertia.y * rxn_a.y,
                                  body_a.inv_inertia.z * rxn_a.z};
                const Vector3T<T> iixn_b{body_b.inv_inertia.x * rxn_b.x,
                                  body_b.inv_inertia.y * rxn_b.y,
                                  body_b.inv_inertia.z * rxn_b.z};

                const T w = body_a.inv_mass + body_b.inv_mass +
                                 dot(rxn_a, iixn_a) + dot(rxn_b, iixn_b);
                if (w <= T(0))
                    return;

                const T alpha_tilde = h > T(0) ? c.compliance / (h * h) : T(0);
                const T delta_lambda = (-error - alpha_tilde * lambda) / (w + alpha_tilde);
                lambda += delta_lambda;

                const Vector3T<T> impulse = n * delta_lambda;
                body_a.position = body_a.position - impulse * body_a.inv_mass;
                body_b.position = body_b.position + impulse * body_b.inv_mass;

                body_a.orientation = apply_angular_correction(
                    body_a.orientation, iixn_a * (-delta_lambda));
                body_b.orientation = apply_angular_correction(
                    body_b.orientation, iixn_b * delta_lambda);
            }
        };

        /**
         * @brief The boundary distance projection: `XpbdDistanceProjectionT` fixed to `Scalar`.
         */
        using XpbdDistanceProjection = XpbdDistanceProjectionT<Scalar>;

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
