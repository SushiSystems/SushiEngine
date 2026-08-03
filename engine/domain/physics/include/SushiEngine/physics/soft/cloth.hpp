/**************************************************************************/
/* cloth.hpp                                                              */
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
 * @file cloth.hpp
 * @brief SushiLoop M5: cloth as a grid of `XPBDDistanceConstraint`s (no new solver).
 *
 * A cloth (or a rope, degenerately at one row) is not a new physics primitive —
 * it is a mass-spring topology expressed entirely in terms of the constraint type
 * `xpbd_demo.cpp`'s hanging chain already uses, per `docs/slop/SUSHILOOP.md`.
 * `build_cloth_grid` registers one `RigidBody` per grid point (zero inverse
 * inertia, anchors implicitly at each body's own origin, so no angular coupling —
 * the same linear-only degeneration `XPBDDistanceConstraint` already supports) and
 * wires structural constraints (horizontal/vertical neighbours) plus shear
 * constraints (diagonal neighbours, which resist the grid collapsing into a
 * parallelogram under shear) into the caller's `PhysicsWorld<XPBDDistanceConstraint>`.
 * The whole grid's first row is pinned (`inv_mass == 0`) so it hangs, mirroring the
 * existing hanging-chain demos.
 *
 * Volumetric (tetrahedral) soft bodies are explicitly out of scope for M5 — cloth
 * is a 2D constraint grid, not a general deformable-solid solver, and adding one
 * is a distinct milestone, not a trivial extension of this file.
 */

#include <cmath>
#include <cstddef>
#include <vector>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/scene/physics_world.hpp>
#include <SushiEngine/physics/core/handle.hpp>
#include <SushiEngine/physics/core/rigid_body.hpp>
#include <SushiEngine/physics/constraints/xpbd_constraint.hpp>
#include <SushiEngine/physics/solver/solver_interface.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief The body ids of a cloth's grid points, addressable by (row, column).
         *
         * Row 0 is the pinned edge; `bodies` is row-major (`row * cols + col`), which
         * is also the order `build_cloth_grid` registered them in with `add_body`.
         */
        struct ClothGrid
        {
            std::size_t rows = 0;
            std::size_t cols = 0;
            std::vector<BodyId> bodies;

            /**
             * @brief The body id at grid position (row, column).
             * @param row Row index, `< rows`.
             * @param col Column index, `< cols`.
             * @return The body id registered at that grid point.
             */
            BodyId at(std::size_t row, std::size_t col) const noexcept
            {
                return bodies[row * cols + col];
            }
        };

        /**
         * @brief Builds a pinned-top cloth grid of distance constraints into @p world.
         *
         * Registers `rows * cols` bodies (not yet finalized — call `world.finalize()`
         * afterward, same as any other `PhysicsWorld` usage) laid out in the XZ plane,
         * `spacing` apart, with every body in row 0 pinned (`inv_mass == 0`) so the
         * grid hangs from that edge under gravity. Adds a structural constraint to
         * each right and each below neighbour, and a shear constraint to each
         * diagonal neighbour pair, all at `rest_length` equal to the two points'
         * initial separation and the given @p compliance.
         *
         * @tparam Constraint The world's constraint type; its `Real` sets the grid's precision.
         * @param world      The physics world to register bodies and constraints into;
         *                   must not have been `finalize()`d yet.
         * @param rows       Number of grid rows (>= 1); row 0 is pinned.
         * @param cols       Number of grid columns (>= 1).
         * @param spacing    Distance between adjacent grid points, in world units (> 0).
         * @param origin     World-space position of grid point (0, 0).
         * @param compliance XPBD compliance applied to every constraint in the grid;
         *                   `0` is fully rigid, matching the hanging-chain demos.
         * @return The grid's body ids, addressable by (row, column).
         */
        template <typename Constraint>
        ClothGrid build_cloth_grid(PhysicsWorld<Constraint>& world,
                                   std::size_t rows, std::size_t cols,
                                   typename Constraint::Real spacing,
                                   Vector3T<typename Constraint::Real> origin,
                                   typename Constraint::Real compliance = 0)
        {
            using Real = typename Constraint::Real;

            ClothGrid grid;
            grid.rows = rows;
            grid.cols = cols;
            grid.bodies.reserve(rows * cols);

            for (std::size_t row = 0; row < rows; ++row)
                for (std::size_t col = 0; col < cols; ++col)
                {
                    RigidBodyT<Real> body;
                    body.position =
                        origin + Vector3T<Real>{Real(col) * spacing, Real(0), Real(row) * spacing};
                    body.inv_mass = (row == 0) ? Real(0) : Real(1);
                    body.inv_inertia = Vector3T<Real>{0, 0, 0};
                    grid.bodies.push_back(world.add_body(body));
                }

            const auto link = [&](BodyId a, BodyId b, Real rest_length)
            {
                world.add_constraint(Constraint{
                    a, b, Vector3T<Real>{0, 0, 0}, Vector3T<Real>{0, 0, 0}, rest_length, compliance});
            };

            const Real diagonal = spacing * Real(std::sqrt(2.0));
            for (std::size_t row = 0; row < rows; ++row)
                for (std::size_t col = 0; col < cols; ++col)
                {
                    if (col + 1 < cols)
                        link(grid.at(row, col), grid.at(row, col + 1), spacing);
                    if (row + 1 < rows)
                        link(grid.at(row, col), grid.at(row + 1, col), spacing);
                    if (row + 1 < rows && col + 1 < cols)
                    {
                        link(grid.at(row, col), grid.at(row + 1, col + 1), diagonal);
                        link(grid.at(row, col + 1), grid.at(row + 1, col), diagonal);
                    }
                }

            return grid;
        }

        /**
         * @brief The same grid, addressed by solver handles rather than world ids.
         *
         * `PhysicsWorld` numbers its bodies; `IConstraintSolver` hands out
         * generational handles, because it admits and retires bodies mid-simulation
         * and an index alone cannot say whether it still names the body it named
         * last tick (§6.4). So the two cannot share a return type, and pretending
         * they could by casting one to the other is exactly the class of bug the
         * generation counter exists to catch.
         */
        struct ClothGridHandles
        {
            std::size_t rows = 0;
            std::size_t cols = 0;
            std::vector<BodyHandle> bodies;

            /** @brief The body handle at grid position (row, column). */
            BodyHandle at(std::size_t row, std::size_t col) const noexcept
            {
                return bodies[row * cols + col];
            }
        };

        /**
         * @brief Builds a pinned-top cloth grid into a constraint solver.
         *
         * The same topology as the `PhysicsWorld` overload above and deliberately
         * the same arithmetic — structural links to the right and below, shear links
         * across both diagonals, row 0 pinned — expressed against the solver seam so
         * a cloth is bodies and constraints in the *one* solver that also holds the
         * rigid bodies (§0.1). That is what makes cloth-to-rigid contact an ordinary
         * contact rather than a coupling between two worlds driven in lockstep.
         *
         * @tparam T The solver's scalar element type.
         * @param solver     The solver to register bodies and constraints into.
         * @param rows       Number of grid rows (>= 1); row 0 is pinned.
         * @param cols       Number of grid columns (>= 1).
         * @param spacing    Distance between adjacent grid points, in world units (> 0).
         * @param origin     World-space position of grid point (0, 0).
         * @param compliance XPBD compliance applied to every constraint in the grid.
         * @return The grid's body handles, addressable by (row, column).
         */
        template <typename T>
        ClothGridHandles build_cloth_grid(IConstraintSolver<T>& solver, std::size_t rows,
                                          std::size_t cols, T spacing, Vector3T<T> origin,
                                          T compliance = 0)
        {
            ClothGridHandles grid;
            grid.rows = rows;
            grid.cols = cols;
            grid.bodies.reserve(rows * cols);

            for (std::size_t row = 0; row < rows; ++row)
                for (std::size_t col = 0; col < cols; ++col)
                {
                    RigidBodyT<T> body;
                    body.position =
                        origin + Vector3T<T>{T(col) * spacing, T(0), T(row) * spacing};
                    body.previous_position = body.position;
                    body.inv_mass = (row == 0) ? T(0) : T(1);
                    body.inv_inertia = Vector3T<T>{0, 0, 0};
                    // A pinned row is immovable, not asleep: it must keep conducting
                    // constraint corrections to the row below it.
                    grid.bodies.push_back(solver.add_body(body));
                }

            const auto link = [&](BodyHandle a, BodyHandle b, T rest_length)
            {
                XPBDDistanceConstraintT<T> constraint;
                constraint.a = std::uint32_t(solver.body_slot(a));
                constraint.b = std::uint32_t(solver.body_slot(b));
                constraint.rest_length = rest_length;
                constraint.compliance = compliance;
                solver.add_constraint(constraint);
            };

            const T diagonal = spacing * T(std::sqrt(2.0));
            for (std::size_t row = 0; row < rows; ++row)
                for (std::size_t col = 0; col < cols; ++col)
                {
                    if (col + 1 < cols)
                        link(grid.at(row, col), grid.at(row, col + 1), spacing);
                    if (row + 1 < rows)
                        link(grid.at(row, col), grid.at(row + 1, col), spacing);
                    if (row + 1 < rows && col + 1 < cols)
                    {
                        link(grid.at(row, col), grid.at(row + 1, col + 1), diagonal);
                        link(grid.at(row, col + 1), grid.at(row + 1, col), diagonal);
                    }
                }

            return grid;
        }
    } // namespace Physics
} // namespace SushiEngine
