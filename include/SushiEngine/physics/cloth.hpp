/**************************************************************************/
/* cloth.hpp                                                             */
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
 * @brief SushiLoop M5: cloth as a grid of `XpbdDistanceConstraint`s (no new solver).
 *
 * A cloth (or a rope, degenerately at one row) is not a new physics primitive —
 * it is a mass-spring topology expressed entirely in terms of the constraint type
 * `xpbd_demo.cpp`'s hanging chain already uses, per `docs/slop/SUSHILOOP.md`.
 * `build_cloth_grid` registers one `RigidBody` per grid point (zero inverse
 * inertia, anchors implicitly at each body's own origin, so no angular coupling —
 * the same linear-only degeneration `XpbdDistanceConstraint` already supports) and
 * wires structural constraints (horizontal/vertical neighbours) plus shear
 * constraints (diagonal neighbours, which resist the grid collapsing into a
 * parallelogram under shear) into the caller's `PhysicsWorld<XpbdDistanceConstraint>`.
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
#include <SushiEngine/physics/physics_world.hpp>
#include <SushiEngine/physics/rigid_body.hpp>
#include <SushiEngine/physics/xpbd_constraint.hpp>

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
    } // namespace Physics
} // namespace SushiEngine
