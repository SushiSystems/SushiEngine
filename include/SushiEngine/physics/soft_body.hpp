/**************************************************************************/
/* soft_body.hpp                                                         */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/* Licensed under the Apache License, Version 2.0 (the "License");        */
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
 * @file soft_body.hpp
 * @brief A volumetric soft body as a 3D lattice of XPBD distance constraints.
 *
 * The 3D generalization of `physics/cloth.hpp`: where cloth is a 2D grid of particles
 * held by distance constraints, a soft body is a 3D grid (`nx * ny * nz` particles)
 * held by the same constraint type — structural constraints along each axis and shear
 * constraints across each face diagonal, which together resist stretch and shear so
 * the block keeps its shape while still deforming under load. Like cloth it introduces
 * no new solver or constraint: it wires a topology into the caller's
 * `PhysicsWorld<XpbdDistanceConstraint>` (of whichever precision), so the existing
 * graph-coloured `XpbdSolver` runs it unchanged. Tetrahedral volume constraints are a
 * later refinement; a structural+shear lattice is the mass-spring soft body many
 * engines ship and is enough to make a deformable block that settles and recovers.
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
         * @brief The particle ids of a soft body's 3D lattice, addressable by (x, y, z).
         *
         * `bodies` is laid out `((z * ny) + y) * nx + x`, the order
         * `build_soft_body_lattice` registered them in.
         */
        struct SoftBodyLattice
        {
            std::size_t nx = 0;
            std::size_t ny = 0;
            std::size_t nz = 0;
            std::vector<BodyId> bodies;

            /**
             * @brief The particle id at lattice cell (x, y, z).
             * @param x Column index, `< nx`.
             * @param y Row index, `< ny`.
             * @param z Depth index, `< nz`.
             * @return The body id at that cell.
             */
            BodyId at(std::size_t x, std::size_t y, std::size_t z) const noexcept
            {
                return bodies[(z * ny + y) * nx + x];
            }
        };

        /**
         * @brief Builds a solid box soft body of distance constraints into @p world.
         *
         * Registers `nx * ny * nz` particles on a regular grid `spacing` apart with its
         * corner at @p origin, then links every axis-aligned neighbour with a structural
         * constraint and every face-diagonal neighbour with a shear constraint, all at
         * the given @p compliance. Optionally pins the bottom layer (`y == 0`) so the
         * body hangs; otherwise every particle is free and the body falls and deforms as
         * a whole. Call `world.finalize()` afterward, as with any `PhysicsWorld` usage.
         *
         * @tparam Constraint The world's constraint type; its `Real` sets the precision.
         * @param world      The physics world to register into; not yet finalized.
         * @param nx         Particles along x (>= 1).
         * @param ny         Particles along y (>= 1).
         * @param nz         Particles along z (>= 1).
         * @param spacing    Distance between adjacent particles (> 0).
         * @param origin     World position of particle (0, 0, 0).
         * @param compliance XPBD compliance of every constraint; 0 is fully rigid, larger
         *                   values make a softer, more deformable body.
         * @param pin_bottom Whether the `y == 0` layer is pinned (`inv_mass == 0`).
         * @return The lattice's particle ids, addressable by (x, y, z).
         */
        template <typename Constraint>
        SoftBodyLattice build_soft_body_lattice(PhysicsWorld<Constraint>& world, std::size_t nx,
                                                std::size_t ny, std::size_t nz,
                                                typename Constraint::Real spacing,
                                                Vector3T<typename Constraint::Real> origin,
                                                typename Constraint::Real compliance = 0,
                                                bool pin_bottom = false)
        {
            using Real = typename Constraint::Real;

            SoftBodyLattice lattice;
            lattice.nx = nx;
            lattice.ny = ny;
            lattice.nz = nz;
            lattice.bodies.reserve(nx * ny * nz);

            for (std::size_t z = 0; z < nz; ++z)
                for (std::size_t y = 0; y < ny; ++y)
                    for (std::size_t x = 0; x < nx; ++x)
                    {
                        RigidBodyT<Real> body;
                        body.position = origin + Vector3T<Real>{Real(x) * spacing,
                                                                Real(y) * spacing,
                                                                Real(z) * spacing};
                        body.inv_mass = (pin_bottom && y == 0) ? Real(0) : Real(1);
                        body.inv_inertia = Vector3T<Real>{0, 0, 0};
                        lattice.bodies.push_back(world.add_body(body));
                    }

            const auto link = [&](BodyId a, BodyId b, Real rest_length)
            {
                world.add_constraint(Constraint{a, b, Vector3T<Real>{0, 0, 0},
                                                Vector3T<Real>{0, 0, 0}, rest_length, compliance});
            };

            const Real diagonal = spacing * Real(std::sqrt(2.0));
            for (std::size_t z = 0; z < nz; ++z)
                for (std::size_t y = 0; y < ny; ++y)
                    for (std::size_t x = 0; x < nx; ++x)
                    {
                        // Structural constraints along each axis.
                        if (x + 1 < nx)
                            link(lattice.at(x, y, z), lattice.at(x + 1, y, z), spacing);
                        if (y + 1 < ny)
                            link(lattice.at(x, y, z), lattice.at(x, y + 1, z), spacing);
                        if (z + 1 < nz)
                            link(lattice.at(x, y, z), lattice.at(x, y, z + 1), spacing);

                        // Shear constraints across the three face diagonals of each cell.
                        if (x + 1 < nx && y + 1 < ny)
                        {
                            link(lattice.at(x, y, z), lattice.at(x + 1, y + 1, z), diagonal);
                            link(lattice.at(x + 1, y, z), lattice.at(x, y + 1, z), diagonal);
                        }
                        if (x + 1 < nx && z + 1 < nz)
                        {
                            link(lattice.at(x, y, z), lattice.at(x + 1, y, z + 1), diagonal);
                            link(lattice.at(x + 1, y, z), lattice.at(x, y, z + 1), diagonal);
                        }
                        if (y + 1 < ny && z + 1 < nz)
                        {
                            link(lattice.at(x, y, z), lattice.at(x, y + 1, z + 1), diagonal);
                            link(lattice.at(x, y + 1, z), lattice.at(x, y, z + 1), diagonal);
                        }
                    }

            return lattice;
        }
    } // namespace Physics
} // namespace SushiEngine
