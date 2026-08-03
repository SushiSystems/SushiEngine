/**************************************************************************/
/* tetrahedral_lattice.hpp                                                */
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

// A regular tetrahedral lattice, and a `FiniteElementModel` built from one.
//
// Several tests need a soft body whose behaviour has a closed-form answer —
// the cantilever against Euler-Bernoulli, a cube resting on a surface, an
// embedding that has to round-trip — and none of them want the cooker in the
// way. The cooker's job is turning arbitrary dirty meshes into a lattice and it
// is exercised on its own terms in `test_soft_body_cooker.cpp`; what these
// tests need is a *known* lattice, so it is built here, once, rather than
// copied into each of them.

#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <utility>
#include <vector>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/soft/finite_element_model.hpp>

namespace SushiEngine
{
    namespace Harness
    {
        /** @brief A box of cells, each split into six tetrahedra. */
        struct TetrahedralLattice
        {
            std::vector<Vector3> vertices;
            /** @brief Four particle indices per element. */
            std::vector<std::uint32_t> tetrahedra;
            std::size_t length_cells = 0; /**< Cells along X. */
            std::size_t width_cells = 0;  /**< Cells along Y. */
            std::size_t height_cells = 0; /**< Cells along Z. */
        };

        /**
         * @brief The grid vertex at (x, y, z), in the lattice's own numbering.
         *
         * @param x            Index along X, `0..length_cells`.
         * @param y            Index along Y, `0..width_cells`.
         * @param z            Index along Z, `0..height_cells`.
         * @param width_cells  The lattice's Y cell count.
         * @param height_cells The lattice's Z cell count.
         * @return The vertex index into @ref TetrahedralLattice::vertices.
         */
        inline std::uint32_t lattice_vertex_index(std::size_t x, std::size_t y, std::size_t z,
                                                  std::size_t width_cells,
                                                  std::size_t height_cells) noexcept
        {
            return std::uint32_t((x * (width_cells + 1) + y) * (height_cells + 1) + z);
        }

        /**
         * @brief Six tetrahedra for one cell, each corrected to a positive rest volume.
         *
         * The "orbit around the main diagonal" Kuhn decomposition — the same one
         * `physics/cooking/tetrahedral_mesh.hpp` names — with each candidate's two
         * trailing corners swapped when this particular corner numbering happens to
         * wind the element inside out, since a negative rest volume is a collapsed
         * element to every formula downstream.
         *
         * @param vertices The lattice's vertices.
         * @param corner   The cell's eight corners, in the order
         *                 (000, 100, 010, 110, 001, 101, 011, 111).
         * @param out      Receives twenty-four indices, four per element.
         */
        inline void add_cell_tetrahedra(const std::vector<Vector3>& vertices,
                                        const std::uint32_t corner[8],
                                        std::vector<std::uint32_t>& out)
        {
            const std::uint32_t c000 = corner[0];
            const std::uint32_t c100 = corner[1];
            const std::uint32_t c010 = corner[2];
            const std::uint32_t c110 = corner[3];
            const std::uint32_t c001 = corner[4];
            const std::uint32_t c101 = corner[5];
            const std::uint32_t c011 = corner[6];
            const std::uint32_t c111 = corner[7];

            const std::uint32_t candidates[6][4] = {
                {c000, c100, c110, c111}, {c000, c110, c010, c111}, {c000, c010, c011, c111},
                {c000, c011, c001, c111}, {c000, c001, c101, c111}, {c000, c101, c100, c111}};

            for (const auto& candidate : candidates)
            {
                std::uint32_t indices[4] = {candidate[0], candidate[1], candidate[2], candidate[3]};
                const Vector3& a = vertices[indices[0]];
                const Vector3 edge1 = vertices[indices[1]] - a;
                const Vector3 edge2 = vertices[indices[2]] - a;
                const Vector3 edge3 = vertices[indices[3]] - a;
                if (dot(edge1, cross(edge2, edge3)) < Scalar(0))
                    std::swap(indices[2], indices[3]);
                for (int i = 0; i < 4; ++i)
                    out.push_back(indices[i]);
            }
        }

        /**
         * @brief Builds a box lattice of `length x width x height` cells.
         *
         * @param length_cells Cells along X (>= 1).
         * @param width_cells  Cells along Y (>= 1).
         * @param height_cells Cells along Z (>= 1).
         * @param cell_size    Edge length of one cell, in metres.
         * @return The lattice, with its origin corner at the world origin.
         */
        inline TetrahedralLattice build_tetrahedral_lattice(std::size_t length_cells,
                                                            std::size_t width_cells,
                                                            std::size_t height_cells,
                                                            Scalar cell_size)
        {
            TetrahedralLattice lattice;
            lattice.length_cells = length_cells;
            lattice.width_cells = width_cells;
            lattice.height_cells = height_cells;

            lattice.vertices.resize((length_cells + 1) * (width_cells + 1) * (height_cells + 1));
            for (std::size_t x = 0; x <= length_cells; ++x)
                for (std::size_t y = 0; y <= width_cells; ++y)
                    for (std::size_t z = 0; z <= height_cells; ++z)
                        lattice.vertices[lattice_vertex_index(x, y, z, width_cells, height_cells)] =
                            Vector3{Scalar(x) * cell_size, Scalar(y) * cell_size,
                                    Scalar(z) * cell_size};

            for (std::size_t x = 0; x < length_cells; ++x)
                for (std::size_t y = 0; y < width_cells; ++y)
                    for (std::size_t z = 0; z < height_cells; ++z)
                    {
                        const std::uint32_t corner[8] = {
                            lattice_vertex_index(x, y, z, width_cells, height_cells),
                            lattice_vertex_index(x + 1, y, z, width_cells, height_cells),
                            lattice_vertex_index(x, y + 1, z, width_cells, height_cells),
                            lattice_vertex_index(x + 1, y + 1, z, width_cells, height_cells),
                            lattice_vertex_index(x, y, z + 1, width_cells, height_cells),
                            lattice_vertex_index(x + 1, y, z + 1, width_cells, height_cells),
                            lattice_vertex_index(x, y + 1, z + 1, width_cells, height_cells),
                            lattice_vertex_index(x + 1, y + 1, z + 1, width_cells, height_cells)};
                        add_cell_tetrahedra(lattice.vertices, corner, lattice.tetrahedra);
                    }
            return lattice;
        }

        /**
         * @brief `Dm^-1`'s three columns and the rest volume of one tetrahedron.
         *
         * The closed-form inverse of the matrix whose *rows* are the three edges —
         * whose rows are then `Dm^-1`'s columns — assembled exactly the way
         * `physics/cooking/tetrahedral_mesh.cpp` assembles it, so a lattice built
         * here carries the same rest state the real cooker would produce for it.
         *
         * @param a,b,c,d          The element's four rest positions.
         * @param column0/1/2      Receive `Dm^-1`'s three columns.
         * @param volume           Receives the rest volume.
         * @return False for a degenerate or inverted element, leaving the outputs untouched.
         */
        inline bool lattice_rest_inverse(const Vector3& a, const Vector3& b, const Vector3& c,
                                         const Vector3& d, Vector3& column0, Vector3& column1,
                                         Vector3& column2, Scalar& volume)
        {
            const Vector3 e1 = b - a;
            const Vector3 e2 = c - a;
            const Vector3 e3 = d - a;
            const Scalar signed_volume = dot(e1, cross(e2, e3)) / Scalar(6);
            if (!(signed_volume > Scalar(0)))
                return false;

            const Scalar determinant = e1.x * (e2.y * e3.z - e2.z * e3.y) -
                                       e1.y * (e2.x * e3.z - e2.z * e3.x) +
                                       e1.z * (e2.x * e3.y - e2.y * e3.x);
            if (determinant == Scalar(0))
                return false;
            const Scalar inverse = Scalar(1) / determinant;
            column0 = Vector3{(e2.y * e3.z - e2.z * e3.y) * inverse,
                              (e1.z * e3.y - e1.y * e3.z) * inverse,
                              (e1.y * e2.z - e1.z * e2.y) * inverse};
            column1 = Vector3{(e2.z * e3.x - e2.x * e3.z) * inverse,
                              (e1.x * e3.z - e1.z * e3.x) * inverse,
                              (e1.z * e2.x - e1.x * e2.z) * inverse};
            column2 = Vector3{(e2.x * e3.y - e2.y * e3.x) * inverse,
                              (e1.y * e3.x - e1.x * e3.y) * inverse,
                              (e1.x * e2.y - e1.y * e2.x) * inverse};
            volume = signed_volume;
            return true;
        }

        /**
         * @brief The boundary triangles of a tetrahedral mesh, wound outward.
         *
         * A face shared by two elements is interior; a face named exactly once is
         * on the boundary. Each surviving face is then wound so its normal points
         * away from the element's fourth vertex, which is what "outward" means for
         * a face whose element is the only solid behind it — checked per face
         * rather than assumed from a corner ordering, so the result does not depend
         * on how the elements happened to be enumerated.
         *
         * @param vertices   The mesh's vertices.
         * @param tetrahedra Four indices per element.
         * @return Three indices per boundary triangle, in element order.
         */
        inline std::vector<std::uint32_t> build_surface_triangles(
            const std::vector<Vector3>& vertices, const std::vector<std::uint32_t>& tetrahedra)
        {
            struct Face
            {
                std::uint32_t index[3];
                std::uint32_t opposite;
            };

            const std::size_t element_count = tetrahedra.size() / 4;
            std::vector<Face> faces;
            faces.reserve(element_count * 4);
            std::map<std::array<std::uint32_t, 3>, int> occurrences;

            for (std::size_t t = 0; t < element_count; ++t)
            {
                const std::uint32_t* corner = tetrahedra.data() + t * 4;
                const int face_corner[4][3] = {{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}};
                const int opposite_corner[4] = {3, 2, 1, 0};
                for (int f = 0; f < 4; ++f)
                {
                    Face face;
                    for (int i = 0; i < 3; ++i)
                        face.index[i] = corner[face_corner[f][i]];
                    face.opposite = corner[opposite_corner[f]];
                    faces.push_back(face);

                    std::array<std::uint32_t, 3> key = {face.index[0], face.index[1],
                                                        face.index[2]};
                    std::sort(key.begin(), key.end());
                    ++occurrences[key];
                }
            }

            std::vector<std::uint32_t> surface;
            for (const Face& face : faces)
            {
                std::array<std::uint32_t, 3> key = {face.index[0], face.index[1], face.index[2]};
                std::sort(key.begin(), key.end());
                if (occurrences[key] != 1)
                    continue;

                std::uint32_t wound[3] = {face.index[0], face.index[1], face.index[2]};
                const Vector3& a = vertices[wound[0]];
                const Vector3 normal = cross(vertices[wound[1]] - a, vertices[wound[2]] - a);
                if (dot(normal, vertices[face.opposite] - a) > Scalar(0))
                    std::swap(wound[1], wound[2]);
                surface.push_back(wound[0]);
                surface.push_back(wound[1]);
                surface.push_back(wound[2]);
            }
            return surface;
        }

        /**
         * @brief A complete `FiniteElementModel` from a lattice: elements, masses, surface.
         *
         * Masses are lumped a quarter of each element's to each of its four
         * vertices — the scheme `tetrahedral_mesh.cpp` uses — accumulated as mass
         * and inverted once per vertex afterward, since a vertex shared by several
         * elements must sum their contributions before the reciprocal is taken
         * rather than average the reciprocals.
         *
         * A degenerate element is dropped rather than carried, so a caller that
         * cares can compare `elements.size()` against the lattice's element count.
         *
         * @param lattice  The lattice to realize.
         * @param material The constitutive parameters; its density sets the masses.
         * @return The model, at rest, with nothing pinned.
         */
        inline Physics::FiniteElementModel<Scalar> build_lattice_model(
            const TetrahedralLattice& lattice, const Physics::SoftBodyMaterial& material)
        {
            Physics::FiniteElementModel<Scalar> model;
            model.material = material;

            model.particles.resize(lattice.vertices.size());
            for (std::size_t i = 0; i < lattice.vertices.size(); ++i)
            {
                Physics::RigidBodyT<Scalar>& particle = model.particles[i];
                particle.position = lattice.vertices[i];
                particle.prev_position = particle.position;
                particle.orientation = Quaternion{0, 0, 0, 1};
                particle.prev_orientation = particle.orientation;
                particle.inv_inertia = Vector3{0, 0, 0};
                particle.inv_mass = Scalar(0);
            }

            const std::size_t element_count = lattice.tetrahedra.size() / 4;
            model.elements.reserve(element_count);
            std::vector<Scalar> accumulated_mass(model.particles.size(), Scalar(0));
            for (std::size_t t = 0; t < element_count; ++t)
            {
                const std::uint32_t* corner = lattice.tetrahedra.data() + t * 4;
                Vector3 column0, column1, column2;
                Scalar volume = 0;
                if (!lattice_rest_inverse(lattice.vertices[corner[0]], lattice.vertices[corner[1]],
                                          lattice.vertices[corner[2]], lattice.vertices[corner[3]],
                                          column0, column1, column2, volume))
                    continue;

                Physics::FEMTetrahedron element;
                for (int i = 0; i < 4; ++i)
                    element.vertex[i] = corner[i];
                element.rest_inverse_column_0 = column0;
                element.rest_inverse_column_1 = column1;
                element.rest_inverse_column_2 = column2;
                element.plastic_inverse_column_0 = column0;
                element.plastic_inverse_column_1 = column1;
                element.plastic_inverse_column_2 = column2;
                element.rest_volume = volume;
                model.elements.push_back(element);

                const Scalar vertex_mass = volume * material.density / Scalar(4);
                for (int i = 0; i < 4; ++i)
                    accumulated_mass[corner[i]] += vertex_mass;
            }

            for (std::size_t i = 0; i < model.particles.size(); ++i)
                model.particles[i].inv_mass = accumulated_mass[i] > Scalar(0)
                                                  ? Scalar(1) / accumulated_mass[i]
                                                  : Scalar(0);

            model.surface_indices = build_surface_triangles(lattice.vertices, lattice.tetrahedra);
            std::vector<bool> on_surface(model.particles.size(), false);
            for (const std::uint32_t index : model.surface_indices)
                on_surface[index] = true;
            for (std::uint32_t i = 0; i < model.particles.size(); ++i)
                if (on_surface[i])
                    model.surface_vertices.push_back(i);

            return model;
        }
    } // namespace Harness
} // namespace SushiEngine
