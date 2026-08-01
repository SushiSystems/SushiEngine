/**************************************************************************/
/* test_finite_element_model.cpp                                         */
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
/* permissions and limitations under the License.                        */
/**************************************************************************/

// Integration_FiniteElementModel: P6's own acceptance criterion, the
// *mukavemet* test — "the cantilever-deflection test matches theory" — run
// against `FiniteElementModel` end to end rather than asserted in prose.
//
// A cantilever beam, pinned at one end, sagging under its own weight, has a
// closed-form tip deflection (Euler-Bernoulli, distributed load):
//
//   delta = (mass * g * L^3) / (8 * E * I)
//
// where `I = width * height^3 / 12` for a rectangular cross-section. This is
// the strongest oracle available for a brand-new constitutive model — not
// "does it look bent," but "is the stiffness the material parameters state
// the stiffness the beam actually exhibits."
//
// The beam mesh is built by hand here (a Kuhn six-tetrahedra-per-cube
// lattice, the same decomposition `tetrahedral_mesh.hpp`'s own file comment
// names, sign-corrected per element) rather than through the cooker: the
// cooker's job is turning arbitrary dirty meshes into this kind of lattice,
// and is exercised on its own terms in `test_soft_body_cooker.cpp`. This test
// needs a *known*, regular lattice to have a closed-form answer to check
// against, which a voxelized-and-conformed mesh would not give as cleanly.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/physics/soft/finite_element_model.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    /** @brief A rectangular beam of tetrahedra, `length_cells` long along X. */
    struct BeamMesh
    {
        std::vector<Vector3> vertices;
        std::vector<std::uint32_t> tetrahedra; // 4 indices per element
        std::size_t vertices_per_x = 0;        // (width_cells+1) * (height_cells+1)
    };

    std::uint32_t vertex_index(std::size_t x, std::size_t y, std::size_t z, std::size_t ny,
                               std::size_t nz)
    {
        return std::uint32_t((x * (ny + 1) + y) * (nz + 1) + z);
    }

    /**
     * @brief Six tetrahedra per unit cube (the "orbit around the main diagonal"
     *        Kuhn decomposition), sign-corrected so every element has a
     *        positive rest volume regardless of which way this particular
     *        corner numbering happens to wind.
     */
    void add_cube_tetrahedra(const std::vector<Vector3>& vertices, std::uint32_t c000,
                             std::uint32_t c100, std::uint32_t c010, std::uint32_t c110,
                             std::uint32_t c001, std::uint32_t c101, std::uint32_t c011,
                             std::uint32_t c111, std::vector<std::uint32_t>& out)
    {
        const std::uint32_t candidates[6][4] = {
            {c000, c100, c110, c111}, {c000, c110, c010, c111}, {c000, c010, c011, c111},
            {c000, c011, c001, c111}, {c000, c001, c101, c111}, {c000, c101, c100, c111}};

        for (const auto& tet : candidates)
        {
            std::uint32_t indices[4] = {tet[0], tet[1], tet[2], tet[3]};
            const Vector3& a = vertices[indices[0]];
            const Vector3 edge1 = vertices[indices[1]] - a;
            const Vector3 edge2 = vertices[indices[2]] - a;
            const Vector3 edge3 = vertices[indices[3]] - a;
            const Scalar signed_volume = dot(edge1, cross(edge2, edge3));
            if (signed_volume < Scalar(0))
                std::swap(indices[2], indices[3]);
            for (int i = 0; i < 4; ++i)
                out.push_back(indices[i]);
        }
    }

    /**
     * @brief Builds a beam `length_cells x width_cells x height_cells` cubes,
     *        each cube split into six tetrahedra.
     */
    BeamMesh build_beam_mesh(std::size_t length_cells, std::size_t width_cells,
                             std::size_t height_cells, Scalar cell_size)
    {
        BeamMesh mesh;
        const std::size_t nx = length_cells;
        const std::size_t ny = width_cells;
        const std::size_t nz = height_cells;
        mesh.vertices_per_x = (ny + 1) * (nz + 1);

        mesh.vertices.resize((nx + 1) * (ny + 1) * (nz + 1));
        for (std::size_t x = 0; x <= nx; ++x)
            for (std::size_t y = 0; y <= ny; ++y)
                for (std::size_t z = 0; z <= nz; ++z)
                    mesh.vertices[vertex_index(x, y, z, ny, nz)] =
                        Vector3{Scalar(x) * cell_size, Scalar(y) * cell_size,
                               Scalar(z) * cell_size};

        for (std::size_t x = 0; x < nx; ++x)
            for (std::size_t y = 0; y < ny; ++y)
                for (std::size_t z = 0; z < nz; ++z)
                {
                    const std::uint32_t c000 = vertex_index(x, y, z, ny, nz);
                    const std::uint32_t c100 = vertex_index(x + 1, y, z, ny, nz);
                    const std::uint32_t c010 = vertex_index(x, y + 1, z, ny, nz);
                    const std::uint32_t c110 = vertex_index(x + 1, y + 1, z, ny, nz);
                    const std::uint32_t c001 = vertex_index(x, y, z + 1, ny, nz);
                    const std::uint32_t c101 = vertex_index(x + 1, y, z + 1, ny, nz);
                    const std::uint32_t c011 = vertex_index(x, y + 1, z + 1, ny, nz);
                    const std::uint32_t c111 = vertex_index(x + 1, y + 1, z + 1, ny, nz);
                    add_cube_tetrahedra(mesh.vertices, c000, c100, c010, c110, c001, c101, c011,
                                       c111, mesh.tetrahedra);
                }
        return mesh;
    }

    /** @brief `Dm^-1`'s three columns for one tetrahedron, or false if degenerate. */
    bool rest_inverse_of(const Vector3& a, const Vector3& b, const Vector3& c, const Vector3& d,
                         Vector3& column0, Vector3& column1, Vector3& column2, Scalar& volume)
    {
        const Vector3 e1 = b - a;
        const Vector3 e2 = c - a;
        const Vector3 e3 = d - a;
        volume = dot(e1, cross(e2, e3)) / Scalar(6);
        if (!(volume > Scalar(0)))
            return false;

        // Standard closed-form inverse of the matrix whose ROWS are e1, e2, e3
        // (Dm^T, since Dm's COLUMNS are e1/e2/e3); its rows are then Dm^-1's
        // columns, matching physics/cooking/tetrahedral_mesh.cpp's own
        // invert_3x3 exactly, so this test builds rest state the identical way
        // the real cooker does.
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
        return true;
    }
} // namespace

TEST(Integration_FiniteElementModel, CantileverTipDeflectionMatchesEulerBernoulli)
{
    constexpr std::size_t LENGTH_CELLS = 10;
    constexpr std::size_t WIDTH_CELLS = 2;
    constexpr std::size_t HEIGHT_CELLS = 2;
    const Scalar cell_size = Scalar(0.1); // a 1.0 x 0.2 x 0.2 m beam

    const BeamMesh mesh = build_beam_mesh(LENGTH_CELLS, WIDTH_CELLS, HEIGHT_CELLS, cell_size);

    FiniteElementModel<Scalar> model;
    // A soft, rubber-scale material chosen so the beam sags a clearly
    // measurable amount while staying inside the small-deflection regime
    // Euler-Bernoulli theory assumes (a handful of percent of the beam's own
    // length, not a fraction of it folding over).
    model.material.young_modulus = Scalar(1.84e7);
    model.material.poisson_ratio = Scalar(0.3);
    model.material.density = Scalar(1000.0);
    model.material.damping = Scalar(3.0); // heavily damped: this test wants the static answer
    model.external_acceleration = Vector3{0, 0, Scalar(-9.81)};

    model.particles.resize(mesh.vertices.size());
    for (std::size_t i = 0; i < mesh.vertices.size(); ++i)
    {
        RigidBodyT<Scalar>& particle = model.particles[i];
        particle.position = mesh.vertices[i];
        particle.prev_position = particle.position;
        particle.orientation = Quaternion{0, 0, 0, 1};
        particle.prev_orientation = particle.orientation;
        particle.inv_inertia = Vector3{0, 0, 0};
        particle.inv_mass = Scalar(0); // filled in below, once every element's mass is known
    }

    const std::size_t element_count = mesh.tetrahedra.size() / 4;
    model.elements.reserve(element_count);
    Scalar total_volume = 0;
    for (std::size_t t = 0; t < element_count; ++t)
    {
        const std::uint32_t* corner = mesh.tetrahedra.data() + t * 4;
        Vector3 column0, column1, column2;
        Scalar volume = 0;
        ASSERT_TRUE(rest_inverse_of(mesh.vertices[corner[0]], mesh.vertices[corner[1]],
                                    mesh.vertices[corner[2]], mesh.vertices[corner[3]], column0,
                                    column1, column2, volume));
        total_volume += volume;

        FemTetrahedron element;
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
    }

    // A quarter of each element's mass to each of its four vertices — the
    // same lumped-mass scheme `tetrahedral_mesh.cpp` uses. Accumulated as mass
    // first and inverted once per vertex afterward, since a vertex shared by
    // several elements must sum their contributions before the reciprocal is
    // taken, not average the reciprocals.
    std::vector<Scalar> accumulated_mass(model.particles.size(), Scalar(0));
    for (std::size_t t = 0; t < element_count; ++t)
    {
        const FemTetrahedron& element = model.elements[t];
        const Scalar vertex_mass =
            element.rest_volume * model.material.density / Scalar(4);
        for (int i = 0; i < 4; ++i)
            accumulated_mass[element.vertex[i]] += vertex_mass;
    }
    for (std::size_t i = 0; i < model.particles.size(); ++i)
        model.particles[i].inv_mass =
            accumulated_mass[i] > Scalar(0) ? Scalar(1) / accumulated_mass[i] : Scalar(0);

    // Pin the wall end: every vertex at x = 0.
    for (std::size_t y = 0; y <= WIDTH_CELLS; ++y)
        for (std::size_t z = 0; z <= HEIGHT_CELLS; ++z)
            model.particles[vertex_index(0, y, z, WIDTH_CELLS, HEIGHT_CELLS)].inv_mass = Scalar(0);

    const Scalar dt = Scalar(1.0 / 60.0);
    // §16.19's 9x mystery, resolved (2026-08-01): the deviatoric constraint was
    // `||F|| - sqrt(3)`, which scales every deviatoric force by a factor that
    // vanishes at small strain — see `evaluate_deviatoric_constraint`'s own
    // documentation for the full account. With the constraint corrected to
    // `||F||` (plus Smith et al.'s `lambda + mu` reparameterization in the
    // hydrostatic term), what remains is ordinary single-iteration XPBD
    // convergence: the rest state carries a residual contraction that shrinks
    // as h^2 (measured: -2.2e-2 m at 30 substeps, -5.9e-3 at 60, -1.5e-3 at
    // 120, on the axial version of this scene). 60 substeps is where the
    // remaining bias fits inside this test's 35% discretization tolerance
    // without doubling the runtime again.
    constexpr std::size_t SUBSTEPS = 60;
    constexpr int TICKS = 3000; // 50 s of simulated time, heavily damped
    for (int tick = 0; tick < TICKS; ++tick)
        model.step(dt, SUBSTEPS);

    // Tip deflection: how far the free end's vertices have sagged in Z from
    // where they started.
    Scalar tip_z_deflection = 0;
    int tip_vertex_count = 0;
    for (std::size_t y = 0; y <= WIDTH_CELLS; ++y)
        for (std::size_t z = 0; z <= HEIGHT_CELLS; ++z)
        {
            const std::uint32_t index =
                vertex_index(LENGTH_CELLS, y, z, WIDTH_CELLS, HEIGHT_CELLS);
            const Scalar rest_z = mesh.vertices[index].z;
            tip_z_deflection += rest_z - model.particles[index].position.z;
            ++tip_vertex_count;
        }
    tip_z_deflection /= Scalar(tip_vertex_count);

    // Euler-Bernoulli, distributed self-weight load:
    //   delta = mass * g * L^3 / (8 * E * I),  I = width * height^3 / 12.
    const Scalar length = Scalar(LENGTH_CELLS) * cell_size;
    const Scalar width = Scalar(WIDTH_CELLS) * cell_size;
    const Scalar height = Scalar(HEIGHT_CELLS) * cell_size;
    const Scalar total_mass = total_volume * model.material.density;
    const Scalar second_moment_of_area = width * height * height * height / Scalar(12);
    const Scalar expected_deflection = (total_mass * Scalar(9.81) * length * length * length) /
                                       (Scalar(8) * model.material.young_modulus *
                                        second_moment_of_area);

    ASSERT_GT(double(expected_deflection), 0.0);
    // The expected deflection should itself be a small fraction of the beam's
    // length, or this scene is not the small-deflection regime the theory
    // assumes and the test's own setup would be at fault, not the model.
    ASSERT_LT(double(expected_deflection), double(length) * 0.2);

    EXPECT_GT(double(tip_z_deflection), 0.0) << "the beam did not sag at all";
    // A generous tolerance: ten cells along the beam's length is a coarse
    // discretization of a continuum theory, and this is the tolerance a
    // 10x2x2 tetrahedral lattice earns, not the tolerance the theory itself
    // is capable of at finer resolution.
    EXPECT_NEAR(double(tip_z_deflection), double(expected_deflection),
                double(expected_deflection) * 0.35);
}
