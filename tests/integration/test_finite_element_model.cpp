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
// The beam mesh is a Kuhn six-tetrahedra-per-cube lattice from
// `tests/common/tetrahedral_lattice.hpp` rather than something the cooker
// produced: the cooker's job is turning arbitrary dirty meshes into this kind
// of lattice, and is exercised on its own terms in `test_soft_body_cooker.cpp`.
// This test needs a *known*, regular lattice to have a closed-form answer to
// check against, which a voxelized-and-conformed mesh would not give as cleanly.

#include <cmath>
#include <cstddef>
#include <cstdint>

#include <gtest/gtest.h>

#include <SushiEngine/physics/soft/finite_element_model.hpp>

#include "tetrahedral_lattice.hpp"

using namespace SushiEngine;
using namespace SushiEngine::Physics;
using namespace SushiEngine::Harness;


TEST(Integration_FiniteElementModel, CantileverTipDeflectionMatchesEulerBernoulli)
{
    constexpr std::size_t LENGTH_CELLS = 10;
    constexpr std::size_t WIDTH_CELLS = 2;
    constexpr std::size_t HEIGHT_CELLS = 2;
    const Scalar cell_size = Scalar(0.1); // a 1.0 x 0.2 x 0.2 m beam

    const TetrahedralLattice lattice =
        build_tetrahedral_lattice(LENGTH_CELLS, WIDTH_CELLS, HEIGHT_CELLS, cell_size);

    // A soft, rubber-scale material chosen so the beam sags a clearly
    // measurable amount while staying inside the small-deflection regime
    // Euler-Bernoulli theory assumes (a handful of percent of the beam's own
    // length, not a fraction of it folding over).
    SoftBodyMaterial material;
    material.young_modulus = Scalar(1.84e7);
    material.poisson_ratio = Scalar(0.3);
    material.density = Scalar(1000.0);
    material.damping = Scalar(3.0); // heavily damped: this test wants the static answer

    FiniteElementModel<Scalar> model = build_lattice_model(lattice, material);
    model.external_acceleration = Vector3{0, 0, Scalar(-9.81)};
    // Every element of a sign-corrected lattice must have survived; one dropped
    // as degenerate would quietly remove stiffness this test then measures.
    ASSERT_EQ(model.elements.size(), lattice.tetrahedra.size() / 4);

    Scalar total_volume = 0;
    for (const FEMTetrahedron& element : model.elements)
        total_volume += element.rest_volume;

    // Pin the wall end: every vertex at x = 0.
    for (std::size_t y = 0; y <= WIDTH_CELLS; ++y)
        for (std::size_t z = 0; z <= HEIGHT_CELLS; ++z)
            model.particles[lattice_vertex_index(0, y, z, WIDTH_CELLS, HEIGHT_CELLS)].inv_mass =
                Scalar(0);

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
                lattice_vertex_index(LENGTH_CELLS, y, z, WIDTH_CELLS, HEIGHT_CELLS);
            const Scalar rest_z = lattice.vertices[index].z;
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
