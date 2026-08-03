/**************************************************************************/
/* test_fem_plasticity_integration.cpp                                   */
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

// Integration_FEMPlasticity: §9.4's own acceptance phrase — "a body past
// yield keeps a permanent dent" — run through FiniteElementModel::step()'s
// real substep loop rather than by calling apply_fem_plasticity directly the
// way the unit tests do. A single tetrahedron, pinned at one vertex, is
// pulled hard enough and long enough to yield a low-yield material, the pull
// is released, and the body is left to settle. Unlike the unit tests (which
// check the *exact* full-creep and ceiling-clamping arithmetic), this only
// checks the qualitative claim the doc actually makes: plastic strain
// accumulated, the rest shape changed, and the settled shape did not spring
// back to the original — the magnitude a from-scratch dynamic scene produces
// is not something to hard-code an exact number against.

#include <cmath>

#include <gtest/gtest.h>

#include <SushiEngine/physics/soft/finite_element_model.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;

TEST(Integration_FEMPlasticity, APulledElementKeepsAPermanentDentAfterTheLoadIsRemoved)
{
    FiniteElementModel<Scalar> model;
    model.material.young_modulus = 1.0e5; // soft enough to move visibly this test's tick budget
    model.material.poisson_ratio = 0.3;
    model.material.density = 1000.0;
    model.material.damping = 2.0; // heavily damped: this test wants the settled answer
    model.material.yield_stress = 1.0e3; // low: this scene is meant to yield easily
    model.material.plastic_creep = 0.2;
    model.material.maximum_plastic_strain = 10.0; // generous; not what this test is checking

    model.particles.resize(4);
    model.particles[0].position = Vector3{0.0, 0.0, 0.0};
    model.particles[1].position = Vector3{1.0, 0.0, 0.0};
    model.particles[2].position = Vector3{0.0, 1.0, 0.0};
    model.particles[3].position = Vector3{0.0, 0.0, 1.0};
    for (auto& particle : model.particles)
    {
        particle.previous_position = particle.position;
        particle.orientation = Quaternion{0.0, 0.0, 0.0, 1.0};
        particle.previous_orientation = particle.orientation;
        particle.inv_mass = Scalar(1.0);
    }
    model.particles[0].inv_mass = Scalar(0.0); // the anchor the load pulls against

    FEMTetrahedron element;
    element.vertex[0] = 0;
    element.vertex[1] = 1;
    element.vertex[2] = 2;
    element.vertex[3] = 3;
    element.rest_inverse_column_0 = Vector3{1.0, 0.0, 0.0};
    element.rest_inverse_column_1 = Vector3{0.0, 1.0, 0.0};
    element.rest_inverse_column_2 = Vector3{0.0, 0.0, 1.0};
    element.plastic_inverse_column_0 = element.rest_inverse_column_0;
    element.plastic_inverse_column_1 = element.rest_inverse_column_1;
    element.plastic_inverse_column_2 = element.rest_inverse_column_2;
    element.rest_volume = 1.0 / 6.0;
    model.elements.push_back(element);

    const Scalar dt = Scalar(1.0 / 60.0);
    constexpr std::size_t SUBSTEPS = 20;

    // Load: pull hard, well past this material's low yield. 800 m/s^2 on three
    // 1 kg vertices puts a few kilopascals through the element — several times
    // the 1 kPa yield at the correctly-mapped stiffness. (The original 50 was
    // calibrated against the pre-§16.19-fix material, which was soft enough
    // that even that gentle pull crossed yield.)
    model.external_acceleration = Vector3{800.0, 0.0, 0.0};
    for (int tick = 0; tick < 60; ++tick)
        model.step(dt, SUBSTEPS);

    // Release: let it settle with no further load.
    model.external_acceleration = Vector3{0.0, 0.0, 0.0};
    for (int tick = 0; tick < 180; ++tick)
        model.step(dt, SUBSTEPS);

    // It must actually have yielded — this scene's whole point.
    EXPECT_GT(model.elements[0].accumulated_plastic_strain, 0.0);

    // The rest volume must have followed the permanent stretch rather than
    // staying at the original unit tetrahedron's 1/6.
    EXPECT_GT(model.elements[0].rest_volume, 1.0 / 6.0 + 1e-4);

    // And the settled shape must not have sprung back to the original rest
    // positions: the pulled vertex should still be measurably further from
    // the anchor than the original unit edge length.
    const Scalar settled_distance =
        length(model.particles[1].position - model.particles[0].position);
    EXPECT_GT(double(settled_distance), 1.05);
}
