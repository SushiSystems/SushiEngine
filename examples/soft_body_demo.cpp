/**************************************************************************/
/* soft_body_demo.cpp                                                    */
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

// Two Phase-4 physics additions, worked and self-checked headlessly:
//   * A volumetric soft body: a 4x4x4 XPBD distance-constraint lattice pinned at its
//     top, stepped under gravity on the real solver, then checked to have kept its
//     structural rest length (a rigid lattice sags only slightly).
//   * Collision contacts: a free particle dropped under gravity with ground contact
//     resolution each sub-step, checked to come to rest exactly on the plane.

#include <cmath>
#include <cstddef>
#include <cstdio>

#include <SushiEngine/SushiEngine.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    constexpr std::size_t N = 4;
    constexpr std::size_t SUBSTEPS = 4;
}

int main()
{
    auto runtime = SushiRuntime::API::Runtime::create();

    // --- Soft body: a pinned-top lattice that holds its shape under gravity --------
    const Scalar spacing = Scalar(0.5);
    const Scalar h = Scalar(1.0 / 60.0) / Scalar(SUBSTEPS);
    const Vector3 origin{0, Scalar(3), 0};

    PhysicsWorld<XpbdDistanceConstraint> world(runtime);
    // Pin the y == 0 layer (the bottom of the lattice's own frame) so the block hangs.
    const SoftBodyLattice lattice =
        build_soft_body_lattice(world, N, N, N, spacing, origin, Scalar(0), /*pin_bottom=*/true);
    world.finalize(16, h, XpbdDistanceProjection{});

    for (int step = 0; step < 120; ++step)
        world.step(Vector3{0, Scalar(-9.8), 0}, SUBSTEPS);

    const RigidBody& top = world.body(lattice.at(0, N - 1, 0));
    const RigidBody& below = world.body(lattice.at(0, N - 2, 0));
    const Scalar gap = length(top.position - below.position);
    const bool soft_body_ok =
        world.body_count() == N * N * N && world.compile_count() == 1 &&
        std::fabs(double(gap) - double(spacing)) < 0.2;

    // --- Contacts: a dropped particle rests exactly on the ground ------------------
    RigidBody particle;
    particle.position = Vector3{0, Scalar(5), 0};
    particle.inv_mass = Scalar(1);
    const Scalar radius = Scalar(0.5);
    const PlaneCollider<Scalar> ground; // y = 0, up

    for (int step = 0; step < 400; ++step)
    {
        predict(particle, Vector3{0, Scalar(-9.8), 0}, h * Scalar(SUBSTEPS));
        resolve_contacts(&particle, &radius, 1, ground, 4);
        update_velocity(particle, h * Scalar(SUBSTEPS));
    }
    const bool contact_ok = std::fabs(double(particle.position.y) - 0.5) < 1e-2;

    std::printf("soft_body_demo: bodies=%zu gap=%.4f (rest=%.4f) resting_y=%.4f\n",
                world.body_count(), double(gap), double(spacing), double(particle.position.y));
    std::printf("soft_body_ok=%s contact_ok=%s\n", soft_body_ok ? "true" : "false",
                contact_ok ? "true" : "false");

    const bool ok = soft_body_ok && contact_ok;
    std::printf("RESULT: %s\n", ok ? "OK" : "FAIL");
    return ok ? 0 : 1;
}
