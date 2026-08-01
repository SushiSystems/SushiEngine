/**************************************************************************/
/* test_penetration_contract.cpp                                         */
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

// Regression_PenetrationContract: §15.4's golden-metric scenes for §0.4's
// contract — "what the player sees never interpenetrates more deeply than
// what the solver resolved" — measured end to end through the real seam
// (IPhysicsScene), not asserted in prose.
//
// Three scenes, matching the phase's own acceptance criterion word for word:
// resting penetration stays within tolerance, nothing tunnels at the tested
// speeds, and the Hausdorff error is reported per asset.
//
// The tunnelling scene has no way to seed a body's velocity directly — the
// live `IRigidBodyService` always starts a new body at rest (by design: a
// reconciled body keeps its live velocity, and a genuinely new one has none
// yet to keep) — so the scene builds its speed the only way the public seam
// allows: a strong acceleration for a few ticks while still far above
// anything, then an unaccelerated coast at that exact speed into the target.
// Semi-implicit Euler integrates a constant acceleration exactly, so the
// speed at the end of the ramp is exact, not approximate.

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/SushiEngine.hpp>
#include <SushiEngine/geometry/triangle_mesh.hpp>
#include <SushiEngine/physics/cooking/collision_cooker.hpp>
#include <SushiEngine/sim/physics_simulation.hpp>

#include "test_helpers.hpp"

using namespace SushiEngine;
using namespace SushiEngine::Simulation;

namespace
{
    constexpr std::size_t ITERATIONS = 8;
    constexpr std::size_t SUBSTEPS = 4;
    constexpr Scalar SUBSTEP_DT = Scalar(1.0 / 240.0);
    constexpr Scalar TICK_DT = Scalar(1.0 / 60.0); // SUBSTEP_DT * SUBSTEPS

    GravitySampler earth_gravity()
    {
        return [](const Vector3&) { return Vector3{0, Scalar(-9.8), 0}; };
    }

    GravitySampler no_gravity()
    {
        return [](const Vector3&) { return Vector3{0, 0, 0}; };
    }

    GravitySampler constant_downward_acceleration(Scalar magnitude)
    {
        return [magnitude](const Vector3&) { return Vector3{0, -magnitude, 0}; };
    }

    Collider box_collider(Vector3 half_extents)
    {
        Collider collider;
        collider.shape = ColliderShape::Box;
        collider.half_extents = half_extents;
        return collider;
    }

    Collider sphere_collider(Scalar radius)
    {
        Collider collider;
        collider.shape = ColliderShape::Sphere;
        collider.radius = radius;
        return collider;
    }

    std::vector<PlaneDesc> ground()
    {
        PlaneDesc plane;
        plane.point = Vector3{0, 0, 0};
        plane.normal = Vector3{0, 1, 0};
        return {plane};
    }
} // namespace

// A stack of crates must come to rest at its analytic height, not floating on
// a fat contact or sunk into one another — the maximum-penetration-at-rest
// half of §7.6's contract.
TEST(Regression_PenetrationContract, StackedCratesRestWithinTolerance)
{
    auto physics = create_physics_simulation(Harness::shared_runtime());

    constexpr int CRATE_COUNT = 3;
    std::vector<RigidBodyDesc> bodies;
    for (int i = 0; i < CRATE_COUNT; ++i)
    {
        RigidBodyDesc desc;
        desc.id = i + 1;
        // Seeded with a small gap between crates so each one actually falls and
        // makes its own contact, rather than starting already touching.
        desc.position = Vector3{0, Scalar(0.5) + Scalar(i) * Scalar(1.05), 0};
        desc.inv_mass = Scalar(1);
        desc.collider = box_collider(Vector3{0.5, 0.5, 0.5});
        bodies.push_back(desc);
    }
    physics->set_rigid_bodies(bodies, ITERATIONS, SUBSTEP_DT);
    physics->set_static_planes(ground());

    for (int tick = 0; tick < 900; ++tick)
        physics->step(earth_gravity(), SUBSTEPS);

    for (int i = 0; i < CRATE_COUNT; ++i)
    {
        SolvedPose pose;
        ASSERT_TRUE(physics->rigid_pose(i + 1, pose));
        const double expected = 0.5 + double(i) * 1.0;
        EXPECT_NEAR(double(pose.position.y), expected, 5e-2)
            << "crate " << i << " rests outside the penetration/gap tolerance";
    }
}

// A fast, small body against a thin static plate — the tunnelling half of
// §7.5/§7.6's contract, at the exact scale the phase's own acceptance
// criterion names: "a 200 m/s sphere through a 1 cm plate must not pass."
// Run at three escalating speeds, because a mechanism that only happens to
// catch one magnitude is not the same claim as one that catches a range.
TEST(Regression_PenetrationContract, FastSphereDoesNotTunnelThroughAThinPlate)
{
    const Scalar plate_half_thickness = Scalar(0.005); // a 1 cm plate
    const Scalar sphere_radius = Scalar(0.1);
    const Scalar start_height = Scalar(20.0);
    const Scalar expected_rest = plate_half_thickness + sphere_radius;

    for (const Scalar target_speed : {Scalar(50.0), Scalar(100.0), Scalar(200.0)})
    {
        SCOPED_TRACE("target speed " + std::to_string(double(target_speed)) + " m/s");

        auto physics = create_physics_simulation(Harness::shared_runtime());

        RigidBodyDesc plate;
        plate.id = 1;
        plate.position = Vector3{0, 0, 0};
        plate.inv_mass = Scalar(0); // static
        plate.collider = box_collider(Vector3{2.0, plate_half_thickness, 2.0});

        RigidBodyDesc sphere;
        sphere.id = 2;
        sphere.position = Vector3{0, start_height, 0};
        sphere.inv_mass = Scalar(1);
        sphere.collider = sphere_collider(sphere_radius);

        physics->set_rigid_bodies({plate, sphere}, ITERATIONS, SUBSTEP_DT);

        // Ramp: a few ticks of strong acceleration while still far above the
        // plate, reaching exactly `target_speed` at the end of it (semi-implicit
        // Euler sums `acceleration * h` once per sub-step, so the total over a
        // whole tick is exact regardless of the sub-step count).
        constexpr std::size_t RAMP_TICKS = 4;
        const Scalar ramp_acceleration = target_speed / (Scalar(RAMP_TICKS) * TICK_DT);
        for (std::size_t tick = 0; tick < RAMP_TICKS; ++tick)
            physics->step(constant_downward_acceleration(ramp_acceleration), SUBSTEPS);

        // Coast at that exact speed the rest of the way down and well past it,
        // so the scene both approaches and settles within the same run.
        const Scalar travel_per_tick = target_speed * TICK_DT;
        const std::size_t coast_ticks =
            std::size_t(double(start_height) / double(travel_per_tick)) + 40;
        for (std::size_t tick = 0; tick < coast_ticks; ++tick)
            physics->step(no_gravity(), SUBSTEPS);

        SolvedPose pose;
        ASSERT_TRUE(physics->rigid_pose(2, pose));
        // The plate must have stopped it, not merely slowed it: tunnelling
        // through and coasting away shows up as a large negative Y, nowhere
        // near the plate's own thickness.
        EXPECT_GT(double(pose.position.y), 0.0);
        EXPECT_NEAR(double(pose.position.y), double(expected_rest), 0.1);
    }
}

namespace
{
    const std::uint32_t BOX_FACES[12][3] = {{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7},
                                            {0, 1, 5}, {0, 5, 4}, {3, 7, 6}, {3, 6, 2},
                                            {0, 4, 7}, {0, 7, 3}, {1, 2, 6}, {1, 6, 5}};

    /** @brief An outward-wound box, optionally offset. */
    Geometry::TriangleMesh box_mesh(float hx, float hy, float hz, float ox = 0.0f,
                                    float oy = 0.0f, float oz = 0.0f)
    {
        Geometry::TriangleMesh mesh;
        const float corners[8][3] = {{-hx, -hy, -hz}, {hx, -hy, -hz}, {hx, hy, -hz},
                                     {-hx, hy, -hz},  {-hx, -hy, hz}, {hx, -hy, hz},
                                     {hx, hy, hz},    {-hx, hy, hz}};
        for (const auto& corner : corners)
        {
            mesh.positions.push_back(corner[0] + ox);
            mesh.positions.push_back(corner[1] + oy);
            mesh.positions.push_back(corner[2] + oz);
        }
        for (const auto& face : BOX_FACES)
        {
            mesh.indices.push_back(face[0]);
            mesh.indices.push_back(face[1]);
            mesh.indices.push_back(face[2]);
        }
        return mesh;
    }

    /** @brief Appends @p addition's geometry to @p target, renumbering its indices. */
    void append_mesh(Geometry::TriangleMesh& target, const Geometry::TriangleMesh& addition)
    {
        const std::uint32_t offset = std::uint32_t(target.vertex_count());
        target.positions.insert(target.positions.end(), addition.positions.begin(),
                                addition.positions.end());
        for (const std::uint32_t index : addition.indices)
            target.indices.push_back(index + offset);
    }

    /** @brief Two overlapping bars: the simplest solid a single convex hull cannot match. */
    Geometry::TriangleMesh l_shaped_mesh()
    {
        Geometry::TriangleMesh mesh = box_mesh(1.0f, 0.25f, 0.25f);
        append_mesh(mesh, box_mesh(0.25f, 1.0f, 0.25f, -0.75f, 0.75f, 0.0f));
        return mesh;
    }
} // namespace

// The cooker must surface the collider's Hausdorff error against the source
// mesh (§7.6's "the number an artist can see and fix"), for both the case
// where it is genuinely zero (a box's hull is exact) and the case where it
// cannot be (an L cannot be one convex piece), rather than reporting the same
// number regardless of the input.
TEST(Regression_PenetrationContract, HausdorffErrorIsReportedPerAsset)
{
    using namespace SushiEngine::Physics::Cooking;

    CollisionCooker cooker;
    CookingParameters parameters;
    parameters.fidelity = 0.5f;
    parameters.distance_field_resolution = 16;

    {
        const Geometry::TriangleMesh box = box_mesh(0.5f, 0.5f, 0.5f);
        std::vector<std::byte> bytes;
        const CookingReport report = cooker.cook(box.view(), parameters, nullptr, nullptr, bytes);
        ASSERT_EQ(report.status, CookingStatus::Succeeded);
        EXPECT_GE(report.hausdorff_error, 0.0f);
        EXPECT_NEAR(report.hausdorff_error, 0.0f, 1e-3f);
    }

    {
        const Geometry::TriangleMesh l_shape = l_shaped_mesh();
        CookingParameters single_piece = parameters;
        // Pinned to one piece deliberately: `Unit_ConvexDecomposition.
        // SplitsAConcaveShapeAndTheErrorFalls` already establishes that a
        // *budgeted* decomposition can split this exact L back into two exact
        // boxes with close to no error, which would make this scene's outcome
        // depend on how good the decomposer happens to be rather than on
        // whether the report plumbing works. Forcing one piece asks the
        // question this test actually cares about: a single hull around an L
        // necessarily fills the open corner, so the reported error must be a
        // real, bounded, positive number — not zero (the measurement was
        // skipped) and not a runaway value (the decomposition failed silently).
        single_piece.convex_piece_count = 1;
        std::vector<std::byte> bytes;
        const CookingReport report =
            cooker.cook(l_shape.view(), single_piece, nullptr, nullptr, bytes);
        ASSERT_EQ(report.status, CookingStatus::Succeeded);
        EXPECT_EQ(report.convex_piece_count, 1u);
        EXPECT_GT(report.hausdorff_error, 0.01f);
        EXPECT_LT(report.hausdorff_error, 1.0f);
    }
}
