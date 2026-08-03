/**************************************************************************/
/* test_soft_soft_collision.cpp                                           */
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

// §9.6.2: two deformable surfaces against each other.
//
// Four claims, in order of how much they depend on each other: the closest-
// feature routines return the weights the correction is split by, the
// coplanarity solve finds the instant a crossing happens, a soft body dropped
// on another soft body rests on it, and a sheet moving fast enough to cross
// another sheet within a single substep is stopped when — and only when — its
// body is marked continuous. That last pair is the whole argument for the
// continuous test existing, so it is tested both ways round rather than only
// in the configuration that passes.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/physics/soft/soft_body_scene.hpp>
#include <SushiEngine/physics/soft/soft_soft_collision.hpp>

#include "tetrahedral_lattice.hpp"

using namespace SushiEngine;
using namespace SushiEngine::Physics;
using namespace SushiEngine::Harness;

namespace
{
    SoftBodyMaterial compliant_material()
    {
        SoftBodyMaterial material;
        material.young_modulus = Scalar(1e6);
        material.poisson_ratio = Scalar(0.3);
        material.density = Scalar(1000);
        material.damping = Scalar(2.0);
        return material;
    }

    void translate(FiniteElementModel<Scalar>& model, const Vector3& offset)
    {
        for (RigidBodyT<Scalar>& particle : model.particles)
        {
            particle.position = particle.position + offset;
            particle.previous_position = particle.position;
        }
    }

    void pin_every_particle(FiniteElementModel<Scalar>& model)
    {
        for (RigidBodyT<Scalar>& particle : model.particles)
            particle.inv_mass = Scalar(0);
    }

    Scalar lowest_z(const FiniteElementModel<Scalar>& model)
    {
        Scalar lowest = model.particles[0].position.z;
        for (const RigidBodyT<Scalar>& particle : model.particles)
            if (particle.position.z < lowest)
                lowest = particle.position.z;
        return lowest;
    }

    SoftSurfaceView<Scalar> view_of(FiniteElementModel<Scalar>& model)
    {
        SoftSurfaceView<Scalar> view;
        view.particles = model.particles.data();
        view.particle_count = model.particles.size();
        view.surface_indices = model.surface_indices.data();
        view.index_count = model.surface_indices.size();
        view.collision = model.collision;
        return view;
    }

    /**
     * @brief Fires a slab at a pinned slab fast enough to cross it in one substep.
     *
     * 300 m/s over a substep of 1/1200 s is a quarter of a metre of travel
     * against a slab five centimetres thick: whichever substep the crossing
     * happens in, the slab is comfortably clear on the far side by the end of
     * it, and nowhere near the other one at the tick's start when a discrete
     * contact set would have been built.
     *
     * @param continuous Whether the moving slab opts into the swept test.
     * @return True if the slab ended up on the far side — that is, tunnelled.
     */
    bool sheet_passes_through(bool continuous)
    {
        const TetrahedralLattice lattice = build_tetrahedral_lattice(2, 2, 1, Scalar(0.05));

        FiniteElementModel<Scalar> stationary = build_lattice_model(lattice, compliant_material());
        pin_every_particle(stationary);
        stationary.collision.thickness = Scalar(0.005);

        FiniteElementModel<Scalar> moving = build_lattice_model(lattice, compliant_material());
        translate(moving, Vector3{0, 0, Scalar(0.6)});
        moving.collision.thickness = Scalar(0.005);
        moving.collision.continuous = continuous;
        for (RigidBodyT<Scalar>& particle : moving.particles)
            particle.velocity = Vector3{0, 0, Scalar(-300)};

        SoftSoftCollider<Scalar> collider;
        collider.first = view_of(moving);
        collider.second = view_of(stationary);
        collider.build();

        SoftBodyScene<Scalar> scene;
        scene.add_body(&stationary);
        scene.add_body(&moving);
        scene.add_pair_collider(&collider);
        scene.step(Scalar(1.0 / 60.0), 20);

        // The stationary slab's underside is at zero, so anything below that
        // went through it rather than off it.
        return double(lowest_z(moving)) < 0.0;
    }
} // namespace

TEST(Unit_ClosestPoint, TriangleWeightsSumToOneAndLocateTheRegion)
{
    const Vector3 a{0, 0, 0};
    const Vector3 b{1, 0, 0};
    const Vector3 c{0, 1, 0};

    Scalar weight[3];
    const Vector3 interior =
        closest_point_on_triangle(Vector3{Scalar(0.25), Scalar(0.25), Scalar(1)}, a, b, c, weight);
    EXPECT_NEAR(double(weight[0] + weight[1] + weight[2]), 1.0, 1e-12);
    EXPECT_NEAR(double(interior.z), 0.0, 1e-12);
    EXPECT_NEAR(double(weight[1]), 0.25, 1e-12);
    EXPECT_NEAR(double(weight[2]), 0.25, 1e-12);

    // Beyond the first corner: all of the weight belongs to it, so a correction
    // there pushes only the corner and not the two far ones.
    closest_point_on_triangle(Vector3{Scalar(-1), Scalar(-1), Scalar(0)}, a, b, c, weight);
    EXPECT_NEAR(double(weight[0]), 1.0, 1e-12);
    EXPECT_NEAR(double(weight[1]), 0.0, 1e-12);
    EXPECT_NEAR(double(weight[2]), 0.0, 1e-12);
}

TEST(Unit_ClosestPoint, CrossingEdgesMeetAtTheirMidpoints)
{
    Scalar along_first = 0;
    Scalar along_second = 0;
    closest_points_on_edges(Vector3{-1, 0, 0}, Vector3{1, 0, 0}, Vector3{0, -1, Scalar(0.5)},
                            Vector3{0, 1, Scalar(0.5)}, along_first, along_second);

    EXPECT_NEAR(double(along_first), 0.5, 1e-12);
    EXPECT_NEAR(double(along_second), 0.5, 1e-12);
}

TEST(Unit_ContinuousProximity, FindsTheInstantAVertexCrossesATriangle)
{
    // A vertex a metre above a static triangle, moving four metres down in the
    // step: it is coplanar with the triangle a quarter of the way through.
    const Vector3 start[4] = {Vector3{Scalar(0.2), Scalar(0.2), Scalar(1)}, Vector3{0, 0, 0},
                              Vector3{1, 0, 0}, Vector3{0, 1, 0}};
    const Vector3 end[4] = {Vector3{Scalar(0.2), Scalar(0.2), Scalar(-3)}, Vector3{0, 0, 0},
                            Vector3{1, 0, 0}, Vector3{0, 1, 0}};

    Scalar time[3];
    const int count = vertex_triangle_coplanarity_times(start, end, time);
    ASSERT_GE(count, 1);
    EXPECT_NEAR(double(time[0]), 0.25, 1e-9);
}

TEST(Unit_ContinuousProximity, FindsNothingWhenTheVertexNeverReachesThePlane)
{
    const Vector3 start[4] = {Vector3{Scalar(0.2), Scalar(0.2), Scalar(1)}, Vector3{0, 0, 0},
                              Vector3{1, 0, 0}, Vector3{0, 1, 0}};
    const Vector3 end[4] = {Vector3{Scalar(0.2), Scalar(0.2), Scalar(0.5)}, Vector3{0, 0, 0},
                            Vector3{1, 0, 0}, Vector3{0, 1, 0}};

    Scalar time[3];
    EXPECT_EQ(vertex_triangle_coplanarity_times(start, end, time), 0);
}

TEST(Integration_SoftSoftCollision, ACubeDroppedOnACubeRestsOnIt)
{
    const TetrahedralLattice lattice = build_tetrahedral_lattice(2, 2, 2, Scalar(0.05));

    FiniteElementModel<Scalar> lower = build_lattice_model(lattice, compliant_material());
    pin_every_particle(lower);
    lower.collision.thickness = Scalar(0.01);

    FiniteElementModel<Scalar> upper = build_lattice_model(lattice, compliant_material());
    translate(upper, Vector3{0, 0, Scalar(0.16)});
    upper.external_acceleration = Vector3{0, 0, Scalar(-9.81)};
    upper.collision.thickness = Scalar(0.01);

    SoftSoftCollider<Scalar> collider;
    collider.first = view_of(upper);
    collider.second = view_of(lower);
    collider.build();

    SoftBodyScene<Scalar> scene;
    scene.add_body(&lower);
    scene.add_body(&upper);
    scene.add_pair_collider(&collider);

    for (int tick = 0; tick < 120; ++tick)
        scene.step(Scalar(1.0 / 60.0), 30);

    // The lower cube's top face is at 0.1; the two thicknesses keep the surfaces
    // 0.02 apart, so the upper cube's underside belongs at 0.12.
    const Scalar resting = lowest_z(upper);
    EXPECT_GT(double(resting), 0.1) << "the upper cube sank into the lower one";
    EXPECT_NEAR(double(resting), 0.12, 0.006);
}

TEST(Integration_SoftSoftCollision, TheSpeculativeMarginStopsTheFastSheetOnItsOwn)
{
    // The discrete path catches a sheet crossing in a single substep on its own,
    // so the continuous flag is not what makes this scene safe.
    //
    // The contact set is built once per tick, so the question is not "where is the
    // sheet at the end of this substep" but "where can it get to during this
    // tick". Sizing the narrow phase's acceptance by the distance a particle can
    // actually travel — the same sizing the broad phase uses — makes the discrete
    // path see the crossing coming and put a speculative contact in front of it.
    // At 300 m/s that margin is a quarter of a metre, and the constraint it
    // creates does nothing until the surfaces really do close.
    EXPECT_FALSE(sheet_passes_through(false));
}

TEST(Integration_SoftSoftCollision, TheContinuousTestNeverLosesWhatTheDiscreteOneCatches)
{
    // The property that makes the flag safe to turn on. The swept pass *adds* to
    // the tick's speculative set rather than replacing it, so enabling it can only
    // ever find more contacts. Replacing the set would let a body marked continuous
    // tunnel through something the same body would have hit with the flag off, and
    // a flag that costs more and detects less is the one shape of bug nobody thinks
    // to look for, so it is pinned here rather than assumed.
    EXPECT_FALSE(sheet_passes_through(true));
}
