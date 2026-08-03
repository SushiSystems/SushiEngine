/**************************************************************************/
/* test_soft_self_collision.cpp                                           */
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

// §9.6.3: a soft body's surface against itself.
//
// The scene is one body with two disconnected halves — a single particle array,
// a single element list, two slabs — which is the cleanest way to put the
// mechanism under test rather than the surrounding machinery. Nothing about it
// reaches the two-body collider: if the upper half stays up, self-collision is
// what held it, and if it falls through, self-collision is what was missing.
// So the same scene is run twice, once with the flag and once without, and the
// two results are asserted separately.
//
// Two smaller claims come first, because the integration scene depends on both:
// the grid must offer each pair of triangles once rather than once per shared
// cell, and a triangle must never be tested against one it shares a vertex
// with — every surface is permanently "touching" itself at every corner, and a
// body that answered those would spend its life pushing itself apart.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/physics/soft/soft_self_collision.hpp>

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
            particle.prev_position = particle.position;
        }
    }

    /**
     * @brief One model holding both halves, with the second half's indices rebased.
     *
     * Two disconnected components of a single body: nothing links them, so the
     * only thing that can keep them apart is a contact the body makes with
     * itself.
     */
    FiniteElementModel<Scalar> merge(const FiniteElementModel<Scalar>& lower,
                                     const FiniteElementModel<Scalar>& upper)
    {
        FiniteElementModel<Scalar> merged = lower;
        const std::uint32_t offset = std::uint32_t(lower.particles.size());

        merged.particles.insert(merged.particles.end(), upper.particles.begin(),
                                upper.particles.end());
        for (FEMTetrahedron element : upper.elements)
        {
            for (int i = 0; i < 4; ++i)
                element.vertex[i] += offset;
            merged.elements.push_back(element);
        }
        for (const std::uint32_t index : upper.surface_indices)
            merged.surface_indices.push_back(index + offset);
        for (const std::uint32_t index : upper.surface_vertices)
            merged.surface_vertices.push_back(index + offset);
        return merged;
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
     * @brief Drops one half of a body onto the other, pinned half.
     *
     * @param self_collision Whether the body opts into testing itself.
     * @return The lowest Z the falling half reached.
     */
    Scalar dropped_half_resting_height(bool self_collision)
    {
        const TetrahedralLattice lattice = build_tetrahedral_lattice(2, 2, 2, Scalar(0.05));

        FiniteElementModel<Scalar> lower = build_lattice_model(lattice, compliant_material());
        FiniteElementModel<Scalar> upper = build_lattice_model(lattice, compliant_material());
        translate(upper, Vector3{0, 0, Scalar(0.16)});

        FiniteElementModel<Scalar> body = merge(lower, upper);
        body.external_acceleration = Vector3{0, 0, Scalar(-9.81)};
        body.collision.thickness = Scalar(0.01);
        body.collision.self_collision = self_collision;

        const std::size_t half = lower.particles.size();
        for (std::size_t i = 0; i < half; ++i)
            body.particles[i].inv_mass = Scalar(0);

        SoftSelfCollider<Scalar> collider;
        collider.surface = view_of(body);
        body.collider = &collider;

        for (int tick = 0; tick < 120; ++tick)
            body.step(Scalar(1.0 / 60.0), 30);

        Scalar lowest = body.particles[half].position.z;
        for (std::size_t i = half; i < body.particles.size(); ++i)
            if (body.particles[i].position.z < lowest)
                lowest = body.particles[i].position.z;
        return lowest;
    }
} // namespace

TEST(Unit_SoftSurfaceGrid, OffersEachTrianglePairOnceHoweverManyCellsTheyShare)
{
    const TetrahedralLattice lattice = build_tetrahedral_lattice(2, 2, 2, Scalar(0.05));
    const FiniteElementModel<Scalar> model = build_lattice_model(lattice, compliant_material());

    std::vector<Vector3> positions(model.particles.size());
    for (std::size_t i = 0; i < model.particles.size(); ++i)
        positions[i] = model.particles[i].position;

    SoftSurfaceGrid<Scalar> grid;
    // A cell much smaller than a triangle, so every triangle spans many of them
    // and any pair that shares one shares several.
    grid.build(positions.data(), model.surface_indices.data(),
               std::uint32_t(model.surface_indices.size() / 3), Scalar(0.01));

    std::vector<SoftTrianglePair> pairs;
    grid.collect_pairs(pairs);

    ASSERT_FALSE(pairs.empty());
    for (const SoftTrianglePair& pair : pairs)
        EXPECT_LT(pair.first, pair.second) << "a pair should be offered in one direction only";
    for (std::size_t i = 1; i < pairs.size(); ++i)
    {
        const bool ascending = pairs[i - 1].first < pairs[i].first ||
                               (pairs[i - 1].first == pairs[i].first &&
                                pairs[i - 1].second < pairs[i].second);
        EXPECT_TRUE(ascending) << "pairs should be strictly ascending, so none repeats";
    }
}

TEST(Unit_SoftSelfCollision, NeighbouringTrianglesNeverContactEachOther)
{
    // A single cell: twelve surface triangles, every pair of which either shares
    // a vertex or is separated by real geometry. The thickness has to be chosen
    // against that geometry rather than against the cell size, and the number
    // that matters is smaller than it looks: two *edges* of a 0.05 m cube that
    // share no vertex still pass within 0.05/sqrt(3) = 0.0289 m of each other,
    // because the closest approach of a face diagonal and a cube edge is not
    // along an axis. A combined thickness above that would make the cube
    // genuinely, correctly self-intersecting — and an earlier version of this
    // case set exactly that and read the correct answer as a bug.
    //
    // At 0.01 each, 0.02 combined, the only contacts reachable are the
    // topological ones, which is what makes an empty result meaningful.
    const TetrahedralLattice lattice = build_tetrahedral_lattice(1, 1, 1, Scalar(0.05));
    FiniteElementModel<Scalar> model = build_lattice_model(lattice, compliant_material());
    model.collision.thickness = Scalar(0.01);
    model.collision.self_collision = true;

    SoftSelfCollider<Scalar> collider;
    collider.surface = view_of(model);
    collider.generate_contacts(model.particles.data(), model.particles.size(), Scalar(1.0 / 60.0));

    EXPECT_GT(collider.candidate_count(), 0u) << "the broad phase should have found pairs";
    EXPECT_TRUE(collider.contacts().empty())
        << "every pair here shares a vertex, so none of them is a contact";
}

TEST(Integration_SoftSelfCollision, TheFallingHalfRestsOnThePinnedHalf)
{
    const Scalar resting = dropped_half_resting_height(true);

    // The pinned half's top face is at 0.1 and both surfaces carry a centimetre
    // of thickness, so the falling half belongs at 0.12.
    EXPECT_GT(double(resting), 0.1) << "the halves passed through each other";
    EXPECT_NEAR(double(resting), 0.12, 0.006);
}

TEST(Integration_SoftSelfCollision, TheFallingHalfPassesThroughWhenItIsOff)
{
    // The default, and deliberately asserted: self-collision costs a grid and a
    // narrow phase every tick, and a body that does not need it must not be
    // paying for it. If this ever starts resting, the flag has stopped meaning
    // anything.
    EXPECT_LT(double(dropped_half_resting_height(false)), 0.1);
}
