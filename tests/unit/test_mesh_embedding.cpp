/**************************************************************************/
/* test_mesh_embedding.cpp                                                */
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

// §8.6's invariants 1 and 2, which are the two the embedding kernel itself is
// responsible for:
//
//   1. Every render vertex is bound, and one that is not is *counted* rather
//      than silently placed somewhere.
//   2. At rest, the reconstructed render mesh reproduces the source mesh — the
//      embedding round-trips.
//
// Invariant 4 — fracture preserves binding — is shared with fracture and is
// covered at the bottom of this file, from the embedding's side. Invariant 3 (no
// lag) belongs to whoever calls this per tick, and invariant 5 holds by
// construction because collision reads the same particles this does.
//
// Normals get their own cases because they are the half of the kernel that has
// no obvious right answer to check against: a plane's normal is known, so a
// deformed plane is the case where "area-weighted" can be stated exactly.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/physics/soft/mesh_embedding.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    /**
     * @brief One tetrahedron and a handful of render vertices embedded in it.
     *
     * The unit tetrahedron at the origin, whose barycentric coordinates are
     * `(1 - x - y - z, x, y, z)` — so every expectation below is arithmetic and
     * not a recorded output.
     */
    struct EmbeddedFixture
    {
        std::vector<Vector3> vertices;
        std::vector<std::uint32_t> tetrahedra;
        std::vector<Cooking::SoftBodyLevelRecord> levels;
        std::vector<Cooking::SoftBodyBinding> bindings;
        Cooking::SoftBodyAssetView view;

        explicit EmbeddedFixture(bool bind_one_out_of_range = false)
        {
            vertices = {Vector3{0, 0, 0}, Vector3{1, 0, 0}, Vector3{0, 1, 0}, Vector3{0, 0, 1}};
            tetrahedra = {0, 1, 2, 3};

            Cooking::SoftBodyLevelRecord level{};
            level.first_vertex = 0;
            level.vertex_count = 4;
            level.first_tetrahedron = 0;
            level.tetrahedron_count = 1;
            levels.push_back(level);

            const Vector3 render[3] = {Vector3{Scalar(0.25), Scalar(0.25), Scalar(0.25)},
                                       Vector3{Scalar(0.5), Scalar(0.25), Scalar(0.1)},
                                       Vector3{Scalar(0.1), Scalar(0.1), Scalar(0.7)}};
            for (const Vector3& point : render)
            {
                Cooking::SoftBodyBinding binding{};
                binding.tetrahedron = 0;
                binding.weights[0] =
                    float(1.0 - double(point.x) - double(point.y) - double(point.z));
                binding.weights[1] = float(point.x);
                binding.weights[2] = float(point.y);
                binding.weights[3] = float(point.z);
                bindings.push_back(binding);
            }
            if (bind_one_out_of_range)
            {
                Cooking::SoftBodyBinding orphan{};
                orphan.tetrahedron = 99;
                orphan.weights[0] = 1.0f;
                bindings.push_back(orphan);
            }

            view.levels = levels.data();
            view.level_count = 1;
            view.vertices = vertices.data();
            view.vertex_count = 4;
            view.tetrahedra = tetrahedra.data();
            view.tetrahedron_count = 1;
            view.bindings = bindings.data();
            view.binding_count = std::uint32_t(bindings.size());
            view.valid = true;
        }

        /** @brief The four particles of the tetrahedron, at rest and translated by @p origin. */
        std::vector<RigidBodyT<Scalar>> particles(const Vector3& origin) const
        {
            std::vector<RigidBodyT<Scalar>> out(4);
            for (std::size_t i = 0; i < 4; ++i)
            {
                out[i].position = vertices[i] + origin;
                out[i].prev_position = out[i].position;
                out[i].inv_mass = Scalar(1);
            }
            return out;
        }
    };
} // namespace

TEST(Unit_MeshEmbedding, ReproducesTheSourceMeshAtRest)
{
    // §8.6 invariant 2, at the origin. The reconstruction of a rest pose must be
    // the rest pose exactly — not nearly — because every deformation downstream
    // is measured against it.
    const EmbeddedFixture fixture;
    const MeshEmbedding<Scalar> embedding =
        build_mesh_embedding<Scalar>(fixture.view, 0u, Vector3{0, 0, 0});
    ASSERT_EQ(embedding.vertex_count(), 3u);
    EXPECT_EQ(embedding.unbound_count, 0u);

    const std::vector<RigidBodyT<Scalar>> particles = fixture.particles(Vector3{0, 0, 0});
    std::vector<Vector3> deformed(embedding.vertex_count());
    embedding.deform(particles.data(), particles.size(), deformed.data());

    for (std::size_t v = 0; v < deformed.size(); ++v)
        EXPECT_LT(double(length(deformed[v] - embedding.rest_positions[v])), 1e-12)
            << "render vertex " << v;
}

TEST(Unit_MeshEmbedding, RoundTripsWhenTheBodyWasPlacedAwayFromTheOrigin)
{
    // The same invariant with the model somewhere else, because the asset's rest
    // positions are in the asset's own frame and the particles are in the world's.
    // An embedding that forgot the placement would round-trip at the origin and
    // nowhere else, which is exactly the kind of bug a single-origin test misses.
    const Vector3 origin{Scalar(12), Scalar(-4), Scalar(3)};
    const EmbeddedFixture fixture;
    const MeshEmbedding<Scalar> embedding = build_mesh_embedding<Scalar>(fixture.view, 0u, origin);

    const std::vector<RigidBodyT<Scalar>> particles = fixture.particles(origin);
    std::vector<Vector3> deformed(embedding.vertex_count());
    embedding.deform(particles.data(), particles.size(), deformed.data());

    for (std::size_t v = 0; v < deformed.size(); ++v)
        EXPECT_LT(double(length(deformed[v] - embedding.rest_positions[v])), 1e-12)
            << "render vertex " << v;
}

TEST(Unit_MeshEmbedding, FollowsTheLatticeWhenItDeforms)
{
    // Doubling the tetrahedron about the origin must double every embedded
    // vertex's offset from it — the one deformation whose answer is obvious.
    const EmbeddedFixture fixture;
    const MeshEmbedding<Scalar> embedding =
        build_mesh_embedding<Scalar>(fixture.view, 0u, Vector3{0, 0, 0});

    std::vector<RigidBodyT<Scalar>> particles = fixture.particles(Vector3{0, 0, 0});
    for (RigidBodyT<Scalar>& particle : particles)
        particle.position = particle.position * Scalar(2);

    std::vector<Vector3> deformed(embedding.vertex_count());
    embedding.deform(particles.data(), particles.size(), deformed.data());

    for (std::size_t v = 0; v < deformed.size(); ++v)
        EXPECT_LT(double(length(deformed[v] - embedding.rest_positions[v] * Scalar(2))), 1e-12)
            << "render vertex " << v;
}

TEST(Unit_MeshEmbedding, CountsAnUnboundVertexAndLeavesItAtRest)
{
    // §8.6 invariant 1. The count is the point: a cook that failed to bind
    // something must say so, and the vertex it failed on must stay put rather
    // than land at the world origin looking like a spike of geometry.
    const EmbeddedFixture fixture(true);
    const MeshEmbedding<Scalar> embedding =
        build_mesh_embedding<Scalar>(fixture.view, 0u, Vector3{0, 0, 0});

    ASSERT_EQ(embedding.vertex_count(), 4u);
    EXPECT_EQ(embedding.unbound_count, 1u);

    std::vector<RigidBodyT<Scalar>> particles = fixture.particles(Vector3{0, 0, 0});
    for (RigidBodyT<Scalar>& particle : particles)
        particle.position = particle.position + Vector3{Scalar(5), 0, 0};

    std::vector<Vector3> deformed(embedding.vertex_count());
    embedding.deform(particles.data(), particles.size(), deformed.data());

    EXPECT_LT(double(length(deformed[3] - embedding.rest_positions[3])), 1e-12)
        << "the unbound vertex moved";
}

TEST(Unit_DeformedNormals, APlaneGetsItsOwnNormal)
{
    const std::vector<Vector3> positions = {Vector3{0, 0, 0}, Vector3{1, 0, 0}, Vector3{1, 1, 0},
                                            Vector3{0, 1, 0}};
    const std::vector<std::uint32_t> indices = {0, 1, 2, 0, 2, 3};

    std::vector<Vector3> normals(positions.size());
    compute_deformed_normals(positions.data(), positions.size(), indices.data(), indices.size(),
                             normals.data());

    for (const Vector3& normal : normals)
    {
        EXPECT_NEAR(double(normal.z), 1.0, 1e-12);
        EXPECT_NEAR(double(length(normal)), 1.0, 1e-12);
    }
}

TEST(Unit_DeformedNormals, TheLargerFaceCarriesTheSharedVertex)
{
    // Area weighting, stated so it can fail. A vertex shared by a large face and
    // a sliver must end up nearly parallel to the large one; averaging normalized
    // face normals instead would put it halfway between, which is what makes a
    // deformed mesh's shading crawl along its triangulation.
    const std::vector<Vector3> positions = {
        Vector3{0, 0, 0}, Vector3{1, 0, 0}, Vector3{0, 1, 0},          // large, +z
        Vector3{Scalar(0.001), 0, Scalar(-0.001)}};                     // sliver, tilted
    const std::vector<std::uint32_t> indices = {0, 1, 2, 0, 3, 1};

    std::vector<Vector3> normals(positions.size());
    compute_deformed_normals(positions.data(), positions.size(), indices.data(), indices.size(),
                             normals.data());

    EXPECT_GT(double(normals[0].z), 0.99);
}

TEST(Unit_DeformedNormals, LeavesACancellingVertexUnnormalizedRatherThanInventingOne)
{
    // Two coincident triangles wound opposite ways: there is no normal, and
    // saying so is better than picking one.
    const std::vector<Vector3> positions = {Vector3{0, 0, 0}, Vector3{1, 0, 0}, Vector3{0, 1, 0}};
    const std::vector<std::uint32_t> indices = {0, 1, 2, 0, 2, 1};

    std::vector<Vector3> normals(positions.size());
    compute_deformed_normals(positions.data(), positions.size(), indices.data(), indices.size(),
                             normals.data());

    for (const Vector3& normal : normals)
        EXPECT_LT(double(length(normal)), 1e-12);
}

TEST(Unit_DeformedNormals, IgnoresATriangleNamingAVertexThatIsNotThere)
{
    const std::vector<Vector3> positions = {Vector3{0, 0, 0}, Vector3{1, 0, 0}, Vector3{0, 1, 0}};
    const std::vector<std::uint32_t> indices = {0, 1, 2, 0, 1, 99};

    std::vector<Vector3> normals(positions.size());
    compute_deformed_normals(positions.data(), positions.size(), indices.data(), indices.size(),
                             normals.data());

    EXPECT_NEAR(double(normals[0].z), 1.0, 1e-12);
}

// ---------------------------------------------------------------------------
// §8.6 invariant 4: "Fracture preserves binding: a duplicated simulation vertex
// inherits its parent's binding, so a crack does not tear a hole in the render
// mesh." The embedding is the half of that claim that can be tested here; the
// splitting itself is tested in test_fem_fracture.cpp.
// ---------------------------------------------------------------------------

namespace
{
    /**
     * @brief The hinge scene from the fracture tests, with an embedding over it.
     *
     * Three tetrahedra meeting at particle 0, two of which touch each other only
     * there. One render vertex is bound to each survivor, so after the crack the
     * two must be following *different* particles at the hinge — which is what
     * "the binding was preserved" means once there are two vertices to preserve
     * it onto.
     */
    FemTetrahedron tetrahedron(std::uint32_t a, std::uint32_t b, std::uint32_t c,
                               std::uint32_t d, Scalar stress)
    {
        FemTetrahedron element;
        element.vertex[0] = a;
        element.vertex[1] = b;
        element.vertex[2] = c;
        element.vertex[3] = d;
        element.von_mises_stress = stress;
        return element;
    }
} // namespace

TEST(Unit_MeshEmbedding, ADuplicatedVertexTakesItsParentsBindingWithIt)
{
    FiniteElementModel<Scalar> model;
    model.material.fracture_stress = 100.0;
    model.particles.resize(8);
    for (RigidBodyT<Scalar>& particle : model.particles)
        particle.inv_mass = Scalar(1);
    model.elements.push_back(tetrahedron(0, 1, 2, 3, 50.0));  // element 0, survives
    model.elements.push_back(tetrahedron(0, 4, 5, 6, 50.0));  // element 1, survives
    model.elements.push_back(tetrahedron(0, 1, 4, 7, 1.0e6)); // element 2, fractures

    // One render vertex per element, at its centroid.
    MeshEmbedding<Scalar> embedding;
    embedding.vertices.resize(3);
    embedding.rest_positions.assign(3, Vector3{0, 0, 0});
    for (std::uint32_t e = 0; e < 3; ++e)
    {
        EmbeddedVertex<Scalar>& binding = embedding.vertices[e];
        binding.element = e;
        for (int corner = 0; corner < 4; ++corner)
        {
            binding.particle[corner] = model.elements[e].vertex[corner];
            binding.weight[corner] = Scalar(0.25);
        }
    }

    FemFractureBudget budget;
    budget.minimum_fragment_element_count = 1;
    std::uint32_t total = 0;
    FemFractureRemap remap;
    const FemFractureReport report = apply_fem_fracture<Scalar>(model, budget, total, &remap);
    ASSERT_EQ(report.vertices_duplicated, 1u);

    embedding.follow_fracture(model, remap);

    // The two survivors' render vertices are now driven by different particles
    // where they used to share one — the crack, seen from the render mesh.
    ASSERT_EQ(model.elements.size(), 2u);
    EXPECT_EQ(embedding.vertices[0].particle[0], model.elements[0].vertex[0]);
    EXPECT_EQ(embedding.vertices[1].particle[0], model.elements[1].vertex[0]);
    EXPECT_NE(embedding.vertices[0].particle[0], embedding.vertices[1].particle[0]);

    // The weights are untouched: "inherits its parent's binding" means the same
    // place inside the same element, not a re-fit.
    for (int corner = 0; corner < 4; ++corner)
        EXPECT_NEAR(double(embedding.vertices[1].weight[corner]), 0.25, 1e-12);

    // The vertex whose element was fractured away keeps the particles it had, so
    // it goes on being drawn rather than snapping to the origin or vanishing.
    EXPECT_EQ(embedding.vertices[2].element, EmbeddedVertex<Scalar>::UNBOUND);
    EXPECT_EQ(embedding.vertices[2].particle[0], 0u);
    EXPECT_EQ(embedding.vertices[2].particle[3], 7u);
}

TEST(Unit_MeshEmbedding, TheSplitItselfMovesNothing)
{
    // The crack opens over the following ticks; it must not open on the tick it
    // is created. The copies start exactly where the original was, so every
    // render vertex reconstructs to the same point before and after.
    FiniteElementModel<Scalar> model;
    model.material.fracture_stress = 100.0;
    model.particles.resize(8);
    for (std::size_t i = 0; i < model.particles.size(); ++i)
    {
        model.particles[i].position = Vector3{Scalar(i), Scalar(i * 2), Scalar(i * 3)};
        model.particles[i].prev_position = model.particles[i].position;
        model.particles[i].inv_mass = Scalar(1);
    }
    model.elements.push_back(tetrahedron(0, 1, 2, 3, 50.0));
    model.elements.push_back(tetrahedron(0, 4, 5, 6, 50.0));
    model.elements.push_back(tetrahedron(0, 1, 4, 7, 1.0e6));

    MeshEmbedding<Scalar> embedding;
    embedding.vertices.resize(2);
    embedding.rest_positions.assign(2, Vector3{0, 0, 0});
    for (std::uint32_t e = 0; e < 2; ++e)
    {
        EmbeddedVertex<Scalar>& binding = embedding.vertices[e];
        binding.element = e;
        for (int corner = 0; corner < 4; ++corner)
        {
            binding.particle[corner] = model.elements[e].vertex[corner];
            binding.weight[corner] = Scalar(0.25);
        }
    }

    std::vector<Vector3> before(2);
    embedding.deform(model.particles.data(), model.particles.size(), before.data());

    FemFractureBudget budget;
    budget.minimum_fragment_element_count = 1;
    std::uint32_t total = 0;
    FemFractureRemap remap;
    ASSERT_EQ(apply_fem_fracture<Scalar>(model, budget, total, &remap).vertices_duplicated, 1u);
    embedding.follow_fracture(model, remap);

    std::vector<Vector3> after(2);
    embedding.deform(model.particles.data(), model.particles.size(), after.data());
    for (std::size_t v = 0; v < 2; ++v)
        EXPECT_LT(double(length(after[v] - before[v])), 1e-12) << "render vertex " << v << " jumped";
}
