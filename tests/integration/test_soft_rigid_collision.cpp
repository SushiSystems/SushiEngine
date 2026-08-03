/**************************************************************************/
/* test_soft_rigid_collision.cpp                                          */
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

// §9.6.1: a soft body's surface against a rigid body's cooked distance field.
//
// The three claims the design makes, each checked here rather than argued:
// the field answers depth and normal correctly however deep a vertex has sunk,
// a soft body settles on the surface at its own collision thickness instead of
// sinking through it or hovering, and the coupling is two-way — a rigid body
// with mass takes its share of every correction, and one without takes none.
//
// The fields are built analytically. A half-space's signed distance is linear,
// so the trilinear sampler reproduces it exactly and any error the resting test
// measures belongs to the contact solve rather than to the field; the box field
// the two-way case uses is exact on the face the contact lands on, which is the
// only part of it under test.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/physics/soft/soft_rigid_collision.hpp>

#include "tetrahedral_lattice.hpp"

using namespace SushiEngine;
using namespace SushiEngine::Physics;
using namespace SushiEngine::Harness;

namespace
{
    /** @brief A baked field and the storage behind it, since `SdfCollider` only borrows. */
    struct BakedField
    {
        std::vector<float> distances;
        SdfCollider<Scalar> field;
    };

    /** @brief The world position of voxel (x, y, z)'s centre, in the field's local frame. */
    Vector3 voxel_center(const SdfCollider<Scalar>& field, std::int32_t x, std::int32_t y,
                         std::int32_t z)
    {
        const Vector3 span = field.field_max - field.field_min;
        const Scalar step = Scalar(1) / Scalar(field.resolution);
        return Vector3{field.field_min.x + span.x * (Scalar(x) + Scalar(0.5)) * step,
                       field.field_min.y + span.y * (Scalar(y) + Scalar(0.5)) * step,
                       field.field_min.z + span.z * (Scalar(z) + Scalar(0.5)) * step};
    }

    /** @brief Fills a field's voxels from a signed-distance function of the local point. */
    template <typename Function>
    void bake(BakedField& baked, Function distance_of)
    {
        const std::int32_t resolution = baked.field.resolution;
        baked.distances.resize(std::size_t(resolution) * resolution * resolution);
        for (std::int32_t z = 0; z < resolution; ++z)
            for (std::int32_t y = 0; y < resolution; ++y)
                for (std::int32_t x = 0; x < resolution; ++x)
                    baked.distances[std::size_t(x) +
                                    std::size_t(resolution) *
                                        (std::size_t(y) +
                                         std::size_t(resolution) * std::size_t(z))] =
                        float(distance_of(voxel_center(baked.field, x, y, z)));
        baked.field.distances = baked.distances.data();
    }

    /** @brief A half-space filling everything below local Z, placed with its surface at @p surface_z. */
    BakedField make_half_space_field(Scalar surface_z, Scalar extent, std::int32_t resolution)
    {
        BakedField baked;
        baked.field.resolution = resolution;
        baked.field.field_min = Vector3{-extent, -extent, -extent};
        baked.field.field_max = Vector3{extent, extent, extent};
        baked.field.center = Vector3{0, 0, surface_z};
        baked.field.orientation = Quaternion{0, 0, 0, 1};
        bake(baked, [](const Vector3& point) { return point.z; });
        return baked;
    }

    /** @brief A solid box of half-extent @p half, centred on the field's own origin. */
    BakedField make_box_field(Scalar half, Scalar extent, std::int32_t resolution)
    {
        BakedField baked;
        baked.field.resolution = resolution;
        baked.field.field_min = Vector3{-extent, -extent, -extent};
        baked.field.field_max = Vector3{extent, extent, extent};
        baked.field.center = Vector3{0, 0, 0};
        baked.field.orientation = Quaternion{0, 0, 0, 1};
        bake(baked,
             [half](const Vector3& point)
             {
                 const Vector3 outside{std::abs(point.x) - half, std::abs(point.y) - half,
                                       std::abs(point.z) - half};
                 const Vector3 clamped{outside.x > 0 ? outside.x : Scalar(0),
                                       outside.y > 0 ? outside.y : Scalar(0),
                                       outside.z > 0 ? outside.z : Scalar(0)};
                 const Scalar deepest =
                     std::min(std::max(outside.x, std::max(outside.y, outside.z)), Scalar(0));
                 return std::sqrt(dot(clamped, clamped)) + deepest;
             });
        return baked;
    }

    /**
     * @brief Moves every particle of a model by @p offset.
     *
     * The rest state does not move with it and does not need to: `Dm^-1` is
     * built from edges, so a translated body reads as unstrained.
     */
    void translate(FiniteElementModel<Scalar>& model, const Vector3& offset)
    {
        for (RigidBodyT<Scalar>& particle : model.particles)
        {
            particle.position = particle.position + offset;
            particle.prev_position = particle.position;
        }
    }

    /** @brief The lowest Z any of the model's particles sits at. */
    Scalar lowest_particle_z(const FiniteElementModel<Scalar>& model)
    {
        Scalar lowest = model.particles.empty() ? Scalar(0) : model.particles[0].position.z;
        for (const RigidBodyT<Scalar>& particle : model.particles)
            if (particle.position.z < lowest)
                lowest = particle.position.z;
        return lowest;
    }

    SoftBodyMaterial compliant_material()
    {
        SoftBodyMaterial material;
        material.young_modulus = Scalar(1e6);
        material.poisson_ratio = Scalar(0.3);
        material.density = Scalar(1000);
        material.damping = Scalar(2.0);
        return material;
    }
} // namespace

TEST(Unit_ParticleSdfManifold, ReportsTheDistanceAndANormalIntoTheSolid)
{
    const BakedField baked = make_half_space_field(Scalar(0), Scalar(1), 32);

    const ContactManifold<Scalar> manifold =
        generate_particle_sdf_manifold(Vector3{0, 0, Scalar(0.05)}, baked.field, Scalar(0.1));

    ASSERT_EQ(manifold.point_count, 1u);
    EXPECT_NEAR(double(manifold.points[0].separation), 0.05, 1e-6);
    // The pair convention: from the particle (a) toward the field's solid (b),
    // which for a half-space open downward is straight down.
    EXPECT_NEAR(double(manifold.normal.z), -1.0, 1e-6);
    EXPECT_NEAR(double(manifold.normal.x), 0.0, 1e-6);
    EXPECT_NEAR(double(manifold.normal.y), 0.0, 1e-6);
}

TEST(Unit_ParticleSdfManifold, IsStillCorrectDeepInsideTheSolid)
{
    const BakedField baked = make_half_space_field(Scalar(0), Scalar(1), 32);

    // A vertex four tenths of a metre inside — the case a triangle-mesh
    // narrowphase has to guess the side of and a field simply knows.
    const ContactManifold<Scalar> manifold =
        generate_particle_sdf_manifold(Vector3{0, 0, Scalar(-0.4)}, baked.field, Scalar(0));

    ASSERT_EQ(manifold.point_count, 1u);
    EXPECT_NEAR(double(manifold.points[0].separation), -0.4, 1e-6);
    EXPECT_NEAR(double(manifold.normal.z), -1.0, 1e-6);
}

TEST(Unit_ParticleSdfManifold, GeneratesNothingBeyondTheContactOffset)
{
    const BakedField baked = make_half_space_field(Scalar(0), Scalar(1), 32);

    const ContactManifold<Scalar> manifold =
        generate_particle_sdf_manifold(Vector3{0, 0, Scalar(0.2)}, baked.field, Scalar(0.05));

    EXPECT_EQ(manifold.point_count, 0u);
}

TEST(Integration_SoftRigidCollision, ASoftCubeSettlesOnTheSurfaceAtItsThickness)
{
    const TetrahedralLattice lattice = build_tetrahedral_lattice(2, 2, 2, Scalar(0.05));
    FiniteElementModel<Scalar> model = build_lattice_model(lattice, compliant_material());
    ASSERT_FALSE(model.surface_vertices.empty());
    translate(model, Vector3{0, 0, Scalar(0.05)});

    model.external_acceleration = Vector3{0, 0, Scalar(-9.81)};
    model.collision.thickness = Scalar(0.01);

    const BakedField baked = make_half_space_field(Scalar(0), Scalar(1), 64);
    SoftRigidCollider<Scalar> collider;
    collider.field = baked.field;
    collider.surface_vertices = model.surface_vertices.data();
    collider.surface_vertex_count = model.surface_vertices.size();
    collider.contact_offset = Scalar(0.01);
    collider.params = make_soft_rigid_params(model.collision, PhysicsMaterial{},
                                             Scalar(2 * 9.81 / 60.0));
    model.collider = &collider;

    const Scalar dt = Scalar(1.0 / 60.0);
    for (int tick = 0; tick < 180; ++tick)
        model.step(dt, 30);

    const Scalar resting_z = lowest_particle_z(model);
    // Above the surface, by its thickness: the body neither sinks through nor
    // hovers a visible gap above what it is lying on.
    EXPECT_GT(double(resting_z), 0.0) << "the cube sank through the surface";
    EXPECT_NEAR(double(resting_z), double(model.collision.thickness), 0.004);
}

TEST(Integration_SoftRigidCollision, ARigidPartnerWithMassTakesItsShareOfTheCorrection)
{
    const TetrahedralLattice lattice = build_tetrahedral_lattice(1, 1, 1, Scalar(0.05));
    FiniteElementModel<Scalar> model = build_lattice_model(lattice, compliant_material());
    // Every particle pinned, so the whole correction has nowhere to go but the
    // rigid body — which is exactly the quantity under test.
    for (RigidBodyT<Scalar>& particle : model.particles)
        particle.inv_mass = Scalar(0);
    translate(model, Vector3{Scalar(-0.025), Scalar(-0.025), Scalar(0.05)});

    const BakedField baked = make_box_field(Scalar(0.1), Scalar(0.3), 48);

    RigidBodyT<Scalar> rigid;
    rigid.position = Vector3{0, 0, 0};
    rigid.prev_position = rigid.position;
    rigid.orientation = Quaternion{0, 0, 0, 1};
    rigid.prev_orientation = rigid.orientation;
    rigid.inv_mass = Scalar(1);
    rigid.inv_inertia = Vector3{0, 0, 0};

    SoftRigidCollider<Scalar> collider;
    collider.field = baked.field;
    collider.rigid = &rigid;
    collider.surface_vertices = model.surface_vertices.data();
    collider.surface_vertex_count = model.surface_vertices.size();
    collider.params.rest_offset = Scalar(0.01);
    model.collider = &collider;

    model.step(Scalar(1.0 / 60.0), 4);

    ASSERT_FALSE(collider.contacts().empty()) << "the surface never reached the box";
    // Pushed down and away from the pinned body sitting on its upper face.
    EXPECT_LT(double(rigid.position.z), 0.0);
}

TEST(Integration_SoftRigidCollision, AStaticRigidPartnerTakesNone)
{
    const TetrahedralLattice lattice = build_tetrahedral_lattice(1, 1, 1, Scalar(0.05));
    FiniteElementModel<Scalar> model = build_lattice_model(lattice, compliant_material());
    for (RigidBodyT<Scalar>& particle : model.particles)
        particle.inv_mass = Scalar(0);
    translate(model, Vector3{Scalar(-0.025), Scalar(-0.025), Scalar(0.05)});

    const BakedField baked = make_box_field(Scalar(0.1), Scalar(0.3), 48);

    RigidBodyT<Scalar> rigid;
    rigid.position = Vector3{0, 0, 0};
    rigid.prev_position = rigid.position;
    rigid.orientation = Quaternion{0, 0, 0, 1};
    rigid.prev_orientation = rigid.orientation;
    rigid.inv_mass = Scalar(1);
    rigid.inv_inertia = Vector3{0, 0, 0};
    // Immovable by decision rather than by mass, which is the case
    // `generalized_inverse_mass` answers with zero however heavy the body is.
    rigid.flags = BodyFlags::static_body;

    SoftRigidCollider<Scalar> collider;
    collider.field = baked.field;
    collider.rigid = &rigid;
    collider.surface_vertices = model.surface_vertices.data();
    collider.surface_vertex_count = model.surface_vertices.size();
    collider.params.rest_offset = Scalar(0.01);
    model.collider = &collider;

    model.step(Scalar(1.0 / 60.0), 4);

    ASSERT_FALSE(collider.contacts().empty());
    EXPECT_NEAR(double(rigid.position.x), 0.0, 1e-12);
    EXPECT_NEAR(double(rigid.position.y), 0.0, 1e-12);
    EXPECT_NEAR(double(rigid.position.z), 0.0, 1e-12);
}

TEST(Integration_SoftRigidCollision, ContactsAreOrderedByParticleIndex)
{
    const TetrahedralLattice lattice = build_tetrahedral_lattice(2, 2, 2, Scalar(0.05));
    FiniteElementModel<Scalar> model = build_lattice_model(lattice, compliant_material());
    // Buried, so every surface particle is in contact and the ordering claim is
    // tested against the whole set rather than against whichever two touched.
    translate(model, Vector3{0, 0, Scalar(-0.2)});

    const BakedField baked = make_half_space_field(Scalar(0), Scalar(1), 64);
    SoftRigidCollider<Scalar> collider;
    collider.field = baked.field;
    collider.surface_vertices = model.surface_vertices.data();
    collider.surface_vertex_count = model.surface_vertices.size();

    collider.generate_contacts(model.particles.data(), model.particles.size(), Scalar(1.0 / 60.0));

    ASSERT_EQ(collider.contacts().size(), model.surface_vertices.size());
    for (std::size_t i = 1; i < collider.contacts().size(); ++i)
        EXPECT_LT(collider.contacts()[i - 1].particle, collider.contacts()[i].particle);
}
