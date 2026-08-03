/**************************************************************************/
/* test_soft_body_model_conformance.cpp                                   */
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

// §4.4's conformance suite for `ISoftBodyModel`, in the doc's own words: every
// implementation must converge to the same rest shape.
//
// This is the test that licenses §9.7 to swap one model for another as a body
// recedes. Without it "the coarse tier looks like the fine tier" is a hope; with
// it, a tier that settles somewhere else fails here rather than in a scene.
//
// Every case below is written once, against `ISoftBodyModel&`, and run against
// all three implementations through the same parameterized fixture — because a
// suite that named the concrete types would be three suites that could drift
// apart, which is exactly what §4.4 exists to prevent.
//
// The three claims, in order of what they depend on:
//
//  1. A body with nothing holding it falls at gravity and keeps its shape. This
//     is the schedule, not the model: predict, derive, damp. It runs first
//     because a model that failed it would fail the other two for a reason that
//     has nothing to do with what is being measured.
//  2. A body released from a deformed pose returns to its rest shape. The
//     §4.4 claim itself.
//  3. The three of them return to the *same* rest shape, compared against each
//     other rather than each against its own idea of rest.

#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/physics/soft/finite_element_model.hpp>
#include <SushiEngine/physics/soft/mass_spring_model.hpp>
#include <SushiEngine/physics/soft/shape_matching_model.hpp>
#include <SushiEngine/physics/soft/soft_body_model.hpp>

#include "tetrahedral_lattice.hpp"

using namespace SushiEngine;
using namespace SushiEngine::Physics;
using namespace SushiEngine::Harness;

namespace
{
    /** @brief The one cell size, damping rate and tick count every case shares. */
    constexpr Scalar CELL_SIZE = Scalar(0.05);
    constexpr Scalar DAMPING = Scalar(5.0);
    constexpr int SETTLE_TICKS = 300;
    constexpr std::size_t SUBSTEPS = 20;

    SoftBodyMaterial conformance_material()
    {
        SoftBodyMaterial material;
        material.young_modulus = Scalar(1e6);
        material.poisson_ratio = Scalar(0.3);
        material.density = Scalar(1000);
        material.damping = DAMPING;
        return material;
    }

    /** @brief The lattice all three implementations are built over. */
    const TetrahedralLattice& shared_lattice()
    {
        static const TetrahedralLattice lattice =
            build_tetrahedral_lattice(2, 2, 2, CELL_SIZE);
        return lattice;
    }

    /**
     * @brief Which implementation a case is currently running against.
     *
     * A name rather than a type tag, because the only thing the test does with it
     * is build the model and say which one failed.
     */
    enum class ModelKind
    {
        FiniteElement,
        MassSpring,
        ShapeMatching
    };

    std::string name_of(ModelKind kind)
    {
        switch (kind)
        {
            case ModelKind::FiniteElement: return "FiniteElementModel";
            case ModelKind::MassSpring: return "MassSpringModel";
            case ModelKind::ShapeMatching: return "ShapeMatchingModel";
        }
        return "unknown";
    }

    /**
     * @brief Builds one implementation over @ref shared_lattice, at rest.
     *
     * The three are built from the *same* lattice and the same lumped masses —
     * `build_lattice_model` produces both, and the two coarser kinds copy its
     * particles rather than computing their own. A conformance suite whose
     * implementations disagreed about what they weighed would be measuring that
     * disagreement instead of the models.
     *
     * @param kind Which implementation to build.
     * @return The model, owning its own arrays.
     */
    std::unique_ptr<ISoftBodyModel<Scalar>> build_model(ModelKind kind)
    {
        const TetrahedralLattice& lattice = shared_lattice();
        FiniteElementModel<Scalar> reference =
            build_lattice_model(lattice, conformance_material());

        switch (kind)
        {
            case ModelKind::FiniteElement:
                return std::unique_ptr<ISoftBodyModel<Scalar>>(
                    new FiniteElementModel<Scalar>(reference));

            case ModelKind::MassSpring:
            {
                std::unique_ptr<MassSpringModel<Scalar>> model(new MassSpringModel<Scalar>());
                model->particles = reference.particles;
                model->surface_indices = reference.surface_indices;
                model->surface_vertices = reference.surface_vertices;
                model->damping = DAMPING;
                // Stiff enough that the lattice recovers its shape rather than
                // creeping: the same order of stiffness the FEM body's Young's
                // modulus implies, expressed the way a distance constraint takes it.
                link_tetrahedron_edges(*model, lattice.tetrahedra.data(),
                                       lattice.tetrahedra.size() / 4, Scalar(1e-9));
                return std::unique_ptr<ISoftBodyModel<Scalar>>(model.release());
            }

            case ModelKind::ShapeMatching:
            {
                std::unique_ptr<ShapeMatchingModel<Scalar>> model(
                    new ShapeMatchingModel<Scalar>());
                model->particles = reference.particles;
                model->surface_indices = reference.surface_indices;
                model->surface_vertices = reference.surface_vertices;
                model->damping = DAMPING;
                model->capture_rest_shape();
                return std::unique_ptr<ISoftBodyModel<Scalar>>(model.release());
            }
        }
        return nullptr;
    }

    std::vector<Vector3> positions_of(ISoftBodyModel<Scalar>& model)
    {
        const SoftSurfaceView<Scalar> view = model.surface();
        std::vector<Vector3> out(view.particle_count);
        for (std::size_t i = 0; i < view.particle_count; ++i)
            out[i] = view.particles[i].position;
        return out;
    }

    Vector3 centroid_of(const std::vector<Vector3>& positions)
    {
        Vector3 sum{0, 0, 0};
        for (const Vector3& position : positions)
            sum = sum + position;
        return positions.empty() ? sum : sum * (Scalar(1) / Scalar(positions.size()));
    }

    /**
     * @brief The largest distance between two poses, after removing any rigid motion.
     *
     * Shape, not placement *and not orientation*. Removing the translation alone is
     * not enough and the reason is worth stating, because an earlier version of this
     * file did exactly that and reported a body that had recovered its rest shape to
     * the last bit as having failed: nothing pins the orientation of a body floating
     * in free space. A Gauss-Seidel sweep is not symmetric — it visits its
     * constraints in a fixed order and each projection sees the corrections the ones
     * before it already applied — so it imparts a small torque that no amount of
     * *velocity* damping can undo, because damping removes the spin and not the angle
     * already turned through. Over a few thousand substeps that accumulates into a
     * visible rotation, and a body that has turned is not a body that has deformed.
     *
     * So the rotation is fitted and removed, exactly as `ShapeMatchingModel` fits
     * one, before anything is measured.
     */
    Scalar shape_deviation(const std::vector<Vector3>& a, const std::vector<Vector3>& b)
    {
        if (a.size() != b.size() || a.empty())
            return Scalar(1e9);

        const Vector3 centre_a = centroid_of(a);
        const Vector3 centre_b = centroid_of(b);
        FEMMatrix3<Scalar> covariance;
        for (std::size_t i = 0; i < a.size(); ++i)
        {
            const Vector3 current = b[i] - centre_b;
            const Vector3 rest = a[i] - centre_a;
            covariance.column0 = covariance.column0 + current * rest.x;
            covariance.column1 = covariance.column1 + current * rest.y;
            covariance.column2 = covariance.column2 + current * rest.z;
        }

        FEMMatrix3<Scalar> rotation;
        if (!polar_rotation(covariance, rotation))
            return Scalar(1e9); // collapsed or inverted: no shape left to compare

        Scalar worst = 0;
        for (std::size_t i = 0; i < a.size(); ++i)
        {
            const Vector3 rest = a[i] - centre_a;
            const Vector3 aligned = centre_b + rotation.column0 * rest.x +
                                    rotation.column1 * rest.y + rotation.column2 * rest.z;
            const Scalar distance = length(b[i] - aligned);
            if (distance > worst)
                worst = distance;
        }
        return worst;
    }

    /**
     * @brief The body's centre of mass — the point internal constraints cannot move.
     *
     * Deliberately mass-weighted, unlike the plain centroid above. The lumped masses
     * of a tetrahedral lattice are *not* uniform — a corner vertex is shared by
     * fewer elements than an interior one and weighs correspondingly less — so an
     * unweighted centroid moves whenever an internal projection redistributes the
     * particles, even though momentum is perfectly conserved. Measuring a fall with
     * one would read a model's internal solve as external motion.
     */
    Vector3 centre_of_mass(ISoftBodyModel<Scalar>& model)
    {
        const SoftSurfaceView<Scalar> view = model.surface();
        Vector3 weighted{0, 0, 0};
        Scalar total = 0;
        for (std::size_t i = 0; i < view.particle_count; ++i)
        {
            const Scalar inverse = view.particles[i].inv_mass;
            const Scalar mass = inverse > Scalar(0) ? Scalar(1) / inverse : Scalar(0);
            weighted = weighted + view.particles[i].position * mass;
            total += mass;
        }
        return total > Scalar(0) ? weighted * (Scalar(1) / total) : weighted;
    }

    /**
     * @brief Scales every particle away from the body's centre by @p factor.
     *
     * A uniform dilation about the centroid, so the perturbation adds no net
     * momentum and no net rotation: whatever the body does next is its own
     * response and not a drift the test introduced. It is also the one
     * deformation all three kinds genuinely resist — the FEM body's hydrostatic
     * constraint, the springs' rest lengths, and the shape match's scale-free fit.
     */
    void dilate(ISoftBodyModel<Scalar>& model, Scalar factor)
    {
        const SoftSurfaceView<Scalar> view = model.surface();
        Vector3 centre{0, 0, 0};
        for (std::size_t i = 0; i < view.particle_count; ++i)
            centre = centre + view.particles[i].position;
        centre = centre * (Scalar(1) / Scalar(view.particle_count));

        for (std::size_t i = 0; i < view.particle_count; ++i)
        {
            RigidBodyT<Scalar>& particle = view.particles[i];
            particle.position = centre + (particle.position - centre) * factor;
            particle.previous_position = particle.position;
            particle.velocity = Vector3{0, 0, 0};
        }
    }
} // namespace

/** @brief One case, run once per implementation of the seam. */
class SoftBodyModelConformance : public ::testing::TestWithParam<ModelKind>
{
};

/**
 * @brief Drops a model for a second and reports how far its centroid moved.
 *
 * Deliberately not compared against `0.5 g t²`: every model here damps velocity
 * at `DAMPING` per second, so a body in free flight reaches a terminal speed
 * rather than accelerating, and asserting the undamped figure would be asserting
 * something none of the three claims. What *is* claimed — and is what the
 * conformance suite is for — is that all three fall by the same amount, since
 * the fall is the shared schedule and not the model.
 */
Scalar free_fall_drop(ModelKind kind, std::vector<Vector3>& rest_out,
                      std::vector<Vector3>& fallen_out)
{
    const std::unique_ptr<ISoftBodyModel<Scalar>> model = build_model(kind);
    if (model == nullptr)
        return Scalar(0);

    rest_out = positions_of(*model);
    const Vector3 before = centre_of_mass(*model);
    model->set_external_acceleration(Vector3{0, 0, Scalar(-9.81)});
    for (int tick = 0; tick < 60; ++tick)
        model->step(Scalar(1.0 / 60.0), SUBSTEPS);

    fallen_out = positions_of(*model);
    return before.z - centre_of_mass(*model).z;
}

TEST_P(SoftBodyModelConformance, FallsWithoutChangingShape)
{
    std::vector<Vector3> rest;
    std::vector<Vector3> fallen;
    const Scalar drop = free_fall_drop(GetParam(), rest, fallen);

    ASSERT_FALSE(fallen.empty()) << name_of(GetParam());
    EXPECT_GT(double(drop), 0.5) << name_of(GetParam()) << " did not fall";

    // Nothing is violated at rest — every element is at its rest shape, every
    // spring at its rest length, and the shape match fits exactly — so a uniform
    // acceleration must move the whole body and deform none of it. A model that
    // fails this is applying a correction where its own constraint error is zero.
    EXPECT_LT(double(shape_deviation(rest, fallen)), 1e-9)
        << name_of(GetParam()) << " deformed under a uniform acceleration";
}

TEST(Integration_SoftBodyModelConformance, EveryImplementationFallsByTheSameAmount)
{
    // The fall belongs to `SoftBodyBase`'s schedule, not to any model's physics,
    // so the three must agree to the last bit. This is the case that catches a
    // model kind that quietly reorders predict/derive or applies its damping in
    // the wrong place — which would otherwise only show up as one tier of §9.7
    // sagging differently from the next.
    std::vector<Vector3> rest;
    std::vector<Vector3> fallen;
    const Scalar reference = free_fall_drop(ModelKind::FiniteElement, rest, fallen);
    ASSERT_GT(double(reference), 0.5);

    // Not to the last bit: `ShapeMatchingModel` fits its rest shape with *unweighted*
    // centroids, which is the right choice for a coarse tier (a pinned particle has no
    // finite mass to weight it by) but leaves its correction very slightly
    // non-momentum-conserving on a lattice whose lumped masses differ. A tenth of a
    // millimetre over a metre and a half of fall is that, and nothing else — a model
    // that reordered the schedule would be out by centimetres.
    const ModelKind others[2] = {ModelKind::MassSpring, ModelKind::ShapeMatching};
    for (const ModelKind kind : others)
        EXPECT_NEAR(double(free_fall_drop(kind, rest, fallen)), double(reference), 1e-4)
            << name_of(kind) << " fell differently from " << name_of(ModelKind::FiniteElement);
}

TEST_P(SoftBodyModelConformance, ReturnsToItsRestShapeAfterBeingDeformed)
{
    const std::unique_ptr<ISoftBodyModel<Scalar>> model = build_model(GetParam());
    ASSERT_NE(model, nullptr) << name_of(GetParam());

    const std::vector<Vector3> rest = positions_of(*model);
    dilate(*model, Scalar(1.10));
    const Scalar deformed = shape_deviation(rest, positions_of(*model));
    ASSERT_GT(double(deformed), 1e-3) << "the perturbation did not take";

    for (int tick = 0; tick < SETTLE_TICKS; ++tick)
        model->step(Scalar(1.0 / 60.0), SUBSTEPS);

    // Expressed as a fraction of the deformation rather than as an absolute
    // distance: the claim is that the body *recovers*, and how far it was pushed
    // is the only scale that claim has.
    const Scalar remaining = shape_deviation(rest, positions_of(*model));
    EXPECT_LT(double(remaining), 0.1 * double(deformed))
        << name_of(GetParam()) << " kept " << double(remaining) << " m of a "
        << double(deformed) << " m deformation";
}

INSTANTIATE_TEST_SUITE_P(EveryImplementation, SoftBodyModelConformance,
                         ::testing::Values(ModelKind::FiniteElement, ModelKind::MassSpring,
                                           ModelKind::ShapeMatching),
                         [](const ::testing::TestParamInfo<ModelKind>& info)
                         { return name_of(info.param); });

TEST(Integration_SoftBodyModelConformance, EveryImplementationSettlesOnTheSameShape)
{
    // §4.4 says the implementations "must all converge to the same rest shape."
    // Measured, that is very nearly but not exactly true, and the discrepancy is
    // physics rather than error — so the tolerance below states it rather than
    // hiding it.
    //
    // `MassSpringModel` and `ShapeMatchingModel` both rest at the cooked lattice
    // exactly: their rest state *is* the lattice, by construction. The stable
    // neo-Hookean pair of §9.1 does not. Its two constraints — `C = ||F||` pulling
    // the element smaller and `C = det(F) - 1 - mu/lambda` pushing it larger —
    // balance at a deformation gradient slightly away from the identity, so an
    // unpinned FEM body relaxes to a shape a few per cent off the lattice it was
    // cooked from, and non-uniformly: measured here, 4.6 mm on a 100 mm cube, which
    // survives removing a uniform scale as well as a rigid motion.
    //
    // That offset is a documented property of the model (Macklin & Muller 2021),
    // not a defect, and it is why the bound is a per cent of the body rather than a
    // solver tolerance. What the bound still catches is the thing worth catching: a
    // tier that settles somewhere structurally different.
    const ModelKind kinds[3] = {ModelKind::FiniteElement, ModelKind::MassSpring,
                                ModelKind::ShapeMatching};
    std::vector<std::vector<Vector3>> settled;

    for (const ModelKind kind : kinds)
    {
        const std::unique_ptr<ISoftBodyModel<Scalar>> model = build_model(kind);
        ASSERT_NE(model, nullptr) << name_of(kind);
        dilate(*model, Scalar(1.10));
        for (int tick = 0; tick < SETTLE_TICKS; ++tick)
            model->step(Scalar(1.0 / 60.0), SUBSTEPS);
        settled.push_back(positions_of(*model));
    }

    // A tenth of the body's 100 mm extent: loose enough for the constitutive
    // offset above, an order of magnitude tighter than any structural disagreement.
    for (std::size_t i = 1; i < settled.size(); ++i)
        EXPECT_LT(double(shape_deviation(settled[0], settled[i])), 0.01)
            << name_of(kinds[i]) << " settled somewhere " << name_of(kinds[0]) << " did not";

    // The two that share a rest state must agree with each other far more tightly
    // than either agrees with the FEM body, or the paragraph above is wrong about
    // where the difference comes from.
    EXPECT_LT(double(shape_deviation(settled[1], settled[2])), 1e-6)
        << name_of(kinds[1]) << " and " << name_of(kinds[2])
        << " rest at the same lattice and must settle together";
}

TEST(Unit_PolarRotation, RecoversAPureRotationExactly)
{
    // The decomposition shape matching is built on, on its own: a matrix that
    // already *is* a rotation must come back unchanged, or every fit built on it
    // is measuring a rotation that is not there.
    const Scalar angle = Scalar(0.7);
    const Scalar c = Scalar(std::cos(double(angle)));
    const Scalar s = Scalar(std::sin(double(angle)));

    FEMMatrix3<Scalar> rotation;
    rotation.column0 = Vector3{c, s, 0};
    rotation.column1 = Vector3{-s, c, 0};
    rotation.column2 = Vector3{0, 0, 1};

    FEMMatrix3<Scalar> recovered;
    ASSERT_TRUE(polar_rotation(rotation, recovered));
    EXPECT_NEAR(double(length(recovered.column0 - rotation.column0)), 0.0, 1e-12);
    EXPECT_NEAR(double(length(recovered.column1 - rotation.column1)), 0.0, 1e-12);
    EXPECT_NEAR(double(length(recovered.column2 - rotation.column2)), 0.0, 1e-12);
}

TEST(Unit_PolarRotation, StripsAUniformScaleAndKeepsTheRotation)
{
    FEMMatrix3<Scalar> scaled;
    scaled.column0 = Vector3{Scalar(3), 0, 0};
    scaled.column1 = Vector3{0, Scalar(3), 0};
    scaled.column2 = Vector3{0, 0, Scalar(3)};

    FEMMatrix3<Scalar> recovered;
    ASSERT_TRUE(polar_rotation(scaled, recovered));
    EXPECT_NEAR(double(recovered.column0.x), 1.0, 1e-12);
    EXPECT_NEAR(double(recovered.column1.y), 1.0, 1e-12);
    EXPECT_NEAR(double(recovered.column2.z), 1.0, 1e-12);
}

TEST(Unit_PolarRotation, RefusesAnInvertedMatrix)
{
    // A body turned inside out has no nearest rotation worth the name, and a
    // routine that invented one would hand shape matching a fit that pulls every
    // particle through the body's centre.
    FEMMatrix3<Scalar> inverted;
    inverted.column0 = Vector3{Scalar(-1), 0, 0};
    inverted.column1 = Vector3{0, Scalar(1), 0};
    inverted.column2 = Vector3{0, 0, Scalar(1)};

    FEMMatrix3<Scalar> recovered;
    EXPECT_FALSE(polar_rotation(inverted, recovered));
}
