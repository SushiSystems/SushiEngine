/**************************************************************************/
/* test_beam_constraint.cpp                                               */
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

// Unit_BeamConstraint: P7-A and P7-B — §11.1's beam, and §11.2's claim that its
// numbers are derived from a material rather than typed in. The load recovery is
// checked against Hooke's law rather than against itself: a compliant beam's
// reported axial force must be `E·A·Δ/L`, which is the whole argument for deriving
// the compliance from a Young's modulus at all. Plasticity, breakage and the
// rate-based damping are checked against the three properties each was written to
// have — no creep below threshold, no overshoot of the strain ceiling, and a damping
// that is a rate rather than a per-substep fraction.

#include <cmath>

#include <gtest/gtest.h>

#include <SushiEngine/physics/constraints/beam_projection.hpp>
#include <SushiEngine/physics/soft/beam_properties.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    /** @brief Node 0 pinned at the origin, node 1 free at (x, 0, 0). */
    void place(RigidBody nodes[2], Scalar x)
    {
        nodes[0] = RigidBody{};
        nodes[0].position = Vector3{0.0, 0.0, 0.0};
        nodes[0].inv_mass = 0.0;
        nodes[1] = RigidBody{};
        nodes[1].position = Vector3{x, 0.0, 0.0};
        nodes[1].inv_mass = 1.0;
    }

    /** @brief A rigid beam of unit rest length between node 0 and node 1. */
    BeamConstraint unit_beam()
    {
        BeamConstraint beam;
        beam.a = 0;
        beam.b = 1;
        beam.rest_length = 1.0;
        beam.initial_rest_length = 1.0;
        return beam;
    }

    /** @brief The material the vehicle panels of §11.2 are made of, in a compliant range. */
    SoftBodyMaterial compliant_material()
    {
        SoftBodyMaterial material;
        material.young_modulus = 1.0e6;
        material.poisson_ratio = 0.3;
        material.density = 1000.0;
        material.damping = 0.0;
        material.yield_stress = 1.0e30;
        material.fracture_stress = 1.0e30;
        return material;
    }
} // namespace

// A stretched beam pulls its free node back toward the rest length, and reports the
// pull as tension. Both halves matter: a sign convention nobody checks is a sign
// convention that is wrong half the time.
TEST(Unit_BeamConstraint, StretchedBeamPullsBackAndReportsTension)
{
    RigidBody nodes[2];
    place(nodes, 1.2);
    BeamConstraint beam = unit_beam();

    BeamProjection{}(beam, nodes, 1.0 / 240.0, true);

    EXPECT_NEAR(nodes[1].position.x, 1.0, 1e-12);
    EXPECT_GT(beam.axial_force, 0.0);
    EXPECT_EQ(beam.force_samples, 1u);
}

// The mirror case, so "positive is tension" cannot be satisfied by a projection that
// simply reports a magnitude.
TEST(Unit_BeamConstraint, CompressedBeamPushesBackAndReportsCompression)
{
    RigidBody nodes[2];
    place(nodes, 0.8);
    BeamConstraint beam = unit_beam();

    BeamProjection{}(beam, nodes, 1.0 / 240.0, true);

    EXPECT_NEAR(nodes[1].position.x, 1.0, 1e-12);
    EXPECT_LT(beam.axial_force, 0.0);
}

// P7-B's whole claim, measured. A beam whose compliance came from a Young's modulus
// and a cross-section must report the load an axially loaded bar of that material
// would carry: `F = E·A·Δ/L`. Nothing else in this phase pins the derivation to
// physics rather than to itself.
TEST(Unit_BeamConstraint, DerivedComplianceReportsHookeanLoad)
{
    const SoftBodyMaterial material = compliant_material();
    const Scalar length_rest = 1.0;
    const Scalar area = 1.0e-4;
    const Scalar stretch = 1.0e-3;

    BeamConstraint beam;
    beam.a = 0;
    beam.b = 1;
    apply_beam_material(beam, material, length_rest, area);

    RigidBody nodes[2];
    place(nodes, length_rest + stretch);
    BeamProjection{}(beam, nodes, 1.0 / 480.0, true);

    const Scalar expected = material.young_modulus * area * stretch / length_rest;
    EXPECT_NEAR(beam.axial_force, expected, expected * 1.0e-2);
}

// The sentinel `1e30` a material carries for "does not yield" must not be converted
// into a threshold of `1e30 * area`, which is a finite force a big enough impact
// reaches. Zero is what the beam reads as never.
TEST(Unit_BeamConstraint, NonYieldingMaterialGivesNoThresholds)
{
    BeamConstraint beam;
    apply_beam_material(beam, compliant_material(), 1.0, 1.0e-4);

    EXPECT_EQ(beam.deform_force, 0.0);
    EXPECT_EQ(beam.break_force, 0.0);
    EXPECT_FALSE(beam_should_break(beam));
}

// Sheet steel does yield and does fracture, and the two thresholds are the material's
// stresses times the area — the conversion this file exists to pin down.
TEST(Unit_BeamConstraint, SteelThresholdsAreStressTimesArea)
{
    const SoftBodyMaterial steel = sheet_steel_material<Scalar>();
    const Scalar area = 2.0e-4;

    BeamConstraint beam;
    apply_beam_material(beam, steel, 0.5, area);

    EXPECT_NEAR(beam.deform_force, steel.yield_stress * area, 1e-6);
    EXPECT_NEAR(beam.break_force, steel.fracture_stress * area, 1e-6);
    EXPECT_NEAR(beam.rest_length, 0.5, 1e-12);
    EXPECT_NEAR(beam.initial_rest_length, 0.5, 1e-12);
    EXPECT_EQ(beam.accumulated_plastic_strain, 0.0);
}

// Below the deform threshold a beam is perfectly elastic: whatever it has been
// through, it rests where it was cooked.
TEST(Unit_BeamConstraint, BelowThresholdNothingIsPermanent)
{
    RigidBody nodes[2];
    place(nodes, 1.05);

    BeamConstraint beam = unit_beam();
    beam.deform_force = 1.0e6;
    beam.plastic_creep = 0.5;
    beam.maximum_plastic_strain = 0.2;
    beam.peak_force = 1.0e3;
    beam.force_samples = 8;

    apply_beam_plasticity(nodes, beam);

    EXPECT_EQ(beam.rest_length, 1.0);
    EXPECT_EQ(beam.accumulated_plastic_strain, 0.0);
}

// Past the threshold the rest length walks a `plastic_creep` fraction of the way to
// where the beam actually is — the axial case of §9.4, and the dent that stays.
TEST(Unit_BeamConstraint, PastThresholdTheRestLengthCreeps)
{
    RigidBody nodes[2];
    place(nodes, 1.10);

    BeamConstraint beam = unit_beam();
    beam.deform_force = 1.0e3;
    beam.plastic_creep = 0.5;
    beam.maximum_plastic_strain = 0.2;
    beam.peak_force = 5.0e3;
    beam.force_samples = 8;

    apply_beam_plasticity(nodes, beam);

    EXPECT_NEAR(beam.rest_length, 1.05, 1e-12);
    EXPECT_NEAR(beam.accumulated_plastic_strain, 0.05, 1e-12);
    EXPECT_NEAR(beam_plastic_strain(beam), 0.05, 1e-12);
}

// The ceiling is a ceiling: a beam held past yield for many ticks converges to
// `maximum_plastic_strain` and stops, rather than creeping past it and then being
// clamped back — which would let the rest length overshoot for one tick.
TEST(Unit_BeamConstraint, PlasticStrainConvergesToItsCeilingWithoutOvershooting)
{
    BeamConstraint beam = unit_beam();
    beam.deform_force = 1.0e3;
    beam.plastic_creep = 0.9;
    beam.maximum_plastic_strain = 0.05;
    beam.peak_force = 5.0e4;
    beam.force_samples = 8;

    for (int tick = 0; tick < 64; ++tick)
    {
        RigidBody nodes[2];
        place(nodes, 1.5);
        apply_beam_plasticity(nodes, beam);
        EXPECT_LE(beam.accumulated_plastic_strain, 0.05 + 1e-12);
    }

    EXPECT_NEAR(beam.accumulated_plastic_strain, 0.05, 1e-9);
    // Every tick's creep was toward the same stretched pose, so the permanent set is
    // the accumulator's own ceiling and not some larger number it passed through.
    EXPECT_NEAR(beam_plastic_strain(beam), 0.05, 1e-9);
}

// A beam that has never been stepped has not failed, however small its threshold.
// The guard exists because a structure is inspected as soon as it is built.
TEST(Unit_BeamConstraint, AnUnsteppedBeamHasNotBroken)
{
    BeamConstraint beam = unit_beam();
    beam.break_force = 1.0e-6;
    EXPECT_FALSE(beam_should_break(beam));
}

// Breakage is measured against the peak substep load. The tick's mean would be near
// zero for an impact that rebounds, which is the load that actually tears a beam out.
TEST(Unit_BeamConstraint, PeakLoadDecidesBreakageNotTheMean)
{
    BeamConstraint beam = unit_beam();
    beam.break_force = 1.0e4;
    beam.force_samples = 2;
    beam.force_sum = 0.0;      // +1.6e6 then -1.6e6: a mean of nothing
    beam.peak_force = 1.6e6;

    EXPECT_NEAR(beam_force(beam), 0.0, 1e-12);
    EXPECT_TRUE(beam_should_break(beam));
}

// A broken beam is skipped rather than removed mid-solve (§6.6), so the projection
// must leave both its nodes exactly where they were.
TEST(Unit_BeamConstraint, ABrokenBeamCorrectsNothing)
{
    RigidBody nodes[2];
    place(nodes, 1.4);

    BeamConstraint beam = unit_beam();
    beam.flags |= BeamFlags::broken;

    BeamProjection{}(beam, nodes, 1.0 / 240.0, true);

    EXPECT_EQ(nodes[1].position.x, 1.4);
    EXPECT_EQ(beam.axial_force, 0.0);
}

// A sleeping pair keeps its last live load rather than reporting zero, which is the
// rule `JointProjectionT` applies for the same reason: a settled structure that reads
// as carrying nothing is a wrong answer, not a missing one.
TEST(Unit_BeamConstraint, ASleepingPairKeepsItsLastLoad)
{
    RigidBody nodes[2];
    place(nodes, 1.2);
    nodes[1].flags |= BodyFlags::sleeping;

    BeamConstraint beam = unit_beam();
    beam.axial_force = 344.0;
    beam.peak_force = 344.0;
    beam.force_sum = 344.0;
    beam.force_samples = 1;

    BeamProjection{}(beam, nodes, 1.0 / 240.0, true);

    EXPECT_EQ(beam.axial_force, 344.0);
    EXPECT_EQ(beam.force_samples, 1u);
    EXPECT_EQ(nodes[1].position.x, 1.2);
}

// Damping removes relative velocity along the beam and nothing else. A beam that
// damped the full relative velocity would make a network resist shear it was never
// given the stiffness to resist.
TEST(Unit_BeamConstraint, DampingTouchesOnlyTheAxialComponent)
{
    RigidBody nodes[2];
    place(nodes, 1.0);
    nodes[1].velocity = Vector3{2.0, 3.0, 0.0};

    BeamConstraint beam = unit_beam();
    beam.damping = 30.0;

    BeamVelocityProjection{}(beam, nodes, 1.0 / 240.0);

    EXPECT_LT(nodes[1].velocity.x, 2.0);
    EXPECT_EQ(nodes[1].velocity.y, 3.0);
}

// `damping` is a rate in inverse seconds, so the same wall-clock interval must remove
// very nearly the same velocity however the substep schedule divided it. This is the
// property that keeps a vehicle from feeling different because something else in the
// scene raised the substep count (§6.2).
TEST(Unit_BeamConstraint, DampingIsARateNotAPerSubstepFraction)
{
    const Scalar interval = 1.0 / 240.0;
    const Scalar rate = 10.0;

    RigidBody coarse[2];
    place(coarse, 1.0);
    coarse[1].velocity = Vector3{1.0, 0.0, 0.0};
    BeamConstraint beam = unit_beam();
    beam.damping = rate;
    BeamVelocityProjection{}(beam, coarse, interval);

    RigidBody fine[2];
    place(fine, 1.0);
    fine[1].velocity = Vector3{1.0, 0.0, 0.0};
    BeamVelocityProjection{}(beam, fine, interval * 0.5);
    BeamVelocityProjection{}(beam, fine, interval * 0.5);

    EXPECT_NEAR(fine[1].velocity.x, coarse[1].velocity.x, 1.0e-3);
}

// The tributary rule is a conservation statement: the network's total `Σ A·L` is the
// body's volume, so the beams are made of the body's material and no more of it.
TEST(Unit_BeamConstraint, TributaryAreasConserveTheBodyVolume)
{
    const Scalar volume = 0.75;
    const std::size_t beams = 12;
    const Scalar lengths[3] = {0.1, 0.1 * std::sqrt(2.0), 0.2};

    Scalar total = 0.0;
    for (std::size_t i = 0; i < beams; ++i)
    {
        const Scalar rest = lengths[i % 3];
        total += beam_tributary_area(volume, beams, rest) * rest;
    }
    EXPECT_NEAR(total, volume, 1e-12);
}

// Degenerate inputs come back rigid and unbreakable rather than infinite, so nothing
// downstream has to test for a not-a-number that a division would have produced.
TEST(Unit_BeamConstraint, DegenerateGeometryYieldsNoStiffnessRatherThanInfinity)
{
    const BeamProperties<Scalar> zero_area =
        beam_properties_from_material(sheet_steel_material<Scalar>(), 1.0, 0.0);
    EXPECT_EQ(zero_area.compliance, 0.0);
    EXPECT_EQ(zero_area.deform_force, 0.0);
    EXPECT_EQ(zero_area.break_force, 0.0);

    EXPECT_EQ(beam_tributary_area<Scalar>(0.0, 4, 1.0), 0.0);
    EXPECT_EQ(beam_tributary_area<Scalar>(1.0, 0, 1.0), 0.0);
    EXPECT_EQ(beam_tributary_area<Scalar>(1.0, 4, 0.0), 0.0);
}
