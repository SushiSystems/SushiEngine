/**************************************************************************/
/* test_soft_body_lod.cpp                                                 */
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

// §9.7: which tier simulates a body, and what happens to its pose when that
// changes.
//
// The three pieces are tested separately because they fail separately. The
// selector is a pure function and is tested as one — including the case
// hysteresis exists for, a body sitting exactly on a boundary, which a test that
// only walked the coverage monotonically down would never reach. The transfer is
// tested against a two-level asset small enough to state the right answer by
// hand. Only then is the chain tested, and what it is asked is the one thing
// §9.7 promises out loud: that a body does not pop.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/physics/soft/rigid_soft_body_model.hpp>
#include <SushiEngine/physics/soft/shape_matching_model.hpp>
#include <SushiEngine/physics/soft/soft_body_lod.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    /**
     * @brief A two-level asset small enough to check by hand.
     *
     * Level one is the unit tetrahedron at the origin; level zero is four points
     * inside it. The mappings are the exact barycentric coordinates of those four
     * points, which for this tetrahedron are `(1 - x - y - z, x, y, z)` — so
     * every expected value below is arithmetic rather than a recorded output.
     *
     * Built as a view over its own arrays rather than through the blob writer:
     * what is under test is the transfer, and routing it through a serializer
     * would mean a format change could fail these cases for a reason that has
     * nothing to do with levels of detail.
     */
    struct TwoLevelAsset
    {
        std::vector<Vector3> vertices;
        std::vector<std::uint32_t> tetrahedra;
        std::vector<Cooking::SoftBodyLevelRecord> levels;
        std::vector<Cooking::SoftBodyBinding> mappings;
        Cooking::SoftBodyAssetView view;

        static constexpr std::uint32_t FINE_COUNT = 4;
        static constexpr std::uint32_t COARSE_COUNT = 4;

        TwoLevelAsset()
        {
            const Vector3 fine[FINE_COUNT] = {
                Vector3{Scalar(0.25), Scalar(0.25), Scalar(0.25)},
                Vector3{Scalar(0.40), Scalar(0.20), Scalar(0.20)},
                Vector3{Scalar(0.20), Scalar(0.40), Scalar(0.20)},
                Vector3{Scalar(0.20), Scalar(0.20), Scalar(0.40)}};
            const Vector3 coarse[COARSE_COUNT] = {Vector3{0, 0, 0}, Vector3{1, 0, 0},
                                                  Vector3{0, 1, 0}, Vector3{0, 0, 1}};

            for (const Vector3& vertex : fine)
                vertices.push_back(vertex);
            for (const Vector3& vertex : coarse)
                vertices.push_back(vertex);

            // Level zero's element, then level one's, in that order — which is what
            // makes level one's `first_tetrahedron` one rather than zero.
            const std::uint32_t elements[8] = {0, 1, 2, 3, 4, 5, 6, 7};
            for (const std::uint32_t index : elements)
                tetrahedra.push_back(index);

            Cooking::SoftBodyLevelRecord level0{};
            level0.first_vertex = 0;
            level0.vertex_count = FINE_COUNT;
            level0.first_tetrahedron = 0;
            level0.tetrahedron_count = 1;
            levels.push_back(level0);

            Cooking::SoftBodyLevelRecord level1{};
            level1.first_vertex = FINE_COUNT;
            level1.vertex_count = COARSE_COUNT;
            level1.first_tetrahedron = 1;
            level1.tetrahedron_count = 1;
            level1.first_mapping = 0;
            level1.mapping_count = FINE_COUNT;
            levels.push_back(level1);

            for (const Vector3& point : fine)
            {
                Cooking::SoftBodyBinding binding{};
                binding.tetrahedron = 1;
                binding.weights[0] = float(1.0 - double(point.x) - double(point.y) -
                                           double(point.z));
                binding.weights[1] = float(point.x);
                binding.weights[2] = float(point.y);
                binding.weights[3] = float(point.z);
                mappings.push_back(binding);
            }

            view.levels = levels.data();
            view.level_count = std::uint32_t(levels.size());
            view.vertices = vertices.data();
            view.vertex_count = std::uint32_t(vertices.size());
            view.tetrahedra = tetrahedra.data();
            view.tetrahedron_count = 2;
            view.mappings = mappings.data();
            view.mapping_count = std::uint32_t(mappings.size());
            view.valid = true;
        }

        /** @brief One level's particles, at rest. */
        std::vector<RigidBodyT<Scalar>> particles_at_rest(std::uint32_t level) const
        {
            const Cooking::SoftBodyLevelRecord& record = levels[level];
            std::vector<RigidBodyT<Scalar>> out(record.vertex_count);
            for (std::uint32_t i = 0; i < record.vertex_count; ++i)
            {
                out[i].position = vertices[record.first_vertex + i];
                out[i].prev_position = out[i].position;
                out[i].inv_mass = Scalar(1);
                out[i].inv_inertia = Vector3{0, 0, 0};
            }
            return out;
        }
    };

    void translate(std::vector<RigidBodyT<Scalar>>& particles, const Vector3& offset)
    {
        for (RigidBodyT<Scalar>& particle : particles)
        {
            particle.position = particle.position + offset;
            particle.prev_position = particle.position;
        }
    }

    Scalar worst_distance(const std::vector<RigidBodyT<Scalar>>& particles,
                          const std::vector<Vector3>& expected)
    {
        Scalar worst = 0;
        for (std::size_t i = 0; i < particles.size() && i < expected.size(); ++i)
        {
            const Scalar distance = length(particles[i].position - expected[i]);
            if (distance > worst)
                worst = distance;
        }
        return worst;
    }
} // namespace

TEST(Unit_SoftBodyLODSelection, CoverageIsIndependentOfHowFarAwayTheSameAngleIs)
{
    // Twice the radius at twice the distance is the same on screen, which is the
    // property that makes this a *screen* coverage rather than a distance dial.
    const Scalar near_body = soft_body_screen_coverage(Scalar(1), Scalar(10), Scalar(0.5));
    const Scalar far_body = soft_body_screen_coverage(Scalar(2), Scalar(20), Scalar(0.5));
    EXPECT_NEAR(double(near_body), double(far_body), 1e-12);
}

TEST(Unit_SoftBodyLODSelection, DropsToACoarserTierOnlyOnceItIsClearOfTheThreshold)
{
    SoftBodyLODSettings<Scalar> settings;
    settings.thresholds = {Scalar(0.5), Scalar(0.2)};
    settings.hysteresis = Scalar(0.2);

    // Just below the threshold but inside the band: it stays where it is.
    EXPECT_EQ(select_soft_body_tier(settings, 3u, 0u, Scalar(0.45)), 0u);
    // Clear of it: it drops.
    EXPECT_EQ(select_soft_body_tier(settings, 3u, 0u, Scalar(0.35)), 1u);
}

TEST(Unit_SoftBodyLODSelection, ClimbsBackOnlyOnceItIsClearTheOtherWay)
{
    SoftBodyLODSettings<Scalar> settings;
    settings.thresholds = {Scalar(0.5), Scalar(0.2)};
    settings.hysteresis = Scalar(0.2);

    EXPECT_EQ(select_soft_body_tier(settings, 3u, 1u, Scalar(0.55)), 1u);
    EXPECT_EQ(select_soft_body_tier(settings, 3u, 1u, Scalar(0.65)), 0u);
}

TEST(Unit_SoftBodyLODSelection, NeverSwapsWhileTheCoverageStaysInsideTheBand)
{
    // The case the whole mechanism exists for: a body parked on a boundary while
    // the camera breathes. Without hysteresis this sequence swaps tiers on every
    // sample, and a tier swap is the most expensive thing this system does.
    SoftBodyLODSettings<Scalar> settings;
    settings.thresholds = {Scalar(0.5)};
    settings.hysteresis = Scalar(0.2);

    std::size_t tier = 0;
    const Scalar samples[6] = {Scalar(0.52), Scalar(0.48), Scalar(0.51), Scalar(0.45),
                               Scalar(0.55), Scalar(0.49)};
    for (const Scalar coverage : samples)
    {
        tier = select_soft_body_tier(settings, 2u, tier, coverage);
        EXPECT_EQ(tier, 0u) << "swapped at coverage " << double(coverage);
    }
}

TEST(Unit_SoftBodyLODSelection, SkipsStraightToTheRightTierAfterAJump)
{
    // A camera cut, not a walk. The selector must land on the correct tier in one
    // call rather than take one frame per tier to get there.
    SoftBodyLODSettings<Scalar> settings;
    settings.thresholds = {Scalar(0.5), Scalar(0.2), Scalar(0.05)};
    settings.hysteresis = Scalar(0.2);

    EXPECT_EQ(select_soft_body_tier(settings, 4u, 0u, Scalar(0.001)), 3u);
    EXPECT_EQ(select_soft_body_tier(settings, 4u, 3u, Scalar(10.0)), 0u);
}

TEST(Unit_SoftBodyLODTransfer, RefiningATranslationIsExact)
{
    const TwoLevelAsset asset;
    std::vector<RigidBodyT<Scalar>> coarse = asset.particles_at_rest(1);
    std::vector<RigidBodyT<Scalar>> fine = asset.particles_at_rest(0);

    const Vector3 offset{Scalar(3), Scalar(-2), Scalar(7)};
    translate(coarse, offset);

    ASSERT_TRUE(refine_soft_body_state(asset.view, 1u, coarse.data(), coarse.size(), fine.data(),
                                       fine.size()));

    std::vector<Vector3> expected;
    for (std::uint32_t i = 0; i < TwoLevelAsset::FINE_COUNT; ++i)
        expected.push_back(asset.vertices[i] + offset);
    EXPECT_LT(double(worst_distance(fine, expected)), 1e-12);
}

TEST(Unit_SoftBodyLODTransfer, CoarseningATranslationIsExact)
{
    // The property the scatter is built for. A body that is merely falling must
    // cross a tier boundary with no motion whatsoever, and a transfer that only
    // approximated translation would make every distant body twitch each time the
    // camera drifted across a threshold.
    const TwoLevelAsset asset;
    std::vector<RigidBodyT<Scalar>> fine = asset.particles_at_rest(0);
    std::vector<RigidBodyT<Scalar>> coarse = asset.particles_at_rest(1);

    const Vector3 offset{Scalar(-1), Scalar(4), Scalar(0.5)};
    translate(fine, offset);

    ASSERT_TRUE(coarsen_soft_body_state(asset.view, 1u, fine.data(), fine.size(), coarse.data(),
                                        coarse.size()));

    std::vector<Vector3> expected;
    for (std::uint32_t i = 0; i < TwoLevelAsset::COARSE_COUNT; ++i)
        expected.push_back(asset.vertices[TwoLevelAsset::FINE_COUNT + i] + offset);
    EXPECT_LT(double(worst_distance(coarse, expected)), 1e-12);
}

TEST(Unit_SoftBodyLODTransfer, TheRestPoseCrossesInBothDirectionsWithoutMoving)
{
    // The pop, stated as a test. An undeformed body handed to another tier must
    // come back undeformed — which is true only because the transfer is written
    // in displacements; the reconstruction of a rest pose is *not* the rest pose.
    const TwoLevelAsset asset;
    const std::vector<RigidBodyT<Scalar>> fine_rest = asset.particles_at_rest(0);
    const std::vector<RigidBodyT<Scalar>> coarse_rest = asset.particles_at_rest(1);

    std::vector<RigidBodyT<Scalar>> coarse = coarse_rest;
    ASSERT_TRUE(coarsen_soft_body_state(asset.view, 1u, fine_rest.data(), fine_rest.size(),
                                        coarse.data(), coarse.size()));
    std::vector<Vector3> coarse_expected;
    for (const RigidBodyT<Scalar>& particle : coarse_rest)
        coarse_expected.push_back(particle.position);
    EXPECT_LT(double(worst_distance(coarse, coarse_expected)), 1e-12);

    std::vector<RigidBodyT<Scalar>> fine = fine_rest;
    ASSERT_TRUE(refine_soft_body_state(asset.view, 1u, coarse_rest.data(), coarse_rest.size(),
                                       fine.data(), fine.size()));
    std::vector<Vector3> fine_expected;
    for (const RigidBodyT<Scalar>& particle : fine_rest)
        fine_expected.push_back(particle.position);
    EXPECT_LT(double(worst_distance(fine, fine_expected)), 1e-12);
}

TEST(Unit_SoftBodyLODTransfer, CarriesVelocityAcrossSoABodyDoesNotStall)
{
    const TwoLevelAsset asset;
    std::vector<RigidBodyT<Scalar>> fine = asset.particles_at_rest(0);
    std::vector<RigidBodyT<Scalar>> coarse = asset.particles_at_rest(1);

    const Vector3 velocity{0, 0, Scalar(-9)};
    for (RigidBodyT<Scalar>& particle : fine)
        particle.velocity = velocity;

    ASSERT_TRUE(coarsen_soft_body_state(asset.view, 1u, fine.data(), fine.size(), coarse.data(),
                                        coarse.size()));
    for (const RigidBodyT<Scalar>& particle : coarse)
        EXPECT_LT(double(length(particle.velocity - velocity)), 1e-12);
}

TEST(Unit_SoftBodyLODTransfer, RefusesALevelTheAssetDoesNotHave)
{
    const TwoLevelAsset asset;
    std::vector<RigidBodyT<Scalar>> a = asset.particles_at_rest(0);
    std::vector<RigidBodyT<Scalar>> b = asset.particles_at_rest(1);

    EXPECT_FALSE(refine_soft_body_state(asset.view, 0u, b.data(), b.size(), a.data(), a.size()));
    EXPECT_FALSE(refine_soft_body_state(asset.view, 5u, b.data(), b.size(), a.data(), a.size()));
    EXPECT_FALSE(coarsen_soft_body_state(asset.view, 0u, a.data(), a.size(), b.data(), b.size()));
}

TEST(Integration_SoftBodyLOD, SwapsTiersAndKeepsThePoseAcrossTheSwap)
{
    // Both tiers here sit on the *same* lattice — shape matching and rigid, §9.7's
    // two coarsest — so the transfer is a copy and the pose must survive it
    // exactly. That makes this the case that isolates the chain's bookkeeping
    // from the remap's arithmetic, which the transfer cases above cover.
    const TwoLevelAsset asset;

    std::unique_ptr<ShapeMatchingModel<Scalar>> fine(new ShapeMatchingModel<Scalar>());
    fine->particles = asset.particles_at_rest(1);
    fine->capture_rest_shape();

    std::unique_ptr<RigidSoftBodyModel<Scalar>> coarse(new RigidSoftBodyModel<Scalar>());
    coarse->particles = asset.particles_at_rest(1);
    coarse->freeze_from_particles();

    SoftBodyLODChain<Scalar> chain;
    chain.settings.thresholds = {Scalar(0.5)};
    chain.settings.hysteresis = Scalar(0.2);
    chain.add_tier(std::unique_ptr<ISoftBodyModel<Scalar>>(fine.release()), 1u);
    chain.add_tier(std::unique_ptr<ISoftBodyModel<Scalar>>(coarse.release()), 1u);

    ASSERT_NE(chain.active(), nullptr);
    EXPECT_EQ(chain.current_tier(), 0u);

    // Displace the fine tier, then push it over the boundary.
    const Vector3 offset{Scalar(0.3), Scalar(-0.1), Scalar(0.2)};
    const SoftSurfaceView<Scalar> before = chain.active()->surface();
    std::vector<Vector3> displaced;
    for (std::size_t i = 0; i < before.particle_count; ++i)
    {
        before.particles[i].position = before.particles[i].position + offset;
        before.particles[i].prev_position = before.particles[i].position;
        displaced.push_back(before.particles[i].position);
    }

    EXPECT_TRUE(chain.update(asset.view, Scalar(0.1)));
    EXPECT_EQ(chain.current_tier(), 1u);

    const SoftSurfaceView<Scalar> after = chain.active()->surface();
    ASSERT_EQ(after.particle_count, displaced.size());
    for (std::size_t i = 0; i < after.particle_count; ++i)
        EXPECT_LT(double(length(after.particles[i].position - displaced[i])), 1e-12)
            << "particle " << i << " popped when the tier changed";

    // And back, with the same requirement in the other direction.
    EXPECT_TRUE(chain.update(asset.view, Scalar(2.0)));
    EXPECT_EQ(chain.current_tier(), 0u);
    const SoftSurfaceView<Scalar> returned = chain.active()->surface();
    for (std::size_t i = 0; i < returned.particle_count; ++i)
        EXPECT_LT(double(length(returned.particles[i].position - displaced[i])), 1e-12)
            << "particle " << i << " popped on the way back";
}

TEST(Integration_SoftBodyLOD, StaysOnTheSameTierWhenTheCoverageBarelyMoves)
{
    const TwoLevelAsset asset;

    std::unique_ptr<ShapeMatchingModel<Scalar>> fine(new ShapeMatchingModel<Scalar>());
    fine->particles = asset.particles_at_rest(1);
    fine->capture_rest_shape();
    std::unique_ptr<RigidSoftBodyModel<Scalar>> coarse(new RigidSoftBodyModel<Scalar>());
    coarse->particles = asset.particles_at_rest(1);
    coarse->freeze_from_particles();

    SoftBodyLODChain<Scalar> chain;
    chain.settings.thresholds = {Scalar(0.5)};
    chain.settings.hysteresis = Scalar(0.2);
    chain.add_tier(std::unique_ptr<ISoftBodyModel<Scalar>>(fine.release()), 1u);
    chain.add_tier(std::unique_ptr<ISoftBodyModel<Scalar>>(coarse.release()), 1u);

    const Scalar samples[4] = {Scalar(0.49), Scalar(0.52), Scalar(0.47), Scalar(0.53)};
    for (const Scalar coverage : samples)
        EXPECT_FALSE(chain.update(asset.view, coverage))
            << "changed tier at coverage " << double(coverage);
}
