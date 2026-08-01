/**************************************************************************/
/* test_soft_body_precision.cpp                                           */
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

// §6.5's cosmetic float column (P6-I). The rule is small enough to state in a
// sentence and important enough that each clause of it gets its own case — the
// one that matters most being the override: a body a rollback replays is
// simulated in double whatever its component asked for, because two machines
// agreeing in double and disagreeing in float is the entire failure the
// determinism rules exist to prevent.

#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/physics/soft/soft_body_instance.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    /** @brief A one-tetrahedron asset, enough to instantiate at either width. */
    struct TinyAsset
    {
        std::vector<Vector3> vertices;
        std::vector<Scalar> mass;
        std::vector<std::uint32_t> tetrahedra;
        std::vector<Vector3> rest_inverse;
        std::vector<Scalar> rest_volume;
        std::vector<Cooking::SoftBodyLevelRecord> levels;
        Cooking::SoftBodyAssetView view;

        TinyAsset()
        {
            vertices = {Vector3{0, 0, 0}, Vector3{1, 0, 0}, Vector3{0, 1, 0}, Vector3{0, 0, 1}};
            mass = {Scalar(1), Scalar(1), Scalar(1), Scalar(1)};
            tetrahedra = {0, 1, 2, 3};
            rest_inverse = {Vector3{1, 0, 0}, Vector3{0, 1, 0}, Vector3{0, 0, 1}};
            rest_volume = {Scalar(1.0 / 6.0)};

            Cooking::SoftBodyLevelRecord level{};
            level.first_vertex = 0;
            level.vertex_count = 4;
            level.first_tetrahedron = 0;
            level.tetrahedron_count = 1;
            levels.push_back(level);

            view.levels = levels.data();
            view.level_count = 1;
            view.vertices = vertices.data();
            view.vertex_mass = mass.data();
            view.vertex_count = 4;
            view.tetrahedra = tetrahedra.data();
            view.rest_inverse = rest_inverse.data();
            view.rest_volume = rest_volume.data();
            view.tetrahedron_count = 1;
            view.valid = true;
        }
    };
} // namespace

TEST(Unit_SoftBodyPrecision, AnOrdinaryBodyGetsTheGameplayColumn)
{
    const TinyAsset asset;
    SoftBodyPrecisionRequest request;
    EXPECT_EQ(resolve_soft_body_precision(asset.view, request), SoftBodyPrecision::Gameplay);
}

TEST(Unit_SoftBodyPrecision, TheCosmeticFlagNarrowsTheColumn)
{
    const TinyAsset asset;
    SoftBodyPrecisionRequest request;
    request.cosmetic = true;
    EXPECT_EQ(resolve_soft_body_precision(asset.view, request), SoftBodyPrecision::Cosmetic);
}

TEST(Unit_SoftBodyPrecision, RollbackOverridesTheCosmeticFlag)
{
    // The clause that is not a preference. A body the deterministic island
    // replays must be bit-reproducible, and the flag is not allowed to trade
    // that away however loudly it is set.
    const TinyAsset asset;
    SoftBodyPrecisionRequest request;
    request.cosmetic = true;
    request.participates_in_rollback = true;
    EXPECT_EQ(resolve_soft_body_precision(asset.view, request), SoftBodyPrecision::Gameplay);
}

TEST(Unit_SoftBodyPrecision, AnUnusableAssetIsNeverNarrowed)
{
    Cooking::SoftBodyAssetView empty;
    SoftBodyPrecisionRequest request;
    request.cosmetic = true;
    EXPECT_EQ(resolve_soft_body_precision(empty, request), SoftBodyPrecision::Gameplay);
}

TEST(Unit_SoftBodyInstance, InstantiatesInWhicheverColumnItWasGiven)
{
    const TinyAsset asset;
    SoftBodyMaterialT<Scalar> material;

    SoftBodyInstance gameplay;
    ASSERT_TRUE(gameplay.create(asset.view, 0u, material, Vector3{0, 0, 0},
                                SoftBodyPrecision::Gameplay));
    EXPECT_EQ(gameplay.precision(), SoftBodyPrecision::Gameplay);
    EXPECT_NE(gameplay.gameplay_model(), nullptr);
    EXPECT_EQ(gameplay.cosmetic_model(), nullptr);

    SoftBodyInstance cosmetic;
    ASSERT_TRUE(cosmetic.create(asset.view, 0u, material, Vector3{0, 0, 0},
                                SoftBodyPrecision::Cosmetic));
    EXPECT_EQ(cosmetic.precision(), SoftBodyPrecision::Cosmetic);
    EXPECT_EQ(cosmetic.gameplay_model(), nullptr);
    EXPECT_NE(cosmetic.cosmetic_model(), nullptr);

    // Both answer in the boundary type, which is what keeps the decision from
    // spreading past this class.
    ASSERT_EQ(gameplay.particle_count(), cosmetic.particle_count());
    for (std::size_t i = 0; i < gameplay.particle_count(); ++i)
        EXPECT_LT(double(length(gameplay.particle_position(i) - cosmetic.particle_position(i))),
                  1e-6)
            << "particle " << i << " was placed differently at the two widths";
}

TEST(Unit_SoftBodyInstance, BothColumnsFallTheSameWayToWithinTheirPrecision)
{
    // Not a determinism claim — the two widths are allowed to differ, which is
    // exactly why only one of them may be replayed. What is claimed is that the
    // narrow column is the *same simulation*, not a different one: a centimetre
    // of disagreement after a second would mean the float body was not usable
    // even as decoration.
    const TinyAsset asset;
    SoftBodyMaterialT<Scalar> material;

    SoftBodyInstance gameplay;
    SoftBodyInstance cosmetic;
    ASSERT_TRUE(gameplay.create(asset.view, 0u, material, Vector3{0, 0, 0},
                                SoftBodyPrecision::Gameplay));
    ASSERT_TRUE(cosmetic.create(asset.view, 0u, material, Vector3{0, 0, 0},
                                SoftBodyPrecision::Cosmetic));
    gameplay.set_external_acceleration(Vector3{0, 0, Scalar(-9.81)});
    cosmetic.set_external_acceleration(Vector3{0, 0, Scalar(-9.81)});

    for (int tick = 0; tick < 60; ++tick)
    {
        gameplay.step(Scalar(1.0 / 60.0), 10);
        cosmetic.step(Scalar(1.0 / 60.0), 10);
    }

    // Measured as a *fraction of the distance fallen*, not as an absolute
    // displacement. Float carries about seven significant digits, and this scene
    // accumulates six hundred position updates in which the increment is four
    // orders of magnitude smaller than the position it is added to — so the error
    // grows with how far the body has travelled, and an absolute bound would be a
    // bound that tightens the longer the test runs.
    //
    // The bound is set from measurement rather than from theory: the worst particle
    // diverges by 1.2 mm over a 4.97 m fall, a part in four thousand. Float's own
    // epsilon is a part in eight million, so the extra three orders of magnitude are
    // six hundred accumulations plus a neo-Hookean projection solved at the same
    // width every substep. One part in a thousand leaves four times that headroom
    // and is still a hundred times tighter than any schedule error would be.
    const double fallen =
        double(gameplay.particle_position(0).z - Scalar(0)) < 0.0
            ? -double(gameplay.particle_position(0).z)
            : 1.0;
    for (std::size_t i = 0; i < gameplay.particle_count(); ++i)
    {
        const double divergence =
            double(length(gameplay.particle_position(i) - cosmetic.particle_position(i)));
        EXPECT_LT(divergence, 1e-3 * fallen)
            << "particle " << i << " diverged past what float explains, after falling " << fallen
            << " m";
    }
}

TEST(Unit_SoftBodyInstance, RefusesALevelTheAssetDoesNotHave)
{
    const TinyAsset asset;
    SoftBodyMaterialT<Scalar> material;
    SoftBodyInstance instance;

    EXPECT_FALSE(
        instance.create(asset.view, 7u, material, Vector3{0, 0, 0}, SoftBodyPrecision::Cosmetic));
    EXPECT_FALSE(instance.valid()) << "a refused create must leave nothing half-built";
    EXPECT_EQ(instance.particle_count(), 0u);
}
