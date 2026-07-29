/**************************************************************************/
/* test_atmosphere_quality.cpp                                            */
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

// Pins the atmosphere tier table — the grid ladder that used to ride the render
// quality resolver and moved to the simulation's own tier so a rendering knob can
// never rebuild the weather. Every row here is a value the Phase B measurements
// were taken against; changing one is changing what a tier costs and simulates,
// and must be a deliberate act that updates this test with its rationale.

#include <gtest/gtest.h>

#include <SushiEngine/sim/simulation_settings.hpp>

namespace
{
    using SushiEngine::Simulation::AtmosphereQuality;
    using SushiEngine::Simulation::resolve_atmosphere_quality;

    void expect_grid(AtmosphereQuality quality, std::uint32_t cells_x, std::uint32_t cells_z,
                     std::uint32_t levels, float spacing_m)
    {
        const SushiEngine::Render::AtmosphereNestSize size = resolve_atmosphere_quality(quality);
        EXPECT_EQ(size.cells_x, cells_x);
        EXPECT_EQ(size.cells_z, cells_z);
        EXPECT_EQ(size.levels, levels);
        EXPECT_FLOAT_EQ(size.spacing_m, spacing_m);
        EXPECT_FLOAT_EQ(size.top_m, 18000.0f);
    }
} // namespace

TEST(Unit_AtmosphereQuality, GridTablePinnedPerTier)
{
    expect_grid(AtmosphereQuality::Low, 96u, 96u, 32u, 4000.0f);
    expect_grid(AtmosphereQuality::Medium, 128u, 128u, 40u, 3000.0f);
    expect_grid(AtmosphereQuality::High, 192u, 192u, 48u, 2000.0f);
    expect_grid(AtmosphereQuality::Ultra, 256u, 256u, 64u, 1500.0f);
}

TEST(Unit_AtmosphereQuality, EveryTierSimulatesTheSameDomain)
{
    // The horizontal domain is 384 km at every tier: raising the tier resolves the
    // same weather more finely rather than simulating a different amount of world.
    for (const AtmosphereQuality quality :
         {AtmosphereQuality::Low, AtmosphereQuality::Medium, AtmosphereQuality::High,
          AtmosphereQuality::Ultra})
    {
        const SushiEngine::Render::AtmosphereNestSize size = resolve_atmosphere_quality(quality);
        EXPECT_FLOAT_EQ(static_cast<float>(size.cells_x) * size.spacing_m, 384000.0f);
        EXPECT_FLOAT_EQ(static_cast<float>(size.cells_z) * size.spacing_m, 384000.0f);
    }
}

TEST(Unit_AtmosphereQuality, HighIsTheShippedBaseline)
{
    // High must equal AtmosphereNestSize's own defaults: the resolver's contract is
    // that the default tier is the shipped baseline, not a variation of it.
    const SushiEngine::Render::AtmosphereNestSize baseline{};
    const SushiEngine::Render::AtmosphereNestSize high =
        resolve_atmosphere_quality(AtmosphereQuality::High);
    EXPECT_EQ(high.cells_x, baseline.cells_x);
    EXPECT_EQ(high.cells_z, baseline.cells_z);
    EXPECT_EQ(high.levels, baseline.levels);
    EXPECT_FLOAT_EQ(high.spacing_m, baseline.spacing_m);
    EXPECT_FLOAT_EQ(high.top_m, baseline.top_m);
}
