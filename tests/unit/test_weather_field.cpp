/**************************************************************************/
/* test_weather_field.cpp                                                 */
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

// Unit_WeatherField: the spatial weather field (docs/design/atmosphere_system.md §7), which
// exists to close §1.1 — a simulation that reaches the renderer as one column sampled under
// the observer has nothing it computes visible as spatial structure. The load-bearing claim
// these cases defend is therefore *that the field is not uniform when the simulation is not
// uniform*, plus the addressing that decides where in the world each cell lands. Pure host
// maths; no SushiRuntime needed, same reasoning as the other weather tests.

#include <algorithm>
#include <vector>
#include <cmath>
#include <memory>

#include <gtest/gtest.h>

#include <SushiEngine/SushiEngine.hpp>
#include <SushiEngine/simulation/ingested_weather.hpp>
#include <SushiEngine/simulation/weather_field_buffer.hpp>
#include <SushiEngine/simulation/weather_provider.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Simulation;

namespace
{
    constexpr double EARTH_RADIUS_M = 6371000.0;
    constexpr double DEGREES_TO_RADIANS = 3.14159265358979323846 / 180.0;

    const Render::WeatherFieldSample& sample_at(const Render::WeatherField& field, int level,
                                                int ix, int iz)
    {
        const std::size_t index =
            (std::size_t(level) * std::size_t(field.cells_z) + std::size_t(iz)) *
                std::size_t(field.cells_x) + std::size_t(ix);
        return field.samples[index];
    }

    // What the renderer does per march sample, in the one place a test can check it against
    // the producer's own intent: scene metres -> field UV.
    void field_uv(const Render::WeatherField& field, double scene_x, double scene_z, double& u,
                  double& v)
    {
        u = double(field.uv_scale_x) * scene_x + double(field.uv_offset_x);
        v = double(field.uv_scale_z) * scene_z + double(field.uv_offset_z);
    }

    //: Cells per axis of the synthetic mirrors below. The real one is
    //: Render::ATMOSPHERE_MIRROR_CELLS; a smaller lattice keeps the tests readable and the
    //: buffer's own resampling is exercised by the size difference rather than hidden by it.
    constexpr int MIRROR_CELLS = 8;

    // A mirror's worth of columns whose low-band coverage is `coverage(ix, iz)`. Built here
    // rather than by running a provider because the grid now comes from the GPU: a CPU test can
    // either state what the readback said or test nothing at all.
    template <typename Coverage>
    std::vector<Render::AtmosphereMirrorColumn> mirror_columns(int cells, Coverage coverage)
    {
        std::vector<Render::AtmosphereMirrorColumn> columns(std::size_t(cells) *
                                                            std::size_t(cells));
        for (int iz = 0; iz < cells; ++iz)
            for (int ix = 0; ix < cells; ++ix)
            {
                Render::AtmosphereMirrorColumn& column =
                    columns[std::size_t(iz) * std::size_t(cells) + std::size_t(ix)];
                column.bands[int(CloudLevel::Low)][0] = coverage(ix, iz);
                column.bands[int(CloudLevel::Low)][1] = 1.0f;
            }
        return columns;
    }

    /** @brief A borrowed view of @p columns with a unit mapping the caller may overwrite. */
    Render::AtmosphereMirror mirror_view(
        const std::vector<Render::AtmosphereMirrorColumn>& columns, int cells)
    {
        Render::AtmosphereMirror mirror{};
        mirror.columns = columns.data();
        mirror.cells = cells;
        mirror.revision = 1;
        mirror.uv_scale_x = 1.0f;
        mirror.uv_scale_z = 1.0f;
        return mirror;
    }
} // namespace

TEST(Unit_WeatherField, UniformFillPublishesOneCellCarryingTheColumn)
{
    WeatherColumn column{};
    column.levels[int(CloudLevel::Low)].coverage = 0.7f;
    column.levels[int(CloudLevel::Low)].density_scale = 1.4f;
    column.levels[int(CloudLevel::Low)].convective_fraction = 0.25f;
    column.precipitation = 0.5f;

    WeatherFieldBuffer buffer;
    buffer.fill_uniform(column);
    const Render::WeatherField field = buffer.view();

    ASSERT_TRUE(field.valid());
    EXPECT_EQ(field.cells_x, 1);
    EXPECT_EQ(field.cells_z, 1);
    EXPECT_EQ(field.level_count, CLOUD_LEVEL_COUNT);

    const Render::WeatherFieldSample& low = sample_at(field, int(CloudLevel::Low), 0, 0);
    EXPECT_FLOAT_EQ(low.coverage, 0.7f);
    EXPECT_FLOAT_EQ(low.density_scale, 1.4f);
    EXPECT_FLOAT_EQ(low.convective_fraction, 0.25f);
    EXPECT_FLOAT_EQ(low.precipitation, 0.5f);

    // A uniform provider must read the same everywhere, however far the march wanders:
    // constant-mapped onto the single texel's centre rather than left to drift off the edge.
    double u = 0.0;
    double v = 0.0;
    field_uv(field, 500000.0, -900000.0, u, v);
    EXPECT_DOUBLE_EQ(u, 0.5);
    EXPECT_DOUBLE_EQ(v, 0.5);
}

TEST(Unit_WeatherField, GridFillIsSpatiallyNonUniformWhenTheMirrorIs)
{
    // The whole point of the field, asked of the path the grid now actually arrives by.
    //
    // Phase A filled this from a CPU weather grid, and Phase B2 deleted that grid: the
    // atmosphere lives on the GPU, so the only spatial weather a provider can publish is what
    // came back from the nest. A `ProceduralWeather` with no mirror bound therefore publishes a
    // uniform clear field on purpose — `PublishedFieldFallsBackToClearWithoutAMirror` pins that
    // — and asking it for a varying grid is asking the retired design.
    const GeodeticPosition observer{45.0 * DEGREES_TO_RADIANS, 10.0 * DEGREES_TO_RADIANS};
    const std::vector<Render::AtmosphereMirrorColumn> columns =
        mirror_columns(MIRROR_CELLS, [](int ix, int iz)
        {
            // A front: clear on one side, overcast on the other, with a gradient between.
            return float(ix + iz) / float(2 * (MIRROR_CELLS - 1));
        });
    const Render::AtmosphereMirror mirror = mirror_view(columns, MIRROR_CELLS);

    WeatherFieldBuffer buffer;
    buffer.fill_from_mirror(mirror, observer);
    const Render::WeatherField field = buffer.view();

    ASSERT_TRUE(field.valid());
    ASSERT_GT(field.cells_x, 1);
    ASSERT_GT(field.cells_z, 1);

    float minimum = 2.0f;
    float maximum = -1.0f;
    for (int iz = 0; iz < field.cells_z; ++iz)
        for (int ix = 0; ix < field.cells_x; ++ix)
        {
            const float coverage = sample_at(field, int(CloudLevel::Low), ix, iz).coverage;
            minimum = std::min(minimum, coverage);
            maximum = std::max(maximum, coverage);
        }

    EXPECT_GT(maximum - minimum, 0.01f)
        << "the published field is uniform, so no front, shower, or clearing could ever be "
           "visible in the sky -- the exact defect the field exists to remove";
}

TEST(Unit_WeatherField, GridAddressingIsTheMirrorsOwnAndNotASecondDerivation)
{
    // Get this wrong and every cloud is drawn over the wrong part of the world.
    //
    // The field derives no lattice of its own. The nest is square in *metres* and is centred
    // on the observer by the renderer, and the mirror carries the scene-absolute mapping it
    // was centred with; passing that through rather than rebuilding it is the point, because
    // two derivations of one lattice are two chances to disagree about where the weather is.
    // A geodetic lattice built here would make easting run faster than northing by
    // 1/cos(latitude) on a plate-carree grid, so what a test can check is that the field
    // took the mirror's addressing unchanged.
    const GeodeticPosition observer{45.0 * DEGREES_TO_RADIANS, 10.0 * DEGREES_TO_RADIANS};
    const std::vector<Render::AtmosphereMirrorColumn> columns =
        mirror_columns(MIRROR_CELLS, [](int, int) { return 0.5f; });
    Render::AtmosphereMirror mirror = mirror_view(columns, MIRROR_CELLS);

    // A nest centred on scene (0, 0) spanning `span` metres: world zero lands at the middle.
    const double span = 384000.0;
    mirror.uv_scale_x = float(1.0 / span);
    mirror.uv_scale_z = float(1.0 / span);
    mirror.uv_offset_x = 0.5f;
    mirror.uv_offset_z = 0.5f;

    WeatherFieldBuffer buffer;
    buffer.fill_from_mirror(mirror, observer);
    const Render::WeatherField field = buffer.view();
    ASSERT_TRUE(field.valid());

    double u = 0.0;
    double v = 0.0;
    field_uv(field, 0.0, 0.0, u, v);
    const double half_cell_u = 0.5 / double(field.cells_x);
    const double half_cell_v = 0.5 / double(field.cells_z);
    EXPECT_NEAR(u, 0.5, half_cell_u);
    EXPECT_NEAR(v, 0.5, half_cell_v);

    // Scene +X is east and +Z is north, so both must increase with the coordinate, and the two
    // must scale *identically*: the nest's cells are square in metres, and a field that scaled
    // them differently would be re-introducing a projection the nest does not have.
    EXPECT_GT(field.uv_scale_x, 0.0f);
    EXPECT_GT(field.uv_scale_z, 0.0f);
    EXPECT_FLOAT_EQ(field.uv_scale_x, mirror.uv_scale_x);
    EXPECT_FLOAT_EQ(field.uv_scale_z, mirror.uv_scale_z);
    EXPECT_FLOAT_EQ(field.uv_offset_x, mirror.uv_offset_x);
    EXPECT_FLOAT_EQ(field.uv_offset_z, mirror.uv_offset_z);

    // Half the span east of the observer is the field's own edge, not somewhere off it.
    field_uv(field, span * 0.5, 0.0, u, v);
    EXPECT_NEAR(u, 1.0, 1e-6);
}

TEST(Unit_WeatherField, EveryProviderPublishesAUsableField)
{
    // The substitutability the seam claims, at the field half of the contract: whatever the
    // host installs, the renderer gets something it can address — which is only askable
    // because the field is on the interface rather than on a concrete provider type.
    const GeodeticPosition observer{45.0 * DEGREES_TO_RADIANS, 10.0 * DEGREES_TO_RADIANS};

    Render::Cloudscape authored;
    authored.enabled = true;
    authored.decks[0].enabled = true;
    authored.decks[0].genus = Render::CloudGenus::Stratocumulus;

    std::unique_ptr<IWeatherProvider> providers[3];
    providers[0] = std::make_unique<StaticWeather>(authored);
    providers[1] = std::make_unique<ProceduralWeather>(/*seed=*/11, EARTH_RADIUS_M);
    auto ingested = std::make_unique<IngestedWeather>();
    ingested->add_station(observer, "LTBA 121350Z 27012KT 9999 BKN020 18/12 Q1013");
    providers[2] = std::move(ingested);

    for (std::unique_ptr<IWeatherProvider>& provider : providers)
    {
        provider->tick(1.0, observer, 2451545.0);
        WeatherFieldBuffer buffer;
        provider->publish_field(observer, buffer);
        const Render::WeatherField field = buffer.view();

        ASSERT_TRUE(field.valid());
        EXPECT_GE(field.cells_x, 1);
        EXPECT_LE(field.cells_x, Render::WEATHER_FIELD_MAX_CELLS);
        EXPECT_GE(field.cells_z, 1);
        EXPECT_LE(field.cells_z, Render::WEATHER_FIELD_MAX_CELLS);
        EXPECT_EQ(field.level_count, CLOUD_LEVEL_COUNT);
        EXPECT_GT(field.revision, 0u);
        // Ascending band centres: the renderer's altitude-to-band mapping divides by their
        // differences and would produce nonsense from an unordered set.
        EXPECT_LT(field.level_altitudes[0], field.level_altitudes[1]);
        EXPECT_LT(field.level_altitudes[1], field.level_altitudes[2]);
    }
}

TEST(Unit_WeatherField, AuthoredColumnDoesNotClaimTheGenusAuthority)
{
    // `StaticWeather`'s column *is* an authored deck stack decomposed into bands, so the bake
    // must not round-trip it back through the classifier and overrule the author's own genus.
    // The producer says so; the renderer does not guess (Render::WeatherField::derives_genus).
    WeatherColumn column{};
    column.levels[int(CloudLevel::Low)].coverage = 0.42f;
    column.levels[int(CloudLevel::Mid)].coverage = 0.11f;

    WeatherFieldBuffer buffer;
    buffer.fill_uniform(column, false);
    const Render::WeatherField field = buffer.view();

    ASSERT_TRUE(field.valid());
    EXPECT_FALSE(field.derives_genus);
    // With no classification there is no field-derived march shell either, so the renderer
    // keeps deriving it from the deck stack, which is the only thing that knows the sky.
    EXPECT_FLOAT_EQ(field.union_base_m, 0.0f);
    EXPECT_FLOAT_EQ(field.union_top_m, 0.0f);
    EXPECT_FLOAT_EQ(sample_at(field, int(CloudLevel::Low), 0, 0).coverage, 0.42f);
}

TEST(Unit_WeatherField, ClassifiedColumnPublishesTheShellItsGeneraNeed)
{
    // The march shell has to bound every deck the bake can resolve, or a cloud the simulation
    // put 300 km away is simply outside the span the march crosses and is never seen. A
    // strongly convective, well-covered low band resolves to cumulus *and* tops it with a
    // cumulonimbus, so the published span must reach that tower's own top.
    WeatherColumn column{};
    WeatherLevelState& low = column.levels[int(CloudLevel::Low)];
    low.coverage = 0.6f;
    low.convective_fraction = 0.9f;

    WeatherFieldBuffer buffer;
    buffer.fill_uniform(column, true);
    const Render::WeatherField field = buffer.view();

    ASSERT_TRUE(field.valid());
    EXPECT_TRUE(field.derives_genus);
    const Render::CloudGenusProfile tower =
        Render::cloud_genus_profile(Render::CloudGenus::Cumulonimbus);
    const Render::CloudGenusProfile cumulus =
        Render::cloud_genus_profile(Render::CloudGenus::Cumulus);
    EXPECT_FLOAT_EQ(field.union_top_m, tower.top_altitude);
    EXPECT_FLOAT_EQ(field.union_base_m, std::min(tower.base_altitude, cumulus.base_altitude));
}

TEST(Unit_WeatherField, BandBelowTheEnableThresholdCarriesNoDeck)
{
    // A trace of coverage is not a cloud. The classifier's enable threshold is the one place
    // that decides so, and the published shell has to agree with it — otherwise the march
    // would cross a span held open by a band that bakes nothing.
    WeatherColumn column{};
    column.levels[int(CloudLevel::High)].coverage =
        Render::cloud_genus_thresholds().enable_coverage * 0.5f;

    WeatherFieldBuffer buffer;
    buffer.fill_uniform(column, true);
    const Render::WeatherField field = buffer.view();

    ASSERT_TRUE(field.valid());
    EXPECT_TRUE(field.derives_genus);
    EXPECT_FLOAT_EQ(field.union_base_m, 0.0f);
    EXPECT_FLOAT_EQ(field.union_top_m, 0.0f);
}
