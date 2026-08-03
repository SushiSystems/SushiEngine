/**************************************************************************/
/* test_climatology.cpp                                                   */
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

// Unit_Climatology: T0, the mean state the global core is a departure from
// (`docs/slop/atmosphere_system.md` §4).
//
// Two kinds of claim are tested here and they need different instruments.
//
// The *format* claims -- that a blob round-trips, and that a blob which is not what it says
// it is gets refused rather than half-read -- are tested against synthesized bytes, because a
// test that could only fail when a 3.4 MB asset is present is a test that will be skipped on
// the machine where it matters. A malformed climatology is the dangerous case: it would not
// crash, it would grow weather on a mean state nobody chose, and that looks like physics.
//
// The *data* claims -- that the jet is where the reanalysis puts it, that the water is in
// the tropics, that the Sahara is land -- can only be tested against the shipped asset, so
// those cases locate it and skip with a reason if it is genuinely absent.

#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/atmosphere/climatology.hpp>
#include <SushiEngine/simulation/climatology_asset.hpp>

#include "test_helpers.hpp"

using namespace SushiEngine;
using namespace SushiEngine::Atmosphere;

namespace
{
    constexpr double PI = 3.14159265358979323846;

    double radians(double degrees) { return degrees * PI / 180.0; }

    GeographicPosition at(double latitude_degrees, double longitude_degrees)
    {
        GeographicPosition position;
        position.latitude_radians = radians(latitude_degrees);
        position.longitude_radians = radians(longitude_degrees);
        return position;
    }

    /**
     * @brief Bytes before the first float: magic, version, two grids, and the provenance length.
     *
     * Named rather than counted at the call site, because getting it wrong writes a float over
     * the provenance length and the blob is then refused for a reason that has nothing to do
     * with what the test was about.
     */
    constexpr std::size_t HEADER_BYTES = 4 + 4 + 4 + 4 + 4 + 4 + 4;

    /** @brief How the bake lays a `SET0` blob out, rebuilt here so a test can bend it. */
    struct BlobLayout
    {
        char magic[4] = {'S', 'E', 'T', '0'};
        std::uint32_t version = 1;
        std::int32_t bands = 4;
        std::int32_t months = 12;
        std::int32_t surface_longitudes = 4;
        std::int32_t surface_latitudes = 2;
        std::string provenance = "a test";

        /** @brief Upper wind written into every band of every month, m/s. */
        float upper = 30.0f;
        float lower = 10.0f;
        float saturation = 50.0f;
        float land = 0.0f;
        float sea = 290.0f;
    };

    template <typename T>
    void put(std::vector<std::uint8_t>& blob, const T& value)
    {
        const auto* bytes = reinterpret_cast<const std::uint8_t*>(&value);
        blob.insert(blob.end(), bytes, bytes + sizeof(T));
    }

    void put_floats(std::vector<std::uint8_t>& blob, std::size_t count, float value)
    {
        for (std::size_t i = 0; i < count; ++i)
            put(blob, value);
    }

    /** @brief Builds a well-formed blob from @p layout, so a test can break exactly one thing. */
    std::vector<std::uint8_t> build(const BlobLayout& layout = BlobLayout())
    {
        std::vector<std::uint8_t> blob;
        blob.insert(blob.end(), layout.magic, layout.magic + 4);
        put(blob, layout.version);
        put(blob, layout.bands);
        put(blob, layout.months);
        put(blob, layout.surface_longitudes);
        put(blob, layout.surface_latitudes);
        put(blob, std::uint32_t(layout.provenance.size()));

        const std::size_t profile = std::size_t(layout.bands) * std::size_t(layout.months);
        const std::size_t plane =
            std::size_t(layout.surface_longitudes) * std::size_t(layout.surface_latitudes);
        put_floats(blob, profile, layout.upper);
        put_floats(blob, profile, layout.lower);
        put_floats(blob, profile, layout.saturation);
        put_floats(blob, plane, layout.land);
        put_floats(blob, plane * std::size_t(layout.months), layout.sea);
        blob.insert(blob.end(), layout.provenance.begin(), layout.provenance.end());
        return blob;
    }

    /**
     * @brief Finds the shipped asset by walking up from the working directory.
     *
     * CTest runs from the build tree and a developer runs the binary from anywhere, so the
     * relative path the engine uses is not reliable here. Empty when it genuinely is not
     * checked out.
     */
    std::string locate_shipped_asset()
    {
        std::string prefix;
        for (int depth = 0; depth < 6; ++depth)
        {
            const std::string candidate = prefix + Simulation::CLIMATOLOGY_ASSET_PATH;
            std::ifstream probe(candidate, std::ios::binary);
            if (probe)
                return candidate;
            prefix += "../";
        }
        return {};
    }
} // namespace

// --------------------------------------------------------------------------------------
// The analytic path: not a fallback that fails, but the mean state a non-Earth body uses.
// --------------------------------------------------------------------------------------

TEST(Unit_Climatology, WithNoAssetTheAnalyticBandsAnswerAndSayThatTheyAre)
{
    const Climatology climatology;
    EXPECT_FALSE(climatology.baked());
    EXPECT_TRUE(climatology.provenance().empty());

    // The analytic jet is one Gaussian per hemisphere centred on the configured latitude, so
    // it must peak there and be symmetric about the equator.
    const AnalyticClimatology& bands = climatology.analytic_bands();
    const double peak = climatology.upper_zonal_wind_mps(bands.jet_latitude_radians, 0.0);
    EXPECT_NEAR(peak, bands.upper_jet_speed_mps, 0.2);
    EXPECT_NEAR(climatology.upper_zonal_wind_mps(-bands.jet_latitude_radians, 0.0), peak, 1e-9);
    EXPECT_LT(climatology.upper_zonal_wind_mps(0.0, 0.0), peak);

    // Water belongs in the tropics; that is what puts the rain there without a radiation
    // scheme having to be run to find out where the tropics are.
    EXPECT_NEAR(climatology.saturation_kg_per_m2(0.0, 0.0),
                bands.equatorial_saturation_kg_per_m2, 1e-9);
    EXPECT_LT(climatology.saturation_kg_per_m2(radians(80.0), 0.0),
              climatology.saturation_kg_per_m2(radians(20.0), 0.0));
}

TEST(Unit_Climatology, TheAnalyticPathHasNoSeasonAndNoContinents)
{
    const Climatology climatology;
    // Analytic bands are a statement about latitude alone. Answering 0 for land is truthful;
    // inventing a plausible coastline would not be.
    EXPECT_DOUBLE_EQ(climatology.land_fraction(at(40.0, 30.0)), 0.0);
    EXPECT_DOUBLE_EQ(climatology.upper_zonal_wind_mps(radians(45.0), 0.0),
                     climatology.upper_zonal_wind_mps(radians(45.0), 0.5));
    // But the surface temperature still has to answer something usable, so a consumer never
    // has to branch on whether an asset was loaded.
    EXPECT_GT(climatology.sea_surface_temperature_kelvin(at(0.0, 0.0), 0.0),
              climatology.sea_surface_temperature_kelvin(at(85.0, 0.0), 0.0));
}

// --------------------------------------------------------------------------------------
// The format: what it accepts, and -- more importantly -- what it refuses.
// --------------------------------------------------------------------------------------

TEST(Unit_Climatology, AWellFormedBlobIsAdoptedAndReadBack)
{
    BlobLayout layout;
    layout.upper = 42.0f;
    layout.land = 0.25f;
    layout.sea = 301.5f;
    layout.provenance = "NCEP-NCAR, for the sake of argument";

    Climatology climatology;
    ASSERT_TRUE(climatology.adopt(build(layout)));
    EXPECT_TRUE(climatology.baked());
    EXPECT_EQ(climatology.provenance(), layout.provenance);
    EXPECT_NEAR(climatology.upper_zonal_wind_mps(radians(45.0), 0.3), 42.0, 1e-5);
    EXPECT_NEAR(climatology.land_fraction(at(10.0, 200.0)), 0.25, 1e-6);
    EXPECT_NEAR(climatology.sea_surface_temperature_kelvin(at(10.0, 200.0), 0.3), 301.5, 1e-4);
}

TEST(Unit_Climatology, ABlobThatIsNotWhatItSaysItIsGetsRefused)
{
    // Each case breaks exactly one thing about an otherwise valid blob. A half-understood
    // climatology is a mean state nobody chose, and the weather grown on it would be wrong in
    // a way that looks like physics -- so every one of these must be a refusal, not a guess.
    {
        BlobLayout layout;
        layout.magic[3] = '1';
        Climatology climatology;
        EXPECT_FALSE(climatology.adopt(build(layout))) << "bad magic";
    }
    {
        BlobLayout layout;
        layout.version = 2;
        Climatology climatology;
        EXPECT_FALSE(climatology.adopt(build(layout))) << "a version this reader does not know";
    }
    {
        BlobLayout layout;
        layout.bands = 1; // a single band cannot be interpolated between
        Climatology climatology;
        EXPECT_FALSE(climatology.adopt(build(layout))) << "too few bands";
    }
    {
        BlobLayout layout;
        layout.months = 0; // used as a divisor
        Climatology climatology;
        EXPECT_FALSE(climatology.adopt(build(layout))) << "zero months";
    }
    {
        std::vector<std::uint8_t> blob = build();
        blob.resize(blob.size() / 2);
        Climatology climatology;
        EXPECT_FALSE(climatology.adopt(blob)) << "truncated payload";
    }
    {
        std::vector<std::uint8_t> blob = build();
        blob.push_back(0); // the provenance length no longer reaches the end
        Climatology climatology;
        EXPECT_FALSE(climatology.adopt(blob)) << "trailing bytes";
    }
    {
        Climatology climatology;
        EXPECT_FALSE(climatology.adopt({})) << "nothing at all";
    }
}

TEST(Unit_Climatology, ARefusedBlobLeavesTheWorkingClimatologyUntouched)
{
    // The promise `adopt` makes: it reads into locals and commits only once every check has
    // passed. A host that hot-reloads a bad asset must keep the weather it had.
    BlobLayout good;
    good.upper = 33.0f;
    Climatology climatology;
    ASSERT_TRUE(climatology.adopt(build(good)));
    const double before = climatology.upper_zonal_wind_mps(radians(45.0), 0.25);
    const std::string provenance = climatology.provenance();

    std::vector<std::uint8_t> truncated = build(good);
    truncated.resize(truncated.size() / 3);
    EXPECT_FALSE(climatology.adopt(truncated));

    EXPECT_TRUE(climatology.baked());
    EXPECT_EQ(climatology.provenance(), provenance);
    EXPECT_DOUBLE_EQ(climatology.upper_zonal_wind_mps(radians(45.0), 0.25), before);
}

TEST(Unit_Climatology, TheYearWrapsRatherThanEndingAtAWall)
{
    // December must interpolate into January. A scene played across new year would otherwise
    // step to a different jet between two frames.
    BlobLayout layout;
    layout.bands = 4;
    layout.months = 12;
    std::vector<std::uint8_t> blob = build(layout);

    // Rewrite the upper-wind profile so each month carries a different value; without that,
    // every interpolation returns the same number and the test could not fail.
    const std::size_t profile = std::size_t(layout.bands) * std::size_t(layout.months);
    for (std::size_t i = 0; i < profile; ++i)
    {
        const float value = 10.0f + float(i / std::size_t(layout.bands));
        std::memcpy(blob.data() + HEADER_BYTES + i * sizeof(float), &value, sizeof(float));
    }

    Climatology climatology;
    ASSERT_TRUE(climatology.adopt(blob));
    const double latitude = radians(10.0);

    // Continuity, not equality: the two samples are a real (if tiny) interval apart, and the
    // profile steps 1 m/s per month, so the honest bound is the interval times that slope
    // rather than an epsilon. A December that clamped instead of wrapping would land 11 m/s
    // away -- four orders above this -- so the bound does not have to be tight to catch it.
    const double gap = 1.0e-6;
    const double before_wrap = climatology.upper_zonal_wind_mps(latitude, 1.0 - gap);
    const double after_wrap = climatology.upper_zonal_wind_mps(latitude, 0.0);
    EXPECT_NEAR(before_wrap, after_wrap, gap * 12.0 * 11.0 + 1e-9);
    EXPECT_NEAR(before_wrap, 15.5, 1e-3) << "halfway between December's 21 and January's 10";

    // And a year fraction outside [0, 1) is the same instant one year on, not a clamp.
    EXPECT_NEAR(climatology.upper_zonal_wind_mps(latitude, 1.25),
                climatology.upper_zonal_wind_mps(latitude, 0.25), 1e-9);
    EXPECT_NEAR(climatology.upper_zonal_wind_mps(latitude, -0.75),
                climatology.upper_zonal_wind_mps(latitude, 0.25), 1e-9);
}

TEST(Unit_Climatology, LatitudeClampsAtThePolesInsteadOfFoldingAcrossThem)
{
    // There is nothing past a pole to interpolate toward, and folding across one would blend
    // a band with itself.
    Climatology climatology;
    ASSERT_TRUE(climatology.adopt(build()));
    const double pole = climatology.upper_zonal_wind_mps(0.5 * PI, 0.0);
    EXPECT_NEAR(climatology.upper_zonal_wind_mps(0.5 * PI + 1.0, 0.0), pole, 1e-9);
    EXPECT_TRUE(std::isfinite(climatology.saturation_kg_per_m2(-0.5 * PI, 0.0)));
}

// --------------------------------------------------------------------------------------
// The data: what the shipped asset claims about the actual Earth.
// --------------------------------------------------------------------------------------

TEST(Unit_Climatology, TheShippedAssetPutsTheJetsWhereTheReanalysisDoes)
{
    const std::string path = locate_shipped_asset();
    if (path.empty())
        GTEST_SKIP() << "no baked climatology checked out; run `se climatology bake`";

    const Climatology climatology = Simulation::load_climatology(path);
    ASSERT_TRUE(climatology.baked());
    EXPECT_FALSE(climatology.provenance().empty()) << "attribution must travel in the asset";

    // Annual-mean upper wind, per hemisphere. The reanalysis puts the northern jet near 37N
    // and the southern near 49S; a bake that regridded or flipped a latitude axis would move
    // them, and moving them is the failure that would otherwise be invisible.
    for (int hemisphere = 0; hemisphere < 2; ++hemisphere)
    {
        double best = -1.0e9;
        double best_latitude = 0.0;
        for (int degree = 5; degree < 89; ++degree)
        {
            const double latitude = radians(hemisphere ? double(degree) : -double(degree));
            double sum = 0.0;
            for (int month = 0; month < 12; ++month)
                sum += climatology.upper_zonal_wind_mps(latitude, (month + 0.5) / 12.0);
            if (sum / 12.0 > best)
            {
                best = sum / 12.0;
                best_latitude = hemisphere ? double(degree) : -double(degree);
            }
        }
        EXPECT_GT(best, 15.0) << "a mid-latitude jet, not a breeze";
        EXPECT_LT(best, 60.0) << "a zonal *mean*, not a jet streak";
        EXPECT_GT(std::fabs(best_latitude), 25.0);
        EXPECT_LT(std::fabs(best_latitude), 60.0);
    }

    // The shear is what decides whether the mean state makes storms at all, so it is worth
    // its own bound rather than being left implied by the two profiles.
    double peak_shear = 0.0;
    for (int degree = -89; degree <= 89; ++degree)
    {
        const double latitude = radians(double(degree));
        peak_shear = std::max(peak_shear,
                              std::fabs(climatology.upper_zonal_wind_mps(latitude, 0.0) -
                                        climatology.lower_zonal_wind_mps(latitude, 0.0)));
    }
    EXPECT_GT(peak_shear, 15.0) << "below this the flow is barely unstable and makes no weather";
    EXPECT_LT(peak_shear, 60.0);
}

TEST(Unit_Climatology, TheShippedAssetPutsTheWaterInTheTropics)
{
    const std::string path = locate_shipped_asset();
    if (path.empty())
        GTEST_SKIP() << "no baked climatology checked out; run `se climatology bake`";

    const Climatology climatology = Simulation::load_climatology(path);
    ASSERT_TRUE(climatology.baked());

    const double tropics = climatology.saturation_kg_per_m2(radians(5.0), 0.5);
    const double midlatitude = climatology.saturation_kg_per_m2(radians(45.0), 0.5);
    const double pole = climatology.saturation_kg_per_m2(radians(85.0), 0.5);
    EXPECT_GT(tropics, midlatitude);
    EXPECT_GT(midlatitude, pole);
    // A saturated tropical column holds tens of kg/m^2, not hundreds and not units. This is
    // the ceiling the core's condensation scheme measures against, so a wrong exponent here
    // would make the world either a desert or a permanent storm.
    EXPECT_GT(tropics, 40.0);
    EXPECT_LT(tropics, 120.0);
}

TEST(Unit_Climatology, TheShippedAssetKnowsWhereTheContinentsAre)
{
    const std::string path = locate_shipped_asset();
    if (path.empty())
        GTEST_SKIP() << "no baked climatology checked out; run `se climatology bake`";

    const Climatology climatology = Simulation::load_climatology(path);
    ASSERT_TRUE(climatology.baked());

    // Longitudes east of the prime meridian, which is how the asset indexes them -- a mask
    // that had been rolled the wrong way would put the Sahara in the Atlantic and still look
    // like a plausible planet.
    struct Place { const char* name; double latitude; double longitude; bool land; };
    const Place places[] = {
        {"Sahara", 25.0, 15.0, true},
        {"Amazon", -5.0, 300.0, true},
        {"Siberia", 62.0, 100.0, true},
        {"Antarctica", -80.0, 0.0, true},
        {"mid-Pacific", 0.0, 200.0, false},
        {"mid-Atlantic", 30.0, 320.0, false},
        {"Southern Ocean", -55.0, 90.0, false},
    };
    for (const Place& place : places)
    {
        const double fraction = climatology.land_fraction(at(place.latitude, place.longitude));
        EXPECT_GE(fraction, 0.0) << place.name;
        EXPECT_LE(fraction, 1.0) << place.name;
        if (place.land)
            EXPECT_GT(fraction, 0.5) << place.name << " should be land";
        else
            EXPECT_LT(fraction, 0.5) << place.name << " should be ocean";
    }
}

TEST(Unit_Climatology, TheShippedAssetsSeaSurfaceIsColdAtThePolesEvenUnderTheIce)
{
    const std::string path = locate_shipped_asset();
    if (path.empty())
        GTEST_SKIP() << "no baked climatology checked out; run `se climatology bake`";

    const Climatology climatology = Simulation::load_climatology(path);
    ASSERT_TRUE(climatology.baked());

    // 80S is land at every longitude, so every cell there is *filled* rather than measured.
    // An earlier bake filled it from the global ocean mean and wrote 287 K under the ice
    // sheet; this is the regression that catches that returning.
    const double antarctic = climatology.sea_surface_temperature_kelvin(at(-80.0, 0.0), 0.5);
    EXPECT_LT(antarctic, 278.0) << "a filled polar cell must take its own latitude's water";
    EXPECT_GT(antarctic, 265.0) << "and still be sea water, not an absolute zero sentinel";

    const double equator = climatology.sea_surface_temperature_kelvin(at(0.0, 200.0), 0.5);
    EXPECT_GT(equator, 295.0);
    EXPECT_LT(equator, 310.0);
    EXPECT_GT(equator, antarctic + 15.0);
}
