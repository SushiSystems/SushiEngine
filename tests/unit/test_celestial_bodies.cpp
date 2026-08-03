/**************************************************************************/
/* test_celestial_bodies.cpp                                             */
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

// Unit_CelestialBodies: the body catalogue and the positions it generates. The planets
// must land near their real mean distances from the Sun at J2000, the Moon near its 60
// Earth-radii orbit, Earth's pole along the equatorial +Z, and every physical property
// must stay in its valid range. These are the values the whole sky is assembled from.

#include <cmath>

#include <gtest/gtest.h>

#include <SushiEngine/astro/celestial_bodies.hpp>
#include <SushiEngine/astro/julian_date.hpp>
#include <SushiEngine/astro/orbital_elements.hpp>

#include "test_helpers.hpp"

using namespace SushiEngine;
using namespace SushiEngine::Astro;

TEST(Unit_CelestialBodies, CatalogueHasElevenBodies)
{
    EXPECT_EQ(BODY_COUNT, 11);
    EXPECT_EQ(static_cast<int>(BodyId::Sun), 0);
    EXPECT_EQ(static_cast<int>(BodyId::Earth), 3);
    EXPECT_EQ(static_cast<int>(BodyId::Moon), 4);
}

TEST(Unit_CelestialBodies, PhysicalPropertiesAreInRange)
{
    for (int index = 0; index < BODY_COUNT; ++index)
    {
        const BodyProperties p = body_properties(static_cast<BodyId>(index));
        EXPECT_GT(p.mean_radius_metres, 0.0) << p.name;
        EXPECT_GE(p.geometric_albedo, 0.0) << p.name;
        EXPECT_LE(p.geometric_albedo, 1.0) << p.name;
    }
    EXPECT_TRUE(body_properties(BodyId::Sun).is_star);
    EXPECT_FALSE(body_properties(BodyId::Earth).is_star);
    EXPECT_NEAR(body_properties(BodyId::Earth).mean_radius_metres, 6.371e6, 1e3);
}

TEST(Unit_CelestialBodies, EarthSitsAboutOneAstronomicalUnitFromTheSun)
{
    const Vector3 earth = planet_heliocentric_au(BodyId::Earth, J2000_JULIAN_DATE);
    EXPECT_NEAR(length(earth), 1.0, 0.02);
}

TEST(Unit_CelestialBodies, PlanetsSitNearTheirMeanDistances)
{
    struct Expectation
    {
        BodyId body;
        double low;
        double high;
    };
    // Perihelion..aphelion brackets so the eccentric orbits still pass at any epoch.
    const Expectation table[] = {
        {BodyId::Mercury, 0.30, 0.47}, {BodyId::Venus, 0.71, 0.74},
        {BodyId::Mars, 1.38, 1.67},    {BodyId::Jupiter, 4.95, 5.46},
        {BodyId::Saturn, 9.01, 10.07}, {BodyId::Neptune, 29.7, 30.4},
    };
    for (const Expectation& e : table)
    {
        const double r = length(planet_heliocentric_au(e.body, J2000_JULIAN_DATE));
        EXPECT_GE(r, e.low) << body_properties(e.body).name;
        EXPECT_LE(r, e.high) << body_properties(e.body).name;
    }
}

TEST(Unit_CelestialBodies, MoonOrbitsAtRoughlySixtyEarthRadii)
{
    // Mean Earth-Moon distance is ~384400 km ~ 0.00257 AU; the abridged series stays
    // within a few percent of that across a lunation.
    for (double day = 0.0; day < 30.0; day += 2.0)
    {
        const Vector3 moon = moon_geocentric_ecliptic_au(J2000_JULIAN_DATE + day);
        const double au = length(moon);
        EXPECT_GT(au, 0.0022) << "day " << day;
        EXPECT_LT(au, 0.0028) << "day " << day;
    }
}

TEST(Unit_CelestialBodies, NorthPolesAreUnitAndEarthPointsUp)
{
    for (int index = 0; index < BODY_COUNT; ++index)
        EXPECT_TRUE(Harness::is_unit(body_north_pole_equatorial(static_cast<BodyId>(index)), 1e-9));

    // Earth's pole is the equatorial +Z axis by definition (RA 0, Dec 90).
    const Vector3 earth_pole = body_north_pole_equatorial(BodyId::Earth);
    EXPECT_TRUE(Harness::approx_equal(earth_pole, Vector3{0.0, 0.0, 1.0}, 1e-9));
}

TEST(Unit_CelestialBodies, EarthKeplerRowIsOneAstronomicalUnit)
{
    EXPECT_NEAR(keplerian_elements_for(BodyId::Earth).semi_major_axis_au, 1.0, 1e-3);
    // The Sun and Moon are not propagated as heliocentric planets: a zeroed row.
    EXPECT_DOUBLE_EQ(keplerian_elements_for(BodyId::Sun).semi_major_axis_au, 0.0);
    EXPECT_DOUBLE_EQ(keplerian_elements_for(BodyId::Moon).semi_major_axis_au, 0.0);
}

TEST(Unit_CelestialBodies, SurfacePresetsDescribeEarthAsWgs84AndSunAsUnlandable)
{
    const SurfacePreset earth = surface_preset(BodyId::Earth);
    EXPECT_TRUE(earth.landable);
    EXPECT_TRUE(earth.has_atmosphere);
    EXPECT_NEAR(earth.semi_major_metres, 6378137.0, 1.0);
    EXPECT_NEAR(earth.inverse_flattening, 298.257223563, 1e-6);

    // The Sun falls through to the default preset: not a surface you land on.
    EXPECT_FALSE(surface_preset(BodyId::Sun).landable);

    // Airless bodies carry no atmosphere shell.
    EXPECT_FALSE(surface_preset(BodyId::Mercury).has_atmosphere);
    EXPECT_FALSE(surface_preset(BodyId::Moon).has_atmosphere);
}
