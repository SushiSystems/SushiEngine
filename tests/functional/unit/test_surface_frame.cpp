/**************************************************************************/
/* test_surface_frame.cpp                                                */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

// Unit_SurfaceFrame: the body-fixed geodetic surface frame on the WGS84 ellipsoid. The
// geodetic<->Cartesian map must place the equator at the semi-major radius and the pole
// at the semi-minor radius and invert (Bowring) to sub-metre accuracy; the ellipsoid
// normal and ENU tangent basis must be orthonormal and correctly oriented; and the
// surface gravity must recover ~9.8 m/s^2 for Earth and point inward.

#include <cmath>

#include <gtest/gtest.h>

#include <SushiEngine/astro/celestial_bodies.hpp>
#include <SushiEngine/astro/surface_frame.hpp>

#include "test_helpers.hpp"

using namespace SushiEngine;
using namespace SushiEngine::Astro;

namespace
{
    constexpr double PI = 3.141592653589793;
    constexpr double WGS84_A = 6378137.0;
    constexpr double WGS84_B = 6356752.314245; // a * sqrt(1 - e^2)
}

TEST(Unit_SurfaceFrame, EccentricitySquaredMatchesFlattening)
{
    const double f = 1.0 / 298.257223563;
    EXPECT_NEAR(ellipsoid_eccentricity_squared(BodyId::Earth), f * (2.0 - f), 1e-15);
    // A body modelled as a sphere has zero eccentricity.
    EXPECT_DOUBLE_EQ(ellipsoid_eccentricity_squared(BodyId::Moon), 0.0);
}

TEST(Unit_SurfaceFrame, EquatorSitsAtSemiMajorAndPoleAtSemiMinor)
{
    const Vector3 equator =
        geodetic_to_body_fixed(BodyId::Earth, GeodeticCoordinate{0.0, 0.0, 0.0});
    EXPECT_NEAR(equator.x, WGS84_A, 1e-3);
    EXPECT_NEAR(equator.y, 0.0, 1e-6);
    EXPECT_NEAR(equator.z, 0.0, 1e-6);

    const Vector3 pole =
        geodetic_to_body_fixed(BodyId::Earth, GeodeticCoordinate{PI / 2.0, 0.0, 0.0});
    EXPECT_NEAR(pole.x, 0.0, 1e-3);
    EXPECT_NEAR(pole.y, 0.0, 1e-3);
    EXPECT_NEAR(pole.z, WGS84_B, 1e-3);
}

TEST(Unit_SurfaceFrame, LongitudePlacesThePointAroundTheEquator)
{
    const Vector3 east90 =
        geodetic_to_body_fixed(BodyId::Earth, GeodeticCoordinate{0.0, PI / 2.0, 0.0});
    // 90 deg east on the equator is +Y.
    EXPECT_NEAR(east90.x, 0.0, 1e-3);
    EXPECT_NEAR(east90.y, WGS84_A, 1e-3);
    EXPECT_NEAR(east90.z, 0.0, 1e-6);
}

TEST(Unit_SurfaceFrame, GeodeticRoundTripsThroughCartesian)
{
    const GeodeticCoordinate samples[] = {
        {0.6, 0.4, 0.0},      {-0.9, -2.1, 1200.0}, {1.2, 3.0, -300.0},
        {0.0, 0.0, 8848.0},   {1.55, 1.0, 500.0},
    };
    for (const GeodeticCoordinate& g : samples)
    {
        const Vector3 ecef = geodetic_to_body_fixed(BodyId::Earth, g);
        const GeodeticCoordinate back = body_fixed_to_geodetic(BodyId::Earth, ecef);
        EXPECT_NEAR(back.latitude_radians, g.latitude_radians, 1e-9);
        EXPECT_NEAR(back.longitude_radians, g.longitude_radians, 1e-9);
        EXPECT_NEAR(back.altitude_metres, g.altitude_metres, 1e-3);
    }
}

TEST(Unit_SurfaceFrame, SphericalBodyRoundTripsAndNormalIsRadial)
{
    // The Moon is modelled as a sphere: the normal is exactly the radial direction.
    const GeodeticCoordinate g{0.7, -1.1, 400.0};
    const Vector3 ecef = geodetic_to_body_fixed(BodyId::Moon, g);
    const GeodeticCoordinate back = body_fixed_to_geodetic(BodyId::Moon, ecef);
    EXPECT_NEAR(back.latitude_radians, g.latitude_radians, 1e-9);
    EXPECT_NEAR(back.altitude_metres, g.altitude_metres, 1e-3);

    const Vector3 normal = geodetic_normal(BodyId::Moon, ecef);
    EXPECT_TRUE(Harness::approx_equal(normal, normalize(ecef), 1e-12));
}

TEST(Unit_SurfaceFrame, GeodeticNormalIsOutwardAtLandmarks)
{
    const Vector3 equator = geodetic_to_body_fixed(BodyId::Earth, GeodeticCoordinate{0.0, 0.0, 0.0});
    EXPECT_TRUE(Harness::approx_equal(geodetic_normal(BodyId::Earth, equator),
                                      Vector3{1.0, 0.0, 0.0}, 1e-9));

    const Vector3 pole =
        geodetic_to_body_fixed(BodyId::Earth, GeodeticCoordinate{PI / 2.0, 0.0, 0.0});
    EXPECT_TRUE(Harness::approx_equal(geodetic_normal(BodyId::Earth, pole),
                                      Vector3{0.0, 0.0, 1.0}, 1e-9));
}

TEST(Unit_SurfaceFrame, TangentBasisIsRightHandedOrthonormal)
{
    const GeodeticCoordinate samples[] = {
        {0.6, 0.4, 0.0}, {-0.9, 2.1, 0.0}, {0.0, -1.0, 0.0}, {1.4, 0.2, 0.0},
    };
    for (const GeodeticCoordinate& g : samples)
    {
        const Vector3 ecef = geodetic_to_body_fixed(BodyId::Earth, g);
        const TangentBasis basis = local_tangent_basis(BodyId::Earth, ecef);
        EXPECT_TRUE(Harness::is_orthonormal_basis(basis.east, basis.north, basis.up, 1e-9))
            << "lat=" << g.latitude_radians << " lon=" << g.longitude_radians;
        // Up agrees with the ellipsoid normal.
        EXPECT_TRUE(Harness::approx_equal(basis.up, geodetic_normal(BodyId::Earth, ecef), 1e-12));
    }
}

TEST(Unit_SurfaceFrame, SurfaceGravityRecoversEarthsValueAndPointsInward)
{
    EXPECT_NEAR(surface_gravity(BodyId::Earth), 9.798, 0.02);
    EXPECT_LT(surface_gravity(BodyId::Moon), surface_gravity(BodyId::Earth));

    const Vector3 equator = geodetic_to_body_fixed(BodyId::Earth, GeodeticCoordinate{0.0, 0.0, 0.0});
    const Vector3 g = surface_gravity_vector(BodyId::Earth, equator);
    // Points back toward the centre: antiparallel to the outward normal.
    EXPECT_LT(g.x, 0.0);
    EXPECT_NEAR(length(g), surface_gravity(BodyId::Earth), 1e-6);
}
