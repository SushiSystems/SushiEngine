/**************************************************************************/
/* test_body_orientation.cpp                                             */
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

// Unit_BodyOrientation: a body's spin and its own equatorial frame. Earth must stay
// bit-identical to the fixed-obliquity path (so the home sky never shifts); every other
// body's ecliptic<->equatorial rotation must invert exactly and carry its pole to +Z;
// the prime-meridian angle must stay wrapped and advance one turn per rotation period.

#include <cmath>

#include <gtest/gtest.h>

#include <SushiEngine/astro/body_orientation.hpp>
#include <SushiEngine/astro/celestial_bodies.hpp>
#include <SushiEngine/astro/julian_date.hpp>
#include <SushiEngine/astro/orbital_elements.hpp>

#include "test_helpers.hpp"

using namespace SushiEngine;
using namespace SushiEngine::Astro;

TEST(Unit_BodyOrientation, RotationAngleStaysWrappedAndAdvances)
{
    constexpr double two_pi = 6.283185307179586;
    for (int index = 0; index < BODY_COUNT; ++index)
    {
        const BodyId body = static_cast<BodyId>(index);
        for (double day = 0.0; day < 3.0; day += 0.31)
        {
            const double w = body_rotation_angle(body, J2000_JULIAN_DATE + day);
            EXPECT_GE(w, 0.0);
            EXPECT_LT(w, two_pi);
        }
    }
    // Earth advances one full turn per solar day (~360.986 deg), so consecutive days
    // differ by ~0.986 deg of residual after wrapping.
    const double a = body_rotation_angle(BodyId::Earth, J2000_JULIAN_DATE);
    const double b = body_rotation_angle(BodyId::Earth, J2000_JULIAN_DATE + 1.0);
    EXPECT_GT(std::fabs(a - b), 0.0);
}

TEST(Unit_BodyOrientation, EarthEquatorialConversionMatchesFixedObliquity)
{
    // Earth is deliberately routed through the exact fixed-obliquity conversion so the
    // home sky is unchanged from the original path.
    const Vector3 v{0.4, -0.6, 0.7};
    EXPECT_TRUE(Harness::approx_equal(ecliptic_to_body_equatorial(BodyId::Earth, v),
                                      ecliptic_to_equatorial(v), 1e-15));
    EXPECT_TRUE(Harness::approx_equal(body_equatorial_to_ecliptic(BodyId::Earth, v),
                                      equatorial_to_ecliptic(v), 1e-15));
}

TEST(Unit_BodyOrientation, EclipticBodyEquatorialRoundTripsForEveryBody)
{
    const Vector3 v{0.2, 0.5, -0.8};
    for (int index = 0; index < BODY_COUNT; ++index)
    {
        const BodyId body = static_cast<BodyId>(index);
        const Vector3 round =
            body_equatorial_to_ecliptic(body, ecliptic_to_body_equatorial(body, v));
        EXPECT_TRUE(Harness::approx_equal(round, v, 1e-12))
            << "body " << body_properties(body).name;
    }
}

TEST(Unit_BodyOrientation, PoleMapsToTheBodyEquatorialZAxis)
{
    for (int index = 0; index < BODY_COUNT; ++index)
    {
        const BodyId body = static_cast<BodyId>(index);
        if (body == BodyId::Earth)
            continue; // Earth uses the fixed-obliquity frame, tested above
        const Vector3 pole_ecliptic = body_pole_ecliptic(body);
        EXPECT_TRUE(Harness::is_unit(pole_ecliptic, 1e-9));
        const Vector3 pole_in_frame = ecliptic_to_body_equatorial(body, pole_ecliptic);
        EXPECT_TRUE(Harness::approx_equal(pole_in_frame, Vector3{0.0, 0.0, 1.0}, 1e-9))
            << "body " << body_properties(body).name;
    }
}

TEST(Unit_BodyOrientation, EquatorialToBodyEquatorialIsIdentityOnEarthAndPreservesLength)
{
    const Vector3 v{0.6, -0.3, 0.74};
    EXPECT_TRUE(Harness::approx_equal(equatorial_to_body_equatorial(BodyId::Earth, v), v, 1e-15));

    // On another body it is a pure rotation: length is preserved.
    const Vector3 mars = equatorial_to_body_equatorial(BodyId::Mars, v);
    EXPECT_NEAR(length(mars), length(v), 1e-12);
}
