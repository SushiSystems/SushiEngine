/**************************************************************************/
/* test_reference_frame.cpp                                              */
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

// Unit_ReferenceFrame: the body-centred inertial frames a free entity is stored in. The
// Sun's frame is the heliocentric frame itself (a zero shift); a state must survive a
// round trip through any body frame and back; a rebase between two frames must be exactly
// invertible; and the active-frame selector must pick the tightest sphere of influence
// containing a point (Earth near Earth, the Sun in deep space).

#include <cmath>

#include <gtest/gtest.h>

#include <SushiEngine/astro/gravity.hpp>
#include <SushiEngine/astro/reference_frame.hpp>

#include "test_helpers.hpp"

using namespace SushiEngine;
using namespace SushiEngine::Astro;

TEST(Unit_ReferenceFrame, SunFrameIsAZeroShift)
{
    const ReferenceFrame sun = frame_for(BodyId::Sun, J2000_JULIAN_DATE);
    EXPECT_EQ(sun.body, BodyId::Sun);
    EXPECT_NEAR(sun.center.x, 0.0, 1e-6);
    EXPECT_NEAR(sun.center.y, 0.0, 1e-6);
    EXPECT_NEAR(sun.center.z, 0.0, 1e-6);
    EXPECT_NEAR(sun.center_velocity.x, 0.0, 1e-9);
    EXPECT_NEAR(sun.center_velocity.y, 0.0, 1e-9);
    EXPECT_NEAR(sun.center_velocity.z, 0.0, 1e-9);
}

TEST(Unit_ReferenceFrame, EarthFrameRidesTheEphemeris)
{
    const ReferenceFrame earth = frame_for(BodyId::Earth, J2000_JULIAN_DATE);
    const WorldVector3 truth = body_heliocentric_metres(BodyId::Earth, J2000_JULIAN_DATE);
    EXPECT_TRUE(Harness::approx_equal(earth.center, truth, 1e-3));
    // Earth's orbital speed is ~29.8 km/s; the finite-difference velocity recovers it.
    const double speed = std::sqrt(earth.center_velocity.x * earth.center_velocity.x +
                                   earth.center_velocity.y * earth.center_velocity.y +
                                   earth.center_velocity.z * earth.center_velocity.z);
    EXPECT_NEAR(speed, 29800.0, 2000.0);
}

TEST(Unit_ReferenceFrame, StateRoundTripsThroughABodyFrame)
{
    const ReferenceFrame earth = frame_for(BodyId::Earth, J2000_JULIAN_DATE);
    const StateVector helio{WorldVector3{1.6e11, 2.0e9, -5.0e8},
                            WorldVector3{1.0e3, 2.9e4, 5.0e2}};
    const StateVector local = to_frame(helio, earth);
    const StateVector back = to_heliocentric(local, earth);
    EXPECT_TRUE(Harness::approx_equal(back.position, helio.position, 1e-3));
    EXPECT_TRUE(Harness::approx_equal(back.velocity, helio.velocity, 1e-6));
}

TEST(Unit_ReferenceFrame, RebaseIsIdentityWithinAFrameAndInvertible)
{
    const ReferenceFrame earth = frame_for(BodyId::Earth, J2000_JULIAN_DATE);
    const ReferenceFrame mars = frame_for(BodyId::Mars, J2000_JULIAN_DATE);
    const StateVector local{WorldVector3{1.0e7, -2.0e6, 3.0e6}, WorldVector3{100.0, 200.0, -50.0}};

    // Rebasing into the same frame changes nothing.
    const StateVector same = rebase(local, earth, earth);
    EXPECT_TRUE(Harness::approx_equal(same.position, local.position, 1e-3));
    EXPECT_TRUE(Harness::approx_equal(same.velocity, local.velocity, 1e-6));

    // Rebasing to Mars and back to Earth is the identity.
    const StateVector round = rebase(rebase(local, earth, mars), mars, earth);
    EXPECT_TRUE(Harness::approx_equal(round.position, local.position, 1e-3));
    EXPECT_TRUE(Harness::approx_equal(round.velocity, local.velocity, 1e-6));
}

TEST(Unit_ReferenceFrame, ActiveFrameIsEarthNearEarthAndSunInDeepSpace)
{
    const WorldVector3 earth = body_heliocentric_metres(BodyId::Earth, J2000_JULIAN_DATE);

    // A point 10000 km from Earth's centre is inside Earth's sphere of influence.
    const WorldVector3 near_earth{earth.x + 1.0e7, earth.y, earth.z};
    EXPECT_EQ(active_frame_body(near_earth, J2000_JULIAN_DATE), BodyId::Earth);

    // A point 66 AU above the ecliptic plane is outside every planet's SOI: the Sun.
    const WorldVector3 deep_space{0.0, 0.0, 1.0e13};
    EXPECT_EQ(active_frame_body(deep_space, J2000_JULIAN_DATE), BodyId::Sun);
}
