/**************************************************************************/
/* test_scene_frame.cpp                                                  */
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

// Unit_SceneFrame: the rigid isometry between heliocentric-ecliptic metres and the
// scene's body-anchored local frame. Being a rigid isometry, it must round-trip exactly
// both ways and preserve distances; the observer body's centre must map to the scene
// offset the ephemeris uses; and the construction must be body-parametric (Earth and
// Mars alike).

#include <cmath>

#include <gtest/gtest.h>

#include <SushiEngine/astro/scene_frame.hpp>

#include "test_helpers.hpp"

using namespace SushiEngine;
using namespace SushiEngine::Astro;

namespace
{
    double distance(const WorldVector3& a, const WorldVector3& b)
    {
        const double dx = a.x - b.x;
        const double dy = a.y - b.y;
        const double dz = a.z - b.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }
}

TEST(Unit_SceneFrame, PositionRoundTripsBothWays)
{
    const SceneFrame frame = scene_frame_for(J2000_JULIAN_DATE + 100.0, 0.7, 0.3, BodyId::Earth);

    const WorldVector3 helio{frame.observer_heliocentric.x + 5.0e6,
                             frame.observer_heliocentric.y - 2.0e6,
                             frame.observer_heliocentric.z + 1.0e6};
    const WorldVector3 scene = frame.scene_from_heliocentric(helio);
    const WorldVector3 back = frame.heliocentric_from_scene(scene);
    EXPECT_TRUE(Harness::approx_equal(back, helio, 1e-4));

    // And starting from a scene point.
    const WorldVector3 scene_point{1234.0, -5678.0, 9012.0};
    const WorldVector3 helio_point = frame.heliocentric_from_scene(scene_point);
    EXPECT_TRUE(Harness::approx_equal(frame.scene_from_heliocentric(helio_point), scene_point, 1e-4));
}

TEST(Unit_SceneFrame, MapIsAnIsometryPreservingDistance)
{
    const SceneFrame frame = scene_frame_for(J2000_JULIAN_DATE, -0.4, 1.2, BodyId::Earth);
    const WorldVector3 a{frame.observer_heliocentric.x + 1.0e7,
                         frame.observer_heliocentric.y, frame.observer_heliocentric.z};
    const WorldVector3 b{frame.observer_heliocentric.x, frame.observer_heliocentric.y + 3.0e6,
                         frame.observer_heliocentric.z - 4.0e6};
    EXPECT_NEAR(distance(frame.scene_from_heliocentric(a), frame.scene_from_heliocentric(b)),
                distance(a, b), 1e-3);
}

TEST(Unit_SceneFrame, DirectionRoundTripsAndIsRotationOnly)
{
    const SceneFrame frame = scene_frame_for(J2000_JULIAN_DATE, 0.5, -0.9, BodyId::Earth);
    const Vector3 dir{0.3, -0.6, 0.74};
    const Vector3 back = frame.direction_to_heliocentric(frame.direction_to_scene(dir));
    EXPECT_TRUE(Harness::approx_equal(back, dir, 1e-12));
    // A direction carries no translation: its length is preserved.
    EXPECT_NEAR(length(frame.direction_to_scene(dir)), length(dir), 1e-12);
}

TEST(Unit_SceneFrame, ObserverBodyCentreMapsToTheSceneOffset)
{
    const SceneFrame frame = scene_frame_for(J2000_JULIAN_DATE, 0.7, 0.3, BodyId::Earth);
    // The body centre (observer_heliocentric) sits at observer_center in the scene.
    const WorldVector3 centre = frame.scene_from_heliocentric(frame.observer_heliocentric);
    EXPECT_TRUE(Harness::approx_equal(centre, frame.observer_center, 1e-6));

    // The scene origin is the observer's surface point: one body radius from the centre.
    const WorldVector3 origin_helio = frame.heliocentric_from_scene(WorldVector3{0.0, 0.0, 0.0});
    EXPECT_NEAR(distance(origin_helio, frame.observer_heliocentric),
                length(Vector3{frame.observer_center.x, frame.observer_center.y,
                               frame.observer_center.z}),
                1e-3);
}

TEST(Unit_SceneFrame, ConstructionIsBodyParametric)
{
    // The same round-trip guarantee holds for an observer standing on Mars.
    const SceneFrame frame = scene_frame_for(J2000_JULIAN_DATE + 50.0, 0.2, 0.6, BodyId::Mars);
    const WorldVector3 helio{frame.observer_heliocentric.x + 2.0e6,
                             frame.observer_heliocentric.y + 3.0e6,
                             frame.observer_heliocentric.z - 1.0e6};
    const WorldVector3 back = frame.heliocentric_from_scene(frame.scene_from_heliocentric(helio));
    EXPECT_TRUE(Harness::approx_equal(back, helio, 1e-4));
}
