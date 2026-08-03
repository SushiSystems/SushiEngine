/**************************************************************************/
/* test_terrain_frame.cpp                                                 */
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

// Unit_TerrainFrame: the camera's crossing into the body's own frame
// (docs/slop/solar_system_overhaul.md §7, §9).
//
// Three conventions meet in this one function and each of them is silently wrong in a way
// that still renders something: whether the matrix is stored by columns or by rows (a
// transposed rotation is a differently-oriented planet), which half-space a frustum plane
// keeps (an inverted plane culls exactly what should be drawn), and whether the camera
// offset is taken before or after the rotation. So the tests here check each against a
// property rather than against the numbers the implementation happens to produce: the
// camera round-trips through the matrix, a known point in front of the camera survives
// every plane, and the same point behind it does not.

#include <cmath>

#include <gtest/gtest.h>

#include <SushiEngine/astro/ephemeris.hpp>
#include <SushiEngine/astro/celestial_bodies.hpp>
#include <SushiEngine/astro/julian_date.hpp>

#include "terrain/terrain_frame.hpp"

using namespace SushiEngine;
using namespace SushiEngine::Render;

// Two namespaces are spelled Terrain -- the neutral one and the renderer's --
// so neither is ever written unqualified here. `tile_cache.hpp` makes the same
// point: the ambiguity is a compile error rather than a wrong type, which is the
// right side of the trade only if nobody tidies the qualification away.
namespace Field = SushiEngine::Terrain;
namespace RenderTerrain = SushiEngine::Render::Terrain;

namespace
{
    constexpr double DEGREE = 0.017453292519943295;

    /** An observer standing somewhere on the Moon, with the camera at the scene origin. */
    Environment lunar_sky(double latitude_degrees, double longitude_degrees)
    {
        Environment environment;
        environment.observer.observer_body = 4;
        environment.observer.latitude_radians = latitude_degrees * DEGREE;
        environment.observer.longitude_radians = longitude_degrees * DEGREE;
        environment.observer.julian_date = Astro::J2000_JULIAN_DATE + 120.0;
        Astro::fill_environment_sky(environment, WorldVector3{0.0, 0.0, 0.0});
        return environment;
    }

    /**
     * A camera looking along -Z with the given vertical half-angle. The view matrix is the
     * identity rotation, so its rows give right = +X, up = +Y, forward = -Z; the projection
     * carries only the two diagonal terms this code reads, Y-flipped as the engine's is.
     */
    CameraView camera_looking_down_negative_z(double tan_half_y, double aspect)
    {
        CameraView camera;
        for (int i = 0; i < 16; ++i)
            camera.view.m[i] = (i % 5 == 0) ? 1.0f : 0.0f;
        for (int i = 0; i < 16; ++i)
            camera.projection.m[i] = 0.0f;
        camera.projection.m[0] = static_cast<float>(1.0 / (tan_half_y * aspect));
        camera.projection.m[5] = static_cast<float>(-1.0 / tan_half_y);
        camera.near_plane = 0.5f;
        return camera;
    }

    /** Whether a point survives every plane; the selector's own test, at radius zero. */
    bool inside(const Field::FrustumPlanes& frustum, const Vector3& point)
    {
        for (int index = 0; index < 6; ++index)
        {
            const double* plane = frustum.plane[index];
            if (plane[0] * point.x + plane[1] * point.y + plane[2] * point.z + plane[3] < 0.0)
                return false;
        }
        return true;
    }

    /** Carries a body-fixed direction back to the scene through the packed matrix. */
    Vector3 through_matrix(const float m[16], const Vector3& body)
    {
        return Vector3{m[0] * body.x + m[4] * body.y + m[8] * body.z,
                       m[1] * body.x + m[5] * body.y + m[9] * body.z,
                       m[2] * body.x + m[6] * body.y + m[10] * body.z};
    }
} // namespace

TEST(Unit_TerrainFrame, TheCameraLandsOnTheObserversOwnSurfacePoint)
{
    // The camera sits at the scene origin, which is by construction the observer's surface
    // point. Crossing into the body's frame therefore has to produce that same point:
    // one lunar radius out, at the latitude and longitude the observer was placed at.
    const double latitude = 26.0;
    const double longitude = 18.0;
    const Environment environment = lunar_sky(latitude, longitude);

    const CameraView camera = camera_looking_down_negative_z(std::tan(0.5), 16.0 / 9.0);
    const double eye[3] = {0.0, 0.0, 0.0};
    Field::FrustumPlanes frustum;
    RenderTerrain::TerrainFrameView view;
    RenderTerrain::build_terrain_frame(camera, environment, eye, 1080u, 7u, 1u, frustum, view);

    const Vector3 position = view.camera_body_fixed;
    const double radius = Astro::surface_preset(Astro::BodyId::Moon).semi_major_metres;
    EXPECT_NEAR(length(position), radius, 1e-6);
    EXPECT_NEAR(std::asin(position.z / length(position)) / DEGREE, latitude, 1e-9);
    EXPECT_NEAR(std::atan2(position.y, position.x) / DEGREE, longitude, 1e-9);
}

TEST(Unit_TerrainFrame, TheMatrixIsColumnMajorAndUndoesTheCrossing)
{
    // Stored by columns is what mat3(body_to_scene) in terrain_common.glsl assumes. A
    // transposed rotation is still orthonormal and still looks like a planet, which is why
    // this is checked rather than trusted.
    const Environment environment = lunar_sky(-40.0, 122.0);
    const CameraView camera = camera_looking_down_negative_z(std::tan(0.4), 1.5);
    const double eye[3] = {1200.0, 340.0, -900.0};
    Field::FrustumPlanes frustum;
    RenderTerrain::TerrainFrameView view;
    RenderTerrain::build_terrain_frame(camera, environment, eye, 720u, 3u, 0u, frustum, view);

    const Vector3 back = through_matrix(view.body_to_scene, view.camera_body_fixed);
    const Vector3 expected{eye[0] - environment.planet_center.x,
                           eye[1] - environment.planet_center.y,
                           eye[2] - environment.planet_center.z};
    // Relative, not absolute: the matrix is float and the vector is a planetary radius.
    EXPECT_LT(length(back - expected) / length(expected), 1e-6);

    // The translation column is untouched, and the rotation is proper.
    EXPECT_FLOAT_EQ(view.body_to_scene[12], 0.0f);
    EXPECT_FLOAT_EQ(view.body_to_scene[13], 0.0f);
    EXPECT_FLOAT_EQ(view.body_to_scene[14], 0.0f);
    EXPECT_FLOAT_EQ(view.body_to_scene[15], 1.0f);
}

TEST(Unit_TerrainFrame, TheFrustumKeepsWhatIsInFrontAndDropsWhatIsBehind)
{
    const Environment environment = lunar_sky(0.0, 0.0);
    const double tan_half_y = std::tan(0.5);
    const double aspect = 16.0 / 9.0;
    const CameraView camera = camera_looking_down_negative_z(tan_half_y, aspect);
    const double eye[3] = {0.0, 0.0, 0.0};
    Field::FrustumPlanes frustum;
    RenderTerrain::TerrainFrameView view;
    RenderTerrain::build_terrain_frame(camera, environment, eye, 1080u, 0u, 0u, frustum, view);

    const Vector3* axes = environment.planet_body_axes;
    auto to_body = [&](const Vector3& scene)
    { return Vector3{dot(scene, axes[0]), dot(scene, axes[1]), dot(scene, axes[2])}; };

    // Straight ahead, well past the near plane: kept.
    EXPECT_TRUE(inside(frustum, to_body(Vector3{0.0, 0.0, -1000.0})));
    // Directly behind: dropped, and it is the near plane that drops it.
    EXPECT_FALSE(inside(frustum, to_body(Vector3{0.0, 0.0, 1000.0})));
    // Just inside the near plane, and just outside it.
    EXPECT_TRUE(inside(frustum, to_body(Vector3{0.0, 0.0, -0.6})));
    EXPECT_FALSE(inside(frustum, to_body(Vector3{0.0, 0.0, -0.4})));

    // Inside the vertical half-angle at a thousand metres, and outside it.
    const double edge = tan_half_y * 1000.0;
    EXPECT_TRUE(inside(frustum, to_body(Vector3{0.0, edge * 0.98, -1000.0})));
    EXPECT_FALSE(inside(frustum, to_body(Vector3{0.0, edge * 1.05, -1000.0})));
    EXPECT_TRUE(inside(frustum, to_body(Vector3{0.0, -edge * 0.98, -1000.0})));
    EXPECT_FALSE(inside(frustum, to_body(Vector3{0.0, -edge * 1.05, -1000.0})));

    // And the horizontal one, which is the vertical scaled by the aspect ratio.
    const double side = tan_half_y * aspect * 1000.0;
    EXPECT_TRUE(inside(frustum, to_body(Vector3{side * 0.98, 0.0, -1000.0})));
    EXPECT_FALSE(inside(frustum, to_body(Vector3{side * 1.05, 0.0, -1000.0})));
    EXPECT_TRUE(inside(frustum, to_body(Vector3{-side * 0.98, 0.0, -1000.0})));
    EXPECT_FALSE(inside(frustum, to_body(Vector3{-side * 1.05, 0.0, -1000.0})));
}

TEST(Unit_TerrainFrame, TheFarPlaneRejectsNothing)
{
    // The engine renders with an infinite far plane; leaving the sixth plane zeroed is how
    // that is expressed, and a zero plane has to be a plane that keeps everything --
    // including a point a hundred astronomical units away.
    const Environment environment = lunar_sky(0.0, 0.0);
    const CameraView camera = camera_looking_down_negative_z(std::tan(0.5), 1.0);
    const double eye[3] = {0.0, 0.0, 0.0};
    Field::FrustumPlanes frustum;
    RenderTerrain::TerrainFrameView view;
    RenderTerrain::build_terrain_frame(camera, environment, eye, 1080u, 0u, 0u, frustum, view);

    for (int component = 0; component < 4; ++component)
        EXPECT_DOUBLE_EQ(frustum.plane[5][component], 0.0);

    const Vector3* axes = environment.planet_body_axes;
    const Vector3 scene{0.0, 0.0, -1.5e13};
    EXPECT_TRUE(inside(frustum, Vector3{dot(scene, axes[0]), dot(scene, axes[1]),
                                        dot(scene, axes[2])}));
}

TEST(Unit_TerrainFrame, TheFieldOfViewComesOutOfTheProjection)
{
    const Environment environment = lunar_sky(10.0, 10.0);
    const double half_angle = 0.37;
    const CameraView camera = camera_looking_down_negative_z(std::tan(half_angle), 1.7);
    const double eye[3] = {0.0, 0.0, 0.0};
    Field::FrustumPlanes frustum;
    RenderTerrain::TerrainFrameView view;
    RenderTerrain::build_terrain_frame(camera, environment, eye, 900u, 11u, 2u, frustum, view);

    EXPECT_NEAR(view.vertical_field_of_view_radians, 2.0 * half_angle, 1e-6);
    EXPECT_DOUBLE_EQ(view.viewport_height_pixels, 900.0);
    EXPECT_EQ(view.frame_index, 11u);
    EXPECT_EQ(view.frame_slot, 2u);
    EXPECT_EQ(view.frustum, &frustum);
}
