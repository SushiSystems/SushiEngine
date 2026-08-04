/**************************************************************************/
/* test_cube_sphere.cpp                                                   */
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

// Unit_CubeSphere: the planetary tile addressing and its projection
// (docs/design/solar_system_overhaul.md §4, §9). Four claims are checked, and the last of
// them is the one the whole terrain design rests on:
//
//   1. The tangent warp is an involution's two halves and fixes the face edges exactly.
//   2. The cube map inverts on all six faces, which is what makes a face-crossing
//      neighbour computable rather than tabulated.
//   3. Neighbour adjacency is symmetric across face seams, corners included.
//   4. The cancellation-free difference form holds millimetre accuracy at depth 20 in
//      single precision -- while the naive single-precision form, checked side by side
//      on the same vertices, does not. The second half matters as much as the first: it
//      is what shows the extra arithmetic is buying something.

#include <cmath>
#include <cstddef>
#include <cstdint>

#include <gtest/gtest.h>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/terrain/cube_sphere.hpp>
#include <SushiEngine/terrain/tile_address.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Terrain;

namespace
{
    constexpr double EARTH_EQUATORIAL_METRES = 6378137.0;
    constexpr double EARTH_INVERSE_FLATTENING = 298.257223563;

    const CubeFace ALL_FACES[CUBE_FACE_COUNT] = {
        CubeFace::PositiveX, CubeFace::NegativeX, CubeFace::PositiveY,
        CubeFace::NegativeY, CubeFace::PositiveZ, CubeFace::NegativeZ};

    const TileEdge ALL_EDGES[TILE_EDGE_COUNT] = {TileEdge::NegativeS, TileEdge::PositiveS,
                                                 TileEdge::NegativeT, TileEdge::PositiveT};

    /** Whether @p candidate is one of @p tile's four neighbours. */
    bool is_neighbour_of(const TileAddress& candidate, const TileAddress& tile)
    {
        for (TileEdge edge : ALL_EDGES)
        {
            TileAddress found;
            if (tile_neighbour(tile, edge, found) && found == candidate)
                return true;
        }
        return false;
    }

    Vector3 subtract(const Vector3& a, const Vector3& b)
    {
        return Vector3{a.x - b.x, a.y - b.y, a.z - b.z};
    }

    Vector3T<float> to_float(const Vector3& v)
    {
        return Vector3T<float>{static_cast<float>(v.x), static_cast<float>(v.y),
                               static_cast<float>(v.z)};
    }

    /** Distance between a double reference and a single-precision result, metres. */
    double error_metres(const Vector3& reference, const Vector3T<float>& measured)
    {
        const double dx = reference.x - static_cast<double>(measured.x);
        const double dy = reference.y - static_cast<double>(measured.y);
        const double dz = reference.z - static_cast<double>(measured.z);
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }
} // namespace

TEST(Unit_CubeSphere, WarpFixesTheFaceEdgesAndTheCentre)
{
    // The warp must not move the face's own boundary: a tile edge at grid +-1 has to
    // land on the cube edge at face +-1, or adjacent faces would not meet.
    EXPECT_NEAR(grid_to_face(1.0), 1.0, 1e-15);
    EXPECT_NEAR(grid_to_face(-1.0), -1.0, 1e-15);
    EXPECT_NEAR(grid_to_face(0.0), 0.0, 1e-18);
}

TEST(Unit_CubeSphere, WarpRoundTrips)
{
    for (int step = -20; step <= 20; ++step)
    {
        const double grid = static_cast<double>(step) / 20.0;
        EXPECT_NEAR(face_to_grid(grid_to_face(grid)), grid, 1e-14);
    }
}

TEST(Unit_CubeSphere, WarpEvensOutCellSize)
{
    // The reason the warp exists: without it the centre cell of a face subtends far more
    // solid angle than a corner cell. Compare the arc a fixed grid step covers at the
    // face centre against the same step near its edge; unwarped the ratio is about 1.4,
    // warped it must be well under 1.1.
    const double step = 0.01;
    const auto arc = [](double from, double to, bool warped)
    {
        const double a = warped ? grid_to_face(from) : from;
        const double b = warped ? grid_to_face(to) : to;
        const Vector3 first = normalize(face_direction<double>(CubeFace::PositiveZ, a, 0.0));
        const Vector3 second = normalize(face_direction<double>(CubeFace::PositiveZ, b, 0.0));
        double cosine = dot(first, second);
        cosine = cosine > 1.0 ? 1.0 : cosine;
        return std::acos(cosine);
    };

    const double unwarped_ratio = arc(0.0, step, false) / arc(1.0 - step, 1.0, false);
    const double warped_ratio = arc(0.0, step, true) / arc(1.0 - step, 1.0, true);

    EXPECT_GT(unwarped_ratio, 1.3);
    EXPECT_LT(warped_ratio, 1.1);
}

TEST(Unit_CubeSphere, CubeMapInvertsOnEveryFace)
{
    for (CubeFace face : ALL_FACES)
    {
        for (int si = -9; si <= 9; si += 3)
        {
            for (int ti = -9; ti <= 9; ti += 3)
            {
                const double s = static_cast<double>(si) / 10.0;
                const double t = static_cast<double>(ti) / 10.0;
                const Vector3 direction = face_direction<double>(face, s, t);
                const FaceCoordinate recovered = direction_to_face(direction);
                EXPECT_EQ(static_cast<int>(recovered.face), static_cast<int>(face));
                EXPECT_NEAR(recovered.s, s, 1e-14);
                EXPECT_NEAR(recovered.t, t, 1e-14);
            }
        }
    }
}

TEST(Unit_CubeSphere, ParentAndChildRoundTrip)
{
    const TileAddress root{CubeFace::NegativeY, 0, 0, 0};
    EXPECT_EQ(tile_parent(root), root); // a root walks up to itself rather than underflowing

    TileAddress tile{CubeFace::PositiveX, 4, 11, 6};
    for (std::uint32_t quadrant = 0; quadrant < 4u; ++quadrant)
    {
        const TileAddress child = tile_child(tile, quadrant);
        EXPECT_EQ(static_cast<int>(child.depth), static_cast<int>(tile.depth) + 1);
        EXPECT_TRUE(tile_address_valid(child));
        EXPECT_EQ(tile_parent(child), tile);
    }

    const TileAddress deepest{CubeFace::PositiveX, MAX_TILE_DEPTH, 0, 0};
    EXPECT_EQ(tile_child(deepest, 3), deepest); // and stops at the deepest level
}

TEST(Unit_CubeSphere, AddressKeysAreDistinct)
{
    const TileAddress addresses[] = {
        {CubeFace::PositiveX, 0, 0, 0},   {CubeFace::NegativeX, 0, 0, 0},
        {CubeFace::PositiveX, 1, 0, 0},   {CubeFace::PositiveX, 1, 1, 0},
        {CubeFace::PositiveX, 1, 0, 1},   {CubeFace::NegativeZ, MAX_TILE_DEPTH, 1048575, 1048575}};
    for (std::size_t i = 0; i < sizeof(addresses) / sizeof(addresses[0]); ++i)
    {
        EXPECT_EQ(tile_address_key(addresses[i]), tile_address_key(addresses[i]));
        for (std::size_t j = i + 1; j < sizeof(addresses) / sizeof(addresses[0]); ++j)
            EXPECT_NE(tile_address_key(addresses[i]), tile_address_key(addresses[j]));
    }
}

TEST(Unit_CubeSphere, NeighboursInsideAFaceAreAnIntegerStep)
{
    const TileAddress tile{CubeFace::PositiveZ, 3, 4, 4};
    TileAddress found;

    ASSERT_TRUE(tile_neighbour(tile, TileEdge::PositiveS, found));
    EXPECT_EQ(found, (TileAddress{CubeFace::PositiveZ, 3, 5, 4}));
    ASSERT_TRUE(tile_neighbour(tile, TileEdge::NegativeT, found));
    EXPECT_EQ(found, (TileAddress{CubeFace::PositiveZ, 3, 4, 3}));
}

TEST(Unit_CubeSphere, NeighbourAdjacencyIsSymmetricAcrossFaceSeams)
{
    // Every tile on a face boundary, at three depths, on all six faces -- so every seam
    // and every cube corner is covered. A neighbour that crossed a seam into the wrong
    // face, or into a diagonal cell, breaks the symmetry this asserts.
    const std::uint8_t depths[] = {1, 3, 5};
    for (std::uint8_t depth : depths)
    {
        const std::uint32_t side = tiles_per_side(depth);
        for (CubeFace face : ALL_FACES)
        {
            for (std::uint32_t index = 0; index < side; ++index)
            {
                const TileAddress boundary[] = {{face, depth, 0, index},
                                                {face, depth, side - 1u, index},
                                                {face, depth, index, 0},
                                                {face, depth, index, side - 1u}};
                for (const TileAddress& tile : boundary)
                {
                    for (TileEdge edge : ALL_EDGES)
                    {
                        TileAddress found;
                        ASSERT_TRUE(tile_neighbour(tile, edge, found));
                        EXPECT_TRUE(tile_address_valid(found));
                        EXPECT_NE(found, tile);
                        EXPECT_TRUE(is_neighbour_of(tile, found))
                            << "asymmetric neighbour at depth " << static_cast<int>(depth)
                            << " face " << static_cast<int>(face) << " cell (" << tile.x
                            << ", " << tile.y << ")";
                    }
                }
            }
        }
    }
}

TEST(Unit_CubeSphere, DifferenceFormHoldsMillimetreAccuracyAtDepthTwenty)
{
    // The claim of §9.2, measured. At depth 20 an Earth tile is about 9.5 m across and a
    // render cell is 0.075 m, so a form that cannot resolve well below a centimetre here
    // is a form the terrain visibly boils under.
    const Ellipsoid earth =
        ellipsoid_of_revolution(EARTH_EQUATORIAL_METRES, EARTH_INVERSE_FLATTENING);
    const TileAddress tile{CubeFace::PositiveZ, MAX_TILE_DEPTH, 700000, 300000};
    const TileGridRect rect = tile_grid_rect(tile);
    const double centre_s = 0.5 * (rect.s_minimum + rect.s_maximum);
    const double centre_t = 0.5 * (rect.t_minimum + rect.t_maximum);

    const Vector3 centre_cube =
        face_direction<double>(tile.face, grid_to_face(centre_s), grid_to_face(centre_t));
    const Vector3 centre_surface =
        ellipsoid_point(earth, normalize(centre_cube));

    double worst_difference_form = 0.0;
    double worst_naive_form = 0.0;
    for (std::uint32_t row = 0; row <= TILE_GRID_SIZE - 1u; row += 32u)
    {
        for (std::uint32_t column = 0; column <= TILE_GRID_SIZE - 1u; column += 32u)
        {
            double grid_s = 0.0;
            double grid_t = 0.0;
            tile_sample_grid_coordinate(tile, column + TILE_APRON, row + TILE_APRON, grid_s,
                                        grid_t);
            const Vector3 vertex_cube =
                face_direction<double>(tile.face, grid_to_face(grid_s), grid_to_face(grid_t));

            // The reference: the exact displacement from the tile centre, in double.
            const Vector3 reference =
                subtract(ellipsoid_point(earth, normalize(vertex_cube)), centre_surface);

            // The shipping form: the host holds the centre in double, the device sees only
            // the centre direction and a small offset, both in float.
            const Vector3T<float> centre_float = to_float(centre_cube);
            const Vector3T<float> offset_float = to_float(subtract(vertex_cube, centre_cube));
            const Vector3T<float> measured =
                ellipsoid_point_delta(earth, centre_float, offset_float);

            // The naive form: build both planet-space points in float and subtract, which
            // is what a shader does if nobody thinks about it.
            const Vector3T<float> vertex_float = to_float(vertex_cube);
            const Vector3T<float> naive_vertex =
                ellipsoid_point(earth, normalize(vertex_float));
            const Vector3T<float> naive_centre =
                ellipsoid_point(earth, normalize(centre_float));
            const Vector3T<float> naive{naive_vertex.x - naive_centre.x,
                                        naive_vertex.y - naive_centre.y,
                                        naive_vertex.z - naive_centre.z};

            const double difference_error = error_metres(reference, measured);
            const double naive_error = error_metres(reference, naive);
            if (difference_error > worst_difference_form)
                worst_difference_form = difference_error;
            if (naive_error > worst_naive_form)
                worst_naive_form = naive_error;
        }
    }

    EXPECT_LT(worst_difference_form, 1.0e-3)
        << "the difference form lost millimetre accuracy at depth " << static_cast<int>(MAX_TILE_DEPTH);

    // And the contrast that makes the extra arithmetic worth its cost: the naive form,
    // on the same vertices, is off by a large fraction of a render cell or worse.
    EXPECT_GT(worst_naive_form, 1.0e-1)
        << "the naive single-precision form was unexpectedly accurate -- if this ever "
           "fails the precision argument in §9.1 needs re-deriving, not the code";
    EXPECT_GT(worst_naive_form, worst_difference_form * 100.0);
}

TEST(Unit_CubeSphere, DifferenceFormAgreesWithTheDirectFormInDouble)
{
    // The algebra itself, independent of precision: the rearrangement must be an
    // identity, not merely a better-conditioned approximation.
    const Vector3 centre = face_direction<double>(CubeFace::NegativeY, 0.31, -0.62);
    const Vector3 offset{1.0e-4, -3.0e-5, 7.0e-5};
    const Vector3 direct =
        subtract(normalize(Vector3{centre.x + offset.x, centre.y + offset.y, centre.z + offset.z}),
                 normalize(centre));
    const Vector3 rearranged = normalized_difference(centre, offset);
    EXPECT_NEAR(rearranged.x, direct.x, 1e-15);
    EXPECT_NEAR(rearranged.y, direct.y, 1e-15);
    EXPECT_NEAR(rearranged.z, direct.z, 1e-15);
}

TEST(Unit_CubeSphere, EllipsoidPlacesTheEquatorAndPoleAtTheirRadii)
{
    const Ellipsoid earth =
        ellipsoid_of_revolution(EARTH_EQUATORIAL_METRES, EARTH_INVERSE_FLATTENING);
    const Vector3 equator = ellipsoid_point(earth, Vector3{1.0, 0.0, 0.0});
    EXPECT_NEAR(equator.x, EARTH_EQUATORIAL_METRES, 1e-6);

    const Vector3 pole = ellipsoid_point(earth, Vector3{0.0, 0.0, 1.0});
    const double flattening = 1.0 / EARTH_INVERSE_FLATTENING;
    EXPECT_NEAR(pole.z, EARTH_EQUATORIAL_METRES * (1.0 - flattening), 1e-6);

    // A sphere's geodetic normal is radial; a flattened body's is not, and the departure
    // is why elevation displaces along the normal rather than along the radius.
    const Vector3 mid = normalize(Vector3{1.0, 0.0, 1.0});
    const Vector3 surface = ellipsoid_point(earth, mid);
    const Vector3 normal = ellipsoid_normal(earth, surface);
    EXPECT_NEAR(length(normal), 1.0, 1e-12);
    EXPECT_GT(dot(normal, normalize(surface)), 0.9999);
    EXPECT_LT(dot(normal, normalize(surface)), 1.0);
}

TEST(Unit_CubeSphere, TileAngularRadiusContainsEveryStoredSample)
{
    const TileAddress tile{CubeFace::PositiveY, 4, 3, 9};
    const Vector3 centre = tile_sample_direction(tile, TILE_STRIDE / 2u, TILE_STRIDE / 2u);
    const double radius = tile_angular_radius(tile);
    EXPECT_GT(radius, 0.0);

    for (std::uint32_t row = 0; row < TILE_STRIDE; row += 13u)
    {
        for (std::uint32_t column = 0; column < TILE_STRIDE; column += 13u)
        {
            const Vector3 direction = tile_sample_direction(tile, column, row);
            double cosine = dot(centre, direction);
            cosine = cosine > 1.0 ? 1.0 : cosine;
            EXPECT_LE(std::acos(cosine), radius + 1e-12);
        }
    }
}
