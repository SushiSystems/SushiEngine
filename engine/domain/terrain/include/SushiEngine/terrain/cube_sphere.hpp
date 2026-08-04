/**************************************************************************/
/* cube_sphere.hpp                                                        */
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

#pragma once

/**
 * @file cube_sphere.hpp
 * @brief Where a tile is: the cube-to-ellipsoid map, and the difference form that
 *        survives planet scale in single precision.
 *
 * Two things live here (`docs/design/solar_system_overhaul.md` §4, §9). The first is the
 * map itself — a tangent-warped cube projected onto a triaxial ellipsoid — together
 * with its inverse, which is what turns a direction back into a tile and therefore what
 * makes a neighbour across a face seam computable rather than tabulated.
 *
 * The second is @ref normalized_difference, and it is the reason the whole design
 * works. A vertex on Earth is 6.37e6 m from the body centre; float32 carries about
 * 1.2e-7 of relative precision, so *any* planet-space quantity evaluated in float32
 * lands with roughly 0.76 m of error, against a 0.075 m cell at depth 20. Subtracting
 * the camera afterwards does not help — the error is already in the operand. What does
 * help is never forming the large quantity at all: split the face coordinate into a
 * node centre held in double on the host and a small offset from the grid, and evaluate
 * the *difference* of the two normalizations in a form with no subtraction of
 * nearly-equal quantities in it. Nine operations more than the naive form, no doubles,
 * no branches, one code path from depth 0 to depth 20.
 */

#include <cmath>
#include <cstdint>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/terrain/tile_address.hpp>

namespace SushiEngine
{
    namespace Terrain
    {
        /** @brief One quarter of pi, the tangent warp's scale. */
        constexpr double QUARTER_PI = 0.78539816339744830961;

        /**
         * @brief Grid coordinate to face coordinate: the tangent warp.
         *
         * The naive cube-to-sphere map compresses cells toward a face's corners, giving
         * the centre cell about 1.4 times the solid angle of a corner one. Warping the
         * uniform grid parameter so that its *angle* is uniform drops that ratio to
         * about 1.06, which is close enough that nothing downstream has to compensate.
         *
         * @param grid Uniform grid coordinate, normally within [-1, 1].
         * @return The face coordinate the cube map is evaluated at.
         */
        inline double grid_to_face(double grid) noexcept
        {
            return std::tan(grid * QUARTER_PI);
        }

        /**
         * @brief Face coordinate back to grid coordinate; the inverse of @ref grid_to_face.
         * @param face Face coordinate.
         * @return The uniform grid coordinate that warps to it.
         */
        inline double face_to_grid(double face) noexcept
        {
            return std::atan(face) / QUARTER_PI;
        }

        /**
         * @brief The cube point a face coordinate names, before normalization.
         *
         * The cubemap face basis, so the layout matches what a baked cube texture would
         * use and the convention is one a reader already knows. The result is a cube
         * point, not a unit direction: one component is exactly ±1.
         *
         * @tparam T Element type; double on the host, float where a shader mirrors this.
         * @param face Which face.
         * @param s    Face coordinate along the face's first axis, within [-1, 1].
         * @param t    Face coordinate along the face's second axis, within [-1, 1].
         * @return The cube point.
         */
        template <typename T>
        inline Vector3T<T> face_direction(CubeFace face, T s, T t) noexcept
        {
            switch (face)
            {
                case CubeFace::PositiveX: return Vector3T<T>{T(1), -t, -s};
                case CubeFace::NegativeX: return Vector3T<T>{T(-1), -t, s};
                case CubeFace::PositiveY: return Vector3T<T>{s, T(1), t};
                case CubeFace::NegativeY: return Vector3T<T>{s, T(-1), -t};
                case CubeFace::PositiveZ: return Vector3T<T>{s, -t, T(1)};
                case CubeFace::NegativeZ: return Vector3T<T>{-s, -t, T(-1)};
            }
            return Vector3T<T>{T(0), T(0), T(0)};
        }

        /** @brief A point on the cube, as the face it lies on and its coordinates there. */
        struct FaceCoordinate
        {
            CubeFace face = CubeFace::PositiveX;
            double s = 0.0;
            double t = 0.0;
        };

        /**
         * @brief Which face a direction points at, and where on it — @ref face_direction
         *        inverted.
         *
         * The dominant axis picks the face; dividing the other two components by its
         * magnitude recovers the face coordinates. Host-side by design: this is the map
         * a neighbour query and a layer footprint need, never a per-vertex one.
         *
         * @param direction Any non-zero direction; its length is irrelevant.
         * @return The face and the face coordinates, each within [-1, 1].
         */
        inline FaceCoordinate direction_to_face(const Vector3& direction) noexcept
        {
            const double absolute_x = std::fabs(direction.x);
            const double absolute_y = std::fabs(direction.y);
            const double absolute_z = std::fabs(direction.z);

            FaceCoordinate coordinate;
            if (absolute_x >= absolute_y && absolute_x >= absolute_z)
            {
                const double inverse = 1.0 / absolute_x;
                if (direction.x > 0.0)
                {
                    coordinate.face = CubeFace::PositiveX;
                    coordinate.s = -direction.z * inverse;
                }
                else
                {
                    coordinate.face = CubeFace::NegativeX;
                    coordinate.s = direction.z * inverse;
                }
                coordinate.t = -direction.y * inverse;
            }
            else if (absolute_y >= absolute_z)
            {
                const double inverse = 1.0 / absolute_y;
                coordinate.s = direction.x * inverse;
                if (direction.y > 0.0)
                {
                    coordinate.face = CubeFace::PositiveY;
                    coordinate.t = direction.z * inverse;
                }
                else
                {
                    coordinate.face = CubeFace::NegativeY;
                    coordinate.t = -direction.z * inverse;
                }
            }
            else
            {
                const double inverse = 1.0 / absolute_z;
                if (direction.z > 0.0)
                {
                    coordinate.face = CubeFace::PositiveZ;
                    coordinate.s = direction.x * inverse;
                }
                else
                {
                    coordinate.face = CubeFace::NegativeZ;
                    coordinate.s = -direction.x * inverse;
                }
                coordinate.t = -direction.y * inverse;
            }
            return coordinate;
        }

        /**
         * @brief The unit direction a grid coordinate on a face names.
         * @param face   Which face.
         * @param grid_s Uniform grid coordinate along the face's first axis.
         * @param grid_t Uniform grid coordinate along the face's second axis.
         * @return The unit direction from the body centre.
         */
        inline Vector3 grid_direction(CubeFace face, double grid_s, double grid_t) noexcept
        {
            return normalize(face_direction<double>(face, grid_to_face(grid_s),
                                                    grid_to_face(grid_t)));
        }

        /**
         * @brief The unit direction of one of a tile's stored samples.
         * @param address The tile.
         * @param column  Sample column, below @ref TILE_STRIDE.
         * @param row     Sample row, below @ref TILE_STRIDE.
         * @return The unit direction from the body centre.
         */
        inline Vector3 tile_sample_direction(const TileAddress& address, std::uint32_t column,
                                             std::uint32_t row) noexcept
        {
            double grid_s = 0.0;
            double grid_t = 0.0;
            tile_sample_grid_coordinate(address, column, row, grid_s, grid_t);
            return grid_direction(address.face, grid_s, grid_t);
        }

        /**
         * @brief The angular radius of the cap that contains a whole stored tile.
         *
         * Measured to the corners of the *stored* extent, apron included, so a cap built
         * from it contains every sample the tile carries and not merely its own grid.
         * This is what an overlap test against a layer footprint is asked, so it has to
         * be a bound rather than an estimate.
         *
         * @param address The tile.
         * @return The angle from the tile's centre direction to its furthest stored
         *         sample, radians.
         */
        inline double tile_angular_radius(const TileAddress& address) noexcept
        {
            const std::uint32_t last = TILE_STRIDE - 1u;
            const Vector3 centre =
                tile_sample_direction(address, TILE_STRIDE / 2u, TILE_STRIDE / 2u);
            double widest = 0.0;
            for (std::uint32_t corner = 0; corner < 4u; ++corner)
            {
                const std::uint32_t column = (corner & 1u) != 0u ? last : 0u;
                const std::uint32_t row = (corner & 2u) != 0u ? last : 0u;
                const Vector3 direction = tile_sample_direction(address, column, row);
                double cosine = dot(centre, direction);
                cosine = cosine < -1.0 ? -1.0 : (cosine > 1.0 ? 1.0 : cosine);
                const double angle = std::acos(cosine);
                if (angle > widest)
                    widest = angle;
            }
            return widest;
        }

        /**
         * @brief A tile's neighbour across one of its edges, face crossings included.
         *
         * Within a face this is `tile_neighbour_on_face`'s integer step. Across a seam it
         * is geometric: reflect the tile's centre direction through the midpoint of the
         * edge being crossed, which lands one tile-width past the seam in whichever face
         * and orientation that turns out to be, then invert the projection and quantize.
         * A hand-written adjacency table would be twenty-four entries of rotation and
         * mirroring, each of which can be wrong in a way that only shows up as a seam on
         * one edge of one face; this cannot be, because it never names an edge pairing.
         *
         * @param address The tile to step from; must be valid.
         * @param edge    Which side to step across.
         * @param out     Receives the neighbour.
         * @return true when a neighbour was produced, false only for an invalid address.
         */
        inline bool tile_neighbour(const TileAddress& address, TileEdge edge,
                                   TileAddress& out) noexcept
        {
            if (!tile_address_valid(address))
                return false;
            if (tile_neighbour_on_face(address, edge, out))
                return true;

            const TileGridRect rect = tile_grid_rect(address);
            const double centre_s = 0.5 * (rect.s_minimum + rect.s_maximum);
            const double centre_t = 0.5 * (rect.t_minimum + rect.t_maximum);
            double edge_s = centre_s;
            double edge_t = centre_t;
            switch (edge)
            {
                case TileEdge::NegativeS: edge_s = rect.s_minimum; break;
                case TileEdge::PositiveS: edge_s = rect.s_maximum; break;
                case TileEdge::NegativeT: edge_t = rect.t_minimum; break;
                case TileEdge::PositiveT: edge_t = rect.t_maximum; break;
            }

            const Vector3 centre = grid_direction(address.face, centre_s, centre_t);
            const Vector3 midpoint = grid_direction(address.face, edge_s, edge_t);
            const Vector3 reflected{2.0 * midpoint.x - centre.x, 2.0 * midpoint.y - centre.y,
                                    2.0 * midpoint.z - centre.z};

            const FaceCoordinate coordinate = direction_to_face(reflected);
            const double side = static_cast<double>(tiles_per_side(address.depth));
            const double cell_s = (face_to_grid(coordinate.s) + 1.0) * 0.5 * side;
            const double cell_t = (face_to_grid(coordinate.t) + 1.0) * 0.5 * side;

            const std::int64_t last = static_cast<std::int64_t>(tiles_per_side(address.depth)) - 1;
            std::int64_t x = static_cast<std::int64_t>(std::floor(cell_s));
            std::int64_t y = static_cast<std::int64_t>(std::floor(cell_t));
            x = x < 0 ? 0 : (x > last ? last : x);
            y = y < 0 ? 0 : (y > last ? last : y);

            out = TileAddress{coordinate.face, address.depth, static_cast<std::uint32_t>(x),
                              static_cast<std::uint32_t>(y)};
            return true;
        }

        /**
         * @brief A body's reference surface: a triaxial ellipsoid, semi-axes in metres.
         *
         * Triaxial rather than an ellipsoid of revolution because the vocabulary costs
         * nothing and some bodies are measurably triaxial; @ref ellipsoid_of_revolution
         * builds the common case.
         */
        struct Ellipsoid
        {
            double semi_axis_x = 1.0;
            double semi_axis_y = 1.0;
            double semi_axis_z = 1.0;
        };

        /**
         * @brief The ellipsoid of revolution a radius and a flattening name.
         * @param equatorial_metres Equatorial (semi-major) radius, metres.
         * @param inverse_flattening 1/f; zero, or any non-positive value, means a sphere.
         * @return The ellipsoid, with its polar axis along z.
         */
        inline Ellipsoid ellipsoid_of_revolution(double equatorial_metres,
                                                 double inverse_flattening) noexcept
        {
            const double flattening = inverse_flattening > 0.0 ? 1.0 / inverse_flattening : 0.0;
            const double polar = equatorial_metres * (1.0 - flattening);
            return Ellipsoid{equatorial_metres, equatorial_metres, polar};
        }

        /**
         * @brief The surface point a unit direction names.
         *
         * Scaling a unit direction componentwise by the semi-axes maps the unit sphere
         * onto the ellipsoid, so this is the exact surface and not an approximation of it.
         *
         * @tparam T Element type.
         * @param ellipsoid The body's reference surface.
         * @param direction A unit direction from the body centre.
         * @return The point on the ellipsoid, metres.
         */
        template <typename T>
        inline Vector3T<T> ellipsoid_point(const Ellipsoid& ellipsoid,
                                           const Vector3T<T>& direction) noexcept
        {
            return Vector3T<T>{static_cast<T>(ellipsoid.semi_axis_x) * direction.x,
                               static_cast<T>(ellipsoid.semi_axis_y) * direction.y,
                               static_cast<T>(ellipsoid.semi_axis_z) * direction.z};
        }

        /**
         * @brief The outward geodetic normal at a point on the ellipsoid.
         *
         * The gradient of the ellipsoid's implicit form, normalized. This — not the
         * radial direction — is the direction elevation displaces along, because it is
         * how every digital elevation model defines its heights; on Earth the two differ
         * by up to 0.19°, which at a 3 km mountain is a ten-metre horizontal error.
         *
         * @tparam T Element type.
         * @param ellipsoid The body's reference surface.
         * @param point     A point on (or near) that surface, metres.
         * @return The unit outward normal.
         */
        template <typename T>
        inline Vector3T<T> ellipsoid_normal(const Ellipsoid& ellipsoid,
                                            const Vector3T<T>& point) noexcept
        {
            const T inverse_x = T(1) / static_cast<T>(ellipsoid.semi_axis_x * ellipsoid.semi_axis_x);
            const T inverse_y = T(1) / static_cast<T>(ellipsoid.semi_axis_y * ellipsoid.semi_axis_y);
            const T inverse_z = T(1) / static_cast<T>(ellipsoid.semi_axis_z * ellipsoid.semi_axis_z);
            return normalize(Vector3T<T>{point.x * inverse_x, point.y * inverse_y,
                                         point.z * inverse_z});
        }

        /**
         * @brief `normalize(centre + offset) - normalize(centre)`, without cancellation.
         *
         * The whole of §9.2. Writing `g = c + d`, the identity
         * `|c| - |g| = (|c|^2 - |g|^2) / (|c| + |g|)` with `|c|^2 - |g|^2 = -(2 c.d + |d|^2)`
         * turns the difference into
         *
         *     d / |g|  -  c * (2 c.d + |d|^2) / (|g| |c| (|c| + |g|))
         *
         * in which no subtraction of nearly-equal quantities survives: every term is
         * either small (the offset, and its dot products) or a well-conditioned ratio of
         * order-one quantities. The rounding of @p centre only ever multiplies a factor
         * of the offset's own order, so the position error stays proportional to the
         * offset rather than to the body's radius — which is what makes the whole
         * geometry path single precision without a visible seam at any depth.
         *
         * @tparam T Element type; the point of this function is that float32 suffices.
         * @param centre The direction the offset is measured from; need not be unit.
         * @param offset The (small) offset added to it.
         * @return The difference of the two normalized directions.
         */
        template <typename T>
        inline Vector3T<T> normalized_difference(const Vector3T<T>& centre,
                                                 const Vector3T<T>& offset) noexcept
        {
            const Vector3T<T> sum{centre.x + offset.x, centre.y + offset.y,
                                  centre.z + offset.z};
            const T length_centre = Math::sqrt(dot(centre, centre));
            const T length_sum = Math::sqrt(dot(sum, sum));
            const T excess = T(2) * dot(centre, offset) + dot(offset, offset);
            const T scale =
                excess / (length_sum * length_centre * (length_centre + length_sum));
            return Vector3T<T>{offset.x / length_sum - centre.x * scale,
                               offset.y / length_sum - centre.y * scale,
                               offset.z / length_sum - centre.z * scale};
        }

        /**
         * @brief The ellipsoid-surface displacement an offset from a centre produces.
         *
         * `P(c + d) - P(c)`, where `P` scales a normalized direction by the semi-axes.
         * Because that scaling is linear it commutes with the difference, so this is
         * @ref normalized_difference with the axes applied — the precision argument
         * carries over unchanged, which is the reason the reference surface is defined
         * by a componentwise scale in the first place.
         *
         * @tparam T Element type.
         * @param ellipsoid The body's reference surface.
         * @param centre    The cube direction the offset is measured from.
         * @param offset    The (small) offset added to it.
         * @return The displacement between the two surface points, metres.
         */
        template <typename T>
        inline Vector3T<T> ellipsoid_point_delta(const Ellipsoid& ellipsoid,
                                                 const Vector3T<T>& centre,
                                                 const Vector3T<T>& offset) noexcept
        {
            return ellipsoid_point(ellipsoid, normalized_difference(centre, offset));
        }
    } // namespace Terrain
} // namespace SushiEngine
