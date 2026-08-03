/**************************************************************************/
/* quadtree.hpp                                                           */
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
 * @file quadtree.hpp
 * @brief Which patches of a body are drawn this frame, and at what resolution.
 *
 * The selection half of CDLOD (`docs/slop/solar_system_overhaul.md` §7.1, §8.2): descend
 * the six face quadtrees, split a node while the camera is close enough that its cells
 * would project larger than the screen-space error target, and emit the cut of the tree
 * that results. What comes out is a flat list of nodes, each carrying the camera-relative
 * frame the vertex shader needs and the morph range that makes the transition to its
 * parent continuous.
 *
 * **Host, double precision, no graphics header.** Two reasons, and the second is the one
 * that decides it. A node centre is a planet-scale coordinate, and this is the one place
 * in the whole terrain path allowed to hold one — everything downstream receives it
 * already made camera-relative, which is what §9 rests on. And the same cut of the tree is
 * what the collision patch set and the builder's placement queries need, neither of which
 * has a renderer to ask.
 */

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/terrain/cube_sphere.hpp>
#include <SushiEngine/terrain/height_source.hpp>
#include <SushiEngine/terrain/tile_address.hpp>

namespace SushiEngine
{
    namespace Terrain
    {
        /**
         * @brief Cells per side of the grid mesh one node is drawn with.
         *
         * Deliberately a quarter of @ref TILE_GRID_SIZE's cell count: a tile carries four
         * times the linear resolution of the geometry drawn from it, which is what feeds
         * the fragment normal and keeps a node's silhouette honest while its triangle count
         * stays affordable. 32 cells is 2048 triangles per node, so a few hundred visible
         * nodes is a couple of million triangles.
         */
        constexpr std::uint32_t NODE_GRID_CELLS = 32;

        /**
         * @brief Where in a node's range its morph toward the parent begins.
         *
         * A node is drawn over a distance band whose far end is where its parent takes
         * over; over the last third of that band its vertices are interpolated onto the
         * parent's grid, so the swap happens between two geometries that have already
         * become identical. Starting the morph too late makes the swap visible; too early
         * throws away resolution the node was selected for.
         */
        constexpr double NODE_MORPH_START_RATIO = 0.65;

        /**
         * @brief Six half-spaces, in the camera-relative frame the nodes are expressed in.
         *
         * `plane[i]` is `{nx, ny, nz, offset}` with a unit normal pointing *into* the
         * volume, so a sphere at `c` with radius `r` is outside plane `i` when
         * `dot(n, c) + offset < -r`.
         */
        struct FrustumPlanes
        {
            double plane[6][4] = {};
        };

        /** @brief One patch of a body, selected for drawing this frame. */
        struct TerrainNode
        {
            TileAddress address;

            /**
             * @brief The node centre's un-normalized cube point.
             *
             * The `c` of §9.2's difference form. Handed to the shader as float32 alongside
             * @ref origin_camera_relative, and the pair is what lets the whole vertex path
             * be single precision.
             */
            Vector3 centre_cube{Vector3{0.0, 0.0, 1.0}};

            /**
             * @brief The node centre's point on the reference surface, minus the camera.
             *
             * Zero elevation: the vertex shader adds the sampled height along the geodetic
             * normal itself. Camera-relative and therefore small, which is the entire point.
             */
            Vector3 origin_camera_relative{Vector3{0.0, 0.0, 0.0}};

            /** @brief Unit direction from the body centre to the node centre. */
            Vector3 centre_direction{Vector3{0.0, 0.0, 1.0}};

            /** @brief Radius of the sphere bounding the node's terrain, metres. */
            double bounding_radius_metres = 0.0;

            /** @brief Camera to the nearest point of that sphere, metres; zero inside it. */
            double distance_metres = 0.0;

            /** @brief Distance at which this node begins morphing toward its parent. */
            double morph_start_metres = 0.0;

            /** @brief Distance at which the morph completes and the parent takes over. */
            double morph_end_metres = 0.0;

            /** @brief The elevation band used to bound it, metres. */
            float minimum_metres = 0.0f;
            float maximum_metres = 0.0f;
        };

        /** @brief What the selection is asked for. */
        struct QuadtreeParameters
        {
            /** @brief Screen-space size a node's cell is allowed to project to, pixels. */
            double screen_error_pixels = 2.0;

            /** @brief Height of the viewport the error is measured against, pixels. */
            double viewport_height_pixels = 1080.0;

            /** @brief Vertical field of view, radians. */
            double vertical_field_of_view_radians = 1.0471975511965976;

            /** @brief Deepest level the selection may descend to. */
            std::uint8_t maximum_depth = MAX_TILE_DEPTH;

            /**
             * @brief Most nodes to emit.
             *
             * Reaching it stops the descent rather than truncating the list: the selection
             * degrades to coarser terrain everywhere it could not afford to refine, which
             * is a whole planet at lower resolution instead of a hole in one.
             */
            std::size_t maximum_nodes = 4096;

            /** @brief Elevation band assumed where the source cannot report one, metres. */
            double elevation_floor_metres = -12000.0;
            double elevation_ceiling_metres = 12000.0;

            /** @brief Frustum to reject against, or null to select the whole body. */
            const FrustumPlanes* frustum = nullptr;
        };

        /** @brief What the selection did, for the editor's readout and for tests. */
        struct QuadtreeStatistics
        {
            std::size_t visited = 0;   /**< Nodes considered, including rejected ones. */
            std::size_t selected = 0;  /**< Nodes emitted. */
            std::size_t rejected = 0;  /**< Nodes the frustum removed, subtrees included. */
            std::uint8_t deepest = 0;  /**< Deepest level reached. */

            /**
             * @brief Whether the node budget stopped a refinement that was wanted.
             *
             * Surfaced rather than swallowed: silently drawing coarser terrain than the
             * error target asked for reads as a quality bug with no cause.
             */
            bool budget_exhausted = false;
        };

        namespace Detail
        {
            /** @brief Difference of two vectors; spelled out to avoid relying on operators. */
            inline Vector3 subtract(const Vector3& a, const Vector3& b) noexcept
            {
                return Vector3{a.x - b.x, a.y - b.y, a.z - b.z};
            }

            /** @brief A body's cube-face edge arc, from its mean radius. */
            inline double face_arc_metres(const Ellipsoid& ellipsoid) noexcept
            {
                const double mean = (ellipsoid.semi_axis_x + ellipsoid.semi_axis_y +
                                     ellipsoid.semi_axis_z) /
                                    3.0;
                return 1.5707963267948966 * mean;
            }

            /** @brief The camera distance below which a node at @p depth wants to split. */
            inline double split_range_metres(double arc, double focal_pixels,
                                             double error_pixels, std::uint8_t depth) noexcept
            {
                const double cell =
                    arc / (static_cast<double>(NODE_GRID_CELLS) *
                           static_cast<double>(std::uint64_t(1) << depth));
                return cell * focal_pixels / error_pixels;
            }

            /** @brief One node's bounding sphere and its camera-relative frame. */
            struct NodeGeometry
            {
                Vector3 centre_cube;
                Vector3 centre_direction;
                Vector3 origin_camera_relative;
                Vector3 bounding_centre_camera_relative;
                double bounding_radius_metres;
                double distance_metres;
            };

            /**
             * @brief Places a node: its frame, the sphere bounding its terrain, and the
             *        camera's distance to that sphere.
             *
             * The sphere is fitted to the eight corners of the node's elevation band rather
             * than estimated from its angular size, because a node spanning a 20 km cliff is
             * nothing like the sphere its footprint alone would suggest.
             */
            inline NodeGeometry place_node(const Ellipsoid& ellipsoid, const TileAddress& address,
                                           const Vector3& camera, double minimum_metres,
                                           double maximum_metres) noexcept
            {
                const TileGridRect rect = tile_grid_rect(address);
                const double centre_s = 0.5 * (rect.s_minimum + rect.s_maximum);
                const double centre_t = 0.5 * (rect.t_minimum + rect.t_maximum);

                NodeGeometry geometry;
                geometry.centre_cube = face_direction<double>(
                    address.face, grid_to_face(centre_s), grid_to_face(centre_t));
                geometry.centre_direction = normalize(geometry.centre_cube);

                const Vector3 surface = ellipsoid_point(ellipsoid, geometry.centre_direction);
                const Vector3 normal = ellipsoid_normal(ellipsoid, surface);
                geometry.origin_camera_relative = subtract(surface, camera);

                const double middle = 0.5 * (minimum_metres + maximum_metres);
                const Vector3 bounding_centre{surface.x + normal.x * middle,
                                              surface.y + normal.y * middle,
                                              surface.z + normal.z * middle};
                geometry.bounding_centre_camera_relative = subtract(bounding_centre, camera);

                double widest = 0.0;
                for (std::uint32_t corner = 0; corner < 4u; ++corner)
                {
                    const double corner_s =
                        (corner & 1u) != 0u ? rect.s_maximum : rect.s_minimum;
                    const double corner_t =
                        (corner & 2u) != 0u ? rect.t_maximum : rect.t_minimum;
                    const Vector3 direction = grid_direction(address.face, corner_s, corner_t);
                    const Vector3 corner_surface = ellipsoid_point(ellipsoid, direction);
                    const Vector3 corner_normal = ellipsoid_normal(ellipsoid, corner_surface);
                    for (int side = 0; side < 2; ++side)
                    {
                        const double elevation = side == 0 ? minimum_metres : maximum_metres;
                        const Vector3 point{corner_surface.x + corner_normal.x * elevation,
                                            corner_surface.y + corner_normal.y * elevation,
                                            corner_surface.z + corner_normal.z * elevation};
                        const double reach = length(subtract(point, bounding_centre));
                        widest = reach > widest ? reach : widest;
                    }
                }
                geometry.bounding_radius_metres = widest;

                const double centre_distance = length(geometry.bounding_centre_camera_relative);
                geometry.distance_metres =
                    centre_distance > widest ? centre_distance - widest : 0.0;
                return geometry;
            }

            /** @brief Whether a camera-relative sphere is outside any frustum plane. */
            inline bool outside_frustum(const FrustumPlanes& frustum, const Vector3& centre,
                                        double radius) noexcept
            {
                for (int index = 0; index < 6; ++index)
                {
                    const double* plane = frustum.plane[index];
                    const double signed_distance = plane[0] * centre.x + plane[1] * centre.y +
                                                   plane[2] * centre.z + plane[3];
                    if (signed_distance < -radius)
                        return true;
                }
                return false;
            }
        } // namespace Detail

        /**
         * @brief Selects the cut of a body's quadtree that is drawn this frame.
         *
         * **Refinement, not descent.** The selection starts from the six root faces — which
         * are already a complete cover of the body — and repeatedly replaces the node that
         * most wants more resolution with its four children, while the budget allows.
         * Splitting a cut yields a cut, so the result covers every point of the body exactly
         * once at every stage, including the stage where the budget stops it. A recursive
         * descent cannot promise that: it commits to refining a subtree before it knows
         * whether it can afford to emit the whole of it, and running out part-way leaves a
         * hole rather than a coarser patch.
         *
         * The order is by how far over the error target a node is, so a budget that binds
         * spends what it has where the picture needs it most.
         *
         * Each emitted node carries the camera-relative frame §9.2's difference form
         * consumes and the distance band over which it morphs into its parent, so nothing
         * downstream recomputes either.
         *
         * @param ellipsoid The body's reference surface.
         * @param source    Consulted for each node's elevation band; a source that cannot
         *                  report one falls back to the parameters' global band.
         * @param camera    Camera position in body-fixed metres.
         * @param parameters What to select for.
         * @param nodes     Cleared, then filled with the selection.
         * @return What the selection did.
         */
        inline QuadtreeStatistics select_terrain_nodes(const Ellipsoid& ellipsoid,
                                                       const IHeightSource& source,
                                                       const Vector3& camera,
                                                       const QuadtreeParameters& parameters,
                                                       std::vector<TerrainNode>& nodes)
        {
            nodes.clear();
            QuadtreeStatistics statistics;

            const double arc = Detail::face_arc_metres(ellipsoid);
            const double half_field = 0.5 * parameters.vertical_field_of_view_radians;
            const double tangent = std::tan(half_field);
            const double focal_pixels =
                tangent > 1.0e-9 ? parameters.viewport_height_pixels / (2.0 * tangent)
                                 : parameters.viewport_height_pixels;
            const double error =
                parameters.screen_error_pixels > 1.0e-6 ? parameters.screen_error_pixels : 1.0e-6;
            const std::uint8_t maximum_depth =
                parameters.maximum_depth <= MAX_TILE_DEPTH ? parameters.maximum_depth
                                                           : MAX_TILE_DEPTH;

            // The working cut. A split retires an entry and appends its survivors rather
            // than erasing, so indices already in the heap stay valid; the retired ones are
            // dropped in one compaction at the end.
            std::vector<TerrainNode> pool;
            std::vector<bool> live;
            std::size_t live_count = 0;

            /** How badly a node wants more resolution; larger is more urgent. */
            struct Candidate
            {
                double urgency;
                std::size_t index;
            };
            const auto weaker = [](const Candidate& a, const Candidate& b)
            { return a.urgency < b.urgency; };
            std::vector<Candidate> heap;

            const auto admit = [&](const TileAddress& address) -> void
            {
                ++statistics.visited;

                float minimum = static_cast<float>(parameters.elevation_floor_metres);
                float maximum = static_cast<float>(parameters.elevation_ceiling_metres);
                source.tile_bounds(address, minimum, maximum);
                if (maximum < minimum)
                {
                    const float swap = minimum;
                    minimum = maximum;
                    maximum = swap;
                }

                const Detail::NodeGeometry geometry = Detail::place_node(
                    ellipsoid, address, camera, static_cast<double>(minimum),
                    static_cast<double>(maximum));

                if (parameters.frustum != nullptr &&
                    Detail::outside_frustum(*parameters.frustum,
                                            geometry.bounding_centre_camera_relative,
                                            geometry.bounding_radius_metres))
                {
                    ++statistics.rejected;
                    return;
                }

                const double range =
                    Detail::split_range_metres(arc, focal_pixels, error, address.depth);

                TerrainNode node;
                node.address = address;
                node.centre_cube = geometry.centre_cube;
                node.centre_direction = geometry.centre_direction;
                node.origin_camera_relative = geometry.origin_camera_relative;
                node.bounding_radius_metres = geometry.bounding_radius_metres;
                node.distance_metres = geometry.distance_metres;
                node.minimum_metres = minimum;
                node.maximum_metres = maximum;

                // The parent takes over at twice this node's own range, because a level up
                // is a cell twice the size. Depth zero has no parent to morph into.
                if (address.depth == 0)
                {
                    node.morph_start_metres = 1.0e30;
                    node.morph_end_metres = 1.0e30;
                }
                else
                {
                    node.morph_end_metres = range * 2.0;
                    node.morph_start_metres = node.morph_end_metres * NODE_MORPH_START_RATIO;
                }

                pool.push_back(node);
                live.push_back(true);
                ++live_count;

                if (address.depth < maximum_depth && geometry.distance_metres < range)
                {
                    const double urgency =
                        range / (geometry.distance_metres > 1.0 ? geometry.distance_metres
                                                                : 1.0);
                    heap.push_back(Candidate{urgency, pool.size() - 1u});
                    std::push_heap(heap.begin(), heap.end(), weaker);
                }
            };

            for (std::uint8_t face = 0; face < CUBE_FACE_COUNT; ++face)
                admit(TileAddress{static_cast<CubeFace>(face), 0, 0, 0});

            // One split turns one node into at most four, so it costs at most three more.
            while (!heap.empty())
            {
                if (live_count + 3u > parameters.maximum_nodes)
                {
                    statistics.budget_exhausted = true;
                    break;
                }
                std::pop_heap(heap.begin(), heap.end(), weaker);
                const Candidate best = heap.back();
                heap.pop_back();
                if (!live[best.index])
                    continue;

                const TileAddress parent = pool[best.index].address;
                live[best.index] = false;
                --live_count;
                for (std::uint32_t quadrant = 0; quadrant < 4u; ++quadrant)
                    admit(tile_child(parent, quadrant));
            }

            nodes.reserve(live_count);
            for (std::size_t index = 0; index < pool.size(); ++index)
            {
                if (!live[index])
                    continue;
                nodes.push_back(pool[index]);
                if (pool[index].address.depth > statistics.deepest)
                    statistics.deepest = pool[index].address.depth;
            }
            statistics.selected = nodes.size();
            return statistics;
        }
    } // namespace Terrain
} // namespace SushiEngine
