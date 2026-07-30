/**************************************************************************/
/* collision_asset.hpp                                                    */
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
 * @file collision_asset.hpp
 * @brief The `.sushicollision` blob: what a body collides as, cooked once.
 *
 * §5.4's first row, on `Animation::SkeletonBlob`'s shape — a versioned header,
 * offset-based sections, no pointers, loadable with one read. The point of that shape is
 * @ref collision_asset_hull and @ref collision_asset_mesh: both hand the runtime a view
 * whose pointers are *into the mapped bytes*, so instancing an asset copies nothing and
 * every instance of a crate shares one vertex array. That is why the geometry is stored
 * in `Scalar` rather than in `float` despite being twice the size — a `float` array would
 * have to be converted at load, and a format that needs converting is not a format that
 * is memory-mappable.
 *
 * The distance field is the deliberate exception and stays `float`: at the fidelity dial's
 * top resolution it is 128³ values, and doubling eight megabytes to match a convention
 * would be indefensible when nothing points a `Scalar` view at it.
 *
 * **Two shapes, one blob.** A dynamic body's asset carries convex pieces; static geometry
 * carries a triangle hierarchy instead (§8.4 item 4), because decomposing a level's
 * terrain spends minutes producing a worse collider than the exact triangles. The header's
 * flag says which, and the unused sections are empty rather than absent so the layout does
 * not fork.
 */

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/geometry/signed_distance_field.hpp>
#include <SushiEngine/physics/geometry/mesh_bvh.hpp>
#include <SushiEngine/physics/geometry/shapes.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        namespace Cooking
        {
            /** @brief Magic at offset 0 of a `.sushicollision` blob. */
            constexpr char COLLISION_BLOB_MAGIC[8] = {'S', 'U', 'S', 'H', 'C', 'O', 'L', 'L'};

            /** @brief Current `.sushicollision` format version. */
            constexpr std::uint32_t COLLISION_BLOB_VERSION = 1;

            /** @brief Bits in @ref CollisionBlobHeader::flags. */
            enum CollisionAssetFlags : std::uint32_t
            {
                /** @brief The asset carries a triangle hierarchy, not convex pieces. */
                COLLISION_ASSET_STATIC_MESH = 1u << 0,
            };

            /**
             * @brief One convex piece, as it sits in the blob.
             *
             * Field order chosen so the record has no interior padding: the blob must be
             * byte-reproducible for the same input, and padding bytes a `memcpy` leaves
             * indeterminate would make an unchanged mesh cook to two different files.
             */
            struct CollisionPieceRecord
            {
                /** @brief The piece's hull centroid, in the asset's frame. */
                Vector3 center;

                /** @brief The inflation `support()` adds; the vertices are shrunk to match. */
                Scalar convex_radius;

                /** @brief The piece's own hull volume, for the report. */
                Scalar volume;

                /** @brief First vertex, into the blob's hull-vertex array. */
                std::uint32_t first_vertex;

                /** @brief How many vertices this piece owns. */
                std::uint32_t vertex_count;
            };

            /**
             * @brief What the asset weighs, how it spins, and how wrong it is.
             *
             * One record rather than fields on the header, because these travel together
             * everywhere they are read — the inspector shows all of them and the extract
             * consumes all of them — and a caller holding the mass without the principal
             * rotation is holding a diagonal expressed in the wrong frame.
             */
            struct CollisionAssetSummary
            {
                /** @brief Mass at the cooked density, in kilogrammes. */
                Scalar mass;

                /** @brief Centre of mass in the asset's own frame. */
                Vector3 center_of_mass;

                /** @brief Principal moments of inertia about @ref center_of_mass. */
                Vector3 principal_inertia;

                /** @brief Rotation from the principal frame to the asset's frame. */
                Quaternion principal_rotation;

                /** @brief Enclosed volume of the source mesh, in cubic local units. */
                Scalar volume;

                /**
                 * @brief How far the collision geometry protrudes past the source mesh.
                 *
                 * §7.6's number, in local units, and a sampled lower bound — the maximum is
                 * over a finite lattice, so a spike between samples is under-reported.
                 */
                float hausdorff_error;

                /**
                 * @brief The collider's volume over the mesh's, minus one.
                 *
                 * Positive means the collider encloses more. Estimated from the pieces'
                 * volumes **summed**, so a region two pieces both cover is counted twice
                 * and the figure errs toward reporting too much collider — which is the
                 * safe direction for a threshold.
                 */
                float volume_error;

                /** @brief Substeps the fidelity dial suggests for a body of this asset. */
                std::uint32_t suggested_substep_count;

                /** @brief Convex pieces, or triangles for a static asset; mirrors the header. */
                std::uint32_t element_count;
            };

            /**
             * @brief The fixed header at offset 0.
             *
             * Fixed-width fields and byte offsets from the blob's start, so the blob is
             * position independent and the loader rebuilds every pointer from these.
             */
            struct CollisionBlobHeader
            {
                char magic[8];                      /**< @ref COLLISION_BLOB_MAGIC. */
                std::uint32_t version;              /**< @ref COLLISION_BLOB_VERSION. */
                std::uint32_t flags;                /**< @ref CollisionAssetFlags bits. */
                std::uint32_t total_size;           /**< Whole blob size in bytes. */
                std::uint32_t piece_count;
                std::uint32_t hull_vertex_count;    /**< Total across every piece. */
                std::uint32_t mesh_vertex_count;
                std::uint32_t mesh_triangle_count;
                std::uint32_t mesh_node_count;
                std::uint32_t field_resolution;     /**< Voxels per axis; zero for no field. */
                std::uint32_t summary_offset;       /**< CollisionAssetSummary. */
                std::uint32_t pieces_offset;        /**< CollisionPieceRecord[piece_count]. */
                std::uint32_t hull_vertices_offset; /**< Vector3[hull_vertex_count]. */
                std::uint32_t mesh_vertices_offset; /**< Vector3[mesh_vertex_count]. */
                std::uint32_t mesh_indices_offset;  /**< uint32[3 * mesh_triangle_count]. */
                std::uint32_t mesh_nodes_offset;    /**< MeshBvhNode<Scalar>[mesh_node_count]. */
                std::uint32_t mesh_order_offset;    /**< uint32[mesh_triangle_count]. */
                std::uint32_t mesh_adjacency_offset;/**< uint32[3 * mesh_triangle_count]. */
                std::uint32_t field_offset;         /**< float[resolution^3]. */
                float field_min[3];                 /**< Padded field bounds, asset frame. */
                float field_max[3];
            };

            /** @brief The arrays a cooked collision asset owns, before serialization. */
            struct CollisionAsset
            {
                std::vector<CollisionPieceRecord> pieces;
                std::vector<Vector3> hull_vertices;
                std::vector<Vector3> mesh_vertices;
                std::vector<std::uint32_t> mesh_indices;
                std::vector<MeshBvhNode<Scalar>> mesh_nodes;
                std::vector<std::uint32_t> mesh_order;
                std::vector<std::uint32_t> mesh_adjacency;
                Geometry::SignedDistanceFieldBrick field;
                CollisionAssetSummary summary{};
                bool static_mesh = false;
            };

            /**
             * @brief A view over a validated blob; every pointer is into the bytes.
             *
             * Non-owning, so it must not outlive the blob. That is the whole design: the
             * runtime holds shapes that reference this, and the bytes are shared by every
             * instance of the asset (§12's immutable cooked assets).
             */
            struct CollisionAssetView
            {
                const CollisionPieceRecord* pieces = nullptr;
                std::uint32_t piece_count = 0;
                const Vector3* hull_vertices = nullptr;
                std::uint32_t hull_vertex_count = 0;

                const Vector3* mesh_vertices = nullptr;
                std::uint32_t mesh_vertex_count = 0;
                const std::uint32_t* mesh_indices = nullptr;
                const MeshBvhNode<Scalar>* mesh_nodes = nullptr;
                const std::uint32_t* mesh_order = nullptr;
                const std::uint32_t* mesh_adjacency = nullptr;
                std::uint32_t mesh_triangle_count = 0;
                std::uint32_t mesh_node_count = 0;

                const float* distances = nullptr;
                std::uint32_t field_resolution = 0;
                float field_min[3] = {0.0f, 0.0f, 0.0f};
                float field_max[3] = {0.0f, 0.0f, 0.0f};

                CollisionAssetSummary summary{};
                bool static_mesh = false;
                bool valid = false;
            };

            namespace detail
            {
                /** @brief Rounds @p value up to the next multiple of @p alignment. */
                inline std::size_t collision_align_up(std::size_t value,
                                                      std::size_t alignment) noexcept
                {
                    return (value + alignment - 1) & ~(alignment - 1);
                }
            } // namespace detail

            /**
             * @brief Serializes @p asset into a `.sushicollision` blob.
             *
             * Refuses rather than writes a blob it would not itself load: a piece naming a
             * vertex range outside the hull array is an asset whose symptom, unchecked, is a
             * support function reading past the end of the file. A writer whose validation
             * is weaker than its reader's is a format that cannot be trusted at either end.
             *
             * @param asset The cooked arrays.
             * @param out   Receives the blob bytes; cleared first, left empty on refusal.
             * @return False when @p asset is not well formed.
             */
            bool build_collision_blob(const CollisionAsset& asset, std::vector<std::byte>& out);

            /**
             * @brief Whether @p data is a blob this build can load.
             *
             * Every section is checked to lie inside the declared total *and* inside the
             * actual size, because a header that lies about its length is the interesting
             * case — a truncated or hostile asset must fail here rather than produce a view
             * whose pointers walk off the end.
             *
             * @param data The blob bytes.
             * @param size Their length.
             * @return True when @ref load_collision_blob will produce a usable view.
             */
            bool validate_collision_blob(const std::byte* data, std::size_t size) noexcept;

            /**
             * @brief Rebuilds a view over a validated blob.
             *
             * @param data The blob bytes.
             * @param size Their length.
             * @return A view, or a default (invalid) one when the blob does not validate.
             */
            CollisionAssetView load_collision_blob(const std::byte* data,
                                                   std::size_t size) noexcept;

            /**
             * @brief Piece @p index as a shape the narrowphase can collide.
             *
             * The placement is left at the identity: a piece's stored centre is in the
             * *asset's* frame, and putting the asset in the world is the body's job. The
             * caller composes the two, which is what keeps one cooked asset usable by a
             * thousand bodies.
             *
             * @param view  A validated asset.
             * @param index Which piece; out of range yields an empty hull, which supports
             *              its own centre rather than reading past the end of nothing.
             * @return The hull view, pointing into the blob.
             */
            inline ConvexHullView<Scalar> collision_asset_hull(const CollisionAssetView& view,
                                                               std::uint32_t index) noexcept
            {
                ConvexHullView<Scalar> hull;
                if (view.pieces == nullptr || index >= view.piece_count)
                    return hull;
                const CollisionPieceRecord& piece = view.pieces[index];
                hull.vertices = view.hull_vertices + piece.first_vertex;
                hull.vertex_count = piece.vertex_count;
                hull.center = piece.center;
                hull.convex_radius = piece.convex_radius;
                return hull;
            }

            /**
             * @brief The asset's static triangle geometry as a shape, or an empty view.
             *
             * @param view A validated asset.
             * @return The mesh view, pointing into the blob; empty for a hull asset.
             */
            inline TriangleMeshView<Scalar> collision_asset_mesh(
                const CollisionAssetView& view) noexcept
            {
                TriangleMeshView<Scalar> mesh;
                if (!view.static_mesh || view.mesh_triangle_count == 0)
                    return mesh;
                mesh.vertices = view.mesh_vertices;
                mesh.indices = view.mesh_indices;
                mesh.nodes = view.mesh_nodes;
                mesh.order = view.mesh_order;
                mesh.adjacency = view.mesh_adjacency;
                mesh.triangle_count = view.mesh_triangle_count;
                mesh.node_count = view.mesh_node_count;
                return mesh;
            }

            /**
             * @brief The asset's collision geometry as line segments, for a viewport overlay.
             *
             * §14's "draws the actual collision geometry as an overlay so 'the collider is not
             * the mesh' is *visible*". A number in an inspector saying the collider is three
             * centimetres fatter is useful; seeing where is what stops someone spending an
             * afternoon on an invisible wall.
             *
             * Hull faces are **rebuilt** rather than read, because the asset deliberately does
             * not store them — the runtime never reads a hull face, so storing them would be
             * paying memory in every shipped asset for a debug view. `build_convex_hull_mesh`
             * reconstructs them from the stored point set, which is the same routine that
             * produced them at cook time and therefore cannot disagree with what was cooked.
             *
             * @param view  A validated asset.
             * @param out   Receives six floats per segment (two endpoints, asset frame);
             *              cleared first.
             * @return The number of segments written.
             */
            std::size_t collision_asset_wireframe(const CollisionAssetView& view,
                                                  std::vector<float>& out);

            /**
             * @brief Samples the asset's distance field at a point in the asset's frame.
             *
             * Nearest-voxel rather than trilinear, deliberately: the two callers §7.5 and
             * §9.6 name — deep-penetration recovery and "is this soft vertex inside me" —
             * both want a cheap sign and a rough depth, and a smooth gradient is a
             * different requirement that can add an interpolating overload when something
             * needs one.
             *
             * @param view  A validated asset.
             * @param point The query, in the asset's frame.
             * @return The signed distance, or zero when the asset carries no field.
             */
            inline Scalar collision_asset_distance(const CollisionAssetView& view,
                                                   const Vector3& point) noexcept
            {
                if (view.distances == nullptr || view.field_resolution == 0)
                    return 0;

                const std::int32_t resolution = std::int32_t(view.field_resolution);
                const Scalar component[3] = {point.x, point.y, point.z};
                std::int32_t voxel[3];
                for (int axis = 0; axis < 3; ++axis)
                {
                    const Scalar low = Scalar(view.field_min[axis]);
                    const Scalar span = Scalar(view.field_max[axis]) - low;
                    if (!(span > 0))
                        return 0;
                    const Scalar normalized = (component[axis] - low) / span;
                    std::int32_t index = std::int32_t(normalized * Scalar(resolution));
                    // Clamped rather than refused: a query outside the padded brick is
                    // ordinary (a body approaching from a distance) and the nearest boundary
                    // voxel is the right answer there, being the furthest-outside value.
                    if (index < 0)
                        index = 0;
                    if (index >= resolution)
                        index = resolution - 1;
                    voxel[axis] = index;
                }
                const std::size_t offset =
                    std::size_t(voxel[0]) +
                    std::size_t(resolution) *
                        (std::size_t(voxel[1]) + std::size_t(resolution) * std::size_t(voxel[2]));
                return Scalar(view.distances[offset]);
            }
        } // namespace Cooking
    } // namespace Physics
} // namespace SushiEngine
