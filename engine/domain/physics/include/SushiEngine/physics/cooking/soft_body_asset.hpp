/**************************************************************************/
/* soft_body_asset.hpp                                                    */
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
 * @file soft_body_asset.hpp
 * @brief The `.sushisoft` blob: a mesh's simulable interior, cooked once.
 *
 * §5.4's second row. Larger than the collision asset because it carries four things a
 * collider does not — a tetrahedral interior, the render mesh's embedding in it, a chain
 * of coarser lattices with the mappings between them, and the cooking parameters that
 * produced all three.
 *
 * **The parameters are inside the blob on purpose** (§8.3 stage 10). A cooked asset whose
 * settings live somewhere else is an asset nobody can reproduce: the dial moved, the
 * project file was not committed, and the only record of what made these 4 000 tetrahedra
 * is gone. Carrying them makes a re-cook reproducible and a mismatch detectable.
 *
 * **One limitation of the parameters section, stated because it is measurable.** The
 * section is a struct copy of @ref CookingParameters, whose members leave one byte of tail
 * padding. A `memcpy` carries whatever the caller's object held there, so two cooks of the
 * same input produce assets that are semantically identical and may differ in that one
 * byte. Nothing depends on it — the §8.1 cache keys on hashes of the *inputs*, not on the
 * output's bytes — and the alternative was a declared reserved field, which is the "dead
 * field" this codebase forbids. It is recorded rather than fixed.
 *
 * **Nothing consumes this yet, and that is worth stating rather than leaving to be
 * discovered.** The finite-element model, the plasticity and the fracture that read a
 * tetrahedral mesh are **P6**; `physics/soft/` today holds a mass-spring box lattice and
 * no element solver. So this asset's status is honestly "produced and validated, not yet
 * consumed" — which is precisely the distinction §16.10 was written about, made in advance
 * this time instead of in a later audit.
 */

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/geometry/signed_distance_field.hpp>
#include <SushiEngine/physics/cooking/cooking_parameters.hpp>
#include <SushiEngine/physics/geometry/mesh_bvh.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        namespace Cooking
        {
            /** @brief Magic at offset 0 of a `.sushisoft` blob. */
            constexpr char SOFT_BLOB_MAGIC[8] = {'S', 'U', 'S', 'H', 'S', 'O', 'F', 'T'};

            /** @brief Current `.sushisoft` format version. */
            constexpr std::uint32_t SOFT_BLOB_VERSION = 1;

            /**
             * @brief One point driven by four tetrahedron vertices.
             *
             * Used for both bindings the asset carries — a render vertex into the finest
             * lattice, and a coarse lattice's vertex into the next finer one — because they
             * are the same question asked of different points, and giving them two records
             * would give them two chances to disagree about weight order.
             *
             * Twenty bytes with no interior padding, so an array of them serializes as a
             * `memcpy` and the blob stays byte-reproducible.
             */
            struct SoftBodyBinding
            {
                /** @brief Index into the target level's tetrahedra. */
                std::uint32_t tetrahedron;

                /** @brief Barycentric weights summing to one; negative where extrapolated. */
                float weights[4];
            };

            /**
             * @brief One simulation level: where its arrays sit and what grid produced it.
             *
             * Levels are stored back to back in shared arrays rather than as separate
             * sections per level, so the section count does not depend on the fidelity dial.
             * Level zero is the finest and the one the render mesh is embedded in.
             */
            struct SoftBodyLevelRecord
            {
                std::uint32_t first_vertex;
                std::uint32_t vertex_count;
                std::uint32_t first_tetrahedron;
                std::uint32_t tetrahedron_count;
                std::uint32_t first_surface_index;
                std::uint32_t surface_index_count;

                /**
                 * @brief The next *finer* level's vertices, bound into **this** level's elements.
                 *
                 * Zero-length for level zero, which has nothing finer to reconstruct. The
                 * direction follows from what a level of detail is for: the coarse lattice is
                 * the one being simulated, and the finer one has to be reconstructed from it
                 * so the chain reaches the render mesh (§8.3 stage 9). It is the render-mesh
                 * embedding again with different points, which is why it is the same record.
                 */
                std::uint32_t first_mapping;
                std::uint32_t mapping_count;

                /** @brief The grid cell size this level was voxelized at. */
                Scalar cell_size;

                /** @brief World position of the level's grid corner (0, 0, 0). */
                Vector3 grid_origin;

                /** @brief Grid cells per axis. */
                std::int32_t grid[3];
            };

            /** @brief What the body weighs and how it spins, for the rigid fallback (§9.7). */
            struct SoftBodySummary
            {
                /** @brief Mass at the cooked density, in kilogrammes. */
                Scalar mass;

                /** @brief Centre of mass in the asset's own frame. */
                Vector3 center_of_mass;

                /** @brief Principal moments about @ref center_of_mass. */
                Vector3 principal_inertia;

                /** @brief Rotation from the principal frame to the asset's frame. */
                Quaternion principal_rotation;

                /** @brief Summed rest volume of the finest level's elements. */
                Scalar volume;

                /** @brief The worst element quality in the finest level, in [0, 1]. */
                float worst_element_quality;

                /** @brief How far the simulation surface departs from the source mesh. */
                float hausdorff_error;

                /** @brief Render vertices bound by extrapolation rather than containment. */
                std::uint32_t extrapolated_binding_count;

                /** @brief Substeps the fidelity dial suggests for a body of this asset. */
                std::uint32_t suggested_substep_count;
            };

            /**
             * @brief The fixed header at offset 0.
             *
             * Counts and byte offsets only, so the blob is position independent. The
             * offsets are produced by one section-layout pass rather than by a hand-written
             * expression per section, which is where a format this wide otherwise acquires
             * an off-by-one nobody notices until a section reads its neighbour.
             */
            struct SoftBlobHeader
            {
                char magic[8];
                std::uint32_t version;
                std::uint32_t total_size;

                std::uint32_t level_count;
                std::uint32_t vertex_count;         /**< Summed across levels. */
                std::uint32_t tetrahedron_count;    /**< Summed across levels. */
                std::uint32_t surface_index_count;  /**< Summed across levels. */
                std::uint32_t binding_count;        /**< Render-mesh bindings. */
                std::uint32_t mapping_count;        /**< Inter-level bindings, summed. */
                std::uint32_t surface_node_count;   /**< Level zero's hierarchy. */
                std::uint32_t surface_triangle_count;
                std::uint32_t field_resolution;

                std::uint32_t parameters_offset;    /**< CookingParameters. */
                std::uint32_t summary_offset;       /**< SoftBodySummary. */
                std::uint32_t levels_offset;        /**< SoftBodyLevelRecord[level_count]. */
                std::uint32_t vertices_offset;      /**< Vector3[vertex_count]. */
                std::uint32_t vertex_mass_offset;   /**< Scalar[vertex_count]. */
                std::uint32_t tetrahedra_offset;    /**< uint32[4 * tetrahedron_count]. */
                std::uint32_t rest_inverse_offset;  /**< Vector3[3 * tetrahedron_count]. */
                std::uint32_t rest_volume_offset;   /**< Scalar[tetrahedron_count]. */
                std::uint32_t surface_indices_offset;/**< uint32[surface_index_count]. */
                std::uint32_t bindings_offset;      /**< SoftBodyBinding[binding_count]. */
                std::uint32_t mappings_offset;      /**< SoftBodyBinding[mapping_count]. */
                std::uint32_t surface_nodes_offset; /**< MeshBvhNode<Scalar>[surface_node_count]. */
                std::uint32_t surface_order_offset; /**< uint32[surface_triangle_count]. */
                std::uint32_t surface_adjacency_offset; /**< uint32[3 * surface_triangle_count]. */
                std::uint32_t field_offset;         /**< float[resolution^3]. */

                float field_min[3];
                float field_max[3];
            };

            /** @brief The arrays a cooked soft-body asset owns, before serialization. */
            struct SoftBodyAsset
            {
                std::vector<SoftBodyLevelRecord> levels;
                std::vector<Vector3> vertices;
                std::vector<Scalar> vertex_mass;
                std::vector<std::uint32_t> tetrahedra;
                std::vector<Vector3> rest_inverse;
                std::vector<Scalar> rest_volume;
                std::vector<std::uint32_t> surface_indices;
                std::vector<SoftBodyBinding> bindings;
                std::vector<SoftBodyBinding> mappings;
                std::vector<MeshBvhNode<Scalar>> surface_nodes;
                std::vector<std::uint32_t> surface_order;
                std::vector<std::uint32_t> surface_adjacency;
                Geometry::SignedDistanceFieldBrick field;
                CookingParameters parameters{};
                SoftBodySummary summary{};
            };

            /** @brief A view over a validated blob; every pointer is into the bytes. */
            struct SoftBodyAssetView
            {
                const SoftBodyLevelRecord* levels = nullptr;
                std::uint32_t level_count = 0;

                const Vector3* vertices = nullptr;
                const Scalar* vertex_mass = nullptr;
                std::uint32_t vertex_count = 0;

                const std::uint32_t* tetrahedra = nullptr;
                const Vector3* rest_inverse = nullptr;
                const Scalar* rest_volume = nullptr;
                std::uint32_t tetrahedron_count = 0;

                const std::uint32_t* surface_indices = nullptr;
                std::uint32_t surface_index_count = 0;

                const SoftBodyBinding* bindings = nullptr;
                std::uint32_t binding_count = 0;

                const SoftBodyBinding* mappings = nullptr;
                std::uint32_t mapping_count = 0;

                const MeshBvhNode<Scalar>* surface_nodes = nullptr;
                const std::uint32_t* surface_order = nullptr;
                const std::uint32_t* surface_adjacency = nullptr;
                std::uint32_t surface_node_count = 0;
                std::uint32_t surface_triangle_count = 0;

                const float* distances = nullptr;
                std::uint32_t field_resolution = 0;
                float field_min[3] = {0.0f, 0.0f, 0.0f};
                float field_max[3] = {0.0f, 0.0f, 0.0f};

                CookingParameters parameters{};
                SoftBodySummary summary{};
                bool valid = false;
            };

            /**
             * @brief Serializes @p asset into a `.sushisoft` blob.
             *
             * Refuses rather than writes a blob it would not itself load. The checks are
             * about *cross-references*, since that is what a format this wide gets wrong: a
             * level naming a vertex range past the end, a tetrahedron naming a vertex outside
             * its own level, a binding naming an element that does not exist. Each of those
             * unchecked is a read into a neighbouring section, which produces geometry rather
             * than a crash and is therefore the worst possible failure.
             *
             * @param asset The cooked arrays.
             * @param out   Receives the blob bytes; cleared first, left empty on refusal.
             * @return False when @p asset is not well formed.
             */
            bool build_soft_body_blob(const SoftBodyAsset& asset, std::vector<std::byte>& out);

            /**
             * @brief Whether @p data is a blob this build can load.
             *
             * @param data The blob bytes.
             * @param size Their length.
             * @return True when @ref load_soft_body_blob will produce a usable view.
             */
            bool validate_soft_body_blob(const std::byte* data, std::size_t size) noexcept;

            /**
             * @brief Rebuilds a view over a validated blob.
             *
             * @param data The blob bytes.
             * @param size Their length.
             * @return A view, or a default (invalid) one when the blob does not validate.
             */
            SoftBodyAssetView load_soft_body_blob(const std::byte* data, std::size_t size) noexcept;

            /**
             * @brief Reads a binding's weights into the solver's precision, renormalized.
             *
             * The blob stores weights as `float` so a binding is twenty bytes and an
             * array of them serializes as a `memcpy`. That is the right trade for the
             * file, and it has one consequence every reader has to handle: four
             * `float`s that summed to one in the cooker do **not** sum to one after
             * the round trip, they sum to one within about six decimal digits.
             *
             * That residue is not harmless, because the reconstruction it feeds is
             * `sum(weight * position)` over *absolute* positions. A shortfall of
             * `1e-7` against a body sitting a hundred metres from the origin displaces
             * the reconstructed point by ten micrometres; at planet scale it is
             * centimetres, and it appears as the render mesh detaching from the
             * simulation the further the world is from its origin — a bug that
             * reproduces nowhere near the origin, which is where it would be tested.
             *
             * Renormalizing here makes every reader immune by construction rather
             * than each remembering, and costs one division per binding at load time
             * against a per-tick error.
             *
             * @param binding The cooked binding.
             * @param out     Receives the four weights, summing to one in @c T.
             */
            template <typename T>
            inline void read_binding_weights(const SoftBodyBinding& binding, T out[4]) noexcept
            {
                T total = 0;
                for (int i = 0; i < 4; ++i)
                {
                    out[i] = T(binding.weights[i]);
                    total += out[i];
                }
                // A binding whose weights sum to nothing is not a binding; leaving it
                // alone lets the caller's own validity test see it unchanged rather
                // than turning it into an infinity here.
                if (total > T(1e-6) || total < T(-1e-6))
                    for (int i = 0; i < 4; ++i)
                        out[i] /= total;
            }

            /**
             * @brief Where a bound point is, given the level it is bound into.
             *
             * The per-tick reconstruction, in scalar form — `sum(weight[i] * vertex[i])` —
             * against the asset's own arrays rather than against a copy of them, so a test
             * asserting §8.6 invariant 2 measures the expression the runtime will use.
             *
             * @param view    A validated asset.
             * @param level   Which level's lattice drives the point.
             * @param binding The point's binding.
             * @return The reconstructed position; the origin for an out-of-range binding.
             */
            Vector3 evaluate_soft_binding(const SoftBodyAssetView& view, std::uint32_t level,
                                          const SoftBodyBinding& binding) noexcept;
        } // namespace Cooking
    } // namespace Physics
} // namespace SushiEngine
