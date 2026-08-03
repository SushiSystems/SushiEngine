/**************************************************************************/
/* cooking_parameters.hpp                                                 */
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
 * @file cooking_parameters.hpp
 * @brief One dial the artist turns, and the seven numbers it resolves to.
 *
 * §8.2's fidelity dial. The user's stated requirement is that dropping a mesh in
 * produces a simulation asset "ve bunun hassasiyetini ayarlayabiliyoruz" — and one
 * knob is the whole point: an artist asked to choose a voxel resolution, a tetrahedron
 * budget, a convex piece count and a distance-field resolution independently will
 * choose a combination nobody has ever measured.
 *
 * So @ref CookingParameters carries the dial and @ref resolve_cooking_parameters
 * turns it into @ref DerivedCookingParameters, which is what every stage reads. Any
 * one derived field can still be pinned by hand — the engineer keeps the override the
 * artist does not want — and a pinned field is recorded in the parameters hash like
 * everything else, so pinning one does not quietly reuse a cache entry cooked without
 * it.
 *
 * **Why the interpolation is geometric.** Voxel resolution runs 16 to 256 and the
 * tetrahedron budget 200 to 120 000. Interpolated linearly, half the dial's travel
 * buys almost nothing and the last tenth buys everything, because cost grows with the
 * cube of a resolution. Geometric interpolation makes each equal step of the dial a
 * constant *factor* on the result, which is the only mapping under which "0.5" means
 * something an artist can learn. The three small counts — levels of detail, conforming
 * passes, substeps — interpolate linearly, because 1 to 4 has no factor worth
 * preserving and rounding a geometric curve across four values just moves the steps
 * somewhere less obvious.
 */

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include <SushiEngine/geometry/triangle_mesh.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        namespace Cooking
        {
            /** @brief An override field's value meaning "take it from the dial". */
            constexpr std::int32_t DERIVE_FROM_FIDELITY = -1;

            /**
             * @brief What the cooker is being asked to produce, and how accurately.
             *
             * Trivially copyable and hashable: this record is half of a cache key
             * (§8.1), so it must not grow a pointer, a string, or a field whose value
             * differs between two runs that should hit the same entry.
             */
            struct CookingParameters
            {
                /**
                 * @brief The dial, in [0, 1]; clamped rather than rejected.
                 *
                 * Clamped because this arrives from a slider and a cook that refuses to
                 * run because a value read 1.0000001 is a worse outcome than one that
                 * runs at maximum fidelity.
                 */
                float fidelity = 0.5f;

                /** @brief Voxels along the longest axis, or @ref DERIVE_FROM_FIDELITY. */
                std::int32_t voxel_resolution = DERIVE_FROM_FIDELITY;

                /** @brief Target tetrahedron count, or @ref DERIVE_FROM_FIDELITY. */
                std::int32_t target_tetrahedron_count = DERIVE_FROM_FIDELITY;

                /** @brief Simulation levels of detail, or @ref DERIVE_FROM_FIDELITY. */
                std::int32_t simulation_level_count = DERIVE_FROM_FIDELITY;

                /** @brief Maximum convex pieces for a rigid cook, or @ref DERIVE_FROM_FIDELITY. */
                std::int32_t convex_piece_count = DERIVE_FROM_FIDELITY;

                /** @brief Distance-field voxels per axis, or @ref DERIVE_FROM_FIDELITY. */
                std::int32_t distance_field_resolution = DERIVE_FROM_FIDELITY;

                /** @brief Surface-conforming passes, or @ref DERIVE_FROM_FIDELITY. */
                std::int32_t surface_conforming_passes = DERIVE_FROM_FIDELITY;

                /** @brief Suggested substeps for a body of this asset, or @ref DERIVE_FROM_FIDELITY. */
                std::int32_t suggested_substep_count = DERIVE_FROM_FIDELITY;

                /**
                 * @brief Vertices a single cooked convex piece may keep.
                 *
                 * Not on the dial. A hull's vertex count is bounded by what the
                 * narrowphase can afford to iterate per pair rather than by how
                 * accurate the artist wants the asset, and those are different
                 * questions with different owners.
                 */
                std::int32_t hull_vertex_budget = 32;

                /** @brief Weld distance for the repair stage, in the asset's own units. */
                float weld_tolerance = 1.0e-5f;

                /**
                 * @brief Mass per unit volume, for the cooked mass properties.
                 *
                 * A number rather than a material index, because the cooker must not
                 * need a scene to run: an importer on a build machine has no material
                 * table. The scene's material overrides it at instancing.
                 */
                float density = 1000.0f;

                /**
                 * @brief Sampling lattice order for the accuracy report.
                 *
                 * Governs the reported Hausdorff error's tightness, not the asset, so a
                 * cook whose only change is this number produces identical geometry with
                 * a different report — which is why it is still in the hash.
                 */
                std::int32_t accuracy_lattice_order = 4;

                /** @brief Whether to cook a rigid collision asset. */
                bool cook_collision = true;

                /** @brief Whether to cook a soft-body asset. */
                bool cook_soft_body = false;

                /**
                 * @brief Whether to cook a node-beam asset (§11.3).
                 *
                 * Off by default for the same reason @ref cook_soft_body is: a node-beam
                 * cook is minutes rather than milliseconds and wanted by vehicles, not by
                 * the rest of a project's meshes.
                 */
                bool cook_node_beam = false;

                /**
                 * @brief Whether the source geometry is authored static.
                 *
                 * Static geometry skips convex decomposition entirely and cooks a
                 * triangle-mesh hierarchy instead (§8.4 item 4), which is both cheaper
                 * and exact. Decomposing a level's terrain is spending minutes to
                 * produce a worse collider.
                 */
                bool static_geometry = false;
            };

            /** @brief Every resolution a stage reads, with nothing left to derive. */
            struct DerivedCookingParameters
            {
                std::int32_t voxel_resolution = 0;
                std::int32_t target_tetrahedron_count = 0;
                std::int32_t simulation_level_count = 0;
                std::int32_t convex_piece_count = 0;
                std::int32_t distance_field_resolution = 0;
                std::int32_t surface_conforming_passes = 0;
                std::int32_t suggested_substep_count = 0;
                std::int32_t hull_vertex_budget = 0;
                std::int32_t accuracy_lattice_order = 0;
                float weld_tolerance = 0.0f;
                float density = 0.0f;
                bool static_geometry = false;
            };

            namespace Detail
            {
                /** @brief @p value held inside [@p low, @p high]. */
                inline float clamp_unit(float value) noexcept
                {
                    if (!(value > 0.0f))     // catches a not-a-number too
                        return 0.0f;
                    return value > 1.0f ? 1.0f : value;
                }

                /**
                 * @brief Geometric interpolation from @p low to @p high, rounded.
                 *
                 * `low * (high/low)^t`, so equal steps of the dial are equal factors on
                 * the result. Rounded to nearest rather than truncated: truncation makes
                 * fidelity 1 land one below the documented maximum, and a table in the
                 * design document that the code misses by one is worse than either.
                 */
                inline std::int32_t geometric(std::int32_t low, std::int32_t high,
                                              float t) noexcept
                {
                    const float value = float(low) * std::pow(float(high) / float(low), t);
                    return std::int32_t(value + 0.5f);
                }

                /** @brief Linear interpolation from @p low to @p high, rounded. */
                inline std::int32_t linear(std::int32_t low, std::int32_t high, float t) noexcept
                {
                    return std::int32_t(float(low) + (float(high) - float(low)) * t + 0.5f);
                }

                /** @brief @p override_value when it is not @ref DERIVE_FROM_FIDELITY. */
                inline std::int32_t pinned_or(std::int32_t override_value,
                                              std::int32_t derived) noexcept
                {
                    return override_value == DERIVE_FROM_FIDELITY ? derived : override_value;
                }
            } // namespace Detail

            /**
             * @brief Resolves the dial into the numbers the stages read.
             *
             * The endpoints are §8.2's table verbatim. Reading them out of code rather
             * than out of prose is the point of this function existing: the document and
             * the cooker disagreeing about what fidelity 0.5 means is a bug nobody can
             * see, because both halves look right on their own.
             *
             * @param parameters The authored dial and any pinned overrides.
             * @return Every derived resolution, with overrides applied.
             */
            inline DerivedCookingParameters
            resolve_cooking_parameters(const CookingParameters& parameters) noexcept
            {
                const float t = Detail::clamp_unit(parameters.fidelity);

                DerivedCookingParameters derived;
                derived.voxel_resolution = Detail::pinned_or(parameters.voxel_resolution,
                                                             Detail::geometric(16, 256, t));
                derived.target_tetrahedron_count =
                    Detail::pinned_or(parameters.target_tetrahedron_count,
                                      Detail::geometric(200, 120000, t));
                derived.simulation_level_count =
                    Detail::pinned_or(parameters.simulation_level_count, Detail::linear(1, 4, t));
                derived.convex_piece_count =
                    Detail::pinned_or(parameters.convex_piece_count, Detail::geometric(4, 64, t));
                derived.distance_field_resolution =
                    Detail::pinned_or(parameters.distance_field_resolution,
                                      Detail::geometric(16, 128, t));
                derived.surface_conforming_passes =
                    Detail::pinned_or(parameters.surface_conforming_passes,
                                      Detail::linear(0, 3, t));
                derived.suggested_substep_count =
                    Detail::pinned_or(parameters.suggested_substep_count,
                                      Detail::geometric(8, 32, t));

                // Not on the dial, but still resolved here so a stage has exactly one
                // record to read and cannot reach past it to the authored parameters.
                derived.hull_vertex_budget =
                    parameters.hull_vertex_budget > 3 ? parameters.hull_vertex_budget : 4;
                derived.accuracy_lattice_order =
                    parameters.accuracy_lattice_order > 0 ? parameters.accuracy_lattice_order : 1;
                derived.weld_tolerance =
                    parameters.weld_tolerance > 0.0f ? parameters.weld_tolerance : 0.0f;
                derived.density = parameters.density > 0.0f ? parameters.density : 0.0f;
                derived.static_geometry = parameters.static_geometry;
                return derived;
            }

            /**
             * @brief FNV-1a 64 over a value's bytes, folded into @p seed.
             *
             * Bytes rather than fields, because a hash written field by field acquires a
             * bug the day someone adds a field and forgets this function — and that bug
             * is a cache that serves an asset cooked with the old parameters.
             *
             * @tparam T A trivially copyable type.
             * @param seed  The running hash.
             * @param value The value to fold in.
             * @return The updated hash.
             */
            template <typename T>
            inline std::uint64_t hash_bytes(std::uint64_t seed, const T& value) noexcept
            {
                static_assert(std::is_trivially_copyable<T>::value,
                              "hash_bytes reads a value's object representation");
                const unsigned char* bytes = reinterpret_cast<const unsigned char*>(&value);
                for (std::size_t i = 0; i < sizeof(T); ++i)
                {
                    seed ^= std::uint64_t(bytes[i]);
                    seed *= 1099511628211ull;
                }
                return seed;
            }

            /**
             * @brief The parameters half of a cook's cache key (§8.1).
             *
             * Hashes the *resolved* parameters rather than the authored ones, and that
             * choice is the interesting half. Two cooks that resolve to the same seven
             * numbers produce byte-identical assets, so they must share a cache entry
             * even when their dials read 0.500 and 0.501 — otherwise a slider drag
             * re-cooks at every pixel it passes through. The fidelity value itself is
             * *not* folded in for the same reason.
             *
             * @param parameters The authored parameters; resolved internally.
             * @return A hash that changes when and only when the produced asset would.
             */
            inline std::uint64_t cooking_parameters_hash(const CookingParameters& parameters) noexcept
            {
                const DerivedCookingParameters derived = resolve_cooking_parameters(parameters);
                std::uint64_t hash = 1469598103934665603ull;
                hash = hash_bytes(hash, derived.voxel_resolution);
                hash = hash_bytes(hash, derived.target_tetrahedron_count);
                hash = hash_bytes(hash, derived.simulation_level_count);
                hash = hash_bytes(hash, derived.convex_piece_count);
                hash = hash_bytes(hash, derived.distance_field_resolution);
                hash = hash_bytes(hash, derived.surface_conforming_passes);
                hash = hash_bytes(hash, derived.suggested_substep_count);
                hash = hash_bytes(hash, derived.hull_vertex_budget);
                hash = hash_bytes(hash, derived.accuracy_lattice_order);
                hash = hash_bytes(hash, derived.weld_tolerance);
                hash = hash_bytes(hash, derived.density);
                hash = hash_bytes(hash, derived.static_geometry);
                // Which cookers ran is part of the key: the same geometry asked for a
                // collision asset and for a soft body is two different outputs.
                hash = hash_bytes(hash, parameters.cook_collision);
                hash = hash_bytes(hash, parameters.cook_soft_body);
                hash = hash_bytes(hash, parameters.cook_node_beam);
                return hash;
            }

            /**
             * @brief FNV-1a 64 over a mesh's geometry — the source half of a cache key.
             *
             * Over the positions and indices only, and in the mesh's own order, so the
             * same file imported twice hashes the same and a mesh whose vertices moved
             * by a float does not. Normals are excluded because no cooking stage reads
             * them: hashing them would invalidate a cache entry over a change that
             * cannot affect the output.
             *
             * @param mesh The geometry to hash.
             * @return The hash, or the unseeded basis for an empty mesh.
             */
            std::uint64_t mesh_content_hash(const Geometry::TriangleMeshView& mesh) noexcept;
        } // namespace Cooking
    } // namespace Physics
} // namespace SushiEngine
