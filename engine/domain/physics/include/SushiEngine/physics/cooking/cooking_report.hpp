/**************************************************************************/
/* cooking_report.hpp                                                     */
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
 * @file cooking_report.hpp
 * @brief What a cook produced, what it had to fix, and whether it should have shipped.
 *
 * §8.5, whose premise is worth restating because it shapes the whole type: *a silent
 * bad cook is the worst outcome*. A cooker that fails loudly costs an artist a
 * minute. A cooker that quietly produces a body which explodes on the first frame
 * costs whoever eventually debugs it a day, and the trail back to the import is cold
 * by then.
 *
 * So the report carries numbers rather than a verdict, and @ref CookingThresholds
 * turns numbers into the verdict separately. The split is what lets a project decide
 * that eight unembedded vertices are acceptable in a background prop and none are in
 * a hero vehicle, without either decision being compiled into the cooker.
 *
 * Every field here is either measured or absent. A field that is structurally always
 * zero is the same failure as a made-up number wearing the opposite mask, so a
 * quantity this pipeline cannot yet measure does not get a field to be zero in.
 */

#include <cstddef>
#include <cstdint>

#include <SushiEngine/geometry/mesh_utilities.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        namespace Cooking
        {
            /** @brief Why a cook failed, when it did. */
            enum class CookingStatus : std::uint32_t
            {
                /** @brief The asset was produced and passed every threshold. */
                Succeeded = 0,

                /** @brief The source mesh held no usable triangle at all. */
                EmptyInput,

                /** @brief A stage could not produce its output from the input it was given. */
                StageFailed,

                /** @brief The asset was produced and a configured threshold rejected it. */
                RejectedByThreshold,
            };

            /**
             * @brief What one cook measured.
             *
             * Read by the editor's collider and soft-body inspectors (§14) and by the
             * tests that assert the §15.1 invariants. Counts are exact; the two
             * distances are sampled and say so in their own documentation.
             */
            struct CookingReport
            {
                /** @brief Whether the asset shipped, and if not, why not. */
                CookingStatus status = CookingStatus::EmptyInput;

                /**
                 * @brief The stage that refused, when @ref status is @ref CookingStatus::StageFailed.
                 *
                 * A borrowed pointer to an @ref ICookingStage::name, which is a literal with
                 * static storage, so it outlives the report without the report owning a string.
                 */
                const char* failed_stage = nullptr;

                /**
                 * @brief The source mesh as it arrived, before any repair.
                 *
                 * **Unmeasured when @ref served_from_cache is set**, and left at its default
                 * then rather than filled with something plausible: a cache hit never looks
                 * at the source, so a clean-looking topology report would be an invention.
                 * The flag is the discriminator; read it before reading this.
                 */
                Geometry::MeshTopologyReport source;

                /** @brief What the repair stage changed; unmeasured on a cache hit, as @ref source. */
                Geometry::MeshRepairReport repair;

                /** @brief Convex pieces produced, or zero for a static triangle-mesh cook. */
                std::uint32_t convex_piece_count = 0;

                /** @brief Vertices in the largest convex piece; the narrowphase's per-pair cost. */
                std::uint32_t largest_piece_vertex_count = 0;

                /** @brief Triangles in the cooked static hierarchy, or zero for a hull cook. */
                std::uint32_t collision_triangle_count = 0;

                /** @brief Voxels per axis of the baked narrow-band distance field, or zero. */
                std::uint32_t distance_field_resolution = 0;

                /** @brief Tetrahedra in the finest simulation level, or zero for a rigid cook. */
                std::uint32_t tetrahedron_count = 0;

                /** @brief Simulation levels of detail produced, or zero for a rigid cook. */
                std::uint32_t level_of_detail_count = 0;

                /** @brief Nodes placed by a node-beam cook, or zero for any other. */
                std::uint32_t node_count = 0;

                /** @brief Beams connecting them, structural and bracing together. */
                std::uint32_t beam_count = 0;

                /**
                 * @brief How many of @ref beam_count are bracing diagonals.
                 *
                 * Reported separately because it is the number that says whether §11.3's
                 * diagonal rule fired at all. A structure that is all structural beams is
                 * an edge network with no shear rigidity — it will hold its length and
                 * fold flat — and the count is the only place that shows before someone
                 * drives it.
                 */
                std::uint32_t bracing_beam_count = 0;

                /**
                 * @brief Render-mesh vertices no simulation geometry could be found for.
                 *
                 * §8.6 invariant 1: no tetrahedron for a soft-body cook, no node within
                 * reach for a node-beam one. Above a threshold this fails the cook,
                 * because a vertex bound to nothing is a hole that opens the first time
                 * the body deforms — and it opens in the render mesh, where the
                 * simulation looks correct and the asset looks broken.
                 */
                std::uint32_t unembedded_vertex_count = 0;

                /**
                 * @brief Render vertices bound by extrapolation rather than containment.
                 *
                 * Not a failure and not counted by @ref unembedded_vertex_count — an
                 * extrapolated vertex is attached and moves correctly. It is reported because
                 * it is the *actionable* number the artist has: a large count means whole
                 * features are thinner than a voxel, and the answer is to raise the fidelity
                 * rather than to fix the mesh.
                 */
                std::uint32_t extrapolated_binding_count = 0;

                /** @brief Tetrahedra with inverted volume after optimization; must be zero. */
                std::uint32_t inverted_element_count = 0;

                /**
                 * @brief The worst element quality in the finest level, in [0, 1].
                 *
                 * One is a regular tetrahedron and zero is a sliver. Slivers destroy the
                 * conditioning of a finite-element solve, so this is the number §8.3
                 * stage 4 exists to raise and §17.5's risk row is watching.
                 */
                float worst_element_quality = 0.0f;

                /** @brief The cooked mass, in kilogrammes, at the authored density. */
                float mass = 0.0f;

                /** @brief The cooked centre of mass, in the asset's own frame. */
                float center_of_mass[3] = {0.0f, 0.0f, 0.0f};

                /** @brief The cooked principal moments of inertia, about that centre. */
                float principal_inertia[3] = {0.0f, 0.0f, 0.0f};

                /**
                 * @brief How far the collision geometry protrudes past the render mesh.
                 *
                 * In local units, and §7.6's number: "the collider is 3 cm fatter than
                 * the mesh" as a measurement rather than an unstated assumption. A
                 * **sampled lower bound** — the maximum is over a finite lattice, so a
                 * spike between samples is under-reported, and the lattice order is a
                 * cooking parameter for exactly that reason.
                 */
                float hausdorff_error = 0.0f;

                /**
                 * @brief The fraction of the source mesh's volume the collider adds or loses.
                 *
                 * Signed: positive means the collision geometry encloses more than the
                 * mesh does, which is the normal direction for a convex decomposition and
                 * the one that produces invisible walls.
                 */
                float volume_error = 0.0f;

                /** @brief Wall-clock milliseconds the cook took, measured by the caller. */
                float cook_milliseconds = 0.0f;

                /** @brief Bytes the serialized asset occupies. */
                std::size_t asset_bytes = 0;

                /** @brief Whether the asset came out of the cache rather than being cooked. */
                bool served_from_cache = false;

                /** @brief Whether an asset exists to load, whatever the thresholds said. */
                bool has_asset() const noexcept
                {
                    return status == CookingStatus::Succeeded ||
                           status == CookingStatus::RejectedByThreshold;
                }
            };

            /**
             * @brief The limits above which a produced asset must not ship.
             *
             * Defaults chosen to reject what is unambiguously broken and to pass what is
             * merely imperfect, because a threshold that fires on ordinary content gets
             * turned off and then protects nothing.
             */
            struct CookingThresholds
            {
                /**
                 * @brief Unembedded render vertices tolerated before the cook fails.
                 *
                 * Zero: a vertex bound to nothing tears the render mesh, and §8.6's
                 * fallback binds by *extrapolated* barycentric coordinates precisely so
                 * that this count can be zero for any mesh with a tetrahedron anywhere
                 * near it. A non-zero count means the lattice missed a whole feature.
                 */
                std::uint32_t max_unembedded_vertices = 0;

                /** @brief Inverted elements tolerated; zero, since one is a solve that diverges. */
                std::uint32_t max_inverted_elements = 0;

                /**
                 * @brief The worst element quality accepted in a soft-body cook.
                 *
                 * Low enough that an ordinary lattice passes and a collapsed one does
                 * not. Raising it is how a project trades cook failures for solve
                 * stability.
                 */
                float min_element_quality = 0.01f;

                /**
                 * @brief The largest collider protrusion accepted, in local units.
                 *
                 * Zero or less disables the check, which is the right default for a
                 * pipeline whose assets are authored at wildly different scales: a
                 * five-centimetre limit is generous on a car and absurd on a doorknob,
                 * so the number belongs to the project rather than to the cooker.
                 */
                float max_hausdorff_error = 0.0f;

                /** @brief Whether a non-watertight source mesh fails the cook outright. */
                bool require_watertight_source = false;
            };

            /**
             * @brief Applies @p thresholds to @p report, setting its status.
             *
             * Applied after the asset is built rather than before, so a rejected cook
             * still has an asset and a report to show: an artist told "this failed" needs
             * to see the geometry that failed, and re-cooking to look at it would be a
             * different cook.
             *
             * @param thresholds The project's limits.
             * @param report     The measured report; its @c status is overwritten unless
             *                   it already records a hard failure, which a threshold
             *                   cannot un-fail.
             * @return True when the asset passed every threshold.
             */
            inline bool apply_cooking_thresholds(const CookingThresholds& thresholds,
                                                 CookingReport& report) noexcept
            {
                if (report.status == CookingStatus::EmptyInput ||
                    report.status == CookingStatus::StageFailed)
                    return false;

                bool passed = true;
                if (report.unembedded_vertex_count > thresholds.max_unembedded_vertices)
                    passed = false;
                if (report.inverted_element_count > thresholds.max_inverted_elements)
                    passed = false;
                // Only a cook that produced elements has an element quality to judge.
                if (report.tetrahedron_count > 0 &&
                    report.worst_element_quality < thresholds.min_element_quality)
                    passed = false;
                if (thresholds.max_hausdorff_error > 0.0f &&
                    report.hausdorff_error > thresholds.max_hausdorff_error)
                    passed = false;
                if (thresholds.require_watertight_source && !report.source.watertight())
                    passed = false;

                report.status =
                    passed ? CookingStatus::Succeeded : CookingStatus::RejectedByThreshold;
                return passed;
            }
        } // namespace Cooking
    } // namespace Physics
} // namespace SushiEngine
