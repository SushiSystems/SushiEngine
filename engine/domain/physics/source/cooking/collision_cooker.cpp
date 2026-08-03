/**************************************************************************/
/* collision_cooker.cpp                                                   */
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

#include <SushiEngine/physics/cooking/collision_cooker.hpp>

#include <algorithm>
#include <cmath>
#include <memory>

#include <SushiEngine/geometry/mesh_utilities.hpp>
#include <SushiEngine/physics/geometry/mesh_mass_properties.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        namespace Cooking
        {
            namespace
            {
                using Stage = ICookingStage<CollisionCookContext>;

                /**
                 * @brief Stage one: measure the source, then repair it.
                 *
                 * Every later stage reads the repaired mesh, because welding is what makes
                 * adjacency meaningful and adjacency is what the hierarchy, the
                 * decomposition and the mass integration all rest on. The *unrepaired*
                 * topology is what the report carries, since that is the fact the artist
                 * needs (§8.3 stage 1).
                 */
                class RepairStage final : public Stage
                {
                public:
                    const char* name() const noexcept override { return "Repair"; }

                    bool run(CollisionCookContext& context) override
                    {
                        context.report.source =
                            Geometry::analyze_mesh_topology(context.source);

                        Geometry::MeshRepairOptions options;
                        options.weld_tolerance = context.parameters.weld_tolerance;
                        context.report.repair = Geometry::repair_mesh(context.source, options,
                                                                     context.repaired);
                        if (context.repaired.triangle_count() == 0)
                            return false;
                        return context.surface.build(context.repaired.view());
                    }
                };

                /**
                 * @brief Stage two: what the asset weighs and how it spins.
                 *
                 * Integrated over the repaired *source* mesh and not over the pieces, per
                 * §8.4 item 1 — so the decomposition's approximation never reaches what a
                 * body weighs, only what it bumps into.
                 *
                 * An open mesh encloses no volume and yields a zero mass, which is left as
                 * zero rather than guessed at. Zero density already means "keep the authored
                 * mass" everywhere downstream, so a wall that is a single-sided sheet
                 * degrades to hand-authored values instead of to a body of no mass.
                 */
                class MassPropertiesStage final : public Stage
                {
                public:
                    const char* name() const noexcept override { return "MassProperties"; }

                    bool run(CollisionCookContext& context) override
                    {
                        const MeshMassProperties properties = mesh_mass_properties(
                            context.repaired.view(), Scalar(context.parameters.density));

                        CollisionAssetSummary& summary = context.asset.summary;
                        if (properties.valid)
                        {
                            summary.mass = properties.properties.mass;
                            summary.center_of_mass = properties.properties.center_of_mass;
                            summary.principal_inertia = properties.properties.inertia;
                            summary.principal_rotation = properties.principal_rotation;
                            summary.volume = properties.volume;
                        }
                        summary.suggested_substep_count =
                            std::uint32_t(context.parameters.suggested_substep_count);

                        context.report.mass = float(summary.mass);
                        context.report.center_of_mass[0] = float(summary.center_of_mass.x);
                        context.report.center_of_mass[1] = float(summary.center_of_mass.y);
                        context.report.center_of_mass[2] = float(summary.center_of_mass.z);
                        context.report.principal_inertia[0] = float(summary.principal_inertia.x);
                        context.report.principal_inertia[1] = float(summary.principal_inertia.y);
                        context.report.principal_inertia[2] = float(summary.principal_inertia.z);
                        // Not a failure: a non-watertight collider is ordinary content, and
                        // `require_watertight_source` is the threshold that decides whether
                        // this project accepts one.
                        return true;
                    }
                };

                /** @brief Stage three, dynamic: decompose into convex pieces (§8.4 item 2). */
                class DecomposeStage final : public Stage
                {
                public:
                    const char* name() const noexcept override { return "Decompose"; }

                    bool run(CollisionCookContext& context) override
                    {
                        ConvexDecompositionOptions options;
                        options.max_pieces = context.parameters.convex_piece_count;
                        options.vertex_budget = context.parameters.hull_vertex_budget;
                        options.accuracy_lattice_order =
                            std::uint32_t(context.parameters.accuracy_lattice_order);
                        // Scaled to the asset rather than absolute: a millimetre is a
                        // reasonable error on a car and an absurd one on a doorknob, and the
                        // decomposition has the asset in front of it while the dial does not.
                        float minimum[3];
                        float maximum[3];
                        float extent = 1.0f;
                        if (Geometry::compute_bounds(context.repaired.view(), minimum, maximum))
                        {
                            extent = std::max(maximum[0] - minimum[0],
                                              std::max(maximum[1] - minimum[1],
                                                       maximum[2] - minimum[2]));
                        }
                        options.concavity_tolerance = extent * 0.02f;

                        const ConvexDecompositionReport decomposition = decompose_convex(
                            context.repaired.view(), context.surface, options, context.pieces);
                        if (context.pieces.empty())
                            return false;

                        context.asset.static_mesh = false;
                        context.asset.pieces.reserve(context.pieces.size());
                        for (const ConvexPiece& piece : context.pieces)
                        {
                            CollisionPieceRecord record{};
                            record.center = piece.center;
                            // Left at zero on purpose. §5.2's inflation is only sound
                            // alongside shrinking the vertices by the same amount, which
                            // means offsetting the hull inward along its own planes; setting
                            // the radius without the shrink would make every cooked collider
                            // fatter than the mesh it was cooked from, which is the one error
                            // §7.6 exists to keep at zero.
                            record.convex_radius = 0;
                            record.volume = piece.volume;
                            record.first_vertex =
                                std::uint32_t(context.asset.hull_vertices.size());
                            record.vertex_count = std::uint32_t(piece.vertices.size());
                            context.asset.pieces.push_back(record);
                            context.asset.hull_vertices.insert(context.asset.hull_vertices.end(),
                                                               piece.vertices.begin(),
                                                               piece.vertices.end());
                        }

                        context.report.convex_piece_count = decomposition.piece_count;
                        context.report.largest_piece_vertex_count =
                            decomposition.largest_piece_vertex_count;
                        context.report.hausdorff_error = decomposition.worst_concavity;
                        context.asset.summary.hausdorff_error = decomposition.worst_concavity;

                        const Scalar mesh_volume = context.asset.summary.volume;
                        if (mesh_volume > 0)
                        {
                            context.report.volume_error =
                                float(decomposition.summed_volume / mesh_volume - Scalar(1));
                        }
                        context.asset.summary.volume_error = context.report.volume_error;
                        return true;
                    }
                };

                /**
                 * @brief Stage three, static: cook the triangle hierarchy instead (§8.4 item 4).
                 *
                 * Exact and cheaper. The error fields are genuinely zero here rather than
                 * unmeasured: the collider *is* the mesh, so it protrudes nowhere and
                 * encloses exactly what the mesh does.
                 */
                class StaticHierarchyStage final : public Stage
                {
                public:
                    const char* name() const noexcept override { return "StaticHierarchy"; }

                    bool run(CollisionCookContext& context) override
                    {
                        const std::size_t vertices = context.repaired.vertex_count();
                        const std::size_t triangles = context.repaired.triangle_count();
                        if (vertices == 0 || triangles == 0)
                            return false;

                        context.asset.static_mesh = true;
                        context.asset.mesh_vertices.reserve(vertices);
                        for (std::size_t i = 0; i < vertices; ++i)
                        {
                            context.asset.mesh_vertices.push_back(
                                Vector3{Scalar(context.repaired.positions[i * 3 + 0]),
                                        Scalar(context.repaired.positions[i * 3 + 1]),
                                        Scalar(context.repaired.positions[i * 3 + 2])});
                        }
                        context.asset.mesh_indices = context.repaired.indices;

                        CookedTriangleMesh<Scalar> cooked = build_mesh_bvh<Scalar>(
                            context.asset.mesh_vertices.data(), context.asset.mesh_indices.data(),
                            std::uint32_t(triangles));
                        context.asset.mesh_nodes = std::move(cooked.nodes);
                        context.asset.mesh_order = std::move(cooked.order);
                        context.asset.mesh_adjacency = std::move(cooked.adjacency);
                        if (context.asset.mesh_nodes.empty())
                            return false;

                        context.report.collision_triangle_count = std::uint32_t(triangles);
                        context.report.hausdorff_error = 0.0f;
                        context.report.volume_error = 0.0f;
                        return true;
                    }
                };

                /**
                 * @brief Stage four: bake the distance field §7.5 and §9.6 query.
                 *
                 * One `MeshDistanceQuery` lookup per voxel, which is what makes the dial's
                 * upper resolutions affordable at all. It is a **full** brick and not yet the
                 * narrow band §8.4 item 3 asks for: at 128 voxels per axis that is eight
                 * megabytes an asset, and narrowing it is a format change rather than a
                 * baking change, so it is named as an open item instead of half-done.
                 */
                class BakeDistanceFieldStage final : public Stage
                {
                public:
                    const char* name() const noexcept override { return "BakeDistanceField"; }

                    bool run(CollisionCookContext& context) override
                    {
                        const std::int32_t resolution =
                            context.parameters.distance_field_resolution;
                        if (resolution <= 0)
                            return true;   // asked for no field, which is not a failure

                        context.asset.field = Geometry::bake_signed_distance_field(
                            context.repaired.view(), resolution);
                        if (context.asset.field.distances.empty())
                            return false;
                        context.report.distance_field_resolution =
                            std::uint32_t(context.asset.field.resolution);
                        return true;
                    }
                };

                /** @brief Stage five: write the blob. */
                class SerializeStage final : public Stage
                {
                public:
                    explicit SerializeStage(std::vector<std::byte>& out) : out_(out) {}

                    const char* name() const noexcept override { return "Serialize"; }

                    bool run(CollisionCookContext& context) override
                    {
                        if (!build_collision_blob(context.asset, out_))
                            return false;
                        context.report.asset_bytes = out_.size();
                        return true;
                    }

                private:
                    std::vector<std::byte>& out_;
                };
            } // namespace

            CookedAssetKey CollisionCooker::cache_key(const Geometry::TriangleMeshView& mesh,
                                                      const CookingParameters& parameters)
                const noexcept
            {
                CookedAssetKey key;
                key.source_hash = mesh_content_hash(mesh);
                key.parameters_hash = cooking_parameters_hash(parameters);
                key.cooker_version = version();
                key.kind = kind();
                return key;
            }

            CookingReport CollisionCooker::cook(const Geometry::TriangleMeshView& mesh,
                                                const CookingParameters& parameters,
                                                ICookedAssetStore* store,
                                                ICookingProgressSink* progress,
                                                std::vector<std::byte>& out)
            {
                out.clear();

                CollisionCookContext context;
                context.source = mesh;
                context.parameters = resolve_cooking_parameters(parameters);

                if (!mesh.has_triangles())
                {
                    context.report.status = CookingStatus::EmptyInput;
                    return context.report;
                }

                const CookedAssetKey key = cache_key(mesh, parameters);

                if (store != nullptr && store->load(key, out) && !out.empty())
                {
                    const CollisionAssetView view =
                        load_collision_blob(out.data(), out.size());
                    if (view.valid)
                    {
                        // What the report can honestly say about a cache hit is what the asset
                        // itself carries. The source topology is *not* filled in, because
                        // nothing looked at the source — see CookingReport::source.
                        context.report.served_from_cache = true;
                        context.report.asset_bytes = out.size();
                        context.report.convex_piece_count = view.piece_count;
                        context.report.collision_triangle_count = view.mesh_triangle_count;
                        context.report.distance_field_resolution = view.field_resolution;
                        context.report.mass = float(view.summary.mass);
                        context.report.center_of_mass[0] = float(view.summary.center_of_mass.x);
                        context.report.center_of_mass[1] = float(view.summary.center_of_mass.y);
                        context.report.center_of_mass[2] = float(view.summary.center_of_mass.z);
                        context.report.principal_inertia[0] =
                            float(view.summary.principal_inertia.x);
                        context.report.principal_inertia[1] =
                            float(view.summary.principal_inertia.y);
                        context.report.principal_inertia[2] =
                            float(view.summary.principal_inertia.z);
                        context.report.hausdorff_error = view.summary.hausdorff_error;
                        context.report.volume_error = view.summary.volume_error;
                        for (std::uint32_t i = 0; i < view.piece_count; ++i)
                        {
                            context.report.largest_piece_vertex_count =
                                std::max(context.report.largest_piece_vertex_count,
                                         view.pieces[i].vertex_count);
                        }
                        context.report.status = CookingStatus::Succeeded;
                        apply_cooking_thresholds(thresholds_, context.report);
                        return context.report;
                    }
                    // A blob that does not validate is a cache entry from a different build
                    // or a damaged file. Dropping it and cooking is the only safe response;
                    // trusting it would be trusting bytes this build cannot read.
                    out.clear();
                    store->evict(key);
                }

                std::vector<std::unique_ptr<Stage>> stages;
                stages.push_back(std::make_unique<RepairStage>());
                stages.push_back(std::make_unique<MassPropertiesStage>());
                if (context.parameters.static_geometry)
                    stages.push_back(std::make_unique<StaticHierarchyStage>());
                else
                    stages.push_back(std::make_unique<DecomposeStage>());
                stages.push_back(std::make_unique<BakeDistanceFieldStage>());
                stages.push_back(std::make_unique<SerializeStage>(out));

                for (std::size_t i = 0; i < stages.size(); ++i)
                {
                    if (progress != nullptr)
                    {
                        CookingProgress update;
                        update.stage = stages[i]->name();
                        update.completed_stages = std::uint32_t(i);
                        update.total_stages = std::uint32_t(stages.size());
                        progress->on_progress(update);
                    }
                    if (!stages[i]->run(context))
                    {
                        context.report.status = CookingStatus::StageFailed;
                        context.report.failed_stage = stages[i]->name();
                        out.clear();
                        return context.report;
                    }
                }

                context.report.status = CookingStatus::Succeeded;
                apply_cooking_thresholds(thresholds_, context.report);

                // Cached even when a threshold rejected it. The asset is what it is, and
                // re-cooking it on every inspection to show the artist the geometry that
                // failed would make a failing cook the slowest thing in the editor.
                if (store != nullptr && !out.empty())
                    store->store(key, out);
                return context.report;
            }
        } // namespace Cooking
    } // namespace Physics
} // namespace SushiEngine
