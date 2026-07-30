/**************************************************************************/
/* soft_body_cooker.cpp                                                   */
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

#include <SushiEngine/physics/cooking/soft_body_cooker.hpp>

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
                using Stage = ICookingStage<SoftBodyCookContext>;

                /** @brief The options the finest level wants, target count included. */
                TetrahedralizationOptions finest_options(const DerivedCookingParameters& parameters)
                {
                    TetrahedralizationOptions options;
                    options.voxel_resolution = parameters.voxel_resolution;
                    options.target_tetrahedron_count = parameters.target_tetrahedron_count;
                    options.conforming_passes = parameters.surface_conforming_passes;
                    options.density = parameters.density;
                    return options;
                }

                /**
                 * @brief The options a coarser level wants, halving from the *effective* base.
                 *
                 * @p base is the resolution the finest level actually used, not the one the dial
                 * authored. The distinction is not academic: the element-count target scales the
                 * finest level's resolution down, so halving the authored number instead
                 * produces a "coarser" level that is finer than the one above it — and the chain
                 * then refuses to grow, because a level that is not coarser is a second copy.
                 */
                TetrahedralizationOptions coarser_options(
                    const DerivedCookingParameters& parameters, std::int32_t base,
                    std::int32_t level)
                {
                    TetrahedralizationOptions options;
                    std::int32_t resolution = base;
                    for (std::int32_t i = 0; i < level; ++i)
                        resolution = std::max(resolution / 2, 2);
                    options.voxel_resolution = resolution;
                    // Zero, so the target does not drag every level back to the same size,
                    // which is the opposite of a level of detail.
                    options.target_tetrahedron_count = 0;
                    options.conforming_passes = parameters.surface_conforming_passes;
                    options.density = parameters.density;
                    return options;
                }

                /** @brief §8.3 stage 1. */
                class RepairStage final : public Stage
                {
                public:
                    const char* name() const noexcept override { return "Repair"; }

                    bool run(SoftBodyCookContext& context) override
                    {
                        context.report.source = Geometry::analyze_mesh_topology(context.source);
                        Geometry::MeshRepairOptions options;
                        options.weld_tolerance = context.parameters.weld_tolerance;
                        context.report.repair =
                            Geometry::repair_mesh(context.source, options, context.repaired);
                        if (context.repaired.triangle_count() == 0)
                            return false;
                        return context.surface.build(context.repaired.view());
                    }
                };

                /** @brief §8.3 stages 2, 3, 4, 5 and 7 — everything that shares the voxel grid. */
                class TetrahedralizeStage final : public Stage
                {
                public:
                    const char* name() const noexcept override { return "Tetrahedralize"; }

                    bool run(SoftBodyCookContext& context) override
                    {
                        TetrahedralMesh finest;
                        const TetrahedralizationReport report = build_tetrahedral_mesh(
                            context.repaired.view(), context.surface,
                            finest_options(context.parameters), finest);
                        if (finest.tetrahedron_count() == 0)
                            return false;

                        context.base_resolution = report.resolution;
                        context.report.tetrahedron_count = report.tetrahedron_count;
                        context.report.inverted_element_count = report.inverted_element_count;
                        context.report.worst_element_quality = report.worst_element_quality;
                        context.levels.push_back(std::move(finest));

                        // The body's mass comes from the *simulation* surface rather than from
                        // the source mesh, and the difference is the point: what the body
                        // weighs must be what it will collide and deform as, or the rigid
                        // fallback §9.7 drops to would weigh something the soft body did not.
                        const Geometry::TriangleMesh surface = context.levels[0].surface_mesh();
                        const MeshMassProperties properties = mesh_mass_properties(
                            surface.view(), Scalar(context.parameters.density));
                        SoftBodySummary& summary = context.asset.summary;
                        if (properties.valid)
                        {
                            summary.mass = properties.properties.mass;
                            summary.center_of_mass = properties.properties.center_of_mass;
                            summary.principal_inertia = properties.properties.inertia;
                            summary.principal_rotation = properties.principal_rotation;
                        }
                        summary.volume = report.total_volume;
                        summary.worst_element_quality = report.worst_element_quality;
                        summary.suggested_substep_count =
                            std::uint32_t(context.parameters.suggested_substep_count);

                        context.report.mass = float(summary.mass);
                        context.report.center_of_mass[0] = float(summary.center_of_mass.x);
                        context.report.center_of_mass[1] = float(summary.center_of_mass.y);
                        context.report.center_of_mass[2] = float(summary.center_of_mass.z);
                        context.report.principal_inertia[0] = float(summary.principal_inertia.x);
                        context.report.principal_inertia[1] = float(summary.principal_inertia.y);
                        context.report.principal_inertia[2] = float(summary.principal_inertia.z);

                        // §7.6's number for a soft body: how far the surface that will actually
                        // be simulated departs from the one the artist modelled. It is the
                        // voxel resolution's error, made visible.
                        context.report.hausdorff_error = Geometry::one_sided_hausdorff_distance(
                            surface.view(), context.surface,
                            std::uint32_t(context.parameters.accuracy_lattice_order));
                        summary.hausdorff_error = context.report.hausdorff_error;

                        const Scalar source_volume =
                            Scalar(context.report.repair.after.signed_volume);
                        if (source_volume > Scalar(0))
                        {
                            context.report.volume_error =
                                float(report.total_volume / source_volume - Scalar(1));
                        }
                        return true;
                    }
                };

                /** @brief §8.3 stage 6 — the table that *is* §0.4's guarantee. */
                class EmbedStage final : public Stage
                {
                public:
                    const char* name() const noexcept override { return "Embed"; }

                    bool run(SoftBodyCookContext& context) override
                    {
                        // The **source** mesh's vertices, in the source's own order, and this
                        // is the one thing in the stage that matters. The repair welds and
                        // compacts, so a binding table built against the repaired mesh is
                        // indexed in an order the renderer has never heard of — every render
                        // vertex would then be driven by some other vertex's element, which
                        // reconstructs a mirrored or shuffled mesh rather than a torn one, and
                        // is therefore invisible until someone looks at the model. Read through
                        // the view so an arbitrary vertex stride is honoured too.
                        std::vector<Vector3> points;
                        points.reserve(context.source.vertex_count);
                        for (std::size_t i = 0; i < context.source.vertex_count; ++i)
                        {
                            float position[3];
                            context.source.read_position(i, position);
                            points.push_back(Vector3{Scalar(position[0]), Scalar(position[1]),
                                                     Scalar(position[2])});
                        }

                        std::vector<TetrahedronBinding> bindings;
                        const std::uint32_t extrapolated =
                            embed_points(context.levels[0], points.data(), points.size(), bindings);

                        // Extrapolated is *not* unembedded. §8.6 invariant 1 counts vertices
                        // bound to nothing, and the extrapolated fallback exists precisely so
                        // that count can be zero: a thin feature that fell through the lattice
                        // is still attached and still moves correctly, through a negative
                        // barycentric coordinate.
                        context.report.unembedded_vertex_count = 0;
                        context.report.extrapolated_binding_count = extrapolated;
                        context.asset.summary.extrapolated_binding_count = extrapolated;

                        context.asset.bindings.reserve(bindings.size());
                        for (const TetrahedronBinding& binding : bindings)
                        {
                            SoftBodyBinding record{};
                            record.tetrahedron = binding.tetrahedron;
                            for (int corner = 0; corner < 4; ++corner)
                                record.weights[corner] = binding.weights[corner];
                            context.asset.bindings.push_back(record);
                        }
                        if (context.asset.bindings.size() != context.source.vertex_count)
                            return false;
                        return true;
                    }
                };

                /** @brief §8.3 stage 7's hierarchy and stage 8's rest-shape distance field. */
                class BakeRestShapeStage final : public Stage
                {
                public:
                    const char* name() const noexcept override { return "BakeRestShape"; }

                    bool run(SoftBodyCookContext& context) override
                    {
                        const Geometry::TriangleMesh surface = context.levels[0].surface_mesh();
                        if (surface.triangle_count() == 0)
                            return false;

                        // The hierarchy is over the finest level's boundary only: the coarser
                        // levels are simulation lattices and are never collided against, so
                        // building them a tree would be building something with no caller.
                        std::vector<Vector3> vertices;
                        vertices.reserve(surface.vertex_count());
                        for (std::size_t i = 0; i < surface.vertex_count(); ++i)
                        {
                            vertices.push_back(Vector3{Scalar(surface.positions[i * 3 + 0]),
                                                       Scalar(surface.positions[i * 3 + 1]),
                                                       Scalar(surface.positions[i * 3 + 2])});
                        }
                        CookedTriangleMesh<Scalar> cooked = build_mesh_bvh<Scalar>(
                            vertices.data(), surface.indices.data(),
                            std::uint32_t(surface.triangle_count()));
                        context.asset.surface_nodes = std::move(cooked.nodes);
                        context.asset.surface_order = std::move(cooked.order);
                        context.asset.surface_adjacency = std::move(cooked.adjacency);

                        const std::int32_t resolution =
                            context.parameters.distance_field_resolution;
                        if (resolution > 0)
                        {
                            context.asset.field =
                                Geometry::bake_signed_distance_field(surface.view(), resolution);
                            if (context.asset.field.distances.empty())
                                return false;
                            context.report.distance_field_resolution =
                                std::uint32_t(context.asset.field.resolution);
                        }
                        return true;
                    }
                };

                /** @brief §8.3 stage 9 — coarser lattices, and the mapping down the chain. */
                class LevelsOfDetailStage final : public Stage
                {
                public:
                    const char* name() const noexcept override { return "LevelsOfDetail"; }

                    bool run(SoftBodyCookContext& context) override
                    {
                        const std::int32_t wanted =
                            std::max(context.parameters.simulation_level_count, 1);
                        for (std::int32_t level = 1; level < wanted; ++level)
                        {
                            TetrahedralMesh coarser;
                            build_tetrahedral_mesh(
                                context.repaired.view(), context.surface,
                                coarser_options(context.parameters, context.base_resolution, level),
                                coarser);
                            // A level that came out no smaller than the one above it is not a
                            // level of detail, it is a second copy — which costs memory and
                            // buys nothing, so the chain stops here rather than padding itself
                            // out to the count the dial asked for.
                            if (coarser.tetrahedron_count() == 0 ||
                                coarser.tetrahedron_count() >=
                                    context.levels.back().tetrahedron_count())
                                break;
                            context.levels.push_back(std::move(coarser));
                        }
                        context.report.level_of_detail_count =
                            std::uint32_t(context.levels.size());
                        return true;
                    }
                };

                /** @brief §8.3 stage 10 — flatten the levels into the blob's shared arrays. */
                class SerializeStage final : public Stage
                {
                public:
                    explicit SerializeStage(std::vector<std::byte>& out) : out_(out) {}

                    const char* name() const noexcept override { return "Serialize"; }

                    bool run(SoftBodyCookContext& context) override
                    {
                        SoftBodyAsset& asset = context.asset;
                        asset.parameters = context.authored;

                        // Levels are concatenated into one set of arrays, so the section count
                        // does not depend on the fidelity dial. Each level records where its
                        // own run starts, and every index it holds is rebased into the shared
                        // arrays as it is copied — which is the one thing in this stage that
                        // must not be got wrong, since an unrebased index reads another level's
                        // vertices and produces a body that deforms into its own coarse copy.
                        for (std::size_t level = 0; level < context.levels.size(); ++level)
                        {
                            const TetrahedralMesh& mesh = context.levels[level];
                            SoftBodyLevelRecord record{};
                            record.first_vertex = std::uint32_t(asset.vertices.size());
                            record.vertex_count = std::uint32_t(mesh.vertex_count());
                            record.first_tetrahedron =
                                std::uint32_t(asset.tetrahedra.size() / 4);
                            record.tetrahedron_count = std::uint32_t(mesh.tetrahedron_count());
                            record.first_surface_index =
                                std::uint32_t(asset.surface_indices.size());
                            record.surface_index_count =
                                std::uint32_t(mesh.surface_indices.size());
                            record.first_mapping = std::uint32_t(asset.mappings.size());
                            record.mapping_count = 0;
                            record.cell_size = mesh.cell_size;
                            record.grid_origin = mesh.grid_origin;
                            for (int axis = 0; axis < 3; ++axis)
                                record.grid[axis] = mesh.grid[axis];

                            const std::uint32_t vertex_base = record.first_vertex;
                            asset.vertices.insert(asset.vertices.end(), mesh.vertices.begin(),
                                                  mesh.vertices.end());
                            asset.vertex_mass.insert(asset.vertex_mass.end(),
                                                     mesh.vertex_mass.begin(),
                                                     mesh.vertex_mass.end());
                            for (const std::uint32_t index : mesh.tetrahedra)
                                asset.tetrahedra.push_back(index + vertex_base);
                            asset.rest_inverse.insert(asset.rest_inverse.end(),
                                                      mesh.rest_inverse.begin(),
                                                      mesh.rest_inverse.end());
                            asset.rest_volume.insert(asset.rest_volume.end(),
                                                     mesh.rest_volume.begin(),
                                                     mesh.rest_volume.end());
                            for (const std::uint32_t index : mesh.surface_indices)
                                asset.surface_indices.push_back(index + vertex_base);

                            asset.levels.push_back(record);
                        }

                        // The render bindings name elements of level zero, whose run starts at
                        // zero, so they need no rebasing. Stated rather than relied on: if
                        // level zero ever stopped being first, this would silently be wrong.
                        if (!asset.levels.empty() && asset.levels[0].first_tetrahedron != 0)
                            return false;

                        // Each coarse level reconstructs the level above it, so the mapping is
                        // the finer level's vertices bound into this level's elements.
                        for (std::size_t level = 1; level < context.levels.size(); ++level)
                        {
                            const TetrahedralMesh& finer = context.levels[level - 1];
                            std::vector<TetrahedronBinding> bindings;
                            embed_points(context.levels[level], finer.vertices.data(),
                                         finer.vertices.size(), bindings);

                            SoftBodyLevelRecord& record = asset.levels[level];
                            record.first_mapping = std::uint32_t(asset.mappings.size());
                            record.mapping_count = std::uint32_t(bindings.size());
                            for (const TetrahedronBinding& binding : bindings)
                            {
                                SoftBodyBinding entry{};
                                entry.tetrahedron = binding.tetrahedron + record.first_tetrahedron;
                                for (int corner = 0; corner < 4; ++corner)
                                    entry.weights[corner] = binding.weights[corner];
                                asset.mappings.push_back(entry);
                            }
                        }

                        if (!build_soft_body_blob(asset, out_))
                            return false;
                        context.report.asset_bytes = out_.size();
                        return true;
                    }

                private:
                    std::vector<std::byte>& out_;
                };
            } // namespace

            CookedAssetKey SoftBodyCooker::cache_key(const Geometry::TriangleMeshView& mesh,
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

            CookingReport SoftBodyCooker::cook(const Geometry::TriangleMeshView& mesh,
                                               const CookingParameters& parameters,
                                               ICookedAssetStore* store,
                                               ICookingProgressSink* progress,
                                               std::vector<std::byte>& out)
            {
                out.clear();

                SoftBodyCookContext context;
                context.source = mesh;
                context.authored = parameters;
                context.parameters = resolve_cooking_parameters(parameters);

                if (!mesh.has_triangles())
                {
                    context.report.status = CookingStatus::EmptyInput;
                    return context.report;
                }

                const CookedAssetKey key = cache_key(mesh, parameters);

                if (store != nullptr && store->load(key, out) && !out.empty())
                {
                    const SoftBodyAssetView view = load_soft_body_blob(out.data(), out.size());
                    if (view.valid)
                    {
                        // As with the collision cook: what a cache hit can honestly report is
                        // what the asset carries, and the source topology stays unmeasured
                        // because nothing looked at the source.
                        context.report.served_from_cache = true;
                        context.report.asset_bytes = out.size();
                        context.report.level_of_detail_count = view.level_count;
                        context.report.tetrahedron_count =
                            view.level_count > 0 ? view.levels[0].tetrahedron_count : 0;
                        context.report.distance_field_resolution = view.field_resolution;
                        context.report.worst_element_quality = view.summary.worst_element_quality;
                        context.report.hausdorff_error = view.summary.hausdorff_error;
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
                        context.report.status = CookingStatus::Succeeded;
                        apply_cooking_thresholds(thresholds_, context.report);
                        return context.report;
                    }
                    out.clear();
                    store->evict(key);
                }

                std::vector<std::unique_ptr<Stage>> stages;
                stages.push_back(std::make_unique<RepairStage>());
                stages.push_back(std::make_unique<TetrahedralizeStage>());
                stages.push_back(std::make_unique<EmbedStage>());
                stages.push_back(std::make_unique<BakeRestShapeStage>());
                stages.push_back(std::make_unique<LevelsOfDetailStage>());
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
                if (store != nullptr && !out.empty())
                    store->store(key, out);
                return context.report;
            }
        } // namespace Cooking
    } // namespace Physics
} // namespace SushiEngine
