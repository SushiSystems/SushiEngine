/**************************************************************************/
/* node_beam_cooker.cpp                                                   */
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

#include <SushiEngine/physics/cooking/node_beam_cooker.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <unordered_map>

#include <SushiEngine/geometry/mesh_utilities.hpp>
#include <SushiEngine/physics/geometry/mesh_mass_properties.hpp>
#include <SushiEngine/physics/soft/beam_properties.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        namespace Cooking
        {
            namespace
            {
                using Stage = ICookingStage<NodeBeamCookContext>;

                /** @brief The value a material's yield or fracture stress carries when it never fails. */
                constexpr Scalar NEVER_FAILS = Scalar(1e29);

                /**
                 * @brief A uniform grid over the node cloud, for the skinning search.
                 *
                 * Built rather than reusing the lattice's own grid, and the reason is the
                 * conforming pass: a boundary vertex that was snapped onto the surface no
                 * longer sits in the cell it was generated from, so an index derived from the
                 * lattice's grid would miss exactly the nodes a render vertex is nearest to.
                 *
                 * Ordinary chained hashing rather than anything clever. The search is over a
                 * few thousand nodes per vehicle and runs once per cook; the thing it must be
                 * is deterministic, which it is because every bucket is filled in node order
                 * and read back in it.
                 */
                class NodeGrid
                {
                public:
                    /** @brief Indexes @p nodes at a cell size of @p spacing. */
                    void build(const std::vector<NodeBeamNodeRecord>& nodes, Scalar spacing)
                    {
                        buckets_.clear();
                        spacing_ = spacing > Scalar(0) ? spacing : Scalar(1);
                        for (std::size_t i = 0; i < nodes.size(); ++i)
                            buckets_[key_of(nodes[i].position)].push_back(std::uint32_t(i));
                    }

                    /**
                     * @brief Appends every node within @p rings cells of @p point to @p out.
                     *
                     * @param point The query position.
                     * @param rings How many cells to reach in each direction.
                     * @param out   Receives the candidates, in node order per cell.
                     */
                    void gather(const Vector3& point, int rings,
                                std::vector<std::uint32_t>& out) const
                    {
                        const std::int64_t cx = cell_of(point.x);
                        const std::int64_t cy = cell_of(point.y);
                        const std::int64_t cz = cell_of(point.z);
                        for (std::int64_t x = cx - rings; x <= cx + rings; ++x)
                        {
                            for (std::int64_t y = cy - rings; y <= cy + rings; ++y)
                            {
                                for (std::int64_t z = cz - rings; z <= cz + rings; ++z)
                                {
                                    const auto found = buckets_.find(pack(x, y, z));
                                    if (found == buckets_.end())
                                        continue;
                                    out.insert(out.end(), found->second.begin(),
                                               found->second.end());
                                }
                            }
                        }
                    }

                private:
                    static std::uint64_t pack(std::int64_t x, std::int64_t y,
                                              std::int64_t z) noexcept
                    {
                        std::uint64_t hash = 1469598103934665603ull;
                        hash = hash_bytes(hash, x);
                        hash = hash_bytes(hash, y);
                        hash = hash_bytes(hash, z);
                        return hash;
                    }

                    std::int64_t cell_of(Scalar value) const noexcept
                    {
                        return std::int64_t(std::floor(double(value / spacing_)));
                    }

                    std::uint64_t key_of(const Vector3& point) const noexcept
                    {
                        return pack(cell_of(point.x), cell_of(point.y), cell_of(point.z));
                    }

                    std::unordered_map<std::uint64_t, std::vector<std::uint32_t>> buckets_;
                    Scalar spacing_ = 1;
                };

                /** @brief A candidate node for one render vertex, ordered for determinism. */
                struct SkinCandidate
                {
                    Scalar distance;
                    std::uint32_t node;

                    /** @brief Nearest first, and by node index where two are equidistant. */
                    bool operator<(const SkinCandidate& other) const noexcept
                    {
                        if (distance != other.distance)
                            return distance < other.distance;
                        return node < other.node;
                    }
                };

                /** @brief §8.3 stage 1, unchanged: the same repair every cook starts from. */
                class RepairStage final : public Stage
                {
                public:
                    const char* name() const noexcept override { return "Repair"; }

                    bool run(NodeBeamCookContext& context) override
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

                /**
                 * @brief §11.3's node placement, on the tetrahedralizer's lattice.
                 *
                 * The lattice is asked for rather than reimplemented: it already voxelizes,
                 * flood-fills the interior, conforms the boundary and returns per-vertex
                 * masses and an outward-wound surface, which is the whole of what a node cloud
                 * is. A second voxelizer would decide "inside" differently on a dirty mesh,
                 * and a dirty mesh is the only kind where the answer is interesting.
                 */
                class PlaceNodesStage final : public Stage
                {
                public:
                    const char* name() const noexcept override { return "PlaceNodes"; }

                    bool run(NodeBeamCookContext& context) override
                    {
                        TetrahedralizationOptions options;
                        options.voxel_resolution = context.parameters.voxel_resolution;
                        options.target_tetrahedron_count =
                            context.parameters.target_tetrahedron_count;
                        options.conforming_passes = context.parameters.surface_conforming_passes;
                        options.density = context.parameters.density;

                        const TetrahedralizationReport report = build_tetrahedral_mesh(
                            context.repaired.view(), context.surface, options, context.lattice);
                        if (context.lattice.vertex_count() == 0 ||
                            context.lattice.tetrahedron_count() == 0)
                            return false;

                        context.report.inverted_element_count = report.inverted_element_count;

                        // Which nodes are on the collision surface, so a contact pass can skip
                        // the interior without walking the index list to find out.
                        std::vector<bool> on_surface(context.lattice.vertex_count(), false);
                        for (const std::uint32_t index : context.lattice.surface_indices)
                            on_surface[index] = true;

                        // A node's wind cross-section is the surface area it carries, split
                        // evenly between a triangle's three corners. The coefficient is *not*
                        // folded in here: a node has no orientation, so it cannot present a
                        // different area to a different wind direction, and the vehicle-level
                        // drag coefficient is a property of the whole body that §11.6 applies
                        // at instancing. Interior nodes carry none, which is correct — they
                        // are inside and the wind never reaches them.
                        std::vector<Scalar> tributary_area(context.lattice.vertex_count(),
                                                           Scalar(0));
                        for (std::size_t i = 0; i + 2 < context.lattice.surface_indices.size();
                             i += 3)
                        {
                            const std::uint32_t ia = context.lattice.surface_indices[i];
                            const std::uint32_t ib = context.lattice.surface_indices[i + 1];
                            const std::uint32_t ic = context.lattice.surface_indices[i + 2];
                            const Vector3 ab =
                                context.lattice.vertices[ib] - context.lattice.vertices[ia];
                            const Vector3 ac =
                                context.lattice.vertices[ic] - context.lattice.vertices[ia];
                            const Scalar area = length(cross(ab, ac)) * Scalar(0.5);
                            const Scalar share = area / Scalar(3);
                            tributary_area[ia] += share;
                            tributary_area[ib] += share;
                            tributary_area[ic] += share;
                        }

                        const Scalar shell_fraction =
                            Scalar(1) -
                            Scalar(Detail::clamp_unit(context.settings.core_mass_fraction));
                        const Scalar radius = context.lattice.cell_size * Scalar(0.5);

                        context.asset.nodes.reserve(context.lattice.vertex_count());
                        for (std::size_t i = 0; i < context.lattice.vertex_count(); ++i)
                        {
                            NodeBeamNodeRecord node{};
                            node.position = context.lattice.vertices[i];
                            node.mass = context.lattice.vertex_mass[i] * shell_fraction;
                            node.radius = radius;
                            node.drag_area = tributary_area[i];
                            // One part. Splitting a vehicle into doors and panels is an
                            // authoring decision the cooker has no way to infer from a mesh,
                            // and inventing a split from connectivity would produce parts no
                            // artist asked for and cannot rename.
                            node.part = 0;
                            node.flags = on_surface[i] ? NodeBeamNodeFlags::surface
                                                       : NodeBeamNodeFlags::none;
                            context.asset.nodes.push_back(node);
                        }

                        context.asset.surface_indices = context.lattice.surface_indices;
                        context.report.node_count =
                            std::uint32_t(context.asset.nodes.size());
                        context.report.collision_triangle_count =
                            std::uint32_t(context.lattice.surface_triangle_count());
                        return true;
                    }
                };

                /**
                 * @brief §11.3's beams: the lattice's edges, deduplicated and classified.
                 *
                 * The diagonals are already in the edge set — a lattice tetrahedralization
                 * produces them by construction — so the "diagonal rule" is a classification
                 * rather than a second construction pass. Adding more would double-brace a
                 * network that is already braced, which reads as a structure that will not
                 * deform at all and is diagnosed as a compliance bug.
                 */
                class ConnectBeamsStage final : public Stage
                {
                public:
                    const char* name() const noexcept override { return "ConnectBeams"; }

                    bool run(NodeBeamCookContext& context) override
                    {
                        static const int EDGES[6][2] = {{0, 1}, {0, 2}, {0, 3},
                                                        {1, 2}, {1, 3}, {2, 3}};

                        std::vector<std::uint64_t> edges;
                        edges.reserve(context.lattice.tetrahedron_count() * 6);
                        for (std::size_t element = 0;
                             element < context.lattice.tetrahedron_count(); ++element)
                        {
                            const std::uint32_t* corners =
                                context.lattice.tetrahedra.data() + element * 4;
                            for (const auto& edge : EDGES)
                            {
                                const std::uint32_t a = corners[edge[0]];
                                const std::uint32_t b = corners[edge[1]];
                                if (a == b)
                                    continue;
                                const std::uint32_t low = a < b ? a : b;
                                const std::uint32_t high = a < b ? b : a;
                                edges.push_back((std::uint64_t(low) << 32) | std::uint64_t(high));
                            }
                        }
                        std::sort(edges.begin(), edges.end());
                        edges.erase(std::unique(edges.begin(), edges.end()), edges.end());

                        // The volume the tributary areas are divided out of, so that
                        // `sum(area * length)` reproduces it. Taken from the lattice's own
                        // elements rather than from the source mesh: the beams hold up the
                        // lattice, and sizing them against a volume the lattice does not have
                        // would make a body whose cross-sections do not add up to itself.
                        Scalar volume = 0;
                        for (const Scalar element_volume : context.lattice.rest_volume)
                            volume += element_volume;
                        context.asset.summary.structure_volume = volume;

                        const Scalar structural_limit =
                            context.lattice.cell_size *
                            Scalar(context.settings.structural_length_ratio);

                        Scalar shortest = 0;
                        Scalar longest = 0;
                        std::uint32_t bracing = 0;

                        context.asset.beams.reserve(edges.size());
                        for (const std::uint64_t packed : edges)
                        {
                            const std::uint32_t a = std::uint32_t(packed >> 32);
                            const std::uint32_t b = std::uint32_t(packed & 0xffffffffull);
                            const Scalar rest = length(context.lattice.vertices[b] -
                                                       context.lattice.vertices[a]);
                            if (!(rest > Scalar(0)))
                                continue;

                            const Scalar area =
                                beam_tributary_area(volume, edges.size(), rest);
                            const BeamProperties<Scalar> properties =
                                beam_properties_from_material(context.settings.material, rest,
                                                              area);

                            NodeBeamBeamRecord beam{};
                            beam.a = a;
                            beam.b = b;
                            beam.part = 0;
                            beam.kind = rest <= structural_limit
                                            ? NodeBeamBeamKind::structural
                                            : NodeBeamBeamKind::bracing;
                            beam.rest_length = rest;
                            beam.compliance = properties.compliance;
                            beam.damping = properties.damping;
                            beam.deform_force = properties.deform_force;
                            beam.break_force = properties.break_force;
                            beam.plastic_creep = context.settings.material.plastic_creep;
                            beam.maximum_plastic_strain =
                                context.settings.material.maximum_plastic_strain;
                            if (beam.kind == NodeBeamBeamKind::bracing)
                                ++bracing;

                            if (context.asset.beams.empty() || rest < shortest)
                                shortest = rest;
                            if (rest > longest)
                                longest = rest;
                            context.asset.beams.push_back(beam);
                        }
                        if (context.asset.beams.empty())
                            return false;

                        context.asset.summary.shortest_beam_length = shortest;
                        context.asset.summary.longest_beam_length = longest;
                        context.report.beam_count =
                            std::uint32_t(context.asset.beams.size());
                        context.report.bracing_beam_count = bracing;
                        return true;
                    }
                };

                /**
                 * @brief §11.2's node-beam binding: the render mesh skinned by distance.
                 *
                 * Against the **source** mesh's vertices in the source's own order, for the
                 * reason the soft-body embed stage gives at length: the repair welds and
                 * compacts, so a table built against the repaired mesh is indexed in an order
                 * the renderer has never heard of, and every vertex would then be driven by
                 * some other vertex's nodes — which reconstructs a shuffled mesh rather than a
                 * torn one, and is invisible until someone looks at the model.
                 */
                class SkinStage final : public Stage
                {
                public:
                    const char* name() const noexcept override { return "Skin"; }

                    bool run(NodeBeamCookContext& context) override
                    {
                        NodeGrid grid;
                        grid.build(context.asset.nodes, context.lattice.cell_size);

                        const Scalar reach = context.lattice.cell_size *
                                             Scalar(context.settings.skin_search_ratio > 0.0f
                                                        ? context.settings.skin_search_ratio
                                                        : 1.0f);
                        const int rings =
                            int(std::ceil(double(reach / context.lattice.cell_size)));

                        std::uint32_t unskinned = 0;
                        std::vector<std::uint32_t> candidates;
                        std::vector<SkinCandidate> ranked;

                        context.asset.skin.reserve(context.source.vertex_count);
                        for (std::size_t i = 0; i < context.source.vertex_count; ++i)
                        {
                            float position[3];
                            context.source.read_position(i, position);
                            const Vector3 point{Scalar(position[0]), Scalar(position[1]),
                                                Scalar(position[2])};

                            candidates.clear();
                            ranked.clear();
                            grid.gather(point, rings, candidates);
                            for (const std::uint32_t node : candidates)
                            {
                                const Scalar distance =
                                    length(context.asset.nodes[node].position - point);
                                if (distance <= reach)
                                    ranked.push_back(SkinCandidate{distance, node});
                            }
                            std::sort(ranked.begin(), ranked.end());

                            NodeBeamSkinRecord record{};
                            if (ranked.empty())
                            {
                                // Reported rather than tethered to whatever was nearest. A
                                // weight computed against a node on the other side of the car
                                // is not a binding, and a vertex that reads as bound is one
                                // nobody will look at again.
                                ++unskinned;
                                context.asset.skin.push_back(record);
                                continue;
                            }

                            const std::size_t used =
                                std::min<std::size_t>(ranked.size(),
                                                      NODE_BEAM_SKIN_INFLUENCES);
                            Scalar weights[NODE_BEAM_SKIN_INFLUENCES] = {0, 0, 0, 0};
                            Scalar total = 0;
                            for (std::size_t influence = 0; influence < used; ++influence)
                            {
                                // Inverse distance, floored so a vertex sitting exactly on a
                                // node produces a weight of one rather than an infinity.
                                const Scalar distance =
                                    ranked[influence].distance > Scalar(1e-9)
                                        ? ranked[influence].distance
                                        : Scalar(1e-9);
                                weights[influence] = Scalar(1) / distance;
                                total += weights[influence];
                            }
                            for (std::size_t influence = 0;
                                 influence < NODE_BEAM_SKIN_INFLUENCES; ++influence)
                            {
                                // Unused slots repeat the nearest node, which is in range by
                                // construction: the format requires every index to be valid
                                // whatever its weight, so a reader never tests a weight before
                                // trusting an index.
                                record.nodes[influence] =
                                    influence < used ? ranked[influence].node : ranked[0].node;
                                record.weights[influence] =
                                    influence < used
                                        ? float(weights[influence] / total)
                                        : 0.0f;
                            }
                            // The vertex's displacement from the centroid its own nodes
                            // define, in the frame they define. Without it the reconstruction
                            // is the centroid, and the centroid of the four nodes nearest a
                            // corner sits inside that corner — a mesh visibly shrunk before
                            // anything has moved.
                            build_node_beam_skin_offset(context.asset.nodes.data(), record, point,
                                                        record.offset);
                            context.asset.skin.push_back(record);
                        }

                        if (context.asset.skin.size() != context.source.vertex_count)
                            return false;
                        context.report.unembedded_vertex_count = unskinned;
                        context.asset.summary.unskinned_vertex_count = unskinned;
                        return true;
                    }
                };

                /**
                 * @brief §11.2's rigid core, and the shell attachments that hold the cloud on.
                 *
                 * The core's inertia is the *whole body's* inertia scaled to the core's share
                 * of the mass, which is exact for a homogeneous body and is what a chassis
                 * carrying most of the mass actually spins like. Deriving it from the node
                 * cloud instead would give the core the inertia of a shell — hollow, and far
                 * too willing to rotate about its long axis.
                 */
                class BuildCoreStage final : public Stage
                {
                public:
                    const char* name() const noexcept override { return "BuildCore"; }

                    bool run(NodeBeamCookContext& context) override
                    {
                        Scalar node_mass = 0;
                        Vector3 weighted{0, 0, 0};
                        for (const NodeBeamNodeRecord& node : context.asset.nodes)
                        {
                            node_mass += node.mass;
                            weighted = weighted + node.position * node.mass;
                        }

                        // At unit density, so the ratio below is the only place the cooked
                        // mass enters and a density of zero cannot divide anything.
                        const MeshMassProperties unit =
                            mesh_mass_properties(context.repaired.view(), Scalar(1));

                        Scalar lattice_mass = 0;
                        for (const Scalar mass : context.lattice.vertex_mass)
                            lattice_mass += mass;

                        const Scalar core_mass = lattice_mass - node_mass;
                        NodeBeamCoreRecord& core = context.asset.core;
                        if (core_mass > Scalar(0) && unit.valid && unit.properties.mass > Scalar(0))
                        {
                            core.mass = core_mass;
                            core.center_of_mass = unit.properties.center_of_mass;
                            core.principal_rotation = unit.principal_rotation;
                            core.principal_inertia =
                                unit.properties.inertia * (core_mass / unit.properties.mass);
                        }

                        const Scalar total_mass = node_mass + core.mass;
                        context.asset.summary.node_mass = node_mass;
                        context.asset.summary.total_mass = total_mass;
                        if (total_mass > Scalar(0))
                        {
                            context.asset.summary.center_of_mass =
                                (weighted + core.center_of_mass * core.mass) *
                                (Scalar(1) / total_mass);
                        }

                        if (!node_beam_has_core(core) || !context.settings.attach_shell_to_core)
                            return true;

                        // The lattice's interior is chassis and its boundary is shell. That is
                        // the only split a cooker can make from a mesh alone — a real
                        // vehicle's mounting points are authored — and it is the split §11.2
                        // describes: the rigid core carries the bulk and the deformable skin
                        // hangs off it. An asset whose lattice is one cell thick has no
                        // interior and therefore no attachments, which is a pure node-beam
                        // shell and is reported as such by the attachment count rather than
                        // by a failure.
                        const Scalar area =
                            context.lattice.cell_size * context.lattice.cell_size;
                        const Scalar break_force =
                            context.settings.material.fracture_stress < NEVER_FAILS
                                ? context.settings.material.fracture_stress * area
                                : Scalar(0);

                        for (std::size_t i = 0; i < context.asset.nodes.size(); ++i)
                        {
                            const NodeBeamNodeRecord& node = context.asset.nodes[i];
                            if ((node.flags & NodeBeamNodeFlags::surface) != 0)
                                continue;
                            NodeBeamAttachmentRecord attachment{};
                            attachment.node = std::uint32_t(i);
                            attachment.part = node.part;
                            // Relative to the core's centre of mass, which is where the core
                            // body's frame will be placed at instancing.
                            attachment.core_anchor = node.position - core.center_of_mass;
                            attachment.compliance = 0;
                            attachment.break_force = break_force;
                            context.asset.attachments.push_back(attachment);
                        }
                        return true;
                    }
                };

                /** @brief The accuracy readout and the blob. */
                class SerializeStage final : public Stage
                {
                public:
                    explicit SerializeStage(std::vector<std::byte>& out) : out_(out) {}

                    const char* name() const noexcept override { return "Serialize"; }

                    bool run(NodeBeamCookContext& context) override
                    {
                        // How far the collision surface departs from the source, as a
                        // **sampled lower bound**: the maximum is over the surface's own
                        // vertices, so a bulge between them is under-reported. Stated rather
                        // than presented as exact, which is §7.6's rule for every accuracy
                        // number this pipeline produces.
                        float worst = 0.0f;
                        for (std::size_t i = 0; i < context.lattice.vertex_count(); ++i)
                        {
                            if ((context.asset.nodes[i].flags & NodeBeamNodeFlags::surface) == 0)
                                continue;
                            const Vector3& position = context.lattice.vertices[i];
                            const float point[3] = {float(position.x), float(position.y),
                                                    float(position.z)};
                            const float distance =
                                std::fabs(context.surface.signed_distance(point));
                            if (distance > worst)
                                worst = distance;
                        }
                        context.asset.summary.hausdorff_error = worst;
                        context.report.hausdorff_error = worst;

                        context.asset.summary.suggested_substep_count =
                            std::uint32_t(context.parameters.suggested_substep_count > 0
                                              ? context.parameters.suggested_substep_count
                                              : 1);
                        context.asset.summary.part_count = 1;
                        context.asset.parameters = context.authored;

                        context.report.mass = float(context.asset.summary.total_mass);
                        context.report.center_of_mass[0] =
                            float(context.asset.summary.center_of_mass.x);
                        context.report.center_of_mass[1] =
                            float(context.asset.summary.center_of_mass.y);
                        context.report.center_of_mass[2] =
                            float(context.asset.summary.center_of_mass.z);
                        context.report.principal_inertia[0] =
                            float(context.asset.core.principal_inertia.x);
                        context.report.principal_inertia[1] =
                            float(context.asset.core.principal_inertia.y);
                        context.report.principal_inertia[2] =
                            float(context.asset.core.principal_inertia.z);

                        if (!build_node_beam_blob(context.asset, out_))
                            return false;
                        context.report.asset_bytes = out_.size();
                        return true;
                    }

                private:
                    std::vector<std::byte>& out_;
                };
            } // namespace

            CookedAssetKey NodeBeamCooker::cache_key(const Geometry::TriangleMeshView& mesh,
                                                     const CookingParameters& parameters)
                const noexcept
            {
                CookedAssetKey key;
                key.source_hash = mesh_content_hash(mesh);
                // The settings are folded into the parameters half of the key, because they
                // are parameters — they are simply ones only this cooker has. Without this,
                // the same mesh cooked as aluminium and as steel resolves to one key and the
                // second cook is served the first one's asset, which is a cache producing a
                // wrong answer rather than a slow one.
                std::uint64_t hash = cooking_parameters_hash(parameters);
                hash = hash_bytes(hash, settings_.material);
                hash = hash_bytes(hash, settings_.core_mass_fraction);
                hash = hash_bytes(hash, settings_.structural_length_ratio);
                hash = hash_bytes(hash, settings_.skin_search_ratio);
                hash = hash_bytes(hash, settings_.attach_shell_to_core);
                key.parameters_hash = hash;
                key.cooker_version = version();
                key.kind = kind();
                return key;
            }

            CookingReport NodeBeamCooker::cook(const Geometry::TriangleMeshView& mesh,
                                               const CookingParameters& parameters,
                                               ICookedAssetStore* store,
                                               ICookingProgressSink* progress,
                                               std::vector<std::byte>& out)
            {
                out.clear();

                NodeBeamCookContext context;
                context.source = mesh;
                context.authored = parameters;
                context.parameters = resolve_cooking_parameters(parameters);
                context.settings = settings_;

                if (!mesh.has_triangles())
                {
                    context.report.status = CookingStatus::EmptyInput;
                    return context.report;
                }

                const CookedAssetKey key = cache_key(mesh, parameters);

                if (store != nullptr && store->load(key, out) && !out.empty())
                {
                    const NodeBeamAssetView view = load_node_beam_blob(out.data(), out.size());
                    if (view.valid)
                    {
                        // What a cache hit can honestly report is what the asset carries. The
                        // source topology stays unmeasured, because nothing looked at the
                        // source — a clean-looking report there would be an invention.
                        context.report.served_from_cache = true;
                        context.report.asset_bytes = out.size();
                        context.report.node_count = view.node_count;
                        context.report.beam_count = view.beam_count;
                        context.report.collision_triangle_count = view.surface_index_count / 3;
                        context.report.unembedded_vertex_count =
                            view.summary.unskinned_vertex_count;
                        context.report.hausdorff_error = view.summary.hausdorff_error;
                        context.report.mass = float(view.summary.total_mass);
                        context.report.center_of_mass[0] = float(view.summary.center_of_mass.x);
                        context.report.center_of_mass[1] = float(view.summary.center_of_mass.y);
                        context.report.center_of_mass[2] = float(view.summary.center_of_mass.z);
                        context.report.principal_inertia[0] = float(view.core.principal_inertia.x);
                        context.report.principal_inertia[1] = float(view.core.principal_inertia.y);
                        context.report.principal_inertia[2] = float(view.core.principal_inertia.z);
                        // Not carried by the blob: the structural/bracing split is a property
                        // of the cook rather than of the asset, and the records that would
                        // answer it are the ones a cache hit deliberately does not walk. Left
                        // at zero rather than counted, so it reads as unmeasured.
                        context.report.status = CookingStatus::Succeeded;
                        apply_cooking_thresholds(thresholds_, context.report);
                        return context.report;
                    }
                    out.clear();
                    store->evict(key);
                }

                std::vector<std::unique_ptr<Stage>> stages;
                stages.push_back(std::make_unique<RepairStage>());
                stages.push_back(std::make_unique<PlaceNodesStage>());
                stages.push_back(std::make_unique<ConnectBeamsStage>());
                stages.push_back(std::make_unique<SkinStage>());
                stages.push_back(std::make_unique<BuildCoreStage>());
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
