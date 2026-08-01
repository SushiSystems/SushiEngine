/**************************************************************************/
/* node_beam_cooker.hpp                                                   */
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
 * @file node_beam_cooker.hpp
 * @brief §11.3: a vehicle mesh in, a `.sushinodebeam` out — and the end of hand-authoring.
 *
 * §11.2's first correction to BeamNG, which is the one every other correction stands on.
 * There, a vehicle *is* the node/beam description: thousands of numbers, each authored by a
 * human, each only as right as someone's guess. Here the topology comes from the mesh at the
 * fidelity dial's resolution and every beam's four numbers come from a `SoftBodyMaterial` and
 * a cross-section — "sheet steel" is a Young's modulus and a yield stress, not four thousand
 * tuned constants. Hand-authoring stays possible, because the asset is data; it stops being
 * mandatory.
 *
 * **The lattice is the tetrahedralizer's, not a second voxelizer.** §8.3's stage 2 already
 * voxelizes, flood-fills the interior, conforms the boundary and hands back a lattice with
 * per-vertex masses and an outward-wound surface. A node-beam cook needs exactly those four
 * things, so it asks for them rather than growing a parallel implementation that would decide
 * "inside" differently on the same dirty mesh — and it is on a dirty mesh that the two would
 * disagree, which is to say on the meshes that matter.
 *
 * **The beams are the lattice's edges, deduplicated.** Every tetrahedron contributes six, and
 * a lattice tetrahedralization's edge set already contains both kinds §11.1 names: the ones
 * along a grid axis, which carry the structure, and the diagonals, which are the bracing that
 * stops the network shearing. So §11.3's "add bracing beams by a diagonal rule" is a
 * *classification* here rather than a second construction pass — the diagonals are already
 * there, and inventing more would double-brace a lattice that is already braced.
 *
 * **The render mesh is skinned by distance, not embedded.** §11.2's last row: a FEM part gets
 * barycentric tetrahedral embedding, a node-beam part gets distance-weighted node skinning.
 * They are different bindings because they answer different questions — an embedded vertex
 * follows the element containing it, a skinned vertex follows the nodes near it — and a node
 * cloud whose beams have broken has no elements left to be contained by.
 */

#include <cstdint>
#include <vector>

#include <SushiEngine/geometry/mesh_distance_query.hpp>
#include <SushiEngine/physics/cooking/cooker_interface.hpp>
#include <SushiEngine/physics/cooking/node_beam_asset.hpp>
#include <SushiEngine/physics/cooking/tetrahedral_mesh.hpp>
#include <SushiEngine/physics/soft/soft_body_material.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        namespace Cooking
        {
            /** @brief The `NodeBeamCooker`'s output-format version (see §8.1's cache key). */
            constexpr std::uint32_t NODE_BEAM_COOKER_VERSION = 1;

            /**
             * @brief The vehicle-shaped decisions a node-beam cook needs and the dial cannot make.
             *
             * Separate from @ref CookingParameters, which is shared by every cooker and is half
             * of §8.1's cache key. A material belongs to *this* cook and to no other, and adding
             * it to the shared record would make every collision and soft-body asset in the
             * project carry a field they have no use for.
             *
             * The consequence is one the cooker has to handle rather than the caller: these
             * settings are **folded into the cache key by the cooker itself**
             * (@ref NodeBeamCooker::cache_key). Without that, the same mesh cooked as aluminium
             * and as steel would resolve to one key and the second cook would be served the
             * first one's asset — a cache serving the wrong material is worse than no cache.
             */
            struct NodeBeamCookerSettings
            {
                /**
                 * @brief The material every beam's numbers are derived from.
                 *
                 * Defaults to whatever `SoftBodyMaterialT` defaults to, which is a soft
                 * unbreakable solid — a safe thing to cook by accident, because it deforms
                 * visibly and never loses parts, so a forgotten material looks wrong rather
                 * than looking right and failing later.
                 */
                SoftBodyMaterial material{};

                /**
                 * @brief The fraction of the cooked mass the rigid core carries, in [0, 1].
                 *
                 * §11.2's dial. At zero the whole mass is in the node cloud and the asset is a
                 * pure node-beam vehicle; at nine tenths the chassis is rigid and the shell is
                 * a skin over it. The default follows §11.2's "the core carries the bulk".
                 */
                float core_mass_fraction = 0.8f;

                /**
                 * @brief Nodes past which a beam is not considered structural, as a length ratio.
                 *
                 * A lattice edge along a grid axis is one cell long; a face diagonal is √2 and a
                 * body diagonal √3. Anything longer than this multiple of the cell size is
                 * bracing. Between the two values rather than at either, so a conforming pass
                 * that moved a boundary vertex does not reclassify the edge it moved.
                 */
                float structural_length_ratio = 1.2f;

                /**
                 * @brief How far past the cell size a node may be from a render vertex, as a ratio.
                 *
                 * A vertex with no node inside this radius is reported unskinned rather than
                 * bound to whatever happened to be nearest — a weight computed against a node
                 * on the other side of the car is not a binding, it is a tether.
                 */
                float skin_search_ratio = 2.0f;

                /**
                 * @brief Whether the cooker attaches shell nodes to the core.
                 *
                 * On, and off is meaningful: an asset whose shell is not attached is a node
                 * cloud that falls off its own chassis, which is what a pure node-beam cook
                 * wants and what a hybrid one never does. Ignored when
                 * @ref core_mass_fraction leaves no core.
                 */
                bool attach_shell_to_core = true;
            };

            /**
             * @brief Turns a vehicle mesh into a node cloud, a beam network, and a rigid core.
             *
             * Stateless apart from its settings and thresholds, and safe on a background
             * thread; two concurrent cooks want two cookers, since @ref cook is not reentrant.
             */
            class NodeBeamCooker final : public IMeshCooker
            {
            public:
                const char* name() const noexcept override { return "NodeBeamCooker"; }
                std::uint32_t version() const noexcept override
                {
                    return NODE_BEAM_COOKER_VERSION;
                }
                CookedAssetKind kind() const noexcept override
                {
                    return CookedAssetKind::NodeBeam;
                }

                CookedAssetKey cache_key(const Geometry::TriangleMeshView& mesh,
                                         const CookingParameters& parameters)
                    const noexcept override;

                CookingReport cook(const Geometry::TriangleMeshView& mesh,
                                   const CookingParameters& parameters, ICookedAssetStore* store,
                                   ICookingProgressSink* progress,
                                   std::vector<std::byte>& out) override;

                /** @brief Sets the vehicle-shaped decisions; part of the cache key. */
                void set_settings(const NodeBeamCookerSettings& settings) noexcept
                {
                    settings_ = settings;
                }

                /** @brief The settings currently applied. */
                const NodeBeamCookerSettings& settings() const noexcept { return settings_; }

                /** @brief Sets the limits a produced asset is judged against (§8.5). */
                void set_thresholds(const CookingThresholds& thresholds) noexcept
                {
                    thresholds_ = thresholds;
                }

                /** @brief The limits currently applied. */
                const CookingThresholds& thresholds() const noexcept { return thresholds_; }

            private:
                NodeBeamCookerSettings settings_;
                CookingThresholds thresholds_;
            };

            /**
             * @brief The state a node-beam cook's stages pass between them.
             *
             * Public for the reason the other two contexts are: a project adding a stage has to
             * be able to name what that stage reads, and a context hidden in a translation unit
             * closes the extension point §4.2 exists to open.
             */
            struct NodeBeamCookContext
            {
                /** @brief The source geometry as handed in. */
                Geometry::TriangleMeshView source;

                /** @brief The dial, already resolved. */
                DerivedCookingParameters parameters;

                /** @brief The parameters as authored, for the copy the blob carries. */
                CookingParameters authored;

                /** @brief The vehicle-shaped decisions. */
                NodeBeamCookerSettings settings;

                /** @brief The repaired source; every later stage reads this. */
                Geometry::TriangleMesh repaired;

                /** @brief A distance hierarchy over @ref repaired. */
                Geometry::MeshDistanceQuery surface;

                /** @brief The lattice the nodes and the collision surface come from. */
                TetrahedralMesh lattice;

                /** @brief The asset being assembled. */
                NodeBeamAsset asset;

                /** @brief Filled as the stages learn things; returned to the caller. */
                CookingReport report;
            };
        } // namespace Cooking
    } // namespace Physics
} // namespace SushiEngine
