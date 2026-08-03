/**************************************************************************/
/* node_beam_asset.hpp                                                    */
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
 * @file node_beam_asset.hpp
 * @brief The `.sushinodebeam` blob: a vehicle's node cloud, its beams, and what holds them on.
 *
 * §11.2's structure, written down. Five things travel together because a vehicle is not
 * usable without all five: the node cloud, the beam network over it, the collision surface
 * that cloud presents, the rigid core the shell hangs from, and the render mesh's skinning
 * onto the nodes. Split across files they would drift; a shell whose attachment records name
 * nodes from an older cook is a vehicle that loses its doors on load.
 *
 * **The rigid core is a mass, not a body.** @ref NodeBeamCoreRecord carries mass and inertia
 * and nothing else — no shape, no collider handle. The core's *collision* is a `.sushicollision`
 * asset that the vehicle asset (P7-F) names alongside this one, because the same node-beam
 * structure is legitimately reused with different core colliders and because a cooked
 * collider is already a format with an owner. What is here is only what the solver needs to
 * create the core body and attach the shell to it.
 *
 * **A core of zero mass is a pure node-beam vehicle**, and that is the mechanism §11.2 promised
 * rather than a special case bolted on: the architecture does not choose between hybrid and
 * pure, the asset does, and the difference is one number.
 *
 * **The beam records are not @ref BeamConstraintT.** They carry the cooked half — topology,
 * rest length, the four derived numbers, the two plastic parameters — and none of the runtime
 * half: no accumulated strain, no force accumulators, no live rest length. Two reasons, and
 * the second is the load-bearing one. A blob that stored the runtime struct would be a blob
 * whose bytes changed the moment the solver's struct grew a field, which breaks every cached
 * asset for a change no artist made. And `physics/cooking` does not include `physics/constraints`
 * anywhere; making a beam record *be* a constraint would be the first time, for the sake of
 * saving one assignment loop in the instancing code that P7-E owns.
 *
 * **Nothing consumes this yet.** The instancing that turns these records into bodies and
 * constraints is P7-E, and the cooker that produces them is P7-D. This asset's honest status
 * is "produced and validated, not yet consumed" — the same distinction §16.10 was written
 * about, and stated in advance rather than found in a later audit.
 */

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/cooking/cooking_parameters.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        namespace Cooking
        {
            /** @brief Magic at offset 0 of a `.sushinodebeam` blob. */
            constexpr char NODE_BEAM_BLOB_MAGIC[8] = {'S', 'U', 'S', 'H', 'N', 'B', 'E', 'M'};

            /** @brief Current `.sushinodebeam` format version. */
            constexpr std::uint32_t NODE_BEAM_BLOB_VERSION = 1;

            /** @brief How many nodes drive one render vertex. */
            constexpr std::uint32_t NODE_BEAM_SKIN_INFLUENCES = 4;

            /** @brief Bit flags on a cooked node. */
            namespace NodeBeamNodeFlags
            {
                /** @brief Nothing set. */
                inline constexpr std::uint32_t none = 0u;

                /**
                 * @brief The node is welded to the core frame rather than integrated.
                 *
                 * Distinct from a node of zero mass, which is a cooking mistake. This one
                 * says the cooker *meant* it: a mounting point that follows the chassis
                 * exactly and gives the beams around it something to pull against.
                 */
                inline constexpr std::uint32_t fixed = 1u << 0;

                /**
                 * @brief The node is on the collision surface.
                 *
                 * A lattice cook places interior nodes that carry mass and stiffness and
                 * never touch anything. Marking the shell lets a collision pass skip them
                 * without walking the surface index list to find out which are named.
                 */
                inline constexpr std::uint32_t surface = 1u << 1;
            } // namespace NodeBeamNodeFlags

            /**
             * @brief One node: a particle with a mass, a size, and a wind cross-section.
             *
             * Fifty-six bytes with no interior padding, so an array of them serializes as a
             * `memcpy` and the blob stays byte-reproducible.
             */
            struct NodeBeamNodeRecord
            {
                /** @brief Rest position in the asset's own frame. */
                Vector3 position;

                /** @brief Mass in kilogrammes; zero makes the node kinematic. */
                Scalar mass;

                /** @brief Collision radius in metres. */
                Scalar radius;

                /**
                 * @brief Drag coefficient times frontal area, in square metres.
                 *
                 * The product rather than the two factors, because that product is the
                 * only form §11.6's wind coupling uses and a node cannot present a
                 * different area to a different wind direction without an orientation it
                 * does not have.
                 */
                Scalar drag_area;

                /** @brief Which part of the vehicle the node belongs to; see @ref NodeBeamSummary::part_count. */
                std::uint32_t part;

                /** @brief @ref NodeBeamNodeFlags bits. */
                std::uint32_t flags;
            };

            /** @brief What a beam is for; see @ref NodeBeamBeamRecord::kind. */
            namespace NodeBeamBeamKind
            {
                /** @brief Follows a mesh edge or a lattice axis; carries the structure. */
                inline constexpr std::uint32_t structural = 0u;

                /** @brief A diagonal that gives the lattice shear and torsional rigidity. */
                inline constexpr std::uint32_t bracing = 1u;
            } // namespace NodeBeamBeamKind

            /**
             * @brief One beam: two nodes and the numbers a cook derived for them.
             *
             * Seventy-two bytes with no interior padding. The four stiffness-and-failure
             * numbers are what `beam_properties.hpp` produces from a `SoftBodyMaterial` and
             * a cross-section, stored resolved rather than as a material index — the reason
             * @ref BeamConstraintT gives, kept here so the record and the constraint agree.
             */
            struct NodeBeamBeamRecord
            {
                /** @brief First node, an index into the node array. */
                std::uint32_t a;

                /** @brief Second node, an index into the node array. */
                std::uint32_t b;

                /** @brief Which part of the vehicle the beam belongs to; see @ref NodeBeamSummary::part_count. */
                std::uint32_t part;

                /**
                 * @brief Whether the cooker produced this beam as structure or as bracing.
                 *
                 * §11.1's last topology row. Kept because the two are tuned against
                 * different things — a structural beam follows a mesh edge and carries the
                 * panel, a bracing diagonal exists only to stop the lattice shearing — and
                 * an editor that cannot tell them apart cannot show a structure worth
                 * looking at.
                 */
                std::uint32_t kind;

                /** @brief Cooked rest length in metres. */
                Scalar rest_length;

                /** @brief XPBD compliance of the axial row, in metres per newton. */
                Scalar compliance;

                /** @brief Axial velocity damping, as a rate in inverse seconds. */
                Scalar damping;

                /** @brief Load above which the rest length creeps, in newtons. */
                Scalar deform_force;

                /** @brief Load above which the beam breaks, in newtons. */
                Scalar break_force;

                /** @brief Fraction of the elastic deviation made permanent per tick. */
                Scalar plastic_creep;

                /** @brief The permanent strain the beam hardens at. */
                Scalar maximum_plastic_strain;
            };

            /**
             * @brief One shell node held to the rigid core.
             *
             * §10.3's attachment constraint, cooked. Forty-eight bytes, no interior padding.
             * A part comes off when enough of its attachments pass @ref break_force, which is
             * the same mechanism a beam breaking uses and deliberately so: a door that tears
             * at its hinges and a panel that tears at its welds should not be two systems.
             */
            struct NodeBeamAttachmentRecord
            {
                /** @brief The shell node, an index into the node array. */
                std::uint32_t node;

                /** @brief Which part of the vehicle the attachment belongs to. */
                std::uint32_t part;

                /** @brief Where the node is held, in the core's frame. */
                Vector3 core_anchor;

                /** @brief XPBD compliance of the attachment, in metres per newton. */
                Scalar compliance;

                /** @brief Load above which the attachment fails, in newtons. */
                Scalar break_force;
            };

            /**
             * @brief One render vertex, driven by up to four nodes.
             *
             * §11.2's last row: the node-beam half of the two binding strategies, against
             * the tetrahedral embedding a FEM part gets. Forty-four bytes, no interior
             * padding.
             *
             * **The offset is what makes this a binding rather than an approximation.** A
             * weighted sum of node positions alone does not reproduce the vertex it was
             * bound to: the centroid of the four nodes nearest a box corner sits *inside*
             * the corner, so a mesh skinned that way is visibly shrunk before anything has
             * moved. Measured at a coarse lattice it is decimetres, which is not a tuning
             * problem, it is a wrong formulation. So a vertex is stored as a displacement
             * from that centroid, expressed in a frame the nodes themselves define
             * (@ref node_beam_skin_frame) — which reproduces the rest pose exactly, follows
             * the structure when it rotates, and stretches with it when it deforms.
             *
             * The offset is `float` and that is safe where a position would not be, because
             * it is *local*: a render vertex is centimetres from the nodes driving it
             * however far the vehicle is from the world origin.
             *
             * Weights are `float` for the reason @ref SoftBodyBinding gives, and carry the
             * same consequence — four floats that summed to one in the cooker do not sum to
             * one after the round trip. Read them through @ref read_node_beam_skin_weights,
             * which renormalizes, rather than reading the array directly.
             *
             * A vertex influenced by fewer than four nodes zeroes the unused weights and
             * repeats a valid node index in the unused slots. Every index is required to be
             * in range whatever its weight, so a reader never has to test a weight before
             * trusting an index.
             */
            struct NodeBeamSkinRecord
            {
                /** @brief The driving nodes, all in range. */
                std::uint32_t nodes[NODE_BEAM_SKIN_INFLUENCES];

                /** @brief Their weights, summing to one; unused slots are zero. */
                float weights[NODE_BEAM_SKIN_INFLUENCES];

                /** @brief Where the vertex sits relative to the node frame, in metres. */
                float offset[3];
            };

            /**
             * @brief The orthonormal frame three nodes define, for a skin offset.
             *
             * Gram-Schmidt on the two edges from @p a, which is the cheapest construction
             * that rotates with the structure and shears with it only in the sense a node
             * cloud can — nodes have no orientation of their own, so the only frame
             * available is the one their *arrangement* implies.
             *
             * Falls back to the asset's own axes when the three nodes are coincident or
             * collinear, and the fallback matters more than the general case: it must be
             * decided identically at the cook and at every reconstruction, or a vertex is
             * stored in one frame and read in another. That is why this is one function
             * both sides call rather than the same six lines written twice.
             *
             * @param a    The first node's position; the frame's reference corner.
             * @param b    The second node's position.
             * @param c    The third node's position.
             * @param axes Receives the three orthonormal axes.
             */
            inline void node_beam_skin_frame(const Vector3& a, const Vector3& b, const Vector3& c,
                                             Vector3 axes[3]) noexcept
            {
                axes[0] = Vector3{1, 0, 0};
                axes[1] = Vector3{0, 1, 0};
                axes[2] = Vector3{0, 0, 1};

                const Vector3 first = b - a;
                const Scalar first_length = length(first);
                if (!(first_length > Scalar(1e-9)))
                    return;
                const Vector3 u = first * (Scalar(1) / first_length);

                const Vector3 second = c - a;
                const Vector3 rejected = second - u * dot(u, second);
                const Scalar rejected_length = length(rejected);
                if (!(rejected_length > Scalar(1e-9)))
                    return;

                axes[0] = u;
                axes[1] = rejected * (Scalar(1) / rejected_length);
                axes[2] = cross(axes[0], axes[1]);
            }

            /**
             * @brief The rigid core's mass properties — §11.2's chassis, without its shape.
             *
             * A core of zero mass is a pure node-beam vehicle. That is the whole of the
             * hybrid switch, and it is a number rather than a flag so an artist can walk it:
             * a core carrying nine tenths of the mass and a core carrying none are the same
             * asset with the dial in different places.
             */
            struct NodeBeamCoreRecord
            {
                /** @brief Centre of mass in the asset's own frame. */
                Vector3 center_of_mass;

                /** @brief Principal moments about @ref center_of_mass. */
                Vector3 principal_inertia;

                /** @brief Rotation from the principal frame to the asset's frame. */
                Quaternion principal_rotation;

                /** @brief Mass in kilogrammes; zero means there is no core. */
                Scalar mass;
            };

            /** @brief What the structure weighs and how well it was cooked. */
            struct NodeBeamSummary
            {
                /** @brief Core mass plus every node's, in kilogrammes. */
                Scalar total_mass;

                /** @brief Centre of mass of core and nodes together, at rest. */
                Vector3 center_of_mass;

                /** @brief Summed node mass, so the hybrid split is readable. */
                Scalar node_mass;

                /** @brief The shortest cooked beam, in metres. */
                Scalar shortest_beam_length;

                /** @brief The longest cooked beam, in metres. */
                Scalar longest_beam_length;

                /**
                 * @brief The volume the beams' tributary areas were divided out of.
                 *
                 * `beam_tributary_area` conserves `sum(area * length) = volume`, and that
                 * identity is only checkable against the volume the cook actually used.
                 * Recomputing it from the surface later would check a different number.
                 */
                Scalar structure_volume;

                /** @brief How far the collision surface departs from the source mesh. */
                float hausdorff_error;

                /** @brief Render vertices no node was close enough to drive. */
                std::uint32_t unskinned_vertex_count;

                /** @brief Substeps the fidelity dial suggests for this structure. */
                std::uint32_t suggested_substep_count;

                /** @brief How many distinct parts the records name. */
                std::uint32_t part_count;
            };

            /**
             * @brief The fixed header at offset 0.
             *
             * Counts and byte offsets only, so the blob is position independent, and laid
             * out by the same one-pass reservation the soft-body blob uses rather than by a
             * hand-written cursor expression per section.
             */
            struct NodeBeamBlobHeader
            {
                char magic[8];
                std::uint32_t version;
                std::uint32_t total_size;

                std::uint32_t node_count;
                std::uint32_t beam_count;
                std::uint32_t surface_index_count;
                std::uint32_t attachment_count;
                std::uint32_t skin_count;

                std::uint32_t parameters_offset;      /**< CookingParameters. */
                std::uint32_t summary_offset;         /**< NodeBeamSummary. */
                std::uint32_t core_offset;            /**< NodeBeamCoreRecord. */
                std::uint32_t nodes_offset;           /**< NodeBeamNodeRecord[node_count]. */
                std::uint32_t beams_offset;           /**< NodeBeamBeamRecord[beam_count]. */
                std::uint32_t surface_indices_offset; /**< uint32[surface_index_count]. */
                std::uint32_t attachments_offset;     /**< NodeBeamAttachmentRecord[…]. */
                std::uint32_t skin_offset;            /**< NodeBeamSkinRecord[skin_count]. */
            };

            /** @brief The arrays a cooked node-beam asset owns, before serialization. */
            struct NodeBeamAsset
            {
                std::vector<NodeBeamNodeRecord> nodes;
                std::vector<NodeBeamBeamRecord> beams;
                std::vector<std::uint32_t> surface_indices;
                std::vector<NodeBeamAttachmentRecord> attachments;
                std::vector<NodeBeamSkinRecord> skin;
                NodeBeamCoreRecord core{};
                CookingParameters parameters{};
                NodeBeamSummary summary{};
            };

            /** @brief A view over a validated blob; every pointer is into the bytes. */
            struct NodeBeamAssetView
            {
                const NodeBeamNodeRecord* nodes = nullptr;
                std::uint32_t node_count = 0;

                const NodeBeamBeamRecord* beams = nullptr;
                std::uint32_t beam_count = 0;

                const std::uint32_t* surface_indices = nullptr;
                std::uint32_t surface_index_count = 0;

                const NodeBeamAttachmentRecord* attachments = nullptr;
                std::uint32_t attachment_count = 0;

                const NodeBeamSkinRecord* skin = nullptr;
                std::uint32_t skin_count = 0;

                NodeBeamCoreRecord core{};
                CookingParameters parameters{};
                NodeBeamSummary summary{};
                bool valid = false;
            };

            /** @brief Whether @p core describes a chassis at all; see §11.2. */
            inline bool node_beam_has_core(const NodeBeamCoreRecord& core) noexcept
            {
                return core.mass > Scalar(0);
            }

            /**
             * @brief Serializes @p asset into a `.sushinodebeam` blob.
             *
             * Refuses rather than writes a blob it would not itself load. The checks are
             * about *cross-references* — a beam naming a node past the end, an attachment
             * naming one, a skin record naming one — because each of those unchecked is a
             * read into a neighbouring section, which produces a vehicle rather than a crash
             * and is therefore the worse failure.
             *
             * Refuses a beam whose two ends are the same node as well. That one is not a
             * memory-safety question: a self-beam has no axis, projects nothing, and would
             * sit in the structure reporting zero load forever while the panel it was meant
             * to hold flaps. Cheaper to reject at the cook than to explain later.
             *
             * @param asset The cooked arrays.
             * @param out   Receives the blob bytes; cleared first, left empty on refusal.
             * @return False when @p asset is not well formed.
             */
            bool build_node_beam_blob(const NodeBeamAsset& asset, std::vector<std::byte>& out);

            /**
             * @brief Whether @p data is a blob this build can load.
             *
             * @param data The blob bytes.
             * @param size Their length.
             * @return True when @ref load_node_beam_blob will produce a usable view.
             */
            bool validate_node_beam_blob(const std::byte* data, std::size_t size) noexcept;

            /**
             * @brief Rebuilds a view over a validated blob.
             *
             * @param data The blob bytes.
             * @param size Their length.
             * @return A view, or a default (invalid) one when the blob does not validate.
             */
            NodeBeamAssetView load_node_beam_blob(const std::byte* data, std::size_t size) noexcept;

            /**
             * @brief Reads a skin record's weights into the solver's precision, renormalized.
             *
             * The same round-trip residue @ref read_binding_weights exists for, with the same
             * consequence: the reconstruction is a weighted sum of *absolute* positions, so a
             * shortfall of `1e-7` displaces a vertex by ten micrometres a hundred metres from
             * the origin and by centimetres at planet scale. It appears as the render mesh
             * sliding off the structure the further the world is from its origin — which is
             * to say, nowhere near where it would be tested.
             *
             * @tparam T  The precision to read into.
             * @param record The cooked skin record.
             * @param out    Receives the weights, summing to one in @c T.
             */
            template <typename T>
            inline void read_node_beam_skin_weights(const NodeBeamSkinRecord& record,
                                                    T out[NODE_BEAM_SKIN_INFLUENCES]) noexcept
            {
                T total = 0;
                for (std::uint32_t i = 0; i < NODE_BEAM_SKIN_INFLUENCES; ++i)
                {
                    out[i] = T(record.weights[i]);
                    total += out[i];
                }
                // A record whose weights sum to nothing is an unskinned vertex, and leaving
                // it alone lets the caller see that rather than an infinity produced here.
                if (total > T(1e-6) || total < T(-1e-6))
                    for (std::uint32_t i = 0; i < NODE_BEAM_SKIN_INFLUENCES; ++i)
                        out[i] /= total;
            }

            /**
             * @brief Where a skinned render vertex sits, given node positions.
             *
             * The per-tick reconstruction in scalar form, and the expression the runtime
             * will use — `centroid + offset in the node frame`. Published with an explicit
             * position array rather than reading only the asset's, because the runtime
             * calls it with *live* positions and a test that could only call it with rest
             * ones would be measuring a different function.
             *
             * @param view      A validated asset, for the node count and the records.
             * @param record    The vertex's skin record.
             * @param positions Node positions, at least @c view.node_count of them; the
             *                  asset's own rest positions when null.
             * @return The reconstructed position; the origin for an out-of-range record.
             */
            Vector3 evaluate_node_beam_skin(const NodeBeamAssetView& view,
                                            const NodeBeamSkinRecord& record,
                                            const Vector3* positions = nullptr) noexcept;

            /**
             * @brief The offset that binds @p point to @p record's nodes, at rest.
             *
             * The cook side of @ref evaluate_node_beam_skin, and its exact inverse: an
             * offset produced here and fed back through the evaluation reproduces @p point
             * to rounding. One function rather than an expression in the cooker, because
             * the two halves disagreeing is a defect that looks like a mesh sagging
             * slightly and is diagnosed as a solver problem.
             *
             * @param nodes  The node positions the record indexes into.
             * @param record The vertex's nodes and weights; its offset is not read.
             * @param point  Where the vertex rests.
             * @param out    Receives the offset, in the node frame.
             */
            void build_node_beam_skin_offset(const NodeBeamNodeRecord* nodes,
                                             const NodeBeamSkinRecord& record,
                                             const Vector3& point, float out[3]) noexcept;
        } // namespace Cooking
    } // namespace Physics
} // namespace SushiEngine
