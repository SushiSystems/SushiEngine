/**************************************************************************/
/* test_node_beam_asset.cpp                                               */
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

// §11.2's asset. The interesting assertions are not "it round trips" — every format does
// that on the day it is written — but the three things a wide binary format actually gets
// wrong: a section that reads its neighbour because a count and an array disagreed, a
// cross-reference nobody checked, and a record whose padding makes two identical cooks
// produce different bytes.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/physics/cooking/node_beam_asset.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;
using namespace SushiEngine::Physics::Cooking;

namespace
{
    /** @brief A node at @p position with plausible mass, size and drag. */
    NodeBeamNodeRecord node_at(Scalar x, Scalar y, Scalar z, std::uint32_t part = 0)
    {
        NodeBeamNodeRecord node{};
        node.position = Vector3{x, y, z};
        node.mass = 2.5;
        node.radius = 0.05;
        node.drag_area = 0.01;
        node.part = part;
        node.flags = NodeBeamNodeFlags::surface;
        return node;
    }

    /** @brief A beam between @p a and @p b with steel-ish numbers. */
    NodeBeamBeamRecord beam_between(std::uint32_t a, std::uint32_t b, Scalar rest,
                                    std::uint32_t part = 0)
    {
        NodeBeamBeamRecord beam{};
        beam.a = a;
        beam.b = b;
        beam.part = part;
        beam.kind = NodeBeamBeamKind::structural;
        beam.rest_length = rest;
        beam.compliance = 1.0e-7;
        beam.damping = 8.0;
        beam.deform_force = 40000.0;
        beam.break_force = 90000.0;
        beam.plastic_creep = 0.2;
        beam.maximum_plastic_strain = 0.3;
        return beam;
    }

    /**
     * @brief A four-node tetrahedral frame with six beams, one attachment and one skinned
     *        vertex — the smallest asset that exercises every section at once.
     */
    NodeBeamAsset small_asset()
    {
        NodeBeamAsset asset;
        asset.nodes.push_back(node_at(0.0, 0.0, 0.0));
        asset.nodes.push_back(node_at(1.0, 0.0, 0.0));
        asset.nodes.push_back(node_at(0.0, 1.0, 0.0));
        asset.nodes.push_back(node_at(0.0, 0.0, 1.0));

        asset.beams.push_back(beam_between(0, 1, 1.0));
        asset.beams.push_back(beam_between(0, 2, 1.0));
        asset.beams.push_back(beam_between(0, 3, 1.0));
        asset.beams.push_back(beam_between(1, 2, std::sqrt(2.0)));
        asset.beams.push_back(beam_between(1, 3, std::sqrt(2.0)));
        asset.beams.push_back(beam_between(2, 3, std::sqrt(2.0)));
        asset.beams[3].kind = NodeBeamBeamKind::bracing;
        asset.beams[4].kind = NodeBeamBeamKind::bracing;
        asset.beams[5].kind = NodeBeamBeamKind::bracing;

        asset.surface_indices = {0, 1, 2, 0, 2, 3, 0, 3, 1, 1, 3, 2};

        NodeBeamAttachmentRecord attachment{};
        attachment.node = 0;
        attachment.part = 0;
        attachment.core_anchor = Vector3{0.0, 0.0, 0.0};
        attachment.compliance = 1.0e-8;
        attachment.break_force = 25000.0;
        asset.attachments.push_back(attachment);

        NodeBeamSkinRecord skin{};
        skin.nodes[0] = 0;
        skin.nodes[1] = 1;
        skin.nodes[2] = 2;
        skin.nodes[3] = 3;
        skin.weights[0] = 0.25f;
        skin.weights[1] = 0.25f;
        skin.weights[2] = 0.25f;
        skin.weights[3] = 0.25f;
        asset.skin.push_back(skin);

        asset.core.mass = 900.0;
        asset.core.center_of_mass = Vector3{0.25, 0.25, 0.25};
        asset.core.principal_inertia = Vector3{300.0, 400.0, 500.0};
        asset.core.principal_rotation = Quaternion{0.0, 0.0, 0.0, 1.0};

        asset.summary.total_mass = 910.0;
        asset.summary.node_mass = 10.0;
        asset.summary.center_of_mass = Vector3{0.25, 0.25, 0.25};
        asset.summary.shortest_beam_length = 1.0;
        asset.summary.longest_beam_length = std::sqrt(2.0);
        asset.summary.structure_volume = 0.166;
        asset.summary.hausdorff_error = 0.002f;
        asset.summary.unskinned_vertex_count = 0;
        asset.summary.suggested_substep_count = 16;
        asset.summary.part_count = 1;
        return asset;
    }
} // namespace

// The records are copied into the blob with a `memcpy` per array, which is only correct if
// they have no interior padding. Padding would not fail a round trip -- the bytes come back
// as they went in -- it would make two cooks of the same input differ in bytes nobody wrote,
// and the §8.1 cache would then serve entries that look changed and are not.
TEST(Unit_NodeBeamAsset, EveryRecordIsPackedSoTheBlobIsByteReproducible)
{
    EXPECT_EQ(sizeof(NodeBeamNodeRecord),
              sizeof(Vector3) + 3 * sizeof(Scalar) + 2 * sizeof(std::uint32_t));
    EXPECT_EQ(sizeof(NodeBeamBeamRecord), 4 * sizeof(std::uint32_t) + 7 * sizeof(Scalar));
    EXPECT_EQ(sizeof(NodeBeamAttachmentRecord),
              2 * sizeof(std::uint32_t) + sizeof(Vector3) + 2 * sizeof(Scalar));
    EXPECT_EQ(sizeof(NodeBeamSkinRecord),
              NODE_BEAM_SKIN_INFLUENCES * (sizeof(std::uint32_t) + sizeof(float)) +
                  3 * sizeof(float));
    EXPECT_EQ(sizeof(NodeBeamCoreRecord),
              2 * sizeof(Vector3) + sizeof(Quaternion) + sizeof(Scalar));
}

TEST(Unit_NodeBeamAsset, ARoundTripPreservesEverySection)
{
    const NodeBeamAsset asset = small_asset();
    std::vector<std::byte> blob;
    ASSERT_TRUE(build_node_beam_blob(asset, blob));

    const NodeBeamAssetView view = load_node_beam_blob(blob.data(), blob.size());
    ASSERT_TRUE(view.valid);
    ASSERT_EQ(view.node_count, asset.nodes.size());
    ASSERT_EQ(view.beam_count, asset.beams.size());
    ASSERT_EQ(view.surface_index_count, asset.surface_indices.size());
    ASSERT_EQ(view.attachment_count, asset.attachments.size());
    ASSERT_EQ(view.skin_count, asset.skin.size());

    for (std::uint32_t i = 0; i < view.node_count; ++i)
    {
        EXPECT_DOUBLE_EQ(view.nodes[i].position.x, asset.nodes[i].position.x);
        EXPECT_DOUBLE_EQ(view.nodes[i].position.y, asset.nodes[i].position.y);
        EXPECT_DOUBLE_EQ(view.nodes[i].position.z, asset.nodes[i].position.z);
        EXPECT_DOUBLE_EQ(view.nodes[i].mass, asset.nodes[i].mass);
        EXPECT_DOUBLE_EQ(view.nodes[i].drag_area, asset.nodes[i].drag_area);
        EXPECT_EQ(view.nodes[i].flags, asset.nodes[i].flags);
    }
    for (std::uint32_t i = 0; i < view.beam_count; ++i)
    {
        EXPECT_EQ(view.beams[i].a, asset.beams[i].a);
        EXPECT_EQ(view.beams[i].b, asset.beams[i].b);
        EXPECT_EQ(view.beams[i].kind, asset.beams[i].kind);
        EXPECT_DOUBLE_EQ(view.beams[i].rest_length, asset.beams[i].rest_length);
        EXPECT_DOUBLE_EQ(view.beams[i].compliance, asset.beams[i].compliance);
        EXPECT_DOUBLE_EQ(view.beams[i].break_force, asset.beams[i].break_force);
        EXPECT_DOUBLE_EQ(view.beams[i].maximum_plastic_strain,
                         asset.beams[i].maximum_plastic_strain);
    }
    EXPECT_DOUBLE_EQ(view.core.mass, asset.core.mass);
    EXPECT_DOUBLE_EQ(view.core.principal_inertia.y, asset.core.principal_inertia.y);
    EXPECT_DOUBLE_EQ(view.summary.total_mass, asset.summary.total_mass);
    EXPECT_EQ(view.summary.suggested_substep_count, asset.summary.suggested_substep_count);
    EXPECT_DOUBLE_EQ(view.attachments[0].break_force, asset.attachments[0].break_force);
}

// The parameters live inside the blob (§8.3 stage 10) so a re-cook is reproducible without
// the project file. A format that dropped them would be one nobody could reproduce.
TEST(Unit_NodeBeamAsset, TheCookingParametersTravelInsideTheBlob)
{
    NodeBeamAsset asset = small_asset();
    asset.parameters.fidelity = 0.75f;
    asset.parameters.density = 7850.0f;
    asset.parameters.voxel_resolution = 96;

    std::vector<std::byte> blob;
    ASSERT_TRUE(build_node_beam_blob(asset, blob));
    const NodeBeamAssetView view = load_node_beam_blob(blob.data(), blob.size());
    ASSERT_TRUE(view.valid);
    EXPECT_FLOAT_EQ(view.parameters.fidelity, 0.75f);
    EXPECT_FLOAT_EQ(view.parameters.density, 7850.0f);
    EXPECT_EQ(view.parameters.voxel_resolution, 96);
}

// §11.2's hybrid switch, and the whole of it: the architecture does not choose between a
// rigid core and a pure node-beam structure, the asset does, and the difference is one
// number rather than a flag or a second code path.
TEST(Unit_NodeBeamAsset, ACoreOfZeroMassIsAPureNodeBeamVehicle)
{
    NodeBeamAsset asset = small_asset();
    EXPECT_TRUE(node_beam_has_core(asset.core));

    asset.core = NodeBeamCoreRecord{};
    asset.attachments.clear();
    asset.summary.total_mass = asset.summary.node_mass;

    std::vector<std::byte> blob;
    ASSERT_TRUE(build_node_beam_blob(asset, blob));
    const NodeBeamAssetView view = load_node_beam_blob(blob.data(), blob.size());
    ASSERT_TRUE(view.valid);
    EXPECT_FALSE(node_beam_has_core(view.core));
    EXPECT_EQ(view.attachment_count, 0u);
}

TEST(Unit_NodeBeamAsset, ABeamNamingANodePastTheEndIsRefused)
{
    NodeBeamAsset asset = small_asset();
    asset.beams.push_back(beam_between(2, 9, 1.0));

    std::vector<std::byte> blob;
    EXPECT_FALSE(build_node_beam_blob(asset, blob));
    EXPECT_TRUE(blob.empty());
}

// Not a memory-safety question: a self-beam has no axis, projects nothing, and would sit in
// the structure reporting zero load forever while the panel it was meant to hold flaps.
TEST(Unit_NodeBeamAsset, ABeamOntoItselfIsRefused)
{
    NodeBeamAsset asset = small_asset();
    asset.beams.push_back(beam_between(2, 2, 1.0));

    std::vector<std::byte> blob;
    EXPECT_FALSE(build_node_beam_blob(asset, blob));
}

TEST(Unit_NodeBeamAsset, AnAttachmentNamingANodePastTheEndIsRefused)
{
    NodeBeamAsset asset = small_asset();
    NodeBeamAttachmentRecord attachment = asset.attachments[0];
    attachment.node = 7;
    asset.attachments.push_back(attachment);

    std::vector<std::byte> blob;
    EXPECT_FALSE(build_node_beam_blob(asset, blob));
}

// Every slot, whatever its weight. The format's promise is that a reader never has to test a
// weight before trusting an index, and a promise checked only where the weight is non-zero is
// one that fails the first time a reader iterates all four.
TEST(Unit_NodeBeamAsset, AZeroWeightedSkinSlotStillHasToNameARealNode)
{
    NodeBeamAsset asset = small_asset();
    NodeBeamSkinRecord skin{};
    skin.nodes[0] = 1;
    skin.nodes[1] = 2;
    skin.nodes[2] = 3;
    skin.nodes[3] = 40;
    skin.weights[0] = 0.5f;
    skin.weights[1] = 0.3f;
    skin.weights[2] = 0.2f;
    skin.weights[3] = 0.0f;
    asset.skin.push_back(skin);

    std::vector<std::byte> blob;
    EXPECT_FALSE(build_node_beam_blob(asset, blob));
}

TEST(Unit_NodeBeamAsset, ASurfaceIndexListThatIsNotTrianglesIsRefused)
{
    NodeBeamAsset asset = small_asset();
    asset.surface_indices.push_back(1);

    std::vector<std::byte> blob;
    EXPECT_FALSE(build_node_beam_blob(asset, blob));
}

// An asset whose summary disagrees with its records reports one structure and simulates
// another, and the part count is the field a hand edit gets wrong first.
TEST(Unit_NodeBeamAsset, ASummaryThatMiscountsThePartsIsRefused)
{
    NodeBeamAsset asset = small_asset();
    asset.nodes.push_back(node_at(1.0, 1.0, 1.0, 3));

    std::vector<std::byte> blob;
    EXPECT_FALSE(build_node_beam_blob(asset, blob));

    asset.summary.part_count = 4;
    EXPECT_TRUE(build_node_beam_blob(asset, blob));
}

TEST(Unit_NodeBeamAsset, AnAssetWithNoNodesIsRefused)
{
    NodeBeamAsset asset;
    asset.core.mass = 900.0;

    std::vector<std::byte> blob;
    EXPECT_FALSE(build_node_beam_blob(asset, blob));
}

// A structure with no beams at all is legitimate and must not be refused: a vehicle whose
// shell is a single welded panel is nodes plus attachments, and refusing it would make the
// format decide something the asset is supposed to.
TEST(Unit_NodeBeamAsset, AnAssetWithNodesAndNoBeamsIsValid)
{
    NodeBeamAsset asset = small_asset();
    asset.beams.clear();
    asset.summary.shortest_beam_length = 0.0;
    asset.summary.longest_beam_length = 0.0;

    std::vector<std::byte> blob;
    ASSERT_TRUE(build_node_beam_blob(asset, blob));
    const NodeBeamAssetView view = load_node_beam_blob(blob.data(), blob.size());
    ASSERT_TRUE(view.valid);
    EXPECT_EQ(view.beam_count, 0u);
    EXPECT_EQ(view.beams, nullptr);
}

TEST(Unit_NodeBeamAsset, ABlobWithTheWrongMagicOrVersionDoesNotLoad)
{
    const NodeBeamAsset asset = small_asset();
    std::vector<std::byte> blob;
    ASSERT_TRUE(build_node_beam_blob(asset, blob));
    ASSERT_TRUE(validate_node_beam_blob(blob.data(), blob.size()));

    std::vector<std::byte> wrong_magic = blob;
    wrong_magic[1] = std::byte{'X'};
    EXPECT_FALSE(validate_node_beam_blob(wrong_magic.data(), wrong_magic.size()));
    EXPECT_FALSE(load_node_beam_blob(wrong_magic.data(), wrong_magic.size()).valid);

    std::vector<std::byte> wrong_version = blob;
    const std::uint32_t future = NODE_BEAM_BLOB_VERSION + 1;
    std::memcpy(wrong_version.data() + offsetof(NodeBeamBlobHeader, version), &future,
                sizeof(future));
    EXPECT_FALSE(validate_node_beam_blob(wrong_version.data(), wrong_version.size()));
}

TEST(Unit_NodeBeamAsset, ATruncatedBlobDoesNotLoad)
{
    const NodeBeamAsset asset = small_asset();
    std::vector<std::byte> blob;
    ASSERT_TRUE(build_node_beam_blob(asset, blob));

    EXPECT_FALSE(validate_node_beam_blob(blob.data(), blob.size() - 1));
    EXPECT_FALSE(validate_node_beam_blob(blob.data(), sizeof(NodeBeamBlobHeader) / 2));
    EXPECT_FALSE(validate_node_beam_blob(nullptr, blob.size()));
}

// The check that matters most, because the failure it catches produces geometry rather than
// a crash: a count raised without the bytes behind it makes the last records of a section
// read whatever the next section holds.
TEST(Unit_NodeBeamAsset, ACountRaisedWithoutItsBytesIsCaught)
{
    const NodeBeamAsset asset = small_asset();
    std::vector<std::byte> blob;
    ASSERT_TRUE(build_node_beam_blob(asset, blob));

    std::vector<std::byte> lying = blob;
    const std::uint32_t inflated = 4096;
    std::memcpy(lying.data() + offsetof(NodeBeamBlobHeader, node_count), &inflated,
                sizeof(inflated));
    EXPECT_FALSE(validate_node_beam_blob(lying.data(), lying.size()));

    lying = blob;
    std::memcpy(lying.data() + offsetof(NodeBeamBlobHeader, beam_count), &inflated,
                sizeof(inflated));
    EXPECT_FALSE(validate_node_beam_blob(lying.data(), lying.size()));
}

// Validation is not only a write-time check: a blob may come from an older writer or from a
// hand edit, so the cross-references are re-checked against the bytes on the way in.
TEST(Unit_NodeBeamAsset, AHandEditedCrossReferenceIsCaughtOnLoad)
{
    const NodeBeamAsset asset = small_asset();
    std::vector<std::byte> blob;
    ASSERT_TRUE(build_node_beam_blob(asset, blob));

    NodeBeamBlobHeader header{};
    std::memcpy(&header, blob.data(), sizeof(header));

    std::vector<std::byte> edited = blob;
    const std::uint32_t past_the_end = 12;
    std::memcpy(edited.data() + header.beams_offset + offsetof(NodeBeamBeamRecord, b),
                &past_the_end, sizeof(past_the_end));
    EXPECT_FALSE(validate_node_beam_blob(edited.data(), edited.size()));

    edited = blob;
    std::memcpy(edited.data() + header.surface_indices_offset, &past_the_end,
                sizeof(past_the_end));
    EXPECT_FALSE(validate_node_beam_blob(edited.data(), edited.size()));
}

// §8.6 invariant 2, for the node-beam binding: at rest, the reconstruction reproduces the
// point that was bound. The vertex chosen is deliberately *not* the centroid of its nodes —
// a corner of the tetrahedron's bounding region — because the centroid is the one point a
// weighted sum alone would have got right, and it is the whole reason the offset exists.
TEST(Unit_NodeBeamAsset, AtRestTheSkinReconstructsExactlyWhereItWasBound)
{
    NodeBeamAsset asset = small_asset();
    const Vector3 vertex{0.9, 0.9, 0.9};

    NodeBeamSkinRecord skin = asset.skin[0];
    build_node_beam_skin_offset(asset.nodes.data(), skin, vertex, skin.offset);
    asset.skin.push_back(skin);

    std::vector<std::byte> blob;
    ASSERT_TRUE(build_node_beam_blob(asset, blob));
    const NodeBeamAssetView view = load_node_beam_blob(blob.data(), blob.size());
    ASSERT_TRUE(view.valid);

    const Vector3 reconstructed = evaluate_node_beam_skin(view, view.skin[1]);
    EXPECT_NEAR(reconstructed.x, vertex.x, 1e-6);
    EXPECT_NEAR(reconstructed.y, vertex.y, 1e-6);
    EXPECT_NEAR(reconstructed.z, vertex.z, 1e-6);

    // And a zero offset is still the centroid, so the frame changes nothing about the simple
    // case the offset generalizes.
    const Vector3 centroid = evaluate_node_beam_skin(view, view.skin[0]);
    EXPECT_NEAR(centroid.x, 0.25, 1e-12);
    EXPECT_NEAR(centroid.y, 0.25, 1e-12);
    EXPECT_NEAR(centroid.z, 0.25, 1e-12);
}

// The property that makes the frame worth its twelve bytes: a rigid motion of the nodes
// carries the skinned vertex with it. A stored world-space displacement would leave the
// vertex behind the moment the vehicle turned a corner, which is a defect that never appears
// in a straight-line test.
TEST(Unit_NodeBeamAsset, ARigidMotionOfTheNodesCarriesTheSkinnedVertex)
{
    NodeBeamAsset asset = small_asset();
    const Vector3 vertex{0.9, -0.4, 0.2};

    NodeBeamSkinRecord skin = asset.skin[0];
    build_node_beam_skin_offset(asset.nodes.data(), skin, vertex, skin.offset);
    asset.skin[0] = skin;

    std::vector<std::byte> blob;
    ASSERT_TRUE(build_node_beam_blob(asset, blob));
    const NodeBeamAssetView view = load_node_beam_blob(blob.data(), blob.size());
    ASSERT_TRUE(view.valid);

    // A quarter turn about z, then a translation.
    const Vector3 shift{10.0, -3.0, 7.0};
    const auto moved = [&shift](const Vector3& point)
    { return Vector3{-point.y, point.x, point.z} + shift; };

    std::vector<Vector3> positions;
    for (std::uint32_t i = 0; i < view.node_count; ++i)
        positions.push_back(moved(view.nodes[i].position));

    const Vector3 reconstructed =
        evaluate_node_beam_skin(view, view.skin[0], positions.data());
    const Vector3 expected = moved(vertex);
    EXPECT_NEAR(reconstructed.x, expected.x, 1e-6);
    EXPECT_NEAR(reconstructed.y, expected.y, 1e-6);
    EXPECT_NEAR(reconstructed.z, expected.z, 1e-6);
}

// Three coincident or collinear nodes define no frame, and the fallback must be the *same*
// one on both sides — a vertex stored against the asset's own axes and read back against a
// degenerate Gram-Schmidt result would land somewhere neither side intended.
TEST(Unit_NodeBeamAsset, ADegenerateNodeTripleFallsBackToTheAssetAxesOnBothSides)
{
    NodeBeamAsset asset = small_asset();
    // Three nodes on one line: 0, 1 and a new one beyond it.
    asset.nodes.push_back(node_at(2.0, 0.0, 0.0));
    asset.summary.node_mass += 2.5;
    asset.summary.total_mass += 2.5;

    NodeBeamSkinRecord skin{};
    skin.nodes[0] = 0;
    skin.nodes[1] = 1;
    skin.nodes[2] = 4;
    skin.nodes[3] = 0;
    skin.weights[0] = 0.5f;
    skin.weights[1] = 0.25f;
    skin.weights[2] = 0.25f;
    skin.weights[3] = 0.0f;

    const Vector3 vertex{0.3, 0.7, -0.2};
    build_node_beam_skin_offset(asset.nodes.data(), skin, vertex, skin.offset);
    asset.skin.push_back(skin);

    std::vector<std::byte> blob;
    ASSERT_TRUE(build_node_beam_blob(asset, blob));
    const NodeBeamAssetView view = load_node_beam_blob(blob.data(), blob.size());
    ASSERT_TRUE(view.valid);

    const Vector3 reconstructed = evaluate_node_beam_skin(view, view.skin[1]);
    EXPECT_NEAR(reconstructed.x, vertex.x, 1e-6);
    EXPECT_NEAR(reconstructed.y, vertex.y, 1e-6);
    EXPECT_NEAR(reconstructed.z, vertex.z, 1e-6);
}

// The residue is the point. Four floats that summed to one in the cooker do not sum to one
// after the round trip, and the reconstruction is a weighted sum of *absolute* positions --
// so the shortfall scales with distance from the origin and shows up as the render mesh
// sliding off the structure nowhere near where anyone would test it.
TEST(Unit_NodeBeamAsset, RenormalizingKeepsASkinnedVertexPutFarFromTheOrigin)
{
    NodeBeamAsset asset = small_asset();
    const Scalar far_away = 1.0e5;
    for (NodeBeamNodeRecord& node : asset.nodes)
        node.position = node.position + Vector3{far_away, far_away, far_away};

    // Thirds, which no binary float represents exactly and whose sum is therefore not one.
    NodeBeamSkinRecord skin{};
    skin.nodes[0] = 0;
    skin.nodes[1] = 1;
    skin.nodes[2] = 2;
    skin.nodes[3] = 0;
    skin.weights[0] = 1.0f / 3.0f;
    skin.weights[1] = 1.0f / 3.0f;
    skin.weights[2] = 1.0f / 3.0f;
    skin.weights[3] = 0.0f;
    asset.skin.push_back(skin);

    std::vector<std::byte> blob;
    ASSERT_TRUE(build_node_beam_blob(asset, blob));
    const NodeBeamAssetView view = load_node_beam_blob(blob.data(), blob.size());
    ASSERT_TRUE(view.valid);

    Scalar weights[NODE_BEAM_SKIN_INFLUENCES];
    read_node_beam_skin_weights<Scalar>(view.skin[1], weights);
    Scalar total = 0;
    for (const Scalar weight : weights)
        total += weight;
    EXPECT_NEAR(total, 1.0, 1e-15);

    const Vector3 reconstructed = evaluate_node_beam_skin(view, view.skin[1]);
    EXPECT_NEAR(reconstructed.x, far_away + 1.0 / 3.0, 1e-9);
    EXPECT_NEAR(reconstructed.y, far_away + 1.0 / 3.0, 1e-9);
    EXPECT_NEAR(reconstructed.z, far_away, 1e-9);
}

// An unskinned vertex is a real cooking outcome, not an error, and it must read as zero
// rather than as an infinity produced by dividing by a weight sum of nothing.
TEST(Unit_NodeBeamAsset, AnUnskinnedVertexReadsAsZeroRatherThanAnInfinity)
{
    NodeBeamSkinRecord skin{};
    skin.nodes[0] = 0;
    skin.nodes[1] = 0;
    skin.nodes[2] = 0;
    skin.nodes[3] = 0;
    for (float& weight : skin.weights)
        weight = 0.0f;

    Scalar weights[NODE_BEAM_SKIN_INFLUENCES];
    read_node_beam_skin_weights<Scalar>(skin, weights);
    for (const Scalar weight : weights)
    {
        EXPECT_TRUE(std::isfinite(weight));
        EXPECT_DOUBLE_EQ(weight, 0.0);
    }
}

// A view that never loaded must not be walked. The pointers are null, and a caller that
// forgot to check `valid` gets the origin rather than a read through nothing.
TEST(Unit_NodeBeamAsset, AnInvalidViewEvaluatesToTheOrigin)
{
    NodeBeamAssetView view;
    NodeBeamSkinRecord skin{};
    skin.weights[0] = 1.0f;

    const Vector3 reconstructed = evaluate_node_beam_skin(view, skin);
    EXPECT_DOUBLE_EQ(reconstructed.x, 0.0);
    EXPECT_DOUBLE_EQ(reconstructed.y, 0.0);
    EXPECT_DOUBLE_EQ(reconstructed.z, 0.0);
}

// Two builds of the same asset must produce the same bytes, or the §8.1 cache serves entries
// that look changed and are not. This is what the packing test above protects, asserted end
// to end rather than through the record sizes alone.
TEST(Unit_NodeBeamAsset, TwoCooksOfTheSameAssetProduceTheSameBytes)
{
    const NodeBeamAsset asset = small_asset();
    std::vector<std::byte> first;
    std::vector<std::byte> second;
    ASSERT_TRUE(build_node_beam_blob(asset, first));
    ASSERT_TRUE(build_node_beam_blob(asset, second));

    ASSERT_EQ(first.size(), second.size());
    EXPECT_EQ(std::memcmp(first.data(), second.data(), first.size()), 0);
}
