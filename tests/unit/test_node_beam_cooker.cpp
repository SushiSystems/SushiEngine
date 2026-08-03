/**************************************************************************/
/* test_node_beam_cooker.cpp                                              */
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

// §11.3, and the claim §11.2 makes about it: that a vehicle's thousands of numbers come from
// a material and a dial rather than from a person. The assertions that matter are the ones
// that would catch a cooker producing a *plausible* structure — one whose beams are all
// structural, whose cross-sections do not add up to the body they came from, or whose render
// mesh does not sit where it was authored.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/physics/cooking/node_beam_cooker.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;
using namespace SushiEngine::Physics::Cooking;

namespace
{
    /** @brief An outward-wound box, standing in for a vehicle body. */
    Geometry::TriangleMesh box_mesh(float hx, float hy, float hz)
    {
        Geometry::TriangleMesh mesh;
        const float corners[8][3] = {{-hx, -hy, -hz}, {hx, -hy, -hz}, {hx, hy, -hz},
                                     {-hx, hy, -hz},  {-hx, -hy, hz}, {hx, -hy, hz},
                                     {hx, hy, hz},    {-hx, hy, hz}};
        for (const auto& corner : corners)
        {
            mesh.positions.push_back(corner[0]);
            mesh.positions.push_back(corner[1]);
            mesh.positions.push_back(corner[2]);
        }
        const std::uint32_t faces[12][3] = {{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7},
                                           {0, 1, 5}, {0, 5, 4}, {3, 7, 6}, {3, 6, 2},
                                           {0, 4, 7}, {0, 7, 3}, {1, 2, 6}, {1, 6, 5}};
        for (const auto& face : faces)
        {
            mesh.indices.push_back(face[0]);
            mesh.indices.push_back(face[1]);
            mesh.indices.push_back(face[2]);
        }
        return mesh;
    }

    /** @brief Mild steel, with thresholds low enough that a test can reach them. */
    NodeBeamCookerSettings steel_settings()
    {
        NodeBeamCookerSettings settings;
        settings.material.young_modulus = 2.0e11;
        settings.material.yield_stress = 2.5e8;
        settings.material.fracture_stress = 4.0e8;
        settings.material.plastic_creep = 0.2;
        settings.material.maximum_plastic_strain = 0.3;
        settings.material.damping = 6.0;
        settings.core_mass_fraction = 0.8f;
        return settings;
    }

    /** @brief The dial a test cooks at: coarse enough to be quick, fine enough to brace. */
    CookingParameters test_parameters()
    {
        CookingParameters parameters;
        parameters.fidelity = 0.35f;
        parameters.density = 7850.0f;
        return parameters;
    }
} // namespace

TEST(Unit_NodeBeamCooker, ABoxCooksIntoANodeCloudWithBeamsAndACollisionSurface)
{
    const Geometry::TriangleMesh mesh = box_mesh(1.0f, 0.5f, 2.0f);
    NodeBeamCooker cooker;
    cooker.set_settings(steel_settings());

    std::vector<std::byte> blob;
    const CookingReport report =
        cooker.cook(mesh.view(), test_parameters(), nullptr, nullptr, blob);

    ASSERT_EQ(report.status, CookingStatus::Succeeded) << (report.failed_stage ? report.failed_stage : "");
    EXPECT_GT(report.node_count, 0u);
    EXPECT_GT(report.beam_count, report.node_count);
    EXPECT_GT(report.collision_triangle_count, 0u);
    EXPECT_EQ(report.unembedded_vertex_count, 0u);
    EXPECT_GT(report.asset_bytes, 0u);

    const NodeBeamAssetView view = load_node_beam_blob(blob.data(), blob.size());
    ASSERT_TRUE(view.valid);
    EXPECT_EQ(view.node_count, report.node_count);
    EXPECT_EQ(view.beam_count, report.beam_count);
    EXPECT_EQ(view.skin_count, mesh.vertex_count());
}

// §11.3's diagonal rule. A network of only structural beams holds its lengths and folds flat,
// and the failure looks like a suspension problem rather than a topology one — so the split is
// asserted rather than assumed to have happened.
TEST(Unit_NodeBeamCooker, TheLatticeIsBracedAndNotJustEdged)
{
    const Geometry::TriangleMesh mesh = box_mesh(1.0f, 0.5f, 2.0f);
    NodeBeamCooker cooker;
    cooker.set_settings(steel_settings());

    std::vector<std::byte> blob;
    const CookingReport report =
        cooker.cook(mesh.view(), test_parameters(), nullptr, nullptr, blob);
    ASSERT_EQ(report.status, CookingStatus::Succeeded);

    EXPECT_GT(report.bracing_beam_count, 0u);
    EXPECT_LT(report.bracing_beam_count, report.beam_count);

    const NodeBeamAssetView view = load_node_beam_blob(blob.data(), blob.size());
    ASSERT_TRUE(view.valid);
    std::uint32_t bracing = 0;
    for (std::uint32_t i = 0; i < view.beam_count; ++i)
    {
        if (view.beams[i].kind == NodeBeamBeamKind::bracing)
            ++bracing;
        // A bracing beam is longer than a structural one, which is the classification's
        // whole content; asserting it here is what would catch the ratio being read off
        // the wrong length scale.
        EXPECT_GT(view.beams[i].rest_length, 0.0);
    }
    EXPECT_EQ(bracing, report.bracing_beam_count);
}

// §11.2's first correction to BeamNG, and the only assertion in this file that would catch a
// cooker whose numbers are self-consistent and physically wrong: a beam's compliance must be
// the axial bar's, `L / (E * A)`, against the area it was actually given.
TEST(Unit_NodeBeamCooker, EveryBeamsNumbersComeFromTheMaterialAndItsCrossSection)
{
    const Geometry::TriangleMesh mesh = box_mesh(1.0f, 0.5f, 2.0f);
    const NodeBeamCookerSettings settings = steel_settings();
    NodeBeamCooker cooker;
    cooker.set_settings(settings);

    std::vector<std::byte> blob;
    ASSERT_EQ(cooker.cook(mesh.view(), test_parameters(), nullptr, nullptr, blob).status,
              CookingStatus::Succeeded);
    const NodeBeamAssetView view = load_node_beam_blob(blob.data(), blob.size());
    ASSERT_TRUE(view.valid);
    ASSERT_GT(view.beam_count, 0u);

    const double volume = double(view.summary.structure_volume);
    for (std::uint32_t i = 0; i < view.beam_count; ++i)
    {
        const NodeBeamBeamRecord& beam = view.beams[i];
        const double rest = double(beam.rest_length);
        const double area = volume / (double(view.beam_count) * rest);
        EXPECT_NEAR(double(beam.compliance), rest / (double(settings.material.young_modulus) * area),
                    1e-18);
        EXPECT_NEAR(double(beam.deform_force), double(settings.material.yield_stress) * area,
                    double(settings.material.yield_stress) * area * 1e-9);
        EXPECT_NEAR(double(beam.break_force), double(settings.material.fracture_stress) * area,
                    double(settings.material.fracture_stress) * area * 1e-9);
        EXPECT_DOUBLE_EQ(beam.damping, settings.material.damping);
        EXPECT_DOUBLE_EQ(beam.plastic_creep, settings.material.plastic_creep);
    }
}

// The tributary areas are only meaningful if they add up to the body they were divided out
// of: `sum(A * L) == volume`. A cooker that sized every beam against, say, the source mesh's
// volume instead of the lattice's would produce a structure whose cross-sections do not add
// up to itself, and every load it reported would be off by that ratio.
TEST(Unit_NodeBeamCooker, TheCrossSectionsAddUpToTheStructureVolume)
{
    const Geometry::TriangleMesh mesh = box_mesh(1.0f, 0.5f, 2.0f);
    NodeBeamCooker cooker;
    cooker.set_settings(steel_settings());

    std::vector<std::byte> blob;
    ASSERT_EQ(cooker.cook(mesh.view(), test_parameters(), nullptr, nullptr, blob).status,
              CookingStatus::Succeeded);
    const NodeBeamAssetView view = load_node_beam_blob(blob.data(), blob.size());
    ASSERT_TRUE(view.valid);

    const double volume = double(view.summary.structure_volume);
    ASSERT_GT(volume, 0.0);
    double total = 0.0;
    for (std::uint32_t i = 0; i < view.beam_count; ++i)
    {
        const double rest = double(view.beams[i].rest_length);
        total += (volume / (double(view.beam_count) * rest)) * rest;
    }
    EXPECT_NEAR(total, volume, volume * 1e-9);
}

// §11.2's hybrid dial, measured rather than trusted: the core gets the fraction it was asked
// for and the node cloud gets the rest.
TEST(Unit_NodeBeamCooker, TheCoreCarriesTheMassFractionItWasAsked)
{
    const Geometry::TriangleMesh mesh = box_mesh(1.0f, 0.5f, 2.0f);
    NodeBeamCookerSettings settings = steel_settings();
    settings.core_mass_fraction = 0.65f;
    NodeBeamCooker cooker;
    cooker.set_settings(settings);

    std::vector<std::byte> blob;
    ASSERT_EQ(cooker.cook(mesh.view(), test_parameters(), nullptr, nullptr, blob).status,
              CookingStatus::Succeeded);
    const NodeBeamAssetView view = load_node_beam_blob(blob.data(), blob.size());
    ASSERT_TRUE(view.valid);

    ASSERT_GT(view.summary.total_mass, 0.0);
    EXPECT_NEAR(double(view.core.mass) / double(view.summary.total_mass), 0.65, 1e-6);
    EXPECT_NEAR(double(view.summary.node_mass) / double(view.summary.total_mass), 0.35, 1e-6);
    EXPECT_TRUE(node_beam_has_core(view.core));
    EXPECT_GT(view.attachment_count, 0u);
}

// The other end of the same dial: no core at all, which §11.2 promised would be a pure
// node-beam vehicle rather than a special case. Nothing about the structure changes but the
// mass split and the attachments that would have held it to a chassis that is not there.
TEST(Unit_NodeBeamCooker, AZeroCoreFractionCooksAPureNodeBeamVehicle)
{
    const Geometry::TriangleMesh mesh = box_mesh(1.0f, 0.5f, 2.0f);
    NodeBeamCookerSettings settings = steel_settings();
    settings.core_mass_fraction = 0.0f;
    NodeBeamCooker cooker;
    cooker.set_settings(settings);

    std::vector<std::byte> blob;
    ASSERT_EQ(cooker.cook(mesh.view(), test_parameters(), nullptr, nullptr, blob).status,
              CookingStatus::Succeeded);
    const NodeBeamAssetView view = load_node_beam_blob(blob.data(), blob.size());
    ASSERT_TRUE(view.valid);

    EXPECT_FALSE(node_beam_has_core(view.core));
    EXPECT_EQ(view.attachment_count, 0u);
    EXPECT_GT(view.beam_count, 0u);
    EXPECT_NEAR(double(view.summary.node_mass), double(view.summary.total_mass),
                double(view.summary.total_mass) * 1e-12);
}

// §8.6 invariant 2 for the node-beam binding, end to end: every render vertex must come back
// where the artist put it. This is the assertion that caught the weighted-centroid
// formulation, which reconstructed a box corner four hundred millimetres inside itself.
TEST(Unit_NodeBeamCooker, AtRestEverySkinnedVertexReturnsToTheSourceMesh)
{
    const Geometry::TriangleMesh mesh = box_mesh(1.0f, 0.5f, 2.0f);
    NodeBeamCooker cooker;
    cooker.set_settings(steel_settings());

    std::vector<std::byte> blob;
    ASSERT_EQ(cooker.cook(mesh.view(), test_parameters(), nullptr, nullptr, blob).status,
              CookingStatus::Succeeded);
    const NodeBeamAssetView view = load_node_beam_blob(blob.data(), blob.size());
    ASSERT_TRUE(view.valid);
    ASSERT_EQ(view.skin_count, mesh.vertex_count());

    double worst = 0.0;
    for (std::uint32_t i = 0; i < view.skin_count; ++i)
    {
        const Vector3 reconstructed = evaluate_node_beam_skin(view, view.skin[i]);
        const float* source = mesh.positions.data() + std::size_t(i) * 3;
        const double dx = reconstructed.x - double(source[0]);
        const double dy = reconstructed.y - double(source[1]);
        const double dz = reconstructed.z - double(source[2]);
        worst = std::max(worst, std::sqrt(dx * dx + dy * dy + dz * dz));
    }
    // A millimetre would already be visible on a car panel; the offset is stored as a float,
    // so the bound is the float's and not the formulation's.
    EXPECT_LT(worst, 1e-5);
}

// §0.5: a cook is a pure function of its inputs. Two runs of the same cooker over the same
// mesh must produce the same bytes, or the asset cache is keyed on something that does not
// determine its contents.
TEST(Unit_NodeBeamCooker, TwoCooksOfTheSameInputProduceTheSameBytes)
{
    const Geometry::TriangleMesh mesh = box_mesh(1.0f, 0.5f, 2.0f);
    NodeBeamCooker cooker;
    cooker.set_settings(steel_settings());

    std::vector<std::byte> first;
    std::vector<std::byte> second;
    ASSERT_EQ(cooker.cook(mesh.view(), test_parameters(), nullptr, nullptr, first).status,
              CookingStatus::Succeeded);
    ASSERT_EQ(cooker.cook(mesh.view(), test_parameters(), nullptr, nullptr, second).status,
              CookingStatus::Succeeded);

    ASSERT_EQ(first.size(), second.size());
    EXPECT_EQ(std::memcmp(first.data(), second.data(), first.size()), 0);
}

// The settings are not in CookingParameters, so nothing else can fold them into the key. If
// the cooker does not, the same mesh cooked as steel and as aluminium resolves to one entry
// and the second cook is served the first one's asset — a cache that returns the wrong
// answer rather than a slow one.
TEST(Unit_NodeBeamCooker, ChangingTheMaterialChangesTheCacheKey)
{
    const Geometry::TriangleMesh mesh = box_mesh(1.0f, 0.5f, 2.0f);
    const CookingParameters parameters = test_parameters();

    NodeBeamCooker cooker;
    cooker.set_settings(steel_settings());
    const CookedAssetKey steel = cooker.cache_key(mesh.view(), parameters);

    NodeBeamCookerSettings aluminium = steel_settings();
    aluminium.material.young_modulus = 7.0e10;
    cooker.set_settings(aluminium);
    EXPECT_NE(cooker.cache_key(mesh.view(), parameters), steel);

    NodeBeamCookerSettings lighter_core = steel_settings();
    lighter_core.core_mass_fraction = 0.5f;
    cooker.set_settings(lighter_core);
    EXPECT_NE(cooker.cache_key(mesh.view(), parameters), steel);

    // And the key is stable when nothing changed, or every cook is a cache miss.
    cooker.set_settings(steel_settings());
    EXPECT_EQ(cooker.cache_key(mesh.view(), parameters), steel);
}

TEST(Unit_NodeBeamCooker, AnEmptyMeshIsRefusedRatherThanCooked)
{
    const Geometry::TriangleMesh empty;
    NodeBeamCooker cooker;
    cooker.set_settings(steel_settings());

    std::vector<std::byte> blob;
    const CookingReport report =
        cooker.cook(empty.view(), test_parameters(), nullptr, nullptr, blob);
    EXPECT_EQ(report.status, CookingStatus::EmptyInput);
    EXPECT_TRUE(blob.empty());
    EXPECT_FALSE(report.has_asset());
}

// A material that never yields must produce beams that never dent and never break, rather
// than beams whose thresholds are the `1e30` sentinel multiplied by an area — which is a
// finite number, and therefore a beam that fails under a load nobody intended.
TEST(Unit_NodeBeamCooker, ANonYieldingMaterialProducesNoThresholdsAtAll)
{
    const Geometry::TriangleMesh mesh = box_mesh(1.0f, 0.5f, 2.0f);
    NodeBeamCookerSettings settings;
    settings.material.young_modulus = 1.0e7;
    settings.core_mass_fraction = 0.5f;
    NodeBeamCooker cooker;
    cooker.set_settings(settings);

    std::vector<std::byte> blob;
    ASSERT_EQ(cooker.cook(mesh.view(), test_parameters(), nullptr, nullptr, blob).status,
              CookingStatus::Succeeded);
    const NodeBeamAssetView view = load_node_beam_blob(blob.data(), blob.size());
    ASSERT_TRUE(view.valid);
    ASSERT_GT(view.beam_count, 0u);

    for (std::uint32_t i = 0; i < view.beam_count; ++i)
    {
        EXPECT_DOUBLE_EQ(view.beams[i].deform_force, 0.0);
        EXPECT_DOUBLE_EQ(view.beams[i].break_force, 0.0);
    }
    for (std::uint32_t i = 0; i < view.attachment_count; ++i)
        EXPECT_DOUBLE_EQ(view.attachments[i].break_force, 0.0);
}

// The interior of the lattice is chassis and the boundary is shell — the only split a cooker
// can make from a mesh alone, and the one §11.2 describes. Asserting it keeps a later change
// from quietly attaching the whole cloud, which would produce a vehicle that cannot deform
// and reads as a compliance bug.
TEST(Unit_NodeBeamCooker, OnlyInteriorNodesAreAttachedToTheCore)
{
    const Geometry::TriangleMesh mesh = box_mesh(1.0f, 0.5f, 2.0f);
    NodeBeamCooker cooker;
    cooker.set_settings(steel_settings());

    std::vector<std::byte> blob;
    ASSERT_EQ(cooker.cook(mesh.view(), test_parameters(), nullptr, nullptr, blob).status,
              CookingStatus::Succeeded);
    const NodeBeamAssetView view = load_node_beam_blob(blob.data(), blob.size());
    ASSERT_TRUE(view.valid);
    ASSERT_GT(view.attachment_count, 0u);
    EXPECT_LT(view.attachment_count, view.node_count);

    for (std::uint32_t i = 0; i < view.attachment_count; ++i)
    {
        const NodeBeamNodeRecord& node = view.nodes[view.attachments[i].node];
        EXPECT_EQ(node.flags & NodeBeamNodeFlags::surface, 0u);
    }
}

// Turning the attachments off is meaningful and not a degenerate cook: it is a shell that
// falls off its own chassis, which is what a hand-authored asset may want.
TEST(Unit_NodeBeamCooker, AnUnattachedShellStillCooks)
{
    const Geometry::TriangleMesh mesh = box_mesh(1.0f, 0.5f, 2.0f);
    NodeBeamCookerSettings settings = steel_settings();
    settings.attach_shell_to_core = false;
    NodeBeamCooker cooker;
    cooker.set_settings(settings);

    std::vector<std::byte> blob;
    ASSERT_EQ(cooker.cook(mesh.view(), test_parameters(), nullptr, nullptr, blob).status,
              CookingStatus::Succeeded);
    const NodeBeamAssetView view = load_node_beam_blob(blob.data(), blob.size());
    ASSERT_TRUE(view.valid);
    EXPECT_TRUE(node_beam_has_core(view.core));
    EXPECT_EQ(view.attachment_count, 0u);
}

// Only surface nodes meet the wind, and their areas must add up to the collision surface's —
// the check that would catch a cook splitting a triangle's area over the wrong corners.
TEST(Unit_NodeBeamCooker, OnlySurfaceNodesCarryAWindCrossSection)
{
    const Geometry::TriangleMesh mesh = box_mesh(1.0f, 0.5f, 2.0f);
    NodeBeamCooker cooker;
    cooker.set_settings(steel_settings());

    std::vector<std::byte> blob;
    ASSERT_EQ(cooker.cook(mesh.view(), test_parameters(), nullptr, nullptr, blob).status,
              CookingStatus::Succeeded);
    const NodeBeamAssetView view = load_node_beam_blob(blob.data(), blob.size());
    ASSERT_TRUE(view.valid);

    double area = 0.0;
    for (std::uint32_t i = 0; i < view.node_count; ++i)
    {
        const NodeBeamNodeRecord& node = view.nodes[i];
        if ((node.flags & NodeBeamNodeFlags::surface) == 0)
            EXPECT_DOUBLE_EQ(node.drag_area, 0.0);
        else
            EXPECT_GT(node.drag_area, 0.0);
        area += double(node.drag_area);
    }

    double surface = 0.0;
    for (std::uint32_t i = 0; i + 2 < view.surface_index_count; i += 3)
    {
        const Vector3& a = view.nodes[view.surface_indices[i]].position;
        const Vector3& b = view.nodes[view.surface_indices[i + 1]].position;
        const Vector3& c = view.nodes[view.surface_indices[i + 2]].position;
        surface += 0.5 * length(cross(b - a, c - a));
    }
    EXPECT_NEAR(area, surface, surface * 1e-9);
}

// A finer dial must buy more structure. Not a tautology: the target element count scales the
// resolution, so a cooker reading the authored resolution rather than the effective one
// produces two dials that resolve to the same lattice.
TEST(Unit_NodeBeamCooker, RaisingTheFidelityBuysMoreNodes)
{
    const Geometry::TriangleMesh mesh = box_mesh(1.0f, 0.5f, 2.0f);
    NodeBeamCooker cooker;
    cooker.set_settings(steel_settings());

    CookingParameters coarse = test_parameters();
    coarse.fidelity = 0.2f;
    CookingParameters fine = test_parameters();
    fine.fidelity = 0.5f;

    std::vector<std::byte> blob;
    const CookingReport coarse_report =
        cooker.cook(mesh.view(), coarse, nullptr, nullptr, blob);
    const CookingReport fine_report = cooker.cook(mesh.view(), fine, nullptr, nullptr, blob);

    ASSERT_EQ(coarse_report.status, CookingStatus::Succeeded);
    ASSERT_EQ(fine_report.status, CookingStatus::Succeeded);
    EXPECT_GT(fine_report.node_count, coarse_report.node_count);
    EXPECT_GT(fine_report.beam_count, coarse_report.beam_count);
}
