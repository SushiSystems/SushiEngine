/**************************************************************************/
/* test_import_chain.cpp                                                  */
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

// §8.1's chain, which is the half of P4's acceptance criterion that says "without a manual
// step". The tests here are about *who decides* and *what happens when one part fails* —
// the cooking itself is covered by the two cooker suites, and re-asserting it here would
// only make this file fail twice for one bug.

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/geometry/triangle_mesh.hpp>
#include <SushiEngine/physics/cooking/collision_asset.hpp>
#include <SushiEngine/physics/cooking/cooked_asset_store.hpp>
#include <SushiEngine/physics/cooking/cooking_service.hpp>
#include <SushiEngine/physics/cooking/import_profile.hpp>
#include <SushiEngine/physics/cooking/mesh_post_processor.hpp>
#include <SushiEngine/physics/cooking/soft_body_asset.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics::Cooking;

namespace
{
    /** @brief An outward-wound box. */
    Geometry::TriangleMesh box_mesh(float h)
    {
        Geometry::TriangleMesh mesh;
        const float c[8][3] = {{-h, -h, -h}, {h, -h, -h}, {h, h, -h}, {-h, h, -h},
                               {-h, -h, h},  {h, -h, h},  {h, h, h},  {-h, h, h}};
        for (const auto& corner : c)
        {
            mesh.positions.push_back(corner[0]);
            mesh.positions.push_back(corner[1]);
            mesh.positions.push_back(corner[2]);
        }
        const std::uint32_t f[12][3] = {{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7},
                                        {0, 1, 5}, {0, 5, 4}, {3, 7, 6}, {3, 6, 2},
                                        {0, 4, 7}, {0, 7, 3}, {1, 2, 6}, {1, 6, 5}};
        for (const auto& face : f)
        {
            mesh.indices.push_back(face[0]);
            mesh.indices.push_back(face[1]);
            mesh.indices.push_back(face[2]);
        }
        return mesh;
    }

    /**
     * @brief A loader that answers from a table rather than from a disk.
     *
     * The reason the loader is a seam at all: the chain is testable against geometry built
     * in memory, with no file, no cgltf and no renderer.
     */
    MeshLoader table_loader(std::atomic<int>* calls = nullptr)
    {
        return [calls](const std::string& path, Geometry::TriangleMesh& out) -> bool
        {
            if (calls != nullptr)
                calls->fetch_add(1);
            if (path == "crate.gltf")
            {
                out = box_mesh(0.5f);
                return true;
            }
            if (path == "pillar.gltf")
            {
                out = box_mesh(0.25f);
                return true;
            }
            return false;   // anything else is a file that is not there
        };
    }

    /** @brief A profile that cooks quickly; a unit test is not a bake farm. */
    ImportProfile quick_profile()
    {
        ImportProfile profile;
        profile.parameters.fidelity = 0.0f;
        profile.parameters.voxel_resolution = 5;
        profile.parameters.distance_field_resolution = 8;
        profile.parameters.simulation_level_count = 1;
        profile.parameters.surface_conforming_passes = 0;
        profile.parameters.accuracy_lattice_order = 2;
        return profile;
    }

    /** @brief A processor that records that it ran, for the ordering tests. */
    class MarkerProcessor final : public IMeshPostProcessor
    {
    public:
        MarkerProcessor(const char* name, int order, std::vector<std::string>* log,
                        bool wanted = true)
            : name_(name), order_(order), log_(log), wanted_(wanted)
        {
        }

        const char* name() const noexcept override { return name_; }
        int order() const noexcept override { return order_; }
        bool wants(const ImportProfile&) const noexcept override { return wanted_; }

        bool process(const Geometry::TriangleMeshView&, const ImportProfile&,
                     ICookedAssetStore*, ICookingProgressSink*,
                     MeshPostProcessResult& out) override
        {
            log_->push_back(name_);
            out.processor = name_;
            out.report.status = CookingStatus::Succeeded;
            out.bytes.assign(4, std::byte{1});
            return true;
        }

    private:
        const char* name_;
        int order_;
        std::vector<std::string>* log_;
        bool wanted_;
    };

    /** @brief A processor that always fails, to prove it does not stop the chain. */
    class FailingProcessor final : public IMeshPostProcessor
    {
    public:
        FailingProcessor(int order, std::vector<std::string>* log) : order_(order), log_(log) {}

        const char* name() const noexcept override { return "Failing"; }
        int order() const noexcept override { return order_; }
        bool wants(const ImportProfile&) const noexcept override { return true; }

        bool process(const Geometry::TriangleMeshView&, const ImportProfile&,
                     ICookedAssetStore*, ICookingProgressSink*,
                     MeshPostProcessResult& out) override
        {
            log_->push_back("Failing");
            out.report.status = CookingStatus::StageFailed;
            return false;
        }

    private:
        int order_;
        std::vector<std::string>* log_;
    };
} // namespace

TEST(Unit_ImportProfile, FoldsAPerAssetOverrideOverTheProjectDefault)
{
    ImportProfileLibrary library;
    ImportProfile project;
    project.parameters.fidelity = 0.3f;
    project.parameters.cook_collision = true;
    project.parameters.cook_soft_body = false;
    library.set_project_default(project);

    // The whole point of §8.1's split: an asset with nothing said about it is cooked at the
    // project's settings, with no action from anyone.
    const ImportProfile untouched = library.resolve("rock.gltf");
    EXPECT_FLOAT_EQ(untouched.parameters.fidelity, 0.3f);
    EXPECT_TRUE(untouched.parameters.cook_collision);
    EXPECT_FALSE(untouched.parameters.cook_soft_body);

    // And the one crate in a hundred that wants to be deformable says so in one field,
    // without a project-wide setting changing and without every rock paying for a
    // tetrahedral mesh.
    ImportProfileOverride crate;
    crate.cook_soft_body = true;
    crate.fidelity = 0.8f;
    library.set_override("crate.gltf", crate);

    const ImportProfile overridden = library.resolve("crate.gltf");
    EXPECT_FLOAT_EQ(overridden.parameters.fidelity, 0.8f);
    EXPECT_TRUE(overridden.parameters.cook_soft_body);
    // Untouched fields still come from the project, so a changed default reaches this asset.
    EXPECT_TRUE(overridden.parameters.cook_collision);
    EXPECT_FLOAT_EQ(library.resolve("rock.gltf").parameters.fidelity, 0.3f);

    // An empty override removes the entry rather than storing a no-op, so "reset to project
    // default" and "was never set" are one state — otherwise a project accumulates entries
    // that do nothing and a changed default silently fails to reach them.
    EXPECT_EQ(library.override_count(), 1u);
    library.set_override("crate.gltf", ImportProfileOverride{});
    EXPECT_FALSE(library.has_override("crate.gltf"));
    EXPECT_EQ(library.override_count(), 0u);
    EXPECT_FLOAT_EQ(library.resolve("crate.gltf").parameters.fidelity, 0.3f);
}

TEST(Unit_MeshPostProcessorChain, RunsInOrderRegardlessOfRegistrationOrder)
{
    std::vector<std::string> log;
    MeshPostProcessorChain chain;
    // Registered back to front on purpose: the sequence must be a property of the
    // processors, not of the order somebody happened to add them in.
    chain.add(std::make_unique<MarkerProcessor>("third", 300, &log));
    chain.add(std::make_unique<MarkerProcessor>("first", 100, &log));
    chain.add(std::make_unique<MarkerProcessor>("second", 200, &log));

    ASSERT_EQ(chain.size(), 3u);
    EXPECT_STREQ(chain.at(0).name(), "first");
    EXPECT_STREQ(chain.at(1).name(), "second");
    EXPECT_STREQ(chain.at(2).name(), "third");

    const Geometry::TriangleMesh mesh = box_mesh(0.5f);
    const std::vector<MeshPostProcessResult> results =
        chain.run(mesh.view(), quick_profile(), nullptr, nullptr);
    ASSERT_EQ(results.size(), 3u);
    EXPECT_EQ(log, std::vector<std::string>({"first", "second", "third"}));

    // A tie keeps insertion order, so the chain is stable between builds.
    std::vector<std::string> tied_log;
    MeshPostProcessorChain tied;
    tied.add(std::make_unique<MarkerProcessor>("a", 100, &tied_log));
    tied.add(std::make_unique<MarkerProcessor>("b", 100, &tied_log));
    tied.run(mesh.view(), quick_profile(), nullptr, nullptr);
    EXPECT_EQ(tied_log, std::vector<std::string>({"a", "b"}));

    // A null registration is ignored rather than stored.
    tied.add(nullptr);
    EXPECT_EQ(tied.size(), 2u);
}

TEST(Unit_MeshPostProcessorChain, SkipsWhatTheProfileDoesNotWantAndSurvivesAFailure)
{
    std::vector<std::string> log;
    MeshPostProcessorChain chain;
    chain.add(std::make_unique<MarkerProcessor>("wanted", 100, &log, true));
    chain.add(std::make_unique<FailingProcessor>(200, &log));
    chain.add(std::make_unique<MarkerProcessor>("unwanted", 250, &log, false));
    chain.add(std::make_unique<MarkerProcessor>("after", 300, &log, true));

    const Geometry::TriangleMesh mesh = box_mesh(0.5f);
    const std::vector<MeshPostProcessResult> results =
        chain.run(mesh.view(), quick_profile(), nullptr, nullptr);

    // The failing processor ran and contributed nothing; the one after it still ran. A mesh
    // whose soft-body cook fails must still get its collider, or one bad cook silently costs
    // the asset its collision too.
    EXPECT_EQ(log, std::vector<std::string>({"wanted", "Failing", "after"}));
    ASSERT_EQ(results.size(), 2u);
    EXPECT_EQ(results[0].processor, "wanted");
    EXPECT_EQ(results[1].processor, "after");
}

TEST(Unit_MeshPostProcessorChain, TheShippedChainAsksTheProfileWhichCookersRun)
{
    const MeshPostProcessorChain chain = MeshPostProcessorChain::with_shipped_processors();
    ASSERT_EQ(chain.size(), 2u);
    EXPECT_STREQ(chain.at(0).name(), "CollisionPostProcessor");
    EXPECT_STREQ(chain.at(1).name(), "SoftBodyPostProcessor");
    // The cheap one first: a collider is milliseconds and wanted by almost everything, a
    // tetrahedral mesh is minutes and wanted by few assets.
    EXPECT_LT(chain.at(0).order(), chain.at(1).order());

    ImportProfile collider_only = quick_profile();
    collider_only.parameters.cook_collision = true;
    collider_only.parameters.cook_soft_body = false;
    EXPECT_TRUE(chain.at(0).wants(collider_only));
    EXPECT_FALSE(chain.at(1).wants(collider_only));

    ImportProfile both = collider_only;
    both.parameters.cook_soft_body = true;
    EXPECT_TRUE(chain.at(0).wants(both));
    EXPECT_TRUE(chain.at(1).wants(both));

    ImportProfile neither = collider_only;
    neither.parameters.cook_collision = false;
    const Geometry::TriangleMesh mesh = box_mesh(0.5f);
    EXPECT_TRUE(chain.run(mesh.view(), neither, nullptr, nullptr).empty());
}

TEST(Unit_MeshPostProcessorChain, ProducesBothAssetKindsForAMeshThatAsksForBoth)
{
    const MeshPostProcessorChain chain = MeshPostProcessorChain::with_shipped_processors();
    MemoryCookedAssetStore store;

    ImportProfile profile = quick_profile();
    profile.parameters.cook_collision = true;
    profile.parameters.cook_soft_body = true;

    const Geometry::TriangleMesh mesh = box_mesh(0.5f);
    const std::vector<MeshPostProcessResult> results =
        chain.run(mesh.view(), profile, &store, nullptr);

    ASSERT_EQ(results.size(), 2u);
    EXPECT_EQ(results[0].kind, CookedAssetKind::Collision);
    EXPECT_EQ(results[1].kind, CookedAssetKind::SoftBody);

    // Both products are loadable, which is P4's acceptance criterion for one mesh.
    ASSERT_TRUE(load_collision_blob(results[0].bytes.data(), results[0].bytes.size()).valid);
    ASSERT_TRUE(load_soft_body_blob(results[1].bytes.data(), results[1].bytes.size()).valid);

    // And both are in the cache, under different keys, so neither cook repeats and neither
    // serves the other's entry.
    EXPECT_EQ(store.size(), 2u);
}

TEST(Unit_CookingService, CooksOffTheCallingThreadAndHandsBackEachImportOnce)
{
    const MeshPostProcessorChain chain = MeshPostProcessorChain::with_shipped_processors();
    MemoryCookedAssetStore store;
    std::atomic<int> loads{0};

    ImportProfile profile = quick_profile();
    profile.parameters.cook_soft_body = true;

    {
        CookingService service(table_loader(&loads), chain, &store);
        service.submit("crate.gltf", profile);
        service.submit("pillar.gltf", profile);
        service.wait_until_idle();

        const std::vector<CookedImport> imports = service.take_completed();
        ASSERT_EQ(imports.size(), 2u);
        // One worker, so results arrive in submission order — which is what keeps an import
        // log readable and this assertion meaningful at all.
        EXPECT_EQ(imports[0].asset_path, "crate.gltf");
        EXPECT_EQ(imports[1].asset_path, "pillar.gltf");
        EXPECT_EQ(loads.load(), 2);

        for (const CookedImport& imported : imports)
        {
            EXPECT_TRUE(imported.loaded);
            EXPECT_EQ(imported.source_triangle_count, 12u);
            ASSERT_NE(imported.product(CookedAssetKind::Collision), nullptr);
            ASSERT_NE(imported.product(CookedAssetKind::SoftBody), nullptr);
            EXPECT_TRUE(imported.product(CookedAssetKind::Collision)->report.has_asset());
        }

        // Taken once: a caller polling every frame must not see an import twice.
        EXPECT_TRUE(service.take_completed().empty());

        const CookingServiceStatus idle = service.status();
        EXPECT_FALSE(idle.busy);
        EXPECT_EQ(idle.queued, 0u);
        EXPECT_TRUE(idle.asset_path.empty());
    }
}

TEST(Unit_CookingService, ReportsAFileItCouldNotReadRatherThanDroppingIt)
{
    const MeshPostProcessorChain chain = MeshPostProcessorChain::with_shipped_processors();
    CookingService service(table_loader(), chain, nullptr);

    service.submit("missing.gltf", quick_profile());
    service.wait_until_idle();

    const std::vector<CookedImport> imports = service.take_completed();
    // An import that produced nothing still comes back. Dropping it silently is how an
    // artist ends up staring at a crate that is not solid with nothing anywhere saying why.
    ASSERT_EQ(imports.size(), 1u);
    EXPECT_EQ(imports[0].asset_path, "missing.gltf");
    EXPECT_FALSE(imports[0].loaded);
    EXPECT_TRUE(imports[0].products.empty());
    EXPECT_EQ(imports[0].source_triangle_count, 0u);
}

TEST(Unit_CookingService, DestroysCleanlyWithWorkStillQueued)
{
    // Closing the editor must not take as long as the largest cook still pending. Nothing is
    // lost by abandoning the queue: the cache means the next session cooks only what it must.
    const MeshPostProcessorChain chain = MeshPostProcessorChain::with_shipped_processors();
    MemoryCookedAssetStore store;
    ImportProfile profile = quick_profile();
    profile.parameters.cook_soft_body = true;

    {
        CookingService service(table_loader(), chain, &store);
        for (int i = 0; i < 32; ++i)
            service.submit("crate.gltf", profile);
        // Destructor runs here with work almost certainly outstanding.
    }
    SUCCEED();
}

TEST(Unit_CookingService, SecondImportOfAnUnchangedMeshIsServedFromTheCache)
{
    const MeshPostProcessorChain chain = MeshPostProcessorChain::with_shipped_processors();
    MemoryCookedAssetStore store;
    CookingService service(table_loader(), chain, &store);

    service.submit("crate.gltf", quick_profile());
    service.wait_until_idle();
    const std::vector<CookedImport> first = service.take_completed();
    ASSERT_EQ(first.size(), 1u);
    ASSERT_NE(first[0].product(CookedAssetKind::Collision), nullptr);
    EXPECT_FALSE(first[0].product(CookedAssetKind::Collision)->report.served_from_cache);

    service.submit("crate.gltf", quick_profile());
    service.wait_until_idle();
    const std::vector<CookedImport> second = service.take_completed();
    ASSERT_EQ(second.size(), 1u);
    ASSERT_NE(second[0].product(CookedAssetKind::Collision), nullptr);
    // §8.1's promise, through the whole chain rather than at the cooker: an unchanged mesh
    // with unchanged parameters is never re-cooked.
    EXPECT_TRUE(second[0].product(CookedAssetKind::Collision)->report.served_from_cache);
    EXPECT_EQ(second[0].product(CookedAssetKind::Collision)->bytes,
              first[0].product(CookedAssetKind::Collision)->bytes);
}
