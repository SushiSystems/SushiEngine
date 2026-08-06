/**************************************************************************/
/* test_prefab_serializer.cpp                                             */
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

// A prefab is an entity subtree saved as a file, and three of its properties have to hold or
// the feature built on it is unusable rather than merely wrong.
//
// The subtree is a *slice*: everything under the root and nothing beside it, with the root
// written as a root even though it has a parent in the scene it came from. A capture that
// walked the whole world would put a street's worth of entities in a street light.
//
// The revision is a content hash, so two captures of unchanged content agree. The refresh pass
// rebuilds an instance whose revision differs from its file's; a revision that varied between
// two identical captures would rebuild every instance in every scene on every open, and one
// that ignored a change would rebuild none of them ever. Both directions are pinned here.
//
// And `prefab_entity_id` is written by this phase and read by none of it. Override resolution
// needs to name what it overrides, an EntityId does not survive a save, and a prefab authored
// without an identifier is unmatchable afterwards — so the field is asserted present and
// unique before anything depends on it.

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <set>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/simulation/simulation.hpp>

#include "prefab_serializer.hpp"

using namespace SushiEngine;
using namespace SushiEngine::Simulation;

namespace
{
    /** @brief Empties the world, demo seeds included, so a test builds from zero. */
    void clear_world(IWorldEditor& world)
    {
        for (const EntityId id : world.entities())
            world.destroy(id);
    }

    /** @brief The first entity carrying @p name, or NULL_ENTITY. */
    EntityId find_by_name(IWorldEditor& world, const std::string& name)
    {
        for (const EntityId id : world.entities())
            if (world.name(id) == name)
                return id;
        return NULL_ENTITY;
    }

    /** @brief A transform whose every field differs from the default. */
    EntityTransform authored_transform(Scalar seed)
    {
        EntityTransform transform;
        transform.position = Vector3{seed, seed + 1, seed + 2};
        transform.rotation = Quaternion{0.5, 0.5, 0.5, 0.5};
        transform.scale = Vector3{seed + 3, seed + 4, seed + 5};
        return transform;
    }

    /**
     * @brief Authors Root -> {Left -> Deep, Right} and returns Root.
     *
     * Two levels below the root and two children at one level: enough that a reader which
     * flattened the tree, or reversed a parent link, would show it.
     */
    EntityId build_subtree(IWorldEditor& world)
    {
        clear_world(world);
        const EntityId root = world.create("Root");
        const EntityId left = world.create("Left");
        const EntityId deep = world.create("Deep");
        const EntityId right = world.create("Right");
        world.set_parent(left, root);
        world.set_parent(deep, left);
        world.set_parent(right, root);
        world.set_transform(root, authored_transform(1));
        world.set_transform(left, authored_transform(2));
        world.set_transform(deep, authored_transform(3));
        world.set_transform(right, authored_transform(4));
        return root;
    }

    void expect_transform_equal(const EntityTransform& actual, const EntityTransform& expected)
    {
        EXPECT_DOUBLE_EQ(actual.position.x, expected.position.x);
        EXPECT_DOUBLE_EQ(actual.position.y, expected.position.y);
        EXPECT_DOUBLE_EQ(actual.position.z, expected.position.z);
        EXPECT_DOUBLE_EQ(actual.rotation.x, expected.rotation.x);
        EXPECT_DOUBLE_EQ(actual.rotation.y, expected.rotation.y);
        EXPECT_DOUBLE_EQ(actual.rotation.z, expected.rotation.z);
        EXPECT_DOUBLE_EQ(actual.rotation.w, expected.rotation.w);
        EXPECT_DOUBLE_EQ(actual.scale.x, expected.scale.x);
        EXPECT_DOUBLE_EQ(actual.scale.y, expected.scale.y);
        EXPECT_DOUBLE_EQ(actual.scale.z, expected.scale.z);
    }
} // namespace

TEST(Unit_PrefabSerializer, ASubtreeRoundTripsWithItsShapeNamesAndTransforms)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();

    const EntityId source = build_subtree(world);
    const nlohmann::json document = Scene::capture_prefab(world, source);

    // Cleared entirely, so what comes back was rebuilt from the document and not merely left
    // in place by a capture that did nothing.
    clear_world(world);
    const EntityId root = Scene::apply_prefab(world, document, NULL_ENTITY);

    ASSERT_NE(root, NULL_ENTITY);
    EXPECT_EQ(world.name(root), "Root");
    EXPECT_EQ(world.parent(root), NULL_ENTITY);
    EXPECT_EQ(world.entities().size(), 4u);

    const EntityId left = find_by_name(world, "Left");
    const EntityId deep = find_by_name(world, "Deep");
    const EntityId right = find_by_name(world, "Right");
    ASSERT_NE(left, NULL_ENTITY);
    ASSERT_NE(deep, NULL_ENTITY);
    ASSERT_NE(right, NULL_ENTITY);

    EXPECT_EQ(world.parent(left), root);
    EXPECT_EQ(world.parent(deep), left);
    EXPECT_EQ(world.parent(right), root);

    expect_transform_equal(world.transform(root), authored_transform(1));
    expect_transform_equal(world.transform(left), authored_transform(2));
    expect_transform_equal(world.transform(deep), authored_transform(3));
    expect_transform_equal(world.transform(right), authored_transform(4));
}

TEST(Unit_PrefabSerializer, TheRootIsEntryZeroAndCarriesNoParent)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();

    const EntityId root = build_subtree(world);
    const nlohmann::json document = Scene::capture_prefab(world, root);

    ASSERT_TRUE(document.contains("entities"));
    const nlohmann::json& entities = document["entities"];
    ASSERT_EQ(entities.size(), 4u);
    EXPECT_EQ(entities.front().value("name", std::string()), "Root");
    // -1 and not an index: the root's own parent lies outside the document, and a capture that
    // wrote an index here would parent the instance to whatever entity that index landed on.
    EXPECT_EQ(entities.front().value("parent", 0), -1);

    // Every other entry's parent chain reaches entry 0.
    for (std::size_t i = 1; i < entities.size(); ++i)
    {
        int walker = entities[i].value("parent", -1);
        std::size_t steps = 0;
        while (walker > 0 && steps < entities.size())
        {
            walker = entities[static_cast<std::size_t>(walker)].value("parent", -1);
            ++steps;
        }
        EXPECT_EQ(walker, 0) << "entry " << i << " does not descend from the root";
    }
}

TEST(Unit_PrefabSerializer, CapturingASubtreeExcludesEverythingOutsideIt)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();

    const EntityId root = build_subtree(world);
    // A sibling of the root and a child of that sibling: neither descends from Root, and a
    // capture that walked the whole world rather than the subtree would carry both.
    const EntityId outsider = world.create("Outsider");
    world.set_parent(world.create("OutsiderChild"), outsider);

    const nlohmann::json document = Scene::capture_prefab(world, root);
    ASSERT_TRUE(document.contains("entities"));
    ASSERT_EQ(document["entities"].size(), 4u);
    for (const auto& entry : document["entities"])
    {
        EXPECT_NE(entry.value("name", std::string()), "Outsider");
        EXPECT_NE(entry.value("name", std::string()), "OutsiderChild");
    }
}

TEST(Unit_PrefabSerializer, EveryEntryHasAPrefabEntityIdAndNoTwoAreEqual)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();

    const EntityId root = build_subtree(world);
    const nlohmann::json document = Scene::capture_prefab(world, root);
    ASSERT_TRUE(document.contains("entities"));

    std::set<std::string> seen;
    for (const auto& entry : document["entities"])
    {
        ASSERT_TRUE(entry.contains("prefab_entity_id"));
        const std::string identifier = entry["prefab_entity_id"].get<std::string>();
        EXPECT_FALSE(identifier.empty());
        EXPECT_TRUE(seen.insert(identifier).second) << "duplicate identifier " << identifier;
    }
    EXPECT_EQ(seen.size(), document["entities"].size());
}

TEST(Unit_PrefabSerializer, IdenticalContentHashesIdentically)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();

    const EntityId root = build_subtree(world);
    const nlohmann::json first = Scene::capture_prefab(world, root);
    const nlohmann::json second = Scene::capture_prefab(world, root);

    // The same subtree captured twice: equal revisions, or the refresh rebuilds every instance
    // on every load. This is also what fails if the identifier is made random or time-based.
    EXPECT_EQ(first.value("revision", std::string("a")),
              second.value("revision", std::string("b")));
    EXPECT_FALSE(first.value("revision", std::string()).empty());
}

TEST(Unit_PrefabSerializer, AChangedTransformChangesTheRevision)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();

    const EntityId root = build_subtree(world);
    const std::string before = Scene::capture_prefab(world, root).value("revision", std::string());

    const EntityId deep = find_by_name(world, "Deep");
    ASSERT_NE(deep, NULL_ENTITY);
    EntityTransform moved = world.transform(deep);
    moved.position.x += 1.0;
    world.set_transform(deep, moved);

    const std::string after = Scene::capture_prefab(world, root).value("revision", std::string());
    EXPECT_NE(before, after);
}

TEST(Unit_PrefabSerializer, AnAddedEntityChangesTheRevision)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();

    const EntityId root = build_subtree(world);
    const std::string before = Scene::capture_prefab(world, root).value("revision", std::string());

    const EntityId right = find_by_name(world, "Right");
    ASSERT_NE(right, NULL_ENTITY);
    world.set_parent(world.create("Added"), right);

    const std::string after = Scene::capture_prefab(world, root).value("revision", std::string());
    EXPECT_NE(before, after);
}

TEST(Unit_PrefabSerializer, AReorderedArrayChangesTheRevision)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();

    const EntityId root = build_subtree(world);
    const nlohmann::json document = Scene::capture_prefab(world, root);
    ASSERT_GE(document["entities"].size(), 3u);

    // Order is one of the things a revision has to notice. It holds trivially while the
    // revision hashes the serialized array, and stops holding the day someone "improves" it
    // into a hash over a set of per-entity hashes — which is what this case is here to fail.
    nlohmann::json reordered = document["entities"];
    std::swap(reordered[1], reordered[2]);
    EXPECT_NE(Scene::prefab_revision(document["entities"]), Scene::prefab_revision(reordered));
}

TEST(Unit_PrefabSerializer, CapturingANonLiveEntityYieldsAnEmptyDocument)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();
    clear_world(world);

    const nlohmann::json document = Scene::capture_prefab(world, NULL_ENTITY);
    ASSERT_TRUE(document.contains("entities"));
    EXPECT_TRUE(document["entities"].is_array());
    EXPECT_TRUE(document["entities"].empty());
}

TEST(Unit_PrefabSerializer, CapturingADestroyedEntityYieldsAnEmptyDocument)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();

    const EntityId root = build_subtree(world);
    clear_world(world);

    // A destroyed id is not NULL_ENTITY, so a guard that only checked for NULL_ENTITY would
    // walk it and write a record of whatever the world answers for a dead entity.
    const nlohmann::json document = Scene::capture_prefab(world, root);
    ASSERT_TRUE(document.contains("entities"));
    EXPECT_TRUE(document["entities"].empty());
}

TEST(Unit_PrefabSerializer, ApplyingAnEmptyDocumentCreatesNothingAndReturnsNullEntity)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();
    clear_world(world);
    const EntityId survivor = world.create("Survivor");

    const nlohmann::json empty =
        nlohmann::json{{"revision", ""}, {"entities", nlohmann::json::array()}};
    EXPECT_EQ(Scene::apply_prefab(world, empty, NULL_ENTITY), NULL_ENTITY);

    // apply_prefab is not apply_scene: it adds a subtree and clears nothing.
    EXPECT_EQ(world.entities().size(), 1u);
    EXPECT_EQ(world.name(survivor), "Survivor");
}

TEST(Unit_PrefabSerializer, AnAppliedSubtreeHangsUnderTheGivenParent)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();

    const EntityId source = build_subtree(world);
    const nlohmann::json document = Scene::capture_prefab(world, source);

    clear_world(world);
    const EntityId host = world.create("Host");
    const EntityId root = Scene::apply_prefab(world, document, host);

    ASSERT_NE(root, NULL_ENTITY);
    EXPECT_EQ(world.parent(root), host);
    EXPECT_EQ(world.entities().size(), 5u);
}

TEST(Unit_PrefabSerializer, ReadingTheRevisionOfAMissingFileFails)
{
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "sushiengine_prefab_absent.sushiprefab";
    std::error_code error;
    std::filesystem::remove(path, error);

    std::string revision = "stale";
    EXPECT_FALSE(Scene::read_prefab_revision(path.string(), revision));
    // Cleared on failure, so a caller that ignores the return does not compare against the
    // revision it happened to be holding and conclude the instance is current.
    EXPECT_TRUE(revision.empty());
}

TEST(Unit_PrefabSerializer, ReadingTheRevisionOfAMalformedFileFails)
{
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "sushiengine_prefab_malformed.sushiprefab";
    {
        std::ofstream file(path.string());
        ASSERT_TRUE(static_cast<bool>(file));
        file << "{ this is not json";
    }

    std::string revision = "stale";
    EXPECT_FALSE(Scene::read_prefab_revision(path.string(), revision));
    EXPECT_TRUE(revision.empty());

    std::error_code error;
    std::filesystem::remove(path, error);
}

TEST(Unit_PrefabSerializer, TheRevisionOnDiskIsTheOneCaptureWrote)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();

    const EntityId root = build_subtree(world);
    const nlohmann::json document = Scene::capture_prefab(world, root);

    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "sushiengine_prefab_revision.sushiprefab";
    {
        std::ofstream file(path.string());
        ASSERT_TRUE(static_cast<bool>(file));
        file << document.dump(2);
    }

    std::string revision;
    ASSERT_TRUE(Scene::read_prefab_revision(path.string(), revision));
    EXPECT_EQ(revision, document.value("revision", std::string()));
    // The cheap read and the full computation agree, which is the premise of the refresh pass
    // asking the cheap one for every instance and parsing only the stale ones.
    EXPECT_EQ(revision, Scene::prefab_revision(document["entities"]));

    std::error_code error;
    std::filesystem::remove(path, error);
}
