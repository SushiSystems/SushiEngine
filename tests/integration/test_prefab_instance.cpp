/**************************************************************************/
/* test_prefab_instance.cpp                                               */
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

// Refreshing a placed prefab when its file has moved on, and the four ways that can go wrong.
//
// The rebuild has to take the *content* from the file and the *placement* from the scene. A
// rebuild that took everything from the file would move every placed street light back to the
// origin and rename it; one that took too much from the scene would refresh a prefab into
// whatever it used to be.
//
// It has to not run when the revision matches. That case is invisible unless something inside
// the subtree is watched across the call, because an unconditional rebuild produces the same
// entity count -- so a marker is watched, not a count.
//
// It has to leave a missing prefab's entities alone. Nothing the user placed disappears
// because a file was not pulled yet, and the link survives so restoring the file restores it.
//
// And it must not run in apply_scene. That function is the path undo restores through, so
// refreshing there would reinstate the very prefab edit the user is undoing. That case is the
// reason this file exists rather than the cases above being folded into the serializer's.

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/simulation/simulation.hpp>

#include "prefab_serializer.hpp"
#include "scene_serializer.hpp"

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

    /** @brief Authors Root -> {Left -> Deep, Right} and returns Root. */
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
        return root;
    }

    /** @brief Writes @p document to @p path, and returns whether it landed. */
    bool write_prefab(const nlohmann::json& document, const std::filesystem::path& path)
    {
        std::ofstream file(path.string());
        if (!file)
            return false;
        file << document.dump(2);
        return static_cast<bool>(file);
    }

    /** @brief Links @p root to @p path at @p revision. */
    void link_instance(IWorldEditor& world, EntityId root, const std::filesystem::path& path,
                       const std::string& revision)
    {
        PrefabInstanceParameters link;
        link.path = path.string();
        link.revision = revision;
        world.set_prefab_instance(root, link);
    }

    /** @brief A path under the temporary directory, removed first so a rerun starts clean. */
    std::filesystem::path scratch(const char* name)
    {
        const std::filesystem::path path = std::filesystem::temp_directory_path() / name;
        std::error_code error;
        std::filesystem::remove(path, error);
        return path;
    }
} // namespace

TEST(Integration_PrefabInstance, AStaleInstanceIsRebuiltAndTheRootKeepsItsNameAndTransform)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();

    const std::filesystem::path path = scratch("sushiengine_prefab_stale.sushiprefab");

    // Author the prefab, then change it, so the file on disk is ahead of the instance.
    const EntityId source = build_subtree(world);
    const nlohmann::json first = Scene::capture_prefab(world, source);
    world.set_parent(world.create("AddedLater"), find_by_name(world, "Left"));
    const nlohmann::json second = Scene::capture_prefab(world, source);
    ASSERT_NE(first.value("revision", std::string("a")),
              second.value("revision", std::string("b")));
    ASSERT_TRUE(write_prefab(second, path));

    // Place an instance built from the *first* revision, and give its root a placement of its
    // own that the rebuild must not overwrite.
    clear_world(world);
    const EntityId instance = Scene::apply_prefab(world, first, NULL_ENTITY);
    ASSERT_NE(instance, NULL_ENTITY);
    world.set_name(instance, "LampPost_03");
    EntityTransform placement;
    placement.position = Vector3{100.0, 200.0, 300.0};
    placement.scale = Vector3{2.0, 2.0, 2.0};
    world.set_transform(instance, placement);
    link_instance(world, instance, path, first.value("revision", std::string()));

    EXPECT_TRUE(Scene::refresh_prefab_instances(world).empty());

    // The subtree is the file's, not the snapshot's.
    EXPECT_NE(find_by_name(world, "AddedLater"), NULL_ENTITY);
    EXPECT_EQ(world.entities().size(), 5u);

    // The placement is the scene's, not the file's. Looked up by name rather than by the id
    // captured above, because the rebuild replaces the root entity: the prefab's own root may
    // carry components, and re-dressing the old entity instead of replacing it would drop them.
    const EntityId rebuilt = find_by_name(world, "LampPost_03");
    ASSERT_NE(rebuilt, NULL_ENTITY);
    EXPECT_DOUBLE_EQ(world.transform(rebuilt).position.x, 100.0);
    EXPECT_DOUBLE_EQ(world.transform(rebuilt).position.y, 200.0);
    EXPECT_DOUBLE_EQ(world.transform(rebuilt).scale.x, 2.0);
    // And the instance is no longer stale, or the next load rebuilds it all over again.
    EXPECT_EQ(world.prefab_instance(rebuilt).revision, second.value("revision", std::string()));
    EXPECT_EQ(world.prefab_instance(rebuilt).path, path.string());

    std::error_code error;
    std::filesystem::remove(path, error);
}

TEST(Integration_PrefabInstance, ARebuiltRootKeepsThePrefabsOwnComponents)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();

    const std::filesystem::path path = scratch("sushiengine_prefab_single.sushiprefab");

    // The commonest prefab there is: one object, no children. Its root *is* its content, so a
    // rebuild that treated the root as a placeholder to be re-dressed would refresh this into
    // an empty entity — the whole prefab silently gone.
    clear_world(world);
    const EntityId box = world.create_box("Crate");
    ASSERT_NE(box, NULL_ENTITY);
    ShapeParameters authored = world.shape_parameters(box);
    authored.mesh_path = "models/crate_v1.gltf";
    world.set_shape_parameters(box, authored);
    const nlohmann::json first = Scene::capture_prefab(world, box);
    ASSERT_EQ(first["entities"].size(), 1u);

    authored.mesh_path = "models/crate_v2.gltf";
    world.set_shape_parameters(box, authored);
    const nlohmann::json second = Scene::capture_prefab(world, box);
    ASSERT_TRUE(write_prefab(second, path));

    clear_world(world);
    const EntityId instance = Scene::apply_prefab(world, first, NULL_ENTITY);
    ASSERT_NE(instance, NULL_ENTITY);
    link_instance(world, instance, path, first.value("revision", std::string()));

    EXPECT_TRUE(Scene::refresh_prefab_instances(world).empty());

    const EntityId rebuilt = find_by_name(world, "Crate");
    ASSERT_NE(rebuilt, NULL_ENTITY);
    EXPECT_EQ(world.entities().size(), 1u);
    ASSERT_TRUE(world.has_shape(rebuilt));
    EXPECT_EQ(world.shape_parameters(rebuilt).mesh_path, "models/crate_v2.gltf");

    std::error_code error;
    std::filesystem::remove(path, error);
}

TEST(Integration_PrefabInstance, AnInstanceAtTheCurrentRevisionIsLeftAlone)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();

    const std::filesystem::path path = scratch("sushiengine_prefab_current.sushiprefab");

    const EntityId source = build_subtree(world);
    const nlohmann::json document = Scene::capture_prefab(world, source);
    ASSERT_TRUE(write_prefab(document, path));

    clear_world(world);
    const EntityId instance = Scene::apply_prefab(world, document, NULL_ENTITY);
    ASSERT_NE(instance, NULL_ENTITY);
    link_instance(world, instance, path, document.value("revision", std::string()));

    // A marker inside the subtree, so "left alone" is observable rather than inferred from the
    // entity count — an unconditional rebuild produces the same count and destroys this.
    const EntityId marker = find_by_name(world, "Deep");
    ASSERT_NE(marker, NULL_ENTITY);

    EXPECT_TRUE(Scene::refresh_prefab_instances(world).empty());

    EXPECT_EQ(find_by_name(world, "Deep"), marker) << "the subtree was rebuilt when it was current";
    EXPECT_EQ(instance, find_by_name(world, "Root")) << "the root was replaced needlessly";
    EXPECT_EQ(world.entities().size(), 4u);

    std::error_code error;
    std::filesystem::remove(path, error);
}

TEST(Integration_PrefabInstance, AMissingPrefabLeavesTheEntitiesInPlaceAndDoesNotCrash)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();

    const std::filesystem::path path = scratch("sushiengine_prefab_missing.sushiprefab");

    const EntityId source = build_subtree(world);
    const nlohmann::json document = Scene::capture_prefab(world, source);
    clear_world(world);
    const EntityId instance = Scene::apply_prefab(world, document, NULL_ENTITY);
    ASSERT_NE(instance, NULL_ENTITY);
    link_instance(world, instance, path, "whatever-it-was");

    const std::vector<std::string> unreadable = Scene::refresh_prefab_instances(world);
    ASSERT_EQ(unreadable.size(), 1u);
    EXPECT_EQ(unreadable.front(), path.string());

    // A missing prefab unlinks a subtree in the editor's display; it does not delete one, and
    // it does not clear the component either. A file that is merely not pulled yet must not
    // destroy the association permanently — restoring the file has to restore the link.
    EXPECT_EQ(world.entities().size(), 4u);
    EXPECT_NE(find_by_name(world, "Deep"), NULL_ENTITY);
    EXPECT_TRUE(world.has_prefab_instance(instance));
    EXPECT_EQ(world.prefab_instance(instance).revision, "whatever-it-was");
}

TEST(Integration_PrefabInstance, ApplySceneDoesNotRefresh)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();

    const std::filesystem::path path = scratch("sushiengine_prefab_undo.sushiprefab");

    const EntityId source = build_subtree(world);
    const nlohmann::json first = Scene::capture_prefab(world, source);
    world.set_parent(world.create("AddedLater"), find_by_name(world, "Left"));
    ASSERT_TRUE(write_prefab(Scene::capture_prefab(world, source), path));

    clear_world(world);
    const EntityId instance = Scene::apply_prefab(world, first, NULL_ENTITY);
    ASSERT_NE(instance, NULL_ENTITY);
    link_instance(world, instance, path, first.value("revision", std::string()));

    const nlohmann::json snapshot = Scene::capture_scene(world);
    world.destroy(find_by_name(world, "Right"));
    Scene::apply_scene(world, snapshot);

    // apply_scene is the path undo restores through: refreshing there would reinstate the
    // prefab edit the user is undoing, so the instance must still be stale afterwards and the
    // file's later entity must still be absent.
    const EntityId restored = find_by_name(world, "Root");
    ASSERT_NE(restored, NULL_ENTITY);
    EXPECT_EQ(world.prefab_instance(restored).revision, first.value("revision", std::string()));
    EXPECT_EQ(find_by_name(world, "AddedLater"), NULL_ENTITY);
    EXPECT_EQ(world.entities().size(), 4u);

    std::error_code error;
    std::filesystem::remove(path, error);
}

TEST(Integration_PrefabInstance, LoadSceneRefreshes)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();

    const std::filesystem::path prefab_path = scratch("sushiengine_prefab_on_load.sushiprefab");
    const std::filesystem::path scene_path = scratch("sushiengine_prefab_on_load.sushiscene");

    // A scene saved against revision one and a prefab that has since moved on: the situation a
    // user meets when a teammate edits a prefab they have placed.
    const EntityId source = build_subtree(world);
    const nlohmann::json first = Scene::capture_prefab(world, source);
    clear_world(world);
    const EntityId instance = Scene::apply_prefab(world, first, NULL_ENTITY);
    ASSERT_NE(instance, NULL_ENTITY);
    link_instance(world, instance, prefab_path, first.value("revision", std::string()));
    ASSERT_TRUE(Scene::save_scene(world, scene_path.string()));

    const EntityId rebuilt_source = build_subtree(world);
    world.set_parent(world.create("AddedLater"), find_by_name(world, "Left"));
    const nlohmann::json second = Scene::capture_prefab(world, rebuilt_source);
    ASSERT_TRUE(write_prefab(second, prefab_path));

    clear_world(world);
    ASSERT_TRUE(Scene::load_scene(world, scene_path.string()));

    EXPECT_NE(find_by_name(world, "AddedLater"), NULL_ENTITY)
        << "load_scene did not refresh the instance";
    const EntityId loaded = find_by_name(world, "Root");
    ASSERT_NE(loaded, NULL_ENTITY);
    EXPECT_EQ(world.prefab_instance(loaded).revision, second.value("revision", std::string()));

    std::error_code error;
    std::filesystem::remove(prefab_path, error);
    std::filesystem::remove(scene_path, error);
    std::filesystem::remove(std::filesystem::path(scene_path.string() + ".atmos"), error);
}

// Override resolution (P2, docs/design/prefab_system.md §10). What these cover is the
// decision that no override is ever recorded: nothing here calls a "mark as overridden"
// API, because there is not one. An edit is an edit, and the refresh finds it by
// comparing the live member against the prefab's record for the same identity.

TEST(Integration_PrefabInstance, AnEditedMemberKeepsItsEditThroughARefresh)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();
    const std::filesystem::path path = scratch("sushiengine_prefab_override.sushiprefab");

    const EntityId source = build_subtree(world);
    const nlohmann::json first = Scene::capture_prefab(world, source);
    world.set_parent(world.create("AddedLater"), find_by_name(world, "Left"));
    const nlohmann::json second = Scene::capture_prefab(world, source);
    ASSERT_TRUE(write_prefab(second, path));

    clear_world(world);
    const EntityId instance = Scene::apply_prefab(world, first, NULL_ENTITY);
    ASSERT_NE(instance, NULL_ENTITY);
    link_instance(world, instance, path, first.value("revision", std::string()));

    // The local edit: a member moved somewhere the prefab never put it.
    const EntityId edited = find_by_name(world, "Right");
    ASSERT_NE(edited, NULL_ENTITY);
    EntityTransform moved = world.transform(edited);
    moved.position = Vector3{7.0, 8.0, 9.0};
    world.set_transform(edited, moved);

    EXPECT_TRUE(Scene::refresh_prefab_instances(world).empty());

    // The prefab's change arrived and the author's survived it.
    EXPECT_NE(find_by_name(world, "AddedLater"), NULL_ENTITY)
        << "the prefab's own change did not reach the instance";
    const EntityId kept = find_by_name(world, "Right");
    ASSERT_NE(kept, NULL_ENTITY);
    EXPECT_DOUBLE_EQ(world.transform(kept).position.x, 7.0)
        << "the refresh discarded the local edit, which is what P2 exists to stop";
    EXPECT_DOUBLE_EQ(world.transform(kept).position.z, 9.0);

    std::error_code error;
    std::filesystem::remove(path, error);
}

TEST(Integration_PrefabInstance, AnUneditedMemberTakesThePrefabsNewValue)
{
    // The other half, and the one a naive keep-everything implementation fails: a
    // component the author never touched must follow the prefab.
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();
    const std::filesystem::path path = scratch("sushiengine_prefab_follows.sushiprefab");

    const EntityId source = build_subtree(world);
    const nlohmann::json first = Scene::capture_prefab(world, source);

    // The prefab moves one of its own members.
    const EntityId moved_in_prefab = find_by_name(world, "Right");
    ASSERT_NE(moved_in_prefab, NULL_ENTITY);
    EntityTransform authored = world.transform(moved_in_prefab);
    authored.position = Vector3{-4.0, 0.0, 0.0};
    world.set_transform(moved_in_prefab, authored);
    const nlohmann::json second = Scene::capture_prefab(world, source);
    ASSERT_TRUE(write_prefab(second, path));

    clear_world(world);
    const EntityId instance = Scene::apply_prefab(world, first, NULL_ENTITY);
    ASSERT_NE(instance, NULL_ENTITY);
    link_instance(world, instance, path, first.value("revision", std::string()));

    EXPECT_TRUE(Scene::refresh_prefab_instances(world).empty());

    const EntityId followed = find_by_name(world, "Right");
    ASSERT_NE(followed, NULL_ENTITY);
    EXPECT_DOUBLE_EQ(world.transform(followed).position.x, -4.0)
        << "an untouched member did not follow the prefab; every value is being pinned";

    std::error_code error;
    std::filesystem::remove(path, error);
}

TEST(Integration_PrefabInstance, AnEntityTheAuthorAddedToAnInstanceSurvivesARefresh)
{
    // Destroyed silently today, with no warning and no way to get it back. It carries no
    // identity, so the refresh cannot match it, and an unmatched member survives.
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();
    const std::filesystem::path path = scratch("sushiengine_prefab_added.sushiprefab");

    const EntityId source = build_subtree(world);
    const nlohmann::json first = Scene::capture_prefab(world, source);
    world.set_parent(world.create("AddedLater"), find_by_name(world, "Left"));
    const nlohmann::json second = Scene::capture_prefab(world, source);
    ASSERT_TRUE(write_prefab(second, path));

    clear_world(world);
    const EntityId instance = Scene::apply_prefab(world, first, NULL_ENTITY);
    ASSERT_NE(instance, NULL_ENTITY);
    link_instance(world, instance, path, first.value("revision", std::string()));

    // The author's own entity, belonging to no prefab.
    const EntityId mine = world.create("AuthorsOwnLamp");
    world.set_parent(mine, find_by_name(world, "Right"));

    EXPECT_TRUE(Scene::refresh_prefab_instances(world).empty());

    EXPECT_NE(find_by_name(world, "AuthorsOwnLamp"), NULL_ENTITY)
        << "the refresh destroyed an entity the prefab never owned";

    std::error_code error;
    std::filesystem::remove(path, error);
}

TEST(Integration_PrefabInstance, AMemberThePrefabNoLongerClaimsSurvivesUnlinked)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();
    const std::filesystem::path path = scratch("sushiengine_prefab_removed.sushiprefab");

    const EntityId source = build_subtree(world);
    const nlohmann::json first = Scene::capture_prefab(world, source);

    // The artist removes one entity from the prefab.
    world.destroy(find_by_name(world, "Right"));
    const nlohmann::json second = Scene::capture_prefab(world, source);
    ASSERT_TRUE(write_prefab(second, path));

    clear_world(world);
    const EntityId instance = Scene::apply_prefab(world, first, NULL_ENTITY);
    ASSERT_NE(instance, NULL_ENTITY);
    link_instance(world, instance, path, first.value("revision", std::string()));

    EXPECT_TRUE(Scene::refresh_prefab_instances(world).empty());

    const EntityId orphan = find_by_name(world, "Right");
    ASSERT_NE(orphan, NULL_ENTITY) << "the entity was destroyed rather than unlinked";
    EXPECT_TRUE(world.prefab_entity_id(orphan).empty())
        << "the survivor kept its identity and will be rematched by the next refresh";

    std::error_code error;
    std::filesystem::remove(path, error);
}

TEST(Integration_PrefabInstance, ReorderingAPrefabLeavesEveryIdentityUnchanged)
{
    // The test the positional scheme fails. Under it every id is its index, so inserting
    // an entity ahead of another retargets every override after it.
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();

    const EntityId source = build_subtree(world);
    const nlohmann::json first = Scene::capture_prefab(world, source);
    const std::string deep_before = world.prefab_entity_id(find_by_name(world, "Deep"));
    ASSERT_FALSE(deep_before.empty()) << "capture did not stamp an identity on the source";

    // A new entity ahead of Deep in the walk order, then a re-save.
    world.set_parent(world.create("Inserted"), find_by_name(world, "Root"));
    const nlohmann::json second = Scene::capture_prefab(world, source);

    EXPECT_EQ(world.prefab_entity_id(find_by_name(world, "Deep")), deep_before)
        << "the identity moved when the contents were reordered";
    EXPECT_NE(first.value("revision", std::string("a")),
              second.value("revision", std::string("b")))
        << "the revision must still move when the contents change";
}
