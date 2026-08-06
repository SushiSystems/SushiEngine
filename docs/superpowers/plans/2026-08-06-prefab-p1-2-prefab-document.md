# Prefab P1 Implementation Plan — part two, the prefab document

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development
> (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use
> checkbox (`- [ ]`) syntax for tracking.

**Goal:** Save an entity subtree as a `.sushiprefab` asset, place instances of it, and rebuild stale
instances when a scene is opened.

**Architecture:** A prefab file is one rooted slice of the entity array `capture_scene` already
produces. Task 1 extracts that per-entity record into functions so the prefab serializer calls them
rather than copying them. A scene stores an instance expanded, plus one component on its root naming
the prefab and the revision it was built from. A refresh pass runs in `load_scene` — beside
`resolve_scene_assets`, which already sits there for the same reason — and never in `apply_scene`,
which is undo's path.

**Tech Stack:** C++17, nlohmann_json, GoogleTest, CMake with `sushiengine_add_module`.

**Spec:** `docs/design/prefab_system.md`. Section references (§N) point into it.

## Global Constraints

- **C++17 only.** No C++20/23 facilities.
- **Allman braces**, namespaces included; nested namespaces written out, never `namespace A::B`.
- **Naming:** types `PascalCase`, functions and variables `snake_case`, members trailing underscore,
  constants `UPPER_SNAKE`, namespaces `PascalCase`.
- **No abbreviations in any identifier or in prose.** Acronyms stay fully upper-case.
- **Every CMake target, function and option starts with `sushiengine_`.**
- **Every new source file carries the Apache 2.0 header**, copied verbatim from a neighbour.
- **Every public function carries Doxygen**: `@brief` (why it exists), one line on the mechanism
  when it is not obvious, `@param` for each parameter, `@return`.
- **No historical references in comments**, no separator comments.
- **Prose in `docs/` obeys `docs/documentation-style-guide.md`**: 100-column lines, present tense,
  no marketing, honest about gaps, every path real and every link resolving.
- **Never invoke `cmake` or `ninja` directly.** Only the `se` CLI.

## Build and test policy — read before Task 1

**The implementing agent does not run builds or tests.** Do not start any process that runs `se`,
`cmake`, `ninja` or `ctest`, in the foreground or the background, and do not write or edit any build
configuration file, including `cli/config.local.toml`. This machine cannot carry a build; the user
runs every one. Steps marked **[USER CHECKPOINT]** are handed to the user with the exact command and
the expected output.

**Never report a test as passing that you did not see pass.** Writing the test is the agent's job;
observing it fail and then pass is the user's.

**Test suite names must be prefixed** `Unit_`, `Integration_` or `Regression_`.
`tests/CMakeLists.txt:665-676` registers `gtest_discover_tests` three times filtering on exactly
those, so a suite without a prefix is discovered by nothing and the run reports green having
executed zero of its cases. The suite currently stands at **1469 tests, all passing**; anything
added must raise that count.

Checks the agent runs itself — pure Python, no compiler:

```bash
python tools/documentation/check_documentation_length.py
python tools/documentation/check_module_documentation.py
python tools/layering/check_include_layering.py
```

Pre-existing and not yours: four changelog bullet-length warnings, and three `world -> presentation`
layering notes.

## Where this plan sits

This is **part two of four**: the `.sushiprefab` file itself — captured from a subtree, applied back
into a world, and hashed into a revision. It consumes part one's extracted record functions and must
not start until part one is committed.

The others, in order: `2026-08-06-prefab-p1-1-entity-record.md` (done first),
`2026-08-06-prefab-p1-3-instance-and-refresh.md` (the component and the refresh on load), and
`2026-08-06-prefab-p1-4-authoring-and-import.md` (the authoring gesture and the model importer).
Task numbers run across all four.

Split into four because a plan file obeys `docs/documentation-style-guide.md`'s 900-line ceiling.

---

## File structure

**Created:**

| Path | Responsibility |
|---|---|
| `engine/world/serialization/include/SushiEngine/serialization/prefab_serializer.hpp` | Write a subtree to a prefab document, read one back, compute its revision. |
| `engine/world/serialization/source/prefab_serializer.cpp` | Those three, over part one's record functions. |
| `tests/unit/test_prefab_serializer.cpp` | Round trip, revision stability, identity. |

**Modified:**

| Path | Change |
|---|---|
| `engine/world/serialization/source/byte_encoding.hpp/.cpp` | A `std::string` overload of `content_hash`. |
| `engine/world/serialization/CMakeLists.txt` | Add `source/prefab_serializer.cpp`. |
| `engine/world/serialization/README.md` | The new header, and the test. |
| `tests/CMakeLists.txt` | Register the new test file. |

---

## Task 2: The prefab document

**Files:**
- Create: `engine/world/serialization/include/SushiEngine/serialization/prefab_serializer.hpp`,
  `engine/world/serialization/source/prefab_serializer.cpp`
- Modify: `engine/world/serialization/source/byte_encoding.hpp`, `source/byte_encoding.cpp`,
  `CMakeLists.txt`, `README.md`, `tests/CMakeLists.txt`
- Test: `tests/unit/test_prefab_serializer.cpp`

**Interfaces:**
- Consumes: Task 1's three `Detail` functions.
- Produces: `Scene::capture_prefab`, `Scene::apply_prefab`, `Scene::prefab_revision`,
  `Scene::read_prefab_revision`. Tasks 3, 4 and part two call these.

- [ ] **Step 1: Add the string overload of the content hash**

`prefab_revision` hashes text, and this module already owns an FNV-1a 64 content hash over bytes
(`source/byte_encoding.cpp:190`). Reuse it rather than adding a second hash. In
`source/byte_encoding.hpp`, beside the existing declaration:

```cpp
/**
 * @brief FNV-1a 64 of a string's bytes.
 *
 * The same function as the byte overload, over the other thing this module hashes: a document's
 * serialized text. One hash for both keeps a prefab's revision and a blob's key comparable.
 *
 * @param text The text to hash.
 * @return The hash (the offset basis for empty input).
 */
std::uint64_t content_hash(const std::string& text) noexcept;
```

In `byte_encoding.cpp`, give both overloads one body by having each call a file-local
`hash_bytes(const unsigned char* data, std::size_t length)` holding the loop that is currently in
`content_hash`. Add `#include <string>` to the header.

- [ ] **Step 2: Write the public header**

`include/SushiEngine/serialization/prefab_serializer.hpp`, Apache header copied from
`scene_serializer.hpp` lines 1-22, then:

```cpp
#pragma once

/**
 * @file prefab_serializer.hpp
 * @brief An entity subtree as a reusable asset.
 *
 * A prefab document is one rooted slice of the entity array `capture_scene` writes, built from
 * the same per-entity record, so a field added to an entity is carried by prefabs the day it is
 * added to scenes. The environment is deliberately absent — a prefab is a subtree, and a street
 * light carrying the sky it was authored under would apply that sky wherever it is placed.
 */

#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include <SushiEngine/simulation/simulation.hpp>

namespace SushiEngine
{
    namespace Scene
    {
        /**
         * @brief Captures @p root and every descendant as a prefab document.
         *
         * Adds a `prefab_entity_id` per entry: a value unique within the document, which override
         * resolution will key on (§4.4). Nothing reads it in this phase and it is written anyway,
         * because a prefab authored without one cannot be matched afterwards.
         *
         * @param world The world to read.
         * @param root  The subtree's root; entry 0 of the result.
         * @return An object `{ "revision": "...", "entities": [...] }`, or one whose `entities`
         *     array is empty when @p root is not a live entity.
         */
        nlohmann::json capture_prefab(SushiEngine::Simulation::IWorldEditor& world,
                                      SushiEngine::Simulation::EntityId root);

        /**
         * @brief Builds a prefab document's entities under @p parent.
         *
         * Nothing existing is destroyed: a caller replacing a subtree removes the old one first.
         *
         * @param world    The world to populate.
         * @param document An object in the shape @ref capture_prefab produces.
         * @param parent   The entity to hang the subtree under, or `NULL_ENTITY` for a root.
         * @return The created root, or `NULL_ENTITY` when the document holds no entity.
         */
        SushiEngine::Simulation::EntityId apply_prefab(
            SushiEngine::Simulation::IWorldEditor& world, const nlohmann::json& document,
            SushiEngine::Simulation::EntityId parent);

        /**
         * @brief The content hash of a prefab document's entity array.
         *
         * A hash rather than a counter, so the same content hashes the same on two machines and
         * reverting a prefab restores its previous revision instead of advancing past it — which
         * is what the staleness comparison in §5 needs and what a counter gets wrong in both.
         *
         * @param entities The document's `entities` array.
         * @return The revision string.
         */
        std::string prefab_revision(const nlohmann::json& entities);

        /**
         * @brief Reads just the revision out of a prefab file.
         *
         * Separate from a full read because the refresh pass asks "is this instance stale" for
         * every instance in a scene and rebuilds only the ones that are; parsing whole documents
         * to answer a string comparison would make opening a scene scale with the size of its
         * prefabs rather than their number.
         *
         * @param path Path to a `.sushiprefab` file.
         * @param out  Receives the revision; cleared on failure.
         * @return False when the file cannot be read or parsed, or carries no revision.
         */
        bool read_prefab_revision(const std::string& path, std::string& out);
    } // namespace Scene
} // namespace SushiEngine
```

- [ ] **Step 3: Write the failing tests**

Create `tests/unit/test_prefab_serializer.cpp` with the Apache header copied from
`tests/integration/test_scene_serializer_roundtrip.cpp` lines 1-22, then a file comment stating what
the file pins, then:

```cpp
#include <filesystem>
#include <fstream>
#include <algorithm>
#include <set>
#include <string>
#include <system_error>

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

    const nlohmann::json document = capture_prefab_of_reference_subtree(world);

    // Clear the world entirely, so what comes back was rebuilt from the document and not
    // merely left in place.
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
    // -1 and not an index: the root's own parent lies outside the document, and a capture
    // that wrote an index here would parent the instance to whatever entity that index hit.
    EXPECT_EQ(entities.front().value("parent", 0), -1);

    // Every other entry's parent chain reaches entry 0 (§3).
    for (std::size_t i = 1; i < entities.size(); ++i)
    {
        int walker = entities[i].value("parent", -1);
        int steps = 0;
        while (walker > 0 && steps < static_cast<int>(entities.size()))
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
    // A sibling of the root and a child of that sibling: neither is a descendant of Root, and
    // a capture that walked the whole world rather than the subtree would carry both.
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

    // The same subtree captured twice: equal revisions, or §5 rebuilds every instance on every
    // load. This is also what fails if the identifier is made random or time-based.
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

    EntityTransform moved = world.transform(find_by_name(world, "Deep"));
    moved.position.x += 1.0;
    world.set_transform(find_by_name(world, "Deep"), moved);

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

    world.set_parent(world.create("Added"), find_by_name(world, "Right"));

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

    // §9 lists order among the things a revision must notice. It holds trivially while the
    // revision hashes the serialized array, and stops holding the day someone "improves" it
    // into a hash over a set of per-entity hashes — which is exactly the change this case is
    // here to fail.
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

TEST(Unit_PrefabSerializer, ApplyingAnEmptyDocumentCreatesNothingAndReturnsNullEntity)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();
    clear_world(world);
    const EntityId survivor = world.create("Survivor");

    const nlohmann::json empty = nlohmann::json{{"revision", ""},
                                                {"entities", nlohmann::json::array()}};
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
    // The cheap read and the full computation agree, which is the whole premise of §5 asking
    // the cheap one.
    EXPECT_EQ(revision, Scene::prefab_revision(document["entities"]));

    std::error_code error;
    std::filesystem::remove(path, error);
}
```

`capture_prefab_of_reference_subtree` in the first case is shorthand for the two lines every other
case writes out; replace it with them:

```cpp
    const EntityId root = build_subtree(world);
    const nlohmann::json document = Scene::capture_prefab(world, root);
```

- [ ] **Step 4: Register the source and the test**

Add `source/prefab_serializer.cpp` to `engine/world/serialization/CMakeLists.txt`'s `SOURCES`, and
`unit/test_prefab_serializer.cpp` to `tests/CMakeLists.txt`'s source list beside the other unit
tests.

- [ ] **Step 5: [USER CHECKPOINT] Verify it fails**

Hand the user: `se build`
Expected: a link error naming `capture_prefab`. Wait for their report.

- [ ] **Step 6: Implement**

`prefab_serializer.cpp` includes `entity_record.hpp` and `byte_encoding.hpp`.

`capture_prefab` collects the subtree parent-before-child. `IWorldEditor` exposes `parent` but no
`children`, so walk `world.entities()` — which is in creation order, not tree order — to a child
list first, then descend:

```cpp
// Parent before child, so apply_prefab's link pass always finds a parent already created,
// and so entry 0 is the root.
std::vector<EntityId> ordered;
if (root != NULL_ENTITY)
{
    std::unordered_map<EntityId, std::vector<EntityId>> children;
    for (const EntityId id : world.entities())
        children[world.parent(id)].push_back(id);

    std::vector<EntityId> pending{root};
    while (!pending.empty())
    {
        const EntityId id = pending.back();
        pending.pop_back();
        ordered.push_back(id);
        const auto found = children.find(id);
        if (found == children.end())
            continue;
        for (auto it = found->second.rbegin(); it != found->second.rend(); ++it)
            pending.push_back(*it);
    }
}
```

`root != NULL_ENTITY` alone does not prove the entity is live — a destroyed id is also not
`NULL_ENTITY`. Guard on membership in `world.entities()` instead, which is what
`CapturingANonLiveEntityYieldsAnEmptyDocument` asserts.

Then build `index_of` over `ordered` only, call `Detail::write_entity_record(world, id, index_of,
nullptr)` per entity, and set `entry["prefab_entity_id"]` to `"e" + std::to_string(position)`.

A sequential index is enough for §4.4: unique within the document and stable across reads. §4.4
explicitly does not require it to survive a re-author. **Do not use a random or time-based value** —
it would make the revision hash unstable and fail `IdenticalContentHashesIdentically`.

`blobs` is `nullptr`: a prefab file must open on another machine, so a cooked asset is inlined, the
same choice `save_scene` makes and `ASoftBodySurvivesTheSceneFileByValue` pins.

`apply_prefab` mirrors `apply_scene`'s two passes without the clear:

```cpp
std::vector<EntityId> created;
created.reserve(entities.size());
for (const auto& entry : entities)
    created.push_back(Detail::read_entity_record(world, entry, nullptr));
for (std::size_t i = 0; i < entities.size(); ++i)
    Detail::link_entity_record(world, entities[i], created[i], created);
world.set_parent(created.front(), parent);
return created.front();
```

The reparent comes after the link pass, not before: entry 0's own `parent` is -1, so the link pass
leaves it a root, and this line places it.

`prefab_revision` returns `std::to_string(Detail::content_hash(entities.dump()))`.

`read_prefab_revision` clears `out` first, opens the file, parses inside `try`/`catch
(const nlohmann::json::parse_error&)` the way `load_scene:1551` does, and returns false unless the
result is an object carrying a non-empty string `revision`.

- [ ] **Step 7: [USER CHECKPOINT] Verify the tests pass**

```bash
se build
se test --suite functional
```
Expected: every `Unit_PrefabSerializer.*` case passes, and the total rises from 1469 to 1484 — the
fourteen cases above plus `CapturingADestroyedEntityYieldsAnEmptyDocument`, which the
implementation added because a destroyed id is not `NULL_ENTITY` and a guard checking only for
`NULL_ENTITY` would walk one. A total that rises by less means a case was dropped; a total
unchanged means the suite prefix is wrong and nothing was discovered.

- [ ] **Step 8: Update the README and commit**

Add `prefab_serializer.hpp` to `engine/world/serialization/README.md`'s public-surface table, and
name `tests/unit/test_prefab_serializer.cpp` in its Tests section. Then:

```bash
python tools/documentation/check_documentation_length.py
python tools/documentation/check_module_documentation.py
python tools/layering/check_include_layering.py
git add engine/world/serialization tests/unit/test_prefab_serializer.cpp tests/CMakeLists.txt
git commit -m "feat(serialization): save an entity subtree as a prefab document"
```

