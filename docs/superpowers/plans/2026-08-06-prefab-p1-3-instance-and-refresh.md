# Prefab P1 Implementation Plan — part one, the asset and the instance

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

This is **part three of four**: the component that records which prefab a subtree came from, and the
pass that rebuilds a stale one when a scene opens. It consumes parts one and two and must not start
until both are committed.

The others, in order: `2026-08-06-prefab-p1-1-entity-record.md`,
`2026-08-06-prefab-p1-2-prefab-document.md`, then
`2026-08-06-prefab-p1-4-authoring-and-import.md` (the authoring gesture and the model importer).
Task numbers run across all four, so "Task 2" below is part two's task.

Split into four because a plan file obeys `docs/documentation-style-guide.md`'s 900-line ceiling.

---

## File structure

**Created:**

| Path | Responsibility |
|---|---|
| `tests/integration/test_prefab_instance.cpp` | Refresh on load, and that undo does not refresh. |

**Modified:**

| Path | Change |
|---|---|
| `engine/world/simulation/include/SushiEngine/simulation/simulation.hpp` | `PrefabInstanceParameters` and its `IWorldEditor` accessor triple. |
| `engine/world/simulation/source/runtime_simulation.cpp` | Implement the triple on the concrete world. |
| `engine/world/serialization/source/scene_serializer.cpp` | Round-trip the component in the record functions; call the refresh in `load_scene`. |
| `engine/world/serialization/include/SushiEngine/serialization/prefab_serializer.hpp` | Declare `refresh_prefab_instances`. |
| `engine/world/serialization/source/prefab_serializer.cpp` | Implement it. |
| `tests/integration/test_scene_serializer_roundtrip.cpp` | Three cases for the new component. |
| `tests/CMakeLists.txt` | Register the new test file. |
| `docs/reference/changelog.md` | An `Added` bullet. |

---

## Task 3: The instance component

**Files:**
- Modify: `engine/world/simulation/include/SushiEngine/simulation/simulation.hpp`,
  `engine/world/simulation/source/runtime_simulation.cpp`,
  `engine/world/serialization/source/entity_record.hpp`'s implementation in `scene_serializer.cpp`
- Test: `tests/integration/test_scene_serializer_roundtrip.cpp`

**Interfaces:**
- Consumes: nothing from Tasks 1-2.
- Produces: `Simulation::PrefabInstanceParameters` with fields `std::string path` and
  `std::string revision`; `IWorldEditor::has_prefab_instance(EntityId) const`,
  `prefab_instance(EntityId) const`, `set_prefab_instance(EntityId, const
  PrefabInstanceParameters&)`. Task 4 and part two call all four.

- [ ] **Step 1: Declare the component and the accessors**

In `simulation.hpp`, beside the other component structs:

```cpp
/**
 * @brief The prefab an entity's subtree was built from, and the revision it was built at.
 *
 * Carried by an instance's root entity and by no other. Two fields are the whole linkage: the
 * refresh pass compares @ref revision against the prefab's current one and rebuilds what does
 * not match. Edits made inside the subtree do not survive that rebuild; preserving them is
 * override resolution, a later phase.
 */
struct PrefabInstanceParameters
{
    /** @brief The `.sushiprefab` this subtree was built from. */
    std::string path;

    /** @brief The prefab's revision at the time it was built. */
    std::string revision;
};
```

Add the accessor triple to `IWorldEditor` in the shape `has_crowd`/`crowd_parameters`/
`set_crowd_parameters` uses (`simulation.hpp` near `:1736`), Doxygen included. There is no
`set_has_prefab_instance`: a non-empty `path` is what makes an entity an instance, so
`set_prefab_instance` with an empty path clears it. Say that in the setter's Doxygen — it is the one
place this component departs from its neighbours' shape, and a reader who does not find the fourth
accessor needs to be told it is absent on purpose.

- [ ] **Step 2: Write the failing tests**

In `tests/integration/test_scene_serializer_roundtrip.cpp`, beside the existing component cases. The
suite in that file is `Integration_SceneSerializer` — not `..._SceneSerializerRoundtrip`; match it
or the cases land in a suite of their own.

```cpp
TEST(Integration_SceneSerializer, APrefabInstanceSurvivesTheSceneFile)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();

    clear_world(world);
    const EntityId lamp = world.create("StreetLamp");
    PrefabInstanceParameters authored;
    authored.path = "prefabs/street_lamp.sushiprefab";
    authored.revision = "1234567890";
    world.set_prefab_instance(lamp, authored);

    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "sushiengine_prefab_instance.sushiscene";
    std::error_code error;
    std::filesystem::remove(path, error);
    ASSERT_TRUE(Scene::save_scene(world, path.string()));

    clear_world(world);
    ASSERT_TRUE(Scene::load_scene(world, path.string()));

    const EntityId restored = find_by_name(world, "StreetLamp");
    ASSERT_NE(restored, NULL_ENTITY);
    ASSERT_TRUE(world.has_prefab_instance(restored));
    EXPECT_EQ(world.prefab_instance(restored).path, authored.path);
    // The revision has to survive too, or every instance reads as stale on the next load and
    // the refresh rebuilds a world that never changed.
    EXPECT_EQ(world.prefab_instance(restored).revision, authored.revision);

    std::filesystem::remove(path, error);
    std::filesystem::remove(std::filesystem::path(path.string() + ".atmos"), error);
}

TEST(Integration_SceneSerializer, APrefabInstanceSurvivesCaptureApply)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();

    clear_world(world);
    const EntityId lamp = world.create("StreetLamp");
    PrefabInstanceParameters authored;
    authored.path = "prefabs/street_lamp.sushiprefab";
    authored.revision = "1234567890";
    world.set_prefab_instance(lamp, authored);

    const nlohmann::json snapshot = Scene::capture_scene(world);
    world.set_prefab_instance(lamp, PrefabInstanceParameters{});
    Scene::apply_scene(world, snapshot);

    // Undo restores the link, so undoing a "break prefab link" is undoable like anything else.
    const EntityId restored = find_by_name(world, "StreetLamp");
    ASSERT_NE(restored, NULL_ENTITY);
    ASSERT_TRUE(world.has_prefab_instance(restored));
    EXPECT_EQ(world.prefab_instance(restored).path, authored.path);
    EXPECT_EQ(world.prefab_instance(restored).revision, authored.revision);
}

TEST(Integration_SceneSerializer, AnEntityWithNoPrefabInstanceComesBackWithout)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();

    clear_world(world);
    world.create("PlainBox");

    // A component nobody set must not bloat every entity in every file, and must not come back
    // as an instance of a prefab that does not exist — which the refresh pass would then try
    // to read on every load.
    const nlohmann::json snapshot = Scene::capture_scene(world);
    ASSERT_TRUE(snapshot.contains("entities"));
    ASSERT_EQ(snapshot["entities"].size(), 1u);
    EXPECT_FALSE(snapshot["entities"].front().contains("prefab_instance"));

    Scene::apply_scene(world, snapshot);
    const EntityId box = find_by_name(world, "PlainBox");
    ASSERT_NE(box, NULL_ENTITY);
    EXPECT_FALSE(world.has_prefab_instance(box));
}
```

- [ ] **Step 3: [USER CHECKPOINT] Verify the tests fail**

`se build` — expect a compile error naming `set_prefab_instance`.

- [ ] **Step 4: Implement**

Add `PrefabInstanceParameters prefab_instance{};` to the `Record` struct in
`runtime_simulation.cpp` (near `:1987`, where `crowd_parameters` sits) and implement the triple
beside `has_crowd` (`:944-981`). There is no `has_prefab_instance` bool on the record:
`has_prefab_instance` returns `record != nullptr && !record->prefab_instance.path.empty()`, which is
what makes an empty path a clear.

Then round-trip it in Task 1's `write_entity_record` and `read_entity_record`:

```cpp
// In write_entity_record, beside the other optional components:
if (world.has_prefab_instance(id))
{
    const auto parameters = world.prefab_instance(id);
    entry["prefab_instance"] = json{{"path", parameters.path},
                                    {"revision", parameters.revision}};
}

// In read_entity_record:
if (entry.contains("prefab_instance"))
{
    const json& p = entry["prefab_instance"];
    SushiEngine::Simulation::PrefabInstanceParameters parameters;
    parameters.path = p.value("path", std::string());
    parameters.revision = p.value("revision", std::string());
    world.set_prefab_instance(id, parameters);
}
```

No `has_prefab_instance` flag in the record: the block's presence is the flag, which is what
`AnEntityWithNoPrefabInstanceComesBackWithout` asserts. That differs from the neighbours only
because this component has no empty-but-present state to distinguish.

- [ ] **Step 5: [USER CHECKPOINT] Verify**

```bash
se build
se test --suite functional
```
Expected: the three new `Integration_SceneSerializer` cases pass; total 1486.

- [ ] **Step 6: Commit**

```bash
git add engine/world tests/integration/test_scene_serializer_roundtrip.cpp
git commit -m "feat(sim): record which prefab an entity subtree came from"
```

## Task 4: Refresh a stale instance when a scene opens

This task carries §5, whose one hazard is placement. Read §5 before writing.

**Files:**
- Modify: `engine/world/serialization/include/SushiEngine/serialization/prefab_serializer.hpp`,
  `source/prefab_serializer.cpp`, `source/scene_serializer.cpp:1540-1580`
- Test: `tests/integration/test_prefab_instance.cpp`
- Modify: `tests/CMakeLists.txt`, `docs/reference/changelog.md`

**Interfaces:**
- Consumes: Task 2's `read_prefab_revision` and `apply_prefab`; Task 3's accessor triple.
- Produces: `std::vector<std::string> Scene::refresh_prefab_instances(Simulation::IWorldEditor&)`,
  returning the prefab paths it could not read. Called from `load_scene` and from nowhere else in
  this part; part two's editor calls it after authoring a prefab and surfaces the return.

- [ ] **Step 1: Declare it**

In `prefab_serializer.hpp`:

```cpp
/**
 * @brief Rebuilds every prefab instance in @p world whose revision no longer matches its file.
 *
 * For each root carrying `PrefabInstanceParameters`: read the prefab's current revision, and when
 * it differs, destroy the subtree below the root, rebuild it from the file, and write the new
 * revision. The root's own name and transform are preserved — they are the instance's placement,
 * not the prefab's content.
 *
 * Call this from `load_scene` and not from `apply_scene`. `apply_scene` is the path undo restores
 * through, and refreshing there would reinstate the very change being undone (§5).
 *
 * @param world The world to refresh.
 * @return The paths of prefabs that could not be read, in encounter order. Their instances are
 *     left exactly as they were, so a missing file unlinks a subtree rather than deleting it.
 */
std::vector<std::string> refresh_prefab_instances(SushiEngine::Simulation::IWorldEditor& world);
```

- [ ] **Step 2: Write the failing tests**

Create `tests/integration/test_prefab_instance.cpp` with the Apache header and a file comment naming
what it pins. It needs the same `clear_world`, `find_by_name` and `build_subtree` helpers as the
prefab serializer's file; repeat them in this file's anonymous namespace rather than sharing a
header — the existing test files each carry their own, and one shared helper header linking two
suites is a change this plan does not make.

Plus one helper this file owns:

```cpp
    /** @brief Writes @p document to @p path, and returns whether it landed. */
    bool write_prefab(const nlohmann::json& document, const std::filesystem::path& path)
    {
        std::ofstream file(path.string());
        if (!file)
            return false;
        file << document.dump(2);
        return static_cast<bool>(file);
    }
```

```cpp
TEST(Integration_PrefabInstance, AStaleInstanceIsRebuiltAndTheRootKeepsItsNameAndTransform)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();

    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "sushiengine_prefab_stale.sushiprefab";
    std::error_code error;
    std::filesystem::remove(path, error);

    // Author the prefab, then change it, so the file on disk is ahead of the instance.
    const EntityId source = build_subtree(world);
    const nlohmann::json first = Scene::capture_prefab(world, source);
    world.set_parent(world.create("AddedLater"), find_by_name(world, "Left"));
    const nlohmann::json second = Scene::capture_prefab(world, source);
    ASSERT_NE(first.value("revision", std::string("a")),
              second.value("revision", std::string("b")));
    ASSERT_TRUE(write_prefab(second, path));

    // Place an instance built from the *first* revision, and give the root a placement of its
    // own that the rebuild must not overwrite.
    clear_world(world);
    const EntityId instance = Scene::apply_prefab(world, first, NULL_ENTITY);
    ASSERT_NE(instance, NULL_ENTITY);
    world.set_name(instance, "LampPost_03");
    EntityTransform placement;
    placement.position = Vector3{100.0, 200.0, 300.0};
    placement.scale = Vector3{2.0, 2.0, 2.0};
    world.set_transform(instance, placement);
    PrefabInstanceParameters link;
    link.path = path.string();
    link.revision = first.value("revision", std::string());
    world.set_prefab_instance(instance, link);

    const std::vector<std::string> unreadable = Scene::refresh_prefab_instances(world);
    EXPECT_TRUE(unreadable.empty());

    // The subtree is the file's, not the snapshot's.
    EXPECT_NE(find_by_name(world, "AddedLater"), NULL_ENTITY);
    EXPECT_EQ(world.entities().size(), 5u);
    // The placement is the scene's, not the file's.
    EXPECT_EQ(world.name(instance), "LampPost_03");
    EXPECT_DOUBLE_EQ(world.transform(instance).position.x, 100.0);
    EXPECT_DOUBLE_EQ(world.transform(instance).scale.x, 2.0);
    // And the instance is no longer stale, or the next load rebuilds it again.
    EXPECT_EQ(world.prefab_instance(instance).revision, second.value("revision", std::string()));

    std::filesystem::remove(path, error);
}

TEST(Integration_PrefabInstance, AnInstanceAtTheCurrentRevisionIsLeftAlone)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();

    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "sushiengine_prefab_current.sushiprefab";
    std::error_code error;
    std::filesystem::remove(path, error);

    const EntityId source = build_subtree(world);
    const nlohmann::json document = Scene::capture_prefab(world, source);
    ASSERT_TRUE(write_prefab(document, path));

    clear_world(world);
    const EntityId instance = Scene::apply_prefab(world, document, NULL_ENTITY);
    ASSERT_NE(instance, NULL_ENTITY);
    PrefabInstanceParameters link;
    link.path = path.string();
    link.revision = document.value("revision", std::string());
    world.set_prefab_instance(instance, link);

    // A marker inside the subtree, so "left alone" is observable rather than inferred from the
    // entity count — an unconditional rebuild produces the same count and destroys this.
    const EntityId marker = find_by_name(world, "Deep");
    ASSERT_NE(marker, NULL_ENTITY);

    EXPECT_TRUE(Scene::refresh_prefab_instances(world).empty());

    EXPECT_EQ(find_by_name(world, "Deep"), marker) << "the subtree was rebuilt when it was current";
    EXPECT_EQ(world.entities().size(), 4u);

    std::filesystem::remove(path, error);
}

TEST(Integration_PrefabInstance, AMissingPrefabLeavesTheEntitiesInPlaceAndDoesNotCrash)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();

    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "sushiengine_prefab_missing.sushiprefab";
    std::error_code error;
    std::filesystem::remove(path, error);

    const EntityId source = build_subtree(world);
    const nlohmann::json document = Scene::capture_prefab(world, source);
    clear_world(world);
    const EntityId instance = Scene::apply_prefab(world, document, NULL_ENTITY);
    ASSERT_NE(instance, NULL_ENTITY);
    PrefabInstanceParameters link;
    link.path = path.string();
    link.revision = "whatever-it-was";
    world.set_prefab_instance(instance, link);

    const std::vector<std::string> unreadable = Scene::refresh_prefab_instances(world);
    ASSERT_EQ(unreadable.size(), 1u);
    EXPECT_EQ(unreadable.front(), path.string());

    // §8 says a missing prefab "marks the root unlinked". Read that as what the editor
    // *shows*, not as clearing the component — the returned path is what the editor marks
    // from. Clearing it would make a file that is merely not pulled yet, or on an unmounted
    // drive, destroy the link permanently: restoring the file would restore nothing, and the
    // user would have no way to tell that from a prefab they deliberately unlinked. The
    // component is the cheapest thing in the scene and the only record of the association.
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

    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "sushiengine_prefab_undo.sushiprefab";
    std::error_code error;
    std::filesystem::remove(path, error);

    const EntityId source = build_subtree(world);
    const nlohmann::json first = Scene::capture_prefab(world, source);
    world.set_parent(world.create("AddedLater"), find_by_name(world, "Left"));
    ASSERT_TRUE(write_prefab(Scene::capture_prefab(world, source), path));

    clear_world(world);
    const EntityId instance = Scene::apply_prefab(world, first, NULL_ENTITY);
    ASSERT_NE(instance, NULL_ENTITY);
    PrefabInstanceParameters link;
    link.path = path.string();
    link.revision = first.value("revision", std::string());
    world.set_prefab_instance(instance, link);

    const nlohmann::json snapshot = Scene::capture_scene(world);
    world.destroy(find_by_name(world, "Right"));
    Scene::apply_scene(world, snapshot);

    // §5's whole hazard. apply_scene is the path undo restores through: refreshing there would
    // reinstate the prefab edit the user is undoing, so the instance must still be stale here
    // and the file's later entity must still be absent.
    const EntityId restored = find_by_name(world, "Root");
    ASSERT_NE(restored, NULL_ENTITY);
    EXPECT_EQ(world.prefab_instance(restored).revision, first.value("revision", std::string()));
    EXPECT_EQ(find_by_name(world, "AddedLater"), NULL_ENTITY);
    EXPECT_EQ(world.entities().size(), 4u);

    std::filesystem::remove(path, error);
}

TEST(Integration_PrefabInstance, LoadSceneRefreshes)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();

    const std::filesystem::path prefab_path =
        std::filesystem::temp_directory_path() / "sushiengine_prefab_on_load.sushiprefab";
    const std::filesystem::path scene_path =
        std::filesystem::temp_directory_path() / "sushiengine_prefab_on_load.sushiscene";
    std::error_code error;
    std::filesystem::remove(prefab_path, error);
    std::filesystem::remove(scene_path, error);

    // A scene saved against revision one, and a prefab file that has since moved on: the exact
    // situation a user meets when a teammate edits a prefab they have placed.
    const EntityId source = build_subtree(world);
    const nlohmann::json first = Scene::capture_prefab(world, source);
    clear_world(world);
    const EntityId instance = Scene::apply_prefab(world, first, NULL_ENTITY);
    ASSERT_NE(instance, NULL_ENTITY);
    PrefabInstanceParameters link;
    link.path = prefab_path.string();
    link.revision = first.value("revision", std::string());
    world.set_prefab_instance(instance, link);
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

    std::filesystem::remove(prefab_path, error);
    std::filesystem::remove(scene_path, error);
    std::filesystem::remove(std::filesystem::path(scene_path.string() + ".atmos"), error);
}
```

- [ ] **Step 3: Register the test**

Add `integration/test_prefab_instance.cpp` to `tests/CMakeLists.txt`.

- [ ] **Step 4: [USER CHECKPOINT] Verify they fail**

`se build` — expect a link error naming `refresh_prefab_instances`.

- [ ] **Step 5: Implement**

In `prefab_serializer.cpp`:

```cpp
std::vector<std::string> refresh_prefab_instances(IWorldEditor& world)
{
    std::vector<std::string> unreadable;

    // Snapshot the entity list first: the loop destroys and creates entities, and iterating a
    // list that the body invalidates is the bug this line exists to prevent.
    const std::vector<EntityId> roots = world.entities();
    for (const EntityId id : roots)
    {
        if (!world.has_prefab_instance(id))
            continue;
        const PrefabInstanceParameters link = world.prefab_instance(id);

        std::string current;
        if (!read_prefab_revision(link.path, current))
        {
            unreadable.push_back(link.path);
            continue;
        }
        if (current == link.revision)
            continue;
        // ... rebuild
    }
    return unreadable;
}
```

The rebuild, in order:

1. Read the document with the same `try`/`catch` `read_prefab_revision` uses. A file whose revision
   parsed but whose body does not is still an unreadable prefab — push the path and `continue`.
2. Destroy the root's descendants. `IWorldEditor` has no `children`, so collect them by walking
   `world.entities()` and following `parent` up to the root before destroying anything; destroying
   inside the walk invalidates it.
3. `apply_prefab(world, document, id)` — the new subtree hangs under the surviving root, which is
   how the root's name and transform are preserved without saving and restoring them.
4. The document's own entry 0 became a *child* of the root in step 3. Reparent entry 0's children to
   the root and destroy that duplicate entry, so the instance does not gain a level on every
   refresh. **This is the step a rebuild most easily gets wrong**, and
   `AStaleInstanceIsRebuiltAndTheRootKeepsItsNameAndTransform` asserts the entity count that catches
   it: 5, not 6.
5. `world.set_prefab_instance(id, PrefabInstanceParameters{link.path, current});`

Then call it from `load_scene`, beside the two existing `resolve_scene_assets(world, *assets)` calls
at `:1562` and `:1573`:

```cpp
apply_scene(world, root);
if (assets != nullptr)
    resolve_scene_assets(world, *assets);
// Beside resolve_scene_assets and for the same reason: both are post-load passes that must not
// run in apply_scene, which is undo's path (§5). load_scene has nowhere to report to, so the
// unreadable paths are dropped here; the editor calls this directly and surfaces them.
refresh_prefab_instances(world);
```

`load_scene` includes `prefab_serializer.hpp`.

- [ ] **Step 6: [USER CHECKPOINT] Verify**

```bash
se build
se test --suite functional
```
Expected: every `Integration_PrefabInstance.*` case passes, `ApplySceneDoesNotRefresh` included;
total 1491.

- [ ] **Step 7: Changelog and commit**

Add under `## [Unreleased]` / `Added`, at most 240 characters:

```markdown
- Added prefab assets: an entity subtree saved as a `.sushiprefab`, placed as instances, and
  rebuilt from the asset when a scene is opened at an older revision.
```

```bash
python tools/documentation/check_documentation_length.py
python tools/layering/check_include_layering.py
git add engine/world tests docs/reference/changelog.md
git commit -m "feat(serialization): rebuild a stale prefab instance when a scene opens"
```


---

## What parts one to three do not build

- **The authoring gesture.** Nothing writes a `.sushiprefab` yet; `capture_prefab` has no caller
  outside its tests. Part four adds the drag from the Hierarchy to the Project panel, unlinked
  ImGui first.
- **The model importer producing prefabs** (§7). Part four.
- **Overrides, nesting, prefab edit mode, runtime instantiation.** Later phases (§10).
