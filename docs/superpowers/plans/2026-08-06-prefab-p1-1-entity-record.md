# Prefab P1 Implementation Plan — part one, the entity record

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

This is **part one of four**, and the only one that changes no behaviour. It makes one entity's
JSON record callable on its own, so the prefab serializer part two builds calls it instead of
copying it.

The others, in order: `2026-08-06-prefab-p1-2-prefab-document.md` (the `.sushiprefab` file),
`2026-08-06-prefab-p1-3-instance-and-refresh.md` (the component and the refresh on load), and
`2026-08-06-prefab-p1-4-authoring-and-import.md` (the authoring gesture and the model importer).
Each consumes the one before it. Task numbers run across all four, so Task 1 here is the Task 1 a
later part's `Interfaces` block names.

Split into four because a plan file obeys `docs/documentation-style-guide.md`'s 900-line ceiling.

---

## File structure

**Created:**

| Path | Responsibility |
|---|---|
| `engine/world/serialization/source/entity_record.hpp` | The three extracted signatures, private to the module. |

**Modified:**

| Path | Change |
|---|---|
| `engine/world/serialization/source/scene_serializer.cpp` | The three loop bodies become the three functions; both callers call them. |

No test file is created. This task's gate is that the existing suite is unmoved.

---

## Task 1: Extract the per-entity record

`capture_scene` and `apply_scene` are single functions of 348 and 430 lines whose bodies are one
loop each. Nothing in them is callable from a second serializer, so a prefab serializer written
today would copy the record shape — and the two copies would drift the first time an entity gains a
field. This task makes them callable. **It changes no behaviour.** Its gate is that all 1469
existing tests still pass.

The extraction is mechanical because `apply_scene` is already two passes, and `:1296`'s comment says
why: the first pass builds each entity from its own entry alone, and the second resolves the
index-based links (parent, joint partner) once every entity exists. That split is exactly what a
prefab needs — instantiating a prefab differs from loading a scene only by not clearing the world.

**Files:**
- Create: `engine/world/serialization/source/entity_record.hpp`
- Modify: `engine/world/serialization/source/scene_serializer.cpp:570-1350`
- Test: none new. The existing suite is the gate.

**Interfaces:**
- Produces, in `SushiEngine::Scene::Detail`:
  - `nlohmann::json write_entity_record(Simulation::IWorldEditor& world, Simulation::EntityId id,
    const std::unordered_map<Simulation::EntityId, int>& index_of, ISceneBlobTable* blobs);`
  - `Simulation::EntityId read_entity_record(Simulation::IWorldEditor& world,
    const nlohmann::json& entry, const ISceneBlobTable* blobs);`
  - `void link_entity_record(Simulation::IWorldEditor& world, const nlohmann::json& entry,
    Simulation::EntityId id, const std::vector<Simulation::EntityId>& created);`

  Task 2 calls all three.

- [ ] **Step 1: Create the private header**

`engine/world/serialization/source/entity_record.hpp`, Apache header copied from
`source/byte_encoding.hpp`, then:

```cpp
#pragma once

/**
 * @file entity_record.hpp
 * @brief One entity's JSON record: written, read back, and linked to its neighbours.
 *
 * Private to this module. It exists so a scene and a prefab share one record shape instead of
 * two that drift: a field added to an entity is carried by both the day it is added to one.
 *
 * Reading is two functions because a record's links are array indices into the document, so
 * neither end can be resolved until every entity exists. `read_entity_record` builds an entity
 * from its own entry alone; `link_entity_record` runs afterwards over the same entries.
 */

#include <unordered_map>
#include <vector>

#include <nlohmann/json.hpp>

#include <SushiEngine/simulation/simulation.hpp>

#include "scene_blob_table.hpp"

namespace SushiEngine
{
    namespace Scene
    {
        namespace Detail
        {
            /**
             * @brief Writes @p id as one entity record.
             *
             * @param world    The world to read.
             * @param id       The entity to write.
             * @param index_of Document index for every entity the document contains. A parent
             *     absent from the map is written as -1, which is how a subtree's root — whose
             *     parent lies outside the document — becomes a root in the document.
             * @param blobs    Where a cooked asset is named rather than inlined, or nullptr to
             *     inline it.
             * @return The record.
             */
            nlohmann::json write_entity_record(
                SushiEngine::Simulation::IWorldEditor& world,
                SushiEngine::Simulation::EntityId id,
                const std::unordered_map<SushiEngine::Simulation::EntityId, int>& index_of,
                ISceneBlobTable* blobs);

            /**
             * @brief Creates one entity from a record and applies every component it carries.
             *
             * Leaves the record's index-based links alone; @ref link_entity_record resolves them.
             *
             * @param world The world to populate.
             * @param entry The record.
             * @param blobs Where a named cooked asset is resolved from, or nullptr.
             * @return The created entity.
             */
            SushiEngine::Simulation::EntityId read_entity_record(
                SushiEngine::Simulation::IWorldEditor& world, const nlohmann::json& entry,
                const ISceneBlobTable* blobs);

            /**
             * @brief Resolves a record's index-based links once every entity exists.
             *
             * @param world   The world to update.
             * @param entry   The record.
             * @param id      The entity @p entry was read into.
             * @param created Every entity of the document, in document order.
             */
            void link_entity_record(SushiEngine::Simulation::IWorldEditor& world,
                                    const nlohmann::json& entry,
                                    SushiEngine::Simulation::EntityId id,
                                    const std::vector<SushiEngine::Simulation::EntityId>& created);
        } // namespace Detail
    } // namespace Scene
} // namespace SushiEngine
```

- [ ] **Step 2: Move the three bodies**

In `scene_serializer.cpp`, define the three in `namespace Detail` **above** `capture_scene`, and
move the loop bodies into them verbatim. Three edits and no others:

1. `write_entity_record`'s body is `capture_scene`'s loop body (`:577-917`), with `json entry;` as
   its first line and `return entry;` as its last.
2. `read_entity_record`'s body is `apply_scene`'s first-pass loop body (`:942-1294`), minus
   `created.push_back(id);`, with `return id;` as its last line.
3. `link_entity_record`'s body is the second-pass loop body (`:1303-1349`), with every
   `created[i]` replaced by the `id` parameter.

**The one deliberate behaviour change:** `entry["parent"]` currently reads
`index_of.at(parent_id)`, which throws when the parent is not in the map. Write instead:

```cpp
const EntityId parent_id = world.parent(id);
const auto parent_entry = index_of.find(parent_id);
entry["parent"] = parent_entry == index_of.end() ? -1 : parent_entry->second;
```

In `capture_scene` every parent is in the map, so this is equivalent there. It is what lets a
subtree capture write its root as a root.

- [ ] **Step 3: Call them from the two callers**

`capture_scene`'s loop becomes:

```cpp
json root = json::array();
for (const EntityId id : ids)
    root.push_back(Detail::write_entity_record(world, id, index_of, blobs));
```

`apply_scene`'s two loops become:

```cpp
std::vector<EntityId> created;
created.reserve(entity_list.size());
for (const auto& entry : entity_list)
    created.push_back(Detail::read_entity_record(world, entry, blobs));

for (std::size_t i = 0; i < entity_list.size(); ++i)
    Detail::link_entity_record(world, entity_list[i], created[i], created);
```

Keep every surrounding line — the environment restore, the wholesale clear, and `:1296`'s comment
(move it onto `link_entity_record`, since it explains that function's existence).

- [ ] **Step 4: [USER CHECKPOINT] Verify nothing changed**

```bash
se build
se test --suite functional
```
Expected: **1469 passed, 0 failed** — the same number as before, because this task adds no test and
changes no behaviour. Any failure here is a transcription error in Step 2, not a design problem;
re-read the moved body against the original before changing anything else.

- [ ] **Step 5: Commit**

```bash
git add engine/world/serialization
git commit -m "refactor(serialization): make one entity's record readable and writable on its own"
```

