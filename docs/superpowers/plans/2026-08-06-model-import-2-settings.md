# Model Import Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development
> (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use
> checkbox (`- [ ]`) syntax for tracking.

**Goal:** Import a glTF file's node graph as an entity hierarchy, driven by per-asset settings
in a `.meta` sidecar, reachable by dragging the asset out of the Project panel.

**Architecture:** Three layers that cannot see each other. `engine/asset/gltf` reads the file into a
renderer-free `GLTFSceneDescription`. A new `engine/asset/model` module turns that plus a
`ModelImportSettings` into a `ModelInstantiationPlan` through one pure function with no device and
no editor. The editor executes the plan against `IWorldEditor` and `IAssetLibrary` and decides
nothing. Every hard decision is therefore unit-testable without a GPU.

**Tech Stack:** C++17, cgltf 1.15 (already vendored, owned by `sushiengine_gltf`), nlohmann_json,
GoogleTest, CMake with `sushiengine_add_module`, Dear ImGui for the editor surface.

**Spec:** `docs/design/model_import.md`. Every section reference below (§N) points into it.

## Global Constraints

- **C++17 only.** No C++20/23 facilities. (`docs/CONTRIBUTING.md` §4)
- **Allman braces**, opening brace on its own line, including for namespaces. Nested namespaces
  written out (`namespace A\n{\n namespace B\n{`), never `namespace A::B`.
- **Naming:** types `PascalCase`, functions and variables `snake_case`, members trailing underscore,
  constants `UPPER_SNAKE`, namespaces `PascalCase`.
- **No abbreviations in any identifier or in prose.** `Statistics` not `Stats`, `Parameters` not
  `Params`. Well-known acronyms stay fully upper-case: `GPU`, `API`, `GLTF`, `ECS`.
- **Every CMake target, function and option starts with `sushiengine_`.** Never bare `sushi_`.
- **Every new source file carries the Apache 2.0 header** copied verbatim from a neighbouring file.
- **Every public function carries Doxygen** with `@brief` (why it exists), one line on the mechanism
  when non-obvious, `@param` for each parameter and `@return`. (`docs/CONTRIBUTING.md` §4)
- **No historical references in comments.** No "previously", no "we used to", no issue numbers.
- **No separator comments** (`// =====`, `// -----`).
- **Prose in `docs/` obeys `docs/documentation-style-guide.md`:** 100-column lines, present tense,
  no marketing, honest about gaps, every path real and every link resolving.
- **Never invoke `cmake` or `ninja` directly.** Only the `se` CLI. (`CLAUDE.md`)

## Build and test policy — read before Task 1

**The implementing agent does not run builds or tests.** This machine cannot carry them; the user
runs every `se build` and `se test` invocation. Steps below marked **[USER CHECKPOINT]** are handed
to the user with the exact command, and work stops until they report the result.

An agent must never report a test as passing that it did not see pass. Where a step cannot be
executed, say so plainly and hand it over. Writing the test is the agent's job; observing it fail
and then pass is the user's.

The three checks that guard the tree, all runnable by the agent because they are pure Python and
touch no compiler:

```bash
python tools/documentation/check_documentation_length.py
python tools/documentation/check_module_documentation.py
python tools/layering/check_include_layering.py
```

The build and test commands the user runs:

```bash
se build                          # configure and build
se test --suite functional         # unit + integration + regression
```

---
---

## Test naming — non-negotiable

`tests/CMakeLists.txt:665-676` registers `gtest_discover_tests` three times, filtering on
`Unit_*`, `Integration_*` and `Regression_*`. **A test whose suite name carries none of those
prefixes is discovered by nothing**: CTest runs zero of its cases and `se test --suite functional`
reports green having never executed them. Every suite name in this part is therefore
`Unit_<Name>` for a file under `tests/unit/` and `Integration_<Name>` for one under
`tests/integration/`. Do not drop the prefix, and do not trust a green run of a test you cannot
see named in the CTest output.

## Where this plan sits

This is **part two of four** of the model import plan. It builds the per-asset settings and
their `.meta` sidecar,

together with the one-way migration off the project's path-keyed override store.

Consumes from part one: nothing. This part is independent of it and could be built first;
it is second because part three needs both.

Produces `Model::ModelImportSettings`, `load_model_import_settings`,
`save_model_import_settings`, `model_import_settings_path` and
`migrate_cooking_overrides_to_sidecars`, and the `sushiengine_model` module itself.

The plan is split because a plan file obeys `docs/documentation-style-guide.md`'s 900-line
ceiling. Each part is one subject and each ends with something that builds and tests.

---

## Task 2: The `model` module and its import settings

**Files:**
- Create: `engine/asset/model/CMakeLists.txt`, `README.md`,
  `include/SushiEngine/model/import_settings.hpp`,
  `include/SushiEngine/model/import_settings_io.hpp`, `source/import_settings_io.cpp`
- Modify: `cmake/EngineLayers.cmake:18-42`, `engine/asset/CMakeLists.txt`,
  `docs/modules/README.md`, `tests/CMakeLists.txt`
- Test: `tests/unit/test_model_import_settings_io.cpp`

**Interfaces:**
- Consumes: `Physics::Cooking::ImportProfileOverride` from
  `engine/domain/physics/include/SushiEngine/physics/cooking/import_profile.hpp:110-134`.
- Produces: `SushiEngine::Model::ModelImportSettings`, and
  `bool load_model_import_settings(const std::string& asset_path, ModelImportSettings& out)`,
  `bool save_model_import_settings(const std::string& asset_path, const ModelImportSettings&)`,
  `std::string model_import_settings_path(const std::string& asset_path)`. Tasks 3, 4, 6 and 9
  depend on these names.

- [ ] **Step 1: Register the module in the layer manifest**

In `cmake/EngineLayers.cmake`, inside `set(SUSHIENGINE_MODULE_LAYERS ...)`, add a line directly
after `gltf            asset`:

```cmake
    model           asset
```

The list is `<module> <layer>` pairs; misaligning it is cosmetic, omitting it makes
`sushiengine_add_module` raise `FATAL_ERROR` at configure time.

In `engine/asset/CMakeLists.txt`, after `add_subdirectory(gltf)`:

```cmake
add_subdirectory(model)
```

- [ ] **Step 2: Write the settings header**

Create `engine/asset/model/include/SushiEngine/model/import_settings.hpp` with the Apache header:

```cpp
#pragma once

/**
 * @file import_settings.hpp
 * @brief What one model asset says about how it is imported.
 *
 * The payload of the `.meta` file that sits beside the asset. Settings live beside the asset
 * rather than in a project-wide table keyed by path so that moving, renaming or copying the
 * asset carries them along instead of orphaning them.
 *
 * Every default is "change nothing". glTF fixes a right-handed coordinate system with +Y up and
 * metres for every linear distance, so a conformant file needs no correction and the two
 * transform fields exist for a file whose exporter ignored that.
 */

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/cooking/import_profile.hpp>

namespace SushiEngine
{
    namespace Model
    {
        /** @brief 3-vector in import precision; the same per-namespace alias `Geometry` uses. */
        using Vector3f = Vector3T<float>;

        /** @brief The per-asset import settings a `.meta` file holds. */
        struct ModelImportSettings
        {
            /** @brief Uniform scale applied at import; 1 leaves the file's own units alone. */
            float scale_factor = 1.0f;

            /**
             * @brief Rotation applied to the imported subtree's root, in degrees per axis.
             *
             * An escape hatch for a file whose exporter ignored glTF's orientation, not an
             * axis-convention picker. Zero, the default, is what a conformant file needs.
             */
            Vector3f root_rotation_degrees{0.0f, 0.0f, 0.0f};

            /** @brief Adopt the file's materials; off leaves each entity the engine default. */
            bool import_materials = true;

            /** @brief Turn `KHR_lights_punctual` lights into Light entities. */
            bool import_lights = true;

            /** @brief Turn glTF cameras into Camera entities, created inactive. */
            bool import_cameras = true;

            /** @brief Keep nodes that carry nothing, so authored pivots survive the import. */
            bool preserve_pivots = true;

            /** @brief Queue every mesh-carrying node for the physics cooking pipeline. */
            bool generate_colliders = false;

            /** @brief What this asset says differs from the project's cooking defaults. */
            Physics::Cooking::ImportProfileOverride cooking;
        };

        /**
         * @brief Whether two settings objects would produce the same import.
         *
         * @param a First operand.
         * @param b Second operand.
         * @return True when every field matches.
         */
        bool operator==(const ModelImportSettings& a, const ModelImportSettings& b) noexcept;

        /** @brief The negation of @ref operator==. */
        bool operator!=(const ModelImportSettings& a, const ModelImportSettings& b) noexcept;
    } // namespace Model
} // namespace SushiEngine
```

- [ ] **Step 3: Write the input and output header**

Create `engine/asset/model/include/SushiEngine/model/import_settings_io.hpp` with the Apache
header:

```cpp
#pragma once

/**
 * @file import_settings_io.hpp
 * @brief Reading and writing an asset's `.meta` sidecar.
 *
 * JSON, matching the encoding `CookBakeState` already uses for the project's cooking defaults,
 * so the one-way migration off that document is a field-for-field move rather than a format
 * change.
 */

#include <string>

#include <SushiEngine/model/import_settings.hpp>

namespace SushiEngine
{
    namespace Model
    {
        /**
         * @brief The sidecar path for an asset.
         *
         * Appends `.meta` to the whole path, extension included, so `Car.gltf` and `Car.glb`
         * in one directory keep separate settings.
         *
         * @param asset_path Path to the asset itself.
         * @return The sidecar's path.
         */
        std::string model_import_settings_path(const std::string& asset_path);

        /**
         * @brief Reads an asset's settings, or the defaults when it has none.
         *
         * @param asset_path Path to the asset, not to the sidecar.
         * @param out        Receives the settings; set to the defaults on any failure.
         * @return False when a sidecar exists but could not be read or parsed. A missing
         *         sidecar is not a failure: an asset that has never been configured uses the
         *         defaults, and that is the normal case rather than an error.
         */
        bool load_model_import_settings(const std::string& asset_path, ModelImportSettings& out);

        /**
         * @brief Writes an asset's settings to its sidecar, replacing what was there.
         *
         * @param asset_path Path to the asset, not to the sidecar.
         * @param settings   What to write.
         * @return False when the sidecar could not be opened or written.
         */
        bool save_model_import_settings(const std::string& asset_path,
                                        const ModelImportSettings& settings);
    } // namespace Model
} // namespace SushiEngine
```

- [ ] **Step 4: Write the failing test**

Create `tests/unit/test_model_import_settings_io.cpp` with the Apache header:

```cpp
#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <string>

#include <SushiEngine/model/import_settings_io.hpp>

using SushiEngine::Model::ModelImportSettings;

namespace
{
    // A path in the build directory the test owns and removes, so a run leaves nothing behind.
    std::string scratch_asset()
    {
        return std::string("test_model_import_settings.gltf");
    }

    struct ScratchSidecar
    {
        ~ScratchSidecar()
        {
            std::remove(SushiEngine::Model::model_import_settings_path(scratch_asset()).c_str());
        }
    };
}

TEST(Unit_ModelImportSettingsIO, TheSidecarPathAppendsMetaToTheWholeAssetPath)
{
    EXPECT_EQ(SushiEngine::Model::model_import_settings_path("models/Car.gltf"),
              "models/Car.gltf.meta");
    EXPECT_EQ(SushiEngine::Model::model_import_settings_path("models/Car.glb"),
              "models/Car.glb.meta");
}

TEST(Unit_ModelImportSettingsIO, AnAssetWithNoSidecarLoadsTheDefaultsAndDoesNotFail)
{
    ModelImportSettings settings;
    settings.scale_factor = 42.0f;
    EXPECT_TRUE(SushiEngine::Model::load_model_import_settings(
        "no_such_asset_anywhere.gltf", settings));
    EXPECT_EQ(settings, ModelImportSettings{});
}

TEST(Unit_ModelImportSettingsIO, DefaultedSettingsSurviveAWriteAndRead)
{
    ScratchSidecar cleanup;
    const ModelImportSettings written;
    ASSERT_TRUE(SushiEngine::Model::save_model_import_settings(scratch_asset(), written));

    ModelImportSettings read;
    read.preserve_pivots = false;
    ASSERT_TRUE(SushiEngine::Model::load_model_import_settings(scratch_asset(), read));
    EXPECT_EQ(read, written);
}

TEST(Unit_ModelImportSettingsIO, EveryFieldSurvivesAWriteAndRead)
{
    ScratchSidecar cleanup;
    ModelImportSettings written;
    written.scale_factor = 0.01f;
    written.root_rotation_degrees = SushiEngine::Vector3f{-90.0f, 0.0f, 180.0f};
    written.import_materials = false;
    written.import_lights = false;
    written.import_cameras = false;
    written.preserve_pivots = false;
    written.generate_colliders = true;
    written.cooking.fidelity = 0.75f;
    written.cooking.cook_collision = true;
    written.cooking.cook_soft_body = false;
    written.cooking.cook_node_beam = true;
    written.cooking.static_geometry = false;
    ASSERT_TRUE(SushiEngine::Model::save_model_import_settings(scratch_asset(), written));

    ModelImportSettings read;
    ASSERT_TRUE(SushiEngine::Model::load_model_import_settings(scratch_asset(), read));
    EXPECT_EQ(read, written);
}

TEST(Unit_ModelImportSettingsIO, AnUnsetCookingOverrideStaysUnsetRatherThanBecomingAValue)
{
    ScratchSidecar cleanup;
    ModelImportSettings written;
    written.cooking.fidelity = 0.5f;
    ASSERT_TRUE(SushiEngine::Model::save_model_import_settings(scratch_asset(), written));

    ModelImportSettings read;
    ASSERT_TRUE(SushiEngine::Model::load_model_import_settings(scratch_asset(), read));
    EXPECT_TRUE(read.cooking.fidelity.has_value());
    EXPECT_FALSE(read.cooking.cook_collision.has_value());
    EXPECT_FALSE(read.cooking.cook_soft_body.has_value());
    EXPECT_FALSE(read.cooking.cook_node_beam.has_value());
    EXPECT_FALSE(read.cooking.static_geometry.has_value());
}

TEST(Unit_ModelImportSettingsIO, AMalformedSidecarFailsAndYieldsTheDefaults)
{
    ScratchSidecar cleanup;
    {
        std::ofstream stream(SushiEngine::Model::model_import_settings_path(scratch_asset()));
        stream << "{ this is not json";
    }
    ModelImportSettings read;
    read.scale_factor = 7.0f;
    EXPECT_FALSE(SushiEngine::Model::load_model_import_settings(scratch_asset(), read));
    EXPECT_EQ(read, ModelImportSettings{});
}
```

The last case is the behaviour §8 requires: a `.meta` that fails to parse is reported rather than
silently ignored, and the caller still gets usable defaults.

- [ ] **Step 5: Write the module's CMakeLists**

Create `engine/asset/model/CMakeLists.txt`:

```cmake
# model — what a model asset says about how it is imported, and the decision that turns a glTF
# node graph into a list of entities to create. It links no device and no editor, deliberately:
# every hard decision in the import path is here, so all of it is testable on a machine with no
# GPU and no window.

# physics is public because import_settings.hpp embeds an ImportProfileOverride. gltf is public
# because instantiation_plan.hpp names a GLTFSceneDescription. core is public for Vector3f.
sushiengine_add_module(NAME model LAYER asset
    SOURCES
        source/import_settings_io.cpp
        source/instantiation_plan.cpp
    PUBLIC_DEPENDS core physics gltf
    EXTERNAL_PUBLIC nlohmann_json::nlohmann_json)
```

Confirm the exact spelling of the physics module target and of the nlohmann keyword against a
module that already uses them — `engine/world/authoring/CMakeLists.txt` links nlohmann for
`cook_bake_state.cpp` and is the nearest precedent. If `EXTERNAL_PUBLIC` is not the keyword
`sushiengine_add_module` accepts (see `cmake/Module.cmake:108-117` for its real signature), use the
one it does. `source/instantiation_plan.cpp` is listed now and created in Task 4; create it as an
empty file carrying only the Apache header in this task so the module configures.

- [ ] **Step 6: Implement the settings comparison and the JSON**

Create `engine/asset/model/source/import_settings_io.cpp` with the Apache header. Read
`engine/world/authoring/source/cook_bake_state.cpp:252-298` first and match its JSON idiom.

Write `operator==` comparing every field including each `std::optional` member of `cooking`. Write
the JSON as an object with one key per settings field, writing a `cooking` sub-object that contains
only the optionals that have a value — an absent key is what "not overridden" means, which is why
Step 4's fifth case exists. Read with `value(...)` defaults so a sidecar written by an older build
that lacks a key loads rather than failing.

- [ ] **Step 7: Register the test**

In `tests/CMakeLists.txt`, add `unit/test_model_import_settings_io.cpp` to the executable's source
list, and add beside the existing per-module link lines (near line 619):

```cmake
target_link_libraries(sushiengine_functional_tests PRIVATE sushiengine_model)
```

- [ ] **Step 8: Write the module README**

Create `engine/asset/model/README.md`. Copy the structure of `engine/asset/gltf/README.md`
exactly — a `{#module-model}` anchor on the title, then Tier, Dependencies, Public surface and
Tests sections. The anchor is required: `Doxyfile`'s INPUT includes `engine`, and READMEs collide on
one Doxygen page identifier without it. Then index it in `docs/modules/README.md` beside the `gltf`
entry, link only, no facts.

- [ ] **Step 9: [USER CHECKPOINT] Verify the tests pass**

Hand the user:

```bash
se build
se test --suite functional
```

Expected: `Unit_ModelImportSettingsIO.*` all pass. `se build` failing at configure time with a
`FATAL_ERROR` about a module not in `SUSHIENGINE_MODULE_LAYERS` means Step 1 was missed.

- [ ] **Step 10: Run the checks and commit**

```bash
python tools/documentation/check_documentation_length.py
python tools/documentation/check_module_documentation.py
python tools/layering/check_include_layering.py
git add engine/asset/model cmake/EngineLayers.cmake engine/asset/CMakeLists.txt \
        docs/modules/README.md tests/unit/test_model_import_settings_io.cpp tests/CMakeLists.txt
git commit -m "feat(model): store per-asset import settings in a .meta sidecar"
```

---

## Task 3: Migrate the path-keyed overrides into sidecars

**Files:**
- Modify: `engine/world/authoring/include/SushiEngine/authoring/cook_bake_state.hpp:130-170`
- Modify: `engine/world/authoring/source/cook_bake_state.cpp:252-298`
- Modify: `engine/world/authoring/CMakeLists.txt` (link `sushiengine_model`)
- Modify: `engine/world/authoring/README.md`
- Test: `tests/unit/test_model_import_settings_io.cpp` (extend)

**Interfaces:**
- Consumes: `save_model_import_settings`, `load_model_import_settings` from Task 2.
- Produces: nothing new that later tasks name. This task's product is a behaviour change and a
  project document that no longer carries an `overrides` object.

- [ ] **Step 1: Write the failing test**

Append to `tests/unit/test_model_import_settings_io.cpp`. The migration is a free function so it
can be tested without constructing a `CookBakeState`, which owns a worker thread. Declare it in
`import_settings_io.hpp` beside the others:

```cpp
/**
 * @brief Moves a project document's per-asset cooking overrides into `.meta` sidecars.
 *
 * Runs once, when the project's cooking document is read. Every override whose asset still
 * exists is folded into that asset's sidecar, preserving any settings the sidecar already
 * holds; an override whose asset is gone is dropped, because it was already unreachable and
 * carrying it forward would hide that.
 *
 * @param project_document_path The project's cooking JSON document.
 * @param out_migrated          Receives the asset paths whose sidecars were written.
 * @param out_dropped           Receives the asset paths that no longer exist.
 * @return False when the document exists but could not be read, parsed or rewritten. A
 *         document that does not exist, or that carries no `overrides` object, is not a
 *         failure — it is a project that has nothing to migrate.
 */
bool migrate_cooking_overrides_to_sidecars(const std::string& project_document_path,
                                           std::vector<std::string>& out_migrated,
                                           std::vector<std::string>& out_dropped);
```

The document's real shape is `{"project_default": {...}, "overrides": {"<path>": {...}}}`
(`cook_bake_state.cpp:286-291`). Confirm the override object's own key spelling by reading
`import_profile_override_to_json` in that file before writing the fixture, and use whatever it
actually writes rather than the names guessed here.

```cpp
namespace
{
    const char* PROJECT_DOCUMENT = R"json({
      "project_default": { "parameters": { "fidelity": 0.5 } },
      "overrides": {
        "present_asset.gltf": { "fidelity": 0.9, "cook_soft_body": true },
        "vanished_asset.gltf": { "fidelity": 0.1 }
      }
    })json";

    void write_file(const std::string& path, const char* contents)
    {
        std::ofstream stream(path, std::ios::binary | std::ios::trunc);
        stream << contents;
    }

    std::string read_file(const std::string& path)
    {
        std::ifstream stream(path, std::ios::binary);
        return std::string(std::istreambuf_iterator<char>(stream),
                           std::istreambuf_iterator<char>());
    }
}

TEST(Unit_CookingOverrideMigration, AnOverrideBecomesASidecarAndLeavesTheDocument)
{
    const std::string document = "test_migration_project.json";
    const std::string present = "present_asset.gltf";
    write_file(document, PROJECT_DOCUMENT);
    write_file(present, "");

    std::vector<std::string> migrated;
    std::vector<std::string> dropped;
    ASSERT_TRUE(SushiEngine::Model::migrate_cooking_overrides_to_sidecars(document, migrated,
                                                                         dropped));

    ASSERT_EQ(migrated.size(), 1u);
    EXPECT_EQ(migrated[0], present);
    ASSERT_EQ(dropped.size(), 1u);
    EXPECT_EQ(dropped[0], "vanished_asset.gltf");

    ModelImportSettings settings;
    ASSERT_TRUE(SushiEngine::Model::load_model_import_settings(present, settings));
    ASSERT_TRUE(settings.cooking.fidelity.has_value());
    EXPECT_FLOAT_EQ(*settings.cooking.fidelity, 0.9f);
    ASSERT_TRUE(settings.cooking.cook_soft_body.has_value());
    EXPECT_TRUE(*settings.cooking.cook_soft_body);
    // An override that said nothing about a field must not invent a value for it.
    EXPECT_FALSE(settings.cooking.cook_node_beam.has_value());

    const std::string rewritten = read_file(document);
    EXPECT_EQ(rewritten.find("overrides"), std::string::npos);
    EXPECT_NE(rewritten.find("project_default"), std::string::npos);

    std::remove(document.c_str());
    std::remove(present.c_str());
    std::remove(SushiEngine::Model::model_import_settings_path(present).c_str());
}

TEST(Unit_CookingOverrideMigration, RunningItTwiceChangesNothingTheSecondTime)
{
    const std::string document = "test_migration_idempotent.json";
    const std::string present = "present_asset.gltf";
    write_file(document, PROJECT_DOCUMENT);
    write_file(present, "");

    std::vector<std::string> migrated;
    std::vector<std::string> dropped;
    ASSERT_TRUE(SushiEngine::Model::migrate_cooking_overrides_to_sidecars(document, migrated,
                                                                         dropped));
    const std::string sidecar_after_first =
        read_file(SushiEngine::Model::model_import_settings_path(present));

    migrated.clear();
    dropped.clear();
    ASSERT_TRUE(SushiEngine::Model::migrate_cooking_overrides_to_sidecars(document, migrated,
                                                                         dropped));
    EXPECT_TRUE(migrated.empty());
    EXPECT_TRUE(dropped.empty());
    EXPECT_EQ(read_file(SushiEngine::Model::model_import_settings_path(present)),
              sidecar_after_first);

    std::remove(document.c_str());
    std::remove(present.c_str());
    std::remove(SushiEngine::Model::model_import_settings_path(present).c_str());
}

TEST(Unit_CookingOverrideMigration, AProjectWithNoDocumentIsNotAFailure)
{
    std::vector<std::string> migrated;
    std::vector<std::string> dropped;
    EXPECT_TRUE(SushiEngine::Model::migrate_cooking_overrides_to_sidecars(
        "no_such_project_document.json", migrated, dropped));
    EXPECT_TRUE(migrated.empty());
    EXPECT_TRUE(dropped.empty());
}
```

Add `#include <iterator>` and `#include <vector>` for these cases. The migration must preserve a
sidecar's non-cooking fields, which the idempotence case covers implicitly: the second run reads a
sidecar it wrote and must not disturb it.

- [ ] **Step 2: [USER CHECKPOINT] Verify the test fails**

Hand the user: `se build`
Expected: a link error naming `migrate_cooking_overrides_to_sidecars`.

- [ ] **Step 3: Implement the migration**

In `engine/asset/model/source/import_settings_io.cpp`. Read the document, iterate the `overrides`
object, and for each entry: if the asset path does not exist on disk, record it in `out_dropped` and
skip; otherwise load the asset's existing sidecar, overwrite only its `cooking` member from the
override, save it, and record it in `out_migrated`. When the loop ends, erase the `overrides` key
and rewrite the document. Overwriting only `cooking` is what makes the migration idempotent and what
stops it discarding settings a sidecar already carries.

- [ ] **Step 4: Call it from `CookBakeState::load_profiles`**

In `cook_bake_state.cpp:252-279`, before the existing `overrides` loop, call the migration on
`profile_storage_path_`. Then delete the `overrides` loop and the `overrides` object from
`save_profiles` at `:286-291`, so the document holds `project_default` alone and nothing writes the
key back. Update the Doxygen at `cook_bake_state.hpp:130-170` where it describes overrides living in
this document; the header is documentation and a stale sentence there is a defect.

Report what moved through the existing editor console rather than silently: the migration's two
output vectors exist so a project that had overrides says so once on load.

- [ ] **Step 5: [USER CHECKPOINT] Verify**

```bash
se build
se test --suite functional
```

Expected: `Unit_CookingOverrideMigration.*` pass and no existing cooking test regresses.

- [ ] **Step 6: Update the README and commit**

Update `engine/world/authoring/README.md` where it states that per-asset overrides live in the
project document, and add the changelog entry under `## [Unreleased]` / `Changed` in
`docs/reference/changelog.md`, one bullet, at most 240 characters:

```markdown
- Changed per-asset cooking overrides to live in a `<asset>.meta` sidecar instead of a path-keyed
  object in the project document, so moving or renaming an asset no longer orphans its settings.
```

```bash
python tools/documentation/check_documentation_length.py
git add engine/world/authoring engine/asset/model docs/reference/changelog.md \
        tests/unit/test_model_import_settings_io.cpp
git commit -m "refactor(authoring): move per-asset cooking overrides into .meta sidecars"
```
