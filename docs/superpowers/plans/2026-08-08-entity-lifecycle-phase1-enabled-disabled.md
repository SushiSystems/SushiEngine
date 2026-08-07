# Entity Lifecycle Phase 1 — Enabled/Disabled Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development
> (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use
> checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make an entity's enabled/disabled state real across physics, audio and render — not just
render, as it is today — while keeping the existing render-only `visible` flag's meaning unchanged.

**Architecture:** Add one new field (`Record::enabled`, default `true`) and one hierarchy-walk
helper (`enabled_in_hierarchy`) to `RuntimeSimulation`, expose both through `IWorldEditor`, and gate
every consumer — `extract()`'s nine render call sites, physics's two independent gather points, and
the editor's live audio poll — on that single source of truth. No scene-file migration: `enabled`
is additive and defaults to the value that reproduces today's behavior exactly.

**Tech Stack:** C++17, the existing `RuntimeSimulation`/`IWorldEditor` seam, GoogleTest
(`tests/CMakeLists.txt`), Dear ImGui (editor panels).

## Global Constraints

- **The spec is `docs/design/entity_lifecycle_system.md` §4.** Every task below implements one
  piece of that section; do not deviate from its data-model decisions (two independent flags,
  velocity reset on re-enable, "stop immediately" for audio) without going back to the user.
- **Never invoke the build system directly, and never run `se build`/`se test`/`se editor`
  yourself.** This machine cannot run the build. Every step that would normally "run the test" is
  instead phrased as: hand the exact command to the user and ask them to run it and paste back the
  output. Do not claim a test passes without that output in hand.
- **SOLID/encapsulation is priority 1, performance is priority 2** (`CLAUDE.md`). Do not duplicate
  the hierarchy walk outside `RuntimeSimulation` — `audio_editor_system.cpp` must call the new
  `IWorldEditor::enabled_in_hierarchy`, not reimplement it.
- **Documentation ships in the same change as the code** (`docs/CONTRIBUTING.md` §5): the changelog,
  the architecture chapter, and the module README all update in Task 6, not later.
- **100-column line width** and this codebase's existing comment style (Apache license header
  block, `@brief` Doxygen tags, "Unity's `X`" cross-references) apply to every new file.
- **Commit style:** `type(scope): summary`, one commit per task, following this repo's existing
  history (see `git log --oneline -20` for recent examples).
- **No backward-compatibility shims.** `enabled` is a new field with an in-class default; there is
  no old data to migrate and no toggle to gate the new behavior behind.

---

### Task 1: Data model and API surface

**Files:**
- Modify: `engine/world/simulation/source/runtime_simulation.cpp`
  - `Record` struct (currently lines 2045-2117+)
  - New private helper beside `visible_in_hierarchy` (currently lines 2721-2733)
  - `visible()` (currently lines 287-291)
  - `set_visible()` (currently lines 1969-1977)
- Modify: `engine/world/simulation/include/SushiEngine/simulation/simulation.hpp`
  - `IWorldEditor` getter block (`visible` declared at line 1209)
  - `IWorldEditor` setter block (`set_visible` declared at line 1406)
- Create: `tests/integration/test_entity_lifecycle.cpp`
- Modify: `tests/CMakeLists.txt` (register the new test file near line 559)

**Interfaces:**
- Produces: `Record::enabled` (bool, default `true`); private
  `bool enabled_in_hierarchy(const Record*) const noexcept`; public
  `IWorldEditor::enabled(EntityId) const noexcept`,
  `IWorldEditor::set_enabled(EntityId, bool)`,
  `IWorldEditor::enabled_in_hierarchy(EntityId) const noexcept`.
- Consumes: existing `RuntimeSimulation::find(EntityId)`, `Record::parent`, `NULL_ENTITY`,
  `records_.size()` (all already in scope where the new code is added).

- [ ] **Step 1: Add the `Record::enabled` field**

  In `engine/world/simulation/source/runtime_simulation.cpp`, in the `Record` struct, insert a new
  field immediately after `bool is_camera = false;` (currently line 2051) and before
  `EntityId parent = NULL_ENTITY;` (currently line 2052):

  ```cpp
                        bool is_camera = false;
                        // Unity's `activeSelf`: this entity's own on/off switch. Distinct from
                        // `visible` (a render-only local flag, see below) — disabling an entity
                        // stops physics, audio and render for it and its whole subtree
                        // (`enabled_in_hierarchy`), where `visible=false` only stops its own
                        // render contribution.
                        bool enabled = true;
                        EntityId parent = NULL_ENTITY;
  ```

  This position is deliberate, not arbitrary: every `Record{entity, display_name, true, false}`-style
  aggregate initializer in this file positionally sets at most the first five fields (`entity`,
  `name`, `visible`, `animated`, `is_camera` — confirmed by
  `records_.emplace(id, Record{entity, display_name, true, false, true});` at line 1839, the deepest
  positional initializer in the file). Inserting `enabled` at position 6, after `is_camera`, means
  none of those call sites need to change — every one of them gets `enabled = true` from the
  in-class default, which is the correct default for every one of them (every entity created today
  is, and must remain, enabled).

- [ ] **Step 2: Add the `enabled_in_hierarchy(const Record*)` private helper**

  In the same file, immediately after `visible_in_hierarchy` (currently lines 2721-2733), add:

  ```cpp
                    /**
                     * @brief Whether @p record and every ancestor above it are enabled.
                     *
                     * Unity's `activeInHierarchy`: physics, audio and render all gate existence
                     * on this, not on @ref Record::enabled alone, which is local to one entity.
                     * (Task 2 of this change retires the render-only `visible_in_hierarchy` this
                     * was modeled on and folds its callers into this one instead.)
                     */
                    bool enabled_in_hierarchy(const Record* record) const noexcept
                    {
                        std::size_t guard = records_.size() + 1;
                        for (const Record* current = record; current != nullptr && guard > 0;
                             --guard)
                        {
                            if (!current->enabled)
                                return false;
                            current = current->parent == NULL_ENTITY ? nullptr
                                                                     : find(current->parent);
                        }
                        return true;
                    }
  ```

- [ ] **Step 3: Add `enabled()` and the public `enabled_in_hierarchy(EntityId)` overload**

  In the same file, immediately after `visible()` (currently lines 287-291):

  ```cpp
                    bool visible(EntityId id) const noexcept override
                    {
                        const Record* record = find(id);
                        return record != nullptr && record->visible;
                    }

                    bool enabled(EntityId id) const noexcept override
                    {
                        const Record* record = find(id);
                        return record != nullptr && record->enabled;
                    }

                    bool enabled_in_hierarchy(EntityId id) const noexcept override
                    {
                        const Record* record = find(id);
                        return record != nullptr && enabled_in_hierarchy(record);
                    }
  ```

  (`enabled_in_hierarchy` is overloaded: the `EntityId` version above is the public one declared on
  `IWorldEditor`; the `const Record*` version from Step 2 is the private walk it and `extract()`
  both call. This mirrors how `visible()` and `visible_in_hierarchy` are already two different
  functions today, just given matching names on both sides of the seam.)

- [ ] **Step 4: Add `set_enabled()`**

  In the same file, immediately after `set_visible()` (currently lines 1969-1977):

  ```cpp
                    void set_visible(EntityId id, bool value) override
                    {
                        Record* record = find(id);
                        if (record != nullptr)
                        {
                            record->visible = value;
                            extract();
                        }
                    }

                    void set_enabled(EntityId id, bool value) override
                    {
                        Record* record = find(id);
                        if (record != nullptr)
                        {
                            record->enabled = value;
                            extract();
                        }
                    }
  ```

  (`extract()` here only refreshes the render snapshot immediately, matching `set_visible`'s
  existing behavior. Physics and audio pick up the new `enabled` state on their own next read —
  physics on the next fixed tick's `physics_source_entities()` call, audio on the next
  `AudioEditorSystem::update()` — Tasks 3 and 4 wire those reads; no push is needed from here.)

- [ ] **Step 5: Declare the three methods on `IWorldEditor`**

  In `engine/world/simulation/include/SushiEngine/simulation/simulation.hpp`, immediately after
  `virtual bool visible(EntityId id) const noexcept = 0;` (currently line 1209):

  ```cpp
                /** @brief Whether the entity is drawn. */
                virtual bool visible(EntityId id) const noexcept = 0;

                /** @brief Whether the entity's own flag is set (Unity's `activeSelf`). */
                virtual bool enabled(EntityId id) const noexcept = 0;

                /**
                 * @brief Whether the entity and every ancestor above it are enabled (Unity's
                 * `activeInHierarchy`). What physics, audio and render actually gate on — not
                 * @ref enabled alone, which is local to this one entity.
                 */
                virtual bool enabled_in_hierarchy(EntityId id) const noexcept = 0;
  ```

  Immediately after `virtual void set_visible(EntityId id, bool visible) = 0;` (currently line
  1406):

  ```cpp
                /** @brief Sets whether the entity is drawn. */
                virtual void set_visible(EntityId id, bool visible) = 0;

                /** @brief Sets the entity's own flag (Unity's `activeSelf`). */
                virtual void set_enabled(EntityId id, bool enabled) = 0;
  ```

- [ ] **Step 6: Write the failing test file**

  Create `tests/integration/test_entity_lifecycle.cpp`:

  ```cpp
  /**************************************************************************/
  /* test_entity_lifecycle.cpp                                              */
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

  // entity_lifecycle_system.md Phase 1: `enabled` (Unity's `activeSelf`/`activeInHierarchy`) is a
  // real, hierarchical on/off switch, distinct from the render-only local `visible` flag. This
  // file covers the flag's own semantics; render, physics and audio gating on it are covered in
  // their own tests below and, for physics, in the entities-and-transforms cases further down.

  #include <gtest/gtest.h>

  #include <SushiEngine/simulation/simulation.hpp>

  using namespace SushiEngine;
  using namespace SushiEngine::Simulation;

  namespace
  {
      void clear_world(IWorldEditor& world)
      {
          for (const EntityId id : world.entities())
              world.destroy(id);
      }

      const RenderInstance* find_instance(const RenderScene& scene, EntityId id)
      {
          for (const RenderInstance& instance : scene.instances)
              if (instance.id == id)
                  return &instance;
          return nullptr;
      }
  } // namespace

  TEST(Integration_EntityLifecycle, EnabledDefaultsToTrueAndIsSelfOnly)
  {
      const auto simulation = create_simulation();
      ASSERT_NE(simulation, nullptr);
      IWorldEditor& world = simulation->world();
      clear_world(world);

      const EntityId id = world.create("Widget");
      EXPECT_TRUE(world.enabled(id));
      EXPECT_TRUE(world.enabled_in_hierarchy(id));

      world.set_enabled(id, false);
      EXPECT_FALSE(world.enabled(id));
      EXPECT_FALSE(world.enabled_in_hierarchy(id));
  }

  TEST(Integration_EntityLifecycle, DisablingAnAncestorDisablesTheWholeSubtree)
  {
      const auto simulation = create_simulation();
      ASSERT_NE(simulation, nullptr);
      IWorldEditor& world = simulation->world();
      clear_world(world);

      const EntityId root = world.create("Root");
      const EntityId child = world.create("Child");
      world.set_parent(child, root);

      EXPECT_TRUE(world.enabled_in_hierarchy(child));
      world.set_enabled(root, false);
      EXPECT_FALSE(world.enabled_in_hierarchy(child))
          << "a disabled ancestor must disable everything under it";
      // The child's own flag is untouched -- only the hierarchy view changed.
      EXPECT_TRUE(world.enabled(child));
  }

  TEST(Integration_EntityLifecycle, ReEnablingAParentDoesNotAdoptAChildsOwnDisabledFlag)
  {
      const auto simulation = create_simulation();
      ASSERT_NE(simulation, nullptr);
      IWorldEditor& world = simulation->world();
      clear_world(world);

      const EntityId root = world.create("Root");
      const EntityId child = world.create("Child");
      world.set_parent(child, root);
      world.set_enabled(child, false);

      world.set_enabled(root, false);
      world.set_enabled(root, true);

      EXPECT_TRUE(world.enabled_in_hierarchy(root));
      EXPECT_FALSE(world.enabled_in_hierarchy(child))
          << "re-enabling the parent silently re-enabled a child the author had disabled";
  }
  ```

- [ ] **Step 7: Register the test file**

  In `tests/CMakeLists.txt`, immediately after the `integration/test_shape_render_extraction.cpp`
  entry (currently line 559), add:

  ```cmake
      # entity_lifecycle_system.md Phase 1: `enabled`/`activeInHierarchy` as a real, hierarchical
      # on/off switch distinct from the render-only local `visible` flag.
      integration/test_entity_lifecycle.cpp
  ```

- [ ] **Step 8: Ask the user to build and run the new test**

  Ask the user to run:

  ```
  se build && se test --suite functional --filter 'Integration_EntityLifecycle.*'
  ```

  Expected: the build succeeds (Steps 1-5 already added everything the test calls) and all three
  cases in `Integration_EntityLifecycle` pass. If the build fails, re-read Steps 1-5 for a typo or
  a positional-initializer site this plan missed — do not proceed to Task 2 until the user confirms
  a clean pass.

- [ ] **Step 9: Commit**

  ```bash
  git add engine/world/simulation/source/runtime_simulation.cpp \
          engine/world/simulation/include/SushiEngine/simulation/simulation.hpp \
          tests/integration/test_entity_lifecycle.cpp tests/CMakeLists.txt
  git commit -m "feat(sim): add a real enabled/activeInHierarchy flag

Distinct from the existing render-only visible flag. Nothing gates on
it yet -- Tasks 2-4 wire render, physics and audio."
  ```

---

### Task 2: Render gate

**Files:**
- Modify: `engine/world/simulation/source/runtime_simulation.cpp`
  - Retire `visible_in_hierarchy` (currently lines 2721-2733)
  - Nine `extract()` call sites (currently lines 3772, 3798, 3872, 3913, 3953, 3980, 4247, 4280,
    4422)
- Modify: `tests/integration/test_shape_render_extraction.cpp` (remove the two hierarchy tests,
  currently lines 93-144)
- Modify: `tests/integration/test_entity_lifecycle.cpp` (add the migrated and new render-gate
  tests)

**Interfaces:**
- Consumes: `enabled_in_hierarchy(const Record*)` and `Record::visible`, both from Task 1.
- Produces: nothing new for later tasks — this task only changes what `extract()` reads.

- [ ] **Step 1: Add the render-gate tests (failing first)**

  Append to `tests/integration/test_entity_lifecycle.cpp`, after the three tests Task 1 added:

  ```cpp
  TEST(Integration_EntityLifecycle, DisablingAParentHidesItsRenderedDescendants)
  {
      const auto simulation = create_simulation();
      ASSERT_NE(simulation, nullptr);
      IWorldEditor& world = simulation->world();
      clear_world(world);

      // The shape an imported model has: a bare pivot root over the part that actually draws.
      const EntityId root = world.create("Model");
      const EntityId part = world.create_box("Part");
      ASSERT_NE(root, NULL_ENTITY);
      ASSERT_NE(part, NULL_ENTITY);
      world.set_parent(part, root);
      simulation->tick(simulation->fixed_dt_seconds());
      ASSERT_NE(find_instance(simulation->render_scene(), part), nullptr);

      // Unity's `activeInHierarchy`: disabling the root has to hide the model. Gating on the
      // part's own flag alone would leave every part of an imported model drawing after its
      // root was switched off, which is the whole model still on screen.
      world.set_enabled(root, false);
      simulation->tick(simulation->fixed_dt_seconds());
      EXPECT_EQ(find_instance(simulation->render_scene(), part), nullptr)
          << "the child kept drawing after its parent was disabled";
  }

  TEST(Integration_EntityLifecycle, ReEnablingAParentLeavesAnIndividuallyDisabledChildDisabled)
  {
      const auto simulation = create_simulation();
      ASSERT_NE(simulation, nullptr);
      IWorldEditor& world = simulation->world();
      clear_world(world);

      const EntityId root = world.create("Model");
      const EntityId shown = world.create_box("Shown");
      const EntityId hidden = world.create_box("Hidden");
      world.set_parent(shown, root);
      world.set_parent(hidden, root);
      world.set_enabled(hidden, false);

      // The reason this is a walk and not a write-through on `set_enabled`: the child's own
      // flag is its own. Toggling the parent off and back on must not adopt its children.
      world.set_enabled(root, false);
      world.set_enabled(root, true);
      simulation->tick(simulation->fixed_dt_seconds());

      EXPECT_NE(find_instance(simulation->render_scene(), shown), nullptr);
      EXPECT_EQ(find_instance(simulation->render_scene(), hidden), nullptr)
          << "re-enabling the parent silently re-enabled a child the author had disabled";
      EXPECT_TRUE(world.enabled(root));
      EXPECT_FALSE(world.enabled(hidden));
  }

  TEST(Integration_EntityLifecycle, VisibleIsLocalOnlyAndDoesNotCascade)
  {
      const auto simulation = create_simulation();
      ASSERT_NE(simulation, nullptr);
      IWorldEditor& world = simulation->world();
      clear_world(world);

      const EntityId root = world.create("Model");
      const EntityId part = world.create_box("Part");
      world.set_parent(part, root);

      // `visible` is the render-only local flag (Unity's Renderer.enabled): turning it off on
      // the root must not touch the child's own render contribution, unlike `enabled`, which
      // does cascade (see DisablingAParentHidesItsRenderedDescendants above).
      world.set_visible(root, false);
      simulation->tick(simulation->fixed_dt_seconds());
      EXPECT_NE(find_instance(simulation->render_scene(), part), nullptr)
          << "a parent's local visible flag must not hide a child that has its own";
  }

  TEST(Integration_EntityLifecycle, DisabledHidesEvenWhenVisibleIsTrue)
  {
      const auto simulation = create_simulation();
      ASSERT_NE(simulation, nullptr);
      IWorldEditor& world = simulation->world();
      clear_world(world);

      const EntityId id = world.create_box("Box");
      ASSERT_TRUE(world.visible(id));
      world.set_enabled(id, false);
      simulation->tick(simulation->fixed_dt_seconds());

      EXPECT_EQ(find_instance(simulation->render_scene(), id), nullptr)
          << "enabled=false must hide rendering even though visible is still true";
  }
  ```

  Ask the user to build and run
  `se test --suite functional --filter 'Integration_EntityLifecycle.*'`. Expected: the four new
  cases FAIL (extract() still gates on `visible_in_hierarchy`, which does not know about `enabled`
  at all yet) while the three from Task 1 keep passing.

- [ ] **Step 2: Remove the two tests these four replace**

  In `tests/integration/test_shape_render_extraction.cpp`, delete
  `TEST(Integration_ShapeRenderExtraction, HidingAParentHidesItsDescendants)` and
  `TEST(Integration_ShapeRenderExtraction, ShowingAParentLeavesAnIndividuallyHiddenChildHidden)`
  (currently lines 93-144 — everything from the `TEST(Integration_ShapeRenderExtraction,
  HidingAParentHidesItsDescendants)` line through the closing brace of
  `ShowingAParentLeavesAnIndividuallyHiddenChildHidden`). They tested `set_visible` cascading
  through the hierarchy, which was `visible_in_hierarchy`'s job; that job now belongs to `enabled`
  and Step 1's rewritten versions, and `visible` no longer cascades at all (by design — see
  `docs/design/entity_lifecycle_system.md` §4.1). Leaving the old tests in place would make them
  fail as soon as Step 4 below rewires `extract()`, asserting behavior this change deliberately
  removes.

- [ ] **Step 3: Retire `visible_in_hierarchy`**

  In `engine/world/simulation/source/runtime_simulation.cpp`, delete the `visible_in_hierarchy`
  method (currently lines 2721-2733, immediately before `enabled_in_hierarchy` which Task 1 added
  beside it). Nothing calls it once Step 4 rewires its nine call sites, and leaving unused dead code
  behind fails this codebase's own maintainability bar.

- [ ] **Step 4: Rewire the nine `extract()` call sites**

  In the same file, replace every occurrence of the exact substring `!visible_in_hierarchy(record)`
  with `!enabled_in_hierarchy(record) || !record->visible`. This substring occurs at exactly nine
  locations before this change (verify with a search for `visible_in_hierarchy` — the count must be
  nine, all in `extract()`'s mesh/cloth/soft-body/vehicle-shell/particle/particle-emitter/light/
  decal/crowd instancing loops) and zero after. Each site keeps its existing `||` chain shape; the
  replacement is a straight substitution, e.g. at the mesh-instancing site (currently lines
  3771-3773):

  ```cpp
                          if (!record->has_shape || !record->has_renderer ||
                              !enabled_in_hierarchy(record) || !record->visible)
                              continue;
  ```

  and at the particle-emitter site that has a trailing condition after it (currently lines
  3979-3981):

  ```cpp
                          if (record == nullptr || !record->has_particle_emitter ||
                              !enabled_in_hierarchy(record) || !record->visible ||
                              !world_.alive(record->entity))
                              continue;
  ```

  Apply the same substitution at the remaining seven sites (cloth, soft body, vehicle shell,
  cosmetic particle emitter, light, decal, crowd), each keeping its own surrounding `||` terms
  unchanged and only swapping the `visible_in_hierarchy` term for the two new terms.

- [ ] **Step 5: Ask the user to build and run the full render-extraction and entity-lifecycle
  suites**

  ```
  se build && se test --suite functional --filter 'Integration_EntityLifecycle.*:Integration_ShapeRenderExtraction.*'
  ```

  Expected: every case in both suites passes — the four new `Integration_EntityLifecycle` cases
  from Step 1, the three from Task 1, and the (now two fewer) `Integration_ShapeRenderExtraction`
  cases.

- [ ] **Step 6: Commit**

  ```bash
  git add engine/world/simulation/source/runtime_simulation.cpp \
          tests/integration/test_entity_lifecycle.cpp \
          tests/integration/test_shape_render_extraction.cpp
  git commit -m "feat(sim): gate render extraction on enabled, not just visible

extract()'s nine instancing loops now skip a subtree via
enabled_in_hierarchy the way they used to skip it via visible alone;
visible keeps its old meaning as a local, non-cascading render-only
flag. Migrates the two hierarchy tests that exercised the old,
conflated behavior."
  ```

---

### Task 3: Physics gate

**Files:**
- Modify: `engine/world/simulation/source/runtime_simulation.cpp`
  - `physics_source_entities()` (currently lines 3409-3447)
  - `gather_vehicle_descriptions()` (currently lines 2328-2351)
- Modify: `tests/integration/test_entity_lifecycle.cpp`

**Interfaces:**
- Consumes: `enabled_in_hierarchy(const Record*)` (Task 1); `IWorldEditor::physics_body_debug`
  (existing, declared at `simulation.hpp:2198`, implemented at `runtime_simulation.cpp:1619-1627`);
  `RigidDebugState` (existing, `simulation.hpp:781-802`).
- Produces: nothing new for later tasks.

- [ ] **Step 1: Write the failing test**

  Append to `tests/integration/test_entity_lifecycle.cpp`:

  ```cpp
  TEST(Integration_EntityLifecycle, DisablingAnEntityRemovesItsRigidBodyFromPhysics)
  {
      const auto simulation = create_simulation();
      ASSERT_NE(simulation, nullptr);
      IWorldEditor& world = simulation->world();
      clear_world(world);

      const EntityId id = world.create_box("Box");
      EntityTransform transform = world.transform(id);
      transform.position = Vector3{0, 5, 0};
      world.set_transform(id, transform);
      PhysicsBodyParameters body;
      body.density = Scalar(1000);
      world.set_has_physics_body(id, true);
      world.set_physics_body_parameters(id, body);

      simulation->tick(simulation->fixed_dt_seconds());
      RigidDebugState state;
      ASSERT_TRUE(world.physics_body_debug(id, state))
          << "an enabled entity with a physics body must be tracked by the solver";

      world.set_enabled(id, false);
      simulation->tick(simulation->fixed_dt_seconds());
      EXPECT_FALSE(world.physics_body_debug(id, state))
          << "a disabled entity's rigid body must be removed from the solver";

      world.set_enabled(id, true);
      simulation->tick(simulation->fixed_dt_seconds());
      EXPECT_TRUE(world.physics_body_debug(id, state))
          << "re-enabling must re-admit the body to the solver";
  }
  ```

  Note for the implementer: this task deliberately does not add a `gather_vehicle_descriptions`
  test. Proving it needs the cooked-`.sushinodebeam` harness `tests/integration/
  test_vehicle_component.cpp` already builds, which is a heavier setup than this one field-toggle
  deserves its own copy of; Step 3 below applies the identical one-line filter there, and its
  correctness is verifiable by inspection against Step 2's version in the same diff. Say this
  plainly if asked what covers vehicles — do not claim test coverage that is not there.

  Ask the user to run `se build && se test --suite functional --filter
  'Integration_EntityLifecycle.DisablingAnEntityRemovesItsRigidBodyFromPhysics'`. Expected: FAIL —
  `physics_body_debug` still returns `true` after `set_enabled(id, false)` because
  `physics_source_entities()` does not consult `enabled` yet.

- [ ] **Step 2: Filter `physics_source_entities()`**

  In `engine/world/simulation/source/runtime_simulation.cpp`, in `physics_source_entities()`
  (currently lines 3409-3447), change the existing skip condition at line 3416 from:

  ```cpp
                          if (record == nullptr || !world_.alive(record->entity))
                              continue;
  ```

  to:

  ```cpp
                          if (record == nullptr || !world_.alive(record->entity) ||
                              !enabled_in_hierarchy(record))
                              continue;
  ```

  This one change covers every consumer of `physics_source_entities()`: `gather_rigid_descriptions`
  (line 3392, via `extract_rigid_bodies`), `gather_static_planes` (line 3462, via
  `extract_static_planes`), and both `gather_soft_body_descriptions` and
  `gather_cloth_descriptions` (lines 3486 and 3512, confirmed to route through the same source at
  implementation time — if either turns out not to, apply the identical filter directly in that
  function's own loop instead, following the same pattern as Step 3 below).

- [ ] **Step 3: Filter `gather_vehicle_descriptions()`**

  In the same file, in `gather_vehicle_descriptions()` (currently lines 2328-2351), change the
  existing skip condition at lines 2334-2336 from:

  ```cpp
                            Record* record = find(id);
                            if (record == nullptr || !record->has_vehicle ||
                                !world_.alive(record->entity))
                                continue;
  ```

  to:

  ```cpp
                            Record* record = find(id);
                            if (record == nullptr || !record->has_vehicle ||
                                !world_.alive(record->entity) || !enabled_in_hierarchy(record))
                                continue;
  ```

  This is a second, independent filter point because `gather_vehicle_descriptions` does not route
  through `physics_source_entities()` — Step 2 alone does not cover vehicles.

- [ ] **Step 4: Ask the user to build and run the test**

  ```
  se build && se test --suite functional --filter 'Integration_EntityLifecycle.*'
  ```

  Expected: all cases pass, including the new one from Step 1.

- [ ] **Step 5: Commit**

  ```bash
  git add engine/world/simulation/source/runtime_simulation.cpp \
          tests/integration/test_entity_lifecycle.cpp
  git commit -m "feat(sim): gate physics gathering on enabled_in_hierarchy

physics_source_entities() and gather_vehicle_descriptions() (a
separate filter point) both skip a disabled subtree now. set_rigid_bodies
is already a diff against the incoming description list, so no change
was needed inside PhysicsSimulation itself: a disabled body is removed
the same way a genuinely-gone one always was, and re-enabling re-adds it
at rest (no velocity field exists to preserve, and this was confirmed
as an accepted tradeoff -- see entity_lifecycle_system.md §4.2)."
  ```

---

### Task 4: Audio gate

**Files:**
- Modify: `applications/editor/source/audio/audio_editor_system.cpp` (the emitter loop, currently
  lines 183-211)

**Interfaces:**
- Consumes: `IWorldEditor::enabled_in_hierarchy(EntityId)` (Task 1, already on the interface this
  file already includes via `Simulation::IWorldEditor&`).
- Produces: nothing new for later tasks.

- [ ] **Step 1: Filter the emitter loop**

  In `applications/editor/source/audio/audio_editor_system.cpp`, in the loop building
  `snapshot.emitters` (currently lines 183-211), change:

  ```cpp
              for (Simulation::EntityId id : ids)
              {
                  if (!world.has_audio_emitter(id))
                      continue;
  ```

  to:

  ```cpp
              for (Simulation::EntityId id : ids)
              {
                  if (!world.has_audio_emitter(id) || !world.enabled_in_hierarchy(id))
                      continue;
  ```

  `AudioEditorSystem::update` (which owns this loop) rebuilds the whole `Audio::SceneSnapshot` and
  calls `scene_.apply(snapshot)` every call (currently line 214) — an emitter that drops out of the
  snapshot on the next `update()` stops immediately, using the existing rebuild-and-apply mechanism
  rather than new machinery. This produces the "stop immediately" behavior confirmed with the user
  in brainstorming.

- [ ] **Step 2: Note on test coverage**

  No test target links `applications/editor` source (confirmed: nothing under
  `applications/editor/` is referenced anywhere in `tests/CMakeLists.txt`), so there is no
  precedent gtest harness for `AudioEditorSystem` to extend, and building one is out of scope for
  this task — it would mean standing up a new test target for a class that owns a live SDL audio
  device, which is a bigger decision than this one-line filter warrants. This is a manual-only
  verification step; say so plainly rather than claiming automated coverage that does not exist.

  Ask the user, once they have the editor built (Task 5 also needs a build, so this can be
  batched with that task's manual check): place an Audio Emitter on an entity, confirm it is
  audible in the editor, then uncheck the entity's Enabled checkbox (added in Task 5) and confirm
  the sound stops immediately.

- [ ] **Step 3: Commit**

  ```bash
  git add applications/editor/source/audio/audio_editor_system.cpp
  git commit -m "feat(editor): stop a disabled entity's audio emitter

The editor's live audio poll now skips a subtree enabled_in_hierarchy
says is off, through the IWorldEditor seam rather than reimplementing
the hierarchy walk in this translation unit."
  ```

---

### Task 5: Editor UI

**Files:**
- Modify: `applications/editor/source/scene/inspector_panel.cpp` (header checkbox, currently lines
  283-296)
- Modify: `applications/editor/source/scene/hierarchy_panel.cpp` (row rendering, currently lines
  112-144)

**Interfaces:**
- Consumes: `IWorldEditor::enabled`/`set_enabled`/`enabled_in_hierarchy`/`visible`/`set_visible`
  (all already on the interface after Task 1).
- Produces: nothing new for later tasks — this is the last task that touches production code.

- [ ] **Step 1: Split the Inspector header checkbox**

  In `applications/editor/source/scene/inspector_panel.cpp`, replace the existing single checkbox
  block (currently lines 283-296):

  ```cpp
              bool visible = world->visible(id);
              const bool visible_mixed = selection_mixed(
                  targets, [&](EntityId e) { return world->visible(e); }, visible);
              if (visible_mixed)
                  ImGui::PushItemFlag(ImGuiItemFlags_MixedValue, true);
              const bool visible_changed = ImGui::Checkbox("##visible", &visible);
              if (visible_mixed)
                  ImGui::PopItemFlag();
              if (visible_changed)
              {
                  context.history.record(*world);
                  for (const EntityId target : targets)
                      world->set_visible(target, visible);
              }
              ImGui::SameLine();
  ```

  with two checkboxes — `enabled` in the header's original position (Unity's `activeSelf` slot),
  `visible` beside it:

  ```cpp
              bool enabled = world->enabled(id);
              const bool enabled_mixed = selection_mixed(
                  targets, [&](EntityId e) { return world->enabled(e); }, enabled);
              if (enabled_mixed)
                  ImGui::PushItemFlag(ImGuiItemFlags_MixedValue, true);
              const bool enabled_changed = ImGui::Checkbox("##enabled", &enabled);
              if (enabled_mixed)
                  ImGui::PopItemFlag();
              if (enabled_changed)
              {
                  context.history.record(*world);
                  for (const EntityId target : targets)
                      world->set_enabled(target, enabled);
              }
              ImGui::SameLine();

              bool visible = world->visible(id);
              const bool visible_mixed = selection_mixed(
                  targets, [&](EntityId e) { return world->visible(e); }, visible);
              if (visible_mixed)
                  ImGui::PushItemFlag(ImGuiItemFlags_MixedValue, true);
              const bool visible_changed = ImGui::Checkbox("##visible", &visible);
              if (visible_mixed)
                  ImGui::PopItemFlag();
              if (visible_changed)
              {
                  context.history.record(*world);
                  for (const EntityId target : targets)
                      world->set_visible(target, visible);
              }
              ImGui::SameLine();
  ```

  Two checkboxes rather than nesting "Visible" inside the Renderer component section, because
  `visible` already gates every visual representation an entity can carry (mesh, cloth,
  soft/vehicle shell, particles, lights, decals), not just the Renderer component specifically — it
  is an entity-level flag and its placement should say so (see
  `docs/design/entity_lifecycle_system.md` §4.5).

- [ ] **Step 2: Dim disabled rows in the Hierarchy panel**

  In `applications/editor/source/scene/hierarchy_panel.cpp`, in `draw_entity_node` (currently lines
  112-144), wrap the `TreeNodeEx` call (currently line 144) with a conditional style push:

  ```cpp
                  ImGuiTreeNodeFlags flags =
                      ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_SpanAvailWidth;
                  if (children.empty())
                      flags |= ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen;
                  if (is_selected(context, id))
                      flags |= ImGuiTreeNodeFlags_Selected;

                  const bool row_disabled = !world->enabled_in_hierarchy(id);
                  if (row_disabled)
                      ImGui::PushStyleColor(ImGuiCol_Text,
                                            ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
                  const bool open = ImGui::TreeNodeEx(entity_name.c_str(), flags);
                  if (row_disabled)
                      ImGui::PopStyleColor();
  ```

  Selection, drag-and-drop and the context menu are all driven by `ImGui::IsItemClicked`/
  `IsItemDeactivated`/`BeginDragDropSource` etc. immediately following this block (currently lines
  145-184), none of which read text color — this is a visual indicator only, and every existing
  interaction keeps working on a dimmed row exactly as it does on a normal one.

- [ ] **Step 3: Ask the user to build and manually verify**

  Ask the user to run `se editor` and:
  1. Select an entity, confirm the Inspector header now shows two checkboxes side by side.
  2. Uncheck "Enabled" on a parent with children; confirm the children stop rendering in the Scene
     view and their Hierarchy rows (and the parent's) turn grey.
  3. Re-check "Enabled" on the parent; confirm a child that was individually disabled beforehand
     stays disabled and grey, while its siblings come back.
  4. Uncheck "Visible" (not "Enabled") on an entity with a physics body; confirm it disappears from
     the Scene view but keeps colliding (e.g. drop another object on top of it and see it stop,
     rather than fall through).
  5. Complete Task 4 Step 2's audio check in the same session.

- [ ] **Step 4: Commit**

  ```bash
  git add applications/editor/source/scene/inspector_panel.cpp \
          applications/editor/source/scene/hierarchy_panel.cpp
  git commit -m "feat(editor): expose enabled/activeInHierarchy in the Inspector and Hierarchy

The Inspector header checkbox now sets enabled (Unity's activeSelf
slot); a second checkbox beside it sets the existing render-only
visible flag. Hierarchy rows dim when enabled_in_hierarchy is false,
matching Unity/Unreal's inactive-row convention."
  ```

---

### Task 6: Documentation

**Files:**
- Modify: `docs/architecture/domain-physics.md` (§1.5, currently ends at line 377)
- Modify: `engine/world/simulation/README.md` (Tests section, lines 65-67; Further reading
  section, lines 71-80)
- Modify: `docs/reference/changelog.md` (`## [Unreleased]` → `### Added`, currently starting line
  12)

**Interfaces:** None — this task changes no code.

- [ ] **Step 1: Document the feature in the architecture chapter**

  In `docs/architecture/domain-physics.md`, at the end of §1.5 "Editor authoring: cloth, UI, and
  custom components" (currently ending at line 377 with "...alongside the other optional
  components."), append a new bulleted paragraph in the same style as the section's existing
  "Cloth as an object"/"UI (Canvas + elements)"/"Custom (script) components" entries:

  ```markdown

  **Enabled/disabled lifecycle.** `RuntimeSimulation::Record::enabled` (default `true`) is Unity's
  `activeSelf`: a real, hierarchical on/off switch, distinct from `visible`, which keeps its
  original meaning as a local, non-cascading, render-only flag (Unity's `Renderer.enabled`).
  `enabled_in_hierarchy` walks an entity's ancestor chain the same way `visible_in_hierarchy` used
  to, over the new flag instead. Everything that gates on it uses that one walk: `extract()`'s
  mesh/cloth/soft-body/vehicle-shell/particle/light/decal/crowd instancing loops (combined with the
  local `visible` check), `physics_source_entities()` and `gather_vehicle_descriptions()` (a
  second, independent filter point — see [the entity lifecycle design](../design/entity_lifecycle_system.md)
  §1 for why), and the editor's live audio poll in `AudioEditorSystem::update` through the
  `IWorldEditor` seam. A disabled entity's rigid body is removed from the physics world the same
  way `set_rigid_bodies`' existing diff already removes a genuinely-destroyed one, and comes back
  at rest on re-enable — there is no velocity field to preserve across the gap, and preserving one
  was deliberately scoped out (see the design doc's §4.2).
  ```

- [ ] **Step 2: Update the module README's Tests section**

  In `engine/world/simulation/README.md`, in the "Tests" section, change the "seam itself" bullet
  (currently lines 65-67) from:

  ```markdown
  - The seam itself: `tests/integration/test_scene_serializer_roundtrip.cpp` drives a real
    simulation through `IWorldEditor`, and `tests/integration/test_audio_ecs.cpp` the audio
    extract.
  ```

  to:

  ```markdown
  - The seam itself: `tests/integration/test_scene_serializer_roundtrip.cpp` drives a real
    simulation through `IWorldEditor`, `tests/integration/test_audio_ecs.cpp` the audio extract,
    and `tests/integration/test_entity_lifecycle.cpp` the `enabled`/`activeInHierarchy` gate
    shared by render, physics and (through the editor) audio.
  ```

- [ ] **Step 3: Update the module README's Further reading section**

  In the same file, in "Further reading" (currently lines 71-80), add a new bullet after the
  `physics_system.md` entry:

  ```markdown
  - [`entity_lifecycle_system.md`](../../../docs/design/entity_lifecycle_system.md) — the
    enabled/disabled flag this module's `Record` carries and every system that gates on it.
  ```

- [ ] **Step 4: Add the changelog entry**

  In `docs/reference/changelog.md`, under `## [Unreleased]` → `### Added` (currently starting at
  line 12), add a new top-level bullet at the top of that group (matching the existing
  newest-first ordering):

  ```markdown
  - 2026-08-08 — Added a real `enabled`/`activeInHierarchy` flag: a disabled entity now stops
    physics, audio and render for its whole subtree, not just render. The existing `visible` flag
    keeps its old meaning as a local, non-cascading, render-only toggle. See
    `docs/design/entity_lifecycle_system.md`.
    - Added the Inspector header's second ("Visible") checkbox and Hierarchy panel dimming for
      disabled rows.
  ```

- [ ] **Step 5: Commit**

  ```bash
  git add docs/architecture/domain-physics.md engine/world/simulation/README.md \
          docs/reference/changelog.md
  git commit -m "docs: document the enabled/activeInHierarchy lifecycle

Closes the documentation gap entity_lifecycle_system.md's audit
found: domain-physics.md §1.5, the simulation module README, and the
changelog all now describe the flag this phase added."
  ```

---

## Self-Review

**Spec coverage** (`docs/design/entity_lifecycle_system.md` §4):
- §4.1 data model → Task 1.
- §4.2 physics → Task 3.
- §4.3 audio → Task 4.
- §4.4 API surface → Task 1.
- §4.5 editor UI → Task 5.
- §4.6 testing → woven into Tasks 1-3 (TDD, failing-test-first for every code change); §4.6's audio
  case is Task 4 Step 2, explicitly marked manual since no harness exists to automate it.
- §4.7 documentation → Task 6, corrected to `domain-physics.md` (the design doc's original
  `world.md` citation was wrong — fixed in both documents before this plan was written) and
  expanded to include the changelog entry `docs/CONTRIBUTING.md` §5 requires that §4.7 did not
  itself enumerate.

**Placeholder scan:** no TBD/TODO; the one explicitly-scoped-out item (a `gather_vehicle_descriptions`-specific
automated test, an `AudioEditorSystem` automated test) is stated as a deliberate, reasoned
omission in Task 3 Step 1 and Task 4 Step 2, not left vague.

**Type consistency:** `enabled`/`set_enabled`/`enabled_in_hierarchy` names and signatures match
across Task 1 (declaration + implementation), Task 2 (render gate call sites), Task 3 (physics gate
call sites), Task 4 (audio), and Task 5 (UI) — verified by re-reading each task's code blocks
against Task 1's Step 5 interface declarations while writing this plan.
