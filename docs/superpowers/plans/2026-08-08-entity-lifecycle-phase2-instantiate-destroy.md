# Entity Lifecycle Phase 2 — Runtime Instantiate/Destroy Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development
> (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use
> checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give `RuntimeSimulation` a deferred, tick-safe spawn/destroy surface —
`request_instantiate`/`request_destroy` — that queues structural changes and applies them at one
fixed barrier inside `step_once()`, so future mid-tick callers (Phase 3's native hooks) never mutate
`records_`/`order_`/`world_` while another phase of the same tick is iterating them. The existing
synchronous `create()`/`destroy()` are untouched in behavior; they gain a small internal refactor
that removes a redundant `extract()` call per node of a destroyed subtree.

**Architecture:** Split `create()`/`destroy()` into `_without_extract` halves the new deferred path
reuses, so no lifecycle logic (recursive child collection, the four physics-adjacent dirty flags,
joint teardown, UI-mirror cleanup) is duplicated. Add one small ordered queue
(`std::vector<PendingCommand>`) and one flush point, called once per tick immediately after
`schedule_.run(world_)` and before `step_particle_emitters()`.

**Tech Stack:** C++17, the existing `RuntimeSimulation`/`IWorldEditor` seam, GoogleTest
(`tests/CMakeLists.txt`).

## Global Constraints

- **The spec is `docs/design/entity_lifecycle_system.md` §5.** Every task below implements one
  piece of that section; do not deviate from its architecture (thin queue over existing operations,
  one flush point, pre-allocated ids, cancel-on-destroy-of-pending-instantiate, idempotent destroy)
  without going back to the user. §5.6 was corrected after an implementability gap was found while
  writing this plan (the mid-tick test-double registration it originally proposed is not achievable
  with the actual `Schedule`/`IWorldEditor` surface) — the corrected version, which this plan's
  tests follow, proves the same safety property via before/after-`tick()` assertions instead.
- **Never invoke the build system directly, and never run `se build`/`se test`/`se editor`
  yourself.** This machine cannot run the build. Every step that would normally "run the test" is
  instead phrased as: hand the exact command to the user and ask them to run it and paste back the
  output. Do not claim a test passes without that output in hand.
- **SOLID/encapsulation is priority 1, performance is priority 2** (`CLAUDE.md`). No lifecycle
  logic is duplicated between the synchronous and deferred paths — both call the same
  `_without_extract` implementations.
- **Documentation ships in the same change as the code** (`docs/CONTRIBUTING.md` §5): the
  architecture chapter and the module README update in Task 3, not later.
- **100-column line width** and this codebase's existing comment style (Apache license header
  block, `@brief` Doxygen tags, "Unity's `X`" cross-references) apply to every new file and edit.
- **Commit style:** `type(scope): summary`, one commit per task, following this repo's existing
  history (see `git log --oneline -20` for recent examples).
- **No backward-compatibility shims.** `request_instantiate`/`request_destroy` are new methods with
  no prior callers; `create()`/`destroy()`'s public signatures and synchronous behavior do not
  change.
- **Line numbers below are from the snapshot this plan was written against.** If a prior task in
  this same plan has already shifted lines in a file a later task also touches, use the
  surrounding code shown in that later task's diff to locate the right spot, not the raw number.

---

### Task 1: Extract-free refactor of `create()`/`destroy()`

**Files:**
- Modify: `engine/world/simulation/source/runtime_simulation.cpp`
  - `create()` (currently lines 311-324)
  - `destroy()` (currently lines 326-363)
  - New private helpers beside `find()` (currently lines 2568-2572)

**Interfaces:**
- Produces: private `EntityId create_without_extract(EntityId id, const std::string& display_name,
  EntityId parent)`, private `void destroy_without_extract(EntityId id)`. Both are consumed by
  Task 2's deferred path.
- Consumes: existing `world_.spawn(...)`, `records_`, `order_`, `next_id_`, `set_parent()`
  (`:2032-2082`), `CommandBuffer` (`engine/foundation/ecs/include/SushiEngine/ecs/command_buffer.hpp`),
  all already in scope.

This task has no new externally observable behavior — `create()`/`destroy()`'s signatures, return
values, and effects are unchanged from a caller's point of view. Its verification step is therefore
the existing regression suite, not a new failing test: the "test" this task must pass is that
every existing caller of `create()`/`destroy()`, across the whole tree, behaves identically after
the refactor. The change is worth making now (rather than folding it silently into Task 2) because
it removes a real inefficiency `destroy()` has today: each recursive `destroy(child)` call ends
with its own `extract()` (a full render-scene rebuild), so destroying an N-node subtree rebuilds the
scene N times. After this task, a subtree destroy rebuilds it once.

- [ ] **Step 1: Add the two new private helpers**

  In `engine/world/simulation/source/runtime_simulation.cpp`, immediately after `find()`
  (currently lines 2568-2572):

  ```cpp
                    const Record* find(EntityId id) const noexcept
                    {
                        const auto it = records_.find(id);
                        return it != records_.end() ? &it->second : nullptr;
                    }

                    /**
                     * @brief The non-extracting half of @ref create. Builds the entity and its
                     * @ref Record, optionally parenting it, but does not rebuild the render scene —
                     * the caller decides when that happens.
                     */
                    void create_without_extract(EntityId id, const std::string& display_name,
                                                EntityId parent)
                    {
                        const Entity entity = world_.spawn(Transform{}, Orientation{});
                        order_.push_back(id);
                        Record record{entity, display_name, true, false};
                        records_.emplace(id, record);
                        if (parent != NULL_ENTITY)
                            set_parent(id, parent);
                    }

                    /**
                     * @brief The non-extracting half of @ref destroy. Recurses without
                     * re-extracting per node — the caller extracts once, after the whole subtree
                     * is gone.
                     */
                    void destroy_without_extract(EntityId id)
                    {
                        const auto it = records_.find(id);
                        if (it == records_.end())
                            return;
                        std::vector<EntityId> children;
                        for (auto& entry : records_)
                            if (entry.second.parent == id)
                                children.push_back(entry.first);
                        for (const EntityId child : children)
                            destroy_without_extract(child);
                        if (it->second.has_physics_body)
                            physics_dirty_ = true;
                        if (it->second.has_cloth)
                            cloth_dirty_ = true;
                        if (it->second.has_vehicle)
                            vehicles_dirty_ = true;
                        joints_dirty_ = true;
                        if (world_.alive(it->second.ui_mirror))
                            world_.destroy(it->second.ui_mirror);
                        CommandBuffer commands;
                        commands.destroy(it->second.entity);
                        commands.apply(world_);
                        records_.erase(it);
                        order_.erase(std::remove(order_.begin(), order_.end(), id), order_.end());
                    }
  ```

  This is a straight lift of `create()`/`destroy()`'s existing bodies, with two changes: the
  recursive call is `destroy_without_extract(child)` instead of `destroy(child)`, and neither
  method calls `extract()`.

- [ ] **Step 2: Simplify `create()` and `destroy()` to call the new helpers**

  Replace the body of `create()` (currently lines 311-324):

  ```cpp
                    EntityId create(const std::string& display_name) override
                    {
                        // A truly empty entity: just the mandatory Transform/Orientation,
                        // no Renderer and no mesh, matching Unity's empty GameObject. A
                        // Renderer (and, bound to it, a mesh Shape) is added on demand
                        // through Add Component, so a bare Create Entity never draws.
                        const EntityId id = next_id_++;
                        create_without_extract(id, display_name, NULL_ENTITY);
                        extract();
                        return id;
                    }
  ```

  Replace the body of `destroy()` (currently lines 326-363):

  ```cpp
                    void destroy(EntityId id) override
                    {
                        destroy_without_extract(id);
                        extract();
                    }
  ```

  `destroy()` no longer needs its own `records_.find(id)` guard or comments about recursive
  erasure — `destroy_without_extract` (Step 1) owns all of that now.

- [ ] **Step 3: Hand the regression command to the user**

  Ask the user to run:

  ```
  se build && se test --suite functional --filter 'Integration_EntityLifecycle.*|Integration_ShapeRenderExtraction.*|Integration_PrefabInstance.*|Unit_PrefabSerializer.*'
  ```

  Expected: identical pass/fail counts to before this task's changes — every test in these suites
  exercises `create()`/`destroy()` transitively (directly, or through `apply_prefab`), so this
  range is the actual regression surface. Do not proceed to Step 4 without this output in hand.

- [ ] **Step 4: Commit**

  ```bash
  git add engine/world/simulation/source/runtime_simulation.cpp
  git commit -m "refactor(simulation): split create()/destroy() into extract-free halves

  No behavior change. destroy() of an N-node subtree now rebuilds the render
  scene once instead of N times. Prepares the deferred instantiate/destroy
  path (entity_lifecycle_system.md §5) to reuse this logic without duplicating it."
  ```

---

### Task 2: Deferred queue, `request_instantiate`/`request_destroy`, and the flush point

**Files:**
- Modify: `engine/world/simulation/source/runtime_simulation.cpp`
  - New `PendingCommand` struct and `pending_commands_` member, beside `next_id_`
    (currently line 4623, after Task 1's edits)
  - New `request_instantiate`/`request_destroy`, beside `destroy()` (currently ending line 363,
    before Task 1's edits shift it — see Step 1 for the exact anchor after Task 1)
  - New `flush_deferred_commands()`, beside `destroy_without_extract` (added in Task 1)
  - `step_once()` (currently lines 3305-3394, before Task 1's edits — the flush call goes between
    `schedule_.run(world_)` and `step_particle_emitters()`, currently lines 3390-3391)
- Modify: `engine/world/simulation/include/SushiEngine/simulation/simulation.hpp`
  - `IWorldEditor`, beside `destroy()` (currently line 1268)
- Modify: `tests/integration/test_entity_lifecycle.cpp` (append new tests)

**Interfaces:**
- Consumes: `create_without_extract`, `destroy_without_extract` (Task 1); `set_enabled`
  (`runtime_simulation.cpp:2004`); `exists(EntityId)` (`:232`); `next_id_`.
- Produces: `IWorldEditor::request_instantiate(const std::string&, EntityId = NULL_ENTITY) ->
  EntityId`, `IWorldEditor::request_destroy(EntityId)`. No later task in this plan consumes these,
  but they are the surface Phase 3 (native lifecycle hooks) is expected to call once it exists.

- [ ] **Step 1: Write the failing tests**

  Append to `tests/integration/test_entity_lifecycle.cpp`, after its last existing test:

  ```cpp
  TEST(Integration_EntityLifecycle, RequestDestroyDefersRemovalUntilTheNextFlush)
  {
      const auto simulation = create_simulation();
      ASSERT_NE(simulation, nullptr);
      IWorldEditor& world = simulation->world();
      clear_world(world);

      const EntityId id = world.create("Widget");
      world.request_destroy(id);
      EXPECT_TRUE(world.exists(id)) << "the entity was removed before any flush ran";
      EXPECT_FALSE(world.enabled(id)) << "request_destroy must disable immediately, not defer that";

      simulation->tick(simulation->fixed_dt_seconds());
      EXPECT_FALSE(world.exists(id)) << "the flush did not remove the entity";
  }

  TEST(Integration_EntityLifecycle, RequestDestroyCascadesToChildrenAtFlush)
  {
      const auto simulation = create_simulation();
      ASSERT_NE(simulation, nullptr);
      IWorldEditor& world = simulation->world();
      clear_world(world);

      const EntityId parent = world.create("Parent");
      const EntityId child = world.create("Child");
      world.set_parent(child, parent);

      world.request_destroy(parent);
      simulation->tick(simulation->fixed_dt_seconds());

      EXPECT_FALSE(world.exists(parent));
      EXPECT_FALSE(world.exists(child)) << "the child did not go with its parent";
  }

  TEST(Integration_EntityLifecycle, RequestInstantiateDefersCreationUntilTheNextFlush)
  {
      const auto simulation = create_simulation();
      ASSERT_NE(simulation, nullptr);
      IWorldEditor& world = simulation->world();
      clear_world(world);

      const EntityId id = world.request_instantiate("Spawned");
      EXPECT_FALSE(world.exists(id)) << "the entity was created before any flush ran";

      simulation->tick(simulation->fixed_dt_seconds());
      EXPECT_TRUE(world.exists(id)) << "the flush did not create the entity";
      EXPECT_TRUE(world.enabled(id)) << "a freshly instantiated entity must default to enabled";
      EXPECT_NE(find_instance(simulation->render_scene(), id), nullptr)
          << "the flush ran, but the tick's own extract() did not see the new entity";
  }

  TEST(Integration_EntityLifecycle, RequestDestroyOfAPendingInstantiateCancelsItOutright)
  {
      const auto simulation = create_simulation();
      ASSERT_NE(simulation, nullptr);
      IWorldEditor& world = simulation->world();
      clear_world(world);

      const EntityId id = world.request_instantiate("Cancelled");
      world.request_destroy(id);

      simulation->tick(simulation->fixed_dt_seconds());
      EXPECT_FALSE(world.exists(id)) << "a cancelled spawn must never materialize";
  }

  TEST(Integration_EntityLifecycle, RequestDestroyIsIdempotentBeforeFlush)
  {
      const auto simulation = create_simulation();
      ASSERT_NE(simulation, nullptr);
      IWorldEditor& world = simulation->world();
      clear_world(world);

      const EntityId id = world.create("Widget");
      world.request_destroy(id);
      world.request_destroy(id);

      simulation->tick(simulation->fixed_dt_seconds());
      EXPECT_FALSE(world.exists(id));
  }

  TEST(Integration_EntityLifecycle, RequestDestroyOnAnUnknownIdIsANoOp)
  {
      const auto simulation = create_simulation();
      ASSERT_NE(simulation, nullptr);
      IWorldEditor& world = simulation->world();
      clear_world(world);

      world.request_destroy(EntityId(999999));
      simulation->tick(simulation->fixed_dt_seconds());
      SUCCEED() << "request_destroy on an unknown id must not crash";
  }
  ```

- [ ] **Step 2: Run the tests to verify they fail to compile**

  Ask the user to run:

  ```
  se build && se test --suite functional --filter 'Integration_EntityLifecycle.*'
  ```

  Expected: a compile error — `request_destroy`/`request_instantiate` are not members of
  `IWorldEditor` yet.

- [ ] **Step 3: Declare the new methods on `IWorldEditor`**

  In `engine/world/simulation/include/SushiEngine/simulation/simulation.hpp`, immediately after
  `destroy()` (currently line 1268):

  ```cpp
                /** @brief Destroys @p id; a no-op if it does not exist. */
                virtual void destroy(EntityId id) = 0;

                /**
                 * @brief Queues @p id (and its whole subtree) for removal at this tick's flush
                 * point, rather than removing it immediately.
                 *
                 * Disables @p id immediately — @ref enabled_in_hierarchy gates it out of physics,
                 * audio and render from this call onward — but the actual `Record`/entity removal
                 * happens once, inside the current or next call to `ISimulation::tick`, alongside
                 * every other command queued this tick. Safe to call from code that runs mid-tick,
                 * unlike @ref destroy: it never mutates the entity table itself.
                 *
                 * A second call for an id already queued, or for an id nothing knows about, is a
                 * harmless no-op. A call for an id returned by a not-yet-flushed
                 * @ref request_instantiate cancels that spawn outright — the entity is never
                 * created at all.
                 *
                 * @param id The entity (and its subtree) to remove.
                 */
                virtual void request_destroy(EntityId id) = 0;

                /**
                 * @brief Queues a new entity for creation at this tick's flush point, rather than
                 * creating it immediately.
                 *
                 * Returns the entity's id immediately, pre-allocated from the same counter
                 * @ref create uses, so it can be passed to another @ref request_instantiate call's
                 * @p parent or to @ref request_destroy within the same tick, before the entity it
                 * names actually exists. `exists()` on that id returns `false` until the flush
                 * runs. Safe to call from code that runs mid-tick, unlike @ref create: it never
                 * mutates the entity table itself.
                 *
                 * @param name Display name for the new entity.
                 * @param parent The entity to parent the new entity under, or `NULL_ENTITY` for
                 * none.
                 * @return The new entity's id, valid to reference immediately, not yet backed by
                 * a `Record` until the next flush.
                 */
                virtual EntityId request_instantiate(const std::string& name,
                                                     EntityId parent = NULL_ENTITY) = 0;
  ```

- [ ] **Step 4: Add the `PendingCommand` queue**

  In `engine/world/simulation/source/runtime_simulation.cpp`, immediately after
  `EntityId next_id_ = 1;` (currently line 4623, after Task 1's edits — search for that exact line
  if it has shifted):

  ```cpp
                    EntityId next_id_ = 1;

                    /**
                     * @brief One queued structural change, applied at the next
                     * @ref flush_deferred_commands call. See entity_lifecycle_system.md §5.1.
                     */
                    struct PendingCommand
                    {
                        enum class Kind { Instantiate, Destroy };
                        Kind kind;
                        EntityId id;                   // Destroy: the target. Instantiate: the
                                                        // pre-allocated id.
                        std::string display_name;      // Instantiate only.
                        EntityId parent = NULL_ENTITY;  // Instantiate only.
                    };
                    std::vector<PendingCommand> pending_commands_;
  ```

- [ ] **Step 5: Implement `request_instantiate`/`request_destroy`**

  In `engine/world/simulation/source/runtime_simulation.cpp`, immediately after `destroy()`
  (after Task 1's edits, this is the three-line body ending `extract(); }`):

  ```cpp
                    void destroy(EntityId id) override
                    {
                        destroy_without_extract(id);
                        extract();
                    }

                    void request_destroy(EntityId id) override
                    {
                        // Cancel a same-tick, not-yet-materialized spawn outright: it never gets
                        // a Record, never touches world_, and the caller's id simply becomes
                        // invalid, exactly as if it had never been requested.
                        const auto pending = std::find_if(pending_commands_.begin(),
                            pending_commands_.end(), [id](const PendingCommand& c) {
                                return c.kind == PendingCommand::Kind::Instantiate && c.id == id;
                            });
                        if (pending != pending_commands_.end())
                        {
                            pending_commands_.erase(pending);
                            return;
                        }
                        // Idempotent: a second request this tick, or a request for an id nothing
                        // knows about, is a harmless no-op — matching CommandBuffer::destroy's own
                        // alive()-guard philosophy (command_buffer.hpp:62-63).
                        const bool already_queued = std::any_of(pending_commands_.begin(),
                            pending_commands_.end(), [id](const PendingCommand& c) {
                                return c.kind == PendingCommand::Kind::Destroy && c.id == id;
                            });
                        if (already_queued || !exists(id))
                            return;
                        set_enabled(id, false);
                        pending_commands_.push_back({PendingCommand::Kind::Destroy, id});
                    }

                    EntityId request_instantiate(const std::string& display_name,
                                                 EntityId parent) override
                    {
                        const EntityId id = next_id_++;
                        pending_commands_.push_back(
                            {PendingCommand::Kind::Instantiate, id, display_name, parent});
                        return id;
                    }
  ```

- [ ] **Step 6: Implement and wire the flush**

  In `engine/world/simulation/source/runtime_simulation.cpp`, immediately after
  `destroy_without_extract` (added in Task 1, beside `find()`):

  ```cpp
                    /**
                     * @brief Applies every queued @ref PendingCommand, in the order each was
                     * requested, then clears the queue. Called once per tick, from @ref
                     * step_once — see entity_lifecycle_system.md §5.3 for why that point.
                     */
                    void flush_deferred_commands()
                    {
                        for (const PendingCommand& command : pending_commands_)
                        {
                            if (command.kind == PendingCommand::Kind::Instantiate)
                                create_without_extract(command.id, command.display_name,
                                                       command.parent);
                            else
                                destroy_without_extract(command.id);
                        }
                        pending_commands_.clear();
                    }
  ```

  Then, in `step_once()` (currently lines 3305-3394), insert the flush call between
  `schedule_.run(world_);` and `step_particle_emitters();` (currently lines 3390-3391):

  ```cpp
                        schedule_.run(world_);
                        flush_deferred_commands();
                        step_particle_emitters();
                        step_crowd_playback();
                        extract();
  ```

- [ ] **Step 7: Run the tests to verify they pass**

  Ask the user to run:

  ```
  se build && se test --suite functional --filter 'Integration_EntityLifecycle.*'
  ```

  Expected: all tests in this suite pass, including the six new ones from Step 1.

- [ ] **Step 8: Commit**

  ```bash
  git add engine/world/simulation/source/runtime_simulation.cpp \
          engine/world/simulation/include/SushiEngine/simulation/simulation.hpp \
          tests/integration/test_entity_lifecycle.cpp
  git commit -m "feat(simulation): add request_instantiate/request_destroy

  A deferred, tick-safe spawn/destroy surface for callers that run
  mid-tick — queued and applied once per tick, after schedule_.run() and
  before that tick's own extract(). Reuses create_without_extract/
  destroy_without_extract (no duplicated lifecycle logic). Implements
  entity_lifecycle_system.md §5."
  ```

---

### Task 3: Documentation

**Files:**
- Modify: `docs/architecture/domain-physics.md` §1.5
- Modify: `engine/world/simulation/README.md`

**Interfaces:**
- Consumes: nothing new — this task only documents Task 2's shipped surface.
- Produces: nothing consumed by another task.

- [ ] **Step 1: Update `docs/architecture/domain-physics.md` §1.5**

  Find the "Enabled/disabled lifecycle" bulleted entry Phase 1 added to §1.5 ("Editor authoring").
  Add a new entry immediately after it, in the same style:

  ```markdown
  - **Runtime instantiate/destroy.** `IWorldEditor::request_instantiate`/`request_destroy` queue a
    spawn or removal instead of applying it immediately — safe to call from code that runs mid-tick,
    where the synchronous `create`/`destroy` are not. Both are applied once per tick, inside
    `RuntimeSimulation::step_once`, immediately after the ECS schedule runs and before that tick's
    `extract()`. `request_destroy` disables its target immediately (see the entry above) even though
    the actual removal is deferred; a `request_destroy` naming a not-yet-flushed
    `request_instantiate`'s id cancels that spawn outright rather than creating and immediately
    destroying it.
  ```

- [ ] **Step 2: Update `engine/world/simulation/README.md`**

  Find the module's owned-behavior list (the same list Phase 1's Task 6 added `enabled` to). Add:
  "deferred, tick-safe instantiate/destroy (`request_instantiate`/`request_destroy`)".

- [ ] **Step 3: Commit**

  ```bash
  git add docs/architecture/domain-physics.md engine/world/simulation/README.md
  git commit -m "docs(simulation): document request_instantiate/request_destroy"
  ```
