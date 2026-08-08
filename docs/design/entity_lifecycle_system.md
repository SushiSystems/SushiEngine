# Entity Lifecycle System — real enabled/disabled, spawn/destroy, and native hooks (`SushiEngine::Simulation`)

**Status:** designed — Phases 1 and 2 are specified below; Phases 3-4 are named and ordered but
not yet specified in detail.

A scene entity's "enabled" state is cosmetic today. `Record::visible`
(`engine/world/simulation/source/runtime_simulation.cpp:2049`) and the hierarchy walk over it,
`visible_in_hierarchy` (`runtime_simulation.cpp:2721-2733`), gate nothing but what `extract()` pushes
into the render scene. Physics, audio and the entity lifecycle itself do not consult it: a hidden
rigid body still collides and consumes solver time, a hidden emitter still plays, and there is no
engine concept of "this object does not exist right now" beyond render culling. An editor built for
AAA content authoring needs the real thing — an entity that is truly off, in every system that
touches it, the way Unity's `GameObject.SetActive(false)` or Unreal's actor `SetActorHiddenInGame` +
tick-disable is the real thing.

This document is the design for that whole initiative, broken into four phases so each can be
specified, planned and implemented on its own terms:

1. **Enabled/disabled lifecycle** — an entity that is truly inactive across physics, audio and
   render, not just hidden from render.
2. **Runtime instantiate/destroy** — bringing scene-object spawn/destroy onto the deferred,
   barrier-applied discipline `engine/foundation/ecs` already has, and giving gameplay code (once
   there is any) a way to call it.
3. **Native lifecycle hooks** — an `OnEnable`/`OnDisable`/`OnSpawn`/`OnDestroy` callback surface,
   implemented in native C++ rather than through a scripting VM, since none exists yet (§2 Non-goals).
4. **Inter-object messaging** — entities calling named methods on each other, built on the dispatch
   surface Phase 3 establishes.

Phases 1 and 2 are specified in this revision. Phases 3-4 are placeholders (§6-§7) naming what each
covers and why it is ordered where it is; each is filled in with its own audit, non-goals and design
before its own implementation starts, in a later revision of this same document.

---

## §1 Audit — what exists, and what each phase builds on (2026-08-07)

- **The render gate is real but narrow.** `visible_in_hierarchy` (`runtime_simulation.cpp:2721-2733`)
  walks a `Record`'s ancestor chain and returns `false` if any ancestor (or the record itself) has
  `visible == false`. It is called at exactly nine sites inside `extract()`
  (`runtime_simulation.cpp:3772, 3798, 3872, 3913, 3953, 3980, 4247, 4280, 4422`) — mesh, cloth,
  soft-body/vehicle shell, particle, light and decal instancing. It is called nowhere else.
- **Physics has one primary filter point, and a second, independent one.**
  `physics_source_entities()` (`runtime_simulation.cpp:3409-3447`) walks `order_` and skips a record
  only for `record == nullptr || !world_.alive(record->entity)`
  (`runtime_simulation.cpp:3416`) — never `visible`. It feeds `gather_rigid_descriptions`
  (`runtime_simulation.cpp:3392`), `gather_static_planes` (`runtime_simulation.cpp:3462`), and (per
  their own bodies, not re-verified per-kind in this pass) soft body and cloth gathering. But
  `gather_vehicle_descriptions` (`runtime_simulation.cpp:2328-2351`) does **not** go through
  `physics_source_entities()` — it walks `order_` itself with its own inline filter
  (`runtime_simulation.cpp:2334-2336`), so it is a second call site that needs its own gate. Whether
  the character controller has a third, independent gather is not yet confirmed and is a Phase 1
  implementation-time check, not an assumption this document makes.
- **Audio is pulled, not pushed, and lives outside `runtime_simulation.cpp` entirely.** The comment at
  `runtime_simulation.cpp:1164-1166` is explicit: audio is "plain host bookkeeping read live by the
  editor's audio system," not something `extract()` snapshots. The actual read site is
  `applications/editor/source/audio/audio_editor_system.cpp:183-211`, which walks `world.entities()`
  and calls `world.has_audio_emitter(id)`/`world.audio_emitter_parameters(id)` through the
  `IWorldEditor` interface — a different translation unit, a different binary target, with no access
  to `RuntimeSimulation`'s private `Record`/`visible_in_hierarchy`. Gating audio therefore cannot reuse
  the render gate's internals directly; it needs a hierarchy check exposed *on the interface*.
- **The toggle API today is `IWorldEditor::visible`/`set_visible`**
  (`engine/world/simulation/include/SushiEngine/simulation/simulation.hpp:1209, 1406`), implemented by
  `RuntimeSimulation::visible`/`set_visible` (`runtime_simulation.cpp:287-...`, `~1930s`), and it is the
  single checkbox in the Inspector header (`applications/editor/source/scene/inspector_panel.cpp:283-296`).
  There is no separate "enabled" concept anywhere in the tree — not in `Record`, not in
  `IWorldEditor`, not in the Hierarchy panel.
- **The low-level ECS already has the deferred discipline Phase 2 needs.**
  `CommandBuffer::spawn`/`destroy` (`engine/foundation/ecs/include/SushiEngine/ecs/command_buffer.hpp:54, 67`)
  record closures; `apply()` (`command_buffer.hpp:76`) replays them once, at an explicit barrier. The
  scene-object layer does not use this: `RuntimeSimulation::create()`
  (`runtime_simulation.cpp:293-...`) calls `world_.spawn(...)` immediately, and `destroy()`
  (`runtime_simulation.cpp:308-...`) builds a local `CommandBuffer` and calls `apply()` synchronously
  in the same call — editor-authoring semantics, not deferred simulation semantics.
- **There is no scripting VM.** `ScriptComponent`
  (`engine/world/simulation/include/SushiEngine/simulation/simulation.hpp:1069-1073`) is authoring
  data only — a type name and a field list — with no interpreter or compiled-callback path behind it.
  Phase 3's hooks are specified as a native C++ interface for exactly this reason: building a
  scripting language is its own multi-week project and is not what "hooks" requires.

## §2 Non-goals (whole document)

- **No scripting VM or scripting language.** Phase 3's hooks are native C++ virtual dispatch, not a
  bytecode interpreter, not a new authoring language. If a scripting VM is built later, it would
  consume Phase 3's dispatch surface rather than this document redesigning it.
- **No velocity preservation across disable/enable.** Physics bodies are removed from the solver on
  disable and re-added at rest on enable (§4.2). This is a deliberate simplicity choice made with the
  user, not an oversight — see §4.2 for the reasoning.
- **No prefab override resolution, no entity persistent identity.** Out of scope for every phase here;
  `docs/design/prefab_system.md` §1 already names entity identity as the blocker for the phase that
  needs it, and this document does not revisit that.
- **No networked/rollback semantics for enable/disable or spawn/destroy.** `docs/design/SUSHILOOP.md`
  owns determinism and rollback; this document's phases produce host-side behavior only, and joining
  them to the deterministic loop (if ever required) is that document's concern, not this one's.

## §3 Roadmap

| Phase | Covers | Status |
| --- | --- | --- |
| 1 | Enabled/disabled lifecycle: `enabled`/`enabled_in_hierarchy` gating physics, audio and render; Inspector + Hierarchy UI | specified (§4) |
| 2 | Runtime instantiate/destroy on the deferred/command-buffer discipline; a gameplay-time spawn/destroy surface | specified (§5) |
| 3 | Native `OnEnable`/`OnDisable`/`OnSpawn`/`OnDestroy` hooks | named, not specified |
| 4 | Inter-object messaging on Phase 3's dispatch surface | named, not specified |

---

## §4 Phase 1 — Enabled/disabled lifecycle

### §4.1 Data model

`Record` gains one field: `bool enabled = true;` — Unity's `activeSelf`. `Record::visible` is
redefined by the same change: it becomes a purely local (non-hierarchical), render-only flag, where
today it cascades — `visible_in_hierarchy` (§1) walks the ancestor chain over it, so a hidden parent
currently hides its descendants too. After this phase only `enabled` cascades. That is a real
behavior change to a flag every scene on disk already carries, and it is deliberate rather than
incidental: two flags that both cascade are two answers to one question, and the cascade belongs to
the one that means "this object is not active", not to the one that means "do not draw this". The
consequence is stated here rather than left to be discovered — a scene authored with a
`visible=false` parent will draw the children that the old cascade hid, and an author who wanted the
subtree gone wants `enabled` on that parent instead. What does not change is a `visible=false`
entity's own rendering, which is off before and after. `enabled` is new and defaults to `true`, so
no scene-file migration is required; a record written before the key existed reads back as enabled.

A new private helper, `enabled_in_hierarchy(const Record*)`, is added beside
`visible_in_hierarchy` (`runtime_simulation.cpp:2721-2733`), identical in shape: walks the ancestor
chain, returns `false` if any node (including the record itself) has `enabled == false`.

`visible_in_hierarchy` itself is retired. Its nine call sites in `extract()`
(`runtime_simulation.cpp:3772, 3798, 3872, 3913, 3953, 3980, 4247, 4280, 4422`) become
`!enabled_in_hierarchy(record) || !record->visible` — hierarchy gates existence, the local flag gates
this entity's own render contribution on top of that, matching how a disabled parent's inactive
children never reach a child's own `visible` check in Unity either.

`enabled` persists everywhere `visible` already does, and for the same reason: it is authored state.
That is `write_entity_record`/`read_entity_record` in
`engine/world/serialization/source/scene_serializer.cpp` — one pair of functions carrying Save/Load,
Undo/Redo, the Play→Stop restore and prefab capture/apply, since `capture_scene`/`apply_scene` and
`capture_prefab`/`apply_prefab` all route through them — plus `ClipboardEntity`
(`applications/editor/source/core/editor_context.hpp`) for copy/paste and duplicate. A flag the
capture forgets is not merely unsaved: undoing any unrelated edit would re-enable every disabled
entity in the scene.

### §4.2 Physics integration

- `physics_source_entities()` (`runtime_simulation.cpp:3409-3447`) gains the check at its existing
  filter line 3416: an entity with `!enabled_in_hierarchy(record)` is skipped, exactly like a dead
  entity is today. This one change covers every consumer of `physics_source_entities()` — rigid
  bodies, static planes, and (pending the implementation-time confirmation in §1) soft bodies and
  cloth.
- `gather_vehicle_descriptions()` (`runtime_simulation.cpp:2328-2351`) gets its own, separate
  `!enabled_in_hierarchy(record)` check alongside its existing `!record->has_vehicle` filter
  (`runtime_simulation.cpp:2334-2336`), since it does not route through `physics_source_entities()`.
- No change is needed inside `PhysicsSimulation` itself. `set_rigid_bodies`
  (`engine/world/simulation/include/SushiEngine/simulation/physics_simulation.hpp:159-218`) is already
  a diff against the incoming description list: an entity missing from the list has its body
  `remove_body`'d (`physics_simulation.hpp:179`); an entity newly present is `add_body`'d fresh
  (`physics_simulation.hpp:196-214`) from position/orientation/mass/inertia — there is no velocity
  field on `RigidBodyDescription`'s creation path, so a body created this way always starts at rest.
  Disabling an entity therefore removes its body from the world exactly as if it had never been there
  this frame, and re-enabling adds it back at rest. This was confirmed with the user as an accepted
  tradeoff: simplicity over momentum preservation, and it is called out again in §2 so it is not
  mistaken for an oversight later.

### §4.3 Audio integration

Per §1's audit, audio is read live through `IWorldEditor`, not pushed through `extract()`, so the
render gate's private helper cannot be reused directly. `IWorldEditor` gains a new pure-virtual method,
`enabled_in_hierarchy(EntityId id) const noexcept`, implemented by `RuntimeSimulation` as a thin
public wrapper over the same private helper §4.1 adds. This keeps the hierarchy walk written once,
consumed by both `extract()` (internally) and `audio_editor_system.cpp` (through the interface) — the
alternative, walking `parent()`/`enabled()` by hand inside `audio_editor_system.cpp`, would duplicate
logic that already exists and is exactly the kind of cross-boundary drift this codebase's module
boundaries exist to prevent.

`audio_editor_system.cpp`'s emitter loop (`audio_editor_system.cpp:183-211`) adds
`if (!world.enabled_in_hierarchy(id)) continue;` beside its existing `!world.has_audio_emitter(id)`
check. Because `AudioEditorSystem::update` rebuilds the whole `Audio::SceneSnapshot` and calls
`scene_.apply(snapshot)` (`audio_editor_system.cpp:214`) every call, an emitter that drops out of the
snapshot stops immediately — this is the existing update mechanism, not new machinery, and it produces
exactly the "stop immediately" behavior agreed with the user.

### §4.4 API surface

`IWorldEditor` (`engine/world/simulation/include/SushiEngine/simulation/simulation.hpp`) gains, beside
the existing `visible`/`set_visible` (lines 1209, 1406):

```cpp
virtual bool enabled(EntityId id) const noexcept = 0;
virtual void set_enabled(EntityId id, bool enabled) = 0;
virtual bool enabled_in_hierarchy(EntityId id) const noexcept = 0;
```

`enabled`/`set_enabled` read and write the local flag (mirrors `visible`/`set_visible`'s existing
implementation shape exactly). `enabled_in_hierarchy` is read-only — it is derived, not authored, so
there is no setter.

### §4.5 Editor UI

- **Inspector header** (`applications/editor/source/scene/inspector_panel.cpp:283-296`): the existing
  checkbox moves from `visible`/`set_visible` to `enabled`/`set_enabled` — this is the position Unity's
  own Inspector header checkbox (`activeSelf`) occupies. A second checkbox, "Visible", is added on the
  same line, bound to `visible`/`set_visible`. Both follow the existing mixed-value pattern for
  multi-selection (`inspector_panel.cpp:284-290`) independently. Two checkboxes rather than nesting
  "Visible" inside the Renderer component section, because `visible` already gates every visual
  representation an entity can carry (mesh, cloth, soft/vehicle shell, particles, lights, decals per
  §4.1's nine call sites) — it is an entity-level flag, not a Renderer-component-level one, and its UI
  placement should say so.
- **Hierarchy panel**: rows for entities where `enabled_in_hierarchy(id) == false` render with
  `ImGuiCol_TextDisabled` instead of the normal text color — selection, drag-and-drop and context menu
  behavior are unchanged, this is a visual indicator only, matching Unity/Unreal's greyed-out inactive
  rows.

### §4.6 Testing

- `tests/integration/test_shape_render_extraction.cpp` (existing `visible`/hierarchy coverage at
  lines 112-135) gains parallel cases for `enabled`: a disabled ancestor hides a render-active
  descendant; `visible=true, enabled=false` still hides; `visible=false, enabled=true` hides only this
  entity's own render contribution, independent of `enabled`.
- `tests/integration/test_physics_simulation.cpp` gains cases asserting that disabling an entity
  removes its rigid body from the physics world (observable via the existing body-count/debug-state
  surface) and that re-enabling adds it back at rest.
- An audio-side case is added at implementation time once the right test target for
  `audio_editor_system.cpp` is confirmed (no existing test references it as of this audit).

### §4.7 Documentation

Per `docs/CONTRIBUTING.md` §5, the same PR updates:

- `docs/architecture/domain-physics.md` §1.5 ("Editor authoring") — a new bulleted entry
  documenting `enabled`/`visible` as as-built, in the same style as that section's existing
  "Cloth as an object"/"UI"/"Custom (script) components" entries, including which systems gate on
  which. (`docs/architecture/world.md`, this document's first guess at the right file, turned out to
  cover an unrelated subsystem — SushiLoop's rollback/net layers under `engine/world/loop/` — not
  `RuntimeSimulation`; `domain-physics.md` is where `IWorldEditor`/`Record`-level authoring
  semantics are actually documented today, imperfect a fit as a physics-titled file is for a
  cross-cutting concern. This is itself the gap: no architecture doc owns the scene-object model as
  its own subject. Fixing that categorization is out of scope for this phase.)
- `engine/world/simulation/README.md` — the module's owned-behavior list gains `enabled`.
- `docs/design/README.md` — gains a row for this document (done: see the corpus index).

---

## §5 Phase 2 — Runtime instantiate/destroy

### §5.0 Audit correction and scope

The line numbers §1 cited for `create()`/`destroy()` (`runtime_simulation.cpp:293, 308`) have drifted
with edits made since that audit; in this snapshot they are `runtime_simulation.cpp:311` and `:326`.
Re-reading both confirms §1's characterization: `create()` calls `world_.spawn(...)` and
`records_.emplace(...)` directly (`:317-321`), and `destroy()` builds a local `CommandBuffer`,
`apply()`s it in the same call (`:356-358`), then erases from `records_`/`order_` in the same call
(`:359-361`) — both are synchronous, editor-authoring operations start to finish.

No caller in the tree needs anything different from that today (confirmed: nothing calls `create`/
`destroy` from inside `step_once()`, and prefab instantiation is edit-time only — see the research
this phase's brainstorm ran). This phase is not built because something is broken; it is built because
Phase 3 (§6) names `OnSpawn`/`OnDestroy` firing "from Phase 2's instantiate/destroy," which means
Phase 3's native hooks — the first callers that can run *during* a tick, from inside `schedule_.run()`
or the impact-response path — need a spawn/destroy surface that is safe to call from there. Calling
today's `create()`/`destroy()` from mid-tick would not be safe: `destroy()` mutates `order_` and
`records_` while `step_once()`'s own physics-readback loop (`:3376-3388`) is walking `order_`, and nothing
guards against the reentrancy. This phase's job is to make that surface exist and be provably safe
before Phase 3 has any code that calls it.

### §5.1 Architecture: a thin queue over the existing operations, not a new implementation

`RuntimeSimulation` gains one new private member, an ordered queue of pending commands, plus two new
methods declared on `IWorldEditor` (§5.2) and implemented by `RuntimeSimulation` to write to that
queue. The queue does not reimplement spawn/destroy — every piece of existing bookkeeping this
document's Phase 1 work depends on (recursive child collection, the four physics-adjacent dirty flags,
joint teardown, UI-mirror cleanup) stays in exactly one place. What the queue controls is *when* that
existing logic runs, not *what* it does.

```cpp
struct PendingCommand
{
    enum class Kind { Instantiate, Destroy };
    Kind kind;
    EntityId id;                        // Destroy: the target. Instantiate: the pre-allocated id.
    std::string display_name;           // Instantiate only.
    EntityId parent = NULL_ENTITY;      // Instantiate only.
};
std::vector<PendingCommand> pending_commands_;
```

A `std::vector` rather than two separate queues (one per kind) because ordering across kinds matters —
see §5.2's cancellation case, which depends on being able to find an entity's own not-yet-applied
`Instantiate` entry before a later `Destroy` request for the same id is queued.

`create()` and `destroy()` are each split into an `_without_extract` implementation and a thin public
wrapper that calls it once and then calls `extract()` once:

```cpp
EntityId create(const std::string& display_name) override
{
    const EntityId id = next_id_++;
    create_without_extract(id, display_name, NULL_ENTITY);
    extract();
    return id;
}

private:
    void create_without_extract(EntityId id, const std::string& display_name, EntityId parent)
    {
        const Entity entity = world_.spawn(Transform{}, Orientation{});
        order_.push_back(id);
        Record record{entity, display_name, true, false};
        records_.emplace(id, record);
        if (parent != NULL_ENTITY)
            set_parent(id, parent);
    }
```

```cpp
void destroy(EntityId id) override
{
    destroy_without_extract(id);
    extract();
}

private:
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
            destroy_without_extract(child);   // was destroy(child) — no longer re-extracts per node
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

This split is worth doing regardless of Phase 2, and is included in this phase because Phase 2 is what
exposes it: today's `destroy()` calls `extract()` once per node of a destroyed subtree (each recursive
`destroy(child)` hits the `extract()` at the end of its own call), rebuilding the entire render scene
N times for an N-node subtree. `destroy_without_extract` recurses without extracting; the public
`destroy()` extracts exactly once, after the whole subtree is gone. Same behavior, less wasted work —
`extract()` is a full render-scene rebuild, not a cheap call, and this codebase's stated performance
principle is to not spend cycles a caller cannot tell were wasted.

`create_without_extract` gains a `parent` parameter `create()` does not expose (it is always called
with `NULL_ENTITY` from `create()`, preserving today's behavior exactly) — Phase 2's own
`request_instantiate` (§5.2) needs to parent a spawned entity in the same operation that creates it,
and reusing the existing `set_parent()` (`:2032-2082`) rather than duplicating its transform-preserving
math is the only place that capability can live without copying code.

### §5.2 API surface

`IWorldEditor` gains two new pure-virtual methods, beside `create`/`destroy`:

```cpp
virtual EntityId request_instantiate(const std::string& display_name,
                                     EntityId parent = NULL_ENTITY) = 0;
virtual void request_destroy(EntityId id) = 0;
```

`RuntimeSimulation`'s implementations:

```cpp
EntityId request_instantiate(const std::string& display_name, EntityId parent) override
{
    const EntityId id = next_id_++;
    pending_commands_.push_back({PendingCommand::Kind::Instantiate, id, display_name, parent});
    return id;
}

void request_destroy(EntityId id) override
{
    // Cancel a same-tick, not-yet-materialized spawn outright: it never gets a
    // Record, never touches world_, and the caller's id simply becomes invalid,
    // exactly as if it had never been requested.
    const auto pending = std::find_if(pending_commands_.begin(), pending_commands_.end(),
        [id](const PendingCommand& c) {
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
    const bool already_queued = std::any_of(pending_commands_.begin(), pending_commands_.end(),
        [id](const PendingCommand& c) {
            return c.kind == PendingCommand::Kind::Destroy && c.id == id;
        });
    if (already_queued || !exists(id))
        return;
    set_enabled(id, false);
    pending_commands_.push_back({PendingCommand::Kind::Destroy, id});
}
```

`request_instantiate` returns the id immediately by pre-allocating it from `next_id_` — the same
counter `create()` uses, so ids from both APIs are drawn from one space and never collide. The id is
usable as a `parent` argument to another `request_instantiate` call, or as a `request_destroy` target,
within the same tick, before the entity it names has actually been created — because both operations
only ever read/write the queue and `next_id_`, never `records_`, until flush. Querying anything else
about a pre-allocated id before flush (`enabled()`, `exists()`, `world_transform()`, ...) behaves
exactly as it does for any other unknown id today: `exists()` returns `false`, and callers that do not
guard on `exists()` first are already responsible for that, on every existing `IWorldEditor` method.

### §5.3 Flush timing and ordering

One private method, `flush_deferred_commands()`, applies the whole queue in the order it was recorded
and clears it:

```cpp
void flush_deferred_commands()
{
    for (const PendingCommand& command : pending_commands_)
    {
        if (command.kind == PendingCommand::Kind::Instantiate)
            create_without_extract(command.id, command.display_name, command.parent);
        else
            destroy_without_extract(command.id);
    }
    pending_commands_.clear();
}
```

`step_once()` calls it once, at one fixed point: immediately after `schedule_.run(world_)`
(`:3390`) and before `step_particle_emitters()` (`:3391`). That point, not the very end of `step_once()`,
because `step_particle_emitters()`, `step_crowd_playback()` and the tick's own `extract()` (`:3393`)
should all see the tick's structural changes already resolved — a particle emitter should not be
stepped once more, or a destroyed entity extracted once more, after this tick already decided it is
gone. `schedule_.run()` is the natural point because it is where Phase 3's native hooks are expected to
execute (§6) — the ECS systems phase is where behavior runs — so this is also the last point before the
barrier a hook's `request_destroy`/`request_instantiate` call can land.

Applying in recorded order, rather than batching all destroys before all instantiates or the reverse,
is what makes §5.2's cancellation check correct: a `request_destroy` can only find and erase a
still-pending `Instantiate` entry because both live in the same ordered container, addressable by the
id `request_instantiate` already returned. A single `extract()` covers every command the flush just
applied, at `step_once()`'s existing call site (`:3393`) — the flush itself calls neither `create()` nor
`destroy()`, only their `_without_extract` halves, so no command in a queue of any size triggers more
than the one unavoidable render-scene rebuild for that tick.

Editor-authored `create()`/`destroy()` calls are untouched by any of this — they still run their
`_without_extract` half followed by their own `extract()`, synchronously, in the same call, exactly as
before. The queue exists purely for callers that need to be safe to invoke from inside `step_once()`;
nothing about the editor's existing synchronous authoring model changes.

### §5.4 Interaction with Phase 1

`request_destroy(id)` calls `set_enabled(id, false)` before it ever touches the queue. This is not new
gating logic — it is Phase 1's existing `enabled_in_hierarchy()` cascade (§4.1), reused rather than
duplicated: the moment a destroy is requested, the target and its whole subtree are immediately excluded
from physics, audio and render for the remainder of this tick and any tick before the flush actually
removes them, with zero new code in any of those three systems. A "destroy" is defined, precisely, as
"disable now, actually remove at the next barrier" — which is the same disable-before-destroy
relationship §1's audit anticipated (`entity_lifecycle_system.md:248-250`, this document's own earlier
placeholder text) before either phase had a line of implementation.

A freshly `request_instantiate`d entity's default `enabled` state needs no special handling: it is
created through `create_without_extract`, which builds a `Record` the same way `create()` always has —
`enabled` defaults to `true` (§4.1) — so a spawned entity is active from the moment it exists, matching
`create()`'s own behavior today.

### §5.5 Edge cases and non-goals

- **A pending instantiate's parent is itself pending-destroyed in the same tick.** Not specially
  handled: `set_parent()` (`:2032-2082`) already refuses to attach to an id it cannot `find()`
  (`:2035`), and by the time `flush_deferred_commands()` reaches an `Instantiate` command, any `Destroy`
  command queued *before* it in recorded order has already run — an instantiate that names an
  already-removed parent silently gets no parent, the same outcome `set_parent()` already produces for
  any other missing-parent call today. Queuing the instantiate before the destroy, naming that same
  parent, is a caller ordering choice this phase does not need to guess at or forbid.
- **Two `request_destroy` calls for the same id, one before and one after its subtree changes.** Each
  call re-checks `exists()`/the pending-instantiate table fresh, so this is covered by the idempotency
  check already in §5.2 — the second call is always a no-op once the first has queued (or applied) the
  destroy.
- **No deferred variant of anything beyond instantiate/destroy.** `set_parent`, `set_enabled`,
  `set_transform` and every other existing `IWorldEditor` mutator remain synchronous. Nothing in Phase
  3's placeholder text (§6) names a hook that needs a deferred reparent or a deferred flag flip, and
  adding queued variants speculatively is exactly the kind of surface this document's Phase 2 audit
  (§5.0) already argues against building before a real caller needs it.
- **No public `flush_deferred_commands()`.** It is called from exactly one place, `step_once()`, and
  there is no caller — editor or otherwise — that should ever trigger a flush outside the tick that
  owns it.

### §5.6 Testing

All new coverage lives in `tests/integration/test_entity_lifecycle.cpp`, alongside Phase 1's cases,
since every case here depends on Phase 1's `enabled`/`enabled_in_hierarchy` machinery being present. A
test double registered with `schedule_.run()`'s `Schedule` stands in for "code running mid-tick" —
Phase 3 does not exist yet to provide a real one, but the queue's safety claim is specifically about
being called from inside that phase, so the test has to call it from there.

- `RequestDestroyMidTickRemovesTheEntityBeforeThatTicksExtract` — a system queued into `schedule_`
  calls `request_destroy` on a sibling entity; after one `tick()`, the entity is gone from `exists()`
  and absent from the extracted render scene, with no crash from `step_once()`'s own `order_` walk that
  ran earlier in the same tick.
- `RequestDestroyMidTickCascadesToChildren` — the same setup, targeting a parent with children; all
  descendants are gone after the same tick, proving `destroy_without_extract`'s recursion still runs
  correctly from the deferred path.
- `RequestInstantiateMidTickCreatesTheEntityBeforeThatTicksExtract` — a system calls
  `request_instantiate`, capturing the returned id; before `flush_deferred_commands()` (mid-tick),
  `exists(id)` is `false`; after the same `tick()` returns, `exists(id)` is `true` and the entity
  appears in that tick's extracted render scene.
- `RequestDestroyOfAPendingInstantiateCancelsItOutright` — one system calls `request_instantiate`,
  captures the id, and in the same tick another queued call requests `request_destroy(id)` before flush;
  after `tick()`, `exists(id)` is `false` and no `Record` was ever created for it (observable via the
  existing scene/render-extraction assertions Phase 1's tests already use to prove absence).
- `RequestDestroyIsIdempotentWithinATick` — two `request_destroy(id)` calls for the same live entity in
  one tick do not crash and produce exactly one removal.
- Regression: every existing Phase 1 test in this file, and every existing `create()`/`destroy()`-based
  test elsewhere in the suite, is expected to pass unchanged — neither method's public signature or
  synchronous behavior changes.

### §5.7 Documentation

Per `docs/CONTRIBUTING.md` §5, the same PR updates:

- `docs/architecture/domain-physics.md` §1.5 — a new bulleted entry beside Phase 1's, documenting
  `request_instantiate`/`request_destroy` as the deferred, tick-safe counterpart to `create`/`destroy`,
  and naming the one flush point in `step_once()`.
- `engine/world/simulation/README.md` — the module's owned-behavior list gains the deferred
  instantiate/destroy surface.

## §6 Phase 3 — Native lifecycle hooks (placeholder)

An `OnEnable`/`OnDisable`/`OnSpawn`/`OnDestroy` callback surface, dispatched in native C++ against
whatever component carries behavior — not a scripting VM (§2). Ordered after Phases 1 and 2 because
its four hooks are named for, and fire from, the events those phases make real: `OnEnable`/`OnDisable`
fire from Phase 1's `enabled` transitions, `OnSpawn`/`OnDestroy` from Phase 2's instantiate/destroy.
Building this first would mean hooking events that do not yet exist. Specified in a later revision.

## §7 Phase 4 — Inter-object messaging (placeholder)

Entities invoking named methods on each other, built on the dispatch mechanism Phase 3 establishes for
lifecycle hooks — the two are the same underlying capability (call a named thing on an entity) aimed
at two different callers (the engine, for hooks; another entity, for messages). Ordered last because
it has no dispatch surface to build on until Phase 3 exists. Specified in a later revision.
