# Entity Lifecycle System — real enabled/disabled, spawn/destroy, and native hooks (`SushiEngine::Simulation`)

**Status:** designed — Phase 1 (enabled/disabled) specified below; Phases 2-4 are named and
ordered but not yet specified in detail.

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

Only Phase 1 is specified in this revision. Phases 2-4 are placeholders (§5-§7) naming what each
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
| 2 | Runtime instantiate/destroy on the deferred/command-buffer discipline; a gameplay-time spawn/destroy surface | named, not specified |
| 3 | Native `OnEnable`/`OnDisable`/`OnSpawn`/`OnDestroy` hooks | named, not specified |
| 4 | Inter-object messaging on Phase 3's dispatch surface | named, not specified |

---

## §4 Phase 1 — Enabled/disabled lifecycle

### §4.1 Data model

`Record` gains one field: `bool enabled = true;` — Unity's `activeSelf`. `Record::visible` keeps its
current meaning unchanged: a local (non-hierarchical), render-only flag. Because `enabled` is new and
defaults to `true`, every existing scene's behavior is unchanged after this ships — an entity authored
with `visible=false` today continues to be render-hidden only, exactly as it is now. No scene-file
migration is required.

A new private helper, `enabled_in_hierarchy(const Record*)`, is added beside
`visible_in_hierarchy` (`runtime_simulation.cpp:2721-2733`), identical in shape: walks the ancestor
chain, returns `false` if any node (including the record itself) has `enabled == false`.

`visible_in_hierarchy` itself is retired. Its nine call sites in `extract()`
(`runtime_simulation.cpp:3772, 3798, 3872, 3913, 3953, 3980, 4247, 4280, 4422`) become
`!enabled_in_hierarchy(record) || !record->visible` — hierarchy gates existence, the local flag gates
this entity's own render contribution on top of that, matching how a disabled parent's inactive
children never reach a child's own `visible` check in Unity either.

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

## §5 Phase 2 — Runtime instantiate/destroy (placeholder)

Brings `RuntimeSimulation::create()`/`destroy()` (`runtime_simulation.cpp:293, 308`) onto the
`CommandBuffer` deferred/barrier discipline `engine/foundation/ecs` already has
(`command_buffer.hpp:54, 67, 76`), and adds a gameplay-time spawn/destroy surface for whatever calls
into the engine at simulation time — today that is nothing, since there is no scripting VM (§1), but
the surface should not assume the editor is the only caller forever. Ordered after Phase 1 because a
freshly spawned entity's default `enabled` state and an about-to-be-destroyed entity's disable-before-
destroy semantics are natural to define in terms of Phase 1's flag, not before it exists. Specified in
a later revision of this document, before its own implementation begins.

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
