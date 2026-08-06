# Prefab System — an authored entity subtree as an asset (`SushiEngine::Scene`)

**Status:** designed, 2026-08-06 (§11).

A scene is the only place an entity composite can live. Building a street light out of a post, a
lamp, a light and a collider means building it once per placement, and changing it means changing
every copy by hand. There is no way to author a composite once and reuse it, and no way to place a
model such that editing the source updates what was placed.

This document specifies the first phase of a prefab system: a subtree saved as an asset, instances
placed in scenes, and instances that rebuild themselves when the asset changes. It deliberately does
not specify override resolution, which is the phase that follows and the reason this one is drawn
where it is.

It also takes over a responsibility another document declined. `model_import.md` §4.3 withdrew the
link from a placed subtree back to the glTF it came from, on the grounds that a prefab system owns
it whole. §7 here is that link.

Companion docs: `model_import.md` §4.3 and §12, whose withdrawn section this document completes;
`editor_ux_overhaul.md`, whose wire-or-remove rule §8 applies.

---

## §1 Audit — what exists, and what it gives this for free

- **The scene serializer already produces exactly the shape a prefab file needs.** `capture_scene`
  (`engine/world/serialization/include/SushiEngine/serialization/scene_serializer.hpp`) returns
  `{ "entities": [...], "environment": {...} }`, and `apply_scene` restores from it. A prefab is one
  rooted slice of that entity array. Nothing about prefab serialization has to be written from
  scratch, and the format a prefab is written in is one that is already exercised on every save,
  every undo step and every play-mode transition.
- **That machinery is hotter than it looks.** The same two functions carry undo and redo and the
  play-mode enter/exit snapshot, not just file save. §4's choice of instance representation is made
  to leave both untouched.
- **Entities have no persistent identity.** `capture_scene` writes parentage as an *array index*
  (`engine/world/serialization/source/scene_serializer.cpp:584`,
  `entry["parent"] = index_of.at(parent_id)`), and `EntityId`
  (`engine/world/simulation/include/SushiEngine/simulation/simulation.hpp:77`) is a runtime
  `std::uint64_t` that does not survive a save and load. Unity keys every prefab override on a
  persistent file-scoped identifier; this codebase has no equivalent. That absence is the whole
  reason override resolution is a separate phase (§10) rather than a section here.
- **The hierarchy the instance needs is already built.** `IWorldEditor::parent`/`set_parent`
  (`simulation.hpp:1250-1263`) maintain the tree and reject cycles.
- **The drag source a prefab is authored through already exists.** The Hierarchy panel publishes a
  `HIERARCHY_ENTITY` payload for every row
  (`applications/editor/source/scene/hierarchy_panel.cpp:146`). §6 adds the target, not the
  protocol.
- **Content hashing is an established mechanism here.** The cooking pipeline keys cached assets by
  content hash (`engine/domain/physics/include/SushiEngine/physics/cooking/cooked_asset_store.hpp`),
  which is what §3's revision follows rather than inventing a scheme.

## §2 Non-goals

Each of these is a later phase, and naming them is what keeps this one finishable.

- **No override resolution.** An instance is exactly its prefab. Edits made inside an instance are
  lost when it rebuilds, and §8 requires the editor to say so before the click rather than after.
  This is the phase that needs entity identity (§1) and it is deferred until identity is designed.
- **No nested prefabs.** A prefab whose source subtree contains an instance of another prefab is
  flattened when it is authored, and §8 reports it. Nesting needs override resolution to be
  meaningful, so it follows it.
- **No prefab edit mode.** There is no isolated view for editing a prefab. It is edited by placing
  an instance, changing it, and authoring over the same path.
- **No runtime instantiation API.** Nothing outside the editor creates an instance. Gameplay
  spawning is a separate surface with its own lifetime and determinism questions.
- **No change to how a scene file is written or read.** §4 adds one component, serialized beside
  the existing ones; `capture_scene` and `apply_scene` keep their current meaning, and the undo,
  redo and play-mode paths that run through them are untouched.

## §3 The prefab asset

A `.sushiprefab` file:

```json
{
  "revision": "<content hash of the entity array>",
  "entities": [
    /* capture_scene's own entity records, each with one field added:
       "prefab_entity_id": a value unique within this file and stable across
       reads. Parent stays an array index, as capture_scene writes it. */
  ]
}
```

The added `prefab_entity_id` has **no consumer in this phase** and is written anyway. §4.4 states
why.

`entities[0]` is the root, and every other entry's `parent` chain reaches it. The array is exactly
what `capture_scene` produces for the subtree, so `apply_scene`'s reader is reused rather than
duplicated, and a field added to an entity record is carried by prefabs the day it is added to
scenes.

`revision` is a hash of the entity array's serialized bytes, not a counter. Two machines that author
the same subtree produce the same revision, and reverting a prefab to a previous state restores its
previous revision rather than advancing past it — which is the behaviour §5's staleness comparison
needs, and which a counter gets wrong in both cases.

The environment is not written. A prefab is a subtree, not a scene, and a street light carrying a
copy of the sky it was authored under would apply that sky to every scene it is placed in.

## §4 The instance

One component, on the instance's root entity and on no other:

```cpp
namespace SushiEngine::Simulation
{
    struct PrefabInstanceParameters
    {
        std::string path;       /**< The .sushiprefab this subtree was built from. */
        std::string revision;   /**< The prefab's revision at the time it was built. */
    };
}
```

It gets the accessor triple every other component has on `IWorldEditor` — `has_prefab_instance`,
`prefab_instance`, `set_prefab_instance` — and a block in `scene_serializer.cpp` beside the
existing ones.

**A scene file stores the instance's entities in full, expanded, exactly as it stores any other
entity, plus this component on the root.** That is the whole of the representation, and it is chosen
for what it does *not* disturb: `capture_scene` and `apply_scene` keep their current meaning, so
undo, redo and the play-mode snapshot are untouched by this document. A representation that stored
only a reference would have to teach both functions to collapse and expand a subtree, which means
changing the code path that runs on every undo step to serve a feature that runs when a scene is
opened.

It also fails better. A prefab file that has gone missing leaves the placed entities intact and
merely unlinked; a reference-only scene would lose the objects entirely.

### §4.4 Identity, and the one thing this phase owes the next

An override — the whole subject of the phase after this one — has to name what it overrides. This
codebase gives it nothing to name with: an entity's parentage is written as an array index
(`scene_serializer.cpp:584`) and `EntityId` is a runtime value that does not survive a save (§1).

The scope of that problem is narrower than it first looks. **An override is always relative to a
prefab.** Saying "this instance's `Tire` uses a different material" requires telling that `Tire`
apart from the prefab's other entities — not from every entity in the project. So identity belongs
to the **prefab file**, not to the scene, and the scene's entity records need no new field at all.
That is why `.sushiprefab` carries a `prefab_entity_id` per entry (§3) and a scene carries none.

The alternatives were weighed and rejected on one criterion — which of them makes a P1 decision
irreversible:

- **A sibling-index path** (`2/0/1`) costs nothing to store and breaks the moment a node is inserted
  into the prefab: every override past it shifts and silently applies to the wrong entity. Silent
  misapplication is the most expensive failure class in this codebase, and it is what §8 exists to
  prevent elsewhere.
- **A name path** (`Body/Wheel_FL/Tire`) is readable and costs nothing, but a rename severs the
  override, and §5's own sibling naming permits two cousins to share a name.
- **A scene-wide identifier on every entity** is what a project needs once one scene references
  another. Nothing does yet, and adopting it here would widen every entity record and disturb the
  save, undo and play-mode paths §4 is specifically shaped to leave alone.

So this phase writes an identifier it does not read. That is a deliberate exception to this
document's own YAGNI, and the reason is asymmetric cost: the field is one value per entity and is
discarded in a day if the scheme turns out wrong, whereas a prefab format shipped without it means
every prefab authored before override resolution arrives is unmatchable afterwards.

**What this phase does not solve, and must not pretend to:** preserving an identifier across a
*re-author*, when a prefab is written again from an edited instance. That needs a correspondence
between the old file's entities and the new one's, which is override resolution's problem and is
designed with it. In this phase a re-author may assign fresh identifiers, because nothing reads
them yet.

## §5 Refresh on load

After `load_scene` finishes, one pass over the world: for every root carrying
`PrefabInstanceParameters`, read the prefab's current revision and compare. When they differ,
destroy the subtree below that root, rebuild it from the prefab, and write the new revision.

**The root's own name and transform are preserved.** They are the instance's placement, not the
prefab's content, and a rebuild that moved every street light back to the origin would be a rebuild
nobody could use.

The pass runs in `load_scene` and **not** in `apply_scene`. This is the detail that is easiest to
get wrong and the most damaging to get wrong: `apply_scene` is the path undo restores through, and
refreshing a prefab during an undo would reinstate the very change the user is undoing. `load_scene`
is the only entry point where "the world is being populated from a file the user just chose" is
true.

## §6 Authoring a prefab

Dragging an entity from the Hierarchy panel onto the Project panel writes `<name>.sushiprefab` into
the browsed folder, and **converts the dragged entity into an instance of what it just wrote** —
which is the behaviour that makes the gesture reversible in the user's head, because what is left
selected is the thing they will edit next.

The payload already exists (§1); the Project panel gains the drop target. A name collision
disambiguates with the `" (n)"` convention `unique_child_path`
(`applications/editor/source/project/project_panel.cpp:156-163`) already applies to new files rather
than overwriting an existing prefab.

## §7 Where model import connects

`plan_model_instantiation` (`model_import.md` §5) already produces a list of entities with their
parents, transforms and components. That list is the content of a prefab file. So the importer
writes a `.sushiprefab` instead of creating entities directly, and dragging a `.gltf` into a scene
places an instance of it.

Reimport is then not a feature anyone writes. Changing an asset's `.meta` changes the plan, which
changes the prefab, which changes its revision, and §5 rebuilds every instance in every scene the
next time it is opened. The gap `model_import.md` §4.3 left open closes without a line of code that
exists only to close it.

The generated prefab is written beside the source asset as `<asset>.sushiprefab` — the whole path
with the extension appended, the same convention `.meta` follows (`model_import.md` §4.2), so
`models/Car.gltf` produces `models/Car.gltf.meta` and `models/Car.gltf.sushiprefab`, and a `.glb`
and a `.gltf` of the same name in one directory cannot collide. It is a real file that can be
opened, inspected and version-controlled like any other, rather than a hidden generated artefact.

## §8 Errors and what the editor must say

- **A missing or unreadable prefab** leaves the instance's entities where they are, marks the root
  unlinked, and reports the path. Nothing disappears.
- **A rebuild** states how many entities it replaced, before it runs, on the control that triggers
  it. Edits inside an instance are lost (§2) and the artist is told that in the tooltip, not
  discovered afterwards.
- **A flattened nested instance** is reported by name when a prefab is authored from a subtree that
  contained one (§2).
- **No control that cannot do what it says.** `editor_ux_overhaul.md`'s wire-or-remove rule applies
  in full: there is no disabled "Override" affordance hinting at §10, because a control that
  advertises a capability the build does not have is the failure that rule exists to prevent.

## §9 Testing

The weight sits where it can run without a device:

- Prefab write, read and compare: a subtree round-trips through `.sushiprefab` with its parentage,
  names and transforms intact.
- Revision stability: identical content hashes identically; a changed transform, an added entity and
  a reordered array each change it.
- Refresh: a scene carrying an instance at a stale revision rebuilds it, and the root's name and
  transform survive; an instance at the current revision is left untouched, which is the case a
  naive implementation gets wrong by rebuilding unconditionally.
- **`apply_scene` does not refresh.** An undo through a snapshot that holds a stale instance leaves
  it stale. This is §5's whole hazard and it gets its own case.
- A missing prefab leaves the entities in place, marks the root unlinked and does not crash.

## §10 The later phases, and what each needs from the one before

Designed only as far as their boundaries and their prerequisites. Each gets its own document and
its own implementation plan; what is fixed here is the order and what each may assume.

**P2 — Override resolution.** An instance records how it differs from its prefab, and a rebuild
preserves those differences instead of discarding them. Needs from P1: `prefab_entity_id` (§4.4),
which P1 writes. Owns, and must design: how an override is stored in the scene (a list on the
instance root keyed by identifier and field), what happens to an override whose target no longer
exists in the prefab, how an identifier survives a re-author, and the Inspector affordances that
show a value as overridden and revert it. This is the phase that turns §2's "edits are lost" into
"edits are kept", and it is the largest of the four.

**P3 — Nested prefabs and prefab edit mode.** A prefab whose content includes an instance of
another, and an isolated view for editing a prefab without placing it. Needs from P2: override
resolution, because a nested instance's overrides compose with its parent's and there is nothing to
compose before P2 exists. P1 flattens a nested instance and reports it (§2); P3 is what stops
flattening.

**P4 — Runtime instantiation.** Gameplay creating an instance during a frame rather than an artist
creating one in the editor. Needs from P1: the asset format and the instantiation path. Owns:
lifetime, pooling, and whether instantiation is deterministic enough for the fixed-step loop —
questions that belong to the simulation, not to authoring, which is why this is last rather than
second despite being independent of P2.

**What none of them changes:** §4's decision that a scene stores an instance expanded. P2 adds an
override list beside the instance component; it does not move to storing a reference. That keeps
`capture_scene` and `apply_scene`, and therefore undo, redo and the play-mode snapshot, untouched
by the whole roadmap rather than only by its first phase.

## §11 Roadmap

P1 — §3 to §9 — **designed, not started.**

What P1 delivers: a subtree authored as an asset, instances placed from it, instances refreshed when
the asset changes, and model import producing prefabs rather than loose entities. What it does not
deliver, stated so it is not mistaken for an oversight: no overrides, no nesting, no prefab edit
mode, no runtime API, and edits made inside an instance do not survive a rebuild.
