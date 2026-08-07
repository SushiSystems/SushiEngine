# Prefab override resolution (P2) — design

**Date:** 2026-08-08
**Phase:** `docs/design/prefab_system.md` §10, P2 — "the largest of the four"
**Status:** approved

## 1 What this builds

An instance keeps its local edits when the prefab it came from changes. Today
`refresh_prefab_instances` destroys the whole subtree and rebuilds it from the file, which is what
§2 means by "edits are lost".

## 2 What the tree actually has

`prefab_entity_id` is written by `save_prefab`
(`engine/world/serialization/source/prefab_serializer.cpp`) and **read by nothing**. `apply_prefab`
never looks at it, and nothing in the world remembers which prefab entity an instantiated entity
came from — so there is no key an override could be attached to.

The id is also **positional**, `"e" + index`. An artist inserting an entity mid-list shifts every id
after it. The comment explaining that choice reasons from the revision hash: the revision is a
content hash of the whole entities array, so "anything varying between two captures of the same
subtree would change the revision with it". That reasoning holds only for ids *minted afresh on
every save*. An id minted once and **preserved** on re-save leaves the hash exactly as stable.

And there is **no component reflection** — no registry, no field enumeration. `ComponentEditor` uses
compile-time member pointers. Unity and Godot do field-level overrides because both have reflection;
this engine does not.

## 3 The two decisions

**Overrides are component-level, not field-level.** No reflection is needed, the granularity matches
what `capture_scene`/`apply_scene` already move, and serialization reuses the writers that exist.
The cost is stated plainly: overriding an entity's position pins its whole `Transform`, so a later
prefab change to its scale will not reach that instance.

**Overrides are computed, not stored.** This is the decision that shapes the implementation, and it
came out of asking how a stored list would ever be *filled*: something would have to record "this
was overridden" at the moment of the edit, which means `set_transform`,
`set_physics_body_parameters`, `set_collider_parameters` and every future component setter would
have to know about prefabs. A new component whose author forgets that hook produces a system that
silently loses overrides, with nothing failing — the exact class of defect this codebase keeps
producing.

Computing them removes the possibility. At rebuild time the live member is serialized and compared
against the prefab's record for the same id; the components that differ **are** the overrides. There
is no list to drift, no hook to forget, and a component added tomorrow is covered by construction.

## 4 How a rebuild works

```
1  collect the subtree's members, each with its prefab_entity_id and its current record
2  load the prefab document; index its entries by id
3  matched member   → diff component keys against its entry; differing keys are the patch
   unmatched member → a survivor (see §5)
4  patch the document: for each entry with a patch, overwrite those component keys
5  destroy the matched members, bottom-up
6  instantiate from the patched document
7  reparent the surviving roots under the rebuilt root
```

Step 4 is why no new reader is needed. `read_entity_record` creates an entity from a record and
cannot apply one to an existing entity — but nothing has to. The document is patched *before*
instantiation, so the rebuild reads file-plus-patch and the existing path does the rest.

## 5 A member with no match

Two ways a live member can fail to match: its id is gone from the prefab (the artist removed that
entity), or it has no id at all (the author added it to this instance by hand).

Both **survive, unlinked**. The entity stays in the scene under the rebuilt root as an ordinary
entity, and the refresh reports how many were detached. The trade is stated: cleaning an entity out
of a prefab no longer cleans it out of the instances, and an author who wants that removes it
per instance.

The second case is a real improvement rather than a side effect. Today an entity the author added
to an instance is destroyed by the next refresh with no warning at all.

A survivor keeps only its *unmatched* descendants: matched descendants are destroyed and rebuilt
like any other member, and each surviving root — a survivor whose parent is not itself a survivor —
is reparented to the rebuilt root.

## 6 Identity

The world gains a per-entity `prefab_entity_id`, serialized with the scene so it survives save,
load, undo and redo.

`save_prefab` **preserves** an entity's existing id and mints only for entities without one, which
is what makes an id survive a re-author: an artist who reorders a prefab's contents and saves again
keeps every id, so every instance's overrides stay attached to the entity they were made against.
`apply_prefab` sets the id from the record it created the entity from.

## 7 Acceptance

1. An instance whose member was edited keeps that edit through a refresh.
2. A component the instance did **not** edit takes the prefab's new value.
3. An entity removed from the prefab survives in the instance, unlinked, and is reported.
4. An entity the author added to the instance survives a refresh — which it does not today.
5. Re-saving a prefab whose entities were reordered leaves every id unchanged, and an instance's
   overrides stay on the entities they were made against.
6. An instance with no edits refreshes to exactly the prefab, byte for byte in the scene file.

## 8 Out of scope

- **Field-level granularity.** Named and declined in §3; it needs a reflection layer this engine
  does not have, and building one is its own project.
- **Inspector affordances** — showing a component as overridden and reverting it. The diff that
  would drive them is computed by this phase; presenting it is a second, editor-only pass, and the
  UI-first rule means it gets its own review anyway.
- **Nested instances.** P3, which needs this phase first.
