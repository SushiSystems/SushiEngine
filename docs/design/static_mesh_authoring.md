# Static Mesh Authoring — an imported asset as a scene prop (`SushiEngine::Render`)

**Status:** shipped, 2026-08-05 (§11).

An artist can import a glTF today only through two narrow doors: `Crowd`, for a rigged, animated
character, and the physics cooking pipeline, which turns a mesh into a collider, a tetrahedral soft
body, or a node-beam vehicle shell — never into a plain visual object. There is no way to place an
ordinary static prop (a car body, a rock, a building) in the scene at all. This document specifies
the small addition that closes that gap: an "Imported" mesh kind on the Renderer component that
already exists, wired to render and import machinery that is already built.

This is a render-and-authoring change only. It does not touch the physics cooking pipeline —
a static mesh placed this way has no collider and no simulated body unless the entity separately
carries the existing, independent `Rigid Body`/`Collider` components (§8) — and it does not touch
`Crowd`'s animated path.

Companion docs: `docs/architecture/presentation-render.md` (the frame graph this feeds),
`docs/design/physics_system.md` §8 (the cooking pipeline this document explicitly does not change),
and `docs/design/editor_feature_sync_gaps.md` (the audit convention this document's §1 follows — a
claim is a file:line, not a description).

---

## §1 Audit — what exists today, and where it stops

Traced end to end, file:line, rather than assumed from either component's name.

- **The render side is fully built and already consumes an imported mesh.** `Render::MeshInstance`
  (`engine/presentation/render/include/SushiEngine/render/scene_view.hpp:103-124`) carries a `mesh`
  field: "An imported mesh to draw instead of the primitive named by `kind`... When set, `kind` and
  `shape_parameters` are ignored." Every consumer already branches on it —
  `engine/presentation/render/source/passes/opaque_pass.cpp:356,474-476,535-537` and
  `engine/presentation/render/source/scene/instance_system.cpp:86-88` all read
  `instance.mesh != INVALID_MESH` and fetch the imported mesh instead of a procedural primitive.
  Instancing is already geometry-grouped:
  `engine/presentation/render/source/passes/opaque_pass.cpp`, "Instances draw grouped by geometry so
  each mesh's buffers are bound once per group rather than once per instance."
- **The import call is fully built and has zero callers.** `Render::IAssetLibrary::load_gltf`
  (`engine/presentation/render/include/SushiEngine/render/asset_library_interface.hpp:118`) imports
  a glTF's meshes and materials generically (not skinned). Its implementation
  (`engine/presentation/render/source/material/asset_library.cpp:93`) is exercised by nothing
  outside the render module itself — `grep` for `.load_gltf(` outside `engine/presentation/render/`
  matches only `SkeletonPreview::load_gltf`, an unrelated method on a different class.
- **The authoring side never reaches either.** `Simulation::RenderInstance`
  (`engine/world/simulation/include/SushiEngine/simulation/simulation.hpp:122-131`), the
  world-tier struct the editor extracts every frame, carries no `MeshId` field at all. The copy
  from it into a `Render::MeshInstance` (`applications/editor/source/main.cpp:489-501`) sets
  `model`, `color`, `id`, `kind`, `shape_parameters`, `material` — never `mesh`, which is why it
  stays `INVALID_MESH` on every instance today.
- **`ShapeParameters` — the "Renderer" component's own data — only knows three procedural
  primitives.** `PrimitiveKind`
  (`engine/world/simulation/include/SushiEngine/simulation/components.hpp:256-262`) is
  `Box`/`Sphere`/`Cylinder`/`Plane`, shared between `ShapeParameters` (visual) and
  `ColliderParameters` (physics). The Inspector's Renderer section
  (`applications/editor/source/scene/inspector_panel.cpp:602-648`) offers only the first three as a
  mesh choice (`MESH_NAMES[] = {"Box", "Sphere", "Cylinder"}`, `:616`) — its own comment calls the
  Renderer "Unity's MeshFilter folded into the MeshRenderer here" (`:602-605`).
- **No creation path exists.** `Create ▸ Objects`
  (`applications/editor/source/scene/scene_commands.cpp:67-124`) offers
  Box/Sphere/Cylinder/Terrain/Cloth/Crowd/Particle System/Light/Decal — no "Mesh" or "Prop".
  `applications/editor/source/project/project_panel.cpp` registers no
  `ImGui::BeginDragDropSource` at all; the only drag-drop payload in the editor is
  `HIERARCHY_ENTITY` (`applications/editor/source/scene/hierarchy_panel.cpp:144-153`), for
  reparenting entities that already exist.
- **The precedent to copy already exists, one component over.** `Crowd` imports a mesh from a path
  exactly this way, minus skinning: `CrowdParameters::mesh_path`
  (`engine/world/simulation/include/SushiEngine/simulation/simulation.hpp`) is bound by
  `bind_crowd_mesh()` (`applications/editor/source/scene/inspector_panel.cpp:144-165`), which calls
  `context.assets->load_gltf_skinned_mesh(...)` and writes the returned `MeshId`/`Material` back
  onto the component. `engine/world/serialization/source/scene_serializer.cpp` re-runs the same bind
  on scene load, so a `mesh_path` survives a save/load round trip without the id it resolved to
  needing to be persisted.

## §2 Non-goals

- **No new component.** `ShapeParameters` already owns "what does this entity draw"
  (`applications/editor/source/scene/inspector_panel.cpp`); a second, competing component would
  split that responsibility for no reason. See §3 for why the extension does not touch
  `PrimitiveKind`.
- **No collider convenience.** Placing a mesh does not create or suggest a collider. `Collider` is
  already independent of the Renderer's mesh
  (`applications/editor/source/scene/inspector_panel.cpp`, "The collision volume's shape,
  independent of the Renderer's mesh") and stays that way — an artist who wants a physical body adds
  `Rigid Body`/`Collider` separately, unchanged by this document.
- **No scatter/instancing authoring tool.** "Instancing" here means the render pass's existing
  per-geometry grouping (§1) applying automatically when several entities share a mesh — not a new
  one-entity-many-transforms component. That is a materially larger, separate tool and is not needed
  for the problem this document solves.
- **No drag-and-drop from the Project panel in this phase.** Deferred to §10.
- **No change to the cooking pipeline.** A static mesh placed this way is not cooked, has no
  `CookingReport`, and does not appear in the Bake panel's asset list. Cooking is for physics
  representations (collider, soft body, node-beam) — an unrelated concern this document does not
  touch.

## §3 Data model

`PrimitiveKind` stays exactly `Box`/`Sphere`/`Cylinder`/`Plane` — it is not extended, because it
is shared with `ColliderParameters` (§1), and a collider cannot be an arbitrary imported mesh
through this enum without implying a capability (mesh-based collision through the primitive-kind
picker) that does not exist and is not being built here. Instead, `ShapeParameters` gains two
fields, following the same discriminator convention `Render::MeshInstance` already uses — presence
of a mesh overrides `kind`, rather than `kind` gaining a value that means "ignore my own other
field":

```cpp
struct ShapeParameters
{
    PrimitiveKind kind = PrimitiveKind::Box; // meaningful only while mesh == INVALID_MESH
    Vector3 parameters{Vector3{0.5, 0.5, 0.5}};
    std::string mesh_path;                    // glTF path `mesh` was imported from; empty = none
    Render::MeshId mesh = Render::INVALID_MESH;
};
```

`Simulation::RenderInstance`
(`engine/world/simulation/include/SushiEngine/simulation/simulation.hpp:122-131`) gains the matching
field:

```cpp
Render::MeshId mesh = Render::INVALID_MESH; // mirrors ShapeParameters::mesh; INVALID_MESH draws kind
```

No new `has_*` flag: presence is `shape_parameters.mesh != Render::INVALID_MESH`, exactly as
`Render::MeshInstance` already discriminates (§1). An entity cannot simultaneously "be" a Box and an
imported mesh — the field layout makes that state unrepresentable rather than documenting it as an
invariant to maintain by hand.

## §4 Import and binding

A new `bind_shape_mesh(EditorContext&, ShapeParameters&)`, in the same file and shape as
`bind_crowd_mesh()` (`applications/editor/source/scene/inspector_panel.cpp:144-165`):

```cpp
void bind_shape_mesh(EditorContext& context, ShapeParameters& values)
{
    values.mesh = Render::INVALID_MESH;
    if (context.assets == nullptr || values.mesh_path.empty())
        return;
    Render::MeshId meshes[1] = {Render::INVALID_MESH};
    Render::Material materials[1]{};
    if (context.assets->load_gltf(values.mesh_path.c_str(), meshes, materials, 1) == 0)
    {
        editor_log(context, "No mesh imported from '" + values.mesh_path + "'.", LogLevel::Warning);
        return;
    }
    values.mesh = meshes[0];
    // materials[0] is not adopted onto the entity's own Material: an imported prop keeps the
    // Renderer's authored Material (already editable, already serialized) rather than silently
    // overwriting it on every re-import, the one place this deliberately differs from Crowd —
    // a crowd has no other source of material, a Shape always does.
}
```

`load_gltf` imports one primitive into slot 0 (`count = 1`), matching how `bind_crowd_mesh` reads
skin 0 today; a multi-primitive glTF's remaining primitives are out of scope (§10).

## §5 Render wiring

Three sites, all additive, none touching an existing code path's behavior when `mesh` is
`INVALID_MESH`:

1. `engine/world/simulation/source/runtime_simulation.cpp`'s instance-population loop (`:3583-3596`)
   adds `instance.mesh = record->shape_parameters.mesh;` beside the existing
   `shape_kind`/`shape_parameters` copy.
2. `applications/editor/source/main.cpp`'s `RenderInstance → MeshInstance` copy (`:489-501`) adds
   `instance.mesh = source.mesh;`.
3. Nothing else — §1 already confirmed every consumer
   (`engine/presentation/render/source/passes/opaque_pass.cpp`,
   `engine/presentation/render/source/scene/instance_system.cpp`, and by the same mechanism the
   shadow, depth-prepass and SDF/GI passes that also read `frame.draws.instances`) branches on
   `mesh != INVALID_MESH` today.

## §6 Editor UI

- **Inspector.** The Renderer section's mesh picker
  (`applications/editor/source/scene/inspector_panel.cpp`) gets a fourth entry, `"Imported"`.
  Selecting it hides the Box/Sphere/Cylinder dimension editor
  (`applications/editor/source/scene/inspector_panel.cpp`) and shows the same "path field + Load
  button" pair the Soft Body section already uses
  (`applications/editor/source/scene/inspector_panel.cpp`) — same warning-on-empty, same
  `editor_log` on a failed import, for one consistent "how do I attach an asset" pattern across the
  whole Inspector rather than a fourth bespoke one.
- **Create menu.** `applications/editor/source/scene/scene_commands.cpp`'s `Objects` submenu (§1)
  gains `"Imported Mesh"`: creates an empty entity with a Renderer whose `kind` is irrelevant
  because `mesh_path` starts empty — the artist fills the path in the Inspector immediately after,
  same two-step flow Soft Body already established.

## §7 Serialization

`engine/world/serialization/source/scene_serializer.cpp` extends `ShapeParameters`'s existing
write/read (wherever the Renderer's `kind`/`parameters` are already round-tripped) with `mesh_path`,
and re-binds `mesh` on load in the same place `:1478-1495` re-binds `crowd.mesh` — `mesh` itself is
never serialized, only `mesh_path`, so a scene file stays portable across a session that resolves
the same path to a different `MeshId`.

## §8 Physics

Unchanged. `Collider`/`Rigid Body` remain their own components, added and edited independently
(§2). An entity with an imported Renderer mesh and no `Collider` renders and cannot be collided
with, exactly like a `Box`-kind Renderer with no `Collider` today.

## §9 Testing

- `tests/integration/test_scene_serializer_roundtrip.cpp`: an Imported-mesh `ShapeParameters` round
  trip through Save→Load, Undo/Redo — parallel to the Crowd round trip already there (§1) and to the
  soft-body one `capture_scene`'s fix added (`docs/reference/changelog.md`, 2026-08-04).
- A unit test on the extraction step (§5): `shape_parameters.mesh` set and `kind == Box` produces a
  `RenderInstance`/`MeshInstance` with the imported mesh, proving `kind` is ignored rather than
  merely unread by convention.
- A unit test on `bind_shape_mesh`: an unresolvable path leaves `mesh == INVALID_MESH` and logs a
  warning, without touching the entity's `Material`.

## §10 Future work, explicitly deferred

- **Drag-and-drop from the Project panel.** Dropping a `.gltf`/`.glb` tile onto the viewport or
  Hierarchy creates an entity the way `"Imported Mesh"` does today, skipping the manual path
  typing. Needs a new `ImGui::BeginDragDropSource` on Project panel asset tiles (none exists
  today, §1) and a drop target on the viewport (none exists today either) — a real, separate
  piece of work, not a small addition to this one.
- **Multi-primitive glTF import.** `load_gltf`'s `count` parameter already supports importing more
  than one primitive; binding all of them (e.g., one entity per primitive, auto-parented) instead of
  only slot 0 is deferred until an asset that needs it is the actual motivation, per YAGNI.

## §11 Roadmap

P0 — this document's entire scope (§3-§9) — **complete**, built via
`docs/superpowers/plans/2026-08-05-static-mesh-authoring.md`, five tasks plus a final-review fix
wave, all task-scoped and reviewed.

The final whole-branch review found and closed seven real gaps this document's illustrative code
left open: the Renderer's Mesh combo's switch to "Imported" mode had been reworked as a hand-rolled
`ImGui::Combo` that recorded no undo step, did not mark the scene dirty, and wrote only the primary
entity of a multi-selection, silently skipping the rest; switching back to a primitive kind left
`ShapeParameters::mesh_path` populated, so `resolve_scene_assets` silently re-imported it after a
Save→Load — the exact "invalid state" §2 calls unrepresentable; the player's identical
`RenderInstance`→`MeshInstance` copy loop (`applications/player/source/player_app.cpp`), a second
copy of the extraction §1 only ever audited in the editor, was missed entirely and never carried
`mesh` either; `PhysicsSourceEntity`
(`engine/world/simulation/include/SushiEngine/simulation/physics_extract.hpp`) embedded the whole
`ShapeParameters` by value and paid a heap allocation per entity per fixed tick for a `mesh_path`
the physics extract never reads; `resolve_scene_assets`'s Shape block ran `set_shape_parameters` —
and the full extract it triggers — for every Shape in the scene on every load, unguarded, unlike the
Material and Decal blocks beside it; the Source Mesh field committed to the world on every keystroke
instead of on a deliberate action, unlike Crowd's and Soft Body's equivalent fields; and
`AssetLibrary::load_gltf` had no path-keyed cache, so pressing Load twice leaked a mesh and ten
entities importing the same file never actually shared one `MeshId`, defeating the per-geometry
instancing §1 describes as already grouped. All seven are fixed on the shipped branch; `load_gltf`
now caches by path the same way `TextureLibrary::load` already does for textures, and the Mesh combo
hides "Imported" during a multi-selection rather than fanning out a per-entity asset choice — the
one interaction detail that changed during implementation.
