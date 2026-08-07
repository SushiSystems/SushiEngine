# Project panel prefab thumbnails (Phase 4)

**Status:** designed, 2026-08-07.

## Problem

`EntryCategory::Prefab` (`.sushiprefab`) entries in the Project panel's Grid view still always show
Phase 1's static hexagon glyph. Unlike a model file, a prefab has no single well-defined "the mesh
it looks like" — it is a captured entity subtree, and rendering it faithfully means rendering
*exactly the entities it actually contains today*, not a proxy for them.

## Why Phase 3a's renderer cannot be reused directly

Confirmed by reading the actual serializer/instantiation code
(`engine/world/serialization/source/prefab_serializer.cpp`,
`engine/world/serialization/source/scene_serializer.cpp`): a `.sushiprefab` file is a JSON snapshot
of an arbitrary entity subtree, produced by `Scene::capture_prefab` from whatever entity a user
drags from the Hierarchy panel onto the Project panel. Each entity in that subtree carries its own
independent `Shape` (`mesh_path` + `source_node` + `primitive` — a file, a node inside it, and a
primitive inside that node), its own material, and its own local transform composed against its
parent. Nothing in the format ties the whole prefab to one source file.

The common case — a prefab written by the model-import pipeline (`write_model_prefab`) — does happen
to give every entity the same `mesh_path` (the imported glTF), but that is a special case of the
format, not a guarantee. A hand-authored prefab (the streetlight example from
`docs/design/prefab_system.md`: a post, a lamp, and a bulb, each possibly imported from a different
file) is a genuine multi-file scene graph. And even the common case can drift from its source file
after import — an artist can nudge a child's transform, override a material, or delete an entity —
at which point re-rendering the *original glTF* would show something the prefab no longer is.

So this phase needs a renderer that draws the prefab's actual, current, resolved entity data — not
one that re-imports a file.

## Architecture

A new render-tier primitive, `IPrefabThumbnailRenderer` (parallel to Phase 3a's
`IMeshThumbnailRenderer`, added the same way via `IWindowRenderer::create_prefab_thumbnail_renderer()`),
whose one method takes a `.sushiprefab` file path and produces the same `FrameImage` output Phase 3a
already established. Its `render_thumbnail` does, per call:

1. **Instantiate into a throwaway world.** Create a fresh, disposable `IWorldEditor` via
   `SushiEngine::Simulation::create_simulation()` — the exact pattern
   `engine/world/model_import/source/prefab_output.cpp`'s `write_model_prefab` already uses for a
   scratch world with no live editor session attached. Parse the prefab's JSON and call
   `Scene::apply_prefab(*scratch_world, document, NULL_ENTITY)`, then
   `Scene::resolve_scene_assets(*scratch_world, *isolated_assets)` (see below) — the same two-call
   sequence `scene_commands.cpp`'s `place_model_instance` already uses to turn a prefab document into
   fully resolved, drawable entities. This scratch world is created fresh for every render and
   discarded at the end of the call — cheap, since it holds no GPU resources, and it sidesteps any
   entity-lifecycle bookkeeping a persistent scratch world would otherwise need between unrelated
   prefabs.
2. **Compute the union bounding box.** Walk every entity the scratch world now holds, compose each
   one's world transform up its parent chain, and accumulate its mesh's local bounding box (the
   same per-vertex accumulation Phase 3a's extended `import_gltf` already does, generalized to run
   per entity instead of per whole-file import) into one `SushiEngine::Geometry::AABB3` covering the
   whole prefab. Feed that into `three_quarter_camera_for_bounds` — reused completely unmodified
   from Phase 3a, since it was already written as a pure function of an `AABB3`.
3. **Draw every entity flat/unlit, each with its own model matrix.** Phase 3a's shader pair
   (`mesh_thumbnail.vert`/`.frag`) is reused verbatim — flat headlight-plus-ambient, one base-color
   texture sample — but the push constant's `model` field, which Phase 3a's fix round *removed* as
   permanently-identity dead weight (every glTF import bakes its transform in), comes back here with
   real content: each entity's own composed world matrix. One draw call per entity, same pipeline,
   same isolated bindless heap.
4. **Read back and return**, identically to Phase 3a's own readback sequence.

**Isolated, persistent asset stack, periodically recreated.** The actual GPU-resident cost — mesh
and texture data pulled in by `resolve_scene_assets` — is owned by a `SamplerCache`/`DescriptorHeap`/
`MeshRegistry`/`TextureLibrary` quartet built the same way Phase 3a's isolated stack is, and shares
its unbounded-growth constraint (no removal API) and its mitigation (periodic destroy-and-recreate).
Because a prefab can pull in more distinct files and entities per render than a single model import,
the recreation threshold is a smaller **24** renders (versus Phase 3b's 64), tuned conservatively
until real usage says otherwise. This stack is what `resolve_scene_assets` resolves against — it is
effectively this renderer's own private `IAssetLibrary`, exactly mirroring how Phase 3a's isolated
stack already stood in for one.

**Entity/primitive limits.** A fixed cap of **64 entities** and **128 drawable primitives** total
per prefab (a prefab's entities can each carry more than one primitive, hence the higher primitive
ceiling relative to Phase 3a's single-file `MAX_PRIMITIVES = 64`) — a prefab exceeding either is
treated as a load failure, per the established failure policy below, not a partial render.

## `PrefabThumbnailCache`

A new editor-tier class, structurally identical to Phase 3b's `ModelThumbnailCache` — same
synchronous per-frame budget of one render, same 128×128 output resolution, same **128**-entry
resident LRU with the same delayed-eviction shape, same permanent-failure policy split between a
silent render failure and a logged-once upload failure, same non-owning `EditorContext` pointer
field and construction/destruction ordering in `main.cpp`. The only structural difference is which
render-tier interface it holds (`IPrefabThumbnailRenderer` instead of `IMeshThumbnailRenderer`) and
its recreation threshold (24, per above).

## Integration

`project_panel.cpp`'s Grid view gains a third branch beside the existing `Image` and
`Model`-with-`.gltf`/`.glb` ones: for a visible `EntryCategory::Prefab` entry, ask
`PrefabThumbnailCache::texture_for(path)`. `EntryCategory::Prefab` already means "the file is
`.sushiprefab`" (Phase 1's classifier has no further extension branching within this category), so
there is no extension gate to add here the way Phase 3b needed one for `.fbx`/`.obj`.

## Error handling

A render failure (corrupt/unparseable JSON, a `mesh_path` that no longer resolves to a file, more
entities/primitives than the fixed cap) is silently permanent for that path, exactly like Phase 3a's
glTF-load failure — the hexagon glyph keeps showing forever, no log, never retried. An upload
failure (the GPU-texture-creation step after a successful render) is logged once via
`Console::append(LogLevel::Warning, ...)` and is also permanent for that path — identical to every
prior phase's policy.

## Testing

As with Phase 3a and 3b, none of this is reachable from the test binary (Vulkan, a live-ish
`IWorldEditor`, and ImGui all in the loop). The one candidate for GPU-independent unit testing is
the composed-world-transform accumulation into a union `AABB3` across a small entity hierarchy —
this can be extracted as a pure function (given a list of `{local_transform, parent_index,
local_bounds}` records, compute each entity's world-space bounds and their union) and tested the
same way Phase 3a's own bounding-box math was, without needing a real `IWorldEditor` or any Vulkan
device.

## Explicitly out of scope (this phase)

- Any prefab whose subtree contains non-mesh visual elements this renderer doesn't already know how
  to draw (particle effects, lights, decals) — those entities' *meshes* (if any) still draw; their
  other visual contributions are not represented in the thumbnail, matching the flat/unlit fidelity
  ceiling every phase of this feature has already accepted.
- Nested prefab instances that themselves reference other prefab files recursively — resolved only
  as deep as `Scene::apply_prefab`/`resolve_scene_assets` already resolve them today, with no new
  recursive-prefab-loading logic added by this phase.
- Any camera angle, lighting, or resolution control beyond what Phase 3a already fixed.
