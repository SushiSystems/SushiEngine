# Project panel model thumbnails (Phase 3)

**Status:** designed, 2026-08-07.

## Problem

Phase 1 gave every asset category a procedurally drawn vector glyph; Phase 2 replaced that glyph
with a real decoded image for `EntryCategory::Image` entries in Grid view. `EntryCategory::Model`
entries (`.gltf`, `.glb`, `.fbx`, `.obj`) still always show Phase 1's static wireframe-cube glyph,
identical for every model regardless of its actual geometry. This phase closes that gap for the
formats the engine can actually load.

## Scope

Real thumbnails apply to **`.gltf` and `.glb` only** — confirmed (via codebase research) to be the
only two of the four `EntryCategory::Model` extensions with a working, render-ready import path
today (`Render::Assets::import_gltf`, already used by the Scene view's drag-and-drop). `.fbx` and
`.obj` are recognized by the Project panel's classifier but have no loader anywhere in the engine
— no third-party importer (assimp, ufbx, tinyobj, or similar) is linked. Adding one is a distinct,
much larger piece of work with its own licensing, build, and format-coverage questions, and is out
of scope here. `.fbx`/`.obj` entries keep showing Phase 1's wireframe-cube glyph indefinitely,
exactly as before this phase — there is no attempt to detect or special-case them differently from
today.

As in Phase 2, this applies to Grid view only. List view keeps its fixed generic icon regardless of
category.

## Why this needs new engine surface, not just a new cache class

Phase 2's `ThumbnailCache` could decode an image entirely off the render thread and only touch
Vulkan for the final upload. A model thumbnail cannot: producing one means loading real mesh and
material data and drawing it, which only ever happens through the renderer. Every existing
offscreen-render path in this codebase — the Scene view, the Game view, the Animator Preview, the
Particle Preview — goes through one interface, `IWindowRenderer::create_scene_view() →
std::unique_ptr<ISceneView>`, and `ISceneView::render(...)` is a full-scene call: camera,
environment (sun/atmosphere/clouds/stars), punctual lights, decals, skinned characters, GPU
particles, UI overlay, TAA/temporal history, a ground grid, gizmos, picking. There is no reduced
surface underneath it for "just rasterize this one mesh with flat shading into a small offscreen
image," and standing up a full `ISceneView` per thumbnail — with its own environment and TAA
history — would be both wasteful and awkward to pool for potentially dozens of resident thumbnails.

So this phase adds one small, purpose-built entry point at the render tier: given a mesh, a
material set, and a target resolution, rasterize it with fixed simple lighting into an offscreen
color image and read the result back to the CPU. This is new engine-tier surface, not new editor
code layered on what already exists — the honest cost of doing this properly rather than
shoehorning a thumbnail into machinery built for a very different job.

## The new render-tier primitive

`engine/presentation/render`'s public interface gains one new entry point, mirroring the shape of
the existing `IWindowRenderer::create_scene_view() → std::unique_ptr<ISceneView>`: a new
`IWindowRenderer::create_mesh_thumbnail_renderer() → std::unique_ptr<IMeshThumbnailRenderer>`,
where `IMeshThumbnailRenderer` is a small new interface with one real method (its contract below).
Mirroring `create_scene_view()`'s shape rather than inventing a different one keeps this addition
consistent with how every other offscreen-render capability is already exposed from
`IWindowRenderer`. Its contract:

- **Input:** a mesh (positions, normals, UVs, indices — already what `Render::Assets::import_gltf`
  produces), a material (base-color texture at minimum; PBR maps beyond that are not required for
  this phase's fixed-simple-lighting model), a target width/height.
- **Camera:** computed automatically from the mesh's bounding box. A fixed three-quarter isometric
  angle (the same convention Unity's and Blender's default asset icons use) is positioned and
  distanced so the whole bounding box fits the frame with a small margin — no per-model tuning, no
  per-model camera authoring.
- **Lighting:** one fixed directional "headlight" from roughly the camera's direction, plus a flat
  ambient term. No shadows, no IBL, no atmosphere, no post-processing (no TAA, no bloom, no tone
  curve beyond a basic gamma-correct output) — this is deliberately as cheap as a single opaque
  draw call plus one clear, matching the "simple/unlit-leaning shading" fidelity level chosen for
  this phase over full Scene-view-quality PBR.
- **Output:** an RGBA8 buffer the caller can read back to the CPU (the render tier already has a
  read-back primitive in `ISceneView::read_output`/`FrameImage`; the new primitive follows the same
  shape) — the editor-tier cache below owns turning that into a resident GPU texture for ImGui,
  exactly as Phase 2's `ThumbnailCache` already does for decoded image bytes.

This primitive draws exactly one mesh per call and holds no state between calls beyond what one
render needs — no persistent scene, no temporal history, no accumulated frame state — so nothing
about calling it repeatedly, or interleaving it with real Scene-view frames, requires special
handling elsewhere in the renderer.

## `ModelThumbnailCache`

A new editor-tier class, `applications/editor/source/project/model_thumbnail_cache.hpp/.cpp`,
parallel to Phase 2's `ThumbnailCache` in spirit but not sharing its code — the two pipelines
differ enough (no background thread is possible here; the work is inherently render-thread work)
that forcing a shared base class would obscure more than it would save. What it does share:

- **The same generic `SushiEngine::Imaging::LruCache<Key, Value>`** (from Phase 2's
  `engine/domain/imaging` module) for eviction bookkeeping — this is exactly the kind of second
  consumer that module was built to serve.
- **The same delayed-eviction shape**: an evicted resident thumbnail's Vulkan resources are held
  in a pending-eviction queue for a few frames before actual destruction, so a command buffer from
  a still-in-flight frame can never sample a freed resource — the identical hazard `ThumbnailCache`
  already fixed for images, present here too since both are ordinary Vulkan-resident textures once
  built.
- **The same per-frame, budgeted `update()` shape** and the same permanent-failure-on-error
  policy (a load or render failure logs once and never retries that path, exactly like an upload
  failure in Phase 2).

What's different:

- **No worker thread.** `texture_for(path)` enqueues a request; `update()` pops **at most one**
  queued path per frame and, synchronously within that frame, loads the glTF/GLB (mesh + material)
  into the isolated registry below, calls the new render-tier primitive, and uploads the RGBA8
  result to a resident GPU texture — the entire pipeline for one model, in one frame, because
  nothing in it can run off the render thread. This is a much smaller per-frame budget than
  Phase 2's `UPLOADS_PER_FRAME = 2` specifically because the unit of work is far more expensive
  (mesh import plus a real draw call, not a buffer copy).
- **An isolated `MeshRegistry`/`TextureLibrary` pair**, owned by `ModelThumbnailCache` itself and
  never shared with the live scene's asset registries. Loading a model for its thumbnail — or
  evicting one from the LRU — never touches, invalidates, or duplicates anything the Scene view has
  loaded for the same file, even if a user has that exact model open in the scene at the same time.
  This isolated pair is torn down along with everything else in `ModelThumbnailCache`'s destructor,
  following the same "stop and drain before the device goes away" ordering `ThumbnailCache`
  already established.
- **A much smaller resident cap: 32** (versus Phase 2's 256), reflecting the far higher per-entry
  cost (a real mesh, material, and texture set resident in GPU memory, versus a single 128×128
  RGBA8 texture). The same `EVICTION_DELAY_FRAMES` conservative constant from Phase 2 applies here
  unchanged.

## Integration

`EditorContext` gains a second cache pointer, `ModelThumbnailCache* model_thumbnail_cache =
nullptr;`, constructed and wired in `main.cpp` the same way `ThumbnailCache` already is (declared
after the renderer/ImGui backend so it is destroyed before either, `update()` called once per frame
alongside `ThumbnailCache::update()`). `project_panel.cpp`'s Grid view gains a second, parallel
branch beside the existing `EntryCategory::Image` one: for a visible (`ImGui::IsRectVisible`)
`EntryCategory::Model` entry whose extension is `.gltf` or `.glb`, ask
`model_thumbnail_cache->texture_for(path)`; a `.fbx`/`.obj` entry never reaches this branch at all
(the extension check happens before the cache is asked anything, so those two formats cost nothing
extra) and always falls through to Phase 1's wireframe-cube glyph, matching every other
not-yet-resolved case.

## Testing

The render-tier primitive and the live Vulkan resource lifecycle in `ModelThumbnailCache` are, like
Phase 2's `ThumbnailCache`, not reachable from this codebase's test binary (no GPU device in that
lane) and are verified by reading and manual `se editor` verification rather than an automated
suite. The one piece of new algorithmic content with no device dependency — the bounding-box-to-
camera-framing math (computing the three-quarter-view camera position/distance from an arbitrary
axis-aligned bounding box so it fits the frame with margin) — should be extracted as a small,
free-standing, GPU-independent function and unit-tested the same way Phase 2's box-filter downscale
was, following the same "engine/domain, no Vulkan/ImGui dependency" pattern
(`engine/domain/imaging` or a new similarly-scoped module, decided at implementation time based on
where the function most naturally belongs).

## Explicitly out of scope (this phase)

- `.fbx` and `.obj` thumbnails (no importer exists; a separate, larger effort).
- Full PBR/lit shading matching the live Scene view's quality (fixed simple lighting only, per the
  approved fidelity choice).
- Thumbnails in List view (unchanged from every prior phase's scope decision).
- Any camera control, per-model framing override, or user-authored thumbnail angle.
- Reusing the scene's live asset registries for thumbnail loading (isolated registries only, per
  the approved isolation choice).
