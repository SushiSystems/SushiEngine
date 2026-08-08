# Project panel prefab thumbnails: the editor-tier cache (Phase 4b)

**Status:** designed, 2026-08-08.

## Problem

Phase 4a (`docs/superpowers/specs/2026-08-07-project-panel-prefab-thumbnails-design.md`, implemented
on `docs/design-corpus-audit`) built the render-tier primitive that can draw a prefab's resolved
entities into a thumbnail image, but nothing calls it yet. `project_panel.cpp`'s `EntryCategory::Prefab`
branch still always shows Phase 1's static hexagon glyph. This phase is the editor-tier consumer that
actually produces prefab thumbnails.

## Why this phase's scope is larger than the original Phase 4 design anticipated

The original design (Phase 4a's spec) planned for the render-tier renderer itself to instantiate a
prefab into a throwaway world, call `Scene::apply_prefab`/`Scene::resolve_scene_assets`, and walk the
result. While implementing that, a real tier violation was discovered and fixed:
`engine/presentation/render` (the `presentation` tier) is forbidden from depending on
`engine/world/simulation`/`engine/world/serialization` (the `world` tier) — `world` sits above
`presentation` in this repository's tier order (`cmake/EngineLayers.cmake`), and `render`-depends-on-
`simulation` is separately and permanently forbidden (`SUSHIENGINE_FORBIDDEN_EDGES`). The user's own
build caught this (the includes didn't even resolve).

The fix moved all of that orchestration out of the render tier. `IPrefabThumbnailRenderer`'s final,
merged shape (`engine/presentation/render/include/SushiEngine/render/prefab_thumbnail_renderer.hpp`)
is now:

```cpp
struct PrefabThumbnailDraw
{
    MeshId mesh = INVALID_MESH;
    Material material;
    Matrix4 model; // this entity's own composed world transform
};

class IPrefabThumbnailRenderer
{
    public:
        virtual ~IPrefabThumbnailRenderer() = default;
        virtual IAssetLibrary& asset_library() noexcept = 0;
        virtual bool render_thumbnail(const PrefabThumbnailDraw* draws, std::size_t count,
                                      std::uint32_t width, std::uint32_t height,
                                      FrameImage& out_image) = 0;
};
```

It takes an already-resolved array of draws and frames its own camera internally from their bounds.
Everything upstream of that array — instantiating the prefab, resolving its assets, walking the
result — is this phase's job, because `application` (the editor) is free to depend on both `world`
and `presentation`.

This phase also adds one small, additive amendment to that already-merged interface: a
`max_draws() const noexcept` accessor, so this cache can learn the renderer's fixed draw-count
ceiling (currently `128`, previously a private implementation constant) instead of hardcoding a
number that could silently drift out of sync with the renderer's own limit.

## Architecture

A new editor-tier class, `PrefabThumbnailCache`, structurally a near-twin of Phase 3b's
`ModelThumbnailCache` (`applications/editor/source/project/model_thumbnail_cache.hpp`), but its
per-request render step does real orchestration instead of a single file-path call. Per request,
`render_one(path)` does:

1. **Instantiate.** `SushiEngine::Simulation::create_simulation()` builds a fresh, disposable
   `ISimulation`, discarded at the end of this call — a new scratch world per request, never reused
   across prefabs, the same pattern `write_model_prefab` already uses for a scratch world with no
   live editor session attached.
2. **Load and apply.** Read the `.sushiprefab` file's bytes, parse as JSON, then
   `Scene::apply_prefab(scratch_world, document, Simulation::NULL_ENTITY)`.
3. **Resolve assets.** `Scene::resolve_scene_assets(scratch_world, prefab_renderer_->asset_library())`
   — resolved directly against the `IPrefabThumbnailRenderer`'s own isolated asset library, so this
   never touches what the live scene has loaded for the same files.
4. **Walk and build draws.** For every id in `scratch_world.entities()`: skip it if
   `!scratch_world.has_shape(id) || scratch_world.shape_parameters(id).mesh == Render::INVALID_MESH`
   — an entity's `Shape` resolves to at most one `MeshId` (`ShapeParameters::mesh`), so lights, empty
   transform nodes, and procedural (non-mesh) primitives are silently skipped; they simply do not
   appear in the thumbnail, the same flat/unlit fidelity ceiling every phase of this feature already
   accepts. For every entity that does carry a resolved mesh, append
   `PrefabThumbnailDraw{ shape_parameters(id).mesh, scratch_world.material(id),
   compose_transform(world_transform(id).position, world_transform(id).rotation,
   world_transform(id).scale) }`. If the running count would exceed `prefab_renderer_->max_draws()`,
   stop and treat the whole render as a load failure (see Error handling). If the walk finishes with
   zero draws, that is also a load failure, not a blank-but-successful render.
5. **Render.** One call: `prefab_renderer_->render_thumbnail(draws.data(), draws.size(),
   THUMBNAIL_SIZE, THUMBNAIL_SIZE, out_image)`. The renderer frames its own camera internally from
   the given draws — no bounds computation happens on this side at all.
6. **Upload**, identically to `ModelThumbnailCache::upload_one`: create a small resident GPU texture,
   copy `out_image`'s pixels in, register it with the ImGui Vulkan backend.

## `PrefabThumbnailCache`

```cpp
class PrefabThumbnailCache
{
    public:
        PrefabThumbnailCache(SushiEngine::Render::IWindowRenderer& renderer,
                              ImGuiBackend& backend, Console& console);
        ~PrefabThumbnailCache();
        PrefabThumbnailCache(const PrefabThumbnailCache&) = delete;
        PrefabThumbnailCache& operator=(const PrefabThumbnailCache&) = delete;

        void update();
        std::optional<ImTextureID> texture_for(const std::filesystem::path& path);

    private:
        struct ResidentThumbnail { /* identical shape to ModelThumbnailCache's */ };
        struct PendingEviction { /* identical shape to ModelThumbnailCache's */ };

        bool build_draws(const std::string& path,
                         std::vector<SushiEngine::Render::PrefabThumbnailDraw>& out);
        void render_one(const std::string& path);
        void upload_one(const std::string& path);
        void destroy_thumbnail(ResidentThumbnail& thumbnail);
        void recreate_prefab_renderer();

        static constexpr std::uint32_t THUMBNAIL_SIZE = 128;
        static constexpr std::size_t RESIDENT_CAPACITY = 128;
        static constexpr int EVICTION_DELAY_FRAMES = 4;
        // After this many successful renders through one IPrefabThumbnailRenderer, it is destroyed
        // and replaced to reclaim its unbounded-growth isolated asset stack. Smaller than Phase 3b's
        // 64: a single prefab render can pull in more distinct meshes/textures per call than a
        // single-model render, so the isolated asset stack fills faster per successful render.
        static constexpr std::size_t PREFABS_PER_RENDERER_LIFETIME = 24;

        SushiEngine::Render::IWindowRenderer& renderer_;
        std::unique_ptr<SushiEngine::Render::IPrefabThumbnailRenderer> prefab_renderer_;
        std::size_t prefabs_rendered_since_recreation_ = 0;

        VkDevice device_ = VK_NULL_HANDLE;
        VmaAllocator allocator_ = VK_NULL_HANDLE;
        ImGuiBackend& backend_;
        Console& console_;

        std::deque<std::string> pending_;
        std::unordered_set<std::string> in_flight_;
        std::unordered_set<std::string> failed_;
        std::uint64_t frame_counter_ = 0;
        std::deque<PendingEviction> pending_evictions_;

        SushiEngine::Imaging::LruCache<std::string, ResidentThumbnail> resident_{RESIDENT_CAPACITY};
};
```

`update()` keeps Phase 3b's synchronous one-render-per-frame budget: `render_thumbnail` blocks the
calling thread until the GPU finishes, and a prefab render can now additionally include file I/O and
a full asset-resolution pass, so doing more than one per frame risks a visible frame hitch. `update()`
does the whole build-draws-render-upload sequence for at most one path per frame, exactly mirroring
`ModelThumbnailCache::update`'s shape.

## Error handling

The established policy from every prior phase carries over unchanged: a render failure is silent and
permanent for that path (the hexagon glyph keeps showing forever, no log, never retried); an upload
failure is logged once via `Console::append(LogLevel::Warning, ...)` and is also permanent. This phase
adds one new failure cause to the "render failure" bucket, alongside corrupt/unparseable JSON and a
`mesh_path` that no longer resolves: a prefab whose walk would exceed `max_draws()`, or whose walk
finds zero drawable entities, is a load failure — exactly Phase 3a's own "more primitives than this
renderer's capacity" case, generalized to "zero or too many."

## Integration

`project_panel.cpp`'s `EntryCategory::Prefab` branch (currently unconditional — every `.sushiprefab`
entry shows the static hexagon glyph with no further branching) gains the same shape Phase 3b added
for `EntryCategory::Model`: for a visible prefab entry, ask
`context.prefab_thumbnail_cache->texture_for(path)`; on `std::nullopt`, fall back to the existing
hexagon glyph. `EditorContext` gains a `PrefabThumbnailCache* prefab_thumbnail_cache = nullptr;`
pointer, constructed in `main.cpp` after `thumbnail_cache`/`model_thumbnail_cache`/`imgui`/`renderer`
(matching the existing construction order), with `context.prefab_thumbnail_cache->update()` called
once per frame alongside the other two caches' `update()` calls.

## Testing

As with every prior phase, none of this is reachable from the test binary — Vulkan, a live-ish
`ISimulation`/`IWorldEditor`, and ImGui are all in the loop. This phase introduces no new pure-function
surface of its own; the one piece of pure, testable math this whole feature needed
(`expand_aabb_sphere`, Phase 4a's Task 1) already has its own unit tests and, as of this phase, finally
gets a real production caller (inside `VulkanPrefabThumbnailRenderer::render_thumbnail`, which computes
the camera-framing bounds internally from the draws this cache hands it).

## Explicitly out of scope (this phase)

Carried over unchanged from the original Phase 4 design:

- Any prefab whose subtree contains non-mesh visual elements this renderer doesn't already know how
  to draw (particle effects, lights, decals) — those entities' *meshes* (if any) still draw; their
  other visual contributions are not represented in the thumbnail.
- Nested prefab instances that themselves reference other prefab files recursively — resolved only as
  deep as `Scene::apply_prefab`/`resolve_scene_assets` already resolve them today.
- Any camera angle, lighting, or resolution control beyond what Phase 3a already fixed.
- Live invalidation when a `.sushiprefab` file's on-disk revision changes after its thumbnail has
  already been rendered — matching every prior phase's "render once, cache forever" policy; a changed
  prefab shows its old thumbnail until the editor restarts, the same as a changed model or image file
  today.
