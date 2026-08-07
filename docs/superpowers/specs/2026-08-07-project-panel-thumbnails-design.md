# Project panel view overhaul (Phase 2: real image thumbnails)

**Status:** designed, 2026-08-07.

## Problem

Phase 1 (`docs/superpowers/specs/2026-08-07-project-panel-view-overhaul-design.md`) gave every
asset category a procedurally drawn vector glyph, including images — a picture-frame rectangle
with a sun and a mountain fold, identical for every image file regardless of its content. Unity's
Project window renders the actual image content as the tile's icon. This phase closes that gap
for Grid view.

## Scope

Real thumbnails apply to **Grid view only**, for entries classified `EntryCategory::Image` by
Phase 1's classifier. List view keeps a small fixed-size icon regardless of category (Phase 1's
list rows are a fixed ~20px height; a real thumbnail at that size carries negligible information
over the flat glyph, and List already does not participate in the zoom control). Every image tile
in Grid shows Phase 1's picture-frame glyph as a placeholder until its real thumbnail finishes
loading, and continues showing it permanently if the load fails (corrupt file, unsupported
format, decode error) — there is no separate "broken image" glyph.

## Engine-tier extension

`Render::NativeDeviceHandles` (`engine/presentation/render/include/SushiEngine/render/rhi/device.hpp:110-117`)
gains one field:

```cpp
void* allocator = nullptr; // VmaAllocator, opaque per this struct's existing convention
```

This struct's doc comment already frames it as "the one deliberate escape hatch from the
abstraction... for the editor's Dear ImGui Vulkan backend" — the same justification that puts
`instance`/`physical_device`/`device`/`graphics_queue` here applies to the allocator, since the
Vulkan-backed asset system already exposes it internally
(`engine/presentation/render/source/rhi/vulkan/vulkan_device.hpp:139`,
`VmaAllocator allocator() const noexcept`) but the editor's escape hatch never carried it out.
`VulkanDevice::native_handles()` (`vulkan_device.cpp:454-463`) gets one added line populating it.
No other engine-tier code changes; every other Vulkan object this feature needs (image, view,
sampler, staging buffer, command buffer) is created and destroyed entirely inside the new editor
component below, using this allocator and the existing device/queue handles.

## New component: `ThumbnailCache`

`applications/editor/source/project/thumbnail_cache.hpp/.cpp` — a self-contained, editor-only
class. It is not a generalized asset-loading abstraction; it does one thing: given a file path to
an image, eventually produce an `ImTextureID` for it, or nothing if the file cannot be decoded.

**Public surface:**

```cpp
class ThumbnailCache
{
public:
    ThumbnailCache(Render::NativeDeviceHandles handles, ImGuiBackend& backend,
                   Editor::Console& console);
    ~ThumbnailCache();

    // Call once per frame from the Project panel, before drawing tiles.
    // Uploads at most a small fixed number of completed decodes to the GPU
    // and registers them with ImGuiBackend.
    void update();

    // Returns a valid texture id if the thumbnail for `path` is resident,
    // otherwise enqueues a decode request (if not already pending/resident)
    // and returns std::nullopt.
    std::optional<ImTextureID> texture_for(const std::filesystem::path& path);
};
```

`texture_for` is the only entry point `project_panel.cpp` calls per visible image tile; `update`
is called once per frame from the panel's top level. This mirrors the existing
`CookBakeState`/`IAssetLibrary` pattern of a long-lived system object owned by `EditorContext` and
polled once per frame — nothing new is introduced at the integration layer.

**Internals:**

- A `std::thread` worker, started in the constructor, joined in the destructor. It reads decode
  requests off a mutex-guarded queue, blocking when empty (condition variable) — the same
  producer/consumer shape a bounded work queue always takes; no third-party queue library is
  introduced since one bounded `std::deque` under a `std::mutex` is the entire requirement.
- Decode step, on the worker thread: load the file into an RGBA8 buffer, box-filter downscale to
  a fixed **128×128**, push the result onto a second mutex-guarded "ready" queue. Load failures
  (bad path, unsupported format, decode error) push nothing — the request is simply dropped, and
  `texture_for` keeps returning `std::nullopt` for that path forever (matching "no separate
  broken-image glyph": Grid keeps showing the Phase 1 placeholder indefinitely).
- Upload step, on the main thread inside `update()`: pop up to **2 ready decodes per frame** (a
  fixed budget, so a folder full of new images can't spike a single frame's GPU work), create a
  128×128 `VK_FORMAT_R8G8B8A8_UNORM` image + view + sampler via the allocator and device handles
  from `NativeDeviceHandles`, upload through a staging buffer and a one-shot command buffer
  (submitted and waited on synchronously — this is a low-frequency, low-priority path; no
  fencing/pipelining infrastructure is justified for at most 2 uploads/frame), then call
  `ImGuiBackend::register_texture` to obtain the `ImTextureID`.
- LRU eviction: resident thumbnails are capped at **256**. Every `texture_for` hit marks that
  entry most-recently-used; when a new upload would exceed the cap, the least-recently-used
  entry's Vulkan image/view/sampler are destroyed and its `ImGuiBackend::unregister_texture`
  called before evicting it from the map. At 128×128 RGBA8 (64KB per thumbnail), 256 resident
  thumbnails is a ~16MB steady-state ceiling.
- No cross-session persistence — thumbnails are decoded fresh every editor run. No file-system
  watching or invalidation on external edits — this matches Phase 1's scope discipline and the
  fact that no other part of the Project panel currently reacts to files changing on disk outside
  the editor.

**Viewport-culled requests:** `draw_project_grid_view` only calls `texture_for` for tiles that are
actually visible in the scrolled child window this frame (it already computes each tile's
position for layout, so testing that rect against the child window's visible `Y` range before
calling `texture_for` is a few added lines, not a new subsystem). Scrolling past an image without
it ever appearing on screen never enqueues a decode for it.

## Integration

- `EditorContext` (`applications/editor/source/core/editor_context.hpp`) gains
  `ThumbnailCache* thumbnail_cache = nullptr;`, following the existing non-owning pointer pattern
  used for `assets` and `cook_bake_state`.
- `main.cpp` constructs the `ThumbnailCache` once, after the renderer, `ImGuiBackend`, and
  `Editor::Console` all exist (it needs `native_handles()` and references to the backend and the
  console), and destroys it before the renderer is torn down — the worker thread must be joined and every resident Vulkan resource
  freed while the `VmaAllocator`/`VkDevice` it used are still alive. This ordering constraint is
  the same one every other renderer-dependent editor system already respects.
- `draw_project_grid_view` (`applications/editor/source/project/project_panel.cpp`): for a tile
  whose `entry_category() == EntryCategory::Image`, call
  `context.thumbnail_cache->texture_for(path)`; if it returns a value, draw it with
  `ImGui::Image` sized to the tile's icon area (same rect Phase 1's `draw_entry_icon` glyph
  occupies); otherwise fall through to Phase 1's existing picture-frame glyph path unchanged.
  `draw_project_panel`'s per-frame setup calls `context.thumbnail_cache->update()` once, before
  the grid/list view functions run.

## Error handling

- Decode failure (corrupt file, unrecognized format, zero-byte file): silently drop the request
  on the worker thread; the tile keeps showing the Phase 1 glyph. No error is surfaced to the
  user — a failed thumbnail is not a failure of the panel, and Phase 1's glyph is a legitimate,
  intentional fallback rather than an error state.
- Vulkan upload failure (out of device memory, unexpected driver error): the same fallback path —
  log once via the editor's existing Console facility
  (`applications/editor/source/core/console.hpp`'s `Editor::Console`, the same backlog the
  Console panel reads and that ~150 call sites across the editor already write to through
  `editor_log`), at `LogLevel::Warning`, then keep the tile on the Phase 1 glyph and do not retry
  that path automatically (retrying would require the same viewport visibility to trigger
  `texture_for` again, which naturally happens if the user scrolls the tile out and back — no
  separate retry mechanism is needed). `ThumbnailCache` takes an `Editor::Console&` directly
  (not an `EditorContext&`, which would create a construction-order cycle with
  `EditorContext::thumbnail_cache` below) so it can log without depending on the wider context
  type. RHI-tier code is never the one reporting this failure — `engine/presentation/render`'s
  Vulkan layer has no logging facility of its own and today only throws on device error;
  `ThumbnailCache`'s upload step catches around its own Vulkan calls and reports through
  `Editor::Console` itself, entirely inside the editor tier.

## Testing

The box-filter downscale and the LRU bookkeeping (which path is evicted, hit/miss accounting) are
pure logic with no Vulkan or ImGui dependency, and are the first automated tests in this feature
area:

- `tests/unit/test_thumbnail_downscale.cpp`: feeds known small RGBA8 buffers through the
  box-filter downscale function and asserts pixel-exact (or tolerance-bounded, if the filter
  isn't integer-exact) output for a few hand-computed cases (uniform color, checkerboard,
  non-square source).
- `tests/unit/test_thumbnail_lru.cpp`: drives the LRU bookkeeping structure directly (insert,
  touch, evict-at-capacity) with fake keys, independent of any real image or GPU resource,
  asserting eviction order matches least-recently-used.

Both require the downscale function and the LRU structure to be extracted as GPU-independent,
freestanding units inside `thumbnail_cache.cpp` (or a small internal header) rather than inlined
into GPU-touching code — this is a deliberate design requirement, not an implementation detail,
so that the one piece of this feature with real algorithmic content is verifiable without a GPU.

The Vulkan upload path itself, the worker thread's lifecycle, and the end-to-end
decode-to-visible-tile behavior remain manual verification (no existing test infrastructure in
this codebase exercises GPU resource creation or `draw_*_panel` functions — consistent with
Phase 1's testing section). Manual verification: open a Project folder containing several image
files of different formats/sizes, confirm each shows its real content in Grid view within a
couple of frames of becoming visible; scroll a large folder and confirm off-screen images don't
front-load; confirm List view still shows the generic image glyph; confirm a deliberately
corrupted image file keeps showing the Phase 1 glyph indefinitely without an error dialog or
crash; confirm editor shutdown with thumbnails loaded does not crash or leak-report under any
validation layers already enabled in debug builds.

## Explicitly out of scope (this phase)

- Thumbnails in List view (fixed generic icon regardless of category, per Scope above).
- Any thumbnail resolution other than 128×128, or user-configurable resolution.
- Cross-session thumbnail caching or disk-backed cache.
- Reacting to files changing on disk after being decoded (no filesystem watch).
- A distinct "broken image" glyph — decode/upload failure always falls back to Phase 1's normal
  image placeholder glyph.
