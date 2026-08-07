# Imaging {#module-imaging}

`imaging` owns pixel-buffer resampling and generic bounded caching, with no device and no file
format knowledge of its own. It exists so a consumer that does own a device and a decoder — today,
the editor's Project panel thumbnail pipeline — can keep its one piece of real algorithmic content
testable without a GPU, the same reason `geometry` was split out from behind Vulkan.

## Tier

`domain` — the second tier in `cmake/EngineLayers.cmake`, so a module here may depend on
`foundation` and on other `domain` modules, and on nothing above.

## Dependencies

- None. The module links nothing — no Vulkan, no stb_image, no ImGui — deliberately: a decoder or
  a GPU upload path built against a live device belongs to its consumer, not here, and the filter
  and cache below must stay reachable from a plain unit test binary.

## Public surface

Headers are relative to `include/SushiEngine/imaging/`.

| Header | Declares |
|---|---|
| `box_downscale.hpp` | `box_downscale_rgba8` — an unweighted box-filter resample of a tightly packed RGBA8 buffer to an exact target size. Every output texel averages the source texels whose box falls under it; a target box narrower than one source texel clamps to a single sample, degenerating gracefully to nearest-neighbour rather than needing a separate upscale case. |
| `lru_cache.hpp` | `LruCache<Key, Value>` — a fixed-capacity, generic least-recently-used cache: O(1) `touch`/`insert` backed by a doubly linked list plus a hash index, `insert` returning the evicted least-recently-used entry once past capacity, and `drain` for orderly shutdown. Not thread-safe; a caller with a resource that must be freed on eviction owns that lifecycle itself — `insert`'s returned entry, not a destructor, is what tells it to. |

Only `box_downscale.cpp` compiles; `lru_cache.hpp` is header-only, since a template class has no
non-template translation unit to give it.

## Consumers

`applications/editor/source/project/thumbnail_cache.hpp`'s `ThumbnailCache` is the one real
consumer today: its background worker thread calls `box_downscale_rgba8` to reduce a decoded image
to a fixed 128x128 before it ever touches Vulkan, and its main-thread upload step keys an
`LruCache<std::string, ResidentThumbnail>` by file path to bound how many GPU-resident thumbnails
stay alive at once. Neither the downscale filter nor the cache know that Vulkan, ImGui, or a
background thread exist on the other side of that boundary — the same separation `geometry` keeps
from the physics cooker and the renderer.

## Tests

Covered by the functional suite in `tests/`, which links `sushiengine_imaging` directly (via the
`SushiEngine` facade) without pulling in `sushiengine_editor` or `sushiengine_render`.
`tests/unit/test_thumbnail_downscale.cpp` checks the box filter against a few hand-computed cases
(a uniform field, a checkerboard whose average is known exactly, and a non-square source that
would catch a transposed width/height). `tests/unit/test_thumbnail_lru.cpp` drives the LRU
bookkeeping with plain `int` keys, independent of any real image or GPU resource, asserting
eviction order matches least-recently-used.

## Further reading

- [`2026-08-07-project-panel-thumbnails-design.md`](../../../docs/superpowers/specs/2026-08-07-project-panel-thumbnails-design.md) —
  the design this module was extracted for, and why its two pieces had to be GPU-independent.
