# Project panel thumbnails (Phase 2) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Show a real, decoded thumbnail image for each `EntryCategory::Image` tile in the
Project panel's Grid view, replacing Phase 1's generic picture-frame glyph once the thumbnail
finishes loading.

**Architecture:** A new `ThumbnailCache` editor-tier class owns a background decode thread, a
bounded LRU-cached set of resident 128×128 GPU textures, and the raw Vulkan image/view/sampler
lifecycle for each — created through a new `allocator` field on the engine's existing
`NativeDeviceHandles` escape hatch. The pure, GPU-independent pieces (the box-filter downscale
and the generic LRU bookkeeping) live in a new neutral engine module,
`engine/domain/imaging`, following the same pattern the `geometry` module already uses to keep
device-adjacent logic unit-testable without a renderer. `project_panel.cpp`'s Grid view calls
`ThumbnailCache::texture_for()` per visible image tile and falls back to Phase 1's existing glyph
whenever no texture is ready yet.

**Tech Stack:** C++17, Vulkan 1.3 (core sync2), VMA (`vk_mem_alloc.h`), Dear ImGui (docking
branch), stb_image (already vendored via vcpkg's `Stb` package and compiled once in
`engine/presentation/render/source/material/stb_impl.cpp`), GoogleTest.

## Global Constraints

- **Scope:** real thumbnails apply to Grid view only, for `EntryCategory::Image` entries. List
  view always shows its existing fixed 16px generic image glyph — never a real thumbnail — per
  the approved spec.
- **Thumbnail resolution:** fixed **128×128**, `VK_FORMAT_R8G8B8A8_UNORM`, produced by a box-filter
  downscale run on the decode worker thread. Not user-configurable.
- **Resident cache cap:** LRU, **256** resident thumbnails maximum.
- **Per-frame upload budget:** at most **2** completed decodes uploaded to the GPU per frame,
  inside `ThumbnailCache::update()`.
- **Viewport culling:** `texture_for()` is only called for tiles the Grid view's scrolled child
  window is currently showing; a tile scrolled fully out of view never triggers a decode request.
- **Failure handling:** a decode failure or an upload failure is not a distinct "broken image"
  state — the tile keeps showing Phase 1's picture-frame glyph forever for that path. An upload
  failure is logged once via `Console::append(LogLevel::Warning, ...)`
  (`applications/editor/source/core/console.hpp`); a decode failure is silently dropped (no log —
  a corrupt or unsupported file the user opens by browsing to it is not itself remarkable, and
  every decode failure would otherwise be a warning line for every non-image or malformed file in
  a project).
- **No cross-session persistence, no filesystem watching.** Thumbnails decode fresh every editor
  run; a file changed on disk after being decoded is not picked back up automatically.
- **No new third-party dependency.** stb_image is already vendored and linked into
  `sushiengine_render`; VMA is already `EXTERNAL_PUBLIC` on that same target, so both are already
  visible to `sushiengine_editor` once it links `sushiengine_render` (which it already does)
  without any new `find_package` call for either. Only `Stb`'s include directory needs adding to
  the editor target explicitly, since `sushiengine_render` only exposes it `PRIVATE`.
- **This machine cannot run builds.** Neither the user nor any implementer/reviewer subagent runs
  `se build`, `se test`, `se editor`, or raw cmake/ninja during this plan's execution — the
  machine cannot handle it. Every task is verified by reading the code and reasoning about it by
  hand, never by compiling or running it. The user builds and tests the whole branch themselves
  once every task is complete. Wherever this plan's steps say "run the test," that step is
  performed as a careful by-hand trace instead, and the step's text says so explicitly.
- **No engine-tier code may depend on anything above its own tier**
  (`cmake/EngineLayers.cmake`'s `foundation < domain < asset < presentation < world <
  application` order). `engine/domain/imaging` may depend on `core` (foundation) at most; it must
  never include anything from `presentation` (Vulkan, ImGui) or `application` (the editor).

---

### Task 1: Extend `NativeDeviceHandles` with the VMA allocator

**Files:**
- Modify: `engine/presentation/render/include/SushiEngine/render/rhi/device.hpp:110-117`
- Modify: `engine/presentation/render/source/rhi/vulkan/vulkan_device.cpp:454-463`

**Interfaces:**
- Produces: `SushiEngine::Render::NativeDeviceHandles::allocator` (`void*`, opaque per this
  struct's existing convention — a Vulkan consumer `static_cast`s it to `VmaAllocator`). Every
  later task that creates a `NativeDeviceHandles` value (Task 5's `main.cpp` wiring) and every
  task that reads one (Task 4's `ThumbnailCache` constructor) uses this field.

There is no test for this task — it is a two-line, mechanical addition to a plain struct and an
existing accessor's return statement, with no logic branch to exercise. Correctness is verified
by reading the diff against the two exact locations below.

- [ ] **Step 1: Add the `allocator` field to `NativeDeviceHandles`**

In `engine/presentation/render/include/SushiEngine/render/rhi/device.hpp`, the struct currently
reads (lines 110-117):

```cpp
        struct NativeDeviceHandles
        {
            void* instance = nullptr;
            void* physical_device = nullptr;
            void* device = nullptr;
            void* graphics_queue = nullptr;
            std::uint32_t graphics_queue_family = 0;
        };
```

Change it to:

```cpp
        struct NativeDeviceHandles
        {
            void* instance = nullptr;
            void* physical_device = nullptr;
            void* device = nullptr;
            void* graphics_queue = nullptr;
            std::uint32_t graphics_queue_family = 0;
            /** @brief The VmaAllocator bound to this device, for a consumer that must
             *  create its own Vulkan images outside the renderer's own resource system
             *  (e.g. the editor's Project panel thumbnail cache). */
            void* allocator = nullptr;
        };
```

- [ ] **Step 2: Populate it in `VulkanDevice::native_handles()`**

In `engine/presentation/render/source/rhi/vulkan/vulkan_device.cpp`, the function currently reads
(lines 454-463):

```cpp
            NativeDeviceHandles VulkanDevice::native_handles() const noexcept
            {
                NativeDeviceHandles handles;
                handles.instance = instance_.instance;
                handles.physical_device = device_.physical_device;
                handles.device = device_.device;
                handles.graphics_queue = graphics_queue_;
                handles.graphics_queue_family = graphics_queue_family_;
                return handles;
            }
```

Add one line before the `return`:

```cpp
            NativeDeviceHandles VulkanDevice::native_handles() const noexcept
            {
                NativeDeviceHandles handles;
                handles.instance = instance_.instance;
                handles.physical_device = device_.physical_device;
                handles.device = device_.device;
                handles.graphics_queue = graphics_queue_;
                handles.graphics_queue_family = graphics_queue_family_;
                handles.allocator = allocator_;
                return handles;
            }
```

`allocator_` is the existing private `VmaAllocator` member this same class already exposes
through its (unrelated, private-to-the-render-tier) `allocator()` accessor
(`engine/presentation/render/source/rhi/vulkan/vulkan_device.hpp:139`) — this step exposes the
same value through the public escape hatch instead.

- [ ] **Step 3: Commit**

```bash
git add engine/presentation/render/include/SushiEngine/render/rhi/device.hpp \
        engine/presentation/render/source/rhi/vulkan/vulkan_device.cpp
git commit -m "feat(render): expose the VMA allocator through NativeDeviceHandles"
```

---

### Task 2: New `engine/domain/imaging` module with a box-filter downscale

**Files:**
- Modify: `cmake/EngineLayers.cmake`
- Modify: `engine/domain/CMakeLists.txt`
- Create: `engine/domain/imaging/CMakeLists.txt`
- Create: `engine/domain/imaging/include/SushiEngine/imaging/box_downscale.hpp`
- Create: `engine/domain/imaging/source/box_downscale.cpp`
- Modify: `CMakeLists.txt` (repo root)
- Modify: `tests/CMakeLists.txt`
- Create: `tests/unit/test_thumbnail_downscale.cpp`

**Interfaces:**
- Produces: `SushiEngine::Imaging::box_downscale_rgba8(const std::uint8_t* source,
  std::uint32_t source_width, std::uint32_t source_height, std::uint32_t target_width,
  std::uint32_t target_height) -> std::vector<std::uint8_t>` — a tightly packed
  `target_width * target_height * 4`-byte RGBA8 buffer. Task 4's `ThumbnailCache` worker thread
  calls this with `target_width = target_height = 128`.

This is the first piece of the new module; Task 3 adds the LRU cache to the same module and its
own CMake wiring is already in place after this task, so Task 3 only adds one more source file
and one more test file to lists this task creates.

- [ ] **Step 1: Register the module's tier**

In `cmake/EngineLayers.cmake`, `SUSHIENGINE_MODULE_LAYERS` currently lists (among others):

```cmake
    geometry        domain
    physics         domain
    material        domain
```

Add `imaging` to the same `domain` tier, next to `geometry` (alphabetical company, not a
requirement — just where it reads naturally):

```cmake
    geometry        domain
    imaging         domain
    physics         domain
    material        domain
```

- [ ] **Step 2: Add the module directory to the domain tier's aggregator**

`engine/domain/CMakeLists.txt` currently reads:

```cmake
add_subdirectory(geometry)
add_subdirectory(physics)
```

Add the new line between them (matching the manifest's ordering above, though CMake does not
care about the order these two files agree on):

```cmake
add_subdirectory(geometry)
add_subdirectory(imaging)
add_subdirectory(physics)
```

- [ ] **Step 3: Write the module's `CMakeLists.txt`**

Create `engine/domain/imaging/CMakeLists.txt`:

```cmake
# imaging — pixel-buffer resampling and generic bounded caching, with no device and no file
# format knowledge of its own. It exists so the editor's thumbnail pipeline (which does own a
# device and a decoder) can keep its one piece of real algorithmic content — the downscale
# filter and the cache eviction order — testable without a GPU, the same reason `geometry` was
# split out from behind Vulkan.
sushiengine_add_module(NAME imaging LAYER domain
    SOURCES
        source/box_downscale.cpp)
```

- [ ] **Step 4: Write the downscale header**

Create `engine/domain/imaging/include/SushiEngine/imaging/box_downscale.hpp`:

```cpp
/**************************************************************************/
/* box_downscale.hpp                                                     */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/* Licensed under the Apache License, Version 2.0 (the "License");        */
/* you may not use this file except in compliance with the License.       */
/* You may obtain a copy of the License at                                */
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

#pragma once

/**
 * @file box_downscale.hpp
 * @brief A box-filter RGBA8 resample, with no device and no file format knowledge.
 *
 * Every output texel is the unweighted average of the source texels whose box falls under
 * it, which is the simplest resample that does not alias when shrinking an image — a
 * point-sample would silently drop entire rows of pixels between texels once the ratio gets
 * large enough. There is no separate "upscale" case: a target box narrower than one source
 * texel just clamps to a single sample, which degenerates gracefully to nearest-neighbour.
 */

#include <cstdint>
#include <vector>

namespace SushiEngine
{
    namespace Imaging
    {
        /**
         * @brief Box-filter downscales an RGBA8 image to an exact target size.
         * @param source        Tightly packed RGBA8 pixels, @p source_width * @p source_height * 4 bytes.
         * @param source_width  Source width in texels. Must be at least 1.
         * @param source_height Source height in texels. Must be at least 1.
         * @param target_width  Output width in texels. Must be at least 1.
         * @param target_height Output height in texels. Must be at least 1.
         * @return A tightly packed RGBA8 buffer, @p target_width * @p target_height * 4 bytes.
         */
        std::vector<std::uint8_t> box_downscale_rgba8(const std::uint8_t* source,
                                                       std::uint32_t source_width,
                                                       std::uint32_t source_height,
                                                       std::uint32_t target_width,
                                                       std::uint32_t target_height);
    } // namespace Imaging
} // namespace SushiEngine
```

- [ ] **Step 5: Write the implementation**

Create `engine/domain/imaging/source/box_downscale.cpp`:

```cpp
/**************************************************************************/
/* box_downscale.cpp                                                     */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/* Licensed under the Apache License, Version 2.0 (the "License");        */
/* you may not use this file except in compliance with the License.       */
/* You may obtain a copy of the License at                                */
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

#include "SushiEngine/imaging/box_downscale.hpp"

#include <algorithm>
#include <cstddef>

namespace SushiEngine
{
    namespace Imaging
    {
        std::vector<std::uint8_t> box_downscale_rgba8(const std::uint8_t* source,
                                                       std::uint32_t source_width,
                                                       std::uint32_t source_height,
                                                       std::uint32_t target_width,
                                                       std::uint32_t target_height)
        {
            std::vector<std::uint8_t> result(
                std::size_t(target_width) * std::size_t(target_height) * 4);

            for (std::uint32_t ty = 0; ty < target_height; ++ty)
            {
                const std::uint32_t source_y0 = ty * source_height / target_height;
                const std::uint32_t source_y1 =
                    std::max(source_y0 + 1, (ty + 1) * source_height / target_height);

                for (std::uint32_t tx = 0; tx < target_width; ++tx)
                {
                    const std::uint32_t source_x0 = tx * source_width / target_width;
                    const std::uint32_t source_x1 =
                        std::max(source_x0 + 1, (tx + 1) * source_width / target_width);

                    std::uint32_t sum[4] = {0, 0, 0, 0};
                    std::uint32_t count = 0;
                    for (std::uint32_t sy = source_y0; sy < source_y1 && sy < source_height; ++sy)
                    {
                        for (std::uint32_t sx = source_x0; sx < source_x1 && sx < source_width; ++sx)
                        {
                            const std::uint8_t* texel =
                                source + (std::size_t(sy) * source_width + sx) * 4;
                            for (int channel = 0; channel < 4; ++channel)
                                sum[channel] += texel[channel];
                            ++count;
                        }
                    }

                    std::uint8_t* out =
                        result.data() + (std::size_t(ty) * target_width + tx) * 4;
                    for (int channel = 0; channel < 4; ++channel)
                        out[channel] =
                            count > 0 ? static_cast<std::uint8_t>(sum[channel] / count) : 0;
                }
            }

            return result;
        }
    } // namespace Imaging
} // namespace SushiEngine
```

- [ ] **Step 6: Link the module into the `SushiEngine` facade**

In the repository root `CMakeLists.txt`, `target_link_libraries(SushiEngine INTERFACE ...)`
currently lists (among others):

```cmake
    sushiengine_geometry
    sushiengine_input
```

Add `sushiengine_imaging` next to `sushiengine_geometry`:

```cmake
    sushiengine_geometry
    sushiengine_imaging
    sushiengine_input
```

This is what makes the module's headers and compiled code reachable from
`sushiengine_functional_tests` (Step 8) without that test binary linking `sushiengine_editor` or
`sushiengine_render` — exactly how `sushiengine_geometry` is already reachable there today.

- [ ] **Step 7: Write the failing test**

Create `tests/unit/test_thumbnail_downscale.cpp`:

```cpp
/**************************************************************************/
/* test_thumbnail_downscale.cpp                                          */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/* Licensed under the Apache License, Version 2.0 (the "License");        */
/* you may not use this file except in compliance with the License.       */
/* You may obtain a copy of the License at                                */
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

// Unit_ThumbnailDownscale: the Project panel thumbnail pipeline's box filter, checked against
// hand-computed cases rather than against itself — a uniform field, a checkerboard whose
// average is known exactly, and a non-square source, so a transposed width/height would fail
// the third case even though the first two would still pass by symmetry.

#include <gtest/gtest.h>

#include "SushiEngine/imaging/box_downscale.hpp"

using SushiEngine::Imaging::box_downscale_rgba8;

TEST(Unit_ThumbnailDownscale, UniformColorStaysUniform)
{
    // 4x4 solid (200, 100, 50, 255), downscaled to 2x2: every output texel averages four
    // identical source texels, so every output texel must equal the input color exactly.
    std::vector<std::uint8_t> source(4 * 4 * 4);
    for (std::size_t i = 0; i < source.size(); i += 4)
    {
        source[i + 0] = 200;
        source[i + 1] = 100;
        source[i + 2] = 50;
        source[i + 3] = 255;
    }

    const std::vector<std::uint8_t> result = box_downscale_rgba8(source.data(), 4, 4, 2, 2);
    ASSERT_EQ(result.size(), std::size_t(2 * 2 * 4));
    for (std::size_t i = 0; i < result.size(); i += 4)
    {
        EXPECT_EQ(result[i + 0], 200);
        EXPECT_EQ(result[i + 1], 100);
        EXPECT_EQ(result[i + 2], 50);
        EXPECT_EQ(result[i + 3], 255);
    }
}

TEST(Unit_ThumbnailDownscale, CheckerboardAveragesToMidGray)
{
    // 2x2 source: black, white, white, black (row-major). Downscaled to 1x1, the single
    // output texel is the box average of all four: (0 + 255 + 255 + 0) / 4 = 127 (integer
    // division truncates 127.5 down).
    const std::uint8_t source[2 * 2 * 4] = {
        0,   0,   0,   255, // top-left: black
        255, 255, 255, 255, // top-right: white
        0,   0,   0,   255, // bottom-left: black
        255, 255, 255, 255  // bottom-right: white
    };

    const std::vector<std::uint8_t> result = box_downscale_rgba8(source, 2, 2, 1, 1);
    ASSERT_EQ(result.size(), std::size_t(1 * 1 * 4));
    EXPECT_EQ(result[0], 127);
    EXPECT_EQ(result[1], 127);
    EXPECT_EQ(result[2], 127);
    EXPECT_EQ(result[3], 255);
}

TEST(Unit_ThumbnailDownscale, NonSquareSourceMapsWidthAndHeightIndependently)
{
    // 4-wide, 2-tall source, downscaled to 2x1: a width/height swap bug would either crash
    // (out-of-bounds read past a 4x2 buffer treated as 2x4) or silently average the wrong
    // texels together. Left half is red, right half is blue.
    std::uint8_t source[4 * 2 * 4];
    for (std::uint32_t y = 0; y < 2; ++y)
    {
        for (std::uint32_t x = 0; x < 4; ++x)
        {
            std::uint8_t* texel = source + (y * 4 + x) * 4;
            const bool left_half = x < 2;
            texel[0] = left_half ? 255 : 0;
            texel[1] = 0;
            texel[2] = left_half ? 0 : 255;
            texel[3] = 255;
        }
    }

    const std::vector<std::uint8_t> result = box_downscale_rgba8(source, 4, 2, 2, 1);
    ASSERT_EQ(result.size(), std::size_t(2 * 1 * 4));
    // Left output texel: pure red.
    EXPECT_EQ(result[0], 255);
    EXPECT_EQ(result[2], 0);
    // Right output texel: pure blue.
    EXPECT_EQ(result[4], 0);
    EXPECT_EQ(result[6], 255);
}
```

- [ ] **Step 8: Wire the test file into the test binary**

In `tests/CMakeLists.txt`, find the line `unit/test_geometry_sdf.cpp` inside the
`add_executable(sushiengine_functional_tests ...)` source list and add the new test file
immediately after it:

```cmake
    unit/test_geometry_sdf.cpp
    unit/test_thumbnail_downscale.cpp
```

- [ ] **Step 9: Verify by reading, not by building**

This machine cannot run `se test` (see Global Constraints). Instead: re-read
`box_downscale_rgba8` against each of the three test cases above by hand and confirm the
arithmetic matches — in particular, walk the `UniformColorStaysUniform` case's index math
(`source_y0`/`source_y1`/`source_x0`/`source_x1` for `ty=0,tx=0` through `ty=1,tx=1` on a 4x4→2x2
resample) and confirm every 2x2 source block maps to exactly one 2x2 target quadrant with no
overlap and no gap. The user runs `se test` after the branch is complete and will report back if
anything fails.

- [ ] **Step 10: Commit**

```bash
git add cmake/EngineLayers.cmake engine/domain/CMakeLists.txt \
        engine/domain/imaging/CMakeLists.txt \
        engine/domain/imaging/include/SushiEngine/imaging/box_downscale.hpp \
        engine/domain/imaging/source/box_downscale.cpp \
        CMakeLists.txt tests/CMakeLists.txt tests/unit/test_thumbnail_downscale.cpp
git commit -m "feat(imaging): add the imaging domain module with a box-filter downscale"
```

---

### Task 3: Generic LRU cache in the `imaging` module

**Files:**
- Create: `engine/domain/imaging/include/SushiEngine/imaging/lru_cache.hpp`
- Modify: `tests/CMakeLists.txt`
- Create: `tests/unit/test_thumbnail_lru.cpp`

**Interfaces:**
- Consumes: nothing from Task 2 (independent header, same module).
- Produces: `SushiEngine::Imaging::LruCache<Key, Value>` with:
  - `explicit LruCache(std::size_t capacity)`
  - `Value* touch(const Key& key)` — returns the value and marks it most-recently-used, or
    `nullptr` if `key` is absent.
  - `std::optional<std::pair<Key, Value>> insert(const Key& key, Value value)` — inserts or
    overwrites `key` as most-recently-used; if this pushes the cache over `capacity`, evicts and
    returns the least-recently-used entry (`std::nullopt` if nothing was evicted).
  - `std::size_t size() const noexcept`
  - `std::vector<std::pair<Key, Value>> drain()` — empties the cache and returns every entry, for
    orderly shutdown.

  Task 4's `ThumbnailCache` is the consumer: `LruCache<std::string, ResidentThumbnail>` keyed by
  the image's file path string, capacity `256`.

- [ ] **Step 1: Write the header**

Create `engine/domain/imaging/include/SushiEngine/imaging/lru_cache.hpp`:

```cpp
/**************************************************************************/
/* lru_cache.hpp                                                          */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/* Licensed under the Apache License, Version 2.0 (the "License");        */
/* you may not use this file except in compliance with the License.       */
/* You may obtain a copy of the License at                                */
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

#pragma once

/**
 * @file lru_cache.hpp
 * @brief A fixed-capacity least-recently-used cache, generic over key and value.
 *
 * Backed by a doubly linked list ordered most- to least-recently-used, plus a hash index from
 * key to that list's iterator — so both @ref touch and @ref insert are O(1), the property that
 * makes an LRU eviction policy worth choosing over just walking a vector. Not thread-safe: the
 * caller (the thumbnail cache's main-thread upload step) is the only place this is touched.
 */

#include <cstddef>
#include <list>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

namespace SushiEngine
{
    namespace Imaging
    {
        template <typename Key, typename Value>
        class LruCache
        {
            public:
                explicit LruCache(std::size_t capacity) : capacity_(capacity) {}

                /**
                 * @brief Looks up @p key, promoting it to most-recently-used on a hit.
                 * @return A pointer to the value, valid until the next @ref insert that evicts
                 *         it or the cache is destroyed; @c nullptr if @p key is absent.
                 */
                Value* touch(const Key& key)
                {
                    const auto found = index_.find(key);
                    if (found == index_.end())
                        return nullptr;
                    order_.splice(order_.begin(), order_, found->second);
                    return &found->second->second;
                }

                /**
                 * @brief Inserts or overwrites @p key as most-recently-used.
                 * @return The evicted least-recently-used entry if this insert pushed the cache
                 *         past capacity; @c std::nullopt otherwise (including when @p key
                 *         already existed, which never evicts).
                 */
                std::optional<std::pair<Key, Value>> insert(const Key& key, Value value)
                {
                    const auto found = index_.find(key);
                    if (found != index_.end())
                    {
                        found->second->second = std::move(value);
                        order_.splice(order_.begin(), order_, found->second);
                        return std::nullopt;
                    }

                    order_.emplace_front(key, std::move(value));
                    index_[key] = order_.begin();

                    if (order_.size() <= capacity_)
                        return std::nullopt;

                    std::pair<Key, Value> evicted = std::move(order_.back());
                    index_.erase(evicted.first);
                    order_.pop_back();
                    return evicted;
                }

                /** @brief How many entries are resident right now. */
                std::size_t size() const noexcept { return order_.size(); }

                /** @brief Empties the cache, returning every entry it held. */
                std::vector<std::pair<Key, Value>> drain()
                {
                    std::vector<std::pair<Key, Value>> all;
                    all.reserve(order_.size());
                    for (auto& entry : order_)
                        all.push_back(std::move(entry));
                    order_.clear();
                    index_.clear();
                    return all;
                }

            private:
                std::size_t capacity_;
                // Front = most recently used, back = least recently used.
                std::list<std::pair<Key, Value>> order_;
                std::unordered_map<Key, typename std::list<std::pair<Key, Value>>::iterator>
                    index_;
        };
    } // namespace Imaging
} // namespace SushiEngine
```

- [ ] **Step 2: Write the failing test**

Create `tests/unit/test_thumbnail_lru.cpp`:

```cpp
/**************************************************************************/
/* test_thumbnail_lru.cpp                                                 */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/* Licensed under the Apache License, Version 2.0 (the "License");        */
/* you may not use this file except in compliance with the License.       */
/* You may obtain a copy of the License at                                */
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

// Unit_ThumbnailLru: the thumbnail cache's eviction bookkeeping, driven with plain int keys and
// values so the test has no Vulkan/ImGui dependency at all — the property under test is purely
// "which entry goes away," never what a resident thumbnail happens to hold.

#include <gtest/gtest.h>

#include "SushiEngine/imaging/lru_cache.hpp"

using SushiEngine::Imaging::LruCache;

TEST(Unit_ThumbnailLru, InsertingPastCapacityEvictsTheOldestEntry)
{
    LruCache<int, std::string> cache(2);
    EXPECT_EQ(cache.insert(1, "a"), std::nullopt);
    EXPECT_EQ(cache.insert(2, "b"), std::nullopt);

    // Cache holds {1, 2}; inserting 3 must evict 1, the least recently touched.
    const std::optional<std::pair<int, std::string>> evicted = cache.insert(3, "c");
    ASSERT_TRUE(evicted.has_value());
    EXPECT_EQ(evicted->first, 1);
    EXPECT_EQ(evicted->second, "a");
    EXPECT_EQ(cache.size(), std::size_t(2));
    EXPECT_EQ(*cache.touch(2), "b");
    EXPECT_EQ(*cache.touch(3), "c");
    EXPECT_EQ(cache.touch(1), nullptr);
}

TEST(Unit_ThumbnailLru, TouchingAnEntryProtectsItFromTheNextEviction)
{
    LruCache<int, std::string> cache(2);
    cache.insert(1, "a");
    cache.insert(2, "b");

    // Touching 1 makes 2 the least recently used, even though 2 was inserted more recently.
    ASSERT_NE(cache.touch(1), nullptr);

    const std::optional<std::pair<int, std::string>> evicted = cache.insert(3, "c");
    ASSERT_TRUE(evicted.has_value());
    EXPECT_EQ(evicted->first, 2);
    EXPECT_NE(cache.touch(1), nullptr);
    EXPECT_NE(cache.touch(3), nullptr);
}

TEST(Unit_ThumbnailLru, ReinsertingAnExistingKeyOverwritesWithoutEvicting)
{
    LruCache<int, std::string> cache(2);
    cache.insert(1, "a");
    cache.insert(2, "b");

    EXPECT_EQ(cache.insert(1, "a-updated"), std::nullopt);
    EXPECT_EQ(cache.size(), std::size_t(2));
    EXPECT_EQ(*cache.touch(1), "a-updated");
}

TEST(Unit_ThumbnailLru, DrainEmptiesTheCacheAndReturnsEveryEntry)
{
    LruCache<int, std::string> cache(4);
    cache.insert(1, "a");
    cache.insert(2, "b");

    const std::vector<std::pair<int, std::string>> drained = cache.drain();
    EXPECT_EQ(drained.size(), std::size_t(2));
    EXPECT_EQ(cache.size(), std::size_t(0));
    EXPECT_EQ(cache.touch(1), nullptr);
    EXPECT_EQ(cache.touch(2), nullptr);
}
```

- [ ] **Step 3: Wire the test file into the test binary**

In `tests/CMakeLists.txt`, add the new file right after `unit/test_thumbnail_downscale.cpp` (from
Task 2, Step 8):

```cmake
    unit/test_thumbnail_downscale.cpp
    unit/test_thumbnail_lru.cpp
```

- [ ] **Step 4: Verify by reading, not by building**

Per Global Constraints, this machine cannot run `se test`. Trace
`InsertingPastCapacityEvictsTheOldestEntry` by hand through `insert()`: after the two seed
inserts, `order_` is `[2, 1]` (front = most recent) and `index_` maps `1 -> iterator to order_`'s
second node, `2 -> iterator to order_`'s first node. Inserting `3` pushes `order_` to
`[3, 2, 1]` (size 3 > capacity 2), so `order_.back()` is `1`'s entry — confirm the code evicts
exactly that node and erases `1` from `index_`, not `2`.

- [ ] **Step 5: Commit**

```bash
git add engine/domain/imaging/include/SushiEngine/imaging/lru_cache.hpp \
        tests/CMakeLists.txt tests/unit/test_thumbnail_lru.cpp
git commit -m "feat(imaging): add a generic LRU cache for the thumbnail pipeline"
```

---

### Task 4: `ThumbnailCache` — Vulkan resource lifecycle, worker thread, decode/upload pipeline

**Files:**
- Create: `applications/editor/source/project/thumbnail_cache.hpp`
- Create: `applications/editor/source/project/thumbnail_cache.cpp`

**Interfaces:**
- Consumes:
  - `SushiEngine::Render::NativeDeviceHandles` (Task 1) — `device`, `graphics_queue`,
    `graphics_queue_family`, `allocator`, all `void*`/`uint32_t` per that struct.
  - `SushiEngine::Imaging::box_downscale_rgba8` (Task 2).
  - `SushiEngine::Imaging::LruCache<Key, Value>` (Task 3).
  - `SushiEngine::Editor::Console::append(LogLevel, const std::string&)`
    (`applications/editor/source/core/console.hpp`, already in the tree).
  - `SushiEngine::Editor::ImGuiBackend::register_texture(void* sampler, void* image_view) ->
    ImTextureID` and `::unregister_texture(ImTextureID)`
    (`applications/editor/source/ui/imgui_backend.hpp`, already in the tree).
- Produces: `SushiEngine::Editor::ThumbnailCache`, consumed by Task 5 (construction/wiring) and
  Task 6 (`texture_for`/`update` calls from the Grid view):
  - `ThumbnailCache(SushiEngine::Render::NativeDeviceHandles handles, ImGuiBackend& backend,
    Console& console)`
  - `~ThumbnailCache()`
  - `void update()`
  - `std::optional<ImTextureID> texture_for(const std::filesystem::path& path)`

This is one cohesive class and one cohesive gate: its pieces (construction, the worker thread,
the upload path, teardown) only make sense reviewed together, so there is one task rather than
several. There is no unit test for this task — every line of it either touches Vulkan/ImGui or
a live thread, neither of which this codebase's test binary can reach (see Global Constraints and
the spec's Testing section) — verification is the by-hand trace in Step 4 below plus the
downstream manual verification once the whole branch builds.

- [ ] **Step 1: Write the header**

Create `applications/editor/source/project/thumbnail_cache.hpp`:

```cpp
/**************************************************************************/
/* thumbnail_cache.hpp                                                    */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/* Licensed under the Apache License, Version 2.0 (the "License");        */
/* you may not use this file except in compliance with the License.       */
/* You may obtain a copy of the License at                                */
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

#pragma once

/**
 * @file thumbnail_cache.hpp
 * @brief Real image content for the Project panel's Grid view tiles.
 *
 * The second editor component that legitimately speaks Vulkan directly (the first is
 * ImGuiBackend) — everything here is the minimal, from-scratch image/view/sampler/staging-buffer
 * path a 128x128 thumbnail needs, deliberately not routed through the renderer's own
 * TextureLibrary, which only ever hands back a bindless heap index rather than the raw
 * VkImageView/VkSampler pair ImGui::Image needs. A background thread decodes and downscales;
 * update() uploads a small budget of finished decodes to the GPU each frame; an LRU keeps
 * resident GPU memory bounded. See docs/superpowers/specs/2026-08-07-project-panel-thumbnails-design.md.
 */

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <imgui.h>
#include <vk_mem_alloc.h>
#include <vulkan/vulkan.h>

#include <SushiEngine/imaging/lru_cache.hpp>
#include <SushiEngine/render/rhi/device.hpp>

#include "../core/console.hpp"
#include "../ui/imgui_backend.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief Decodes, downscales, and GPU-uploads image thumbnails on a background thread.
         *
         * Non-copyable: it owns a live worker thread and a set of Vulkan resources tied to one
         * device. Construction starts the worker; destruction stops it, joins it, and frees
         * every resident thumbnail's Vulkan resources before the device it was built against
         * may be torn down — the caller (main()) must destroy this before the renderer.
         */
        class ThumbnailCache
        {
            public:
                /**
                 * @brief Binds the cache to a device and starts its decode worker.
                 * @param handles The window renderer's native handles (needs @c device,
                 *   @c allocator, @c graphics_queue, @c graphics_queue_family all set).
                 * @param backend The ImGui Vulkan backend thumbnails are registered with.
                 * @param console Where an upload failure is logged (see class docs).
                 * @throws std::runtime_error if the internal Vulkan command pool cannot be
                 *   created.
                 */
                ThumbnailCache(SushiEngine::Render::NativeDeviceHandles handles,
                               ImGuiBackend& backend, Console& console);

                /** @brief Stops and joins the worker, then frees every resident thumbnail. */
                ~ThumbnailCache();

                ThumbnailCache(const ThumbnailCache&) = delete;
                ThumbnailCache& operator=(const ThumbnailCache&) = delete;

                /**
                 * @brief Uploads a small, fixed budget of finished decodes to the GPU.
                 *
                 * Call once per frame, before any panel that might call @ref texture_for reads
                 * its result for this frame.
                 */
                void update();

                /**
                 * @brief The thumbnail texture for @p path, requesting a decode if needed.
                 * @return A texture id if @p path's thumbnail is already resident; otherwise
                 *   @c std::nullopt, having enqueued a decode request unless one is already
                 *   in flight for the same path. A path whose decode failed keeps returning
                 *   @c std::nullopt forever (see class docs on failure handling).
                 */
                std::optional<ImTextureID> texture_for(const std::filesystem::path& path);

            private:
                /** @brief One decoded, downscaled thumbnail awaiting a GPU upload. */
                struct DecodedImage
                {
                    std::string path;
                    std::vector<std::uint8_t> pixels; // THUMBNAIL_SIZE^2 * 4 bytes, RGBA8.
                };

                /** @brief One thumbnail's live Vulkan resources plus its ImGui texture id. */
                struct ResidentThumbnail
                {
                    VkImage image = VK_NULL_HANDLE;
                    VmaAllocation allocation = VK_NULL_HANDLE;
                    VkImageView view = VK_NULL_HANDLE;
                    VkSampler sampler = VK_NULL_HANDLE;
                    ImTextureID texture = static_cast<ImTextureID>(0);
                };

                void worker_main();
                void upload_one(const DecodedImage& decoded);
                void destroy_thumbnail(ResidentThumbnail& thumbnail);

                static constexpr std::uint32_t THUMBNAIL_SIZE = 128;
                static constexpr std::size_t RESIDENT_CAPACITY = 256;
                static constexpr int UPLOADS_PER_FRAME = 2;

                VkDevice device_ = VK_NULL_HANDLE;
                VmaAllocator allocator_ = VK_NULL_HANDLE;
                VkQueue graphics_queue_ = VK_NULL_HANDLE;
                VkCommandPool command_pool_ = VK_NULL_HANDLE;
                ImGuiBackend& backend_;
                Console& console_;

                std::thread worker_;
                std::mutex requests_mutex_;
                std::condition_variable requests_cv_;
                std::deque<std::string> requests_;
                bool stop_ = false;

                std::mutex ready_mutex_;
                std::deque<DecodedImage> ready_;

                // Main-thread-only state; never touched from worker_.
                SushiEngine::Imaging::LruCache<std::string, ResidentThumbnail> resident_{
                    RESIDENT_CAPACITY};
                std::unordered_map<std::string, bool> in_flight_;
        };
    } // namespace Editor
} // namespace SushiEngine
```

- [ ] **Step 2: Write the implementation**

Create `applications/editor/source/project/thumbnail_cache.cpp`:

```cpp
/**************************************************************************/
/* thumbnail_cache.cpp                                                    */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/* Licensed under the Apache License, Version 2.0 (the "License");        */
/* you may not use this file except in compliance with the License.       */
/* You may obtain a copy of the License at                                */
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

#include "thumbnail_cache.hpp"

#include <cstring>
#include <stdexcept>
#include <utility>

#include <stb_image.h>

#include <SushiEngine/imaging/box_downscale.hpp>

namespace SushiEngine
{
    namespace Editor
    {
        ThumbnailCache::ThumbnailCache(SushiEngine::Render::NativeDeviceHandles handles,
                                       ImGuiBackend& backend, Console& console)
            : device_(static_cast<VkDevice>(handles.device))
            , allocator_(static_cast<VmaAllocator>(handles.allocator))
            , graphics_queue_(static_cast<VkQueue>(handles.graphics_queue))
            , backend_(backend)
            , console_(console)
        {
            VkCommandPoolCreateInfo pool_info{};
            pool_info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
            pool_info.flags = VK_COMMAND_POOL_CREATE_TRANSIENT_BIT;
            pool_info.queueFamilyIndex = handles.graphics_queue_family;
            if (vkCreateCommandPool(device_, &pool_info, nullptr, &command_pool_) != VK_SUCCESS)
                throw std::runtime_error(
                    "SushiEngine: ThumbnailCache command pool creation failed");

            worker_ = std::thread(&ThumbnailCache::worker_main, this);
        }

        ThumbnailCache::~ThumbnailCache()
        {
            {
                std::lock_guard<std::mutex> lock(requests_mutex_);
                stop_ = true;
            }
            requests_cv_.notify_all();
            if (worker_.joinable())
                worker_.join();

            for (auto& entry : resident_.drain())
                destroy_thumbnail(entry.second);

            vkDestroyCommandPool(device_, command_pool_, nullptr);
        }

        std::optional<ImTextureID> ThumbnailCache::texture_for(const std::filesystem::path& path)
        {
            const std::string key = path.string();
            if (ResidentThumbnail* found = resident_.touch(key))
                return found->texture;

            if (in_flight_.find(key) == in_flight_.end())
            {
                in_flight_[key] = true;
                {
                    std::lock_guard<std::mutex> lock(requests_mutex_);
                    requests_.push_back(key);
                }
                requests_cv_.notify_one();
            }
            return std::nullopt;
        }

        void ThumbnailCache::update()
        {
            for (int i = 0; i < UPLOADS_PER_FRAME; ++i)
            {
                DecodedImage decoded;
                {
                    std::lock_guard<std::mutex> lock(ready_mutex_);
                    if (ready_.empty())
                        return;
                    decoded = std::move(ready_.front());
                    ready_.pop_front();
                }
                in_flight_.erase(decoded.path);
                upload_one(decoded);
            }
        }

        void ThumbnailCache::worker_main()
        {
            for (;;)
            {
                std::string path;
                {
                    std::unique_lock<std::mutex> lock(requests_mutex_);
                    requests_cv_.wait(lock, [this] { return stop_ || !requests_.empty(); });
                    if (stop_)
                        return;
                    path = requests_.front();
                    requests_.pop_front();
                }

                // A decode failure (bad path, unsupported format, corrupt file) is silently
                // dropped: the tile keeps showing Phase 1's picture-frame glyph forever for
                // this path, which is the design's stated fallback rather than an error state.
                int width = 0;
                int height = 0;
                int source_channels = 0;
                stbi_uc* pixels =
                    stbi_load(path.c_str(), &width, &height, &source_channels, 4);
                if (pixels == nullptr)
                    continue;

                DecodedImage decoded;
                decoded.path = path;
                decoded.pixels = SushiEngine::Imaging::box_downscale_rgba8(
                    pixels, static_cast<std::uint32_t>(width),
                    static_cast<std::uint32_t>(height), THUMBNAIL_SIZE, THUMBNAIL_SIZE);
                stbi_image_free(pixels);

                std::lock_guard<std::mutex> lock(ready_mutex_);
                ready_.push_back(std::move(decoded));
            }
        }

        void ThumbnailCache::upload_one(const DecodedImage& decoded)
        {
            ResidentThumbnail thumbnail;
            VkImage image = VK_NULL_HANDLE;
            VmaAllocation allocation = VK_NULL_HANDLE;
            VkImageView view = VK_NULL_HANDLE;
            VkSampler sampler = VK_NULL_HANDLE;
            VkBuffer staging = VK_NULL_HANDLE;
            VmaAllocation staging_allocation = VK_NULL_HANDLE;
            VkCommandBuffer command = VK_NULL_HANDLE;
            VkFence fence = VK_NULL_HANDLE;

            try
            {
                const VkDeviceSize byte_size =
                    VkDeviceSize(THUMBNAIL_SIZE) * THUMBNAIL_SIZE * 4;

                VkImageCreateInfo image_info{};
                image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
                image_info.imageType = VK_IMAGE_TYPE_2D;
                image_info.format = VK_FORMAT_R8G8B8A8_UNORM;
                image_info.extent = {THUMBNAIL_SIZE, THUMBNAIL_SIZE, 1};
                image_info.mipLevels = 1;
                image_info.arrayLayers = 1;
                image_info.samples = VK_SAMPLE_COUNT_1_BIT;
                image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
                image_info.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;

                VmaAllocationCreateInfo alloc_info{};
                alloc_info.usage = VMA_MEMORY_USAGE_AUTO;
                if (vmaCreateImage(allocator_, &image_info, &alloc_info, &image, &allocation,
                                   nullptr) != VK_SUCCESS)
                    throw std::runtime_error("vmaCreateImage(thumbnail) failed");

                VkImageViewCreateInfo view_info{};
                view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
                view_info.image = image;
                view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
                view_info.format = VK_FORMAT_R8G8B8A8_UNORM;
                view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                view_info.subresourceRange.levelCount = 1;
                view_info.subresourceRange.layerCount = 1;
                if (vkCreateImageView(device_, &view_info, nullptr, &view) != VK_SUCCESS)
                    throw std::runtime_error("vkCreateImageView(thumbnail) failed");

                VkSamplerCreateInfo sampler_info{};
                sampler_info.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
                sampler_info.magFilter = VK_FILTER_LINEAR;
                sampler_info.minFilter = VK_FILTER_LINEAR;
                sampler_info.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
                sampler_info.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
                sampler_info.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
                sampler_info.maxLod = 1.0f;
                if (vkCreateSampler(device_, &sampler_info, nullptr, &sampler) != VK_SUCCESS)
                    throw std::runtime_error("vkCreateSampler(thumbnail) failed");

                VkBufferCreateInfo staging_info{};
                staging_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
                staging_info.size = byte_size;
                staging_info.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
                VmaAllocationCreateInfo staging_alloc_info{};
                staging_alloc_info.usage = VMA_MEMORY_USAGE_AUTO;
                staging_alloc_info.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
                                            VMA_ALLOCATION_CREATE_MAPPED_BIT;
                VmaAllocationInfo staging_mapped{};
                if (vmaCreateBuffer(allocator_, &staging_info, &staging_alloc_info, &staging,
                                    &staging_allocation, &staging_mapped) != VK_SUCCESS)
                    throw std::runtime_error("vmaCreateBuffer(thumbnail staging) failed");
                std::memcpy(staging_mapped.pMappedData, decoded.pixels.data(),
                           static_cast<std::size_t>(byte_size));

                VkCommandBufferAllocateInfo command_alloc{};
                command_alloc.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
                command_alloc.commandPool = command_pool_;
                command_alloc.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
                command_alloc.commandBufferCount = 1;
                if (vkAllocateCommandBuffers(device_, &command_alloc, &command) != VK_SUCCESS)
                    throw std::runtime_error("vkAllocateCommandBuffers(thumbnail) failed");

                VkCommandBufferBeginInfo begin{};
                begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
                begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
                vkBeginCommandBuffer(command, &begin);

                VkImageMemoryBarrier2 to_transfer{};
                to_transfer.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
                to_transfer.srcStageMask = VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT;
                to_transfer.dstStageMask = VK_PIPELINE_STAGE_2_COPY_BIT;
                to_transfer.dstAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
                to_transfer.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
                to_transfer.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
                to_transfer.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                to_transfer.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                to_transfer.image = image;
                to_transfer.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
                VkDependencyInfo dependency_to_transfer{};
                dependency_to_transfer.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
                dependency_to_transfer.imageMemoryBarrierCount = 1;
                dependency_to_transfer.pImageMemoryBarriers = &to_transfer;
                vkCmdPipelineBarrier2(command, &dependency_to_transfer);

                VkBufferImageCopy copy{};
                copy.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                copy.imageSubresource.layerCount = 1;
                copy.imageExtent = {THUMBNAIL_SIZE, THUMBNAIL_SIZE, 1};
                vkCmdCopyBufferToImage(command, staging, image,
                                       VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copy);

                VkImageMemoryBarrier2 to_shader_read{};
                to_shader_read.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
                to_shader_read.srcStageMask = VK_PIPELINE_STAGE_2_COPY_BIT;
                to_shader_read.srcAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
                to_shader_read.dstStageMask = VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT;
                to_shader_read.dstAccessMask = VK_ACCESS_2_SHADER_SAMPLED_READ_BIT;
                to_shader_read.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
                to_shader_read.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
                to_shader_read.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                to_shader_read.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                to_shader_read.image = image;
                to_shader_read.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
                VkDependencyInfo dependency_to_shader_read{};
                dependency_to_shader_read.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
                dependency_to_shader_read.imageMemoryBarrierCount = 1;
                dependency_to_shader_read.pImageMemoryBarriers = &to_shader_read;
                vkCmdPipelineBarrier2(command, &dependency_to_shader_read);

                vkEndCommandBuffer(command);

                VkFenceCreateInfo fence_info{};
                fence_info.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
                if (vkCreateFence(device_, &fence_info, nullptr, &fence) != VK_SUCCESS)
                    throw std::runtime_error("vkCreateFence(thumbnail) failed");

                VkCommandBufferSubmitInfo command_submit{};
                command_submit.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO;
                command_submit.commandBuffer = command;
                VkSubmitInfo2 submit{};
                submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO_2;
                submit.commandBufferInfoCount = 1;
                submit.pCommandBufferInfos = &command_submit;
                if (vkQueueSubmit2(graphics_queue_, 1, &submit, fence) != VK_SUCCESS)
                    throw std::runtime_error("vkQueueSubmit2(thumbnail) failed");

                // Synchronous: at most UPLOADS_PER_FRAME of these run per frame, unlike the
                // render loop's own frames-in-flight uploads, which must never stall a frame.
                vkWaitForFences(device_, 1, &fence, VK_TRUE, UINT64_MAX);
                vkDestroyFence(device_, fence, nullptr);
                vkFreeCommandBuffers(device_, command_pool_, 1, &command);
                vmaDestroyBuffer(allocator_, staging, staging_allocation);

                thumbnail.image = image;
                thumbnail.allocation = allocation;
                thumbnail.view = view;
                thumbnail.sampler = sampler;
                thumbnail.texture = backend_.register_texture(sampler, view);
            }
            catch (const std::exception& error)
            {
                console_.append(LogLevel::Warning, "Thumbnail upload failed for '" +
                                                        decoded.path + "': " + error.what());
                if (fence != VK_NULL_HANDLE)
                    vkDestroyFence(device_, fence, nullptr);
                if (command != VK_NULL_HANDLE)
                    vkFreeCommandBuffers(device_, command_pool_, 1, &command);
                if (staging != VK_NULL_HANDLE)
                    vmaDestroyBuffer(allocator_, staging, staging_allocation);
                if (sampler != VK_NULL_HANDLE)
                    vkDestroySampler(device_, sampler, nullptr);
                if (view != VK_NULL_HANDLE)
                    vkDestroyImageView(device_, view, nullptr);
                if (image != VK_NULL_HANDLE)
                    vmaDestroyImage(allocator_, image, allocation);
                return;
            }

            std::optional<std::pair<std::string, ResidentThumbnail>> evicted =
                resident_.insert(decoded.path, thumbnail);
            if (evicted.has_value())
                destroy_thumbnail(evicted->second);
        }

        void ThumbnailCache::destroy_thumbnail(ResidentThumbnail& thumbnail)
        {
            backend_.unregister_texture(thumbnail.texture);
            if (thumbnail.sampler != VK_NULL_HANDLE)
                vkDestroySampler(device_, thumbnail.sampler, nullptr);
            if (thumbnail.view != VK_NULL_HANDLE)
                vkDestroyImageView(device_, thumbnail.view, nullptr);
            if (thumbnail.image != VK_NULL_HANDLE)
                vmaDestroyImage(allocator_, thumbnail.image, thumbnail.allocation);
        }
    } // namespace Editor
} // namespace SushiEngine
```

- [ ] **Step 3: Verify by reading, not by building**

Per Global Constraints, this machine cannot build or run this code — read it through for these
specific hazards instead:

1. **Lifetime of `decoded.path` in the catch block.** `upload_one` takes `const DecodedImage&
   decoded` by reference; confirm the `catch` block's use of `decoded.path` is safe (it is — the
   caller, `update()`, owns `decoded` on its own stack frame for the whole call).
2. **Thread-safety boundary.** Confirm `resident_` and `in_flight_` are read/written only from
   `texture_for()` and `update()`/`upload_one()` — i.e. only from the main thread — and that
   `worker_main()` never touches either, only `requests_`/`ready_` under their own mutexes.
3. **Double-free on a partially constructed `ResidentThumbnail`.** Confirm every early `throw` in
   `upload_one`'s `try` block leaves exactly the resources created so far for the `catch` block's
   guarded cleanup to free, with no double-destroy (e.g. the `vkCreateFence` failure path must
   not attempt to destroy `command`'s not-yet-submitted work twice).
4. **Destructor ordering.** Confirm `~ThumbnailCache()` stops the worker and joins it *before*
   touching `resident_` or `command_pool_` — a still-running worker could otherwise decode into
   `ready_` after that queue's own destructor already ran undefined-behavior territory (it
   cannot, in practice, join happens first, but re-read the actual statement order to be sure the
   text matches this reasoning).

- [ ] **Step 4: Commit**

```bash
git add applications/editor/source/project/thumbnail_cache.hpp \
        applications/editor/source/project/thumbnail_cache.cpp
git commit -m "feat(editor): add ThumbnailCache, the Project panel's image decode/upload pipeline"
```

---

### Task 5: Wire `ThumbnailCache` into `EditorContext` and `main.cpp`

**Files:**
- Modify: `applications/editor/source/core/editor_context.hpp`
- Modify: `applications/editor/source/main.cpp`
- Modify: `applications/editor/CMakeLists.txt`

**Interfaces:**
- Consumes: `SushiEngine::Editor::ThumbnailCache` (Task 4).
- Produces: `EditorContext::thumbnail_cache` (`ThumbnailCache*`, non-owning, following the exact
  pattern `EditorContext::cook_bake_state` already uses), which Task 6 reads from
  `project_panel.cpp`.

- [ ] **Step 1: Forward-declare `ThumbnailCache` in `editor_context.hpp`**

In `applications/editor/source/core/editor_context.hpp`, the `namespace Editor` block currently
opens with (lines 74-83):

```cpp
    namespace Editor
    {
        /** @brief The live particle-effect preview, owned by main() (see effect_preview.hpp). */
        class EffectPreview;

        /** @brief The live GPU-skinned character preview, owned by main() (see animated_mesh_preview.hpp). */
        class AnimatedMeshPreview;

        /** @brief The live editor audio system, owned by main() (see audio/audio_editor_system.hpp). */
        class AudioEditorSystem;
```

Add one more forward declaration after `AudioEditorSystem`:

```cpp
        /** @brief The live editor audio system, owned by main() (see audio/audio_editor_system.hpp). */
        class AudioEditorSystem;

        /** @brief The Project panel's image thumbnail pipeline, owned by main() (see project/thumbnail_cache.hpp). */
        class ThumbnailCache;
```

- [ ] **Step 2: Add the field to `EditorContext`**

In the same file, `EditorContext`'s field list currently has (lines 260-264):

```cpp
            // The Bake surface's model, owned by main() and injected here so a panel that
            // brings a mesh into the project (today: the Project panel's glTF open/preview
            // flow) can queue it for cooking without owning a worker thread itself. Null in
            // a headless editor, which is why every use is guarded.
            Authoring::CookBakeState* cook_bake_state = nullptr;
```

Add the new field immediately after it:

```cpp
            // The Bake surface's model, owned by main() and injected here so a panel that
            // brings a mesh into the project (today: the Project panel's glTF open/preview
            // flow) can queue it for cooking without owning a worker thread itself. Null in
            // a headless editor, which is why every use is guarded.
            Authoring::CookBakeState* cook_bake_state = nullptr;

            // The Project panel's image thumbnail pipeline, owned by main() and injected here
            // so the Grid view can ask for a tile's real thumbnail texture. Null in a headless
            // editor, which is why every use is guarded.
            ThumbnailCache* thumbnail_cache = nullptr;
```

- [ ] **Step 3: Include the header and add `Stb`'s include directory in `applications/editor/CMakeLists.txt`**

The file currently opens with (lines 19-25):

```cmake
find_package(SDL2 CONFIG REQUIRED)
# The imported Vulkan targets are directory-scoped; the renderer's own find_package does not
# export them here, so the editor resolves them itself.
find_package(VulkanHeaders CONFIG REQUIRED)
find_package(VulkanLoader CONFIG REQUIRED)
# Header-only JSON for the editor's persisted Preferences and the .sushiscene format.
find_package(nlohmann_json CONFIG REQUIRED)
```

Add a `Stb` lookup right after the `nlohmann_json` one — the editor needs it directly because
`sushiengine_render` only exposes its own `Stb_INCLUDE_DIR` `PRIVATE`ly (per Global Constraints):

```cmake
find_package(SDL2 CONFIG REQUIRED)
# The imported Vulkan targets are directory-scoped; the renderer's own find_package does not
# export them here, so the editor resolves them itself.
find_package(VulkanHeaders CONFIG REQUIRED)
find_package(VulkanLoader CONFIG REQUIRED)
# Header-only JSON for the editor's persisted Preferences and the .sushiscene format.
find_package(nlohmann_json CONFIG REQUIRED)
# stb_image for the Project panel's thumbnail decode path. The symbols themselves are already
# compiled once into sushiengine_render (source/material/stb_impl.cpp) and this target already
# links that library, so this only adds the header search path — sushiengine_render keeps its
# own Stb_INCLUDE_DIR private, so the editor must resolve it again itself.
find_package(Stb REQUIRED)
```

The `add_executable(sushiengine_editor ...)` source list currently has, among the `source/project/`
entries:

```cmake
    source/project/project_panel.cpp
    source/project/project_picker.cpp
```

Add the new file:

```cmake
    source/project/project_panel.cpp
    source/project/project_picker.cpp
    source/project/thumbnail_cache.cpp
```

The `target_link_libraries(sushiengine_editor PRIVATE ...)` block currently reads:

```cmake
target_link_libraries(sushiengine_editor PRIVATE
    sushiengine_imgui
    sushiengine_render
    sushiengine_simulation
    sushiengine_serialization
    sushiengine_authoring
    sushiengine_model
    sushiengine_model_import
    sushiengine_platform
    sushiengine_input_backend
    sushiengine_audio_backend
    nlohmann_json::nlohmann_json)
```

Add `sushiengine_imaging`:

```cmake
target_link_libraries(sushiengine_editor PRIVATE
    sushiengine_imgui
    sushiengine_render
    sushiengine_simulation
    sushiengine_serialization
    sushiengine_authoring
    sushiengine_model
    sushiengine_model_import
    sushiengine_platform
    sushiengine_input_backend
    sushiengine_audio_backend
    sushiengine_imaging
    nlohmann_json::nlohmann_json)
```

Finally, add the include directory. This file has no existing `target_include_directories` call
for `sushiengine_editor` (every other header the editor needs arrives transitively through a
linked target's `PUBLIC`/`INTERFACE` include dirs); add one new call right after the
`target_link_libraries` block above:

```cmake
# Stb_INCLUDE_DIR is PRIVATE on sushiengine_render, so it does not arrive transitively; the
# thumbnail cache's stb_image.h include needs it resolved here directly.
target_include_directories(sushiengine_editor PRIVATE "${Stb_INCLUDE_DIR}")
```

- [ ] **Step 4: Construct and wire `ThumbnailCache` in `main.cpp`**

In `applications/editor/source/main.cpp`, add the include near the other project-panel-adjacent
includes at the top of the file (find the line including `"project/project_panel.hpp"` or
similar in the existing include block and add this line next to it):

```cpp
#include "project/thumbnail_cache.hpp"
```

Then, at lines 343-346, the file currently reads:

```cpp
        // Injected so a panel that brings a mesh into the project can queue it for
        // cooking automatically (see project_panel.cpp's glTF open handler) instead of
        // an artist having to find and press the Bake panel's button for every asset.
        context.cook_bake_state = &cook_bake_state;
```

Add the `ThumbnailCache` construction and wiring immediately after that line:

```cpp
        // Injected so a panel that brings a mesh into the project can queue it for
        // cooking automatically (see project_panel.cpp's glTF open handler) instead of
        // an artist having to find and press the Bake panel's button for every asset.
        context.cook_bake_state = &cook_bake_state;

        // The Project panel's real-image-thumbnail pipeline. Declared after `imgui` and
        // `renderer` (both already constructed above) so it is destroyed before either of
        // them, in the normal C++ stack-unwind order — its worker thread and every resident
        // Vulkan resource must be torn down while the device and allocator they were built
        // against are still alive.
        SushiEngine::Editor::ThumbnailCache thumbnail_cache(renderer->native_handles(), imgui,
                                                             context.console);
        context.thumbnail_cache = &thumbnail_cache;
```

Then, at line 564, the main loop currently reads:

```cpp
            imgui.new_frame();
```

Add the per-frame upload budget call immediately after it:

```cpp
            imgui.new_frame();

            context.thumbnail_cache->update();
```

- [ ] **Step 5: Verify by reading, not by building**

Per Global Constraints, this machine cannot build. Re-read the full span of `main.cpp` from the
`renderer`/`imgui` declarations (lines ~162-165) down through the new `thumbnail_cache`
declaration and confirm the C++ stack-unwind order holds: local variables are destroyed in the
reverse of their declaration order, so `thumbnail_cache` (declared after both) is destroyed
before `imgui` and `renderer` are, with no explicit destructor call needed anywhere in `main`.

- [ ] **Step 6: Commit**

```bash
git add applications/editor/source/core/editor_context.hpp \
        applications/editor/source/main.cpp \
        applications/editor/CMakeLists.txt
git commit -m "feat(editor): construct and wire ThumbnailCache into the editor's main loop"
```

---

### Task 6: Grid view integration — viewport-culled real thumbnails

**Files:**
- Modify: `applications/editor/source/project/project_panel.cpp`

**Interfaces:**
- Consumes: `EditorContext::thumbnail_cache` (Task 5), `ThumbnailCache::texture_for` (Task 4).
- Produces: nothing further downstream — this is the plan's last task.

- [ ] **Step 1: Include the header**

In `applications/editor/source/project/project_panel.cpp`, the include block currently reads
(lines 24-33):

```cpp
#include "project_panel.hpp"

#include <SushiEngine/authoring/cook_bake_state.hpp>
#include <SushiEngine/model/import_settings_io.hpp>

#include "prefab_serializer.hpp"

#include "../animation/animated_mesh_preview.hpp"
#include "../scene/scene_commands.hpp"
#include "../ui/panel_widgets.hpp"
```

Add the new include next to `prefab_serializer.hpp`:

```cpp
#include "project_panel.hpp"

#include <SushiEngine/authoring/cook_bake_state.hpp>
#include <SushiEngine/model/import_settings_io.hpp>

#include "prefab_serializer.hpp"
#include "thumbnail_cache.hpp"

#include "../animation/animated_mesh_preview.hpp"
#include "../scene/scene_commands.hpp"
#include "../ui/panel_widgets.hpp"
```

- [ ] **Step 2: Draw the real thumbnail in place of the glyph, with viewport culling**

`draw_project_grid_view` (lines 552-623) currently draws the icon unconditionally for every
entry:

```cpp
                        ImDrawList* draw_list = ImGui::GetWindowDrawList();
                        const bool selected = context.selected_project_path == path_string;
                        if (selected || ImGui::IsItemHovered())
                            draw_list->AddRectFilled(
                                origin, ImVec2(origin.x + tile_size, origin.y + tile_size),
                                ImGui::GetColorU32(selected ? ImGuiCol_ButtonActive : ImGuiCol_ButtonHovered),
                                3.0f);
                        draw_entry_icon(draw_list, ImVec2(origin.x + (tile_size - icon_size) * 0.5f, origin.y),
                                       icon_size, entry_category(entry.path(), is_dir),
                                       ImGui::GetColorU32(ImGuiCol_Text));
```

Replace the `draw_entry_icon` call with a check for a resident real thumbnail on
`EntryCategory::Image` entries that are actually visible in the scrolled child window, falling
back to the existing glyph otherwise:

```cpp
                        ImDrawList* draw_list = ImGui::GetWindowDrawList();
                        const bool selected = context.selected_project_path == path_string;
                        if (selected || ImGui::IsItemHovered())
                            draw_list->AddRectFilled(
                                origin, ImVec2(origin.x + tile_size, origin.y + tile_size),
                                ImGui::GetColorU32(selected ? ImGuiCol_ButtonActive : ImGuiCol_ButtonHovered),
                                3.0f);

                        const EntryCategory category = entry_category(entry.path(), is_dir);
                        std::optional<ImTextureID> thumbnail_texture;
                        if (category == EntryCategory::Image && context.thumbnail_cache != nullptr &&
                            ImGui::IsRectVisible(origin, ImVec2(origin.x + tile_size, origin.y + tile_size)))
                        {
                            // IsRectVisible tests the tile's screen rect against the grid child
                            // window's current clip rect, so a tile scrolled out of view never
                            // requests a decode — a folder full of images scrolled past should
                            // never front-load every one of them.
                            thumbnail_texture = context.thumbnail_cache->texture_for(entry.path());
                        }

                        if (thumbnail_texture.has_value())
                        {
                            draw_list->AddImage(*thumbnail_texture,
                                                ImVec2(origin.x + (tile_size - icon_size) * 0.5f, origin.y),
                                                ImVec2(origin.x + (tile_size - icon_size) * 0.5f + icon_size,
                                                       origin.y + icon_size));
                        }
                        else
                        {
                            draw_entry_icon(draw_list,
                                           ImVec2(origin.x + (tile_size - icon_size) * 0.5f, origin.y),
                                           icon_size, category, ImGui::GetColorU32(ImGuiCol_Text));
                        }
```

The later `entry_category(entry.path(), is_dir)` call inside this same iteration (used only for
`draw_entry_icon` before this change) is now redundant with the `category` computed above — no
other call site in this function needs it a second time.

`ImGui::IsRectVisible` takes the same absolute screen-space coordinates `origin` already is
(`ImGui::GetCursorScreenPos()`, captured earlier in this same loop iteration), and checks them
against the current window's clip rect — the same test ImGui itself uses internally to skip
drawing off-screen content, so this reuses an existing, already-correct primitive rather than
re-deriving the scroll math by hand.

- [ ] **Step 3: Verify by reading, not by building**

Per Global Constraints, this machine cannot build or run the editor. Re-read the edited block
against three cases by hand: (1) a `Folder`/`Scene`/etc. entry — `category != EntryCategory::Image`
short-circuits `thumbnail_texture` to stay unset, so the `else` branch runs and behavior is
byte-for-byte the same as before this task; (2) an `Image` entry scrolled out of view —
`ImGui::IsRectVisible` should evaluate false for it, so `context.thumbnail_cache->texture_for` is
never called for it this frame; (3) an `Image` entry in view whose thumbnail has not resolved yet
— `texture_for` returns `std::nullopt`, so `draw_entry_icon` still runs (Phase 1's glyph), and
the tile only switches to `draw_list->AddImage` on a later frame once the background
decode/upload completes.

- [ ] **Step 4: Commit**

```bash
git add applications/editor/source/project/project_panel.cpp
git commit -m "feat(editor): show real image thumbnails in the Project panel's Grid view"
```

---

## Manual verification (after the branch builds)

No ImGui panel or Vulkan resource path in this codebase has automated coverage beyond the
box-filter/LRU units this plan adds (see Global Constraints and the spec's Testing section). Once
the user has built the branch, verification is the checklist the approved spec already states:
open a Project folder with several image files of different formats/sizes and confirm each shows
its real content in Grid view within a couple of frames of becoming visible; scroll a large image
folder and confirm off-screen images do not front-load; confirm List view still always shows the
generic image glyph, never a real thumbnail; confirm a deliberately corrupted image file keeps
showing the Phase 1 glyph indefinitely with no error dialog, no crash, and no console warning
(decode failures are silent by design); confirm an editor session that loaded several thumbnails
shuts down cleanly under any Vulkan validation layers already enabled in debug builds
(`--validation`, per `main.cpp`'s existing flag).
