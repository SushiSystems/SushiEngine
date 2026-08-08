# Profiling System PROF2 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development
> (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use
> checkbox (`- [ ]`) syntax for tracking.

**Goal:** `RenderFrameStatistics` on `ISceneView` — draw calls, visible triangles, cull counts,
dynamic resolution, video-memory heap budgets — wired into the Profiler panel's Renderer and
Memory sections plus a Statistics summary line, per `docs/design/profiling_system.md` §5.

**Architecture:** CPU counters live in a `FrameStatistics` struct the scene view owns and hands
to every pass through a non-const pointer on `FrameContext` (the same shape as its `descriptors`
pointer). Visible triangles on the GPU-driven path come from a spare word in the cull shader's
statistics buffer (all four words already round-trip to a mapped readback). Heap numbers come
from `vmaGetHeapBudgets` behind a `VK_EXT_memory_budget` enable. Everything reaches the editor
through one new defaulted accessor mirroring `cull_statistics()`, copied per view per frame into
`EditorContext`.

**Tech Stack:** C++17, Vulkan + VMA, GLSL compute (`cull.comp`), Dear ImGui, `se` CLI builds.

## Global Constraints

- **The project owner runs every build and test.** Never run `se`, cmake, ninja, or a compiler.
  Steps that need a build say "ask the owner"; verify their pasted output by reading.
- **A parallel agent shares this tree.** Stage by exact path only; never `git add -A`. Leave
  unrelated modified files alone.
- C++17, Allman braces, nested namespaces written out (never `namespace A::B`), snake_case
  functions/variables, members trailing underscore, UPPER_SNAKE constants, no abbreviations in
  identifiers, acronyms upper-case. License header block on every new file (none planned).
- No separator comments, no historical comments. Doxygen on new public functions: `@brief`,
  every `@param`, `@return`.
- Commits `type(scope): lowercase imperative`; body bullets past-tense; end with
  `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`.
- Changelog bullet(s) ≤240 chars under `## [Unreleased]` in `docs/reference/changelog.md`.
- Honesty rules (`docs/design/profiling_system.md` §8): a zero from an unsupported extension or
  an unrendered view renders as "n/a", never as a measurement.
- **Semantics locked by this plan** (repeat verbatim in code docs where the fields are declared):
  `draw_calls` counts every draw command recorded this frame across all passes (shadow and depth
  included; one indirect draw counts as one). `triangles` counts main-view geometry once —
  classic path: opaque + skinned + deformable + transparent submissions; GPU-driven path: the
  cull shader's surviving-instance sum. Depth-prepass and shadow resubmissions are excluded so
  the number answers "how much geometry survived culling", not "how many times it was drawn".
- **Testing reality, recorded honestly:** the renderer is deliberately excluded from the unit
  test binary (`tests/CMakeLists.txt` ~line 675 documents this), so this plan ships no new unit
  tests; acceptance is the owner's editor run in Task 4 plus the design doc's PROF2 criteria.

---

### Task 1: Triangle count through the cull shader's spare word

**Files:**
- Modify: `engine/presentation/render/shaders/cull.comp`
- Modify: `engine/presentation/render/source/passes/cull_pass.hpp`
- Modify: `engine/presentation/render/source/passes/cull_pass.cpp`

**Interfaces:**
- Consumes: the existing stats SSBO (`cull.comp:82-85`, unsized `uint data[]`; CPU allocates 4
  words), `BucketMeta.index_count` (`cull.comp:50-56`), the mode-0 seed at `cull.comp:197-201`,
  the compaction tail at `cull.comp:245-249`, `CullPass::statistics(slot)` at
  `cull_pass.cpp:223-233`.
- Produces: `CullStatistics` gains `std::uint32_t triangles = 0;` — Task 2 reads it.

- [ ] **Step 1: Zero the spare word in the seed**

In `cull.comp`'s mode-0 thread-0 block (lines 197-201, currently `stats.data[0] = 0u;
stats.data[1] = pc.candidate_count;`), add:

```glsl
        stats.data[2] = 0u;
```

Update the SSBO's comment at line 84 from `// [0] = drawn, [1] = tested` to
`// [0] = drawn, [1] = tested, [2] = triangles`.

- [ ] **Step 2: Accumulate triangles in the compaction tail**

Beside `atomicAdd(stats.data[0], 1u);` (line 249), where `bucket` is already loaded:

```glsl
    atomicAdd(stats.data[2], bucket_block.meta[bucket].index_count / 3u);
```

Division per instance is exact — `index_count` is a whole triangle list per bucket mesh.

- [ ] **Step 3: Read the word back**

`cull_pass.hpp`: extend the struct (keep its comment style):

```cpp
            /** @brief The per-frame cull counts the editor surfaces. */
            struct CullStatistics
            {
                std::uint32_t drawn = 0;     /**< Instances that survived the cull. */
                std::uint32_t tested = 0;    /**< Instances the cull considered. */
                std::uint32_t triangles = 0; /**< Triangles of the surviving instances. */
            };
```

Update `STATS_WORDS`'s comment (`cull_pass.hpp:113`) to "drawn, tested, triangles, and one
spare." In `cull_pass.cpp:223-233`'s `statistics()`, add `out.triangles = words[2];`. The
buffer copy already moves all four words — no other change.

- [ ] **Step 4: Commit**

```bash
git add engine/presentation/render/shaders/cull.comp \
        engine/presentation/render/source/passes/cull_pass.hpp \
        engine/presentation/render/source/passes/cull_pass.cpp
git commit -m "feat(render): count surviving triangles in the cull statistics buffer (PROF2)"
```

---

### Task 2: `RenderFrameStatistics` — CPU counters and the `ISceneView` seam

**Files:**
- Modify: `engine/presentation/render/include/SushiEngine/render/scene_view.hpp`
- Modify: `engine/presentation/render/source/frame/frame_context.hpp`
- Modify: `engine/presentation/render/source/rhi/vulkan/vulkan_scene_view.hpp`
- Modify: `engine/presentation/render/source/rhi/vulkan/vulkan_scene_view.cpp`
- Modify: `engine/presentation/render/source/passes/opaque_pass.cpp`
- Modify: `engine/presentation/render/source/passes/depth_prepass.cpp`
- Modify: `engine/presentation/render/source/passes/shadow_pass.cpp`
- Modify: `engine/presentation/render/source/passes/light_shadow_pass.cpp`
- Modify: `engine/presentation/render/source/passes/transparent_pass.cpp`

**Interfaces:**
- Consumes: Task 1's `CullStatistics::triangles`; `FrameContext`'s non-const-pointer precedent
  (`descriptors`/`samplers`/`layout` fields); the draw sites the scout mapped (listed per step).
- Produces (Tasks 3-4 rely on these exact names):
  - `SushiEngine::Render::RenderFrameStatistics` in `scene_view.hpp`:
    `std::uint32_t draw_calls; std::uint64_t triangles; std::uint32_t instances_drawn;
    std::uint32_t instances_tested; std::uint32_t render_width; std::uint32_t render_height;
    std::uint64_t heap_used_bytes; std::uint64_t heap_budget_bytes;` (all zero-initialized)
  - `virtual RenderFrameStatistics render_statistics() const noexcept` on `ISceneView`,
    defaulted to return `{}` (the `cull_statistics` precedent, value-return form)
  - `Frame::FrameStatistics { std::uint32_t draw_calls = 0; std::uint64_t triangles = 0; }`
    and `FrameStatistics* statistics = nullptr;` on `FrameContext`

- [ ] **Step 1: Declare the public struct and defaulted accessor**

In `scene_view.hpp`, directly after `ScenePassTiming` (line 256), add (Doxygen carries the
Global Constraints' locked semantics verbatim for `draw_calls` and `triangles`):

```cpp
        /**
         * @brief One view's renderer counters from the last completed frame.
         *
         * draw_calls counts every draw command recorded this frame across all passes
         * (shadow and depth included; one indirect draw counts as one). triangles counts
         * main-view geometry once — classic path: opaque + skinned + deformable +
         * transparent submissions; GPU-driven path: the cull shader's surviving-instance
         * sum. Depth-prepass and shadow resubmissions are excluded so the number answers
         * "how much geometry survived culling", not "how many times it was drawn".
         * Heap numbers are whole-allocator totals, zero when VK_EXT_memory_budget is
         * absent — a reader renders that as unavailable, not as empty memory.
         */
        struct RenderFrameStatistics
        {
            std::uint32_t draw_calls = 0;       /**< Draw commands recorded, all passes. */
            std::uint64_t triangles = 0;        /**< Main-view triangles, counted once. */
            std::uint32_t instances_drawn = 0;  /**< Instances that survived the GPU cull. */
            std::uint32_t instances_tested = 0; /**< Instances the GPU cull considered. */
            std::uint32_t render_width = 0;     /**< Settled internal render width. */
            std::uint32_t render_height = 0;    /**< Settled internal render height. */
            std::uint64_t heap_used_bytes = 0;   /**< Device heap bytes in use. */
            std::uint64_t heap_budget_bytes = 0; /**< Device heap budget, 0 = unknown. */
        };
```

On `ISceneView`, directly after the `cull_statistics` defaulted accessor (line 488):

```cpp
                /**
                 * @brief The renderer counters from the last completed frame.
                 *
                 * A capability with a default, like cull_statistics above: a backend
                 * that counts nothing returns the zero-filled aggregate and the reader
                 * treats zeros from an absent producer as unavailable.
                 *
                 * @return The last frame's counters; value-initialized before any frame.
                 */
                virtual RenderFrameStatistics render_statistics() const noexcept
                {
                    return RenderFrameStatistics{};
                }
```

- [ ] **Step 2: The frame-context counter sink**

In `frame_context.hpp`, beside the struct's other declarations (before `FrameContext`), add:

```cpp
        /**
         * @brief The frame's CPU-side draw counters, accumulated by the passes.
         *
         * Reached through a pointer on the const frame context — the same shape as the
         * descriptor allocator pointer — so recording stays const-correct while the
         * counters mutate. Null when no one is counting; every increment site checks.
         */
        struct FrameStatistics
        {
            std::uint32_t draw_calls = 0; /**< Draw commands recorded, all passes. */
            std::uint64_t triangles = 0;  /**< Main-view triangles, classic path only. */
        };
```

and on `FrameContext`, beside `descriptors`: `FrameStatistics* statistics = nullptr;` with a
one-line comment pointing at the struct.

- [ ] **Step 3: Own and reset it in the view**

`vulkan_scene_view.hpp`: add a member `Frame::FrameStatistics frame_statistics_;` and an
override declaration `RenderFrameStatistics render_statistics() const noexcept override;`.
`vulkan_scene_view.cpp`, in `render()` where the local `Frame::FrameContext frame;` is
populated (line 444 area): `frame_statistics_ = {};` then `frame.statistics =
&frame_statistics_;`. The counters describe the frame being recorded; the accessor exposes
them beside the one-slot-late GPU numbers, and that half-frame skew is documented on the
override rather than hidden.

Implement the override beside `cull_statistics` (line 386):

```cpp
        RenderFrameStatistics VulkanSceneView::render_statistics() const noexcept
        {
            RenderFrameStatistics out;
            const Passes::CullStatistics cull = cull_pass_.statistics(current_slot_);
            out.instances_drawn = cull.drawn;
            out.instances_tested = cull.tested;
            // GPU-driven frames count triangles in the cull shader; classic frames count
            // them at the draw sites. The two are exclusive per frame, so summing is safe.
            out.triangles = static_cast<std::uint64_t>(cull.triangles) +
                            frame_statistics_.triangles;
            out.draw_calls = frame_statistics_.draw_calls;
            out.render_width = render_width_;
            out.render_height = render_height_;
            return out;
        }
```

(Heap fields stay zero here; Task 3 fills them.)

- [ ] **Step 4: Increment at every draw site**

Pattern, applied at each site — two statements, no helper (a lambda per pass would outweigh
the counters):

```cpp
                    if (frame.statistics != nullptr)
                        ++frame.statistics->draw_calls;
```

and, only at main-view classic-path geometry sites, additionally:

```cpp
                    if (frame.statistics != nullptr)
                        frame.statistics->triangles += mesh.index_count / 3;
```

(spell the count field as the site spells it — `mesh.index_count`, `range.index_count`).

Sites (verify each against the file; anchors are the scout's, lines may have drifted):
- `opaque_pass.cpp:511` classic `vkCmdDrawIndexed` — draw_calls + triangles.
- `opaque_pass.cpp:617-630` indirect loop — draw_calls only, once per `vkCmdDrawIndexedIndirect`
  (triangles come from the cull word).
- `opaque_pass.cpp:659-683` skinned ranges — draw_calls + triangles.
- `opaque_pass.cpp:712-728` deformable ranges — draw_calls + triangles.
- `depth_prepass.cpp:284-297` indirect and `:329` classic — draw_calls only.
- `shadow_pass.cpp:195` — draw_calls only.
- `light_shadow_pass.cpp:155` — draw_calls only.
- `transparent_pass.cpp:381` and `:413` — draw_calls + triangles.

The meshlet path (`opaque_pass.cpp:515-584`, `vkCmdDrawMeshTasksEXT`) also counts draw_calls
per emitted draw; it needs mesh-shader hardware this project's machine lacks, so count it for
completeness but note in the report that it is unexercised here.

- [ ] **Step 5: Commit**

```bash
git add engine/presentation/render/include/SushiEngine/render/scene_view.hpp \
        engine/presentation/render/source/frame/frame_context.hpp \
        engine/presentation/render/source/rhi/vulkan/vulkan_scene_view.hpp \
        engine/presentation/render/source/rhi/vulkan/vulkan_scene_view.cpp \
        engine/presentation/render/source/passes/opaque_pass.cpp \
        engine/presentation/render/source/passes/depth_prepass.cpp \
        engine/presentation/render/source/passes/shadow_pass.cpp \
        engine/presentation/render/source/passes/light_shadow_pass.cpp \
        engine/presentation/render/source/passes/transparent_pass.cpp
git commit -m "feat(render): expose per-frame renderer counters on ISceneView (PROF2)"
```

---

### Task 3: Heap budgets via `VK_EXT_memory_budget`

**Files:**
- Modify: `engine/presentation/render/source/rhi/vulkan/vulkan_device.hpp`
- Modify: `engine/presentation/render/source/rhi/vulkan/vulkan_device.cpp`
- Modify: `engine/presentation/render/source/rhi/vulkan/vulkan_scene_view.cpp`

**Interfaces:**
- Consumes: the extension-enable precedent at `vulkan_device.cpp:270`
  (`physical.enable_extension_if_present(VK_EXT_MESH_SHADER_EXTENSION_NAME)` before
  `device_builder.build()`), the allocator create at `vulkan_device.cpp:409-419`, Task 2's
  `render_statistics()` override.
- Produces: `bool VulkanDevice::supports_memory_budget() const noexcept` and filled
  `heap_used_bytes` / `heap_budget_bytes` on the accessor.

- [ ] **Step 1: Enable the extension and the allocator flag**

In `vulkan_device.cpp`, beside the mesh-shader enable (line 270 area):

```cpp
            supports_memory_budget_ =
                physical.enable_extension_if_present(VK_EXT_MEMORY_BUDGET_EXTENSION_NAME);
```

In the allocator create (line 409-419), after the buffer-device-address flag:

```cpp
            if (supports_memory_budget_)
                allocator_info.flags |= VMA_ALLOCATOR_CREATE_EXT_MEMORY_BUDGET_BIT;
```

`vulkan_device.hpp`: member `bool supports_memory_budget_ = false;` beside the other
`supports_*_` members, and beside the other accessors:

```cpp
                    /** @brief Whether VK_EXT_memory_budget backs vmaGetHeapBudgets. */
                    bool supports_memory_budget() const noexcept
                    {
                        return supports_memory_budget_;
                    }
```

- [ ] **Step 2: Query budgets in the accessor**

In `vulkan_scene_view.cpp`'s `render_statistics()` from Task 2, before `return out;` (the
view already holds the device — find its member name and spell it as the file does; the
pattern below assumes `device_`):

```cpp
            if (device_.supports_memory_budget())
            {
                VmaBudget budgets[VK_MAX_MEMORY_HEAPS] = {};
                vmaGetHeapBudgets(device_.allocator(), budgets);
                VkPhysicalDeviceMemoryProperties properties{};
                vkGetPhysicalDeviceMemoryProperties(device_.physical_device(), &properties);
                for (std::uint32_t heap = 0; heap < properties.memoryHeapCount; ++heap)
                {
                    if ((properties.memoryHeaps[heap].flags &
                         VK_MEMORY_HEAP_DEVICE_LOCAL_BIT) == 0)
                        continue;
                    out.heap_used_bytes += budgets[heap].usage;
                    out.heap_budget_bytes += budgets[heap].budget;
                }
            }
```

If `vkGetPhysicalDeviceMemoryProperties` is already cached on the device (check
`read_device_info` at `vulkan_device.cpp:421` and the `info_` struct), read the cached copy
instead of re-querying — spell it the way the device exposes it.

- [ ] **Step 3: Commit**

```bash
git add engine/presentation/render/source/rhi/vulkan/vulkan_device.hpp \
        engine/presentation/render/source/rhi/vulkan/vulkan_device.cpp \
        engine/presentation/render/source/rhi/vulkan/vulkan_scene_view.cpp
git commit -m "feat(render): query device heap budgets through VK_EXT_memory_budget (PROF2)"
```

---

### Task 4: Editor wiring — both views, Memory section, Statistics summary

**Files:**
- Modify: `applications/editor/source/ui/viewport_panel.hpp`
- Modify: `applications/editor/source/core/editor_context.hpp`
- Modify: `applications/editor/source/main.cpp`
- Modify: `applications/editor/source/ui/profiler_panel.hpp`
- Modify: `applications/editor/source/ui/profiler_panel.cpp`
- Modify: `applications/editor/source/ui/editor_panels.cpp`
- Modify: `docs/reference/changelog.md`
- Modify: `docs/design/remaining_work.md`
- Modify: `docs/design/profiling_system.md` (status line: PROF2 built)

**Interfaces:**
- Consumes: `ISceneView::render_statistics()` (Task 2/3),
  `IAssetLibrary::resident_texture_bytes()` (`asset_library_interface.hpp:181-186`, pure
  virtual, guarded `context.assets` pointer), the Profiler panel's held-copy pause pattern
  (`profiler_panel.hpp` held fields).
- Produces: `EditorContext::render_statistics` —
  `std::vector<ViewportRenderStatistics>` where
  `ViewportRenderStatistics { std::string viewport; SushiEngine::Render::RenderFrameStatistics statistics; }` —
  and `EditorContext::resident_texture_bytes` (`std::size_t`, 0 when `assets` is null).

- [ ] **Step 1: Forward on the viewport panel**

`viewport_panel.hpp`, beside the `cull_statistics` forwarder (line 376-385):

```cpp
                /**
                 * @brief The renderer counters from this view's last resolved frame.
                 * @return Zero-filled before the first frame or on a non-counting backend.
                 */
                SushiEngine::Render::RenderFrameStatistics render_statistics() const noexcept
                {
                    return view_->render_statistics();
                }
```

- [ ] **Step 2: Context fields and the per-frame copy**

`editor_context.hpp`: declare `ViewportRenderStatistics` beside `ViewportGPUStatistics`
(line 226-230) with the same copied-not-referenced doc rationale, and add beside
`gpu_statistics`:

```cpp
            // Each visible viewport's renderer counters, refilled like gpu_statistics.
            std::vector<ViewportRenderStatistics> render_statistics;

            // Bytes of texture memory resident on the device, 0 with no asset library.
            std::size_t resident_texture_bytes = 0;
```

`main.cpp`: in the existing loop that fills `context.gpu_statistics` from
`profiled_viewports` (line 1066-1094 area), also clear `context.render_statistics` and push
`{entry.panel->title(), entry.panel->render_statistics()}` for each visible viewport — one
loop, not two. After it: `context.resident_texture_bytes =
context.assets != nullptr ? context.assets->resident_texture_bytes() : 0;`.

- [ ] **Step 3: Wire the Profiler sections**

`profiler_panel.hpp/.cpp`: extend the held state with
`std::vector<ViewportRenderStatistics> held_render_statistics;` and
`std::size_t held_resident_texture_bytes = 0;`, refreshed in the existing unpaused block.
Renderer section: one sub-block per held viewport entry (title, then the table) — draw calls,
triangles, instances drawn/tested (available when `statistics.instances_tested > 0`), render
width/height (available when `statistics.render_width > 0`). Delete the old
`held_scene_cull_*` / `held_scene_render_*` fields and their reads — the per-view aggregate
replaces them. Memory section: "Video memory used (MiB)" and "budget (MiB)" from the Scene
entry's heap fields, available when `heap_budget_bytes > 0`; "Texture residency (MiB)" from
`held_resident_texture_bytes`, available when the context's `assets` was non-null — carry
that as a held bool, not by treating 0 bytes as absence (an empty library is a real zero).
Process/system rows stay "n/a" (PROF3).

- [ ] **Step 4: Statistics summary line**

`editor_panels.cpp` `draw_statistics_panel`: after the Frame/FPS block, one dimmed line from
the Scene entry of `context.render_statistics` (guard: entry present):

```cpp
            const ViewportRenderStatistics* scene_statistics = nullptr;
            for (const ViewportRenderStatistics& entry : context.render_statistics)
                if (entry.viewport == "Scene")
                    scene_statistics = &entry;
            if (scene_statistics != nullptr)
                ImGui::TextDisabled(
                    "%u draws  %llu tris  %.1f MiB textures",
                    scene_statistics->statistics.draw_calls,
                    static_cast<unsigned long long>(scene_statistics->statistics.triangles),
                    static_cast<double>(context.resident_texture_bytes) / (1024.0 * 1024.0));
```

- [ ] **Step 5: Documentation**

Changelog (`### Added`): one bullet — renderer counters (draw calls, triangles, cull counts,
heap budgets, texture residency) in the Profiler and Statistics panels. `remaining_work.md`
PROF row: remainder is PROF3 + PROF4. `profiling_system.md` line 3: PROF0–PROF2 built,
PROF3–PROF4 open.

- [ ] **Step 6: Owner build + acceptance (spec §9 PROF2 row)**

Ask the owner to run `se build`, then `se editor`, and check: draw calls match a hand count
on a simple known scene (e.g. empty scene ≈ a handful; each added entity adds prepass +
opaque + per-cascade shadow draws); triangles scale down when the camera looks away (culling);
heap-used moves visibly when a large model loads; the Memory section shows real MiB numbers
on this machine (GTX 1060 supports the extension) and texture residency is nonzero once any
model with textures is open.

- [ ] **Step 7: Commit**

```bash
git add applications/editor/source/ui/viewport_panel.hpp \
        applications/editor/source/core/editor_context.hpp \
        applications/editor/source/main.cpp \
        applications/editor/source/ui/profiler_panel.hpp \
        applications/editor/source/ui/profiler_panel.cpp \
        applications/editor/source/ui/editor_panels.cpp \
        docs/reference/changelog.md docs/design/remaining_work.md \
        docs/design/profiling_system.md
git commit -m "feat(editor): wire the renderer counters into the Profiler and Statistics panels (PROF2)"
```

---

## After this plan

PROF3 (the system metrics provider) and PROF4 (polish + architecture chapter) remain; PROF3
gets its own plan. Do not start it from this document.
