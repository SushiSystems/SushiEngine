# Profiling System PROF0–PROF1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development
> (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use
> checkbox (`- [ ]`) syntax for tracking.

**Goal:** The mock-first Profiler panel (PROF0) and the CPU frame profiler module wired into the
editor loop (PROF1), per `docs/design/profiling_system.md` §4 and §7.

**Architecture:** A new header+source foundation module `engine/foundation/profiling` owns a
`FrameProfiler` (fixed channels, RAII `ScopedTimer`, 240-frame history ring, injected clock for
deterministic tests). The editor instruments its main loop, copies a `FrameProfileSnapshot` into
`EditorContext` once per frame, and two panels read the copy. The Profiler panel ships complete
against mock data first and is approved on appearance before any engine change.

**Tech Stack:** C++17, Dear ImGui (docked editor), GoogleTest under `tests/unit/`, CMake via
`sushiengine_add_module`, all builds through the `se` CLI.

## Global Constraints

- **The project owner runs every build and test.** You never run `se build`, `se test`, `se
  editor`, cmake or ninja yourself. A step that needs a build says "ask the owner to run …",
  then you wait for their pasted output and verify by reading it. This is a standing rule of
  this workspace, not a preference.
- **Another agent works in this tree in parallel.** Stage files one by one, by exact path.
  `git add -A`, `git add .`, and committing files you did not write are forbidden. If a build
  error arrives from a file this plan never touched, report it and leave it alone.
- C++17 only. Allman braces everywhere, including lambdas and control blocks.
- Namespaces `PascalCase` (`SushiEngine::Profiling`, `SushiEngine::Editor`); types `PascalCase`;
  functions and variables `snake_case`; members trailing underscore (`clock_`); constants
  `UPPER_SNAKE`.
- **No abbreviations in any identifier or file name.** `Statistics` not `Stats`,
  `milliseconds` not `ms`. Well-known acronyms stay upper-case: `GPU`, `CPU`, `UI`.
- Nested namespaces written out Allman-style (`namespace SushiEngine\n{\n    namespace …`),
  never `namespace A::B`.
- Every new file starts with the Apache 2.0 header block copied verbatim from
  `applications/editor/source/core/panel_state.hpp` lines 1–22, with the first line's file name
  changed to the new file's name (keep the box width aligned).
- No separator comments (`// ----`), no historical references in comments. Comments say *why*,
  briefly. Public functions carry Doxygen per `docs/CONTRIBUTING.md` §4: `@brief` (why), one
  mechanism line only when needed, every `@param`, `@return`.
- Commits: `type(scope): lowercase imperative summary`, body bullets past-tense. End every
  commit message with `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`.
- Every user-visible change adds a bullet under `## [Unreleased]` in
  `docs/reference/changelog.md`, ≤240 characters, in the right group (`Added`/`Changed`).
- The design spec is `docs/design/profiling_system.md`. When a decision here seems to conflict
  with it, the spec wins; stop and say so rather than improvising.

---

### Task 1: PROF0 — the mock Profiler panel

The complete panel, drawn from constants plus the GPU timings the context already carries.
No engine module is touched. The task ends with the owner looking at the panel in the editor
and approving its appearance — that approval is PROF0's whole acceptance criterion, so this
task's final step is a hard stop.

**Files:**
- Create: `applications/editor/source/ui/profiler_panel.hpp`
- Create: `applications/editor/source/ui/profiler_panel.cpp`
- Modify: `engine/world/authoring/include/SushiEngine/authoring/panel_visibility.hpp` (add flag)
- Modify: `applications/editor/source/core/panel_state.hpp` (add `ProfilerPanelState`)
- Modify: `applications/editor/source/ui/editor_panels.cpp` (menu item ~line 260, dock ~line 1060)
- Modify: `applications/editor/source/main.cpp` (state instance + draw call ~line 1221)
- Modify: `applications/editor/CMakeLists.txt` (add the new source, beside line 59)
- Modify: `docs/reference/changelog.md`

**Interfaces:**
- Consumes: `EditorContext` (`context.panels.profiler`, `context.gpu_statistics`,
  `context.physics_statistics`), `ProfilerPanelState`.
- Produces: `void draw_profiler_panel(EditorContext& context, ProfilerPanelState& state)` in
  `namespace SushiEngine { namespace Editor { … } }` — Task 3 rewires its Frame and CPU
  sections onto real data without changing this signature.

- [ ] **Step 1: Add the visibility flag**

In `panel_visibility.hpp`, after `bool audio_authoring = false;` (line 100), add:

```cpp
            /** @brief The Profiler window: frame history and per-system cost breakdowns. */
            bool profiler = false;
```

- [ ] **Step 2: Add the panel state**

In `panel_state.hpp`, after `MeteorologyPanelState` (ends line 116), add:

```cpp
        /** @brief The Profiler panel's between-frame state. */
        struct ProfilerPanelState
        {
            bool paused = false; /**< Freeze the displayed numbers while comparing. */
        };
```

- [ ] **Step 3: Write the panel header**

Create `applications/editor/source/ui/profiler_panel.hpp` (license header per Global
Constraints, then):

```cpp
#pragma once

#include "../core/editor_context.hpp"
#include "../core/panel_state.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief Draws the Profiler window: frame history, CPU and GPU breakdowns,
         *        renderer counters, memory and system utilization.
         *
         * Reads only the per-frame copies on the context, like every panel. Sections
         * without a wired producer render their values as "n/a" rather than zeros, per
         * `docs/design/profiling_system.md` §8.
         *
         * @param context The shared editor state the panels read.
         * @param state   The panel's own between-frame scratch (pause).
         */
        void draw_profiler_panel(EditorContext& context, ProfilerPanelState& state);
    } // namespace Editor
} // namespace SushiEngine
```

- [ ] **Step 4: Write the panel body (mock data)**

Create `applications/editor/source/ui/profiler_panel.cpp`. Mock values are file-local
constants so Task 3's rewiring deletes them in one place. The GPU section is real from day
one — `context.gpu_statistics` already exists — and the physics rows in the CPU section are
real too (`context.physics_statistics`). Everything else is the mock.

```cpp
#include "profiler_panel.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include <imgui.h>

namespace SushiEngine
{
    namespace Editor
    {
        namespace
        {
            // PROF0's stand-in numbers, deleted when PROF1/PROF2/PROF3 wire the sections.
            // Chosen to look like a plausible busy frame so layout is judged on real shapes.
            struct MockChannel
            {
                const char* name;
                float milliseconds;
            };
            constexpr MockChannel MOCK_CPU_CHANNELS[] = {
                {"event pump", 0.21f},        {"simulation tick", 2.85f},
                {"animation preview", 0.42f}, {"scene render submit", 3.10f},
                {"game render submit", 1.95f}, {"ui build", 1.35f},
                {"present wait", 4.80f},
            };
            constexpr float MOCK_FRAME_MILLISECONDS = 14.9f;

            /** @brief Prints a value row, or a dimmed "n/a" when the producer is not wired. */
            void value_row(const char* label, bool available, const char* format, double value)
            {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted(label);
                ImGui::TableSetColumnIndex(1);
                if (available)
                    ImGui::Text(format, value);
                else
                    ImGui::TextDisabled("n/a");
            }
        } // namespace

        void draw_profiler_panel(EditorContext& context, ProfilerPanelState& state)
        {
            if (!context.panels.profiler)
                return;
            if (!ImGui::Begin("Profiler", &context.panels.profiler))
            {
                ImGui::End();
                return;
            }

            ImGui::Checkbox("Pause", &state.paused);
            ImGui::Separator();

            if (ImGui::CollapsingHeader("Frame", ImGuiTreeNodeFlags_DefaultOpen))
            {
                // Mock ring: a noisy sine so the plot's scaling and height can be judged.
                float mock_history[240];
                for (int i = 0; i < 240; ++i)
                    mock_history[i] = MOCK_FRAME_MILLISECONDS +
                                      2.0f * std::sin(static_cast<float>(i) * 0.13f);
                ImGui::PlotLines("##frame_history", mock_history, 240, 0, nullptr, 0.0f,
                                 33.3f, ImVec2(-1.0f, 60.0f));
                ImGui::Text("CPU frame: %.2f ms (%.0f FPS)", MOCK_FRAME_MILLISECONDS,
                            1000.0f / MOCK_FRAME_MILLISECONDS);
                ImGui::SameLine();
                ImGui::TextDisabled("avg %.2f  worst %.2f", MOCK_FRAME_MILLISECONDS + 0.3f,
                                    MOCK_FRAME_MILLISECONDS + 2.1f);
            }

            if (ImGui::CollapsingHeader("CPU", ImGuiTreeNodeFlags_DefaultOpen))
            {
                if (ImGui::BeginTable("cpu_channels", 3,
                                      ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp))
                {
                    ImGui::TableSetupColumn("Channel");
                    ImGui::TableSetupColumn("ms", ImGuiTableColumnFlags_WidthFixed, 70.0f);
                    ImGui::TableSetupColumn("% frame", ImGuiTableColumnFlags_WidthFixed, 70.0f);
                    ImGui::TableHeadersRow();
                    for (const MockChannel& channel : MOCK_CPU_CHANNELS)
                    {
                        ImGui::TableNextRow();
                        ImGui::TableSetColumnIndex(0);
                        ImGui::TextUnformatted(channel.name);
                        ImGui::TableSetColumnIndex(1);
                        ImGui::Text("%6.3f", channel.milliseconds);
                        ImGui::TableSetColumnIndex(2);
                        ImGui::Text("%5.1f", 100.0f * channel.milliseconds /
                                                 MOCK_FRAME_MILLISECONDS);
                    }
                    ImGui::EndTable();
                }
                // The physics stages measure themselves off-thread; shown beside the main
                // thread's channels rather than inside them, labeled with their origin.
                const SushiEngine::Physics::PhysicsStatistics& physics =
                    context.physics_statistics;
                ImGui::TextDisabled("Physics (worker thread, profiling-gated):");
                ImGui::TextDisabled("  broadphase %.3f  narrowphase %.3f  solve %.3f  total %.3f",
                                    physics.timings.broadphase_ms, physics.timings.narrowphase_ms,
                                    physics.timings.solve_ms, physics.timings.total_ms);
            }

            if (ImGui::CollapsingHeader("GPU", ImGuiTreeNodeFlags_DefaultOpen))
            {
                if (context.gpu_statistics.empty())
                    ImGui::TextDisabled("n/a — no viewport has rendered yet");
                for (const ViewportGPUStatistics& viewport : context.gpu_statistics)
                {
                    float total = 0.0f;
                    for (const GPUPassStatistic& pass : viewport.passes)
                        total += pass.milliseconds;
                    ImGui::Text("%s: %.3f ms", viewport.viewport.c_str(), total);
                    // Sorted descending by cost with a percent column, unlike Statistics'
                    // graph-order list: this window answers "what is expensive".
                    std::vector<const GPUPassStatistic*> sorted;
                    sorted.reserve(viewport.passes.size());
                    for (const GPUPassStatistic& pass : viewport.passes)
                        sorted.push_back(&pass);
                    std::sort(sorted.begin(), sorted.end(),
                              [](const GPUPassStatistic* left, const GPUPassStatistic* right)
                              {
                                  return left->milliseconds > right->milliseconds;
                              });
                    for (const GPUPassStatistic* pass : sorted)
                        ImGui::TextDisabled("  %-22s %6.3f  %5.1f%%", pass->pass.c_str(),
                                            pass->milliseconds,
                                            total > 0.0f ? 100.0f * pass->milliseconds / total
                                                         : 0.0f);
                }
            }

            if (ImGui::CollapsingHeader("Renderer"))
            {
                if (ImGui::BeginTable("renderer_counters", 2, ImGuiTableFlags_RowBg))
                {
                    value_row("Draw calls", false, "%.0f", 0.0);
                    value_row("Visible triangles", false, "%.0f", 0.0);
                    value_row("Instances drawn", true,
                              "%.0f", static_cast<double>(context.scene_cull_drawn));
                    value_row("Instances tested", true,
                              "%.0f", static_cast<double>(context.scene_cull_tested));
                    value_row("Render width", true,
                              "%.0f", static_cast<double>(context.scene_render_width));
                    value_row("Render height", true,
                              "%.0f", static_cast<double>(context.scene_render_height));
                    ImGui::EndTable();
                }
            }

            if (ImGui::CollapsingHeader("Memory"))
            {
                if (ImGui::BeginTable("memory", 2, ImGuiTableFlags_RowBg))
                {
                    value_row("Video memory used (MiB)", false, "%.1f", 0.0);
                    value_row("Video memory budget (MiB)", false, "%.1f", 0.0);
                    value_row("Texture residency (MiB)", false, "%.1f", 0.0);
                    value_row("Process working set (MiB)", false, "%.1f", 0.0);
                    value_row("System memory used (MiB)", false, "%.1f", 0.0);
                    ImGui::EndTable();
                }
            }

            if (ImGui::CollapsingHeader("System"))
            {
                if (ImGui::BeginTable("system", 2, ImGuiTableFlags_RowBg))
                {
                    value_row("CPU utilization (%)", false, "%.1f", 0.0);
                    value_row("GPU utilization (%)", false, "%.1f", 0.0);
                    value_row("GPU temperature (C)", false, "%.0f", 0.0);
                    value_row("GPU busy, derived (%)", false, "%.1f", 0.0);
                    ImGui::EndTable();
                }
            }

            ImGui::End();
        }
    } // namespace Editor
} // namespace SushiEngine
```

Before writing, open `applications/editor/source/physics/physics_statistics_panel.cpp` and
copy the exact member names it reads off `PhysicsStatistics` (the `timings.broadphase_ms`
spellings above must match that file; correct them here if they differ).

- [ ] **Step 5: Register the panel (menu, dock, draw, build)**

In `editor_panels.cpp`, inside the Analysis menu after the Statistics item (line 260):

```cpp
                    ImGui::MenuItem("Profiler", nullptr, &context.panels.profiler);
```

In the same file's `build_default_layout`, after `DockBuilderDockWindow("Statistics", bottom);`
(line 1060):

```cpp
            ImGui::DockBuilderDockWindow("Profiler", bottom);
```

In `main.cpp`: declare the state beside the other panel states (search for
`terrain_panel_state` near the top of `main`'s setup, declare
`SushiEngine::Editor::ProfilerPanelState profiler_panel_state;` beside it), and add the draw
call directly after `draw_statistics_panel(context);` (line 1221):

```cpp
            SushiEngine::Editor::draw_profiler_panel(context, profiler_panel_state);
```

Add `#include` of the new header where `editor_panels.hpp` is included in `main.cpp` if panel
headers are included individually there — mirror however `physics_statistics_panel` reaches
`main.cpp` and do the same.

In `applications/editor/CMakeLists.txt`, after `source/ui/editor_panels.cpp` (line 59):

```cmake
    source/ui/profiler_panel.cpp
```

- [ ] **Step 6: Changelog**

In `docs/reference/changelog.md` under `## [Unreleased]` / `### Added`:

```markdown
- Added the Profiler panel: frame history, CPU/GPU breakdowns, renderer, memory and system
  sections (mock-first per `docs/design/profiling_system.md` PROF0).
```

- [ ] **Step 7: Owner build + visual approval (HARD STOP)**

Ask the owner to run `se editor` (their usual build-and-run flow), open Analysis ▸ Profiler,
and say whether the layout is approved. Do not start Task 2 until they approve; rework the
panel on their feedback. This gate is PROF0's acceptance criterion in the spec.

- [ ] **Step 8: Commit**

```bash
git add applications/editor/source/ui/profiler_panel.hpp \
        applications/editor/source/ui/profiler_panel.cpp \
        applications/editor/source/ui/editor_panels.cpp \
        applications/editor/source/core/panel_state.hpp \
        applications/editor/source/main.cpp \
        applications/editor/CMakeLists.txt \
        engine/world/authoring/include/SushiEngine/authoring/panel_visibility.hpp \
        docs/reference/changelog.md
git commit -m "feat(editor): add the mock-first Profiler panel (PROF0)"
```

(Body bullets: what was added, that the GPU section reads live timings, that every unwired
value renders as n/a; plus the Co-Authored-By line.)

---

### Task 2: PROF1a — the `engine/foundation/profiling` module

`FrameProfiler`, `ScopedTimer`, `FrameProfileSnapshot`, unit-tested with an injected clock.
No editor change yet.

**Files:**
- Modify: `cmake/EngineLayers.cmake` (manifest row, beside line 20's `ecs foundation`)
- Modify: `engine/foundation/CMakeLists.txt` (`add_subdirectory(profiling)` after line 7)
- Create: `engine/foundation/profiling/CMakeLists.txt`
- Create: `engine/foundation/profiling/README.md`
- Create: `engine/foundation/profiling/include/SushiEngine/profiling/frame_profiler.hpp`
- Create: `engine/foundation/profiling/source/frame_profiler.cpp`
- Create: `tests/unit/test_frame_profiler.cpp`
- Modify: `tests/CMakeLists.txt` (source list ~line 44; link `sushiengine_profiling` where the
  test target links `sushiengine_simulation`, ~line 182)

**Interfaces:**
- Produces (Task 3 consumes exactly these):
  - `SushiEngine::Profiling::FrameProfiler{ explicit FrameProfiler(ProfilerClock clock = nullptr) }`
  - `using ChannelId = std::uint32_t;` and `using ProfilerClock = std::uint64_t (*)();`
    (the clock returns nanoseconds; `nullptr` selects `std::chrono::steady_clock`)
  - `ChannelId register_channel(const char* name)` — call before the first `begin_frame()`
  - `void begin_frame()`, `void end_frame()`
  - `void begin_scope(ChannelId channel)`, `void end_scope(ChannelId channel)`
  - `class ScopedTimer { ScopedTimer(FrameProfiler&, ChannelId); ~ScopedTimer(); }` non-copyable
  - `FrameProfileSnapshot snapshot() const` with members:
    `float frame_milliseconds`, `std::vector<ChannelValue> channels`
    (`ChannelValue { std::string name; float milliseconds; std::uint32_t depth; }`),
    `std::vector<float> frame_history` (oldest first, ≤ `HISTORY_FRAMES` = 240 entries)

- [ ] **Step 1: Declare the module to the build**

`cmake/EngineLayers.cmake`, in the manifest beside `execution foundation` (line 21):

```
    profiling       foundation
```

(match the file's exact column alignment). `engine/foundation/CMakeLists.txt`, after
`add_subdirectory(execution)`:

```cmake
add_subdirectory(profiling)
```

`engine/foundation/profiling/CMakeLists.txt`:

```cmake
# profiling — the CPU frame profiler: named channels, RAII scopes, and a fixed history
# ring, with an injectable clock so its tests are deterministic. Depends on nothing, so
# any tier may instrument itself with it.
sushiengine_add_module(NAME profiling LAYER foundation SOURCES source/frame_profiler.cpp)
```

- [ ] **Step 2: Write the module README**

`engine/foundation/profiling/README.md` — follow the shape of
`engine/foundation/core/README.md` (open it and mirror its sections). State: owns CPU frame
timing (`FrameProfiler`, `ScopedTimer`, `FrameProfileSnapshot`); tier foundation; depends on
nothing; tested by `tests/unit/test_frame_profiler.cpp`. Design record:
`../../../docs/design/profiling_system.md` (verify the relative depth from the module
directory before committing — the citation checker will).

- [ ] **Step 3: Write the failing tests**

`tests/unit/test_frame_profiler.cpp` (license header, then). Open a neighboring unit test
first (`tests/unit/test_atmosphere_quality.cpp` or similar) and mirror its include and naming
conventions; the cases to write:

```cpp
#include <gtest/gtest.h>

#include <SushiEngine/profiling/frame_profiler.hpp>

namespace
{
    // The injected clock: a file-local tick counter the tests advance by hand, so every
    // duration below is exact and the suite never sleeps.
    std::uint64_t fake_now_nanoseconds = 0;
    std::uint64_t fake_clock()
    {
        return fake_now_nanoseconds;
    }
} // namespace

TEST(FrameProfiler, AScopeAddsItsElapsedTimeToItsChannel)
{
    fake_now_nanoseconds = 0;
    SushiEngine::Profiling::FrameProfiler profiler(&fake_clock);
    const SushiEngine::Profiling::ChannelId tick = profiler.register_channel("tick");
    profiler.begin_frame();
    {
        SushiEngine::Profiling::ScopedTimer timer(profiler, tick);
        fake_now_nanoseconds += 2'000'000; // 2 ms
    }
    fake_now_nanoseconds += 1'000'000; // 1 ms outside any scope
    profiler.end_frame();
    const SushiEngine::Profiling::FrameProfileSnapshot snapshot = profiler.snapshot();
    EXPECT_FLOAT_EQ(snapshot.channels[tick].milliseconds, 2.0f);
    EXPECT_FLOAT_EQ(snapshot.frame_milliseconds, 3.0f);
}

TEST(FrameProfiler, TwoScopesOnOneChannelAccumulateWithinTheFrame)
{
    fake_now_nanoseconds = 0;
    SushiEngine::Profiling::FrameProfiler profiler(&fake_clock);
    const SushiEngine::Profiling::ChannelId tick = profiler.register_channel("tick");
    profiler.begin_frame();
    {
        SushiEngine::Profiling::ScopedTimer timer(profiler, tick);
        fake_now_nanoseconds += 1'000'000;
    }
    {
        SushiEngine::Profiling::ScopedTimer timer(profiler, tick);
        fake_now_nanoseconds += 500'000;
    }
    profiler.end_frame();
    EXPECT_FLOAT_EQ(profiler.snapshot().channels[tick].milliseconds, 1.5f);
}

TEST(FrameProfiler, ANewFrameClearsTheLastFramesChannelTimes)
{
    fake_now_nanoseconds = 0;
    SushiEngine::Profiling::FrameProfiler profiler(&fake_clock);
    const SushiEngine::Profiling::ChannelId tick = profiler.register_channel("tick");
    profiler.begin_frame();
    {
        SushiEngine::Profiling::ScopedTimer timer(profiler, tick);
        fake_now_nanoseconds += 1'000'000;
    }
    profiler.end_frame();
    profiler.begin_frame();
    profiler.end_frame();
    EXPECT_FLOAT_EQ(profiler.snapshot().channels[tick].milliseconds, 0.0f);
}

TEST(FrameProfiler, NestedScopesRecordDepthAndInclusiveTime)
{
    fake_now_nanoseconds = 0;
    SushiEngine::Profiling::FrameProfiler profiler(&fake_clock);
    const SushiEngine::Profiling::ChannelId outer = profiler.register_channel("outer");
    const SushiEngine::Profiling::ChannelId inner = profiler.register_channel("inner");
    profiler.begin_frame();
    {
        SushiEngine::Profiling::ScopedTimer outer_timer(profiler, outer);
        fake_now_nanoseconds += 1'000'000;
        {
            SushiEngine::Profiling::ScopedTimer inner_timer(profiler, inner);
            fake_now_nanoseconds += 2'000'000;
        }
    }
    profiler.end_frame();
    const SushiEngine::Profiling::FrameProfileSnapshot snapshot = profiler.snapshot();
    EXPECT_FLOAT_EQ(snapshot.channels[outer].milliseconds, 3.0f); // inclusive
    EXPECT_FLOAT_EQ(snapshot.channels[inner].milliseconds, 2.0f);
    EXPECT_EQ(snapshot.channels[outer].depth, 0u);
    EXPECT_EQ(snapshot.channels[inner].depth, 1u);
}

TEST(FrameProfiler, TheHistoryRingHoldsTheLast240FramesOldestFirst)
{
    fake_now_nanoseconds = 0;
    SushiEngine::Profiling::FrameProfiler profiler(&fake_clock);
    for (int frame = 0; frame < 300; ++frame)
    {
        profiler.begin_frame();
        fake_now_nanoseconds += 1'000'000 * static_cast<std::uint64_t>(frame + 1);
        profiler.end_frame();
    }
    const SushiEngine::Profiling::FrameProfileSnapshot snapshot = profiler.snapshot();
    ASSERT_EQ(snapshot.frame_history.size(),
              SushiEngine::Profiling::FrameProfiler::HISTORY_FRAMES);
    // Frames 61..300 survive: the oldest entry is frame 61's 61 ms total.
    EXPECT_FLOAT_EQ(snapshot.frame_history.front(), 61.0f);
    EXPECT_FLOAT_EQ(snapshot.frame_history.back(), 300.0f);
}

TEST(FrameProfiler, TheDefaultClockProducesNonNegativeTimes)
{
    SushiEngine::Profiling::FrameProfiler profiler;
    const SushiEngine::Profiling::ChannelId tick = profiler.register_channel("tick");
    profiler.begin_frame();
    {
        SushiEngine::Profiling::ScopedTimer timer(profiler, tick);
    }
    profiler.end_frame();
    EXPECT_GE(profiler.snapshot().frame_milliseconds, 0.0f);
}
```

Register the file in `tests/CMakeLists.txt`'s source list (alphabetical position among the
`unit/` entries) and add `sushiengine_profiling` to the test target's `target_link_libraries`
list beside the other `sushiengine_` modules.

- [ ] **Step 4: Write the header**

`engine/foundation/profiling/include/SushiEngine/profiling/frame_profiler.hpp` (license
header; include guard `SUSHIENGINE_PROFILING_FRAME_PROFILER_HPP`, matching
`panel_visibility.hpp`'s guard style):

```cpp
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace SushiEngine
{
    namespace Profiling
    {
        /** @brief Index of a registered channel; stable for the profiler's lifetime. */
        using ChannelId = std::uint32_t;

        /**
         * @brief The time source: a free function returning nanoseconds on a monotonic
         *        clock. Injected so tests advance time by hand; null selects the real clock.
         */
        using ProfilerClock = std::uint64_t (*)();

        /** @brief One channel's value in a completed frame's snapshot. */
        struct ChannelValue
        {
            std::string name;          /**< The name given at registration. */
            float milliseconds = 0.0f; /**< Inclusive time accumulated this frame. */
            std::uint32_t depth = 0;   /**< Nesting depth of the channel's last scope. */
        };

        /**
         * @brief One completed frame's CPU times, copied out for display.
         *
         * A copy rather than a reference, matching how the GPU timings reach the editor's
         * panels: the reader holds a frame's numbers and cannot reach back into the
         * profiler through them.
         */
        struct FrameProfileSnapshot
        {
            float frame_milliseconds = 0.0f;   /**< begin_frame() to end_frame(). */
            std::vector<ChannelValue> channels; /**< Indexed by ChannelId. */
            /** @brief Completed frames' totals, oldest first, at most HISTORY_FRAMES. */
            std::vector<float> frame_history;
        };

        /**
         * @brief Measures the main thread's frame: named channels, nested scopes, and a
         *        fixed ring of completed frames.
         *
         * Single-threaded by contract — every call comes from the thread that owns the
         * frame loop. A scope costs two clock reads and an add. Channels are registered
         * once at startup and referenced by index so the hot path never hashes a name.
         */
        class FrameProfiler
        {
            public:
                /** @brief Completed frames the history ring holds (four seconds at 60). */
                static constexpr std::size_t HISTORY_FRAMES = 240;

                /**
                 * @brief Builds an empty profiler.
                 * @param clock Nanosecond time source; null selects std::chrono::steady_clock.
                 */
                explicit FrameProfiler(ProfilerClock clock = nullptr);

                FrameProfiler(const FrameProfiler&) = delete;
                FrameProfiler& operator=(const FrameProfiler&) = delete;

                /**
                 * @brief Registers a channel before the first frame.
                 * @param name The display name; stored by copy.
                 * @return The channel's stable index.
                 */
                ChannelId register_channel(const char* name);

                /** @brief Opens a frame: stamps its start and clears every channel. */
                void begin_frame();

                /** @brief Closes the frame and pushes its totals into the history ring. */
                void end_frame();

                /**
                 * @brief Opens a scope on a channel; ScopedTimer is the intended caller.
                 * @param channel A value register_channel returned.
                 */
                void begin_scope(ChannelId channel);

                /**
                 * @brief Closes the innermost open scope on a channel and adds its time.
                 * @param channel The same value the matching begin_scope was given.
                 */
                void end_scope(ChannelId channel);

                /**
                 * @brief Copies the last completed frame and the history out for display.
                 * @return The snapshot; empty channels and zero totals before any frame.
                 */
                FrameProfileSnapshot snapshot() const;

            private:
                struct Channel
                {
                    std::string name;
                    std::uint64_t nanoseconds = 0;
                    std::uint64_t scope_start = 0;
                    std::uint32_t depth = 0;
                };

                std::uint64_t now() const;

                ProfilerClock clock_;
                std::vector<Channel> channels_;
                std::uint64_t frame_start_ = 0;
                std::uint64_t frame_nanoseconds_ = 0;
                std::uint32_t open_scopes_ = 0;
                std::vector<float> history_;
                std::size_t history_next_ = 0;
                std::size_t history_count_ = 0;
        };

        /**
         * @brief RAII scope: adds the elapsed time to its channel on destruction.
         *
         * Constructed on the stack around the code being measured, so an early return
         * cannot leave a scope open.
         */
        class ScopedTimer
        {
            public:
                /**
                 * @brief Opens a scope on the channel.
                 * @param profiler The profiler owning the channel.
                 * @param channel  A value register_channel returned.
                 */
                ScopedTimer(FrameProfiler& profiler, ChannelId channel);
                ~ScopedTimer();

                ScopedTimer(const ScopedTimer&) = delete;
                ScopedTimer& operator=(const ScopedTimer&) = delete;

            private:
                FrameProfiler& profiler_;
                ChannelId channel_;
        };
    } // namespace Profiling
} // namespace SushiEngine
```

- [ ] **Step 5: Write the implementation**

`engine/foundation/profiling/source/frame_profiler.cpp`:

```cpp
#include <SushiEngine/profiling/frame_profiler.hpp>

#include <chrono>

namespace SushiEngine
{
    namespace Profiling
    {
        namespace
        {
            std::uint64_t steady_clock_nanoseconds()
            {
                return static_cast<std::uint64_t>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::steady_clock::now().time_since_epoch())
                        .count());
            }

            float to_milliseconds(std::uint64_t nanoseconds)
            {
                return static_cast<float>(nanoseconds) * 1.0e-6f;
            }
        } // namespace

        FrameProfiler::FrameProfiler(ProfilerClock clock)
            : clock_(clock != nullptr ? clock : &steady_clock_nanoseconds)
        {
            history_.resize(HISTORY_FRAMES, 0.0f);
        }

        std::uint64_t FrameProfiler::now() const
        {
            return clock_();
        }

        ChannelId FrameProfiler::register_channel(const char* name)
        {
            Channel channel;
            channel.name = name;
            channels_.push_back(channel);
            return static_cast<ChannelId>(channels_.size() - 1);
        }

        void FrameProfiler::begin_frame()
        {
            frame_start_ = now();
            open_scopes_ = 0;
            for (Channel& channel : channels_)
                channel.nanoseconds = 0;
        }

        void FrameProfiler::end_frame()
        {
            frame_nanoseconds_ = now() - frame_start_;
            history_[history_next_] = to_milliseconds(frame_nanoseconds_);
            history_next_ = (history_next_ + 1) % HISTORY_FRAMES;
            if (history_count_ < HISTORY_FRAMES)
                ++history_count_;
        }

        void FrameProfiler::begin_scope(ChannelId channel)
        {
            channels_[channel].scope_start = now();
            channels_[channel].depth = open_scopes_;
            ++open_scopes_;
        }

        void FrameProfiler::end_scope(ChannelId channel)
        {
            channels_[channel].nanoseconds += now() - channels_[channel].scope_start;
            --open_scopes_;
        }

        FrameProfileSnapshot FrameProfiler::snapshot() const
        {
            FrameProfileSnapshot snapshot;
            snapshot.frame_milliseconds = to_milliseconds(frame_nanoseconds_);
            snapshot.channels.reserve(channels_.size());
            for (const Channel& channel : channels_)
            {
                ChannelValue value;
                value.name = channel.name;
                value.milliseconds = to_milliseconds(channel.nanoseconds);
                value.depth = channel.depth;
                snapshot.channels.push_back(value);
            }
            snapshot.frame_history.reserve(history_count_);
            for (std::size_t i = 0; i < history_count_; ++i)
            {
                const std::size_t oldest =
                    (history_next_ + HISTORY_FRAMES - history_count_) % HISTORY_FRAMES;
                snapshot.frame_history.push_back(history_[(oldest + i) % HISTORY_FRAMES]);
            }
            return snapshot;
        }

        ScopedTimer::ScopedTimer(FrameProfiler& profiler, ChannelId channel)
            : profiler_(profiler), channel_(channel)
        {
            profiler_.begin_scope(channel_);
        }

        ScopedTimer::~ScopedTimer()
        {
            profiler_.end_scope(channel_);
        }
    } // namespace Profiling
} // namespace SushiEngine
```

- [ ] **Step 6: Owner runs the tests**

Ask the owner to run `se build` and then `se test --suite functional` (or the narrower CTest
label they prefer for unit tests). Expected: the six `FrameProfiler.*` cases listed, all
passing, nothing else newly failing. Read their pasted output; if a case fails, fix and ask
for a re-run before continuing.

- [ ] **Step 7: Commit**

```bash
git add cmake/EngineLayers.cmake engine/foundation/CMakeLists.txt \
        engine/foundation/profiling/CMakeLists.txt engine/foundation/profiling/README.md \
        engine/foundation/profiling/include/SushiEngine/profiling/frame_profiler.hpp \
        engine/foundation/profiling/source/frame_profiler.cpp \
        tests/unit/test_frame_profiler.cpp tests/CMakeLists.txt
git commit -m "feat(profiling): add the CPU frame profiler foundation module (PROF1)"
```

---

### Task 3: PROF1b — instrument the editor loop and wire the panels

**Files:**
- Modify: `applications/editor/source/main.cpp` (profiler instance, channels, scopes,
  snapshot copy)
- Modify: `applications/editor/source/core/editor_context.hpp` (snapshot field)
- Modify: `applications/editor/source/ui/profiler_panel.cpp` (Frame + CPU sections real)
- Modify: `applications/editor/source/ui/editor_panels.cpp` (Statistics Frame line real)
- Modify: `applications/editor/CMakeLists.txt` only if the editor target does not already link
  `sushiengine_profiling` transitively — check how it links foundation modules and add it the
  same way if needed
- Modify: `docs/reference/changelog.md`, `docs/design/remaining_work.md`

**Interfaces:**
- Consumes: everything Task 2's Interfaces block lists, verbatim.
- Produces: `EditorContext::frame_profile` of type
  `SushiEngine::Profiling::FrameProfileSnapshot` — the field name later phases read.

- [ ] **Step 1: Add the context field**

In `editor_context.hpp`, beside `gpu_statistics` (line 642), add (and include
`<SushiEngine/profiling/frame_profiler.hpp>` with the other engine includes):

```cpp
            // The last completed frame's CPU times, snapshotted per frame like the GPU
            // timings above: the panels read a copy and cannot reach the live profiler.
            SushiEngine::Profiling::FrameProfileSnapshot frame_profile;
```

- [ ] **Step 2: Instrument the loop**

In `main.cpp`, before the `while (running)` loop (beside `last_frame_time`, line 425):

```cpp
        SushiEngine::Profiling::FrameProfiler frame_profiler;
        const SushiEngine::Profiling::ChannelId profile_event_pump =
            frame_profiler.register_channel("event pump");
        const SushiEngine::Profiling::ChannelId profile_simulation_tick =
            frame_profiler.register_channel("simulation tick");
        const SushiEngine::Profiling::ChannelId profile_animation_preview =
            frame_profiler.register_channel("animation preview");
        const SushiEngine::Profiling::ChannelId profile_scene_render =
            frame_profiler.register_channel("scene render submit");
        const SushiEngine::Profiling::ChannelId profile_game_render =
            frame_profiler.register_channel("game render submit");
        const SushiEngine::Profiling::ChannelId profile_ui_build =
            frame_profiler.register_channel("ui build");
```

Then place the scopes. Anchors in today's `main.cpp` (verify each against the file as it is
when you edit — the parallel session may have shifted lines):

1. `frame_profiler.begin_frame();` immediately after `last_frame_time = frame_time;`
   (line 434).
2. Wrap the preview updates (lines 438–441) in a block with
   `SushiEngine::Profiling::ScopedTimer timer(frame_profiler, profile_animation_preview);`.
3. Wrap `if (!window.pump_events()) …` (line 446) the same way with `profile_event_pump`.
4. Wrap the `simulation->tick(…)` if/else pair (lines 484–487) with
   `profile_simulation_tick`.
5. Wrap the Scene-view block — from `if (context.panels.scene_view)` down past
   `scene_view.cull_statistics(…)` (line 992) — with `profile_scene_render`.
6. Wrap the Game-view block (lines 997–1044) with `profile_game_render`.
7. Wrap the panel-draw run (lines 1205–1238, `draw_hierarchy_panel` through
   `draw_scene_action_confirm_modal`) with `profile_ui_build`.
8. At the very bottom of the loop body, after the last statement that belongs to the frame
   (find where the frame ends — the ImGui render/present call — and place these after it,
   still inside the `while`):

```cpp
            frame_profiler.end_frame();
            context.frame_profile = frame_profiler.snapshot();
```

Each wrap is a new brace block only where one does not already exist; where the code is
already inside braces (the Scene-view `if`), declare the timer as the block's first
statement instead of adding braces.

- [ ] **Step 3: Wire the panel's Frame and CPU sections**

In `profiler_panel.cpp`: delete `MOCK_CPU_CHANNELS` and `MOCK_FRAME_MILLISECONDS`; the Frame
section plots `context.frame_profile.frame_history` (guard: if it is empty, draw the
disabled text "n/a — first frame pending"); the header numbers become
`context.frame_profile.frame_milliseconds`, its average over `frame_history`, and the ring's
maximum. The CPU table iterates `context.frame_profile.channels`, indenting each name by
`depth` (prefix two spaces per depth level), with the percent column against
`frame_milliseconds`. Honor `state.paused`: when paused, skip copying into a small
`FrameProfileSnapshot` member added to `ProfilerPanelState`
(`SushiEngine::Profiling::FrameProfileSnapshot held;` plus `#include` — panel_state.hpp may
include the foundation header, it is dependency-free) and render from `state.held` instead;
when not paused, refresh `state.held = context.frame_profile;` each frame and render from it.

- [ ] **Step 4: Make the Statistics Frame line real**

In `editor_panels.cpp` `draw_statistics_panel` (lines 694–696), replace the `io.Framerate`
lines with:

```cpp
            if (context.frame_profile.frame_milliseconds > 0.0f)
            {
                ImGui::Text("Frame: %.2f ms", context.frame_profile.frame_milliseconds);
                ImGui::Text("FPS:   %.0f", 1000.0f / context.frame_profile.frame_milliseconds);
            }
            else
            {
                ImGui::TextDisabled("Frame: n/a — first frame pending");
            }
```

Remove the now-unused `const ImGuiIO& io` line only if nothing else in the function reads
`io`. Check the status bar (`draw_status_bar`, line 809 area) — the spec retires ImGui's
average everywhere it is *displayed*, so switch that read to `context.frame_profile` the same
way.

- [ ] **Step 5: Documentation**

Changelog, under `### Changed`:

```markdown
- Changed the Statistics panel's Frame/FPS line and the status bar to the engine's own frame
  measurement; the Profiler panel's Frame and CPU sections now show live per-channel times.
```

In `docs/design/remaining_work.md`, reword the PROF row's "What it still needs" to name only
what remains (renderer counters and memory behind `ISceneView` accessors, the system metrics
provider, the polish pass) — PROF0 and PROF1 no longer belong in it.

- [ ] **Step 6: Owner build + acceptance run**

Ask the owner to run `se build`, `se test --suite functional`, then `se editor`, and to
report: (a) the Statistics Frame value beside the old FPS feel — spec acceptance is
"agree within noise on an idle scene"; (b) whether the Profiler CPU channel sum is visibly
close to the frame total; (c) a screenshot if anything looks off. Fix on feedback.

- [ ] **Step 7: Commit**

```bash
git add applications/editor/source/main.cpp \
        applications/editor/source/core/editor_context.hpp \
        applications/editor/source/core/panel_state.hpp \
        applications/editor/source/ui/profiler_panel.cpp \
        applications/editor/source/ui/editor_panels.cpp \
        docs/reference/changelog.md docs/design/remaining_work.md
git commit -m "feat(editor): instrument the frame loop and wire the Profiler panel (PROF1)"
```

(Add `applications/editor/CMakeLists.txt` to the add list only if Step 1's link check changed
it.)

---

## After this plan

PROF2 (renderer counters and memory budgets) and PROF3 (the system metrics provider) get their
own plans once PROF0's approved panel fixes the display contract. Do not start them from this
document.
