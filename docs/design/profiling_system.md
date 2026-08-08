# Profiling system — frame measurement and the editor's Profiler panel

**Status:** PROF0–PROF2 are built. Phases PROF3–PROF4 below remain open.

## 1. Purpose

The editor cannot answer the questions an optimization pass starts from. The Statistics panel's
"Frame" line is `ImGui::GetIO().Framerate` — Dear ImGui's own moving average, not an engine
measurement. No CPU-side timing infrastructure exists anywhere in the tree: no scoped timer, no
per-system breakdown, no frame-time history. Draw calls and triangles are not counted. Video
memory is measured in one place — `resident_texture_bytes()` on the asset library interface,
whose own documentation says the editor surfaces it — and displayed nowhere. Host memory and
processor or GPU utilization are not measured at all.

The render optimization program that follows this work needs before-and-after numbers for every
change it makes. This document designs the measurement layer first, so that program's acceptance
criteria are numbers a panel shows rather than impressions.

## 2. What exists today

The GPU side is in good shape and is reused, not replaced:

- `engine/presentation/render/source/graph/gpu_profiler.hpp` — one timestamp query pool per
  frame slot, written around every render-graph pass, resolved after the slot's fence wait, so
  readback never stalls. Surfaced as `pass_timing_count()` / `pass_timing()` on `ISceneView` in
  `engine/presentation/render/include/SushiEngine/render/scene_view.hpp`.
- `engine/presentation/render/source/passes/cull_pass.hpp` — a device statistics buffer with
  instances tested and drawn, read back one slot late, surfaced as `cull_statistics()` with a
  defaulted zero-returning implementation on the interface. That defaulted-accessor shape is the
  precedent every new seam in this document follows.
- `engine/domain/physics/include/SushiEngine/physics/core/statistics.hpp` — the one subsystem
  with full timing instrumentation, gated behind its profiling flag and drawn by its own panel.

The editor's data flow is a single shape: `applications/editor/source/main.cpp` copies a
snapshot of each subsystem's numbers into `EditorContext`
(`applications/editor/source/core/editor_context.hpp`) once per frame, and panels read the copy.
Panels are free functions guarded by a boolean in `PanelVisibility`, with heavier scratch state
in structs owned by `main()` (`applications/editor/source/core/panel_state.hpp`).

Measured today but displayed nowhere: texture residency bytes, terrain tile residency, and the
Game view's culling counts. Not measured at all: CPU time per system, draw calls, primitives,
video-memory heap budgets, host memory, utilization percentages, frame history.

## 3. Architecture

Four components, each behind its own boundary, joined only at the editor's existing
snapshot-into-context seam:

1. **`engine/foundation/profiling`** — a new foundation module owning CPU frame timing. It
   depends on nothing above the foundation tier; every tier above it may use it.
2. **Renderer counters** — draw calls, primitives, culling counts and memory budgets, produced
   inside `engine/presentation/render` and exposed through defaulted accessors on `ISceneView`.
3. **System metrics** — operating-system and hardware utilization, measured by an editor-owned
   provider behind an interface. The engine never links or names a vendor library.
4. **Editor surface** — the Statistics panel keeps its role as a glance, and a new Profiler
   panel owns depth: history graphs, breakdowns, memory and system sections.

Nothing here introduces a cross-cutting service locator. Each producer keeps owning its numbers;
the editor's per-frame copy remains the only join point, exactly as the documentation inside
`applications/editor/source/core/editor_context.hpp` prescribes.

## 4. `engine/foundation/profiling` — the CPU frame profiler

A small module: one class, one RAII helper, one snapshot type.

- `FrameProfiler` — owns a fixed set of named channels and a history ring. `begin_frame()`
  stamps the frame start; `end_frame()` closes the frame, computes the frame total from its own
  clock, and pushes the frame's channel durations into the ring.
- `ScopedTimer` — RAII: constructed with a profiler reference and a channel identifier, adds the
  elapsed time to that channel on destruction. Scopes may nest; each records its own inclusive
  time and its depth, and the first version presents channels as a flat list, not a tree.
- `FrameProfileSnapshot` — plain data: frame total in milliseconds, per-channel milliseconds and
  depth, and a view over the history ring. Copyable, so the editor can take it wholesale.

Decisions, with their reasons:

- **Main thread only, first version.** The editor's simulation step, render submission and UI
  build all run on the main thread, so a single-threaded profiler covers the frame the user
  sees. Physics measures itself off-thread already and arrives through its existing statistics
  path, shown beside the profiler's channels rather than inside them. Per-thread contexts are a
  later extension, not a hidden assumption — nothing in the API implies thread safety.
- **Fixed channel registration, no string hashing per scope.** Channels are registered once at
  startup and referenced by index. A scope costs two clock reads and an add.
- **A 240-frame ring** — four seconds at sixty frames per second — holding the frame total and
  every channel, sized once at registration. The ring lives in the profiler, not the panel, so a
  freshly opened panel has history.
- **The editor instruments roughly eight channels**: event pump, simulation tick, animation
  preview, scene render submission, game render submission, UI build, present wait, and asset
  work on the main thread. The exact list is settled during implementation against
  `applications/editor/source/main.cpp`'s real loop shape.
- The module carries its own `README.md` and unit tests in the functional suite: ring wraparound,
  nested scopes, channel accumulation across multiple scopes in one frame — all clock-injected so
  the tests are deterministic.

The Statistics panel's "Frame" line switches to this measurement. Dear ImGui's average survives
nowhere as a displayed number.

## 5. Renderer counters and memory

One plain-data type, `RenderFrameStatistics`, filled per view per frame and exposed as a
defaulted accessor on `ISceneView` beside `cull_statistics()`:

- **Draw calls** — counted on the CPU where the passes issue them. The GPU-driven path counts
  one per bucket; the classic path counts its real per-instance calls; shadow and depth passes
  count theirs. The counter lives in the frame context the passes already share.
- **Instances tested and drawn** — already measured by the culling pass; folded into the same
  structure and read for both the Scene and the Game view. Today only the Scene view is asked.
- **Visible triangles** — the culling shader's statistics buffer reserves four words and uses
  two. The compaction pass adds the surviving instances' index counts into a spare word with an
  atomic add; index count over three is the triangle estimate. Honest caveat carried into the
  interface documentation: this counts submitted triangles, before any clipping.
- **Dynamic resolution** — the settled render extent, already surfaced, folded in.
- **Video memory** — two numbers with different owners. Heap usage and budget come from the
  device via `VK_EXT_memory_budget` through the allocator's budget query, exposed on the same
  structure; when the extension is absent the budget reports zero and the panel says so. Texture
  residency bytes already exist on the asset library interface and are finally read.

Backends that cannot answer return zeros through the defaulted implementation, matching the
`cull_statistics()` precedent, and the panel renders a zero from a defaulted accessor as
"unavailable", never as a measurement.

## 6. System metrics — the editor-owned provider

An interface in the editor, not the engine, because only the editor displays these numbers and
the measurement is platform- and vendor-specific:

- `ISystemMetricsProvider` — `poll()` plus a snapshot: process working-set bytes, total and
  available physical memory, whole-system processor utilization, and optionally GPU utilization,
  dedicated video memory used, and GPU temperature with a flag saying whether the GPU section is
  available at all.
- `WindowsSystemMetricsProvider` — processor utilization from `GetSystemTimes` deltas, process
  memory from `GetProcessMemoryInfo`, system memory from `GlobalMemoryStatusEx`. The GPU section
  loads NVIDIA's management library (`nvml.dll`, shipped with the driver) dynamically at
  startup; when the library or a compatible device is absent, the GPU section reports
  unavailable and nothing else changes. No new package dependency, no import-library link.
- Polling is throttled inside the provider to roughly twice a second; the editor calls `poll()`
  every frame and the provider decides when to actually measure. No background thread.

A derived "GPU busy" percentage — the sum of the view's pass timings against the measured frame
time — is computed in the editor from numbers it already has, so a machine without the vendor
library still gets a defensible GPU load figure, labeled as derived.

## 7. Editor surface and data flow

Per the project's authoring workflow, the panel ships unlinked first: PROF0 builds the complete
Profiler panel against a mock snapshot filled with plausible fixed values, gets approved on
looks, and only then is wired. Each later phase replaces one mock section with real data.

- **Statistics** keeps its current sections. Its Frame line becomes the real measurement, and it
  gains one summary line — draw calls, visible triangles, video memory used — so the glance
  panel answers the first question without opening the Profiler.
- **Profiler** is a new free-function panel in its own translation unit,
  `applications/editor/source/ui/profiler_panel.cpp`, with a `ProfilerPanelState` owned by
  `main()`: pause, section collapse state, and nothing the profiler already stores. Sections:
  - **Frame** — CPU and GPU frame-time history plotted from the profiler's ring, with the
    current, average and worst-of-ring values printed beside it.
  - **CPU** — the channel table, plus the physics stage timings the physics statistics already
    carry, labeled with their off-thread origin.
  - **GPU** — the per-pass timings the Statistics panel prints today, with a percent-of-frame
    column and sorted descending by cost.
  - **Renderer** — draw calls, instances tested and drawn per view, visible triangles, dynamic
    resolution.
  - **Memory** — heap used against budget as a bar, texture residency, process working set,
    system memory.
  - **System** — processor utilization, GPU utilization, temperature; the derived GPU-busy
    figure when the vendor section is unavailable.
- Registration follows the standard three touch points: the menu item and dock home in
  `applications/editor/source/ui/editor_panels.cpp`, the visibility boolean in
  `engine/world/authoring/include/SushiEngine/authoring/panel_visibility.hpp`, and the draw call
  in `applications/editor/source/main.cpp`.
- Data flow is unchanged in shape: `main()` copies `FrameProfileSnapshot`,
  `RenderFrameStatistics` per view, and the system metrics snapshot into `EditorContext` fields
  each frame; panels read the copies and never reach into live subsystems.

## 8. Honesty rules

Every number the panels print is one of three things, and the panel distinguishes them: a
measurement, a derived figure labeled as derived, or "unavailable". A defaulted accessor's zero,
an absent extension, or a missing vendor library produce "unavailable" — never a zero rendered
as if it were measured. The audio profile's processor-load field, which is hardcoded to zero at
its source, is not displayed until something measures it.

## 9. Phases

| Phase | Scope | Acceptance |
| --- | --- | --- |
| PROF0 | The Profiler panel, complete, against a mock snapshot; the Statistics summary line, mock. | The panel is approved on appearance in the editor. No engine change. |
| PROF1 | `engine/foundation/profiling`: `FrameProfiler`, `ScopedTimer`, snapshot, ring, tests; the editor loop instrumented; Statistics' Frame line and the Profiler's Frame and CPU sections wired. | The unit tests pass; the Frame line and Dear ImGui's own average agree within noise on an idle scene; the channel sum is visibly close to the frame total. |
| PROF2 | `RenderFrameStatistics`: draw calls, both views' culling counts, visible triangles, dynamic resolution, heap budgets, texture residency; Renderer and Memory sections wired. | Draw calls match a hand count on a known scene; triangles scale with instance count under culling; heap-used moves when a large scene loads. |
| PROF3 | `ISystemMetricsProvider` and the Windows implementation; System section wired; derived GPU-busy figure. | Utilization tracks an external monitor within a few percent on this machine; unplugging the vendor path (renaming the library) degrades to "unavailable" without any other change. |
| PROF4 | Polish pass: percent-of-frame and sorting in the GPU section, pause, worst-of-ring, panel documentation in the architecture chapters. | The architecture chapter describing the editor names the panel and its seams, and every sentence in it is true. |

Phases land in order; PROF2 and PROF3 are independent of each other and may swap if review
finds a reason.

## 10. Relationship to the render optimization program

This document deliberately measures and does not optimize. The optimization program — mesh
level-of-detail, import-time mesh optimization, per-cascade shadow culling, opaque sort keys,
sky cost — is a separate design document, written after this one lands, and every phase of it
states its acceptance criterion as a number this panel shows.
