# Milestones

This file covers what has landed and what is still in flight, milestone by milestone, with a
pointer from each into the file that describes the mechanism.

## 1. Milestones

- **WP-3 — the ECS layer (done).** Archetype-chunk storage, systems scheduled by component
  access, deferred spawn/destroy via a command buffer, compiled once and replayed, validated
  against a scalar reference. This is the substrate plan's first end-to-end milestone (the
  SushiEngine side of WP-3). It uses dense per-chunk columns and whole-column resource identity;
  graduating to region-keyed sub-chunks (runtime WP-2) and device residency (WP-6) follows as
  scale grows.

- **WP-3 — the physics solver (done).** Graph-coloured Projected Gauss-Seidel over distance
  constraints (see [physics](domain-physics.md#1-the-physics-constraint-solver)), generic over
  the constraint type, validated against a scalar reference. Next constraint types (contacts,
  joints) and rigid-body state build on the same colouring and graph structure.

- **SushiLoop M2 — XPBD core (done).** The unified rigid-body XPBD solver (see
  [XPBD](domain-physics.md#11-xpbd-the-rigid-body-generalization-sushiloop-m2)), generalizing the
  PGS distance constraint to two rigid bodies' attachment points with a compliance term,
  validated against a scalar reference and against the original PGS demo's chain shape in the
  zero-inertia case. Contacts, joints, soft bodies, and cloth (M5) are further constraint types
  over the same solver. The editor's "Rigid Body" Inspector toggle is the first consumer,
  currently free-body only (no joints).

- **SushiLoop M3 — Snapshot/rollback core (done).** `Loop::RollbackBuffer` (see
  [rollback](world.md#1-sushiloop-snapshot-rollback-m3)), per-tick per-chunk byte snapshots with
  restore, proven against the milestone's key invariant (rollback-and-replay bit-identical to an
  uninterrupted run). Scoped to no structural change across a capture/restore pair and
  whole-chunk (not per-write-dirty) capture; both are follow-on work, not this milestone's. M4
  (network, reconciliation) and M3's own dirty-tracking refinement build on this without changing
  its capture/restore contract.

- **SushiLoop M4 — Network layer (done).** `Loop::Net` (see
  [the net layer](world.md#11-sushiloop-net-loopback-reconciliation-m4)): a loopback-only,
  in-process client/server command channel (`LoopbackChannel<Command>`) and server-authoritative
  reconciliation (`Net::reconcile`) built on M3's `RollbackBuffer` unchanged, plus deterministic
  entity identity (`Net::make_network_id`) so client and server agree on a spawned entity's id
  without a matching round-trip. No real sockets/threads/serialization — that is explicitly out
  of scope, same as the whole-chunk capture scoping in M3.

- **SushiLoop M5 — Soft bodies and cloth (done).** `Physics::build_cloth_grid` (see
  [cloth](domain-physics.md#12-cloth-sushiloop-m5)): a pinned-top grid of `RigidBody`s connected
  by structural and shear `XPBDDistanceConstraint`s over the existing `PhysicsWorld`, no new
  solver or constraint type. `samples/physics/cloth_demo.cpp` and `Integration_Cloth` validate it
  the same way `xpbd_demo.cpp`/`Integration_PhysicsWorld` validate the hanging chain. Volumetric
  (tetrahedral) soft bodies are out of scope.

- **Rendering (in progress).** A greenfield Vulkan 1.4 renderer behind an RHI abstraction (see
  [the render seam](presentation-render.md#1-the-render-seam)), reaching first pixels headlessly
  (`sushiengine_render_probe`) and now driving the editor window; live simulation state enters as
  the opaque sink node of that seam.

- **Editor host shell.** The editor as a host application that runs the game as a scene, with
  play/pause and inspection panels. The `sushiengine_editor` shell (SDL2 window + Dear ImGui
  presenting through the Vulkan renderer, `applications/editor/`) hosts a Unity-style panel set —
  Hierarchy (with drag-and-drop reparenting, rename, and filtering), Inspector, Project browser,
  a tabbed Text Editor, a fixed Play/Pause/Step toolbar strip, a Console, a Statistics panel, and
  a Profiler panel, the windows toggled from a domain-grouped Window menu — over the **live
  World** itself: there is no editor-side scene model, because a second copy of the truth is a
  second thing to keep in sync.

  Panels read and write through `Simulation::IWorldEditor`, and `EditorContext`
  (`applications/editor/source/core/editor_context.hpp`) carries only what is genuinely the
  editor's: the selection, the undo history, the panel set, and each panel's between-frame
  scratch (`applications/editor/source/core/panel_state.hpp`). The runtime stays behind the
  windowing, presentation, and ImGui-adapter seams of
  [the render seam](presentation-render.md#1-the-render-seam).

  Three seams feed the Statistics and Profiler panels their measurements, each owned by the
  tier that can actually take it. `engine/foundation/profiling`'s `FrameProfiler` instruments
  the editor's own loop in `applications/editor/source/main.cpp`, and each frame's snapshot is
  copied into `EditorContext` the same way every other subsystem's numbers are, so panels read
  a copy rather than a live profiler. The renderer's draw-call, culling, and GPU pass-timing
  counters reach the editor through `ISceneView`'s defaulted accessors — `render_statistics()`,
  `cull_statistics()`, and `pass_timing()` — so a backend that cannot answer returns zero rather
  than the editor reaching past the seam. Host and GPU utilization come from an editor-owned
  provider under `applications/editor/source/system/`, which loads NVIDIA's NVML library
  dynamically at run time; the engine itself never links or names a vendor.

  - **One directory per domain, one panel per translation unit.** Under
    `applications/editor/source/`: `scene/` (hierarchy, inspector, the New/Open/Save/clipboard
    commands), `render/` (the `RenderSettings` panels, lighting), `environment/` and
    `atmosphere/` (the sky and the weather simulation that drives it), `project/` (the browser
    and the documents it opens), `vfx/`, `animation/`, `audio/`, `physics/`, `scripting/`,
    `input/`, `system/` (the host metrics provider behind `ISystemMetricsProvider`).
    `ui/editor_panels.cpp` keeps only what is genuinely the shell: the menu bar, the
    toolbar strip, the Console, Statistics, the status bar, the theme, and the default dock
    layout. The vocabulary those panels share — undo-aware fields, the `Scalar`↔float round trip,
    the inline rename, the environment-edit bracket, the component-section header — lives once in
    `ui/panel_widgets.hpp`.

  - **The Inspector edits the selection, not an entity.**
    `applications/editor/source/ui/component_editor.hpp` names a component field by a
    pointer-to-member, which is what lets one mechanism read that field from every selected
    entity to decide whether they agree and write an edit back to all of them, for any component,
    without knowing which. The alternative — diffing the component's bytes before and after the
    widgets run — is exhaustive for free and unsound: nudging a float from 1.0 to 1.0000001
    changes only its low bytes, and copying those into another entity's 5.0 leaves
    5.000-something. A field is the unit of an edit, so a field is what the mechanism addresses.

    Where a widget can render indeterminacy it does (a checkbox through ImGui's mixed-value flag,
    a numeric field as a dash); where it cannot, the row is labelled `mixed` rather than quietly
    showing one entity's value as if it were all of theirs. The same class scopes to a single
    entity (`OneEntity`) for the list views — the Lighting panel's light list edits the row you
    clicked, not the Hierarchy's selection — and each component's *field list* lives with its
    domain (`draw_light_fields` in `applications/editor/source/render/lighting_panel.hpp`, the
    VFX and audio sections likewise), so a component authored in two windows cannot offer two
    different sets of fields.

- **Player host shell (PLATFORM0, S1–S6, done).** `sushiengine_player` (`applications/player/`)
  is the editor's sibling, not its subset in the ImGui sense — it links `sushiengine_platform`,
  `sushiengine_render`, `sushiengine_simulation`, `sushiengine_serialization`, and
  `sushiengine_input_backend`, and never `sushiengine_imgui`, which is the actual enforcement
  mechanism the split promises: an authoring feature that leaked out of `applications/editor/`
  into something the player needed would show up as a link error, not a lint rule someone can
  ignore.

  `PlayerApp` (`applications/player/source/player_app.{hpp,cpp}`) is a much smaller loop than
  `applications/editor/source/main.cpp` — the same `RenderScene` extract (mesh instances,
  deformables, deterministic particle billboards; lights, decals, skinned instances, and cosmetic
  emitters are already `Render::`-typed in `RenderScene` and pass straight through) with every
  editor-only concern (picking, gizmos, undo, panels, the UI overlay channel) cut, and it ends a
  frame through `present_scene_view()` (see
  [the render seam](presentation-render.md#1-the-render-seam)) rather than an ImGui-opened
  `begin_frame()`/`end_frame()` pair.

  `start()`/`frame()`/`suspend()`/`resume()`/`shutdown()` are separate calls, not a
  constructor-owned lifetime: `applications/player/source/main.cpp`'s desktop
  `while(!app.should_quit())` sits outside `PlayerApp`, which assumes no particular loop shape,
  so a future mobile host can drive `frame()` from its own OS callback instead.
  `suspend()`/`resume()` release and rebuild the swapchain-owning renderer and scene view — real
  behavior, wired to SDL's minimize/restore events on this desktop build, for whatever a host's
  drawing surface being reclaimed and given back actually looks like on its platform.

  A launch is configured by a `boot.json` beside the executable
  (`applications/player/source/boot_manifest.{hpp,cpp}`, read the same tolerant,
  missing-field-degrades-to-default way `engine/world/authoring/source/preferences.cpp`'s
  `JSONPreferencesStore` reads its own JSON) — the shape a double-click-to-play build needs,
  since there is no terminal to pass `--scene` to — with every field still overridable from the
  command line for local testing.

  `--headless --frames N` (PLATFORM0 S6) runs a fixed number of deterministic-tick frames with no
  window and exits, for CI on a runner with no display: `PlayerApp` still ticks the world and
  still renders every call (a caller can read the pixels back through
  `ISceneView::read_output()`), it just never opens a window, pumps input, or calls
  `present_scene_view()`/`end_frame()` — see the `IWindowRenderer` entry in
  [the render seam](presentation-render.md#1-the-render-seam) for the no-surface construction
  path this rides on. Reached through `se player` (`cli/sushiengine/services/player.py`, its own
  `build-player/` tree, mirroring `editor.py`'s pattern exactly).
