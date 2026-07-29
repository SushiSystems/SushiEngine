# Editor UX Overhaul — from a pile of panels to a Unity-class editor

Status: **Audit + engineering plan.** Companion to
[editor_feature_sync_gaps.md](editor_feature_sync_gaps.md) — this document absorbs and
supersedes that file's two "Deferred" sections (§2.4 here carries every item forward with
a verdict), while its "Fixed this pass" log remains historical record.

The audit behind this plan was a line-level sweep of `editor/` (four independent passes:
window/layout inventory, settings/domain coupling, dead-control detection with
consumer-side verification, and architecture/data-flow), cross-checked against the
serializers, `render/frame/quality.cpp`, and the engine headers. Every claim below
carries a `file:line` reference into the tree as of branch `atmosphere/phase-b1`.

Three decisions were taken up front with the project owner:

1. **Unity is the reference paradigm.** Left: Hierarchy; center: Scene/Game tabs;
   right: Inspector; bottom: Project + Console. The editor's window names
   (Hierarchy, Inspector, Project, Console, Game) already speak Unity's vocabulary,
   so this is the lowest-friction target.
2. **Quality tiers become per-domain, with an overall preset on top.** Each domain
   (Rendering, Atmosphere/Meteorology, …) owns an independent tier; an optional
   "Overall" preset sets them all in one gesture and reads "Custom" when they diverge.
3. **All four workstreams are in scope**: docking/layout, settings domain separation,
   wire-or-remove for unbound controls, and visual identity/polish. Report first,
   implementation after sign-off.

---

## 0. Executive summary — the findings that matter most

| # | Finding | Where | Severity |
|---|---|---|---|
| 1 | **Undo, Save Scene, and Play→Stop silently destroy every light, decal, and material** — `capture_scene`/`apply_scene` never serialize them, and all three paths round-trip through that serializer | `editor/serialization/scene_serializer.cpp:620-800`, `editor/core/command_history.cpp:47,73,86`, `editor/ui/editor_panels.cpp:5289-5299` | Critical — data loss |
| 2 | **Rendering "Quality" rebuilds the meteorology simulation** — the Ultra branch writes the atmosphere nest grid (256×256×64 @ 1.5 km), the renderer detects the size change and destroys the running weather via `vkDeviceWaitIdle` | `render/frame/quality.cpp:259`, `render/rhi/vulkan/vulkan_scene_view.cpp:456-458`, `render/material/asset_library.cpp:186-196` | Critical — the reported SOLID violation, mechanism confirmed |
| 3 | **13 of 22 panels have no dock slot and no default position** — they open floating at ImGui's cascade origin; `Environment` (default-open, ~1000 lines of controls) lands on top of the fresh layout on first run | `editor/ui/editor_panels.cpp:6586-6614` | High — the reported window-pile symptom |
| 4 | **Panel open/closed state is not persisted, layout has no reset, and `imgui.ini` is CWD-relative** — layout silently varies by launch directory, and 12 default-closed panels cost a menu click every launch | `editor/core/editor_context.hpp:66-98`, no `io.IniFilename` assignment anywhere | High |
| 5 | **The scene file's environment block is written but never read** — `open_scene` loads it, then immediately overwrites it with `preferences.environment`; the ~35 authored meteorology constants survive **neither** the scene path nor the preferences path | `editor_panels.cpp:717-720`, `core/preferences.cpp:433-486` | High — authored work evaporates |
| 6 | **Hard-dead controls ship in the UI**: the Upscaler combo (FSR/DLSS/XeSS) is never read by the renderer, Autosave has no implementation, GPU-culling Freeze/Statistics have zero consumers, the Physics Timings section is permanently unreachable | §1.4 | High — trust |
| 7 | **Two whole panels are structurally disconnected**: Audio Authoring is not in the build and has no caller; Animator Graph authors a controller that no entity, scene, or preview ever consumes | `editor/CMakeLists.txt:56-76`, `animation/animator_graph_panel.cpp:87-91` | High |
| 8 | **Duplicated settings with divergent ranges**: sun edited in two panels, shadows edited in two panels with different legal ranges for the same field, exposure in two panels, time-of-day in three places | §1.3 | Medium |
| 9 | **`editor_panels.cpp` is a 6,616-line monolith**; ~3,400 lines extract cleanly along seams the codebase already uses (`animation/`, `audio/`, `physics/` directories exist) | §1.5 | Medium — velocity |
| 10 | **Interaction parity gaps vs Unity**: no Duplicate (Ctrl+D), no Delete key, no drag-drop outside the Hierarchy, no Open Scene menu item, no Recent Scenes, console has no severity/filter, Inspector is single-object only | §1.5 | Medium |

---

## 1. What the audit found

### 1.1 Window inventory and layout

**The inventory.** 22 dockable panels tracked by `PanelVisibility`
(`editor/core/editor_context.hpp:66-98`), plus `Preferences` and `Input Manager` on
loose bools (`editor_context.hpp:389-390`), the ImGui demo (`:408`), the dock host, and
the status bar — **27 possible top-level ImGui windows**. The struct's doc comment
("Defaults to everything visible on a fresh layout", `editor_context.hpp:64`) is false:
10 of 22 default open, 12 closed.

| Domain | Windows (default state) |
|---|---|
| Scene authoring | Scene (open), Hierarchy (open), Inspector (open), Toolbar (open), Project (open) |
| Play | Game (open) |
| Rendering | Rendering, Lighting, Post Process, GPU Culling (all closed) |
| World / weather | Environment (**open**), Meteorology (closed) |
| Animation | Animation, Animator Graph, Animator (all closed) + shared Preview (closed) |
| Audio | Audio Mixer, Audio Profiler (closed) [+ dead Audio Authoring] |
| Physics | Physics (closed) |
| Dev tools | Console (open), Statistics (open), Text Editor (open), Preferences, Input Manager, ImGui Demo |

**The layout facts:**

- A dockspace exists (`main.cpp:110-137`, docking enabled at `imgui_backend.cpp:76`;
  multi-viewport tear-out is **not** enabled). `build_default_layout`
  (`editor_panels.cpp:6586-6614`) runs once when no ini node exists and docks **only 9
  windows**: Toolbar (top), Scene+Game (center tabs), Hierarchy (left),
  Inspector+Statistics (right tabs), Project+Text Editor+Console (bottom tabs).
- The other **13 panels have no dock slot, no `SetNextWindowPos`, no size hint** (the
  only `SetNextWindowSize(FirstUseEver)` calls in the tree are Preferences `:6345` and
  Input Manager `:6480`). They open floating at the default cascade origin, overlapping
  the Scene view — this is the "windows without a parent, stacked" symptom.
- **`Environment` is default-open but undocked** (`editor_context.hpp:80`): the very
  first frame of a fresh install shows a ~1000-line floating settings window on top of
  the new layout.
- **`Preview` is a 3D viewport that floats** at ImGui's tiny auto-size
  (`main.cpp:796-810`) — worst case for a render-target panel.
- **`Console` is tab-hidden behind `Project`** by default, so log output is invisible
  on a fresh layout. If a user drags the 13 floating panels into the bottom node it
  reaches 16 tabs; no tab-bar fitting policy is configured anywhere — this is the
  "tab bar about to overflow" symptom.
- **`io.IniFilename` is never set** — the layout file is CWD-relative (`.gitignore:6`
  confirms it lands in the repo root). Launching from another directory silently loads
  a different layout. Preferences, by contrast, are correctly path-locked
  (`preferences.cpp:695` → `user_config_dir()/preferences.json`).
- **No reset-layout feature, no explicit save/load.** A wedged layout has no in-app
  recovery.
- **Panel visibility is not persisted** (no `PanelVisibility` field in
  `preferences.cpp`) while dock geometry *is* (ImGui ini) — asymmetric: the editor
  remembers *where* Lighting was docked but forgets *that it was open*.
- **First-frame sizing bug**: `draw_dockspace()` runs before the menu bar and status
  bar exist (`main.cpp:442` vs `:466-467`), so the first-run layout bakes a node size
  too tall by both bars' heights into the persisted ini.
- **Two independent fullscreen state machines fight over the `Game` window**:
  `ViewportPanel` members (`viewport_panel.cpp:392-413`) vs function-local statics in
  `draw_no_camera_game_view` (`game_view_toolbar.hpp:109-132`). Toggling fullscreen
  while camera availability flips across `main.cpp:761` can restore the wrong dock id.
- **The Window menu is one flat 22-item list** (`editor_panels.cpp:850-877`), ungrouped
  and unordered (`Preview` sits between `Physics` and `Project`). Closing `Toolbar`
  removes all mouse access to Play/Pause/Step — which have no keyboard bindings
  (`editor/input/editor_contexts.hpp:58-81`).

### 1.2 The quality tier crosses domains — the "Ultra" mechanism

The single `RenderQuality` enum (`include/SushiEngine/render/render_settings.hpp:50`)
feeds `resolve_quality()` (`render/frame/quality.cpp:62-279`), whose Ultra branch
writes, in one gesture: shadow resolution/taps, light and decal budgets, GTAO
slices/steps, cloud march steps, VRS coarseness, material lobes, probe GI, bloom/DoF/
motion blur, meshlets, GPU particles — **and the meteorology nest grid**:

- `quality.cpp:259` sets `params.atmosphere_nest = {256, 256, 64, 1500 m, 18 km}`
  (High is 192×192×48 @ 2 km, `quality.cpp:181`).
- `vulkan_scene_view.cpp:456-458` stages the resolved size each frame;
  `asset_library.cpp:186-196` compares it to the live nest and, on mismatch, calls
  `vkDeviceWaitIdle`, **destroys the nest, and rebuilds it — losing all running
  weather** (conceded at `quality_params.hpp:326-328`).
- The physics is not tier-invariant: cell size crosses the 2 km convection-resolving
  threshold between tiers, and `docs/slop/atmosphere_system.md:476-484` records that
  the same configuration leaves High with 5.8 % cloudy columns and the other tiers
  between 0 and 0.1 %. **A rendering dropdown changes the meteorological result.**
- The combo itself (`editor_panels.cpp:2598-2601`) carries no tooltip and no warning;
  the weather-restart warning lives in a different panel (`:3029-3036`).

Aggravating findings around the same seam:

- **Dead tier outputs**: `params.max_particles`, `particle_sim_substeps`
  (`quality.cpp:254-255`) and the entire animation budget block
  (`max_skinned_instances`, `bone_lod_bias`, `animation_influences`,
  `quality.cpp:264-266`) have **zero consumers** — `batch_evaluator.hpp:51` claims
  the resolver fills them; nothing reads them.
- **Contradictory documented contracts**: `weather_provider.hpp:566-578` says the QG
  global grid is *deliberately not* tier-resolved; `quasigeostrophic_core.hpp:104-108`
  says it *is* chosen by the tier. One of the two is wrong today.
- **A raw-enum leak**: `ray_traced_shadow_pass.cpp:138` branches on
  `settings.quality == RenderQuality::Ultra` directly, violating the resolver contract
  stated at `quality_params.hpp:32-34`.
- **A latent multi-view hazard**: `RenderSettings` is per-viewport by design
  (`viewport_panel.hpp:211-217`) but the nest is device-global and "last view wins"
  (`asset_library.hpp:124-131`) — if two views ever diverge in tier, they rebuild the
  nest (with a device stall) every frame.
- **No test pins any of this** — `grep resolve_quality tests/` is empty.

The structural root: **there is no simulation-side settings container at all.** The
only struct with a tier concept is `RenderSettings`, so any parameter that wants a
tier gets pulled into it regardless of domain. Nest physics rides inside the
render-owned `Environment`, the QG grid is a hard-coded static, and the sim clock
lives as loose doubles on `EditorContext`.

### 1.3 Settings ownership is scrambled

**Same value, multiple editors:**

| State | Editors | Who wins |
|---|---|---|
| Sun elevation/azimuth/color/intensity | Lighting `:3804-3835` and Environment `:4000-4034` — near-verbatim duplicate widget blocks, different labels | Last drawn per frame (same fields, so correct by accident) |
| Shadow enable/cascades/distance/contact | Rendering `:2630-2691` (12 controls) and Lighting `:3857-3869` (4-control subset) — **the two panels disagree on the legal range of `distance` (20–4000 vs 50–2000)** | Last drawn (Lighting draws later) |
| Exposure | Environment `:4041` (`environment.exposure`) and Post Process `:3602-3629` (`render_settings.post`) — two knobs, no cross-reference | Both live, compounding |
| Time-of-day / clock | Environment observer `:4255-4263`, Environment Solar System `:4934-4976`, and Meteorology's "Match sky to atmosphere" `:3105-3109` (which re-times the whole solar system from a diagnostics panel) | Three controls, two state owners |
| Procedural weather enable | Environment radios `:4195-4205` and a Meteorology button `:3205` | World state (this one is coherent) |
| Quality-tier readout | Triplicated "Tier resolves to" tree in Rendering `:2774`, Post Process `:3697`, GPU Culling `:3763` | — |

**Persistence, who actually wins:**

- `Environment` exists in three copies: the world's live copy, `Preferences::environment`,
  and the `.sushiscene` block. **Preferences wins absolutely**: `open_scene`
  (`editor_panels.cpp:711-720`) applies the scene's environment via `load_scene`
  (`scene_serializer.cpp:1122`) and then immediately overwrites it with
  `context.preferences.environment`. Every scene-authored environment value is written
  to disk on save and discarded on load.
- Meanwhile `environment_to_json` (`preferences.cpp:433-486`) persists only
  `sun, atmosphere, clouds, surface, stars, night, ambient, exposure, ibl` — **no fog,
  no fog volumes, no GI, no atmosphere nest, no observer**. Net effect: fog, fog
  volumes, GI, and all ~35 nest physics constants survive **neither** path — lost on
  restart *and* on scene reload. `atmosphere_nest.hpp:33-35` documents the exact
  opposite ("serialized with the scene, and editable").
- The Environment panel's own comment (`:4987-4990`, "they persist through the
  preferences store") is therefore false for the fields users most tune.
- One button mutates environment state *without* updating the preferences mirror —
  Meteorology's "Switch the nest on" (`:3213-3218`) — so it is silently reverted on
  the next scene open. Every other environment write updates the mirror
  (`:3851`, `:4987`).
- **Lifetime hazard**: `Environment` carries borrowed pointers
  (`AtmosphereForcing::samples`, `WeatherField`), and both panels copy the whole live
  struct into long-lived `Preferences::environment` — which is re-installed into the
  world at startup (`main.cpp:275`) and on every scene open (`:720`), carrying a stale
  non-null pointer that `AtmosphereForcing::valid()` (`atmosphere_nest.hpp:1179`)
  cannot distinguish from a live one.
- Weather→render implicit writes: the sim writes `Environment::weather.fog_density_bias`
  / `ground_wetness` (`weather_world_coupling.hpp:97,113`), consumed by
  `volumetric_fog_pass.cpp:286-297` and `pbr.frag` — and `volumetric_fog_pass.cpp:296`
  gates the result on the render tier, so **the Low tier suppresses fog the simulation
  says is there**.

**Wrong-panel ownership:** the entire regional-nest physics (~35 meteorological
constants: surface energy balance, boundary layer, microphysics, ice, dynamics —
`editor_panels.cpp:4559-4890`) lives in the **Environment** panel, while the
**Meteorology** panel (`:2976-3581`) authors nothing — it is read-only diagnostics
plus three buttons that mutate other panels' state plus a CSV writer
(`std::ofstream` inside a UI draw function, `:3510-3578`). The nest's *grid
resolution* meanwhile lives in the **Rendering** panel, as the tier.

### 1.4 Controls that do not tell the truth

Verification method: for every bound variable, the consuming side was grepped across
`sim/`, `render/`, `include/SushiEngine/`, and the shader tree; serializer coverage was
checked in both `preferences.cpp` and `scene_serializer.cpp`. Only zero-consumer /
provably-omitted findings are listed.

**Hard-dead (zero consumers):**

| Control | Site | Evidence |
|---|---|---|
| "Autosave" checkbox | Preferences, `editor_panels.cpp:6369` | `preferences.autosave` written, serialized (`preferences.cpp:633,664`), read by nothing — no timer, no save call keyed to it |
| "Upscaler" combo (None / Temporal / FSR 3.1 / DLSS / XeSS) | Rendering ▸ Frame Delivery, `:2856` | `settings.upscale` has no renderer-side reader; the upscale path is decided solely by `anti_aliasing` (`frame_context.hpp:374`, `taa_pass.cpp:79,100`). Selecting None does not disable upscaling; FSR/DLSS/XeSS change nothing. The "Runs:" label (`:2861-2869`) is self-referential — `upscaler_availability()`/`resolve_upscale_mode()` (`upscaler.cpp:31,69`) have no callers outside these editor lines |
| "Freeze frustum (debug)" | GPU Culling, `:3751` | `cull.freeze` never uploaded — `cull_pass.cpp:271-273` packs only frustum/occlusion/min-diameter |
| "Show statistics" | GPU Culling, `:3752` | `cull.show_statistics` unconsumed; its help text points at a "Profiler HUD" **that does not exist as any window** — the timings actually surface in Statistics (`:6086-6088`) |
| "Receive Shadows" / "GPU Instancing" | Material Inspector (`material_inspector.cpp`) | Carried from `editor_feature_sync_gaps.md`: no pass reads `Material::receive_shadows` / `gpu_instancing` |

**Structurally disconnected:**

- **Audio Authoring panel** (`audio/audio_authoring_panel.cpp:112`): not in
  `editor/CMakeLists.txt:56-76`, no caller, no `PanelVisibility` flag, no menu entry.
  Dead on two independent counts.
- **Animator Graph panel** (`animator_graph_panel.cpp:881`): authors a
  `ControllerDesc` in a file-static singleton (`:87-91`) that no entity component, no
  scene serializer, and not even the live `AnimatedMeshPreview` ever consumes (the
  preview compiles its own controller internally, `animated_mesh_preview.cpp:214`).
  Its only egress is a `controller.json` Save/Load that nothing reads back. The panel
  gives no indication the graph drives nothing.
- **Physics ▸ Timings** (`physics_statistics_panel.cpp:107-125`): gated on
  `PhysicsConfiguration::profiling`, which **nothing ever sets** —
  `create_simulation()` takes no arguments (`main.cpp:239`). "Profiling off" shows
  forever, with no control anywhere to turn it on.
- **Animator preview** (`animator_preview_panel.cpp:222`): the only `load_gltf` call
  is hard-coded to `examples/assets/rigged_arm_anim.gltf` (`main.cpp:269`); the
  Project browser routes `.gltf` double-clicks to an **external app**
  (`editor_panels.cpp:5127-5134`). If the hard-coded asset is missing the panel is
  permanently stuck on "No character loaded".
- "Show in Explorer" / open-with-default are silent no-ops off Windows
  (`editor_panels.cpp:170-192`, bare `(void)path` on the `#else` branch).

**Serialization asymmetries (beyond §1.3):**

- `recent_scenes` round-trips through preferences (`preferences.cpp:636,665`) but
  nothing ever appends to it and there is no Open Recent menu — writes an empty array
  forever. There is **no File ▸ Open Scene item at all**; opening a scene requires
  double-clicking it in Project.
- "Secondary Shadow Casters" (`:2682`) *is* consumed by the engine
  (`quality.cpp:84,225`) but is the one shadow field omitted from the preferences
  block (`preferences.cpp:96-111`) — the only Rendering slider that resets on restart.
- `GameViewSettings` (aspect/orientation/fullscreen) has no serializer — resets every
  launch. Meteorology's log path/interval are function-statics — same.
- Menu labels lie about bindings: "Ctrl+N" on New Entity (`:785`) is display-only
  (`editor_contexts.hpp:58-63` binds no such chord), and Redo's "Ctrl+Y" text is
  hardcoded, not derived from the actual binding.

**Stale documentation that misdescribes live behavior** (worth fixing in the same
passes): `editor_context.hpp:64` (visibility defaults), `editor_context.hpp:113-115`
(play state "not wired" — it is), `editor_panels.hpp:311-318` (central node "empty
for a future viewport" — it holds Scene/Game), `game_view_settings.hpp:61-64`
(fullscreen ≠ "Maximize on Play"), `editor_panels.cpp:3757` (nonexistent "Profiler
HUD"), `editor_panels.cpp:4988-4989` (persistence claim), `atmosphere_nest.hpp:33-35`
(scene-serialization claim), `batch_evaluator.hpp:51` (resolver fills budgets — it
does not).

### 1.5 Architecture debt behind the UX

- **`editor_panels.cpp` is 6,616 lines** holding 15 windows, the menu bar, modals,
  scene commands, the VFX authoring UI, the input manager, and the layout builder.
  The codebase already established the per-domain pattern (`animation/`, `audio/`,
  `physics/` directories); the monolith simply predates it. Clean extraction seams
  (line ranges in §3, phase UX4) take it to ~500 lines of genuine shell.
- **Undo is whole-world JSON snapshots** (`command_history.hpp:40-49`, depth 50).
  Coverage is good (~150 sites) but three big surfaces bypass it entirely: the whole
  particle-system authoring UI (`:5747-6024`, `set_particle_effect_source` with no
  record), the 1018-line Environment panel (`:4981-4983`), and the Lighting panel's
  environment writes — and `draw_lighting_panel:3963-3967` calls `record()` **once
  per frame of a slider drag**, flooding the 50-deep stack in under a second
  (everywhere else correctly uses `begin_change`/`end_change`).
- **`ViewportPanel::draw` takes 31 positional parameters** and never sees
  `EditorContext` (`viewport_panel.hpp:184-208`) — the largest bypass of the shared
  context; `main.cpp` hand-unpacks the context at three call sites.
- **21 function-local statics hold real UI state** (rename buffers ×3, meteorology
  logger, weather-map controls, effect-library cache, rebind state, the two animation
  panels' entire authoring state) — process-global, non-reentrant, unpersistable.
- **The clipboard silently guts entities**: `ClipboardEntity`
  (`editor_context.hpp:147-173`) has no material, particle emitter, audio
  emitter/listener/reverb-zone, reference frame, or surface anchor — all real
  components. Copy/paste of an audio source yields a silent shell.
- **Duplicated helpers**: `world_of()` copied verbatim into `animation_panel.cpp:79`;
  the sun-editor block duplicated across two panels; the inline-rename block written
  three times with three static buffers; the `memcmp(&settings_before,…)` persistence
  idiom pasted five times; the `begin_change`/`IsItemDeactivatedAfterEdit`/`end_change`
  dance hand-written ~30 times while `vector3_field`/`scalar_field` (`:273`, `:294`)
  already encapsulate it and are used only by Transform.
- **Widget inconsistency**: 117 `SliderFloat` vs 40 `DragFloat` with no rule for which
  quantity gets which; sun intensity is a slider in two panels while light intensity
  is a drag; the same distance field is formatted `"%.0f"` in one panel and
  `"%.0f m"` in the other; `TreeNode` / `CollapsingHeader` / `SeparatorText` are
  interchanged for section breaks within a single panel.
- **Interaction parity vs Unity** — present and correct: multi-select in Hierarchy
  (click/Ctrl/Shift with anchor semantics, works filtered), hierarchy drag-reparent
  and reorder with insertion line, double-click-to-frame, per-field undo on nearly
  every Inspector widget. Missing: multi-object Inspector editing (Inspector shows
  `selected_entity` only, `:1507`; viewport cannot multi-select, `main.cpp:901-907`),
  **Duplicate (Ctrl+D)**, Delete-key binding, every drag-drop except within Hierarchy
  (no Project→Hierarchy, Project→Inspector texture slots — the decal texture is a
  typed path string, Project→viewport), recursive Project search, console
  severity/filter/collapse (raw `vector<string>` with Clear only, `:6036-6046`),
  component context menus (Reset/Copy/Paste values), Lock Inspector, prefabs.

---

## 2. Target design

### 2.1 Window taxonomy and the default layout

Every window gets a home. The Unity-shaped default layout, built by a rewritten
`build_default_layout` that docks **all** panels (open or not — DockBuilder docks
closed windows too, so opening them later lands them in the right place instead of
floating):

```
+------------------------------------------------------------------+
| Menu bar                                                         |
| Toolbar (fixed strip: play/pause/step | gizmo | overall quality) |
+---------------+--------------------------------+-----------------+
|               |                                |                 |
| Hierarchy     |     Scene | Game | Preview     |  Inspector      |
|               |          (tabs)                |  [Environment]  |
|               |                                |  [Lighting]     |
|               |                                |  [Rendering]    |
|               |                                |  [Post Process] |
|               |                                |  [Meteorology]  |
+---------------+--------------------------------+  [...]          |
| Project | Console | Text Editor | Animation |  |                 |
| Audio Mixer | Statistics | ... (tabs)        |  |                 |
+------------------------------------------------+-----------------+
| Status bar                                                       |
+------------------------------------------------------------------+
```

- **Center**: Scene, Game, Preview as tabs (Preview joins the center node — it is a
  viewport and needs viewport real estate).
- **Right**: Inspector, with all *settings* panels (Environment, Lighting, Rendering,
  Post Process, GPU Culling, Meteorology, Physics) docked behind it as tabs. This is
  Unity's own pattern (Inspector / Lighting / Occlusion share the right stack) and it
  bounds the tab overflow: settings panels are open-on-demand, land in a predictable
  place, and never float over the scene.
- **Bottom**: Project, Console, and the timeline-shaped tools (Text Editor, Animation,
  Animator Graph, Animator, Audio Mixer, Audio Profiler, Statistics) as tabs.
  Console defaults to its own tab *position* directly after Project.
- **Toolbar becomes a fixed strip**, not a dockable window: rendered with
  `BeginViewportSideBar` like the status bar, no close button, no title bar. Play
  gains a keyboard chord (Ctrl+P, matching Unity). This removes both the 6 %-split
  hack and the "closed the Toolbar, lost the Play button" failure.
- **Window menu gets domains as submenus** — General (Scene, Game, Hierarchy,
  Inspector, Project, Console), Rendering (Rendering, Lighting, Post Process,
  GPU Culling), World (Environment, Meteorology), Animation (three panels + Preview),
  Audio (Mixer, Profiler), Analysis (Statistics, Physics, Text Editor), and at the
  bottom: **Reset Layout**, ImGui Demo.

### 2.2 Layout lifecycle

- **Pin `io.IniFilename`** to `user_config_dir()/layout.ini` — same directory as
  `preferences.json`, same rationale. Launch directory stops mattering.
- **Persist `PanelVisibility`** (and `show_preferences`/`show_input_manager` folded
  into it) in `preferences.json`. The open set survives restart; combined with
  full-coverage DockBuilder defaults, a fresh machine and a returning user both get a
  coherent screen.
- **Window ▸ Reset Layout**: deletes the ini node tree and re-runs
  `build_default_layout` next frame (ImGui supports `DockBuilderRemoveNode` +
  rebuild), and resets `PanelVisibility` to defaults. The escape hatch for every
  wedged-layout state.
- **Fix the first-frame order**: draw menu bar and status bar before the dockspace so
  the work area is correct when the default layout is first built.
- **Unify the Game fullscreen state machine** into `ViewportPanel` (the no-camera
  fallback delegates to the same panel object instead of function statics).
- Persist `GameViewSettings` alongside (it is two enums and a bool).

### 2.3 Settings architecture — one owner per domain

**The structs.** `RenderSettings` stays the render authority and *loses* its foreign
cargo. A new `SushiEngine::Simulation` settings aggregate is introduced (working name
`SimulationSettings`, `include/SushiEngine/sim/simulation_settings.hpp`):

```
enum class AtmosphereQuality { Low, Medium, High, Ultra };   // sim-owned

struct AtmosphereSimulationSettings
{
    AtmosphereQuality quality = AtmosphereQuality::High;      // resolves the nest grid
    // future: QG core grid override, step budget, …
};

struct SimulationSettings
{
    AtmosphereSimulationSettings atmosphere;
    // future: physics profiling toggle (§UX3), physics substeps, …
};
```

- `resolve_quality()` **stops emitting `atmosphere_nest`** and the dead
  particle/animation budgets. A sim-side
  `resolve_atmosphere_quality(AtmosphereQuality) → AtmosphereNestSize` takes over the
  Low…Ultra grid table (`quality.cpp:171-259` rows move verbatim).
- The resolved nest size travels to the renderer through `Environment` (which already
  carries `atmosphere_nest` parameters to `stage_atmosphere`) instead of through the
  per-view `QualityParams`. This *also* closes the latent multi-view hazard: the nest
  size becomes device-global state carried by device-global data, not per-view state
  that happens to agree.
- The raw-enum leak in `ray_traced_shadow_pass.cpp:138` moves into a resolved
  `QualityParams` field, restoring the "no pass branches on the raw enum" contract.
- The contradictory QG-grid doc comments are reconciled (the grid stays deliberately
  fixed; `quasigeostrophic_core.hpp:104-108` is corrected).

**The preset.** The Toolbar (and the Rendering panel header) gets an **Overall
Quality** combo: Low / Medium / High / Ultra / Custom. It is *derived, not stored*:
it displays the common tier when all domain tiers agree and "Custom" when they
diverge; selecting a tier writes every domain tier in one gesture. Each domain's own
combo lives in its own panel (Render Quality in Rendering; Atmosphere Quality in
Meteorology, which finally owns something) with a tooltip stating exactly what the
tier controls — and the atmosphere combo warns inline, *at the control*, that
changing it restarts the running weather.

**Persistence ownership matrix** (the corrected contract):

| State | Owner | Persisted | Rationale |
|---|---|---|---|
| `RenderSettings` incl. `RenderQuality` | per-user | `preferences.json` | Machine capability, not scene content |
| `SimulationSettings` incl. `AtmosphereQuality` | per-user | `preferences.json` | Same — grid resolution is a budget |
| `Environment` (sun, sky, fog, GI, clouds, nest physics, observer) | **per-scene** | `.sushiscene` (full round-trip, load applies it, preferences no longer override) | Lighting/weather are scene content in every shipping engine; fixes §1.3's "written but never read" and the ~35 lost constants |
| Default environment for *new* scenes | per-user | `preferences.json` (renamed `default_environment`) | Keeps today's convenience without the override bug |
| Panel visibility, layout, GameViewSettings, gizmo mode/space, last scene, recent scenes | per-user | `preferences.json` + `layout.ini` | Editor session state |

The borrowed-pointer hazard is closed in the same pass: the persisted environment
copy strips/never-serializes the borrowed `AtmosphereForcing`/`WeatherField` pointers,
and re-installation nulls them explicitly.

**Panel ownership moves:**

- The ~35-constant nest-physics block (`:4559-4890`) moves from Environment to
  **Meteorology**, which becomes the atmosphere authoring+diagnostics panel its name
  promises. Environment keeps what a level artist touches: sun, sky, fog, GI,
  surface, stars, clouds *look*.
- The sun block appears in exactly one panel (Environment; Lighting keeps punctual
  lights, IBL, and shadow *rendering* controls) — the duplicate widget block and the
  divergent shadow subset in Lighting are deleted, replaced by a "Sun & sky live in
  Environment" link-line (ImGui `TextLinkOpenURL`-style focus jump).
- Exposure: one owner (Post Process). The Environment atmosphere section links to it.
- Time-of-day: one owner (Environment's observer/Solar System section); Meteorology's
  "Match sky to atmosphere" remains but is re-labeled to state what it changes and
  routed through the same code path.
- The triplicated "Tier resolves to" readout becomes one shared widget drawn from one
  helper.

### 2.4 Truth-in-UI policy — wire or remove, with verdicts

The standing rule going forward: **a control that does nothing is a bug, not a
placeholder.** Disabled+tooltip is the only legitimate way to ship a not-yet-wired
control. Verdicts on today's inventory (absorbing `editor_feature_sync_gaps.md`'s
deferred list):

| Item | Verdict |
|---|---|
| Autosave checkbox | **Wire it** — a real timer keyed to `scene_is_dirty` + interval; it is a small, honest feature and the checkbox already round-trips |
| Upscaler combo + "Runs:" label | **Remove the combo** (and `RenderSettings::upscale` with it) until a second upscale backend exists; the temporal path is already governed by Anti-Aliasing. Removal per the contract at `upscaler.cpp` — or, if the seam should stay visible, disable with "Built-in temporal only today" tooltip. Recommended: remove |
| GPU Culling "Freeze frustum" | **Wire it** — upload the flag, latch the frustum in `cull_pass.cpp`; it is the single most useful culling debug tool and the struct doc already reserves it |
| GPU Culling "Show statistics" | **Remove the checkbox**, replace the dead text with the real numbers: a compact readback of cull counts into the panel (the Statistics-panel timing plumbing already exists) |
| Physics ▸ Timings | **Wire it** — `PhysicsConfiguration::profiling` set true while the Physics panel is open (its own doc comment says exactly this); timings then flow |
| Material "Receive Shadows" / "GPU Instancing" | **Remove both checkboxes** until the passes read the fields (engine work tracked separately); today they are pure false promises |
| Audio Authoring panel | **Decision needed** (§4): either add to build + `PanelVisibility` + Window ▸ Audio, or delete the files. Recommended: wire it in — the audio system doc says compile-verified, ready |
| Animator Graph panel | **Connect minimally**: drive `AnimatedMeshPreview` from the authored graph (it already compiles a controller internally), state clearly on the panel that scene entities do not consume controllers yet. Full entity-side animator components are engine scope, out of this overhaul |
| Animator preview loader | **Wire it** — "Load character…" file picker + Project double-click routing for `.gltf/.glb` to the preview; drop the hard-coded path |
| `recent_scenes` | **Wire it** — append on open/save, File ▸ Open Recent submenu; add the missing **File ▸ Open Scene…** while in there |
| "Ctrl+N" label, "Ctrl+Y" label | Bind New *Scene* to Ctrl+N (relabel New Entity), derive menu shortcut text from the live bindings |
| Off-Windows Explorer/open no-ops | Disable the menu items with a tooltip on non-Windows rather than silently doing nothing |
| VFX SortMode / SimulationDomain / BeamModule; audio acoustic-geometry authoring | **Backlog** (unchanged from `editor_feature_sync_gaps.md`) — real authoring surfaces, out of UX-overhaul scope |

### 2.5 Widget and interaction standards

Codified in a new shared header (`editor/ui/panel_widgets.hpp`, phase UX4) and applied
tree-wide:

- **Field helpers own undo**: `scalar_field`/`vector3_field` siblings for float,
  angle, color, combo, checkbox — every one wrapping the
  `begin_change`/`IsItemDeactivatedAfterEdit`/`end_change` dance once. The ~30
  hand-written copies collapse into calls.
- **Widget selection rule**: bounded physical quantity → Slider with unit suffix in
  the format string (`"%.0f m"`); unbounded → Drag; discrete → Combo. One rule ends
  the 117-vs-40 split.
- **Double-precision round-trip helper** replaces the ~90 hand-written
  `static_cast<float>` dances.
- **Section rule**: `CollapsingHeader` for top-level sections, `SeparatorText` inside,
  `TreeNode` only for genuinely tree-shaped data.
- **Naming alignment**: window title = Window-menu label = `PanelVisibility` field
  (`animator_preview` → `animator`, etc.); stale doc comments from §1.4 fixed.
- **Unity-parity interactions** (phase UX5): Duplicate (Ctrl+D), Delete key,
  drag-drop Project→Hierarchy (instantiate), Project→Inspector texture/mesh slots
  (replacing the typed-path decal field), Project→Scene viewport, recursive Project
  search, console severity levels + filter + collapse (structured `ConsoleLine`
  replacing `vector<string>`), component header context menu (Reset / Remove /
  Copy-Paste values), clipboard completed to full component coverage, rename/UX
  statics moved into `EditorContext`.

---

## 3. Implementation plan

Phases are ordered so that trust-destroying defects die first, each phase is
independently shippable, and no phase depends on a later one. Every phase updates
CHANGELOG.md, fixes the stale doc comments it touches, and lands its tests in the
same change (CONTRIBUTING §5). All building/verification through `se build` /
`se test --suite functional`; editor-visual checks are enumerated per phase as a
manual checklist since ImGui panels have no automated harness.

### UX0 — Stop the data loss  *(prerequisite for everything; small, surgical)*

**Status: shipped (2026-07-29).** All items below landed; the one scope change is
recorded in item 4.

1. **Serialize lights, decals, and materials** in `capture_scene`/`apply_scene`
   (`scene_serializer.cpp`) — the `IWorldEditor` API already exists
   (`simulation.hpp:601,709,1027,1030,1098`). This single fix repairs Undo, Redo,
   Save/Load, and Play→Stop simultaneously, because all four ride the same
   serializer. Shipped with it, following the VFX live-handle precedent
   (`effect_serializer.cpp:225-232`): texture **source paths** persisted on the
   entity (`Simulation::MaterialTexturePaths`, `DecalParams::*_path`), with the
   load-from-disk resolve pass (`resolve_scene_textures`) re-deriving every handle
   from its path — kept beside the material, not inside it, because the extract
   copies the material per instance per frame and nine strings do not belong on
   that path.
2. **Complete `ClipboardEntity`** (material + paths, particle emitter + effect,
   audio emitter/listener/reverb zone, reference frame, surface anchor) so
   copy/paste stops gutting entities.
3. **Fix the per-frame undo flood**: `draw_lighting_panel`'s per-light list moves to
   `begin_change`/`end_change` like every other drag site.
4. **Add undo to particle-system authoring** (drag = one bracketed step, discrete
   library-load/burst edits = one recorded step). *Scope change:* the
   Environment/Lighting environment writes were **deliberately deferred to UX2** —
   undo snapshots carry only the entity array today, so bracketing environment
   edits now would commit snapshots that cannot restore the environment: a fake
   undo, worse than none. UX2's snapshot format change (environment riding the
   undo/play captures) is the honest place for it.
5. **Tests** (`tests/functional/integration/test_scene_serializer_roundtrip.cpp`,
   compiled against the editor's serializer TUs and the real simulation via
   `sushi_sim`): capture/apply equality for light + decal + material + paths,
   default-material file cleanliness, and undo/redo through `CommandHistory`.
   All three pass, measured standalone against the live runtime.

Acceptance: Ctrl+Z after any edit leaves lights/decals/materials intact; Play→Stop
restores the scene bit-identically; the round-trip test pins it. ✔

### UX1 — Layout overhaul  *(the visible transformation)*

**Status: shipped (2026-07-29).** All nine items landed. Implementation notes:
`PanelVisibility` moved to its own header (`editor/core/panel_visibility.hpp`) and the
gizmo enums to `editor/gizmo/gizmo_state.hpp`, so `preferences.hpp` can persist both
without inheriting ImGui; the two-modifier chord (Ctrl+Shift+P) needed a new
`ButtonBuilder::bind(Key, Key, Key)` overload (`ChordGate` always supported two, the
fluent API offered one — and the more specific chord must be tested first, since a
chord requires its modifiers rather than excluding extras). "Audio Authoring" is
docked into the bottom stack pre-emptively so UX3's wiring lands already homed.
Persistence is pinned by `Unit_PreferencesRoundTrip` (3 cases, run standalone against
the real store). The acceptance checklist below is visual and remains for the
project owner's editor pass.

1. Rewrite `build_default_layout` to dock **all** windows per §2.1's map.
2. Pin `io.IniFilename` to `user_config_dir()/layout.ini`.
3. Persist `PanelVisibility` + `GameViewSettings` + gizmo mode/space in
   `preferences.json` (round-trip tests in the existing preferences test pattern).
4. Add **Window ▸ Reset Layout** (DockBuilder node teardown + visibility reset).
5. Reorder the frame: menu bar and status bar before the dockspace (fixes the
   first-run node-size bake).
6. Convert Toolbar to a fixed `BeginViewportSideBar` strip; bind Play/Pause to
   Ctrl+P / Ctrl+Shift+P in `editor_contexts.hpp`.
7. Regroup the Window menu into domain submenus; alphabetize within groups.
8. Unify the Game fullscreen state machine into `ViewportPanel`.
9. Fix stale comments: `editor_context.hpp:64`, `editor_panels.hpp:311-318`.

Acceptance (manual checklist): fresh start (deleted layout.ini + preferences) shows
the §2.1 layout with nothing floating; every panel opened from the Window menu docks
into its home node; Reset Layout recovers from any drag state; restart preserves
open panels and layout regardless of launch directory.

### UX2 — Settings domain separation  *(the SOLID fix)*

**Status: shipped (2026-07-29).** All eight items landed. Implementation notes and
deviations from the letter of the plan:
- The environment JSON shape was extracted into its own owner
  (`editor/serialization/environment_serializer.{hpp,cpp}`), shared by the scene
  file, the preferences' `default_environment` (legacy `environment` key still
  read), and `capture_scene` — which now returns `{entities, environment}` (bare
  arrays still accepted). The shape gained fog, fog volumes, GI, observer, and
  `ocean_roughness`, which the "full round-trip" claim required and the old shape
  silently lacked.
- The env-write undo bracketing is one shared `commit_environment_edit` /
  `finish_environment_edit` pair (edge-triggered like the particle panel's, since
  these panels detect changes by memcmp, not per widget), used by Environment,
  Lighting, Meteorology, and Post Process's scene-exposure slider.
- Exposure's single owner is Post Process, which now hosts the environment's
  pre-tonemap multiplier ("Scene Exposure") beside the EV chain and labels which
  is scene content — rather than leaving `Environment::exposure` authorable
  nowhere.
- Item 2 resolved as "delete": the animation budgets went too
  (`max_skinned_instances`, `bone_lod_bias`, `animation_influences` had no
  consumer either); `quality_params.hpp` now states the every-field-has-a-consumer
  rule.
- The QG grid is documented as deliberately fixed (`QuasiGeostrophicGridSize`),
  with the sidecar-match rationale; only the nest is tiered.
- `MeteorologyLog` moved onto `EditorContext` (`core/meteorology_log.hpp`).
- Tests: `Unit_AtmosphereQuality` (table, 384 km invariant, High = baseline) and
  two new `Integration_SceneSerializer` cases (environment capture/apply,
  environment undo/redo) — all run standalone against the real runtime; the first
  run caught and fixed a stale-fog-volume-tail defect in the new serializer.
  The acceptance's weather-clock observation and the scene-file reopen check
  remain for the project owner's editor pass.

1. Introduce `SimulationSettings` / `AtmosphereQuality` /
   `resolve_atmosphere_quality()` (§2.3); move the nest-grid table out of
   `resolve_quality()`; route the resolved size through `Environment`.
2. Delete the dead tier outputs (`max_particles`, `particle_sim_substeps`, animation
   budgets) from `QualityParams` — or wire the animation budgets if the batch
   evaluator grows a consumer in the same pass; recommended: delete, re-add with a
   consumer. Fix `batch_evaluator.hpp:51`'s claim accordingly.
3. Fix the raw-enum leak (`ray_traced_shadow_pass.cpp:138`) via a resolved param.
4. Overall Quality preset combo (derived, "Custom" on divergence) in the Toolbar;
   Render Quality combo stays in Rendering with a truthful tooltip; Atmosphere
   Quality combo lands in Meteorology **with the weather-restart warning at the
   control**.
5. **Environment becomes scene-owned**: `open_scene` stops overwriting the loaded
   environment; preferences keep a `default_environment` applied only to *new*
   scenes; `preferences.cpp` serializes the full environment (fog, fog volumes, GI,
   observer included) for that default; borrowed pointers stripped/nulled on both
   paths. The Meteorology "Switch the nest on" mirror bug disappears with the
   override. In the same pass, the undo/play-mode snapshots gain an `environment`
   block beside the entity array, and the Environment/Lighting panels' environment
   writes get their deferred-from-UX0 `begin_change`/`end_change` bracketing —
   deferring both together is what makes environment undo real rather than a
   snapshot that cannot restore what it claims to.
6. Panel ownership moves per §2.3: nest physics → Meteorology; single sun block;
   single exposure owner; single shadow editor; shared "Tier resolves to" widget;
   Meteorology's CSV logger moves out of the draw function into a small
   `MeteorologyLog` service owned by `EditorContext`.
7. Reconcile the QG-grid doc contradiction; fix `atmosphere_nest.hpp:33-35` and
   `editor_panels.cpp:4988-4989`.
8. **Tests**: `resolve_quality()` output contains no atmosphere field (compile-time
   by struct change); `resolve_atmosphere_quality` grid table pinned per tier;
   environment scene round-trip including nest physics + fog + GI; preferences
   round-trip of `SimulationSettings`.

Acceptance: switching Render Quality Low↔Ultra never rebuilds the nest (weather
clock uninterrupted, verifiable in Meteorology's clock section); switching
Atmosphere Quality does, with the warning shown at the control; a saved scene's
authored environment survives close→reopen bit-identically.

### UX3 — Wire or remove  *(every control tells the truth)*

**Status: shipped (2026-07-29).** The four items the first tranche left open landed
in the second: **Freeze frustum** is wired end to end (`cull.comp` gained
`frozen_view_proj`/`frozen_delta_eye` in `CullParams`, `CullPass` latches the
camera-relative view-projection and the eye at the freeze and rebases each sphere
onto the latched origin, so the frozen frustum stays pinned to the world; LOD gate
and occlusion keep the live camera); **Show statistics** became real numbers (the
dead checkbox and `GpuCullingSettings::show_statistics` are gone; the panel shows
drawn/tested/%-culled from `ISceneView::cull_statistics`, which existed unused —
plumbed through `ViewportPanel` and `EditorContext::scene_cull_drawn/tested`);
the **Animator Graph drives the preview** (`AnimatedMeshPreview::apply_controller`
compiles the authored `ControllerDesc` against the loaded character, restoring the
known-good controller on failure; the panel states scene entities do not consume
controllers yet); and the **preview loader** is wired (Animator panel Load
Character field + Project-panel double-click routing for `.gltf/.glb` through
`open_character_in_preview`). ⚠ The second tranche was **not** compile-verified
locally at the user's direction ("derleme, devam et") — `se build` is the first
compile of `cull_pass.{hpp,cpp}`, `cull.comp` (host `Params` and the GLSL
`CullParams` block must mirror field-for-field, in order), the animator files, and
the panel edits.

**First tranche (earlier the same day):** Landed: Autosave (real timer + interval
preference, `Unit_Autosave`), Upscaler combo + `RenderSettings::upscale` removed
(the frame never read it; `IUpscaler` seam stays), Physics profiling wired
(`ISimulation::set_physics_profiling` → `IPhysicsStepper::set_profiling_requested`
→ `PhysicsConfiguration::profiling`, consumed at solver build; the panel says so),
Material Receive-Shadows/GPU-Instancing checkboxes removed, Audio Authoring wired
into build + PanelVisibility + Window ▸ Audio (which exposed and fixed a missing
`audio_scene.hpp` include in `audio/bank.hpp`), `recent_scenes` + File ▸ Open
Scene…/Open Recent, Ctrl+N → New Scene with menu shortcut hints derived from live
bindings, off-Windows shell items disabled with a reason. Out of plan but landed
with this tranche (user requests): Scene view fullscreen (Shift+Space through the
shared `ViewportPanel` fullscreen machine) and the Scene-resize image-loss fix
(descriptor sets freed before the idle wait + per-frame target rebuilds during a
drag; now rebuild-then-release plus a ~100 ms debounce).
**Remaining:** GPU-culling Freeze-frustum wire, Show-statistics → real cull
counts, Animator Graph → preview connection, Animator preview loader.

Execute the §2.4 verdict table: wire Autosave, Freeze-frustum, Physics profiling,
recent-scenes + File ▸ Open Scene/Open Recent, animator-preview loading, Ctrl+N
rebind + live shortcut labels; remove the Upscaler combo, GPU-culling
Show-statistics checkbox (replaced by real counts), Material
receive-shadows/GPU-instancing checkboxes; connect Animator Graph to the preview;
Audio Authoring per the §4 decision; disable-with-tooltip the off-Windows shell
items. Each wire lands with its consumer-side test where the seam is testable
(autosave timer logic, recent-scenes list mutation, profiling flag propagation).

Acceptance: a sweep of every interactive control in every panel finds zero
zero-consumer controls; every not-yet-implemented seam is visibly disabled with a
reason.

### UX4 — Monolith decomposition + widget library  *(velocity for everything after)*

Extraction order (each is a pure move + include fix, no behavior change, verified by
clean build):

| Step | New file | Moves | LOC |
|---|---|---|---|
| 1 | `editor/atmosphere/meteorology_panel.{hpp,cpp}` | `editor_panels.cpp:2879-3581` | ~703 |
| 2 | `editor/environment/weather_panel.{hpp,cpp}` | `:4184-4890` | ~707 |
| 3 | `editor/vfx/particle_panel.{hpp,cpp}` | `:5378-6024` | ~647 |
| 4 | `editor/render/render_settings_panels.{hpp,cpp}` | `:2577-2877`, `:3583-3777` | ~496 |
| 5 | `editor/project/project_panel.{hpp,cpp}` | `:82-269`, `:4994-5269` (takes `<windows.h>` out of the monolith TU) | ~463 |
| 6 | `editor/scene/scene_commands.{hpp,cpp}` | `:503-774` (document model, not UI) | ~272 |
| 7 | `editor/scene/hierarchy_panel.{hpp,cpp}` + `editor/scene/inspector_panel.{hpp,cpp}` (one `draw_X_component` per component) | `:882-1259`, `:1488-2575` | ~1466 |
| 8 | `editor/scripting/script_panel.{hpp,cpp}`, `editor/input/input_manager_window.{hpp,cpp}`, `editor/ui/modals.{hpp,cpp}` | `:1261-1477`, `:6403-6584`, `:6182-6338` | ~556 |

Plus: `editor/ui/panel_widgets.{hpp,cpp}` (§2.5 helpers: undo-wrapping fields,
shared sun editor, `push_if_changed` persistence helper replacing the five `memcmp`
copies, the double↔float round-trip, shared rename widget replacing the three static
buffers); `world_of()` promoted there and the `animation_panel.cpp` copy deleted;
`ViewportPanel::draw`'s 31 parameters folded into a `ViewportFrameInputs` struct;
the 21 state-bearing function statics migrated into `EditorContext`/panel state
structs. End state: `editor_panels.cpp` ≈ 500 lines of shell (menu bar, toolbar,
console, statistics, status bar, theme, layout).

### UX5 — Interaction parity  *(the Unity muscle-memory set)*

Duplicate (Ctrl+D) + Delete key; drag-drop: Project→Hierarchy, Project→Inspector
asset slots (decal texture path field replaced), Project→viewport; recursive Project
search; structured Console (severity, filter, collapse, timestamps; `editor_log`
gains a level parameter); component-header context menus (Reset / Copy / Paste
values / Remove); filtered-Hierarchy context menu gains the missing Unparent; text
editor close prompts on dirty tabs; multi-object Inspector editing for the common
components (Transform first) — scoped per §4's decision.

### UX6 — Visual identity + polish

Theme pass over `apply_theme` (`editor_panels.cpp:6164`): consistent spacing/rounding
scale, hover/active states, disabled-state clarity; icon set for panel tabs and the
toolbar (play/pause/step/gizmo); widget-rule sweep (§2.5) across every panel; unit
suffixes and tooltips on every physical quantity (the atmosphere panels' tooltip
discipline — e.g. `:4636-4642` — extended everywhere); status-bar content review;
`se editor` first-run experience check on a clean profile.

**Dependencies:** UX0 → none (do first). UX1 ⟂ UX2 (parallelizable). UX3 depends on
UX2 for the settings items only. UX4 any time after UX0, ideally after UX2 so moved
panels move once. UX5/UX6 last.

---

## 4. Open questions — resolved (project owner, 2026-07-29)

1. **Audio Authoring** (UX3): **wire into the build + menu.**
2. **Animator Graph** (UX3): **minimal connect-to-preview**, with the panel stating
   that scene entities do not consume controllers yet.
3. **Multi-object Inspector editing** (UX5): **full common-component multi-edit** —
   not Transform-only. Fields shared by every selected entity edit all of them;
   mixed values render as ImGui's mixed-value state.
4. **Upscaler seam** (UX3): **remove the combo** (and `RenderSettings::upscale`)
   until a second backend exists.
5. **`default_environment` preference** (UX2): **keep** the user-level default
   environment for new scenes.

---

## 5. Acceptance bar for the whole overhaul

1. **Nothing lies.** Every interactive control either changes engine/editor state
   observably or is disabled with a stated reason. (UX0, UX2, UX3)
2. **Nothing is lost.** Any authored state — scene content, environment, layout,
   panel set, preferences — survives undo, play-stop, save/load, and restart, or the
   UI says explicitly that it will not. (UX0, UX1, UX2)
3. **One owner per setting.** No value is editable in two places; no control in
   domain A changes behavior in domain B without saying so at the control. Render
   Ultra never touches the weather. (UX2)
4. **A fresh install looks like an editor.** First launch shows the Unity-shaped
   layout, nothing floating, Console visible, and every window opened later lands in
   a sensible dock. (UX1)
5. **The muscle memory works.** Ctrl+D, Delete, Ctrl+P, Ctrl+N, Ctrl+S, drag-drop
   from Project, search everywhere it is expected. (UX5)

---

## 6. Handoff — for the next agent (written 2026-07-29, end of the UX0–UX3 session)

**Where things stand.** UX0 (data loss), UX1 (layout), UX2 (settings domain
separation), and UX3 (wire-or-remove) are implemented, plus two out-of-plan user
items: Scene-view fullscreen (Shift+Space, `ViewportPanel::set_fullscreen`) and the
Scene-resize image-loss fix (`resize_to` frees ImGui descriptor sets only *after*
the view's idle-waiting rebuild; `request_resize` debounces a live drag ~6 frames).
Remaining phases: **UX4** (monolith decomposition + `panel_widgets.hpp`), **UX5**
(interaction parity — multi-edit must cover ALL common components, per the locked
decision in §4), **UX6** (visual polish).

**Verification state — read this before trusting the tree.**
- Everything up to and including UX3's *first* tranche was syntax-checked with the
  project flags and the runnable tests pass standalone: `Unit_PreferencesRoundTrip`
  (3), `Unit_AtmosphereQuality` (3), `Unit_Autosave` (3),
  `Integration_SceneSerializer` (5, against the real runtime).
- UX3's *second* tranche (freeze-frustum: `cull_pass.{hpp,cpp}` + `cull.comp`;
  cull counts: `viewport_panel.hpp`, GPU Culling panel; animator bridge:
  `animated_mesh_preview.{hpp,cpp}`, `animator_graph_panel.cpp`,
  `animator_preview_panel.cpp`; character loader in `editor_panels.cpp`) was **not
  compile-verified locally** at the user's direction — `se build` is its first
  compile. The editor-side TUs of that tranche *did* pass `-fsyntax-only`; the
  render-side pair and the shader did not get checked (no local Vulkan-SDK include
  path / GLSL validator). `CullPass::Params` and `cull.comp`'s `CullParams` block
  must stay mirrored field-for-field, in order.
- Syntax-check trap: `clang … | head; echo $?` reports *head's* exit code — use
  `${PIPESTATUS[0]}`. `editor/main.cpp` additionally needs
  `-IC:/Projects/sushiengine/input`.

**Concurrent-session caution.** A parallel session works in this same tree on the
physics/atmosphere phases. Shared files this overhaul also touched:
`sim/runtime_simulation.cpp`, `include/SushiEngine/sim/{simulation.hpp,
physics_services.hpp, physics_simulation.hpp}`,
`include/SushiEngine/atmosphere/quasigeostrophic_core.hpp`, `tests/CMakeLists.txt`,
`docs/CHANGELOG.md`. That session independently added the same
`profiling_requested_` member to `physics_simulation.hpp` (deduplicated; theirs
kept). Expect its in-flight work in any commit touching those files.

**Small debts left deliberately.**
- The standalone run of `Integration_SceneSerializer` currently needs an
  `add_reduce` shim for `runtime_graph_builder.hpp` (recipe in the session memory
  and in [[standalone-test-harness]]); the `se test` lane is unaffected.
- The Audio Authoring project is session-scoped (no project-file persistence yet);
  the panel is wired and docked.
- `docs/ARCHITECTURE.md` §10's editor bullet still describes the pre-live-World
  shell in places beyond the lines corrected here.
- The environment's undo bracket shares one `environment_change_active` flag
  across panels; a drag spanning a release-then-immediate-grab across *different*
  panels coalesces into one step (accepted, matches the particle panel's pattern).
