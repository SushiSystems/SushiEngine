# Changelog

All notable changes to SushiEngine are documented here.

Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) — versions follow
[Semantic Versioning](https://semver.org/).

---

## [Unreleased]

### Added
- 2026-08-08 — Added `docs/design/asset_cooking.md`: import-time cooking to a GPU-ready blob
  (meshoptimizer LOD chains, quantization, BC textures) and hitch-free instantiation over a
  transfer-queue staging ring, phased COOK0–COOK5.
- 2026-08-08 — Added `docs/design/render_optimization.md`: baseline, per-cascade shadow
  culling, sort keys, GPU LOD selection, sky split and the prepass experiment, phased R0–R6
  with Profiler-number acceptance.
- 2026-08-08 — Added a real `enabled`/`activeInHierarchy` flag: a disabled entity now stops
  physics, audio and render for its whole subtree, not just render. `visible` is narrowed to a
  local, render-only toggle by the same change, and no longer cascades (see Changed below). See
  `docs/design/entity_lifecycle_system.md`.
  - Added the Inspector header's second ("Visible") checkbox and Hierarchy panel dimming for
    disabled rows. Both checkboxes carry a tooltip, since neither has room for a label.
  - Added `enabled` to scene serialization and the copy/paste clipboard, so it survives Save/Load,
    Undo/Redo, Play→Stop, prefab capture/apply and duplicate.
- 2026-08-08 — Added the Profiler panel: frame history, CPU/GPU breakdowns, renderer, memory
  and system sections (mock-first per `docs/design/profiling_system.md` PROF0).
- 2026-08-08 — Wired draw calls, triangles, cull counts, heap budgets and texture residency into
  the Profiler and Statistics panels (PROF2).
- 2026-08-08 — Wired the Profiler panel's System and Memory rows to live CPU, memory and GPU
  (NVML) readings; NVML absence falls back to a derived GPU-busy figure instead of blank rows
  (PROF3).
- 2026-08-07 — Added `tools/documentation/check_design_citations.py`: it resolves every backticked
  path a `docs/design/` document cites and fails on one naming nothing, in continuous integration,
  so a stale citation fails the build.
- 2026-08-07 — Added `docs/design/remaining_work.md`: every phase the corpus audit found open,
  withdrawn or unmeasured, in one table derived from the seventeen roadmap sections and rewritten
  whenever one of them changes.
- 2026-08-06 — Added prefab assets: an entity subtree saved as a `.sushiprefab`, placed as
  instances, and rebuilt from the asset when a scene is opened at an older revision. See
  `docs/design/prefab_system.md`.
  - Added the authoring gestures: drag an entity from the Hierarchy onto a project folder to save
    it as a prefab, and drag an imported model into the Scene view to place an instance of it.
  - Added `model_import`, which writes `<asset>.sushiprefab` beside a glTF file, so changing an
    asset's `.meta` rebuilds every instance of it the next time a scene is opened.
- 2026-08-05 — Added "Imported" mesh support to Renderer (`ShapeParameters::mesh_path`/`::mesh`),
  wired to render extraction and render passes. Static props now work without `Crowd` or physics.
  See `docs/design/static_mesh_authoring.md`.
  - Added the Inspector Renderer section's "Imported" mesh choice and Create ▸ Objects'
    "Imported Mesh" entry.
  - Added `mesh_path` to scene serialization and round-tripped with undo/redo, preserving both
    the mesh path and the loaded mesh handle; uses the same pattern as `CrowdParameters`.
- 2026-08-05 — Added File > New/Load Project: `EditorContext::project_root` can be changed at
  runtime instead of only at startup, through the same unsaved-changes guard `request_new_scene`
  already uses. See `docs/design/project_selection.md`.
  - Added a directories-only picker modal (`project_picker.cpp`), no native OS dialog and no new
    dependency.
  - Changed `default_projects_root` from a `main.cpp`-local function into a shared one so the
    File menu can seed the picker's starting directory.
- 2026-08-05 — Added `docs/design/project_selection.md`: File ▸ New/Load Project, reusing the
  already cross-platform preferences store and the scene-replacement unsaved-changes guard. Design
  only, §9's P0 not started.
- 2026-08-05 — Added `docs/design/static_mesh_authoring.md`: placing an imported glTF as a plain
  visual prop, an "Imported" kind on the existing Renderer component rather than a new one. Design
  only, §11's P0 not started.
- 2026-08-04 — Added a root `README.md`, which the project never had, and reduced `docs/README.md`
  to an index that holds no facts of its own.
- 2026-08-04 — Added `docs/architecture/`, sixteen chapters split out of the 3841-line
  `ARCHITECTURE.md` and grouped by tier, with 73 section-number citations rewritten as links and
  353 pre-restructure paths corrected.
- 2026-08-04 — Added a `README.md` to each of the twenty-two modules, stating what it owns, its
  tier, its dependencies and its test coverage; `docs/modules/README.md` indexes them.
- 2026-08-04 — Added `docs/documentation-style-guide.md`, which sets the voice, the naming rules
  and length ceilings a script can measure, plus `docs/reference/glossary.md` mapping every phase
  code family to its owning document.
- 2026-08-04 — Added `tools/documentation/check_documentation_length.py`, enforcing those ceilings
  and failing on a link whose file *or* anchor resolves to nothing, and
  `tools/documentation/check_module_documentation.py`, failing on a module with no README. Both
  run in a new `Structure and documentation rules` CI job.
- 2026-08-04 — Added `tools/layering/check_include_layering.py`: `sushiengine_add_module()` only
  checks declared dependencies, so a raw `target_include_directories()` into another module's
  include root created the same coupling by a route nothing inspected.

- 2026-08-04 — Added `--backend runtime|native` to `se build`, `se test`, `se run` and `se clean`:
  nothing in the CLI could select the native CMake lane. Each lane configures into its own tree,
  reconfigured when its cache disagrees.
  - Added a CI job that configures, builds and runs the native lane on a stock runner, with no SYCL
    toolchain and no SushiRuntime checkout.
  - Added CTest labels to `se_native_execution_tests`, which registered its tests with none, so
    `ctest -L` matched nothing on that lane.
- 2026-08-04 — Added `Unit_PlayerLaunchConfiguration` and `Unit_PlayerBootManifest`, covering the
  player's `boot.json` reader and the command line over it — the part of `applications/player/`
  reachable before a window or a device exists.
- 2026-08-04 — Added `Unit_InputReplayJSON`, pinning the round trip of `input/replay_json.hpp`,
  which shipped with no test. Its lack of an engine consumer is deliberate: one would put
  `nlohmann/json` on the dependency-free input core.
- 2026-08-04 — Added the Terrain window (Window ▸ World ▸ Terrain): the near-field body's layer
  stack, editable in place, plus every field of the last frame's node selection.
  - Added `Terrain::ITerrainAuthoring` and `ISceneView::terrain_authoring()`, the seam a host
    reaches a body's ground through; `PlanetTerrain` implements it.
  - Changed a layer edit to queue the resident tiles it invalidated for recompilation, bounded by
    the frame's upload budget, so an edit reshapes ground that is already on screen instead of only
    ground that streams in later.
  - Changed `PlanetTerrain::set_body` to drop the layer stack with the body it was authored against.
- 2026-08-04 — Added the Crowd component's authoring surface: an Entity ▸ Objects ▸ Crowd item, an
  Inspector section, an Add Component entry, copy/paste, and a `crowd` block in the scene format.
  - Added `skeleton_path`/`clip_path`/`mesh_path` to `CrowdParameters`, so the component carries the
    files its handles came from; `set_crowd_parameters` re-registers the skeleton and clip from
    those paths on every write.
  - Added a Crowd round-trip to `test_scene_serializer_roundtrip.cpp` covering the snapshot path,
    the scene file, an unloadable rig, and undo.
- 2026-08-04 — Added a measured peak-stress readout to the Inspector's Soft Body section, driven off
  `IWorldEditor::soft_body_maximum_stress` each frame, with a warning row once the body is past the
  yield stress its material declares.
- 2026-08-04 — Added a soft-knee bloom threshold, applied at the first downsample only because every
  coarser level is built from its output, so `BloomSettings::threshold` and `threshold_knee` shape
  the pyramid instead of only persisting.

### Fixed
- 2026-08-07 — Fixed 1,074 stale citation sites across seventeen design documents. The restructure
  moved every engine module and no cited path followed it, leaving 617 of 680 resolving to nothing.
- 2026-08-07 — Corrected seven status lines across eleven design documents and withdrew four claims
  the tree does not carry: SushiLoop's Windows continuous integration, chunk-delta recording and
  real sockets, plus model import's colliders.
- 2026-08-07 — Corrected the design corpus's understated phases: physics PX, VFX beams and SDF
  collision are complete, and model import, the prefab system, planetary terrain and the
  cross-platform plan are in progress rather than designed.
- 2026-08-07 — Fixed physics `P1`, `P2`, `P6` and `P7` reading as accepted: their §13.1 numbers are
  timed against a desktop GPU that no development machine here has, so each now reads as built and
  unmeasured.
- 2026-08-07 — Fixed a malformed row in `docs/design/physics_system.md`'s phase table: seven
  pipe-separated fields where every other row carries six, splitting `P4`'s status across cells
  reading "In progress." and "Complete".
- 2026-08-07 — Registered four missing phase-code families in `docs/reference/glossary.md` — model
  import, prefabs, static mesh authoring, project selection — and corrected the `VFX`, `RHI`,
  `PLATFORM` and `UHM` ranges.
- 2026-08-07 — Fixed hiding an entity leaving its children drawing: every render extract gated on
  its own `visible` flag, so hiding a model's root hid only the root. They now ask the whole
  ancestor chain, as `activeInHierarchy` does.
- 2026-08-07 — Fixed an imported glTF appearing in the Hierarchy but never drawing: its geometry
  entities got a Shape but no Renderer, and drawing gates on both, so every part resolved its
  mesh and stayed invisible without an error.
- 2026-08-07 — Fixed a nested entity losing its local transform through `apply_scene`, drifting
  hierarchies on every Save/Load and undo: `set_parent` preserves the world pose, so parenting
  after the transform divided the stored local out.
- 2026-08-06 — Fixed a vehicle's authored surfaces never reaching its contacts: every body of every
  car resolved to the default material. Added `VehicleAsset::materials` and resolved each body's
  `material_index` against it.
- 2026-08-06 — Fixed `docs/architecture/domain-physics.md` describing a contact path that no longer
  exists: `resolve_contacts()`, a `ContactBody` view over two worlds, sweep-and-prune, and "no
  friction, no restitution".
- 2026-08-06 — Fixed a raycast, sweep or overlap missing a vehicle entirely: the scene's query
  hierarchy held its rigid bodies and planes but never a car's shell nodes or wheels, and placing a
  vehicle did not mark the hierarchy stale.
- 2026-08-06 — Fixed a `.meta` sidecar that fails to parse cooking its asset at the project default
  silently: `CookBakeState::take_unreadable_sidecars` names it and the Bake panel logs one warning
  per asset.
- 2026-08-06 — Fixed a vehicle in a scene receiving neither gravity nor a broadphase proxy: the
  scene's body inventory never enumerated its vehicles, so a car spawned at altitude fell
  1.67e-05 m over 90 ticks instead of metres.
  - Added `VehicleInstanceT::for_each_body`, the one enumeration `live_slot_count`,
    `write_every_body` and `rebuild_contact_index` now share.
  - Added `CollisionLayers::vehicle`, so a car's shell nodes and wheels collide with the world
    at their authored radii and not with each other.
- 2026-08-04 — Fixed `samples/sandbox` and `samples/physics/pgs_demo` calling
  `SushiRuntime::API::Runtime::create()`. They are the two targets that prove the native lane
  builds, and SushiRuntime is not in that lane's build graph at all.
- 2026-08-04 — Fixed the command line guide documenting neither `se player` nor `se climatology`,
  and listing two `--suite` values where the tool accepts five.
- 2026-08-04 — Fixed the getting-started samples, which named types, headers and signatures that no
  longer exist: `World` and `Schedule` take an `Execution::Context`, and `run()` returns an
  `Execution::RunReport`.
- 2026-08-04 — Fixed `tests/goldens/render/README.md` claiming no baseline is recorded; two are,
  and are checked in.
- 2026-08-04 — Fixed `docs/design/physics_system.md` declaring "nothing in this document is
  implemented" while its own roadmap records P0 through P7 and PX complete.
- 2026-08-04 — Fixed 144 source files citing `docs/slop/`, and every citation of the deleted
  `ARCHITECTURE.md` across the design corpus, five C++ files and the pull request template.
- 2026-08-04 — Fixed the `functional` CI job installing no cgltf while the root CMakeLists adds
  `engine/` on every lane and `engine/asset/gltf` locates the header with `find_path(... REQUIRED)`,
  so the job could not configure.
- 2026-08-04 — Fixed `load_boot_manifest` throwing out of the launch when a `boot.json` field has
  the wrong type, which also left the manifest half-overwritten; a mistyped field now costs only
  itself.
- 2026-08-04 — Fixed `docs/README.md`'s test-tree paths, file counts, `ctest --test-dir` invocation
  and editor build-tree name.
- 2026-08-04 — Fixed `ClothParameters` carrying no documentation: its Doxygen block sat above
  `CrowdParameters`, which had one of its own, so Doxygen attached neither block to the cloth
  struct.
- 2026-08-04 — Fixed `capture_scene` never capturing a `SoftBodyParameters` component, which made
  Save→Load, Undo/Redo and Play→Stop each destroy every soft body in the scene.
  - Added `Scene::ISceneBlobTable`/`SceneBlobTable` and optional `blobs` parameters on
    `capture_scene`/`apply_scene`: a scene file inlines the cooked blob as base64, while a snapshot
    names it by content hash so undo steps share one copy.
  - Changed `CommandHistory` and the editor's play-mode snapshot to each own a blob table beside
    their snapshots.
  - Changed a soft body whose asset resolves neither inline nor from the table to restore as an
    entity with no soft body, rather than one holding an empty blob the physics can never build.

### Removed
- 2026-08-04 — Removed `Material::blend_mode`, `Render::BlendMode`,
  `Material::render_queue`/`receive_shadows`/`gpu_instancing` and `ColorGradeSettings::lut_enabled`
  with their widgets and serialization: no renderer or shader read them.

### Changed
- 2026-08-08 — Changed `visible` to stop cascading through the hierarchy: only `enabled` cascades
  now. A scene with a `visible=false` parent will render children that the old cascade hid; an
  author who wanted the whole subtree gone wants `enabled` off on that parent instead.
- 2026-08-06 — Changed per-asset cooking overrides to live in a `<asset>.meta` sidecar rather than
  a path-keyed object in the project document, so moving an asset no longer orphans its settings.
  - Migrated an older project's overrides once, when its cooking document is read, reporting both
    what moved and what was dropped because the asset it named is gone.
- 2026-08-04 — Changed `docs/slop/` to `docs/design/`, indexed, each document carrying a sourced
  status header; a directory named after what the corpus looked like when it was dumped there told
  readers to skip the only account of why anything is shaped the way it is.
- 2026-08-04 — Changed `INTRODUCTION.md`, `CLI_GUIDE.md`, `VEHICLES.md` and `CHANGELOG.md` into
  `docs/getting-started/`, `docs/guides/` and `docs/reference/`. The three files GitHub resolves by
  name stay at `docs/` root.
- 2026-08-04 — Changed 339 changelog bullets over 240 characters down to the ceiling, keeping the
  clause carrying the engineering reason. Bullets already inside it were left untouched.
- 2026-08-04 — Changed the design corpus to wrap at 100 columns: 1131 paragraphs rewrapped, word
  stream verified identical against the previous revision.
- 2026-08-04 — Changed the player's launch parsing into `resolve_launch_configuration`, so the
  manifest-then-command-line precedence is a function with a return value rather than straight-line
  code in `main` that no test could reach.
  - Documented both execution backends and the new flag in `docs/README.md` and `docs/CLI_GUIDE.md`.
- 2026-08-04 — Changed material texture sampling to honour `Material::wrap_mode` and
  `anisotropic_filtering`, registering it once per sampling configuration in the bindless heap,
  whose combined image sampler the draw path cannot swap.
  - Changed the texture library's baseline registration from repeat/16× anisotropy to the sampling
    an unedited material asks for, repeat/8×. Light cookies, decals, particle atlases and the UI
    atlas share that baseline and so sample at 8× too.
- 2026-08-03 — Moved the world-tier authoring types out of `SushiEngine::Editor`, which the editor
  application also used for its panels, into `SushiEngine::Authoring`.
- 2026-08-03 — Restored the truncated Apache licence banner in 246 files, removed all 216 separator
  comments, and rewrote the comments that narrated what the code used to be into statements of what
  it does.
- 2026-08-03 — Renamed every abbreviated variable, parameter, member and function name this
  repository owns to its full spelling (`params` → `parameters` and thirteen others); external and
  serializer-key names are untouched.
- 2026-08-03 — Renamed every CMake target, function and macro this repository owns to the
  `sushiengine_` prefix, so none of them can collide with a SushiRuntime one in the single flat
  CMake namespace the two trees share.
  - Removed the seven bare-`sushi_` `ALIAS` shims; every consumer now names the module target
    directly (`sushi_sim` → `sushiengine_simulation`, `sushi_scene` → `sushiengine_serialization`,
    and so on).
  - Renamed the executable targets (`se_editor` → `sushiengine_editor`, `render_probe` →
    `sushiengine_render_probe`, and the seven others) and gave each an `OUTPUT_NAME`, so every
    binary on disk keeps the name it had.
  - Renamed the nine build options the same way (`SE_BUILD_TESTS` → `SUSHIENGINE_BUILD_TESTS` and
    the rest); an existing tree's cached `SE_*` entries are now silently ignored, so re-pass the new
    names or reconfigure clean.
- 2026-08-03 — Moved every engine source into `engine/<tier>/<module>/`, each module carrying its
  own `include/` and `source/`, with the tier order enforced at configure time by
  `sushiengine_add_module()`.
  - Added `applications/`, `samples/`, `tools/` and a flattened `tests/`; the root now holds project
    metadata and eight directories.
  - Changed the `se` CLI's project-root marker to a `.sushiengine-root` sentinel, and gave each
    configure lane its own tree under `build/`.
- 2026-08-03 — Changed the demos to build only under the new `se build --examples`; `sandbox` and
  `pgs_demo` still build by default.
- 2026-08-03 — Renamed every title-cased acronym in an identifier this repository owns to its full
  upper-case spelling (`XpbdSolver` → `XPBDSolver` and the rest), C++ and GLSL mirrors together;
  external API is untouched.
- 2026-08-03 — Renamed every abbreviated type name this repository owns to its full spelling (`Mat4`
  → `Matrix4`, `AppConfig` → `AppConfiguration` and the rest); the GLSL block names those structs
  mirror and external types are untouched.
  - Split the two `Render::Atmosphere*` parameter structs, which shared a stem while describing
    different things: the nest's meteorology constants are `AtmosphereNestParameters`, the sky's
    scattering ones `AtmosphereScatteringParameters`.

### Added
- 2026-08-03 — Added PLATFORM0 S6: a headless `se_player`, a `boot.json` launch manifest and
  `--manifest`/`--headless`/`--frames N`. Headless runs tick a deterministic 1/60 rather than
  wall-clock, so a CI run reproduces on any machine.
  - Verified by running `se player -- --headless --frames 5`: a headless Vulkan device ticked an
    empty world five times and exited clean, with `se editor --no-run`, `se render --probe golden`
    and `se test` re-run at their prior results.
  - Recorded two gaps as not verified: loading a real `.sushiscene` end to end through `se player`,
    and a windowed launch with the suspend/resume path, which needs a display and manual
    interaction.
- 2026-08-03 — Added PLATFORM0 S5: `se_player` and `PlayerApp`, the ImGui-free runtime shell.
  `start()`/`frame()`/`suspend()`/`resume()` are separate calls, so a host with its own loop shape
  can drive them.
  - Recorded what is out of scope rather than missing: headless operation (S6), the `Render::UiView`
    overlay channel, so a scene's Canvas entities do not draw in the player yet, and any audio
    playback.
  - Verified via `se player --no-run`, with zero `sushi_imgui` in the link line. Not yet launched:
    opening a window and presenting a frame needs a display to verify against.
- 2026-08-03 — Closed RHI0: `se render --probe golden -- --update` recorded both cases and a
  following plain run reproduced them bit for bit, confirming the `pass_capture.cpp` format-table
  fix put `depth_prepass` back in the pass list.
- 2026-08-03 — Added the build verification promised for PLATFORM0 S4's `present_scene_view()`: it
  compiles clean under `se editor --no-run`, the build that actually exercises the changed
  translation units. Not yet exercised at runtime.
- 2026-08-03 — Moved `editor/`'s scene serializers into a standalone `sushi_scene` and its window
  seam into `sushi_platform`, neither depending on ImGui, so a future `se_player` can link them
  without ever linking `sushi_imgui`.
- 2026-08-03 — Changed the renderer's pipeline-cache and shader-source paths off the build tree,
  adding `Platform::user_data_directory`: a shipped player writing its cache into its install
  directory fails on a notarized macOS bundle.
- 2026-08-03 — Added `IWindowRenderer::present_scene_view()`, the only way a player with no ImGui
  gets a scene view onto the swapchain. It blits rather than copies, since the resolve is
  `R8G8B8A8_UNORM` and the swapchain `B8G8R8A8_UNORM`, and restores the resolve to the layout the
  render graph believes it is resting in, or its next `render()` compiles a barrier for a transition
  that never happened.
- 2026-08-03 — Changed `Loop::App` to build via `Execution::Runtime::create()` and the root CMake
  file to read `SE_EXECUTION_BACKEND` before `project()`, since reading it after was too late to
  stop the compiler probe demanding SYCL.
  - Recorded two pre-existing, unrelated failures surfaced by this session's `se test`:
    `Unit_SynopticField.TheFlowAroundALowIsCyclonic` and
    `Integration_VehicleComponent.TheEntityFollowsTheVehiclesCore`.
- 2026-08-03 — Corrected two stale claims in physics_system.md §17.5/§18: the island-per-region row
  described a design never built, and R4/R6 are built and unused rather than unavailable; R9 (raised
  2026-08-02) is mirrored (§16.43).
- 2026-08-02 — Changed `RuntimeGraphBuilder` to skip graph emission for a zero-capacity contact,
  joint or distance-constraint band, which previously dispatched a node per colour whose `when()`
  predicate was always false (§16.37).
- 2026-08-02 — Added `PhysicsStageTimings`'s per-node breakdown, which the 2026-07-30 entry below
  named as blocked: §18 R8 closed by threading `NodeDescriptor::name` through
  `add_parallel`/`add_host`, so `solve_ms` can be attributed.
  - `PhysicsNodeKind` (`physics/core/statistics.hpp`) names the twelve kinds `build_graph` emits,
    and `node_timings` is a fixed array rather than a map a hot path would allocate into: the kind
    set is closed at compile time.
  - Added `physics/core/statistics_from_report.hpp`, grouping a `RunReport`'s node timings by name.
    It is the one file under `physics/core` naming `SushiRuntime::Core::RunReport`, per §17.5's
    one-adapter rule. Not yet called.
  - The Physics panel (`editor/physics/physics_statistics_panel.cpp`) gained a "Solve, by node"
    section under Timings, drawing only the kinds that actually dispatched this tick.
  - Added `test_physics_statistics.cpp`: the conversion's per-name grouping against a synthetic
    `RunReport`, an unrecognised name (`unnamed_task`) contributing to nothing, and every
    `PhysicsNodeKind` round-tripping through its own label.

### Added
- 2026-08-03 — Closed six findings of §16.45's audit of physics features built but never reachable
  from the editor: trigger volumes, continuous collision, `park_sleeping_joints`, the statistics
  panel's breakdown, and two cooking gaps.
  - Added `trigger`/`continuous_collision` to `ColliderParams`, with an Inspector section and scene
    round-tripping: the solver had read `Collider::flags` since the collision system was built and
    nothing authored it.
  - Added a `park_sleeping_joints` checkbox to the Physics panel, staged on `EditorContext` and
    pushed into `ISimulation::set_park_sleeping_joints` once a frame: the solver toggle had zero
    callers and no authoring path at all.
  - Added rows for `PhysicsStatistics::joints`/`::elements`/`::beams` and
    `PhysicsStageTimings::soft_body_ms` to the Physics panel: all four were populated correctly
    every tick and never drawn, so no solver code changed.
  - Added an Advanced section to the Bake panel for the twelve cooking fields with no widget — the
    seven fidelity pin overrides and all five `CookingThresholds` fields — each pinnable field
    showing its derived value dimmed until pinned.
  - Added a "Cooking Override..." item to the Project panel's asset context menu and
    `ImportProfileLibrary::get_override()`: `ImportProfileOverride` had zero call sites outside its
    own tests, so every asset cooked at the project default.
  - Removed `DerivedCookingParameters::fidelity`, computed by `resolve_cooking_parameters` and read
    by no cooker or panel, since the panel reads the authored `parameters.fidelity` directly.
  - Corrected the claim that `IJointService::set_joint_motor`/`set_joint_limits` are unreached:
    destroy-and-recreate is a stated correctness choice, since its warm-started multipliers
    accumulated under the old target (§16.45.1).
  - Added `CookBakeState::set_profile_storage_path`/`load_profiles`/`save_profiles`, serializing the
    project default and the override map to `<project_root>/cooking_profile.json`: every cooking
    profile was session-only.
  - Added a Soft Body entry to Inspector's Add Component popup, edited through
    `mutable_values()`/`write_primary()` rather than the field-fanning helpers, since cooked bytes
    are one entity's own body, not a value a multi-selection shares.
  - Full `se test -s functional` run: 1366/1368, the same two pre-existing, unrelated failures
    (`Unit_SynopticField.TheFlowAroundALowIsCyclonic`,
    `Integration_VehicleComponent.TheEntityFollowsTheVehiclesCore`) and no new ones.
  - Decided, with the project owner, to drop the `sycl::half` storage integration:
    `SoftBodyHalfStorage` stays additive and unwired, since the one measurement available is a 0.7%
    mean loss and no GPU here can take the device-side one.
- 2026-08-03 — Added `park_sleeping_joints`, an opt-in flag dropping a sleeping island's joints from
  the solve graph. A joint parks only on a tick its projection contributed nothing, so dispatch cost
  changes and the solve does not.
  - Added `test_joint_parking.cpp`: nothing parks by default, a settled island's joint drops
    `PhysicsStatistics::joints` to zero, teleporting the held body restores it with its load intact,
    and editing a parked joint's motor wakes it.
  - Recorded beams, elements and distance constraints as resident regardless of sleep: beams do not
    participate in island connectivity, so parking by island could park one node while a beam-only
    neighbour is still solving (§16.44).
- 2026-08-03 — Added `NativeBackend`, the SYCL-free implementation of `SushiEngine::Execution`,
  removing `execution/context.hpp`'s `#error` for `-DSE_EXECUTION_BACKEND=native`. Not yet compiled,
  so treat it as a first implementation.
  - Added `Execution::Detail::HazardCore`, an interval map per allocation holding the last writer
    and reader set per byte range; reader predecessors are collected only when the new access
    writes, since read-after-read is never a hazard.
  - Added `NativeBackend::DagCompiler`, ordering a flat node list through `HazardCore` and running
    it as a waved Kahn schedule: `compile()` already ordered every conflicting pair, so a whole wave
    is safe to dispatch at once.
  - Added `NativeBackend::ThreadPool`, a fixed worker pool over a shared ready queue whose dispatch
    is node-granular rather than element-granular, matching a workload of thousands of small,
    independent per-chunk nodes.
  - Added `NativeBackend::{Buffer, Graph, Region, DynamicGraph, Context, Runtime}`, duck-typed to
    match `RuntimeBackend` exactly so consumers do not know which backend they compiled against;
    `DynamicGraph`'s recompose is a full rebuild.
  - `execution_native/CMakeLists.txt` now builds `sushi_exec_native` as a real `STATIC` library
    (`Threads::Threads`) instead of M1's empty placeholder.
  - Retargeted `test_execution_dynamic_graph.cpp` onto `Execution::Runtime::create()`/`.context()`
    instead of naming `SushiRuntime::API::Runtime`, making it a real conformance check once native
    is buildable.
- 2026-08-03 — Added `Execution::Runtime`, the portable factory that stands up a backend: every
  caller previously had to construct a `SushiRuntime::API::Runtime` by hand, a name no
  native-backend translation unit can spell.
- 2026-08-02 — Added `Execution::DynamicGraph`/`Execution::Region`, wrapping the SushiRuntime types
  the way `Graph`/`Context` already do, so §6.6's one region per island stops needing a
  `SushiRuntime::` name at the seam.
  - Added `test_execution_dynamic_graph.cpp`: two independently keyed regions write disjoint slices,
    a drop takes effect on `has_region()` immediately, and a run with no region change does not
    advance `compile_count()`.
- 2026-08-02 — Added a per-tick `continuous_advancement_budget` and populated
  `PhysicsStatistics::continuous_escalations`, which was always zero: conservative advancement runs
  host-side, outside the solver the statistics copy from.
- 2026-08-02 — Added `soft_body_half_storage.hpp`, narrowing a cosmetic soft body's stored position
  and velocity to `sycl::half` between ticks. `HalfVector3` carries no arithmetic: the rule is half
  to store, float to compute (§6.5).
  - Added `examples/soft_body_half_storage_budget.cpp`, stepping the same lattice with plain `float`
    storage and through the narrow/widen seam: it measures the host solver's conversion cost, which
    has no device analogue yet (§16.15).
  - Added `test_soft_body_half_storage.cpp`: a narrow/widen round trip stays within half precision's
    rounding bound, `HalfVector3` is half the width of `Vector3T<float>`, and a free-fall trajectory
    survives the seam.
  - 2026-08-03 addendum: the harness ran — 1331 particles, 6000 elements, 32 substeps: float 12.87
    ms/tick against half 12.97 ms/tick, a 0.7% loss on the host solver's conversion cost and weak
    evidence against the device case.
- 2026-08-02 — Changed manual weather into a seed, so a planet has weather in some places and not
  others: Manual was defined as having no `IWeatherProvider`, so one hand-authored deck stack
  applied to every square metre of the body.
  - Replaced the boolean with `ISimulation::weather_mode` (`Manual`/`Procedural`), both installing a
    provider: `SeededWeather` places pressure centres on a zonal climatology, `ProceduralWeather`
    grows them. The seed survives a mode switch.
  - Added a zonal cloud climatology: three Gaussians put optically thick fractions near
    0.64/0.06/0.66/0.31 by latitude, and the subtropical minimum is why an orbital photograph has
    large clear ocean rather than reading as overcast.
  - Weighted placement onto the storm tracks rather than uniformly over the sphere, since real
    cyclones queue along the jet and the subtropics stay empty. SplitMix64 replaces `std::mt19937`,
    whose mapping is implementation-defined.
  - Added `IWeatherProvider::synoptic_field`, publishing the placement in closed form so
    `cloud.frag` evaluates the same twelve centres as the bake: `publish_field`'s lattice is a few
    hundred kilometres across, far too coarse from orbit.
  - Changed `cloud_globe_envelope` off `cloud_deck_a[i].z`, which restated the observer's own column
    as the weather everywhere, and made the mesoscale noise multiplicative so it cannot manufacture
    cloud where the field says clear.
  - Added a geostrophic `wind_at` from the provider's coverage gradient, so a seeded storm's cloud
    advects as its circulation says; the sign is easy to write backwards, so the test asserts a
    northern low's wind blows west to its north.
  - Scene files write `mode` and `seed`; the pre-existing `procedural_enabled` key is still read, so
    scenes saved before this open as what they were.
- 2026-08-02 — Added `cloud_globe_envelope`, a third, coarsest layer under both windows, so clouds
  cover the whole planet rather than a 262 km camera-centred square: a unit vector into a seamlessly
  tiling volume has no rim to distort.
  - Kept the layer deliberately coarse, since the regional nest is only 384 km across and invented
    structure beyond it would be detail claiming to be weather; the windows now fade into it rather
    than into nothing.
  - `cloud_height_gradient` moved to `cloud_field_window.glsl` so the bake and the march build the
    same deck profile; two answers about where a deck's top is would have shown as a step at the
    rim.
  - Out there neither the light volume nor the far window's baked sun-depth channel exists, so the
    field carries an analytic self-shadow (the slant path to its own deck top). Without it a planet
    from orbit reads as a uniformly lit sheet.
  - Changed empty-space skipping past the windows into a stated bound rather than a proof, since the
    planet-scale field is a point evaluation and not a max-pool, and gated the near-field cone light
    march to the window it refines.
- 2026-08-03 — Added shadows for secondary directional lights, which shaded flat before, and
  clustered punctual reads to the analytic planet ground and the volumetric fog march, so a light
  near the ground or in fog actually lights it.
- 2026-08-03 — Added a "No cameras rendering" placeholder and an aspect/orientation/fullscreen
  toolbar to the Game view, which previously skipped its `ImGui::Begin` entirely and vanished
  whenever the scene had no camera or no display.

### Fixed
- 2026-08-04 — Fixed `.sushieffect` silently reverting an emitter's `RenderAlignment::Beam` to
  `FaceCamera` on load: the reader range-checked the alignment against four enumerators when there
  are five.
  - Added the missing `beam` object to the effect shape, so `BeamModule`'s seven fields (`enabled`,
    `start`, `end`, `width`, `sag`, `noise_amplitude`, `noise_frequency`) survive a save and load at
    all — they were never written.
  - Moved every enumerator count the reader checks against next to its enum
    (`VFX::RENDER_ALIGNMENT_COUNT` and five siblings), so adding an enumerator is one edit rather
    than two distant ones. Only the alignment was wrong.
  - Added `tests/unit/test_vfx_effect_serializer.cpp`, which round-trips every enumerator of every
    persisted enum by enumeration rather than by example, and every `BeamModule` field.
- 2026-08-04 — Fixed `VFX::SortMode` deciding nothing: it was authored while the depth sort was
  gated on blend mode. `ParticleSortPass` now runs its bitonic stages only when an emitter both
  blends true-alpha and asks for `ViewDistance`.
  - Changed `RenderModule::sort`'s default to `ViewDistance`, so every existing effect keeps the
    ordering it had and `None` becomes a deliberate opt-out; the sort is a whole-pass decision
    because the alpha bucket is shared across emitters.
  - Renamed `ParticleSystem::has_alpha()` to `needs_alpha_sort()`, which is what it now answers.
- 2026-08-04 — Added the Particle Inspector's Beam and Sort controls, two VFX features implemented
  end to end and reachable by no author: `RenderAlignment::Beam` joins the Alignment combo, and a
  Sort combo sits beside Blend.
  - Each particle combo now asserts its label list against its enum's enumerator count at compile
    time, so an enumerator added without a label is a build error rather than a mode nobody can
    select.

- 2026-08-02 — Fixed the march's budget coarsening drawing concentric rings: the step scaled by
  `exp2(0.5 * (real_samples - STEPS))`, and `real_samples` is an integer, so two adjacent pixels
  marched a factor of √2 apart.
  - Bisected rather than guessed: with `Clouds Enabled` cleared the globe is spotless, so the rings
    are cloud-path, and from orbit every pre-existing distance-driven term in the march is
    saturated, leaving the new coarsening.
  - Changed the budget into a distance rather than a count: the closed form
    `t₀ · (1 + march_angular)^STEPS` is continuous in `t₀`, so nothing integer enters the step size
    and no contour can form. `real_samples` is deleted.
  - Recorded the general rule: anything derived from a loop counter is quantised, and a quantised
    quantity varying smoothly across the frame is a visible contour unless something downstream
    smooths it — a ray march has nothing downstream.
- 2026-08-02 — Fixed `Clouds Enabled` not switching off: `WeatherCloudscapeCompiler::compile` forced
  `clouds.enabled = true` and `RuntimeSimulation` assigns that result back every tick, overwriting
  the checkbox before the next frame.
  - Removed a level-triggered override that ran every tick: it could not tell a scene authored with
    clouds off from an author clearing the checkbox half a second ago, and stayed hidden only
    because Manual mode ran no compiler.
  - Changed `enabled` to carry through from the medium like every other authored field: a compiler
    derives what the sky contains, but whether the sky is drawn is the author's, and a derivation
    must not overwrite a decision it cannot read.
- 2026-08-02 — Fixed the cloud shell being a sphere fitted at the observer's latitude on a WGS84
  planet: every consumer subtracted `planet_surface_reference_metres`, right for a local scene but
  wrong once PL1 made the shell planetary.
  - Measured the error: WGS84's radius runs 6 356 752 m at the pole to 6 378 137 m at the equator,
    so a shell fitted at 41° N buries a 1 300 m deck under the tropics, and each boundary crossing
    is a circle of constant latitude.
  - Added `cloud_planet_radius_at` as the single definition — the ellipsoid's radius along a
    geocentric direction — and routed `cloud.frag` and `cloud_panorama.comp` through it, with
    `cloud_ray_shell` replacing the spherical intersections.
  - Contained the fix: the horizon gate takes the ratio of a radius to the surface and the Bruneton
    LUTs are parameterised by altitude above their own spherical bottom, so the medium stays
    spherical and only the geometry became oblate.
  - The bake, light volume, shadow map and far-light passes needed no change: they are all
    parameterised in `height01` and never convert a position, so the entire defect was reader-side.
    That is the layering doing its job.
- 2026-08-02 — Fixed the march spending a cost budget as a reach budget: `cloud.frag` ran while
  `real_samples < STEPS` and then stopped, so a ray that exhausted the tier's budget contributed
  nothing for the rest of its length.
  - Measured where it bit: the budget is charged per envelope sample and the envelope is non-zero
    across far more sky than the carve fills, so with the near step floored at 20 m the cheap tier's
    48 samples were gone by about 1.2 km.
  - Changed the budget to coarsen the step past its limit rather than stop the march, since a cost
    bound must degrade resolution and not reach. `carve_shape` integrates the threshold over the
    sample's own footprint, so it stays honest.
- 2026-08-02 — Fixed the cloud resolve having no way to reject stale history: `cloud_variance_clip`
  switched off the only mechanism that evicts a stale sample, so a reprojection error compounded
  over about 33 frames instead of decaying.
  - Changed the tier knob to select which rejection rather than whether: a 9-tap YCoCg variance clip
    above the cheap floor, and a 5-tap cross min/max clamp on it, which bounds the history by the
    extremes present around the pixel.
  - Clamped alpha alongside the colour in both paths: this buffer's alpha is the march's
    transmittance and `CloudCompositePass` folds the sky through it, so an unbounded alpha history
    leaves a stale silhouette with correct colour inside.
- 2026-08-02 — Fixed `synoptic_zonal_coverage` being shaped to total cloud fraction while every
  consumer treats it as the fraction of sky filled with opaque, lit cloud, which drew a third of the
  clearest sky on the planet solid.
  - Restated it as the optically thick fraction: 0.64 at the ITCZ, 0.06 in the subtropics, 0.66 in
    the storm track, 0.31 at the pole, 0.30 across ordinary midlatitudes, mirrored in
    `synoptic_field.glsl` so the two evaluators agree.
  - Changed the high étage's `0.12 +` cirrus term to multiplicative: as an additive floor it was the
    one term no anticyclone could suppress, so a subsiding column kept a permanent veil over the
    top.
- 2026-08-02 — Fixed a cloud being lit by where the camera stood: `cloud.frag` took the solar zenith
  cosine in the camera's radial frame and gated the direct beam on a hard horizon test against it,
  once per pixel.
  - Corrected the comment justifying that gate: the march now reaches tens of degrees of arc, and
    the horizon test is a step, where at sunset a degree is the whole difference between a lit deck
    and a black one.
  - Changed `cloud_sun_at` to evaluate the gate and the transmittance fetch at the sample's own
    radial and radius, inside the `density > 0.001` branch, softened over the Sun's angular radius
    so the terminator is not a razor line.
  - Fixed the Moon's gate being camera-anchored: from a daylit camera the Moon is below the camera's
    horizon, so the `continue` skipped the one light the dark limb had. The reflected sum is now cut
    off at each sample's own horizon.
  - Kept skylight measured where it was measured: the sky-view LUT is a directional map of the
    camera's sky, so the march adds back only the difference, and a sample no sunnier than the
    camera contributes exactly zero.
- 2026-08-02 — Fixed an orbital view rendering its clouds yellow: Bruneton's ratio identity holds
  only while the ray climbs, and at a downward `mu` both fetches march through the body, so their
  ratio is an arbitrary saturated colour.
  - The same bug was silently removing *all* distance extinction from near-horizon rays seen from
    the ground, where the wrong quotient exceeds one and clamps: that is part of the bright horizon
    slab.
- 2026-08-02 — Changed the carve's LOD footprint to the equal-volume mean
  `pow(lateral² · axial, 1/3)`: looking straight down from orbit the step alone band-limits a
  hundred-metre lateral footprint to a kilometre and erases the field.
- 2026-08-02 — Fixed distant clouds rendering as white blocks: `cloud_density_carved` floored its
  feature scale at `footprint * 4`, which keeps the sampler inside Nyquist by making a distant
  cumulus kilometres across at full solidity.
  - Changed `Textures::CloudNoise` to build the march volume a real mip chain with a compute box
    filter (`cloud_noise_mip.comp`) rather than a blit, so the filter wraps the way the REPEAT
    sampler does, and gave it a sampler of its own.
  - Changed the threshold to move with the sampling, integrating the ramp over the detail the filter
    removed: a box-chain mip level is a conditional expectation, so the residual variance is exactly
    `Var(level 0) − Var(level l)`.
  - Recorded that the footprint used for LOD selection is the integration step rather than the
    pixel, deliberately: past `JITTER_FREEZE_METERS` the dither is frozen, so quadrature error there
    would read as structure that is not cloud.
- 2026-08-02 — Fixed the bright line along the horizon: `cloud_composite.frag`'s upsample demotes a
  mismatched tap to weight 0.05, so when all four are rejected the divide renormalises them back
  into ordinary bilinear weights.
  - Explained the one-row width: the epsilon is 2% relative depth and at grazing incidence one
    screen row spans an unbounded range of ground distance, while bilinear reaches only one texel,
    so every row below has four clean taps.
  - Traced why it read near-white: the blend drags `cloud_distance` from about 100 km down under
    `AERIAL_MAX_DISTANCE`, so that row skips the whole extinction tail — a convex blend can never be
    brighter than both neighbours.
  - Fixed it by taking the single nearest-depth tap outright when nothing matches. The leading
    hypothesis, the editor grid plane bounding `t1`, was refuted: `GridPass` writes no depth and
    runs after the clouds.
- 2026-08-02 — Fixed auto-exposure being unable to reach a lit night: `min_ev` clamps the metered
  average from below and so the exposure from above, and the old default of −6 capped the gain at
  11.5× where a moonlit deck needs ~1000×.
  - Changed default `min_ev` from −6 to −14, extended the slider to −16, and relabelled both fields
    by what they do (`Darkest Metered`/`Brightest Metered`), dropping the misleading `EV` unit.
  - Documented that `adapt()`'s `max(avg_luminance, 1e-4)` divide-guard is also a hard 1800×
    exposure ceiling, so `min_ev` stops doing anything below about −13.3.
- 2026-08-02 — Fixed terrain's first drawn frame losing the device: set 0 is a push set that
  `pbr.frag` reads twenty-six descriptors out of, and the terrain pass wrote only the two its own
  vertex shader wanted.
  - Added `passes/shading_set.hpp`, which owns the set as a pair that must stay paired —
    `declare_shading_set` and `write_shading_set` — so a new binding reaches every shading pass
    instead of two of three.
  - Terrain pushes one material per frame, coloured from `PlanetParams::ground_albedo` — the colour
    the analytic ground would have used — so handing the body over to real geometry changes what
    shades it without changing what it looks like.
- 2026-08-02 — Fixed the Assembly window crashing on open: `scalar_field` and `vector3_field` open
  with `ImGui::TableNextRow`, so calling one from a panel that draws no table dereferences a null
  current table.
  - Fixed it by making the widgets total rather than teaching every caller a rule: they now lay
    themselves out by where they find themselves, so there is no precondition left to forget and the
    Inspector is unchanged.
  - The Assembly and joint panels also stopped reaching for those two entirely and draw plain rows,
    which is what `ComponentEditor::number` already does for every component section and was the
    pattern to follow in the first place.
- 2026-08-02 — `cooked_asset_extension` had no case for `CookedAssetKind::NodeBeam`, so a node-beam
  asset written through the store landed as `.sushicooked`. The build had been warning about the
  unhandled enumerator on every editor compile.
- 2026-08-02 — Fixed clouds receiving no skylight and no moonlight: the `sun_radiance * 0.02`
  ambient proxy had no colour of its own and went to exactly zero once the beam was correctly gated,
  taking the night sky with it.
  - Changed `cloud.frag` to bind `SKY_VIEW_LUT_BINDING` and take two taps per pixel, the zenith and
    the sun's azimuth at the horizon, scaled by the raw top-of-atmosphere beam since the LUT carries
    the atmospheric path internally.
  - Changed the Moon and every other reflecting body to reach the clouds: `Astro::celestial_lights`
    already derived each as a directional light sorted by what it delivers, and no cloud pass had
    ever read it.
  - Folded the reflected light into the ambient rather than marching it as a second self-shadowed
    beam: these sources are five to six orders of magnitude under the sun, so what survives to the
    eye is the multiply-scattered part.
- 2026-08-02 — Fixed clouds being lit by a sun that never set: `cloud.frag` multiplied the
  top-of-atmosphere beam with no transmittance and no horizon test, the only sun consumer that did
  not attenuate the beam for itself.
  - Changed `CloudPass` to take `AtmosphereLutPass` and attenuate `sun_radiance` once per pixel at
    the deck's mid-altitude, since the slant path varies by well under a per cent across a 1–2 km
    deck.
  - Added the missing horizon gate (`atmo_ray_sphere(...) > 0` ⇒ no direct beam). Without it the
    LUT's parameterisation folds back below the horizon and returns the *upward* answer, which is
    what kept the deck lit through the night.
  - Fixed the `ambient_color = scene.ambient + sun_radiance * 0.02` leak: with the beam pinned at
    noon that proxy dominated the ambient after dark, so a fully self-shadowed cloud face kept a
    flat colourless glow.
- 2026-08-02 — Fixed distant clouds having no aerial perspective past 31 km: the froxel volume's
  last slice sits at `AERIAL_MAX_DISTANCE`, while a deck seen from the ground runs past 100 km to
  its own base-sphere tangent.
  - Changed `cloud_composite.frag` to continue the view path analytically past the volume with
    Bruneton's ratio identity, applying the chromatic residual to the cloud directly since the
    volume's single luma channel cannot carry it.
  - Fixed the composite's additive in-scatter missing the `sun_radiance` scale that `sky.frag`
    applies to the identical fetch, so it arrived at a twentieth strength and distant cloud was
    never pushed toward the sky's colour.
- 2026-08-02 — Fixed the GPU profiler stopping after 16 passes, where overflow read as absence:
  `measured_frame_milliseconds()` steers auto-exposure and the dynamic-resolution governor from that
  sum. Raised to 64.
- 2026-08-02 — Fixed every body but Earth having its surface rotated: `body_rotation_angle` returns
  the IAU angle W, measured from the node with the J2000 equator, while
  `ecliptic_to_body_equatorial` builds one noded on the ecliptic.
  - Recorded why nothing had noticed: every existing test was a round trip through the same wrong
    convention, and a uniformly rotated planet under a uniformly rotated sky is self-consistent.
    Real elevation data made it observable.
  - Added `Astro::prime_meridian_angle(body, jd)` as the single answer to where the prime meridian
    is, with four consumers going through it; Earth resolves to Greenwich sidereal time, so the home
    sky does not move.
  - Proved it externally, since self-consistency proves nothing here: the Moon is tidally locked, so
    from selenographic (0, 0) the Earth must stand within about 10° of the zenith — 29°–40° of
    elevation before, 80°–89° after.
- 2026-08-02 — Fixed terrain slot eviction overwriting an image a frame in flight was reading:
  `TileResidency::insert` spared only slots bound this frame, while frames N−1 and N−2 hold bindings
  the device has not finished with.
- 2026-08-01 — Fixed the carve's sign: every version until now built clouds by subtraction, which
  can only produce concave features, while a cumulus congestus top is convex protrusions with the
  readability in the creases between them.
  - Added a billow ladder to `cloud.frag` — three inverted-Worley octaves added to the base field
    before the threshold — displacing the isosurface outward into a bulge instead of biting a hollow
    out of a finished shape.
  - The polarity was already free: `noise_worley` returns `1 − distance`, so channel `g` is high at
    cell centres and low at cell walls. The old code took `1 − g` and subtracted it, converting
    round bumps into round *holes*.
  - Changed `cloud_noise_volume.comp` kind 4's `g` channel from an fbm of an fbm nine octaves deep,
    each nesting multiplying the standard deviation by 0.68 and burying the crisp cell boundaries,
    to one inverted Worley octave.
  - Changed creases to be shaded rather than merely carved: the ladder's signed value doubles as a
    local occlusion measure, since the light volume's 128 m texels cannot resolve a 100 m crease.
  - Kept subtraction only under the convective base, where a cloud really is being torn, weighted by
    `1 − cauliflower` so the regimes never fight over a sample; the old `wisp = g` polarity is
    corrected too.
- 2026-08-01 — Fixed `cloud.frag` taking roughly one sample per cloud and never integrating the sky
  in front of the camera: `seg_min` was twelve per cent of the union of every enabled deck, 1344 m,
  and also the clamp's floor.
  - Separated searching for cloud from integrating one: integration is angular (2% of camera
    distance, floored at 20 m, ceilinged at 1200 m), and the interval is tracked apart from the
    jittered sample inside it, an unbiased midpoint rule.
  - Rescaled `cloud_primary_steps_far` from a minimum step count across the march length, which
    resolved a horizon ray at 5 km per sample and a vertical one at 500 m, to a scale-free
    multiplier on the angular rate.
  - Made empty-space skipping conservative: `t += max(dist_step, cell_size)` hopped 2.6 untested
    cells per probe, while `cloud_skip_exit` computes the exact distance to the boundary of the
    region the trilinear probe proved empty.
  - Added `CLOUD_SKIP_RESOLUTION_Y`/`CLOUD_FAR_RESOLUTION_Y` to `cloud_field_window.glsl`, mirroring
    `CloudscapeCompilePass`'s own constants, since these are fixed engine sizes rather than tiered
    settings.

### Changed
- 2026-08-01 — Changed the volumetric cloud model to a per-sample analytic carve, replacing the
  bake-time carve whose 128 m/1024 m window texels could not hold a cloud's shape. See
  `docs/design/atmosphere_system.md` §7.6.
  - Changed `cloudscape_field.comp` to store only the physics envelope — coverage, vertical profile
    and in-cloud water at half scale. All noise taps left the bake, and decks combine as a coverage
    union instead of a density sum.
  - Changed `cloud.frag` to carve the shape per march sample at every distance — domain warp,
    CDF-uniformised base threshold, height-flipped Worley erosion, a fine octave near the camera,
    and a hand-off to the mean past 80 km.
  - New precombined march noise volume (`cloud_noise_volume.comp` kind 4, `CloudNoise::march`,
    128³): uniformised base / erosion fbm / fine fbm / curl potential in one texture, because the
    march has exactly one free image binding.
  - Changed every sun-depth integral to state mass against the carved sky via the shared
    `CLOUD_ENVELOPE_MEAN_SHAPE = 0.45`; the skip field pools the unscaled envelope-times-water
    product so the probe stays a conservative ceiling.
  - The far bake's supersampling dropped 8×8 → 2×2: it anti-aliased a threshold that no longer
    exists in the bake.
- 2026-08-01 — Fixed the cloud march's horizon reach: the length was capped at fourteen shell
  thicknesses, calibrated against a 10 km union, so fourteen times a 1.3 km deck is 19 km and the
  far window was never marched. Now 160 km.

### Added
- 2026-08-02 — Added `CookingParameters::cook_node_beam` and a `NodeBeamPostProcessor`, putting the
  node-beam cooker in the Bake window: `NodeBeamCooker` had no entry point outside C++, so
  `docs/VEHICLES.md`'s first step needed a test.
  - Made the Bake panel's material picker an action rather than a stored choice:
    `NodeBeamCookerSettings` has no field for which preset this was, so the combo applies one and
    relabels itself back to "Apply preset...".
  - Added a node-beam report beside the Collider and Soft body ones, reading only the fields this
    cooker measures — nodes, beams, the bracing count, unbound vertices, mass and inertia — not the
    fields that stay zero for this cook.
- 2026-08-02 — Added `IVehicleService::vehicle_surface`, publishing the shell's collision surface as
  a deformable mesh on the channel cloth already uses, read off the live node bodies, so the drawing
  cannot disagree with the collision.
- 2026-08-02 — Added keyboard driving — arrows for throttle, brake and steering, Left Shift for the
  clutch, Page Up/Down for gears — on a context that is a no-op without a Vehicle. Arrows rather
  than WASD, the gizmo keys.
  - Ramped the controls rather than switching them: a key is a bit and a throttle is not, and
    stepping a pedal between 0 and 1 makes a simulated car undriveable. The ramp lives in the input
    layer, since it is a property of the device.
- 2026-08-02 — Added `docs/VEHICLES.md`: mesh to driving car, why `core_mass_fraction` is a dial
  rather than a switch, why the drivetrain is not made of constraints, and the three things still
  missing, named rather than discovered.
- 2026-08-02 — Added `examples/physics_sample_scene.cpp`, writing the sample scene to a
  `.sushiscene` from the same builder the integration suite steps: a hand-authored blob would be a
  second definition free to drift from the tested one.
  - Changed the scene builder to take a world and nothing else: it took an `EditorContext`, which
    drags ImGui in behind it and made a scene builder unusable from anything that is not the editor
    shell.
- 2026-08-02 — Added `Environment::planet_body_axes` and a `TerrainPass`, putting every visible node
  of the Moon on screen through `pbr.frag`, since `terrain.vert` matches `mesh.vert`'s output
  signature exactly (§9.4).
  - Changed the analytic ground to stand down once per frame rather than per pixel:
    `Scene::suppress_analytic_ground` switches it off when the selection produced nodes, since a
    lunar mare is two kilometres below the reference sphere.
  - Changed `PlanetTerrain` to start with no body and no slot pool, created with the first pack that
    loads and kept across bodies, since the slots are anonymous storage. A scene with no planet pays
    nothing.
  - Added `terrain_frame.hpp`, header-only and Vulkan-free on purpose: three conventions meet in
    that function and each is wrong in a way that still draws a planet, so all three are checked
    without a device.
  - Recorded that terrain draws but does not meet its own exit criteria: the camera starts inside
    the shell, the shipped lunar tier stores 2665 m per texel, and the material is one flat colour
    until P7. The punch list is §20.1's P2c.
- 2026-08-02 — Added §5.5's `VehicleInstance`: a component names a `.sushinodebeam`,
  `IVehicleService` instances it into the scene's own solver, and the rigid core's solved pose is
  written back onto the entity's Transform.
  - Stored a path on the component, unlike every other asset reference at this boundary: a vehicle
    is placed by an author in a scene file that has to survive being reopened on another machine,
    and a path is what survives that.
  - Made `set_vehicles` a rebuild rather than a diff: a vehicle is four hundred bodies and two
    thousand beams placed relative to a cooked structure, so the same vehicle with one number
    changed cannot be patched in place.
  - Forced the tick order rather than choosing it: the tyre model reads its normal load off this
    tick's manifolds, so it cannot run before contacts are submitted, and it puts impulses on the
    wheels, so it cannot run after they are solved.
  - Held input rather than consuming it: throttle is a state a device holds down, so a caller that
    stops calling leaves the pedal where it was, which lets a slider and an input action drive the
    same car.
  - Added a Scene tab to the Vehicle window: put this setup on the selected entity, drive it, read
    the drivetrain back, and see the shell as a side elevation, where a sagging suspension and a
    caved panel are both visible.
- 2026-08-02 — Changed `submit_contacts` off its hard-coded 0.6/0.5/0, carrying `PhysicsMaterial`
  per body on `RigidBodyDesc` and resolving it per pair through `make_contact_params`: an ice cube
  and a rubber block behaved identically.
  - Carried the material by value rather than as an index into a scene table, as `SoftBodyDesc`
    already does: the manifold pass runs on the host with the body's own record in hand, and a table
    would be a second place a material lives.
  - Resolved a side with no rigid body to the default surface, deliberately a fixed ground rather
    than a mirror of whatever stands on it: a floor that copied the cube's friction would cancel the
    difference the author authored.
- 2026-08-02 — Added a layer index and a mask to `ColliderParams`, so §7.7's collision filter is
  authorable: an index rather than a 32-bit field because a body is in exactly one layer, with the
  shift done in `collider_from_params`.
- 2026-08-02 — Added the Assembly window and with it P3's last outstanding item: parts, joints
  between part indices, the collision-filter matrix with symmetry enforced, and a live readout of
  what each joint carries against its threshold.
  - Made instancing produce ordinary entities, one per part: an opaque scene-graph node would be a
    second kind of thing the Hierarchy, Inspector, undo and save each need a case for. The instance
    forgets it was an assembly.
  - Moved the joint editor to `editor/physics/joint_widgets.cpp` for both the Inspector and this
    panel: a second copy of the widget list is how a new limit ends up editable in one and invisible
    in the other.
- 2026-08-02 — Added §14's physics debug draw, toggled per category: contact points with
  impulse-scaled normals, broadphase bounds, island colouring, sleeping markers, and the selected
  entity's joint gizmo with its twist-limit arc.
  - Added `IRigidBodyService::rigid_debug_state`, since a bound is not the collider, an island is
    not a component and "asleep" is the difference between a settled stack and a broken one — none
    of the three was visible from outside.
- 2026-08-02 — Added Entity ▸ Physics Sample Scene: a settling stack, a ramp holding rubber and
  losing ice, an excluded-layer pair beside a control pair, a breakable door, a pendulum and a
  cloth, built through `IWorldEditor` alone.
- 2026-08-02 — Added §5.5's `PhysicsJoint`, so an author adds the component in the Inspector:
  `ISimulation` does not expose the physics boundary, so a joint could only exist in C++ that named
  `IJointService` directly.
  - Added `sim/joint_params.hpp`: `JointType`, its limits, its motor and `JointState` were defined
    inside `physics_services.hpp`, which includes `simulation.hpp`, so the authoring boundary could
    not name them without closing a cycle.
  - Put the joint on one of its two bodies naming the other, rather than on a third entity naming
    both, because that is the ownership question: a door's hinge belongs to the door, and deleting
    the door should take its hinge.
  - Changed `RuntimeSimulation` to reconcile joints each step in the same diff shape
    `set_rigid_bodies` uses, so an unchanged joint keeps its warm start. Staleness is a revision
    counter, since `PhysicsJointParams` is not free of padding.
  - Kept a broken joint broken: the solver destroys it and reports it once while the authoring
    survives, and a runtime flag stops the next reconcile rebuilding the mount. Editing any field
    clears it.
  - Added `damping` to `JointMotorDesc`, a bug found by the exercise rather than a feature: the
    boundary conversion silently dropped `JointMotorT::damping`, so everything reaching the solver
    built spring drives with no damper.
  - Serialized it with the scene as an array index, resolved in the same second pass the parent link
    uses, since an `EntityId` is assigned at creation: a file storing one would reconnect to
    whatever entity was handed that number next load.
- 2026-08-01 — Added `terrain/tile_residency.hpp`, the tile cache's slot table (§7.2): it evicts
  least-recently-bound rather than least-recently-uploaded, and binds a node whose tile has not
  arrived to its nearest resident ancestor.
  - Kept it free of any graphics API — a slot table, a clock and a rectangle: the rectangle is a
    pure function with its own test, since a half-texel error there shifts terrain everywhere it
    inherits and is undiagnosable from a screenshot.
  - Binding through an ancestor touches that ancestor, so a tile many descendants inherit from is
    not evicted underneath them; and a slot already bound this frame is never stolen, so a full
    cache refuses rather than corrupting a queued draw.
- 2026-08-01 — Added `terrain/quadtree.hpp`, CDLOD node selection (§7.1), producing the cut of a
  body's quadtree drawn this frame. Host-only and double-precision: a node centre is a planet-scale
  coordinate, the one place allowed one.
  - Made selection refinement rather than descent: it starts from the six root faces, which already
    cover the body, and splits the node most over the error target, so the cover is exact at every
    stage including where a budget stops it.
  - Added `IHeightSource::tile_bounds`, a capability with a false default, overridden by
    `PackHeightSource` to read a node's elevation band from the pack index without decoding a tile.
  - Measured the node count rather than assuming it and corrected §17's budget: through a 60°
    frustum on the Moon it is nearly flat across altitude (621–1272 nodes at 4 pixels), so the
    shipping target moves to 4 px.
- 2026-08-01 — Added `se planet bake` and the baked terrain asset it writes (§5): a body's
  topography is reprojected onto the cube-sphere quadtree and written as a `.planet` pack with its
  provenance and source checksums inside it.
  - Added `terrain/pack_format.hpp`: the byte layout, `PlanetPack` (a value that adopts bytes and
    refuses a malformed blob whole), and `PackHeightSource`, which resamples the nearest ancestor
    when the quadtree descends past the data.
  - Added `cli/sushiengine/services/planet/`, mirroring the climatology baker, behind a `planet`
    extras group: its dependencies are numpy and a downloader, since global topography ships as a
    raw raster beside a plain-text PDS3 label.
  - Verified the raster's grid convention against known landmarks before baking, because a mirrored
    planet is entirely plausible to look at, and audited the output against the source: worst
    deviation 0.092 m against a 0.184 m step.
  - Added `test_planet_pack.cpp`. Every refusal is driven by synthesized bytes so it is tested on a
    machine with no asset checked out; the cases that need real data read the baked lunar pack and
    skip when it is absent.
- 2026-08-01 — Added `SushiEngine::Terrain`, the planetary terrain foundation (P0): host-only
  headers holding the cube-sphere tile addressing, the tangent-warped projection onto a body's
  ellipsoid, the height seam and the layer stack.
  - Added `cube_sphere.hpp`'s `normalized_difference`, the cancellation-free form §9.2 rests on:
    float32 lands about 0.76 m off against a 0.075 m cell at depth 20, while rearranging the
    difference holds it to about a micrometre.
  - Added `layer_stack.hpp`, where an edit is a small ordered record rather than a raster, which is
    what lets a crater or a dam replicate over a network and be undone; `TerrainLayer::order` is
    unique, so composed ground is a pure function.
  - Added `test_cube_sphere.cpp` and `test_terrain_layers.cpp`: face-crossing neighbours are checked
    by adjacency symmetry over every boundary tile of every face at three depths, and the precision
    claim against a double reference.
- 2026-08-01 — Added §11.1's beam, the fifth constraint kind (§16.22): an axial link whose rest
  length creeps once the recovered load passes a deform threshold, and which is removed once it
  passes a break threshold.
  - Added `physics/constraints/beam_constraint.hpp` and `beam_projection.hpp`, with
    `apply_beam_plasticity` running once per tick on `apply_fem_plasticity`'s reasoning, so a dent
    never depends on the substep schedule.
  - Added `physics/soft/beam_properties.hpp`, deriving a beam's compliance and both thresholds from
    a `SoftBodyMaterial` and a cross-section by the axial-bar relations — §11.2's first correction
    to BeamNG.
  - Changed `IConstraintSolver` to admit beams
    (`add_beam`/`remove_beam`/`read_beam`/`write_beam`/`beam_capacity`), and both solvers to project
    them one node per colour per substep on the same shared colouring every other kind uses.
  - Added `PhysicsCapacities::beams` (zero by default, opt-in like `elements`) and
    `PhysicsStatistics::beams`.
- 2026-08-01 — Added the `.sushinodebeam` asset (§16.23), §11.2's vehicle structure. Five sections
  travel together because a vehicle is not usable without all five: a shell naming nodes from an
  older cook loses its doors on load.
  - Made a core of zero mass a pure node-beam vehicle: §11.2 promised the architecture would not
    choose between hybrid and pure, so the difference is one number in `NodeBeamCoreRecord`, not a
    flag and not a second code path.
  - Kept the beam records deliberately apart from `BeamConstraintT`: they carry the cooked half
    only, so the blob's bytes do not change the day the solver's struct grows a field, invalidating
    every cached asset for a change no artist made.
  - Packed every record with no interior padding (56/72/48/44/88 bytes, asserted), because padding
    makes two cooks of the same input differ in bytes nobody wrote, and the §8.1 cache then serves
    entries that look changed and are not.
  - Re-checked cross-references on the way in: a blob can come from an older writer or a hand edit,
    and each unchecked reference reads into a neighbouring section, which produces a plausible
    vehicle rather than a crash.
  - Stored a skin record as a frame-relative offset, a correction rather than a design: the weighted
    centroid of the four nodes nearest a box corner sits inside that corner, so the first
    formulation rebuilt the rest pose 0.4 m off.
  - Renormalized `read_node_beam_skin_weights` for the reason `read_binding_weights` does: the
    reconstruction sums absolute positions, so a `1e-7` shortfall slides the render mesh off in
    proportion to the distance from the origin.
- 2026-08-01 — Added `NodeBeamCooker` (§16.24), §11.3's cooker: a vehicle's thousands of numbers now
  come from a `SoftBodyMaterial` and the fidelity dial rather than from a person. Six stages, Repair
  through Serialize.
  - Reused the tetrahedralizer's lattice rather than adding a second voxelizer: §8.3's stage 2
    already voxelizes, flood-fills the interior and returns per-vertex masses, and a parallel
    implementation would decide inside differently.
  - Classified the bracing by length rather than building it: a lattice tetrahedralization's edge
    set already contains both of §11.1's kinds. Measured on a 2×1×4 box at fidelity 0.35: 96 nodes,
    429 beams, 209 of them bracing.
  - Took beam numbers from `beam_properties_from_material`, the same function P7-B wrote rather than
    a copy, so a beam's compliance is the axial bar's `L/(E·A)` against a tributary area conserving
    `Σ A·L = volume`.
  - Folded `NodeBeamCookerSettings` into the cache key in the cooker, since the material is not in
    `CookingParameters`: without it the same mesh cooked as steel and as aluminium resolves to one
    key and the second cook is served the first.
  - Split the lattice's interior from its boundary and attached only the interior to the core, the
    only split derivable from a mesh alone: mounting points and parts that come off as units are
    authoring, so the cook produces one part.
- 2026-08-01 — Added `NodeBeamStructure` (§16.25), §11.2's hybrid alive in a solver: node records
  become particles, beam records the fifth constraint kind, the core a rigid body, and `end_tick` is
  where a vehicle dents and loses parts.
  - Made the shell-to-core attachment a ball joint rather than a new constraint kind: §10.3's
    averaging answers a soft-body question, while a node-beam shell has none — the cooker already
    chose the node, and a node is a whole body.
  - Kept the tick boundary with the owner rather than the solver, which projects and does not decide
    policy: `end_tick` reads each beam back, applies plasticity and removes what passed its break
    threshold, and the same for the mounts.
  - Made a part come off by losing its last tie, after which nothing is done to it: its nodes are
    already free bodies, so what the structure adds is the report, since reconstructing whether a
    part is held would walk every beam.
  - Built the core in its principal frame, the only frame in which an inertia tensor is the diagonal
    `RigidBodyT::inv_inertia` stores. Verified by round trip: `body_origin` comes back on the
    authored origin to 1e-12 m.
  - Rolled the whole vehicle back when a budget runs out part way through, verified by what fits
    afterwards rather than by a flag: a vehicle missing the beams that did not fit folds the first
    time it is touched.
  - Measured on the host solver: a mount holds its node to 1.4e-8 m across 19.5 m of travel, a beam
    past its yield dents from the third tick, a door at 50 m/s is reported detached once, and two
    runs agree bit for bit.
  - Added a position-pair form of `apply_beam_plasticity`, with the node-array form deferring to it:
    a structure holds solver slots it reads back a pair at a time, and two implementations of the
    same rule would have drifted.
- 2026-08-01 — Added suspension, wheels and `VehicleInstance` (§16.26): a corner is a `Slider`
  between the core and a hub and a `Hinge` between that hub and the wheel, since suspension is
  joints and drives rather than beams.
  - Added a `damping` rate to `JointMotorT`, the one drive the library could not express: a position
    drive at a compliance is a spring and a spring alone rings forever, while the velocity motor's
    force limit makes a Coulomb damper.
  - Steered without a third joint: the slider locks all three rotations, so the carrier's
    orientation is the slider's chassis-side frame, and rotating it about the strut axis moves
    nothing else — a MacPherson strut.
  - Fixed `predict`'s first-order orientation integration and `update_velocity`'s recovery, which
    together multiplied angular velocity by `1/sqrt(1 + (|ω|h/2)²)` every substep: a free body at 50
    rad/s was down to 33 within a second.
  - Measured on the host solver: four planted corners settle at 0.0617 m against the 0.0617 m
    `m·g/k` predicts and report 2159 N against 2158 N of corner weight, and a soft spring stops on
    its bump stop at 0.1200 m of 0.12 m travel.
  - Made `VehicleAsset` name the `.sushinodebeam`, the core's `.sushicollision` and the corners,
    while `physics/vehicle` reads only the corners: an asset store behind this seam would make
    `physics/` depend on where files live.
  - An asset with corners and no rigid core is refused rather than instanced without its suspension:
    §11.2 keeps the pure node-beam path open through an empty core, and a strut has no chassis body
    to hang from when it is taken.
- 2026-08-01 — Finished P7 with the wind seam and the acceptance scene (§16.29, §16.30): `sim/`
  gained a `WindSampler` alongside its `GravitySampler`, and the physics never names the meteorology
  behind it (§4.5).
  - Made wind arrive as a difference rather than a second drag force: `physics/aero/wind.hpp`
    returns the gap between the drag a body is really in and the drag `predict` is about to apply,
    which is zero in still air.
  - Measured it: a tailwind at the body's own speed gives back exactly what `predict` takes, a
    headwind costs exactly the difference of the two squared airspeeds, and a wind of zero returns a
    hard zero rather than a small number.
  - Wired the cooker's per-node `drag_area` to a body at last: it has travelled in the
    `.sushinodebeam` since P7-C and been read by nothing, and shell nodes now carry a drag constant
    derived from it.
  - Wrote vehicle drag into the core's own drag constant at create time, since drag is a constant
    and not a per-tick force; downforce is not, because it acts along the car's down axis at a
    centre of pressure that is not the centre of mass.
  - Added P7's acceptance scene (`Integration_VehicleAcceptance`): the throttle reaches the driven
    wheels, a hull beam is permanently deformed after the impact, a door is reported detached
    exactly once, and two runs agree bit for bit.
  - Measured the §13.1 vehicle — 400 nodes, 2 000 beams, four wheels, a powertrain — at 0.460
    ms/tick on the host reference solver, printed rather than asserted: a number one solver produces
    and the other cannot is not comparable.
  - Recorded a constraint on that scene: the colour count must be at least the number of constraints
    touching the core, since colouring cannot put two constraints sharing a body in one colour;
    sixteen silently instances nothing.
  - Added the Vehicle window (`editor/physics/vehicle_panel.cpp`): corners, tyres, drivetrain and
    aerodynamics, with the derived arithmetic shown beside the field that feeds it, since the
    numbers that catch mistakes are the derived ones.
- 2026-08-01 — Added the tyre model (§16.28), §11.5: a slip-based force model per wheel, split as
  `tyre.hpp` — slip and load in, force out, no solver — and `tyre_projection.hpp`, which finds the
  patch and spends the answer.
  - Used the brush model rather than a magic formula: Pacejka's curve is a fit whose coefficients
    have no meaning on their own, while the brush model derives the whole curve from two stiffnesses
    and a friction coefficient.
  - Saturated one slip vector once, so combined slip cannot be got wrong: saturating the two axes
    separately lets a wheel produce `μN` sideways and `μN` forwards, which is 1.41 times the
    friction the surface has.
  - Read the load back rather than inventing it: a wheel is a real body with a real contact, and
    nothing here raycasts for the ground — a raycast wheel is the arcade shortcut, and why arcade
    cars cannot drive over a kerb.
  - Fixed the force conversion: `ContactPoint::normal_lambda` is a positional multiplier, so the
    force is `λ/h²` and not `λ/h` — off by the substep is off by a few hundred, and
    `contact_point_load` is now the one place it is written.
  - Added `IConstraintSolver::body_handle`, the inverse of `body_slot`: contacts name their bodies
    by slot, and it cannot be rebuilt outside, since the solver is the authority on which generation
    a slot is on.
  - Kept a wheel off Coulomb friction, because the solver's own friction runs on the same contact
    inside the substep loop and the two add; `SuspensionSetupT::material_index` lets a wheel point
    at a frictionless material.
  - Used one patch per wheel rather than one per manifold point: slip is a property of the patch,
    and evaluating the curve per point would give a wheel across a kerb edge more total grip than
    one flat on the road.
  - Recorded rather than fixed: §7.3's manifold refresh anchors a contact to a material point, so a
    fast-spinning round body carries it round its rim within a tick — 38° at 40 rad/s. It belongs to
    the narrowphase, not the tyre.
- 2026-08-01 — Added the powertrain chain (§16.27), §11.4: engine torque curve and inertia, clutch,
  gearbox, differential and driven wheels. `PowertrainT` is a one-dimensional system knowing nothing
  about bodies, handles or solvers.
  - Kept the state one number: everything downstream of the clutch has its speed determined by
    speeds the caller measured, so the crankshaft is the only free coordinate and a shaft member per
    stage would cache derived values.
  - Solved the clutch rather than damping it: the torque that equalises both sides is computed in
    closed form and clamped to the plate's capacity, since a stiff spring between a flywheel and a
    car in first gear is explicit integration.
  - Made the differential one number rather than three kinds: `lock_torque` at zero is open, large
    is a spool, between is a limited-slip. The lock torques are balanced to sum to zero, because a
    differential divides torque.
  - Charged the gearbox's own shafts their share: geared to the wheels they weigh
    `inertia × ratio²`, and delivering full shaft torque while charging the clutch solve for that
    inertia left the two halves disagreeing.
  - Changed both wheels of an axle to point their axles the same way: `axle` was documented as
    pointing outboard, which nothing physical asked for, so a car handed its differential a mean of
    zero and two reactions that cancelled.
  - Landed the reaction on the chassis rather than the carrier: the differential's casing is bolted
    to the chassis, so squat under power is a sprung reaction, and a vehicle's engine cannot change
    its own total angular momentum.
  - Gave the engine a torque curve rather than one peak number, held flat outside its ends rather
    than extrapolated: a linear extrapolation crosses zero, and reverse torque above the highest
    authored speed is a bug.
  - Reverse is a negative ratio and neutral is a ratio of exactly zero, both in the one ordered list
    a driver moves through — so selecting a gear is an index and never a mode plus an index.
  - A vehicle whose drivetrain `configure` would refuse is a trailer, not a failure: it instances,
    drives nothing, and ignores the throttle.
- 2026-08-01 — Added `render_golden`, the renderer's first regression oracle (RHI0, §5.6): it
  renders a fixed scene headlessly and compares the output against a checked-in reference, so a
  refactor can be shown behaviour-preserving.
  - Added `ISceneView::read_output`, a capability with a `false` default rather than an obligation:
    `ViewResources::read_output` owns its staging buffer and restores the resolve image's layout,
    which the graph tracks across frames.
  - Gave a golden a hash and a 32×18 thumbprint: an exact hash is a statement about one GPU, so the
    thumbprint reports a per-channel distance, where a level or two is rounding and tens is a pass
    that stopped running.
  - The set deliberately excludes sky and cloud (both switched off at the `Environment`) while the
    cloudscape is under active development. A golden that goes red every day is a golden people
    learn to ignore.
  - Added per-pass output hashing (`render/graph/pass_capture.cpp`), so a mismatch says which pass
    changed: being inside the graph is what makes it cheap, since it records where the copy left the
    image and the next barrier derives from it.
  - Shaped `PassCapture` exactly like `GpuProfiler` — one store per frame slot, `begin_frame(slot)`,
    `resolve(slot)` — because it has the same lifecycle for the same reason. `ISceneView` gains two
    more `false`-default capabilities.
  - Wrote down what capture covers: mip 0, the depth aspect of depth/stencil targets, and formats it
    can size; anything else is reported un-copyable and counted, and outputs past the staging budget
    are dropped, never truncated.
  - Capture adds `TRANSFER_SRC` to every transient, and usage is part of the pool's reuse key, so a
    captured frame aliases differently: `render_golden --no-capture` re-renders the shipping
    allocation against the same reference.
  - Fixed a latent undefined behaviour: the view's resolve image was created without
    `VK_IMAGE_USAGE_TRANSFER_SRC_BIT` while `read_output` copies out of it, which returned the right
    pixels on the driver it was written against.
  - 2026-08-02, from running it: fixed pass names being folded to one whitespace-free token on the
    way out but not on the way in, so every pass compared as both gone and new, and registered
    `se render --probe golden`.
  - Gave the temporal history pair `TRANSFER_SRC` only while capture is attached, so the TAA resolve
    is capturable without a shipping build paying for it: unconditional would risk lossless
    framebuffer compression in every frame.
  - Corrected two errors in the plan: `TraceCommandList` implements `ICommandList`, which is not
    born until RHI2, so it moves there; and "zero production code changes" is restated as zero
    change to rendering behaviour.
- 2026-08-01 — Added `SushiEngine::Execution`, the execution seam the ECS is now typed against
  (UHM0/RUNTIME-PORT0): the normative hazard semantic, `NodeDescriptor`, and the names a
  compile-time backend policy resolves.
  - Changed `World`, `Archetype`, `Chunk` and `Schedule` to name
    `Execution::Context`/`Graph`/`Buffer` instead of `SushiRuntime::API` types; `Schedule::run`
    returns `Execution::RunReport`, the subset every backend can answer for.
  - Added `execution/backend/runtime_backend.hpp`, the SushiRuntime implementation: a member-wise
    lowering of intervals to `Core::ResourceRegion` with no inference or caching of its own.
    Behaviour is unchanged.
  - Added the `SE_EXECUTION_BACKEND` build option (`runtime`, the default; `native`, unimplemented)
    which publishes the choice as a compile definition on the `SushiEngine` target.
  - Added `tests/functional/unit/test_hazard_semantics.cpp`, the conformance suite pinning the
    semantic's safety and determinism floors against the vocabulary rather than any one tracker.
  - Changed `Physics::RuntimeGraphBuilder`, the deepest runtime coupling site, to build its solve
    graph through the seam, so it names no runtime type: its ten node emissions go through one
    `emit_node` helper carrying each node's accesses.
  - Added the seam surface that retarget needed: `ElementRange`, `DeviceIndex`,
    `BackendCapabilities`, buffer sub-range intervals and host transfers, `Context::capabilities()`,
    `Context::set_work_migration()`, and `Graph::add_reduce()`.
  - Changed a late-bound `Provider` to copy its callable into inline storage rather than reference
    it, so a caller emitting nodes in a loop may pass temporaries; the callable must be trivially
    copyable, which a device backend requires anyway.
  - Changed the standalone solvers and the batch animation evaluator onto the seam too: their nodes
    declared data through a capture list, and each now declares its accesses explicitly and takes
    raw pointers.
  - Added `Math::{sqrt, fmod, floor}` to `core/types.hpp`, the one place a kernel names scalar
    maths, resolving to `sycl::` under a SYCL build and `std::` otherwise; its three consumers were
    the last `sycl::` calls in kernel bodies.
  - Removed `NodeKind::{Reduce, SegmentedReduce}`: a fixed-order fold takes two buffers and a
    combiner, not an access list and a per-element kernel, so it is a graph verb
    (`Graph::add_reduce`) rather than a node kind a descriptor could carry.
- 2026-08-01 — Added §9.6's soft-body collision (P6-E): soft-versus-rigid hands each contact to the
  existing `solve_manifold_positions`/`solve_manifold_velocities`, so friction and restitution are
  the code a crate on a floor uses.
- 2026-08-01 — Added §3.3's `ISoftBodyModel` seam and its three implementations —
  `FiniteElementModel`, `MassSpringModel`, `ShapeMatchingModel` — with §4.4's conformance suite
  asserting all three converge to the same rest shape.
- 2026-08-01 — Added §9.7's levels of detail (P6-F): coverage-based tier selection, a
  `RigidSoftBodyModel` floor tier, and a state transfer written in displacements rather than
  positions, so a rest pose crosses a boundary exactly.
- 2026-08-01 — Added §8.6's embedding kernel and deformed normals (P6-G):
  `Σ weight · tetrahedron_vertex` from the cooked binding table, with both invariants under test.
  Normals are area-weighted by construction.
- 2026-08-01 — Added `Render::DeformableMeshView`, replacing `ClothStrandView`: rows and columns
  describe a sheet, while a tetrahedral body's surface is a closed triangle mesh and a fractured one
  is not even connected.
  - Replaced the row-stride formula with `build_vertex_triangle_adjacency`, an explicit inverse map,
    which keeps the pass atomic-free: one thread per vertex gathering, rather than one per triangle
    accumulating through float atomics.
  - Removed the second compute dispatch entirely: indices upload verbatim and mesh-local into a
    host-visible buffer that doubles as the drawn index buffer, and the draw supplies `base_vertex`
    as its vertex offset.
- 2026-08-01 — Added `ISoftBodyService` and the soft-body entity (P6-G3): `SoftBodyParams` carries
  the cooked `.sushisoft` blob by value, because an entity that lost its blob has lost its body,
  unlike a cloth grid built from numbers.
  - Ran the soft solve before the rigid early-out, so a scene made only of soft bodies steps at all,
    and reported its own `soft_body_ms` rather than charging `solve_ms`: it is a host schedule
    outside the device composition.
  - Made §8.6's third invariant structural rather than promised: `extract()` reads the live
    particles, so there is no second copy to be a tick behind. The test pins both halves, since
    either alone passes trivially.
- 2026-08-01 — Added the editor's soft-body debug views (P6-G5), under Analysis ▸ Soft Body View:
  the tetrahedral wireframe and §9.3/§9.4's heat maps, since a shaded surface cannot show elements
  deep inside a beam that are past yield.
  - Scaled the heat maps against a material threshold rather than the body's own maximum:
    normalizing against the maximum makes something always red, answering which element is worst
    when the question is whether any is in trouble.
  - Split the reading from the drawing (`soft_body_heat.hpp`) so the part with a right answer is
    testable without a UI toolkit or a device. Shared edges are drawn once per owning element, since
    one colour would hide the boundary.
- 2026-08-01 — Generalized the constraint machinery from two bodies to N (P6-J1): a tetrahedron
  touches four particles, so colouring it as though it had two endpoints leaves the other two
  unprotected and the parallel sweep races.
  - Added `constraint_bodies.hpp`, the one place that knows how to read a constraint's bodies, as a
    customization point: a kind with more than two opts in by declaring `BODY_COUNT`, required
    rather than inferred, since inference races.
  - Named the N-body entry points apart (`assign_bodies`, `take_bodies`, `place_bodies` and the
    rest) rather than overloading the two-body ones, since a literal `0` is both a body index and a
    null pointer constant.
  - Made `take_bodies` validate every index before writing any, since a colour marked on some of a
    constraint's bodies is owned by nothing; `release_bodies` skips out-of-range indices instead,
    because releasing as much as possible is safe.
- 2026-08-01 — Registered the FEM element as a constraint kind in the device graph (P6-J2): both
  solvers gain `add_element`/`remove_element`, and the element takes its colour from the same
  colourer a distance constraint does.
  - Put both projections in one node per colour rather than two: within a colour no two elements
    share a particle, so splitting them would be two barriers where the material wants one
    Gauss-Seidel sweep.
  - Gave `FemTetrahedronT` its own Lamé pair: one device buffer holds every soft body's elements and
    a kernel captures nothing per body, so a material passed alongside would have become a
    per-element indirection into a table.
  - Made `PhysicsCapacities::elements` zero by default and opt-in: an element is several times a
    distance constraint's size and spent by the ten thousand, so a default share would make every
    scene carry megabytes for an unused kind.
  - Made both removal sweeps test all four vertices, P6-J1 restated at the removal end: testing only
    the first two would leave three quarters of the elements naming a freed slot, and a wrong
    element still projects perfectly well.
- 2026-08-01 — Added `examples/soft_body_budget` (P6-J3), the measurement P6's acceptance line was
  waiting on: a 15³ lattice of 20 250 tetrahedra stepped through `RuntimeGraphBuilder` at the 32
  substeps §13.1 names.
  - Reported the number and its caveat: mean 29.4 ms/tick against a 3 ms budget, but SushiRuntime
    found only a twelve-core CPU on this machine while §13.1 states its target against a
    desktop-class GPU, so the line stays open.
  - Made it a probe rather than a suite assertion, as `atmosphere_probe` already is: a test
    asserting 3 ms asserts the machine it runs on. The scene's shape is machine-independent, so that
    half is a conformance test.
  - Corrected both scenes' sizing of `PhysicsCapacities::elements`, a banded budget:
    `ConstraintStore` divides it evenly into one band per colour, so what must fit is the busiest
    band, bounded by a quarter of the particle count.
- 2026-08-01 — Added §9.1's bending constraint (P6-H), isometric rather than dihedral-angle: the
  textbook `acos(dot(n1, n2))` form divides by zero at the flat configuration every piece of cloth
  starts at.
- 2026-08-01 — Added §6.5's cosmetic `float` column (P6-I): `resolve_soft_body_precision` reads the
  cooked asset and the component flags, and participation in rollback overrides the cosmetic flag
  rather than being weighed against it.
- 2026-08-01 — Added a closing condition to the nest's cloud-top cooling
  (`cloud_top_equilibrium_depression`): the loss falls as the top cools below its environment and is
  gone at the depression, so the term is a flux rather than a sink.
- 2026-08-01 — Added the cloud's downwelling longwave to the surface energy balance, carried out of
  the extinction stage's column walk as a cloud-base temperature and liquid water path.
  `cloud_shade` widens from two channels to four.
- 2026-08-01 — Added `--cloud-top-floor` to `atmosphere_probe`.
- 2026-08-01 — Added `sky_coverage_pinned` to `atmosphere_probe`: the fraction of mirror columns
  whose low-band coverage is exactly 0 or exactly 1, the saturation detector for the frozen-sky
  diagnosis.
- 2026-08-01 — Added a cloud-top entrainment closure to the nest: a radiating deck now warms and
  dries itself across its inversion and can decay, so `cloud_top_equilibrium_depression` demotes to
  a runaway guard. Default raised to 0.8.
- 2026-08-01 — Changed `atmosphere_probe`'s step-cost budget line to §12's measured 12.3 ms
  sustained-clock figure; the retired 2 ms aspiration reported every run as OVER.
- 2026-08-01 — Fixed the cloudscape's four confirmed-by-eye defects (specks at low coverage,
  viewpoint-dependent faint cloud, seam at the near/far ring, flat far-field cloud):
  - The far window bakes supersampled (8×8 taps at the near carve scale, threshold-then-filter), so
    both windows carve the same clouds; the bake amortizes over 16 frames instead of a whole-volume
    hitch.
  - The far sun depth is sqrt-encoded across its 8-bit channel, spending resolution at shallow
    depths where shading still changes.
  - The nest carve scale adapts as 1/sqrt(coverage), so cloud count scales with coverage instead of
    cloud size.
  - The nest carve tapers its top against the level above, and the extinction interior profile is
    floored by the cell's cloud fraction so one-level decks stop shading as all-edge.
- 2026-08-01 — Fixed the cloud TAA's sky reprojection being blind to camera translation: the
  fallback reprojected the view ray as a direction at infinity, so a deck one to three kilometres
  away had no parallax and smeared.
- 2026-08-01 — Fixed clouds staying frozen on screen after `clouds.enabled` was switched off:
  `CloudCompositePass` sampled the TAA's pass-owned history unconditionally, and with both idle that
  history never changed.
- 2026-08-01 — Fixed `coverage_reference_lwc`'s default (1.5 → 0.4 g/m³): the bake divides by cloud
  fraction before normalising, so the reference must be in-cloud water, and 1.5 g/m³ is a value no
  real cloud reaches.
- 2026-07-31 — Added cloud-top longwave cooling to the regional nest, as the flux difference across
  a level so the scheme is conservative at any vertical resolution; `cloud_top_longwave_flux`
  carries it as data, and 0 removes the term.
  - Measured, it maintains nocturnal cloud rather than removing it, which is what makes nocturnal
    stratocumulus exist: the overnight fall is 15% where it was 40%.
  - Costs 0.096 ms in the `forces` stage, 1.6 % of the whole step.
- 2026-07-31 — Added a Weisman–Klemp relative-humidity ceiling to the nest's base state, without
  which the corrected vapour profile saturates near 9.5 km and every run starts under a global
  cirrus deck.
- 2026-07-31 — Added `--sponge-depth`, `--sponge-rate` and `--cloud-top-lw` to `atmosphere_probe`.

### Changed
- 2026-08-01 — Changed the nest's Rayleigh sponge from a quarter of the domain to half (lower edge
  13 → 9 km), which removes the tropopause oscillation the design doc has carried as open since
  2026-07-31. Measured over 72 h.
  - Measured the cause: the mode parked at 12.4 km, immediately below the old edge, where the ramp's
    weight is exactly zero, and doubling `sponge_rate` moved it only from ±13 K to ±11 K. Placement
    decides this, not strength.
  - The old sponge was not working inside itself either: at 14 km it carried 1.4 m/s and +3 K,
    against 0.11 m/s now.
  - Checked the weather below is untouched: the deck at 1585 m reads −13.98 K against −13.93 and
    cloud water agrees to three significant figures. A 6 km edge was measured too and is what
    overreach looks like.

### Fixed
- 2026-08-01 — Fixed soft-body contacts being generated only for surfaces already touching, so a
  body faster than its own thickness per tick passed through: a set built once per tick has to cover
  the whole tick, so it is speculative now.
- 2026-08-01 — Fixed a body marked `continuous` tunnelling through geometry the same body would have
  hit with the flag off: the swept pass replaced the tick's contact set each substep instead of
  adding to it.
- 2026-08-01 — Fixed cooked binding weights being applied to absolute positions without
  renormalizing: `float` weights sum to one only to about six digits, displacing a point by
  `|position| × 1e-7` — invisible at the origin.
- 2026-08-01 — Fixed §9.5's fracture leaving two pieces welded at shared vertices (P6-G): it needed
  no cooked adjacency, since two elements are on the same side of a crack exactly when they share
  three vertices including it.
- 2026-08-01 — Fixed `apply_fem_fracture` being declared `noexcept` while building several vectors,
  which would have turned an allocation failure into a termination.
- 2026-08-01 — Fixed the soft-body distance field being sampled at nearest-voxel only, which settled
  a surface onto a staircase and returned an unrelated normal, since a half-voxel central difference
  inside one voxel reads exactly zero.
- 2026-08-01 — Fixed the nest's longwave budget being one-way, which cooled a column without bound:
  a cloud-top loss independent of the cloud's own temperature, and a ground radiating to a clear sky
  the same deck shaded it from.
  - Isolated by running the same 72 h with the cloud-top term off: the surface holds +4.5 to +5.8 K
    across the whole run instead, at the same 0.84 cloud cover.
  - Verified on the same 72 h: net radiation goes −76.3 to +1.3 W/m² and the sensible flux −57.7 to
    −1.6, so the ground is in balance under an overcast noon sky, and the surface settles at −1.4 K
    against −21.7.

- 2026-07-31 — Fixed `humidity_scale_height` folding the base state's relative humidity rather than
  its mixing ratio, which decayed the profile twice and left the airmass far drier than documented —
  41% at 1.3 km against 62% now.
  - Changed the Meteorology panel's land-cover presets to set the airmass humidity along with the
    surface properties, because a semi-desert is not merely a dry surface: under a 70% airmass a dry
    surface still built a 2 km deck.

### Changed
- 2026-07-30 — Changed the skeleton debug draw and the transform gizmo to share one
  `project_to_screen` in `editor/core/viewport_projection.hpp`, replacing the identical copy each
  had grown.
- 2026-07-30 — Changed `Physics::mesh_mass_properties` to check the surface is closed before
  integrating: an open shell used to return the cone-fan volume, so a box missing one face reported
  five sixths of its mass rather than refusing.
- 2026-07-30 — Changed `Geometry::closest_point_on_triangle` to guard each edge region's division,
  so a triangle collapsed by welding returns a point on itself instead of a not-a-number.
- 2026-07-30 — Changed `Geometry::bake_signed_distance_field` to query a `MeshDistanceQuery` per
  voxel instead of sweeping every triangle, which is what makes a bake at the fidelity dial's upper
  resolutions affordable.
- 2026-07-30 — Changed `sim/PhysicsSimulation` to partition islands and put settled ones to sleep,
  which P2 built and never connected to the live tick.
- 2026-07-30 — Changed `runtime_graph_builder` to fold its motion maximum with the runtime's
  `Graph::add_reduce`, deleting the hand-built two-node reduction and its partial column.
- 2026-07-30 — Changed `PhysicsConfiguration::profiling` from a flag nobody read into the switch
  that decides whether per-stage timings are measured; the Physics panel's timing rows are no longer
  structurally zero.
- 2026-07-30 — Changed `PhysicsStageTimings` to drop `velocity_ms`: the device composition cannot be
  split per stage without a node label the runtime's public `add()` does not carry (§18 R8).
- 2026-07-30 — Changed `JointProjectionT` to leave a joint's load accumulators alone when neither
  body is simulated, so a settled hinge reports the load it last carried instead of zero.
- 2026-07-30 — Changed the glTF importers and cgltf's implementation unit from `render/material/` to
  a new `import/` module that links nothing, replacing the stopgap that compiled three renderer
  translation units into the test target.
- 2026-07-30 — Changed the Inspector (editor UX phase UX5) to edit the whole selection: a field
  shared by every selected entity writes to all of them, and one they disagree about draws as a
  mixed value rather than as the primary's.
- 2026-07-30 — Changed every Inspector component section (UX5) to carry a header context menu —
  Reset, Copy Values, Paste Values, Remove — over a type-erased per-component value clipboard.
- 2026-07-30 — Changed `apply_theme` (UX6) to apply one geometry scale and one accent hue over the
  base palette, so hover, active and disabled read as distinct states in all three themes.
- 2026-07-30 — Changed the toolbar's playback and transform-tool buttons (UX6) to drawn icons with
  live-binding tooltips, replacing the word labels.
- 2026-07-30 — Changed the render and post-process settings (UX6) to carry their units in the value
  format rather than the label, and gave 36 opaque sliders an explanatory tooltip.
- 2026-07-30 — Changed the status bar (UX6) to report the selection count, frame cost, and a
  clickable warning/error tally that opens the Console.
- 2026-07-30 — Changed the punctual light's field list to one definition in `render/lighting_panel`,
  used by both the Lighting panel's list and the Inspector's component section.
- 2026-07-30 — Changed `JointDesc` to endpoints plus a `JointParams`, so an assembly asset carries
  the joint vocabulary instead of a copy of it.
- 2026-07-30 — Changed `ConstraintStore::place` to try the next free colour when the assigned
  colour's band is full; disjoint constraints (joints) could previously reach only
  `capacity / colors` of the budget.
- 2026-07-30 — Changed joint break thresholds to read the peak substep load rather than the mean,
  which cancels to nothing across an impact.
- 2026-07-30 — Changed Ctrl+D/Delete/Cut/Paste to route through one deferred
  `pending_entity_command`, fixing a bug where Cut/Paste from a Hierarchy row corrupted that panel's
  own list walk.
- 2026-07-30 — Changed the Console to structured log lines (severity, timestamp) with per-severity
  filters, text search, and collapse-repeats.
- 2026-07-30 — Changed the Project panel filter to search the whole subtree (400-result cap) and
  made assets drag sources for material/decal texture slots.
- 2026-07-30 — Changed closing a dirty text tab to prompt Save / Don't Save / Cancel instead of
  discarding the buffer; added Unparent to the filtered Hierarchy's context menu.
- 2026-07-30 — Changed `ui/editor_panels.cpp` (editor UX phase UX4) from 7,157 lines to 725 by
  splitting it into per-domain translation units
  (scene/render/atmosphere/environment/project/vfx/scripting/input/core/ui-modals).
- 2026-07-30 — Changed panels (UX4) to share `track_item_undo`, `push_if_changed`, and
  `inline_rename_field` helpers instead of each keeping its own duplicated copy.
- 2026-07-30 — Changed `ViewportPanel::draw` (UX4) to take a `ViewportFrameInputs` struct instead of
  31 positional arguments.
- 2026-07-30 — Changed all panel function-local statics (UX4) to live on a shared `PanelState` or
  their own owning header.
- 2026-07-29 — Changed panel ownership to match domain ownership (UX2): atmosphere physics moved to
  Meteorology, sun/exposure/tier-readout each got one owning panel.
- 2026-07-29 — Changed `QualityParams` to drop resolved fields with no consumer (`max_particles`,
  `particle_sim_substeps`, `max_skinned_instances`, `bone_lod_bias`, `animation_influences`).
- 2026-07-29 — Changed the Toolbar from a dockable window to a fixed strip under the menu bar, and
  regrouped the Window menu into domain submenus.
- 2026-07-29 — Changed the editor frame order to submit the menu bar/toolbar/status bar before the
  dockspace, so the default layout bakes against the space it actually occupies.
- 2026-07-04 — Changed `Scalar` precision from a build-selectable option (`SE_SCALAR_DOUBLE`) to
  always `double`; moved rendering to camera-relative space.
- 2026-07-03 — Changed editor/engine naming to PascalCase namespaces and no-abbreviation
  identifiers; reorganized the editor layout.
- 2026-07-02 — Changed cloud noise generation to run on the GPU (Perlin-Worley shape, erosion
  detail).

### Added
- 2026-07-30 — Added the velocity half of speculative contacts (P5, §7.5): manifolds are generated
  out to the contact offset plus how far the pair can close this tick, so a body crossing a surface
  mid-tick has a constraint waiting.
  - Added `ContactProxy::speculative_margin`, taken from the same travel the broadphase sweeps its
    bounds by.
  - Changed contact events to keep their old touching test, so a speculative manifold is a
    constraint without reporting a contact — and an impact sound — from metres away.
- 2026-07-30 — Added a maximum depenetration velocity (P5, §7.6):
  `ContactSolveParams::max_depenetration` bounds how far a contact may push apart per substep, so a
  deeply-overlapping spawn is pushed out over several ticks.
- 2026-07-30 — Added `Geometry::analyze_mesh_topology` and `Geometry::repair_mesh`, the cooking
  pipeline's measurement and repair stage: welding, degenerate and duplicate removal, consistent
  winding, and a report of what changed.
- 2026-07-30 — Added `Geometry::MeshDistanceQuery`, a host bounding-volume hierarchy answering
  closest-point and signed-distance queries, plus `sample_surface_points`,
  `one_sided_hausdorff_distance`, and `max_protrusion_distance`.
- 2026-07-30 — Added `Physics::mesh_mass_properties`, exact mass, centre of mass and principal
  inertia integrated over a closed mesh with a Jacobi eigendecomposition to principal axes.
- 2026-07-30 — Added a `sushi_cooking` module holding the fidelity dial, the cook report and its
  thresholds, the `ICookingStage`/`IMeshCooker`/`ICookedAssetStore` seams, and the content-hash
  cache in memory and on disk.
- 2026-07-30 — Added `Cooking::CollisionCooker`, the five-stage rigid cooker: repair, mass
  properties, convex decomposition or a static triangle hierarchy, distance-field bake, and
  serialization.
- 2026-07-30 — Added the `.sushicollision` blob — `build_collision_blob`, `validate_collision_blob`,
  `load_collision_blob`, and the `collision_asset_hull`/`collision_asset_mesh` views that point
  straight into the loaded bytes.
- 2026-07-30 — Added the editor's Bake window (Analysis ▸ Bake): the fidelity dial with what it
  derives, live cook progress, per-asset cook reports for both asset kinds, and a Re-cook button.
- 2026-07-30 — Added `Authoring::CookBakeState`, the Bake window's UI-free model — the profile
  library, the worker, the filed cook reports, and the collider overlay's geometry.
- 2026-07-30 — Added `Cooking::collision_asset_wireframe` and `Editor::draw_collision_overlay`,
  which draw a cooked collider over the mesh it came from so "the collider is not the mesh" is
  visible rather than a number.
- 2026-07-30 — Added `IMeshCooker::cache_key`, so a Re-cook can evict the entry a content hash
  cannot see is stale — the case where the cooker changed rather than the mesh.
- 2026-07-30 — Added `Cooking::MeshPostProcessorChain` and `IMeshPostProcessor`, §8.1's ordered,
  registered import chain, with the collision and soft-body cookers registered as its shipped
  members.
- 2026-07-30 — Added `Cooking::ImportProfile` and `ImportProfileLibrary`: a per-project default plus
  a partial per-asset override, so one crate can be deformable without every rock paying for a
  tetrahedral mesh.
- 2026-07-30 — Added `Cooking::CookingService`, which runs the import chain on a worker thread and
  reports stage progress, so a dropped asset does not freeze the editor for the length of its cook.
- 2026-07-30 — Added `Geometry::import_gltf_mesh`, a glTF-to-`TriangleMesh` importer in the
  `import/` module, so the cooking pipeline starts at a file instead of at a mesh somebody already
  had.
- 2026-07-30 — Added `Cooking::SoftBodyCooker`, the six-stage soft-body cooker covering §8.3's ten
  steps: repair, voxelize and tetrahedralize, embed the render mesh, bake the rest shape, build the
  level chain, and serialize.
- 2026-07-30 — Added the `.sushisoft` blob — `build_soft_body_blob`, `validate_soft_body_blob`,
  `load_soft_body_blob`, and `evaluate_soft_binding`; it carries the cooking parameters that
  produced it, so a re-cook is reproducible.
- 2026-07-30 — Added `Cooking::build_tetrahedral_mesh` and `Cooking::embed_points`: surface
  rasterization, exterior flood fill, a conforming lattice, sliver removal, boundary extraction, and
  the barycentric embedding.
- 2026-07-30 — Added `Cooking::tetrahedron_quality`, the normalized element metric the sliver
  threshold and the cook report both read.
- 2026-07-30 — Added `Cooking::decompose_convex` and `Cooking::build_convex_hull_mesh`, an
  approximate convex decomposition driven by the measured protrusion it reports, with an exact
  small-point-set hull build underneath it.
- 2026-07-30 — Added the cooking pipeline's test suites (`Unit_CookingParameters`,
  `Unit_ConvexDecomposition`, `Unit_CollisionCooker` and six more), including the deliberately
  dirty-mesh corpus P4's acceptance criterion names.
- 2026-07-30 — Added glTF `weights` animation-channel import, so a clip's morph-weight tracks can
  come from an asset instead of only from hand-authored bytes.
  - Added `GltfAnimationImport::morph_target_names`, the mesh's target order the clip's tracks are
    matched against.
  - Added clip-driven morph weights to `AnimatedMeshPreview` (with a "Driven by clip" toggle in the
    Animator Preview window), replacing the manual-only seam.
  - Added `examples/assets/morph_face.gltf` and `Unit_AnimationMorphImport`, the animation stack's
    first tests under `tests/functional/`.
- 2026-07-30 — Added the animation stack's regression suites, which had no tests in
  `tests/functional/` at all — every phase had shipped with only an `examples/*_demo.cpp` behind it.
  - Added `Unit_AnimationClip`: the skeleton cook's topological reorder, both clip formats and their
    refusals, the sampling contract, and the compressed format's error bound on both easy and
    hostile content.
  - Added `Unit_AnimatorStep`: the state machine's semantics (exit time, typed conditions, a trigger
    consumed once, crossfades, events, root motion) and §0.2's byte-exact determinism and
    rollback-replay contract.
  - Added `Unit_AnimationBlendTree`: all five node kinds resolved from a compiled blob, swept for
    the unit-partition invariant, plus the resolver's capacity bound.
  - Added `Unit_AnimationLayers`: layer folding order, avatar mask weights and defaults, additive
    blending, and the additive bake's delta round trip.
  - Added `Unit_AnimationIk`: each solver's convergence and limits, §10's zero-weight-is-a-no-op
    claim, and the stranded-child and self-rotating-joint failure modes, including foot placement on
    raised ground and on a slope.
  - Added `Unit_AnimationRetarget`: the avatar's name mapping, the bind-pose-delta transfer between
    two differently-proportioned rigs, hip-height stride scaling, mirroring, and the runtime path
    agreeing with the baked one.
  - Added `Unit_AnimationControllerJson`: the Animator's JSON round trip, asserted by compiling the
    desc to a `.sushictrl` before and after and comparing the bytes, plus every enum's name round
    trip and the tolerant read's documented defaults.
  - Added `Unit_AnimationKeyframe`: the authoring curves' three interpolation modes, Catmull-Rom
    auto-tangents reproducing a straight line exactly, the bake against the curve at each frame's
    own time, and the recorder's round trip.
  - Added `Unit_AnimationAuthoringTail`: §12.4's remaining items — the motion-matching database and
    its crossfade driver, dual-quaternion skinning against closed forms, the ARKit-52 facial
    mapping, and the sequencer's event interval.
- 2026-07-30 — Added `ClipView::sample_morph_track`, which samples one morph track instead of all of
  them.
- 2026-07-30 — Added `ClipView::sample_generic_track`, the generic-track counterpart, so a caller
  with a fixed-size buffer can read within it.
- 2026-07-30 — Added `SequenceFloatTrack::sort_keys` and `SequenceTimeline::sort_tracks`, so a
  bulk-loaded timeline can meet the ascending-key requirement `sample` has always had.
- 2026-07-30 — Added four tests holding the incremental colouring to what determinism needs
  (validity, a function of the sequence, inside greedy's bound, colours released on removal) — the
  §17.5 risk row, written at last.
- 2026-07-30 — Added the negative case `Unit_ConvexManifold.CornerContactStaysOnePoint` claims in
  its own comment: a tilted hull lifted clear of the ground must produce no manifold.
- 2026-07-30 — Added `ui/component_editor.hpp`: pointer-to-member component fields that read a value
  from the whole selection to detect disagreement and write an edit back to all of it, plus the
  shared `component_header`.
- 2026-07-30 — Added the `PhysicsAssembly` asset (phase P3): a `.sushiassembly` blob of parts,
  joints and collision-filter groups, plus the instancing that turns one into rigid-body and joint
  descriptions.
- 2026-07-30 — Added the ragdoll wiring (phase P3): a capsule-per-bone rig built from a cooked
  skeleton, and the resolve that feeds `Animation::RagdollBlend` the object-space targets it has
  been waiting for.
- 2026-07-30 — Added the physics joint library (phase P3): seven joint kinds (fixed, ball, hinge,
  slider, distance, cone-twist, general), limits, drives, breakable joints, and `IJointService`.
- 2026-07-30 — Repaired the tests build lane, which had not linked since the editor UX overhaul
  (missing `sushi_sim` subdirectory for the test target, and two `-Werror` breaks in committed test
  files).
- 2026-07-29 — Added the second UX3 tranche: freeze frustum, real GPU-culling statistics, the
  Animator Graph driving the preview character, and a Load Character workflow.
- 2026-07-29 — Added Scene view fullscreen (Shift+Space, rebindable).
- 2026-07-29 — Added the first UX3 tranche: real autosave, a Physics Timings toggle, the Audio
  Authoring panel, File ▸ Open Scene/Open Recent, and a rebindable Ctrl+N.
- 2026-07-29 — Added `SimulationSettings`/`AtmosphereQuality` (UX2) and a derived Overall Quality
  toolbar preset.
- 2026-07-29 — Added a shared `environment_serializer` (UX2) so the scene file, default environment,
  and undo all read/write one environment shape.
- 2026-07-29 — Added persistent editor layout (UX1): per-user config dir, open-window set, gizmo
  tool/space, and Window ▸ Reset Layout.
- 2026-07-29 — Added Unity's playback shortcuts: Ctrl+P (Play/Stop) and Ctrl+Shift+P (Pause).
- 2026-07-29 — Added a `Climatology` seam, a baked `SET0` asset, and `se climatology bake`
  (atmosphere phase C, tasks #23–#25).
- 2026-07-29 — Added the global dynamical core as the source of the running climatology (atmosphere
  phase C, task #26).
- 2026-07-29 — Added seasons: the mean state now moves with the simulation's date (atmosphere phase
  C, task #27).
- 2026-07-29 — Added a read-only Mean State (T0) section to the Meteorology panel, closing
  atmosphere phase C (task #29).
- 2026-07-29 — Added persistent texture source paths for materials and decals.
- 2026-07-29 — Added undo coverage to the particle-system authoring panel.
- 2026-07-29 — Added `docs/design/editor_ux_overhaul.md`, the UX0–UX6 overhaul plan.
- 2026-07-29 — Added the global dynamical core as the driver of the regional nest, editor, and
  gameplay wind, deleting `SynopticLayer` (atmosphere phase C3).
- 2026-07-29 — Added `sushi_atmosphere` and T1, the global dynamical core: two-layer moist
  quasi-geostrophic flow with emergent cyclogenesis (atmosphere phase C2).
- 2026-07-29 — Added large-scale vertical motion (Ekman pumping) to the atmosphere forcing seam
  (atmosphere phase C1).
- 2026-07-28 — Added surface energy balance, cloud shading, and ice to the Meteorology panel, scene
  format, and `atmosphere_probe` (atmosphere phase B3e).
- 2026-07-28 — Added ice as a diagnosed phase to the regional atmosphere's microphysics (atmosphere
  phase B3d).
- 2026-07-28 — Added the cloud-shading feedback to the regional atmosphere (atmosphere phase B3b).
- 2026-07-28 — Added a prognostic surface energy balance to the regional atmosphere (atmosphere
  phase B3a).
- 2026-07-28 — Added domain-structure diagnostics and authored surface-heterogeneity scales to the
  regional nest.
- 2026-07-28 — Added the regional atmosphere to the render quality tier, resolved like every other
  pass.
- 2026-07-28 — Added per-stage GPU timing to the regional nest's step.
- 2026-07-28 — Added a subgrid cloud-fraction closure to the regional nest.
- 2026-07-28 — Added a "Render path" diagnostic section to the Meteorology panel.
- 2026-07-28 — Added land-cover presets to the Meteorology panel's surface forcing.
- 2026-07-28 — Added a Bowen-ratio readout to the Meteorology panel.
- 2026-07-28 — Added a domain-wide sky summary to `atmosphere_probe`.
- 2026-07-28 — Added the observer column's full vertical profile to the Meteorology panel.
- 2026-07-27 — Added a boundary-layer mixing scheme to the regional nest.
- 2026-07-27 — Added `atmosphere_probe`, a headless driver for the regional nest.
- 2026-07-27 — Added `Render::AtmosphereProfileLevel`, the observer column's unreduced vertical
  state.
- 2026-07-27 — Added atmosphere phase B2b: the regional nest's optical extinction drives cloud shape
  directly.
- 2026-07-27 — Added a Meteorology panel (`Window ▸ Meteorology`).
- 2026-07-27 — Added atmosphere phase B1: baked the cloudscape into camera-centred, non-wrapping
  windows.
- 2026-07-27 — Added atmosphere phase A: coupled the weather simulation to the sky.
- 2026-07-26 — Added editor UI for GTAO, SSR, ray-traced shadows, particle bursts, and blend shapes
  — engine features that were already implemented but unreachable from the Inspector.
- 2026-07-26 — Added physics broadphase/collision, animation authoring tools, particle mesh
  rendering, and UI subsystem expansions.
- 2026-07-25 — Added VFX particle material features: sprite textures through the bindless heap,
  flipbook atlases, soft particles, and per-emitter lit shading.
- 2026-07-25 — Added audio codec/HRTF/acoustics hardening: Vorbis and streaming-compressed codecs,
  MagLS/anthropometric HRTF, ray-traced acoustics, and an authoring DAW panel.
- 2026-07-25 — Added a Game view no-camera placeholder with an aspect/orientation/fullscreen
  toolbar.
- 2026-07-25 — Added audio ECS integration (phase S6): emitter/listener/reverb-zone components and a
  wall-clock snapshot extract bridging the ECS to the audio engine.
- 2026-07-25 — Added a reverb system and GPU-driven per-particle punctual lighting.
- 2026-07-24 — Added animation, audio, input, and VFX systems with full editor integration,
  including headless S0 vertical slices for audio/input and an action-mapped input layer.
- 2026-07-24 — Added an upscaler seam, scene-interop groundwork, and shader enhancements (Phase 11).
- 2026-07-23 — Added Saturn's rings to the analytic celestial-body sky.
- 2026-07-23 — Added Phase 6 lighting: three-cascade probe-volume GI (4/8/16 m spacing) and emissive
  materials that light the scene through GI.
- 2026-07-23 — Added lunar/solar eclipse light-loss dimming the whole scene.
- 2026-07-23 — Added the post-processing stack (Phase 9) and GPU-driven geometry (Phase 10):
  per-mesh-bucket instancing, indirect draws, and single-phase GPU occlusion culling.
- 2026-07-22 — Added a Vulkan 1.4 baseline: a background PSO optimizer and a descriptor-writer seam.
- 2026-07-22 — Added Phase 3 advanced passes: GTAO, Hi-Z, clustered lighting, and SSR.
- 2026-07-22 — Added Phase 7 atmosphere: the Hillaire LUT stack and volumetric fog.
- 2026-07-21 — Added the Phase 3 foundation: the quality-tier contract, SH-9 IBL, and
  `ViewResources`.
- 2026-07-21 — Added the render graph architecture, PBR materials, IBL, and TAA; added astro
  reference frames and gravity; added shader hot-reload, a disk-backed pipeline cache with
  pipeline-library linking, and a bindless descriptor heap.
- 2026-07-04 — Added PBR materials, a WGS84 planet, atmosphere, clouds, and stars.
- 2026-07-04 — Added collision shapes (Sphere/Plane/Box/OBB via SAT), soft bodies, a UI framework,
  two-way cloth↔rigid contact coupling, and a sweep-and-prune broadphase.
- 2026-07-03 — Added selection outlining and Hierarchy reordering.
- 2026-07-03 — Added primitive objects (Box/Sphere/Cylinder/Terrain), colliders, and cloth
  rendering.
- 2026-07-03 — Added SushiLoop M4 (loopback network layer) and M5 (cloth grids over the XPBD
  solver).
- 2026-07-03 — Added a scene-less editor start — the live world now starts empty instead of seeding
  demo cubes — plus unsaved-scene tracking and a confirm-before-close prompt.
- 2026-07-03 — Added Hierarchy multi-select; fixed the Project panel's double-click handling.
- 2026-07-03 — Added undo/redo and New Scene.
- 2026-07-03 — Added `.sushiscene` save/open.
- 2026-07-03 — Added hierarchy parenting via drag-and-drop.
- 2026-07-03 — Added a Unity-style Project window with double-click open and a relocated project
  root.
- 2026-07-03 — Added rotate/scale gizmos, Local/World space, wheel-zoom, and middle-mouse pan.
- 2026-07-03 — Added a Camera component with per-display selection; the Game view no longer picks.
- 2026-07-03 — Added a Preferences window with persisted settings.
- 2026-07-03 — Added net reconciliation wired into a live client/server demo.
- 2026-07-03 — Added cloth grids wired into `RuntimeSimulation` and the Inspector.
- 2026-07-02 — Added a translate gizmo for the selected entity.
- 2026-07-02 — Added GPU id-buffer picking with a selection highlight.
- 2026-07-02 — Added an entity-aware editor over the live world, with the world as the single source
  of truth.
- 2026-07-02 — Added the Game view as a second viewport driven from the world camera.
- 2026-07-02 — Added a live ECS world on SushiRuntime behind an `ISimulation` seam.
- 2026-07-02 — Added a Scene viewport with a Unity-style fly camera.
- 2026-07-02 — Added Vulkan presentation for the editor.
- 2026-07-02 — Added a Unity-style editor panel set.
- 2026-06-28 — Added the `se` developer CLI and the functional test suite.
- 2026-06-26 — Added a graph-coloured Projected Gauss-Seidel physics solver.
- 2026-06-26 — Added the WP-3 archetype ECS layer.
- 2026-06-26 — Added the milestone-A headless core.

### Fixed
- 2026-07-30 — Fixed root-motion rotation never being sampled: `animator_step` filled only
  `RootMotionDelta::position`, so a clip whose root turned moved the entity without turning it.
- 2026-07-30 — Fixed `sample_morph_state` overrunning its 64-entry scratch buffer for a clip
  carrying more than `MAX_MORPH_TARGETS` morph tracks; it now samples only the tracks a mesh's
  targets name.
- 2026-07-30 — Fixed `apply_generic_tracks` overrunning its 64-entry stack buffer, the sibling of
  the morph fix above: it clamped its dispatch loop to `MAX_GENERIC_TRACKS` while sampling every
  track the clip carried.
- 2026-07-30 — Fixed `skin_position_lbs` not being linear blend skinning: it blended rotation with
  `nlerp`, so it had no candy-wrapper collapse, and dual-quaternion skinning was measured against a
  baseline with no defect to remove.
- 2026-07-30 — Fixed `FootPlacementIk`'s comment describing a guard that was never there: the solver
  plants in both directions, which is correct, and an unreachable floor is handled by `TwoBoneIk`
  extending without stretching.
- 2026-07-30 — Fixed `SequenceFloatTrack`'s claim that `SequenceTimeline::evaluate` sorts its keys:
  it does not, and sorting there would put a copy and a sort on the per-frame path, so the
  requirement is stated instead.
- 2026-07-29 — Fixed authored pressure lows landing about three times deeper than a natural one:
  `inject_vorticity` now compensates each blob's circulation so the far-field pressure signature
  stays local.
- 2026-07-29 — Fixed the viewport image vanishing on a Scene-window resize; live drags no longer
  rebuild every frame.
- 2026-07-29 — Fixed `numpy` being a hard import-time dependency of the entire `se` CLI.
- 2026-07-29 — Fixed selecting a Render Quality tier silently rebuilding and restarting the weather
  simulation.
- 2026-07-29 — Fixed the scene file's environment block (sun, fog, GI, ~35 nest constants) being
  written but never read back on open.
- 2026-07-29 — Fixed the ray-traced shadow pass branching on the raw `RenderQuality` enum instead of
  resolved `QualityParams`.
- 2026-07-29 — Fixed `capture_scene`/`apply_scene` omitting lights, decals, and materials, so
  undo/save/Play-Stop silently discarded them.
- 2026-07-29 — Fixed copy/cut/paste dropping material, particle, audio, reference-frame, and
  surface-anchor data.
- 2026-07-29 — Fixed the Lighting panel recording one undo step per frame of a slider drag.
- 2026-07-29 — Fixed the synoptic layer's geostrophic wind being 735× too fast.
- 2026-07-28 — Fixed the atmosphere nest's thermal seed having no real length or time scale, tying
  weather structure to render tier and frame rate.
- 2026-07-28 — Fixed the atmosphere nest's query mirror missing nearly every completed readback
  during continuous stepping.
- 2026-07-28 — Fixed `max_steps_per_frame` capping the nest at one step per frame regardless of its
  setting.
- 2026-07-28 — Fixed the atmosphere nest's step landing inside the frame that reads it, dropping
  frames; the step is now staged and submitted at frame end.
- 2026-07-28 — Fixed the atmosphere nest's time step being pinned three times shorter than the CFL
  condition required.
- 2026-07-28 — Fixed the atmosphere nest spending 5 of 12 pressure-solve sweeps on convergence that
  had already been reached.
- 2026-07-28 — Fixed the atmosphere nest's pressure relaxation recomputing per column values that
  depend only on height.
- 2026-07-28 — Fixed the atmosphere nest's vertical diffusivity weakening exactly where the boundary
  layer needed it strongest.
- 2026-07-28 — Fixed the cloudscape bake eroding measured cloud density instead of shape, flickering
  thin decks.
- 2026-07-28 — Fixed the Meteorology panel's clock verdict and "Match sky to atmosphere" button
  assuming 60 fps.
- 2026-07-28 — Fixed the atmosphere nest's cloud base/top using a different threshold than its
  reported coverage.
- 2026-07-28 — Fixed the atmosphere nest's band coverage reporting optical opacity instead of
  coverage.
- 2026-07-28 — Fixed the nest's boundary-layer parameters not being serialized with the scene.
- 2026-07-28 — Fixed the atmosphere nest's thermal seed being an unbounded random walk, producing
  permanent ground fog.
- 2026-07-28 — Fixed a momentarily-stable atmosphere level decoupling entirely from the boundary
  layer's mixing.
- 2026-07-27 — Fixed `se build` not reconfiguring a stale `SE_BUILD_TESTS` cache setting.
- 2026-07-27 — Fixed `test_atmosphere_nest.cpp` and `test_weather_field.cpp` failing to compile,
  which had gone unnoticed because the suite wasn't being built.
- 2026-07-27 — Fixed the atmosphere nest's boundary layer never moistening due to half-float
  rounding.
- 2026-07-27 — Fixed the atmosphere nest advecting potential temperature without its stratification
  term.
- 2026-07-27 — Fixed the atmosphere nest's Coriolis parameter never reaching the GPU.
- 2026-07-27 — Fixed the atmosphere nest reading its pressure/divergence/surface-rain volumes before
  anything wrote them.
- 2026-07-27 — Fixed cloud coverage having almost no visible effect due to a mismatched shape-field
  assumption.
- 2026-07-27 — Fixed the far cloudscape window rendering as a featureless white square.
- 2026-07-27 — Fixed the atmosphere nest's buoyancy using total water vapour instead of the
  departure from the base state.
- 2026-07-27 — Fixed volumetric fog blinding the camera under an overcast sky.
- 2026-07-26 — Fixed remaining device-loss paths (LUTs, MRT blend, IBL descriptors).
- 2026-07-26 — Fixed functional CI by installing POCL, and fixed push-constant/image-layout
  validation errors.
- 2026-07-26 — Fixed CI flakiness by pinning the intel/llvm nightly to an older, more
  POCL-5.0-compatible build.
