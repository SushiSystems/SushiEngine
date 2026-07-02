# Changelog

All notable changes to SushiEngine are documented here.

Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) — versions follow [Semantic Versioning](https://semver.org/).

---

## [Unreleased]

### Added
- **Unity-style Project window.** The Project panel is now a two-pane file browser: a
  recursive folder tree on the left and a searchable icon-grid of the current folder's
  contents on the right, replacing the old single-list browser. Supports create
  (Folder / C++ Header / C++ Source / Text File), inline rename, delete, "Show in
  Explorer", and double-click open — text files (`.h`/`.hpp`/`.cpp`/`.txt`/etc.) open in
  the built-in text editor, everything else opens via the OS default application
  (`ShellExecuteW` on Windows). The default project root is no longer inside the engine's
  own source tree: it now resolves to `<user profile>/sushiengine/project` (created if
  missing) and is persisted as `last_project_root` in `Preferences` once chosen.
- **Rotate/scale gizmos with a Local/World axis toggle.** `GizmoController` now offers
  a full W/E/R handle set (translate, rotate, scale) plus a toolbar toggle for whether
  Translate/Rotate drag along world axes or the selection's own local axes (`GizmoSpace`);
  Scale always drags in local axes to avoid shearing a rotated object. The rotate ring
  drag now intersects the mouse ray with the axis's own plane through the pivot each
  frame and measures the signed world-space angle swept, instead of a screen-space
  angle — the previous screen-space approach inverted when the camera viewed the axis
  from its far side.

- **Camera component with display selection.** Cameras are now first-class ECS entities
  (a `Camera` component: fov, clip planes, `display_index`, `priority`, `active`), posed
  by their transform and appearing in the Hierarchy. `IWorldEditor` gained
  `create_camera` / `is_camera` / `camera_params` / `set_camera_params`, and `RenderScene`
  now carries the resolved camera per display (`display_cameras`) — the extract step
  picks, for each display, the active camera with the highest priority. The Game view
  has a display dropdown to choose which display it shows, so two or more cameras on
  different displays do not conflict, and the Inspector edits a selected camera's lens
  and routing. A seeded "Main Camera" entity replaces the old hardcoded game camera.
- **Editor Preferences with persisted settings.** An `Edit ▸ Preferences…` window edits
  General (Scalar precision, theme), Editor (autosave, camera move speed), and Scene
  (grid, snap) settings. Persistence sits behind an `IPreferencesStore` abstraction
  (`editor/preferences.hpp`) with a JSON implementation writing
  `<user-config>/SushiEngine/preferences.json` — the UI depends on the interface, not the
  file (dependency inversion). Live-effective fields (theme, camera speed) apply
  immediately; the precision control is compile-time, so it records intent and flags a
  rebuild when it differs from the running binary. Uses the newly declared
  `nlohmann-json` dependency (`cli/sushistack.deps.toml`, provisioned via `ss`).
- **Selectable Scalar precision (`SE_SCALAR_DOUBLE`).** The engine's `Scalar` (and the
  `Vec3`/`Mat4`/`Quat` built on it) can now be single or double precision, chosen at
  build time. It is a compile-time switch because `Scalar` is a typedef baked into
  trivially-copyable components and device storage, so it cannot flip at runtime. The
  option is declared in `cmake/ProjectOptions.cmake` and threaded as one compile
  definition on the `SushiEngine` INTERFACE target (and mirrored on `sushi_render`,
  which shares the value types across its interface structs but does not link the
  engine target), consumed at the single seam in `core/blas_placeholder.hpp`. The GPU
  upload path narrows to 32-bit explicitly, so shader data is unchanged in either
  build. The `se` CLI gained `--double` on `se build` and `se editor`, and a persisted
  `scalar_double` config field, so the choice survives across builds. Both precisions
  build clean and the ECS sandbox validates identically under each.
- **Translate gizmo in the Scene viewport.** The selected entity gets three world-axis
  handles (drawn over the viewport with ImGui's draw list, no extra Vulkan), and
  dragging one moves the entity along that axis, written straight back to its transform
  through `IWorldEditor`. Grabbing a handle never reselects (it takes priority over
  picking), and the mouse-pixel-to-world mapping is captured at grab time so the drag
  stays stable as the object moves. Combined with the Hierarchy's Add/Delete and the
  GameObject menu, the scene is now fully interactive: create, select, move, edit, and
  destroy entities.
- **Click-to-select in the viewport (GPU id-buffer picking) with selection highlight.**
  The scene view now draws a second `R32_UINT` colour target alongside the shaded image,
  writing each instance's picking id per pixel (the grid writes none), copies it to a
  host-visible buffer each frame, and exposes `ISceneView::pick(x, y)` to read the
  entity under a pixel back. A left-click in either viewport selects the entity beneath
  the cursor (right mouse stays reserved for fly navigation), kept in sync with the
  Hierarchy and Inspector. `ISceneView::render` takes the selected id and the mesh
  shader lifts the selected entity toward a warm highlight so it stands out.
  `MeshInstance` carries a 32-bit `id`; the push constant grew the draw id and selected
  id (120 bytes, still within the 128-byte floor) and now reaches the fragment stage.
- **Entity-aware editor over the live world.** The world is now the single source of
  truth for entities: the editor-side scene model (`editor/scene_model.hpp`) is gone,
  and the Hierarchy and Inspector operate directly on the simulation. The simulation
  seam grew an `IWorldEditor` read/write surface (`include/SushiEngine/sim/simulation.hpp`)
  — split from `ISimulation` so a panel that only edits depends on the narrow interface
  (interface segregation) — addressing entities by a stable `EntityId` with query
  (`entities`, `name`, `transform`, `color`, `visible`) and mutation (`create`,
  `destroy`, `set_name`, `set_transform`, `set_color`, `set_visible`) operations. The
  Hierarchy lists the world's entities with select, create, delete, filter, and inline
  / context-menu rename; the Inspector edits the selection's name, visibility,
  transform (Euler in the UI, quaternion in the world), and colour, writing straight
  through to the ECS components. Entities the editor creates carry no motion, so they
  stay where placed and stay editable while the world plays — only the seeded demo
  cubes are driven by the systems. `RenderInstance` now carries its `EntityId` (the
  key the upcoming viewport picking reads back), and the extract skips hidden
  entities. `World::get` gained a `const` overload for read-only host access.
- Editor **Game view**: a second Unity viewport rendering the live world from the
  world's own camera, alongside the Scene view's free-flying camera. The camera is
  factored behind an `ISceneCamera` seam (`editor/scene_camera.hpp`) with a
  navigable `FlyCameraSource` (Scene) and a world-posed `WorldCameraSource` (Game),
  so one `ViewportPanel` — now taking the camera by injection — serves both, and a
  new camera kind is a new implementation rather than a new panel. The two panels
  dock tabbed in the layout centre like Unity, and the Game panel joins the Window
  menu. `ViewportPanel` no longer owns a camera; the Scene fly camera and the Game
  world camera are host-owned and passed in.
- **Live ECS world driven on SushiRuntime**, behind a plain-C++ simulation seam.
  A new `sushi_sim` static library (`sim/`) owns a `SushiRuntime::API::Runtime`, an
  ECS `World`, and a `Schedule`, and drives a world of spinning, orbiting cubes —
  two systems (`spin` advancing orientation, `orbit` advancing position) over
  disjoint components, so the runtime's dependency tracker runs them in parallel,
  exactly as the sandbox proves. Every value a kernel reads is precomputed on the
  host into a component (the per-step rotation quaternion, the per-step orbit
  rotation as a cos/sin pair), so the kernels are pure arithmetic and capture no
  host state — legal device code. The library exposes only the plain-C++
  `ISimulation` seam (`include/SushiEngine/sim/simulation.hpp`: `ISimulation`,
  `create_simulation()`, `RenderScene`, `RenderInstance`, `CameraState`), which
  names no runtime, SYCL, or ECS type — the editor depends on the abstraction, and
  the runtime/SYCL/ECS stay contained in one translation unit. Each tick runs the
  schedule and an extract pass reads the shared-USM columns back on the host into
  the `RenderScene` the editor draws (a host copy today; device-shared interop is a
  later milestone). The editor (`editor/main.cpp`) creates the simulation, ticks it
  only while the toolbar is Playing (binding the existing `PlayState`), draws the
  extracted instances in the Scene viewport, and reports the live entity count in
  the Statistics panel. Turning `SE_BUILD_EDITOR` on now also builds `sushi_sim`,
  so the editor's final link is SYCL-aware and its executable ships the runtime DLL.
- Editor **Scene viewport** with a Unity-style fly camera. A dockable Scene panel
  shows a Vulkan-rendered 3D view — a ground grid and lit cubes — that the offscreen
  scene renderer draws and ImGui samples via `ImGui::Image`. Right-mouse enables
  free-look and WASD/QE flight with Shift to boost, frame-rate independent and active
  only while the panel is interacted with. New pieces: an offscreen scene-view
  abstraction (`include/SushiEngine/render/scene_view.hpp`: `ISceneView`,
  `CameraView`, `MeshInstance`) created by `IWindowRenderer::create_scene_view()`, its
  Vulkan implementation (`render/rhi/vulkan/vulkan_scene_view.*` — double-buffered
  colour+depth targets, a lit-mesh and a flat-line pipeline sharing one push-constant
  layout, drawn with 1.3 dynamic rendering and left shader-readable for ImGui), the
  mesh/line/grid shaders (`render/shaders/mesh.vert`, `mesh.frag`, `line.frag`), and
  the editor's camera/input seams (`editor/fly_camera.hpp`, `camera_controller.hpp`,
  `input_state.hpp`, `viewport_panel.*`). The Dear ImGui adapter gained
  `register_texture`/`unregister_texture` to expose the offscreen target as an ImGui
  texture. The Scene panel joins the Window menu and docks into the layout centre.
- Matrix and quaternion math on the single BLAS seam
  (`include/SushiEngine/core/blas_placeholder.hpp`, exposed via `core/types.hpp`):
  `Mat4`, `Quat`, and the vector/matrix/quaternion operations the renderer and camera
  need (`perspective`, `look_at`, `compose_transform`, `mat4_from_quat`, `mul`,
  `normalize`, `cross`, `dot`, …), documented as placeholders SushiBLAS will own.

### Fixed
- **Rotate gizmo direction flipping with camera angle.** Dragging a rotate ring while
  viewing the axis from the opposite side used to spin the object backwards; the drag
  math is now computed from world-space vectors (ray/plane intersection), which is
  camera-orientation independent instead of a screen-space angle.
- **Game view no longer selects objects.** Clicking in the Game viewport used to pick an
  entity through the id buffer, the same as the Scene view. Picking is now gated to the
  Scene view (a `pickable` flag on the viewport panel); the Game view is played, not
  authored, so a click there never changes the selection.

### Changed
- Editor now presents through Vulkan instead of OpenGL. The `se_editor` shell keeps
  its SDL2 window and Dear ImGui dockspace but renders through the engine's Vulkan
  renderer (`sushi_render`), so the same device that will draw the viewport also
  draws the UI. The change is layered behind three narrow seams so the app loop
  (`editor/main.cpp`) names no windowing or graphics API directly: a windowing seam
  (`editor/platform_window.hpp` `IPlatformWindow`, implemented by
  `editor/sdl_window.*`), the renderer's presentation facade
  (`include/SushiEngine/render/window_renderer.hpp` `IWindowRenderer` /
  `create_window_renderer()`, Vulkan implementation
  `render/rhi/vulkan/vulkan_window_renderer.*` — device + swapchain + frame sync,
  transparent swapchain rebuild on resize), and the Dear ImGui ↔ Vulkan adapter
  (`editor/imgui_backend.*`, the one editor component that speaks Vulkan). The RHI
  device (`render/rhi/device.hpp`) gained a `SurfaceFactory` hook and
  `required_instance_extensions` so the host creates the presentation surface without
  the renderer ever calling a windowing library, plus a `native_handles()` escape
  hatch the ImGui Vulkan backend uses. Turning `SE_BUILD_EDITOR` on now implies
  `SE_BUILD_RENDER`. The SDL2 dependency now requires its `[vulkan]` feature
  (`cli/sushistack.deps.toml`); the unused OpenGL dependency was removed.

### Added
- Renderer foundation (`render/`, `include/SushiEngine/render/`): the first cut of a
  greenfield Vulkan 1.3 renderer, gated behind the `SE_BUILD_RENDER` CMake option
  (OFF by default). Consumers program against an abstract RHI device
  (`render/rhi/device.hpp`: `IRenderDevice`, `create_render_device()`, `DeviceInfo`,
  `RenderDeviceDesc`) that carries no Vulkan types — the dependency-inversion seam a
  future D3D12/Metal backend slots into. The Vulkan backend (`render/rhi/vulkan/`)
  brings up a 1.3 instance and device (dynamic rendering + synchronization2) via
  vk-bootstrap and a VMA allocator, selecting the physical device by preference and
  exposing its UUID — the key a later milestone matches against SushiRuntime's SYCL
  device for zero-copy interop. It renders its first pixels: a one-shot offscreen
  triangle via 1.3 dynamic rendering (`render/rhi/vulkan/vulkan_offscreen.*`), whose
  shaders are compiled from GLSL to embedded SPIR-V at build time by a glslang host
  tool (`render/tools/shader_compiler/`, driven by the `sushi_compile_shader` CMake
  helper) so the renderer carries no runtime shader-compiler dependency. A headless
  `render_probe` target brings the device up, renders the triangle, and reads two
  pixels back to assert the pipeline path (center lit, corner cleared) — validating
  the whole chain (and vcpkg provisioning) without a display, the render analogue of
  the ECS sandbox. Like the editor, the renderer is a plain compiled target — no
  runtime link, no SYCL — so it builds on a stock toolchain. Vulkan, VMA,
  vk-bootstrap, and glslang are declared in `cli/sushistack.deps.toml` and provisioned
  by `ss install`. New `se` CLI command `se render` configures with the flag on,
  builds `render_probe`, and runs it.
- Editor panels (`editor/`): the `se_editor` shell grows a Unity-style panel set on
  top of its SDL2 + Dear ImGui dockspace — Hierarchy, Inspector, Project, a tabbed
  Text Editor, a Toolbar, a Console, and a Statistics panel, plus a status bar —
  laid out with a default dock split on first run. A **Window** menu toggles every
  panel, and each panel's title-bar close button shares that visibility bit so a
  closed panel reopens from the menu. The Hierarchy shows an editor-side scene tree
  with select, create, delete, a search filter, double-click / context-menu rename,
  and mouse drag-and-drop reparenting (drop onto a node to parent, onto empty space
  or via the "Unparent" menu to promote to a root); the Inspector edits the
  selection's name, visibility, and transform; the Project panel is a
  `std::filesystem` browser rooted at the working directory that opens text files
  into the editor; the Text Editor hosts open files as reorderable tabs with
  dirty-state tracking and save; the Toolbar drives a Play/Pause/Step playback state
  (a stub seam for a future World loop); the Console records editor actions; the
  Statistics panel reports frame time, FPS, and entity/document counts. The panels
  operate on an editor-owned `Scene`/`EditorContext` model (`scene_model.hpp`,
  `editor_context.hpp`), keeping the shell free of the runtime and SYCL, so this
  lane still builds on a stock toolchain. `imgui_stdlib` is vendored into the
  `sushi_imgui` library for `std::string`-backed text fields.
- Test suite (`tests/`): a GoogleTest-based functional suite mirroring
  SushiRuntime's layout, gated behind the `SE_BUILD_TESTS` CMake option (OFF by
  default; GoogleTest comes from vcpkg). One binary, `se_functional_tests`, built
  as a SYCL translation-unit set so kernels run against the real runtime — no
  mocks. Tests are grouped into CTest labels (`unit` / `integration` /
  `regression`) from the GTest suite-name prefix convention (`Unit_*` /
  `Integration_*` / `Regression_*`), so `ctest -L unit` selects a sub-suite.
  Coverage: World entity directory mechanics (spawn/get/destroy, generation
  invalidation, swap-remove repointing, structure versioning, queries),
  CommandBuffer deferral, graph-colouring properties, and end-to-end Schedule and
  PGS-solver runs checked against scalar references (`compile_count == 1`).
- Developer CLI (`cli/`): a Typer/Rich command-line tool installed as `se` (and
  `sushiengine`), mirroring SushiRuntime's `sr`. Core surface: `se project
  build/test/run/clean/doxygen`, `se config show`, and `se env dump`. It consumes
  the SushiRuntime sibling's bundled clang++ and vcpkg rather than provisioning a
  toolchain — when `cxx` / `vcpkg_root` are unset they resolve from the runtime's
  `dependencies/` tree, so a normal checkout needs no local config. Layered
  configuration (`cli/config.toml` -> `config.local.toml` -> `SE_*` env vars) and
  a cached MSVC-environment snapshot on Windows.
- Physics constraint solver (substrate-plan WP-3, physics half): a Projected
  Gauss-Seidel solver built on graph colouring. `color_constraints` edge-colours the
  constraints so each colour is a conflict-free batch (no two constraints in a colour
  share a body); `ConstraintSolver` compiles each colour into one parallel task and
  lets the runtime's dependency tracker order the colours into a sequential sweep —
  Gauss-Seidel across colours, parallel within one — repeated for the iteration count
  and compiled once. The solver is generic over the constraint type and its
  projection (a new constraint type is its own POD plus projection), with
  `DistanceConstraint` / `DistanceProjection` provided. The `pgs_demo` example solves
  a hanging chain and checks the device result against a scalar reference running the
  same colours in the same order (max error ~1e-6, compile_count == 1).
- ECS layer (substrate-plan WP-3): the entity / component / system core on top of
  SushiRuntime. Entities are generation-checked handles; components live in
  archetype chunks of structure-of-arrays columns, each column its own runtime
  allocation so the dependency tracker keys on it. A system declares its component
  reads and writes (`Read<T>` / `Write<T>`) and the `Schedule` emits one graph node
  per chunk; the runtime's dependency tracker orders systems by access — conflicting
  ones run in order, disjoint ones (and disjoint chunks) in parallel — so the engine
  writes no scheduler. Node iteration counts are late-bound to each chunk's live
  entity count, so the graph is compiled once and replayed while entities spawn and
  die. Structural changes go through a `CommandBuffer` applied at the frame barrier,
  with O(1) swap-remove destroys; new chunk sets are the only trigger for a rebuild,
  reported by the world's `structure_version`.
- `sandbox` target: the WP-3 worked example and single SYCL translation unit. A
  particle world (Position, Velocity, Mass, Lifetime) driven by `apply_forces`,
  `integrate`, and a parallel `decay_lifetime` system, spawning and destroying
  entities every frame, checked against an independent scalar reference with the
  graph compiled exactly once.
- `core/types.hpp`: the single integration seam for value types. It aliases a
  temporary placeholder today and is the one file to re-point at SushiBLAS when
  that library lands.
- Project governance and docs: `CONTRIBUTING.md`, `SECURITY.md`,
  `CODE_OF_CONDUCT.md`, and `docs/guides/ARCHITECTURE.md`, mirroring SushiRuntime's
  conventions.
