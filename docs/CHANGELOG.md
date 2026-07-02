# Changelog

All notable changes to SushiEngine are documented here.

Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) — versions follow [Semantic Versioning](https://semver.org/).

---

## [Unreleased]

### Added
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
