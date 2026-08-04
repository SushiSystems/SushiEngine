# Render

This file covers the presentation tier: the renderer's dependency-inversion boundary, the frame
graph behind it, the material/IBL, temporal and shadow machinery, and the lighting and sky the
frame is drawn with.

## 1. The render seam

Rendering does not belong inside the runtime — the runtime knows no graphics, just as it knows no
math. The renderer is a separate compiled library (`engine/presentation/render/`), a **greenfield
Vulkan 1.4** backend behind a dependency-inversion boundary so a D3D12/Metal backend can follow
without touching a consumer. The layering, from abstract to concrete:

- **RHI device**
  (`engine/presentation/render/include/SushiEngine/render/rhi/device.hpp`): `IRenderDevice` /
  `create_render_device()` carry no Vulkan types. `DeviceInformation` exposes the physical
  device's UUID — the key a later milestone matches against SushiRuntime's SYCL device for
  zero-copy interop. `RenderDeviceDescription` carries a `SurfaceFactory` hook and required
  instance extensions so a windowed host supplies its presentation surface without the renderer
  ever calling a windowing library; `native_handles()` is the single, explicit escape hatch a
  native-API adapter (the editor's ImGui Vulkan backend) uses.

- **Presentation facade**
  (`engine/presentation/render/include/SushiEngine/render/window_renderer.hpp`):
  `IWindowRenderer` / `create_window_renderer()` own the device and swapchain and drive the
  acquire → clear → submit → present cycle; a host opens a frame, records into the returned
  command buffer, and closes it. Swapchain rebuild on resize is internal. The Vulkan
  implementation is `engine/presentation/render/source/rhi/vulkan/vulkan_window_renderer.*`.

  `present_scene_view(view, slot, width, height)` (PLATFORM0 S4) is the other way a host ends a
  frame: it blits an `ISceneView` slot's resolve image straight onto the swapchain image —
  outside any dynamic-rendering scope, since the blit cannot record inside one — for a host with
  no other UI to draw over it (`sushiengine_player`, unlike the editor, which instead samples the
  scene view's texture through ImGui). A `WindowRendererDescription` with no `surface_factory`
  (PLATFORM0 S6) builds no swapchain at all: `begin_frame`/`present_scene_view`/`end_frame`
  become well-defined no-ops, and a host still gets a working device, asset library, and
  `create_scene_view()` — the offscreen-render, no-window shape `se player --headless` uses for
  CI.

- **Headless target** (`engine/presentation/render/source/rhi/vulkan/vulkan_offscreen.*`): the
  same device path without a window, used by `sushiengine_render_probe` to validate the pipeline
  in CI. A second, independent headless path is `IWindowRenderer` itself with no surface (above)
  — the one `se player --headless` uses, since it needs the same
  `create_scene_view()`/`assets()` surface a windowed player gets, not the offscreen probe's
  narrower one.

- **Scene view** (`engine/presentation/render/include/SushiEngine/render/scene_view.hpp`:
  `ISceneView`, created by `IWindowRenderer::create_scene_view()`): an offscreen camera view of a
  `MeshInstance` set (each tagged with a `MeshKind` — Box, Sphere, or Cylinder — plus per-kind
  shape params) plus a ground grid, drawn from a `CameraView`. The Vulkan implementation
  (`engine/presentation/render/source/rhi/vulkan/vulkan_scene_view.*`) is double-buffered — the
  frame being sampled by the UI is never the frame being drawn — and leaves its colour image
  shader-readable so the editor samples it with `ImGui::Image`. It exposes only the sampler/view
  handles a UI backend needs, never a full descriptor set.

  Alongside the shaded image it renders a second `R32_UINT` **id target** carrying each
  instance's picking id, copied to a host buffer each frame so `pick(x, y)` resolves a click to
  the entity under the cursor (GPU id-buffer picking); `render` also takes the selected id, which
  the mesh shader highlights, and an optional `DeformableMeshView` list, shaded on the GPU per
  frame and drawn pickable through the same mesh pipeline Box/Sphere/Cylinder use (see
  [cloth](domain-physics.md#12-cloth-sushiloop-m5)).

- **Lighting, materials, and the sky** (see
  [that section](#15-lighting-materials-and-the-sky)). `render` also takes a
  `const Render::Environment&` and the camera's world position, and draws the frame in three HDR
  passes rather than one, giving PBR meshes and a WGS84 planet with a physical atmosphere.

The editor composes these behind its own **windowing seam**
(`engine/foundation/platform/include/SushiEngine/platform/platform_window.hpp`
`IPlatformWindow`, SDL implementation `engine/foundation/platform/source/sdl_window.cpp`) and a
**Dear ImGui ↔ Vulkan adapter** (`applications/editor/source/ui/imgui_backend.*`) — the one
editor component that speaks Vulkan, kept apart from the app loop and panels so the rest of the
editor names no graphics API.

A single `ViewportPanel` (`applications/editor/source/ui/viewport_panel.*`) owns an offscreen
scene view and renders it from an injected camera — the `ISceneCamera` seam
(`applications/editor/source/camera/scene_camera.hpp`). Two implementations back the two Unity
viewports: a navigable `FlyCameraSource` (the **Scene** view) driving a fly camera
(`applications/editor/source/camera/fly_camera.hpp`) through a stateless controller
(`applications/editor/source/camera/camera_controller.hpp`) that reads a library-neutral
`InputState` (`applications/editor/source/input/input_state.hpp`) the panel fills from ImGui, and
a `WorldCameraSource` (the **Game** view) posed each frame from the simulation's camera.

`FlyCamera` and `CameraController` store and compute in `Scalar` (see
[the value-type seam](foundation.md#2-the-value-type-seam)), so the camera pipeline runs at the
same precision as the rest of the engine — `InputState` fields stay `float` (ImGui pixel deltas)
and are `static_cast`-ed to `Scalar` at the computation boundary. So the same panel serves both
viewports, the controller depends on no input source and stays unit-testable, and a new camera
kind is a new implementation rather than a new panel.

Interaction closes the loop: a left-click picks via the id target and the Scene view draws the
transform gizmo at the selection (`applications/editor/source/gizmo/gizmo_controller.*`, ImGui
draw list, projecting through the camera), so an entity is created from the Hierarchy, selected
in any viewport, moved with the gizmo, edited in the Inspector, and destroyed — all against the
one live world. `GizmoController` offers translate/rotate/scale (Unity's W/E/R) and a
`GizmoSpace` (Local/World) the toolbar toggles; Scale always drags local axes to avoid shearing a
rotated object. Rotate drags are computed by intersecting the mouse ray with the axis's own plane
through the pivot each frame and measuring the signed world-space angle swept between grab-time
and current plane vectors — a screen-space angle would invert once the camera crosses to the far
side of the axis, which is why translate/scale axes and the ray/plane math live in world space
rather than screen space throughout.

Editor and project settings sit behind a **preferences seam**
(`engine/world/authoring/include/SushiEngine/authoring/preferences.hpp` `IPreferencesStore`, JSON
implementation writing a per-user `preferences.json`). The Preferences window edits a plain
`Preferences` aggregate; the loop persists changes and applies the live-effective ones (theme,
camera speed). The precision setting selects the physics-solve precision of
[the value-type seam](foundation.md#2-the-value-type-seam) (a live runtime choice that rebuilds
the running simulation from a scene snapshot); the boundary `Scalar` itself is always double.

The **Project panel** (`applications/editor/source/project/project_panel.cpp`) is a two-pane file
browser over the on-disk project: a recursive folder tree and a searchable icon-grid of the
current folder, supporting create/rename/delete, "Show in Explorer", and double-click open (text
extensions open in the built-in text editor; anything else opens via the OS default application,
`ShellExecuteW` on Windows). The project root defaults to `<user profile>/sushiengine/project` —
outside the engine's own source tree — and is persisted as `Preferences::last_project_root` once
resolved, so authored project files never mix with engine source.

Scene persistence (`engine/world/serialization/source/scene_serializer.cpp`,
`ISceneSerializer`-free — it is two free functions, `save_scene`/`load_scene`, since there is only
one format) writes/reads a `.sushiscene` JSON file purely through `IWorldEditor`'s existing
query/mutate surface, so it adds no engine-side type. Parent links are stored as indices into the
saved entity array rather than raw `EntityId`s (ids are not guaranteed stable across a
destroy-and-reload); loading destroys every existing entity, recreates the file's in order, and
resolves parent indices in a second pass once all entities exist, so a child listed before its
parent in the file still resolves correctly. `File ▸ Save Scene`/`Save Scene As...`/`New Scene`
and the Project panel's double-click/`Open` on a `.sushiscene` file are the entry points;
`capture_scene`/`apply_scene` are the same functions `save_scene`/`load_scene` wrap around file
I/O, and are reused directly by undo/redo below.

One component does not fit the "write every field inline" rule: `SoftBodyParameters` holds a
whole cooked `.sushisoft` blob by value, megabytes of it, because nothing can re-derive a
tetrahedral lattice at runtime. So `capture_scene`/`apply_scene` take an optional
`Scene::ISceneBlobTable*`
(`engine/world/serialization/include/SushiEngine/serialization/scene_blob_table.hpp`). Given none
— the file path — the blob is written into the entry as base64, since a `.sushiscene` has to open
without whatever session wrote it. Given a table, the entry carries only the blob's FNV-1a 64
content hash and the bytes go in the table once, which is what makes `CommandHistory`'s
fifty-deep snapshot stack and the play-mode snapshot affordable: keying on content means the
table holds one entry per *distinct* cook, not one per snapshot. An entry whose asset resolves
neither way restores as an entity with no soft body rather than one holding an empty blob,
matching `create_soft_body`'s own refusal.

Undo/redo (`engine/world/authoring/include/SushiEngine/authoring/command_history.hpp` and
`engine/world/authoring/source/command_history.cpp`, `CommandHistory`) is whole-world
snapshot-based rather than a per-field command hierarchy: every step is a full
`capture_scene`/`apply_scene` round-trip, which is simple and correct at this entity count at the
cost of coarser granularity. Two recording modes cover the panels: `record()` snapshots
immediately before a discrete, single-frame mutation (create, delete, rename, reparent, a
checkbox toggle); `begin_change()`/`end_change()` bracket a continuous edit spanning several
frames (an Inspector slider held down, a gizmo drag) so it costs one undo step regardless of how
many frames it runs.

Panels call `begin_change` on the widget's activation edge (`ImGui::IsItemActivated()`) and
`end_change` on its deactivation edge (`IsItemDeactivatedAfterEdit()`); the gizmo does the same
off `GizmoController::dragging()`'s grab/release edge, tracked in the main loop since the gizmo
lives inside `ViewportPanel::draw()`. `Edit ▸ Undo`/`Redo` (Ctrl+Z/Ctrl+Y, ignored while
`ImGuiIO::WantTextInput` is set so a rename field's own text editing is not hijacked) drive it.
Because undo/redo swaps the whole world, entity ids are not preserved across the step, so both
clear the current selection rather than risk it aliasing an unrelated new entity.

`CommandHistory::revision()` is a counter bumped by every `record()`, committed `end_change()`,
`undo()`, and `redo()` — a cheap "has the world changed" signal a host can compare against a
stashed value without diffing snapshots. `EditorContext` stashes it as `saved_scene_revision` on
every successful New/Open/Save; `scene_is_dirty()`
(`applications/editor/source/core/editor_context.hpp`) is just the inequality of the two, and
backs both the status bar's `*` on the scene name and the close-confirm prompt below.

Ctrl+S and `File ▸ Save Scene` both go through `save_current_scene()`
(`applications/editor/source/scene/scene_commands.*`), which saves straight to `scene_path` if
set or opens the existing Save-As prompt if the scene has never been saved, so all three save
entry points (menu, shortcut, close-confirm) agree on when the scene becomes clean. Closing the
window (the title bar's X or `File ▸ Exit`) sets `EditorContext::close_requested` instead of
exiting directly; the main loop's `draw_exit_confirm_modal` lets the frame close immediately if
the scene is clean, otherwise prompts Save/Don't Save/Cancel, deferring to the Save-As modal
(tracked by `EditorContext::exit_after_save`) when Save has no path yet.

Live simulation state reaches the renderer through the **simulation seam**
(`engine/world/simulation/include/SushiEngine/simulation/simulation.hpp`): `ISimulation` /
`create_simulation()`, plain C++ that names no runtime, SYCL, or ECS type — only the value types
from [the value-type seam](foundation.md#2-the-value-type-seam). The concrete world lives in one
compiled library, `sushiengine_simulation` (`engine/world/simulation/`), the single place device
code exists outside an example: it owns a `SushiRuntime::API::Runtime`, the `Execution::Context`
built over it, an ECS `World`, and a `Schedule`, and starts with no entities — every archetype is
pre-reserved up front so the editor's own creates never trigger a mid-run chunk allocation, but
nothing is seeded into them.

Two systems over disjoint components (`spin` writes orientation, `orbit` writes position) are
registered for entities that carry `SpinStep`/`OrbitState`, which today only exist if authored
directly against `World` (no editor path attaches them); the dependency tracker still runs them
in parallel whenever such entities are present. Every value a kernel reads is precomputed on the
host into a component so the kernels are pure arithmetic that capture no host state — the
discipline that keeps them legal device code (see
[the ECS and the system graph](foundation.md#1-the-ecs-and-the-system-graph)).

This is dependency inversion at the largest seam in the engine: the editor links
`sushiengine_simulation` and depends only on `ISimulation`, so the runtime, SYCL, and ECS never
enter the editor's translation units, and a different world backend (or a headless stub) can
replace it without the editor changing. Because the editor links a SYCL library, its final link
is SYCL-aware and it ships the runtime DLL — the plain-toolchain lane is held by `sandbox` and
`sushiengine_render_probe`, not the editor.

Each `tick()` runs the schedule and an **extract** pass reads the world's shared-USM columns back
on the host (via `World::get`) into a read-only `RenderScene` (`RenderInstance` — an `EntityId` +
transform + colour — and the resolved cameras) the editor draws. Cameras are ECS entities too (a
`Camera` component: lens plus a `display_index`/`priority`/`active` routing), posed by their
transform; the extract picks, per display, the active camera with the highest priority into
`RenderScene::display_cameras`, and the Game view chooses which display it shows so two cameras
never conflict. `RenderScene::has_camera` reports whether any camera resolved at all;
`create_simulation()` seeds no demo entities — the live world starts empty, and
`default_camera()` exists only to give `RenderScene::camera` a well-formed value when
`has_camera` is false, not as something the Game view renders through.

`GameViewRenderPolicy::should_render`
(`applications/editor/source/core/game_view_render_policy.hpp`) is the one place that gates a
render pass on `has_active_camera && has_display`; when it says no, the Game window still opens
(it no longer skips `ImGui::Begin` and disappear) and shows a centered "No cameras rendering"
placeholder plus its toolbar instead of a render, via `ViewportPanel::draw_no_camera` — a method
on the same panel object as the render path, so the fullscreen state machine
(`apply_fullscreen_transition`) is one implementation instead of a member copy and a
function-static copy that could disagree about the dock slot to restore.

That toolbar — an aspect/resolution preset, a Landscape/Portrait orientation combo, and a
Fullscreen checkbox, held in `EditorContext::game_view_settings` (`GameViewSettings`,
`engine/world/authoring/include/SushiEngine/authoring/game_view_settings.hpp`) — is shared
between the no-camera placeholder and the normal render path so the row never drifts into two
implementations; when a preset constrains the aspect, `ViewportPanel::draw`
letterboxes/pillarboxes the rendered image within the panel instead of stretching it to the
panel's shape, independently of Fullscreen — which instead undocks the panel and expands it to
cover the whole editor viewport (Unity's "Maximize on Play"), restoring its dock slot when
unchecked.

The Scene view authors the world (pick, gizmo) and is the only place a selection is drawn
highlighted; the Game view is played, not authored, so it neither picks nor receives the Scene
selection. The editor ticks only while the toolbar is Playing (or on a one-shot `step_requested`,
set by the toolbar's Step button and cleared every frame), binding the existing `PlayState`.
Pressing Play captures the scene into `EditorContext::play_mode_snapshot` via `capture_scene`;
pressing Stop re-applies it via `apply_scene` and clears the snapshot, so play-mode mutations
(spawns, destroys, transform/physics edits) never leak into the edited scene, mirroring Unity's
edit/play-mode separation — this reuses the same `capture_scene`/`apply_scene` round-trip
`CommandHistory` already relies on for undo/redo, rather than a second snapshot mechanism. The
extract is a host copy today. A later interop milestone promotes it to a device-shared sink
pinned to a render thread, so the scheduler can overlap the next step's simulation with the
current step's draw and skip the round-trip.

All of SushiEngine's built-in ECS components — `Transform`, `Orientation`, `SpinStep`,
`OrbitState`, `Tint`, `Camera` — are declared in one place,
`engine/world/simulation/include/SushiEngine/simulation/components.hpp`, rather than inline in
`runtime_simulation.cpp`; component registration order across translation units must agree (see
[the ECS and the system graph](foundation.md#1-the-ecs-and-the-system-graph)), so keeping the
canonical set in one header is what makes that guarantee easy to keep as more consumers are
added.

Transform + Orientation are mandatory on every entity; Tint (the Renderer component) and Camera
are independently pluggable per entity — Unity-style add/remove — through
`IWorldEditor::set_has_renderer`/`has_renderer` and `set_is_camera`/`is_camera` (distinct from
`create_camera`, which spawns a fresh camera entity). The ECS has no in-place component
add/remove — an entity's component set is fixed by its archetype — so a toggle is implemented as
a migration: destroy the entity and respawn it into the archetype matching the new component set,
carrying over Transform/Orientation and any surviving Tint/Camera value
(`RuntimeSimulation::migrate_components`). Seeded, animated demo cubes (`SpinStep`/`OrbitState`)
are exempt from migration — their component set is fixed for the demo.

The **world is the single source of truth for entities** — there is no separate editor-side scene
model. The editor reads and writes it through `IWorldEditor`, split from `ISimulation` so a panel
that only inspects or edits depends on the narrow surface (interface segregation): entities are
addressed by a stable `EntityId`, queried (`entities`, `name`, `transform`, `color`, `visible`,
`has_renderer`, `is_camera`) and mutated (`create`, `destroy`, `set_name`, `set_transform`,
`set_color`, `set_visible`, `set_has_renderer`, `set_is_camera`). Transform, colour, and the
Camera lens are real ECS components the surface writes through; names, visibility, and parenting
are host-side editor metadata the simulation keeps beside each entity's handle
(`parent`/`set_parent`). Editor-created entities carry no motion components, so the spin/orbit
systems never match them and they stay authorable while the world plays — only the seeded demo
cubes are system-driven.

The Hierarchy renders these entities as a tree (drag-and-drop reparents; dropping on empty space
unparents to root), guarded against cycles by walking the candidate parent's own ancestor chain
before accepting a drop, and the Inspector edits the selection — including, for Camera and
Renderer, an "x" on the header to detach the component and an "Add Component" menu offering
whichever is missing; the editor GUI goes through Dear ImGui. `EditorContext` splits selection in
two: `selected_entity` is the single "primary" target the Inspector, viewport gizmo, and
Align/Move-to-View act on, while `selected_entities` is the Hierarchy's full multi-selection
(Ctrl+click toggles membership; Shift+click ranges from `selection_anchor` — the last plain or
Ctrl click — over the tree's depth-first display order, or the filtered order when a search
filter narrows the list). A plain click collapses both back to one entity
(`select_only`/`toggle_selected`/`is_selected` in
`applications/editor/source/core/editor_context.hpp`); `Delete` acts on the whole vector.

Because parenting is host metadata rather than an ECS `Parent` component, both the extract pass
and a reparent walk the parent chain on the host (`RuntimeSimulation::world_transform`, bounded
by the live entity count against a corrupt chain) rather than in a kernel — the same
host-copy-first posture as extract itself, revisited only if parenting needs to affect systems
running on the device. World pose is composed as a shear-free hierarchical TRS chain rather than
a general `Matrix4` product (`world_scale = parent_scale * local_scale`,
`world_rotation = parent_rotation * local_rotation`,
`world_position = parent_position + parent_rotation ∘ (parent_scale * local_position)`, matching
Unity's model) precisely because that form is invertible: `set_parent` uses the inverse to
recompute the child's local transform at the moment of reparenting, so its resolved world-space
pose is unchanged by the move rather than being reinterpreted (and visibly jumping) in the new
parent's space.

### 1.1. The render graph

`ISceneView`'s Vulkan implementation is not a sequence of hand-recorded passes but a **frame
graph**. Each frame the scene view builds a `Frame::FrameContext` — camera, extent, quality tier,
draw list, and the handles of this frame's targets — and asks each pass to register itself; the
graph then derives everything that used to be written by hand.

The quality tier does not reach the passes raw. Once per frame the scene view runs
`resolve_quality` (`engine/presentation/render/source/frame/quality.cpp`, public type
`QualityParameters`), which turns `RenderQuality` into the concrete parameters passes actually
read — soft-shadow tap counts, contact-march length, cloud budget, the coarsest variable-rate
tile, the shadow atlas size and cascade count, and which advanced BRDF lobes are evaluated. The
policy lives in that one file so the tier cannot mean one thing in the shadow pass and another in
the cloud pass; a pass reads resolved parameters, never the enum. The authored settings are the
High baseline, so High resolves to the request verbatim and a lower tier scales the expensive
half down from it.

- **`engine/presentation/render/source/graph/`** — `RenderGraph`, `RenderPassBuilder`,
  `PassContext`, `TextureHandle` / `BufferHandle`, and the access vocabulary. A pass declares
  *what* it touches (`read`/`write`/`color_attachment`/`depth_stencil_attachment`) and the graph
  derives *how*: `resource_state.*` maps each declared access to exactly one (stage, access mask,
  layout) triple, so every `VkImageMemoryBarrier2` and every `vkCmdBeginRendering` scope —
  viewport and scissor included — is generated, never authored.

  `compile()` culls passes whose outputs nothing reads, then walks the schedule assigning
  physical resources: a transient is returned to its pool the moment its last reader is
  scheduled, so two disjoint lifetimes land on one allocation. That is the graph's memory
  aliasing.

  `pass_capture.*` is the graph's debug instrument, absent unless something attaches it: it
  copies out every texture a pass wrote and hashes it on the host, so a golden mismatch can name
  the pass rather than only the frame. It lives inside the graph because only the graph tracks
  image layouts across passes — a capture outside it would have to restore what it changed, while
  one inside simply records it.

- **`engine/presentation/render/source/resources/`** — `TexturePool` / `BufferPool` (the physical
  backing, one set per frame slot so a pool never hands this frame a resource the previous
  frame's submit is still reading), `DescriptorAllocator` (per-slot pools reset wholesale, so a
  resize rebuilds no descriptor set), `DescriptorHeap` (the bindless update-after-bind array
  bound as set 1), `PipelineCache` (a `VkPipelineCache` persisted to disk) and
  `GraphicsPipelineFactory` (four independently cached `VK_EXT_graphics_pipeline_library` halves,
  with monolithic creation as the fallback when the extension is absent), `SamplerCache`, and
  `ShaderLibrary`.

  A pipeline never makes a pass wait on its own best version: `GraphicsPipelineFactory` hands out
  a `PipelineHandle` pointing at the fast-linked (GPL) pipeline the instant it exists, while a
  background thread rebuilds the same pipeline monolithically and swaps the handle's atomic
  pointer (release/acquire) once it is ready; the superseded pipeline retires after a delay sized
  past every view sharing the factory's clock, the same reasoning `TextureLibrary`'s streaming
  retirement below uses. `DescriptorWriter` and `bind_descriptor_set()` are the matching
  write/bind seam every pass, the compute/RT passes, and the scene layout route through, so the
  announced `VK_EXT_descriptor_heap` lands as a swap behind these two functions rather than a
  sweep of every pass.

- **`engine/presentation/render/source/passes/`** — one file per pass, each honouring the same
  `register_pass(graph, frame)` contract and owning only its own pipelines: the environment
  capture, the opaque geometry pass, the shading-rate mask, the sky pass, the cloudscape
  field/light-volume/shadow-map bakes, the half-resolution cloud march, the cloud composite, the
  temporal resolve, the **post-processing stack** (depth of field, motion blur, auto-exposure
  metering, and bloom), the display transform, the spatial anti-aliasing filter, and the picking
  readback. Adding an effect is adding one of these and registering it; no neighbouring pass
  changes. A pass that this frame's settings do not call for registers nothing, so the chain
  reconfigures without any pass learning what the others do.

- **The post-processing stack** runs after the temporal resolve: `DofPass` and `MotionBlurPass`
  (gather-based, tier-gated) each hand their output to the next stage; `AutoExposurePass` builds
  a luminance histogram the scene view reads back to adapt the exposure; `BloomPass` builds a
  Karis-averaged mip pyramid into a half-resolution target; and `TonemapPass` is the single
  display transform that applies the resolved exposure, a colour grade, one of three tone curves
  (AgX / ACES / Khronos Neutral), the lens effects, and the sRGB encode with a blue-noise dither.
  All of them read one `PostProcessUniforms` block (scene-set binding 31) the scene view fills
  from `RenderSettings::post`, which the editor's **Post Process** window authors — the passes
  never name the editor.

- **The GPU-driven geometry path** (Phase 10) replaces the CPU's one-draw-per-instance loop with
  two device buffers and a cull dispatch, so the CPU cost is flat in the number of distinct
  meshes rather than the number of instances. `InstanceSystem`
  (`engine/presentation/render/source/scene/`) packs every opaque mesh instance into a per-frame
  `GPUInstance` storage-buffer record — camera-relative transform, bounding sphere, and the
  material/motion/pick indices the classic draw used to push — and groups them by mesh into
  per-mesh buckets, one host-mapped buffer per frame slot in the exact shape `MaterialSystem` and
  `MotionSystem` already use.

  `CullPass` then runs before the depth prepass: one thread per instance tests the bounding
  sphere against the view frustum, its own on-screen diameter (a screen-coverage LOD gate that
  drops instances too small to matter), and the occlusion pyramid, then compacts the survivors
  per bucket and writes one `VkDrawIndexedIndirectCommand` per bucket whose instance count it
  decides — no CPU readback in the loop (a survivor counter is read back one frame late only for
  the editor's cull statistics).

  `OcclusionPass` owns that pyramid: a persistent max-Z (farthest-depth) mip chain built after
  the depth prepass, the conservative twin of the `HiZPass` nearest-depth pyramid the SSR trace
  marches (nearest is right for reflections and wrong for culling). It lives outside the render
  graph and is read at the *start* of the next frame by the cull — reprojected with the previous
  view-projection and the eye delta — so an instance is tested against the depth the last frame
  actually rendered; a freshly (re)created image clears to "far" so nothing occludes until real
  depth lands, which self-corrects with no popping and no readback.

  The draw itself runs through `mesh_gpu.vert`, the twin of `mesh.vert`: it reads the model
  matrix, material index, and picking id from the instance record (an indirect draw carries no
  push constant), indexing the cull pass's compacted survivor list from the one value still
  pushed per bucket — the bucket's base into that list. Its instance and compacted buffers ride a
  **set-2** descriptor set (`SceneLayout::INSTANCE_SET`), so the bindless heap keeps set 1 and
  both vertex shaders feed the same `pbr.frag`.

  The whole path is **two-path**: the scene view takes the GPU-driven route when the tier permits
  it (`QualityParameters::gpu_driven` — off on Low, on for Medium/High/Ultra), the author has
  left `GPUCullingSettings::enabled` on, the bindless heap is present, and nothing is selected;
  anything else falls back to the classic CPU per-instance draw (a selection keeps it so the
  outline's stencil mask still works), while the cull machinery stays primed so the pyramid
  remains fresh for when it resumes. The editor's **GPU Culling** window authors
  `RenderSettings::gpu_culling` — enable, frustum, occlusion, min-screen-diameter, a debug
  frustum freeze, and the per-frame statistics — and, as with post-processing, no pass names the
  editor.

- **Shadows beyond the atlas** (Phase 12.3). The punctual shadow atlas holds a fixed number of
  tiles, and a light that does not fit one used to shade unshadowed. It now gets shadowed
  stochastically instead: `clustered_lighting.glsl` splits a cluster's lights into those with a
  tile (filtered against it as before) and those without, importance-samples a tier-scaled few of
  the latter per pixel, and marches the GI distance clipmap toward each with `sdf_visibility()`,
  weighting by one over the probability it was picked. Because that estimator is unbiased, the
  temporal resolve is the denoiser — no new pass, no new history.

  The field is the one `SDFProbeTracer` already builds for probe GI, offered through
  `IProbeTracer::visibility_field()` so the shading pass depends on a field existing and never on
  which tier produced it; it reaches every shading pipeline through the bindless heap's volume
  array, because the per-frame push set is full at its guaranteed 32 bindings. The consequence
  worth stating plainly: the number of shadowed lights stops being a memory budget and becomes a
  sample budget.

- **Two queues, one schedule** (Phase 11). The graph does not assume a single queue. A pass may
  declare `PassQueue::AsyncCompute`, and `compile()` splits the schedule wherever the queue
  changes into `Submission`s — one command buffer each, recorded and submitted in order. What
  orders them is derived, like the barriers, from the resource declarations: a submission waits
  on the latest earlier submission *on the other queue* that produced what it consumes or
  consumed what it overwrites (a dependency on its own queue is already ordered by submission
  order, and per-queue timeline values rise monotonically, so one wait covers every earlier one).

  The same walk marks which resources both queues touch, and only those are allocated with
  concurrent sharing (`TextureDescription::cross_queue`) — the graph cannot transfer queue-family
  ownership, and paying for concurrent sharing on every transient would cost attachment
  compression for nothing. Two conditions a flagging pass owes the graph: everything it produces
  must be *declared* (a pass that hand-barriers a resource it owns would leave its consumers
  unsynchronised), and what it shares must be a graph transient rather than an import, whose
  sharing mode the graph cannot change. Flagged today: the clustered light cull and the GTAO
  horizon march. Gated three ways — a compute queue family distinct from graphics,
  `QualityParameters::async_compute` (off on Low), and `FrameDeliverySettings::async_compute` —
  and with any of them absent every pass records on the graphics queue exactly as before.

- **Frame delivery** (`ViewResources`). Each queue carries one monotonic **timeline semaphore**:
  every submission signals its own value, waits on the value it depends on, and a frame slot is
  reusable once both timelines have reached what that slot submitted. Command buffers are handed
  out per submission, not per slot, so a frame that compiles to several submissions never records
  two of them into one buffer. How far the CPU may run ahead
  (`FrameDeliverySettings::frames_in_flight`, 2 or 3) and how frames are paced onto the display
  (`PresentMode`, applied through `IWindowRenderer::set_present_mode`) are settings; all slots are
  allocated up front, so changing depth costs an idle and no reallocation. The editor's **Frame
  Delivery** section authors the block.

- **Reconstruction is an interface**
  (`engine/presentation/render/source/frame/upscaler.hpp`). Rendering below the output extent and
  reconstructing back up is a contract — colour, depth, motion, history, jitter, exposure, and
  the two extents — that `TAAPass` is simply the first implementation of. A vendor upscaler
  (FSR/DLSS/XeSS) lands as another implementation rather than as a fork of the frame loop;
  `Frame::upscaler_availability` reports which backends this build carries, and one it does not
  resolves back to the built-in temporal reconstruction with the reason surfaced in the editor.

- **`engine/presentation/render/source/interop/`** — a device-local buffer whose memory another
  API can import by OS handle: a dedicated, exportable allocation stamped with the device UUID
  that `RenderDeviceDescription::required_uuid` already selects the graphics device by, exposed
  through the public
  `engine/presentation/render/include/SushiEngine/render/interop.hpp` with no Vulkan, SYCL, or
  platform type in sight. The renderer *exports* only; importing belongs to whoever owns the
  other API, which for SushiRuntime means the runtime — the dependency points one way.

- **`engine/presentation/render/source/scene/`,
  `engine/presentation/render/source/geometry/`,
  `engine/presentation/render/source/textures/`** — the shared scene uniform block and
  descriptor/pipeline layout, the built-in unit meshes and the per-slot soft-body buffers, and
  the cloud noise volumes. The noise set (two Perlin-Worley volumes, an anisotropic cirrus
  volume, a weather map, and the view march's precombined carve volume) is generated by compute
  dispatches at bring-up rather than on a CPU thread pool.

  The carve volume alone carries a mip chain, box-filtered by a compute pass so it wraps the way
  the sampler does, because it is the only one sampled at a world scale fixed in metres by a
  march whose step grows with distance — and the same bring-up submit reads its finest level back
  so the host can measure, per level, the variance the filter removed. The march needs that
  number: its coverage threshold is a percentile of a field that is uniform only before
  filtering, so a coarse fetch has to be thresholded against the spread it lost rather than
  against its mean.

Two supporting mechanisms make the graph usable day to day. `GPUProfiler` brackets every executed
pass with timestamp queries and resolves a slot's results at the point its fence has already been
waited on, so per-pass GPU times cost no stall; `ISceneView::pass_timing_count()` /
`pass_timing()` surface them, the main loop copies each visible viewport's breakdown into the
editor context, and the Statistics panel lists them, which is what lets every later pass be
landed against a measured budget.

`ShaderLibrary` ships build-time SPIR-V but also watches `engine/presentation/render/shaders/`
when it exists: an edited shader is recompiled in process with glslang, the device is idled, and
every pipeline is rebuilt — a compile error leaves the previous module in place and reports on
stderr.

`VulkanSceneView` is left as the orchestrator: build the frame context, register the passes,
compile, submit. It records no barrier and opens no render pass of its own. The precision
invariants are unchanged and inherited by every pass — camera-relative rendering (the eye
subtracted in double before the float cast) and reverse-Z.

### 1.2. Materials, textures, and image-based lighting

The vertex format carries position, normal, tangent, two UV sets, and a vertex colour — the
minimum that makes normal mapping, parallax, and a detail set possible, and therefore the thing
everything else in this section rests on. A zero tangent is a legal value meaning "none
authored"; the shader falls back to screen-space derivatives, so an imported mesh without
tangents still normal-maps.

**The material** (`engine/domain/material/include/SushiEngine/material/material.hpp`) is the
authored surface: albedo, packed metallic-roughness (ORM supported), normal, height, occlusion,
emission, a Unity-style detail set, per-set tiling/offset, the advanced lobes (anisotropy,
clearcoat, sheen, transmission), and the rendering state (surface type, cull mode, blend mode,
render queue, shadow flags, sampler settings). Every map is optional. That is not a convenience:
an unset slot resolves to one of four neutral default textures — white, flat normal, black,
neutral MR — so the shader samples unconditionally and branches only on behaviour a default
*cannot* stand in for, which travels as flag bits. A material with no textures shades exactly as
it did before textures existed.

**`engine/presentation/render/source/material/`** holds the machinery:

- `AssetLibrary` — the device's shared store. Textures, meshes, the bindless heap, the shader
  library, and the pipeline and sampler caches are device-level, so two viewports drawing the
  same model share one upload and one pipeline. It is also the implementation of the public
  `IAssetLibrary` seam (`IWindowRenderer::assets()`), which is how a host loads a texture or a
  glTF file without seeing a Vulkan type — including `morph_target_count(MeshId)`, which lets a
  host (the editor's blend-shape sliders) size a per-target weight buffer without reaching into
  the mesh registry.

- `TextureLibrary` — decode (stb), upload with a GPU-generated mip chain, bindless registration,
  path deduplication. Residency is mip-based against a budget: a texture that will not fit is
  uploaded from a lower mip and upgraded later, at most one per frame. Uploads never block the
  frame.

  Where `host_image_copy` is available (the Vulkan 1.4 floor makes it common, though it stays an
  optional feature), a mip chain is box-filtered on the CPU and copied straight into the
  optimal-tiled image with no staging buffer, no submit, and no fence — the superseded image and
  heap slot are instead reclaimed once a shared frame counter has advanced past every view that
  could still be sampling them. The staging-plus-blit path remains the fallback: each of its
  uploads carries its own fence, and its superseded image, staging memory, and heap slot are
  reclaimed only once that fence signals.

- `MaterialSystem` — packs authored materials into a per-frame storage-buffer array of
  fixed-layout records holding heap indices. A draw then carries one material index in its push
  constant rather than a payload of parameters, which is the shape the indirect-draw work in
  Phase 7 needs.

- `gltf_importer` — one mesh per primitive, baked into its node's world transform so a multi-part
  asset assembles without a scene graph on the render side. The core material maps across
  directly; `KHR_materials_*` drives the advanced lobes; spec-gloss is converted (lossily, and
  deliberately so); missing tangents are generated.

**Image-based lighting** (`engine/presentation/render/source/passes/ibl_pass.*`) is captured from
the engine's own analytic sky rather than an imported HDRI: six 90-degree views of the atmosphere
are rendered into a cubemap with the same `sky.frag`, GGX-prefiltered into a roughness mip chain,
and cosine-convolved into an irradiance cube. A split-sum BRDF LUT is generated once at bring-up
— it depends on nothing but the BRDF. Capture is rate-limited and gated on the sun or the
atmosphere having measurably moved, so a slowly turning sun costs almost nothing, and the
lighting tracks time of day for free.

The BRDF itself (`engine/presentation/render/shaders/pbr_common.glsl`) is no longer
single-scatter: height-correlated Smith visibility, **Kulla-Conty multi-scatter compensation**
driven by the same BRDF LUT the IBL needs (which is why rough metals stop losing most of their
energy), roughness-aware Fresnel, specular occlusion from AO, and a dielectric F0 derived from
the material's index of refraction instead of the 0.04 every plastic uses.

Indirect diffuse is no longer a single global value. **Probe-volume global illumination**
(`engine/presentation/render/source/gi/`, Phase 6) places three nested camera-relative cascades
of irradiance probes (32×8×32 each, 4/8/16 m spacing, ~124/248/496 m reach), every one snapped to
its own world grid (so probes hold fixed world positions and do not swim as the camera moves).
Each probe stores the same nine SH coefficients the IBL build produces, and `pbr.frag` walks the
cascades finest-to-coarsest, blends the eight probes around a surface trilinearly in the first
cascade that contains it in place of the single global set — falling back to the global
environment SH beyond the coarsest cascade or when GI is off, so nine coefficients always shade a
pixel.

`IrradianceVolumePass` owns the probe SH grid (scene set-0 bindings 29–30, pass-owned and
hand-barriered like the IBL SH buffer) and relights it each frame through a pluggable
`IProbeTracer` — the strategy seam (DIP) that decides how a probe gathers incident radiance. The
default `SDFProbeTracer` (Tier A, all hardware) rebuilds a coarse scene distance clipmap (64³)
each frame from the frame's analytic primitives and the per-mesh signed-distance bricks
`MeshRegistry` bakes at import (`Geometry::bake_signed_distance_field`, reached through the
`engine/presentation/render/source/gi/mesh_sdf_baker.hpp` adapter), then sphere-traces it per
probe: a hit contributes one coloured bounce plus any emitted radiance (a parallel emissive
clipmap injects `material.emissive` at probe rate — lights from materials, no separate light
path), a miss the distant environment, projected back to SH.

The trace is amortized — a
round-robin quarter of all probes per frame, forced full only for a cascade that shifts a cell
(the coarse grids shift far less often) or on a sun move — and rough reflections fall back to the
same probe cache before the sky. `EnvironmentProbeTracer` (broadcast the environment SH, no
trace) is the cheaper floor-tier strategy behind the same seam. Tier-gated (High/Ultra) and
author-gated (`Environment::gi`).

### 1.3. The temporal core

Everything that relates one frame to the next lives behind one small block,
`Scene::TemporalUniforms` at set 0 binding 9, deliberately separate from the scene block: that
one describes the world and is declared as a truncated prefix by most shaders, which makes
appending to it fragile, while this one has different readers and a different reason to change.

- **Motion vectors.** The geometry pass writes a third target holding each pixel's UV
  displacement since the previous frame. The current clip position is `gl_Position`; the previous
  one comes from `Scene::MotionSystem`, a per-frame array of previous transforms keyed by picking
  id that a draw indexes with one push-constant slot — the same shape as the material array, and
  for the same reason. Each side is camera-relative against its own frame's eye, subtracted in
  double before the float cast, so the camera's own translation shows up in the motion vector
  without planet-scale metres ever entering single precision. Pixels no draw covered carry no
  motion vector; the resolve reprojects those from the view ray instead, which is exact for the
  sky.

- **Jitter.** `Frame::frame_jitter` walks a Halton (2, 3) sequence and the offset is added to the
  projection's third column and to the sky and cloud ray directions — nowhere else, so no
  world-space value moves. The sign is not free: the third column is scaled by view z and the
  perspective divide is by −z, so a positive entry shifts the result negative, and the ray offset
  and the motion vector's jitter removal are both matched against that.

- **The resolve.** `TAAPass` dilates the motion vector to the closest surface in a 3×3
  neighbourhood, clips the reprojected history into that neighbourhood's colour distribution in
  YCoCg, reconstructs both inputs with a Catmull-Rom filter, blends under Karis tone weighting,
  and sharpens to offset the temporal softening. The history lives at the *output* extent while
  the scene is rendered at the internal extent, which is what makes rendering small and resolving
  large an upscale rather than a blur. Two history images ping-pong by frame parity, not by frame
  slot: with two frames in flight, the other slot's image is two frames old.

- **Dynamic resolution.** `Frame::ResolutionController` maps the GPU time the Phase 0 timers
  measured to an internal render scale, dropping quickly on an overrun and recovering gradually,
  quantised to sixteenths so a scale hovering near a boundary does not reallocate every
  transient. Picking follows the internal extent and `pick()` scales the click into it.

- **Variable rate shading.** A compute pass derives a per-tile rate image from the previous
  frame's luminance contrast and this frame's motion, and the sky pass binds it through
  `RenderPassBuilder::shading_rate_attachment`. It sits between the geometry and the sky because
  that is the only non-circular ordering — the mask wants the motion vectors the geometry pass
  writes — and because at planet scale the sky is the pass worth steering. Absent
  `VK_KHR_fragment_shading_rate` the handle stays invalid and every declaration of it is a no-op,
  so no pass branches on support.

`RenderSettings`
(`engine/presentation/render/include/SushiEngine/render/render_settings.hpp`) is the public seam
for all of it, kept apart from `Environment` on the same principle: `Environment` describes the
world being drawn, `RenderSettings` describes the machinery drawing it.

### 1.4. Shadows

The sun is shadowed at three scales, because no single mechanism spans the range a planet-scale
scene does.

- **Cascades** (`Scene::fit_shadow_cascades`, `ShadowPass`) carry the body of it. Four cascades
  share one atlas as a two-by-two grid of tiles, so four cascades cost one image, one pass, one
  barrier, and one profiler entry — the cascade being drawn changes with a viewport and a
  push-constant slot, never a rebind.

  The split positions are the practical scheme; each cascade is bounded by a **sphere** rather
  than by its frustum corners and its light-space origin is **snapped to whole shadow texels**,
  which together are what stop the edges crawling: a sphere's extent does not change with
  rotation, and a texel-aligned origin cannot shift sub-texel. The whole fit is camera-relative.
  The maps are orthographic and therefore linear in depth, so unlike the camera they do **not**
  use reverse-Z — there is no precision to redistribute.

- **A depth prepass** (`DepthPrepass`) runs the same `mesh.vert` the shading pass does, with no
  fragment stage. Sharing the shader is a correctness requirement, not a convenience: the opaque
  pass tests against depths it recomputes itself, and only the same shader guarantees the two
  agree bit for bit.

- **Screen-space contact shadows** (`ContactShadowPass`) march that depth buffer toward the sun
  over a distance measured in metres, recovering the contact a cascade texel is orders of
  magnitude too coarse to resolve. This is why the prepass exists — the answer has to be known
  before the surface is shaded.

- **The clouds** shadow lit surfaces through `cloud_shadow_common.glsl`'s
  `cloud_sun_transmittance`, one fetch of `CloudShadowMapPass`'s baked 768² optical-depth map
  (see [the cloud pass](#15-lighting-materials-and-the-sky)) turned into a transmittance with the
  cloud march's own Beer-Lambert scale. The sky pass's analytic-ground shadow and a mesh's own
  shadow both call this same function against the same map — a single shadow authority instead of
  two independent approximations — so a surface standing on the ground and the ground itself
  always agree on where a cloud is.

- **A traced ray** (`RayTracing::SceneAccelerator`, `RayTracedShadowPass`) replaces all of that on
  the Ultra tier, where the device offers `VK_KHR_acceleration_structure` and `VK_KHR_ray_query`.
  Two levels: a bottom-level structure per distinct mesh, built once and kept, and a top-level
  rebuilt each frame from 64-byte instance records — which is why a thousand copies of one mesh
  cost a thousand records rather than a thousand rebuilds.

  Every structure a frame needs is created and sized *before* any build is recorded, because they
  share one scratch buffer and growing it between builds whose scratch address is already baked
  in is a use-after-free the GPU finds long after the call that caused it. The result is a
  screen-space mask, not a term inside the material shader: only the trace shader then needs the
  ray-query extension, so the material shader stays one build that runs everywhere. It is also
  the one place in the renderer that writes its own barrier — an acceleration structure is
  neither an image nor a buffer, so there is no declaration the graph could have derived one
  from.

`Scene::ShadowUniforms` at set 0 binding 10 carries all of it, separate from the scene block for
the same reason the temporal block is.

### 1.5. Lighting, materials, and the sky

The environment the renderer lights against is a neutral seam,
`engine/domain/environment/include/SushiEngine/environment/environment.hpp`, depending only on
`engine/foundation/core/include/SushiEngine/core/types.hpp` — so the simulation authors it and
the renderer consumes it without either depending on the other. It holds the sun
(`DirectionalLight`), the sky's derived `CelestialLight` list, the `Wgs84` ellipsoid (equatorial
radius 6378137 m, inverse flattening 298.257223563), and the `AtmosphereParams`,
`PlanetParameters`, the genus-driven `Cloudscape` (up to `CLOUD_MAX_DECKS` `CloudDeck`s, each a
WMO `CloudGenus` resolved through `cloud_genus_profile`), `StarParameters`, and
metallic-roughness `Material` that describe how the planet is lit and surrounded. The simulation
carries one `Environment` on `RenderScene` and a `Material` per `RenderInstance` (its albedo kept
in sync with the entity's `Tint`), authored through `IWorldEditor`'s
`environment()`/`set_environment()` and `material()`/`set_material()`; the editor's
**Environment** panel and the Inspector's material section drive them.

`ISceneView::render` takes the `Environment` and the camera's world position and draws the frame
in **three HDR passes** (the Vulkan scene view's targets are now linear `R16G16B16A16_SFLOAT`,
resolved to the `R8G8B8A8_UNORM` image the editor samples):

1. **Opaque** — the grid, meshes, and cloth into the HDR colour + `R32_UINT` id + depth targets,
   drawn in the same **camera-relative space** the sky pass uses. Each model's translation has
   the camera eye subtracted in double precision before the `float` cast (`make_push`), cloth
   points are offset by the eye as they are written to their buffer, and the uploaded view matrix
   carries no translation — so a mesh far from the ECEF origin reaches the GPU as a small offset
   from the camera and keeps full `float` precision instead of jittering on the ~16 m grid a raw
   1.5e8 m coordinate collapses to.

   `pbr.frag` shades each mesh with a Cook-Torrance metallic-roughness BRDF (GGX distribution,
   Smith geometry, Schlick Fresnel) lit by the sun and an ambient floor, taking the view direction
   as `-v_world_position` (the camera is the origin of this frame), and reads a per-frame scene
   uniform block — the scene view's first descriptor set, shared by every pass.

2. **Sky** — a fullscreen pass (`sky.frag`) that works in **camera-relative space** (the camera
   is the origin; the planet centre arrives relative to it, so planet-scale metres never leave
   double precision on the CPU). It intersects the WGS84 ellipsoid for the lit ground (onto which
   the cloud stack casts its combined shadow), and evaluates the atmosphere's Rayleigh + Mie
   scattering not as a per-pixel march but through the **Hillaire 2020 LUT stack** a preceding
   `AtmosphereLUTPass` builds
   (`engine/presentation/render/source/passes/atmosphere_lut_pass.*`,
   `engine/presentation/render/shaders/atmosphere_common.glsl`): a **transmittance LUT** and a
   **multiple-scattering LUT** (view-independent, change-gated), a per-frame **sky-view LUT** (the
   background sky in the camera's local frame), and a per-frame **aerial-perspective froxel
   volume** (the air in front of each mesh).

   A background pixel is a sky-view fetch, a mesh within 32 km an aerial-volume fetch, and the
   analytic ground and far geometry keep a march that reads the sun's transmittance and the
   multiple scattering from the LUTs. Over the top a `VolumetricFogPass`
   (`engine/presentation/render/source/passes/volumetric_fog_pass.*`,
   `engine/presentation/render/shaders/fog_scatter.comp`) marches an authored ground-hugging fog
   — a global height term plus up to eight local box/ellipsoid volumes — into a second froxel
   volume folded over every pixel in the composite.

   It adds the real bright-star catalogue — each of the ~60 stars in
   `engine/domain/astro/include/SushiEngine/astro/star_catalog.hpp` (J2000 position, magnitude,
   B-V colour) is rotated by the ephemeris into the observer's local sky and drawn as a point at
   its true direction, so the constellations are recognisable rather than a random field —
   composited over the opaque scene by the sampled depth, so geometry occludes the sky and thin
   air is added over it as aerial perspective. The stars are faded by the atmosphere's optical
   depth, so as the camera climbs the blue sky thins into black space and the stars emerge: the
   near-surface-to-orbit transition falls out of the physics rather than a hard switch.

3. **Cloud** — the genus-driven volumetric cloudscape, ray-marched in a **dedicated pass**
   (`cloud.frag`) at `QualityParameters::cloud_buffer_scale` of the render extent (design doc
   §4.7's "Cloud buffer" row — a third on Low, half on Medium/High, ¾ on Ultra) into two MRT
   targets: (`scattered.rgb`, `transmittance`) and, since W3, the transmittance-weighted mean
   march depth (`frame.targets.cloud_depth`, accumulated the same way `scattered` is, normalized
   by the total in-scatter weight) — the aerial-perspective coupling's input, described with the
   composite below.

   Density comes from `CloudscapeCompilePass`
   (`engine/presentation/render/source/passes/cloudscape_compile_pass.{hpp,cpp}`,
   `engine/presentation/render/shaders/cloudscape_field.comp`,
   `engine/presentation/render/shaders/cloudscape_skip.comp`): a compute bake that runs the genus
   loop once per texel of a 3D field — since atmosphere phase B1 (see
   [`domain-atmosphere.md`](domain-atmosphere.md)),
   two **camera-centred, non-wrapping windows** (a near one at a 32,768 m span/128 m texel, a far
   one at 262,144 m/~1 km texel, addressed through the shared
   `engine/presentation/render/shaders/cloud_field_window.glsl`) rather than a periodic,
   camera-locked tile — instead of once per march sample, gated by the window drifting out of
   range, an authored change, or the weather/wind/sun advancing (see the phase B1 cadence in
   [`domain-atmosphere.md`](domain-atmosphere.md) for the exact rules).

   Two more passes bake off that same field, both amortized across 8 frames instead of
   change-gated (the sun moves every frame the clock advances, so there is no "settled" state to
   gate on): `CloudLightVolumePass`
   (`engine/presentation/render/source/passes/cloud_light_volume_pass.{hpp,cpp}`,
   `engine/presentation/render/shaders/cloud_light_volume.comp`) bakes a 256x256x32 volume of
   summed density toward the sun, refreshing a Y-slice group at a time, and `CloudShadowMapPass`
   (`engine/presentation/render/source/passes/cloud_shadow_map_pass.{hpp,cpp}`,
   `engine/presentation/render/shaders/cloud_shadow_map.comp`) bakes a 768² map of the same
   quantity addressed directly in window space, refreshing a row group at a time — the single
   shadow authority `cloud_shadow_common.glsl` and the analytic ground both read (see
   [shadows](#14-shadows)).

   A fourth bake shares the same cadence: `CloudPanoramaPass`
   (`engine/presentation/render/source/passes/cloud_panorama_pass.{hpp,cpp}`,
   `engine/presentation/render/shaders/cloud_panorama.comp`, W3) marches the field and light
   volume into a 512×256 equirectangular panorama — the design doc's far-field/reflection-probe
   impostor — refreshing a row group at a time; landed as a verified, standalone bake this phase,
   with wiring a consumer (the reflection-probe capture in `IBLPass`) scoped out (see the W3
   CHANGELOG entry). All four bakes register first in the frame, ahead of every consumer.

   `cloud.frag`'s march reads the field with one or two fetches (plus a coarser max-pooled skip
   field for its coarse probe) where it used to run the full per-deck loop; wind is absorbed into
   the window's own origin rather than applied as a UV offset at sample time (see the phase B1
   window in [`domain-atmosphere.md`](domain-atmosphere.md)),
   so the bake stays cacheable while the sky still advects every frame. Since phase B1, the field
   the march reads is itself already spatially resolved from the simulation (see
   [the spatial weather field](domain-atmosphere.md#13-the-spatial-weather-field-atmosphere-phase-a)
   and the window section) — there is no separate per-sample weather-field fetch layered on top
   of the bake any more; that mechanism (the coverage-scale/reference-column lookup) was deleted
   once the bake itself carried the simulation's per-column coverage.

   The march's step length grows with distance already travelled from the camera (not with camera
   altitude, the variable it keyed on before W0 — that starved the march exactly when a climb
   made the cloud band thinner on screen, so quality fell with altitude for no geometric reason),
   floored by the tier's near-camera budget and capped by its far-field step count; an empty skip
   cell advances the march by the skip field's own cell size rather than consuming the sample
   budget (the Nubis3 `step = max(skip_distance, 0.08*sqrt(dist), min_step)` rule).

   Within 200 m
   of the camera (design doc §4.4/§4.7's near-band radius) an extra fixed-scale (811 m) erosion
   tap, with a curl-noise warp folded in near cloud bases, adds detail the field's own
   camera-independent bake cannot carry; on the High/Ultra tiers
   (`QualityParameters::cloud_near_far_split`) the march's temporal dither also freezes at a
   fixed phase beyond 250 m instead of animating every frame (Nubis3's "jitter animated only
   < 250 m, static hash beyond"), which is what keeps a distant silhouette's sample pattern — and
   so the cloud TAA's history below — stable at range; the literal dual-viewport near/far
   resolution split design doc §4.4 also describes is a scoped W3 deferral (see the CHANGELOG
   entry).

   Lighting samples the baked light volume for one fetch per lit sample
   (`QualityParameters::cloud_light_taps`, 0-3, tiers how often a costlier inline cone march
   layers a correction on top) and shapes it with dual-lobe Henyey-Greenstein scattering (the
   author's forward `forward_scattering` g paired with a fixed back lobe) through the analytic
   in-step energy-conserving integration `(1 - exp(-sigma*ds))`; ambient uses the field's own
   baked vertical-profile channel (`pow(1 - profile, 0.5)`) for dark edges and inner glow instead
   of a flat height blend. The march stops at the opaque depth or analytic ground so clouds draw
   over terrain. Running at a quarter of the pixels is the largest single saving on top of the
   field; variable-rate shading steers this pass the same way it steers the sky pass. Disabling
   clouds (`Environment::Clouds::enabled`) skips every bake and the march entirely — their
   render-graph nodes clear/no-op in hardware instead of running a shader.

4. **Tonemap** — before the composite, `CloudTAAPass`
   (`engine/presentation/render/source/passes/cloud_taa_pass.{hpp,cpp}`,
   `engine/presentation/render/shaders/cloud_taa.comp`, W3) resolves the cloud buffer's own
   dedicated temporal history: a YCoCg neighbourhood variance clip (gamma ≈ 1.2,
   `QualityParameters::cloud_variance_clip` — Low instead takes a cheaper 5-tap cross min/max
   clamp, design doc §4.7's TAA row; the flag selects *which* neighbourhood rejection runs, never
   whether one does, because the rejection test is this feedback loop's stability condition
   rather than a quality feature layered on it, and both paths clamp alpha as well as colour
   since the composite folds the sky through that alpha) against motion vectors sourced from the
   existing velocity target where opaque geometry is visible and a view-ray reprojection fallback
   where it is not (the common case: most of a cloud pixel's neighbourhood is sky), with history
   acceptance ramping up over accepted frames and boosted further under sub-pixel motion.

   It owns its resolved colour and a per-pixel history-acceptance weight as two pairs of
   pass-owned images ping-ponged by frame parity — sized at a *fixed* half of the view's output
   extent rather than the dynamic render extent, so tier and dynamic-resolution changes reach it
   the same render/output-extent reconciliation `taa.frag` already does for the main resolve,
   without forcing a history resize on every step. This runs entirely before — and is independent
   of — the frame's own `taa_pass_`, which only ever sees the already-composited cloud
   contribution as an ordinary shaded pixel.

   `cloud_composite_pass` then resolves CloudTAAPass's output over the full-resolution sky
   (`sky * transmittance + scattered`) with a nearest-depth-aware upsample — weighting each of
   the four nearest cloud texels by whether its depth roughly matches the output pixel's, so a
   background cloud sample cannot bleed a halo across a foreground silhouette, the same four
   weights also reconstructing the W3 cloud-depth MRT — and folds in aerial perspective by
   sampling the Hillaire aerial-perspective froxel volume (`AtmosphereLUTPass`) once per pixel at
   that reconstructed depth: the cloud's own in-scattered light is attenuated by the air in front
   of it and the air's own in-scatter is added back, weighted by how much cloud is actually
   there, so a distant deck hazes, desaturates, and sinks toward the horizon exactly like a mesh
   already does. `tonemap.frag` then applies exposure, the ACES filmic curve, and a gamma encode,
   resolving the HDR composite into the sampled LDR image.

The planet is drawn analytically in the sky pass rather than as tessellated terrain, which is why
a real WGS84 Earth and its atmosphere render with no level-of-detail machinery; the ground grid
the editor shows sits tangent to the ellipsoid at the local origin. The existing floating-origin /
ECEF types (see [the value-type seam](foundation.md#2-the-value-type-seam)) anchor the camera's
world position that the sky pass places the planet relative to.

Depth is **reverse-Z with an infinite far plane** on a `D32_SFLOAT_S8_UINT` buffer: `perspective`
maps the near plane to clip depth 1 and infinity to 0, the pipeline clears depth to 0 and
compares `GREATER_OR_EQUAL`, and floating-point precision is spread almost uniformly across the
whole range — so one camera resolves a few-centimetre prop and a planet 10⁷ m away in the same
frame without z-fighting, and nothing is ever clipped for distance. The sky pass reconstructs
view-z straight from the projection matrix, so it is independent of the depth convention; its
"geometry is here" tests key on `depth > 0`.
