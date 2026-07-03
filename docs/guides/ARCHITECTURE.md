# SushiEngine Architecture

This document explains how SushiEngine is put together: the relationship to
SushiRuntime, the layers, and the seams a non-trivial change will touch. Read it
before making a change that crosses more than one file.

## 1. The head and the battery

SushiEngine is a **head**; SushiRuntime is a **plugged-in component** — the
battery. The engine is the product a game is built against: it owns the loop, the
world, and — as it grows — the window, the renderer, and the editor. SushiRuntime
is a hardware-agnostic orchestration backbone the engine hands work to.

The dependency points one way only:

```
SushiEngine  ──depends on──▶  SushiRuntime
```

The runtime never knows the engine exists. It has no concept of a game, a frame, an
entity, a component, or a renderer; it schedules an abstract task graph across
whatever hardware is present. This is deliberate, and it is the rule that keeps both
projects changeable: the engine may be rewritten without touching the runtime, and
the runtime may gain backends without the engine noticing.

A practical consequence: a feature that only *composes* the runtime's public API
belongs in the engine. A feature that needs the runtime to do something it cannot
yet do belongs in SushiRuntime, behind its public API — not bolted onto the engine
as a workaround.

## 2. Layers

The engine is header-only at this stage. Each layer depends only on the ones below.

| Layer | Headers | Responsibility |
|-------|---------|----------------|
| SushiLoop core | `loop/fixed_timestep.hpp`, `loop/rng.hpp`, `loop/input.hpp`, `loop/rollback.hpp`, `loop/net.hpp` | Fixed-step time, seeded RNG, per-tick input capture, rollback snapshots, and loopback network reconciliation (see §8, §9). |
| Physics     | `physics/pgs_solver.hpp`, `physics/graph_coloring.hpp`, `physics/xpbd_solver.hpp`, `physics/rigid_body.hpp`, `physics/physics_world.hpp`, `physics/cloth.hpp` | Graph-coloured PGS solver, its unified XPBD rigid-body generalization, and cloth grids built from it (see §4). |
| Schedule    | `ecs/schedule.hpp` | Compiles systems to a runtime graph and replays it. |
| Commands    | `ecs/command_buffer.hpp` | Records structural changes, applied at a barrier. |
| World       | `ecs/world.hpp` | Entities, archetypes, spawn/destroy, component access. |
| Storage     | `ecs/archetype.hpp`, `ecs/chunk.hpp` | Archetype chunks of structure-of-arrays columns. |
| Identity    | `ecs/entity.hpp`, `ecs/component.hpp` | Entity handles, component ids, access tags. |
| Value types | `core/types.hpp` | The single seam for scalars and vectors (see §6). |

`SushiEngine.hpp` is the umbrella header that pulls the surface together.

## 3. The ECS and the system graph

This is where the engine's "the graph behaves like a game engine" thesis is built —
on the runtime, not in it.

**Storage is archetype chunks.** Entities sharing one component set form an
*archetype*; an archetype stores its entities in fixed-capacity *chunks*. Within a
chunk each component is a separate contiguous column backed by its own runtime
allocation (structure-of-arrays). A column's base pointer is therefore a distinct
resource the runtime's dependency tracker keys on, which is what makes chunks the
unit of parallelism: two systems touching different chunks run in parallel, two
touching the same column are ordered — with no scheduler written in the engine.

**A system is a graph node.** A system declares the components it reads and writes
(`Read<T>` / `Write<T>`); the Schedule emits one node per matching chunk, keyed on
that chunk's columns. The runtime's dependency tracker *is* the system scheduler:
it derives the ordering from component access. A read-after-write on a component
orders the two systems; disjoint access leaves them parallel.

**Counts are late-bound; structure is compiled.** Each node iterates its chunk's
live entity count, re-read every step, so spawning and destroying entities within
existing chunks varies the work with no recompile. The graph is rebuilt only when
the *chunk set* changes (a new archetype or chunk) — reported by the world's
`structure_version`. Pre-reserving chunks keeps a steady spawn/destroy workload at a
single compile.

**Structural changes are deferred.** Systems run as device kernels and must never
see entities appear or vanish mid-frame. Gameplay code records spawns and destroys
into a `CommandBuffer` during the frame, and the loop applies them once, at an
explicit barrier between steps. Destroy is an O(1) swap-remove that keeps a chunk's
live rows packed; entity handles carry a generation so a stale handle to a destroyed
entity is detected, not silently reused.

The worked example in `sandbox/main.cpp` exercises all of this — Position, Velocity,
Mass, and Lifetime components; `apply_forces`, `integrate`, and a parallel
`decay_lifetime` system; per-frame spawn and deferred destroy — and checks every
surviving entity against an independent scalar reference, with the graph compiled
exactly once across the whole run.

## 4. The physics constraint solver

Physics is a domain layer on top of the runtime, the same way the ECS is: it
expresses itself as ordinary read/write sets and lets the dependency tracker do the
ordering. The solver here is **Projected Gauss-Seidel** (PGS), the sequential
constraint method, parallelised by **graph colouring**.

A sequential Gauss-Seidel sweep cannot run all constraints at once: two constraints
that share a body would race. So the constraints are *edge-coloured* over the bodies
(`color_constraints`) — each colour is a batch in which no two constraints share a
body. The `ConstraintSolver` then emits one task per colour: a parallel projection
over that colour's constraints, which is race-free because the bodies are disjoint.
Every colour reads and writes the shared position array, so the runtime orders the
colours into a sequential sweep — colour k+1 after colour k — while parallelising
fully *within* a colour. That ordering is exactly Gauss-Seidel across colours, and
because a colour's constraints are independent, the parallel result equals the
sequential one. The sweep is repeated for the iteration count, and the whole solve is
one graph compiled once and replayed every frame.

The solver owns no engine concept beyond bodies and constraints — it takes a position
array, an inverse-mass array, and a projection functor — so a new constraint type
(contacts, angular joints) is added by providing its POD and its device projection;
the colouring and the graph structure are reused unchanged. `DistanceConstraint` with
`DistanceProjection` is the first concrete type, exercised by `examples/pgs_demo.cpp`
(a hanging chain checked against a scalar reference).

### 4.1. XPBD: the rigid-body generalization (SushiLoop M2)

`physics/rigid_body.hpp`, `physics/xpbd_constraint.hpp`, and `physics/xpbd_solver.hpp`
add the unified XPBD (position-based dynamics) solver `SUSHILOOP.md` calls for:
one compliant-constraint framework meant to grow into rigid bodies, soft bodies,
cloth, and rope, rather than a family of special-cased solvers living side by side.

`RigidBody` extends the PGS solver's bare position + inverse mass with an
orientation (`Quat`) and a diagonal, body-local inverse inertia tensor, plus the
predicted pre-solve pose (`prev_position`/`prev_orientation`) XPBD's velocity update
needs. `predict()`/`update_velocity()` are the two halves of one XPBD sub-step:
integrate external forces into a predicted pose, solve constraints against that
prediction, then recover velocity and angular velocity from how far the solve moved
it — never from an explicit force/torque integration.

`XpbdDistanceConstraint` generalizes `DistanceConstraint` to two attachment points
(offsets in each body's own local frame, so an anchor at the origin recovers a
plain rigid link) and adds `compliance` — XPBD's defining feature over plain PBD:
a constraint's stiffness is a physical unit (inverse stiffness, really) instead of
an artifact of iteration count or step size, so `compliance == 0` is a fully rigid
constraint and identical PGS behaviour (verified in `test_xpbd_solver.cpp`: with
zero inverse inertia and zero-offset anchors, no angular coupling can occur, so
the two solvers' linear terms are the same arithmetic).

`XpbdSolver<Constraint>` reuses `color_constraints`/`ColorBatches` unchanged — same
graph-colouring, same compile-once-replay-every-frame structure as
`ConstraintSolver` — but the shared resource is one `RigidBody` buffer instead of
separate position/mass buffers, and each constraint carries a per-step Lagrange
multiplier (`lambda`) that `solve()` resets to zero before every step, because the
compliance term is only meaningful accumulated within a single step.
`examples/xpbd_demo.cpp` ports `pgs_demo.cpp`'s hanging chain onto `RigidBody`/
`XpbdSolver`, checked against a byte-for-byte host mirror of the projection.

`physics/physics_world.hpp`'s `PhysicsWorld<Constraint>` is the layer above
`XpbdSolver` that turns a one-shot solve into an actual loop: register bodies and
constraints, `finalize()` once (uploads the bodies, compiles the graph — mirrors
`XpbdSolver`'s own build-once-replay-every-frame split), then `step()` every frame
runs predict → solve → derive-velocity for each requested sub-step. It takes no
dependency on `ecs/` on purpose, keeping the layering direction in §2 intact; the
ECS-facing half (mapping entities to body indices, syncing `Transform`/
`Orientation` each frame) is a `sim/`-level concern that builds on this seam rather
than being folded into it.

**The editor's "Rigid Body" toggle** (`sim/runtime_simulation.cpp`) is the first
consumer of that seam, and takes a different route from the generic
`sim/physics_bridge.hpp` below: Renderer/Camera need an ECS component migration
(`migrate_components`) because their data — colour, lens — lives in a component
only present when attached, but a Rigid Body's data (position, orientation) is
already `Transform`/`Orientation`, always present. So attaching/detaching physics
is plain host bookkeeping in `RuntimeSimulation::Record` (`has_physics_body`,
`physics_params`), and a `Physics::PhysicsWorld<XpbdDistanceConstraint>` (no
constraints registered — this is free-body physics only, no joints yet) is rebuilt
lazily in `tick()` whenever the physics-driven entity count changes, the same
"rebuild only when the input set changes" discipline `Schedule` and `XpbdSolver`
already follow. A rebuild snapshots every currently-simulated body's live state
first, so toggling physics on one entity never resets another already-falling
body in the scene; a brand-new body seeds from the entity's current `Transform`/
`Orientation` at rest instead. `tick()` steps the world under gravity and writes
the solved pose back before the ECS schedule runs, at a fixed assumed ~1/60s frame
— `loop/fixed_timestep.hpp`'s `FixedTimestepClock` is not wired into this loop yet.
`.sushiscene` (`editor/scene_serializer.cpp`) carries `has_physics_body`/
`physics_body` as an independent field pair (not mutually exclusive with camera/
renderer, unlike those two).

`sim/physics_bridge.hpp` is that `sim/`-level half. `sim::PhysicsBody` is an
ordinary component naming which `PhysicsWorld` body an entity owns (`INVALID` until
registered — an entity can carry the component before it has one); this keeps the
mapping in the ECS itself rather than a side table. `sim::initial_rigid_body()`
reads an entity's current `Transform`/`Orientation` once, at
`PhysicsWorld::add_body()` time, to seed the body's starting pose.
`sim::sync_transforms_from_physics()` is the one direction wired up so far: every
tick, after `PhysicsWorld::step()`, it walks every archetype matching
`{PhysicsBody, Transform, Orientation}` (the same `World::query()` +
per-chunk-column walk `Schedule::each` uses internally, but as a plain host loop —
there is no parallel work here worth a graph node) and copies each registered
body's solved position/orientation into the entity's `Transform`/`Orientation`.
There is no reverse (ECS -> physics) sync yet — nothing today needs to teleport a
physics-driven entity by editing its `Transform` directly — and no wiring into
`RuntimeSimulation`'s tick loop or the editor yet; this is the seam, not the
integration.

### 4.2. Cloth (SushiLoop M5)

`physics/cloth.hpp`'s `build_cloth_grid` is the confirmation of §4.1's claim that
XPBD is one framework, not a family of special-cased solvers: cloth adds no new
solver or constraint type, only a topology. It registers `rows * cols` `RigidBody`s
(zero inverse inertia, so anchors implicitly at each body's own origin recover the
same linear-only degeneration `xpbd_demo.cpp`'s hanging chain already relies on)
into the caller's `PhysicsWorld<XpbdDistanceConstraint>`, with row 0 pinned
(`inv_mass == 0`), and wires a structural `XpbdDistanceConstraint` to each right and
below neighbour plus a shear constraint to each diagonal neighbour pair — the shear
links are what keep the grid from collapsing into a parallelogram under load, since
structural links alone only resist stretching along the grid axes. `ClothGrid`
exposes the registered body ids by `(row, column)` so a caller (a demo, or later a
gameplay layer) can address a specific grid point without recomputing the row-major
indexing itself.

`examples/cloth_demo.cpp` mirrors `xpbd_demo.cpp`: a device solve through
`PhysicsWorld`, checked against a byte-for-byte host mirror of
`XpbdDistanceProjection` run over the identical topology. `Integration_Cloth`
(`tests/functional/integration/test_cloth.cpp`) proves the grid's shape (body/
constraint counts, which row is pinned) and that the pinned row never moves while
the rest of the grid falls and settles under gravity.

Volumetric (tetrahedral) soft bodies are explicitly **not** built here — cloth is a
2D constraint grid over point masses, not a general deformable-solid solver, and a
tet-mesh XPBD extension (volume-preservation constraints, a different topology
entirely) is a distinct future milestone, not a natural extension of this file.

## 5. The render seam

Rendering does not belong inside the runtime — the runtime knows no graphics, just
as it knows no math. The renderer is a separate compiled library (`render/`,
`include/SushiEngine/render/`), a **greenfield Vulkan 1.3** backend behind a
dependency-inversion boundary so a D3D12/Metal backend can follow without touching a
consumer. The layering, from abstract to concrete:

- **RHI device** (`render/rhi/device.hpp`): `IRenderDevice` / `create_render_device()`
  carry no Vulkan types. `DeviceInfo` exposes the physical device's UUID — the key a
  later milestone matches against SushiRuntime's SYCL device for zero-copy interop.
  `RenderDeviceDesc` carries a `SurfaceFactory` hook and required instance extensions
  so a windowed host supplies its presentation surface without the renderer ever
  calling a windowing library; `native_handles()` is the single, explicit escape
  hatch a native-API adapter (the editor's ImGui Vulkan backend) uses.
- **Presentation facade** (`render/window_renderer.hpp`): `IWindowRenderer` /
  `create_window_renderer()` own the device and swapchain and drive the
  acquire → clear → submit → present cycle; a host opens a frame, records into the
  returned command buffer, and closes it. Swapchain rebuild on resize is internal.
  The Vulkan implementation is `render/rhi/vulkan/vulkan_window_renderer.*`.
- **Headless target** (`render/rhi/vulkan/vulkan_offscreen.*`): the same device path
  without a window, used by `render_probe` to validate the pipeline in CI.
- **Scene view** (`render/scene_view.hpp`: `ISceneView`, created by
  `IWindowRenderer::create_scene_view()`): an offscreen camera view of a `MeshInstance`
  set plus a ground grid, drawn from a `CameraView`. The Vulkan implementation
  (`render/rhi/vulkan/vulkan_scene_view.*`) is double-buffered — the frame being
  sampled by the UI is never the frame being drawn — and leaves its colour image
  shader-readable so the editor samples it with `ImGui::Image`. It exposes only the
  sampler/view handles a UI backend needs, never a full descriptor set. Alongside the
  shaded image it renders a second `R32_UINT` **id target** carrying each instance's
  picking id, copied to a host buffer each frame so `pick(x, y)` resolves a click to
  the entity under the cursor (GPU id-buffer picking); `render` also takes the selected
  id, which the mesh shader highlights.

The editor composes these behind its own **windowing seam** (`editor/platform_window.hpp`
`IPlatformWindow`, SDL implementation `editor/sdl_window.*`) and a **Dear ImGui ↔
Vulkan adapter** (`editor/imgui_backend.*`) — the one editor component that speaks
Vulkan, kept apart from the app loop and panels so the rest of the editor names no
graphics API. A single `ViewportPanel` (`editor/viewport_panel.*`) owns an offscreen
scene view and renders it from an injected camera — the `ISceneCamera` seam
(`editor/scene_camera.hpp`). Two implementations back the two Unity viewports: a
navigable `FlyCameraSource` (the **Scene** view) driving a fly camera
(`editor/fly_camera.hpp`) through a stateless controller (`editor/camera_controller.hpp`)
that reads a library-neutral `InputState` (`editor/input_state.hpp`) the panel fills
from ImGui, and a `WorldCameraSource` (the **Game** view) posed each frame from the
simulation's camera. So the same panel serves both viewports, the controller depends
on no input source and stays unit-testable, and a new camera kind is a new
implementation rather than a new panel. Interaction closes the loop: a left-click picks
via the id target and the Scene view draws the transform gizmo at the selection
(`editor/gizmo_controller.*`, ImGui draw list, projecting through the camera), so an
entity is created from the Hierarchy, selected in any viewport, moved with the gizmo,
edited in the Inspector, and destroyed — all against the one live world.
`GizmoController` offers translate/rotate/scale (Unity's W/E/R) and a `GizmoSpace`
(Local/World) the toolbar toggles; Scale always drags local axes to avoid shearing a
rotated object. Rotate drags are computed by intersecting the mouse ray with the axis's
own plane through the pivot each frame and measuring the signed world-space angle swept
between grab-time and current plane vectors — a screen-space angle would invert once
the camera crosses to the far side of the axis, which is why translate/scale axes and
the ray/plane math live in world space rather than screen space throughout.

Editor and project settings sit behind a **preferences seam** (`editor/preferences.hpp`
`IPreferencesStore`, JSON implementation writing a per-user `preferences.json`). The
Preferences window edits a plain `Preferences` aggregate; the loop persists changes and
applies the live-effective ones (theme, camera speed). Precision is the one setting the
running editor cannot change — it is the compile-time `SE_SCALAR_DOUBLE` of §6 — so the
window records intent and prompts a rebuild.

The **Project panel** (`editor/editor_panels.cpp`) is a two-pane file browser over the
on-disk project: a recursive folder tree and a searchable icon-grid of the current
folder, supporting create/rename/delete, "Show in Explorer", and double-click open
(text extensions open in the built-in text editor; anything else opens via the OS
default application, `ShellExecuteW` on Windows). The project root defaults to
`<user profile>/sushiengine/project` — outside the engine's own source tree — and is
persisted as `Preferences::last_project_root` once resolved, so authored project files
never mix with engine source.

Scene persistence (`editor/scene_serializer.*`, `ISceneSerializer`-free — it is two free
functions, `save_scene`/`load_scene`, since there is only one format) writes/reads a
`.sushiscene` JSON file purely through `IWorldEditor`'s existing query/mutate surface,
so it adds no engine-side type. Parent links are stored as indices into the saved
entity array rather than raw `EntityId`s (ids are not guaranteed stable across a
destroy-and-reload); loading destroys every existing entity, recreates the file's in
order, and resolves parent indices in a second pass once all entities exist, so a
child listed before its parent in the file still resolves correctly. `File ▸ Save
Scene`/`Save Scene As...`/`New Scene` and the Project panel's double-click/`Open` on a
`.sushiscene` file are the entry points; `capture_scene`/`apply_scene` are the same
functions `save_scene`/`load_scene` wrap around file I/O, and are reused directly by
undo/redo below.

Undo/redo (`editor/command_history.*`, `CommandHistory`) is whole-world snapshot-based
rather than a per-field command hierarchy: every step is a full
`capture_scene`/`apply_scene` round-trip, which is simple and correct at this entity
count at the cost of coarser granularity. Two recording modes cover the panels:
`record()` snapshots immediately before a discrete, single-frame mutation (create,
delete, rename, reparent, a checkbox toggle); `begin_change()`/`end_change()` bracket a
continuous edit spanning several frames (an Inspector slider held down, a gizmo drag)
so it costs one undo step regardless of how many frames it runs — panels call
`begin_change` on the widget's activation edge (`ImGui::IsItemActivated()`) and
`end_change` on its deactivation edge (`IsItemDeactivatedAfterEdit()`); the gizmo does
the same off `GizmoController::dragging()`'s grab/release edge, tracked in the main
loop since the gizmo lives inside `ViewportPanel::draw()`. `Edit ▸ Undo`/`Redo`
(Ctrl+Z/Ctrl+Y, ignored while `ImGuiIO::WantTextInput` is set so a rename field's own
text editing is not hijacked) drive it. Because undo/redo swaps the whole world,
entity ids are not preserved across the step, so both clear the current selection
rather than risk it aliasing an unrelated new entity.

`CommandHistory::revision()` is a counter bumped by every `record()`, committed
`end_change()`, `undo()`, and `redo()` — a cheap "has the world changed" signal a host
can compare against a stashed value without diffing snapshots. `EditorContext` stashes
it as `saved_scene_revision` on every successful New/Open/Save; `scene_is_dirty()`
(`editor_context.hpp`) is just the inequality of the two, and backs both the status
bar's `*` on the scene name and the close-confirm prompt below. Ctrl+S and
`File ▸ Save Scene` both go through `save_current_scene()` (`editor_panels.*`), which
saves straight to `scene_path` if set or opens the existing Save-As prompt if the
scene has never been saved, so all three save entry points (menu, shortcut,
close-confirm) agree on when the scene becomes clean. Closing the window (the title
bar's X or `File ▸ Exit`) sets `EditorContext::close_requested` instead of exiting
directly; the main loop's `draw_exit_confirm_modal` lets the frame close immediately
if the scene is clean, otherwise prompts Save/Don't Save/Cancel, deferring to the
Save-As modal (tracked by `EditorContext::exit_after_save`) when Save has no path yet.

Live simulation state reaches the renderer through the **simulation seam**
(`include/SushiEngine/sim/simulation.hpp`): `ISimulation` / `create_simulation()`,
plain C++ that names no runtime, SYCL, or ECS type — only the value types from §6.
The concrete world lives in one compiled library, `sushi_sim` (`sim/`), the single
place device code exists outside an example: it owns a `SushiRuntime::API::Runtime`,
an ECS `World`, and a `Schedule`, and starts with no entities — every archetype is
pre-reserved up front so the editor's own creates never trigger a mid-run chunk
allocation, but nothing is seeded into them. Two systems over disjoint components
(`spin` writes orientation, `orbit` writes position) are registered for entities
that carry `SpinStep`/`OrbitState`, which today only exist if authored directly
against `World` (no editor path attaches them); the dependency tracker still runs
them in parallel whenever such entities are present. Every value a
kernel reads is precomputed on the host into a component so the kernels are pure
arithmetic that capture no host state — the discipline that keeps them legal device
code (see §3). This is dependency inversion at the largest seam in the engine: the
editor links `sushi_sim` and depends only on `ISimulation`, so the runtime, SYCL, and
ECS never enter the editor's translation units, and a different world backend (or a
headless stub) can replace it without the editor changing. Because the editor links a
SYCL library, its final link is SYCL-aware and it ships the runtime DLL — the
plain-toolchain lane is held by `sandbox` and `render_probe`, not the editor.

Each `tick()` runs the schedule and an **extract** pass reads the world's shared-USM
columns back on the host (via `World::get`) into a read-only `RenderScene`
(`RenderInstance` — an `EntityId` + transform + colour — and the resolved cameras) the
editor draws. Cameras are ECS entities too (a `Camera` component: lens plus a
`display_index`/`priority`/`active` routing), posed by their transform; the extract picks,
per display, the active camera with the highest priority into `RenderScene::display_cameras`,
and the Game view chooses which display it shows so two cameras never conflict.
`RenderScene::has_camera` reports whether any camera resolved at all; the Game view
draws zero instances when it is false rather than falling back to a synthetic default
camera, since there is then nothing to play the scene through. `create_simulation()`
seeds no demo entities — the live world starts empty, and `default_camera()` exists
only to give `RenderScene::camera` a well-formed value when `has_camera` is false, not
as something the Game view renders through. The Scene view authors the world (pick,
gizmo) and is the only place a selection is drawn
highlighted; the Game view is played, not authored, so it neither picks nor receives the
Scene selection. The editor ticks only while the toolbar is Playing (or on a one-shot
`step_requested`, set by the toolbar's Step button and cleared every frame), binding the
existing `PlayState`. Pressing Play captures the scene into
`EditorContext::play_mode_snapshot` via `capture_scene`; pressing Stop re-applies it
via `apply_scene` and clears the snapshot, so play-mode mutations (spawns, destroys,
transform/physics edits) never leak into the edited scene, mirroring Unity's
edit/play-mode separation — this reuses the same `capture_scene`/`apply_scene`
round-trip `CommandHistory` already relies on for undo/redo, rather than a second
snapshot mechanism. The extract is a host copy today. A later interop milestone
promotes it to a device-shared sink pinned to a render thread, so the scheduler can
overlap the next step's simulation with the current step's draw and skip the round-trip.

All of SushiEngine's built-in ECS components — `Transform`, `Orientation`, `SpinStep`,
`OrbitState`, `Tint`, `Camera` — are declared in one place,
`include/SushiEngine/sim/components.hpp`, rather than inline in `runtime_simulation.cpp`;
component registration order across translation units must agree (see §3), so keeping
the canonical set in one header is what makes that guarantee easy to keep as more
consumers are added. Transform + Orientation are mandatory on every entity; Tint (the
Renderer component) and Camera are independently pluggable per entity — Unity-style
add/remove — through `IWorldEditor::set_has_renderer`/`has_renderer` and
`set_is_camera`/`is_camera` (distinct from `create_camera`, which spawns a fresh camera
entity). The ECS has no in-place component add/remove — an entity's component set is
fixed by its archetype — so a toggle is implemented as a migration: destroy the entity
and respawn it into the archetype matching the new component set, carrying over
Transform/Orientation and any surviving Tint/Camera value (`RuntimeSimulation::migrate_components`).
Seeded, animated demo cubes (`SpinStep`/`OrbitState`) are exempt from migration — their
component set is fixed for the demo.

The **world is the single source of truth for entities** — there is no separate
editor-side scene model. The editor reads and writes it through `IWorldEditor`, split
from `ISimulation` so a panel that only inspects or edits depends on the narrow surface
(interface segregation): entities are addressed by a stable `EntityId`, queried
(`entities`, `name`, `transform`, `color`, `visible`, `has_renderer`, `is_camera`) and
mutated (`create`, `destroy`, `set_name`, `set_transform`, `set_color`, `set_visible`,
`set_has_renderer`, `set_is_camera`). Transform, colour, and the Camera lens are real ECS
components the surface writes through; names, visibility, and parenting are host-side
editor metadata the simulation keeps beside each entity's handle (`parent`/`set_parent`).
Editor-created entities carry no motion components, so the spin/orbit systems never match
them and they stay authorable while the world plays — only the seeded demo cubes are
system-driven. The Hierarchy renders these entities as a tree (drag-and-drop reparents;
dropping on empty space unparents to root), guarded against cycles by walking the
candidate parent's own ancestor chain before accepting a drop, and the Inspector edits
the selection — including, for Camera and Renderer, an "x" on the header to detach the
component and an "Add Component" menu offering whichever is missing; the editor GUI goes
through Dear ImGui. `EditorContext` splits selection in two: `selected_entity` is the
single "primary" target the Inspector, viewport gizmo, and Align/Move-to-View act on,
while `selected_entities` is the Hierarchy's full multi-selection (Ctrl+click toggles
membership; Shift+click ranges from `selection_anchor` — the last plain or Ctrl click —
over the tree's depth-first display order, or the filtered order when a search filter
narrows the list). A plain click collapses both back to one entity
(`select_only`/`toggle_selected`/`is_selected` in `editor_context.hpp`); `Delete` acts on
the whole vector.

Because parenting is host metadata rather than an ECS `Parent` component, both the
extract pass and a reparent walk the parent chain on the host
(`RuntimeSimulation::world_transform`, bounded by the live entity count against a
corrupt chain) rather than in a kernel — the same host-copy-first posture as extract
itself, revisited only if parenting needs to affect systems running on the device. World
pose is composed as a shear-free hierarchical TRS chain rather than a general `Mat4`
product (`world_scale = parent_scale * local_scale`, `world_rotation = parent_rotation *
local_rotation`, `world_position = parent_position + parent_rotation ∘ (parent_scale *
local_position)`, matching Unity's model) precisely because that form is invertible:
`set_parent` uses the inverse to recompute the child's local transform at the moment of
reparenting, so its resolved world-space pose is unchanged by the move rather than being
reinterpreted (and visibly jumping) in the new parent's space.

## 6. The value-type seam

The engine takes its scalar, vector, matrix, and quaternion types — and the
operations on them — from `core/types.hpp` and nowhere else. Those types belong to
**SushiBLAS** (tensors, and the floats derived from them). Until that library exists,
`core/types.hpp` aliases a minimal placeholder in `core/blas_placeholder.hpp`, which
now carries `Vec3`, `Mat4`, `Quat` and the handful of operations the renderer and
camera need (`perspective`, `look_at`, `compose_transform`, `mul`, …). When SushiBLAS lands, re-point `core/types.hpp` at it
and delete the placeholder — a single-file change, because nothing else in the
engine names the underlying type.

This is the same discipline as §1: one seam, not parallel paths.

The same seam also carries the planet-scale floating-origin types: `WorldVec3` is an
always-double 3-vector for absolute ECEF positions (fixed precision, independent of
`SE_SCALAR_DOUBLE`'s choice of `Scalar`), `SectorCoord` is an integer index of a fixed-size
cube ("sector") in that world space, and `FloatingOriginVec3` pairs a `SectorCoord` with a
`Scalar`-precision local offset from that sector's corner. `to_floating_origin`/
`from_floating_origin` convert between `WorldVec3` and `FloatingOriginVec3` given a sector
size. Keeping the local offset small (at most one sector wide) is what lets gameplay,
physics, and rendering work in single precision at planetary distances instead of paying
for double precision everywhere. These types are the SushiLoop M0 foundation
(`docs/slop/SUSHILOOP.md`) and are not yet consumed by any simulation code.

Because everything routes through this one seam, **precision is a build-time choice**.
The `SE_SCALAR_DOUBLE` option (`cmake/ProjectOptions.cmake`) switches the placeholder's
`Float` between `float` and `double`; it is threaded as a compile definition on the
`SushiEngine` INTERFACE target so every consumer — sandbox, `pgs_demo`, `sushi_sim`
(and thus the editor), and the tests — agrees on `sizeof(Scalar)`. It is compile-time,
not runtime, because `Scalar` is baked into trivially-copyable components and device
storage. `sushi_render` is the one target that shares the value types (across
`MeshInstance`/`CameraView`) without linking the engine target — it links the runtime
otherwise — so it mirrors the same definition to keep the ABI in agreement. The Vulkan
upload path narrows to 32-bit explicitly at the push-constant boundary, so GPU data and
the shaders are identical in either build. The `se` CLI exposes it as `--double` on
`se build`/`se editor` and a persisted `scalar_double` config field.

## 7. Validation and tooling

The engine ships no device code of its own, so there is nothing to test in
isolation — a meaningful test must instantiate kernels and run them against the
real runtime. The suite in `tests/` does exactly that: it follows SushiRuntime's
layout (`functional/{unit,integration,regression}`, a shared `common/` with the
GoogleTest entry point and a process-wide runtime fixture) and builds one binary,
`se_functional_tests`, as a SYCL translation-unit set. There are no mocks. Tests
carry the `Unit_*` / `Integration_*` / `Regression_*` suite-name prefixes, which
`tests/CMakeLists.txt` turns into CTest labels so a sub-suite is one `ctest -L`
away. The integration tests re-run the sandbox and PGS claims as assertions
(scalar-reference agreement, `compile_count == 1`); the unit tests pin the host
bookkeeping (entity directory, swap-remove, command buffer, graph colouring).

The `se` developer CLI (`cli/`) is the counterpart to the runtime's `sr`: a thin
Typer layer over a service layer that issues the cmake/ctest calls. It owns no
build knowledge the CMake does not — its job is to resolve the toolchain the
engine consumes (SushiRuntime's bundled clang++ and vcpkg) and snapshot the MSVC
environment on Windows, then drive configure/build/test/run. The same one-way
dependency holds: the CLI reads the runtime's `dependencies/` tree but the engine
never reaches back into runtime source.

## 8. SushiLoop Snapshot: rollback (M3)

`loop/rollback.hpp`'s `RollbackBuffer` is SushiLoop's Snapshot layer: a
fixed-capacity ring of per-tick world snapshots, keyed directly by `Chunk*`.
`capture(world, tick)` walks every archetype (`World::query` with the empty
signature matches all of them, since `signature_contains` treats an empty `need`
as a subset of everything) and every chunk, byte-copying each column's *live* rows
(`count() * column_size`, not the chunk's full capacity). `restore(tick)` writes
those bytes straight back into the same chunks and restores each chunk's live
count via `Chunk::restore_count` — a rollback-only accessor that bypasses
`allocate_row`/`remove_row`'s entity-directory bookkeeping entirely, which is safe
only because of this class's central constraint: **no entity may spawn or be
destroyed, and no chunk/archetype may be created, between a capture and its
matching restore.** A `Chunk*` is stable identity only as long as the chunk it
names keeps existing and keeps holding the same entities in the same rows;
`RollbackBuffer` does not (yet) defend against a violation, which is why the
constraint is documented as a hard scope boundary rather than handled generically.

This also means capture is deliberately *not* the "only what changed" delta the
design note (`docs/slop/SUSHILOOP.md`) ultimately wants — every live chunk is
copied in full every tick. Real per-write dirty tracking needs something upstream
(`Schedule`, `CommandBuffer`) to mark a chunk touched, which nothing does yet;
getting the capture/restore/replay invariant right first, on the whole-chunk case,
is this milestone's scope. `RollbackBuffer` also does not decide *when* to roll
back or replay ticks forward afterward — that orchestration (a game loop, or
later the Net layer's reconciliation) is the caller's job, the same way `Chunk`
does not know what a system is.

## 8.1. SushiLoop Net: loopback reconciliation (M4)

`loop/net.hpp` (namespace `loop::net`) is SushiLoop's network layer, scoped
deliberately narrow: **loopback only**. `LoopbackChannel<Command>` is an in-process,
synchronous stand-in for a client-to-server command link — `client_send(tick,
command)` records the client's own prediction into an `InputHistory<Command>` and
queues it; `server_process(corrector)` drains the queue and returns one `Ack` per
tick, where `corrector` (a caller-supplied callable) stands in for whatever a real
server would compute authoritatively. There are no sockets, no threads, no
serialization, and no general P2P/lockstep protocol here — those are out of scope
for this milestone, the same way M3 scoped out per-write dirty tracking.

Reconciliation (`net::reconcile`) is built entirely on the existing M3 machinery,
not a parallel mechanism: for every ack that disagrees with what the client
predicted for that tick, it corrects the client's `InputHistory` in place (a new
`InputHistory::correct`, an overwrite rather than `record`'s append-only insert),
tracks the earliest disagreeing tick, and — if any correction happened — calls
`RollbackBuffer::restore` on that earliest tick and replays every tick from there
through the caller's current tick, re-applying the (now-corrected) history via a
caller-supplied `apply` function. Ticks the client already predicted correctly are
simply re-simulated identically; only the mispredicted ticks change on replay. This
is the direct generalization of §8's invariant (rollback+replay reproduces an
uninterrupted run) to the case where the *input itself* changes underneath the
replay, not just the tick range.

`net::make_network_id(client_id, tick, spawn_sequence)` is the deterministic-id half
M4 needs: an entity spawned mid-simulation must get the same id on server and
client without a matching round-trip, so the id is derived from facts both sides
already agree on from the numbered command stream itself — which client is
spawning, which tick, and that spawn's index among the client's spawns that tick —
packed into one 64-bit value, rather than assigned by whichever side's spawn call
happens to run first.

`Integration_NetReconciliation`
(`tests/functional/integration/test_net_reconciliation.cpp`) proves the milestone's
key invariant: a client that mispredicts several ticks and later reconciles against
the server's authoritative commands converges to exactly what an uninterrupted
server-only simulation would have produced. `Unit_NetworkId`
(`tests/functional/unit/test_network_id.cpp`) covers `make_network_id`'s collision
behaviour directly. Real transport, and rebasing `RollbackBuffer` across a
structural change caused by network-driven spawns, remain later work.

## 9. SushiLoop core

`docs/slop/SUSHILOOP.md` is the design note; this section is the pointer from
architecture to it. `loop/` holds the first, purely host-side layer of SushiLoop —
plain C++, no runtime or SYCL involvement — that the fixed-tick sim/net/snapshot
work (M1 onward) builds on:

- **`FixedTimestepClock`** (`loop/fixed_timestep.hpp`) turns real elapsed time into a
  whole number of fixed simulation steps plus a leftover interpolation fraction. It
  never reads the wall clock itself — the host accumulates real delta time into it —
  which is what keeps the *number* of ticks a run performs independent of timing
  jitter, a determinism precondition.
- **`RngState`** (`loop/rng.hpp`) is a trivially copyable xorshift128+ generator,
  storable as an ECS component so seeded randomness travels with the world through
  snapshots and rollback instead of living in a hidden global.
- **`InputHistory<Command>`** (`loop/input.hpp`) is the per-tick, numbered command
  buffer shape that networked input capture and rollback replay (M3/M4) will read
  and write; the command type itself is left to the game.

None of this is wired into `Schedule`/`World` yet — that is later SushiLoop
milestones' job. `SE_DETERMINISTIC_FP` (`cmake/ProjectOptions.cmake`, default `ON`)
disables fast-math and FP contraction on the `SushiEngine` INTERFACE target, closing
off two ways a build could make the same floating-point expression evaluate
differently between runs.

## 10. Milestones

- **WP-3 — the ECS layer (done).** Archetype-chunk storage, systems scheduled by
  component access, deferred spawn/destroy via a command buffer, compiled once and
  replayed, validated against a scalar reference. This is the substrate plan's first
  end-to-end milestone (the SushiEngine side of WP-3). It uses dense per-chunk
  columns and whole-column resource identity; graduating to region-keyed sub-chunks
  (runtime WP-2) and device residency (WP-6) follows as scale grows.
- **WP-3 — the physics solver (done).** Graph-coloured Projected Gauss-Seidel over
  distance constraints (§4), generic over the constraint type, validated against a
  scalar reference. Next constraint types (contacts, joints) and rigid-body state
  build on the same colouring and graph structure.
- **SushiLoop M2 — XPBD core (done).** The unified rigid-body XPBD solver (§4.1),
  generalizing the PGS distance constraint to two rigid bodies' attachment points
  with a compliance term, validated against a scalar reference and against the
  original PGS demo's chain shape in the zero-inertia case. Contacts, joints, soft
  bodies, and cloth (M5) are further constraint types over the same solver. The
  editor's "Rigid Body" Inspector toggle (§4.1) is the first consumer, currently
  free-body only (no joints).
- **SushiLoop M3 — Snapshot/rollback core (done).** `loop::RollbackBuffer` (§8),
  per-tick per-chunk byte snapshots with restore, proven against the milestone's
  key invariant (rollback-and-replay bit-identical to an uninterrupted run).
  Scoped to no structural change across a capture/restore pair and whole-chunk
  (not per-write-dirty) capture; both are follow-on work, not this milestone's.
  M4 (network, reconciliation) and M3's own dirty-tracking refinement build on
  this without changing its capture/restore contract.
- **SushiLoop M4 — Network layer (done).** `loop::net` (§8.1): a loopback-only,
  in-process client/server command channel (`LoopbackChannel<Command>`) and
  server-authoritative reconciliation (`net::reconcile`) built on M3's
  `RollbackBuffer` unchanged, plus deterministic entity identity
  (`net::make_network_id`) so client and server agree on a spawned entity's id
  without a matching round-trip. No real sockets/threads/serialization — that is
  explicitly out of scope, same as the whole-chunk capture scoping in M3.
- **SushiLoop M5 — Soft bodies and cloth (done).** `Physics::build_cloth_grid`
  (§4.2): a pinned-top grid of `RigidBody`s connected by structural and shear
  `XpbdDistanceConstraint`s over the existing `PhysicsWorld`, no new solver or
  constraint type. `examples/cloth_demo.cpp` and `Integration_Cloth` validate it
  the same way `xpbd_demo.cpp`/`Integration_PhysicsWorld` validate the hanging
  chain. Volumetric (tetrahedral) soft bodies are out of scope.
- **Rendering (in progress).** A greenfield Vulkan 1.3 renderer behind an RHI
  abstraction (§5), reaching first pixels headlessly (`render_probe`) and now driving
  the editor window; live simulation state enters as the opaque sink node of §5.
- **Editor host shell.** The editor as a host application that runs the game as a
  scene, with play/pause and inspection panels. The `se_editor` shell (SDL2 window +
  Dear ImGui presenting through the Vulkan renderer, `editor/`) currently hosts a
  Unity-style panel set — Hierarchy (with drag-and-drop reparenting, rename, and
  filtering), Inspector, Project browser, a tabbed Text Editor, a Play/Pause/Step
  Toolbar, a Console, and a Statistics panel, all toggled from a Window menu — over an
  editor-owned `Scene` model (`scene_model.hpp`, `editor_context.hpp`), decoupled from
  the runtime behind the windowing, presentation, and ImGui-adapter seams of §5.
  Wiring these panels onto a live World, plus
  a viewport and play/pause, is the remaining work.
