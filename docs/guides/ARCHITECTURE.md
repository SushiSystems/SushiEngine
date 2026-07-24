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
| SushiLoop core | `loop/app.hpp`, `loop/fixed_timestep.hpp`, `loop/rng.hpp`, `loop/input.hpp`, `loop/rollback.hpp`, `loop/net.hpp` | The `Loop::App` authoring API over a fixed-step deterministic loop; plus seeded RNG, per-tick input capture, rollback snapshots, and loopback network reconciliation (see §8, §9, §10). |
| UI | `ui/rect.hpp`, `ui/components.hpp`, `ui/layout.hpp`, `ui/interaction.hpp`, `ui/ui.hpp` | Retained ECS UI (Unity UGUI-shaped): `RectTransform`/`Canvas`/`UIImage`/`UIText`/`UIButton` components, the `resolve_rect` anchor solver, the pointer/click model, and the `UI` façade that builds, lays out, and drives a canvas of buttons (see §11). |
| Physics     | `physics/pgs_solver.hpp`, `physics/graph_coloring.hpp`, `physics/xpbd_solver.hpp`, `physics/rigid_body.hpp`, `physics/physics_world.hpp`, `physics/cloth.hpp` | Graph-coloured PGS solver, its unified XPBD rigid-body generalization, and cloth grids built from it (see §4). |
| Animation   | `animation/skeleton*.hpp`, `animation/clip*.hpp`, `animation/animator_*.hpp`, `animation/blend_tree.hpp`, `animation/avatar_mask.hpp`, `animation/additive.hpp`, `animation/pose_modifier.hpp`, `animation/ik_*.hpp`, `animation/morph.hpp`, `animation/generic_track.hpp`, `animation/humanoid.hpp`, `animation/retarget.hpp`, `animation/edit_preview.hpp`, `animation/animation_database.hpp` | Skeletal-animation stack (phases A0–A9): skeleton/clip/controller/mask assets, the deterministic `animator_step`, the `AnimatorEvaluator` (blend trees, mask-gated layers, additive), the IK / pose-modifier stack, morph + generic tracks, humanoid retargeting, and controller JSON authoring, behind the `IAnimationDatabase` seam (see §12). |
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
orientation (`Quaternion`) and a diagonal, body-local inverse inertia tensor, plus the
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
`physics_params`). The physics itself lives behind the `Simulation::IPhysicsSimulation`
seam (`sim/physics_simulation.hpp`), not in `RuntimeSimulation`: whenever the
physics-driven entity set changes, `tick()` gathers one `RigidBodyDesc` per Rigid Body
entity and calls `set_rigid_bodies`, which rebuilds a free-body `PhysicsWorld` (no
constraints registered — no joints yet) inside the seam, the same "rebuild only when
the input set changes" discipline `Schedule` and `XpbdSolver` follow. That rebuild
snapshots every currently-simulated body's live state first, so toggling physics on one
entity never resets another already-falling body; a brand-new body seeds from its
descriptor pose at rest instead. `RuntimeSimulation` only marshals poses across the
seam and no longer owns a `PhysicsWorld` (single responsibility). `tick()` steps the world under gravity and writes
the solved pose back before the ECS schedule runs, at a fixed assumed ~1/60s frame
— `loop/fixed_timestep.hpp`'s `FixedTimestepClock` is not wired into this loop yet.
`.sushiscene` (`editor/scene_serializer.cpp`) carries `has_physics_body`/
`physics_body` as an independent field pair (not mutually exclusive with camera/
renderer, unlike those two).

`RuntimeSimulation` now owns a `Loop::FixedTimestepClock` (§9) instead of assuming
a fixed ~1/60s frame: `ISimulation::tick()` takes the host's measured real elapsed
time (`real_delta_seconds`) instead of no argument, accumulates it into the clock,
and runs one full step — physics, then the ECS schedule, then the render snapshot
extract — once per whole fixed step the clock reports (zero on a fast host frame,
more than one after a hitch). The physics sub-step duration is derived from the
clock's fixed step (`fixed_dt() / PHYSICS_SUBSTEPS_PER_TICK`) rather than a second,
separately hardcoded constant, so there is one source of truth for tick duration.
The editor's main loop (`editor/main.cpp`) is the one place that reads the wall
clock, measuring real frame time and passing it to `tick()`; the "Step" toolbar
button instead calls `tick(fixed_dt_seconds())` to force exactly one step regardless
of elapsed time. The clock's leftover interpolation fraction is computed and stored
on `RuntimeSimulation` after each `tick()` but has no consumer yet — render
interpolation is a later milestone.

`sim/physics_bridge.hpp` is that `sim/`-level half. `Simulation::PhysicsBody` is an
ordinary component naming which `PhysicsWorld` body an entity owns (`INVALID` until
registered — an entity can carry the component before it has one); this keeps the
mapping in the ECS itself rather than a side table. `Simulation::initial_rigid_body()`
reads an entity's current `Transform`/`Orientation` once, at
`PhysicsWorld::add_body()` time, to seed the body's starting pose.
`Simulation::sync_transforms_from_physics()` is the one direction wired up so far: every
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

**The editor's "Cloth" toggle** (`sim/runtime_simulation.cpp`) wires
`build_cloth_grid` into the live tick loop, following the same route §4.1's Rigid
Body toggle takes rather than `sim/physics_bridge.hpp`: a cloth grid is a single
host-side record — `RuntimeSimulation::Record::has_cloth`/`cloth_params`
(`Simulation::ClothParams`: rows, columns, spacing, compliance) — not one ECS entity per
grid point. Unlike a Rigid Body, whose count is the only thing that forces a rebuild,
*any* `ClothParams` edit forces one (`cloth_dirty_`), because rows/cols change the
grid's body count and there is no meaningful partial state to carry across a topology
change the way a falling free body's position/velocity survives an unrelated Rigid Body
toggle. `tick()` gathers one `ClothDesc` per Cloth entity and calls
`IPhysicsSimulation::set_cloth_grids`; the rebuild is a wholesale replace, every grid
torn down and rebuilt from its current `Transform::position` as the grid origin, at
rest. Inside the seam, cloth lives in its own `PhysicsWorld`, separate from the Rigid
Body world — same constraint type, a second instance — specifically so the
full-rebuild-on-any-change discipline never forces the free-body snapshot-and-carry-over
logic to special-case an entire pinned grid. `IPhysicsSimulation::step` advances both
worlds under the same gravity and sub-step count, driven by the fixed step
`Loop::FixedTimestepClock` reports (§4.1) — there is no separately hardcoded cloth
tick rate.

Cloth's world-space particle positions are exposed read-only via
`IWorldEditor::cloth_particle_positions(id)` (row-major, matching
`Physics::ClothGrid`), and also flow into `RenderScene::cloth_instances`/
`cloth_vertices` every `extract()`: a `Simulation::ClothInstance` per live grid
(rows, cols, and an offset into the shared `cloth_vertices` array) rather than
through `RenderScene::instances` — that type remains one entity, one fixed-mesh
instance, and still cannot express a multi-vertex deforming mesh. `Simulation::
ClothInstance` also carries the owning entity's `id` (for picking) and a `color`,
defaulted to the same fixed tint the wireframe used to hardcode — cloth entities
carry no `Tint` component yet, so there is nothing else to read a colour from. The
editor copies `cloth_instances`/`cloth_vertices`/`id`/`color` into
`Render::ClothStrandView`s each frame (`editor/main.cpp`) and hands them to
`ISceneView::render`'s optional strands parameter. The Vulkan scene view
triangulates each strand's grid (`render/cloth_mesh.hpp`'s
`triangulate_cloth_grid` — two triangles per quad, per-vertex normals averaged
from the adjacent triangles) into a host-visible vertex/index buffer pair and
draws it through the same lit `mesh_pipeline_` Box/Sphere/Cylinder use (already
double-sided, since a triangulated cloth sheet is single-sided geometry that can
flip), so a cloth grid now shades and picks like any other object instead of
drawing as a bare grid-edge wireframe; its outline pass is extended the same way
as the primitive shapes'. `.sushiscene` carries `has_cloth`/`cloth` as an
independent field pair, the same shape as `has_physics_body`/`physics_body`.

### 4.3. Primitive shapes, colliders, and Terrain

Three concerns previously conflated into one hardcoded cube now separate cleanly:
what an entity *looks like*, what it *collides as*, and what drives its *motion*.
`Simulation::ShapeParams`/`ColliderParams` (`sim/simulation.hpp`) are both
`{PrimitiveKind kind; Vector3 params;}` pairs, editor-facing and, like
`ClothParams`, plain host-side bookkeeping on `RuntimeSimulation::Record`
(`has_shape`/`shape_params`, `has_collider`/`collider_params`) rather than ECS
components — neither is read or written by any `Schedule` system, so there is
nothing to gain from an archetype migration. `PrimitiveKind` (`Box`, `Sphere`,
`Cylinder`, `Plane`) is declared in `sim/components.hpp` even though it backs no
component, since it is the vocabulary both authoring structs share.

`IWorldEditor::create_box`/`create_sphere`/`create_cylinder` each spawn a Renderer
entity with a `Shape` and a `Collider` defaulted to the same kind/params — a
created Box is collidable out of the box, and either can be edited or removed
independently afterward. `create_terrain` spawns a large, thin flat `Box` Shape
(the visual) paired with a `Plane` `Collider`, and — critically — **no Rigid
Body/`PhysicsBody`**: nothing integrates Terrain's pose, which is what makes it
immune to gravity, while its `Collider` still marks it as a future narrowphase
participant. **No narrowphase or contact solver reads `Collider` data yet** — it
is pure authoring data for a rigidbody/rigidbody and rigidbody/softbody contact-
resolution milestone that has not been built; see §4.1's note that XPBD today has
no collision detection at all, only distance constraints.

`RenderInstance`/`Render::MeshInstance` both gained `shape_kind`/`shape_params`
(mirrored as `Render::MeshKind` to keep the render seam free of any dependency on
`Simulation`; the editor's per-frame copy loop maps one to the other). `extract()`
gates drawing on `has_shape` **and** `has_renderer` together, because the mesh
(Shape) is now a feature of the Renderer rather than an independent component: the
Inspector edits the mesh kind and dimensions inside the Renderer header, adding a
Renderer attaches a default Box mesh, and removing the Renderer takes the mesh with
it (`editor_panels.cpp`). `create()` makes a truly empty entity — a plain
`Transform`/`Orientation` with no Renderer and no mesh — so a bare "Create Entity"
draws nothing, matching Unity's empty GameObject; the mesh kind is also now editable
(Box↔Sphere↔Cylinder) rather than fixed at creation. The Vulkan scene view (`vulkan_scene_view.cpp`) builds a unit sphere and
a unit cylinder alongside its existing unit cube in `create_geometry()`, and its
draw pass groups instances by `MeshKind` to bind each mesh's buffers once per
group; an instance's `shape_params` become a local scale multiplied into its model
matrix before the MVP push constant (`shape_scale()`), so a default `{0.5,0.5,0.5}`
Box still renders as the historical unit cube.

Entity creation ("Create Empty Entity", Camera, and the Box/Sphere/Cylinder/Terrain
`Objects` submenu) lives in one place, `draw_create_object_menu_items` in
`editor_panels.cpp`, called by the Entity menu and every Hierarchy context menu
(row, filtered-search row, empty space) so they can never drift apart. Copy/Cut/
Paste follow the same pattern via `draw_clipboard_menu_items`: Copy snapshots the
selection through `IWorldEditor`'s getters into `EditorContext::ClipboardEntity`
entries (transform, colour, visibility, and every optional component's attached-
flag/params), Paste replays them through the matching setters onto newly `create`d
entities, and Cut is Copy immediately followed by `destroy` on the originals — no
new engine-side clone primitive, just existing `IWorldEditor` surface replayed.

### 4.3 Collision and soft bodies

Two additions extend the XPBD physics without touching the graph-coloured solver.
`physics/collision.hpp` is the narrowphase: element-parametric collider shapes
(`SphereCollider<T>`, `PlaneCollider<T>`, `BoxCollider<T>`) and pure functions that
return a `Contact` (unit normal from the first shape to the second, positive
penetration depth) for each shape pair. They are geometry only — no runtime, ECS, or
solver dependency — so they are unit-tested directly (`Unit_Collision`).
`physics/contact_solver.hpp` consumes them: non-penetration is an inequality
constraint that only pushes bodies apart, so rather than living in the compile-once
`XpbdSolver` (whose constraint set is fixed) it is a positional projection pass
regenerated from the narrowphase each sub-step, run between `predict` and
`update_velocity`. Because `update_velocity` derives velocity from the post-projection
position, a body that lands on a surface loses its downward velocity with no explicit
restitution term (inelastic contact).

`physics/soft_body.hpp` is the 3D counterpart of the cloth grid (§4.2):
`build_soft_body_lattice` wires an `nx*ny*nz` particle grid held by structural (axis)
and shear (face-diagonal) `XpbdDistanceConstraint`s into a `PhysicsWorld`, so the same
solver runs a deformable block with no new constraint type — a mass-spring soft body
(tetrahedral volume constraints are a later refinement). Both are validated headlessly
(`Integration_SoftBody`, `examples/soft_body_demo.cpp`).

Contacts are now wired into the **live tick**. `PhysicsWorld::step` takes an optional
post-solve callback — run each sub-step between the constraint solve and the velocity
derivation — so the world stays collider-agnostic while a caller injects a narrowphase.
`PhysicsSimulation<T>` uses it: rigid bodies collide as spheres (radius from the entity's
Collider/Shape) against each other and the scene's static `Plane` colliders (Terrain,
supplied every tick via `set_static_planes`), and cloth particles collide against those
planes and against the rigid bodies snapshotted as sphere obstacles — one-way coupling, so
cloth drapes over a rigid without pushing back on it yet. So a body dropped on terrain
comes to rest (its downward velocity absorbed with no restitution term, since velocity is
derived from the post-contact position) and a cloth sheet settles over a sphere. Two-way
cloth→rigid reaction, true box/oriented contacts (bodies collide as spheres today), and a
broadphase are the follow-ups; rendering a deforming surface mesh (cloth and soft bodies
reach the renderer as vertex sets, drawn as wireframes) is the remaining visual work.

### 4.4 Editor authoring: cloth, UI, and custom components

Every capability §4.1–§4.3 added is now authorable in the editor, all through the same
plain-C++ `IWorldEditor` seam and all as attach/detach components.

**Cloth as an object.** `create_cloth` (Entity ▸ Objects ▸ Cloth) makes a bare entity
owning a cloth grid. So a fresh cloth is visible without pressing Play, `extract()`
synthesises a flat resting sheet from `ClothParams` (matching `build_cloth_grid`'s
`origin + (col, 0, row) * spacing` layout) whenever the physics grid has not been built
yet; once the world is played the simulated particle positions take over. The wireframe
already reached the renderer as `ClothStrandView`s (§5), so no render change was needed.

**UI (Canvas + elements).** UI is a host-side record on the entity — `UIElementKind`
(Canvas/Panel/Image/Text/Button) plus a `UIElementParams` that is a uGUI RectTransform
(anchors, pivot, anchored position, size, colour, opacity, text) — the same
no-ECS-migration bookkeeping as cloth, since nothing in the `Schedule` reads it.
`create_canvas`/`create_ui_element` add them from Entity ▸ UI (elements parent to the
selected UI entity so they lay out inside it). The editor draws the tree as a 2D overlay:
each frame `main.cpp` flattens every UI entity into `UIOverlayElement`s (params + the
index of the UI parent) and both viewports paint them with ImGui's draw list
(`paint_ui_overlay`), resolving each rect against the panel rect via a top-left, y-down
variant of the uGUI formula and tinting buttons on hover/press. This is a deliberate
shortcut over a dedicated Vulkan 2D pass — it makes canvases and buttons visible and
editable now; the engine-side `SushiEngine::UI` module (§11) remains the runtime path.

The overlay is also a **RectTransform manipulator** in the Scene view: it is drawn
translucent with outlines (a full-screen canvas therefore no longer hides the 3D scene,
and a canvas is never picked by its body — clicks fall through to the scene or a child),
clicking an element selects it, dragging its body moves it, and dragging a corner handle
resizes it. Each drag inverts the layout formula (`ui_apply_screen_rect`) to write the new
screen rect back as `position`/`size_delta`, and is one undo step (begin/end mirroring the
transform gizmo). The Game view draws the same overlay solid and non-interactive. This is
why `ViewportPanel::draw` takes a mutable `UIOverlay` (elements plus edit-mode and pick/
edit outputs) rather than a const element array.

**Custom (script) components.** The engine has no scripting VM, so a "custom component"
is authoring data: `ScriptComponent` (a `type_name` and a list of `ScriptField`s, each a
tagged float/int/bool/vec3/colour/text value). Instances live per entity on
`RuntimeSimulation::Record::scripts`; the *catalog* of definitions lives in
`EditorContext::script_catalog` and is repopulated from any script found while a scene
loads, so the Add Component ▸ Scripts menu survives a round-trip. "New Script…" scaffolds
a `<Name>.hpp` C++ system stub in the project (Apache header, a `struct`, and a commented
`app.system<…>().each(…)` registration), opens it in the Text Editor, and registers +
attaches the new type. Both UI params and script components serialize with the scene and
travel through the copy/paste clipboard alongside the other optional components.

## 5. The render seam

Rendering does not belong inside the runtime — the runtime knows no graphics, just
as it knows no math. The renderer is a separate compiled library (`render/`,
`include/SushiEngine/render/`), a **greenfield Vulkan 1.4** backend behind a
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
  set (each tagged with a `MeshKind` — Box, Sphere, or Cylinder — plus per-kind
  shape params) plus a ground grid, drawn from a `CameraView`. The Vulkan
  implementation (`render/rhi/vulkan/vulkan_scene_view.*`) is double-buffered — the
  frame being sampled by the UI is never the frame being drawn — and leaves its
  colour image shader-readable so the editor samples it with `ImGui::Image`. It
  exposes only the sampler/view handles a UI backend needs, never a full descriptor
  set. Alongside the shaded image it renders a second `R32_UINT` **id target**
  carrying each instance's picking id, copied to a host buffer each frame so
  `pick(x, y)` resolves a click to the entity under the cursor (GPU id-buffer
  picking); `render` also takes the selected id, which the mesh shader highlights,
  and an optional `ClothStrandView` list, triangulated per frame and drawn shaded
  and pickable through the same mesh pipeline Box/Sphere/Cylinder use (see §4.2).
- **Lighting, materials, and the sky (§5.1).** `render` also takes a
  `const Render::Environment&` and the camera's world position, and draws the frame
  in three HDR passes rather than one, giving PBR meshes and a WGS84 planet with a
  physical atmosphere.

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
simulation's camera. `FlyCamera` and `CameraController` store and compute in `Scalar`
(§6), so the camera pipeline runs at the same precision as the rest of the engine —
`InputState` fields stay `float` (ImGui pixel deltas) and are `static_cast`-ed to
`Scalar` at the computation boundary. So the same panel serves both viewports, the
controller depends on no input source and stays unit-testable, and a new camera kind is a new
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
applies the live-effective ones (theme, camera speed). The precision setting selects the
physics-solve precision of §6 (a live runtime choice that rebuilds the running
simulation from a scene snapshot); the boundary `Scalar` itself is always double.

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

### 5.0. The render graph

`ISceneView`'s Vulkan implementation is not a sequence of hand-recorded passes but a
**frame graph**. Each frame the scene view builds a `Frame::FrameContext` — camera,
extent, quality tier, draw list, and the handles of this frame's targets — and asks
each pass to register itself; the graph then derives everything that used to be
written by hand.

The quality tier does not reach the passes raw. Once per frame the scene view runs
`resolve_quality` (`render/frame/quality.cpp`, public type `QualityParams`), which
turns `RenderQuality` into the concrete parameters passes actually read — soft-shadow
tap counts, contact-march length, cloud budget, the coarsest variable-rate tile, the
shadow atlas size and cascade count, and which advanced BRDF lobes are evaluated. The
policy lives in that one file so the tier cannot mean one thing in the shadow pass and
another in the cloud pass; a pass reads resolved parameters, never the enum. The
authored settings are the High baseline, so High resolves to the request verbatim and
a lower tier scales the expensive half down from it.

- **`render/graph/`** — `RenderGraph`, `RenderPassBuilder`, `PassContext`,
  `TextureHandle` / `BufferHandle`, and the access vocabulary. A pass declares *what*
  it touches (`read`/`write`/`color_attachment`/`depth_stencil_attachment`) and the
  graph derives *how*: `resource_state.*` maps each declared access to exactly one
  (stage, access mask, layout) triple, so every `VkImageMemoryBarrier2` and every
  `vkCmdBeginRendering` scope — viewport and scissor included — is generated, never
  authored. `compile()` culls passes whose outputs nothing reads, then walks the
  schedule assigning physical resources: a transient is returned to its pool the
  moment its last reader is scheduled, so two disjoint lifetimes land on one
  allocation. That is the graph's memory aliasing.
- **`render/resources/`** — `TexturePool` / `BufferPool` (the physical backing, one
  set per frame slot so a pool never hands this frame a resource the previous frame's
  submit is still reading), `DescriptorAllocator` (per-slot pools reset wholesale, so
  a resize rebuilds no descriptor set), `DescriptorHeap` (the bindless
  update-after-bind array bound as set 1), `PipelineCache` (a `VkPipelineCache`
  persisted to disk) and `GraphicsPipelineFactory` (four independently cached
  `VK_EXT_graphics_pipeline_library` halves, with monolithic creation as the fallback
  when the extension is absent), `SamplerCache`, and `ShaderLibrary`. A pipeline never
  makes a pass wait on its own best version: `GraphicsPipelineFactory` hands out a
  `PipelineHandle` pointing at the fast-linked (GPL) pipeline the instant it exists,
  while a background thread rebuilds the same pipeline monolithically and swaps the
  handle's atomic pointer (release/acquire) once it is ready; the superseded pipeline
  retires after a delay sized past every view sharing the factory's clock, the same
  reasoning `TextureLibrary`'s streaming retirement below uses. `DescriptorWriter` and
  `bind_descriptor_set()` are the matching write/bind seam every pass, the compute/RT
  passes, and the scene layout route through, so the announced
  `VK_EXT_descriptor_heap` lands as a swap behind these two functions rather than a
  sweep of every pass.
- **`render/passes/`** — one file per pass, each honouring the same
  `register_pass(graph, frame)` contract and owning only its own pipelines: the
  environment capture, the opaque geometry pass, the shading-rate mask, the sky pass,
  the half-resolution cloud march, the cloud composite, the temporal resolve, the
  **post-processing stack** (depth of field, motion blur, auto-exposure metering, and
  bloom), the display transform, the spatial anti-aliasing filter, and the picking
  readback. Adding an effect is adding one of these and registering it; no neighbouring
  pass changes. A pass that this frame's settings do not call for registers nothing, so
  the chain reconfigures without any pass learning what the others do.
- **The post-processing stack** runs after the temporal resolve: `DofPass` and
  `MotionBlurPass` (gather-based, tier-gated) each hand their output to the next stage;
  `AutoExposurePass` builds a luminance histogram the scene view reads back to adapt the
  exposure; `BloomPass` builds a Karis-averaged mip pyramid into a half-resolution target;
  and `TonemapPass` is the single display transform that applies the resolved exposure, a
  colour grade, one of three tone curves (AgX / ACES / Khronos Neutral), the lens effects,
  and the sRGB encode with a blue-noise dither. All of them read one `PostProcessUniforms`
  block (scene-set binding 31) the scene view fills from `RenderSettings::post`, which the
  editor's **Post Process** window authors — the passes never name the editor.
- **The GPU-driven geometry path** (Phase 10) replaces the CPU's one-draw-per-instance loop
  with two device buffers and a cull dispatch, so the CPU cost is flat in the number of
  distinct meshes rather than the number of instances. `InstanceSystem` (`render/scene/`)
  packs every opaque mesh instance into a per-frame `GpuInstance` storage-buffer record —
  camera-relative transform, bounding sphere, and the material/motion/pick indices the classic
  draw used to push — and groups them by mesh into per-mesh buckets, one host-mapped buffer
  per frame slot in the exact shape `MaterialSystem` and `MotionSystem` already use. `CullPass`
  then runs before the depth prepass: one thread per instance tests the bounding sphere against
  the view frustum, its own on-screen diameter (a screen-coverage LOD gate that drops instances
  too small to matter), and the occlusion pyramid, then compacts the survivors per bucket and
  writes one `VkDrawIndexedIndirectCommand` per bucket whose instance count it decides — no CPU
  readback in the loop (a survivor counter is read back one frame late only for the editor's
  cull statistics). `OcclusionPass` owns that pyramid: a persistent max-Z (farthest-depth) mip
  chain built after the depth prepass, the conservative twin of the `HizPass` nearest-depth
  pyramid the SSR trace marches (nearest is right for reflections and wrong for culling). It
  lives outside the render graph and is read at the *start* of the next frame by the cull —
  reprojected with the previous view-projection and the eye delta — so an instance is tested
  against the depth the last frame actually rendered; a freshly (re)created image clears to
  "far" so nothing occludes until real depth lands, which self-corrects with no popping and no
  readback. The draw itself runs through `mesh_gpu.vert`, the twin of `mesh.vert`: it reads the
  model matrix, material index, and picking id from the instance record (an indirect draw
  carries no push constant), indexing the cull pass's compacted survivor list from the one
  value still pushed per bucket — the bucket's base into that list. Its instance and compacted
  buffers ride a **set-2** descriptor set (`SceneLayout::INSTANCE_SET`), so the bindless heap
  keeps set 1 and both vertex shaders feed the same `pbr.frag`. The whole path is **two-path**:
  the scene view takes the GPU-driven route when the tier permits it (`QualityParams::gpu_driven`
  — off on Low, on for Medium/High/Ultra), the author has left `GpuCullingSettings::enabled`
  on, the bindless heap is present, and nothing is selected; anything else falls back to the
  classic CPU per-instance draw (a selection keeps it so the outline's stencil mask still
  works), while the cull machinery stays primed so the pyramid remains fresh for when it
  resumes. The editor's **GPU Culling** window authors `RenderSettings::gpu_culling` —
  enable, frustum, occlusion, min-screen-diameter, a debug frustum freeze, and the per-frame
  statistics — and, as with post-processing, no pass names the editor.
- **Shadows beyond the atlas** (Phase 12.3). The punctual shadow atlas holds a fixed number of
  tiles, and a light that does not fit one used to shade unshadowed. It now gets shadowed
  stochastically instead: `clustered_lighting.glsl` splits a cluster's lights into those with a
  tile (filtered against it as before) and those without, importance-samples a tier-scaled few of
  the latter per pixel, and marches the GI distance clipmap toward each with `sdf_visibility()`,
  weighting by one over the probability it was picked. Because that estimator is unbiased, the
  temporal resolve is the denoiser — no new pass, no new history. The field is the one
  `SdfProbeTracer` already builds for probe GI, offered through `IProbeTracer::visibility_field()`
  so the shading pass depends on a field existing and never on which tier produced it; it reaches
  every shading pipeline through the bindless heap's volume array, because the per-frame push set
  is full at its guaranteed 32 bindings. The consequence worth stating plainly: the number of
  shadowed lights stops being a memory budget and becomes a sample budget.
- **Two queues, one schedule** (Phase 11). The graph does not assume a single queue. A pass
  may declare `PassQueue::AsyncCompute`, and `compile()` splits the schedule wherever the
  queue changes into `Submission`s — one command buffer each, recorded and submitted in
  order. What orders them is derived, like the barriers, from the resource declarations: a
  submission waits on the latest earlier submission *on the other queue* that produced what it
  consumes or consumed what it overwrites (a dependency on its own queue is already ordered by
  submission order, and per-queue timeline values rise monotonically, so one wait covers every
  earlier one). The same walk marks which resources both queues touch, and only those are
  allocated with concurrent sharing (`TextureDesc::cross_queue`) — the graph cannot transfer
  queue-family ownership, and paying for concurrent sharing on every transient would cost
  attachment compression for nothing. Two conditions a flagging pass owes the graph: everything
  it produces must be *declared* (a pass that hand-barriers a resource it owns would leave its
  consumers unsynchronised), and what it shares must be a graph transient rather than an
  import, whose sharing mode the graph cannot change. Flagged today: the clustered light cull
  and the GTAO horizon march. Gated three ways — a compute queue family distinct from graphics,
  `QualityParams::async_compute` (off on Low), and `FrameDeliverySettings::async_compute` —
  and with any of them absent every pass records on the graphics queue exactly as before.
- **Frame delivery** (`ViewResources`). Each queue carries one monotonic **timeline
  semaphore**: every submission signals its own value, waits on the value it depends on, and a
  frame slot is reusable once both timelines have reached what that slot submitted. Command
  buffers are handed out per submission, not per slot, so a frame that compiles to several
  submissions never records two of them into one buffer. How far the CPU may run ahead
  (`FrameDeliverySettings::frames_in_flight`, 2 or 3) and how frames are paced onto the display
  (`PresentMode`, applied through `IWindowRenderer::set_present_mode`) are settings; all slots
  are allocated up front, so changing depth costs an idle and no reallocation. The editor's
  **Frame Delivery** section authors the block.
- **Reconstruction is an interface** (`render/frame/upscaler.hpp`). Rendering below the output
  extent and reconstructing back up is a contract — colour, depth, motion, history, jitter,
  exposure, and the two extents — that `TaaPass` is simply the first implementation of. A
  vendor upscaler (FSR/DLSS/XeSS) lands as another implementation rather than as a fork of the
  frame loop; `Frame::upscaler_availability` reports which backends this build carries, and one
  it does not resolves back to the built-in temporal reconstruction with the reason surfaced in
  the editor.
- **`render/interop/`** — a device-local buffer whose memory another API can import by OS
  handle: a dedicated, exportable allocation stamped with the device UUID that
  `RenderDeviceDesc::required_uuid` already selects the graphics device by, exposed through the
  public `interop.hpp` with no Vulkan, SYCL, or platform type in sight. The renderer *exports*
  only; importing belongs to whoever owns the other API, which for SushiRuntime means the
  runtime — the dependency points one way.
- **`render/scene/`, `render/geometry/`, `render/textures/`** — the shared scene
  uniform block and descriptor/pipeline layout, the built-in unit meshes and the
  per-slot soft-body buffers, and the cloud noise volumes. The noise set (two Perlin-
  Worley volumes, an anisotropic cirrus volume, and a weather map) is generated by
  compute dispatches at bring-up rather than on a CPU thread pool.

Two supporting mechanisms make the graph usable day to day. `GpuProfiler` brackets
every executed pass with timestamp queries and resolves a slot's results at the
point its fence has already been waited on, so per-pass GPU times cost no stall;
`ISceneView::pass_timing_count()` / `pass_timing()` surface them, the main loop copies
each visible viewport's breakdown into the editor context, and the Statistics panel
lists them, which is what lets every later pass be landed against a measured budget. `ShaderLibrary` ships build-time SPIR-V but also watches
`render/shaders/` when it exists: an edited shader is recompiled in process with
glslang, the device is idled, and every pipeline is rebuilt — a compile error leaves
the previous module in place and reports on stderr.

`VulkanSceneView` is left as the orchestrator: build the frame context, register the
passes, compile, submit. It records no barrier and opens no render pass of its own.
The precision invariants are unchanged and inherited by every pass — camera-relative
rendering (the eye subtracted in double before the float cast) and reverse-Z.

### 5.0.1. Materials, textures, and image-based lighting

The vertex format carries position, normal, tangent, two UV sets, and a vertex
colour — the minimum that makes normal mapping, parallax, and a detail set possible,
and therefore the thing everything else in this section rests on. A zero tangent is a
legal value meaning "none authored"; the shader falls back to screen-space
derivatives, so an imported mesh without tangents still normal-maps.

**The material** (`include/SushiEngine/render/material.hpp`) is the authored surface:
albedo, packed metallic-roughness (ORM supported), normal, height, occlusion,
emission, a Unity-style detail set, per-set tiling/offset, the advanced lobes
(anisotropy, clearcoat, sheen, transmission), and the rendering state (surface type,
cull mode, blend mode, render queue, shadow flags, sampler settings). Every map is
optional. That is not a convenience: an unset slot resolves to one of four neutral
default textures — white, flat normal, black, neutral MR — so the shader samples
unconditionally and branches only on behaviour a default *cannot* stand in for, which
travels as flag bits. A material with no textures shades exactly as it did before
textures existed.

**`render/material/`** holds the machinery:

- `AssetLibrary` — the device's shared store. Textures, meshes, the bindless heap, the
  shader library, and the pipeline and sampler caches are device-level, so two
  viewports drawing the same model share one upload and one pipeline. It is also the
  implementation of the public `IAssetLibrary` seam (`IWindowRenderer::assets()`),
  which is how a host loads a texture or a glTF file without seeing a Vulkan type.
- `TextureLibrary` — decode (stb), upload with a GPU-generated mip chain, bindless
  registration, path deduplication. Residency is mip-based against a budget: a texture
  that will not fit is uploaded from a lower mip and upgraded later, at most one per
  frame. Uploads never block the frame. Where `host_image_copy` is available (the
  Vulkan 1.4 floor makes it common, though it stays an optional feature), a mip chain
  is box-filtered on the CPU and copied straight into the optimal-tiled image with no
  staging buffer, no submit, and no fence — the superseded image and heap slot are
  instead reclaimed once a shared frame counter has advanced past every view that
  could still be sampling them. The staging-plus-blit path remains the fallback: each
  of its uploads carries its own fence, and its superseded image, staging memory, and
  heap slot are reclaimed only once that fence signals.
- `MaterialSystem` — packs authored materials into a per-frame storage-buffer array of
  fixed-layout records holding heap indices. A draw then carries one material index in
  its push constant rather than a payload of parameters, which is the shape the
  indirect-draw work in Phase 7 needs.
- `gltf_importer` — one mesh per primitive, baked into its node's world transform so a
  multi-part asset assembles without a scene graph on the render side. The core
  material maps across directly; `KHR_materials_*` drives the advanced lobes;
  spec-gloss is converted (lossily, and deliberately so); missing tangents are
  generated.

**Image-based lighting** (`render/passes/ibl_pass.*`) is captured from the engine's own
analytic sky rather than an imported HDRI: six 90-degree views of the atmosphere are
rendered into a cubemap with the same `sky.frag`, GGX-prefiltered into a roughness mip
chain, and cosine-convolved into an irradiance cube. A split-sum BRDF LUT is generated
once at bring-up — it depends on nothing but the BRDF. Capture is rate-limited and
gated on the sun or the atmosphere having measurably moved, so a slowly turning sun
costs almost nothing, and the lighting tracks time of day for free.

The BRDF itself (`render/shaders/pbr_common.glsl`) is no longer single-scatter:
height-correlated Smith visibility, **Kulla-Conty multi-scatter compensation** driven
by the same BRDF LUT the IBL needs (which is why rough metals stop losing most of
their energy), roughness-aware Fresnel, specular occlusion from AO, and a dielectric
F0 derived from the material's index of refraction instead of the 0.04 every plastic
uses.

Indirect diffuse is no longer a single global value. **Probe-volume global illumination**
(`render/gi/`, Phase 6) places three nested camera-relative cascades of irradiance probes
(32×8×32 each, 4/8/16 m spacing, ~124/248/496 m reach), every one snapped to its own world
grid (so probes hold fixed world positions and do not swim as the camera moves). Each probe
stores the same nine SH coefficients the IBL build produces, and `pbr.frag` walks the
cascades finest-to-coarsest, blends the eight probes around a surface trilinearly in the
first cascade that contains it in place of the single global set — falling back to the
global environment SH beyond the coarsest cascade or when GI is off, so nine coefficients
always shade a pixel. `IrradianceVolumePass` owns the probe SH grid (scene set-0 bindings
29–30, pass-owned and hand-barriered like the IBL SH buffer) and relights it each frame
through a pluggable `IProbeTracer` — the strategy seam (DIP) that decides how a probe
gathers incident radiance. The default `SdfProbeTracer` (Tier A, all hardware) rebuilds a
coarse scene distance clipmap (64³) each frame from the frame's analytic primitives and the
per-mesh signed-distance bricks `MeshRegistry` bakes at import (`mesh_sdf_baker`), then
sphere-traces it per probe: a hit contributes one coloured bounce plus any emitted radiance
(a parallel emissive clipmap injects `material.emissive` at probe rate — lights from
materials, no separate light path), a miss the distant environment, projected back to SH.
The trace is amortized — a round-robin quarter of all probes per frame, forced full only
for a cascade that shifts a cell (the coarse grids shift far less often) or on a sun move —
and rough reflections fall back to the same probe cache before the sky.
`EnvironmentProbeTracer` (broadcast the environment SH, no trace) is the cheaper floor-tier
strategy behind the same seam. Tier-gated (High/Ultra) and author-gated
(`Environment::gi`).

### 5.0.2. The temporal core

Everything that relates one frame to the next lives behind one small block,
`Scene::TemporalUniforms` at set 0 binding 9, deliberately separate from the scene
block: that one describes the world and is declared as a truncated prefix by most
shaders, which makes appending to it fragile, while this one has different readers and
a different reason to change.

- **Motion vectors.** The geometry pass writes a third target holding each pixel's UV
  displacement since the previous frame. The current clip position is `gl_Position`;
  the previous one comes from `Scene::MotionSystem`, a per-frame array of previous
  transforms keyed by picking id that a draw indexes with one push-constant slot — the
  same shape as the material array, and for the same reason. Each side is camera-
  relative against its own frame's eye, subtracted in double before the float cast, so
  the camera's own translation shows up in the motion vector without planet-scale
  metres ever entering single precision. Pixels no draw covered carry no motion vector;
  the resolve reprojects those from the view ray instead, which is exact for the sky.
- **Jitter.** `Frame::frame_jitter` walks a Halton (2, 3) sequence and the offset is
  added to the projection's third column and to the sky and cloud ray directions —
  nowhere else, so no world-space value moves. The sign is not free: the third column
  is scaled by view z and the perspective divide is by −z, so a positive entry shifts
  the result negative, and the ray offset and the motion vector's jitter removal are
  both matched against that.
- **The resolve.** `TaaPass` dilates the motion vector to the closest surface in a 3×3
  neighbourhood, clips the reprojected history into that neighbourhood's colour
  distribution in YCoCg, reconstructs both inputs with a Catmull-Rom filter, blends
  under Karis tone weighting, and sharpens to offset the temporal softening. The
  history lives at the *output* extent while the scene is rendered at the internal
  extent, which is what makes rendering small and resolving large an upscale rather
  than a blur. Two history images ping-pong by frame parity, not by frame slot: with
  two frames in flight, the other slot's image is two frames old.
- **Dynamic resolution.** `Frame::ResolutionController` maps the GPU time the Phase 0
  timers measured to an internal render scale, dropping quickly on an overrun and
  recovering gradually, quantised to sixteenths so a scale hovering near a boundary
  does not reallocate every transient. Picking follows the internal extent and `pick()`
  scales the click into it.
- **Variable rate shading.** A compute pass derives a per-tile rate image from the
  previous frame's luminance contrast and this frame's motion, and the sky pass binds
  it through `RenderPassBuilder::shading_rate_attachment`. It sits between the geometry
  and the sky because that is the only non-circular ordering — the mask wants the
  motion vectors the geometry pass writes — and because at planet scale the sky is the
  pass worth steering. Absent `VK_KHR_fragment_shading_rate` the handle stays invalid
  and every declaration of it is a no-op, so no pass branches on support.

`RenderSettings` (`include/SushiEngine/render/render_settings.hpp`) is the public seam
for all of it, kept apart from `Environment` on the same principle: `Environment`
describes the world being drawn, `RenderSettings` describes the machinery drawing it.

### 5.0.3. Shadows

The sun is shadowed at three scales, because no single mechanism spans the range a
planet-scale scene does.

- **Cascades** (`Scene::fit_shadow_cascades`, `ShadowPass`) carry the body of it. Four
  cascades share one atlas as a two-by-two grid of tiles, so four cascades cost one
  image, one pass, one barrier, and one profiler entry — the cascade being drawn changes
  with a viewport and a push-constant slot, never a rebind. The split positions are the
  practical scheme; each cascade is bounded by a **sphere** rather than by its frustum
  corners and its light-space origin is **snapped to whole shadow texels**, which
  together are what stop the edges crawling: a sphere's extent does not change with
  rotation, and a texel-aligned origin cannot shift sub-texel. The whole fit is
  camera-relative. The maps are orthographic and therefore linear in depth, so unlike
  the camera they do **not** use reverse-Z — there is no precision to redistribute.
- **A depth prepass** (`DepthPrepass`) runs the same `mesh.vert` the shading pass does,
  with no fragment stage. Sharing the shader is a correctness requirement, not a
  convenience: the opaque pass tests against depths it recomputes itself, and only the
  same shader guarantees the two agree bit for bit.
- **Screen-space contact shadows** (`ContactShadowPass`) march that depth buffer toward
  the sun over a distance measured in metres, recovering the contact a cascade texel is
  orders of magnitude too coarse to resolve. This is why the prepass exists — the answer
  has to be known before the surface is shaded.
- **The clouds** shadow lit surfaces through `cloud_shadow_common.glsl`, which takes the
  same transmittance the sky pass marches for the analytic ground out of the weather map
  for two fetches, with the sky pass's parameterisation and warp so the two agree on
  where a cloud is.
- **A traced ray** (`RayTracing::SceneAccelerator`, `RayTracedShadowPass`) replaces all
  of that on the Ultra tier, where the device offers `VK_KHR_acceleration_structure` and
  `VK_KHR_ray_query`. Two levels: a bottom-level structure per distinct mesh, built once
  and kept, and a top-level rebuilt each frame from 64-byte instance records — which is
  why a thousand copies of one mesh cost a thousand records rather than a thousand
  rebuilds. Every structure a frame needs is created and sized *before* any build is
  recorded, because they share one scratch buffer and growing it between builds whose
  scratch address is already baked in is a use-after-free the GPU finds long after the
  call that caused it. The result is a screen-space mask, not a term inside the material
  shader: only the trace shader then needs the ray-query extension, so the material
  shader stays one build that runs everywhere. It is also the one place in the renderer
  that writes its own barrier — an acceleration structure is neither an image nor a
  buffer, so there is no declaration the graph could have derived one from.

`Scene::ShadowUniforms` at set 0 binding 10 carries all of it, separate from the scene
block for the same reason the temporal block is.

### 5.1. Lighting, materials, and the sky

The environment the renderer lights against is a neutral seam,
`render/environment.hpp`, depending only on `core/types.hpp` — so the simulation
authors it and the renderer consumes it without either depending on the other. It
holds the sun (`DirectionalLight`), the sky's derived `CelestialLight` list, the `Wgs84` ellipsoid (equatorial radius
6378137 m, inverse flattening 298.257223563), and the `AtmosphereParams`,
`PlanetParams`, the genus-driven `Cloudscape` (up to `CLOUD_MAX_DECKS` `CloudDeck`s, each a WMO `CloudGenus` resolved through `cloud_genus_profile`), `StarParams`, and metallic-roughness `Material` that
describe how the planet is lit and surrounded. The simulation carries one
`Environment` on `RenderScene` and a `Material` per `RenderInstance` (its albedo kept
in sync with the entity's `Tint`), authored through `IWorldEditor`'s
`environment()`/`set_environment()` and `material()`/`set_material()`; the editor's
**Environment** panel and the Inspector's material section drive them.

`ISceneView::render` takes the `Environment` and the camera's world position and draws
the frame in **three HDR passes** (the Vulkan scene view's targets are now linear
`R16G16B16A16_SFLOAT`, resolved to the `R8G8B8A8_UNORM` image the editor samples):

1. **Opaque** — the grid, meshes, and cloth into the HDR colour + `R32_UINT` id +
   depth targets, drawn in the same **camera-relative space** the sky pass uses. Each
   model's translation has the camera eye subtracted in double precision before the
   `float` cast (`make_push`), cloth points are offset by the eye as they are written
   to their buffer, and the uploaded view matrix carries no translation — so a mesh
   far from the ECEF origin reaches the GPU as a small offset from the camera and
   keeps full `float` precision instead of jittering on the ~16 m grid a raw 1.5e8 m
   coordinate collapses to. `pbr.frag` shades each mesh with a Cook-Torrance
   metallic-roughness BRDF (GGX distribution, Smith geometry, Schlick Fresnel) lit by
   the sun and an ambient floor, taking the view direction as `-v_world_position`
   (the camera is the origin of this frame), and reads a per-frame scene uniform block
   — the scene view's first descriptor set, shared by every pass.
2. **Sky** — a fullscreen pass (`sky.frag`) that works in **camera-relative
   space** (the camera is the origin; the planet centre arrives relative to it, so
   planet-scale metres never leave double precision on the CPU). It intersects the
   WGS84 ellipsoid for the lit ground (onto which the cloud stack casts its combined
   shadow), and evaluates the atmosphere's Rayleigh + Mie scattering not as a
   per-pixel march but through the **Hillaire 2020 LUT stack** a preceding
   `AtmosphereLutPass` builds (`render/passes/atmosphere_lut_pass.*`,
   `atmosphere_common.glsl`): a **transmittance LUT** and a **multiple-scattering LUT**
   (view-independent, change-gated), a per-frame **sky-view LUT** (the background sky in
   the camera's local frame), and a per-frame **aerial-perspective froxel volume** (the
   air in front of each mesh). A background pixel is a sky-view fetch, a mesh within
   32 km an aerial-volume fetch, and the analytic ground and far geometry keep a march
   that reads the sun's transmittance and the multiple scattering from the LUTs. Over
   the top a `VolumetricFogPass` (`render/passes/volumetric_fog_pass.*`,
   `fog_scatter.comp`) marches an authored ground-hugging fog — a global height term
   plus up to eight local box/ellipsoid volumes — into a second froxel volume folded
   over every pixel in the composite. It adds the real bright-star catalogue — each of
   the ~60 stars in
   `astro/star_catalog.hpp` (J2000 position, magnitude, B-V colour) is rotated by
   the ephemeris into the observer's local sky and drawn as a point at its true
   direction, so the constellations are recognisable rather than a random field —
   composited over the opaque scene by the sampled depth, so geometry occludes the
   sky and thin air is added over it as aerial perspective. The stars are faded by
   the atmosphere's optical depth, so as the camera climbs the blue sky thins into
   black space and the stars emerge: the near-surface-to-orbit transition falls out
   of the physics rather than a hard switch.
3. **Cloud** — the genus-driven volumetric cloudscape, ray-marched in a **dedicated
   half-resolution pass** (`cloud.frag`) into an HDR target holding
   (`scattered.rgb`, `transmittance`). It marches the union shell of all enabled genus
   decks in one pass, so the decks composite, self-shadow (a cone light march plus a
   multiple-scatter energy term), and stop at the opaque depth or analytic ground so
   clouds draw over terrain. A two-tier LOD (dense in-atmosphere, coarse from orbit), a
   cheap-density probe with distance LOD, and empty-space skipping keep the march
   affordable; running at a quarter of the pixels is the largest single saving. The
   tonemap pass upsamples and composites the result over the sky.
4. **Tonemap** — `tonemap.frag` upsamples the half-res cloud target and composites it
   over the sky (`sky * transmittance + scattered`), then applies exposure, the ACES
   filmic curve, and a gamma encode, resolving the HDR composite into the sampled LDR
   image.

The planet is drawn analytically in the sky pass rather than as tessellated terrain,
which is why a real WGS84 Earth and its atmosphere render with no level-of-detail
machinery; the ground grid the editor shows sits tangent to the ellipsoid at the local
origin. The existing floating-origin / ECEF types (§6) anchor the camera's world
position that the sky pass places the planet relative to.

Depth is **reverse-Z with an infinite far plane** on a `D32_SFLOAT_S8_UINT` buffer:
`perspective` maps the near plane to clip depth 1 and infinity to 0, the pipeline clears
depth to 0 and compares `GREATER_OR_EQUAL`, and floating-point precision is spread almost
uniformly across the whole range — so one camera resolves a few-centimetre prop and a
planet 10⁷ m away in the same frame without z-fighting, and nothing is ever clipped for
distance. The sky pass reconstructs view-z straight from the projection matrix, so it is
independent of the depth convention; its "geometry is here" tests key on `depth > 0`.

### 5.2. The solar system: ephemeris, gravity, and frames

The `include/SushiEngine/astro/` headers place and move bodies. They are pure
double-precision host code that fills the neutral `Environment` — the engine ships no
device code here. `julian_date.hpp` and `orbital_elements.hpp` propagate the Standish
Keplerian rows; `celestial_bodies.hpp` catalogues each body's radius, colour, pole,
`SurfacePreset`, and `ring_extent` (Saturn's ring span, zero for every other body);
`ephemeris.hpp`'s `fill_environment_sky` assembles them into the local
sky each frame and selects the **dominant body** (the one whose surface is the analytic
ground) by the surface hand-off altitude.

**Saturn's rings** are drawn the same analytic, mesh-free way as the bodies themselves.
`fill_environment_sky` threads `ring_extent(BodyId::Saturn)` onto the body's
`CelestialBody::ring_inner_metres`/`ring_outer_metres` (oriented by the body's real
`pole`, i.e. its J2000 equatorial plane) and, once Saturn is the dominant near-field
planet, onto `Environment::planet_ring_*`. `scene_uniforms.cpp` packs the far-field radii
into the body record's previously-unused lanes and appends one `planet_ring` vec4 for the
near-field case (after the block's arrays, so the shaders that share the scene block and
read only its earlier fields keep their offsets). `sky.frag` then ray-tests the equatorial
annulus in both regimes — the far-field body loop (the ring resolves as the camera nears
Saturn past the far-disk LOD) and the near-field planet — shading it with banded opacity
(C/B rings, the empty Cassini division, the A ring with its Encke gap), fine ringlets and
self-gravity-wake clumping, a back-scatter opposition surge, translucency over the disk,
and the planet's shadow cast across the ring.

Because every body is placed with its true direction, distance, and angular radius in one
frame, **eclipses fall out of the geometry** rather than being scripted. `fill_environment_sky`
computes the solar-eclipse coverage once — the circle-circle overlap (`disk_overlap_fraction`)
of the Sun's disk by any nearer body — and hands it to the renderer as a single scalar
(`Environment::solar_eclipse`, packed into `sky_counts.w`); the sky, PBR, and cloud passes
all dim the direct sun by it, so the whole scene dusks toward totality together. The
lunar eclipse is the mirror case: the Moon's disk against Earth's umbra at the anti-solar
point, folded into the Moon body's colour and brightness on the CPU (a coppery, dimmed
disk) with no shader involved. Both are Earth-consistent and ephemeris-driven, so they
occur only at the real alignments.

**Every body in that sky is also a light.** The ephemeris turns the body list into an
ordered `CelestialLight` array (`Environment::lights`), and the Sun is not a special case
in it: an emitter's irradiance is authored, a reflector's follows from the definition of
geometric albedo — incident irradiance times `albedo * (radius/distance)^2` times
`phase_brightness`, which is a Lambert sphere for smooth bodies and Allen's lunar fit
(opposition surge included) for regolith ones. Nothing names a body, so the Moon over
Earth, Jupiter over Europa, and earthshine on the Moon are one code path, and the numbers
are real — a full Moon lands near `2.8e-6` of sunlight. The list is ordered by what each
light *delivers* at the camera (irradiance weighted by elevation), which is what lets the
single shadow-cascade atlas belong to the Sun by day and to the dominant reflector after
sunset; the PBR pass and the analytic ground both loop the array, shading the rest with
the same BRDF minus the cascades. Because the lights are physical, so is the exposure
path: the auto-exposure histogram's floor reaches `2^-20` to meter a moonlit surface
rather than crush it, and `PlanetParams::ocean_roughness` gives the ocean mask a tight
GGX lobe, so a body low over water draws a glitter path instead of a round highlight.

The pipeline is organised around **three coordinate spaces**: *solar* (heliocentric
ecliptic J2000, double metres — where every body lives), *planet* (body-fixed per body,
origin at its centre, turning with its pole and spin — where surface entities live), and
*local* (the camera-relative scene frame the renderer draws in, origin at the observer's
surface point, +Y along the geodetic normal). The transforms between them are fully
body-parametric, so the sky is built the same way on any planet rather than only Earth:
`SkyObserver::observer_body` names the body the observer stands on, `fill_environment_sky`
anchors the scene origin to *that* body's surface and places every other body relative to
it, and `astro/body_orientation.hpp` supplies the per-body spin (`body_rotation_angle`,
the IAU W(t)) and the rotation of a direction into the observer body's equatorial frame
(`ecliptic_to_body_equatorial` / `equatorial_to_body_equatorial`). Earth is routed through
the exact fixed-obliquity conversion and sidereal time so the home sky is unchanged; every
other body gets its true pole and day. The editor re-anchors `observer_body` to whichever
body the camera is on and rebases the camera on a change, so time animation and precision
hold on every planet, not just Earth.

Three modules give that model gravity and a planet-relative transform, all bodies
handled by the same parametric code (Mercury–Pluto and the Moon, no per-body branches):

- **`gravity.hpp`** — the gravitational field. Bodies stay on their analytic Keplerian
  rails (the *sources*); a free entity is a `StateVector` integrated through the summed
  Newtonian field `gravity_field()` by a symplectic velocity-Verlet `integrate_step()`.
  Keeping the planets on rails makes the field a deterministic function of position and
  time — the property SushiLoop's lockstep needs — and stops long orbits drifting the
  way a fully dynamic N-body system would. `standard_gravitational_parameter()` tabulates
  GM per body; `sphere_of_influence_radius()` sizes the Laplace SOI. The inverse-square
  field is evaluated **only in double** at the seam: `|r|³` over ~1e11 m collapses in
  the physics solve's optional single precision.
- **`reference_frame.hpp`** — the active reference frame. A `ReferenceFrame` is
  body-centred but keeps inertial (ecliptic) axes, so a state expressed in it is the
  heliocentric state minus the body's own — a Galilean shift with no fictitious terms.
  `active_frame_body()` picks the most local dominant attractor at a point; `rebase()`
  is the single double-precision coordinate change a sphere-of-influence crossing
  triggers, the orbital analogue of the floating-origin sector rebase (§6).
- **`surface_frame.hpp`** — the body-fixed surface frame that makes a planet-relative
  pose work. An entity near a body stores its position as body-fixed Cartesian metres
  (ECEF, `geodetic_to_body_fixed` / `body_fixed_to_geodetic`, with lat/lon only as a
  boundary conversion for authoring and the map) and its orientation relative to the
  local East-North-Up `local_tangent_basis()` at that position. Because the tangent
  basis is *derived from position*, "upright, facing north" is identity orientation
  everywhere on the body — the reason a southern-hemisphere entity stands straight
  rather than tilted. `surface_gravity_vector()` is the near-field pull: the inward
  ellipsoid normal times `surface_gravity()`, correctly oriented over the whole body.
- **`gravity_field.hpp`** — `IGravityField`, the field behind a dependency-inversion seam,
  and `SummedRailsGravityField`, the default on-rails summation. The orbital integrator
  names the interface, so a patched-conic or full N-body field can replace it without the
  integrator changing.
- **`astro_dynamics.hpp`** — `advance_astro_state()` joins one field-parameterised
  `integrate_step` and the SOI `rebase` into the single per-step authority update: lift the
  body-centred state to heliocentric, step it through the injected field, re-select the
  active frame from where it lands, express it there.
- **`scene_frame.hpp`** — `SceneFrame`, the exact rigid bijection between a
  heliocentric-ecliptic position and the scene's local frame, reproducing the ephemeris's
  scene construction so a free body and the planet it orbits line up. `topocentric.hpp`
  holds the observer's East-Up-South basis (shared with the ephemeris, DRY);
  `body_orientation.hpp`'s `body_equatorial_to_ecliptic` is its inverse rotation.

`RuntimeSimulation` consumes this two ways. **Per-body gravity:** each physics step builds a
`Simulation::GravitySampler` (`make_gravity_sampler`) and hands it to `IPhysicsSimulation::step`,
which samples it at every body's own position each sub-step (`PhysicsWorld::predict_substep_field`).
The sampler maps a body's scene position to heliocentric, samples the injected
`Astro::IGravityField` — the *same* `SummedRailsGravityField` the orbital integrator uses, so
gravity has one source — and rotates the acceleration back into scene axes, so each body feels
the true field (1/r² falloff, curvature toward the attractor, third-body terms) rather than one
vector shared by the whole scene. Sampling at the current position keeps the semi-implicit predict
symplectic. With no dominant body it falls back to a uniform demo-gravity sampler. There is no
separate "astro body" mode: a body's orbital motion emerges from this same per-body gravity plus
its velocity, through the one physics path (the exclusive astro toggle and its parallel
`derive_astro_transforms` pose-derivation were removed; the `Astro::` dynamics modules they used
remain and now back the gravity sampler, and — ahead — the unified dynamic body's Free authority).
The simulation still owns the **master epoch**: `julian_date()` advances by the fixed step (scaled by
`set_time_scale_days_per_second`), the editor drives the sky from it through
`set_sky_observer`, and the extracted snapshot carries it back — one clock for orbits,
planets, and the rendered sky.

Two planet-relative constraints run in `RuntimeSimulation::apply_surface_constraints`
(from `extract`, so they hold both while playing and after an edit), gated on a dominant
body so plain non-astronomical scenes are untouched:

- **Surface anchoring** — an entity toggled through `set_surface_anchored` stores its
  orientation *ground-local* (relative to the East-North-Up tangent frame at its
  position); the pass composes the tangent frame onto it, so "upright" is identity
  everywhere on the body and a southern-hemisphere entity stands straight rather than
  tilted. "Up" is the **geodetic normal** (`surface_normal_scene`, the ellipsoid gradient),
  not the geocentric radial, so a flattened body's local vertical is exact — matching
  `Astro::geodetic_normal` in `surface_frame.hpp`. This is host-side `Record` bookkeeping
  (like colliders and cloth), not an ECS component — no Schedule system reads it, and it
  needs the `Environment` the systems do not see.
- **The planet collider** — every entity is kept outside the reference ellipsoid (its true
  flattened radius along the outward direction); a penetrating rigid body is re-posed
  through the physics seam so the surface is a hard floor. The editor's Scene fly-camera is
  *not* clamped: it flies freely and may enter a body (Unity/Blender behaviour), the
  infinite-far reverse-Z projection carrying the depth range.

**Frame-local authoring.** An entity carries a **reference frame** (`Simulation::EntityFrame`
= a celestial body index + `FrameMode` Auto/Free/Surface) — the Unity-parent analogue with a
body as the parent. It is an *authoring-boundary projection*, not a second source of truth: the
scene-frame `Transform` stays what physics and render read, and `frame_local_transform` /
`set_frame_local_transform` convert between the frame-local pose and the scene `Transform` through
the scene-frame bijection at the master epoch. In **Surface** mode the frame-local position is a
**geodetic** coordinate (latitude, longitude, altitude — `scene_to_body_fixed` /
`body_fixed_to_scene` unwind the body's spin `W(t)`, `surface_frame.hpp` does the ellipsoid
conversion), so a spawn is placed the way a map reads; in **Free** it is a Cartesian offset from
the body's scene centre (`reference_center_scene`). This is what lets the whole
solar system be placed through one Transform without typing heliocentric numbers, and it is the
reference descriptor — frame-independent — that a future networking layer syncs for zero-conflict
(never the per-client scene `Transform`). A reference body of -1 is the scene root, so an entity
that never picks a body is unchanged. Surface mode drives the surface anchoring above.

## 6. The value-type seam

The engine takes its scalar, vector, matrix, and quaternion types — and the
operations on them — from `core/types.hpp` and nowhere else. Those types belong to
**SushiBLAS** (tensors, and the floats derived from them). Until that library exists,
`core/types.hpp` aliases a minimal placeholder in `core/blas_placeholder.hpp`, which
now carries `Vector3`, `Mat4`, `Quaternion` and the handful of operations the renderer and
camera need (`perspective`, `look_at`, `compose_transform`, `mul`, …). When SushiBLAS lands, re-point `core/types.hpp` at it
and delete the placeholder — a single-file change, because nothing else in the
engine names the underlying type.

This is the same discipline as §1: one seam, not parallel paths.

The same seam also carries the planet-scale floating-origin types: `WorldVector3` is a
double 3-vector for absolute ECEF positions (a distinct type from `Vector3` to mark
absolute-vs-local intent), `SectorCoord` is an integer index of a fixed-size
cube ("sector") in that world space, and `FloatingOriginVector3` pairs a `SectorCoord` with a
`Scalar`-precision local offset from that sector's corner. `to_floating_origin`/
`from_floating_origin` convert between `WorldVector3` and `FloatingOriginVector3` given a sector
size. Keeping the local offset small (at most one sector wide) keeps a fragment's
distance from the camera representable when it narrows to 32-bit at the GPU boundary,
regardless of how far the sector is from the world origin. These types are the SushiLoop
M0 foundation (`docs/slop/SUSHILOOP.md`) and are not yet consumed by any simulation code.

The boundary `Scalar` is **always double** — there is no build switch. The engine's
purpose is to simulate planet- and solar-scale worlds, where single precision quantises
camera and transform math to roughly a metre at 10 000 km out (float32 carries ~7
significant digits), making it unusable at the seam; double is the engine's one and only
`Scalar`. The placeholder's `Float` is a plain `using Float = double`
(`core/blas_placeholder.hpp`), the sole reader of the choice. The vector and quaternion
types are, however, element-parametric (`Vector3T<T>`/`QuaternionT<T>`), and the physics
layer templates on that element (`RigidBodyT<T>`, `XpbdDistanceConstraintT<T>`), so the
**simulation's physics-solve precision is a separate runtime choice**
(`Simulation::Precision`): both a float and a double solver are compiled into `sushi_sim`
and `create_simulation(Precision)` picks one behind `Simulation::IPhysicsSimulation`,
letting the editor switch physics precision live (rebuilding the world from a scene
snapshot) without a rebuild of the binary. `sushi_render` shares the value types (across
`MeshInstance`/`CameraView`) without linking the engine target — it links the runtime
otherwise. The Vulkan upload path narrows to 32-bit explicitly at the push-constant
boundary, camera-relative, so absolute double positions never reach the GPU as a raw
cast.

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

Beyond the ECS/physics core, three areas are now covered directly. The
**`astro/` solar-system model** — pure double-precision maths with no device code —
is verified against astronomical reference landmarks (J2000 = JD 2451545.0, Earth
~1 AU from the Sun, WGS84 semi-major/minor radii, ~9.8 m/s² surface gravity, Earth's
SOI ~0.9 Gm) and against structural invariants that hold regardless of the exact
arithmetic: Kepler's equation is satisfied by the returned anomaly; the
ecliptic↔equatorial, topocentric, body-equatorial, and scene-frame transforms all
round-trip to the identity; every basis is right-handed orthonormal; the symplectic
integrator conserves orbital energy over a full revolution; and `advance_astro_state`
keeps a low orbit bound and deterministic (`Unit_JulianDate`, `Unit_OrbitalElements`,
`Unit_CelestialBodies`, `Unit_Gravity`, `Unit_Topocentric`, `Unit_BodyOrientation`,
`Unit_SurfaceFrame`, `Unit_StarCatalog`, `Unit_SceneFrame`, `Unit_ReferenceFrame`,
`Integration_AstroDynamics`, `Integration_Ephemeris`). The **camera/transform maths**
in `core/types.hpp` (quaternion rotation vs. its matrix form, the reverse-Z
infinite-far projection, `look_at`, `compose_transform`) is pinned in
`Unit_MathPrimitives`. The **determinism guard rails** — the seeded xorshift128+ RNG
and the fixed-timestep accumulator — are pinned in `Unit_Rng` (identical seeds replay
identically; a snapshot replays the future exactly, as rollback needs) and
`Unit_FixedTimestep` (step count depends only on total elapsed time, never on how it
was chunked across frames).

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

`loop/net.hpp` (namespace `Loop::Net`) is SushiLoop's network layer, scoped
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
key invariant with a toy `Scalar` command: a client that mispredicts several ticks
and later reconciles against the server's authoritative commands converges to
exactly what an uninterrupted server-only simulation would have produced.
`Unit_NetworkId` (`tests/functional/unit/test_network_id.cpp`) covers
`make_network_id`'s collision behaviour directly. Both are kept as narrow, isolated
proofs of `net.hpp`'s mechanics.

**`examples/net_demo.cpp` and `Integration_NetClientServer`**
(`tests/functional/integration/test_net_client_server.cpp`) wire the same machinery
into a live client/server harness driven by a real gameplay command instead of the
toy one. `PlayerCommand` — two movement axes, applied to a player entity's
`Position` — is the `Command` SushiLoop's real command stream now uses; it is
deliberately game-side (defined at the point of use, not in `loop/`), matching
`input.hpp`'s stance that the command type is left to the game. "Client" and
"server" are modelled as two logical roles in one process, each owning its own
`ecs::World`, which is the honest shape of a loopback-only milestone — there is no
second process or thread to model them as. The harness proves the full chain live:
per-tick prediction into `InputHistory`, batched `LoopbackChannel::server_process`
acks, `net::reconcile` rolling back and replaying on misprediction, and convergence
to an uninterrupted authoritative-only baseline world — then, in a second phase
kept strictly outside the ticks captured by `RollbackBuffer`, `make_network_id`
proves its agreement against an actual independent spawn on both the client's and
the server's `World`, with no matching round trip.

That second phase's placement is deliberate, not incidental: `RollbackBuffer`
still cannot survive a spawn or destroy inside a tick range it might roll back and
replay (§8's hard constraint), and this milestone does not attempt to lift that —
the demo and test sidestep it by never spawning within the reconciled window,
rather than solving rebasing across a structural change. Real transport (sockets),
and rebasing `RollbackBuffer` across a network-driven structural change, remain
later work; so does wiring any of this into the editor's Play mode, which still
runs `RuntimeSimulation` directly with no client/server split.

## 9. SushiLoop core

`docs/slop/SUSHILOOP.md` is the design note; this section is the pointer from
architecture to it. `loop/` holds the first, purely host-side layer of SushiLoop —
plain C++, no runtime or SYCL involvement — that the fixed-tick sim/net/snapshot
work (M1 onward) builds on:

- **`FixedTimestepClock`** (`loop/fixed_timestep.hpp`) turns real elapsed time into a
  whole number of fixed simulation steps plus a leftover interpolation fraction. It
  never reads the wall clock itself — the host accumulates real delta time into it —
  which is what keeps the *number* of ticks a run performs independent of timing
  jitter, a determinism precondition. `RuntimeSimulation` (§4.1) now owns one of
  these and is its first consumer: the editor's main loop measures real frame time
  and feeds it into `ISimulation::tick(real_delta_seconds)`, keeping the one
  wall-clock read outside `sim/` entirely.
- **`RngState`** (`loop/rng.hpp`) is a trivially copyable xorshift128+ generator,
  storable as an ECS component so seeded randomness travels with the world through
  snapshots and rollback instead of living in a hidden global.
- **`InputHistory<Command>`** (`loop/input.hpp`) is the per-tick, numbered command
  buffer shape that networked input capture and rollback replay (M3/M4) will read
  and write; the command type itself is left to the game.
- **`Loop::App<Command>`** (`loop/app.hpp`) is the authoring API that ties the above
  together into the settled surface a game is written against. It owns the
  `SushiRuntime`, `World`, and `Schedule`, and drives one fixed-step deterministic
  loop: each `step_once()` captures the tick's command into `InputHistory` (and a
  `RollbackBuffer` snapshot when enabled), applies it via the game's `on_command`,
  runs the `Schedule`, and applies the `CommandBuffer` barrier. Systems are declared
  ergonomically with `app.system<Read<A>, Write<B>>("name").each(fn)`, a thin wrapper
  over `Schedule::each` (via `SystemBuilder`). The loop is **always
  multiplayer-ready**: the command stream is numbered every tick regardless of
  network state, and the network is reached only through
  `Loop::Net::INetworkTransport<Command>` (§8.1), so `connect()`-ing a transport
  turns a single-player game networked with no change to its systems. This is the
  point at which the SushiLoop core layers are wired into `Schedule`/`World` — the
  standalone-game host, distinct from `sim/`'s editor-facing `ISimulation` (§4.1).

`SE_DETERMINISTIC_FP` (`cmake/ProjectOptions.cmake`, default `ON`)
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
- **SushiLoop M3 — Snapshot/rollback core (done).** `Loop::RollbackBuffer` (§8),
  per-tick per-chunk byte snapshots with restore, proven against the milestone's
  key invariant (rollback-and-replay bit-identical to an uninterrupted run).
  Scoped to no structural change across a capture/restore pair and whole-chunk
  (not per-write-dirty) capture; both are follow-on work, not this milestone's.
  M4 (network, reconciliation) and M3's own dirty-tracking refinement build on
  this without changing its capture/restore contract.
- **SushiLoop M4 — Network layer (done).** `Loop::Net` (§8.1): a loopback-only,
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
- **Rendering (in progress).** A greenfield Vulkan 1.4 renderer behind an RHI
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

## 11. UI (retained ECS canvas)

The UI is retained and lives in the ECS, the same choice the rest of the engine
makes: a `Canvas` is an entity, and every button, panel, and label is an entity under
it carrying `ui/components.hpp` components — `RectTransform` (UGUI anchor/pivot/offset
layout), `UIImage`, `UIText`, `UIButton`, linked by `UIParent`. So UI appears in the
hierarchy, serializes with the scene, and (in a networked game) snapshots like any
other component.

`resolve_rect` (`ui/layout.hpp`) is the entire anchor model as a pure function of a
parent rectangle and a `RectTransform`, so it is unit-tested (`Unit_UILayout`) with no
world. The `UI` façade (`ui/ui.hpp`) is the authoring surface: `canvas`/`panel`/
`image`/`label`/`button` builders spawn the entities into an existing `World` and keep
a light ordered index (the same host-record pattern §4.1 uses), giving a deterministic
paint and hit-test order. Each frame `update(screen_size, pointer)` resolves every
`ComputedRect` (parents before children), runs the button state machine and
press-and-release-inside click detection off an explicit per-frame `PointerInput`
(input as a value, not a global — the same determinism discipline the sim follows),
tints each button's graphic, and fires `on_click` callbacks. `build_draw_list()` emits
a renderer-agnostic `UIDrawList` of coloured rects and text runs, in paint order — the
seam a Vulkan 2D overlay pass (not yet written) consumes. `examples/ui_demo.cpp` and
`Integration_UI` drive a canvas + button headlessly and assert layout, clicks, and
button states. Rasterising the draw list and authoring UI in the editor's panels are
the remaining visual work.

## 12. Skeletal animation (assets, animator, blend trees, layers, IK, retargeting — phases A0–A9)

The skeletal-animation stack (full plan in `docs/design/animation_system.md`) is a
Unity-Mecanim-parity character system split across three domains — an asset domain that
cooks skeletons, clips, and controllers; a deterministic simulation domain that advances
animator state at fixed tick; and a per-frame evaluation domain that samples, blends, and
skins. Phase A0 is the foundation the other domains stand on: the math and the skeleton
asset, with no evaluator, ECS columns, or renderer coupling yet.

**The math seam additions (`core/types.hpp`, §6).** Sampling, blending, and IK are all
interpolation and matrix algebra the seam did not previously expose. A0 adds, behind the
same one file, `lerp` (vector and scalar), `nlerp`/`slerp` (both neighbourhood-corrected:
they flip a rotation to the near hemisphere so a blend takes the short arc, and `slerp`
degrades to `nlerp` when the inputs are nearly parallel), `quaternion_from_matrix`,
`affine_inverse` (a true inverse through the adjugate, so it is correct under non-uniform
scale — this is what builds inverse-bind matrices, where the rotation-only transpose short
cut would be wrong), and `decompose_transform` (the TRS decompose that inverts
`compose_transform`). The interpolators are element-parametric (`Vector3T<T>`/
`QuaternionT<T>`) so the evaluator can run float while the boundary stays double; the
matrix ops stay at boundary precision because they run at import, not per frame.

**The skeleton asset (`animation/`).** A skeleton is a flat, immutable, relocatable blob,
never a pointer-linked tree. `SkeletonView` (`skeleton.hpp`) is a non-owning
structure-of-arrays view: a topologically sorted parent array with the invariant
`parent[i] < i` (so composing model space is a forward scan, never a pointer chase — this
is what retires the engine's old host-side, non-topological hierarchy), bind-pose local
TRS, object-space inverse-bind matrices (`JointMatrix`, 16 floats, GLSL column-major — the
palette layout, distinct from the double `Mat4` because object-space joint data never needs
the range that forces double), FNV-1a 64 joint name hashes (`hash.hpp`) for mask / IK /
attachment lookup, and a bone-LOD table. The `.sushiskel` format (`skeleton_blob.hpp`) is
versioned, little-endian, and position-independent (byte offsets, no pointers): the cook
`build_skeleton_blob` sorts joints by depth so the invariant holds, remaps every parent
reference, derives inverse-bind matrices from the bind pose when the source did not supply
them (glTF does), and lays the SoA sections out at aligned offsets; the load
`load_skeleton_blob` validates the header and returns a `SkeletonView` that aliases the
bytes with zero copy or parse.

**The glTF import lane (`animation/gltf_skeleton_import.hpp`).** The mesh importer bakes
each primitive into its node's world transform and drops the node graph, skins, and
inverse-bind matrices — the exact data a rig is. `import_gltf_skeleton` is the parallel
lane that keeps them: it reads one skin, turns each joint node's local transform into
bind-pose TRS, copies the inverse-bind matrices through (or lets the cook derive them), and
produces a `.sushiskel` blob. The declaration lives on the engine's animation surface; the
implementation lives in the renderer's cgltf lane — the one place cgltf is linked — and adds
no new dependency, since animation is header-only over `core/types.hpp`.

**The database seam (`animation_database.hpp`).** `IAnimationDatabase` is the
dependency-inversion boundary the simulation and evaluator will sit behind — they request a
`SkeletonView` by `AssetId` and never see the importer, the file format, or the byte
buffers. `AnimationDatabase` is the in-memory owner: each blob lives in its own heap buffer
so the buffers do not move as the database grows and the views it hands out stay valid for
its lifetime. Clip and controller views extend this same interface in later phases; the id
space is shared across all three asset kinds.

**The editor overlay (`editor/animation/skeleton_debug_draw.*`).** `SkeletonPreview` imports a
rigged glTF and caches its bind-pose model-space joint positions; `draw_skeleton_overlay`
projects those through the Scene panel's camera (the gizmo's exact view-projection) and paints
bones, joint octahedra, and names over the rendered image. The viewport panel takes it as one
optional, defaulted parameter, so no other call site changes. A "Skeleton Preview" window in
the editor drives it — the surface behind A0's "load a rigged glTF, see its rest pose". The
readable names come from the blob's debug name table (`SkeletonView::joint_name`), added for
this and the coming joint inspector; the runtime path still addresses joints by hash.

`examples/skeleton_demo.cpp` validates the whole data foundation headlessly (TRS round-trips,
the affine inverse, slerp/nlerp behaviour, an out-of-order chain that cooks and loads to
satisfy `parent[i] < i`, name-hash lookup, name-string round-trip, and an identity skin matrix
at the bind pose), and `import_gltf_skeleton` is exercised by `examples/assets/rigged_chain.gltf`.
With this, **A0 is complete**; A1 adds single-clip playback and GPU skinning (a skinned
character looping an animation with zero TAA ghosting).

**The evaluation + animator stack (phases A1–A5, CPU cores shipped).** Above the A0 assets sits
the two-halves split of design §5. The *deterministic tick* (`animator_step.hpp`) is the
interpreter over the compiled `.sushictrl` controller (`animator_controller.hpp`): per layer it
advances normalized time, runs the state machine (Any-State then current, crossfades, triggers,
events), and accumulates root motion — all over trivially-copyable ECS columns
(`animator_components.hpp`), so it is byte-snapshottable and rollback-exact (A3). The *derived
frame pose* (`animator_evaluator.hpp`) is recomputed each frame and never snapshotted: it
resolves each layer's active state to weighted clip contributions, samples and weight-blends
them, folds the layers, composes model space, and builds the object-space palette. A state's
motion is a single clip or a **blend tree** (`blend_tree.hpp`, A4) — five node kinds (1D, 2D
simple/freeform-directional, freeform-cartesian, direct) over a flat node/child array with a
gradient-band pair table baked at compile; the controller blob is v2 to carry it. Layers fold
gated by an **avatar mask** (`avatar_mask.hpp`, A5) — a name-hash-keyed `.sushimask` resolved to
the rig — either override (nlerp) or **additive** (FMA of a delta baked at import by
`additive.hpp`); a layer's weight can be driven by a parameter through `animator_step`. Clip
sampling and compression (`clip*.hpp`, A1/A2) and the batched crowd evaluator with bone-LOD and
update-rate throttling (`batch_evaluator.hpp`, A2) sit behind the same views. After the layers fold, an ordered **pose-modifier stack** (`pose_modifier.hpp`, A6) runs in
model space between compose and palette — the `IPoseModifier` seam, Unity's pass ordering, so IK
corrects the final blended pose. The shipped solvers are analytic pole-controlled two-bone
(`ik_two_bone.hpp`), weight-distributed cone-clamped look-at (`ik_look_at.hpp`), iteration-capped
FABRIK (`ik_chain.hpp`), and composite foot placement (`ik_foot_placement.hpp`) that rays to the
ground through the `IPoseTaskContext` seam. Beyond posing joints, a clip carries **morph-weight and generic float tracks** (`.sushianim` v2,
A7): `morph.hpp` maps a clip's morph tracks onto a mesh's target order and gives the CPU
reference of the skin-pass morph blend, and `generic_track.hpp` routes generic tracks to an
`IFloatSink` binding registry (material/UI/script hooks). **Retargeting** (A8) lets one clip
library drive many rigs: `humanoid.hpp` maps a skeleton's joints to canonical `HumanBone`s (by an
alias heuristic or an explicit table), and `retarget.hpp` transfers a clip's bind-pose deltas onto
another rig (`retarget_clip`) or mirrors them left-to-right (`mirror_clip`). **Authoring** (A9)
persists a controller as JSON (`animator_controller_json.hpp`, the only animation header that
pulls in nlohmann/json — the editor's save/load and undo/redo, round-tripping to a byte-identical
blob) and previews states off the loop (`edit_preview.hpp`'s `scrub_to_state`). Under the *dense* runtime clip sits the *sparse* **keyframe authoring model** (`keyframe.hpp`):
`ScalarCurve` / `QuaternionCurve` (the dope-sheet / curve-editor primitives, constant/linear/
cubic), a `ClipAuthoring` bundle that `bake`s to the dense `ClipDesc`, and a `PoseRecorder` that
captures a live pose into keys (the "record" workflow). The editor's **Animation window**
(`editor/animation/animation_panel.*`) is a GUI over it, Unity's Animation-window shape: it targets
the **Hierarchy-selected entity**, and with Record armed keys its transform as it is moved, or with
Record off **drives the object live in the Scene view** from its keys as the timeline is scrubbed —
Bake writes a dense `.sushianim`. The **Animator window** (`editor/animation/animator_graph_panel.*`)
is the Mecanim state-machine graph editor over a `ControllerDesc` — a grid canvas of draggable
state nodes, transition arrows made by right-click ▸ Make Transition To (a `"Exit"` target compiles
to the layer's entry), Entry/Exit/Any-State nodes, a parameter panel, and JSON save/load through
`animator_controller_json.hpp`. Verified headless
by `clip_demo` (A1), `animation_benchmark` (A2), `animator_demo` (A3), `blend_tree_demo` (A4),
`layered_animation_demo` (A5), `ik_demo` (A6), `morph_demo` (A7), `retarget_demo` (A8),
`authoring_demo` (A9), and `keyframe_demo`. What remains is GPU/editor-side: the A1 GPU skinning pipeline
(`render/passes/skinning_pass.*`, `render/scene/skinning_system.*`, `skinning.comp`, written and
awaiting a GPU build) and the A9 editor windows (Animator graph, dope sheet, live scrubbing UI).

## 13. Input (device-abstracted actions, Phase 1)

Design: `docs/design/input_manager.md`. Gameplay and editor code bind to named actions
(`"Move"`, `"Jump"`), never to a device control — a control is what a *binding* names, in
data, once. Keyboard/mouse, gamepad, and touch all reduce to three action shapes: `Button`,
`Axis1D`, `Axis2D`. No consumer branches on device type.

**The header-only action layer (`include/SushiEngine/input/`)** carries zero SDL, zero
SYCL, and no runtime link — the same discipline `sushi_render`'s abstract side keeps. Its
pieces are seven single-responsibility objects (SRP by construction):

- `controls.hpp` — the engine-owned control enums. `Key` is numbered by USB HID keyboard
  usage IDs (the table SDL scancodes are built on), so the numbering is physical-position
  based and layout-independent, and the SDL translator *reinterprets* a scancode as a `Key`
  rather than looking it up. Every ordinal here is a serialized wire value and must never
  renumber. A `ControlPath` (family + ordinal) is the atom a binding stores.
- `events.hpp` — the single trivially-copyable `InputEvent` record every source emits, the
  `EventType` tags, the stable `DeviceId` slot scheme (keyboard 0, mouse 1, gamepads from 2),
  and a float `Vector2` (distinct from `UI::Vector2`, which is double).
- `source.hpp` — `IInputSource` (one method: drain this frame's events) and the header-only
  `ScriptedInputSource`. Because all state is folded from events *above* the source, a
  scripted, SDL, and (later) virtual source are indistinguishable downstream — Liskov holds
  by construction, which is what lets the whole layer run headless in tests with no mocks.
- `device_registry.hpp` — `DeviceRegistry`, the single owner of "what is held now", folded
  from the event stream. Level state (keys, buttons, stick/trigger values) persists; relative
  state (mouse delta, wheel) accumulates within a frame and resets at its start — the split
  the tick-boundary sampler (Phase 2) needs.
- `bindings.hpp` — bindings as data: `Binding`, `Deadzone` (radial for whole sticks, axial
  for triggers), `ChordGate` (Ctrl+Z), and the composite/`Vector2` bindings. Every evaluator
  is a pure function of the registry, run through a fixed processor order (deadzone → invert →
  scale, then composite assembly and diagonal normalization), so a keyboard composite and an
  analog stick feed one action through identical downstream math. A new binding shape is a new
  struct and evaluator, no consumer change — OCP as data.
- `action_map.hpp` — `Action`/`ActionType`, `InputContext` with fluent builders
  (`add_axis2d("Move").bind_composite(W,S,A,D).bind(GamepadAxis::LeftStick, ...)`), and
  `ActionMapper`. The mapper holds a priority-ordered context stack and resolves every action
  once per host frame into an `ActionSnapshot`; a higher context *consumes* the controls its
  actions reference, so pushing a `"Menu"` context masks gameplay movement with no consumer
  testing a flag. An `InputGate` mirrors ImGui's `WantCapture*` flags to suppress key- or
  mouse-sourced actions in one place.
- `input_manager.hpp` — the `InputManager` façade. `begin_frame()` drains every registered
  source, folds the events into the registry, and resolves the mapper. It never touches the
  `World` and never registers a system.

**The compiled backend (`input/`, `sushi_input` STATIC).** The mirror of `sushi_render`'s
recipe: links `SDL2::SDL2` only, no SYCL, no runtime, C++17. It holds the one SDL-aware input
component, `Input::SdlInputTranslator`, which turns already-pumped `SDL_Event` records into
engine `InputEvent`s. It does **not** pump SDL — the single `SDL_PollEvent` loop stays in
`SdlWindow`, and the translator registers on the window's event-handler seam alongside ImGui.
That seam grew from one handler to a handler list: `IPlatformWindow::add_event_handler`
appends, so ImGui still sees events first. The native `SDL_Event` crosses the translator's
header as `const void*`, so SDL leaks into no consumer translation unit — the editor's
"only SDL-aware components" set grows from two (`SdlWindow`, `ImGuiBackend`) to three.

**Gamepad and haptics.** A controller is a device family, not a special case: the same
`"Move"`/`"Jump"` bindings drive keyboard or pad. `SdlWindow` inits `SDL_INIT_GAMECONTROLLER`
(core SDL2) and the translator opens/closes controllers on hot-plug, translating their button
and axis events to the same `InputEvent` shapes a stick binding already reads (SDL's button and
axis ordinals match the engine enums, so the translation is a reinterpretation). A controller's
identity to the engine is its `DeviceId` slot, allocated by `GamepadSlotTable` — a header-only,
SDL-free, unit-tested policy that hands a pad the lowest free slot and keeps it across an
unplug/replug of the same ordering, so bindings and player assignments survive a reconnect. The
translator also implements `IHapticsSink`, so the object gameplay drives rumble through is the
same one it never sees as SDL — `SDL_GameControllerRumble` behind
`rumble(device, low, high, duration)`.

**Rebinding and persistence.** Because a binding is data, changing one changes no consumer
code — the property `rebinding.hpp` makes operational. `RebindingListener` captures the next
control of an expected shape (a button rebind is deaf to axis noise; an axis rebind demands
deflection past a threshold so a drifting stick cannot bind itself) and cancels on Escape or
timeout; `binding_conflict` warns when a captured control is already used in the context, and
`set_button_binding` writes it back. Persistence is `bindings_json.hpp`
(`bindings_to_json`/`bindings_from_json`), tolerant field-by-field with defaulted reads exactly
like the editor's `render_settings`: a missing action entry keeps its compiled-in defaults, a
malformed entry is ignored rather than throwing, and unknown actions survive a round-trip. It
is the only input header that pulls in nlohmann/json, so the core action layer and any headless
build that never persists stay dependency-free. The editor holds the serialized document in
`Preferences::input_bindings` (as text, so the struct stays JSON-free) and nests it in
`preferences.json`; the game owns where its own file lives — the engine provides the functions,
not a path policy.

**Touch and virtual controls.** Touch decomposes into two layers, and only the first is device
code. Pointers: the translator turns `SDL_FINGERDOWN/MOTION/UP` into touch events (normalized →
pixels, stable finger→slot), and the `DeviceRegistry` folds up to `MAX_TOUCH_POINTS` of them; a
`set_mouse_as_pointer` flag folds the mouse into pointer 0 so a touch UI is developable on
desktop, and `primary_pointer()` is the one pointer the engine UI reads (via the opt-in
`ui_pointer` adapter) instead of the host hand-feeding `UI::PointerInput`. Virtual controls:
`VirtualControlSource` owns a screen-space layout of sticks and buttons and, each frame, claims
the pointers inside them and emits ordinary gamepad-shaped events — a virtual stick is
`GamepadAxis::LeftStick` on a dedicated slot, so a `"Move"` binding resolves it through the exact
path a hardware stick takes. It runs on the manager's second-pass virtual-source stage
(`add_virtual_source`), polled after the primary fold so it reads this frame's pointer state and
its output is folded back for the mapper. Adding touch to a shipped gamepad game is placing
controls, not writing input code; rendering them is the engine UI's job, and gesture recognition
is a recorded follow-on.

**Editor migration.** The editor's shortcuts and tool keys, once scattered `ImGui::IsKeyPressed`
polls, are now two rebindable contexts (`editor/input/editor_contexts.hpp`): `EditorGlobal`
(Undo/Redo/Save/Copy/Cut/Paste, chorded on Control) and `EditorViewport` (gizmo W/E/R). `main.cpp`
pushes them, applies any overrides saved in `Preferences::input_bindings`, and consumes
`input.snapshot().pressed(...)` for the global shortcuts; the toolbar reads the same snapshot for
the gizmo keys (via a non-owning `ActionSnapshot*` on `EditorContext`). The `!WantTextInput` guards
each poll wrote by hand are now the mapper's single capture gate. The Preferences window gained a
rebind page — list an action, click Rebind, press a key — that runs the `RebindingListener`,
rewrites the binding, and serializes the set so a rebind survives a restart. Camera flight (WASD
while right-mouse is held) stays on the viewport's own `Editor::InputState` seam, unchanged.

**Local-multiplayer routing.** Binding resolution asks a `DeviceAssignment` — held per
`ActionMapper` — which device answers each control family, instead of hard-coding the first
connected pad. The default (keyboard, mouse, first pad) reproduces single-player exactly, so the
routing costs nothing until used. A local-multiplayer game gives each player a `PlayerHandle` (its
own mapper, tick accumulator, and assignment) resolving the *shared* contexts against its own
devices, collected in a `PlayerRoster` that routes claims and answers "press A to join"
(`join_candidates`). Device events are folded once into the one shared registry; N players is N
reductions of that state, one per `Command` — the exact shape SushiLoop's "a player is an ECS
entity, input a per-tick command" already prescribes.

**Completion pass.** Four recorded follow-ons close the system out. *Buffering* (§2.2): the
`TickSampleAccumulator` stamps each tick with ticks-since-press per action, so
`TickSample::pressed_within(name, window)` gives jump-buffer and coyote-time windows at the tick
cadence. *Gestures* (`gestures.hpp`, §2.4): a `GestureRecognizer` turns the registry's pointers
into tap/long-press/drag/pinch results, time-driven by `update(dt)` and left for the consumer to
map onto actions — a pure sensor. *Replay* (`replay.hpp`, §6): an `InputRecorder` captures the
frame event stream and replays it through a `ScriptedInputSource`, reproducing *mapper* behaviour
the way `InputHistory` reproduces the *sim* — the two straddle the tick boundary, the design's
central line; `replay_json.hpp` is an opt-in file format. *Text* (`text_input.hpp`, §6): a
`TextInputChannel` is an active-gated UTF-8 buffer the translator feeds `SDL_TEXTINPUT` into, with
actions suppressed for its duration by the same capture gate — text is a channel, not an action.
The editor exposes all of it through an **Edit > Input Manager** window: contexts and their actions
with the current binding, click-to-rebind with conflict flagging, and Reset to Defaults, persisted
to preferences.

**The tick boundary (`tick_sample.hpp`).** Two consumers read the input at two cadences. The
editor and any immediate-mode UI read the per-frame `ActionSnapshot`. The simulation reads a
per-tick `TickSample`, and only ever as a reduced, quantized value — because the fixed-step
loop (`App::advance`) runs zero, one, or several ticks per host frame, per-frame state cannot
cross directly (a tap in a zero-tick frame would vanish; one press in a two-tick hitch would
fire twice). `TickSampleAccumulator` resolves this: `accumulate` folds each frame's snapshot,
`consume` (called once per tick from inside `sample_command`) hands out one sample and then
clears the edges and zeroes the relative axes it returned. Three laws hold by construction —
edges stay sticky until a tick consumes them, the first tick of a burst consumes the edge
while later ticks see level only, and relative axes (mouse deltas) sum since the last tick
while absolute axes (sticks) are latest-wins. A sub-frame tap survives because the
`DeviceRegistry` folds press/release *edges* independently of the final level, so the mapper's
button edges do not depend on a frame-boundary level change. `quantize_axis` maps a normalized
value to a symmetric `std::int16_t` *before* it enters the game's `Command`, so `InputHistory`,
rollback replay, and server reconciliation all operate on bit-identical values — prediction
misses from float jitter cannot exist. The manager stays on the OS-event thread forever, and
`consume_tick_sample()` returns a value snapshot, which is already the thread-crossing currency
the future sim-thread split prescribes.

**Layering.** The one-way `SushiEngine → SushiRuntime` arrow is untouched — input never sees
the runtime. Headless examples and tests use `ScriptedInputSource` and link neither SDL nor
`sushi_input`; only a binary that opens a window needs the compiled library. `SE_BUILD_INPUT`
gates it (forced on by `SE_BUILD_EDITOR`). All seven roadmap phases and the design's recorded
follow-ons (buffering, gestures, replay, text) have landed; the editor consumes the action
snapshot for its shortcuts and tool keys and configures bindings through its Input Manager window.
The one deliberately-untouched seam is the viewport's camera-flight `Editor::InputState` fill,
kept on ImGui because its per-mode latching (right-mouse-held owns WASD) is viewport state, not a
binding — a good seam the design keeps as-is.

## 14. Audio (from-scratch AAA game-audio, Phase S0)

Design: `docs/design/audio_system.md`. A from-scratch, middleware-free game-audio engine,
placed like skeletal animation and cosmetic VFX — a **wall-clock snapshot consumer that
lives outside the deterministic sim island**, so a run is byte-identical with audio on or
off. Its shaping decision is the split every shipping AAA runtime is built on: a **control
plane** (the game/ECS thread, which allocates freely and only publishes intent) and an
**audio-render plane** (a high-priority callback thread that mixes under hard real-time
discipline — no heap allocation, no locks, no syscalls). The real-time mix runs on the
device callback and deliberately **sidesteps SushiRuntime**, which is a
block-until-quiescent throughput engine with no real-time thread class; the runtime enters
only through an optional, deferred GPU batch-DSP seam.

Phase S0 stands up the render-plane boundary end to end with a trivial renderer, before any
DSP exists — the same "seams first, then fill" discipline the render and input stacks used.

**The header-only seams (`include/SushiEngine/audio/`)** carry zero SDL, zero SYCL, and no
runtime link:

- `device.hpp` — the device I/O seam. `AudioStreamFormat` is the negotiated stream shape
  (sample rate, channel count, power-of-two block size); a backend may return a format
  other than the one requested, so a consumer always reads the obtained format back.
  `IAudioRenderer` is the real-time render sink: `render(float* const* channels, int
  channel_count, int frame_count) noexcept`, called once per block on the audio thread with
  **planar** (deinterleaved) buffers — the discipline the DSP core keeps throughout,
  interleaving only at the device boundary. `IAudioDevice` (open/close/is_running/format)
  isolates the one unstable, platform-specific dependency so the whole mix is testable
  against a trivial renderer and the backend is swappable without touching a line of DSP.
- `accelerator.hpp` — `IDspAccelerator`, the optional GPU batch-DSP seam, declared now and
  implemented later (S10). It carries a single `available()` query so a subsystem that could
  offload long convolution / HRTF / ambisonic decode asks first and falls back to its CPU
  path — which is every build today. Kept intentionally thin because the runtime's fluent
  API is unstable; the batch-submit surface lands with the real implementation.

**The compiled backend (`audio/`, `sushi_audio` STATIC).** The mirror of `sushi_input`: a
plain STATIC library linking SDL2 and nothing else — no SYCL, no runtime — so it builds on a
stock toolchain and never touches the one-way `SushiEngine → SushiRuntime` arrow. It carries
the sole SDL-aware audio component, `Audio::SdlAudioDevice`, which opens an
`SDL_AudioDevice` and drives an `IAudioRenderer` once per block on SDL's callback thread:
the planar scratch and channel-pointer table are allocated once in `open` (never in the
callback), the renderer fills them, and the device interleaves the result into SDL's output
buffer. The OS handle crosses its header as an opaque `std::uint32_t`, so SDL never leaks
into a consumer translation unit.

**The `App::runtime()` seam.** `Loop::App` now exposes its owned or borrowed runtime — the
one handle the later GPU accelerator path needs to allocate USM. It does not weaken the
one-way dependency: the App still owns the lifetime, and a borrowed runtime is returned,
never destroyed. Gameplay never needs it; the loop still hides the runtime behind `world()`,
`commands()`, and `system()`.

**Layering & build.** `SE_BUILD_AUDIO` gates the compiled backend (OFF by default). The
`audio_demo` example is the S0 vertical slice: a headless, self-checking software block loop
(pump N blocks through a silence renderer, assert every sample is silent and the renderer
ran), then a best-effort real device open that is a clean no-op on a headless host. `se
audio` builds and runs it (configuring `SE_BUILD_AUDIO=ON`, exactly as `se render` does for
the renderer probe).

**The DSP core (`include/SushiEngine/audio/dsp/`, Phase S1).** The portable, real-time-safe
C++17 layer beneath the seams — no SDL, no SYCL, no runtime — that every later mix, filter,
and spatializer is built on. It has no build option: like the input action layer it rides
the SushiEngine INTERFACE target and is exercised headlessly by `Unit_Audio` tests.

- **Real-time primitives.** `ScopedNoDenormals` (`denormals.hpp`) is the RAII guard that
  sets flush-to-zero + denormals-are-zero on the FPU for the span of a callback — the fix
  for the slow subnormal arithmetic a decaying IIR or reverb tail would otherwise fall into;
  it restores the exact prior control word, and being outside the deterministic island the
  bit-level change is harmless. `SpscRing<T>` (`spsc_ring.hpp`) is the one queue of the
  two-plane model: a power-of-two, wait-free single-producer/single-consumer ring with its
  two indices on separate cache lines and acquire/release publication — the control thread
  pushes command records, the audio thread drains them, neither ever locks.
- **SIMD kernels (`simd.hpp`).** The per-block hot loops — `apply_gain`, `apply_gain_ramp`
  (the zipper-free level change), `mix_accumulate`, `copy_scaled`, `fill`, and constant-power
  `equal_power_pan` — as a 4-wide SSE path with a scalar remainder tail on x86 and a scalar
  fallback elsewhere. Applied to *summed* bus buffers, so they cost O(bus), not O(voice).
- **Filters (`dsp/filters/`).** `OnePole` (the cheapest smoother / DC block / parameter
  ramp), `Biquad` (the RBJ cookbook set — low/high-pass, band-pass, notch, peaking, shelves
  — in the numerically well-behaved Transposed Direct Form II), and `StateVariableFilter`
  (Andrew Simper's Cytomic TPT-SVF, the modulation-stable default that yields every mode from
  one pair of integrator states). Coefficients are designed in `double` off-thread and stored
  `float`, the audio-path rule even though the engine's `Scalar` is double.
- **Block graph (`graph.hpp`, `nodes.hpp`).** `BlockGraph` is a DAG of `Node` processors
  linearized by a Kahn topological sort computed off the audio thread; `process()` pulls one
  fixed block through that order and never allocates. **Feedback** is a connection flag, not a
  special node: a feedback edge is excluded from the ordering and, because each output buffer
  persists between blocks, the consumer reads the producer's previous-block output — a
  one-block z⁻¹, which is exactly what lets a comb or reverb loop be computed in a single
  forward pass. Built-in nodes are `SineNode`, `GainNode` (per-block ramp), `MixNode`, and
  `BiquadNode`. The `audio_dsp_demo` slice wires sine → mix → low-pass → gain and self-checks
  that the settled RMS proves the high tone was filtered out; `Unit_Audio`
  (`tests/functional/unit/test_audio_dsp.cpp`) pins the ring, denormal flush, every filter,
  the SIMD kernels, the pan law, the topological order, and the one-block feedback exactly.

**The action layer (`include/SushiEngine/audio/`, Phase S2).** The header-only game glue
over the DSP core — a prioritized multi-source mix — reached through the `audio/audio.hpp`
umbrella (now in `SushiEngine.hpp`). Like the DSP core it has no build option and is
exercised headlessly by `Unit_Audio`.

- **Parameters (`parameter.hpp`).** Every runtime mix change crosses the thread boundary the
  same way: `SmoothedValue` holds an atomic **target** the control thread publishes and the
  audio thread **slews toward** at a configured rate, bracketing each block as `[start, end]`
  for a click-free gain ramp — never a raw jump. `Rtpc` layers the "game variable → authored
  curve → target" mapping on top, evaluating the clamped piecewise-linear `RtpcCurve` on the
  control thread so the audio thread only ever ramps.
- **Voices (`voice.hpp`).** A `VoiceSource` renders mono and also offers a cheap `advance`
  (skip forward without producing output) — the path a *virtualized* voice takes to keep its
  play position current for ~free. `VoiceDescriptor` carries gain, priority, bus, pan, and a
  linear distance attenuation that reaches true silence, which is what makes it a clean
  culling signal. `ToneSource` and `BufferSource` are the S2 sources; decode/streaming is S8.
- **Mixer (`mixer.hpp`).** Voices sum into stereo **buses**, not into each other, so an
  effect runs once on a summed bus buffer (O(bus)) instead of per voice. A `Bus` has a series
  **insert chain** (`IBusEffect`), a post-fader `SmoothedValue` gain, an **output** route into
  a parent, and **aux sends** that copy its signal at a level into a parallel bus (the
  reverb-send pattern). `MixerGraph` orders buses by a topological sort over the routing and
  send edges so every contributor precedes its consumer, with the master rendered last. Buses
  are stereo now; the ambisonic scene bus of §4 replaces that path at S4 without changing the
  routing/insert/send structure.
- **Voice manager + engine (`voice_manager.hpp`, `engine.hpp`).** `VoiceManager` holds a
  fixed pool and, each block, computes an effective **audibility** (base gain × attenuation)
  per active voice, ranks the set by `(priority, audibility)`, promotes the top **real**
  voices up to a cap to full rendering, and leaves the rest **virtual** (position bookkeeping
  only) — so hundreds of possible sounds collapse to a bounded render set that pans into a
  handful of bus buffers. Real voices ramp their gain and free themselves when a one-shot
  ends. `AudioEngine` is the `IAudioRenderer` the device drives: it sets the denormal guard,
  clears the mixer, folds the voice manager's real voices into the buses, runs the bus graph,
  and fans the stereo master out to the device channels. `audio_mixer_demo` starts 26 voices
  against a cap of 6 and self-checks the cap and the out-of-earshot virtualization; the
  `Unit_Audio` suite (`test_audio_mixer.cpp`) pins the smoother, RTPC, aux-send arithmetic,
  post-fader gain, bus order, the real/virtual cap, distance culling, pan centring, and
  one-shot lifetime. Occlusion, the HDR window, a separate decode cap, and voice stealing
  layer onto this ranking in later phases.

**Propagation (`propagation.hpp` + `dsp/fractional_delay.hpp`, `dsp/air_absorption.hpp`,
Phase S3).** A source's whole travel through the air is modelled as **one variable
fractional delay line of length distance/c**, and the Doppler falls out for free: the read
pointer sits that far behind the write pointer, so when the distance changes between blocks
the read rate stops being one — and a read rate other than one *is* a pitch shift. This is
why there is no velocity term anywhere; motion is implicit in the frame-to-frame distance
change. `FractionalDelayLine` does the read with a 4-point cubic-Lagrange (Farrow) kernel
(non-recursive, no ringing, clean to modulate). `SourcePropagation` wraps it: it slew-limits
the delay so a source cannot appear to break the sound barrier (`|v_radial| < 0.9·c`),
**snaps** the delay on a teleport instead of sweeping it (a sweep would fire a synthetic
Doppler screech), then dulls the block with a distance-driven air-absorption low-pass and
scales it by the distance gain. The air model is the full ISO 9613-1 absorption
(`air_absorption_db_per_meter` from temperature/humidity/pressure) reduced to a one-pole
corner that falls with distance (`air_absorption_cutoff`); the speed of sound feeds both the
delay and the absorption from the same temperature, so delay, Doppler, and dullness stay
consistent. `DistanceModel` (Linear/Inverse/Exponent) and the shared `distance_attenuation`
give the rolloff, used by *both* the audibility ranking and the rendered gain so a voice is
culled by the level it is played at. The voice manager runs a `SourcePropagation` per spatial
real voice; `VoiceManager::set_voice_position` is how a moving emitter (or, at S6, the ECS
snapshot) drives the whole effect. `audio_propagation_demo` flies a tone past the listener
(pitch up approaching, down receding) and `test_audio_propagation.cpp` pins the delay-line
accuracy, the ISO absorption, the distance laws, the delay ≈ distance/c, the Doppler
direction, and the teleport snap.

`AudioEngine::render` clamps its internal work to the prepared maximum block and zero-fills
any surplus device samples, so an OS mixer that hands back a larger callback block than
`prepare` was told degrades to brief silence rather than overrunning a buffer.

**The spatializer (`spatializer.hpp` + `dsp/spherical_harmonics.hpp`, Phase S4).** The 3D
rendering core, and the reason the audio path scales: a source is placed by **encoding** it
into a shared **ambisonic scene bus** with its spherical-harmonic gains — `(order+1)²` gains,
cheap, per source — so any number of sources collapse into one fixed field. The harmonics are
real AmbiX (**ACN** ordering, **SN3D** normalisation, so W = 1 and the first-order channels
are the direction's y/z/x), evaluated from the associated-Legendre recurrence. The field is
**decoded once** to a fixed 26-point virtual-speaker layout and each speaker is rendered to
the two ears through an analytic head model — a Woodworth interaural time difference (a
fractional delay per ear) and a head-shadow low-pass on the far ear — so the number of ear
renders is constant no matter how many sources play. Head tracking is free: the voice manager
encodes each source in **head-relative** coordinates (`head_relative_direction` rotates a
world direction into the listener's `forward`/`up` frame), so a head turn re-aims every source
with no extra state. `AudioEngine` owns the spatializer, decodes it to binaural each block,
and sums that with the non-spatial stereo master; a spatial voice encodes into the scene bus
instead of stereo-panning when a spatializer is present, and the decode is skipped on a block
with no spatial source. `audio_spatial_demo` orbits a tone around the head (audibly circling
on headphones); `test_audio_spatial.cpp` pins the SH convention and orthogonality, the
encode/decode kernel, the left/right level cues, the ITD, front symmetry, and head-tracking.

The analytic HRTF is self-contained (no measured data) and gives solid horizontal
localisation and externalisation; a measured-HRTF / SOFA + MagLS decode is a fidelity upgrade
that slots in behind the same encode → decode seam. The roadmap continues S5 (FDN reverb +
I3DL2 + per-zone aux) through S10; S0–S4 (the critical path) are complete.

## 15. VFX particle system (authoring model, dual backends, GPU render — phase VFX1)

Design: `docs/design/vfx_particle_system.md`. One authored effect asset feeds **two** simulation
backends behind one seam — a GPU-cosmetic path (millions of particles, render-side, *outside* the
deterministic sim island, like skinning/audio) and a CPU-deterministic path (bounded, byte-
reproducible, rollback-safe, like `AnimatorInstance`). Both consume the same compiled POD.

**Authoring model (`include/SushiEngine/vfx/`, header-only, C++17, depends only on `core/types.hpp`).**
An emitter is a stack of modules across four stages (spawn / shape / init / update) plus a render
module; each module is its own trivially-copyable descriptor struct (the Open/Closed seam — a new
behaviour adds a descriptor, a compiler handler, and a shader/integrator branch, touching no
existing module). `AnimationCurve` and `ColorGradient` are keyframed authoring types bakeable to
fixed-width LUTs. `EmitterCompiler` flattens a `ParticleEffect` (a list of `EmitterDescriptor`)
into a `CompiledEffect`: an array of POD `CompiledEmitter` records plus two baked LUT atlases — the
single artifact both backends and the GPU consume, the particle equivalent of resolving authored
`RenderSettings` into a POD `QualityParams`. `EffectDatabase` is the AssetId registry (lazy
compilation), mirroring `AnimationDatabase`. `GpuParticle` is the shared 80-byte, five-`vec4`
std430 record used by the CPU backend, the GPU pools, and the shaders.

**Deterministic backend (`vfx/deterministic_backend.hpp` + `Simulation::ParticleEmitter`).** A
fixed-pool integrator run as an ECS system inside the fixed step. Its whole per-emitter state is a
`DeterministicEmitterState` — a capped `GpuParticle` pool, a count, a `Pcg32`, and a few scalars,
pointer-free — so a tick is byte-snapshottable and a rolled-back-then-replayed tick reproduces it
exactly (`Integration_ParticleDeterminism`). Shapes, forces (gravity/drag/curl-noise turbulence),
and the size/colour-over-life LUTs all run here too, sampling the same baked LUTs the GPU does.

**GPU render path.** `Render::Scene::ParticleSystem` owns the shared, persistent, device-local
particle pool (zero-cleared once), the per-slot host-visible emitter table (`GpuEmitter`, a std430
mirror of the compute-visible `CompiledEmitter` subset with the emitter's world transform and ring
cursor baked in), and the uploaded LUT atlases. `ParticleSimPass` (a compute `IRenderPass`, after
`skinning_pass_`) sweeps the pool: a simulate dispatch advances/ages/retires and appends survivors
to a compacted draw list, then a per-emitter emit dispatch allocates ring slots and initialises new
particles — both atomically build a `VkDrawIndirectCommand`. `ParticlePass` (a graphics
`IRenderPass`, between `ssr_pass_` and `taa_pass_`, writing `scene_final`) draws vertex-lessly: six
vertices per alive particle, expanded into a camera-facing billboard pulled from the draw list,
sampling the scene depth (never attaching it) to discard fragments behind geometry, additively
blended. The compacted draw list and indirect args are graph transients (the graph derives the
compute→draw barriers); the pool is system-owned. `ISceneView::render` gained a
`ParticleEmitterView` extract channel (opaque `CompiledEmitter` bytes + LUT pointers + host-computed
spawn count), the same shape as `SkinnedInstance`. Additive-only for the slice; alpha depth-sorting,
lit particles, ribbons, mesh particles, and GPU collision are phases VFX2–VFX5.

**Enabling seams.** `Resources::GraphicsPipelineDesc` gained a `ColorBlend` member folded into the
fragment-output pipeline-library key (defaults reproduce opaque, so transparent draws are now
expressible without disturbing existing passes). `QualityParams` gained `gpu_particles` /
`max_particles` / `particle_sim_substeps`, scaled per tier (Low drops cosmetic particles). Build:
the four shaders (`particle_emit.comp`, `particle_simulate.comp`, `particle.vert`, `particle.frag`,
sharing `particle_common.glsl`) go through `sushi_compile_shader` + the shader catalogue; the two
systems and two passes into `sushi_render`.

### 15.1 Deterministic emitter entities + alpha particles (Bağla + VFX2a)

**Deterministic emitter entities.** A gameplay entity carries a particle emitter through the sim's
`IWorldEditor` (the emitter quartet `create_particle_emitter` / `has_particle_emitter` /
`particle_emitter_params` / `set_particle_emitter_params` / `set_has_particle_emitter` +
`particle_effect_count`/`particle_effect_name`). Like cloth, this is host bookkeeping — no ECS
migration: `RuntimeSimulation::Record` gains the fixed `Vfx::DeterministicEmitterState pool` (~80 KB,
off the ECS chunk), plus the effect handle and play head. `step_particle_emitters()` runs inside
`step_once()` (after the schedule, before extract), advancing every playing pool one fixed step via
`CpuDeterministicBackend::step`; `extract()` emits one `RenderScene::particle_billboard` per live
particle. A built-in effect library (Fire/Sparks/Smoke, Deterministic domain) lives on the sim's
`EffectDatabase`. The renderer draws these through a new `ParticleBillboard` extract channel on
`ISceneView::render` — already-simulated world-space particles billboarded directly (the particle
analogue of `ClothStrandView`), uploaded to a host-visible `GpuParticle` buffer by
`ParticleSystem::prepare_billboards` and drawn by a `vkCmdDraw` in `ParticlePass`. The editor adds
"GameObject ▸ Particle Emitter", an Add-Component entry, an Inspector section (effect/seed/playing),
and `.sushiscene` persistence.

**Alpha particles (VFX2a).** `GpuEmitter` now carries the emitter's blend and sort modes. During the
compute compaction, `particle_simulate.comp`/`particle_emit.comp` bucket each particle by blend —
additive/premultiplied into `particle_draw`, true-alpha into `particle_alpha` — each with its own
`VkDrawIndirectCommand` in a single `particle_args` buffer (additive at offset 0, alpha at 16).
`ParticlePass` draws the two buckets with two pipelines: the additive glow (`src+dst`, order-
independent) and a premultiplied "over" for true alpha (`src + dst*(1-a)`), both on the premultiplied
fragment output. Smoke and dust composite correctly instead of only glowing. Deferred to the next
VFX2 slice: clustered-punctual-light particles (the froxel version — needs camera-relative conversion).

**Alpha depth-sort (VFX2a).** The alpha bucket is bitonic-sorted back-to-front on the GPU. A new
`ParticleSortPass` (compute, between the sim and draw passes) runs `particle_sort.comp`: mode 0 seeds
one `{-distance², index}` key per pool slot from the alpha list's camera distance (padding slots sink to
the end); mode 1 is one bitonic compare-exchange stage, dispatched `log2(N)*(log2(N)+1)/2` times by the
host over the power-of-two pool capacity — the engine's "dispatch a host-known max, read the alive count
on the GPU" idiom, so no indirect dispatch is needed (the tree's first GPU sort). The seed stage always
runs so the key buffer has a producer; the bitonic stages run only when `ParticleSystem::has_alpha()`.
The alpha draw uses `particle_sorted.vert`, which indexes the alpha list through the sorted keys
(binding 2 on the draw pass's set), so `gl_InstanceIndex` walks particles far-to-near; the additive and
billboard draws keep the direct `particle.vert`. The pool capacity was set to 2^16 to keep the sort
tractable.

**Lit particles (VFX2b).** The true-alpha bucket (smoke, dust) is lit by the sun; the additive bucket
(fire, sparks) stays emissive. The sun is a **world-space** directional light (`Environment::sun`), so
lit particles need no camera-relative conversion — sidestepping the biggest hazard of clustered-light
particles. The billboard fragment shades the sprite as a camera-facing hemisphere (a spherical normal
from the sprite offset), takes `max(dot(n, sun_dir), 0)`, and adds a flat ambient; the sun direction,
radiance, and a per-draw lit flag ride the 128-byte push constant (which replaced the unused viewport
lanes). The clustered-punctual-light and IBL-SH version (camera-relative, set-0 b13-17) is the deferred
refinement.
