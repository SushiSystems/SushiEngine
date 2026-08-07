# Physics

This file covers the physics domain: the graph-coloured constraint solver, the XPBD
generalization built on it, cloth and soft bodies, colliders and contacts, and the editor
authoring surface that reaches all of them.

## 1. The physics constraint solver

Physics is a domain layer on top of the runtime, the same way the ECS is: it expresses itself as
ordinary read/write sets and lets the dependency tracker do the ordering. The solver here is
**Projected Gauss-Seidel** (PGS), the sequential constraint method, parallelised by **graph
colouring**.

A sequential Gauss-Seidel sweep cannot run all constraints at once: two constraints that share a
body would race. So the constraints are *edge-coloured* over the bodies (`color_constraints`) —
each colour is a batch in which no two constraints share a body. The `ConstraintSolver` then
emits one task per colour: a parallel projection over that colour's constraints, which is
race-free because the bodies are disjoint. Every colour reads and writes the shared position
array, so the runtime orders the colours into a sequential sweep — colour k+1 after colour k —
while parallelising fully *within* a colour. That ordering is exactly Gauss-Seidel across
colours, and because a colour's constraints are independent, the parallel result equals the
sequential one. The sweep is repeated for the iteration count, and the whole solve is one graph
compiled once and replayed every frame.

The solver owns no engine concept beyond bodies and constraints — it takes a position array, an
inverse-mass array, and a projection functor — so a new constraint type (contacts, angular
joints) is added by providing its POD and its device projection; the colouring and the graph
structure are reused unchanged. `DistanceConstraint` with `DistanceProjection` is the first
concrete type, exercised by `samples/physics/pgs_demo.cpp` (a hanging chain checked against a
scalar reference).

### 1.1. XPBD: the rigid-body generalization (SushiLoop M2)

`engine/domain/physics/include/SushiEngine/physics/core/rigid_body.hpp`,
`.../physics/constraints/xpbd_constraint.hpp`, and `.../physics/solver/xpbd_solver.hpp` add the
unified XPBD (position-based dynamics) solver `docs/design/SUSHILOOP.md` calls for: one
compliant-constraint framework meant to grow into rigid bodies, soft bodies, cloth, and rope,
rather than a family of special-cased solvers living side by side.

`RigidBody` extends the PGS solver's bare position + inverse mass with an orientation
(`Quaternion`) and a diagonal, body-local inverse inertia tensor, plus the predicted pre-solve
pose (`previous_position`/`previous_orientation`) XPBD's velocity update needs.
`predict()`/`update_velocity()` are the two halves of one XPBD sub-step: integrate external
forces into a predicted pose, solve constraints against that prediction, then recover velocity
and angular velocity from how far the solve moved it — never from an explicit force/torque
integration.

`XPBDDistanceConstraint` generalizes `DistanceConstraint` to two attachment points (offsets in
each body's own local frame, so an anchor at the origin recovers a plain rigid link) and adds
`compliance` — XPBD's defining feature over plain PBD: a constraint's stiffness is a physical
unit (inverse stiffness, really) instead of an artifact of iteration count or step size, so
`compliance == 0` is a fully rigid constraint and identical PGS behaviour (verified in
`tests/integration/test_xpbd_solver.cpp`: with zero inverse inertia and zero-offset anchors, no
angular coupling can occur, so the two solvers' linear terms are the same arithmetic).

`XPBDSolver<Constraint>` reuses `color_constraints`/`ColorBatches` unchanged — same
graph-colouring, same compile-once-replay-every-frame structure as `ConstraintSolver` — but the
shared resource is one `RigidBody` buffer instead of separate position/mass buffers, and each
constraint carries a per-step Lagrange multiplier (`lambda`) that `solve()` resets to zero before
every step, because the compliance term is only meaningful accumulated within a single step.
`samples/physics/xpbd_demo.cpp` ports `pgs_demo.cpp`'s hanging chain onto
`RigidBody`/`XPBDSolver`, checked against a byte-for-byte host mirror of the projection.

`engine/domain/physics/include/SushiEngine/physics/scene/physics_world.hpp`'s
`PhysicsWorld<Constraint>` is the layer above `XPBDSolver` that turns a one-shot solve into an
actual loop: register bodies and constraints, `finalize()` once (uploads the bodies, compiles the
graph — mirrors `XPBDSolver`'s own build-once-replay-every-frame split), then `step()` every
frame runs predict → solve → derive-velocity for each requested sub-step. It takes no dependency
on `engine/foundation/ecs/` on purpose, keeping the layering direction in
[the layer table](overview.md#2-layers) intact; the ECS-facing half (mapping entities to body
indices, syncing `Transform`/`Orientation` each frame) is an `engine/world/simulation/`-level
concern that builds on this seam rather than being folded into it.

**The editor's "Rigid Body" toggle** (`engine/world/simulation/source/runtime_simulation.cpp`) is
the first consumer of that seam, and takes a different route from the generic
`engine/world/simulation/include/SushiEngine/simulation/physics_bridge.hpp` below: Renderer/Camera
need an ECS component migration (`migrate_components`) because their data — colour, lens — lives
in a component only present when attached, but a Rigid Body's data (position, orientation) is
already `Transform`/`Orientation`, always present. So attaching/detaching physics is plain host
bookkeeping in `RuntimeSimulation::Record` (`has_physics_body`, `physics_parameters`).

The physics itself lives behind the `Simulation::IPhysicsScene` seam
(`engine/world/simulation/include/SushiEngine/simulation/physics_services.hpp`), not in
`RuntimeSimulation`: whenever the physics-driven entity set changes, `tick()` gathers one
`RigidBodyDescription` per Rigid Body entity and calls `set_rigid_bodies`, which *diffs* that set
against the single `Physics::RuntimeGraphBuilder` solver the seam owns
(`engine/domain/physics/include/SushiEngine/physics/solver/runtime_graph_builder.hpp`). An entity
that was here last frame keeps its body, its handle and its velocity; one that has gone is removed
with its constraints; one that is new is admitted at its descriptor pose, at rest. So toggling
physics on one entity never disturbs another already-falling body.

`RuntimeSimulation` only marshals poses across the seam; it owns no solver of its own. `tick()`
steps the scene under a gravity sampler and writes the solved pose back before the ECS schedule
runs. `.sushiscene`
(`engine/world/serialization/source/scene_serializer.cpp`) carries
`has_physics_body`/`physics_body` as an independent field pair (not mutually exclusive with
camera/renderer, unlike those two).

`RuntimeSimulation` owns a `Loop::FixedTimestepClock` (see
[SushiLoop core](world.md#2-sushiloop-core), and
`engine/world/loop/include/SushiEngine/loop/fixed_timestep.hpp`):
`ISimulation::tick()` takes the host's measured real elapsed time (`real_delta_seconds`),
accumulates it into the clock, and runs one full step — physics, then the ECS
schedule, then the render snapshot extract — once per whole fixed step the clock reports (zero on
a fast host frame, more than one after a hitch). The physics sub-step duration is derived from
the clock's fixed step (`fixed_dt() / PHYSICS_SUBSTEPS_PER_TICK`) rather than a second,
separately hardcoded constant, so there is one source of truth for tick duration. The editor's
main loop (`applications/editor/source/main.cpp`) is the one place that reads the wall clock,
measuring real frame time and passing it to `tick()`; the "Step" toolbar button instead calls
`tick(fixed_dt_seconds())` to force exactly one step regardless of elapsed time. The clock's
leftover interpolation fraction is computed and stored on `RuntimeSimulation` after each `tick()`
but has no consumer yet — render interpolation is a later milestone.

`engine/world/simulation/include/SushiEngine/simulation/physics_bridge.hpp` is that
simulation-level half. `Simulation::PhysicsBody` is an ordinary component naming which
`PhysicsWorld` body an entity owns (`INVALID` until registered — an entity can carry the
component before it has one); this keeps the mapping in the ECS itself rather than a side table.
`Simulation::initial_rigid_body()` reads an entity's current `Transform`/`Orientation` once, at
`PhysicsWorld::add_body()` time, to seed the body's starting pose.

`Simulation::sync_transforms_from_physics()` is the one direction wired up so far: every tick,
after `PhysicsWorld::step()`, it walks every archetype matching `{PhysicsBody, Transform,
Orientation}` (the same `World::query()` + per-chunk-column walk `Schedule::each` uses
internally, but as a plain host loop — there is no parallel work here worth a graph node) and
copies each registered body's solved position/orientation into the entity's
`Transform`/`Orientation`. It has no reverse (ECS -> physics) direction and no wiring into
`RuntimeSimulation`'s tick loop or the editor; it is a seam beside the live path rather than the
one the live path takes, and placing a physics-driven entity is `IRigidBodyService::set_rigid_pose`
on `IPhysicsScene`.

### 1.2. Cloth (SushiLoop M5)

`engine/domain/physics/include/SushiEngine/physics/soft/cloth.hpp`'s `build_cloth_grid` is the
confirmation of [XPBD's](#11-xpbd-the-rigid-body-generalization-sushiloop-m2) claim that XPBD is
one framework, not a family of special-cased solvers: cloth adds no new solver or constraint
type, only a topology. It registers `rows * cols` `RigidBody`s (zero inverse inertia, so anchors
implicitly at each body's own origin recover the same linear-only degeneration `xpbd_demo.cpp`'s
hanging chain already relies on) into the caller's `PhysicsWorld<XPBDDistanceConstraint>`, with
row 0 pinned (`inv_mass == 0`), and wires a structural `XPBDDistanceConstraint` to each right and
below neighbour plus a shear constraint to each diagonal neighbour pair — the shear links are
what keep the grid from collapsing into a parallelogram under load, since structural links alone
only resist stretching along the grid axes. `ClothGrid` exposes the registered body ids by
`(row, column)` so a caller (a demo, or later a gameplay layer) can address a specific grid point
without recomputing the row-major indexing itself.

`samples/physics/cloth_demo.cpp` mirrors `xpbd_demo.cpp`: a device solve through `PhysicsWorld`,
checked against a byte-for-byte host mirror of `XPBDDistanceProjection` run over the identical
topology. `Integration_Cloth` (`tests/integration/test_cloth.cpp`) proves the grid's shape (body/
constraint counts, which row is pinned) and that the pinned row never moves while the rest of the
grid falls and settles under gravity.

Volumetric (tetrahedral) soft bodies are **not** built here — cloth is a 2D constraint grid over
point masses, not a general deformable-solid solver, and a tetrahedral mesh is a different
topology entirely. They live in `engine/domain/physics/include/SushiEngine/physics/soft/` as the
finite-element model (`finite_element_model.hpp`, `fem_element.hpp`), not as an extension of this
file.

**The editor's "Cloth" toggle** (`engine/world/simulation/source/runtime_simulation.cpp`) wires
`build_cloth_grid` into the live tick loop, following the same route the
[Rigid Body toggle](#11-xpbd-the-rigid-body-generalization-sushiloop-m2) takes rather than
`physics_bridge.hpp`: a cloth grid is a single host-side record —
`RuntimeSimulation::Record::has_cloth`/`cloth_parameters` (`Simulation::ClothParameters`: rows,
columns, spacing, compliance) — not one ECS entity per grid point. Unlike a Rigid Body, whose
count is the only thing that forces a rebuild, *any* `ClothParameters` edit forces one
(`cloth_dirty_`), because rows/cols change the grid's body count and there is no meaningful
partial state to carry across a topology change the way a falling free body's position/velocity
survives an unrelated Rigid Body toggle.

`tick()` gathers one `ClothDescription` per Cloth entity and calls
`IClothService::set_cloth_grids`. A set whose grids all keep the shape they already had costs a
comparison and nothing else (`cloth_matches`); anything else is a wholesale replace, every grid
torn down and rebuilt from its current `Transform::position` as the grid origin, at rest. Inside
the seam a cloth particle is an ordinary body in the same solver as every rigid body, not a second
world of its own — which is what makes rigid-to-cloth contact an ordinary contact rather than a
coupling between two solvers. `IPhysicsStepper::step` advances all of it under one gravity sampler
and one substep schedule, floored by the fixed step `Loop::FixedTimestepClock`
[reports](#11-xpbd-the-rigid-body-generalization-sushiloop-m2) — there is no separately hardcoded
cloth tick rate.

Cloth's world-space particle positions are exposed read-only via
`IWorldEditor::cloth_particle_positions(id)` (row-major, matching `Physics::ClothGrid`), and also
flow into `RenderScene::deformable_instances` / `deformable_vertices` / `deformable_indices`
every `extract()`: a `Simulation::DeformableInstance` per live surface rather than through
`RenderScene::instances` — that type remains one entity, one fixed-mesh instance, and still
cannot express a multi-vertex deforming mesh.

That channel used to be `ClothInstance`, carrying a grid's rows and columns. It no longer does,
because rows and columns describe only a sheet: a tetrahedral soft body's surface is a closed
triangle mesh with no grid structure, and a fractured one is not even connected. So an instance
now carries a vertex range and a triangle range, and says nothing about how either was arranged;
`extract()` runs a grid through `Render::build_grid_indices` to produce the triangle list a sheet
used to imply. Each surface's indices are numbered relative to its own `first_vertex`, not into
the concatenated array, so a surface's topology is independent of what else happened to be
extracted this frame. `DeformableInstance` also carries the owning entity's `id` (for picking)
and a `color`.

The editor copies those ranges into `Render::DeformableMeshView`s each frame
(`applications/editor/source/main.cpp`) and hands them to `ISceneView::render`'s optional
deformable parameter. Shading is GPU-side
(`engine/presentation/render/shaders/deformable.comp`): one thread per vertex sums the face
normals of the triangles touching it, area-weighted by leaving each face normal un-normalised.
One thread per *vertex* rather than per triangle is what keeps the pass atomic-free —
accumulating into shared vertices from a per-triangle thread would need float atomics, and float
atomics make the result depend on scheduling order. The price is the inverse map, vertex to
triangles, which `Render::build_vertex_triangle_adjacency` builds on the host and
`DeformableBuffers` caches: it is a pure function of the index list, so it is rebuilt only when
the topology changes, keyed on the mesh's id, its `topology_revision`, and its two counts. A
cache miss costs a rebuild and nothing else — it can make a frame slower, never wrong.

Indices reach the GPU verbatim, mesh-local, in a host-visible buffer that doubles as the index
buffer the draw binds; the draw supplies `base_vertex` as its vertex offset. That replaced a
second compute dispatch whose only job was to rewrite every index into the shared numbering.
Surfaces draw through the same lit `mesh_pipeline_` Box/Sphere/Cylinder use (already
double-sided, since a cloth sheet is single-sided geometry that can flip), so a soft body shades
and picks like any other object instead of drawing as a bare wireframe; its outline pass is
extended the same way as the primitive shapes'. `.sushiscene` carries `has_cloth`/`cloth` as an
independent field pair, the same shape as `has_physics_body`/`physics_body`.

### 1.3. Primitive shapes, colliders, and Terrain

Three concerns separate cleanly: what an entity *looks like*, what it *collides as*, and what
drives its *motion*. `Simulation::ShapeParameters`/`ColliderParameters`
(`engine/world/simulation/include/SushiEngine/simulation/simulation.hpp`) both start from a
`{PrimitiveKind kind; Vector3 parameters;}` pair, editor-facing and, like `ClothParameters`,
plain host-side bookkeeping on `RuntimeSimulation::Record`
(`has_shape`/`shape_parameters`, `has_collider`/`collider_parameters`) rather than ECS components
— neither is read or written by any `Schedule` system, so there is nothing to gain from an
archetype migration. `PrimitiveKind` (`Box`, `Sphere`, `Cylinder`, `Plane`) is declared in
`engine/world/simulation/include/SushiEngine/simulation/components.hpp` even though it backs no
component, since it is the vocabulary both authoring structs share. `ShapeParameters` alone also
carries a `mesh`/`mesh_path` pair naming an imported glTF the Renderer draws instead of the
primitive named by `kind` (`docs/design/static_mesh_authoring.md`); `ColliderParameters` stays
exactly the kind/parameters pair, since a collider has no imported-geometry path.

`IWorldEditor::create_box`/`create_sphere`/`create_cylinder` each spawn a Renderer entity with a
`Shape` and a `Collider` defaulted to the same kind/params — a created Box is collidable out of
the box, and either can be edited or removed independently afterward. `create_terrain` spawns a
large, thin flat `Box` Shape (the visual) paired with a `Plane` `Collider`, and — critically —
**no Rigid Body/`PhysicsBody`**: nothing integrates Terrain's pose, which is what makes it immune
to gravity, while `extract_static_planes`
(`engine/world/simulation/include/SushiEngine/simulation/physics_extract.hpp`) turns its `Collider`
into the standing plane every body rests on. `Collider` is what the contact pass and the scene
queries read: the extract copies it onto each `RigidBodyDescription`, `PhysicsSimulation` keys it
by body slot, and `shape_for_slot` turns it into the `CollisionShape` the broadphase bounds, the
narrowphase generates a manifold from, and a raycast tests against.

`Simulation::RenderInstance` gained `shape_kind`/`shape_parameters`, mirrored on
`Render::MeshInstance` as `kind`/`shape_parameters` — the render-side enumerator is
`Render::MeshKind`, which keeps the render seam free of any dependency on `Simulation`, and the
editor's per-frame copy loop maps one to the other. `extract()` gates drawing on `has_shape` **and**
`has_renderer` together, because the mesh (Shape) is now a feature of the Renderer rather than an
independent component: the Inspector edits the mesh kind and dimensions inside the Renderer
header, adding a Renderer attaches a default Box mesh, and removing the Renderer takes the mesh
with it (`applications/editor/source/scene/inspector_panel.cpp`). `create()` makes a truly empty
entity — a plain `Transform`/`Orientation` with no Renderer and no mesh — so a bare "Create
Entity" draws nothing, matching Unity's empty GameObject; the mesh kind is also now editable
(Box↔Sphere↔Cylinder, or Imported for a glTF loaded from a path — see
`docs/design/static_mesh_authoring.md`) rather than fixed at creation.

`engine/presentation/render/source/geometry/mesh_registry.hpp` is the device's mesh store: a unit
cube, a unit sphere and a unit cylinder, plus every imported mesh. The draw passes group instances
by `MeshKind` to bind each mesh's buffers once per group, and `Geometry::shape_scale` turns an
instance's `shape_parameters` into the local scale multiplied into its model matrix, so a default
`{0.5,0.5,0.5}` Box draws as a unit cube.

Entity creation ("Create Empty Entity", Camera, and the Box/Sphere/Cylinder/Terrain `Objects`
submenu) lives in one place, `draw_create_object_menu_items` in
`applications/editor/source/scene/scene_commands.cpp`, called by the Entity menu and every
Hierarchy context menu (row, filtered-search row, empty space) so they can never drift apart.
Copy/Cut/Paste follow the same pattern via `draw_clipboard_menu_items`: Copy snapshots the
selection through `IWorldEditor`'s getters into `EditorContext::ClipboardEntity` entries
(transform, colour, visibility, and every optional component's attached-flag/params), Paste
replays them through the matching setters onto newly `create`d entities, and Cut is Copy
immediately followed by `destroy` on the originals — no new engine-side clone primitive, just
existing `IWorldEditor` surface replayed.

### 1.4. Collision and soft bodies

Two additions extend the XPBD physics without touching the graph-coloured solver.
`engine/domain/physics/include/SushiEngine/physics/collision/narrowphase.hpp` is the narrowphase:
element-parametric collider shapes (`SphereCollider<T>`, `PlaneCollider<T>`, `BoxCollider<T>`,
and the oriented `OrientedBox<T>`) and pure functions that return a `Contact` (unit normal from
the first shape to the second, positive penetration depth, contact point) for each shape pair —
including a full 15-axis SAT `collide_obb_obb` for oriented-box vs. oriented-box. They are
geometry only — no runtime, ECS, or solver dependency — so they are unit-tested directly
(`Unit_Collision`).

`engine/domain/physics/include/SushiEngine/physics/collision/contact_solver.hpp` consumes them for
`PhysicsWorld`: non-penetration is an inequality constraint that only pushes bodies apart, so
rather than living in the compile-once `XPBDSolver` (whose constraint set is fixed) it is a
positional projection pass regenerated from the narrowphase each sub-step, run between `predict`
and `update_velocity`. Because `update_velocity` derives velocity from the post-projection
position, a body that lands on a surface loses its downward velocity with no explicit restitution
term. That path is the one `samples/physics/soft_body_demo.cpp` and `Unit_Collision` drive; the
live scene takes a different one, below.

`engine/domain/physics/include/SushiEngine/physics/soft/soft_body.hpp` is the 3D counterpart of
the [cloth grid](#12-cloth-sushiloop-m5): `build_soft_body_lattice` wires an `nx*ny*nz` particle
grid held by structural (axis) and shear (face-diagonal) `XPBDDistanceConstraint`s into a
`PhysicsWorld`, so the same solver runs a deformable block with no new constraint type — a
mass-spring soft body. Both are validated headlessly (`Integration_SoftBody`,
`samples/physics/soft_body_demo.cpp`). It is not what the live scene runs either:
`ISoftBodyService::set_soft_bodies` instances a cooked tetrahedral asset as a
`Physics::SoftBodyInstance` over the finite-element model in `physics/soft/fem_element.hpp`.

Contacts are wired into the **live tick**. `PhysicsWorld::step` takes an optional post-solve
callback — run each sub-step between the constraint solve and the velocity derivation — so that
world stays collider-agnostic while a caller injects a narrowphase.
`Simulation::PhysicsSimulation` does it in one solver instead, and needs no injection seam,
because every kind of body *is* a body to it: a cloth particle, a vehicle's shell node and a crate
are the same thing to the solve. Each tick it keeps a `Physics::BVHBroadphase` over one proxy per
body — a rigid body as the oriented box or sphere its `Collider` names, a cloth particle, shell
node or wheel as the sphere its radius names, plus the scene's static `Plane` colliders (Terrain,
supplied every tick via `set_static_planes`) — generates a manifold per candidate pair, and
submits each one to the solver as a contact constraint.

Coupling is **two-way** with no special case: a rigid body and a cloth particle that overlap are
each pushed apart by their own generalized inverse mass, so cloth drapes over a rigid *and* pushes
back on it. Cloth-cloth pairs and the pairs inside one vehicle are excluded, by each layer's
self-excluding `CollisionFilter` rather than by a test in the manifold pass.

So a body dropped on terrain comes to rest, and a cloth sheet settles over, and visibly displaces,
a rigid sphere it overlaps. Friction and restitution come from the two bodies' `PhysicsMaterial`s,
combined per pair by `make_contact_parameters`; a manifold carries up to four points, so a resting
box does not rock. The remaining gap is cloth self-collision: soft bodies have it
(`physics/soft/soft_self_collision.hpp`), cloth does not, and its layer excludes itself for that
reason. Rendering a deforming surface mesh (cloth and soft bodies reach the renderer as vertex
sets) is complete — cloth draws shaded and pickable through the
[mesh pipeline](#12-cloth-sushiloop-m5), not as a wireframe.

### 1.5. Editor authoring: cloth, UI, and custom components

Every capability [XPBD](#11-xpbd-the-rigid-body-generalization-sushiloop-m2) through
[collision and soft bodies](#14-collision-and-soft-bodies) added is now authorable in the editor,
all through the same plain-C++ `IWorldEditor` seam and all as attach/detach components.

**Cloth as an object.** `create_cloth` (Entity ▸ Objects ▸ Cloth) makes a bare entity owning a
cloth grid. So a fresh cloth is visible without pressing Play, `extract()` synthesises a flat
resting sheet from `ClothParameters` (matching `build_cloth_grid`'s
`origin + (col, 0, row) * spacing` layout) whenever the physics grid has not been built yet; once
the world is played the simulated particle positions take over. The wireframe already reached the
renderer as deformable surfaces (see [the render seam](presentation-render.md#1-the-render-seam)),
so no render change was needed.

**UI (Canvas + elements).** UI is a host-side record on the entity — `UIElementKind`
(Canvas/Panel/Image/Text/Button) plus a `UIElementParameters` that is a uGUI RectTransform
(anchors, pivot, anchored position, size, colour, opacity, text) — the same no-ECS-migration
bookkeeping as cloth, since nothing in the `Schedule` reads it.
`create_canvas`/`create_ui_element` add them from Entity ▸ UI (elements parent to the selected UI
entity so they lay out inside it). The editor draws the tree as a 2D overlay: each frame
`main.cpp` flattens every UI entity into `UIOverlayElement`s (params + the index of the UI
parent) and both viewports paint them with ImGui's draw list (`paint_ui_overlay`), resolving each
rect against the panel rect via a top-left, y-down variant of the uGUI formula and tinting
buttons on hover/press. This is a deliberate shortcut over a dedicated Vulkan 2D pass — it makes
canvases and buttons visible and editable now; the engine-side `SushiEngine::UI` module
([`domain-ui.md`](domain-ui.md)) remains the runtime path.

The overlay is also a **RectTransform manipulator** in the Scene view: it is drawn translucent
with outlines (a full-screen canvas therefore no longer hides the 3D scene, and a canvas is never
picked by its body — clicks fall through to the scene or a child), clicking an element selects
it, dragging its body moves it, and dragging a corner handle resizes it. Each drag inverts the
layout formula (`ui_apply_screen_rect`) to write the new screen rect back as
`position`/`size_delta`, and is one undo step (begin/end mirroring the transform gizmo). The Game
view draws the same overlay solid and non-interactive. This is why `ViewportPanel::draw` takes a
mutable `UIOverlay` (elements plus edit-mode and pick/edit outputs) rather than a const element
array.

**Custom (script) components.** The engine has no scripting VM, so a "custom component" is
authoring data: `ScriptComponent` (a `type_name` and a list of `ScriptField`s, each a tagged
float/int/bool/vec3/colour/text value). Instances live per entity on
`RuntimeSimulation::Record::scripts`; the *catalog* of definitions lives in
`EditorContext::script_catalog` and is repopulated from any script found while a scene loads, so
the Add Component ▸ Scripts menu survives a round-trip. "New Script…" scaffolds a `<Name>.hpp` C++
system stub in the project (Apache header, a `struct`, and a commented `app.system<…>().each(…)`
registration), opens it in the Text Editor, and registers + attaches the new type. Both UI params
and script components serialize with the scene and travel through the copy/paste clipboard
alongside the other optional components.

**Enabled/disabled lifecycle.** `RuntimeSimulation::Record::enabled` (default `true`) is Unity's
`activeSelf`: a real, hierarchical on/off switch, distinct from `visible`, which keeps its
original meaning as a local, non-cascading, render-only flag (Unity's `Renderer.enabled`).
`enabled_in_hierarchy` walks an entity's ancestor chain the same way `visible_in_hierarchy`
used to, over the new flag instead. Everything that gates on it uses that one walk: `extract()`'s
mesh/cloth/soft-body/vehicle-shell/particle/light/decal/crowd instancing loops (combined with
the local `visible` check), and four independent physics gather points — `physics_source_entities()`
(feeding `gather_rigid_descriptions`/`gather_static_planes`), `gather_vehicle_descriptions()`,
`gather_soft_body_descriptions()`, and `gather_cloth_descriptions()`, none routing through a
shared entry point — plus the editor's live audio poll in `AudioEditorSystem::update` through the
`IWorldEditor` seam. `set_enabled()` marks all four physics dirty flags (`physics_dirty_`/`soft_dirty_`/
`cloth_dirty_`/`vehicles_dirty_`) on a real change, unconditionally, so a toggle on an entity with
no physics component of its own still invalidates a descendant's gathered state. A disabled entity's
rigid body is removed from the physics world the same way `set_rigid_bodies`' existing diff already
removes a genuinely-destroyed one, and comes back at rest on re-enable — there is no velocity field
to preserve across the gap, and preserving one was deliberately scoped out (see [the entity lifecycle
design](../design/entity_lifecycle_system.md) §4.2 for the full audit).
