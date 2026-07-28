# Physics System — unified XPBD rigid + soft bodies, the cooking pipeline, and the road to vehicle-grade soft-body simulation (`SushiEngine::Physics`)

This document is the **umbrella** for SushiEngine's AAA physics simulation: the product vision (one
unified XPBD solver that carries rigid bodies, articulated assemblies, cloth, and volumetric soft
bodies with real material strength), the offline **cooking pipeline** that turns any imported mesh
into a simulation-ready asset with a single fidelity dial, the **penetration** contract that binds the
visible mesh to the simulated one, and the multi-phase road to a BeamNG-class deformable-vehicle
simulation. It specifies the **architecture and the seams**, not the kernel source.

Companion docs: `SUSHILOOP.md` (the determinism rules and the locked "GPU XPBD in double with a
floating origin" decision this plan implements), `vfx_particle_system.md` (the structural template for
a two-backend subsystem behind one seam), `animation_system.md` (the ragdoll/pose seam physics feeds),
`atmosphere_system.md` (the wind field vehicle aerodynamics and cloth will sample), and the renderer's
`../ARCHITECTURE.md` §4 (the physics layer as it stands today).

**Status (2026-07-28): nothing in this document is implemented.** §1 is an honest audit of what exists
now; everything from §3 onward is the plan. The roadmap in §16 marks each phase's state and is the
single place progress is recorded. The four decisions the project owner has settled — the hybrid
vehicle structure, cosmetic narrow precision, phase order, and running the simulation on SushiRuntime
— are recorded in §17.3; §6.6 is the concrete runtime execution model that follows from the last of
them, and §8.6 is the mesh-to-physics binding written out end to end.

---

## §0 The decisions that shape everything

Five decisions determine the whole design. Each is stated with its cost, because each one closes doors.

### 0.1 One solver, not a family of solvers

Every simulated thing — a crate, a car door on a hinge, a rope, a flag, a tyre sidewall, a dented
fender — is a set of **particles or rigid bodies plus constraints**, projected by one substepping XPBD
solver. `ARCHITECTURE.md` §4.1 already claims this and cloth already proves it; this plan holds the
line for FEM soft bodies, joints, contacts, and node-beam vehicles as well.

*Cost:* XPBD is a positional method, so quantities other engines get for free — exact constraint
forces, reduced-coordinate articulations, analytic stiffness — have to be **recovered** from Lagrange
multipliers rather than solved for directly (§10.4). We accept that; the payoff is that a car's chassis,
its door hinge, its deforming panel, and the cloth seat inside it all converge in the same sweep and
couple two-way for free.

### 0.2 Small steps, not many iterations

The solver takes **many substeps with one constraint iteration each**, not one step with many
iterations (Macklin et al. 2019, *Small Steps in Physics Simulation*). Error falls with the square of
the substep, so N substeps beat N iterations at identical cost, and stiff constraints (a suspension
spring, a rigid hinge) stop needing compliance hacks.

*Consequence:* the outer tick rate and the substep count are the two knobs that buy stability, and the
substep count must be derived from **simulation state, never from the wall clock** (SushiLoop's rule).
A 60 Hz tick at 32 substeps is a 1920 Hz effective solve — the rate a node-beam vehicle needs.

### 0.3 Cooked assets, not runtime discovery

Anything expensive and deterministic — tetrahedralization, convex decomposition, signed-distance
fields, bounding-volume hierarchies, mass properties, render-mesh embedding, simulation levels of
detail — is computed **once, offline, into a versioned blob keyed by a content hash**, exactly the way
`Animation::SkeletonBlob` and `Vfx::CompiledEffect` already work. The runtime loads and simulates; it
never cooks.

*Consequence:* the "drop a mesh in and it becomes a soft body" experience is an **import processor**
that runs the cooker in the background, not a runtime feature. And the fidelity dial is a *cooking*
parameter, so changing it re-cooks — which is why cooking has to be fast and cached.

### 0.4 The visible mesh is the simulated mesh

The penetration contract, stated once and enforced everywhere: **what the player sees never
interpenetrates more deeply than what the solver resolved.** Two mechanisms deliver it, and both are
mandatory, not optional quality settings:

- **Rigid:** collision geometry is a cooked approximation of the render mesh whose one-sided Hausdorff
  error is measured and reported at cook time, so "the collider is fatter than the mesh" is a number
  in the inspector rather than a surprise. Contacts carry a `rest_offset` so surfaces come to rest
  *touching*, and a `contact_offset` so they are generated *before* they touch (§7.6).
- **Soft:** the render mesh is barycentrically **embedded** in the simulated tetrahedral mesh, so it
  is not "synced" to the simulation — it *is* the simulation, interpolated. The collision surface is
  the simulated surface. There is no second geometry to disagree.

*Cost:* an embedded render mesh cannot be edited independently of its simulation mesh at runtime, and
a re-cook invalidates the embedding. Accepted.

### 0.5 Determinism is a build-time property, not a mode

Every part of this system lives **inside** SushiLoop's deterministic island: fixed substep count from
state, fixed constraint colour order, contacts sorted by a stable key before they are solved, no
wall-clock, no unseeded randomness, no iteration over a hash container where order is observable. GPU
execution is allowed because SushiLoop's promise is *same-binary* determinism, not cross-vendor
determinism.

*Cost:* several attractive optimizations are forbidden — non-deterministic atomic accumulation orders,
"solve whatever is ready" work stealing, floating-point reassociation. Where a parallel reduction is
needed, it is a **fixed-order** reduction (§12.2).

---

## §1 Audit — what exists today, honestly

`include/SushiEngine/physics/` is 2 233 lines of header-only templates and it is a *good* skeleton: the
XPBD core, graph colouring, a device solve graph, and a clean precision-parametric design. It is also
roughly 8 % of a AAA physics engine. This section is the honest inventory, because every phase in §16
is scoped against it.

### 1.1 What is there and works

| File | What it delivers |
|---|---|
| `rigid_body.hpp` | `RigidBodyT<T>`: pose, previous pose, velocity, diagonal body-local inverse inertia, inverse mass, quadratic drag. `predict()` / `update_velocity()` — the two halves of an XPBD substep. Trivially copyable, device-safe. |
| `xpbd_constraint.hpp` | `XpbdDistanceConstraintT<T>`: two body indices, two local anchors, a rest length, a compliance. **The only constraint type in the engine.** |
| `xpbd_solver.hpp` | `XpbdDistanceProjectionT<T>` (the correct generalized-inverse-mass projection with angular coupling) and `XpbdSolver<Constraint>`, which graph-colours the constraint set, uploads one buffer per colour, and compiles a replayable SushiRuntime graph of `iterations × colours` nodes. |
| `graph_coloring.hpp` | Greedy edge colouring over bodies. Correct, deterministic, host-side, `O(constraints × colours)`. |
| `physics_world.hpp` | `PhysicsWorld<Constraint>`: register bodies/constraints, `finalize()` once, then `step()` or the split `predict_substep_field()` / `solve_constraints()` / `derive_velocity()` trio that lets two worlds run in lockstep. |
| `collision.hpp` | Sphere, plane, axis-aligned box, oriented box; sphere-plane, sphere-sphere, box-sphere, box-plane, oriented-box-plane, oriented-box-sphere, and oriented-box-oriented-box by separating-axis test. Pure, unit-tested. |
| `broadphase.hpp` | `Aabb<T>`, overlap test, and `sweep_and_prune()` on the X axis. |
| `contact_solver.hpp` | `ContactBody<T>` — a shape plus live pointers into whichever buffer owns the body — generalized inverse mass with the angular term, single-point two-way resolution, and plane resolution. |
| `cloth.hpp` / `soft_body.hpp` | Topology builders: a 2D grid and a 3D structural+shear lattice of distance constraints. No new solver, exactly as intended. |
| `pgs_solver.hpp` | The earlier point-mass Projected Gauss-Seidel solver, superseded by XPBD but still tested and demoed. |
| `sim/physics_simulation.hpp` | `IPhysicsSimulation` — the boundary seam. Rebuild bodies, rebuild cloth grids, set static planes, step, read poses. Solve runs in `double`, converts at the edge. |
| `sim/physics_bridge.hpp` | The ECS-component half: `PhysicsBody` component and `sync_transforms_from_physics()`. |

Tests already cover the geometry, the colouring, the solver against a byte-for-byte host mirror, the
bridge, and cloth. That test culture is the reason this plan can be aggressive.

### 1.2 What is missing, by consequence

These are not "nice to have" — each one is load-bearing for something the user asked for.

1. **No joints.** The only constraint is a distance constraint. A hinged car door is unbuildable today:
   there is no hinge, ball, slider, fixed, or cone-twist constraint, no limits, no motors, no drives,
   and no articulation concept at all. *(Blocks: assemblies, MBD, ragdolls, vehicles.)*
2. **No friction and no restitution, anywhere.** `contact_solver.hpp` is a positional projection only;
   velocity is whatever the position change implies. Every contact in the engine is perfectly
   inelastic and perfectly frictionless. A box cannot rest on a ramp, a ball cannot bounce, a tyre
   cannot grip. *(Blocks: everything that matters.)*
3. **Single-point contact manifolds.** `collide_obb_obb` returns the midpoint of the two support
   points. `contact_solver.hpp`'s own comment says it: *"a box resting on a face is held by one corner
   at a time and rocks slightly."* Stacking, resting, and any large flat contact are wrong.
4. **The narrowphase knows three shapes.** Sphere, oriented box, half-space plane. `ColliderParams`
   authors Box / Sphere / Cylinder / Plane, and `RuntimeSimulation::gather_rigid_descs()` collapses
   anything that is not a Box into **a sphere of one radius** — a Cylinder collider silently simulates
   as a sphere. No convex hulls, no capsules, no triangle meshes, no height fields, no signed-distance
   fields. Static world geometry can only be an **infinite half-space plane**.
5. **No entity scale reaches the collider.** `Transform::scale` is a render concept; the physics reads
   the authored collider extents directly. Scaling an object in the editor does not scale its physics.
6. **No mass properties.** `PhysicsBodyParams` makes the author type an inverse mass and a *diagonal
   inverse inertia tensor* by hand. Nothing computes inertia from a shape or a density.
7. **No continuous collision.** Nothing is swept, nothing is speculative. A fast body tunnels.
8. **The world cannot change shape.** `PhysicsWorld::finalize()` compiles the colour graph once; any
   change to the body count rebuilds the entire world and recompiles the graph
   (`PhysicsSimulation::set_rigid_bodies`). Adding a body, breaking a constraint, fracturing an
   element, or streaming a chunk all cost a full rebuild. *(Blocks: fracture, breakable joints,
   streaming, spawning.)*
9. **No islands, no sleeping.** Every body is integrated and every constraint is projected every
   substep forever. A scene of a thousand settled crates costs the same as a thousand tumbling ones.
10. **The broadphase is rebuilt from scratch, twice per substep.** `PhysicsSimulation::resolve_contacts`
    runs `CONTACT_ITERATIONS = 2` sweeps, and *each* sweep refills the AABB array and re-runs
    `sweep_and_prune`, which sorts `O(n log n)` and allocates two vectors internally. It is also
    hard-coded to the X axis — a scene laid out along a road is the degenerate case.
11. **Only the constraint projection is on the device; everything around it is a host loop.** Per
    substep, `PhysicsWorld::step` runs a host loop over every body (`predict`), then
    `XpbdSolver::solve()` — which **zeroes every Lagrange multiplier in a host loop over every
    constraint** before replaying the graph — then the contact pass on the host, then another host
    loop over every body (`update_velocity`). At 32 substeps and 100 000 constraints that is
    3.2 million host-side writes per tick before any physics happens, plus 32 blocking
    host↔device round trips.
12. **The solve graph cannot change size without recompiling.** `XpbdSolver::build_graph` bakes
    `Extent{n}` per colour at construction, using the capture-style `Graph::add(Extent, Cap0, ...)`
    overload — which is the one overload with **no `Dynamic` counterpart** in the runtime API. Since
    contacts appear and vanish every tick, this is the reason contacts were pushed out of the solver
    and into a host pass in the first place (`contact_solver.hpp`'s own file comment says so). §6.6
    shows this is a solvable API-shape problem, not a fundamental one.
13. **Soft bodies are mass-spring lattices.** `build_soft_body_lattice` links axis and face-diagonal
    neighbours with distance constraints. There is no volume preservation, so it collapses and inverts
    under load; no bending resistance; no material parameters (Young's modulus, Poisson ratio,
    density); no stress; no plasticity; no fracture; and no relationship to any imported mesh.
14. **No mesh reaches the physics at all.** The only triangle-mesh representation in the engine lives
    behind Vulkan in `render/geometry/mesh_registry.hpp` (`Geometry::MeshVertex`, device buffers), and
    the only signed-distance baker (`render/gi/mesh_sdf_baker.hpp`, `Gi::MeshSdfBrick`) is a
    render-side global-illumination tool. There is no engine-neutral triangle mesh, no cooking step,
    no physics asset, and no import hook. **This is the single biggest gap relative to what the
    project wants**, and §8 exists to close it.
15. **`ContactBody::is_cloth` is a type tag inside a value type.** Behaviour switches on a boolean
    (`if (a.is_cloth && b.is_cloth) return;`). Adding soft bodies, characters, triggers, or vehicles
    this way multiplies the flags and the branches — the exact Open/Closed violation §4 exists to stop.
16. **Cloth rebuilds lose all state**, by documented design: any `ClothParams` edit replaces the grid.
17. **No queries.** No raycast, no sweep, no overlap, no trigger volumes, no collision layers or
    filters, no contact events. Gameplay cannot ask the physics anything.
18. **No character controller, no kinematic bodies.**
19. **No profiling.** The physics contributes nothing to the GPU profiler or any statistics panel.

### 1.3 Two small correctness notes worth fixing early

- `collide_box_sphere` and `collide_obb_sphere` push out along **+Y** when the sphere centre is exactly
  inside the box (distance ≈ 0), rather than out of the nearest face. Deep penetration recovery picks
  an arbitrary direction.
- `contact_solver.hpp`'s plane path scales the impulse by `inv_mass / w` so the centre-of-mass motion
  matches the old purely-positional behaviour. That is a deliberate compatibility choice, and it means
  the angular share of a plane contact does **not** conserve the correction — it is not the same
  projection the pair path uses. Worth unifying when manifolds arrive.

---

## §2 Survey — what this system adopts

Model only. No code, no dependency, unless §17.2 says otherwise.

| Source | What SushiEngine takes |
|---|---|
| Macklin et al. 2019, *Small Steps in Physics Simulation* | The substepping schedule: many substeps, one iteration each. The single most valuable idea in the plan. |
| Müller et al. 2020, *Detailed Rigid Body Simulation with XPBD* | Rigid contact with static friction in the positional pass and dynamic friction + restitution in a velocity pass; the generalized-inverse-mass formulation the engine already uses for distance constraints. |
| Macklin & Müller 2021, *A Constraint-based Formulation of Stable Neo-Hookean Materials* | The two-constraint (deviatoric + hydrostatic) tetrahedral FEM soft body. This is the soft-body model. |
| Müller et al. 2005, *Meshless Deformations Based on Shape Matching* | The cheap soft-body level of detail: a shape-matching cluster for distant or low-fidelity bodies. |
| Bridson et al. 2002, *Robust Treatment of Collisions, Contact and Friction for Cloth Animation* | Continuous vertex-triangle and edge-edge collision with thickness; the soft/cloth self-collision model. |
| **PhysX** | `contact_offset` / `rest_offset` as the visible-penetration contract; persistent contact manifolds with warm starting; scene queries, filter shaders, and the collision-layer model; the convex-decomposition-based dynamic mesh collider policy. |
| **Havok / Jolt** | Island detection and sleeping; the "large island split" idea for parallel solving; deterministic body ordering as an explicit design goal. |
| **Chaos (Unreal)** | The cooked "physics asset" concept: an authored bag of parts, shapes, and joints instanced as one unit; strain-based joint and cluster breaking. |
| **BeamNG.drive** | The node/beam soft-body vehicle: point-mass nodes, spring-damper beams with *deform* and *break* thresholds, bracing beams for torsion, and a high substep rate. The permanent-deformation model in §11 is this, expressed as XPBD plasticity. |
| **Houdini Vellum / Blender** | The tetrahedralization-and-embed workflow: cook a simulation mesh at a chosen resolution, embed the render mesh in it, simulate the coarse mesh, display the fine one. |
| **V-HACD** | Approximate convex decomposition as the default rigid mesh collider strategy. |
| SushiEngine `Vfx::EmitterCompiler` | The authoring→compile→POD-blob pipeline shape (`vfx_particle_system.md` §3). The physics cooker is the same shape with heavier geometry. |
| SushiEngine `Animation::SkeletonBlob` | The versioned, trivially-loadable cooked-asset blob format and its import path. |
| SushiEngine `Gi::MeshSdfBrick` | An existing, working signed-distance baker to **lift and share**, not to duplicate (§3.4). |
| SushiRuntime `SIMULATION_ENGINE_SUBSTRATE_PLAN.md` | The substrate contract: late-bound sizes, region graphs, device residency, native host nodes, and the WP-4 determinism analysis. Its locked decisions L7/L8/L11/L12/L14 name this system as the runtime's target workload (§6.6). |

**Skip list, deliberately:** fluid simulation (SPH/FLIP/PBF) — a separate system, not this one; runtime
Voronoi fracture of arbitrary rigid geometry (§17.1 revisits it); reduced-coordinate Featherstone
articulations as the *primary* path (§10.5 keeps it as an escape hatch); cross-vendor bit-exact
determinism (SushiLoop already ruled it out); soft-body **self**-collision at cloth scale in the first
soft-body phase (§9.6 schedules it).

---

## §3 Target architecture

### 3.1 The layer map

The dependency direction is strictly one way. Nothing lower knows anything about anything higher.

```
 editor/physics/            Inspector panels, gizmos, bake UI, debug draw, profiler view
 ──────────────────────────────────────────────────────────────────────────────────────
 sim/                       IPhysicsScene facade + ECS components + extract  (the boundary)
 ──────────────────────────────────────────────────────────────────────────────────────
 physics/scene/             PhysicsScene: bodies, islands, sleeping, events, queries, stepping
 physics/vehicle/           Node-beam assemblies, powertrain, tyres, aerodynamic sampling
 physics/soft/              FEM tetrahedral bodies, cloth, ropes, plasticity, fracture
 physics/constraints/       Joint library: fixed, ball, hinge, slider, cone-twist, distance, motors
 physics/solver/            Unified substepping XPBD: colouring, projection dispatch, velocity pass
 physics/collision/         Broadphase, narrowphase, manifolds, continuous collision, queries
 physics/geometry/          Shapes, signed-distance fields, bounding-volume hierarchies, mass properties
 physics/cooking/           The offline pipeline: mesh in, simulation asset out       (host-only)
 physics/core/              Body state, materials, handles, precision policy, configuration
 ──────────────────────────────────────────────────────────────────────────────────────
 geometry/                  Engine-neutral TriangleMesh + the shared signed-distance baker (NEW, §3.4)
 ──────────────────────────────────────────────────────────────────────────────────────
 SushiRuntime               Task graph, unified shared memory, SYCL
```

`physics/cooking/` is **host-only and never linked into a shipping runtime path** — it is the one
module allowed to be slow, allocate freely, and depend on `geometry/`.

### 3.2 What each module owns (single responsibility, stated as a sentence)

| Module | Owns exactly | Never |
|---|---|---|
| `physics/core` | Body state, handles, `PhysicsMaterial`, `PhysicsConfiguration`, the precision policy. | Knows about shapes or solvers. |
| `physics/geometry` | Shape value types, signed-distance sampling, bounding-volume hierarchy traversal, mass-property computation. | Knows about bodies or constraints. Pure geometry, pure functions, unit-testable in isolation — the property `collision.hpp` already has and must keep. |
| `physics/collision` | Turning a set of shapes and poses into a sorted set of contact manifolds. | Moves anything. It reports; it does not resolve. |
| `physics/solver` | Turning a set of constraints and manifolds into corrected positions and velocities. | Knows what a shape is, or where a contact came from. |
| `physics/constraints` | Joint descriptors and their projections. | Owns bodies. |
| `physics/soft` | Soft-body topology, material response, plastic and fracture state. | Owns collision or the solve schedule. |
| `physics/vehicle` | Node-beam assemblies, powertrain state, tyre response. | Reimplements a solver. |
| `physics/scene` | The lifecycle: what exists, what is awake, what island it is in, what happened this tick, and the order everything runs in. | Contains physics *math*. It is an orchestrator. |
| `physics/cooking` | Everything expensive that can be precomputed. | Runs at simulation time. |

### 3.3 The seams (dependency inversion points)

Each is a pure-virtual interface owned by the **consumer**, so an implementation can be swapped
without the consumer changing (§4.4).

```cpp
// physics/collision/broadphase_interface.hpp
class IBroadphase                 // BoundingVolumeHierarchyBroadphase | SweepAndPruneBroadphase | DeviceBroadphase
class INarrowphase                // shape-pair dispatch; a new shape adds a registration, not a branch
// physics/solver/solver_interface.hpp
class IConstraintSolver           // HostXpbdSolver | DeviceXpbdSolver
// physics/soft/soft_body_model.hpp
class ISoftBodyModel              // MassSpringModel | FiniteElementModel | ShapeMatchingModel  (LOD swap)
// physics/cooking/cooker_interface.hpp
class IMeshCooker                 // CollisionCooker | SoftBodyCooker | NodeBeamCooker
class ICookedAssetStore           // where blobs live; the runtime only ever reads through this
// physics/scene/event_sink.hpp
class IPhysicsEventSink           // gameplay, audio impacts, VFX impacts — physics never calls them directly
// sim/
class IPhysicsScene               // the ECS-facing facade over the ISP-split services (§4.3)
```

### 3.4 One refactor this plan depends on: a neutral `geometry/` module

`render/gi/mesh_sdf_baker.hpp` bakes a signed-distance brick from `Geometry::MeshVertex`. The physics
needs exactly that, plus a triangle mesh it can traverse on the host and the device, and it must not
depend on Vulkan to get it.

**Action (phase P0):** create a top-level `geometry/` module holding an engine-neutral
`Geometry::TriangleMesh` (positions, indices, optional normals, bounds — no device handles) and move
the signed-distance baker there as `Geometry::SignedDistanceFieldBaker`. `render/gi/` and
`render/geometry/mesh_registry.cpp` consume it from the new home; `Gi::MeshSdfBrick` becomes a thin
alias or is replaced outright. This is a **pure move plus a rename**, doc-visible but behaviour-neutral,
and it unblocks every mesh-aware feature in this plan.

*Why this and not "physics reads the mesh registry":* the physics layer sits **below** the renderer in
the dependency order. Pointing it at a Vulkan-owning class inverts the layering and makes the cooker
require a device. The neutral module is the correct fix, and the renderer benefits too — a mesh's
distance field stops being a global-illumination private.

---

## §4 The SOLID contract

The user's stated first priority, and `CLAUDE.md`'s. Stated per principle, with the concrete mechanism
that enforces it and the test that catches a violation.

### 4.1 Single responsibility

The table in §3.2 *is* the contract. Two specific splits it forces on today's code:

- **`PhysicsSimulation` does five jobs today** — it owns two worlds, converts precision, rebuilds
  topology, runs the whole contact pass inline (`build_contact_bodies`, `resolve_contacts`), and
  steps. The contact pass moves to `physics/collision`; the world lifecycle moves to
  `physics/scene`; what remains at the `sim/` boundary is *marshalling only*.
- **`RuntimeSimulation` gathers descriptors** (`gather_rigid_descs`, `gather_static_planes`,
  `collision_radius`). That is a translation responsibility, and it belongs in one
  `Simulation::PhysicsExtract` unit, tested on its own, the way the render extract is.

### 4.2 Open/closed — the three extension points that must never require editing existing code

This is where a physics engine usually rots. Three registries, three rules:

**A new collision shape** adds (1) a shape value type in `physics/geometry`, (2) an entry in the
narrowphase **dispatch table**, keyed by the ordered shape-type pair, (3) a mass-property function.
Nothing else is touched. The dispatch table is a `constexpr` two-dimensional array of function
pointers indexed by `ShapeType`, populated by registration, not a `switch` — because a `switch` is a
file every new shape must edit, and that is the violation.

**A new constraint or joint** adds (1) a trivially-copyable descriptor POD exposing `body_a` /
`body_b` (so `color_constraints` works unchanged — it already only needs `.a` and `.b`, and gains a
uniform accessor), (2) a captureless projection functor with the established
`operator()(const Constraint&, Body*, Real& lambda, Real h)` signature, (3) one registration line in
the solver's constraint-kind list. `XpbdSolver` gains **nothing** — this is the pattern
`XpbdDistanceProjectionT` already establishes, generalized to a heterogeneous constraint set (§6.3).

**A new soft-body material model** implements `ISoftBodyModel` and is selected by the cooked asset. The
solver schedules it by the constraints it emits; it does not know the model exists.

The cooker follows the same rule: a cooking stage is an `ICookingStage` in an ordered pipeline
(`Repair → Voxelize → Tetrahedralize → Optimize → Embed → Decompose → BakeDistanceField → BuildLevelsOfDetail → Serialize`),
so adding a stage inserts an object rather than editing a function.

### 4.3 Interface segregation — splitting `IPhysicsSimulation`

Today's single interface mixes rigid bodies, cloth, static planes, and stepping. Every consumer
depends on all of it. It becomes:

```cpp
class IRigidBodyService     { add/remove/update/pose/teleport/apply_impulse/... };
class ISoftBodyService      { instantiate/release/vertices/stress/plastic_state/... };
class IClothService         { ... };
class IJointService         { create/destroy/set_motor/set_limit/joint_force/break/... };
class IAssemblyService      { instantiate(PhysicsAssembly)/release/part_body/... };
class ICollisionQueryService{ raycast/sweep/overlap/closest_point/... };
class IPhysicsStepper       { step(substeps, gravity_sampler)/statistics/... };

class IPhysicsScene : public IRigidBodyService, public ISoftBodyService, /* ... */ {};
```

A gameplay system that only raycasts depends on `ICollisionQueryService` and nothing else. The editor's
soft-body panel depends on `ISoftBodyService`. `IPhysicsScene` exists only for the host that owns the
whole thing.

### 4.4 Liskov — the substitutability tests

Every seam in §3.3 ships with a **shared conformance test suite** run against every implementation:
`IBroadphase` implementations must all emit the same sorted pair set for the same input;
`IConstraintSolver` implementations must agree within a stated tolerance on a reference scene;
`ISoftBodyModel` implementations must all converge to the same rest shape. This is how a
device solver is allowed to replace a host solver without silently changing behaviour, and it is the
generalization of what `test_xpbd_solver.cpp` already does with its host mirror.

The `ContactBody::is_cloth` flag (§1.2 item 15) is deleted: behaviour differences become **filter
masks and material properties**, which are data, not type tags.

### 4.5 Dependency inversion — who names whom

- `physics/` names `Geometry::TriangleMesh`. It never names `Render::Geometry::MeshRegistry`, Vulkan,
  or the editor.
- `physics/scene` reports collisions to an `IPhysicsEventSink` it is handed. It never calls audio,
  VFX, or gameplay.
- The cooker writes through `ICookedAssetStore`. It never writes files directly, so the editor, the
  test suite, and a future content server all plug in.
- `sim/` continues to hand the physics a `GravitySampler` — an existing, good example of this
  principle already in the codebase — and gains a `WindSampler` alongside it, backed by the
  atmosphere system's wind field (§11.6).

---

## §5 The data model

### 5.1 Bodies

`RigidBodyT<T>` stays the state column and grows the fields a real engine needs, all still trivially
copyable:

```
  position, orientation, previous_position, previous_orientation     (existing)
  velocity, angular_velocity                                          (existing)
  inverse_mass, inverse_inertia (diagonal, body-local)                (existing)
  drag_coefficient                                                    (existing)
+ center_of_mass_local          — the offset from the origin to the centre of mass
+ material_index                — into the scene's PhysicsMaterial table
+ flags                         — Dynamic | Kinematic | Static | Sleeping | ContinuousCollision | Trigger
+ sleep_timer, motion_measure   — the exponentially-smoothed motion metric sleeping reads
+ island_index                  — assigned by the scene each tick
```

`inverse_inertia` stays a diagonal in the body-local frame — the existing choice is correct and the
projection code already handles the similarity transform. Bodies whose cooked inertia tensor is not
diagonal store the **principal-axis rotation** produced by the cooker's eigendecomposition, folded into
`center_of_mass_local`'s frame.

### 5.2 Shapes

```
Sphere · Capsule · Box · ConvexHull · TriangleMesh(static) · HeightField · SignedDistanceField · Compound
```

A shape is a value type in `physics/geometry`, referencing cooked data by handle (a convex hull's plane
and vertex arrays, a mesh's bounding-volume hierarchy, a distance field's brick) so a shape itself
stays small and copyable. `Compound` is a list of child shapes with local transforms — this is how one
body gets several colliders, which is how a car chassis is one body with a dozen convex pieces.

Every shape carries a **`convex_radius`** (a small inflation used by the closest-point routines to keep
them numerically stable — the standard trick) and the body-level `contact_offset` / `rest_offset`
(§7.6).

### 5.3 Materials

```cpp
struct PhysicsMaterial
{
    Scalar static_friction, dynamic_friction, restitution, density;
    Scalar rolling_friction, spinning_friction;      // needed for wheels and settling spheres
    FrictionCombineMode friction_combine;            // Average | Minimum | Maximum | Multiply
    RestitutionCombineMode restitution_combine;
};
```

An editor asset, referenced by index. Soft bodies extend it with `SoftBodyMaterial` (§9.2).

### 5.4 Cooked assets — the blob family

All four follow `Animation::SkeletonBlob`'s shape: a versioned header, a content hash, offset-based
sections, no pointers, memory-mappable, loadable with one read.

| Asset | Extension | Contains |
|---|---|---|
| `CollisionAsset` | `.sushicollision` | Convex hull set (or a triangle-mesh bounding-volume hierarchy for static geometry), mass properties, a narrow-band distance field, the measured Hausdorff error against the source mesh. |
| `SoftBodyAsset` | `.sushisoft` | Tetrahedral mesh (vertices, tetrahedra, rest inverse matrices, rest volumes), surface triangle set, render-mesh embedding (per render vertex: tetrahedron index + barycentric weights), per-level-of-detail lattices and the mappings between them, a rest-shape distance field, and the cooking parameters that produced it. |
| `PhysicsAssembly` | `.sushiassembly` | Parts (each naming a `CollisionAsset` or `SoftBodyAsset`, a local transform, a material, a mass override), joints (kind, the two part indices, frames, limits, motors, break thresholds), and collision-filter groups. This is the car. |
| `NodeBeamAsset` | `.sushinodebeam` | Nodes (rest position, mass, radius, material), beams (node pair, rest length, stiffness, damping, deform threshold, break threshold, group), bracing sets, and the render-mesh skinning weights onto the node cloud. |

### 5.5 The ECS surface

New components (trivially copyable, per `components.hpp`'s rule) and new host-side records:

```
  RigidBody            (exists as host record; gains material, flags, collision filter)
+ SoftBody             { SoftBodyAssetId asset; Scalar fidelity_scale; std::uint32_t flags; }
+ PhysicsJoint         { EntityId body_a, body_b; JointKind kind; ... }        // host record + asset
+ AssemblyInstance     { AssemblyAssetId asset; std::uint32_t root_part; }
+ VehicleInstance      { NodeBeamAssetId asset; PowertrainState state; }
+ CollisionFilter      { std::uint32_t layer; std::uint32_t collides_with_mask; }
```

`ColliderParams` is superseded by a `Collider` record that can name a cooked `CollisionAsset` **or** a
primitive, and that finally honours `Transform::scale` (§1.2 item 5).

---

## §6 The unified solver

### 6.1 The tick

```
PhysicsScene::step(dt, substeps):
    broadphase.update(swept_bounds)                 # once per tick, not per substep
    pairs = broadphase.overlapping_pairs()          # persistent, incremental
    manifolds = narrowphase.generate(pairs)         # persistent, warm-started, sorted by stable key
    islands = island_builder.build(bodies, constraints, manifolds)
    wake/sleep islands
    for substep in 0..substeps:                     # h = dt / substeps
        integrate_positions(h)                      # predict; gravity + wind sampled per body
        solve_positions(h)                          # one iteration: joints, soft-body, contacts
        derive_velocities(h)                        # v = (x - x_prev) / h
        solve_velocities(h)                         # dynamic friction, restitution, joint damping
    write_back(), fire_events()
```

Two deliberate departures from what exists today:

- **Collision detection runs once per tick, not once per substep.** Manifolds are generated against
  *swept* bounds with a contact offset, then re-evaluated (depth and points refreshed, features kept)
  cheaply each substep. This is the standard XPBD-with-substeps arrangement and it is what makes 32
  substeps affordable. Today's code runs a full broadphase twice *per substep*.
- **A velocity pass is added.** XPBD's positional projection cannot express restitution or dynamic
  friction; both are velocity-level. Today's engine has no velocity pass at all, which is exactly why
  it has neither.

### 6.2 Substep count derived from state

```
substeps = clamp(ceil(max_body_motion_per_tick / motion_budget), minimum, maximum)
```

where `max_body_motion_per_tick` is the largest `|velocity| * dt / characteristic_size` over awake
bodies. Derived from **simulation state only** — SushiLoop's determinism rule. A vehicle scene pins it
near the maximum; an idle scene drops to the minimum. Per-island substepping is a §13 optimization,
not a first-phase feature.

### 6.3 Heterogeneous constraints in one graph

`XpbdSolver<Constraint>` is single-typed today. The generalization keeps the compile-once-replay
structure and adds a **kind dimension**:

- Constraints are stored in **per-kind, per-colour buffers**. Colouring runs over the union of all
  constraints (a joint and a distance constraint on the same body pair must not share a colour), so
  `color_constraints` is generalized to take a body-pair accessor rather than requiring `.a`/`.b`
  fields — one small change, and every existing caller keeps working.
- The graph emits one node per `(kind, colour)` pair. Within a colour the kinds are independent by
  construction, so their relative order does not matter; across colours the runtime orders them, as it
  does now.
- Contacts are a constraint kind like any other, but their buffers are **rebuilt each tick** rather
  than at finalize time — which is why §6.4 exists.

### 6.4 The world must be mutable

The single most important structural fix. `PhysicsWorld` gains:

```cpp
BodyHandle add_body(...);          // valid immediately, no finalize
void       remove_body(BodyHandle);
ConstraintHandle add_constraint(...);
void       remove_constraint(ConstraintHandle);
```

backed by:

- **Generational handles** over a free-list, so a removed body's index can be reused without a stale
  handle silently addressing the wrong body.
- **Fixed-capacity device buffers with a live count**, not capacity doubling. A `SushiRuntime::Buffer`
  cannot be resized in place — a growth reallocates and moves, which invalidates the pointer every
  graph node captured. So the scene allocates at a configured capacity, the *live* element count is a
  per-step value (§6.6), and exceeding capacity is a budgeted, reported, recomposing event rather than
  a routine one.
- **Incremental recolouring**: a new constraint takes the lowest colour free on both its bodies (the
  existing greedy rule, applied to one constraint); a removed constraint vacates its slot. A full
  recolour runs only when the colour count would grow past a threshold or fragmentation exceeds a
  budget — and it is scheduled, not stop-the-world.
- **Graph recomposition only when the (kind, colour) *structure* changes**, not when counts within a
  colour change. This is the point §6.6 makes precise.

This unblocks fracture, breakable joints, spawning, streaming, and vehicle damage — all of which are
"the constraint set changed this tick."

### 6.5 Precision

The gameplay solve stays `double`: SushiLoop's locked decision, `Scalar` is already double, and a
planet-scale world with a floating origin needs it. Everything precision-related is already a template
parameter (`RigidBodyT<T>`, `XpbdDistanceConstraintT<T>`, `PhysicsWorld<Constraint>::Real`), so this is
a policy choice per body kind, not new machinery.

**Cosmetic bodies get a narrower column.** A soft body or cloth marked non-gameplay lives outside the
deterministic island's authority in exactly the way cosmetic VFX already do, so it may run:

- **`float`** as the default cosmetic column — half the bandwidth, and bandwidth is what a soft-body
  solve is actually limited by.
- **`sycl::half` for storage only**, where profiling shows it pays: positions and velocities stored at
  half precision and widened to `float` inside the projection. Half-precision *arithmetic* is not used
  for the projection itself — a neo-Hookean solve in 11 significant bits is not a stability trade, it
  is a broken solve. The rule is **half to store, float to compute**, and it applies to a curtain, a
  flag, or a distant vehicle's shell, never to anything a rollback replays.

The precision of a body is a property of its cooked asset and its component flags, resolved once at
instantiation. A body cannot change precision while it is simulating.

Positions are relative to the floating origin, which the engine already implements and tests
(`test_floating_origin.cpp`, `test_floating_origin_stress.cpp`). Physics must rebase with it: a
`rebase(offset)` on the scene shifts every position and every cached contact point in one pass.

### 6.6 Execution on SushiRuntime

**Yes — the simulation runs on SushiRuntime**, and more of it than does today. This section is the
concrete mapping, because "put it on the GPU" is not a plan and the runtime's shape dictates the
solver's shape. Everything below is checked against the runtime tree at
`../sushiruntime` (`docs/ARCHITECTURE.md`, `include/SushiRuntime/api/`) as of 2026-07-28.

#### The runtime was designed for this workload

Worth stating first, because it changes how much of this plan is speculative. SushiRuntime's own
engineering plan (`sushiruntime/docs/slop/SIMULATION_ENGINE_SUBSTRATE_PLAN.md`) records the owner's
locked decisions, and four of them name this system directly:

- **L7** — *"Full physics is in scope: particle/N-body, grid/field PDE, soft-body/constraint, **and
  rigid body**, with **collision detection as a central feature**. Frame-varying graph topology is
  guaranteed (collision pairs change every frame). Solvers need ordered islands."*
- **L8** — *"Constraint/rigid solver style is Gauss-Seidel / PGS. Needs **graph colouring** to
  parallelize within a colour and ordered dependency chains across colours. The dependency tracker
  provides the ordering; the ECS/physics layer provides the colouring."*
- **L11** — streaming regions with **sleeping islands** as a first-class requirement.
- **L14** — the role split: *"SushiEngine = the engine (entities, physics, scenes — the what);
  SushiRuntime = computes and schedules (the when / where / how)."*

So the late-binding, region-graph, and device-residency machinery this section leans on was not
built for something else and borrowed; it was built for exactly this. What remains is for the engine
side to actually use it — the current physics uses almost none of it.

#### What runs where

| Stage | Where | Why |
|---|---|---|
| Broadphase bound update, hierarchy refit, pair generation | **Runtime graph** | Per-body and per-pair parallel; the classic linear-bounding-volume-hierarchy build maps cleanly. |
| Narrowphase and manifold generation | **Runtime graph** | One task per candidate pair. |
| Contact sorting (the determinism requirement, §12.1) | **Runtime graph** | A fixed-order key sort, not a hash. |
| Constraint projection (joints, soft-body elements, contacts) | **Runtime graph** | Already there; extended to every constraint kind. |
| Predict, derive velocity, velocity pass | **Runtime graph** | Per-body parallel. These are host loops today (§1.2 item 11) and have no business being so. |
| Island building, sleep transitions, level-of-detail selection | **Runtime graph, as native host nodes** | Sequential traversal over a small structure — but it does not have to leave the graph. `Graph::add_host(reads, writes, fn)` marks a task that submits no device work; the scheduler then runs it **directly on the worker thread** and reports an empty event, so its successors release through the ordinary path with no SYCL queue acquired. Sequential work stays inside the one composition. |
| Constraint colouring, handle allocation, topology change | **Host, between ticks** | Build → compile → execute are distinct phases and the first two are **not thread-safe relative to each other** (runtime invariant 1). Topology mutation therefore happens at a step boundary on the simulation thread, never against a running graph. |
| **The whole cooking pipeline** | **Host** | Offline, one-time, allocation-heavy. Optionally accelerated (below). |

#### The five runtime facilities this design leans on

The runtime already provides what a physics engine needs. The current physics code predates or
ignores every one of them.

**1. `Dynamic` — late-bound size and enablement.**
`Graph::add(Dynamic dyn, const Reads&, const Writes&, std::size_t capacity, fn)` compiles once but
polls `when(predicate)` and `sized(count_provider)` **once per step, on the driver thread**, before
the plan replays. The runtime's own words: *"an ECS-style graph that spawns/destroys entities or
sleeps subsystems per frame compiles once — `compile_count()` stays 1."*

This is the answer to §1.2 item 12. Contact buffers are allocated at capacity; the per-tick contact
count is a `sized()` provider; a colour with nothing in it this tick is skipped by `when()`. **The
graph stops needing recomposition when counts change**, which is what finally lets contacts live
inside the solve graph instead of in a host pass.

Three exact constraints that come with it, and they shape the buffer design:

- **Bindings may change size and enablement, but never *which resources* a node touches** — the
  dependency edges are fixed at compile. So a node's buffer set is immutable for the life of the
  composition, which is precisely why §6.4 allocates at capacity instead of growing.
- **The live size must not exceed the baked capacity.** Exceeding it is the budgeted, reported,
  recomposing event, not a routine one.
- **The `Dynamic` overload's callable is `fn(i)`** — one index, no buffer arguments. The kernel
  captures the raw USM pointers by value instead of receiving them, which is safe exactly because
  the buffers never move. This is a real signature change from the capture-style
  `add(Extent, In(...), ..., fn(id, ptrs...))` form `XpbdSolver` uses today, and it is the enabling
  P0 task for most of §7 and §13.

**2. `DynamicGraph` — regions that stream in and out.**
Regions are keyed by a caller-chosen identity; `region(key)` records with the same `add()` surface,
`drop(key)` removes one, and mutations apply **at a step boundary, never against a running DAG**. A
commit costs *O(changed region + affected boundary pins)*, and `compose_count()` advances once per
mutation, not per region. Physics maps **one region per island**: a sleeping island is dropped, a
waking one composed. That is how §13.2's "a settled island costs nothing" is actually implemented.

The constraint to design around: **cross-region edges are derived in ascending region-key order — the
lower-keyed region is the producer.** Island keys must therefore be assigned deterministically and,
where two islands genuinely share an allocation, ordered intentionally. Islands are disjoint by
construction, so the common case has no cross-region edges at all; the shared static-geometry
hierarchy is the exception and is read-only, and read/read sharing stays parallel.

**3. Sub-region dependency tracking.**
The tracker keys on `ResourceRegion{base, offset, length}` and orders by **overlap**, not pointer
equality: *"disjoint sub-regions stay parallel"*. `Buffer::region(ElementRange)` names a slice, and
`Reads(...)`/`Writes(...)` accept it in place of a whole buffer. So the per-colour constraint views
become slices of **one** constraint allocation rather than one allocation per colour (today's
`constraint_buffers_` vector), and two colours writing disjoint slices are still parallel.

**4. Residency, and reading back.**
`Residency::Device` allocates device-only USM behind the same handle API. The hot state columns
become device-resident, which is what stops the per-substep host traffic in §1.2 item 11. The price
is explicit and worth stating: **`operator[]` and the bulk `host()` span throw `invalid_access` on a
device-resident handle** — the host uses `read_range()` / `write_range()`, which copy a sub-range
through the owning queue. Every host-side loop in today's physics has to go, not be ported.

**5. `add_host()` — native CPU nodes inside the graph.**
A task that submits no device work runs directly on the worker thread and reports an empty event, so
its successors release through the ordinary path. The sequential stages (island building, sleep
transitions, level-of-detail selection) stay **inside** the one composition instead of splitting the
tick into several `run()` calls.

#### Two runtime constraints that are easy to get wrong

- **The fluent API only tracks dependencies through handle-based overloads.** The runtime cannot infer
  what a kernel touches — a per-element callable is `void(std::size_t)` capturing raw USM pointers —
  so every physics node must name its data with `In`/`Out`/`InOut` or `Reads`/`Writes`. The runtime
  now spells the dependency-blind overloads `add_untracked` (§18, R5), so the rule this plan needs
  becomes a mechanical one: **`add_untracked` does not appear anywhere under `physics/`**. Every
  launch shape has a tracked overload, so there is never a reason for it to.
- **One node's buffers must share one device context.** USM is context-bound; the scheduler verifies
  co-location on a node's first dispatch and fails the run with `invalid_graph` if a node's buffers
  straddle a boundary. All of a scene's physics allocations are pinned to one `DeviceIndex`.
  Multi-device physics is a domain-decomposition problem the runtime defers to its own WP-5, so this
  plan targets one device per scene and says so rather than discovering it later.

#### The rule that follows: one `run()` per tick, not one per substep

`Graph::run()` blocks. The runtime is a throughput engine with no asynchronous step, so 32 substeps
implemented as 32 blocking `run()` calls is 32 round trips per tick, and at a 60 Hz tick that latency
dominates everything else in this document.

**The whole substep loop is unrolled into the graph** — predict, the colour sweep, derive, velocity
pass, repeated `substeps` times — and the tick issues exactly **one** `run()`. `XpbdSolver` already
does this for its *iterations*; the plan extends it to the substeps and to the stages around them. The
state-derived substep count (§6.2) becomes a `when()` predicate on the trailing substeps' nodes, so
varying it does **not** recompose the graph: the nodes exist for the maximum substep count and the
surplus ones are disabled that tick.

Two consequences worth stating plainly:

- Anything the host must see *between* substeps has to move onto the device, because there is no cheap
  point to look. That is a feature: it forces the design that was correct anyway.
- The graph is composed per **structure**, and structure changes are budgeted and reported through
  `compose_count()`. A `compose_count()` that climbs every tick is a bug, and a test asserts it does
  not — the same way `XpbdSolver::compile_count()` is already asserted in the existing tests.

#### Runtime configuration the physics scene must set

Three settings, each with a reason:

- **`Runtime::rebalancer(false)`.** The thermal rebalancer is a background thread on a ~5 ms
  heartbeat that migrates tasks mid-run. The runtime's architecture doc names the case exactly:
  *"Disable it for a low-jitter real-time frame or a deterministic/replay run."* For a 60 Hz physics
  tick the reason is **jitter**, not correctness — the runtime's WP-4 analysis is that work stealing
  and migration do not change results as long as no accumulation depends on schedule order (below).
  Off for physics, either way.
- **Profiling on only when asked.** `RunReport::node_timings` / `worker_timings` are populated only
  when the runtime was created with `RuntimeConfig::profiling`; with it off, the dispatch hot path
  reads no timestamps. `PhysicsStatistics` (§13.3) reports what it can either way and the per-node
  breakdown appears when the profiler panel is open.
- **One `DeviceIndex` for the whole scene**, per the co-location constraint above.

#### Deterministic reductions: the runtime provides them

The runtime's WP-4 is explicit that on a single architecture determinism reduces to *"making every
order-sensitive reduction use a fixed combination order"*, and equally explicit about the hard case:
*"GPU reductions are the hard case — a device `reduce` is not guaranteed fixed-order. Deterministic
state-affecting reductions on GPU must use an explicit fixed-order kernel."*

This plan was written when that primitive did not exist and assumed the physics layer would build it.
It now exists (§18, R2): `Graph::add_reduce` and `Graph::add_segmented_reduce`, guaranteeing that the
combination order is a function of the element layout alone — not of the worker count, the device, the
steal pattern, or the work-group size. The segmented form is precisely the shape §12.2 needs: one
work-item folds one segment left to right, which is per-body contact-impulse accumulation and
per-vertex accumulation across the tetrahedra sharing a vertex.

So **§12.2's fixed-order accumulation is a call, not a deliverable**, and it leaves P0's scope. What
does not leave is the *conformance requirement*: the byte-equality test with the worker count varied
(§15.5) still has to pass, because a fixed-order primitive used through an order-sensitive call site
is still an order-sensitive result. The one thing to watch is load balance — a segment with ten
thousand contributions and a segment with one cost one work-item each, which is the price of the
guarantee and is the caller's layout problem, i.e. ours.

#### Cooking on the runtime, optionally

The cooker is host code, but its two heaviest stages — voxelization and signed-distance-field baking —
are embarrassingly parallel grid problems. Since SushiRuntime resolves a CPU/OpenCL backend when no
GPU toolchain is present (`SR_GPU_BACKEND=auto` falls through to `cpu`), the cooker may dispatch those
two stages through the runtime when one is available, behind the same `ICookingStage` seam, with a
host implementation as the reference. This is an optimization with a conformance test (§4.4), not a
dependency: **cooking must work with no device present**, because an importer that needs a GPU is an
importer that fails on a build machine.

#### Keeping the seam thin — and honouring a decision already taken

The runtime's API is explicitly unstable and this plan puts weight on it, so: **exactly one adapter
names `SushiRuntime::`.** A `Physics::RuntimeGraphBuilder` behind `IConstraintSolver` (§3.3) owns
every `Buffer`, `Graph`, `DynamicGraph`, `Dynamic`, `Reads`/`Writes`, and `Extent` in the physics
layer. No constraint, no shape, no cooker, and nothing in `physics/scene` includes a runtime header.
When the runtime API moves, one file moves with it, and the conformance suite proves the behaviour
did not.

This is not a new preference. The runtime's substrate plan records it as a locked owner decision —
**L12**: *"Claude builds the domain layers (ECS + physics) as separate, zero-coupling libraries (no
runtime header in them)."* Today's physics layer does the opposite: `physics_world.hpp`,
`xpbd_solver.hpp`, and `sim/physics_simulation.hpp` all include `SushiRuntime.h` directly, and
`XpbdSolver` holds `Buffer` and `Graph` members in its own class body. So the adapter is not an
improvement someone thought of — it is a **restoration** of a decision the codebase drifted from,
and P0 is where that drift is paid back.

---

## §7 Collision and the penetration contract

This section is the direct answer to *"penetrasyon çok önemli — rigid ve soft farketmeksizin, hem mesh
hem fiziksel olarak."*

### 7.1 Broadphase

`SweepAndPruneBroadphase` (today's, fixed: incremental, three-axis, no per-call allocation) is kept as
the reference implementation and the small-scene fast path. The production path is
`BoundingVolumeHierarchyBroadphase`: a dynamic bounding-volume hierarchy with

- **fat bounds** (an inflation proportional to velocity), so a body only re-inserts when it leaves its
  fattened box — most bodies do nothing most ticks,
- **refit on move, rebalance on budget**, never a full rebuild,
- a **persistent pair cache** producing `added` / `persisted` / `removed` pair streams, which is what
  lets manifolds and their warm-start impulses survive across ticks,
- **swept bounds** for bodies flagged for continuous collision, so the pair exists before the impact.

Static geometry lives in a separate, never-rebuilt tree.

### 7.2 Narrowphase

- **Analytic fast paths** for the common pairs: sphere-sphere, sphere-capsule, capsule-capsule,
  sphere-box, box-box (the existing separating-axis test, kept and given a real manifold).
- **General convex-convex** by GJK with EPA for the penetrating case, on shapes inflated by their
  convex radius. This one routine covers hull-hull, hull-box, hull-capsule, and every future convex
  shape — the Open/Closed payoff.
- **Convex vs triangle mesh**: query the mesh's bounding-volume hierarchy with the convex shape's
  swept bounds, then run the convex routine per candidate triangle, with **adjacent-face normal
  correction** so a shape sliding across a tessellated floor does not catch on internal edges. (The
  classic "ghost collision" bug; the fix is to reject contact normals that point into an adjacent
  triangle's Voronoi region.)
- **Signed-distance field vs anything**: sample the field at the query point, take the gradient as the
  normal. `O(1)`, exact to the field's resolution, and the *right* answer for deep penetration where
  hull-based methods degrade. This is the primary path for soft-body vertices against rigid bodies and
  for concave static geometry.
- **Height field** by direct cell lookup.

Dispatch is the table from §4.2. Every routine returns the same `ContactManifold`.

### 7.3 Manifolds

```cpp
struct ContactPoint
{
    Vector3 anchor_a_local, anchor_b_local;   // in each body's frame — stable under rotation
    Scalar  separation;                       // negative = penetrating
    Scalar  normal_lambda, tangent_lambda[2]; // warm-start accumulators
    std::uint32_t feature_id;                 // the pair of features that produced it
};
struct ContactManifold
{
    BodyHandle a, b;
    Vector3 normal;
    ContactPoint points[4];
    std::uint8_t point_count;
    std::uint16_t material_a, material_b;
};
```

Generated by **face clipping** (Sutherland–Hodgman against the reference face's side planes) and
reduced to at most four points by maximizing the enclosed area — the standard reduction, and the fix
for the rocking box in §1.2 item 3. Points are matched to the previous tick's by `feature_id`, so their
`lambda` values carry over: **warm starting**, which is what makes a stack of ten crates converge in
one iteration per substep instead of never.

### 7.4 Friction and restitution

Per Müller et al. 2020, and this is the recipe the implementation follows exactly:

- **Positional pass, per contact point:** apply the normal correction Δλ_n; then compute the tangential
  relative displacement of the contact anchors since the substep began and apply a tangential
  correction, **clamped to the static-friction cone** `λ_t ≤ μ_static λ_n`. Static friction that is
  positional is what lets a box sit still on a ramp instead of creeping.
- **Velocity pass, per contact point:** dynamic friction
  `Δv = -v̂_tangent · min(μ_dynamic |λ_n| / h, |v_tangent|)`, then restitution
  `Δv = n · (-v_normal + max(-e · v_normal_before_solve, 0))`, with restitution **suppressed** when
  `|v_normal| < 2 g h` — the standard anti-jitter threshold that stops resting bodies from buzzing.
- Materials combine by their declared combine mode (§5.3).

### 7.5 Continuous collision — nothing tunnels

Three tiers, cheapest first, chosen per body by its flags and motion:

1. **Speculative contacts (default, always on).** Generate contacts out to `contact_offset + |v·n|h`
   with a *positive* target separation. The solver treats an approaching body as constrained not to
   pass the surface, so the vast majority of fast motion never needs a sweep at all. Cost: near zero.
2. **Conservative advancement** for bodies whose per-substep motion exceeds a fraction of their
   thinnest dimension: iteratively advance to the earliest time of impact using the closest-distance
   query, then solve at that time. Cost: a few closest-point queries for the few bodies that need it.
3. **Substep escalation** for the pathological case (a bullet, a wheel at speed): the island's substep
   count is raised locally. Deterministic, because the trigger is a state-derived threshold.

Soft bodies get the continuous treatment differently (§9.6): vertex-triangle and edge-edge continuous
tests with a thickness, per Bridson.

### 7.6 The visible-penetration contract, made concrete

Two per-shape distances, PhysX's model, and the reason it is in this plan explicitly:

- **`contact_offset`** — contacts are *generated* at this separation. Larger means earlier detection
  and more stable stacking. Default: a small fraction of the shape's size.
- **`rest_offset`** — contacts are *resolved* to this separation. Zero means surfaces come to rest
  exactly touching. A small positive value keeps a visible sliver of air; a small negative value lets
  a tyre visibly deform into the ground.

The **enforced invariant**, checked by a regression test rather than asserted in prose: in the standard
stacking, ramp, and vehicle-landing scenes, the maximum measured penetration of any contact at rest
does not exceed `rest_offset + tolerance`, and no contact's penetration exceeds a hard *depenetration
budget* mid-motion — a maximum depenetration velocity clamps recovery so a deeply-overlapping spawn
pushes apart smoothly instead of exploding.

For **mesh accuracy**, the cooker reports the one-sided Hausdorff distance between the render mesh and
its cooked collision geometry, in metres, in the inspector. "The collider is 3 cm fatter than the
visible mesh along the wheel arch" becomes a number the artist can see and fix by raising the convex
piece budget — instead of a mystery gap at runtime.

For **soft bodies**, §0.4's second mechanism removes the problem at the root: there is no second
geometry. The render mesh is the embedded interpolation of the simulated mesh, and the simulated
surface is what collided.

### 7.7 Queries

`ICollisionQueryService`: `raycast_closest`, `raycast_all`, `sweep`, `overlap`, `closest_point`, each
taking a filter (layer mask plus an optional predicate). Built on the same broadphase tree, so a query
costs a tree descent, not a scan. Triggers are shapes flagged `Trigger` that generate events and no
impulses.

---

## §8 The cooking pipeline — a mesh goes in, a simulation asset comes out

This is the user's headline feature: *"mesh'i projeye attıktan sonra otomatik olarak soft body optimized
bake eden bir motor, ve bunun hassasiyetini ayarlayabiliyoruz."*

### 8.1 The import path

```
  a mesh file lands in the project
            │
            ▼
  MeshImporter  ──▶  Geometry::TriangleMesh  ──▶  render upload (existing path)
            │
            ├──▶ IMeshPostProcessor chain (new; ordered, registered, Open/Closed)
            │        ├─ CollisionCooker    → .sushicollision   (if the import profile asks)
            │        ├─ SoftBodyCooker     → .sushisoft        (if the import profile asks)
            │        └─ NodeBeamCooker     → .sushinodebeam    (opt-in, §11)
            │
            ▼
  ICookedAssetStore, keyed by (source content hash, cooker version, parameters hash)
```

Cooking runs **off the main thread**, reports progress, and is fully cached: an unchanged mesh with
unchanged parameters is never re-cooked. The import profile (a per-project default plus a per-asset
override) decides which cookers run — this is how "drop a mesh in and it is soft-body ready" happens
without every rock in the level paying for a tetrahedral mesh.

### 8.2 The fidelity dial

One authored number, `fidelity ∈ [0, 1]`, drives every internal resolution through a documented
mapping. The artist turns one knob; the engineer can still override any individual field.

| Derived parameter | At fidelity 0 (fastest) | At fidelity 1 (most accurate) |
|---|---|---|
| Voxel resolution (longest axis) | 16 | 256 |
| Target tetrahedron count | ~200 | ~120 000 |
| Simulation levels of detail | 1 | 4 |
| Convex pieces (rigid) | 4 | 64 |
| Distance-field resolution | 16³ | 128³ |
| Surface-conforming passes | 0 (voxel-blocky) | 3 (marching-tetrahedra fit + smoothing) |
| Suggested substep count | 8 | 32 |

The inspector shows what the dial *produced* — tetrahedron count, cook time, estimated per-tick cost,
memory, Hausdorff error — so the trade-off is visible rather than felt three weeks later.

### 8.3 The soft-body cooker, stage by stage

Each stage is an `ICookingStage`; the pipeline is a list.

1. **Repair.** Weld duplicate vertices, drop degenerate triangles, orient consistently, compute a
   connected-component report. Report — do not silently fix — a non-manifold or non-watertight input,
   because the artist needs to know.
2. **Voxelize.** Rasterize the surface into a grid at the fidelity-derived resolution, then flood-fill
   the exterior from a corner. Everything not reached is interior. This is robust to the meshes real
   projects actually contain: self-intersecting, non-manifold, open-shelled. **Choosing voxelization
   over direct constrained Delaunay tetrahedralization is deliberate** — Delaunay is more accurate on
   clean input and fails on dirty input, and dirty input is the common case.
3. **Tetrahedralize.** Fill the interior with a **body-centred cubic lattice** (each cell yields
   tetrahedra that tile without slivers), then conform the boundary: cells straddling the surface are
   split by marching tetrahedra against the surface's distance field, and boundary vertices are snapped
   onto the surface within a tolerance. This is the Houdini/Blender approach — predictable element
   quality, resolution driven by one number, no failure mode that produces zero output.
4. **Optimize.** Remove tetrahedra below a minimum-volume or minimum-dihedral-angle threshold
   (slivers destroy the conditioning of a finite-element solve), smooth interior vertices toward their
   neighbourhood centroid, and re-check that no element is inverted. Report the worst element quality.
5. **Compute rest state.** Per tetrahedron: the inverse rest-shape matrix `Dm⁻¹`, the rest volume, and
   the mass distributed to its four vertices from the material density. Per vertex: the summed inverse
   mass. Also compute the whole body's mass, centre of mass, and inertia tensor (for the rigid
   approximation the level-of-detail system falls back to).
6. **Embed the render mesh.** For every render-mesh vertex, find the containing tetrahedron and store
   its index plus four barycentric weights. Vertices outside every tetrahedron (thin features that
   fell through the lattice) bind to the nearest tetrahedron by the *extrapolated* barycentric
   coordinates, which keeps them attached and moving correctly. This table **is** §0.4's guarantee.
7. **Build the surface set.** Extract the tetrahedral mesh's boundary triangles — this is the soft
   body's collision surface — and build its bounding-volume hierarchy.
8. **Bake the rest distance field.** A narrow-band signed-distance field of the rest shape, used for
   fast "is this point inside me" queries during collision and for the self-collision broad pass.
9. **Build levels of detail.** Coarser lattices at halved resolution, plus a barycentric mapping from
   each level to the next finer one, so a distant body simulates 500 tetrahedra and the render mesh is
   still driven correctly through the chain. The coarsest level is a shape-matching cluster set; below
   that, the body falls back to its rigid approximation.
10. **Serialize.** One `.sushisoft` blob, with the cooking parameters recorded inside it so a re-cook
    is reproducible and a mismatch is detectable.

### 8.4 The collision cooker (rigid)

1. **Mass properties** by exact polyhedral integration over the closed mesh (Mirtich's method) — mass,
   centre of mass, full inertia tensor, then an eigendecomposition to principal axes so the runtime's
   diagonal `inverse_inertia` is *correct* rather than authored by hand (§1.2 item 6).
2. **Approximate convex decomposition** into at most `N(fidelity)` pieces, each simplified to a vertex
   budget, with the volumetric error and the Hausdorff error reported.
3. **A narrow-band distance field** of the whole shape, for deep-penetration recovery and soft-vertex
   queries.
4. For geometry authored as **static**, skip decomposition: cook a triangle-mesh bounding-volume
   hierarchy instead, which is both cheaper and exact.
5. **Serialize** to `.sushicollision`.

### 8.5 Validation, because a silent bad cook is the worst outcome

The cooker emits a `CookingReport` — element count and worst quality, watertightness, inverted
elements, unembedded render vertices, Hausdorff error, estimated per-tick cost, memory. The editor
shows it, and a cook that violates a configured threshold **fails loudly** rather than shipping a body
that explodes on the first frame.

### 8.6 The mesh binding, end to end

The connection between a mesh and its physics is the thing this whole system is built to get right, so
it is written out as one continuous path rather than left implied across four sections. Today the
engine has **no** link at all (§1.2 item 14): the only triangle mesh lives behind Vulkan and the
physics has never seen it.

```
  IMPORT (host, once)                    RUNTIME (per tick)                    RENDER (per frame)
  ───────────────────────                ──────────────────                    ──────────────────
  mesh file
    │
    ├─▶ Geometry::TriangleMesh  ────────────────────────────────────────────▶  MeshRegistry (existing)
    │        │                                                                        ▲
    │        │  CollisionCooker                                                       │
    │        ├─▶ convex pieces ────────┐                                              │
    │        ├─▶ inertia tensor ───────┤                                              │
    │        └─▶ signed-distance field ┤                                              │
    │                                  ├─▶ rigid body  ──▶ one pose ──────────────────┤
    │                                  │                   (Transform + Orientation)  │
    │        SoftBodyCooker            │                                              │
    ├─▶ tetrahedral mesh ──────────────┤                                              │
    ├─▶ surface triangles + hierarchy ─┤                                              │
    ├─▶ EMBEDDING TABLE ───────────────┼─▶ soft body ──▶ per-vertex positions ────────┤
    │     (render vertex → tet + 4     │                  (deformed vertex buffer)    │
    │      barycentric weights)        │                                              │
    └─▶ level-of-detail lattices ──────┘                                              │
                                                                                      │
           NodeBeamCooker: node cloud + skinning weights ──▶ vehicle ──▶ skinned ──────┘
```

**Three binding strategies, one chosen per part by the cooker:**

| Part is | Binding | Per-tick cost | Where the render vertex comes from |
|---|---|---|---|
| Rigid | One transform | Nothing | The body's pose; the mesh is drawn as-is, exactly as today. |
| FEM soft | **Barycentric tetrahedral embedding** | One 4-way weighted sum per render vertex | `Σ weight[i] · tetrahedron_vertex[i]`. Smooth, exact, and continuous across element boundaries. |
| Node-beam | Distance-weighted node skinning | One K-way weighted sum per render vertex | The same shape as skeletal skinning, with nodes in place of joints — so it reuses the skinning path the renderer already has. |

**The deformed-vertex path to the renderer.** Cloth already proves this route end to end:
`Simulation::ClothInstance` + a shared `cloth_vertices` array → `Render::ClothStrandView` →
`render/geometry/cloth_buffers.cpp` (`ARCHITECTURE.md` §4.2). A soft body is the same channel with a
triangle topology that comes from the asset instead of from a grid, so the plan **generalizes
`ClothStrandView` into a `DeformableMeshView`** rather than adding a third parallel path. Cloth becomes
one kind of deformable mesh, not a special case — the same consolidation §4.2 asks for everywhere else.

The vertex deformation itself (applying the embedding) is a per-vertex parallel kernel and runs in the
same runtime graph as the solve (§6.6), so the deformed positions are produced device-side and handed
to the renderer without a host round trip.

**Normals** are recomputed from the deformed positions — per-face, then area-weighted per-vertex — in
the same kernel, because a dented panel that keeps its rest normals looks undented no matter how
accurate the simulation is.

**The invariants, each with a test (§15):**

1. Every render vertex is bound. Unbound vertices are counted at cook time and the count is reported;
   a non-zero count above a threshold fails the cook (§8.5).
2. At rest, the reconstructed render mesh reproduces the source mesh within a stated tolerance —
   the embedding round-trips.
3. A deformation applied to the simulated mesh is visible in the render mesh in the *same* tick; there
   is no lag, because there is no synchronization step to lag.
4. Fracture (§9.5) preserves binding: a duplicated simulation vertex inherits its parent's binding, so
   a crack does not tear a hole in the render mesh.
5. Collision happens against the simulated surface, so §0.4's contract holds by construction rather
   than by convention.

---

## §9 Soft bodies

### 9.1 The model

Two XPBD constraints per tetrahedron (Macklin & Müller 2021), replacing today's distance lattice:

- **Deviatoric** (resists shape change): `C_deviatoric = ‖F‖_Frobenius − √3`, compliance `1/(μ·V_rest)`.
- **Hydrostatic** (resists volume change): `C_hydrostatic = det(F) − 1 − μ/λ`, compliance `1/(λ·V_rest)`.

where `F = Ds · Dm⁻¹` is the deformation gradient and `μ`, `λ` are the Lamé parameters derived from
Young's modulus and Poisson's ratio. This is a stable neo-Hookean solid: it resists inversion, it
preserves volume, it has real material parameters, and it is two constraint projections — the same
shape as the distance projection already in the codebase.

Cloth keeps its distance + bending formulation (a dihedral-angle bending constraint is added, which
today's cloth lacks — §1.2 item 13's two-dimensional cousin). Ropes are distance chains with a
bending constraint. Same solver, three topologies.

### 9.2 Material parameters — the artist-facing surface

```cpp
struct SoftBodyMaterial
{
    Scalar young_modulus;       // Pa — stiffness. Rubber ~1e7, car steel panel ~2e11
    Scalar poisson_ratio;       // [0, 0.5) — how much it bulges when squeezed
    Scalar density;             // kg/m³
    Scalar damping;             // velocity damping toward the rigid-body mode
    Scalar yield_stress;        // Pa — above this, deformation becomes permanent (§9.4)
    Scalar plastic_creep;       // [0, 1] — how fast permanent deformation accumulates
    Scalar maximum_plastic_strain;
    Scalar fracture_stress;     // Pa — above this, the element separates (§9.5)
};
```

Presets ship for rubber, foam, soft tissue, sheet steel, and aluminium, so an artist starts from a
material name rather than from a number they have to guess.

### 9.3 Strength — the *mukavemet* readout

Every element's **Green strain tensor** and **Cauchy stress** fall out of the deformation gradient the
solver already computed; the **von Mises equivalent stress** is one scalar per tetrahedron. It costs
almost nothing because `F` is already in hand, and it delivers:

- a **stress heat map** debug view (blue → red over the body) — the thing that makes "is this beam
  strong enough" answerable rather than guessable,
- the trigger for plasticity (§9.4) and fracture (§9.5),
- a gameplay query: `ISoftBodyService::maximum_stress(entity)` and per-region aggregates, so a game can
  ask "how badly is this damaged."

For **rigid** assemblies, the analogue is the joint-force readout in §10.4 — the same question, asked
of a constraint instead of an element.

### 9.4 Plasticity — the permanent dent

Multiplicative decomposition: the total deformation `F` splits into an elastic part and a plastic part,
and the cooked rest matrix `Dm⁻¹` is what stores the plastic part. Each substep, per element:

```
  if (elastic strain measure > yield_stress-derived threshold)
      advance the plastic deformation toward the current shape by plastic_creep
      clamp the accumulated plastic strain to maximum_plastic_strain
      write back the updated rest matrix and rest volume
```

Because the plastic state lives in the rest configuration, a dented panel *stays* dented, springs back
elastically around its new shape, and needs no separate damage system. This is the mechanism behind
car-body deformation in §11, and it is a dozen lines inside the element projection.

### 9.5 Fracture

When an element's von Mises stress exceeds `fracture_stress`, the element is removed and its shared
vertices are duplicated along the crack surface, splitting the topology. This requires the mutable
world of §6.4 — constraints removed, vertices added, colouring updated incrementally — and it requires
the render embedding to follow, which it does, because embedding is per-vertex and a duplicated vertex
inherits its parent's binding.

Guard rails: a per-tick fracture budget (deterministic, state-derived), a minimum fragment size, and a
scene-level cap. Uncapped fracture is how a physics engine dies.

### 9.6 Soft-body collision — three problems, three answers

1. **Soft vs rigid.** Surface vertices query the rigid body's cooked signed-distance field: `O(1)` per
   vertex, exact depth, exact gradient normal, and correct at any penetration depth. This is *the*
   reason §8.4 bakes a distance field for every rigid collider. Two-way: the correction's reaction is
   applied to the rigid body weighted by generalized inverse mass, so a soft body actually pushes back.
2. **Soft vs soft.** Broad: each body's bounding-volume hierarchy against the other's. Narrow:
   vertex-triangle and edge-edge with a thickness, continuous (Bridson) for thin bodies, discrete for
   thick ones.
3. **Self-collision.** A spatial hash over the surface vertices at a cell size of the collision
   thickness, excluding topological neighbours, with the continuous vertex-triangle and edge-edge tests.
   Off by default (it is the expensive one), enabled per body, scheduled in the phase where it can be
   measured rather than assumed.

### 9.7 Levels of detail

Cooked in §8.3 step 9, selected at runtime by screen coverage and distance, with hysteresis so a body
does not oscillate between levels. The transition maps the fine state onto the coarse one through the
stored barycentric mapping, so a body does not pop. The coarsest tier is shape matching; below that, a
rigid body with the cooked inertia tensor. `ISoftBodyModel` (§3.3) is the seam that makes the swap a
substitution rather than a special case.

---

## §10 Assemblies and multibody dynamics

The direct answer to *"otomobil şasisi üzerine menteşe ile bağlanmış kapı, mukavemet ve MBD hesapları."*

### 10.1 The joint library

Every joint is a POD descriptor plus a projection functor, registered per §4.2. Each holds two
**joint frames** — a position and orientation in each body's local space — which is the uniform way to
express "where and how these two bodies are attached."

| Joint | Removes | Authored parameters |
|---|---|---|
| `FixedJoint` | 6 degrees of freedom | Compliance (a "welded but it can flex" seam is just a compliant fixed joint). |
| `BallJoint` | 3 translational | Swing and twist limits, cone angle. |
| `HingeJoint` | 3 translational + 2 rotational | Axis, lower/upper angle limits, motor, friction. **The car door.** |
| `SliderJoint` | 3 rotational + 2 translational | Axis, travel limits, motor. **Suspension travel.** |
| `DistanceJoint` | Range along a line | Minimum/maximum, compliance. Already exists, generalized. |
| `ConeTwistJoint` | 3 translational, limits rotation | Swing cone + twist range. **Ragdolls.** |
| `GearJoint` / `RackJoint` | Couples two rotations | Ratio. **Differentials, steering rack.** |
| `SixDegreeOfFreedomJoint` | Configurable | Per-axis free/limited/locked, with per-axis drives. The general case. |

**Limits** are inequality constraints projected only when violated. **Motors** are target-position or
target-velocity drives with a maximum force, expressed as compliant constraints so they inherit XPBD's
step-size independence.

### 10.2 The assembly asset

`PhysicsAssembly` (§5.4) is authored in the editor and instanced atomically: parts become bodies,
joints become constraints, collision-filter groups stop the door from colliding with the chassis it is
attached to. One entity carries the `AssemblyInstance`; child entities carry the parts, so the
hierarchy the editor already supports (`set_parent`, `move_entity`) is the authoring surface.

The car-door scenario end to end:

```
  chassis.fbx  →  CollisionCooker  →  chassis.sushicollision   (12 convex pieces, inertia, distance field)
  door.fbx     →  CollisionCooker  →  door.sushicollision      (3 convex pieces, inertia, distance field)
                                            │
  car.sushiassembly:                        ▼
     part[0] = chassis, mass 900 kg, material "car_steel"
     part[1] = door,    mass  35 kg, material "car_steel"
     joint[0] = Hinge(part 0 ↔ part 1)
                  axis = door local +Y
                  limits = [0°, 68°]
                  friction = 4 N·m          (it does not swing free)
                  break_force = 12 kN       (a hard enough impact tears it off)
     filter: part 0 and part 1 do not collide with each other
```

Break the joint at runtime — the door becomes a free rigid body, the constraint is removed from the
mutable world (§6.4), and a `JointBroken` event fires. Swap `part[1]`'s collision asset for a
`SoftBodyAsset` and the door dents instead of merely swinging (§9), with no change to the joint.

### 10.3 Two-way coupling between body kinds

A joint's endpoint may be a rigid body **or** a soft-body vertex set (an "attachment", which averages
the correction across a small vertex neighbourhood so it does not tear a single vertex out of the
mesh). This is what mounts a rigid hinge to a deformable panel — the exact case a real car door is —
and it is the same generalized-inverse-mass split the engine already performs.

### 10.4 Multibody quantities — reading forces back out

XPBD gives Lagrange multipliers, not forces. The recovery is exact and cheap:

```
  force  = λ · direction / h²          torque = λ_angular · axis / h²
```

accumulated per joint per substep and exposed as `IJointService::joint_force(handle)` /
`joint_torque(handle)`. That single quantity delivers: break thresholds, the "how much load is this
mount carrying" readout that is the rigid-body half of *mukavemet*, motor-effort feedback for a
drivetrain, and a diagnostic overlay showing which joint in an assembly is the one about to fail.

### 10.5 When XPBD is not enough

A drivetrain has mass ratios in the thousands (a crankshaft against a vehicle) and joints that must be
exactly rigid. Substepping handles a great deal of it, but not all. Two escape hatches, in order of
preference:

1. **Solve the stiff sub-chain in one dimension.** A powertrain is a chain of rotational inertias, not
   a spatial mechanism. Simulating it as an independent one-dimensional multibody system (§11.4) and
   coupling it to the wheels through a torque constraint is both cheaper and more accurate than
   forcing it through the three-dimensional solver.
2. **A reduced-coordinate articulation path** (Featherstone) behind `IConstraintSolver`, for chains
   that genuinely need it. Kept as a documented option, not a phase-one commitment, because it doubles
   the solver surface and every AAA engine that shipped both regrets the maintenance.

---

## §11 Vehicles — the BeamNG evolution

The user's stated destination. This phase is late in the roadmap because it composes everything before
it; nothing here is new physics, it is new *assembly*.

### 11.1 The node-beam model, in XPBD terms

| BeamNG concept | SushiEngine expression |
|---|---|
| Node | A particle: a `RigidBodyT` with zero inverse inertia, a collision radius, a material, and drag. Exactly what cloth and the current soft-body lattice already use. |
| Beam | An `XpbdDistanceConstraint` with a compliance from its spring rate, a damping term, a **deform threshold**, and a **break threshold**. |
| `beamDeform` | Plasticity on a distance constraint: when the recovered force exceeds the threshold, the rest length advances toward the current length by a creep rate. §9.4's mechanism, applied to one dimension. |
| `beamStrength` | The break threshold: exceed it and the constraint is removed from the mutable world (§6.4), permanently. |
| Bracing / support beams | Additional constraints in the cooked topology that give the structure torsional and shear rigidity a pure edge network lacks. |
| Collision triangles | The node cloud's surface triangle set, colliding against rigid bodies through their distance fields (§9.6) and against terrain through the height field. |
| 2 000 Hz physics rate | A 60 Hz tick at 32 substeps ≈ 1 920 Hz effective (§0.2). The rate arrives by construction. |

### 11.2 The structure: a rigid core with a deformable shell — and the BeamNG mistakes it avoids

**Decided (§17.3): the hybrid is the default.** A pure node-beam vehicle is beautiful and expensive:
every gram of the car is in the solver, and handling quality depends on beam tuning that takes months.
The structure is:

- The **chassis core** is a rigid body carrying the bulk of the mass and the inertia tensor. Handling is
  stable, tunable, and cheap.
- The **shell** (panels, bumpers, doors, bonnet) is node-beam or FEM soft, attached to the core by the
  §10.3 attachment constraint. It deforms, dents permanently, and tears off.
- **Suspension** is joints and drives (§10.1), not beams — a slider joint with a spring-damper drive is
  more controllable and more stable than a beam network, and it is what every shipping racing game does.
- **Wheels** are rigid bodies with a tyre force model (§11.5).

The pure node-beam path stays open, because `VehicleInstance` names a `NodeBeamAsset` and an asset
whose rigid core is empty *is* a pure node-beam vehicle. **The architecture does not choose; the asset
does.**

#### What BeamNG got wrong, and what we do instead

BeamNG is the reference for what a deformable vehicle should *feel* like. It is also a decade-old
design with known structural limits, and the point of arriving later is to not inherit them. Each row
is a real limitation of that architecture and the specific mechanism in this plan that avoids it.

| BeamNG limitation | Why it happens | What SushiEngine does instead |
|---|---|---|
| **Authoring is thousands of hand-tuned numbers.** A vehicle is a large hand-written node/beam description; every beam's stiffness, damping, deform, and break value is authored by a human. | There is no cooker: the node-beam network *is* the source asset. | §11.3's cooker derives the topology from the mesh at a fidelity setting, and derives beam properties from a **`SoftBodyMaterial`** (§9.2) — "sheet steel" is a material with a Young's modulus and a yield stress, not four thousand tuned constants. Hand-authoring stays possible; it stops being mandatory. |
| **Deformation is not physical.** `beamDeform`/`beamStrength` are per-beam magic numbers with no relationship to a material's actual yield stress, so a panel's behaviour is only as right as someone's guess. | The model is a spring network, not a continuum. | Deformation comes from the **FEM plasticity model** (§9.4) driven by yield stress and creep rate, with a von Mises stress readout (§9.3) that can be checked against a cantilever test (§15.3). The deformation is derivable, not tuned. |
| **Explicit spring-damper integration caps stiffness.** Beams must stay soft enough to be stable at the timestep; a genuinely rigid structure needs an ever-higher rate, which is a large part of why the whole simulation runs at 2 kHz. | Explicit integration's stability depends on stiffness × timestep. | **XPBD is unconditionally stable and step-size independent** (§0.2). A rigid beam is compliance zero and costs the same as a soft one. The high effective rate (§11.1) is chosen for collision fidelity, not bought to keep springs from exploding. |
| **The whole vehicle pays the maximum rate, always.** A parked car costs what a crashing one costs. | One global physics rate, no level of detail. | State-derived substep counts (§6.2), **per-island substepping** (§13.2), sleeping (§13.2), and soft-body levels of detail (§9.7) — a distant vehicle's shell collapses to its rigid core and the island's region is dropped from the `DynamicGraph` (§6.6). |
| **Collision is point-sampled at the nodes.** Fidelity is bounded by node density; nodes can slip past thin geometry and catch on internal edges. | Node-versus-collision-triangle testing, no continuous test, no distance field. | **Signed-distance-field contact** (§7.2, §9.6) gives exact depth and gradient normal at any penetration depth; surface triangles collide with thickness; continuous collision (§7.5) means nothing tunnels; adjacent-face normal correction removes edge catching. |
| **The collision shape is not the visible shape.** Simplified collision triangles diverge from the render mesh, so what you see is not quite what hit. | Collision geometry is authored separately. | §0.4's contract: the render mesh is **embedded** in the simulated mesh, and the cooker reports the Hausdorff error for any approximation (§7.6). Divergence becomes a measured number instead of an unstated assumption. |
| **CPU-bound and effectively one vehicle per core.** Vehicle counts scale with cores, which is why large multiplayer scenes are hard. | The solver is a CPU spring network. | The entire solve is a **SushiRuntime graph** (§6.6): graph-coloured, device-resident, many vehicles in one composition. Vehicle count scales with device width, not core count. |
| **Not built for rollback netcode.** Multiplayer arrived late and carries sync artifacts, because determinism was not a design constraint. | Determinism was not designed in from the start. | Determinism is a **build-time property** (§0.5, §12), with byte-equal replay and snapshot-rollback tests (§15.5) from P0 — not a later retrofit. |
| **The render mesh is skinned to nodes with limited fidelity**, so fine deformation detail is bounded by node count. | One binding strategy, node-weighted. | Two bindings, chosen per part by the cooker: **barycentric tetrahedral embedding** for FEM parts (smooth, exact, §8.3 step 6) and distance-weighted node skinning for node-beam parts — with unembedded-vertex counts reported (§8.5). |

The honest counterpoint: BeamNG's pure node-beam model produces emergent behaviour a hybrid cannot —
a chassis that twists under load, a structure whose failure mode was never authored. The hybrid keeps
that reachable (an empty rigid core), and §17.4 records the open question of the core-to-shell seam,
which is where visual artifacts will show up first.

### 11.3 The node-beam cooker

`NodeBeamCooker` turns a vehicle mesh plus an authored structure into a `NodeBeamAsset`: place nodes at
mesh features or on a lattice at the fidelity-derived resolution, connect them with structural beams,
add bracing beams by a diagonal rule, distribute mass by density, and skin the render mesh onto the
node cloud with distance-weighted blending. Manual authoring stays possible — the asset is data, and a
hand-authored node-beam file is as valid as a cooked one.

### 11.4 Powertrain

A one-dimensional multibody chain, simulated inside the physics tick at the same substep rate: engine
torque curve and inertia → clutch (a torque-limited coupling) → gearbox (a ratio) → differential (a
`GearJoint`-style constraint splitting torque between outputs) → axle → wheel angular velocity. The
coupling to the three-dimensional world is a torque applied to each wheel body and the reaction on the
chassis. §10.5's first escape hatch, applied.

### 11.5 Tyres

A slip-based force model (a Pacejka-style magic-formula curve, or a simpler Brush model as the first
implementation) evaluated per wheel per substep: compute longitudinal and lateral slip from the wheel's
contact patch velocity, look up the force curve, apply the force at the contact point. Combined-slip
handling by the friction ellipse. Load sensitivity from the contact normal force the solver already
recovered.

The alternative — a node-beam tyre with pressurized volume, which is what BeamNG actually does — is a
soft body with a volume constraint over its interior surface (§9.1's hydrostatic constraint, applied to
a closed shell). It is genuinely better and genuinely much more expensive. Both are supported by the
same asset structure; the vehicle asset chooses.

### 11.6 Aerodynamics — a cross-system tie-in

The engine already has a full atmosphere and wind system (`sim/weather_wind.hpp`,
`sim/atmosphere_forcing_buffer.hpp`, `docs/slop/atmosphere_system.md`). Vehicle drag and downforce, and
cloth and rope wind response, sample it through a `WindSampler` seam that mirrors the existing
`GravitySampler` exactly (§4.5) — the physics names the abstraction, never the meteorology behind it.
A flag on a pole in a storm and a car's high-speed lift come from the same field.

---

## §12 Determinism, precision, and networking

### 12.1 The rules, restated as physics requirements

1. **Body order is stable.** Bodies are iterated by handle index, never by hash-container order.
   Where the boundary layer keeps an `std::unordered_map<EntityId, BodyHandle>` (as it does today),
   any iteration whose *order is observable* is replaced by a sorted index vector.
2. **Contacts are sorted before they are solved**, by `(body_a, body_b, feature_id)`. Broadphase output
   order is an implementation detail and must never reach the solver.
3. **Colour order is fixed** and the incremental recolouring is deterministic — the greedy rule already
   is; the incremental version must produce the same colouring as a full recolour would, or explicitly
   record its divergence in the snapshot.
4. **Substep counts, fracture budgets, level-of-detail selections, and sleep transitions derive from
   simulation state**, never from the clock or a frame index.
5. **No fast math, no floating-point reassociation** in the physics translation units.

### 12.2 Deterministic parallel accumulation — the whole determinism problem, in one place

The runtime's WP-4 analysis settles what determinism actually costs here, and it is worth quoting
because it is a load-bearing simplification: *"The compiled DAG already guarantees a
dependency-correct result regardless of dispatch order. The only source of run-to-run variation in
the result is order-sensitive floating-point accumulation. Therefore, on a single architecture,
determinism does not require a single-threaded engine or disabling work stealing."*

So the work-stealing scheduler, the adaptive event poller, and even task migration are **not**
determinism problems. Exactly three things are:

1. **Order-sensitive accumulation.** Graph colouring already removes write conflicts between
   constraints. What remains is genuine accumulation across parallel work: per-body contact impulse
   summation in a Jacobi-style pass, and soft-body vertex accumulation across the tetrahedra that
   share a vertex. Both use a **fixed-order** reduction — a scatter into a deterministically-ordered
   slot list followed by an in-order segmented tree reduce — never floating-point atomics and never a
   device `reduce` whose combination order is unspecified. Since the runtime does not ship this
   primitive (§6.6), the physics layer builds it, as a P0 deliverable.
2. **Schedule-order leakage into state.** No "first writer wins" race, no non-commutative in-place
   update whose result depends on which sibling ran first. The region-overlap tracker makes a genuine
   write-write conflict an explicit dependency rather than a silent race, which is what makes this
   auditable at all.
3. **Random number generation.** Per-body and per-element streams seeded by stable identity, never by
   a global counter touched in schedule order. Nothing in this plan needs randomness today; the rule
   is recorded so nothing introduces it carelessly later.

The rebalancer is still switched off for physics (§6.6) — for frame-time jitter, not for correctness.
The distinction matters: turning it on would not make the simulation wrong, it would make the frame
time unpredictable.

### 12.3 Snapshot and rollback

The physics state that must be snapshottable per tick: body state columns, soft-body vertex state,
plastic rest matrices, joint multipliers and break flags, sleep state, and the warm-start accumulators.
All of it is already a pointer-free column, which is what makes SushiLoop's dirty-chunk snapshot
(`SUSHILOOP.md`) applicable without a special case. The **cooked assets are immutable and shared** and
are never snapshotted — only the handles.

Rollback correctness gets its own test: simulate N ticks, snapshot at tick K, roll back, replay, and
assert byte equality of the whole physics state (§15.4).

---

## §13 Performance and scale

### 13.1 Targets

Stated so they can be measured and failed, not as aspiration. Reference: one desktop-class GPU through
SushiRuntime, 60 Hz tick.

| Scene | Target |
|---|---|
| 1 000 awake rigid bodies, mixed shapes, stacking | ≤ 2 ms/tick |
| 10 000 mostly-sleeping rigid bodies | ≤ 0.5 ms/tick |
| One soft body, 20 000 tetrahedra, 32 substeps | ≤ 3 ms/tick |
| One hybrid vehicle (400 nodes, 2 000 beams, 4 wheels, powertrain) | ≤ 2 ms/tick |
| Four such vehicles in contact | ≤ 6 ms/tick |
| Cooking: a 50 000-triangle mesh at fidelity 0.5 | ≤ 3 s, off-thread |

### 13.2 Where the time is won

1. **Sleeping and islands.** The single largest win in any real scene and the cheapest to implement.
   A settled island costs its broadphase bound update and nothing else.
2. **Persistent everything.** The broadphase tree, the pair cache, the manifolds, the warm-start
   accumulators, and the colouring all survive across ticks. Today's code rebuilds all of them, twice
   per substep.
3. **Collision once per tick, not per substep** (§6.1).
4. **Device-resident solve, one `run()` per tick.** The constraint solve is already on the device;
   the plan moves the broadphase, the narrowphase, the contact solve, and the predict/derive/velocity
   stages there too, and unrolls the whole substep loop into a single graph composition so a tick is
   one blocking call rather than 32 (§6.6). Structure-of-arrays layout for the state columns, since a
   projection touches four of twelve fields. Sleeping islands are dropped from the `DynamicGraph`
   rather than run and discarded.
5. **Levels of detail.** Distance-based soft-body tiers (§9.7); a distant vehicle's shell freezing to
   its rigid core; a coarse substep count for distant islands.
6. **Budgets.** Per-tick caps on contacts, fracture events, and continuous-collision escalations, all
   state-derived, all reported when hit — a physics engine that silently exceeds its budget is how a
   frame-time spike becomes a mystery.

### 13.3 Instrumentation

`PhysicsStatistics` per tick: awake/sleeping body counts, island count and largest island, broadphase
pairs tested and produced, manifolds and contact points, constraints per colour, substeps taken,
continuous-collision escalations, fracture events, and per-stage timings — wired into the existing GPU
profiler (`render/graph/gpu_profiler.hpp`) for device stages and an editor panel for the rest. Nothing
in §13.1 is verifiable without it, so it is a P0 deliverable, not a P8 one.

---

## §14 The editor and authoring surface

Following the established pattern from `editor-component-inspector-pattern`: panels edit the **selected
entity's** component, and previews get their own viewport.

- **Physics Material** — a project asset with a preview (a ball dropped on a ramp at the authored
  friction and restitution).
- **Collider inspector** — primitive or cooked asset; shows the cook report (convex piece count,
  Hausdorff error, mass, inertia) and a "Re-cook" button; draws the actual collision geometry as an
  overlay so "the collider is not the mesh" is *visible*.
- **Soft Body inspector** — the asset, the **fidelity slider**, the material, a "Bake" button with
  progress, and the cook report (tetrahedra, worst element quality, unembedded vertices, estimated
  cost). Debug views: wireframe tetrahedra, stress heat map, plastic-strain heat map.
- **Assembly editor** — the parts list, the joints list, joint gizmos (an axis and an arc for a hinge's
  limits, drawn in the viewport and draggable), collision-filter matrix, and a live joint-load readout
  while playing.
- **Vehicle editor** — node/beam visualization, group colouring, mass distribution readout, powertrain
  curve editing.
- **Physics debug draw** — contacts and normals, manifold points, island colouring, sleeping state,
  broadphase bounds, query results. Toggled per category.
- **Physics profiler panel** — §13.3's statistics over time.

`.sushiscene` serialization extends for each new component, following the existing independent-field-pair
convention (`ARCHITECTURE.md` §4.1).

---

## §15 Testing strategy

Extending the existing `tests/functional/{unit,integration,regression}` layout and `se test --suite
functional`.

1. **Unit — geometry and math.** Every narrowphase routine against analytic answers, including the
   degenerate cases §1.3 names. Mass properties against closed-form results for a box, sphere, and
   cylinder. Tetrahedralization invariants: no inverted elements, every render vertex embedded,
   volume preserved within tolerance. Distance-field sampling against brute-force triangle distance.
2. **Unit — solver.** Each constraint projection against a host mirror, exactly as
   `test_xpbd_solver.cpp` does today. Convergence: N substeps must reduce the residual by the expected
   order.
3. **Integration — physical correctness.** These are the tests that make the difference between "it
   runs" and "it is right":
   - A ball with restitution `e` dropped from height `h` returns to `e²h` within tolerance.
   - A box on a ramp begins to slide exactly at the angle `atan(μ_static)` — the friction test that
     today's engine would fail outright.
   - A stack of ten crates is stable and does not drift for 10 000 ticks.
   - A pendulum's period matches the analytic value; energy drift over 10 000 ticks stays bounded.
   - A hinge with limits does not exceed them; a motor reaches its target; a break threshold breaks at
     the right load.
   - A cantilever beam's tip deflection under load matches Euler–Bernoulli theory within the tolerance
     the element count justifies. **This is the *mukavemet* test** — it proves the FEM material
     parameters mean what they say.
   - A soft body under sustained load past its yield stress keeps a permanent deformation of the
     predicted magnitude.
4. **Regression — the penetration contract.** Golden-metric scenes measuring maximum penetration at
   rest and in motion, tunneling at escalating speeds (a 200 m/s sphere through a 1 cm plate must not
   pass), and the render-versus-collision Hausdorff error for a set of reference meshes.
5. **Determinism.** Byte-equal replay over 10 000 ticks; snapshot-rollback-replay byte equality;
   identical results with a shuffled body insertion order (which catches every accidental
   order-dependence); and — the one that actually catches §12.2 violations — **byte-equal results
   with the runtime's worker count varied**, since a fixed-order reduction is exactly the thing that
   must not care how many threads ran it.
6. **Runtime-contract tests.** `compose_count()` / `compile_count()` hold at 1 across an unmutated
   multi-step run and advance by exactly one per topology change; a tick issues exactly one `run()`;
   `add_untracked` appears nowhere under `physics/`, which is a grep rather than a review checklist
   now that the runtime names the dependency-blind overloads (§6.6, §18); and a contact count driven
   past the compiled capacity fails the tick loudly rather than truncating (§18, R1).
7. **Conformance suites** per §4.4, run against every implementation of each seam.
8. **Performance benchmarks** with recorded baselines, run in the same suite, failing on regression
   beyond a threshold — because a physics engine degrades silently.

---

## §16 Roadmap

Each phase is independently shippable, ends green under `se build` / `se test --suite functional`, and
has an acceptance criterion that is a **test**, not an opinion. The status column is the single place
progress is recorded.

| Phase | Deliverable | Acceptance | Status |
|---|---|---|---|
| **P0** | **Foundations.** Neutral `geometry/` module + the distance-baker move (§3.4). `physics/` restructured into §3.1's modules. `IPhysicsSimulation` split per §4.3. **`RuntimeGraphBuilder`: the single adapter naming `SushiRuntime::`, the solver migrated to the `Dynamic` `add()` form, the substep loop unrolled into one composition, predict/derive moved off the host, device residency for the hot columns (with the four-byte count slots left `Shared`), the rebalancer switched off, and the accumulation paths wired to the runtime's segmented fixed-order reduce rather than a hand-built one (§6.6, §12.2, §18).** Mutable world with generational handles, fixed-capacity buffers, and incremental recolouring (§6.4). `PhysicsMaterial`, body flags, `center_of_mass_local`. Deterministic contact ordering. `PhysicsStatistics` + profiler wiring (§13.3). Substepping schedule (§6.1, §6.2). The §1.3 correctness fixes. | Existing tests stay green; a body can be added and removed mid-simulation without a rebuild; **one `run()` per tick and a `compose_count()` that stops climbing**; statistics appear in the editor. | **In progress** — see §16.1 |
| **P1** | **Contact quality.** Persistent manifolds with face clipping and reduction (§7.3), warm starting, static friction in the positional pass, dynamic friction and restitution in the velocity pass (§7.4), `contact_offset`/`rest_offset` (§7.6), contact events. Broadphase made incremental and three-axis, once per tick. | The restitution, angle-of-repose, and ten-crate-stack tests pass. Contact cost drops measurably against the P0 baseline. | Not started |
| **P2** | **Shapes and scale.** Capsule, convex hull with GJK/EPA, static triangle mesh with a bounding-volume hierarchy and edge-normal correction, height field, compound shapes. `Transform::scale` honoured. Islands and sleeping, mapped onto `DynamicGraph` regions (§6.6). Bounding-volume-hierarchy broadphase. Collision filters and layers. Scene queries and triggers (§7.7). | 1 000 mixed-shape bodies at the §13.1 target; 10 000 mostly-sleeping bodies at target; queries return correct hits under a conformance suite. | Not started |
| **P3** | **Joints, assemblies, MBD.** The §10.1 joint library with limits, motors, and drives. Joint force/torque recovery (§10.4). Breakable joints. `PhysicsAssembly` asset, instancing, and the editor (§14). Ragdoll wired to `Animation::RagdollBlend`. | **The chassis-plus-hinged-door scene works end to end**: the door swings within its limits, carries load, reports its hinge force, and tears off above its break threshold. Joint accuracy tests pass. | Not started |
| **P4** | **The cooking pipeline.** `geometry/` triangle mesh utilities, the import-processor chain, `CollisionCooker` (mass properties, convex decomposition, distance field), `SoftBodyCooker` (§8.3, all ten stages), the fidelity dial, the content-hash cache, `CookingReport`, and the editor bake surface. | **Dropping a mesh into the project produces a `.sushisoft` and a `.sushicollision` without a manual step**, at the authored fidelity, cached, with a report; the cooker invariants of §15.1 hold on a corpus of deliberately dirty meshes. | Not started |
| **P5** | **Penetration hardening.** Speculative contacts, conservative advancement, substep escalation (§7.5). Signed-distance-field collision as a first-class narrowphase path. Maximum depenetration velocity. The regression scenes of §15.4. | **Nothing tunnels** at the tested speeds; measured resting penetration stays within `rest_offset + tolerance`; the Hausdorff error is reported per asset. | Not started |
| **P6** | **FEM soft bodies and strength.** The neo-Hookean two-constraint model (§9.1), `SoftBodyMaterial` and presets, stress readout and heat map (§9.3), plasticity (§9.4), fracture (§9.5), soft-vs-rigid and soft-vs-soft collision (§9.6), levels of detail (§9.7), and **the mesh binding of §8.6 driven end to end** — the embedding kernel, deformed normals, and `ClothStrandView` generalized to `DeformableMeshView`. Cloth gains bending. Cosmetic bodies gain the `float` column (§6.5). | **The cantilever-deflection test matches theory**; a body past yield keeps a permanent dent; a fractured body's render mesh follows correctly; 20 000 tetrahedra at the §13.1 target. | Not started |
| **P7** | **Vehicles.** `NodeBeamAsset` and its cooker, beam plasticity and breakage, the hybrid rigid-core structure, suspension joints, the powertrain chain (§11.4), the tyre model (§11.5), wind coupling (§11.6), the vehicle editor. | **A drivable vehicle that deforms permanently on impact and loses parts**, at the §13.1 target, deterministic under replay. | Not started |
| **P8** | **Scale.** Device-resident broadphase, narrowphase, and contact solve. Structure-of-arrays state columns. Deterministic parallel accumulation (§12.2). Per-island substepping. Half-precision storage measured and kept or dropped (§6.5). Optional runtime-accelerated cooking stages (§6.6). Budgets and their reporting. | Every §13.1 target met or beaten; determinism tests still byte-equal; the conformance suites pass for the device implementations. | Not started |
| **P9** | **Gameplay surface.** Kinematic bodies, character controller, the full event stream into gameplay/audio/VFX through `IPhysicsEventSink`, rollback integration and its snapshot, and the networking validation harness. | Snapshot-rollback-replay byte equality across 10 000 ticks including contacts and fracture; impact events drive audio and VFX in a demo scene. | Not started |

### 16.1 P0, item by item

The phase table is deliberately coarse. P0 is large enough that "in progress" hides more than it
says, so this is what is actually done and what is not.

| Item | State |
|---|---|
| `physics/` restructured into §3.1's modules | **Done.** `core/`, `geometry/`, `collision/`, `constraints/`, `solver/`, `soft/`, `scene/`. `collision.hpp` split: the shape value types went down to `geometry/shapes.hpp`, the pair logic stayed in `collision/narrowphase.hpp`, and `Contact` moved to `collision/contact.hpp` so the solver can name a contact without naming a shape. |
| `physics/core` | **Done.** Generational `BodyHandle`/`ConstraintHandle` over a fixed-capacity `HandleTable`, `PhysicsMaterial` with combine modes, `BodyFlags` + `CollisionFilter`, `PhysicsConfiguration`, `PhysicsStatistics`, and `RigidBodyT`'s new columns (`center_of_mass_local`, `material_index`, `flags`, `motion_measure`, `sleep_timer`, `island_index`). |
| Mass properties (§1.2 item 6) | **Done.** `geometry/mass_properties.hpp`: closed forms for sphere, box and cylinder, the parallel-axis shift for compounds, and the inversion that keeps zero meaning "cannot rotate about this axis". Not yet wired into the extract, so an author still types an inverse mass. |
| The §1.3 correctness fixes | **Done.** Both. A sphere centre inside a box now leaves through the nearest face instead of always through +Y, and the plane contact uses the same projection as the pair contact — it used to carry an extra `inv_mass / w` factor that was only right for a body of unit inverse mass. |
| Mutable world (§6.4) | **Done.** `add_body`/`remove_body`/`add_constraint`/`remove_constraint`, valid immediately, no `finalize()`. Fixed-capacity buffers, generational handles, per-colour bands kept dense by swap-remove, and incremental recolouring over a 64-bit per-body colour mask. Removing a body removes its constraints. |
| `RuntimeGraphBuilder` (§6.6) | **Done for the rigid distance solver.** One file names `SushiRuntime::`; `IConstraintSolver` is the seam. The whole substep loop is unrolled into one composition and the tick issues one `run()`. Every node is late-bound, so a world that changes every tick keeps `compile_count() == 1`. Predict, projection and the velocity derivation are graph nodes; the hot columns are `Residency::Device` and the host reaches them only through `read_range`/`write_range`. The rebalancer is switched off and every allocation is pinned to one `DeviceIndex`. |
| Substepping schedule (§6.2) | **Done.** Derived from the previous tick's motion maximum, which the graph computes with the runtime's fixed-order `add_reduce` so the value does not depend on the worker count. Surplus substeps are switched off by `when()` rather than removed, so the count moves without recomposing. |
| Neutral `geometry/` module (§3.4) | **Not started.** The distance-baker still lives in `render/gi/`. Nothing in P0 needs it yet; P4 does. |
| `IPhysicsSimulation` split (§4.3) | **Done for what exists.** `IRigidBodyService`, `IClothService`, `IStaticGeometryService` and `IPhysicsStepper`, composed by `IPhysicsScene`, in `sim/physics_services.hpp` — a header that names no runtime type, no solver and no shape, so depending on one service no longer drags in the whole solver. `ICollisionQueryService` and the rest of §4.3's list arrive with the features behind them. |
| The extract (§4.1) | **Done.** `gather_rigid_descs`, `gather_static_planes` and `collision_radius` left `RuntimeSimulation` for `sim/physics_extract.hpp`, which takes a flat list of source entities and returns descriptors. `RuntimeSimulation` now owes it only what needs a world: who is alive, in what order, and where the hierarchy puts them. Nine unit tests cover the cases that used to be unreachable — a cylinder collapsing to a sphere, a collider overriding a visual, a plane that is also a rigid body. |
| Deterministic contact ordering (§12.1) | **Done for the host pass.** The candidate pairs are sorted by body index before resolution, so the Gauss-Seidel order is a function of simulation state rather than of whatever order the sweep's axis sort happened to produce. The pass is still on the host; moving it into the graph is what §12.2 waits on. |
| Broadphase once per tick (§6.1) | **Done.** It ran twice per sub-step — sorting the whole scene tens of times a tick to learn something that barely changes, and the single reason a large sub-step count was unaffordable. It now runs once, against bounds swept by how far a body can travel over the whole tick, which is what makes once-per-tick sound rather than merely cheaper. |
| Segmented accumulation (§12.2) | **Not started**, and blocked on the item above rather than merely unscheduled. `add_segmented_reduce` folds per-body contact impulses; there are no per-body contact impulses to fold until the contact pass is inside the graph. The plain fixed-order `add_reduce` *is* in use, for the substep schedule's motion maximum. |
| `PhysicsStatistics` wiring (§13.3) | **Partial.** Populated by both the solver and the `sim/` boundary, reported through `IPhysicsStepper::statistics()`, and asserted by tests. Two gaps remain: nothing surfaces them in the editor, and the per-stage timings are a single total rather than a breakdown, because the contact pass is still outside the graph and has no device timings to read. |

**Ordering rationale.** P0–P2 fix the foundation, because every later phase multiplies whatever is
underneath it — and P0's runtime work in particular, since a solver that recomposes its graph every
tick makes every performance number in §13.1 unreachable no matter what is built on top. P3 lands the
assembly scenario as early as it can be landed (confirmed in §17.3), since it depends only on the
solver, not on cooking. P4 must precede P5 and P6 because both consume cooked distance fields and
tetrahedral meshes. P7 composes P3, P5, and P6, and cannot come earlier. P8 optimizes a correct system
rather than a moving one, which is the only order that works.

---

## §17 Risks, open questions, and scope

### 17.1 Explicitly out of scope

Fluids of any kind. Runtime Voronoi fracture of arbitrary rigid geometry (soft-body fracture in §9.5
covers the deformable case; rigid shattering is a separate cooked-fragment system, and if it is wanted
it is its own phase). Cross-machine bit-exact determinism (SushiLoop already ruled it out). Cloth-scale
self-collision in P6 — it is scheduled but explicitly allowed to be off by default. A reduced-coordinate
articulation solver as the primary path (§10.5).

### 17.2 Dependencies — the one thing worth buying

Everything in this plan is implementable in-house, and greenfield is the engine's established
preference. The single honest exception is **tetrahedralization quality**: robust tetrahedral meshing
of arbitrary dirty input is a research-grade problem, and §8.3's voxel-plus-body-centred-cubic approach
deliberately trades some element quality for never failing. If the quality proves insufficient in P6's
cantilever test, the options are (a) invest in a conforming Delaunay stage, or (b) take a dependency
through `ss install` (per `dependency-provisioning-via-ss`). **This is a P4 decision point, and it
should be made with the P6 test results in hand, not before.**

### 17.3 Decisions taken

Settled with the project owner on 2026-07-28. Recorded here so a later reader knows these were chosen,
not assumed.

1. **Vehicle structure: the hybrid** — a rigid chassis core with a deformable node-beam or FEM shell
   (§11.2). The pure node-beam path stays reachable per asset. The explicit brief is *"do not repeat
   BeamNG's mistakes"*, which is why §11.2 now carries the limitation-by-limitation table rather than
   just a preference.
2. **Cosmetic precision: `float`, and `sycl::half` for storage where it pays** (§6.5). Non-gameplay
   soft bodies and cloth are outside the deterministic island and may use a narrower column. The one
   engineering line drawn on top of the owner's answer: **half precision stores, it does not compute** —
   a neo-Hookean projection evaluated in 11 significant bits is not a trade-off, it is a wrong answer.
3. **Phase order unchanged: P3 (joints and assemblies) before P4 (cooking).** The car-door scenario
   lands first.
4. **The simulation runs on SushiRuntime**, and considerably more of it than today — see §6.6 for
   what runs on the device and what stays on the host, and §18 for the seams the design depends on.
   Four of the seven asks in §18 have since been built in the runtime, including the two that shaped
   this design most: a device-driven iteration count and fixed-order reductions.

### 17.4 Remaining open questions

1. **The core-to-shell seam (§11.2).** A hybrid vehicle's rigid core and deformable shell meet at the
   §10.3 attachment constraint, and that seam is where visual artifacts will appear first — a panel
   that dents perfectly but whose mount looks rigid. Whether the fix is a graded stiffness zone around
   the attachment or a wider attachment neighbourhood is a P7 question that needs to be *seen* before
   it is answered.
2. **Tetrahedralization quality (§17.2).** A P4 decision point to be made with P6's cantilever test
   results in hand.
3. **Half-precision payoff (§6.5).** Whether half-precision *storage* actually pays after the widen
   cost is a measurement, not a prediction; it is scheduled in P8 and may be dropped if it does not.

### 17.5 The largest technical risks

| Risk | Mitigation |
|---|---|
| ~~**The island-per-region mapping (§6.6) is serialized by whole-allocation cross-region ordering.**~~ **Closed.** The runtime's boundary layer now carries each pin's byte intervals and tests interval overlap (§18, R3), so two islands writing disjoint slices of one body column gain no edge. | Nothing to mitigate. The `when()`-gated fallback is retired and per-island allocations are not needed. What remains is a *layout* requirement on us: an island must be a set of index ranges the solver can name with `Buffer::region({offset, count})`, which the incremental recolouring in §6.4 already produces. |
| Incremental recolouring diverges from a full recolour and breaks determinism (§6.4). | A test asserting the incremental colouring equals the full recolour for a randomized add/remove sequence; if it cannot be made to hold, fall back to a deterministic scheduled full recolour with the divergence recorded in the snapshot. |
| Tetrahedralization quality is too poor for a correct finite-element solve (§17.2). | The cooker reports worst element quality and fails loudly at a threshold; the decision point is scheduled with test evidence. |
| Device-resident collision (P8) cannot be made deterministic at acceptable cost (§12.2). | The host implementation stays behind the same seam and remains the reference. Determinism wins over throughput; that is the standing rule. |
| The runtime's API moves under us — it is explicitly unstable and this plan leans on `Dynamic`, `DynamicGraph`, residency, and sub-region tracking. | One adapter names `SushiRuntime::` (§6.6), restoring the runtime's own L12 decision. A move costs one file plus a conformance run, not a rewrite. |
| The fixed-order reduction (now the runtime's, §18 R2) turns out to be the bottleneck on the device. Its segmented form gives one work-item per segment, so a body with ten thousand contacts and a body with one cost the same launch slot. | It is used on exactly two paths (contact impulse and soft-body vertex accumulation), both of which have a colouring-based alternative that avoids accumulation entirely at some convergence cost. Measure before optimizing; the fallback exists. The imbalance is a layout problem on our side, not a runtime defect — the guarantee is what forbids a work-stealing split. |
| A runtime bug lands in the physics tick (the runtime's own `TODO.md` still carries an open thread-local-magazine teardown item, #29). | The physics scene owns its allocations through one adapter with a single lifetime, so a teardown-ordering issue has one place to be fixed, and the engine pins a known-good runtime rather than tracking its tip. |
| Fracture plus mutable topology destabilizes the solve. | Per-tick budgets, minimum fragment sizes, and scene caps, all state-derived and all reported. |
| Scope. This is a large plan. | Every phase is independently shippable and every phase ends with a working, tested engine. There is no phase whose failure leaves the tree worse than it started. |

---

## §18 What SushiRuntime must gain

Per `docs/CONTRIBUTING.md`, *"a change that needs new runtime behavior belongs in the runtime, behind
its public API, not bolted onto the engine."* This section is the engine-side record of what this plan
needs from below; the runtime-side engineering request, with `file:line` evidence, lives in
`sushiruntime/docs/slop/PHYSICS_SUBSTRATE_REQUIREMENTS.md`.

**Four of the seven have since been built** on the runtime's `feature/physics-substrate-seams` branch,
which changes what this plan has to carry itself. Each row states what was asked, what landed, and —
for the three still open — what the physics does **without** it, because no phase may be blocked on
another repo.

| # | Ask | Status | What the physics does |
|---|---|---|---|
| **R1** | **Device-driven iteration count.** | **Built** — `API::sized_from_device(counter, index)`. | Uses it. The tick is one composition sized to the live count. See below: the mechanism is not the one this plan predicted. |
| **R2** | **Fixed-order deterministic reduction primitives** (the runtime's WP-4 item 1). | **Built** — `Graph::add_reduce` / `add_segmented_reduce`, with `Sum`/`Minimum`/`Maximum`. | Uses them. §12.2's accumulation is no longer a physics-layer deliverable; the segmented form is exactly the per-body / per-vertex shape §12.2 needs. |
| **R3** | **Sub-range cross-region ordering** in `DynamicGraph`. | **Built** — boundary pins carry byte intervals and every hazard test is an interval overlap. | Uses it. One region per island now works as intended; the `when()`-gated fallback in §17.5 is retired. |
| **R5** | **Dependency tracking on every `add()` overload.** | **Built, as Option A** — every launch shape has a tracked overload (three were missing), and the dependency-blind ones are now spelled `add_untracked`. | Uses it. The review checklist becomes a grep for `add_untracked` in `physics/`, which is a real guarantee rather than an eyeball one. |
| **R4** | **Asynchronous run** — `run()` blocks, so a tick cannot overlap with the render or audio extract. | **Open** | The simulation thread blocks for the tick, as today. Costs overlap, not correctness. |
| **R6** | **A late-bound base offset** alongside `sized()`, so a colour slice whose offset shifts between ticks needs no indirection buffer. | **Open** | An index-indirection buffer read on the device. One extra load per element. |
| **R7** | Closing the **`ThreadLocalMagazine` teardown** correctness item (#29). | **Open** | Nothing the engine can do. It is a long-running-process risk and sits on the runtime's own v1 list. |

#### R1: the answer was better than any of the three options

This plan reasoned that `Dynamic::sized()` is a host-side provider polled on the driver thread at the
step boundary, so within one `run()` it cannot carry a count a kernel produced in that same run — and
concluded that only three shapes existed: two runs with a readback between them, dispatch at capacity
with a per-lane early-out, or indirect dispatch. The plan committed to the second and kept the
`RuntimeGraphBuilder` seam thin so it could move to the third.

**That reasoning was wrong about *when* the count is read**, and the correction is worth recording
because it removes a compromise the whole §6.6 design was shaped around. A node's `sycl::handler`
closure is not evaluated when the graph is built or when the step boundary polls the host providers —
it runs **on a worker thread at dispatch**, and a node is only dispatched once `EventPolling` has seen
every predecessor's event complete and called `wait()` on it. At that point the predecessor kernel's
USM writes are visible, so a count it wrote can simply be read there.

So the runtime resolves the live size at dispatch. The result is what option (3) promised — one run,
no sync, no wasted width — without needing indirect dispatch at all:

```
broadphase   →  writes pair_count       (device)
narrowphase  →  sized_from_device(pair_count)      iterates exactly that many
             →  writes manifold_count   (device)
solver       →  sized_from_device(manifold_count)  iterates exactly that many
```

One `run()`, `compile_count()` pinned at 1, and a contact pass sized for a 50 000-contact pile-up
dispatching 800 lanes on the ordinary tick that has 800 contacts.

Two properties of the API matter for how §6.6 is written:

- **The producer edge is structural.** The counter is registered as a read of the consuming task by
  the runtime itself, so `RuntimeGraphBuilder` does not have to remember to name it in `Reads(...)` and
  cannot get it wrong.
- **Overflow fails the run; it does not clamp.** A live count above the compiled capacity throws with
  both numbers in the message. That is the correct behaviour for this system — a clamped contact count
  is a silently dropped contact — but it means **capacity planning is now a hard failure mode, not a
  quality-of-simulation one**, and §6.4's fixed-capacity buffers must be sized against the escalation
  path in §7, not against a typical tick. The engine's own budget clamp has to run *before* the count
  reaches the runtime.

One constraint the design must honour: the counter buffer must be host-addressable
(`Residency::Shared`). Everything else in the scene stays device-resident; the counters are four bytes
each and are the only exception.

#### What this changes elsewhere in this plan

- **§6.6** — the "dispatch at capacity, early-out per lane" shape is no longer the plan. The
  `RuntimeGraphBuilder` seam still exists for API churn, but not as a switch waiting for indirect
  dispatch.
- **§12.2** — the fixed-order reduction moves from "the physics layer builds it" to "the physics layer
  calls it". It leaves P0 as a deliverable and stays as a *conformance requirement*: the byte-equality
  test with the worker count varied (§15.5) still has to pass, and now it tests the runtime's primitive
  rather than ours.
- **§17.5** — the island-per-region serialization risk is closed. Option (c), `when()`-gated islands,
  is no longer the P2 fallback.
- **§15.6** — "no physics node is added through a raw-kernel overload" stops being a review checklist
  and becomes a mechanical check.

None of this changes a phase boundary or a deliverable's scope. It removes work from P0 and removes
two risks from §17.5.
