# Physics System — unified XPBD, cooking, and deformable vehicles (`SushiEngine::Physics`)

**Status:** in progress, P0 to P7 and PX complete, P8 under way, P9 not started (§16). P1, P2, P6
and P7's §13.1 acceptance numbers are built but unmeasured: every one of them is timed against a
desktop GPU, and SushiRuntime finds none on the machine this is developed on (§16.35).

This document is the **umbrella** for SushiEngine's AAA physics simulation: the product vision (one
unified XPBD solver that carries rigid bodies, articulated assemblies, cloth, and volumetric soft
bodies with real material strength), the offline **cooking pipeline** that turns any imported mesh
into a simulation-ready asset with a single fidelity dial, the **penetration** contract that binds
the visible mesh to the simulated one, and the multi-phase road to a BeamNG-class deformable-vehicle
simulation. It specifies the **architecture and the seams**, not the kernel source.

Companion docs: `docs/design/SUSHILOOP.md` (the determinism rules and the locked "GPU XPBD in double
with a floating origin" decision this plan implements), `docs/design/vfx_particle_system.md` (the
structural template for a two-backend subsystem behind one seam), `docs/design/animation_system.md`
(the ragdoll/pose seam physics feeds), `docs/design/atmosphere_system.md` (the wind field vehicle
aerodynamics and cloth will sample), and the renderer's `docs/architecture/domain-physics.md` §1
(the physics layer as it stands today).

§1 is an honest audit of what exists now; everything from §3 onward is the plan. The roadmap in §16
marks each phase's state and is the single place progress is recorded. The four decisions the
project owner has settled — the hybrid vehicle structure, cosmetic narrow precision, phase order,
and running the simulation on SushiRuntime — are recorded in §17.3; §6.6 is the concrete runtime
execution model that follows from the last of them, and §8.6 is the mesh-to-physics binding written
out end to end.

---

## §0 The decisions that shape everything

Five decisions determine the whole design. Each is stated with its cost, because each one closes
doors.

### 0.1 One solver, not a family of solvers

Every simulated thing — a crate, a car door on a hinge, a rope, a flag, a tyre sidewall, a dented
fender — is a set of **particles or rigid bodies plus constraints**, projected by one substepping
XPBD solver. `docs/architecture/domain-physics.md` §1.1 already claims this and cloth already proves
it; this plan holds the line for FEM soft bodies, joints, contacts, and node-beam vehicles as well.

*Cost:* XPBD is a positional method, so quantities other engines get for free — exact constraint
forces, reduced-coordinate articulations, analytic stiffness — have to be **recovered** from
Lagrange multipliers rather than solved for directly (§10.4). We accept that; the payoff is that a
car's chassis, its door hinge, its deforming panel, and the cloth seat inside it all converge in the
same sweep and couple two-way for free.

### 0.2 Small steps, not many iterations

The solver takes **many substeps with one constraint iteration each**, not one step with many
iterations (Macklin et al. 2019, *Small Steps in Physics Simulation*). Error falls with the square
of the substep, so N substeps beat N iterations at identical cost, and stiff constraints (a
suspension spring, a rigid hinge) stop needing compliance hacks.

*Consequence:* the outer tick rate and the substep count are the two knobs that buy stability, and
the substep count must be derived from **simulation state, never from the wall clock** (SushiLoop's
rule). A 60 Hz tick at 32 substeps is a 1920 Hz effective solve — the rate a node-beam vehicle
needs.

### 0.3 Cooked assets, not runtime discovery

Anything expensive and deterministic — tetrahedralization, convex decomposition, signed-distance
fields, bounding-volume hierarchies, mass properties, render-mesh embedding, simulation levels of
detail — is computed **once, offline, into a versioned blob keyed by a content hash**, exactly the
way `Animation::SkeletonBlob` and `VFX::CompiledEffect` already work. The runtime loads and
simulates; it never cooks.

*Consequence:* the "drop a mesh in and it becomes a soft body" experience is an **import processor**
that runs the cooker in the background, not a runtime feature. And the fidelity dial is a *cooking*
parameter, so changing it re-cooks — which is why cooking has to be fast and cached.

### 0.4 The visible mesh is the simulated mesh

The penetration contract, stated once and enforced everywhere: **what the player sees never
interpenetrates more deeply than what the solver resolved.** Two mechanisms deliver it, and both are
mandatory, not optional quality settings:

- **Rigid:** collision geometry is a cooked approximation of the render mesh whose one-sided
  Hausdorff error is measured and reported at cook time, so "the collider is fatter than the mesh"
  is a number in the inspector rather than a surprise. Contacts carry a `rest_offset` so surfaces
  come to rest *touching*, and a `contact_offset` so they are generated *before* they touch (§7.6).
- **Soft:** the render mesh is barycentrically **embedded** in the simulated tetrahedral mesh, so it
  is not "synced" to the simulation — it *is* the simulation, interpolated. The collision surface is
  the simulated surface. There is no second geometry to disagree.

*Cost:* an embedded render mesh cannot be edited independently of its simulation mesh at runtime,
and a re-cook invalidates the embedding. Accepted.

### 0.5 Determinism is a build-time property, not a mode

Every part of this system lives **inside** SushiLoop's deterministic island: fixed substep count
from state, fixed constraint colour order, contacts sorted by a stable key before they are solved,
no wall-clock, no unseeded randomness, no iteration over a hash container where order is observable.
GPU execution is allowed because SushiLoop's promise is *same-binary* determinism, not cross-vendor
determinism.

*Cost:* several attractive optimizations are forbidden — non-deterministic atomic accumulation
orders, "solve whatever is ready" work stealing, floating-point reassociation. Where a parallel
reduction is needed, it is a **fixed-order** reduction (§12.2).

---

## §1 Audit — what exists today, honestly

`include/SushiEngine/physics/` is 2 233 lines of header-only templates and it is a *good* skeleton:
the XPBD core, graph colouring, a device solve graph, and a clean precision-parametric design. It is
also roughly 8 % of a AAA physics engine. This section is the honest inventory, because every phase
in §16 is scoped against it.

### 1.1 What is there and works

| File | What it delivers |
|---|---|
| `engine/domain/physics/include/SushiEngine/physics/core/rigid_body.hpp` | `RigidBodyT<T>`: pose, previous pose, velocity, diagonal body-local inverse inertia, inverse mass, quadratic drag. `predict()` / `update_velocity()` — the two halves of an XPBD substep. Trivially copyable, device-safe. |
| `engine/domain/physics/include/SushiEngine/physics/constraints/xpbd_constraint.hpp` | `XpbdDistanceConstraintT<T>`: two body indices, two local anchors, a rest length, a compliance. **The only constraint type in the engine.** |
| `engine/domain/physics/include/SushiEngine/physics/solver/xpbd_solver.hpp` | `XpbdDistanceProjectionT<T>` (the correct generalized-inverse-mass projection with angular coupling) and `XpbdSolver<Constraint>`, which graph-colours the constraint set, uploads one buffer per colour, and compiles a replayable SushiRuntime graph of `iterations × colours` nodes. |
| `engine/domain/physics/include/SushiEngine/physics/solver/graph_coloring.hpp` | Greedy edge colouring over bodies. Correct, deterministic, host-side, `O(constraints × colours)`. |
| `engine/domain/physics/include/SushiEngine/physics/scene/physics_world.hpp` | `PhysicsWorld<Constraint>`: register bodies/constraints, `finalize()` once, then `step()` or the split `predict_substep_field()` / `solve_constraints()` / `derive_velocity()` trio that lets two worlds run in lockstep. |
| `collision.hpp` (since split — see §16) | Sphere, plane, axis-aligned box, oriented box; sphere-plane, sphere-sphere, box-sphere, box-plane, oriented-box-plane, oriented-box-sphere, and oriented-box-oriented-box by separating-axis test. Pure, unit-tested. |
| `engine/domain/physics/include/SushiEngine/physics/collision/broadphase.hpp` | `Aabb<T>`, overlap test, and `sweep_and_prune()` on the X axis. |
| `engine/domain/physics/include/SushiEngine/physics/collision/contact_solver.hpp` | `ContactBody<T>` — a shape plus live pointers into whichever buffer owns the body — generalized inverse mass with the angular term, single-point two-way resolution, and plane resolution. |
| `engine/domain/physics/include/SushiEngine/physics/soft/cloth.hpp` / `engine/domain/physics/include/SushiEngine/physics/soft/soft_body.hpp` | Topology builders: a 2D grid and a 3D structural+shear lattice of distance constraints. No new solver, exactly as intended. |
| `engine/domain/physics/include/SushiEngine/physics/solver/pgs_solver.hpp` | The earlier point-mass Projected Gauss-Seidel solver, superseded by XPBD but still tested and demoed. |
| `engine/world/simulation/include/SushiEngine/simulation/physics_simulation.hpp` | `IPhysicsSimulation` — the boundary seam. Rebuild bodies, rebuild cloth grids, set static planes, step, read poses. Solve runs in `double`, converts at the edge. |
| `engine/world/simulation/include/SushiEngine/simulation/physics_bridge.hpp` | The ECS-component half: `PhysicsBody` component and `sync_transforms_from_physics()`. |

Tests already cover the geometry, the colouring, the solver against a byte-for-byte host mirror, the
bridge, and cloth. That test culture is the reason this plan can be aggressive.

### 1.2 What is missing, by consequence

These are not "nice to have" — each one is load-bearing for something the user asked for.

1. **No joints.** The only constraint is a distance constraint. A hinged car door is unbuildable
   today: there is no hinge, ball, slider, fixed, or cone-twist constraint, no limits, no motors, no
   drives, and no articulation concept at all. *(Blocks: assemblies, MBD, ragdolls, vehicles.)*
2. **No friction and no restitution, anywhere.**
   `engine/domain/physics/include/SushiEngine/physics/collision/contact_solver.hpp` is a positional
   projection only; velocity is whatever the position change implies. Every contact in the engine is
   perfectly inelastic and perfectly frictionless. A box cannot rest on a ramp, a ball cannot
   bounce, a tyre cannot grip. *(Blocks: everything that matters.)*
3. **Single-point contact manifolds.** `collide_obb_obb` returns the midpoint of the two support
   points. `engine/domain/physics/include/SushiEngine/physics/collision/contact_solver.hpp`'s own
   comment says it: *"a box resting on a face is held by one corner at a time and rocks slightly."*
   Stacking, resting, and any large flat contact are wrong.
4. **The narrowphase knows three shapes.** Sphere, oriented box, half-space plane. `ColliderParams`
   authors Box / Sphere / Cylinder / Plane, and `RuntimeSimulation::gather_rigid_descs()` collapses
   anything that is not a Box into **a sphere of one radius** — a Cylinder collider silently
   simulates as a sphere. No convex hulls, no capsules, no triangle meshes, no height fields, no
   signed-distance fields. Static world geometry can only be an **infinite half-space plane**.
5. **No entity scale reaches the collider.** `Transform::scale` is a render concept; the physics
   reads the authored collider extents directly. Scaling an object in the editor does not scale its
   physics.
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
10. **The broadphase is rebuilt from scratch, twice per substep.**
    `PhysicsSimulation::resolve_contacts` runs `CONTACT_ITERATIONS = 2` sweeps, and *each* sweep
    refills the AABB array and re-runs `sweep_and_prune`, which sorts `O(n log n)` and allocates two
    vectors internally. It is also hard-coded to the X axis — a scene laid out along a road is the
    degenerate case.
11. **Only the constraint projection is on the device; everything around it is a host loop.** Per
    substep, `PhysicsWorld::step` runs a host loop over every body (`predict`), then
    `XpbdSolver::solve()` — which **zeroes every Lagrange multiplier in a host loop over every
    constraint** before replaying the graph — then the contact pass on the host, then another host
    loop over every body (`update_velocity`). At 32 substeps and 100 000 constraints that is 3.2
    million host-side writes per tick before any physics happens, plus 32 blocking host↔device round
    trips.
12. **The solve graph cannot change size without recompiling.** `XpbdSolver::build_graph` bakes
    `Extent{n}` per colour at construction, using the capture-style `Graph::add(Extent, Cap0, ...)`
    overload — which is the one overload with **no `Dynamic` counterpart** in the runtime API. Since
    contacts appear and vanish every tick, this is the reason contacts were pushed out of the solver
    and into a host pass in the first place
    (`engine/domain/physics/include/SushiEngine/physics/collision/contact_solver.hpp`'s own file
    comment says so). §6.6 shows this is a solvable API-shape problem, not a fundamental one.
13. **Soft bodies are mass-spring lattices.** `build_soft_body_lattice` links axis and face-diagonal
    neighbours with distance constraints. There is no volume preservation, so it collapses and
    inverts under load; no bending resistance; no material parameters (Young's modulus, Poisson
    ratio, density); no stress; no plasticity; no fracture; and no relationship to any imported
    mesh.
14. **No mesh reaches the physics at all.** The only triangle-mesh representation in the engine
    lives behind Vulkan in `engine/presentation/render/source/geometry/mesh_registry.hpp`
    (`Geometry::MeshVertex`, device buffers), and the only signed-distance baker
    (`engine/presentation/render/source/gi/mesh_sdf_baker.hpp`, `Gi::MeshSdfBrick`) is a render-side
    global-illumination tool. There is no engine-neutral triangle mesh, no cooking step, no physics
    asset, and no import hook. **This is the single biggest gap relative to what the project
    wants**, and §8 exists to close it.
15. **`ContactBody::is_cloth` is a type tag inside a value type.** Behaviour switches on a boolean
    (`if (a.is_cloth && b.is_cloth) return;`). Adding soft bodies, characters, triggers, or vehicles
    this way multiplies the flags and the branches — the exact Open/Closed violation §4 exists to
    stop.
16. **Cloth rebuilds lose all state**, by documented design: any `ClothParams` edit replaces the
    grid.
17. **No queries.** No raycast, no sweep, no overlap, no trigger volumes, no collision layers or
    filters, no contact events. Gameplay cannot ask the physics anything.
18. **No character controller, no kinematic bodies.**
19. **No profiling.** The physics contributes nothing to the GPU profiler or any statistics panel.

### 1.3 Two small correctness notes worth fixing early

- `collide_box_sphere` and `collide_obb_sphere` push out along **+Y** when the sphere centre is
  exactly
  inside the box (distance ≈ 0), rather than out of the nearest face. Deep penetration recovery
  picks
  an arbitrary direction.
- `engine/domain/physics/include/SushiEngine/physics/collision/contact_solver.hpp`'s plane path
  scales the impulse by `inv_mass / w` so the centre-of-mass motion matches the old
  purely-positional behaviour. That is a deliberate compatibility choice, and it means the angular
  share of a plane contact does **not** conserve the correction — it is not the same projection the
  pair path uses. Worth unifying when manifolds arrive.

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
| SushiEngine `VFX::EmitterCompiler` | The authoring→compile→POD-blob pipeline shape (`docs/design/vfx_particle_system.md` §3). The physics cooker is the same shape with heavier geometry. |
| SushiEngine `Animation::SkeletonBlob` | The versioned, trivially-loadable cooked-asset blob format and its import path. |
| SushiEngine `Gi::MeshSdfBrick` | An existing, working signed-distance baker to **lift and share**, not to duplicate (§3.4). |
| SushiRuntime `SIMULATION_ENGINE_SUBSTRATE_PLAN.md` | The substrate contract: late-bound sizes, region graphs, device residency, native host nodes, and the WP-4 determinism analysis. Its locked decisions L7/L8/L11/L12/L14 name this system as the runtime's target workload (§6.6). |

**Skip list, deliberately:** fluid simulation (SPH/FLIP/PBF) — a separate system, not this one;
runtime Voronoi fracture of arbitrary rigid geometry (§17.1 revisits it); reduced-coordinate
Featherstone articulations as the *primary* path (§10.5 keeps it as an escape hatch); cross-vendor
bit-exact determinism (SushiLoop already ruled it out); soft-body **self**-collision at cloth scale
in the first soft-body phase (§9.6 schedules it).

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
| `physics/geometry` | Shape value types, signed-distance sampling, bounding-volume hierarchy traversal, mass-property computation. | Knows about bodies or constraints. Pure geometry, pure functions, unit-testable in isolation — the property the pre-split `collision.hpp` already had and must keep. |
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

`engine/presentation/render/source/gi/mesh_sdf_baker.hpp` bakes a signed-distance brick from
`Geometry::MeshVertex`. The physics needs exactly that, plus a triangle mesh it can traverse on the
host and the device, and it must not depend on Vulkan to get it.

**Action (phase P0):** create a top-level `geometry/` module holding an engine-neutral
`Geometry::TriangleMesh` (positions, indices, optional normals, bounds — no device handles) and move
the signed-distance baker there as `Geometry::SignedDistanceFieldBaker`. `render/gi/` and
`engine/presentation/render/source/geometry/mesh_registry.cpp` consume it from the new home;
`Gi::MeshSdfBrick` becomes a thin alias or is replaced outright. This is a **pure move plus a
rename**, doc-visible but behaviour-neutral, and it unblocks every mesh-aware feature in this plan.

*Why this and not "physics reads the mesh registry":* the physics layer sits **below** the renderer
in the dependency order. Pointing it at a Vulkan-owning class inverts the layering and makes the
cooker require a device. The neutral module is the correct fix, and the renderer benefits too — a
mesh's distance field stops being a global-illumination private.

---

## §4 The SOLID contract

The user's stated first priority, and `CLAUDE.md`'s. Stated per principle, with the concrete
mechanism
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

**A new soft-body material model** implements `ISoftBodyModel` and is selected by the cooked asset.
The solver schedules it by the constraints it emits; it does not know the model exists.

The cooker follows the same rule: a cooking stage is an `ICookingStage` in an ordered pipeline
(`Repair → Voxelize → Tetrahedralize → Optimize → Embed → Decompose → BakeDistanceField →
BuildLevelsOfDetail → Serialize`),
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

A gameplay system that only raycasts depends on `ICollisionQueryService` and nothing else. The
editor's soft-body panel depends on `ISoftBodyService`. `IPhysicsScene` exists only for the host
that owns the whole thing.

### 4.4 Liskov — the substitutability tests

Every seam in §3.3 ships with a **shared conformance test suite** run against every implementation:
`IBroadphase` implementations must all emit the same sorted pair set for the same input;
`IConstraintSolver` implementations must agree within a stated tolerance on a reference scene;
`ISoftBodyModel` implementations must all converge to the same rest shape. This is how a device
solver is allowed to replace a host solver without silently changing behaviour, and it is the
generalization of what `tests/integration/test_xpbd_solver.cpp` already does with its host mirror.

The `ContactBody::is_cloth` flag (§1.2 item 15) is deleted: behaviour differences become **filter
masks and material properties**, which are data, not type tags.

### 4.5 Dependency inversion — who names whom

- `physics/` names `Geometry::TriangleMesh`. It never names `Render::Geometry::MeshRegistry`,
  Vulkan,
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
diagonal store the **principal-axis rotation** produced by the cooker's eigendecomposition, folded
into `center_of_mass_local`'s frame.

### 5.2 Shapes

```
Sphere · Capsule · Box · ConvexHull · TriangleMesh(static) · HeightField · SignedDistanceField · Compound
```

A shape is a value type in `physics/geometry`, referencing cooked data by handle (a convex hull's
plane and vertex arrays, a mesh's bounding-volume hierarchy, a distance field's brick) so a shape
itself stays small and copyable. `Compound` is a list of child shapes with local transforms — this
is how one body gets several colliders, which is how a car chassis is one body with a dozen convex
pieces.

Every shape carries a **`convex_radius`** (a small inflation used by the closest-point routines to
keep them numerically stable — the standard trick) and the body-level `contact_offset` /
`rest_offset` (§7.6).

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

New components (trivially copyable, per
`engine/world/simulation/include/SushiEngine/simulation/components.hpp`'s rule) and new host-side
records:

```
  RigidBody            (exists as host record; gains material, flags, collision filter)
+ SoftBody             { SoftBodyAssetId asset; Scalar fidelity_scale; std::uint32_t flags; }
+ PhysicsJoint         { EntityId body_a, body_b; JointKind kind; ... }        // host record + asset
+ AssemblyInstance     { AssemblyAssetId asset; std::uint32_t root_part; }
+ VehicleInstance      { NodeBeamAssetId asset; PowertrainState state; }
+ CollisionFilter      { std::uint32_t layer; std::uint32_t collides_with_mask; }
```

`ColliderParams` is superseded by a `Collider` record that can name a cooked `CollisionAsset` **or**
a primitive, and that finally honours `Transform::scale` (§1.2 item 5).

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
  *swept* bounds with a contact offset, then re-evaluated (depth and points refreshed, features
  kept) cheaply each substep. This is the standard XPBD-with-substeps arrangement and it is what
  makes 32 substeps affordable. Today's code runs a full broadphase twice *per substep*.
- **A velocity pass is added.** XPBD's positional projection cannot express restitution or dynamic
  friction; both are velocity-level. Today's engine has no velocity pass at all, which is exactly
  why it has neither.

### 6.2 Substep count derived from state

```
substeps = clamp(ceil(max_body_motion_per_tick / motion_budget), minimum, maximum)
```

where `max_body_motion_per_tick` is the largest `|velocity| * dt / characteristic_size` over awake
bodies. Derived from **simulation state only** — SushiLoop's determinism rule. A vehicle scene pins
it near the maximum; an idle scene drops to the minimum. Per-island substepping is a §13
optimization, not a first-phase feature.

### 6.3 Heterogeneous constraints in one graph

`XpbdSolver<Constraint>` is single-typed today. The generalization keeps the compile-once-replay
structure and adds a **kind dimension**:

- Constraints are stored in **per-kind, per-colour buffers**. Colouring runs over the union of all
  constraints (a joint and a distance constraint on the same body pair must not share a colour), so
  `color_constraints` is generalized to take a body-pair accessor rather than requiring `.a`/`.b`
  fields — one small change, and every existing caller keeps working.
- The graph emits one node per `(kind, colour)` pair. Within a colour the kinds are independent by
  construction, so their relative order does not matter; across colours the runtime orders them, as
  it does now.
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
- **Fixed-capacity device buffers with a live count**, not capacity doubling. A
  `SushiRuntime::Buffer` cannot be resized in place — a growth reallocates and moves, which
  invalidates the pointer every graph node captured. So the scene allocates at a configured
  capacity, the *live* element count is a per-step value (§6.6), and exceeding capacity is a
  budgeted, reported, recomposing event rather than a routine one.
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
planet-scale world with a floating origin needs it. Everything precision-related is already a
template parameter (`RigidBodyT<T>`, `XpbdDistanceConstraintT<T>`,
`PhysicsWorld<Constraint>::Real`), so this is a policy choice per body kind, not new machinery.

**Cosmetic bodies get a narrower column.** A soft body or cloth marked non-gameplay lives outside
the deterministic island's authority in exactly the way cosmetic VFX already do, so it may run:

- **`float`** as the default cosmetic column — half the bandwidth, and bandwidth is what a soft-body
  solve is actually limited by.
- **`sycl::half` for storage only**, where profiling shows it pays: positions and velocities stored
  at half precision and widened to `float` inside the projection. Half-precision *arithmetic* is not
  used for the projection itself — a neo-Hookean solve in 11 significant bits is not a stability
  trade, it is a broken solve. The rule is **half to store, float to compute**, and it applies to a
  curtain, a flag, or a distant vehicle's shell, never to anything a rollback replays.

The precision of a body is a property of its cooked asset and its component flags, resolved once at
instantiation. A body cannot change precision while it is simulating.

Positions are relative to the floating origin, which the engine already implements and tests
(`tests/unit/test_floating_origin.cpp`, `tests/unit/test_floating_origin_stress.cpp`). Physics must
rebase with it: a `rebase(offset)` on the scene shifts every position and every cached contact point
in one pass.

### 6.6 Execution on SushiRuntime

**Yes — the simulation runs on SushiRuntime**, and more of it than does today. This section is the
concrete mapping, because "put it on the GPU" is not a plan and the runtime's shape dictates the
solver's shape. Everything below is checked against the runtime tree at `../sushiruntime`
(`docs/architecture/`, `include/SushiRuntime/api/`) as of 2026-07-28.

#### The runtime was designed for this workload

Worth stating first, because it changes how much of this plan is speculative. SushiRuntime's own
engineering plan (`sushiruntime/docs/design/SIMULATION_ENGINE_SUBSTRATE_PLAN.md`) records the
owner's locked decisions, and four of them name this system directly:

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

**2. `DynamicGraph` — regions that stream in and out.** Regions are keyed by a caller-chosen
identity; `region(key)` records with the same `add()` surface, `drop(key)` removes one, and
mutations apply **at a step boundary, never against a running DAG**. A commit costs *O(changed
region + affected boundary pins)*, and `compose_count()` advances once per mutation, not per region.
Physics maps **one region per island**: a sleeping island is dropped, a waking one composed. That is
how §13.2's "a settled island costs nothing" is actually implemented.

The constraint to design around: **cross-region edges are derived in ascending region-key order —
the lower-keyed region is the producer.** Island keys must therefore be assigned deterministically
and, where two islands genuinely share an allocation, ordered intentionally. Islands are disjoint by
construction, so the common case has no cross-region edges at all; the shared static-geometry
hierarchy is the exception and is read-only, and read/read sharing stays parallel.

**3. Sub-region dependency tracking.** The tracker keys on `ResourceRegion{base, offset, length}`
and orders by **overlap**, not pointer equality: *"disjoint sub-regions stay parallel"*.
`Buffer::region(ElementRange)` names a slice, and `Reads(...)`/`Writes(...)` accept it in place of a
whole buffer. So the per-colour constraint views become slices of **one** constraint allocation
rather than one allocation per colour (today's `constraint_buffers_` vector), and two colours
writing disjoint slices are still parallel.

**4. Residency, and reading back.** `Residency::Device` allocates device-only USM behind the same
handle API. The hot state columns become device-resident, which is what stops the per-substep host
traffic in §1.2 item 11. The price is explicit and worth stating: **`operator[]` and the bulk
`host()` span throw `invalid_access` on a device-resident handle** — the host uses `read_range()` /
`write_range()`, which copy a sub-range through the owning queue. Every host-side loop in today's
physics has to go, not be ported.

**5. `add_host()` — native CPU nodes inside the graph.** A task that submits no device work runs
directly on the worker thread and reports an empty event, so its successors release through the
ordinary path. The sequential stages (island building, sleep transitions, level-of-detail selection)
stay **inside** the one composition instead of splitting the tick into several `run()` calls.

#### Two runtime constraints that are easy to get wrong

- **The fluent API only tracks dependencies through handle-based overloads.** The runtime cannot
  infer what a kernel touches — a per-element callable is `void(std::size_t)` capturing raw USM
  pointers — so every physics node must name its data with `In`/`Out`/`InOut` or `Reads`/`Writes`.
  The runtime now spells the dependency-blind overloads `add_untracked` (§18, R5), so the rule this
  plan needs becomes a mechanical one: **`add_untracked` does not appear anywhere under
  `physics/`**. Every launch shape has a tracked overload, so there is never a reason for it to.
- **One node's buffers must share one device context.** USM is context-bound; the scheduler verifies
  co-location on a node's first dispatch and fails the run with `invalid_graph` if a node's buffers
  straddle a boundary. All of a scene's physics allocations are pinned to one `DeviceIndex`.
  Multi-device physics is a domain-decomposition problem the runtime defers to its own WP-5, so this
  plan targets one device per scene and says so rather than discovering it later.

#### The rule that follows: one `run()` per tick, not one per substep

`Graph::run()` blocks. The runtime is a throughput engine with no asynchronous step, so 32 substeps
implemented as 32 blocking `run()` calls is 32 round trips per tick, and at a 60 Hz tick that
latency dominates everything else in this document.

**The whole substep loop is unrolled into the graph** — predict, the colour sweep, derive, velocity
pass, repeated `substeps` times — and the tick issues exactly **one** `run()`. `XpbdSolver` already
does this for its *iterations*; the plan extends it to the substeps and to the stages around them.
The state-derived substep count (§6.2) becomes a `when()` predicate on the trailing substeps' nodes,
so varying it does **not** recompose the graph: the nodes exist for the maximum substep count and
the surplus ones are disabled that tick.

Two consequences worth stating plainly:

- Anything the host must see *between* substeps has to move onto the device, because there is no
  cheap point to look. That is a feature: it forces the design that was correct anyway.
- The graph is composed per **structure**, and structure changes are budgeted and reported through
  `compose_count()`. A `compose_count()` that climbs every tick is a bug, and a test asserts it does
  not — the same way `XpbdSolver::compile_count()` is already asserted in the existing tests.

#### Runtime configuration the physics scene must set

Three settings, each with a reason:

- **`Runtime::rebalancer(false)`.** The thermal rebalancer is a background thread on a ~5 ms
  heartbeat that migrates tasks mid-run. The runtime's architecture doc names the case
  exactly: *"Disable it for a low-jitter real-time frame or a deterministic/replay run."* For a 60
  Hz physics tick the reason is **jitter**, not correctness — the runtime's WP-4 analysis is that
  work stealing and migration do not change results as long as no accumulation depends on schedule
  order (below). Off for physics, either way.
- **Profiling on only when asked.** `RunReport::node_timings` / `worker_timings` are populated only
  when the runtime was created with `RuntimeConfig::profiling`; with it off, the dispatch hot path
  reads no timestamps. `PhysicsStatistics` (§13.3) reports what it can either way and the per-node
  breakdown appears when the profiler panel is open.
- **One `DeviceIndex` for the whole scene**, per the co-location constraint above.

#### Deterministic reductions: the runtime provides them

The runtime's WP-4 is explicit that on a single architecture determinism reduces to *"making every
order-sensitive reduction use a fixed combination order"*, and equally explicit about the hard
case: *"GPU reductions are the hard case — a device `reduce` is not guaranteed fixed-order.
Deterministic state-affecting reductions on GPU must use an explicit fixed-order kernel."*

This plan was written when that primitive did not exist and assumed the physics layer would build
it. It now exists (§18, R2): `Graph::add_reduce` and `Graph::add_segmented_reduce`, guaranteeing
that the combination order is a function of the element layout alone — not of the worker count, the
device, the steal pattern, or the work-group size. The segmented form is precisely the shape §12.2
needs: one work-item folds one segment left to right, which is per-body contact-impulse accumulation
and per-vertex accumulation across the tetrahedra sharing a vertex.

So **§12.2's fixed-order accumulation is a call, not a deliverable**, and it leaves P0's scope. What
does not leave is the *conformance requirement*: the byte-equality test with the worker count varied
(§15.5) still has to pass, because a fixed-order primitive used through an order-sensitive call site
is still an order-sensitive result. The one thing to watch is load balance — a segment with ten
thousand contributions and a segment with one cost one work-item each, which is the price of the
guarantee and is the caller's layout problem, i.e. ours.

#### Cooking on the runtime, optionally

The cooker is host code, but its two heaviest stages — voxelization and signed-distance-field baking
— are embarrassingly parallel grid problems. Since SushiRuntime resolves a CPU/OpenCL backend when
no GPU toolchain is present (`SR_GPU_BACKEND=auto` falls through to `cpu`), the cooker may dispatch
those two stages through the runtime when one is available, behind the same `ICookingStage` seam,
with a host implementation as the reference. This is an optimization with a conformance test (§4.4),
not a dependency: **cooking must work with no device present**, because an importer that needs a GPU
is an importer that fails on a build machine.

#### Keeping the seam thin — and honouring a decision already taken

The runtime's API is explicitly unstable and this plan puts weight on it, so: **exactly one adapter
names `SushiRuntime::`.** A `Physics::RuntimeGraphBuilder` behind `IConstraintSolver` (§3.3) owns
every `Buffer`, `Graph`, `DynamicGraph`, `Dynamic`, `Reads`/`Writes`, and `Extent` in the physics
layer. No constraint, no shape, no cooker, and nothing in `physics/scene` includes a runtime header.
When the runtime API moves, one file moves with it, and the conformance suite proves the behaviour
did not.

This is not a new preference. The runtime's substrate plan records it as a locked owner decision —
**L12**: *"Claude builds the domain layers (ECS + physics) as separate, zero-coupling libraries (no
runtime header in them)."* Today's physics layer does the opposite:
`engine/domain/physics/include/SushiEngine/physics/scene/physics_world.hpp`,
`engine/domain/physics/include/SushiEngine/physics/solver/xpbd_solver.hpp`, and
`engine/world/simulation/include/SushiEngine/simulation/physics_simulation.hpp` all include
`SushiRuntime.h` directly, and `XpbdSolver` holds `Buffer` and `Graph` members in its own class
body. So the adapter is not an improvement someone thought of — it is a **restoration** of a
decision the codebase drifted from, and P0 is where that drift is paid back.

---

## §7 Collision and the penetration contract

This section is the direct answer to *"penetrasyon çok önemli — rigid ve soft farketmeksizin, hem
mesh hem fiziksel olarak."*

### 7.1 Broadphase

`SweepAndPruneBroadphase` (today's, fixed: incremental, three-axis, no per-call allocation) is kept
as the reference implementation and the small-scene fast path. The production path is
`BoundingVolumeHierarchyBroadphase`: a dynamic bounding-volume hierarchy with

- **fat bounds** (an inflation proportional to velocity), so a body only re-inserts when it leaves
  its fattened box — most bodies do nothing most ticks,
- **refit on move, rebalance on budget**, never a full rebuild,
- a **persistent pair cache** producing `added` / `persisted` / `removed` pair streams, which is
  what lets manifolds and their warm-start impulses survive across ticks,
- **swept bounds** for bodies flagged for continuous collision, so the pair exists before the
  impact.

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
- **Signed-distance field vs anything**: sample the field at the query point, take the gradient as
  the normal. `O(1)`, exact to the field's resolution, and the *right* answer for deep penetration
  where hull-based methods degrade. This is the primary path for soft-body vertices against rigid
  bodies and for concave static geometry.
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
for the rocking box in §1.2 item 3. Points are matched to the previous tick's by `feature_id`, so
their `lambda` values carry over: **warm starting**, which is what makes a stack of ten crates
converge in one iteration per substep instead of never.

### 7.4 Friction and restitution

Per Müller et al. 2020, and this is the recipe the implementation follows exactly:

- **Positional pass, per contact point:** apply the normal correction Δλ_n; then compute the
  tangential relative displacement of the contact anchors since the substep began and apply a
  tangential correction, **clamped to the static-friction cone** `λ_t ≤ μ_static λ_n`. Static
  friction that is positional is what lets a box sit still on a ramp instead of creeping.
- **Velocity pass, per contact point:** dynamic friction as an impulse
  `p_t = -t̂ · min(|v_tangent| / w, μ_dynamic |λ_n| / h)` — *both* terms are impulses: the first is
  what would halt the slide outright, the second is what the normal impulse can pay for. Writing the
  minimum as `min(μ_dynamic |λ_n| / h, |v_tangent|)` reads naturally and compares an impulse against
  a speed; the deceleration then comes out wrong by the generalized inverse mass `w`, which for a
  box's corner contact is a factor of four. Then restitution
  `Δv = n · (-v_normal + max(-e · v_normal_before_solve, 0))`, with restitution **suppressed** when
  `|v_normal| < 2 g h` — the standard anti-jitter threshold that stops resting bodies from buzzing.
  The restitution term runs whenever the point carried a normal impulse, **not only when `e > 0`**:
  at `e = 0` it reads "leave this contact at zero closing speed", which is what removes the velocity
  the positional depenetration would otherwise inject (a body that sank in over a tick is pushed out
  in a substep, and `update_velocity` reads that push back as metres per second of real motion).
- Materials combine by their declared combine mode (§5.3).

### 7.5 Continuous collision — nothing tunnels

Three tiers, cheapest first, chosen per body by its flags and motion:

1. **Speculative contacts (default, always on).** Generate contacts out to `contact_offset + |v·n|h`
   with a *positive* target separation. The solver treats an approaching body as constrained not to
   pass the surface, so the vast majority of fast motion never needs a sweep at all. Cost: near
   zero.
2. **Conservative advancement** for bodies whose per-substep motion exceeds a fraction of their
   thinnest dimension: iteratively advance to the earliest time of impact using the closest-distance
   query, then solve at that time. Cost: a few closest-point queries for the few bodies that need
   it.
3. **Substep escalation** for the pathological case (a bullet, a wheel at speed): the island's
   substep
   count is raised locally. Deterministic, because the trigger is a state-derived threshold.

Soft bodies get the continuous treatment differently (§9.6): vertex-triangle and edge-edge
continuous
tests with a thickness, per Bridson.

### 7.6 The visible-penetration contract, made concrete

Two per-shape distances, PhysX's model, and the reason it is in this plan explicitly:

- **`contact_offset`** — contacts are *generated* at this separation. Larger means earlier detection
  and more stable stacking. Default: a small fraction of the shape's size.
- **`rest_offset`** — contacts are *resolved* to this separation. Zero means surfaces come to rest
  exactly touching. A small positive value keeps a visible sliver of air; a small negative value
  lets a tyre visibly deform into the ground.

The **enforced invariant**, checked by a regression test rather than asserted in prose: in the
standard stacking, ramp, and vehicle-landing scenes, the maximum measured penetration of any contact
at rest does not exceed `rest_offset + tolerance`, and no contact's penetration exceeds a hard
*depenetration budget* mid-motion — a maximum depenetration velocity clamps recovery so a
deeply-overlapping spawn pushes apart smoothly instead of exploding.

For **mesh accuracy**, the cooker reports the one-sided Hausdorff distance between the render mesh
and its cooked collision geometry, in metres, in the inspector. "The collider is 3 cm fatter than
the visible mesh along the wheel arch" becomes a number the artist can see and fix by raising the
convex piece budget — instead of a mystery gap at runtime.

For **soft bodies**, §0.4's second mechanism removes the problem at the root: there is no second
geometry. The render mesh is the embedded interpolation of the simulated mesh, and the simulated
surface is what collided.

### 7.7 Queries

`ICollisionQueryService`: `raycast_closest`, `raycast_all`, `sweep`, `overlap`, `closest_point`,
each taking a filter (layer mask plus an optional predicate). Built on the same broadphase tree, so
a query costs a tree descent, not a scan. Triggers are shapes flagged `Trigger` that generate events
and no impulses.

---

## §8 The cooking pipeline — a mesh goes in, a simulation asset comes out

This is the user's headline feature: *"mesh'i projeye attıktan sonra otomatik olarak soft body
optimized
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

The inspector shows what the dial *produced* — tetrahedron count, cook time, estimated per-tick
cost,
memory, Hausdorff error — so the trade-off is visible rather than felt three weeks later.

### 8.3 The soft-body cooker, stage by stage

Each stage is an `ICookingStage`; the pipeline is a list.

1. **Repair.** Weld duplicate vertices, drop degenerate triangles, orient consistently, compute a
   connected-component report. Report — do not silently fix — a non-manifold or non-watertight
   input, because the artist needs to know.
2. **Voxelize.** Rasterize the surface into a grid at the fidelity-derived resolution, then
   flood-fill the exterior from a corner. Everything not reached is interior. This is robust to the
   meshes real projects actually contain: self-intersecting, non-manifold, open-shelled. **Choosing
   voxelization over direct constrained Delaunay tetrahedralization is deliberate** — Delaunay is
   more accurate on clean input and fails on dirty input, and dirty input is the common case.
3. **Tetrahedralize.** Fill the interior with a **body-centred cubic lattice** (each cell yields
   tetrahedra that tile without slivers), then conform the boundary: cells straddling the surface
   are split by marching tetrahedra against the surface's distance field, and boundary vertices are
   snapped onto the surface within a tolerance. This is the Houdini/Blender approach — predictable
   element quality, resolution driven by one number, no failure mode that produces zero output.
4. **Optimize.** Remove tetrahedra below a minimum-volume or minimum-dihedral-angle threshold
   (slivers destroy the conditioning of a finite-element solve), smooth interior vertices toward
   their neighbourhood centroid, and re-check that no element is inverted. Report the worst element
   quality.
5. **Compute rest state.** Per tetrahedron: the inverse rest-shape matrix `Dm⁻¹`, the rest volume,
   and the mass distributed to its four vertices from the material density. Per vertex: the summed
   inverse mass. Also compute the whole body's mass, centre of mass, and inertia tensor (for the
   rigid approximation the level-of-detail system falls back to).
6. **Embed the render mesh.** For every render-mesh vertex, find the containing tetrahedron and
   store its index plus four barycentric weights. Vertices outside every tetrahedron (thin features
   that fell through the lattice) bind to the nearest tetrahedron by the *extrapolated* barycentric
   coordinates, which keeps them attached and moving correctly. This table **is** §0.4's guarantee.
7. **Build the surface set.** Extract the tetrahedral mesh's boundary triangles — this is the soft
   body's collision surface — and build its bounding-volume hierarchy.
8. **Bake the rest distance field.** A narrow-band signed-distance field of the rest shape, used for
   fast "is this point inside me" queries during collision and for the self-collision broad pass.
9. **Build levels of detail.** Coarser lattices at halved resolution, plus a barycentric mapping
   from each level to the next finer one, so a distant body simulates 500 tetrahedra and the render
   mesh is still driven correctly through the chain. The coarsest level is a shape-matching cluster
   set; below that, the body falls back to its rigid approximation.
10. **Serialize.** One `.sushisoft` blob, with the cooking parameters recorded inside it so a
    re-cook
    is reproducible and a mismatch is detectable.

### 8.4 The collision cooker (rigid)

1. **Mass properties** by exact polyhedral integration over the closed mesh (Mirtich's method) —
   mass, centre of mass, full inertia tensor, then an eigendecomposition to principal axes so the
   runtime's diagonal `inverse_inertia` is *correct* rather than authored by hand (§1.2 item 6).
2. **Approximate convex decomposition** into at most `N(fidelity)` pieces, each simplified to a
   vertex
   budget, with the volumetric error and the Hausdorff error reported.
3. **A narrow-band distance field** of the whole shape, for deep-penetration recovery and
   soft-vertex
   queries.
4. For geometry authored as **static**, skip decomposition: cook a triangle-mesh bounding-volume
   hierarchy instead, which is both cheaper and exact.
5. **Serialize** to `.sushicollision`.

### 8.5 Validation, because a silent bad cook is the worst outcome

The cooker emits a `CookingReport` — element count and worst quality, watertightness, inverted
elements, unembedded render vertices, Hausdorff error, estimated per-tick cost, memory. The editor
shows it, and a cook that violates a configured threshold **fails loudly** rather than shipping a
body that explodes on the first frame.

### 8.6 The mesh binding, end to end

The connection between a mesh and its physics is the thing this whole system is built to get right,
so it is written out as one continuous path rather than left implied across four sections. Today the
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
`render/geometry/cloth_buffers.cpp` (`docs/architecture/domain-physics.md` §1.2). A soft body is the
same channel with a triangle topology that comes from the asset instead of from a grid, so the plan
**generalizes `ClothStrandView` into a `DeformableMeshView`** rather than adding a third parallel
path. Cloth becomes one kind of deformable mesh, not a special case — the same consolidation §4.2
asks for everywhere else.

The vertex deformation itself (applying the embedding) is a per-vertex parallel kernel and runs in
the same runtime graph as the solve (§6.6), so the deformed positions are produced device-side and
handed to the renderer without a host round trip.

**Normals** are recomputed from the deformed positions — per-face, then area-weighted per-vertex —
in the same kernel, because a dented panel that keeps its rest normals looks undented no matter how
accurate the simulation is.

**The invariants, each with a test (§15):**

1. Every render vertex is bound. Unbound vertices are counted at cook time and the count is
   reported;
   a non-zero count above a threshold fails the cook (§8.5).
2. At rest, the reconstructed render mesh reproduces the source mesh within a stated tolerance — the
   embedding round-trips.
3. A deformation applied to the simulated mesh is visible in the render mesh in the *same* tick;
   there is no lag, because there is no synchronization step to lag.
4. Fracture (§9.5) preserves binding: a duplicated simulation vertex inherits its parent's binding,
   so a crack does not tear a hole in the render mesh.
5. Collision happens against the simulated surface, so §0.4's contract holds by construction rather
   than by convention.

---

## §9 Soft bodies

### 9.1 The model

Two XPBD constraints per tetrahedron (Macklin & Müller 2021), replacing today's distance lattice:

- **Deviatoric** (resists shape change): `C_deviatoric = ‖F‖_Frobenius − √3`, compliance
  `1/(μ·V_rest)`.
- **Hydrostatic** (resists volume change): `C_hydrostatic = det(F) − 1 − μ/λ`, compliance
  `1/(λ·V_rest)`.

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

Every element's **Green strain tensor** and **Cauchy stress** fall out of the deformation gradient
the solver already computed; the **von Mises equivalent stress** is one scalar per tetrahedron. It
costs almost nothing because `F` is already in hand, and it delivers:

- a **stress heat map** debug view (blue → red over the body) — the thing that makes "is this beam
  strong enough" answerable rather than guessable,
- the trigger for plasticity (§9.4) and fracture (§9.5),
- a gameplay query: `ISoftBodyService::maximum_stress(entity)` and per-region aggregates, so a game
  can ask "how badly is this damaged."

For **rigid** assemblies, the analogue is the joint-force readout in §10.4 — the same question,
asked
of a constraint instead of an element.

### 9.4 Plasticity — the permanent dent

Multiplicative decomposition: the total deformation `F` splits into an elastic part and a plastic
part, and the cooked rest matrix `Dm⁻¹` is what stores the plastic part. Each substep, per element:

```
  if (elastic strain measure > yield_stress-derived threshold)
      advance the plastic deformation toward the current shape by plastic_creep
      clamp the accumulated plastic strain to maximum_plastic_strain
      write back the updated rest matrix and rest volume
```

Because the plastic state lives in the rest configuration, a dented panel *stays* dented, springs
back elastically around its new shape, and needs no separate damage system. This is the mechanism
behind car-body deformation in §11, and it is a dozen lines inside the element projection.

### 9.5 Fracture

When an element's von Mises stress exceeds `fracture_stress`, the element is removed and its shared
vertices are duplicated along the crack surface, splitting the topology. This requires the mutable
world of §6.4 — constraints removed, vertices added, colouring updated incrementally — and it
requires the render embedding to follow, which it does, because embedding is per-vertex and a
duplicated vertex inherits its parent's binding.

Guard rails: a per-tick fracture budget (deterministic, state-derived), a minimum fragment size, and
a scene-level cap. Uncapped fracture is how a physics engine dies.

### 9.6 Soft-body collision — three problems, three answers

1. **Soft vs rigid.** Surface vertices query the rigid body's cooked signed-distance field: `O(1)`
   per vertex, exact depth, exact gradient normal, and correct at any penetration depth. This is
   *the* reason §8.4 bakes a distance field for every rigid collider. Two-way: the correction's
   reaction is applied to the rigid body weighted by generalized inverse mass, so a soft body
   actually pushes back.
2. **Soft vs soft.** Broad: each body's bounding-volume hierarchy against the other's. Narrow:
   vertex-triangle and edge-edge with a thickness, continuous (Bridson) for thin bodies, discrete
   for thick ones.
3. **Self-collision.** A spatial hash over the surface vertices at a cell size of the collision
   thickness, excluding topological neighbours, with the continuous vertex-triangle and edge-edge
   tests. Off by default (it is the expensive one), enabled per body, scheduled in the phase where
   it can be measured rather than assumed.

### 9.7 Levels of detail

Cooked in §8.3 step 9, selected at runtime by screen coverage and distance, with hysteresis so a
body does not oscillate between levels. The transition maps the fine state onto the coarse one
through the stored barycentric mapping, so a body does not pop. The coarsest tier is shape matching;
below that, a rigid body with the cooked inertia tensor. `ISoftBodyModel` (§3.3) is the seam that
makes the swap a substitution rather than a special case.

---

## §10 Assemblies and multibody dynamics

The direct answer to *"otomobil şasisi üzerine menteşe ile bağlanmış kapı, mukavemet ve MBD
hesapları."*

### 10.1 The joint library

Every joint is a POD descriptor plus a projection functor, registered per §4.2. Each holds two
**joint frames** — a position and orientation in each body's local space — which is the uniform way
to express "where and how these two bodies are attached."

| Joint | Removes | Authored parameters |
|---|---|---|
| `FixedJoint` | 6 degrees of freedom | Compliance (a "welded but it can flex" seam is just a compliant fixed joint). |
| `BallJoint` | 3 translational | Compliance. The pure spherical joint — see the note below. |
| `HingeJoint` | 3 translational + 2 rotational | Axis, lower/upper angle limits, motor, friction. **The car door.** |
| `SliderJoint` | 3 rotational + 2 translational | Axis, travel limits, motor. **Suspension travel.** |
| `DistanceJoint` | Range along a line | Minimum/maximum, compliance. Already exists, generalized. |
| `ConeTwistJoint` | 3 translational, limits rotation | Swing cone + twist range. **Ragdolls.** |
| `GearJoint` / `RackJoint` | Couples two rotations | Ratio. **Differentials, steering rack.** Deferred to §11.4 — see the note below. |
| `SixDegreeOfFreedomJoint` | Configurable | Per-axis free/limited/locked, with per-axis drives. The general case. |

**The frame convention, fixed once:** a joint frame's **local x axis is its primary axis** — the
hinge's rotation axis, the slider's travel axis, the cone-twist's twist axis. The whole angular
vocabulary is a swing/twist decomposition about one named axis, and a convention that varied per
kind would mean each kind re-deriving which of three axes it meant.

**Limits** are inequality constraints projected only when violated. `lower == upper` locks the axis
and a disabled limit leaves it free, so free/limited/locked are three readings of one range rather
than a mode word that could disagree with the numbers. **Motors** are target-position or
target-velocity drives with a maximum force; the position drive is a positional row so its stiffness
is step-size independent like everything else here, and the rate drive is velocity-level because a
rate is not a position. A rate drive with a target of zero and a small force limit *is* joint
friction — which is how §10.2's "it does not swing free" is expressed.

**Two entries above were refined when the library was built (§16.8).** A ball joint *with* limits is
a cone-twist, so `BallJoint` carries none: two kinds differing only in whether an author remembered
to enable a limit are two names for one thing. And `GearJoint`/`RackJoint` couple two *accumulated*
rotations, which needs an unwrapped angle carried across ticks that no other joint needs — and §10.5
already rules that a drivetrain is solved as an independent one-dimensional chain, so the gear
belongs with the powertrain rather than in the three-dimensional solver.

### 10.2 The assembly asset

`PhysicsAssembly` (§5.4) is authored in the editor and instanced atomically: parts become bodies,
joints become constraints, collision-filter groups stop the door from colliding with the chassis it
is attached to. One entity carries the `AssemblyInstance`; child entities carry the parts, so the
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

A joint's endpoint may be a rigid body **or** a soft-body vertex set (an "attachment", which
averages the correction across a small vertex neighbourhood so it does not tear a single vertex out
of the mesh). This is what mounts a rigid hinge to a deformable panel — the exact case a real car
door is — and it is the same generalized-inverse-mass split the engine already performs.

### 10.4 Multibody quantities — reading forces back out

XPBD gives Lagrange multipliers, not forces. The recovery is exact and cheap:

```
  force  = λ · direction / h²          torque = λ_angular · axis / h²
```

accumulated per joint per substep and exposed as `IJointService::joint_state(id)`. That single
quantity delivers: break thresholds, the "how much load is this mount carrying" readout that is the
rigid-body half of *mukavemet*, motor-effort feedback for a drivetrain, and a diagnostic overlay
showing which joint in an assembly is the one about to fail.

**Two accumulations, not one, and the distinction decides whether break thresholds work at all.**
The *mean* over the tick's substeps — a vector — is the load readout: which way and how hard the
mount is being pulled. The *peak* single-substep magnitude is what a break threshold reads. §16.8
records why: an impact is a large correction followed by an almost equally large one the other way,
so the mean of a hard hit is nearly its resting load while the substeps either side of it carry a
thousand times that. Averaging a load whose direction reverses measures the net pull, and what tears
a mount out is the magnitude.

### 10.5 When XPBD is not enough

A drivetrain has mass ratios in the thousands (a crankshaft against a vehicle) and joints that must
be exactly rigid. Substepping handles a great deal of it, but not all. Two escape hatches, in order
of preference:

1. **Solve the stiff sub-chain in one dimension.** A powertrain is a chain of rotational inertias,
   not a spatial mechanism. Simulating it as an independent one-dimensional multibody system (§11.4)
   and coupling it to the wheels through a torque constraint is both cheaper and more accurate than
   forcing it through the three-dimensional solver.
2. **A reduced-coordinate articulation path** (Featherstone) behind `IConstraintSolver`, for chains
   that genuinely need it. Kept as a documented option, not a phase-one commitment, because it
   doubles the solver surface and every AAA engine that shipped both regrets the maintenance.

---

## §11 Vehicles — the BeamNG evolution

The user's stated destination. This phase is late in the roadmap because it composes everything
before
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
every gram of the car is in the solver, and handling quality depends on beam tuning that takes
months. The structure is:

- The **chassis core** is a rigid body carrying the bulk of the mass and the inertia tensor.
  Handling is
  stable, tunable, and cheap.
- The **shell** (panels, bumpers, doors, bonnet) is node-beam or FEM soft, attached to the core by
  the §10.3 attachment constraint. It deforms, dents permanently, and tears off.
- **Suspension** is joints and drives (§10.1), not beams — a slider joint with a spring-damper drive
  is more controllable and more stable than a beam network, and it is what every shipping racing
  game does.
- **Wheels** are rigid bodies with a tyre force model (§11.5).

The pure node-beam path stays open, because `VehicleInstance` names a `NodeBeamAsset` and an asset
whose rigid core is empty *is* a pure node-beam vehicle. **The architecture does not choose; the
asset does.**

#### What BeamNG got wrong, and what we do instead

BeamNG is the reference for what a deformable vehicle should *feel* like. It is also a decade-old
design with known structural limits, and the point of arriving later is to not inherit them. Each
row is a real limitation of that architecture and the specific mechanism in this plan that avoids
it.

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
a chassis that twists under load, a structure whose failure mode was never authored. The hybrid
keeps that reachable (an empty rigid core), and §17.4 records the open question of the core-to-shell
seam, which is where visual artifacts will show up first.

### 11.3 The node-beam cooker

`NodeBeamCooker` turns a vehicle mesh plus an authored structure into a `NodeBeamAsset`: place nodes
at mesh features or on a lattice at the fidelity-derived resolution, connect them with structural
beams, add bracing beams by a diagonal rule, distribute mass by density, and skin the render mesh
onto the node cloud with distance-weighted blending. Manual authoring stays possible — the asset is
data, and a hand-authored node-beam file is as valid as a cooked one.

### 11.4 Powertrain

A one-dimensional multibody chain, simulated inside the physics tick at the same substep rate:
engine torque curve and inertia → clutch (a torque-limited coupling) → gearbox (a ratio) →
differential (a `GearJoint`-style constraint splitting torque between outputs) → axle → wheel
angular velocity. The coupling to the three-dimensional world is a torque applied to each wheel body
and the reaction on the chassis. §10.5's first escape hatch, applied.

### 11.5 Tyres

A slip-based force model (a Pacejka-style magic-formula curve, or a simpler Brush model as the first
implementation) evaluated per wheel per substep: compute longitudinal and lateral slip from the
wheel's contact patch velocity, look up the force curve, apply the force at the contact point.
Combined-slip handling by the friction ellipse. Load sensitivity from the contact normal force the
solver already recovered.

The alternative — a node-beam tyre with pressurized volume, which is what BeamNG actually does — is
a soft body with a volume constraint over its interior surface (§9.1's hydrostatic constraint,
applied to a closed shell). It is genuinely better and genuinely much more expensive. Both are
supported by the same asset structure; the vehicle asset chooses.

### 11.6 Aerodynamics — a cross-system tie-in

The engine already has a full atmosphere and wind system
(`engine/world/simulation/include/SushiEngine/simulation/weather_wind.hpp`,
`engine/world/simulation/include/SushiEngine/simulation/atmosphere_forcing_buffer.hpp`,
`docs/design/atmosphere_system.md`). Vehicle drag and downforce, and cloth and rope wind response,
sample it through a `WindSampler` seam that mirrors the existing `GravitySampler` exactly (§4.5) —
the physics names the abstraction, never the meteorology behind it. A flag on a pole in a storm and
a car's high-speed lift come from the same field.

---

## §12 Determinism, precision, and networking

### 12.1 The rules, restated as physics requirements

1. **Body order is stable.** Bodies are iterated by handle index, never by hash-container order.
   Where the boundary layer keeps an `std::unordered_map<EntityId, BodyHandle>` (as it does today),
   any iteration whose *order is observable* is replaced by a sorted index vector.
2. **Contacts are sorted before they are solved**, by `(body_a, body_b, feature_id)`. Broadphase
   output order is an implementation detail and must never reach the solver.
3. **Colour order is fixed** and the incremental recolouring is deterministic — the greedy rule
   already
   is; the incremental version must produce the same colouring as a full recolour would, or
   explicitly
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
   slot list followed by an in-order segmented tree reduce — never floating-point atomics and never
   a device `reduce` whose combination order is unspecified. Since the runtime does not ship this
   primitive (§6.6), the physics layer builds it, as a P0 deliverable.
2. **Schedule-order leakage into state.** No "first writer wins" race, no non-commutative in-place
   update whose result depends on which sibling ran first. The region-overlap tracker makes a
   genuine write-write conflict an explicit dependency rather than a silent race, which is what
   makes this auditable at all.
3. **Random number generation.** Per-body and per-element streams seeded by stable identity, never
   by a global counter touched in schedule order. Nothing in this plan needs randomness today; the
   rule is recorded so nothing introduces it carelessly later.

The rebalancer is still switched off for physics (§6.6) — for frame-time jitter, not for
correctness. The distinction matters: turning it on would not make the simulation wrong, it would
make the frame time unpredictable.

### 12.3 Snapshot and rollback

The physics state that must be snapshottable per tick: body state columns, soft-body vertex state,
plastic rest matrices, joint multipliers and break flags, sleep state, and the warm-start
accumulators. All of it is already a pointer-free column, which is what makes SushiLoop's
dirty-chunk snapshot (`docs/design/SUSHILOOP.md`) applicable without a special case. The **cooked
assets are immutable and shared** and are never snapshotted — only the handles.

Rollback correctness gets its own test: simulate N ticks, snapshot at tick K, roll back, replay, and
assert byte equality of the whole physics state (§15.4).

---

## §13 Performance and scale

### 13.1 Targets

Stated so they can be measured and failed, not as aspiration. Reference: one desktop-class GPU
through
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
   accumulators, and the colouring all survive across ticks. Today's code rebuilds all of them,
   twice per substep.
3. **Collision once per tick, not per substep** (§6.1).
4. **Device-resident solve, one `run()` per tick.** The constraint solve is already on the device;
   the plan moves the broadphase, the narrowphase, the contact solve, and the
   predict/derive/velocity stages there too, and unrolls the whole substep loop into a single graph
   composition so a tick is one blocking call rather than 32 (§6.6). Structure-of-arrays layout for
   the state columns, since a projection touches four of twelve fields. Sleeping islands are dropped
   from the `DynamicGraph` rather than run and discarded.
5. **Levels of detail.** Distance-based soft-body tiers (§9.7); a distant vehicle's shell freezing
   to its rigid core; a coarse substep count for distant islands.
6. **Budgets.** Per-tick caps on contacts, fracture events, and continuous-collision escalations,
   all state-derived, all reported when hit — a physics engine that silently exceeds its budget is
   how a frame-time spike becomes a mystery.

### 13.3 Instrumentation

`PhysicsStatistics` per tick: awake/sleeping body counts, island count and largest island,
broadphase pairs tested and produced, manifolds and contact points, constraints per colour, substeps
taken, continuous-collision escalations, fracture events, and per-stage timings — wired into the
existing GPU profiler (`engine/presentation/render/source/graph/gpu_profiler.hpp`) for device stages
and an editor panel for the rest. Nothing in §13.1 is verifiable without it, so it is a P0
deliverable, not a P8 one.

---

## §14 The editor and authoring surface

Following the established pattern from `editor-component-inspector-pattern`: panels edit the
**selected
entity's** component, and previews get their own viewport.

- **Physics Material** — a project asset with a preview (a ball dropped on a ramp at the authored
  friction and restitution).
- **Collider inspector** — primitive or cooked asset; shows the cook report (convex piece count,
  Hausdorff error, mass, inertia) and a "Re-cook" button; draws the actual collision geometry as an
  overlay so "the collider is not the mesh" is *visible*.
- **Soft Body inspector** — the asset, the **fidelity slider**, the material, a "Bake" button with
  progress, and the cook report (tetrahedra, worst element quality, unembedded vertices, estimated
  cost). Debug views: wireframe tetrahedra, stress heat map, plastic-strain heat map.
- **Assembly editor** — the parts list, the joints list, joint gizmos (an axis and an arc for a
  hinge's
  limits, drawn in the viewport and draggable), collision-filter matrix, and a live joint-load
  readout
  while playing.
- **Vehicle editor** — node/beam visualization, group colouring, mass distribution readout,
  powertrain
  curve editing.
- **Physics debug draw** — contacts and normals, manifold points, island colouring, sleeping state,
  broadphase bounds, query results. Toggled per category.
- **Physics profiler panel** — §13.3's statistics over time.

`.sushiscene` serialization extends for each new component, following the existing
independent-field-pair
convention (`docs/architecture/domain-physics.md` §1.1).

---

## §15 Testing strategy

Extending the existing `tests/{unit,integration,regression}` layout and `se test --suite
functional`.

1. **Unit — geometry and math.** Every narrowphase routine against analytic answers, including the
   degenerate cases §1.3 names. Mass properties against closed-form results for a box, sphere, and
   cylinder. Tetrahedralization invariants: no inverted elements, every render vertex embedded,
   volume preserved within tolerance. Distance-field sampling against brute-force triangle distance.
2. **Unit — solver.** Each constraint projection against a host mirror, exactly as
   `tests/integration/test_xpbd_solver.cpp` does today. Convergence: N substeps must reduce the
   residual by the expected order.
3. **Integration — physical correctness.** These are the tests that make the difference between "it
   runs" and "it is right":
   - A ball with restitution `e` dropped from height `h` returns to `e²h` within tolerance.
   - A box on a ramp begins to slide exactly at the angle `atan(μ_static)` — the friction test that
     today's engine would fail outright.
   - A stack of ten crates is stable and does not drift for 10 000 ticks.
   - A pendulum's period matches the analytic value; energy drift over 10 000 ticks stays bounded.
   - A hinge with limits does not exceed them; a motor reaches its target; a break threshold breaks
     at the right load.
   - A cantilever beam's tip deflection under load matches Euler–Bernoulli theory within the
     tolerance
     the element count justifies. **This is the *mukavemet* test** — it proves the FEM material
     parameters mean what they say.
   - A soft body under sustained load past its yield stress keeps a permanent deformation of the
     predicted magnitude.
4. **Regression — the penetration contract.** Golden-metric scenes measuring maximum penetration at
   rest and in motion, tunneling at escalating speeds (a 200 m/s sphere through a 1 cm plate must
   not pass), and the render-versus-collision Hausdorff error for a set of reference meshes.
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

Each phase is independently shippable, ends green under `se build` / `se test --suite functional`,
and has an acceptance criterion that is a **test**, not an opinion. The status column is the single
place progress is recorded.

| Phase | Deliverable | Acceptance | Status |
|---|---|---|---|
| **P0** | **Foundations.** Neutral `geometry/` module + the distance-baker move (§3.4). `physics/` restructured into §3.1's modules. `IPhysicsSimulation` split per §4.3. **`RuntimeGraphBuilder`: the single adapter naming `SushiRuntime::`, the solver migrated to the `Dynamic` `add()` form, the substep loop unrolled into one composition, predict/derive moved off the host, device residency for the hot columns (with the four-byte count slots left `Shared`), the rebalancer switched off, and the accumulation paths wired to the runtime's segmented fixed-order reduce rather than a hand-built one (§6.6, §12.2, §18).** Mutable world with generational handles, fixed-capacity buffers, and incremental recolouring (§6.4). `PhysicsMaterial`, body flags, `center_of_mass_local`. Deterministic contact ordering. `PhysicsStatistics` + profiler wiring (§13.3). Substepping schedule (§6.1, §6.2). The §1.3 correctness fixes. | Existing tests stay green; a body can be added and removed mid-simulation without a rebuild; **one `run()` per tick and a `compose_count()` that stops climbing**; statistics appear in the editor. | **Complete** — see §16.1 |
| **P1** | **Contact quality.** Persistent manifolds with face clipping and reduction (§7.3), warm starting, static friction in the positional pass, dynamic friction and restitution in the velocity pass (§7.4), `contact_offset`/`rest_offset` (§7.6), contact events. Broadphase made incremental and three-axis, once per tick. | The restitution, angle-of-repose, and ten-crate-stack tests pass. Contact cost drops measurably against the P0 baseline. | **Complete** — manifolds, warm starting and friction; **contacts as a constraint kind inside the solve graph** (§6.3, §16.5); **the live `sim/` tick on one `IConstraintSolver`**; and contact events (§16.6). **The acceptance clause's number is unmeasured:** "contact cost drops measurably against the P0 baseline" is a GPU-timed comparison, and §16.35 records that every §13.1 target still needs a GPU this machine does not have. Built, not measured. |
| **P2** | **Shapes and scale.** Capsule, convex hull with GJK/EPA, static triangle mesh with a bounding-volume hierarchy and edge-normal correction, height field, compound shapes. `Transform::scale` honoured. Islands and sleeping, mapped onto `DynamicGraph` regions (§6.6). Bounding-volume-hierarchy broadphase. Collision filters and layers. Scene queries and triggers (§7.7). | 1 000 mixed-shape bodies at the §13.1 target; 10 000 mostly-sleeping bodies at target; queries return correct hits under a conformance suite. | **Complete** — see §16.4, and §16.10 for the half of it that had been built without being connected: the island partition and sleeping reached only their own unit tests until 2026-07-30. **The acceptance clause's numbers are unmeasured:** the 1 000 mixed-shape and 10 000 mostly-sleeping body counts are §13.1 targets, and §16.35 records that every §13.1 target still needs a GPU this machine does not have. Built, not measured; the conformance-suite half of the clause does pass. |
| **P3** | **Joints, assemblies, MBD.** The §10.1 joint library with limits, motors, and drives. Joint force/torque recovery (§10.4). Breakable joints. `PhysicsAssembly` asset, instancing, and the editor (§14). Ragdoll wired to `Animation::RagdollBlend`. | **The chassis-plus-hinged-door scene works end to end**: the door swings within its limits, carries load, reports its hinge force, and tears off above its break threshold. Joint accuracy tests pass. | **Complete.** The joint library, force/torque recovery, breakable joints and `IJointService` (§16.8); the `PhysicsAssembly` asset, its blob and its instancing, and the ragdoll wired to `RagdollBlend` (§16.9). The §14 assembly editor is the one item outstanding, and §16.10 sizes it honestly: it needs §5.5's `PhysicsJoint` component, its serialization and an `ISimulation` surface for joints before it can be a panel at all, because `ISimulation` deliberately does not expose the physics boundary. **Those three now exist** (PX-1 — see §16.31): the component, an `IWorldEditor` surface including the live load readout, scene serialization that stores the partner as an array index, and a per-step reconcile that keeps an unchanged joint's warm start. It also recovered a field the boundary had been silently dropping — `JointMotorDesc::damping`, without which every drive that reached the solver through `IJointService` was a spring with no damper. **And the panel now exists** (PX-2 - see §16.32): parts, joints against part indices, the collision-filter matrix, a live joint-load list over the scene, and instancing that produces ordinary entities rather than a scene-graph node of its own. **P3 is closed.** |
| **P4** | **The cooking pipeline.** `geometry/` triangle mesh utilities, the import-processor chain, `CollisionCooker` (mass properties, convex decomposition, distance field), `SoftBodyCooker` (§8.3, all ten stages), the fidelity dial, the content-hash cache, `CookingReport`, and the editor bake surface. | **Dropping a mesh into the project produces a `.sushisoft` and a `.sushicollision` without a manual step**, at the authored fidelity, cached, with a report; the cooker invariants of §15.1 hold on a corpus of deliberately dirty meshes. | **Complete** — see §16.11 (foundations), §16.12 (rigid), §16.13 (soft), §16.14 (import chain), §16.15 (bake surface). **94 of 94** tests pass across nine suites, dirty-mesh corpus included. Two things deliberately outside the phase: nothing consumes `.sushisoft` until P6 writes the element solver, and §14's soft-body *debug views* (stress and plastic-strain heat maps) read quantities only that solver produces. |
| **P5** | **Penetration hardening.** Speculative contacts, conservative advancement, substep escalation (§7.5). Signed-distance-field collision as a first-class narrowphase path. Maximum depenetration velocity. The regression scenes of §15.4. | **Nothing tunnels** at the tested speeds; measured resting penetration stays within `rest_offset + tolerance`; the Hausdorff error is reported per asset. | **Complete** — see §16.16 for the build and §16.19's RESOLVED addendum for the last failure. Maximum depenetration velocity, speculative contacts, conservative advancement, and the SDF narrowphase path are all written and tested, and all three §15.4 regression scenes pass — including the tunnelling scene at 200 m/s, which held out longest and turned out not to be a CCD bug at all: the manifold was correct at every speed, and §7.6's depenetration budget was the culprit, correcting 6 mm of a 0.4 m last-substep violation and letting the next tick's nearest-face manifold walk the buried sphere out the far side. The budget now also covers however much the pair closed during the substep, so a spawned overlap still pays out at 3 m/s while a moving body can always be stopped by what it hit. Global substep escalation from the motion maximum already exists in `derive_substep_count`; the **per-island** part stays P8's, not P5's. |
| **P6** | **FEM soft bodies and strength.** The neo-Hookean two-constraint model (§9.1), `SoftBodyMaterial` and presets, stress readout and heat map (§9.3), plasticity (§9.4), fracture (§9.5), soft-vs-rigid and soft-vs-soft collision (§9.6), levels of detail (§9.7), and **the mesh binding of §8.6 driven end to end** — the embedding kernel, deformed normals, and `ClothStrandView` generalized to `DeformableMeshView`. Cloth gains bending. Cosmetic bodies gain the `float` column (§6.5). | **The cantilever-deflection test matches theory**; a body past yield keeps a permanent dent; a fractured body's render mesh follows correctly; 20 000 tetrahedra at the §13.1 target. | **Complete but for one unmeasured number.** The cantilever-vs-Euler-Bernoulli acceptance test **passes** — §16.19 records the three fixes (the deviatoric constraint is `‖F‖`, not `‖F‖ − √3`; the hydrostatic term uses Smith's `λ + μ` reparameterization; the case runs at 60 substeps). §9.1–§9.5 as before, and now §9.6's three collision problems, §9.7's tiers and pop-free transfer, §8.6's embedding kernel and deformed normals, §9.5's vertex duplication along the crack with the binding following it, §9.1's bending constraint, and §6.5's cosmetic `float` column — all written, all with tests. See §16.20. `DeformableMeshView`, `ISoftBodyService` and the editor's debug views closed P6-G2/G3/G5; **P6-J1 generalized the colourer and the constraint store from two body indices to N, and P6-J2 made the FEM element a constraint kind in the device graph**, so a 20 250-tetrahedron lattice now places, colours and steps through `RuntimeGraphBuilder` with zero rejections and zero recompositions. **The one acceptance item still open is the number itself:** §16.21 records **29.4 ms/tick** for that scene at 32 substeps against a 3 ms budget — but on the **CPU backend**, because SushiRuntime finds no GPU device on the machine it was run on, and §13.1's target is written against a desktop GPU. Not a miss; an unmeasured line on the hardware that was meant to measure it. |
| **P7** | **Vehicles.** `NodeBeamAsset` and its cooker, beam plasticity and breakage, the hybrid rigid-core structure, suspension joints, the powertrain chain (§11.4), the tyre model (§11.5), wind coupling (§11.6), the vehicle editor. | **A drivable vehicle that deforms permanently on impact and loses parts**, at the §13.1 target, deterministic under replay. | **Complete.** The beam exists as a constraint kind and its numbers come from a material (P7-A, P7-B, P7-A2 — see §16.22): the descriptor with its deform and break thresholds, the axial projection and its load recovery, the rate-based damping pass, tick-boundary plasticity, the derivation from a `SoftBodyMaterial` and a cross-section, and the fifth kind wired into both solvers with four conformance scenes holding them to each other. The `.sushinodebeam` asset exists too (P7-C — see §16.23): nodes, beams, the collision surface, the rigid core's mass properties, the shell-to-core attachments and the render-mesh skinning, in one blob that refuses what it would not itself load. And `NodeBeamCooker` produces one from a mesh, a dial and a `SoftBodyMaterial` (P7-D — see §16.24): the tetrahedralizer's lattice as the node cloud, its edges as the beams classified by length into structural and bracing, a frame-relative render skinning that reproduces the rest pose to 1e-8 m, and the mass split between a rigid core and the shell hung off it. And the hybrid is alive in a solver (P7-E — see §16.25): nodes as particles, beams as the fifth kind, the core in its principal frame, the shell held on by ball-joint mounts, and a tick boundary where beams dent, mounts tear out, and a part that has lost its last tie is reported once as having come off. Suspension and wheels are built (P7-F — see §16.26): a corner is a slider with a spring-damper drive plus a hinge for the axle, steering is that slider's frame turned about its own axis, `JointMotorT` grew the damping rate a spring-damper drive needs, and a leak in the core angular integrator that cost a spinning wheel a third of its speed per second was found and fixed. And the drivetrain turns them (P7-G — see §16.27): §10.5's first escape hatch applied, a one-dimensional chain with the crankshaft as its only free coordinate, a clutch whose torque is solved and clamped rather than sprung, a differential that is one lock number instead of three kinds and that balances to conserve, and a coupling that puts a torque on each driven wheel and the exact negative of it on the chassis. And the tyres are under it (P7-H — see §16.28): a brush model whose curve is derived rather than fitted, one slip vector saturated once so combined slip cannot be got wrong, load sensitivity read off the normal force the solver already recovered, and the reaction shared back by load so the world loses exactly what the wheel gains. And the row is closed (P7-I and P7-J — see §16.29 and §16.30): §11.6's `WindSampler` seam mirroring `GravitySampler`, with the wind term written as a *difference* against the still-air drag `predict` already applies so that still air costs exactly nothing; the cooker's per-node drag area finally read; the car's own drag as a body constant and its downforce at a centre of pressure that is not its centre of mass; the acceptance scene proving all four clauses of this row at once; and the Vehicle window, whose derived column is the half of it that catches mistakes. **The acceptance clause's number is unmeasured:** "at the §13.1 target" is a GPU-timed budget, and §16.35 records that every §13.1 target still needs a GPU this machine does not have. Built, not measured. |
| **PX** | **Exposure.** Everything P0-P7 built, reachable without writing C++: §5.5's `PhysicsJoint`, `VehicleInstance` and `CollisionFilter` components with their `IWorldEditor` surfaces and scene serialization; §5.3's surface materials carried per body and combined per pair; the Assembly editor; §14's physics debug draw and joint gizmo; the vehicle drawn, driven from the keyboard and instanced from a path; and the demonstration scene as a file. | **An author can build, see and drive every physics feature the engine has from the editor**, and anything they cannot is a named gap rather than a shortcut. | **Complete** — see §16.31 (joints), §16.32 (materials, filters, the Assembly editor, debug draw), §16.33 (the vehicle in a scene), §16.34 (drawn, driven, shipped), §16.35 for the remainder, and §16.36 for PX-9, the last item, which closed the stream. The node-beam cooker now has an editor entry point in the Bake window: `NodeBeamPostProcessor` is registered into the shipped chain in `engine/domain/physics/source/cooking/mesh_post_processor.cpp`, `ImportProfile::node_beam_settings` and `CookingParameters::cook_node_beam` carry the settings, and `tests/unit/test_import_chain.cpp` covers it, so step one of `docs/guides/vehicles.md` no longer needs C++. Three smaller items §16.35 tabulated stay open and are outside this phase: the vehicle setup has no serializer, the cooked render skinning is not drawn, and joint gizmos are not draggable. |
| **P8** | **Scale.** Device-resident broadphase, narrowphase, and contact solve. Structure-of-arrays state columns. Deterministic parallel accumulation (§12.2). Per-island substepping and the one `DynamicGraph` region per island it needs (moved here from P2 — see §16.10). Half-precision storage measured and kept or dropped (§6.5). Optional runtime-accelerated cooking stages (§6.6). Budgets and their reporting. | Every §13.1 target met or beaten; determinism tests still byte-equal; the conformance suites pass for the device implementations. | **In progress — see §16.38 to §16.44.** P8-A's measurement (§16.37) set the order: reduce barriers before building device collision the measured scene has no use for. **Closed:** the zero-capacity structural skip that measurement itself exposed (§16.38); the `Execution::DynamicGraph`/`Region` seam extension the runtime side of per-island regions needs (§16.39); the per-node device-timing breakdown surfaced into `PhysicsStatistics` and the editor (§16.40, closes §18 R8's deferred half); a per-tick budget on continuous-collision escalation and the statistic that was structurally always zero until now (§16.40); the `sycl::half` cosmetic storage path, its measurement harness, and now (§16.41, addendum) the harness's printed numbers, read once build access made running it possible — the device-buffer verdict §6.5 actually asks for is still unmeasured, for the reason the harness's own comment already gave; two stale §18 rows corrected from Open to Built (R4, R6) and one from missing to recorded (R9, mirrored into the runtime's own requirements doc — §16.43); a settled island's *joint* now actually costs nothing rather than merely early-outing its math, behind an opt-in flag, tested (§16.44). **Open, and why:** the deeper barrier-reduction primitive P8-A's own finding asks for has no supporting runtime call today and is recorded as a new runtime ask rather than guessed at engine-side (§16.42, §18 R9); per-island substepping's full form — constraint bands becoming island-aware, one `DynamicGraph` region actually replacing the monolithic composition — needs the seam §16.39 built but is not itself built, and needs the project owner's answer on the design question §16.43 narrowed it to; beams and elements do not yet get §16.44's parking, and beams were found not to participate in island connectivity at all, a real prerequisite bug recorded rather than guessed at; structure-of-arrays state columns are scoped and now build/test-verifiable rather than blind, but the ~60-file job itself is a dedicated pass's work, not this one's remainder (§16.42 item 3, revised); device-resident broadphase/narrowphase/contact-detection remains the largest single item, sequenced last, not attempted. **Every §13.1 target still needs a GPU this machine does not have** (§16.35's finding stands unchanged) — nothing in this update closes that gap, and nothing claims to. |
| **P9** | **Gameplay surface.** Kinematic bodies, character controller, the full event stream into gameplay/audio/VFX through `IPhysicsEventSink`, rollback integration and its snapshot, and the networking validation harness. | Snapshot-rollback-replay byte equality across 10 000 ticks including contacts and fracture; impact events drive audio and VFX in a demo scene. | Not started |

### 16.1 P0, item by item

The phase table is deliberately coarse. P0 is large enough that "in progress" hides more than it
says, so this is what is actually done and what is not.

| Item | State |
|---|---|
| `physics/` restructured into §3.1's modules | **Done.** `core/`, `geometry/`, `collision/`, `constraints/`, `solver/`, `soft/`, `scene/`. `collision.hpp` split: the shape value types went down to `engine/domain/physics/include/SushiEngine/physics/geometry/shapes.hpp`, the pair logic stayed in `engine/domain/physics/include/SushiEngine/physics/collision/narrowphase.hpp`, and `Contact` moved to `engine/domain/physics/include/SushiEngine/physics/collision/contact.hpp` so the solver can name a contact without naming a shape. |
| `physics/core` | **Done.** Generational `BodyHandle`/`ConstraintHandle` over a fixed-capacity `HandleTable`, `PhysicsMaterial` with combine modes, `BodyFlags` + `CollisionFilter`, `PhysicsConfiguration`, `PhysicsStatistics`, and `RigidBodyT`'s new columns (`center_of_mass_local`, `material_index`, `flags`, `motion_measure`, `sleep_timer`, `island_index`). |
| Mass properties (§1.2 item 6) | **Done, and wired.** `engine/domain/physics/include/SushiEngine/physics/geometry/mass_properties.hpp`: closed forms for sphere, box, cylinder and capsule, the parallel-axis shift for compounds, and the inversion that keeps zero meaning "cannot rotate about this axis". `engine/world/simulation/include/SushiEngine/simulation/physics_extract.hpp` derives a body's inverse mass and inverse inertia from its scaled shape whenever it authors a positive density, and keeps the authored values otherwise — this row read "not yet wired into the extract" for a phase after it was. |
| The §1.3 correctness fixes | **Done.** Both. A sphere centre inside a box now leaves through the nearest face instead of always through +Y, and the plane contact uses the same projection as the pair contact — it used to carry an extra `inv_mass / w` factor that was only right for a body of unit inverse mass. |
| Mutable world (§6.4) | **Done.** `add_body`/`remove_body`/`add_constraint`/`remove_constraint`, valid immediately, no `finalize()`. Fixed-capacity buffers, generational handles, per-colour bands kept dense by swap-remove, and incremental recolouring over a 64-bit per-body colour mask. Removing a body removes its constraints. |
| `RuntimeGraphBuilder` (§6.6) | **Done for the rigid distance solver.** One file names `SushiRuntime::`; `IConstraintSolver` is the seam. The whole substep loop is unrolled into one composition and the tick issues one `run()`. Every node is late-bound, so a world that changes every tick keeps `compile_count() == 1`. Predict, projection and the velocity derivation are graph nodes; the hot columns are `Residency::Device` and the host reaches them only through `read_range`/`write_range`. The rebalancer is switched off and every allocation is pinned to one `DeviceIndex`. |
| Substepping schedule (§6.2) | **Done.** Derived from the previous tick's motion maximum, which the graph computes with the runtime's fixed-order `add_reduce` so the value does not depend on the worker count. Surplus substeps are switched off by `when()` rather than removed, so the count moves without recomposing. |
| Neutral `geometry/` module (§3.4) | **Done.** `SushiEngine::Geometry` is a top-level module linking nothing at all — no Vulkan, no SYCL, no runtime. It holds `TriangleMesh` (owning) and `TriangleMeshView` (borrowed, with a stride), and the signed-distance baker moved there from `render/gi/` as `bake_signed_distance_field`. `Gi::MeshSdfBrick` is now an alias and `Gi::bake_mesh_sdf` a four-line adapter that describes the renderer's 60-byte vertex array as a strided view, so nothing is copied and no GI call site changed. Five unit tests reach the baker without a renderer, which was the point. |
| `IPhysicsSimulation` split (§4.3) | **Done for what exists.** `IRigidBodyService`, `IClothService`, `IStaticGeometryService` and `IPhysicsStepper`, composed by `IPhysicsScene`, in `engine/world/simulation/include/SushiEngine/simulation/physics_services.hpp` — a header that names no runtime type, no solver and no shape, so depending on one service no longer drags in the whole solver. `ICollisionQueryService` and the rest of §4.3's list arrive with the features behind them. |
| The extract (§4.1) | **Done.** `gather_rigid_descs`, `gather_static_planes` and `collision_radius` left `RuntimeSimulation` for `engine/world/simulation/include/SushiEngine/simulation/physics_extract.hpp`, which takes a flat list of source entities and returns descriptors. `RuntimeSimulation` now owes it only what needs a world: who is alive, in what order, and where the hierarchy puts them. Nine unit tests cover the cases that used to be unreachable — a cylinder collapsing to a sphere, a collider overriding a visual, a plane that is also a rigid body. |
| Deterministic contact ordering (§12.1) | **Done for the host pass.** The candidate pairs are sorted by body index before resolution, so the Gauss-Seidel order is a function of simulation state rather than of whatever order the sweep's axis sort happened to produce. The pass is still on the host; moving it into the graph is what §12.2 waits on. |
| Broadphase once per tick (§6.1) | **Done.** It ran twice per sub-step — sorting the whole scene tens of times a tick to learn something that barely changes, and the single reason a large sub-step count was unaffordable. It now runs once, against bounds swept by how far a body can travel over the whole tick, which is what makes once-per-tick sound rather than merely cheaper. |
| Segmented accumulation (§12.2) | **Closed by the other route, and the route matters.** The ask was for accumulation paths to use the runtime's fixed-order reduce rather than a hand-built one. The physics has exactly one accumulation path — the substep schedule's motion maximum — and it now uses `Graph::add_reduce` (§16.10), the hand-built pair of nodes deleted. There is no *segmented* path, because contacts resolve by colouring rather than by folding per-body impulses: §17.5 records the colouring alternative as the fallback and it is the one in use. `add_segmented_reduce` is the right primitive for P6's soft-body vertex accumulation and is where it will be reached from. |
| Conformance suite (§4.4) | **Done.** `HostXpbdSolver` is a second `IConstraintSolver`, and `tests/integration/test_solver_conformance.cpp` runs the same scenes against both. The two share the layout (`ConstraintStore`) and the arithmetic (`XpbdDistanceProjectionT`, moved out of the runtime-including header for the purpose), so what the suite actually measures is the claim colouring makes: that projecting a colour in parallel equals projecting it in sequence. It earned its place immediately — see below. |
| `PhysicsStatistics` wiring (§13.3) | **Done.** Populated by the solver and the `sim/` boundary, reported through `IPhysicsStepper::statistics()`, asserted by tests, and surfaced by `applications/editor/source/physics/physics_statistics_panel.cpp`. The per-stage timings are real as of §16.10: the host stages from a host clock that is not read when profiling is off, the device half from the run report. The one thing not broken out is the composition's internals, which needs a node label the runtime's public `add()` does not carry (§18 R8) — recorded as an ask rather than estimated. |

**What P0 did not do, deliberately.** Contacts still resolve in a host pass rather than
inside the solve graph, so there are no per-body contact impulses for
`add_segmented_reduce` to fold and no per-stage device timings for the editor panel to
show. Both wait on the same change, and that change is P1's, not P0's: moving contacts
into the graph is only worth doing once there are *manifolds* to move, and P0's contact
model is still a single point per pair. The plain fixed-order `add_reduce` is in use
today, for the substep schedule's motion maximum.

### 16.2 What the conformance suite found

Worth recording, because it is the argument for building the suite at all rather
than after the fact.

The motion-measuring node — the one that feeds the substep schedule — read the body
buffer but declared `Reads()`, empty. The runtime cannot infer what a kernel touches;
a `Dynamic` callable is a `void(std::size_t)` capturing raw pointers, so a node that
does not *name* the data it reads has no edge to the work that produces it. That node
was free to run beside the solve, measuring velocities that were still being written.

It produced no crash, no warning, and nothing visibly wrong in a scene: the substep
count is a quality dial, so the symptom was a simulation that occasionally spent four
substeps where it should have spent eight. The only reason it surfaced is that a
second implementation derived a different number from the same state, and a test
compared them.

Two things follow. First, §6.6's rule — *every physics node names its data with
`Reads`/`Writes`* — is not a style preference, and the one place it was skipped is the
one place a bug appeared. Second, a conformance suite is only worth what its scenes
exercise: three of the first drafts passed against a deliberately broken solver
because their constraints were laid out already satisfied, so the projection had
nothing to correct and the ordering could not matter. Each scene here has since been
checked against a deliberately reversed colour order and made to fail.

### 16.3 Where this stands

**P0 closed on 2026-07-29.** The functional suite is 453 tests, all passing. What
follows is the handover: what a reader can now rely on, what is deliberately still
missing, and what the next session picks up.

**What exists and is tested.**

- `physics/` is seven modules with the dependency direction running one way, and
  `physics/geometry` owns single shapes while `physics/collision` owns pairs of them.
- A body has a material index, a flag word, a collision filter, a centre-of-mass
  offset and its sleeping bookkeeping. Mass and inertia can be derived from a shape
  and a density instead of typed by hand — though nothing calls that yet, so an
  author still types an inverse mass (see below).
- The world is mutable with no `finalize()`. Handles carry a generation; buffers are
  fixed at capacity and a budget being exceeded is a counted event; colouring is
  incremental; removing a body removes its constraints.
- The tick is one composition and one `run()`, built for the maximum substep count
  with every node late-bound. `compile_count()` is 1 after warm-up however much the
  world churns, and a test asserts it.
- `IConstraintSolver` has two implementations and a shared conformance suite. The
  substep count is derived from state through the runtime's fixed-order reduce.
- The `sim/` boundary is four segregated services; the extract is its own tested unit.
- `SushiEngine::Geometry` is a neutral module that links nothing, holding the triangle
  mesh and the shared signed-distance baker.

**What P0 deliberately left.** Three things, and they are one thing: the contact pass
is still a host pass outside the solve graph.

1. **Contacts are not constraints yet.** They are resolved by
   `engine/domain/physics/include/SushiEngine/physics/collision/contact_solver.hpp` between
   substeps, not projected inside the graph. So there is no per-body contact impulse for
   `add_segmented_reduce` (§12.2) to fold and no per-stage device timing for the Physics panel to
   show. Both arrive with the same change, and that change belongs to P1, because moving a
   *single-point* contact into the graph would be moving the wrong thing — manifolds come first.
2. **Mass properties are computed but not wired.**
   `engine/domain/physics/include/SushiEngine/physics/geometry/mass_properties.hpp` is complete and
   tested in isolation; `PhysicsBodyParams` still carries a hand-authored inverse mass and diagonal
   inverse inertia. Connecting them is a `sim/` change that belongs with the `Collider` record
   replacing `ColliderParams` (§5.5), not with P0.
3. **`ContactBody::is_cloth` survives.** §4.4 wants it replaced by filter masks and
   material properties, which now exist — but the contact pass that reads it is the
   pass P1 rewrites, so deleting the flag now would mean writing its replacement twice.

**What P1 opens with.** §16's P1 row, in the order the dependencies fall: persistent
manifolds with face clipping and reduction (§7.3) first, because everything else in the
phase reads them; then warm starting, then the velocity pass that finally makes
restitution and dynamic friction expressible (§7.4). The contact-as-constraint-kind
move (§6.3) lands with the manifolds, and it is what closes the three carry-overs
above.

**Two things worth reading before touching the solver.** §16.2 records the bug the conformance suite
caught and why an undeclared `Reads` is not a style question. And
`engine/domain/physics/include/SushiEngine/physics/solver/runtime_graph_builder.hpp`'s file comment
records why every projection node writes the whole body buffer — the serialization between colours
is deliberate, not an oversight.

**Ordering rationale.** P0–P2 fix the foundation, because every later phase multiplies whatever is
underneath it — and P0's runtime work in particular, since a solver that recomposes its graph every
tick makes every performance number in §13.1 unreachable no matter what is built on top. P3 lands
the assembly scenario as early as it can be landed (confirmed in §17.3), since it depends only on
the solver, not on cooking. P4 must precede P5 and P6 because both consume cooked distance fields
and tetrahedral meshes. P7 composes P3, P5, and P6, and cannot come earlier. P8 optimizes a correct
system rather than a moving one, which is the only order that works.

### 16.4 P2, and where it leaves P1

**P2 closed on 2026-07-29.** Its acceptance is met for everything P2 owns, with one scope
statement made explicitly below rather than buried. 192 tests cover the physics modules and
pass; every header is clean under `-fsycl -Wall -Wextra -Werror -pedantic`.

**What exists and is tested.**

- **Shapes.** Capsule, cooked convex hull, triangle mesh with a hierarchy and the
  internal-edge correction, height field, compound. One GJK/EPA routine collides every
  convex pairing; an ordered-`ShapeType`-pair table of function pointers dispatches, filled
  by folding a type list rather than written out.
- **Scale.** `engine/world/simulation/include/SushiEngine/simulation/collider.hpp` holds §5.5's
  `Collider` record: the extract resolves a collider from the authoring component, applies the
  entity's scale, and derives mass and inertia from the *scaled* shape and a density. That is **P0
  carry-over 2, closed** — and the reason it waited is the word "scaled": deriving mass from the
  authored shape would have given a doubled crate the mass of a single one, which is worse than a
  hand-typed number because it looks derived. A density of zero, the default, keeps the authored
  mass exactly, so nothing changes under an author who did not ask for it. `ColliderParams` remains
  the authoring surface until the cooked `CollisionAsset` it should be able to name exists (P4);
  `Collider` already carries the asset identifier.
- **Broadphase.** `IBroadphase` with two implementations — the sweep, kept as the reference
  and small-scene path, and a two-tree bounding-volume hierarchy — held by a shared
  conformance suite to the *same sorted pair set* through motion, removal and identifier
  reuse. Fat bounds with hysteresis, refit-and-rebalance on the insertion path, swept bounds
  for continuous bodies, a separate static tree, and an `added`/`persisted`/`removed` pair
  cache, which is what lets a manifold and its warm-start impulses survive a tick.
  One decision worth recording: everything that decides *which boxes are tested* — the
  enlargement, the hysteresis, the filter rules — lives in the shared base, and only *how
  the overlaps are found* belongs to an implementation. Left to the implementations, the
  hierarchy's remembered box and the sweep's freshly computed one would differ by a tick of
  motion and the suite would be asserting something neither should have to promise.
- **Filters and layers.** Honoured by the broadphase and the contact pass, and
  **`ContactBody::is_cloth` is deleted** (P0 carry-over 3): cloth is a body on the cloth
  layer whose mask excludes that layer, which is one line of initialization instead of a
  branch every routine touching the type had to know about.
- **Queries.** `ICollisionQueryService` joins the §4.3 split and `PhysicsSimulation`
  implements it. Closed-form rays for sphere, plane, oriented box and triangle; one
  conservative-advancement routine for everything else, which needs only a support function
  and so covers the capsule, the cooked hull and every convex shape not yet written. Sweeps,
  overlaps and closest points on the same tree. Hits are ordered by distance with ties broken
  by proxy, never by traversal order — a tree's traversal order is a function of its
  insertion history, and a gameplay system reading `hits[0]` must not be reading a history.
- **Islands and sleeping.** `engine/domain/physics/include/SushiEngine/physics/scene/islands.hpp`.
  Static bodies conduct nothing, so a warehouse is not one island. An island's key is the lowest
  body index in it and islands are numbered in ascending key order, because §6.6 derives
  cross-region edges in that order. A revision counter advances on genuine sleep/wake transitions
  and on nothing else. Sleeping is now enforced in one place — `generalized_inverse_mass` reports
  zero for a body that is not simulated, and the impulse appliers return early — so "asleep" means
  the same thing to the distance constraint, the contact solve and every joint P3 adds.

**The measured numbers**, on this machine, at `-O2`, for everything P2 owns — proxy updates,
the hierarchy's repair, the pair search and its cache, the narrowphase over every produced
pair, and the island partition:

| Scene | Measured | §13.1 target |
|---|---|---|
| 1 000 mixed-shape bodies, 25 stacks of 40, 975 contacts | **1.37 ms/tick** | ≤ 2 ms (whole tick) |
| 10 000 settled bodies, 10 000 sleeping islands | **0.27 ms/tick** | ≤ 0.5 ms |

**What that does and does not measure.** It does not include the constraint solve, which is
on the device behind SushiRuntime; these are the collision and island halves of the tick.
Stated plainly because a number whose scope is vague is a number nobody can act on.

Two things the measurement changed, and both are worth keeping. The 10 000-body scene first
came out at **6.7 ms**, thirteen times over target, for two reasons that only a measurement
finds. The island decision was never reaching the broadphase, so the hierarchy kept
descending from ten thousand leaves whose bodies had been asleep for a minute — sleeping
that nothing is told about saves nothing. And the partition sorted its bodies every tick;
since an island's key *is* its lowest body index, walking bodies in ascending order meets
each island's key exactly when it first appears, so the partition is two linear passes and
no comparison sort at all.

**What P2 did not do.** One item deliberately, and — as §16.10 records — one item that was not
noticed at all: the island layer below was never called by the live tick, and stayed a library with
a benchmark until 2026-07-30. The deliberate one: **one `DynamicGraph` region per island is not
wired**, and it has since moved to P8. The island layer produces exactly what that wiring needs — a
deterministic partition, keys in ascending order, and a revision that moves only on a transition —
and sleeping already removes a settled body from the predict, the projection and the velocity pass.
What is missing is the composition-side change in
`engine/domain/physics/include/SushiEngine/physics/solver/runtime_graph_builder.hpp`, whose
constraint bands are per colour rather than per island; re-cutting them is a solver change with P0's
`compile_count() == 1` property to preserve, and it belongs with the same move that puts contacts
inside the graph. That move is P1's remaining work, below.

**What P1 still owes.** P1's machinery is built and tested — manifolds, warm starting, positional
static friction, the velocity pass, `contact_offset`/`rest_offset` — but the live `sim/` tick still
runs the old single-point
`engine/domain/physics/include/SushiEngine/physics/collision/contact_solver.hpp`. So the phase is
*partly done*, and what remains is one change with several consequences: move contacts into the
solve graph as a constraint kind (§6.3). That closes P0 carry-over 1, gives `add_segmented_reduce`
the per-body contact impulses it has been waiting for (§12.2), gives the Physics panel its per-stage
device timings, and is the natural moment to cut the constraint bands per island and finish the
paragraph above. Contact events are the other open P1 item and are small beside it.

### 16.5 Contacts as a constraint kind, and what P1 still owes after it

**Done on 2026-07-29.** §16.4's "one change with several consequences" — move contacts
into the solve graph as a constraint kind (§6.3) — has landed. **P0 carry-over 1 is
closed.** The solve graph now holds contacts on exactly the terms it holds a distance
constraint: the same colouring, the same fixed per-colour bands, the same late
binding, one node per (kind, colour) per substep.

**The four pieces, and the decision inside each.**

- **`engine/domain/physics/include/SushiEngine/physics/solver/contact_constraint.hpp`** — the
  descriptor and three shared projections. A contact names two body slots and exposes `a`/`b` in the
  shape the colourer expects, so a contact colours against joints without the colourer knowing
  contacts exist. Static geometry is `null_contact_body` and every projection substitutes
  `immovable_body` for it, rather than a second one-sided projection — which is how a plane contact
  and a pair contact end up disagreeing, the mistake §1.3 recorded once already. Three callables and
  not one because the schedule needs them at three different points: preparation *before* predict
  (the arrival speed restitution is a statement about is gone by the time the positional solve has
  run), the positional projection with the other positional projections, and the velocity pass after
  `update_velocity`, because until then there is no velocity for dynamic friction and restitution to
  be statements about.
- **`engine/domain/physics/include/SushiEngine/physics/solver/contact_store.hpp`** — the layout, for
  one tick. Deliberately *not* `ConstraintStore` with a flag: a contact has no handle, no lifetime
  and no swap-remove to mirror, and building persistence machinery for something that is not
  persistent is a cost paid for nothing. What survives a tick is the manifold, which the caller
  keeps keyed by the broadphase pair cache — which is what makes warm starting possible at all. The
  union colouring is **layered, not merged**. The persistent kinds are coloured once, when added;
  contacts are recoloured every tick. One shared colourer cannot do both, because clearing it for
  the contacts would throw the joints away. So each tick a body's mask *starts* as what the
  persistent colourer says it holds, and the contacts pile on top — seeded lazily, on first mention,
  because a scene with four thousand body slots and forty contacts should not pay four thousand
  copies.
- **`IConstraintSolver`** gained `begin_contacts` / `add_contact` / `contact_count` /
  `read_contact`. Submitted, not added: no handle comes back and what is not
  resubmitted is simply gone. `read_contact` reports in **submission** order, because
  storage order is a colouring — an implementation detail a caller can neither
  predict nor use, and one that would silently pair the wrong impulses with the wrong
  pairs if it leaked.
- **The graph** holds, per substep, one preparation node, one positional node and one
  velocity node per colour. The ordering is not forced with false writes: the
  preparation reads the bodies and predict writes them, so the write-after-read edge
  the tracker already derives puts predict second. Contacts go to the device as one
  transfer per non-empty band and come back the same way — the readback is not
  optional, because the impulses the solve settled on are what warm starting inherits
  and what a contact event will report.

**What was measured.** The shared conformance suite grew four contact scenes and all
eleven pass: contacts against static geometry, body-to-body contacts in a stack, a
body under both a contact and a distance constraint, and a contact count that walks
from nothing to three and back down. The two solvers agree to **1e-9** through all of
it, and `compile_count()` stays at **1** across a tick set that is rebuilt from
nothing every frame — which is the whole claim late binding was adopted for. Eleven
new unit tests cover the store's colouring and the seam.

**A defect the suite found, worth recording.** `RuntimeGraphBuilder::read_body`
served the device buffer while the write that had just been staged was still in the
host mirror, so a body added and read before the first `step` came back as the
retired slot it used to be. The host solver has no staging and so no way to
disagree — this is precisely the class of divergence a conformance suite exists for,
and it had been sitting there since P0 because nothing had read a body before
stepping. The fix keeps one rule rather than two: the device owns the state, and
everything staged is flushed onto it before anyone reads.

**What P1 still owes, and why it is a separate move.** `sim/PhysicsSimulation` still
runs `Physics::PhysicsWorld` with the old single-point host contact pass; it does not
hold an `IConstraintSolver` at all. Moving it is not a wiring change, it is a
migration: the rigid and cloth worlds are driven in lockstep so cross-world contacts
resolve before either derives its velocity, gravity is sampled *per body per substep*
from a field rather than passed as the one uniform vector `StepParameters` carries,
and both properties have to survive the move. That belongs with the same pass that
gives `RuntimeGraphBuilder` a per-body external acceleration — which §5's "a
non-uniform field is sampled per body by the scene above and folded in there" already
names as the intended shape. Contact events are the other open P1 item and are small
beside it.

**And the paragraph §16.4 left open** — one `DynamicGraph` region per island — is now
unblocked rather than done. The constraint bands are still cut per colour; cutting
them per island is the change, and it can now be made for both kinds at once instead
of for one kind and then again for the other.

### 16.6 The live tick, moved onto the solver

**Done on 2026-07-29.** `sim/PhysicsSimulation` no longer holds a `PhysicsWorld`, and
no longer holds two of them. It holds **one `IConstraintSolver`**, and §0.1's "one
solver, not a family of solvers" is in effect rather than aspirational.

**What actually changed.**

- **A cloth particle is a body in the same buffer as the rigid bodies**, linked to
  its neighbours by the same `XpbdDistanceConstraint` a rope uses. It was never a
  different kind of thing; it lived in a different world only because the world was
  immutable and a cloth had to be built all at once. `build_cloth_grid` gained an
  overload against the solver seam — a separate overload rather than a template,
  because `PhysicsWorld` numbers its bodies and the solver hands out generational
  handles, and quietly treating one as the other is the bug the generation counter
  exists to catch.
- **Rigid-to-cloth contact is an ordinary contact.** The two worlds used to be driven
  in lockstep so a contact spanning them could be resolved between their substeps.
  There is nothing to keep in step now, so the lockstep is gone, and with it the last
  reason the tick could not be one `run()`.
- **The old single-point host contact pass is gone.** The tick generates manifolds on
  the host from one broadphase over every collidable thing — rigid colliders, cloth
  particles and static planes in one hierarchy — warm-starts each from last tick's,
  and submits them as §6.3 contacts. The solve happens on the device, inside the
  substep loop, in the right place in the schedule.
- **`set_rigid_bodies` is a diff, not a rebuild.** The world is mutable (§6.4), so an
  entity that was here last frame keeps its body, its handle and its velocity. The
  previous implementation rebuilt the world every call and carried velocities across
  by hand — re-deriving what the handles already knew.
- **`RigidBodyDesc` lost `radius`, `box` and `half_extents`.** They were a flattened
  copy of `Collider`, kept because the single-point path could not read a collider.
  §16.4 said they go when the manifold path takes over the live simulation; it has,
  and they have. A derived copy of a record that is already present is how a body
  ends up colliding as a box of one size while reporting a radius from another — and
  the integration tests were, until this was done, passing against a default collider
  while setting fields nothing read.

**Three behaviour changes, stated rather than discovered.**

1. **The substep count is derived.** The caller's `substeps` argument is now a
   *floor* — the quality it is willing to pay for regardless of how slowly things
   happen to be moving — carried in the new `StepParameters::substep_floor`. State
   may raise the count, the caller may raise the floor, and neither lowers what the
   other asked for. §6.2 does not allow a caller to set it outright.
2. **The gravity field is sampled per body per tick, not per substep**, and lands in
   the new `RigidBodyT::external_acceleration`. This is forced, not chosen: `predict`
   runs on the device inside one composition, so there is no point inside the substep
   loop at which a host sampler could be called at all. `StepParameters::gravity`
   already said a non-uniform field is "sampled per body by the scene above and
   folded in there"; there is now somewhere for it to be folded into.
3. **The `iterations` argument is ignored.** §0.2: small steps, not many iterations.
   The substep schedule subsumes it, and honouring both would be two dials on one
   quantity.

**What was measured.** All seven of `tests/integration/test_physics_simulation.cpp`'s integration
scenes pass unchanged against the new implementation — a box settling on its face and
not its bounding radius, a tilted box on its edge, cloth and a rigid body pushing on
each other, a four-high stack that does not interpenetrate, and a body fast enough to
cross its target inside one tick still being found. 197 physics unit tests pass. The
solver conformance suite is unaffected and still passes at 11/11.

**What is still on the host, and named as such.** Broadphase, manifold generation and the warm-start
cache. §6.6 puts all three in the graph; they are not there yet, and
`engine/world/simulation/include/SushiEngine/simulation/physics_simulation.hpp` is where they will
move from. The bodies are read out in one bulk transfer per tick precisely because of it — the host
needs the poses to find the contacts — and written back the same way. One pair of transfers, not one
per body, and it is the honest remaining cost of host-side collision detection.

**Contact events, and P1 closed.** The last item. `IContactEventService` joins the
§4.3 split — its own interface, because a pressure plate, an impact sound, a damage
volume and a checkpoint trigger are all this one question and none of them has any
business depending on rigid-body lifecycle to ask it.

- **Reported per pair of colliders, not per contact point.** A crate landing flat
  produces four points and one event, because "the crate landed" is one thing that
  happened. The point and normal are the manifold's *deepest* point: averaging the
  four would put the point in the middle of a face the body only touched at a corner.
- **Begin, persist and end are a merge, not three tests.** The touching pairs are
  kept in a list sorted by pair key rather than in a hash map, and this tick's list
  is merged against last tick's; a pair only in the new list, in both, or only in the
  old *is* the three phases. Deriving them any other way means deciding twice what
  "still touching" means. The sort is also what makes the sequence a function of the
  scene rather than of the broadphase's insertion history — and a listener that
  spawns an effect observes the sequence, so §12.1 applies to it.
- **The impulse is real.** Total normal impulse over the manifold, in newton-seconds,
  which is what separates a scrape from a crash — and it is the reason the solved
  manifolds are read back off the device at all. Zero on a trigger, which is detected
  and never resolved, and on an `End`, which reports what the contact carried on its
  last live tick.
- **A trigger is a flag on the event, not a second stream.** A listener that wants
  both — a damage volume that also pushes — should not have to subscribe twice and
  correlate; one that wants only triggers has a one-line filter.
- **Static geometry reports `b == NULL_ENTITY`.** Not a missing value: static
  geometry is not an entity, and inventing one so the field is never null would mean
  every listener filtering out a fiction.

Four integration scenes cover it and pass: a box that begins **once** and then
persists with a positive impulse, a body lifted clear that ends exactly once and then
reports nothing at all, a trigger that is reported with zero impulse while the body
falls straight through it, and five bodies landing out of order whose events still
arrive in scene order. **P1 is complete.**

### 16.7 Where this stands, and what to pick up next

Written on **2026-07-29**, at the end of the session that closed P1. This is the
handover: what is true now, what the next person should not have to re-derive, and
what the next move is.

**Phase state.** P0, P1 and P2 are **complete**. P3 (joints and assemblies) is the
next phase and nothing blocks it. Every P0 carry-over is closed.

**The tick, as it actually runs today.** One `IConstraintSolver`
(`RuntimeGraphBuilder`) holds every body — rigid and cloth alike — and every
constraint kind. Per tick: the poses come to the host in one bulk transfer, the
gravity field is sampled per body into `RigidBodyT::external_acceleration`, one
bounding-volume hierarchy over rigid colliders, cloth particles and static planes
yields the candidate pairs, a manifold is generated per pair and warm-started from
last tick's, the whole set is submitted as the per-tick constraint kind, and **one
`run()`** advances everything. Then the solved manifolds come back, the touching-pair
lists are merged, and the contact events fall out of that merge.

**Four things worth not re-deriving.**

1. **The substep count is derived; the caller's `substeps` is a floor.** Raising the
   floor is how a scene of tall stacks buys quality; the motion measure is how a fast
   scene buys it. Neither lowers the other.
2. **The gravity field is per body per tick.** Not a compromise — `predict` runs on
   the device inside one composition, so no host sampler can be called inside the
   substep loop at all.
3. **Warm-start keys are proxy indices**, so the contact proxy list is renumbered
   only when the *membership* changes, never merely because something moved. A list
   rebuilt every tick would silently restart every impulse from zero, and the symptom
   would be a stack that creeps rather than anything that looks like a bug.
4. **Contact events are a merge of two key-sorted lists, not three tests.** The sort
   is what makes the sequence a function of the scene rather than of the broadphase's
   insertion history, and a listener that spawns an effect observes the sequence.

**The reduction, and the rule it taught.** `Graph::add_reduce` and `API::Maximum` do not exist in
the runtime (see the correction in §18). The fold operators now live where the engine's math lives —
`engine/foundation/core/include/SushiEngine/core/blas_placeholder.hpp`, alongside `dot` and `cross`,
re-exported through `engine/foundation/core/include/SushiEngine/core/types.hpp`, and moving to
SushiBLAS with the rest of it: `Sum`, `Product`, `Minimum`, `Maximum`, and `fold_range` as the
sequential reference. The *order* is deliberately not in them, because order is not arithmetic —
`engine/domain/physics/include/SushiEngine/physics/solver/runtime_graph_builder.hpp` builds its own
from two ordinary nodes, one work-item per 256-slot segment folding ascending, then one work-item
folding the partials, also ascending. Both orders are functions of the capacity and the segment
size, both fixed at construction, so the substep count derived from the result cannot depend on the
worker count or the steal pattern (§12.1). If the runtime ever ships WP-4, adopting it is deleting
those two nodes.

**What is still on the host, and where it will move from.** Broadphase, manifold generation and the
warm-start cache, all inside
`engine/world/simulation/include/SushiEngine/simulation/physics_simulation.hpp`. §6.6 puts all three
in the graph. The single bulk read of the bodies at the top of the tick exists *because* of them —
the host needs the poses to find contacts — so that transfer and those three stages retire together,
not separately.

**The next three moves, in the order the dependencies fall.**

1. **P3: joints.** The constraint-kind dimension is built and has two kinds in it
   (§6.3); a joint is a third, and `IConstraintSolver` needs no new shape to admit
   one. This is the phase with the least in its way.
2. **One `DynamicGraph` region per island.** Unblocked rather than done: the island
   layer produces a deterministic partition with keys in ascending order, and the
   solve graph now has *both* kinds banded per colour. Cutting the bands per island
   is one change that serves both, where doing it earlier would have meant doing it
   twice. It needs R3 from the runtime, which is not reachable — see §18.
3. **Collision detection into the graph** (§6.6), which is what retires the per-tick
   transfer pair above and feeds the editor its per-stage device timings from real
   nodes instead of a host clock.

**One seam landed alongside this and is worth knowing about.** The Physics panel's
profiling request now rides `IPhysicsStepper::set_profiling_requested`, wired from
`applications/editor/source/main.cpp` through `ISimulation::set_physics_profiling`. Profiling is a
*construction-time* property of the solve graph — with it off the hot path carries no
timestamping at all — so the request is consumed when the solver is next built rather
than toggling a running one. Recreating a live solver to honour a checkbox would
discard every body's velocity to answer a question about timing.

### 16.8 The joint library, and the door that comes off

**Done on 2026-07-30.** P3's acceptance criterion passes: a 35 kg door on a hinge
about its local +Y, limited to `[0°, 68°]`, with 4 N·m of friction so it does not
swing free, hung off a chassis, reporting its load, and tearing off above 12 kN. It
is driven through `IJointService` on the live `PhysicsSimulation` rather than
through the solver, so what the acceptance test measures is the whole path.

**A joint is the third constraint kind, and it cost the solver nothing new.**
§6.3's kind dimension was built for exactly this: joints take their colour from the
same union, live in their own fixed per-colour bands, and get one node per (kind,
colour) per substep — a positional node and a velocity node. `compile_count()` is
still one through a tick set that adds and removes joints every frame, and a test
asserts it. `IConstraintSolver` gained `add_joint` / `remove_joint` / `read_joint` /
`write_joint`, and both implementations satisfy them.

**Seven kinds, one descriptor, one registration line.**

- **`engine/domain/physics/include/SushiEngine/physics/constraints/joint.hpp`** — one POD for every
  kind. §4.2 requires that a new joint touch nothing that exists; it does *not* require a buffer per
  kind, and a buffer per kind would have multiplied the compiled node count eightfold to express a
  difference that lives in eight lines of arithmetic. So the kinds share a descriptor and a band and
  diverge inside one kernel. The cost is stated rather than hidden: a ball joint carries a linear
  limit it never reads, and a joint descriptor is a few hundred bytes in a scene that holds hundreds
  of them.
- **`engine/domain/physics/include/SushiEngine/physics/constraints/joint_primitives.hpp`** — the
  four rows every kind is built from (Müller et al. 2020 §3.3–3.4): one positional, one angular, and
  their velocity-level counterparts. Each takes a **world-space violation vector** and treats it as
  the constraint function whose gradient acts along `+v̂` on body *b* and `−v̂` on body *a* —
  identical to `XpbdDistanceProjectionT`, deliberately, because a joint row and a distance
  constraint that disagreed about the sense of a correction would be two formulations of the same
  thing and §1.3 records what that costs. A caller's whole job is to produce a `v` whose reduction
  satisfies the joint; the sign lives in its direction.
- **`engine/domain/physics/include/SushiEngine/physics/constraints/joint_projection.hpp`** — the
  seven kinds as traits structs, a `JointKinds` type list as the registration, and a fold that turns
  the list into the dispatch. A fold rather than the function-pointer table
  `engine/domain/physics/include/SushiEngine/physics/collision/narrowphase_dispatch.hpp` uses, and
  that is not a stylistic difference: the narrowphase runs on the host, a joint projection runs
  **inside the solve graph** on the device, and SYCL device code has no indirect calls. A pointer
  table would work on the CPU backend and fail on a GPU. The fold compiles to a chain of compares
  against a compile-time constant.
- **The angular vocabulary is one swing/twist decomposition.** A hinge is "swing must
  vanish"; a cone-twist is "swing is bounded and twist is ranged"; a fixed joint is
  both bounded at zero; the general joint adds per-axis offset limits. Deriving them
  all from one exact decomposition of one relative rotation is why each kind is a
  three-line list of shared rows.

**Two refinements to §10.1, stated rather than quietly made.**

1. **`BallJoint` has no limits.** §10.1's table gives it "swing and twist limits,
   cone angle", which is exactly `ConeTwistJoint`'s row. Two kinds that differ only in
   whether the author remembered to enable a limit are two names for one thing, so
   `Ball` is the pure spherical joint and a ball joint with limits *is* a cone-twist.
   The library is one kind smaller and no capability is lost.
2. **`GearJoint` and `RackJoint` are deferred to P7, not dropped.** They couple two
   *accumulated* rotations, which means carrying an unwrapped angle across ticks —
   state no other joint needs — and their use is the differential and the steering
   rack. §10.5 has already ruled that a drivetrain is solved as an independent
   one-dimensional chain coupled through a torque constraint, precisely because
   forcing it through the three-dimensional solver is slower and less accurate. So the
   gear belongs with the powertrain in §11.4, and building it here would be building
   the tool §10.5 says not to use.

**Three defects the tests found, all worth recording.**

1. **A hinge diverged to two meganewtons in twenty substeps**, and the cause was one
   shared frame resolution. An angular correction rotates a body about its centre of
   mass, which *moves the attachment point* by the lever arm. The linear rows then
   read frames resolved before that rotation and corrected a gap already partly
   closed — over-correcting by the lever-arm share every substep, which the next
   substep's angular row dutifully fixed, which the linear row over-corrected again.
   The loop gain is the positional row's angular share, about three quarters for a
   door-shaped body, on top of a full angular correction: above one. The fix is
   **re-resolving the frames between the angular group and the linear group**, and it
   is load-bearing rather than tidiness. Within a group staleness is harmless, and
   that is a property of the groups rather than luck: a swing lock and a twist limit
   act about perpendicular axes, and the general joint's three offset limits act along
   perpendicular ones.
2. **A break threshold on the mean load never fires.** The plan said mean rather than
   peak, on the reasoning that a single substep's multiplier during a stiff transient
   is noise. That reasoning is wrong in the one case that matters. A hard impact is a
   large separation the constraint closes in one substep and then overshoots, so the
   next substep's correction points the other way and is very nearly as large: a door
   yanked two metres off its hinge and snapped back reported **344 N** — its own
   resting weight — while the substeps either side of the snap carried **sixteen
   meganewtons**. Averaging a load whose direction reverses measures the *net pull*,
   and what tears a mount out is the magnitude. So the descriptor now carries both:
   the vector sums give the mean, which is the load readout and the rigid-body half of
   *mukavemet*, and `peak_force` / `peak_torque` give the worst instant, which is what
   the threshold reads. The worry that a peak would be noisy at rest does not hold up —
   a resting hinge's peak and mean differ by about a newton in three hundred, because
   nothing in a settled joint spikes.
3. **`ConstraintStore::place` gave up when the *assigned* colour's band was full**
   rather than trying the next free colour. Equivalent for a cloth lattice, whose
   constraints share bodies and therefore spread across colours by construction. Not
   equivalent for constraints on *disjoint* pairs, which is what joints almost always
   are: a hundred car doors on a hundred chassis each see every colour as free, so
   every one is offered colour 0 and only `capacity / colors` of them can ever be
   placed — the rest reported as a capacity overflow with the buffer mostly empty.
   `ContactStore::place` had the right rule from the start, for the same reason
   (contacts against one ground plane are disjoint too). Fixed for both persistent
   kinds; `IncrementalColoring` gained `take()` so a caller can apply its own
   constraint on the choice and then take the colour it settled on.

**The force recovery is exact, and that is checkable.** §10.4's `force = λ n̂ / h²`
costs nothing beyond the addition, so every row folds its share. A door of mass *m*
hung from a hinge and at rest has one external force and an angular row that carries
pure torque, so the hinge's reaction must be exactly *mg* and its torque exactly
*mg·r*. It reports **343.35 N** and **171.67 N·m** for a 35 kg door on a 0.5 m lever —
statics, not a tolerance.

**The boundary.** `IJointService` joins the §4.3 split with its own vocabulary
(`JointDesc`, `JointLimitDesc`, `JointMotorDesc`, `JointState`, `JointBrokenEvent`) so
a mechanism that creates a hinge does not thereby depend on the solver that projects
one. Two decisions inside it:

- **A joint's endpoints are both bodies.** An immovable endpoint is a body with zero
  inverse mass, not a missing one — which keeps every joint two-sided and stops a
  one-sided projection existing to disagree with the two-sided one, the mistake §1.3
  recorded for plane contacts.
- **Breaking is evaluated on the host, at the step boundary.** Not a compromise:
  removing a joint is a topology change, and a topology change never happens against a
  running graph (§6.6). The load it is tested against came off the device with the
  joint, which is why joints are read back at all.

**What was measured.** 25 unit tests over the frame algebra, the seven kinds, the
limits, the drives, the force recovery and the lifetime through the seam — including a
static assertion that every `JointKind` has registered traits, so an unregistered kind
is a build error rather than a joint the solver silently declines to project. Four new
conformance scenes (a hinged door, a cone-twist chain across colours, all three kinds
on one body, and a joint set rebuilt every tick), which take the shared
`IConstraintSolver` suite to 15 and all 15 pass. Six integration scenes for the
acceptance criterion.

**Two build-lane repairs came with it, neither of them physics.** `se build`
(editor off, tests on) could not link: `sushi_sim` was only added as a subdirectory
for the editor, while the test suite links it for the scene-serializer round-trip, and
the two glTF importers it calls live under `render/` although neither touches Vulkan.
The root `CMakeLists.txt` now adds `sim` for either consumer, and the test target
compiles the two importer translation units directly rather than dragging a graphics
stack into the tests lane. Two committed test files also failed `-Werror` on unused
locals.

**What P3 still owes.** The `PhysicsAssembly` asset (§5.4, §10.2) and its editor
surface (§14), and the ragdoll wired to `Animation::RagdollBlend`. Both are
composition over machinery that now exists: an assembly is parts, joints and filter
groups instanced as a unit, and a ragdoll is a cone-twist chain with limits an
animator authors. Neither needs a new constraint kind, and the cone-twist chain the
conformance suite already runs is the ragdoll's solver half. The assembly asset is the
one item that would benefit from waiting on P4, since a part naming a cooked
`CollisionAsset` is the shape §10.2 writes out — `Collider` already carries the asset
identifier for it.

### 16.9 The assembly asset, and the ragdoll that finally reaches `RagdollBlend`

**Done on 2026-07-30.** P3's other two items: `PhysicsAssembly` with its blob and its
instancing, and the ragdoll wiring. §14's assembly editor is the phase's one remaining
piece and is deliberately not in this slice — the editor tree is being reworked
concurrently, and a panel written against the shell it is moving away from would land on
a moving target. Everything that panel needs at runtime exists.

**The asset lives at the `sim/` boundary, not under `physics/`.** Its parts are described
by `Collider`, which is already the boundary record: authored, scaled, carrying the
cooked-asset identifier P4 will fill, and already what the extract hands the physics. An
assembly expressed in physics-layer shape types would be a second description of the same
thing, which is §5.5's argument against `ColliderParams`'s flattened copies applied one
layer out.

**A correction to §4.3, stated rather than quietly made.** §4.3 sketched an
`IAssemblyService { instantiate(PhysicsAssembly) / release / part_body }`. That shape does
not survive contact with how the world runs: `set_rigid_bodies` is a **diff driven by the
ECS every tick** and removes any body whose entity is not in the list it was handed, so a
body the physics created behind the ECS's back is destroyed a frame later. §10.2 already
says the right thing — *"one entity carries the `AssemblyInstance`; child entities carry
the parts"* — so the ECS owns them, and what is left for this side is exactly the shape
§4.1 blessed for `PhysicsExtract`: a **pure translation** in one unit, tested on its own.
`instantiate_assembly` turns an asset plus a root pose plus one caller-supplied entity per
part into the `RigidBodyDesc` list and the `JointDesc` list the caller already knows how
to feed. Nothing holds state, so there is nothing to release; releasing an assembly is
destroying its entities, which the ECS already knows how to do.

**`JointDesc` split into endpoints plus `JointParams`.** A joint is two bodies and what is
held between them, and the second half is authored where the first is not yet known: an
assembly describes its joints against *part indices* and only learns which entities those
became when instanced. Factoring the parameters out is what lets the asset carry the joint
vocabulary rather than a copy of it — and a copy is how a parameter added to one ends up
honoured by a hand-built joint and silently ignored by an assembled one.

**The filter matrix is authoritative.** Each part names a group; the asset carries one
mask per group; instancing resolves that into the part's `Collider::filter`, overwriting
whatever the authored collider held. §10.2's "part 0 and part 1 do not collide with each
other" is a statement about the *assembly*, and a per-part filter that could disagree with
the matrix would be a second place the same question is answered.

**The blob refuses what its loader would refuse.** A joint naming a part that does not
exist is rejected at write time *and* on load — the second because a blob may come from an
older writer or be edited by hand, and its unchecked symptom is a joint silently projected
against part 0. That double check is what lets `instantiate_assembly` index the parts
without bounds-testing them again.

**The ragdoll: a capsule per bone, a cone-twist per joint, mass by volume.**
`Animation::RagdollBlend` is deliberately the blend and not the physics — it takes
per-joint object-space transforms "a caller already resolved from XPBD bodies". Nothing
resolved them and nothing built the bodies, so it has been complete and unreachable since
§12.4. Both halves now exist: `build_ragdoll_rig` turns a cooked skeleton into a
`PhysicsAssembly` plus part-to-joint bindings, and `resolve_ragdoll_targets` reads the
solved poses back as `RagdollJointTarget`s in the character's object space.

Four decisions inside it worth reading:

- **One part per joint that has children; leaves get none.** A bone is the segment from a
  joint to its children, so a joint with no children has no bone to be a capsule of.
  Fingertips, the top of the head and the end of every chain therefore get no part — and
  that is the answer rather than a gap, because `RagdollBlend`'s `recompose()` regenerates
  every affected joint *and its descendants*, so a leaf keeps its animated local pose
  relative to a physics-driven parent. A finger that stays curled while the arm falls is
  what a ragdoll should look like, and twenty finger capsules would be worse in both cost
  and appearance.
- **The bone is the segment to the *mean* of the children.** A hip or a chest has several,
  and picking the first would aim the torso's capsule down a leg.
- **The binding carries a bind offset, and it is load-bearing.** A capsule's segment runs
  along its own local +Y, so a part's orientation is the direction of its bone and *not*
  the joint's. The binding therefore stores the joint's pose in the part's local frame,
  measured at bind, and resolving a target is `part_world * offset` — so the capsule's
  axis convention never reaches the answer. `Collider` has no local rotation to hide the
  difference in, and adding one to make the two coincide would be reshaping a record to
  avoid storing six numbers.
- **Mass is spread by volume at one uniform density**, not by bone length. That is the
  physical model rather than an approximation of one: a thick torso comes out heavier than
  a thin forearm because it is bigger. It is also why no per-part mass is authored — a
  per-part mass is exactly what an author cannot get right by hand.

**A defect the rig found in its own limits, and it is the interesting one.** Both of a
joint's frames are derived by the shortest rotation onto the same world axis but **in each
part's own local space**, so they generally differ by a rotation *about* that axis — which
is a non-zero twist at the bind pose. A twist range of `[-t, +t]` is therefore centred
somewhere the rig has never been, and the limit fights the rest pose from the first
substep: a chain built that way pumps itself apart inside a second. The range is now
centred on the bind twist, computed exactly as the solver will compute it. The *swing*
needs no such correction and that is not luck — both axes are the same world direction at
bind, so the relative rotation is a pure twist and the swing is zero by construction.

**What was measured.** 19 unit tests over the blob (round trip, names, every refusal
including a hand-edited dangling joint), instancing (placement under a rotated root, the
filter matrix overriding the collider, density-derived versus authored mass, refusing a
partial instance) and the rig (a part per bone and none per leaf, capsules spanning their
bones and aimed down them, the authored total mass reached by volume, the bind offset
recovering the pose it was measured from, object space surviving a character placed
elsewhere in the world, the twist range centred on the bind twist, every part hanging from
the nearest ancestor with one). Two integration scenes join the acceptance file: the §10.2
car instanced from an asset rather than hand-written, and a ragdoll built from a skeleton
that falls under gravity without coming apart.

**The functional suite is 725 tests and all of them pass.** It stood at 703 before P3's
two slices.

**A third defect, and it was in a test rather than in the engine.** The conformance scene
for "all three kinds on one body" asserted that the contact took a third colour, and it
did not — because the tether it used held the crate 0.1 m clear of the ground, and a
manifold is only generated inside the 0.03 contact offset. There was no third kind at all.
It is the same failure §16.2 recorded for the first draft of that suite, which passed
against a deliberately broken solver because its constraints were laid out already
satisfied: a scene that does not exercise the thing it names will assert about it happily.
The tether now wants the crate *below* the ground so both are working, and the scene
asserts the contact exists before claiming anything about its colour.

### 16.10 Closing out P0 to P3 — the debt that was not in the roadmap

**Done on 2026-07-30**, in a session whose brief was the opposite of a feature: *no
technical debt and no unimplemented feature outside P4–P9*. That reframing is what found
most of what follows, because the roadmap's status column records phases and the debt was
not in phases — it was in rows of §16.1 that said "Partial", in a risk row nobody had
turned into a test, and in one case in a library that had been built, measured, documented
as complete, and never connected to anything.

**The audit's finding, first, because it is the uncomfortable one. P2's islands and sleeping were
not running.** `engine/domain/physics/include/SushiEngine/physics/scene/islands.hpp` was reached by
two unit tests and by nothing else. `sim/PhysicsSimulation` never partitioned a scene, nothing on
the live path ever set `BodyFlags::sleeping`, `update_motion_measure` was called nowhere at all, and
`RuntimeGraphBuilder` reported `sleeping_bodies = 0` as a literal while `islands` and
`largest_island` were written by no one. §16.4's 10 000-body measurement — 0.27 ms/tick against a
0.5 ms target — was taken against the island layer *directly*, which is a fair benchmark of that
layer and not evidence that a scene ever benefited from it.

The lesson is about how the phase was reported rather than about the code. §16.4 says "Islands and
sleeping. `engine/domain/physics/include/SushiEngine/physics/scene/islands.hpp`" and then describes
the design, which is all true; what it never says is which caller invokes it. **A module's status is
a property of its call sites, not of its contents**, and the tell was available all along: the same
section's honest paragraph about what P2 did not do lists the graph regions and not the wiring,
because the wiring was assumed to be the easy half that had obviously happened.

So it now runs. The partition is built at the end of the tick, not the start, because the
eligibility test reads a body's smoothed motion and that is a statement about the tick that
just finished — deciding at the top would put a body to sleep on last tick's evidence and
then solve it anyway. The edges are the things that actually transmit a disturbance and each
one is something the scene already owns rather than something the solver is asked for: this
tick's resolved contacts, the joints, and each cloth lattice, joined at one particle
because a lattice is connected by construction and produces the same component either way.
Waking is immediate and asymmetric — `set_rigid_pose`, a parameter change and a new joint
all call `wake_island`, so a crate teleported into a settled stack does not leave that stack
hanging in the air.

**Two consequences that had to be decided rather than discovered.**

1. **A sleeping pair stays a touching pair.** A settled stack that fell asleep has not
   stopped touching itself. If its contacts left the current list, every one of them would
   report `End` and then `Begin` again on waking, and §16.6's "begins once, then persists"
   would be false for every stack that ever settles. So the manifold is still generated and
   still listed; what it no longer does is get submitted, because both projections early-out
   on a body that is not simulated and a contact between two sleeping bodies would spend a
   slot and a colour band computing nothing.
2. **A sleeping joint keeps its last measured load.** This one arrived as two failing
   tests, and it is the more interesting of the pair. `JointProjectionT` cleared its load
   accumulators on the tick's first substep and then early-outed on non-simulated bodies —
   so the moment a door settled and its island slept, the hinge reported carrying
   *nothing*. A hanging door reporting zero load is not a missing measurement, it is a
   wrong answer, and it is exactly what a §14 load readout would have shown. The joint is
   now skipped entirely when neither body is simulated, accumulators included, so the last
   live measurement survives — the same rule §16.6 already applies to a contact `End`,
   which reports what the contact carried on its last live tick.

**The dead flag, and per-stage timings.** `PhysicsConfiguration::profiling` was written by
`sim/` and read by nobody; the editor's Physics panel drew every timing field and every
field was structurally zero. Both halves are now real. The host stages — broadphase,
narrowphase, the island pass, and the two bulk transfers — are measured with a host clock
that is not read at all when the flag is off, which is the shape §13.3 asks for. The device
half is the run report's own wall clock, which *is* the solve's cost because the composition
holds nothing but physics stages.

What is deliberately **not** there is a breakdown inside that number, and the reason is
worth recording because it is a runtime ask rather than a shortcut. The runtime reports
device time per node and names each one, but its public `add()` surface carries no label,
so every physics node arrives as `unnamed_task`; the only handle on a specific node is its
plan index, which is a compile-time internal. Splitting predict from the projection sweeps
from the velocity derivation therefore needs one small thing from below — see §18 R8 — and
inventing it here would mean making an engine-side claim about a runtime detail, which is
the specific mistake §18's correction exists to record. `PhysicsStageTimings` lost its
`velocity_ms` field for the same reason: **a field that is structurally always zero is the
same failure as a made-up timing wearing the opposite mask.**

**The reduction is the runtime's now.**
`engine/domain/physics/include/SushiEngine/physics/solver/runtime_graph_builder.hpp` hand-built its
fixed-order maximum out of two ordinary nodes, with a comment saying that adopting
`Graph::add_reduce` would be deleting them. It has been: the runtime's version folds contiguous
tiles of at most 256 left to right and folds the partials the same way, which is the same order and
the same guarantee, and it owns its own intermediate levels so the partial column is gone too.
§16.7's paragraph about building the order out of two nodes is now history rather than description.

**The risk row that became a test, and what writing it found.** §17.5 has carried
"incremental recolouring diverges from a full recolour and breaks determinism" since P0,
with the mitigation named as a test that nobody wrote — a risk row is not a feature, so it
never came up for scheduling. Writing it forced the claim to be stated precisely, and the
precise version is not the one the row implies: incremental colouring **does** diverge from
a full recolour, necessarily, because greedy over an insertion order is not greedy over a
final set, and asserting equality would be asserting that the order a scene was built in
leaves no trace. What determinism needs is weaker and testable, and is what the four new
tests assert: the colouring is valid, it is a function of the sequence rather than of the
container or the worker count, it stays inside greedy's bound, and a removal releases its
colour.

The test's first run failed, and what it caught was itself. It reported the incremental
colouring using 32 colours where a rebuild needed 25 — which looked exactly like the drift
the risk row warns about, and was not. `place()` skips a colour whose band is *full* as well
as one that is taken (§16.8's third defect), so at 512 slots over 32 colours a band holds 16
and the store climbs to the ceiling without a single conflict having forced it there. **The
measurement was of band capacity, not of colouring quality**, and a test that cannot tell
those apart cannot support the claim it exists to support. With capacity ample enough that no
band can fill, the property holds.

**And a correction to §16.9's own record.** The unused `crate` in
`tests/unit/test_convex_manifold.cpp` was reported there as a test that had lost an assertion. It
had not: at the origin that tilted hull's lowest corner sits at about y = −0.79 while the ground box
spans y ∈ [−1, 0], so the origin placement is deeply penetrating rather than clear, and asserting no
contact against it fails correctly. `crate` was a plain leftover. The test *did* lack the negative
case its own comment claims — "the alignment test is what keeps it from reporting a face that is not
touching" — so that assertion is now written against a placement where it means something: the same
hull lifted half a metre clear produces no manifold.

**The layering fix, finished properly.** §16.9 recorded the tests lane being repaired by
compiling three of the renderer's translation units into the test target, and called it a
stopgap. It is now a module: `import/` holds both glTF importers and cgltf's single
implementation unit, links nothing at all — the same rule `geometry/` follows — and the
renderer links *it* rather than owning the parser. The importers never touched a device;
they read a glTF file and write an `Animation::SkeletonBlob` or a `ClipBlob`. The cost of
the misplacement was concrete rather than theoretical: a lane that wanted a skeleton could
not link one without bringing up a graphics stack.

**Where this leaves the two remaining items, and both are named rather than quietly
carried.**

- **One `DynamicGraph` region per island is reclassified into P8**, where its sibling
  already lives. The audit that ran while wiring the islands changed the case for it: every
  node in the composition is `when(...).and_sized(live count)`, so an empty colour is not
  dispatched at all and a live one is dispatched at its live width. Regions would therefore
  not reduce the node count and would not reduce a dispatch width — the two benefits that
  remain are per-island substep counts and cross-island overlap, and P8's deliverable list
  already reads "Per-island substepping". The prerequisite the item was really waiting on
  was the wiring above, and that is done, so it is a scale item now rather than a blocked
  one.
- **The §14 assembly editor is the one P3 item still open, and it needs a seam that does not
  exist.** `ISimulation` deliberately does not expose the physics boundary —
  `engine/world/simulation/include/SushiEngine/simulation/simulation.hpp` includes the statistics
  type and says so explicitly — so the editor has no path to `IJointService` at all. An assembly
  editor is therefore not a panel: it is §5.5's `PhysicsJoint` component, its serialization, an
  `ISimulation` surface for creating and reading joints, and only then the parts list, the filter
  matrix, the load readout and the draggable hinge arcs. Sizing it honestly is what this entry is
  for; it was previously recorded as held back on the editor rework, which was true and also not the
  whole reason.

**On verification, stated plainly.** The tree builds clean. The suite was at **727 of 729**
when the two joint-assembly scenes failed on the sleeping-hinge load described above; the
fix for it compiles but **the suite has not been re-run since**, so those two scenes are
diagnosed and repaired rather than observed green. Everything else here was run: the four
new colouring tests pass, and the convex-manifold file passes at 8 of 8 with its restored
negative case.

### 16.11 P4's foundation layer, and the line it stops at

**Started 2026-07-30.** P4 is the largest phase in the roadmap by some distance — ten
soft-body stages, a rigid cooker, an import chain and an editor surface — so this
entry exists to record what is in and, more usefully, what is *not*, in the shape
§16.10 established: a module's status is a property of its call sites, not of its
contents.

**What is in, and tested.**

- **`geometry/` grew the mesh utilities P4's first line asks for.** `analyze_mesh_topology`
  measures without touching the geometry; `repair_mesh` produces a new mesh and reports
  every change. The four repair stages run in the only order that works, and the order
  is the interesting part: welding first, because which triangles share an edge is
  undecidable until coincident corners are one vertex; then the degenerate and duplicate
  drop, because **welding creates degenerate triangles** — a sliver whose two ends weld
  together becomes a line, and a pipeline that dropped degenerates first would keep it;
  then orientation, which needs the adjacency the first two stages made meaningful; then
  compaction, the only stage that renumbers.
- **Two options were deliberately removed** rather than implemented. "Do not drop
  degenerate triangles" and "do not drop duplicates" both produce a state the orientation
  pass has no answer for — a duplicated triangle makes all three of its edges
  non-manifold, which is exactly the input propagation cannot resolve. Both are now
  unconditional and both are still *counted*, which is what §8.3 stage 1 actually asks
  for: the artist is told, not protected.
- **`MeshDistanceQuery`,** a host hierarchy answering closest-point and signed-distance queries,
  because four stages ask that one question and only differ in what they do with the answer. It is
  separate from `engine/domain/physics/include/SushiEngine/physics/geometry/mesh_bvh.hpp` on
  purpose: that one answers *overlap* per tick in the engine's scalar policy, this one answers
  *distance* at cook time in `float`, below the physics, because an offline cooker must not depend
  upward to get it.
- **The distance-field bake now queries it.**
  `engine/domain/geometry/include/SushiEngine/geometry/signed_distance_field.hpp` used to document
  its O(voxels × triangles) sweep as acceptable because "a mesh large enough for that to matter is a
  mesh that wants a hierarchy, which is a later change". P4 is when that stops being true — 128³
  voxels against 50 000 triangles is a hundred billion triangle tests against §13.1's three-second
  budget — so the later change is this one, and that sentence is now false and corrected.
- **The fidelity dial**, resolving §8.2's table verbatim, with per-field overrides. The
  interpolation is **geometric** for the four resolution-like quantities and linear for the
  three small counts, and the reason is worth keeping: cost grows with the cube of a
  resolution, so a linearly interpolated dial spends its first half buying nothing and its
  last tenth buying everything. Under geometric interpolation each equal step of the dial
  is a constant *factor*, which is the only mapping under which 0.5 means something an
  artist can learn.
- **`CookingReport` and `CookingThresholds`, split.** The report carries numbers; the
  thresholds turn numbers into a verdict. That split is what lets a project accept eight
  unembedded vertices in a background prop and none in a hero vehicle without either
  decision being compiled into the cooker. A rejected cook **still has an asset**, because
  being told "this failed" without being able to look at the geometry that failed is not a
  diagnosis.
- **The three seams** (`ICookingStage`, `IMeshCooker`, `ICookedAssetStore`) and the
  content-hash cache behind the third, in memory and on a disk. The key is
  (source hash, parameters hash, **cooker version**), and the third component is the one a
  pipeline usually forgets: its absence means a bug fixed in the cooker never reaches the
  assets cooked before the fix.
- **Two decisions inside the cache worth not re-deriving.** The parameters hash is over the
  *resolved* numbers, not the dial, so dragging a fidelity slider does not re-cook at every
  pixel it passes through — two dials that resolve identically produce byte-identical assets
  and must share an entry. And the mesh content hash reads positions *through the view's
  stride* and normalizes negative zero, so the same geometry hashes identically whether it
  arrives tightly packed or inside a sixty-byte render vertex; otherwise the cache misses
  every time the import path changes shape.
- **Polyhedral mass properties with principal axes** (§8.4 item 1), which closes the hole
  `engine/domain/physics/include/SushiEngine/physics/geometry/mass_properties.hpp` has documented
  since P0: a cooked hull has no closed form, its properties are integrated over its faces, and the
  cooker is what produces faces. The integration is a signed sum over tetrahedra spanned from the
  origin, so the origin's position is irrelevant — whatever lies outside the surface is spanned
  twice with opposite orientation and cancels. The eigendecomposition is here and not later because
  `RigidBodyT::inv_inertia` is a *diagonal*, and keeping only the diagonal of a tensor that is not
  diagonal in the modelled frame silently discards the products of inertia and produces a body that
  tumbles plausibly and wrongly.
- **Jacobi and not the closed-form cubic**, and the axis assignment is not cosmetic. Each
  coordinate axis takes the eigenvector most aligned with it, so a box modelled
  axis-aligned reports the identity rotation instead of an arbitrary permutation of its own
  axes. The tests pin that, and one of them rotates by **thirty** degrees rather than
  forty-five: at forty-five each eigenvector is equally aligned with two axes, the
  assignment is a genuine tie, and asserting one arm of it would make the test lie about
  what the code guarantees.

**What is not in.** `CollisionCooker` and the `.sushicollision` blob, `SoftBodyCooker` and
its ten stages and `.sushisoft`, the `IMeshPostProcessor` import chain, and the editor bake
surface. **P4's acceptance criterion is therefore not met** — dropping a mesh into the
project still produces nothing — and the phase status says "In progress" rather than
anything warmer.

**One finding that changes the shape of the rigid cooker, and is worth having before it is
written.** `ConvexHullView` carries only `vertices`, `vertex_count`, a placement and a
`convex_radius`; GJK's support function scans the vertex array. A cooked convex hull is
therefore a **point set**, not a face topology — so the decomposition needs no
face-building quickhull to produce a *correct* asset, only to produce a compact one, and a
superset of hull vertices costs support-query time rather than correctness. Mass properties
do not need the hulls at all: §8.4 item 1 integrates over the closed source mesh, which is
what `mesh_mass_properties` already does. That collapses the riskiest-looking part of §8.4
into a vertex-budget selection problem whose error is measurable with
`max_protrusion_distance`, which exists.

**On verification, stated plainly.** `se build` was run once at the start of the session and
returned exit code 0, which establishes only that the tree was clean **before** any of this
landed. Everything above is **unbuilt and unrun**: the user asked for no builds partway
through, so not one line of the new code has been compiled and not one of the new tests has
executed. Treat every claim in this section as a design statement, not an observation. The
suite also still has §16.10's open item — it was at 727 of 729 with two joint-assembly
scenes repaired but never re-run.

### 16.12 The rigid cooker, and the number it optimizes

**Done 2026-07-30**, immediately after §16.11 and on the finding recorded at the end of it.
A mesh now cooks to a `.sushicollision` a body can collide as.

**The finding did most of the work.** `ConvexHullView` holds a vertex array, a placement and
a convex radius, and `support()` scans the vertices — so a cooked hull is a **point set**,
and nothing in the runtime reads a hull face. Two consequences that between them removed
most of §8.4's apparent difficulty:

1. **The vertex budget comes before the hull, not after.** Selecting at most `N` vertices of
   a part and hulling *those* bounds the hull build at `N` points, where brute-force
   enumeration of triples is affordable and short enough to be read and believed. An
   incremental hull that is subtly wrong produces a collider that is subtly wrong, which is
   the least findable bug this pipeline can ship.
2. **Simplification can only make a collider thinner.** Every selected point is a real mesh
   vertex, so the selected hull sits inside the part's true hull. A thin collider is a body
   that sinks a millimetre; a fat one is an invisible wall. The budget errs in the direction
   that is merely imperfect rather than the one players report as a bug.

**The hull build's one real trap is coplanarity, and it is why triples find *planes* rather
than faces.** The four corners of a square face all satisfy every one of their own triples,
so emitting a triangle per accepted triple covers that face twice and doubles its
contribution to the volume — a hull of a cube's eight corners would report a volume of two.
So accepted triples are deduplicated into supporting planes, and each plane is then tiled
once by fanning its own points in angular order about their centroid. The test asserts the
volume, because the volume is the assertion that catches this.

**Concavity is measured as the number that gets reported.** A part's concavity is
`Geometry::max_protrusion_distance` from its hull's surface to the source mesh — which is
not a proxy for §7.6's error, it *is* §7.6's error, "the collider is three centimetres fatter
than the mesh", in local units. The decomposition splits the part that departs furthest,
picks whichever of three axis-aligned planes leaves the better *worse* child (the report is a
maximum, so improving the better child moves nothing), and stops when a split buys no
reduction. Optimizing the reported quantity directly avoids the usual arrangement where a
volume ratio is minimized and a distance is reported.

**Mass properties never touch the pieces.** §8.4 item 1 integrates over the closed source
mesh, so the decomposition's approximation reaches what a body bumps into and never what it
weighs or how it spins.

**And that is where a real bug surfaced, found by writing the test rather than by reading the
code.** `mesh_mass_properties` checked only that the volume came out positive. The divergence
theorem given an *open* surface still returns a number — the cone fan from the origin — and a
unit box missing one face integrates to five sixths of its volume. Plausible, wrong, and
untraceable three weeks later. Closure is now checked, and the refusal is the point: zero mass
already means "keep the authored value" at the extract, so a single-sided wall degrades to
hand-authored numbers instead of to a confidently incorrect one. A **self-intersecting** mesh
is closed and does integrate, with its overlap counted twice; that is inherent to the input,
and the honest place to notice it is the repair report's component count.

**Three things deliberately left undone rather than half-done.**

- **`convex_radius` is zero on every cooked piece.** §5.2's inflation is only sound alongside
  shrinking the vertices by the same amount, which means offsetting the hull inward along its
  own planes. Setting the radius without the shrink would make every cooked collider fatter
  than the mesh it came from — the one error §7.6 exists to keep at zero — so it stays zero
  until the shrink is written.
- **The distance field is a full brick, not the narrow band §8.4 item 3 asks for.** At 128
  voxels per axis that is eight megabytes an asset. Narrowing it is a *format* change rather
  than a baking change, so it is named here instead of approximated.
- **The union volume is estimated by summing the pieces**, so a region two pieces both cover
  counts twice and `volume_error` reports more collider than there is. That is the safe
  direction for a threshold, and it is documented as an estimate rather than presented as the
  union.

**What the cooker does end to end.** Five stages, each an `ICookingStage` in a list so a sixth
is an object rather than a case: repair, mass properties, geometry, distance field, serialize.
Only the third differs between the two shapes — convex pieces, or the exact triangle hierarchy
for authored-static geometry, whose error fields are genuinely zero rather than unmeasured
because the collider *is* the mesh. The cache check is the cooker's own, so "an unchanged mesh
is never re-cooked" is a property of the pipeline and not of every call site remembering to
ask; a cache hit reports what the asset carries and leaves the source topology **unmeasured**,
with `served_from_cache` as the discriminator, rather than filling in a clean-looking report
nothing looked at. A cached blob this build cannot validate is evicted and re-cooked, because
trusting it would be trusting bytes this build cannot read.

**On verification.** Written as unbuilt and unrun; **run on 2026-07-30, and it is now
observation rather than design.** The five test files compile and execute at **61 of 61**,
including the dirty-mesh corpus P4's acceptance criterion names (exploded, mixed winding,
inside out, self-intersecting, degenerate-and-duplicate, open shell). This did not need the
project's build: `geometry/` and `cooking/` name `SushiRuntime` nowhere, so the suites link
against nothing but their own two modules and gtest.

**What the first execution found — three defects, all three in the tests.** Worth recording
because the ratio is the interesting part: the pipeline the entry above describes was correct
everywhere the tests disagreed with it.

1. **One test did not compile at all**: `FilesystemCookedAssetStore store(std::string());` is
   a most-vexing-parse, so the whole file was a function declaration where it read as a
   variable. Nothing about the cache was being asserted, and nothing said so — this is the
   failure mode a never-executed test file has, and the only detector for it is execution.
2. **Two volume expectations were off by exactly 2×**, in `MatchesTheClosedFormForABox` and
   `LeavesAConvexShapeAsOnePiece`, and the same way in both: `box_mesh` takes **half**-extents,
   the expected literal converts them to full extents, and the *x* factor was left undoubled
   in each (`0.5 * 3.0 * 0.5` for a box whose volume is `1.0 * 3.0 * 0.5`). Two independent
   files carrying the identical slip is the tell that it came from the convention and not from
   a typo.
3. **`DiagonalizesARotatedBox` held absolute tolerances a float32 vertex buffer cannot meet.**
   The residual was 1.6e-5 on a mass of 1200 — 1.3e-8 relative — and which side of the seam it
   was on was *measured*, not assumed: integrating the same float32 corners in long double
   with an independent tetra-fan formula reproduces the cooker's answer to its last two digits
   (1199.9999839774744 against 1199.9999839774746). The integrator is exact; a rotated corner
   simply is not representable in float32 where an axis-aligned one is, which is why the
   unrotated box in the same file holds absolute bounds and this one now holds relative ones.

Still **not** written, unchanged by the above: `SoftBodyCooker` and `.sushisoft`, the
import-processor chain, and the editor bake surface.

### 16.13 The soft-body cooker, and three bugs the first run found

**Done 2026-07-30.** `.sushisoft` exists: a mesh cooks to a tetrahedral interior with the render
mesh embedded in it, a level chain, a rest-shape distance field, and the parameters that produced
all of it. Six `ICookingStage` objects cover §8.3's ten numbered steps — the mapping is tabulated in
`engine/domain/physics/include/SushiEngine/physics/cooking/soft_body_cooker.hpp` rather than left to
be counted, and stages 2 through 5 and 7 are one object because they share the voxel grid and
publishing that grid as an interface would be a worse violation of §3.2 than the grouping is of
§8.3's numbering.

**Two deviations from §8.3, named rather than glossed.**

1. **The lattice is Kuhn's six-tetrahedra-per-cell triangulation, not body-centred cubic.**
   Kuhn conforms across cell faces *by construction* — the diagonal a shared face splits
   along is the same computed from either side, verified by hand for all three axes — so
   there is no parity bookkeeping to get wrong. Every element is congruent, which makes the
   worst element quality a lattice constant (measured at roughly 0.42, against 1.0 for a
   regular tetrahedron) rather than a property of the input. Body-centred cubic gives a
   better constant and is a swap inside `build_tetrahedral_mesh`.
2. **Boundary conforming is snapping, not marching tetrahedra.** Snapping cannot add a
   vertex, so a feature thinner than a cell is lost rather than resolved. The voxel
   resolution controls it and the report's accuracy number measures it.

**The division of labour in the voxelizer is the part worth keeping.** The flood fill answers
the *global* question — is this region enclosed — topologically, with no reliance on a
surface normal, which is why a self-intersecting or open-shelled mesh still produces a solid
body and is the whole of §8.3 stage 2's argument. But the fill cannot pass through the marked
band, so every band cell comes out "not reached". The band, **and only the band**, is
therefore decided by the local question: is this cell's centre behind the surface. One sign
lookup, in the one place where it is the best information available, and an error there costs
a single cell at a crease.

**Three bugs, and the first execution found all three.** This is the entry's real content,
because two of them were in the pipeline and neither was visible by reading.

1. **The simulation mesh was a full cell fatter than the source in every direction.** The
   surface band was marked by a circumradius test plus a one-cell bounds pad, which marks a
   shell the surface never enters — and a marked cell is treated as part of the body. A unit
   box measured **1.94** against a true volume of 1.0. The fix is an exact test: the closest
   point of a triangle to a cell centre lies inside the cell exactly when the triangle meets
   the cell, so comparing each component against half the cell is exact rather than
   conservative. The box then measures 1.00000 and 216 interior cells for a resolution of 6,
   which is 6³ — the right answer for the right reason.
2. **The render-mesh binding table was indexed in the wrong order, and this is the one that
   matters.** It was built against the *repaired* mesh, and `repair_mesh` welds and compacts,
   so it renumbers. Every render vertex would have been driven by some other vertex's
   element. The symptom is not a torn mesh but a **shuffled** one — the test caught it as a
   vertex reconstructing at `y = +0.7` where the source has `-0.7`, an exact mirror — so it
   reconstructs plausible geometry and is invisible until somebody looks at the model. The
   bindings are now built against the source view, in source order, honouring its stride.
3. **The level chain silently collapsed to one level.** The element-count target scales the
   finest level's resolution *down* (16 to 3, for a 200-element target), and the coarser
   levels were halving the **authored** resolution — so level 1 came out finer than level 0,
   the "a level that is not coarser is a second copy" guard fired, and the chain refused to
   grow. Coarser levels now halve the *effective* resolution, which
   `TetrahedralizationReport` reports for the purpose — and reporting it also satisfies
   §8.2's "the inspector shows what the dial produced".

**A fourth defect surfaced while fixing the first.** Marking the band with an exact
comparison decides the equidistant case — a face lying on a cell boundary, which is what a
mesh modelled on round numbers produces constantly — by whichever way the grid arithmetic
rounded, and it rounds differently for different faces of the same box. One wall of a cube
marked both its layers and the opposite wall marked one, so an open box produced **zero**
interior cells. The tie is now included with a tolerance and the outer layer is removed by
the sign pass, so the outcome is a decision rather than a rounding artefact.

**What is deliberately not done.** The `.sushisoft` asset has **no runtime consumer**: the
finite-element model, the plasticity and the fracture that read a tetrahedral mesh are
**P6**, and `physics/soft/` today holds a mass-spring box lattice and no element solver. So
this asset's status is "produced and validated, not yet consumed" — recorded in advance this
time rather than found by a later audit, which is what §16.10 was written about. Of §8.6's
five binding invariants, 1 and 2 are asserted here (every render vertex bound; the
reconstruction reproduces the source at rest, *through the serialized blob* rather than
through the cooker's own intermediate state, since a rebasing bug lives exactly in that gap).
Invariants 3, 4 and 5 are properties of a running solve and belong to P6; claiming a test for
them here would be claiming a test for code that does not exist.

**On verification — and this section is measured, not designed.** **78 of 78 pass**, across
the six P4 suites and the pre-existing signed-distance-field suite, the last of which
confirms that routing the bake through `MeshDistanceQuery` is behaviour-neutral. Compiled
clean under `-Wall -Wextra -pedantic`. Run **without** the project build, through the
standalone harness, which works because nothing in `geometry/` or `cooking/` names
`SushiRuntime` — that independence is a property of the layering and is the reason this loop
is measurable at all.

### 16.14 The import chain — "without a manual step", finally

**Done 2026-07-30.** §8.1's chain exists, so a mesh path in produces a `.sushicollision` and,
if the profile asks, a `.sushisoft`, cached, on a worker thread, with nobody pressing
anything. That is the clause of P4's acceptance criterion the two cookers could not satisfy
on their own.

**What §8.1 asks for is three words, and each is a mechanism rather than an intention.**
*Ordered* — a processor names its own position, so the chain's sequence is a property of the
processors and not of the order somebody registered them in; the test registers them
backwards to prove it. *Registered* — the chain holds objects it was given, so §11's
node-beam cooker arrives as one `add()` call at whatever point P7 writes it. *Open/closed* —
the chain never asks *what* a processor is, only whether the profile wants it, which is the
one decision that would otherwise become a `switch` every new asset kind has to edit. The two
shipped processors are one template over the cooker, because the only things that differ
between them are which cooker they hold, which profile flag they read and what they are
called, and writing that twice is writing the same bug twice.

**The mesh loader is a seam, and that is what keeps `cooking/` honest.** The chain takes
triangles; getting them out of a file is the consumer's job, wired in as a `MeshLoader`. So
`sushi_cooking` still links nothing but `sushi_geometry` and Threads — no cgltf, no device —
and the chain is testable against a box built in memory. The glTF side is
`Geometry::import_gltf_mesh`, declared in the neutral geometry surface and implemented in
`import/`, the one module that links cgltf, which is the same split
`Animation::import_gltf_skeleton` already uses. It walks **nodes** rather than meshes, because
a node is what carries a transform and a model assembled from instanced nodes imported
mesh-by-mesh arrives as a pile of overlapping parts at the origin.

**Three decisions the tests exist to pin down.**

1. **A failing processor does not stop the chain.** A mesh whose tetrahedralization cannot be
   filled must still get its collider, or one bad soft-body cook silently costs the asset its
   collision too — and what the artist then sees is a crate that has stopped being solid,
   with nothing anywhere saying why.
2. **An import that produced nothing still comes back.** `CookedImport::loaded` is false and
   the products are empty, rather than the whole import being dropped. Same reasoning: a
   silent absence is the failure that costs a day to trace.
3. **An empty per-asset override removes its entry** rather than storing a no-op, so "reset
   to project default" and "was never set" are one state. Otherwise a project accumulates
   entries that do nothing, and a changed default silently fails to reach the assets that
   carry them.

**The override is deliberately small — four fields.** Fidelity, and which of the three kinds
of thing this mesh is. Everything else in `CookingParameters` is engineering policy that
belongs to the project, and letting the vertex budget or the sampling order be overridden per
asset produces a project where no two assets were cooked comparably and no measurement means
anything.

**On the service.** One worker, not a pool. A cook is already allocation-heavy and
cache-hostile so several at once mostly contend, and — the real reason — a single worker makes
the result order a function of the submission order, which keeps an import log readable and a
test deterministic. The caller polls; no callback fires on the worker, because a callback
landing on a background thread is one that eventually touches a renderer from the wrong one.
The destructor abandons the queue rather than draining it, since closing the editor must not
take as long as the largest pending cook and the cache means nothing is lost.
`CookingServiceStatus` has no "current processor" field: the chain does not report which
member is running, and a field that is structurally always empty is the same failure as a
made-up one — the stage name is also the better of the two, since "Tetrahedralize" says both
what is happening and which cooker is doing it.

**On verification.** **87 of 87 pass** across the eight P4 suites, the nine import-chain tests
green on their first run. Compiled clean under `-Wall -Wextra -pedantic`; the glTF importer
compiles clean against cgltf separately, since it is the one piece the standalone harness
cannot link into the same binary. Still run without the project build.

**What remains of P4 is the editor bake surface (§14), and one thing it needs.** The collider
inspector wants the cook report, a Re-cook button and the collision geometry drawn as an
overlay; the soft-body panel wants fidelity, progress and the report. The report and the
progress are both available now — `CookingService::status()` is shaped for exactly that
readout. The overlay needs hull *faces*, which the asset deliberately does not store because
the runtime never reads one; `build_convex_hull_mesh` rebuilds them from the stored point set,
so the panel has a supported route rather than a reason to change the format.

### 16.15 The bake surface, and P4 closed

**Done 2026-07-30.** The last P4 item. §14's collider and soft-body inspectors, as one **Bake**
window (Analysis ▸ Bake) rather than two, because both show the same thing about the same
asset — what the cook made and what it cost — and splitting them would have an artist checking
two places to find out whether a crate is solid.

**The split that matters is not between the two inspectors, it is between the panel and its model.**
`CookBakeState` links no UI and holds every decision: which profile applies, whether Re-cook has to
evict, when the overlay's geometry is rebuilt, whether a re-bake updates a row or adds one.
`applications/editor/source/physics/cook_bake_panel.cpp` is widgets over it. That is the arrangement
the scene serializer and the command history already use, and the reason is that a bake surface's
interesting behaviour is exactly the part an ImGui call cannot reach — so the seven tests are
against the model and the untestable file is kept small.

**Re-cook needed something that did not exist, and the shape of the fix is the point.** The
button's whole purpose is to get past a cache entry whose key has *not* changed, which is the
case when the **cooker** changed rather than the mesh — the one staleness a content hash cannot
detect. Evicting needs the key; the key needs the source's content hash; the source is behind
the loader on the worker thread. The first attempt at this was dead code that computed keys
from an empty mesh and discarded them, which is worth recording because it looked plausible.
The fix is `IMeshCooker::cache_key`: the cooker owns its version, the version is the part of
the key nothing else can derive, so the cooker is what publishes it. Then `ImportProfile` gains
a `force_recook` flag — on the *profile* and deliberately not in `CookingParameters`, because a
flag that reached `cooking_parameters_hash` would not bypass the stale entry, it would file the
result under a different key and leave the next ordinary cook reading the stale one again — and
the processor evicts at the first point where the mesh and the cooker that owns the key are
both in hand. The test asserts all three states: a plain bake is served from the cache, a
re-cook is not, and the bake after it is served again, so Re-cook did not simply switch the
cache off.

**The overlay is drawn from rebuilt faces, not stored ones.** §14 asks for the collision
geometry over the scene "so 'the collider is not the mesh' is *visible*", and the asset
deliberately stores no hull faces — the runtime never reads one, so storing them would be
memory in every shipped asset for a debug view. `collision_asset_wireframe` rebuilds them with
the same routine that produced them at cook time, which is why it cannot disagree with what was
cooked, and `CookBakeState` rebuilds on *selection* rather than per frame because that is a
cook-shaped cost. A flat piece has no faces and is drawn as a star from its centre, since a
piece that shows nothing reads as a piece that is not there.

**One duplication removed rather than added to.** `project_to_screen` existed identically in
the skeleton debug draw and in the transform gizmo, and the overlay would have been a third
copy. Three definitions of a projection is three places for a near-plane rule to drift, and the
symptom of drift is one overlay clipping a line another draws across the whole viewport. It now
lives once, in `applications/editor/source/core/viewport_projection.hpp`.

**What §14 lists that this is not.** §14 covers seven surfaces and only two are P4's. The assembly
editor is P3's one outstanding item and still needs §5.5's `PhysicsJoint` component first; the
vehicle editor is P7; physics debug draw is P8/P9; the profiler panel already exists. The soft-body
panel's *debug views* — wireframe tetrahedra, stress and plastic-strain heat maps — are listed under
§14 but two of the three read quantities a running finite-element solve produces, which is **P6**.
The report, the fidelity dial, the progress and the collider overlay are what P4 can honestly
deliver, and they are delivered.

**On verification. 94 of 94 pass** across the nine P4 suites — the seven bake-state tests green
on their first run, including the eighteen-unique-edge assertion that catches a wireframe dedup
that does not work. The three ImGui translation units cannot link into the standalone harness,
so they were compiled `-fsyntax-only` against the real imgui headers, clean under `-Wall
-Wextra`; the two files whose duplicated helper was removed were re-checked the same way.

**P4 is closed.** The clause that has been the phase's headline since §8's first line — *drop a
mesh into the project and it is soft-body ready, and you can set how accurate that is* — is
true: a path in produces a `.sushicollision` and, at the profile's word, a `.sushisoft`, at the
authored fidelity, cached, off the main thread, with a report and a measured error. The two
things that remain are **not** P4's and are recorded as such: nothing consumes `.sushisoft`
until P6 writes the element solver, and the editor's soft-body debug views need the same.

### 16.16 Conservative advancement, the SDF narrowphase path, and the three regression scenes

**Written 2026-08-01, not yet built or run — read the last paragraph before touching this
phase.** This session closed the three items §16's P5 row had left open. What follows is what
each one is, in the same detail the rest of §16 holds every other item to, and then the one
thing that is different about this entry.

**Conservative advancement (§7.5 item 2).**
`engine/domain/physics/include/SushiEngine/physics/collision/conservative_advancement.hpp` is Redon
et al.'s method, over the engine's own existing `collide_convex`
(`engine/domain/physics/include/SushiEngine/physics/geometry/gjk.hpp`): a distance query at an
advanced pose, and a lower bound on how fast that distance can shrink before the next query — the
pair's closing speed along the current normal, plus a conservative allowance for whatever either
body's own rotation could add, regardless of which way it happens to be turning
(`collision_shape_bounding_radius`, an upper bound on how far a rotation can swing a surface point).
The bound can only overestimate the true rate, so the search can never step past the real time of
impact.

The reason this is a *second* tier and not a replacement for tier 1's speculative contacts: tier 1's
widened, once-per-tick manifold already covers straight-line fast motion for free, and covers it
well — a body translating toward a fixed feature keeps the same closest point for the whole
approach, so the manifold generated at the tick's start pose is the right one. What it cannot see is
rotation: `ContactProxy::speculative_margin`
(`engine/world/simulation/include/SushiEngine/simulation/physics_simulation.hpp`) is built from
`velocity * delta_time` alone, with no angular contribution at all, so a body that is only spinning
— a blade, a wheel's rim — can sweep a point into something tier 1 never widened its query toward.
`needs_conservative_advancement` (the same header) is the trigger: a body's motion this tick,
translation plus the swept arc of its own rotation, compared against a fraction of its own thinnest
dimension — the same state-derived, `BodyFlags::continuous_collision`-widened decision §7.5
describes. Tier 2 only runs when tier 1's manifold came back empty *and* the trigger fired, so an
ordinary tick pays nothing extra, and when it does run its witness points and normal — sampled at
the real advanced pose, not guessed at the tick's start one — become the manifold tier 1 would have
built, through the same `make_point_manifold` / `refresh_manifold` path every other narrowphase
routine uses. `tests/unit/test_conservative_advancement.cpp` checks the closed-form sphere-sphere
case (an exact time of impact against arithmetic, the same oracle `tests/unit/test_gjk.cpp` uses),
the two honest terminations (separating, and already-touching-at-zero), and the property that
motivates the whole file: identical geometry, zero translation, spinning versus not — impact found
with the spin, none without it.

**The SDF narrowphase path (§7.5, "as a first-class narrowphase path").** A signed-distance field is
not a bounded convex set, so it was never going to be a `support()` overload — the same treatment a
half-space plane already gets in the dispatch table, for the same reason. `SdfCollider`
(`engine/domain/physics/include/SushiEngine/physics/geometry/shapes.hpp`) is a non-owning view over
a cooked field's baked cube, exactly the reference-not-own pattern `ConvexHullView` already uses for
a hull's vertices, and it comes with a nearest-voxel sampler and a central-difference gradient —
which doubles as the outward surface normal, by the eikonal property, valid whether the query is
outside the solid or already deep inside it.
`engine/domain/physics/include/SushiEngine/physics/collision/sdf_manifold.hpp`'s
`generate_convex_sdf_manifold` is deliberately a single-point manifold, two passes of the field (a
rough direction from the shape's centre, then a refined sample at its support point toward the
solid) — the same scope `generate_sphere_sphere_manifold` and `generate_obb_sphere_manifold` already
keep. It is registered in the dispatch table
(`engine/domain/physics/include/SushiEngine/physics/collision/narrowphase_dispatch.hpp`'s
`register_convex_row`) for every convex shape against `ShapeType::signed_distance_field`, both
orders, the same way a plane is.
`engine/domain/physics/include/SushiEngine/physics/cooking/collision_asset.hpp` gained
`collision_asset_field`, mirroring `collision_asset_hull`, so a validated `.sushicollision` blob's
field can be placed in the world without a second copy of the placement arithmetic.
`tests/unit/test_sdf_manifold.cpp` checks the gradient against a field with a *known* closed form (a
flat plane, whose signed distance is just a coordinate), both dispatch orders agreeing with each
other, `world_bounds` following placement, and the empty-field refusal.

**What this does not do, deliberately.** Nothing at the `sim/` authoring layer names
`ShapeType::signed_distance_field` yet — `Simulation::Collider`
(`engine/world/simulation/include/SushiEngine/simulation/collider.hpp`) has no `SignedDistanceField`
member of `ColliderShape`, and a `CollisionAssetId` collider still falls back to its bounding box
exactly as it did before this session (§5.5's placeholder, unchanged). Wiring a cooked asset's field
as a *selectable collider kind* an author can put on a body is a content-authoring decision — which
assets get a field-backed collider instead of (or alongside) convex decomposition, and how the two
coexist for one body — that belongs with whoever designs that authoring surface, not with the
narrowphase routine that now exists underneath it. The path is real, registered, and tested; nothing
yet asks for it by name outside a test.

**The three §15.4 regression scenes**, `tests/regression/test_penetration_contract.cpp`:
a stack of crates checked against its analytic resting height rather than merely "does it look
stacked"; a sphere driven to 50, 100, and 200 m/s against a 1 cm static plate — the exact figure
§15.4 and this phase's own acceptance criterion name — built by ramping a strong acceleration for a
few ticks while still far above anything and then coasting at that exact speed, since the live
`IRigidBodyService` seeds a new body at rest and has no velocity-setting call yet (a real, if
narrow, gap in §4.3's interface list — recorded here rather than worked around by reaching past the
seam); and the cooker's Hausdorff error checked against two cases with known answers — zero for a
box (an exact hull) and a real, bounded, positive number for an L forced to one convex piece
(`CookingParameters::convex_piece_count = 1`, which reuses the exact scenario
`Unit_ConvexDecomposition.SplitsAConcaveShapeAndTheErrorFalls` already proves produces a concavity
over 0.1 — chosen over the L's *default* piece budget deliberately, because that same test proves a
generous budget can split this particular L back into close to its original two boxes, which would
have made the scene's outcome depend on how good the decomposer happens to be rather than on whether
the report plumbing works).

**Update, same day: built and verified.** The code above was written without a build available —
builds on this machine were the project owner's to run, not this assistant's, for the duration this
stood (see [[dont-run-builds-myself]] in the assistant's own memory, for a later reader who finds
that surprising). It was written and re-read line by line against the header files it calls into
(`engine/domain/physics/include/SushiEngine/physics/core/rigid_body.hpp`'s
`apply_angular_correction` for the pose advance,
`engine/domain/physics/include/SushiEngine/physics/collision/manifold.hpp`'s anchor convention, the
narrowphase dispatch table's registration pattern), and the two-phase template-lookup hazard that
would have broken an early draft (`safe_normalize` called before its own definition, from code
inserted earlier in `engine/domain/physics/include/SushiEngine/physics/geometry/shapes.hpp` than its
first existing use) was found and fixed by inspection rather than by a compiler diagnostic. The
project owner then ran `se build` and `se test` and confirmed: it builds, and the tests pass. P5 is
closed.

**Addendum, same day: a real bug found and fixed after that green run.** Re-running the suite
surfaced a genuine failure in `FastSphereDoesNotTunnelThroughAThinPlate` (alongside two unrelated,
pre-existing failures from a same-day SushiRuntime merge that flipped the rebalancer's default).
Temporary tracing (added to
`engine/domain/physics/include/SushiEngine/physics/collision/broadphase.hpp`/`engine/world/simulation/include/SushiEngine/simulation/physics_simulation.hpp`
and removed once diagnosed) found the actual cause: `needs_conservative_advancement` was computed
correctly but never told to the *broadphase* — `refresh_contact_index` widened the narrowphase's
speculative margin but never called `contact_index_.set_proxy_state()` to fold
`BodyFlags::continuous_collision` into the proxy's flags, so the broadphase kept using its ordinary
fixed 2×-one-tick lean, which for a fast body can lag a tick or two behind. Since `submit_contacts`
only iterates pairs the broadphase already proposed, a pair it never proposes gets neither tier of
§7.5 no matter how wide the narrowphase's own margin is. Fixed by setting that flag whenever the
trigger fires. A second, narrower bug at the highest tested speed (200 m/s): tier 1 samples geometry
only at the tick's discrete start pose, and at high enough speed that sample can already be
embedded, so the deep-penetration fallback resolves to the *far* face — a wrong-side answer, not a
miss. Fixed by no longer gating tier 2 on "tier 1 found nothing": it now runs whenever the trigger
fires and overrides tier 1's manifold when it finds an impact, since conservative advancement's
time-of-impact search does not sample at a single discrete pose the way tier 1 does. The first fix
was verified by the trace that found it and holds: 50 and 100 m/s pass. **The second fix does not
fully hold.** Rebuilt and re-run the same day: 200 m/s still fails, landing at `y ≈ -0.105` — the
exact mirror of the correct resting height, meaning tier 2 is still resolving to the wrong side of
the crossing at this speed, not merely missing it. The "override tier 1 whenever the trigger fires"
change is necessary (it is what fixed 50/100 m/s) but evidently not sufficient at 200 m/s; the
remaining wrong-side answer at the highest tested speed is an **open, unfixed bug**, not a
documentation gap — the discrete-pose diagnosis above narrowed the 50/100 m/s case correctly, but
the 200 m/s case needs its own fresh trace (the earlier one was removed once the first fix looked
sufficient) before it is touched again. §15.4/§16's acceptance criterion — "nothing tunnels at the
tested speeds" — is therefore **not yet met**; P5's regression suite is red at one of its three
tested speeds. Do not mark this row "Complete" again until that trace happens.

### 16.17 P6-A: the neo-Hookean element, host-only

**Written 2026-08-01, in the same session as §16.16, same caveat: not built or run — read
the last paragraph before touching this.** P6 is the largest phase in this document by a wide
margin — nine sub-items (§9.1 through §9.7, the mesh binding, cloth bending, the cosmetic
column), each of which is its own real piece of work. This entry closes the first and most
load-bearing one: the constitutive model every later P6 item reads from.

**A genuine architectural finding first, because it changed the shape of everything after it.** §6.3
describes adding a constraint kind as "a trivially-copyable descriptor POD exposing
`body_a`/`body_b`" — true for every kind that exists today (distance, contact, joint), all strictly
two-body, and confirmed by reading
`engine/domain/physics/include/SushiEngine/physics/solver/constraint_store.hpp`,
`engine/domain/physics/include/SushiEngine/physics/solver/incremental_coloring.hpp`, and
`engine/domain/physics/include/SushiEngine/physics/solver/graph_coloring.hpp` line by line:
`ConstraintStore::place`, `IncrementalColoring::assign/take/release`, and the free function
`color_constraints` all take exactly two body indices, with no N-body overload anywhere. A
tetrahedral element's two constraints each touch **four** bodies — the tet's own vertices — so
registering it as a constraint kind the way a joint was registered in P3 is not available off the
shelf; it would require generalizing the colouring and store machinery to N bodies first, which is
real, separable work of its own. Asked which way to go, the project owner chose **host-only first**:
get the constitutive model correct and tested against elasticity theory as a small, self-contained
reference solver, and take on the device/parallel-colouring generalization later, in the same
relationship `HostXpbdSolver` already has to `RuntimeGraphBuilder` — except for now the host solver
is the *only* implementation, not a conformance mirror of one. That decision is why nothing below
touches `physics/solver/` or `physics/scene/` at all.

**What exists.**

- `engine/domain/physics/include/SushiEngine/physics/soft/soft_body_material.hpp` —
  `SoftBodyMaterialT<T>` (§9.2's eight fields exactly), `lame_parameters()` deriving `mu`/`lambda`
  from Young's modulus and Poisson ratio, and the five named presets (rubber, foam, soft tissue,
  sheet steel, aluminium) at textbook order-of-magnitude values, each commented as such rather than
  presented as measured data.
- `engine/domain/physics/include/SushiEngine/physics/soft/fem_element.hpp` — `FemTetrahedronT<T>`,
  the FEM analogue of `XpbdDistanceConstraintT`: four vertex indices, `Dm^-1`'s three columns (the
  cooked rest state — `Cooking::SoftBodyAssetView::rest_inverse`, unchanged since P4), the rest
  volume, and two accumulated Lagrange multipliers. It also carries a **plastic** copy of the three
  columns, read by every projection instead of the original rest state and left equal to it until
  §9.4 (P6-C) begins writing to it — chosen so the projection code needs no branch for "has this
  element ever yielded," now or after P6-C lands.
- `engine/domain/physics/include/SushiEngine/physics/soft/fem_projection.hpp` — the deformation
  gradient `F = Ds·Dm^-1`, the deviatoric constraint `‖F‖_Frobenius - √3` and the hydrostatic one
  `det(F) - 1 - μ/λ`, both with their four-vertex gradients, and the N-body XPBD update those
  gradients drive. The file's own comment records the gradient derivation in full — which column of
  `Dm^-1` weights which vertex's contribution, and why vertex 0's gradient is the negated sum of the
  other three (translation invariance, since every edge is `x_k - x_0`) — because a transposed index
  here is invisible until a cantilever bends the wrong amount, and there is no compiler that would
  catch it.
- `engine/domain/physics/include/SushiEngine/physics/soft/finite_element_model.hpp` —
  `FiniteElementModel<T>`, a flat particle array (`RigidBodyT`, the same point-mass-no-rotation
  particle every soft body in this engine already uses) and a flat element array, stepped with
  §0.2's schedule: one Gauss-Seidel sweep per sub-step, fixed element order, both constraints per
  element before the next element — deterministic per §0.5 by construction.
  `build_finite_element_model` is **the first thing that has ever read a `.sushisoft` blob**
  (§16.11's own words: "nothing consumes this yet"), turning one simulation level's already-rebased
  tetrahedra and rest state into a working model. Placement is translation-only for now, on purpose:
  rotating a placed body's particle positions without rotating the rest state they are measured
  against would read as instantaneous strain at spawn, and getting that right is separate work this
  function declines to paper over.

**What was checked, given there was no build to check it against.** Every gradient formula was
re-derived independently from `F`'s column definition (not copied from memory of the paper's own
index conventions) and cross-checked two ways: against the standard literature identity
`∂C/∂x_i = (1/C)·F·(row i-1 of Dm^-1)`, and, in `tests/unit/test_fem_projection.cpp`,
against a central finite difference of the constraint value itself for a generic, non-symmetric
configuration — the one test that would catch a sign or transposed-index error independent of
whether a hand-picked closed-form case happens to be symmetric enough to hide one. Three closed-form
cases back it further: the rest configuration gives `F = I` and a deviatoric value of exactly zero;
a uniform scale gives `F = s·I` with the expected norm and determinant; and a pure rotation
(translated and rotated by an arbitrary quaternion, not merely axis-aligned) gives `‖F‖ = √3`
exactly, proving the deviatoric term really is rotation-invariant rather than only appearing to be
at some special angle.

**The acceptance test, and why it is the least certain thing in this entry.**
`tests/integration/test_finite_element_model.cpp` builds a 10×2×2-cell tetrahedral beam by hand (a
Kuhn six-tets-per-cube lattice, sign-corrected per element so the specific diagonal choice cannot
matter), pins the wall end, lets it sag under its own weight for 3000 ticks at 30 sub-steps with
heavy damping, and checks the tip deflection against the closed-form Euler-Bernoulli answer for a
cantilever under distributed self-weight load, `δ = mgL³/(8EI)`, within 35%. Unlike the pure-math
tests above, this asks something no test in this codebase has ever asked before: does a from-scratch
constitutive model, run to equilibrium through XPBD at a finite substep count, reproduce a continuum
elasticity result to within a stated tolerance. Every piece of it was reasoned through carefully —
the material constant was chosen backward from a target deflection so the scene sits in the
small-deflection regime the theory assumes, the lumped mass matches the cooker's own scheme, the
rest-inverse matrix is built with the identical formula
`engine/domain/physics/source/cooking/tetrahedral_mesh.cpp`'s `invert_3x3` uses — but XPBD
convergence at a finite substep count, and 240 tetrahedra's worth of discretization error against a
continuum formula, are exactly the two things that cannot be verified by inspection. This is,
honestly, the single most likely test in this session to need a numeric adjustment (the tolerance,
the substep count, or the material constant) once it is actually run.

**What P6-A did not do, deliberately, beyond what §16's P6 row already lists as separate
items.** No stress/strain readout (§9.3, P6-B — the von Mises stress field `FemTetrahedronT`
already has a slot for), no plasticity (§9.4, P6-C — the plastic rest-state columns already
exist and are already what every projection reads, waiting for something to write them), no
fracture, no collision of any kind (a `FiniteElementModel` today has no way to touch anything
else in the scene), no `ISoftBodyModel` interface (deliberately deferred to P6-F, which is the
first point a second implementation — `ShapeMatchingModel` — actually needs one to swap
against; building the interface now, with exactly one implementation behind it, would be the
premature abstraction §4's own principles rule out). The concrete class is named
`FiniteElementModel` rather than something generic specifically so that when P6-F does
introduce `ISoftBodyModel`, this class already has the name the doc's own §3.3 seam table
gives it.

**Update, same day: built once, reported passing — then found failing on a later full-suite run, and
P6-A is not closed.** The project owner's first build and test run of this section reported the
suite passing. A later full-suite run the same day (after P6-B/C/D below had also landed) found
`Integration_FiniteElementModel.CantileverTipDeflectionMatchesEulerBernoulli` **failing**, measuring
a tip deflection of roughly `0.183 m` against a theoretical `0.020 m` — about 9x too soft, nowhere
near the 35% tolerance. Whether this test was already failing at the first, "passing" run and simply
was not individually called out, or whether something in P6-B/C/D changed its behavior, is **not
established** — the two are read-only additions to `FiniteElementModel::step` gated by material
defaults that should not affect this scene (P6-B's stress readout writes a diagnostic field nothing
else reads; P6-C's plasticity is a no-op unless `plastic_creep`/`maximum_plastic_strain` are set,
and this test's material never sets them, verified by inspection), and the core substep loop itself
is byte-for-byte what it was before either landed — but "should not affect it" is exactly the kind
of claim this whole document's running caveat exists to distrust until a build confirms it, and no
build confirmed this one before it started failing. A quick experiment (raising `SUBSTEPS` from 30
to 200) reduced the measured deflection by only about 9% (`0.183 m` to `0.167 m`), which rules out
plain XPBD under-convergence as the dominant cause — Macklin's substep theory predicts the answer
converges *toward* the compliance-implied stiffness as the step shrinks, and a near-7x increase in
step count moving the result by under a tenth is not that curve. Recorded as an **open, unfixed
numerical discrepancy** — see §16.19 for the leading suspects — rather than closed. P6-A's model and
material code are still believed correct (every one of `tests/unit/test_fem_projection.cpp`'s
closed-form and finite-difference cases passed on the same build), but **its own phase acceptance
test does not currently pass**, so the phase itself is not closed on the strength of it.

### 16.18 P6-B, P6-C, P6-D: stress, plasticity, fracture — built, own tests pass

**Written 2026-08-01, same session as §16.17. Update, same day, after the full-suite run referenced
in §16.19: all three built, and their own tests are green** — `tests/unit/test_fem_stress.cpp`
(`Unit_FemStress.*`), `tests/unit/test_fem_plasticity.cpp` (`Unit_FemPlasticity.*`),
`tests/unit/test_fem_fracture.cpp` (`Unit_FemFracture.*`), and
`tests/integration/test_fem_plasticity_integration.cpp` (`Integration_FemPlasticity.*`) all passed.
This does **not** extend to P6-A's own acceptance test — the cantilever-vs-Euler-Bernoulli case is
failing, but that failure is isolated to P6-A's element/compliance math (see §16.19); it does not
implicate the stress readout, plasticity, or fracture logic described below, none of which that test
exercises. Since these three sit directly on top of P6-A and each other, an error in P6-A would
still be an error in everything after it — but nothing found so far points there rather than at
bending-specific discretization, per §16.19.

**P6-B, §9.3's readout** (`engine/domain/physics/include/SushiEngine/physics/soft/fem_stress.hpp`):
Green-Lagrange strain `E = (F^T F - I)/2` from the deformation gradient
`engine/domain/physics/include/SushiEngine/physics/soft/fem_projection.hpp` already builds, a St.
Venant-Kirchhoff second Piola-Kirchhoff stress `S = lambda*tr(E)*I + 2*mu*E` from the same Lame
parameters the constraints already read (not the exact stress conjugate to the two XPBD constraints
themselves, which is a positional formulation without one — the file's own header comment explains
the choice), pulled forward to Cauchy stress `sigma = F*S*F^T / det(F)`, reduced to one von Mises
scalar per element. Computed once per tick in `FiniteElementModel::step`, after the substep loop,
reusing the tick's final pose. `FemTetrahedronT::von_mises_stress` holds it;
`FiniteElementModel::maximum_stress()` is the model-level aggregate query — not yet
`ISoftBodyService::maximum_stress(entity)`, deliberately: no soft body is wired into `sim/` as an
entity yet, so there is nothing to hang a service method or an editor heat map panel off of until
P6-G does that wiring. Verified by hand against the one case with a textbook closed form: small
uniaxial strain (`F = diag(1+e, 1, 1)`), whose longitudinal stress is the P-wave modulus result
`sigma_xx = (lambda + 2*mu)*e` and whose von Mises reduction of that diagonal state comes out to
exactly `2*mu*e` — both re-derived by hand and matched to what the code computes before writing the
test (`tests/unit/test_fem_stress.cpp`), not fitted to whatever the code happened to produce.

**P6-C, §9.4's permanent dent**
(`engine/domain/physics/include/SushiEngine/physics/soft/fem_plasticity.hpp`): the multiplicative
decomposition `engine/domain/physics/include/SushiEngine/physics/soft/fem_element.hpp` was already
shaped for since P6-A — `plastic_inverse_column_*` is the rest matrix every projection actually
reads, separate from the original cooked `rest_inverse_column_*` — turns yielding into exactly one
operation: blend the plastic rest inverse toward the *current* shape's own inverse by a fraction
`plastic_creep`, derived in the file's own header comment from `Dp'^-1 = (1-c)*Dp^-1 + c*Ds^-1`,
which at `c=1` sets the current shape as the new rest state exactly, dropping the elastic strain the
constraints see to zero in one step. `Ds^-1` needs a genuine runtime 3x3 inverse — nothing bakes it
in advance the way P6-A's cooked rest inverse is — so
`engine/domain/physics/include/SushiEngine/physics/soft/fem_projection.hpp` gained
`invert_fem_matrix3`, the same cofactor identity the cooker's own `invert_3x3` uses, reassembled
into the column-major shape this file's matrices are already in. `accumulated_plastic_strain` (a new
field on `FemTetrahedronT`) is clamped to `maximum_plastic_strain`, and the rest volume is
recomputed from the new plastic rest shape rather than left at the original. Runs once per tick,
gated on the von Mises stress P6-B just measured, rather than once per sub-step the way §9.4's
pseudocode literally reads — a deliberate scoping choice the file's header comment states plainly,
not an oversight. Tested at the unit level against the algebraic guarantee the derivation makes
(full creep zeroes the elastic strain exactly, traced by hand through the geometric-series argument
for why the ceiling-clamping test converges to its limit rather than overshooting it) and at the
integration level by actually running `FiniteElementModel::step()` — pulling a low-yield element
past yield, releasing the load, and checking the settled shape did not spring back.

**P6-D, §9.5's removal**
(`engine/domain/physics/include/SushiEngine/physics/soft/fem_fracture.hpp`): the three guard rails
§9.5 names — a per-tick budget, a minimum fragment size, and a scene-level cap — all deterministic,
all state-derived, checked via a union-find over the *surviving* elements' shared vertices so a
removal that would leave a sliver below the minimum is refused before it happens rather than after.
**What this does not yet do, named rather than glossed over:** §9.5's other clause — "its shared
vertices are duplicated along the crack surface" — needs face-adjacency data (which element sits
across which of a tetrahedron's four faces) that the flat vertex list does not carry, so a fracture
that actually severs a body into two independently falling pieces will currently still show them
held together at whatever vertices they happen to still share after removal.
`FemFractureReport::fragment_count` reports the union-find's component count precisely so this gap
is visible to a caller rather than silently wrong — a real, scoped-out piece of work for a later
P6-D pass, not a bug in what exists. The removal mechanics themselves were re-derived carefully
after an initial draft had a genuine index-invalidation bug (removing candidates one at a time by
mutating the element array mid-loop, which shifts every later index down by one and makes an
already-captured later candidate index point at the wrong element) — fixed by deciding every removal
against a boolean mask over *original* indices and only rebuilding the element array once, after
every decision is made. Tested against hand-constructed element sets (a chain of three tetrahedra,
severing the middle one) whose resulting connectivity is knowable by inspection, not against real
cooked geometry.

**Read before extending any of this further:** P6-B/C/D's own tests are green (see above), but that
is not the same as P6-E-ready — P6-E's soft-vs-rigid collision reads the same
`SdfCollider`/`collision_asset_field` P5 built, and would compound an error in the stress or
plasticity pipeline into a collision response reading a wrong force. When this was written P6-A's
acceptance test was failing, so the element math these three build on had not been shown correct for
the case that matters most for collision response — bending under an external load — only for
uniaxial strain and pure creep. **That caveat is discharged:** §16.19's RESOLVED addendum records
the three fixes and the cantilever now matches Euler-Bernoulli inside tolerance. The standing
instruction survives it, because the coupling it warns about does. Re-run
`se test --suite functional --filter
'Unit_FemStress.*:Unit_FemPlasticity.*:Unit_FemFracture.*:Integration_FemPlasticity.*'` after any
change touching P6-A, not just once at the start of a session.

### 16.19 Full-suite run, 2026-08-01: two known-open issues, committed rather than chased further

**The user built the full test suite and ran it after §16.16–16.18 were written.** Result: 987 of
992 functional tests passed. Three of the five failures are unrelated to this session's work —
`Integration_RuntimeGraphBuilder.TheRebalancerIsOffForAPhysicsScene` and two
`Integration_JointAssembly` force-mismatch tests — caused by a same-day, same-machine SushiRuntime
merge that flipped the rebalancer's default (see the project memory note
`sushiruntime-realtime-gaps`); nothing in P5 or P6 touches the rebalancer or joint assembly. The
other two are real, and are recorded here rather than fixed on the spot, per an explicit decision to
commit what's done and continue rather than keep debugging blind:

**Open issue 1 — P5, `Regression_PenetrationContract.FastSphereDoesNotTunnelThroughAThinPlate` still
fails at 200 m/s.** §16.16 already covers the two fixes attempted this session: telling the
broadphase to sweep a fast body's full tick of travel (confirmed fixed — 50 and 100 m/s both pass),
and removing tier-2 CCD's gate on "tier 1 found nothing" so it always runs and can override a
wrong-side tier-1 manifold. The second fix does **not** hold at 200 m/s: the sphere still lands at
`y ≈ -0.105` — the mirror image of the correct `+0.105` — meaning conservative advancement itself,
or the manifold it hands off to tier-1's resolution path, is still picking the far side of the plate
at this speed. This was not re-diagnosed further this session (no new tracing added) because doing
so needs another build-and-trace cycle, which is the user's to run, not something to speculate
through by inspection alone. Next step: re-add temporary tracing to `conservative_advance()` and
`generate_obb_sphere_manifold`'s deep-penetration fallback, specifically logging which side of the
plate's normal the resolved contact normal points to at each of the three tested speeds, to see
where 200 m/s diverges from the two passing speeds.

**Open issue 2 — P6-A, `Integration_FiniteElementModel.CantileverTipDeflectionMatchesEulerBernoulli`
measures ~9x too much sag.** Measured tip deflection is `≈0.183 m` against a theoretical `≈0.020 m`
(35% tolerance, so this is not a near-miss). One experiment was run this session: raising `SUBSTEPS`
from 30 to 200 (a ~7x increase) only reduced the measured deflection to `≈0.167 m`, an 9% change.
That rules out plain XPBD under-convergence as the dominant cause — Macklin's substep theory
predicts the answer converges *toward* the compliance-implied stiffness as `h` shrinks, and a
near-7x step-count increase moving the result by under a tenth is not consistent with still being
far from that limit; something else is wrong, not just under-solved. `SUBSTEPS` was reverted to 30
rather than left at the experimental 200, since 200 does not fix the test and is not otherwise
justified as a runtime cost. Two suspects are named but neither isolated yet: (a) the test's
cross-section is only 2x2 cells of linear tetrahedra, which is a known-weak configuration for
bending specifically (linear tets are notoriously too soft in bending at low through-thickness
resolution — "shear locking" and its relatives), independent of anything about XPBD; (b) a possible
error in how Young's modulus/Poisson ratio map to this two-constraint model's deviatoric/hydrostatic
compliances specifically under a bending-dominated load, as opposed to the uniaxial-strain case
§16.18's P6-B stress readout was checked against, which would not show up in any test run so far
because none of them load an element in bending. Next step: build a single-element or single-column
bending test with a known closed form to isolate (a) from (b) before touching the cantilever test's
mesh density.

**Decision recorded:** rather than chase either of these further in this pass, both are marked
known/open here, the roadmap table below reflects "in progress" rather than "complete" for P5 and
P6-A, and the commit that follows this write-up includes that honest status — per explicit
instruction to commit now and continue working rather than keep debugging uninterrupted.

**RESOLVED (2026-08-01, later the same day) — all five failures fixed; the full suite is 992/992.**

- *Rebalancer test:* updated to enable the rebalancer explicitly before asserting the builder turns
  it off, since the runtime's default flipped to off (the property under test — construction
  disables it — is unchanged).
- *Joint force readout zero:* **not** the runtime merge after all. Tick-by-tick tracing showed the
  readout correct (343.35 N) until exactly `sleep_delay` and zero after: the door sleeps, but the
  *pinned* chassis (`inv_mass = 0`, dynamic-flagged) never does, so `JointProjectionT`'s "both ends
  non-simulated" guard never fired, the accumulators were reset every first substep and then
  measured a pair nothing integrates. Fix: either endpoint sleeping now freezes the joint's
  accounting (`engine/domain/physics/include/SushiEngine/physics/constraints/joint_projection.hpp`,
  both passes) — safe because an island sleeps as a unit and a static/pinned/kinematic partner
  cannot be moved by the projection anyway.
- *200 m/s tunnelling:* the CCD manifold was correct (traced: normal +Y at the impact tick, all
  three speeds). The culprit was §7.6's depenetration budget: at 200 m/s the crossing happens in the
  tick's last substep with a ~0.4 m violation, the 3 m/s budget corrects 6 mm of it, the velocity
  pass kills the speed, and the next tick's nearest-face manifold walks the now-buried sphere out
  the far side — the mirror-image rest pose. Fix
  (`engine/domain/physics/include/SushiEngine/physics/solver/contact_projection.hpp`): the budget
  now additionally covers however much the pair *closed during this substep* (measured from
  `prev_position` anchors), so a spawned overlap still pays out at 3 m/s while a moving body can
  always be stopped by what it hit. Cancelling the approach removes exactly the motion the substep
  added, so it cannot inject energy.
- *FEM cantilever 9x:* suspect (b), and now isolated. The deviatoric constraint was
  `||F||_F - sqrt(3)`; the Macklin-Müller formulation requires `||F||_F` itself — with compliance
  `1/mu` that yields force `mu * F` exactly, while the rest-zeroed version scales every deviatoric
  force by `(||F|| - sqrt(3))/||F||`, which vanishes at small strain *and* un-balances the
  hydrostatic `mu/lambda` offset whose whole purpose is to cancel the norm term's rest-state pull.
  Also applied Smith et al.'s `lambda + mu` reparameterization in the hydrostatic stiffness/offset
  (linearizing the constraint pair shows effective Lame `(mu, lambda_used - mu)`). Remaining error
  is ordinary single-iteration XPBD h^2 convergence (axial rest residual measured -2.2e-2 m at 30
  substeps, -5.9e-3 at 60, -1.5e-3 at 120); the cantilever test runs at 60 substeps and passes
  inside its 35% tolerance. The plasticity integration scene was recalibrated (50 → 800 m/s^2 pull)
  because its old load only crossed yield against the erroneously soft material.

### 16.20 Finishing P6's host side, 2026-08-01: what was built, and what a build has not yet seen

Everything §16's P6 row listed as "still not started" is written and has tests, with one group of
exceptions named at the bottom. This entry records the decisions worth keeping and, separately, the
honest verification status — which is not the same thing.

**§9.6, collision, in three parts.** Soft-vs-rigid was the one that could have gone badly and did
not: rather than a new contact solver for particles, each contacting surface vertex becomes an
ordinary one-point `ContactManifold` handed to the existing `solve_manifold_positions` /
`solve_manifold_velocities`. Friction, restitution, rest offset and the depenetration budget are
then literally the same code path a crate on a floor takes, and two-way coupling is free because the
rigid body is the manifold's second body. It also forced a real fix underneath: the cooked distance
field was only sampled at nearest voxel, which settles a deformable surface onto a staircase, and
its gradient — a half-voxel central difference *inside* one voxel — reads exactly zero and fell
through to `sdf_gradient_world`'s fixed-axis guard, returning a normal unrelated to the surface. A
trilinear sampler with an analytic gradient sits beside the old one; the convex-pair narrowphase is
untouched.

Soft-vs-soft is a build-once/refit-every-tick hierarchy per body, vertex-triangle both ways plus
edge-edge, and the continuous path is Bridson's coplanarity cubic solved by bracketing over the
derivative's roots rather than in closed form — grazing contacts are double roots and the closed
form is worst exactly there. Contacts are keyed by feature and reduced, which removes the "as many
times too stiff as the vertex has neighbours" error *and* makes the solve order a function of
topology rather than traversal (§12.1).

Two structural consequences, both of which the later work needed anyway. `FiniteElementModel::step`
became a composition of named phases, and `SoftBodyScene` interleaves several bodies' substeps —
without it two soft bodies in contact cannot be correct however good the contact code is, because
each would finish its whole tick against poses the other has not reached.

**§3.3's seam, and the shape it actually wanted.** `ISoftBodyModel` is the seam; `SoftBodyBase` is
the substep schedule three of the four model kinds share, as a template method with exactly one hole
(`project_constraints`). Splitting them matters: a consumer of the seam should not depend on a
substep loop it never calls, and the rigid tier of §9.7 genuinely does not want that loop — its tick
is "integrate one body", and forcing it through the particle schedule would mean integrating
hundreds of particles to arrive at what one `predict` already gives. §4.4's conformance suite runs
one set of cases against all three deformable kinds through `ISoftBodyModel&`.

**§9.7's transfer is written in displacements, and that is the whole of "no pop."** Reconstructing a
coarse vertex as `sum(weight * fine_vertex)` is exact only if the fine lattice sits exactly where
the embedding says, which it never does — a coarse lattice cannot represent every pose a finer one
can, so the reconstruction of an *undeformed* body already lands slightly off its own rest position.
Working in displacements makes the rest pose transfer exactly by construction and leaves only the
genuinely unrepresentable part of the deformation to be lost, which reads as softening rather than
as a jump. Coarsening has no stored inverse, so it is the transpose of refining — a mass-lumped
scatter — chosen because it reproduces a rigid translation exactly, and a body that is merely
falling must cross a tier boundary with no motion at all.

**§9.5's vertex duplication turned out not to need what it was blocked on.** This document recorded
the crack-splitting clause as requiring cooked face-adjacency data that `FemTetrahedronT`'s flat
vertex list does not carry. That was wrong. A vertex's *star* is a handful of elements, not a mesh,
and two of them lie on the same side of a crack exactly when they share three vertices one of which
is that vertex — which the vertex lists answer directly, at a cost proportional to the crack rather
than to the body. Splitting divides the vertex's mass rather than copying it (a copy would make a
body heavier every time it broke), keeps a pinned vertex pinned in every copy, and starts the copies
coincident so the split itself moves nothing. The boundary is rebuilt from the surviving elements
afterward, because a collision surface still describing the shape a body had before it broke is
worse than none.

**§9.1's bending is isometric, not dihedral, and the reason is the common case.** The textbook
constraint `C = acos(dot(n1, n2)) - angle_rest` carries a `1 / sqrt(1 - dot^2)` factor in its
gradient. For a flat stencil that dot product is exactly minus one — a division by zero sitting
precisely on the configuration every piece of cloth starts at and spends most of its life near. The
numerator vanishes too, so the correction has a finite limit, but computing it divides one
cancelling quantity by another exactly where the constraint most needs to be reliable. The
coplanarity-weight form has no trigonometry, no normals, no singularity, and constant gradients.
P6's own acceptance clause — zero stiffness reproduces the old behaviour — is met structurally
rather than numerically: at zero stiffness no bending constraints are created, so the sweep is the
same sequence of the same projections it was before.

**§6.5's `float` column.** `resolve_soft_body_precision` reads the asset and the component flags;
`SoftBodyInstance` owns one column or the other and answers in `Scalar` either way, so the decision
does not spread past instantiation. Participation in rollback *overrides* the cosmetic flag rather
than being weighed against it — two machines agreeing in `double` and disagreeing in `float` is the
entire failure §0.5 exists to prevent. Half-precision storage stays in P8, where the measurement
that justifies it lives.

**Verification status.** Built and run: **1061 of 1061 functional tests pass.** The first run after
this work failed nine, all of them mine, and they were worth having — two were real defects in the
engine and the rest were tests asserting things that are not true. Both defects were found by
measurement rather than by reading, which is the only reason they were found at all.

**Defect 1: the narrow phase was not widened by the tick's travel.** A cube dropped on a cube fell
straight through it, ending 4.07 m below where it should have rested. The broad phase was already
inflating its bounds by how far a particle can travel in a tick, so the candidate pairs were correct
— 576 of them, every tick, throughout the fall. The narrow phase then tested those pairs against the
pose at the *tick's start* and rejected every one for not touching yet. A body falling at 1 m/s
covers 16 mm per 60 Hz tick, which is more than a centimetre-thick surface, so the contact set was
empty at every moment it was built and full only in the ticks where the body happened to be
mid-surface. Contacts are found once per tick by design (§6.1); what was missing is that a set built
once per tick has to cover the whole tick. The fix is speculative contacts — accept a pair within
`thickness + travel`, keep `rest_distance` at `thickness` — and it applies to all three of §9.6's
mechanisms, so `ISoftBodyCollider::generate_contacts` now takes the tick duration. The projection is
an inequality, so the extra contacts cost list length and never a spurious push. The same bug was
behind the self-collision scene's falling half passing through its pinned half.

**Defect 2: `continuous` was strictly worse than `discrete`.** With the margin in place the discrete
path stops a sheet crossing at 300 m/s on its own. The continuous path still did not — because it
*replaced* the tick's contact set with the swept one each substep instead of adding to it. A body
marked continuous therefore tunnelled through something the same body would have hit with the flag
off. The swept pass now adds to the speculative set, so enabling the flag can only ever find more. A
flag that costs more and detects less is the one shape of defect nobody goes looking for, and it is
now pinned by a test named after that property rather than after the scene.

**Four of the nine failures were tests asserting false things**, and correcting them is worth
recording because each was a wrong belief rather than a loose tolerance:

- *The comparator removed translation but not rotation.* Nothing pins the orientation of a body
  floating in free space, and a Gauss-Seidel sweep is not symmetric — it imparts a small torque that
  velocity damping cannot undo, because damping removes the spin and not the angle already turned
  through. `MassSpringModel` had recovered its rest shape to 4.7e-17 up to a rigid motion and was
  being reported as having failed by 0.129 m. The conformance comparator now fits and removes the
  rotation.
- *§4.4's "all implementations converge to the same rest shape" is very nearly but not exactly
  true.* The spring and shape-matching models rest at the cooked lattice by construction; the stable
  neo-Hookean pair does not. Its two constraints balance at a deformation gradient slightly off the
  identity, so an unpinned FEM body relaxes to a shape a few per cent from the lattice it was cooked
  from — measured, 4.6 mm on a 100 mm cube, and non-uniformly, since it survives removing a uniform
  scale as well as a rigid motion. That is a documented property of the model, so the cross-model
  bound states it instead of hiding it, and a second assertion pins the two that *do* share a rest
  state to 1e-6 of each other so the explanation cannot silently stop being the right one.
- *A fall was being measured at the unweighted centroid.* A tetrahedral lattice's lumped masses are
  not uniform, so the plain centroid moves whenever an internal projection redistributes particles
  even though momentum is perfectly conserved. Measured at the centre of mass, the three models
  agree.
- *A single cube cell is self-intersecting at a large enough thickness.* Two edges of a 0.05 m cell
  that share no vertex pass within 0.05/√3 = 0.0289 m, because the closest approach of a face
  diagonal and an edge is not along an axis. The test had set a combined thickness of 0.03 and read
  the correct answer as a bug.

**The remaining precision finding.** `Cooking::SoftBodyBinding` stores its weights as `float`, so
after the round trip they sum to one only to about six digits. Both readers multiplied them by
*absolute* positions, which displaces a reconstructed point by `|position| × 1e-7` — micrometres
near the origin, centimetres at planet scale, and invisible to any test that places its body at the
origin. Two of the nine failures were exactly this, caught only because one case deliberately placed
the body 12 m away. `Cooking::read_binding_weights` now renormalizes at the one point every reader
goes through.

### 16.21 P6's acceptance number, measured

P6-J1 generalized `IncrementalColoring` and `ConstraintStore` from two body indices to N, and P6-J2
made the FEM element a constraint kind in the device graph alongside distance constraints and
joints. That is what the 20 000-tetrahedron acceptance line was waiting on, so it can now be
measured rather than deferred. `samples/physics/soft_body_budget.cpp` is that measurement: a 15³
Freudenthal lattice — 20 250 tetrahedra over 4 096 particles, top layer pinned so every element
deforms — stepped at the 32 substeps §13.1 names.

It is a probe and not a suite assertion on purpose. §13.1 states its targets against "one
desktop-class GPU through SushiRuntime", and a test asserting 3 ms would be asserting the machine it
happened to run on. So the *shape* is pinned by `ATetrahedralLatticeColoursCleanlyAndComposesOnce`
in the conformance suite — every element finds a band, the lattice colours inside the ceiling, the
graph composes once and never again — and the *number* is reported by the probe.

**The number, on the machine at hand.** 20 250 elements placed, none rejected; 32 colours of 48
used; zero recompositions after the first tick; **mean 29.4 ms/tick, best 25.4 ms/tick, against a 3
ms budget.** Roughly ten times over.

That is stated without spin, and so is its one large caveat: SushiRuntime found exactly one device
on this machine — `AMD Ryzen 5 7600X`, twelve workers — so this is the **CPU backend**, not the
desktop GPU the target is written against. The honest reading is not "P6 misses §13.1 by 10×" but
"§13.1's soft-body line has not yet been measured on the hardware it was written for; on a
twelve-core CPU backend the same scene costs 29.4 ms." Until this is run against a GPU device, the
acceptance line stays open.

Two things the measurement does settle, because neither depends on the backend. The scene builds and
runs without a single rejected element or recomposition, which is what J1 and J2 were for. And the
work per tick is now a known quantity: 20 250 elements × 32 substeps × two projections is 1.3
million projections a tick, dispatched as 32 colours × 32 substeps = 1 024 graph nodes, so 1 024
barriers a tick as well. Which of those two the 29.4 ms is mostly spent on — the arithmetic or the
barriers — this probe does not distinguish, and guessing would be worth less than profiling it.

**What P6 does not owe any more.** The stale list that stood here — `ClothStrandView` not yet
generalized, no `ISoftBodyService`, no editor tetrahedra view, the FEM element not a constraint kind
— was closed by P6-G2, P6-G3, P6-G5 and P6-J1/J2 respectively.

### 16.22 P7-A, P7-B, P7-A2: the beam, and the four decisions it forced

P7 opens with the one thing every later part of it stands on. A vehicle is a node cloud held by
beams, and until a beam exists as something the solver actually projects, the cooker has nothing to
emit and the hybrid has nothing to attach. This entry records what was built and the four choices
that were not obvious, each of which could have gone the other way.

**A beam has no anchors, and therefore applies no torque.** `XpbdDistanceConstraintT` carries a
local anchor per body so it can hold two *points on two rigid bodies*. §11.1 defines a node as a
particle — zero inverse inertia, no meaningful orientation — and an anchor on a body that cannot
rotate is a constant offset that could have been folded into the node's position. So the two anchors
are dropped, 48 bytes per beam are not spent to express nothing, and the consequence is written into
the header rather than left to be discovered: attaching a deformable shell to a rigid chassis core
is §10.3's attachment constraint, which does carry a lever, and never a beam.

**Plasticity runs once per tick, not once per substep.** `plastic_creep` is read as the fraction of
the current excess that becomes permanent *per tick*, which is exactly how `apply_fem_plasticity`
already reads it. Running it per substep would make a beam's permanent set a function of the substep
count — and the substep count is derived from scene motion (§6.2). A dent that deepened because
something unrelated in the scene sped up would be the least explicable behaviour in the system.

**The two plastic parameters live on the beam, not in a material it points at.** The element solver
can afford to take a `SoftBodyMaterialT` argument because it sits in `physics/soft`, which includes
`physics/constraints`. A constraint kind that named a soft-body material would invert that
dependency. Two scalars per beam is the price of the layering, and
`engine/domain/physics/include/SushiEngine/physics/soft/beam_properties.hpp` is where both are named
at once — the derivation from a material lives on the side of the seam that may name both.

**The load is `-lambda / h²`, and the sign is load-bearing.** §10.4's recovery, with the negation
that makes tension positive, because a member's load has a direction an engineer expects to read. It
is also what the two thresholds are measured against, and both are measured against the *peak*
substep load rather than the mean — for the reason `JointConstraintT::break_force` records at
length: an impact's mean over a tick that also contains the rebound is near zero, and what tears a
member out is the magnitude.

**What the derivation buys.** §11.2's first row against BeamNG is that a vehicle there is thousands
of hand-tuned numbers with no stated relation to any material. `beam_properties_from_material` is
the correction, and it is small: `compliance = L / (E·A)`, `deform_force = yield_stress · A`,
`break_force = fracture_stress · A`. The one thing that is not a material's to know is the
cross-section, which is a property of the *structure* — how much of the body this member stands in
for — so `beam_tributary_area` states the conservation rule (`Σ A·L` is the body's volume) here
rather than inside the cooker, because it is a claim about physics and not about voxel grids. The
honest limitation is recorded with it: an even split is right in aggregate for a roughly isotropic
lattice and no better than roughly right when beam lengths differ by a large factor.

One sentinel needed handling rather than passing through. A material that does not yield carries
`yield_stress = 1e30`, and converting that into a threshold would give `1e30 · A` — a *finite* force
a big enough impact reaches. It is passed through as zero, which is what the beam reads as never.

**The fifth kind, wired.** `IConstraintSolver` gained `add_beam`/`remove_beam`/`read_beam`/
`write_beam`/`beam_capacity`; both solvers project beams immediately after the distance band (the
two kinds that say "these two points are this far apart" belong together) and damp them immediately
after the velocity derivation. `write_beam` exists where `write_element` does not, and that
asymmetry is the point: a dent and a failure are decided at the tick boundary, by the scene, from
the load the solve recovered. The band is skipped structurally when the budget is zero — the default
— so a scene with no vehicle in it does not carry a zero-extent region per colour per substep for a
kind it never uses.

**Verification status, stated plainly.** Written with tests, compiling clean, with **no suite result
recorded here**: sixteen unit cases in `tests/unit/test_beam_constraint.cpp` and four conformance
scenes in `tests/integration/test_solver_conformance.cpp` (a beam chain across every colour, a beam
and a distance constraint sharing a node, the load readout agreeing between the two solvers, and a
removed body taking its beams with it). The unit suite pins the load against Hooke's law rather than
against itself — a compliant beam must report `E·A·Δ/L` — which is the only assertion in the set
that would catch a compliance derivation that is self-consistent and wrong.

### 16.23 P7-C: the `.sushinodebeam` asset, and why five things travel together

A vehicle is not one array. It is a node cloud, a beam network over that cloud, the collision
surface the cloud presents, a rigid core the shell hangs from, and the render mesh's binding onto
the nodes. The first decision was whether those are five assets or one, and one is not a
convenience: the four cross-referencing sections all index the node array, so splitting them gives
them four chances to be loaded at different versions. A shell whose attachment records name nodes
from an older cook is a vehicle that loses its doors on load, and it loses them silently, because
every index is still in range — just of the wrong cloud.

**The rigid core is a mass, not a body.** `NodeBeamCoreRecord` carries mass, centre of mass,
principal inertia and principal rotation, and no shape at all. The core's *collision* is a
`.sushicollision` asset the vehicle asset (P7-F) names alongside this one, because the same
node-beam structure is legitimately reused with different core colliders and because a cooked
collider is already a format with an owner. What is in this blob is only what the solver needs to
create the core body and attach the shell to it.

**A core of zero mass is a pure node-beam vehicle**, and that is §11.2's promise kept as a number
rather than as a branch. The architecture does not choose between the hybrid and the pure structure;
the asset does, and an artist can walk between them — a core carrying nine tenths of the mass and a
core carrying none are the same asset with the dial in different places. `node_beam_has_core` is the
only test anything performs.

**The beam records are not `BeamConstraintT`, and the reason is not tidiness.** A record carries the
cooked half — topology, rest length, the four derived numbers, the two plastic parameters — and none
of the runtime half: no accumulated strain, no force accumulators, no live rest length. Storing the
solver's struct would make the blob's bytes change the day that struct grows a field, which
invalidates every cached asset in the project for a change no artist made. The second reason is the
layering: `physics/cooking` includes `physics/constraints` nowhere, and making a cooked record *be*
a constraint would be the first time, to save one assignment loop in the instancing code P7-E owns.

**Three failures a wide binary format actually has, and what is done about each.**

1. *A count raised without the bytes behind it.* Every section's extent is checked against the
   blob's own `total_size` before a pointer is handed out, so a header claiming four thousand nodes
   over a four-node payload fails validation instead of reading the next section as node records.
2. *A cross-reference nobody re-checked on the way in.* Beams, surface indices, attachments and skin
   slots are validated at load as well as at write, because a blob can come from an older writer or
   a hand edit. Each unchecked reference is a read into a neighbouring section, which produces a
   *plausible vehicle* rather than a crash — the worse of the two failures by a wide margin.
3. *Padding.* Every record is packed with no interior holes — 56, 72, 48, 44 and 88 bytes, asserted
   rather than assumed. Padding does not break a round trip; the bytes come back as they went in. It
   makes two cooks of the same input differ in bytes nobody wrote, and §8.1's cache then serves
   entries that look changed and are not.

Two smaller decisions worth recording. A beam onto itself is **refused at the cook**, and that is
not a memory-safety check: a self-beam has no axis, projects nothing, and would sit in the structure
reporting zero load forever while the panel it was meant to hold flaps. And every skin slot must
name a real node **whatever its weight**, so a reader never has to test a weight before trusting an
index — a rule enforced only where the weight is non-zero is one that fails the first time someone
iterates all four influences.

**The skin weights are renormalized at load**, by `read_node_beam_skin_weights`, for exactly the
reason `read_binding_weights` exists (§8.5). The weights are stored as `float` so a record is
thirty-two bytes and an array of them is a `memcpy`; four floats that summed to one in the cooker
sum to one within six decimal digits after the round trip. The reconstruction is a weighted sum of
*absolute* positions, so that shortfall scales with distance from the world origin: ten micrometres
at a hundred metres, centimetres at planet scale. It appears as the render mesh sliding off the
structure the further the world is from its origin, which is to say nowhere near where it would be
tested.

**What this is not.** The instancing that turns these records into bodies and constraints is P7-E,
so the honest status is "produced and validated, not yet consumed" — §16.10's distinction, stated in
advance this time rather than found in a later audit. Twenty-three unit cases in
`tests/unit/test_node_beam_asset.cpp` cover the round trip, the packing, the eight refusals, the
four corruptions, the rest-pose reconstruction, the rigid-motion property, the degenerate frame and
the far-from-origin renormalization.

### 16.24 P7-D: the cooker, and the skinning that had to be thrown away

§11.3 in six stages — Repair, PlaceNodes, ConnectBeams, Skin, BuildCore, Serialize — and the point
of all six is §11.2's first row: a vehicle stops being thousands of hand-typed numbers and becomes a
mesh, a dial, and a material.

**Two things were reused rather than written, and both were the whole decision.**

*The lattice is the tetrahedralizer's.* §8.3's stage 2 already voxelizes, flood-fills the interior,
conforms the boundary, and hands back per-vertex masses with an outward-wound surface. A node cloud
is exactly those four things, so the cooker asks for them. The alternative — sampling the distance
field on a grid and keeping what reads negative — is four lines and wrong in the one case that
matters: `MeshDistanceQuery` documents that its sign comes from the nearest triangle's plane and
says outright that deciding what is interior to a *dirty* mesh is what the flood fill is for. Two
implementations of "inside" would agree on every test mesh and disagree on every shipped one.

*The bracing was already there.* A lattice tetrahedralization's edge set contains both kinds §11.1
names — the ones along a grid axis, and the diagonals. So "add bracing beams by a diagonal rule" is
a **classification by length**, not a second construction pass; generating more diagonals would
double-brace a network that is already braced, and the symptom of that is a structure that will not
deform, which gets diagnosed as a compliance bug. On a 2×1×4 box at fidelity 0.35: 96 nodes, 429
beams, 209 of them bracing.

**The cache key had to grow, and the cooker had to be the one to grow it.** A material is not in
`CookingParameters` — no other cooker has a use for one, and putting it there would make every
collision asset in the project carry a field it ignores. But §8.1's key is built from that record,
so nothing *else* can fold the material in. `NodeBeamCooker::cache_key` therefore hashes its own
settings into the parameters half. Without it, the same mesh cooked as steel and as aluminium
resolves to one key and the second cook is served the first one's asset: a cache that returns the
wrong answer rather than a slow one, which is the only kind of cache bug worth designing against.

**The skinning was written, measured, and thrown away.** The first formulation was the obvious one —
a render vertex is the weighted sum of the nodes nearest it — and it is wrong in a way that only a
measurement shows. The centroid of the four nodes nearest a box corner sits *inside* that corner; at
the lattice spacing this cook produces, the rest pose reconstructed **0.4 m** off. That is not a
tolerance to tighten, it is a mesh that is visibly shrunk before anything has moved, and it violates
§0.4's contract that the render mesh is embedded in the simulated one.

The replacement stores a vertex as a displacement from that centroid, expressed in a frame the nodes
themselves define: Gram-Schmidt on the two edges from the first influence. It reproduces the rest
pose to **1.3e-8 m** — the bound is the `float` the offset is stored in, not the formulation —
rotates with the structure, and stretches with it. Three properties had to be checked rather than
assumed, and each is a test: the rest pose is exact, a rigid motion of the nodes carries the vertex
with it (a stored world-space displacement would pass every straight-line test and fail the first
corner), and three collinear nodes fall back to the asset's own axes **identically on both sides** —
which is why the frame is one function the cook and the runtime both call rather than the same six
lines written twice.

**What the cooker does not decide.** It produces one part. Splitting a vehicle into doors and panels
cannot be inferred from a mesh, and a split invented from connectivity would produce parts no artist
asked for and cannot rename; that is P7-J. The core/shell split it *can* make is the lattice's own:
interior is chassis, boundary is shell, and only the interior is attached. A lattice one cell thick
has no interior and therefore no attachments, which is a pure node-beam shell and is reported by the
attachment count rather than by a failure.

Fifteen unit cases in `tests/unit/test_node_beam_cooker.cpp`. The ones that would catch a
*plausible* cook: the compliance against `L/(E·A)` at the area the beam was actually given, `Σ A·L`
against the structure volume, the mass fraction the dial asked for, the rest-pose reconstruction end
to end, and the two cache-key inequalities.

### 16.25 P7-E: the hybrid alive, and the three things instancing had to decide

The asset describes a vehicle. This is the step that makes it exist: every node record becomes a
particle in the solver, every beam record becomes the fifth constraint kind, and — when the asset
carries one — the rigid core becomes a body the shell hangs from. §11 opens by saying nothing in it
is new physics, it is new *assembly*, and this entry is the assembly.

**The shell-to-core attachment is a ball joint, and that is a decision rather than a shortcut.**
§10.3 describes an attachment that averages its correction across a small vertex neighbourhood so a
mount does not tear a single vertex out of a mesh. That averaging answers a *soft-body* question:
which patch of a continuum the mount acts on, when the continuum has no natural unit at that scale.
A node-beam shell has no such ambiguity. The cooker already chose which node the mount acts on, and
a node is a whole body with a mass and an inverse mass. So the attachment is `JointKind::Ball` with
the lever on the core and none on the node — which is exactly the constraint that was wanted, and
which arrives with §10.4's force recovery, `joint_should_break`, and a compliance already built. A
new constraint kind would have re-derived all three, and it would have needed its own colouring
band, its own conformance scenes and its own device node to do it.

The one thing that needed care is which frame the anchors are in. `RigidBodyT` stores its inertia as
a body-frame *diagonal*, which is a statement that the body frame is the principal frame — so the
core is instanced rotated into it, its centre of mass expressed there, and the attachment anchors
rotated with it. The check that this is right is not an inspection, it is a round trip:
`body_origin` of a core spawned at an arbitrary position and orientation must come back on the
asset's authored origin, and it does, to 1e-12 m.

**The tick boundary belongs to the owner, not the solver.** A solver projects; it does not decide
policy. `end_tick` reads each beam back, applies §11.1's plasticity to it, removes the ones that
passed their break threshold, and does the same for the mounts. That is the asymmetry
`IConstraintSolver::write_beam` exists to serve and `write_element` deliberately does not have, and
this is its first real caller — before P7-E, `apply_beam_plasticity` was called only by its own unit
test. It runs in index order throughout, because the pass *removes* constraints and a removal order
that varied would leave a device solver with a different slot layout on a replay, which is §0.5's
whole failure mode rather than a cosmetic difference.

`apply_beam_plasticity` grew a position-pair form for this, with the existing node-array form
deferring to it. A structure holds solver slots and reads them back a pair at a time; it has no
array indexed by `beam.a` to hand over. Writing the rule a second time against two positions would
have been four lines and a divergence waiting to happen.

**A part comes off by losing its last tie, and then nothing is done to it.** This is the part of the
acceptance criterion — *loses parts* — that looked like it needed machinery and does not. When every
mount holding a part and every beam joining it to another part have broken, the part is already
free: its nodes are still bodies, still beamed to each other, still colliding, and they now fly away
as the loose node cloud a torn-off door is. There is nothing to remove, nothing to respawn, and no
second representation to keep in step. What the structure adds is the *report* — a caller that wants
to play a sound or spawn debris has to be told, and reconstructing "is this part still held" from
outside would mean walking the whole beam list every tick. It is a counter per part, decremented as
ties go.

A part that was never tied to anything is never reported detached, which is deliberate and not an
oversight: a single-part vehicle with no core is held together by nothing *by design*, and answering
"detached" for the whole thing on its first tick would make the readout useless for the case it
exists to report.

**Refusal rolls the whole vehicle back.** A budget that runs out part way through removes everything
already added. A vehicle missing the beams that did not fit is a structure that folds the first time
it is touched, and it folds in a way that reads as a physics bug rather than as a capacity one. The
test measures that by what fits *afterwards* rather than by a flag — a solver sized for four bodies
that refused a five-body vehicle must still have four free, and the only way to see that is to put
four in.

**What the measurements say.** Fifteen unit cases in `tests/unit/test_node_beam_structure.cpp`, on
the host solver. A mount holds its node to 1.4e-8 m in the core's frame across 19.5 m of travel. A
beam loaded past its yield threshold does not dent on the first tick — it dents on the third,
because a beam starts at its rest length and the load that yields it is the one that builds as its
node pulls away — and it then work-hardens at exactly the authored maximum strain. A door thrown at
50 m/s tears off its one beam and its one mount and is reported detached exactly once, however long
it keeps falling, while the chassis mounts authored unbreakable do not move. Two runs of that crash
agree bit for bit on every node position.

One thing the tests had to learn the hard way and is worth writing down: **gravity cannot load a
vehicle.** It is uniform, so a vehicle with no ground under it falls as one piece and every beam and
mount in it carries exactly nothing. The first draft of these tests asserted against free fall and
passed trivially while measuring nothing. An impact is a relative velocity, and the smallest way to
make one is to throw a single body.

**What P7-E does not do.** Suspension, wheels, and the `VehicleAsset` that names both a
`.sushinodebeam` and the core's `.sushicollision` are P7-F. The node cloud's collision surface is
carried in the asset and instanced with it, but P7-E generates no contacts from it — that is the
`physics/scene` wiring, and it belongs with the vehicle the scene can actually drive. Node drag
areas travel with the nodes and are read by nothing until §11.6's wind coupling in P7-I.

### 16.26 P7-F: the corner, the drive the library could not express, and a leak in the integrator

§11.2's third row is one sentence: *"Suspension is joints and drives (§10.1), not beams — a slider
joint with a spring-damper drive is more controllable and more stable than a beam network, and it is
what every shipping racing game does."* Building it turned up two things that were not in the plan.

**A corner is two joints, and the second one is why.** The strut is a `Slider` between the chassis
core and a carrier, and the axle is a `Hinge` between that carrier and the wheel. The carrier is not
padding: the two statements a corner makes are about *different pairs*. The spring acts between the
chassis and something that does not spin; the axle is between that something and something that
does. One body cannot be both, and a slider that also let its body spin would be a
six-degree-of-freedom joint whose single drive would then have to be the spring *and* the brake.

**Steering costs no third joint.** The slider locks all three rotations, which means the carrier's
orientation *is* the slider's frame on the chassis side. Rotating that frame about its own primary
axis therefore steers the corner — and because the primary axis is the strut axis, the rotation
moves nothing else: the travel direction is unchanged, the spring is unchanged, and the axle turns
because it is fixed in the carrier. That is a MacPherson strut, where the kingpin and the damper
axis coincide, and it is worth saying that the geometry was *chosen* for this rather than assumed. A
double wishbone would need the steering axis authored separately from the travel axis and a third
joint to hold it. Measured: a 0.4 rad steer turns the two front axles by 0.3998 rad and leaves the
rear two at 0.0002.

**The library could not express a spring-damper drive, so the drive grew a damper.** A position
drive at a compliance is a spring. A spring alone rings forever. §10.1 offers the velocity drive
with a force limit as damping — and that is *Coulomb* friction, a constant force, not the viscous
resistance a damper is; and in any case a joint has one motor and one mode, so a strut could have
the spring or the damper and not both. Two joints between the same pair was the alternative and it
is worse: a second slider would duplicate the rotation locks and the perpendicular locks, which is
not a cost but a *bug* — the same degree of freedom constrained twice.

So `JointMotorT` gained `damping`, read independently of the mode. It is deliberately the same
statement `BeamConstraintT::damping` makes and deliberately the same arithmetic: a fraction
`min(1, damping·h)` of the coordinate's relative rate removed per substep, which makes it a **rate**
rather than a per-substep fraction. That distinction is the one a test has to pin, because the
substep count is derived from scene motion (§6.2) and a suspension whose firmness depended on what
else was moving nearby would be the second-least explicable behaviour in the system. Measured: the
same damping over the same wall time at 4 and at 16 substeps leaves within 0.71 % of the same
motion. A disabled mode with a damping set is a pure damper, which is a real mechanism — a steering
damper, a door closer — and not a misconfiguration.

**And a defect in the core integrator, found by a wheel.** The first version of the
free-spinning-axle test asserted that an unbraked wheel keeps its speed, and it failed: 50 rad/s
became 33 rad/s in a second. It was not the hinge. A **free rigid body with no constraint on it at
all** lost spin at exactly the same rate.

`predict` advanced orientation with `apply_angular_correction`'s first-order form — normalize
`q + ½(ωh)q` — whose applied rotation is `2·atan(θ/2)` and not `θ`. `update_velocity` then recovered
`2·vec(δq)/h`, which reads `2·sin(θ/2)` and not `θ`. Together they multiply angular velocity by
`1/sqrt(1 + (θ/2)²)` every substep. At 50 rad/s and 480 Hz that is 4 % a tick. A car at 100 km/h
turns its wheels at 82 rad/s, so every number in §11.4's powertrain would have been tuned against a
leak, and the leak would have looked like drivetrain drag.

The fix is the exact pair: the exponential map in `predict` (`integrate_orientation`), its logarithm
in `update_velocity`. Spin is now conserved to 2e-12 over two seconds. **Constraint corrections keep
the first-order form**, and that is not an oversight — a correction is a fraction of a degree, where
the first-order map is the standard, cheaper, and entirely adequate choice, and changing it would
alter every solver result for no accuracy anyone can measure. The two uses differ by two orders of
magnitude in angle and it is right that they differ in method. This one is recorded at length
because it is the kind of defect that never announces itself: nothing crashes, nothing is unstable,
and every rotating body in the engine was quietly slowing down in proportion to how fast it was
going.

**What the measurements say.** Ten unit cases in `tests/unit/test_vehicle_suspension.cpp` and four
added to `tests/unit/test_joint_projection.cpp`. Four planted corners settle at 0.0617 m against the
0.0617 m that `m·g/k` predicts, and report a corner load of 2159 N against 2158 N of corner weight —
the assertion that earns its keep, because a strut with its spring rate read as a compliance or its
travel signed the wrong way still holds a car up and still moves when it is pushed. The damped
strut's swing is 0.00006 m where the undamped one's is 0.082 m. A soft spring rides down onto its
bump stop at 0.1200 m of its 0.12 m travel and stops there. A body budget one short of a four-corner
car refuses the vehicle and gives every slot back.

**What P7-F does not do.** Nothing here generates a contact, so the tests bolt the wheels to the
world to stand in for the ground — the tyre is §11.5's and P7-H's. Drive torque is §11.4's and
P7-G's; the hinge each corner exposes is where it lands, and `set_brake_torque` already writes that
motor, so the seam is present rather than promised. Ackermann geometry is a steering *rack* and
belongs with §11.4's rack constraint, which is why `set_steer_angle` takes one angle and not one per
corner.

### 16.27 P7-G: the drivetrain, and why it is not made of constraints

§10.5 decides the shape of this before §11.4 describes it. A powertrain has mass ratios in the
thousands — a crankshaft against a car — and couplings that are exactly rigid, and pushing that
through the three-dimensional solver buys a stiffness the substep count then has to pay for. Its
first escape hatch: *"a powertrain is a chain of rotational inertias, not a spatial mechanism.
Simulating it as an independent one-dimensional multibody system and coupling it to the wheels
through a torque constraint is both cheaper and more accurate."* So `PowertrainT` knows nothing
about bodies, handles or solvers. It is handed each driven wheel's spin rate and its inertia about
its own axle and hands back a torque per wheel; which body, about which axis, and where the reaction
lands are `VehicleInstanceT::begin_tick`'s, and that is the whole three-dimensional half.

**The state is one number.** Gearbox and final drive are exact ratios and the differential's outputs
*are* the wheels, so every speed downstream of the clutch is determined by speeds the caller already
measured. The crankshaft is the only free rotational coordinate in the chain. A member per stage
would have been a cache of derived values that went stale the first time anything else moved a wheel
— a `GearJoint`-shaped mistake made in scalars instead of constraints.

**The clutch is solved, not damped.** The torque for which the crank and the clutch's output arrive
at the same speed at the end of the step is available in closed form from the two inertias and the
engine's torque; compute it, then clamp it to the plate's capacity. Below capacity the clutch locks
exactly; above it, it slips at exactly its rating. There is no stiffness to tune and nothing to go
unstable — the same trade XPBD makes everywhere else in this engine. Modelled as a stiff spring
instead it would ring, and the ringing would be blamed on the tyres. Measured: after twenty ticks
the crank and the geared wheel speed agree to 1e-6 rad/s.

**The differential is one number, not three kinds.** Open splits torque evenly and lets its outputs
turn at any speeds; locked forces them together; limited-slip is the first with a bounded amount of
the second. Three kinds would have been a branch, a constraint the solver does not have — §10.2's
table defers `GearJoint` to exactly here — and two of the three untested most of the time.
`DifferentialSettingsT::lock_torque` is one clamp: zero is open, large is a spool, between is a
limited-slip, and the number is the one a differential is actually specified by. The lock torques
are balanced to sum to zero before they leave, because a differential *divides* torque and can never
be a source of it; with unequal wheel inertias or one wheel's clamp biting, the raw values do not
cancel on their own. Balancing can push a wheel past the authored bound by at most that bound again,
which is the right way to be wrong — a differential that invented torque would accelerate a car with
its wheels in the air.

**The gearbox's own shafts take their share, and leaving that out is what broke the clutch.** The
shafts are geared to the wheels, so they accelerate with them: referred to the wheels they weigh
`inertia × ratio²`, which in first gear is comparable to the wheels themselves. The first draft
delivered the full shaft torque to the wheels *and* charged the clutch solve for the shafts'
inertia. The two halves of the chain then disagreed about how fast the driveline was turning, and
the symptom was that the clutch never quite locked — a 3 rad/s residual that no amount of staring at
the clutch solve would have explained, because the clutch solve was right.

**Both wheels of an axle point their axles the same way now.** `SuspensionSetupT::axle` was
documented as pointing out of the wheel's outboard face. A wheel is symmetric about its axle, so
nothing physical asked for that — and it meant the two wheels of one axle spun in opposite senses
when the car rolled forward. §11.4 reads one signed speed per wheel and writes one signed torque
back, so a car built that way handed its differential a mean of zero and its chassis two reactions
that cancelled exactly. It was caught by the reaction test reading a clean zero, which is the useful
kind of failure: not noise, but the one number that says "these two terms are equal and opposite
when they should have added". The axle is now a *convention the vehicle shares*, not a description
of which way the hub cap faces, and it costs nothing because no other reader of it cares about the
sign.

**The reaction lands on the chassis, not the carrier.** A driven wheel is turned by a shaft from the
differential and the differential's casing is bolted to the chassis, so what a driver feels as squat
under power is a sprung reaction. Summing the wheel impulses and negating them on the core also
means a vehicle's own engine cannot change its total angular momentum — the statement that stops a
car from driving itself around in mid-air. Measured: the core takes -13.402212 N·m·s against the
wheels' +13.402212, about the axle and about nothing else.

**The engine.** A torque curve and not a peak-torque number, because the shape *is* the engine's
character — where it peaks decides which gear a corner is taken in — and one number makes every
engine feel like the same electric motor. Held flat outside its ends rather than extrapolated: a
linear extrapolation past the last sample crosses zero and goes negative, and an engine that
produces reverse torque above its highest authored speed is a bug that only shows up on a long
straight. Idle is a proportional governor over an authored band, so its droop is a number the author
chose rather than a surprise — a proportional controller settles wherever its output balances the
load, and leaving the band implicit is how an engine ends up idling ten per cent low with nothing to
point at. The limiter is a throttle cut, and its overshoot — up to one tick of peak torque on a
light crank, 22.7 rad/s here — is what a real limiter does, so the test asserts that bound rather
than a round number. The engine does not stall: stalling needs an ignition state and a starter, and
those are driver-input surface rather than §11.4.

**Reverse is a negative ratio and neutral is a ratio of exactly zero**, both in the one ordered list
a driver moves through. Selecting a gear is then an index and never a mode plus an index.

**What the measurements say.** Seventeen unit cases in `tests/unit/test_vehicle_powertrain.cpp`. The
clutch locks to 1e-6 rad/s and reports itself unslipped; an overwhelmed clutch carries exactly its
40 N·m and a released one carries exactly zero; open and locked differentials both sum to the shaft
torque to 1e-9 N·m, and the locked one gives the slower wheel 1085 N·m against the faster one's 599;
lifting off in gear turns the chain into a brake; forty ticks in reverse leave the wheel turning
backwards and the crank turning forwards; the chassis takes the exact negative of the drive impulse;
and two identical runs agree bit for bit.

**What P7-G does not do.** There is still no ground: the driven wheels spin up against nothing,
because the tyre is §11.5's and P7-H's, and until it exists a throttle produces wheel speed rather
than vehicle speed. One differential is shared over every driven corner, which is exactly a front or
rear differential on two wheels and a simplification on four — a cascade with a centre differential
is not modelled, and §11.4 names one. There is no torque converter and no automatic gear selection;
both are policy over this chain rather than physics in it. And the coupling is explicit at the tick
boundary: the tyre load reaches the chain as the wheel speeds it measures, one tick late, which is
the same lag every explicit coupling in this engine already accepts.

### 16.28 P7-H: the patch, and the load that was already there

§11.5 asks for *"a slip-based force model, evaluated per wheel: compute longitudinal and lateral
slip from the wheel's contact patch velocity, look up the force curve, apply the force at the
contact point. Combined-slip handling by the friction ellipse. Load sensitivity from the contact
normal force the solver already recovered."* Split in two, along the same seam P7-G used:
`engine/domain/physics/include/SushiEngine/physics/vehicle/tyre.hpp` is slip and load in, force out,
and knows nothing about bodies;
`engine/domain/physics/include/SushiEngine/physics/vehicle/tyre_projection.hpp` finds the patch,
asks the model, and spends the answer.

**The brush model rather than the magic formula.** Pacejka's curve is a *fit*. Its
coefficients have no physical meaning on their own, they are measured on a rig from a real
tyre, and an author who has not measured one is left tuning fourteen numbers that
interact. The brush model is a *derivation*: bristles on the tread deflect until the local
shear reaches the friction limit and then slide, and integrating that over a parabolic
pressure distribution gives the whole curve from two stiffnesses and a friction
coefficient. Every number in `TyreSettingsT` is therefore something an author can reason
about, and the shape — a linear rise, a rounded peak, a fall into sliding — is a real
tyre's shape because it comes from the same argument and not because someone matched it.
The cubic reaches exactly `μN` with slope exactly zero, because its derivative is
`(1 − θ)²`; a kink at the limit is a discontinuity a car crosses several times a second
under hard driving and is felt as a tyre that grips and lets go rather than one that gives
way. The magic formula stays reachable — it would be a second `tyre_force` with the same
signature and nothing above it would change.

**One slip vector, saturated once.** Longitudinal and lateral slip are two components of
one vector and the saturation is applied to its magnitude, so combined slip is automatic
and cannot be got wrong. A tyre already at its limit under braking has nothing left to
steer with, which is what understeer under braking *is*, and it falls out rather than
being detected. Computing the two axes separately and clipping afterwards is the classic
mistake: it lets a wheel produce `μN` sideways *and* `μN` forwards, which is 1.41 times
the friction the surface has. Measured: two fully saturated axes together carry 4800 N
against 4800 N of `μN`, where a square limit gives 6788 N.

**The load is read back, not invented.** A wheel is a real rigid body with a real contact
(§11.2's fourth row), so the normal force is already a number the solver produced. Nothing
here raycasts for the ground or models a spring to it — a raycast wheel is the arcade
shortcut and it is the reason arcade cars cannot drive over a kerb: the ray finds the
ground and the wheel is not really there.

**And the units of that load were the one real defect.** `ContactPoint::normal_lambda` is
a *positional* XPBD multiplier, not an impulse: the solver spends it as a displacement, so
it carries an extra factor of the substep and the force is `λ/h²`. The contact solver's
own dynamic-friction pass is the proof — it compares `normal_lambda / h` against a
quantity in kilogram-metres per second. Getting it wrong does not look wrong: a load off
by the substep is a load off by a few hundred, and a tyre asked for grip proportional to
half a newton simply produces almost nothing. A car that rolls but will not drive, with
every formula in the model correct. `contact_point_load` is now the one place the
conversion is written.

**`IConstraintSolver` gained `body_handle`, the inverse of `body_slot`.** Contacts name
their bodies by slot, because they are rebuilt every tick and read by a device kernel that
has no notion of a generation. So anything that reads a contact and then wants to act on
the body it names — a tyre spending its reaction on whatever it is standing on, a damage
system asking what hit what — has a slot and needs a handle. Rebuilding it outside is not
possible and should not be: the solver is the authority on which generation a slot is on,
and a caller that guessed would address a body that had already been recycled.

**One patch per wheel, not one per point.** A manifold has up to four points and a wheel
can touch several bodies at once, but slip is a property of *the patch* — one velocity,
one slip angle. So the points fold into a single load-weighted patch, the model is asked
once, and the reaction is shared back by each contact's share of the load. Evaluating the
curve per point would saturate each separately and give a wheel across a kerb edge more
total grip than a wheel flat on the road, which is exactly backwards. Sharing the reaction
by load keeps the impulse the wheel gains equal to the impulse the world loses: measured
as 300.000000000 unchanged, to the last digit printed, across a wheel sliding on a movable
floor.

**The wheel must not also have Coulomb friction.** The solver's own friction runs inside
the substep loop on the same contact the tyre reads, so a gripping wheel gets both and
ends up with grip nobody authored and no single wrong number to find.
`SuspensionSetupT::material_index` exists so a wheel can point at a frictionless material.
This is the one mistake the file cannot detect for you, and it is stated where a reader
will meet it.

**What the measurements say.** Fourteen unit cases in `tests/unit/test_vehicle_tyre.cpp`. Small slip
reads the stiffness straight off; no slip in five decades takes the tyre past `μN`; the
curve rises monotonically and its largest single step near the peak is 30 N; friction
falls from 1.29 at half rated load to 1.02 at double and never to zero; a crawling wheel
reports a slip of 5e-4 rather than a singularity. In a solver: a parked wheel reports
196.2000 N against 196.2000 N of its own weight and drifts by nothing at all; a driven
wheel accelerates from rest to 2.0468 m/s against the 2.0400 m/s its held spin rolls at; a
locked wheel at 25 m/s is dragged at exactly `−μN` and slows.

**What P7-H does not do.** There is no carcass relaxation length, so a slip angle builds
instantly rather than over a few centimetres of travel — audible in a hard flick of the
wheel and not much else, and it is one first-order lag when it is wanted. There is no
camber thrust and no self-aligning torque, so a steering wheel has no weight to it; both
are additions to the same model rather than replacements. §11.5's pressurized node-beam
tyre is untouched and remains the other branch the asset may choose. And the coupling is
explicit at the tick boundary, like the powertrain's: the patch and its load are a tick
old.

**One thing found and not fixed, deliberately.** §7.3's manifold refresh anchors a contact
to a *material point* on a body, so a fast-spinning round body carries that point round its
rim within a tick and the bottom of the wheel stops being where the contact is — 38° in one
tick at 40 rad/s, which is a wheel at 50 km/h. It belongs to the narrowphase, where the fix
is to derive a round shape's anchor from the normal rather than carry it, and changing that
would move every rolling contact in the engine. It is recorded here because it is the reason
the tyre tests carry a rest offset, and because a vehicle at speed will meet it.

### 16.29 P7-I: the wind seam, and the term that had to be a difference

§4.5 names it before §11.6 describes it: *"`sim/` continues to hand the physics a
`GravitySampler` — an existing, good example of this principle already in the codebase —
and gains a `WindSampler` alongside it, backed by the atmosphere system's wind field."* So
`WindSampler` is the same shape as `GravitySampler` down to the signature, because the two
are the same *kind* of thing: a field the live world knows about and the solver must not.
Still air is a sampler that returns zero; a scene with no weather installed passes an empty
one, which is cheaper still.

**The force term had to be a difference, and that is the whole design.**
`RigidBodyT::drag_coefficient` already existed and `predict` already spent it, once per
substep, as `-k|v|v` — still-air drag on the body's own velocity. What a wind field changes
is not *that* there is drag but *what the drag is measured against*: the real force is
`-k|v − w|(v − w)`. Adding a second drag term on top would double-count. Replacing the first
is not available, because `predict` runs on the device inside one composition (§6.6) and
cannot call a host sampler at all. So `wind_drag_acceleration` returns exactly the gap
between the two, folded into `external_acceleration` beside gravity, and the two together
are the right force.

That decision buys the property that matters most: **still air costs exactly nothing.** Not
nearly nothing — a term that perturbed a scene with no weather in it would move every
determinism test in the suite by an amount small enough to be blamed on anything at all.
Measured: a tailwind at the body's own speed gives back exactly what `predict` takes
(0.800000000 against 0.800000000), a headwind costs exactly the difference of the two
squared airspeeds, and a wind of zero returns a hard zero.

**The cooker's `drag_area` finally reaches a body.** It has travelled in the
`.sushinodebeam` since P7-C and been read by nothing. Shell nodes now carry a drag constant
derived from it, from an air density and a plate's drag coefficient on the structure's
settings — derived rather than authored, because what an asset has is an area and what
`predict` spends is an acceleration.

**The car's own aerodynamics split in two, and the split is physical.** Drag is written
into the core's drag constant *at create time*, because drag is a constant and not a
per-tick force — and because that is the path the wind sampler already reaches, so a car
meets a gust through exactly the mechanism a flag on a pole does. Downforce is applied per
tick, because it is not drag: it acts along the car's *own* down axis, so a car on a banked
corner keeps it and one over a crest keeps it pointing at the road; and it acts at a centre
of pressure that is not the centre of mass, which is why a wing pitches a car under load.
That lever is the whole of aerodynamic balance, and modelling downforce without it would be
modelling extra grip. Measured at 60 m/s against `½ρClAv²` to 1e-9 relative, with the pitch
present when the wing is behind the centre of mass and exactly zero when it is on it.

Airspeed for downforce is the car's own speed and not its speed through the air, because
the wind sampler is `sim/`'s and `physics/vehicle` is not allowed to name it. A headwind
therefore adds drag, where the sampler does reach, but not downforce. That is an
understatement rather than an invention, and the alternative is this layer reaching for the
meteorology.

### 16.30 P7-J: the acceptance scene, and the editor

P7's row asks for one thing in one sentence: *"a drivable vehicle that deforms permanently
on impact and loses parts, at the §13.1 target, deterministic under replay."* Every earlier
P7 file tests one piece against its own closed form. `Integration_VehicleAcceptance` tests
that the pieces are a car, and it asks the four questions in the roadmap's own words.

**Drivable.** The throttle reaches the driven wheels through the clutch, the gearbox and
the differential: 51.4 rad/s at the rear, and the front two do not turn. Steering moves two
corners and not four. Off the throttle and out of gear, the brake stops the rear wheels to
0.005 rad/s — *out of gear*, because braking with the clutch engaged is braking against a
flywheel at 680 rad/s and the wheel settles where the two balance rather than stopping,
which is right and is not what that assertion is about.

**Deforms permanently.** A hull beam's rest length after the impact is *different*, and its
accumulated plastic strain is positive and bounded by the authored maximum. The magnitude
is micrometres, because the shell in that fixture is bolted to a rigid core and cannot move
far — and permanence is not a matter of degree. The assertion that earns its keep is `!=`:
a structure that merely bent and sprang back passes every test about forces and fails this
one.

**Loses parts.** A door tied to the hull by two beams and nothing else loses both, is
reported detached exactly once — not once per tick — and is 38 m away by the end, still a
pair of bodies still beamed to each other, which is what a torn-off door is.

**Deterministic under replay.** Two runs of the same crash agree bit for bit on every node
position, on the crankshaft, and on the wheel.

**The §13.1 row is measured and printed rather than asserted.** The target — 2 ms/tick for
400 nodes, 2 000 beams, four wheels and a powertrain — is quoted for one desktop-class GPU
through SushiRuntime, and the suite runs the host reference solver on a CPU. §16 already
says why that matters: a number from one solver that the other cannot produce is a number
nobody can compare. So what is asserted is the *shape* — that the scene really is 400 nodes
and 2 000 beams and four corners, all instanced and stepping — and the cost is reported for
a human to read. It is **0.460 ms/tick**, which is a useful thing to know and is not the
target being met.

Building that scene surfaced a real constraint worth writing down: **the colour count must
be at least the number of constraints touching any one body.** Graph colouring cannot put
two constraints sharing a body in the same colour, so fifty shell mounts on one rigid core
need fifty colours, and a configuration with sixteen instances nothing at all. It is not a
defect — it is what colouring means — but it is invisible from the capacity numbers, and a
vehicle is the first scene in this engine where one body carries dozens of constraints.

**The Vehicle window** is the authoring surface: corners, tyres, drivetrain, aerodynamics,
and the two asset identifiers `physics/vehicle` never dereferences. Its distinguishing
feature is the derived column. A vehicle editor that only echoed back what was typed would
be a form, and the numbers that catch mistakes are not the ones that were typed: `m·g/k`
beside the spring rate, the road speed at the limiter beside the gear ratio, the friction at
*this corner's* load beside the tyre's rated figure. A spring rate read as a compliance and
a ratio with the decimal point in the wrong place are both invisible in their own fields and
both obvious in those.

**What the editor does not do, and why.** It edits a *document* rather than a selected
entity's component, because there is no `Vehicle` component in the ECS: a vehicle reaches a
scene through `VehicleInstanceT` against a solver, and the authoring record has no owner in
the entity world to hang from. When that component exists the panel becomes an inspector
over it and nothing else in it changes. It has no viewport either — a live vehicle preview
needs its own render target and never the Scene view — and part and mounting-point authoring
lives in the cooked structure, where a part is a node's part index and a mount is an
attachment record, so the panel names the `.sushinodebeam` and the Cook & Bake window cooks
it.

### 16.31 PX-1: the joint component, and the field the boundary was dropping

P7 closed the last *building* phase before P8, and closing it exposed something the roadmap
had been recording without acting on since §16.8: a great deal of this system is built,
tested, and unreachable. §16.10 sized it for P3 — "the §14 assembly editor is the one item
outstanding, and it needs §5.5's `PhysicsJoint` component, its serialization and an
`ISimulation` surface for joints before it can be a panel at all, because `ISimulation`
deliberately does not expose the physics boundary" — and §16.15 repeated it for §14's seven
surfaces. This is the first of those, and the one every other one waits behind.

**What was actually missing was narrow.** `IJointService` exists and is complete: create,
destroy, read the load, replace the drive and the limits, and the broken-event stream. The
gap was one layer up. An entity could not *carry* a joint, so nothing an author did could
produce one, and nothing the scene file stored could restore one.

**The vocabulary had to move before anything could use it.** `JointType`, `JointLimitDesc`,
`JointMotorDesc`, `JointParams` and `JointState` were defined in
`engine/world/simulation/include/SushiEngine/simulation/physics_services.hpp` — which includes
`engine/world/simulation/include/SushiEngine/simulation/simulation.hpp`. So the authoring boundary
could not name a joint without closing an include cycle, and duplicating the types is exactly how a
new limit ends up honoured by a hand-built joint and silently ignored by an authored one. They now
live in `engine/world/simulation/include/SushiEngine/simulation/joint_params.hpp`, which both
include. Nothing in it names an entity, which is what keeps it includable from the header where
`EntityId` is defined — and that constraint is not a compromise: *what is held between two bodies*
is separable from *which two bodies*, which is the same split `JointParams` was already making so an
assembly asset could describe joints against part indices it has not yet resolved.

**The joint lives on one of its two bodies.** Not on a third entity naming both, because the
question is ownership: a door's hinge belongs to the door, and deleting the door should take
its hinge with it — which it does, for free, when the hinge is the door's own record. The
alternative leaves a joint behind pointing at something that is gone. The owner is body `a`,
so `anchor_a` and `axis_a` are read in the owner's own local space.

**Reconciliation is a diff, in the same shape and for the same reason as
`set_rigid_bodies`.** A joint that has not changed keeps its solver handle and therefore its
warm start, so merely stepping a scene rebuilds nothing. Staleness is a **revision counter**
rather than a comparison of the parameters: `PhysicsJointParams` is trivially copyable but
not free of padding, so a byte comparison can report a difference that is not one, and a
field-by-field comparison is a second place every future joint parameter must be remembered.
A counter cannot be forgotten, because the one function that bumps it is the one that writes.
The walk is over the authoring order, not the record map — joint identities are handed out in
call order, so a hash-order walk would be an unstable joint numbering, which is precisely the
kind of leak §12.1's first rule exists to forbid.

**A broken joint stays broken.** The solver destroys it and reports it once; the authoring
survives, because an author who set a threshold has not thereby deleted their hinge. A
runtime flag records the break and keeps the next reconcile from building the mount back —
without it a breakable joint tears off and reappears on the following tick, forever. The flag
clears when any field is edited, which is what an author changing the threshold means.

**One real bug, found by exercising the seam rather than by reading it.** `JointMotorT`
carries `damping`, added in P7-F so §11.2's spring-damper strut is one joint rather than two.
`JointMotorDesc` did not, and the boundary conversion therefore wrote zero. Everything
reaching the solver through `IJointService` — every assembly, and every joint this phase now
lets an author create — has been building spring drives with no damper, and a spring alone
rings forever. The field is now carried, converted, serialized and authored.

**The partner is serialized as an array index.** An `EntityId` is assigned at creation and is
not a property of the scene, so a file that stored one would reconnect to whatever entity
happened to be handed that number on the next load. It is written as an index into the entity
array and resolved in the same second pass the parent link already uses, since either end can
be written before the other — and the test that proves it deliberately creates the joint's
owner *first*.

**On verification.** Ten integration tests against the real simulation: a joint holds a body
up that a third, identically built and jointed to nothing, falls a metre without; the load
readout reports what the mount carries; a joint below its break threshold tears out, drops
the body, and does not come back until edited; destroying the partner releases the joint
without marking it broken, because nothing exceeded a limit; and every authored field —
including the recovered `damping` — survives capture and apply. The three states an
unconnected joint can be in are distinguished by the tests, not merely by the panel, because
all three read as a load of zero.

**What this does not do.** The Inspector's joint section edits the primary entity rather than
fanning out across a multi-selection, and that is a decision rather than a limitation:
`ComponentEditor` addresses a field by pointer-to-member and a joint's parameters are nested
one level, but the reason not to reach for the general mechanism is that fanning a joint out
would attach every selected entity to the *same* partner at the *same* anchor, which is never
what is meant. Joint *gizmos* — the axis and the limit arc, drawn and draggable in the
viewport — are §14's assembly-editor bullet and are PX-2's, not this one's.

### 16.32 PX-2, PX-5, PX-6, PX-7: the surfaces, and the type nothing was reading

§16.31 opened the exposure work by observing that a great deal of this system is built,
tested, and unreachable. This closes four more of §14's seven surfaces, and the most
important thing it found was not a missing panel.

**`PhysicsMaterial` was a type nothing read.** §5.3's material has existed since P0 with
static and dynamic friction, restitution, and four combine modes with a stricter-wins rule.
`submit_contacts` built **one** `ContactSolveParams` per tick at a hard-coded 0.6 / 0.5 / 0
and gave it to every contact in the world. So an ice cube and a rubber block on the same
slope did the same thing, always had, and no test caught it because every test that cared
about friction set the constant it was already using.

The fix is per-body materials on `RigidBodyDesc`, combined per pair by
`make_contact_params` — which already existed and had exactly one caller, in the soft-body
path. Carried by value rather than as an index into a scene table, for the same reason
`SoftBodyDesc` carries its constitutive material by value: `RigidBodyT::material_index`
exists so a *device* kernel can reach a material without following a pointer, and the
manifold pass that resolves a contact's surfaces runs on the host with the body's record
already in hand. A table would be a second place a material could live and a second thing
to keep in step.

One decision inside that is worth recording because the other answer is tempting. A side
with no rigid body — the standing plane, a cloth particle — resolves to the *default*
surface rather than to a mirror of the body it is touching. Mirroring reads as neutral and
is not: an ice cube should be slippery against the floor, and a floor that copied the
cube's friction would cancel exactly the difference the author authored.

**The filter was authorable only by the assembly instancer.** `CollisionFilter` has been
honoured by the broadphase since P2 and `instantiate_assembly` was the only thing that
could set one. `ColliderParams` now carries it, as a layer *index* rather than a mask —
a body is in exactly one layer, and offering a 32-bit field for a value with one bit set is
offering an author a way to write something the filter cannot mean. The shift happens once,
in `collider_from_params`. The Inspector's matrix states the rule in the words the rule is
actually in: two bodies interact only when *each* one's layer is in the other's mask, which
makes the relation symmetric by construction and makes a one-sided exclusion do nothing.

**The Assembly editor instances into entities.** This is the panel's whole architecture and
it is a choice rather than an implementation detail. An assembly instanced as an opaque
scene-graph node would be a second kind of thing the Hierarchy, the Inspector, undo, save,
serialization and the debug draw would each need a case for. Instanced as one ordinary
entity per part — Transform, Collider, Rigid Body, and one Physics Joint per assembly joint
— it is *already* all of those, and the parts stay editable afterwards, which is what an
author does with a ragdoll five minutes after placing one. The cost is stated in the
panel's own header rather than discovered: the instance forgets it was an assembly, so
re-instancing does not update one already in the scene. A prefab relationship is a real
feature and it is a scene-system feature, not a physics one.

The joint editor is shared. §10.1's argument for a single `JointParams` is that an entity joint and
an assembly joint are the same joint; a second copy of the *widget list* is exactly how a new limit
ends up editable in one of them and invisible in the other, so the widgets moved to
`applications/editor/source/physics/joint_widgets.cpp` before there were two callers rather than
after.

**The debug draw needed one new boundary call and no more.** Contacts reuse the stream
gameplay already receives — the same one, not a second — so what is drawn is exactly what a
listener would be told about, and an `End` event is drawn rather than filtered because a
contact that vanishes for one tick and returns is a symptom a filtered stream would hide.
What was genuinely invisible from outside the solver is a body's *bound*, its island and
whether it is asleep: a bound is not the collider, an island is not a component, and
"asleep" is the difference between a settled stack and a broken one.
`IRigidBodyService::rigid_debug_state` reports the three, and computes the bound with the
same routine the broadphase uses rather than reading a stored proxy — a debug view whose
boxes lag the bodies they belong to is worse than none, because it looks like a broadphase
bug.

The joint gizmo's arc is the half of it worth the code. A hinge's limits are two numbers in
radians, which is the quantity nobody can picture; an arc drawn from the joint's own zero,
in the joint's own plane, turns "0 to 1.7" into a door that opens most of the way — and a
sign error into a door that opens backwards, visibly, before anyone plays it.

**§14's material preview is a derivation, not a simulation.** §14 asks for "a ball dropped
on a ramp at the authored friction and restitution". The two rows beside the fields are
that scene's *answers*: an object begins to slide at `atan(static friction)` and returns to
`restitution²` of the height it fell from. Those are exact, so a simulated ramp could only
reproduce them with noise on top; it would also want its own render target and its own
physics world, and neither buys an author anything the two rows do not already say. The
sample scene carries the simulated version — two blocks differing in nothing but their
material, on one slope — where it can be watched rather than parameterized.

**On verification.** Six integration tests over the authoring surface, and the two that
matter most are comparisons rather than absolutes, because gravity here is the celestial
sum sampled per body and not a constant: a body at 0.9 restitution returns visibly more
height than one at 0, and a 17-degree ramp holds a block at 1.4 static friction while
losing one at 0.02. The filter test carries a control pair — identical geometry on default
layers — because a filter bug that dropped *every* contact would otherwise read the same as
a working exclusion.

**What the exposure work still owes.** `VehicleInstance` is not built: a vehicle reaches a
scene through `VehicleInstanceT` against a solver, and putting one in a scene needs a
vehicle service on `IPhysicsScene`, `.sushinodebeam` loading, per-tick stepping and a pose
write-back for four hundred nodes — the largest single item in this stream and the one
whose absence keeps the Vehicle window a document editor rather than an inspector. Drive
input follows it. Joint gizmos are drawn and not draggable; dragging needs hit-testing and
a drag state machine, which the transform gizmo has and this does not share yet.

### 16.33 PX-3 and PX-4: the vehicle in a scene, and the two ticks it has to fit between

§16.32 closed with a list of what the exposure work still owed, and `VehicleInstance` was
the largest item on it. P7 built a drivable vehicle and proved every clause of its
acceptance row — but against solvers the tests constructed themselves. Nothing could put
one in a scene.

**The component stores a path, and that is the one asset reference at this boundary that
does.** A soft body's `.sushisoft` crosses as bytes, because whoever cooked it owns them
and the physics copies out what it needs. A vehicle is different in a way that decides the
representation: it is *placed by an author, in a scene file, that has to survive being
reopened on another machine* — and the only thing that survives that is a path relative to
the project. The bytes are read once per path and held on the record, because a vehicle
blob is megabyte-scale and a reconcile fires whenever any vehicle in the scene changes; a
read that fails is remembered as having failed, so a path typo is one `open` rather than
one per tick.

**`set_vehicles` is a rebuild and says so.** Every other reconcile at this boundary is a
diff — `set_rigid_bodies` is explicitly "a diff, not a rebuild", and that property is what
lets a body keep its live velocity across a scene edit. A vehicle cannot have it. Four
hundred bodies and two thousand beams are placed *relative to a cooked structure*, so "the
same vehicle with one number changed" is not a thing that exists to be patched, and a
partial rebuild would leave half a car built to one setup and half to another. The
interface says rebuild, the panel's button says rebuild, and the cost is paid where an
author asks for it rather than per tick.

**The tick order is forced, not chosen.** This is the part worth recording, because it
looks arbitrary and is not. `VehicleInstanceT::begin_tick` does three things — downforce,
tyres, drivetrain — and the tyre model reads its normal load off the *contact multipliers
of this tick's manifolds* (§11.5, §16.28). So it cannot run before `submit_contacts`,
because the manifolds do not exist yet. It applies velocity impulses at the contact patch,
so it cannot run after `solver_->step`, because the velocities it would be correcting have
already been solved. There is exactly one point in the tick that satisfies both, and that
is where it went. The structure's own boundary — beams denting, mounts tearing out, a part
that has lost its last tie reported once — runs after the solve, beside
`break_overloaded_joints`, for §6.6's standing reason: a topology change never happens
against a running graph.

**The pose that comes back is the core's.** §11.2's hybrid puts the mass and the inertia in
one rigid body and hangs a deformable shell off it, so a node's position is a *panel's*
position and only the core's is the car's. Writing back a node would make a dented door
move the entity.

**Input is held, not consumed.** `set_vehicle_input` records; the step spends. Throttle is
a state an input device holds down rather than an event it delivers, so a caller that stops
calling keeps the pedal where it left it — which is what a pedal does, and what lets the
Vehicle window's slider and a future input action drive the same car without either of them
having to know whose turn it is. The record is kept on the *authoring* side as well, so a
panel can show back what it asked for even on a tick where the vehicle is not live: an
author dragging a throttle at a car whose asset failed to load should see the slider move
and be told why nothing else does.

**The panel edits a component here and a document everywhere else.** §16.30 recorded the
Vehicle window as a document editor and named the missing component as the reason. It now
has both, and the split is deliberate rather than transitional: the corners, the tyres and
the drivetrain are the same numbers whether or not anything is placed, and only the Scene
tab is about a car that is in the world. §14's node/beam visualization lives there as a
side elevation drawn in the panel rather than in the viewport, following the editor's own
rule that a preview gets its own surface — and a side elevation is the view in which a
sagging suspension and a caved panel are both visible at once.

**On verification.** Six integration tests, and the asset is **cooked into a real file**
rather than handed over as bytes, because a path is precisely what this phase is about. A
named structure becomes six shell bodies and a readable drivetrain; the entity falls
because its core does, against a control entity that does not move at all; full throttle
against a disconnected clutch spins the crankshaft up and lifting off lets it fall back;
the three ways to have no vehicle — no path, a path that did not load, no component — are
distinguished rather than collapsed, because all three read as a stationary car; the path
survives capture and apply and the vehicle is live again after it; and detaching the
component takes its bodies out of the solver rather than merely hiding them.

**What is still open.** The authored *setup* is not serialized — the scene file stores the
structure path, and a reloaded vehicle comes back at the default corners and drivetrain
until `VehicleAssetT` has a serializer of its own. That is stated in the writer where it
happens rather than left as a silent loss. Drive input reaches the vehicle from the panel
and not yet from the input manager, so a car is driven with sliders rather than with keys.
And joint gizmos are drawn but not draggable, which §16.32 already recorded.

### 16.34 PX-8, PX-10, PX-11: the car became visible, drivable, and openable

Three gaps closed, all of the same kind: the vehicle worked and nobody could *see* it, drive
it, or open the scene it was in. Worth recording together because the pattern is the point —
§16.33 scoped PX-3 to "reaches the solver", and that turned out to be two steps short of
"reaches a person".

**The shell is drawn as the surface it collides as.** `IVehicleService::vehicle_surface`
publishes the node cloud's collision triangles on the same deformable channel cloth and soft
bodies already use, which is P6-G2's argument reused: how a surface's triangles were produced
is not something the renderer can see. Read straight off the live node bodies with no cache
in between, so a dented panel is dented on screen in the tick it dented — and the drawing
cannot disagree with the collision, because they are literally the same triangles.

The cooked asset also carries a *render* skinning — up to four nodes per vertex, stored as a
displacement in the frame those nodes' arrangement implies, which reproduces the rest pose
exactly where a weighted sum of node positions would visibly shrink the mesh at every corner.
That is the prettier answer and it is not drawn yet, because it needs the visual mesh's own
index buffer, which lives in the visual asset and not in the `.sushinodebeam`. What is drawn
is the surface the physics owns end to end.

**The controls are ramped, and the ramp is in the input layer.** A key is a bit and a
throttle is not. Setting the pedal to 1 the instant a key goes down is what makes keyboard
driving in a physically simulated car undriveable: every input is a step, every step is a
shock through a drivetrain that models its own inertia, and the tyres spend their life at the
limit. Each control moves toward its target at a rate, and the rates differ because the
mechanisms do — a throttle cable is quick, a steering rack slower, and both return to centre
faster than they leave it.

The placement matters more than the rates. The ramp is a property of the *input device*, not
of the car, so it lives in the editor's input layer: a wheel-and-pedal set would deliver the
same `VehicleInput` with no ramp at all, and the vehicle must not be able to tell which it is
talking to. That is §4.5's rule applied to a seam nobody had thought of it as.

Arrow keys rather than WASD, on their own input context that is a no-op unless the selection
carries a Vehicle. W, E and R are the gizmo keys; a driving binding that stole them would
make the tool keys stop working the moment an author selected a car, which is exactly the
modal surprise a rebindable context exists to prevent.

**The sample scene ships as a file, generated rather than written.** A scene is a *file*, and
shipping the demonstration as a builder behind a menu item made it the one scene in the
engine that could not be opened, edited, saved or diffed like the thing it demonstrates.
`examples/physics_sample_scene` writes it, from the same builder the integration suite steps
and asserts on — a hand-authored blob would be a second definition of the scene, free to
drift from the tested one, and the drift would surface as a demo that quietly stopped
demonstrating something.

Removing the menu item before this existed was the wrong order, and building the replacement
found a latent break worth recording: the builder took an `EditorContext`, which drags ImGui
in behind it, which made a *scene builder* impossible to use from anything that is not the
editor shell — including the test target that had just started compiling it. `se editor`
builds with tests off, so it had not fired. The builder takes a world and nothing else now;
recording undo and moving a selection are the caller's business, and were never the scene's.

**And one crash, whose cause is a class rather than an instance.** `scalar_field` and
`vector3_field` open with `ImGui::TableNextRow` — they are *row emitters* for the two-column
table the Inspector's Transform block sets up. Their doc said so, as a description of what
they do rather than as a condition on where they may be called, and the Assembly window
called one outside a table: a null current table, dereferenced, on the first frame the panel
was opened.

The fix was to make the widgets **total** rather than to teach every future caller a rule.
They lay themselves out according to where they find themselves now — a table row inside a
table, a plain labelled drag outside one — so there is no precondition left to forget. A
widget that works in one context and crashes in another is a trap whose only defence is
documentation nobody re-reads.

### 16.35 Where this stands, and what to pick up next

The exposure stream (PX) took everything P0–P7 built and made it reachable without writing
C++. Eleven of its twelve items are done and recorded in §16.31 to §16.34. What follows is
the honest remainder, in the order it should be picked up.

**PX-9, the one item left in the stream, and the one that blocks an author outright.**
`NodeBeamCooker` exists (P7-D, §16.24) and has no entry point outside C++: the Cook & Bake
window produces `.sushicollision` and `.sushisoft` only. So step one of `docs/guides/vehicles.md` —
cook a mesh into a `.sushinodebeam` — cannot be done from the editor at all, and the test
suite cooks its own asset in C++ because that is the only way there is. Wiring it needs the
cooker's settings on the bake surface (the material, the core mass fraction, the
structural/bracing length ratio, the skin radius) and its cook report beside the other two.
Nothing structural stands in the way; it is the same shape as the two cookers already there.

**Three smaller things, each stated where it lives rather than left to be discovered.**

| Open | Where it bites | Why it was deferred |
|---|---|---|
| `VehicleAssetT` has no serializer | A saved scene stores the structure path; the vehicle reloads at the *default* corners and drivetrain | It is a large nested record, and writing it field by field in the scene serializer would be a second definition of it |
| The cooked render skinning is not drawn | The car is visible as its collision surface rather than as its art | Needs the visual mesh's index buffer, which is in the visual asset, not the `.sushinodebeam` |
| Joint gizmos are drawn, not draggable | A hinge's limits are typed rather than dragged | Dragging needs hit-testing and a drag state machine; the transform gizmo has both and this does not share them yet |

**Then P8.** Its twelve items are broken down in the roadmap row and none has started. Two
things about it are worth having written down before anybody begins:

1. **P8-A is not negotiable as the first step.** §16.21 measured 29.4 ms against a 3 ms budget
   and stated plainly that it could not distinguish whether the cost was the arithmetic or the
   1 024 barriers per tick. Building the device broadphase, narrowphase and contact solve
   without that answer is months of optimization not knowing which bottleneck is being
   removed.
2. **P8's acceptance cannot close on the current machine.** Every §13.1 target is written
   against "one desktop-class GPU through SushiRuntime", and SushiRuntime finds exactly one
   device here — an `AMD Ryzen 5 7600X`, the CPU backend. The work is doable and measurable;
   the *acceptance* stays open the same way P6's soft-body line does, and reporting a CPU
   number as if it had met a GPU target would be the one thing §16 has never done.

### 16.36 PX-9, and the exposure stream closed

The twelfth item, and the one §16.35 said blocked an author outright: `NodeBeamCooker` (P7-D)
existed and had no entry point outside C++. It has one now, and it is the shape §16.35
predicted — "nothing structural stands in the way; it is the same shape as the two cookers
already there" — with one exception the prediction also named.

**The exception is where the settings live, and it is not `CookingParameters`.**
`NodeBeamCookerSettings`'s own doc comment (§11.3) already said why: a material, a core mass
fraction, a structural-length ratio and a skin-search ratio belong to *this* cook and no
other, and folding them into the record every cooker shares would make every collision and
soft-body asset in the project carry four fields it has no use for and hash into a cache key
it cannot use. `CookingThresholds` had already answered the same question once, by living on
`ImportProfile` rather than on `CookingParameters` — read by one cooker, set on it before each
cook, no part of the shared dial. `ImportProfile::node_beam_settings` takes the same seat.
`CookingParameters::cook_node_beam` is the one bit that *does* belong on the shared record,
because "which cookers run" is exactly what that record already answers for the other two —
and `cooking_parameters_hash` folds it in beside them, the same as `cook_collision` and
`cook_soft_body`.

**The chain gained a class, not a template instantiation.** `CookerPostProcessor<Cooker>` is
the shape `CollisionPostProcessor` and `SoftBodyPostProcessor` share, and it is one line —
`cooker_.set_thresholds(...)` — away from fitting a third. That one line is the whole reason
`NodeBeamPostProcessor` is its own class rather than a third template argument bent to carry
it: `set_settings(profile.node_beam_settings)` has nowhere to go inside a shape built for
cookers that take only thresholds, and forcing it in would have meant the template stops
describing what the other two *are* in order to describe what one of three merely *has*.
Registered last in `with_shipped_processors()`, at `POST_PROCESS_ORDER_NODE_BEAM` — reserved
for exactly this since P7 — because a node-beam cook is rarer than either of the other two and
the cheap ones should not wait behind it.

**The Bake panel's material picker is an action, not a stored choice.** The obvious widget —
a combo whose current position reflects the settings' current material — has nowhere to read
its position *from*: `NodeBeamCookerSettings` carries the numbers a material resolves to, not
a tag saying which named material produced them, and it should not grow one only to feed a
combo box. So the picker applies a preset when one is chosen and relabels itself back to
"Apply preset..." on the very next frame, which is honest in a way a persisted selection would
not be — an artist who nudges Young's modulus after picking "Sheet steel" has a material that
is no longer sheet steel, and a combo still reading "Sheet steel" would be lying about it.

**The report shows what this cooker measures and nothing it does not.** §8.5's rule — a field
that is structurally always zero is the same failure as an invented number wearing the
opposite mask — applies to a panel reading the report exactly as much as it applies to the
report itself. `draw_node_beam_report` reads nodes, beams, the bracing count that says whether
§11.3's diagonal rule fired at all, unbound vertices, the Hausdorff departure, mass and
inertia; it does not read `tetrahedron_count`, `worst_element_quality` or the convex-piece
fields, which stay at their default for a node-beam cook and would read as measurements of
something nobody measured.

**Tested through the chain the panel actually drives, not only through the cooker directly.**
The node-beam suite (§16.24) already covers the cooker in isolation; what §16.35 named as
missing was the *reachability*, so `Unit_MeshPostProcessorChain.CooksANodeBeamAssetWhenThe
ProfileAsksForOne` cooks a box through `with_shipped_processors()` with only
`cook_node_beam` set, and cooks it a second time to confirm the settings the profile carries
are part of what the cache key remembers — the one failure mode a wiring bug could produce
that no cooker-level test would ever see, since the cooker's own key logic is not what a
profile-driven cook exercises.

Twelve items into the exposure stream and none left open: P0 through P7 built a physics system
that could simulate a vehicle nothing but a test could reach, and PX made every part of it
something an author opens the editor and does. The three smaller items §16.35 named — no
`VehicleAssetT` serializer, the cooked render skinning not drawn, joint gizmos not draggable —
are unrelated to the stream and stay exactly as open as they were.

### 16.37 P8-A, measured: it is barriers, not arithmetic

§16.21's open question, answered rather than guessed at. With §18 R8 landed and
`samples/physics/soft_body_budget.cpp` reading the profiled report after its last timed tick, two
independent signals — the named-node breakdown and the per-worker busy/overhead split — agree,
and neither is close.

**`element_project`, the constraint kernel doing the actual FEM math, is under half the tick.** Two
runs: 13.952 ms of 28.170 ms (49.5%), then 13.221 ms of 29.430 ms (44.9%). Every other named node —
`predict` (~2%), `update_velocity` (~2%), `motion_measure` (~0%) — is noise beside it, and the
rigid-body node kinds (`contact_*`, `distance_project`, `joint_*`) report exactly zero device time,
which is correct and not a bug: this scene has no contacts and no joints, so those nodes dispatch
against zero live elements and do nothing. **So at most half the tick is the 1.3 million projections
§16.21 named as one of the two suspects.**

**The other signal says where the rest went, and it is not subtle.** Summed across the twelve
workers for that one tick: 8.187 ms busy against 513.895 ms stealing + polling + idle (1.6%
busy), then on the second run 7.233 ms against 185.984 ms (3.7% busy). A worker spending
96–98% of its time between dispatches rather than running one is not a machine short on
arithmetic throughput — it is a machine spending almost all of its time *waiting at* or
*coordinating around* barriers. 1,536 dispatches per named kernel this tick (more than the
32 colours × 32 substeps = 1,024 the earlier estimate used, because the contact and joint
kinds dispatch too, at zero cost, but still as barriers) is a lot of synchronization points
for a CPU backend's twelve workers to pass through, and the busy/idle split says that
synchronization — not the projection math — is most of where the 29 ms goes.

**What this settles for how P8 gets scoped.** A device broadphase, narrowphase and contact
solve — P8's headline deliverable — would remove work this scene does not have (it has no
contacts). It would not touch the actual bottleneck this measurement found, which is the
*shape* of the dispatch: one graph node per colour per substep, 1,536 barriers deep, on a
backend where crossing a barrier costs far more than the several-microsecond kernel it guards.
The lever P8 needs first is reducing how many of those barriers exist per tick — batching
multiple colours' work behind fewer dispatches, or a persistent-kernel shape that keeps workers
inside one submission across colours — before device-resident collision is the thing worth
building next. Recorded here rather than acted on: which of those shapes is right is a design
question this measurement was only scoped to unblock, not answer.

### 16.38 P8-B: the zero-capacity skip §16.37's own measurement was missing

The beam and element bands in `RuntimeGraphBuilder::build_graph()` already carried a
`band_capacity() > 0` guard on their colour loop, so a scene with no vehicle or no soft body
never compiled nodes for a kind it never uses — the comment beside each says so explicitly.
The contact bands (`contact_prepare`, `contact_position`, `contact_velocity`) and the joint
bands (`joint_project`, `joint_velocity`), plus the base distance-constraint band
(`distance_project`), had no equivalent guard. §16.37's own profiling scene — the P6 cantilever
lattice, `capacities.contacts = 0`, `capacities.joints = 0`, `capacities.beams = 0` — therefore
still built and dispatched all five contact/joint stages, every colour, every substep, each one
a real barrier crossing whose `when()` predicate could only ever come back false. That is not
what the beam and element bands next to them do, and there was no reason for the difference.

All six loops now carry the same structural guard. This does not touch the scene §16.37
measured — it still has 20,250 elements and nothing else, and `element_project`'s own 1,536
dispatches are unaffected — but it removes several thousand needless barrier crossings from
every scene that uses fewer than all five kinds, which is most scenes an author will ever
build: a rigid-only stack has no elements or beams; a soft-body-only scene has no contacts,
joints, or beams until something touches it. Measuring this scene's own number again is future
work — the point of this change is the *shape* it corrects, not a re-run of §16.37's harness
against a scene it was never the bottleneck for.

### 16.39 The `Execution::DynamicGraph`/`Region` seam, and what it does not yet do

§6.6 names `DynamicGraph` as the mechanism "one region per island" needs, and §17.5 records the
runtime-side prerequisite — sub-range cross-region ordering (§18 R3) — as Built. Neither of those
facts made the capability reachable: `RuntimeGraphBuilder` is the one file allowed to name
`SushiRuntime::` directly (this file's own header comment says so), and it never called
`Runtime::dynamic_graph()`. Every other physics file reaches the execution backend only through
`Execution::Context`/`Execution::Graph`/`Execution::Buffer`
(`engine/foundation/execution/include/SushiEngine/execution/context.hpp`), which published exactly
those three names and nothing region-shaped. So the runtime has carried this facility since before
P8 began, and nothing under `physics/` — or anywhere else in the engine — could have used it without
reaching past the seam that exists specifically so a runtime migration costs one file.

**`engine/foundation/execution/include/SushiEngine/execution/backend/runtime_backend.hpp` now wraps
it, the same member-wise way it already wraps `Graph`/`Context`.** `RuntimeBackend::Region`
re-exposes `add_parallel`/`add_host`/ `add_reduce` against a `SushiRuntime::API::Region&` exactly as
`RuntimeBackend::Graph` does against an owned `API::Graph`; `RuntimeBackend::DynamicGraph` wraps
`API::DynamicGraph` — `region(key)`, `has_region(key)`, `drop(key)`, `region_count()`, `size()`,
`compile_count()`, `run()`, `native_report()` — and `Context::create_dynamic_graph()` is the one new
entry point, beside `create_graph()`.
`engine/foundation/execution/include/SushiEngine/execution/context.hpp` publishes
`Execution::DynamicGraph` and `Execution::Region` next to `Execution::Graph`. Nothing about
`NodeDescriptor`, `ResourceAccess`, `BufferInterval`, or `ElementRange` changed or needed to: a
region records work with the identical access-declaration vocabulary a plain graph does, which is
exactly what lets `emit_node`'s eventual per-island form change *which graph object* it calls
`add_parallel` on without changing how it builds the `NodeDescriptor` at all.

`tests/integration/test_execution_dynamic_graph.cpp` proves the wrapper's contract at the seam
level: two regions keyed independently write disjoint slices of one buffer; dropping a region takes
effect on `has_region()` immediately, not only after the next `run()` (§6.6's own words, now
asserted); a fresh region at a new key can replace the dropped one's slice without disturbing the
region that was never touched; and a `run()` that follows no `region()`/`drop()` call since the last
one does not advance `compile_count()` — the late-binding promise a static `Graph` already had to
keep, restated for the mutable one.

**What this is not.** No physics file calls any of this yet. The seam existing is the
prerequisite the physics-side wiring was blocked on; it is not the wiring, and §16.42 states
plainly why that half is not this session's.

### 16.40 Budgets and their reporting: two structurally-zero fields, closed

§13.3 lists `PhysicsStatistics` as a P0 deliverable, but two of its fields were never actually
written to, which is the exact failure §8.5's rule and §16.36 both name for a report: a field
that reads as a measurement of something nobody measured.

**The per-node device-timing breakdown §18 R8 was closed to enable, and never wired in.** R8 landed
for P8-A (§16.37) — `add_parallel`/`add_host` forward `NodeDescriptor::name` through — but
`PhysicsStatistics::timings.solve_ms` stayed one number, and the panel's own comment said the split
was "a runtime ask, not something to guess at here," which had stopped being true. `PhysicsNodeKind`
(`engine/domain/physics/include/SushiEngine/physics/core/statistics.hpp`) names the twelve kinds
`build_graph` emits; `physics_node_timings_from_report`
(`engine/domain/physics/include/SushiEngine/physics/core/statistics_from_report.hpp` — the one file
under `physics/core` naming `SushiRuntime::Core::RunReport`, per §17.5's one-adapter rule) groups a
run report's rows by name into `PhysicsStageTimings::node_timings`, the same grouping
`samples/physics/soft_body_budget.cpp` already did by hand with a `std::map`.
`RuntimeGraphBuilder::refresh_statistics()` now calls it when profiling is on; the Physics panel's
"Solve, by node" section draws one row per kind that actually dispatched this tick, skipping the
rest rather than drawing a false zero.

**Continuous-collision escalation had a statistic, a budget nothing enforced, and neither.**
§13.2 item 6 asks for "per-tick caps on contacts, fracture events, and continuous-collision
escalations, all state-derived, all reported when hit." Fracture already has exactly this
shape (`FemFractureBudget`, `FemFractureReport::elements_skipped`). Contacts have it too, via
`PhysicsCapacities::contacts` and the cumulative `capacity_overflows` counter. Continuous
collision had neither: §7.5 tier 2 (`Physics::conservative_advance`, called from
`sim/physics_simulation.hpp::submit_contacts`) ran for every candidate pair that asked for it,
uncapped, and `PhysicsStatistics::continuous_escalations` — a field that has existed since this
struct was written and that the editor panel has always drawn a row for — was never assigned
anywhere, because tier 2 runs entirely on the host, outside the solver `PhysicsStatistics` is
copied from. `PhysicsConfiguration::continuous_advancement_budget` (default 256, `configuration
.hpp`) now caps escalations per tick on `FemFractureBudget`'s own reasoning; a pair that loses
the budget keeps tier 1's speculative manifold rather than being dropped, which is the safe,
over-generating direction §1.2 already established. `continuous_escalations` is populated for
the first time, and `continuous_advancement_skipped` reports whether the budget actually bound,
mirroring `elements_skipped`.

Both closed independently and both are covered by test: `tests/unit/test_physics_statistics.cpp` for
the per-name grouping (a synthetic report, not a live one, since a live one needs a device this
suite does not require); the continuous-advancement budget by the existing conformance/scene tests
that already exercise fast-moving bodies, extended to read the new fields rather than only the
position they land at.

### 16.41 Half-precision storage: the path exists, the verdict does not

§6.5's second half — `sycl::half` for a cosmetic body's *stored* position and velocity, `float` for
every projection that touches it — was deliberately left unbuilt until this phase could measure
whether it pays for itself;
`engine/domain/physics/include/SushiEngine/physics/soft/soft_body_instance.hpp`'s own comment said
so. `engine/domain/physics/include/SushiEngine/physics/soft/soft_body_half_storage.hpp` is that path
now: `HalfVector3`, three `sycl::half` lanes and no arithmetic of its own (a type that could add or
scale itself would invite doing so at eleven significant bits, which is the mistake the rule rules
out); `widen_half_vector3`/ `narrow_to_half_vector3`, the only two points storage becomes something
a projection may read or a projection's result becomes something storage may hold;
`SoftBodyHalfStorage`, which mirrors a `FiniteElementModel<float>`'s position and velocity at half
width and touches it only at those two seams — `widen_into` before a tick's `step()`, `narrow_from`
after.

**Additive, not wired in.** No body constructs one of these; `SoftBodyInstance` and
`SoftBodyPrecision` are unchanged. `tests/unit/test_soft_body_half_storage.cpp` checks the storage
path on its own terms — an exact zero round-trip, a round-trip error bound held across
representative magnitudes from millimetre to vehicle scale, `sizeof(HalfVector3)` actually half of
`Vector3T<float>`'s three lanes, a rest pose round-tripped within tolerance, and a free-fall
trajectory routed through the seam once a tick staying close to the same trajectory computed with no
seam at all — not performance.

**`samples/physics/soft_body_half_storage_budget.cpp` is the measurement §6.5 asks for, and it does
not render a verdict.** It steps the identical lattice scene twice, once with plain `float` storage
and once through the narrow/widen seam, and prints the mean/best wall-clock cost of each and their
difference. Its own comment states the asymmetry a reader must not miss: `FiniteElementModel` is
the host-only reference solver and is not yet a constraint kind in the device graph (that is
§16.42's SoA/device-collision gap, not this one's), so this measures a single-threaded host loop's
conversion cost, not the device-buffer bandwidth saving §6.5 is actually written about. A positive
result here is strong evidence for the device case; a negative result here is much weaker evidence
against it. Reading the printed numbers and deciding keep-or-drop needs a build, which this
environment does not have — recorded as open in §17.4 item 3, exactly where it already was, now with
a harness that can actually answer it.

**Addendum, same session, after build access arrived:** the harness ran. 1331 particles, 6000
elements, 32 substeps, 30 timed ticks — float storage: 12.8738 ms/tick mean, 12.7455 ms best;
half storage: 12.9666 ms/tick mean, 12.7989 ms best. **+0.7% mean, a small loss, not a win.**
Read against the asymmetry the paragraph above already states: this is the host conversion
cost, negative, which per that same reasoning is *weak* evidence against the device-buffer
bandwidth case rather than a verdict on it — FEM still is not a device graph kind, so the case
§6.5 actually asked about remains genuinely unmeasured. What this number does settle is the
narrower question of whether the storage path is free to adopt on the host reference solver
today: it is not, by three quarters of a percent, which is not nothing but is not the kind of
number that should block a decision that is really about the device path. **Still open**, now
with both halves of the honest picture on record instead of neither.

**Decision (§16.45's follow-up pass): dropped, not built.** Put to the project owner directly rather
than guessed at, given this document's own standing rule for genuine keep/drop forks — the answer
was drop. `SoftBodyHalfStorage` stays exactly where the addendum above leaves it: additive, tested
on its own terms, not wired into `SoftBodyInstance`/`SoftBodyPrecision`, and not further developed.
The reasoning: the one measurement available shows a small loss, not a win, and this environment has
no GPU to take the device-side measurement that would actually settle §6.5's real question —
building the wiring on spec, against evidence pointing the other way, would be the kind of work this
document's honesty rule exists to avoid committing to blind. If a device becomes available and the
device-buffer bandwidth case is worth measuring properly, this section and
`samples/physics/soft_body_half_storage_budget.cpp` are exactly where that work resumes.

### 16.42 What is still open, and why it is not this session's

Four items from the roadmap row remain undone, each for a stated reason rather than by
omission.

1. **The deeper barrier-reduction primitive §16.37 itself asked for.** Batching several colours'
   work behind one dispatch, or a persistent-kernel shape that keeps workers inside one submission
   across colours, needs a runtime call that does not exist: `Execution::Graph`/ `Region`'s
   `add_parallel` is one node in, one node out, with no shape for "this one call covers several
   ordered sub-ranges." Recorded as **R9** in §18 rather than approximated engine-side — an
   engine-side workaround (a kernel that loops over colours internally) would either serialize what
   colouring exists to parallelize, or reinvent the scheduler's own ordering guarantee inside a
   single node, which is precisely the "a layer that did would be a second scheduler hiding inside
   an adapter" trap
   `engine/foundation/execution/include/SushiEngine/execution/backend/runtime_backend.hpp`'s own
   file comment warns against.
2. **Per-island substepping's *physics* half.** §16.39 closed the seam; nothing under `physics/`
   uses it, and a second pass through the actual code (this session, checking §17.5's "already
   produces" claim against the files instead of trusting it) found the gap is larger than that row
   said: `ConstraintStore`/`ContactStore` colour bands are global today, spanning every body in the
   scene regardless of island, with no island concept anywhere in either file
   (`engine/domain/physics/include/SushiEngine/physics/solver/constraint_store.hpp`,
   `engine/domain/physics/include/SushiEngine/physics/solver/contact_store.hpp`) or in
   `IncrementalColoring`
   (`engine/domain/physics/include/SushiEngine/physics/solver/incremental_coloring.hpp`) —
   recolouring assigns a colour, not an island. And `IslandBuilder`/`IslandSet`
   (`engine/domain/physics/include/SushiEngine/physics/scene/islands.hpp`) is not a dormant layout
   waiting to be read; it is a separate host-only partition over body-slot indices, built *after*
   the solve runs (`engine/world/simulation/include/SushiEngine/simulation/physics_simulation.hpp`'s
   `update_islands()` follows `solver_->step()`, not the reverse), fed from *this tick's own*
   resolved contacts, and consumed today only to decide next tick's sleep state. Nothing in
   `physics/solver/` reads it, and `tests/unit/test_islands.cpp` tests it in total isolation from
   the solver.

   That ordering is the actual blocker, not a formality: shaping *this* tick's solve by island needs
   the partition before the solve produces the contacts the partition would be built from. Two
   honest ways out, neither decidable by guessing — **(a)** solve from the *previous* tick's island
   partition, accepting the same one-tick staleness sleeping already tolerates (a body newly
   touching another mid-tick joins its region one tick late), or **(b)** restructure the tick so
   islands are rebuilt from the previous tick's resolved contacts *before* this tick's solve, which
   changes what "this tick's contacts" means for every other reader of `PhysicsStatistics.islands`.
   Either choice also still needs the whole-capacity `motion_maximum` fixed-order reduce (§16's
   `build_graph`, the final `add_reduce`) answered, since it has no `DynamicGraph`-wide equivalent:
   N per-island partial folds behind a small aggregator, or moving it outside the `DynamicGraph`
   entirely. A wrong call on (a) vs (b), made blind against a codebase this session cannot compile,
   would not fail to build — it would silently change which tick's geometry a body's constraints
   solve against, which is exactly the failure class item 3 below is also declined for. Recorded
   here, with the ground truth behind it, for the pass that takes it on with the project owner's
   answer on (a) vs (b) in hand.
3. **Structure-of-arrays state columns.** `RigidBodyT<T>` and every constraint/contact descriptor
   remain one struct per element in one `Buffer<T>`, exactly as §13.2 item 4 still describes it as
   future work. Splitting the hot fields a projection actually touches from the cold ones it does
   not touches every kernel's capture list,
   `engine/domain/physics/include/SushiEngine/physics/solver/host_solver.hpp`'s reference
   implementation the conformance suite holds the device solver to,
   `engine/world/simulation/include/SushiEngine/simulation/physics_extract.hpp`, and the editor's
   debug draw — about 60 files reference a `RigidBodyT` field directly, checked by grep rather than
   guessed at.

   **The reason to decline this is no longer "no compiler here."** §16.44's pass gained build and
   test access (`se build -t relwithdebinfo`, `se test`) partway through this phase, which changes
   what "attempted blind" means: a wrong column split is now caught two ways it was not before —
   most mis-splits fail to *compile* (a call site still naming the old field), and the ones that
   compile but disagree between host and device are exactly what
   `tests/integration/test_solver_conformance.cpp` already exists to catch (confirmed still 100%
   passing this pass, `se test`). What did not change is the size of the job: about 60 call sites,
   done carefully rather than quickly, is a dedicated pass's work, not a slice of one that also does
   three other things — and §16.37's own numbers say it is worth doing carefully. `element_project`,
   the FEM kernel, is 44.9–49.5% of the tick by itself (§16.37, not the 96–98%
   *idle-between-dispatches* figure R9 is about, which is a different measurement of the same tick)
   — a real, substantial share a tighter working set could plausibly cut into, not the sliver a
   hasty read of §16.37 could mistake it for. Recorded here, not attempted here.
4. **Device-resident broadphase, narrowphase, and contact detection.** §16.37's own conclusion
   stands: the measured scene has no contacts, so this would not have moved that number, and
   building an LBVH construction and parallel pair-generation kernel, plus a device narrowphase
   dispatch table, is the largest single item in this phase's original scope. Build and test access
   does not close as much of the gap here as it does for item 3: a device kernel's syntax compiles
   clean or it does not, and `se test` runs the conformance suite on the *host* reference solver
   this backend has, but neither replaces designing a parallel tree-construction algorithm correctly
   the first time against a device this machine cannot run one on to see fail. Sequenced last
   deliberately, per §16.37's own recorded reasoning, not dropped.

**Every §13.1 acceptance target still needs a GPU.** §16.35's finding is unchanged: this machine's
`SushiRuntime` finds one device, the `AMD Ryzen 5 7600X` CPU backend, and every target in §13.1 is
written against a desktop GPU. Nothing in this update closes that gap — it could not be closed here
— and nothing above claims a number this machine cannot produce.

### 16.43 A second pass on §18, and one aspirational claim §17.5 was carrying

Asked to finish P8 outright rather than leave the four items above open, the honest next step was
checking whether they actually were open, the same discipline §18's own correction block already
demanded once (*"an engine-side claim about a runtime API is only tested by a translation unit that
instantiates it"*). Two findings came out of that check, both by reading the runtime's headers
directly rather than trusting either document's prose.

**§17.5 was wrong, not just optimistic.** Its island-per-region risk row closed on the sentence "an
island must be a set of index ranges the solver can name with `Buffer::region({offset, count})`,
which the incremental recolouring in §6.4 already produces." Checked against
`engine/domain/physics/include/SushiEngine/physics/solver/constraint_store.hpp`,
`engine/domain/physics/include/SushiEngine/physics/solver/contact_store.hpp`,
`engine/domain/physics/include/SushiEngine/physics/solver/incremental_coloring.hpp`, and
`engine/domain/physics/include/SushiEngine/physics/scene/islands.hpp` directly: recolouring produces
colour bands, not island ranges, and `IslandBuilder`/`IslandSet` is a separate host-only partition,
built *after* the solve from *this tick's* resolved contacts, read by nothing under
`physics/solver/`. The row is corrected in place above rather than left standing on a claim nobody
had checked. This is why item 2 above reads differently than the version of this section written
before the check: the blocker is now the actual one (islands are known after the solve that would
need to shape itself by them), not the assumed one (a data-layout change with no other obstacle).

**R4 and R6 were already built and this document had not caught up.** `sushiruntime`'s own
`PHYSICS_SUBSTRATE_REQUIREMENTS.md` says so, but §18's own correction block is explicit that a
sibling document's claim is not evidence either — so both were checked directly against
`engine/domain/audio/include/SushiEngine/audio/dsp/graph.hpp`, `run_handle.hpp`, and
`api/vocabulary/dynamic.hpp` on *this* checkout before the §18 rows below were changed from Open to
Built. Both are real: `run_async()`/`RunHandle` for R4, `based_at()`/`based_at_device()` for R6 —
and `engine/foundation/execution/include/SushiEngine/execution/backend/runtime_backend.hpp`'s
`Detail::to_dynamic` already forwards a bound `node.base` to `and_based_at`, which is the
runtime-side half of exactly what item 2's per-island regions would need to shift a band's base per
region. Neither is consumed by physics yet; both are now recorded as unused capacity rather than a
missing primitive, so the next pass at item 2 is not also rediscovering that the seam is ready.

**R9, raised in this document, was not yet recorded where §18 itself says the record belongs.**
§18's own first line: *"the runtime-side engineering request... lives in
`sushiruntime/docs/design/PHYSICS_SUBSTRATE_REQUIREMENTS.md`."* R9 was added to this document's §18
table without a matching entry there. Fixed — R9 is now in that document's delivery table too,
alongside R8 (raised and closed after that document's own "all seven" framing was written, and also
missing until this pass).

This pass alone closes none of items 2, 3, or 4. It narrows item 2 to an actual, statable design
question instead of a shrug, and leaves the runtime side of the ledger accurate for the next reader.
What follows in §16.44 is a later pass in the same session, made possible by something that changed
mid-phase: build and test access (`se build -t relwithdebinfo`, `se test`), which turned "no
compiler here" from this document's standing reason to decline items 2 through 4 into a reason that
applies unevenly — some of the remaining work is now genuinely safe to attempt, and some still is
not, for reasons restated below with that distinction in mind rather than the blanket one this
section was written under.

### 16.44 Item 2, the slice that was actually safe: sleeping joints park

§16.42 item 2's design question — solve from the previous tick's island partition, or restructure
the tick so islands are known before the solve they would shape — is not a question this pass
answers, because it still is not this pass's to answer blind. But tracing *why* the physics-side
wiring was believed to need that answer first turned up something the roadmap row's "per-island
substepping" framing did not separate out: §13.2 item 1's actual claim, *"a settled island costs its
broadphase bound update and nothing else,"* was already true for contacts
(`engine/world/simulation/include/SushiEngine/simulation/physics_simulation.hpp`'s `submit_contacts`
already skips placing a contact between two bodies that are not simulated — checked directly, not
assumed) and **was not true for joints**, which stay resident in their colour band forever,
dispatched every substep, with the projection's own
`has_any_flag(a.flags | b.flags, BodyFlags::sleeping)` check the only thing standing between a
parked vehicle and a crashing one paying the same rate — the exact case §17.5's risk table already
named and left unfixed.

That gap did not need the `DynamicGraph`/island-partition machinery item 2's larger form does. It
needed exactly two existing primitives already proven correct for a different lifecycle event —
`IConstraintSolver::read_joint`/`remove_joint`/`add_joint`, the same three `remove_body` already
uses to take a destroyed body's joints with it — called from a new tick-boundary pass,
`PhysicsSimulation::update_joint_parking()`, run after `update_islands()` has written this tick's
sleep decision. A live joint whose two ends satisfy the *exact* condition the projection already
early-outs on is removed from `joints_store_` and its full solved state (motor, limits, peak load —
everything `read_joint` returns, not just the authored parameters) cached on `JointEntry`; a parked
joint whose condition no longer holds is re-added from that cache. Mirroring the kernel's own
condition rather than inventing a stricter or looser one is what makes this provably inert on
correctness: a joint is parked only on a tick where the kernel was already contributing nothing to
the solve, so nothing about *what* the solve computes changes, only whether a no-op still pays for a
dispatch.

Two edge cases the mirror alone does not cover, both handled: `joint_state`/`set_joint_motor`/
`set_joint_limits` read through a live `JointHandle` and would otherwise fail on a parked joint
(handle intentionally invalid) — `joint_state` now falls back to the cached state, and an edit to a
parked joint's motor or limits unparks it first, on the same "a disturbance wakes it" precedent
`create_joint` already sets. A capacity overflow on the way back in is left parked rather than
losing the joint's state, matching `add_joint`'s own reporting convention.

**Opt-in, off by default, and live rather than construction-time** —
`IPhysicsStepper::set_park_sleeping_joints_requested`, mirroring `set_profiling_requested`'s shape
but not its "before the scene first steps" restriction, since parking is tick-state, not solve-graph
construction. `tests/integration/test_joint_parking.cpp` (four tests, `se test` green, no
regressions across the 1364-test functional suite beyond two pre-existing failures in unrelated
systems — atmosphere and vehicle-component code this pass never touched) proves: off by default
nothing changes; a settled island's joint drops `PhysicsStatistics::joints` to zero; a
teleport-driven wake restores it with its state intact and the body still ends up held where the
joint says it should; editing a parked joint's motor wakes it immediately rather than failing as if
it had no live state.

**What this is not.** Beams, elements, and authored distance constraints stay resident whether their
island sleeps or not — the same fix, generalized, needs a body-driven removal already proven for
joints (`remove_joints_touching` has beam and element siblings,
`remove_beams_touching`/`remove_elements_touching`, called today only from `remove_body`) but
checked this pass and found to need a prerequisite of its own first: **beams do not participate in
island connectivity.** `update_islands()` feeds `island_builder_.connect()` from contacts, joints,
and cloth — never from `vehicle`/node-beam structure — so two beam-linked nodes with no other
connection between them are, today, reported as separate islands despite being rigidly tied. Parking
beams by island membership on top of that gap would risk parking one node's beams while a
beam-only-connected neighbour is still being solved — the exact silent-corruption failure mode this
whole session has been declining to risk. **Recorded as a real, separate, newly-found bug** rather
than folded into this fix: closing it is adding a fourth `connect()` call site for whatever tracks
beam structure at the `PhysicsSimulation` level, which this pass did not locate and verify carefully
enough to change blind. The vehicle case — "a parked car costs what a crashing one costs" — is
exactly what would benefit most, and is exactly why it should not be the thing this pass guesses at.

### 16.45 The editor-connectivity audit: everything built that a user cannot actually reach

Requested directly: not "is P8 done" but "walk the whole physics pipeline and find what was built
and never connected." Four independent read-only passes — solver-layer feature inventory, ECS
component/binding tracing, editor-UI tracing, cooking-pipeline tracing — each citing file:line,
cross-checked against each other rather than any prior claim in this document. The picture is more
gapped than §16.42-§16.44 alone suggested, because those sections were about P8 specifically; this
is every physics feature, P0 through PX, held up against what an editor user can actually do.

**16.45.1 — Built with zero authoring path: reachable only from raw `IPhysicsScene`/test code.**
Three features are fully implemented and tested against the low-level solver interface, but no
component, no binding code, and no editor UI ever calls them in a running editor — the *only* call
sites in the entire repository are test files constructing a bare `IPhysicsScene` directly:

- `park_sleeping_joints` (§16.44). `ISimulation::set_park_sleeping_joints`
  (`engine/world/simulation/include/SushiEngine/simulation/simulation.hpp`) has exactly one override
  (`RuntimeSimulation`, `engine/world/simulation/source/runtime_simulation.cpp`) and *zero* call
  sites anywhere against that override — no `Record` field, no editor UI, nothing in `editor/`
  mentions "sleep" or "park" beyond the read-only sleeping-body count and the sleeping debug-draw
  checkbox (`applications/editor/source/physics/physics_statistics_panel.cpp`), which visualize
  sleep state but do not toggle parking. Confirmed dead end-to-end, not merely "no UI yet" — there
  is no wiring at any layer between the ECS and this toggle.
- `IJointService::set_joint_motor`/`set_joint_limits` — the *live, in-place* joint-edit entry points
  (`engine/world/simulation/include/SushiEngine/simulation/physics_services.hpp:644,659`). The
  editor's joint UI (`applications/editor/source/physics/joint_widgets.cpp`) does let a user edit
  motor/limits, and it is fully wired — but only through `JointParams`, which `sync_joints`
  (`engine/world/simulation/source/runtime_simulation.cpp`) applies by **destroy+recreate** on any
  revision bump, never through these two methods. `set_joint_limits` has no call site in the entire
  repository, not even in tests. `set_joint_motor`'s only callers are
  `tests/integration/test_joint_assembly.cpp` and `tests/integration/test_joint_parking.cpp`, both
  against a bare `IPhysicsScene`. The live-update path this session built `unpark_joint()` on top of
  (§16.44) is itself unreachable from the editor.
- Body trigger/CCD flags. `Collider::flags`
  (`engine/world/simulation/include/SushiEngine/simulation/collider.hpp`) is read by the solver for
  both trigger detection
  (`engine/world/simulation/include/SushiEngine/simulation/physics_simulation.hpp`, feeds
  `ContactEvent::trigger`, `engine/world/simulation/include/SushiEngine/simulation/simulation.hpp`)
  and continuous-collision routing
  (`engine/world/simulation/include/SushiEngine/simulation/physics_simulation.hpp`) — but
  `collider_from_params` (`engine/world/simulation/include/SushiEngine/simulation/collider.hpp`),
  the only function that turns an authored `ColliderParams` into a `Collider`, never assigns
  `flags`; it stays `0` always. The single place in the whole codebase that sets
  `BodyFlags::trigger` on a `Collider` is `tests/integration/test_physics_simulation.cpp`, built by
  hand. **Trigger volumes and continuous-collision opt-in are fully solved and fully eventable, and
  completely unauthorable** — there is no checkbox, no `ColliderParams` field, nothing.

**16.45.1 corrections, made while closing items rather than guessing at them.**
`park_sleeping_joints` and the trigger/CCD flags were genuine gaps and are now closed:
`ColliderParams::trigger`/ `::continuous_collision`
(`engine/world/simulation/include/SushiEngine/simulation/simulation.hpp`), wired through
`collider_from_params`
(`engine/world/simulation/include/SushiEngine/simulation/collider.hpp:157-160`), an Inspector
"Behaviour" section (`applications/editor/source/scene/inspector_panel.cpp`), scene-file
round-tripping (`engine/world/serialization/source/scene_serializer.cpp`), and a new integration
test proving both directions — the body passes straight through and the overlap is still reported
(`test_physics_authoring.cpp:ATriggerVolumeReportsOverlapButNeverStopsTheBody`).
`park_sleeping_joints` is now a "Settings" checkbox on the Physics panel, staged on `EditorContext`
and pushed into `ISimulation::set_park_sleeping_joints` once a frame from
`applications/editor/source/main.cpp`, the same pattern `physics_statistics`'s read direction
already uses in reverse.

The `set_joint_motor`/`set_joint_limits` bullet does **not** get the same treatment, and reading
`sync_joints`'s own comment (`engine/world/simulation/source/runtime_simulation.cpp`) closely enough
to implement against it turned up why: *"Any edit is a new joint: the solver's is rebuilt on the
next reconcile rather than patched, because its multipliers were accumulated under the limits it is
being taken out of."* That is not an oversight, it is a stated, reasoned correctness choice — an
XPBD joint's accumulated Lagrange multipliers were warm-started under the *old* limit or motor
target, and patching just the target in place would let the next substep's solve start from an
impulse basis that no longer matches what it is being asked to satisfy. `touch_joint` is
deliberately blind to *which* field changed for exactly this reason: a motor-only edit is not
obviously safer to live-patch than an anchor edit, and the record does not currently carry enough
information to tell the two apart even if it wanted to.

So the honest correction is: **this was mis-filed as a wiring gap; it is a primitive built for a
caller that does not exist yet, not a live-update path the ECS forgot to use.** `set_joint_motor`/
`set_joint_limits` read as built for continuous, high-frequency joint control — a gameplay/
scripting API driving a motor target every tick, the way `SuspensionUnitT::set_steer_angle`/
`set_brake_torque` (`engine/domain/physics/include/SushiEngine/physics/vehicle/suspension.hpp`)
already do for vehicles, but through their own dedicated mechanism rather than `IJointService`, and
without `sync_joints`'s destroy+recreate in the way. No such caller exists today — nothing outside
`Vehicle` drives a joint continuously — so building one now would be speculative rather than closing
a real gap. Left as `#25`-style: not implemented, and not implemented on purpose, until something
needs it.

**16.45.2 — Components that exist but no panel ever writes them.** `SoftBodyParams`
(`engine/world/simulation/include/SushiEngine/simulation/simulation.hpp`) is wired end-to-end into
the solver exactly as thoroughly as `ClothParams` is — `gather_soft_body_descs`
(`engine/world/simulation/source/runtime_simulation.cpp`) reads every field, including `cosmetic`
(the storage-precision request, see 16.45.4). But the Inspector's "Add Component" popup offers Rigid
Body, Cloth, Collider, Physics Joint, Vehicle — **no "Soft Body"**
(`applications/editor/source/scene/inspector_panel.cpp`). `has_soft_body`/`soft_body_params` are
read in exactly one place in `editor/`, `applications/editor/source/main.cpp:919-927`, to feed the
debug-view overlay — never written by any panel. A general tetrahedral soft body cannot be placed on
an entity from the editor at all today; the only user-facing soft-body surface is the Bake window's
"Cook soft body" checkbox, which configures the *cooker* (an asset-pipeline step), not a live
entity's simulation parameters. `SoftBodyDesc::participates_in_rollback` compounds this — even the
test/hand-authoring path can't reach it, since `SoftBodyParams` (the authoring struct) has no
corresponding field, so it is always `false` regardless of what a `Desc` built by hand might set.

**16.45.3 — The cooking dial exposes four booleans and one slider; the pipeline reads seventeen.**
`applications/editor/source/physics/cook_bake_panel.cpp` lets a user set `fidelity`,
`cook_collision`, `cook_soft_body`, `cook_node_beam`, `static_geometry`, and (when node-beam cooking
is on) the node-beam material and shell-attachment settings. Every other field `CookingParameters`
declares — `voxel_resolution`, `target_tetrahedron_count`, `simulation_level_count`,
`convex_piece_count`, `distance_field_resolution`, `surface_conforming_passes`,
`suggested_substep_count` (all as *pin* overrides against the fidelity dial —
`engine/domain/physics/include/SushiEngine/physics/cooking/cooking_parameters.hpp`),
`hull_vertex_budget`, `weld_tolerance`, `density`, `accuracy_lattice_order` — is read by a cooker
(`engine/domain/physics/source/cooking/collision_cooker.cpp`/`engine/domain/physics/source/cooking/soft_body_cooker.cpp`/`engine/domain/physics/source/cooking/node_beam_cooker.cpp`/`engine/domain/physics/source/cooking/tetrahedral_mesh.cpp`,
file :line list in the audit transcript) but has no widget anywhere in
`applications/editor/source/physics/cook_bake_panel.cpp`; the panel shows their *derived* values
read-only (:407-412) and nothing more. `ImportProfile::thresholds` (`CookingThresholds`, applied by
every cooker via `apply_cooking_thresholds`) has zero references in
`applications/editor/source/physics/cook_bake_panel.cpp`. And `ImportProfileOverride` — the entire
per-asset-override mechanism §8.1 describes, `resolve_import_profile`-tested and working — has no
call site outside tests anywhere in the repository;
`applications/editor/source/project/project_panel.cpp` always bakes at the single project default,
so "per-asset cooking overrides" does not exist as a feature a user can reach, only as one that
compiles and passes its own unit test.

**16.45.4 — The half-precision path from §16.41 is one layer short of where the benchmark
measured.** `SoftBodyParams::cosmetic` *is* fully wired through to
`SoftBodyPrecisionRequest::cosmetic`
(`engine/world/simulation/include/SushiEngine/simulation/physics_simulation.hpp:429`) →
`resolve_soft_body_precision` → `SoftBodyPrecision::Cosmetic`
(`engine/domain/physics/include/SushiEngine/physics/soft/soft_body_instance.hpp:92-131`) — so the
*selection* is authorable, in principle, the moment 16.45.2's gap is closed. But
`SoftBodyPrecision::Cosmetic` being selected does not itself construct a `SoftBodyHalfStorage` —
that class's own header states nothing in `SoftBodyInstance`/ `SoftBodyPrecision` constructs one yet
(`engine/domain/physics/include/SushiEngine/physics/soft/soft_body_half_storage.hpp:57-59`).
§16.41's benchmark numbers (float 12.8738ms vs half 12.9666ms) measured `SoftBodyHalfStorage`
directly, not through this selection path — meaning the precision toggle §16.41 discusses and the
storage class §16.41 benchmarked are not yet the same wire. Marking the half-precision *feature* (as
opposed to the standalone benchmark harness) built would currently be wrong on two independent
counts: no editor path to set `cosmetic` (16.45.2), and no code that turns `Cosmetic` selection into
an actual `SoftBodyHalfStorage` construction even if one existed.

**16.45.5 — Profiling surface gaps.**
`applications/editor/source/physics/physics_statistics_panel.cpp` draws most of `PhysicsStatistics`
but not all of it: the aggregate `constraints` count is shown, but its per-kind breakdown —
`PhysicsStatistics::joints`/`::elements`/`::beams`
(`engine/domain/physics/include/SushiEngine/physics/core/statistics.hpp`) — never is, and
`PhysicsStageTimings::soft_body_ms`
(`engine/domain/physics/include/SushiEngine/physics/core/statistics.hpp:223`, the separate host XPBD
schedule's own timing line) is never drawn anywhere in `editor/`. Both are populated correctly by
`refresh_statistics()` — this is a display gap, not a measurement gap.

**16.45.6 — Confirmed dead fields, no behavior attached at all.** `Collider::asset`
(`engine/world/simulation/include/SushiEngine/simulation/collider.hpp`) — never assigned by
`collider_from_params`, never read anywhere; `Collider::applied_scale` (:118) — assigned but never
read by anything (comment says "carried for the cooker," a P4 consumer that does not exist yet);
`DerivedCookingParameters::fidelity`
(`engine/domain/physics/include/SushiEngine/physics/cooking/cooking_parameters.hpp`) — computed by
`resolve_cooking_parameters` and then never read by any cooker or report (the UI reads
`parameters.fidelity` directly instead, bypassing the derived copy entirely).

**16.45.7 — What does not have this problem.** Named for contrast, since a list this long risks
implying the whole pipeline is disconnected, which it is not: joint type/anchor/axis/compliance,
joint motors and limits and break thresholds (fully wired, editable, live-load readouts, both from
the Inspector and the Assembly panel), rigid body mass/inertia/density/drag, collider
shape/friction/restitution/collision-filter, cloth, and the entire vehicle authoring surface
(suspension corners, tyres, drivetrain/gearing, aerodynamics, live driving input, telemetry
readback, a dedicated preview viewport per §14's own convention) are all genuinely built, wired end
to end, and usable by a person sitting at the editor today. §14's other named surfaces — debug-draw
overlays for contacts/bounds/islands/joints, the Bake window's core cook/re-cook loop — work as
documented. The gaps above are real, but they are gaps *in* an otherwise-connected pipeline, not
evidence the pipeline itself is aspirational.

**16.45.8 — Closed in the follow-up pass, once asked to act on this rather than just report it.**
Given engineering authority over how to sequence and close the findings above, six landed — verified
by `se editor --no-run` and, where behavior changed, `se test` (full suite: 1366/1368, the same two
pre-existing unrelated failures throughout this pass):

- Trigger volumes and continuous collision (16.45.1's third bullet) — `ColliderParams::trigger`/
  `::continuous_collision`, wired through `collider_from_params`, an Inspector "Behaviour" section,
  and scene-file round-tripping. `tests/integration/test_physics_authoring.cpp`'s
  `ATriggerVolumeReportsOverlapButNeverStopsTheBody` proves both halves at once.
- `park_sleeping_joints` (16.45.1's first bullet) — a "Settings" checkbox on the Physics panel,
  staged on `EditorContext` and pushed into `ISimulation::set_park_sleeping_joints` once a frame.
- The statistics panel gap (16.45.5) — `joints`/`elements`/`beams` and `soft_body_ms` are now
  drawn; no solver code changed, since both were already computed correctly every tick.
- The cooking Advanced section and the per-asset override UI (16.45.3) — eleven previously
  UI-less `CookingParameters`/`CookingThresholds` fields, and a "Cooking Override..." modal
  reachable from the Project panel wired to `ImportProfileLibrary::set_override`/the new
  `get_override` (raw-record read-back, distinct from `resolve`'s folded view).
- Cook-bake profile persistence (16.45.3/16.45.4's shared finding) — `CookBakeState::
  save_profiles`/`load_profiles` against `<project_root>/cooking_profile.json`; no project-scoped
  settings mechanism existed anywhere to reuse, checked before building one.
- The dead `DerivedCookingParameters::fidelity` field (16.45.6) — removed, confirmed unused by
  grep first, unlike `Collider::asset`/`applied_scale`, which are real P4 scaffolding and were
  left alone.
- General (non-cloth) Soft Body authoring (16.45.2) — an Inspector section (source-mesh load from a
  `CookBakeState` entry, level, the same five material presets and eight sliders the Bake panel's
  node-beam settings use, thickness, self-collision, the `cosmetic` precision request) and an "Add
  Component" entry, both wired to `IWorldEditor::create_soft_body`/ `set_soft_body_params`, which
  already existed and were called from nowhere in the editor. The ECS-to-solver path itself already
  had coverage (`tests/integration/test_soft_body_service.cpp`); this closed only the reachability
  half.

The second `set_joint_motor`/`set_joint_limits` bullet (16.45.1) was investigated and
**deliberately not implemented** — see the correction inline above; it is not a wiring gap.

The `sycl::half` storage keep/drop call (16.45.4) went to the project owner rather than being
guessed at, and the answer was **drop** — see §16.41's decision addendum. Nothing left open from
this audit's original six findings.

---

## §17 Risks, open questions, and scope

### 17.1 Explicitly out of scope

Fluids of any kind. Runtime Voronoi fracture of arbitrary rigid geometry (soft-body fracture in §9.5
covers the deformable case; rigid shattering is a separate cooked-fragment system, and if it is
wanted it is its own phase). Cross-machine bit-exact determinism (SushiLoop already ruled it out).
Cloth-scale self-collision in P6 — it is scheduled but explicitly allowed to be off by default. A
reduced-coordinate articulation solver as the primary path (§10.5).

### 17.2 Dependencies — the one thing worth buying

Everything in this plan is implementable in-house, and greenfield is the engine's established
preference. The single honest exception is **tetrahedralization quality**: robust tetrahedral
meshing of arbitrary dirty input is a research-grade problem, and §8.3's
voxel-plus-body-centred-cubic approach deliberately trades some element quality for never failing.
If the quality proves insufficient in P6's cantilever test, the options are (a) invest in a
conforming Delaunay stage, or (b) take a dependency through `ss install` (per
`dependency-provisioning-via-ss`). **This is a P4 decision point, and it should be made with the P6
test results in hand, not before.**

### 17.3 Decisions taken

Settled with the project owner on 2026-07-28. Recorded here so a later reader knows these were
chosen,
not assumed.

1. **Vehicle structure: the hybrid** — a rigid chassis core with a deformable node-beam or FEM shell
   (§11.2). The pure node-beam path stays reachable per asset. The explicit brief is *"do not repeat
   BeamNG's mistakes"*, which is why §11.2 now carries the limitation-by-limitation table rather
   than just a preference.
2. **Cosmetic precision: `float`, and `sycl::half` for storage where it pays** (§6.5). Non-gameplay
   soft bodies and cloth are outside the deterministic island and may use a narrower column. The one
   engineering line drawn on top of the owner's answer: **half precision stores, it does not
   compute** — a neo-Hookean projection evaluated in 11 significant bits is not a trade-off, it is a
   wrong answer.
3. **Phase order unchanged: P3 (joints and assemblies) before P4 (cooking).** The car-door scenario
   lands first.
4. **The simulation runs on SushiRuntime**, and considerably more of it than today — see §6.6 for
   what runs on the device and what stays on the host, and §18 for the seams the design depends on.
   Four of the seven asks in §18 have since been built in the runtime, including the two that shaped
   this design most: a device-driven iteration count and fixed-order reductions.

### 17.4 Remaining open questions

1. **The core-to-shell seam (§11.2).** A hybrid vehicle's rigid core and deformable shell meet at
   the §10.3 attachment constraint, and that seam is where visual artifacts will appear first — a
   panel that dents perfectly but whose mount looks rigid. Whether the fix is a graded stiffness
   zone around the attachment or a wider attachment neighbourhood is a P7 question that needs to be
   *seen* before it is answered.
2. **Tetrahedralization quality (§17.2).** A P4 decision point to be made with P6's cantilever test
   results in hand.
3. **Half-precision payoff (§6.5).** Whether half-precision *storage* actually pays after the widen
   cost is a measurement, not a prediction; it is scheduled in P8 and may be dropped if it does not.
   The storage path and its measurement harness exist (§16.41, `examples/soft_body_half_storage_
   budget.cpp`), and the harness has now run (§16.41's addendum): +0.7% mean on the host reference
   solver's conversion cost, a small loss. What remains open is the actual question — the
   device-buffer bandwidth case, which needs FEM as a device graph kind before it can be measured at
   all, not another build.

### 17.5 The largest technical risks

| Risk | Mitigation |
|---|---|
| **The runtime-side half is closed; the engine-side layout it assumed was never built, and this row previously claimed otherwise.** §18 R3 (sub-range cross-region ordering) is genuinely built and `Execution::DynamicGraph`/`Region` now reaches the engine (§16.39) — so two islands writing disjoint slices of one body column really would gain no edge from the runtime. But the sentence that used to close this row — "an island must be a set of index ranges the solver can name with `Buffer::region({offset, count})`, which the incremental recolouring in §6.4 already produces" — does not describe the actual code, checked directly against it in the pass that produced §16.42 item 2: `IncrementalColoring`/`ConstraintStore`/`ContactStore` (`engine/domain/physics/include/SushiEngine/physics/solver/incremental_coloring.hpp`, `engine/domain/physics/include/SushiEngine/physics/solver/constraint_store.hpp`, `engine/domain/physics/include/SushiEngine/physics/solver/contact_store.hpp`) place constraints into flat, scene-wide, *colour*-indexed bands with no island concept anywhere in any of the three files. `IslandBuilder`/`IslandSet` (`engine/domain/physics/include/SushiEngine/physics/scene/islands.hpp`) is a separate, host-only partition over body slot indices, built *after* `RuntimeGraphBuilder::step()` runs (`engine/world/simulation/include/SushiEngine/simulation/physics_simulation.hpp`'s `update_islands()` is called after `solver_->step()`, not before), and used today only to decide next tick's sleep state — it names no `Buffer::region`, is read by nothing in `physics/solver/`, and there is no test connecting it to the solver at all (`tests/unit/test_islands.cpp` tests it in isolation). So the layout requirement this row said was "already produced" is unbuilt, and using it to shape *this* tick's solve raises a question the recolouring code does not answer either: islands are only known once this tick's contacts exist, which is after the solve they would need to shape runs. **Not closed — reopened as §16.42 item 2, now with the actual blocker recorded instead of an assumed one.** | Nothing to mitigate on the runtime side. On the engine side: either accept a one-tick-stale island partition (shape tick *N*'s solve from tick *N-1*'s islands, same staleness sleeping already tolerates) or restructure the tick to build islands from the *previous* tick's resolved contacts before this tick's solve runs — a sequencing decision for whoever picks up §16.42 item 2, not a fact this document should keep asserting was already settled. |
| ~~Incremental recolouring diverges from a full recolour and breaks determinism (§6.4).~~ **Closed, with the claim restated.** | Four tests over a randomized add/remove sequence (§16.10). Equality with a full recolour is *not* asserted, because greedy over an insertion order is not greedy over a final set and equality would mean the build order left no trace. What is asserted is what determinism needs: the colouring is valid, it is a function of the sequence rather than of the container or the worker count, it stays inside greedy's degree bound and within one colour of a rebuild's depth, and a removal releases its colour. The scheduled-full-recolour fallback is not needed. |
| Tetrahedralization quality is too poor for a correct finite-element solve (§17.2). | The cooker reports worst element quality and fails loudly at a threshold; the decision point is scheduled with test evidence. |
| Device-resident collision (P8) cannot be made deterministic at acceptable cost (§12.2). | The host implementation stays behind the same seam and remains the reference. Determinism wins over throughput; that is the standing rule. |
| The runtime's API moves under us — it is explicitly unstable and this plan leans on `Dynamic`, `DynamicGraph`, residency, and sub-region tracking. | One adapter names `SushiRuntime::` (§6.6), restoring the runtime's own L12 decision. A move costs one file plus a conformance run, not a rewrite. |
| The fixed-order reduction (now the runtime's, §18 R2) turns out to be the bottleneck on the device. Its segmented form gives one work-item per segment, so a body with ten thousand contacts and a body with one cost the same launch slot. | It is used on exactly two paths (contact impulse and soft-body vertex accumulation), both of which have a colouring-based alternative that avoids accumulation entirely at some convergence cost. Measure before optimizing; the fallback exists. The imbalance is a layout problem on our side, not a runtime defect — the guarantee is what forbids a work-stealing split. |
| A runtime bug lands in the physics tick (the runtime's own `TODO.md` still carries an open thread-local-magazine teardown item, #29). | The physics scene owns its allocations through one adapter with a single lifetime, so a teardown-ordering issue has one place to be fixed, and the engine pins a known-good runtime rather than tracking its tip. |
| Fracture plus mutable topology destabilizes the solve. | Per-tick budgets, minimum fragment sizes, and scene caps, all state-derived and all reported. |
| Scope. This is a large plan. | Every phase is independently shippable and every phase ends with a working, tested engine. There is no phase whose failure leaves the tree worse than it started. |

---

## §18 What SushiRuntime must gain

Per `docs/CONTRIBUTING.md`, *"a change that needs new runtime behavior belongs in the runtime,
behind its public API, not bolted onto the engine."* This section is the engine-side record of what
this plan needs from below; the runtime-side engineering request, with `file:line` evidence, lives
in `sushiruntime/docs/design/PHYSICS_SUBSTRATE_REQUIREMENTS.md`.

**Four of the eight were recorded here as built** on the runtime's `feature/physics-substrate-seams`
branch. Each row states what was asked, what landed, and — for those still open — what the physics
does **without** it, because no phase may be blocked on another repo.

> **Correction, 2026-07-29 — that branch is not reachable, and this section was believed until it
> was checked.** `C:/Projects/sushiruntime` has `main` and one unrelated fix branch; there is no
> `feature/physics-substrate-seams`, on the working tree or on `origin`. None of
> `sized_from_device`, `add_reduce`, `add_segmented_reduce` or `add_untracked` appears anywhere in
> the runtime's headers. So **R1, R2, R3 and R5 must be read as *open*** until that branch turns up
> or is rebuilt, and this table's "Built" is a record of what was agreed, not of what a build can
> link against.
>
> The cost of believing it was real and specific.
> `engine/domain/physics/include/SushiEngine/physics/solver/runtime_graph_builder.hpp` was written
> calling `Graph::add_reduce(..., API::Maximum<T>{}, ...)`, which compiles for as long as nobody
> *instantiates* the template — a dependent call is not looked up until instantiation, so header
> syntax checks and every test that only names the type stayed green. It came down the moment the
> live tick moved onto the solver and `engine/world/simulation/source/runtime_simulation.cpp`
> finally built one. **The lesson is a build rule, not a scolding:** an engine-side claim about a
> runtime API is only tested by a translation unit that instantiates it, so at least one must exist
> for every runtime seam the physics leans on.
>
> R2 is closed engine-side in the meantime — see §16.7 — and closed in a way that is one deletion
> away from using the runtime primitive if it does arrive.
>
> **Second correction, 2026-07-30 — the correction above was about a path, not about the
> runtime.** This project is worked on from two machines and the sibling checkout is not at the
> same place on both: `C:/Projects/sushiruntime` on the one that correction was written from,
> `D:/Projects/sushiruntime` here. On *this* checkout `sized_from_device`, `add_reduce`,
> `add_segmented_reduce`, `add_untracked`, `DynamicGraph::region` and boundary pins carrying
> `Core::ResourceRegion` byte intervals are all present in the runtime's headers. So **R1, R2,
> R3 and R5 are built, and the four "Built" rows below can be linked against** — from here.
>
> Both corrections are kept rather than one replacing the other, because together they are the
> actual lesson and neither is alone. The first one's build rule stands unchanged and is what
> caught this: *an engine-side claim about a runtime API is only tested by a translation unit
> that instantiates it.* What the second adds is that a claim about a repository is only tested
> by the machine you are standing on, and "I checked and it is not there" is a statement with a
> hostname in it. The hand-built reduction the first correction produced has now been deleted in
> favour of `Graph::add_reduce` (§16.10) — which is the only way to find out which correction
> was right.

| # | Ask | Status | What the physics does |
|---|---|---|---|
| **R1** | **Device-driven iteration count.** | **Built** — `API::sized_from_device(counter, index)`. | Uses it. The tick is one composition sized to the live count. See below: the mechanism is not the one this plan predicted. |
| **R2** | **Fixed-order deterministic reduction primitives** (the runtime's WP-4 item 1). | **Built** — `Graph::add_reduce` / `add_segmented_reduce`, with `Sum`/`Minimum`/`Maximum`. | Uses them. §12.2's accumulation is no longer a physics-layer deliverable; the segmented form is exactly the per-body / per-vertex shape §12.2 needs. |
| **R3** | **Sub-range cross-region ordering** in `DynamicGraph`. | **Built** — boundary pins carry byte intervals and every hazard test is an interval overlap. | Uses it. One region per island now works as intended; the `when()`-gated fallback in §17.5 is retired. |
| **R5** | **Dependency tracking on every `add()` overload.** | **Built, as Option A** — every launch shape has a tracked overload (three were missing), and the dependency-blind ones are now spelled `add_untracked`. | Uses it. The review checklist becomes a grep for `add_untracked` in `physics/`, which is a real guarantee rather than an eyeball one. |
| **R4** | **Asynchronous run** — `run()` blocks, so a tick cannot overlap with the render or audio extract. | **Built** (found 2026-08-03, checked directly against `engine/domain/audio/include/SushiEngine/audio/dsp/graph.hpp` and `run_handle.hpp` rather than trusted from the runtime's own request doc, per the standing rule this section's correction already established) — `Graph::run_async()`/`DynamicGraph::run_async()` return a move-only `RunHandle<Graph>` that splits `run()`'s submit and complete halves, with the caller's own work running in between. | **Not used.** `RuntimeGraphBuilder::step()` still calls the blocking `run()` (`engine/domain/physics/include/SushiEngine/physics/solver/runtime_graph_builder.hpp`); nothing in `engine/world/simulation/include/SushiEngine/simulation/physics_simulation.hpp` overlaps the tick with render or audio extract yet. Consuming this is a scheduling decision above the solver — what the simulation thread does with the returned `RunHandle` while the device runs — not a one-file change, so it stays a named opportunity rather than something this pass wired in blind. |
| **R6** | **A late-bound base offset** alongside `sized()`, so a colour slice whose offset shifts between ticks needs no indirection buffer. | **Built** (found 2026-08-03, same direct check) — `API::based_at(provider)` and `API::based_at_device(slot, index)`, mirroring `sized()`/`sized_from_device()`; resolved and validated together with the count at dispatch. `Execution::Detail::to_dynamic` (`engine/foundation/execution/include/SushiEngine/execution/backend/runtime_backend.hpp:131-141`) already forwards `NodeDescriptor::base` to `and_based_at`/`and_based_at_device` when a node sets it. | **Not used.** Every banded loop in `RuntimeGraphBuilder::build_graph()` still bakes its band's `base` into the kernel lambda's capture instead of binding `node.base` (confirmed across all nine banded loops — none set `.base`), so this is unused capacity rather than a missing primitive. It is exactly what §16.42 item 2's per-island regions would need to shift a band's base per region without an indirection buffer, once that item's design question is settled. |
| **R7** | Closing the **`ThreadLocalMagazine` teardown** correctness item (#29). | **Open** | Nothing the engine can do. It is a long-running-process risk and sits on the runtime's own v1 list. |
| **R8** | **A node label on the ordinary `add()` overloads.** `RunReport::NodeTiming` already carries a name and the engine already reads the report, but only `add_offload` lets a caller set one — so every physics node arrives as `unnamed_task` and per-stage device timing cannot be attributed. | **Built** (raised 2026-07-30, closed 2026-08-02, for P8-A — see §16.37) | The `Dynamic` per-element `add()` overloads and `add_host()` take a trailing, defaulted `const char* name`, threaded through to `Node::metadata`. `Execution::Graph::add_parallel`/`add_host` (`engine/foundation/execution/include/SushiEngine/execution/backend/runtime_backend.hpp`) now pass `NodeDescriptor::name` through instead of dropping it, so every already-named physics node (`predict`, `xpbd_project`/`pgs_project`, `update_velocity`, `motion_measure`) reports under its real name. **The per-name breakdown deferred past P8-A is also closed now** (§16.40): `physics_node_timings_from_report` groups the report by name into `PhysicsStatistics.timings.node_timings`, and the Physics panel draws one row per kind that actually dispatched. |
| **R9** | **A multi-colour (or persistent-kernel) dispatch node** — one `add()` call covering several ordered sub-ranges, so a kind's colour sweep costs one barrier crossing per substep rather than one per colour per substep. | **Open** (raised 2026-08-02, for P8 — see §16.37, §16.42) | The composed graph pays a full scheduler round trip — dependency check, event submission, event poll — per (kind, colour, substep) triple, and §16.37 measured that round trip, not the kernel it guards, as 96–98% of the tick on this backend. §16.38 removed the triples a zero-capacity kind pays for; it did not reduce the count for a kind that is actually in use, which needs this. Until it exists the physics does not approximate it: a kernel that looped over colours internally would either serialize what colouring exists to parallelize, or duplicate the scheduler's own ordering guarantee inside one node — the "second scheduler hiding inside an adapter" `engine/foundation/execution/include/SushiEngine/execution/backend/runtime_backend.hpp` is written to avoid. |

#### R1: the answer was better than any of the three options

This plan reasoned that `Dynamic::sized()` is a host-side provider polled on the driver thread at
the step boundary, so within one `run()` it cannot carry a count a kernel produced in that same run
— and concluded that only three shapes existed: two runs with a readback between them, dispatch at
capacity with a per-lane early-out, or indirect dispatch. The plan committed to the second and kept
the `RuntimeGraphBuilder` seam thin so it could move to the third.

**That reasoning was wrong about *when* the count is read**, and the correction is worth recording
because it removes a compromise the whole §6.6 design was shaped around. A node's `sycl::handler`
closure is not evaluated when the graph is built or when the step boundary polls the host providers
— it runs **on a worker thread at dispatch**, and a node is only dispatched once `EventPolling` has
seen every predecessor's event complete and called `wait()` on it. At that point the predecessor
kernel's USM writes are visible, so a count it wrote can simply be read there.

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
  the runtime itself, so `RuntimeGraphBuilder` does not have to remember to name it in `Reads(...)`
  and cannot get it wrong.
- **Overflow fails the run; it does not clamp.** A live count above the compiled capacity throws
  with both numbers in the message. That is the correct behaviour for this system — a clamped
  contact count is a silently dropped contact — but it means **capacity planning is now a hard
  failure mode, not a quality-of-simulation one**, and §6.4's fixed-capacity buffers must be sized
  against the escalation path in §7, not against a typical tick. The engine's own budget clamp has
  to run *before* the count reaches the runtime.

One constraint the design must honour: the counter buffer must be host-addressable
(`Residency::Shared`). Everything else in the scene stays device-resident; the counters are four
bytes each and are the only exception.

#### What this changes elsewhere in this plan

- **§6.6** — the "dispatch at capacity, early-out per lane" shape is no longer the plan. The
  `RuntimeGraphBuilder` seam still exists for API churn, but not as a switch waiting for indirect
  dispatch.
- **§12.2** — the fixed-order reduction moves from "the physics layer builds it" to "the physics
  layer calls it". It leaves P0 as a deliverable and stays as a *conformance requirement*: the
  byte-equality test with the worker count varied (§15.5) still has to pass, and now it tests the
  runtime's primitive rather than ours.
- **§17.5** — the island-per-region serialization risk is closed. Option (c), `when()`-gated
  islands,
  is no longer the P2 fallback.
- **§15.6** — "no physics node is added through a raw-kernel overload" stops being a review
  checklist
  and becomes a mechanical check.

None of this changes a phase boundary or a deliverable's scope. It removes work from P0 and removes
two risks from §17.5.
