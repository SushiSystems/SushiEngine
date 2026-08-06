# Vehicles

How to get a car from a mesh to something you can drive, and what each number in the way
actually means.

A SushiEngine vehicle is a **hybrid**: a rigid core carrying most of the mass and the
inertia, with a deformable node-beam shell hung off it. That is not a compromise between
two techniques — it is the arrangement that gives you a chassis that handles like a chassis
and panels that dent like panels, and it is why the shell can lose a door without the car
losing its handling.

The design record is `docs/design/physics_system.md` §11. This page is the path through it.

---

## The five pieces

| Piece | What it is | Where it comes from |
|---|---|---|
| **Structure** | Nodes, beams, collision surface, rigid core, attachments | `.sushinodebeam` cook |
| **Corners** | Suspension travel, spring rate, damping, steering, drive | Vehicle window |
| **Tyres** | Friction, stiffnesses, load sensitivity | Vehicle window |
| **Drivetrain** | Engine curve, clutch, gearbox, differential | Vehicle window |
| **Aerodynamics** | Frontal area, drag, downforce, centre of pressure | Vehicle window |

Only the first is cooked. The other four are numbers you type, and they are numbers rather
than a file because they are the ones you change forty times in an afternoon.

---

## 1. Cook the structure

Open **Window ▸ Analysis ▸ Bake**, tick **Cook node beam**, and set the node-beam settings
beside it. `NodeBeamCooker` takes **a mesh, a fidelity dial, a `SoftBodyMaterial`, and a core
mass fraction**, and produces the node cloud, the beam network, the surface and the skinning.
The Vehicle window names an already-cooked structure; it does not cook one.

What it does with them:

- The **tetrahedralizer's lattice** becomes the node cloud. Each node gets a position, a
  share of the shell's mass, a collision radius and a drag area.
- The **lattice's edges** become the beams. An edge along a grid axis is one cell long, a
  face diagonal is √2 and a body diagonal √3; anything longer than the structural ratio is
  **bracing** rather than **structural**. The threshold sits *between* those values so a
  conforming pass that moved a boundary vertex does not reclassify the edge it moved.
- Each beam's **compliance, damping, deform force, break force and plastic creep** are
  derived from the `SoftBodyMaterial` and the beam's cross-section. You do not type them per
  beam; you type the material once and the cook does the arithmetic.
- **`core_mass_fraction`** splits the mass. At zero the whole mass is in the node cloud and
  you have a pure node-beam vehicle; at nine tenths the chassis is rigid and the shell is a
  skin over it. It is a dial rather than a switch on purpose — walk it and watch the
  handling change.
- Render vertices are **bound to nearby nodes**, up to four each, as a displacement in the
  frame those nodes' arrangement implies. A vertex with no node within the skin radius is
  reported *unskinned* rather than tethered to a node on the other side of the car.
- **Attachments** bolt shell nodes to the core. Turning them off gives you a node cloud that
  falls off its own chassis, which is what a pure node-beam cook wants and a hybrid one
  never does.

### The material is the car's strength

`SoftBodyMaterial` defaults to a soft, unbreakable solid — chosen so that a forgotten
material *looks* wrong rather than looking right and failing later. Set it deliberately:
the deform force is where a panel starts taking a permanent dent, and the break force is
where a beam gives up entirely.

---

## 2. Put it in a scene

1. Select an entity (or make an empty one).
2. **Add Component ▸ Vehicle**.
3. Open **Window ▸ Analysis ▸ Vehicle**, go to the **Scene** tab.
4. Type the `.sushinodebeam` path.

The component stores a **path**, not bytes. Every other cooked asset at this boundary
crosses as bytes because whoever cooked it owns them; a vehicle is placed by you, in a
scene file, that has to survive being reopened on another machine — and a path is the only
thing that survives that.

If the path is empty or does not load, the panel says which. Both look like a stationary
car from outside, so they are distinguished rather than collapsed.

Once it is live, the entity's Transform follows the vehicle's **rigid core** — not a node.
A node's position is a panel's position; only the core's is the car's.

The other four tabs edit the window's own copy of the setup. **Apply Setup**, on the Scene
tab, is what puts those numbers on the car — and it rebuilds it, because a vehicle is four
hundred bodies placed relative to a cooked structure and there is no patching one.

---

## 3. Author the corners

A corner is a **slider with a spring-damper drive**, plus a hinge for the axle. Steering is
that slider's frame turned about its own axis.

- **Spring rate** — the Vehicle window shows `m·g/k` beside it, the sag it produces at that
  corner's share of the mass. That number, not the rate, is what tells you whether the car
  sits right; it turns amber when the sag exceeds the bump travel.
- **Damping** — a rate in inverse seconds, so a strut damped at 8 s⁻¹ damps the same amount
  per second whatever the substep count. Without it the spring rings forever.
- **`axle`** — a vehicle-wide convention, the *same* direction on both sides. Pointing the
  two wheels of an axle outboard makes them turn in opposite senses rolling forward, which
  hands the differential a mean of zero and the chassis two cancelling reactions. This was a
  real bug once; the doc comment on the field is the fix.
- **`steered` / `driven`** — which corners the steering angle reaches and which the
  drivetrain feeds.
- **`material_index`** — which entry of `VehicleAsset::materials` the wheel and its carrier
  collide as. **Point it at a frictionless entry whenever that corner's tyre model is on.**
  The solver's own Coulomb friction runs inside the substep loop on the same contact the tyre
  model reads, so a gripping wheel gets both, and the car ends up with grip nobody authored
  and no single wrong number to find. `VehicleAsset::node_material_index` and
  `::core_material_index` do the same for the shell and the core; an index naming no entry
  contacts as the default solid.

---

## 4. Author the drivetrain

§11.4's chain is **one-dimensional**, with the crankshaft as its only free coordinate. It is
not built out of constraints, and that is deliberate: §10.5's first escape hatch says to
solve a stiff 1-D sub-chain independently rather than through the 3-D solver.

- **Engine curve** — torque against crankshaft speed, in rad/s and N·m. Four or five points
  is plenty. Below the idle band a governor holds the engine up; past the limiter the curve
  is cut, and the overshoot you see is one tick of peak torque on the crank's inertia, which
  is real limiter behaviour rather than a bug.
- **Gearbox ratios** — an *index* selects one, and reverse is a negative ratio like any
  other rather than a separate sign. The window shows the road speed each gear reaches at
  the limiter, which is the number that catches a decimal point in the wrong place.
- **Clutch capacity** — the torque the plate can carry. It is *solved and clamped*, not
  sprung: the torque that would equalise both sides at the end of the step, limited to what
  the plate can hold. A properly-sized clutch locks to within a millionth of a rad/s.
- **Differential lock torque** — one number, not three kinds. The lock torques are balanced
  to sum to zero, so a differential can never be a source of torque.

---

## 5. Drive it

Select the vehicle and use the arrow keys.

| Key | Control |
|---|---|
| **Up** | Throttle |
| **Down** | Brake |
| **Left / Right** | Steering |
| **Left Shift** | Clutch (held = disengaged) |
| **Page Up / Page Down** | Gear up / down |

Arrow keys rather than WASD because W, E and R are the gizmo keys, and a driving binding
that stole them would break the tool keys the moment you selected a car. They are
rebindable from Preferences like every other binding.

The controls are **ramped, not switched**: a key is a bit and a throttle is not. Each moves
toward its target at a rate, and the rates differ because the mechanisms do — a throttle
cable is quick, a steering rack slower, and both return to centre faster than they leave
it. The ramp lives in the editor's input layer, not in the physics: a wheel-and-pedal set
would deliver the same controls with no ramp at all, and the car must not be able to tell
which it is talking to.

The Vehicle window's **Scene** tab has the same controls as sliders, plus the live readout —
engine speed, engine torque, clutch torque, whether the clutch is slipping, and how many
beams have broken and parts come off.

---

## 6. Watch it

- The **shell is drawn** as the surface it collides as, straight off the live nodes with no
  cache in between. A dented panel is dented on screen in the tick it dented, and the
  drawing cannot disagree with the collision because they are the same triangles.
- The Vehicle window's **Shell** view is a side elevation of the node cloud — the view in
  which a sagging suspension and a caved panel are both visible at once.
- **Window ▸ Analysis ▸ Physics ▸ Debug Draw** turns on contacts, broadphase bounds, island
  colouring and sleeping markers. Contact normals scale with the impulse, so a crash reads
  longer than a scrape.

---

## What is still missing

Stated rather than discovered:

- The authored **setup is not serialized**. The scene file stores the structure path and
  nothing else; a reloaded vehicle comes back at the default corners, tyres, drivetrain and
  aerodynamics until `VehicleAsset` has a serializer of its own. The Vehicle window says so
  rather than the file losing it silently.
- The **material table has no editor surface**. The scene resolves every vehicle body's
  `material_index` against `VehicleAsset::materials` and applies the result to its contacts,
  but the Vehicle window draws neither the table nor the indices; authoring one means calling
  `IWorldEditor::set_vehicle_parameters` from code.
- The cooked **render skinning is not drawn**. What you see is the collision surface, which
  is the shape the physics owns end to end; drawing the pretty mesh needs that mesh's own
  index buffer, which lives in the visual asset rather than in the `.sushinodebeam`.
- A car collides as **a sphere per shell node and per wheel**, at the radii the asset authored,
  rather than as the surface triangles it is drawn from. Its own bodies are on a collision layer
  that excludes itself, which is what stops a wheel contacting the hub it is bolted to — so two
  cars do not collide with each other either.
- The Structure tab names the node-beam and core-collision assets **by numeric identifier**,
  not by browsing. The panel is the asset layer's front end rather than a second resolver,
  so it hands the identifier on without dereferencing it.

---

## The numbers, if you want to check the work

From §16.27 and §16.28's measurements:

- The clutch locks to **1e-6 rad/s**; the differential's outputs sum to the shaft torque to
  **1e-9 N·m**.
- A parked wheel reads **196.2000 N** of tyre load against 196.2000 N of weight; a locked
  wheel reads exactly **−μN**.
- The §13.1 vehicle — 400 nodes, 2 000 beams, four wheels, a powertrain — runs at
  **0.460 ms/tick** on the host reference solver. That is *printed, not asserted*: the 2 ms
  target is quoted for a desktop GPU through SushiRuntime, and a suite that fails on the
  hardware it happens to run on is a suite people stop reading.
