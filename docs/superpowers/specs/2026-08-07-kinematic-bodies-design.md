# Kinematic bodies (P9-A) — design

**Date:** 2026-08-07
**Phase:** `docs/design/physics_system.md` §16, row P9, first of five sub-projects
**Status:** approved

## 1 What this builds

A body the game moves and the simulation does not: a lift, a moving platform, a swinging door
driven by an animation clip, and — in P9-B — the capsule a character controller stands in. It is
pushed by nothing and pushes everything.

`Physics::BodyFlags::kinematic` already exists
(`engine/domain/physics/include/SushiEngine/physics/core/body_flags.hpp:59`) and **nothing reads
it**. Nothing writes it either: `physics_extract.hpp` fills a body description's `position`,
`inv_mass`, `inv_inertia` and `drag_coefficient` and never touches the flag word, so every
extracted body reaches the solver as `dynamic_body`. The bit is a promise the tree does not keep,
and an author who finds it and sets `inv_mass = 0` instead gets a static body that pushes nothing.

## 2 What already works, and why the change is small

Three mechanisms this feature needs are built, and were built for other reasons.

**Immovability.** Every projection scales its correction by the body's inverse mass —
`apply_positional_impulse`, `apply_velocity_impulse`, `apply_angular_impulse` and
`generalized_inverse_mass` in
`engine/domain/physics/include/SushiEngine/physics/core/rigid_body.hpp`. A body at `inv_mass == 0`
with a zero `inv_inertia` takes no share of any constraint, contact or joint. Nothing needs a
kinematic case; the arithmetic already gives the answer.

**Pushing.** A kinematic body pushes not by applying a force but by moving. Its pose advances each
substep, the next substep's contact projection sees the overlap, and the depenetration term moves
the dynamic body — the whole of which is code that exists.

**Island isolation.** `IslandBuilder::conducts`
(`engine/domain/physics/include/SushiEngine/physics/scene/islands.hpp:321`) requires
`inv_mass > T(0)`, so a kinematic body does not merge the islands on either side of it. A conveyor
belt cannot hold a whole scene awake through a chain of contacts. This is the behaviour PhysX
implements deliberately, and here it falls out of a rule written for static bodies.

**One solver, not two.** `HostSolver` (`host_solver.hpp:514`) and `RuntimeGraphBuilder`'s device
composition (`runtime_graph_builder.hpp:901`) both call the same free `predict()`. A change to that
function reaches the host and device paths at once, and `tests/integration/test_solver_conformance.cpp`
holds them to each other without a new mechanism.

## 3 The gap, stated precisely

`predict()` (`rigid_body.hpp:451`) guards acceleration accumulation and position integration with
one condition (`:463`):

```cpp
if (body.inv_mass > T(0))
{
    body.velocity = body.velocity + (linear_acceleration + body.external_acceleration) * h;
    /* drag */
    body.position = body.position + body.velocity * h;
}
```

That is correct for the two body kinds that exist. A kinematic body falls between them: it must
**skip** the acceleration and **perform** the integration. There is no branch for that because no
body kind wanted one.

## 4 The command model

**Target pose, velocity derived.** `IRigidBodyService::move_rigid_body(id, position, orientation)`
sets where the body should be at the end of this tick. The scene derives

    v = (target_position − current_position) / dt
    ω = the axis-angle of (target_orientation · current_orientation⁻¹) / dt

and writes them onto the body. `predict()` then integrates that velocity across the substeps.

This is PhysX's `setKinematicTarget` and Unity's `Rigidbody.MovePosition`. It was chosen over a
velocity-setting API for three reasons: an author driving a platform from an animation clip has a
pose and would otherwise have to differentiate it; a derived velocity cannot accumulate positional
drift, because each tick re-derives from the true target; and the derived velocity is exactly the
number a contact's friction term reads, so a crate on a moving platform is carried without anything
being written to carry it.

`move_rigid_body` is the sibling of `set_rigid_pose` (`physics_simulation.hpp:285`), which
teleports and zeroes velocity — "the body is placed, not thrown", in that function's own words.
Both survive: placing and moving are different acts.

## 5 Behaviour

| | `static_body` | **`kinematic`** | dynamic |
|---|---|---|---|
| Integrated | no | **yes, from the command** | yes, from forces |
| Takes gravity, drag, external field | — | **no** | yes |
| Moved by a constraint | no | **no** (`inv_mass == 0`) | yes |
| Moves others | no | **yes** | yes |
| Conducts an island | no | **no** | yes |
| Can sleep | — | **yes; a command wakes it** | yes |

`is_simulated()` (`body_flags.hpp:102`) returns **true** for a kinematic body and its definition
does not change. The question that function asks is "is this body integrated and projected this
tick", and for a kinematic body both are yes — only the input to the integration differs. Twenty
call sites ask it; narrowing its meaning would drop kinematic bodies out of the contact list, the
statistics and the wake logic in one move.

`update_velocity()` (`rigid_body.hpp:505`) **skips** a kinematic body. Re-deriving `v` from the
pose delta would return the commanded velocity for the linear term but pass the angular term
through a small-angle logarithmic map, losing a little of the command every substep. Skipping keeps
the command bit-exact, which P9-D's byte-equality test will depend on.

## 6 The sleeping-box problem

Because a kinematic body does not conduct, a crate resting on a platform forms its own island,
settles, and sleeps. When the platform then moves, the crate is not integrated and not projected —
it hangs in the air while the platform leaves.

`move_rigid_body` fixes this where the motion is commanded: when the target differs from the
current pose, it walks the scene's current contact records and wakes every body sharing a record
with the moved slot. Those records are one tick old. That is the same staleness sleeping already
tolerates, and `docs/design/physics_system.md` §16.42 already names it as an acceptable trade for
island work.

The wake is bounded by the platform's own contact count, not by scene size.

## 7 What changes, by layer

| Layer | File | Change |
|---|---|---|
| Solver core | `physics/core/rigid_body.hpp` | The kinematic branch in `predict`; the skip in `update_velocity` |
| Solver core | `physics/core/body_flags.hpp` | `is_kinematic()`; `is_simulated()` unchanged |
| Boundary | `simulation/physics_extract.hpp` | Description gains `kinematic`; writes the flag and forces `inv_mass`/`inv_inertia` to zero |
| Service | `simulation/physics_services.hpp` | `move_rigid_body` on `IRigidBodyService` |
| Scene | `simulation/physics_simulation.hpp` | Velocity derivation; the contact wake |
| ECS | `simulation/simulation.hpp` | `PhysicsBodyParameters::kinematic` |
| Scene file | the rigid-body serializer | The field round-trips |
| Editor | the Rigid Body inspector | A checkbox, drawn unlinked first |

The invariant "kinematic implies zero inverse mass" is established **at the boundary**, in the
extract, not defended inside the solver. The solver's job is to integrate what it is given.

## 8 Acceptance

A test scene with a kinematic platform and a dynamic crate:

1. The platform does not fall under gravity.
2. The crate resting on it does not push it down.
3. The platform moving horizontally carries the crate through friction.
4. A crate that has fallen asleep on the platform wakes when the platform is commanded to move.
5. A conformance scene with a kinematic body agrees between the host and device solvers.

Every one of these runs on the CPU backend. **P9-A needs no GPU.**

## 9 Out of scope

- **The character controller.** P9-B, and it depends on this.
- **Continuous collision for kinematic bodies by default.** The `continuous_collision` flag
  (`body_flags.hpp:68`) already exists and a fast platform's author opts in, exactly as any other
  body's author does. Making it implicit would spend the sweep on every door in a scene.
- **Kinematic soft bodies, cloth particles and vehicle nodes.** `node_beam_asset.hpp:121` already
  gives a zero-mass node its own kinematic meaning; unifying the two vocabularies is not this
  sub-project's.
- **A kinematic body's own contact events.** They already flow; nothing here changes them.
