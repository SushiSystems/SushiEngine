# Character controller (P9-B) — design

**Date:** 2026-08-07
**Phase:** `docs/design/physics_system.md` §16, row P9, second of five sub-projects
**Depends on:** P9-A (kinematic bodies), landed 2026-08-07
**Status:** approved

## 1 What this builds

A capsule the game moves by a displacement each tick, resolved against the world: it slides along
what it hits, climbs a stair, is stopped by a slope too steep to walk, and reports whether it is
standing on something. It is a kinematic rigid body, so it pushes the crates it walks into and
rides the platforms it stands on — both of which P9-A already gives for nothing.

Nothing like it exists. Every `character` in `applications/editor/source/` is animation preview.

## 2 Two decisions, taken

**The controller is a geometry solver, not a character.** `move_character` takes the full
displacement the game already computed — including whatever the game did about gravity and jumping
— and answers what the world allows. It holds no fall speed, no jump state, no coyote time.

The reason is not purity. This engine samples gravity per body through `GravitySampler`, and
`docs/design/solar_system_overhaul.md` is working toward a walkable planetary surface. A controller
that owned "falling" would have to own a gravity direction, and on a sphere that is a function of
position. Keeping it out means the controller never has to know.

**`up` is a per-call parameter, not a constant.** For the same reason. Unity, Unreal and every
tutorial controller bake world +Y into the slope test, the step-up and the ground probe. On a flat
scene that is correct and on a sphere it is silently wrong: at the equator the local up is the
pole's sideways. Passing it costs one argument now and saves re-reading every slope-related line
later.

## 3 The layers

Four, each ignorant of the one above it.

| Layer | File | Owns |
|---|---|---|
| Algorithm | `engine/domain/physics/include/SushiEngine/physics/character/character_mover.hpp` | Collide-and-slide, step-up, ground probe, the slope rule |
| Query | `engine/world/simulation/include/SushiEngine/simulation/physics_services.hpp` | `sweep_capsule`, the one query the boundary lacks |
| Scene | `engine/world/simulation/include/SushiEngine/simulation/physics_simulation.hpp` | `ICharacterService`: binds the mover to the sweep and writes through `move_rigid_body` |
| ECS + editor | `engine/world/simulation/include/SushiEngine/simulation/simulation.hpp` | `CharacterParameters` and its inspector section |

The algorithm names **no** broadphase, no scene, no ECS and no solver. It takes a sweep as a
callable:

```cpp
// sweep(const CapsuleCollider<T>& posed, const Vector3T<T>& direction, T distance) -> RayHit<T>
```

which is what makes it unit-testable against a hand-written lambda — no device, no world, no
`create_physics_simulation`. That is the same dependency inversion `physics_extract.hpp` uses to
make the extract testable without a scene, and §3.3's stated seam rule.

## 4 The algorithm

```
move_character(sweep, capsule, displacement, up, settings):
    split displacement into `along up` and `across up`
    1. slide the horizontal part          — sweep, advance, project the leftover onto the
                                            hit plane, repeat up to max_slides
    2. if still blocked and step_height>0 — sweep up, forward, then down; accept the
                                            landing only if its normal is walkable
    3. slide the vertical part            — the same slide, so a descent follows a ramp
    4. probe for ground                   — one sweep along -up, out to ground_snap
```

**The slope rule is one comparison, used three times.** `dot(normal, up) >= max_slope_cosine`
decides whether a surface is walkable, and that single test is what makes a steep face a wall
rather than a hill: a sliding projection along an unwalkable face has its up-component removed
afterwards, so the character cannot climb a cliff by pressing into it. The same test rejects a
step-up whose landing is too steep, and the same test decides `grounded`.

**Skin width is subtracted from every advance**, never added to the sweep result, so the capsule
keeps a constant clearance and never starts a tick already touching. A sweep that reports a hit at
distance `d` advances `max(0, d - skin_width)`.

## 5 The result

```cpp
struct CharacterMoveResult {
    Vector3T<T> position;       // where the capsule ended up
    Vector3T<T> remaining;      // displacement that could not be spent
    Vector3T<T> ground_normal;  // the surface under it, or `up` when airborne
    bool grounded;
    bool stepped;               // a step-up was taken this call
    int sweeps;                 // what it cost, for the profiler and the tests
};
```

`remaining` is not diagnostic padding — it is how a caller knows it walked into a wall, and how a
future controller-driven animation picks a stop pose.

`sweeps` exists because the loop is bounded by `max_slides` and a caller that never sees the count
cannot tell a cheap tick from one that spent its whole budget.

## 6 The scene binding

`ICharacterService::move_character(EntityId, displacement, up)` resolves the move, then writes the
result through `move_rigid_body` — so the character's body is kinematic, dynamics are pushed by it,
and a character standing on a kinematic platform is carried by exactly the friction path P9-A
tested. Nothing new carries it.

The capsule comes from the entity's `CharacterParameters`, not from its collider: a character's
collider may be authored for hit detection at a different size, and the controller's shape is the
controller's.

## 7 Acceptance

Unit, against hand-written sweeps:

1. Unobstructed motion is spent in full.
2. A wall is slid along, not stopped dead — and never climbed.
3. A stair below `step_height` is climbed; one above it is not.
4. A slope steeper than `max_slope_degrees` is a wall, not a hill.
5. An overhang low enough to block the forward leg of a step-up rejects the step.
6. With `up` set to a non-Y axis, every one of the above still holds. This is the test that fails
   if any slope-related line hardcodes world +Y.

Integration, against the live physics:

7. A character walks up a real stair in a real scene.
8. A wall stops it, and `remaining` says so.
9. It stops at a dynamic crate rather than walking through it.
10. It crosses flat ground at all — which fails if the sweep does not exclude its own body.

**Clauses 9 and 10 as first written were wrong, corrected 2026-08-07 during implementation.**
They read "it rides a kinematic platform" and "it pushes a dynamic crate", and neither is what
this design produces. Both bodies of a character-on-platform pair are kinematic, so both have zero
inverse mass and the contact between them resolves to nothing. And the controller's own sweep stops
the capsule a skin width short of a crate, so no overlap exists for the contact projection to
spend. Unity's `CharacterController` and PhysX's `PxController` behave identically and both make
each an explicit opt-in the caller wires up.

Both are now integration tests that pin the *absence*, each carrying an instruction to rewrite
rather than loosen it, so closing either gap fails a test deliberately instead of quietly starting
to pass. They are recorded as follow-ups in `docs/design/remaining_work.md`.

Every one runs on the CPU backend. **P9-B needs no GPU.**

## 8 Out of scope

- **Input, camera and a playable demo scene.** Named and excluded by the scope decision, not
  forgotten. Driving the controller from the keyboard is game code.
- **Crouching, swimming, ladders, moving-platform-relative input.** A controller can grow these;
  none is needed to call the row done.
- **Character-versus-character resolution beyond what the kinematic body already does.** Two
  kinematic capsules do not push each other — both have zero inverse mass — and that is correct
  until something says otherwise.
