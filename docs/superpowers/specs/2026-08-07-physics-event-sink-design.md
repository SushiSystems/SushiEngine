# Physics event sink and impact response (P9-C) — design

**Date:** 2026-08-07
**Phase:** `docs/design/physics_system.md` §16, row P9, third of five sub-projects
**Depends on:** nothing new — the event stream it fans out already exists
**Status:** approved

## 1 What this builds

Two things, and the second is why the first is worth having.

A **push interface**, `IPhysicsEventSink`, registered on the physics scene with a filter it
declares up front, called once per qualifying event immediately after the tick produces them.

An **`ImpactResponse` component** and the engine's own listener for it, so an author gets an impact
sound and a burst of sparks by ticking a box rather than by writing C++.

## 2 Why the second half is not optional

`ContactEvent` has existed since P1. It carries phase, world point, normal and the total normal
impulse, and it reaches the boundary through `ISimulation::physics_contacts`. **Nothing outside the
test suite reads it.**

Shipping only the sink would add a second unread interface on top of an unread stream — the failure
class the 2026-08-07 corpus audit spent a session cataloguing, and the one PX's standard exists to
prevent. The row's own acceptance clause says impact events *drive* audio and VFX in a scene, which
a raw interface does not do.

## 3 The filter is declared, not applied

A settled stack of ten crates emits nine `Persist` events every tick without moving. At 60 Hz that
is 540 calls a second into every listener, and a listener that filters them itself pays that cost
once per listener.

So the filter is part of registration:

```cpp
struct PhysicsEventFilter {
    bool begin = true;              // the phase that means "something just happened"
    bool persist = false;
    bool end = false;
    Scalar minimum_impulse = 0;     // newton-seconds; what separates a scrape from a crash
    Scalar pair_cooldown = 0;       // seconds before the same pair may fire again
    bool joint_breaks = true;
};
```

The scene evaluates it once per event and never calls a sink that would have discarded the call.
`minimum_impulse` reads `ContactEvent::impulse`, which exists for exactly this — its own
documentation calls it "what separates a scrape from a crash" and names it as the reason solved
manifolds are read back off the device at all.

The cooldown is per sink per entity pair. Without it a crate landing on a floor produces a `Begin`,
bounces a millimetre, and produces another — one impact, several sounds.

## 4 Where it lives

On `IPhysicsScene`, dispatched immediately after `build_contact_events()` in the tick. Events are
delivered where they are produced, `ISimulation` is not widened, and a gameplay system that wants
impacts asks the physics for them rather than asking the world.

## 5 The impact listener, and the one rule the audio system forces

`ImpactResponse` names what an entity does when it is hit hard enough:

```cpp
struct ImpactResponse {
    Scalar minimum_impulse;    // below this, nothing happens
    Scalar full_impulse;       // at or above this, the sound plays at full gain
    Scalar cooldown_seconds;
    bool plays_audio;          // retrigger this entity's audio emitter
    bool emits_particles;      // retrigger this entity's particle emitter
};
```

Gain is the impulse's position between the two thresholds, clamped — a linear ramp rather than a
curve, because a curve needs an editor for it and a ramp is already the difference between a tap
and a crash.

**The listener must clear `playing` the tick after it sets it**, and that is not tidiness. From
`engine/domain/audio/include/SushiEngine/audio/audio_scene.hpp`: a snapshot with `playing` false
stops the voice; one with `playing` true and no live voice starts a new one; and a non-looping
voice that has finished frees itself, at which point its mapping is dropped. So an emitter left at
`playing = true` after a one-shot finishes is started again on the very next frame, and again after
that. A single impact becomes a permanent loop of a sound authored not to loop.

Setting it for one tick and clearing it on the next gives exactly one voice per impact, and gives
the retrigger-on-next-impact behaviour for free: while the previous voice is still live the code
above updates its gain instead of stacking a second one.

## 6 What changes

| Layer | File | Change |
|---|---|---|
| Services | `physics_services.hpp` | `IPhysicsEventSink`, `PhysicsEventFilter`, the registry on `IPhysicsScene` |
| Scene | `physics_simulation.hpp` | The sink list, the per-pair cooldown clock, dispatch after `build_contact_events` |
| ECS | `simulation.hpp` | `ImpactResponse` and its `IWorldEditor` surface |
| Listener | `simulation/impact_response.hpp` | The engine's own sink: reads the component, fires the emitters, quiets them next tick |
| Scene file | the serializer | The component round-trips |
| Editor | the inspector | An Impact Response section, drawn unlinked first |

## 7 Acceptance

1. A sink below its impulse threshold is not called; above it, it is.
2. A settled stack delivers nothing, because `persist` is off by default.
3. The cooldown suppresses a second hit inside its window and allows one after it.
4. A removed sink stops being called.
5. A real impact leaves the entity's audio emitter playing with a gain between the thresholds.
6. The tick after, that emitter is quiet again — the test that fails if the loop rule is lost.

All on the CPU backend. **P9-C needs no GPU.**

## 8 Out of scope

- **A demo scene file.** It needs an impact sound and a spark material, and inventing silent or
  invisible placeholders would be closing the acceptance clause in name only. The listener and its
  tests are the deliverable; a scene is an asset task.
- **Curved impulse-to-gain mapping, material-dependent sounds, per-surface impact sets.** All real,
  none needed to make the stream reach a speaker.
- **Trigger enter/exit as a distinct event kind.** `ContactEvent` already carries trigger overlaps
  with a zero impulse and a phase; a listener that wants them declares `begin` and a zero threshold.
