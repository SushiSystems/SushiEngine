# Physics snapshot and rollback (P9-D) — design

**Date:** 2026-08-08
**Phase:** `docs/design/physics_system.md` §16, row P9, fourth of five sub-projects
**Status:** approved, rigid core first

## 1 What this builds

`IPhysicsScene` gains the ability to hand out its live state as bytes and to be put back from
them. The acceptance is not a feature anyone sees; it is a proof: capture at tick K, run on to N,
restore K, replay to N, and the state at N must be **byte-identical** to the state the
uninterrupted run reached.

This is the one item of the P9 row that needs no GPU. §0.5 makes determinism a build-time property
and the CPU backend is deterministic, so the number this row is graded on can actually be produced
on the machine it is being written on — which is not true of any §13.1 target.

## 2 What is captured, and what is deliberately not

The rule that sorts them is one question: **does the next tick read this, or rebuild it?**

**Captured** — the next tick reads it:

| State | Where it lives | Why |
|---|---|---|
| Body columns | the solver, mirrored in `bodies_` | Pose, velocity, flags, sleep timer, island index — the simulation itself |
| Joint state | `joints_` and the solver | Multipliers and break flags; a joint that has taken load is not a fresh joint |
| Contact records | `current_`, `previous_` | **The load-bearing one — see §3** |
| Island partition | `islands_` | Next tick's sleep decision is taken against it |
| Tick scalars | the scene | The event clock, the sink cooldowns, the continuous-collision budget counters |
| Kinematic targets | `RigidEntry::kinematic_target` | A command issued before the rollback point is still owed |

**Not captured** — the next tick rebuilds it:

`contact_index_`, `contact_proxies_`, `query_index_`, `query_shapes_`, `query_entities_`,
`events_`, `joint_events_`. Every one is derived from a dirty flag. Capturing them would double the
blob and create a second source of truth for something that already has one, which is the more
expensive of the two mistakes: a stale derived index that disagrees with the state it was derived
from fails in a way nothing traces back to the snapshot.

Restore therefore ends by marking those dirty rather than by writing them.

## 3 The contact records are the whole difficulty

`current_` and `previous_` look like bookkeeping and are not. They are what the manifold pass
matched against last tick, and **warm starting lives in them**: a contact that persists carries its
accumulated impulse forward, which is what lets a stack settle instead of sinking and rebounding.

A snapshot that skipped them would restore a simulation that looks identical — every body in
exactly the right place — and then converges differently on the very next tick, because every
contact starts cold. Byte equality would fail at the first touch, and the pose comparison that
would have caught it earlier passes.

§12.3 names "the warm-start accumulators" in its list without saying where they are. They are here.

## 4 The blob

A flat byte vector, written and read through a bounds-checked cursor
(`engine/world/simulation/include/SushiEngine/simulation/physics_snapshot.hpp`). Arrays of
trivially copyable, pointer-free structs are memcpy'd wholesale, which §12.3 already anticipates:
*"All of it is already a pointer-free column, which is what makes SushiLoop's dirty-chunk snapshot
applicable without a special case."*

**It is an in-process snapshot, not a file format.** Nothing writes it to disk and nothing reads
one produced by a different build, so a struct gaining a field is not a compatibility event. The
header carries a magic and a version anyway, because the cost is eight bytes and the failure it
prevents — a blob read as the wrong shape — is silent.

**Restore requires the same body set.** The blob records the entity ids in slot order and refuses
a restore whose set differs. Rolling back across a spawn or a despawn is a real requirement and a
separate piece of work: it needs the solver's handle table rewound too, and doing it wrong
produces a body that exists in one place and not another. Refusing loudly is the correct behaviour
until then.

## 5 Scope

**This pass: the rigid core.** Bodies, joints, contacts, islands, scalars.

Soft bodies, their plastic rest matrices and fracture, beams, elements, vehicles and cloth are the
next pass and are named as follow-ups. The acceptance clause's "including contacts" is met here;
"and fracture" is not, and that is stated rather than implied.

Taking the rigid core first is a deliberate ordering, not a convenience: a 10,000-tick byte
comparison that fails tells you *that* something diverged, and the smaller the captured surface the
smaller the search. Capturing six subsystems at once and then finding a divergence would make the
first question "which one" rather than "where".

## 6 Acceptance

1. Two fresh scenes given the same input produce byte-identical blobs at every tick. **This is the
   control**, and it comes first: without it, a rollback test that fails cannot distinguish a bad
   snapshot from a simulation that was never deterministic.
2. Capture at K, run to N, restore K, replay to N — the two blobs at N are byte-identical.
3. The same, across a scene with live contacts, so the warm-start half of §3 is exercised.
4. A blob from a different body set is refused rather than misapplied.
5. 10,000 ticks, which is the row's own number.

All on the CPU backend.

## 7 Risk stated up front

The conformance suite compares the host and device solvers at a **1e-9 tolerance**, not bit
equality. Nothing in this codebase has ever asserted byte equality of physics state. It is entirely
possible that the first run of acceptance 1 fails, and that would be a finding rather than a
setback — it would mean a real determinism defect that §12.1's five rules exist to prevent has gone
unnoticed because nothing measured it.

If that happens, the honest response is to report the divergence and its source, not to loosen the
test.
