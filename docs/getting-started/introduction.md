# Getting started with SushiEngine

SushiEngine is a **head** built on top of [SushiRuntime](https://github.com/SushiSystems/SushiRuntime),
the battery it plugs into. The engine owns the game-facing concepts the runtime
knows nothing about — entities, components, systems, physics, rendering, the
editor — and expresses every one of them as an ordinary read/write task graph
handed to the runtime. The runtime's dependency tracker infers the ordering;
the engine never writes a scheduler of its own.

The big idea: **a system just declares which components it reads and writes.**
You never wire up "run this after that." The ECS schedule compiles your
systems into a runtime task graph once, keyed on the archetype chunks each
system touches, and replays that graph every frame — spawning and destroying
entities within existing chunks costs no recompile.

This guide takes you from an empty machine to a working first program, then
tours the core API. Every code sample here is the real, current API and
mirrors what `sandbox/main.cpp` — the project's own worked example — does.

> For the full command reference, see [CLI_GUIDE.md](CLI_GUIDE.md). For the
> architecture and design — the ECS, physics, and render seams — see
> [ARCHITECTURE.md](ARCHITECTURE.md).

---

## 1. Quick start

SushiEngine consumes SushiRuntime as a sibling checkout and reuses its SYCL
toolchain, so get SushiRuntime built first (see its own
[INTRODUCTION.md](../../sushiruntime/docs/INTRODUCTION.md) if you haven't).
Check both projects out side by side:

```
Projects/
  sushiruntime/
  sushiengine/
```

Install the CLI and build. `sushiengine-cli` depends on the shared, unpublished
`sushicli` presentation layer, so install that once first from its sibling
checkout:

```bash
pip install -e ../sushicli        # once, if not already installed
pip install -e cli                # puts `se` and `sushiengine` on your PATH
se build                   # Release build (the default)
se run sandbox              # runs the worked ECS example against its scalar reference
```

You'll need the same SYCL 2020 compiler SushiRuntime uses (the bundled
intel/llvm `clang++` is the primary path), CMake, and vcpkg. See
[CONTRIBUTING.md](CONTRIBUTING.md) for the raw `cmake` invocation if you'd
rather skip the CLI, and how to point it at a SushiRuntime checkout that isn't
at `../sushiruntime`.

Verify everything works:

```bash
se test --suite functional
```

If `sandbox` prints `RESULT: OK`, you're ready to write code.

---

## 2. The ECS mental model

A program written against SushiEngine has five moving parts:

1. **`SushiRuntime::API::Runtime`** — the runtime instance the engine hands
   task graphs to. Created once, passed to the `World` and `Schedule`.

2. **Components** — plain structs. Each distinct type gets its own component
   id and its own column inside an archetype chunk (structure-of-arrays), so
   two systems touching different components can run in parallel with no
   engine-side scheduling code.

3. **`World`** — owns entities and their component storage (archetype chunks).
   You `reserve` an archetype's chunk capacity up front, `spawn` entities into
   it, and `get` a live entity's component.

4. **`Schedule`** — where you register systems. Each system names the
   components it reads (`Read<T>`) and writes (`Write<T>`) and a per-entity
   function; the runtime's dependency tracker orders the systems from that
   access alone. `schedule.run(world)` compiles the graph the first time (and
   after any structural change) and replays it every other call.

5. **`CommandBuffer`** — structural changes (destroy, and in the future spawn
   deferred to the barrier) are recorded here during the frame and applied
   once, after the schedule runs. Systems run as device kernels and must never
   see an entity appear or vanish mid-frame.

The lifecycle every frame is: **run the schedule → apply deferred structural
changes**. Registering systems and pre-reserving archetypes happens once,
before the loop starts.

---

## 3. Your first program

This is `sandbox/main.cpp`, the project's own worked example: a particle world
— position, velocity, mass, and a lifetime that despawns an entity — driven
entirely by the ECS and checked every frame against an independent scalar
reference.

```cpp
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <unordered_map>
#include <vector>

#include <SushiEngine/SushiEngine.hpp>

using namespace SushiEngine;

namespace
{
    // Components. Distinct types so each gets its own component id and column.
    struct Position { Vector3 v; };
    struct Velocity { Vector3 v; };
    struct Mass     { Scalar value; };
    struct Lifetime { Scalar value; };

    constexpr Scalar DT      = Scalar(0.01);
    constexpr Scalar FORCE_Y = Scalar(-9.8);   // a downward force; accel = F / mass
    constexpr std::size_t CHUNK_CAPACITY = 2048; // one chunk holds the whole world
}

int main()
{
    // 1. Create the runtime, then the World and Schedule over it.
    auto runtime = SushiRuntime::API::Runtime::create();
    World world(runtime, CHUNK_CAPACITY);
    Schedule schedule(runtime);

    // 2. Pre-reserve the one archetype the whole world lives in, so a spawn
    //    never allocates a new chunk mid-run (which would force a recompile).
    world.reserve<Position, Velocity, Mass, Lifetime>(CHUNK_CAPACITY);

    // 3. Register systems. apply_forces writes Velocity, integrate reads it —
    //    a read-after-write chain the runtime orders for you. decay_lifetime
    //    touches a disjoint component and runs in parallel with both.
    schedule.each<Write<Velocity>, Read<Mass>>("apply_forces",
        [](std::size_t i, Velocity* vel, const Mass* mass)
        {
            vel[i].v.y += (FORCE_Y / mass[i].value) * DT;
        });
    schedule.each<Write<Position>, Read<Velocity>>("integrate",
        [](std::size_t i, Position* pos, const Velocity* vel)
        {
            pos[i].v = pos[i].v + vel[i].v * DT;
        });
    schedule.each<Write<Lifetime>>("decay_lifetime",
        [](std::size_t i, Lifetime* life)
        {
            life[i].value -= DT;
        });

    // 4. Spawn some entities.
    const Entity e = world.spawn(Position{}, Velocity{}, Mass{Scalar(1)}, Lifetime{Scalar(5)});

    // 5. Run the schedule once per frame.
    CommandBuffer commands;
    for (std::size_t frame = 0; frame < 300; ++frame)
    {
        const SushiRuntime::RunReport report = schedule.run(world);
        (void)report; // report.total_duration_ms, etc.

        if (world.get<Lifetime>(e).value <= Scalar(0))
            commands.destroy(e);          // deferred — not applied yet

        commands.apply(world);            // applied at the frame barrier
    }

    // 6. Read results back.
    if (world.alive(e))
        std::printf("y = %f\n", world.get<Position>(e).v.y);

    return 0;
}
```

### Building and running it

Declare your `.cpp` as its own target in the root `CMakeLists.txt` (there is no
separate `examples/CMakeLists.txt` — every example is its own single-TU SYCL
executable declared with `sushiengine_add_sycl_executable(name examples/name.cpp)`,
right alongside the ~50 existing ones), then build and run through the CLI:

```bash
se build
se run sandbox
```

`se run` matches the target name exactly first, then by substring, so a
partial name works too.

---

## 4. The core API, in detail

### `Entity`

A stable handle: a slot `index` plus a `generation`. A handle is valid only
while the world's generation for its slot still matches — so a stale handle to
a destroyed-and-reused slot is detected (`world.alive(e)` returns `false`),
never silently treated as pointing at the wrong entity.

```cpp
Entity e = world.spawn(Position{}, Velocity{}, Mass{Scalar(1)}, Lifetime{Scalar(5)});
if (e.is_null()) { /* ... */ }
```

### `World`

```cpp
World world(runtime, chunk_capacity);
```

- **`reserve<Ts...>(count)`** — pre-allocates an archetype's chunk storage for
  up to `count` entities of that exact component set. Do this for every
  archetype you'll spawn into before the schedule first runs, so a later spawn
  never triggers a mid-run chunk allocation (which bumps `structure_version`
  and forces a recompile).
- **`spawn(Ts... values)`** — creates an entity with the given component
  values, placing it into the archetype matching that component set.
- **`destroy(Entity e)`** — immediate destroy (asserts the entity is alive).
  Prefer `CommandBuffer::destroy` from inside a frame that has already run its
  schedule this step, so a system never observes the removal mid-frame.
- **`alive(Entity e) const`** — checks the slot's generation still matches.
- **`get<T>(Entity e)`** — returns a reference to entity `e`'s `T` component
  (asserts the entity is alive and its archetype has a column for `T`).

### `Schedule`

```cpp
Schedule schedule(runtime);
```

- **`each<Access...>(name, fn)`** — registers a system. Each `Access` is
  `Read<T>` or `Write<T>`; `fn` takes an index plus one pointer per `Access`,
  in the same order (`const T*` for `Read<T>`, `T*` for `Write<T>`). The
  Schedule emits one graph node per matching chunk, keyed on that chunk's
  columns — this is the entirety of "the system scheduler": the runtime's
  read/write dependency tracker derives the ordering from the access lists
  alone.
- **`run(world)`** — compiles the graph the first time it's called and after
  any structural change (`world`'s `structure_version` changed since the last
  compile), then replays the compiled graph. Returns a
  `SushiRuntime::RunReport` (timing, task count).
- **`compile_count()`** — how many times the graph has been (re)compiled.
  Should stay `1` across a run with pre-reserved archetypes and no new
  archetype/chunk — a useful assertion in your own code, the way
  `sandbox/main.cpp` checks it.
- **`system_count()`** — the number of registered systems.

### `CommandBuffer`

```cpp
CommandBuffer commands;
commands.destroy(e);      // recorded, not applied yet
commands.apply(world);    // applies every recorded destroy, once
```

Record structural changes during the frame (after the schedule has run) and
apply them at one explicit barrier before the next `schedule.run(world)` call.
Destroy is an O(1) swap-remove that keeps a chunk's live rows packed; the
destroyed entity's slot gets a bumped generation, so any handle still
referring to it fails `world.alive()`.

### `Read<T>` / `Write<T>`

The access tags a system's template argument list is built from
(`ecs/component.hpp`). `Read<T>` grants a `const T*` to the system body;
`Write<T>` grants a `T*`. Two systems with disjoint access run in parallel;
any read/write or write/write overlap on the same component is ordered by the
runtime.

---

## 5. Where to go next

- **[CLI_GUIDE.md](CLI_GUIDE.md)** — every `se` command: build types, test
  suites, running binaries, the editor, the render/audio probes, and Docker.
- **[ARCHITECTURE.md](ARCHITECTURE.md)** — the full layer map: the ECS and
  system graph in depth (§3), the physics constraint solver (§4), the render
  seam and frame graph (§5), and everything above the ECS (animation, UI,
  SushiLoop).
- **`sandbox/main.cpp`** — the runnable version of §3 above, with the scalar
  reference check in full.
- **`examples/`** — further worked examples over physics (`pgs_demo.cpp`,
  `xpbd_demo.cpp`, `cloth_demo.cpp`, `soft_body_demo.cpp`), animation, audio,
  and UI.
- **`se doxygen`** — generate the full API reference from the headers.
