# SushiEngine

SushiEngine is a C++17 game engine built as a **head** on top of
[SushiRuntime](https://github.com/SushiSystems/SushiRuntime), the **battery** it
plugs into. SushiRuntime provides hardware discovery, USM allocation, the SYCL
task graph, and the dependency tracker; it knows nothing about a game, an
entity, a component, or a renderer. SushiEngine provides everything that does:
the ECS, the fixed-step deterministic loop, a physics constraint solver, the
animation and audio stacks, the Vulkan renderer, and an editor shell — all
expressed as ordinary read/write task graphs handed to the runtime.

The dependency points one way only:

```
SushiEngine  ──depends on──▶  SushiRuntime
```

It is built around one central idea, repeated at every layer: **a system
declares which memory it reads and writes, and the runtime's dependency
tracker derives the ordering.** Nothing in the engine writes its own
scheduler. Concretely:

- **The ECS *is* the runtime's dependency tracker, wearing a game-engine
  costume.** Components live in archetype chunks of structure-of-arrays
  columns; a column's base pointer is the actual resource the runtime keys
  on. Two systems touching disjoint chunks/columns run in parallel with no
  scheduling code written in the engine at all.
- **The physics solver is Projected Gauss-Seidel, parallelized by graph
  colouring**, and its unified XPBD (position-based) generalization — one
  compliant-constraint framework that rigid bodies, cloth, and soft bodies all
  reuse without a new solver type per shape.
- **The renderer is a real frame graph**, not a sequence of hand-recorded
  passes: a pass declares what it touches and the graph derives the barriers,
  the layouts, and the transient-memory aliasing.
- **SushiLoop** (`loop/`) ties a fixed-timestep clock, seeded RNG, per-tick
  input, and rollback/reconciliation into one authoring surface (`Loop::App`)
  for deterministic, replayable simulation.

## Requirements

- The **SushiStack** workspace repository, which holds the shared
  `dependencies/` tree used by both SushiEngine and SushiRuntime: the
  intel/llvm SYCL `clang++` toolchain, vcpkg, CMake, CTest, Doxygen, and
  pkgconf. There are no system-wide dependencies to install outside of it.
- The **SushiRuntime** sibling, checked out next to this repository (at
  `../sushiruntime` by default; override with `-DSUSHIRUNTIME_DIR=...` or the
  `SUSHIRUNTIME_DIR`/`SUSHISTACK_HOME` environment variables).
- On Windows, a Developer Command Prompt (vcvars) so MSVC and the resource
  compiler are on the path — the `se` CLI snapshots this environment for you.
- Everything below this line is resolved through vcpkg (via the shared
  `dependencies/` tree) and is **not** something you install by hand:
  - **SDL2** (`sdl2[vulkan]`) — only pulled in when the editor, the audio
    library, or the input library is built.
  - **Vulkan** — `vulkan-headers`, `vulkan-loader`,
    `vulkan-memory-allocator`, `vk-bootstrap`, and `glslang`, all from vcpkg.
    There is no dependency on a system-wide Vulkan SDK install.
  - **GoogleTest**, only when tests are enabled.
  - **nlohmann_json**, **stb**, **cgltf**.
  - Optional audio codecs/HRTF — **Opus**, **Vorbis**, **HDF5** (for measured
    SOFA/MagLS HRTF) — soft-guarded; the demos that need them are skipped,
    not the engine build, when they're absent.
- **Dear ImGui** is vendored as a **git submodule** (docking branch) at
  `third_party/imgui` — run `git submodule update --init --recursive` before
  building the editor. **miniaudio** is vendored as a single committed header
  at `third_party/miniaudio/miniaudio.h` (not a submodule).

## Setup

The toolchain and dependencies live inside the SushiStack workspace repository,
shared with SushiRuntime. Get it with the umbrella installer, then add both
engine repositories:

```powershell
irm https://sushisystems.io/install.ps1 | iex   # Windows (PowerShell)
```

```bash
curl -fsSL https://sushisystems.io/install.sh | bash   # Linux / WSL
```

```
ss add sushiruntime sushiengine    # clone both repos into the workspace
ss install-cli sushiengine         # install `se` (and `sushiengine`)
```

If SushiStack is already cloned, you only need the two repositories checked
out side by side and the `se` CLI installed (`pip install -e cli`, after
installing the shared, unpublished `sushicli` presentation layer once:
`pip install -e ../sushicli`).

Build and run the ECS worked example:

```bash
se build
se run sandbox
se test --suite functional
```

If `sandbox` prints `RESULT: OK` and the functional suite passes, you're ready
to write code. See [INTRODUCTION.md](INTRODUCTION.md) for a full walkthrough
of the ECS from an empty program.

### Machine-specific paths

The CLI resolves the compiler and vcpkg from the runtime/stack automatically.
When the toolchain lives somewhere the defaults don't expect, put absolute
paths in a gitignored `cli/config.local.toml` next to `cli/config.toml`, using
the same `[tool.<platform>]` section layout. Run `se config` to print the
resolved values and where each came from.

## The `se` CLI

`se` (and its long name, `sushiengine`) wraps CMake/CTest so you don't type
the configure line by hand.

| Command | What it does |
|---|---|
| `se build [--type release\|debug\|relwithdebinfo] [--clean] [--no-test]` | Configure and build against the SushiRuntime sibling. The test suite builds by default; `--no-test` sets `SE_BUILD_TESTS=OFF`. `--clean` deletes the build tree first. |
| `se test [--suite unit\|regression\|integration\|functional\|all] [--filter <regex>] [--repeat N]` | Run the suite via CTest labels (`functional` is the default, matching `unit\|integration\|regression`). `--filter` is a `ctest -R` regex over `Suite.Case` names. `--repeat N` re-runs until the first failure. |
| `se run [target] [--sort] [-- args…]` | Run a built executable (default target: `sandbox`). Matches exactly, then by substring. `--sort` picks one interactively. Arguments after `--` are forwarded. |
| `se editor [--type release\|debug\|relwithdebinfo] [--no-run]` | Build the ImGui editor (`SE_BUILD_EDITOR=ON`, its own `build-editor/` tree so it never clobbers `se build`'s `CMAKE_BUILD_TYPE`) and launch it. `--no-run` builds only. |
| `se render [--no-run]` | Build and run the headless Vulkan `render_probe` smoke test (`SE_BUILD_RENDER=ON`). |
| `se audio [--no-run]` | Build and run the audio demo (`SE_BUILD_AUDIO=ON`). |
| `se clean` | Remove the `build/` tree. |
| `se doxygen` | Generate Doxygen documentation. |
| `se config` | Print the resolved config and each value's source. |
| `se env [--all]` | Print the environment build/run subprocesses execute under. |
| `se docker build [--no-cache] [--runtime-ref <ref>]` | Build the containerized dev image (toolchain + SushiRuntime sibling cloned at `<ref>`, default `main`, + CLI). |
| `se docker run [--admin] [--no-gpu]` | Start an interactive container with the source mounted live. |

After changing toolchain paths in `config.local.toml`, run `se clean` before
reconfiguring — CMake bakes the old paths into its cache.

## Building without the CLI

```bash
cmake -S . -B build -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_COMPILER=<sushistack>/dependencies/toolchains/llvm-sycl/bin/clang++ \
  -DVCPKG_ROOT=<sushistack>/dependencies/vcpkg \
  -DCMAKE_TOOLCHAIN_FILE=<sushistack>/dependencies/vcpkg/scripts/buildsystems/vcpkg.cmake

cmake --build build --target sandbox     # ECS worked example
cmake --build build --target pgs_demo    # physics demo
```

```bash
./build/sandbox     # exits 0 on success
```

Every build option is off by default and additive:

| Option | Default | Gates |
|---|---|---|
| `SE_BUILD_TESTS` | OFF | `tests/` (GoogleTest, `se_functional_tests`) |
| `SE_BUILD_RENDER` | OFF | `render/` — the Vulkan library and `render_probe` |
| `SE_BUILD_INPUT` | OFF | `input/` — the SDL input translator |
| `SE_BUILD_AUDIO` | OFF | `audio/` and its 21 `audio_*_demo` executables |
| `SE_BUILD_EDITOR` | OFF | `editor/` and `sim/`; also forces `SE_BUILD_RENDER`, `SE_BUILD_INPUT`, and `SE_BUILD_AUDIO` to `ON` |
| `SE_DETERMINISTIC_FP` | ON | `-fno-fast-math -ffp-contract=off` on Clang/AppleClang/GNU |

**`Scalar` is always `double`** (`include/SushiEngine/core/types.hpp`), with no
build option or CLI flag to change it — the engine's world is planet- and
solar-scale, where single precision quantizes camera and transform math to the
metre. (Physics *constraint solving* separately supports a template
precision parameter per call site — a distinct, lower-level knob from the
`Scalar` boundary type.)

## Testing

```bash
se test --suite functional
# or directly:
ctest --test-dir build -L "unit|integration|regression" --output-on-failure
```

Tests live under `tests/common/` (shared GoogleTest entry point) and
`tests/functional/{unit,integration,regression}/` — one binary,
`se_functional_tests`, with CTest labels derived from each test's GTest suite
prefix (`Unit_*`, `Integration_*`, `Regression_*`), not from directory
location. The suite instantiates real kernels against the real runtime —
there are no mocks. CI (`.github/workflows/ci.yml`) builds and runs this suite
on every push/PR inside the `intel/oneapi-basekit` image, builds the editor
target separately (no tests), and publishes a Doxygen API site.

## How it works

A system names the components it reads and writes; the schedule compiles that
into a runtime task graph once and replays it every frame. This is
`sandbox/main.cpp`, the project's own worked example, trimmed:

```cpp
#include <SushiEngine/SushiEngine.hpp>
using namespace SushiEngine;

struct Position { Vector3 v; };
struct Velocity { Vector3 v; };
struct Mass     { Scalar value; };

auto runtime = SushiRuntime::API::Runtime::create();
World world(runtime, /*chunk_capacity=*/2048);
Schedule schedule(runtime);

world.reserve<Position, Velocity, Mass>(2048);   // pre-reserve: no mid-run chunk alloc

// Write<Velocity> then Read<Velocity> in "integrate" is a read-after-write chain —
// the runtime's dependency tracker orders the two systems; it is never told to.
schedule.each<Write<Velocity>, Read<Mass>>("apply_forces",
    [](std::size_t i, Velocity* vel, const Mass* mass) { vel[i].v.y += -9.8 / mass[i].value * 0.01; });
schedule.each<Write<Position>, Read<Velocity>>("integrate",
    [](std::size_t i, Position* pos, const Velocity* vel) { pos[i].v = pos[i].v + vel[i].v * 0.01; });

Entity e = world.spawn(Position{}, Velocity{}, Mass{Scalar(1)});
for (int frame = 0; frame < 300; ++frame)
    schedule.run(world);   // compiles once, replays every other call

std::printf("y = %f\n", world.get<Position>(e).v.y);
```

See [INTRODUCTION.md](INTRODUCTION.md) for the full tour (`World`, `Schedule`,
`CommandBuffer`, `Entity`) and [ARCHITECTURE.md](ARCHITECTURE.md) for how every
layer below fits together.

## Architecture

The engine is organized in layers under `include/SushiEngine/`, most of it
header-only, with a handful of compiled libraries where device code or a
third-party backend (SDL2, Vulkan) is involved.

### SushiLoop core (`loop/`)
`Loop::App<Command>` (`app.hpp`) is the authoring surface: it owns a `World`,
`Schedule`, `CommandBuffer`, `FixedTimestepClock`, seeded `RngState`,
`InputHistory<Command>`, and an optional `RollbackBuffer`, and drives one
fixed tick — capture snapshot → sample command → apply → run the schedule →
apply deferred structural changes → reconcile against a network transport if
one is connected. `App::system<Access...>(name).each(fn)` is sugar directly
over `Schedule::each`. Rollback capture/restore and the reconciliation
algorithm (find-earliest-mismatch, restore, replay) are real and covered by
determinism tests — but two things are deliberately scoped out today:
rollback cannot survive a spawn/destroy inside the rolled-back range, and
`Net::LoopbackTransport`/`reconcile` are loopback-only scaffolding proving the
shape of a future transport — there are no sockets or serialization yet.

### ECS (`ecs/`)
Entities are generation-checked handles (`Entity{index, generation}`).
Components are trivially-copyable structs; each distinct type gets a process-
global id and its own column. An **archetype** is one entity shape; it owns a
list of **chunks**, each chunk a structure-of-arrays block where every
component is its own runtime (USM) allocation — a chunk's column base pointer
*is* the dependency key the runtime's tracker orders systems by. `World`
handles spawn/destroy/get/query; destroy is an O(1) swap-remove that bumps the
freed slot's generation so a stale handle is caught, not silently reused.
`Schedule::each<Read<T>/Write<T>...>` resolves each access to a column pointer
per chunk and emits one graph node per chunk; it rebuilds only when the
world's `structure_version` changes (a new archetype or chunk), and replays
the compiled graph otherwise. `CommandBuffer` defers destroys to an explicit
barrier so a system running as a device kernel never sees an entity vanish
mid-frame.

### Physics (`physics/`)
A **Projected Gauss-Seidel** (PGS) constraint solver, parallelized by
**graph colouring**: constraints sharing a body are edge-coloured apart, so
each colour is race-free and runs fully in parallel while colours themselves
are ordered into a sequential sweep by the shared position buffer.
`ConstraintSolver<DistanceConstraint>` is the concrete instance. **XPBD**
(`RigidBody`, `XpbdDistanceConstraint`, `XpbdSolver<Constraint>`) reuses the
same colouring and compile-once/replay structure over one `RigidBody` buffer
(position + orientation + inverse inertia), adding a per-constraint
`compliance` — `compliance == 0` is mathematically the plain PGS case.
`PhysicsWorld<Constraint>` turns a one-shot solve into a loop:
register → `finalize()` → `step()` runs predict → solve → derive-velocity per
sub-step, with an optional post-solve callback for contacts. Cloth
(`build_cloth_grid`) and soft bodies (`build_soft_body_lattice`) introduce no
new solver — they're topologies of `RigidBody`s wired with structural and
shear `XpbdDistanceConstraint`s. Narrowphase (`collision.hpp`) is pure,
runtime-free geometry — sphere/plane/AABB **and oriented-box (OBB) via full
15-axis SAT**, returning a `Contact{normal, depth, point}`. `contact_solver.hpp`
resolves non-penetration as a positional projection pass regenerated each
sub-step (inelastic — no restitution term), and its `ContactBody`-based path
is genuinely **two-way**: rigid bodies and cloth particles are broadphased and
resolved together, each pushed by its generalized inverse mass, confirmed by
a test that a rigid body is displaced by an unpinned cloth particle it
overlaps. Known gaps, stated plainly rather than glossed over: no friction, no
restitution, one contact point per pair (a resting box can rock), and no
cloth self-collision yet.

### Animation (`animation/`)
A full skeletal-animation stack: skeleton/clip assets, the deterministic
`animator_step` interpreter, `AnimatorEvaluator` (blend trees, mask-gated
layers, additive blending), an IK/pose-modifier stack (two-bone, full-body,
look-at, foot placement), morph targets and generic tracks, humanoid
retargeting, and JSON controller authoring behind an `IAnimationDatabase`
seam — plus motion matching, jiggle bones, ragdoll blending, dual-quaternion
skinning, and facial blendshapes. Every one of these has a real header and a
runnable example under `examples/`. Ten CTest suites and 125 cases cover it: the
asset formats, the deterministic tick and its rollback invariant, the blend trees,
the layer/mask fold, the pose-modifier stack, retargeting, the controller's JSON
persistence, the authoring curves and their bake, and the motion-matching /
dual-quaternion / blendshape / sequencer extras. One piece is still untested and
named as such — the device-side batch evaluator, the only part that needs
SushiRuntime.

### Audio (`audio/`)
The full audio roadmap (phases S0 through S10) is implemented: spatialization
and DSP, mixing, environmental propagation with portals and early
reflections, occlusion, streaming banks/events, an in-editor profiler,
procedural/modal synthesis, convolution, and a SYCL accelerator backend — plus
further hardening beyond the roadmap: measured SOFA/MagLS HRTF, Vorbis/Opus
streaming codecs, ray-traced acoustics, an authoring DAW-style panel, a bus
dynamics rack, discrete multichannel surround, and a lock-free control-plane
ring for multi-core voice rendering. Five of the 21 audio demos
(`audio_opus_demo`, `audio_vorbis_demo`, `audio_sofa_demo`, `audio_magls_demo`,
`audio_stream_compressed_demo`) need the optional Opus/Vorbis/HDF5 vcpkg
packages; the source is complete and wired into CMake either way, they simply
don't compile without those packages installed.

### UI (`ui/`)
A retained, ECS-native UI (Unity UGUI-shaped): `RectTransform`, `Canvas`,
`UIImage`, `UIText`, `UIButton` components, a `resolve_rect` anchor solver,
a pointer/click model, and a `UI` façade whose `canvas()`/`panel()`/
`image()`/`label()`/`button()` calls spawn real ECS entities rather than a
separate scene graph.

### Render (`render/`, `include/SushiEngine/render/`)
A **Vulkan 1.4** renderer, the only backend implemented today, behind a
dependency-inversion `IRenderDevice` boundary so a second backend can follow
without touching a consumer. `IWindowRenderer` drives the
acquire → clear → submit → present cycle; a headless offscreen path backs
`render_probe`, a CI smoke test that renders one triangle and checks it —
verified passing on real hardware in this session, though it exercises only
the device/shader/submit path, not the full pass stack. `ISceneView` is
double-buffered by default (a 3-slot ceiling supports triple-buffering via
settings), draws `MeshKind::{Box,Sphere,Cylinder}` primitives plus imported
glTF meshes, skinned and particle instances, and triangulated cloth strands,
and exposes GPU id-buffer picking through a dedicated `R32_UINT` target. The
**render graph** (`render/graph/`) is a real compiler, not a stub: it culls
passes with no live reader, derives every barrier's stage/access/layout triple
from declared `read`/`write`/attachment access, aliases transient resources
by lifetime, and splits submissions across queues wherever async compute is
used. Around three dozen passes sit on top of it — opaque/transparent/shadow
geometry, GTAO, SSR, ray-traced shadows, a full volumetric cloud/atmosphere
stack (LUTs, light volume, shadow map, panorama, march, composite), the
post-processing chain (DoF, motion blur, auto-exposure, bloom, tonemap), TAA/
FXAA, and GPU-driven culling (frustum + screen-coverage LOD + Hi-Z occlusion).
The resource layer (`render/resources/`) — pooled transients, a bindless
descriptor heap, a disk-backed pipeline cache with background pipeline
upgrades, a sampler cache, and a shader library with dev-time hot reload — is
likewise a real, working implementation throughout.

> **Current state of the cloud/weather system**: Phase A (coupling the
> weather simulation to the sky spatially) has shipped. Phase B (windowing the
> baked cloudscape into non-wrapping, camera-centred fields and deriving cloud
> genus per column from the simulation) is implemented and compiles/links
> cleanly, but as of this writing its own test suite and a live visual check
> in the editor have not both been run in the same session — treat it as an
> active, working-but-not-fully-verified refactor rather than a finished
> feature. See `docs/CHANGELOG.md`'s `[Unreleased]` entry for specifics.

### Astronomy (`astro/`)
Orbital mechanics and ephemeris backing the planet- and solar-scale world the
render seam draws: Julian dates, orbital elements, gravity/gravity fields,
reference frames (topocentric, surface, scene), body orientation, a star
catalogue, and celestial-body/ephemeris data.

### VFX (`vfx/`)
A data-driven particle-effect authoring pipeline: emitter descriptors are
compiled (`emitter_compiler.hpp`) into a `CompiledEmitter` a deterministic
backend can step, with curve/gradient/random modules and an effect database —
feeding the renderer's GPU-simulated and mesh-particle passes.

### Simulation seam (`sim/`)
The one compiled library outside an example that owns device code end to end,
`sushi_sim`, behind the plain-C++ `ISimulation` seam. It owns a
`SushiRuntime::API::Runtime`, a `World`, and a `Schedule`; every archetype is
pre-reserved up front so editor-driven creates never trigger a mid-run
allocation. Each `tick()` runs the schedule, then an extract pass copies the
world's shared-USM columns back into a read-only `RenderScene` the editor
draws — the one seam that keeps the runtime, SYCL, and ECS out of the
editor's own translation units entirely.

### Editor (`editor/`)
An SDL2 + Dear ImGui desktop shell with a docking layout: Hierarchy,
Inspector, Project browser, Text Editor, Toolbar, Console, Statistics,
Preferences, Environment, Post Process, Rendering, GPU Culling, Lighting, and
Input Manager panels, plus Scene and Game viewports that are two instances of
the same `ViewportPanel`. Undo/redo is whole-world JSON-snapshot based
(`CommandHistory`), with a single-step mode for discrete edits and a
begin/end-bracketed mode for continuous drags. A `GizmoController` offers
translate/rotate/scale in local or world space. Scenes round-trip through a
`.sushiscene` JSON file (`scene_serializer.*`) purely through the same
`IWorldEditor` query/mutate surface the panels use.

## Project layout

```
include/SushiEngine/   Header-only engine core
  loop/                 SushiLoop: App, FixedTimestepClock, rng, input, rollback, net
  ecs/                  Entity, component, archetype, chunk, world, command buffer, schedule
  physics/              PGS + graph colouring, XPBD, cloth, soft body, collision, contacts
  animation/            Skeleton/clip/controller assets, evaluator, IK, retargeting (44 files)
  audio/                Spatialization, DSP, propagation, banks, procedural synthesis (54 files)
  ui/                   Retained ECS UI: RectTransform, Canvas, UIImage/Text/Button
  render/               Public render seam: RHI device, scene view, window renderer (11 files)
  astro/                Orbital mechanics, ephemeris, reference frames (13 files)
  vfx/                  Particle-effect authoring: emitter compiler, modules, curves
  sim/                  Public simulation seam consumed by the editor (16 files)
  input/                Public input action layer (19 files)
  core/                 The single alias point for value types (Vector3, Scalar)
  SushiEngine.hpp        Umbrella header
sandbox/                 The ECS worked example, its own single-TU SYCL executable
examples/                ~50 topic demos: physics, animation, audio, VFX, UI, networking
render/                  Compiled Vulkan 1.4 renderer library (sushi_render) + render_probe
sim/                     The sushi_sim library: runtime_simulation.cpp behind ISimulation
audio/                   Compiled SDL-backed audio device (sushi_audio)
input/                   Compiled SDL-backed input device
editor/                  The SDL2 + Dear ImGui editor shell (se_editor)
tests/
  common/                 Shared GoogleTest entry point and helpers
  functional/unit/         53 files
  functional/integration/  20 files
  functional/regression/   1 file
cli/                     The `se` / `sushiengine` developer CLI (Python/Typer)
third_party/             Vendored imgui (submodule) and miniaudio (header)
cmake/                   ProjectOptions, Runtime, SyclTarget, Toolchain, Vcpkg modules
.github/workflows/       CI: functional tests + editor build per push/PR, nightly Doxygen
docs/                    This guide, ARCHITECTURE.md, INTRODUCTION.md, CLI_GUIDE.md, CHANGELOG.md
```

## Documentation

- [INTRODUCTION.md](INTRODUCTION.md) — a from-scratch tour of the ECS, ending
  in the real `sandbox/main.cpp` example.
- [ARCHITECTURE.md](ARCHITECTURE.md) — how every layer fits together in full
  depth, and the seams a cross-cutting change touches.
- [CLI_GUIDE.md](CLI_GUIDE.md) — every `se` command in detail.
- [CONTRIBUTING.md](CONTRIBUTING.md) — coding style, the documentation
  requirements for a change, and how to open a pull request.
- [docs/CHANGELOG.md](CHANGELOG.md) — notable changes, accumulating under
  `[Unreleased]` until the project cuts its first tagged release (see
  [CONTRIBUTING.md](CONTRIBUTING.md)).

## License

This project is licensed under the Apache License, Version 2.0 — see the
[LICENSE](../LICENSE) file for details.
