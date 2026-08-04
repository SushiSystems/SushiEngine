# SushiEngine

SushiEngine is a C++17 game engine built as a **head** on top of
[SushiRuntime](https://github.com/SushiSystems/SushiRuntime), the **battery** it plugs into.
SushiRuntime provides hardware discovery, unified-memory allocation, the SYCL task graph and
the dependency tracker; it knows nothing about a game, an entity, a component or a renderer.
SushiEngine provides everything that does: the entity-component-system, the fixed-step
deterministic loop, a physics constraint solver, the animation and audio stacks, the Vulkan
renderer and an editor shell — all expressed as ordinary read and write task graphs handed to
the runtime.

The dependency points one way only:

```
SushiEngine  ──depends on──▶  SushiRuntime
```

One idea is repeated at every layer: **a system declares which memory it reads and writes, and
the runtime's dependency tracker derives the ordering.** Nothing in the engine writes its own
scheduler.

- **The entity-component-system *is* the runtime's dependency tracker, wearing a game-engine
  costume.** Components live in archetype chunks of structure-of-arrays columns, and a column's
  base pointer is the resource the runtime keys on. Two systems touching disjoint chunks run in
  parallel with no scheduling code in the engine at all.
- **The physics solver is Projected Gauss-Seidel, parallelized by graph colouring**, with a
  unified extended position-based dynamics generalization — one compliant-constraint framework
  that rigid bodies, cloth and soft bodies reuse without a solver type per shape.
- **The renderer is a real frame graph**, not a sequence of hand-recorded passes: a pass
  declares what it touches and the graph derives the barriers, the layouts and the transient
  memory aliasing.
- **SushiLoop** ties a fixed-timestep clock, a seeded random number generator, per-tick input
  and rollback reconciliation into one authoring surface for deterministic, replayable
  simulation.

## Getting started

The toolchain and every dependency live in the **SushiStack** workspace repository, shared with
SushiRuntime. There is nothing to install system-wide.

```powershell
irm https://sushisystems.io/install.ps1 | iex          # Windows (PowerShell)
```

```bash
curl -fsSL https://sushisystems.io/install.sh | bash   # Linux
```

```bash
ss add sushiruntime sushiengine    # clone both repositories into the workspace
ss install-cli sushiengine         # install the `se` command line tool
git submodule update --init --recursive   # Dear ImGui, needed for the editor
```

Then build and run the worked example:

```bash
se build
se run sandbox            # prints RESULT: OK
se test --suite functional
```

If the sample prints `RESULT: OK` and the functional suite passes, the tree is healthy.
[The getting-started tour](docs/getting-started/introduction.md) walks the
entity-component-system from an empty program.

Every build and program action goes through the `se` command line tool — never `cmake` or
`ninja` directly. The five commands above cover most days; the
[command line guide](docs/guides/command-line-interface.md) is the full reference.

## Requirements

- The **SushiStack** workspace, which holds the shared `dependencies/` tree: the intel/llvm
  SYCL `clang++` toolchain, vcpkg, CMake, CTest, Doxygen and pkgconf.
- The **SushiRuntime** sibling checked out beside this repository, at `../sushiruntime` by
  default. Override with `-DSUSHIRUNTIME_DIR=…` or the `SUSHIRUNTIME_DIR` and `SUSHISTACK_HOME`
  environment variables.
- On Windows, a Developer Command Prompt so the Microsoft Visual C++ libraries and the resource
  compiler are on the path. The `se` tool snapshots that environment for you.
- **Dear ImGui** as a git submodule at `third_party/imgui`, and **miniaudio** as a single
  committed header at `third_party/miniaudio/miniaudio.h`.

Everything else — SDL2, the Vulkan headers, loader, memory allocator, `vk-bootstrap` and
`glslang`, GoogleTest, `nlohmann_json`, `stb`, `cgltf`, and the optional Opus, Vorbis and HDF5
codecs — is resolved through vcpkg out of the shared `dependencies/` tree. There is no
dependency on a system-wide Vulkan software development kit.

## Build options

Every option is off by default and additive.

| Option | Default | Gates |
| --- | --- | --- |
| `SUSHIENGINE_BUILD_TESTS` | OFF | `tests/` — GoogleTest and `se_functional_tests` |
| `SUSHIENGINE_BUILD_RENDER` | OFF | `engine/presentation/render/` and `render_probe` |
| `SUSHIENGINE_BUILD_INPUT` | OFF | the SDL input translator |
| `SUSHIENGINE_BUILD_AUDIO` | OFF | the SDL audio device and its demonstration executables |
| `SUSHIENGINE_BUILD_EDITOR` | OFF | `applications/editor/`; forces render, input and audio on |
| `SUSHIENGINE_BUILD_PLAYER` | OFF | `applications/player/` |
| `SUSHIENGINE_BUILD_EXAMPLES` | OFF | `samples/` |
| `SUSHIENGINE_DETERMINISTIC_FP` | ON | `-fno-fast-math -ffp-contract=off` on Clang, AppleClang and GNU |

`SUSHIENGINE_EXECUTION_BACKEND` decides what `SushiEngine::Execution` denotes, and is settled
before `project()` so it also decides whether SushiRuntime is part of the build at all.
`runtime`, the default, is the runtime's SYCL task graph. `native` is the thread-pool backend
for platforms the runtime cannot reach; it needs neither a SushiRuntime checkout nor a SYCL
toolchain. Each lane owns its own build tree, so switching never reuses the other's cache.

**`Scalar` is always `double`**
(`engine/foundation/core/include/SushiEngine/core/types.hpp`), with no option to change it. The
engine's world is planet- and solar-scale, where single precision quantizes camera and transform
mathematics to the metre.

## Repository layout

```
engine/         the engine, one directory per module, grouped by tier
  foundation/     core · ecs · execution · platform
  domain/         animation · astro · atmosphere · audio · environment · geometry
                  input · material · physics · terrain · ui · vfx
  asset/          gltf
  presentation/   render
  world/          authoring · loop · serialization · simulation
applications/   editor · player
samples/        the worked examples
tools/          shader_compiler · probes · documentation
tests/          unit · integration · regression · native_execution · goldens
cli/            the `se` command line tool
docs/           the manual
```

A module may depend on its own tier and on anything below it. The order lives in
`cmake/EngineLayers.cmake` and is enforced at configure time by `sushiengine_add_module()`, so a
violation fails the configure rather than shipping. Each module owns its `include/`, `source/`,
`tests/` and a `README.md` stating what it owns.

## Documentation

[`docs/README.md`](docs/README.md) is the manual's index. The entries worth knowing by name:

- [Getting started](docs/getting-started/introduction.md) — the tour, from an empty program.
- [Architecture](docs/architecture/README.md) — how each subsystem works.
- [Modules](docs/modules/README.md) — what each module owns.
- [Command line guide](docs/guides/command-line-interface.md) — every `se` command and flag.
- [Contributing](docs/CONTRIBUTING.md) — the conventions a change is held to.
- [Changelog](docs/reference/changelog.md) — what changed and when.

## License

Apache License, Version 2.0. See [`LICENSE`](LICENSE).
