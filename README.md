# SushiEngine

A C++17 game engine built on top of [SushiRuntime](../sushiruntime).

SushiEngine depends on SushiRuntime and never the other way around. SushiRuntime
provides hardware discovery, USM allocation, the SYCL task graph, and the
dependency tracker. SushiEngine provides the frame loop, the entity-component-system
(ECS), a physics constraint solver, and an editor shell. The engine ships no SYCL
device code of its own; kernels are instantiated in the consuming translation unit
(for example `sandbox/main.cpp`), which links against the runtime.

## What currently exists

- **ECS** (`include/SushiEngine/ecs/`, header-only). Entities are
  generation-checked handles. Components are stored in archetype chunks of
  structure-of-arrays columns. A system declares which components it reads and
  writes; the schedule emits one graph node per chunk and the runtime's dependency
  tracker orders them. The graph is compiled once and replayed each frame; it is
  only rebuilt when a new chunk is allocated. Structural changes (spawn/destroy) go
  through a command buffer applied at a frame barrier.
- **Physics** (`include/SushiEngine/physics/`). A Projected Gauss-Seidel constraint
  solver parallelised by graph colouring. Generic over the constraint type;
  `DistanceConstraint` is provided.
- **Editor** (`editor/`). An SDL2 + Dear ImGui desktop application that presents
  through the engine's Vulkan renderer, with a docking layout: Hierarchy, Inspector,
  Project browser, Text Editor, Toolbar, Console, and Statistics panels. The window,
  the Vulkan presentation, and the ImGui/Vulkan glue sit behind narrow seams
  (`platform_window.hpp`, `render/window_renderer.hpp`, `imgui_backend.*`). It
  operates on an editor-owned scene model. Turning the editor on builds the renderer
  too; it needs SDL2 with its `[vulkan]` feature. A **Scene** panel shows a
  Vulkan-rendered 3D viewport navigated with a Unity-style fly camera — right-mouse
  look plus WASD/QE and Shift to boost. The viewport draws a **live ECS world**
  ticked on SushiRuntime: a ring of spinning, orbiting cubes, ticked only while the
  toolbar is Playing. The world lives behind a plain-C++ simulation seam
  (`include/SushiEngine/sim/simulation.hpp`), implemented by the `sushi_sim` library
  that contains the runtime, ECS, and SYCL — so the editor depends only on the
  abstraction. A **Game** view renders the same world from the world's own camera,
  tabbed next to the Scene view like Unity; the camera each viewport uses sits behind
  an `ISceneCamera` seam, so one panel serves both.
- **`se` CLI** (`cli/`). A Python/Typer tool that wraps configure/build/test/run.

## Repository layout

- `include/SushiEngine/core/types.hpp` — the single alias point for value types
  (`Vec3`, `Scalar`), currently a placeholder.
- `include/SushiEngine/ecs/` — entities, components, archetype chunks, world,
  command buffer, schedule.
- `include/SushiEngine/physics/` — PGS solver and graph colouring.
- `include/SushiEngine/SushiEngine.hpp` — umbrella header.
- `sandbox/main.cpp` — ECS worked example; a single SYCL translation unit. Runs the
  ECS and an independent scalar reference in parallel and asserts they match.
- `examples/pgs_demo.cpp` — physics solver demo (a hanging chain).
- `editor/` — the ImGui editor shell.
- `sim/` — the `sushi_sim` library: the runtime-backed live world behind the
  plain-C++ `ISimulation` seam (`include/SushiEngine/sim/`).
- `tests/` — GoogleTest functional suite (unit / integration / regression), run
  against the real runtime.
- `cli/` — the `se` developer CLI.

## Requirements

- The **SushiStack** repository, which is the workspace and holds the shared
  `dependencies/` tree: the intel/llvm SYCL clang++ toolchain, vcpkg, cmake, ctest,
  doxygen, and pkgconf. Everything lives inside this repo — there are no
  system-wide dependencies to install.
- The **SushiRuntime** sibling checked out next to this repository (at
  `../sushiruntime`, or overridden with `-DSUSHIRUNTIME_DIR=...`).
- On Windows, a Developer Command Prompt (vcvars) so MSVC and the resource compiler
  are on the path. The `se` CLI snapshots this environment for you.

## Setup

The toolchain and dependencies are not installed system-wide; they live inside the
SushiStack repository, which is the workspace. Get it with the umbrella installer,
then add the two repositories:

```powershell
irm https://sushisystems.io/install.ps1 | iex   # Windows (PowerShell)
```

```bash
curl -fsSL https://sushisystems.io/install.sh | bash   # Linux / WSL
```

The installer clones the workspace into `sushistack`, installs the `ss` CLI, and
downloads the shared `dependencies/` tree. Then:

```
ss add sushiruntime sushiengine    # clone both repos into the workspace
ss install-cli sushiengine         # install `se` (and `sushiengine`) via pipx
```

If the SushiStack repository is already cloned, you only need the two
repositories side by side and the `se` CLI installed.

### Machine-specific paths

The CLI resolves the compiler and vcpkg from the runtime/stack automatically. When
the toolchain lives somewhere the defaults do not expect, put absolute paths in a
gitignored `cli/config.local.toml` next to `cli/config.toml`, using the same section
layout. On this machine the tools resolve from `D:/Projects/sushistack/dependencies`:

```toml
[tool.windows]
cmake_exe   = "D:/Projects/sushistack/dependencies/tools/cmake/bin/cmake.EXE"
ctest_exe   = "D:/Projects/sushistack/dependencies/tools/cmake/bin/ctest.EXE"
doxygen_exe = "D:/Projects/sushistack/dependencies/tools/doxygen/doxygen.EXE"
pkgconf_exe = "D:/Projects/sushistack/dependencies/vcpkg/installed/x64-windows/tools/pkgconf/pkgconf.exe"
```

Run `se config` to print the resolved values and where each came from.

## The `se` CLI

`se` wraps the cmake/ctest calls so you do not type the long configure line by hand.

| Command | What it does |
|---|---|
| `se build [--type release\|debug\|relwithdebinfo] [--clean] [--no-test]` | Configure and build against the SushiRuntime sibling. The test suite builds by default; `--no-test` sets `SE_BUILD_TESTS=OFF`. `--clean` deletes the build tree first. |
| `se test [--suite <label>] [--filter <regex>] [--repeat N]` | Run the suite via CTest labels. Labels: `functional` (default), `unit`, `integration`, `regression`, `all`. `--filter` is a `ctest -R` regex over `Suite.Case` names. `--repeat` re-runs until failure. |
| `se run <target> [-- args…]` | Run a built executable (for example `se run sandbox`). Arguments after `--` are forwarded. `--sort` picks one interactively. |
| `se editor [--no-run]` | Build the ImGui editor (configures with `SE_BUILD_EDITOR=ON`) and launch it. `--no-run` builds only. |
| `se clean` | Remove the `build/` tree. |
| `se doxygen` | Generate Doxygen documentation. |
| `se config` | Print the resolved config and each value's source. |
| `se env [--all]` | Print the environment build/run subprocesses execute under. |
| `se docker build [--no-cache] [--runtime-ref <ref>]` | Build the containerized dev image (toolchain + runtime + CLI). |
| `se docker run [--admin] [--no-gpu]` | Start an interactive container with the source mounted live. |

After changing toolchain paths in `config.local.toml`, delete the `build/` tree
(`se clean`) before reconfiguring — cmake bakes the old paths into its cache.

## Building without the CLI

```
cmake -S . -B build -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_COMPILER=D:/Projects/sushistack/dependencies/toolchains/llvm-sycl/bin/clang++ \
  -DVCPKG_ROOT=D:/Projects/sushistack/dependencies/vcpkg \
  -DCMAKE_TOOLCHAIN_FILE=D:/Projects/sushistack/dependencies/vcpkg/scripts/buildsystems/vcpkg.cmake

cmake --build build --target sandbox     # ECS worked example
cmake --build build --target pgs_demo    # physics demo
```

Run the sandbox to validate the ECS; it exits 0 on success:

```
./build/sandbox.exe
```

The editor is behind `-DSE_BUILD_EDITOR=ON` and the tests behind
`-DSE_BUILD_TESTS=ON` (both off by default; `se editor` and `se build` set them).

## Testing

Tests are off by default. Enable and run them:

```
cmake --build build --target se_functional_tests
ctest --test-dir build --output-on-failure
ctest --test-dir build -L 'unit|integration|regression'
```

GoogleTest comes from vcpkg. The suite instantiates real kernels and runs them
against the runtime — there are no mocks.

## Documentation

- `docs/guides/ARCHITECTURE.md` — how the layers fit together and the seams a
  cross-cutting change touches.
- `docs/CHANGELOG.md` — notable changes.
