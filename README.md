# SushiEngine

A 3D game engine head built on top of [SushiRuntime](../sushiruntime).

SushiEngine is the head; SushiRuntime is a plugged-in component (the battery).
The engine owns the loop, the world, and — later — the window and renderer; the
runtime is a hardware-agnostic orchestration backbone the engine hands work to.
The dependency points one way only: `SushiEngine -> SushiRuntime`.

## Status — ECS layer (WP-3)

The entity / component / system core is in place: components live in archetype
chunks, systems declare the components they read and write, and the SushiRuntime
dependency tracker orders them — conflicting systems run in sequence, disjoint ones
in parallel — with no scheduler written in the engine. The schedule is compiled once
and replayed every frame; entities spawn and die through a deferred command buffer
with no per-frame recompile. The `sandbox` target is the worked example: a particle
world checked against an independent scalar reference.

Next: rendering (a window, The-Forge, Dear ImGui — render as an opaque host sink
node), then the editor host shell.

## Layout

- `include/SushiEngine/core/types.hpp` — the single seam for value types. Today it
  aliases a placeholder; when SushiBLAS lands, re-point it there in one file.
- `include/SushiEngine/ecs/` — the ECS: entities, components, archetype chunks, the
  world, the deferred command buffer, and the system schedule.
- `include/SushiEngine/SushiEngine.hpp` — umbrella header for the whole surface.
- `sandbox/main.cpp` — the worked example and the single SYCL translation unit.
- `tests/` — the GoogleTest functional suite (unit / integration / regression),
  run against the real runtime.
- `cli/` — the `se` developer CLI (build / test / run / diagnostics).

## Install

SushiEngine builds on top of the shared SushiStack workspace, which provides the
SYCL toolchain, vcpkg, and the SushiRuntime sibling. On a fresh machine one
command installs Python and Git if missing, clones the workspace, installs the
`ss` CLI, and downloads the shared dependencies:

```bash
curl -fsSL https://sushisystems.io/install.sh | bash      # Linux / WSL
```

```powershell
irm https://sushisystems.io/install.ps1 | iex             # Windows (PowerShell)
```

On Windows use `irm`, not `curl` — in PowerShell `curl` is an alias for
`Invoke-WebRequest` and does not pipe a script the same way.

Then add the modules and install this CLI through the umbrella:

```bash
ss add sushiruntime sushiengine   # clone both into the workspace
ss install-cli sushiengine        # install `se` (and `sushiengine`) via pipx
```

`ss install-cli` sets up an isolated pipx venv and wires in the shared
`sushicli` presentation layer. Add `--no-editable` for a non-developer install.

## Developer CLI

The `se` CLI wraps configure, build, test, and run so you do not type the long
cmake line by hand. It reads the SushiRuntime sibling's bundled clang++ and
vcpkg automatically, so a normal side-by-side checkout needs no configuration.

| Command | What it does |
|---|---|
| `se build [--type release\|debug\|relwithdebinfo] [--clean] [--no-test]` | Configure and build against the SushiRuntime sibling. Tests build by default. |
| `se test [--suite <label>] [--filter <regex>] [--repeat N]` | Run the test suite via CTest labels. `--suite all` runs everything. |
| `se run <target> [-- args…]` | Run a built executable (e.g. `se run sandbox`). Args after `--` are forwarded to it. `--sort` picks one interactively. |
| `se editor [--no-run]` | Build and launch the ImGui editor (configures with `SE_BUILD_EDITOR=ON`). |
| `se clean` | Remove the `build/` tree. |
| `se doxygen` | Generate Doxygen documentation. |
| `se config` | Print the resolved config and where each value came from. |
| `se env [--all]` | Print the environment cmake/ctest/run subprocesses execute under. |
| `se docker build [--no-cache] [--runtime-ref <ref>]` | Build the containerized dev image (toolchain + runtime sibling + CLI). |
| `se docker run [--admin] [--no-gpu]` | Start an interactive container with the engine source mounted live. |

The `--suite` labels are `functional` (default), `unit`, `integration`,
`regression`, and `all`.

Machine-specific paths (a non-standard runtime location, a scoop-installed
cmake) go in a gitignored `cli/config.local.toml`; see `cli/config.toml` for the
keys.

## Testing

Tests are off by default. Enable them with `-DSE_BUILD_TESTS=ON` (the CLI's
`se build` does this for you), then:

```
cmake --build build --target se_functional_tests
ctest --test-dir build --output-on-failure          # all tests
ctest --test-dir build -L 'unit|integration|regression'
```

GoogleTest is pulled from vcpkg, the same toolchain the runtime already requires.

## Building

Needs the SushiRuntime sibling checkout at `../sushiruntime` (override with
`-DSUSHIRUNTIME_DIR=...`) and its SYCL toolchain. Configure with the same compiler
the runtime uses, e.g. the bundled intel/llvm clang++:

```
cmake -S . -B build -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_COMPILER=<sushiruntime>/dependencies/toolchains/llvm-sycl/bin/clang++ \
  -DVCPKG_ROOT=<sushiruntime>/dependencies/vcpkg \
  -DCMAKE_TOOLCHAIN_FILE=<sushiruntime>/dependencies/vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build build --target sandbox
```

On Windows, configure from a Developer environment (vcvars) so the resource
compiler and MSVC libraries are on the path.
