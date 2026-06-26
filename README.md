# SushiEngine

A small, debloated game engine head built on top of [SushiRuntime](../sushiruntime).

SushiEngine is the head; SushiRuntime is a plugged-in component (the battery).
The engine owns the loop, the world, and — later — the window and renderer; the
runtime is a hardware-agnostic orchestration backbone the engine hands work to.
The dependency points one way only: `SushiEngine -> SushiRuntime`.

## Status — Milestone A (headless core)

Proves the loop with no rendering: a structure-of-arrays component store whose
positions are integrated by velocity each fixed timestep, expressed as a single
SushiRuntime graph node and replayed every step. The `sandbox` target spawns a
million entities, runs the sim, and reads positions back to check determinism and
print the per-step cost.

Next: Milestone B adds a window, The-Forge rendering, and Dear ImGui (render as an
opaque host sink node); Milestone C adds the editor host shell.

## Layout

- `include/SushiEngine/core/types.hpp` — the single seam for value types. Today it
  aliases a placeholder; when SushiBLAS lands, re-point it there in one file.
- `include/SushiEngine/ecs/` — components and the SoA `World` store.
- `include/SushiEngine/sim/simulation.hpp` — builds and replays the sim graph.
- `include/SushiEngine/app/application.hpp` — owns the runtime, world, and loop.
- `sandbox/main.cpp` — the example game and the single SYCL translation unit.

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
