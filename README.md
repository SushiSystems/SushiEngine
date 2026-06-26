# SushiEngine

A small, debloated game engine head built on top of [SushiRuntime](../sushiruntime).

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
