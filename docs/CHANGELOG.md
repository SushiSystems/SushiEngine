# Changelog

All notable changes to SushiEngine are documented here.

Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) — versions follow [Semantic Versioning](https://semver.org/).

---

## [Unreleased]

### Added
- Test suite (`tests/`): a GoogleTest-based functional suite mirroring
  SushiRuntime's layout, gated behind the `SE_BUILD_TESTS` CMake option (OFF by
  default; GoogleTest comes from vcpkg). One binary, `se_functional_tests`, built
  as a SYCL translation-unit set so kernels run against the real runtime — no
  mocks. Tests are grouped into CTest labels (`unit` / `integration` /
  `regression`) from the GTest suite-name prefix convention (`Unit_*` /
  `Integration_*` / `Regression_*`), so `ctest -L unit` selects a sub-suite.
  Coverage: World entity directory mechanics (spawn/get/destroy, generation
  invalidation, swap-remove repointing, structure versioning, queries),
  CommandBuffer deferral, graph-colouring properties, and end-to-end Schedule and
  PGS-solver runs checked against scalar references (`compile_count == 1`).
- Developer CLI (`cli/`): a Typer/Rich command-line tool installed as `se` (and
  `sushiengine`), mirroring SushiRuntime's `sr`. Core surface: `se project
  build/test/run/clean/doxygen`, `se config show`, and `se env dump`. It consumes
  the SushiRuntime sibling's bundled clang++ and vcpkg rather than provisioning a
  toolchain — when `cxx` / `vcpkg_root` are unset they resolve from the runtime's
  `dependencies/` tree, so a normal checkout needs no local config. Layered
  configuration (`cli/config.toml` -> `config.local.toml` -> `SE_*` env vars) and
  a cached MSVC-environment snapshot on Windows.
- Physics constraint solver (substrate-plan WP-3, physics half): a Projected
  Gauss-Seidel solver built on graph colouring. `color_constraints` edge-colours the
  constraints so each colour is a conflict-free batch (no two constraints in a colour
  share a body); `ConstraintSolver` compiles each colour into one parallel task and
  lets the runtime's dependency tracker order the colours into a sequential sweep —
  Gauss-Seidel across colours, parallel within one — repeated for the iteration count
  and compiled once. The solver is generic over the constraint type and its
  projection (a new constraint type is its own POD plus projection), with
  `DistanceConstraint` / `DistanceProjection` provided. The `pgs_demo` example solves
  a hanging chain and checks the device result against a scalar reference running the
  same colours in the same order (max error ~1e-6, compile_count == 1).
- ECS layer (substrate-plan WP-3): the entity / component / system core on top of
  SushiRuntime. Entities are generation-checked handles; components live in
  archetype chunks of structure-of-arrays columns, each column its own runtime
  allocation so the dependency tracker keys on it. A system declares its component
  reads and writes (`Read<T>` / `Write<T>`) and the `Schedule` emits one graph node
  per chunk; the runtime's dependency tracker orders systems by access — conflicting
  ones run in order, disjoint ones (and disjoint chunks) in parallel — so the engine
  writes no scheduler. Node iteration counts are late-bound to each chunk's live
  entity count, so the graph is compiled once and replayed while entities spawn and
  die. Structural changes go through a `CommandBuffer` applied at the frame barrier,
  with O(1) swap-remove destroys; new chunk sets are the only trigger for a rebuild,
  reported by the world's `structure_version`.
- `sandbox` target: the WP-3 worked example and single SYCL translation unit. A
  particle world (Position, Velocity, Mass, Lifetime) driven by `apply_forces`,
  `integrate`, and a parallel `decay_lifetime` system, spawning and destroying
  entities every frame, checked against an independent scalar reference with the
  graph compiled exactly once.
- `core/types.hpp`: the single integration seam for value types. It aliases a
  temporary placeholder today and is the one file to re-point at SushiBLAS when
  that library lands.
- Project governance and docs: `CONTRIBUTING.md`, `SECURITY.md`,
  `CODE_OF_CONDUCT.md`, and `docs/guides/ARCHITECTURE.md`, mirroring SushiRuntime's
  conventions.
