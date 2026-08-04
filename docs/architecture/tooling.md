# Validation and tooling

This file covers how the engine is checked and driven: the functional test suite's shape and what
it pins, and the `se` developer CLI that resolves the toolchain and runs everything.

## 1. Validation and tooling

The engine ships no device code of its own, so there is nothing to test in isolation — a
meaningful test must instantiate kernels and run them against the real runtime. The suite in
`tests/` does exactly that: it follows SushiRuntime's layout (`unit/`, `integration/`,
`regression/`, a shared `common/` with the GoogleTest entry point and a process-wide runtime
fixture) and builds one binary, `sushiengine_functional_tests`, as a SYCL translation-unit set.
There are no mocks. Tests carry the `Unit_*` / `Integration_*` / `Regression_*` suite-name
prefixes, which `tests/CMakeLists.txt` turns into CTest labels so a sub-suite is one `ctest -L`
away. The integration tests re-run the sandbox and PGS claims as assertions (scalar-reference
agreement, `compile_count == 1`); the unit tests pin the host bookkeeping (entity directory,
swap-remove, command buffer, graph colouring).

Beyond the ECS/physics core, three areas are now covered directly. The **`engine/domain/astro/`
solar-system model** — pure double-precision maths with no device code — is verified against
astronomical reference landmarks (J2000 = JD 2451545.0, Earth ~1 AU from the Sun, WGS84
semi-major/minor radii, ~9.8 m/s² surface gravity, Earth's SOI ~0.9 Gm) and against structural
invariants that hold regardless of the exact arithmetic: Kepler's equation is satisfied by the
returned anomaly; the ecliptic↔equatorial, topocentric, body-equatorial, and scene-frame
transforms all round-trip to the identity; every basis is right-handed orthonormal; the
symplectic integrator conserves orbital energy over a full revolution; and `advance_astro_state`
keeps a low orbit bound and deterministic (`Unit_JulianDate`, `Unit_OrbitalElements`,
`Unit_CelestialBodies`, `Unit_Gravity`, `Unit_Topocentric`, `Unit_BodyOrientation`,
`Unit_SurfaceFrame`, `Unit_StarCatalog`, `Unit_SceneFrame`, `Unit_ReferenceFrame`,
`Integration_AstroDynamics`, `Integration_Ephemeris`).

The **camera/transform maths** in
`engine/foundation/core/include/SushiEngine/core/types.hpp` (quaternion rotation vs. its matrix
form, the reverse-Z infinite-far projection, `look_at`, `compose_transform`) is pinned in
`Unit_MathPrimitives`. The **determinism guard rails** — the seeded xorshift128+ RNG and the
fixed-timestep accumulator — are pinned in `Unit_RNG` (identical seeds replay identically; a
snapshot replays the future exactly, as rollback needs) and `Unit_FixedTimestep` (step count
depends only on total elapsed time, never on how it was chunked across frames).

The `se` developer CLI (`cli/`) is the counterpart to the runtime's `sr`: a thin Typer layer over
a service layer that issues the cmake/ctest calls. It owns no build knowledge the CMake does not
— its job is to resolve the toolchain the engine consumes (SushiRuntime's bundled clang++ and
vcpkg) and snapshot the MSVC environment on Windows, then drive configure/build/test/run. The
same one-way dependency holds: the CLI resolves the toolchain from the shared SushiStack
workspace's `dependencies/` tree (falling back to a runtime-local copy for backward
compatibility) but the engine never reaches back into runtime source.
