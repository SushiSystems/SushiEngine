# Changelog

All notable changes to SushiEngine are documented here.

Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) — versions follow [Semantic Versioning](https://semver.org/).

---

## [Unreleased]

### Added
- Milestone A (headless core): the first engine slice on top of SushiRuntime. A
  structure-of-arrays `World` stores entity components in runtime memory (position
  as a double-buffered `State`, velocity as a `Buffer`); `Simulation` builds one
  integrate node (`position_next = position + velocity * dt`) as a SushiRuntime
  graph, compiled once and replayed every fixed timestep; `Application` owns the
  runtime, the world, and the loop, keeping control of stepping in the engine (the
  head) while the runtime acts as the plugged-in compute backbone (the battery).
- `sandbox` target: the example game and single SYCL translation unit. Spawns a
  million entities under constant velocity, runs the simulation, and reads
  positions back to check the result against its closed-form value and report the
  per-step cost.
- `core/types.hpp`: the single integration seam for value types. It aliases a
  temporary placeholder today and is the one file to re-point at SushiBLAS when
  that library lands.
- Project governance and docs: `CONTRIBUTING.md`, `SECURITY.md`,
  `CODE_OF_CONDUCT.md`, and `docs/guides/ARCHITECTURE.md`, mirroring SushiRuntime's
  conventions.
