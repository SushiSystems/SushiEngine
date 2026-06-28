# Contributing to SushiEngine

Thanks for your interest in improving SushiEngine. This document explains how to
get a working build, the conventions we expect in a change, and how to get a pull
request merged.

By contributing you agree that your contributions are licensed under the
project's **Apache License, Version 2.0** (see [`LICENSE`](LICENSE)).

> New to the codebase? Read [`docs/guides/ARCHITECTURE.md`](docs/guides/ARCHITECTURE.md)
> first — it explains the head/battery split, the layers, and how the engine hands
> simulation work to SushiRuntime as a task graph.

**Note:** SushiEngine is a head built on top of
[SushiRuntime](https://github.com/SushiSystems/SushiRuntime). The dependency points
one way only — `SushiEngine -> SushiRuntime` — and the runtime never depends on the
engine. A change that needs new runtime behavior belongs in the runtime, behind its
public API, not bolted onto the engine.

---

## 1. Getting set up

SushiEngine consumes SushiRuntime as a sibling checkout and reuses its SYCL
toolchain. Check both out side by side:

```
Projects/
  sushiruntime/
  sushiengine/
```

Configure with the same compiler the runtime uses (the bundled intel/llvm
`clang++` is the primary path) and point CMake at the runtime's vcpkg toolchain:

```bash
cmake -S . -B build -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_COMPILER=<sushiruntime>/dependencies/toolchains/llvm-sycl/bin/clang++ \
  -DVCPKG_ROOT=<sushiruntime>/dependencies/vcpkg \
  -DCMAKE_TOOLCHAIN_FILE=<sushiruntime>/dependencies/vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build build --target sandbox
```

Override the runtime location with `-DSUSHIRUNTIME_DIR=...` when it is not at
`../sushiruntime`. On **Windows**, configure from a Developer environment
(`vcvars64.bat`) so the resource compiler and MSVC libraries are on the path.

---

## 2. Before you start coding

- **Open an issue first** for anything beyond a small fix, so the approach can be
  agreed before you invest time.
- **Keep changes focused.** One logical change per pull request. A refactor and a
  behavior change in the same PR are hard to review and hard to revert.
- **Respect the head/battery boundary.** The engine depends on the runtime, never
  the other way around. If a feature only composes the runtime's public API, it
  belongs in the engine; if it needs the runtime to do something new, raise it in
  SushiRuntime.

---

## 3. Building and testing your change

```bash
cmake --build build --target sandbox     # the example game / smoke target
./build/sandbox                          # runs the headless simulation check
```

Expectations for a mergeable change:

1. **The tree builds clean** with the primary intel-llvm toolchain, warnings and
   all (`-Wall -Wextra` are inherited from the runtime's flags).
2. **New behavior ships with a check.** Until a dedicated test tree exists, a new
   system or component must be exercised by the `sandbox` (or a small target
   beside it) with a deterministic assertion, the way the Milestone A integrator
   is checked against its closed-form result.
3. **Determinism holds.** Same input, same architecture, same result — a change
   that makes a simulation step non-deterministic needs a stated reason.

---

## 4. Coding style

The codebase shares SushiRuntime's deliberate, consistent style. Match the file
you're editing.

- **Allman braces.** Opening brace on its own line — functions, types, lambdas,
  control blocks. Trivial one-line accessors (`int x() const { return x_; }`) may
  stay compact.
- **Nested namespaces, Allman.** Write `namespace A\n{\n namespace B\n{` — not
  `namespace A::B`.
- **C++17.** The codebase targets C++17, the same standard SushiRuntime compiles
  on across every toolchain. Do not reach for C++20/23 facilities.
- **Comments are few and useful.** A short plain-English line that explains *why*,
  not a restatement of the code. Delete redundant narration.
- **No historical references in comments or Doxygen.** Do not mention the PR,
  issue, contributor, or past implementation that a change replaced — those belong
  in the commit message. A comment that says "we used to do X" or "fixed in #123"
  rots immediately and misleads future readers.
- **No separator comments.** Lines like `// <-----------`, `// =========`, or
  `// --------` are forbidden everywhere in the codebase.
- **Naming.** Types `PascalCase`, functions/variables `snake_case`, members
  trailing underscore (`impl_`), constants `UPPER_SNAKE`. No abbreviated type
  names — spell `Statistics`, not `Stats`. Follow the surrounding code.
- **Every source file carries the Apache 2.0 license header** used across the
  tree. Copy it verbatim into new files.

### Doxygen documentation

Every public function carries a Doxygen comment that answers two questions in
plain English: **why** the function exists (what problem it solves for the
caller) and **how** it does it (the mechanism, in one or two technical lines).
Then document **every** parameter and the return value — leave nothing open.

Keep it tight. The rule is completeness, not length: technical, objective, plain
English, no filler and no restating the obvious. If a `@param` only repeats the
parameter name, the line is noise — say what the value *means* or constrains.

The tags, in order:

- `@brief` — one sentence on *why* the function exists.
- One short line on *how* it works or what to watch for — only when the mechanism
  is not obvious from the brief.
- `@param` for each parameter — its meaning, units, ownership, or valid range.
- `@return` for the result — what it is and what each outcome signals.
- `@throws` / `@warning` only when a caller must know about failure or a
  precondition. Do not add them by reflex.

---

## 5. Documentation is part of the change

Documentation is not a follow-up task — it ships in the same pull request as the
code it describes. If your change makes any existing sentence false, fixing that
sentence is part of the change.

Treat the following as hard requirements, not suggestions:

- **Every user-visible change updates [`docs/CHANGELOG.md`](docs/CHANGELOG.md).**
  Add an entry under `## [Unreleased]` in the right group (`Added` / `Changed` /
  `Fixed` / `Removed` / `Deprecated`).
- **A new or changed feature updates the guides.** When you add, rename, or change
  behavior, update [`docs/guides/ARCHITECTURE.md`](docs/guides/ARCHITECTURE.md) —
  every class, system, and concept it names must still exist — and the top-level
  [`README.md`](README.md) when the change touches what the project *is* or how a
  first-time reader gets started.
- **Public API carries Doxygen.** New or changed public functions follow §4. The
  header *is* documentation.

The PR description must say which docs you touched, or state explicitly that the
change is doc-invisible (an internal refactor with no API or behavior change).
"Docs later" is not accepted.

---

## 6. Commits and pull requests

### Branch names

Branch off the current `main` with a type prefix and a short, lowercase,
hyphenated description:

```
feature/soa-component-archetypes
fix/integrator-state-swap-on-skip
chore/bump-runtime-pin
docs/architecture-render-seam
```

Use `feature/` for new functionality, `fix/` for bug fixes, `chore/` for
build/tooling upkeep, `docs/` for documentation-only work.

### Commit headers

Every commit header is `type: short summary`:

- **Prefix** with the type: `feat:`, `fix:`, `chore:`, `docs:`, `test:`,
  `refactor:`. Add a scope when it sharpens the line: `feat(ecs):`, `fix(sim):`.
- **Lowercase** the summary, imperative mood, no trailing period. Short and
  precise — say what the commit does, not how it feels.

```
feat(ecs): add archetype-based component storage
fix(sim): keep the integrate node's read/write keys after a resize
docs: document the render sink-node seam
```

### Commit body and PR description

When a commit or PR needs explaining, write the body as bullet points. Each bullet
starts with a past-tense verb describing what the change did, then the object:

```
- Added a structure-of-arrays World keyed by component type.
- Changed the simulation to build its graph once and replay it per frame.
- Removed the per-step buffer reallocation from the loop.
```

Plain English, technical, and objective. State the mechanism and the effect — no
marketing, no adjectives selling the change.

The PR description states **what** changed, **why**, and **how you tested it**,
names the docs you touched (see §5), and links the issue. Rebase onto the current
`main` before opening the PR; keep history clean.

---

## 7. Reporting bugs and requesting features

Use the GitHub issue templates. A good bug report includes the platform and
toolchain, the build type, the exact command, and the smallest reproducer you can
manage. For anything that looks like a **security** issue, do **not** open a
public issue — follow [`SECURITY.md`](SECURITY.md).

---

## 8. Questions

Open a GitHub Discussion, join the community Discord (see [`SECURITY.md`](SECURITY.md)
for the current invite), or email **mustafagarip@sushisystems.io**.

Please be respectful in all project spaces — see
[`CODE_OF_CONDUCT.md`](CODE_OF_CONDUCT.md).
