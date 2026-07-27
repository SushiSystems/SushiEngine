# Contributing to SushiEngine

Thanks for your interest in improving SushiEngine. This document explains how to
get a working build, the conventions we expect in a change, and how to get a pull
request merged.

By contributing you agree that your contributions are licensed under the
project's **Apache License, Version 2.0** (see [`LICENSE`](LICENSE)).

> New to the codebase? Read [`ARCHITECTURE.md`](ARCHITECTURE.md)
> first — it explains the head/battery split, the layers, and how the engine hands
> simulation work to SushiRuntime as a task graph.

**Note:** SushiEngine is a head built on top of
[SushiRuntime](https://github.com/SushiSystems/SushiRuntime). The dependency points
one way only — `SushiEngine -> SushiRuntime` — and the runtime never depends on the
engine. A change that needs new runtime behavior belongs in the runtime, behind its
public API, not bolted onto the engine.

---

## 1. Getting set up

SushiEngine consumes SushiRuntime as a sibling checkout and shares the
**SushiStack** workspace's toolchain — the intel/llvm `clang++`, vcpkg, CMake,
and CTest live in SushiStack's `dependencies/` tree, not inside either engine
repo. Check both engine repos out side by side, get SushiStack, then install
the CLI:

```
Projects/
  sushistack/     # workspace: dependencies/ (toolchains, vcpkg, cmake, ctest)
  sushiruntime/
  sushiengine/
```

```bash
ss add sushiruntime sushiengine    # clone both engine repos into the workspace
ss install-cli sushiengine         # install `se` / `sushiengine`
```

Every build and program action goes through the `se` CLI (per this repo's
`CLAUDE.md`) — never invoke `cmake`/`ninja` directly:

```bash
se build            # configure and build against the SushiRuntime sibling
se run sandbox       # the worked ECS example
```

Override the runtime location with `-DSUSHIRUNTIME_DIR=...` (or the
`SUSHIRUNTIME_DIR`/`SUSHISTACK_HOME` environment variables) when it is not at
`../sushiruntime`. On **Windows**, `se` snapshots a Developer environment
(`vcvars64.bat`) for you so the resource compiler and MSVC libraries are on
the path. See [README.md](README.md) for the full requirements list and the
raw `cmake` invocation if you need to skip the CLI.

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
se build                      # builds se_functional_tests by default
se test --suite functional     # unit + integration + regression
se run sandbox                 # the ECS worked example, exits 0 on success
```

Expectations for a mergeable change:

1. **The tree builds clean** with the primary intel-llvm toolchain, warnings and
   all (`-Wall -Wextra -Werror -pedantic` are inherited from the runtime's flags).
2. **New behavior ships with a test.** Add a GoogleTest case under
   `tests/functional/unit/`, `integration/`, or `regression/` (see
   [README.md](README.md) for the exact layout and CTest label scheme) —
   a device kernel checked against an independent scalar/host reference, the
   way the existing ECS, physics, and loop tests are structured. A new example
   or demo under `examples/`/`sandbox/` is a useful illustration but does not
   substitute for a test.
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
- **Namespaces are `PascalCase`.** `SushiEngine::Simulation`, `SushiEngine::Editor`,
  `SushiEngine::Render`, `SushiEngine::Render::Vulkan` — never a lowercase or
  abbreviated segment (`sim`, `sushi::editor`, `vulkan`). A namespace name follows
  the same abbreviation and acronym rules as a type name (below).
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
  trailing underscore (`impl_`), constants `UPPER_SNAKE`. Follow the surrounding
  code.
- **No abbreviations, anywhere an identifier is spelled.** Spell the word out in
  full: `Statistics` not `Stats`, `Vector3` not `Vec3`, `Quaternion` not `Quat`,
  `Simulation` not `Sim`. This applies to types, namespaces, functions, variables,
  and file names alike.
- **Well-known acronyms are the exception, and stay fully upper-case.** An
  initialism that is more recognizable as an acronym than spelled out keeps its
  acronym casing even inside a `PascalCase` identifier or namespace segment:
  `SushiEngine::API`, `PGSSolver`, `RHIDevice`. Do not title-case an acronym
  (`Api`, `Pgs`) and do not invent new acronyms for the sake of brevity — this
  exception is for terms the reader already knows as an acronym (API, PGS, RHI,
  GPU, ECS, XPBD), not a shortcut around the no-abbreviation rule.
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

- **Every user-visible change updates [`CHANGELOG.md`](CHANGELOG.md)**, under
  `## [Unreleased]` in the right group (`Added` / `Changed` / `Fixed` /
  `Removed` / `Deprecated`). The project has not yet been assigned a semantic
  version — we are in prototype stage — so entries accumulate under
  `[Unreleased]` until the project cuts its first tagged release; a
  `## [1.0.0]` (or later) heading is only opened once that release actually
  happens.
- **A changelog entry is a short bullet, not an essay.** Same shape as a
  commit/PR body (§6): one line, a past-tense verb, then the object — "Added
  X", "Fixed Y", "Changed Z to do W." One or two supporting sub-bullets are
  fine for a change with real user-facing parts; a multi-paragraph writeup
  belongs in the PR description or `ARCHITECTURE.md`, not the changelog.
- **A new or changed feature updates the guides.** When you add, rename, or change
  behavior, update [`ARCHITECTURE.md`](ARCHITECTURE.md) —
  every class, system, and concept it names must still exist — and
  [`INTRODUCTION.md`](INTRODUCTION.md) (the getting-started tour and its code
  samples must compile against the current API), and the top-level
  [`README.md`](README.md) when the change touches what the project *is* or how a
  first-time reader gets started.
- **Any CLI change updates the CLI docs.** If you add, rename, or change an
  `se` / `sushiengine` command, flag, or its output, update
  [`CLI_GUIDE.md`](CLI_GUIDE.md) — the full command reference — *and* the
  build/test snippets in this file and the README that invoke it. Command help
  text (Typer `--help`) and the guide must agree.
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
