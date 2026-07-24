# Security Policy

We take the security of SushiEngine seriously. This document explains how to
report a vulnerability and what to expect in return.

## Reporting a vulnerability

**Please do not open a public GitHub issue for a security problem.** Public
disclosure before a fix is available puts every user at risk.

Instead, report it privately through one of:

- **Email (preferred):** **mustafagarip@sushisystems.io**
- **Discord:** reach the maintainer privately on the community server —
  invite: `https://discord.gg/y5639YuaPh`

If you prefer, you may also use GitHub's **private vulnerability reporting**
("Report a vulnerability" under the repository's Security tab).

### What to include

A good report lets us reproduce and assess quickly:

- A clear description of the issue and its impact.
- The affected component (e.g. the world store, the simulation graph build, the
  asset loader) and the version / commit hash.
- Step-by-step reproduction, ideally with a minimal proof of concept.
- The platform, SYCL toolchain (oneAPI / intel-llvm), and build configuration.
- Any suggested mitigation, if you have one.

## What to expect

- **Acknowledgement** within **72 hours**.
- An initial assessment and severity classification within **7 days**.
- Regular updates while we work on a fix, and credit in the release notes once a
  fix ships (unless you ask to remain anonymous).

Please give us a reasonable window to release a fix before any public
disclosure. We are happy to coordinate timing with you.

## Scope and threat model

A few properties are intentional design decisions rather than vulnerabilities —
please keep them in mind:

- **The engine executes the game you build with it.** Gameplay is C++ compiled
  into the application; an engine integration is not a sandbox. We are interested
  in memory-safety and concurrency defects *inside* the engine (use-after-free,
  out-of-bounds in the component store, races in the frame loop), not in code a
  game author chooses to run.
- **The compute backbone lives in SushiRuntime.** SushiEngine is a head built on
  top of [SushiRuntime](https://github.com/SushiSystems/SushiRuntime). A defect in
  the runtime's scheduler, memory pool, dependency tracker, or distributed
  protocol is in scope **there** — report it under SushiRuntime's `SECURITY.md`.
  Report against SushiEngine when the engine layer itself is at fault (the world
  store, the simulation graph it builds, asset handling, the editor/renderer).

In scope, and especially valued:

- Memory-safety bugs in the engine layer (the component store, the loop, asset or
  scene parsing once it exists).
- Data races or ordering bugs in the engine's frame loop that corrupt state.
- Parsing / deserialization flaws in any scene or asset format the engine loads.

## Supported versions

SushiEngine is pre-1.0 and under active development. Security fixes are applied to
the latest `main`; there is no long-term support branch yet. Always test against
the current `main` before reporting.

## Contact

For anything that isn't sensitive, a GitHub issue or Discussion is fine. For
security matters, use the private channels above or email
**mustafagarip@sushisystems.io**.
