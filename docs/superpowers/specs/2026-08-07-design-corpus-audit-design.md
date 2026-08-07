# Design corpus audit — verifying every claim in `docs/design/` against the tree

**Status:** designed, 2026-08-07.

`docs/design/` holds eighteen engineering plans, 19,079 lines between them. Each opens with a
status line and names the roadmap section that records its per-phase progress. Nothing checks
either against the tree, so a phase marked done stays marked done whether or not it was built,
and a path cited as evidence stays cited whether or not the file still exists.

This document specifies the audit that closes both gaps, and the permanent check that stops
them reopening.

The subject is the corpus, not the engine. This pass changes documentation to match the tree.
It does not change the tree to match documentation, and it does not rewrite the design
reasoning those documents carry — `docs/design/README.md` states that the corpus is a record of
intent rather than a description of the tree, so a plan describing unbuilt work is correct by
construction. The defect this audit hunts is narrower: a document that claims work is *done*
when it is not, or cites evidence that no longer resolves.

Companion documents: [`docs/documentation-style-guide.md`](../../documentation-style-guide.md),
whose "a path in prose is a real path" rule this audit enforces for the first time, and
[`docs/CONTRIBUTING.md`](../../CONTRIBUTING.md) §5, which makes documentation part of a change.

---

## 1. Audit — what exists today, and where it stops

- **Two checks already run over `docs/`, and neither covers this.**
  `tools/documentation/check_documentation_length.py` enforces the length ceilings and
  resolves markdown links, anchor included. `tools/documentation/check_module_documentation.py`
  fails a build that adds a module without a `README.md`. Neither looks at a backticked path,
  which is the form every claim in `docs/design/` actually takes.

- **The corpus states its own evidence standard and cannot meet it.**
  `docs/design/static_mesh_authoring.md` §1 writes the rule plainly — a claim is a `file:line`,
  not a description — and `docs/documentation-style-guide.md` repeats it as "a path in prose is
  a real path. Verify it before you write it." That same guide already records the failure:
  "the restructure left several hundred stale paths behind precisely because nobody did."

- **836 unique paths are cited across the corpus, in 1,283 places. 720 of them do not
  resolve.** Measured 2026-08-07 by suffix-matching each cited path against the 1,214 tracked
  files outside `build*/`, `docs/`, `third_party/` and `cooked/`:

  | Class | Unique paths | Share | What it means |
  | --- | ---: | ---: | --- |
  | Resolves today | 116 | 13.9% | Correct as written |
  | One tree file ends with the cited path | 525 | 62.8% | RESTRUCTURE0 moved the file; the repair is unambiguous |
  | One tree file carries the cited base name | 100 | 12.0% | The directory was renamed too; the repair is unambiguous |
  | Several tree files match | 24 | 2.9% | `editor/main.cpp` matches nine files; needs a decision |
  | No file of that name anywhere | 71 | 8.5% | Three different situations, mixed together |

- **The 8.5% is the only class that carries signal, and it is not homogeneous.** It holds
  citations into the sibling SushiRuntime checkout and into system headers
  (`ENGINE_BACKBONE_REFACTOR.md`, `SushiRuntime.h`, `vulkan.h`, `vk_mem_alloc.h`), which are
  legitimate external references; run-time artifacts that a repository correctly does not carry
  (`boot.json`, `preferences.json`, `controller.json`, `vcpkg.json`); paths that predate the
  documentation reorganization (`docs/CHANGELOG.md`, `docs/glossary.md`, `ARCHITECTURE.md`);
  and files a document names while claiming the work that would contain them is finished. Only
  the last group is a false claim.

- **Three surfaces derive from the corpus and are already out of step with it.**
  `docs/reference/glossary.md` records the VFX family as `VFX1`–`VFX6` while
  `docs/design/vfx_particle_system.md` reports through `VFX7`, and it registers no family at all
  for `model_import.md`, `prefab_system.md`, `static_mesh_authoring.md` or
  `project_selection.md`. `docs/design/README.md` lists `model_import.md` and `prefab_system.md`
  as `designed`, but the last fifteen commits on `main` implement both.

---

## 2. What the audit produces

Five stages, run in order. The order is a requirement, not a preference: stage D4 is a large
mechanical diff and stage D3 is a small semantic one, and combining them would bury every
judgement D3 records under several hundred path rewrites.

### D1 — the evidence base

`tools/documentation/check_design_citations.py`, beside the two checks that already govern
`docs/`, following their conventions: run from the repository root, print every violation, exit
non-zero on the first category that has any.

It extracts each backticked token that names a repository path, resolves it against the tree,
and classifies the result into the five classes of §1. Paths that a repository does not carry
are declared in an explicit allow list inside the script, with the reason each is allowed, so an
external reference is distinguishable from an unwritten file rather than both being silently
tolerated.

The check is permanent. A one-off repair leaves the corpus correct for one day; a check leaves
it correct.

### D2 — the roadmap verdicts

For each of the eighteen documents, its roadmap section is read and every phase it marks
complete is tested against the tree by locating the symbol, file, command or test that phase
names. Each phase receives one verdict:

| Verdict | Meaning |
| --- | --- |
| `CONFIRMED` | What the phase names exists and is reachable from a build target |
| `OVERSTATED` | The phase is marked done; the tree carries none of it, or only part |
| `STALE` | The work exists under a different name or in a different place than claimed |
| `UNVERIFIABLE-HERE` | Confirming the claim requires running the engine on a GPU |

`UNVERIFIABLE-HERE` is a real verdict and not a hedge. This audit does not build or run the
engine, so a claim that rests on what appears on screen is recorded as unconfirmed rather than
assumed either way.

### D3 — the semantic synchronization

Status lines and roadmap markers are corrected wherever D2 returned a verdict other than
`CONFIRMED`, and the three derived surfaces are brought back into agreement:
`docs/design/README.md`, `docs/reference/glossary.md`, `docs/architecture/roadmap.md`.

### D4 — the mechanical repair

The 625 unambiguous citations are rewritten to the path that resolves. The 24 ambiguous and 71
unresolved ones are decided individually, each becoming a corrected path, an allow-list entry
with its reason, or a deletion where the citation names a file the plan has not yet produced.

### D5 — the consolidated backlog

One table of every phase across all eighteen documents that is not `CONFIRMED` complete, with
the document that owns it, and a proposed order of work. The order is a proposal; the project
owner sets it.

---

## 3. What this pass deliberately does not do

- **No engine code changes.** A false claim is corrected in the document, and the work it
  claimed becomes a backlog entry. Building the missing work is a separate change with its own
  plan.
- **No rewriting of design reasoning.** The argument each document makes stays as written, for
  the reason `docs/design/README.md` gives.
- **No builds or test runs on this machine.** Verification is by reading the tree. This is why
  `UNVERIFIABLE-HERE` exists.
- **No repair of citations outside `docs/design/`.** The rest of `docs/` describes the tree as
  it is and is governed by the existing checks; extending the citation check to cover it is
  worth doing and is not this change.

---

## 4. Verification

- `tools/documentation/check_design_citations.py` exits zero over `docs/design/`.
- `tools/documentation/check_documentation_length.py` exits zero, so no correction introduced a
  line over 100 columns or a link that does not resolve.
- Every status line in `docs/design/` names a verdict that D2 recorded, and every phase marker
  in a roadmap section agrees with that verdict.
- `docs/reference/glossary.md` registers a family for all eighteen documents.
- `docs/reference/changelog.md` carries the entry `CONTRIBUTING.md` §5 requires.

---

## 5. Commits

Four, in the order the stages run:

1. `feat(documentation): check that a design document's cited paths resolve` — D1.
2. `docs: correct the design corpus's status lines and roadmap markers` — D2 and D3.
3. `docs: repair the design corpus's paths after the repository restructure` — D4.
4. `docs: record what every design document has left to build` — D5.
