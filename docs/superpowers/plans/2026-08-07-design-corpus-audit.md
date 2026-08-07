# Design Corpus Audit Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development
> (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use
> checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make every claim in `docs/design/` either true or visibly marked untrue, and add the
check that keeps it that way.

**Architecture:** A new Python checker classifies every backticked repository path cited in
`docs/design/` against the tree and fails on any that neither resolves nor appears in an
explicit allow list with a stated reason. The eighteen documents' roadmap sections are then read
and each completed phase is tested against the tree, producing a verdict per phase. Status lines,
roadmap markers and the three derived surfaces are corrected from those verdicts, the stale paths
are repaired, and what remains becomes one consolidated backlog.

**Tech Stack:** Python 3 (standard library only), markdown, GitHub Actions.

## Global Constraints

- Prose obeys `docs/documentation-style-guide.md`: 100-column lines, declarative voice, present
  tense, no abbreviations, acronyms fully upper-case, no marketing, no hedging.
- `docs/superpowers/plans/` is **not** exempt from the 900-line file ceiling. `docs/design/` is
  exempt from the file and paragraph ceilings but **not** from the 100-column line ceiling.
- Python tools live in `tools/documentation/`, run from the repository root, locate the tree from
  `Path(__file__).resolve().parents[2]`, print every violation, and exit non-zero on failure.
  They use the standard library only — the CI `structure` job installs nothing.
- Never `git add -A`. Another session edits this tree concurrently; stage explicit paths only.
- Never run `se build`, `se test`, `cmake` or `ninja` on this machine. Verification is by reading
  the tree. A claim that needs a GPU is recorded `UNVERIFIABLE-HERE`, never assumed either way.
- Every commit header is `type: short summary`, lowercase, imperative, no trailing period.
  Bodies are past-tense bullets.
- `docs/reference/changelog.md` gains an entry under `## [Unreleased]` for the user-visible
  parts, each bullet at most 240 characters.
- Measured baseline, 2026-08-07: 836 unique cited paths, 116 alive, 525 suffix-repairable, 100
  base-name-repairable, 24 ambiguous, 71 unresolved.

---

## Task 1: The citation checker

**Files:**
- Create: `tools/documentation/check_design_citations.py`
- Test: no Python test suite exists in this repository. The three existing checkers are
  validated by running them over the real tree plus a deliberate negative fixture; this one
  follows that convention.

**Interfaces:**
- Produces: `tools/documentation/check_design_citations.py`, runnable as
  `python3 tools/documentation/check_design_citations.py [--report] [PATH ...]`.
  `--report` prints the classification table instead of failing, which Tasks 5 and 6 consume.
  Exit code 0 when every citation resolves or is allowed, 1 otherwise.
- Produces: `ALLOWED`, a dictionary in that script mapping a cited path to
  `(category, reason)` where category is one of `external`, `runtime-artifact`, `planned`.

- [ ] **Step 1: Write the negative fixture**

Create the fixture in the scratchpad, not the repository — it must never be committed:

```bash
mkdir -p "$SCRATCH/citation_fixture"
cat > "$SCRATCH/citation_fixture/fixture.md" <<'EOF'
# Fixture

A path that resolves: `tools/documentation/check_module_documentation.py`.
A path that moved: `physics/xpbd_solver.hpp`.
A path that matches several files: `editor/main.cpp`.
A path that exists nowhere: `engine/domain/nothing/never_written.hpp`.
EOF
```

Set `SCRATCH` to the session scratchpad directory first.

- [ ] **Step 2: Run the not-yet-written checker against the fixture to verify it fails**

Run: `python3 tools/documentation/check_design_citations.py --report "$SCRATCH/citation_fixture"`
Expected: FAIL with `No such file or directory`.

- [ ] **Step 3: Write the checker**

```python
"""Fails when a path a design document cites as evidence does not resolve.

``docs/design/`` states its own evidence standard — ``static_mesh_authoring.md`` §1 writes it
as "a claim is a file:line, not a description" — and ``docs/documentation-style-guide.md``
repeats it as "a path in prose is a real path". Nothing checked either, and the repository
restructure left 720 of 836 cited paths pointing at nothing.

``check_documentation_length.py`` already resolves markdown *links*. A design document does not
cite evidence as a link; it cites it as a backticked path, which nothing looked at until this
script.

Three kinds of cited path legitimately do not resolve, and they are not interchangeable, so
each is declared in ``ALLOWED`` with the category that says why:

* ``external`` — a file in the sibling SushiRuntime checkout, or a system header. Real, and
  outside this repository.
* ``runtime-artifact`` — a file the program writes or reads at run time. A repository that
  carried one would be carrying a user's state.
* ``planned`` — a file a plan names and no commit has produced yet. Legitimate in a corpus that
  records intent, and the entry is what makes the intent auditable rather than assumed.

An ``ALLOWED`` entry whose path *does* resolve is itself a failure. Without that rule the list
would silently outlive the reason it was added, which is the exact rot this script exists to
stop.

Run it from the repository root; it locates the tree from its own path either way.
"""

from __future__ import annotations

import os
import re
import sys
from collections import defaultdict
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DESIGN_DIRECTORY = REPOSITORY_ROOT / "docs" / "design"

# Directories that hold no citable source: build trees, generated output, vendored code, and
# the cooked asset cache. Walking them would make a stale build directory answer for the tree.
SKIPPED_DIRECTORIES = {
    ".git", ".venv", "__pycache__", "build", "build-editor", "build-player", "cooked",
    "docs", "third_party",
}

# A citation is a backticked token that names a file: it carries an extension this tree
# actually uses, and optionally a :line or :line-line suffix, which is trimmed before lookup.
CITATION_PATTERN = re.compile(
    r"`([A-Za-z0-9_./-]+\.(?:cpp|hpp|h|c|py|comp|frag|vert|glsl|slang|json|toml|cmake|md|txt))"
    r"(?::[0-9]+(?:-[0-9]+)?)?`")

ALLOWED = {}


def index_tree():
    """Every citable file in the repository, as repository-relative posix paths.

    :return: a (all_paths, by_base_name) pair, the second keyed by file name so a citation
             whose directory was renamed can still be resolved.
    """
    all_paths = []
    by_base_name = defaultdict(list)

    for directory, subdirectories, file_names in os.walk(REPOSITORY_ROOT):
        subdirectories[:] = [d for d in subdirectories if d not in SKIPPED_DIRECTORIES]
        for file_name in file_names:
            relative = (Path(directory) / file_name).relative_to(REPOSITORY_ROOT).as_posix()
            all_paths.append(relative)
            by_base_name[file_name].append(relative)

    return all_paths, by_base_name


def classify(citation, by_base_name):
    """Decides what a single cited path is, and what would repair it.

    Resolution is tried three ways, narrowest first: the path as written, then the unique tree
    file ending with it, then the unique tree file carrying its base name. Narrowest-first
    matters because a citation that resolves as written must never be "repaired" to a
    same-named file elsewhere.

    :param citation: the cited path, already stripped of any :line suffix.
    :param by_base_name: the index returned by index_tree().
    :return: a (verdict, detail) pair. verdict is 'alive', 'repairable', 'ambiguous' or
             'unresolved'; detail is the repaired path for 'repairable', the candidate list
             for 'ambiguous', and None otherwise.
    """
    if (REPOSITORY_ROOT / citation).exists():
        return "alive", None

    base_name = citation.rsplit("/", 1)[-1]
    candidates = by_base_name.get(base_name, [])

    by_suffix = [c for c in candidates if c.endswith("/" + citation)]
    if len(by_suffix) == 1:
        return "repairable", by_suffix[0]
    if len(by_suffix) > 1:
        return "ambiguous", by_suffix

    if len(candidates) == 1:
        return "repairable", candidates[0]
    if len(candidates) > 1:
        return "ambiguous", candidates

    return "unresolved", None


def collect(paths):
    """Every citation in every markdown file under the given paths, with where it was cited.

    :param paths: files or directories to scan.
    :return: a dictionary mapping a cited path to the sorted document names citing it.
    """
    documents = []
    for path in paths:
        if path.is_dir():
            documents.extend(sorted(path.rglob("*.md")))
        elif path.suffix == ".md":
            documents.append(path)

    citations = defaultdict(set)
    for document in documents:
        relative = document.relative_to(REPOSITORY_ROOT).as_posix()
        text = document.read_text(encoding="utf-8")
        for match in CITATION_PATTERN.finditer(text):
            citations[match.group(1)].add(relative)

    return {citation: sorted(where) for citation, where in citations.items()}


def main():
    arguments = sys.argv[1:]
    report_only = "--report" in arguments
    targets = [Path(a).resolve() for a in arguments if not a.startswith("--")]
    if not targets:
        targets = [DESIGN_DIRECTORY]

    for target in targets:
        if not target.exists():
            print(f"{target}: No such file or directory", file=sys.stderr)
            sys.exit(1)

    _, by_base_name = index_tree()
    citations = collect(targets)

    tally = defaultdict(int)
    failures = []
    lines = []

    for citation in sorted(citations):
        where = ", ".join(citations[citation])
        verdict, detail = classify(citation, by_base_name)

        if citation in ALLOWED:
            category, reason = ALLOWED[citation]
            if verdict == "alive":
                failures.append(
                    f"{citation}: allowed as '{category}' ({reason}) but the file now exists. "
                    "Remove the ALLOWED entry.")
            tally[f"allowed:{category}"] += 1
            lines.append(f"  allowed:{category:17} {citation}")
            continue

        tally[verdict] += 1
        if verdict == "alive":
            lines.append(f"  {'alive':25} {citation}")
        elif verdict == "repairable":
            lines.append(f"  {'repairable':25} {citation}\n      -> {detail}")
            failures.append(f"{citation} ({where}): does not resolve. It is now {detail}.")
        elif verdict == "ambiguous":
            lines.append(f"  {'ambiguous':25} {citation}  ({len(detail)} candidates)")
            failures.append(
                f"{citation} ({where}): does not resolve and {len(detail)} files match. "
                "Cite the full path.")
        else:
            lines.append(f"  {'unresolved':25} {citation}")
            failures.append(
                f"{citation} ({where}): names no file in the tree. Correct it, or add an "
                "ALLOWED entry saying whether it is external, a run-time artifact, or planned.")

    if report_only:
        print("\n".join(lines))
        print()
        for key in sorted(tally):
            print(f"  {key:25} {tally[key]:>5}")
        print(f"  {'TOTAL':25} {sum(tally.values()):>5}")
        return

    if failures:
        for failure in failures:
            print(failure, file=sys.stderr)
        print(f"\n{len(failures)} citation(s) in the design corpus do not resolve.",
              file=sys.stderr)
        sys.exit(1)

    print(f"Design corpus citations resolve: {sum(tally.values())} cited paths across "
          f"{len(set(w for ws in citations.values() for w in ws))} documents.")


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Run the checker against the fixture to verify each class**

Run: `python3 tools/documentation/check_design_citations.py --report "$SCRATCH/citation_fixture"`

Expected: exactly four citations, classified `alive`, `repairable`, `ambiguous` and
`unresolved`, one each. If `physics/xpbd_solver.hpp` does not come back `repairable`, find the
file that does move under `engine/domain/physics/` and use that name in the fixture instead —
the fixture must exercise the class, not a particular file.

- [ ] **Step 5: Run the checker against the fixture without `--report` to verify it fails**

Run: `python3 tools/documentation/check_design_citations.py "$SCRATCH/citation_fixture"`
Expected: exit 1, three failures printed, no traceback.

- [ ] **Step 6: Run the checker over the real corpus and record the baseline**

```bash
python3 tools/documentation/check_design_citations.py --report > "$WORKSPACE/citations-baseline.txt"
```

Expected: the totals reproduce the Global Constraints baseline — 836 cited paths, 116 alive.
A different total is not automatically wrong (the regexes differ slightly from the probe that
produced the baseline), but a total that differs by more than a few is a bug in `collect` or
`CITATION_PATTERN`, not a discovery. Investigate before continuing.

Keep this file. Tasks 5 and 6 read it.

The check is **not** wired into continuous integration in this task. It would fail from the
moment it landed until Task 6 repairs the paths, and a knowingly-red check in the default
branch is a defect however well justified. Task 6 wires it in the same commit that makes it
pass.

- [ ] **Step 7: Verify the documentation checks still pass**

Run: `python3 tools/documentation/check_documentation_length.py`
Expected: `documentation ceilings hold across N files`, with only the four pre-existing
changelog bullet warnings.

Run: `python3 tools/documentation/check_module_documentation.py`
Expected: exit 0.

- [ ] **Step 8: Commit**

```bash
git add tools/documentation/check_design_citations.py
git commit -m "feat(documentation): check that a design document's cited paths resolve"
```

Body:

```
- Added a checker that classifies every backticked path in docs/design/ against the tree.
- Added an ALLOWED list that separates an external reference from a run-time artifact and
  from a file a plan has not produced yet, and fails when an allowed path starts resolving.
```

---

## Task 2: Audit the six documents that claim to be shipped

**Files:**
- Create: `$WORKSPACE/verdicts.md` (working notes, never committed)
- Read: `docs/design/animation_system.md`, `audio_system.md`, `SUSHILOOP.md`,
  `project_selection.md`, `static_mesh_authoring.md`, `editor_ux_overhaul.md`

**Interfaces:**
- Produces: `$WORKSPACE/verdicts.md`, one section per document, one row per phase:
  `| phase | verdict | evidence |` where verdict is `CONFIRMED`, `OVERSTATED`, `STALE` or
  `UNVERIFIABLE-HERE` and evidence is a real path in the tree or the search that found nothing.
  Tasks 3, 4, 5 and 7 read this file.

A `shipped` status is the strongest claim the corpus makes, which is why these are audited
first: an overstatement here is the most misleading, and the cheapest to falsify.

- [ ] **Step 1: Read each document's roadmap section and list its phases**

Each document's opening status line names the section that records progress —
`animation_system.md` names §12, `audio_system.md` its roadmap, `SUSHILOOP.md` its milestone
list. Read that section, not the prose above it. Write every phase and its claimed state into
`$WORKSPACE/verdicts.md` before verifying anything, so the verification cannot quietly skip a
phase it fails to find.

- [ ] **Step 2: Verify each phase against the tree**

For each phase, take the concrete noun it names — a type, a function, a file, an `se`
subcommand, a test name — and find it:

```bash
# A type or function: it must exist and live in a module that CMake builds.
rg -n "class AnimatorEvaluator|animator_step" engine/ applications/ --type cpp
# A file: its module must carry a CMakeLists.txt, or nothing links it.
ls engine/domain/animation/CMakeLists.txt
# An se subcommand: the CLI is Python, and the guide must agree with it.
rg -n "add_parser\(" cli/sushiengine/ | rg -i "editor|player|planet"
# A test: CTest only discovers what tests/CMakeLists.txt registers.
rg -n "Integration_Cloth" tests/
```

A phase is `CONFIRMED` only when the thing it names exists **and** is reachable from a build
target. A header that no `CMakeLists.txt` pulls in is not shipped code; it is a file.

- [ ] **Step 3: Record the known suspects explicitly**

Three claims are already known to need a verdict. Do not let them pass by inspection:

- `animation_system.md` claims GPU skinning shipped and cites `render/passes/skinning_pass.cpp`
  and `.hpp`. Neither name exists anywhere in the tree. Establish whether the pass exists under
  another name (`STALE`) or does not exist (`OVERSTATED`).
- `animation_system.md` says the editor GUI parts are "still unverified on hardware". That is
  `UNVERIFIABLE-HERE`, and the status line must keep saying so rather than rounding up.
- `atmosphere_system.md` is audited in Task 3, but its cited test
  `test_weather_determinism.cpp` does not exist; note it here so Task 3 does not rediscover it.

- [ ] **Step 4: Commit the working notes to the scratchpad only**

Nothing in this task touches the repository. There is no commit. Confirm with:

Run: `git status --short`
Expected: only Task 1's already-committed files, and nothing new.

---

## Task 3: Audit the five documents that claim to be in progress

**Files:**
- Modify: `$WORKSPACE/verdicts.md` (append five sections)
- Read: `docs/design/physics_system.md` §16, `render_pipeline_refactor.md` §3,
  `atmosphere_system.md` §11, `vfx_particle_system.md` §12, `repository_restructure.md`

**Interfaces:**
- Consumes: the verdict table format defined in Task 2.
- Produces: the same table extended by five sections.

- [ ] **Step 1: Audit `physics_system.md` §16**

The status line claims P0–P7 and PX complete but for PX-9, P8 under way, P9 not started. Two
facts already contradict parts of that. Verify both:

```bash
# PX-9 is the node-beam cooker in the Bake window. Does the Bake window offer it?
rg -n "node.beam|NodeBeam|beam" applications/editor/source/physics/
# The vehicle scene-integration debt: does anything count vehicles in the live slot inventory?
rg -n "live_slot_count|vehicles_" engine/world/simulation/ engine/domain/physics/
```

Recent commits `f3fbd56 fix(sim): enumerate a scene's vehicles in its body inventory` and
`7ee9f57 fix(sim): put a vehicle's collision surface in the scene's query hierarchy` suggest
that debt closed after the status line was written. If it did, the phase is `STALE`, not
`OVERSTATED` — the work exists and the document describes it wrongly.

- [ ] **Step 2: Audit `render_pipeline_refactor.md` §3**

The status line claims phases 0 to 11 shipped in core form. Every one of its 41 cited paths is
dead, so no claim in it can be confirmed by citation alone. Verify each phase by its named
mechanism instead:

```bash
rg -n "clustered|Cluster" engine/presentation/render/ --type cpp -l
ls engine/presentation/render/shaders/ | head -50
rg -n "QualityParams" engine/ applications/ -l
```

- [ ] **Step 3: Audit `atmosphere_system.md` §11**

Phases A to C are claimed shipped, D to F open. Confirm the phase A–C artifacts exist, and
record the missing `test_weather_determinism.cpp` noted in Task 2 Step 3:

```bash
ls engine/domain/atmosphere/
rg -n "weather|atmosphere" tests/ -l
```

- [ ] **Step 4: Audit `vfx_particle_system.md` §12**

The status line claims VFX1 to VFX7 done, beams and SDF collision open — while
`docs/reference/glossary.md` registers only `VFX1`–`VFX6`. One of the two is wrong. Determine
which by finding whether a seventh phase exists in §12 at all:

```bash
rg -n "^#+.*VFX[0-9]" docs/design/vfx_particle_system.md
ls engine/domain/vfx/ engine/presentation/render/shaders/ | rg -i particle
```

- [ ] **Step 5: Audit `repository_restructure.md`**

Phases 1 to 3 are claimed complete and phase 4 — documentation — under way. Task 1's baseline
is the measurement of exactly how far phase 4 got: 720 of 836 cited paths still unrepaired.
Record that number as the phase 4 evidence, and confirm phases 1 to 3 by the tier layout and
the configure-time enforcement they promise:

```bash
ls engine/*/ -d
cat cmake/EngineLayers.cmake | head -40
python3 tools/layering/check_include_layering.py
```

- [ ] **Step 6: Verify no repository file changed**

Run: `git status --short`
Expected: nothing new. This task produces judgement, not edits.

---

## Task 4: Audit the five designed documents and the superseded one

**Files:**
- Modify: `$WORKSPACE/verdicts.md` (append six sections)
- Read: `docs/design/model_import.md`, `prefab_system.md`, `solar_system_overhaul.md`,
  `cross_platform_engineering_plan.md`, `unified_hazard_model.md`,
  `editor_feature_sync_gaps.md`

**Interfaces:**
- Consumes: the verdict table format defined in Task 2.
- Produces: the same table extended by six sections.

A `designed` status claims that **no** implementation exists. It is falsified by finding code,
which makes it the cheapest class to audit and the one most likely to be wrong here: the last
fifteen commits on `main` implement two of these five.

- [ ] **Step 1: Falsify `model_import.md` and `prefab_system.md`**

```bash
git log --oneline -30 -- engine/world/model_import/ engine/world/serialization/ engine/asset/
ls engine/world/model_import/ engine/asset/model/ engine/asset/gltf/
rg -n "sushiprefab|PrefabInstance" engine/ applications/ -l
```

Both documents say `designed`. Commits `6ed72c1`, `7ff6ce7`, `65022f5`, `95be2e6` and
`bc312a7` say otherwise. Read each document's roadmap section (`model_import.md` §13,
`prefab_system.md` §11) and mark each phase against what the tree carries. Expect a mix: the
early phases `CONFIRMED`, the later ones genuinely not started.

- [ ] **Step 2: Confirm the other three are genuinely unbuilt**

```bash
# solar_system_overhaul: is there a planet bake command or a terrain tile pipeline?
rg -n "planet" cli/sushiengine/ -l ; ls engine/domain/terrain/
# cross_platform: is there any non-Windows platform branch or an RHI split?
ls engine/foundation/platform/ ; rg -n "RHI" engine/presentation/render/ -l | head
# unified_hazard_model: does an Execution::Handoff or equivalent exist?
rg -n "Handoff|hazard" engine/foundation/execution/ -l
```

A `designed` document with no implementation is `CONFIRMED` — the status is accurate. Say so;
an audit that only reports faults is not an audit.

Note that `docs/design/README.md` describes `solar_system_overhaul.md` as `designed` while the
memory of prior work records phases P0–P2b as committed. Resolve which by the tree, not by
either record.

- [ ] **Step 3: Check `editor_feature_sync_gaps.md`'s supersession claim**

It claims its two "Deferred" sections were absorbed into `editor_ux_overhaul.md` §2.4. Open
§2.4 and confirm every deferred item appears there with a verdict. A supersession that lost
items is a real defect, because the items vanish from the backlog entirely.

- [ ] **Step 4: Verify no repository file changed**

Run: `git status --short`
Expected: nothing new.

---

## Task 5: Correct the status lines, the roadmap markers and the derived surfaces

**Files:**
- Modify: every file under `docs/design/` whose verdict table holds a non-`CONFIRMED` row
- Modify: `docs/design/README.md`, `docs/reference/glossary.md`,
  `docs/architecture/roadmap.md`, `docs/reference/changelog.md`

**Interfaces:**
- Consumes: `$WORKSPACE/verdicts.md` from Tasks 2, 3 and 4.

This is the semantic commit. It is deliberately separate from Task 6's path repair so that
every judgement it records stays legible in `git blame` instead of being buried under several
hundred mechanical rewrites.

- [ ] **Step 1: Correct each document's status line and roadmap markers**

For every `OVERSTATED` row, the phase marker changes from complete to whatever the tree
supports, and the opening status line changes with it. For every `STALE` row, the description
changes to name what the tree actually carries. For every `UNVERIFIABLE-HERE` row, the document
says so in those terms rather than claiming either outcome.

Write in the corpus's own voice: declarative, present tense, specific, no hedging. A corrected
line names the file or symbol that justifies it.

- [ ] **Step 2: Bring `docs/design/README.md` into agreement**

Its `Status` column must repeat each document's own status line. At minimum `model_import.md`
and `prefab_system.md` move off `designed`; apply whatever Tasks 2 to 4 found for the rest.

- [ ] **Step 3: Bring `docs/reference/glossary.md` into agreement**

Two known defects, plus whatever the audit added:

- The VFX family is registered as `VFX1`–`VFX6`; correct it to the range
  `vfx_particle_system.md` actually mints, as established in Task 3 Step 4.
- No family is registered for `model_import.md`, `prefab_system.md`,
  `static_mesh_authoring.md` or `project_selection.md`. Add a row for each, using the phase
  codes those documents mint.

- [ ] **Step 4: Bring `docs/architecture/roadmap.md` into agreement**

This file describes the tree as it is, so unlike `docs/design/` it may not describe unbuilt
work at all. Check every milestone it marks done against the verdict table and correct any that
the audit contradicted.

- [ ] **Step 5: Add the changelog entry**

Under `## [Unreleased]`, in `Fixed`, at most 240 characters per bullet:

```markdown
- Corrected the design corpus's status lines and roadmap markers against the tree, and
  registered the four missing phase-code families in the glossary.
```

- [ ] **Step 6: Verify the documentation checks pass**

Run: `python3 tools/documentation/check_documentation_length.py`
Expected: no new warnings, no failures. A corrected line that runs past 100 columns fails here.

- [ ] **Step 7: Commit**

```bash
git add docs/design docs/reference/glossary.md docs/reference/changelog.md \
        docs/architecture/roadmap.md
git commit -m "docs: correct the design corpus's status lines and roadmap markers"
```

---

## Task 6: Repair the stale paths

**Files:**
- Modify: every file under `docs/design/` carrying an unresolved citation
- Modify: `tools/documentation/check_design_citations.py` (the `ALLOWED` dictionary)
- Modify: `.github/workflows/ci.yml` (the `structure` job, deferred here from Task 1)

**Interfaces:**
- Consumes: `$WORKSPACE/citations-baseline.txt` from Task 1, and the `ALLOWED` shape
  `{path: (category, reason)}` defined there.

- [ ] **Step 1: Rewrite the unambiguous citations**

The checker prints `-> <repaired path>` for each. Apply them. Do this per document and re-run
the checker after each, so a bad substitution is caught against one file rather than eighteen.

Beware the `:line` suffixes: `physics/xpbd.hpp:53` becomes the new path with `:53` intact, but
the line number itself is now almost certainly wrong. Where a line number cannot be
re-verified against the file, drop the suffix and cite the path alone. A wrong line number is
worse than no line number — it reads as precision.

- [ ] **Step 2: Decide the 24 ambiguous citations**

Each names a file whose base name occurs several times, such as `editor/main.cpp` with nine
candidates. Read the surrounding sentence to find which one it means, and write the full path.
Where the sentence genuinely means "each of these", write the directory instead of a file.

- [ ] **Step 3: Classify the 71 unresolved citations into `ALLOWED` or a correction**

Sort each into one of four outcomes:

- Names a file in the sibling SushiRuntime checkout or a system header
  (`ENGINE_BACKBONE_REFACTOR.md`, `SushiRuntime.h`, `vulkan.h`, `vk_mem_alloc.h`): add an
  `ALLOWED` entry with category `external`.
- Names a file the program writes at run time (`boot.json`, `preferences.json`,
  `controller.json`, `cooking_profile.json`): category `runtime-artifact`.
- Names a file a plan has not produced yet (`terrain_tile_compile.comp`, `imagery.py`,
  `landcover.py`): category `planned`. The reason string states which document plans it.
- Names a file that was renamed rather than removed (`docs/CHANGELOG.md` →
  `docs/reference/changelog.md`, `docs/glossary.md` → `docs/reference/glossary.md`,
  `ARCHITECTURE.md`, `docs/CLI_GUIDE.md`, `docs/VEHICLES.md`): correct the citation. These are
  not allowed entries; the file exists under a new name.

A citation that is `planned` **and** sits inside a phase Task 2 to 4 marked complete is not a
`planned` entry. It is evidence for an `OVERSTATED` verdict that Task 5 should have caught.
Stop and fix Task 5's output rather than allow-listing it.

- [ ] **Step 4: Run the checker to verify it passes**

Run: `python3 tools/documentation/check_design_citations.py`
Expected: exit 0, `Design corpus citations resolve: N cited paths across 18 documents.`

- [ ] **Step 5: Verify no line ran past the ceiling**

Run: `python3 tools/documentation/check_documentation_length.py`
Expected: no new warnings. Repaired paths are longer than the ones they replace —
`physics/xpbd.hpp` becomes `engine/domain/physics/include/SushiEngine/physics/xpbd.hpp` — so
this step will find lines that now overflow. Rewrap them.

- [ ] **Step 6: Wire the check into continuous integration**

The check now passes, so it can be enforced. In `.github/workflows/ci.yml`, in the `structure`
job, add a fourth step after the existing documentation length check:

```yaml
      - name: Check that a design document's cited paths resolve
        # docs/design/ states that a claim is a file:line rather than a description. Nothing
        # measured that, and the restructure left 720 of 836 cited paths resolving to nothing.
        run: python3 tools/documentation/check_design_citations.py
```

Then correct the job's own comment, which says "three Python scripts over the tree":

```
    # Cheap and dependency-free on purpose: four Python scripts over the tree, no toolchain
```

- [ ] **Step 7: Commit**

```bash
git add docs/design tools/documentation/check_design_citations.py .github/workflows/ci.yml
git commit -m "docs: repair the design corpus's paths after the repository restructure"
```

---

## Task 7: Write the consolidated backlog

**Files:**
- Create: `docs/design/remaining_work.md`
- Modify: `docs/design/README.md` (one index row)

**Interfaces:**
- Consumes: `$WORKSPACE/verdicts.md` from Tasks 2, 3 and 4.

The corpus records what each subsystem plans. Nothing records what the *project* has left,
because that answer only exists by reading eighteen roadmap sections at once. This file is that
reading, and it holds no facts of its own — every row cites the document that owns the phase.

- [ ] **Step 1: Write the backlog**

One table, one row per phase that Tasks 2 to 4 found incomplete:

Four columns: the phase code, a relative link to the document that owns it (the file sits in
`docs/design/`, so the link is the bare file name), the state the audit found, and one line on
what the phase still needs. Every row's third and fourth columns come from the verdict table —
this file introduces no fact of its own.

Sort by document, not by priority — the priority order is the project owner's and is proposed
separately, in the response rather than the file, so the file does not go stale the moment the
owner reorders it.

- [ ] **Step 2: Add the index row to `docs/design/README.md`**

Status column: `living`. It is neither a plan nor an audit; it is a derived view that changes
whenever any roadmap does.

- [ ] **Step 3: Verify the checks pass**

Run: `python3 tools/documentation/check_design_citations.py`
Run: `python3 tools/documentation/check_documentation_length.py`
Expected: both exit 0.

- [ ] **Step 4: Commit**

```bash
git add docs/design/remaining_work.md docs/design/README.md
git commit -m "docs: record what every design document has left to build"
```

- [ ] **Step 5: Propose the priority order**

Present the backlog to the project owner with a proposed order and the reasoning behind it.
The order is a proposal. The owner sets it.

---

## Verification

The whole plan is done when all of these hold:

- `python3 tools/documentation/check_design_citations.py` exits 0.
- `python3 tools/documentation/check_documentation_length.py` exits 0 with no new warnings.
- `python3 tools/documentation/check_module_documentation.py` exits 0.
- `python3 tools/layering/check_include_layering.py` exits 0.
- Every status line in `docs/design/` matches a verdict in the audit, and every roadmap phase
  marker agrees with it.
- `docs/reference/glossary.md` registers a phase-code family for all eighteen documents.
- `docs/design/remaining_work.md` accounts for every non-`CONFIRMED` phase.
- `git log --oneline -4` shows the four commits, in the order the tasks define them.
