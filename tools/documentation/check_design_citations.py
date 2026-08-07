"""Fails when a path a design document cites as evidence does not resolve.

``docs/design/`` states its own evidence standard — ``static_mesh_authoring.md`` §1 writes it
as "a claim is a file:line, not a description" — and ``docs/documentation-style-guide.md``
repeats it as "a path in prose is a real path". Nothing checked either, and this script
measures 617 of 680 cited paths pointing at nothing.

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
# ``docs`` is NOT among them. A design document cites other documents, the pattern below
# matches ``.md``, and the two renames the reorganization performed — ``docs/glossary.md`` to
# ``docs/reference/glossary.md`` and its neighbours — are only repairable if those files are
# indexed. Only the generated reference under ``docs/api`` and ``docs/api-site`` is skipped,
# the same exclusion ``check_documentation_length.py`` makes.
SKIPPED_DIRECTORIES = {
    ".git", ".venv", "__pycache__", "api", "api-site", "build", "build-editor", "build-player",
    "cooked", "third_party",
}

# A citation is a backticked token that names a file: it carries an extension this tree
# actually uses, and optionally a line locator, which is trimmed before lookup. The locator is
# one line, one range, or a comma-separated run of either (design docs cite "cpp:47,73,86" to
# point at three call sites in one file, not three files).
#
# The extension list is source code and shader/configuration formats, plus the asset formats the
# design corpus cites by concrete example rather than by extension alone: glTF/GLB import
# sources (``gltf``), their Unity-parity sidecar (``meta``), editor layout state (``ini``), CI
# workflow files (``yml``) and this project's own prefab format (``sushiprefab``, first landed
# by the prefab system design) and atmosphere climatology data (``set0``).
CITATION_PATTERN = re.compile(
    r"`([A-Za-z0-9_./-]+\.(?:cpp|hpp|h|c|py|comp|frag|vert|glsl|slang|json|toml|cmake|md|txt"
    r"|gltf|meta|ini|yml|set0|sushiprefab))"
    r"(?::[0-9]+(?:-[0-9]+)?(?:,[0-9]+(?:-[0-9]+)?)*)?`")

ALLOWED = {}


def index_tree():
    """Every citable file in the repository, keyed by file name.

    :return: a dictionary mapping a file name to every repository-relative posix path that
             carries it, so a citation whose directory was renamed can still be resolved.
    """
    by_base_name = defaultdict(list)

    for directory, subdirectories, file_names in os.walk(REPOSITORY_ROOT):
        subdirectories[:] = [d for d in subdirectories if d not in SKIPPED_DIRECTORIES]
        for file_name in file_names:
            relative = (Path(directory) / file_name).relative_to(REPOSITORY_ROOT).as_posix()
            by_base_name[file_name].append(relative)

    return by_base_name


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

    A target need not live under the repository tree — the negative fixture this script is
    verified against deliberately sits in scratch space so it can never be committed — so a
    document outside ``REPOSITORY_ROOT`` is reported by its own path rather than one relative
    to a root it is not under.

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
        try:
            relative = document.relative_to(REPOSITORY_ROOT).as_posix()
        except ValueError:
            relative = document.as_posix()
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

    by_base_name = index_tree()
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
