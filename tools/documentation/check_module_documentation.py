"""Fails when the module tree and the module documentation stop describing each other.

Three asymmetries rot silently and none of them breaks a build, which is why they need a
check rather than a convention:

1. A module gains a directory under ``engine/<tier>/`` and never gains a ``README.md``, so
   the one place that says what it owns, what it links and what covers it does not exist.
2. A ``README.md`` loses its ``{#module-<name>}`` heading label, or keeps a label from
   before a rename. The label is not decoration: ``Doxyfile`` pulls ``engine`` in with
   ``FILE_PATTERNS = *.md`` and ``RECURSIVE = YES``, so every module README becomes a page
   on the generated site, and pages named ``README.md`` collide on the same auto-derived
   label. The explicit label is what keeps them apart, and a wrong one silently points a
   cross-reference at another module's page.
3. ``docs/modules/README.md`` — the index, which holds no facts of its own beyond a
   one-line summary — drifts from the tree, either by missing a module or by linking a
   README that is not there.

Run it from the repository root; it locates the tree from its own path either way.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
ENGINE_DIRECTORY = REPOSITORY_ROOT / "engine"
INDEX_PATH = REPOSITORY_ROOT / "docs" / "modules" / "README.md"

# The label as it must appear on the README's first level-one heading.
HEADING_LABEL_PATTERN = re.compile(r"\{#module-([A-Za-z0-9_]+)\}")
LEVEL_ONE_HEADING_PATTERN = re.compile(r"^#(?!#)\s*(.*)$")

# Inline markdown links only: ``[text](target)``. Reference-style links and bare URLs are
# not used in the index, and a link with a title or an anchor is trimmed to its path.
MARKDOWN_LINK_PATTERN = re.compile(r"\[[^\]]*\]\(([^)\s]+)(?:\s+\"[^\"]*\")?\)")


def discover_modules():
    """Every directory two levels under engine/ that declares itself to the build.

    A module exists to CMake once it has a ``CMakeLists.txt``, so that file — not a naming
    convention and not a manifest this script would have to parse — is the membership test.
    Directories such as ``engine/include`` therefore fall out on their own.

    :return: (tier, name, directory) triples, sorted by tier then name.
    """
    modules = []
    if not ENGINE_DIRECTORY.is_dir():
        return modules

    for tier_directory in sorted(p for p in ENGINE_DIRECTORY.iterdir() if p.is_dir()):
        for module_directory in sorted(p for p in tier_directory.iterdir() if p.is_dir()):
            if (module_directory / "CMakeLists.txt").is_file():
                modules.append((tier_directory.name, module_directory.name, module_directory))
    return modules


def check_readme(module_name, readme_path, failures):
    """Checks one module README exists and carries the heading label for its own directory."""
    relative = readme_path.relative_to(REPOSITORY_ROOT).as_posix()

    if not readme_path.is_file():
        failures.append(f"{relative}: missing. Every module directory needs a README.md.")
        return

    heading = None
    for line in readme_path.read_text(encoding="utf-8").splitlines():
        match = LEVEL_ONE_HEADING_PATTERN.match(line)
        if match:
            heading = match.group(1)
            break

    if heading is None:
        failures.append(f"{relative}: has no level-one heading to carry a Doxygen label.")
        return

    label = HEADING_LABEL_PATTERN.search(heading)
    if label is None:
        failures.append(
            f"{relative}: its heading carries no {{#module-{module_name}}} label. "
            "Without one this page collides with every other module README on the "
            "Doxygen site.")
        return

    if label.group(1) != module_name:
        failures.append(
            f"{relative}: labelled {{#module-{label.group(1)}}} but the directory is "
            f"'{module_name}'.")


def check_index(modules, failures):
    """Checks the index links every module README, and links nothing that is not there."""
    relative_index = INDEX_PATH.relative_to(REPOSITORY_ROOT).as_posix()

    if not INDEX_PATH.is_file():
        failures.append(f"{relative_index}: missing. It is the index onto the module READMEs.")
        return

    linked = set()
    for target in MARKDOWN_LINK_PATTERN.findall(INDEX_PATH.read_text(encoding="utf-8")):
        if "://" in target or target.startswith("#"):
            continue

        path_part = target.split("#", 1)[0]
        if not path_part:
            continue

        resolved = (INDEX_PATH.parent / path_part).resolve()
        if not resolved.exists():
            failures.append(f"{relative_index}: links '{target}', which does not exist.")
            continue

        linked.add(resolved)

    for tier, name, directory in modules:
        readme_path = (directory / "README.md").resolve()
        if readme_path not in linked:
            expected = Path("..", "..", "engine", tier, name, "README.md").as_posix()
            failures.append(
                f"{relative_index}: does not link the '{name}' module. Add a row pointing "
                f"at {expected}.")


def main():
    failures = []

    modules = discover_modules()
    if not modules:
        failures.append(
            f"{ENGINE_DIRECTORY.relative_to(REPOSITORY_ROOT).as_posix()}: no modules found. "
            "Either the tree moved or this script is looking in the wrong place.")

    for _, name, directory in modules:
        check_readme(name, directory / "README.md", failures)

    check_index(modules, failures)

    if failures:
        for failure in failures:
            print(failure, file=sys.stderr)
        sys.exit(1)

    print(f"Module documentation is symmetric: {len(modules)} modules, each with a labelled "
          "README.md linked from docs/modules/README.md.")


if __name__ == "__main__":
    main()
