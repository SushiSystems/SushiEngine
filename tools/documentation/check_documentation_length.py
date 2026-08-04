#!/usr/bin/env python3
"""Enforce the length and link ceilings in ``docs/documentation-style-guide.md``.

The repository has carried a "a changelog entry is a short bullet, not an essay" rule in
prose since it started, and has ignored it the whole time, because nothing measured it. This
script is what measures it. It checks, over every markdown file under ``docs/`` and every
module ``README.md`` under ``engine/``:

* prose lines wider than 100 columns,
* paragraphs longer than 1,200 characters,
* changelog bullets longer than 240 characters (fatal past 400),
* files longer than 900 lines,
* markdown links that resolve to no file, or to no heading in the file they name.

The anchor half matters more than it looks. Splitting ``ARCHITECTURE.md`` turned several hundred
``see §5.0.1`` references into anchor links, and a section renamed later would break them
silently — a bare file-existence check would keep passing.

The exemptions are the style guide's own: table rows and fenced code blocks cannot be
wrapped, a line whose overflow is one unbreakable token has nowhere to break, and
``docs/design/`` and ``docs/api/`` are exempt from the file and paragraph ceilings because a
design document is one long argument and the generated reference is not hand-written.

Run from the repository root. Exits non-zero on the first category that has any violation,
after printing all of them.
"""

from __future__ import annotations

import pathlib
import re
import sys

COLUMN_CEILING = 100
PARAGRAPH_CEILING = 1200
CHANGELOG_BULLET_TARGET = 240
CHANGELOG_BULLET_CEILING = 400
FILE_LINE_CEILING = 900

CHANGELOG = pathlib.Path("docs/reference/changelog.md")
GENERATED_OR_DESIGN = ("docs/design/", "docs/api/", "docs/api-site/")
# The changelog grows by append and is split at a release boundary, not by subject, so the
# file ceiling would only ever ask for a split that makes the history harder to read. Its
# bullets carry their own, stricter ceiling instead.
EXEMPT_FROM_FILE_CEILING = (CHANGELOG,)

FENCE = re.compile(r"^\s*(```|~~~)")
TABLE_ROW = re.compile(r"^\s*\|")
BULLET = re.compile(r"^(\s*)[-*]\s+")
HEADING = re.compile(r"^\s*#{1,6}\s")
# Markdown inline links, minus image embeds and anything with a scheme or a bare anchor.
LINK = re.compile(r"(?<!\!)\[[^\]^]*\]\(([^)\s]+)\)")


def documentation_files() -> list[pathlib.Path]:
    """Every hand-written markdown file this guide governs, in a stable order."""
    files = [
        path
        for path in pathlib.Path("docs").rglob("*.md")
        if not path.as_posix().startswith("docs/api-site/")
    ]
    files.extend(pathlib.Path("engine").rglob("README.md"))
    files.extend(pathlib.Path("tests").rglob("README.md"))
    files.extend(pathlib.Path("assets").rglob("README.md"))
    root_readme = pathlib.Path("README.md")
    if root_readme.is_file():
        files.append(root_readme)
    return sorted(files)


def is_exempt_from_structure(path: pathlib.Path) -> bool:
    """True for the trees exempt from the file and paragraph ceilings."""
    return path.as_posix().startswith(GENERATED_OR_DESIGN)


def prose_lines(lines: list[str]) -> list[tuple[int, str]]:
    """The lines a ceiling can be applied to: not fenced code, not a table row."""
    kept: list[tuple[int, str]] = []
    inside_fence = False
    for number, line in enumerate(lines, start=1):
        if FENCE.match(line):
            inside_fence = not inside_fence
            continue
        if inside_fence or TABLE_ROW.match(line):
            continue
        kept.append((number, line))
    return kept


def overflow_is_unbreakable(line: str) -> bool:
    """True when nothing in the line crosses the ceiling at a space, so it cannot wrap."""
    if len(line) <= COLUMN_CEILING:
        return True
    return " " not in line[:COLUMN_CEILING].strip()


def paragraphs(lines: list[str]) -> list[tuple[int, int]]:
    """Runs of consecutive prose lines, as (first line number, character count)."""
    found: list[tuple[int, int]] = []
    start = 0
    length = 0
    inside_fence = False
    for number, line in enumerate(lines, start=1):
        if FENCE.match(line):
            inside_fence = not inside_fence
            start, length = 0, 0
            continue
        stripped = line.strip()
        breaks_paragraph = (
            inside_fence
            or not stripped
            or TABLE_ROW.match(line)
            or HEADING.match(line)
            or BULLET.match(line)
            or stripped.startswith(">")
        )
        if breaks_paragraph:
            if start:
                found.append((start, length))
            start, length = 0, 0
            continue
        if not start:
            start = number
        length += len(stripped) + 1
    if start:
        found.append((start, length))
    return found


def changelog_bullets(lines: list[str]) -> list[tuple[int, int]]:
    """Every bullet in the changelog, as (first line number, character count).

    A bullet runs until the next bullet, the next blank line or the next heading, so a
    wrapped bullet is measured whole rather than one line at a time.
    """
    found: list[tuple[int, int]] = []
    start = 0
    length = 0
    for number, line in enumerate(lines, start=1):
        stripped = line.strip()
        if BULLET.match(line):
            if start:
                found.append((start, length))
            start = number
            length = len(BULLET.sub("", line).strip())
            continue
        if not start:
            continue
        if not stripped or HEADING.match(line):
            found.append((start, length))
            start, length = 0, 0
            continue
        length += 1 + len(stripped)
    if start:
        found.append((start, length))
    return found


def heading_anchors(path: pathlib.Path) -> set[str]:
    """The anchors GitHub derives from a file's headings, duplicate suffixes included.

    Lower-cased, punctuation dropped, spaces hyphenated. A Doxygen page label — the
    ``{#module-physics}`` form the module READMEs carry — is stripped before the anchor is
    derived but is also accepted as an anchor of its own, because that is what Doxygen links to.
    """
    anchors: set[str] = set()
    seen: dict[str, int] = {}
    inside_fence = False
    for line in path.read_text(encoding="utf-8").splitlines():
        if FENCE.match(line):
            inside_fence = not inside_fence
            continue
        if inside_fence or not HEADING.match(line):
            continue
        title = line.lstrip("#").strip()
        label = re.search(r"\{#([^}]+)\}", title)
        if label:
            anchors.add(label.group(1))
            title = title[: label.start()].strip()
        # GitHub's rule exactly: lower-case, drop everything that is not a word character,
        # a hyphen or a space, then turn each remaining space into one hyphen. Collapsing
        # runs of spaces first would be wrong — "polish & data" loses the ampersand and
        # keeps both spaces, so its anchor really does carry a double hyphen.
        slug = re.sub(r"[^\w\- ]", "", title.replace("`", "")).strip().lower()
        slug = slug.replace(" ", "-")
        if not slug:
            continue
        count = seen.get(slug, 0)
        seen[slug] = count + 1
        anchors.add(slug if count == 0 else f"{slug}-{count}")
    return anchors


def broken_links(
    path: pathlib.Path, lines: list[str], anchors: dict[pathlib.Path, set[str]]
) -> list[tuple[int, str]]:
    """Relative markdown links from this file resolving to no file, or to no heading in one."""
    broken: list[tuple[int, str]] = []
    for number, line in enumerate(lines, start=1):
        for target in LINK.findall(line):
            if "://" in target or target.startswith("mailto:"):
                continue
            relative, _, anchor = target.partition("#")
            destination = path if not relative else path.parent / relative
            if relative and not destination.exists():
                broken.append((number, target))
                continue
            if not anchor or destination.suffix != ".md":
                continue
            resolved = destination.resolve()
            if resolved not in anchors:
                anchors[resolved] = heading_anchors(destination)
            if anchor not in anchors[resolved]:
                broken.append((number, f"{target} (no such heading)"))
    return broken


def main() -> int:
    if not pathlib.Path("docs").is_dir():
        print("check_documentation_length.py must be run from the repository root")
        return 2

    failures: list[str] = []
    warnings: list[str] = []
    files = documentation_files()
    anchors: dict[pathlib.Path, set[str]] = {}

    for path in files:
        text = path.read_text(encoding="utf-8")
        lines = text.splitlines()
        display = path.as_posix()

        for number, line in prose_lines(lines):
            if len(line) > COLUMN_CEILING and not overflow_is_unbreakable(line):
                failures.append(f"{display}:{number}: {len(line)} columns, ceiling is {COLUMN_CEILING}")

        if not is_exempt_from_structure(path):
            if len(lines) > FILE_LINE_CEILING and path not in EXEMPT_FROM_FILE_CEILING:
                failures.append(
                    f"{display}: {len(lines)} lines, ceiling is {FILE_LINE_CEILING} — split it by subject"
                )
            for number, size in paragraphs(lines):
                if size > PARAGRAPH_CEILING:
                    failures.append(
                        f"{display}:{number}: paragraph of {size} characters, "
                        f"ceiling is {PARAGRAPH_CEILING}"
                    )

        for number, target in broken_links(path, lines, anchors):
            failures.append(f"{display}:{number}: link resolves to nothing: {target}")

    if CHANGELOG.is_file():
        for number, size in changelog_bullets(CHANGELOG.read_text(encoding="utf-8").splitlines()):
            if size > CHANGELOG_BULLET_CEILING:
                failures.append(
                    f"{CHANGELOG.as_posix()}:{number}: changelog bullet of {size} characters, "
                    f"ceiling is {CHANGELOG_BULLET_CEILING}"
                )
            elif size > CHANGELOG_BULLET_TARGET:
                warnings.append(
                    f"{CHANGELOG.as_posix()}:{number}: changelog bullet of {size} characters, "
                    f"target is {CHANGELOG_BULLET_TARGET}"
                )

    for warning in warnings:
        print(f"warning: {warning}")
    for failure in failures:
        print(f"error: {failure}")

    if failures:
        print(f"\n{len(failures)} documentation violations; see docs/documentation-style-guide.md")
        return 1
    print(f"documentation ceilings hold across {len(files)} files")
    return 0


if __name__ == "__main__":
    sys.exit(main())
