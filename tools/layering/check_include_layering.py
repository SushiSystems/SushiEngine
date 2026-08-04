#!/usr/bin/env python3
"""Catch module edges that reach another module by include path instead of by link.

``sushiengine_add_module()`` runs ``sushiengine_check_module_edge()`` over every
``PUBLIC_DEPENDS`` and ``PRIVATE_DEPENDS`` entry, so a declared dependency pointing up the tier
order fails the configure. A raw ``target_include_directories()`` naming another module's
``include/`` root creates exactly the same coupling — the consumer compiles against the other
module's headers — and the checker never sees it, because it is not a declared dependency.

Three modules use that form today, all of them legally: ``authoring``, ``serialization`` and
``simulation`` reach ``presentation/render`` so that a ``SUSHIENGINE_BUILD_RENDER=OFF``
configure still produces a compiling tree. ``world`` sits above ``presentation``, so those
edges point down and are allowed. The hole is that nothing would have caught them if they
pointed up.

This closes the hole statically. It reads the tier order, the module-to-tier map and the
forbidden-edge list out of ``cmake/EngineLayers.cmake`` — the same data the configure-time
check reads — then maps every ``engine/<tier>/<module>`` path mentioned inside a
``target_include_directories()`` call back to a module and applies the same rule.

Every such edge is reported as a note whether or not it is legal, because an edge the
configure-time checker cannot see is worth being visible somewhere. Only an illegal one fails
the run. Run from the repository root.
"""

from __future__ import annotations

import pathlib
import re
import sys

LAYERS_FILE = pathlib.Path("cmake/EngineLayers.cmake")

LIST_VARIABLE = r"set\(\s*{name}\s+([^)]*)\)"
INCLUDE_CALL = re.compile(r"target_include_directories\s*\((.*?)\)", re.DOTALL)
MODULE_PATH = re.compile(r"engine/([a-z_]+)/([a-z_]+)/(?:include|source)")


def read_list(text: str, name: str) -> list[str]:
    """The whitespace-separated words of a `set(<name> …)` block, quotes stripped."""
    match = re.search(LIST_VARIABLE.format(name=name), text, re.MULTILINE)
    if not match:
        raise SystemExit(f"{LAYERS_FILE}: no {name} found")
    return [word.strip().strip('"') for word in match.group(1).split() if word.strip()]


def pairs(words: list[str]) -> dict[str, str]:
    """A flat `a b c d` list read as `{a: b, c: d}`."""
    return dict(zip(words[0::2], words[1::2]))


def build_lists() -> list[pathlib.Path]:
    """Every module and application `CMakeLists.txt`, in a stable order."""
    found = sorted(pathlib.Path("engine").glob("*/*/CMakeLists.txt"))
    found.extend(sorted(pathlib.Path("applications").glob("*/CMakeLists.txt")))
    return found


def main() -> int:
    if not LAYERS_FILE.is_file():
        print("check_include_layering.py must be run from the repository root")
        return 2

    manifest = LAYERS_FILE.read_text(encoding="utf-8")
    order = read_list(manifest, "SUSHIENGINE_LAYER_ORDER")
    module_layer = pairs(read_list(manifest, "SUSHIENGINE_MODULE_LAYERS"))
    forbidden = set(
        zip(*[iter(read_list(manifest, "SUSHIENGINE_FORBIDDEN_EDGES"))] * 2)
    )
    rank = {tier: index for index, tier in enumerate(order)}

    failures: list[str] = []
    notes: list[str] = []

    for lists_file in build_lists():
        consumer = lists_file.parent.name
        consumer_tier = module_layer.get(consumer)
        if consumer_tier is None:
            continue

        body = lists_file.read_text(encoding="utf-8")
        for call in INCLUDE_CALL.findall(body):
            for _, module in MODULE_PATH.findall(call.replace("\\", "/")):
                dependency_tier = module_layer.get(module)
                if module == consumer or dependency_tier is None:
                    continue
                where = f"{lists_file.as_posix()}: {consumer} includes {module} by path"
                notes.append(f"{where} ({consumer_tier} -> {dependency_tier})")
                if (consumer, module) in forbidden:
                    failures.append(f"{where}, and that edge is forbidden outright")
                elif rank[dependency_tier] > rank[consumer_tier]:
                    failures.append(
                        f"{where}, but {dependency_tier} sits above {consumer_tier} "
                        f"in the tier order ({' < '.join(order)})"
                    )

    for note in sorted(set(notes)):
        print(f"note: {note}")
    for failure in failures:
        print(f"error: {failure}")

    if failures:
        print(f"\n{len(failures)} include-path edges break the tier rule")
        return 1
    print(f"include-path edges hold the tier rule across {len(module_layer)} modules")
    return 0


if __name__ == "__main__":
    sys.exit(main())
