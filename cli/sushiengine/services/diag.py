"""Diagnostic commands: inspect the resolved config and the build environment.

These are read-only troubleshooting helpers. When a build picks the "wrong"
compiler or a path looks off, `se config show` answers *what* the CLI resolved
and *where each value came from* (default / config.toml / env override), and
`se env dump` answers *which environment* cmake/ctest actually run under.
"""

from __future__ import annotations

import os
import platform
from dataclasses import fields
from pathlib import Path

from rich.table import Table

from .. import console
from ..config import (
    Config,
    _ENV_OVERRIDES,
    _merge_tool_table,
    _read_toml,
    config_dir,
    find_project_root,
    load_config,
)

# Environment variables worth surfacing by default in `env dump` — the ones that
# actually steer a SYCL build. `--all` overrides this filter.
_ENV_OF_INTEREST = (
    "PATH", "CC", "CXX", "SYCL", "VCPKG", "PKG_CONFIG", "CMAKE", "NINJA",
    "INCLUDE", "LIB", "LD_LIBRARY_PATH", "SUSHIRUNTIME",
)


def _toml_keys(root: Path, plat: str) -> set[str]:
    """Field names explicitly present in config.toml / config.local.toml."""
    cfg_dir = config_dir(root)
    keys: set[str] = set()
    for fname in ("config.toml", "config.local.toml"):
        doc = _read_toml(cfg_dir / fname)
        if doc:
            keys.update(_merge_tool_table(doc, plat).keys())
    return keys


def config_show() -> int:
    console.header("Resolved Configuration")
    root = find_project_root()
    plat = platform.system().lower()
    cfg = load_config()
    toml_keys = _toml_keys(root, plat)

    table = Table(show_header=True, header_style="bold magenta")
    table.add_column("Setting")
    table.add_column("Value")
    table.add_column("Source")

    for f in fields(Config):
        value = getattr(cfg, f.name)
        env_var = _ENV_OVERRIDES.get(f.name)
        if env_var and env_var in os.environ:
            source = f"env:{env_var}"
        elif f.name in toml_keys:
            source = "config.toml"
        else:
            source = "default"
        table.add_row(f.name, str(value), source)

    console.console.print(table)

    # The two resolved-at-runtime paths the build actually uses.
    console.info(f"Project root      : {root}")
    console.info(f"SushiRuntime dir  : {cfg.runtime_dir(root)}")
    console.info(f"Resolved compiler : {cfg.resolved_compiler(root)}")
    console.info(f"Resolved vcpkg    : {cfg.resolved_vcpkg(root) or '(none)'}")

    cfg_dir = config_dir(root)
    console.info(f"Config dir        : {cfg_dir}")
    for fname in ("config.toml", "config.local.toml"):
        path = cfg_dir / fname
        mark = "found" if path.is_file() else "absent"
        console.info(f"  {fname:<20} [{mark}]")
    return 0


def env_dump(show_all: bool = False) -> int:
    console.header("Build Environment")
    from ..env import load_build_env  # local import: avoids a cycle at module load

    root = find_project_root()
    cfg = load_config()
    env = load_build_env(cfg, root / "build")

    def keep(key: str) -> bool:
        return show_all or any(tok in key.upper() for tok in _ENV_OF_INTEREST)

    shown = {k: v for k, v in sorted(env.items()) if keep(k)}

    table = Table(show_header=True, header_style="bold magenta")
    table.add_column("Variable")
    table.add_column("Value", overflow="fold")
    for k, v in shown.items():
        table.add_row(k, v)
    console.console.print(table)

    console.info(f"{len(shown)} of {len(env)} variables shown"
                 + ("" if show_all else " (build-relevant; use --all for everything)"))
    return 0
