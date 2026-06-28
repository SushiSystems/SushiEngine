"""Layered configuration loading for the SushiEngine CLI.

Precedence (lowest to highest):
    built-in defaults -> config.toml -> config.local.toml -> SE_* env vars

The active platform's ``[tool.<platform>]`` table is merged over the common
``[tool]`` table, so a single file describes both Linux and Windows.

The engine is the head of the stack: it consumes SushiRuntime's SYCL toolchain
and vcpkg tree rather than provisioning its own. So this config does not select a
toolchain — it locates the runtime sibling and the compiler/vcpkg it already
installed, plus the handful of host tools (cmake, ninja, vcvars) a build needs.
"""

from __future__ import annotations

import os
import platform

try:
    import tomllib  # Python 3.11+
except ModuleNotFoundError:  # Python 3.10 fallback
    import tomli as tomllib
from dataclasses import dataclass, fields
from pathlib import Path


def find_project_root(start: Path | None = None) -> Path:
    """Locate the repo root by walking up from CWD until CMakeLists.txt is found.

    The CLI is installed (pip/pipx) outside the repo, so the package location
    tells us nothing about where the project lives — the invocation directory
    does. Run any `se` command from anywhere inside the checkout.
    """
    cur = (start or Path.cwd()).resolve()
    for d in (cur, *cur.parents):
        if (d / "CMakeLists.txt").is_file():
            return d
    raise SystemExit(
        "Not inside a SushiEngine project: no CMakeLists.txt found in the "
        "current directory or any parent. cd into the repo and try again."
    )


def config_dir(root: Path | None = None) -> Path:
    """Directory holding config.toml / config.local.toml (the repo's cli/)."""
    root = root or find_project_root()
    return root / "cli"


# Maps a Config field to the SE_* env var that overrides it.
_ENV_OVERRIDES = {
    "cxx": "SE_CXX",
    "generator": "SE_CMAKE_GENERATOR",
    "vcpkg_root": "SE_VCPKG_ROOT",
    "vs_vcvars": "SE_VCVARS",
    "ninja_exe": "SE_NINJA",
    "cmake_exe": "SE_CMAKE",
    "ctest_exe": "SE_CTEST",
    "pkgconf_exe": "SE_PKGCONF",
    "doxygen_exe": "SE_DOXYGEN",
    "vcpkg_triplet": "SE_VCPKG_TRIPLET",
    "sushiruntime_dir": "SUSHIRUNTIME_DIR",
    "target_bin": "SE_TARGET_BIN",
}


@dataclass
class Config:
    """Resolved, platform-specific tool configuration."""

    # The SYCL compiler. Empty means "discover it": the runtime's bundled clang++
    # (under <runtime>/dependencies/toolchains/llvm-sycl/bin) if present, else a
    # clang++ on PATH.
    cxx: str = ""
    generator: str = "Ninja"
    use_vcpkg: bool = False

    # Tool roots / paths (mostly Windows-specific absolutes).
    vcpkg_root: str = ""
    vs_vcvars: str = ""
    ninja_exe: str = ""
    # cmake/ctest are resolved from PATH when empty. They are configurable because
    # VS BuildTools does not ship the CMake component, so on Windows cmake commonly
    # lives in a scoop/standalone install that is not on PATH.
    cmake_exe: str = ""
    ctest_exe: str = ""
    pkgconf_exe: str = ""
    # doxygen is resolved from PATH when empty; configurable because on Windows it
    # commonly installs outside PATH (winget/choco shims or Program Files).
    doxygen_exe: str = ""
    vcpkg_triplet: str = "x64-windows"

    # The SushiRuntime sibling checkout. Empty means "../sushiruntime relative to
    # the project root" (matching SUSHIRUNTIME_DIR's default in CMakeLists.txt).
    sushiruntime_dir: str = ""

    # Run defaults.
    target_bin: str = "sandbox"

    # Derived.
    platform: str = ""

    @property
    def is_windows(self) -> bool:
        return self.platform == "windows"

    def expand(self, value: str) -> str:
        """Expand ~ and env vars in a path-like config value."""
        return os.path.expandvars(os.path.expanduser(value)) if value else value

    def runtime_dir(self, root: Path) -> Path:
        """Resolve the SushiRuntime checkout this engine builds against.

        @param root The engine project root (the directory holding CMakeLists.txt).
        @return The runtime directory, defaulting to ``<root>/../sushiruntime``.
        """
        if self.sushiruntime_dir:
            return Path(self.expand(self.sushiruntime_dir)).resolve()
        return (root / ".." / "sushiruntime").resolve()

    def bundled_clang(self, root: Path) -> str:
        """The runtime-bundled clang++ path, or '' when the bundle is absent.

        @param root The engine project root.
        @return The absolute clang++ path under the runtime's toolchain bundle, or
                an empty string when it has not been installed there.
        """
        exe = "clang++.exe" if self.is_windows else "clang++"
        candidate = self.runtime_dir(root) / "dependencies" / "toolchains" / "llvm-sycl" / "bin" / exe
        return str(candidate) if candidate.is_file() else ""

    def resolved_compiler(self, root: Path) -> str:
        """The C++ compiler to drive the build: explicit, then bundled, then PATH.

        @param root The engine project root.
        @return A configured cxx, else the runtime-bundled clang++, else bare
                'clang++' for PATH resolution.
        """
        if self.cxx:
            return self.expand(self.cxx)
        bundled = self.bundled_clang(root)
        return bundled or "clang++"

    def resolved_vcpkg(self, root: Path) -> str:
        """The vcpkg root: explicit, then the runtime-bundled tree, else ''.

        @param root The engine project root.
        @return An absolute vcpkg root, or '' when none is configured or bundled.
        """
        if self.vcpkg_root:
            return self.expand(self.vcpkg_root)
        bundled = self.runtime_dir(root) / "dependencies" / "vcpkg"
        return str(bundled) if bundled.is_dir() else ""


def _read_toml(path: Path) -> dict:
    if not path.is_file():
        return {}
    with path.open("rb") as fh:
        return tomllib.load(fh)


def _merge_tool_table(doc: dict, plat: str) -> dict:
    """Merge common [tool] with [tool.<platform>] (platform wins)."""
    tool = dict(doc.get("tool", {}))
    merged = {k: v for k, v in tool.items() if not isinstance(v, dict)}
    plat_table = tool.get(plat, {})
    if isinstance(plat_table, dict):
        merged.update(plat_table)
    return merged


def load_config() -> Config:
    """Load and resolve the layered configuration for the current platform."""
    plat = platform.system().lower()  # 'windows' | 'linux' | 'darwin'

    cfg_dir = config_dir()
    values: dict = {}
    for fname in ("config.toml", "config.local.toml"):
        doc = _read_toml(cfg_dir / fname)
        if doc:
            values.update(_merge_tool_table(doc, plat))

    # Env overrides (highest precedence below CLI flags).
    for field_name, env_var in _ENV_OVERRIDES.items():
        if env_var in os.environ:
            values[field_name] = os.environ[env_var]

    # Coerce booleans that may arrive as strings from env.
    if isinstance(values.get("use_vcpkg"), str):
        values["use_vcpkg"] = values["use_vcpkg"].strip().lower() in ("1", "true", "yes", "on")

    known = {f.name for f in fields(Config)}
    cfg = Config(**{k: v for k, v in values.items() if k in known})
    cfg.platform = plat
    return cfg
