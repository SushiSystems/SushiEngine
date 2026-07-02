"""Layered configuration loading for the SushiEngine CLI.

Precedence (lowest to highest):
    built-in defaults -> config.toml -> config.local.toml -> SE_* env vars

The active platform's ``[tool.<platform>]`` table is merged over the common
``[tool]`` table, so a single file describes both Linux and Windows.

The engine is a head of the stack: it consumes the shared SushiStack SYCL
toolchain and vcpkg tree rather than provisioning its own. So this config does
not select a toolchain — it locates the shared ``<workspace>/dependencies`` tree
(provisioned by ``ss install``) for the compiler and vcpkg, the runtime sibling
module it builds against, plus the handful of host tools (cmake, ninja, vcvars)
a build needs. Standalone (outside a workspace) it falls back to the runtime's
own bundled dependencies for backward compatibility.
"""

from __future__ import annotations

import os
import platform

from dataclasses import dataclass
from pathlib import Path

# Domain-agnostic config plumbing shared by every Sushi* CLI. The generic build-
# tool schema (cmake/ninja/vcpkg paths) and the layered-load skeleton live in
# sushicli; this repo adds only the engine-specific fields below.
from sushicli.config_base import ToolConfig, load_tool_config
from sushicli.workspace import has_marker, resolve_env_path, walk_up


def find_project_root(start: Path | None = None) -> Path:
    """Locate the repo root by walking up from CWD until CMakeLists.txt is found.

    The CLI is installed (pip/pipx) outside the repo, so the package location
    tells us nothing about where the project lives — the invocation directory
    does. Run any `se` command from anywhere inside the checkout.
    """
    root = walk_up(start or Path.cwd(), has_marker("CMakeLists.txt"))
    if root is None:
        raise SystemExit(
            "Not inside a SushiEngine project: no CMakeLists.txt found in the "
            "current directory or any parent. cd into the repo and try again."
        )
    return root


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
class Config(ToolConfig):
    """Resolved, platform-specific tool configuration for the engine build.

    Inherits the generic host build-tool fields (cmake/ninja/vcpkg paths, etc.)
    from :class:`ToolConfig`. The engine selects no SYCL toolchain — it consumes
    the shared one — so it adds only the sibling-runtime checkout below. ``cxx``
    (from ToolConfig) empty means "discover it": the runtime's bundled clang++
    (under <runtime>/dependencies/toolchains/llvm-sycl/bin) if present, else a
    clang++ on PATH.
    """

    # The SushiRuntime sibling checkout. Empty means "../sushiruntime relative to
    # the project root" (matching SUSHIRUNTIME_DIR's default in CMakeLists.txt).
    sushiruntime_dir: str = ""

    # Run defaults.
    target_bin: str = "sandbox"

    # Engine Scalar precision. False = single (float), True = double. Persisted here
    # (the editor's Preferences writes it) and consumed as -DSE_SCALAR_DOUBLE at
    # configure time; changing it requires a rebuild because Scalar is compile-time.
    scalar_double: bool = False

    def runtime_dir(self, root: Path) -> Path:
        """Resolve the SushiRuntime checkout this engine builds against.

        @param root The engine project root (the directory holding CMakeLists.txt).
        @return The runtime directory, defaulting to ``<root>/../sushiruntime``
                (which, inside a SushiStack workspace, is the sibling module).
        """
        if self.sushiruntime_dir:
            return Path(self.expand(self.sushiruntime_dir)).resolve()
        return (root / ".." / "sushiruntime").resolve()

    def sushistack_home(self, root: Path) -> Path | None:
        """Locate the SushiStack workspace root, or None when standalone.

        SushiStack is the umbrella that provisions one shared dependency tree for
        every module. Resolution: ``SUSHISTACK_HOME`` env var, then a walk up from
        the engine root looking for the ``.sushistack`` marker.

        @param root The engine project root.
        @return The workspace root, or None when not inside a workspace.
        """
        home = resolve_env_path("SUSHISTACK_HOME")
        if home:
            return home
        return walk_up(root, has_marker(".sushistack"))

    def deps_dir(self, root: Path) -> Path:
        """The shared dependency tree this engine resolves its toolchain from.

        Inside a SushiStack workspace that is ``<workspace>/dependencies`` (shared
        by every module); standalone it falls back to the runtime's own bundled
        ``dependencies`` tree. ``SUSHISTACK_DEPS_DIR`` overrides both.

        @param root The engine project root.
        @return The dependency tree root.
        """
        override = os.environ.get("SUSHISTACK_DEPS_DIR")
        if override:
            return Path(self.expand(override))
        home = self.sushistack_home(root)
        if home:
            return home / "dependencies"
        return self.runtime_dir(root) / "dependencies"

    def bundled_clang(self, root: Path) -> str:
        """The shared bundled clang++ path, or '' when the bundle is absent.

        @param root The engine project root.
        @return The absolute clang++ path under the shared toolchain bundle, or an
                empty string when it has not been installed there.
        """
        exe = "clang++.exe" if self.is_windows else "clang++"
        candidate = self.deps_dir(root) / "toolchains" / "llvm-sycl" / "bin" / exe
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
        """The vcpkg root: explicit, then the shared bundled tree, else ''.

        @param root The engine project root.
        @return An absolute vcpkg root, or '' when none is configured or bundled.
        """
        if self.vcpkg_root:
            return self.expand(self.vcpkg_root)
        bundled = self.deps_dir(root) / "vcpkg"
        return str(bundled) if bundled.is_dir() else ""


def _shared_config_local() -> Path | None:
    """The workspace-shared config.local.toml ``ss install`` writes, if any.

    Inside a SushiStack workspace the machine-specific tool paths (compiler,
    vcpkg, cmake) are resolved once by ``ss`` and written to ``<home>/cli/
    config.local.toml``; every module reads them from there so nothing is
    configured twice.
    """
    home = resolve_env_path("SUSHISTACK_HOME")
    if home is None:
        try:
            home = walk_up(find_project_root(), has_marker(".sushistack"))
        except SystemExit:
            home = None
    return (home / "cli" / "config.local.toml") if home else None


def load_config() -> Config:
    """Load and resolve the layered configuration for the current platform.

    Precedence (low to high): repo config.toml -> workspace-shared
    config.local.toml (written by ``ss``) -> repo config.local.toml -> env.
    """
    plat = platform.system().lower()  # 'windows' | 'linux' | 'darwin'

    cfg_dir = config_dir()
    shared = _shared_config_local()
    sources = [cfg_dir / "config.toml"]
    if shared is not None:
        sources.append(shared)
    sources.append(cfg_dir / "config.local.toml")
    return load_tool_config(Config, sources, plat, _ENV_OVERRIDES)
