"""Project build / test / run / clean / doxygen logic.

The cmake / ctest invocations are issued directly via subprocess with the
snapshotted build env. The engine configures against the SushiRuntime sibling's
bundled clang++ and vcpkg, so the configure args resolve those from the runtime
checkout (or the config overrides) rather than provisioning anything.
"""

from __future__ import annotations

import enum
import shutil
import subprocess
from pathlib import Path

from .. import console
from ..config import Config, find_project_root, load_config
from ..env import load_build_env
from . import discovery


class BuildType(str, enum.Enum):
    release = "release"
    debug = "debug"
    relwithdebinfo = "relwithdebinfo"


class Suite(str, enum.Enum):
    # Fine-grained CTest labels (assigned in tests/CMakeLists.txt from the GTest
    # suite-name prefix convention).
    unit = "unit"
    regression = "regression"
    integration = "integration"
    functional = "functional"   # umbrella: unit + regression + integration
    all = "all"


# Each suite maps to a CTest label-selection regex (`ctest -L`). The umbrella
# (functional) is an alternation over its sub-labels rather than a label of its
# own, because a baked-in umbrella label does not survive gtest_discover_tests on
# current CMake (see tests/CMakeLists.txt). `all` selects everything (no -L).
_SUITE_LABEL_REGEX = {
    Suite.unit: "^unit$",
    Suite.regression: "^regression$",
    Suite.integration: "^integration$",
    Suite.functional: "^(unit|integration|regression)$",
}

_CMAKE_BUILD_TYPE = {
    BuildType.release: "Release",
    BuildType.debug: "Debug",
    BuildType.relwithdebinfo: "RelWithDebInfo",
}


def _build_dir(root: Path) -> Path:
    return root / "build"


def _cmake(cfg: Config) -> str:
    """The cmake executable: the configured path if set, else bare 'cmake'."""
    return cfg.expand(cfg.cmake_exe) if cfg.cmake_exe else "cmake"


def _ctest(cfg: Config) -> str:
    """The ctest executable: the configured path if set, else bare 'ctest'."""
    return cfg.expand(cfg.ctest_exe) if cfg.ctest_exe else "ctest"


def _resolve_exe(name: str, env: dict[str, str]) -> str:
    """Return the full path to *name* from env PATH, or *name* itself as fallback.

    Using the full path sidesteps Windows PATH case-sensitivity bugs when a
    plain-dict env is passed to subprocess (os.environ uses 'Path', overlays may
    use 'PATH' — subprocess sees both and picks unpredictably).
    """
    env_path = next((v for k, v in env.items() if k.upper() == "PATH"), None)
    found = shutil.which(name, path=env_path) or shutil.which(name)
    return found or name


def _run(cmd: list[str], env: dict[str, str], cwd: Path) -> int:
    resolved = list(cmd)
    resolved[0] = _resolve_exe(cmd[0], env)
    console.command(subprocess.list2cmdline(resolved))
    try:
        return subprocess.run(resolved, cwd=str(cwd), env=env).returncode
    except FileNotFoundError:
        console.error(
            f"Executable not found: '{cmd[0]}'.\n"
            f"  - it is not on PATH and no explicit path is configured.\n"
            f"  - set its path in config.local.toml (e.g. cmake_exe / ctest_exe / ninja_exe), or\n"
            f"  - run `se config` to see what the CLI resolved.")
        return 1


def _run_drained(cmd: list[str], env: dict[str, str], cwd: Path) -> int:
    """Run *cmd* while actively draining its output, echoing it line by line.

    Unlike :func:`_run` (which lets the child inherit our stdout), this pipes the
    child's stdout/stderr and reads them here. That matters for ``ctest`` on
    Windows: its ``gtest_discover_tests`` step spawns the test binary to enumerate
    cases, and when ctest's stdout is an inherited, slowly-drained pipe the
    discovery child intermittently stalls and registers nothing — surfacing as
    "No tests were found". Draining the pipe ourselves keeps enumeration reliable.
    """
    resolved = list(cmd)
    resolved[0] = _resolve_exe(cmd[0], env)
    console.command(subprocess.list2cmdline(resolved))
    try:
        proc = subprocess.Popen(
            resolved, cwd=str(cwd), env=env,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1)
    except FileNotFoundError:
        console.error(f"Executable not found: '{cmd[0]}'. Run `se config`.")
        return 1
    assert proc.stdout is not None
    for line in proc.stdout:
        console.console.print(line.rstrip("\n"), highlight=False, soft_wrap=True)
    return proc.wait()


def _needs_configure(build_dir: Path, generator: str) -> bool:
    if not build_dir.is_dir():
        return True
    sentinel = "build.ninja" if generator == "Ninja" else "Makefile"
    return not (build_dir / sentinel).is_file()


def _check_runtime(cfg: Config, root: Path) -> int:
    """Fail early (with guidance) when the SushiRuntime sibling is missing.

    The engine cannot configure without it — the runtime provides the SYCL target
    and the add_sycl_to_target command the engine's CMakeLists pulls in.
    """
    runtime = cfg.runtime_dir(root)
    if (runtime / "CMakeLists.txt").is_file():
        return 0
    console.error(
        f"SushiRuntime sibling not found at: {runtime}\n"
        f"  - clone it next to this repo, or\n"
        f"  - set sushiruntime_dir in cli/config.local.toml, or SUSHIRUNTIME_DIR in the env.")
    return 1


def _configure_args(cfg: Config, root: Path, build_dir: Path,
                    build_type: str, tests: bool,
                    scalar_double: bool = False) -> list[str]:
    runtime = cfg.runtime_dir(root)
    cxx = cfg.resolved_compiler(root)
    vcpkg = cfg.resolved_vcpkg(root)

    # Always pass the precision explicitly so toggling it re-caches cleanly: an
    # absent -D would otherwise leave a stale value in CMakeCache.txt.
    double = scalar_double or cfg.scalar_double

    args = [
        _cmake(cfg), "-S", str(root), "-B", str(build_dir), "-G", cfg.generator,
        f"-DCMAKE_BUILD_TYPE={build_type}",
        f"-DCMAKE_CXX_COMPILER={cxx}",
        f"-DSUSHIRUNTIME_DIR={runtime}",
        f"-DSE_BUILD_TESTS={'ON' if tests else 'OFF'}",
        f"-DSE_SCALAR_DOUBLE={'ON' if double else 'OFF'}",
    ]
    # On Windows clang++ also drives the C probe; point both slots at it.
    if cfg.is_windows:
        args.append(f"-DCMAKE_C_COMPILER={cxx}")
        if cfg.ninja_exe:
            args.append(f"-DCMAKE_MAKE_PROGRAM={cfg.expand(cfg.ninja_exe)}")
        if cfg.pkgconf_exe:
            args.append(f"-DPKG_CONFIG_EXECUTABLE={cfg.expand(cfg.pkgconf_exe)}")
    if vcpkg:
        args += [
            f"-DCMAKE_TOOLCHAIN_FILE={vcpkg}/scripts/buildsystems/vcpkg.cmake",
            f"-DVCPKG_ROOT={vcpkg}",
        ]
        if cfg.is_windows:
            args += [
                f"-DVCPKG_TARGET_TRIPLET={cfg.vcpkg_triplet}",
                f"-DCMAKE_PREFIX_PATH={vcpkg}/installed/{cfg.vcpkg_triplet}",
            ]
    return args


def build(build_type: BuildType, clean: bool = False, tests: bool = True,
          double: bool = False) -> int:
    console.header("Project Build")
    root = find_project_root()
    cfg = load_config()
    cmake_build_type = _CMAKE_BUILD_TYPE[build_type]
    build_dir = _build_dir(root)

    if (rc := _check_runtime(cfg, root)) != 0:
        return rc

    if clean:
        clean_tree(root)

    scalar_double = double or cfg.scalar_double
    console.info(f"Tests: {'ON' if tests else 'OFF'}")
    console.info(f"Precision: {'double' if scalar_double else 'single'}")
    console.info(f"Runtime: {cfg.runtime_dir(root)}")

    env = load_build_env(cfg, build_dir)

    # An explicit --double toggles a compile-time switch, so an existing tree must
    # be re-configured for it to take (the in-place re-run updates CMakeCache.txt).
    if _needs_configure(build_dir, cfg.generator) or double:
        console.info(f"Configuring CMake... (type={build_type.value}, tests={'ON' if tests else 'OFF'})")
        if _needs_configure(build_dir, cfg.generator) and build_dir.is_dir():
            shutil.rmtree(build_dir, ignore_errors=True)
        args = _configure_args(cfg, root, build_dir, cmake_build_type, tests,
                               scalar_double=scalar_double)
        rc = _run(args, env, cwd=root)
        if rc != 0:
            console.error("CMake configure failed.")
            return rc
        build_dir.mkdir(parents=True, exist_ok=True)

    console.info("Building...")
    rc = _run([_cmake(cfg), "--build", str(build_dir), "--config", cmake_build_type],
              env, cwd=root)
    if rc == 0:
        console.success("Build completed successfully!")
    else:
        console.error("Build failed.")
    return rc


def test(suite: Suite, filter: str | None = None, repeat: int = 0) -> int:
    console.header("Project Test")
    root = find_project_root()
    cfg = load_config()
    build_dir = _build_dir(root)
    if not build_dir.is_dir():
        console.error("build/ not found. Run `se build` first.")
        return 1

    env = load_build_env(cfg, build_dir)
    cmd = [_ctest(cfg), "--test-dir", str(build_dir), "--output-on-failure"]
    if suite != Suite.all:
        cmd += ["-L", _SUITE_LABEL_REGEX[suite]]
    if filter:
        # gtest_discover_tests registers tests as "Suite.Case"; ctest -R filters
        # those names directly (a richer alternative to --gtest_filter).
        cmd += ["-R", filter]
    if repeat > 0:
        cmd += ["--repeat", f"until-fail:{repeat}"]
        console.info(f"Repeating each test up to {repeat}x (stop on first failure).")
    return _run_drained(cmd, env, cwd=build_dir)


def run(target: str | None = None, sort: bool = False,
        app_args: list[str] | None = None) -> int:
    console.header("Project Run")
    root = find_project_root()
    cfg = load_config()
    build_dir = _build_dir(root)
    if not build_dir.is_dir():
        console.error("build/ not found. Run `se build` first.")
        return 1

    env = load_build_env(cfg, build_dir)

    if sort:
        exe = discovery.select_interactive(build_dir)
    elif target:
        exe = discovery.match_by_name(build_dir, target)
        if exe is None:
            console.error(f"Executable matching '{target}' not found.")
            return 1
    else:
        exe = discovery.match_by_name(build_dir, cfg.target_bin)
        if exe is None:
            console.error(f"Default target '{cfg.target_bin}' not found.")
            return 1

    if exe is None:
        return 1

    console.info(f"Executing: {exe.name}")
    console.console.print(f"[dim]{exe}[/dim]")
    return _run([str(exe), *(app_args or [])], env, cwd=root)


def clean_tree(root: Path) -> None:
    build_dir = _build_dir(root)
    if build_dir.is_dir():
        console.info(f"Removing {build_dir}...")
        shutil.rmtree(build_dir, ignore_errors=True)
        console.success("Build directory cleaned.")
    else:
        console.info("build/ does not exist, nothing to clean.")


def clean() -> int:
    console.header("Project Clean")
    clean_tree(find_project_root())
    return 0


def doxygen() -> int:
    console.header("Doxygen Generation")
    root = find_project_root()
    cfg = load_config()
    env = load_build_env(cfg, _build_dir(root))
    doxyfile = root / "Doxyfile"
    if not doxyfile.is_file():
        console.error("Doxyfile not found at repo root.")
        return 1

    doxy = cfg.expand(cfg.doxygen_exe) if cfg.doxygen_exe else "doxygen"
    if _resolve_exe(doxy, env) == doxy and not Path(doxy).is_file():
        console.error(
            "Doxygen is not installed or not on PATH.\n"
            "  - Windows: winget install DimitriVanHeesch.Doxygen\n"
            "  - Linux:   apt-get install -y doxygen graphviz\n"
            "  - macOS:   brew install doxygen graphviz\n"
            "  - or set doxygen_exe in config.local.toml to an existing doxygen binary.")
        return 1

    (root / "docs").mkdir(exist_ok=True)
    return _run([doxy, "Doxyfile"], env, cwd=root)
