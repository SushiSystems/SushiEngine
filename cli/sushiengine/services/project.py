"""Project build / test / run / clean / doxygen logic.

The cmake / ctest invocations are issued directly via subprocess with the
snapshotted build env. The engine configures against the SushiRuntime sibling's
bundled clang++ and vcpkg, so the configure args resolve those from the runtime
checkout (or the config overrides) rather than provisioning anything.

Every command here takes the execution backend the tree is configured for
(:class:`ExecutionBackend`), because it decides both what CMake is asked for and which
subdirectory of ``build/`` the answer lands in.
"""

from __future__ import annotations

import enum
import shutil
import subprocess
from pathlib import Path
from typing import Optional

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


class ExecutionBackend(str, enum.Enum):
    """Which implementation SushiEngine::Execution denotes for a configure.

    The two lanes are mutually exclusive and are chosen before `project()` runs, so a
    configure commits to one for the whole tree (see the root CMakeLists.txt and
    cmake/ProjectOptions.cmake):

    * runtime — SushiRuntime's SYCL task graph. Needs the sibling checkout and a SYCL
      toolchain, and is the lane every shell, sample and test target is declared on.
    * native  — the thread-pool backend, for platforms SushiRuntime cannot reach. Adds
      no SushiRuntime subproject and needs no SYCL compiler; declares only the sandbox,
      pgs_demo and the Execution conformance tests.
    """

    runtime = "runtime"
    native = "native"


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


# Which subdirectory of build/ each execution backend configures into. They are
# separate trees because the two lanes disagree about the contents of the cache down
# to the compiler: the runtime lane bakes in the SYCL clang++ and the SushiRuntime
# subproject, the native lane neither. Reusing one tree for both would need a wipe on
# every switch, which is the same cost with a worse failure mode when it is forgotten.
_BACKEND_BUILD_DIRECTORY = {
    ExecutionBackend.runtime: "default",
    ExecutionBackend.native: "native",
}


def _build_dir(root: Path, backend: ExecutionBackend = ExecutionBackend.runtime) -> Path:
    """The tree *backend*'s lane configures into. Each lane gets its own subdirectory of
    build/ so a lane's CMAKE_BUILD_TYPE, backend and -D flags never clobber another's
    cache; the paths match the binaryDir of the CMakePresets.json preset of the same name."""
    return root / "build" / _BACKEND_BUILD_DIRECTORY[backend]


def _build_hint(backend: ExecutionBackend) -> str:
    """The `se build` invocation that produces *backend*'s tree, for error messages."""
    if backend == ExecutionBackend.runtime:
        return "se build"
    return f"se build --backend {backend.value}"


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


def _cached_value(build_dir: Path, entry: str) -> Optional[str]:
    """The value CMake baked into build_dir's cache for *entry*, or None if absent.

    Reads CMakeCache.txt rather than shelling out to `cmake -L`, so it costs nothing and
    works on a tree whose configure failed part way through.

    :param build_dir: The build tree to inspect.
    :param entry: The cache entry's name, without its `:TYPE` suffix.
    :return: The entry's value, or None when the tree is unconfigured or lacks the entry.
    """
    cache = build_dir / "CMakeCache.txt"
    if not cache.is_file():
        return None
    prefix = entry + ":"
    for line in cache.read_text(encoding="utf-8", errors="ignore").splitlines():
        if line.startswith(prefix):
            return line.split("=", 1)[1].strip()
    return None


def _needs_configure(build_dir: Path, generator: str, build_type: str = None,
                     backend: ExecutionBackend = None) -> bool:
    if not build_dir.is_dir():
        return True
    sentinel = "build.ninja" if generator == "Ninja" else "Makefile"
    if not (build_dir / sentinel).is_file():
        return True
    # Single-config generators (Ninja, Make) bake CMAKE_BUILD_TYPE into the
    # existing cache; `cmake --build --config X` is silently ignored for them,
    # so a stale tree must be reconfigured rather than reused as-is.
    if build_type is not None and _cached_value(build_dir, "CMAKE_BUILD_TYPE") != build_type:
        return True
    # Neither can a tree configured for the other execution backend: whether the
    # SushiRuntime subproject and the SYCL toolchain are part of the build is settled
    # before project() and recorded in the cache, so re-running configure over it would
    # only produce a half-converted tree. The two lanes already have separate
    # directories, so this catches a tree configured by hand or from a CMake preset.
    if backend is not None and \
            _cached_value(build_dir, "SUSHIENGINE_EXECUTION_BACKEND") != backend.value:
        return True
    return False


def _check_runtime(cfg: Config, root: Path) -> int:
    """Fail early (with guidance) when the SushiRuntime sibling is missing.

    The runtime lane cannot configure without it — the runtime provides the SYCL target
    and the add_sycl_to_target command the engine's CMakeLists pulls in. Callers on the
    native lane do not call this: that lane adds no such subproject.
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
                    build_type: str, tests: bool, examples: bool = False,
                    backend: ExecutionBackend = ExecutionBackend.runtime) -> list[str]:
    cxx = cfg.resolved_compiler(root)
    vcpkg = cfg.resolved_vcpkg(root)

    args = [
        _cmake(cfg), "-S", str(root), "-B", str(build_dir), "-G", cfg.generator,
        f"-DCMAKE_BUILD_TYPE={build_type}",
        f"-DCMAKE_CXX_COMPILER={cxx}",
        f"-DSUSHIENGINE_EXECUTION_BACKEND={backend.value}",
        f"-DSUSHIENGINE_BUILD_TESTS={'ON' if tests else 'OFF'}",
        f"-DSUSHIENGINE_BUILD_EXAMPLES={'ON' if examples else 'OFF'}",
    ]
    # Only the runtime lane add_subdirectory()s the sibling checkout. Naming its location
    # on the native lane would bake a path into the cache that nothing there reads.
    if backend == ExecutionBackend.runtime:
        args.append(f"-DSUSHIRUNTIME_DIR={cfg.runtime_dir(root)}")
    # On Windows clang++ also drives the C probe; point both slots at it.
    if cfg.is_windows:
        args.append(f"-DCMAKE_C_COMPILER={cxx}")
        if cfg.ninja_exe:
            args.append(f"-DCMAKE_MAKE_PROGRAM={cfg.expand(cfg.ninja_exe)}")
        if cfg.pkgconf_exe:
            args.append(f"-DPKG_CONFIG_EXECUTABLE={cfg.expand(cfg.pkgconf_exe)}")
        if cfg.rc_exe:
            args.append(f"-DCMAKE_RC_COMPILER={cfg.expand(cfg.rc_exe)}")
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
          examples: bool = False,
          backend: ExecutionBackend = ExecutionBackend.runtime) -> int:
    console.header("Project Build")
    root = find_project_root()
    cfg = load_config()
    cmake_build_type = _CMAKE_BUILD_TYPE[build_type]
    build_dir = _build_dir(root, backend)

    # The native lane configures and builds with no SushiRuntime checkout at all, so
    # demanding one there would refuse a build that works.
    if backend == ExecutionBackend.runtime and (rc := _check_runtime(cfg, root)) != 0:
        return rc

    if clean:
        clean_tree(root, backend)

    console.info(f"Backend: {backend.value}")
    console.info(f"Tests: {'ON' if tests else 'OFF'}")
    console.info(f"Examples: {'ON' if examples else 'OFF'}")
    if backend == ExecutionBackend.runtime:
        console.info(f"Runtime: {cfg.runtime_dir(root)}")

    env = load_build_env(cfg, build_dir)

    # Configure on every build, not only when the tree has to be thrown away. `se render`
    # and `se audio` reach the same tree with their own -D flags, so a cache left by one of
    # them can disagree with what this call was asked for — and the disagreement is silent:
    # the header above would report "Tests: ON" while the cache said OFF and nothing built
    # the suite, so `se test` would then pass against a stale binary. Re-running configure in
    # place is cheap; CMake picks the changed -D up without rebuilding what did not change.
    fresh = _needs_configure(build_dir, cfg.generator, cmake_build_type, backend)
    if fresh:
        console.info(f"Configuring CMake... (backend={backend.value}, type={build_type.value}, "
                     f"tests={'ON' if tests else 'OFF'})")
        if build_dir.is_dir():
            shutil.rmtree(build_dir, ignore_errors=True)
    else:
        console.info(f"Reconfiguring in place (tests={'ON' if tests else 'OFF'})...")
    rc = _run(_configure_args(cfg, root, build_dir, cmake_build_type, tests, examples, backend),
              env, cwd=root)
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


def test(suite: Suite, filter: str | None = None, repeat: int = 0,
         backend: ExecutionBackend = ExecutionBackend.runtime) -> int:
    console.header("Project Test")
    root = find_project_root()
    cfg = load_config()
    build_dir = _build_dir(root, backend)
    if not build_dir.is_dir():
        console.error(f"{build_dir} not found. Run `{_build_hint(backend)}` first.")
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
        app_args: list[str] | None = None,
        backend: ExecutionBackend = ExecutionBackend.runtime) -> int:
    console.header("Project Run")
    root = find_project_root()
    cfg = load_config()
    build_dir = _build_dir(root, backend)
    if not build_dir.is_dir():
        console.error(f"{build_dir} not found. Run `{_build_hint(backend)}` first.")
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


def clean_tree(root: Path, backend: ExecutionBackend = ExecutionBackend.runtime) -> None:
    build_dir = _build_dir(root, backend)
    if build_dir.is_dir():
        console.info(f"Removing {build_dir}...")
        shutil.rmtree(build_dir, ignore_errors=True)
        console.success("Build directory cleaned.")
    else:
        console.info(f"{build_dir} does not exist, nothing to clean.")


def clean(backend: ExecutionBackend = ExecutionBackend.runtime) -> int:
    console.header("Project Clean")
    clean_tree(find_project_root(), backend)
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
