"""Player build-and-run logic.

The ImGui-free runtime shell (PLATFORM0 S5) is a separate, ImGui-independent
target gated behind SE_BUILD_PLAYER. `se player` reconfigures in place with
that flag on (cheap and incremental — it does not wipe the build tree),
builds only the `se_player` target, and launches it. Arguments after `--`
are forwarded to it, so `se player -- --scene physics_sample.sushiscene
--validation` works — see `applications/player/source/main.cpp` for what it accepts.
"""

from __future__ import annotations

from typing import Sequence

from .. import console
from ..config import find_project_root, load_config
from ..env import load_build_env
from . import discovery
from . import project
from .project import BuildType, _CMAKE_BUILD_TYPE


def _player_build_dir(root):
    """The player gets its own build tree, the same reason build/editor does:
    its always-Release-by-default configure must never clobber a debug
    `build/default` tree produced by `se build --type debug`."""
    return root / "build" / "player"


def build_and_run(run: bool = True, build_type: BuildType = BuildType.release,
                  args: Sequence[str] = ()) -> int:
    console.header("Player")
    root = find_project_root()
    cfg = load_config()
    build_dir = _player_build_dir(root)
    cmake_build_type = _CMAKE_BUILD_TYPE[build_type]

    if (rc := project._check_runtime(cfg, root)) != 0:
        return rc

    env = load_build_env(cfg, build_dir)

    # In-place configure with the player flag on. Re-running configure is cheap;
    # CMake picks up the changed -D without a clean rebuild of the runtime.
    configure = project._configure_args(cfg, root, build_dir, cmake_build_type, tests=False)
    configure.append("-DSE_BUILD_PLAYER=ON")
    console.info(f"Configuring (player ON, type={build_type.value})...")
    if (rc := project._run(configure, env, cwd=root)) != 0:
        console.error("CMake configure failed.")
        return rc

    console.info("Building se_player...")
    rc = project._run(
        [project._cmake(cfg), "--build", str(build_dir),
         "--config", cmake_build_type, "--target", "se_player"],
        env, cwd=root)
    if rc != 0:
        console.error("Player build failed.")
        return rc
    console.success("Player built.")

    if not run:
        return 0

    exe = discovery.match_by_name(build_dir, "se_player")
    if exe is None:
        console.error("se_player binary not found after build.")
        return 1
    console.info(f"Launching: {exe.name}")
    return project._run([str(exe), *args], env, cwd=root)
