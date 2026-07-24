"""Audio build-and-run logic.

The audio backend is a separate, runtime-independent target gated behind
SE_BUILD_AUDIO. `se audio` reconfigures in place with that flag on (cheap and
incremental — it does not wipe the build tree), builds the `audio_demo` target (the
phase S0 silent block-producing vertical slice), and runs it. It needs only the SDL2
vcpkg package the editor and input backend already require (`ss install`).
"""

from __future__ import annotations

from .. import console
from ..config import find_project_root, load_config
from ..env import load_build_env
from . import discovery
from . import project


def build_and_run(run: bool = True) -> int:
    console.header("Audio")
    root = find_project_root()
    cfg = load_config()
    build_dir = project._build_dir(root)

    if (rc := project._check_runtime(cfg, root)) != 0:
        return rc

    env = load_build_env(cfg, build_dir)

    # In-place configure with the audio flag on. Re-running configure is cheap;
    # CMake picks up the changed -D without a clean rebuild of the runtime.
    args = project._configure_args(cfg, root, build_dir, "Release", tests=False)
    args.append("-DSE_BUILD_AUDIO=ON")
    console.info("Configuring (audio ON)...")
    if (rc := project._run(args, env, cwd=root)) != 0:
        console.error("CMake configure failed.")
        return rc

    console.info("Building audio_demo...")
    rc = project._run(
        [project._cmake(cfg), "--build", str(build_dir),
         "--config", "Release", "--target", "audio_demo"],
        env, cwd=root)
    if rc != 0:
        console.error("Audio build failed.")
        return rc
    console.success("Audio built.")

    if not run:
        return 0

    exe = discovery.match_by_name(build_dir, "audio_demo")
    if exe is None:
        console.error("audio_demo binary not found after build.")
        return 1
    console.info(f"Launching: {exe.name}")
    return project._run([str(exe)], env, cwd=root)
