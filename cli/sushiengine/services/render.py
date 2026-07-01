"""Renderer build-and-run logic.

The Vulkan renderer is a separate, runtime-independent target gated behind
SE_BUILD_RENDER. `se render` reconfigures in place with that flag on (cheap and
incremental — it does not wipe the build tree), builds the headless `render_probe`
target, and runs it to confirm a device comes up. The Vulkan/VMA/vk-bootstrap vcpkg
packages must be provisioned (`ss install`).
"""

from __future__ import annotations

from .. import console
from ..config import find_project_root, load_config
from ..env import load_build_env
from . import discovery
from . import project


def build_and_run(run: bool = True) -> int:
    console.header("Renderer")
    root = find_project_root()
    cfg = load_config()
    build_dir = project._build_dir(root)

    if (rc := project._check_runtime(cfg, root)) != 0:
        return rc

    env = load_build_env(cfg, build_dir)

    # In-place configure with the render flag on. Re-running configure is cheap;
    # CMake picks up the changed -D without a clean rebuild of the runtime.
    args = project._configure_args(cfg, root, build_dir, "Release", tests=False)
    args.append("-DSE_BUILD_RENDER=ON")
    console.info("Configuring (render ON)...")
    if (rc := project._run(args, env, cwd=root)) != 0:
        console.error("CMake configure failed.")
        return rc

    console.info("Building render_probe...")
    rc = project._run(
        [project._cmake(cfg), "--build", str(build_dir),
         "--config", "Release", "--target", "render_probe"],
        env, cwd=root)
    if rc != 0:
        console.error("Renderer build failed.")
        return rc
    console.success("Renderer built.")

    if not run:
        return 0

    exe = discovery.match_by_name(build_dir, "render_probe")
    if exe is None:
        console.error("render_probe binary not found after build.")
        return 1
    console.info(f"Launching: {exe.name}")
    return project._run([str(exe)], env, cwd=root)
