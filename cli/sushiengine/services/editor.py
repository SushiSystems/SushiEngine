"""Editor build-and-run logic.

The editor is a separate, runtime-independent target gated behind
SE_BUILD_EDITOR. `se editor` reconfigures in place with that flag on (cheap and
incremental — it does not wipe the build tree), builds only the `se_editor`
target, and launches it. The ImGui submodule must be initialized.
"""

from __future__ import annotations

from .. import console
from ..config import find_project_root, load_config
from ..env import load_build_env
from . import discovery
from . import project
from .project import BuildType, _CMAKE_BUILD_TYPE


def _editor_build_dir(root):
    """Editor gets its own build tree so its always-Release-by-default configure
    never clobbers a debug `build/` tree produced by `se build --type debug`."""
    return root / "build-editor"


def build_and_run(run: bool = True, build_type: BuildType = BuildType.release) -> int:
    console.header("Editor")
    root = find_project_root()
    cfg = load_config()
    build_dir = _editor_build_dir(root)
    cmake_build_type = _CMAKE_BUILD_TYPE[build_type]

    imgui = root / "third_party" / "imgui" / "imgui.cpp"
    if not imgui.is_file():
        console.error(
            "Dear ImGui sources not found.\n"
            "  - run: git submodule update --init --recursive")
        return 1

    if (rc := project._check_runtime(cfg, root)) != 0:
        return rc

    env = load_build_env(cfg, build_dir)

    # In-place configure with the editor flag on. Re-running configure is cheap;
    # CMake picks up the changed -D without a clean rebuild of the runtime.
    args = project._configure_args(cfg, root, build_dir, cmake_build_type, tests=False)
    args.append("-DSE_BUILD_EDITOR=ON")
    console.info(f"Configuring (editor ON, type={build_type.value})...")
    if (rc := project._run(args, env, cwd=root)) != 0:
        console.error("CMake configure failed.")
        return rc

    console.info("Building se_editor...")
    rc = project._run(
        [project._cmake(cfg), "--build", str(build_dir),
         "--config", cmake_build_type, "--target", "se_editor"],
        env, cwd=root)
    if rc != 0:
        console.error("Editor build failed.")
        return rc
    console.success("Editor built.")

    if not run:
        return 0

    exe = discovery.match_by_name(build_dir, "se_editor")
    if exe is None:
        console.error("se_editor binary not found after build.")
        return 1
    console.info(f"Launching: {exe.name}")
    return project._run([str(exe)], env, cwd=root)
