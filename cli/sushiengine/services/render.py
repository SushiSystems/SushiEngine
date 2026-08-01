"""Renderer build-and-run logic.

The Vulkan renderer is a separate, runtime-independent target gated behind
SE_BUILD_RENDER. `se render` reconfigures in place with that flag on (cheap and
incremental — it does not wipe the build tree), builds a headless probe target, and
runs it. The Vulkan/VMA/vk-bootstrap vcpkg packages must be provisioned (`ss install`).

These probes share this path because they share everything that makes it slow — the
configure and the `sushi_render` build — and differ only in which executable comes out:

* `render_probe` renders the triangle offscreen and reads two pixels back, which
  proves the device, shaders, pipeline and submit path came up. The default.
* `atmosphere_probe` steps the regional nest through hours of simulated weather and
  writes the observer column's vertical profile. A measuring instrument rather than a
  smoke test, so it takes arguments; see `--help`.
* `render_golden` renders a fixed scene and compares it — whole frame and per pass —
  against the references in `render/probe/goldens/` (RHI0). Run it before and after a
  render change; `--update` re-records, which is a deliberate act and not a remedy for
  a red run. It takes arguments, so `se render --probe golden -- --dump` works.
"""

from __future__ import annotations

from typing import Sequence

from .. import console
from ..config import find_project_root, load_config
from ..env import load_build_env
from . import discovery
from . import project

#: Probe targets `se render` can build, and what each is for.
PROBES = {
    "render": "render_probe",
    "atmosphere": "atmosphere_probe",
    "golden": "render_golden",
}


def build_and_run(run: bool = True, probe: str = "render",
                  args: Sequence[str] = ()) -> int:
    """Configures with the renderer on, builds a probe, and optionally runs it.

    :param run: Run the probe after building it.
    :param probe: Which probe to build; a key of :data:`PROBES`.
    :param args: Arguments passed through to the probe when it runs.
    :return: A process exit code; non-zero on any failed step.
    """
    target = PROBES.get(probe)
    if target is None:
        console.error(f"Unknown probe '{probe}'. Choose one of: "
                      f"{', '.join(sorted(PROBES))}.")
        return 2

    console.header("Renderer")
    root = find_project_root()
    cfg = load_config()
    build_dir = project._build_dir(root)

    if (rc := project._check_runtime(cfg, root)) != 0:
        return rc

    env = load_build_env(cfg, build_dir)

    # In-place configure with the render flag on. Re-running configure is cheap;
    # CMake picks up the changed -D without a clean rebuild of the runtime.
    configure = project._configure_args(cfg, root, build_dir, "Release", tests=False)
    configure.append("-DSE_BUILD_RENDER=ON")
    console.info("Configuring (render ON)...")
    if (rc := project._run(configure, env, cwd=root)) != 0:
        console.error("CMake configure failed.")
        return rc

    console.info(f"Building {target}...")
    rc = project._run(
        [project._cmake(cfg), "--build", str(build_dir),
         "--config", "Release", "--target", target],
        env, cwd=root)
    if rc != 0:
        console.error("Renderer build failed.")
        return rc
    console.success("Renderer built.")

    if not run:
        return 0

    exe = discovery.match_by_name(build_dir, target)
    if exe is None:
        console.error(f"{target} binary not found after build.")
        return 1
    console.info(f"Launching: {exe.name}")
    return project._run([str(exe), *args], env, cwd=root)
