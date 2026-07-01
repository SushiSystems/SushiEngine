"""Docker environment management for the SushiEngine dev image.

The image bundles the clang++ -fsycl toolchain, a SushiRuntime sibling checkout,
and the `se` CLI, so a fresh container can `se build` straight away. See
the repo-root Dockerfile for what the image provisions.
"""

from __future__ import annotations

import subprocess

from .. import console
from ..config import find_project_root

IMAGE = "sushiengine"


def _run(cmd: list[str], cwd) -> int:
    console.command(" ".join(cmd))
    return subprocess.run(cmd, cwd=str(cwd)).returncode


def build(no_cache: bool = False, runtime_ref: str | None = None) -> int:
    """Build the dev image from the repo-root Dockerfile.

    @param no_cache Rebuild every layer, ignoring the Docker layer cache.
    @param runtime_ref The SushiRuntime branch/tag/sha to clone into the image
                       (passed as --build-arg SUSHIRUNTIME_REF). Defaults to the
                       Dockerfile's own default (main) when unset.
    """
    console.header("Docker Build")
    root = find_project_root()
    cmd = ["docker", "build", "-t", IMAGE]
    if no_cache:
        console.info("Building without the layer cache (--no-cache).")
        cmd.append("--no-cache")
    if runtime_ref:
        console.info(f"Pinning the SushiRuntime sibling to '{runtime_ref}'.")
        cmd += ["--build-arg", f"SUSHIRUNTIME_REF={runtime_ref}"]
    cmd.append(".")
    return _run(cmd, cwd=root)


def run(admin: bool = False, no_gpu: bool = False) -> int:
    """Start an interactive container with the engine source mounted live.

    The engine checkout is bind-mounted over /workspace/sushiengine so edits on
    the host are visible inside; the runtime sibling baked into the image stays
    in place at /workspace/sushiruntime.

    @param admin Run privileged (--privileged --cap-add=SYS_ADMIN), e.g. for perf.
    @param no_gpu Skip GPU passthrough (the CPU SYCL device still works).
    """
    console.header("Docker Run")
    root = find_project_root()
    cmd = ["docker", "run", "-it", "--rm"]
    if no_gpu:
        console.info("Running without GPU passthrough (--no-gpu).")
    else:
        cmd += ["--gpus", "all"]
    if admin:
        console.info("Running in ADMIN mode (privileged)")
        cmd += ["--privileged", "--cap-add=SYS_ADMIN"]
    cmd += ["-v", f"{root}:/workspace/sushiengine", IMAGE, "/bin/bash"]
    return _run(cmd, cwd=root)
