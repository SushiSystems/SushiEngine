"""SushiEngine developer CLI (`se`).

Thin Typer layer: commands only parse arguments and delegate to the service
layer in ``sushiengine.services``.
"""

from __future__ import annotations

from typing import List, Optional

import typer

from .services import diag as diag_svc
from .services import docker as docker_svc
from .services import editor as editor_svc
from .services import project as project_svc
from .services import render as render_svc
from .services.project import BuildType, Suite

app = typer.Typer(
    name="se",
    help="SushiEngine developer CLI — build, test, and run the engine.",
    no_args_is_help=True,
    add_completion=False,
    rich_markup_mode="rich",
)

docker_app = typer.Typer(help="Build and run the containerized dev environment.",
                         no_args_is_help=True)
app.add_typer(docker_app, name="docker")


# --------------------------------------------------------------------------- #
# build / test / run / clean / doxygen
# --------------------------------------------------------------------------- #
@app.command("build")
def build(
    type: BuildType = typer.Option(
        BuildType.release, "--type", "-t", case_sensitive=False,
        help="Build type: release | debug | relwithdebinfo."),
    clean: bool = typer.Option(
        False, "--clean", help="Delete the build tree before configuring."),
    no_test: bool = typer.Option(
        False, "--no-test",
        help="Skip compiling the test suite (SE_BUILD_TESTS=OFF). Tests build by default."),
):
    """Configure and build the project against the SushiRuntime sibling."""
    raise typer.Exit(project_svc.build(type, clean, tests=not no_test))


@app.command("test")
def test(
    suite: Suite = typer.Option(
        Suite.functional, "--suite", "-s", case_sensitive=False,
        help="Which CTest label group to run ('all' runs every test)."),
    filter: Optional[str] = typer.Option(
        None, "--filter", "-f",
        help="ctest -R regex over 'Suite.Case' test names."),
    repeat: int = typer.Option(
        0, "--repeat", "-r", min=0,
        help="Re-run each test up to N times, stopping on the first failure "
             "(ctest --repeat until-fail). Handy for hunting flaky tests."),
):
    """Run the test suite via CTest labels.

    For GTest-level knobs (shuffle, break-on-failure) run the binary directly:
    se run se_functional_tests -- --gtest_shuffle --gtest_break_on_failure
    """
    raise typer.Exit(project_svc.test(suite, filter, repeat))


@app.command(
    "run",
    context_settings={"allow_extra_args": True, "ignore_unknown_options": True},
)
def run(
    ctx: typer.Context,
    target: Optional[str] = typer.Argument(
        None, help="Executable name to run (exact, then substring match)."),
    sort: bool = typer.Option(
        False, "--sort", help="Interactively pick an executable."),
):
    """Run a built executable. Args after `--` are forwarded to it.

    Example: se run sandbox
    """
    raise typer.Exit(project_svc.run(target=target, sort=sort, app_args=list(ctx.args)))


@app.command("clean")
def clean():
    """Remove the build/ tree."""
    raise typer.Exit(project_svc.clean())


@app.command("doxygen")
def doxygen():
    """Generate Doxygen documentation."""
    raise typer.Exit(project_svc.doxygen())


# --------------------------------------------------------------------------- #
# editor
# --------------------------------------------------------------------------- #
@app.command("editor")
def editor(
    no_run: bool = typer.Option(
        False, "--no-run", help="Build the editor but do not launch it."),
):
    """Build and launch the ImGui editor (configures with SE_BUILD_EDITOR=ON)."""
    raise typer.Exit(editor_svc.build_and_run(run=not no_run))


# --------------------------------------------------------------------------- #
# render
# --------------------------------------------------------------------------- #
@app.command("render")
def render(
    no_run: bool = typer.Option(
        False, "--no-run", help="Build the renderer probe but do not run it."),
):
    """Build and run the Vulkan renderer probe (configures with SE_BUILD_RENDER=ON)."""
    raise typer.Exit(render_svc.build_and_run(run=not no_run))


# --------------------------------------------------------------------------- #
# docker
# --------------------------------------------------------------------------- #
@docker_app.command("build")
def docker_build(
    no_cache: bool = typer.Option(
        False, "--no-cache", help="Rebuild every layer, ignoring the Docker cache."),
    runtime_ref: Optional[str] = typer.Option(
        None, "--runtime-ref",
        help="SushiRuntime branch/tag/sha to clone into the image (default: main)."),
):
    """Build the `sushiengine` dev image (toolchain + runtime sibling + CLI)."""
    raise typer.Exit(docker_svc.build(no_cache=no_cache, runtime_ref=runtime_ref))


@docker_app.command("run")
def docker_run(
    admin: bool = typer.Option(
        False, "--admin", help="Run privileged (--privileged --cap-add=SYS_ADMIN)."),
    no_gpu: bool = typer.Option(
        False, "--no-gpu", help="Skip GPU passthrough (CPU SYCL device still works)."),
):
    """Start an interactive container with the engine source mounted live."""
    raise typer.Exit(docker_svc.run(admin=admin, no_gpu=no_gpu))


# --------------------------------------------------------------------------- #
# config / env (diagnostics)
# --------------------------------------------------------------------------- #
@app.command("config")
def config():
    """Print the resolved config and where each value came from."""
    raise typer.Exit(diag_svc.config_show())


@app.command("env")
def env(
    all: bool = typer.Option(
        False, "--all", help="Show every variable, not just build-relevant ones."),
):
    """Print the environment cmake/ctest/run subprocesses execute under."""
    raise typer.Exit(diag_svc.env_dump(show_all=all))


if __name__ == "__main__":
    app()
