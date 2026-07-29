"""SushiEngine developer CLI (`se`).

Thin Typer layer: commands only parse arguments and delegate to the service
layer in ``sushiengine.services``.
"""

from __future__ import annotations

from pathlib import Path
from typing import List, Optional

import typer

from .services import audio as audio_svc
from .services import climatology as climatology_svc
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

climatology_app = typer.Typer(help="Bake and inspect the T0 climatology asset.",
                              no_args_is_help=True)
app.add_typer(climatology_app, name="climatology")


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
    type: BuildType = typer.Option(
        BuildType.release, "--type", "-t", case_sensitive=False,
        help="Build type: release | debug | relwithdebinfo."),
    no_run: bool = typer.Option(
        False, "--no-run", help="Build the editor but do not launch it."),
):
    """Build and launch the ImGui editor (configures with SE_BUILD_EDITOR=ON).

    Uses its own build-editor/ tree, separate from `se build`'s build/, so the
    two never clobber each other's CMAKE_BUILD_TYPE.
    """
    raise typer.Exit(editor_svc.build_and_run(run=not no_run, build_type=type))


# --------------------------------------------------------------------------- #
# render
# --------------------------------------------------------------------------- #
@app.command("render", context_settings={"allow_extra_args": True,
                                         "ignore_unknown_options": True})
def render(
    ctx: typer.Context,
    no_run: bool = typer.Option(
        False, "--no-run", help="Build the probe but do not run it."),
    probe: str = typer.Option(
        "render", "--probe",
        help="Which headless probe to build: 'render' (triangle smoke test) or "
             "'atmosphere' (steps the regional nest and reports its column)."),
):
    """Build and run a headless Vulkan probe (configures with SE_BUILD_RENDER=ON).

    Arguments after the options are passed through to the probe, so
    `se render --probe atmosphere -- --hours 3 --profile p.csv` works.
    """
    raise typer.Exit(
        render_svc.build_and_run(run=not no_run, probe=probe, args=ctx.args))


# --------------------------------------------------------------------------- #
# audio
# --------------------------------------------------------------------------- #
@app.command("audio")
def audio(
    no_run: bool = typer.Option(
        False, "--no-run", help="Build the audio demo but do not run it."),
):
    """Build and run the audio demo (configures with SE_BUILD_AUDIO=ON).

    The phase S0 vertical slice: the silent block-producing device loop over the
    SDL2 backend. Verifies the block loop in software, then best-effort opens a real
    device (a no-op on a headless host).
    """
    raise typer.Exit(audio_svc.build_and_run(run=not no_run))


# --------------------------------------------------------------------------- #
# climatology
# --------------------------------------------------------------------------- #
@climatology_app.command("bake")
def climatology_bake(
    refresh: bool = typer.Option(
        False, "--refresh", help="Re-download the sources instead of using the cache."),
    bands: int = typer.Option(
        climatology_svc.DEFAULT_BANDS, "--bands", min=2,
        help="Latitude bands for the zonal profiles (one degree by default)."),
    output: Optional[Path] = typer.Option(
        None, "--output", "-o",
        help="Where to write; defaults to assets/atmosphere/climatology.set0."),
):
    """Bake T0's climatology from public reanalysis and coastline data.

    Downloads about 15 MB once (NCEP-NCAR Reanalysis 1, NOAA OISST V2, Natural Earth —
    no credentials), derives the three zonal profiles the global core relaxes toward plus
    the two surface fields, and writes the asset with its provenance inside it. Prints an
    audit of everything it read and derived, and refuses to write if the land total, the
    implied humidity, or the round trip disagrees.

    Needs the optional extras: pip install -e cli[climatology]
    """
    raise typer.Exit(climatology_svc.bake(refresh=refresh, bands=bands, output=output))


@climatology_app.command("inspect")
def climatology_inspect(
    path: Optional[Path] = typer.Argument(
        None, help="Asset to read; defaults to assets/atmosphere/climatology.set0."),
):
    """Print a baked asset's grid, extremes, and provenance."""
    raise typer.Exit(climatology_svc.inspect(path))


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
