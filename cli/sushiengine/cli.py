"""SushiEngine developer CLI (`se`).

Thin Typer layer: commands only parse arguments and delegate to the service
layer in ``sushiengine.services``.
"""

from __future__ import annotations

from typing import List, Optional

import typer

from .services import diag as diag_svc
from .services import project as project_svc
from .services.project import BuildType, Suite

app = typer.Typer(
    name="se",
    help="SushiEngine developer CLI — build, test, and run the engine.",
    no_args_is_help=True,
    add_completion=False,
    rich_markup_mode="rich",
)

project_app = typer.Typer(help="C++ project build / test / run management.",
                          no_args_is_help=True)
config_app = typer.Typer(help="Inspect resolved configuration.",
                         no_args_is_help=True)
env_app = typer.Typer(help="Inspect the build environment.",
                      no_args_is_help=True)
app.add_typer(project_app, name="project")
app.add_typer(config_app, name="config")
app.add_typer(env_app, name="env")


# --------------------------------------------------------------------------- #
# project
# --------------------------------------------------------------------------- #
@project_app.command("build")
def project_build(
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


@project_app.command("test")
def project_test(
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
    se project run se_functional_tests -- --gtest_shuffle --gtest_break_on_failure
    """
    raise typer.Exit(project_svc.test(suite, filter, repeat))


@project_app.command(
    "run",
    context_settings={"allow_extra_args": True, "ignore_unknown_options": True},
)
def project_run(
    ctx: typer.Context,
    target: Optional[str] = typer.Argument(
        None, help="Executable name to run (exact, then substring match)."),
    sort: bool = typer.Option(
        False, "--sort", help="Interactively pick an executable."),
):
    """Run a built executable. Args after `--` are forwarded to it.

    Example: se project run sandbox
    """
    raise typer.Exit(project_svc.run(target=target, sort=sort, app_args=list(ctx.args)))


@project_app.command("clean")
def project_clean():
    """Remove the build/ tree."""
    raise typer.Exit(project_svc.clean())


@project_app.command("doxygen")
def project_doxygen():
    """Generate Doxygen documentation."""
    raise typer.Exit(project_svc.doxygen())


# --------------------------------------------------------------------------- #
# config / env (diagnostics)
# --------------------------------------------------------------------------- #
@config_app.command("show")
def config_show():
    """Print the resolved config and where each value came from."""
    raise typer.Exit(diag_svc.config_show())


@env_app.command("dump")
def env_dump(
    all: bool = typer.Option(
        False, "--all", help="Show every variable, not just build-relevant ones."),
):
    """Print the environment cmake/ctest/run subprocesses execute under."""
    raise typer.Exit(diag_svc.env_dump(show_all=all))


if __name__ == "__main__":
    app()
