# The SushiEngine CLI

SushiEngine ships with a small command-line tool that drives everything you do
day to day: building the engine, running its tests, launching the editor and
the standalone render/audio probes, and managing the Docker image. It is a
thin wrapper around CMake and CTest that reads your machine-specific toolchain
paths from a config file so you don't have to retype long compiler flags.

This guide explains every command in plain English. If you just want to get
the project compiling and running for the first time, see
[CONTRIBUTING.md](CONTRIBUTING.md) instead — it walks you through the sibling
checkout with SushiRuntime and your first build.

## Installing the CLI

The CLI is a Python package that lives in the `cli/` folder. Install it once
and it puts two commands on your `PATH`:

- **`se`** — short name.
- **`sushiengine`** — long name.

They are identical; use whichever you prefer. Every example below uses `se`.

To install:

```bash
pip install -e cli               # inside a venv/conda env
```

The package depends on `sushicli` (the shared CLI presentation layer used
across the Sushi stack), which is not published to any index — install it
once from its sibling checkout first (`pip install -e ../sushicli`).

### How the CLI finds your compiler

The CLI reads toolchain paths from two files:

- **`cli/config.toml`** — committed, shared defaults (the same for everyone).
- **`cli/config.local.toml`** — your machine's absolute paths (the SushiRuntime
  sibling location, vcpkg location, compiler executables). This file is
  gitignored, so your personal paths never get committed.

If you're ever unsure what the CLI is actually using, run `se config` to see
the final resolved values and where each one came from.

## Command overview

| Command                                              | What it's for                                    |
|-------------------------------------------------------|---------------------------------------------------|
| `se build` / `test` / `run` / `clean` / `doxygen`     | Build, test, run, and document the C++ project    |
| `se editor`                                           | Build and launch the ImGui editor                 |
| `se render`                                           | Build and run a headless Vulkan probe             |
| `se audio`                                            | Build and run the audio demo                      |
| `se planet`                                           | Bake and inspect planetary terrain assets         |
| `se docker`                                           | Build and run the development container           |
| `se config`                                           | Show the resolved configuration                   |
| `se env`                                               | Show the environment your builds run under        |

Run any command with `--help` to see its options.

## `se build` / `test` / `run` / `clean` / `doxygen`

This is the group you'll use most.

### `se build`

Configures (if needed) and builds the engine against the SushiRuntime sibling
checkout.

```bash
se build                  # Release build (the default)
se build --type debug     # Debug build
se build --clean          # delete the build tree first, then build from scratch
se build --no-test        # skip compiling the test suite (SE_BUILD_TESTS=OFF)
```

The `--type` (`-t`) option accepts `release`, `debug`, or `relwithdebinfo`. The
test suite is compiled by default; pass `--no-test` to skip it for a faster,
engine-only build.

### `se test`

Runs the test suite through CTest. Tests are grouped by label, and you pick a
group with `--suite` (`-s`):

```bash
se test                          # functional suite (the default)
se test --suite functional       # unit + regression + integration
se test --suite all              # every test
se test --suite functional --filter 'Integrator.*'   # ctest -R over test names
se test --repeat 50               # re-run each test up to 50x, stop on first failure
```

`--filter` (`-f`) is a `ctest -R` regex matched against `Suite.Case` test
names. `--repeat` (`-r`) is handy for hunting down flaky tests. For
GoogleTest-level options CTest doesn't expose (shuffling, break on failure),
run the binary directly with `se run` and pass flags after `--`.

### `se run`

Runs a built executable. With no target it runs the project's default.

```bash
se run                                                     # run the default target
se run sandbox                                             # run a specific binary by name
se run --sort                                              # interactively pick from the list of executables
se run se_functional_tests -- --gtest_shuffle --gtest_break_on_failure
```

The target name is matched exactly first, then by substring. Anything after
`--` is forwarded straight to the program.

### `se clean` and `se doxygen`

```bash
se clean       # remove the build/ tree
se doxygen     # generate Doxygen documentation
```

## `se editor` — the ImGui editor

```bash
se editor                 # build (Release) and launch the editor
se editor --type debug    # Debug build
se editor --no-run        # build the editor but do not launch it
```

Configures with `SE_BUILD_EDITOR=ON` and builds into its own `build-editor/`
tree, separate from `se build`'s `build/`, so the two never clobber each
other's `CMAKE_BUILD_TYPE`.

## `se render` — the headless Vulkan probes

```bash
se render              # build and run the renderer probe (triangle smoke test)
se render --no-run     # build only
se render --probe atmosphere -- --hours 3 --profile column.csv
```

Configures with `SE_BUILD_RENDER=ON`. Every probe runs without a window, so they work
over SSH and in CI.

`--probe render` (the default) renders a triangle offscreen and reads two pixels back,
which proves the device, shaders, pipeline and submit path came up.

`--probe golden` is the renderer's regression oracle (RHI0): it renders a fixed scene
for a fixed number of frames and compares it against the references in
`render/probe/goldens/` — the whole frame by hash and thumbprint, and each pass's output
by its own hash, so a mismatch says *which pass* changed rather than only that something
did. Run it before and after any change to a pass.

```bash
se render --probe golden              # compare
se render --probe golden -- --dump    # ...and write a PPM of any mismatch
se render --probe golden -- --update  # re-record, deliberately
```

A golden is a statement about one GPU and one driver, which is why this is not a ctest
case and why `--update` is an act rather than a remedy: a red run is the harness working.
Read the thumbprint distance and the per-pass lines it prints, and re-record only once
the change is understood and wanted. `--no-capture` skips the per-pass half and renders
the way a shipping build allocates, which is how to tell a real difference from one that
exists only under capture.

`--probe atmosphere` steps the regional weather nest through hours of simulated time in
seconds of wall clock and reports both the observer column and the whole domain's sky —
a measuring instrument rather than a smoke test, so everything after `--` is passed
straight through to it. Run it with `--help` for the full list; the ones worth knowing
are `--hours`, `--diurnal` (drive the sun through a real day instead of holding it),
`--profile <path.csv>` (the full vertical state, one row per level per sample),
`--series <path.csv>` (one row per sample), and the `--sensible` / `--latent` /
`--seed` / `--eddy` / `--pbl-depth` / `--pbl-w` / `--critical` / `--humidity` /
`--sweeps` overrides that isolate one term of the physics at a time.

```bash
se render --probe atmosphere -- --hours 11 --diurnal --sample 45
```

The three rightmost columns are the domain's sky — what fraction of columns hold cloud,
the mean coverage and the mean cloud base — because a single column is a noisy sample of
a 192² field, and "is there cloud" is a question about the sky rather than about where
the observer happens to be standing.

## `se audio` — the audio demo

```bash
se audio             # build and run the audio demo
se audio --no-run     # build only
```

Configures with `SE_BUILD_AUDIO=ON`.

## `se planet` — baked planetary terrain

Builds the `.planet` assets the terrain system reads: a cube-sphere height pyramid per body
per quality tier (`docs/slop/solar_system_overhaul.md` §5).

```bash
pip install -e cli[planet]                       # numpy + requests, an optional extra
se planet bake                                   # the Moon, compact tier (33 MB download)
se planet bake --body moon --tier standard       # 64 pixels/degree (530 MB download)
se planet bake --depth 5                         # deeper than the source supports; see below
se planet bake --refresh                         # re-download instead of using the cache
se planet bake -o /tmp/moon.planet               # write somewhere else
se planet inspect                                # print a baked asset's provenance and pyramid
```

The bake downloads a public-domain topography raster once (LOLA for the Moon, from the NASA
PDS Geosciences Node — no credentials), caches it under `build/planet-cache/`, and writes the
asset to `assets/planet/<body>.<tier>.planet`. Those are gitignored: see
`assets/planet/README.md`. Nothing breaks without one — a body with no baked terrain falls
back to the analytic ground the sky pass already draws.

Three things it does that are worth knowing about:

- **It verifies the grid convention before baking anything.** A raster read with longitude
  mirrored produces a planet that looks entirely reasonable and is wrong everywhere, so the
  bake samples known landmarks (the South Pole–Aitken floor, the far-side highlands) and
  refuses to proceed if they are not where they should be.
- **It reports the depth the *data* supports, not the depth you asked for.** `--depth` may go
  deeper, but the asset still records the source's own resolution, so nothing downstream
  mistakes resampled levels for measurement.
- **It audits its own output.** After writing, it re-reads the asset through every rule the
  engine's reader applies and compares decoded elevations against the source raster. It
  refuses to claim success if anything came back further off than quantisation can account
  for — for the compact lunar tier that is 0.09 m against a 0.18 m step.

## `se docker` — the development container

```bash
se docker build                       # build the `sushiengine` dev image
se docker build --no-cache            # rebuild every layer, ignoring Docker's cache
se docker build --runtime-ref <ref>   # clone a specific SushiRuntime branch/tag/sha (default: main)
se docker run                         # start the container with the source mounted
se docker run --admin                 # run privileged (--privileged --cap-add=SYS_ADMIN)
se docker run --no-gpu                # skip GPU passthrough (CPU SYCL device still works)
```

## `se config` and `se env` — diagnostics

```bash
se config         # print the resolved config and where each value came from

se env            # print the environment cmake/ctest/run subprocesses run under
se env --all      # show every variable, not just build-relevant ones
```

## Building without the CLI

The CLI is a convenience, not a requirement — it only runs CMake and CTest for
you. See the **Getting set up** section of [CONTRIBUTING.md](CONTRIBUTING.md)
for the equivalent raw `cmake` invocation.
