# The SushiEngine command-line interface

SushiEngine ships with a small command-line tool that drives everything you do day to day:
building the engine, running its tests, launching the editor and the player, running the
headless render and audio probes, baking the planetary and climatology assets, and managing
the Docker image. It is a thin wrapper around CMake and CTest that reads your
machine-specific toolchain paths from a config file so you don't have to retype long
compiler flags.

This guide is the full command reference, in plain English. If you just want to get the
project compiling and running for the first time, see
[CONTRIBUTING.md](../CONTRIBUTING.md) instead — it walks you through the sibling checkout
with SushiRuntime and your first build.

## Installing the CLI

The CLI is a Python package that lives in the `cli/` folder. Install it once and it puts two
commands on your `PATH`:

- **`se`** — short name.
- **`sushiengine`** — long name.

They are identical; use whichever you prefer. Every example below uses `se`.

To install:

```bash
pip install -e cli               # inside a venv/conda env
```

The package depends on `sushicli` (the shared CLI presentation layer used across the Sushi
stack), which is not published to any index — install it once from its sibling checkout
first (`pip install -e ../sushicli`).

Two commands need extra Python packages that everything else does without. They are declared
as optional extras so a plain install stays light, and each command prints its install line
rather than an import traceback when the extra is missing:

```bash
pip install -e cli[planet]        # numpy + requests, for `se planet bake`
pip install -e cli[climatology]   # numpy + requests + netCDF4, for `se climatology bake`
```

### How the CLI finds your compiler

The CLI reads toolchain paths from three files, each overriding the one before it:

- **`cli/config.toml`** — committed, shared defaults (the same for everyone).
- **The workspace-shared `cli/config.local.toml`** that `ss install` writes into the
  SushiStack home, so every module in the workspace resolves the same toolchain.
- **`cli/config.local.toml`** in this repository — your machine's absolute paths (the
  SushiRuntime sibling location, vcpkg location, compiler executables). This file is
  gitignored, so your personal paths never get committed.

`SE_*` environment variables override all three, and a command-line flag overrides
everything. If you're ever unsure what the CLI is actually using, run `se config` to see the
final resolved values and where each one came from.

## Command overview

| Command                                           | What it's for                              |
|---------------------------------------------------|--------------------------------------------|
| `se build` / `test` / `run` / `clean` / `doxygen` | Build, test, run and document the project  |
| `se editor`                                       | Build and launch the ImGui editor          |
| `se player`                                       | Build and launch the ImGui-free player     |
| `se render`                                       | Build and run a headless Vulkan probe      |
| `se audio`                                        | Build and run the audio demo               |
| `se planet`                                       | Bake and inspect planetary terrain assets  |
| `se climatology`                                  | Bake and inspect the climatology asset     |
| `se docker`                                       | Build and run the development container    |
| `se config`                                       | Show the resolved configuration            |
| `se env`                                          | Show the environment builds run under      |

Run any command with `--help` to see its options.

## `se build` / `test` / `run` / `clean` / `doxygen`

This is the group you'll use most.

### `se build`

Configures (if needed) and builds the engine against the SushiRuntime sibling checkout.

```bash
se build                  # Release build (the default)
se build --type debug     # Debug build
se build --clean          # delete the build tree first, then build from scratch
se build --no-test        # skip compiling the test suite (SUSHIENGINE_BUILD_TESTS=OFF)
se build --examples       # also build the worked examples under samples/
se build --backend native # the SYCL-free execution lane, into build/native
```

| Option           | Values                              | Default   |
|------------------|-------------------------------------|-----------|
| `--type`, `-t`   | `release`, `debug`, `relwithdebinfo`| `release` |
| `--clean`        | flag                                | off       |
| `--no-test`      | flag                                | off (tests build) |
| `--examples`     | flag                                | off       |
| `--backend`      | `runtime`, `native`                 | `runtime` |

The test suite is compiled by default; pass `--no-test` to skip it for a faster,
engine-only build.

`--examples` turns on `SUSHIENGINE_BUILD_EXAMPLES`, which declares the demos under
`samples/`. It is off by default because each demo is its own SYCL translation unit and so
its own device compile; a plain `se build` only produces `sandbox` and `pgs_demo`, the two
targets that prove a lane builds at all. Run `se build --examples` first if `se run <demo>`
cannot find the binary you asked for.

`--backend` selects which implementation `SushiEngine::Execution` denotes, which CMake
settles before `project()` and therefore also decides whether SushiRuntime is part of the
build at all:

- `runtime` (the default) — the runtime's SYCL task graph. Needs the sibling checkout and a
  SYCL toolchain, and is the only lane the shells, the samples and the full test suite are
  declared on.
- `native` — the thread-pool backend for platforms the runtime cannot reach. No SushiRuntime
  subproject and no SYCL compiler; it carries `sandbox`, `pgs_demo` and the `Execution`
  conformance binary (`se_native_execution_tests`).

`se test`, `se run` and `se clean` take the same `--backend`, so
`se build --backend native && se test --backend native` builds and runs that lane end to
end.

Each lane configures into its own subdirectory of `build/` — `build/default` for `se build`,
`build/native` for `se build --backend native`, `build/editor` for `se editor`,
`build/player` for `se player` — so no two lanes ever clobber each other's cache. Switching
backend also re-configures from scratch if a tree ever does end up carrying the other lane's
cache. `se render` and `se audio` are the exception: they reconfigure `build/default` in
place with their own flag turned on rather than owning a tree, which is why `se build`
re-runs configure on every invocation instead of only on a fresh tree.

### `se test`

Runs the test suite through CTest. Tests are grouped by label, and you pick a group with
`--suite` (`-s`):

```bash
se test                          # functional suite (the default)
se test --suite unit             # just the unit label
se test --suite functional       # unit + regression + integration
se test --suite all              # every test
se test --suite functional --filter 'Integrator.*'   # ctest -R over test names
se test --repeat 50              # re-run each test up to 50x, stop on first failure
```

| Option           | Values                                                   | Default      |
|------------------|----------------------------------------------------------|--------------|
| `--suite`, `-s`  | `unit`, `regression`, `integration`, `functional`, `all` | `functional` |
| `--filter`, `-f` | a `ctest -R` regex                                       | none         |
| `--repeat`, `-r` | integer, 0 or more                                       | `0` (once)   |
| `--backend`      | `runtime`, `native`                                      | `runtime`    |

`unit`, `regression` and `integration` each select one CTest label. `functional` is the
umbrella over all three, and `all` runs every registered test with no label selection at
all.

`--filter` is a `ctest -R` regex matched against `Suite.Case` test names. `--repeat` is
handy for hunting down flaky tests. For GoogleTest-level options CTest doesn't expose
(shuffling, break on failure), run the binary directly with `se run` and pass flags after
`--`.

### `se run`

Runs a built executable. With no target it runs the project's default, which
`cli/config.toml` sets to `sandbox`.

```bash
se run                                                     # run the default target
se run sandbox                                             # run a specific binary by name
se run --sort                                              # interactively pick from the list
se run se_functional_tests -- --gtest_shuffle --gtest_break_on_failure
```

| Option      | Values              | Default   |
|-------------|---------------------|-----------|
| `TARGET`    | an executable name  | `target_bin` from the config (`sandbox`) |
| `--sort`    | flag                | off       |
| `--backend` | `runtime`, `native` | `runtime` |

The target name is matched exactly first, then by substring. Anything after `--` is
forwarded straight to the program.

### `se clean` and `se doxygen`

```bash
se clean                  # remove the build/default tree
se clean --backend native # remove the build/native tree
se doxygen                # generate Doxygen documentation from the repo's Doxyfile
```

`se clean` takes `--backend` (`runtime` or `native`, default `runtime`) and nothing else.
`se doxygen` takes no options; set `doxygen_exe` in `config.local.toml` if Doxygen is not on
your `PATH`.

## `se editor` — the ImGui editor

```bash
se editor                 # build (Release) and launch the editor
se editor --type debug    # Debug build
se editor --no-run        # build the editor but do not launch it
```

| Option         | Values                               | Default   |
|----------------|--------------------------------------|-----------|
| `--type`, `-t` | `release`, `debug`, `relwithdebinfo` | `release` |
| `--no-run`     | flag                                 | off       |

Configures with `SUSHIENGINE_BUILD_EDITOR=ON`, builds the `sushiengine_editor` target into
its own `build/editor` tree, separate from `se build`'s `build/default`, so the two never
clobber each other's `CMAKE_BUILD_TYPE`, and launches the `se_editor` binary.

## `se player` — the ImGui-free player

```bash
se player                                     # build (Release) and launch the player
se player --type debug                        # Debug build
se player --no-run                            # build the player but do not launch it
se player -- --scene path/to/scene.sushiscene # load a scene explicitly
se player -- --headless --frames 30           # run 30 frames with no window, then exit
```

| Option         | Values                               | Default   |
|----------------|--------------------------------------|-----------|
| `--type`, `-t` | `release`, `debug`, `relwithdebinfo` | `release` |
| `--no-run`     | flag                                 | off       |

Configures with `SUSHIENGINE_BUILD_PLAYER=ON`, builds the `sushiengine_player` target into
its own `build/player` tree, and launches the `se_player` binary. Arguments after `--` are
forwarded to that binary, which accepts:

- `--manifest <path>` — the boot manifest to read. A `boot.json` sitting next to the built
  executable is read automatically when this is not given.
- `--scene <path>` — the scene to load, overriding whatever the manifest named. A bare
  positional path means the same thing.
- `--validation` — turn the Vulkan validation layers on.
- `--headless` — run without a window and exit after a fixed number of frames, which is what
  makes the player runnable on a machine with no display.
- `--frames <n>` — how many frames `--headless` runs. Default 60.

The command line always wins over the manifest, so local testing never means editing the
shipped configuration file.

## `se render` — the headless Vulkan probes

```bash
se render              # build and run the renderer probe (triangle smoke test)
se render --no-run     # build only
se render --probe atmosphere -- --hours 3 --profile column.csv
```

| Option     | Values                             | Default  |
|------------|------------------------------------|----------|
| `--no-run` | flag                               | off      |
| `--probe`  | `render`, `atmosphere`, `golden`   | `render` |

Configures `build/default` in place with `SUSHIENGINE_BUILD_RENDER=ON`, always as a Release
build — there is no `--type` here. Every probe runs without a window, so they work over SSH
and in continuous integration. Anything after the options is passed straight through to the
probe.

`--probe render` (the default) renders a triangle offscreen and reads two pixels back, which
proves the device, shaders, pipeline and submit path came up. It takes no arguments of its
own.

`--probe golden` is the renderer's regression oracle: it renders a fixed scene for a fixed
number of frames and compares it against the references in `tests/goldens/render/` — the
whole frame by hash and thumbprint, and each pass's output by its own hash, so a mismatch
says *which pass* changed rather than only that something did. Run it before and after any
change to a pass.

```bash
se render --probe golden                    # compare
se render --probe golden -- --dump          # ...and write a PPM of any mismatch
se render --probe golden -- --update        # re-record, deliberately
se render --probe golden -- --no-capture    # whole-frame comparison only
se render --probe golden -- --goldens DIR   # read and write references elsewhere
```

A golden is a statement about one GPU and one driver, which is why this is not a CTest case
and why `--update` is an act rather than a remedy: a red run is the harness working. Read
the thumbprint distance and the per-pass lines it prints, and re-record only once the change
is understood and wanted. `--no-capture` skips the per-pass half and renders the way a
shipping build allocates, which is how to tell a real difference from one that exists only
under capture; it cannot be combined with `--update`. See
[`tests/goldens/render/README.md`](../../tests/goldens/render/README.md) for what a golden
covers and what it does not.

`--probe atmosphere` steps the regional weather nest through hours of simulated time in
seconds of wall clock and reports both the observer column and the whole domain's sky — a
measuring instrument rather than a smoke test. Run it with `--help` for the full list; the
ones worth knowing are `--hours` (default 3), `--sample <minutes>` (default 10), `--diurnal`
(drive the sun through a real day instead of holding it), `--profile <path.csv>` (the full
vertical state, one row per level per sample), `--series <path.csv>` (one row per sample),
`--tier` (`low`, `medium`, `high` — the default — or `ultra`), and the `--albedo` /
`--beta` / `--slab` / `--exchange` / `--surface-temp` / `--seed` / `--eddy` / `--pbl-depth`
/ `--pbl-w` / `--critical` / `--humidity` / `--sweeps` overrides that isolate one term of
the physics at a time.

```bash
se render --probe atmosphere -- --hours 11 --diurnal --sample 45
```

The three rightmost columns are the domain's sky — what fraction of columns hold cloud, the
mean coverage and the mean cloud base — because a single column is a noisy sample of a 192²
field, and "is there cloud" is a question about the sky rather than about where the observer
happens to be standing.

## `se audio` — the audio demo

```bash
se audio             # build and run the audio demo
se audio --no-run    # build only
```

| Option     | Values | Default |
|------------|--------|---------|
| `--no-run` | flag   | off     |

Configures `build/default` in place with `SUSHIENGINE_BUILD_AUDIO=ON`, always as a Release
build, and builds the `audio_demo` target: the
block-producing device loop over the SDL2 backend, which verifies the block loop in software
and then best-effort opens a real device — a no-op on a headless host.

## `se planet` — baked planetary terrain

Builds the `.planet` assets the terrain system reads: a cube-sphere height pyramid per body
per quality tier (`docs/design/solar_system_overhaul.md` §5).

```bash
se planet bake                                   # the Moon, compact tier (33 MB download)
se planet bake --body moon --tier standard       # 64 pixels/degree (530 MB download)
se planet bake --depth 5                         # deeper than the source supports; see below
se planet bake --refresh                         # re-download instead of using the cache
se planet bake -o /tmp/moon.planet               # write somewhere else
se planet inspect                                # print a baked asset's provenance and pyramid
se planet inspect assets/planet/moon.standard.planet
```

`se planet bake`:

| Option           | Values                        | Default                                  |
|------------------|-------------------------------|------------------------------------------|
| `--body`, `-b`   | `moon`                        | `moon`                                   |
| `--tier`, `-t`   | `compact`, `standard`         | `compact`                                |
| `--refresh`      | flag                          | off (use the cache)                      |
| `--depth`, `-d`  | integer, 0 to 20              | the depth the source data supports       |
| `--output`, `-o` | a path                        | `assets/planet/<body>.<tier>.planet`     |

`se planet inspect` takes one optional positional path and defaults to
`assets/planet/moon.compact.planet`.

The bake downloads a public-domain topography raster once (LOLA for the Moon, from the NASA
PDS Geosciences Node — no credentials), caches it under `build/planet-cache/`, and writes
the asset to `assets/planet/<body>.<tier>.planet`. Those are gitignored: see
`assets/planet/README.md`. Nothing breaks without one — a body with no baked terrain falls
back to the analytic ground the sky pass already draws.

Three things it does that are worth knowing about:

- **It verifies the grid convention before baking anything.** A raster read with longitude
  mirrored produces a planet that looks entirely reasonable and is wrong everywhere, so the
  bake samples known landmarks (the South Pole–Aitken floor, the far-side highlands) and
  refuses to proceed if they are not where they should be.
- **It reports the depth the *data* supports, not the depth you asked for.** `--depth` may
  go deeper, but the asset still records the source's own resolution, so nothing downstream
  mistakes resampled levels for measurement.
- **It audits its own output.** After writing, it re-reads the asset through every rule the
  engine's reader applies and compares decoded elevations against the source raster. It
  refuses to claim success if anything came back further off than quantisation can account
  for — for the compact lunar tier that is 0.09 m against a 0.18 m step.

## `se climatology` — the baked climatology asset

Builds the climatology the global atmospheric core relaxes toward: three zonal profiles plus
two surface fields, written with their provenance inside the asset.

```bash
se climatology bake                    # about 15 MB of downloads, once
se climatology bake --bands 90         # two-degree latitude bands instead of one
se climatology bake --refresh          # re-download instead of using the cache
se climatology bake -o /tmp/clim.set0  # write somewhere else
se climatology inspect                 # print the grid, extremes and provenance
se climatology inspect /tmp/clim.set0
```

`se climatology bake`:

| Option           | Values           | Default                              |
|------------------|------------------|--------------------------------------|
| `--refresh`      | flag             | off (use the cache)                  |
| `--bands`        | integer, 2 or more | `180` (one-degree bands)           |
| `--output`, `-o` | a path           | `assets/atmosphere/climatology.set0` |

`se climatology inspect` takes one optional positional path and defaults to
`assets/atmosphere/climatology.set0`.

The bake reads NCEP-NCAR Reanalysis 1, NOAA OISST V2 and Natural Earth coastlines — all
public, none needing credentials — derives the profiles and surface fields, and prints an
audit of everything it read and derived. It refuses to write if the land total, the implied
humidity, or the round trip through its own reader disagrees.

## `se docker` — the development container

```bash
se docker build                       # build the `sushiengine` dev image
se docker build --no-cache            # rebuild every layer, ignoring Docker's cache
se docker build --runtime-ref <ref>   # clone a specific SushiRuntime branch/tag/sha
se docker run                         # start the container with the source mounted
se docker run --admin                 # run privileged (--privileged --cap-add=SYS_ADMIN)
se docker run --no-gpu                # skip GPU passthrough (CPU SYCL device still works)
```

| Command            | Option          | Values   | Default |
|--------------------|-----------------|----------|---------|
| `se docker build`  | `--no-cache`    | flag     | off     |
| `se docker build`  | `--runtime-ref` | a git ref| `main`  |
| `se docker run`    | `--admin`       | flag     | off     |
| `se docker run`    | `--no-gpu`      | flag     | off     |

## `se config` and `se env` — diagnostics

```bash
se config         # print the resolved config and where each value came from

se env            # print the environment cmake/ctest/run subprocesses run under
se env --all      # show every variable, not just build-relevant ones
```

`se config` takes no options. `se env` takes `--all`, off by default.

## Building without the CLI

The CLI is a convenience, not a requirement — it only runs CMake and CTest for you. See the
**Getting set up** section of [CONTRIBUTING.md](../CONTRIBUTING.md) for the equivalent raw
`cmake` invocation.
