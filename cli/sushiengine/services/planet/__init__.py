"""`se planet bake` -- baked planetary terrain, from public elevation data.

Three things happen here, in this order, and the middle one is the reason the other two are
worth trusting:

- the sources are fetched and their digests recorded, so the provenance the asset carries is
  assembled from the same table the downloader used rather than typed out beside it;
- the grid convention is **checked against known landmarks** before a single tile is baked,
  because a mirrored body is entirely plausible to look at and impossible to notice later;
- the written asset is re-read and audited against the source raster, so the accuracy claim
  is a measured number rather than an assertion -- a writer that has never been read is a
  writer whose layout has never been tested.

**Nothing numeric is imported at module scope, and that is load-bearing.** `cli.py` imports
this package to register `se planet`, so anything imported here is imported by *every* `se`
command. An `import numpy` on that line makes `se build` fail on a machine that has no numpy
-- which is every machine that only wants to compile the engine, since the bake's
dependencies are an optional extra precisely so they are not needed for that.
"""

from __future__ import annotations

import math

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional

from ... import console
from ...config import find_project_root

# `sources` only, and only because it is pure standard library -- it names the downloads and
# defers `requests` to the moment one is fetched.
from . import sources

# Where baked assets land, relative to the project root. Ignored by git: the compact lunar
# tier is 17 MB and the deeper ones are hundreds, so the asset reproduces from this command
# rather than from the history. `assets/planet/README.md` says so beside them.
ASSET_DIRECTORY = Path("assets") / "planet"

# How many tiles the accuracy audit decodes back out of the finished asset. Spread across
# the pyramid rather than clustered, so a systematic error at one depth cannot hide.
AUDIT_TILES = 24


@dataclass(frozen=True)
class Body:
    """A body the baker knows how to build, and the tiers it offers."""

    key: str
    display_name: str
    ephemeris_id: int
    """Ordinal in `Astro::BodyId`, carried in the asset header."""
    equatorial_radius_metres: float
    inverse_flattening: float
    tiers: Dict[str, str]
    """Tier name to the key in `sources.SOURCES` that backs it."""

    @property
    def semi_axes(self):
        """The reference ellipsoid's three semi-axes, metres, polar axis on z."""
        polar = (self.equatorial_radius_metres * (1.0 - 1.0 / self.inverse_flattening)
                 if self.inverse_flattening > 0.0 else self.equatorial_radius_metres)
        return (self.equatorial_radius_metres, self.equatorial_radius_metres, polar)

    @property
    def face_arc_metres(self) -> float:
        """Arc length of one cube-face edge at the equator.

        Only used to choose the data depth, so the equatorial value is close enough for a
        body whose flattening is small -- and it is exact for the spherical ones this
        currently ships.
        """
        return 2.0 * math.pi * self.equatorial_radius_metres / 4.0


BODIES: Dict[str, Body] = {
    "moon": Body(
        key="moon",
        display_name="Moon",
        ephemeris_id=4,
        equatorial_radius_metres=1737400.0,
        inverse_flattening=0.0,
        tiers={"compact": "lola_16", "standard": "lola_64"},
    ),
}

DEFAULT_BODY = "moon"
DEFAULT_TIER = "compact"


def _load_bake_modules():
    """Imports the parts that need numpy, turning a missing extra into advice.

    Deferred rather than imported at the top of the file: see the note beside the imports.

    @return `(cube, dem, pack)`, or None once the reason has been printed.
    """
    try:
        from . import cube, dem, pack
    except ImportError as error:
        console.error(f"`se planet` needs a package that is not installed: "
                      f"{getattr(error, 'name', None) or error}.")
        # The backslash escapes the bracket for Rich, which would otherwise read "[planet]"
        # as a style tag and print the line with the extra silently missing -- which is the
        # one word the reader needs.
        console.info(r"  pip install -e cli\[planet]")
        console.info("  (an optional extra, so `se build` never needs it)")
        return None
    return cube, dem, pack


def _provenance(body: Body, tier: str, source: sources.Source, digest: str,
                depth: int, data_depth: int, tile_count: int) -> str:
    """Assembles the attribution string that travels inside the asset.

    Built from the same source table the downloader used, so an added source cannot be read
    but not credited. The resolution claim is stated here too, because an asset that has
    been copied away from this command is the only thing left to ask.
    """
    lines: List[str] = [
        f"SushiEngine planetary terrain: {body.display_name}, tier '{tier}'.",
        f"Cube-sphere quadtree to depth {depth} ({tile_count} tiles); "
        f"measured data supports depth {data_depth}. Elevations are metres above a "
        f"reference ellipsoid with semi-axes "
        f"{body.semi_axes[0]:.1f}/{body.semi_axes[1]:.1f}/{body.semi_axes[2]:.1f} m.",
        "Anything below the data depth is resampled, not measured.",
        "Sources:",
        f"  - {source.attribution} [sha256:{digest}]",
    ]
    return "\n".join(lines)


def bake(body_key: str = DEFAULT_BODY, tier: str = DEFAULT_TIER, refresh: bool = False,
         depth: Optional[int] = None, output: Optional[Path] = None) -> int:
    """Downloads a body's topography, reprojects it onto the cube-sphere, writes the asset.

    @param body_key Which body, a key of @ref BODIES.
    @param tier     Which quality tier, a key of the body's `tiers`.
    @param refresh  Re-download the source even when a cached copy exists.
    @param depth    Bake to this quadtree depth instead of the source's own data depth.
                    Deeper costs four times the tiles per level and adds no measurement.
    @param output   Where to write; defaults to `assets/planet/<body>.<tier>.planet`.
    @return A process exit code: 0 on success.
    """
    modules = _load_bake_modules()
    if modules is None:
        return 1
    cube, dem, pack = modules

    console.header("Planet bake")

    body = BODIES.get(body_key)
    if body is None:
        console.error(f"Unknown body '{body_key}'. Known: {', '.join(sorted(BODIES))}.")
        return 1
    source_key = body.tiers.get(tier)
    if source_key is None:
        console.error(f"{body.display_name} has no '{tier}' tier. "
                      f"Known: {', '.join(sorted(body.tiers))}.")
        return 1
    source = sources.SOURCES[source_key]

    root = find_project_root()
    cache = sources.cache_directory(root)
    destination = output or (root / ASSET_DIRECTORY /
                             f"{body.key}.{tier}.planet")

    console.info(f"Body:   {body.display_name} (ephemeris id {body.ephemeris_id})")
    console.info(f"Tier:   {tier} <- {source.key}, {source.describes}")
    console.info(f"Cache:  {cache}")
    if source.approximate_bytes > 100_000_000:
        console.warn(f"  {source.key} is about "
                     f"{source.approximate_bytes / 1e6:.0f} MB; the first run will take a while.")

    try:
        label_path = sources.fetch(source.label_url, source.label_filename, cache, refresh)
        image_path = sources.fetch(source.url, source.filename, cache, refresh)
    except Exception as error:
        console.error(f"{source.key}: {error}")
        console.info(f"  needed for: {source.describes}")
        return 1

    try:
        grid = dem.open_grid(image_path, label_path, source.key)
    except Exception as error:
        console.error(str(error))
        return 1

    console.info(f"Source: {grid.values.shape[1]}x{grid.values.shape[0]} at "
                 f"{grid.pixels_per_degree:g} pixels/degree, "
                 f"{grid.metres_per_pixel:.1f} m/pixel, reference radius "
                 f"{grid.reference_radius_metres:.0f} m")

    # The convention audit, before anything is baked. A mirrored or flipped grid produces a
    # body that looks completely reasonable and is wrong everywhere.
    ok, rows = dem.check_landmarks(grid)
    console.info("Landmark check (verifies the grid convention, not the data):")
    for name, expected, measured, tolerance in rows:
        mark = "ok " if abs(measured - expected) <= tolerance else "BAD"
        console.info(f"  {mark} {name:<52} expected {expected:+8.0f} m, "
                     f"read {measured:+8.0f} m")
    if not ok:
        console.error("The raster does not read as the body it claims to be. That is a "
                      "longitude or latitude convention error in this reader, not bad data "
                      "-- baking it would produce a mirrored planet.")
        return 1

    data_depth = cube.depth_for_resolution(body.face_arc_metres, grid.metres_per_pixel)
    bake_depth = data_depth if depth is None else depth
    if not 0 <= bake_depth <= cube.MAX_TILE_DEPTH:
        console.error(f"Depth {bake_depth} is outside 0..{cube.MAX_TILE_DEPTH}.")
        return 1

    tile_count = cube.pyramid_tile_count(bake_depth)
    texel = body.face_arc_metres / ((cube.TILE_GRID_SIZE - 1) * (1 << bake_depth))
    estimated = tile_count * cube.TILE_SAMPLE_COUNT * 2 / 1e6
    console.info(f"Depth:  {bake_depth} ({tile_count} tiles, texel {texel:.1f} m, "
                 f"about {estimated:.0f} MB)")
    if bake_depth > data_depth:
        console.warn(f"  The source supports depth {data_depth}. Levels past it are "
                     f"resampled, and the asset reports {data_depth} as its data depth so "
                     f"nothing downstream mistakes them for measurement.")

    digest = sources.digest(image_path)
    provenance = _provenance(body, tier, source, digest, bake_depth, data_depth, tile_count)

    writer = pack.PackWriter(destination, body_id=body.ephemeris_id,
                             semi_axes=body.semi_axes, height_data_depth=data_depth,
                             tile_count=tile_count, provenance=provenance)
    step = max(1, tile_count // 8)
    written = 0
    try:
        for face, tile_depth, x, y in cube.tile_addresses(bake_depth):
            latitude, longitude = cube.tile_geographic(face, tile_depth, x, y)
            writer.add(face, tile_depth, x, y, grid.sample(latitude, longitude))
            written += 1
            if written % step == 0 or written == tile_count:
                console.info(f"  baked {written}/{tile_count} tiles "
                             f"({100.0 * written / tile_count:.0f}%)")
        writer.close()
    except Exception as error:
        console.error(f"The bake failed after {written} tiles: {error}")
        return 1

    return _audit(destination, grid, cube, pack)


def _audit(destination: Path, grid, cube, pack) -> int:
    """Re-reads the finished asset and measures it against the source it came from.

    Two questions, both of which a bake can get wrong while looking successful: does the
    asset parse under every rule the engine's reader applies, and do the elevations that
    come back out match the raster they went in from, within the quantisation step the
    format promises?

    @return A process exit code.
    """
    try:
        with destination.open("rb") as handle:
            header, records = pack.read_header(handle)
            console.info(f"Verify: header and {len(records)} index records parse; "
                         f"data depth {header.height_data_depth}")

            keys = sorted(records)
            chosen = (keys if len(keys) <= AUDIT_TILES
                      else [keys[i * (len(keys) - 1) // (AUDIT_TILES - 1)]
                            for i in range(AUDIT_TILES)])
            worst_error = 0.0
            worst_step = 0.0
            for key in chosen:
                record = records[key]
                face = (key >> 45) & 0x7
                depth = (key >> 40) & 0x1F
                x = (key >> 20) & 0xFFFFF
                y = key & 0xFFFFF
                latitude, longitude = cube.tile_geographic(face, depth, x, y)
                expected = grid.sample(latitude, longitude)
                measured = pack.read_tile(handle, record)
                error = float(abs(measured - expected).max())
                step = pack.quantisation_step(record.quantised_minimum,
                                              record.quantised_maximum)
                worst_error = max(worst_error, error)
                worst_step = max(worst_step, step)
    except Exception as error:
        console.error(f"The asset failed its own verification: {error}")
        return 1

    console.info(f"  accuracy over {len(chosen)} tiles: worst deviation from the source "
                 f"{worst_error:.3f} m, against a quantisation step of up to "
                 f"{worst_step:.3f} m")
    if worst_error > worst_step:
        console.error("An elevation came back further from the source than quantisation "
                      "can account for. That is a reprojection or layout error, not "
                      "rounding.")
        return 1

    size = destination.stat().st_size
    console.success(f"Wrote {destination} ({size / 1e6:.1f} MB), "
                    f"round trip and accuracy verified.")
    console.info("  provenance travels inside the asset; `se planet inspect` prints it")
    return 0


def inspect(path: Optional[Path] = None) -> int:
    """Prints what a baked asset says about itself.

    @param path The asset; defaults to the compact lunar tier.
    @return A process exit code.
    """
    modules = _load_bake_modules()
    if modules is None:
        return 1
    _, _, pack = modules

    console.header("Planet asset")
    target = path or (find_project_root() / ASSET_DIRECTORY /
                      f"{DEFAULT_BODY}.{DEFAULT_TIER}.planet")
    if not target.exists():
        console.error(f"No planet asset at {target}.")
        console.command(f"  se planet bake --body {DEFAULT_BODY} --tier {DEFAULT_TIER}")
        return 1

    try:
        with target.open("rb") as handle:
            header, records = pack.read_header(handle)
    except Exception as error:
        console.error(f"{target}: {error}")
        return 1

    console.info(f"{target} ({target.stat().st_size / 1e6:.1f} MB)")
    console.info(f"  body id           {header.body_id}")
    console.info(f"  semi-axes         {header.semi_axes[0]:.1f} / "
                 f"{header.semi_axes[1]:.1f} / {header.semi_axes[2]:.1f} m")
    console.info(f"  tiles             {header.tile_count}")
    console.info(f"  data depth        {header.height_data_depth}")
    depths: Dict[int, int] = {}
    for key in records:
        depth = (key >> 40) & 0x1F
        depths[depth] = depths.get(depth, 0) + 1
    console.info("  tiles per depth   " +
                 ", ".join(f"{depth}:{count}" for depth, count in sorted(depths.items())))
    elevations = [(record.grid_minimum, record.grid_maximum) for record in records.values()]
    if elevations:
        console.info(f"  elevation range   {min(low for low, _ in elevations):+.0f} to "
                     f"{max(high for _, high in elevations):+.0f} m")
    console.info("  provenance:")
    for line in header.provenance.splitlines():
        console.info(f"    {line}")
    return 0
