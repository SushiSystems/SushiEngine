"""Bakes T0's climatology asset from public reanalysis and coastline data.

`docs/slop/atmosphere_system.md` §4 calls sourcing and baking these fields a task of Phase
C rather than an afterthought, and this is that task. The output is a single `SET0` blob
that `Climatology::adopt` reads; the engine never touches the network, and the bake never
touches the engine.

**The audit is the deliverable, not the file.** A climatology is data nobody can eyeball,
baked from sources nobody re-downloads, feeding a mean state that decides whether the
world has weather at all. So every run prints what it read, what it derived, what it threw
away, and the extremes of every field it wrote -- and checks three things that would
otherwise fail silently:

- the rasterised land total against the known land fraction of the Earth, which is what
  catches a coastline dataset whose polygons are not disjoint;
- the implied column relative humidity, which is the saturation derivation being divided
  by the observed water that was never baked -- if the thermodynamics were wrong this is
  where it shows;
- a full round trip of the written bytes, because a writer that has never been read is a
  writer whose layout has never been tested.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, List

import numpy as np

from ... import console
from ...config import find_project_root
from . import asset as asset_module
from . import landmask, reanalysis, sources

# Where the baked asset lands, relative to the project root.
ASSET_PATH = Path("assets") / "atmosphere" / "climatology.set0"

# Latitude bands the zonal profiles are written on. One degree, matching §4's table and
# `ClimatologyProfileGrid`'s default; the reanalysis behind them is 2.5 degrees, so this is
# an interpolation onto the engine's grid rather than a claim of extra information.
DEFAULT_BANDS = 180

# The reanalysis resolves 2.5 degrees, so a one-degree profile is smooth by construction
# and this is a bound on interpolation overshoot, not on the atmosphere.
MAX_PLAUSIBLE_WIND_MPS = 80.0


def _provenance(digests: Dict[str, str], bands: int, months: int,
                surface: str) -> str:
    """Assembles the attribution string that travels inside the asset.

    Built from the same source table the downloader used, so an added source cannot be
    read but not credited.
    """
    lines: List[str] = [
        f"SushiEngine T0 climatology, base period {sources.BASE_PERIOD}.",
        f"Zonal profiles: {bands} latitude bands x {months} months. Surface: {surface}.",
        "Upper/lower winds are zonal means at "
        f"{reanalysis.UPPER_LEVEL_HPA:g}/{reanalysis.LOWER_LEVEL_HPA:g} hPa. "
        "Saturated column water is Tetens over liquid, integrated "
        f"{reanalysis.COLUMN_BASE_HPA:g}-{reanalysis.COLUMN_TOP_HPA:g} hPa.",
        "Sources:",
    ]
    for key, source in sources.SOURCES.items():
        if key in digests:
            lines.append(f"  - {source.attribution} [sha256:{digests[key]}]")
    return "\n".join(lines)


def _report_profile(name: str, values: np.ndarray, unit: str,
                    latitudes: np.ndarray) -> None:
    """Prints where a profile peaks and how deep its trough runs."""
    annual = values.mean(axis=0)
    peak = int(np.argmax(annual))
    trough = int(np.argmin(annual))
    console.info(
        f"  {name:<22} annual peak {annual[peak]:+7.2f} {unit} at "
        f"{latitudes[peak]:+6.1f}deg, min {annual[trough]:+7.2f} at "
        f"{latitudes[trough]:+6.1f}deg, monthly range "
        f"[{values.min():+.2f}, {values.max():+.2f}]")


def bake(refresh: bool = False, bands: int = DEFAULT_BANDS,
         output: Path | None = None) -> int:
    """Downloads the sources, derives the fields, writes and verifies the asset.

    @param refresh Re-download even when the cache holds a copy.
    @param bands   Latitude bands for the zonal profiles.
    @param output  Where to write; defaults to `assets/atmosphere/climatology.set0`.
    @return A process exit code.
    """
    console.header("Climatology bake")
    root = find_project_root()
    cache = sources.cache_dir(root)

    try:
        import numpy  # noqa: F401
    except ImportError:
        console.error("numpy is required to bake the climatology.")
        console.info("  pip install -e cli[climatology]")
        return 1

    # ---------------------------------------------------------------- download
    console.info(f"Cache: {cache}")
    paths: Dict[str, Path] = {}
    digests: Dict[str, str] = {}
    for key, source in sources.SOURCES.items():
        try:
            path = sources.fetch(source, cache, refresh=refresh)
        except Exception as error:  # network, HTTP, disk
            console.error(f"{key}: {error}")
            console.info(f"  needed for: {source.describes}")
            return 1
        paths[key] = path
        digests[key] = sources.digest(path)
        console.info(f"  {key:<7} {path.stat().st_size / 1e6:6.2f} MB  "
                     f"sha256:{digests[key]}  {source.describes}")

    # ------------------------------------------------------------- derivation
    try:
        profiles, months = reanalysis.read_zonal_profiles(
            paths["uwnd"], paths["air"], paths["pr_wtr"], bands)
        sea_surface, surface_longitudes, surface_latitudes = \
            reanalysis.read_sea_surface_temperature(paths["sst"])
        fraction, rings = landmask.rasterise(
            paths["land"], surface_longitudes, surface_latitudes)
    except Exception as error:
        console.error(str(error))
        return 1

    if sea_surface.shape[0] != months:
        console.error(f"SST has {sea_surface.shape[0]} months and the reanalysis has "
                      f"{months}; the asset carries one month count for both.")
        return 1

    # ------------------------------------------------------------------ audit
    latitudes = reanalysis.band_centres(bands)
    console.info(f"Zonal profiles: {bands} bands x {months} months "
                 f"(reanalysis 2.5deg, interpolated)")
    _report_profile("upper wind", profiles.upper_wind_mps, "m/s", latitudes)
    _report_profile("lower wind", profiles.lower_wind_mps, "m/s", latitudes)
    _report_profile("saturated water", profiles.saturation_kg_per_m2, "kg/m2", latitudes)

    shear = profiles.upper_wind_mps - profiles.lower_wind_mps
    console.info(f"  vertical shear         max {shear.max():+.2f} m/s -- this is the "
                 f"number that decides whether the mean state makes storms")

    # The cross-check: observed precipitable water over derived saturation. It is never
    # written to the asset; it exists to say whether the thermodynamics above is physical.
    with np.errstate(divide="ignore", invalid="ignore"):
        humidity = (profiles.observed_water_kg_per_m2 /
                    np.maximum(profiles.saturation_kg_per_m2, 1e-6))
    # Poleward of 60 the cross-check stops being a check. The reanalysis carries slightly
    # negative precipitable water over the Antarctic plateau -- an assimilation artefact
    # where the true value is a fraction of a kg/m2 -- and the saturation ceiling there is
    # inflated by below-ground extrapolation, so the ratio of the two is meaningless in
    # both directions at once. Neither field is baked from that region's ratio; the check
    # is reported where it can be read.
    tropics = np.abs(latitudes) < 60.0
    checked = humidity[:, tropics]
    console.info(f"  implied column RH      {checked.mean():.2f} mean within 60deg, "
                 f"range [{checked.min():.2f}, {checked.max():.2f}] -- observed water "
                 f"over derived saturation, never baked")
    negative = int((profiles.observed_water_kg_per_m2 < 0.0).sum())
    if negative:
        console.info(f"  (the cross-check ignores {negative} polar band-months where the "
                     f"reanalysis' own precipitable water is slightly negative)")
    if not (0.2 <= checked.mean() <= 0.9):
        console.error("Implied column relative humidity is not physical; the saturation "
                      "derivation is wrong and the asset was not written.")
        return 1

    land_total = landmask.area_weighted_total(fraction)
    console.info(f"Surface: {surface_longitudes}x{surface_latitudes} "
                 f"({360 / surface_longitudes:g}deg), {rings} coastline rings")
    console.info(f"  land fraction          {land_total:.3f} area-weighted "
                 f"(expected {landmask.EXPECTED_LAND_FRACTION:.3f}), "
                 f"{int((fraction > 0).sum())} cells touched by land")
    if abs(land_total - landmask.EXPECTED_LAND_FRACTION) > \
            landmask.LAND_FRACTION_TOLERANCE:
        console.error("Rasterised land total is off; the coastline polygons are probably "
                      "not disjoint, so the even-odd fill cancelled overlaps to water. "
                      "The asset was not written.")
        return 1

    console.info(f"  sea surface            [{sea_surface.min():.1f}, "
                 f"{sea_surface.max():.1f}] K; land cells filled from their own latitude, "
                 f"never a sentinel and never the global mean")

    # The fill is the part of the SST field that is invented rather than measured, so it
    # is the part worth checking. Under Antarctica every longitude is land, and a fill
    # that reached for the global mean would quietly write a tropical ocean there.
    def _at(latitude_degrees: float) -> float:
        row = int((latitude_degrees + 90.0) / 180.0 * surface_latitudes)
        return float(sea_surface[:, min(max(row, 0), surface_latitudes - 1)].mean())

    polar = _at(-80.0)
    console.info(f"  filled-cell check      80S {polar:.1f} K, 80N {_at(80.0):.1f} K, "
                 f"equator {_at(0.0):.1f} K")
    if polar > 278.0:
        console.error(f"The fill under Antarctica reads {polar:.1f} K. Land cells are "
                      f"taking a temperature from somewhere that is not their own "
                      f"latitude; the asset was not written.")
        return 1

    if np.abs(profiles.upper_wind_mps).max() > MAX_PLAUSIBLE_WIND_MPS or \
            np.abs(profiles.lower_wind_mps).max() > MAX_PLAUSIBLE_WIND_MPS:
        console.error(f"A zonal-mean wind exceeds {MAX_PLAUSIBLE_WIND_MPS:g} m/s, which a "
                      f"2.5deg reanalysis cannot produce; the interpolation overshot.")
        return 1

    # ------------------------------------------------------------------ write
    surface_label = (f"{surface_longitudes}x{surface_latitudes} "
                     f"({360 / surface_longitudes:g} degree)")
    built = asset_module.ClimatologyAsset(
        upper_wind_mps=profiles.upper_wind_mps,
        lower_wind_mps=profiles.lower_wind_mps,
        saturation_kg_per_m2=profiles.saturation_kg_per_m2,
        land_fraction=fraction,
        sea_surface_kelvin=sea_surface,
        provenance=_provenance(digests, bands, months, surface_label))

    try:
        blob = asset_module.pack(built)
        asset_module.verify(blob, built)
    except ValueError as error:
        console.error(f"The asset failed its own round trip: {error}")
        return 1

    destination = output or (root / ASSET_PATH)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_bytes(blob)
    console.success(f"Wrote {destination} ({len(blob) / 1e6:.2f} MB), round trip verified.")
    console.info(f"  provenance travels inside the asset "
                 f"({len(built.provenance.encode('utf-8'))} bytes)")
    return 0


def inspect(path: Path | None = None) -> int:
    """Prints the header and provenance of an already-baked asset.

    The counterpart to the bake's audit: a checked-out asset is bytes nobody remembers
    baking, and this is how the question "what mean state is this scene actually using"
    gets an answer without a rebuild.
    """
    console.header("Climatology")
    target = path or (find_project_root() / ASSET_PATH)
    if not target.is_file():
        console.error(f"No climatology asset at {target}.")
        console.info("  se climatology bake")
        return 1

    try:
        read = asset_module.unpack(target.read_bytes())
    except ValueError as error:
        console.error(f"{target}: {error}")
        return 1

    longitudes, latitudes = read.surface_shape
    console.info(f"{target} ({target.stat().st_size / 1e6:.2f} MB)")
    console.info(f"  {read.bands} latitude bands x {read.months} months, "
                 f"surface {longitudes}x{latitudes}")
    latitude_centres = reanalysis.band_centres(read.bands)
    _report_profile("upper wind", read.upper_wind_mps, "m/s", latitude_centres)
    _report_profile("lower wind", read.lower_wind_mps, "m/s", latitude_centres)
    _report_profile("saturated water", read.saturation_kg_per_m2, "kg/m2",
                    latitude_centres)
    console.info("Provenance:")
    for line in read.provenance.splitlines():
        console.info(f"  {line}")
    return 0
