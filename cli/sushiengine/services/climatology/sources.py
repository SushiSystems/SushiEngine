"""Where T0's climatology comes from, and how it gets here.

One module, one job: name the sources, fetch them once, and record what was fetched so
the provenance the asset carries is assembled from the same table the downloader used
rather than typed out again beside it.

Every source here is public and needs no credentials. That is not a convenience — it is
what makes the bake reproducible by anyone with the checkout, which is the only way a
claim about the asset can be checked rather than believed.
"""

from __future__ import annotations

import hashlib

from dataclasses import dataclass
from pathlib import Path
from typing import Dict

# The reanalysis base period every gridded source below is drawn from. Kept as one
# constant because a climatology assembled from mismatched base periods is a mean state
# of no particular era, and the seam between the pieces would be invisible.
BASE_PERIOD = "1981-2010"

_PSL = "https://downloads.psl.noaa.gov/Datasets/"
_NE = ("https://raw.githubusercontent.com/nvkelso/natural-earth-vector/master/"
       "geojson/")


@dataclass(frozen=True)
class Source:
    """One downloadable input, with the attribution it obliges us to carry."""

    key: str
    url: str
    filename: str
    describes: str
    """What this file is *for* — printed in the audit so a reader can tell whether the
    file that failed to download matters to the field they care about."""
    attribution: str


SOURCES: Dict[str, Source] = {
    "uwnd": Source(
        key="uwnd",
        url=_PSL + f"ncep.reanalysis.derived/pressure/uwnd.mon.ltm.{BASE_PERIOD}.nc",
        filename="uwnd.mon.ltm.nc",
        describes="zonal wind on pressure levels -> the two jet profiles T1 relaxes toward",
        attribution=("NCEP-NCAR Reanalysis 1 monthly long-term means, zonal wind; "
                     "Kalnay et al. (1996), NOAA PSL, Boulder, Colorado, USA, "
                     "https://psl.noaa.gov/"),
    ),
    "air": Source(
        key="air",
        url=_PSL + f"ncep.reanalysis.derived/pressure/air.mon.ltm.{BASE_PERIOD}.nc",
        filename="air.mon.ltm.nc",
        describes="air temperature on pressure levels -> saturated column water",
        attribution=("NCEP-NCAR Reanalysis 1 monthly long-term means, air temperature; "
                     "Kalnay et al. (1996), NOAA PSL, Boulder, Colorado, USA, "
                     "https://psl.noaa.gov/"),
    ),
    "pr_wtr": Source(
        key="pr_wtr",
        url=_PSL + (f"ncep.reanalysis.derived/surface/pr_wtr.eatm.mon.ltm."
                    f"{BASE_PERIOD}.nc"),
        filename="pr_wtr.eatm.mon.ltm.nc",
        describes="observed precipitable water -> audit cross-check only, never baked",
        attribution=("NCEP-NCAR Reanalysis 1 monthly long-term means, precipitable "
                     "water; Kalnay et al. (1996), NOAA PSL, Boulder, Colorado, USA, "
                     "https://psl.noaa.gov/"),
    ),
    "sst": Source(
        key="sst",
        url=_PSL + f"noaa.oisst.v2/sst.ltm.{BASE_PERIOD}.nc",
        filename="sst.ltm.nc",
        describes="monthly sea surface temperature",
        attribution=("NOAA Optimum Interpolation SST V2 monthly long-term means; "
                     "Reynolds et al. (2002), NOAA PSL, Boulder, Colorado, USA, "
                     "https://psl.noaa.gov/"),
    ),
    "land": Source(
        key="land",
        url=_NE + "ne_50m_land.geojson",
        filename="ne_50m_land.geojson",
        describes="land polygons -> land area fraction",
        attribution=("Natural Earth 1:50m physical land vectors, public domain, "
                     "https://www.naturalearthdata.com/"),
    ),
}


def cache_dir(root: Path) -> Path:
    """Where downloads are kept between bakes.

    Under the build tree rather than the source tree: these are 15 MB of inputs that
    reproduce from the network on demand, and a source tree is for things that do not.
    """
    return root / "build" / "climatology-cache"


def fetch(source: Source, into: Path, refresh: bool = False) -> Path:
    """Downloads @p source into @p into, reusing the cached copy unless @p refresh.

    Downloads to a temporary name and renames on success, so an interrupted fetch leaves
    no half a file behind that the next run would happily read as a whole one.

    @param source  Which input to fetch.
    @param into    The cache directory; created if absent.
    @param refresh Re-download even when a cached copy exists.
    @return The path to the local file.
    @raises RuntimeError on any non-200 response.
    """
    import requests  # deferred: only the bake needs it, and it is an optional extra

    into.mkdir(parents=True, exist_ok=True)
    dest = into / source.filename
    if dest.exists() and not refresh:
        return dest

    partial = dest.with_suffix(dest.suffix + ".partial")
    with requests.get(source.url, timeout=600, stream=True) as response:
        if response.status_code != 200:
            raise RuntimeError(
                f"{source.key}: HTTP {response.status_code} from {source.url}")
        with partial.open("wb") as handle:
            for chunk in response.iter_content(chunk_size=1 << 20):
                handle.write(chunk)
    partial.replace(dest)
    return dest


def digest(path: Path) -> str:
    """A short SHA-256 of @p path, so the audit names the exact bytes that were read."""
    hasher = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            hasher.update(chunk)
    return hasher.hexdigest()[:16]
