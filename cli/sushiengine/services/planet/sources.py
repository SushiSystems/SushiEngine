"""Where a body's elevation comes from, and how it gets here.

One module, one job: name the sources, fetch them once, and record what was fetched so the
provenance the asset carries is assembled from the same table the downloader used rather
than typed out again beside it.

Every source here is public domain and needs no credentials. That is not a convenience --
it is what makes the bake reproducible by anyone with the checkout, which is the only way a
claim about the asset can be checked rather than believed. Sources with better resolution
but a licence attached (Copernicus GLO-30 is the notable one) are deliberately absent from
the defaults: a licence obligation should be something a user opts into knowingly.

Pure standard library, like the climatology table it mirrors: `requests` is deferred to the
moment something is actually fetched, so importing this module costs `se build` nothing.
"""

from __future__ import annotations

import hashlib

from dataclasses import dataclass
from pathlib import Path
from typing import Dict

_PDS_LOLA = ("https://pds-geosciences.wustl.edu/lro/lro-l-lola-3-rdr-v1/lrolol_1xxx/"
             "data/lola_gdr/cylindrical/img/")


@dataclass(frozen=True)
class Source:
    """One downloadable raster, with the attribution it obliges us to carry."""

    key: str
    url: str
    filename: str
    label_url: str
    label_filename: str
    describes: str
    """What this raster is *for* -- printed in the audit so a reader can tell whether the
    file that failed to download matters to the tier they asked for."""
    approximate_bytes: int
    """Roughly how large the download is, so the audit can warn before it starts rather
    than after half an hour."""
    metres_per_pixel: float
    """The source's real ground resolution at the equator. This, not the depth the bake was
    asked for, is what sets the asset's `height_data_depth` -- the number that keeps the
    engine honest about where measurement stops and synthesis begins."""
    attribution: str


SOURCES: Dict[str, Source] = {
    "lola_16": Source(
        key="lola_16",
        url=_PDS_LOLA + "ldem_16.img",
        filename="ldem_16.img",
        label_url=_PDS_LOLA + "ldem_16.lbl",
        label_filename="ldem_16.lbl",
        describes="Moon global topography at 16 pixels/degree -> the compact tier",
        approximate_bytes=33_177_600,
        metres_per_pixel=1895.21,
        attribution=("LRO LOLA Global Digital Elevation Model, 16 pixels/degree "
                     "(LDEM_16); Smith et al., NASA PDS Geosciences Node, "
                     "https://pds-geosciences.wustl.edu/ -- public domain"),
    ),
    "lola_64": Source(
        key="lola_64",
        url=_PDS_LOLA + "ldem_64.img",
        filename="ldem_64.img",
        label_url=_PDS_LOLA + "ldem_64.lbl",
        label_filename="ldem_64.lbl",
        describes="Moon global topography at 64 pixels/degree -> the standard tier",
        approximate_bytes=530_841_600,
        metres_per_pixel=473.80,
        attribution=("LRO LOLA Global Digital Elevation Model, 64 pixels/degree "
                     "(LDEM_64); Smith et al., NASA PDS Geosciences Node, "
                     "https://pds-geosciences.wustl.edu/ -- public domain"),
    ),
}


def cache_directory(root: Path) -> Path:
    """Where fetched rasters live.

    Under the build tree rather than the source tree: these are hundreds of megabytes of
    inputs that reproduce from the network on demand, and a source tree is for things that
    do not.
    """
    return root / "build" / "planet-cache"


def fetch(url: str, filename: str, into: Path, refresh: bool = False) -> Path:
    """Downloads @p url into @p into, reusing the cached copy unless @p refresh.

    Downloads to a temporary name and renames on success, so an interrupted fetch leaves no
    half a file behind that the next run would happily read as a whole one.

    @param url      What to fetch.
    @param filename The name to cache it under.
    @param into     The cache directory; created if absent.
    @param refresh  Re-download even when a cached copy exists.
    @return The path to the local file.
    @raises RuntimeError on any non-200 response.
    """
    import requests  # deferred: only the bake needs it, and it is an optional extra

    into.mkdir(parents=True, exist_ok=True)
    destination = into / filename
    if destination.exists() and not refresh:
        return destination

    partial = destination.with_suffix(destination.suffix + ".partial")
    with requests.get(url, timeout=1800, stream=True) as response:
        if response.status_code != 200:
            raise RuntimeError(f"{filename}: HTTP {response.status_code} from {url}")
        with partial.open("wb") as handle:
            for chunk in response.iter_content(chunk_size=1 << 20):
                handle.write(chunk)
    partial.replace(destination)
    return destination


def digest(path: Path) -> str:
    """A short SHA-256 of @p path, so the audit names the exact bytes that were read."""
    hasher = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            hasher.update(chunk)
    return hasher.hexdigest()[:16]
