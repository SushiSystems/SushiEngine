"""Coastline vectors -> land area fraction on the asset's surface grid.

The field is a *fraction*, not a flag, and that is the whole reason this rasterises from
vectors rather than downsampling somebody's binary mask: a one-degree cell holding a third
of a peninsula should read 0.33, and the only way to know that is to measure the polygon.
"""

from __future__ import annotations

import json

from pathlib import Path
from typing import Iterator, List, Tuple

import numpy as np

# Samples per cell edge. 8 gives 64 samples per cell, so the fraction is resolved to about
# 1.5 %, well below any error the coastline dataset itself carries at 1:50m.
SAMPLES_PER_CELL = 8

# What fraction of the Earth is land, and how far the rasterised total may sit from it.
#
# This is a correctness check, not a cosmetic one. The scanline below relies on the land
# polygons being mutually disjoint so that a single even-odd parity over every ring at once
# is the union of them; if that assumption failed -- overlapping polygons, an unclosed ring
# -- overlapping regions would cancel to water and the total would fall visibly short.
EXPECTED_LAND_FRACTION = 0.292
LAND_FRACTION_TOLERANCE = 0.02


def _rings(geometry: dict) -> Iterator[List[Tuple[float, float]]]:
    """Yields every linear ring of a Polygon or MultiPolygon geometry."""
    kind = geometry.get("type")
    if kind == "Polygon":
        yield from geometry["coordinates"]
    elif kind == "MultiPolygon":
        for polygon in geometry["coordinates"]:
            yield from polygon


def _edges(path: Path) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, int]:
    """Flattens every ring in the GeoJSON into one array of edges.

    @return `(x0, y0, x1, y1, ring_count)` in degrees, longitudes in [-180, 180].
    """
    with path.open("r", encoding="utf-8") as handle:
        collection = json.load(handle)

    x0: List[float] = []
    y0: List[float] = []
    x1: List[float] = []
    y1: List[float] = []
    rings = 0
    for feature in collection.get("features", []):
        for ring in _rings(feature.get("geometry") or {}):
            points = np.asarray(ring, dtype=np.float64)
            if points.shape[0] < 3:
                continue
            # Close the ring if the source left it open; an open ring leaks parity across
            # the whole scanline to its east.
            if not np.array_equal(points[0], points[-1]):
                points = np.vstack([points, points[:1]])
            x0.extend(points[:-1, 0]); y0.extend(points[:-1, 1])
            x1.extend(points[1:, 0]);  y1.extend(points[1:, 1])
            rings += 1

    return (np.asarray(x0), np.asarray(y0), np.asarray(x1), np.asarray(y1), rings)


def rasterise(path: Path, longitude_cells: int, latitude_cells: int,
              samples_per_cell: int = SAMPLES_PER_CELL) -> Tuple[np.ndarray, int]:
    """Rasterises land polygons to an area fraction in [0, 1].

    Scanline even-odd fill: for each sample latitude, every edge that straddles it
    contributes one crossing, and a sample point is inside iff an odd number of crossings
    lie to its west. Because the land polygons are disjoint, one parity taken over all
    rings at once is exactly their union, and holes (which lie inside their own exterior)
    subtract themselves.

    @param path             The GeoJSON land vectors.
    @param longitude_cells  Cells east-west; the asset indexes these from the prime
                            meridian eastward.
    @param latitude_cells   Cells south-north.
    @param samples_per_cell Supersampling factor per cell edge.
    @return `(fraction[lat][lon], ring_count)`.
    """
    x0, y0, x1, y1, rings = _edges(path)

    sample_lon_count = longitude_cells * samples_per_cell
    sample_lat_count = latitude_cells * samples_per_cell
    # Sample at cell-sub-centres so no sample lands exactly on a cell edge, where a
    # coastline that follows a meridian would be counted by rounding rather than geometry.
    sample_lon = (np.arange(sample_lon_count) + 0.5) / sample_lon_count * 360.0 - 180.0
    sample_lat = (np.arange(sample_lat_count) + 0.5) / sample_lat_count * 180.0 - 90.0

    inside = np.zeros((sample_lat_count, sample_lon_count), dtype=bool)
    horizontal = y0 == y1  # contributes no crossing, and would divide by zero below
    span = np.where(horizontal, 1.0, y1 - y0)
    for row, latitude in enumerate(sample_lat):
        straddles = ((y0 > latitude) != (y1 > latitude)) & ~horizontal
        if not straddles.any():
            continue
        crossings = x0[straddles] + (latitude - y0[straddles]) * (
            (x1[straddles] - x0[straddles]) / span[straddles])
        crossings.sort()
        inside[row] = np.searchsorted(crossings, sample_lon, side="right") % 2 == 1

    # Average each cell's samples, then roll the prime meridian to index 0: the samples run
    # from -180 but the asset's x index counts eastward from 0.
    fraction = inside.reshape(latitude_cells, samples_per_cell,
                              longitude_cells, samples_per_cell).mean(axis=(1, 3))
    return np.roll(fraction, -(longitude_cells // 2), axis=1), rings


def area_weighted_total(fraction: np.ndarray) -> float:
    """The global land fraction implied by @p fraction, weighted by cell area.

    Cells shrink toward the poles, so a plain mean would count Antarctica as though it
    were tropical and report a land fraction well over the truth.
    """
    latitude_cells = fraction.shape[0]
    centres = np.deg2rad((np.arange(latitude_cells) + 0.5) / latitude_cells * 180.0 - 90.0)
    weights = np.cos(centres)
    return float((fraction.mean(axis=1) * weights).sum() / weights.sum())
