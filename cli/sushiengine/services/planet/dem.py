"""Reading a PDS3 elevation raster, and sampling it.

The planetary science community publishes global topography as a raw binary raster beside
a plain-text PDS3 label describing it. That is a gift: no GDAL, no rasterio, no projection
library -- the label states the grid and the scaling, and simple cylindrical is a formula.
`numpy` and the standard library are the whole dependency.

**The one trap, stated because it is silent when got wrong.** A LOLA label reads

    SCALING_FACTOR = 0.5
    OFFSET         = 1737400.
    /* HEIGHT = (DN * SCALING_FACTOR).                    */
    /* PLANETARY_RADIUS = (DN * SCALING_FACTOR) + OFFSET  */

`OFFSET` is the radius of the reference sphere, not a term in the height. Adding it gives a
planetary radius, and a terrain baked from radii instead of elevations is 1737 km of solid
rock with a plausible-looking surface on top. Elevation is `DN * SCALING_FACTOR`, full stop.

The grid convention is verified rather than assumed -- see `check_landmarks`, which is run
by the bake against known lunar extremes. Getting longitude backwards produces a mirrored
body that looks entirely reasonable until someone recognises a crater.
"""

from __future__ import annotations

import re

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Tuple

import numpy as np

# The label is ASCII and small; reading more than this means it is not a label.
_MAX_LABEL_BYTES = 1 << 20

_ASSIGNMENT = re.compile(r"^\s*([A-Z_0-9:^]+)\s*=\s*(.+?)\s*$")


def read_label(path: Path) -> Dict[str, str]:
    """Parses a PDS3 label into its first-occurrence key/value pairs.

    First occurrence rather than last: the IMAGE object precedes the projection block in
    every product this reads, and the keys that collide (`OFFSET`, `LINES`) mean the image's
    thing there. Unit suffixes like `<pix/deg>` are stripped.

    @param path Path to the `.lbl` file.
    @return The parsed assignments, keys upper-case.
    @raises ValueError when the file is too large to be a label.
    """
    size = path.stat().st_size
    if size > _MAX_LABEL_BYTES:
        raise ValueError(f"{path.name} is {size} bytes; that is not a PDS label")

    values: Dict[str, str] = {}
    for line in path.read_text(encoding="latin-1").splitlines():
        if line.lstrip().startswith("/*"):
            continue
        match = _ASSIGNMENT.match(line)
        if not match:
            continue
        key, value = match.group(1), match.group(2)
        value = re.sub(r"<[^>]*>", "", value).strip().strip('"')
        if key not in values:
            values[key] = value
    return values


def _number(label: Dict[str, str], key: str) -> float:
    if key not in label:
        raise ValueError(f"the label has no {key}")
    try:
        return float(label[key])
    except ValueError as error:
        raise ValueError(f"{key} is {label[key]!r}, which is not a number") from error


@dataclass
class EquirectangularGrid:
    """A global simple-cylindrical elevation raster, in metres above a reference sphere.

    Row 0 is the northern edge and column 0 is longitude zero, both verified in
    `check_landmarks` rather than trusted. Values stay 16-bit on disk and are memory-mapped:
    a 64 pixel/degree lunar grid is 530 MB as stored and 2.1 GB as float64, and the bake
    reads a few tens of thousands of scattered samples at a time rather than all of it.
    """

    values: np.ndarray            # (lines, samples), integer, memory-mapped
    scaling_factor: float         # metres per stored unit
    pixels_per_degree: float
    reference_radius_metres: float
    source_key: str

    @property
    def metres_per_pixel(self) -> float:
        """Ground sample distance at the equator."""
        circumference = 2.0 * np.pi * self.reference_radius_metres
        return circumference / (360.0 * self.pixels_per_degree)

    def sample(self, latitude_degrees: np.ndarray,
               longitude_degrees: np.ndarray) -> np.ndarray:
        """Bilinearly samples elevation at geographic coordinates.

        Longitude wraps and latitude clamps, which is what the grid actually is: a cylinder
        closed in one direction and bounded in the other. Sampling rather than nearest
        neighbour because the alternative aliases ridges into noise, and a ridge is the
        feature terrain is most judged on.

        @param latitude_degrees  Planetocentric latitude, degrees north.
        @param longitude_degrees Longitude, degrees east; any range, wrapped here.
        @return Elevation in metres above the reference sphere, same shape as the inputs.
        """
        lines, samples = self.values.shape
        x = np.mod(longitude_degrees, 360.0) * self.pixels_per_degree - 0.5
        y = (90.0 - latitude_degrees) * self.pixels_per_degree - 0.5
        y = np.clip(y, 0.0, lines - 1.0)

        x_floor = np.floor(x)
        y_floor = np.floor(y)
        fx = x - x_floor
        fy = y - y_floor

        x0 = np.mod(x_floor.astype(np.int64), samples)
        x1 = np.mod(x0 + 1, samples)
        y0 = y_floor.astype(np.int64)
        y1 = np.minimum(y0 + 1, lines - 1)

        top = self.values[y0, x0] * (1.0 - fx) + self.values[y0, x1] * fx
        bottom = self.values[y1, x0] * (1.0 - fx) + self.values[y1, x1] * fx
        return (top * (1.0 - fy) + bottom * fy) * self.scaling_factor


def open_grid(image: Path, label: Path, source_key: str) -> EquirectangularGrid:
    """Opens a PDS3 raster as a sampleable grid, checking the label against the file.

    Every dimension the label claims is checked against the file's actual size and against
    the map resolution, so a truncated download or a product that is not global fails here
    with a sentence rather than three stages later as impossible terrain.

    @param image      Path to the `.img` raster.
    @param label      Path to its `.lbl` label.
    @param source_key The key in `sources.SOURCES` this came from, for error messages.
    @return The opened grid.
    @raises ValueError when the label and the file disagree, or the product is not a global
            simple-cylindrical 16-bit raster.
    """
    fields = read_label(label)

    sample_type = fields.get("SAMPLE_TYPE", "")
    sample_bits = int(_number(fields, "SAMPLE_BITS"))
    if sample_type != "LSB_INTEGER" or sample_bits != 16:
        raise ValueError(f"{source_key}: this reader handles LSB_INTEGER/16, the label says "
                         f"{sample_type}/{sample_bits}")

    lines = int(_number(fields, "LINES"))
    samples = int(_number(fields, "LINE_SAMPLES"))
    resolution = _number(fields, "MAP_RESOLUTION")
    if samples != int(round(360.0 * resolution)) or lines != int(round(180.0 * resolution)):
        raise ValueError(f"{source_key}: {samples}x{lines} is not a global raster at "
                         f"{resolution} pixels/degree; this reader only handles global ones")

    expected = lines * samples * 2
    actual = image.stat().st_size
    if actual != expected:
        raise ValueError(f"{source_key}: {image.name} is {actual} bytes, the label describes "
                         f"{expected}. A truncated download reads as terrain, so it is "
                         f"refused here rather than baked.")

    # A_AXIS_RADIUS is in kilometres in every product this reads; OFFSET repeats it in
    # metres. Preferring OFFSET keeps the reference surface exact rather than rounded.
    radius = _number(fields, "OFFSET")
    if not 1.0e5 < radius < 1.0e8:
        raise ValueError(f"{source_key}: reference radius {radius} m is not a planetary one")

    values = np.memmap(image, dtype="<i2", mode="r", shape=(lines, samples))
    return EquirectangularGrid(values=values,
                               scaling_factor=_number(fields, "SCALING_FACTOR"),
                               pixels_per_degree=resolution,
                               reference_radius_metres=radius,
                               source_key=source_key)


# Two points whose position and height are independently documented, used to prove the
# grid convention rather than assume it. If longitude were mirrored or latitude flipped,
# these land somewhere unremarkable and the check fails -- which is the whole point, since
# a mirrored body is otherwise entirely plausible to look at.
LUNAR_LANDMARKS = (
    ("global minimum (South Pole-Aitken, near Antoniadi)", -70.34, 187.53, -8981.0, 400.0),
    ("global maximum (South Pole-Aitken far rim)", 5.41, 201.34, 10686.0, 400.0),
    ("Mare Serenitatis", 26.0, 18.0, -2627.0, 900.0),
    ("far-side highlands", -10.0, 175.0, 1517.0, 900.0),
)


def check_landmarks(grid: EquirectangularGrid) -> Tuple[bool, list]:
    """Samples known landmarks and reports whether the grid reads as the body it claims.

    @param grid The opened grid.
    @return `(ok, rows)`, where each row is `(name, expected, measured, tolerance)`.
    """
    rows = []
    ok = True
    for name, latitude, longitude, expected, tolerance in LUNAR_LANDMARKS:
        measured = float(grid.sample(np.array([latitude]), np.array([longitude]))[0])
        rows.append((name, expected, measured, tolerance))
        if abs(measured - expected) > tolerance:
            ok = False
    return ok, rows
