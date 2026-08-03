"""The `SET0` blob: the one place this side of the seam knows the byte layout.

`Climatology::adopt` in `engine/domain/atmosphere/source/climatology.cpp` is the reader, and it is the
authority. This module is deliberately a transcription of it -- same magic, same version,
same field order -- rather than a shared schema, because a shared schema between a Python
tool and an engine static library would be a third thing to keep in step with both.

Drift is caught rather than tolerated: `adopt` refuses a blob whose magic, version or
dimensions it does not recognise, so a mismatch shows up as a loud refusal at load time,
never as weather quietly grown on a misread mean state. `verify` below re-reads what was
just written for the same reason.
"""

from __future__ import annotations

import struct

from dataclasses import dataclass
from typing import Tuple

import numpy as np

MAGIC = b"SET0"
VERSION = 1

# `adopt` refuses a longer one; the bake must not write what the engine will not read.
MAX_PROVENANCE_BYTES = 4096

_HEADER = struct.Struct("<4sIiiiiI")


@dataclass(frozen=True)
class ClimatologyAsset:
    """Everything the asset carries, on the asset's own conventions.

    Latitude runs south to north and longitude eastward from the prime meridian in every
    array here; the source modules have already done that flip.
    """

    upper_wind_mps: np.ndarray          # (months, bands)
    lower_wind_mps: np.ndarray          # (months, bands)
    saturation_kg_per_m2: np.ndarray    # (months, bands)
    land_fraction: np.ndarray           # (surface_lat, surface_lon)
    sea_surface_kelvin: np.ndarray      # (months, surface_lat, surface_lon)
    provenance: str

    @property
    def months(self) -> int:
        return int(self.upper_wind_mps.shape[0])

    @property
    def bands(self) -> int:
        return int(self.upper_wind_mps.shape[1])

    @property
    def surface_shape(self) -> Tuple[int, int]:
        """`(longitude_cells, latitude_cells)`, in the header's own order."""
        return int(self.land_fraction.shape[1]), int(self.land_fraction.shape[0])


def _check(asset: ClimatologyAsset) -> None:
    """Refuses anything `adopt` would refuse, here, where the message can be useful."""
    if asset.bands < 2 or asset.months < 1:
        raise ValueError(f"a climatology needs >= 2 bands and >= 1 month, got "
                         f"{asset.bands} and {asset.months}")
    for name, profile in (("lower_wind", asset.lower_wind_mps),
                          ("saturation", asset.saturation_kg_per_m2)):
        if profile.shape != asset.upper_wind_mps.shape:
            raise ValueError(f"{name} is {profile.shape}, upper_wind is "
                             f"{asset.upper_wind_mps.shape}; profiles share one grid")
    longitudes, latitudes = asset.surface_shape
    if asset.sea_surface_kelvin.shape != (asset.months, latitudes, longitudes):
        raise ValueError(
            f"sea surface temperature is {asset.sea_surface_kelvin.shape}, expected "
            f"{(asset.months, latitudes, longitudes)} from the land mask and month count")
    for name, field in (("upper_wind", asset.upper_wind_mps),
                        ("lower_wind", asset.lower_wind_mps),
                        ("saturation", asset.saturation_kg_per_m2),
                        ("land_fraction", asset.land_fraction),
                        ("sea_surface", asset.sea_surface_kelvin)):
        if not np.isfinite(field).all():
            raise ValueError(f"{name} contains a non-finite value; a NaN baked into a "
                             f"mean state propagates into every cell the core inverts")
    if len(asset.provenance.encode("utf-8")) > MAX_PROVENANCE_BYTES:
        raise ValueError(f"provenance exceeds {MAX_PROVENANCE_BYTES} bytes, which adopt() "
                         f"refuses")


def _floats(values: np.ndarray) -> bytes:
    return np.ascontiguousarray(values, dtype="<f4").tobytes()


def pack(asset: ClimatologyAsset) -> bytes:
    """Serialises @p asset into the bytes `Climatology::adopt` reads."""
    _check(asset)
    longitudes, latitudes = asset.surface_shape
    provenance = asset.provenance.encode("utf-8")
    return b"".join((
        _HEADER.pack(MAGIC, VERSION, asset.bands, asset.months,
                     longitudes, latitudes, len(provenance)),
        _floats(asset.upper_wind_mps),
        _floats(asset.lower_wind_mps),
        _floats(asset.saturation_kg_per_m2),
        _floats(asset.land_fraction),
        _floats(asset.sea_surface_kelvin),
        provenance,
    ))


def unpack(blob: bytes) -> ClimatologyAsset:
    """Reads a blob back, applying every check `adopt` applies.

    Exists so the bake can verify its own output rather than assert it: a writer that has
    never been read is a writer whose layout has never been tested.
    """
    if len(blob) < _HEADER.size or blob[:4] != MAGIC:
        raise ValueError("not a climatology asset: bad magic")
    magic, version, bands, months, longitudes, latitudes, provenance_length = \
        _HEADER.unpack_from(blob)
    if version != VERSION:
        raise ValueError(f"climatology asset version {version}, this tool writes {VERSION}")
    if bands < 2 or months < 1 or longitudes < 0 or latitudes < 0:
        raise ValueError(f"refused dimensions: {bands} bands, {months} months, "
                         f"{longitudes}x{latitudes} surface")
    if provenance_length > MAX_PROVENANCE_BYTES:
        raise ValueError(f"provenance length {provenance_length} exceeds the limit")

    cursor = _HEADER.size
    profile_values = bands * months
    surface_values = longitudes * latitudes

    def take(count: int, shape: Tuple[int, ...]) -> np.ndarray:
        nonlocal cursor
        end = cursor + count * 4
        if end > len(blob):
            raise ValueError("climatology asset is shorter than its header describes")
        block = np.frombuffer(blob, dtype="<f4", count=count,
                              offset=cursor).reshape(shape)
        cursor = end
        return block

    upper = take(profile_values, (months, bands))
    lower = take(profile_values, (months, bands))
    saturation = take(profile_values, (months, bands))
    land = take(surface_values, (latitudes, longitudes))
    sea = take(surface_values * months, (months, latitudes, longitudes))
    if cursor + provenance_length != len(blob):
        raise ValueError("climatology asset has trailing bytes after its provenance")

    return ClimatologyAsset(
        upper_wind_mps=upper, lower_wind_mps=lower, saturation_kg_per_m2=saturation,
        land_fraction=land, sea_surface_kelvin=sea,
        provenance=blob[cursor:cursor + provenance_length].decode("utf-8"))


def verify(blob: bytes, original: ClimatologyAsset) -> None:
    """Round-trips @p blob and confirms it says what @p original said.

    Exact equality, not a tolerance: the payload is float32 both times, so anything other
    than bit-for-bit agreement is a layout error rather than a rounding one.
    """
    read = unpack(blob)
    for name, before, after in (
            ("upper_wind", original.upper_wind_mps, read.upper_wind_mps),
            ("lower_wind", original.lower_wind_mps, read.lower_wind_mps),
            ("saturation", original.saturation_kg_per_m2, read.saturation_kg_per_m2),
            ("land_fraction", original.land_fraction, read.land_fraction),
            ("sea_surface", original.sea_surface_kelvin, read.sea_surface_kelvin)):
        if not np.array_equal(np.asarray(before, dtype="<f4"), after):
            raise ValueError(f"{name} did not survive the round trip")
    if read.provenance != original.provenance:
        raise ValueError("provenance did not survive the round trip")
