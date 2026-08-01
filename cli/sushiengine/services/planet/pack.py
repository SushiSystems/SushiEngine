"""The `.planet` blob: the one place this side of the seam knows the byte layout.

`PlanetPack::adopt` in `include/SushiEngine/terrain/pack_format.hpp` is the reader, and it
is the authority. This module is deliberately a transcription of it -- same magic, same
version, same field order -- rather than a shared schema, because a shared schema between a
Python tool and an engine header would be a third thing to keep in step with both.

Drift is caught rather than tolerated: `adopt` refuses a blob whose magic, version, tile
geometry or index ordering it does not recognise, so a mismatch shows up as a loud refusal
at load time and never as terrain quietly grown on a misread height. `verify` below re-reads
what was just written for the same reason.

The file is written in one pass with a seek: every payload is the same size, so the index's
size is known before any tile is baked, and the writer reserves it, streams the payloads
past it, then seeks back to fill it in. That keeps memory bounded by one tile rather than by
the asset, which matters at the sizes the deeper tiers reach.
"""

from __future__ import annotations

import struct

from dataclasses import dataclass
from pathlib import Path
from typing import BinaryIO, Dict, List, Optional, Tuple

import numpy as np

from . import cube

MAGIC = b"SUSHIPLA"
VERSION = 1
HEADER_BYTES = 64
RECORD_BYTES = 40
CODEC_QUANTISED = 0
MAX_PROVENANCE_BYTES = 65536

_HEADER = struct.Struct("<8sIIdddIIB3xII4x")
_RECORD = struct.Struct("<QQIHHffff")

_PAYLOAD_BYTES = cube.TILE_SAMPLE_COUNT * 2


def tile_address_key(face: int, depth: int, x: int, y: int) -> int:
    """The packed key `Terrain::tile_address_key` produces for the same address."""
    return (face << 45) | (depth << 40) | (x << 20) | y


@dataclass(frozen=True)
class Record:
    """One index entry, in the reader's own field order."""

    key: int
    offset: int
    length: int
    codec: int
    quantised_minimum: float
    quantised_maximum: float
    grid_minimum: float
    grid_maximum: float


def quantise(heights: np.ndarray) -> Tuple[bytes, float, float, float, float]:
    """Turns a tile's elevations into the stored 16-bit payload and its ranges.

    The quantisation bounds are floored and ceiled to whole metres before being narrowed to
    float32. Two things follow, both load-bearing: the bounds are exactly representable, so
    the writer and the reader agree on them bit for bit rather than nearly; and they bracket
    every sample, so no value clips. The cost is at most a metre or two of extra range on a
    65535-step scale, which is below the quantisation step it widens.

    @param heights A `(TILE_STRIDE, TILE_STRIDE)` array of elevations in metres.
    @return `(payload, quantised_minimum, quantised_maximum, grid_minimum, grid_maximum)`.
    """
    if heights.shape != (cube.TILE_STRIDE, cube.TILE_STRIDE):
        raise ValueError(f"a tile is {cube.TILE_STRIDE}^2 samples, got {heights.shape}")
    if not np.isfinite(heights).all():
        raise ValueError("a tile contains a non-finite elevation; a NaN baked into terrain "
                         "propagates into every normal derived from it")

    quantised_minimum = float(np.float32(np.floor(heights.min())))
    quantised_maximum = float(np.float32(np.ceil(heights.max())))

    interior = heights[cube.TILE_APRON:cube.TILE_APRON + cube.TILE_GRID_SIZE,
                       cube.TILE_APRON:cube.TILE_APRON + cube.TILE_GRID_SIZE]
    grid_minimum = float(np.float32(interior.min()))
    grid_maximum = float(np.float32(interior.max()))

    span = quantised_maximum - quantised_minimum
    if span > 0.0:
        scaled = np.rint((heights - quantised_minimum) * (65535.0 / span))
        values = np.clip(scaled, 0.0, 65535.0).astype("<u2")
    else:
        values = np.zeros(heights.shape, dtype="<u2")
    return (values.tobytes(), quantised_minimum, quantised_maximum,
            grid_minimum, grid_maximum)


def quantisation_step(quantised_minimum: float, quantised_maximum: float) -> float:
    """The elevation one stored unit covers -- the bake's accuracy bound, in metres."""
    return (quantised_maximum - quantised_minimum) / 65535.0


class PackWriter:
    """Writes a `.planet` asset, reserving its index and filling it in on close."""

    def __init__(self, path: Path, *, body_id: int, semi_axes: Tuple[float, float, float],
                 height_data_depth: int, tile_count: int, provenance: str):
        """Opens the asset and reserves room for its header, provenance and index.

        @param path              Where to write.
        @param body_id           Ephemeris body ordinal the asset is for.
        @param semi_axes         The reference ellipsoid's three semi-axes, metres.
        @param height_data_depth The deepest depth with real measurement behind it.
        @param tile_count        How many tiles will be added; must match exactly.
        @param provenance        The attribution string carried inside the asset.
        """
        encoded = provenance.encode("utf-8")
        if len(encoded) > MAX_PROVENANCE_BYTES:
            raise ValueError(f"provenance is {len(encoded)} bytes, the reader refuses past "
                             f"{MAX_PROVENANCE_BYTES}")
        if not 0 <= height_data_depth <= cube.MAX_TILE_DEPTH:
            raise ValueError(f"data depth {height_data_depth} is outside the addressable range")

        self._path = path
        self._provenance = encoded
        self._tile_count = tile_count
        self._records: List[Record] = []
        self._index_offset = HEADER_BYTES + len(encoded)
        self._payload_offset = self._index_offset + tile_count * RECORD_BYTES

        path.parent.mkdir(parents=True, exist_ok=True)
        self._handle: Optional[BinaryIO] = path.open("wb")
        self._handle.write(_HEADER.pack(MAGIC, VERSION, body_id,
                                        semi_axes[0], semi_axes[1], semi_axes[2],
                                        cube.TILE_GRID_SIZE, cube.TILE_APRON,
                                        height_data_depth, tile_count, len(encoded)))
        self._handle.write(encoded)
        self._handle.seek(self._payload_offset)

    def add(self, face: int, depth: int, x: int, y: int, heights: np.ndarray) -> Record:
        """Appends one tile's payload and records where it landed.

        @param face   Cube face index.
        @param depth  Quadtree depth.
        @param x      Cell column.
        @param y      Cell row.
        @param heights The tile's elevations, metres.
        @return The index record written for it.
        """
        if self._handle is None:
            raise RuntimeError("this writer is closed")
        payload, quantised_minimum, quantised_maximum, grid_minimum, grid_maximum = \
            quantise(heights)
        offset = self._payload_offset + len(self._records) * _PAYLOAD_BYTES
        self._handle.write(payload)
        record = Record(key=tile_address_key(face, depth, x, y), offset=offset,
                        length=_PAYLOAD_BYTES, codec=CODEC_QUANTISED,
                        quantised_minimum=quantised_minimum,
                        quantised_maximum=quantised_maximum,
                        grid_minimum=grid_minimum, grid_maximum=grid_maximum)
        self._records.append(record)
        return record

    def close(self) -> None:
        """Sorts the index, writes it into the reserved space, and closes the file.

        The sort is what licenses the reader's binary search, and the reader refuses a pack
        whose keys do not ascend strictly -- so a duplicate address is caught here, where the
        message can name it, rather than at load time.
        """
        if self._handle is None:
            return
        if len(self._records) != self._tile_count:
            self._handle.close()
            self._handle = None
            raise ValueError(f"reserved room for {self._tile_count} tiles, wrote "
                             f"{len(self._records)}")

        ordered = sorted(self._records, key=lambda record: record.key)
        for first, second in zip(ordered, ordered[1:]):
            if first.key == second.key:
                self._handle.close()
                self._handle = None
                raise ValueError(f"two tiles share address key {first.key}")

        self._handle.seek(self._index_offset)
        for record in ordered:
            self._handle.write(_RECORD.pack(record.key, record.offset, record.length,
                                            record.codec, 0,
                                            record.quantised_minimum,
                                            record.quantised_maximum,
                                            record.grid_minimum, record.grid_maximum))
        self._handle.close()
        self._handle = None


@dataclass(frozen=True)
class PackHeader:
    """What a pack says about itself."""

    body_id: int
    semi_axes: Tuple[float, float, float]
    height_data_depth: int
    tile_count: int
    provenance: str


def read_header(handle: BinaryIO) -> Tuple[PackHeader, Dict[int, Record]]:
    """Reads a pack's header and index, applying every check the engine's reader applies.

    Exists so the bake can verify its own output rather than assert it: a writer that has
    never been read is a writer whose layout has never been tested.

    @param handle An open binary file positioned anywhere; it is seeked.
    @return The header and its index, keyed by tile address key.
    @raises ValueError on anything `PlanetPack::adopt` would refuse.
    """
    handle.seek(0)
    raw = handle.read(HEADER_BYTES)
    if len(raw) < HEADER_BYTES or raw[:8] != MAGIC:
        raise ValueError("not a planet pack: bad magic")
    (_, version, body_id, axis_x, axis_y, axis_z, grid_size, apron,
     data_depth, tile_count, provenance_length) = _HEADER.unpack(raw)
    if version != VERSION:
        raise ValueError(f"planet pack version {version}, this tool writes {VERSION}")
    if grid_size != cube.TILE_GRID_SIZE or apron != cube.TILE_APRON:
        raise ValueError(f"pack was baked for a {grid_size}^2 grid with a {apron} apron; "
                         f"this build uses {cube.TILE_GRID_SIZE}/{cube.TILE_APRON}")
    if provenance_length > MAX_PROVENANCE_BYTES:
        raise ValueError(f"provenance length {provenance_length} exceeds the limit")
    if data_depth > cube.MAX_TILE_DEPTH:
        raise ValueError(f"data depth {data_depth} is outside the addressable range")

    provenance = handle.read(provenance_length).decode("utf-8")
    index_end = HEADER_BYTES + provenance_length + tile_count * RECORD_BYTES

    records: Dict[int, Record] = {}
    previous_key = None
    for _ in range(tile_count):
        raw_record = handle.read(RECORD_BYTES)
        if len(raw_record) < RECORD_BYTES:
            raise ValueError("planet pack is shorter than its index describes")
        (key, offset, length, codec, _, quantised_minimum, quantised_maximum,
         grid_minimum, grid_maximum) = _RECORD.unpack(raw_record)
        if previous_key is not None and key <= previous_key:
            raise ValueError("planet pack index does not ascend strictly")
        previous_key = key
        if codec != CODEC_QUANTISED:
            raise ValueError(f"unknown codec {codec}")
        if length != _PAYLOAD_BYTES:
            raise ValueError(f"tile payload is {length} bytes, expected {_PAYLOAD_BYTES}")
        if offset < index_end:
            raise ValueError("a tile payload overlaps the index")
        if quantised_maximum < quantised_minimum:
            raise ValueError("a tile's quantisation range is inverted")
        records[key] = Record(key, offset, length, codec, quantised_minimum,
                              quantised_maximum, grid_minimum, grid_maximum)

    return (PackHeader(body_id=body_id, semi_axes=(axis_x, axis_y, axis_z),
                       height_data_depth=data_depth, tile_count=tile_count,
                       provenance=provenance), records)


def read_tile(handle: BinaryIO, record: Record) -> np.ndarray:
    """Decodes one stored tile back to elevations in metres.

    The exact inverse of `quantise`, so the difference between this and the source raster is
    the quantisation step and nothing else -- which is what the bake's accuracy audit
    measures.

    @param handle An open binary file.
    @param record The tile's index record.
    @return A `(TILE_STRIDE, TILE_STRIDE)` array of elevations.
    """
    handle.seek(record.offset)
    raw = handle.read(record.length)
    if len(raw) < record.length:
        raise ValueError("planet pack is shorter than a tile payload it indexes")
    values = np.frombuffer(raw, dtype="<u2").astype(np.float64)
    span = record.quantised_maximum - record.quantised_minimum
    scaled = record.quantised_minimum + values * (span / 65535.0)
    return scaled.reshape(cube.TILE_STRIDE, cube.TILE_STRIDE)
