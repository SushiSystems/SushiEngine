"""Equirectangular to cube-sphere: the reprojection, and nothing else.

A transcription of `include/SushiEngine/terrain/cube_sphere.hpp` and `tile_address.hpp`.
The engine headers are the authority; this exists because the bake has to place a sample at
exactly the position the renderer will later evaluate for it, and the two arriving at
different answers would show up as terrain that is subtly, unfixably wrong rather than as a
crash. Every constant below is named after the header constant it mirrors, and
`test_cube_sphere.cpp` pins the C++ side of the same arithmetic.

Bodies are treated as spheres here, which is exact for the Moon and is why the Moon is the
first body. A flattened body needs the planetocentric/geodetic latitude distinction handled
at this seam -- the elevation model states one and the ellipsoid wants the other -- and that
lands with the body that first needs it rather than as untested generality now.
"""

from __future__ import annotations

import math

from typing import Iterator, Tuple

import numpy as np

# Mirrors of the header constants. A mismatch is caught rather than tolerated: the pack
# header carries the grid size and apron it was baked for, and `PlanetPack::adopt` refuses a
# pack whose geometry is not the one the engine was compiled with.
TILE_GRID_SIZE = 129
TILE_APRON = 1
TILE_STRIDE = TILE_GRID_SIZE + 2 * TILE_APRON
TILE_SAMPLE_COUNT = TILE_STRIDE * TILE_STRIDE
MAX_TILE_DEPTH = 20

QUARTER_PI = math.pi / 4.0

FACE_NAMES = ("+X", "-X", "+Y", "-Y", "+Z", "-Z")
CUBE_FACE_COUNT = 6


def grid_to_face(grid: np.ndarray) -> np.ndarray:
    """The tangent warp: uniform grid parameter to face coordinate."""
    return np.tan(grid * QUARTER_PI)


def face_direction(face: int, s: np.ndarray, t: np.ndarray) -> np.ndarray:
    """The cube point a face coordinate names, in the cubemap face basis.

    @param face Face index, 0..5, ordered as `Terrain::CubeFace`.
    @param s    Face coordinate along the face's first axis.
    @param t    Face coordinate along the face's second axis.
    @return An array of shape `s.shape + (3,)` holding un-normalized cube points.
    """
    one = np.ones_like(s)
    if face == 0:
        components = (one, -t, -s)
    elif face == 1:
        components = (-one, -t, s)
    elif face == 2:
        components = (s, one, t)
    elif face == 3:
        components = (s, -one, -t)
    elif face == 4:
        components = (s, -t, one)
    elif face == 5:
        components = (-s, -t, -one)
    else:
        raise ValueError(f"face {face} is not one of the six")
    return np.stack(components, axis=-1)


def tile_grid_rect(depth: int, x: int, y: int) -> Tuple[float, float]:
    """The tile's grid-space origin; its span is `2 / 2**depth` on both axes."""
    step = 2.0 / float(1 << depth)
    return -1.0 + x * step, -1.0 + y * step


def tile_sample_coordinates(depth: int, x: int, y: int) -> Tuple[np.ndarray, np.ndarray]:
    """The grid coordinates of every stored sample of a tile, apron included.

    @return `(grid_s, grid_t)`, each of shape `(TILE_STRIDE, TILE_STRIDE)` and indexed
            `[row, column]` so the flat order matches `tile_sample_index`.
    """
    s_minimum, t_minimum = tile_grid_rect(depth, x, y)
    step = 2.0 / float(1 << depth)
    cells = float(TILE_GRID_SIZE - 1)
    axis = (np.arange(TILE_STRIDE, dtype=np.float64) - TILE_APRON) / cells
    grid_s = s_minimum + axis * step
    grid_t = t_minimum + axis * step
    return np.meshgrid(grid_s, grid_t, indexing="xy")


def tile_geographic(face: int, depth: int, x: int, y: int) -> Tuple[np.ndarray, np.ndarray]:
    """Where a tile's samples land on the body, as latitude and longitude in degrees.

    @return `(latitude, longitude)`, each `(TILE_STRIDE, TILE_STRIDE)`; latitude is
            planetocentric and longitude is degrees east.
    """
    grid_s, grid_t = tile_sample_coordinates(depth, x, y)
    direction = face_direction(face, grid_to_face(grid_s), grid_to_face(grid_t))
    direction /= np.linalg.norm(direction, axis=-1, keepdims=True)
    latitude = np.degrees(np.arcsin(np.clip(direction[..., 2], -1.0, 1.0)))
    longitude = np.degrees(np.arctan2(direction[..., 1], direction[..., 0]))
    return latitude, longitude


def tile_addresses(max_depth: int) -> Iterator[Tuple[int, int, int, int]]:
    """Every tile of a full pyramid, coarse first.

    Coarse first so a pack is useful while it is still being written and so the index it
    produces is close to sorted before the writer sorts it.

    @param max_depth Deepest level to emit, at most `MAX_TILE_DEPTH`.
    @return `(face, depth, x, y)` tuples.
    """
    if not 0 <= max_depth <= MAX_TILE_DEPTH:
        raise ValueError(f"depth {max_depth} is outside 0..{MAX_TILE_DEPTH}")
    for depth in range(max_depth + 1):
        side = 1 << depth
        for face in range(CUBE_FACE_COUNT):
            for y in range(side):
                for x in range(side):
                    yield face, depth, x, y


def pyramid_tile_count(max_depth: int) -> int:
    """How many tiles a full pyramid to @p max_depth holds."""
    return CUBE_FACE_COUNT * ((1 << (2 * (max_depth + 1))) - 1) // 3


def depth_for_resolution(face_arc_metres: float, metres_per_pixel: float) -> int:
    """The deepest quadtree depth a source's resolution actually supports.

    A tile texel at depth d spans `face_arc / (128 * 2**d)`; the deepest honest depth is the
    largest d whose texel is no finer than the source. This is what the asset reports as its
    data depth, and the reason the bake never claims the depth it was asked for.

    @param face_arc_metres  Arc length of one cube-face edge on the body, metres.
    @param metres_per_pixel The source raster's ground sample distance.
    @return The depth, clamped to `MAX_TILE_DEPTH`.
    """
    depth = 0
    while depth < MAX_TILE_DEPTH:
        texel = face_arc_metres / ((TILE_GRID_SIZE - 1) * float(1 << (depth + 1)))
        if texel < metres_per_pixel:
            break
        depth += 1
    return depth
