"""Reanalysis fields -> the three zonal profiles T1 relaxes toward.

The reanalysis is on a 2.5 degree grid running north to south; the asset is on a
one-degree grid running south to north. Both facts are handled here and nowhere else, so
the rest of the bake never has to remember which way up a source was.
"""

from __future__ import annotations

import warnings

from dataclasses import dataclass
from pathlib import Path
from typing import Tuple

import numpy as np

# Pressure levels the two wind profiles are read at, hPa.
#
# 250 is the jet core and 850 is the lowest level clear of most orography, so their
# difference is the depth-integrated thermal wind the two-layer core is a model of. The
# core has exactly two layers; picking one level for each is not an approximation of a
# vertical integral, it *is* the model's vertical structure.
UPPER_LEVEL_HPA = 250.0
LOWER_LEVEL_HPA = 850.0

# Pressure band the saturated column water is integrated over, hPa.
#
# Stops at 300 because saturation vapour pressure has fallen by three orders of magnitude
# by then -- the tropopause carries no water worth counting -- and starts at 1000 because
# that is the reanalysis's own lowest level.
#
# The known cost: where the ground is above 1000 hPa -- the Antarctic plateau, the Andes,
# Tibet -- the levels beneath it are extrapolated rather than measured, and being warmer
# than the real surface they inflate the saturation ceiling there. It is a ceiling over a
# place that is dry for other reasons, so nothing downstream reaches it; the honest fix is
# a surface-pressure field, which is a separate download for a field nothing yet reads.
COLUMN_TOP_HPA = 300.0
COLUMN_BASE_HPA = 1000.0

_GRAVITY = 9.80665
_DRY_TO_VAPOUR = 0.622  # ratio of molecular weights, water to dry air


@dataclass(frozen=True)
class ZonalProfiles:
    """The three profiles, each `(months, bands)` on the asset's own latitude grid."""

    upper_wind_mps: np.ndarray
    lower_wind_mps: np.ndarray
    saturation_kg_per_m2: np.ndarray
    observed_water_kg_per_m2: np.ndarray
    """Observed precipitable water, on the same grid. Never written to the asset -- it is
    carried so the audit can divide it by the saturation and show the implied column
    relative humidity, which is the one number that says whether the saturation
    derivation is physical."""


def band_centres(bands: int) -> np.ndarray:
    """Latitudes of the asset's bands, degrees north, south to north.

    Mirrors `Climatology::sample_profile`: band `i` sits at `(i + 0.5) / bands * 180 - 90`.
    Duplicating the formula is the point -- if the reader's convention ever changes, the
    bake must be changed deliberately rather than silently resampled onto the wrong grid.
    """
    return (np.arange(bands) + 0.5) / bands * 180.0 - 90.0


def _regrid(values: np.ndarray, source_lat: np.ndarray, bands: int) -> np.ndarray:
    """Resamples `(months, source_lat)` onto the asset's bands, south to north.

    Linear in latitude, and clamped at both ends rather than extrapolated: past the last
    reanalysis latitude there is no data, and a linear extrapolation off the end of a
    profile that is curving would invent a jet at the pole.
    """
    order = np.argsort(source_lat)
    ascending_lat = source_lat[order]
    targets = band_centres(bands)
    return np.stack([np.interp(targets, ascending_lat, month[order])
                     for month in values])


def saturation_specific_humidity(temperature_k: np.ndarray,
                                 pressure_pa: np.ndarray) -> np.ndarray:
    """Saturation specific humidity, kg/kg, by the Tetens formula over liquid water.

    Liquid rather than ice even below freezing: the field is the ceiling the core's
    condensation scheme measures column water against, and a mixed-phase ceiling would
    make that ceiling jump discontinuously across the freezing line for reasons the core
    has no way to know about.
    """
    saturation_pressure = 611.2 * np.exp(
        17.67 * (temperature_k - 273.15) / (temperature_k - 29.65))
    return (_DRY_TO_VAPOUR * saturation_pressure /
            (pressure_pa - 0.378 * saturation_pressure))


def _column_integral(specific_humidity: np.ndarray,
                     pressure_pa: np.ndarray) -> np.ndarray:
    """Integrates `q dp / g` over the level axis (axis 1), giving kg/m^2."""
    thickness = np.diff(pressure_pa)
    midpoints = 0.5 * (specific_humidity[:, :-1] + specific_humidity[:, 1:])
    shape = (1, thickness.size) + (1,) * (specific_humidity.ndim - 2)
    return -(midpoints * thickness.reshape(shape)).sum(axis=1) / _GRAVITY


def _open(path: Path):
    """Opens a NetCDF file, turning the missing optional dependency into advice."""
    try:
        import netCDF4
    except ImportError as error:  # pragma: no cover - environment-dependent
        # The bracket is escaped for Rich, which reads "[climatology]" as a style tag and
        # would drop the one word the reader needs.
        raise RuntimeError(
            "netCDF4 is required to bake the climatology and is not installed.\n"
            "It is an optional extra so that `se build` does not drag it in:\n"
            r"    pip install -e cli\[climatology]") from error
    return netCDF4.Dataset(path)


def read_zonal_profiles(wind_path: Path, temperature_path: Path, water_path: Path,
                        bands: int) -> Tuple[ZonalProfiles, int]:
    """Builds the three profiles from the three reanalysis files.

    @param wind_path        `uwnd` monthly long-term means on pressure levels.
    @param temperature_path `air` monthly long-term means on pressure levels.
    @param water_path       `pr_wtr` monthly long-term means, for the audit only.
    @param bands            Latitude bands the asset will carry.
    @return The profiles, and the number of months found in the sources.
    @raises RuntimeError if the sources disagree on shape or lack a needed level.
    """
    with _open(wind_path) as dataset:
        latitude = np.asarray(dataset.variables["lat"][:], dtype=np.float64)
        levels = np.asarray(dataset.variables["level"][:], dtype=np.float64)
        wind = np.asarray(dataset.variables["uwnd"][:], dtype=np.float64)

    for name, level in (("upper", UPPER_LEVEL_HPA), ("lower", LOWER_LEVEL_HPA)):
        if not np.any(levels == level):
            raise RuntimeError(
                f"{wind_path.name} has no {level:g} hPa level, so the {name} jet cannot "
                f"be read; levels present: {levels.tolist()}")

    months = wind.shape[0]
    upper = wind[:, int(np.where(levels == UPPER_LEVEL_HPA)[0][0])].mean(axis=2)
    lower = wind[:, int(np.where(levels == LOWER_LEVEL_HPA)[0][0])].mean(axis=2)

    with _open(temperature_path) as dataset:
        air_levels = np.asarray(dataset.variables["level"][:], dtype=np.float64)
        # NCEP stores air temperature in degrees Celsius; every formula below is in
        # kelvin, and the unit is read rather than assumed.
        units = str(getattr(dataset.variables["air"], "units", "")).lower()
        air = np.asarray(dataset.variables["air"][:], dtype=np.float64)
        air_latitude = np.asarray(dataset.variables["lat"][:], dtype=np.float64)
    if "c" in units and "k" not in units:
        air = air + 273.15
    if air.shape[0] != months or not np.array_equal(air_latitude, latitude):
        raise RuntimeError(
            f"{temperature_path.name} and {wind_path.name} are on different grids "
            f"({air.shape} vs {wind.shape}); they must come from one reanalysis.")

    band = (air_levels <= COLUMN_BASE_HPA) & (air_levels >= COLUMN_TOP_HPA)
    pressure = air_levels[band] * 100.0
    humidity = saturation_specific_humidity(
        air[:, band], pressure.reshape((1, pressure.size, 1, 1)))
    saturation = _column_integral(humidity, pressure).mean(axis=2)

    with _open(water_path) as dataset:
        observed = np.asarray(dataset.variables["pr_wtr"][:], dtype=np.float64)
        observed_latitude = np.asarray(dataset.variables["lat"][:], dtype=np.float64)
    if observed.shape[0] != months or not np.array_equal(observed_latitude, latitude):
        raise RuntimeError(
            f"{water_path.name} is on a different grid from {wind_path.name}; the "
            f"cross-check would compare fields that are not about the same places.")
    observed_zonal = observed.mean(axis=2)

    return ZonalProfiles(
        upper_wind_mps=_regrid(upper, latitude, bands),
        lower_wind_mps=_regrid(lower, latitude, bands),
        saturation_kg_per_m2=_regrid(saturation, latitude, bands),
        observed_water_kg_per_m2=_regrid(observed_zonal, latitude, bands),
    ), months


def read_sea_surface_temperature(path: Path) -> Tuple[np.ndarray, int, int]:
    """Reads monthly SST onto the asset's surface convention.

    The asset indexes latitude south to north and longitude eastward from the prime
    meridian, which is the OISST grid flipped in latitude and nothing else -- so the
    surface grid is the source grid, and no interpolation error is introduced at all.

    Land cells carry no SST. They are filled with the zonal mean of the same row rather
    than a sentinel, because a consumer sampling near a coast would otherwise blend a
    real temperature with a flag value and get a plausible number that is not one.

    @return `(kelvin[month][lat][lon], longitude_cells, latitude_cells)`.
    """
    with _open(path) as dataset:
        latitude = np.asarray(dataset.variables["lat"][:], dtype=np.float64)
        variable = dataset.variables["sst"]
        units = str(getattr(variable, "units", "")).lower()
        values = np.ma.masked_invalid(np.ma.asarray(variable[:], dtype=np.float64))

    if "c" in units and "k" not in units:
        values = values + 273.15

    if latitude[0] > latitude[-1]:  # OISST runs north to south; the asset does not
        values = values[:, ::-1, :]

    filled = np.asarray(values.filled(np.nan))
    latitude_count = filled.shape[1]
    for month in range(filled.shape[0]):
        rows = filled[month]
        missing = np.isnan(rows)
        if not missing.any():
            continue
        # A row can be entirely land -- around 80S the Antarctic continent spans every
        # longitude -- and nanmean warns rather than fails on those. The all-land case is
        # handled below, so the warning is noise about a case already covered.
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            row_means = np.nanmean(np.where(missing, np.nan, rows), axis=1)

        # An all-land row takes the nearest latitude that has ocean, not the global mean.
        # The global mean is about 287 K, and writing that under Antarctica would hand a
        # future surface-flux model a tropical ocean beneath the ice sheet. The nearest
        # ocean row gives it the Southern Ocean it is actually adjacent to.
        known = ~np.isnan(row_means)
        if not known.any():
            raise RuntimeError("the sea surface field has no ocean anywhere; the land "
                               "mask or the missing-value convention was misread")
        indices = np.arange(latitude_count)
        row_means = np.interp(indices, indices[known], row_means[known])

        rows[missing] = np.broadcast_to(row_means[:, None], rows.shape)[missing]

    return filled, filled.shape[2], filled.shape[1]
