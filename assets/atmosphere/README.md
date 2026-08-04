# T0 climatology

`climatology.set0` is the mean state the global dynamical core relaxes toward — the "what
is normally true here, at this time of year" that T1's weather is a departure from. See
`docs/design/atmosphere_system.md` §4.

It is committed rather than downloaded on demand: a scene must have a mean state without a
network, and this is 3.4 MB once.

## Reading it

```
se climatology inspect
```

prints the grid, the extremes of each profile, and the full attribution — which travels
*inside* the asset rather than in this file, so it cannot be separated from the data it
describes. Trust that string over this README.

## Rebaking it

```
pip install -e cli[climatology]
se climatology bake
```

Downloads about 15 MB of public reanalysis and coastline data (NCEP-NCAR Reanalysis 1,
NOAA OISST V2, Natural Earth — no credentials), derives the fields, and writes the asset.
Every run prints an audit of what it read and derived, and refuses to write if the
rasterised land total, the implied column humidity, the polar sea-surface fill, or a full
round trip of the written bytes disagrees with what it should be.

The download is cached under `build/climatology-cache/`; `--refresh` re-fetches.

## What is in it

| Field | Grid | Read by |
|---|---|---|
| Upper / lower zonal wind | 180 bands × 12 months | T1's mean state |
| Saturated column water | 180 bands × 12 months | T1's moisture relaxation |
| Land area fraction | 360 × 180 | T2 surface fluxes (not yet wired) |
| Monthly sea surface temperature | 360 × 180 × 12 | T2 surface fluxes (not yet wired) |

The two surface fields are baked but not yet read by anything — they come from the same
sources and the same asset, and the task that consumes them is named in the phase list
rather than left implied.

Missing or unreadable, the engine falls back to analytic latitude bands. That is a working
mean state, not an error state: it is what a non-Earth body uses, and it reproduces every
number the core measured before this asset existed.
