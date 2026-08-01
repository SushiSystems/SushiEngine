# Baked planetary terrain

This directory holds `.planet` assets — the cube-sphere height pyramids the terrain system
reads (`docs/slop/solar_system_overhaul.md` §5.2, `include/SushiEngine/terrain/pack_format.hpp`).

**They are not committed**, and that is deliberate. The compact lunar tier is 17 MB and the
deeper tiers are hundreds; unlike `assets/atmosphere/climatology.set0` at 3.4 MB, these are
too large for a source tree. The same reasoning as `assets/hrtf/`.

## Getting one

```
pip install -e cli[planet]      # numpy + requests, an optional extra
se planet bake --body moon --tier compact
se planet inspect
```

The bake downloads a public-domain topography raster (LOLA for the Moon, from the NASA PDS
Geosciences Node — no credentials), verifies it reads as the body it claims by sampling
known landmarks, reprojects it onto the cube-sphere, and writes the asset with its full
provenance and source checksums inside it. `se planet inspect` prints that provenance back.

## Without one

Nothing breaks. `PlanetPack::load_planet_pack` returns an unloaded pack, `PackHeightSource`
reports no coverage, and the body falls back to the analytic ground the sky pass already
draws — which is what shipped before terrain existed.

## Attribution

Every source is public domain (NASA / NOAA / USGS). The attribution string travels inside
each asset rather than living only here, so an asset copied away from this checkout still
says where it came from.
