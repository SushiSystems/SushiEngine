# HRTF datasets

The measured-HRTF path (`sofa_hrtf.hpp` → `BinauralSpatializer::set_hrtf_database`) loads
[SOFA](https://www.sofaconventions.org/) HRIR files (an HDF5 container, AES69
`SimpleFreeFieldHRIR` convention).

`*.sofa` files are **not committed** (large binaries — see `.gitignore`). `audio_sofa_demo`
loads a real set from this directory if present and otherwise bakes a synthetic ring with
`write_sofa`, so it runs either way.

To use a real set, drop a `.sofa` file here. The demo looks for:

```
assets/hrtf/mit_kemar_normal_pinna.sofa
```

which can be fetched from the public SOFA database, e.g.:

```
curl -L -o assets/hrtf/mit_kemar_normal_pinna.sofa \
  https://sofacoustics.org/data/database/mit/mit_kemar_normal_pinna.sofa
```

Any `SimpleFreeFieldHRIR` SOFA file works; the loader resamples the impulse responses to the
stream rate on load.
