# Volumetric Cloud Overhaul

Status: in progress · Owner: renderer · Target: flight-sim-grade, real-altitude cloudscape

## Problem

The existing cloudscape (single-pass volumetric ray march in `render/shaders/sky.frag`,
noise volumes generated CPU-side in `render/rhi/vulkan/vulkan_scene_view.cpp`, genus-deck
model in `include/SushiEngine/render/environment.hpp`) reads as obviously synthetic: soft,
repetitive blobs with flat lighting and no felt volume. The architecture is sound — the
defects are in the frequency/scale tuning of the noise and in the shallowness of the
lighting, not in the ray-march structure.

### Diagnosis (ranked by visual impact)

1. **Noise resolution is far too low for its tile scale.** The shape volume is `96³`
   sampled over `shape_scale = 40000 m` (cumulus) → **~416 m per texel**; the smallest
   trilinearly resolvable feature is ~830 m. A ~1 km cumulus is therefore described by a
   single noise lump — no cauliflower silhouette, no billows. The detail volume is `32³`
   over `detail_scale = 4500 m` → ~140 m per texel, so the "high-frequency" erosion acts
   at ~280 m and carves nothing fine. This is the primary cause of the blobby look.
2. **The weather/coverage field is uniform.** `weather_scale = 60000 m` varies coverage
   only over tens of km, so a cloud field is an evenly spaced carpet of identical puffs
   instead of the fractal clumping (cloud streets, meso-scale clusters, clear breaks) of a
   real sky.
3. **Lighting is shallow.** The light march toward the sun samples in a straight line (no
   cone), so self-shadow gradient — the main cue for volume — is weak. Ambient is a linear
   `mix(0.35, 1.0, height01)` with no depth-into-cloud occlusion and no multiple-scatter
   energy term, so clouds read flat/plastic rather than dark-based and bright-cored.
4. **Undersampling.** Empty-space skipping charges the bounded loop (`i += 2`), lowering
   the real sample count; combined with (1) it further flattens the volume.

### What actually creates the 3D-volume percept

- Correct noise frequency: texel-to-feature ratio near 10–50 m, not ~400 m.
- Cone-sampled light march → core-to-edge brightness gradient (self-shadowing).
- Beer–Powder plus an octave multiple-scatter approximation → dark base, silver-lined
  edge, bright core.
- Depth-into-cloud ambient occlusion.

## Plan

Sequenced, each phase independently buildable and visually verifiable. Priority: balanced
quality/perf (half-res march + temporal upsampling target 60+ FPS), flight-sim use case.

- **Phase 0 — Scale/resolution fix (highest impact, lowest risk).** Raise `SHAPE_RES`
  96→128 and `DETAIL_RES` 32→64; retune `shape_scale`/`detail_scale` in the cumuliform
  genus profiles toward physical cloud sizes. Expected: crisp cauliflower structure from a
  single change.
- **Phase 1 — Lighting depth.** Cone-sampled `cloud_light_march`; Beer–Powder plus a
  2-octave multiple-scatter energy approximation; depth-based ambient occlusion.
- **Phase 2 — Realistic distribution engine.** Fractal, clustered coverage (cloud streets,
  meso clumps, clear breaks) driven by the weather field; decks placed by true étage
  altitudes, hardening the existing genus system.
- **Phase 3 — Easy cloud tab.** One-click weather presets (Clear / Fair-weather cumulus /
  Overcast / Storm) over the existing genus-deck model; advanced controls behind a
  collapsible section. Presets are pure-data functions, mirroring `cloud_genus_profile`
  (open for extension, closed for modification).
- **Phase 4 — Optimization.** Half-resolution march + temporal reprojection/upsampling,
  early-out thresholds, altitude-adaptive step budget (the camera lives inside the
  atmosphere in flight, so LOD is budgeted against altitude).

## Progress

- **Phase 0 — done.** Shape `128³`, detail `64³`, cirrus `96³`; cumuliform genus
  `shape_scale`/`detail_scale` retuned so crispness comes from detail erosion, not a small
  base tile.
- **Phase 1 — done.** Cone light march; 3-octave Beer/Wrenninge multiple-scatter energy;
  two-octave domain warp for anti-tiling.
- **Ground gating fixed.** Clouds march over terrain (stop at ground / geometry), not only
  against open sky — the flight-sim down-looking view.
- **Phase 2 — done.** Clustered two-scale coverage with wind-aligned streets and broad
  self-warp; distinct clouds with clear gaps.
- **White lighting — done.** Multiple-scatter octaves blended toward isotropic and rescaled
  by 4π so lit cloud reads white from any angle, not only a sun-ward rim; powder default
  1.0→0.3, ambient fill 0.4→0.5.
- **Phase 4a (shader-side) — done.** Cheap density probe in empty space, distance LOD to
  the cheap density, light-march reuse every other lit sample, 5 light taps, earlier
  cutoff, max march budget 128→96.
- **Phase 3 — done.** `WeatherPreset` + `cloud_weather_preset` (pure data) with editor
  preset buttons; per-deck and medium controls moved under an *Advanced* node.
- **Phase 4b — done (half-res pass); temporal upsampling deferred.** The cloud march is
  split into `cloud.frag`, rendered at half width/height into a new HDR target
  (`scattered.rgb`, `transmittance`) reusing the sky descriptor set (UBO + depth + noise);
  the tonemap pass gains a binding-2 cloud sampler and composites it over the sky
  (`sky * transmittance + scattered`) with a linear upsample. New pipeline `cloud_pipeline_`,
  new per-slot `cloud` image/view, one extra descriptor write, and a cloud pass with its own
  layout barriers between the sky and tonemap passes. Depth-aware/temporal upsampling to
  remove half-res edge softening is a later refinement. Target: 60+ FPS on a GTX 1060.

## Constraints

- C++17 only; Allman braces; components trivially copyable; SOLID.
- Docs ship in the same PR (CHANGELOG, ARCHITECTURE, this doc).
- One-way dependency `SushiEngine → SushiRuntime` preserved; no device code added to the
  engine — the cloud noise is generated on the host and the march stays in `sky.frag`.
