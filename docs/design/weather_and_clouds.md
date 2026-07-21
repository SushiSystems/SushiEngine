# Weather & Volumetric Clouds — Planetary Meteorology and a Flight-Sim-Grade Cloudscape

Status: **Design / engineering plan.** Companion to
[render_pipeline_refactor.md](render_pipeline_refactor.md) (its Phase 8 defers
here). Supersedes the retired `volumetric_clouds.md` (removed; in git history),
whose phases 0–4b shipped and whose remaining items are absorbed into the
roadmap below.

The plan is built on three inputs: a line-level audit of the current
implementation (§1), a primary-source survey of every major shipped system —
Guerrilla's Nubis 1/Evolved/3 decks, Hillaire's Frostbite course notes and the
2020 sky LUT paper, **War Thunder's actual open-source cloud renderer**
(DagorEngine `daSkies2`), MSFS/meteoblue, X-Plane 12, DCS, RDR2, UE/Unity HDRP —
and the constraint that made those systems fast (§2).

---

## 0. Goals and hard acceptance criteria

1. **Flight-sim-grade skies** — the visual bar is War Thunder / MSFS: distinct,
   sharply-shaped, genus-correct clouds from ground level to orbit.
2. **No visible repetition.** Raw tiled noise never reaches the screen
   unmodulated; the sky reads as *weather*, not as a texture.
3. **Altitude-stable quality.** Crisp at 0 m, at 3 000 m, at 20 000 m. Climbing
   changes the ray geometry, never the content quality. No altitude-driven
   sample starvation.
4. **Flyable-through.** Entering, flying inside, and breaking out of a cloud
   holds up: near-field detail, in-cloud whiteout, wisps at the canopy, no
   reprojection smearing.
5. **Dynamic meteorology for Earth.** Weather *evolves*: pressure systems and
   fronts move, wind has direction and altitude profile, humidity condenses into
   cloud and falls as precipitation, the diurnal cycle breathes cumulus. Driven
   by the same ephemeris clock as the sky.
6. **Extreme performance.** Hard budgets (§8): the whole cloud render ≤ 2.5 ms
   at 1080p internal on the High tier (mid-range GPU), weather simulation
   amortized to noise-level cost. Published reference points: Nubis3 flyable
   voxel clouds 2.1–4.0 ms on PS5 at 960×540; Frostbite 1.6 ms at 1080p on XB1;
   War Thunder ships its prebaked-field pipeline on far weaker hardware.
7. **Deterministic.** The weather simulation is part of the SushiLoop
   determinism domain: fixed-tick, seeded, serializable, replayable. Rendering
   consumes it and never feeds back.

---

## 1. Why the current clouds fail — audit summary

Line-level audit of `cloud.frag` (386 L), `sky.frag` (923 L),
`cloud_composite.frag`, `cloud_noise*.comp`, the passes, and the data model.
The user-visible complaints map to concrete root causes:

**"They repeat."**
- Coverage comes from one static 512² weather map (2 channels, plain fbm) over
  REPEAT-addressed noise; the domain warp is only 2 octaves
  (`cloud.frag:176–178`); nothing decorrelates the layers.
- `evolution_rate` is authored in the editor, packed into the UBO
  (`scene_uniforms.cpp:180`) and **never read by any shader** — the field
  translates with wind but never morphs. The weather is a still photograph.

**"They degrade with altitude."**
- The march budget is a direct function of camera altitude:
  `STEPS = mix(96, 32, altitude_factor)` (`cloud.frag:325–328`) — exactly the
  wrong variable. Step size ignores pixel footprint and distance.
- The march starts at the **outer atmosphere sphere**, not the occupied cloud
  band (`cloud.frag:306–318`), wasting samples in clear air above the decks.
- Empty-space skipping **charges the loop counter** (`t += big_seg; i += 2`,
  `cloud.frag:380–382`), so clear-sky pixels get far fewer real samples still.

**"They're slow."** (ranked)
1. The atmosphere itself: full-res per-pixel Nishita march, 32 view × 6 light
   steps ≈ **192 density evaluations per pixel, no LUTs** (`sky.frag:628–691`).
2. Cloud density: ~8 texture fetches × up to 6 decks × every sample; the 5-tap
   cone light march on top (`cloud.frag:123–267`). Nothing is precomputed.
3. No temporal accumulation of the march; every frame pays full price.
4. VRS steers the (cheaper) sky pass but **not** the cloud pass
   (`cloud_pass.cpp` vs `sky_pass.cpp:103`).
5. Cloud + composite passes register and run even when clouds are disabled
   (`vulkan_scene_view.cpp:690`).
6. The full cloud density stack is **duplicated** in `sky.frag` (two functions
   of it now dead there), inside a ~400-line `main()` — register pressure and
   compile bloat.

**"They look flat / disconnected."**
- Composite is plain bilinear, not depth-aware (`cloud_composite.frag:19–26`)
  → silhouette halos; half-res depth is sampled with a *linear* sampler
  (`cloud_pass.cpp:98`) → edge bleed.
- **No aerial perspective on clouds** — distant clouds never haze into the air
  column; ambient is a flat `scene.ambient + sun·0.02`, not sky radiance
  (`cloud.frag:338`, `cloud_composite.frag:25`).
- Ambient has no depth-into-cloud occlusion; interiors read uniform.

What *is* right and carries forward: the WMO genus catalogue and preset system
(`environment.hpp`), GPU compute noise generation, the half-res split pass, the
camera-relative march, per-pass GPU profiling, and the mesh cloud-shadow seam
(`cloud_shadow_common.glsl`).

---

## 2. Survey conclusions — what we adopt and why

| Source | What we take |
|---|---|
| **War Thunder (Dagor `daSkies2`, source)** | The backbone. **Prebaked 3D cloud field** (256×256×32 R8/BC4 baseline, XZ wraps a 65 536 m weather tile, Y = layer-band fraction) rebaked only on weather change; downsampled copy for empty-space skip; **per-slice occupied-altitude readback** clamping every ray to the real cloud band; procedural RGBA weather texture (layer1, layer2, type, cumulonimbus/rain) from decorrelated multi-octave signals; genus **type/height LUT** for vertical profiles; erosion noise at a deliberately **incommensurate 811 m** scale scrolled by wind; curl-noise warp at cloud bases; dedicated cloud buffer (RGBA16F + R8 TAA-weight + L16 depth) with its own variance-clip TAA; 8×8 tile min-depth composite; 768² cloud shadow map; panorama impostor for the far field and reflections. |
| **Nubis3 (Guerrilla 2023)** | The march and lighting upgrades. Step rule `max(SDF_or_skip_distance, 0.08·√dist, 1 m)`; jitter animated only < 250 m; HF detail only < 150 m; **decoupled lighting** — 2 inline sun taps + an amortized summed-density **light volume (256×256×32, refreshed over 8 frames, 0.1–0.2 ms)**; profile-gradient ambient `pow(1 − profile, 0.5)` (free dark edges / inner glow); **near/far resolution split** for the 60 Hz mode (¼-area near field — near content is low-frequency; distant silhouettes need the pixels). |
| **Frostbite (Hillaire 2016/2020)** | The integration math and the atmosphere coupling. **Analytic energy-conserving in-step scattering integration** (the single biggest sample-count reducer — 16 samples converge where naive marching needs hundreds); dual-lobe HG phase; jittered-EMA temporal integration as the robust fallback; **aerial perspective on clouds via transmittance-weighted mean depth** → one froxel/LUT sample; the 2020 LUT stack itself is the main roadmap's Phase 7 and is explicitly planet-scale. |
| **MSFS / X-Plane 12** | The weather *data model*: layered columns (coverage/type/density per altitude level + wind/T/RH), and the proof that a 3-stage blend (baseline → gridded model → local overrides) degrades gracefully. We synthesize the grid procedurally first; the representation stays ingestion-ready for real GRIB/METAR data later. |
| **DCS dynamic weather** | The cheapest credible synoptic generator: **moving elliptical Gaussian pressure systems**; wind = rotated pressure gradient (geostrophic, hemisphere-signed); cloudiness from pressure anomaly; fronts as geometry on a low's periphery. Analytic, deterministic, no grid required. |
| **Ghost of Tsushima** | The wind pattern: one analytic global field + local perturbations, *evaluated per consumer*, never a dense stored volume. |
| **Skip list** | 1/16-pixel reprojection upscaling (Guerrilla's own retraction for flythrough), full-res marching, 4-D scattering LUTs, mesh clouds, per-frame field rebakes, global fluid simulation (Stormscapes stays a hero-storm option for one 20 km domain, later, maybe never). |

---

## 3. Architecture — three weather tiers, one render tier

```
 T1  Synoptic (CPU, analytic, deterministic)         ~µs / tick
     pressure systems · fronts · jet bands · climate priors(lat, season)
        │  evaluated anywhere on the WGS84 sphere
        ▼
 T2  Regional grid (compute job, tick 10–30 s)       ≤0.1 ms amortized
     256×256 × 8–16 levels over ~1 000 km around the camera
     semi-Lagrangian RH/T advection · orographic lift · diurnal CAPE ·
     condensation → cloud water → precipitation → evaporation
        │  layered columns: coverage/type/density/wind per level
        ▼
 T3  Cloudscape compile (GPU, on weather change)     ≤1 ms, amortized
     RGBA weather texture (L1, L2, type, CN/rain)
     3D cloud field 256×256×32 R8/BC4 (+512×512×64 tier) · downsampled
     skip field · per-slice occupied-altitude readback · light volume seed
        │  wrap-tiled 65 km region, camera-recentered
        ▼
 T4  Render (every frame)                            ≤2.5 ms @1080p High
     field march + erosion + curl · decoupled lighting · dedicated
     cloud buffer + cloud TAA · depth-aware composite · AP coupling ·
     cloud shadow map · panorama far field
```

One-way data flow, three seams (DIP):

- **`Sim::IWeatherProvider`** — produces the layered column state. Two
  implementations: `ProceduralWeather` (T1+T2, deterministic, default) and,
  later, `IngestedWeather` (GRIB/METAR blend, X-Plane-style). The renderer
  cannot tell them apart.
- **`Render::CloudscapeCompiler`** — compiles column state into the GPU field
  set (T3). Pure function of its input; owns no policy.
- **The pass contract** — T4 passes consume the field set through graph handles
  like any other pass; the weather never reaches into rendering, rendering
  never writes weather.

The simulation tiers live in the **sim domain** (SushiLoop determinism: fixed
tick, seeded RNG, serialized with the scene, replayable); T3/T4 are render-side
and free to be nondeterministic (temporal jitter etc.). `Environment` carries
the compiled per-frame parameters exactly as it does today (OCP over the
existing seam) — `Cloudscape`/`CloudDeck` authoring remains as the *manual
override* mode, and presets become initial synoptic states.

---

## 4. The render tier (T4) in detail

### 4.1 Density: prebaked field + erosion (kills liabilities 2, and the repeat)

- Per-sample density = `field(weatherUV, heightFraction) · 2`, eroded by one
  detail fetch at the **811 m-class incommensurate scale**, curl-warped near
  cloud bases, exactly the Dagor recipe — **~2–3 fetches per sample instead of
  today's ~48** (8 × 6 decks). The 6-deck loop, the per-sample weather-map
  shaping, the streets fetch, and both domain-warp fetches all move into the
  T3 bake.
- The genus system survives as the **type/height LUT**: WMO genus → two
  density-height gradients + erosion strength per height band. Ten genera =
  ten LUT columns; `cloud_genus_profile()` becomes the LUT generator (pure
  data, same extension pattern).
- Anti-repetition, layered: unique planetary weather from T1/T2 modulates the
  tile; two cloud layers with decorrelated seeds and mirrored-offset UVs;
  incommensurate scale ladder (65 536 / 8 192 / 811 m + curl); erosion scrolls
  with wind so the field *morphs* (`evolution` finally means something — the
  dead `evolution_rate` uniform becomes the synoptic advance multiplier, §5).
- Optional Ultra: a 512×512×64 field (Nubis3-class ~8 m profile precision) and
  hand-placed NVDF-style hero envelopes near airfields.

### 4.2 Marching: altitude-stable by construction (kills the altitude complaint)

- **Delete the altitude LOD.** Step size depends on **distance and the skip
  field only**: `step = max(skip_distance, 0.08·√dist, min_step)` with mip
  escalation by distance. The march budget is a *tier* constant, not an
  altitude function.
- Ray bounds: analytic intersection with the **occupied altitude band** from
  the T3 readback (not the outer atmosphere sphere) — climbing to 20 km just
  shrinks the segment; sample density inside it is unchanged.
- Empty-space skipping via the downsampled field advances `t` **without
  charging the sample budget** (fixes the loop-counter burn).
- Jitter from the shared blue-noise sequence (main roadmap Phase 5), animated
  only < 250 m, static hash beyond — history stays stable at distance.
- HF erosion detail only < 150 m of the camera; in-cloud near field rendered
  by the close-layer pass (§4.4).

### 4.3 Lighting (the realism half)

- **Sun**: 2 inline exponential-step taps + the **amortized light volume**
  (256×256×32 summed density along the sun axis, refreshed across 8 frames,
  0.1–0.2 ms) — replaces the 5-tap cone march on every other sample.
- **Transmittance/scattering**: Beer-Lambert with 2–3 Wrenninge octaves *plus
  Hillaire's analytic in-step scattering integration* — this is what lets the
  step counts of §4.2 converge.
- **Phase**: dual-lobe HG (g ≈ 0.8 / −0.2), silver-lining boost tier-gated.
- **Ambient**: sky radiance from the Phase 7 sky-view LUT (or SH capture until
  it lands) × **profile-gradient ambient `pow(1−profile, 0.5)`** × the light
  volume's ambient sum — dark edges, inner glow, and depth-into-cloud
  occlusion all fall out of data we already have. Ground bounce = albedo-tinted
  bottom lobe. (Closes the flat-ambient audit item and the never-implemented
  "cloud AO".)
- **Cloud shadows**: one **768² transmittance map** projected along the sun,
  rebaked with the light volume cadence — replaces the 8-step ground march in
  `sky.frag` *and* the weather-map approximation in `cloud_shadow_common.glsl`
  with a single authority consumed by terrain, meshes, froxel fog, and aerial
  perspective. (One mechanism instead of two, cheaper than either.)
- **Lightning** (with §5 storms): light injected into the light volume with
  Nubis3's pseudo-attenuation — storms flash from inside.

### 4.4 Buffers & temporal (kills liabilities 3–4)

- Dedicated cloud buffer set at **½ res: RGBA16F color, R8 TAA weight, L16
  cloud depth** (transmittance-weighted mean depth — also the AP coupling
  input), plus the 8×8 tile min-depth for composite.
- **Dedicated cloud TAA** (War Thunder's shipped scheme): YCoCg variance clip
  (γ ≈ 1.2), nearest-depth motion vectors from the existing velocity target,
  history acceptance boosted under sub-pixel motion. The main TAA then treats
  the composited result like any other shaded pixel.
- **Near/far split when inside or near clouds** (Nubis3 60 Hz mode): < 200 m
  at ¼ area, beyond at ½ res — in-cloud content is low-frequency, distant
  silhouettes get the pixels. This is the flyable-through guarantee together
  with §4.2's near-field rules.
- **VRS**: the cloud pass binds the existing shading-rate image (one-line fix;
  it is the most expensive fragment work in the frame and the only pass that
  skipped it).
- Passes register **only when clouds are enabled** (graph-level skip, not a
  shader early-out).

### 4.5 Composite & atmosphere coupling (kills the disconnect)

- **Depth-aware upsample** using the tile min-depth + cloud depth (replaces
  the naive bilinear; fixes silhouette halos and the linear-sampler depth
  bleed).
- **Aerial perspective on clouds**: sample the Phase 7 aerial froxel volume
  once at the weighted mean cloud depth — distant clouds haze, desaturate,
  and sink into the horizon like terrain does. Until Phase 7 lands, an
  analytic single-sample fallback of the current scattering suffices.
- Sun disk, stars, and eclipse work stay in `sky.frag`; the dead duplicated
  cloud stack there is **deleted** (only the shadow-map consumer remains),
  shrinking the 923-line shader and its register pressure.

### 4.6 Far field

- **Panorama impostor** beyond ~100–150 km and for reflection probes / IBL
  capture: the same march rendered into a compressed panorama refreshed over
  many frames (War Thunder's low-end mode is our far-field mode). Reflection
  probes stop paying march cost entirely.

### 4.7 Quality tiers

| | Low | Medium | High | Ultra |
|---|---|---|---|---|
| Cloud buffer | ⅓ res | ½ res | ½ res + near split | ½–full + near split |
| Field | 128×128×32 | 256×256×32 | 256×256×32 | 512×512×64 (+hero envelopes) |
| Steps (view) | 32 | 48 | 64–96 | 96–128 |
| Light | volume only | 1 inline + volume | 2 inline + volume | 3 + volume, silver lining |
| TAA | EMA jitter | cloud TAA | cloud TAA | cloud TAA |
| Shadows | map 384² | map 768² | map 768² + fog coupling | + near shadow volume |
| Far field | panorama < 50 km | panorama < 100 km | panorama < 150 km | raymarch to 280 km |

---

## 5. The weather simulation (T1/T2) in detail

### 5.1 T1 — synoptic layer (analytic, global, deterministic)

- **Pressure systems**: N moving elliptical Gaussians on the sphere (DCS
  pattern) seeded from the RNG + climate priors (latitude bands, season from
  the ephemeris date; ITCZ, mid-latitude westerlies, polar easterlies as
  prior fields). Each system: center track, depth, radii, life cycle
  (deepen → mature → fill).
- **Wind** = geostrophic: `k̂ × ∇p` with hemisphere-dependent sign + friction
  turning near the surface + jet-band bias at altitude; altitude profile via
  a small per-level gradient (the WT `wind_alt_gradient` shape).
- **Fronts**: warm/cold front curves carried on each low's periphery as
  distance fields — stratiform sheet ahead of the warm front, convective
  line at the cold front, exactly the textbook geometry, all analytic masks
  feeding T2's humidity/type fields.
- Evaluated anywhere on the WGS84 sphere in microseconds; serialized as the
  system list (a few hundred bytes). `evolution_rate` (the dead knob) becomes
  the synoptic time multiplier.

### 5.2 T2 — regional grid (the meteorology)

- Grid: **256×256 cells × 8–16 levels over ~1 000 km**, camera-centered,
  rebased with the floating region (camera-relative discipline; rebase story
  documented like the CSM texel snap). Per cell-level: wind (u, v),
  temperature offset, humidity/dew point, cloud water, precipitation. This is
  the X-Plane/MSFS column representation — proven, and ingestion-ready.
- Tick every **10–30 s** (compute job or SYCL, amortized over frames):
  1. Semi-Lagrangian advection of RH/T by the T1 wind (unconditionally
     stable at any Δt).
  2. **Orographic lift**: `w ≈ u·∇terrain` → windward RH boost/condensation,
     leeward föhn drying (needs only the coarse terrain height field).
  3. **Diurnal convection**: insolation(lat, ephemeris ToD) × surface type →
     CAPE proxy → cumulus growth/decay; the sky *breathes* with the day.
  4. Moisture closure: condensation → cloud water → precipitation threshold →
     evaporation; drives the CN/rain channel and clears rained-out cells.
- Output: the layered columns T3 compiles. Interpolated in time between ticks
  so nothing steps visibly.
- **Determinism**: fixed tick on the SushiLoop clock, seeded, no render-side
  inputs; state serializes with the scene and replays bit-exactly (float
  discipline as per the physics solver).

### 5.3 Coupling weather → world (beyond clouds)

- **Fog/visibility**: dew-point spread + precipitation → froxel-fog density
  and Mie turbidity in the atmosphere parameters (they are runtime-dynamic in
  the Hillaire model). Valley fog from the orographic term.
- **Precipitation rendering**: rain/snow particle systems under raining
  cells; cloud-base darkening from the rain channel (WT's
  `density += density·weather.a`); the 768² shadow map doubles as the
  rain-occlusion mask.
- **Wet surfaces**: wetness parameter per region → albedo darken + roughness
  drop in the material system (a material-flag path, no shader fork).
- **Wind consumers**: one sampling API `weather_wind(position, altitude)`
  (GoT pattern — analytic + perturbation, no dense volume): cloth, particles,
  ocean/sea state (main doc Phase 13 water), and the flight model. Turbulence
  intensity from front proximity + CAPE + terrain roughness — the flight-sim
  payoff.
- **Lightning**: probability from the CN channel; render via §4.3.

### 5.4 Real-data seam (later, optional)

`IngestedWeather` implements `IWeatherProvider` from GRIB (GFS/WAFS winds,
temperature, humidity) blended toward METARs near airfields — the X-Plane 12
three-stage blend. The column representation is already identical; nothing
downstream changes. Multiplayer determinism: ingested snapshots are
timestamped inputs distributed like any other command.

---

## 6. Editor integration

- **Weather panel v2** (replaces the Clouds panel's Advanced section):
  synoptic map overlay (pressure systems, fronts, wind barbs — drawn from T1
  state), time-scrub coupled to the ephemeris clock, per-system authoring
  (place/drag a low, set depth), preset buttons now seed synoptic states
  (Clear / Fair / Overcast / Front Passage / Storm).
- **Manual mode** keeps the existing `Cloudscape`/deck authoring for
  screenshots and tests — a static `IWeatherProvider` implementation.
- **Debug views**: weather texture channels, cloud field slices, light
  volume, skip-field heatmap, march-step count heatmap (the profiler HUD
  already hosts per-pass ms).

---

## 7. Phased roadmap

Each phase ships editor surface, tier wiring, profiler budget, and
CHANGELOG/ARCHITECTURE entries per repo policy. W0 is pure wins on the
existing code; the system rebuild starts at W1.

### W0 — Triage: fix what's measurably wrong today (small, immediate)

1. March bounds: clamp to the **deck band** `[base_r, top_r]`, not the outer
   sphere; skip advances stop charging the sample budget.
2. **Delete the altitude step LOD**; steps become distance-driven
   (`0.08·√dist` rule) at constant budget.
3. Bind **VRS** in the cloud pass; register cloud passes only when enabled.
4. **Depth-aware composite** (tile min-depth) + point-sampled depth in the
   march; blue-noise dither replaces `hash13`.
5. Delete the dead cloud stack from `sky.frag`; wire `evolution_rate` to
   erosion-noise scroll as a stopgap so the sky stops being static.
6. Record baseline per-pass ms in the profiler HUD before/after.

*Acceptance: no altitude-dependent quality cliff, no silhouette halos,
measured cloud+sky ms reduction, clouds visibly evolve.*

### W1 — Cloudscape compile (T3): the prebaked field

Weather texture (RGBA = L1/L2/type/CN) generated on change; **3D cloud field
+ downsampled skip field + occupied-altitude readback**; genus type/height
LUT replacing the per-sample 6-deck loop. Current lighting kept.
*Acceptance: per-sample fetch count ~48 → ~3; two decorrelated layers on
screen; march cost drops by the profiler's word.*

### W2 — March & lighting upgrade

Nubis3 step rule over the skip field; 811 m erosion + curl warp;
dual-lobe HG + analytic in-step integration + Wrenninge octaves; **amortized
light volume**; profile-gradient ambient; **768² cloud shadow map** replacing
both legacy shadow paths.
*Acceptance: High tier ≤ 2.0 ms at 1080p internal for the cloud pass on the
reference GPU; dark edges/inner glow visible; one shadow authority.*

### W3 — Temporal & optical integration

Dedicated cloud buffer + cloud TAA; near/far res split (flyable-through);
aerial perspective via weighted mean depth (froxel once Phase 7 lands);
sky-LUT ambient; panorama far field + reflection-probe path.
*Acceptance: no shimmer under motion at ½ res; inside-cloud flight holds
detail; distant clouds haze into the horizon; reflections cost ~0.*

### W4 — Meteorology (T1 + T2)

Synoptic systems + fronts + geostrophic wind; regional grid with advection /
orographic / diurnal / moisture closure; deterministic tick + serialization;
Weather panel v2. Presets become synoptic seeds.
*Acceptance: a front visibly crosses the region over authored minutes —
stratus sheet, wind veer, cumulus line, clearing; replay is bit-exact.*

### W5 — World coupling

Fog/visibility + turbidity; precipitation particles + base darkening + wet
surfaces; lightning; `weather_wind()` API into cloth/particles/flight model;
sea-state hook.
*Acceptance: rain falls from the cell that is raining, under a dark base,
with wet ground and reduced visibility — one cause, every symptom.*

### W6 — Flight-sim polish & data ingestion (tier/optional)

In-cloud whiteout tuning, canopy wisp particles, icing/turbulence exposure to
gameplay, hero envelope assets near airfields (Ultra), `IngestedWeather`
(GRIB/METAR) behind the provider seam.

---

## 8. Performance budget summary (High tier, 1080p internal, mid-range GPU)

| System | Budget | Amortization |
|---|---|---|
| Cloud march (½ res + near split) | ≤ 2.0 ms | per frame |
| Cloud TAA + composite | ≤ 0.3 ms | per frame |
| Light volume + shadow map refresh | ≤ 0.2 ms | over 8 frames |
| T3 compile (field + weather tex) | ≤ 1.0 ms | on weather change, spread over frames |
| T2 grid tick | ≤ 0.1 ms equiv. | every 10–30 s, async/SYCL |
| T1 synoptic | CPU µs | per tick |
| Panorama refresh | ≤ 0.2 ms | over many frames |
| **Total steady-state** | **≤ 2.7 ms** | vs. multiple ms today with less realism |

The atmosphere LUT stack (main doc Phase 7) removes a further large fixed
cost from `sky.frag` independently.

---

## 9. SOLID

- **SRP** — weather simulation (sim domain), cloudscape compilation, and
  rendering are three modules with one reason to change each; the genus LUT
  is data, `cloud_genus_profile()` its generator.
- **OCP** — new genera, new presets, new synoptic behaviors are data; new
  render features are new passes; `Environment` extends, never mutates
  callers.
- **LSP** — every `IWeatherProvider` (procedural, static-manual, ingested) is
  interchangeable behind one contract the compiler consumes.
- **ISP** — the renderer sees compiled fields and parameters, never the
  simulation; gameplay sees `weather_wind()`/state queries, never textures.
- **DIP** — the sim depends on the provider abstraction; the compiler on the
  column schema; the passes on graph handles. Vulkan stays in `rhi/vulkan`.

## 10. Dependencies on the main roadmap

- **Phase 5** (blue-noise asset) → W0.4; **Phase 7** (sky-view/AP LUTs) →
  W3's froxel coupling and LUT ambient (analytic fallbacks specified until
  then); **Phase 11** (async compute / SYCL interop) → T2's preferred home
  (a graphics-queue compute job is the interim); **Phase 13** water → sea
  state from `weather_wind()`.
- Nothing here blocks the main roadmap; W0–W2 can start immediately after
  main-doc Phase 3 (they touch only cloud files and the new modules).
