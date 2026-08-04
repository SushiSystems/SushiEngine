# Atmosphere — Real Meteorology on the GPU and a Planet-Scale Cloudscape

Status: **Design / engineering plan.** Companion to
[render_pipeline_refactor.md](render_pipeline_refactor.md) (its Phase 8 defers here).

**Supersedes `weather_and_clouds.md`** (removed; in git history), whose W0–W3 render
tier shipped and carries forward largely intact (§8), and whose W4–W6 simulation tier
shipped but does not do what it claims (§1) and is replaced wholesale by this document.

The plan rests on three inputs: a line-level audit of the shipped W0–W6 implementation
(§1), the actual practice of numerical weather prediction and the reduced-physics model
hierarchy underneath it (§2), and three decisions taken up front that make real
meteorology affordable in a game (§0).

---

## 0. The three decisions this document is built on

**1. Weather leaves the determinism domain.** The atmosphere is no longer part of
SushiLoop's fixed-tick, seeded, rollback-serialized state. It runs free on the GPU and
couples to gameplay *loosely*: a small, low-frequency, latency-tolerant summary
(§9) is what crosses into the deterministic world, and only that summary carries a
guarantee. This is the decision that unlocks everything else — a bit-exact-replayable
atmosphere would have forced fixed-order reductions, no atomics, and a grid an order of
magnitude smaller than the one physics actually needs.

**2. Both ground level and every flight altitude are first-class.** The tier stack
therefore spans the whole hierarchy: a global core so the planet is coherent at 5 000 km,
a regional nest so convection is explicit at 2 km, and a data plane rich enough to answer
"what is the icing risk at FL180" as well as "is it raining on this hill".

**3. Spatial coupling before physics.** The single largest defect in the shipped system
is not the quality of its physics — it is that *none* of its physics reaches the screen
spatially (§1.1). Phase A (§11) fixes only that, using the existing physics, and is
expected to change the look of the sky more than every later phase combined.

The consequence of decision 1 that must be stated plainly: **`test_weather_determinism.cpp`
and the bit-exact-replay guarantee it proves are deliberately being given up.** What
replaces them is §9's contract — the summary the deterministic world consumes is a
snapshotted, versioned input, distributed like any other command in a networked session,
exactly as `weather_and_clouds.md` §5.4 already described for ingested data.

---

## 1. Audit — why the shipped W4–W6 weather is not what it claims

### 1.1 The root cause: the meteorology never reaches the sky spatially

`RegionalWeatherGrid` simulates 64×64 columns × 3 levels over 1 000 km
(`regional_weather_grid.hpp:81-84`). `RuntimeSimulation::extract()` then samples
**exactly one column — the one under the observer** (`sim/runtime_simulation.cpp:2580`)
and compiles it into a **globally uniform** `Render::Cloudscape` of six decks.

What produces the pattern actually visible in the sky is `cloud_noise_weather.comp`: a
static procedural fBm texture, generated once. `cloudscape_field.comp:138` reads

```glsl
float coverage = clamp(la.z + (weather - 0.5) * 1.35, 0.0, 1.0);
//                     ^^^^ the deck's global coverage_bias — the simulation's entire contribution
//                            ^^^^^^^ tiled fBm noise — the entire visible structure
```

So the simulation's only effect on the rendered sky is to slide the global bias of a
tiled noise texture. Every consequence follows from this one line:

- A front "crossing the region" is a global coverage number ramping. There is no front
  in the sky, no leading cirrus deck ahead of it, no clearing behind it.
- Rain here and sun 40 km away is not representable. Precipitation is a scalar for the
  whole world (`WeatherColumn::precipitation`), which is why the rain emitter is
  camera-attached (`runtime_simulation.cpp:2776`) rather than placed under the cell that
  is actually raining — the stated W5 acceptance bar, unmet.
- Advection is invisible. The pattern you see is a static texture; it cannot translate
  with the wind, because the wind only moves data the renderer never reads.
- The tiling is visible precisely because the noise *is* the signal, at full amplitude,
  rather than a detail modulation on top of a unique field.

### 1.2 The data contract is the ceiling

`WeatherColumn` (`weather_types.hpp:146`) is 3 levels × 4 floats + 3 floats ≈ **15 floats
at one point**, and it is the entire output of `IWeatherProvider`. It cannot express:
cloud base or top altitude, visibility, absolute temperature or pressure, dew point, a
wind profile (only surface `u`/`v` exist), precipitation type, freezing level, liquid
water content, ice phase, turbulence, or vertical velocity.

No improvement to the simulation can pass through this. Three downstream honesty notes
in the shipped code are all the same defect surfacing: rain can never be snow because
there is no temperature (`runtime_simulation.cpp:2787`); `icing_risk` had to have
`temperature_offset_c` retrofitted into the struct in W6 to exist at all; visibility is
derived from a coverage proxy rather than from anything optical.

**The interface shape, not the CPU-ness, is the architectural root cause.**

### 1.3 The model is heuristics, not thermodynamics

`regional_weather_grid.hpp:386-496`, `tick_grid` — a single 110-line function with
thirteen magic constants declared in its own body:

| What meteorology requires | What the code does |
|---|---|
| Saturation vapour pressure (Clausius–Clapeyron / Magnus) | `if (humidity > 0.85)` |
| Mixing ratio `q_v` [kg/kg] | dimensionless `humidity ∈ [0, 1.3]` |
| Latent heating on condensation | none; `cloud_water += condensed * 8.0`, commented "arbitrary units" |
| Hydrostatic balance, lapse rate, pressure coordinate | none |
| **Vertical advection / vertical velocity** | **none — the three levels are independent 2-D fields** |
| Buoyancy, parcel ascent, LFC/EL, real CAPE | `cape = insolation * humidity - 0.3` |
| Warm-rain microphysics (autoconversion, accretion, evaporation) | `if (cloud_water > 0.5)` |
| Geostrophic balance | correct form, then scaled by `WIND_SCALE = 6.0e4` (`synoptic_weather.hpp:255`) — a tuning knob, not a physical constant |
| Coriolis across the tropics | floored at `|sin φ| ≥ 0.15` (`synoptic_weather.hpp:246`) — ITCZ, monsoon, and tropical cyclones are structurally impossible |

The absence of vertical advection is decisive. Convection *is* the vertical transport of
moisture and heat; with three independent layers a cumulus cannot grow, a cumulonimbus
cannot build an anvil, a cold pool cannot form and trigger the next cell. These are
exactly the phenomena that read as "AAA sky".

Fronts are not simulated either: `front_proximity` (`synoptic_weather.hpp:287`) is a
fixed-angle ray pair hung off each low at 60°/135°, an analytic mask — the document that
specified it says so. Frontogenesis, the process that makes fronts sharp and is the
reason they look the way they do, is absent.

### 1.4 The advection scheme is numerically dead

Cell size is 1e6/64 = **15.6 km**; the tick is 15 s. At 20 m/s the semi-Lagrangian
departure point moves 300 m — **0.019 of a cell**. Bilinear interpolation at Courant
number ≪ 1 is the maximally diffusive regime: each tick blends ~98 % of a cell with ~2 %
of its neighbour, 240 times per simulated hour. Any sharp gradient — which is to say, any
front — is smeared into a gradient-free mush within hours.

The fix is standard and is adopted in §6: choose Δt so Courant ≈ 1, and use a monotone
higher-order scheme (cubic Hermite, MacCormack, or BFECC) rather than plain bilinear.

### 1.5 CPU, single-threaded, with the SYCL runtime sitting idle

`tick_grid` is a doubly nested scalar loop: 64² cells × 3 levels, each evaluating
`front_proximity` (loop over up to 8 systems with `exp`/`sqrt`),
`solar_elevation_fraction` (`sin`/`cos`/`fmod`), three bilinear gathers, and four
`std::function` indirect calls into `orographic_lift` — **which is a documented no-op
returning 0**, because no terrain height field exists in the engine
(`regional_weather_grid.hpp:50-55`). It runs synchronously on the simulation thread. The
resolution the superseded document actually asked for (256×256) is 16× this cost, i.e. a
tens-of-milliseconds hitch every 15 s of simulated time.

Meanwhile physics (`physics_simulation.hpp:253`), animation, audio, and VFX all take a
`SushiRuntime::API::Runtime&`. Weather is the only simulation system in the engine that
never touches the GPU.

### 1.6 SOLID

- **DIP, broken at the top.** The abstract interface returns a concrete class:
  `virtual ProceduralWeather* procedural_weather()` (`simulation.hpp:733`), and the host
  stores `std::unique_ptr<ProceduralWeather>` (`runtime_simulation.cpp:3301`), not
  `IWeatherProvider`. The consequence is on the record in the W6 CHANGELOG entry:
  `IngestedWeather` is written, unit-tested, and **impossible to install** — "not wired
  into `RuntimeSimulation`'s `procedural_weather_` slot". The seam whose purpose was to
  prove LSP cannot be used.
- **SRP.** `RegionalWeatherGrid` is simultaneously grid storage, floating-origin rebase,
  advection, microphysics, insolation/radiation, background climatology, and temporal
  interpolation.
- **OCP.** `pick_low_genus`/`pick_mid_genus`/`pick_high_genus`
  (`weather_cloudscape_compiler.hpp:129-150`) are hard-coded `if` chains over the genus
  enum; every physical constant is a function-local `constexpr`. A new microphysics
  scheme, a new genus, or a per-biome parameter set means editing these classes.
- **ISP.** `IWeatherProvider` exposes a *point* query and nothing else. There is no way
  to ask it for a field — which is the direct interface-level cause of §1.1.
- **DRY / correctness.** `solar_elevation_fraction` (`regional_weather_grid.hpp:308`)
  reimplements a solar-position model to avoid depending on `Astro::Ephemeris`. The sun
  that heats the ground and the sun that is rendered are two different suns.

### 1.7 What carries forward

The render tier is not the problem. W0–W3 shipped competent, measured work and survives
this rewrite nearly intact: the prebaked field + skip field + occupied-altitude readback,
the Nubis3 step rule, the amortized light volume, the 768² cloud shadow map as the single
shadow authority, the dedicated cloud buffer with its own YCoCg-variance-clip TAA, the
near/far split, the depth-aware composite, the panorama impostor, the quality tiers, and
the per-pass GPU profiling. §8 states precisely what changes there, and it is small.

Also carried forward: the WMO genus catalogue and preset system as an *authoring* and
*labelling* vocabulary (§7.4), the METAR parser (`metar_parser.hpp`), and the shape of
the gameplay query APIs in `weather_wind.hpp` / `weather_flight_hazards.hpp` — all
retargeted onto a new data source rather than rewritten.

---

## 2. What real meteorology simulation is, and what a game can afford

### 2.1 Operational numerical weather prediction

GFS, IFS, ICON, WRF, HRRR all solve the same thing: the primitive equations on a rotating
sphere — hydrostatic or fully non-hydrostatic compressible Euler, plus moisture — with
semi-implicit semi-Lagrangian time stepping, and with the physics that actually determines
the weather living in *parameterizations*: radiation (RRTMG), boundary layer (YSU, MYNN),
deep convection (only where unresolved), microphysics (Thompson, Morrison), and a land
surface model (Noah-MP). Global grids are 9–13 km, regional 1–3 km. Cost is
supercomputer-hours per forecast day.

None of that is real-time, and none of it is what a game needs: a game needs an
atmosphere that is *coherent, evolving, and physically honest*, not one that is
predictive of a specific real Tuesday.

### 2.2 The reduced-physics hierarchy that is real-time

Meteorology has a well-developed hierarchy of reduced models, each of which is a real
model with real emergent behaviour, not an approximation of appearance:

| Model | Produces emergently | Real-time cost |
|---|---|---|
| **2-layer moist quasi-geostrophic / shallow-water primitive** on the sphere | Baroclinic instability → mid-latitude cyclones, **frontogenesis**, Rossby waves, jet meanders, storm tracks | 512×256×2, GPU, sub-100 µs per step |
| **Anelastic / Boussinesq non-hydrostatic cloud model** over a regional domain | Explicit convection: thermals, updrafts, cumulus life cycle, anvils, **cold pools** triggering the next cell, squall lines | 256×256×48, GPU, single-digit ms per step (the shipped 192×192×48 nest measures ~8 ms on a GTX 1060 — §6) |
| **Warm-rain (Kessler) + simple ice microphysics** | Cloud → rain → evaporation → downdraft; snow vs. rain vs. graupel by phase | 3–5 extra advected fields |
| **Surface energy balance + bulk boundary layer** | Diurnal thermals, sea breeze, valley fog, orographic enhancement | 2-D, negligible |
| **LES patch** (100 m) | Individual thermals, rotor/turbulence structure, hero storms | 200³, expensive, optional |

The counter-intuitive affordability argument, stated once because it drives every budget
in §12: **these models are stepped in game time, and their stability-limited time steps
are far longer than a frame.** The regional nest's Δt is ~6 s of simulated time (the
measured CFL bound — §11's B2c); at 1× time scale that is one step every six seconds of
wall clock. A 1.8-million-cell non-hydrostatic step costing ~8 ms as measured, taken
every 6 s, amortizes to **~0.02 ms per frame** at 60 Hz. Real cloud physics is cheap in a game not because it is simplified, but because
weather is slow.

### 2.3 What we take, and what we skip

| Source | What we take |
|---|---|
| **Two-layer QG theory (Phillips 1954; Holton ch. 6)** | The global core. The minimum model that produces cyclogenesis and fronts *emergently* from a baroclinically unstable mean state, rather than placing them by hand. |
| **Anelastic cloud models (Ogura–Phillips; Klemp–Wilhelmson; the WRF/CM1 lineage)** | The regional nest. Sound-filtered so Δt is set by advection and buoyancy, not by acoustic waves — the reason a non-hydrostatic model is affordable at all. |
| **Kessler (1969) warm rain; Rutledge–Hobbs ice** | The microphysics. The standard minimal closed scheme: `q_v`/`q_c`/`q_r` (+ `q_i`/`q_s`), saturation adjustment, autoconversion, accretion, evaporation, terminal fall speed. |
| **Davies (1976) relaxation nesting** | One-way T1 → T2 lateral boundary coupling with a relaxation zone, the standard technique, instead of hard boundary injection that reflects. |
| **MacCormack / BFECC advection (graphics lineage: Selle et al.)** | Monotone, low-diffusion semi-Lagrangian transport at Courant ≈ 1 — directly fixes §1.4. |
| **Reanalysis climatology (shipped as NCEP-NCAR R1 — §4) / ETOPO / MODIS land cover** | T0 climatology as *data*: zonal-mean jets, monthly SST, terrain, surface type. The reason the Sahara differs from the Amazon without simulating either. |
| **Nubis3, Frostbite, War Thunder `daSkies2`** (via the superseded doc) | The entire render tier — unchanged and shipped. See §8. |
| **X-Plane 12 / MSFS** | The data-plane shape: a real vertical *profile*, and a three-stage blend (model → gridded → local observation) that degrades gracefully. §9's `AtmosphereProfile` and the retained METAR path. |
| **Skip list** | Spectral transform cores and cubed-sphere grids (a polar Fourier filter is enough at our resolution), full radiative transfer, two-moment microphysics, data assimilation, chemistry/aerosol transport, ocean coupling, two-way nesting, and any attempt at forecast skill. |

---

## 3. Architecture — the `Atmosphere` module

### 3.1 Tier stack

```
 T0  Climatology (data, not simulation)                    static assets
     SST · terrain · land cover · zonal-mean jet · solar constant
        │  relaxation targets and surface forcing for T1/T2
        ▼
 T1  Global dynamical core (CPU, §3.3)                     ~36 ms per step, ~1 step/6 min game time
     2-layer moist QG on 512×256 lat/lon (~78 km)
     emergent: cyclogenesis · fronts · jet · storm tracks · precipitable water
        │  one-way nesting: Davies relaxation into T2's boundary zone
        ▼
 T2  Regional nest (GPU)                                   ~8 ms per step (measured), ~1 step/6 s game time
     anelastic non-hydrostatic, 192×192×48 over 384 km (2 km / ~54-560 m, High tier — §6)
     explicit convection · Kessler microphysics, diagnosed ice phase · surface fluxes · orography
        │  θ, q_v, q_c, q_r, u, v, w, π'
        ├──────────────────────────────┐
        ▼                              ▼
 T3  Cloudscape compile (GPU)     Query mirror (async readback)
     extinction field from q_c/q_i   coarse fields + player column, 2-3 frames stale
     camera-centred, non-wrapping    ↓
     skip field · light volume       gameplay · audio · flight model · VFX
        │
        ▼
 T4  Render (every frame)                                  ≤2.5 ms @1080p High
     unchanged from W0-W3 apart from its density source (§8)
```

### 3.2 Two faces, one authority

The atmosphere's state lives on the GPU and is written by exactly one path. It is read by
two, and they are deliberately different interfaces (ISP):

- **`IAtmosphereField`** — publishes GPU resource handles. The renderer binds them; no
  copy, no latency, no CPU involvement. This is what §1.1's defect required and never had.
- **`IAtmosphereQuery`** — answers point and profile questions on the CPU, served from an
  **asynchronous readback mirror**: a coarse subset of the fields (§9) copied back every
  few frames into a ring buffer, plus the full-resolution column under each player. Two
  to three frames of staleness is invisible in a medium whose own time scale is minutes.

Nothing reads the authoritative GPU state synchronously; nothing writes it except the
tier steps. That is the whole concurrency story.

### 3.3 Where it runs

**Vulkan compute**, in the render domain's queue family, for T2/T3. The rationale is
narrow and worth stating because the alternative is the house default:

- Every consumer of the heavy fields is the renderer, every frame. Running them in SYCL
  would require exporting external memory into Vulkan for the one path that matters, and
  SushiRuntime's deployment posture (JIT-first, one vendor per build, shared-USM default)
  makes that interop the most fragile part of the stack for zero gain here.
- The gameplay side needs a *coarse, stale* mirror, which is a readback — cheap and
  already a solved pattern in the engine.
**T1 runs on the CPU, and this section used to say it would not.** The escape clause was here
from the start — *T1 is small enough (131 k cells) that if a future need arises to run it
CPU-side for global gameplay queries, it can move behind `IDynamicalCore` without touching
anything else* — and building it made the clause the answer rather than the fallback. Every
consumer of T1 is host-side: the nest's parent forcing is assembled in host memory by
`AtmosphereForcingBuffer`, and the gameplay question this tier exists to answer — *what is the
weather a thousand kilometres away* — is a CPU query. No render pass reads a T1 field; T3 reads
T2's extinction. Putting the tier on the device would have bought a step that is already nearly
free and paid for it with a readback and its latency, and it would have made the whole tier
untestable without a device and non-reproducible without a driver. See §11's C2 for the
measured cost.

Async compute queue where available (`render_pipeline_refactor.md` Phase 11), graphics
queue otherwise; the tier steps are latency-insensitive by construction, which makes them
ideal async-compute candidates.

### 3.4 Determinism posture — stated once, precisely

The atmosphere is **not** deterministic and **not** part of the rollback snapshot. What
*is* guaranteed:

1. **Reproducible from a seed and a schedule, in the absence of GPU nondeterminism.** Same
   seed, same time scale, same T0 data → statistically identical weather, and in practice
   bit-identical on the same driver and device. Not a contract.
2. **The gameplay summary is a versioned, snapshottable input.** §9's summary is captured
   at a fixed cadence, serialized with the scene, and — in a networked session —
   distributed by the authority as a timestamped input like any other command. Clients do
   not simulate the atmosphere for gameplay purposes; they render their own copy and
   consume the authority's summary for anything that affects the world.
3. **Replays reproduce the summary exactly** (it is recorded), and the visual atmosphere
   approximately (it is re-simulated from the same seed).

This is the same posture the engine already takes for VFX cosmetic backends, stated at
higher stakes.

### 3.5 Seams

```
IAtmosphereSource          steps state; knows nothing about consumers
 ├─ SimulatedAtmosphere    T0+T1+T2, composed from the policy objects below
 ├─ IngestedAtmosphere     GRIB/METAR-fed (retargeted ingested_weather.hpp + metar_parser.hpp)
 └─ StaticAtmosphere       a fixed authored profile — the manual-authoring mode as a peer

  composed from (all injected, all replaceable — OCP):
    IDynamicalCore         advection + dynamics       (QgCore, AnelasticCore, …)
    IMicrophysics          saturation + precipitation (Kessler + diagnosed ice phase — B3d, …)
    ISurfaceModel          fluxes, roughness, albedo  (BulkSurfaceModel, …)
    IRadiationModel        heating/cooling profile    (GreyRadiation, …)
    AtmosphereParameters   every physical constant, as serialized data

IAtmosphereField           GPU handles → render tier only
IAtmosphereQuery           CPU point/profile queries → gameplay only
IAtmosphereAuthoring       optional capability: place/edit systems, force scenarios (editor only)
```

**The four composed interfaces above are how this was drawn before any of it ran on a GPU, and
two of them never became types.** `ISurfaceModel` and `IRadiationModel` shipped in Phase B3 as
*stages* — `atmosphere_surface.comp` is the surface model and the radiation is its shortwave and
longwave terms — because on the GPU a "model" is a compute shader plus a parameter group, and
there is exactly one of each. An interface with one implementation forever is a stub wearing a
vtable. The substitutability the sketch was reaching for lives in `AtmosphereParameters` instead:
a different planet, a different land cover or a different microphysics tuning is a data edit. See
§11's B3c for the seam that *would* earn a type, and why it is blocked.

Rules that follow, and that the shipped system violated:

- The host owns `std::unique_ptr<IAtmosphereSource>`. **No concrete atmosphere type
  appears in `ISimulation`, `IWorldEditor`, or any other abstract interface.** The editor
  reaches authoring through `IAtmosphereAuthoring`, queried as an optional capability, so
  a source that cannot be authored (ingested) is a legal, installable peer.
- Every physical constant lives in `AtmosphereParameters`, is serialized with the scene,
  and is editable. None is a function-local `constexpr`.
- Solar geometry comes from `Astro::Ephemeris` through an injected seam. One sun.
- The render tier consumes `IAtmosphereField` and never the source; gameplay consumes
  `IAtmosphereQuery` and never a texture.

---

## 4. T0 — Climatology (data)

Static assets, sampled as textures, providing boundary conditions and relaxation targets.
Not simulated.

| Field | Resolution | Source | Used by |
|---|---|---|---|
| Terrain elevation | 1–2 km | engine terrain system (**blocker, §15**) | T2 orography, surface model |
| Land/sea fraction | 1° | Natural Earth 1:50m vector, area-averaged | surface fluxes, albedo, roughness |
| Monthly SST | 1° | NOAA OISST v2 1981–2010 monthly climatology | surface moisture and heat flux |
| Zonal-mean thermal wind / jet profile | 1° × month | NCEP-NCAR R1 monthly LTM, 250/850 hPa | T1's baroclinic mean state |
| Saturated column water | 1° × month | NCEP-NCAR R1 temperature, Tetens integral | T1 moisture relaxation, ITCZ band |
| Soil moisture / vegetation | 5 km | seasonal climatology | latent vs. sensible flux partition |

**Two corrections to this table, both deliberate.**

*ERA5 became NCEP-NCAR Reanalysis 1.* ERA5 is behind the Copernicus CDS, which requires an
account, and an asset nobody else can reproduce is an asset nobody else can check. NCEP R1's
monthly long-term means are on a public HTTP endpoint at NOAA PSL and need no credentials,
so `se climatology bake` downloads and computes rather than trusting a number typed in from
a figure. The cost is resolution — 2.5° against ERA5's 0.25° — which is irrelevant here,
because every field taken from it is a *zonal mean* and is one-dimensional by the time it
reaches the asset.

*"Precipitable water" became "saturated column water".* The core does not relax moisture
toward observed water; it evaporates toward a fraction of a ceiling and condenses above it
(§5), so a field of observed precipitable water fed into that slot would be a ceiling about
30 % too low everywhere and it would rain continuously. The bake integrates saturation
specific humidity over the reanalysis' own temperature profile instead, which is the
ceiling the scheme actually means. Observed precipitable water is still downloaded, and is
still never written: it is divided by the derived saturation to report the implied column
relative humidity, which comes out at 0.58 in the tropics and dips to 0.42 in the
subtropics — the descending branch of the Hadley cell, reproduced by a derivation that was
never told it exists. That is the check that the thermodynamics is right.

For a non-Earth body, T0 degrades to analytic latitude bands driven by `PlanetParams` and
the body's obliquity/rotation rate — the system stays functional, just less specific.

**Why the surface fields are 1° and not 5 km.** The earlier 5 km figure assumed T0's mask
was what told T2 where the shoreline is. It is not, and it cannot be: a nest cell is 2 km
across over a 384 km footprint (§6), so a global raster fine enough to place a coastline
inside one nest cell would be 200 M cells — and the scene's own terrain already carries
that shoreline at metre resolution, from the same heightfield the renderer draws. What T0's
surface fields are actually for is the *global, synoptic* question — what is normally under
this column, and how warm is the water — and 1° (111 km) resolves every continent and every
basin-scale SST pattern. It is also OISST's native grid, so the SST is stored with no
reduction and no interpolation error at all. The one thing genuinely lost is the sharpness
of western-boundary fronts (Gulf Stream, Kuroshio), which matter for a mesoscale
ocean–atmosphere coupling this engine does not model.

The field is a *fraction*, not a boolean, precisely so the 1° cell keeps the subgrid
information: Natural Earth's vector coastline is supersampled and area-averaged, so a cell
that is a third of Anatolia reads 0.33 rather than rounding to land.

None of this is baked into the format. The surface grid is a header field and `adopt()`
validates whatever dimensions arrive, so the day a T2 surface-flux task measures a need for
finer, the bake tool changes a default and no reader changes at all.

### 4.1 What the real mean state did to T1 (measured)

`atmosphere_global_probe --climatology assets/atmosphere/climatology.set0`, 512×256, 60 days,
against the same probe on the analytic bands:

| | analytic bands | baked climatology (January) |
|---|---|---|
| Eddy energy growth | 0.2600 /day | **0.2098 /day** |
| Amplitude e-folding | 7.69 days | 9.53 days |
| Peak vertical shear | 20 m/s at 45° (by construction) | 34 m/s at 29°N |
| Jet latitude reported | 45° | **29–31°N** |
| Zonal KE | ~4.2 × 10⁵ J/m² | ~1.0 × 10⁶ J/m² |
| Eddy KE at saturation | ~1.1 × 10⁵ J/m² | ~2.9 × 10⁵ J/m² |
| Global mean rain | 0.235 mm/day | **1.5 – 2.4 mm/day** |

**No retuning was needed, which was not the expectation.** The prediction going in was that a
mean state with nearly twice the shear would grow much faster and would push
`grid_scale_damping_seconds` off its calibration. It grows *slower* — 0.21 against 0.26 — and the
life cycle is intact: exponential growth through day 38, saturation at day 48, decay, and a second
cycle beginning at day 58. The reason the extra shear does not buy extra growth is that the real
jet is narrow and sits at 29°N rather than broad at 45°, and β is larger there, so the Phillips
threshold it has to clear is higher. The damping parameter is untouched.

The jet latitude is the result worth pointing at: nothing tells the core where to put a jet. It
reports 29–31°N because that is where January's subtropical jet is in the reanalysis it is
relaxing toward.

**Named limit: the moisture cycles too slowly, and the ceiling is not what is wrong.** Global mean
column water settles at 31.3 kg/m² against an observed ~25, while global mean rain reaches
1.5–2.4 mm/day against an observed ~2.7. Those two miss in *opposite* directions, so no value of
`evaporation_relative_humidity` fixes both — a lower ceiling would bring the water down and push
the rain further away. What that pattern indicates is the condensation and evaporation
*timescales*, not the saturation profile. `evaporation_relative_humidity` is therefore left at
0.75 deliberately, rather than moved to the observed 0.55 because a single number happened to
match. (It is worth noting how much closer the rain already is: the analytic mean state produced
0.235 mm/day, an order of magnitude low.)

### 4.2 The season

`ProceduralWeather::tick` used to drop its `julian_date`. It now converts it to a position in
the year and hands it to `QuasiGeostrophicCore::set_year_fraction` before advancing the flow, so
a step always runs against the climatology of the moment it belongs to.

**A season is not a force on the flow.** `set_year_fraction` re-reads exactly two things — the
saturation ceiling and the climatological state — and touches nothing prognostic. Potential
vorticity and column water carry straight through. A cyclone alive in April does not vanish
because the target jet moved; it finds itself in a slightly different mean flow, which is what a
season is.

**Determinism (§3.4) is preserved by quantizing, not by thresholding.** The obvious way to avoid
rebuilding two tables every frame is "rebuild when the date has moved enough", and that is wrong:
it makes the mean state depend on the *call history*, so two hosts ticking at different rates
diverge. Instead the year fraction is quantized to whole days — a pure function of the date. T0
holds twelve monthly fields, so a day already oversamples the data thirty to one and nothing is
lost, while the rebuild happens ~365 times a simulated year rather than 60 times a second.
Measured: a host ticking 1000 times and a host ticking 10 times across the same interval land on
mean states identical to nine decimal places.

**The date is applied before the seed, not after.** A scene opening in July would otherwise be
seeded with January's jet and spend its first simulated weeks migrating — a transient nobody
asked for that would read as the weather being wrong. `ProceduralWeather`'s constructor takes the
epoch for that reason.

Measured through the provider, seeded at each solstice:

| | January | July |
|---|---|---|
| 35°N upper wind | 34.55 m/s | 11.37 m/s |
| 30°S upper wind | 16.45 m/s | 35.97 m/s |
| SH jet peak latitude | 48°S | 29°S |

The last row is the one worth pointing at. Nothing in the engine knows the hemispheres behave
differently, yet the southern jet **moves equatorward into winter** while the northern one simply
weakens. That is the Southern Hemisphere's double-jet structure: one merged eddy-driven jet near
50°S in summer, splitting in winter so the strongest flow is the subtropical jet at 30°S. It
falls out of the data. (It also cost a wrong assertion to find — a check written at 45°S failed,
because 45°S is exactly the crossover between the two jets and says nothing either way.)

Licensing note: NCEP-NCAR R1, NOAA OISST, and Natural Earth (all shipped in the bake) and
ETOPO / MODIS land cover (blocked with §15's terrain system) are all public,
attribution-class data; the bake itself is done (`se climatology bake`, §15) — ERA5 was
dropped for the reproducibility reason given above. The provenance travels *inside* the
asset (a length-prefixed string every reader can show) rather than in a sidecar that can be
separated from the data it describes.

---

## 5. T1 — Global dynamical core

**Formulation.** Two-layer moist quasi-geostrophic flow on a 512×256 lat/lon grid
(0.70°, ~78 km at the equator). Prognostic: potential vorticity `q_k` per layer plus
column moisture.

```
q_k = ∇²ψ_k + f + (−1)^k · (f₀² / g′H) · (ψ₁ − ψ₂)          k = 1 (upper), 2 (lower)

∂q_k/∂t + J(ψ_k, q_k) = −(q_k − q_k^clim)/τ_relax + S_k − D_k
∂W/∂t  + ∇·(u W)      = E − P
```

- `ψ` is recovered from `q` each step by inverting the elliptic operator: FFT in
  longitude, tridiagonal solve in latitude, 2×2 layer coupling — the standard QG
  inversion, and the reason this core is cheap.
- Wind is `u = k̂ × ∇ψ`, geostrophic by construction — no `WIND_SCALE` fudge factor.
- **Baroclinic instability is the point.** Relaxing toward T0's climatological
  thermal-wind mean state keeps the flow unstable in the mid-latitudes; cyclones,
  frontogenesis, jet meanders, and storm tracks then *emerge*. Nothing places a low.
- `W` is column precipitable water, advected with a monotone semi-Lagrangian scheme;
  large-scale condensation plus a Betts–Miller-style convective relaxation gives global
  precipitation and, with T0's SST forcing, an ITCZ band.
- Poles: a Fourier filter on the high zonal wavenumbers near the poles (the standard
  lat/lon fix). A cubed-sphere core is a legal future `IDynamicalCore`, not a Phase C
  requirement.
- Time step: RK3, Δt ≈ 6 min game time. CFL at 78 km with a 60 m/s jet permits ~20 min;
  6 min is comfortable and keeps moisture transport accurate.

**Tropical honesty.** A 2-layer QG model does not produce a true Hadley circulation, and
tropical cyclogenesis is not emergent from it. The ITCZ is a T0-forced convergence band
modulated by SST anomalies, and tropical cyclones — if wanted — are seeded features
advected by the core's own steering flow, not spontaneous. This is a named limit (§14),
not a gap to be discovered later.

**`SynopticLayer` does not survive as a simulation.** Its editor value does: "place a low
here" becomes an injected vorticity anomaly in `q`, which then evolves under the real
dynamics instead of translating as a rigid Gaussian. This is strictly better authoring
and is how §10's map overlay stays useful.

---

## 6. T2 — Regional nest

**Formulation.** Anelastic non-hydrostatic, sound-filtered, on a camera-following domain.

```
∂u/∂t = −(u·∇)u − c_p θ̄_v ∇π′ + B k̂ + f k̂×u + ∇·(K ∇u)
∇·(ρ̄ u) = 0                                    → elliptic solve for π′
∂θ/∂t = −(u·∇)θ + L/(c_p Π) · (condensation) + Q_rad + Q_surface
∂q_x/∂t = −(u·∇)q_x + microphysics(q_v, q_c, q_r; ice diagnosed by phase — B3d) − ∂(V_x q_x)/∂z
B = g · (θ_v′/θ̄_v − q_c − q_r)                 buoyancy including condensate loading
```

**Discretization.**

**Shipped, and resolved by the render quality tier** (`QualityParams::atmosphere_nest`), so
the nest is discretized by the same resolver every other pass reads. Step costs are *measured*
on a GTX 1060 6 GB, not scaled from a model; VRAM is scaled from the measured High.

| | Low | Medium | **High** | Ultra |
|---|---|---|---|---|
| Horizontal | 96×96 @ 4 km | 128×128 @ 3 km | **192×192 @ 2 km** | 256×256 @ 1.5 km |
| Vertical | 32 levels | 40 levels | **48 levels** | 64 levels, all 0–18 km |
| Cells | 0.29 M | 0.65 M | **1.77 M** | 4.19 M |
| Δz surface → aloft | ~90 → 900 m | ~71 → 700 m | **~54 → 560 m** | ~44 → 420 m |
| Step cost | 1.65 ms | 3.63 ms | **8.37 ms** | 19.5 ms |
| VRAM | ~22 MB | ~50 MB | **~135 MB** | ~320 MB |

**The horizontal domain is 384 km at every tier and only the resolution changes**, so raising
the tier resolves the same weather more finely rather than simulating a different amount of
world — which is what keeps a scene's sky recognisably itself across machines, and what stops
two players in one scene from getting different weather. Verified: across all four tiers a
quiescent airmass ends six hours at 48.6–49.8 % surface humidity and 12.73–12.75 kg/m² of column
water, and a convecting one at 100 % cloudy columns with 0.49–0.62 coverage.

High is the baseline the resolver's contract requires and is unchanged from what shipped; 2 km
is not an arbitrary rung on that ladder but the spacing at which convection stops being
parameterized and starts being *resolved* (§2.2), which is what this phase's acceptance bar — a
cumulus that grows on its own — rests on. Medium and Low sit above it and their convection is
correspondingly smoother and more parameterized.

**Named limit: the subgrid cloud closure is not rescaled with the tier — and the scaling that was
supposed to fix that was built, measured, and rejected.** `cloud_critical_humidity` sets where a
cell's humidity distribution begins to hold cloud, and its 0.80 is the standard value *at 2 km*:
the subgrid variance a cell hides is a function of how big the cell is, so in isolation the
correct value falls at coarser spacing and approaches 1 as the grid resolves the cloud itself.

That argument is sound and the scaling it implies is not a guess — the spread is the standard
deviation of a field filtered at the cell scale, and an inertial-range spectrum fixes its exponent
at 1/3 rather than leaving it to calibration, giving 0.748 / 0.771 / 0.800 / 0.818 across the four
tiers. Built and measured end to end, **it makes tier agreement worse**, so it is not shipped.
Marginal sky, coverage at the moment the closure is the deciding term:

| | Low | Medium | High | Ultra |
|---|---|---|---|---|
| one authored value everywhere | 0.089 | 0.011 | 0.014 | 0.010 |
| scaled with spacing | 0.147 | 0.039 | 0.014 | 0.001 |

At onset the single authored value already agrees to within 5 % (Low 0.089 against High 0.085) and
the scaling breaks that agreement. What is left of the disagreement is *later* in the day and is
resolved convection rather than the closure — the coarse grid's deck is thinner, and no critical
humidity addresses that.

The symptom this limit was originally written from — High 5.8 % cloudy against every other tier
under 0.1 % — is **no longer reproducible**, because it was a property of the too-dry base state
the vapour profile fix removed, not of the closure. Medium, High and Ultra now agree to
0.010–0.014; Low is the outlier, at the spacing where this section already says convection is
smoother and more parameterized.

Every prognostic field is fp32, including the `q_*` fields. Half floats were the
original choice for them on a range argument — mixing ratios are a few grams per
kilogram and nowhere near the format's ceiling — and §11's Phase B2c measures that
argument to be the wrong one: what decides a format here is the ratio of a step's
tendency to the value it lands on, and the surface latent flux's increment was a third
of one unit in the last place, so the boundary layer never moistened at all. The VRAM
column above is scaled from the measured fp32 High; the shipped 192×192×48 nest is
~135 MB. Double buffering only for the fields the advection step needs both
states of.

**Advection** is monotone semi-Lagrangian at Courant ≈ 1 (MacCormack with a limiter, or
BFECC), directly replacing §1.4's diffusive scheme. **Pressure**, as shipped (§11's B2),
is a vertical line solver rather than the FFT-based solve this section originally
specified: the anisotropic grid makes the Laplacian's vertical coupling dominate by
`(Δx/Δz)²`, so the vertical is solved exactly per column by a Thomas sweep and the
horizontal iterated red-black, defaulting to four sweeps — twice the measured convergence
point (B2c). This is the dominant cost of the step.

**Microphysics** (`IMicrophysics`, default Kessler warm rain with a *diagnosed* ice
phase — §11's B3d: a temperature partition, not prognostic `q_i`/`q_s` species):

```
e_s(T) = 611.2 · exp(17.67 (T − 273.15) / (T − 29.65))      Pa      (Magnus/Teten)
q_s    = 0.622 e_s / (p − 0.378 e_s)
saturation adjustment: condense (q_v − q_s) with latent heating into θ
autoconversion:  A  = k₁ · max(q_c − q_c0, 0)
accretion:       C  = k₂ · q_c · q_r^0.875
rain evaporation below cloud base; terminal fall speed V_t = 36.34 (ρ q_r)^0.1364
ice: a phase fraction ramping 0 → 1 over 0…−20 °C blends the saturation curve, latent
     heat, autoconversion threshold, fall speed and effective radius (B3d)
```

Every one of `k₁`, `q_c0`, `k₂`, the exponents, and the phase-band limits is a field of
`AtmosphereParameters` — data, editable, serialized. Not a `constexpr` inside a loop.

**Surface and boundary layer** (`ISurfaceModel`): bulk aerodynamic sensible and latent
heat fluxes from T0's surface type, soil moisture, and SST, driven by `Astro::Ephemeris`
insolation with a slab surface heat capacity. This is what produces the diurnal cycle —
morning clear, midday cumulus, evening decay — and sea breezes and valley fog, all
emergent rather than authored.

**Orography** enters through the terrain-following lower boundary and is the mechanism
for upslope condensation, föhn drying, and orographic enhancement. It requires the
terrain height field the engine does not have (§15).

**Nesting.** One-way. T1 supplies the lateral boundaries through a Davies relaxation zone
~8 cells wide (fields nudged toward the interpolated T1 solution with a weight ramping
from 1 to 0 inward), which is what prevents boundary reflection. T2 recentres on the
player using the same absolute-lattice snap discipline the shipped grid already uses
(`regional_weather_grid.hpp:357`) — but as a GPU shift, with newly exposed cells filled
from T1 rather than from a background climatology guess.

**Beyond the nest.** Outside the 384 km domain, the atmosphere is T1's resolution. §7.5
describes what the sky does there.

---

## 7. T3 — Cloudscape compile (the spatial coupling)

This tier is where §1.1 is fixed. It converts simulated condensate into the exact
resources the shipped render tier already consumes.

### 7.1 Extinction, from physics

Per field texel, from T2's `q_c`, `q_i`, and density:

```
LWC   = ρ · q_c                          [kg/m³]
σ_ext = 3 · LWC / (2 · ρ_w · r_eff)      [1/m]      r_eff from AtmosphereParameters
                                                     (~6 µm maritime, ~10 µm continental)
```

plus an ice term with its own effective radius. `σ_ext` is what the march integrates —
the same Beer–Lambert path W2 already implements, now fed by a physical quantity instead
of a remapped noise value.

### 7.2 The field stops wrapping

The shipped field is a **periodic 65 km tile** in XZ (`cloudscape_field.comp:10-18`),
which is only coherent because the weather above it is globally uniform. With spatial
weather it must become **camera-centred and non-wrapping**: 256×256×64 over the near
region (~64–128 km), rebaked when the camera crosses a rebake threshold or when T2 steps,
whichever is sooner, amortized across frames exactly as the current bake already is.

This is the one structurally significant change to the render tier, and it is the price
of the feature. **Shipped in Phase B1**, as two windows rather than one — a near window at
32 768 m / 128 m per texel and a far window at 262 144 m / ~1 km, cross-faded — because the
march reaches ~150 km and a single window with that reach would have coarsened the near field
by five times.

### 7.3 Noise becomes detail, not signal

```glsl
// shipped:
float coverage = clamp(deck.coverage_bias + (weather_noise - 0.5) * 1.35, 0.0, 1.0);

// target:
float sigma = simulated_extinction(world_pos);                    // unique, planet-scale
sigma *= 1.0 + (detail_noise - 0.5) * DETAIL_AMPLITUDE;           // ~0.35, sub-cell only
sigma  = erode_and_curl_warp(sigma, world_pos);                   // unchanged from W2
```

Anti-repetition then follows structurally rather than by trickery: the signal is a unique,
non-periodic simulated field over hundreds of kilometres; tiled noise only adds
sub-2 km structure that the simulation genuinely cannot resolve, at reduced amplitude, at
the existing incommensurate scale ladder (65 536 / 8 192 / 811 m + curl). The visible
repetition ends because the repeating thing is no longer the thing you are looking at.

### 7.4 Genus becomes a label, not an input

The `Cloudscape`/`CloudDeck`/genus stack was an *input* to the density field. It cannot
remain one: cloud shape now comes from where the condensate actually is, and a single
column may contain a stratus deck, a cumulus field, and a cirrus sheet at once, with real
bases and tops.

Genus is instead **derived** from the simulated profile — layer base/top, depth,
condensate phase, vertical velocity variance, convective character — and used for: the
editor's readout, METAR-style reporting, audio/gameplay classification, and the
authoring/ingested paths that still need to *specify* a sky. `cloud_genus_profile`
becomes the classifier's reference table plus the `StaticAtmosphere` generator. Nothing
about the WMO vocabulary is lost; it stops being load-bearing for rendering.

`WeatherCloudscapeCompiler` is **kept rather than deleted** (Phase B1 revises §16's
disposition): the genus *choice* moved out of it into `Render::classify_cloud_genus`, which
the GPU bake and the label path now share, but the label and the medium — the scattering
knobs, the erosion scale, `evolution_rate` — still need a producer, and this is a truthful
one. What it stopped being is the sole answer to "what is in the sky".

### 7.5 The far field

- Inside the nest (≤384 km): T2's own condensate.
- Beyond it: T1's column moisture and layer thicknesses expand into a coarse, smooth
  extinction field — enough for a correct horizon and the panorama impostor, which is
  where those pixels end up anyway.
- The existing panorama pass, cloud shadow map, light volume, and skip field all consume
  the new field with no interface change.

### 7.6 CloudsV2 — the carve leaves the bake (2026-08-01)

The user rejected the rendered result outright ("this is never a real cumulus… worth
rewriting from scratch"), and the diagnosis agreed with the verdict even though the
pipeline's skeleton did not deserve the blame: **the shape of a cloud was stored in
textures that cannot hold a shape.** The near window carves into 128 m texels, the far
window into 1024 m ones; a cauliflower edge lives at 10–50 m. Whatever the carve produced,
trilinear reconstruction shipped its smeared average to the march — and the march's own
procedural detail was gated to within *200 metres* of the camera, so effectively the whole
sky was bake-lattice mush. That is a structural property, not a tuning error, and no
amount of constant-twiddling inside the old split could fix it.

So the split moved. This is the War Thunder / Nubis arrangement proper:

- **The bake stores the envelope, not the shape.** `cloudscape_field.comp` now writes
  r = coverage envelope (the nest's diagnosed cloud fraction, top-openness-tapered; or the
  deck stack's height-gradient-tapered coverage), g = vertical profile (unchanged),
  a = in-cloud water amplitude at half scale. All noise taps are gone from the bake; the
  deck paths keep only their weather-map modulation, streets and anvil. Multi-deck
  combination became a probabilistic union of coverages instead of a density sum.
- **The march carves, at every distance.** `cloud.frag`'s `cloud_density_carved` runs the
  full Nubis recipe per sample — domain warp, CDF-uniformised base threshold at
  `1 − envelope`, height-flipped Worley erosion (wispy base, billowy top), plus a fine
  incommensurate octave and the curl warp near the camera — in the same world-anchored,
  wind-advected pattern frame the bake evaluates weather in (`cloud_field_pattern`, newly
  published through the scene tail). The carve scale is coverage-adaptive
  (1/√envelope — count falls with coverage, not size) and floored at 4× the march's local
  step, which is the bake's old Nyquist floor restated against the true sampler. Past
  80 km the carve hands off, faded, to the statistical mean `envelope · water · 0.45`.
- **One new noise volume instead of new bindings.** The march had exactly one free image
  slot, so `cloud_noise_volume.comp` kind 4 precombines everything the carve needs into
  one 128³ RGBA8: uniformised base (the CDF transform runs at generation now), combined
  erosion fbm, fine fbm, curl potential. Binding 4 swaps the old detail volume for it.
- **Every sun-depth integral states mass against the carved sky.** The light volume, the
  far light channel, the cloud shadow map and the panorama impostor all march the
  envelope×water product scaled by `CLOUD_ENVELOPE_MEAN_SHAPE = 0.45` (the threshold's
  expected yield env/2, minus the erosion's bite), one shared constant in
  cloud_field_window.glsl; the inline cone march applies the same factor. The skip field
  pools the *unscaled* product, because a probe must stay a ceiling.
- **The far bake's 64-tap supersampling dropped to 4.** It existed to anti-alias a
  threshold; there is no threshold in the bake any more. 16× less far-bake work.

What stays: the pass topology (half-res march → cloud TAA → composite, both just fixed),
the budgets and step rule, the dual-lobe multi-octave sun energy, the two-window
addressing, and the meteorology as the sole author of *where* clouds are.

*(Superseded 2026-08-01 by the fifth entry: the whiteout and the profile-gradient ambient
were both listed here as staying, and both are defects — see §11. The step rule and the
budgets are also on that entry's list.)* Known deliberate gaps, stated rather than hidden: cirriform
decks currently carve with the cumuliform recipe (streak styling is a follow-up), and
beyond the far window's rim there is still no cloud — §7.5's planet-scale far field
remains future work.

---

## 8. T4 — Render tier: exactly what changes

Everything W0–W3 shipped stays. The changes are:

1. **`cloudscape_field.comp`** — density source becomes T3's extinction field; the
   six-deck genus loop is deleted (it moves to §7.4's classifier, in the other direction).
2. **The field becomes camera-centred and non-wrapping** (§7.2), with a rebake cadence
   driven by camera motion and T2 steps. `CloudscapeCompilePass`'s change-detection
   snapshot (`cloudscape_compile_pass.cpp:287`) is replaced by that cadence.
3. **Aerial perspective, fog, and turbidity** read T2's humidity and precipitation fields
   spatially instead of `WeatherCoupling`'s global scalars — the froxel fog volume can
   sample the simulated field directly, which is what makes a rain shaft visible as a
   local darkening rather than a global one.
4. **Precipitation VFX** are placed under the cells that are raining, from T3's surface
   precipitation field, replacing the camera-attached emitter
   (`runtime_simulation.cpp:2776`). This finally meets the W5 acceptance bar.
5. **Lightning** is driven by T2's graupel/updraft product (the standard proxy for
   charge separation), injected into the light volume as W2 already provides for.

Unchanged: the march and its step rule, the light volume, the shadow map, cloud TAA, the
near/far split, the composite, the panorama, the quality tiers, the profiler wiring.

---

## 9. World coupling and the gameplay data plane

### 9.1 `AtmosphereProfile` — the replacement contract

`WeatherColumn`'s 15 floats (§1.2) are replaced by a real profile plus derived diagnostics:

```
AtmosphereProfile                       // N levels, N ≈ 24 for the query mirror
  altitude_m, pressure_pa, temperature_k, dewpoint_k
  wind_east_mps, wind_north_mps, wind_up_mps
  liquid_water, ice_water, rain, snow
  turbulence_intensity, icing_rate

AtmosphereDiagnostics                   // derived once, cached
  cloud_layers[]     base_m, top_m, coverage, genus_label, phase
  surface            visibility_m, precipitation_rate, precipitation_type, wind + gust
  freezing_level_m, cape_j_per_kg, cin_j_per_kg, wet_bulb_zero_m, tropopause_m
```

Everything the shipped code had to scope down or fake — snow vs. rain, icing, real cloud
bases, visibility, a wind profile — is a direct read from this.

### 9.2 The query mirror

A coarse subset of T2 (64×64×16, fp16, a handful of fields ≈ 2 MB) is copied back
asynchronously every ~4 frames into a triple-buffered ring, together with the
full-resolution column beneath each player. `IAtmosphereQuery` serves point queries,
profiles, and diagnostics from the most recently completed readback — 2–3 frames stale,
which for a medium with a minutes-long time scale is not observable. (This is Phase E's
target shape; the interim mirror Phase B2 pulled forward is a 32×32 `WeatherColumn`
lattice, plus the observer column's full `AtmosphereProfileLevel` profile added in B2c.)

Existing consumers keep their API shape and change only their source:
`weather_wind.hpp`'s `weather_wind()` / `wind_gust()`, `weather_flight_hazards.hpp`'s
`icing_risk()` / `turbulence_intensity()`, `weather_world_coupling.hpp`'s fog/wetness/
turbidity signals, plus audio (rain and wind beds placed by where it is actually raining)
and VFX.

### 9.3 The deterministic summary

What crosses into SushiLoop's deterministic world is not the mirror but a small,
versioned **summary**: surface wind, visibility, precipitation rate and type, temperature,
and cloud base, over a coarse tile set, captured at a fixed cadence. It is serialized with
the scene, recorded into replays, and — in a networked session — authored by the server
and distributed as a timestamped input. Gameplay-visible weather is therefore identical
across clients even though each client's rendered atmosphere is its own simulation.

---

## 10. Editor integration

**Shipped: the Meteorology panel** (`Window ▸ Meteorology`), the first slice of the list below.
It is deliberately a *tuning and logging* surface rather than a visualiser, because the questions
this tier raises are numerical: is the sky animating faster than the nest can step, what is the
solar forcing right now, what does the observer's column actually contain, and — when it contains
nothing — which link in the chain from "procedural weather" to "readback complete" is broken. It
names that link and offers the fix beside it, rather than reporting a symptom.

Its CSV log is sampled on the **nest's own clock**, not the wall clock, so a line is a fixed
interval of simulated weather however fast the sky is being animated, and two runs at different
time scales are comparable. Phase B2c's entire diagnosis came out of that file.

- **Atmosphere panel** replacing the Weather panel: global map overlay (T1 pressure/
  vorticity/precipitable water, with fronts diagnosed from the thermal gradient rather
  than drawn from a ray pair), the nest's footprint, and a time-scrub coupled to the
  ephemeris clock.
- **Authoring through `IAtmosphereAuthoring`**: inject a vorticity anomaly ("place a low"),
  set the mean-state strength (a stormier or calmer planet), force a scenario, jump the
  clock forward to let a system develop, freeze/step the tiers.
- **Skew-T / hodograph readout** of `AtmosphereProfile` at the camera — the single most
  useful debug view for a system like this, and the one that makes "is the physics right"
  answerable at a glance.
- **Debug views**: T1 fields, T2 slices (θ, q_v, q_c, w), the extinction field, the skip
  field, the light volume, march-step heatmap, the query mirror's staleness.
- **Presets** seed initial conditions (mean-state strength, moisture, a placed anomaly),
  not deck parameters. `StaticAtmosphere` remains for screenshots and tests.

---

## 11. Phased roadmap

Each phase ships editor surface, tier wiring, profiler budget, and CHANGELOG/ARCHITECTURE
entries per repo policy.

### HANDOVER — 2026-08-02. Read this before touching the cloud stack.

This section is the state of play, written for whoever picks the work up next. The entries
below it are the chronological record of how each defect was found; this is the summary, the
priority order, and — as importantly — **the list of things that are already settled and must
not be re-litigated.**

#### How this work is being judged

The user looks at the screen. That is the acceptance test, and four rounds of confident
reasoning-from-source were wrong about what was actually on it. **Ask for a screenshot before
forming a hypothesis about appearance, and ask what the tier and the camera altitude are** —
two whole rounds were spent explaining artefacts that turned out to be the Low tier's
one-third-resolution cloud buffer and an orbital camera the T3 window system is documented as
not covering.

Working constraints:
* **Do not build.** The user's machine runs `se build` / `se editor`; write and verify by
  reading. Never invoke cmake or ninja directly.
* **Another agent edits this tree.** Never `git add -A`; stage explicit paths.
* Physically-derived numbers are worth computing before proposing a fix. Several rounds were
  settled by arithmetic against measured lux ratios rather than by opinion.

#### What is fixed (all in the working tree, all unverified by eye except where noted)

| | Defect | Root cause |
|---|---|---|
| CV1 | Deep cloud interiors decayed to black, then a post-march "whiteout" flattened everything | Every octave in `cloud_sun_energy` carried the same Beer term. Replaced with a saturating multiple-scatter floor that rises with the sample's *own* sun depth |
| CV2 | Erosion deleted ~42 % of every cloud, leaving filaments | Flat-amplitude subtraction against a CDF-uniform ramp with no plateau; height flip keyed to the 800–12 000 m *union* shell so cumulus never left the base regime |
| CV4 | One sample per cloud; a 1.3 km unrendered block in front of the camera; skips crossing untested sky | `seg_min = max(shell_thick * 0.12, 40)` — 12 % of the union of every enabled deck, 1344 m — used simultaneously as the integration step, the clamp floor and the jitter span |
| CV10 | No cauliflower at any erosion strength | **Sign error.** The carve built clouds by *subtraction*, which can only produce concave features. The billow ladder is now *added before the threshold* so it displaces the isosurface outward |
| CV11 | Snow-white deck against a black evening sky | `cloud.frag` never sampled the transmittance LUT and had no horizon gate, so the sun that lit a cloud was identical at midnight and noon |
| CV12 | After CV11, clouds went perfectly black at night | Nothing replaced the sun: no skylight, and the Moon — which the ephemeris already derives as a real directional light — never reached the cloud pass |
| CV13 | A one-pixel white line at the horizon | The depth-aware upsample demotes a mismatched tap to 0.05; when *all four* miss, `/ wsum` renormalises them back into plain bilinear and the rejection cancels itself out |
| CV3 | *"berbat LOD, bulutları direkt beyaz render ediyor"* | The carve band-limited the **feature size** instead of the sampling: `carve_scale = max(carve_scale, footprint * 4)` made a distant cumulus 3–5 km wide at full solidity. Under it, every tap was an implicit-LOD `texture()` on a `mipLevels = 1` volume through a `max_lod = 0` sampler. Fixed by a real mip chain plus a threshold integrated over the detail the filter removes — see the eighth entry |
| WM-SEED | A planet was uniformly overcast; no place could be clear while another was stormy | "Manual mode" was *defined* as having no `IWeatherProvider`, so nothing published a `WeatherField` and one authored deck stack covered the body. Both modes install a provider now; Manual places weather from a seed (`SeededWeather` + `Atmosphere::SynopticField`) — see the eleventh entry |
| CV15 | Lit clouds went black when the *camera* entered shadow; night-side clouds vanished entirely | The solar zenith cosine, the horizon gate and the Moon's gate were all taken once per pixel in the **camera's** radial frame. The code's own bound — "under a degree, finer than the LUT resolves" — was invalidated by PL1 and was never valid anyway, because the gate is a step function. Per-sample now, with a solar-disc penumbra — see the tenth entry |
| — | Auto-exposure could not reach a physically lit night | `min_ev` is a *maximum-exposure* control despite the name, and the editor slider stopped at −10 (184×) when moonlight needs ~1000× |
| — | `sky`/`clouds` vanished from the profiler at Ultra | `MAX_TIMED_PASSES` was 16 against ~44 passes, and its truncated sum drives auto-exposure adaptation and dynamic resolution |

#### Do not re-litigate these — they were investigated and settled

* **The editor grid does not bound the cloud march.** `GridPass` is a fullscreen pass that
  writes no depth, ray-casts the reference ellipsoid rather than a flat plane, and runs *after*
  the clouds. This was the leading hypothesis for the horizon line and it is refuted.
* **`CARVE_END_METERS`'s hand-off to `mean_density` is mass-conserving** and is *not* what made
  the horizon band opaque. `CLOUD_ENVELOPE_MEAN_SHAPE` is the carve's own measured yield, so
  expected optical depth per metre is unchanged either side of 80 km. It removes variance, not
  extinction. What makes a grazing ray opaque is that past `DETAIL_FADE_END_METERS` the carve
  runs on features floored at `footprint * 4` = 3–5 km, where one blob crossing is ~15 optical
  depths.
* **`MS_FLOOR` is multiplicative in `sun_radiance`** — it is added to `energy`, and `energy` is
  multiplied by the beam — so it cannot brighten a cloud independently of how dim the sun is.
  It was not the cause of the white deck. Its 0.35 against a ~1.08 lit edge does compress
  interior contrast to ~3:1, so 0.15–0.20 is worth revisiting *after* the exposure is right,
  but not before.
* **The engine's night is physically correct, not broken.** Starlight is derived at 8.3e-9 of
  sunlight and full moonlight at ~2.4e-6; both are within ~20 % of measured lux ratios. A
  moonless night really is black. Making night *visible* is an exposure-adaptation decision,
  not a cloud-shader one.
* **`cloud_panorama.comp` has the same missing-transmittance bug and it is deliberately not
  fixed.** The pass has no consumer today: `view()`/`sampler()` are never called and the
  `IblPass` reflection-probe wiring is explicitly scoped out. Fix it *when that consumer lands*,
  or the seam against the corrected primary march will show.
* **Direct sunlight is un-attenuated for *every* surface, not only clouds** (`sky.frag`'s ground
  and `pbr.frag` both read the raw uniform). Clouds were merely the most visible consumer. If
  the ground and meshes are later given the same treatment, do it by attenuating at each
  consumer — **not** by tinting `environment.sun.color` on the CPU, which would double-count
  against the sky, whose LUTs already integrate the transmittance themselves.

#### Open, in the order they should be done

1. ~~**CV3 — far-field LOD that keeps its shape.**~~ **Written 2026-08-02, unverified by eye.**
   Both halves: the mip chain and the `textureLod` selection for (a), and for (b) a threshold
   integrated over the removed detail rather than a hard cut against a filtered mean — which is
   what stops the far field from collapsing into an all-or-nothing decision. See the eighth
   entry for what shipped and for the one judgement call inside it (which footprint the LOD is
   selected against) that a screenshot could still overturn.
2. **CV5 + CV9 together — the generation model.** The user has said twice that this is the real
   problem and they are right. One noise scale (2400 m) decides where every cloud is, so there
   is no mesoscale organisation — no clusters, no shear-aligned streets, no clear lanes — and
   `water` is per *column*, so every cloud sharing a column has identical density with no
   interior structure. CV5 is the per-genus vertical envelope (a real cumulonimbus trunk and
   anvil); CV9 is the distribution and the fill. They meet in the same code and should land
   together.
3. ~~**PL1 — planet scale.**~~ **Written 2026-08-02, unverified by eye** — §7.5's coarse
   planet-scale field is in, built from the deck stack and a globe-addressed pattern, and the
   windows now fade into it instead of into nothing. See the ninth entry. The panorama impostor
   is still unwired, and this remains the *cheap* form of the tier: it carries no nest and no
   T1 structure, because neither reaches the GPU beyond 384 km.
4. ~~**WM-SEED — Manual mode becomes a seed, not a preset.**~~ **Written 2026-08-02, unverified
   by eye.** See the eleventh entry for what shipped, the two-evaluator structure, and the four
   named limits — of which the live one is that Procedural mode publishes no centres, so from
   orbit it shows the zonal climatology alone until T1's own pressure extrema are extracted.
   The original statement of the item follows.

   Promoted here by the user on
   2026-08-02 against a side-by-side of our globe and Google Earth (`image5`/`image6`): ours is a
   uniformly milky sphere, theirs is mostly *clear* ocean with discrete bands and swirls over it.
   *"manual mod artık fair weather overcast ile değil seed ile belirlenecek. bu şekilde dünyanın
   bir yeri bulutlu fırtınalı iken bir yeri tam olarak açık hava olacak."*

   Today Manual mode is not a mode at all: `RuntimeSimulation::procedural_weather_enabled()` is
   literally `static_cast<bool>(weather_provider_)`, so Manual means **no provider**, and the bake
   applies one authored deck stack to the entire planet. `StaticWeather` exists in
   `weather_provider.hpp` for exactly this job and is never installed, so it is dead code.
   Two consequences that shape the work:
   * The mode must become an explicit enum before a Manual provider can exist, because installing
     one would otherwise flip the predicate every consumer keys off.
   * A varying field fixes the *near* view but not the orbital one. `cloud_globe_envelope` reads
     the deck stack and a hard-coded `GLOBE_PATTERN_*` modulation — no seed, one scale (~300 km),
     and it modulates a coverage that is already high everywhere, so it can never produce the
     clear ocean that is most of `image6`. The globe field needs the seeded synoptic field too,
     and it must be able to reach **zero**.

   This overlaps CV5+CV9 above and they should agree about what a cloud system is: that item is
   organisation *within* a scene, this one is organisation *across the globe*.
5. **CV14 — the limb speckle.** *"LOD sistemi resmen göz kanatıyor."* Bright unstable fireflies
   along the terminator limb in `image5`. Partly predicted to fall out of CV15 (tenth entry); if
   not, the suspects in order are the step schedule on rays that graze a 2 km shell for hundreds
   of kilometres, the equal-volume footprint being the wrong band limit for a tangential path,
   and `GLOBE_PATTERN_LOD` being a hard-coded 2.0 with nothing tying it to the pixel.
6. **CV6, CV7, CV8** — physical extinction end to end; advection on the game clock at the
   simulated wind; lighting coupled to the carved density.
7. **The rest of the UX and mode work (MU1, MU2, WM1–WM3) and fog (FG1–FG3)**, then aerodynamics
   (AE1–AE5). On fog, as of 2026-08-02 it is **not** bound to meteorology: `FogParams` is authored
   end to end and meteorology contributes exactly one scalar, `WeatherCoupling::fog_density_bias`,
   added to the author's density in `VolumetricFogPass` and zero whenever no provider is
   installed. It also has no horizontal field at all — the froxel volume is camera-frustum
   aligned, so "foggy in the valley, clear on the ridge" is reachable today only through
   hand-placed `FogVolume` primitives. FG1–FG3 is where that changes.

#### Decisions the user has already made — treat as settled

* **Two weather modes.** *Basic* is a **faithful GTA V port**: named global weather states, one
  at a time, blended over 20–60 s, uniform field, no nest, no QG core. *Procedural* is the
  direct simulation. Each gets quality levels.
* **The Environment window stays, but every weather control moves out of it** into Meteorology.
  Environment keeps GI, surface albedo/ocean, stars, night lighting and the Solar System block.
* **Fog binds fully to meteorology** — it stops being authored.
* **Aerodynamics scope is all three**: environmental wind coupling, a flight model (wings, AoA,
  moments), and vehicle aerodynamics (physics P7).
* **Determinism is not negotiable.** The agreed architecture for the air contract is four
  layers: an ISA base density available today; a versioned ~1 Hz snapshot serialised into the
  command stream with an effective tick; analytic feature primitives (fronts, thermals,
  downbursts, shear layers) evaluated identically on both sides; and seeded, tick-indexed
  turbulence. Sampling the nest grid directly is *not* the plan — its 2 km cell cannot resolve
  a 200 m thermal, so the primitives are better as well as deterministic. The only genuine loss
  is physics→atmosphere feedback, which is cosmetic and can be done render-side.

#### Two constants that rest on an unmeasured estimate

`BILLOW_RELIEF` (1.6) and `CREASE_SENSITIVITY` (4.0) in `cloud.frag` are derived from an
inverted-Worley F1 standard deviation taken as ~0.18, which the shader cannot measure. They
move **together**, keeping roughly a 2.5:1 ratio. `BILLOW_MAX_DISPLACE` exists so that a wrong
estimate bounds the turret size instead of swallowing the coverage threshold.

### Where this stands — 2026-08-02, fourteenth entry: the rings were a loop counter

The bisection the thirteenth entry asked for came back clean and immediately useful: with
`Clouds Enabled` cleared the globe is spotless, and with it set the rings are back at the same
viewpoint. **The rings are cloud-path, the terrain is innocent**, and the user had already fixed
what terrain issues there were.

That narrows it enough to reason instead of guess, and the decisive observation is that the rings
are concentric about the **sub-camera point**, not the pole. So they are contours of something
that varies with angle from the nadir — equivalently, with the chord length a ray takes through
the shell. Now go through everything in the march that is a function of distance from the eye,
with the camera in orbit: `detail_fade` (14–42 km), `near_field` (16 km), `CARVE_END_METERS`
(80 km), both step clamps (1 km and 60 km). **Every one of them is saturated** — the shell entry
is a thousand kilometres away, so all of them return the same answer for every pixel on the disc.
None of the pre-existing distance-driven machinery can draw a contour from orbit at all.

One thing could, and it was added by the twelfth entry the same day: the budget coarsening,
`exp2(0.5 * (real_samples - STEPS))`.

`real_samples` is an **integer**. The step size therefore took discrete values, and two adjacent
pixels whose rays landed on different overdraft counts marched at step sizes a factor of √2
apart, which the accumulated density shows. The set of pixels sharing an overdraft count is a
curve of constant chord length through the shell — a circle centred on the sub-camera point.
Rings. And they concentrate toward the limb, because that is where the chord is long enough
(~780 km grazing a 12 km shell) to overrun any tier's budget in the first place, which matches
where they actually appear.

**The general lesson is worth more than the fix: anything derived from a loop counter is
quantised by construction, and a quantised quantity that varies smoothly across the frame is a
visible contour unless something downstream smooths it.** A ray march has nothing downstream. The
same statement covers mip level indices, iteration counts, and early-out depths — a whole family
of "why is there a ring" bugs reduces to it.

So the budget is a *distance* now, not a count. The natural step is geometric —
`t_{n+1} = t_n * (1 + march_angular)` — so the distance at which `STEPS` of them would be spent
is the closed form `t_0 * (1 + march_angular)^STEPS`, computed once before the loop and
continuous in `t_0`. Past it the step scales by how far past it the sample is. Nothing integer
appears anywhere in the step size, so no contour of any shape can form. Growth is quadratic
overall (the natural step is already ∝ t and this multiplies by t again), so the remaining reach
still falls in a few dozen samples. `real_samples` itself is deleted rather than left in place
implying a limit nothing enforces.

Not verified. The reasoning that no *other* distance-driven term can produce a camera-centred
contour from orbit is solid, but "nothing else I found can do this" is weaker than a measurement.

### Where this stands — 2026-08-02, thirteenth entry: the shell was a sphere, the planet was not

The user, looking at the globe: *"wgs84 şeklinden bulut shaderları çok etkileniyor sanki, halka
halka kesiklikler var"*. That reading was correct, and the mechanism is worth writing down
because **the line that caused it is well-written, well-commented, and was right when it was
written.**

`Environment::planet_surface_reference_metres` is `length(observer_center)` — the observer's own
geocentric radius — and its comment explains the choice: put altitude zero at the ground *under
the camera*, because the naive alternative (the equatorial radius) is worth kilometres of air
density at mid latitudes. For a scene that is a few hundred kilometres across, that is not just
defensible, it is the better of the two available answers.

Then PL1 made the cloud field planetary, and nobody went back to the assumption it invalidated.

WGS84's geocentric radius runs 6 356 752 m at the pole to 6 378 137 m at the equator. A sphere
fitted at one latitude is wrong by up to 21 km at another, and every cloud reader subtracted that
sphere from a position to get the altitude a 1 300 m deck is placed against. With the observer at
41° N the shell sits at ~6 368 900 m, so the ground is 9.2 km **above** it at the equator and
12.1 km **below** it at the pole. The deck was buried underground across the tropics, stratospheric
over the caps, and swept smoothly between the two — crossing every boundary in the deck stack on
the way. Each crossing is a circle of constant latitude. Rings.

This is the third time in this document that the same *shape* of failure has appeared: a bound
that was true under the conditions it was derived for, silently carried into conditions that
broke it. The eighth entry's approximation (an error bound on an input said nothing about a step
function's output). The tenth entry's camera-frame lighting ("the angle differs by under a
degree" — true until the march reached the whole globe). Now this one. **The recurring hazard is
not bad reasoning, it is correct reasoning whose preconditions expired**, and the thing that
expired them every time was the same event: the field's reach growing.

#### Why the fix stayed small

Fixing the *altitude* fixes everything downstream on its own:

* The horizon gate takes the **ratio** of a radius to the surface radius, so it is right the
  moment the altitude is. No separate correction.
* The Bruneton LUTs are parameterised by altitude above their own spherical bottom, so handing
  them a true altitude is strictly better than handing them a latitude-dependent error. The
  atmosphere medium therefore **stays spherical deliberately** — that is the parameterisation,
  not an oversight — and only the geometry became oblate.
* The bake, light volume, shadow map and far-light passes are all parameterised in `height01` and
  never convert a position at all, so they needed no change. The entire defect was reader-side.
  That is the layering earning its keep: the bake says "at height fraction h the envelope is E"
  and stays out of the argument about where h is.

`cloud_planet_radius_at` and `cloud_ray_shell` live in `cloud_field_window.glsl` for the reason
everything else in that file does — the view march and the panorama impostor must bound the *same*
shell, or the impostor continues the sky at a different altitude than the march ended it at.

#### Verified partially: "bir nebze düzeldi"

The re-shoot after this landed improved the globe but did **not** clear the ring banding, which
is the most informative result available: one ring family went and another stayed. The altitude
fix is therefore real and not the whole story, and the remaining family has to be found rather
than guessed at — four hypotheses had already been spent on this artifact by that point, which is
three more than the evidence supported.

What turned up while looking is that **the cloud path is not the only thing in this frame that
generates concentric rings**, and the other candidate had not been considered at all:
`terrain.vert:106` computes its CDLOD morph weight as `length(unmorphed)` against a per-node
band, and positions are camera-relative, so that length is the distance from the eye. Morph
bands are therefore **spherical shells about the camera**, which project onto the globe as
circles centred on the sub-camera point — the same shape, from geometry that has nothing to do
with clouds. The user's reading attributed the rings to the cloud shaders and that was reasonable
given what changed recently, but it is not established.

One toggle separates them completely: **Clouds Enabled off, same viewpoint.** Rings that survive
are terrain and the cloud path is innocent. Rings that vanish are in the cloud path, and
*Atmosphere Enabled* off with clouds on then separates a LUT-driven band from a field-driven one.
Until that is run, anything written here about the remaining family would be a fifth guess.

...and asking for that test is how the next bug surfaced: **the checkbox did not work.**
`WeatherCloudscapeCompiler::compile` forced `clouds.enabled = true` and `RuntimeSimulation`
assigns its result back over the environment every tick, so the author's click was overwritten
before the next frame. The override predates WM-SEED, but it only ever ran under `Procedural`,
which is not the mode anyone was in; making both modes install a provider switched it on
everywhere. A regression caused by this document's own eleventh entry, found only because an
unrelated experiment needed the control.

Two things worth keeping from it. First, **the override was level-triggered to answer a
edge-triggered question** — its reasoning ("a scene authored with clouds off would leave
procedural weather invisible") is about the moment a scene loads, but it was enforced every tick,
and per-tick it cannot distinguish an authored state from a decision made a second ago. Second,
it is the second time in two days that a change to *who installs a provider* silently activated
behaviour written for a narrower case; the first was Manual mode having no `WeatherField` at all.
`IWeatherProvider`'s installation is load-bearing in ways its call sites do not advertise.

#### The horizon is a separate problem, and it is not fixed

The same round included `image12`: from low altitude the cloud near the horizon resolves into long
thin radial streaks converging on the vanishing point. That is **not** this bug — at the
observer's own latitude the old sphere was exact, and 138 km of horizon is 1.2° of arc, worth at
most ~440 m of the 21 km error.

The leading hypothesis is the windows' own geometry. They are flat axis-aligned XZ lattices with
128 m horizontal texels and 32 vertical texels over the whole shell (~350 m each). A ray one
degree below horizontal climbs 350 m in about 20 km, so it stays inside a single vertical texel
row for tens of kilometres and returns a value that barely changes along that stretch — a streak.
Adjacent rays fall into adjacent rows, and every such streak converges where the window's plane
vanishes, which is the horizon. The carve, which is what would normally put structure back, is
*correctly* band-limited to nothing out there (the equal-volume footprint at 100 km is ~700 m).

If that is right then nothing is malfunctioning: the field's own resolution is simply showing
through at the one incidence angle that maximises the path length per texel. It belongs with CV14
and CV5/CV9 rather than being a defect with a local fix, and it is tracked separately. It has not
been verified.

### Where this stands — 2026-08-02, twelfth entry: the two edges the budget drew

Two reports, one screenshot each, and they turned out to be the same *kind* of defect in two
different passes: a resource limit implemented as a **domain** limit instead of a **resolution**
limit. Both produced a boundary anchored to the observer rather than to anything in the world,
which is the signature to recognise — an artifact that slides across the frame when the camera
moves is almost never about what is in the world at that place.

#### The march stopped instead of coarsening (CV16)

`cloud.frag`'s loop ran `while (iter < max_iterations && real_samples < STEPS)`. When the sample
budget ran out the ray simply ended, contributing nothing for whatever remained of its length.
The user's words: *"kameraya yakın yerde bulut renderlanmaması hem büyük ölçekte hem küçük
ölçekte hala var"* — and a second shot at 1 500 m showing a cloud mass sliced off along a hard
line, with *"eğer sola doğru gidersem renderlanmayan kısım renderlanmaya başlıyor"*.

That last clause is the diagnosis. A boundary that retreats as you approach it is a boundary
that depends on the eye, and the only eye-dependent quantity in the march is how much budget a
ray has spent by the time it gets somewhere.

Why it hit the near field hardest is the part worth remembering: **the budget is charged per
envelope sample, and the envelope is non-zero across far more sky than the carve ever fills.**
The coarse probe answers "cloud may exist here", so ordinary clear-but-not-provably-empty air is
charged the full price of a carve evaluation and returns nothing. Combined with the 20 m near
step — which exists for good reasons and costs 25 samples over the first 500 m — the cheap
tier's 48 samples were gone by about 1.2 km. Someone standing under a deck could see the horizon
and not the air in front of their face.

The fix is one line of principle: a cost bound degrades resolution, not reach. Past the budget
the step coarsens rather than the march stopping. It is safe because the quadrature was already
built for it — `carve_shape` integrates the threshold over the sample's own footprint and
converges to the carve's exact mean yield as that footprint grows, which is the identical
mechanism the `CARVE_END_METERS` hand-off rests on. So a coarsened sample is a low-resolution
rendering of the cloud that is there, not its absence.

**The first version of the coarsening was itself wrong, and the fourteenth entry is about how.**
It scaled the step by `exp2(0.5 * (real_samples - STEPS))` — geometric in the number of samples
already spent. Correct in magnitude, quantised in the worst possible variable. See below.

#### The cloud resolve had nothing to evict history with (CV17)

*"kamera hareket ederken shader bu şekilde trail bırakıyor. aşırı rahatsız edici."* The Low tier
set `cloud_variance_clip = false`, and reading `cloud_taa.comp` it becomes clear that flag did
not select a cheaper clip — it removed the clip entirely, leaving a plain EMA at up to 0.97
feedback. The neighbourhood clip is the **only** mechanism in that resolve that ever rejects a
stale sample, so without it reprojection error does not decay, it compounds over ~33 frames.

This is a general trap in tiering temporal passes: a temporal filter is a feedback loop, and the
rejection test is not a quality feature layered on top of it, it is the loop's stability
condition. Cutting it does not make the tier cheaper and softer; it makes the tier divergent. The
knob now chooses *which* rejection — 9-tap YCoCg variance clip, or a 5-tap cross min/max clamp on
the cheap floor — and there is no setting that blends history unrejected.

The second half was there under both settings: **alpha was never clipped at all.** This buffer's
alpha is the march's transmittance and `CloudCompositePass` folds the sky through it, so an
unbounded alpha history leaves a stale *silhouette* — a cloud-shaped hole in the sky with
correctly resolved colour inside it. Clipping colour alone would have fixed the trail's hue and
left its shape, which is the sort of half-fix that reads as "better" in a screenshot and is still
wrong.

#### Not yet verified

None of this has been seen on screen. The march change is confident — the mechanism is plainly
in the code and its signature matches the report exactly — but whether it is the *whole* of what
image10 shows needs the re-shoot. If a hard azimuthal cut survives, the next suspects are the far
window's square rim (a square centred on the camera reaches 131 km along an axis and 185 km along
its diagonal, and the horizon at 1 500 m is ~138 km, so the corners cross it and the edges do
not) and `max_iterations` running out on near-horizontal rays.

### Where this stands — 2026-08-02, eleventh entry: WM-SEED, weather that is somewhere

The user put our Earth next to Google Earth's. Ours: a uniformly milky sphere. Theirs: mostly
*clear* ocean, with weather on it in discrete pieces. Then the instruction:

> *"manual mod artık fair weather overcast ile değil seed ile belirlenecek. bu şekilde dünyanın
> bir yeri bulutlu fırtınalı iken bir yeri tam olarak açık hava olacak."*

**The defect was structural and it was hiding in a predicate.**
`RuntimeSimulation::procedural_weather_enabled()` was literally
`static_cast<bool>(weather_provider_)`. So "Manual" did not mean *a* provider — it meant **no
provider**, and no provider means no published `WeatherField`, which means one hand-authored
deck stack applied to every square metre of a planet. Every downstream symptom followed from
that one line, and `StaticWeather` had been sitting in `weather_provider.hpp` since W4 for
exactly this job, never installed, dead code. A boolean whose two states are "the system" and
"the absence of the system" is not a mode, and it is worth noticing when one is written.

The mode is named now, and both states install a provider. The distinction that replaced it is
worth more than the one it removed — **placed** versus **grown**:

* `SeededWeather` places it. Deterministic, defined at every point on the body, costs nothing to
  run, and evolves in nothing but the season. That is not a weaker `ProceduralWeather`; it is
  what an author who typed a seed asked for.
* `ProceduralWeather` grows it. Evolves on its own, and resolves only the nest's 384 km.

#### The zonal term is the half that costs nothing and does most of the work

Most of what makes a photograph of Earth recognisable is a function of **latitude alone**: a
cloudy ITCZ, startlingly clear subtropics under the descending branch of the Hadley cell, a
cloudy midlatitude storm track, a moderate polar cap. Three Gaussians on a base land it near
0.64 / **0.06** / 0.66 / 0.31 with ordinary midlatitudes at 0.30, and it needs no seed, no
simulation and no data because it is the same every year.

Those numbers were 0.72 / 0.32 / 0.76 / 0.51 when this shipped, taken from annual-mean **total
cloud fraction** climatologies, and the user's first look at the result was "her yer bulutlu
seedde, bulutlu olmayan yer yok mu?" — everywhere is cloudy, is there nowhere clear? They were
right, and the mistake is worth keeping written down because nothing about it looks like a
mistake: the numbers are accurate, sourced, and cited to the right measurement. They are simply
*a different quantity from the one every consumer downstream reads them as*. Total cloud fraction
counts sub-visual cirrus and broken fields that read as clear sky from orbit; the carve treats
this value as the fraction of sky it fills with opaque, lit cloud. So the clearest place on the
planet was drawn a third solid. **An error bound on a number says nothing if the units are
wrong** — the same shape of failure as the eighth entry's approximation, arriving through
dimensional analysis instead of through calculus.

The subtropical minimum is the term that earns its place. It is why an orbital photograph has
large, genuinely clear ocean in it — and a field without it reads as overcast everywhere no
matter how anything else is tuned. Getting the *clear* right turned out to matter more than
getting the cloudy right, which is the opposite of where the previous nine entries spent their
effort.

#### One definition, two evaluators, and why they could not be one

The placement has two consumers on opposite sides of the render seam: the simulation samples it
per column to publish the weather field the bake reads, and the cloud march samples it per step
out past every baked window. The obvious answer — publish a global coverage texture — dies on a
binding: `cloud.frag` has none free (2–6 are taken, 7–10 are named/reserved), and the bindless
heap would drag a descriptor set into a pass that deliberately avoids one.

So it is a **closed form**, and the centre list is the only thing that crosses: twelve
directions and two scalars each. Both sides then evaluate the same function from the same
numbers. That is not merely cheap, it is the correctness property — a bake and the field it
fades into holding different opinions about where the weather is would show up as a seam at the
far window's rim, which is the exact artefact PL1 was written to remove.

The frame conversion lives on the host, in `publish_synoptic_field`, because the host is the
only object holding both halves of it: the provider answers in latitude and longitude, the march
has nothing but a radial in scene space, and `Environment::planet_body_axes` is the rotation
between them. Twelve vectors once on the CPU, versus two inverse trigonometric functions per
centre per march sample.

#### What changed in the globe field, and why the noise had to become multiplicative

`cloud_globe_envelope` was reading `cloud_deck_a[i].z` for its coverage. That value is compiled
from the **camera's own column**, so using it out there restated the weather over the observer's
head as the weather everywhere on the body — the uniformity bug, in the one place written to
cure it.

The mesoscale noise pattern stays, because the placement carries synoptic scale and a satellite
image plainly has structure below it. But it is now a **multiplier** rather than an additive
swing. An additive one cannot leave a sky clear: whatever the weather says, the noise veils half
of it back over, and a subtropical high never reads as empty. A multiplier preserves zero.

#### Named limits, honestly

* **Nothing advects and nothing evolves.** A seeded sky is the same sky an hour later. That is
  the definition of the mode, not a gap — but it does mean a seeded storm arrives nowhere.
* **The cost is a loop per march sample.** Twelve dot products and twelve exponentials, and
  `cloud_globe_envelope` can be called up to three times per sample (probe, envelope, light).
  Hoisting it to one evaluation per sample is the first lever if the far field turns out
  expensive; it was not done pre-emptively because it costs the call sites their independence.
* **Procedural mode publishes no centres**, so from orbit it shows the zonal climatology and
  nothing placed. That is truthful — a core resolving a 384 km nest genuinely does not know
  whether it is raining on the far side of the planet — but the obvious next step is to *extract*
  centres from T1's own pressure field, whose extrema are real highs and lows. A coarse lat/lon
  scan at publish cadence would do it, and then the globe would show the simulated systems.
* **Unverified by eye.** As with the last three entries.

### Where this stands — 2026-08-02, tenth entry: the sun was where the camera was

Two reports, one line of code:

> *"kamera karanlığa girerse aydınlık taraftaki bulutlar kararıyor — oysaki o bulutlar ışık görüyor"*
> *"bulutlar, LOD dahil, akşam güneş görmeyen yerlerde kayboluyor. bir nebze gözükmeli"*

`cloud.frag` computed the solar zenith cosine once per pixel, in the **camera's** radial frame,
and gated the direct beam on it:

```glsl
vec3 camera_up = normalize(-center);
float mu_sun = dot(camera_up, sun);
if (atmo_ray_sphere(deck_mid_radius, mu_sun, surface_radius) > 0.0)
    sun_radiance = vec3(0.0);
```

So the question "is the sun up?" was asked once, about the observer, and its answer was applied
to every cloud in the frame. Stand in your own shadow at sunset and the sunlit tops to the west
go black with you. Look at the Earth from orbit and the whole globe is lit or unlit as one unit,
with no terminator anywhere on the deck.

**The interesting part is that the code argued for itself, and the argument was checkable.** It
said the angle at the deck and the angle at the observer "differ by the deck's angular extent
about the planet centre — under a degree for anything this march can reach — which is finer than
the LUT resolves." Both clauses fail:

* *"for anything this march can reach"* stopped being true earlier the same day. PL1 gave the
  march the whole visible globe — tens of degrees of arc, with a real terminator inside the
  frame. A change that widens a system's reach silently invalidates every bound stated in terms
  of that reach, and this one was two entries above it in the same file.
* *"finer than the LUT resolves"* was **never** true, and this is the part worth keeping. The LUT's
  resolution is irrelevant when what consumes the angle is a **step function**. Near sunset the
  gate's output changes by 1.0 across the degree the argument dismissed. An error bound on an
  input says nothing about the output unless you also bound the derivative, and a discontinuity
  has none. The approximation was exactly wrong at the only moment anyone looks at a sunset.

The fix is per-sample solar geometry (`cloud_sun_at`), inside the `density > 0.001` branch so
only samples that get shaded pay the fetch. The gate is softened across the Sun's angular radius
— it is a disc, not a point, so a deck darkens over the half-degree its limb takes to set, and a
hard test would draw the terminator across the cloud tops as a razor line, which is the one place
on Earth nobody has ever seen one.

**The Moon had the identical bug, and it is what emptied the night side.** The reflected-body
loop gated each body on the *camera's* horizon, so from a daylit camera the Moon is below it,
`continue` fires, and the one light the dark limb had was skipped for the entire frame. The sum
is now accumulated ungated and cut off at each sample's own horizon, using the dominant body's
direction — after sunset the ephemeris sorts the Moon first and it outweighs everything behind it
by orders of magnitude, so one direction for the sum is accurate to well past what is visible.

Skylight is the one term that stayed where it was, and deliberately: the sky-view LUT is a
directional map of the *camera's own* sky and there is no honest way to ask it about a point a
thousand kilometres away. The march adds back only the **difference** — `max(daylight(sample) −
daylight(camera), 0)` against the same 2 % of the top-of-atmosphere beam `CLOUD_SKY_AMBIENT` was
originally calibrated to. A sample no sunnier than the camera contributes exactly zero, so every
view that was already right is bit-identical and the only thing this can change is the case that
was wrong.

**Unverified by eye.** One prediction worth checking against the next screenshot: the limb
speckle (CV14 below) should get *better*, because a large part of it is the old code handing the
full noon beam to samples past the terminator, where a saturated cloud sits against a dark limb
and any density flicker reads as a firefly. If it does not improve, the cause is the step
schedule on grazing rays and not the lighting.

### Where this stands — 2026-08-02, ninth entry: the planet, and the yellow

Four screenshots, and each answered a different question. Two from ~100 km looking down, two from
low altitude looking up; Manual weather mode, fair-weather preset, Ultra.

**PL1 — from orbit the sky was a box, and it was exactly the box the design predicted.** Both
windows are camera-centred flat squares in world XZ (32 km and 262 km) that fade to nothing
across their rim *by design*; `cloud_field_window.glsl` has carried the paragraph explaining why
a flat prism is meaningless at planetary distance since it was written. What was missing was the
thing that paragraph defers to — §7.5's coarse planet-scale far field — and nothing had ever
been put there, so past 131 km the planet simply had no weather.

What went in is deliberately the *cheap* form, and the reason is not effort: **there is no data
to be finer with.** The nest is 384 km across and T1 runs on the host (§3.3), so anything
detailed out there would be invention wearing the clothes of meteorology. So the planet-scale
field is the deck stack's own envelope — the same authored or compiled sky both windows bake
from, through the same `cloud_height_gradient` and the same union — modulated by a planetary
pattern read from the carve volume along the sample's own geocentric unit vector. A unit vector
into a seamlessly tiling volume needs no projection, so it has no rim to distort: that is the
whole reason this can cover a sphere where a window cannot. Its dominant feature lands near
300 km, the scale of a real synoptic cloud mass.

Three consequences worth recording:

* **The windows now fade *into* it rather than into nothing**, so the far rim is a hand-off. The
  layers agree in the mean because they are built from the same decks; they differ only in the
  structure the coarse one cannot know about, which is what a LOD hand-off should look like.
* `cloud_height_gradient` moved into `cloud_field_window.glsl`. Two answers about where a deck's
  top is would have shown as a step at the rim, and the bake and the march are now one answer.
* **The skip guarantee weakens, and it is stated rather than quietly assumed.** Inside the
  windows a zero probe *proves* a region empty (it is a max-pool, and the carve only removes).
  The planet-scale field is a point evaluation, so out there the hop is a bound tied to the
  pattern's own scale — a fortieth of a feature — not a proof. The near-field cone light march
  is also gated to the near window now: past it the volume it refines does not exist, and each
  call is twelve probes.

**The yellow was a real bug, and a satisfying one.** From orbit the deck rendered saturated
yellow, and closer in the whole frame went orange. `cloud_composite.frag` continues the view
path past the froxel volume with Bruneton's ratio identity — `T(near→top) = T(near→far) ·
T(far→top)` — which holds only while the near point's path to the top runs *through* the far
point, i.e. while the ray climbs. Look down from orbit and the far point is the deeper one; the
containment reverses and so must the quotient.

Taking the wrong branch is not an inaccuracy. `transmittance_to_top` integrates the straight
line from `r` along `mu` to the top *sphere*, with no planet in the way, so at a downward `mu`
it marches through the body: the closest approach is `r·sqrt(1 − mu²)`, essentially the centre
for a near-vertical look-down, and an exponential density profile evaluated below the surface is
astronomically large. Both fetches underflow toward zero, they underflow by different amounts
per wavelength — blue first, because Rayleigh — and their ratio is then an arbitrary saturated
colour. Hence yellow, and hence *only* on the clouds: the ground reaches the frame through
`sky.frag`, which never asks this question.

It also explains something in the ground-level shots. For a slightly downward ray the wrong
quotient exceeds one and clamps, which means near-horizon pixels were getting **no distance
extinction at all** — part of the bright horizon slab that this document has blamed on the carve
twice.

**And the eighth entry's named lever got pulled the same day.** That entry chose the integration
step as the carve's LOD footprint and wrote down the case that would overturn it. The orbital
screenshots are that case: looking straight down, the ray crosses a 2 km deck almost
perpendicular — so along-ray error is bounded by the deck's own thickness — while the lateral
footprint on the deck is under a hundred metres. Band-limiting that to the step's kilometre
erases the cloud field outright. The LOD now selects against `pow(lateral² · axial, 1/3)`, the
equal-volume isotropic stand-in for the real anisotropic footprint, which lands near the step at
the horizon and near the pixel from orbit. The ladder's Nyquist fade still uses the step alone,
because that test genuinely is about the quadrature and not about the texture filter.

**Still open after this, and both were confirmed by eye rather than argued:** the clouds do not
read as cumulus at any distance (CV5+CV9 — flat lozenges, no vertical development, no mesoscale
organisation), and the horizon still carries a bright slab, of which the aerial fix above
addresses one contributor and not the rest.

**One risk in this change that the next screenshot should be read for.** The planet-scale field
puts cloud past 131 km where there was none, and from the ground that is precisely the horizon
strip already under complaint. Two things should more than pay for it — the extinction those
rays were not getting at all (above), and the fact that the far window already reached 131 km,
so the slab was never coming only from beyond it — but the honest statement is that the horizon
could read *more* solid rather than less, and if it does, the lever is that strip's extinction
and step rule rather than removing the planet.

### Where this stands — 2026-08-02, eighth entry: CV3, the LOD was band-limiting the wrong thing

The complaint was *"berbat LOD, bulutları direkt beyaz render ediyor"*. There were two defects
under it, and the second one is the interesting one because fixing only the first would have
replaced a white far field with a flat grey one.

**The floor was the LOD, and it was the wrong kind.** `cloud_density_carved` ended its scale
selection with `carve_scale = max(carve_scale, footprint * 4)` — never carve a feature the
march's own step cannot sample. The rule is right and the remedy was backwards: it keeps the
sampler inside its Nyquist limit by *inflating the cloud*. At sixty kilometres the High tier's
step is 1 200 m, so the floor is 4 800 m, and a five-kilometre blob at full solidity is fifteen
optical depths across. That is not a badly-filtered cloud, it is a wall — which is exactly what
"renders them white" describes.

Underneath it, nothing was filtered at all. The march volume was created with `mipLevels = 1`
and read through the shared `max_lod = 0` sampler, so a sample whose step spans a kilometre
point-sampled an 18 m texel: sixty times faster than the sampler, with whichever peak the
lattice landed on surviving the threshold at full amplitude. The taps were also implicit-LOD
`texture()` calls, which a fragment shader resolves from screen-space derivatives — undefined
inside the non-uniform control flow a ray march is made of. Every tap is now `textureLod` with a
stated level.

**Why a compute box filter and not `vkCmdBlitImage`.** The volume tiles under REPEAT addressing,
so its filter has to wrap; a blit's clamps instead, and at the coarse levels *every* texel is a
border texel, which would bake the tile seam into the chain. Because every extent is a power of
two the wrap costs nothing to honour — the eight sources of a destination texel are exactly
`2x + (0,1)³`, all in range — so `cloud_noise_mip.comp` is fifteen lines and states its filter
rather than inheriting one.

**The half that matters: a percentile threshold cannot be applied to a filtered field.**
`coverage` selects the top `coverage` of a field that is uniform on [0, 1] — that uniformity is
the CDF transform's whole purpose, and it is a property of the *unfiltered* field. Filtering
narrows a distribution; by the coarse levels it is nearly a constant at 0.5, so a hard threshold
against a filtered fetch answers "all cloud" or "no cloud" for a whole region, and the delivered
coverage stops resembling the requested one. Band-limiting alone would have traded a white far
field for a binary one.

So the march evaluates the *expectation* of its ramp over the detail the filter removed, and the
amount removed is not estimated. A mip level of a box chain is the conditional expectation of
the field given its block — an orthogonal projection — so the variance decomposes with no cross
term and the residual is exactly `Var(level 0) − Var(level l)`. `CloudNoise` measures it on the
host from a one-time readback of the finest level (the bring-up submit already blocks on a
fence) and pushes eight floats. The residual is modelled as uniform rather than Gaussian,
because the field itself is, which also makes the convolution a closed form instead of an `erf`.

Two properties are what make it worth the plumbing:

* At zero spread it is bit-for-bit the previous ramp, so the near field is untouched by
  construction rather than by tuning.
* As the filter takes everything, the mean goes to 0.5 and the spread to the field's own, and
  the result converges to `envelope · (1 − 1/(2·solidity))` = 0.8 of the envelope's water — the
  same mass `CLOUD_ENVELOPE_MEAN_SHAPE` (0.75, the difference being the erosion, which has
  faded out by 42 km) already tells every sun-depth integral in the frame to assume. So
  `CARVE_END_METERS`'s hand-off to the statistical mean stops being a cross-fade between two
  different answers and becomes purely a cost cut.

**The rag octave is deliberately not faded out at distance, and the billow ladder still is.**
They differ in their mean. The ladder is zero-mean, so fading it *is* its mean effect — and it
must keep fading, because a fully filtered tap converges to the channel's true mean while the
ladder subtracts `BILLOW_MEAN`, an estimate of it; the residual would be a uniform coverage
shift across the whole far field instead of averaging out between neighbouring samples. The rag
removes mass, so dropping it would make cloud undersides grow *denser* with distance — the same
"brighter the further away" failure by another route. The mip fetch converges it to its mean
bite on its own, which is the right limit and needs no fade.

**The judgement call, stated so a screenshot can overturn it.** The LOD is selected against the
*integration step*, not the pixel footprint, and the two differ by more than an order of
magnitude (0.02 of the distance against ~0.001). The step is the defensible choice — the march
is a quadrature and detail it cannot sample along the ray is not detail it can integrate — and
it matters that past `JITTER_FREEZE_METERS` the dither is frozen on purpose, so quadrature error
there is *not* averaged away and would read as stable structure that is not cloud. The cost is
that individual cumuli dissolve into coverage past roughly 25–30 km, and the structure beyond
that comes from the envelope — the nest's own cloud fraction, real weather at 2 km. If that
reads too smooth, the levers are `steps_far` (more samples, so a finer footprint follows) and
the equal-volume compromise `pow(lateral² · axial, 1/3)`; **not** the floor that was removed.

One cost note, unmeasured: at distance a larger fraction of in-envelope samples now return a
small non-zero density instead of exactly zero, so more of them take the shading path. The
sample budget itself is unchanged — `real_samples < STEPS` still caps density evaluations — so
the worst case is a ray spending its whole budget on lit samples, which was already the near
thick-cloud case.

### Where this stands — 2026-08-01, seventh entry: the carve had the wrong sign

A reference photograph settled what "AAA cloud" means here: an airliner view of a cumulus
congestus field, tops packed with self-similar convex turrets, the readability coming almost
entirely from dark creases *between* bulges, with a flat base and cirrus above.

**Every version of this carve until now built clouds by subtraction.** Threshold a base field,
then remove material where an erosion noise is high. Subtraction can only produce *concave*
features — bites, channels, holes — because removing material from a solid is how you dig. A
congestus top is the opposite: convex protrusions bulging outward. That is not a tuning
problem, it is a sign error, and it explains why every setting of the erosion strength landed
on either smooth lumps (weak) or shredded filaments (strong) and never on cauliflower.

Worse, CV2's `(1 - shape)` edge-scaling — which correctly stopped erosion from eating the cloud
body — also removed *all* structure from the cloud's interior. With density saturated at 1
across 60 % of the cross-section and erosion reaching only the rim, the sunlit face of every
cloud was a flat constant. The face is most of what you look at.

**What changed.**
* The billow ladder (three inverted-Worley octaves, lacunarity 0.45, gain 0.5) is **added to
  the base field before the threshold**, which displaces the isosurface outward into a bulge
  instead of biting a hollow out of a finished shape. It is zero-mean, so the coverage the
  envelope asked for still arrives; a hard clamp at 0.35 base-units bounds both turret size and
  coverage distortion regardless of how far the field's assumed spread is off.
* The polarity was free all along. `noise_worley` returns `1 - distance`, so channel `g` is
  already high at cell centres and low at cell walls — round bumps with creases between them.
  The old code took `1 - g` and subtracted it, which converts round bumps into round *holes*.
* Channel `g` itself was an fbm of an fbm, nine octaves deep. Each nesting multiplies the
  standard deviation by 0.68, so the field was too flat to displace anything visible, and the
  crisp cell boundaries — the entire point — were buried under fine detail. It is now nearly a
  single inverted Worley octave (4/9/19, incommensurate); the march's ladder does the fbm.
* **The creases are shaded, not just carved.** The ladder's own signed value is a direct local
  occlusion measure: negative means the sample sits in the gap between two turrets. It feeds
  extra sun-depth and an ambient factor. Without this the cauliflower is geometrically present
  and shades as flat white, because the light volume's 128 m texels cannot resolve a 100 m
  crease and never will.
* Subtraction survives in exactly one place — the underside, below the convective base, where a
  cloud really is being torn rather than pushed. Weighted by `1 - cauliflower` so the two
  regimes never fight over a sample, and with the polarity corrected: the old `wisp = g`
  removed cell *interiors* and left their walls, a web of thin ridges, which is literally what
  "tel tel kadayıf" describes and was the strand generator all along.
* The ladder band-limits itself per octave against the march footprint, so no mip chain is
  needed for it to be safe at distance. CV3 still applies to the base and rag octaves.

Two constants carry an estimate this shader cannot measure — an inverted Worley F1's standard
deviation, taken as ~0.18. `BILLOW_RELIEF` and `CREASE_SENSITIVITY` are derived from it and are
the pair to move together if the relief reads timid or violent. The clamp is what makes a wrong
estimate degrade gracefully instead of swallowing the threshold.

### Where this stands — 2026-08-01, sixth entry: the march was sampling one point per cloud

CV1 and CV2 were built and looked at. The verdict: the strands are gone, and what replaced
them is **speckle** — "nokta nokta", clouds rendering as isolated dots. Plus a second, sharper
observation about the near-camera hole: **it does not happen with stratocumulus.**

Both are the same defect, and that defect is in the march, not the model.

**Why CV2 turned smear into speckle.** CV2 was right and it exposed what was underneath. The
old carve delivered a field that ramped linearly from nothing to full across a whole cloud —
low contrast everywhere, no plateau. Undersample a low-contrast field and you get mush, which
is what "smeared cotton" and "tel tel kadayıf" were. CV2 gave the field a saturated core and a
sharp rim, which is what a cumulus actually is. Undersample a *high*-contrast field with a
per-pixel jittered sample position and each ray independently either lands in a core or misses
it — so neighbouring pixels disagree completely, and the field reads as dots. The contrast was
the fix; the sampling rate was always broken and was merely being hidden by the blur.

How badly: `seg_min = min(march_len / STEPS, max(shell_thick * 0.12, 40))`. `shell_thick` is
the union of every enabled deck, 800..12 000 m in the default sky, so the floor is **1344 m**.
Because seg_min was also the clamp's lower bound, the angular rule never engaged until 54 km
out. A fair-weather cumulus is ~1.5 km across. Every cloud in the sky was integrated by
**one sample**. No carve survives that.

**Why stratocumulus has no hole.** The same constant. The march sampled at `t`, weighted by
`dist_step`, then advanced — a left-endpoint rule started at `t0 + seg_min * dither` — so the
span between the shell entry and the first sample was integrated by nothing at all. Its size
is `seg_min`, which is 12 % of the *union shell*. Enable only stratocumulus and that union is
a few hundred metres thick, so the dead band is ~100 m and invisible. Enable anything tall —
or the full genus catalogue — and the union becomes 11 km, the dead band becomes 1.3 km, and
a large block of sky in front of the camera stops existing. The symptom tracking genus rather
than distance is what identifies the cause uniquely.

**A third defect found while fixing those two.** The empty-space hop was
`t += max(dist_step, cell_size)` against a 512 m near cell. With dist_step pinned at 1344 m, a
near-horizontal ray hopped 2.6 cells on the strength of one probe, all the way to the horizon
— skipping cloud it never tested. Distant cloud appearing and disappearing with camera angle
is that.

**CV4, done.** Search and integration are now separate step rules. Integration is angular
(2 % of camera distance, floored at 20 m, ceilinged at 1200 m) with `steps_far` rescaled from
"divide the march length" — which made the same tier resolve a horizon ray at 5 km per sample
and a vertical ray at 500 m — into a scale-free rate, so one tier means one sampling density in
every direction. The interval being integrated is tracked separately from the jittered sample
inside it, which is the unbiased midpoint rule the old code was reaching for and which starts
at `t0` exactly, leaving no uncovered span. The hop is now the exact distance to the boundary
of the region the trilinear probe actually proved empty (dual-lattice DDA across both windows,
`cloud_skip_exit`), so it is both maximally long and never long enough to cross untested sky.
`cloud_probe`'s vertical early-out gained a one-metre margin, because the first probe of every
ray sits exactly on the base sphere where float32 at planetary radius resolves to half a metre,
and on a coin flip it was answering "empty" for a region no texel had been asked about.

**Reordered.** CV3 (band-limiting the carve) was queued before CV4 and has been moved after it.
Band-limiting alone would have made things *worse*: with 1344 m steps the derived LOD is 5 —
a 4³ base volume — so the carve would have converged to honest mush everywhere instead of
dishonest speckle. The mip chain only pays once the steps are small enough for its LOD to be
low, which is what CV4 delivers.

**Still open, and the user is right about it.** The distribution model itself is thin: one
noise scale (2400 m) decides where every cloud is, so there is no mesoscale structure — no
clusters, no streets, no clear lanes between cell groups — and `water` is per column, so every
cloud sharing a column has identical density with no vertical structure beyond the profile
gradient. That is a real generation defect independent of sampling, and it is queued as CV9
alongside CV5's per-genus vertical geometry. But it is queued *after* CV4/CV3: rewriting what
distributes clouds while the march still takes one sample per cloud would be rewriting
something no one can see.

### Where this stands — 2026-08-01, fifth entry: the carve was eating the cloud

CloudsV2 was looked at and rejected — "tel tel kadayıf", stringy shredded wheat, no
readable cumulus, no cumulonimbus anywhere at moderate coverage, and a large unrendered
region right at the camera. A full audit of the march, the bake, the fog path, the editor
split, the mode seams and the physics side followed; it found fifteen verified defects in
the render tier alone. The four that produce the reported look are arithmetic, not taste,
and the first two are now fixed.

**The whiteout was flattening every thick cloud in the sky (CV1, fixed).**
`cloud.frag`'s post-march whiteout was gated on the *ray's total* transmittance rather than
on the camera being immersed, and its ramp's lower edge (0.02) is numerically identical to
the march's own opacity break — so by construction every ray the march calls opaque lost
100 % of its accumulated shading to a colour varying only with `dot(view, sun)`. At the
default cumulus (σ = 0.00279/m) that is 50 % of the shading gone at 717 m of body and all
of it past 1.4 km: bright shredded edges around a dull featureless mass, which is the
complaint verbatim. The block is deleted. The hole it papered over was real but is a
missing *term*: all three octaves of `cloud_sun_energy` carry the same Beer factor, so a
deep interior decayed to the ambient floor, when a medium of single-scattering albedo
~0.999 converges to a bright near-isotropic radiance instead. A saturating multiple-scatter
term keyed on the sample's own sun depth now rises where the ladder falls — it is exactly
zero at zero depth, so a fully lit edge is unchanged and the ladder needed no
recalibration, and because it is per-sample it *carries form* where the flat glow destroyed
it. A softened phase keeps the residual forward bias the whiteout's `mu` term was reaching
for. Mirrored into `cloud_panorama.comp` so the impostor does not read darker than the
marched sky it continues.

**The erosion was eating the body, not the edge (CV2, fixed).** Three compounding
arithmetic errors, all in the carve:

- The CDF uniformisation that makes the threshold deliver exactly `envelope` also leaves
  the survivors uniform on [0, 1] — a linear ramp across a whole cloud, a tenth of its
  volume above 0.9, no plateau. A `min(shape · 2.5, 1)` push restores a saturated core
  without touching the support, so the coverage statistics the uniformisation exists to
  guarantee are unaffected.
- Both erosion remaps subtracted a *flat* amplitude (0.42 and 0.30) regardless of depth
  into the cloud. Against a uniform ramp that deletes ~42 % of every cloud outright and
  leaves the noise's isolated peaks: filaments, literally. The bite is now scaled by
  `1 - shape`, so it cannot reach the plateau and the strength means "how deep into the rim
  does erosion reach" instead of "how much of the cloud is destroyed".
- The wispy/billowy height flip keyed on `cloud_shell_height` — the height within the
  *union of every enabled deck*, 800–12 000 m in the default sky — so a cumulus at
  1 000–3 200 m sat at 0.018–0.214 and never left the *base* regime, which erodes cell
  interiors and preserves their walls. Only cirrus ever received the round-floret branch.
  The bake now publishes a within-cloud height (0 at a sample's own base, 1 at its own top)
  in the near window's free `b` lane — measured from cloud above versus below on the nest
  path, from the deck's own span on the authored/derived paths — and the flip keys on that.
  The blend is also between two *different scales* now rather than between a value and its
  own complement: `mix(d, 1 − d, f)` has amplitude `|1 − 2f|` and collapses to a constant at
  f = 0.5, so the widest part of a cumulus — most of its silhouette — was receiving no
  erosion structure at all.

Also in CV2: the domain warp's amplitude drops 0.55 → 0.30. At 0.55 the
displacement-per-wavelength ratio is 0.41, into the folding regime, and a folded warp
pinches its features into sheared strands. And `CLOUD_ENVELOPE_MEAN_SHAPE` is re-derived
0.45 → 0.75 from the new carve's actual yield; left at 0.45 every sun-depth integral, cloud
shadow and impostor would light a sky 40 % thinner than the one being drawn.

The `b` lane's two meanings are safe because they are co-sited: the far window overwrites it
with sun depth (`cloudscape_far_light.comp`), and the carve's erosion — the only consumer of
the near meaning — has faded out entirely by 14 km, where the near window hands over.
Verified by reading that the skip, light-volume, shadow-map and panorama consumers all touch
only `r` and `a`.

**Still open on the reported symptoms**, in the order they are queued: the erosion octaves
are 5–30× below the march's Nyquist limit with no footprint floor and one mip level (CV3);
`seg_min` is derived from shell thickness and floors every step at 1.4–1.7 km in the default
sky, leaving the head of every ray uncharged, while the skip hops an XZ cell along an
arbitrary ray and over-runs the vertical cell 6× (CV4 — this is the near-camera clipping);
the envelope's only horizontal shaping is a scalar per column, so a tower that flares into
an anvil is not representable at all (CV5); the bake normalises the nest's physical σ and
clamps it, giving a 16.7× under-scale and 125× at a Cb core (CV6); the sky advects at a
genus constant times a *frame counter*, so wind speed scales with framerate and runs
opposite to the nest's own condensate (CV7); and the shading integrates the smooth envelope
while the geometry lives at 7–155 m, with an ambient term that makes cloud bases exactly as
bright as cloud tops (CV8).

### Where this stands — 2026-08-01, fourth entry: CloudsV2, the visual model rebuilt

The third entry's fixes (normalisation, TAA translation blindness, composite gate) were
correct and stay — but the user's verdict on the *look* stood even with them in: baked
128 m texels cannot hold a cumulus. §7.6 documents the rebuild: the bake now stores only
the physics envelope (coverage / profile / water), and the march carves the shape
analytically per sample at every distance in the shared pattern frame. Touched:
`cloudscape_field.comp` (envelope-only), `cloud.frag` (per-sample carve, the 200 m erosion
band deleted), `cloud_noise_volume.comp` + `CloudNoise` (precombined march volume, kind 4),
`cloudscape_skip/light_volume/far_light/shadow_map/panorama` (envelope×water×mean-shape),
`scene_weather_tail.glsl` / `SceneUniforms` / `CloudscapeCompilePass` (`cloud_field_pattern`
published; far supersample 8→2). Not yet verified by eye — the user builds and judges; the
carve constants (2400 m base, 620 m erosion, 0.42/0.30 strengths, 0.45 mean shape) are the
expected first knobs if the look is close but not right.

### Where this stands — 2026-08-01, third entry: the specks were a stale normalisation

**Twenty real minutes over vegetated summer land rendered nothing but specks, and the cause was
one constant that changed meaning without changing value.** §7.3's carve made the bake state its
density as *in-cloud* water — `σ / fraction`, divided by `coverage_reference_lwc` — but that
reference was still 1.5 g/m³, a number tuned for the old cell-mean semantics ("1 g/m³ averaged
over four square kilometres is a deep solid deck"). Against in-cloud water, 1.5 g/m³ is a value
no real cloud reaches: observed solid stratocumulus tops out at 0.3–0.5 g/m³, and the diurnal
steady state the entrainment closure now holds (72 h probe, 2026-08-01) runs an in-cloud water
path of ~66 g/m² over a ~700 m deck — roughly 0.1 g/m³. Divided by 1.5 g/m³ that baked at
density ≈ 0.06, and the march (`extinction_scale = light_absorption × 0.006`) turned 700 m of
that into optical depth ≈ 0.2: a sky that is 25 % covered by clouds too transparent to see.
Every carve fix in §7.3 was working; its output was being multiplied into invisibility one line
later.

The default is now **0.4 g/m³** — a solid stratocumulus bakes at ≈ 1, the fair-weather cumulus
the sim actually makes at ≈ 0.25 and rising toward its own top, translucent but plainly a cloud.
The Doxygen on `coverage_reference_lwc` now states the in-cloud semantics so the constant cannot
silently change meaning again, and the meteorology panel's "Overcast At" slider range moved down
with it (1e-4 – 2e-3). **Serialized scenes pin the old value:** an environment saved before this
change carries `coverage_reference_lwc = 0.0015` in its JSON and will keep rendering specks until
the slider is set to ≈ 0.0004 and the scene resaved.

**Two temporal defects came out of the same session's editor run, both in the W3 resolve chain
rather than in anything §7 touched.** First, the cloud TAA's sky-pixel reprojection fallback
treated the view ray as a direction at infinity — exact under rotation, blind under translation.
A deck 1–3 km away has real parallax, so flying past it kept the history at the same screen
position and the accumulation smeared it across the sky; worse, the sub-pixel acceptance boost
keys off apparent motion, which the blind reprojection reported as zero, so feedback rose to
0.97 precisely when the history was most wrong. The fallback now anchors the reprojected point
at the march's own transmittance-weighted mean depth (the `cloud_depth` MRT, newly bound to the
TAA) and offsets it by a new `eye_delta` row in the temporal block before applying the
translation-free matrix — restating the point relative to last frame's eye, which is the only
way a ray-marched point (no per-object previous transform) can see the camera move. Second,
CloudCompositePass sampled the TAA's pass-owned history unconditionally: switching clouds off
stopped the march and the resolve but not the composite, so the last resolved frame stayed
glued to the screen until clouds came back. The composite now reads `misc.w` (the cloudscape
master switch, via a truncated scene-block prefix) and passes the sky through untouched when it
is off.

### Where this stands — 2026-08-01, second entry: the deck gets a physical bound

**Cloud-top entrainment is in, and the 15 K floor stops being the operative bound.** Open item 4
below named the deck's steady state honestly: −13.9 K against `cloud_top_equilibrium_depression`,
a parameter and not a balance, with the condensate growing monotonically and the cloud-fraction
clamps pinned domain-wide (open item 1's freeze). What was missing is the term every stratocumulus
review names as the deck's actual regulator: the overturning the radiative cooling drives entrains
warm, dry air down across the inversion, warming the top back toward its environment and — the
half that ends a deck — drying the layer it condenses from.

Shipped as the flux-partitioning closure (Lilly 1968; Nicholls & Turton 1986),
`w_e = A · ΔF / (ρ c_p Δθ)`, in `atmosphere_forces.comp` directly under the cooling term whose
absorbed flux drives it — so the entrainment inherits the same top-concentration
(`exp(−κ·path)`) and the same closing condition, and the two shut down together. The entrained
fraction mixes θ, vapour and cloud water toward the level above; evaporation of the entrained-in
deficit is left to the microphysics dispatch one barrier later, where its latent cooling lands
without double-counting. Gated on a stable interface (Δθ > 0.05 K), because an unstable top is
resolved convection and entraining through it would count the same mixing twice. The efficiency
is data — `cloud_top_entrainment_efficiency` on `AtmosphereParameters`, default 0.8 (the enhanced
field estimate; written at the mid-range 0.4 and raised after the verifying runs below), folding
the evaporative enhancement into one coefficient between the dry 0.2 and enhanced ~0.8 literature
values; 0 removes the closure and reproduces the deck-on-its-floor state for A/B. The probe
carries `--entrainment` for exactly that experiment. The depression floor stays, demoted to the
runaway guard it should have been: a deck should now break by entrainment drying before its top
reaches −15 K.

Magnitude, by arithmetic rather than by run: 0.8 × 70 W/m² over a 5 K inversion at ρ ≈ 1 is
11.1 mm/s — the upper end of the measured nocturnal-stratocumulus range — and at a 400 m top level that is
a drying e-folding of a few hours, which is the timescale a nocturnal deck actually breaks on.
`test_atmosphere_nest.cpp` pins the closure's arithmetic through the mirrored
`atmosphere_cloud_top_entrainment` helper.

**Measured 2026-08-01, same day.** Four probe runs, in order. `--entrainment 0` reproduced the
deck-on-its-floor state to the digit (0.879 / 0.3262 / 0.0605, `sky_pin` 1.0) — the off-switch
and the layout change are clean. Fixed-sun at 0.4: a large improvement (condensate 0.17 instead
of 0.52 kg/m² at 72 h, `sky_pin` 0.31 instead of 1.0, the top riding at −18.8 K — above the
floor, so the bound really is the physics now) but condensate and pin were still creeping
monotonically under the constant 0.9 sun. Diurnal at 0.4: no permanent fog (base 575 m, not
19 m), but the deck never broke — coverage climbed monotonically to 0.84 and pin jumped to 0.94
at hour 46. Diurnal at 0.8: **a periodic steady state** — coverage cycling 0.22–0.29 with the
sun, days two and three nearly identical, condensate oscillating 0.010–0.023 kg/m², base steady
near 1.3 km, pin flat at 0.121, skin cycling 9–38 °C with no runaway. That run is what the
closure exists to produce and is why the shipped default is the field value 0.8, not the
mid-range 0.4 this entry was first written with. The functional suite (993 tests, including the
closure's new pins) passes on the same build.

### Where this stands — 2026-08-01

**The sponge experiment the entry below leaves open was not run, and the reason is the reason it
should not have been: the base state it would have been measured on is broken.** Reproducing that
entry's tropopause run first — 72 h, fixed sun, quiescent parent, exactly the shipped defaults —
reproduced its step count exactly (43 202) and nothing else. The drift near the tropopause is not
−1.07 K, it is a ±20 K oscillation with the sign alternating between adjacent levels, and it is
sitting on top of something an order of magnitude larger that the entry below does not mention at
all: **the column cools without bound.**

| | 6 h | 24 h | 48 h | 72 h |
|---|---|---|---|---|
| θ′ at 1585 m, the deck the run builds | +5.1 | −6.8 | −22.7 | **−42.7 K** |
| θ′ at the surface | +5.1 | +1.1 | −8.4 | **−21.7 K** |
| skin temperature | 33 °C | 13 °C | — | **−10 °C, still falling** |

Linear at about −0.8 K/h from the moment the deck closes over, with the sky field frozen to four
decimal places from 43 h onward — 0.879 cover, 0.3262 deviation, 0.0605 roughness, sample after
sample. A run in that state cannot answer a question about a sponge.

**The cause is the entry below's own newest term, and it was isolated rather than argued.** The
same 72 h with `--cloud-top-lw 0` holds the surface at +4.5 to +5.8 K for the whole run, at the
same 0.84 cloud cover — so the difference is not how much cloud there is, it is the term:

| `cloud_top_longwave_flux` | 6 h | 24 h | 48 h | 72 h | final skin |
|---|---|---|---|---|---|
| **70 W/m² (as shipped)** | +5.1 | −6.8 | −22.7 | **−42.7 K** | −10.4 °C |
| **0** | +5.2 | +5.4 | +5.8 | **+4.5 K** | +13.6 °C |

The conservation argument that entry makes for the term is correct and is untouched — a column
telescopes to `F0 (1 − e^{−κW})` and cannot radiate away more than its top is given. What it is
missing is the *other* bound: `F0` is a constant, so a cloud top that has cooled ten kelvin goes on
losing exactly what one at ambient does, and nothing in this model ever warms a cloud. Written that
way the term is a sink and not a flux. **Fixed** by scaling it with how far the top still is above
the temperature at which the sky above balances it (`cloud_top_equilibrium_depression`, defaulting
to the 15 K at which a 278 K black body has given up exactly the shipped 70 W/m², so the two move
together rather than drifting apart). Exactly 1 at ambient — a transient deck, the case the term
was calibrated on last week, is bit-identical.

**A second, independent half of the same one-way budget, found while isolating the first.** The
surface's downwelling longwave is Brutsaert's clear-sky emissivity *always*, including under a
fully overcast column. So the deck shades the ground's sunlight away above while the ground goes on
radiating to a sky that is not there: even with the cloud-top term removed, net radiation is
−42 W/m² at permanent noon under an 0.84-covered sky. That one is bounded — the slab equilibrates —
so it is a bias rather than a runaway, but it is also the term that would have *closed the loop* on
the first: a ground kept warm under a cooling deck drives a sensible flux back into it. **Fixed** by
blending the downwelling longwave toward the cloud base's own emission by cover and cloud
emissivity. The data it needs — cloud-base temperature and column liquid water path — comes out of
the walk `atmosphere_extinction.comp` already runs for the shading, so `cloud_shade` widens from two
channels to four and nothing new is computed.

**Both fixes verified, on the same 72 h at the same defaults.** The column stops running away, and
the two halves show up in different places, which is what says each is doing its own work:

| | as shipped | cloud-top term off | **both fixes** |
|---|---|---|---|
| θ′ at 1585 m, 72 h | −42.7 K, still falling | +0.6 K | **−13.9 K, settling** |
| θ′ at the surface, 72 h | −21.7 K | +4.5 K | **−1.4 K** |
| skin temperature | −10.4 °C | 13.6 °C | **13.2 °C** |
| net radiation | −76.3 W/m² | −42.4 W/m² | **+1.3 W/m²** |
| sensible flux | −57.7 W/m² | −34.0 W/m² | **−1.6 W/m²** |

The surface row is the unambiguous one: at permanent noon under an 0.879-covered sky the ground is
now within a watt and a half of radiative balance, where before it was losing 76. Notice that
removing the cloud-top term alone does *not* fix that — it leaves 42 W/m² of loss — so the two
defects really were independent, and only the second one is about the ground.

The deck's own row is bounded rather than resolved, and the distinction is worth keeping: it is
settling on **−13.9 K against a 15 K floor** (−2.1, −1.7, −0.9, −0.7, −1.0 K per six hours, still
shrinking), so what is holding it is the closure and not a balance the model found. Under a
quiescent parent with a fixed sun there is nothing else to hold it — the entrainment and subsidence
that limit a real deck are the terms §6 already names as under-resolved. **Named limit:** a
persistent deck sits against its radiative floor rather than at an equilibrium, and the floor is a
parameter. What has changed is that it is a floor at all.

**Two things this run did not settle.** The sky field still freezes to four decimal places from
43 h — 0.879 cover sample after sample — which is a separate question from the temperature and is
still open. And the step cost cannot be compared across these runs: it reads 12.35 ms against the
baseline's 7.87, but `advect wind`, `advect scalars` and `pressure` — none of which this change
touches — all moved by the same ~1.6×, so the machine and not the code is what differs. A cost
measurement for this change has to be taken as a pair of runs back to back.

**Still not measured:** the diurnal comparison. Phase B3b measured this same 72 h *with a day/night
cycle* and found no runaway (−0.86 K above 10 km), so the fixed-sun case above is the harsh one by
construction; what the fixes do to a realistic diurnal forcing has not been run.

**Still open from below, unchanged:** T1's 36 ms step on the main thread; the nest on the graphics
queue; Phase C's nest-side genus acceptance; nothing confirmed by eye. (§12's budget has since
been rewritten in B2c's per-simulated-second terms; its step figures carry the machine caveat of
open item 3 below.)

### The sponge experiment — 2026-08-01

Run on the base state the entry above made hold still. Five configurations, 72 h each, everything
else at defaults. **The oscillation was a placement failure, not a strength one, and the fix is one
number: `sponge_depth` 5000 → 9000 m.**

θ′ at the levels that oscillated:

| altitude | `base` 13 km edge | rate ×2, same edge | **9 km edge** | 9 km edge, rate ×2 | 6 km edge |
|---|---|---|---|---|---|
| 11 936 m | +4.18 K | +2.20 | **+0.03** | +0.03 | −0.01 |
| 12 430 m | **−13.45 K** | −10.93 | **+0.02** | +0.02 | −0.01 |
| 12 930 m | −11.75 K | −12.07 | **+0.02** | +0.01 | −0.01 |
| 13 951 m | +3.01 K | +1.66 | **+0.01** | +0.01 | −0.01 |

**Doubling the rate does essentially nothing and that is the whole finding.** The mode sat at
12.4 km, in the levels immediately *below* the old lower edge, where `nest_sponge_weight` returns
exactly zero — so the damping rate multiplied a zero however large it was. Covering those levels
removes the mode by three orders of magnitude. A Rayleigh sponge is a placement question before it
is a strength one, and this run is what says so rather than an argument.

**The old sponge was not working inside itself either.** At 14 km, within its own layer, `base`
carries 1.4 m/s of wind and +3 K of θ′; the 9 km edge leaves 0.11 m/s there. A damping layer
holding values like that is being fed faster than it absorbs — which is consistent with the mode
below it being the source, and is the second independent sign that the old configuration was not
merely imperfect but inert.

**Does it damp the weather it is meant to leave alone? No, and the 6 km run is what makes that
statable rather than asserted.** At the 9 km edge the boundary layer, the cloud deck and the free
troposphere are unchanged: 1585 m reads −13.98 K against −13.93, cloud water agrees to three
significant figures (6.80e-4 against 6.82e-4), the surface level −1.38 K against −1.37, and the
largest θ′ departure anywhere below 4 km is 0.08 K. At the 6 km edge — deliberately too deep —
wind between 4.6 and 9 km collapses to 0.02 m/s from 0.34 and θ′ at 6.2 km reverses sign, +0.31 to
−0.42. That is a sponge replacing the weather instead of bounding it, and it is what the 9 km
configuration is *not* doing.

**Named limit: the mode was not eliminated, it was relocated and reduced by an order of
magnitude.** The residual at 8.6–9.1 km grows from −0.88/−1.07 K in `base` to −1.59/−1.76 K at the
9 km edge — one level below the new edge, which is where a two-cell mode parks. ±1.8 K is the same
size as the ordinary variability this model already carries between 2 and 6 km, so it is no longer
distinguishable as a defect; it is not gone. Moving the edge again would move it again.

**One thing this run cannot settle.** Between 4.6 and 9 km the 9 km configuration also reduces wind
(0.34 → 0.28 m/s at 6.2 km, 0.33 → 0.17 at 8.2 km) in a region where the sponge weight is
identically zero, so it is not direct damping. It is either the removed mode's own wind signature
disappearing or a genuine change in the wave field beneath. These runs do not separate the two and
this document does not claim they do.

### What is open, in the order it is worth doing — 2026-08-01

Written down at the end of the session that closed the longwave budget and the sponge, so the next
one starts from a list rather than from a re-reading. Ordered by value per unit of work, and each
says what it would cost to answer, because on this machine a 72-hour probe run is six minutes of a
saturated GPU and that is a real constraint on what gets asked.

1. **The sky field freezes — diagnosed by arithmetic 2026-08-01, and the mirror is innocent.**
   The three frozen numbers identify themselves: 0.879 cover is exactly 900/1024 mirror columns,
   0.3262 deviation is exactly `sqrt(900·124)/1024 = 0.32623` — the standard deviation of a
   *binary* field with that mean — and 0.0605 roughness is exactly 120/1984 differing neighbour
   pairs. From 43 h the published coverage field is binary: every one of the 1024 columns reads
   exactly 0 or exactly 1. That is not a stuck publish path, it is `nest_cloud_partition`'s
   clamps (`atmosphere_nest_common.glsl`, the `across <= -1` / `across >= 1` branches): once the
   persistent deck settles onto its radiative floor the cell mean sits outside the top-hat
   width in every deck column, the diagnosed fraction pins at exactly 1 (elsewhere exactly 0),
   and the coverage, deviation and roughness become exact rationals of the column membership —
   which cannot move by less than 1/1024 and therefore holds to four decimal places while θ′
   beneath goes on settling (−0.9, −0.7, −1.0 K per six hours through the same samples). The
   readback, slot collection and publish were read end to end for this and are correct.
   **Not yet confirmed by a run:** `atmosphere_probe` now reports `sky_coverage_pinned`, the
   fraction of columns whose low-band coverage is exactly 0 or 1; the prediction is that it
   reaches 1.0 at the same sample the field freezes, and stays there. One 72 h run settles it.
   If it does, this item closes into item 4 below — a sky that cannot unfreeze is a deck nothing
   erodes, and the defect is the missing entrainment/subsidence, not the mirror.
2. **The diurnal comparison has never been run.** Every measurement in the 2026-08-01 entries is
   under a fixed noon sun and a quiescent parent, which is the harsh case by construction — B3b
   measured the same 72 h *with* a day/night cycle and found no runaway at all (−0.86 K above
   10 km). What the closed longwave budget and the deeper sponge do under a realistic forcing is
   therefore unknown in the direction that matters for shipping. One run, six minutes.
3. **The step cost of both changes is unmeasured.** The verification run reads 12.35 ms against the
   baseline's 7.87, but `advect wind`, `advect scalars` and `pressure` — untouched by either change
   — moved by the same ~1.6×, so the machine differs and not the code. §12's budget (rewritten
   2026-08-01) inherits the same caveat on its measured step figures. Two runs back to back on one
   machine settles it; nothing else will. **Run 2026-08-01, and it settled it:** three back-to-back
   6 h runs of the same binary and config measured **30.3 / 12.3 / 31.0 ms** per submission, with
   every stage's *share* constant across all three (pressure 32.8 / 30.1 / 32.8 %) — the GPU's
   clock state scales the whole step uniformly by ~2.5×, and within one state agreement is ±2 %
   (the 12.3 ms state also matches the 72 h run's 12.29 to three figures). The 7.87 → 12.35
   discrepancy is therefore a clock state, not the code. Isolating the two changes' own cost would
   need interleaved pre-/post-change builds (B3d's method); given that the post-change `forces` and
   `extinction` shares match their as-measured values, a regression is implausible and the
   measurement is not worth a second build unless §12 is ever missed in practice.
4. **The cloud deck settles against a parameter, not a balance.** With the closing condition in, a
   persistent deck stops at −13.9 K, which is its `cloud_top_equilibrium_depression` floor. What
   should bound a real deck is entrainment at its top and subsidence above it, both of which §6 and
   §14 already record as under-resolved at this vertical spacing. This is the largest piece of
   physics still missing and the only one on this list that is a modelling problem rather than a
   measurement — which is why it is last despite being the most important.
   **Addressed 2026-08-01 — the entrainment closure is in; see the second 2026-08-01 entry
   above. Not yet measured; the verifying runs are listed there, and this item stays open until
   they are run.**

Carried unchanged from below and not re-listed above: T1's 36 ms step on the main thread, the nest
on the graphics queue, and Phase C's nest-side genus acceptance. The standing "none of this has
been confirmed by eye" item is **partly retired**: the rendered clouds *were* looked at in the
editor on 2026-08-01, and they fail — see item 5 below. What remains unconfirmed by eye is the
meteorology behind them, not the pixels.

5. **The rendered clouds were confirmed by eye, and the confirmation is a failure on four
   symptoms.** Tuning was tried first and did not close any of them, which is consistent with
   what a read of the bake/march chain finds: each symptom is structural, and each carries a
   decision, so nothing was changed yet. Diagnosis by reading, 2026-08-01:
   - **(a) Clouds start as tiny flickering specks.** The low-coverage regime of the carve. After
     the CDF fix the threshold `remap(base_shape, 1 - c, 1, 0, 1)` really does keep only the top
     `c` of the field (`cloudscape_field.comp:529`), so at a young deck's 5–15 % coverage the
     survivors are the noise's isolated peaks — islands far smaller than one 600 m shape cell —
     and their remapped values sit barely above zero, where the erosion's fixed
     `detail_fbm * 0.45` threshold (`cloudscape_field.comp:535`) erases most of them; which
     peaks survive changes each rebake. This is the "coverage/density retune after the CDF fix"
     this document already records as never done, plus the same marginal-regime erosion shape
     that produced the confetti before it was moved off the density. The structural question a
     retune has to answer: the carve makes cloud *size* grow with coverage, where a real
     fair-weather sky holds full-sized cumuli in smaller *number* — the erosion amplitude and
     the carve law at low coverage are one authoring decision, not two.
   - **(b) Faint cloud visible from one viewpoint and not another.** The two windows do not
     carve the same clouds. `min_shape_scale()` is derived from each window's *own* texel
     (`cloudscape_field.comp:177-181`): the near window's floor is 2048 m, which leaves the
     nest carve's 2400 m alone, while the far window's is 1024 m × 4 × 4 = **16 384 m** — so
     the same sky is carved from two different octaves of the shape volume
     (`cloudscape_field.comp:275` and `:477`). Coverage agrees between them; the individual
     clouds do not, and a marginal cloud exists in one realisation and not the other. The
     near/far blend weight is a function of the camera (`cloud.frag:210-225`), so walking moves
     which realisation is being shown: the cloud appears and disappears with the viewpoint.
     The claim in `cloud_field_window.glsl` that the two windows "differ only in how finely
     they carve its shape" is false at feature level — they differ in *which clouds exist*.
   - **(c) Very harsh tile seams.** The same defect seen at the rim: the cross-fade
     (`CLOUD_WINDOW_BLEND_START = 0.84`, a ~2.6 km ring at ~13.8 km from the camera) blends two
     uncorrelated cloud fields, and on the light side it switches sources at the same ring —
     the amortized light volume inside, the far field's 8-bit sun-depth channel (16 m
     quantisation over 4 096 m) outside (`cloud.frag:267-284`). The far window additionally
     rebakes whole every 4 s of weather or 1° of sun (`cloudscape_compile_pass.hpp:374,385`)
     with no amortization (`record_density` rewrites the full volume), so everything past the
     ring pops on that cadence while the near side does not.
   - **(d) Stretched, flat, horizontal 2D cotton rather than cumulus.** Three quantisations
     compound. Horizontally, most of the visible sky is far-window (past ~14 km of a ~150 km
     march), where the 16 384 m carve floor means the shape noise varies at ~4 km horizontally
     but is effectively constant across a deck's few hundred metres of depth: a vertical
     *extrusion* of a horizontal pattern, which is exactly "2D cotton", and is the "smooth
     kilometre-wide lumps" regime the carve was built to break (`cloudscape_field.comp:449`)
     reappearing wherever the floor overrides it. Vertically, a thin deck occupies one to two
     nest levels, the nest-path bake deliberately applies no height gradient
     (`cloudscape_field.comp:492-498`) so the density steps on and off across one texel — hard
     top and bottom faces — and the extinction's interior profile is exactly 0 for a
     single-level cloud (`atmosphere_extinction.comp:88-93`: `min(above, below)/here` with both
     neighbours empty), so the ambient term lights the whole cloud as edge: flat.
   **Implemented 2026-08-01, same session, all four — by reading; the editor look is the
   verification and has not been done:**
   - **(b), (c), (d-at-distance): the supersampled far bake.** The far window now carves at
     the near window's scale and averages 8×8 sub-texel taps per texel — threshold first,
     then filter, the order the bake's own aliasing note names as correct — so both windows
     carve the *same clouds* and the cross-fade interpolates one realisation
     (`cloudscape_field.comp`: `bake_at` factored out of `main`, `supersample`/`slab_base`
     push fields, `min_shape_scale` divides by the tap count). The 64× cost is paid by
     amortization: the far bake records 16 Z slices a frame into `far_source_`, which nothing
     samples mid-bake, and the completing frame resolves the sun depth and publishes the
     pending placement together (`CloudscapeCompilePass`: `far_baking_`/`far_queued_`
     /`far_completing_` state machine) — the 4 s whole-volume hitch becomes a ~16-frame
     rolling update, and triggers that fire mid-bake queue the next bake instead of
     restarting.
   - **(c), the light side of the ring:** the far sun depth is now square-root encoded across
     its 8-bit channel (`cloud_far_sun_depth_encode/decode` in `cloud_field_window.glsl`,
     shared by writer and both readers): 0.06 m steps at the shallow depths where the Beer
     term still moves, 16 m only past ~1.5 km where it has collapsed. The crossover ring
     stays co-sited with the density blend on purpose — that co-siting is what keeps a sample
     from reading density from one window and light from the other; what made it visible was
     the decorrelation and the quantisation, and both are addressed.
   - **(a): the coverage-adaptive carve.** The nest path carves from the same shape volume at
     a coordinate scale of `1/sqrt(coverage)` (clamped 3×), so island *diameter* stays
     constant and island *count* falls with coverage — the marginal distribution is
     scale-invariant, so the measured CDF statistics and the delivered area both survive
     untouched. The fixed 0.45 erosion was examined and deliberately left: post-CDF the
     percentile remap tops out at 1 inside any island whatever the coverage, so with
     full-sized islands the erosion prunes rims and the weakest clouds rather than erasing a
     young sky wholesale — the speck-erasure was a symptom of island size, not of the
     erosion's amplitude.
   - **(d), vertically:** the nest carve now tapers its *carve coverage* (never the water)
     where the level above is clear — sampled off the extinction field a nest level up — so
     the noise eats domes and turrets out of a deck's top while the base, which is flat
     because the condensation level is, stays flat. And the extinction's interior profile is
     floored at half the cell's own cloud fraction (`atmosphere_extinction.comp`), so a
     one-level deck stops reading as all-edge and the ambient term shades it as a body.

### Where this stood — 2026-07-31

**Shipped since the entry below:** the base-state vapour profile is fixed (§6's carried item, the
one named there as the highest-value thing left in the model), and cloud-top radiative cooling is
in. Two other carried items were *closed by measurement without shipping a change*, which is a
different outcome from being deferred again and is written up as such:

- **`humidity_scale_height` now folds the mixing ratio, not the relative humidity.** The airmass
  was drier than it should be at every altitude — RH 0.41 at 1.3 km and 0.095 at 5 km against a
  documented ~0.62 — because the exponential was applied to RH and then multiplied by `q_s`, so
  the profile decayed twice. Measured after: 0.70 at the ground, 0.62 at 1.3 km, 0.50 at 5 km.
  *Fixing it exposed a second defect the old form had been hiding:* with a single scale height the
  mixing ratio climbs back through saturation aloft, because above ~7 km `q_s` folds faster than
  the vapour does. Measured, RH reached **0.81 at 9.5 km** and every run began under a global
  cirrus deck. The base state therefore now carries the Weisman–Klemp (1982) relative-humidity
  ceiling as two parameters (`free_troposphere_drying`, `free_troposphere_exponent`); it binds
  only above ~4 km and holds the tropopause at 0.175. The land-cover presets were re-measured and
  now set the airmass humidity along with the surface, because a semi-desert is not merely a dry
  *surface* — under a 70 % airmass a dry surface still built a 2 km afternoon deck. The four
  presets deliver the sky each tooltip promises, measured over 11 h from sunrise.
- **Cloud-top radiative cooling is in, and it is not the sink this document said it was.** Added
  to `atmosphere_forces.comp` as the flux difference across a level,
  `F0 (e^{−κW_top} − e^{−κW_bottom})`, which telescopes down a column to `F0 (1 − e^{−κW})` and is
  therefore conservative at any vertical resolution — the differential form `dF/dz` over-cools an
  optically thick level by its own opacity, 16× for a 250 m level holding half a gram per
  kilogram, and was the first version. Measured, the term *maintains* nocturnal cloud rather than
  removing it, which is the textbook result and is why nocturnal stratocumulus exists at all: with
  a subsiding parent the deck still peaks and falls, so C1's evening-decay clause stays closed, but
  the overnight fall is 15 % where it was 40 %. Cost: `forces` 0.789 → 0.885 ms, the whole step
  7.79 → 7.92 ms.
- **The subgrid closure is *not* rescaled with the tier, and that is now a measurement rather than
  a deferral.** See §6, whose named limit has been rewritten: the scaling was built in its
  physically correct form and makes tier agreement worse.
- **The slow cooling near the tropopause is not numerical diffusion in the transport.** See the
  open item below; the attribution in the entry below is withdrawn.

**Still open, and the reason each stopped where it did:**

- **The tropopause drift is diagnosed but not fixed.** Run with the transport stepping 43 202
  times and *no flow at all* (no thermal seed, no surface exchange), the drift near the tropopause
  over 72 h is **0.02 K** — so it is not the semi-Lagrangian scheme diffusing across the base
  state's sharpest θ gradient, which is what this document claimed. Under surface forcing the same
  72 h reaches **−1.07 K at 12.9 km**, and the signal is a *growing oscillation* rather than a
  drift (0.005 → 0.13 → −0.58 → −1.07), concentrated at 12.4–12.9 km — immediately below the
  Rayleigh sponge's lower edge at 13 km. That is the signature of gravity waves radiated by the
  forced motion amplifying as density falls and accumulating under the sponge. The sponge's own
  profile is already a `sin²` ramp with zero slope where it begins, so the next thing to try is
  starting it lower; `atmosphere_probe` now carries `--sponge-depth` and `--sponge-rate` for
  exactly that experiment, and the experiment has not been run. **Run on 2026-08-01 and closed;
  see the entry above.**
- **The global core's 36 ms step is still on the main thread.** Not started.
- **The nest step is still on the graphics queue.** Not started.
- **Phase C's nest-side genus acceptance has not been run.** Not started.
- **Nothing in this pass was confirmed by eye in the editor**, which leaves the standing item
  below unchanged and adds this pass's changes to it. The editor builds.
- **§12's performance budget is still stale**, and now doubly so — the T2 step figures above
  supersede it again.

### Where this stood — 2026-07-29

**Shipped:** A · B1 · B2 · B2b · B2c · B3 (a–e) · C1 · C2 · **C3**. Phase B's acceptance bar is met,
its last clause closed by C1's subsidence. T1 is now the nest's parent: `SynopticLayer` is deleted,
the Davies zone is fed from real fields, and the editor authors by injecting vorticity.

Phase C's acceptance is met at the dynamical level: run through the shipped provider with nothing
injected, the core grows eddies at **0.254/day** and runs a baroclinic life cycle, and a
`FrontPassage` preset produces a measured front passage over the observer — the trough deepens,
passes, the ridge builds, and the wind veers from north-northeast to nearly due west.

**Next: T0's real climatology** sourced and baked, which is what gives the core a mean state with
continents in it instead of analytic latitude bands.

**Settled since:** the preset and click amplitudes were about 3x too strong, and the cause was the
injection placing a *monopole*. Fixed by compensating it — see §11's C3 notes below, where the
before/after table is.

**Carried, deliberately, with the reason each time:**

- ~~**`humidity_scale_height` does not do what it is documented to do**~~ — fixed 2026-07-31, see
  the entry above. It was indeed the highest-value item left in the model, and fixing it changed
  the regime enough to retire one named limit and rewrite another.
- ~~**`cloud_critical_humidity` is calibrated at 2 km and does not scale with the spacing**~~ —
  settled 2026-07-31 by measuring the proposed fix and rejecting it. §6 carries the numbers.
- **`AtmosphereSurface` — the land/sea seam** — blocked on the same terrain height field Phase D
  is blocked on (§15). See B3c for why building it early would be worse than not having it.
- ~~**The nocturnal cloud's only sink is now subsidence.**~~ Cloud-top radiative cooling shipped
  2026-07-31 — and the premise here was wrong: it is a *source* of nocturnal cloud, not a sink.
  What limits a real deck is the entrainment of dry air the cooling drives across the inversion,
  and that is **resolved rather than parameterized**. At 250–560 m spacing aloft the inversion is
  under-resolved, so the nest now carries the maintaining half of the stratocumulus energy balance
  more faithfully than the limiting half and over-produces nocturnal cloud. That is the new named
  limit, in place of the old one.
- **A slow cooling near the tropopause**, ~−0.35 K/day over 72 h under surface forcing. ~~Attributable
  to numerical diffusion in the semi-Lagrangian transport~~ — **that attribution is withdrawn**: with
  the transport stepping 43 202 times and no flow, the drift is 0.02 K over the same 72 h. It is
  flow-driven, it is a growing oscillation rather than a drift, and it sits immediately below the
  sponge's lower edge. Diagnosed, not fixed; see the 2026-07-31 entry.
- **§12's performance budget table is stale.** It budgets the T2 step at 2 ms; the measured step
  is ~10 ms at High. The honest metric established in B2c is *cost per second of weather bought*
  (1.27 ms), not cost per step, and the table has not been rewritten in those terms.
- **An async compute queue** is now a straightforward win rather than a blocked one, since the
  step already submits at frame end and waits on the frame's readers.
- **The global core's rain is still low, but far less so since T0 landed.** A two-layer
  quasi-geostrophic model has no Hadley circulation, so the ITCZ has to be T0-forced. On the
  analytic bands global mean precipitation was 0.25 mm/day against Earth's 2.7; on the baked
  climatology it reaches **1.5–2.4 mm/day** (§4.1). The remaining gap is the moisture *cycling
  timescale*, not the saturation ceiling — see §4.1's named limit.
- **The global core's step is 36 ms and runs on the main thread.** Invisible at 1× time scale, a
  visible hitch under time compression. Moving it to a worker is straightforward and is not done
  because nothing consumes the tier yet.

**Never visually confirmed in the editor.** Everything in B2c, B3 and C1 was settled with
`atmosphere_probe` headlessly, plus builds and tests. The frame-end submission move, the surface
panel rework and the ice optics have not been looked at on screen. C2 is a different case rather
than a worse one: nothing renders the global core, because nothing yet consumes it — the swap
below is what puts it on screen, and until then `atmosphere_global_probe` is not a substitute for
looking, it is the only way to look.

### Phase A — Spatial coupling *(existing physics, maximum visual delta)* — **shipped**

Publish T2's existing grid as a world-addressed field (`Render::WeatherField`, uploaded by
`WeatherFieldPass` into a 64×64×3 3-D texture); make the cloud march read coverage from it
per sample with noise demoted to detail (§7.3); sample the driving column at the camera
rather than at the scene's geodetic anchor, so precipitation and world coupling describe
where the player actually is; repair the DIP break — the host holds `IWeatherProvider`,
the concrete type leaves `ISimulation`, and the editor reaches authoring through
`IWeatherAuthoring` — which makes `IngestedWeather` installable for the first time.

*Acceptance: a front is visible as a line across the sky; it is raining on one side of
the view and clear on the other; the pattern translates with the wind; no visible tiling.*

**How the coverage authority is applied.** The bake keeps its wrapping tile and keeps
carrying *shape*; the field supplies *how much cloud belongs here*, as a scale about the
column the deck stack was compiled from (`WeatherField::reference_coverage`). The scale is
applied as a re-threshold, `remap(d, 1 − c, 1, 0, 1) · c`, not as a density multiply: a
multiply would fade a whole cloud toward transparency, where a re-threshold erodes its
low-density fringe first, so edges retreat and holes open the way falling coverage actually
works. `c == 1` is exactly the identity, which is what makes a uniform sky render
bit-identically to one with no field at all.

**Deferred to Phase B1, and why.** Three items originally listed here turn out to be the
*same* change and are dishonest to split: the field stayed a wrapping 65 km tile, so the
light volume and the cloud shadow map — both addressed in *tile* space, holding a `uv` with
no recoverable world position — could not consult it at all, and genus stayed an input rather
than a derived label. All three were closed together in Phase B1 below.

### Phase B1 — The cloudscape window *(Phase A's deferred trio)* — **shipped**

The one change all three deferred items reduce to: the baked field becomes **camera-centred
and non-wrapping** (§7.2), which makes its addressing invertible and therefore makes every
consumer able to say where it is.

Two windows rather than one, because the march reaches ~150 km and a single window with that
reach would put ~600 m between texels beside the camera: a near window at the old 32 768 m
span and 128 m texel, and a far window over 262 144 m at ~1 km, cross-faded across the near
window's rim. Both are baked from the same weather field, so they agree about where the
weather is and differ only in how finely they carve its shape.

The light volume and the cloud shadow map re-address into the near window and thereby carry
spatial weather with no lookup of their own — the field they march already holds the
simulation's coverage per column. Genus becomes a derived label (§7.4): the bake resolves one
per baked column through `Render::classify_cloud_genus`, the same classifier the editor
readout and the METAR path use, with its thresholds uploaded as data; a manually authored sky
still bakes its own deck stack, because `WeatherField::derives_genus` lets the *producer* say
which it is. The march shell follows the field's own union span when it classifies. The far
window carries its own optical depth toward the sun, since the near light volume does not
reach it. The per-sample coverage-scale/reference-column mechanism, the mirrored
anti-repetition tap, and the field's second density channel all retire.

The wind is absorbed into the window's origin and republished as a residual each frame, so the
sky advects continuously between rebakes; rebakes are triggered by authored change, a moved
shell, 8 % of span of camera drift, a weather cadence, and (far window only) the sun moving a
degree.

*Acceptance: the same bar as Phase A, now also with cloud shadows and sun energy that follow
the front instead of the observer's own column, and with different genera visible in one view
where the simulation put them.*

**Named limit.** Ground more than ~16 km from the camera loses its cloud shadow: the map
covers exactly the near window and fades across its rim. A second, coarser cascade over the
far window is the natural follow-up.

### Phase B2 — Real thermodynamics and vertical structure — **shipped**

`RegionalWeatherGrid` is deleted. In its place a GPU regional nest: prognostic θ, `q_v`, `q_c`,
`q_r` and a staggered Arakawa C-grid velocity field over 192×192×48 cells at 2 km horizontally
and ~54–560 m vertically, stepped in game time on a CFL-chosen Δt. Ten compute stages —
MacCormack-corrected semi-Lagrangian transport with a monotone limiter at Courant ≈ 1, buoyancy
with condensate loading through the virtual potential temperature, Coriolis, subgrid diffusion,
a Rayleigh sponge, Davies lateral relaxation, an anelastic pressure projection, Kessler warm-rain
microphysics with saturation adjustment and latent heating, optical extinction, and the readback.

**The pressure solve is a vertical line solver, not §6's FFT.** The grid is deliberately
anisotropic, so the Laplacian's vertical coupling outweighs its horizontal by `(Δx/Δz)²`; the
vertical is solved exactly per column by a Thomas sweep and the horizontal iterated red-black.
Named limit: no coarse-grid correction, so a fixed sweep count leaves a small residual
divergence. Semi-coarsened multigrid with that shader as its smoother is the refinement.

**Two things forced by the editor rather than chosen.** It runs three `ISceneView`s, so the nest
is a *device-level* service rather than a render pass — one atmosphere, not three divergent ones
— and it is centred on the simulation's observer rather than any camera. Its writes are ordered
against those three readers by a timeline semaphore each view's first submission waits on.

**§9.2's query mirror is pulled forward from Phase E**, deliberately and only as far as it had
to be: the moment the grid moved onto the GPU there was nothing left on the CPU for
`sample_column` to read. A 32×32 lattice of `WeatherColumn`-shaped records, read back
asynchronously, is what every existing gameplay consumer is now served from. §9.1's full
`AtmosphereProfile` stays in Phase E. Before the first readback the answer is the base state — a
clear sky with the synoptic wind — rather than an invented coverage.

**`SynopticLayer` survives as the parent solution**, feeding the Davies zone. Named as an
interim; §5's quasi-geostrophic core replaces it in Phase C.

**Determinism is given up here in practice, not only on paper.**
`test_weather_determinism.cpp` is deleted per §0 and §14, replaced by
`test_atmosphere_nest.cpp`, which pins the base state against the International Standard
Atmosphere, Magnus saturation against its textbook values, the stretched grid closing on its
domain top, and the mirror's cold start and transcription.

*Acceptance: a cumulus grows from a thermal, deepens, rains, and its downdraft cold pool
triggers a neighbour. Cloud bases sit at the lifting condensation level.* **Not confirmed by
eye** — nothing has been run.

### Phase B2b — Extinction drives the cloudscape directly — **shipped**

The last piece of §7.1. `cloudscape_field.comp` gains a third path, taken whenever the nest is
running: `σ_ext` read straight from the nest, with no genus, no deck and no height gradient. A
cumulus has a flat base because the condensation level is flat, not because a gradient function
draws one. Tiled noise survives only as a sub-2 km modulation at 35 % amplitude — §7.3's
demotion in its final form, since the nest resolves 2 km and that is all it is still needed for.

Baked density is stated against the extinction of 1 g/m³ of liquid water rather than in absolute
1/m, so the medium's authored absorption keeps the meaning it has always had: *where* the cloud
is and how it falls off across its own edge is physics, while how opaque a given amount of water
reads stays authored. The march shell follows the readback's own lowest cloud base and highest
cloud top, so the baked field's vertical resolution sits on the cloud rather than on empty
stratosphere.

Three paths now, ordered, each truthful when the one above it has nothing to say: the nest, then
per-column genus from the published field, then the authored deck stack.

### Phase B2c — The correctness pass B2 exposed — **shipped**

**The instrument came first.** `render/probe/atmosphere_main.cpp` (`atmosphere_probe`) brings the
nest up on a headless Vulkan device, steps it through hours of simulated weather in seconds of
wall clock, and prints the observer column's *unreduced* vertical profile. Everything below the
"Fixed in the cloudscape bake" heading was settled with it, and none of it was settleable without
it: three hours of weather is thirty seconds of wall clock rather than half an hour of an editor
session, and the mirror's `WeatherColumn`-shaped record — a vertical reduction by construction —
had already destroyed the evidence every remaining question needed. The profile it reads is
`AtmosphereProfileLevel`, published on `AtmosphereMirror` beside the columns and written by the
same readback dispatch: §9.1's diagnostic slice, deliberately not §9.1's contract.

It takes parameter overrides (`--sensible`, `--latent`, `--seed`, `--seed-length`, `--seed-period`,
`--eddy`, `--sweeps`, `--tier`, `--dt`) so that a claim about one term is separated from the rest by
running without it rather than by arguing about it. The two seed-correlation overrides exist for
exactly that: setting a correlation scale far below the cell or the step reproduces the white field
this phase replaced, which is how the table below was taken without keeping two builds around.

B2 and B2b are shipped and do what they were designed to do. Running them made visible that the
render path they feed, and the nest itself, both carried defects no amount of correct new code
could compensate for. This section is the record of what was **measured**, because on this phase
the measurements are the deliverable — every hypothesis reasoned from a screenshot in this work
turned out wrong, and every one settled by porting the code and sampling it turned out right.

#### Fixed in the cloudscape bake — all three pre-date the nest

**`coverage` did not mean coverage.** The Nubis threshold `remap(base_shape, 1 - coverage, 1, 0,
1) * coverage` is a percentile cut, and a percentile cut is only meaningful on a field that is
uniform on [0, 1]. `cloud_noise_common.glsl` was ported exactly to numpy and sampled on lattices
from 56³ to 100³ (converged to five decimals): the deck path's `base_shape` is a narrow bell,
mean 0.779, standard deviation 0.038, **entire support [0.574, 0.906]**. The threshold therefore
never reached the field above ~0.43 coverage:

| requested coverage | clear sky asked for | clear sky delivered |
|---|---|---|
| 0.42 (Cumulus) | 58 % | **0.0 %** |
| 0.70 (Stratocumulus) | 30 % | **0.0 %** |
| 0.95 (Nimbostratus) | 5 % | **0.0 %** |

Every deck at or above half coverage was a gapless slab shaped by nothing but
`cloud_height_gradient` — a flat-based, flat-topped plateau of uniform white. That is what the
sky had been made of since long before the field became a window.

Fixed by pushing the field through its own cumulative distribution, which is exactly the
transform that turns a threshold into a percentile. `base_shape` is near-Gaussian (skew −0.6),
so the standard logistic approximation to the normal CDF earns its single `exp`: measured, it
lands delivered clear sky within 1.5 points of requested at every coverage. The transform is
**monotone**, so it cannot alter the field's level sets — it relabels them. Cloud shapes are
untouched; only which iso-surface `coverage` selects moves, and that was the whole defect.
Constants are per-path (the deck path mixes two taps, the nest takes one, cirrus reads another
volume). **The authored coverage and density defaults were tuned against the broken field and
need retuning.**

**A window may not carve what its own grid can sample.** The far window spans 262 km across 256
texels — 1024 m each — and was asked to carve 4200 m-shape-scale cumulus. The shape volume lays
four Worley cells per `shape_scale`, so a cloud is 1050 m: **1.03 samples per cloud**. Because
the bake thresholds before storing, the alias returned with its gaps filled rather than blurred,
and 130 km of march through it integrated to an opaque white square with the near window showing
as a hole in the middle. `min_shape_scale()` now floors every path's shape scale at sixteen
texels. The near window's floor lands at 2048 m and every genus sits above it, so this is a
far-window correction that costs the near window nothing; the far window draws ~4 km cloud
*clusters*, which is what a hundred kilometres of sky resolves to anyway.

**The nest was never built, and nothing said so.** `procedural_weather_enabled()` is off by
default and persisted per scene; with it off nothing publishes `AtmosphereForcing`, so
`AssetLibrary` never constructs the nest and the bake silently falls back to the authored deck
stack. The Meteorology panel (§10) now names which rung of that chain is failing instead of
reporting "no readback yet", and offers the fix beside the diagnosis.

#### The nest, measured

Three hours of simulated time through the observer's column, logged from the panel, produced no
condensation whatsoever — `cloud_base`, `cloud_top`, all three bands' coverage, and rain all
exactly zero. The diagnostics added to the readback's three spare `extent` lanes (surface
relative humidity, column peak |w|, lifting condensation level — all free, since the lanes were
already allocated and `moisture` was already bound and never sampled) say why:

| | |
|---|---|
| peak \|w\| anywhere in the column | **3.5 × 10⁻⁴ m/s** — four orders below convective scale |
| surface relative humidity | **69.5 % → 41.4 %**, −9.9 points/hour |
| low-band `dT` | +0.069 K/hour |
| lifting condensation level | **762 m → 1466 m** — receding |

At that updraft a parcel needs 25 days to reach its condensation level, and the level is moving
away faster than anything approaches it. **Running the nest longer does not bring it closer to a
cloud.** Two independent faults: the dynamics produce essentially no vertical velocity, and the
boundary layer loses about a quarter of its surface water in two hours despite a positive latent
flux.

#### Causes: one eliminated, one found

*Eliminated by reading rather than assuming.* Subgrid diffusion was the leading hypothesis — 40
m²/s on a 38 m surface layer would be a 36-second damping timescale and would kill both the
dynamics and the moisture. It is **horizontal only** and divides by the 2 km spacing, giving
~10⁻⁵/s. Far too weak. The hypothesis was wrong.

*Found.* The buoyancy did not compute the equation stated at the top of its own file. `B = g ·
(θ_v′/θ̄_v − q_c − q_r)` requires the prime to cover the vapour term, but `moisture` stores
**totals** while `theta` stores a perturbation, and the vapour was taken at face value. For a 7
g/kg surface layer the spurious part is 0.043 m/s² against 0.0068 m/s² for a 0.2 K thermal —
**six times the signal**. Being horizontally near-uniform it drives no updraft of its own; the
pressure projection cancels it, which is what a projection does to a term already in hydrostatic
balance. But the solve is a fixed number of relaxation sweeps, so the residual it leaves scales
with what it was asked to remove, and the convection was running under a six-fold larger one.
Corrected by subtracting `nest_base_vapour`.

`MOISTURE_UNIT` was defined separately in four shaders and in none of the shared headers, which
is how the diagnostic above first reported 69500 % relative humidity. It now lives in
`atmosphere_nest_common.glsl` beside a note that `theta` and `moisture` use *opposite*
conventions in the same nest.

#### The drying was not drying: the vapour field was frozen by its own storage format

The moisture volume was `rgba16f`, chosen for range — mixing ratios are a few grams per kilogram
and never approach fp16's ceiling. What decides a format is not the range but the **ratio of a
step's tendency to the value it lands on**. The surface latent flux adds

```
Δq_v = F_latent · Δt / (L · ρ̄ Δz) = 90 · 2.44 / (2.501e6 · 66.3) = 1.33 × 10⁻³ g/kg
```

per step to a surface layer holding 7.27 g/kg, where fp16's spacing is 3.91 × 10⁻³ g/kg. **The
increment is 0.34 of one unit in the last place**, so `q_v + Δq_v` rounded straight back to `q_v`,
every step, forever. Measured and then independently reproduced: the probe reported 7.2695 g/kg
unchanged after 4 434 steps, and an emulation of the same rounding predicts 7.269531 exactly,
against 11.25 for the arithmetic the shader was written to perform.

So the boundary layer never moistened at all. Relative humidity fell because potential temperature
(fp32) accumulated heat normally while vapour did not, the lifting condensation level receded
because warming alone pushes it up, and no column could condense however long it ran. The
`−9.9 points/hour` recorded above is that, and only that — **nothing was ever lost.** The mirror
reported a *relative* humidity, which cannot distinguish air that is drying from air that is
warming, and the hypothesis was formed from the one number that could not answer the question.

Fixed by making the moisture volume `rgba32f`, which leaves the increment at some 2 700 units in
the last place. VRAM for the moisture pair goes 27 → 54 MB, the nest's total from ~108 to ~135 MB at
this tier. The memory-cheaper alternative — store vapour as a departure from the base state, as `theta`
does, so the stored magnitude sits near zero where fp16's spacing is tiny — needs the base-state
transport term in the advection and gives the four channels of one texture two conventions, so it
is recorded as the refinement rather than taken.

Confirmed working: column water now integrates to 11.32 kg/m² after three hours against 10.96 at
the start, where `F_latent · t / L` predicts 0.389 kg/m² of gain.

#### The updraft is not weak; the forcing has nowhere to go

With the vapour unfrozen, three hours at the authored fluxes leaves the observer column like this:

| level | altitude | θ′ | q_v | buoyancy at the face below |
|---|---|---|---|---|
| 0 | 19 m | **+17.64 K** | **12.66 g/kg** | — (rigid ground) |
| 1 | 99 m | **0.0000 K** | **6.8719 g/kg** — *bit-identical to its initial value* | **0.317 m/s²** |
| 2 | 214 m | 0.0003 K | 6.3391 g/kg | −6.3 × 10⁻⁵ m/s² |

The surface fluxes have piled 17.6 K and 5.4 g/kg into the lowest 54 m, and after 4 434 steps the
level above has received **nothing** — not a little, not a diffused fraction: the same floats it
started with. The buoyancy at the interface is 0.32 m/s², three orders of magnitude larger than
anything else in the column, and the vertical velocity there is 1.7 × 10⁻⁴ m/s.

The projection is not at fault; it is correct. A **horizontally uniform** heated slab cannot rise,
by mass continuity — there is nowhere for the air to come from — and removing exactly that is what
an anelastic projection is for. The thermal seed's variance is negligible beside 17.6 K of uniform
heating, so what the solver sees is a uniform layer and it annihilates it.

**The nest has no vertical mixing of any kind.** `eddy_viscosity` is applied horizontally only, by
explicit design (`atmosphere_forces.comp`), and there is no boundary-layer parameterization. Real
surface fluxes are carried upward by turbulence far below a 2 km grid — which is precisely what
§2.1 lists among the parameterizations that "actually determine the weather" (YSU, MYNN). Without
one, heat and moisture accumulate in a 54 m slab without bound, the layer above never destabilizes,
and no parcel is ever lifted. Running longer does not help; nor would more pressure sweeps, a
larger seed, or a stronger flux.

The chain above that link is intact, and this was checked rather than assumed. Forced with
`--sensible 0 --latent 220`, the surface layer saturates on its own, and at three hours the probe
reports relative humidity 100 %, the LCL descended to 19 m, and **cloud base at 19 m**: saturation
adjustment, latent heating, the optical extinction and the readback's cloud detection all work.
The nest makes cloud the moment a parcel reaches saturation. It made fog, because fog is what an
atmosphere with no vertical transport can make.

#### Also found, and fixed

- **Potential temperature was advected without its stratification.** `theta` is a perturbation
  about a height-varying base state, so its equation is `∂θ′/∂t + u·∇θ′ + w ∂θ̄/∂z = 0`;
  `atmosphere_advect_scalars.comp` carried only the material derivative. Without the second term a
  parcel keeps its warmth as it rises into an environment whose own warming with height it never
  feels, so it never loses buoyancy and never finds an equilibrium level: the domain's
  Brunt–Väisälä frequency is zero, convection has no top, and gravity waves have no restoring
  force. Added after the monotone limiter deliberately — it is a source, not a transported
  quantity, and clamping it to the upstream stencil would cap exactly the stable stratification it
  supplies. Latent while the updraft was zero; it would have been the next fault exposed.
- **Coriolis was never uploaded.** `NestParams::coriolis` is declared, is documented as riding the
  forcing, and was never assigned — so `f` was identically zero and `AtmosphereForcing::coriolis`,
  which the simulation computes from the observer's latitude, was dropped on the floor.
- **Three volumes were read before anything wrote them.** `atmosphere_shift.comp` seeds the
  prognostic state, because that is what a fresh atmosphere *is*; pressure, divergence and surface
  rain are step outputs and no step has run on the seed frame. Pressure is the one that matters:
  the relaxation warm-starts from the field it left behind, so undefined contents propagate rather
  than being overwritten and a single NaN would be permanent. Cleared on seed.
- **`thermal_seed_amplitude` is a rate, not a magnitude.** The header said K, the shader scales it
  by `dt` and its own comment says K/s. Corrected in the header, with what it implies stated: the
  seed is re-drawn and added each step, so what it produces is a random walk rather than a bounded
  wobble. *(Superseded below: the walk turned out to be the source of the domain's ground fog, and
  the seed now modulates the surface flux instead of θ.)*

#### The boundary layer, added — and what it did and did not fix

The missing transport is now in: vertical eddy diffusion of *total* potential temperature and the
moisture species, on a `K(z)` profile over a mixed-layer depth **diagnosed per column by
the parcel method** — walk up from the surface, stop at the first level whose potential temperature
exceeds the surface parcel's by 0.5 K. `boundary_layer_depth_m` survives as the cap the free
troposphere's inversion puts on it. It lives in the advection stage rather than beside the
horizontal diffusion in `atmosphere_forces.comp`, because that shader reads its stencil from the
image it writes: horizontally that is a smoothing operator either way, but across a
surface-to-air gradient of tens of kelvin it would not be.

Diagnosed rather than prescribed because the *growth* is the diurnal cycle's mechanism. A fixed
depth mixes the whole layer from the first step and dilutes the surface moisture into ten times
the air it should have; a diagnosed one is a few tens of metres at sunrise and deepens as the
surface parcel outgrows more of the stratification above it.

Three things it fixed, measured:

- The mixed layer is real. After eight hours, `q_v` is uniform at ~4.7 g/kg from the ground to
  1 600 m and untouched above — a textbook well-mixed profile where there had been a 54 m slab at
  +17 K against a level that still held its initial value to the last bit.
- Relative humidity now peaks at the **top** of the mixed layer, which is where cumulus belongs.
- Domain peak |w| went from 5 × 10⁻³ to 5 × 10⁻² m/s, and the column began to vary in time
  instead of marching monotonically.

And the thing it did not fix: **the nest still condenses only at the surface.** Across a sweep of
Bowen ratio (1.4 down to 0.26), mixed-layer cap (700–2 500 m) and airmass moisture (base surface
RH 0.70–0.80, parent anomaly 0–0.15), every run that made cloud made it at 19 m — fog — and no run
made cloud at the mixed-layer top. Mixed-layer-top RH reaches 78–95 % and stops there.

That is not a defect to be tuned out; it is the model class. A grid-mean model condenses when the
*cell mean* saturates, and a fair-weather cumulus is a 200 m–1 km thermal overshooting the
mixed-layer top in the **tail** of the subgrid distribution while the mean stays subsaturated.
2 km cannot represent that, which is exactly why every operational model at this resolution carries
a subgrid **cloud-fraction** closure (Sundqvist, Smith) on top of its boundary-layer scheme. What
the nest *can* resolve is the organized end — a storm updraft, a frontal band — because those are
kilometres across.

#### The cloud-fraction closure, and the six defects it uncovered

The closure is in. A cell's humidity is now a **top-hat distribution** about its mean rather than
a single value — half-width `(1 − critical)·q_s`, after Sommeria–Deardorff and Mellor, the uniform
member of the family Smith (1990) generalises — and condensation takes its saturated tail.
Fraction and condensate come out of the same partition, evaluated on total water and the
liquid-water temperature so it is a *diagnosis of the end state* rather than a rate applied to
whatever the last step left behind; evaporation then falls out with no branch. At
`cloud_critical_humidity = 1` the width is zero and it collapses **exactly** onto the
all-or-nothing adjustment it replaces, which is what makes it one condensation path and not two.
`Render::atmosphere_cloud_partition` mirrors it on the CPU so the identities are tested rather
than asserted.

The fraction is carried, not just computed: the moisture volume's fourth channel, the extinction
volume's alpha (displacing a saturation margin nothing read), and from there the bake — which now
thresholds against the fraction and draws at `σ / fraction`, the in-cloud water, instead of
treating the cell mean as both. Band coverage in the mirror is the maximum-random overlap of the
levels' fractions rather than `1 − exp(−τ)`, which measured opacity and called it coverage.

**It did not, on its own, make a cumulus.** What it did was make the nest's actual state legible,
and three defects in the physics came out of the first eight-hour run — each one found by
measurement, none of them visible from a screenshot:

1. **A cold level decouples from the boundary layer permanently.** The parcel test correctly finds
   a column stable when its lowest level is colder than the one above, diagnoses zero mixing depth,
   and leaves the level exchanging with *nothing* — so the thermal seed's random walk accumulated
   in it without limit. Measured at −8.76 K after four hours, 99 % relative humidity, against a
   level 80 m above it at 50 %. Every cloud the closure made in that run was that fog. Fixed with a
   floor of the two lowest levels on the diagnosed depth: mechanical turbulence at the ground does
   not switch off with the stratification, which is why every operational scheme keeps a non-zero
   stable-case exchange.
2. **The diffusivity profile weakened where it needed to be strongest.** The parabola
   `4·K_peak·f(1 − f)` goes as `K_peak·(z/h)` near the ground, so its surface mixing *falls as the
   layer deepens* — and the lowest face is the one that must carry the whole surface flux out of a
   54 m level. With a 2 500 m layer it left 12 m²/s there; the surface level sat 9 K above the one
   80 m up, the layer never homogenised, and its top reached 57 % relative humidity after eight
   hours of heating. Replaced with Troen & Mahrt (1986), `K = κ·w_s·z·(1 − z/h)²`, whose slope does
   not know `h` at all. A profile normalised to its own peak cannot express that, so the parameter
   became the **velocity scale** `w_s` (1.5 m/s, peaking near 220 m²/s a third of the way up).
3. **The thermal seed was an unbounded random walk.** An additive kick on θ redrawn every step is
   exactly that, and with 37 000 columns its cold tail kept a few percent of the domain in
   permanent ground fog that the sky reported as cloud. It now modulates the **surface flux**,
   which is where the heterogeneity physically lives: bounded by construction — a heated surface is
   heated more here and less there, never refrigerated — and it stops at dusk with the flux it
   scales, which is when a stable nocturnal layer should not be stirred. The amplitude became a
   dimensionless patchiness (0.4).

#### The defects that were not in the physics — and cost more than the ones that were

Everything above was found in a day of headless probe runs. The next three took several days of
the user's wall clock, and they are the ones worth reading, because none of them is a
meteorology bug and all three present as **a blue sky** — which is also exactly what a correct
model with a dry airmass presents as. That ambiguity is the actual defect; the individual bugs
are just what was hiding behind it.

4. **The sky ran ahead of the nest and the surplus was silently discarded.** The nest steps in
   game time and takes at most `max_steps_per_frame` steps a frame, dropping the rest — which is
   the right call (§3.4: weather briefly behind beats a frame that stalls to simulate an hour of
   it) and was nowhere reported. Measured in a real session: **4 sky-days asked for, 6.2 hours
   simulated.** The panel's own warning could not catch it because it estimated the sustainable
   rate as `step × steps_per_frame × 60`, and the editor was drawing at about twelve frames a
   second — so it printed *"the atmosphere is keeping up with the sky"* while four of every five
   seconds of weather were being thrown away. The lesson generalises past this tier: **a rate
   derived from an assumed frame rate is not a measurement.** Both the verdict and the
   *Match sky to atmosphere* button now use the observed ratio of weather asked for to weather
   simulated, over a recent window rather than the session total so that acting on it converges.
   The counter-intuitive consequence is worth stating plainly, because it is what wasted the
   days: **animating the sky faster does not make the weather evolve faster, it makes less of it
   happen.**
5. **The bake eroded density where it should have eroded shape.** The fringe erosion subtracts an
   absolute threshold, averaging 0.225, which removes a fringe from the deck path's *authored*
   density of order one. The nest path's density is *measured*: a marginal fair-weather deck —
   14 % cloud fraction, 0.01 g/kg, sitting right at the critical humidity — reaches the erosion
   at 0.058, so it drove 87 % of the texels to zero and crushed the survivors toward it. What
   rendered was not cloud but the noise's own peaks, and since the field rebakes every nest step,
   *which* peaks survived changed each time: flickering specks. The carve and the erosion now run
   on the dimensionless **shape** and the measured water multiplies once at the end — which is
   what §7.3's own split says ("the simulation supplies the coverage, the noise carves the
   shape"); the code had merged them one multiply too early.
6. **Nothing named which rung was empty.** Between condensate and pixels sit a switch, a published
   field, a genus flag, a march shell and a bake, and every one of them fails to the same blue
   sky. The Meteorology panel now carries a **Render path** section that names the rung — clouds
   off, no field, field not marked as meteorology, shell with no height, or a healthy shell with
   its span — beside the observer column's own cloud extent. This is the same "name the rung
   rather than the symptom" pattern the panel already used for the nest's build chain, and it
   should have been extended here the moment the bake started reading condensate.
7. **`max_steps_per_frame` did not do what its own documentation said, so defect 4 had no
   remedy.** The field is documented as "steps a single frame may take before the rest is
   dropped", and the resume note below used to name raising it as "the cheapest lever there is".
   It clamped the *accumulator* and nothing else: `record_step` was called once per frame
   whatever it was set to, so the nest's throughput was pinned at exactly one step per frame and
   raising the number bought more tolerated lag rather than more weather. That is the mechanism
   behind "4 sky-days asked for, 6.2 hours simulated" — and it is why the lever the panel offered
   could not have worked. A frame now records as many steps as are due, bounded by the cap and
   again by what the recording slot's descriptor pool can serve, which is derived from the pool's
   size rather than assumed to fit. The thermal seed's clock moved to a push constant, since
   several steps share one upload of the parameter block and would otherwise draw the identical
   pattern. Verified: handed four steps' worth of elapsed time per call, the nest simulates the
   full hour in a quarter of the submissions, and the column agrees with the one-step-per-call run
   at every sample.
8. **The query mirror froze whenever the nest stepped continuously — including in every editor
   session.** `collect_readback` looked for a slot whose `timeline_value` had passed the
   semaphore counter, but a slot carries only its *most recent* submission's value, and with
   three slots in flight the CPU sits exactly three submissions ahead. The value that has
   actually completed is therefore always one no slot still carries. Measured over a hundred
   consecutive steps: the counter read 95 while the three slots held 96, 97 and 98, and the sweep
   matched nothing every single time. The mirror advanced only when something else idled the
   device — which is why every measurement in this phase came from the probe's own sample points
   and looked fine. Gameplay reads that mirror (§3.2), so the "two to three frames stale" the
   design rests on was in fact "stale until the GPU happens to catch up". A slot is now collected
   at the one point its completion is already established: immediately after the wait that
   precedes overwriting it. Steps measured per hour of simulated weather went from 2 to 1 471 of
   1 478.

#### The symmetry break was doing a fifth of its job

Defect 3 above made the thermal seed *bounded*. It left it **white**: a hash of the cell index and
the step index, so the field had no length scale but one cell and no time scale but one step —
and, both of those being grid quantities, it made the render quality tier and the frame rate into
physical parameters of the weather.

Measuring it needed an instrument that did not exist. Mean coverage cannot tell a broken cumulus
field from a sheet holding the same total cloud, and that distinction is the *entire* question a
symmetry-breaking seed decides; `sky_cloudy_columns` stops discriminating the moment every column
holds a trace, which in a convecting airmass is immediately. So `atmosphere_probe` grew two domain
diagnostics off the mirror it already reads — `sky_coverage_sd`, the spatial deviation of coverage
over the whole domain, and `sky_coverage_roughness`, the mean difference between *neighbouring*
mirror columns. Together they separate the two ways a sky can be variable: a few large cloud masses
give a high deviation and a low roughness, while structure at the lattice scale gives both, and
structure at the lattice scale is the one thing a grid-mean model has not earned.

Roughness over simulated hours 3–6 (192³ nest, 250 W/m² each way, surface RH 0.90), relative to
running with the seed switched off altogether:

| seed field | domain roughness | mean coverage | condensate |
|---|---|---|---|
| none at all | 1.00 | 0.1334 | 0.0027 |
| **white in space and time — what it was** | **1.22** | 0.1335 | 0.0026 |
| 6 km in space, white in time | 1.25 | 0.1335 | 0.0028 |
| 6 km / 60 s | 1.88 | 0.1340 | 0.0027 |
| 3.7 m / 900 s | 3.15 | 0.1361 | 0.0027 |
| 24 km / 900 s | 2.27 | 0.1363 | 0.0031 |
| **6 km / 900 s — now** | **3.72** | 0.1380 | 0.0034 |
| 6 km / 3600 s | 4.94 | 0.1420 | 0.0012 |

**The seed the nest had was within 22 % of not being there.** Four realisations of the field (the
lattice shifted 5.7–6.3 km) agree to 0.5 %, so the ordering is not sampling noise.

The *time* axis carries most of it, and the reason is arithmetic rather than meteorological: an
uncorrelated kick accumulates over N steps as √N where a persistent one accumulates as N, so over
the 150 steps a boundary-layer eddy takes to turn over it is an order of magnitude weaker. Space
alone buys nothing at all (1.25); time alone buys most of it (3.15); the two together buy the rest.
Both scales matter in the right direction — 60 s is too short to lift anything and 24 km is wider
than the plumes that organise — which is what makes 900 s and 6 km physical choices rather than
fitted ones: h/w\* for a 1.5 km mixed layer under a 2 m/s convective scale is about twelve minutes,
and 6 km is the scale land cover, soil moisture and albedo actually vary over. A 3600 s field is
rougher still and is *not* the better answer: its condensate lands below the unseeded run, because
across a single convective afternoon the pattern barely renews.

The field is value noise on an (x, z, t) lattice with smoothstep interpolation, addressed in
**world** metres and game seconds. Two consequences follow from the units alone. It is anchored to
the ground, so the patches stay put when the nest re-centres on a moving observer, where a
cell-indexed field slid with the camera. And the tier stops being a physical parameter: Medium,
High and Ultra now agree on domain structure to 1.10× and on coverage to 1.06×, against 2.01× and
1.23× for a cell-scale field. **Low still does not** — a 4 km cell samples a 6 km patch with a cell
and a half — and that is a resolution limit rather than an inconsistency, stated here rather than
hidden because it is the honest form of one.

*Named limit.* One octave, so the ground this models is patchy at exactly one scale and real land
cover is not. Phase B3 is where the pattern should stop being a hash at all: once a surface energy
balance carries a land/sea mask and an albedo field, *those* are the heterogeneity and this becomes
the unresolved-turbulence residual on top of them.

*What was not confirmed.* The resume note this closes predicted a *visible* artifact — that
neighbouring cells near the closure's critical humidity would scintillate, amplified by the
closure's slope of 2.5. Measured at one sample per step over six hours, the step-to-step change in
the observer column's cloud fraction is 0.12 % of its value with the white seed and 0.19 % with the
correlated one. There was no scintillation to remove: the cloud fraction is a function of the
*state*, and the state integrates the flux over thousands of steps, which is precisely the
averaging that also made the white seed useless. The prediction was wrong and the change is worth
making anyway, for the reason the table gives instead.

#### What it does now, measured

With all of it in, over eleven simulated hours from sunrise, a quiescent parent airmass and no
front:

| land cover | W/m² | Bowen | mean sky coverage | cloud base | spurious fog |
|---|---|---|---|---|---|
| semi-desert / bare soil *(the default)* | 140 / 100 | 1.40 | 0.00 | — | none |
| mixed cropland | 130 / 180 | 0.72 | 0.04 | 1 341 m | none |
| vegetated summer land | 120 / 250 | 0.48 | **0.16** | **1 341 m** | none |
| open water / marsh | 60 / 320 | 0.19 | 0.45 | 700–900 m | none |

The third row is the phase's acceptance bar: a scattered fair-weather cumulus deck at the
mixed-layer top, 16 % of the sky, forming out of nothing but surface heating. The last is
stratocumulus — moist, barely buoyant, low base that stays low — which is what an almost entirely
latent surface should give and is a different genus rather than more of the same. The first row is
the same model told the ground is a semi-desert, and it is *correct*: a Bowen ratio of 1.4 heats
the air far faster than it moistens it, so relative humidity falls all day and the condensation
level runs away upward. Every row is now free of the ground fog that used to be the only thing any
of them produced.

**The ratio is authored, not defaulted.** Which of these a scene stands on is a scene-authoring
decision and the engine has no way to guess it, so the default is left where it is and the choice
is made where it belongs: the Meteorology panel carries the four covers as one-click presets with
the sky each produces in its tooltip, and displays the resulting Bowen ratio, flagged above 1.

#### Where to resume

- **`humidity_scale_height` does not do what it is documented to do, and the airmass is drier
  than it should be because of it.** The field is described as "the e-folding height of the
  base-state *vapour* profile"; `atmosphere_base_vapour` applies the exponential to *relative
  humidity* instead, so `q_v = RH₀·e^(−z/H)·q_s(z)` decays twice — once through the exponential
  and again through `q_s`. A real sounding has the mixing ratio decaying with a ~2.5 km scale
  height while `q_s` falls faster, so relative humidity *rises* through a moist layer; here it
  can only fall, which is why the free troposphere sits at 40 % and why every configuration that
  makes cloud has to be pushed there with surface humidity and a low Bowen ratio. Measured: at
  1 341 m the base state gives RH 0.41, against ~0.62 for the profile the doc describes.
  Deliberately **not** fixed — it changes the look of every existing scene, and that is an
  authoring decision. It is the first thing to reach for if the tier still reads as too dry.
- **A lifted cloud base cannot be an initial condition**, and follows from the above: base-state
  relative humidity is monotone decreasing by construction, so the saturated part of a fresh
  column is always the part touching the ground. An instantly-cloudy scene is a ground-based
  layer whose base then rises as the surface warms. That is not a limitation to remove — a cloud
  base is what the model *produces* by lifting moisture to it — but it is worth stating, because
  "give me a config with clouds already in it" is a reasonable request with a specific answer:
  raise `surface_humidity` to ~0.95 and pick a low-Bowen land cover, which reaches 100 % of
  columns cloudy within ten simulated minutes. The authored deck stack (nest off) remains the
  zero-wait path and is what it is for.
- **The deck forms late** — mid-to-late afternoon rather than late morning. The mixed layer has to
  deepen to ~1 300 m before its top saturates, and the diagnosed depth grows with the surface
  parcel's excess, so the timing is a consequence of the parcel criterion and the heating rate. Not
  wrong, but worth measuring against a real sounding before it is called right.
- ~~**The seed is still white in space.**~~ Closed 2026-07-28 — see *The symmetry break was doing a
  fifth of its job* above. The predicted scintillation turned out not to exist; what the seed was
  actually failing to do was break the symmetry at all, and that is measured rather than argued.
- **The step is now profiled, and the measurement reordered the levers it was supposed to
  choose between.** Timestamps bracket each stage of the step, resolved where the nest already
  waits on the submission's timeline value, so measuring stalls nothing; `atmosphere_probe`
  reports the mean over a run and the Meteorology panel carries the same breakdown. Measured on a
  GTX 1060 6 GB at the shipped tier (192×192×48, 2 km, twelve sweeps), mean over 1 471 steps:

  | stage | ms, as found | ms, after this section's two corrections |
  |---|---|---|
  | **pressure** | **7.71** | **2.74** |
  | advect scalars | 1.38 | 1.47 |
  | advect wind | 1.37 | 1.47 |
  | forces | 0.78 | 0.82 |
  | microphysics | 0.48 | 0.50 |
  | project | 0.34 | 0.37 |
  | extinction | 0.27 | 0.29 |
  | divergence | 0.20 | 0.21 |
  | readback | 0.06 | 0.07 |
  | **whole submission** | **12.60** | **7.95** |

  (The right column's non-pressure stages read slightly higher because they are a larger share of
  a shorter submission and the device clocks accordingly; they are the same work.)

  Three things follow, and the third is the one that matters.

  *The pressure solve is the large share its own header supposed*, and two things came out of
  looking at why.

  The tridiagonal matrix is a function of the level index alone — base-state density, potential
  temperature and the stretched grid's spacings do not vary horizontally — so its Thomas
  factorization is identical for every column. It is now built once per workgroup into shared
  memory instead of once per column into a dynamically indexed 64-float private array that
  spilled to local memory. Same arithmetic, hoisted: 8.90 → 7.71 ms, physics bit-identical.

  Then the sweep count itself, which had never been measured. **Twelve was five sweeps of pure
  waste and the reasoning behind it was misapplied.** The justification was that a solver with no
  coarse-grid correction leaves smooth horizontal error behind — a real property of an
  *isotropic* Poisson problem, and very nearly irrelevant here. The anisotropy that motivates the
  line solver in the first place also makes what remains for the sweeps a small, strongly
  diagonally dominant correction: horizontal coupling is 1/12 of the vertical at the domain top
  and 1/1370 of it at the ground. Measured end-to-end over six simulated hours at 2, 4, 8, 12 and
  20 sweeps, the surface humidity, lifting condensation level, cloud base, column water and sky
  coverage are **identical to every printed figure**, and the peak divergence — the quantity the
  solve exists to control — agrees to **seven significant figures**. At *one* sweep it does not,
  which is what tells convergence apart from a solve that was never doing anything. Repeated
  under a 25 m/s front with a +4 K parent anomaly, the case the argument is weakest for:
  divergence still agrees to six figures at four sweeps.

  `pressure_iterations` therefore defaults to **4** — twice the measured convergence point. The
  stage runs 1.3 / 2.7 / 8.0 / 15.3 ms at 2 / 4 / 12 / 20 sweeps, so twelve was spending 5.3 ms a
  step to move the sixth significant figure. With both corrections the step is **7.95 ms**, down
  from 13.97 as first measured — and the multigrid the shader's header names is no longer a
  refinement worth reaching for at this aspect ratio, because there is no residual left for it to
  remove. It becomes interesting only if the domain is ever made closer to isotropic.

  *And the step itself was three times shorter than anything the model needed.* `choose_step`
  took the vertical CFL against a flat `10 × convective_velocity_scale` — a thunderstorm core,
  20 m/s — which pinned Δt at 2.43 s in every airmass and on every grid, three times tighter than
  the horizontal term ever came to. Measured, a fair-weather domain peaks at **0.02 m/s** and a
  convecting one at 0.03: the assumption was three orders out. And because the transport is
  semi-Lagrangian and therefore *unconditionally stable*, what that bought was not stability but a
  vertical Courant number of 9 × 10⁻⁴ — §1.4's maximally diffusive regime, the exact defect this
  nest exists to have left behind. The short step was making the vertical advection worse, not
  safer.

  Two things were wrong at once. `convective_velocity_scale` is documented as "purely a
  *reporting* scale", and was in fact also setting the time step — one number doing two unrelated
  jobs, with the second invisible from where it is declared. The CFL now binds against the
  **updraft the readback measures**, with a fourfold headroom for what convection can do between
  readbacks and the old constant standing until the first readback lands; it is tracked with a
  decay so it tightens the instant convection strengthens and only relaxes slowly. Measured at
  2.44 / 5 / 6 / 10 / 20 s in both a quiescent and a convecting airmass: unchanged through 6 s
  (sky coverage within 3 %, cloud base within 2 %), and at 10 s and beyond the domain peak
  vertical velocity inflates by an order of magnitude and the sky changes with it — so
  `max_step_seconds`, already 6, is the cap that keeps it the safe side. In practice Δt now sits
  at 6 s until a real updraft exceeds ~2 m/s.

  **Which is the change that matters, because the tier's cost is not the step — it is the step
  over the weather the step buys.**

  | | as found | now |
  |---|---|---|
  | cost per step | 13.97 ms | 7.64 ms |
  | seconds of weather per step | 2.43 | 6.0 |
  | **cost per simulated second** | **5.75 ms** | **1.27 ms** |

  *Over budget per step is nonetheless a twentieth of a millisecond per frame.* A step falls due
  every 6 s of game time, so at 1× time scale and 60 Hz the tier costs **0.021 ms** — under one
  percent of §12's whole 2.6 ms frame budget. §12's per-step figure was optimistic on a 2016 card
  and its per-frame figure was right.

  The corollary is worth stating because it is what an editor session actually feels: the
  per-frame cost is *linear in the sky's animation rate*. At 60× real time the tier costs
  1.27 ms a frame; it crosses 2 ms at about **94×**; the 188× session that prompted this section
  pays 4.0 ms, against the 16 ms it paid before. That is not the tier being heavy — it is the
  amount of weather being ordered, and the price per unit of it is now 4.5× lower.

  **What a 2 ms *per step* would take, stated so it is a decision rather than a discovery.** Not
  tuning: the remaining step is 2.6 ms of pressure, 2.9 ms of MacCormack transport and 1.6 ms of
  everything else, and none of it is waste. It would take roughly four times fewer cells than
  192×192×48 — around 128×128×28 — which means either a coarser vertical or a spacing above 2 km.
  2 km is the spacing at which convection stops being parameterized and starts being *resolved*
  (§2.2), and it is what this phase's acceptance bar rests on, so that is a tier decision (§6's
  Medium/High/Ultra ladder) and not a default to move quietly.

  *So the defect was never the cost; it was where the cost landed.* The 12.6 ms fell inside a
  single frame — one dropped frame every 2.4 s at 1× time scale, and far worse once defect 7
  above was fixed and a frame could legitimately record four steps. Async compute alone would not
  have helped: `vulkan_scene_view.cpp` had each view wait on the step submitted *this* frame, so
  the frame was serialised behind it whatever queue it ran on. The step is now **staged during
  the frame and submitted at its end**, after every scene view — so a frame reads the step before
  it and waits on a value that has almost always already passed. The step waits on the frame's
  readers in turn, because submission order on a queue orders the *start* of submissions and
  promises nothing about one completing before the next begins. One nest step of staleness, a
  couple of seconds of game time, against a medium whose own time scale is minutes.

  Still open, in order: fewer sweeps or a tiled pressure smoother; the **async compute queue**,
  which the engine already has a seam for (`VulkanDevice::share_across_queues`) and which is now
  a straightforward win rather than a blocked one; and the semi-coarsened multigrid
  `atmosphere_pressure.comp`'s header names. **The editor path of the frame-end move has been
  built and its tests pass, but has not been confirmed by eye.**
- **SushiRuntime is the wrong tool for this tier, and the reason is worth recording** so it is not
  re-proposed. The nest's output is a Vulkan 3D image the cloud bake samples with hardware
  filtering; it is a *render resource* that happens to be computed. Moving the arithmetic to
  SYCL/USM would not make it faster — it is the same device — but it would add an interop
  boundary or a copy of the 14 MB extinction volume every step, and SushiRuntime is
  throughput-oriented and blocking, with no real-time thread class and no async step, which is
  precisely the property §3.2's whole concurrency story rests on. Its value is in the
  deterministic simulation domain, where the data does not have to become something the
  rasteriser samples.

### Phase B3 — Surface energy balance and ice — **shipped**; its open clause closed in C1

The other half of Phase B's original acceptance bar. Surface fluxes were prescribed constants; a
real energy balance — insolation through the ephemeris, a slab heat capacity, land/sea
partitioning — is what turns them into the diurnal cycle. Ice microphysics (deposition, freezing,
snow with its own fall speed) joins Kessler here.

*Acceptance: morning clear → midday cumulus → evening decay happens without anything scripting
it.*

#### B3a — the ground became a state variable — **shipped**

Three authored numbers went away: a peak sensible flux, a peak latent flux, and a night-time
cooling rate, each scaled by the sine of the sun's elevation. That is a *prescribed diurnal shape*
wearing the costume of a diurnal cycle, and the three things missing from it are the three things
a diurnal cycle is:

* **No lag.** With no heat capacity the fluxes peaked exactly at solar noon.
* **No response.** The ground could not be warmer or cooler than what an author typed, so as the
  air warmed underneath it nothing reduced the flux into it — and that negative feedback is what
  makes a boundary layer settle at a depth instead of growing all afternoon.
* **No changing Bowen ratio.** A surface drying out kept moistening the air at the rate it had
  when it was wet.

What replaces them is the textbook slab, one dispatch per column ahead of the forces stage:

```
C dT_s/dt = S(1-a) + L_down - e sigma T_s^4  -  H  -  LE
H  = rho c_p C_H |U| (T_s - T_air)
LE = rho L   C_H |U| beta (q_s(T_s) - q_v_air)
```

`atmosphere_surface.comp`, 37 000 invocations against the 1.8 million every 3-D stage runs, and it
measures **0.069 ms — 0.7 % of the step.** The skin temperature is prognostic, so it joins the
shift: a nest that walks two kilometres brings the ground's warmth with it rather than laying a
cold strip along its leading edge.

**Measured, on a 24 h cycle over vegetated summer land:**

| | |
|---|---|
| skin maximum | **+90 min after solar noon**, 36.4 °C |
| sensible flux maximum | +90 min, 197 W/m² |
| sensible flux turns positive | at solar elevation sine **+0.500** — long after sunrise |
| sensible flux turns negative | at sine **+0.065** — before sunset |
| Bowen ratio through the day | **0.50 → −6.25** |

Every row of that is emergent. The 90-minute lag is the slab's heat capacity; the morning
transition waits for the ground to overtake the air rather than for the sun to rise; the evening
one happens while the sun is still up, which is exactly when real convection stops. The Bowen
ratio is now *reported* rather than authored, and it moves.

Three decisions worth stating, because each replaced something that looked simpler:

**The slab is integrated semi-implicitly.** Every flux stiffens as the skin warms — longwave as
4εσT³, sensible linearly, latent through the Clausius–Clapeyron slope — some 85 W/m²/K against a
soil slab of 10⁵ J/m²/K, a 20-minute relaxation time. Explicit would need a step under twice that,
so an author choosing a thin slab (a road, a rock face) would walk into an oscillation that reads
as a physics bug and is an integrator bug. Linearising about the current skin costs one divide,
is unconditionally stable at any step length: the increment is the residual over `C + dt·λ` rather
than over `C`, so the step cannot outrun the response that opposes it.

Two weaker claims than this paragraph first made, and the unit test is why. It is **not** exact in
one step — `q_s` is exponential in the skin, so a step is one Newton iteration — and it is **not**
monotone: a single 10⁶ s step onto a 10³ J/m²/K slab overshoots the equilibrium by 3.3 K on a 16 K
approach, then converges from the other side within a few more. The residual falls every step after
the first and lands on the same equilibrium a patient loop finds. What the scheme buys is the
denominator: the explicit form is the same numerator over `C` alone, which at that slab and that
step is five orders of magnitude larger — not a large error, not a temperature.

**The exchange velocity is not |U|.** A bulk flux proportional to the mean wind vanishes on a calm
morning, which is precisely when free convection carries the most heat — the model has a stable
fixed point at "no wind, no flux, no convection, no wind". Beljaars (1995) adds the convective
velocity scale in quadrature; measured without it, a fixed overhead sun drove the skin **23 K**
above the 54 m level it was exchanging with, because 0.005 × 1 m/s is 6 W/m²/K and moving 144 W/m²
through 6 W/m²/K takes 24 K.

**Downwelling longwave is Brutsaert's, not a constant.** It goes as the seventh root of vapour
pressure over temperature, so a humid night radiates far more back down than a dry one — which is
why a desert freezes after dark and a coast does not. The retired `surface_night_flux` had to fake
that with a number.

*Named limit.* `C_H` is the neutral value: a stability correction belongs with the Monin–Obukhov
length, and this model has no surface layer resolved well enough to earn one. The free-convection
velocity above is the standard cheap substitute and is not the same thing.

*What this did **not** fix, measured.* Phase B's acceptance bar is not met yet, and the surface
balance is not what is standing in the way. Over 24 h the sequence is morning clear → **midday
still clear** → a deck forming at dusk → that deck sitting there all night, unchanged to three
figures. Two separate causes, neither in this section:

1. *Cloud forms late* because surface relative humidity **falls** all day (69.5 % → 39 %) while the
   LCL climbs 897 → 1585 m. That is `humidity_scale_height` (recorded above, deliberately not
   fixed): the base state's relative humidity can only decrease with height, so every configuration
   that makes cloud has to be pushed there.
2. *Cloud never decays* because a saturated layer at 1585 m has **no sink in this model at all**
   overnight — it is too thin to rain, the mixed layer that fed it has collapsed so nothing mixes
   it, and there is no radiative or subsidence drying. B3b's radiative terms are where that
   belongs.

#### B3b — cloud shades the ground it came from — **shipped**

The negative feedback that closes the convective loop, and the only one the tier did not have: a
cumulus forms, its own shadow cuts the heating that lifted it, the updraft weakens. Without it a
heated afternoon has nothing telling it to stop.

The column optical depth is accumulated by **the extinction stage**, not the surface stage, and the
reason is arithmetic: a column walk is 48 fetches, extinction runs once a *frame* and the surface
balance once a *step*. It costs 0.10 ms there (0.377 → 0.480) against roughly 3 ms of per-step
walking, and buys one frame of staleness — the ground is shaded by the cloud the previous frame
ended with, which at a couple of seconds of game time per step is nothing a cloud field notices.

Cover and depth are carried separately, for the same reason the readback separates them: a
tenth-covered column must let nine tenths of the sun straight through, not attenuate all of it by a
tenth of the optical depth. Overlap is **maximum**, which is right for a convecting column and
wrong for a frontal sheet.

**Measured**, the same run with the shading multiply removed and restored — a moist vegetated day
under a 24 h sun, cloud present from dawn:

| solar sine | skin, unshaded | skin, shaded | sensible, unshaded | shaded | domain peak w |
|---|---|---|---|---|---|
| 0.707 (morning) | 20.3 °C | 17.2 °C | 50 W/m² | 20 W/m² | |
| 0.966 | 31.7 °C | 25.4 °C | 157 | 97 | |
| **1.000 (noon)** | **32.5 °C** | **26.8 °C** | **159** | **106** | |
| 0.966 (afternoon) | 34.3 °C | 28.0 °C | 176 | 113 | 0.053 → **0.029 m/s** |

**5.7 K of skin and a third of the sensible flux, at a third-covered sky.** Column water after 14 h
runs 18.83 kg/m² unshaded against 17.03 shaded, and the peak updraft is nearly halved — the model
was running an afternoon that fed itself.

*Named limit.* The slant path is `1/μ` through the **same** column, so every shadow falls directly
under its cloud. That is right at a high sun and wrong at a low one, where a shadow really lands
kilometres downsun — but the error grows exactly as the flux being shaded shrinks, which is what
makes it affordable. A correct one is a shadow march across neighbours and is not worth a march.

**The other half of this phase's task was a wrong prediction, and it is recorded rather than
quietly dropped.** The task predicted a warm drift needing a free-tropospheric radiative cooling
term. Measured over **72 h** of diurnal cycling, mean θ′:

| | 12 h | 24 h | 48 h | 72 h |
|---|---|---|---|---|
| above 3 km | −0.04 | −0.02 | −0.08 | **−0.25 K** |
| above 10 km | −0.04 | +0.08 | −0.21 | **−0.86 K** |
| lowest level | +4.66 | +3.11 | +2.04 | **+0.71 K** |

There is no warm drift; the boundary layer trends *down* across three days and the free troposphere
cools slowly. So the term is **not added** — it would have been a scheme fixing a problem the model
does not have. The slow cooling that is there is concentrated above 10 km, straddling the
tropopause, where the base state's θ gradient is sharpest: that is numerical diffusion in the
semi-Lagrangian transport across a stiff gradient, at −0.29 K/day, and a radiative scheme would
not fix it. Recorded as a named limit rather than masked. *(The diffusion attribution was
withdrawn 2026-07-31: with the transport stepping alone the drift is 0.02 K over the same 72 h.
It is flow-driven — gravity waves accumulating under the sponge's lower edge — and was closed by
moving the sponge edge; see the 2026-08-01 entries at the top of §11.)*

#### B3c — the land/sea seam, deliberately not built — **decided**

B3's charter names land/sea partitioning, and this is the section that does not deliver it, with the
reason rather than an apology.

**Nothing in the engine can produce a land/sea mask.** Phase D is blocked (§15) on the terrain
height field the engine does not have, and a procedurally invented coastline would be *worse* than
none: the renderer would draw land where the atmosphere believes there is sea. That is a lie about
the world, not a missing feature. §16's own rule — these seams "arrive with implementations rather
than as stubs" — applied to itself. `AtmosphereSurface` is blocked on exactly what Phase D is
blocked on, and lands with it.

What was built and measured instead was the physics half: letting the patchiness field mean *soil
moisture* and drive the moisture availability as well as the darkness of the ground, so neighbouring
patches would differ in their **Bowen ratio** — the contrast that drives a sea breeze, at the scale
a 6 km patch can manage.

**It made things worse, and it is not shipped.** Domain coverage roughness over hours 3–6:

| patchiness drives | roughness | condensate | peak w |
|---|---|---|---|
| **absorbed shortwave only (shipped)** | **0.0678** | 0.0186 | 0.032 |
| shortwave and moisture together | 0.0519 | 0.0194 | 0.033 |
| moisture only | 0.0409 | 0.0213 | 0.036 |

The reason is visible once measured and was not before: a wetter patch absorbs *more* and evaporates
*more*, so the two effects cancel in the skin temperature — and it is the **thermal** contrast that
organises convection, not the moisture contrast. Structure falls 40 % for 15 % more condensate. The
story was good and the measurement disagreed with it; the measurement wins.

#### B3d — ice, as a diagnosed phase rather than a second species — **shipped**

The scheme is a **phase partition on temperature**, not cloud ice and snow as prognostic fields,
and the choice is the whole design:

> Carrying them costs another `rgba32f` volume — 28 MB at the shipped tier — plus its share of the
> advection, the boundary-layer mixing, the shift and the readback, and it buys mixed-phase
> *coexistence*: liquid and ice in the same cell with a transfer rate between them. At 2 km that
> coexistence is entirely subgrid. What is resolvable is that a colder cell saturates at a lower
> humidity, releases more latent heat when it condenses, precipitates more readily, and drops
> something that falls at a metre a second rather than seven — and all four are functions of the
> temperature the cell already carries.

So `nest_ice_fraction` ramps from 0 at 0 °C to 1 at −20 °C, and everything else is a function of
it: the saturation curve (liquid and ice Magnus blended), the latent heat (vaporization plus the
ice share of fusion — 13 % more), the autoconversion threshold (a quarter, once glaciated), the
fall speed (rain's and snow's blended), and the **effective radius** the extinction divides by.

**It costs 0.5 % of the step.** Measured by interleaving ice-on and ice-off builds three times in
one session, because the GPU's clocks drifted between runs by 20 % and a straight before/after
would have measured the clocks:

| | microphysics | extinction | whole submission |
|---|---|---|---|
| ratio ice-on / ice-off, mean of 3 interleaved pairs | **1.007** | **1.034** | **1.005** |

The first draft of the extinction path cost twice that, because it computed `(θ_base + θ′)·Π` per
level, which is two Exner evaluations — each a `pow` chain. `T_base + θ′·Π` is the same number with
one, and the column loop runs it 48 times.

**Measured behaviour**, same forcing, three airmasses, autoconversion lowered so a thin deck
precipitates:

| surface | freezing level | mean coverage | condensate | precipitation held / cloud |
|---|---|---|---|---|
| 288 K | 2 308 m | 0.430 | 0.1045 kg/m² | 0.051 |
| 276 K | 438 m | 0.496 | 0.0781 | 0.045 |
| **265 K** | below ground | **0.621** | 0.0618 | **0.177** |

Two signatures, and both are the ice curve rather than a tuning constant. **Coverage rises as the
airmass cools while the condensate falls** — colder air holds less water, so the cloud is thinner,
and yet more of the sky is covered, because the ice curve it condenses against sits well below the
liquid one. And the cold column holds **3.5× the precipitation per unit cloud**, which is the
slower fall speed and the lower glaciated threshold together.

**The warm case is bit-identical to before any of this existed** — coverage, condensate and domain
roughness agree to four decimals — which is the property that makes the partition adoptable: above
the freezing point the blended relations are *exactly* the liquid ones, so every measurement this
phase took earlier still stands. It is pinned by unit test rather than left to inspection.

Melting needs no term: a flake falling into air above freezing arrives in a cell whose ice fraction
is zero, so it falls at rain's speed and evaporates on rain's curve from that level down.

*Named limits.* No supercooled water below the glaciation point, no Bergeron transfer *timescale*
(only its consequence), and melting is instantaneous where it really takes a few hundred metres.
The two-species scheme is the refinement and its cost is quantified above. **Snow reaching the
ground was not demonstrated in a free run**: this configuration cannot build a cloud deep enough to
precipitate on its own, so the precipitating measurements above have the autoconversion threshold
lowered by hand. The fall-speed blend and the saturation curves are pinned by unit test instead.

Also measured, and the reason B3b's third item exists: **the nocturnal deck has no sink at all**.
A saturated layer at 1585 m is too thin to rain, the mixed layer that fed it has collapsed so
nothing mixes it, and there is no subsidence — so it sits unchanged to three figures until dawn.
Radiative cooling would make it *thicker*. The honest candidate is large-scale subsidence off the
parent solution, which is Phase C's to supply and is left there.

#### B3e — the acceptance bar, run: **two clauses of three**

*Acceptance: morning clear → midday cumulus → evening decay happens without anything scripting it.*

Run over 24 h of diurnal cycling, vegetated summer land (albedo 0.18, moisture availability 0.55,
1.5×10⁵ J/m²/K), surface humidity 0.85, with a westerly parent carrying a −0.15 relative-humidity
anomaly at 8 m/s:

| hour | sun | coverage | cloud base | cloud top | skin | what it is |
|---|---|---|---|---|---|---|
| 00:00 | 0.00 | 0.000 | — | — | 15.0 °C | clear |
| 01:30 | +0.38 | 0.163 | 19 m | 99 m | 12.9 | **radiation fog** |
| 03:00 | +0.71 | 0.287 | 19 m | 99 m | 14.7 | fog deepening |
| **04:30** | +0.92 | **0.000** | — | — | 20.1 | **burns off** |
| 06:00 | +1.00 | 0.000 | — | — | 20.9 | clear at solar noon |
| 07:30 | +0.92 | 0.000 | — | — | 22.8 | skin peak, +90 min |
| **09:00** | +0.71 | 0.093 | **1 112 m** | 1 341 m | 20.1 | **cumulus at the mixed-layer top** |
| 10:30 | +0.38 | 0.207 | 897 m | 1 341 m | 16.1 | deepening |
| 12:00 | 0.00 | 0.194 | 19 m | 1 341 m | 13.8 | fog under the deck |
| 18:00 | −1.00 | 0.637 | 19 m | 1 341 m | 10.8 | overnight deck |

**What is emergent, and was not before B3:** a *radiation fog* that forms because the ground
radiates to a cold sky and cools below the dew point, and then **burns off at mid-morning** when
the sensible flux turns positive and lifts it. Neither end of that was reachable with a prescribed
night flux, because the old constant had no idea how humid the air above it was — and both are the
Brutsaert downwelling term and the prognostic skin doing exactly what they were added for. The
same ground fog was a *defect* in Phase B2c; here it has a life cycle.

**Clause by clause.** Morning clear: **yes**, 04:30 through 07:30. Midday cumulus: **partly** — a
proper fair-weather deck forms at the mixed-layer top with a base over a kilometre up, but at
09:00 rather than at noon. Evening decay: **no** — the deck grows through the night instead.

> **Closed in Phase C1.** With the parent supplying subsidence the deck peaks at sunset and falls
> 40 % overnight instead of growing. The diagnosis below was right, and the term that fixes it
> came from a field `SynopticLayer` was already computing.

**Why, measured rather than guessed.** The same run against a quiescent parent was taken as a
control: the closed box peaks at a 35.3 °C skin instead of 22.8 °C and gains water monotonically
(13.31 → 16.55 kg/m²) with coverage reaching 0.61 and never falling. The advected run does lose
water in places (14.75 → 14.04 overnight) — so horizontal advection *is* a sink, and it is far too
slow: 8 m/s across a 384 km domain is thirteen hours, against a night. The deck outlives it.

That leaves the diagnosis from B3b standing and now tested from a second direction: **the missing
sink is large-scale subsidence**, which warms and dries by compression on the timescale a night
actually has, and which a limited-area model does not generate — it receives it from its parent.
That is Phase C's to supply, and the acceptance bar's third clause is carried there with it rather
than being chased with a term invented inside the nest to stand in for it.

The late timing has its own separate cause, already recorded: surface relative humidity **falls**
all day because `humidity_scale_height` does not do what its documentation says, so a column has to
be pushed to its condensation level rather than starting near it.

### Phase C — Global core and climatology — **shipped**

Closed in four steps: **C1** brought the parent's large-scale vertical motion; **C2** built the
two-layer quasi-geostrophic core standing alone on analytic latitude bands; **C3** made it the
weather, deleting `SynopticLayer` and retargeting the nest, the editor, the gameplay wind and the
scene file onto it; and **T0** replaced the analytic bands with baked data (§4, §4.1, §4.2).

What the phase set out to do was make the weather *emerge* rather than be authored, and that is
measurable rather than asserted: the core grows eddies exponentially with nothing placing a low,
runs a complete baroclinic life cycle, puts its jet at 29–31°N because that is where January's
subtropical jet is in the reanalysis it relaxes toward, and moves that jet with the season —
equatorward into the southern winter and merely weaker into the northern one, which is a
hemispheric asymmetry nothing in the engine encodes.

One limit is carried out of the phase rather than hidden by it, and it is named where it is
measured: the moisture cycles too slowly (§4.1). It does not block Phase D or E. The other
long-standing open question — that an authored disturbance was about three times deeper than a
natural one — was closed rather than carried: it turned out to be a monopole injection, not a
tuning problem (§11's C3 notes).

#### C1 — large-scale vertical motion, and the clause B3e carried here — **shipped**

B3e closed with one acceptance clause open and a named cause: an evening deck grew through the
night instead of decaying, because a saturated layer aloft has no sink at all — too thin to rain,
the mixed layer that fed it collapsed, and nothing dries it. The candidate was **large-scale
subsidence**, which a limited-area model receives from its parent rather than generating. This
section is that candidate, built and tested.

**It is the first term the nest receives that is not a boundary condition.** The three parent
anomalies feed the Davies zone; this is applied across the whole domain, because a 384 km window
has no way to know it is sitting under a thousand-kilometre high. In `atmosphere_forces.comp` it is
vertical advection of potential temperature and vapour against the *total* profile — subsidence
warms by compression and dries by bringing down air from where there is less water — with a shape
that rises from zero at the ground to the supplied value near the boundary-layer top and decays to
zero at the tropopause.

**Where it comes from: Ekman pumping, and `SynopticLayer` already had it.** Its geostrophic wind
carries a 25° surface-friction turn that relaxes with altitude, and *that turn is the
cross-isobaric flow*. Air spirals inward toward a low and outward from a high, so the convergence
has to go somewhere and where it goes is up. Taken as the divergence of the near-surface wind over
the Ekman layer, `w = −h·∇·V`, numerically from four neighbouring samples rather than analytically
— the analytic wind already *has* the friction in it, and re-deriving the divergence in closed form
would be a second expression of the same thing, free to drift from the first. Measured: **1.0 cm/s
in the mean and ±5.5 cm/s at the centres of a 20 hPa pair**, which is the textbook synoptic value,
and it falls out of the friction angle rather than out of a constant chosen to produce it.

**The acceptance re-run.** Same 24 h diurnal case as B3e, mean sky coverage:

| hour | sun | no subsidence | −0.5 cm/s | −1 cm/s | −2 cm/s |
|---|---|---|---|---|---|
| 01:30 | +0.38 | 0.126 | 0.125 | 0.123 | 0.119 |
| 04:30 | +0.92 | 0.033 | 0.014 | 0.010 | 0.010 |
| 07:30 | +0.92 | 0.128 | 0.071 | 0.026 | 0.019 |
| 10:30 | +0.38 | 0.503 | 0.446 | 0.360 | 0.119 |
| **12:00** | 0.00 | 0.592 | **0.524** | 0.425 | 0.183 |
| 15:00 | −0.71 | 0.599 | 0.429 | 0.303 | 0.126 |
| 18:00 | −1.00 | 0.604 | 0.353 | 0.274 | 0.092 |
| **24:00** | 0.00 | **0.614** | **0.315** | 0.248 | 0.140 |

Without subsidence the deck grows monotonically to dawn. With half a centimetre a second it peaks
at sunset and **falls 40 % overnight**. The column water says the same thing: 13.31 → 16.54 kg/m²
with no sink, against 13.31 → peak 15.58 → 14.93 with one. The sun switches off in both runs, so
the difference is the sink and nothing else — **the diagnosis was right**.

Phase B's bar now reads: morning clear ✓, midday cumulus ✓ *in kind* and still late in timing,
evening decay ✓. The remaining timing gap is `humidity_scale_height`, recorded above and
deliberately not fixed.

*Named limit.* Two centimetres a second suppresses the afternoon cumulus almost entirely (0.028 at
09:00). The value is not a tuning knob here — it is whatever the pressure field produces — but a
scene parked under a strong high will be clear, which is correct and worth knowing before it is
reported as a bug.

#### The synoptic wind was 735× too fast, and nothing had ever measured it

Found while deriving the pumping, where it showed as the vertical motion saturating its own 10 cm/s
cap *everywhere* — not a subtle symptom.

`V = ∇p/(ρf)` wants pressure in pascals; `pressure_gradient` returns hectopascals per metre. The
conversion is therefore 100/1.225 = **81.6**. The constant was **6.0 × 10⁴**, beside a comment
reading *"tuned so a ~30 hPa deepening low reads ~15-20 m/s"*. Measured, the shipped `FrontPassage`
preset produced a **15 534 m/s** boundary wind; after the fix it produces 26 m/s. The comment
described the intent exactly and the number never matched it.

**Why it survived**, which is the part worth generalising: the test covering this field asserted
that the wind was *non-uniform* — that a front had something to advect it — and a 15 km/s field
satisfies that perfectly. Structure was checked and magnitude never was. Both are now asserted, and
the scale is written as `PASCALS_PER_HECTOPASCAL / AIR_DENSITY` so there is no number left to tune.

#### C2 — the global dynamical core — **shipped**

`SynopticLayer` is not a simulation and never claimed to be: §1.3 records it translating authored
elliptical Gaussians across the sphere and diagnosing a wind from their summed gradient, so a low
can only ever do what it was told to do at genesis. This section is its replacement — §5's
two-layer moist quasi-geostrophic core, in the new engine-neutral `atmosphere/` module, with
`QuasiGeostrophicCore` as the tier and `FourierTransform` as the transform its inversion is built
on. **C2 was the core alone**: it did not yet feed the nest and `SynopticLayer` was still the
parent. C3 made that swap and deleted `SynopticLayer`; T0's real assets landed after it. Phase C
is closed — see §4.1, §4.2 and the phase status below.

**Where it runs, and the section this contradicts.** §3.3 planned T1 on the GPU beside T2 and T3.
That section has been corrected rather than quietly left: every consumer of this tier is
host-side, the device would have bought a step that is already nearly free and paid for it with a
readback, and the whole tier would have become untestable without a device. It is 36 ms of one
core per step at 512×256, one step per six minutes of game time.

**The inversion is the whole reason this tier is cheap, and it decouples exactly.** The layer
coupling enters the two potential-vorticity definitions with the same coefficient and opposite
signs, so the sum and the difference of the layers separate with no approximation: `laplacian(psi_1
+ psi_2)` for one and `(laplacian - 2S)(psi_1 - psi_2)` for the other. Each is then diagonal in
zonal wavenumber and tridiagonal in latitude — a Fourier transform per row and a pre-factored
Thomas sweep per wavenumber. The factorization depends only on the grid and the stratification,
neither of which changes, so it is computed once at construction instead of three times a step
forever.

**Three details that are worth carrying to any future core on this grid.**

- *The poles need no special case in the elliptic operator.* The grid is cell-centred, so its
  polar cell edges fall exactly on the poles where `cos(latitude)` is zero, and "no flux across
  the pole" is what the metric already says. The advective stencils do need to cross, and there
  the even longitude count makes a pole crossing an **exact index shift by half a revolution** —
  no interpolation, no reflection formula.
- *The zonal Laplacian uses the discrete eigenvalue, not `-m^2`.* The Jacobian, the Ekman drag and
  the vertical velocity all read a relative vorticity that this inversion produced. Using the
  continuous eigenvalue would leave those three reading a field that does not quite satisfy the
  equation they assume — a small, permanent, entirely avoidable inconsistency.
- *One filter does two jobs.* A latitude/longitude grid's zonal spacing shrinks as
  `cos(latitude)`, and near the pole it would set the time step for the whole globe. Cutting the
  wavenumbers that spacing cannot carry is the standard fix; making the cut-off clamp to the
  grid's own highest wavenumber equatorward of 60° turns the same filter into the grid-scale
  enstrophy sink a two-dimensional flow needs and does not otherwise have.

**Acceptance: cyclogenesis is emergent, and it equilibrates.** 512×256, 20 m/s shear, seeded with
a perturbation of 1 m/s — a thirtieth of the jet — and nothing else placed:

| day | eddy KE (J/m²) | zonal KE (J/m²) | jet (m/s) | peak wind (m/s) | lowest (hPa) |
|---|---|---|---|---|---|
| 0 | 859 | 704 069 | 29.98 | 30.04 | −36.06 |
| 6 | 1 843 | 524 735 | 26.83 | 28.13 | −22.38 |
| 12 | 11 715 | 472 522 | 25.95 | 29.26 | −17.48 |
| 20 | 69 686 | 483 020 | 29.98 | 42.09 | −21.50 |
| **24** | **51 931** | 485 279 | 31.32 | 40.07 | −21.91 |
| 39 | 113 956 | 437 284 | 28.24 | 45.59 | −24.61 |
| 48 | 68 343 | 432 923 | 29.18 | 42.73 | −22.89 |
| 70 | 84 957 | 417 431 | 26.78 | 39.91 | −25.39 |

The eddy energy grows exponentially — **0.26/day fitted over the whole growth phase, 0.32/day over
days 8–12 where it is purely exponential**, an amplitude e-folding of six to eight days —
saturates near 10⁵ J/m² against a zonal flow of 4.2×10⁵, and then runs a **15–20 day oscillation**:
eddies grow, flatten the shear that feeds them, decay, and the relaxation rebuilds it. That is the
baroclinic life cycle, and the fact that a 70-day run neither dies nor runs away is the half of the
acceptance a growth rate alone does not cover. Over the run the strongest wind reaches 49 m/s and
the deepest cyclone −31 hPa; day 0's −36 hPa is the prescribed jet's own field, not a storm.

*The fit window has to close as well as open.* An earlier version of the probe left it open for
the whole run, and because the equilibrated state spends as much time decaying as growing it
reported 0.047/day — a number that would have read as "no instability" for a core that is
demonstrably unstable. It now closes at the first sample where the energy falls.

**Why the growth is slower than the Eady estimate, and why that is the right answer.** A uniform
20 m/s shear at a 700 km deformation radius suggests an amplitude e-folding near two days. Three
things account for the six: the jet is *localized* (15° wide, so the mode has to fit inside it),
the state is only about 1.5 times critical once Ekman drag has spun the lower layer down —
Phillips' criterion puts the critical shear near `beta * L_d^2`, about 8 m/s here — and near a
threshold the growth rate falls off steeply. The relaxation and the Ekman drag then take about
0.2/day back out of the energy budget. Raising `upper_jet_speed_mps` is the knob if a scene wants
storms sooner.

**Resolution is not a preference here, it is the difference between weather and no weather.** At
64×32 the deformation radius is one cell and the run decays monotonically; at 256×128 it grows at
0.09/day; at 512×256 it grows at 0.32/day. The cause is the grid-scale damping, which is
calibrated against the two-cell wave: at 64 latitudes the most unstable mode *is* nearly the
two-cell wave and the damping eats it, and at 256 it is twelve cells across and the damping cannot
see it. The core's low tiers therefore cannot simply be the same physics on a coarser grid, and
`grid_scale_damping_seconds` is the parameter that has to move with them. **Named limit**, and the
unit test that exercises the instability at 64 latitudes lengthens the damping for exactly this
reason.

**Measured against references that are not itself.** `test_quasigeostrophic_core.cpp`: the fast
transform against a naive O(N²) one and the real-pair packing against two separate transforms;
the recovered streamfunction against the potential-vorticity relation written out a second time
in the test (residual under 10⁻⁶ of the field); the prescribed jet coming back through
streamfunction, vorticity, inversion and differencing at 30 m/s ± 0.6; free advection conserving
the domain integral of `q` to 10⁻⁴ and of `q²` to 2×10⁻³ over a simulated day; and an injected
vorticity blob drifting **northwest** at 19.7 m/s against Rossby's `beta * R²` of 23.3 — the beta
drift, in both of its components, from a core that was never told about it.

Every check that can assert a magnitude does. §11's C1 records the synoptic wind running 735 times
too fast for the entire life of the shipped system, surviving because the one test covering it
asserted that the field was non-uniform — which a 15 km/s field satisfies perfectly.

**Carried out of C2, deliberately:**

- **There is no tropical rain**, and the global mean precipitation is 0.25 mm/day against Earth's
  2.7. §5 already names the cause — a two-layer quasi-geostrophic model has no Hadley circulation,
  so the ITCZ has to be a *T0-forced* convergence band — and T0 is analytic latitude bands in C2.
  What does rain is mid-latitude frontal ascent, which is the part this formulation is entitled to.
  *(Superseded: with T0's baked climatology this reaches 1.5–2.4 mm/day. §4.1.)*
- **A 36 ms step is a visible hitch under time compression.** *(First measured at 47 ms in this
  phase; 36 ms is the figure the later status entries carry and is the one this document uses.)*
  At 1× it lands once per six minutes
  of wall clock and is invisible. At 60× it lands once per six seconds and is not. The step has no
  dependency on anything mid-frame, so moving it to a worker is straightforward; it is not done
  because nothing consumes the tier yet.
- **The moisture is transported by the lower layer's wind alone**, with the ascent-driven
  convergence added as a source. A two-layer model has one interior wind per layer and no profile
  to integrate against, so a column-mean transport would need a weighting invented for the purpose.
- **`inject_vorticity` exists on the core and nothing calls it.** It is §5's "place a low here"
  becoming an injected anomaly that then evolves, and wiring the editor to it is part of the swap
  below rather than part of the core.

#### Phase C3 — the swap — **shipped**

`SynopticLayer` is deleted. C2's core is the nest's parent: Davies relaxation fed from real
fields, and C1's Ekman-pumping estimate replaced by the core's quasi-geostrophic omega, which was
already diagnosed and unused. Editor authoring is vorticity injection. `Astro::Ephemeris` was
already the single solar authority — the nest reads the sine of the *rendered* sun's elevation
straight off the environment the ephemeris filled, so §1.6's "two different suns" defect was
closed in B2c rather than here; this phase only confirmed no second solar model survives.

*The hemisphere defect went out with the class, as planned.* `SynopticLayer::wind_at` applied the
surface-friction turn as the same rotation in both hemispheres; the core turns it correctly and a
unit test pins both signs.

**What the parent now hands the nest.** Three fields, with two deliberately different references:

- `thermal_anomaly_at` and `humidity_anomaly_at` are departures **from the zonal mean**, because
  the nest's own base state already carries the mean and what it cannot generate is the eddy.
  Handing it the absolute meridional gradient would tell a nest at 60° it is permanently 20 K
  cold.
- `frontal_strength_at` is the magnitude of the **total** thermal gradient, K per 100 km, because
  a front is a concentration of the real temperature gradient and removing the zonal mean first
  would erase the background baroclinic zone that frontogenesis concentrates. Threshold is the
  consumer's business: the background zone reads a few tenths, a real front 5 or more.
- `pressure_anomaly_hpa` **was corrected during this phase.** It returned `rho f0 psi_2` — the
  total — which is dominated by the mean jet's own meridional gradient: measured at 139 hPa across
  a 40-degree window, an order more than any cyclone. Reported that way the editor's map would
  have been a picture of latitude. It is now a departure from the zonal mean like its two
  siblings, and `QuasiGeostrophicDiagnostics::lowest_pressure_anomaly_hpa` was corrected with it
  so "the deepest low in the world" names a low rather than whichever cell sits furthest poleward.

**The editor's map samples rather than lists.** There is no `systems[]` left to iterate, so the
Weather panel shades `pressure_anomaly_hpa` (or `frontal_strength_at`) on a 44x44 lattice, draws
the low-level wind as a 9x9 arrow field over it, and injects an anomaly where clicked. Every
slider the old panel offered — depth, radius, speed, heading — set a quantity the core now
derives, and a slider that pretends to set a derived quantity is worse than no slider. What is
left is strictly less direct control and strictly more honest, because the previous control was
over a picture rather than over the weather.

**Persistence is a binary sidecar.** The state is two potential-vorticity fields and a moisture
field on a 512x256 grid — **1 572 920 bytes measured**, which is not going into a human-editable
scene file for a payload no human would edit. `capture`/`restore` write `q - f` as float so all
seven digits go to the varying part; the scene JSON keeps only `atmosphere_sidecar: true`, written
after the bytes are on disk. A missing or wrong-grid sidecar leaves the core as seeded rather than
failing the whole scene load. Round trip measured at 3e-8 hPa; a junk blob is rejected.

##### Measured: the front-passage sequence

`FrontPassage` at 45°N, sampled hourly at the observer (headless, `frontpass.cpp`):

| h | p' (hPa) | grad (K/100km) | wind E | wind N |
|---:|---:|---:|---:|---:|
| 0 | −89.98 | 1.02 | 8.6 | 20.0 |
| 12 | −33.56 | 1.06 | 22.4 | 21.1 |
| 24 | −21.60 | 1.26 | 25.8 | 7.0 |
| 36 | +8.07 | 1.24 | 21.5 | 8.9 |
| 44 | +29.20 | 1.27 | 20.3 | 6.8 |
| 56 | −1.11 | 1.29 | 14.6 | 5.0 |

The sequence is real and unscripted: the trough deepens overhead, passes, the ridge builds behind
it, and the wind **veers from north-northeast to nearly due west** across the passage while the
thermal gradient sharpens from 1.02 to 1.29 K/100 km. That is a front passage, measured, not
asserted.

##### Measured: emergent cyclogenesis, re-confirmed through the provider

`ProceduralWeather`'s own core, seeded and run 24 days with **nothing injected**:

| day | eddy KE (J/m²) | zonal KE | peak wind | rain (mm/day) |
|---:|---:|---:|---:|---:|
| 0 | 859 | 704 069 | 30.0 | 0.000 |
| 8 | 3 218 | 499 851 | 27.8 | 0.005 |
| 14 | 21 844 | 466 831 | 31.0 | 0.086 |
| 20 | 68 054 | 479 643 | 42.4 | 0.235 |
| 24 | 50 812 | 476 142 | 40.2 | 0.171 |

Exponential growth of **0.254/day over days 8–20**, matching C2's independently measured 0.26/day,
followed by the baroclinic life cycle's decay — eddy energy drawn out of the zonal flow (which
falls 704 000 → 466 000) and then returned. **The acceptance clause "cyclogenesis is emergent —
nothing places a low" is met**, and it is met through the shipped provider rather than only
through the probe.

> **A trap worth recording, because it cost real time here.** `advance()` is capped at
> `max_steps_per_advance` (8), so **one call can never cover more than 48 minutes of game time** —
> it deliberately refuses to let a long frame turn into an unbounded catch-up. A first measurement
> called `advance(2 days)` in a loop and concluded the core grew at 0.2 %/day and never rained,
> because it had actually simulated ten hours rather than 24 days. Drive it against
> `simulated_seconds()` in a `while` loop, not by handing it a large interval and trusting it.

##### The injection was three times too loud — found, diagnosed, fixed

**The symptom.** Measured at 128×64 right after injection, against the corrected pressure
reference, a click at the panel's defaults put **−70.6 hPa** over the map; `FrontPassage` −94.8,
`Storm` −110.9, `Clear` +61.9. A deep real low is −30 and the deepest ever recorded is near −50.

**The cause was not arithmetic.** The field was geostrophically self-consistent and 0.451 is the
correct peak-azimuthal-wind constant for a Gaussian vorticity blob. What made it deep is that
`inject_vorticity` placed a **monopole** — a blob with net circulation — whose streamfunction
grows *logarithmically outward* instead of decaying the way a compensated anomaly's does. A low
placed anywhere therefore tilted the pressure field of the entire hemisphere, and did it in a way
that still looked like a plausible synoptic pattern.

**The fix is the physics, not a smaller number.** Every injection now lays down a broader opposing
lobe carrying exactly the core's circulation, scaled by summing both lobes over the actual grid
rather than from a ratio of analytic integrals — so the cancellation is exact at any latitude,
including near the poles where the metric is nothing like a plane. The pair's peak wind is a fifth
below a lone blob's (the compensator opposes the core), so the wind constant was re-derived as
`0.369 · ζ₀R`; without that the parameter would have quietly meant something other than its name,
and a requested 20 m/s blew at 16.2.

**Then, and only then, the amplitudes.** With the far field no longer inflating everything, the
presets were re-measured and found to be asking for 26 and 34 m/s of rotational wind — hurricane
force at synoptic radii. Re-tuned to what a system of each kind actually reaches:

| what injects it | before | after (request → central `p'`) |
|---|---|---|
| editor click, defaults | −70.6 hPa | 700 km, 12 m/s → **−26.1 hPa** |
| `FrontPassage` | −94.8 hPa | 750 km, 15 m/s → **−34.7 hPa** |
| `Storm` | −110.9 hPa | 600 km, 23 m/s → **−44.1 hPa** |
| `Overcast` | — | 1 100 km, 8 m/s → **−24.8 hPa** |
| `Clear` (a ridge) | +61.9 hPa | 1 200 km, −8.5 m/s → **+28.5 hPa** |

And the far field, which is the point: the anomaly 60–120° away fell from *rivalling the centre*
to 2–5 hPa. The editor's strength slider had its ceiling lowered from 45 to 30 m/s, because past
about 30 at these radii it was offering an author a system the atmosphere has never made.

Four tests pin it: the anomaly is local, it blows at the speed it was asked for, it is a depth
that could exist, and its sign is right in both directions.

> **An earlier draft of this section claimed the other end was mis-scaled too — that a naturally
> mature cyclone reads only −6.9 hPa against a textbook −31. That was wrong and is withdrawn.**
> It was measured on the default 30 m/s jet, which sits barely above the Phillips threshold and
> grows slowly. On a properly supercritical jet the core reaches **−35.4 hPa by day 8** — a
> textbook deep low, with the eddy energy peaking and the zonal flow drawn down as it should.
> The anomaly scale is right; the injection was the loud part, and it has since been fixed above.

| day | deepest `p'` | eddy KE | zonal KE |
|---|---|---|---|
| 0 | −1.0 | 802 | 1 413 362 |
| 2 | −2.7 | 6 047 | 1 410 605 |
| 4 | −10.2 | 67 008 | 1 406 887 |
| 6 | −26.7 | 618 804 | 1 467 238 |
| 8 | **−35.4** | 829 649 | 1 909 833 |
| 10 | −35.7 | 478 214 | 1 686 831 |

*(128×64, 45 m/s jet, damping lengthened to match the coarse grid — the configuration
`CyclonesGrowOutOfTheMeanStateWithNothingPlacingThem` already uses. This is now pinned by
`TheEddyPressureAnomalyIsSynopticAndNotAstronomical`.)*

*Acceptance (Phase C overall): cyclogenesis is emergent — nothing places a low. The textbook
front-passage sequence occurs unscripted: cirrus, then altostratus, then continuous rain, wind
veering, a cold-frontal cumulus line with a gust, then clearing to scattered cumulus. Weather
1 000 km away has developed on its own by the time the player flies there.* **Both halves are
measured above and met at the dynamical level. The cloud-genus sequence itself belongs to the
nest and remains unchecked — T0 has since landed, and that check is a nest-side acceptance run
that has not been done rather than one that is blocked.**

### Phase D — Terrain and surface coupling *(blocked, §15)*

Orographic lift and rain shadows, föhn, valley fog, sea and lake breezes, terrain-driven
turbulence, land-cover-driven convective initiation.

*Acceptance: the windward slope is wet and the lee is dry, without either being authored.*

### Phase E — Data plane and flight-sim exposure

`AtmosphereProfile` and `AtmosphereDiagnostics` replace `WeatherColumn`; the query mirror
and the deterministic summary; retargeted `weather_wind` / hazards / world-coupling;
skew-T editor readout; `IngestedAtmosphere` (GRIB + the retained METAR path) blended into
the simulated state, X-Plane-style.

*Acceptance: icing, turbulence, freezing level, visibility, and a real wind profile are
queryable and correct against the simulated state; a real METAR reproduces its reported
sky.*

### Phase F — Microscale patch *(optional)*

A ~20 km, ~100 m LES domain around the player for hero storms, rotor turbulence, and
thermals — one domain, tier-gated, possibly never.

---

## 12. Performance budget (High tier, 1080p internal, mid-range GPU, 1× time scale)

Rewritten 2026-08-01 in the terms §11's B2c established: the honest metric for a simulation
tier is **cost per second of simulated weather**, because its per-frame cost is that number
times the sky's animation rate. Figures marked *measured* are on a GTX 1060 6 GB; the rest
are budgets not yet measured. (Measured 2026-08-01, back to back on one machine: the same
binary runs the T2 step at **12.3 ms in the GPU's sustained clock state and ~30 ms in its
low state**, stage shares constant — so per-step figures are only comparable within one
clock state, and the honest per-simulated-second range at Δt 6 s is **1.3–2.1 ms**. §11's
open item 3 carries the full numbers.)

| System | Per step | Cadence (game time) | Per simulated second | Per frame @60 Hz, 1× |
|---|---|---|---|---|
| T1 global core (CPU) | ~36 ms *(measured)* | 6 min | ~0.1 ms | negligible on average — but it lands inside one frame; moving it to a worker is a standing open item |
| T2 regional nest | ~8 ms *(measured)* | ~6 s (measured CFL) | 1.27 ms *(measured)* | ~0.02 ms |
| T3 cloudscape compile | ~1.0 ms (budget) | on T2 step / camera rebake | — | ~0.05 ms |
| Query mirror readback | ~0.07 ms *(measured)* | every T2 step | — | ~0.01 ms |
| T4 render (unchanged) | — | every frame | — | ≤2.5 ms |
| **Total** | | | | **≤2.6 ms** |

The simulation tiers' per-frame cost is linear in the time scale: ~1.3 ms per frame at 60×,
crossing 2 ms at about 94× (B2c). VRAM: ~135 MB for T2 at High (measured, fp32 — §6), ~40 MB
for T3's field set, ~4 MB for T1 and the mirror.

The headline stands: **at 1× the whole simulation costs less per frame than the cloud
composite**, for the reason given in §2.2.

---

## 13. SOLID

- **SRP** — one reason to change each: the dynamical core, the microphysics, the surface
  stage (whose shader carries the radiation terms — §3.5), the cloudscape compiler, the
  query mirror, and each render pass. §1.6's seven-responsibility class does not reappear.
- **OCP** — a new microphysics scheme, a new core (cubed-sphere, a different closure), or
  a per-biome parameter set is a new class or a data edit; surface and radiation vary
  through `AtmosphereParameters` rather than through swappable types (§3.5). Genus and
  every physical constant are data.
- **LSP** — `SimulatedAtmosphere`, `IngestedAtmosphere`, and `StaticAtmosphere` are
  interchangeable behind `IAtmosphereSource`, and — unlike the shipped seam — *installable*,
  which is the only form of the property that means anything.
- **ISP** — the renderer sees `IAtmosphereField` (handles), gameplay sees
  `IAtmosphereQuery` (profiles), the editor sees `IAtmosphereAuthoring` (an optional
  capability). No consumer can reach state it has no business touching, and the point-query
  bottleneck that caused §1.1 is structurally impossible.
- **DIP** — the host depends on `IAtmosphereSource`; no abstract interface names a
  concrete atmosphere type; Vulkan stays in `rhi/vulkan`; solar geometry is injected.

---

## 14. Named limits

Stated here so they are decisions rather than discoveries:

- **2 km is the nest's resolution.** A storm cell is simulated; a tornado funnel, hail
  growth, and lightning channel physics are below the grid and remain effects driven by
  simulated proxies.
- **Outside the nest, convection is parameterized**, not explicit — as it is in every
  operational global model.
- **Radiation is a grey/band-simplified scheme**, not RRTMG. Cloud radiative feedback on
  the dynamics is approximate.
- **No Hadley cell, no emergent tropical cyclogenesis** (§5). The ITCZ is T0-forced;
  tropical cyclones, if wanted, are seeded and then advected by real dynamics.
- **One-way nesting.** The nest cannot influence the global core. A squall line does not
  alter the parent low.
- **The global core's grid-scale damping does not scale itself with the grid** (§11's C2).
  It is calibrated against the two-cell wave, and at 512×256 the most unstable baroclinic mode
  is twelve cells across and is untouched; halve the latitude count twice and the mode *is*
  nearly the two-cell wave, the damping eats it, and the run decays instead of producing
  weather. A coarser tier therefore has to lengthen `grid_scale_damping_seconds` with it. This
  is a resolution below which the tier stops being a simulation, not a resolution at which it
  is merely less detailed.
- **Microphysics is single-moment.** Droplet number is parameterized, not predicted; no
  aerosol/chemistry transport.
- **No ocean coupling.** SST is climatology plus a slab, not a dynamic ocean.
- **No forecast skill is claimed or intended.**
- **Determinism is given up** (§0, §3.4), and with it `test_weather_determinism.cpp`'s
  bit-exact replay guarantee, replaced by §9.3's summary contract.

---

## 15. Dependencies and blockers

- **Blocker: no terrain height field exists in the engine.** `PlanetParams` and
  `sky.frag`'s `relief_normal` are an analytic shading trick, not queryable elevation
  (`regional_weather_grid.hpp:50-55`). Orography, surface type, valley fog, föhn, rain
  shadows, and terrain-driven turbulence — all of Phase D and part of Phase B's surface
  model — cannot start until the terrain system provides one. Phases A, B (aloft), C, and
  E are unblocked.
- **T0 asset pipeline** — *done*. `se climatology bake` builds `assets/atmosphere/climatology.set0`
  from NCEP-NCAR Reanalysis 1, NOAA OISST V2 and Natural Earth (ERA5 was dropped: it needs a
  Copernicus account, and an asset nobody else can reproduce is an asset nobody else can check).
  Attribution rides inside the asset. Terrain and land-cover remain blocked on §15's terrain
  system, which is what the ETOPO/MODIS half of this item was waiting for.
- `render_pipeline_refactor.md` **Phase 7** (sky-view / aerial-perspective LUTs) —
  consumed by §8.3's spatial fog and AP coupling; analytic fallbacks stand until it lands.
- `render_pipeline_refactor.md` **Phase 11** (async compute) — the preferred home for the
  tier steps; the graphics queue is a working interim.
- **Water/sea state** consumes `IAtmosphereQuery`'s wind field.
- **Legacy references.** Nineteen source files still cite `docs/design/weather_and_clouds.md`
  by section number in their file comments. Those files are rewritten or deleted by Phases
  A–E; the references are corrected as each file is touched rather than swept blindly into
  section numbers that no longer mean anything. The removed document remains in git history.

---

## 16. Disposition of the shipped W4–W6 code

| File | Disposition |
|---|---|
| `synoptic_weather.hpp` | Deleted (Phase C). Its editor affordance becomes vorticity injection into T1. |
| `regional_weather_grid.hpp` | **Deleted (Phase B2).** Replaced by the GPU nest. The policy-object decomposition is not what the GPU form wanted — a stage is a compute shader plus its parameters, so `AtmosphereParameters` carries the data and the eleven shaders carry the schemes. |
| `ISurfaceModel` / `IRadiationModel` | **Never introduced, and B3 is where that was settled rather than deferred again.** Both arrived as *stages*: `atmosphere_surface.comp` is the surface model and the radiation is the shortwave and longwave terms inside it. Neither is a swappable policy object, because on the GPU a "model" is a shader plus a parameter group and there is exactly one of each — an interface with one implementation forever is a stub wearing a vtable. The seam that *would* earn its keep is a **provider of surface properties**, so a terrain or ocean system could publish a real land/sea mask, and that one is blocked on the same terrain field Phase D is (§15). See B3c. |
| `weather_cloudscape_compiler.hpp` | Kept, narrowed (Phase B1). The genus choice moved to §7.4's classifier (`Render::classify_cloud_genus`, shared with the GPU bake); what remains produces the label and the medium. |
| `weather_provider.hpp` | Reshaped (Phase A) into `IAtmosphereSource` / `IAtmosphereField` / `IAtmosphereQuery` / `IAtmosphereAuthoring`. |
| `weather_types.hpp` | Replaced (Phase E) by `AtmosphereProfile` / `AtmosphereDiagnostics`. |
| `ingested_weather.hpp`, `metar_parser.hpp` | Kept, retargeted onto the new contract — and made installable, which they are not today. |
| `weather_wind.hpp`, `weather_flight_hazards.hpp`, `weather_world_coupling.hpp` | API shape kept; source becomes the query mirror. |
| `test_weather_determinism.cpp` | **Deleted (Phase B2)** per §0. Replaced by `test_atmosphere_nest.cpp`, which pins the base state, the saturation relations and the grid against textbook values, plus the mirror's cold start. The summary contract itself is Phase E. |
| W0–W3 render passes and shaders | Kept. Changes enumerated in §8. |
