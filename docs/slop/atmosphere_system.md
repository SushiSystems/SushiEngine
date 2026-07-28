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
| **Anelastic / Boussinesq non-hydrostatic cloud model** over a regional domain | Explicit convection: thermals, updrafts, cumulus life cycle, anvils, **cold pools** triggering the next cell, squall lines | 256×256×48, GPU, ~2 ms per step |
| **Warm-rain (Kessler) + simple ice microphysics** | Cloud → rain → evaporation → downdraft; snow vs. rain vs. graupel by phase | 3–5 extra advected fields |
| **Surface energy balance + bulk boundary layer** | Diurnal thermals, sea breeze, valley fog, orographic enhancement | 2-D, negligible |
| **LES patch** (100 m) | Individual thermals, rotor/turbulence structure, hero storms | 200³, expensive, optional |

The counter-intuitive affordability argument, stated once because it drives every budget
in §12: **these models are stepped in game time, and their stability-limited time steps
are far longer than a frame.** The regional nest's Δt is ~2 s of simulated time; at 1×
time scale that is one step every two seconds of wall clock. A 3.1-million-cell
non-hydrostatic step costing ~2 ms, taken every 2 s, amortizes to **~0.02 ms per frame**
at 60 Hz. Real cloud physics is cheap in a game not because it is simplified, but because
weather is slow.

### 2.3 What we take, and what we skip

| Source | What we take |
|---|---|
| **Two-layer QG theory (Phillips 1954; Holton ch. 6)** | The global core. The minimum model that produces cyclogenesis and fronts *emergently* from a baroclinically unstable mean state, rather than placing them by hand. |
| **Anelastic cloud models (Ogura–Phillips; Klemp–Wilhelmson; the WRF/CM1 lineage)** | The regional nest. Sound-filtered so Δt is set by advection and buoyancy, not by acoustic waves — the reason a non-hydrostatic model is affordable at all. |
| **Kessler (1969) warm rain; Rutledge–Hobbs ice** | The microphysics. The standard minimal closed scheme: `q_v`/`q_c`/`q_r` (+ `q_i`/`q_s`), saturation adjustment, autoconversion, accretion, evaporation, terminal fall speed. |
| **Davies (1976) relaxation nesting** | One-way T1 → T2 lateral boundary coupling with a relaxation zone, the standard technique, instead of hard boundary injection that reflects. |
| **MacCormack / BFECC advection (graphics lineage: Selle et al.)** | Monotone, low-diffusion semi-Lagrangian transport at Courant ≈ 1 — directly fixes §1.4. |
| **ERA5 / ETOPO / MODIS land cover** | T0 climatology as *data*: zonal-mean jets, monthly SST, terrain, surface type. The reason the Sahara differs from the Amazon without simulating either. |
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
 T1  Global dynamical core (GPU)                           ~0.15 ms per step, ~1 step/6 min game time
     2-layer moist QG on 512×256 lat/lon (~78 km)
     emergent: cyclogenesis · fronts · jet · storm tracks · precipitable water
        │  one-way nesting: Davies relaxation into T2's boundary zone
        ▼
 T2  Regional nest (GPU)                                   ~2 ms per step, ~1 step/2 s game time
     anelastic non-hydrostatic, 256×256×48 over ~512 km (2 km / ~100-500 m)
     explicit convection · Kessler+ice microphysics · surface fluxes · orography
        │  θ, q_v, q_c, q_r, q_i, q_s, u, v, w, π'
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

**Vulkan compute**, in the render domain's queue family, for T1/T2/T3. The rationale is
narrow and worth stating because the alternative is the house default:

- Every consumer of the heavy fields is the renderer, every frame. Running them in SYCL
  would require exporting external memory into Vulkan for the one path that matters, and
  SushiRuntime's deployment posture (JIT-first, one vendor per build, shared-USM default)
  makes that interop the most fragile part of the stack for zero gain here.
- The gameplay side needs a *coarse, stale* mirror, which is a readback — cheap and
  already a solved pattern in the engine.
- T1 is small enough (131 k cells) that if a future need arises to run it CPU-side or in
  SYCL for global gameplay queries, it can move behind `IDynamicalCore` without touching
  anything else. That option is preserved, not taken now.

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
    IMicrophysics          saturation + precipitation (KesslerWarmRain, KesslerWithIce, …)
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
| Land/sea mask + surface type | 5 km | MODIS/Natural Earth class | surface fluxes, albedo, roughness |
| Monthly SST | 1° | ERA5/OISST monthly climatology | surface moisture and heat flux |
| Zonal-mean thermal wind / jet profile | 1° × month | ERA5 monthly means | T1's baroclinic mean state |
| Monthly precipitable water | 1° | ERA5 | T1 moisture relaxation, ITCZ band |
| Soil moisture / vegetation | 5 km | seasonal climatology | latent vs. sensible flux partition |

For a non-Earth body, T0 degrades to analytic latitude bands driven by `PlanetParams` and
the body's obliquity/rotation rate — the system stays functional, just less specific.

Licensing note: ERA5 (CDS), ETOPO, Natural Earth, and MODIS land cover are all
redistributable under attribution-class terms; sourcing and baking them into engine
assets is a task of Phase C, not an afterthought.

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
∂q_x/∂t = −(u·∇)q_x + microphysics(q_v, q_c, q_r, q_i, q_s) − ∂(V_x q_x)/∂z
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

**Named limit: the subgrid cloud closure is not rescaled with the tier.**
`cloud_critical_humidity` sets where a cell's humidity distribution begins to hold cloud, and its
0.80 is the standard value *at 2 km* — the subgrid variance a cell hides is a function of how big
the cell is, so the correct value falls at coarser spacing and approaches 1 as the grid resolves
the cloud itself. One authored number therefore means slightly different things per tier.
Measured, this is invisible when the sky is decisively cloudy or decisively clear, and visible
exactly at the margin: a configuration that leaves High with 5.8 % of columns cloudy leaves Low,
Medium and Ultra between 0 and 0.1 %. Scaling the critical humidity with `spacing_m` is the fix,
and it wants its own calibration rather than a plausible-looking exponent.

Every prognostic field is fp32, including the `q_*` fields. Half floats were the
original choice for them on a range argument — mixing ratios are a few grams per
kilogram and nowhere near the format's ceiling — and §11's Phase B2c measures that
argument to be the wrong one: what decides a format here is the ratio of a step's
tendency to the value it lands on, and the surface latent flux's increment was a third
of one unit in the last place, so the boundary layer never moistened at all. The VRAM
figures above assume fp16 and are correspondingly optimistic; the shipped 192×192×48
nest is ~135 MB. Double buffering only for the fields the advection step needs both
states of.

**Advection** is monotone semi-Lagrangian at Courant ≈ 1 (MacCormack with a limiter, or
BFECC), directly replacing §1.4's diffusive scheme. **Pressure** is an FFT-based Poisson
solve in the horizontal with a tridiagonal vertical solve, ~10 iterations of a multigrid
smoother where the boundary conditions break FFT separability. This is the dominant cost
of the step.

**Microphysics** (`IMicrophysics`, default `KesslerWithIce`):

```
e_s(T) = 611.2 · exp(17.67 (T − 273.15) / (T − 29.65))      Pa      (Magnus/Teten)
q_s    = 0.622 e_s / (p − 0.378 e_s)
saturation adjustment: condense (q_v − q_s) with latent heating into θ
autoconversion:  A  = k₁ · max(q_c − q_c0, 0)
accretion:       C  = k₂ · q_c · q_r^0.875
rain evaporation below cloud base; terminal fall speed V_t = 36.34 (ρ q_r)^0.1364
ice: deposition/freezing above the −5…−20 °C band, snow with its own fall speed
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

**Beyond the nest.** Outside ~512 km, the atmosphere is T1's resolution. §7.5 describes
what the sky does there.

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

- Inside the nest (≤512 km): T2's own condensate.
- Beyond it: T1's column moisture and layer thicknesses expand into a coarse, smooth
  extinction field — enough for a correct horizon and the panorama impostor, which is
  where those pixels end up anyway.
- The existing panorama pass, cloud shadow map, light volume, and skip field all consume
  the new field with no interface change.

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
which for a medium with a minutes-long time scale is not observable.

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

### Where this stands — 2026-07-29

**Shipped:** A · B1 · B2 · B2b · B2c · B3 (a–e) · **C1**. Phase B's acceptance bar is met, its
last clause closed by C1's subsidence. 448 tests pass; `se build` and `se editor --no-run` are
clean.

**Next, and it is the largest remaining piece:** the rest of Phase C — T1's 2-layer moist
quasi-geostrophic core replacing the analytic `SynopticLayer`, T0 climatology assets sourced and
baked, and editor authoring becoming vorticity injection rather than system placement. C1 already
took the first bite of the seam that connects them: the parent now supplies vertical motion as
well as wind and anomalies, so a real core has somewhere to publish it.

**Carried, deliberately, with the reason each time:**

- **`humidity_scale_height` does not do what it is documented to do** — the airmass is drier than
  it should be, every configuration that makes cloud has to be pushed there, and the afternoon
  cumulus forms hours later than it ought to. Not fixed because it changes the look of every
  existing scene, which is an authoring decision. This is the single highest-value item left in
  the model and the first thing to reach for if the sky reads as too dry. See B2c's resume notes.
- **`cloud_critical_humidity` is calibrated at 2 km and does not scale with the spacing**, so the
  Low tier disagrees with the others at a marginal sky. Named limit in §6.
- **`AtmosphereSurface` — the land/sea seam** — blocked on the same terrain height field Phase D
  is blocked on (§15). See B3c for why building it early would be worse than not having it.
- **The nocturnal cloud's only sink is now subsidence.** Cloud-top radiative cooling and the
  entrainment it drives are not modelled; with the parent quiescent a deck still persists.
- **A slow cooling near the tropopause**, −0.29 K/day over 72 h, attributable to numerical
  diffusion in the semi-Lagrangian transport across the base state's sharpest θ gradient. Not a
  radiation gap and a radiation scheme would not fix it.
- **§12's performance budget table is stale.** It budgets the T2 step at 2 ms; the measured step
  is ~10 ms at High. The honest metric established in B2c is *cost per second of weather bought*
  (1.27 ms), not cost per step, and the table has not been rewritten in those terms.
- **An async compute queue** is now a straightforward win rather than a blocked one, since the
  step already submits at frame end and waits on the frame's readers.

**Never visually confirmed in the editor.** Everything in B2c, B3 and C1 was settled with
`atmosphere_probe` headlessly, plus builds and tests. The frame-end submission move, the surface
panel rework and the ice optics have not been looked at on screen.

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

  | stage | ms | share |
  |---|---|---|
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
not fix it. Recorded as a named limit rather than masked.

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

### Phase C — Global core and climatology — *in progress*

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

#### The rest of Phase C

T0 assets sourced and baked. T1's 2-layer moist QG core replaces `SynopticLayer`; Davies
relaxation nesting into T2; `Astro::Ephemeris` as the single solar authority. Editor
authoring becomes vorticity injection.

*Acceptance: cyclogenesis is emergent — nothing places a low. The textbook front-passage
sequence occurs unscripted: cirrus, then altostratus, then continuous rain, wind veering,
a cold-frontal cumulus line with a gust, then clearing to scattered cumulus. Weather
1 000 km away has developed on its own by the time the player flies there.*

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

| System | Per step | Cadence (game time) | Amortized per frame @60 Hz |
|---|---|---|---|
| T1 global core | ~0.15 ms | 6 min | negligible |
| T2 regional nest | ~2.0 ms | 2 s | ~0.02 ms |
| T3 cloudscape compile | ~1.0 ms | on T2 step / camera rebake | ~0.05 ms |
| Query mirror readback | ~0.05 ms | every 4 frames | ~0.01 ms |
| T4 render (unchanged) | — | every frame | ≤2.5 ms |
| **Total** | | | **≤2.6 ms** |

At 16× time acceleration the simulation tiers scale linearly to ~0.35 ms per frame and
remain comfortably inside budget. VRAM: ~150 MB for T2 at High, ~40 MB for T3's field set,
~4 MB for T1 and the mirror.

The headline: **the whole simulation costs less per frame than the cloud composite**, for
the reason given in §2.2.

---

## 13. SOLID

- **SRP** — one reason to change each: the dynamical core, the microphysics, the surface
  model, the radiation model, the cloudscape compiler, the query mirror, and each render
  pass. §1.6's seven-responsibility class does not reappear.
- **OCP** — a new microphysics scheme, a new core (cubed-sphere, a different closure), a
  new surface model, or a per-biome parameter set is a new class or a data edit. Genus and
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
- **T0 asset pipeline** (ERA5/ETOPO/MODIS bake) is a Phase C work item with its own
  sourcing and licensing step.
- `render_pipeline_refactor.md` **Phase 7** (sky-view / aerial-perspective LUTs) —
  consumed by §8.3's spatial fog and AP coupling; analytic fallbacks stand until it lands.
- `render_pipeline_refactor.md` **Phase 11** (async compute) — the preferred home for the
  tier steps; the graphics queue is a working interim.
- **Water/sea state** consumes `IAtmosphereQuery`'s wind field.
- **Legacy references.** Nineteen source files still cite `docs/slop/weather_and_clouds.md`
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
