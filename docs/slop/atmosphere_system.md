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

| | Medium | High | Ultra |
|---|---|---|---|
| Horizontal | 192×192 @ 3 km | 256×256 @ 2 km | 384×384 @ 1.3 km |
| Vertical | 32 stretched levels, 0–18 km | 48 levels, 0–20 km | 64 levels, 0–20 km |
| Δz | ~150 m surface → 700 m aloft | ~100 m → 500 m | ~80 m → 400 m |
| Δt (game time) | ~3 s | ~2 s | ~1.5 s |
| VRAM | ~60 MB | ~150 MB | ~420 MB |

`θ`, `u`, `v`, `w`, `π′` are fp32; the `q_*` fields are fp16 (their dynamic range is
small and their gradients are not differentiated) — this is where the VRAM figures come
from. Double buffering only for the fields the advection step needs both states of.

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

### Phase B2c — The correctness pass B2 exposed — *in progress*

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

#### Where to resume

Re-log `peak_w` for half an hour with the buoyancy correction in.

- **Still ~3.5 × 10⁻⁴ m/s** → the loss is in the pressure projection or the thermal seed, not
  the forcing. Look at `pressure_iterations` against the residual the anisotropic grid leaves,
  and at whether `thermal_seed_amplitude` produces enough horizontal variance for the anelastic
  constraint to permit any ascent at all. A horizontally uniform buoyancy field cannot lift
  anything, by mass continuity.
- **Orders of magnitude larger** → follow it to the condensation level and check the moisture
  budget next.

The drying is *not yet explained*. Diffusion is eliminated, advection cannot produce a net
domain-wide loss, and condensation would show as cloud. Settling it needs the vertical profile
of `q_v`, which the mirror does not carry — it reports the surface only. That is the next
diagnostic to add, and it should be added before the next hypothesis is formed.

### Phase B3 — Surface energy balance and ice

The other half of Phase B's original acceptance bar. Surface fluxes are prescribed constants
today; a real energy balance — insolation through `Astro::Ephemeris`, a slab heat capacity,
land/sea partitioning — is what turns them into the diurnal cycle, and `ISurfaceModel` /
`IRadiationModel` are introduced when they have an implementation rather than stubbed ahead of
one. Ice microphysics (deposition, freezing, snow with its own fall speed) joins Kessler here.

*Acceptance: morning clear → midday cumulus → evening decay happens without anything scripting
it.*

### Phase C — Global core and climatology

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
| `regional_weather_grid.hpp` | **Deleted (Phase B2).** Replaced by the GPU nest. The policy-object decomposition is not what the GPU form wanted — a stage is a compute shader plus its parameters, so `AtmosphereParameters` carries the data and the ten shaders carry the schemes; `ISurfaceModel`/`IRadiationModel` arrive in B3 with implementations rather than as stubs. |
| `weather_cloudscape_compiler.hpp` | Kept, narrowed (Phase B1). The genus choice moved to §7.4's classifier (`Render::classify_cloud_genus`, shared with the GPU bake); what remains produces the label and the medium. |
| `weather_provider.hpp` | Reshaped (Phase A) into `IAtmosphereSource` / `IAtmosphereField` / `IAtmosphereQuery` / `IAtmosphereAuthoring`. |
| `weather_types.hpp` | Replaced (Phase E) by `AtmosphereProfile` / `AtmosphereDiagnostics`. |
| `ingested_weather.hpp`, `metar_parser.hpp` | Kept, retargeted onto the new contract — and made installable, which they are not today. |
| `weather_wind.hpp`, `weather_flight_hazards.hpp`, `weather_world_coupling.hpp` | API shape kept; source becomes the query mirror. |
| `test_weather_determinism.cpp` | **Deleted (Phase B2)** per §0. Replaced by `test_atmosphere_nest.cpp`, which pins the base state, the saturation relations and the grid against textbook values, plus the mirror's cold start. The summary contract itself is Phase E. |
| W0–W3 render passes and shaders | Kept. Changes enumerated in §8. |
