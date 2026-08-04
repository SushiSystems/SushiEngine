# Atmosphere and weather

This file covers the weather and atmosphere domain: the superseded synoptic/regional simulation
tier and the provider seam it introduced, the world coupling and data-ingestion phases built on
it, and the spatial weather field, cloudscape window, and GPU regional nest that replace its core.

## 1. Weather simulation (T1 synoptic layer and T2 regional grid, phases W4-W6)

> **Superseded.** This section documents what is *currently shipped*. The design it was built
> from (`docs/design/weather_and_clouds.md`) has been retired — removed, in git history — and
> replaced by **`docs/design/atmosphere_system.md`**, whose §1 audits why the W4-W6 simulation
> tier does not deliver what it claims (chiefly: the meteorology never reaches the sky spatially,
> because `extract()` samples one column and compiles it into a globally uniform `Cloudscape`
> over tiled noise). This section and its
> [world coupling](#11-world-coupling-phase-w5) and
> [data ingestion](#12-flight-sim-polish-and-data-ingestion-phase-w6-final)
> subsections below are replaced tier by tier as that plan's phases land;
> [the render tier](presentation-render.md#15-lighting-materials-and-the-sky) survives it largely
> intact.
>
> **Phase A has landed** — see
> [the spatial weather field](#13-the-spatial-weather-field-atmosphere-phase-a).
> The single-column bridge described below still runs (it is what compiles the deck stack), but
> it is no longer the *only* thing the renderer hears from the simulation, and the column it
> samples is now taken at the camera rather than at the scene's geodetic anchor.

The retired plan split weather into four tiers — T1 synoptic, T2 regional, T3 cloudscape compile,
T4 render — with one rule: the sim tiers (T1/T2) live in the **sim domain** under SushiLoop's
determinism discipline (fixed tick, seeded RNG, serialized, replayable, no wall-clock or
render-side input), and T3/T4 are render-side and free to be nondeterministic. T3
(`CloudscapeCompilePass`, see
[the cloud pass](presentation-render.md#15-lighting-materials-and-the-sky)) and T4 (the cloud
render pipeline) predate this section and are untouched by it; this section documents the new
sim-domain half and the seam that connects it.

**T1 — the global dynamical core**
(`engine/domain/atmosphere/include/SushiEngine/atmosphere/quasigeostrophic_core.hpp`,
`Atmosphere::QuasiGeostrophicCore`) is a two-layer moist quasi-geostrophic model on a 512x256
latitude/longitude grid. Potential vorticity and column water are the only prognostic variables;
wind, pressure, vertical motion, fronts and the jet are all *diagnosed* from them. Cyclogenesis is
emergent — the initial state is a zonal jet plus a perturbation of a hundredth of its speed, and
what grows is whatever the mean state is unstable to, measured at 0.254/day. One step costs
~36 ms of one core and is due once per six minutes of game time, so `advance()` carries the
remainder itself and is capped at eight steps per call, which means **one call covers at most 48
minutes of game time**.

This replaced `SynopticLayer`, an analytic layer of up to eight moving elliptical Gaussian
pressure systems with life-cycle timers and stylized fixed-angle front rays. It was never a
simulation and did not claim to be: it translated authored shapes, so nothing could form, deepen
or decay that had not been placed. `docs/design/atmosphere_system.md` §1 records the audit and
§11's Phase C the swap. The state that used to be a trivially-copyable `SynopticState` is now
megabytes of field, which is why the editor persists it to a binary sidecar beside the scene
rather than into the scene JSON.

**T2 — the regional grid** (`regional_weather_grid.hpp`, since retired — see
[the regional nest](#15-the-regional-nest-anelastic-convection-on-the-gpu-atmosphere-phase-b2);
`RegionalWeatherGrid`) is a camera(observer)-centered grid of columns — wind, temperature offset,
humidity, cloud water, convective fraction, precipitation — over three vertical levels
(`Sim::CloudLevel::Low/Mid/High`, matching `Render::CloudGenus`'s own WMO étage grouping rather
than the finer profile nothing downstream can consume yet).

It ticks on its own nested `Loop::FixedTimestepClock` (15 s default), driven only by the
caller-supplied `dt`, never wall-clock: semi-Lagrangian advection of humidity/temperature/
cloud-water by T1's wind (backtraces each cell to its departure point and bilinearly samples the
prior tick's field — unconditionally stable at any `dt`, unlike naive forward-difference
advection), orographic lift (wired through a pluggable terrain-height sampler that defaults to
flat, since no terrain height field exists anywhere in the engine yet — see the CHANGELOG for the
audit), diurnal convection (a standalone solar-position estimate feeding a CAPE proxy), and a
condensation/precipitation/evaporation moisture closure.

Grid cells address an *absolute* equirectangular tangent-plane lattice (anchored at lat 0/lon 0)
rather than one relative to the grid's own moving origin — the same floating-origin discipline
`shadow_uniforms.cpp`'s cascade snap already uses (see
[shadows](presentation-render.md#14-shadows)): floor to a lattice coordinate that does not itself
depend on where the camera currently is, so a rebase (triggered once the observer drifts a whole
cell width from center) never introduces drift or a visible seam. Cells scrolled in during a
rebase are filled from a deterministic background-climatology function of position, never left
uninitialized and never drawn from the RNG. Samples are linearly blended between the last two
ticks by the nested clock's own interpolation fraction, so a consumer sampling every render frame
sees continuous motion between the grid's much coarser ticks.

**The `Sim::IWeatherProvider` seam**
(`engine/world/simulation/include/SushiEngine/simulation/weather_types.hpp`,
`.../simulation/weather_provider.hpp`, `.../simulation/weather_cloudscape_compiler.hpp`) is where
DIP happens: a provider's entire contract is `WeatherColumn sample_column(GeodeticPosition)` —
coverage/density/convective-mix per level plus surface wind/precipitation — and nothing about
pressure systems, grid cells, or RNGs crosses it. `ProceduralWeather` wraps T1+T2 behind it;
`StaticWeather` formalizes the pre-existing manual `Cloudscape` deck authoring as an equally
legitimate provider (bucketing each enabled deck's genus into its WMO étage), so the renderer
genuinely cannot tell a hand-authored sky from a procedural one — the LSP property the design
doc's §9 calls for. `IngestedWeather`
(`engine/world/simulation/include/SushiEngine/simulation/ingested_weather.hpp`, W6, see
[data ingestion](#12-flight-sim-polish-and-data-ingestion-phase-w6-final)) implements the same
interface from GRIB/METAR-sourced data without this seam changing.

`WeatherCloudscapeCompiler` is the separate, stateless, pure-function class that turns any
provider's `WeatherColumn` into a `Render::Cloudscape` — picking a genus per level from its
coverage/convective mix and reserving a fourth deck slot for Cumulonimbus when the low level is
both filled and strongly convective (the "cumulus line" at a cold front). `RuntimeSimulation`
wires all of this at exactly two points: `step_once()` ticks the installed provider, and
`extract()` writes the compiled `Cloudscape` into `Environment::clouds` — the identical write
path manual authoring already used, so `CloudscapeCompilePass` (T3) and the entire render tier
needed zero changes to consume procedurally-driven weather.

**Which provider is installed is a mode, and it used to be a boolean that meant something else.**
`ISimulation::weather_mode` names `Manual` or `Procedural`; the predicate it replaced was
`static_cast<bool>(weather_provider_)`, which made *the absence of a provider* the definition of
Manual. That is not a naming quibble: with no provider there is no published `WeatherField`, so
one hand-authored deck stack was applied to every square metre of the body and a planet seen from
orbit was uniformly overcast. Both modes install a provider now, and the distinction is where the
sky comes from — **placed** or **grown**:

- `SeededWeather` (`engine/world/simulation/include/SushiEngine/simulation/seeded_weather.hpp`)
  places it. `Atmosphere::SynopticField`
  (`engine/domain/atmosphere/include/SushiEngine/atmosphere/synoptic_field.hpp`) puts a dozen
  signed Gaussian pressure centres on a zonal cloud climatology from a 64-bit seed; a low raises
  coverage and convects, a high drives it to zero. Deterministic (SplitMix64, not a
  standard-library distribution, whose bit-to-value mapping is implementation-defined), defined at
  every point on the body, and evolving in nothing but the ITCZ's seasonal migration — which is
  the correct behaviour for a sky an author chose rather than a limitation of one.

- `ProceduralWeather` grows it, in the quasi-geostrophic core and the GPU nest.

`StaticWeather` — the uniform decomposition of an authored deck stack — remains in
`weather_provider.hpp` as the honest field form of a provider with no horizontal structure, and is
what an embedder wanting exactly that installs.

**The planetary answer travels separately from the local one, and has to.**
`IWeatherProvider::publish_field` fills a lattice a few hundred kilometres across, which is
everything a baked cloudscape window can see and nothing a camera in orbit can: at planetary
distance one texel of a 64-cell global grid is six hundred kilometres.
`IWeatherProvider::synoptic_field` is the second channel — the placement itself, in closed form —
which `RuntimeSimulation::publish_synoptic_field` rotates from geographic into scene space through
`Environment::planet_body_axes` and publishes as `Render::SynopticFieldView`
(`engine/domain/environment/include/SushiEngine/environment/synoptic_field.hpp`), twelve
directions by value.

`cloud.frag`'s planet-scale globe field evaluates the *same* function from
the *same* centres (`engine/presentation/render/shaders/synoptic_field.glsl`), so the baked
windows and the field they fade into cannot disagree about where the weather is — disagreement
would show as a seam at the far window's rim. Null is the honest answer for a provider whose
solution is regional, and leaves the renderer on the zonal climatology alone, which remains true
of every body with an atmosphere.

**Determinism proof.** `tests/integration/test_weather_determinism.cpp` was the weather-domain
analogue of `test_deterministic_replay.cpp` (see
[SushiLoop core](world.md#2-sushiloop-core)) and `test_particle_determinism.cpp` (see
[the particle system](domain-vfx.md)): two independently constructed `ProceduralWeather`
instances, seeded identically and ticked through the same fixed-step/julian-date stream, must
reach byte-identical `SynopticState` and sampled `WeatherColumn`/`Cloudscape` state. A second
suite proves the visible half of the acceptance bar — a hand-placed low's cold front measurably
sweeps past a fixed point over simulated hours and the compiled `Cloudscape` reacts at the moment
it does.

**Editor integration.** The Environment panel's Clouds section
(`applications/editor/source/environment/weather_panel.cpp`) gained a Manual/Procedural mode
radio, a synoptic map overlay drawn from live T1 state, a per-system authoring list (add/remove,
depth/radius/speed/heading sliders), and a time-of-day scrub tied to the master epoch; the
existing preset buttons now seed a synoptic scenario in Procedural mode instead of setting deck
parameters directly. Scene serialization
(`engine/world/serialization/source/scene_serializer.cpp`'s
`weather_to_json`/`weather_from_json`) round-trips the procedural mode flag and the full T1
`SynopticState` (RNG, clock, every live system); T2's regional grid reseeds deterministically from
that restored state on the next tick rather than being serialized cell-by-cell — see the CHANGELOG
for why that scope line was drawn there.

Deliberately out of W4's scope, and unaffected by it: fog/turbidity coupling, precipitation
particles, wet surfaces, lightning, a `weather_wind()` gameplay API, and real-data ingestion (the
retired plan's §5.3/W5/W6) — the `IWeatherProvider` seam this phase introduces is what lets those
land later without revisiting T1, T2, or the bridge. W5, below, is that "later".

### 1.1. World coupling (phase W5)

The retired plan's §5.3 ("coupling weather -> world (beyond clouds)"), with its §7 acceptance bar:
"rain falls from the cell that is raining, under a dark base, with wet ground and reduced
visibility — one cause, every symptom." The whole phase turns on that last clause: every symptom
below is derived from the *same* `WeatherColumn` sample `RuntimeSimulation::extract()` already
takes to compile the `Cloudscape`
([above](#1-weather-simulation-t1-synoptic-layer-and-t2-regional-grid-phases-w4-w6)), not four
independently tuned systems that happen to agree.

**`Simulation::WeatherWorldCoupling`**
(`engine/world/simulation/include/SushiEngine/simulation/weather_world_coupling.hpp`) is
`WeatherCloudscapeCompiler`'s sibling: a separate, stateless, pure-function class that compiles
the identical `WeatherColumn` into `Render::WeatherCoupling`
(`engine/domain/environment/include/SushiEngine/environment/environment.hpp`) — extra froxel-fog
density, extra atmosphere Mie turbidity, ground wetness, a precipitation-intensity echo, and a
near-surface wind passthrough. `extract()` samples one column, hands it to both compilers, and
writes the results into `Environment::clouds` and `Environment::weather` respectively.

`WeatherCoupling` is deliberately additive/multiplicative rather than a replacement:
`VolumetricFogPass` and `AtmosphereLUTPass` add its bias fields onto the author's own
`FogParameters::density`/`AtmosphereParams::mie_coefficient` at push-constant build time, so a
scene with weather off renders exactly as it did before this phase (every field defaults to
zero), and the author's fog/atmosphere sliders are never overwritten in place the way
`Environment::clouds` is — which is what keeps this field safe to recompute from scratch every
`extract()` without the read-modify-write hazard a full overwrite would create through the
editor's `environment()` -> edit -> `set_environment()` round trip. `AtmosphereLUTPass`'s existing
`medium_changed()` memcmp gate already rebuilds the static transmittance/multi-scatter LUTs
exactly when the weather-adjusted Mie coefficient changes tick to tick — "runtime-dynamic in the
Hillaire model" (the design doc's §5.3) needed no new plumbing.

**Cloud-base darkening** is the design doc's literal recipe (`density += density * weather.a`)
applied inside `WeatherCloudscapeCompiler::compile()` itself, on the low deck only (the band T2's
moisture closure actually rains from) — before `CloudscapeCompilePass`'s bake ever sees the deck,
so the bake's own contract is untouched.

**Wet surfaces** follow the exact `MaterialFlags` pattern `MATERIAL_PARALLAX_SHADOWS` already
established: `Render::Material::weather_wettable` (off by default, like `parallax_shadows`) packs
to a new `Assets::MATERIAL_WEATHER_WET` (`MaterialFlags`) bit in
`engine/presentation/render/source/material/material_system.cpp`;
`engine/presentation/render/shaders/pbr.frag` darkens albedo and drops roughness toward a thin
water film only where that bit is set, scaled by a global wetness read from
`scene.light_shadow_b.y` — otherwise-spare UBO space (see
`engine/presentation/render/source/scene/scene_uniforms.hpp`'s field doc) rather than growing
`SceneBlock`, since every shader that already declares `light_shadow_b` (it carries a secondary
caster's shadow index) needed no binding changes. `create_terrain()` opts its own material in by
default so the acceptance scenario shows wet ground without extra authoring; every other material
opts in explicitly.

**Precipitation VFX**: `RuntimeSimulation::extract()` builds one synthetic, sim-owned
`VFX::CompiledEmitter` (`weather_rain_emitter_`) each frame precipitation is active, and appends a
`Render::ParticleEmitterView` for it straight into `RenderScene::particle_emitters` — the same
seam an authored particle-emitter entity's Cosmetic sub-emitters already write through (see
[scene emitters](domain-vfx.md#19-scene-emitters-are-entities)), just sourced from weather instead
of a record. Not an authored ECS entity: that would clutter the Hierarchy/Outliner and the scene
file with a system-generated object. Uses the GPU cosmetic path per
`QualityParameters::gpu_particles`'s own doc ("the deterministic CPU particle path is unaffected;
it is gameplay, not a quality knob") — ambient weather rain is squarely cosmetic. Rain only:
`WeatherColumn` carries no temperature signal, so there is no honest basis to pick snow over rain
(a named scope-down, not a fabricated phase test).

**`Simulation::weather_wind(weather, position, altitude, time)`**
(`engine/world/simulation/include/SushiEngine/simulation/weather_wind.hpp`) is the design doc's
"one sampling API... GoT pattern: analytic + perturbation, no dense volume". The base half is
`IWeatherProvider::wind_at` (altitude-parameterized via `level_fraction`); this file adds only the
position/altitude -> level_fraction mapping and a deterministic, stateless pseudo-noise
perturbation. That perturbation used to be scaled by the distance to a *drawn* front; since Phase
C it is scaled by `IWeatherProvider::frontal_strength_at`, the thermal gradient the flow has
actually concentrated (5 K/100 km being a strong front), because a dynamical core draws nothing.

A free function over `const IWeatherProvider&`, not a new virtual — the provider already answers
everything it needs, and adding a virtual would widen the seam every provider must satisfy for
arithmetic none of them differ on. The concrete W5 consumer is the rain emitter's lateral drift
(fed through the emitter's gravity module, scaled well down since rain falls in ~2 s); cloth and
the CPU-deterministic particle path are named, unbuilt follow-up scope below.

**Scoped down, named rather than silent:**

- **Dew-point spread**, the design doc's literal fog driver, is approximated by the low band's
  cloud coverage/density (itself derived from condensed water past T2's relative-humidity
  threshold) rather than adding a humidity field to `WeatherColumn`'s contract for the one
  consumer that would use it — `weather_world_coupling.hpp`'s file docs name this explicitly.

- **Valley/orographic fog** is not implemented: W4 already left orographic lift wired but inert
  (no terrain-height field exists anywhere in the engine), so there is no real per-position lift
  signal to drive a valley-fog term with; fabricating one would be exactly the "invented signal"
  this phase's own instructions warn against.

- **CAPE and terrain roughness** are not part of `weather_wind()`'s turbulence term. CAPE is an
  internal, per-tick intermediate inside `RegionalWeatherGrid::tick_grid`, not part of
  `WeatherColumn`'s stable sampled contract; terrain roughness has no source anywhere in the
  engine (the same gap the orographic-lift note documents).

- **Cloth and the CPU-deterministic particle path do not consume `weather_wind()`.** The physics
  solver's only per-body external-acceleration channel today is `GravitySampler`
  (`make_gravity_sampler()`), which applies uniformly to every rigid body for orbital gravity
  summation — repurposing it for wind would apply wind-strength acceleration to rigid boxes as
  readily as cloth, which is not physically sound, and a distinct cloth-specific force channel is
  real physics-solver surgery, not an additive extension. Named as a deferral, not silently
  dropped.

- **No dedicated flight model exists in the engine yet**, so `weather_wind()`'s "the flight-sim
  payoff" half of the design doc's ask has no consumer to wire into; the function itself is the
  extension point a future flight model would call.

- **Lightning is not implemented this phase.** It is the one §5.3 sub-bullet not part of the
  design doc's §7 acceptance bar's four required symptoms (rain, dark base, wet ground, reduced
  visibility), and safely wiring a transient flash into `CloudLightVolumePass`'s amortized,
  slowly-refreshing bake (or its named simpler alternative, a direct additive term in
  `cloud.frag`) is real, standalone render-tier work better done as its own follow-up than rushed
  alongside everything above.

- **Ground wetness has no soak-in/dry-out lag.** `WeatherWorldCoupling::compile` is a pure
  function of the current column, so wetness tracks precipitation instantaneously; a
  puddle-persistence model is real, unbuilt follow-up scope.

**Tests.** `tests/unit/test_weather_world_coupling.cpp` (`Unit_WeatherWorldCoupling`,
`Unit_WeatherCloudscapeCompiler`, `Unit_WeatherWind`) exercises the pure functions in isolation
from T1/T2's own tick: a clear column compiles to an all-zero `WeatherCoupling`; a rained-out
column raises fog/turbidity/wetness together and monotonically with more rain; `ground_wetness`
stays clamped; precipitation darkens only the low cloud deck; and `weather_wind()` reduces to the
bare analytic field far from any front and gusts more strongly near one (a front-relative point
solved analytically from `front_proximity`'s own formula, not guessed).

### 1.2. Flight-sim polish and data ingestion (phase W6, final)

The retired plan's §7 W6 phase is marked "tier/optional" and names five items: in-cloud whiteout
tuning, canopy wisp particles, icing/turbulence exposure to gameplay, hero envelope assets near
airfields, and `IngestedWeather` (GRIB/METAR). This is the last phase of the W0-W6 roadmap; it
lands four of the five with real value and scopes the fifth out entirely, named below rather than
left unstated.

**In-cloud whiteout tuning** (`engine/presentation/render/shaders/cloud.frag`) is a
self-contained shader change: past a `transmittance` threshold
(`smoothstep(0.02, 0.25, transmittance)`), the accumulated `scattered` colour blends toward a glow
keyed on the view/sun angle `mu` — the residual forward bias real multiple scattering keeps even
as it isotropizes with depth — instead of the flat, direction-blind tone the march's existing
Beer-decayed sun term and constant ambient term produce on their own once deep inside a thick
deck. No push-constant, MRT, or pass signature changed.

**Canopy wisp particles** follow the exact precedent `WeatherWorldCoupling`'s rain emitter set at
W5: a second synthetic, sim-owned `VFX::CompiledEmitter`
(`RuntimeSimulation::weather_wisp_emitter_`) built fresh each `extract()` and appended to
`RenderScene::particle_emitters`, not an authored ECS entity. Unlike the rain emitter it is not
gated on precipitation or even on `ProceduralWeather` being active — it reacts to whichever
`Cloudscape` is currently live in `scene_.environment.clouds`, Manual or Procedural, gated purely
by the camera sitting within 60 m of an enabled deck's own `base_altitude`/`top_altitude`
(`Render::cloud_genus_profile`) with at least 25% effective coverage — the design doc's
"canopy/base boundary... close flythroughs" read literally as a camera-proximity trigger, not a
weather-state one.

**`Simulation::icing_risk(column, altitude)` /
`turbulence_intensity(synoptic, position, altitude, time)`** (new:
`engine/world/simulation/include/SushiEngine/simulation/weather_flight_hazards.hpp`) are two
small, stateless, pure query functions — explicitly *not* a flight model, following W4/W5's own
established call on this exact ask: no flight/aircraft/vehicle system exists anywhere in the
engine, so these are "the extension point a future flight model would call", the identical framing
`weather_wind()`'s own doc already used.

`icing_risk` needed a temperature signal `WeatherColumn` never carried; rather than invent one, a
new `WeatherLevelState::temperature_offset_c` field (`weather_types.hpp`) surfaces state T2 has
tracked internally since W4 (`WeatherCell::temperature_offset_c`) but never had a consumer for,
combined with a standard-atmosphere lapse rate at the query altitude into a trapezoidal risk curve
peaking in the classic supercooled-liquid band and weighted by the same liquid-water proxy
`weather_world_coupling.hpp` already uses. `turbulence_intensity` wraps a new `wind_gust()`
(`weather_wind.hpp`, factored out of `weather_wind()`, which is now exactly
`wind_at() + wind_gust()`), normalizing the same front-proximity-scaled perturbation gust magnitude
W5 already computes into a bounded `[0, 1]` scalar.

**`IngestedWeather`** (new:
`engine/world/simulation/include/SushiEngine/simulation/ingested_weather.hpp`) is design doc
§5.4's real-data `IWeatherProvider`: a caller-supplied background column blended toward the
nearest of zero or more added METAR stations — pure background beyond 60 km of every station, pure
station inside 15 km, linearly blended between. GRIB2 binary decoding is named future work, not
fabricated: `set_background(WeatherColumn)` is the stub seam a real decoder would eventually fill.

METAR, by contrast, is fully decoded this phase by a real, working parser (new:
`engine/world/simulation/include/SushiEngine/simulation/metar_parser.hpp`,
`parse_metar()`/`metar_to_weather_column()`) — a tolerant whitespace-token scanner covering wind
(calm/variable/gust), cloud layers bucketed into a `CloudLevel` by a new shared
`cloud_level_for_altitude()` helper (`weather_types.hpp`), temperature/dewpoint, and
present-weather-derived precipitation intensity, narrowed to exactly what `WeatherColumn` can use
rather than a general METAR decode.

The phase's key deliverable regardless of how much real decoding it contains: `IngestedWeather` is
a genuine, LSP-substitutable `IWeatherProvider` — `WeatherCloudscapeCompiler` and
`WeatherWorldCoupling` accept it with zero changes, proven by
`tests/unit/test_ingested_weather.cpp`'s
`SubstitutesForAnyOtherProviderThroughTheSharedCompilers`. It is not wired into
`RuntimeSimulation`'s live procedural-weather slot or the editor this phase — there is no real
GRIB/METAR data source feeding it yet, so wiring an unused seam into the live sim would be surface
area with nothing behind it.

**Scoped down, named rather than silent:**

- **Hero envelope assets near airfields (Ultra tier) are not implemented, and no infrastructure
  for them was added either.** The design doc frames this as hand-placed NVDF-style hero cloud
  volumes at a small number of showcase airfields — real art-directed content. This engine has no
  airfield/POI placement system and no art pipeline to author such a volume from; a genuine
  implementation needs a new T3 bake target and an LOD/blend seam into the existing field, real
  architecture work with no data behind it yet. Landing a placement-only data structure with no
  renderer consumer would not honestly earn the "hero" name the design doc uses, so this is scoped
  out entirely rather than partially stubbed.

- **METAR station elevation is not modelled**; a report's temperature only ever informs the `Low`
  `CloudLevel` bucket rather than being placed at a real station altitude, since this engine has
  no airport-elevation database to place it more precisely with.

- **`IngestedWeather` owns no clock and ticks nothing**, unlike `ProceduralWeather` — a real-data
  provider is a snapshot the caller refreshes by re-calling `set_background`/`add_station`, not a
  fixed-step simulation.

**Tests.** `tests/unit/test_weather_flight_hazards.cpp` (`Unit_WeatherFlightHazards`),
`tests/unit/test_metar_parser.cpp` (`Unit_MetarParser`), and
`tests/unit/test_ingested_weather.cpp` (`Unit_IngestedWeather`) exercise the new pure functions,
the parser against realistic report strings, and the provider's blend plus its LSP
substitutability through the shared compilers, respectively. All 395 functional tests pass (369
baseline + 26 new), and `se editor --no-run` builds clean.

### 1.3. The spatial weather field (atmosphere phase A)

`docs/design/atmosphere_system.md` §7. Everything above this subsection describes a bridge that
hands the renderer **one column**, which is why nothing the simulation computed could be seen as
spatial structure: the sky's visible pattern came from a static tiled fBm map, modulated only by
that column's global bias. This subsection is the second channel — the simulation's own horizontal
grid, addressed in world space and read by the cloud march per sample.

**The data.** `Render::WeatherField`
(`engine/domain/environment/include/SushiEngine/environment/weather_field.hpp`) is a *borrowed*
view — `cells_x × cells_z × level_count` samples of
coverage/density/convective-fraction/precipitation, plus a scale-and-offset from scene-absolute
world XZ to field UV, the three band centre altitudes, whether the renderer may resolve genus from
it (see
[derived genus](#14-the-cloudscape-window-and-genus-as-a-derived-label-atmosphere-phase-b1)),
and the altitude span the genera it resolves to occupy. Borrowed rather than copied because
`Environment` is copied per frame while the payload changes on a multi-second cadence; the sim owns
the storage (`Simulation::WeatherFieldBuffer`) and `revision` is how the renderer notices a change.

**Who fills it.** `IWeatherProvider::publish_field`, on the interface rather than behind a
capability query because every implementation can answer it truthfully: `ProceduralWeather`
publishes its grid, `IngestedWeather` samples its own station blend onto a lattice (its structure
is real but implicit in its query), and `StaticWeather` publishes one column everywhere, which is
the whole truth about a manually authored sky.

**The scene mapping**, stated once in
`engine/world/simulation/include/SushiEngine/simulation/weather_field_buffer.hpp` and shared by
`RuntimeSimulation::geodetic_at_scene`: the scene is a flat tangent patch anchored at the sky
observer's geodetic position, with **+X east and +Z north**. Not a new convention — the rain
emitter's wind drift already drove `WindSample::eastward_mps`/`northward_mps` into exactly those
axes. Eastward metres carry a `1/cos(latitude)` factor because the simulation grid's lattice is
plate carrée (`x = R · longitude`).

**Transport.** `WeatherFieldPass`
(`engine/presentation/render/source/passes/weather_field_pass.{hpp,cpp}`) uploads the field into a
fixed 64×64×3 `R8G8B8A8_UNORM` **3-D** image — 3-D rather than a 2-D array so hardware trilinear
blends the three vertical bands as a march sample climbs through them, in the same single fetch the
horizontal interpolation already costs. The image is always the full square and coarser producers
are resampled up on the host, so its view and descriptor are fixed for the renderer's lifetime;
staging is a three-buffer ring indexed by the frame's own slot, and the image is cleared once so a
scene that never publishes weather still binds a defined resource. It registers before the cloud
bakes that read it.

**Consumption.** Phase A read the field *per march sample*, turning its coverage into a scale about
the column the deck stack was compiled from and applying it as a re-threshold. That mechanism is
gone (see
[the window](#14-the-cloudscape-window-and-genus-as-a-derived-label-atmosphere-phase-b1)):
the field is now an input to the cloudscape **bake**, which resolves coverage and genus per baked
column, so the march reads the answer rather than the meteorology and the light volume, the shadow
map and the panorama inherit the same answer instead of each needing their own lookup. The scene
uniforms carrying the field's addressing ride the block's tail
(`engine/presentation/render/shaders/scene_weather_tail.glsl`, included by every shader that needs
to reach them), with the camera position folded into the map's offset in double.

**The column is now sampled at the camera**, not at the scene's geodetic anchor — the same point
while the weather was uniform, not the same point now. The deck stack, the fog, the wetness, and
the rain therefore all describe where the player stands.

Phase A deferred three items together on the record — the field stayed a wrapping tile, so the
light volume and the cloud shadow map could not consult it and genus could not become a derived
label.
[The cloudscape window](#14-the-cloudscape-window-and-genus-as-a-derived-label-atmosphere-phase-b1)
is that one change.

**Substitutability, repaired.** `ISimulation` used to return a concrete `ProceduralWeather*` and
the host stored that type, which is why `IngestedWeather` was written, tested, and uninstallable.
The host now holds `std::unique_ptr<IWeatherProvider>` through a single `install_weather_provider`,
`IWeatherProvider::tick` lets it advance time without knowing what it installed, and the editor
reaches authoring through the optional `IWeatherAuthoring` capability (`weather_authoring()`),
which a provider fed by real observations simply does not offer.

`tests/unit/test_weather_field.cpp` (`Unit_WeatherField`) defends the claim that matters — a
published field is non-uniform when the simulation is — plus the addressing, every provider's
ability to publish, and (since the window phase) the producer's own genus authority. The acceptance
bar has not yet been confirmed by eye in a live editor session.

### 1.4. The cloudscape window, and genus as a derived label (atmosphere phase B1)

`docs/design/atmosphere_system.md` §7.2/§7.4.
[The spatial weather field](#13-the-spatial-weather-field-atmosphere-phase-a) gave the renderer a
spatial field but read it per march sample, on top of a bake that was still one globally compiled
deck stack painted over a periodic tile. This subsection replaces both halves of that: the tile
becomes a camera-centred window, and the deck stack stops deciding what a march sample finds.

**Why the tile had to go.** The baked field was addressed by `fract(p.xz / tile)`. `fract` is
many-to-one, so a bake addressed that way holds a `uv` with **no recoverable world position** —
which is why `CloudLightVolumePass` and `CloudShadowMapPass` could not consult a spatial weather
field at all, and why per-column genus was impossible (every texel evaluated the same decks). Being
camera-relative rather than world-anchored, the tile also travelled with the camera. One property —
invertibility of the address — was blocking all three.

**Two windows.** `engine/presentation/render/shaders/cloud_field_window.glsl` is the single place
their addressing lives, shared by the view march, the light volume, the shadow map, the far-field
panorama and the ground/mesh shadow lookup, so those five cannot disagree about which piece of the
world a texel is. A **near** window keeps the old 32 768 m span and 128 m texel; a **far** window
covers 262 144 m at ~1 km per texel. Two rather than one because the march reaches fourteen shell
thicknesses (~150 km): a single window with that reach would put ~600 m between texels next to the
camera. They cross-fade across the near window's outer rim, and because both are baked from the
same weather field they agree about *where* the weather is and differ only in how finely they carve
its shape — which is what makes the hand-off read as detail falling away with distance rather than
as a ring.

**The derived bakes needed no lookup of their own.** Once the light volume and the shadow map
address window space, the field they march already carries the simulation's coverage per column, so
both became spatially correct by addressing the right texels. A clearing stops casting shadow
because there is no cloud in the field there to cast it.

**Genus, derived.** `Render::classify_cloud_genus` / `cloud_band_towers` /
`cloud_genus_thresholds`
(`engine/domain/environment/include/SushiEngine/environment/environment.hpp`) are the one
authority. The GPU bake resolves a genus per baked column from the three simulated bands and
evaluates that genus's own `cloud_genus_profile`; a cumulonimbus is added *on top of* the low band
where deep convection supports one, rather than replacing it, so a cumulus field with one storm
growing out of it stays representable.

`WeatherCloudscapeCompiler` resolves the same label through the same function for the editor
readout and METAR-style reporting — its three hard-coded `if` chains (the OCP finding in the design
doc's §1.6) are gone, and its thresholds are uploaded to the GPU as data so a retune is a data
change rather than two edits that can disagree. The classifier's *branch structure* is the one
thing that exists twice, in C++ and GLSL, and the two copies name each other.

`Render::WeatherField::derives_genus` is how the **producer** says whether its column state is
meteorology at all: `StaticWeather`'s column *is* an authored deck stack decomposed into bands, and
re-classifying it would overrule the author, so it answers no and the bake stays on the authored
stack. `union_base_m`/`union_top_m` are taken by the producer as it fills the cells — the only
party that sees every column — because with genus per column the march shell is a property of the
field, not of the deck stack: a cumulonimbus growing 300 km away still has to be inside the span
the march crosses.

**The far window's own light.** `engine/presentation/render/shaders/cloudscape_far_light.comp`
writes optical depth toward the sun into the far field's `b` channel, because
`CloudLightVolumePass` covers only the near window and lighting everything past it from that
window's edge would show a front's shape while lighting it flat. Source and destination are two
images rather than one read-modify-write, since the march reads texels its neighbouring
invocations write.

**Placement and cadence.** `CloudscapeCompilePass::update_window` runs between the scene uniform
fill and its upload — the bake and every consumer of the bake read the same mapping out of that
block, and the pass that owns the window state cannot reach it once uploaded, so the pass keeps the
state and the scene view owns only the ordering. Origins snap to each window's own texel lattice,
which is what makes re-centring invisible: the bake is a pure function of the pattern position, so
a window that moved a whole number of texels reproduces the identical value at the identical world
point.

A rebake is triggered by an authored change, a moved march shell, the camera drifting 8 % of the
span out of the window, a weather cadence (1 s near, 4 s far — the published field is interpolated
every frame between the simulation's own 15 s ticks, so "the weather changed" is continuously true
and cannot gate anything alone), and, for the far window's sun channel, the sun moving a degree.
**The wind has no trigger of its own**: it is expressed purely as a shift of the window's origin,
absorbed by the bake and republished as a residual each frame, so the sky advects continuously
between rebakes with no staleness — the lookup slides, the content does not go out of date.

**What this let go.** The per-sample coverage-scale/reference-column mechanism
(`cloud_weather_field.glsl`, `WeatherField::reference_coverage`) is deleted; so is the march's
mirrored anti-repetition tap, which existed to hide a tile period there is no longer any of. The
field's two density channels collapse to one — the cumuliform/cirriform split existed for
per-layer anti-repetition offsets that were never actually distinct, and every consumer summed the
pair — and freeing the march's weather-field binding is what made room for the far window in the
last per-pass image slot. Samplers move from `REPEAT` to `CLAMP_TO_EDGE`.

**Named limit.** Ground more than ~16 km from the camera loses its cloud shadow: the map covers
exactly the near window and fades out across its rim. It had one before, but it was a repeated copy
of the camera's own patch, and at that distance a wrong dappling reads as texture where a missing
one reads as haze. A second, coarser shadow cascade over the far window is the natural follow-up.

### 1.5. The regional nest: anelastic convection on the GPU (atmosphere phase B2)

`docs/design/atmosphere_system.md` §6. [World coupling](#11-world-coupling-phase-w5) and
[data ingestion](#12-flight-sim-polish-and-data-ingestion-phase-w6-final) describe a simulation
whose own audit (§1.3–§1.5 of the design doc) found three independent 2-D layers with no vertical
advection at all, saturation expressed as `if (humidity > 0.85)`, no latent heating,
semi-Lagrangian advection at Courant ≈ 0.02 — its maximally diffusive regime — and all of it in a
single-threaded scalar CPU loop. This subsection replaces it.

**The model.** Prognostic potential temperature, three moisture species (`q_v`, `q_c`, `q_r`) and a
staggered Arakawa C-grid velocity field, over a camera-following domain of 192×192×48 cells at 2 km
horizontally and ~54–560 m vertically. Ten compute stages per step: shift, advect velocity, advect
scalars, forces, divergence, pressure, project, microphysics, extinction, readback.

- **Transport** is MacCormack-corrected semi-Lagrangian with a monotone limiter, at Courant ≈ 1
  chosen from the flow's own speed. The limiter clamps the correction to the upstream stencil's
  range, which is what stops an unlimited scheme manufacturing negative mixing ratios and spurious
  cold pools.

- **The anelastic constraint** is `∇·(ρ̄u) = 0` — the reference density, not the true one, which is
  what filters sound waves out and therefore why the step is set by advection rather than by the
  speed of sound. That single approximation is what the whole cost argument rests on.

- **Buoyancy** is `B = g(θ_v′/θ̄_v − q_c − q_r)`: thermal buoyancy through the *virtual* potential
  temperature, minus the weight of the water the cloud is carrying. Latent heat released by
  condensation raises θ′, which raises B, which lifts more vapour to its condensation level. That
  closed loop is why a cumulus grows rather than merely existing, and it cannot be written down
  without a vertical velocity to feed it.

- **Microphysics** is Kessler warm rain: Magnus saturation, a one-Newton-step saturation adjustment
  (the fixed point moves with the temperature the latent heating changes), autoconversion,
  accretion, sub-cloud evaporation, and terminal-fall-speed sedimentation. Cloud base is nowhere
  placed — it falls out where a rising parcel's `q_s(T)` drops to its own `q_v`, which is the
  lifting condensation level by definition.

- **Cloud-top radiative cooling** is the sink a cloud has and the clear air beside it does not: a
  cloud top is optically thick in the longwave with space above it, so it loses ~70 W/m² across a
  layer tens of metres deep. Carried as the flux *difference* across a level rather than a local
  `dF/dz`, which telescopes down a column to `F0·(1 − e^{−κW})` and so cannot radiate away more
  than the top is given at any vertical resolution — the differential form over-cools an optically
  thick level by its own opacity.

  It also cannot radiate *forever*: the sky above returns a growing share of what the top emits as
  it cools, so the loss is scaled by how far the top still is above the temperature at which the two
  balance and is gone at `cloud_top_equilibrium_depression` below ambient. Without that bound the
  constant is a sink rather than a flux, and nothing in this model warms a cloud — a deck that
  persisted cooled its column 42.7 K over 72 h and did not stop. The entrainment the cooling drives
  is resolved rather than parameterized, and is under-resolved at the spacing the upper levels
  carry.

- **The other direction of the same longwave** reaches the ground: the covered share of the sky the
  surface balance radiates to is the cloud base's own emission rather than Brutsaert's clear-sky
  air, blended by cover and by how black the deck is. A cloud base holding 30 g/m² is 0.98 of a
  black body and it is warm, so an overcast column that is shading the sunlight away is also
  returning most of the loss — applying the clear-sky value under it leaves the ground losing tens
  of watts to a sky that is not there. The cloud-base temperature and liquid water path come out of
  the column walk the extinction stage already runs for the shading.

- **The Rayleigh sponge** hangs from the rigid lid over the upper *half* of the domain — an edge at
  9 km, not the quarter-domain edge at 13 km a cloud model usually takes — and the difference is a
  measurement. Its `sin²` ramp is exactly zero below its lower edge, so a mode that sits underneath
  it is not damped at all however large the rate is: one parked at 12.4 km, immediately below the
  13 km edge, reached ±13 K over 72 h and was still growing, and doubling the rate moved it to
  ±11 K. Covering those levels leaves +0.02 K. Depth is the parameter that decides this and rate is
  not. Deeper is not free either — an edge at 6 km collapses the wind between 4.6 and 9 km to
  0.02 m/s and reverses θ′ at 6 km, which is the sponge standing in for the weather; at 9 km the
  boundary layer, the cloud deck and everything below 4 km are unchanged to within 0.08 K.

- **The boundary layer** is parameterized, because the eddies that carry surface heat and moisture
  out of the lowest level are two orders below a 2 km grid. Vertical diffusion of total θ and the
  moisture species on Troen & Mahrt's `K = κ·w_s·z·(1 − z/h)²` — linear in height at the ground,
  where surface-layer similarity requires it, and peaking at `h/3` — over a depth diagnosed per
  column by the parcel method and capped by `boundary_layer_depth_m`. The depth carries a floor of
  the two lowest levels: mechanical turbulence at the ground does not switch off with the
  stratification, and without the floor a level that is momentarily colder than the one above it
  decouples entirely and then accumulates without limit — measured at −8.76 K, which fogged the
  level.

- **Subgrid cloud fraction** sits on top of the adjustment and is what lets a grid-mean model draw
  a cumulus at all. A cell's humidity is a *mean* over 4 km²; a fair-weather cumulus is a
  200 m–1 km thermal, saturated inside while the cell around it is not. So the cell's humidity is a
  top-hat distribution of half-width `(1 − critical)·q_s` about that mean (Sommeria–Deardorff;
  Smith 1990's uniform member), condensation takes its saturated tail, and the fraction and the
  condensate come out together. At `cloud_critical_humidity = 1` it collapses exactly onto the
  all-or-nothing adjustment it generalises, so there is one condensation path and not two. The
  fraction rides in the moisture volume's fourth channel and the extinction volume's alpha, and it
  is what the cloudscape bake now thresholds against — cell-mean σ divided by it is the in-cloud
  water the bake draws at.

**The pressure solve** is the piece worth reading. The grid is deliberately anisotropic, so the
Laplacian's vertical coupling outweighs its horizontal by `(Δx/Δz)²` — two orders of magnitude near
the ground — and a point smoother would converge appallingly. It solves the vertical *exactly* with
a Thomas sweep per column and iterates the horizontal by red-black colouring. Naive Thomas wants
four private arrays, a kilobyte per column, which spills at a 64-column workgroup; everything but
the `c′` factors is recomputed from analytic functions of the level index and `d′` is written
straight into the pressure image, which is 256 bytes instead. Named limit: with no coarse-grid
correction the smooth horizontal error decays slowly, so a fixed sweep count leaves a small
residual divergence — a slight mass imbalance, not a wrong answer. Semi-coarsened multigrid with
this shader as its smoother is the refinement and drops in unchanged.

**Where it lives, and why that was forced.** The editor runs three `ISceneView`s (Scene, Game, VFX
preview). A per-view nest would simulate three divergent atmospheres at three times the cost and
several hundred megabytes, and the simulation would have to pick one to answer "what is the
weather". So `AtmosphereNest`
(`engine/presentation/render/source/atmosphere/atmosphere_nest.*`, public state in
`engine/domain/environment/include/SushiEngine/environment/atmosphere_nest.hpp`) is a device-level
service in `AssetLibrary`, beside the cloud noise, built on first use so a scene that never enables
weather never pays its ~135 MB. It is centred on the **simulation's observer**, not on any camera,
which decouples it from views entirely.

**Ordering against three readers.** The step records into its own command buffer, submits on the
graphics queue and signals a timeline semaphore each view's first submission waits on. Without it,
three views sampling a resource an earlier submission is writing is a race the validation layers
cannot see. `AtmosphereForcing` carries an *absolute* game clock rather than a per-frame delta, so
the step is idempotent and whichever view reaches it first does the work.

**Seeding is re-centring.** A shift larger than the domain leaves every cell without a source,
which is exactly "fill from the base state and the parent solution" — one code path, not two that
would eventually disagree about what a fresh cell contains. Surviving cells are copied, not
resampled, because the lattice snaps to whole cells against an absolute origin.

**The data plane.** Nothing reads the GPU state synchronously (the design doc's §3.2).
`engine/presentation/render/shaders/atmosphere_readback.comp` compiles the nest into a 32×32
lattice of `WeatherColumn`-shaped records, copied into a triple-buffered host ring and taken once
per completed step. `Render::IAtmosphereMirror` is the one seam that flows renderer → simulation;
`IAssetLibrary` *is* that source, so a host binds the two in one line
(`simulation->set_atmosphere_mirror(&renderer->assets())`). This is the design doc's §9.2 query
mirror pulled forward from phase E, deliberately and only as far as it had to be: the moment the
grid moved onto the GPU there was nothing left on the CPU for `sample_column` to read. The design
doc's §9.1 full `AtmosphereProfile` stays in phase E, where it has consumers written for it.

**The profile beside the columns.** The same dispatch also writes `AtmosphereProfileLevel` — the
observer column's *unreduced* vertical state, published on `AtmosphereMirror` alongside the coarse
records. Every field of a `WeatherColumn` is already a vertical reduction, which is what gameplay
wants and is exactly what a column that refuses to condense has destroyed the evidence in: a log of
exact zeros is the case where the only remaining question ("why") is answerable solely by height.

Sixteen floats a level over at most 64 levels is 4 KB against the columns' 64, written from samples
the dispatch was already fetching. It carries state and not diagnosis — θ′, the three mixing ratios
and the base state's own vapour beside them, the diagnosed cloud fraction, the wind, the extinction,
the buoyancy the level would feel and the divergence the projection was handed — because a derived
summary is what the columns already are. The Meteorology panel renders it level by level, cloudy
rows tinted, which is where a clear sky is diagnosed without leaving the editor.

**`sushiengine_atmosphere_probe`** (`tools/probes/atmosphere/main.cpp`) drives all of it
headlessly: a Vulkan device with no window, the nest stepped through hours of game time in seconds
of wall clock, and the profile written to CSV. A measuring instrument rather than a test — it
asserts nothing, and `tests/unit/test_atmosphere_nest.cpp` is where the pinning lives. It exists
because the questions this tier raises are of the form "what is it doing, and at what height",
which no pass/fail answers and no editor session answers quickly; parameter overrides
(`--sensible`, `--latent`, `--seed`, `--eddy`, `--pbl-w`, `--critical`, `--sweeps`) let one term be
separated from the rest by running without it — which is how all three of the boundary-layer
defects fixed on 2026-07-28 were found. Beside the column it reports the whole mirror's sky:
cloudy-column fraction, mean coverage and mean cloud base, because one 2 km column is a noisy
sample of an intermittent field.

**Before the first readback**, and in a host that never binds a mirror, every weather query answers
from the base state: a clear sky with the synoptic wind. Honest rather than convenient — nothing
has been simulated, so there is no condensate to report, and inventing a coverage there is
precisely the fabricated signal the design doc's §1 audit was written about.

**What T1 is.** The parent solution the Davies relaxation zone nudges toward is
`Atmosphere::QuasiGeostrophicCore` (phase C): real geostrophic flow around lows nothing placed,
with the boundary's temperature and moisture anomalies taken as departures from the core's own
zonal mean — the nest's base state already carries the mean, so what the parent must supply is the
eddy. The interim that stood here, `SynopticLayer`'s stylized frontal mask, is gone; cyclogenesis
is emergent and fronts are diagnosed from the thermal gradient rather than drawn from a ray pair.

**Retired here.** `regional_weather_grid.hpp` (the design doc's own disposition table) and
`test_weather_determinism.cpp` — bit-exact replay of the weather is given up deliberately (the
design doc's §0 and §14), and the test's subject no longer exists.
`tests/unit/test_atmosphere_nest.cpp` replaces it by pinning what *is* still checkable: the base
state against the International Standard Atmosphere, Magnus saturation against its textbook values,
the stretched grid closing on its domain top, and the mirror's cold start and transcription.

**Cloud shape now comes from the condensate (the design doc's §7.1/§7.3, phase B2b).**
`engine/presentation/render/shaders/cloudscape_field.comp` has a third path, taken whenever the
nest is running: it reads `σ_ext` from
`engine/presentation/render/shaders/atmosphere_extinction.comp`'s field directly and uses no genus,
no deck and no height gradient at all. A cumulus has a flat base because the condensation level is
flat, not because `cloud_height_gradient` draws one; an anvil spreads because the updraft spread
it. The nest's stretched levels invert analytically (`w = (altitude/top)^(1/1.5)`), so altitude →
texture W is one `pow`, and the horizontal map is stamped in `update_window` because the nest is
centred on the *simulation's* observer while the scene block is camera-relative — that is the one
place both frames are in hand.

Baked density is stated against the extinction of 1 g/m³ of liquid water rather than in absolute
1/m. That keeps the medium's authored absorption meaning exactly what it always meant: *where* the
cloud is and how its density falls off across its own edge is now physics, while how opaque a given
amount of water reads stays an authored property of the medium. Tiled noise survives only as a
sub-2 km modulation at 35 % amplitude — the design doc's §7.3 demotion in its final form, since the
nest resolves 2 km and that is all the noise is still needed for.

The march shell follows suit: `WeatherFieldBuffer::fill_from_mirror` takes the lowest cloud base
and highest cloud top the readback reports and publishes those as the union span, so the baked
field's thirty-two vertical texels sit on the cloud instead of on the empty stratosphere above it.
A classifier could only ever have said "a cumulus reaches 3.2 km" because the catalogue says so;
the nest knows where *this* cloud's top is.

The three paths are ordered: nest first, then per-column genus from the published field, then the
authored deck stack. Each is the truthful answer when the one above it has nothing to say.

**Two invariants the bake now enforces on itself.** Both were violated for as long as the bake has
existed, and neither is visible without measuring the noise rather than looking at it.

*`coverage` is a percentile, so the field it cuts must be uniform.*
`remap(base_shape, 1 - coverage, 1, 0, 1) * coverage` only means "keep the densest `coverage` of the
sky" if `base_shape` is uniform on [0, 1]. Ported exactly and sampled, it is a narrow bell with
support [0.574, 0.906], so the cut never reached the field above ~0.43 coverage and every deck at
half coverage or more came out a gapless slab shaped only by `cloud_height_gradient`.
`uniform_shape()` pushes it through its own CDF (a logistic approximation to the normal one, one
`exp`) before anything thresholds it. The transform is monotone, so it cannot change the field's
level sets — it changes which one `coverage` selects, which was the entire defect. Statistics are
per-path because the deck path mixes two taps of the shape volume, the nest path takes one, and
cirriform genera read a different volume.

*A window may not carve a feature its own grid cannot sample.* The shape volume lays four Worley
cells per `shape_scale`, so a cloud is `shape_scale / 4` — 1050 m for cumulus, against the far
window's 1024 m texel. One sample per cloud, and since the bake thresholds *before* it stores, the
alias arrives with its gaps filled rather than blurred. `min_shape_scale()` floors the scale at
sixteen texels of whichever window is being baked. The near window's floor is 2048 m and every
genus is above it, so the near field is untouched and the far field draws cloud clusters instead of
a slab.

**The Meteorology panel** (`applications/editor/source/atmosphere/meteorology_panel.cpp`) is the
tuning and logging surface for all of this: the sky's animation rate against the rate the nest can
actually step, the solar forcing, the observer's column, and a CSV log sampled on the *nest's* clock
rather than the wall clock so runs at different time scales stay comparable. When the column is
empty it names which link is broken — procedural weather off, nest disabled, forcing unpublished,
or stepping without a completed readback — because the nest is built lazily behind several
conditions and "no readback yet" on its own sends you to the wrong subsystem. The readback's three
spare `extent` lanes carry surface relative humidity, the column's peak |w|, and the lifting
condensation level: diagnostics the renderer does not consume, but the only ones that answer *why*
a column is clear rather than merely *that* it is.
