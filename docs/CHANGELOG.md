# Changelog

All notable changes to SushiEngine are documented here.

Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) — versions follow [Semantic Versioning](https://semver.org/).

---

## [Unreleased]

### Added
- 2026-07-28 — Added a subgrid cloud-fraction closure to the regional nest. A cell's humidity is now a top-hat distribution about its mean rather than a single value, so condensation takes the saturated part of it instead of waiting for the whole 4 km² cell to cross saturation; cloud fraction and condensate are diagnosed together and the fraction reaches the renderer, which thresholds against it and draws the in-cloud water rather than the cell mean. Authored as `cloud_critical_humidity` (editor: *Cloud From RH*), which at 1.0 collapses exactly onto the all-or-nothing adjustment it generalises.
- 2026-07-28 — Added a "Render path" section to the Meteorology panel, naming which rung between condensate and pixels is the one that is empty: clouds switched off, no field published, a field not marked as meteorology, a march shell with no height, or a healthy shell with its span. All five previously produced the same symptom — a blue sky — which is also the symptom of no condensate at all, and that ambiguity cost more time in this phase than any of the physics.
- 2026-07-28 — Added land-cover presets to the Meteorology panel's surface forcing — semi-desert, mixed cropland, vegetated summer land, open water — each setting the sensible/latent flux pair that cover actually delivers, with the sky it produces measured in its tooltip. Nobody authors a scene by choosing watts per square metre; what a place *is*, for this model, is its Bowen ratio, and that one split decides whether an afternoon reaches its condensation level or merely gets hotter. Mean sky coverage over eleven simulated hours runs 0.00 / 0.04 / 0.16 / 0.45 across the four.
- 2026-07-28 — Added a Bowen-ratio readout to the Meteorology panel, flagged when it exceeds 1. It is a derived number so nothing showed it, and it is the one that decides whether a heated boundary layer reaches its condensation level or merely gets hotter: at the authored 1.4 the nest makes no cloud in eleven simulated hours, and at 0.48 it makes a cumulus deck over 88 % of the domain at 1341 m.
- 2026-07-28 — Added a domain-wide sky summary to `atmosphere_probe` — cloudy column fraction, mean coverage and mean cloud base over the whole mirror — because a single observer column is a noisy sample of a 192² field and "can I see clouds" is a question about the sky.
- 2026-07-28 — Added the observer column's vertical profile to the Meteorology panel: every level's temperature, θ′, relative humidity, vapour, condensate, cloud fraction, vertical wind and extinction, with cloudy levels tinted. The nest has read this back since the profile was added, but seeing it meant leaving the editor for the headless probe.
- 2026-07-27 — Added a boundary-layer mixing scheme to the regional nest: vertical eddy diffusion of potential temperature and the moisture species over a mixed-layer depth diagnosed per column by the parcel method, capped by `boundary_layer_depth_m` and scaled by `boundary_layer_diffusivity`, both exposed in the editor. Without it the surface fluxes accumulated in the lowest 54 m and nothing convected.
- 2026-07-27 — Added `atmosphere_probe`, a headless driver that steps the regional nest through hours of simulated weather in seconds of wall clock and writes the observer column's full vertical profile to CSV, with parameter overrides for isolating one term at a time.
- 2026-07-27 — Added `Render::AtmosphereProfileLevel`, the observer column's unreduced vertical state, published on `Render::AtmosphereMirror` by the same readback that fills the coarse columns.
- 2026-07-27 — Added atmosphere phase B2b: the regional nest's optical extinction now drives cloud shape directly, with no genus, deck or height gradient in the path.
- 2026-07-27 — Added a Meteorology panel (`Window ▸ Meteorology`) for tuning the nest and logging the observer's column to CSV on the nest's own clock.
- 2026-07-27 — Added atmosphere phase B1: baked the cloudscape into two camera-centred, non-wrapping windows (near/far) instead of a periodic tile, and derived cloud genus per column from the simulated field.
- 2026-07-27 — Added atmosphere phase A: coupled the weather simulation to the sky — the cloud march samples the simulation's own horizontal grid (`Render::WeatherField`), applies coverage as a re-threshold, and samples the driving column at the camera instead of the scene anchor.
- 2026-07-26 — Added editor UI for GTAO, SSR, ray-traced shadows, particle bursts, and blend shapes — engine features that were already implemented but unreachable from the Inspector.
- 2026-07-26 — Added physics broadphase/collision, animation authoring tools, particle mesh rendering, and UI subsystem expansions.
- 2026-07-25 — Added VFX particle material features: sprite textures through the bindless heap, flipbook atlases, soft particles, and per-emitter lit shading.
- 2026-07-25 — Added audio codec/HRTF/acoustics hardening: Vorbis and streaming-compressed codecs, MagLS/anthropometric HRTF, ray-traced acoustics, and an authoring DAW panel.
- 2026-07-25 — Added a Game view no-camera placeholder with an aspect/orientation/fullscreen toolbar.
- 2026-07-25 — Added audio ECS integration (phase S6): emitter/listener/reverb-zone components and a wall-clock snapshot extract bridging the ECS to the audio engine.
- 2026-07-25 — Added a reverb system and GPU-driven per-particle punctual lighting.
- 2026-07-24 — Added animation, audio, input, and VFX systems with full editor integration, including headless S0 vertical slices for audio/input and an action-mapped input layer.
- 2026-07-24 — Added an upscaler seam, scene-interop groundwork, and shader enhancements (Phase 11).
- 2026-07-23 — Added Saturn's rings to the analytic celestial-body sky.
- 2026-07-23 — Added Phase 6 lighting: three-cascade probe-volume GI (4/8/16 m spacing) and emissive materials that light the scene through GI.
- 2026-07-23 — Added lunar/solar eclipse light-loss dimming the whole scene.
- 2026-07-23 — Added the post-processing stack (Phase 9) and GPU-driven geometry (Phase 10): per-mesh-bucket instancing, indirect draws, and single-phase GPU occlusion culling.
- 2026-07-22 — Added a Vulkan 1.4 baseline: a background PSO optimizer and a descriptor-writer seam.
- 2026-07-22 — Added Phase 3 advanced passes: GTAO, Hi-Z, clustered lighting, and SSR.
- 2026-07-22 — Added Phase 7 atmosphere: the Hillaire LUT stack and volumetric fog.
- 2026-07-21 — Added the Phase 3 foundation: the quality-tier contract, SH-9 IBL, and `ViewResources`.
- 2026-07-21 — Added the render graph architecture, PBR materials, IBL, and TAA; added astro reference frames and gravity; added shader hot-reload, a disk-backed pipeline cache with pipeline-library linking, and a bindless descriptor heap.
- 2026-07-04 — Added PBR materials, a WGS84 planet, atmosphere, clouds, and stars.
- 2026-07-04 — Added collision shapes (Sphere/Plane/Box/OBB via SAT), soft bodies, a UI framework, two-way cloth↔rigid contact coupling, and a sweep-and-prune broadphase.
- 2026-07-03 — Added selection outlining and Hierarchy reordering.
- 2026-07-03 — Added primitive objects (Box/Sphere/Cylinder/Terrain), colliders, and cloth rendering.
- 2026-07-03 — Added SushiLoop M4 (loopback network layer) and M5 (cloth grids over the XPBD solver).
- 2026-07-03 — Added a scene-less editor start — the live world now starts empty instead of seeding demo cubes — plus unsaved-scene tracking and a confirm-before-close prompt.
- 2026-07-03 — Added Hierarchy multi-select; fixed the Project panel's double-click handling.
- 2026-07-03 — Added undo/redo and New Scene.
- 2026-07-03 — Added `.sushiscene` save/open.
- 2026-07-03 — Added hierarchy parenting via drag-and-drop.
- 2026-07-03 — Added a Unity-style Project window with double-click open and a relocated project root.
- 2026-07-03 — Added rotate/scale gizmos, Local/World space, wheel-zoom, and middle-mouse pan.
- 2026-07-03 — Added a Camera component with per-display selection; the Game view no longer picks.
- 2026-07-03 — Added a Preferences window with persisted settings.
- 2026-07-03 — Added net reconciliation wired into a live client/server demo.
- 2026-07-03 — Added cloth grids wired into `RuntimeSimulation` and the Inspector.
- 2026-07-02 — Added a translate gizmo for the selected entity.
- 2026-07-02 — Added GPU id-buffer picking with a selection highlight.
- 2026-07-02 — Added an entity-aware editor over the live world, with the world as the single source of truth.
- 2026-07-02 — Added the Game view as a second viewport driven from the world camera.
- 2026-07-02 — Added a live ECS world on SushiRuntime behind an `ISimulation` seam.
- 2026-07-02 — Added a Scene viewport with a Unity-style fly camera.
- 2026-07-02 — Added Vulkan presentation for the editor.
- 2026-07-02 — Added a Unity-style editor panel set.
- 2026-06-28 — Added the `se` developer CLI and the functional test suite.
- 2026-06-26 — Added a graph-coloured Projected Gauss-Seidel physics solver.
- 2026-06-26 — Added the WP-3 archetype ECS layer.
- 2026-06-26 — Added the milestone-A headless core.

### Changed
- 2026-07-04 — Changed `Scalar` precision from a build-selectable option (`SE_SCALAR_DOUBLE`, added 2026-07-02) to always `double`, with no switch; moved rendering to camera-relative space.
- 2026-07-03 — Changed editor/engine naming to PascalCase namespaces and no-abbreviation identifiers; reorganized the editor layout.
- 2026-07-02 — Changed cloud noise generation to run on the GPU (Perlin-Worley shape, erosion detail).

### Fixed
- 2026-07-28 — Fixed the atmosphere nest's vertical diffusivity weakening exactly where it had to be strongest. The parabolic profile went as `K_peak·(z/h)` near the ground, so a *deeper* mixed layer mixed its own surface more weakly — and the lowest face carries the entire surface flux out of a 54 m level. Measured: 12 m²/s there, the surface level 9 K above the one 80 m up, the layer never homogenising, and its top reaching 57 % relative humidity after eight hours of heating. Replaced with Troen & Mahrt (1986), `K = κ·w_s·z·(1 − z/h)²`, whose near-surface slope does not depend on the layer depth; `boundary_layer_diffusivity` (m²/s) accordingly becomes `boundary_layer_velocity_scale` (m/s).
- 2026-07-28 — Fixed the atmosphere nest's thermal seed being an unbounded random walk. It added a fresh perturbation to potential temperature every step, so across 37 000 columns its cold tail kept a few percent of the domain in permanent ground fog which the sky then reported as cloud. It now modulates the surface heat flux, which is where surface heterogeneity physically lives: bounded by construction, and it stops at dusk with the flux it scales. `thermal_seed_amplitude` becomes a dimensionless patchiness (editor: *Surface Patchiness*) rather than a rate in K/s.
- 2026-07-28 — Fixed a level that is momentarily colder than the one above it decoupling from the atmosphere nest's boundary layer entirely: the parcel test correctly found the column stable, diagnosed zero mixing depth, and left the level exchanging with nothing — so the thermal seed's random walk accumulated in it without limit, measured at −8.76 K after four hours at 99 % relative humidity while the level above sat at 50 %. The diagnosed depth now carries a floor of the two lowest levels, which is the surface layer's mechanical turbulence and does not switch off with the stratification. Every cloud the nest made before this was that fog.
- 2026-07-28 — Fixed the cloudscape bake eroding the nest path's *density* instead of its *shape*, which turned any thin deck into flickering specks. The fringe erosion subtracts an absolute threshold averaging 0.225, written for the deck path where density is an authored quantity of order one; the nest path's density is measured, and a marginal fair-weather deck (14 % cloud fraction, 0.01 g/kg, sitting at the closure's critical humidity) arrives at 0.058 — so the erosion drove 87 % of its texels to zero and left the noise's own peaks, which changed every rebake. The carve and the erosion now run on the dimensionless shape and the measured water multiplies once at the end.
- 2026-07-28 — Fixed the Meteorology panel's clock verdict and its "Match sky to atmosphere" button assuming 60 fps. A session running at about twelve was told "the atmosphere is keeping up with the sky" while four of every five seconds of weather were being discarded. Both now use the measured ratio of weather asked for to weather simulated, over a recent window rather than the session total so that correcting the rate actually converges.
- 2026-07-28 — Fixed the atmosphere nest's cloud base and top being found by an optical-depth threshold while coverage came from the cloud fraction. The two disagreed on thin decks — a band could report 5 % coverage, cloud the bake does draw, while no level crossed the optical threshold — so the march shell excluded it and clipped away exactly the cloud the field had been given. Both now read the fraction.
- 2026-07-28 — Fixed the atmosphere nest's band coverage reporting opacity rather than coverage. `1 − exp(−optical depth)` is one wherever a column holds any real cloud, so a scattered cumulus field a quarter of the way across the sky was reported to gameplay as overcast; it is now the maximum-random overlap of the levels' own cloud fractions.
- 2026-07-28 — Fixed the nest's boundary-layer parameters not being serialized with the scene, so `boundary_layer_depth_m` and `boundary_layer_diffusivity` silently reverted to their defaults on load.
- 2026-07-27 — Fixed `se build` reporting a test setting it did not apply: it only configured when the build type or generator changed, so a cache left by `se render` or `se editor` could hold `SE_BUILD_TESTS=OFF` while the header said ON — the suite then never compiled and `se test` passed against a stale binary. It now reconfigures in place on every build.
- 2026-07-27 — Fixed `test_atmosphere_nest.cpp` failing to compile against `AtmosphereForcingBuffer::view`'s current signature, and retargeted two `test_weather_field.cpp` cases that still asserted the CPU weather grid phase B2 deleted; both had been invisible while the suite was not being built.
- 2026-07-27 — Fixed the atmosphere nest's boundary layer never moistening: the surface latent flux added a third of one unit in the last place per step to a half-float moisture volume, so every step rounded back to where it started and vapour stayed bit-identical to the base state indefinitely. The moisture volume is now full floats (+28 MB at High). This is why relative humidity fell, the condensation level receded, and no column could ever make cloud.
- 2026-07-27 — Fixed the atmosphere nest advecting potential temperature without its stratification term, which left the whole domain with a Brunt–Väisälä frequency of zero: a rising parcel never lost buoyancy and convection had no equilibrium level.
- 2026-07-27 — Fixed the atmosphere nest's Coriolis parameter never reaching the GPU — it was declared and documented as riding the forcing but never assigned, so `f` was identically zero.
- 2026-07-27 — Fixed the atmosphere nest reading its pressure, divergence and surface-rain volumes before anything wrote them; the pressure solve warm-starts from its own previous field, so undefined contents propagated instead of being overwritten.
- 2026-07-27 — Fixed cloud coverage having almost no effect: the shape field is a narrow bell, not the uniform distribution the coverage threshold assumes, so every deck at half coverage or more rendered as a gapless slab. Authored coverage and density defaults will need retuning against the corrected field.
- 2026-07-27 — Fixed the far cloudscape window rendering as a featureless white square by flooring each window's shape scale at what its own texel spacing can sample.
- 2026-07-27 — Fixed the atmosphere nest's buoyancy using total water vapour where the virtual-temperature term needs the departure from the base state, which put a spurious signal six times the size of the real one into the pressure solve.
- 2026-07-27 — Fixed volumetric fog blinding the camera whenever the sky was overcast, and stopped enabling authored fog as a side effect of switching weather on.
- 2026-07-26 — Fixed remaining device-loss paths (LUTs, MRT blend, IBL descriptors).
- 2026-07-26 — Fixed functional CI by installing POCL, and fixed push-constant/image-layout validation errors.
- 2026-07-26 — Fixed CI flakiness by pinning the intel/llvm nightly to an older, more POCL-5.0-compatible build.
