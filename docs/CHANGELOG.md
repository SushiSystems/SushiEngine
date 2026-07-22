# Changelog

All notable changes to SushiEngine are documented here.

Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) — versions follow [Semantic Versioning](https://semver.org/).

---

## [Unreleased]

### Fixed
- **`World::get<T>()` no longer silently dereferences a null column.** Creating a
  primitive (e.g. sphere/box) in the editor could crash with an access violation
  because `Chunk::column()` returns `nullptr` when an entity's archetype has no
  column for the requested component, and `get<T>()` dereferenced that pointer
  unconditionally. Both `get<T>()` overloads in `world.hpp` now assert on a null
  column so a component/archetype mismatch fails loudly in debug builds instead of
  corrupting memory silently.
- **Stochastic sampling no longer prints static speckle over shadows and clouds.**
  Every pass that trades filter taps for noise — the soft-shadow disc rotation, the
  cloud march's start offset, the contact-shadow march — hashed its per-pixel noise
  from the pixel position alone, so the pattern never changed between frames and the
  temporal resolve converged *onto* the speckle instead of averaging through it:
  penumbrae and cloud edges came out permanently dotted. The hashes were also white
  noise, which clumps neighbouring pixels into visible grain. Both are replaced with
  interleaved gradient noise advanced per frame (`temporal_dither` in
  `temporal_common.glsl`) — driven by the sub-pixel jitter where the temporal block is
  bound, and by the scene clock in the contact pass, which binds only the scene and
  shadow blocks. With anti-aliasing off the jitter is zero and the noise holds still
  instead of shimmering unresolved.
- **The sun no longer lights meshes after it has set.** Direct light in `pbr.frag`
  asked only whether the surface faced the sun, so a favourably tilted mesh stayed
  fully lit with the sun below the horizon. The direct term is now occluded by the
  planet itself: the sun fades out as it crosses the geometric horizon of the shaded
  point — a horizon that dips with altitude, so a mountaintop keeps the sun slightly
  longer than the valley below it — over a small band standing in for the solar disc
  and refraction.

### Changed
- **The GPU profiler moved into the Statistics panel.** The per-pass timing breakdown
  was an overlay drawn over each viewport's image; the main loop now copies both
  visible viewports' timings into the editor context each frame and the Statistics
  window lists them alongside frame time and FPS.

### Added
- **Atmosphere is a Hillaire 2020 LUT stack, not a per-pixel march (Phase 7).** The
  sky's Rayleigh/Mie scattering now precomputes into a shared set of lookups a new
  `AtmosphereLutPass` owns, replacing the full-resolution single-scatter integral the
  sky shader used to run at every pixel. Four resources: a **transmittance LUT**
  (256×64, optical depth from any altitude and sun angle to space) and a
  **multiple-scattering LUT** (32×32, the infinite-order isotropic scattering the single
  march omitted) — both view-independent, rebuilt only when the medium or the planet
  radii move; a per-frame **sky-view LUT** (192×108, the background sky's in-scatter in
  the camera's local frame), so a pixel with no geometry is one fetch; and a per-frame
  **aerial-perspective froxel volume** (32×32×32, camera-frustum-aligned), so a mesh
  reads the air in front of it as one 3D fetch. `sky.frag` selects among them: background
  → sky-view LUT, mesh within 32 km → aerial volume, analytic ground and far geometry →
  a march that now reads the sun's transmittance and the multiple scattering from the
  LUTs instead of a per-sample light ray. The IBL cube capture keeps the march (its
  viewpoints do not match the camera-aligned volumes). Builders and samplers share
  `atmosphere_common.glsl`, so the parameterization can never drift. The captured
  environment (IBL) now also reads the transmittance/multi-scatter LUTs, so it tracks
  the same scattering. Net-negative GPU cost — the sky's heaviest per-pixel work becomes
  texture reads.
- **Volumetric fog.** A ground-hugging participating medium marched into a
  camera-frustum froxel volume (`VolumetricFogPass` + `fog_scatter.comp`): the extinction
  falls off exponentially with altitude, and each froxel gathers the sun (phase-weighted,
  attenuated by the atmosphere transmittance LUT) plus a constant ambient fill, its in-
  scatter and transmittance folded over every pixel in the composite. **Local fog
  volumes** (box/ellipsoid primitives, up to eight) blend extra density into the same
  grid for valley or airfield fog — data, not new passes. Authored through an
  `Environment::fog` block and a **Fog** section in the Environment panel (density,
  height falloff, colour, ambient, sun anisotropy, and the local-volume list). Tier-gated
  off on Low (`QualityParams::volumetric_fog`); a no-op when disabled. Sun-shadowed god
  rays and punctual-light fog are later increments.
- **The renderer's floor is Vulkan 1.4.** Drivers have been conformant industry-wide
  since 2025, so `maintenance5`, `maintenance6`, and push descriptors are now required
  rather than probed, and the fallbacks they obsoleted are gone. Texture streaming
  gains a `host_image_copy` path (probed, stays optional): a CPU-built mip chain is
  copied straight into the optimal-tiled image with no staging buffer, queue submit,
  or fence, gated on `SHADER_READ_ONLY` actually appearing in the device's
  `pCopyDstLayouts`; the staging-plus-blit path is the fallback where it doesn't.
  Scene set 0 is now bound with push descriptors instead of an allocated set, so the
  per-frame allocate/update/bind that used to run ahead of every graphics pass is
  gone across all eleven of them.
- **A swapped-in pipeline no longer costs a hitch.** `GraphicsPipelineFactory` used to
  hand back whatever `VK_EXT_graphics_pipeline_library` fast-linked, which is quick to
  build but not the fastest pipeline the driver can produce. It now hands out a
  `PipelineHandle` pointing at that fast-linked pipeline immediately, while a
  background thread rebuilds the same pipeline monolithically and atomically swaps
  the handle over once it's ready; the superseded pipeline retires once every view
  sharing the factory's clock has had time to stop referencing it.
- **Every descriptor write and set bind now goes through one seam.** New
  `DescriptorWriter` and `bind_descriptor_set()` centralise what used to be a direct
  `vkUpdateDescriptorSets`/`vkCmdBindDescriptorSets` call in each pass, so the
  announced `VK_EXT_descriptor_heap` (Roadmap 2026) will be a swap behind these two
  functions rather than a sweep of every call site. `VK_EXT_descriptor_buffer` is
  deliberately not adopted — a dead end the seam doesn't need.
- **Diffuse image-based lighting is now spherical harmonics, not a cubemap.** The
  captured environment is projected into 9 second-order SH coefficients by a single
  compute dispatch folded into the (change-gated) IBL build, and the shading pass
  evaluates a degree-two polynomial in the surface normal instead of sampling a filtered
  irradiance cube — nine storage reads in place of a cube fetch. The cosine-lobe
  convolution and the 1/π are baked into the coefficients, so the result matches the old
  cube exactly for a uniform environment; the small negative lobe SH can ring into is
  clamped. The coefficients ride a frame-global storage buffer the IBL pass owns and
  keeps fragment-readable, which is also what makes the coming irradiance-probe blending
  a blend of coefficients rather than of cubemaps.
- **The quality tier now drives every major pass.** `RenderQuality` (Low / Medium / High /
  Ultra) was a near-inert enum read in exactly one place; a single resolver
  (`render/frame/quality.cpp`, behind the public `QualityParams`) now turns it into the
  concrete parameters each pass reads — soft-shadow tap counts, contact-march length,
  cloud march budget, the coarsest variable-rate-shading tile, the shadow atlas size and
  cascade count, and which advanced BRDF lobes (anisotropy, clearcoat, sheen,
  transmission) are evaluated at all. A pass consumes *resolved parameters*, never the raw
  tier. The contract is deliberate: the authored settings **are** the High baseline, so
  High renders exactly what the host asked for, a lower tier scales the expensive half
  down, and Ultra pushes it up — dropping a tier moves the whole red line at once instead
  of switching features off one at a time. The Rendering panel gains a "Tier resolves to"
  readout showing what each pass is actually handed, so switching Low↔Ultra visibly
  rescales the passes there and, in milliseconds, in the profiler HUD.
- **Cascaded shadow maps for the sun.** Up to four cascades in one two-by-two atlas —
  one image, one pass, one barrier, one profiler entry. Splits follow the practical
  scheme (a blend of logarithmic and uniform); each cascade is bounded by a *sphere* and
  its light-space origin snapped to whole shadow texels, which together are what stop the
  edges crawling as the camera turns. Fitted entirely in camera-relative space, so a
  scene six million metres from the world origin builds its shadow maps out of small
  numbers. Percentage-closer filtered through a hardware comparison sampler, offset along
  the geometric normal rather than biased into peter-panning, cross-faded between
  cascades and faded out at the last one. The filter is percentage-closer **soft**
  shadows: a blocker search measures how far above each receiver its occluders stand and
  the penumbra is that gap times the sun's angular radius, so a contact stays crisp and a
  silhouette high above the ground goes diffuse — the way a real sun shadow does. Sixteen
  Poisson taps rotated per pixel, which the temporal resolve averages into a smooth
  penumbra far more cheaply than the tap count it would take to be smooth alone. The
  texel snap is anchored to the **world**, not to the camera-relative frame the fit works
  in, since a grid snapped in that frame travels with the camera and stabilises nothing.
  The analytic planet ground receives them too,
  not only the meshes — it is the surface most of the scene stands on, and an object
  casting onto nothing reads as floating over its own terrain. The atlas therefore has a
  frame-global binding of its own rather than one of a pass's local image slots, since
  the sky pass has all six of those spoken for.
- **A depth prepass.** It runs the same `mesh.vert` the shading pass does with no
  fragment stage, which is a correctness requirement and not an optimisation: the opaque
  pass then tests against depths it recomputes itself, and only the same shader
  guarantees the two agree bit for bit. It also rejects occluded fragments before the
  material shader runs on them.
- **Screen-space contact shadows.** A short march of the depth buffer toward the sun,
  bounded in metres rather than pixels, recovering the centimetres of contact a cascade
  texel is far too coarse to resolve. This is what the depth prepass exists for — the
  answer has to be known before a surface is shaded.
- **Clouds shadow lit surfaces.** The sky pass already marched the cloud medium to
  shadow the analytic ground; a mesh standing on that ground now darkens with it, taking
  the same quantity from the weather map for two fetches instead of an eight-step march
  through three volumes. Same parameterisation, warp included, so both agree on where a
  cloud is.
- **`ShadowSettings`** on `RenderSettings`, and the editor Rendering panel exposes it:
  cascade count and resolution, distance, split blend, both biases, filter radius,
  cascade blend, and the contact march.
- **Ray-traced sun shadows, on the Ultra tier.** A two-level acceleration structure —
  one bottom-level per distinct mesh built once and kept, one top-level rebuilt each
  frame from instance records — and a ray query traced from a fullscreen pass into a
  screen-space visibility mask. Tracing asks the geometry directly, so there is no
  cascade resolution, no boundary, no acne and no peter-panning; where it runs it
  replaces the cascades rather than joining them, and resolves contact on its own.
  Camera-relative like everything else: an instance transform has the eye subtracted in
  double before the float cast. Writing a mask rather than tracing inside the material
  shader is what keeps that shader one build for every device — turning the tier on adds
  a pass, it does not fork the shading path. Needs `VK_KHR_acceleration_structure` and
  `VK_KHR_ray_query`; without them the pass never
  registers and the cascades are the whole story. The analytic planet ground is
  ray-marched rather than rasterised and is not in the structure, so it keeps the
  cascades.
- **Motion vectors.** The geometry pass writes a third target: each pixel's screen
  displacement since the previous frame, in UV. The current clip position already
  exists; the previous one comes from a per-frame array of previous transforms that a
  draw indexes with a single push-constant slot — the same shape as the material array,
  and for the same reason. Both sides are camera-relative, each against its own frame's
  eye, so the camera's own translation appears in the motion vector without planet-scale
  metres ever entering single precision. Pixels with no geometry are reprojected from
  the view ray instead, which is exact for the sky.
- **Temporal anti-aliasing.** A jittered Halton projection, a history accumulated at
  the output resolution, and a resolve with velocity dilation, YCoCg neighbourhood
  clipping, Catmull-Rom history reconstruction, tone-weighted blending, and a sharpen
  that offsets the temporal softening. Cheaper than MSAA at these resolutions and able
  to remove shimmer MSAA cannot — specular, alpha-tested, and the ray-marched sky, which
  is now jittered with the rest of the frame.
- **Temporal upscaling.** Because the history lives on the output grid and every frame
  samples a different sub-pixel position, rendering below the output resolution and
  resolving into it recovers detail rather than blurring. `RenderSettings::render_scale`
  is the manual lever; a vendor upscaler lands as a new `UpscaleMode` and a new pass.
- **Dynamic resolution.** A governor drives the internal render scale from the GPU time
  the Phase 0 timers measured, dropping resolution quickly when the budget is blown and
  recovering gradually so it cannot oscillate. Steps are quantised to sixteenths, and
  the temporal upscale is what makes the change hard to see.
- **FXAA fallback.** A purely spatial filter over the encoded image for the low tier
  and for a host that wants no frame-to-frame history at all. It shares nothing with the
  temporal path — no motion vectors, no history, no jitter.
- **Variable rate shading.** A compute pass derives a per-tile rate mask from the
  previous frame's luminance contrast and this frame's motion, and the sky pass — the
  frame's heaviest fill at planet scale — binds it. Ignored on a device without
  `VK_KHR_fragment_shading_rate`, so no pass branches on support.
- **`RenderSettings`**, a public seam separate from `Environment`: that describes the
  world being drawn, this describes the machinery drawing it. Quality tier,
  anti-aliasing mode, upscale mode, render scale, and the temporal, dynamic-resolution,
  and shading-rate parameters. `ISceneView::set_settings` applies it; the editor's
  editor gained a **Rendering** panel that edits it — its own window rather than a
  section of Environment, on the same principle that separates the two structs. It also
  reports the resolution the governor settled on, which is the only way to watch it
  work.
- **Textures, and a material model with Unity-Standard parity.** `Render::Material`
  moves to its own header and grows the full authoring surface: albedo, packed
  metallic-roughness (ORM supported), normal with a bump scale, height with
  **parallax occlusion mapping** (self-shadowing and silhouette clipping included),
  occlusion, emission with an HDR colour and intensity, a Unity-style **detail set**
  (albedo, normal, mask), and per-set **tiling/offset**. Rendering state — surface
  type and cutoff, cull mode, blend mode, render queue, shadow flags, wrap mode,
  anisotropic filtering — is authored alongside. Every map is optional: an unset slot
  resolves to a neutral default texture, so a material with no textures shades exactly
  as it did before textures existed. A new inspector lays all of it out in the order
  an artist expects.
- **A GPU material system.** Authored materials are packed once per frame into a
  storage-buffer array of fixed-layout records holding bindless heap indices, and a
  draw carries a single material index in its push constant instead of a payload of
  parameters — the shape the indirect-draw work in a later phase needs.
- **Texture library with mip generation and streaming.** Images decode through stb,
  upload with a GPU-generated mip chain, and register into the bindless heap; loads
  deduplicate by path. Residency is mip-based against a budget: an over-budget texture
  appears at a lower resolution and is upgraded later, at most one per frame, and no
  upload ever blocks the frame.
- **glTF 2.0 mesh and material import.** Every primitive in a file becomes one mesh,
  baked into its node's world transform so multi-part assets assemble without a scene
  graph on the render side. The core material maps across directly and the
  `KHR_materials_*` extensions (clearcoat, sheen, transmission, volume, ior,
  anisotropy, emissive_strength) drive the advanced lobes; spec-gloss is converted;
  missing tangents are generated. `MeshInstance::mesh` draws an imported mesh in place
  of a primitive, and `IWindowRenderer::assets()` is the seam a host loads through
  without seeing a Vulkan type.
- **Image-based lighting, captured from the engine's own sky.** Six faces of the
  analytic atmosphere are rendered into a cubemap, GGX-prefiltered into a roughness
  mip chain, and cosine-convolved into an irradiance cube; a split-sum BRDF LUT is
  generated once at bring-up. This replaces the flat ambient constant, so indirect
  light tracks the time of day for free. The capture is rate-limited and only runs
  when the sun or the atmosphere has measurably moved.
- **BRDF upgrade.** Height-correlated Smith visibility, **Kulla-Conty multi-scatter
  energy compensation** (rough metals no longer lose most of their energy),
  roughness-aware Fresnel, specular occlusion from AO, and dielectric F0 derived from
  the material's index of refraction. Anisotropy, clearcoat, sheen, and thin-surface
  transmission are available behind material flags, costing nothing when unused.

### Changed
- **The cloud composite left the display transform.** Resolving the half-resolution
  cloud march over the sky used to be two lines at the top of `tonemap.frag`. It is its
  own pass now, because the temporal resolve needs a complete linear HDR scene and the
  display transform has to run after that — nothing may sit on both sides of it.
  `tonemap.frag` is exposure, the tone curve, and the encode, and nothing else, which is
  also the seam the post-processing stack will attach to.
- **Vertex format upgraded to position, normal, tangent, UV0, UV1, and vertex
  colour.** The built-in primitives are regenerated with analytic tangents and UVs;
  imported meshes get generated tangents when a file omits them. A zero tangent is a
  legal value meaning "none authored", and the shader then derives a frame from
  screen-space derivatives, so a mesh without tangents still normal-maps.
- **Device-level assets moved out of the scene view.** Textures, meshes, the bindless
  heap, and the shader/pipeline/sampler caches now live in one `AssetLibrary` per
  device, so two viewports drawing the same model share one upload and one pipeline.
  The scene view keeps only what varies per view: its targets, its per-frame
  allocators, its soft-body buffers, and its passes.

- **A render graph, and the renderer split onto it.** `render/` is no longer one
  ~2300-line class that hand-writes every barrier: passes now declare which textures
  and buffers they read and write, and `render/graph/` derives the rest — every
  `VkImageMemoryBarrier2`, every dynamic-rendering scope (viewport and scissor
  included), which passes are dead and can be culled, and which transient targets may
  share one allocation because their lifetimes do not overlap. The existing four
  passes ported onto it unchanged, plus the picking readback, as one file each under
  `render/passes/` behind a single `register_pass(graph, frame)` contract, so a new
  effect is a new file rather than an edit to the god-class. `VulkanSceneView` is left
  as a thin orchestrator; camera-relative rendering and reverse-Z are unchanged and
  inherited by every pass.
- **Per-pass GPU timings in the editor.** The graph brackets each pass with timestamp
  queries and resolves them at the point the frame's fence has already been waited on,
  so the measurement costs no stall. `ISceneView` gains `pass_timing_count()` /
  `pass_timing()`, and the Statistics panel lists the breakdown per viewport — every
  later pass can now be landed against a measured budget instead of a guess.
- **Shader hot-reload.** Shaders still ship as build-time SPIR-V, but when the source
  tree is present the renderer watches `render/shaders/` and recompiles changed files
  in process with glslang, idles the device, and rebuilds every pipeline. A compile
  error keeps the previous shader and reports on stderr. GLSL `#include` now works in
  both the build-time tool and the reload path, and the shader compiler accepts
  compute shaders.
- **Disk-backed pipeline cache and pipeline libraries.** A `VkPipelineCache` is
  persisted between runs so a second launch skips driver shader compilation, and where
  `VK_EXT_graphics_pipeline_library` is available a pipeline is linked from four
  independently cached halves so pipelines differing in one stage reuse the other
  three. Devices without the extension fall back to monolithic creation.
- **Bindless descriptor heap.** One global update-after-bind array of textures and
  storage buffers, bound as set 1 of the shared scene pipeline layout when the device
  supports descriptor indexing, with the cloud noise volumes registered into it.
  Per-pass descriptor sets are now allocated from per-frame pools that are reset
  wholesale, so resizing the viewport rebuilds no descriptor set.

### Changed
- **Cloud noise is generated on the GPU.** The Perlin-Worley shape, erosion detail,
  anisotropic cirrus, and weather-map textures were built at startup on a
  `std::thread` pool; they are now four compute dispatches in a single fenced submit,
  taking that work off the host's bring-up path.
- **Soft-body vertex buffers are per frame slot.** The cloth buffers were rewritten
  every frame while a previous frame could still be reading them; each frame in flight
  now owns its own, and they still only ever grow.
- **Test coverage for the solar-system model, camera maths, and determinism guard
  rails.** The `astro/` module (Julian date and sidereal time, Keplerian orbital
  elements, the body catalogue, the summed gravity field and symplectic integrator,
  the topocentric/body-orientation/surface/scene frames, the star catalogue, and the
  `advance_astro_state` propagator + `fill_environment_sky` assembler) shipped with no
  tests; it now has twelve dedicated suites (ten `Unit_*`, two `Integration_*`) that
  verify it against astronomical reference landmarks (J2000 = JD 2451545.0, Earth ~1 AU,
  WGS84 radii, ~9.8 m/s² surface gravity, Earth's ~0.9 Gm sphere of influence) and
  against structural invariants — Kepler's equation satisfied, every frame transform
  round-tripping to the identity, every basis right-handed orthonormal, and the
  integrator conserving orbital energy over a full revolution. Adds `Unit_MathPrimitives`
  for the previously-untested quaternion/matrix/`perspective`/`look_at` seam in
  `core/types.hpp`, and `Unit_Rng` / `Unit_FixedTimestep` for the SushiLoop determinism
  contract (identical seeds replay identically; a snapshotted RNG replays the future
  exactly; the fixed-step count depends only on total elapsed time, not on frame
  chunking). Shared test helpers gain a `WorldVector3` comparator and `is_unit` /
  `is_orthonormal_basis`. All fifteen new files register into `se_functional_tests` and
  pick up their CTest labels from the suite-name convention, so `ctest -L unit` and
  `ctest -L integration` run them with the rest.
- **Frame-local transform authoring: place things relative to a celestial body.** The
  inspector's Transform section gains a **Reference** row — a body dropdown ("Scene" or any
  catalogued body) plus a **Auto / Free / Surface** mode. With a body picked, Position and
  Rotation are authored *frame-local*: **Surface** shows the position as **Latitude / Longitude /
  Altitude** (the place-on-a-planet coordinate, via `surface_frame.hpp`'s geodetic conversion and
  the body's spin `W(t)`), with ground-local rotation so "upright" is identity at any pole or
  hemisphere; **Free** shows a Cartesian offset from the body's scene-frame centre. This is the
  Unity-parent analogue with the body as the parent — so the whole solar system is placeable
  through one Transform without ever typing a raw ~1e11 m number. New `IWorldEditor` surface:
  `Simulation::EntityFrame` / `Simulation::FrameMode`, `entity_frame` / `set_entity_frame`,
  and `frame_local_transform` / `set_frame_local_transform`, which convert between the
  frame-local pose and the scene Transform through the scene-frame bijection at the master
  epoch (`reference_center_scene`). This is an **authoring-boundary projection**: the scene
  Transform stays the working truth the physics and renderer read, and the reference descriptor
  is what's serialized (and, ahead, network-synced as the frame-independent value for
  zero-conflict). A reference body of -1 is the scene root — entities that never pick a body are
  unchanged. Old scenes' removed `has_astro_body` flag migrates to a Free reference on the
  dominant body. Surface mode drives the existing surface anchoring.
- **Solar-system dynamics modules: the gravity field, orbital integrator, and scene
  bijection.** The astronomy that lets an entity move under real solar-system gravity, as
  reusable modules. These back the per-body physics gravity field (see *Changed*) and,
  ahead, the unified dynamic body's Free authority. (An earlier exclusive "Astro Body"
  toggle that owned an entity's pose through a parallel path was folded into the one
  dynamic-body concept before release — there is no separate astro mode, component, or
  serialization key.) The modules:
  - `astro/gravity_field.hpp` — `Astro::IGravityField` (the field behind a dependency-
    inversion seam) and `Astro::SummedRailsGravityField` (the default on-rails summation),
    so the orbital integrator names an abstraction, not a concrete model (a patched-conic
    or full N-body field can replace it without the integrator changing).
  - `astro/astro_dynamics.hpp` — a field-parameterised `Astro::integrate_step` overload
    and `Astro::advance_astro_state()`, which joins one symplectic step and the SOI rebase
    into the single per-step authority update the simulation drives.
  - `astro/scene_frame.hpp` — `Astro::SceneFrame`, the exact rigid bijection between a
    heliocentric-ecliptic position and the scene's local frame (position and direction,
    both ways), reproducing the ephemeris's scene construction so a free body and the
    planet it orbits line up. `astro/topocentric.hpp` factors the observer's East-Up-South
    basis (`LocalSkyBasis`, `local_sky_basis`, `to_local`, and its new inverse `from_local`)
    out of `ephemeris.hpp` so the ephemeris and the scene frame share one construction;
    `astro/body_orientation.hpp` gains the inverse rotation `body_equatorial_to_ecliptic`.
- **The simulation owns the master epoch (one clock for orbits and sky).** `ISimulation`
  gains `julian_date()` / `set_julian_date()` / `set_time_scale_days_per_second()` and
  `set_sky_observer()`: the sim advances the Julian Date by the fixed step (scaled by the
  authored days-per-second) and the editor now drives the sky from that clock instead of a
  separate accumulator, so free bodies, the planets they orbit, and the rendered sky are
  all evaluated at the same instant. The sky animates under Play/Step; scrubbing the date
  seeks the epoch.
- **Planet-agnostic sky: three coordinate spaces and per-body spin.** The topocentric
  ephemeris was written around Earth — the scene was always anchored to an Earth-geodetic
  observer, positions rotated into Earth's equatorial frame, and the meridian driven by
  Earth's sidereal time — so standing on another planet gave a broken, Earth-locked sky
  (animated time showed no motion, and flying to a distant planet lost precision as its
  coordinates reached ~1e11 m in the Earth-anchored frame). The sky is now built the same
  way on any body:
  - `SkyObserver` gains `observer_body` (the ephemeris body index the observer stands
    on, Earth by default); `fill_environment_sky` anchors the scene origin to that body's
    surface point and places every other body relative to it, in that body's own
    equatorial frame — one construction, no Earth special case.
  - New `astro/body_orientation.hpp`: `body_rotation_angle()` is the IAU prime-meridian
    angle W(t) (each body's spin, retrograde where real), `ecliptic_to_body_equatorial()`
    / `equatorial_to_body_equatorial()` rotate directions into the observer body's
    equatorial frame. Earth is routed through the exact fixed-obliquity path and sidereal
    time, so the home sky is bit-identical; every other body picks up its true pole and
    day. This realises the **solar → planet → local** space contract: heliocentric
    ecliptic (all bodies), body-fixed per-planet, and the observer's local scene frame.
  - The editor re-anchors to whichever body the camera is on (the dominant near-field
    body becomes `observer_body`), rebasing the camera on a change so the switch is
    seamless. This replaces the Earth-only "ride-along" shim; "animate time" now shows the
    sky move on every planet, and approaching any planet keeps full precision.
- **Solar-system gravity field and planet-relative surface frames.** Three new
  header-only modules under `include/SushiEngine/astro/` give the orbital regime a
  real gravity model and a planet-relative transform basis, all double precision at
  the render/sim seam:
  - `astro/gravity.hpp` — `Astro::standard_gravitational_parameter()` (GM per body),
    `Astro::gravity_field()` (the summed Newtonian field of every catalogue body at
    its ephemeris position), a symplectic velocity-Verlet `Astro::integrate_step()`
    that flies a `StateVector` through that field, and `Astro::sphere_of_influence_radius()`.
    Planets stay on their analytic Keplerian rails (the gravity *sources*); only free
    entities are integrated (the *subjects*) — an on-rails design that keeps the field
    a deterministic function of position and time for lockstep.
  - `astro/reference_frame.hpp` — `Astro::ReferenceFrame` (a body-centred, inertially
    oriented frame), `frame_for()`, `to_frame()`/`to_heliocentric()`, `rebase()` (the
    coordinate change a sphere-of-influence crossing triggers), and
    `active_frame_body()` (the most local dominant attractor at a point).
  - `astro/surface_frame.hpp` — the body-fixed surface frame that makes a
    planet-relative pose work: `geodetic_to_body_fixed()`/`body_fixed_to_geodetic()`
    (ECEF storage with lat/lon only as a boundary conversion), `geodetic_normal()`,
    `local_tangent_basis()` (the East-North-Up rotation an entity's orientation is
    composed onto, so "upright" is identity everywhere on a body — no
    hemisphere-dependent tilt), `surface_gravity()`, and `surface_gravity_vector()`.
  All three are fully body-parametric — every catalogued body from Mercury to Pluto,
  plus the Moon, with no per-body branches.
- **Planet-aware simulation gravity.** `RuntimeSimulation` now derives each step's
  gravity from the scene's dominant celestial body: the real surface acceleration of
  Earth, Mars, the Moon, or any catalogued body, directed at the planet's centre
  rather than a fixed world "down". With no dominant body it falls back to the legacy
  local demo gravity, leaving the plain physics sandboxes unchanged.
- **Planet-relative orientation (surface anchoring).** An entity can be surface-anchored
  through `IWorldEditor::set_surface_anchored` / `set_surface_local_orientation`: its
  stored orientation is treated as *ground-local* (relative to the local East-North-Up
  tangent frame on the dominant body), and the simulation composes the tangent frame
  onto it each step. "Upright" is therefore identity everywhere on a body — an anchored
  entity stands straight in the southern hemisphere and at the poles instead of tilting
  with its position. Anchoring seeds the ground-local orientation from the entity's
  current pose so it does not snap. Exposed in the editor as a **Surface Anchor**
  component (Add Component menu, removable Inspector section) and persisted with the
  scene.
- **Planet collider for entities and the Scene camera.** With a dominant body present,
  `RuntimeSimulation` keeps every entity — cameras included — outside the reference
  ellipsoid each step and after every edit, pushing a sunk position radially out to the
  true (flattened) surface; a penetrating rigid body is re-posed through the physics
  seam so its velocity zeroes and the surface acts as a hard floor.
- **Genus-driven, self-shadowing volumetric cloudscape.** The single cloud shell is
  replaced by a `Render::Cloudscape` of up to `CLOUD_MAX_DECKS` genus decks. A new
  `Render::CloudGenus` enumerates the ten WMO genera (cirrus, cirrocumulus,
  cirrostratus, altocumulus, altostratus, nimbostratus, stratocumulus, stratus,
  cumulus, cumulonimbus); `Render::cloud_genus_profile()` is the catalogue mapping
  each to a physically-plausible `Render::CloudGenusProfile` (étage band, coverage,
  density, a `stratiform` axis from cellular billows to flat sheet, an `anvil`
  spread for deep convection, noise scales, wind, and base-noise kind). A
  `Render::CloudDeck` names a genus and nudges its coverage/density; several decks
  coexist as a real sky does. The sky pass ray marches the union shell of all
  enabled decks in one pass and sums their densities, so decks composite and occlude
  one another and a sample's light march toward the Sun crosses every deck above
  it — high cirrus shadows the cumulus below it for free — and casts the combined
  shadow onto the analytic ground (`ground_shadow_strength`). The base shape gains a
  wind-stretched anisotropic `Cirriform` volume so cirrus reads as feathers, and the
  cumulonimbus anvil towers to the tropopause. **Two-tier LOD**: the march budget
  scales with camera altitude (dense in-atmosphere, coarse from orbit), so tall
  cumulonimbus resolves from space at low cost; **empty-space skipping** strides the
  air gaps between decks so samples land in cloud. Exposed in the editor's
  Environment → Clouds panel as a shared "Medium" section plus a genus combo per
  deck. **Breaking:** `Render::CloudParams` is removed in favour of
  `Render::Cloudscape` + `Render::CloudDeck` + the genus catalogue.
- **Dynamic night lighting from the Moon and stars.** A new
  `Render::NightLighting` (`enabled`, `moon_intensity`, `star_intensity`) lets the
  ephemeris drive `Environment::ambient` each frame from the real sky geometry:
  the Sun's ambient dominates by day and hands off, as it sets, to a cool
  moonlit ambient scaled by the Moon's illuminated fraction (its phase) and
  altitude, plus a small starlight floor on a moonless night. Exposed in the
  editor's Environment panel under a new "Night Lighting" section; only active
  while the astronomical Sun drives the sky, so a manually authored ambient is
  left untouched otherwise.
- **The camera rides a moving planet.** `Environment` now reports the analytic
  ground body (`dominant_body_id`, `dominant_center_metres`); the editor shifts
  the camera by that body's scene-frame displacement each frame so it stays at a
  fixed altitude over a non-Earth planet as the planet's orbit carries it through
  the Earth-anchored frame while time animates (Earth was already tracked by its
  observer-origin anchoring).

### Changed
- **Volumetric clouds read with real 3D volume instead of soft repeating blobs (phases 0–1).**
  Two root causes of the synthetic look were fixed. *Frequency/scale:* the shape noise volume is
  now `128³` and the detail volume `64³` (was `96³`/`32³`), and the cumuliform genus profiles'
  `shape_scale`/`detail_scale` were retuned so a texel lands near cloud-feature scale — crisp
  cauliflower silhouettes carved by fine erosion rather than kilometre-wide lumps. *Repetition:*
  the shader's domain warp (`sky.frag`) is now two incommensurate octaves at wavelengths far
  larger than the shape tile, bending the noise lattice off its grid so no lumps land at the tile
  spacing the eye latches onto. *Lighting depth:* the light march toward the sun is now
  cone-sampled with exponentially growing steps (crisp local self-shadow, soft outer shadow), and
  a 3-octave Beer/Wrenninge multiple-scattering energy term (`cloud_sun_energy`) fills the deep
  interior a single Beer term leaves black — dense cores read luminous and self-shadowed, the
  main cue that a cloud has volume. Design notes in `docs/design/volumetric_clouds.md`.
- **Clouds now draw over terrain, not only against open sky.** The cloud march was gated on
  `!ground_hit`, so any ray that met the planet ground drew no cloud — flying above a deck and
  looking down showed bare ground. The march now runs whenever clouds are enabled, entering at
  the shell's top sphere and stopping at the nearer of the opaque geometry or the planet ground,
  compositing the cloud over the ground colour already accumulated. Essential for the flight-sim
  down-looking view.
- **Realistic cloud distribution and a first optimization pass (phase 2 + 4a).** *Distribution:*
  the weather/coverage field is now a broad weather-system scale times a finer clumping scale
  sampled stretched along each deck's wind, so the sky breaks into wind-parallel clusters with
  clear gaps (cloud streets) instead of an even carpet, and a broad self-warp hides the map tile.
  *Optimization (toward mid-range GPUs):* a cheap density probe skips the expensive detail/dual-
  scale evaluation in the empty air that dominates grazing and look-down rays; samples past a
  distance threshold drop to the cheap density (distance LOD); the cone light march is recomputed
  every other lit sample and reused between (it is the dominant cost); light taps 6→5; earlier
  transmittance cutoff; max view-march budget 128→96.
- **Half-resolution cloud pass (phase 4b).** The volumetric march is split out of `sky.frag`
  into a dedicated `cloud.frag` that renders at half width and height into an HDR target
  (`scattered.rgb`, `transmittance`); the tonemap pass upsamples it (linear) and composites it
  over the full-resolution sky as `sky * transmittance + scattered`. The expensive march now runs
  at a quarter of the pixels while the sun disk, stars, and planet relief stay sharp — the largest
  single GPU saving for the cloudscape. The cloud density helpers remain in `sky.frag` for the
  ground-shadow cast. (Depth-aware/temporal upsampling to remove half-res edge softening is a
  future refinement.)
- **Clouds read white, not grey.** With a strongly forward-scattering phase the clouds only lit
  up when looking toward the sun and read grey otherwise. `cloud_sun_energy` now blends its
  deeper multiple-scatter octaves toward isotropic and rescales by 4π, so a fully-lit sample
  approaches the sun's radiance (white) from any angle while the light march still greys down the
  self-shadowed side; the powder darkening default drops 1.0→0.3 and ambient fill 0.4→0.5.
- **One-click weather presets in the cloud panel.** `Render::WeatherPreset`
  (Clear / Fair Weather / Overcast / Storm) and `cloud_weather_preset` — a pure-data expansion
  mirroring `cloud_genus_profile` — set the whole sky from one button. The editor's Clouds panel
  leads with the preset buttons and tucks the per-deck and medium controls under an *Advanced*
  node, so the common case needs no manual deck editing.
- **Gravity is a per-body field, not one scene-wide vector.** `IPhysicsSimulation::step` now
  takes a `Simulation::GravitySampler` (a `Vector3(const Vector3&)` mapping a body's position to
  its acceleration) instead of a single gravity vector, and the solver samples it at each body's
  own position every sub-step (`PhysicsWorld::predict_substep_field`). `RuntimeSimulation` builds
  the sampler from the injected `Astro::IGravityField` — the *same* summed on-rails field the
  orbital integrator uses (one gravity source) — by mapping each body's scene position to
  heliocentric, sampling the field, and rotating the acceleration back into scene axes. Bodies
  now feel the true field: 1/r² falloff with altitude, curvature toward the attractor, and
  third-body terms, rather than one dominant term evaluated once at the scene origin. Sampling at
  the current position each sub-step keeps the semi-implicit predict symplectic (orbit energy is
  preserved). Non-astronomical scenes get a uniform-sampler fallback, unchanged. `PhysicsWorld`'s
  own `step`/`predict_substep` (used by the demos and tests) keep the uniform-vector form.
- **Scene surface anchoring uses the geodetic normal for "up".** The local East-North-Up
  tangent frame an anchored entity's orientation composes onto now takes its vertical from
  the ellipsoid gradient (`RuntimeSimulation::surface_normal_scene`, the scene-frame analogue
  of `Astro::geodetic_normal`) instead of the geocentric radial `normalize(offset)`. On a
  flattened body the two differ, so a surface-anchored entity now stands along the true local
  vertical everywhere — a step toward folding all tangent-basis math onto
  `surface_frame.hpp`.
- **Rigid-body aerodynamic drag.** A Rigid Body gains a **Drag Coefficient** (`k`), applied in
  the physics predict as a quadratic drag acceleration `-k|v|v` each sub-step (`Physics::predict`),
  so a body reaches terminal velocity under gravity instead of accelerating without bound. Threaded
  through `Simulation::PhysicsBodyParams`, `RigidBodyDesc`/`RigidBodyT`, and
  `update_rigid_body_params` (live edits apply without a rebuild); exposed as an inspector field and
  serialized. `0` disables it, so existing bodies are unchanged.
- **The Scene fly-camera is no longer clamped to planet surfaces.** `CameraController::clamp_outside`
  and its call in `editor/main.cpp` are removed; the editor camera flies freely and may enter a
  body (Unity/Blender behaviour). The planet-scale depth range is already carried by the
  infinite-far reverse-Z projection, so no radial clamp is needed. Entity/rigid-body surface
  collision is unchanged.
- **Real 1/r² gravity replaces the fixed surface-gravity stand-in.** When a scene has a
  dominant celestial body, `RuntimeSimulation` now applies that body's true Newtonian pull
  `-µ/r²` evaluated at the actual distance from its centre (via the frame-agnostic
  `Astro::body_point_gravity`), so gravity falls off with altitude and curves toward the
  planet instead of being pinned to the constant surface value `µ/a²`. Deep-space and
  non-astronomical scenes keep the local demo gravity. `Astro::gravity_field()` is
  refactored to sum the same per-body term (one source of truth).
- **Realistic sky, horizon, and Sun.** The single-scattering sky pass gains a
  cheap isotropic multiple-scattering fill so the twilight horizon stays hazy
  instead of falling to black; hemispherical skylight now lights the analytic
  ground (tinted by the atmosphere's Rayleigh coefficient) so the ground meets
  the sky seamlessly at the limb rather than as a dark ring; the view-ray
  scattering march is 32 steps with a quadratic, camera-clustered distribution so
  the dense near-camera air is resolved and the far ground veils correctly at the
  horizon. The Sun disk gets a physically based limb-darkening law (warming and
  dimming toward the edge) and a scale-independent warm aureole that reddens as
  it nears the horizon, so it no longer reads as a flat white cutout.

### Fixed
- **Viewport gizmo is now frame-aware for surface-anchored entities.** World-space handles are
  meaningless on a curved planet (world +Y is "up" only at one point), so a surface-anchored
  entity's gizmo now resolves against its local **East-North-Up** ground frame — `editor/main.cpp`
  forces Local space there, and since the entity's orientation is stored ground-local and composed
  onto the tangent frame, the handles come out as east/north/up. A gizmo **rotation** on such an
  entity also sticks now: `set_transform` re-derives the stored ground-local orientation from the
  edit, but only for a pure rotation (position unchanged), so the tangent frame is identical and the
  round-trip is exact — a translation never touches it, keeping "upright" upright at every latitude.
- **Solar eclipses are now visible.** The Sun and Moon already subtend near-equal angles from
  Earth (the catalogue radii and real ephemeris distances are exact, and the sky pass sizes every
  body by `asin(radius/distance)`), so the dark Moon disk *was* drawn over the Sun — but the Sun's
  warm aureole and disk were added regardless of occlusion and blazed straight through it. `sky.frag`
  now computes a per-frame `sun_eclipse` fraction (the circle-circle overlap of the Sun's disk by any
  nearer body, `disk_overlap_fraction`) and uses it to dim the daylight, the disk, and the aureole
  toward totality while revealing a thin corona at the limb — so a partial eclipse dusks the sky and a
  total eclipse reads as a dark disk ringed with corona. No CPU/size change; the body sizes were
  already physically exact.
- **Environment → Travel "Earth" sent you to the wrong body.** The travel handler treated Earth
  as a synonym for "home" (the scene origin) — but the scene origin is the *observer* body's
  surface, which becomes whatever planet you last travelled to. So selecting Earth while standing
  on another planet dropped you back on that planet. It now returns to the origin only for the body
  you are already anchored to (`environment.observer.observer_body`), and travels to Earth's real
  position otherwise, via the same body lookup every other destination uses.
- **Planet-scale depth clipping and z-fighting (reverse-Z, infinite far).** The scene
  view rendered with a fixed near/far of 0.1–500 m and a 24-bit unorm depth buffer, so
  flying to planet scale clipped the horizon and distant geometry at 500 m and z-fought
  badly. The projection (`perspective`) is now reverse-Z with an *infinite* far plane, the
  depth buffer is `D32_SFLOAT_S8_UINT`, and the pipeline clears depth to 0 and compares
  with `GREATER_OR_EQUAL` — spreading floating-point precision almost uniformly from a few
  centimetres to 10^7 m and clipping nothing for distance. The selection outline's depth
  bias flips sign to match, and the sky pass's "is there geometry here" tests flip to
  `depth > 0` (its view-z reconstruction reads the projection directly, so it needs no
  other change). Nothing is drawn with a far cutoff anymore.
- **Mesh jitter at planetary distances.** The scene view now renders meshes,
  the ground grid, and cloth in camera-relative space: `make_push` subtracts
  the camera eye from each model's translation in double precision before the
  `float` cast, cloth points are offset by the eye as they are written to their
  buffer, and the uploaded view matrix carries no translation. Previously both
  the model and view matrices baked absolute world translations that lost all
  sub-metre precision the moment they were cast to `float` for the GPU (a ~16 m
  quantisation at 1.5e8 m from the origin), so geometry far from the origin —
  or seen from a far-off camera — visibly jittered even in the double-precision
  build. `pbr.frag` matches the new frame by taking the view direction as
  `-v_world_position` (the camera is at the origin of camera-relative space).
  The sky pass was already camera-relative and is unchanged.

### Changed
- **`Scalar` is now always double precision; the `SE_SCALAR_DOUBLE` build option
  is removed.** The engine simulates planet- and solar-scale worlds, where single
  precision quantises camera and transform math to roughly a metre at 10 000 km
  from the origin (float32's ~7 significant digits) — the cause of the camera
  "wandering" and rotation locking that motivated this change. Double is now the
  engine's one and only boundary/render `Scalar`; there is no compile-time toggle.
  The placeholder's `Float` is a plain `using Float = double`, the option and its
  `target_compile_definitions` plumbing are gone from `cmake/ProjectOptions.cmake`,
  the top-level and `render/` `CMakeLists.txt`, and the `se` CLI (`--double`, the
  `scalar_double` config field, and `current_precision()` are removed). The
  editor's Preferences precision setting now selects only the physics-solve
  precision (`Simulation::Precision`, a live runtime choice) and defaults to
  double. GPU data is unaffected: the upload path already narrows to 32-bit
  camera-relative at the push-constant boundary.
- **Editor camera controls honour the `Scalar` precision seam.** `FlyCamera`,
  `CameraController`, and `FlyCameraSource::set_move_speed` now store and
  compute in `Scalar` (always double) instead of hard-coded `float`.
  `InputState` fields remain `float`
  (they come from ImGui pixel deltas) and are `static_cast` to `Scalar` at the
  computation boundary, so the full camera pipeline — position, orientation,
  projection, and all controller parameters — runs at the same precision as the
  rest of the engine. No API change at the `ISceneCamera` interface; callers
  that pass `float` (e.g. `Preferences::camera_move_speed`) still compile
  through implicit widening.

### Added
- **PBR materials, a physical sky, and a WGS84 planet.** The scene view grew from a
  single hard-coded directional light into a full lit environment rendered in three
  HDR passes. A new neutral seam `render/environment.hpp` carries the sun
  (`DirectionalLight`), the `Wgs84` ellipsoid (a = 6378137 m, 1/f = 298.257223563),
  and the `AtmosphereParams`/`PlanetParams`/`CloudParams`/`StarParams`/`Material`
  that describe how the planet is lit and surrounded — authored by the simulation
  (`RenderScene::environment`, `RenderInstance::material`) and consumed by
  `ISceneView::render`, which now takes a `const Environment&` and the camera's world
  position.
  - **Materials.** `MeshInstance` carries a metallic-roughness `Material`; the new
    `pbr.frag` shades every mesh with a Cook-Torrance BRDF (GGX + Smith + Schlick)
    lit by the sun and an ambient floor, replacing the old flat `mesh.frag`.
  - **Sky.** `sky.frag` ray-marches, in camera-relative space, the WGS84 ellipsoid
    ground, Rayleigh + Mie single-scattering atmosphere, a procedural cloud layer,
    and a hashed star field, composited over the opaque scene by the sampled depth
    (aerial perspective included). As the camera climbs the atmosphere thins to black
    space and the stars emerge — the near-surface-to-orbit transition, driven by
    optical depth rather than a hard switch.
  - **HDR pipeline.** The offscreen target is now linear `R16G16B16A16_SFLOAT`; a
    `tonemap.frag` (exposure + ACES + gamma) resolves it into the `R8G8B8A8_UNORM`
    image the editor samples. The scene view gained a per-frame uniform block (its
    first descriptor set) shared by all passes, plus the sky and tonemap pipelines.
  - **Authoring.** The editor's new **Environment** panel authors the sun
    (azimuth/elevation, colour, intensity), atmosphere on/off, exposure, clouds, and
    stars; the Inspector gained a metallic/roughness/emissive material section.
    `IWorldEditor` grew `environment()`/`set_environment()` and
    `material()`/`set_material()`.
- **Cloth renders as a shaded, pickable mesh.** `render/cloth_mesh.hpp`'s new
  `triangulate_cloth_grid` turns a cloth grid's row-major particle positions into a
  triangle list with averaged per-vertex normals; the Vulkan scene view now uploads
  that mesh into a host-visible index/vertex buffer pair and draws it through the
  same lit `mesh_pipeline_` Box/Sphere/Cylinder use, instead of a flat-coloured,
  unpickable grid-edge wireframe. `Render::ClothStrandView` and `Simulation::
  ClothInstance` gained `color`/`id` fields (defaulted to the wireframe's old fixed
  tint, since cloth entities carry no `Tint` component), so a cloth sheet now
  shades, picks, and outline-highlights like any other object.
- **Two-way cloth↔rigid coupling, true oriented-box (OBB) contacts, and a broadphase.**
  The rigid and cloth worlds are now driven in lockstep (predict → solve both, resolve the
  contacts that span them, then derive velocity for both), so a cloth sheet pushes back on
  the rigid bodies it lands on rather than only draping over them. `collision.hpp` gained an
  `OrientedBox` and SAT-based `collide_obb_obb`/`collide_obb_sphere`/`collide_obb_plane`, so
  boxes collide as the boxes they look like (a box now rests flat on the ground at its
  half-extent instead of hovering by a bounding sphere). A new `broadphase.hpp`
  (`sweep_and_prune`) culls the O(n²) pair set to overlapping AABBs before the narrowphase,
  and the unified pass (`contact_solver.hpp`'s `ContactBody`) resolves every candidate pair
  two-way by inverse mass plus every body against the static planes.
- **Play-mode transform edits move the physics.** Dragging a rigid body's transform while
  the world is playing now teleports its physics body (velocity cleared) instead of being
  overwritten by the next solved pose — `IPhysicsSimulation::set_rigid_pose`, called from
  `set_transform` when the entity has a body.
- **Live collision: rigid bodies rest and stack, cloth drapes over them.** Contacts are
  now wired into the simulation's tick. `PhysicsWorld::step` gained a post-solve callback
  (run each sub-step between the constraint solve and the velocity derivation, keeping the
  world collider-agnostic); `PhysicsSimulation<T>` uses it to resolve rigid↔rigid and
  rigid↔plane contacts, and — snapshotting the rigid bodies as sphere obstacles — cloth↔
  rigid and cloth↔plane contacts, so a body landing on terrain loses its velocity and a
  cloth sheet settles over a sphere. `RigidBodyDesc` carries a collision `radius` (from
  the entity's Collider/Shape), `ClothDesc` a per-particle `thickness`, and the new
  `IPhysicsSimulation::set_static_planes` supplies the scene's `Plane` colliders (e.g.
  Terrain) each tick. `contact_solver.hpp` added multi-plane and static-sphere-obstacle
  passes.
- **In-viewport UI manipulation (RectTransform tool) + a non-blocking canvas.** The Scene
  view's UI overlay is now interactive: click an element to select it, drag its body to
  move it, and drag a corner handle to resize it — each writing the change straight back
  to the element's `UIElementParams` (`ui_apply_screen_rect` inverts the layout formula),
  as one undo step per drag. In the Scene view the overlay is drawn **translucent** with
  outlines so a full-screen canvas no longer hides the 3D scene, and a canvas is never
  picked by its body (clicks fall through to the scene or a child element); the Game view
  still draws UI solid and non-interactive. (`ViewportPanel::draw` now takes a single
  `UIOverlay*` — elements + edit-mode + interaction I/O — instead of an element
  pointer/count.)
- **Editor: Cloth object, UI authoring, custom components, and add/remove for every
  component.** The `ISimulation`/`IWorldEditor` seam and the ImGui editor now expose the
  full component set as attachable, detachable pieces:
  - **Cloth** is a first-class object (`create_cloth`, Entity ▸ Objects ▸ Cloth). A new
    Cloth renders immediately in edit mode as a flat resting sheet (synthesised in
    `extract()` to match `build_cloth_grid`'s layout) and drapes once the world is played.
  - **UI (Canvas + elements).** New seam types `UIElementKind` (Canvas/Panel/Image/Text/
    Button) and `UIElementParams` (a uGUI RectTransform: anchors, pivot, offset, size,
    colour, opacity, text). `create_canvas`/`create_ui_element` add them from Entity ▸ UI;
    the Inspector edits every field; the viewports paint them as a 2D ImGui overlay
    (`UIOverlayElement` + `paint_ui_overlay`) laid out against the panel rect, with button
    hover/press tinting — a working canvas/button surface without a Vulkan 2D pass yet.
  - **Custom (script) components.** `ScriptComponent`/`ScriptField`/`ScriptFieldKind` are a
    data-driven, MonoBehaviour-style component: attach one from Add Component ▸ Scripts,
    edit its typed fields (float/int/bool/vec3/colour/text) in the Inspector, and save it
    with the scene. "New Script…" scaffolds a `<Name>.hpp` C++ system stub in the project,
    opens it in the Text Editor, registers the type in the Add Component catalog, and
    attaches it.
  - **Shape is now a feature of the Renderer.** The mesh (kind + dimensions) is edited
    inside the Renderer header and its lifetime is bound to the Renderer (adding a Renderer
    gives it a Box mesh; removing it takes the mesh with it); the mesh kind is now editable
    (Box/Sphere/Cylinder). `create()` makes a truly empty entity (no renderer), matching
    Unity's empty GameObject.
  - Every optional component (Renderer+Mesh, Camera, Rigid Body, Cloth, Collider, UI
    Element, Scripts) is add/removable via Add Component and each header's close box, and
    all of them round-trip through the scene serializer and the copy/cut/paste clipboard.
- **Collision narrowphase and positional contacts.** `physics/collision.hpp` adds
  element-parametric collider shapes (`SphereCollider<T>`, `PlaneCollider<T>`,
  `BoxCollider<T>`) and pure `Contact`-generating narrowphase tests (sphere-plane,
  sphere-sphere, box-sphere, box-plane), and `physics/contact_solver.hpp` resolves
  penetration positionally (`resolve_plane_contacts`/`resolve_pair_contacts`/
  `resolve_contacts`) between `predict` and `update_velocity`, so a body landing on a
  surface loses its velocity as an inelastic contact — colliders now actually collide.
  Covered by `Unit_Collision` (including a dropped particle coming to rest exactly on
  the ground).
- **Volumetric soft bodies.** `physics/soft_body.hpp`'s `build_soft_body_lattice`
  wires an `nx*ny*nz` grid of particles held by structural (axis) and shear
  (face-diagonal) XPBD distance constraints into a `PhysicsWorld` — the 3D
  generalization of the cloth grid, reusing the graph-coloured solver with no new
  constraint type, so a deformable block settles and recovers under gravity. Covered
  by `Integration_SoftBody`; `examples/soft_body_demo.cpp` (`soft_body_demo` target)
  drives both the lattice and a ground contact headlessly.

- **ECS UI: a retained canvas/button system (Unity UGUI-shaped).** A new `SushiEngine::UI`
  module (`include/SushiEngine/ui/`) where UI is built from ordinary entities: a
  `Canvas` root and a tree of elements carrying a `RectTransform` (anchor/pivot/offset
  layout), `UIImage`, `UIText`, and/or `UIButton`, linked by `UIParent`. `resolve_rect`
  (`ui/layout.hpp`) is the pure UGUI anchor formula; the `UI` façade (`ui/ui.hpp`)
  offers ergonomic `canvas`/`panel`/`image`/`label`/`button` builders over an existing
  `World`, resolves every `ComputedRect` each frame, runs the button state machine and
  press-and-release-inside click detection off a per-frame `PointerInput`, tints button
  graphics per state, fires `on_click` callbacks, and yields a renderer-agnostic
  `UIDrawList` of coloured rects and text runs. `examples/ui_demo.cpp` (`ui_demo`
  target) and `Unit_UILayout` + `Integration_UI` tests cover layout, click, and button
  states headlessly. The Vulkan overlay pass that rasterises the draw list and the
  editor's UI-authoring panels are a following visual increment; the model, layout, and
  interaction are complete and tested here.

- **Runtime-selectable physics precision (float or double), live-switchable from the
  editor.** `Simulation::Precision` and `create_simulation(Precision)` choose the
  scalar the XPBD solve runs in; both a `float` and a `double` variant are compiled
  into the simulation library, so the choice is made at runtime rather than by a build
  flag. The physics is now reached through a new `Simulation::IPhysicsSimulation` seam
  (`sim/physics_simulation.hpp`) with a templated `PhysicsSimulation<T>` implementation
  that converts between the boundary `Scalar` and its solve precision `T`; extracting
  this also removed the two `PhysicsWorld`s and their bookkeeping from
  `RuntimeSimulation`, leaving it to marshal entity poses to and from descriptors
  (single responsibility). The editor's Preferences "Physics precision" control now
  applies live: changing it rebuilds the running simulation in the new precision via
  the scene serializer, preserving the scene — no binary rebuild. `ISimulation` gained
  `precision()`.

### Changed
- **The XPBD physics layer is now element-parametric.** `RigidBodyT<T>`,
  `XpbdDistanceConstraintT<T>`, `XpbdDistanceProjectionT<T>`, and the `predict`/
  `update_velocity`/`apply_angular_correction` helpers template on the scalar element;
  `XpbdSolver<Constraint>`, `PhysicsWorld<Constraint>`, and `build_cloth_grid` derive
  their precision from the constraint's `Real` typedef. Boundary aliases `RigidBody`,
  `XpbdDistanceConstraint`, and `XpbdDistanceProjection` stay fixed to `Scalar`, so
  every existing solver, world, bridge, demo, and test compiles and behaves
  identically; a simulation can now instantiate `PhysicsWorld<XpbdDistanceConstraintT<double>>`
  beside a float renderer. Builds on the parametric math types below.
- **Placeholder math is now element-parametric (`Vector3T<T>`, `QuaternionT<T>`).**
  `core/blas_placeholder.hpp` templates the vector and quaternion types (and their
  operations) on the scalar element, exposed at engine scope as `Vector3T`/
  `QuaternionT` in `core/types.hpp`. The boundary aliases `Vector3` and `Quaternion`
  are unchanged — they fix the element to `Scalar` — so every existing use compiles
  and behaves identically; the new templates let the simulation core compute in a
  runtime-selected precision (float or double in one build) beside a fixed-precision
  render boundary. `Mat4` and `WorldVector3` stay single-fixed-precision (render-side
  and always-double respectively). Foundation for runtime float/double selection.

### Added
- **SushiLoop authoring API (`Loop::App<Command>`).** The settled surface a game is
  written against: it composes the existing `World`, `Schedule`, `FixedTimestepClock`,
  `CommandBuffer`, `InputHistory`, `RollbackBuffer`, and seeded `RngState` into one
  fixed-step deterministic loop with an ergonomic, pure-ECS system-registration form
  — `app.system<Read<A>, Write<B>>("name").each(fn)` — plus `on_start`/`on_command`/
  `sample_command` lifecycle callbacks. It is **always multiplayer-ready**: every
  tick's command is numbered into an `InputHistory` regardless of network state, and
  the network is reached only through the new `Loop::Net::INetworkTransport<Command>`
  abstraction, so making a game networked is one decision — `connect()` a transport —
  with no change to its systems (dependency inversion). Header `loop/app.hpp`, pulled
  into the umbrella `SushiEngine.hpp`.
- **`Loop::Net::INetworkTransport<Command>` and `Loop::Net::LoopbackTransport<Command>`.**
  The transport seam `Loop::App` reconciles against, and an in-process implementation
  backed by the existing `LoopbackChannel` (with optional per-ack latency), so
  single-player, loopback, and a future socket transport are interchangeable behind
  one interface.
- **`examples/first_game.cpp` (`first_game` target).** The first complete SushiLoop
  game: two pure-ECS systems (input→velocity, velocity→position) run in a fixed-step
  loop, checked against a closed-form reference for determinism, then re-run as a
  mispredicting client that reconciles to an authoritative server — proving the
  "one `connect()` makes it multiplayer" claim end to end. Headless, `RESULT: OK` on
  success, `compile_count == 1`.

### Fixed
- **Viewport gizmo incorrectly applied local transforms as world transforms on child objects.**
  The Scene viewport's transform gizmo now correctly uses `world_transform` and writes back through the new `set_world_transform` API on `IWorldEditor`. This prevents objects from jumping to incorrect positions when manipulated after being reparented, keeping the gizmo correctly anchored in world space.

### Added
- **Hierarchy drag-and-drop reordering.** Entities can now be reordered among their siblings by dragging and dropping them within the Hierarchy panel. A drop indicator shows where the entity will be inserted (before or after the target).
- **Hierarchy empty-space deselection.** Clicking on empty space within the Hierarchy panel now clears the active selection.

### Added
- **Entity Copy/Cut/Paste**, with Ctrl+C/Ctrl+X/Ctrl+V shortcuts, in the Edit menu
  and every Hierarchy context menu. Implemented entirely through existing
  `IWorldEditor` getters/setters (no new engine primitive): Copy snapshots the
  selection's full authored state (transform, colour, visibility, Renderer/Camera/
  Rigid Body/Cloth/Shape/Collider) into `EditorContext::clipboard`; Paste replays it
  onto freshly created entities, preserving internal parent/child links within the
  pasted set and re-parenting to the original external parent otherwise; Cut is
  Copy followed by deleting the originals.
- **Primitive object creation: Box, Sphere, Cylinder, and Terrain.** The Hierarchy's
  right-click menu, the GameObject menu, and the empty-space Hierarchy context menu
  all gain "Create Entity" plus an "Objects" submenu (`IWorldEditor::create_box`/
  `create_sphere`/`create_cylinder`/`create_terrain`). Each spawns a Renderer entity
  with a new `ShapeParams` (Box/Sphere/Cylinder mesh + params) and a matching
  `ColliderParams`; Terrain is a large, thin flat Box visual with a `Plane` Collider
  and no physics body, so nothing integrates its pose (gravity-exempt by
  construction). The Vulkan renderer gained unit sphere and cylinder meshes
  alongside the existing cube, selected per-instance by `Render::MeshKind`, and
  scaled to each instance's authored shape params.
- **`Collider` component: pure collision-volume authoring data.** Independent of
  any visual `Shape`, addable/removable through the Inspector's "Add Component"
  popup (`IWorldEditor::has_collider`/`collider_params`/`set_collider_params`/
  `set_has_collider`). Not yet consumed by any narrowphase or contact solver — a
  data-only foundation for a future rigidbody/softbody collision milestone.
- **Cloth grids now render as a wireframe.** `RenderScene` gained
  `cloth_instances`/`cloth_vertices`, extracted every frame from each Cloth
  entity's live simulated particle positions. `ISceneView::render` gained an
  optional `ClothStrandView` list, drawn through the scene view's existing line
  pipeline (previously only used for the ground grid) as horizontal/vertical grid
  edges.

### Changed
- **The "GameObject" menu is renamed "Entity"**, and "Create Entity" is renamed
  "Create Empty Entity" everywhere it appears (menu bar, Hierarchy right-click, and
  the Hierarchy's empty-space context menu). Camera creation moved into the shared
  `draw_create_object_menu_items` helper so the Entity menu and every Hierarchy
  context menu offer identical creation options — previously Camera existed only in
  the top menu, out of sync with the Hierarchy's right-click menu.
- **A plain "Create Entity" is now a bare `Transform`, not a disguised cube.**
  `RuntimeSimulation::create()` no longer attaches a default Renderer/Tint; the
  renderer now draws an entity only when it has both a Renderer and a `Shape`
  (previously, having a Renderer alone drew the renderer's hardcoded cube mesh).
  Existing "Renderer" toggling behavior is unchanged for entities that do have a
  `Shape`.
- **Naming convention overhaul: `PascalCase` namespaces, no abbreviations, and a
  reorganized editor tree.** All namespaces are now `PascalCase`
  (`SushiEngine::Editor`, `SushiEngine::Simulation`, `SushiEngine::Loop`,
  `SushiEngine::Loop::Net`, `SushiEngine::Render`, `SushiEngine::Render::Vulkan`,
  `SushiEngine::Render::Shaders`, `SushiEngine::Detail`), replacing the previous
  mixed-case forms (`sushi::editor`, `sim`, `loop`, `net`, `render`, `vulkan`,
  `shaders`, `detail`). `core/types.hpp` renames `Vec3`→`Vector3`,
  `WorldVec3`→`WorldVector3`, `FloatingOriginVec3`→`FloatingOriginVector3`, and
  `Quat`→`Quaternion`, with the corresponding `*_quat_*` helper functions renamed
  to `*_quaternion_*`. `editor/` is split from a flat directory into
  `core/`, `camera/`, `gizmo/`, `input/`, `serialization/`, `window/`, and `ui/`
  subdirectories by responsibility. See `CONTRIBUTING.md` §4 for the full naming
  rule, including the well-known-acronym exception (`API`, `PGS`, `RHI`, `GPU`,
  `ECS`, `XPBD`).

### Changed
- **`RuntimeSimulation` wires `loop::FixedTimestepClock` into its tick loop instead
  of assuming a fixed ~1/60s real frame.** `ISimulation::tick()` now takes a
  `real_delta_seconds` parameter — the editor's main loop measures real elapsed
  frame time and passes it in, rather than the simulation assuming it. Internally,
  `RuntimeSimulation` accumulates the delta into an owned `FixedTimestepClock` and
  runs one full step (physics + ECS schedule + render extract) per whole fixed step
  the clock reports (zero, one, or more, e.g. on a hitch). The physics sub-step
  duration now derives from the clock's fixed step instead of a separately
  hardcoded constant. The clock's leftover interpolation fraction is computed and
  stored for a future render-interpolation consumer, not yet wired to one.

### Added
- **`loop::net::reconcile`/`LoopbackChannel` wired into a live client/server demo
  and test with a real gameplay `Command`.** `examples/net_demo.cpp` and
  `tests/functional/integration/test_net_client_server.cpp` replace M4's
  synthetic-only proof (a toy `Scalar` command) with `PlayerCommand`, a two-axis
  movement input mapped onto a player entity's `Position` — the smallest command
  shape with an obvious mapping to something real. "Client" and "server" are
  modelled as two logical roles in one process, each owning its own `ecs::World`,
  matching M4's loopback-only scope (`docs/slop/SUSHILOOP.md`). The harness
  proves the whole chain live: per-tick prediction into `InputHistory`, batched
  `LoopbackChannel::server_process` acks, `net::reconcile` rolling back and
  replaying on misprediction, convergence to an uninterrupted authoritative-only
  baseline, and `net::make_network_id` agreeing across an independent
  client-side and server-side spawn of the same logical entity with no matching
  round trip. The network-id spawn is deliberately kept outside the ticks that
  get captured/rolled back, since `RollbackBuffer` still cannot survive a
  structural change (spawn/destroy) inside a rolled-back range — that remains
  later work, not solved here (see ARCHITECTURE.md §8.1). The prior toy-command
  `test_net_reconciliation.cpp`/`test_network_id.cpp` are kept as narrower unit-
  level proofs of the mechanics in isolation. Real transport (sockets) and
  editor Play-mode wiring remain out of scope, per M4's design.
- **The editor's "Cloth" toggle wires `Physics::build_cloth_grid` into
  `RuntimeSimulation`'s live tick loop.** A new `IWorldEditor::set_has_cloth`/
  `has_cloth`/`cloth_params`/`set_cloth_params` surface (mirroring the existing
  "Rigid Body" toggle) attaches a simulated cloth grid to an entity — rows,
  columns, spacing, and XPBD compliance are authorable, the grid originates at
  the entity's `Transform::position`, and it steps every fixed tick in its own
  `PhysicsWorld<XpbdDistanceConstraint>` (kept separate from the Rigid Body
  world so cloth's full-rebuild-on-param-change discipline never disturbs
  `rebuild_physics()`'s live-state carry-over for free bodies). The grid stays a
  single host-side record, not one ECS entity per particle; its world-space
  particle positions are readable via the new `IWorldEditor::cloth_particle_positions`
  for a future debug draw or renderer, since `RenderScene::instances` cannot yet
  express a multi-vertex deforming mesh — cloth is simulated and inspectable but
  not yet drawn in the viewport (see ARCHITECTURE.md §4.2). The Inspector's
  "Cloth" section (Rows/Columns/Spacing/Compliance) and `.sushiscene`
  (`has_cloth`/`cloth`) mirror the Rigid Body toggle's UI and serialization.
- **Cloth grids over the XPBD solver (SushiLoop M5).** `Physics::build_cloth_grid`
  (`physics/cloth.hpp`) wires a pinned-top grid of point-mass `RigidBody`s into a
  `PhysicsWorld<XpbdDistanceConstraint>` with structural (horizontal/vertical
  neighbour) and shear (diagonal neighbour) constraints — no new solver or
  constraint type, reusing the same `XpbdDistanceConstraint`/`XpbdDistanceProjection`
  `xpbd_demo.cpp`'s hanging chain already uses. New `examples/cloth_demo.cpp` mirrors
  `xpbd_demo.cpp`'s structure (device solve checked against a byte-for-byte host
  mirror of the projection); `Integration_Cloth`
  (`tests/functional/integration/test_cloth.cpp`) proves the grid's topology (body
  counts, which row is pinned) and that the pinned row never moves while the rest of
  the grid settles under gravity. Volumetric (tetrahedral) soft bodies are
  explicitly out of scope. See `docs/slop/SUSHILOOP.md`.
- **Floating-origin round-trip stress coverage at planet scale and beyond
  (`tests/functional/unit/test_floating_origin_stress.cpp`).** Extends
  `Unit_FloatingOrigin` with round-trip and decomposition checks at Earth-radius
  (~6.378e6 m) and interplanetary/extreme magnitudes (~1e9, ~1e12), proving
  ARCHITECTURE.md §6's claim that the local offset stays small and
  `Scalar`-precision-accurate on round-trip no matter how far the absolute
  coordinate is from the world origin.
- **`loop::net`, SushiLoop's Net layer (M4): loopback-only, server-authoritative
  reconciliation (`loop/net.hpp`).** `LoopbackChannel<Command>` simulates an
  in-process, synchronous client/server command exchange reusing
  `InputHistory<Command>`/`TickId` — the client predicts and sends a command per
  tick, the server (a caller-supplied corrector function) returns an authoritative
  ack per tick. `net::reconcile` sits on top of the existing `loop::RollbackBuffer`
  unchanged: whenever an ack disagrees with what the client predicted, it corrects
  the client's `InputHistory` (new `InputHistory::correct`, an overwrite instead of
  `record`'s append-only insert), rolls the world back to the earliest such tick via
  `RollbackBuffer::restore`, and replays every tick forward through the current one.
  `net::make_network_id(client_id, tick, spawn_sequence)` derives a deterministic
  `NetworkId` so an entity spawned during simulation gets the same id on server and
  client without a separate matching step. Explicitly out of scope: real
  sockets/threads, serialization, encryption, compression, and general P2P/lockstep
  protocols — this is host-side, loopback-only, per the milestone's stated scope.
  `Integration_NetReconciliation`
  (`tests/functional/integration/test_net_reconciliation.cpp`) proves the milestone's
  key invariant: a client that mispredicts several ticks and later reconciles
  against the server's authoritative commands converges to exactly what an
  uninterrupted server-only simulation would have produced.
  `Unit_NetworkId` (`tests/functional/unit/test_network_id.cpp`) proves the id is
  reproducible from the same inputs and does not collide across client, tick, or
  spawn-sequence. See `docs/slop/SUSHILOOP.md`.
- **Editor Play/Stop now restores the scene, like Unity's edit/play-mode split.**
  Pressing Play captures the whole scene (`editor::capture_scene`) into
  `EditorContext::play_mode_snapshot`; pressing Stop re-applies it
  (`editor::apply_scene`) and discards the snapshot, so any spawns, destroys,
  transform edits, or component toggles made while playing vanish on Stop instead
  of persisting into the edited scene. Fixed two `capture_scene`/`apply_scene`
  round-trip gaps this depended on: `has_renderer` is now captured as its own
  field (previously a Renderer-less entity came back with one on restore), and a
  camera entity's Renderer/colour is now captured independently of `is_camera`
  (previously mutually exclusive, so a camera with a Renderer attached lost it).
- **`loop::RollbackBuffer`, SushiLoop's Snapshot layer (M3).** A fixed-capacity ring
  of per-tick, per-chunk world snapshots (`loop/rollback.hpp`): `capture(world,
  tick)` copies every live chunk's component column bytes; `restore(tick)` writes
  them back, evicting the oldest tick once the ring is full. Scoped to no
  structural change (no spawn/destroy, no new archetype/chunk) between a capture
  and its matching restore — real per-write dirty tracking and rebasing across
  structural change are later work. `Chunk` gained the column-walking accessors
  (`column_count`/`column_id`/`column_size`/`column_at`) and a rollback-only
  `restore_count` this needed. `Integration_Rollback`
  (`tests/functional/integration/test_rollback.cpp`) proves the milestone's key
  invariant: a world rolled back mid-run and replayed forward with the same
  recorded input stream ends up bit-identical to an uninterrupted run, and that a
  restore of an evicted tick correctly fails. See `docs/slop/SUSHILOOP.md`.
- **Rigid Body: the editor's first-class physics toggle.** Any entity can now
  attach/detach physics the same way it attaches/detaches a Renderer or Camera —
  `IWorldEditor::has_physics_body`/`set_has_physics_body`/`physics_body_params`/
  `set_physics_body_params` (`sim/simulation.hpp`'s new `PhysicsBodyParams`: inverse
  mass and diagonal inverse inertia), an Inspector "Rigid Body" panel (Add
  Component menu, mass/inertia drag fields, remove via the header's "x"), and
  `.sushiscene` save/load support (`has_physics_body`/`physics_body` fields).
  Unlike Renderer/Camera this needs no ECS component migration — `Transform`/
  `Orientation` stay the components, physics only starts/stops tracking them —
  so `RuntimeSimulation` (`sim/runtime_simulation.cpp`) instead owns a
  `Physics::PhysicsWorld<XpbdDistanceConstraint>` rebuilt lazily (on the next
  `tick()`, not eagerly on toggle) whenever the physics-driven entity count
  changes, carrying over every still-live body's simulated state (position,
  orientation, velocity) across the rebuild so unrelated toggles never reset
  already-falling bodies. `tick()` steps the physics world under gravity and
  writes the solved pose back into `Transform`/`Orientation` before running the
  ECS schedule. No joints/constraints yet — this is free-body physics only; see
  `docs/slop/SUSHILOOP.md`.
- **`Physics::PhysicsWorld`, the sub-stepped XPBD loop (`physics/physics_world.hpp`).**
  Wraps `XpbdSolver` with the register-then-run lifecycle a physics scene actually
  needs: `add_body()`/`add_constraint()`, one `finalize()` that uploads the bodies
  and compiles the solve graph, then `step(linear_acceleration, substeps)` runs
  predict / solve / derive-velocity per sub-step. Deliberately has no ECS
  dependency — `physics/` stays below `ecs/` in the engine's layering, so this is
  usable standalone and is the seam a later ECS-facing sync layer builds on, not
  something that layer gets folded into. `Integration_PhysicsWorld`
  (`tests/functional/integration/test_physics_world.cpp`) proves a pinned/weight
  pair settles at rest length under the loop's own gravity integration.
- **The ECS-facing physics bridge (`sim/physics_bridge.hpp`).** `sim::PhysicsBody`
  names which `PhysicsWorld` body a physics-driven entity owns (`INVALID` until
  registered); `sim::initial_rigid_body()` builds a body's starting XPBD state from
  an entity's current `Transform`/`Orientation` at registration time; and
  `sim::sync_transforms_from_physics()` writes every registered entity's solved
  pose back each tick. Deliberately one-directional (physics -> ECS) and lives in
  `sim/`, not `physics/`, keeping `PhysicsWorld` itself free of any ECS dependency.
  `Integration_PhysicsBridge`
  (`tests/functional/integration/test_physics_bridge.cpp`) proves a registered
  entity's `Transform` tracks its falling body while an unregistered entity is left
  untouched.
- **Unified rigid-body XPBD solver (SushiLoop M2).** New `Physics::RigidBody`
  (`physics/rigid_body.hpp`: position, orientation, generalized inverse mass —
  inverse mass plus a diagonal body-local inverse inertia tensor — with
  `predict()`/`update_velocity()` implementing XPBD's predict/solve/derive-velocity
  step) and `Physics::XpbdDistanceConstraint`/`XpbdDistanceProjection`
  (`physics/xpbd_constraint.hpp`, `physics/xpbd_solver.hpp`): the rigid-body
  generalization of the existing PGS `DistanceConstraint`, with two attachment
  points (in each body's local frame) and an XPBD `compliance` term (`0` = fully
  rigid, matching PGS). `Physics::XpbdSolver<Constraint>` reuses the existing
  `color_constraints`/`ColorBatches` graph-colouring unchanged, over a single
  `RigidBody` buffer, compiled once and replayed every step exactly like
  `ConstraintSolver`. Also adds `rotate()`/`conjugate()` to the value-type seam
  (`core/types.hpp`) for quaternion-vector rotation. New `examples/xpbd_demo.cpp`
  ports `pgs_demo.cpp`'s hanging chain onto the new solver, and
  `Integration_XpbdSolver` (`tests/functional/integration/test_xpbd_solver.cpp`)
  validates it against a scalar reference and the PGS chain's shape in the
  zero-inertia case. See `docs/slop/SUSHILOOP.md`.
- **SushiLoop core skeleton (M1).** New `SushiEngine::loop` namespace
  (`include/SushiEngine/loop/`), pulled into the `SushiEngine.hpp` umbrella header:
  `FixedTimestepClock` (accumulator-based fixed-step time, no wall-clock reads —
  the host feeds it real elapsed time and it says how many fixed ticks are due,
  plus the leftover interpolation fraction for rendering); `RngState`/`seed_rng`/
  `next_u64`/`next_unit` (a trivially copyable xorshift128+ generator so seeded
  randomness can live in a component and travel with rollback snapshots); and
  `InputHistory<Command>` (a numbered, per-tick command buffer, the shape
  networked input capture and replay will use). A new `SE_DETERMINISTIC_FP` CMake
  option (default `ON`) adds `-fno-fast-math -ffp-contract=off` to the
  `SushiEngine` INTERFACE target on clang/gcc, closing off the two easiest ways a
  build could make floating-point results non-reproducible. A new
  `Integration_DeterministicReplay` test proves the end-to-end claim: replaying
  the same numbered input stream against two fresh worlds with the same seeded
  `RngState` produces bit-identical entity state. See `docs/slop/SUSHILOOP.md`.
- **Floating-origin world types (SushiLoop M0).** `core/types.hpp` gains `WorldVec3`
  (an always-double 3-vector for absolute ECEF positions, independent of the
  `SE_SCALAR_DOUBLE`-selected `Scalar`), `SectorCoord` (an integer floating-origin
  sector index), and `FloatingOriginVec3` (a sector index plus a `Scalar`-precision
  local offset). `to_floating_origin`/`from_floating_origin` convert between the two
  representations. This is the type-seam foundation for SushiLoop's planet-scale,
  floating-origin world space (`docs/slop/SUSHILOOP.md`); no simulation code consumes
  it yet.
- **The editor starts scene-less.** `create_simulation()` no longer seeds a hardcoded
  demo world of spinning/orbiting cubes and a "Main Camera"; the live world starts
  completely empty, matching a fresh project. Use `File ▸ New Scene`, `New Entity`, the
  GameObject menu, or open a `.sushiscene` from the Project panel to populate it. This
  also removes the one case where a component could not be added/removed on an
  existing entity: the seeded demo cubes were flagged `animated` and exempt from
  `migrate_components`, which is what made `set_has_renderer`/`set_is_camera` silently
  no-op on them; entities created or loaded through the normal paths were never
  affected.
- **Unsaved-scene tracking and Ctrl+S.** `CommandHistory` now exposes a `revision()`
  counter bumped by every recording/undo/redo operation; `EditorContext` compares it
  against the revision stashed at the last successful New/Open/Save
  (`saved_scene_revision`, via the new `scene_is_dirty()`) to know whether the scene
  has unsaved changes. The status bar shows the open scene's name with a trailing `*`
  while dirty. Ctrl+S now saves the scene directly (previously only `File ▸ Save Scene`
  did, and only as a menu label with no keybinding); `Save Scene`/Ctrl+S/the new
  close-confirm prompt all go through one `save_current_scene()` so they agree on when
  the scene becomes clean.
- **Confirm-before-close on unsaved changes.** Closing the window (the title bar's X or
  `File ▸ Exit`) no longer exits immediately: both now set
  `EditorContext::close_requested`, and the main loop's new `draw_exit_confirm_modal`
  closes right away if the scene is clean, or offers Save / Don't Save / Cancel if it
  is dirty. Save reuses `save_current_scene()`, deferring to the existing Save-As
  prompt (via `EditorContext::exit_after_save`) when the scene has never been saved.

### Fixed
- **Game view rendered even with no active camera.** With every camera entity either
  deleted or deactivated, the Game view previously fell back to a synthetic default
  camera and kept drawing whatever renderers existed. `RenderScene` now carries a
  `has_camera` flag; the Game view draws zero instances (clears to black) when it is
  false, since there is nothing to play the scene through.

### Added
- **Unity-style modular components (Renderer, Camera).** Every entity keeps a
  mandatory Transform + Orientation; the Renderer (`Tint`) and Camera components are
  now independently attachable/detachable per entity via `IWorldEditor::set_has_renderer`
  and the new `IWorldEditor::set_is_camera` (in-place camera toggle, distinct from
  `create_camera`, which still makes a fresh camera entity), with a matching
  `has_renderer` query. The Inspector's Camera and Renderer headers each carry a
  remove ("x") control, and an "Add Component" menu offers whichever is missing. The
  ECS has no in-place component add/remove, so a toggle migrates the entity into the
  matching archetype (destroy + respawn, carrying Transform/Orientation and any
  surviving Tint/Camera value); seeded, animated demo cubes are exempt. All of
  SushiEngine's built-in ECS components (`Transform`, `Orientation`, `SpinStep`,
  `OrbitState`, `Tint`, `Camera`) are now declared in one place,
  `include/SushiEngine/sim/components.hpp`, instead of inline in
  `sim/runtime_simulation.cpp`.
- **Toolbar Step now advances the world.** Previously logged a message without
  ticking; a one-shot `EditorContext::step_requested` flag now drives exactly one
  `ISimulation::tick()` on the next frame regardless of play state, then clears.
- **Reparenting preserves world transform.** `IWorldEditor::set_parent` now recomputes
  the child's local transform so its resolved world-space pose is unchanged across a
  reparent, instead of reinterpreting the existing local transform in the new
  parent's space (which used to visibly jump the object). World-transform composition
  changed from a general `Mat4` chain-multiply to a shear-free hierarchical TRS chain
  (`world_scale = parent_scale * local_scale`, etc., matching Unity's model), which is
  what makes solving for the compensating local transform tractable.
- **Game view no longer highlights the Scene selection.** Selection picking and
  highlighting are Scene-view-only; the Game view is played, not authored, and no
  longer receives the current selection id.
- **Hierarchy multi-select (Ctrl/Shift-click).** Ctrl+click toggles an entity's
  membership in the selection; Shift+click selects every entity between the last
  clicked ("anchor") entity and the clicked one, ranging over the tree's depth-first
  display order (or the filtered order when a search filter is active). `Delete` (the
  toolbar button and the context menu) now destroys the whole selection. A plain click
  still collapses to a single entity. `EditorContext::selected_entity` remains the
  single "primary" target the Inspector, the viewport gizmo, and Align/Move-to-View
  read; `selected_entities` is the full set only the Hierarchy's bulk operations use.
- **Project panel double-click fix.** Double-clicking a file tile (to open a scene,
  text file, or code file) relied on `ImGui::Button`'s own pressed-on-release return
  value to gate `IsMouseDoubleClicked`, which could miss the second click of a fast
  double-click. Detection now uses `IsItemHovered() && IsMouseDoubleClicked()`
  independent of the button's return value, matching the pattern already proven by the
  Hierarchy's entity double-click.
- **Undo/redo and New Scene.** `Edit ▸ Undo`/`Redo` (Ctrl+Z / Ctrl+Y, ignored while a
  text widget has focus) step through whole-world JSON snapshots via a new
  `CommandHistory` (`editor/command_history.*`), reusing `scene_serializer`'s
  `capture_scene`/`apply_scene` rather than a per-field command hierarchy — simple and
  correct at this entity count, at the cost of coarser granularity than a field-level
  command would give. Continuous edits (a gizmo drag, an Inspector slider held across
  frames) are recorded as one step via `begin_change`/`end_change`, keyed off the
  widget's activate/deactivate edge (or the gizmo's own grab/release edge) so a single
  drag never produces dozens of undo steps; discrete actions (create, delete, rename,
  reparent, checkbox toggles) record with a single `record()` call before the mutation.
  Undoing/redoing clears the current selection, since entity ids are not preserved
  across the destroy-and-reload the snapshot swap performs. `File ▸ New Scene` clears
  the world (itself undoable) and resets the open scene path.
- **`.sushiscene` save/open.** `File ▸ Save Scene` / `Save Scene As...` write the live
  world to a JSON `.sushiscene` file (`editor/scene_serializer.*`); double-clicking one
  in the Project panel (or its context menu's Open) clears the world and reloads it.
  Serialization goes only through `IWorldEditor`'s existing query/mutate surface — no
  new engine-side interface — so it names no runtime or ECS type. Parent links are
  written as indices into the saved array rather than raw `EntityId`s, since ids are
  not stable across a destroy-and-reload; a load resolves them only after every entity
  in the file exists, so a child can be listed before its parent.
- **Hierarchy parenting via drag-and-drop.** Entities can now be nested: dragging one
  onto another in the Hierarchy reparents it (dropping on empty space unparents back
  to root), with a per-child cycle guard so an entity can never end up as its own
  ancestor. `IWorldEditor` gained `parent()`/`set_parent()`; parenting is host-side
  editor metadata (like name/visibility), not an ECS component. The simulation's
  extract step now composes each entity's world matrix by walking its parent chain
  (`RuntimeSimulation::world_matrix`), so a parent's transform propagates to its
  children. A search filter in the Hierarchy still flattens to a plain filtered list,
  since nesting is meaningless once most of the tree is hidden.
- **Unity-style Project window.** The Project panel is now a two-pane file browser: a
  recursive folder tree on the left and a searchable icon-grid of the current folder's
  contents on the right, replacing the old single-list browser. Supports create
  (Folder / C++ Header / C++ Source / Text File), inline rename, delete, "Show in
  Explorer", and double-click open — text files (`.h`/`.hpp`/`.cpp`/`.txt`/etc.) open in
  the built-in text editor, everything else opens via the OS default application
  (`ShellExecuteW` on Windows). The default project root is no longer inside the engine's
  own source tree: it now resolves to `<user profile>/sushiengine/project` (created if
  missing) and is persisted as `last_project_root` in `Preferences` once chosen.
- **Rotate/scale gizmos with a Local/World axis toggle.** `GizmoController` now offers
  a full W/E/R handle set (translate, rotate, scale) plus a toolbar toggle for whether
  Translate/Rotate drag along world axes or the selection's own local axes (`GizmoSpace`);
  Scale always drags in local axes to avoid shearing a rotated object. The rotate ring
  drag now intersects the mouse ray with the axis's own plane through the pivot each
  frame and measures the signed world-space angle swept, instead of a screen-space
  angle — the previous screen-space approach inverted when the camera viewed the axis
  from its far side.

- **Camera component with display selection.** Cameras are now first-class ECS entities
  (a `Camera` component: fov, clip planes, `display_index`, `priority`, `active`), posed
  by their transform and appearing in the Hierarchy. `IWorldEditor` gained
  `create_camera` / `is_camera` / `camera_params` / `set_camera_params`, and `RenderScene`
  now carries the resolved camera per display (`display_cameras`) — the extract step
  picks, for each display, the active camera with the highest priority. The Game view
  has a display dropdown to choose which display it shows, so two or more cameras on
  different displays do not conflict, and the Inspector edits a selected camera's lens
  and routing. A seeded "Main Camera" entity replaces the old hardcoded game camera.
- **Editor Preferences with persisted settings.** An `Edit ▸ Preferences…` window edits
  General (Scalar precision, theme), Editor (autosave, camera move speed), and Scene
  (grid, snap) settings. Persistence sits behind an `IPreferencesStore` abstraction
  (`editor/preferences.hpp`) with a JSON implementation writing
  `<user-config>/SushiEngine/preferences.json` — the UI depends on the interface, not the
  file (dependency inversion). Live-effective fields (theme, camera speed) apply
  immediately; the precision control is compile-time, so it records intent and flags a
  rebuild when it differs from the running binary. Uses the newly declared
  `nlohmann-json` dependency (`cli/sushistack.deps.toml`, provisioned via `ss`).
- **Selectable Scalar precision (`SE_SCALAR_DOUBLE`).** The engine's `Scalar` (and the
  `Vec3`/`Mat4`/`Quat` built on it) can now be single or double precision, chosen at
  build time. It is a compile-time switch because `Scalar` is a typedef baked into
  trivially-copyable components and device storage, so it cannot flip at runtime. The
  option is declared in `cmake/ProjectOptions.cmake` and threaded as one compile
  definition on the `SushiEngine` INTERFACE target (and mirrored on `sushi_render`,
  which shares the value types across its interface structs but does not link the
  engine target), consumed at the single seam in `core/blas_placeholder.hpp`. The GPU
  upload path narrows to 32-bit explicitly, so shader data is unchanged in either
  build. The `se` CLI gained `--double` on `se build` and `se editor`, and a persisted
  `scalar_double` config field, so the choice survives across builds. Both precisions
  build clean and the ECS sandbox validates identically under each.
- **Translate gizmo in the Scene viewport.** The selected entity gets three world-axis
  handles (drawn over the viewport with ImGui's draw list, no extra Vulkan), and
  dragging one moves the entity along that axis, written straight back to its transform
  through `IWorldEditor`. Grabbing a handle never reselects (it takes priority over
  picking), and the mouse-pixel-to-world mapping is captured at grab time so the drag
  stays stable as the object moves. Combined with the Hierarchy's Add/Delete and the
  GameObject menu, the scene is now fully interactive: create, select, move, edit, and
  destroy entities.
- **Click-to-select in the viewport (GPU id-buffer picking) with selection highlight.**
  The scene view now draws a second `R32_UINT` colour target alongside the shaded image,
  writing each instance's picking id per pixel (the grid writes none), copies it to a
  host-visible buffer each frame, and exposes `ISceneView::pick(x, y)` to read the
  entity under a pixel back. A left-click in either viewport selects the entity beneath
  the cursor (right mouse stays reserved for fly navigation), kept in sync with the
  Hierarchy and Inspector. `ISceneView::render` takes the selected id and the mesh
  shader lifts the selected entity toward a warm highlight so it stands out.
  `MeshInstance` carries a 32-bit `id`; the push constant grew the draw id and selected
  id (120 bytes, still within the 128-byte floor) and now reaches the fragment stage.
- **Entity-aware editor over the live world.** The world is now the single source of
  truth for entities: the editor-side scene model (`editor/scene_model.hpp`) is gone,
  and the Hierarchy and Inspector operate directly on the simulation. The simulation
  seam grew an `IWorldEditor` read/write surface (`include/SushiEngine/sim/simulation.hpp`)
  — split from `ISimulation` so a panel that only edits depends on the narrow interface
  (interface segregation) — addressing entities by a stable `EntityId` with query
  (`entities`, `name`, `transform`, `color`, `visible`) and mutation (`create`,
  `destroy`, `set_name`, `set_transform`, `set_color`, `set_visible`) operations. The
  Hierarchy lists the world's entities with select, create, delete, filter, and inline
  / context-menu rename; the Inspector edits the selection's name, visibility,
  transform (Euler in the UI, quaternion in the world), and colour, writing straight
  through to the ECS components. Entities the editor creates carry no motion, so they
  stay where placed and stay editable while the world plays — only the seeded demo
  cubes are driven by the systems. `RenderInstance` now carries its `EntityId` (the
  key the upcoming viewport picking reads back), and the extract skips hidden
  entities. `World::get` gained a `const` overload for read-only host access.
- Editor **Game view**: a second Unity viewport rendering the live world from the
  world's own camera, alongside the Scene view's free-flying camera. The camera is
  factored behind an `ISceneCamera` seam (`editor/scene_camera.hpp`) with a
  navigable `FlyCameraSource` (Scene) and a world-posed `WorldCameraSource` (Game),
  so one `ViewportPanel` — now taking the camera by injection — serves both, and a
  new camera kind is a new implementation rather than a new panel. The two panels
  dock tabbed in the layout centre like Unity, and the Game panel joins the Window
  menu. `ViewportPanel` no longer owns a camera; the Scene fly camera and the Game
  world camera are host-owned and passed in.
- **Live ECS world driven on SushiRuntime**, behind a plain-C++ simulation seam.
  A new `sushi_sim` static library (`sim/`) owns a `SushiRuntime::API::Runtime`, an
  ECS `World`, and a `Schedule`, and drives a world of spinning, orbiting cubes —
  two systems (`spin` advancing orientation, `orbit` advancing position) over
  disjoint components, so the runtime's dependency tracker runs them in parallel,
  exactly as the sandbox proves. Every value a kernel reads is precomputed on the
  host into a component (the per-step rotation quaternion, the per-step orbit
  rotation as a cos/sin pair), so the kernels are pure arithmetic and capture no
  host state — legal device code. The library exposes only the plain-C++
  `ISimulation` seam (`include/SushiEngine/sim/simulation.hpp`: `ISimulation`,
  `create_simulation()`, `RenderScene`, `RenderInstance`, `CameraState`), which
  names no runtime, SYCL, or ECS type — the editor depends on the abstraction, and
  the runtime/SYCL/ECS stay contained in one translation unit. Each tick runs the
  schedule and an extract pass reads the shared-USM columns back on the host into
  the `RenderScene` the editor draws (a host copy today; device-shared interop is a
  later milestone). The editor (`editor/main.cpp`) creates the simulation, ticks it
  only while the toolbar is Playing (binding the existing `PlayState`), draws the
  extracted instances in the Scene viewport, and reports the live entity count in
  the Statistics panel. Turning `SE_BUILD_EDITOR` on now also builds `sushi_sim`,
  so the editor's final link is SYCL-aware and its executable ships the runtime DLL.
- Editor **Scene viewport** with a Unity-style fly camera. A dockable Scene panel
  shows a Vulkan-rendered 3D view — a ground grid and lit cubes — that the offscreen
  scene renderer draws and ImGui samples via `ImGui::Image`. Right-mouse enables
  free-look and WASD/QE flight with Shift to boost, frame-rate independent and active
  only while the panel is interacted with. New pieces: an offscreen scene-view
  abstraction (`include/SushiEngine/render/scene_view.hpp`: `ISceneView`,
  `CameraView`, `MeshInstance`) created by `IWindowRenderer::create_scene_view()`, its
  Vulkan implementation (`render/rhi/vulkan/vulkan_scene_view.*` — double-buffered
  colour+depth targets, a lit-mesh and a flat-line pipeline sharing one push-constant
  layout, drawn with 1.3 dynamic rendering and left shader-readable for ImGui), the
  mesh/line/grid shaders (`render/shaders/mesh.vert`, `mesh.frag`, `line.frag`), and
  the editor's camera/input seams (`editor/fly_camera.hpp`, `camera_controller.hpp`,
  `input_state.hpp`, `viewport_panel.*`). The Dear ImGui adapter gained
  `register_texture`/`unregister_texture` to expose the offscreen target as an ImGui
  texture. The Scene panel joins the Window menu and docks into the layout centre.
- Matrix and quaternion math on the single BLAS seam
  (`include/SushiEngine/core/blas_placeholder.hpp`, exposed via `core/types.hpp`):
  `Mat4`, `Quat`, and the vector/matrix/quaternion operations the renderer and camera
  need (`perspective`, `look_at`, `compose_transform`, `mat4_from_quat`, `mul`,
  `normalize`, `cross`, `dot`, …), documented as placeholders SushiBLAS will own.

### Fixed
- **Rotate gizmo direction flipping with camera angle.** Dragging a rotate ring while
  viewing the axis from the opposite side used to spin the object backwards; the drag
  math is now computed from world-space vectors (ray/plane intersection), which is
  camera-orientation independent instead of a screen-space angle.
- **Game view no longer selects objects.** Clicking in the Game viewport used to pick an
  entity through the id buffer, the same as the Scene view. Picking is now gated to the
  Scene view (a `pickable` flag on the viewport panel); the Game view is played, not
  authored, so a click there never changes the selection.

### Changed
- Editor now presents through Vulkan instead of OpenGL. The `se_editor` shell keeps
  its SDL2 window and Dear ImGui dockspace but renders through the engine's Vulkan
  renderer (`sushi_render`), so the same device that will draw the viewport also
  draws the UI. The change is layered behind three narrow seams so the app loop
  (`editor/main.cpp`) names no windowing or graphics API directly: a windowing seam
  (`editor/platform_window.hpp` `IPlatformWindow`, implemented by
  `editor/sdl_window.*`), the renderer's presentation facade
  (`include/SushiEngine/render/window_renderer.hpp` `IWindowRenderer` /
  `create_window_renderer()`, Vulkan implementation
  `render/rhi/vulkan/vulkan_window_renderer.*` — device + swapchain + frame sync,
  transparent swapchain rebuild on resize), and the Dear ImGui ↔ Vulkan adapter
  (`editor/imgui_backend.*`, the one editor component that speaks Vulkan). The RHI
  device (`render/rhi/device.hpp`) gained a `SurfaceFactory` hook and
  `required_instance_extensions` so the host creates the presentation surface without
  the renderer ever calling a windowing library, plus a `native_handles()` escape
  hatch the ImGui Vulkan backend uses. Turning `SE_BUILD_EDITOR` on now implies
  `SE_BUILD_RENDER`. The SDL2 dependency now requires its `[vulkan]` feature
  (`cli/sushistack.deps.toml`); the unused OpenGL dependency was removed.

### Added
- Renderer foundation (`render/`, `include/SushiEngine/render/`): the first cut of a
  greenfield Vulkan 1.3 renderer, gated behind the `SE_BUILD_RENDER` CMake option
  (OFF by default). Consumers program against an abstract RHI device
  (`render/rhi/device.hpp`: `IRenderDevice`, `create_render_device()`, `DeviceInfo`,
  `RenderDeviceDesc`) that carries no Vulkan types — the dependency-inversion seam a
  future D3D12/Metal backend slots into. The Vulkan backend (`render/rhi/vulkan/`)
  brings up a 1.3 instance and device (dynamic rendering + synchronization2) via
  vk-bootstrap and a VMA allocator, selecting the physical device by preference and
  exposing its UUID — the key a later milestone matches against SushiRuntime's SYCL
  device for zero-copy interop. It renders its first pixels: a one-shot offscreen
  triangle via 1.3 dynamic rendering (`render/rhi/vulkan/vulkan_offscreen.*`), whose
  shaders are compiled from GLSL to embedded SPIR-V at build time by a glslang host
  tool (`render/tools/shader_compiler/`, driven by the `sushi_compile_shader` CMake
  helper) so the renderer carries no runtime shader-compiler dependency. A headless
  `render_probe` target brings the device up, renders the triangle, and reads two
  pixels back to assert the pipeline path (center lit, corner cleared) — validating
  the whole chain (and vcpkg provisioning) without a display, the render analogue of
  the ECS sandbox. Like the editor, the renderer is a plain compiled target — no
  runtime link, no SYCL — so it builds on a stock toolchain. Vulkan, VMA,
  vk-bootstrap, and glslang are declared in `cli/sushistack.deps.toml` and provisioned
  by `ss install`. New `se` CLI command `se render` configures with the flag on,
  builds `render_probe`, and runs it.
- Editor panels (`editor/`): the `se_editor` shell grows a Unity-style panel set on
  top of its SDL2 + Dear ImGui dockspace — Hierarchy, Inspector, Project, a tabbed
  Text Editor, a Toolbar, a Console, and a Statistics panel, plus a status bar —
  laid out with a default dock split on first run. A **Window** menu toggles every
  panel, and each panel's title-bar close button shares that visibility bit so a
  closed panel reopens from the menu. The Hierarchy shows an editor-side scene tree
  with select, create, delete, a search filter, double-click / context-menu rename,
  and mouse drag-and-drop reparenting (drop onto a node to parent, onto empty space
  or via the "Unparent" menu to promote to a root); the Inspector edits the
  selection's name, visibility, and transform; the Project panel is a
  `std::filesystem` browser rooted at the working directory that opens text files
  into the editor; the Text Editor hosts open files as reorderable tabs with
  dirty-state tracking and save; the Toolbar drives a Play/Pause/Step playback state
  (a stub seam for a future World loop); the Console records editor actions; the
  Statistics panel reports frame time, FPS, and entity/document counts. The panels
  operate on an editor-owned `Scene`/`EditorContext` model (`scene_model.hpp`,
  `editor_context.hpp`), keeping the shell free of the runtime and SYCL, so this
  lane still builds on a stock toolchain. `imgui_stdlib` is vendored into the
  `sushi_imgui` library for `std::string`-backed text fields.
- Test suite (`tests/`): a GoogleTest-based functional suite mirroring
  SushiRuntime's layout, gated behind the `SE_BUILD_TESTS` CMake option (OFF by
  default; GoogleTest comes from vcpkg). One binary, `se_functional_tests`, built
  as a SYCL translation-unit set so kernels run against the real runtime — no
  mocks. Tests are grouped into CTest labels (`unit` / `integration` /
  `regression`) from the GTest suite-name prefix convention (`Unit_*` /
  `Integration_*` / `Regression_*`), so `ctest -L unit` selects a sub-suite.
  Coverage: World entity directory mechanics (spawn/get/destroy, generation
  invalidation, swap-remove repointing, structure versioning, queries),
  CommandBuffer deferral, graph-colouring properties, and end-to-end Schedule and
  PGS-solver runs checked against scalar references (`compile_count == 1`).
- Developer CLI (`cli/`): a Typer/Rich command-line tool installed as `se` (and
  `sushiengine`), mirroring SushiRuntime's `sr`. Core surface: `se project
  build/test/run/clean/doxygen`, `se config show`, and `se env dump`. It consumes
  the SushiRuntime sibling's bundled clang++ and vcpkg rather than provisioning a
  toolchain — when `cxx` / `vcpkg_root` are unset they resolve from the runtime's
  `dependencies/` tree, so a normal checkout needs no local config. Layered
  configuration (`cli/config.toml` -> `config.local.toml` -> `SE_*` env vars) and
  a cached MSVC-environment snapshot on Windows.
- Physics constraint solver (substrate-plan WP-3, physics half): a Projected
  Gauss-Seidel solver built on graph colouring. `color_constraints` edge-colours the
  constraints so each colour is a conflict-free batch (no two constraints in a colour
  share a body); `ConstraintSolver` compiles each colour into one parallel task and
  lets the runtime's dependency tracker order the colours into a sequential sweep —
  Gauss-Seidel across colours, parallel within one — repeated for the iteration count
  and compiled once. The solver is generic over the constraint type and its
  projection (a new constraint type is its own POD plus projection), with
  `DistanceConstraint` / `DistanceProjection` provided. The `pgs_demo` example solves
  a hanging chain and checks the device result against a scalar reference running the
  same colours in the same order (max error ~1e-6, compile_count == 1).
- ECS layer (substrate-plan WP-3): the entity / component / system core on top of
  SushiRuntime. Entities are generation-checked handles; components live in
  archetype chunks of structure-of-arrays columns, each column its own runtime
  allocation so the dependency tracker keys on it. A system declares its component
  reads and writes (`Read<T>` / `Write<T>`) and the `Schedule` emits one graph node
  per chunk; the runtime's dependency tracker orders systems by access — conflicting
  ones run in order, disjoint ones (and disjoint chunks) in parallel — so the engine
  writes no scheduler. Node iteration counts are late-bound to each chunk's live
  entity count, so the graph is compiled once and replayed while entities spawn and
  die. Structural changes go through a `CommandBuffer` applied at the frame barrier,
  with O(1) swap-remove destroys; new chunk sets are the only trigger for a rebuild,
  reported by the world's `structure_version`.
- `sandbox` target: the WP-3 worked example and single SYCL translation unit. A
  particle world (Position, Velocity, Mass, Lifetime) driven by `apply_forces`,
  `integrate`, and a parallel `decay_lifetime` system, spawning and destroying
  entities every frame, checked against an independent scalar reference with the
  graph compiled exactly once.
- `core/types.hpp`: the single integration seam for value types. It aliases a
  temporary placeholder today and is the one file to re-point at SushiBLAS when
  that library lands.
- Project governance and docs: `CONTRIBUTING.md`, `SECURITY.md`,
  `CODE_OF_CONDUCT.md`, and `docs/guides/ARCHITECTURE.md`, mirroring SushiRuntime's
  conventions.
