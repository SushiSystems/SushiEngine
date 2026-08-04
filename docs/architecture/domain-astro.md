# Astro

This file covers the astro domain: the ephemeris that places and moves celestial bodies, the
gravitational field and reference frames built on it, and the frame-local authoring boundary that
lets a solar system be placed through one Transform.

## 1. The solar system: ephemeris, gravity, and frames

The `engine/domain/astro/include/SushiEngine/astro/` headers place and move bodies. They are pure
double-precision host code that fills the neutral `Environment` — the engine ships no device code
here. `julian_date.hpp` and `orbital_elements.hpp` propagate the Standish Keplerian rows;
`celestial_bodies.hpp` catalogues each body's radius, colour, pole, `SurfacePreset`, and
`ring_extent` (Saturn's ring span, zero for every other body); `ephemeris.hpp`'s
`fill_environment_sky` assembles them into the local sky each frame and selects the **dominant
body** (the one whose surface is the analytic ground) by the surface hand-off altitude.

**Saturn's rings** are drawn the same analytic, mesh-free way as the bodies themselves.
`fill_environment_sky` threads `ring_extent(BodyId::Saturn)` onto the body's
`CelestialBody::ring_inner_metres`/`ring_outer_metres` (oriented by the body's real `pole`, i.e.
its J2000 equatorial plane) and, once Saturn is the dominant near-field planet, onto
`Environment::planet_ring_*`. `engine/presentation/render/source/scene/scene_uniforms.cpp` packs
the far-field radii into the body record's previously-unused lanes and appends one `planet_ring`
vec4 for the near-field case (after the block's arrays, so the shaders that share the scene block
and read only its earlier fields keep their offsets).

`engine/presentation/render/shaders/sky.frag` then ray-tests the equatorial annulus in both
regimes — the far-field body loop (the ring resolves as the camera nears Saturn past the far-disk
LOD) and the near-field planet — shading it with banded opacity (C/B rings, the empty Cassini
division, the A ring with its Encke gap), fine ringlets and self-gravity-wake clumping, a
back-scatter opposition surge, translucency over the disk, and the planet's shadow cast across
the ring.

Because every body is placed with its true direction, distance, and angular radius in one frame,
**eclipses fall out of the geometry** rather than being scripted. `fill_environment_sky` computes
the solar-eclipse coverage once — the circle-circle overlap (`disk_overlap_fraction`) of the
Sun's disk by any nearer body — and hands it to the renderer as a single scalar
(`Environment::solar_eclipse`, packed into `sky_counts.w`); the sky, PBR, and cloud passes all dim
the direct sun by it, so the whole scene dusks toward totality together. The lunar eclipse is the
mirror case: the Moon's disk against Earth's umbra at the anti-solar point, folded into the Moon
body's colour and brightness on the CPU (a coppery, dimmed disk) with no shader involved. Both
are Earth-consistent and ephemeris-driven, so they occur only at the real alignments.

**Every body in that sky is also a light.** The ephemeris turns the body list into an ordered
`CelestialLight` array (`Environment::lights`), and the Sun is not a special case in it: an
emitter's irradiance is authored, a reflector's follows from the definition of geometric albedo —
incident irradiance times `albedo * (radius/distance)^2` times `phase_brightness`, which is a
Lambert sphere for smooth bodies and Allen's lunar fit (opposition surge included) for regolith
ones. Nothing names a body, so the Moon over Earth, Jupiter over Europa, and earthshine on the
Moon are one code path, and the numbers are real — a full Moon lands near `2.8e-6` of sunlight.

The list is ordered by what each light *delivers* at the camera (irradiance weighted by
elevation), which is what lets the single shadow-cascade atlas belong to the Sun by day and to
the dominant reflector after sunset; the PBR pass and the analytic ground both loop the array,
shading the rest with the same BRDF minus the cascades. Because the lights are physical, so is
the exposure path: the auto-exposure histogram's floor reaches `2^-20` to meter a moonlit surface
rather than crush it, and `PlanetParameters::ocean_roughness` gives the ocean mask a tight GGX
lobe, so a body low over water draws a glitter path instead of a round highlight.

The pipeline is organised around **three coordinate spaces**: *solar* (heliocentric ecliptic
J2000, double metres — where every body lives), *planet* (body-fixed per body, origin at its
centre, turning with its pole and spin — where surface entities live), and *local* (the
camera-relative scene frame the renderer draws in, origin at the observer's surface point, +Y
along the geodetic normal).

The transforms between them are fully body-parametric, so the sky is built the same way on any
planet rather than only Earth: `SkyObserver::observer_body` names the body the observer stands
on, `fill_environment_sky` anchors the scene origin to *that* body's surface and places every
other body relative to it, and `engine/domain/astro/include/SushiEngine/astro/body_orientation.hpp`
supplies the per-body spin and the rotation of a direction into the observer body's equatorial
frame (`ecliptic_to_body_equatorial` / `equatorial_to_body_equatorial`). Earth is routed through
the exact fixed-obliquity conversion and sidereal time so the home sky is unchanged; every other
body gets its true pole and day.

The one subtlety in that, and it is load-bearing: **`prime_meridian_angle` is the spin, not
`body_rotation_angle`.** The latter is the IAU angle W, which is measured from the ascending node
of the body's equator on the *J2000 equator*, while the frame above puts its +X on the node with
the *ecliptic*. Both are legitimate frames with the pole as +Z; they are not the same frame, and
the two nodes are 52.7° apart for the Moon and 117.6° for Venus. Anything asking "where is the
prime meridian right now" — the topocentric sky, the scene-frame bijection, the body-fixed pose
conversions, and the terrain, which reads elevations indexed by real coordinates — calls
`prime_meridian_angle`. Earth resolves to Greenwich sidereal time by definition, which is why it
is exempt from all of this.

The *planet* space is what `Environment::planet_body_axes` publishes to the renderer: three
scene-frame columns whose third is `planet_pole`, filled by the ephemeris because nothing
downstream can derive a body's pole and meridian, and carried as plain vectors so the render seam
stays free of the astro layer. The editor re-anchors `observer_body` to whichever body the camera
is on and rebases the camera on a change, so time animation and precision hold on every planet,
not just Earth.

Three modules give that model gravity and a planet-relative transform, all bodies handled by the
same parametric code (Mercury–Pluto and the Moon, no per-body branches):

- **`gravity.hpp`** — the gravitational field. Bodies stay on their analytic Keplerian rails (the
  *sources*); a free entity is a `StateVector` integrated through the summed Newtonian field
  `gravity_field()` by a symplectic velocity-Verlet `integrate_step()`. Keeping the planets on
  rails makes the field a deterministic function of position and time — the property SushiLoop's
  lockstep needs — and stops long orbits drifting the way a fully dynamic N-body system would.
  `standard_gravitational_parameter()` tabulates GM per body; `sphere_of_influence_radius()`
  sizes the Laplace SOI. The inverse-square field is evaluated **only in double** at the seam:
  `|r|³` over ~1e11 m collapses in the physics solve's optional single precision.

- **`reference_frame.hpp`** — the active reference frame. A `ReferenceFrame` is body-centred but
  keeps inertial (ecliptic) axes, so a state expressed in it is the heliocentric state minus the
  body's own — a Galilean shift with no fictitious terms. `active_frame_body()` picks the most
  local dominant attractor at a point; `rebase()` is the single double-precision coordinate
  change a sphere-of-influence crossing triggers, the orbital analogue of the floating-origin
  sector rebase (see [the value-type seam](foundation.md#2-the-value-type-seam)).

- **`surface_frame.hpp`** — the body-fixed surface frame that makes a planet-relative pose work.
  An entity near a body stores its position as body-fixed Cartesian metres (ECEF,
  `geodetic_to_body_fixed` / `body_fixed_to_geodetic`, with lat/lon only as a boundary conversion
  for authoring and the map) and its orientation relative to the local East-North-Up
  `local_tangent_basis()` at that position. Because the tangent basis is *derived from position*,
  "upright, facing north" is identity orientation everywhere on the body — the reason a
  southern-hemisphere entity stands straight rather than tilted. `surface_gravity_vector()` is
  the near-field pull: the inward ellipsoid normal times `surface_gravity()`, correctly oriented
  over the whole body.

- **`gravity_field.hpp`** — `IGravityField`, the field behind a dependency-inversion seam, and
  `SummedRailsGravityField`, the default on-rails summation. The orbital integrator names the
  interface, so a patched-conic or full N-body field can replace it without the integrator
  changing.

- **`astro_dynamics.hpp`** — `advance_astro_state()` joins one field-parameterised
  `integrate_step` and the SOI `rebase` into the single per-step authority update: lift the
  body-centred state to heliocentric, step it through the injected field, re-select the active
  frame from where it lands, express it there.

- **`scene_frame.hpp`** — `SceneFrame`, the exact rigid bijection between a heliocentric-ecliptic
  position and the scene's local frame, reproducing the ephemeris's scene construction so a free
  body and the planet it orbits line up. `topocentric.hpp` holds the observer's East-Up-South
  basis (shared with the ephemeris, DRY); `body_orientation.hpp`'s `body_equatorial_to_ecliptic`
  is its inverse rotation.

`RuntimeSimulation` consumes this two ways. **Per-body gravity:** each physics step builds a
`Simulation::GravitySampler` (`make_gravity_sampler`) and hands it to `IPhysicsStepper::step`,
which samples it at every body's own position each sub-step
(`PhysicsWorld::predict_substep_field`). The sampler maps a body's scene position to
heliocentric, samples the injected `Astro::IGravityField` — the *same* `SummedRailsGravityField`
the orbital integrator uses, so gravity has one source — and rotates the acceleration back into
scene axes, so each body feels the true field (1/r² falloff, curvature toward the attractor,
third-body terms) rather than one vector shared by the whole scene. Sampling at the current
position keeps the semi-implicit predict symplectic. With no dominant body it falls back to a
uniform demo-gravity sampler.

There is no separate "astro body" mode: a body's orbital motion emerges from this same per-body
gravity plus its velocity, through the one physics path (the exclusive astro toggle and its
parallel `derive_astro_transforms` pose-derivation were removed; the `Astro::` dynamics modules
they used remain and now back the gravity sampler, and — ahead — the unified dynamic body's Free
authority). The simulation still owns the **master epoch**: `julian_date()` advances by the fixed
step (scaled by `set_time_scale_days_per_second`), the editor drives the sky from it through
`set_sky_observer`, and the extracted snapshot carries it back — one clock for orbits, planets,
and the rendered sky.

Two planet-relative constraints run in `RuntimeSimulation::apply_surface_constraints` (from
`extract`, so they hold both while playing and after an edit), gated on a dominant body so plain
non-astronomical scenes are untouched:

- **Surface anchoring** — an entity toggled through `set_surface_anchored` stores its orientation
  *ground-local* (relative to the East-North-Up tangent frame at its position); the pass composes
  the tangent frame onto it, so "upright" is identity everywhere on the body and a
  southern-hemisphere entity stands straight rather than tilted. "Up" is the **geodetic normal**
  (`surface_normal_scene`, the ellipsoid gradient), not the geocentric radial, so a flattened
  body's local vertical is exact — matching `Astro::geodetic_normal` in `surface_frame.hpp`. This
  is host-side `Record` bookkeeping (like colliders and cloth), not an ECS component — no
  Schedule system reads it, and it needs the `Environment` the systems do not see.

- **The planet collider** — every entity is kept outside the reference ellipsoid (its true
  flattened radius along the outward direction); a penetrating rigid body is re-posed through the
  physics seam so the surface is a hard floor. The editor's Scene fly-camera is *not* clamped: it
  flies freely and may enter a body (Unity/Blender behaviour), the infinite-far reverse-Z
  projection carrying the depth range.

**Frame-local authoring.** An entity carries a **reference frame** (`Simulation::EntityFrame` = a
celestial body index + `FrameMode` Auto/Free/Surface) — the Unity-parent analogue with a body as
the parent. It is an *authoring-boundary projection*, not a second source of truth: the
scene-frame `Transform` stays what physics and render read, and `frame_local_transform` /
`set_frame_local_transform` convert between the frame-local pose and the scene `Transform`
through the scene-frame bijection at the master epoch.

In **Surface** mode the frame-local position is a **geodetic** coordinate (latitude, longitude,
altitude — `scene_to_body_fixed` / `body_fixed_to_scene` unwind the body's spin `W(t)`,
`surface_frame.hpp` does the ellipsoid conversion), so a spawn is placed the way a map reads; in
**Free** it is a Cartesian offset from the body's scene centre (`reference_center_scene`). This is
what lets the whole solar system be placed through one Transform without typing heliocentric
numbers, and it is the reference descriptor — frame-independent — that a future networking layer
syncs for zero-conflict (never the per-client scene `Transform`). A reference body of -1 is the
scene root, so an entity that never picks a body is unchanged. Surface mode drives the surface
anchoring above.
