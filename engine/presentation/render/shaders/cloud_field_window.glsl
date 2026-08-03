// The cloudscape field's addressing, shared by everything that reads it: the view march,
// the light volume, the cloud shadow map, the far-field panorama, and the ground/mesh
// shadow lookup. One file so those five can never disagree about which piece of the world a
// texel is.
//
// docs/slop/atmosphere_system.md §7.2. The field is a **camera-centred, non-wrapping window**
// rather than a *periodic 32 km tile*. A tile addressed by `fract(p.xz / tile)` is coherent
// only while the weather above it is the same everywhere, and it is not: the bake resolves
// coverage and genus per column from the simulated field (§7.4). Two things the window gives
// and a tile cannot:
//
//   * `fract()` is many-to-one, so a bake addressed in tile space holds a `uv` with no
//     recoverable world position, and CloudLightVolumePass and CloudShadowMapPass could not
//     consult the weather field at all — they would light and shadow every cell as if it
//     carried the observer's own column. Addressed in *window* space the mapping is
//     invertible, and in fact they need consult nothing: the field they march already carries
//     the simulation's coverage, so a cleared region receives no cloud shadow because there
//     is no cloud in the field there to cast it.
//   * Genus cannot be derived per column while every column bakes the same deck stack.
//
// Two windows, not one. The march runs out to roughly fourteen shell thicknesses (~150 km),
// and a single window that reached that far would put ~600 m between texels right next to
// the camera. So the near window keeps a 32 km span and its 128 m texel, and a far
// window carries the same simulated structure out past the horizon at a coarse texel; a
// sample crossing out of the near window cross-fades into it over the window's outer rim
// rather than clamping to a smeared edge. Both windows are baked from the same weather
// field, so the two agree about where the weather is and differ only in how finely they
// resolve its shape — which is exactly what makes the cross-fade invisible.

// CloudsV2 (docs/slop/atmosphere_system.md §7.6): the field stores the *envelope* — where
// cloud may exist (r = coverage) and how much water fills it (a, encoded half) — and the view
// march carves the actual shape out of that envelope analytically, per sample. Everything
// that integrates the field along a sun ray (the light volume, the far light channel, the
// cloud shadow map, the panorama impostor) therefore over-counts the carved cloud's real
// mass unless it scales by the carve's expected yield. One shared constant so no two
// consumers can disagree about how heavy the same cloud is.
//
// Derived against the carve's solidity push and its edge-scaled erosion (cloud.frag):
//   * The threshold keeps exactly `envelope` of the volume and the survivors are uniform on
//     [0, 1] — that is the CDF uniformisation's guarantee.
//   * `min(shape * 2.5, 1)` maps that uniform ramp to a mean of 0.8: 60 % of the kept volume
//     saturates at 1, and the remaining 40 % averages 0.5.
//   * The erosion's bite is scaled by `1 - shape`, so it cannot touch the saturated 60 %
//     and only reshapes the rim. Integrating the remap over the ramp leaves ~0.78, and the
//     near-field fine octave takes it to ~0.75 averaged over a sky.
// The constant has to track the carve. A value stated for a thinner one — 0.45, the yield of a
// bare threshold — would have every sun depth integral, cloud shadow and impostor light a sky
// 40 % thinner than the one being drawn, which reads as flat, washed-out cloud: the shading
// and the geometry disagreeing about how much cloud is there.
const float CLOUD_ENVELOPE_MEAN_SHAPE = 0.75;

// ---- Altitude on an oblate planet ----------------------------------------------------------
//
// The radius of the reference ellipsoid along a geocentric direction @p up. Every consumer that
// turns a *position* into a cloud *altitude* must go through this, and a planet-scale shell is
// exactly where a spherical reference stops being good enough.
//
// The shells are spheres of radius `Environment::planet_surface_reference_metres`, and that
// value is `length(observer_center)` — **the observer's own geocentric radius**. Read its
// comment and the intent is plain and, for a local scene, exactly right: it puts altitude zero
// at the ground *under the camera*, which is worth kilometres of air density at mid latitudes
// against the naive choice of the equatorial radius. It holds only near the camera.
//
// Once the shell is planetary the assumption holds nowhere but under the observer. WGS84's
// geocentric radius runs from 6 356 752 m at the pole to 6 378 137 m at the equator, so a
// sphere fitted at one latitude is wrong by up to 21 km at another — and the reader subtracts
// it from a position to get an altitude that a 1 300 m deck is then placed against. With the
// observer at 41° N the sphere sits at ~6 368 900 m, which puts the ground 9.2 km *above* it
// at the equator and 12.1 km *below* it at the pole: the deck would be buried underground
// across the tropics and sitting in the stratosphere over the caps, sweeping smoothly between
// the two and crossing every boundary in the deck stack on the way. Each crossing is a circle
// of constant latitude, which reads on screen as ring banding at fixed latitudes.
//
// Fixing the altitude fixes everything downstream on its own, which is what makes this
// containable: the horizon gate takes the *ratio* of a radius to the surface, so it is right as
// soon as the altitude is, and the atmosphere LUTs are parameterised by altitude above their
// own spherical bottom, so feeding them a true altitude is strictly better than feeding them a
// latitude-dependent error. The Bruneton medium therefore deliberately stays spherical — that
// is the parameterisation, not an oversight — and only the geometry becomes oblate.
//
// Exact rather than approximate, and cheap: a point r*up lies on x²/a² + y²/a² + z²/b² = 1 when
// r²((1 - s²)/a² + s²/b²) = 1 for s = dot(up, pole). One dot product and one inverse square root.
float cloud_planet_radius_at(vec3 up, vec3 pole, float semi_major, float semi_minor)
{
    float s = dot(up, pole);
    float s2 = clamp(s * s, 0.0, 1.0);
    float a2 = max(semi_major * semi_major, 1.0);
    float b2 = max(semi_minor * semi_minor, 1.0);
    return inversesqrt(max((1.0 - s2) / a2 + s2 / b2, 1e-30));
}

// The altitude of @p p above that ellipsoid, with its geocentric direction returned so a caller
// that needs the local up (most of them do) does not normalise twice.
float cloud_planet_altitude(vec3 p, vec3 center, vec3 pole, float semi_major, float semi_minor,
                            out vec3 up)
{
    vec3 radial = p - center;
    float radius = max(length(radial), 1.0);
    up = radial / radius;
    return radius - cloud_planet_radius_at(up, pole, semi_major, semi_minor);
}

// Where a ray enters and leaves a shell surface, on the usual `vec2(-1)`-on-a-miss contract, so
// a march's bounds can be swapped from a sphere to the ellipsoid without its structure changing.
// Shared for the same reason as everything else here: the view march and the panorama impostor
// must bound the same shell, or the impostor continues the sky at a different altitude than the
// march ends it at.
//
// The shell surfaces are the reference ellipsoid with both semi-axes offset by an altitude. That
// is not the exact constant-altitude offset surface — a true offset of an ellipsoid is not an
// ellipsoid — but the discrepancy is of order h*f, about 40 m at a 12 km shell top against
// WGS84's flattening. Beside the 21 km a fitted sphere is wrong by, the approximation is not
// worth the closed form it would cost to avoid.
vec2 cloud_ray_shell(vec3 ro, vec3 rd, vec3 c, float a, float b, vec3 pole)
{
    vec3 o = ro - c;
    float o_ax = dot(o, pole);
    vec3 o_rad = o - pole * o_ax;
    float d_ax = dot(rd, pole);
    vec3 d_rad = rd - pole * d_ax;
    float inv_a2 = 1.0 / max(a * a, 1.0);
    float inv_b2 = 1.0 / max(b * b, 1.0);
    float qa = dot(d_rad, d_rad) * inv_a2 + d_ax * d_ax * inv_b2;
    float qb = dot(o_rad, d_rad) * inv_a2 + o_ax * d_ax * inv_b2;
    float qc = dot(o_rad, o_rad) * inv_a2 + o_ax * o_ax * inv_b2 - 1.0;
    float h = qb * qb - qa * qc;
    if (h < 0.0 || qa <= 0.0)
        return vec2(-1.0, -1.0);
    h = sqrt(h);
    return vec2((-qb - h) / qa, (-qb + h) / qa);
}

// Decodes the field's water-amplitude channel (a): the in-cloud extinction multiplier,
// stored at half scale so an authored density_scale of up to 2 survives the UNORM channel.
float cloud_field_water(float encoded)
{
    return encoded * 2.0;
}

// Prefixed so a shader may include this alongside its own `remap` without a redefinition;
// every file that wants one already has one.
float cloud_remap(float v, float a, float b, float c, float d)
{
    return c + (v - a) / (b - a) * (d - c);
}

// A deck's vertical profile: 0 at its base and top, 1 through its middle, with the shoulder
// widths set by what kind of cloud it is. Lives here rather than in the bake because the
// planet-scale far field (cloud.frag, §7.5) has to build the same envelope the bake would
// have, for the part of the world no window covers — and an envelope that disagreed about
// where a deck's top is would show as a step at the far window's rim.
float cloud_height_gradient(float height01, float stratiform, float anvil)
{
    float cumuliform = clamp(cloud_remap(height01, 0.0, 0.15, 0.0, 1.0), 0.0, 1.0) *
                       clamp(cloud_remap(height01, 0.55, 1.0, 1.0, 0.0), 0.0, 1.0);
    float sheet = clamp(cloud_remap(height01, 0.0, 0.08, 0.0, 1.0), 0.0, 1.0) *
                  clamp(cloud_remap(height01, 0.80, 1.0, 1.0, 0.0), 0.0, 1.0);
    float tower = clamp(cloud_remap(height01, 0.0, 0.10, 0.0, 1.0), 0.0, 1.0) *
                  clamp(cloud_remap(height01, 0.90, 1.0, 1.0, 0.0), 0.0, 1.0);
    float g = mix(cumuliform, sheet, stratiform);
    return mix(g, tower, anvil);
}

// Where the near window starts giving way to the far one, as a fraction of the way from the
// window's centre to its edge (Chebyshev, since the window is a square in world XZ). Wide
// enough that the blend spans several kilometres and never reads as a ring, tight enough that
// the near window's own edge texels — the ones a CLAMP_TO_EDGE fetch would smear — are past
// it and contribute nothing.
const float CLOUD_WINDOW_BLEND_START = 0.84;

// The vertical texel counts of the two volumes the march's empty-space probe reads, mirroring
// CloudscapeCompilePass::FIELD_RESOLUTION_Y and SKIP_DOWNSAMPLE_Y.
//
// The horizontal cell sizes arrive per frame in cloud_field_params.zw, because they follow the
// window spans; the vertical ones cannot, because both cloud_field_params and
// cloud_field_pattern are full. Mirroring them as constants is the honest alternative and not a
// new kind of debt: the downsample factor itself is already declared this way (see
// cloudscape_skip.comp's own note) precisely because these are fixed engine sizes rather than
// tiered settings. The march needs them to compute the exact distance to the boundary of the
// region a probe proved empty, which is what makes empty-space skipping conservative.
const float CLOUD_SKIP_RESOLUTION_Y = 16.0; // FIELD_RESOLUTION_Y / SKIP_DOWNSAMPLE_Y
const float CLOUD_FAR_RESOLUTION_Y = 32.0;  // FIELD_RESOLUTION_Y

// The far field stores optical depth toward the sun in an 8-bit channel; this is the metre
// range that byte spans. Past ~1500 m of depth the Beer term has collapsed at every
// extinction the medium is authored with, so 4 km of range costs no visible precision at the
// depths that still shade differently, and keeps the quantisation step (16 m) far below what
// a coarse far-field sample could resolve anyway.
const float CLOUD_FAR_SUN_DEPTH_METERS = 4096.0;

// The far sun depth crosses that 8-bit channel square-root encoded. Stored linearly, the
// byte's step is a flat 16 m everywhere — including at the shallow depths where the Beer term
// still moves fast and a one-step difference is a visible band of light, which is exactly
// where the near/far crossover ring made it read as a seam. The square root spends the byte's
// resolution where the shading still changes: the first step off zero is 0.06 m of depth, and
// the 16 m steps land beyond ~1 500 m, where the Beer term has already collapsed at any
// authored extinction. Writer and readers share these two so they cannot disagree.
float cloud_far_sun_depth_encode(float depth_meters)
{
    return sqrt(clamp(depth_meters / CLOUD_FAR_SUN_DEPTH_METERS, 0.0, 1.0));
}

float cloud_far_sun_depth_decode(float encoded)
{
    return encoded * encoded * CLOUD_FAR_SUN_DEPTH_METERS;
}

// Camera-relative XZ metres -> a window's UV. Outside [0, 1] means "past that window";
// callers weigh the near window's answer with cloud_window_near_weight rather than clamping
// blindly, and the sampler's CLAMP_TO_EDGE only ever catches the sub-texel overhang.
vec2 cloud_window_uv(vec2 map_scale, vec2 map_offset, vec2 xz)
{
    return xz * map_scale + map_offset;
}

// How much of a window's answer applies at @p uv: 1 through the middle, falling to 0 across
// the rim. For the near window that is where the far one takes over; for the far window it is
// where cloud stops existing at all, and that is deliberate.
//
// **Why the far window must fade rather than clamp.** These windows are flat squares in world
// XZ with no vertical extent — which makes them, geometrically, *infinite vertical prisms*.
// That is exactly right within a few hundred kilometres of the camera, where the tangent-plane
// approximation the whole T3 tier is built on holds. It is meaningless at planetary viewing
// distance: a prism centred near the planet's own axis runs straight down it and intersects the
// spherical cloud shell in precisely two places, the north and south polar caps, which is what
// a clamped edge texel smeared across the rest of the sphere looks like from orbit. Fading to
// nothing instead is the honest answer — the tier simulates a region, not a planet, and §7.5's
// coarse planet-scale far field and panorama impostor are what is supposed to cover the rest.
float cloud_window_weight(vec2 uv, float span_meters)
{
    if (span_meters <= 0.0)
        return 0.0;
    vec2 centred = abs(uv - vec2(0.5)) * 2.0; // 0 at the window centre, 1 at its edge
    float edge = max(centred.x, centred.y);
    return 1.0 - smoothstep(CLOUD_WINDOW_BLEND_START, 1.0, edge);
}

// The inverse: the camera-relative XZ metres a window texel stands over. This is the half a
// wrapping tile could not answer, and the reason the bake can look the weather field up at
// the right place and the derived bakes can march real world distances.
vec2 cloud_window_position(vec2 map_scale, vec2 map_offset, vec2 uv)
{
    // scale is 1/span and never zero for a window that has been baked; the guard keeps an
    // unbaked window (span 0, scale 0) from producing infinities rather than an early-out
    // every caller would have to remember to write.
    vec2 safe = vec2(map_scale.x != 0.0 ? map_scale.x : 1.0,
                     map_scale.y != 0.0 ? map_scale.y : 1.0);
    return (uv - map_offset) / safe;
}
