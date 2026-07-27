// The cloudscape field's addressing, shared by everything that reads it: the view march,
// the light volume, the cloud shadow map, the far-field panorama, and the ground/mesh
// shadow lookup. One file so those five can never disagree about which piece of the world a
// texel is.
//
// docs/slop/atmosphere_system.md §7.2. The field used to be a *periodic 32 km tile*: a march
// sample took `fract(p.xz / tile)`, which is coherent only while the weather above it is the
// same everywhere. It is not any more — the bake resolves coverage and genus per column from
// the simulated field (§7.4) — so the tile has to become a **camera-centred, non-wrapping
// window**, and that single change is what closes phase A's three deferred items at once:
//
//   * `fract()` is many-to-one, so a bake addressed in tile space holds a `uv` with no
//     recoverable world position. That is why CloudLightVolumePass and CloudShadowMapPass
//     could not consult the weather field and still lit/shadowed every cell as if it carried
//     the observer's own column. Addressed in *window* space the mapping is invertible, and
//     in fact they no longer need to consult anything: the field they march already carries
//     the simulation's coverage, so a cleared region stops receiving cloud shadow because
//     there is no cloud in the field there to cast it.
//   * Genus could not be derived per column while every column baked the same deck stack.
//
// Two windows, not one. The march runs out to roughly fourteen shell thicknesses (~150 km),
// and a single window that reached that far would put ~600 m between texels right next to
// the camera. So the near window keeps the old 32 km span and its 128 m texel, and a far
// window carries the same simulated structure out past the horizon at a coarse texel; a
// sample crossing out of the near window cross-fades into it over the window's outer rim
// rather than clamping to a smeared edge. Both windows are baked from the same weather
// field, so the two agree about where the weather is and differ only in how finely they
// resolve its shape — which is exactly what makes the cross-fade invisible.

// Where the near window starts giving way to the far one, as a fraction of the way from the
// window's centre to its edge (Chebyshev, since the window is a square in world XZ). Wide
// enough that the blend spans several kilometres and never reads as a ring, tight enough that
// the near window's own edge texels — the ones a CLAMP_TO_EDGE fetch would smear — are past
// it and contribute nothing.
const float CLOUD_WINDOW_BLEND_START = 0.84;

// The far field stores optical depth toward the sun in an 8-bit channel; this is the metre
// range that byte spans. Past ~1500 m of depth the Beer term has collapsed at every
// extinction the medium is authored with, so 4 km of range costs no visible precision at the
// depths that still shade differently, and keeps the quantisation step (16 m) far below what
// a coarse far-field sample could resolve anyway.
const float CLOUD_FAR_SUN_DEPTH_METERS = 4096.0;

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
