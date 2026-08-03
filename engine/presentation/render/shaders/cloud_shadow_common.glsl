// The clouds' shadow on a lit surface: one fetch of CloudShadowMapPass's baked optical
// depth toward the sun (W2's unified shadow authority) turned into a transmittance with
// the same Beer-Lambert scale the cloud march itself uses. sky.frag's ground shadow and
// this file's mesh consumer both call this same function against the same map — the
// design doc's "one mechanism instead of two" — so a surface mesh standing on the
// analytic ground is always shadowed by the same reading the ground itself is.
//
// The map is baked along the primary sun direction only (`scene.sun_dir`), not per
// caller-supplied light direction — the one simplification worth naming: a secondary
// body (the Moon) lit surface reuses the same overhead reading rather than its own ray,
// which the codebase already tolerates elsewhere (mesh/ground shadowing from secondary
// punctual casters is its own disabled path today, see pbr.frag's `false &&` guard).
//
// **Phase B: the map became a place, not a pattern.** It used to be addressed by
// `fract(position.xz / tile)`, so one 32 km bake was repeated across the whole world — which
// was defensible only while the sky above it was uniform. Now that the cloudscape bake
// resolves coverage per column (docs/slop/atmosphere_system.md §7.4), repeating it would
// stamp one region's shadows onto every other region's ground. It instead covers exactly the
// camera-centred near window, and past that window's rim the shadow fades to none.
//
// That fade is a named limit, not an oversight: ground more than ~16 km from the camera loses
// its cloud shadow entirely. Before phase B it had one, but it was the *wrong* one (a
// repeated copy of the camera's own patch), and at that distance a wrong dappling reads as
// texture while a missing one reads as haze. Reaching further wants a second, coarser shadow
// cascade over the far window — the natural follow-up, and cheap, but a separate change.

// How far across the window's outer rim the shadow fades to none. Matches
// cloud_field_window.glsl's CLOUD_WINDOW_BLEND_START so the ground stops being shadowed over
// exactly the band where the sky itself stops reading the near field.
const float CLOUD_SHADOW_FADE_START = 0.84;

float cloud_sun_transmittance(sampler2D shadow_map, vec3 position)
{
    if (scene.misc.w <= 0.5 || scene.cloud_global.x <= 0.0)
        return 1.0;
    // An unbaked window publishes a zero scale; there is no map to read yet.
    if (scene.cloud_field_near.x == 0.0)
        return 1.0;

    // Camera-relative metres straight into the near window's UV — the same mapping the view
    // march, the light volume and the shadow bake all use, so the shadow on the ground and
    // the cloud casting it are never a window apart.
    vec2 uv = position.xz * scene.cloud_field_near.xy + scene.cloud_field_near.zw;
    vec2 centred = abs(uv - vec2(0.5)) * 2.0;
    float reach = 1.0 - smoothstep(CLOUD_SHADOW_FADE_START, 1.0, max(centred.x, centred.y));
    if (reach <= 0.0)
        return 1.0;

    float depth = texture(shadow_map, clamp(uv, vec2(0.0), vec2(1.0))).r;

    float extinction_scale = scene.cloud_light.x * 0.006;
    float transmittance = exp(-depth * extinction_scale);
    return mix(1.0, transmittance, clamp(scene.cloud_global.x, 0.0, 1.0) * reach);
}
