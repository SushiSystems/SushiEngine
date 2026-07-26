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

// Matches CloudscapeCompilePass::tile_meters() on the C++ side: the T3 field (and every
// bake taken from it, including this shadow map) wraps a fixed flat tile this size.
const float CLOUD_FIELD_TILE_METERS = 32768.0;

float cloud_sun_transmittance(sampler2D shadow_map, vec3 position)
{
    if (scene.misc.w <= 0.5 || scene.cloud_global.x <= 0.0)
        return 1.0;

    // Same UV convention cloud.frag's cloud_density uses: deck 0's authored wind stands
    // in for "the" wind the whole flat tile advects under.
    vec3 wind = scene.cloud_deck_c[0].xyz * scene.misc.z;
    vec2 uv = fract((position.xz + wind.xz) / CLOUD_FIELD_TILE_METERS);
    float depth = texture(shadow_map, uv).r;

    float extinction_scale = scene.cloud_light.x * 0.006;
    float transmittance = exp(-depth * extinction_scale);
    return mix(1.0, transmittance, clamp(scene.cloud_global.x, 0.0, 1.0));
}
