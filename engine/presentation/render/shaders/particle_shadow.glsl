#ifndef PARTICLE_SHADOW_GLSL
#define PARTICLE_SHADOW_GLSL

// The sun's visibility at a particle, sampled from the cascade atlas.
//
// A separate sampler from the mesh path's on purpose. A puff of smoke is not a surface:
// it has no normal to offset the sample along, so the normal-bias acne fix the meshes
// rely on has nothing to work with, and it is drawn with heavy overdraw, so the mesh
// path's blocker search plus twenty-odd filter taps would be paid once per overlapping
// sprite layer rather than once per pixel. This is the cheap form: pick the cascade,
// project once, and take a small fixed-radius Vogel disc through the comparison sampler.
// The result is a soft, chunky shadow, which is exactly what a volumetric medium wants —
// the shadow of a building falling across a smoke column reads as a soft gradient across
// the puff, not as a stencilled edge.
//
// Binding-free by design (the atlas arrives as a parameter, like the clustered-lighting
// helpers), so a pass declares the atlas wherever its own set has room.

#include "shadow_common.glsl"

// Taps in the disc. Four is enough at this radius: the sprite's own radial falloff and
// the alpha blend hide the residual, and the count is paid per overdrawn layer.
#define PARTICLE_SHADOW_TAPS 4

// How many texels wide the filter spreads. Fixed rather than penumbra-driven — measuring
// the blocker gap would cost a second atlas and a search loop for a receiver whose own
// silhouette is a blur.
#define PARTICLE_SHADOW_RADIUS 2.5

/**
 * The sun's visibility at a particle centre.
 *
 * @param atlas       The sun cascade atlas, as a comparison sampler.
 * @param position    The particle's camera-relative position — the space the cascade
 *                    matrices are fitted in.
 * @param view_depth  Positive distance down the view axis, which selects the cascade.
 * @param rotation    Per-pixel rotation of the tap disc, radians.
 * @return 1.0 fully lit, 0.0 fully shadowed.
 */
float particle_sun_shadow(sampler2DShadow atlas, vec3 position, float view_depth,
                          float rotation)
{
    if (shadows.flags.x < 0.5)
        return 1.0;

    int cascade = select_shadow_cascade(view_depth);
    vec4 light_clip = shadows.cascade_view_projection[cascade] * vec4(position, 1.0);
    vec3 light = light_clip.xyz / light_clip.w;
    vec2 uv = light.xy * 0.5 + 0.5;
    // Outside the cascade there is no information, so the particle is lit. Guessing the
    // other way would darken every particle past the shadow distance.
    if (any(lessThan(uv, vec2(0.0))) || any(greaterThan(uv, vec2(1.0))) || light.z > 1.0)
        return 1.0;

    // No normal offset is possible, so the whole bias budget goes into the depth bias. The
    // factor covers what the normal offset would have handled on a surface; a particle is
    // a floating point sample, so over-biasing only shifts where its shadow starts, and
    // never prints acne across a face the way it would on geometry.
    float reference = light.z - shadows.bias.x * 3.0;

    vec2 texel = shadow_atlas_texel();
    vec2 tile = shadow_tile_origin(cascade);
    float total = 0.0;
    for (int i = 0; i < PARTICLE_SHADOW_TAPS; ++i)
    {
        vec2 offset = vogel_disc(i, PARTICLE_SHADOW_TAPS, rotation);
        vec2 tap = shadow_tile_clamp(uv * shadows.params.y +
                                         offset * PARTICLE_SHADOW_RADIUS * texel,
                                     texel);
        total += texture(atlas, vec3(tile + tap, reference));
    }
    float visibility = total / float(PARTICLE_SHADOW_TAPS);

    // The last cascade fades out rather than ending, matching the mesh path — otherwise a
    // drifting particle would pop from shadowed to lit as it crossed the shadow distance.
    if (cascade + 1 == int(shadows.params.x))
    {
        float far = shadows.splits[cascade];
        float fade = clamp((view_depth - far * 0.85) / max(far * 0.15, 1e-3), 0.0, 1.0);
        visibility = mix(visibility, 1.0, fade);
    }
    return visibility;
}

#endif // PARTICLE_SHADOW_GLSL
