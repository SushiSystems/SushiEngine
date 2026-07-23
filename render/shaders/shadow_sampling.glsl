// Sampling the sun's shadow cascades: percentage-closer *soft* shadows.
//
// A fixed-radius filter is wrong in the same way for every pixel: it blurs a contact
// that should be crisp and leaves a distant silhouette that should be diffuse looking cut
// out. Real softness comes from the sun being a disc rather than a point, so the penumbra
// at a point is the gap to whatever is blocking it times the sun's angular radius. That
// is what this measures — first find how far above the receiver the blockers stand, then
// filter over exactly that width.
//
// Fragment stages only: it samples, and it reads the fragment position to decorrelate the
// filter. The block itself is in shadow_common.glsl, which every stage can include.
// Requires temporal_common.glsl to have been included first: the filter's per-pixel
// rotation advances with the frame jitter so the temporal resolve can average it.

#include "shadow_common.glsl"

// A Vogel disc: tap i of count, spread by the golden angle so every prefix of the
// sequence already tiles the disc evenly. That even coverage is what makes a low tap
// count read smooth instead of grainy — the fixed Poisson set it replaces printed a
// visible speckle whenever a tier dropped the tap count, and it needs no lookup table.
// `rotation` (radians) turns the whole disc per pixel: on meshes it advances each frame
// so the temporal resolve averages the residual, and on the analytic ground it is a
// stable screen-space value so the pattern holds still instead of shimmering unresolved.
vec2 vogel_disc(int i, int count, float rotation)
{
    float radius = sqrt((float(i) + 0.5) / float(count));
    float theta = float(i) * 2.399963229728653 + rotation; // golden angle
    return vec2(radius * cos(theta), radius * sin(theta));
}

// Which cascade covers a point this far down the view axis. Linear search over at most
// four entries, which is cheaper than any cleverness at this length and, unlike a
// depth-derived index, stays correct when the splits are retuned at runtime.
int select_shadow_cascade(float view_depth)
{
    int count = int(shadows.params.x);
    for (int i = 0; i < count - 1; ++i)
    {
        if (view_depth < shadows.splits[i])
            return i;
    }
    return count - 1;
}

// The atlas is a two-by-two grid of tiles, so a cascade's tile is its index read as two
// bits. One image means one descriptor and one pass.
vec2 shadow_tile_origin(int cascade)
{
    return vec2(float(cascade & 1), float(cascade >> 1)) * shadows.params.y;
}

// Keeps a filter tap inside its own tile. A tap reaching past the edge would read a
// completely unrelated cascade's depth, which is the one way a two-by-two atlas can go
// wrong that four separate images cannot.
vec2 shadow_tile_clamp(vec2 tile_uv, vec2 texel)
{
    float scale = shadows.params.y;
    return clamp(tile_uv, texel * 0.5, vec2(scale) - texel * 0.5);
}

// How far above the receiver the things blocking it stand, averaged, in stored-depth
// units. Returns a negative value when nothing blocks — the caller then skips the filter
// entirely, which is what makes fully lit ground cost eight taps instead of twenty-four.
//
// This reads the atlas through a plain sampler rather than the comparison one. A
// comparison sampler can only report whether a tap passed, never what it stored, and
// recovering a depth from a sweep of comparisons costs an order of magnitude more taps
// than binding the same image a second time.
float shadow_blocker_depth(sampler2D depth_atlas, int cascade, vec2 uv, float receiver,
                           vec2 texel, float search_radius, float angle, bool stable)
{
    vec2 tile = shadow_tile_origin(cascade);
    float total = 0.0;
    float count = 0.0;
    // The tap count is the tier's, read from the block rather than baked in, so a low
    // tier searches with fewer samples. The analytic ground (stable) cannot lean on the
    // temporal resolve, so it holds a smoothness floor of its own — it is a single cheap
    // fullscreen march, so the extra taps are affordable there.
    int taps = clamp(int(shadows.filter_size.w), 1, 16);
    if (stable)
        taps = max(taps, 12);
    for (int i = 0; i < taps; ++i)
    {
        vec2 offset = vogel_disc(i, taps, angle);
        vec2 tap = shadow_tile_clamp(uv * shadows.params.y + offset * search_radius * texel,
                                     texel);
        float depth = texture(depth_atlas, tile + tap).r;
        if (depth < receiver)
        {
            total += depth;
            count += 1.0;
        }
    }
    return count > 0.0 ? total / count : -1.0;
}

// One cascade's visibility, filtered over @p radius texels. The sampler compares in
// hardware, so every tap is already a bilinear two-by-two average and sixteen of them
// spread over a disc give a penumbra with no visible structure.
float shadow_filter(sampler2DShadow atlas, int cascade, vec2 uv, float reference,
                    vec2 texel, float radius, float angle, bool stable)
{
    vec2 tile = shadow_tile_origin(cascade);
    float total = 0.0;
    // Tier-driven tap count, same as the blocker search: fewer taps is a cheaper penumbra
    // the temporal resolve smooths. The Vogel spread keeps even the low counts free of
    // grain; the stable (analytic-ground) path holds a floor since it has no resolve.
    int taps = clamp(int(shadows.params.z), 1, 16);
    if (stable)
        taps = max(taps, 12);
    for (int i = 0; i < taps; ++i)
    {
        vec2 offset = vogel_disc(i, taps, angle);
        vec2 tap = shadow_tile_clamp(uv * shadows.params.y + offset * radius * texel, texel);
        total += texture(atlas, vec3(tile + tap, reference));
    }
    return total / float(taps);
}

// One cascade's soft visibility: measure the penumbra, then filter over it.
float sample_shadow_cascade(sampler2DShadow atlas, sampler2D depth_atlas, int cascade,
                            vec3 position, float slope, float angle, bool stable)
{
    vec4 light_clip = shadows.cascade_view_projection[cascade] * vec4(position, 1.0);
    vec3 light = light_clip.xyz / light_clip.w;
    vec2 uv = light.xy * 0.5 + 0.5;
    // Outside the cascade entirely: lit, not shadowed. Guessing the other way would put
    // a hard dark edge at the end of the shadow distance.
    if (any(lessThan(uv, vec2(0.0))) || any(greaterThan(uv, vec2(1.0))) || light.z > 1.0)
        return 1.0;

    // A polygon seen edge-on to the light spans many depth values within one texel, so
    // the constant bias is scaled by how steeply it leans away.
    float reference = light.z - shadows.bias.x * (1.0 + slope * 2.0);

    vec2 texel = vec2(1.0 / (shadows.flags.w * 2.0));
    float min_radius = shadows.filter_size.x;
    float max_radius = shadows.filter_size.y;

    // Search over the widest penumbra this cascade could produce, so a high blocker is
    // still found; anything narrower would clip the search before the filter.
    float blocker = shadow_blocker_depth(depth_atlas, cascade, uv, reference, texel,
                                         max_radius, angle, stable);
    if (blocker < 0.0)
        return 1.0; // nothing above this point at all

    // The penumbra: the gap between blocker and receiver, in metres, times the sun's
    // angular radius. Converting to texels is what makes one setting hold across
    // cascades whose texels differ by an order of magnitude.
    float gap = max(reference - blocker, 0.0) * shadows.depth_range[cascade];
    float penumbra = gap * shadows.filter_size.z / max(shadows.texel_size[cascade], 1e-5);
    float radius = clamp(penumbra, min_radius, max_radius);
    return shadow_filter(atlas, cascade, uv, reference, texel, radius, angle, stable);
}

// The sun's visibility at a shaded point, across the whole cascade set.
//
// The normal offset is what removes shadow acne without the peter-panning a pure depth
// bias causes: the sample is moved along the surface normal by roughly one shadow texel,
// so a texel that straddles the surface lands clear of it rather than inside it. Scaling
// it by the cascade's own texel size is what keeps one setting working across cascades
// whose texels differ by an order of magnitude.
//
// The last cascade fades out rather than ending, and neighbouring cascades cross-fade
// over a band, so neither boundary reads as a line drawn across the ground.
// @p stable selects how the per-pixel filter rotation advances. Meshes pass false: the
// rotation changes each frame so the temporal resolve averages the residual grain into a
// smooth penumbra. The analytic ground passes true: it is a no-geometry surface the
// resolve reprojects as if at infinity, so under camera translation its history is
// rejected and a frame-varying rotation would read as raw, unresolved speckle — a
// stable screen-space rotation (paired with the Vogel smoothness floor above) gives it a
// steady soft penumbra instead.
float sample_sun_shadow(sampler2DShadow atlas, sampler2D depth_atlas, vec3 position,
                        vec3 normal, vec3 light_dir, float view_depth, bool stable)
{
    if (shadows.flags.x < 0.5)
        return 1.0;

    // The Vogel disc turns the tap set into an even, low-discrepancy spread; the rotation
    // decorrelates it per pixel. Frame-varying for the temporal resolve to average, or a
    // frame-static hash where there is no resolve to average it (the ground).
    float angle = (stable ? interleaved_gradient_noise(gl_FragCoord.xy)
                          : temporal_dither(gl_FragCoord.xy)) *
                  6.28318530718;

    int cascade = select_shadow_cascade(view_depth);
    float n_dot_l = dot(normal, light_dir);
    float slope = clamp(1.0 - abs(n_dot_l), 0.0, 1.0);
    vec3 offset_position = position + normal * shadows.texel_size[cascade] * shadows.bias.y;
    float visibility =
        sample_shadow_cascade(atlas, depth_atlas, cascade, offset_position, slope, angle, stable);

    int count = int(shadows.params.x);
    float far = shadows.splits[cascade];
    float band = far * shadows.params.w;
    if (cascade + 1 < count && band > 0.0 && view_depth > far - band)
    {
        vec3 next_position =
            position + normal * shadows.texel_size[cascade + 1] * shadows.bias.y;
        float next = sample_shadow_cascade(atlas, depth_atlas, cascade + 1, next_position,
                                           slope, angle, stable);
        visibility = mix(visibility, next, clamp((view_depth - (far - band)) / band, 0.0, 1.0));
    }
    else if (cascade + 1 == count)
    {
        // Past the last cascade there is no information, so the shadow is faded out
        // rather than cut off.
        float fade = clamp((view_depth - far * 0.85) / max(far * 0.15, 1e-3), 0.0, 1.0);
        visibility = mix(visibility, 1.0, fade);
    }

    // Terminator fade. As the surface turns edge-on to the sun (n_dot_l -> 0) the direct
    // term this visibility multiplies vanishes anyway, but the stochastic PCSS is at its
    // noisiest exactly here: the slope-scaled bias is largest and the per-pixel blocker
    // search flips on and off across the grazing band, printing a dither the ground has no
    // temporal resolve to average out. Fading visibility back to lit over that band erases
    // the one place the speckle is visible without darkening anything that was actually lit.
    float terminator = 1.0 - smoothstep(0.0, 0.12, max(n_dot_l, 0.0));
    visibility = mix(visibility, 1.0, terminator);
    return visibility;
}
