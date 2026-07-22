#version 450
#extension GL_GOOGLE_include_directive : require

// Blurs the analytic ground's raw per-pixel sun-visibility term (sky.frag's planet ray
// march). Every other stochastic term in the engine gets one of two treatments: mesh
// shadows and SSR lean on TAA (motion vectors + history), GTAO leans on a depth-weighted
// bilateral resolve. The ground has neither — it is ray-marched, not rasterised, so
// there is no motion vector to reproject and no real depth in the depth buffer at these
// pixels to weight a bilateral kernel against. A wide plain blur is what is left: it
// cannot respect a silhouette it cannot see, so the ground's horizon softens by a few
// pixels, which reads as far cheaper than the raw 12-tap PCF speckle it replaces.
//
// rgb (the direct-sun radiance term) passes through unblurred — it is already smooth,
// only the alpha (raw shadow visibility) is noisy — but is sampled through the same
// bilinear taps as the alpha for one texture fetch per tap rather than two.

layout(set = 0, binding = 1) uniform sampler2D ground_shadow_texture;

layout(location = 0) in vec2 v_ndc;

layout(location = 0) out vec4 out_ground_shadow;

void main()
{
    vec2 uv = v_ndc * 0.5 + 0.5;
    vec2 texel = 1.0 / vec2(textureSize(ground_shadow_texture, 0));

    // A 7x7 Gaussian-weighted footprint: wide enough to dissolve the raw filter's 12-tap
    // grain, small enough to stay a local operation rather than a full-screen fog.
    const int RADIUS = 3;
    const float SIGMA = 1.6;
    vec4 sum = vec4(0.0);
    float wsum = 0.0;
    for (int y = -RADIUS; y <= RADIUS; ++y)
    {
        for (int x = -RADIUS; x <= RADIUS; ++x)
        {
            float w = exp(-float(x * x + y * y) / (2.0 * SIGMA * SIGMA));
            sum += texture(ground_shadow_texture, uv + vec2(x, y) * texel) * w;
            wsum += w;
        }
    }
    out_ground_shadow = wsum > 0.0 ? sum / wsum : texture(ground_shadow_texture, uv);
}
