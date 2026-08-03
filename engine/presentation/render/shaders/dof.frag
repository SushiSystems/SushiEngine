#version 450

// Gather-based depth of field. Each pixel's circle of confusion comes from a thin-lens
// defocus against the focus plane; a Vogel disc of taps, scaled by that radius, is gathered
// and weighted so a sharp foreground does not bleed onto a blurred background. Bounded taps
// keep the cost fixed regardless of how far out of focus a region is.

layout(set = 0, binding = 1) uniform sampler2D color_texture;
layout(set = 0, binding = 2) uniform sampler2D depth_texture;

layout(set = 0, binding = 31) uniform PostBlock
{
    vec4 exposure;
    vec4 grade0;
    vec4 lift;
    vec4 gamma;
    vec4 gain;
    vec4 effects;
    vec4 dof;    // x = focus distance, y = focus range, z = coc scale, w = max radius (px)
    vec4 motion;
    vec4 misc;   // x = render width, y = render height, z = camera near plane
} post;

layout(location = 0) in vec2 v_ndc;
layout(location = 0) out vec4 out_color;

const int TAPS = 24;
const float GOLDEN = 2.399963f;

float view_distance(float depth, float near_plane)
{
    // Reverse-Z with an infinite far plane maps depth = near / view_z, so the eye distance is
    // recovered by the inverse. Clamped to a finite far so the sky (depth 0) reads as a large
    // but bounded distance — focusing far then keeps it sharp instead of forcing max blur.
    return min(near_plane / max(depth, 1e-6), 1.0e5);
}

float coc_radius(float distance)
{
    float focus = post.dof.x;
    float range = max(post.dof.y, 1e-3);
    float defocus = abs(distance - focus) - range;
    float amount = clamp(defocus / (range * 4.0), 0.0, 1.0);
    return amount * post.dof.w;
}

void main()
{
    vec2 uv = v_ndc * 0.5 + 0.5;
    vec2 texel = 1.0 / max(post.misc.xy, vec2(1.0));
    float near_plane = post.misc.z;

    float centre_distance = view_distance(texture(depth_texture, uv).r, near_plane);
    float centre_radius = coc_radius(centre_distance);

    vec3 sum = texture(color_texture, uv).rgb;
    float weight = 1.0;
    if (centre_radius > 0.5)
    {
        for (int i = 0; i < TAPS; ++i)
        {
            float t = (float(i) + 0.5) / float(TAPS);
            float radius = sqrt(t) * centre_radius;
            float angle = float(i) * GOLDEN;
            vec2 offset = vec2(cos(angle), sin(angle)) * radius * texel;
            vec2 sample_uv = uv + offset;

            vec3 tap_color = texture(color_texture, sample_uv).rgb;
            float tap_distance = view_distance(texture(depth_texture, sample_uv).r, near_plane);
            float tap_radius = coc_radius(tap_distance);
            // A tap contributes only if its own circle of confusion reaches this pixel, which
            // keeps a sharp foreground from smearing across a blurred background.
            float w = clamp(tap_radius - radius + 1.0, 0.0, 1.0);
            sum += tap_color * w;
            weight += w;
        }
    }

    out_color = vec4(sum / max(weight, 1e-4), 1.0);
}
