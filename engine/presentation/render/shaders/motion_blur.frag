#version 450

// Per-pixel velocity motion blur. The shipped velocity target holds the screen-space motion
// since the previous frame — both camera and object motion — so gathering the colour along
// that vector smears a moving surface the way a finite shutter would. The sample count and
// strength are tier- and author-controlled through the post block.

layout(set = 0, binding = 1) uniform sampler2D color_texture;
layout(set = 0, binding = 2) uniform sampler2D velocity_texture;

layout(set = 0, binding = 31) uniform PostBlock
{
    vec4 exposure;
    vec4 grade0;
    vec4 lift;
    vec4 gamma;
    vec4 gain;
    vec4 effects;
    vec4 dof;
    vec4 motion; // x = intensity, y = sample count, z = enabled
    vec4 misc;
} post;

layout(location = 0) in vec2 v_ndc;
layout(location = 0) out vec4 out_color;

void main()
{
    vec2 uv = v_ndc * 0.5 + 0.5;

    // The velocity is UV displacement per frame; scale by intensity and clamp so a very fast
    // surface cannot gather halfway across the screen.
    vec2 velocity = texture(velocity_texture, uv).rg * post.motion.x;
    float len = length(velocity);
    vec2 clamped = len > 0.1 ? velocity * (0.1 / len) : velocity;

    int samples = max(int(post.motion.y + 0.5), 1);
    vec3 sum = texture(color_texture, uv).rgb;
    float weight = 1.0;
    for (int i = 1; i < samples; ++i)
    {
        // Sample symmetrically about the pixel so the blur is centred, not trailing.
        float t = (float(i) / float(samples)) - 0.5;
        vec2 sample_uv = uv + clamped * t;
        sum += texture(color_texture, sample_uv).rgb;
        weight += 1.0;
    }

    out_color = vec4(sum / weight, 1.0);
}
