#version 450

// Particle billboard fragment shader (design §5.4): a soft round sprite that is occluded by the
// scene. It samples the scene depth (never bound as an attachment, only read) and discards where
// the billboard lies behind opaque geometry — the sky/cloud pattern — so particles do not shine
// through walls. Reverse-Z: a larger depth is nearer, so a fragment whose depth is below the
// stored scene depth is behind that surface. The linear soft-depth fade near surfaces is a VFX2
// refinement; this slice does the hard occlusion plus a radial sprite falloff.

layout(set = 0, binding = 1) uniform sampler2D scene_depth;

layout(push_constant) uniform Push
{
    mat4 view_projection;
    vec4 camera_right;   // xyz world camera right
    vec4 camera_up;      // xyz world camera up
    vec4 sun_direction;  // xyz to-sun; w ambient
    vec4 sun_radiance;   // rgb sun colour * intensity; w lit flag
} pc;

layout(location = 0) in vec2 in_uv;
layout(location = 1) in vec4 in_color;

layout(location = 0) out vec4 out_color;

void main()
{
    ivec2 pixel = ivec2(gl_FragCoord.xy);
    float stored_depth = texelFetch(scene_depth, pixel, 0).r;
    if (gl_FragCoord.z < stored_depth)
        discard; // behind opaque geometry (reverse-Z: smaller depth is farther)

    vec2 offset = in_uv * 2.0 - 1.0;
    float radial = max(0.0, 1.0 - dot(offset, offset));
    float alpha = in_color.a * radial * radial;
    if (alpha <= 0.001)
        discard;

    vec3 rgb = in_color.rgb;
    // Lit particles (the true-alpha bucket, e.g. smoke) receive the sun plus ambient. The
    // sprite is shaded as a camera-facing hemisphere: the offset gives a spherical normal, so
    // the sun rakes across the puff. The sun direction is world-space, so no camera-relative
    // conversion is needed (unlike the clustered froxel lights, deferred to a later slice).
    if (pc.sun_radiance.w > 0.5)
    {
        vec3 forward = normalize(cross(pc.camera_right.xyz, pc.camera_up.xyz));
        vec3 normal = normalize(pc.camera_right.xyz * offset.x + pc.camera_up.xyz * offset.y +
                                forward * sqrt(max(0.0, 1.0 - dot(offset, offset))));
        float n_dot_l = max(dot(normal, pc.sun_direction.xyz), 0.0);
        rgb = in_color.rgb * (pc.sun_radiance.rgb * n_dot_l + pc.sun_direction.w);
    }

    // Premultiplied output: additive reads (rgb*a) + dst, alpha reads (rgb*a) + dst*(1-a).
    out_color = vec4(rgb * alpha, alpha);
}
