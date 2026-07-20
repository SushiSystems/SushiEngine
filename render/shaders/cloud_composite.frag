#version 450

// Resolves the half-resolution cloud march over the full-resolution sky, producing the
// linear HDR image the rest of the frame works on.
//
// This used to be two lines at the top of the tonemap pass. It is its own pass now
// because the temporal resolve has to run on a complete scene — clouds included — and
// the display transform has to run after that, so nothing may sit on both sides of it.
// Splitting it also gives the post-processing stack one obvious place to attach.

layout(set = 0, binding = 1) uniform sampler2D sky_texture;
// rgb = premultiplied in-scattered cloud light, a = transmittance along the view ray.
layout(set = 0, binding = 2) uniform sampler2D cloud_texture;

layout(location = 0) in vec2 v_ndc;

layout(location = 0) out vec4 out_color;

void main()
{
    vec2 uv = v_ndc * 0.5 + 0.5;
    vec3 sky = texture(sky_texture, uv).rgb;
    // The cloud target is half-resolution; the linear sampler is the upsample.
    vec4 cloud = texture(cloud_texture, uv);
    out_color = vec4(sky * cloud.a + cloud.rgb, 1.0);
}
