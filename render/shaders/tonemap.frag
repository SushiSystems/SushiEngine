#version 450

// The display transform, and nothing else: exposure, the ACES filmic tone curve, and
// the gamma encode onto the plain UNORM target the editor samples with ImGui.
//
// It reads whatever the frame resolved to — the temporal resolve's output when
// temporal anti-aliasing is on, the composited scene directly when it is not — so
// this shader never learns which anti-aliasing mode the frame ran.

layout(set = 0, binding = 0) uniform SceneBlock
{
    mat4 view;
    mat4 proj;
    vec4 cam_forward;
    vec4 cam_right;
    vec4 cam_up;
    vec4 planet_center;
    vec4 planet_radii;
    vec4 sun_dir;
    vec4 sun_color;   // w = exposure
    vec4 ambient;
    vec4 rayleigh;
    vec4 scatter;
    vec4 ground_albedo;
    vec4 ocean_color;
    vec4 cloud_params;
    vec4 star_params;
    vec4 misc;
} scene;

layout(set = 0, binding = 1) uniform sampler2D hdr_texture;

layout(location = 0) in vec2 v_ndc;

layout(location = 0) out vec4 out_color;

// Narkowicz 2015 ACES filmic approximation.
vec3 aces(vec3 x)
{
    const float a = 2.51;
    const float b = 0.03;
    const float c = 2.43;
    const float d = 0.59;
    const float e = 0.14;
    return clamp((x * (a * x + b)) / (x * (c * x + d) + e), 0.0, 1.0);
}

void main()
{
    vec2 uv = v_ndc * 0.5 + 0.5;
    vec3 hdr = texture(hdr_texture, uv).rgb;
    vec3 mapped = aces(hdr * scene.sun_color.w);
    mapped = pow(mapped, vec3(1.0 / 2.2));
    out_color = vec4(mapped, 1.0);
}
