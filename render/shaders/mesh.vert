#version 450

// Shared vertex shader for the scene view's lit meshes and flat grid lines. The
// push constant carries the full model-view-projection plus the model's 3x3 normal
// basis smuggled into three vec4 columns, whose w channels carry the instance
// colour — this keeps the whole per-draw payload at 112 bytes, within the 128-byte
// push-constant floor every Vulkan device guarantees.

layout(location = 0) in vec3 in_position;
layout(location = 1) in vec3 in_normal;

layout(push_constant) uniform Push
{
    mat4 mvp;
    vec4 n0; // xyz = normal-basis column 0, w = colour.r
    vec4 n1; // xyz = normal-basis column 1, w = colour.g
    vec4 n2; // xyz = normal-basis column 2, w = colour.b
} pc;

layout(location = 0) out vec3 v_normal;
layout(location = 1) out vec3 v_color;

void main()
{
    gl_Position = pc.mvp * vec4(in_position, 1.0);
    mat3 normal_basis = mat3(pc.n0.xyz, pc.n1.xyz, pc.n2.xyz);
    v_normal = normal_basis * in_normal;
    v_color = vec3(pc.n0.w, pc.n1.w, pc.n2.w);
}
