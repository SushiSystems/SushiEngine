#version 450
#extension GL_GOOGLE_include_directive : require

// Sorted particle billboard vertex shader (VFX2a): identical to particle.vert, but the alpha
// bucket's draw indexes the alpha list through the back-to-front sort keys the sort pass produced,
// so `gl_InstanceIndex` walks particles far-to-near. Shares the draw pass's push constant and the
// billboard expansion; only the particle fetch differs.

#include "particle_common.glsl"

layout(std430, set = 0, binding = 0) readonly buffer DrawList { Particle draw[]; };

struct SortEntry
{
    float key;
    uint index;
};

layout(std430, set = 0, binding = 2) readonly buffer SortKeys { SortEntry keys[]; };

layout(push_constant) uniform Push
{
    mat4 view_projection;
    vec4 camera_right;
    vec4 camera_up;
    vec4 sun_direction;
    vec4 sun_radiance;
} pc;

layout(location = 0) out vec2 out_uv;
layout(location = 1) out vec4 out_color;

const vec2 CORNERS[6] = vec2[](vec2(-1.0, -1.0), vec2(1.0, -1.0), vec2(1.0, 1.0),
                               vec2(-1.0, -1.0), vec2(1.0, 1.0), vec2(-1.0, 1.0));

void main()
{
    Particle p = draw[keys[gl_InstanceIndex].index];
    vec2 corner = CORNERS[gl_VertexIndex];
    out_uv = corner * 0.5 + 0.5;

    float c = cos(p.rotation);
    float s = sin(p.rotation);
    vec2 rotated = vec2(corner.x * c - corner.y * s, corner.x * s + corner.y * c);

    vec3 center = vec3(p.px, p.py, p.pz);
    vec3 world = center + (pc.camera_right.xyz * rotated.x + pc.camera_up.xyz * rotated.y) * p.size;

    out_color = vec4(p.cr, p.cg, p.cb, p.alpha);
    gl_Position = pc.view_projection * vec4(world, 1.0);
}
