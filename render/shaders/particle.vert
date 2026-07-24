#version 450
#extension GL_GOOGLE_include_directive : require

// Particle billboard vertex shader (design §5.4): vertex-less draw. The pipeline fetches no
// vertices; the draw is 6 vertices per instance, one instance per alive particle, and this
// shader expands each into a camera-facing quad by pulling the particle from the compacted draw
// list (gl_InstanceIndex) and placing the corner (gl_VertexIndex) along the camera's world-space
// right/up axes, scaled by the particle's size and spun by its roll.

#include "particle_common.glsl"

layout(std430, set = 0, binding = 0) readonly buffer DrawList { Particle draw[]; };

layout(push_constant) uniform Push
{
    mat4 view_projection; // camera view * projection (float)
    vec4 camera_right;    // xyz world-space camera right
    vec4 camera_up;       // xyz world-space camera up
    vec4 sun_direction;   // xyz direction to the sun; w = ambient
    vec4 sun_radiance;    // rgb sun colour * intensity; w = lit flag
} pc;

layout(location = 0) out vec2 out_uv;
layout(location = 1) out vec4 out_color;

const vec2 CORNERS[6] = vec2[](vec2(-1.0, -1.0), vec2(1.0, -1.0), vec2(1.0, 1.0),
                               vec2(-1.0, -1.0), vec2(1.0, 1.0), vec2(-1.0, 1.0));

void main()
{
    Particle p = draw[gl_InstanceIndex];
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
