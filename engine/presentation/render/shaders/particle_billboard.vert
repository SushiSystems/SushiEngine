#version 450
#extension GL_GOOGLE_include_directive : require

// Deterministic-billboard vertex shader (Bağla): the CPU-simulated particles the sim extracts as
// finished world-space billboards. Identical to particle.vert except that it binds no emitter
// table and always faces the camera — these particles belong to a host-side deterministic pool,
// not to a GPU emitter, so there is no authored alignment to read and index 0 of the emitter
// table (when one exists at all) is an unrelated cosmetic emitter. A shader of its own rather
// than a flag on particle.vert: the two differ in what they may *read*, which is a pipeline
// property, and the quad expansion they share lives in particle_common.glsl either way.

#include "particle_common.glsl"

layout(std430, set = 0, binding = 0) readonly buffer DrawList { Particle draw[]; };

layout(push_constant) uniform Push
{
    mat4 view_projection; // camera view * projection (float)
    vec4 camera_right;    // xyz world-space camera right; w = eye.x
    vec4 camera_up;       // xyz world-space camera up;    w = eye.y
    vec4 sun_direction;   // xyz direction to the sun;     w = eye.z
    vec4 sun_radiance;    // rgb sun colour * intensity; w = lit/ambient flag
} pc;

layout(location = 0) out vec2 out_uv;
layout(location = 1) out vec4 out_color;
// See particle.vert: camera-relative centre (xyz) + positive view depth (w) for clustered lighting.
layout(location = 2) out vec4 out_light;
// See particle.vert. With no emitter there is no material either: zeroed flags say untextured,
// unlit and not soft, so the atlas coordinate is never sampled and the round dot stands.
layout(location = 3) out vec2 out_atlas_uv;
layout(location = 4) flat out uvec2 out_material;
layout(location = 5) flat out float out_soft_fade;

const vec2 CORNERS[6] = vec2[](vec2(-1.0, -1.0), vec2(1.0, -1.0), vec2(1.0, 1.0),
                               vec2(-1.0, -1.0), vec2(1.0, 1.0), vec2(-1.0, 1.0));

void main()
{
    Particle p = draw[gl_InstanceIndex];
    vec2 corner = CORNERS[gl_VertexIndex];
    out_uv = corner * 0.5 + 0.5;
    out_atlas_uv = out_uv;
    out_material = uvec2(0u, 0u);
    out_soft_fade = 0.0;

    vec3 center = vec3(p.px, p.py, p.pz);
    vec3 world = center + particle_quad_offset(p, corner, pc.camera_right.xyz, pc.camera_up.xyz,
                                               ALIGN_FACE_CAMERA, 0.0);

    out_color = vec4(p.cr, p.cg, p.cb, p.alpha);
    gl_Position = pc.view_projection * vec4(world, 1.0);
    vec3 eye = vec3(pc.camera_right.w, pc.camera_up.w, pc.sun_direction.w);
    out_light = vec4(center - eye, gl_Position.w);
}
