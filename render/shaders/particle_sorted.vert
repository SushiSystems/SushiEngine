#version 450
#extension GL_GOOGLE_include_directive : require

// Sorted particle billboard vertex shader (VFX2a): identical to particle.vert, but the alpha
// bucket's draw indexes the alpha list through the back-to-front sort keys the sort pass produced,
// so `gl_InstanceIndex` walks particles far-to-near. Shares the draw pass's push constant and the
// billboard expansion; only the particle fetch differs.

#include "particle_common.glsl"

layout(std430, set = 0, binding = 0) readonly buffer DrawList { Particle draw[]; };
layout(std430, set = 0, binding = 12) readonly buffer Emitters { Emitter emitters[]; };

struct SortEntry
{
    float key;
    uint index;
};

layout(std430, set = 0, binding = 2) readonly buffer SortKeys { SortEntry keys[]; };

layout(push_constant) uniform Push
{
    mat4 view_projection;
    vec4 camera_right;   // xyz world camera right; w = eye.x
    vec4 camera_up;      // xyz world camera up;    w = eye.y
    vec4 sun_direction;  // xyz direction to the sun; w = eye.z
    vec4 sun_radiance;   // rgb sun colour * intensity; w = lit/ambient flag
} pc;

layout(location = 0) out vec2 out_uv;
layout(location = 1) out vec4 out_color;
// See particle.vert: camera-relative centre (xyz) + positive view depth (w) for clustered lighting.
layout(location = 2) out vec4 out_light;
// See particle.vert: the sprite atlas coordinate and the emitter's material.
layout(location = 3) out vec2 out_atlas_uv;
layout(location = 4) flat out uvec2 out_material;
layout(location = 5) flat out float out_soft_fade;

const vec2 CORNERS[6] = vec2[](vec2(-1.0, -1.0), vec2(1.0, -1.0), vec2(1.0, 1.0),
                               vec2(-1.0, -1.0), vec2(1.0, 1.0), vec2(-1.0, 1.0));

void main()
{
    Particle p = draw[keys[gl_InstanceIndex].index];
    Emitter e = emitters[p.emitter_index];
    vec2 corner = CORNERS[gl_VertexIndex];
    out_uv = corner * 0.5 + 0.5;
    out_atlas_uv =
        particle_sprite_uv(corner, p.flipbook_frame, e.flipbook_rows, e.flipbook_columns);
    particle_material(e, out_material.x, out_material.y, out_soft_fade);

    vec3 center = vec3(p.px, p.py, p.pz);
    vec3 world = center + particle_quad_offset(p, corner, pc.camera_right.xyz, pc.camera_up.xyz,
                                               e.alignment, e.velocity_stretch);

    out_color = vec4(p.cr, p.cg, p.cb, p.alpha);
    gl_Position = pc.view_projection * vec4(world, 1.0);
    vec3 eye = vec3(pc.camera_right.w, pc.camera_up.w, pc.sun_direction.w);
    out_light = vec4(center - eye, gl_Position.w);
}
