#version 450
#extension GL_GOOGLE_include_directive : require

// Particle billboard vertex shader (design §5.4): vertex-less draw. The pipeline fetches no
// vertices; the draw is 6 vertices per instance, one instance per alive particle, and this
// shader expands each into a quad by pulling the particle from the compacted draw list
// (gl_InstanceIndex) and placing the corner (gl_VertexIndex) through the shared
// particle_quad_offset — camera-facing, or stretched along the velocity, as the particle's
// emitter authored it. The emitter table is what carries that alignment, so this shader serves
// the GPU-simulated buckets only; host-uploaded deterministic billboards, which belong to no GPU
// emitter, go through particle_billboard.vert.

#include "particle_common.glsl"

layout(std430, set = 0, binding = 0) readonly buffer DrawList { Particle draw[]; };
layout(std430, set = 0, binding = 12) readonly buffer Emitters { Emitter emitters[]; };

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
// Camera-relative shading state for clustered lighting: xyz = particle centre minus the eye
// (the space the froxel light list lives in), w = positive view-space depth (gl_Position.w,
// i.e. the perspective clip w, which is exactly the distance the cluster z-slice is keyed on).
// Taken from the particle centre, not the billboard corner, so it is constant across the quad.
// The eye subtraction is in float, so at planetary distances it inherits the pool's own
// float32 position precision — fine for the near-camera cosmetic particles this path serves.
layout(location = 2) out vec4 out_light;
// The sprite atlas coordinate, kept apart from out_uv because the two mean different things: this
// one is where the texture is sampled (a flipbook cell, so not the whole [0,1] square), while
// out_uv stays the local quad coordinate the round-dot falloff and the hemisphere normal need.
layout(location = 3) out vec2 out_atlas_uv;
// The emitter's material: x = bindless texture slot, y = Vfx::RenderFlags. Flat because it is
// per-emitter, and interpolating a bitfield or a descriptor index is meaningless.
layout(location = 4) flat out uvec2 out_material;
layout(location = 5) flat out float out_soft_fade;

const vec2 CORNERS[6] = vec2[](vec2(-1.0, -1.0), vec2(1.0, -1.0), vec2(1.0, 1.0),
                               vec2(-1.0, -1.0), vec2(1.0, 1.0), vec2(-1.0, 1.0));

void main()
{
    Particle p = draw[gl_InstanceIndex];
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
