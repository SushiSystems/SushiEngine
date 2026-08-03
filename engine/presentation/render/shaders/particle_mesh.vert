#version 450
#extension GL_GOOGLE_include_directive : require

// Mesh-particle vertex shader (design §7.11, VFX4): one mesh instance per particle. Unlike the
// sprite and ribbon paths this one does fetch vertices — it is an indexed indirect draw over a real
// mesh — and takes only the placement from the particle: position, uniform scale from its size, and
// a tumble about its direction of travel.
//
// The instance's particle comes from its emitter's slice of the shared mesh list; the slice's base
// and length are pushed, because one draw binds one mesh and so there is one draw per slice anyway.
// The length also clamps the index: the instance count is written by a GPU atomic that the host has
// no way to cap, so an overflowing instance redraws the slice's last particle instead of reading
// into the next emitter's slice.

#include "particle_common.glsl"

layout(std430, set = 0, binding = 0) readonly buffer MeshList { Particle particles[]; };

layout(location = 0) in vec3 in_position;
layout(location = 1) in vec3 in_normal;
layout(location = 2) in vec4 in_tangent;
layout(location = 3) in vec2 in_uv0;
layout(location = 4) in vec2 in_uv1;
layout(location = 5) in vec4 in_color;

layout(push_constant) uniform Push
{
    mat4 view_projection;
    vec4 sun_direction; // xyz to-sun;                w = eye.x
    vec4 sun_radiance;  // rgb sun colour * intensity; w = eye.y
    vec4 ambient;       // rgb flat ambient;           w = eye.z
    uvec4 slice;        // x = first particle, y = particles in the slice
} pc;

layout(location = 0) out vec3 out_normal;
layout(location = 1) out vec4 out_color;
// Camera-relative position (xyz) + positive view depth (w), as the sprite paths carry it.
layout(location = 2) out vec4 out_light;

// A rotation of @p angle radians about @p axis, applied to @p v (Rodrigues). Cheaper than building
// the matrix for the two vectors this shader turns.
vec3 rotate_about(vec3 v, vec3 axis, float angle)
{
    float c = cos(angle);
    float s = sin(angle);
    return v * c + cross(axis, v) * s + axis * dot(axis, v) * (1.0 - c);
}

void main()
{
    uint index = pc.slice.x + min(uint(gl_InstanceIndex), max(pc.slice.y, 1u) - 1u);
    Particle p = particles[index];

    // Debris tumbles about the way it is going, which is what the particle's own roll already
    // integrates; a particle with no velocity spins about world up instead of collapsing.
    vec3 velocity = vec3(p.vx, p.vy, p.vz);
    vec3 axis = dot(velocity, velocity) > 1e-8 ? normalize(velocity) : vec3(0.0, 1.0, 0.0);

    vec3 local = rotate_about(in_position * p.size, axis, p.rotation);
    vec3 world = vec3(p.px, p.py, p.pz) + local;
    vec3 eye = vec3(pc.sun_direction.w, pc.sun_radiance.w, pc.ambient.w);

    out_normal = rotate_about(in_normal, axis, p.rotation);
    // The particle tints the mesh's own vertex colour, so one authored mesh serves every emitter
    // that wants it in a different colour.
    out_color = vec4(in_color.rgb * vec3(p.cr, p.cg, p.cb), p.alpha);
    gl_Position = pc.view_projection * vec4(world, 1.0);
    out_light = vec4(world - eye, gl_Position.w);
}
