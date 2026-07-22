#version 450
#extension GL_GOOGLE_include_directive : require

// Flat, unlit shading for the ground grid: the grid should read as reference lines,
// not a lit surface, so it emits the per-draw albedo directly. Shares the vertex
// shader and pipeline layout with the mesh pass; the grid is not pickable.

#include "temporal_common.glsl"

layout(push_constant) uniform Push
{
    mat4 model;
    vec4 albedo_metallic;
    vec4 emissive_roughness;
    vec4 outline_shift;
    uint entity_id;
    uint selected;
    uint material_index;
    uint motion_index;
} pc;

layout(location = 0) in vec3 v_world_position;
layout(location = 1) in vec3 v_world_normal;
layout(location = 6) in vec4 v_current_clip;
layout(location = 7) in vec4 v_previous_clip;

layout(location = 0) out vec4 out_color;
layout(location = 1) out uint out_id;
layout(location = 2) out vec2 out_velocity;
layout(location = 3) out vec2 out_gbuffer;

void main()
{
    out_color = vec4(pc.albedo_metallic.xyz, 1.0);
    out_id = 0u;
    out_gbuffer = vec2(1.0, 0.04); // fully rough: the grid never reflects
    // The grid is static, so this is the camera's own motion — which is exactly what
    // the temporal resolve needs to keep the lines from smearing as the camera turns.
    out_velocity = motion_vector(v_current_clip, v_previous_clip);
}
