#version 450

// Flat, unlit shading for the ground grid: the grid should read as reference lines,
// not a lit surface, so it emits the per-draw albedo directly. Shares the vertex
// shader and pipeline layout with the mesh pass; the grid is not pickable.

layout(push_constant) uniform Push
{
    mat4 model;
    vec4 albedo_metallic;
    vec4 emissive_roughness;
    vec4 outline_shift;
    uint entity_id;
    uint selected;
} pc;

layout(location = 0) in vec3 v_world_position;
layout(location = 1) in vec3 v_world_normal;

layout(location = 0) out vec4 out_color;
layout(location = 1) out uint out_id;

void main()
{
    out_color = vec4(pc.albedo_metallic.xyz, 1.0);
    out_id = 0u;
}
