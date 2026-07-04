#version 450

// Outline rendering for the selected object: a solid warm colour, drawn in wireframe
// and masked by stencil so only the silhouette survives. Emits the entity id so a
// click on the outline still picks the object.

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
    out_color = vec4(1.0, 0.65, 0.2, 1.0);
    out_id = pc.entity_id;
}
