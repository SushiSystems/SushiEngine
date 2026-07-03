#version 450

// Outline rendering for selected objects: renders edges in orange, wireframe mode.

layout(location = 0) in vec3 v_normal;
layout(location = 1) in vec3 v_color;

layout(push_constant) uniform Push
{
    mat4 mvp;
    vec4 n0;
    vec4 n1;
    vec4 n2;
    uint entity_id;
    uint selected;
} pc;

layout(location = 0) out vec4 out_color;
layout(location = 1) out uint out_id;

void main()
{
    out_color = vec4(1.0, 0.65, 0.2, 1.0);
    out_id = pc.entity_id;
}
