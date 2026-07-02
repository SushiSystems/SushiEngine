#version 450

// Lit surface shading for scene meshes: a single hard-coded directional light with
// an ambient floor, so a cube reads as a solid three-dimensional form in the
// viewport. Colour arrives per instance from the vertex stage. The shader also writes
// the draw's picking id to a second (integer) target and lifts the selected entity
// toward a warm highlight so the current selection stands out.

layout(push_constant) uniform Push
{
    mat4 mvp;
    vec4 n0;
    vec4 n1;
    vec4 n2;
    uint entity_id;
    uint selected;
} pc;

layout(location = 0) in vec3 v_normal;
layout(location = 1) in vec3 v_color;

layout(location = 0) out vec4 out_color;
layout(location = 1) out uint out_id;

void main()
{
    vec3 light_direction = normalize(vec3(0.4, 0.8, 0.5));
    float diffuse = max(dot(normalize(v_normal), light_direction), 0.0);
    vec3 shaded = v_color * (0.25 + 0.75 * diffuse);

    if (pc.selected != 0u && pc.entity_id == pc.selected)
        shaded = mix(shaded, vec3(1.0, 0.7, 0.2), 0.5) + vec3(0.15);

    out_color = vec4(shaded, 1.0);
    out_id = pc.entity_id;
}
