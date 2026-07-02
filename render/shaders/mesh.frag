#version 450

// Lit surface shading for scene meshes: a single hard-coded directional light with
// an ambient floor, so a cube reads as a solid three-dimensional form in the
// viewport. Colour arrives per instance from the vertex stage.

layout(location = 0) in vec3 v_normal;
layout(location = 1) in vec3 v_color;

layout(location = 0) out vec4 out_color;

void main()
{
    vec3 light_direction = normalize(vec3(0.4, 0.8, 0.5));
    float diffuse = max(dot(normalize(v_normal), light_direction), 0.0);
    vec3 shaded = v_color * (0.25 + 0.75 * diffuse);
    out_color = vec4(shaded, 1.0);
}
