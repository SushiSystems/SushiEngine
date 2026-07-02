#version 450

// Flat, unlit shading for the ground grid: the grid should read as reference lines,
// not a lit surface, so it ignores the normal and emits the interpolated colour
// directly. Shares the vertex shader and push-constant layout with the mesh pass.

layout(location = 0) in vec3 v_normal;
layout(location = 1) in vec3 v_color;

layout(location = 0) out vec4 out_color;

void main()
{
    out_color = vec4(v_color, 1.0);
}
