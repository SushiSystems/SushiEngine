#version 450

// A single triangle generated from gl_VertexIndex — no vertex buffers. Positions are
// in Vulkan clip space (y points down); the per-vertex colours are interpolated
// across the face so the offscreen readback can tell the triangle from the clear.

layout(location = 0) out vec3 frag_color;

vec2 positions[3] = vec2[](
    vec2( 0.0, -0.5),
    vec2( 0.5,  0.5),
    vec2(-0.5,  0.5));

vec3 colors[3] = vec3[](
    vec3(1.0, 0.0, 0.0),
    vec3(0.0, 1.0, 0.0),
    vec3(0.0, 0.0, 1.0));

void main()
{
    gl_Position = vec4(positions[gl_VertexIndex], 0.0, 1.0);
    frag_color = colors[gl_VertexIndex];
}
