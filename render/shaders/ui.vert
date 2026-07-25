#version 450

// UI overlay vertex shader: screen pixels to clip space, nothing else.
//
// The geometry arrives already laid out — the host resolved anchors, pivots and parent
// chains into absolute pixel rectangles — so this stage only rescales. The UI frame is
// top-left origin with y downward, which is Vulkan's clip-space y direction too, so the
// mapping is a scale and a bias with no flip.

layout(location = 0) in vec2 in_position; // screen pixels, top-left origin
layout(location = 1) in vec2 in_uv;       // glyph atlas coordinate
layout(location = 2) in vec4 in_color;    // premultiplied, normalized from RGBA8

layout(push_constant) uniform Push
{
    vec4 screen; // xy = viewport size in pixels; zw spare
    uvec4 atlas; // x = bindless slot of the glyph atlas; yzw spare
} pc;

layout(location = 0) out vec2 out_uv;
layout(location = 1) out vec4 out_color;

void main()
{
    vec2 normalized = in_position / max(pc.screen.xy, vec2(1.0));
    gl_Position = vec4(normalized * 2.0 - 1.0, 0.0, 1.0);
    out_uv = in_uv;
    out_color = in_color;
}
