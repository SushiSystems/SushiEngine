#version 450
#extension GL_EXT_nonuniform_qualifier : require

// UI overlay fragment shader.
//
// One sample and one multiply: the atlas holds white rgb with coverage in alpha, so a
// glyph modulates the vertex colour by its coverage and a solid rectangle — which samples
// the atlas's reserved opaque texel — passes its colour through untouched. That is what
// lets panels and labels share a single pipeline and a single draw.
//
// The vertex colour arrives premultiplied, so the coverage multiply applies to rgb and
// alpha alike and the result stays premultiplied for the "over" blend.
//
// This runs after tone mapping and anti-aliasing, so UI colours are authored in display
// space and reach the screen unchanged — a UI that had been tonemapped would shift hue
// with the scene's exposure, which is exactly what a UI must not do.

layout(set = 1, binding = 0) uniform sampler2D bindless_textures[];

layout(push_constant) uniform Push
{
    vec4 screen;
    uvec4 atlas; // x = bindless slot of the glyph atlas
} pc;

layout(location = 0) in vec2 in_uv;
layout(location = 1) in vec4 in_color;

layout(location = 0) out vec4 out_color;

void main()
{
    float coverage = texture(bindless_textures[nonuniformEXT(pc.atlas.x)], in_uv).a;
    out_color = in_color * coverage;
    if (out_color.a <= 0.0)
        discard;
}
