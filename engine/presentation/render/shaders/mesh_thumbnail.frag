#version 450
#extension GL_EXT_nonuniform_qualifier : require

// The isolated thumbnail renderer's own bindless heap, at the same set/binding this codebase's
// main bindless heap already uses for its texture array (DescriptorHeap::TEXTURE_BINDING == 0;
// set 1 mirrors pbr.frag's convention of reserving set 0 for the pipeline's own per-draw data).
layout(set = 1, binding = 0) uniform sampler2D bindless_textures[];

layout(location = 0) in vec3 in_world_position;
layout(location = 1) in vec3 in_world_normal;
layout(location = 2) in vec2 in_uv0;

layout(location = 0) out vec4 out_color;

layout(push_constant) uniform Push
{
    mat4 view_projection;
    vec4 albedo;
    int albedo_texture_index;
} pc;

void main()
{
    vec4 base_color = pc.albedo;
    if (pc.albedo_texture_index >= 0)
        base_color *= texture(bindless_textures[nonuniformEXT(pc.albedo_texture_index)], in_uv0);

    // One fixed headlight from roughly the camera's own viewing direction, plus a flat ambient
    // floor -- deliberately not physically based (no shadows, no IBL, no atmosphere): this is
    // the "simple/unlit-leaning" fidelity level the approved design chose for model thumbnails,
    // over the far heavier machinery pbr.frag would otherwise require.
    const vec3 light_direction = normalize(vec3(1.0, 0.8, 1.0));
    const float ambient = 0.35;
    vec3 normal = normalize(in_world_normal);
    float headlight = max(dot(normal, light_direction), 0.0);
    float shade = clamp(ambient + headlight * (1.0 - ambient), 0.0, 1.0);

    out_color = vec4(base_color.rgb * shade, base_color.a);
}
