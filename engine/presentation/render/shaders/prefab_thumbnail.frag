#version 450
#extension GL_EXT_nonuniform_qualifier : require

// This renderer's own isolated bindless heap -- same set/binding convention Phase 3a's
// mesh_thumbnail.frag and the main renderer's pbr.frag both already use.
layout(set = 1, binding = 0) uniform sampler2D bindless_textures[];

layout(location = 0) in vec3 in_world_normal;
layout(location = 1) in vec2 in_uv0;

layout(location = 0) out vec4 out_color;

layout(push_constant) uniform Push
{
    mat4 model;
    mat4 view_projection;
    vec4 albedo;
    int albedo_texture_index;
} pc;

void main()
{
    vec4 base_color = pc.albedo;
    if (pc.albedo_texture_index >= 0)
        base_color *= texture(bindless_textures[nonuniformEXT(pc.albedo_texture_index)], in_uv0);

    // Identical fixed headlight-plus-ambient shading to Phase 3a's mesh_thumbnail.frag -- kept
    // in a second file rather than shared, since the two shaders' Push blocks now differ (this
    // one carries a model matrix, Phase 3a's does not) and GLSL has no cross-file struct sharing
    // this codebase's shader-compile pipeline resolves.
    const vec3 light_direction = normalize(vec3(1.0, 0.8, 1.0));
    const float ambient = 0.35;
    vec3 normal = normalize(in_world_normal);
    float headlight = max(dot(normal, light_direction), 0.0);
    float shade = clamp(ambient + headlight * (1.0 - ambient), 0.0, 1.0);

    out_color = vec4(base_color.rgb * shade, base_color.a);
}
