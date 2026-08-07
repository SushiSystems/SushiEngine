#version 450
#extension GL_EXT_nonuniform_qualifier : require

// This renderer's own isolated bindless heap -- same set/binding convention Phase 3a's
// mesh_thumbnail.frag and the main renderer's pbr.frag both already use.
layout(set = 1, binding = 0) uniform sampler2D bindless_textures[];

layout(location = 0) in vec3 in_object_normal;
layout(location = 1) in vec2 in_uv0;

layout(location = 0) out vec4 out_color;

layout(push_constant) uniform Push
{
    mat4 mvp;
    vec4 light_object;
    vec4 albedo;
    int albedo_texture_index;
} pc;

void main()
{
    vec4 base_color = pc.albedo;
    if (pc.albedo_texture_index >= 0)
        base_color *= texture(bindless_textures[nonuniformEXT(pc.albedo_texture_index)], in_uv0);

    // Same fixed headlight-plus-ambient shading as before, computed entirely in object space:
    // dot(object_normal, light_object) equals dot(world_normal, world_light) by construction
    // (see prefab_thumbnail.vert's Push comment and the CPU-side write_light_object doc
    // comment), without ever forming a world-space normal in this shader.
    const float ambient = 0.35;
    vec3 normal = normalize(in_object_normal);
    vec3 light = normalize(pc.light_object.xyz);
    float headlight = max(dot(normal, light), 0.0);
    float shade = clamp(ambient + headlight * (1.0 - ambient), 0.0, 1.0);

    out_color = vec4(base_color.rgb * shade, base_color.a);
}
