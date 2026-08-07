#version 450

// Locations match MeshVertex's 60-byte layout exactly (engine/domain/geometry/include/
// SushiEngine/geometry/mesh_vertex.hpp): position @0, normal @1, tangent @2 (unused here),
// uv0 @3, uv1 @4 (unused here), color @5 (unused here). Vulkan does not require a pipeline's
// vertex input attributes to be contiguous, so skipping the unused locations costs nothing.
layout(location = 0) in vec3 in_position;
layout(location = 1) in vec3 in_normal;
layout(location = 3) in vec2 in_uv0;

layout(location = 0) out vec3 out_world_position;
layout(location = 1) out vec3 out_world_normal;
layout(location = 2) out vec2 out_uv0;

layout(push_constant) uniform Push
{
    mat4 model;
    mat4 view_projection;
    vec4 albedo;
    int albedo_texture_index; // -1 = no texture; flat albedo tint only.
} pc;

void main()
{
    vec4 world_position = pc.model * vec4(in_position, 1.0);
    out_world_position = world_position.xyz;
    // No non-uniform scale in this pipeline's usage (import_gltf already bakes node transforms
    // into vertex positions, so pc.model is always identity here) -- a plain 3x3 rotation of
    // the normal is correct without a separate inverse-transpose normal matrix.
    out_world_normal = mat3(pc.model) * in_normal;
    out_uv0 = in_uv0;
    gl_Position = pc.view_projection * world_position;
}
