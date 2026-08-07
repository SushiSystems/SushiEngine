#version 450

// Locations match MeshVertex's 60-byte layout exactly (engine/domain/geometry/include/
// SushiEngine/geometry/mesh_vertex.hpp), the same subset Phase 3a's mesh_thumbnail.vert uses:
// position @0, normal @1, uv0 @3.
layout(location = 0) in vec3 in_position;
layout(location = 1) in vec3 in_normal;
layout(location = 3) in vec2 in_uv0;

layout(location = 0) out vec3 out_world_normal;
layout(location = 1) out vec2 out_uv0;

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
    // Each prefab entity's own transform can carry non-uniform scale (unlike Phase 3a's single-
    // file case, where every primitive shares one identity model space) -- a plain 3x3 rotation
    // of the normal is still an acceptable approximation for this flat/unlit thumbnail fidelity
    // level, matching the same simplification Phase 3a's own vertex shader already made.
    out_world_normal = mat3(pc.model) * in_normal;
    out_uv0 = in_uv0;
    gl_Position = pc.view_projection * world_position;
}
