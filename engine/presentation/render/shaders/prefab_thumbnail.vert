#version 450

// Locations match MeshVertex's 60-byte layout exactly (engine/domain/geometry/include/
// SushiEngine/geometry/mesh_vertex.hpp), the same subset Phase 3a's mesh_thumbnail.vert uses:
// position @0, normal @1, uv0 @3.
layout(location = 0) in vec3 in_position;
layout(location = 1) in vec3 in_normal;
layout(location = 3) in vec2 in_uv0;

layout(location = 0) out vec3 out_object_normal;
layout(location = 1) out vec2 out_uv0;

// Push constants stay under Vulkan's guaranteed 128-byte minimum (this repo's "house budget",
// see particle_mesh_pass.hpp/particle_pass.hpp/particle_sim_pass.hpp) by carrying a
// pre-multiplied model*view_projection matrix instead of the two matrices separately, and by
// shading in object space -- light_object is the fixed world headlight direction, transformed
// into this draw's object space on the CPU as transpose(model) * light_world, so the shader
// never needs the model matrix itself or a world-space normal.
layout(push_constant) uniform Push
{
    mat4 mvp;
    vec4 light_object; // xyz = fixed headlight direction in this draw's object space
    vec4 albedo;
    int albedo_texture_index; // -1 = no texture; flat albedo tint only.
} pc;

void main()
{
    out_object_normal = in_normal;
    out_uv0 = in_uv0;
    gl_Position = pc.mvp * vec4(in_position, 1.0);
}
