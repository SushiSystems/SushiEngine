#version 450

// Shared vertex shader for the scene view's lit meshes and flat grid lines. The
// per-frame camera (view, projection) comes from the scene uniform block; the
// per-draw model matrix and material come from the push constant. World-space
// position and normal are handed to the fragment stage so the PBR shader can light
// in world space against the scene's directional sun.

layout(set = 0, binding = 0) uniform SceneBlock
{
    mat4 view;
    mat4 proj;
    vec4 cam_forward;
    vec4 cam_right;
    vec4 cam_up;
    vec4 planet_center;
    vec4 planet_radii;
    vec4 sun_dir;
    vec4 sun_color;
    vec4 ambient;
    vec4 rayleigh;
    vec4 scatter;
    vec4 ground_albedo;
    vec4 ocean_color;
    vec4 cloud_params;
    vec4 star_params;
    vec4 misc;
} scene;

layout(push_constant) uniform Push
{
    mat4 model;
    vec4 albedo_metallic;    // xyz = albedo, w = metallic
    vec4 emissive_roughness; // xyz = emissive, w = roughness
    vec4 outline_shift;      // xy = screen-space outline shift, zw spare
    uint entity_id;
    uint selected;
} pc;

layout(location = 0) in vec3 in_position;
layout(location = 1) in vec3 in_normal;

layout(location = 0) out vec3 v_world_position;
layout(location = 1) out vec3 v_world_normal;

void main()
{
    vec4 world = pc.model * vec4(in_position, 1.0);
    v_world_position = world.xyz;
    v_world_normal = mat3(pc.model) * in_normal;
    gl_Position = scene.proj * scene.view * world;
}
