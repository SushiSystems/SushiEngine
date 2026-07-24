#version 450
#extension GL_GOOGLE_include_directive : require

// The skinned variant of mesh.vert. Its vertex buffer is the SkinningPass's output — one
// interleaved SkinnedVertex per base vertex (the MeshVertex layout plus a previous-frame
// skinned position), so it reuses the same fragment shader (pbr.frag) as the classic mesh
// path. The one difference is the motion vector: a skinned vertex's previous position is
// *not* previous_model * current_position (that would give rigid motion and ghost every
// deforming limb), so it reads the previous-frame skinned position the compute pass wrote
// and transforms it by the previous rigid model. This is what keeps a skinned character
// free of TAA ghosting (acceptance criterion 5).

#include "temporal_common.glsl"

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

layout(std430, set = 0, binding = 8) readonly buffer MotionBlock
{
    mat4 previous_model[];
} motion;

layout(push_constant) uniform Push
{
    mat4 model;
    vec4 albedo_metallic;
    vec4 emissive_roughness;
    vec4 outline_shift;
    uint entity_id;
    uint selected;
    uint material_index;
    uint motion_index;
} pc;

layout(location = 0) in vec3 in_position;
layout(location = 1) in vec3 in_normal;
layout(location = 2) in vec4 in_tangent;
layout(location = 3) in vec2 in_uv0;
layout(location = 4) in vec2 in_uv1;
layout(location = 5) in vec4 in_color;
layout(location = 6) in vec3 in_prev_position; // previous-frame skinned position (object space)

layout(location = 0) out vec3 v_world_position;
layout(location = 1) out vec3 v_world_normal;
layout(location = 2) out vec4 v_world_tangent;
layout(location = 3) out vec2 v_uv0;
layout(location = 4) out vec2 v_uv1;
layout(location = 5) out vec4 v_color;
layout(location = 6) out vec4 v_current_clip;
layout(location = 7) out vec4 v_previous_clip;
layout(location = 8) flat out uint v_material_index;
layout(location = 9) flat out uint v_entity_id;

void main()
{
    vec4 world = pc.model * vec4(in_position, 1.0);
    mat3 normal_matrix = mat3(pc.model);
    v_world_position = world.xyz;
    v_world_normal = normal_matrix * in_normal;
    v_world_tangent = vec4(normal_matrix * in_tangent.xyz, in_tangent.w);
    v_uv0 = in_uv0;
    v_uv1 = in_uv1;
    v_color = in_color;

    vec4 clip = scene.proj * scene.view * world;
    v_current_clip = clip;
    // The previous position is the compute pass's previous-frame skinned position, carried
    // through the previous rigid model and the previous view-projection.
    v_previous_clip = temporal.previous_view_projection *
                      motion.previous_model[pc.motion_index] * vec4(in_prev_position, 1.0);
    v_material_index = pc.material_index;
    v_entity_id = pc.entity_id;
    gl_Position = clip;
}
