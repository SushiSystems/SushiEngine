#version 450
#extension GL_GOOGLE_include_directive : require

// Shared vertex shader for the scene view's lit meshes and flat grid lines. The
// per-frame camera comes from the scene uniform block; the per-draw model matrix and
// material index come from the push constant. Everything the fragment stage needs to
// build a tangent frame and sample a material travels with it: camera-relative
// position, normal, tangent (w = bitangent handedness), two UV sets, and the vertex
// colour.
//
// It also carries this vertex through the *previous* frame's camera, so the fragment
// stage can write a motion vector. That second transform comes from the frame's motion
// array rather than the push constant, which keeps the push constant fixed-size.

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

// Where every object drawn this frame was one frame ago, camera-relative against the
// previous frame's eye so it pairs with temporal.previous_view_projection.
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

layout(location = 0) out vec3 v_world_position;
layout(location = 1) out vec3 v_world_normal;
layout(location = 2) out vec4 v_world_tangent;
layout(location = 3) out vec2 v_uv0;
layout(location = 4) out vec2 v_uv1;
layout(location = 5) out vec4 v_color;
layout(location = 6) out vec4 v_current_clip;
layout(location = 7) out vec4 v_previous_clip;

void main()
{
    vec4 world = pc.model * vec4(in_position, 1.0);
    mat3 normal_matrix = mat3(pc.model);
    v_world_position = world.xyz;
    v_world_normal = normal_matrix * in_normal;
    // A zero tangent means the mesh authored none; it survives the transform as zero
    // and the fragment stage derives a frame from screen-space derivatives instead.
    v_world_tangent = vec4(normal_matrix * in_tangent.xyz, in_tangent.w);
    v_uv0 = in_uv0;
    v_uv1 = in_uv1;
    v_color = in_color;

    vec4 clip = scene.proj * scene.view * world;
    // scene.proj already carries this frame's jitter; the previous transform carries
    // none, and motion_vector() removes the difference in the fragment stage.
    v_current_clip = clip;
    v_previous_clip = temporal.previous_view_projection *
                      motion.previous_model[pc.motion_index] * vec4(in_position, 1.0);
    gl_Position = clip;
}
