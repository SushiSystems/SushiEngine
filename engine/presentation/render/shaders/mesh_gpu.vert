#version 450
#extension GL_GOOGLE_include_directive : require

// GPU-driven twin of mesh.vert. Where mesh.vert reads its per-draw model matrix, material
// index, and picking id from a push constant, this reads them from the frame's instance
// record — because an indirect draw carries no push constant. The cull pass wrote a
// compacted list of the survivors for each bucket; the drawn bucket's base into that list
// is the one value still pushed (per bucket, on the CPU), and gl_InstanceIndex walks the
// bucket's slice from there. Everything downstream — the fragment shader, the motion
// vector, the camera-relative transform — is identical to the classic path, which is why
// both feed the same pbr.frag.

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

// Where every object drawn this frame was one frame ago, indexed the same way the classic
// path indexes it — the instance record carries the motion index in place of the push.
layout(std430, set = 0, binding = 8) readonly buffer MotionBlock
{
    mat4 previous_model[];
} motion;

// One record per drawable, the std430 mirror of Scene::GpuInstance. The cull shader and
// this shader must read it identically, so the field order is fixed.
struct GpuInstance
{
    mat4 model;
    vec4 bounding_sphere;
    uint material_index;
    uint motion_index;
    uint entity_id;
    uint bucket_index;
};

layout(std430, set = 2, binding = 0) readonly buffer InstanceBlock
{
    GpuInstance instances[];
} instance_block;

// The cull pass's compacted survivor list: for each bucket, the instance indices that
// passed, packed into the bucket's reserved slice starting at candidate_base.
layout(std430, set = 2, binding = 1) readonly buffer CompactedBlock
{
    uint indices[];
} compacted;

layout(push_constant) uniform Push
{
    uint candidate_base;
    uint reserved;
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
layout(location = 8) flat out uint v_material_index;
layout(location = 9) flat out uint v_entity_id;

void main()
{
    uint instance_index = compacted.indices[pc.candidate_base + uint(gl_InstanceIndex)];
    GpuInstance instance = instance_block.instances[instance_index];

    vec4 world = instance.model * vec4(in_position, 1.0);
    mat3 normal_matrix = mat3(instance.model);
    v_world_position = world.xyz;
    v_world_normal = normal_matrix * in_normal;
    v_world_tangent = vec4(normal_matrix * in_tangent.xyz, in_tangent.w);
    v_uv0 = in_uv0;
    v_uv1 = in_uv1;
    v_color = in_color;

    vec4 clip = scene.proj * scene.view * world;
    v_current_clip = clip;
    v_previous_clip = temporal.previous_view_projection *
                      motion.previous_model[instance.motion_index] * vec4(in_position, 1.0);
    v_material_index = instance.material_index;
    v_entity_id = instance.entity_id;
    gl_Position = clip;
}
