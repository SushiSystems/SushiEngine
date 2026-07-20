#version 450
#extension GL_GOOGLE_include_directive : require

// Vertex shader for the selected object's outline: transforms with the scene camera
// and the per-draw model matrix, then nudges the vertex outward in screen space by
// the push-constant shift so the silhouette reads as a clean, continuous outline
// without breaking apart at sharp edges. It carries the same pair of clip positions
// the lit path does, so the outline writes a motion vector rather than punching a hole
// of stale motion through the object it surrounds.

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
    vec4 outline_shift;      // xy = screen-space shift
    uint entity_id;
    uint selected;
    uint material_index;
    uint motion_index;
} pc;

layout(location = 0) in vec3 in_position;
layout(location = 1) in vec3 in_normal;

layout(location = 0) out vec3 v_world_position;
layout(location = 1) out vec3 v_world_normal;
layout(location = 6) out vec4 v_current_clip;
layout(location = 7) out vec4 v_previous_clip;

void main()
{
    mat4 mvp = scene.proj * scene.view * pc.model;
    vec2 shift = pc.outline_shift.xy;
    float aspect = mvp[0][0] != 0.0 ? abs(mvp[1][1] / mvp[0][0]) : 1.0;

    vec4 clip_pos = mvp * vec4(in_position, 1.0);
    if (clip_pos.w > 0.0001)
    {
        vec2 ndc_pos = clip_pos.xy / clip_pos.w;
        ndc_pos.x += shift.x / aspect;
        ndc_pos.y += shift.y;
        clip_pos.xy = ndc_pos * clip_pos.w;
    }
    gl_Position = clip_pos;

    // The screen-space expansion is part of where this fragment actually landed, so
    // the shifted position is what its motion vector must be measured from.
    v_current_clip = clip_pos;
    v_previous_clip = temporal.previous_view_projection *
                      motion.previous_model[pc.motion_index] * vec4(in_position, 1.0);

    v_world_position = (pc.model * vec4(in_position, 1.0)).xyz;
    v_world_normal = mat3(pc.model) * in_normal;
}
