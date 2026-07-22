#version 450
#extension GL_GOOGLE_include_directive : require

// Outline rendering for the selected object: a solid warm colour, drawn in wireframe
// and masked by stencil so only the silhouette survives. Emits the entity id so a
// click on the outline still picks the object.

#include "temporal_common.glsl"

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

layout(location = 0) in vec3 v_world_position;
layout(location = 1) in vec3 v_world_normal;
layout(location = 6) in vec4 v_current_clip;
layout(location = 7) in vec4 v_previous_clip;

layout(location = 0) out vec4 out_color;
layout(location = 1) out uint out_id;
layout(location = 2) out vec2 out_velocity;
layout(location = 3) out vec2 out_gbuffer;

void main()
{
    out_color = vec4(1.0, 0.65, 0.2, 1.0);
    out_id = pc.entity_id;
    out_velocity = motion_vector(v_current_clip, v_previous_clip);
    out_gbuffer = vec2(1.0, 0.04); // the selection outline is not a reflective surface
}
