#version 450
#extension GL_GOOGLE_include_directive : require

// Depth-only vertex shader for the sun's shadow cascades. It has no fragment stage at
// all: a shadow map is depth and nothing else, and the fixed-function depth write is the
// whole of the work.
//
// The cascade being rendered arrives in the push constant's spare slot rather than in a
// descriptor, so all four cascades are one pass with four viewport changes over one
// atlas — no set rebinding, no per-cascade allocation.

#include "shadow_common.glsl"

layout(push_constant) uniform Push
{
    mat4 model;
    vec4 albedo_metallic;
    vec4 emissive_roughness;
    vec4 outline_shift;      // z = which cascade this draw is rendering
    uint entity_id;
    uint selected;
    uint material_index;
    uint motion_index;
} pc;

layout(location = 0) in vec3 in_position;

void main()
{
    int cascade = clamp(int(pc.outline_shift.z), 0, MAX_SHADOW_CASCADES - 1);
    gl_Position = shadows.cascade_view_projection[cascade] * pc.model * vec4(in_position, 1.0);
}
