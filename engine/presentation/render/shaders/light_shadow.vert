#version 450
#extension GL_GOOGLE_include_directive : require

// Depth-only vertex shader for the punctual (spot) shadow atlas. Like shadow.vert for
// the sun, it has no fragment stage — a shadow map is depth and nothing else — and picks
// its light matrix from a spare push-constant slot, so every caster's tile is one pass
// with a viewport change over one atlas, no descriptor rebind.
//
// The matrices are camera-relative, matching the position the shading pass works in.

struct LightShadow
{
    mat4 view_proj; // camera-relative light clip
    vec4 tile;      // xy = atlas uv offset, z = uv scale, w spare
};

layout(std430, set = 0, binding = 19) readonly buffer LightShadowData
{
    LightShadow records[];
} light_shadow_data;

layout(push_constant) uniform Push
{
    mat4 model;
    vec4 albedo_metallic;
    vec4 emissive_roughness;
    vec4 outline_shift; // z = which shadow record this draw renders
    uint entity_id;
    uint selected;
    uint material_index;
    uint motion_index;
} pc;

layout(location = 0) in vec3 in_position;

void main()
{
    int record = int(pc.outline_shift.z);
    gl_Position = light_shadow_data.records[record].view_proj * pc.model * vec4(in_position, 1.0);
}
