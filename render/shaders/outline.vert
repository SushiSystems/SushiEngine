#version 450

// Vertex shader for selected object outline: scales the object outward from
// its screen-space center to produce a clean, continuous silhouette outline
// without breaking apart sharp edges.

layout(location = 0) in vec3 in_position;
layout(location = 1) in vec3 in_normal;

layout(push_constant) uniform Push
{
    mat4 mvp;
    vec4 n0;
    vec4 n1;
    vec4 n2;
    uint entity_id;
    uint selected;
} pc;

layout(location = 0) out vec3 v_normal;
layout(location = 1) out vec3 v_color;

void main()
{
    // The shift is provided in n0.w (x) and n1.w (y)
    vec2 shift = vec2(pc.n0.w, pc.n1.w);
    
    // Calculate aspect ratio to ensure the shift is circular
    float aspect = pc.mvp[0][0] != 0.0 ? abs(pc.mvp[1][1] / pc.mvp[0][0]) : 1.0;
    
    vec4 clip_pos = pc.mvp * vec4(in_position, 1.0);
    if (clip_pos.w > 0.0001) {
        vec2 ndc_pos = clip_pos.xy / clip_pos.w;
        
        // Apply shift (which is in physical pixel distance, e.g. 0.012)
        // We divide x by aspect to keep it physically uniform on screen
        ndc_pos.x += shift.x / aspect;
        ndc_pos.y += shift.y;
        
        clip_pos.xy = ndc_pos * clip_pos.w;
    }
    gl_Position = clip_pos;
    
    mat3 normal_basis = mat3(pc.n0.xyz, pc.n1.xyz, pc.n2.xyz);
    v_normal = normal_basis * in_normal;
    v_color = vec3(pc.n0.w, pc.n1.w, pc.n2.w);
}
