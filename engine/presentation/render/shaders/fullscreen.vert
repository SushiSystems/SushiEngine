#version 450

// A vertex-only fullscreen triangle: three oversized vertices cover the whole target
// with no vertex buffer. Emits a [-1, 1] NDC coordinate the sky and tonemap fragment
// shaders use to reconstruct the view ray and sample their source images.

layout(location = 0) out vec2 v_ndc;

void main()
{
    vec2 pos = vec2((gl_VertexIndex << 1) & 2, gl_VertexIndex & 2);
    v_ndc = pos * 2.0 - 1.0;
    gl_Position = vec4(v_ndc, 0.0, 1.0);
}
