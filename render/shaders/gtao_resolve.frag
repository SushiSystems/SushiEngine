#version 450
#extension GL_GOOGLE_include_directive : require

// The GTAO resolve: a joint-bilateral upsample of the half-resolution AO to full
// resolution, and the view-space bent normal turned into world space for the shading pass.
//
// A plain bilinear upscale of a half-res term bleeds across silhouettes — the very
// object-boundary halo the engine is trying to be rid of — so each half-res tap is weighted
// by how close its own full-res depth is to this pixel's. That both denoises the AO (the
// slice noise dissolves into its neighbours) and keeps the darkening pinned to the surface
// it belongs to. This pass runs once, full-res, and everything downstream just samples it.

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
    vec4 misc;           // x = near plane
} scene;

layout(set = 0, binding = 1) uniform sampler2D depth_texture; // full resolution
layout(set = 0, binding = 2) uniform sampler2D gtao_texture;  // half resolution: bent.xyz, ao.w

layout(location = 0) in vec2 v_ndc;

layout(location = 0) out vec4 out_ao;

float linear_depth(float d) { return d > 0.0 ? scene.misc.x / d : 0.0; }

void main()
{
    vec2 uv = v_ndc * 0.5 + 0.5;
    float d = texture(depth_texture, uv).r;
    float zc = linear_depth(d);
    if (zc <= 0.0)
    {
        out_ao = vec4(0.0, 0.0, 0.0, 1.0); // sky: fully unoccluded, no bent normal
        return;
    }

    vec2 half_texel = 1.0 / vec2(textureSize(gtao_texture, 0));
    vec4 sum = vec4(0.0);
    float wsum = 0.0;
    for (int y = -1; y <= 1; ++y)
        for (int x = -1; x <= 1; ++x)
        {
            vec2 suv = uv + vec2(float(x), float(y)) * half_texel;
            float sz = linear_depth(texture(depth_texture, suv).r);
            // Depth-similarity weight, scaled by this pixel's distance so the tolerance
            // grows with range (a metre matters at arm's length, not at a kilometre).
            float wz = exp(-abs(sz - zc) / max(zc * 0.05, 0.1));
            sum += texture(gtao_texture, suv) * wz;
            wsum += wz;
        }
    vec4 ao = wsum > 0.0 ? sum / wsum : texture(gtao_texture, uv);

    // View-space bent normal to world. The scene view is camera-relative and
    // translation-free, so its rotation transpose maps a view direction back to world.
    vec3 bent_v = ao.xyz;
    vec3 bent_w = length(bent_v) > 1e-3 ? normalize(transpose(mat3(scene.view)) * bent_v)
                                        : vec3(0.0);
    out_ao = vec4(bent_w, clamp(ao.w, 0.0, 1.0));
}
