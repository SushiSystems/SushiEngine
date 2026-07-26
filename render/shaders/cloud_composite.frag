#version 450
#extension GL_GOOGLE_include_directive : require

// Resolves the cloud buffer's own dedicated TAA (CloudTaaPass) over the full-resolution
// sky, producing the linear HDR image the rest of the frame works on.
//
// This used to be two lines at the top of the tonemap pass. It is its own pass now
// because the temporal resolve has to run on a complete scene — clouds included — and
// the display transform has to run after that, so nothing may sit on both sides of it.
// Splitting it also gives the post-processing stack one obvious place to attach.
//
// W3 adds the cloud's own aerial perspective: the Hillaire froxel volume sampled once
// per pixel at the cloud march's own transmittance-weighted mean depth, so a distant
// deck hazes, desaturates, and sinks toward the horizon exactly like a mesh already does
// — see cloud_sample_ex()'s depth reconstruction and the blend at the bottom of main().

#include "atmosphere_common.glsl"

layout(set = 0, binding = 1) uniform sampler2D sky_texture;
// CloudTaaPass's own resolved history: rgb = premultiplied in-scattered cloud light,
// a = transmittance along the view ray. Point-sampled: the composite below does its own
// weighted reconstruction from the four nearest texels, so a hardware bilinear tap here
// would blend a texel a second time.
layout(set = 0, binding = 2) uniform sampler2D cloud_texture;
// The analytic ground's direct-sun term, held out of sky_texture by sky.frag and
// blurred by ground_shadow_resolve.frag: rgb = unshadowed direct contribution, a = the
// blurred cascade visibility. Folded in here, before the cloud transmittance multiply,
// so it is attenuated by aerial cloud cover exactly as it would have been had it stayed
// inline in sky_texture.
layout(set = 0, binding = 3) uniform sampler2D ground_shadow_texture;
// Full-resolution scene depth, point-sampled, read at this pixel and at the four
// half-resolution cloud texels nearest to it so the upsample below can tell which of
// them saw the same surface this pixel did.
layout(set = 0, binding = 4) uniform sampler2D depth_texture;
// CloudPass's own W3 MRT sibling: the transmittance-weighted mean march depth, same
// grid as cloud_texture, reconstructed by the identical nearest-depth weights so the
// aerial lookup below samples the same "which texel actually matched" choice the colour
// upsample already made.
layout(set = 0, binding = 5) uniform sampler2D cloud_depth_texture;
// The Hillaire aerial-perspective froxel volume atmosphere_lut_pass.cpp builds — the
// same volume sky.frag already samples for mesh pixels within froxel range.
layout(set = 0, binding = 27) uniform sampler3D aerial_volume;

layout(location = 0) in vec2 v_ndc;

layout(location = 0) out vec4 out_color;

// Nearest-depth-aware upsample of the cloud buffer, extended for W3 to reconstruct the
// march's own depth alongside its colour with the same four weights: a plain bilinear
// tap would blend texels that sat on opposite sides of a silhouette (haloing foreground
// geometry) or, for the depth channel, averaging two unrelated cloud distances into a
// meaningless one the aerial lookup would then sample. Weighting each of the four
// bilinear taps by whether its depth roughly matches this pixel's own removes the ones
// that saw a different surface instead of averaging them in.
void cloud_sample_ex(vec2 uv, float depth_here, out vec4 color, out float cloud_distance)
{
    ivec2 cloud_size = textureSize(cloud_texture, 0);
    vec2 texel = 1.0 / vec2(cloud_size);
    vec2 coord = uv * vec2(cloud_size) - 0.5;
    vec2 base = floor(coord);
    vec2 frac_part = coord - base;

    vec2 uv00 = (base + vec2(0.5, 0.5)) * texel;
    vec2 uv10 = (base + vec2(1.5, 0.5)) * texel;
    vec2 uv01 = (base + vec2(0.5, 1.5)) * texel;
    vec2 uv11 = (base + vec2(1.5, 1.5)) * texel;

    // Depth is nonlinear, so an exact match is not expected even between neighbouring
    // texels on the same surface; a small relative epsilon absorbs that without needing
    // to linearise it first.
    float epsilon = max(depth_here, 1e-5) * 0.02;
    float w00 = abs(texture(depth_texture, uv00).r - depth_here) < epsilon ? 1.0 : 0.05;
    float w10 = abs(texture(depth_texture, uv10).r - depth_here) < epsilon ? 1.0 : 0.05;
    float w01 = abs(texture(depth_texture, uv01).r - depth_here) < epsilon ? 1.0 : 0.05;
    float w11 = abs(texture(depth_texture, uv11).r - depth_here) < epsilon ? 1.0 : 0.05;

    float bw00 = (1.0 - frac_part.x) * (1.0 - frac_part.y) * w00;
    float bw10 = frac_part.x * (1.0 - frac_part.y) * w10;
    float bw01 = (1.0 - frac_part.x) * frac_part.y * w01;
    float bw11 = frac_part.x * frac_part.y * w11;

    float wsum = bw00 + bw10 + bw01 + bw11;
    if (wsum < 1e-6)
    {
        color = texture(cloud_texture, uv00);
        cloud_distance = texture(cloud_depth_texture, uv00).r;
        return;
    }

    color = (texture(cloud_texture, uv00) * bw00 + texture(cloud_texture, uv10) * bw10 +
            texture(cloud_texture, uv01) * bw01 + texture(cloud_texture, uv11) * bw11) /
           wsum;
    cloud_distance = (texture(cloud_depth_texture, uv00).r * bw00 +
                      texture(cloud_depth_texture, uv10).r * bw10 +
                      texture(cloud_depth_texture, uv01).r * bw01 +
                      texture(cloud_depth_texture, uv11).r * bw11) /
                     wsum;
}

void main()
{
    vec2 uv = v_ndc * 0.5 + 0.5;
    vec3 sky = texture(sky_texture, uv).rgb;
    float depth_here = texture(depth_texture, uv).r;
    vec4 cloud;
    float cloud_distance;
    cloud_sample_ex(uv, depth_here, cloud, cloud_distance);
    vec4 ground_shadow = texture(ground_shadow_texture, uv);
    vec3 ground_direct = ground_shadow.rgb * ground_shadow.a;

    // Aerial perspective on the cloud's own in-scattered light: the air between the
    // camera and the cloud's own weighted-mean depth attenuates what the cloud itself
    // emits (aerial.a) and adds its own in-scatter on top (aerial.rgb), weighted by how
    // much cloud is actually here (1 - cloud.a) so a clear-sky pixel — whose reconstructed
    // depth is otherwise meaningless — never picks up a phantom haze layer.
    vec4 aerial = sample_aerial(aerial_volume, uv, min(cloud_distance, AERIAL_MAX_DISTANCE));
    vec3 cloud_hazed = cloud.rgb * aerial.a + aerial.rgb * (1.0 - cloud.a);

    out_color = vec4((sky + ground_direct) * cloud.a + cloud_hazed, 1.0);
}
