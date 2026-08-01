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

// Truncated prefix of the shared scene block, declared through `misc` — the one member
// this pass reads. misc.w is the cloudscape master switch, and the composite has to
// consult it because CloudTaaPass's history is pass-owned and outlives a disable:
// without the gate below, switching clouds off left the last resolved frame glued to
// the screen (the march stops, the TAA early-outs, but this pass kept sampling the
// stale accumulation) until clouds were switched back on.
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
    vec4 cloud_global;
    vec4 star_params;
    vec4 misc;
    // Declared through sky_counts so the eclipse fraction is available: the in-scatter added
    // below is scaled by the sun the same way sky.frag scales it, and that includes totality.
    // std140 offsets are positional, so reaching a member means naming everything before it.
    vec4 sky_counts;
} scene;

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
// The transmittance LUT, for the part of the view path the froxel volume above cannot reach.
layout(set = 0, binding = 24) uniform sampler2D transmittance_lut;

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
    float d00 = abs(texture(depth_texture, uv00).r - depth_here);
    float d10 = abs(texture(depth_texture, uv10).r - depth_here);
    float d01 = abs(texture(depth_texture, uv01).r - depth_here);
    float d11 = abs(texture(depth_texture, uv11).r - depth_here);
    float w00 = d00 < epsilon ? 1.0 : 0.05;
    float w10 = d10 < epsilon ? 1.0 : 0.05;
    float w01 = d01 < epsilon ? 1.0 : 0.05;
    float w11 = d11 < epsilon ? 1.0 : 0.05;

    // When *no* tap matches, take the nearest one outright instead of averaging.
    //
    // This is the residual bright line along the horizon. The 0.05 demotion only works while
    // at least one tap is still weighted 1.0; when all four are rejected they are all 0.05,
    // and dividing by their sum renormalises them straight back into ordinary bilinear
    // weights — the rejection cancels itself out and the filter blends across the largest
    // discontinuity in the buffer. The `wsum < 1e-6` guard below could never catch it,
    // because 4 x 0.05 is not a small number.
    //
    // Where that happens is exactly one row of ground pixels. The depth epsilon is 2 %
    // *relative*, and at grazing incidence a single screen row spans an unbounded range of
    // ground distance, so every tap fails. It stays one row wide at any camera height or field
    // of view, because bilinear only reaches one texel: every row below it has all four taps
    // inside the clean ground region, where the filter is a no-op.
    //
    // And it reads near-white rather than as a soft seam because of what it does to the
    // *distance*: the blend drags `cloud_distance` from ~100 km down under AERIAL_MAX_DISTANCE,
    // so that row skips the entire long-path extinction tail below. A convex blend of two
    // neighbours can never be brighter than both; skipping the extinction can.
    bool any_match = w00 > 0.5 || w10 > 0.5 || w01 > 0.5 || w11 > 0.5;
    if (!any_match)
    {
        vec2 nearest = uv00;
        float best = d00;
        if (d10 < best) { best = d10; nearest = uv10; }
        if (d01 < best) { best = d01; nearest = uv01; }
        if (d11 < best) { best = d11; nearest = uv11; }
        color = texture(cloud_texture, nearest);
        cloud_distance = texture(cloud_depth_texture, nearest).r;
        return;
    }

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
    vec4 ground_shadow = texture(ground_shadow_texture, uv);
    vec3 ground_direct = ground_shadow.rgb * ground_shadow.a;

    // Clouds off: pass the sky through untouched instead of sampling a TAA history
    // nothing is updating. Identical to compositing a clean buffer (0, 0, 0, 1).
    if (scene.misc.w <= 0.5)
    {
        out_color = vec4(sky + ground_direct, 1.0);
        return;
    }

    float depth_here = texture(depth_texture, uv).r;
    vec4 cloud;
    float cloud_distance;
    cloud_sample_ex(uv, depth_here, cloud, cloud_distance);

    // Aerial perspective on the cloud's own in-scattered light: the air between the
    // camera and the cloud's own weighted-mean depth attenuates what the cloud itself
    // emits (aerial.a) and adds its own in-scatter on top (aerial.rgb), weighted by how
    // much cloud is actually here (1 - cloud.a) so a clear-sky pixel — whose reconstructed
    // depth is otherwise meaningless — never picks up a phantom haze layer.
    vec4 aerial = sample_aerial(aerial_volume, uv, min(cloud_distance, AERIAL_MAX_DISTANCE));

    // The in-scatter term is stored per unit sun radiance, exactly like the sky's own use of
    // this volume — sky.frag scales the identical fetch by `sun_radiance` before adding it.
    // Unscaled it arrives at 1/intensity of its true strength (the authored sun is 20), which
    // is why distant cloud was only ever dimmed by the grey `aerial.a` and never pushed toward
    // the sky's colour: the half of aerial perspective that *replaces* an object with haze was
    // effectively absent.
    vec3 sun_radiance =
        scene.sun_color.xyz * scene.sun_dir.w * (1.0 - 0.92 * scene.sky_counts.w);

    // Past the froxel volume's reach, continue the view path analytically.
    //
    // The volume's last slice sits at ~31 km, but a cloud deck seen from the ground runs all
    // the way to its own base-sphere tangent — over 100 km for a 1 km base — and the last
    // degree or so of sky above the horizon holds every distance in between. Clamping all of
    // it to the 31 km answer leaves that strip several times too bright *and*, worse, makes it
    // stop darkening with distance at all: the horizon reads as a hard, uniform wall rather
    // than as cloud fading into air. That is most of the white band along the horizon.
    //
    // The continuation is Bruneton's ratio identity. The LUT answers "transmittance from this
    // radius to the top of the atmosphere along this direction", and the near point's path to
    // the top runs through the far point, so T(near->top) = T(near->far) * T(far->top) and the
    // segment we want is the quotient. Two LUT fetches, paid only by the pixels past 31 km.
    if (cloud_distance > AERIAL_MAX_DISTANCE)
    {
        AtmosphereMedium medium;
        medium.rayleigh_scattering = scene.rayleigh.xyz;
        medium.mie_scattering = scene.rayleigh.w;
        medium.mie_extinction = scene.rayleigh.w * 1.1;
        medium.rayleigh_scale_height = scene.scatter.y;
        medium.mie_scale_height = scene.scatter.z;
        medium.bottom_radius = scene.planet_center.w;
        medium.top_radius = scene.planet_center.w + scene.planet_radii.w;

        vec3 view_dir = normalize(scene.cam_forward.xyz + v_ndc.x * scene.cam_right.xyz +
                                  v_ndc.y * scene.cam_up.xyz);
        vec3 origin = -scene.planet_center.xyz; // the camera, in planet-centred coordinates

        // The two ends of the stretch the froxel volume does not cover: where its last slice
        // stopped, and the cloud itself.
        vec3 near_p = origin + view_dir * AERIAL_MAX_DISTANCE;
        vec3 far_p = origin + view_dir * cloud_distance;
        float r_near = max(length(near_p), medium.bottom_radius);
        float r_far = max(length(far_p), medium.bottom_radius);
        float mu_near = dot(near_p / r_near, view_dir);
        float mu_far = dot(far_p / r_far, view_dir);

        // t_near / t_far, not the other way round: the near point has *more* atmosphere ahead
        // of it, so t_near <= t_far and the quotient is the segment's own transmittance. The
        // clamp guards the discretised LUT and the grazing rays where mu folds sign.
        vec3 t_near = sample_transmittance(transmittance_lut, medium, r_near, mu_near);
        vec3 t_far = sample_transmittance(transmittance_lut, medium, r_far, mu_far);
        vec3 tail = clamp(t_near / max(t_far, vec3(1e-5)), vec3(0.0), vec3(1.0));
        float tail_luma = max(dot(tail, vec3(0.2126, 0.7152, 0.0722)), 1e-4);

        // The tail's own glow. The froxel volume reports in-scatter `aerial.rgb` accumulated
        // against transmittance `aerial.a`, so the source radiance those two imply is
        // `aerial.rgb / (1 - aerial.a)`; extending that source across the tail's own
        // extinction is the single-scattering-equivalent continuation, and it is continuous
        // with the last slice by construction, so no ring marks the 31 km hand-off.
        vec3 haze_source = aerial.rgb / max(1.0 - aerial.a, 1e-4);
        aerial.rgb += aerial.a * haze_source * (1.0 - tail_luma);
        aerial.a *= tail_luma;

        // The achromatic part of the tail now rides in `aerial.a`, which is a single number.
        // This is the chromatic residual, and it is the half that matters at sunset: a deck
        // a hundred kilometres away reddens because blue is extinguished along the path, and
        // the froxel volume's rgba16f alpha channel never had room to carry that.
        cloud.rgb *= tail / tail_luma;
    }

    vec3 cloud_hazed = cloud.rgb * aerial.a + sun_radiance * aerial.rgb * (1.0 - cloud.a);

    out_color = vec4((sky + ground_direct) * cloud.a + cloud_hazed, 1.0);
}
