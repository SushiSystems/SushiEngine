#version 450
#extension GL_GOOGLE_include_directive : require

#include "temporal_common.glsl"

// Half-resolution volumetric cloud pass. Split out of sky.frag so the expensive cloud
// ray march runs at a quarter of the pixels while the crisp sky (sun disk, stars, planet
// relief) stays full resolution. Outputs premultiplied cloud lighting and the view-ray
// transmittance as (scattered.rgb, transmittance); the tonemap pass upsamples this and
// composites it over the full-res sky with `sky * transmittance + scattered`. The march
// is bounded by the opaque depth and the analytic ground exactly as before, so clouds sit
// correctly in front of and behind geometry and terrain.

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
    vec4 sky_counts;
    vec4 planet_frame;
    vec4 cloud_light;
    vec4 ibl_params;     // x = intensity, y = specular mip count, z = ambient mode
    vec4 cloud_deck_a[6];
    vec4 cloud_deck_b[6];
    vec4 cloud_deck_c[6];
    vec4 cloud_deck_d[6];
    vec4 bodies[80];
    vec4 sky_stars[128];
#include "scene_weather_tail.glsl"
} scene;

layout(set = 0, binding = 1) uniform sampler2D depth_texture;
// The T3-baked cloudscape (CloudscapeCompilePass): r = density pre-integrated across the
// whole deck stack, g = the contributing deck's own vertical density profile, so a march
// sample is one fetch instead of the six-deck loop. `cloudscape_skip` is the coarser
// max-pooled copy the cheap/coarse probe reads. Both cover the **near window** — a
// camera-centred, non-wrapping span of world (docs/slop/atmosphere_system.md §7.2), no longer
// a periodic tile; `cloudscape_far` carries the same simulated structure out past the horizon
// at a coarser texel, plus its own optical-depth-toward-the-sun channel, for the part of the
// march that leaves the near window.
// Bindings 2-6: the shared SceneLayout reserves 7/8/9/10 by name (MATERIAL_BINDING,
// MOTION_BINDING, TEMPORAL_BINDING, SHADOW_BINDING — see scene_layout.hpp) as a
// different descriptor type each; only 1-6 are genuinely free per-pass image slots, so
// every image this pass owns outright lives there instead. The far field fits because the
// weather field itself no longer needs one: the bake resolves coverage per column now (§7.4),
// so the march reads the answer rather than the meteorology.
layout(set = 0, binding = 2) uniform sampler3D cloudscape_field;
layout(set = 0, binding = 3) uniform sampler3D cloudscape_skip;
// One raw noise volume, re-admitted for W2's near-camera-only (<200 m, see W3's
// NEAR_BAND_METERS) high-frequency erosion at the Nubis3 811 m incommensurate scale and
// the curl-noise warp at cloud bases — detail the T3 bake cannot carry (it has no
// notion of camera distance) and the skip/field fetches alone are too coarse to
// resolve up close.
layout(set = 0, binding = 4) uniform sampler3D cloud_detail_texture;
// CloudLightVolumePass's amortized summed-density-toward-the-sun volume: the same near
// window and (u, v, height01) addressing as cloudscape_field, refreshed a slice at a time
// across 8 frames. The cheap base signal cloud_sun_energy lights every sample from;
// budget.light_taps layers in the costlier inline cone march on top for near-field crispness.
layout(set = 0, binding = 5) uniform sampler3D cloud_light_volume;
// The far window: r = density, g = profile, b = optical depth toward the sun (the far field's
// own light term, since the volume above only covers the near window). Binding 6 is the last
// per-pass image slot the shared SceneLayout leaves free (7-10 are named/reserved; see the
// note on bindings 2-5).
layout(set = 0, binding = 6) uniform sampler3D cloudscape_far;

#include "cloud_field_window.glsl"

layout(location = 0) in vec2 v_ndc;

layout(location = 0) out vec4 out_color;
// W3's dedicated cloud buffer set: the transmittance-weighted mean march depth, the
// same grid as out_color. Accumulated the way `scattered` already is, weighted by
// `(1 - sample_transmit) * transmittance` per step and normalized by the total
// in-scatter weight, so it reads as "how far along the ray the cloud's own visible mass
// sits" rather than the nearest or farthest sample. CloudCompositePass samples the
// aerial-perspective froxel volume at this one value instead of per march sample.
layout(location = 1) out float out_depth;

// The quality tier's march budget, resolved on the CPU and pushed per frame. Rides the
// shared mesh push range (fragment bytes), which this fullscreen pass otherwise leaves
// unused. Every consumer clamps its value, so a low tier cheapens the march and a stale
// or absent push can never spin the loop out of bounds.
layout(push_constant) uniform CloudBudget
{
    uint steps_near;    // charged sample budget: how many real density evaluations
                        // the march may spend, ceiling on near-camera resolution
    uint steps_far;     // minimum guaranteed step count spread across the whole
                        // march; floors how coarse a step may get far from the camera
    uint light_steps;   // self-shadow steps toward the sun
    // The field's own geometry used to ride here as a wrap period and a skip cell. It moved
    // into the scene block (`cloud_field_*`) when the field became a camera-centred window,
    // because the light volume, the shadow map, the panorama and the ground shadow all need
    // the same mapping and only this pass has a push block to put it in.
    uint light_taps;    // tiered inline light-march correction cadence, 0-3 (Low..Ultra)
    uint near_far_split; // W3: QualityParams::cloud_near_far_split, High/Ultra only
} budget;

const float PI = 3.14159265359;

vec2 ray_sphere(vec3 ro, vec3 rd, vec3 c, float r)
{
    vec3 oc = ro - c;
    float b = dot(oc, rd);
    float cc = dot(oc, oc) - r * r;
    float h = b * b - cc;
    if (h < 0.0)
        return vec2(-1.0, -1.0);
    h = sqrt(h);
    return vec2(-b - h, -b + h);
}

float ray_ellipsoid(vec3 ro, vec3 rd, vec3 c, float a, float b, vec3 pole)
{
    vec3 o = ro - c;
    float o_ax = dot(o, pole);
    vec3 o_rad = o - pole * o_ax;
    float d_ax = dot(rd, pole);
    vec3 d_rad = rd - pole * d_ax;
    float inv_a2 = 1.0 / (a * a);
    float inv_b2 = 1.0 / (b * b);
    float qa = dot(d_rad, d_rad) * inv_a2 + d_ax * d_ax * inv_b2;
    float qb = dot(o_rad, d_rad) * inv_a2 + o_ax * d_ax * inv_b2;
    float qc = dot(o_rad, o_rad) * inv_a2 + o_ax * o_ax * inv_b2 - 1.0;
    float h = qb * qb - qa * qc;
    if (h < 0.0)
        return -1.0;
    h = sqrt(h);
    float t = (-qb - h) / qa;
    if (t < 0.0)
        t = (-qb + h) / qa;
    return t;
}

float phase_mie(float mu, float g)
{
    float g2 = g * g;
    return 1.0 / (4.0 * PI) * (1.0 - g2) / pow(1.0 + g2 - 2.0 * g * mu, 1.5);
}

// How much of the near window applies at a camera-relative position: 1 through its middle,
// falling to 0 across its rim, where the far window takes over. Shared by the density and the
// light lookup so the two always cross over together.
float cloud_near_blend(vec3 p, out vec2 near_uv)
{
    near_uv = cloud_window_uv(scene.cloud_field_near.xy, scene.cloud_field_near.zw, p.xz);
    return cloud_window_near_weight(near_uv, scene.cloud_field_params.x);
}

// Total cloud density at p: one fetch of the T3-baked field instead of the six-deck loop
// this used to run per sample — the field already carries each deck's coverage, shape,
// erosion, cross-deck sum *and*, since phase B, the simulation's own coverage at this
// column, baked once on CloudscapeCompilePass's own schedule.
//
// Two things this function no longer does, both because the field became a camera-centred,
// non-wrapping window (docs/slop/atmosphere_system.md §7.2):
//   * It no longer scrolls the lookup by `wind * time`. The wind is absorbed by the bake's
//     own pattern origin; whatever has blown since that bake is already folded into
//     `cloud_field_near.zw` on the CPU, so the pattern still drifts continuously and the
//     lookup still lands inside the window.
//   * It no longer takes a second, mirrored anti-repetition tap. That tap existed to hide a
//     32 km tile period, and there is no period left to hide: the field is unique over the
//     window, and the simulation's structure across it is unique over hundreds of kilometres.
//
// `cheap` selects the coarser, max-pooled skip field (cloud_light_march's many samples, and
// the view march's coarse probe) over the fine field. `profile` is the fine field's baked
// vertical density-gradient channel (0 at a cloud's own edge, 1 through its middle) for the
// W2 profile-gradient ambient term; left at 0 on the cheap path, which only has the skip
// field's single density channel to read.
float cloud_density(vec3 p, bool cheap, out float ambient_h, out float profile)
{
    profile = 0.0;
    float base_min = scene.cloud_global.y;
    float top_max = scene.cloud_global.z;
    float altitude = length(p - scene.planet_center.xyz) - scene.planet_center.w;
    ambient_h = clamp((altitude - base_min) / max(top_max - base_min, 1.0), 0.0, 1.0);
    if (altitude < base_min || altitude > top_max)
        return 0.0;

    vec2 near_uv;
    float near_weight = cloud_near_blend(p, near_uv);

    float density = 0.0;
    if (near_weight > 0.0)
    {
        if (cheap)
        {
            density = texture(cloudscape_skip, vec3(near_uv, ambient_h)).r;
        }
        else
        {
            vec2 fine = texture(cloudscape_field, vec3(near_uv, ambient_h)).rg;
            density = fine.r;
            profile = fine.g;
        }
    }
    if (near_weight < 1.0)
    {
        // Past the near window the same simulated sky continues, resolved coarsely. Both
        // windows were baked from the same weather field, so they agree about where the
        // weather is and differ only in how finely they carve its shape — which is what makes
        // the cross-fade read as detail falling away with distance rather than as a seam.
        vec2 far_uv = cloud_window_uv(scene.cloud_field_far.xy, scene.cloud_field_far.zw, p.xz);
        vec2 coarse = texture(cloudscape_far, vec3(far_uv, ambient_h)).rg;
        density = mix(coarse.r, density, near_weight);
        if (!cheap)
            profile = mix(coarse.g, profile, near_weight);
    }
    return density;
}

// The costlier inline self-shadow cone march: still the ground truth for near-field
// shadow detail the coarse, 8-frame-lagged light volume cannot carry (a cloud's own
// wisp shadowing itself as the camera flies past it). budget.light_taps (see below)
// decides how often the view march can still afford to call this per pixel now that the
// light volume covers the common case.
float cloud_light_march(vec3 p, vec3 sun)
{
    int LIGHT_STEPS = clamp(int(budget.light_steps), 1, 12);
    vec3 b1 = normalize(cross(sun, vec3(0.31, 0.86, 0.41)));
    vec3 b2 = cross(sun, b1);
    float shell = max(scene.cloud_global.z - scene.cloud_global.y, 1.0);
    float cone_radius = shell * 0.08;
    float step_len = shell * 0.04;
    float depth = 0.0;
    float t = 0.0;
    for (int i = 0; i < LIGHT_STEPS; ++i)
    {
        float cone = float(i) / float(LIGHT_STEPS);
        float a = float(i) * 2.4;
        vec3 offset = (b1 * cos(a) + b2 * sin(a)) * cone * cone_radius;
        float h, prof;
        depth += cloud_density(p + sun * (t + step_len * 0.5) + offset, true, h, prof) * step_len;
        t += step_len;
        step_len *= 2.1;
    }
    return depth;
}

// The baked optical depth toward the sun at p, from whichever window covers it: the
// amortized light volume inside the near window, the far field's own light channel past it.
// Cross-faded on exactly the same weight cloud_density uses, so a sample never reads its
// density from one window and its light from the other.
//
// Both windows are needed because they answer different questions. Before phase B a sample
// past the near window could reuse the near reading — the sky was uniform, so it was the same
// answer. With coverage resolved per column it is not: a distant storm and a distant fair
// weather field are different amounts of cloud, and lighting them alike would show the
// front's shape while lighting it flat.
float cloud_light_volume_sample(vec3 p, float ambient_h)
{
    vec2 near_uv;
    float near_weight = cloud_near_blend(p, near_uv);

    float depth = 0.0;
    if (near_weight > 0.0)
        depth = texture(cloud_light_volume, vec3(near_uv, ambient_h)).r;
    if (near_weight < 1.0)
    {
        vec2 far_uv = cloud_window_uv(scene.cloud_field_far.xy, scene.cloud_field_far.zw, p.xz);
        float far_depth = texture(cloudscape_far, vec3(far_uv, ambient_h)).b *
                          CLOUD_FAR_SUN_DEPTH_METERS;
        depth = mix(far_depth, depth, near_weight);
    }
    return depth;
}

// Cheap curl-noise warp (a divergence-free field built from central differences of a
// scalar potential) so cloud bases roll and billow instead of eroding along the noise's
// own straight gradient — the same visual cue Dagor/Nubis3 use to sell a base as
// "boiling" rather than merely fuzzy. `scale` is the potential's sample frequency.
vec3 curl_from_scalar(vec3 p, float scale)
{
    float e = scale * 0.05;
    float nx1 = texture(cloud_detail_texture, (p + vec3(e, 0.0, 0.0)) / scale).r;
    float nx2 = texture(cloud_detail_texture, (p - vec3(e, 0.0, 0.0)) / scale).r;
    float nz1 = texture(cloud_detail_texture, (p + vec3(0.0, 0.0, e)) / scale).r;
    float nz2 = texture(cloud_detail_texture, (p - vec3(0.0, 0.0, e)) / scale).r;
    return vec3(nz1 - nz2, 0.0, -(nx1 - nx2)) * (scale * 0.5);
}

// Near-camera-only (<200 m, the design doc's W3 near-band radius) high-frequency
// erosion at the Nubis3-style 811 m incommensurate scale, with the curl warp folded in
// near cloud bases (height01 < 0.35). The T3 field bakes each deck's own authored
// detail_scale erosion already; this is an *additional*, fixed-scale tap the bake
// cannot carry because it depends on where the camera actually is, deliberately gated
// tight so its cost never reaches the far field or the coarse/cheap paths that do not
// need it.
const float EROSION_SCALE_METERS = 811.0;
// W3 near/far band radius (design doc §4.4/§4.7): inside it the march always keeps full
// quality (the erosion tap above); a literal dual-resolution viewport split beyond it is
// deferred (see the W3 CHANGELOG entry) in favour of the jitter freeze below.
const float NEAR_BAND_METERS = 200.0;
// Nubis3's "jitter animated only < 250 m, static hash beyond": past this distance the
// march's per-pixel dither phase is frozen instead of animated frame to frame, so a
// distant silhouette's sample pattern stays put and the cloud TAA's history never has to
// re-converge on a moving dither — the near/far split's actual contribution to "no
// shimmer under motion" at distance.
const float JITTER_FREEZE_METERS = 250.0;

// The Nubis3 step rule's `skip_distance`: how far the march may hop once the coarse probe
// proves the block it just read is empty. That block is the skip field's max-pool cell inside
// the near window and one far-field texel past it, so the hop is exactly as long as the
// emptiness that was actually proven — never longer.
float cloud_skip_distance(vec3 p)
{
    vec2 near_uv;
    float near_weight = cloud_near_blend(p, near_uv);
    return mix(scene.cloud_field_params.w, scene.cloud_field_params.z, near_weight);
}

float near_field_erosion(vec3 p, float height01)
{
    vec3 wp = p;
    if (height01 < 0.35)
        wp += curl_from_scalar(p, EROSION_SCALE_METERS * 1.6) * (1.0 - height01 / 0.35);
    float n = texture(cloud_detail_texture, wp / EROSION_SCALE_METERS).r;
    return mix(0.55, 1.0, n);
}

// Dual-lobe Henyey-Greenstein scattering with an analytic energy-conserving integration
// across a small Wrenninge-style octave ladder (Hillaire's technique: the biggest
// sample-count reducer, already load-bearing since W0 via this function's caller folding
// `(1 - exp(-sigma*ds))` into the view march itself). The forward lobe is the author's
// own `forward_scattering`; the back lobe (the "dark side" bounce that keeps a cloud from
// reading as a flat silhouette) is a fixed fraction of it, which lands at Nubis3's
// reference g ~= 0.8/-0.2 at the Cloudscape default. `silver_lining` (Ultra only) tilts
// the mix further forward for a sharper glowing rim when looking near-through the sun.
float cloud_sun_energy(float light_depth, float mu, float g, float extinction_scale,
                       bool silver_lining)
{
    const float BACK_G_RATIO = -0.25;
    float forward_mix = silver_lining ? 0.85 : 0.7;
    float energy = 0.0;
    float attenuation = 1.0;
    float contribution = 1.0;
    for (int o = 0; o < 3; ++o)
    {
        float beer = exp(-light_depth * extinction_scale * attenuation);
        float forward = phase_mie(mu, g * attenuation);
        float back = phase_mie(mu, g * BACK_G_RATIO * attenuation);
        // Higher octaves stand in for multiple scattering, which isotropizes with depth,
        // so each successive octave leans the mix toward the isotropic phase.
        float phase = mix(mix(back, forward, forward_mix), 1.0 / (4.0 * PI), float(o) / 2.0);
        energy += contribution * beer * phase * 4.0 * PI;
        attenuation *= 0.5;
        contribution *= 0.5;
    }
    return energy;
}

void main()
{
    out_color = vec4(0.0, 0.0, 0.0, 1.0); // clear sky: no scatter, full transmittance
    out_depth = 0.0; // defined for every early-out below; the real march overwrites it

    if (scene.misc.w <= 0.5 || scene.cloud_global.z <= scene.cloud_global.y)
        return;

    vec3 ro = vec3(0.0);
    // Jittered with the rest of the frame; see sky.frag.
    vec2 ndc = v_ndc - temporal.jitter.xy;
    vec3 rd = normalize(scene.cam_forward.xyz + ndc.x * scene.cam_right.xyz +
                        ndc.y * scene.cam_up.xyz);

    vec3 center = scene.planet_center.xyz;
    float surface_radius = scene.planet_center.w;
    float semi_major = scene.planet_radii.x;
    float semi_minor = scene.planet_radii.z;
    vec3 planet_pole = scene.planet_frame.xyz;
    vec3 sun = normalize(scene.sun_dir.xyz);
    // Dim toward twilight during a solar eclipse (sky_counts.w = covered fraction), so the
    // cloud deck dusks with the sky and ground instead of staying lit through totality.
    vec3 sun_radiance = scene.sun_color.xyz * scene.sun_dir.w * (1.0 - 0.92 * scene.sky_counts.w);

    // Bound the march by the opaque depth and, if the surface planet is active, the ground.
    vec2 uv = v_ndc * 0.5 + 0.5;
    float depth = texture(depth_texture, uv).r;
    float geometry_t = 1e30;
    if (depth > 0.0)
    {
        float z_view = scene.proj[3][2] / (-depth - scene.proj[2][2]);
        float cos_forward = max(dot(rd, scene.cam_forward.xyz), 1e-3);
        geometry_t = (-z_view) / cos_forward;
    }
    bool surface_enabled = scene.sky_counts.z > 0.5;
    float ground_t = surface_enabled
                         ? ray_ellipsoid(ro, rd, center, semi_major, semi_minor, planet_pole)
                         : -1.0;
    bool ground_hit = ground_t > 0.0 && ground_t < geometry_t;

    float base_r = surface_radius + scene.cloud_global.y;
    float top_r = surface_radius + scene.cloud_global.z;
    vec2 outer = ray_sphere(ro, rd, center, top_r);
    if (outer.y <= 0.0)
        return;

    float march_ceiling = geometry_t;
    if (ground_hit)
        march_ceiling = min(march_ceiling, ground_t);
    float t0 = max(outer.x, 0.0);
    float t1 = min(outer.y, march_ceiling);

    // The camera sits below the deck (the ordinary ground-level case): the shell does
    // not start at the camera, it starts where the view ray first climbs to the cloud
    // base. A ray from inside the base sphere always exits it, so `inner.y` is that
    // climb point. Without this the march burns its budget on the empty air underneath
    // the clouds before it ever reaches one.
    float r_cam = length(center);
    if (r_cam < base_r)
    {
        vec2 inner = ray_sphere(ro, rd, center, base_r);
        if (inner.y > 0.0)
            t0 = max(t0, inner.y);
    }

    if (t1 <= t0)
        return;

    float extinction_scale = scene.cloud_light.x * 0.006;

    float march_len = t1 - t0;
    march_len = min(march_len, (top_r - base_r) * 14.0);

    float shell_thick = max(top_r - base_r, 1.0);

    // Distance-driven march: step size grows with distance already travelled from the
    // camera, not with the ray's altitude — altitude picked the wrong pixels to starve,
    // since a grazing ray at cloud-base height paid the same cost as one looking
    // straight up through the thinnest part of the shell. Steps stay at the tier's near
    // floor close to the camera, where screen-space error is largest, and coarsen by
    // 0.08*sqrt(dist) beyond that, capped so the far field is never coarser than the
    // tier's `steps_far` count spread over the whole march.
    int STEPS = clamp(int(budget.steps_near), 8, 160);
    int far_steps = clamp(int(budget.steps_far), 4, STEPS);
    float seg_min = march_len / float(STEPS);
    float seg_max = march_len / float(far_steps);

    // Gradient noise, not a white-noise hash: the march offset's error spreads evenly
    // across neighbouring pixels instead of clumping into speckle, and the jitter-driven
    // phase decorrelates it frame to frame so the temporal resolve integrates it away.
    // W3 near/far split: beyond JITTER_FREEZE_METERS the phase is frozen at a fixed
    // mid-point instead, trading the frame-to-frame decorrelation (which the near-field
    // spatial dither already supplies at distance-scaled pixel footprints) for a stable
    // sample pattern the cloud TAA's history never has to re-converge on.
    bool freeze_jitter = budget.near_far_split != 0u && t0 > JITTER_FREEZE_METERS;
    float dither = freeze_jitter ? 0.5 : temporal_dither(gl_FragCoord.xy);
    float t = t0 + seg_min * dither;
    float t_end = t0 + march_len;

    float mu = dot(rd, sun);
    float g = scene.cloud_light.y;

    vec3 ambient_color = scene.ambient.xyz + sun_radiance * 0.02;

    float lod_distance = shell_thick * 4.0;

    float transmittance = 1.0;
    vec3 scattered = vec3(0.0);
    int lit = 0;
    float sun_energy = 0.0;
    // W3 cloud depth accumulation: the same (1 - sample_transmit) * transmittance
    // weight `scattered` folds radiance in by, applied to the sample's own distance
    // instead, so the normalized sum reads as the in-scattered mass's mean depth.
    float depth_weight_sum = 0.0;
    float weighted_depth_sum = 0.0;
    // `real_samples` is the charged budget — density evaluations only. `iter` is a hard
    // safety cap so a ray that skips through empty space the entire way still
    // terminates; skipping no longer eats into the sample budget the way it used to; a
    // clear-sky pixel that never finds a cloud now costs one coarse fetch per iteration
    // instead of silently forfeiting most of its budget to the skip itself.
    int real_samples = 0;
    int max_iterations = STEPS * 4;
    for (int iter = 0; iter < max_iterations && real_samples < STEPS; ++iter)
    {
        if (transmittance < 0.02 || t >= t_end)
            break;
        float dist_step = clamp(0.08 * sqrt(max(t - t0, 0.0)), seg_min, seg_max);
        vec3 p = ro + rd * t;
        float height01, unused_profile;
        float coarse = cloud_density(p, true, height01, unused_profile);
        if (coarse > 0.001)
        {
            ++real_samples;
            float camera_dist = t - t0;
            float profile = 0.0;
            float density;
            if (t < lod_distance)
            {
                density = cloud_density(p, false, height01, profile);
                if (density > 0.001 && camera_dist < NEAR_BAND_METERS)
                    density *= near_field_erosion(p, height01);
            }
            else
            {
                density = coarse;
            }
            if (density > 0.001)
            {
                float sigma = density * extinction_scale;
                // The amortized light volume lights every lit sample for free (one
                // texture fetch); the costlier inline cone march only layers a
                // correction on top at a tier-controlled cadence (0 = never, 3 = every
                // lit sample) for the near-field shadow detail the volume's
                // 8-frame-lagged, 256x256x32 resolution cannot carry.
                float light_depth = cloud_light_volume_sample(p, height01);
                int tap_stride = budget.light_taps == 0u ? 0 : budget.light_taps == 1u ? 4 :
                                 budget.light_taps == 2u ? 2 : 1;
                if (tap_stride > 0 && (lit % tap_stride) == 0)
                    light_depth = mix(light_depth, cloud_light_march(p, sun), 0.6);
                sun_energy = cloud_sun_energy(light_depth, mu, g, extinction_scale,
                                              budget.light_taps >= 3u);
                ++lit;
                float powder = 1.0 - exp(-density * 2.0);
                powder = mix(1.0, powder, scene.cloud_light.z);

                vec3 sunlight = sun_radiance * sun_energy * powder;
                // Profile-gradient ambient (Nubis3): a sample near its own cloud's edge
                // (profile -> 0) reads brighter (closer to the open sky it borders) than
                // one buried in the middle of a thick deck (profile -> 1) — dark edges
                // and inner glow fall out of data the field bake already carries, no
                // extra fetch.
                vec3 ambient = ambient_color * scene.cloud_light.w * pow(1.0 - profile, 0.5);
                vec3 luminance = sunlight + ambient;

                float sample_transmit = exp(-sigma * dist_step);
                float in_scatter_weight = transmittance * (1.0 - sample_transmit);
                scattered += in_scatter_weight * luminance;
                depth_weight_sum += in_scatter_weight;
                weighted_depth_sum += in_scatter_weight * t;
                transmittance *= sample_transmit;
            }
            t += dist_step;
        }
        else
        {
            // Nubis3 step rule: `step = max(skip_distance, 0.08*sqrt(dist), min_step)`.
            // The coarse probe just proved this whole skip cell is empty (it is a
            // max-pool over the fine field, so a zero reading means zero everywhere
            // inside it), so the march can cross the entire cell in one hop instead of
            // a fixed multiplier that either under- or over-shoots it.
            t += max(dist_step, cloud_skip_distance(p));
        }
    }

    // W6 in-cloud whiteout tuning (design doc §7): deep inside a thick deck, `transmittance`
    // bottoms out near the loop's own 0.02 break threshold while `scattered` keeps
    // accumulating ambient-dominated luminance sample after sample, because `cloud_sun_energy`'s
    // Beer term has already collapsed the direct-sun contribution at that optical depth. The
    // result reads as flat, direction-blind fog rather than a real whiteout: a pilot deep in
    // cloud loses forward visibility but not the sense of where the sun is (a hazy-bright glow
    // looking toward it, comparably dim looking away) -- multiple scattering isotropizes with
    // depth but keeps a residual forward bias, unlike the single-scatter term above which
    // correctly extinguishes. `whiteout` ramps in exactly where that directional cue is
    // otherwise lost (`transmittance` below ~0.25) and blends the accumulated colour toward a
    // glow keyed on `mu` (view/sun angle), instead of leaving the interior a uniform tone.
    float whiteout = 1.0 - smoothstep(0.02, 0.25, transmittance);
    if (whiteout > 0.0)
    {
        float forward_bias = pow(clamp(mu * 0.5 + 0.5, 0.0, 1.0), 3.0);
        vec3 glow = sun_radiance * mix(0.05, 0.25, forward_bias) + ambient_color * 0.6;
        scattered = mix(scattered, glow, whiteout);
        // The interior itself also deepens toward fully opaque as the whiteout takes over, so
        // a dense-enough deck reads as a wall of cloud rather than merely thick haze once the
        // directional glow above is doing the readability work instead of the vanishing
        // transmittance channel.
        transmittance = mix(transmittance, 0.0, whiteout * 0.5);
    }

    out_color = vec4(scattered, transmittance);
    // No in-scattered mass means nothing meaningful to haze; fall back to the march's own
    // end distance rather than zero so a stray sample never reads as "infinitely close".
    out_depth = depth_weight_sum > 1e-5 ? (weighted_depth_sum / depth_weight_sum) : t_end;
}
