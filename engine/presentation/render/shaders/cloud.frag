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

// The Hillaire transmittance LUT, on the shared SceneLayout slot (24) SkyPass already writes.
// This is not one of the six free per-pass image bindings the note below is about — it is a
// named slot, and CloudPass now writes it exactly the way SkyPass does.
//
// **Why the march needs it.** `scene.sun_color * scene.sun_dir.w` is the *top of atmosphere*
// beam. Every other consumer of the sun in this renderer attenuates it for itself before use;
// the cloud march was the one that never did, so the sun that lit a deck was identical at
// midnight and at noon. The ephemeris only ever writes the sun's *direction* — its radiance is
// an authored constant — so nothing upstream dims it either. That is the whole of "evening
// clouds are not this white": the sky goes black because sky.frag integrates this LUT, and the
// deck stays noon-white because the march did not.
layout(set = 0, binding = 24) uniform sampler2D transmittance_lut;

// The sky-view LUT, on the shared slot (26) SkyPass writes. This is the sky's own radiance per
// direction — the thing that actually lights a cloud once the sun has gone. A deck at civil
// twilight is grey and readable because the *sky* is still bright, not because the sun is; a
// flat authored `scene.ambient` constant can carry neither that magnitude nor its colour.
// Stored per unit sun radiance, exactly like the aerial volume, so it is scaled by the
// top-of-atmosphere beam rather than by the attenuated one (sky.frag:557 does the same).
layout(set = 0, binding = 26) uniform sampler2D sky_view_lut;
// The T3-baked cloudscape *envelope* (CloudscapeCompilePass, reshaped by CloudsV2 —
// docs/slop/atmosphere_system.md §7.6): r = coverage envelope, g = the contributing deck's
// vertical profile, a = in-cloud water amplitude (half-encoded). The cloud's actual shape is
// no longer in any texture: this march carves it out of the envelope analytically, per
// sample, in the same world-anchored pattern frame the bake evaluates its weather in — so
// the shape's effective resolution is this march's own sample spacing, not a bake lattice.
// `cloudscape_skip` is the coarser max-pooled envelope*water product the cheap/coarse probe
// reads. Both cover the **near window** — a camera-centred, non-wrapping span of world
// (§7.2), no longer a periodic tile; `cloudscape_far` carries the same simulated structure
// out past the horizon at a coarser texel, plus its own optical-depth-toward-the-sun channel,
// for the part of the march that leaves the near window.
// Bindings 2-6: the shared SceneLayout reserves 7/8/9/10 by name (MATERIAL_BINDING,
// MOTION_BINDING, TEMPORAL_BINDING, SHADOW_BINDING — see scene_layout.hpp) as a
// different descriptor type each; only 1-6 are genuinely free per-pass image slots, so
// every image this pass owns outright lives there instead. The far field fits because the
// weather field itself no longer needs one: the bake resolves coverage per column now (§7.4),
// so the march reads the answer rather than the meteorology.
layout(set = 0, binding = 2) uniform sampler3D cloudscape_field;
layout(set = 0, binding = 3) uniform sampler3D cloudscape_skip;
// The precombined march noise volume (Textures::CloudNoise::march, cloud_noise_volume.comp
// kind 4): r = CDF-uniformised Nubis base shape, g = the three-octave erosion fbm,
// b = an incommensurate fine-erosion fbm for the fly-through octave, a = a Perlin potential
// for the curl warp at cloud bases. One volume because this pass has exactly one free noise
// binding — which is the constraint that shaped CloudsV2's texture layout.
layout(set = 0, binding = 4) uniform sampler3D cloud_march_noise;
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
#include "atmosphere_common.glsl"
#include "synoptic_field.glsl"

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
    uint steps_far;     // far-field sampling density: scales the march's angular step
                        // rate, so one tier means one density in every direction
    uint light_steps;   // self-shadow steps toward the sun
    // The field's own geometry used to ride here as a wrap period and a skip cell. It moved
    // into the scene block (`cloud_field_*`) when the field became a camera-centred window,
    // because the light volume, the shadow map, the panorama and the ground shadow all need
    // the same mapping and only this pass has a push block to put it in.
    uint light_taps;    // tiered inline light-march correction cadence, 0-3 (Low..Ultra)
    uint near_far_split; // W3: QualityParams::cloud_near_far_split, High/Ultra only
    uint carve_mip_count; // levels the march noise volume carries; 0 disables both of these
    // The NDC step between two neighbouring rows of *this pass's own* buffer, which runs at
    // the tier's cloud resolution scale rather than at the frame's. Pushed because that scale
    // is a tier setting and the shader has no other way to learn it; it turns into the
    // sample's lateral world footprint through the camera basis, which is half of what the
    // carve's LOD is selected against.
    float pixel_ndc;
    uint padding; // std430 aligns the vec4 array below to sixteen
    // Per-mip standard deviation of the carve shape that level's filter removed, measured
    // from the generated volume at start-up (Textures::CloudNoise::measure_carve_spread).
    // `carve_shape` integrates the coverage threshold over it, which is what lets a fetch
    // from a coarse level still deliver the right amount of cloud. Eight entries, level
    // order; entry 0 is zero by construction.
    vec4 carve_spread[2];
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
    return cloud_window_weight(near_uv, scene.cloud_field_params.x);
}

float remap(float v, float a, float b, float c, float d)
{
    return c + (v - a) / (b - a) * (d - c);
}

// The altitude of p above the reference *ellipsoid*. One place, so no reader of this shader can
// reintroduce the latitude banding by measuring against the sphere — see
// `cloud_planet_radius_at`, which carries the whole argument.
float cloud_altitude_at(vec3 p, out vec3 up)
{
    return cloud_planet_altitude(p, scene.planet_center.xyz, scene.planet_frame.xyz,
                                 scene.planet_radii.x, scene.planet_radii.z, up);
}

// The shell-relative height of p, and its altitude above the surface via the out param.
float cloud_shell_height(vec3 p, out float altitude)
{
    vec3 up;
    altitude = cloud_altitude_at(p, up);
    return clamp((altitude - scene.cloud_global.y) /
                     max(scene.cloud_global.z - scene.cloud_global.y, 1.0),
                 0.0, 1.0);
}

// ---- The planet-scale far field (§7.5) ------------------------------------------------------
//
// Both windows are camera-centred flat squares in world XZ — 32 km and 262 km — and both fade
// to nothing across their rim by design; cloud_field_window.glsl explains why a flat prism is
// meaningless at planetary viewing distance. Past the far one there was **nothing at all**,
// which from a hundred kilometres up is a 262 km square of weather sitting on an otherwise
// cloudless planet. That is the box in the report.
//
// What covers the rest of the globe is deliberately coarse and analytic, and the reason is that
// there is no data to be finer with: the regional nest is 384 km across and the global core
// runs on the host, so structure invented out there would be detail claiming to be weather. So
// the planet-scale field is **the deck stack's own envelope** — the same authored or compiled
// sky both windows are baked from, evaluated with `cloud_height_gradient` and unioned exactly
// the way the bake unions it — modulated by a planetary-scale pattern standing in for the
// weather map the bake has and this pass does not. The windows then sit *on top of* it as
// detail rather than as the only cloud in existence, and the rim fade becomes a hand-off
// instead of an edge.
//
// Two named limits. The pattern is anchored in the world frame rather than the body-fixed one
// (no prime-meridian angle reaches this shader), so on a rotating body the planetary cloud
// pattern does not turn with the ground; and it does not advect with the wind, which over a
// session moves a 300 km feature by well under a per cent of itself.

// How many times the carve volume tiles across the planet's radius. Five puts the pattern's
// dominant feature near 300 km — the scale of a real synoptic cloud mass, and what a weather
// satellite image reads as.
const float GLOBE_PATTERN_TILES = 5.0;
// How far the pattern may cut into the coverage the weather gives. **Multiplicative**, unlike
// the additive swing this replaces, and the difference is the whole of what a planet looks like
// from orbit: a multiplier cannot manufacture cloud where the synoptic field says the sky is
// clear, so a subtropical high stays genuinely empty instead of being veiled back over by
// noise. What it still does is break a synoptic mass into the mesoscale pieces a satellite
// image shows, which is a scale the placement itself does not carry.
const float GLOBE_PATTERN_AMPLITUDE = 0.9;
// The level the pattern is read at. At five tiles across the planet's radius, level 2 of a
// 128^3 volume is a ~40 km texel — an eighth of the pattern's own feature, so its shape
// survives while the fine octaves that would only alias out here do not.
const float GLOBE_PATTERN_LOD = 2.0;
// How far the march may hop on a zero reading where the globe field is the source. Unlike the
// windows' skip volume this is **a bound, not a proof**: the globe envelope is a point value,
// not a max-pool, so nothing here proves a region empty. What makes the bound safe is the
// pattern's own scale — 8 km is a fortieth of a 300 km feature, and the deck stack's vertical
// extent is bounded by the shell the probe already tests against.
const float GLOBE_SKIP_METERS = 8000.0;

// The deck stack's envelope at an altitude, plus the metres of its own deck standing above the
// sample (which is the only self-shadow term available out here, where no light volume and no
// baked sun-depth channel reach).
float cloud_globe_envelope(vec3 p, float altitude, out float water, out float profile,
                           out float deck_height, out float above_top)
{
    water = 0.0;
    profile = 0.0;
    deck_height = 0.0;
    above_top = 0.0;

    int decks = int(scene.cloud_global.w);
    if (decks <= 0)
        return 0.0;

    // A unit vector into a volume that tiles seamlessly: no projection, so no rim to distort,
    // which is the whole reason this field can cover a sphere where a flat window cannot.
    vec3 radial = p - scene.planet_center.xyz;
    vec3 unit_radial = normalize(radial);
    vec3 sp = unit_radial * GLOBE_PATTERN_TILES;
    float pattern = textureLod(cloud_march_noise, sp, GLOBE_PATTERN_LOD).r;

    // **Where the weather actually is.** This is what turned the planet from a uniformly milky
    // sphere into something that reads like an orbital photograph, and the reason is that the
    // deck stack it used to read cannot answer the question: `cloud_deck_a[i].z` is the coverage
    // compiled from the *camera's own column*, so using it out here restated the weather over
    // the observer's head as the weather everywhere on the body. The synoptic field is defined
    // at every point on the planet, and — the half that matters — it reaches **zero**, which is
    // what a subtropical high looks like and what most of an orbital photograph is made of.
    bool placed = scene.synoptic_params.z >= 0.5;
    float synoptic_convective = 0.5;
    float synoptic = 0.0;
    if (placed)
        synoptic = synoptic_coverage(unit_radial, scene.planet_frame.xyz, synoptic_convective);

    float envelope = 0.0;
    for (int i = 0; i < decks && i < 6; ++i)
    {
        vec4 la = scene.cloud_deck_a[i]; // base, top, coverage, density
        if (la.w <= 0.0 || la.y <= la.x || altitude < la.x || altitude > la.y)
            continue;
        float height01 = (altitude - la.x) / max(la.y - la.x, 1.0);
        float hg = cloud_height_gradient(height01, scene.cloud_deck_b[i].x,
                                         scene.cloud_deck_d[i].x);
        if (hg <= 0.0)
            continue;
        // The placement is one number per column and a deck stack has three étages, so the same
        // split `SeededWeather::column_at` applies on the simulation side is applied here — and
        // it is stated in both places rather than shared because the two live on opposite sides
        // of the render seam. A convective sky is broken at its middle level (towers with gaps),
        // a stratiform one is a sheet at both; cirrus has a floor everywhere plus whatever anvil
        // deep convection throws off. With no placement published it falls back to the deck's
        // own coverage, which is all there is to fall back to.
        float coverage = la.z;
        if (placed)
        {
            // The étage ceilings are `CLOUD_LEVEL_LOW_CEILING_METERS` and its mid counterpart,
            // so a deck lands in the same bucket the simulation would have put it in.
            float deck_mid = 0.5 * (la.x + la.y);
            // The cirrus term's constant used to be an additive 0.12 floor, which is what a
            // *mean* cirrus cover looks like and not what a cleared sky looks like: it survived
            // every high the seed could place, so the one region meant to read as empty ocean
            // kept a permanent veil across it and the globe stayed milky. Scaling the floor by
            // the placement instead keeps ordinary skies capped with thin cirrus while letting
            // a subsiding high take the whole column — including its top — to nothing, which is
            // the state a subtropical high actually produces.
            coverage = deck_mid > 6000.0
                           ? synoptic * (0.30 + 0.25 + 0.45 * synoptic_convective)
                           : (deck_mid > 2500.0 ? synoptic * (1.0 - 0.45 * synoptic_convective)
                                                : synoptic);
        }
        // Multiplicative, so a clear sky stays clear — see GLOBE_PATTERN_AMPLITUDE.
        coverage = clamp(coverage * mix(1.0 - GLOBE_PATTERN_AMPLITUDE, 1.0, pattern), 0.0, 1.0);
        float e = coverage * hg;
        if (e <= 0.001)
            continue;
        // The bake's own union (fold_envelope), so the two agree where they meet.
        envelope = clamp(envelope + e * (1.0 - envelope), 0.0, 1.0);
        water = max(water, la.w);
        if (hg >= profile)
        {
            deck_height = height01;
            above_top = la.y - altitude;
        }
        profile = max(profile, hg);
    }
    return envelope;
}

// The globe field's self-shadow: the slant path through its own deck to that deck's top,
// weighted by how much cloud is in it. Crude by construction — it knows nothing about the
// cloud next door — but it is what keeps a planet seen from orbit from reading as a uniformly
// lit sheet, and it costs no fetch.
float cloud_globe_light_depth(vec3 p, vec3 sun, float envelope, float water, float above_top)
{
    vec3 radial = p - scene.planet_center.xyz;
    vec3 up = radial / max(length(radial), 1.0);
    // Floored so a sun on the horizon gives a long but finite path rather than a divide.
    float slant = above_top / max(dot(up, sun), 0.15);
    return slant * envelope * water * CLOUD_ENVELOPE_MEAN_SHAPE;
}

// The coarse probe: the most cloud mass the carve could possibly leave at p. Reads the
// max-pooled envelope*water product inside the near window and the far field's own product
// past it, where a zero *proves* the carve yields zero (the carve only removes) — which is
// what makes empty-space skipping and the cone light march safe on this alone.
//
// Past both windows the planet-scale field answers instead, and there the guarantee weakens to
// a bound: that field is evaluated at a point, not pooled over a region, so a zero says only
// that this point is clear. `cloud_skip_exit` pays for the difference with a fixed hop tied to
// the pattern's own scale rather than with a lattice exit.
float cloud_probe(vec3 p)
{
    float altitude;
    float h = cloud_shell_height(p, altitude);
    // A metre of margin, because this early-out has to answer for a *region*, not a point.
    // The march begins exactly on the base sphere, where float32 at planetary radius resolves
    // to about half a metre — so on a coin flip the first probe of every ray would return zero
    // without consulting the field, and the empty-space hop that follows would cross the
    // bottom of the deck on the strength of an answer no texel was asked for. Inside the
    // margin the clamped fetch answers honestly for the shell's first row instead.
    if (altitude < scene.cloud_global.y - 1.0 || altitude > scene.cloud_global.z + 1.0)
        return 0.0;

    vec2 near_uv;
    float near_weight = cloud_near_blend(p, near_uv);
    vec2 far_uv = cloud_window_uv(scene.cloud_field_far.xy, scene.cloud_field_far.zw, p.xz);
    float far_weight = cloud_window_weight(far_uv, scene.cloud_field_params.y);

    // The planet-scale field is the base every window sits on. Evaluated only where a window
    // does not already answer in full, because it is the one term here that costs a loop.
    float density = 0.0;
    if (near_weight < 1.0 && far_weight < 1.0)
    {
        float globe_water;
        float globe_profile;
        float globe_deck;
        float globe_above;
        density = cloud_globe_envelope(p, altitude, globe_water, globe_profile, globe_deck,
                                       globe_above) *
                  globe_water;
    }
    // Each layer is skipped where the one above it already answers in full, so a sample deep
    // inside the near window still costs exactly one fetch.
    if (far_weight > 0.0 && near_weight < 1.0)
    {
        vec4 coarse = texture(cloudscape_far, vec3(far_uv, h));
        density = mix(density, coarse.r * cloud_field_water(coarse.a), far_weight);
    }
    if (near_weight > 0.0)
        density = mix(density, texture(cloudscape_skip, vec3(near_uv, h)).r, near_weight);
    return density;
}

// The envelope at p: where cloud may exist (return), how much water fills it (`water`), and
// the vertical profile for the ambient term (`profile`). Three sources, layered coarsest
// first — the planet-scale field, then the far window over it, then the near window over that
// — each blended in by its own rim weight, so a window's edge is detail arriving rather than
// cloud arriving. Both windows are baked from the same weather field, so they agree about
// where the weather is; the planet-scale field agrees with them in the *mean*, because it is
// built from the same deck stack, and differs only in the structure it cannot know about.
float cloud_envelope(vec3 p, float height01, out float water, out float profile,
                     out float deck_height)
{
    water = 0.0;
    profile = 0.0;
    // Falls back to the shell height where no near-window texel carries the real value. The
    // far window's b lane is the sun depth, not this — but the carve's erosion, the only
    // consumer, has faded out entirely by the time the far window takes over, so the
    // fallback is never the value anything shapes a cloud with. See cloudscape_field.comp's
    // store and cloud_field_window.glsl.
    deck_height = height01;

    vec2 near_uv;
    float near_weight = cloud_near_blend(p, near_uv);
    vec2 far_uv = cloud_window_uv(scene.cloud_field_far.xy, scene.cloud_field_far.zw, p.xz);
    float far_weight = scene.cloud_field_params.y > 0.0
                           ? cloud_window_weight(far_uv, scene.cloud_field_params.y)
                           : 0.0;

    // The planet-scale base. Where neither window answers in full this is what the sky *is*;
    // where one does, it is what the window's rim fades into, which is why the far window no
    // longer fades toward zero the way it did.
    float envelope = 0.0;
    if (near_weight < 1.0 && far_weight < 1.0)
    {
        vec3 globe_up;
        float altitude = cloud_altitude_at(p, globe_up);
        float globe_deck;
        float globe_above;
        envelope = cloud_globe_envelope(p, altitude, water, profile, globe_deck, globe_above);
        deck_height = envelope > 0.0 ? globe_deck : height01;
    }
    // Guarded on the far span, not just the rim weight: reading profile and water from a
    // never-baked volume would mix garbage into whatever it blends with. And skipped entirely
    // where the near window already answers in full, so the common case stays one fetch.
    if (far_weight > 0.0 && near_weight < 1.0)
    {
        vec4 coarse = texture(cloudscape_far, vec3(far_uv, height01));
        envelope = mix(envelope, coarse.r, far_weight);
        profile = mix(profile, coarse.g, far_weight);
        water = mix(water, cloud_field_water(coarse.a), far_weight);
        // coarse.b is the far window's sun depth, never a deck height — keep whatever the
        // globe or the shell fallback said rather than reading a lane that means something
        // else out there.
    }
    if (near_weight > 0.0)
    {
        vec4 fine = texture(cloudscape_field, vec3(near_uv, height01));
        envelope = mix(envelope, fine.r, near_weight);
        profile = mix(profile, fine.g, near_weight);
        water = mix(water, cloud_field_water(fine.a), near_weight);
        deck_height = mix(deck_height, fine.b, near_weight);
    }
    return envelope;
}

// Camera-relative XZ -> the world-anchored, wind-advected pattern frame the envelope was
// baked in, which is therefore the frame the carve must evaluate its noise in: carve in any
// other frame and the shapes would stand somewhere other than their own envelope — glued to
// the camera, or left behind by the wind. Reconstructed through the near window's linear
// (unclamped) map wherever that window exists; the two windows share one pattern frame, so
// either mapping names the same world point and the far one only matters before the near
// window's first bake.
vec2 cloud_pattern_xz(vec3 p)
{
    if (scene.cloud_field_params.x > 0.0)
    {
        vec2 uv = cloud_window_uv(scene.cloud_field_near.xy, scene.cloud_field_near.zw, p.xz);
        return scene.cloud_field_pattern.xy + uv * scene.cloud_field_params.x;
    }
    vec2 uv = cloud_window_uv(scene.cloud_field_far.xy, scene.cloud_field_far.zw, p.xz);
    return scene.cloud_field_pattern.zw + uv * scene.cloud_field_params.y;
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
        depth += cloud_probe(p + sun * (t + step_len * 0.5) + offset) * step_len;
        t += step_len;
        step_len *= 2.1;
    }
    // The probe reads the envelope's mass ceiling; the mean-shape factor restates the depth
    // against the mass the carve actually leaves standing, matching what the baked light
    // volume and the far light channel already fold in on their side (CloudsV2).
    return depth * CLOUD_ENVELOPE_MEAN_SHAPE;
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
// Past both windows neither source exists, so the third term is the globe field's own analytic
// slant depth — without it a planet seen from orbit would be lit as a flat unshadowed sheet,
// which is the one thing a cloud layer never looks like.
float cloud_light_volume_sample(vec3 p, float ambient_h, vec3 sun)
{
    vec2 near_uv;
    float near_weight = cloud_near_blend(p, near_uv);
    vec2 far_uv = cloud_window_uv(scene.cloud_field_far.xy, scene.cloud_field_far.zw, p.xz);
    float far_weight = cloud_window_weight(far_uv, scene.cloud_field_params.y);

    float depth = 0.0;
    if (near_weight < 1.0 && far_weight < 1.0)
    {
        float altitude;
        cloud_shell_height(p, altitude);
        float globe_water;
        float globe_profile;
        float globe_deck;
        float globe_above;
        float globe_envelope = cloud_globe_envelope(p, altitude, globe_water, globe_profile,
                                                    globe_deck, globe_above);
        depth = cloud_globe_light_depth(p, sun, globe_envelope, globe_water, globe_above);
    }
    if (far_weight > 0.0 && near_weight < 1.0)
    {
        float far_depth =
            cloud_far_sun_depth_decode(texture(cloudscape_far, vec3(far_uv, ambient_h)).b);
        depth = mix(depth, far_depth, far_weight);
    }
    if (near_weight > 0.0)
        depth = mix(depth, texture(cloud_light_volume, vec3(near_uv, ambient_h)).r, near_weight);
    return depth;
}

// ---- Band-limiting the carve ---------------------------------------------------------------
//
// The carve reads one volume at world scales fixed in metres, from a march whose own step
// grows with distance. Nothing reconciled those two: every tap was an implicit-LOD `texture()`
// against a volume with a single mip level and a sampler clamped to it, so a sample a hundred
// kilometres out — whose integration step spans a kilometre — point-sampled an eighteen-metre
// texel. That is not a cloud, it is whichever noise peak the lattice happened to land on, and
// at full amplitude through the threshold it is the reported "distant clouds render white".
//
// (An implicit LOD would have been wrong here even with a chain. A fragment shader derives it
// from screen-space derivatives of the coordinate, which are undefined inside the non-uniform
// control flow a ray march is made of. Every tap below is `textureLod` with a stated level.)
//
// What the previous code had instead was `carve_scale = max(carve_scale, footprint * 4)`: a
// band limit applied to the *feature size* rather than to the sampling. It kept the sampler
// inside its Nyquist limit — by making distant cumuli three to five kilometres across, at full
// solidity, which is a wall rather than a cloud. That floor is gone; the mip chain does the job
// it was standing in for, and distant clouds keep the size the meteorology gave them.

// The march volume's extent, mirrored from Textures::CloudNoise's MARCH_RESOLUTION. The same
// kind of mirrored engine constant as CLOUD_SKIP_RESOLUTION_Y in cloud_field_window.glsl, and
// for the same reason: it is a fixed engine size rather than a tiered setting, and the push
// block carries the tiered ones.
const float CARVE_NOISE_RESOLUTION = 128.0;

// **Which footprint.** A sample has two of them and they differ by more than an order of
// magnitude: the *lateral* one is the pixel's own cone (about 0.001 of the distance at 1080p),
// the *axial* one is the integration step (0.02 of it at the High tier). Neither alone is the
// answer.
//
//   * Filtering to the axial step is what the quadrature can integrate, and it is safe — but
//     it throws away detail the screen could plainly show. Looking straight down from orbit it
//     is badly wrong: the ray crosses a 2 km deck almost perpendicular, so along-ray error is
//     bounded by the deck's own thickness, while the lateral footprint on the deck is under a
//     hundred metres. Band-limiting that to a kilometre erases the entire cloud field.
//   * Filtering to the lateral cone keeps everything the screen can resolve and hands the
//     difference to the quadrature as error — which past JITTER_FREEZE_METERS is *not*
//     averaged away, because the dither is frozen there on purpose, so it would read as stable
//     structure that is not cloud.
//
// So the LOD is selected against the isotropic filter with the same *volume* as the real
// anisotropic footprint — `pow(lateral^2 * axial, 1/3)`, the standard equal-volume compromise.
// At the horizon it lands nearer the step, looking down from orbit nearer the pixel, and in
// both cases within a factor of a few of the honest answer. The ladder's Nyquist fade keeps
// using the step alone, because that test really is about the quadrature (see `cloud_billow`).

// The level whose texel spans the march's own footprint at a tap of world scale @p scale.
float carve_lod(float scale, float footprint)
{
    float levels = max(float(budget.carve_mip_count), 1.0);
    float texel = max(scale, 1.0) / CARVE_NOISE_RESOLUTION;
    return clamp(log2(max(footprint, 1.0) / texel), 0.0, levels - 1.0);
}

// The spread of the sub-texel shape a fetch at @p lod no longer carries. Interpolated across
// the two levels the hardware itself blends, so it tracks the fetch rather than a rounding of
// it.
float carve_spread_at(float lod)
{
    int last = int(budget.carve_mip_count) - 1;
    if (last <= 0)
        return 0.0;
    int lower = clamp(int(floor(lod)), 0, last);
    int upper = min(lower + 1, last);
    return mix(budget.carve_spread[lower >> 2][lower & 3],
               budget.carve_spread[upper >> 2][upper & 3],
               clamp(lod - float(lower), 0.0, 1.0));
}

// Cheap curl-noise warp (a divergence-free field built from central differences of a
// scalar potential) so cloud bases roll and billow instead of eroding along the noise's
// own straight gradient — the same visual cue Dagor/Nubis3 use to sell a base as
// "boiling" rather than merely fuzzy. The potential is the march volume's a channel;
// `scale` is its sample frequency.
vec3 curl_from_scalar(vec3 p, float scale, float lod)
{
    float e = scale * 0.05;
    float nx1 = textureLod(cloud_march_noise, (p + vec3(e, 0.0, 0.0)) / scale, lod).a;
    float nx2 = textureLod(cloud_march_noise, (p - vec3(e, 0.0, 0.0)) / scale, lod).a;
    float nz1 = textureLod(cloud_march_noise, (p + vec3(0.0, 0.0, e)) / scale, lod).a;
    float nz2 = textureLod(cloud_march_noise, (p - vec3(0.0, 0.0, e)) / scale, lod).a;
    return vec3(nz1 - nz2, 0.0, -(nx1 - nx2)) * (scale * 0.5);
}

// Nubis3's "jitter animated only < 250 m, static hash beyond": past this distance the
// march's per-pixel dither phase is frozen instead of animated frame to frame, so a
// distant silhouette's sample pattern stays put and the cloud TAA's history never has to
// re-converge on a moving dither — the near/far split's actual contribution to "no
// shimmer under motion" at distance.
const float JITTER_FREEZE_METERS = 250.0;

// How much of the sky's own radiance a cloud sample receives, averaged over the zenith and the
// sun's azimuth at the horizon. A deck is illuminated by the whole hemisphere, so the honest
// factor is closer to pi times a mean radiance; this is deliberately below that because the
// march's ambient term is a stand-in for multiple scattering that the octave ladder in
// `cloud_sun_energy` already accounts for part of, and doubling it would brighten every
// daylight cloud. Calibrated so the daylight level lands near the `sun_radiance * 0.02` proxy
// this replaces, which is what the sky was balanced against.
const float CLOUD_SKY_AMBIENT = 1.0;

// The same, for the reflected bodies. 1.0 is the ephemeris' own physically derived irradiance,
// already scaled by the author's `NightLighting::reflected_intensity`; there is no second
// artistic knob here on purpose. Moonlight really is about six orders of magnitude under
// sunlight, so a physically exposed moonlit deck is *dim* — if a brighter night is wanted, the
// place to say so is that setting, not a constant buried in a shader.
const float CLOUD_REFLECTED_AMBIENT = 1.0;

// The skylight a deck receives as a fraction of the top-of-atmosphere beam, in full daylight.
// Not a new number: `CLOUD_SKY_AMBIENT` above states that its measured form was calibrated to
// land near the `sun_radiance * 0.02` proxy it replaced, so this is that same calibration
// reused where the measurement cannot reach. It has to be reused rather than measured because
// the sky-view LUT is a directional map of the *camera's own* sky: a sample a thousand
// kilometres away is not somewhere it can be asked about. See the march's `sample_ambient`.
const float CLOUD_REMOTE_SKY_AMBIENT = 0.02;

// Half the Sun's angular diameter, radians. Near the horizon a change in zenith cosine equals
// the change in angle to first order, so this doubles as the width of the terminator's
// penumbra: the Sun is a disc, not a point, and a deck goes dark over the ~0.5 degree its limb
// takes to set rather than in one step. Without it a planet-scale march draws the terminator as
// a razor line across the cloud tops, which is the one place on Earth nobody has ever seen one.
const float SUN_ANGULAR_RADIUS = 0.00466;

// The zenith cosine at which a point's own horizon sits. Solving
// `atmo_ray_sphere(radius, mu, surface_radius) == 0` for mu gives exactly this, so a gate built
// on it agrees with the ray test it replaces — it is only expressed as a cosine, so the solar
// disc's penumbra can be applied without a trigonometric round trip.
float cloud_horizon_mu(float radius, float surface_radius)
{
    float ratio = surface_radius / max(radius, 1.0);
    return -sqrt(max(1.0 - ratio * ratio, 0.0));
}

/** How much of a disc of the Sun's apparent size clears the horizon at a point, [0, 1]. */
float cloud_horizon_visibility(float mu, float radius, float surface_radius)
{
    float mu_horizon = cloud_horizon_mu(radius, surface_radius);
    return smoothstep(mu_horizon - SUN_ANGULAR_RADIUS, mu_horizon + SUN_ANGULAR_RADIUS, mu);
}

/**
 * The sun that reaches a given point, rather than the one that reaches the camera.
 *
 * **Why this is per sample.** The block in `main` that this replaces took the solar zenith
 * cosine in the *camera's* radial frame, and argued in its own comment that the angle at the
 * deck and the angle at the observer "differ by the deck's angular extent about the planet
 * centre — under a degree for anything this march can reach — which is finer than the LUT
 * resolves". Both halves of that are now false.
 *
 * The first half died when the planet-scale field landed: the march reaches the whole visible
 * globe, tens of degrees of arc, with a real terminator inside the frame. The second half was
 * never true at the only moment anyone looks at it. The horizon test is a *step*, so at sunset
 * a degree is not a rounding error — it is the entire difference between a lit deck and a black
 * one, and no amount of LUT resolution changes that. A camera standing in its own shadow was
 * killing the direct beam on every cloud in the frame, including the ones plainly in sunlight
 * a hundred kilometres to the west, which is exactly what was reported.
 *
 * @param mu     The solar zenith cosine at the sample, `dot(up_p, sun)`.
 * @param radius The sample's geocentric radius, metres.
 * @param toa    The top-of-atmosphere beam, before any atmospheric path.
 * @return       The beam that survives the slant path to this point.
 */
vec3 cloud_sun_at(float mu, float radius, vec3 toa, AtmosphereMedium medium,
                  float surface_radius)
{
    float lit = cloud_horizon_visibility(mu, radius, surface_radius);
    if (lit <= 0.0)
        return vec3(0.0);
    // Clamped to the horizon for the fetch: below it the LUT's parameterisation folds back and
    // returns the upward answer, and inside the penumbra we want the grazing path, not that.
    return toa * lit * sample_transmittance(transmittance_lut, medium, radius,
                                            max(mu, cloud_horizon_mu(radius, surface_radius)));
}

/**
 * The share of full skylight surviving at solar elevation @p mu.
 *
 * A shape, stated as one rather than dressed up as a measurement: full daylight by about seven
 * degrees of solar elevation, gone by about ten degrees below the horizon, which is roughly
 * where nautical twilight ends and a deck stops being lit by anything but the Moon.
 */
float cloud_daylight(float mu)
{
    return smoothstep(-0.18, 0.12, mu);
}

// ---- The two step sizes ---------------------------------------------------------------
//
// Searching for cloud and integrating one are different jobs with different error terms, and
// the old march gave them a single shared `seg_min`. That one constant could not be right for
// both, and being wrong for the second is most of why a cloud read as a smear:
//
//   * A search step costs one skip-volume fetch. Its only error is overshooting a cloud's
//     leading edge, and `cloud_skip_exit` below bounds that exactly — so it should be as long
//     as the emptiness that was proven, and nothing else should shorten it.
//   * An integration step costs the whole carve, a light lookup, a phase evaluation and an
//     exponential. Its error is the quadrature error of the transmittance integral, which is
//     precisely what decides whether a cloud reads as a body or as a smudge.
//
// `seg_min` was `max(shell_thick * 0.12, 40)` — twelve per cent of the *union* of every
// enabled deck, which is 1344 m in the default 800..12 000 m sky. A fair-weather cumulus is
// about 1.5 km across, so every cloud in the sky was integrated by roughly *one* sample. No
// amount of carving detail survives that: the shape is evaluated, multiplied by a kilometre
// of path length, and folded into the transmittance as a single slab.
//
// The rate is angular, so the sample spacing is a constant fraction of a screen pixel and the
// error is uniform across the frame rather than concentrated at whatever distance a metric
// step size happened to disagree with. It is also where the tier's `steps_far` knob now lands:
// that knob used to divide the *march length*, so the same tier resolved a horizon ray at 5 km
// per sample and a ray straight up at 500 m — the step depended on where the ray happened to
// leave the shell rather than on the quality setting. A rate is scale-free, so one tier means
// one sampling density in every direction. Calibrated at the reference below and scaled
// inversely, which puts the shipped tiers at 0.040 / 0.027 / 0.020 / 0.013.
const float MARCH_STEP_ANGULAR = 0.020;
const float MARCH_ANGULAR_REFERENCE_STEPS = 32.0;
// The floor binds below ~1 km, where crossing the first kilometre of a deck costs at most
// fifty samples — affordable against the tier budgets (8..160) — and above which the angular
// rate takes over on its own.
const float MARCH_STEP_MIN_METERS = 20.0;
// The ceiling replaces the old `march_len / far_steps`, which reached 5 km on a horizon ray:
// past about a kilometre the step exceeds any cloud the field can hold, so the integral stops
// resolving cloud and starts resolving deck.
const float MARCH_STEP_MAX_METERS = 1200.0;
// Only to guarantee forward progress: a hop that starts exactly on a lattice boundary computes
// an exit of zero and would otherwise spin. This is the one length the march does advance
// unverified, which is why it is half a metre — sixty-four float32 ULP at 100 km, and four per
// cent of the smallest integration step.
const float SEARCH_ADVANCE_EPSILON_METERS = 0.5;

// ---- Spending the budget without truncating the ray -----------------------------------------
//
// `steps_near` is a *cost* bound. It used to be spent as a *domain* bound: the loop ran while
// `real_samples < STEPS` and simply stopped, leaving the rest of the ray — however much cloud
// stood along it — contributing nothing. That is the reported hard edge in the middle of the
// sky, and the reason it slides across the frame as the camera moves: which pixels exhaust
// first is a function of where the eye is, so the boundary is anchored to the observer rather
// than to anything in the world. A pixel one step to the left of it marched its whole ray and
// its neighbour stopped a kilometre out, and nothing between the two values interpolates.
//
// It bit hardest exactly where the user found it — close to the camera. The budget is charged
// per *envelope* sample, and the envelope is nonzero across far more sky than the carve ever
// fills, so ordinary hazy-but-clear air is charged full price. With the near step floored at
// MARCH_STEP_MIN_METERS the first 500 m of a ray costs 25 of the tier's samples outright, and
// the cheap tier's 48 are gone by ~1.2 km — so an observer standing under a deck could see the
// far field fine and the air in front of their face not at all.
//
// A cost bound should degrade *resolution*, not reach. Past the budget the march therefore
// coarsens geometrically instead of stopping: each overdrawn sample doubles the step every
// 1/rate samples, so the remaining distance is crossed in a bounded handful of extra samples
// and the ray always terminates on its own `t_end`. The quadrature stays honest while it does —
// `carve_shape` already integrates the threshold over the sample's own footprint and converges
// to the carve's exact mean yield as that footprint grows, which is the same mechanism the
// CARVE_END_METERS hand-off relies on. So a coarsened sample reads as a lower-resolution
// version of the cloud that is there, which is what a quality tier is supposed to look like,
// rather than as its absence.
//
// **The coarsening is driven by a distance, not by a sample count, and that is the whole
// design.** The first version of this computed `exp2(0.5 * (real_samples - STEPS))` — geometric
// in the number of samples already spent. It removed the hard edge it was written for and
// introduced a subtler artifact in its place: `real_samples` is an *integer*, so the step size
// took discrete values, and two adjacent pixels whose rays happened to land on different
// overdraft counts marched at step sizes a factor of sqrt(2) apart. The accumulated density
// differs visibly across that boundary, and the set of pixels sharing an overdraft count is a
// curve of constant chord length through the shell — which, seen from orbit, is a **circle
// centred on the sub-camera point**. Concentric rings, exactly as reported.
//
// The lesson generalises past this shader: anything derived from a loop counter is quantised by
// construction, and a quantised quantity that varies across the frame is a visible contour
// unless something downstream smooths it. A march has nothing downstream to smooth it.
//
// So the budget is expressed as the *distance* at which it would run out, which is a closed form
// because the natural step is geometric — `t_{n+1} = t_n * (1 + march_angular)` — and therefore
// continuous in `t`. Past that distance the step scales by how far past it the sample is. Step
// size stays a smooth function of position along the ray, so no contour of any shape can appear.
//
// Growth is quadratic overall (the natural step is already proportional to `t`, and this
// multiplies it by `t` again), so the remaining reach is crossed in a few dozen samples and the
// ray always terminates on its own `t_end` rather than being cut short.

// ---- Empty-space skipping -----------------------------------------------------------------
//
// How far the march may advance once the coarse probe has proved the block at p empty.
//
// The old answer was the block's *side length*, floored by the current step: `t += max(
// dist_step, cell)`. Both halves of that were unsafe. A hop of one side length starting from a
// point already part-way through a block lands up to a full block *past* that block's exit
// face, so it can cross an entirely untested block; and with the old seg_min at 1344 m against
// a 512 m near cell, a near-horizontal ray hopped 2.6 blocks on the strength of one probe,
// every step of the way to the horizon. That is not a conservative skip, it is a distant cloud
// that exists or not depending on where the ray happened to be when it last looked.
//
// The exact answer is the distance to the boundary of the region the fetch actually proved
// empty. Two details make that region well defined:
//   * The probe is a *trilinear* fetch, so a zero reading proves the eight texels surrounding
//     p are all zero (the values are non-negative, so nothing can cancel). Those eight bound
//     the cell of the dual lattice — offset half a texel from the stored one — which is where
//     the `- 0.5` below comes from. Marching to a primal-cell boundary would be conservative
//     but needlessly short by up to half a cell.
//   * The probe mixes both windows wherever they overlap, so the safe hop is the shorter of
//     the two lattices' exits, not the one belonging to whichever weight is larger.

// Distance along the ray until a lattice coordinate crosses its next integer boundary.
float cloud_axis_exit(float coord, float rate)
{
    if (abs(rate) < 1e-9)
        return 1e30; // parallel to this axis' slabs: they never bound the hop
    float boundary = rate > 0.0 ? floor(coord) + 1.0 : floor(coord);
    return (boundary - coord) / rate;
}

// The exit distance from one window's probe lattice. @p cell_xz is that lattice's horizontal
// cell in metres (published per frame, since it follows the window span) and @p res_y its
// vertical texel count (a mirrored engine constant; see cloud_field_window.glsl).
float cloud_lattice_exit(vec2 uv, vec3 p, vec3 rd, float span, float cell_xz, float res_y)
{
    if (span <= 0.0 || cell_xz <= 0.0)
        return 1e30; // window not baked; it bounds nothing

    // uv = xz / span + offset, so d(uv)/dt = rd.xz / span and, in texel units,
    // d(coord)/dt = rd.xz * (span / cell_xz) / span = rd.xz / cell_xz — the window's own
    // placement cancels out, which is why no per-frame lattice origin is needed here.
    vec2 coord = uv * (span / cell_xz) - 0.5;
    float exit_x = cloud_axis_exit(coord.x, rd.x / cell_xz);
    float exit_z = cloud_axis_exit(coord.y, rd.z / cell_xz);

    // The vertical axis is shell-relative, exactly as cloud_probe addresses it. Over a hop of
    // a few hundred metres the local up vector is fixed to well under a millimetre of altitude
    // at planetary radius, so the instantaneous rate is the exact one at this scale — and by
    // the same argument the ellipsoid's own radius is constant over the hop, so the rate below
    // needs no term for how it varies with latitude.
    vec3 up;
    float altitude = cloud_altitude_at(p, up);
    float shell = max(scene.cloud_global.z - scene.cloud_global.y, 1.0);
    float h = (altitude - scene.cloud_global.y) / shell;
    float exit_y = cloud_axis_exit(h * res_y - 0.5, dot(rd, up) * res_y / shell);

    return min(exit_x, min(exit_z, exit_y));
}

float cloud_skip_exit(vec3 p, vec3 rd)
{
    vec2 near_uv;
    float near_weight = cloud_near_blend(p, near_uv);
    vec2 far_uv = cloud_window_uv(scene.cloud_field_far.xy, scene.cloud_field_far.zw, p.xz);
    float far_weight = cloud_window_weight(far_uv, scene.cloud_field_params.y);

    // Where the globe field is the source there is no lattice to leave, only a bound (see
    // GLOBE_SKIP_METERS). Each window's exit applies only while that window is the thing
    // being read: a far lattice cell is ~1 km, and letting it bound the hop out where the far
    // window contributes nothing would make a ray cross a hundred kilometres of empty planet
    // one kilometre at a time and run out of iterations before it reached anything.
    float hop = GLOBE_SKIP_METERS;
    if (far_weight > 0.0)
        hop = min(hop, cloud_lattice_exit(far_uv, p, rd, scene.cloud_field_params.y,
                                          scene.cloud_field_params.w, CLOUD_FAR_RESOLUTION_Y));
    if (near_weight > 0.0)
        hop = min(hop, cloud_lattice_exit(near_uv, p, rd, scene.cloud_field_params.x,
                                          scene.cloud_field_params.z,
                                          CLOUD_SKIP_RESOLUTION_Y));
    return hop;
}

// ---- The CloudsV2 analytic carve ---------------------------------------------------------
//
// This is where a cloud gets its shape, at every distance — the successor to both the bake's
// texel-bound carve (whose 128 m lattice could never hold a cauliflower edge) and the old
// march's <200 m near-band erosion (which left everything past 200 m untextured). Every scale
// is in world metres in the pattern frame, so the shapes stand still in the world and advect
// with the wind, exactly like the envelope.
//
// **The operator, not the noise.** Every version of this carve until now built a cloud by
// *subtraction*: threshold a base field, then remove material where an erosion noise is high.
// That can only ever produce concave features — bites, channels, holes — because subtracting
// from a solid is how you dig. A cumulus congestus top is the exact opposite: a bunch of
// *convex* protrusions, self-similar bulging turrets packed against each other, with the
// darkness living in the creases *between* the bulges. No amount of tuning an erosion strength
// produces that, because it is the wrong sign. It is why the tops always read either as smooth
// lumps (weak erosion) or as shredded filaments (strong erosion), and never as cauliflower.
//
// So the ladder below is *added to the field before the threshold*, which displaces the
// isosurface outward where the ladder is high. That is a protrusion, not a bite. The polarity
// is already available for free: `noise_worley` returns `1 - distance`, so the march volume's
// g channel is high at cell centres and low at cell walls — round bumps with creases between
// them, exactly the primitive a cauliflower is made of. The old code took `1 - g` and
// subtracted it, converting those round bumps into round *holes*.
//
// Two regimes, split by the sample's height within its own cloud:
//   * Below the convective base — the condensation level, a near-planar boundary, which is why
//     every photograph shows a cumulus base as flat — the carve stays subtractive and tears
//     the underside into hanging shreds.
//   * Above it, the billow ladder displaces, and the ladder's own signed value doubles as the
//     self-occlusion term that makes the creases read dark. Without that the cauliflower is
//     geometrically present but shades as flat white, because the light volume's 128 m texels
//     cannot resolve a 100 m crease and never will.

// The carve's base wavelength. Coverage-adaptive below (1/sqrt(envelope)), so cloud *count*
// falls with coverage instead of cloud size: a 10 % sky is a few full-sized cumuli, not
// a field of specks — the same rule the bake's carve established.
const float CARVE_BASE_SCALE_METERS = 2400.0;
// The base-tearing octave: the scale of the shreds hanging under a cloud.
const float CARVE_WISP_SCALE_METERS = 280.0;

// ---- The billow ladder --------------------------------------------------------------------
// The largest turret's *sample* scale, as a multiple of carve_scale. Channel g's dominant cell
// is about a quarter of whatever world scale it is sampled at (it is a Worley fbm whose
// frequency-4 term carries the bulk of the weight), so 1.15 puts the biggest turret at ~0.29 of
// a cloud's own width — roughly a third, which is what a photographed congestus shows.
const float BILLOW_TOP_SCALE_RATIO = 1.15;
// The dominant Worley cell of the volume's erosion channels, as a fraction of the world scale
// they are sampled at. Shared by every octave's Nyquist test below.
const float CARVE_FEATURE_RATIO = 0.25;
const int BILLOW_OCTAVES = 3;
const float BILLOW_LACUNARITY = 0.45;
const float BILLOW_GAIN = 0.5;
// 1 + gain + gain^2: the ladder's full weight, used as a *fixed* normaliser so an octave
// dropping out of the band limit shrinks the relief continuously instead of renormalising the
// survivors and popping.
const float BILLOW_NORM_FULL = 1.75;
// Channel g's mean. It is an inverted Worley F1 field at unit feature density, whose expected
// distance-to-nearest puts this at approximately one half; subtracting it is what keeps the
// displacement zero-mean, so the threshold still delivers the coverage the envelope asked for.
// Whatever residual bias the estimate carries shifts the carve's mean yield by well under a
// per cent, which is inside what CLOUD_ENVELOPE_MEAN_SHAPE already reconciles.
const float BILLOW_MEAN = 0.5;
// How far the ladder may push the isosurface, in units of the base field. **This is the knob
// that decides "lump" versus "cauliflower", and it is larger than it looks.**
//
// The ladder is a deviation about zero, not a [0, 1] field, and its spread is small: an
// inverted Worley F1 has a standard deviation around 0.18, the volume's minor terms take that
// to ~0.15, and stacking three octaves at 1/0.5/0.25 against a fixed 1.75 normaliser leaves
// the ladder itself at roughly 0.10. `base` sweeps [0, 1] across about 0.68 * carve_scale of
// world, so on a 2400 m cloud one unit of the base field is ~1630 m of surface: 1.6 turns a
// typical 0.10 deviation into ~250 m of relief, which is a turret about a tenth of a cloud
// wide, and the ladder's coarsest octave stacks those into the ~700 m bulges the reference
// photograph shows.
//
// The 0.18 is an estimate, not a measurement — this shader cannot sample its own noise
// statistics. BILLOW_MAX_DISPLACE below is what makes that safe: if the real spread is twice
// what is assumed, the clamp bounds both the turret size and the coverage distortion instead
// of letting a rare excursion swallow the threshold. If the relief reads too timid or too
// violent, this and CREASE_SENSITIVITY are the two numbers to move, together.
const float BILLOW_RELIEF = 1.6;
// The hard bound on displacement, in base-field units. The threshold sits at `1 - envelope`,
// so a displacement approaching 1 could push a clear column solid or a solid one clear; 0.35
// keeps the coverage the envelope asked for recognisable at every envelope the bake produces.
const float BILLOW_MAX_DISPLACE = 0.35;
// An octave whose dominant cell spans fewer than this many march steps is aliasing, not detail.
// Faded rather than dropped, so the ladder thins continuously with distance.
//
// The mip chain now band-limits every tap, so this is no longer the *only* thing standing
// between the ladder and the lattice — but it is still load-bearing, for two reasons. It skips
// fetches that could not contribute (the loop breaks on it), and it is what keeps a fully
// filtered octave from contributing a *bias*: a filtered tap converges to the channel's true
// mean, and the ladder subtracts BILLOW_MEAN, an estimate of that mean. Where the two differ
// the residual would be a constant displacement — a uniform coverage shift over the whole far
// field — instead of averaging out across neighbouring samples the way it does at full detail.
const float BILLOW_MIN_STEPS = 2.0;
// Where the convective regime takes over from the flat base, in the sample's height within its
// own cloud.
const float BILLOW_BASE_HEIGHT = 0.12;
const float BILLOW_FULL_HEIGHT = 0.42;
// How sharply the ladder's signed value reads as a crease. Against the ~0.10 spread derived
// above, 4 maps an ordinary crease to roughly 0.4 of the occlusion range and saturates only in
// the deepest gaps between turrets.
const float CREASE_SENSITIVITY = 4.0;
// The self-shadowing a crease stands for, as extra optical depth toward the sun in the same
// metres the light volume reports. This is the term that turns a white blob into a readable
// cauliflower, and it exists because a 256x32x256 light volume over a 32 km window has 128 m
// texels and physically cannot resolve the creases the ladder just created.
const float CREASE_OCCLUSION_METERS = 400.0;
// Creases see less of the sky dome too, not only less sun.
const float CREASE_AMBIENT_OCCLUSION = 0.45;
// How hard the threshold's uniform ramp is pushed toward a saturated core. 2.5 leaves the top
// 60 % of a cloud's own cross-section at full density with a soft rim over the remaining 40 %,
// which is what a photographed cumulus does. The support — and therefore the delivered
// coverage — is untouched at any value above 1.
const float CLOUD_CARVE_SOLIDITY = 2.5;
// The fly-through octave: the wisps an aircraft actually punches through.
const float CARVE_FINE_SCALE_METERS = 140.0;
// Where each erosion band hands off. The main erosion fades over ~14-42 km (past that a
// 620 m feature is sub-pixel anyway); the fine octave and the curl warp only exist near the
// camera, where the march's own steps are dense enough to sample them.
const float DETAIL_FADE_START_METERS = 14000.0;
const float DETAIL_FADE_END_METERS = 42000.0;
const float FINE_DETAIL_END_METERS = 2200.0;
const float CURL_END_METERS = 2500.0;
const float DETAIL_EROSION_STRENGTH = 0.42;
const float FINE_EROSION_STRENGTH = 0.30;
// Past this the carve stops entirely and the sample stands in for the carved sky with its
// statistical mean (envelope * water * CLOUD_ENVELOPE_MEAN_SHAPE) — mass-consistent by
// construction, so the horizon keeps the same brightness the carved mid-field has, and the
// hand-off is faded over the last quarter so no ring marks it.
//
// It is now a *cost* cut rather than a correction. With the threshold integrated over the
// detail the mip filter removed (`carve_shape`), the carve converges to this same mean on its
// own as the footprint grows — 0.8 of the envelope's water against the 0.75 this constant
// states, the difference being the erosion, which has faded out by 42 km. So the two sides of
// the fade agree to within a few per cent by construction instead of by calibration, and what
// is saved past 80 km is every one of the carve's texture fetches.
const float CARVE_END_METERS = 80000.0;

// The signed billow ladder: self-similar round protrusions, returned as a deviation about zero
// so it can be added to the base field without moving the threshold's expected coverage.
// Positive is the outside of a turret, negative is the crease between two of them.
float cloud_billow(vec3 sp, float scale, float footprint, float step)
{
    float deviation = 0.0;
    float amp = 1.0;
    float s = scale;
    for (int o = 0; o < BILLOW_OCTAVES; ++o)
    {
        // Faded against the *step*, not the LOD footprint: this test asks whether the march's
        // samples can resolve the octave along the ray at all, which is a question about the
        // quadrature and not about the texture filter.
        float fade = clamp(s * CARVE_FEATURE_RATIO / (step * BILLOW_MIN_STEPS) - 1.0,
                           0.0, 1.0);
        if (fade <= 0.0)
            break; // this octave and every finer one are below the march's own Nyquist limit
        deviation += amp * fade *
                     (textureLod(cloud_march_noise, sp / s, carve_lod(s, footprint)).g -
                      BILLOW_MEAN);
        amp *= BILLOW_GAIN;
        s *= BILLOW_LACUNARITY;
    }
    return deviation / BILLOW_NORM_FULL;
}

// ---- The threshold, integrated over the detail the filter removed ---------------------------
//
// The carve keeps the top `envelope` of a field that is uniform on [0, 1] — that uniformity is
// what makes a threshold at `1 - envelope` deliver exactly the coverage the meteorology asked
// for, and cloud_noise_volume.comp pushes the channel through its own CDF to guarantee it.
//
// A *filtered* fetch is not that field. Averaging narrows a distribution: at the coarser levels
// the field concentrates around its mean, so a hard threshold against it answers "all cloud" or
// "no cloud" for a whole region, and the delivered coverage stops resembling the requested one
// entirely. Band-limiting the sampling without this correction would trade one distant-white
// failure for another.
//
// What is missing is the *sub-footprint* detail, and the mip chain says exactly how much of it
// there is: a level is a box average over disjoint blocks, hence a conditional expectation,
// hence an orthogonal projection, so the variance splits with no cross term and the residual is
// `Var(level 0) - Var(level l)` — measured on the host and pushed as `carve_spread`.
//
// So instead of evaluating the ramp at the filtered mean, evaluate its *expectation* over that
// residual. Modelled as uniform rather than Gaussian because the underlying field is uniform,
// which also makes the integral a closed form rather than an erf. Two properties earn it:
//
//   * At spread 0 it is the ramp itself, so the near field is bit-for-bit what it was.
//   * As the filter takes everything, the mean goes to 0.5 and the spread to the field's own,
//     and the result converges to `envelope * (1 - 1/(2 * solidity))` — the carve's exact mean
//     yield. The horizon therefore lands on the same mass CLOUD_ENVELOPE_MEAN_SHAPE states,
//     which is what every sun-depth integral in the frame already assumes, and the hand-off at
//     CARVE_END_METERS stops being a cross-fade between two different answers.

// Antiderivative of the clamped ramp `clamp((x - threshold) / width, 0, 1)`.
float carve_ramp_integral(float x, float threshold, float width)
{
    float d = x - threshold;
    if (d <= 0.0)
        return 0.0;
    if (d >= width)
        return d - 0.5 * width; // past the ramp: unit slope, offset by the ramp's own area
    return 0.5 * d * d / width;
}

float carve_shape(float mean, float threshold, float width, float spread)
{
    // The half-width of the uniform distribution with this standard deviation.
    float half_width = spread * 1.7320508;
    if (half_width < 1e-4)
        return clamp((mean - threshold) / width, 0.0, 1.0);
    return (carve_ramp_integral(mean + half_width, threshold, width) -
            carve_ramp_integral(mean - half_width, threshold, width)) /
           (2.0 * half_width);
}

// @p footprint is what the LOD selection band-limits to (the equal-volume mean of the lateral
// and axial footprints); @p step is the integration step alone, which is what decides whether
// an octave can be *sampled* at all and is therefore what the ladder's Nyquist fade uses.
float cloud_density_carved(vec3 p, float altitude, float deck_height, float camera_dist,
                           float footprint, float step, float envelope, float water,
                           out float crease)
{
    crease = 0.0;
    if (envelope <= 0.004 || water <= 0.004)
        return 0.0;
    float mean_density = envelope * water * CLOUD_ENVELOPE_MEAN_SHAPE;
    if (camera_dist >= CARVE_END_METERS)
        return mean_density;

    vec2 pattern = cloud_pattern_xz(p);
    vec3 sp = vec3(pattern.x, altitude, pattern.y);

    float carve_scale = CARVE_BASE_SCALE_METERS * inversesqrt(clamp(envelope, 0.12, 1.0));
    // No footprint floor on the scale any more — see the band-limiting note above
    // `carve_lod`. Keeping the sampler inside its Nyquist limit by inflating the *feature*
    // is what turned every distant cumulus into a three-to-five kilometre solid block; the
    // mip chain keeps it inside that limit by filtering instead, and a cloud a hundred
    // kilometres away is now the same size as the one overhead.
    float base_lod = carve_lod(carve_scale, footprint);
    float base_spread = carve_spread_at(base_lod);

    // A domain warp first, so cloud edges are not the noise's own straight gradients. The
    // g/b/a channels are mutually decorrelated fbm/potential fields, which is all a warp
    // vector needs.
    //
    // The amplitude has to stay well under the warped field's own wavelength or the map stops
    // being injective and folds. The base octave's dominant feature is about 0.68 * carve_scale
    // (a frequency-4 field over a 2.7 * carve_scale tile), against which the old 0.55 gave a
    // peak displacement of 0.275 * carve_scale — a displacement-per-wavelength ratio of 0.41,
    // comfortably into the folding regime, and a folded warp pinches and shears its features
    // into exactly the sheared strands this carve was rejected for. 0.30 puts the ratio at
    // 0.22, which still breaks up the noise's own straight gradients without creasing them.
    float warp_scale = carve_scale * 2.7;
    vec3 warp = textureLod(cloud_march_noise, sp / warp_scale,
                           carve_lod(warp_scale, footprint)).gba - 0.5;
    vec3 wp = sp + warp * carve_scale * 0.30;

    // How convective this sample is: 0 on the flat condensation base, 1 in the turrets above.
    float cauliflower = smoothstep(BILLOW_BASE_HEIGHT, BILLOW_FULL_HEIGHT, deck_height);

    // The ladder, added to the field *before* the threshold so it displaces the isosurface
    // outward into a bulge rather than biting a hollow out of a finished shape. Zero-mean, so
    // the coverage the envelope asked for still arrives.
    float relief = 0.0;
    if (cauliflower > 0.0)
        relief = cloud_billow(sp, carve_scale * BILLOW_TOP_SCALE_RATIO, footprint, step) *
                 cauliflower;

    float displace = clamp(relief * BILLOW_RELIEF, -BILLOW_MAX_DISPLACE, BILLOW_MAX_DISPLACE);

    // The threshold and the solidity push, which used to be two steps with the crease term
    // between them. They compose into one clamped ramp — `min(clamp(d / envelope, 0, 1) *
    // solidity, 1)` is `clamp(d / (envelope / solidity), 0, 1)` — and writing it as one is what
    // lets the expectation over the detail the mip filter removed be taken in closed form.
    //
    // Why the solidity is there at all, kept on the record: `base` is CDF-uniformised at
    // generation (cloud_noise_volume.comp), which is what makes a threshold at `1 - envelope`
    // deliver exactly `envelope` of the volume — but it leaves the survivors uniform on [0, 1]
    // too, so a cloud would be a linear ramp from nothing to full across its whole width, with
    // only a tenth of its own volume above 0.9 and no plateau anywhere. A real cumulus is the
    // opposite: a broad, fully dense interior behind a sharp boundary. Narrowing the ramp
    // restores that without touching the support, so the coverage statistics the uniformisation
    // exists to guarantee are unaffected — the set of points with density > 0 is identical,
    // only its filling changes.
    float threshold = 1.0 - envelope;
    float width = max(envelope / CLOUD_CARVE_SOLIDITY, 1e-4);

    float base = textureLod(cloud_march_noise, wp / carve_scale, base_lod).r;
    float shape = carve_shape(base + displace, threshold, width, base_spread);
    if (shape <= 0.0)
        return 0.0;

    // The crease term. The same ladder value that pushed a turret out says, where it is
    // negative, that this sample sits in the gap between two of them — enclosed on both sides
    // and lit through more cloud than an exposed face. Only the negative half is used: a
    // protrusion is already fully lit and has nothing to gain.
    crease = clamp(-relief * CREASE_SENSITIVITY, 0.0, 1.0);

    float detail_fade = 1.0 - smoothstep(DETAIL_FADE_START_METERS, DETAIL_FADE_END_METERS,
                                         camera_dist);
    if (detail_fade > 0.0)
    {
        // The subtractive regime, and now the *only* place subtraction happens: the underside,
        // where a cloud really is being torn rather than pushed. It is weighted by
        // `1 - cauliflower`, so it stops exactly where the ladder takes over and the two never
        // fight over the same sample.
        //
        // Polarity matters here too. `g` is high at cell centres, so the old `wisp = g` removed
        // cell interiors and left their walls standing — a web of thin ridges, which is
        // literally what "tel tel kadayıf" describes and was the strand generator all along.
        // `1 - g` removes the connective tissue *between* centres instead, leaving separated
        // round shreds hanging off the base, which is what a torn cumulus underside looks like.
        //
        // Unlike the billow ladder this octave is deliberately *not* faded out where the march
        // can no longer resolve it, because its mean is not zero: it removes mass, and dropping
        // it would make cloud undersides grow denser with distance — the same "brighter the
        // further away it is" failure by another route. The mip fetch handles it correctly on
        // its own, converging to the channel's true mean and so to the erosion's mean bite.
        float rag_weight = (1.0 - cauliflower) * detail_fade;
        if (rag_weight > 0.0)
        {
            float rag = 1.0 - textureLod(cloud_march_noise, sp / CARVE_WISP_SCALE_METERS,
                                         carve_lod(CARVE_WISP_SCALE_METERS, footprint)).g;
            // Scaled by how far outside the cloud's own core the sample sits, so the tearing
            // reaches into the rim and leaves the interior standing (CV2's correction, kept).
            float bite = DETAIL_EROSION_STRENGTH * rag_weight * (1.0 - shape);
            shape = clamp(remap(shape, rag * bite, 1.0, 0.0, 1.0), 0.0, 1.0);
        }

        if (shape > 0.0 && camera_dist < FINE_DETAIL_END_METERS)
        {
            // The fly-through octave, on the incommensurate b channel so it never erodes an
            // edge the ladder or the rag already touched. Same corrected polarity.
            vec3 np = sp;
            if (deck_height < 0.35 && camera_dist < CURL_END_METERS)
            {
                float curl_scale = CARVE_WISP_SCALE_METERS * 2.9;
                np += curl_from_scalar(sp, curl_scale, carve_lod(curl_scale, footprint)) *
                      (1.0 - deck_height / 0.35);
            }
            float fine = 1.0 - textureLod(cloud_march_noise, np / CARVE_FINE_SCALE_METERS,
                                          carve_lod(CARVE_FINE_SCALE_METERS, footprint)).b;
            float near_fade = 1.0 - camera_dist / FINE_DETAIL_END_METERS;
            float fine_bite = FINE_EROSION_STRENGTH * near_fade * (1.0 - shape);
            shape = clamp(remap(shape, fine * fine_bite, 1.0, 0.0, 1.0), 0.0, 1.0);
        }
    }

    float carved = shape * water;
    float mean_mix = smoothstep(CARVE_END_METERS * 0.75, CARVE_END_METERS, camera_dist);
    return mix(carved, mean_density, mean_mix);
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

    // The saturating multiple-scatter floor, and the reason the post-march whiteout that
    // used to sit at the bottom of main() is gone.
    //
    // Every octave above carries the same Beer term, so all three decay to zero as the sun
    // depth grows and a deep interior converged on the ambient term alone — "flat,
    // direction-blind fog", which the whiteout then papered over with a colour keyed only
    // on dot(view, sun). That patch was gated on the *ray's total* transmittance rather
    // than on the camera being immersed, so at the cumulus default (sigma = 0.00279/m) it
    // replaced 50 % of the shading at 717 m of body and 100 % past 1.4 km: every
    // optically-thick cloud, seen from anywhere, lost its form and kept only its rim. That
    // is the "bright shredded edges around a dull featureless mass" this pass was rejected
    // for.
    //
    // The hole the patch covered is real, but it is a missing term, not a missing hack. A
    // water cloud's single-scattering albedo is ~0.999: a deep interior does not converge
    // to zero, it converges to a bright, nearly isotropic radiance. So add a term that
    // *rises* with sun depth where the octave ladder falls. It is keyed on this sample's
    // own `light_depth`, so a sample 200 m under the cloud top and one 2 km under it read
    // differently — the term carries form where the flat glow destroyed it — and it is
    // exactly zero at `light_depth == 0`, so a fully lit edge is unchanged by construction
    // and no recalibration of the octave ladder is needed.
    //
    // Multiple scattering isotropizes but keeps a residual forward bias, which is why a
    // pilot inside cloud still senses where the sun is. That is what the whiteout's `mu`
    // term was reaching for; here it falls out of a much softened phase function instead of
    // out of a blend factor.
    const float MS_FLOOR = 0.35; // deep-interior radiance as a fraction of a fully lit edge
    const float MS_RATE = 0.4;   // how fast the interior saturates, in units of extinction
    const float MS_ANISOTROPY = 0.2;
    const float MS_PHASE_WEIGHT = 0.35;
    float ms_phase = mix(1.0, phase_mie(mu, g * MS_ANISOTROPY) * 4.0 * PI, MS_PHASE_WEIGHT);
    energy += MS_FLOOR * ms_phase *
              (1.0 - exp(-light_depth * extinction_scale * MS_RATE));

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
    //
    // Kept as a separate `_toa` value because two different things want it. The *direct beam*
    // on the deck is this attenuated by the atmosphere (below). The *sky-view and aerial LUTs*
    // want it raw, because those store their in-scatter per unit top-of-atmosphere radiance and
    // carry the atmospheric path internally — sky.frag:557 scales the identical fetch the same
    // way. Multiplying a LUT fetch by the attenuated beam would apply the atmosphere twice.
    vec3 sun_radiance_toa =
        scene.sun_color.xyz * scene.sun_dir.w * (1.0 - 0.92 * scene.sky_counts.w);
    // ---- The sun that actually reaches the deck -------------------------------------------
    //
    // Everything above is the *top of atmosphere* beam: an authored colour times an authored
    // intensity, with the eclipse fraction as its only modifier. The ephemeris writes the sun's
    // direction and nothing else, so that product is the same number at midnight as at noon.
    // Every other consumer in the renderer attenuates it for itself — sky.frag folds this LUT
    // into its scattering integral, the fog and aerial-perspective volumes do the same — and
    // the cloud march was the one that never did. Hence a black sky with a snow-white deck in
    // front of it.
    //
    // **The attenuation itself is now per sample**, in `cloud_sun_at`, which carries the whole
    // argument for why the once-per-pixel version that used to live here had to go. What stays
    // here is only what genuinely belongs to the camera: the medium, and the two ambient terms
    // that are measured from the camera's own sky.
    //
    // Skipped when the atmosphere is switched off: `planet_radii.w` is its thickness, and a
    // zero-thickness medium makes the LUT's own parameterisation degenerate. With no
    // atmosphere there is nothing to attenuate the beam with, which is the honest answer.
    //
    // The same block derives the two things that light a deck once the sun is down, and both
    // are why a night sky is not simply black: the sky's own residual glow, and the Moon.
    vec3 sky_ambient = vec3(0.0);
    vec3 reflected_ambient = vec3(0.0);
    // Which way the reflected sum points, so the march can ask whether a given sample can see
    // it. Zero when there is nothing up there, which the gate reads as "visible" and multiplies
    // by a zero sum.
    vec3 reflected_dir = vec3(0.0);
    bool atmosphere_on = scene.planet_radii.w > 1.0;
    // The camera's own solar elevation. No longer used to light anything — only to say how much
    // of the skylight is already accounted for at a given sample, in the march's `sample_ambient`.
    vec3 camera_up = normalize(-center);
    float daylight_camera = cloud_daylight(dot(camera_up, sun));

    AtmosphereMedium sky_medium;
    sky_medium.rayleigh_scattering = scene.rayleigh.xyz;
    sky_medium.mie_scattering = scene.rayleigh.w;
    // The 1.1 absorption factor sky.frag applies, so the two agree about the same air.
    sky_medium.mie_extinction = scene.rayleigh.w * 1.1;
    sky_medium.rayleigh_scale_height = scene.scatter.y;
    sky_medium.mie_scale_height = scene.scatter.z;
    sky_medium.bottom_radius = surface_radius;
    sky_medium.top_radius = surface_radius + scene.planet_radii.w;

    if (atmosphere_on)
    {
        float mu_sun = dot(camera_up, sun);
        // Still the deck's mid-altitude here rather than a sample's: what remains in this block
        // is the camera's own sky and the reflected bodies, neither of which is evaluated per
        // sample. The sun left.
        float deck_mid_radius =
            surface_radius + 0.5 * (scene.cloud_global.y + scene.cloud_global.z);

        // ---- Skylight ----------------------------------------------------------------
        //
        // Two taps of the sky's own radiance, once per pixel: straight up, and toward the
        // horizon under the sun. Those two bracket what a deck actually receives — the
        // zenith carries the blue that lights a cloud top at midday, the sun's azimuth at
        // the horizon carries the whole of a twilight, which is where nearly all of the
        // light comes from in the minutes after sunset. This is the term that decides
        // whether an evening deck reads as grey and legible or as a silhouette.
        //
        // It replaces `sun_radiance * 0.02`, a flat 2 % of the beam that had no colour of
        // its own and, before the beam was attenuated, did not dim after dark either. The
        // LUT is stored per unit top-of-atmosphere radiance, so it is scaled by the raw
        // beam and lands at a similar daylight magnitude while gaining a real twilight.
        vec3 horizon_dir = sun - camera_up * mu_sun;
        float horizon_len = length(horizon_dir);
        // Degenerate only with the sun within a hair of the zenith, where the glow
        // direction is meaningless anyway and any azimuth is as good as another.
        horizon_dir = horizon_len > 1e-3 ? horizon_dir / horizon_len
                                         : normalize(cross(camera_up, scene.cam_right.xyz));
        sky_ambient = sun_radiance_toa * CLOUD_SKY_AMBIENT * 0.5 *
                      (sample_sky_view(sky_view_lut, center, camera_up, surface_radius) +
                       sample_sky_view(sky_view_lut, center, horizon_dir, surface_radius));

        // ---- The Moon, and every other reflecting body -------------------------------
        //
        // The ephemeris already turns each of them into a real directional light from its
        // albedo, radius, distance and phase (Astro::celestial_lights), sorted by what it
        // delivers here — so after sunset `lights[0]` is the Moon rather than the Sun, and
        // a crescent is dimmer than a full moon with no authored difference. None of it
        // reached the clouds, which is why they went perfectly black.
        //
        // Folded into the ambient rather than marched as a second self-shadowed beam, and
        // that is a physical choice rather than a shortcut: these sources are five to six
        // orders of magnitude weaker than the sun, so what survives to the eye is the
        // multiply-scattered part, which leaves an optically thick deck near-uniformly
        // grey. It is also what the sun's own light volume could not help with — that
        // volume is baked toward the sun, so below the horizon its depths describe a ray
        // travelling down out of the deck and would shadow moonlight with geometry that
        // has nothing to do with where the Moon is.
        // **Not gated on the camera's horizon**, for the reason `cloud_sun_at` gives at length:
        // a march that spans a planet has samples on both sides of one. The camera-anchored
        // `continue` that used to sit here is what made the night side of an orbital view go to
        // nothing — from a daylit camera the Moon is usually below the *camera's* horizon, so
        // the one light the dark limb had was skipped for every sample in the frame. The
        // magnitude is still taken once, at the deck's mid-altitude with the slant path clamped
        // to grazing; *which* samples see the body is decided per sample, in the march.
        float deck_horizon_mu = cloud_horizon_mu(deck_mid_radius, surface_radius);
        int light_count = int(scene.light_counts.x);
        float reflected_best = 0.0;
        for (int i = 0; i < light_count && i < 5; ++i)
        {
            // Lane 1's w flags a body that emits its own light. The Sun is the direct beam
            // above; counting it here as well would double it.
            if (scene.lights[i * 2 + 1].w > 0.5)
                continue;
            vec3 body_dir = scene.lights[i * 2 + 0].xyz;
            float body_mu = max(dot(camera_up, body_dir), deck_horizon_mu);
            vec3 contribution =
                scene.lights[i * 2 + 1].xyz * scene.lights[i * 2 + 0].w *
                sample_transmittance(transmittance_lut, sky_medium, deck_mid_radius, body_mu);
            reflected_ambient += contribution;
            // The direction the per-sample gate will use. One direction for the whole sum is an
            // approximation and a defensible one: the ephemeris sorts these by what they deliver
            // here, and after sunset the Moon outweighs everything behind it by orders of
            // magnitude, so the sum is the Moon to within the precision anyone can see.
            float weight = dot(contribution, vec3(1.0));
            if (weight > reflected_best)
            {
                reflected_best = weight;
                reflected_dir = body_dir;
            }
        }
        reflected_ambient *= CLOUD_REFLECTED_AMBIENT;
    }

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

    // The shell is oblate, like the ground under it. A spherical shell fitted at the observer's
    // own latitude misses the surface by up to 21 km everywhere else, which put the deck
    // underground across the tropics and in the stratosphere over the caps — see
    // `cloud_planet_radius_at`.
    vec2 outer = cloud_ray_shell(ro, rd, center, semi_major + scene.cloud_global.z,
                                 semi_minor + scene.cloud_global.z, planet_pole);
    if (outer.y <= 0.0)
        return;

    float march_ceiling = geometry_t;
    if (ground_hit)
        march_ceiling = min(march_ceiling, ground_t);
    float t0 = max(outer.x, 0.0);
    float t1 = min(outer.y, march_ceiling);

    // The camera sits below the deck (the ordinary ground-level case): the shell does
    // not start at the camera, it starts where the view ray first climbs to the cloud
    // base. A ray from inside the base shell always exits it, so `inner.y` is that
    // climb point. Without this the march burns its budget on the empty air underneath
    // the clouds before it ever reaches one.
    //
    // Tested on the camera's own *altitude* rather than by comparing its radius to a shell
    // radius, which is the same question asked in the frame that is actually right.
    vec3 camera_radial;
    float camera_altitude = cloud_altitude_at(ro, camera_radial);
    if (camera_altitude < scene.cloud_global.y)
    {
        vec2 inner = cloud_ray_shell(ro, rd, center, semi_major + scene.cloud_global.y,
                                     semi_minor + scene.cloud_global.y, planet_pole);
        if (inner.y > 0.0)
            t0 = max(t0, inner.y);
    }

    if (t1 <= t0)
        return;

    float extinction_scale = scene.cloud_light.x * 0.006;

    float march_len = t1 - t0;
    // The reach is an absolute distance, not a multiple of shell thickness. It used to be
    // fourteen shell thicknesses, calibrated when the shell was the genus catalogue's 10+ km
    // union and fourteen of it meant ~150 km. The nest publishes the span its condensate
    // actually occupies, and fourteen times a 1.3 km fair-weather deck is 19 km: the sky
    // became a camera-following bubble with the horizon empty past it — the far window was
    // baked, addressed, lit, and then never marched. The horizon is a distance.
    march_len = min(march_len, 160000.0);

    // Distance-driven march: step size grows with distance from the camera, not with the
    // ray's altitude — altitude picked the wrong pixels to starve, since a grazing ray at
    // cloud-base height paid the same cost as one looking straight up through the thinnest
    // part of the shell. See the step-size note above MARCH_STEP_ANGULAR for why searching
    // and integrating now use separate rules instead of one shared floor.
    int STEPS = clamp(int(budget.steps_near), 8, 160);
    int far_steps = clamp(int(budget.steps_far), 4, 160);
    float march_angular =
        MARCH_STEP_ANGULAR * MARCH_ANGULAR_REFERENCE_STEPS / float(far_steps);

    // Gradient noise, not a white-noise hash: the march offset's error spreads evenly
    // across neighbouring pixels instead of clumping into speckle, and the jitter-driven
    // phase decorrelates it frame to frame so the temporal resolve integrates it away.
    // W3 near/far split: beyond JITTER_FREEZE_METERS the phase is frozen at a fixed
    // mid-point instead, trading the frame-to-frame decorrelation (which the near-field
    // spatial dither already supplies at distance-scaled pixel footprints) for a stable
    // sample pattern the cloud TAA's history never has to re-converge on.
    bool freeze_jitter = budget.near_far_split != 0u && t0 > JITTER_FREEZE_METERS;
    float dither = freeze_jitter ? 0.5 : temporal_dither(gl_FragCoord.xy);
    // The interval currently being integrated, tracked separately from the sample inside it —
    // and the fix for the reported hole in front of the camera.
    //
    // The old march sampled at `t`, weighted by `dist_step`, then advanced by `dist_step`: a
    // left-endpoint rule, started at `t0 + seg_min * dither`. So the stretch between the
    // shell entry and the first sample was never integrated by anything, and with seg_min at
    // 1344 m that was up to 1.3 km of cloud immediately ahead contributing nothing at all —
    // the entire deck, when the camera is flying inside one. Jittering the sample *within* an
    // interval that starts exactly at t0 is the unbiased midpoint rule the old code was
    // reaching for, and it leaves no uncovered segment anywhere along the ray.
    float seg_start = t0;
    float t_end = t0 + march_len;

    float mu = dot(rd, sun);
    float g = scene.cloud_light.y;
    // The sample's lateral footprint per metre of distance. `cam_up` carries tan(fov/2), and
    // the ray is built from it over an NDC span of two, so one row of this pass's buffer
    // subtends exactly this angle.
    float pixel_rate = length(scene.cam_up.xyz) * budget.pixel_ndc;

    // The skylight the cloud's shadowed side sees, from three sources that each cover what the
    // others cannot: `scene.ambient` is the ephemeris' day-factor term and its starlight floor,
    // `sky_ambient` is the sky's measured radiance (the whole of a twilight), and
    // `reflected_ambient` is the Moon and anything else reflecting sunlight down.
    //
    // The `sun_radiance * 0.02` proxy this replaces was the bug's accomplice: with the beam
    // pinned at its noon value it was the *dominant* component after dark, so even a fully
    // self-shadowed cloud face kept a flat colourless glow — and once the beam was correctly
    // gated it went to exactly zero, taking the entire night sky's illumination with it, which
    // is the opposite failure. A cloudy evening is grey and legible; neither extreme is that.
    //
    // Only the first two are summed here. The sky-view LUT is a directional map *of the
    // camera's own sky*, so there is no honest way to ask it about a point a thousand
    // kilometres away and its tap stays where it was measured; the march adds the one skylight
    // correction that is visible, per sample, below. `reflected_ambient` is left out of this
    // sum entirely because it is the term the dark side of a planet is lit by, and whether it
    // reaches a sample is a question about that sample's horizon, not the camera's.
    vec3 ambient_color = scene.ambient.xyz + sky_ambient;

    float transmittance = 1.0;
    vec3 scattered = vec3(0.0);
    int lit = 0;
    float sun_energy = 0.0;
    // W3 cloud depth accumulation: the same (1 - sample_transmit) * transmittance
    // weight `scattered` folds radiance in by, applied to the sample's own distance
    // instead, so the normalized sum reads as the in-scattered mass's mean depth.
    float depth_weight_sum = 0.0;
    float weighted_depth_sum = 0.0;
    // The budget is a distance now, not a count. There was a `real_samples` counter here that
    // charged density evaluations; nothing reads such a count any more — the loop no longer ends
    // on it and the coarsening no longer derives from it — so it is gone rather than left
    // sitting there implying a limit that is not enforced. Skipping still costs one coarse fetch
    // per iteration and nothing else, which is what keeps a clear-sky pixel cheap.
    //
    // The distance at which the tier's budget would be exhausted marching at the natural rate.
    // Closed form because that rate is geometric: `t_{n+1} = t_n * (1 + march_angular)`, so
    // `STEPS` of them multiply the start by `(1 + march_angular)^STEPS`. Measured from where the
    // geometric regime actually begins — below `MARCH_STEP_MIN_METERS / march_angular` the floor
    // binds and the step is constant, so starting the product any earlier would understate the
    // reach. Continuous in `t0`, which is the property the whole thing exists for.
    float geometric_start = max(t0, MARCH_STEP_MIN_METERS / max(march_angular, 1.0e-4));
    float budget_reach = geometric_start * pow(1.0 + march_angular, float(STEPS));

    // The loop's only bound. `real_samples` used to end it too; now nothing does but the reach
    // and the iteration cap — a march that stops mid-ray draws a camera-anchored edge across
    // the sky.
    //
    // Sized against the *skip lattice*, not the sample budget: a clear horizon ray crosses
    // ~64 near cells and ~125 far ones, and capping iterations at STEPS * 4 gave the lowest
    // tier (8 steps) 32 hops — a hard, tier-dependent horizon at about a third of the way
    // out. Search iterations cost one fetch each, so the floor is cheap insurance that the
    // sky ends where the march says it ends and not where the budget ran out.
    int max_iterations = max(STEPS * 4, 256);
    for (int iter = 0; iter < max_iterations; ++iter)
    {
        if (transmittance < 0.02 || seg_start >= t_end)
            break;
        // The probe is taken at the interval's start, never at the jittered sample: the hop
        // below advances from exactly the point whose block was proved empty, so the segment
        // between them can never be skipped unexamined.
        vec3 probe_p = ro + rd * seg_start;
        float coarse = cloud_probe(probe_p);
        if (coarse > 0.001)
        {
            // Past the budget's reach the step scales by how far past it this sample is — a
            // smooth function of position along the ray, so it can draw no contour. The ceiling
            // scales with it too: left at its fixed 1200 m the coarsening would be clamped
            // straight back off and the ray still could not finish.
            //
            // The 1024 bound only keeps the products below away from the edges of float32 on a
            // pathological ray; the growth crosses any reachable distance long before it binds.
            float coarsen = min(max(seg_start / max(budget_reach, 1.0), 1.0), 1024.0);
            float dist_step = min(clamp(march_angular * seg_start * coarsen,
                                        MARCH_STEP_MIN_METERS,
                                        MARCH_STEP_MAX_METERS * coarsen),
                                  t_end - seg_start);
            float t = seg_start + dist_step * dither;
            seg_start += dist_step;
            vec3 p = ro + rd * t;
            float altitude;
            float height01 = cloud_shell_height(p, altitude);
            float water;
            float profile;
            float deck_height;
            float envelope = cloud_envelope(p, height01, water, profile, deck_height);
            // `t` is the true distance from the camera (the ray origin is the eye), which is
            // what the carve's detail bands are stated against. The footprint its LOD selects
            // against is the equal-volume mean of the two the sample actually has — see the
            // note above `carve_lod`.
            float lateral = t * pixel_rate;
            float footprint = pow(max(lateral * lateral * dist_step, 1.0), 1.0 / 3.0);
            float crease;
            float density = cloud_density_carved(p, altitude, deck_height, t, footprint,
                                                 dist_step, envelope, water, crease);
            if (density > 0.001)
            {
                float sigma = density * extinction_scale;
                // The amortized light volume lights every lit sample for free (one
                // texture fetch); the costlier inline cone march only layers a
                // correction on top at a tier-controlled cadence (0 = never, 3 = every
                // lit sample) for the near-field shadow detail the volume's
                // 8-frame-lagged, 256x256x32 resolution cannot carry.
                float light_depth = cloud_light_volume_sample(p, height01, sun);
                int tap_stride = budget.light_taps == 0u ? 0 : budget.light_taps == 1u ? 4 :
                                 budget.light_taps == 2u ? 2 : 1;
                // Only inside the near window, which is the only place it can improve on what
                // it is correcting: past that the light volume it refines does not exist, the
                // far window's own baked sun depth or the globe field's analytic one is
                // already the best answer, and each call is twelve probes — twelve deck-stack
                // evaluations out where the globe field is the source.
                bool near_field = t < scene.cloud_field_params.x * 0.5;
                if (tap_stride > 0 && near_field && (lit % tap_stride) == 0)
                    light_depth = mix(light_depth, cloud_light_march(p, sun), 0.6);
                // The creases between turrets, which neither source above can see: the light
                // volume's texels are 128 m and the cone march reads the same envelope, so a
                // 100 m gap between two billows is invisible to both. Added after the cone
                // mix, or the correction would dilute the very term it cannot supply.
                light_depth += crease * CREASE_OCCLUSION_METERS;
                sun_energy = cloud_sun_energy(light_depth, mu, g, extinction_scale,
                                              budget.light_taps >= 3u);
                ++lit;
                float powder = 1.0 - exp(-density * 2.0);
                powder = mix(1.0, powder, scene.cloud_light.z);

                // ---- The sun and the sky at *this sample* --------------------------------
                //
                // Both used to be one per-pixel value taken in the camera's radial frame, and
                // `cloud_sun_at` carries the argument for why that could not survive a march
                // that reaches the whole globe — or, as it turns out, a sunset.
                //
                // Deliberately inside the `density > 0.001` branch: the LUT fetch is paid only
                // by samples that will actually be shaded, which on a typical ray is a small
                // fraction of the ones stepped through.
                vec3 up_p = normalize(p - center);
                float mu_sun_p = dot(up_p, sun);
                float sample_radius = surface_radius + altitude;
                // With no atmosphere there is nothing to attenuate the beam with — but the
                // planet still casts a shadow, so the geometry half of this applies either way.
                vec3 sun_radiance =
                    atmosphere_on
                        ? cloud_sun_at(mu_sun_p, sample_radius, sun_radiance_toa, sky_medium,
                                       surface_radius)
                        : sun_radiance_toa * cloud_horizon_visibility(mu_sun_p, sample_radius,
                                                                      surface_radius);
                // The skylight the camera's taps cannot see. Only the *difference* is added, so
                // a sample no sunnier than the camera contributes exactly zero and every view
                // that was already right renders exactly as it did — what this rescues is the
                // deck that stands in daylight while the observer stands in shadow.
                //
                // The reflected term joins here rather than in `ambient_color` so it can be cut
                // off at this sample's own horizon: it is what keeps a night-side deck faintly
                // legible instead of absent, which is only true where the Moon is actually up.
                vec3 sample_ambient =
                    ambient_color +
                    reflected_ambient * cloud_horizon_visibility(dot(up_p, reflected_dir),
                                                                 sample_radius, surface_radius) +
                    sun_radiance_toa * CLOUD_REMOTE_SKY_AMBIENT *
                        max(cloud_daylight(mu_sun_p) - daylight_camera, 0.0);

                vec3 sunlight = sun_radiance * sun_energy * powder;
                // Profile-gradient ambient (Nubis3): a sample near its own cloud's edge
                // (profile -> 0) reads brighter (closer to the open sky it borders) than
                // one buried in the middle of a thick deck (profile -> 1) — dark edges
                // and inner glow fall out of data the field bake already carries, no
                // extra fetch.
                vec3 ambient = sample_ambient * scene.cloud_light.w * pow(1.0 - profile, 0.5) *
                               (1.0 - crease * CREASE_AMBIENT_OCCLUSION);
                vec3 luminance = sunlight + ambient;

                float sample_transmit = exp(-sigma * dist_step);
                float in_scatter_weight = transmittance * (1.0 - sample_transmit);
                scattered += in_scatter_weight * luminance;
                depth_weight_sum += in_scatter_weight;
                weighted_depth_sum += in_scatter_weight * t;
                transmittance *= sample_transmit;
            }
        }
        else
        {
            // The coarse probe just proved this block empty (it is a max-pool over the fine
            // field, and the carve only ever removes, so a zero reading means zero everywhere
            // inside it). Cross exactly that block — no further, or the hop crosses untested
            // sky; no shorter, or the clear half of the frame pays for nothing.
            seg_start += cloud_skip_exit(probe_p, rd) + SEARCH_ADVANCE_EPSILON_METERS;
        }
    }

    // The in-cloud whiteout that used to sit here is gone; `cloud_sun_energy`'s multiple-
    // scatter floor supersedes it. See the note there for why a term that rises with a
    // *sample's own* sun depth is not the same thing as a post-hoc blend keyed on the
    // *ray's* total transmittance — the latter cannot tell "the camera is inside a deck"
    // from "the camera is looking at a thick deck five kilometres away", and flattened
    // both. Opacity needs no help either: the march already integrated it honestly, and
    // the old `transmittance = mix(transmittance, 0.0, whiteout * 0.5)` was inflating it
    // to compensate for a density the carve was not delivering.

    out_color = vec4(scattered, transmittance);
    // No in-scattered mass means nothing meaningful to haze; fall back to the march's own
    // end distance rather than zero so a stray sample never reads as "infinitely close".
    out_depth = depth_weight_sum > 1e-5 ? (weighted_depth_sum / depth_weight_sum) : t_end;
}
