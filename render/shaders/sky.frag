#version 450
#extension GL_GOOGLE_include_directive : require

#include "temporal_common.glsl"
#include "shadow_sampling.glsl"
#include "atmosphere_common.glsl"

// The planet, atmosphere, clouds, and stars, drawn as one fullscreen ray march after
// the opaque meshes. Works in camera-relative space (the camera is the origin; the
// planet centre arrives relative to it in the scene block), so planet-scale metres
// never leave double precision on the CPU. For each pixel it:
//   * reconstructs the world-space view ray from the camera basis;
//   * reads the opaque depth to bound the march (geometry occludes sky, and thin air
//     is added over geometry as aerial perspective);
//   * intersects the WGS84 ellipsoid for the lit ground, and a spherical shell for the
//     atmosphere, integrating Rayleigh + Mie single scattering (Nishita-style);
//   * ray marches a procedural cloud layer lit by the sun;
//   * adds the real bright-star catalogue, faded by the atmosphere's optical depth —
//     so as the camera climbs out of the air, the blue sky thins into black space and
//     the true constellations emerge.
// Output is linear HDR; a later pass tonemaps it.

layout(set = 0, binding = 0) uniform SceneBlock
{
    mat4 view;
    mat4 proj;
    vec4 cam_forward;    // xyz = unit forward, w = camera pos x (unused here)
    vec4 cam_right;      // xyz = right * tan(fovx/2), w = camera pos y
    vec4 cam_up;         // xyz = up * tan(fovy/2), w = camera pos z
    vec4 planet_center;  // xyz = planet centre relative to camera, w = mean radius
    vec4 planet_radii;   // xyz = ellipsoid semi-axes, w = atmosphere height
    vec4 sun_dir;        // xyz = direction to sun, w = intensity
    vec4 sun_color;      // xyz = colour, w = exposure
    vec4 ambient;        // xyz = ambient radiance
    vec4 rayleigh;       // xyz = per-metre Rayleigh, w = Mie coefficient
    vec4 scatter;        // x = Mie g, y = Rayleigh scale height, z = Mie scale height, w = camera altitude
    vec4 ground_albedo;  // xyz
    vec4 ocean_color;    // xyz
    vec4 cloud_global;   // x = ground shadow strength, y = base_min alt, z = top_max alt, w = deck count
    vec4 star_params;    // x = brightness, y = density, z = atmosphere enabled, w = stars enabled
    vec4 misc;           // x = near, y = far, z = time, w = clouds enabled
    vec4 sky_counts;     // x = body count, y = star count, z = surface planet enabled
    vec4 planet_frame;   // xyz = dominant body's north pole, w = surface style
    vec4 cloud_light;    // x = light absorption, y = forward scatter g, z = powder, w = ambient
    vec4 ibl_params;     // x = intensity, y = specular mip count, z = ambient mode
    // Per-deck cloud parameters, one entry per resolved genus deck:
    //   a = base alt, top alt, coverage, density (0 disables the deck);
    //   b = stratiform (0 cellular .. 1 sheet), detail strength, shape scale, detail scale;
    //   c = wind velocity m/s (xyz), noise kind (0 cumuliform, 1 cirriform);
    //   d = anvil spread, weather scale, evolution rate, spare.
    vec4 cloud_deck_a[6];
    vec4 cloud_deck_b[6];
    vec4 cloud_deck_c[6];
    vec4 cloud_deck_d[6];
    // Solar-system bodies, 5 vec4 each:
    //   [5i+0] = direction.xyz, angular radius; [5i+1] = colour.rgb, brightness;
    //   [5i+2] = sun-facing direction.xyz, is-star flag;
    //   [5i+3] = distance (m), mean radius (m);
    //   [5i+4] = north pole.xyz, surface style.
    vec4 bodies[80];
    // Fixed stars, 2 vec4 each: [2i+0] = direction.xyz, brightness; [2i+1] = colour.rgb.
    vec4 sky_stars[128];
} scene;

#define MAX_BODIES 16
#define MAX_STARS 64

layout(set = 0, binding = 1) uniform sampler2D depth_texture;
layout(set = 0, binding = 2) uniform sampler2D hdr_color_texture;
layout(set = 0, binding = 3) uniform sampler3D cloud_shape_texture;   // R = Perlin-Worley, GBA = Worley octaves
layout(set = 0, binding = 4) uniform sampler3D cloud_detail_texture;  // RGB = high-frequency Worley octaves
layout(set = 0, binding = 5) uniform sampler2D cloud_weather_texture; // R = coverage, G = cloud type
layout(set = 0, binding = 6) uniform sampler3D cloud_cirrus_texture;  // R = anisotropic filament base, GBA = Worley octaves
// The sun's cascades. The analytic ground is the surface most of this scene actually
// stands on, so it has to receive the same shadows the meshes standing on it do —
// otherwise an object casts onto nothing and reads as floating over its own terrain.
layout(set = 0, binding = 11) uniform sampler2DShadow shadow_atlas;
layout(set = 0, binding = 12) uniform sampler2D shadow_atlas_depth;
// The Hillaire LUT stack: view-independent optical depth to space, and the
// infinite-order isotropic multiple scattering. Built by atmosphere_lut_pass.
layout(set = 0, binding = 24) uniform sampler2D transmittance_lut;
layout(set = 0, binding = 25) uniform sampler2D multiscatter_lut;
// The per-frame sky-view LUT: the background sky's in-scatter in the camera's local
// frame, so a pixel with no geometry is one fetch instead of a 32-step march. Disabled
// (ibl_params.w == 0) during the IBL cube capture, which renders from other viewpoints.
layout(set = 0, binding = 26) uniform sampler2D sky_view_lut;
// The aerial-perspective froxel volume: the air between the camera and each mesh pixel,
// read as one 3D fetch in the geometry composite instead of a per-pixel march.
layout(set = 0, binding = 27) uniform sampler3D aerial_volume;
// The volumetric-fog froxel volume: rgb = in-scatter (sun radiance folded in), a =
// transmittance, folded over every pixel in the composite. Same addressing as the aerial
// volume, so sample_aerial reads it too.
layout(set = 0, binding = 28) uniform sampler3D fog_volume;

#define CLOUD_MAX_DECKS 6

layout(location = 0) in vec2 v_ndc;

layout(location = 0) out vec4 out_color;
// The analytic ground's direct-sun term, held back from out_color so it can be
// bilateral-blurred before it lands in the scene: rgb = the unshadowed direct
// contribution (already scaled by total_transmittance), a = the raw, noisy cascade
// visibility. Neither carries the ambient/skylight terms, which are stable per-pixel
// (no PCF noise) and go straight into out_color as before. Ground-shadow-resolve.frag
// blurs the alpha and cloud_composite.frag folds rgb * blurred-a back into the scene.
layout(location = 1) out vec4 out_ground_shadow;

const float PI = 3.14159265359;

// Ray vs sphere centred at c with radius r. Returns (t_near, t_far); t_far < 0 means miss.
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

// Ray vs an oblate spheroid centred at c whose polar axis (semi-minor b) points along
// the unit pole; equatorial semi-major is a. Decomposing into axial and radial parts
// avoids building a basis. Returns nearest positive t, or -1.0 on miss.
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

// Outward geodetic normal of the oriented spheroid at surface point p.
vec3 ellipsoid_normal(vec3 p, vec3 c, float a, float b, vec3 pole)
{
    vec3 v = p - c;
    float ax = dot(v, pole);
    vec3 rad = v - pole * ax;
    return normalize(rad / (a * a) + pole * (ax / (b * b)));
}

float hash13(vec3 p)
{
    p = fract(p * 0.1031);
    p += dot(p, p.zyx + 31.32);
    return fract((p.x + p.y) * p.z);
}

float value_noise(vec3 p)
{
    vec3 i = floor(p);
    vec3 f = fract(p);
    f = f * f * (3.0 - 2.0 * f);
    float n000 = hash13(i + vec3(0, 0, 0));
    float n100 = hash13(i + vec3(1, 0, 0));
    float n010 = hash13(i + vec3(0, 1, 0));
    float n110 = hash13(i + vec3(1, 1, 0));
    float n001 = hash13(i + vec3(0, 0, 1));
    float n101 = hash13(i + vec3(1, 0, 1));
    float n011 = hash13(i + vec3(0, 1, 1));
    float n111 = hash13(i + vec3(1, 1, 1));
    float nx00 = mix(n000, n100, f.x);
    float nx10 = mix(n010, n110, f.x);
    float nx01 = mix(n001, n101, f.x);
    float nx11 = mix(n011, n111, f.x);
    float nxy0 = mix(nx00, nx10, f.y);
    float nxy1 = mix(nx01, nx11, f.y);
    return mix(nxy0, nxy1, f.z);
}

float fbm(vec3 p)
{
    float total = 0.0;
    float amplitude = 0.5;
    for (int i = 0; i < 4; ++i)
    {
        total += amplitude * value_noise(p);
        p *= 2.02;
        amplitude *= 0.5;
    }
    return total;
}

// Procedural surface albedo of a body from its style: 0 rocky (cratered brightness
// noise), 1 earthlike (ocean/land mask), 2 banded (latitude bands wobbled by
// turbulence about the body's pole), 3 star (untinted, emissive). `normal` is the
// unit surface direction from the body's centre; primary/secondary are its two tones.
vec3 surface_albedo(float style, vec3 normal, vec3 pole, vec3 primary, vec3 secondary)
{
    if (style > 2.5)
        return primary;
    if (style > 1.5)
    {
        float latitude = dot(normal, pole);
        float swirl = fbm(normal * 4.0);
        float bands = 0.5 + 0.5 * sin(latitude * 28.0 + swirl * 5.0);
        return mix(secondary, primary, bands);
    }
    if (style > 0.5)
    {
        float land = smoothstep(0.48, 0.52, fbm(normal * 3.0));
        return mix(secondary, primary, land);
    }
    float rock = fbm(normal * 8.0);
    return mix(secondary, primary, clamp(rock * 1.6 - 0.2, 0.0, 1.0));
}

// Tilts a shading normal by the tangential gradient of an fbm height field over the
// unit sphere — cheap terrain relief before any mesh exists ("shader first"). Gas
// giants (banded) and the Sun stay smooth.
vec3 relief_normal(vec3 normal, vec3 sphere_dir, float style, float strength)
{
    if (style > 1.5)
        return normal;
    vec3 t1 = normalize(cross(sphere_dir, vec3(0.31, 0.75, 0.59)));
    vec3 t2 = cross(sphere_dir, t1);
    const float eps = 0.004;
    float h0 = fbm(sphere_dir * 180.0);
    float hx = fbm((sphere_dir + t1 * eps) * 180.0);
    float hy = fbm((sphere_dir + t2 * eps) * 180.0);
    return normalize(normal - (t1 * (hx - h0) + t2 * (hy - h0)) * strength);
}

// Rayleigh phase.
float phase_rayleigh(float mu)
{
    return 3.0 / (16.0 * PI) * (1.0 + mu * mu);
}

// Henyey-Greenstein phase for Mie.
float phase_mie(float mu, float g)
{
    float g2 = g * g;
    return 1.0 / (4.0 * PI) * (1.0 - g2) / pow(1.0 + g2 - 2.0 * g * mu, 1.5);
}

// Linear remap of v from [a,b] to [c,d].
float remap(float v, float a, float b, float c, float d)
{
    return c + (v - a) / (b - a) * (d - c);
}

// Vertical density profile across a deck, chosen by its morphology. `stratiform` morphs
// from a rounded cumuliform mound (0) to a flat continuous sheet (1); `anvil` blends in a
// tall convective column that fills nearly the whole étage (cumulonimbus). height01 is 0
// at the deck base, 1 at its top.
float cloud_height_gradient(float height01, float stratiform, float anvil)
{
    float cumuliform = clamp(remap(height01, 0.0, 0.15, 0.0, 1.0), 0.0, 1.0) *
                       clamp(remap(height01, 0.55, 1.0, 1.0, 0.0), 0.0, 1.0);
    float sheet = clamp(remap(height01, 0.0, 0.08, 0.0, 1.0), 0.0, 1.0) *
                  clamp(remap(height01, 0.80, 1.0, 1.0, 0.0), 0.0, 1.0);
    float tower = clamp(remap(height01, 0.0, 0.10, 0.0, 1.0), 0.0, 1.0) *
                  clamp(remap(height01, 0.90, 1.0, 1.0, 0.0), 0.0, 1.0);
    float g = mix(cumuliform, sheet, stratiform);
    return mix(g, tower, anvil);
}

// Modelled density of a single genus deck at a camera-relative world point. Returns 0
// outside the deck's altitude band or where coverage does not reach; height01 is the
// normalized altitude within the deck. The deck index selects its parameters; its noise
// kind picks the isotropic cumuliform base volume or the wind-stretched anisotropic cirrus
// volume; its stratiform axis flattens cellular billows into a sheet; and its anvil term
// both tows the column up to the tropopause and flares the top outward (cumulonimbus).
float cloud_deck_density(int i, vec3 p, bool cheap, out float height01)
{
    height01 = 0.0;
    vec4 la = scene.cloud_deck_a[i];
    float density_scale = la.w;
    if (density_scale <= 0.0) // disabled deck (density packed to zero)
        return 0.0;

    float surface_r = scene.planet_center.w;
    float base_r = surface_r + la.x;
    float top_r = surface_r + la.y;
    vec3 to_p = p - scene.planet_center.xyz;
    float r = length(to_p);
    if (r < base_r || r > top_r)
        return 0.0;
    height01 = (r - base_r) / (top_r - base_r);
    vec3 up = to_p / max(r, 1.0);

    vec4 lb = scene.cloud_deck_b[i];
    vec4 lc = scene.cloud_deck_c[i];
    vec4 ld = scene.cloud_deck_d[i];
    float stratiform = lb.x;
    float shape_scale = lb.z;
    float detail_scale = lb.w;
    bool cirrus = lc.w > 0.5;
    float anvil = ld.x;
    float weather_scale = max(ld.y, 1.0);

    // Advect this deck's field with its own wind over time, and shift each deck into a
    // different region of the finite noise volumes so decks never repeat the same lumps
    // in lockstep with one another.
    vec3 wind = lc.xyz * scene.misc.z;
    vec3 deck_offset = vec3(float(i) * 6130.0, float(i) * 2710.0, float(i) * 9770.0);
    vec3 sp = p + wind + deck_offset;

    // Coverage from a weather map with realistic clustering. A broad field sets where the
    // weather systems sit (large clear vs cloudy regions, tens of km); a finer field adds the
    // clumping within them (clusters and gaps, a few km), sampled stretched along this deck's
    // wind so the clumps line up into wind-parallel cloud streets. A broad self-warp at a
    // wavelength far larger than the map tile hides the tile period. The fine scale and the
    // streets are skipped on the cheap path (light march / probe) — coverage there needs only
    // the broad field, which keeps the shadow march cheap.
    vec3 t1 = normalize(cross(up, vec3(0.1, 0.0, 1.0)));
    vec3 t2 = cross(up, t1);
    vec2 wpln = vec2(dot(sp, t1), dot(sp, t2));
    vec2 wuv = wpln / weather_scale;
    vec2 wwarp = texture(cloud_weather_texture, wuv * 0.15 + vec2(0.37)).rg - 0.5;
    wuv += wwarp * 0.9;
    float weather = texture(cloud_weather_texture, wuv).r;
    if (!cheap)
    {
        vec2 wind_dir = vec2(dot(lc.xyz, t1), dot(lc.xyz, t2));
        float wlen = length(wind_dir);
        vec2 along = wlen > 1e-3 ? wind_dir / wlen : vec2(1.0, 0.0);
        vec2 across = vec2(-along.y, along.x);
        vec2 street_uv =
            vec2(dot(wpln, along) * 0.25, dot(wpln, across)) / (weather_scale * 0.22);
        float weather_small = texture(cloud_weather_texture, street_uv + vec2(0.13)).r;
        weather = weather * 0.7 + weather_small * 0.3;
    }
    // Center the field on the deck's authored coverage but let it swing to zero, so the sky
    // breaks into distinct clouds with clear gaps rather than an even, uniform carpet.
    float coverage = clamp(la.z + (weather - 0.5) * 1.35, 0.0, 1.0);
    // Anvil: the deep-convective top spreads horizontally, so raise the effective coverage
    // toward the tropopause — the tower blooms into a broad cap instead of a thin spike.
    coverage = clamp(coverage + anvil * smoothstep(0.55, 1.0, height01) * 0.55, 0.0, 1.0);

    // Domain warp is the single biggest cure for visible repetition. Two octaves at
    // incommensurate wavelengths, each much larger than the shape tile so the warp itself
    // never reads as a pattern: a broad, high-amplitude fold bends the whole lattice off
    // its grid, and a medium fold breaks the fold's own regularity. Together they scatter
    // the noise's period so no two lumps land at the tile spacing the eye latches onto.
    vec3 warp_broad = texture(cloud_shape_texture, sp / (shape_scale * 8.0) + vec3(0.19)).rgb - 0.5;
    vec3 warp_med = texture(cloud_detail_texture, sp / (shape_scale * 2.7)).rgb - 0.5;
    vec3 wp = sp + warp_broad * shape_scale * 0.9 + warp_med * shape_scale * 0.35;

    // Dual-scale base sample at a non-integer ratio: the two tile periods beat against each
    // other, so the combined field only repeats over an enormous distance. The `cheap` path
    // (light march, shadow, empty-space probing) takes a single fetch — the second scale and
    // the detail erosion below only shape the silhouette the eye sees along the view ray, not
    // the optical depth toward the sun — which is the main lever on the volumetric cost.
    vec4 base;
    if (cirrus || cheap)
    {
        base = cirrus ? texture(cloud_cirrus_texture, wp / shape_scale)
                      : texture(cloud_shape_texture, wp / shape_scale);
    }
    else
    {
        vec4 base0 = texture(cloud_shape_texture, wp / shape_scale);
        vec4 base1 = texture(cloud_shape_texture, wp / (shape_scale * 2.17) + vec3(0.37));
        base = mix(base0, base1, 0.4);
    }
    float low_freq = base.g * 0.625 + base.b * 0.25 + base.a * 0.125;
    float base_shape = clamp(remap(base.r, low_freq - 1.0, 1.0, 0.0, 1.0), 0.0, 1.0);
    base_shape *= cloud_height_gradient(height01, stratiform, anvil);

    // Coverage: push the shape below the coverage threshold to zero, renormalize the rest.
    float d = clamp(remap(base_shape, 1.0 - coverage, 1.0, 0.0, 1.0), 0.0, 1.0) * coverage;
    if (d <= 0.0)
        return 0.0;

    if (!cheap)
    {
        // Erode the edges with high-frequency detail — wispy at the top, billowy at the base.
        // A sheet-like deck keeps smoother edges (less erosion) than a cellular one.
        vec4 det = texture(cloud_detail_texture, (sp + wind * 0.3) / detail_scale);
        float detail_fbm = det.r * 0.625 + det.g * 0.25 + det.b * 0.125;
        float erosion = mix(detail_fbm, 1.0 - detail_fbm, clamp(height01 * 3.0, 0.0, 1.0));
        d = clamp(remap(d, erosion * lb.y * (1.0 - stratiform * 0.6), 1.0, 0.0, 1.0), 0.0, 1.0);
    }

    return d * density_scale;
}

// Total cloud density at p, summed over every deck. Because the decks are one physical
// medium, the sum is what the view and light marches integrate — so a light ray from a
// low cumulus sample crosses the mid and high decks above it and is shadowed by them
// with no extra machinery. ambient_h is the normalized altitude across the whole cloud
// shell, used for the height-graded ambient fill in the lighting.
float cloud_density(vec3 p, bool cheap, out float ambient_h)
{
    float total = 0.0;
    for (int i = 0; i < CLOUD_MAX_DECKS; ++i)
    {
        float h;
        total += cloud_deck_density(i, p, cheap, h);
    }
    float surface_r = scene.planet_center.w;
    float base_min = surface_r + scene.cloud_global.y;
    float top_max = surface_r + scene.cloud_global.z;
    float r = length(p - scene.planet_center.xyz);
    ambient_h = clamp((r - base_min) / max(top_max - base_min, 1.0), 0.0, 1.0);
    return total;
}

// Cone-sampled optical depth from p toward the sun across the whole cloud shell. Six
// samples with exponentially growing steps — short near p for crisp local self-shadow,
// coarse far out — each pushed sideways on a widening cone so the depth stands for a
// bundle of directions rather than a single ray. That softens the shadow and lets light
// bleed through thin edges the way real cloud does, and spanning the full shell is what
// lets an upper layer shadow a lower one. Takes the cheap density (no detail erosion /
// dual-scale) since it feeds a single scalar depth.
float cloud_light_march(vec3 p, vec3 sun)
{
    const int LIGHT_STEPS = 5;
    vec3 b1 = normalize(cross(sun, vec3(0.31, 0.86, 0.41)));
    vec3 b2 = cross(sun, b1);
    float shell = max(scene.cloud_global.z - scene.cloud_global.y, 1.0);
    float cone_radius = shell * 0.08;
    float step_len = shell * 0.04; // short first step; grows each iteration
    float depth = 0.0;
    float t = 0.0;
    for (int i = 0; i < LIGHT_STEPS; ++i)
    {
        float cone = float(i) / float(LIGHT_STEPS);
        float a = float(i) * 2.4;
        vec3 offset = (b1 * cos(a) + b2 * sin(a)) * cone * cone_radius;
        float h;
        depth += cloud_density(p + sun * (t + step_len * 0.5) + offset, true, h) * step_len;
        t += step_len;
        step_len *= 2.1;
    }
    return depth;
}

// Sunlight energy reaching a sample, modelled as multiple scattering: three octaves of
// Beer extinction with halving optical depth, scatter weight and phase eccentricity
// (Wrenninge/Hillaire). The first octave is the direct single-scatter term; the dimmer,
// broader octaves fill the deep interior the single term leaves black, so dense cores read
// luminous and self-shadowed instead of flat — the main cue that a cloud has volume.
float cloud_sun_energy(float light_depth, float mu, float g, float extinction_scale)
{
    float energy = 0.0;
    float attenuation = 1.0;
    float scatter = 1.0;
    float eccentricity = g;
    for (int o = 0; o < 4; ++o)
    {
        float beer = exp(-light_depth * extinction_scale * attenuation);
        float lobe = mix(phase_mie(mu, eccentricity), phase_mie(mu, -0.15 * eccentricity), 0.5);
        // Deeper octaves are multiply scattered — their light has lost its direction, so blend
        // the lobe toward isotropic as octaves deepen. This is why a cloud reads white from
        // every angle instead of only a bright rim when you look toward the sun. The 4*PI
        // factor rescales the small raw phase value so a fully-lit sample approaches the sun's
        // radiance (white); the light march's Beer term then greys down the self-shadowed side.
        float ph = mix(lobe, 1.0 / (4.0 * PI), float(o) / 3.0);
        energy += scatter * beer * ph * 4.0 * PI;
        attenuation *= 0.5;
        scatter *= 0.65;
        eccentricity *= 0.5;
    }
    return energy;
}

// Transmittance of direct sunlight down to a ground point through the whole cloud stack,
// so the clouds cast their combined shadow onto the analytic ground. Marches from the
// shell base straight up the sun direction; scaled by the author's shadow strength.
float cloud_ground_shadow(vec3 ground_p, vec3 sun)
{
    if (scene.misc.w <= 0.5 || scene.cloud_global.x <= 0.0)
        return 1.0;
    float surface_r = scene.planet_center.w;
    vec2 hit = ray_sphere(ground_p, sun, scene.planet_center.xyz,
                          surface_r + scene.cloud_global.z);
    if (hit.y <= 0.0)
        return 1.0;
    float t0 = max(hit.x, 0.0);
    float t1 = hit.y;
    const int STEPS = 8;
    float seg = (t1 - t0) / float(STEPS);
    float depth = 0.0;
    for (int i = 0; i < STEPS; ++i)
    {
        float h;
        depth += cloud_density(ground_p + sun * (t0 + seg * (float(i) + 0.5)), true, h) * seg;
    }
    float extinction_scale = scene.cloud_light.x * 0.006;
    float shadow = exp(-depth * extinction_scale);
    return mix(1.0, shadow, clamp(scene.cloud_global.x, 0.0, 1.0));
}

void main()
{
    vec3 ro = vec3(0.0); // camera at origin in camera-relative space
    // Jittered to match the geometry pass, whose projection carries the same offset;
    // without it the sky would be the one surface the temporal resolve cannot
    // antialias, and the star field would shimmer.
    vec2 ndc = v_ndc - temporal.jitter.xy;
    vec3 rd = normalize(scene.cam_forward.xyz + ndc.x * scene.cam_right.xyz +
                        ndc.y * scene.cam_up.xyz);

    vec3 center = scene.planet_center.xyz;
    float surface_radius = scene.planet_center.w;
    float semi_major = scene.planet_radii.x;
    float semi_minor = scene.planet_radii.z;
    vec3 planet_pole = scene.planet_frame.xyz;
    float planet_style = scene.planet_frame.w;
    float atmosphere_radius = surface_radius + scene.planet_radii.w;
    vec3 sun = normalize(scene.sun_dir.xyz);
    vec3 sun_radiance = scene.sun_color.xyz * scene.sun_dir.w;

    // Solar eclipse: how much of the Sun's disk a nearer body (the Moon) covers. The
    // ephemeris computes this once on the CPU (a global sky event, negligible parallax
    // across the far Sun) and packs it into sky_counts.w, so the sky, the analytic ground,
    // and the shaded meshes all dim by the same scalar rather than the sky dimming alone.
    // The Sun's own angular radius is still fetched here — the corona hugs its limb below.
    float sun_eclipse = scene.sky_counts.w;
    float sun_angular_radius = 0.0;
    {
        int all_bodies = int(scene.sky_counts.x);
        for (int i = 0; i < MAX_BODIES; ++i)
        {
            if (i >= all_bodies)
                break;
            if (scene.bodies[i * 5 + 2].w > 0.5) // is_star: the Sun
            {
                sun_angular_radius = scene.bodies[i * 5 + 0].w;
                break;
            }
        }
    }
    // Darken the incoming sunlight as the disk is covered — the whole sky and ground fall
    // toward twilight at totality, the way a real eclipse dusks the landscape. Held just
    // short of full black so the scene never becomes pure void.
    sun_radiance *= (1.0 - 0.92 * sun_eclipse);

    // Geometry depth bounds the march: convert sampled depth to a distance along rd.
    vec2 uv = v_ndc * 0.5 + 0.5;
    float depth = texture(depth_texture, uv).r;
    float geometry_t = 1e30;
    // Reverse-Z: the buffer clears to 0 (infinitely far) and rises toward 1 at the near
    // plane, so a texel carries geometry exactly when it is above 0. The view-z solve
    // reads the projection directly, so it needs no change for reverse-Z.
    if (depth > 0.0)
    {
        float z_view = scene.proj[3][2] / (-depth - scene.proj[2][2]);
        float cos_forward = max(dot(rd, scene.cam_forward.xyz), 1e-3);
        geometry_t = (-z_view) / cos_forward;
    }

    // Ground and atmosphere intersections. The surface planet is switched off in the
    // interplanetary regime (sky_counts.z), where the far-field bodies own the whole view.
    bool surface_enabled = scene.sky_counts.z > 0.5;
    float ground_t = surface_enabled
                         ? ray_ellipsoid(ro, rd, center, semi_major, semi_minor, planet_pole)
                         : -1.0;
    bool ground_hit = ground_t > 0.0 && ground_t < geometry_t;
    vec2 atmo = ray_sphere(ro, rd, center, atmosphere_radius);

    vec3 sky_color = vec3(0.0);
    // No ground this pixel: nothing pending for the resolve/composite passes to add,
    // and a=1 keeps a stray sample at a boundary reading as "unshadowed" rather than
    // pulling in a false dark edge.
    vec4 ground_shadow_out = vec4(0.0, 0.0, 0.0, 1.0);
    float march_end = ground_hit ? ground_t : min(atmo.y, geometry_t);
    float march_start = max(atmo.x, 0.0);

    float rayleigh_h = scene.scatter.y;
    float mie_h = scene.scatter.z;
    float mie_g = scene.scatter.x;
    vec3 rayleigh_coeff = scene.rayleigh.xyz;
    float mie_coeff = scene.rayleigh.w;

    vec3 total_transmittance = vec3(1.0);

    // The spherical medium the LUT stack is parameterized against (mean radius; the
    // ellipsoid ground intersection stays exact below). Mie extinction carries the same
    // 1.1 absorption factor the view path applies, so the LUTs and this march agree.
    AtmosphereMedium medium;
    medium.rayleigh_scattering = rayleigh_coeff;
    medium.mie_scattering = mie_coeff;
    medium.mie_extinction = mie_coeff * 1.1;
    medium.rayleigh_scale_height = rayleigh_h;
    medium.mie_scale_height = mie_h;
    medium.bottom_radius = surface_radius;
    medium.top_radius = atmosphere_radius;

    // A pixel that sees neither the analytic ground nor a mesh is pure background sky;
    // there the whole atmosphere column is the sky-view LUT's one fetch, and the view
    // transmittance to space is the transmittance LUT — no per-pixel march at all.
    bool escapes_to_space = !ground_hit && geometry_t > 1e29;
    bool use_sky_view = scene.ibl_params.w > 0.5 && escapes_to_space;
    // A mesh pixel within froxel range reads its aerial perspective from the volume; the
    // analytic ground and geometry past the volume keep the march (their air can extend
    // hundreds of km, far beyond the froxel's reach).
    bool use_aerial = scene.ibl_params.w > 0.5 && depth > 0.0 && !ground_hit &&
                      geometry_t < AERIAL_MAX_DISTANCE;

    if (scene.star_params.z > 0.5 && atmo.y > 0.0 && march_end > march_start && use_sky_view)
    {
        sky_color = sun_radiance * sample_sky_view(sky_view_lut, center, rd, surface_radius);
        float r_camera = length(center);
        float mu_view = dot(normalize(-center), rd);
        total_transmittance = sample_transmittance(transmittance_lut, medium, r_camera, mu_view);
    }
    else if (scene.star_params.z > 0.5 && atmo.y > 0.0 && march_end > march_start && use_aerial)
    {
        vec4 aerial = sample_aerial(aerial_volume, uv, geometry_t);
        sky_color = sun_radiance * aerial.rgb;
        total_transmittance = vec3(aerial.a);
    }
    else if (scene.star_params.z > 0.5 && atmo.y > 0.0 && march_end > march_start)
    {
        const int STEPS = 32;
        float march_len = march_end - march_start;
        float mu = dot(rd, sun);
        float pr = phase_rayleigh(mu);
        float pm = phase_mie(mu, mie_g);
        vec3 inscatter = vec3(0.0);
        vec2 view_depth = vec2(0.0);
        // Quadratic sample distribution: air density falls off exponentially with
        // altitude, so a horizon ray crosses nearly all its mass in the first stretch out
        // of the camera. Uniform steps over the (possibly thousands-of-km) grazing path
        // skip that dense near-camera air and under-count optical depth, leaving the far
        // ground half-visible instead of veiled — the dark ring at the horizon. Clustering
        // the samples toward the camera resolves the dense layer and lets the ground fade
        // seamlessly into the sky at the limb.
        float prev_t = 0.0;
        for (int i = 0; i < STEPS; ++i)
        {
            float t1 = float(i + 1) / float(STEPS);
            t1 = t1 * t1;
            float seg = (t1 - prev_t) * march_len;
            float mid = march_start + (prev_t + (t1 - prev_t) * 0.5) * march_len;
            prev_t = t1;
            vec3 p = ro + rd * mid;
            vec3 to_center = p - center;
            float r_p = length(to_center);
            float altitude = max(r_p - surface_radius, 0.0);
            float dr = exp(-altitude / rayleigh_h) * seg;
            float dm = exp(-altitude / mie_h) * seg;
            view_depth += vec2(dr, dm);

            // Transmittance from the camera to this sample (the accumulated view path).
            vec3 view_transmittance =
                exp(-(rayleigh_coeff * view_depth.x + vec3(mie_coeff) * 1.1 * view_depth.y));

            // Light scattered out of this segment toward the eye: coefficient × density × seg.
            vec3 rayleigh_scatter = dr * rayleigh_coeff;
            vec3 mie_scatter = dm * vec3(mie_coeff);
            float mu_sun = dot(to_center / max(r_p, 1.0), sun);

            // Multiple scattering from the LUT — it already folds the sun's transmittance
            // into every gather direction, so it fills the shadowed air with the hazy
            // twilight glow the single term cannot reach.
            vec3 ms = sample_multiscatter(multiscatter_lut, medium, r_p, mu_sun);
            vec3 multi = ms * (rayleigh_scatter + mie_scatter);

            // Single scattering: the sun's transmittance to this sample, read from the LUT
            // instead of a per-sample light-ray march, applied only where the sample can
            // actually see the sun past the ellipsoid (planet self-shadow).
            vec3 single = vec3(0.0);
            if (ray_ellipsoid(p, sun, center, semi_major, semi_minor, planet_pole) <= 0.0)
            {
                vec3 sun_transmittance =
                    sample_transmittance(transmittance_lut, medium, r_p, mu_sun);
                single = sun_transmittance * (rayleigh_scatter * pr + mie_scatter * pm);
            }

            inscatter += view_transmittance * (single + multi);
        }
        sky_color = sun_radiance * inscatter;

        total_transmittance = exp(-(rayleigh_coeff * view_depth.x +
                                    vec3(mie_coeff) * 1.1 * view_depth.y));
    }

    // Lit ground where the ray hits the planet: the dominant body's style paints the
    // albedo and fbm relief tilts the normal, so Mars is cratered rust, the Moon is
    // grey rubble, Jupiter's cloud tops are smooth bands — all before any mesh.
    if (ground_hit)
    {
        vec3 hit = ro + rd * ground_t;
        vec3 sphere_dir = normalize(hit - center);
        vec3 normal = ellipsoid_normal(hit, center, semi_major, semi_minor, planet_pole);
        vec3 albedo;
        if (planet_style > 0.5 && planet_style < 1.5)
        {
            // Earth keeps its original ground: the coarse ocean/land mask over the
            // geodetic normal, smooth-shaded, no procedural relief.
            float land = smoothstep(0.48, 0.52, fbm(normal * 3.0));
            albedo = mix(scene.ocean_color.xyz, scene.ground_albedo.xyz, land);
        }
        else
        {
            normal = relief_normal(normal, sphere_dir, planet_style, 8.0);
            albedo = surface_albedo(planet_style, sphere_dir, planet_pole,
                                    scene.ground_albedo.xyz, scene.ocean_color.xyz);
        }
        float n_dot_l = max(dot(normal, sun), 0.0);

        // Hemispherical skylight: the same air that hazes the horizon in the pass above
        // also floods the ground with scattered light from the whole sky dome, not just
        // where the sun directly hits — without it the ground stays lit only by the tiny
        // flat `ambient` while the sky right above it is bright with in-scatter, so the
        // horizon reads as a dark ring cut out of the sky instead of one continuous scene.
        // Tinted by the Rayleigh coefficient (blue) and scaled by its optical depth over
        // one scale height, so it brightens and dims together with the sky itself and
        // stays lit a while after direct sunlight leaves (the sun is still lighting the
        // upper air even after it drops below the local horizon).
        float sky_light = clamp(dot(normal, sun) * 0.5 + 0.6, 0.0, 1.0);
        vec3 skylight_ambient = scene.star_params.z > 0.5
                                     ? rayleigh_coeff * rayleigh_h * 0.5 * sun_radiance * sky_light
                                     : vec3(0.0);

        // Cloud shadow: attenuate only the direct sun term by the cloud stack overhead —
        // the sky-dome skylight and flat ambient still reach the shadowed ground, so it
        // darkens under clouds without going black, the way an overcast ground reads.
        float cloud_shadow = cloud_ground_shadow(hit, sun);
        // The cascades are fitted in the same camera-relative frame this march works in,
        // so the hit point goes straight in.
        float cascade_shadow = sample_sun_shadow(shadow_atlas, shadow_atlas_depth, hit,
                                                 normal, sun,
                                                 dot(scene.cam_forward.xyz, hit), true);
        // The direct sun term is held out of sky_color and handed to the resolve/composite
        // passes instead — see the out_ground_shadow declaration above. Everything that
        // does not carry the PCF's per-pixel noise (ambient, skylight) is unaffected and
        // goes straight into sky_color as before.
        vec3 ground_ambient = albedo * (scene.ambient.xyz + skylight_ambient);
        vec3 ground_direct = albedo * sun_radiance * n_dot_l / PI * cloud_shadow;
        sky_color += ground_ambient * total_transmittance;
        ground_shadow_out = vec4(ground_direct * total_transmittance, cascade_shadow);
    }

    float air = clamp((total_transmittance.r + total_transmittance.g +
                       total_transmittance.b) / 3.0, 0.0, 1.0);

    // Real bright-star catalogue: each star is a small point at its true sky position,
    // faded by the air the ray crossed so it fades into the lit atmosphere near the limb
    // and washes out under the daytime in-scatter added above.
    if (scene.star_params.w > 0.5 && escapes_to_space)
    {
        int star_count = int(scene.sky_counts.y);
        for (int i = 0; i < MAX_STARS; ++i)
        {
            if (i >= star_count)
                break;
            vec3 star_dir = scene.sky_stars[i * 2 + 0].xyz;
            float star_brightness = scene.sky_stars[i * 2 + 0].w;
            vec3 star_color = scene.sky_stars[i * 2 + 1].xyz;
            float cos_d = dot(rd, star_dir);
            if (cos_d <= 0.0)
                continue;
            float angle = acos(clamp(cos_d, -1.0, 1.0));
            float core = 1.0 - smoothstep(0.0, 0.0009, angle);
            if (core > 0.0)
            {
                float twinkle = 0.85 + 0.15 * sin(scene.misc.z * 3.0 + float(i) * 12.9);
                float mag = pow(star_brightness, 0.4) * twinkle;
                sky_color += star_color * core * mag * scene.star_params.x * air * 2.0;
            }
        }
    }

    // Solar-system bodies, depth-ordered so the nearest body covering a ray occludes the
    // rest — the Moon crossing in front of the Sun is a solar eclipse, in front of a
    // planet an occultation. Each body picks its own LOD regime: a body large in the view
    // (you are near it) is a true ray-traced sphere, precise because its camera-relative
    // centre is small; a distant body is a phase-lit disk in direction space, which stays
    // precise where a true sphere at 1e12 m would collapse in single precision. The Sun
    // (is-star flag) is emissive; every other body is lit by its sun-facing direction.
    if (escapes_to_space)
    {
        int body_count = int(scene.sky_counts.x);
        float nearest_distance = 1e30; // metres; smallest wins the pixel
        vec3 body_result = vec3(0.0);
        float body_coverage = 0.0;
        for (int i = 0; i < MAX_BODIES; ++i)
        {
            if (i >= body_count)
                break;
            vec4 b0 = scene.bodies[i * 5 + 0];
            vec4 b1 = scene.bodies[i * 5 + 1];
            vec4 b2 = scene.bodies[i * 5 + 2];
            vec4 b3 = scene.bodies[i * 5 + 3];
            vec4 b4 = scene.bodies[i * 5 + 4];
            vec3 body_dir = b0.xyz;
            float angular_radius = b0.w;
            vec3 body_color = b1.rgb;
            float body_brightness = b1.w;
            vec3 body_sun = b2.xyz;
            bool is_star = b2.w > 0.5;
            float body_distance = b3.x;
            float body_radius = b3.y;
            vec3 body_pole = b4.xyz;
            float body_style = b4.w;

            float coverage = 0.0;
            float limb_f = 0.0; // 0 centre .. 1 limb, for the Sun's limb darkening below
            vec3 normal = body_dir; // surface normal at the covered point (toward observer)
            if (angular_radius > 0.02)
            {
                // Near regime: intersect the real sphere at its camera-relative centre.
                vec3 center = body_dir * body_distance;
                vec2 hit = ray_sphere(vec3(0.0), rd, center, body_radius);
                float t = hit.x > 0.0 ? hit.x : hit.y;
                if (hit.y > 0.0 && t > 0.0)
                {
                    coverage = 1.0;
                    normal = normalize(rd * t - center);
                    body_distance = t; // order by the actual hit depth up close
                    limb_f = clamp(1.0 - dot(normal, -rd), 0.0, 1.0);
                }
            }
            else
            {
                // Far regime: a disk in direction space, clamped to a visible minimum size.
                float draw_radius = max(angular_radius, 0.00055);
                float cos_d = dot(rd, body_dir);
                if (cos_d > 0.0)
                {
                    float angle = acos(clamp(cos_d, -1.0, 1.0));
                    if (angle <= draw_radius)
                    {
                        float f = angle / draw_radius; // 0 centre .. 1 limb
                        coverage = 1.0 - smoothstep(0.85, 1.0, f);
                        vec3 tangent = normalize(rd - body_dir * cos_d);
                        normal = tangent * f - body_dir * sqrt(max(0.0, 1.0 - f * f));
                        limb_f = f;
                    }
                }
            }

            if (coverage > 0.0 && body_distance < nearest_distance)
            {
                nearest_distance = body_distance;
                vec3 color;
                if (is_star)
                {
                    // Real limb darkening: the disk centre looks straight into the hot,
                    // dense photosphere; the limb grazes cooler, more tenuous outer layers,
                    // so it dims and reddens toward the edge instead of the disk reading as
                    // a flat, uniformly white cutout. The standard u=0.6 linear law
                    // (I/I0 = 1 - u(1 - cos_theta)) with cos_theta ~ sqrt(1 - limb_f^2),
                    // then warmed toward the edge.
                    float mu_disk = sqrt(max(0.0, 1.0 - limb_f * limb_f));
                    float darkening = 1.0 - 0.75 * (1.0 - mu_disk);
                    vec3 limb_tint = mix(vec3(1.0, 0.95, 0.88), vec3(1.0, 0.7, 0.42),
                                         pow(limb_f, 1.5));
                    color = body_color * sun_radiance * darkening * limb_tint;
                }
                else
                {
                    // The body's style patterns its disk: bands about its own pole for
                    // the giants, crater noise for rocky worlds, ocean/land for Earth —
                    // the same procedural surface the ground regime shades up close.
                    vec3 pattern = surface_albedo(body_style, normal, body_pole,
                                                  vec3(1.0), vec3(0.55));
                    float lambert = max(dot(normal, body_sun), 0.0);
                    color = body_color * pattern * sun_radiance *
                            (lambert * body_brightness + 0.015);
                }
                body_result = color;
                body_coverage = coverage;
            }
        }
        if (body_coverage > 0.0)
            sky_color += body_result * body_coverage * total_transmittance;

        // Warm solar aureole: a soft halo around the Sun in angular space so the disk
        // reads as a radiant star instead of a hard white cutout — a tight inner glow
        // plus a broad faint corona, reddened by the same air the disk shines through
        // (so it turns orange as the Sun nears the horizon). Scale-independent (radians),
        // so it looks right whether the Sun is a distant point or looming up close.
        float sun_angle = acos(clamp(dot(rd, sun), -1.0, 1.0));
        float aureole = exp(-sun_angle * 55.0) * 0.5 + exp(-sun_angle * 7.0) * 0.02;
        vec3 aureole_color = mix(vec3(1.0, 0.85, 0.65), vec3(1.0, 0.55, 0.3),
                                 clamp(1.0 - sun.y, 0.0, 1.0));
        // The halo fades out as the disk is covered (on top of the already-dimmed
        // sun_radiance) so the dark Moon reads instead of blazing white.
        sky_color += aureole_color * aureole * sun_radiance * total_transmittance *
                     (1.0 - sun_eclipse);

        // Corona: the pearly outer atmosphere seen only at totality — a soft ring just
        // beyond the Sun's limb that appears as the disk is covered and fades outward.
        // Uses the true solar angular radius so it hugs the eclipsed edge, and is not
        // dimmed by the eclipse (it is what the covered disk reveals).
        if (sun_eclipse > 0.2 && sun_angular_radius > 0.0)
        {
            float edge = sun_angle / sun_angular_radius; // 1 at the limb
            float ring = exp(-max(edge - 1.0, 0.0) * 3.0) * smoothstep(0.6, 1.0, edge);
            float totality = smoothstep(0.2, 0.95, sun_eclipse);
            sky_color += vec3(1.0, 0.98, 0.95) * ring * totality * 0.6 *
                         scene.sun_color.xyz * total_transmittance;
        }
    }

    // Clouds are drawn in a separate half-resolution pass (cloud.frag) and composited over
    // this sky in the tonemap pass, so the expensive volumetric march runs at a quarter of
    // the pixels while everything above (sun disk, stars, planet relief) stays sharp. The
    // cloud density helpers remain in this file only for cloud_ground_shadow, which the
    // ground branch above still calls to cast the deck stack's shadow onto the terrain.

    // Composite: over background the sky is the whole pixel; over geometry the opaque
    // colour is attenuated by the air in front of it and the in-scatter is added
    // (aerial perspective).
    vec3 scene_color = texture(hdr_color_texture, uv).rgb;
    if (depth > 0.0) // reverse-Z: nonzero depth means opaque geometry is here
        out_color = vec4(scene_color * total_transmittance + sky_color, 1.0);
    else
        out_color = vec4(sky_color, 1.0);

    // Volumetric fog folds over everything: the nearest visible surface's distance picks
    // the froxel depth, and the fog's own transmittance attenuates the scene while its
    // in-scatter (sun radiance already folded in) is added. A no-op when fog is disabled
    // (the volume is all transmittance 1, zero in-scatter). Skipped during the IBL cube
    // capture, whose viewpoints do not match the camera-frustum-aligned volume.
    if (scene.ibl_params.w > 0.5)
    {
        float scene_distance = ground_hit ? ground_t
                                          : (depth > 0.0 ? geometry_t : AERIAL_MAX_DISTANCE);
        vec4 fog = sample_aerial(fog_volume, uv, scene_distance);
        out_color.rgb = out_color.rgb * fog.a + fog.rgb;
        // The analytic ground's held-back direct term is attenuated by the same fog so the
        // sunlit ground under fog dims with the rest of the scene, not through it.
        ground_shadow_out.rgb *= fog.a;
    }
    out_ground_shadow = ground_shadow_out;
}
