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
} scene;

layout(set = 0, binding = 1) uniform sampler2D depth_texture;
layout(set = 0, binding = 3) uniform sampler3D cloud_shape_texture;
layout(set = 0, binding = 4) uniform sampler3D cloud_detail_texture;
layout(set = 0, binding = 5) uniform sampler2D cloud_weather_texture;
layout(set = 0, binding = 6) uniform sampler3D cloud_cirrus_texture;

#define CLOUD_MAX_DECKS 6

layout(location = 0) in vec2 v_ndc;

layout(location = 0) out vec4 out_color;

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

float hash13(vec3 p)
{
    p = fract(p * 0.1031);
    p += dot(p, p.zyx + 31.32);
    return fract((p.x + p.y) * p.z);
}

float phase_mie(float mu, float g)
{
    float g2 = g * g;
    return 1.0 / (4.0 * PI) * (1.0 - g2) / pow(1.0 + g2 - 2.0 * g * mu, 1.5);
}

float remap(float v, float a, float b, float c, float d)
{
    return c + (v - a) / (b - a) * (d - c);
}

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

float cloud_deck_density(int i, vec3 p, bool cheap, out float height01)
{
    height01 = 0.0;
    vec4 la = scene.cloud_deck_a[i];
    float density_scale = la.w;
    if (density_scale <= 0.0)
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

    vec3 wind = lc.xyz * scene.misc.z;
    vec3 deck_offset = vec3(float(i) * 6130.0, float(i) * 2710.0, float(i) * 9770.0);
    vec3 sp = p + wind + deck_offset;

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
    float coverage = clamp(la.z + (weather - 0.5) * 1.35, 0.0, 1.0);
    coverage = clamp(coverage + anvil * smoothstep(0.55, 1.0, height01) * 0.55, 0.0, 1.0);

    vec3 warp_broad = texture(cloud_shape_texture, sp / (shape_scale * 8.0) + vec3(0.19)).rgb - 0.5;
    vec3 warp_med = texture(cloud_detail_texture, sp / (shape_scale * 2.7)).rgb - 0.5;
    vec3 wp = sp + warp_broad * shape_scale * 0.9 + warp_med * shape_scale * 0.35;

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

    float d = clamp(remap(base_shape, 1.0 - coverage, 1.0, 0.0, 1.0), 0.0, 1.0) * coverage;
    if (d <= 0.0)
        return 0.0;

    if (!cheap)
    {
        vec4 det = texture(cloud_detail_texture, (sp + wind * 0.3) / detail_scale);
        float detail_fbm = det.r * 0.625 + det.g * 0.25 + det.b * 0.125;
        float erosion = mix(detail_fbm, 1.0 - detail_fbm, clamp(height01 * 3.0, 0.0, 1.0));
        d = clamp(remap(d, erosion * lb.y * (1.0 - stratiform * 0.6), 1.0, 0.0, 1.0), 0.0, 1.0);
    }

    return d * density_scale;
}

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

float cloud_light_march(vec3 p, vec3 sun)
{
    const int LIGHT_STEPS = 5;
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
        float h;
        depth += cloud_density(p + sun * (t + step_len * 0.5) + offset, true, h) * step_len;
        t += step_len;
        step_len *= 2.1;
    }
    return depth;
}

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
        float ph = mix(lobe, 1.0 / (4.0 * PI), float(o) / 3.0);
        energy += scatter * beer * ph * 4.0 * PI;
        attenuation *= 0.5;
        scatter *= 0.65;
        eccentricity *= 0.5;
    }
    return energy;
}

void main()
{
    out_color = vec4(0.0, 0.0, 0.0, 1.0); // clear sky: no scatter, full transmittance

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
    vec3 sun_radiance = scene.sun_color.xyz * scene.sun_dir.w;

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
    if (t1 <= t0)
        return;

    float extinction_scale = scene.cloud_light.x * 0.006;

    float march_len = t1 - t0;
    march_len = min(march_len, (top_r - base_r) * 14.0);

    float shell_thick = max(top_r - base_r, 1.0);
    float space_factor = clamp(scene.scatter.w / (shell_thick * 6.0), 0.0, 1.0);
    int STEPS = int(mix(96.0, 32.0, space_factor));
    float seg = march_len / float(STEPS);
    float big_seg = seg * 3.0;

    float dither = hash13(vec3(gl_FragCoord.xy, scene.misc.z * 60.0));
    float t = t0 + seg * dither;
    float t_end = t0 + march_len;

    float mu = dot(rd, sun);
    float g = scene.cloud_light.y;

    vec3 ambient_color = scene.ambient.xyz + sun_radiance * 0.02;

    float lod_distance = shell_thick * 4.0;

    float transmittance = 1.0;
    vec3 scattered = vec3(0.0);
    int lit = 0;
    float sun_energy = 0.0;
    for (int i = 0; i < STEPS; ++i)
    {
        if (transmittance < 0.02 || t >= t_end)
            break;
        vec3 p = ro + rd * t;
        float height01;
        float coarse = cloud_density(p, true, height01);
        if (coarse > 0.001)
        {
            float density = t < lod_distance ? cloud_density(p, false, height01) : coarse;
            if (density > 0.001)
            {
                float sigma = density * extinction_scale;
                if ((lit & 1) == 0)
                {
                    float light_depth = cloud_light_march(p, sun);
                    sun_energy = cloud_sun_energy(light_depth, mu, g, extinction_scale);
                }
                ++lit;
                float powder = 1.0 - exp(-density * 2.0);
                powder = mix(1.0, powder, scene.cloud_light.z);

                vec3 sunlight = sun_radiance * sun_energy * powder;
                vec3 ambient = ambient_color * scene.cloud_light.w * mix(0.35, 1.0, height01);
                vec3 luminance = sunlight + ambient;

                float sample_transmit = exp(-sigma * seg);
                scattered += transmittance * luminance * (1.0 - sample_transmit);
                transmittance *= sample_transmit;
            }
            t += seg;
        }
        else
        {
            t += big_seg;
            i += 2;
        }
    }

    out_color = vec4(scattered, transmittance);
}
