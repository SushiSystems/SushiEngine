#version 450

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
//   * adds a hashed star field that the atmosphere's optical depth fades out — so as
//     the camera climbs out of the air, the blue sky thins into black space and stars.
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
    vec4 cloud_params;   // x = base alt, y = top alt, z = coverage, w = density
    vec4 star_params;    // x = brightness, y = density, z = atmosphere enabled, w = stars enabled
    vec4 misc;           // x = near, y = far, z = time, w = clouds enabled
} scene;

layout(set = 0, binding = 1) uniform sampler2D depth_texture;
layout(set = 0, binding = 2) uniform sampler2D hdr_color_texture;

layout(location = 0) in vec2 v_ndc;

layout(location = 0) out vec4 out_color;

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

// Ray vs axis-aligned ellipsoid centred at c with semi-axes radii, by scaling to a
// unit sphere. Returns nearest positive t, or -1.0 on miss.
float ray_ellipsoid(vec3 ro, vec3 rd, vec3 c, vec3 radii)
{
    vec3 o = (ro - c) / radii;
    vec3 d = rd / radii;
    float a = dot(d, d);
    float b = dot(o, d);
    float cc = dot(o, o) - 1.0;
    float h = b * b - a * cc;
    if (h < 0.0)
        return -1.0;
    h = sqrt(h);
    float t = (-b - h) / a;
    if (t < 0.0)
        t = (-b + h) / a;
    return t;
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

// Optical depth of the two densities along a ray segment from ro over length seg_len,
// packed as (rayleigh, mie). Marches a few steps; density falls off exponentially with
// altitude above the surface (radius R).
vec2 optical_depth(vec3 ro, vec3 rd, float seg_len, vec3 center, float surface_radius,
                   float rayleigh_h, float mie_h)
{
    const int STEPS = 6;
    float step_len = seg_len / float(STEPS);
    vec2 depth = vec2(0.0);
    for (int i = 0; i < STEPS; ++i)
    {
        vec3 p = ro + rd * (step_len * (float(i) + 0.5));
        float altitude = length(p - center) - surface_radius;
        altitude = max(altitude, 0.0);
        depth.x += exp(-altitude / rayleigh_h) * step_len;
        depth.y += exp(-altitude / mie_h) * step_len;
    }
    return depth;
}

void main()
{
    vec3 ro = vec3(0.0); // camera at origin in camera-relative space
    vec3 rd = normalize(scene.cam_forward.xyz + v_ndc.x * scene.cam_right.xyz +
                        v_ndc.y * scene.cam_up.xyz);

    vec3 center = scene.planet_center.xyz;
    float surface_radius = scene.planet_center.w;
    vec3 radii = scene.planet_radii.xyz;
    float atmosphere_radius = surface_radius + scene.planet_radii.w;
    vec3 sun = normalize(scene.sun_dir.xyz);
    vec3 sun_radiance = scene.sun_color.xyz * scene.sun_dir.w;

    // Geometry depth bounds the march: convert sampled depth to a distance along rd.
    vec2 uv = v_ndc * 0.5 + 0.5;
    float depth = texture(depth_texture, uv).r;
    float geometry_t = 1e30;
    if (depth < 1.0)
    {
        float z_view = scene.proj[3][2] / (-depth - scene.proj[2][2]);
        float cos_forward = max(dot(rd, scene.cam_forward.xyz), 1e-3);
        geometry_t = (-z_view) / cos_forward;
    }

    // Ground and atmosphere intersections.
    float ground_t = ray_ellipsoid(ro, rd, center, radii);
    bool ground_hit = ground_t > 0.0 && ground_t < geometry_t;
    vec2 atmo = ray_sphere(ro, rd, center, atmosphere_radius);

    vec3 sky_color = vec3(0.0);
    float march_end = ground_hit ? ground_t : min(atmo.y, geometry_t);
    float march_start = max(atmo.x, 0.0);

    float rayleigh_h = scene.scatter.y;
    float mie_h = scene.scatter.z;
    float mie_g = scene.scatter.x;
    vec3 rayleigh_coeff = scene.rayleigh.xyz;
    float mie_coeff = scene.rayleigh.w;

    vec3 total_transmittance = vec3(1.0);

    if (scene.star_params.z > 0.5 && atmo.y > 0.0 && march_end > march_start)
    {
        const int STEPS = 16;
        float seg = (march_end - march_start) / float(STEPS);
        float mu = dot(rd, sun);
        float pr = phase_rayleigh(mu);
        float pm = phase_mie(mu, mie_g);
        vec3 rayleigh_sum = vec3(0.0);
        vec3 mie_sum = vec3(0.0);
        vec2 view_depth = vec2(0.0);
        for (int i = 0; i < STEPS; ++i)
        {
            vec3 p = ro + rd * (march_start + seg * (float(i) + 0.5));
            float altitude = max(length(p - center) - surface_radius, 0.0);
            float dr = exp(-altitude / rayleigh_h) * seg;
            float dm = exp(-altitude / mie_h) * seg;
            view_depth += vec2(dr, dm);

            // Light ray from this sample toward the sun through the atmosphere.
            vec2 light_hit = ray_sphere(p, sun, center, atmosphere_radius);
            float ground_shadow = ray_ellipsoid(p, sun, center, radii);
            if (ground_shadow > 0.0)
                continue; // sample is in the planet's shadow
            vec2 light_depth = optical_depth(p, sun, max(light_hit.y, 0.0), center,
                                             surface_radius, rayleigh_h, mie_h);

            vec3 tau = rayleigh_coeff * (view_depth.x + light_depth.x) +
                       vec3(mie_coeff) * 1.1 * (view_depth.y + light_depth.y);
            vec3 attenuation = exp(-tau);
            rayleigh_sum += attenuation * dr;
            mie_sum += attenuation * dm;
        }
        sky_color = sun_radiance * (rayleigh_sum * rayleigh_coeff * pr +
                                    mie_sum * mie_coeff * pm);
        total_transmittance = exp(-(rayleigh_coeff * view_depth.x +
                                    vec3(mie_coeff) * 1.1 * view_depth.y));
    }

    // Lit ground where the ray hits the planet.
    if (ground_hit)
    {
        vec3 hit = ro + rd * ground_t;
        vec3 normal = normalize((hit - center) / (radii * radii));
        float n_dot_l = max(dot(normal, sun), 0.0);
        // Ocean vs land by a coarse procedural mask over the surface direction.
        float land = smoothstep(0.48, 0.52, fbm(normal * 3.0));
        vec3 albedo = mix(scene.ocean_color.xyz, scene.ground_albedo.xyz, land);
        vec3 ground = albedo * (sun_radiance * n_dot_l / PI + scene.ambient.xyz);
        sky_color += ground * total_transmittance;
    }

    // Star field for rays that escape into space, faded by how much air they crossed.
    if (scene.star_params.w > 0.5 && !ground_hit && geometry_t > 1e29)
    {
        vec3 dir = rd;
        vec3 cell = floor(dir * 700.0);
        float star = hash13(cell);
        float visible = step(1.0 - scene.star_params.y * 0.06, star);
        if (visible > 0.5)
        {
            float twinkle = 0.6 + 0.4 * sin(scene.misc.z * 3.0 + star * 40.0);
            float brightness = pow(hash13(cell + 3.7), 6.0) * scene.star_params.x * twinkle;
            float air = clamp((total_transmittance.r + total_transmittance.g +
                               total_transmittance.b) / 3.0, 0.0, 1.0);
            sky_color += vec3(brightness) * 8.0 * air;
        }
    }

    // Sun disk when looking near the sun through the atmosphere or from space.
    if (!ground_hit && geometry_t > 1e29)
    {
        float mu = dot(rd, sun);
        float disk = smoothstep(0.9995, 0.9998, mu);
        sky_color += sun_radiance * disk * total_transmittance;
    }

    // Cloud layer: march the spherical shell between base and top altitude, lit by sun.
    if (scene.misc.w > 0.5 && !ground_hit)
    {
        float base_r = surface_radius + scene.cloud_params.x;
        float top_r = surface_radius + scene.cloud_params.y;
        vec2 inner = ray_sphere(ro, rd, center, base_r);
        vec2 outer = ray_sphere(ro, rd, center, top_r);
        float t0 = max(max(inner.y, 0.0), max(outer.x, 0.0));
        float t1 = min(outer.y, geometry_t);
        if (outer.y > 0.0 && t1 > t0)
        {
            const int STEPS = 12;
            float seg = (t1 - t0) / float(STEPS);
            float coverage = scene.cloud_params.z;
            float transmittance = 1.0;
            vec3 cloud_light = vec3(0.0);
            for (int i = 0; i < STEPS; ++i)
            {
                vec3 p = ro + rd * (t0 + seg * (float(i) + 0.5));
                vec3 sp = (p - center) / surface_radius * 40.0 + scene.misc.z * 0.02;
                float d = fbm(sp) - (1.0 - coverage);
                float density = clamp(d, 0.0, 1.0) * scene.cloud_params.w;
                if (density > 0.001)
                {
                    float light = clamp(dot(normalize(p - center), sun) * 0.5 + 0.5, 0.0, 1.0);
                    vec3 shade = sun_radiance * (0.15 + 0.85 * light) * 0.02;
                    float dt = density * seg * 0.0025;
                    cloud_light += transmittance * shade * dt;
                    transmittance *= exp(-dt);
                }
            }
            sky_color = sky_color * transmittance + cloud_light;
        }
    }

    // Composite: over background the sky is the whole pixel; over geometry the opaque
    // colour is attenuated by the air in front of it and the in-scatter is added
    // (aerial perspective).
    vec3 scene_color = texture(hdr_color_texture, uv).rgb;
    if (depth < 1.0)
        out_color = vec4(scene_color * total_transmittance + sky_color, 1.0);
    else
        out_color = vec4(sky_color, 1.0);
}
