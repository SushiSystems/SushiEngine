#version 450

// Editor-only reference grid, in two regimes the camera's altitude crossfades between.
//
// Near a planet the grid is its sea level: the pass ray-casts the same reference
// ellipsoid the analytic ground is drawn against, using the same CPU-formed
// double-precision terms, so the lines lie exactly on the ocean surface and stop at the
// true limb instead of running to an infinite flat horizon. Out in space the grid becomes
// the ecliptic plane pinned at the Sun's centre, so the planets ride on it.
//
// Both regimes share one analytic lattice: the line coverage comes from the screen-space
// derivative of the surface position, so the lines are anti-aliased and hold perfectly
// still under camera motion, and the cell size follows a base-10 ladder keyed off the
// on-screen pixel footprint, with the two bracketing decades cross-faded so nothing pops
// as you zoom. That one ladder spans a metre to an astronomical unit unchanged.
//
// It composites in-shader (there is no hardware alpha blend in this engine) over the HDR
// post colour, and is occluded by scene geometry by comparing against the depth buffer.
// Registered only when the editor asks for it; a shipped runtime never runs this pass.

// The full scene block: the sea-level regime reads the same ellipsoid and precision
// terms the sky pass's ground does, and those are the block's last fields.
layout(set = 0, binding = 0) uniform SceneBlock
{
    mat4 view;
    mat4 proj;
    vec4 cam_forward; // xyz = unit forward, w = camera world pos x
    vec4 cam_right;   // xyz = right * tan(fovx/2), w = camera world pos y
    vec4 cam_up;      // xyz = up * tan(fovy/2), w = camera world pos z
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
    vec4 planet_frame; // xyz = dominant body's north pole, w = surface style
    vec4 cloud_light;
    vec4 ibl_params;
    vec4 cloud_deck_a[6];
    vec4 cloud_deck_b[6];
    vec4 cloud_deck_c[6];
    vec4 cloud_deck_d[6];
    vec4 bodies[80];
    vec4 sky_stars[128];
    vec4 planet_ring;
    vec4 planet_precision; // xyz = scaled centre gradient, w = |M c|^2 - 1
} scene;

layout(set = 0, binding = 1) uniform sampler2D color_texture; // HDR post colour to composite over
layout(set = 0, binding = 2) uniform sampler2D depth_texture; // reverse-Z scene depth

// The interplanetary frame, which the scene block does not carry.
layout(push_constant) uniform GridBlock
{
    vec4 sun_position;       // xyz = Sun centre relative to the camera, w = space regime weight
    vec4 ecliptic_normal;    // xyz = unit ecliptic pole, scene frame
    vec4 ecliptic_reference; // xyz = unit vernal-equinox direction in the plane
} grid;

layout(location = 0) in vec2 v_ndc;

layout(location = 0) out vec4 out_color;

// One grid level's coverage: ~1px-wide AA lines on a `cell`-metre lattice. `dP` is the
// world-space size of one pixel, so dividing by the cell gives the derivative in cell
// units and the line stays one pixel wide at every distance.
float grid_level(vec2 p, vec2 dP, float cell)
{
    vec2 g = p / cell;
    vec2 d = max(dP / cell, vec2(1e-6));
    vec2 a = abs(fract(g - 0.5) - 0.5) / d;
    return 1.0 - clamp(min(a.x, a.y), 0.0, 1.0);
}

// Ray vs the oriented reference ellipsoid, camera-origin form. A transcription of
// sky.frag's ray_ellipsoid restricted to ro == 0, which is the only case this pass needs:
// the whole large-minus-large part of the quadratic is then exactly planet_precision.w,
// formed on the CPU in double, so the grid lands on the same surface as the ground and
// does not shimmer against it. Returns the nearest positive t, or -1.0 on a miss.
//
// The roots are taken in the numerically stable form rather than the textbook one. Standing
// near the surface qc is a near-zero difference of planet-scale terms while qb is not, so
// `-qb - sqrt(qb*qb - qa*qc)` cancels catastrophically and the surviving float32 bits of t
// quantise into concentric bands around the camera — invisible in the smoothly shaded
// ground, glaring in a high-frequency lattice drawn on it. Forming the shared factor with
// the sign of qb and recovering the near root as qc/factor keeps every step well
// conditioned; the two roots are then Vieta's pair, not two independent subtractions.
float ray_surface(vec3 rd, float a, float b, vec3 pole)
{
    float inv_a2 = 1.0 / (a * a);
    float inv_b2 = 1.0 / (b * b);
    float d_ax = dot(rd, pole);
    vec3 d_rad = rd - pole * d_ax;
    float qa = dot(d_rad, d_rad) * inv_a2 + d_ax * d_ax * inv_b2;
    float qb = -dot(rd, scene.planet_precision.xyz);
    float qc = scene.planet_precision.w;
    float disc = qb * qb - qa * qc;
    if (disc < 0.0 || qa <= 0.0)
        return -1.0;
    float factor = -(qb + (qb >= 0.0 ? sqrt(disc) : -sqrt(disc)));
    if (factor == 0.0)
        return -1.0;
    float t_far = factor / qa;
    float t_near = qc / factor;
    float lo = min(t_near, t_far);
    float hi = max(t_near, t_far);
    if (lo > 0.0)
        return lo;
    return hi > 0.0 ? hi : -1.0;
}

// Outward geodetic normal at a surface point, camera-relative. The centre term the CPU
// formed in double is subtracted from the small per-fragment term, which is what keeps
// the grazing fade from snapping as the camera moves.
vec3 surface_normal(vec3 p, float a, float b, vec3 pole)
{
    float p_ax = dot(p, pole);
    vec3 p_rad = p - pole * p_ax;
    return normalize((p_rad / (a * a) + pole * (p_ax / (b * b))) - scene.planet_precision.xyz);
}

// The shared lattice: a base-10 ladder picking the decade whose cell is ~12px on screen,
// cross-fading the finer decade into the coarser one so zooming never pops. `axis` is the
// per-component distance to the two origin lines, in the same units as `p`.
vec3 lattice(vec2 p, out float coverage)
{
    vec2 dP = fwidth(p);
    float footprint = max(dP.x, dP.y);
    float level = log2(max(footprint, 1e-8) * 12.0) / log2(10.0);
    float fine_cell = pow(10.0, floor(level));
    float blend = fract(level);
    float fine = grid_level(p, dP, fine_cell);
    float coarse = grid_level(p, dP, fine_cell * 10.0);
    float lines = max(fine * (1.0 - blend), coarse);

    // The two origin axes: the first reads red, the second blue.
    float axis_u = 1.0 - clamp(abs(p.y) / max(dP.y, 1e-6), 0.0, 1.0);
    float axis_v = 1.0 - clamp(abs(p.x) / max(dP.x, 1e-6), 0.0, 1.0);

    vec3 line_color = vec3(0.42, 0.45, 0.52);
    line_color = mix(line_color, vec3(0.90, 0.27, 0.27), axis_u);
    line_color = mix(line_color, vec3(0.27, 0.45, 0.95), axis_v);
    coverage = max(lines, max(axis_u, axis_v));
    return line_color;
}

// Occlusion by scene geometry: project the camera-relative hit and compare against the
// stored reverse-Z depth, where a larger stored value is nearer.
float visible(vec3 hit, vec2 uv)
{
    vec4 clip = scene.proj * scene.view * vec4(hit, 1.0);
    if (clip.w <= 0.0)
        return 0.0;
    float scene_depth = texture(depth_texture, uv).r;
    return scene_depth > clip.z / clip.w + 1e-6 ? 0.0 : 1.0;
}

void main()
{
    vec2 uv = v_ndc * 0.5 + 0.5;
    vec3 base = texture(color_texture, uv).rgb;

    // Camera-relative frame: the eye is the origin, world axes. The eye's world position
    // rides in the .w lanes of the ray basis.
    vec3 rd = normalize(scene.cam_forward.xyz + v_ndc.x * scene.cam_right.xyz +
                        v_ndc.y * scene.cam_up.xyz);
    vec3 eye = vec3(scene.cam_forward.w, scene.cam_right.w, scene.cam_up.w);

    // The two regimes are evaluated independently and blended by weight, so neither
    // pops in or out: at the crossfade altitude both are on screen at half strength.
    float space_weight = clamp(grid.sun_position.w, 0.0, 1.0);
    vec3 surface_color = vec3(0.0);
    float surface_cover = 0.0;
    vec3 space_color = vec3(0.0);
    float space_cover = 0.0;

    // Sea level: the grid is the planet's own reference surface. The lattice coordinate
    // is the hit's world XZ, which at the scene anchor is the local east/north tangent
    // plane and compresses naturally toward the limb as the surface curves away.
    if (space_weight < 1.0)
    {
        float a = scene.planet_radii.x;
        float b = scene.planet_radii.z;
        vec3 pole = normalize(scene.planet_frame.xyz);
        float t = ray_surface(rd, a, b, pole);
        if (t > 0.0)
        {
            vec3 hit = rd * t;
            vec2 p = hit.xz + eye.xz;
            surface_color = lattice(p, surface_cover);
            // Fade where the surface is seen edge-on, at the limb and along the horizon,
            // where a fixed filter would otherwise smear the lattice into a solid sheet.
            surface_cover *= smoothstep(0.0, 0.12, -dot(rd, surface_normal(hit, a, b, pole)));
            surface_cover *= visible(hit, uv);
        }
    }

    // Interplanetary: the ecliptic plane through the Sun's centre, so the grid's origin
    // is the Sun and the planets sit on it rather than crossing it.
    if (space_weight > 0.0)
    {
        vec3 n = normalize(grid.ecliptic_normal.xyz);
        float denominator = dot(rd, n);
        float numerator = dot(grid.sun_position.xyz, n);
        float t = abs(denominator) > 1e-9 ? numerator / denominator : -1.0;
        if (t > 0.0)
        {
            vec3 hit = rd * t;
            vec3 offset = hit - grid.sun_position.xyz;
            vec3 tangent = normalize(grid.ecliptic_reference.xyz -
                                     n * dot(grid.ecliptic_reference.xyz, n));
            vec3 bitangent = cross(n, tangent);
            vec2 p = vec2(dot(offset, tangent), dot(offset, bitangent));
            space_color = lattice(p, space_cover);
            space_cover *= smoothstep(0.0, 0.12, abs(denominator));
            space_cover *= visible(hit, uv);
            // Cooler than the sea-level lines, so the two regimes stay legible where they
            // overlap during the crossfade.
            space_color = mix(space_color, vec3(0.34, 0.40, 0.58), 0.35);
        }
    }

    float surface_weight = surface_cover * (1.0 - space_weight);
    float space_contribution = space_cover * space_weight;
    float coverage = surface_weight + space_contribution;
    if (coverage <= 0.0)
    {
        out_color = vec4(base, 1.0);
        return;
    }
    vec3 line_color =
        (surface_color * surface_weight + space_color * space_contribution) / coverage;

    out_color = vec4(mix(base, line_color, min(coverage, 1.0) * 0.55), 1.0);
}
